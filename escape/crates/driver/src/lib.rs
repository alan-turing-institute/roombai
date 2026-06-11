//! `driver` — continuous control and sensing (SPEC §4.1).
//!
//! Owns the serial port and a single thread that, at a fixed ~20 Hz, reads
//! sensors, integrates odometry, applies reflexes and the watchdog, shapes the
//! planner's setpoint into wheel velocities, and publishes a [`SensorFrame`].
//! The planner holds a [`Driver`] handle and talks to it only through the
//! [`RobotIo`] trait, so the `sim` and `replay` back-ends are drop-in.
//!
//! Layering:
//! - [`oi`] — raw OI byte transport (the only bytes-aware code).
//! - [`kinematics`], [`odometry`], [`reflex`] — pure, fully unit-tested.
//! - [`control::ControlCore`] — the per-tick decision, pure and deterministic.
//! - this module — threading, timing, serial wiring, and startup probing.
//!
//! v1 uses QUERY_LIST polling for the sensor pump (the SPEC fallback), which
//! interleaves naturally with the per-tick DRIVE_DIRECT and is already proven on
//! the 770. OI stream mode (148) is a latency optimization for later.

pub mod config;
pub mod control;
pub mod kinematics;
pub mod odometry;
pub mod oi;
pub mod reflex;

use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use escape_core::{RobotIo, SensorFrame, Twist};
use roomba_pilot::sensors;

pub use config::Config;
pub use control::{ControlCore, RawSensors};
pub use odometry::OdomSource;

/// Packets polled every tick regardless of odometry source (all S1-confirmed):
/// bumps/wheel-drops, battery, OI mode, light bumps. (Cliff sensing is
/// intentionally not wired up yet.)
const BASE_IDS: &[u8] = &[
    sensors::BUMPS_AND_WHEEL_DROPS,
    sensors::BATTERY_CHARGE,
    sensors::OI_MODE,
    46,
    47,
    48,
    49,
    50,
    51,
];

struct Shared {
    /// Latest commanded twist and when it was set (for the watchdog).
    setpoint: Mutex<(Twist, Instant)>,
    /// Fan-out targets for published sensor frames.
    subscribers: Mutex<Vec<Sender<SensorFrame>>>,
    running: AtomicBool,
}

/// Handle to the running control loop. Drop or [`Driver::shutdown`] stops the
/// wheels and returns the OI to passive.
pub struct Driver {
    shared: Arc<Shared>,
    handle: Option<JoinHandle<()>>,
}

impl Driver {
    /// Start the control loop over an already-open transport. Wakes the robot
    /// into SAFE mode, verifies it, probes the odometry source, then spawns the
    /// loop thread.
    pub fn start<T>(transport: T, cfg: Config) -> io::Result<Driver>
    where
        T: Read + Write + Send + 'static,
    {
        let mut oi = oi::Oi::new(transport);
        oi.wake_and_safe()?;

        // Verify SAFE (mode 2). The robot must already be powered on.
        let mode = oi.query(&[sensors::OI_MODE])?;
        match mode.get(&sensors::OI_MODE) {
            Some(2) => {}
            other => {
                return Err(io::Error::other(
                    format!("robot not in SAFE mode after wake (OI mode = {other:?})"),
                ));
            }
        }

        let (source, ids) = probe_source(&mut oi)?;

        let shared = Arc::new(Shared {
            setpoint: Mutex::new((Twist::STOP, Instant::now())),
            subscribers: Mutex::new(Vec::new()),
            running: AtomicBool::new(true),
        });

        let core = ControlCore::new(cfg, source);
        let loop_shared = shared.clone();
        let handle = thread::Builder::new()
            .name("driver-control".into())
            .spawn(move || run_loop(oi, core, cfg, ids, loop_shared))
            .map_err(io::Error::other)?;

        Ok(Driver { shared, handle: Some(handle) })
    }

    /// Open the serial port at 115200 baud (the 770's OI rate) and start.
    pub fn open(port: &str, cfg: Config) -> io::Result<Driver> {
        let transport = serialport::new(port, 115_200)
            .timeout(cfg.tick) // a read can't outlast a tick or we miss the deadline
            .open()
            .map_err(io::Error::other)?;
        Driver::start(transport, cfg)
    }

    /// Stop the control loop, halt the wheels, and return the OI to passive.
    pub fn shutdown(mut self) {
        self.stop_and_join();
    }

    fn stop_and_join(&mut self) {
        self.shared.running.store(false, Ordering::SeqCst);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for Driver {
    fn drop(&mut self) {
        self.stop_and_join();
    }
}

impl RobotIo for Driver {
    fn set_twist(&self, twist: Twist) {
        let mut g = self.shared.setpoint.lock().unwrap();
        *g = (twist, Instant::now());
    }

    fn subscribe(&self) -> Receiver<SensorFrame> {
        let (tx, rx) = mpsc::channel();
        self.shared.subscribers.lock().unwrap().push(tx);
        rx
    }
}

/// Probe which odometry packets the robot answers, preferring the wheel
/// encoders (43/44) and falling back to integrated distance/angle (19/20).
/// Returns the source plus the full per-tick query id list (source + base).
fn probe_source<T: Read + Write>(oi: &mut oi::Oi<T>) -> io::Result<(OdomSource, Vec<u8>)> {
    let (source, odom_ids): (OdomSource, &[u8]) =
        if oi.query(&[sensors::ENCODER_LEFT, sensors::ENCODER_RIGHT]).is_ok() {
            (OdomSource::Encoders, &[sensors::ENCODER_LEFT, sensors::ENCODER_RIGHT])
        } else if oi.query(&[19, 20]).is_ok() {
            (OdomSource::Integrated, &[19, 20])
        } else {
            return Err(io::Error::other(
                "robot answered neither encoders (43/44) nor integrated odometry (19/20)",
            ));
        };
    oi.drain();
    let mut ids = odom_ids.to_vec();
    ids.extend_from_slice(BASE_IDS);
    Ok((source, ids))
}

/// The control-loop body. Runs on its own thread until `running` is cleared.
fn run_loop<T: Read + Write>(
    mut oi: oi::Oi<T>,
    mut core: ControlCore,
    cfg: Config,
    ids: Vec<u8>,
    shared: Arc<Shared>,
) {
    let t0 = Instant::now();
    while shared.running.load(Ordering::SeqCst) {
        let tick_start = Instant::now();
        let (twist, set_at) = *shared.setpoint.lock().unwrap();
        let age = tick_start.saturating_duration_since(set_at);
        let t_s = tick_start.saturating_duration_since(t0).as_secs_f64();

        match oi.query(&ids) {
            Ok(map) => {
                let raw = RawSensors::from_readings(&map);
                let tick = core.step(twist, age, &raw, t_s);
                let _ = oi.drive_direct(tick.wheels.0, tick.wheels.1);
                publish(&shared, tick.frame);
            }
            Err(_) => {
                // Lost a sensor read (timeout / OI hiccup): fail safe by halting
                // the wheels and skipping odometry this tick. The watchdog and
                // the next good read recover us.
                oi.drain();
                let _ = oi.stop();
            }
        }

        let elapsed = tick_start.elapsed();
        if elapsed < cfg.tick {
            thread::sleep(cfg.tick - elapsed);
        }
    }

    // Clean exit: wheels off, OI back to passive.
    let _ = oi.stop();
    let _ = oi.passive();
}

/// Send a frame to every live subscriber, dropping any whose receiver is gone.
fn publish(shared: &Shared, frame: SensorFrame) {
    let mut subs = shared.subscribers.lock().unwrap();
    subs.retain(|tx| tx.send(frame).is_ok());
}

/// The base packet id set polled every tick (excludes the odometry source ids,
/// which are prepended at startup). Exposed for back-ends/tests.
pub fn base_ids() -> &'static [u8] {
    BASE_IDS
}
