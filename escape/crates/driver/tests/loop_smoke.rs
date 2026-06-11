//! End-to-end smoke test of the threaded `Driver` over an in-memory fake robot.
//!
//! Exercises the real control loop through the `RobotIo` trait: wake/probe,
//! periodic frame publishing, setpoint → wheel commands, and a clean shutdown.
//! Timing-tolerant (asserts on counts/trends, not exact rates) so it's stable
//! in CI.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use driver::{Config, Driver};
use escape_core::{RobotIo, Twist};
use roomba_pilot::sensors;

/// Minimal OI fake: answers QUERY_LIST with canned values and integrates the
/// last DRIVE_DIRECT into wrapping wheel encoders so odometry actually moves.
/// Shares its last commanded wheels out so the test can observe them.
#[derive(Clone)]
struct FakeRobot {
    inner: Arc<Mutex<Inner>>,
}

struct Inner {
    rx: VecDeque<u8>,
    cmd: Vec<u8>,
    enc_l: u16,
    enc_r: u16,
    last_right: i16,
    last_left: i16,
    mode: u8,
}

impl FakeRobot {
    fn new() -> Self {
        FakeRobot {
            inner: Arc::new(Mutex::new(Inner {
                rx: VecDeque::new(),
                cmd: Vec::new(),
                enc_l: 1000,
                enc_r: 1000,
                last_right: 0,
                last_left: 0,
                mode: 1,
            })),
        }
    }

    fn last_wheels(&self) -> (i16, i16) {
        let i = self.inner.lock().unwrap();
        (i.last_right, i.last_left)
    }
}

impl Inner {
    fn value(&self, id: u8) -> i32 {
        match id {
            sensors::OI_MODE => self.mode as i32,
            sensors::ENCODER_LEFT => self.enc_l as i32,
            sensors::ENCODER_RIGHT => self.enc_r as i32,
            // High cliff reflectivity = "floor present, no edge".
            28..=31 => 1500,
            sensors::BATTERY_CHARGE => 2200,
            _ => 0,
        }
    }

    fn handle(&mut self) {
        loop {
            let Some(&op) = self.cmd.first() else { return };
            let fixed: Option<usize> = match op {
                128 | 131 | 132 => Some(0),
                145 => Some(4),
                149 => None,
                _ => {
                    self.cmd.remove(0);
                    continue;
                }
            };
            let (args_len, total) = match fixed {
                Some(n) => (n, 1 + n),
                None => {
                    if self.cmd.len() < 2 {
                        return;
                    }
                    let n = self.cmd[1] as usize;
                    (1 + n, 2 + n)
                }
            };
            if self.cmd.len() < total {
                return;
            }
            let args: Vec<u8> = self.cmd[1..1 + args_len].to_vec();
            self.cmd.drain(..total);
            match op {
                128 => self.mode = 1,
                131 => self.mode = 2,
                145 => {
                    self.last_right = i16::from_be_bytes([args[0], args[1]]);
                    self.last_left = i16::from_be_bytes([args[2], args[3]]);
                    // Pretend one tick of motion elapsed: nudge encoders.
                    self.enc_r = self.enc_r.wrapping_add((self.last_right / 20).max(0) as u16);
                    self.enc_l = self.enc_l.wrapping_add((self.last_left / 20).max(0) as u16);
                }
                149 => {
                    for &id in &args[1..] {
                        let w = sensors::width(id).unwrap_or(0);
                        let v = self.value(id);
                        match w {
                            1 => self.rx.push_back(v as u8),
                            2 => self.rx.extend((v as i16 as u16).to_be_bytes()),
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

impl Write for FakeRobot {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut i = self.inner.lock().unwrap();
        i.cmd.extend_from_slice(buf);
        i.handle();
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Read for FakeRobot {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut i = self.inner.lock().unwrap();
        let n = buf.len().min(i.rx.len());
        for b in buf.iter_mut().take(n) {
            *b = i.rx.pop_front().unwrap();
        }
        Ok(n) // Ok(0) when empty mimics a serial read timeout for read_exact.
    }
}

#[test]
fn driver_publishes_frames_and_drives_on_setpoint() {
    let fake = FakeRobot::new();
    let driver = Driver::start(fake.clone(), Config::default()).expect("driver starts");

    let frames = driver.subscribe();

    // Command a forward setpoint and let the loop run a handful of ticks.
    driver.set_twist(Twist { v_mm_s: 200.0, omega_rad_s: 0.0 });
    std::thread::sleep(Duration::from_millis(250));

    // Frames are flowing, sequence numbers are monotonic, and pose advanced.
    let collected: Vec<_> = std::iter::from_fn(|| frames.try_recv().ok()).collect();
    assert!(collected.len() >= 2, "expected several frames, got {}", collected.len());
    assert!(collected.windows(2).all(|w| w[1].seq == w[0].seq + 1), "seq must be monotonic");
    assert!(collected.last().unwrap().pose.x_mm > 0.0, "forward setpoint should advance pose");

    // Wheels are being commanded forward (ramped, but positive).
    let (r, l) = fake.last_wheels();
    assert!(r > 0 && l > 0, "wheels should be driving forward, got ({r}, {l})");

    driver.shutdown();
    // After shutdown the loop halts the wheels on exit.
    let (r, l) = fake.last_wheels();
    assert_eq!((r, l), (0, 0), "shutdown should stop the wheels");
}

#[test]
fn watchdog_zeroes_wheels_without_fresh_setpoints() {
    let fake = FakeRobot::new();
    let driver = Driver::start(fake.clone(), Config::default()).expect("driver starts");

    driver.set_twist(Twist { v_mm_s: 200.0, omega_rad_s: 0.0 });
    std::thread::sleep(Duration::from_millis(120));
    assert!(fake.last_wheels().0 > 0, "should be moving right after a fresh setpoint");

    // Stop refreshing the setpoint; after the 300 ms watchdog the loop ramps to 0.
    std::thread::sleep(Duration::from_millis(600));
    assert_eq!(fake.last_wheels(), (0, 0), "watchdog should zero wheels when setpoints go stale");

    driver.shutdown();
}
