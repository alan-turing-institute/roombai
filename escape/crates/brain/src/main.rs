//! `wander` — drive around using the camera, avoiding obstacles (SPEC M4 / S3).
//!
//! The loop, ~10 Hz:
//!   camera frame → vision::segment_floor → freespace_to_polar → Navigator
//!   → driver.set_twist
//! plus a reflex-recovery override: if the driver reports a bump (its safety
//! layer already suppressed forward motion), we reverse straight by 25 cm
//! before resuming vision steering, so a missed obstacle degrades to a nudge,
//! not a crash. A wheel-drop pauses all motion until the robot is set down.
//! (Cliff sensing is intentionally not wired up yet.)
//!
//! No vision/driver test rig here — those are validated separately. This binary
//! is the integration glue and is meant to run on the robot:
//!   cargo run -p brain --bin wander --release -- --port /dev/ttyUSB0
//!
//! Build on the Pi (where rpicam-vid and the serial port exist); it compiles on
//! the dev machine but won't find a camera/robot there.

mod camera;
mod tts;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use camera::Camera;
use driver::{Config, Driver};
use escape_core::{Reflex, RobotIo, SensorFrame, Twist};
use planner::{freespace_to_polar, ImageCal, NavState, Navigator, NavParams};
use vision::{segment_floor, Frame, Params, GATE_CHROMA_SPREAD};

const LOOP_HZ: u64 = 10;
const CAMERA_FPS: u32 = 15;
/// On a bump, reverse straight by this distance (no turn), measured by odometry.
const REVERSE_DIST_MM: f64 = 250.0;
const REVERSE_SPEED_MM_S: f64 = -100.0;

struct Opts {
    port: String,
    max_speed: f64,
    /// Drive only when a heading's clearance ≥ this (planner `safe_clearance`).
    safe_clearance: f32,
    /// Gate the frame as blocked when chroma spread < this (vision gate).
    block_chroma: f32,
    record: Option<PathBuf>,
}

fn usage() -> ! {
    eprintln!(
        "usage: wander [--port /dev/ttyUSB0] [--max-speed MM_S] \
         [--safe-clearance FRAC] [--block-chroma SPREAD] [--record DIR]"
    );
    std::process::exit(2);
}

fn parse_args() -> Opts {
    let mut o = Opts {
        port: "/dev/ttyUSB0".into(),
        max_speed: 150.0, // conservative first-run cap; raise once tuned
        // Defaults are the decided values, pulled from the library so they stay
        // in sync if the crate defaults change.
        safe_clearance: NavParams::default().safe_clearance,
        block_chroma: GATE_CHROMA_SPREAD,
        record: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--port" => o.port = it.next().unwrap_or_else(|| usage()),
            "--max-speed" => o.max_speed = next_num(&mut it),
            "--safe-clearance" => o.safe_clearance = next_num(&mut it) as f32,
            "--block-chroma" => o.block_chroma = next_num(&mut it) as f32,
            "--record" => o.record = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            _ => usage(),
        }
    }
    o
}

/// Parse the next CLI token as a number, or exit with usage.
fn next_num(it: &mut impl Iterator<Item = String>) -> f64 {
    it.next().and_then(|s| s.parse().ok()).unwrap_or_else(|| usage())
}

fn main() {
    let opts = parse_args();

    let abort = Arc::new(AtomicBool::new(false));
    {
        let a = abort.clone();
        ctrlc::set_handler(move || a.store(true, Ordering::SeqCst)).expect("install ctrl-c handler");
    }

    tts::say("Wander starting.");

    // Bring up the driver (wakes the robot to SAFE, probes odometry, starts the
    // 20 Hz control loop). The robot must already be powered on.
    let driver = match Driver::open(&opts.port, Config::default()) {
        Ok(d) => d,
        Err(e) => {
            tts::say(&format!("Driver failed to start: {e}. Is the robot on and the pilot daemon stopped?"));
            std::process::exit(1);
        }
    };
    let sensors = driver.subscribe();

    let mut cam = match Camera::start(CAMERA_FPS) {
        Ok(c) => c,
        Err(e) => {
            tts::say(&format!("Camera failed to start: {e}."));
            driver.shutdown();
            std::process::exit(1);
        }
    };

    if let Some(dir) = &opts.record {
        let _ = std::fs::create_dir_all(dir);
    }

    let cal = ImageCal::default();
    let nav_params =
        NavParams { max_v_mm_s: opts.max_speed, safe_clearance: opts.safe_clearance, ..Default::default() };
    let mut nav = Navigator::new(nav_params);
    let base_params =
        Params { block_chroma_spread_below: Some(opts.block_chroma), ..Default::default() };
    tts::say(&format!(
        "Safe clearance {:.2}, block chroma {:.1}.",
        opts.safe_clearance, opts.block_chroma
    ));

    tts::say("Driving.");
    let outcome = run_loop(&opts, &driver, &sensors, &mut cam, &mut nav, &cal, &base_params, &abort);

    // Clean stop: wheels off, OI to passive, camera released.
    cam.stop();
    driver.shutdown();
    tts::say(match outcome {
        Outcome::Aborted => "Stopped by operator. Wheels off.",
        Outcome::CameraLost => "Camera stream ended. Wheels off.",
    });
}

enum Outcome {
    Aborted,
    CameraLost,
}

/// Track of recovery / pause behaviour between iterations.
#[derive(Default)]
struct Behaviour {
    /// Pose where the current bump-reverse began; `Some` while reversing.
    reverse_from: Option<escape_core::Pose2>,
    paused_for_drop: bool,
    last_spoken: Option<&'static str>,
}

#[allow(clippy::too_many_arguments)]
fn run_loop(
    opts: &Opts,
    driver: &Driver,
    sensors: &std::sync::mpsc::Receiver<SensorFrame>,
    cam: &mut Camera,
    nav: &mut Navigator,
    cal: &ImageCal,
    base_params: &Params,
    abort: &AtomicBool,
) -> Outcome {
    let period = Duration::from_millis(1000 / LOOP_HZ);
    let t0 = Instant::now();
    let mut beh = Behaviour::default();
    let mut frame_idx: u64 = 0;
    let mut last_record = Instant::now() - Duration::from_secs(1);
    let mut no_frame_since: Option<Instant> = None;
    // Most recent sensor frame, persisted across iterations (for pose + reflex).
    let mut cur: Option<SensorFrame> = None;

    loop {
        let tick = Instant::now();
        if abort.load(Ordering::SeqCst) {
            return Outcome::Aborted;
        }

        // Drain the channel so we act on the freshest sensor frame.
        while let Ok(f) = sensors.try_recv() {
            cur = Some(f);
        }

        // --- Safety overrides, in priority order ---------------------------
        if let Some(f) = &cur {
            // Wheel-drop: hard pause until the robot is set back down.
            if matches!(f.reflex, Some(Reflex::WheelDrop)) {
                driver.set_twist(Twist::STOP);
                speak_once(&mut beh, "Lifted. Waiting to be set down.");
                beh.paused_for_drop = true;
                beh.reverse_from = None;
                sleep_rest(tick, period);
                continue;
            }
            if beh.paused_for_drop {
                beh.paused_for_drop = false;
                speak_once(&mut beh, "Back on the floor. Resuming.");
            }
            // Fresh bump and not already reversing → start a straight 25 cm
            // reverse, anchored at the current pose.
            if beh.reverse_from.is_none() && matches!(f.reflex, Some(Reflex::Bump { .. })) {
                beh.reverse_from = Some(f.pose);
                speak_once(&mut beh, "Contact. Backing up.");
            }
        }

        // Active reverse: drive straight back until odometry says 25 cm.
        if let Some(start) = beh.reverse_from {
            let travelled = cur.map(|f| dist(start, f.pose)).unwrap_or(0.0);
            if travelled < REVERSE_DIST_MM {
                driver.set_twist(Twist { v_mm_s: REVERSE_SPEED_MM_S, omega_rad_s: 0.0 });
                sleep_rest(tick, period);
                continue;
            }
            beh.reverse_from = None;
        }

        // --- Vision-driven steering ----------------------------------------
        let Some(rgb) = cam.latest_rgb() else {
            // No frame yet (startup) or the camera died.
            match no_frame_since {
                None => no_frame_since = Some(Instant::now()),
                Some(t) if t.elapsed() > Duration::from_secs(3) => {
                    driver.set_twist(Twist::STOP);
                    return Outcome::CameraLost;
                }
                _ => {}
            }
            driver.set_twist(Twist::STOP);
            sleep_rest(tick, period);
            continue;
        };
        no_frame_since = None;

        let fs = segment_floor(
            &Frame { width: camera::OUT_W, height: camera::OUT_H, rgb: &rgb },
            base_params,
        );
        let polar = freespace_to_polar(&fs, cal);
        let decision = nav.step(&polar);
        driver.set_twist(decision.twist);

        narrate_state(&mut beh, decision.state);

        if let Some(dir) = &opts.record {
            if last_record.elapsed() >= Duration::from_millis(500) {
                last_record = Instant::now();
                record(dir, frame_idx, &rgb, &decision, t0.elapsed().as_secs_f64());
            }
        }
        frame_idx += 1;

        sleep_rest(tick, period);
    }
}

/// Planar distance between two poses, mm.
fn dist(a: escape_core::Pose2, b: escape_core::Pose2) -> f64 {
    ((a.x_mm - b.x_mm).powi(2) + (a.y_mm - b.y_mm).powi(2)).sqrt()
}

fn narrate_state(beh: &mut Behaviour, state: NavState) {
    match state {
        NavState::Drive => speak_once(beh, "Clear ahead."),
        NavState::Search => speak_once(beh, "Blocked. Looking for an opening."),
    }
}

/// Speak a line only when it differs from the last thing said (anti-spam).
fn speak_once(beh: &mut Behaviour, msg: &'static str) {
    if beh.last_spoken != Some(msg) {
        beh.last_spoken = Some(msg);
        tts::say(msg);
    }
}

/// Save the exact frame the segmenter saw, plus a decision log line. The frame
/// is re-runnable through the offline steer_overlay example for diagnosis.
fn record(dir: &std::path::Path, idx: u64, rgb: &[u8], d: &planner::NavDecision, t_s: f64) {
    let name = format!("frame_{idx:05}.jpg");
    let _ = image::save_buffer(
        dir.join(&name),
        rgb,
        camera::OUT_W,
        camera::OUT_H,
        image::ColorType::Rgb8,
    );
    let line = serde_json::json!({
        "t_s": t_s,
        "frame": name,
        "state": format!("{:?}", d.state),
        "v_mm_s": d.twist.v_mm_s,
        "omega_rad_s": d.twist.omega_rad_s,
        "bearing_rad": d.chosen_bearing,
        "clearance": d.clearance,
    });
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(dir.join("decisions.jsonl"))
    {
        use std::io::Write;
        let _ = writeln!(f, "{line}");
    }
}

fn sleep_rest(tick: Instant, period: Duration) {
    let elapsed = tick.elapsed();
    if elapsed < period {
        std::thread::sleep(period - elapsed);
    }
}
