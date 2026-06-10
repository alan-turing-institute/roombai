//! The scripted phases of the 5-minute characterization session.
//!
//! No external props (no checkerboard, ruler, or tape): the calibration target
//! is the robot's own ego-motion. Video records continuously through the motion
//! phases, and the pixel->ground relationship and odometry consistency are
//! recovered offline from how the floor flows under known encoder motion. Scale
//! is left in encoder/ground units (nominal metric scale comes free from the
//! wheel spec); only self-consistency matters.
//!
//! A   — probe: which OI packets does this 770 actually answer? stream mode?
//! A.5 — sync pulse: forward/back jerks to align video clock to encoder log.
//! B   — straight runs: slow leg (clean floor flow) + fast leg + creep to
//!       bumper contact (light-bump distance curve), then reverse home.
//! C   — rotation: spins (slow CCW, slow CW, fast CCW); encoder rotation is
//!       cross-checked against visually-observed rotation offline.
//! D   — floor traverse: out-and-back over the dark tile / grey hatch band,
//!       logging cliff reflectivity; video collects floor appearance data.
//!
//! Every motion primitive is `drive_until`: re-issues its drive command each
//! tick, samples sensors at 20 Hz, logs every tick, and stops on predicate,
//! bump, wheel-drop, time cap, or ctrl-C — there are no open-loop moves.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use roomba_pilot::protocol::{RADIUS_CCW, RADIUS_STRAIGHT};
use serde_json::{json, Value};

use crate::logger::Logger;
use crate::rig::Rig;
use crate::telemetry::{self, Caps, Odom, OdomSource};
use crate::tts;

const TICK_PERIOD: Duration = Duration::from_millis(50); // 20 Hz
const HALF_SPAN_MM: f64 = telemetry::WHEEL_SPAN_MM / 2.0;

pub struct Ctx<T: Read + Write> {
    pub rig: Rig<T>,
    pub log: Logger,
    pub odom: Odom,
    pub abort: std::sync::Arc<AtomicBool>,
    pub t0: Instant,
    pub step: bool,
    pub results: Vec<Value>,
}

impl<T: Read + Write> Ctx<T> {
    pub fn aborted(&self) -> bool {
        self.abort.load(Ordering::SeqCst)
    }

    /// Abort-aware sleep (50 ms granularity).
    pub fn sleep(&self, d: Duration) {
        let end = Instant::now() + d;
        while Instant::now() < end && !self.aborted() {
            std::thread::sleep(TICK_PERIOD);
        }
    }

    pub fn announce(&mut self, phase: &str, msg: &str) {
        let elapsed = self.t0.elapsed().as_secs();
        tts::say(msg);
        self.log.event(phase, msg, json!({"elapsed_s": elapsed}));
    }

    pub fn pause_if_step(&mut self) {
        if self.step && !self.aborted() {
            tts::say("Paused. Press enter to continue.");
            let mut line = String::new();
            let _ = std::io::stdin().read_line(&mut line);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopReason {
    Predicate,
    Bump,
    WheelDrop,
    Timeout,
    Abort,
    SensorFailure,
}

/// What one motion leg measured, by both odometry sources.
pub struct Leg {
    pub reason: StopReason,
    pub enc_dist_mm: f64,
    pub enc_angle_deg: f64,
    pub i_dist_mm: f64,
    pub i_angle_deg: f64,
    pub dur_s: f64,
}

impl Leg {
    fn to_json(&self, label: &str) -> Value {
        json!({
            "leg": label,
            "reason": format!("{:?}", self.reason),
            "enc_dist_mm": self.enc_dist_mm,
            "enc_angle_deg": self.enc_angle_deg,
            "i_dist_mm": self.i_dist_mm,
            "i_angle_deg": self.i_angle_deg,
            "dur_s": self.dur_s,
        })
    }
}

/// Progress relative to leg start, by the best available odometry source —
/// falling back to commanded-velocity dead reckoning so predicates (and
/// therefore phase behavior) still terminate sensibly with dead odometry.
pub struct Progress {
    pub dist_mm: f64,
    pub angle_deg: f64,
    #[allow(dead_code)] // available to predicates that want time-based cutoffs
    pub dur_s: f64,
}

/// Implied (velocity, yaw-rate) of a DRIVE command, for dead reckoning.
fn implied_rates(vel_mm_s: i16, radius: i16) -> (f64, f64) {
    let v = vel_mm_s as f64;
    if radius == RADIUS_STRAIGHT {
        (v, 0.0)
    } else if radius == 1 {
        (0.0, v / HALF_SPAN_MM * 180.0 / std::f64::consts::PI)
    } else if radius == -1 {
        (0.0, -v / HALF_SPAN_MM * 180.0 / std::f64::consts::PI)
    } else {
        (v, v / radius as f64 * 180.0 / std::f64::consts::PI)
    }
}

pub fn drive_until<T: Read + Write>(
    ctx: &mut Ctx<T>,
    caps: &Caps,
    label: &str,
    vel_mm_s: i16,
    radius: i16,
    cap: Duration,
    stop_on_bump: bool,
    mut pred: impl FnMut(&Progress) -> bool,
) -> Leg {
    let start = Instant::now();
    let snap0 = ctx.odom.snapshot();
    let (dr_v, dr_w) = implied_rates(vel_mm_s, radius);
    let mut sensor_errors = 0u32;

    let reason = loop {
        let _ = ctx.rig.drive(vel_mm_s, radius);
        let tick_start = Instant::now();

        match telemetry::tick(&mut ctx.rig, caps, &mut ctx.odom) {
            Ok(t) => {
                sensor_errors = 0;
                ctx.log.tick(label, &t, &ctx.odom);
                if t.wheel_drop {
                    break StopReason::WheelDrop;
                }
                if let Some(m) = t.oi_mode {
                    if m != 2 {
                        // Cliff/wheel-drop safety kicked the OI to passive.
                        ctx.log.event(label, "oi left safe mode, re-entering", json!({"mode": m}));
                        let _ = ctx.rig.reenter_safe();
                    }
                }
                if stop_on_bump && (t.bump_left || t.bump_right) {
                    break StopReason::Bump;
                }
            }
            Err(e) => {
                sensor_errors += 1;
                ctx.rig.drain();
                ctx.log.event(label, "sensor tick failed", json!({"err": e.to_string(), "consecutive": sensor_errors}));
                if sensor_errors >= 5 {
                    break StopReason::SensorFailure;
                }
            }
        }

        let dur = start.elapsed().as_secs_f64();
        let snap = ctx.odom.snapshot();
        let prog = match caps.source {
            OdomSource::Encoders => Progress {
                dist_mm: snap.enc_dist_mm - snap0.enc_dist_mm,
                angle_deg: snap.enc_angle_deg - snap0.enc_angle_deg,
                dur_s: dur,
            },
            OdomSource::Integrated => Progress {
                dist_mm: snap.i_dist_mm - snap0.i_dist_mm,
                angle_deg: snap.i_angle_deg - snap0.i_angle_deg,
                dur_s: dur,
            },
            OdomSource::None => Progress { dist_mm: dr_v * dur, angle_deg: dr_w * dur, dur_s: dur },
        };
        if pred(&prog) {
            break StopReason::Predicate;
        }
        if start.elapsed() > cap {
            break StopReason::Timeout;
        }
        if ctx.aborted() {
            break StopReason::Abort;
        }
        if let Some(remaining) = TICK_PERIOD.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(remaining);
        }
    };

    let _ = ctx.rig.stop();
    if reason == StopReason::WheelDrop {
        let _ = ctx.rig.reenter_safe();
        tts::say("Wheel drop. Motion halted.");
    }
    let snap = ctx.odom.snapshot();
    let leg = Leg {
        reason,
        enc_dist_mm: snap.enc_dist_mm - snap0.enc_dist_mm,
        enc_angle_deg: snap.enc_angle_deg - snap0.enc_angle_deg,
        i_dist_mm: snap.i_dist_mm - snap0.i_dist_mm,
        i_angle_deg: snap.i_angle_deg - snap0.i_angle_deg,
        dur_s: start.elapsed().as_secs_f64(),
    };
    ctx.log.event(label, "leg complete", leg.to_json(label));
    leg
}

// ---- Phase A: probe ----------------------------------------------------------

const PROBE_IDS: &[u8] = &[
    7, 8, 13, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 35, 43, 44, 45, 46, 47, 48, 49,
    50, 51,
];

pub fn probe<T: Read + Write>(ctx: &mut Ctx<T>) -> Caps {
    ctx.announce("probe", "Phase A. Probing sensors.");
    if let Err(e) = ctx.rig.wake_and_safe() {
        ctx.announce("probe", &format!("Failed to enter safe mode: {e}"));
    }

    let mut supported = std::collections::BTreeSet::new();
    let mut values = std::collections::BTreeMap::new();
    for &id in PROBE_IDS {
        match ctx.rig.query(&[id]) {
            Ok(rs) => {
                supported.insert(id);
                values.insert(id, rs[0].value);
                ctx.log.event("probe", "packet ok", json!({"id": id, "value": rs[0].value}));
            }
            Err(e) => {
                ctx.rig.drain();
                ctx.log.event("probe", "packet unsupported", json!({"id": id, "err": e.to_string()}));
            }
        }
        std::thread::sleep(Duration::from_millis(15));
    }

    if values.get(&35) != Some(&2) {
        ctx.announce("probe", "Warning: could not confirm safe mode.");
    }
    if let (Some(&charge), Some(&capacity)) = (values.get(&25), values.get(&26)) {
        if capacity > 0 {
            ctx.announce("probe", &format!("Battery at {} percent.", charge * 100 / capacity));
        }
    }

    let stream_frames = ctx.rig.stream_test(&[7, 43, 44], Duration::from_secs(2));
    ctx.log.event("probe", "stream test", json!({"valid_frames_in_2s": stream_frames}));

    let caps = Caps::build(supported, values, stream_frames);
    let enc = caps.supported.contains(&43) && caps.supported.contains(&44);
    let lb = (46..=51).all(|id| caps.supported.contains(&id));
    tts::say(&format!(
        "Encoders {}. Light bumps {}. Stream mode {}. Odometry source {:?}.",
        if enc { "present" } else { "missing" },
        if lb { "present" } else { "missing" },
        if caps.stream_frames > 10 { "working" } else { "not working" },
        caps.source,
    ));
    ctx.results.push(json!({"phase": "probe", "caps": caps.to_json()}));
    caps
}

// ---- Phase A.5: sync pulse ----------------------------------------------------

/// Two short forward/back jerks, recorded on video. The distinctive
/// double-lurch lets the offline tooling cross-correlate the video against the
/// encoder trace and align the two clocks — no external sync prop needed.
pub fn sync_pulse<T: Read + Write>(ctx: &mut Ctx<T>, caps: &Caps) {
    ctx.announce("sync", "Sync pulse.");
    for _ in 0..2 {
        if ctx.aborted() {
            return;
        }
        drive_until(ctx, caps, "sync_fwd", 120, RADIUS_STRAIGHT, Duration::from_secs(2), true, |p| p.dist_mm >= 90.0);
        drive_until(ctx, caps, "sync_back", -120, RADIUS_STRAIGHT, Duration::from_secs(2), false, |p| p.dist_mm <= -90.0);
    }
}

// ---- Phase B: straight runs + wall approach ----------------------------------
//
// No external measurement: the straight legs are the data for self-calibrating
// the pixel->ground relationship (in encoder units) from how the floor flows
// through the image. The slow leg gives clean, low-blur optical flow; the fast
// leg checks odometry/flow at real driving speed; the creep-to-contact anchors
// the light-bump signal-vs-distance curve at zero distance.

pub fn wall_run<T: Read + Write>(ctx: &mut Ctx<T>, caps: &Caps) {
    ctx.announce("straight", "Phase B. Straight runs toward the wall. Stand clear of the lane.");

    // Slow calibration leg: 80 mm/s, ~0.9 m. Low blur -> clean floor flow.
    let slow = drive_until(ctx, caps, "fwd_slow", 80, RADIUS_STRAIGHT, Duration::from_secs(16), true, |p| {
        p.dist_mm >= 900.0
    });
    recover_if_bumped(ctx, caps, "fwd_slow", &slow);

    // Fast leg: 200 mm/s until close to the wall (odometry + flow at speed).
    let fast = drive_until(ctx, caps, "fwd_fast", 200, RADIUS_STRAIGHT, Duration::from_secs(8), true, |p| {
        p.dist_mm >= 600.0
    });

    // Creep to bumper contact -> light-bump curve anchored at distance zero.
    let creep = if fast.reason == StopReason::Predicate {
        Some(drive_until(ctx, caps, "wall_creep", 50, RADIUS_STRAIGHT, Duration::from_secs(20), true, |_| false))
    } else {
        None
    };
    let outbound = slow.enc_dist_mm.abs().max(slow.i_dist_mm.abs())
        + fast.enc_dist_mm.abs().max(fast.i_dist_mm.abs())
        + creep.as_ref().map_or(0.0, |c| c.enc_dist_mm.abs().max(c.i_dist_mm.abs()));
    let got_contact = fast.reason == StopReason::Bump
        || creep.as_ref().is_some_and(|c| c.reason == StopReason::Bump);
    tts::say(if got_contact { "Contact." } else { "No bumper contact registered." });
    ctx.sleep(Duration::from_millis(600));

    // Reverse roughly back to the start so later phases have room.
    let target = if outbound > 100.0 { outbound } else { 1400.0 };
    let back = drive_until(ctx, caps, "fwd_back", -200, RADIUS_STRAIGHT, Duration::from_secs(15), false, |p| {
        p.dist_mm <= -target
    });

    ctx.results.push(json!({
        "phase": "straight",
        "contact": got_contact,
        "outbound_dist_mm": outbound,
        "legs": [
            slow.to_json("fwd_slow"),
            fast.to_json("fwd_fast"),
            creep.map(|c| c.to_json("wall_creep")).unwrap_or(Value::Null),
            back.to_json("fwd_back"),
        ],
    }));
}

// ---- Phase C: rotation -------------------------------------------------------
//
// Each spin returns (nominally) to the same heading, so encoder-reported
// rotation can be checked against the visually-observed rotation in the video
// offline — no protractor. Slow spin = clean visual yaw; fast spin exposes
// wheel slip at speed. CCW and CW expose any directional asymmetry.

pub fn rotation<T: Read + Write>(ctx: &mut Ctx<T>, caps: &Caps) {
    ctx.announce("rotation", "Phase C. Rotation runs.");
    let specs: &[(&str, f64, i16)] =
        &[("spin_ccw_45", 45.0, 1), ("spin_cw_45", 45.0, -1), ("spin_ccw_150", 150.0, 1)];
    let mut legs = Vec::new();
    for &(label, deg_s, radius) in specs {
        if ctx.aborted() {
            break;
        }
        let vel = (deg_s.to_radians() * HALF_SPAN_MM) as i16;
        let nominal = 360.0 / deg_s;
        let cap = Duration::from_secs_f64(nominal * 2.5);
        let leg = drive_until(ctx, caps, label, vel, radius, cap, false, |p| p.angle_deg.abs() >= 357.0);
        // Brief settle so the video shows a clean stationary frame to register
        // the start/end heading against.
        ctx.sleep(Duration::from_millis(1200));
        legs.push(leg.to_json(label));
    }
    ctx.results.push(json!({"phase": "rotation", "legs": legs}));
}

// ---- Phase D: floor traverse ---------------------------------------------------

pub fn floor_traverse<T: Read + Write>(ctx: &mut Ctx<T>, caps: &Caps) {
    ctx.announce("floor", "Phase D. Floor traverse over the dark tiles.");
    // Leg lengths are sized for the same lane as the wall run: T0 is 1.8 m from
    // the wall, and a 180° arc of radius 0.3 m bulges 0.3 m past its entry
    // point, so 1.2 m out + arc apex leaves 0.3 m of wall clearance.
    let out = drive_until(ctx, caps, "floor_out", 200, RADIUS_STRAIGHT, Duration::from_secs(10), true, |p| {
        p.dist_mm >= 1200.0
    });
    recover_if_bumped(ctx, caps, "floor_out", &out);
    // 180° arc turn (300 mm radius, CCW) onto the return lane.
    let arc = drive_until(ctx, caps, "floor_arc", 200, 300, Duration::from_secs(12), true, |p| {
        p.angle_deg >= 175.0
    });
    recover_if_bumped(ctx, caps, "floor_arc", &arc);
    let ret = drive_until(ctx, caps, "floor_return", 200, RADIUS_STRAIGHT, Duration::from_secs(10), true, |p| {
        p.dist_mm >= 1200.0
    });
    ctx.results.push(json!({
        "phase": "floor",
        "legs": [out.to_json("floor_out"), arc.to_json("floor_arc"), ret.to_json("floor_return")],
    }));
}

// ---- S2: square drive (rotation calibration) ---------------------------------
//
// Drive a 1 m square: four equal sides with a 90° CCW turn between each. If the
// turns are truly 90° the robot returns to its start point regardless of the
// exact side length — so the home-return error isolates rotation accuracy. S1
// showed the encoder OVER-reports rotation (a commanded full turn physically
// swept only ~320°), so each turn here stops when the encoder reads
// 90 * TURN_ENC_PER_REAL_DEG, i.e. it deliberately lets the encoder run past 90
// to land a true 90. Measure the residual home offset to refine the factor.

/// Encoder-degrees logged per real degree of body rotation, from S1: the spins
/// stopped at ~359° encoder having physically turned ~320°. Edit this after
/// measuring S2's home offset to re-tune (larger -> turns further per corner).
const TURN_ENC_PER_REAL_DEG: f64 = 359.0 / 320.0; // ≈ 1.122

pub fn square_run<T: Read + Write>(ctx: &mut Ctx<T>, caps: &Caps) {
    let turn_target = 90.0 * TURN_ENC_PER_REAL_DEG;
    ctx.announce(
        "square",
        &format!(
            "S2. Driving a 1 metre square. Four 90 degree left turns, stopping each at {:.0} encoder degrees.",
            turn_target
        ),
    );
    let turn_vel = (45.0_f64.to_radians() * HALF_SPAN_MM) as i16; // 45 deg/s, the clean spin rate
    let mut legs = Vec::new();

    for corner in 0..4 {
        if ctx.aborted() {
            break;
        }
        // Side: 1 m forward. Bumping anything means the square is compromised —
        // stop the test rather than grind on and report a meaningless offset.
        let side_label = format!("sq_side_{}", corner + 1);
        ctx.announce("square", &format!("Side {}.", corner + 1));
        let side = drive_until(ctx, caps, &side_label, 150, RADIUS_STRAIGHT, Duration::from_secs(12), true, |p| {
            p.dist_mm >= 1000.0
        });
        legs.push(side.to_json(&side_label));
        if side.reason == StopReason::Bump {
            ctx.announce("square", &format!("Bump on side {}. Aborting square; clear the lane and rerun.", corner + 1));
            break;
        }
        ctx.sleep(Duration::from_millis(700)); // settle for a clean corner on video

        if ctx.aborted() {
            break;
        }
        let turn_label = format!("sq_turn_{}", corner + 1);
        ctx.announce("square", &format!("Turn {}.", corner + 1));
        let turn = drive_until(ctx, caps, &turn_label, turn_vel, RADIUS_CCW, Duration::from_secs(8), false, |p| {
            p.angle_deg >= turn_target
        });
        legs.push(turn.to_json(&turn_label));
        ctx.sleep(Duration::from_millis(700));
    }

    ctx.announce("square", "Square complete. Stopped at the start point. Measure the offset from home.");
    ctx.results.push(json!({
        "phase": "square",
        "side_mm": 1000.0,
        "turn_deg": 90.0,
        "turn_enc_per_real_deg": TURN_ENC_PER_REAL_DEG,
        "turn_target_enc_deg": turn_target,
        "legs": legs,
    }));
}

/// If a leg ended in bumper contact, back off 150 mm so the next leg doesn't
/// start pressed against the obstacle.
fn recover_if_bumped<T: Read + Write>(ctx: &mut Ctx<T>, caps: &Caps, label: &str, leg: &Leg) {
    if leg.reason == StopReason::Bump && !ctx.aborted() {
        ctx.log.event(label, "bump recovery: reversing 150 mm", json!({}));
        drive_until(ctx, caps, "bump_recover", -150, RADIUS_STRAIGHT, Duration::from_secs(3), false, |p| {
            p.dist_mm <= -150.0
        });
    }
}

pub const NOTES_TEMPLATE: &str = r#"# Session 1 — notes (fill in right after the run)

No measurements needed — calibration comes from the robot's own motion in the
video. These notes are just context for the offline analysis.

- Phone video filename (optional, independent cross-check of the spins):
- Roughly which floor regions (dark-blue tiles / grey hatch panels) did the
  robot drive over, and during which phase?
- Anything unexpected (slip, stalls, bumps, camera glitches, near-misses):
"#;
