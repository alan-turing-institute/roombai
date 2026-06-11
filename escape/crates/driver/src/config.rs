//! Tunable driver constants. Defaults come from SPEC §4.1 and the on-robot
//! calibration recorded in the project memory (S2–S4 rotation tests).

use std::f64::consts::PI;
use std::time::Duration;

/// Distance one encoder tick represents: wheel dia 72 mm, 508.8 ticks/rev.
pub const MM_PER_TICK: f64 = 72.0 * PI / 508.8;

/// OI hard limit on commanded wheel velocity.
pub const MAX_WHEEL_MM_S: f64 = 500.0;

/// Nominal wheel track of the 770 (used for commanding motion).
pub const NOMINAL_SPAN_MM: f64 = 235.0;

/// Encoder-degrees logged per real degree of body rotation, measured on the
/// robot. The wheel encoders over-report rotation; integrating on the nominal
/// 235 mm span reads ~1.049× the true angle. This is the exact constant from
/// `characterize::phases::TURN_ENC_PER_REAL_DEG` that S4's perfect 1 m square
/// validated — keep the two in sync if either is re-tuned. (See memory:
/// rotation-calibration-solved.)
pub const TURN_ENC_PER_REAL_DEG: f64 = 1.049;

#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Control + sensing tick period (SPEC: 20 Hz).
    pub tick: Duration,
    /// No fresh setpoint within this window → command zero velocity (SPEC §4.1).
    pub watchdog: Duration,

    /// Max change in commanded forward speed per second (mm/s²) — smooths motion
    /// and protects odometry from wheel slip.
    pub max_lin_accel_mm_s2: f64,
    /// Max change in commanded yaw rate per second (rad/s²).
    pub max_ang_accel_rad_s2: f64,

    /// Wheel span used to turn a `Twist` into wheel velocities. Nominal track of
    /// the 770; the planner closes the loop on the (calibrated) odometry pose,
    /// so commands stay on the nominal geometry — matching how `characterize`
    /// (validated S1–S4) commanded its spins.
    pub cmd_span_mm: f64,
    /// Effective wheel span used when *integrating* encoder odometry. Equals the
    /// nominal span scaled by the measured over-report factor, so the reported
    /// angle is the true body angle: 235 × 1.049 ≈ 246.5 mm.
    pub odom_span_mm: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            tick: Duration::from_millis(50),
            watchdog: Duration::from_millis(300),
            max_lin_accel_mm_s2: 150.0,
            max_ang_accel_rad_s2: 3.0,
            cmd_span_mm: NOMINAL_SPAN_MM,
            odom_span_mm: NOMINAL_SPAN_MM * TURN_ENC_PER_REAL_DEG,
        }
    }
}

impl Config {
    /// Tick period in seconds.
    pub fn dt(&self) -> f64 {
        self.tick.as_secs_f64()
    }
}
