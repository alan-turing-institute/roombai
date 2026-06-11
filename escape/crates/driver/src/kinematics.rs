//! Differential-drive kinematics and acceleration limiting (pure, SPEC §4.1).

use escape_core::Twist;

use crate::config::MAX_WHEEL_MM_S;

/// Right/left wheel velocities for a body `Twist`, given the command wheel span.
///
/// `omega > 0` is CCW (left turn): the right wheel runs faster forward. Result
/// is clamped to the OI ±500 mm/s limit and rounded to i16.
pub fn twist_to_wheels(twist: Twist, span_mm: f64) -> (i16, i16) {
    let half = span_mm / 2.0;
    let right = twist.v_mm_s + twist.omega_rad_s * half;
    let left = twist.v_mm_s - twist.omega_rad_s * half;
    (clamp_wheel(right), clamp_wheel(left))
}

fn clamp_wheel(v: f64) -> i16 {
    v.clamp(-MAX_WHEEL_MM_S, MAX_WHEEL_MM_S).round() as i16
}

/// Slew-rate limiter on the commanded `Twist`. Holds the last applied velocity
/// and steps it toward the target by at most `accel * dt` each tick, so motion
/// ramps instead of jerking (smoother + less wheel slip → better odometry).
#[derive(Debug, Clone, Copy, Default)]
pub struct MotionShaper {
    v_mm_s: f64,
    omega_rad_s: f64,
}

impl MotionShaper {
    /// Advance one tick toward `target`, limited by the per-axis accelerations.
    pub fn shape(
        &mut self,
        target: Twist,
        dt: f64,
        max_lin_accel_mm_s2: f64,
        max_ang_accel_rad_s2: f64,
    ) -> Twist {
        self.v_mm_s = step_toward(self.v_mm_s, target.v_mm_s, max_lin_accel_mm_s2 * dt);
        self.omega_rad_s =
            step_toward(self.omega_rad_s, target.omega_rad_s, max_ang_accel_rad_s2 * dt);
        Twist { v_mm_s: self.v_mm_s, omega_rad_s: self.omega_rad_s }
    }

    /// Current applied velocity (what the wheels were last told to do).
    pub fn current(&self) -> Twist {
        Twist { v_mm_s: self.v_mm_s, omega_rad_s: self.omega_rad_s }
    }
}

fn step_toward(current: f64, target: f64, max_step: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_step {
        target
    } else {
        current + max_step.copysign(delta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_drive_splits_evenly() {
        let (r, l) = twist_to_wheels(Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, 235.0);
        assert_eq!((r, l), (200, 200));
    }

    #[test]
    fn ccw_spin_runs_right_wheel_forward() {
        // Pure CCW (+omega): right wheel forward, left wheel back.
        let (r, l) = twist_to_wheels(Twist { v_mm_s: 0.0, omega_rad_s: 1.0 }, 200.0);
        assert_eq!((r, l), (100, -100));
    }

    #[test]
    fn wheel_speed_is_clamped() {
        let (r, l) = twist_to_wheels(Twist { v_mm_s: 1000.0, omega_rad_s: 0.0 }, 235.0);
        assert_eq!((r, l), (500, 500));
    }

    #[test]
    fn shaper_ramps_then_settles() {
        let mut s = MotionShaper::default();
        let target = Twist { v_mm_s: 100.0, omega_rad_s: 0.0 };
        // accel 150 mm/s², dt 0.5 s → max step 75 mm/s per tick.
        let a = s.shape(target, 0.5, 150.0, 3.0);
        assert_eq!(a.v_mm_s, 75.0);
        let b = s.shape(target, 0.5, 150.0, 3.0);
        assert_eq!(b.v_mm_s, 100.0); // remaining 25 < 75, snaps to target
    }

    #[test]
    fn shaper_ramps_down_symmetrically() {
        let mut s = MotionShaper::default();
        s.shape(Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, 10.0, 150.0, 3.0); // jump to 200
        let d = s.shape(Twist::STOP, 0.5, 150.0, 3.0);
        assert_eq!(d.v_mm_s, 125.0); // 200 - 75
    }
}
