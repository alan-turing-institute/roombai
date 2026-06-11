//! Dead-reckoning the pose from wheel encoders (preferred) or the integrated
//! distance/angle packets (fallback). Pure; the control loop feeds it readings.

use std::f64::consts::PI;

use escape_core::Pose2;

use crate::config::MM_PER_TICK;

/// Which OI source drives the pose. Chosen once at startup by probing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OdomSource {
    /// Wheel encoders 43/44 — fine-grained, trusted (memory: rotation solved).
    Encoders,
    /// Integrated distance/angle 19/20 — coarse, quantized; sets `degraded`.
    Integrated,
}

impl OdomSource {
    pub fn is_degraded(self) -> bool {
        matches!(self, OdomSource::Integrated)
    }
}

/// Incremental pose integrator. Encoder counts are 16-bit and wrap; deltas are
/// taken wrap-aware. Pose uses the core convention (+x forward, +y left, +theta
/// CCW) in the start-anchored frame.
#[derive(Debug, Clone)]
pub struct Odometry {
    pose: Pose2,
    odom_span_mm: f64,
    prev_enc: Option<(i32, i32)>,
}

impl Odometry {
    pub fn new(odom_span_mm: f64) -> Self {
        Odometry { pose: Pose2::default(), odom_span_mm, prev_enc: None }
    }

    pub fn pose(&self) -> Pose2 {
        self.pose
    }

    /// Integrate one encoder sample (left = packet 43, right = packet 44, raw
    /// 16-bit cumulative counts). The first sample only seeds the reference.
    pub fn update_encoders(&mut self, left: i32, right: i32) {
        if let Some((pl, pr)) = self.prev_enc {
            let dl = wrap16(left, pl) as f64 * MM_PER_TICK;
            let dr = wrap16(right, pr) as f64 * MM_PER_TICK;
            self.integrate(dl, dr);
        }
        self.prev_enc = Some((left, right));
    }

    /// Integrate one integrated-packet sample: distance mm (19) and angle deg
    /// (20), each the delta since the previous read.
    pub fn update_integrated(&mut self, dist_mm: f64, angle_deg: f64) {
        let ds = dist_mm;
        let dtheta = angle_deg.to_radians();
        self.advance(ds, dtheta);
    }

    /// Arc step from per-wheel ground distances.
    fn integrate(&mut self, dl: f64, dr: f64) {
        let ds = (dl + dr) / 2.0;
        let dtheta = (dr - dl) / self.odom_span_mm;
        self.advance(ds, dtheta);
    }

    /// Advance the pose by a forward arc (midpoint heading integration).
    fn advance(&mut self, ds: f64, dtheta: f64) {
        let mid = self.pose.theta_rad + dtheta / 2.0;
        self.pose.x_mm += ds * mid.cos();
        self.pose.y_mm += ds * mid.sin();
        self.pose.theta_rad = wrap_pi(self.pose.theta_rad + dtheta);
    }
}

/// Wrap-aware 16-bit encoder delta (counts roll over at 65535).
fn wrap16(cur: i32, prev: i32) -> i16 {
    (cur as u16).wrapping_sub(prev as u16) as i16
}

/// Normalize an angle to (-π, π].
fn wrap_pi(a: f64) -> f64 {
    let mut a = (a + PI).rem_euclid(2.0 * PI) - PI;
    if a <= -PI {
        a += 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap16_handles_rollover() {
        assert_eq!(wrap16(5, 65530), 11);
        assert_eq!(wrap16(65530, 5), -11);
    }

    #[test]
    fn straight_drive_advances_x_only() {
        let mut o = Odometry::new(247.0);
        o.update_encoders(1000, 1000); // seed
        // both wheels +100 ticks → pure forward, no turn
        o.update_encoders(1100, 1100);
        let p = o.pose();
        assert!((p.x_mm - 100.0 * MM_PER_TICK).abs() < 1e-9);
        assert!(p.y_mm.abs() < 1e-9);
        assert!(p.theta_rad.abs() < 1e-9);
    }

    #[test]
    fn spin_in_place_turns_ccw_without_translating() {
        let mut o = Odometry::new(247.0);
        o.update_encoders(1000, 1000);
        // right +100, left -100 → CCW rotation
        o.update_encoders(900, 1100);
        let p = o.pose();
        assert!(p.x_mm.abs() < 1e-9 && p.y_mm.abs() < 1e-9);
        let expected = 200.0 * MM_PER_TICK / 247.0;
        assert!((p.theta_rad - expected).abs() < 1e-9);
        assert!(p.theta_rad > 0.0); // CCW positive
    }

    #[test]
    fn odom_span_calibration_reduces_reported_angle() {
        // Same encoder delta read on nominal 235 vs calibrated 247: the larger
        // span reports the smaller (true) angle.
        let mut nominal = Odometry::new(235.0);
        let mut calibrated = Odometry::new(247.0);
        for o in [&mut nominal, &mut calibrated] {
            o.update_encoders(1000, 1000);
            o.update_encoders(900, 1100);
        }
        assert!(calibrated.pose().theta_rad < nominal.pose().theta_rad);
    }

    #[test]
    fn integrated_source_matches_encoders_for_a_quarter_turn() {
        let mut o = Odometry::new(247.0);
        o.update_integrated(0.0, 90.0);
        assert!((o.pose().theta_rad - PI / 2.0).abs() < 1e-9);
    }

    #[test]
    fn wrap_pi_keeps_angle_bounded() {
        let mut o = Odometry::new(247.0);
        o.update_integrated(0.0, 270.0); // 270° CCW → -90°
        assert!((o.pose().theta_rad - (-PI / 2.0)).abs() < 1e-9);
    }
}
