//! Shared, I/O-free types for the escape stack (SPEC §3.2 `core`).
//!
//! Conventions, fixed once here so every crate agrees:
//! - Angles in radians. **+angle is counter-clockwise (left)**, matching the
//!   wheel-encoder angle sign used in calibration.
//! - Body frame: +x forward, +y left. A `Twist` with `omega > 0` turns left.

use serde::{Deserialize, Serialize};

/// Body-frame velocity command, applied immediately by the driver (SPEC §3.1).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    /// Forward speed, mm/s (+forward).
    pub v_mm_s: f64,
    /// Yaw rate, rad/s (+CCW / left).
    pub omega_rad_s: f64,
}

impl Twist {
    pub const STOP: Twist = Twist { v_mm_s: 0.0, omega_rad_s: 0.0 };

    /// Pure rotation in place at `omega` rad/s (+CCW).
    pub fn spin(omega_rad_s: f64) -> Twist {
        Twist { v_mm_s: 0.0, omega_rad_s }
    }
}

/// Robot pose in the start-anchored ground frame (filled in by driver odometry).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Pose2 {
    pub x_mm: f64,
    pub y_mm: f64,
    pub theta_rad: f64,
}

/// One bearing's clearance.
///
/// `clearance` units are deliberately abstract: in v1 it is an image-space
/// proxy (fraction of frame height that is floor before the first obstacle);
/// once the IPM calibration lands it becomes ground distance. The planner only
/// relies on "larger = more room", so it is unchanged by that upgrade.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Ray {
    /// Bearing of this ray, rad (+left / CCW, 0 = straight ahead).
    pub bearing_rad: f32,
    /// How much room is in this direction; bigger is better. See type docs.
    pub clearance: f32,
    /// 0..=1 confidence in the clearance estimate.
    pub confidence: f32,
}

/// Clearance vs bearing for one frame — the sole input to local navigation.
/// Rays are ordered left (most +bearing) to right, matching image columns.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PolarClearance {
    pub rays: Vec<Ray>,
}

impl PolarClearance {
    pub fn n(&self) -> usize {
        self.rays.len()
    }
}
