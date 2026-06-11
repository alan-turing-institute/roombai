//! Shared, I/O-free types for the escape stack (SPEC §3.2 `core`).
//!
//! Conventions, fixed once here so every crate agrees:
//! - Angles in radians. **+angle is counter-clockwise (left)**, matching the
//!   wheel-encoder angle sign used in calibration.
//! - Body frame: +x forward, +y left. A `Twist` with `omega > 0` turns left.

use serde::{Deserialize, Serialize};

/// Body-frame velocity command, applied immediately by the driver (SPEC §3.1).
/// `Default` is [`Twist::STOP`] (all zero).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
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
/// `clearance` units are deliberately abstract: in v1 it is a 0..=1 drivability
/// derived from image-space floor extent against the small-room horizon (peaks
/// when floor reaches the horizon = a clear path; falls off when the boundary is
/// below it = a near obstacle, or above it = floor-coloured obstacle misread as
/// floor). Once the IPM calibration lands it becomes ground distance. The
/// planner only relies on "larger = more room", so it is unchanged by that
/// upgrade.
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

// ---- Sensing (SPEC §4.1) ----------------------------------------------------

/// Bumper / wheel-drop contact state, decoded from OI packet 7.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Contact {
    pub bump_left: bool,
    pub bump_right: bool,
    /// Any wheel or the caster has dropped (robot lifted / at an edge).
    pub wheel_drop: bool,
}

/// A latched safety event raised by the driver *below* the planner (SPEC §4.1).
///
/// The driver suppresses forward motion while a `Bump` is latched and only
/// clears it once the planner acknowledges by commanding a recovery (non-forward
/// `Twist`). `WheelDrop` is not latched — it gates all motion for exactly as long
/// as the drop is physically present.
///
/// Cliff sensing is intentionally not wired up yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Reflex {
    /// Bumper contact; which side(s) pressed.
    Bump { left: bool, right: bool },
    /// A wheel or the caster dropped — robot lifted or over an edge.
    WheelDrop,
}

/// One sample published by the driver every control tick (~20 Hz). This is the
/// sole sensing interface the mapper/planner consume; the wire-level OI packet
/// decoding never escapes the `driver` crate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SensorFrame {
    /// Seconds since the driver started its control loop.
    pub t_s: f64,
    /// Monotonic tick counter (gaps reveal dropped/late ticks).
    pub seq: u64,
    /// Integrated odometry pose in the start-anchored frame.
    pub pose: Pose2,
    /// True when `pose` came from the coarse integrated-distance/angle fallback
    /// (packets 19/20) rather than the wheel encoders (43/44) — widen
    /// downstream uncertainty accordingly.
    pub pose_degraded: bool,
    pub contact: Contact,
    /// Light-bump proximity signals (packets 46–51), left→right. Larger = closer
    /// obstacle. Proximity only — not a reflex; used by vision self-correction.
    pub light_bumps: [u16; 6],
    /// Battery charge in mAh (packet 25), if available.
    pub battery_charge_mah: Option<u16>,
    /// OI mode (packet 35): 0 off, 1 passive, 2 safe, 3 full. `None` if unread.
    pub oi_mode: Option<u8>,
    /// The reflex in force this tick, if any (see [`Reflex`]).
    pub reflex: Option<Reflex>,
}

/// The driver's public contract (SPEC §3.1). Implemented by the real serial
/// `driver`, the `sim`, and the `replay` harness so everything above it runs
/// unchanged off-robot.
///
/// Note: the SPEC sketch wrote `set_twist(v_mm_s, omega_rad_s)`; we pass the
/// shared [`Twist`] type instead. Both `&self` methods use interior mutability
/// so the planner can hold a shared handle.
pub trait RobotIo: Send + Sync {
    /// Set the commanded body velocity. Takes effect on the next control tick
    /// and persists until superseded or the watchdog zeroes it.
    fn set_twist(&self, twist: Twist);

    /// Open a new stream of sensor frames. Each call yields an independent
    /// receiver; the driver fans every frame out to all live subscribers.
    fn subscribe(&self) -> std::sync::mpsc::Receiver<SensorFrame>;
}
