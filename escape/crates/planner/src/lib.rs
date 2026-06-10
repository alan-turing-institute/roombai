//! Local navigation: turn one frame's free-space profile into a velocity command.
//!
//! This is the "steering seam" (SPEC §4.4 local navigation). Two pieces:
//!
//! 1. [`freespace_to_polar`] — adapt the vision crate's image-space
//!    [`vision::FreeSpace`] into a [`PolarClearance`] (clearance vs bearing).
//!    This is the ONLY spot that knows about image geometry; swapping the v1
//!    image-space mapping for the eventual IPM/ground-plane projection changes
//!    just this function, leaving the navigator untouched.
//! 2. [`Navigator`] — a VFH-style heading chooser. If a safe forward heading
//!    exists, drive toward the clearest one (biased to straight) at a speed that
//!    scales with clearance; otherwise spin in place toward the more open side
//!    until something safe opens up.
//!
//! Everything here is pure (no I/O, no clock) so it is developed and tuned
//! entirely offline: replay recorded frames through `vision::segment_floor`,
//! then through these functions, and overlay the decision (see the
//! `steer_overlay` example).

use escape_core::{PolarClearance, Ray, Twist};
use vision::FreeSpace;

/// Image geometry needed to place columns at bearings. v1 only needs the
/// horizontal field of view; the Camera Module v3 in its default mode is ~66°.
#[derive(Debug, Clone, Copy)]
pub struct ImageCal {
    pub hfov_rad: f32,
}

impl Default for ImageCal {
    fn default() -> Self {
        ImageCal { hfov_rad: 66.0_f32.to_radians() }
    }
}

/// Map an image-space free-space profile to clearance-vs-bearing.
///
/// Column `i` of `n` is centred at horizontal image fraction `(i+0.5)/n`; its
/// bearing is `(0.5 - u) * hfov` so the left of the image is +bearing (left /
/// CCW), matching [`Twist`]'s sign. Clearance is the column's `free_frac`
/// (image-space proxy) until IPM replaces it with ground distance.
pub fn freespace_to_polar(fs: &FreeSpace, cal: &ImageCal) -> PolarClearance {
    let n = fs.columns.len();
    let rays = fs
        .columns
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let u = (i as f32 + 0.5) / n as f32;
            Ray {
                bearing_rad: (0.5 - u) * cal.hfov_rad,
                clearance: c.free_frac,
                confidence: c.confidence,
            }
        })
        .collect();
    PolarClearance { rays }
}

/// Tuning knobs for the navigator. Clearance values are in the same units as
/// [`Ray::clearance`] (v1: image-fraction, 0..=1).
#[derive(Debug, Clone)]
pub struct NavParams {
    /// Minimum clearance for a bearing to count as drivable.
    pub safe_clearance: f32,
    /// Clearance at/above which forward speed saturates to `max_v_mm_s`.
    pub cruise_clearance: f32,
    /// A bearing must be at least this confident to be drivable.
    pub min_confidence: f32,
    /// Score penalty per radian of |bearing| — biases the choice toward straight.
    pub turn_penalty: f32,
    /// Forward speed ceiling, mm/s.
    pub max_v_mm_s: f64,
    /// Forward speed when barely drivable (creep), mm/s.
    pub creep_v_mm_s: f64,
    /// Yaw-rate gain: rad/s commanded per rad of heading error.
    pub kp_omega: f64,
    /// Yaw-rate ceiling while driving, rad/s.
    pub max_omega_rad_s: f64,
    /// Spin rate while searching (blocked), rad/s.
    pub search_omega_rad_s: f64,
}

impl Default for NavParams {
    fn default() -> Self {
        // First-guess values, to be tuned by eye on the steer_overlay output.
        NavParams {
            safe_clearance: 0.40,
            cruise_clearance: 0.80,
            min_confidence: 0.20,
            turn_penalty: 0.6,
            max_v_mm_s: 250.0,
            creep_v_mm_s: 80.0,
            kp_omega: 2.5,
            max_omega_rad_s: 1.2,
            search_omega_rad_s: 0.7,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavState {
    /// A safe heading was found; driving toward it.
    Drive,
    /// Nothing safe ahead; spinning in place to look for an opening.
    Search,
}

/// One navigation decision plus the context behind it (for logging/overlay).
#[derive(Debug, Clone, Copy)]
pub struct NavDecision {
    pub twist: Twist,
    pub state: NavState,
    /// Chosen heading (rad, +left), when driving.
    pub chosen_bearing: Option<f32>,
    /// Clearance of the chosen heading (Drive) or best available (Search).
    pub clearance: f32,
}

/// Best drivable heading by `score = clearance - turn_penalty*|bearing|`,
/// confidence-weighted, among rays clearing the safety/confidence gates.
/// Returns `(bearing, clearance)` or `None` if nothing is drivable.
fn best_heading(profile: &PolarClearance, p: &NavParams) -> Option<(f32, f32)> {
    profile
        .rays
        .iter()
        .filter(|r| r.clearance >= p.safe_clearance && r.confidence >= p.min_confidence)
        .map(|r| {
            let score = (r.clearance - p.turn_penalty * r.bearing_rad.abs()) * r.confidence;
            (r, score)
        })
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .map(|(r, _)| (r.bearing_rad, r.clearance))
}

/// Which side has more total clearance: +1 left, -1 right, 0 if balanced.
fn open_side(profile: &PolarClearance) -> f64 {
    let (mut left, mut right) = (0.0f32, 0.0f32);
    for r in &profile.rays {
        if r.bearing_rad > 0.0 {
            left += r.clearance;
        } else if r.bearing_rad < 0.0 {
            right += r.clearance;
        }
    }
    match left.partial_cmp(&right).unwrap() {
        std::cmp::Ordering::Greater => 1.0,
        std::cmp::Ordering::Less => -1.0,
        std::cmp::Ordering::Equal => 0.0,
    }
}

/// VFH-style local navigator. Holds a little state for search hysteresis so it
/// doesn't dither between left/right when blocked.
pub struct Navigator {
    pub params: NavParams,
    /// Direction to spin when searching: +1 left, -1 right.
    search_dir: f64,
    last_state: NavState,
}

impl Navigator {
    pub fn new(params: NavParams) -> Self {
        Navigator { params, search_dir: 1.0, last_state: NavState::Search }
    }

    /// Decide a velocity command from one frame's clearance profile.
    pub fn step(&mut self, profile: &PolarClearance) -> NavDecision {
        let p = &self.params;
        match best_heading(profile, p) {
            Some((bearing, clearance)) => {
                let omega =
                    (p.kp_omega * bearing as f64).clamp(-p.max_omega_rad_s, p.max_omega_rad_s);
                // Speed scales with clearance, and is throttled when the chosen
                // opening is far off-axis (turn to face it before accelerating).
                let span = (p.cruise_clearance - p.safe_clearance).max(1e-3);
                let cl = (((clearance - p.safe_clearance) / span) as f64).clamp(0.0, 1.0);
                let align = (bearing as f64).cos().max(0.0);
                let v = p.creep_v_mm_s + (p.max_v_mm_s - p.creep_v_mm_s) * cl * align;
                // Remember where the room is, in case the next frame is blocked.
                let s = open_side(profile);
                if s != 0.0 {
                    self.search_dir = s;
                }
                self.last_state = NavState::Drive;
                NavDecision {
                    twist: Twist { v_mm_s: v, omega_rad_s: omega },
                    state: NavState::Drive,
                    chosen_bearing: Some(bearing),
                    clearance,
                }
            }
            None => {
                // Pick a search direction at the moment of blocking, then hold
                // it (hysteresis) until something drivable appears.
                if self.last_state == NavState::Drive {
                    let s = open_side(profile);
                    if s != 0.0 {
                        self.search_dir = s;
                    }
                }
                self.last_state = NavState::Search;
                let best = profile.rays.iter().map(|r| r.clearance).fold(0.0, f32::max);
                NavDecision {
                    twist: Twist::spin(self.search_dir * p.search_omega_rad_s),
                    state: NavState::Search,
                    chosen_bearing: None,
                    clearance: best,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use escape_core::PolarClearance;
    use vision::{Column, FreeSpace};

    fn profile(rays: &[(f32, f32)]) -> PolarClearance {
        PolarClearance {
            rays: rays
                .iter()
                .map(|&(b, c)| Ray { bearing_rad: b, clearance: c, confidence: 1.0 })
                .collect(),
        }
    }

    #[test]
    fn columns_map_left_positive_and_symmetric() {
        let fs = FreeSpace {
            columns: vec![Column { free_frac: 0.9, confidence: 1.0 }; 4],
        };
        let pc = freespace_to_polar(&fs, &ImageCal::default());
        assert_eq!(pc.n(), 4);
        // Leftmost column -> most positive bearing; rightmost -> most negative.
        assert!(pc.rays[0].bearing_rad > 0.0);
        assert!(pc.rays[3].bearing_rad < 0.0);
        // Symmetric about straight-ahead.
        assert!((pc.rays[0].bearing_rad + pc.rays[3].bearing_rad).abs() < 1e-6);
    }

    #[test]
    fn drives_straight_when_open_ahead() {
        let mut nav = Navigator::new(NavParams::default());
        let d = nav.step(&profile(&[(0.3, 0.9), (0.0, 0.95), (-0.3, 0.9)]));
        assert_eq!(d.state, NavState::Drive);
        // Straight is favoured and near-full speed since clearance is high.
        assert!(d.chosen_bearing.unwrap().abs() < 0.05);
        assert!(d.twist.omega_rad_s.abs() < 0.1);
        assert!(d.twist.v_mm_s > 200.0);
    }

    #[test]
    fn steers_toward_the_open_side() {
        let mut nav = Navigator::new(NavParams::default());
        // Left blocked, right open.
        let d = nav.step(&profile(&[(0.3, 0.1), (0.0, 0.2), (-0.3, 0.9)]));
        assert_eq!(d.state, NavState::Drive);
        assert!(d.chosen_bearing.unwrap() < 0.0, "should pick the right opening");
        assert!(d.twist.omega_rad_s < 0.0, "omega should turn right");
    }

    #[test]
    fn searches_when_blocked_and_holds_direction() {
        let mut nav = Navigator::new(NavParams::default());
        // Everything below safe_clearance, but the left half is relatively more open.
        let blocked = profile(&[(0.3, 0.25), (0.0, 0.15), (-0.3, 0.05)]);
        let d = nav.step(&blocked);
        assert_eq!(d.state, NavState::Search);
        assert_eq!(d.twist.v_mm_s, 0.0);
        assert!(d.twist.omega_rad_s > 0.0, "should spin left toward the open side");
        // Hysteresis: a later frame whose open side flips shouldn't flip us.
        let dir1 = d.twist.omega_rad_s.signum();
        let flipped = profile(&[(0.3, 0.05), (0.0, 0.15), (-0.3, 0.25)]);
        let d2 = nav.step(&flipped);
        assert_eq!(d2.twist.omega_rad_s.signum(), dir1, "search direction should hold");
    }
}
