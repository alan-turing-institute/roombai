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

/// Image geometry needed to place columns at bearings and read floor extent.
/// v1 needs the horizontal field of view (Camera Module v3 default ≈ 66°) and a
/// small-room horizon model (see [`drivability`]).
#[derive(Debug, Clone, Copy)]
pub struct ImageCal {
    pub hfov_rad: f32,
    /// Expected horizon height, as a fraction of frame height from the bottom.
    /// This robot only runs in small rooms, so the floor meets the far wall
    /// about halfway up the frame; floor reaching the horizon = a clear path.
    pub horizon_frac: f32,
    /// How far the floor boundary may sit *above* the horizon before the column
    /// is treated as fully blocked. Floor classified above the horizon can't be
    /// real floor — it's a near, floor-coloured obstacle filling the view — so
    /// drivability falls from its peak (at the horizon) to zero over this band.
    pub overshoot_tol: f32,
}

impl Default for ImageCal {
    fn default() -> Self {
        ImageCal { hfov_rad: 66.0_f32.to_radians(), horizon_frac: 0.5, overshoot_tol: 0.2 }
    }
}

/// Map a column's image-space floor fraction to a drivability clearance (0..=1),
/// peaked at the horizon.
///
/// In a small room the floor meets the far wall about halfway up the frame, so:
/// - **at the horizon** (`free_frac == horizon_frac`): floor is visible all the
///   way to the far wall → a clear path → clearance 1.0.
/// - **below the horizon**: less floor is visible because an obstacle/wall sits
///   closer than the far wall → clearance ramps 0→1 as the boundary rises to the
///   horizon.
/// - **above the horizon**: "floor" extending past the horizon is geometrically
///   impossible — it's a near, floor-coloured obstacle right in the bumper's
///   face being misread as floor — so clearance ramps back 1→0 over
///   `overshoot_tol` and the column is rejected. This is the case where the old
///   monotonic mapping wrongly read 70–90% "floor" as wide open.
pub fn drivability(free_frac: f32, horizon_frac: f32, overshoot_tol: f32) -> f32 {
    if free_frac <= horizon_frac {
        (free_frac / horizon_frac.max(1e-3)).clamp(0.0, 1.0)
    } else {
        (1.0 - (free_frac - horizon_frac) / overshoot_tol.max(1e-3)).clamp(0.0, 1.0)
    }
}

/// Map an image-space free-space profile to clearance-vs-bearing.
///
/// Column `i` of `n` is centred at horizontal image fraction `(i+0.5)/n`; its
/// bearing is `(0.5 - u) * hfov` so the left of the image is +bearing (left /
/// CCW), matching [`Twist`]'s sign. Clearance is the column's [`drivability`] —
/// floor extent interpreted against the small-room horizon, not raw `free_frac`.
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
                clearance: drivability(c.free_frac, cal.horizon_frac, cal.overshoot_tol),
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
            safe_clearance: 0.33,
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
    fn drivability_peaks_at_horizon_and_rejects_overshoot() {
        let (h, t) = (0.5, 0.2);
        // Floor reaching the horizon = a clear path to the far wall = max.
        assert!((drivability(0.5, h, t) - 1.0).abs() < 1e-6);
        // Floor "extending" well past the horizon is a near obstacle misread as
        // floor — the case the user flagged — and must be rejected (≈0, well
        // below any sane drive gate).
        assert!(drivability(0.7, h, t) < 1e-3);
        assert!(drivability(0.9, h, t) < 1e-3);
        // Below the horizon: less visible floor = less room, but still drivable.
        assert!(drivability(0.3, h, t) < drivability(0.5, h, t));
        assert!(drivability(0.3, h, t) > 0.0);
        // A column at the horizon out-clears one that overshoots it.
        assert!(drivability(0.5, h, t) > drivability(0.65, h, t));
    }

    #[test]
    fn polar_uses_horizon_drivability_not_raw_fraction() {
        // free_frac 0.5 (at horizon) should beat 0.9 (overshoot), the inverse of
        // the old monotonic mapping where 0.9 looked "most open".
        let fs = FreeSpace {
            columns: vec![
                Column { free_frac: 0.5, confidence: 1.0 },
                Column { free_frac: 0.9, confidence: 1.0 },
            ],
        };
        let pc = freespace_to_polar(&fs, &ImageCal::default());
        assert!(pc.rays[0].clearance > pc.rays[1].clearance);
        assert_eq!(pc.rays[1].clearance, 0.0);
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
