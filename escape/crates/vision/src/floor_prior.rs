//! Known-floor appearance prior — the "floor model bank" (SPEC §4.2), pooled
//! into one HSV histogram.
//!
//! It holds the colour distribution of pixels that are *known* to be floor
//! (extracted from the labeled S1 footage). The segmenter uses it to sanity-
//! check its per-frame adaptive seed: a patch that doesn't resemble any known
//! floor must not be adopted as the floor model. Pooling the floor classes
//! (light-blue carpet, dark-blue tile, grey hatch) into one multi-modal
//! histogram is enough for that gate; it can be split into per-class models
//! later without changing callers.
//!
//! Serializable so the prior can be baked offline into a calibration artifact
//! and shipped to the robot, where the test fixtures aren't present.

use serde::{Deserialize, Serialize};

use crate::hsv_bin;

/// A normalised HSV histogram of known-floor pixels.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FloorPrior {
    pub h_bins: usize,
    pub s_bins: usize,
    pub v_bins: usize,
    /// Per-bin likelihood in 0..=1 (bin count / max bin count).
    pub hist: Vec<f32>,
    /// Mean local texture energy (mean |∇| of grayscale) over known floor.
    /// Carpet/hatch are textured; a smooth wall/door is near-flat, so seed
    /// patches far below this are not floor even when the colour matches.
    pub floor_texture_mean: f32,
}

impl FloorPrior {
    /// How floor-like a colour is, 0..=1 (normalised backprojection).
    pub fn likelihood(&self, r: u8, g: u8, b: u8) -> f32 {
        self.hist[hsv_bin(r, g, b, self.h_bins, self.s_bins, self.v_bins)]
    }
}

/// Accumulates known-floor pixels, then [`finalize`](Self::finalize)s into a
/// normalised [`FloorPrior`]. Bin resolution should match the segmenter's.
pub struct FloorPriorBuilder {
    h_bins: usize,
    s_bins: usize,
    v_bins: usize,
    counts: Vec<u32>,
    texture_sum: f64,
    texture_n: u64,
}

impl FloorPriorBuilder {
    pub fn new(h_bins: usize, s_bins: usize, v_bins: usize) -> Self {
        FloorPriorBuilder {
            h_bins,
            s_bins,
            v_bins,
            counts: vec![0; h_bins * s_bins * v_bins],
            texture_sum: 0.0,
            texture_n: 0,
        }
    }

    /// Add a known-floor pixel's colour to the histogram.
    pub fn add(&mut self, r: u8, g: u8, b: u8) {
        self.counts[hsv_bin(r, g, b, self.h_bins, self.s_bins, self.v_bins)] += 1;
    }

    /// Add one known-floor texture sample (a local |∇| magnitude).
    pub fn add_texture(&mut self, grad: f32) {
        self.texture_sum += grad as f64;
        self.texture_n += 1;
    }

    pub fn finalize(self) -> FloorPrior {
        let max = (*self.counts.iter().max().unwrap_or(&1) as f32).max(1.0);
        FloorPrior {
            h_bins: self.h_bins,
            s_bins: self.s_bins,
            v_bins: self.v_bins,
            hist: self.counts.iter().map(|&c| c as f32 / max).collect(),
            floor_texture_mean: if self.texture_n > 0 {
                (self.texture_sum / self.texture_n as f64) as f32
            } else {
                0.0
            },
        }
    }
}

/// Luma (0..=255) for texture/gradient computations.
pub(crate) fn luma(r: u8, g: u8, b: u8) -> i32 {
    (r as i32 * 299 + g as i32 * 587 + b as i32 * 114) / 1000
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floor_colour_scores_high_unseen_colour_scores_zero() {
        let mut b = FloorPriorBuilder::new(18, 6, 4);
        // Seed with a blue-ish "carpet" colour.
        for _ in 0..100 {
            b.add(60, 90, 160);
        }
        let prior = b.finalize();
        assert!(prior.likelihood(60, 90, 160) > 0.9, "trained colour should score high");
        // A wood/tan wall colour was never added -> different bin -> ~0.
        assert_eq!(prior.likelihood(225, 205, 165), 0.0);
    }
}
