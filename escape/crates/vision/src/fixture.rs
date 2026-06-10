//! Labeled test fixtures: ground-truth free-space boundaries for the recorded
//! photos, used to TDD the floor segmenter.
//!
//! Labeling scheme (per the design discussion):
//! - `columns`: free-distance ground truth, one entry per column, left first,
//!   each the fraction of image height (from the bottom) that is floor. This is
//!   exactly the [`crate::FreeSpace`] contract, so tests compare like-for-like.
//! - `floor_points` / `obstacle_points`: a few unambiguous (x, y) pixel samples
//!   the pixel classifier must get right, in image coordinates (origin top-left).
//! - `ignore_rect`: the burned-in timestamp overlay region to mask.
//! - `tolerance`: allowed per-column |free_frac| error for a pass.
//! - `notes`: free-text context (e.g. "within-floor colour seam at col 10").

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FloorLabel {
    pub image: String,
    pub width: u32,
    pub height: u32,
    /// Per-column floor fraction from the bottom (left column first).
    pub columns: Vec<f32>,
    #[serde(default)]
    pub floor_points: Vec<[u32; 2]>,
    #[serde(default)]
    pub obstacle_points: Vec<[u32; 2]>,
    #[serde(default)]
    pub ignore_rect: Option<[u32; 4]>,
    #[serde(default = "default_tolerance")]
    pub tolerance: f32,
    #[serde(default)]
    pub notes: String,
    /// Held out of algorithm tuning to guard against overfitting.
    #[serde(default)]
    pub holdout: bool,
}

fn default_tolerance() -> f32 {
    0.12
}

impl FloorLabel {
    pub fn n_columns(&self) -> usize {
        self.columns.len()
    }
}

/// Build a [`FloorPrior`](crate::FloorPrior) from the labeled fixtures: for each
/// non-holdout label, the pixels below the drawn boundary (per column) are known
/// floor, so they're accumulated into the prior histogram. This is the offline
/// "known floor from S1 footage" the seed gate checks against.
///
/// `fixtures_dir` must contain `labels/` and `images/` subdirectories.
pub fn build_floor_prior(fixtures_dir: &std::path::Path) -> std::io::Result<crate::FloorPrior> {
    let p = crate::Params::default();
    let mut builder = crate::FloorPriorBuilder::new(p.h_bins, p.s_bins, p.v_bins);
    for label in load_dir(&fixtures_dir.join("labels"))? {
        if label.holdout {
            continue;
        }
        let img = image::open(fixtures_dir.join("images").join(&label.image))
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e.to_string()))?
            .to_rgb8();
        let (w, h) = img.dimensions();
        let n = label.columns.len();
        for (c, &frac) in label.columns.iter().enumerate() {
            let x0 = c as u32 * w / n as u32;
            let x1 = (((c + 1) as u32 * w) / n as u32).max(x0 + 1);
            // Floor is everything from the boundary row down to the bottom.
            let boundary = ((1.0 - frac) * h as f32) as u32;
            for y in boundary..h {
                for x in x0..x1 {
                    let px = img.get_pixel(x, y);
                    builder.add(px[0], px[1], px[2]);
                    if x + 1 < w && y + 1 < h {
                        let g0 = crate::floor_prior::luma(px[0], px[1], px[2]);
                        let gr = img.get_pixel(x + 1, y);
                        let gd = img.get_pixel(x, y + 1);
                        let grad = (crate::floor_prior::luma(gr[0], gr[1], gr[2]) - g0).abs()
                            + (crate::floor_prior::luma(gd[0], gd[1], gd[2]) - g0).abs();
                        builder.add_texture(grad as f32);
                    }
                }
            }
        }
    }
    Ok(builder.finalize())
}

/// Load all `*.json` labels from a directory, sorted by filename.
pub fn load_dir(dir: &std::path::Path) -> std::io::Result<Vec<FloorLabel>> {
    let mut entries: Vec<_> = std::fs::read_dir(dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().map(|x| x == "json").unwrap_or(false))
        .collect();
    entries.sort();
    let mut out = Vec::new();
    for p in entries {
        let text = std::fs::read_to_string(&p)?;
        let label: FloorLabel = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("parse {}: {e}", p.display()));
        out.push(label);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips() {
        let l = FloorLabel {
            image: "img06.jpg".into(),
            width: 640,
            height: 360,
            columns: vec![1.0, 0.8, 0.3],
            floor_points: vec![[320, 340]],
            obstacle_points: vec![[300, 100]],
            ignore_rect: Some([0, 0, 120, 40]),
            tolerance: 0.12,
            notes: "example".into(),
            holdout: false,
        };
        let s = serde_json::to_string(&l).unwrap();
        let back: FloorLabel = serde_json::from_str(&s).unwrap();
        assert_eq!(back.columns, l.columns);
        assert_eq!(back.n_columns(), 3);
    }

    #[test]
    fn parses_minimal_label() {
        // Only the required fields; the rest default.
        let s = r#"{"image":"x.jpg","width":640,"height":360,"columns":[1.0,0.5]}"#;
        let l: FloorLabel = serde_json::from_str(s).unwrap();
        assert_eq!(l.tolerance, 0.12);
        assert!(!l.holdout);
        assert!(l.ignore_rect.is_none());
    }
}
