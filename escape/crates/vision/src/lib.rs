//! Vision: turn a single monocular frame into a description of the drivable
//! free space ahead of the robot.
//!
//! The public contract is [`FreeSpace`] — a per-column free-distance profile in
//! *image space* (the row where floor ends, per column, with a confidence).
//! Downstream (planner, mapper) depends only on this. Projecting it to
//! ground-plane rays in odometry units is a separate, later stage that needs
//! the S1 calibration; everything here is single-frame.
//!
//! Algorithm (Ulrich & Nourbakhsh lineage): seed a floor colour model from the
//! patch directly ahead of the robot (reliably floor), backproject an HSV
//! histogram to get a per-pixel floor likelihood, then scan each column up from
//! the bottom to the first sustained non-floor run. Parameters were tuned
//! against the hand-labeled fixtures in `tests/fixtures` (see
//! `tools/vision_label/prototype_seg.py`); mean abs per-column error ≈ 0.07.

pub mod fixture;

/// One column's view of how far the floor extends before the first obstacle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Column {
    /// Fraction of image height, measured from the bottom, that is floor before
    /// the first sustained non-floor run. 1.0 = floor to the top of the ROI
    /// (open); small = an obstacle is close.
    pub free_frac: f32,
    /// Confidence in this boundary, 0..=1 (how solidly the region below the
    /// boundary classified as floor).
    pub confidence: f32,
}

/// Per-column free-space profile for one frame, left column first.
#[derive(Debug, Clone, PartialEq)]
pub struct FreeSpace {
    pub columns: Vec<Column>,
}

impl FreeSpace {
    pub fn n_columns(&self) -> usize {
        self.columns.len()
    }
}

/// A loaded frame: row-major RGB8, `width * height * 3` bytes.
pub struct Frame<'a> {
    pub width: u32,
    pub height: u32,
    pub rgb: &'a [u8],
}

/// Tuning knobs for the segmenter (explicit so tests/tools can sweep them).
#[derive(Debug, Clone)]
pub struct Params {
    /// Number of output columns.
    pub n_columns: usize,
    /// Rectangle (x, y, w, h) to ignore, e.g. a burned-in overlay. Pixels here
    /// are excluded from the floor-fraction so they never create a boundary.
    pub ignore_rect: Option<(u32, u32, u32, u32)>,
    /// Seed patch (assumed floor): bottom `seed_rows` fraction of height,
    /// central `seed_width` fraction of width.
    pub seed_rows: f32,
    pub seed_width: f32,
    /// HSV histogram resolution.
    pub h_bins: usize,
    pub s_bins: usize,
    pub v_bins: usize,
    /// A pixel is floor if its normalised backprojected likelihood ≥ `density`.
    pub density: f32,
    /// A row is "floor" for a column if its floor fraction ≥ `row_floor_frac`.
    pub row_floor_frac: f32,
    /// Boundary = first run of `run` consecutive non-floor rows (scanning up).
    pub run: usize,
    /// Box-filter width applied to each column's per-row floor fraction.
    pub row_smooth: usize,
    /// Median-filter width applied across the output columns.
    pub col_smooth: usize,
    /// Whole-frame "filled with one thing" gate. If the frame's chroma spread
    /// ([`frame_chroma_spread`]) is below this, the view is a single uniform
    /// surface (wall / locker / door / bag in the bumper's face) with no
    /// navigable structure, so the frame is reported blocked instead of letting
    /// a near obstacle masquerade as open floor. `None` disables the gate.
    ///
    /// Replaces the old patch-based seed gate, which keyed off the bottom-centre
    /// patch against a static floor prior — fragile to lighting/white-balance
    /// drift. This gate needs no prior and no per-run calibration.
    pub block_chroma_spread_below: Option<f32>,
}

impl Default for Params {
    fn default() -> Self {
        // Locked from the prototype grid search against the fixtures.
        Params {
            n_columns: 24,
            ignore_rect: None,
            seed_rows: 0.15,
            seed_width: 0.70,
            h_bins: 18,
            s_bins: 6,
            v_bins: 4,
            density: 0.02,
            row_floor_frac: 0.55,
            run: 4,
            row_smooth: 5,
            col_smooth: 3,
            // The gate is self-contained (no data dependency), so it is on by
            // default. Threshold sits in the gap between the labeled blocked and
            // clear sets (see tests/uniformity_gate.rs).
            block_chroma_spread_below: Some(GATE_CHROMA_SPREAD),
        }
    }
}

/// Default chroma-spread threshold for the whole-frame block gate. Chosen to sit
/// in the gap between the labeled blocked frames (≤ ~9) and clear scenes (≥ ~12);
/// see `tests/uniformity_gate.rs`.
pub const GATE_CHROMA_SPREAD: f32 = 10.5;

/// Fixed grid the block gate works on, so the metric is resolution-independent
/// (the same for a 640×360 camera frame and a 4608×2592 phone photo).
const GATE_GRID_W: usize = 32;
const GATE_GRID_H: usize = 18;

/// Whole-frame colour-variety measure: the mean spread of per-cell chroma
/// `(R−G, G−B)` from the frame's mean chroma, over a fixed
/// [`GATE_GRID_W`]×[`GATE_GRID_H`] grid of block-averaged cells.
///
/// Low when one uniformly-coloured surface fills the view (a wall, locker, door,
/// or bag pressed against the bumper); high for a varied navigable scene (floor,
/// walls, and objects of different colours). It is brightness-independent — only
/// hue/colour direction counts — so it still flags a grey locker whose vents and
/// shadows vary in luma but not in colour.
pub fn frame_chroma_spread(frame: &Frame) -> f32 {
    let (w, h) = (frame.width as usize, frame.height as usize);
    let n_cells = GATE_GRID_W * GATE_GRID_H;
    let mut chroma = Vec::with_capacity(n_cells);
    for gy in 0..GATE_GRID_H {
        let y0 = gy * h / GATE_GRID_H;
        let y1 = ((gy + 1) * h / GATE_GRID_H).max(y0 + 1).min(h);
        for gx in 0..GATE_GRID_W {
            let x0 = gx * w / GATE_GRID_W;
            let x1 = ((gx + 1) * w / GATE_GRID_W).max(x0 + 1).min(w);
            let (mut sr, mut sg, mut sb, mut cnt) = (0u64, 0u64, 0u64, 0u64);
            for y in y0..y1 {
                for x in x0..x1 {
                    let i = (y * w + x) * 3;
                    sr += frame.rgb[i] as u64;
                    sg += frame.rgb[i + 1] as u64;
                    sb += frame.rgb[i + 2] as u64;
                    cnt += 1;
                }
            }
            let cnt = cnt.max(1) as f32;
            let (r, g, b) = (sr as f32 / cnt, sg as f32 / cnt, sb as f32 / cnt);
            chroma.push((r - g, g - b));
        }
    }
    let inv = 1.0 / chroma.len() as f32;
    let (mx, my) = chroma.iter().fold((0.0, 0.0), |(ax, ay), &(x, y)| (ax + x, ay + y));
    let (mx, my) = (mx * inv, my * inv);
    chroma.iter().map(|&(x, y)| ((x - mx).powi(2) + (y - my).powi(2)).sqrt()).sum::<f32>() * inv
}

/// Whether the frame should be gated as "filled with one uniform surface", per
/// [`Params::block_chroma_spread_below`].
pub fn is_frame_blocked(frame: &Frame, params: &Params) -> bool {
    match params.block_chroma_spread_below {
        Some(t) => frame_chroma_spread(frame) < t,
        None => false,
    }
}

/// RGB (0..=255) -> (hue 0..360, sat 0..1, val 0..1). Mirrors the prototype.
pub(crate) fn rgb_to_hsv(r: u8, g: u8, b: u8) -> (f32, f32, f32) {
    let (r, g, b) = (r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0);
    let mx = r.max(g).max(b);
    let mn = r.min(g).min(b);
    let df = mx - mn + 1e-9;
    let h = if mx == r {
        (60.0 * ((g - b) / df)).rem_euclid(360.0)
    } else if mx == g {
        60.0 * ((b - r) / df) + 120.0
    } else {
        60.0 * ((r - g) / df) + 240.0
    };
    (h, df / (mx + 1e-9), mx)
}

/// HSV-histogram bin for a pixel, given the bin resolution. Shared by the live
/// per-frame model ([`Params`]) and the [`FloorPrior`] so both index alike.
pub(crate) fn hsv_bin(r: u8, g: u8, b: u8, h_bins: usize, s_bins: usize, v_bins: usize) -> usize {
    let (h, s, v) = rgb_to_hsv(r, g, b);
    let hi = ((h / 360.0 * h_bins as f32) as usize).min(h_bins - 1);
    let si = ((s * s_bins as f32) as usize).min(s_bins - 1);
    let vi = ((v * v_bins as f32) as usize).min(v_bins - 1);
    (hi * s_bins + si) * v_bins + vi
}

fn bin_index(r: u8, g: u8, b: u8, p: &Params) -> usize {
    hsv_bin(r, g, b, p.h_bins, p.s_bins, p.v_bins)
}

/// Box filter (width `k`) over a slice, edge-clamped, returned as a new Vec.
fn box_smooth(a: &[f32], k: usize) -> Vec<f32> {
    if k <= 1 {
        return a.to_vec();
    }
    let n = a.len();
    let half = k / 2;
    (0..n)
        .map(|i| {
            let lo = i.saturating_sub(half);
            let hi = (i + half + 1).min(n);
            a[lo..hi].iter().sum::<f32>() / (hi - lo) as f32
        })
        .collect()
}

/// Median filter (width `k`) across columns, edge-clamped.
fn median_smooth(a: &[f32], k: usize) -> Vec<f32> {
    if k <= 1 {
        return a.to_vec();
    }
    let n = a.len() as isize;
    let half = (k / 2) as isize;
    (0..n)
        .map(|i| {
            let mut win: Vec<f32> = (-half..=half)
                .map(|d| a[(i + d).clamp(0, n - 1) as usize])
                .collect();
            win.sort_by(|x, y| x.partial_cmp(y).unwrap());
            win[win.len() / 2]
        })
        .collect()
}

pub fn segment_floor(frame: &Frame, params: &Params) -> FreeSpace {
    let (w, h) = (frame.width as usize, frame.height as usize);
    let n = params.n_columns;
    let px = |x: usize, y: usize| -> (u8, u8, u8) {
        let i = (y * w + x) * 3;
        (frame.rgb[i], frame.rgb[i + 1], frame.rgb[i + 2])
    };

    // 0. Whole-frame block gate: if one uniform surface fills the view (no
    //    navigable structure), report blocked before trying to find a boundary.
    if is_frame_blocked(frame, params) {
        return FreeSpace { columns: vec![Column { free_frac: 0.0, confidence: 1.0 }; n] };
    }

    // 1. Floor colour model from the seed patch directly ahead of the robot.
    let r0 = ((1.0 - params.seed_rows) * h as f32) as usize;
    let c0 = ((0.5 - params.seed_width / 2.0) * w as f32) as usize;
    let c1 = ((0.5 + params.seed_width / 2.0) * w as f32) as usize;
    let mut hist = vec![0u32; params.h_bins * params.s_bins * params.v_bins];
    for y in r0..h {
        for x in c0..c1 {
            let (r, g, b) = px(x, y);
            hist[bin_index(r, g, b, params)] += 1;
        }
    }
    let hmax = *hist.iter().max().unwrap_or(&1) as f32;

    // 2. Per-pixel floor mask via normalised backprojection >= density.
    //    ignore_rect pixels are forced to "floor" so they never form a boundary.
    let ignore = |x: usize, y: usize| match params.ignore_rect {
        Some((ix, iy, iw, ih)) => {
            let (ix, iy, iw, ih) = (ix as usize, iy as usize, iw as usize, ih as usize);
            x >= ix && x < ix + iw && y >= iy && y < iy + ih
        }
        None => false,
    };
    let is_floor = |x: usize, y: usize| -> bool {
        if ignore(x, y) {
            return true;
        }
        let (r, g, b) = px(x, y);
        hist[bin_index(r, g, b, params)] as f32 / hmax >= params.density
    };

    // 3. Per-column boundary: first sustained non-floor run scanning up.
    let mut free = vec![0f32; n];
    let mut conf = vec![0f32; n];
    for c in 0..n {
        let x0 = c * w / n;
        let x1 = ((c + 1) * w / n).max(x0 + 1);
        let frac: Vec<f32> = (0..h)
            .map(|y| {
                let f = (x0..x1).filter(|&x| is_floor(x, y)).count();
                f as f32 / (x1 - x0) as f32
            })
            .collect();
        let frac = box_smooth(&frac, params.row_smooth);

        let mut boundary = 0usize;
        let mut bad = 0usize;
        for y in (0..h).rev() {
            if frac[y] < params.row_floor_frac {
                bad += 1;
                if bad >= params.run {
                    boundary = y + params.run;
                    break;
                }
            } else {
                bad = 0;
            }
        }
        free[c] = (h - boundary) as f32 / h as f32;
        // confidence = mean floor fraction over the region called floor.
        conf[c] = if boundary < h {
            frac[boundary..h].iter().sum::<f32>() / (h - boundary) as f32
        } else {
            0.0
        };
    }

    // 4. Median-smooth the boundary across columns (matches the smooth labels).
    let free = median_smooth(&free, params.col_smooth);
    FreeSpace {
        columns: free
            .iter()
            .zip(&conf)
            .map(|(&f, &cf)| Column { free_frac: f, confidence: cf })
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a frame: bottom `floor_h` rows green, the rest red.
    fn split_frame(w: u32, h: u32, floor_h: u32) -> (Vec<u8>, u32, u32) {
        let mut rgb = vec![0u8; (w * h * 3) as usize];
        for y in 0..h {
            let floor = y >= h - floor_h;
            for x in 0..w {
                let i = ((y * w + x) * 3) as usize;
                if floor {
                    rgb[i + 1] = 180; // green
                } else {
                    rgb[i] = 180; // red
                }
            }
        }
        (rgb, w, h)
    }

    #[test]
    fn finds_horizontal_floor_boundary() {
        let (rgb, w, h) = split_frame(64, 48, 24); // bottom half floor
        let fs = segment_floor(&Frame { width: w, height: h, rgb: &rgb }, &Params::default());
        assert_eq!(fs.n_columns(), 24);
        for col in &fs.columns {
            assert!((col.free_frac - 0.5).abs() < 0.1, "free_frac {}", col.free_frac);
        }
    }

    #[test]
    fn all_floor_reports_open() {
        // A single-colour frame is intentionally treated as blocked by the
        // whole-frame gate (it's "filled with one thing"), so disable the gate
        // here to isolate the boundary scan on an all-floor frame.
        let (rgb, w, h) = split_frame(64, 48, 48); // entirely floor
        let params = Params { block_chroma_spread_below: None, ..Default::default() };
        let fs = segment_floor(&Frame { width: w, height: h, rgb: &rgb }, &params);
        for col in &fs.columns {
            assert!(col.free_frac > 0.9, "expected open, got {}", col.free_frac);
        }
    }

    #[test]
    fn box_and_median_helpers() {
        assert_eq!(box_smooth(&[1.0, 1.0, 1.0], 1), vec![1.0, 1.0, 1.0]);
        let m = median_smooth(&[0.5, 0.0, 0.5], 3); // spike at the middle removed
        assert_eq!(m[1], 0.5);
    }
}
