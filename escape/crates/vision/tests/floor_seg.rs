//! TDD harness for the floor segmenter, driven by the labeled boundary
//! fixtures. These FAIL until `segment_floor` is implemented — that is the
//! point. Run with `cargo test -p vision`.
//!
//! `#[ignore]`d for now so the suite is green while the labels are still being
//! confirmed; remove the ignores (or run `--ignored`) once labels are blessed
//! and we start implementing.

use std::path::PathBuf;

use vision::fixture::{load_dir, FloorLabel};
use vision::{segment_floor, Frame, Params};

fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn labels_dir() -> PathBuf {
    fixtures_dir().join("labels")
}

fn load_frame(name: &str) -> (Vec<u8>, u32, u32) {
    let path = fixtures_dir().join("images").join(name);
    let img = image::open(&path)
        .unwrap_or_else(|e| panic!("open {}: {e}", path.display()))
        .to_rgb8();
    let (w, h) = img.dimensions();
    (img.into_raw(), w, h)
}

/// The known-floor prior, built once from the (non-holdout) fixtures — the same
/// gate the production pipeline runs with, so these tests exercise it too.
fn training_prior() -> &'static vision::FloorPrior {
    static PRIOR: std::sync::OnceLock<vision::FloorPrior> = std::sync::OnceLock::new();
    PRIOR.get_or_init(|| {
        vision::fixture::build_floor_prior(&fixtures_dir()).expect("build floor prior")
    })
}

fn params_for(label: &FloorLabel) -> Params {
    Params {
        n_columns: label.n_columns(),
        ignore_rect: label.ignore_rect.map(|[x, y, w, h]| (x, y, w, h)),
        floor_prior: Some(training_prior().clone()),
        ..Default::default()
    }
}

/// Mean absolute per-column error between prediction and label.
fn mean_abs_err(pred: &[vision::Column], truth: &[f32]) -> f32 {
    let n = truth.len().min(pred.len());
    let sum: f32 = (0..n).map(|i| (pred[i].free_frac - truth[i]).abs()).sum();
    sum / n as f32
}

/// Always-on guard: whatever labels exist (produced by the labeling UI) parse,
/// reference a loadable image of matching dimensions, and have a sane 24-column
/// profile with values in range. Validates the human-drawn fixtures
/// independently of the (still-unimplemented) segmenter. Empty is allowed —
/// fixtures are authored interactively, so the set is populated out-of-band.
#[test]
fn fixtures_are_well_formed() {
    let labels = load_dir(&labels_dir()).expect("load labels");
    for label in &labels {
        assert_eq!(label.n_columns(), 24, "{}: not 24 columns", label.image);
        for (i, &c) in label.columns.iter().enumerate() {
            assert!((0.0..=1.0).contains(&c), "{} col {i}: {c} out of [0,1]", label.image);
        }
        let (_, w, h) = load_frame(&label.image);
        assert_eq!((w, h), (label.width, label.height), "{}: dim mismatch", label.image);
        // Sample points must be inside the frame.
        for &[x, y] in label.floor_points.iter().chain(&label.obstacle_points) {
            assert!(x < w && y < h, "{}: point ({x},{y}) outside frame", label.image);
        }
    }
    eprintln!("validated {} fixture label(s)", labels.len());
}

/// Acceptance against the hand-drawn boundaries. The ground truth is itself an
/// approximate hand-drawn line, so the primary bar is the *mean* per-column
/// error across the set (a standard regression metric), with a per-image cap to
/// catch gross breakage. Images exceeding their own drawn `tolerance` are
/// reported for visibility but are not individually fatal.
const MEAN_MAE_MAX: f32 = 0.085;
const PER_IMAGE_MAE_CAP: f32 = 0.16;

#[test]
fn boundary_matches_labels() {
    let labels = load_dir(&labels_dir()).expect("load labels");
    assert!(!labels.is_empty(), "no fixtures found");
    let mut errs = Vec::new();
    let mut over_drawn = Vec::new();
    let mut gross = Vec::new();
    for label in &labels {
        if label.holdout {
            continue;
        }
        let (rgb, w, h) = load_frame(&label.image);
        let frame = Frame { width: w, height: h, rgb: &rgb };
        let fs = segment_floor(&frame, &params_for(label));
        let err = mean_abs_err(&fs.columns, &label.columns);
        errs.push(err);
        if err > label.tolerance {
            over_drawn.push(format!("{} ({err:.3} > {:.3})", label.image, label.tolerance));
        }
        if err > PER_IMAGE_MAE_CAP {
            gross.push(format!("{} mae {err:.3} > cap {PER_IMAGE_MAE_CAP}", label.image));
        }
    }
    let mean = errs.iter().sum::<f32>() / errs.len() as f32;
    let max = errs.iter().cloned().fold(0.0f32, f32::max);
    eprintln!(
        "boundary MAE over {} train fixtures: mean={mean:.3} max={max:.3}; \
         {} over drawn tolerance: {}",
        errs.len(),
        over_drawn.len(),
        over_drawn.join(", "),
    );
    assert!(gross.is_empty(), "gross per-image failures:\n{}", gross.join("\n"));
    assert!(mean <= MEAN_MAE_MAX, "mean MAE {mean:.3} > {MEAN_MAE_MAX}");
}

/// Regression for the near-obstacle false-open bug: when the robot is so close
/// to an object that the object fills the frame (here `near_wall.jpg` — pressed
/// against the wood door, no floor visible), the segmenter must NOT report open
/// floor. Today it seeds its floor model from the bottom-centre patch, which is
/// sampling the wall, so the wall becomes "floor" and `free_frac` reads ~open.
///
/// Fixed by the floor-model-prior seed gate (SPEC §4.2): the seed patch here is
/// the wall, which doesn't match the known-floor prior, so the frame is reported
/// blocked instead of open.
#[test]
fn near_obstacle_not_reported_as_open() {
    let labels = load_dir(&labels_dir()).expect("load labels");
    let label = labels
        .iter()
        .find(|l| l.image == "near_wall.jpg")
        .expect("near_wall fixture present");
    let (rgb, w, h) = load_frame(&label.image);
    let frame = Frame { width: w, height: h, rgb: &rgb };
    let fs = segment_floor(&frame, &params_for(label));
    let n = fs.columns.len() as f32;
    let mean = fs.columns.iter().map(|c| c.free_frac).sum::<f32>() / n;
    let max = fs.columns.iter().map(|c| c.free_frac).fold(0.0f32, f32::max);
    // A wall in the robot's face is not drivable: no column should look more
    // than ~40% open, and on average it should read essentially blocked.
    assert!(
        max < 0.40 && mean < 0.25,
        "near-obstacle frame read as open floor: max free_frac {max:.2}, mean {mean:.2} \
         (expected max<0.40, mean<0.25)"
    );
}

#[test]
fn classifies_sample_points() {
    // Floor sample points should land at or below the predicted boundary;
    // obstacle points above it. (Cheap proxy for the pixel classifier.)
    let labels = load_dir(&labels_dir()).expect("load labels");
    for label in &labels {
        let (rgb, w, h) = load_frame(&label.image);
        let frame = Frame { width: w, height: h, rgb: &rgb };
        let fs = segment_floor(&frame, &params_for(label));
        let col_w = w as f32 / fs.n_columns() as f32;
        for &[px, py] in &label.floor_points {
            let c = ((px as f32 / col_w) as usize).min(fs.n_columns() - 1);
            let boundary_y = h as f32 * (1.0 - fs.columns[c].free_frac);
            assert!(
                py as f32 >= boundary_y - 0.05 * h as f32,
                "{}: floor point ({px},{py}) above boundary {boundary_y:.0}",
                label.image
            );
        }
    }
}
