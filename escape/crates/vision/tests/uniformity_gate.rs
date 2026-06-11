//! TDD for the whole-frame "filled with one thing" block gate.
//!
//! Ground truth is the labeled photo sets at the repo root:
//!   photos/          — clear, navigable scenes; MUST NOT be gated as blocked.
//!   photos/blocked/  — frame filled by one uniform surface (wall/locker/door/
//!                      bag); MUST be gated as blocked.
//! Add images to those folders to extend the spec — this test enforces it.

use std::path::PathBuf;

use vision::{frame_chroma_spread, Frame, Params};

fn photos_dir() -> PathBuf {
    // crates/vision -> repo root
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../photos")
}

fn jpgs(dir: &PathBuf) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().and_then(|x| x.to_str()).map(|x| x.eq_ignore_ascii_case("jpg")).unwrap_or(false))
        .collect();
    v.sort();
    v
}

/// Chroma spread of an image. Big inputs (e.g. 4608×2592 phone photos) are
/// thumbnailed first — the metric is resolution-independent and production
/// frames are 640×360, so this only saves time, it doesn't change the verdict.
fn spread(path: &PathBuf) -> f32 {
    let img = image::open(path).unwrap_or_else(|e| panic!("open {}: {e}", path.display()));
    let img = img.thumbnail(640, 360).to_rgb8();
    let (w, h) = img.dimensions();
    frame_chroma_spread(&Frame { width: w, height: h, rgb: &img.into_raw() })
}

#[test]
fn block_gate_separates_clear_from_blocked() {
    let clear = jpgs(&photos_dir());
    let blocked = jpgs(&photos_dir().join("blocked"));
    assert!(!clear.is_empty() && !blocked.is_empty(), "labeled photo sets must be present");

    let threshold = Params::default().block_chroma_spread_below.expect("gate on by default");
    let mut fails = Vec::new();
    let mut max_blocked = f32::MIN;
    let mut min_clear = f32::MAX;

    for p in &blocked {
        let s = spread(p);
        max_blocked = max_blocked.max(s);
        if s >= threshold {
            fails.push(format!("BLOCKED not gated: {} (spread {s:.2})", p.display()));
        }
    }
    for p in &clear {
        let s = spread(p);
        min_clear = min_clear.min(s);
        if s < threshold {
            fails.push(format!("CLEAR wrongly gated: {} (spread {s:.2})", p.display()));
        }
    }

    eprintln!(
        "chroma-spread: blocked max {max_blocked:.2} | clear min {min_clear:.2} \
         (threshold {threshold}); {} blocked, {} clear",
        blocked.len(), clear.len()
    );
    assert!(fails.is_empty(), "gate misclassifications:\n{}", fails.join("\n"));
    assert!(max_blocked < min_clear, "sets overlap: no separating threshold exists");
}
