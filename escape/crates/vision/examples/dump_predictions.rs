//! Dump segment_floor predictions for every fixture as label-shaped JSON, so
//! the Python overlay tool can render the actual Rust output vs the human line.
//!
//!   cargo run -p vision --example dump_predictions -- /tmp/pred
//! then (homebrew python):
//!   render_compare.py  (see tools/vision_label)

use std::path::PathBuf;

use vision::fixture::load_dir;
use vision::{segment_floor, Frame, Params};

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| "/tmp/pred".into());
    std::fs::create_dir_all(&out).unwrap();
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let labels = load_dir(&root.join("labels")).expect("labels");
    for label in &labels {
        let img = image::open(root.join("images").join(&label.image))
            .unwrap()
            .to_rgb8();
        let (w, h) = img.dimensions();
        let raw = img.into_raw();
        let params = Params {
            n_columns: label.n_columns(),
            ignore_rect: label.ignore_rect.map(|[x, y, w, h]| (x, y, w, h)),
            ..Default::default()
        };
        let fs = segment_floor(&Frame { width: w, height: h, rgb: &raw }, &params);
        let cols: Vec<f32> = fs.columns.iter().map(|c| c.free_frac).collect();
        let json = format!(
            "{{\"image\":\"{}\",\"width\":{w},\"height\":{h},\"columns\":{:?}}}",
            label.image, cols
        );
        std::fs::write(PathBuf::from(&out).join(label.image.replace(".jpg", ".json")), json).unwrap();
    }
    println!("wrote {} predictions to {out}", labels.len());

    // Rough per-frame timing on the first fixture (decode excluded).
    if let Some(label) = labels.first() {
        let img = image::open(root.join("images").join(&label.image)).unwrap().to_rgb8();
        let (w, h) = img.dimensions();
        let raw = img.into_raw();
        let p = Params::default();
        let frame = Frame { width: w, height: h, rgb: &raw };
        let n = 200;
        let t = std::time::Instant::now();
        for _ in 0..n {
            std::hint::black_box(segment_floor(std::hint::black_box(&frame), &p));
        }
        let per = t.elapsed().as_secs_f64() * 1000.0 / n as f64;
        println!("segment_floor: {per:.2} ms/frame at {w}x{h} (this machine, release)");
    }
}
