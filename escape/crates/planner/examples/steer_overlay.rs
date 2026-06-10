//! Offline visualization of the steering seam: run real frames through
//! `vision::segment_floor` -> `freespace_to_polar` -> `Navigator::step`, and
//! draw the result so we can watch the robot decide before any wheels turn.
//!
//! Each output frame is annotated with:
//!   - green polyline  : the floor boundary (per-column free_frac)
//!   - spokes from the bottom-centre "robot": clearance fan, one per ray,
//!     green if drivable, red if below the safe threshold
//!   - thick blue arrow: the chosen heading (Drive), or
//!     yellow arc arrow: spin direction (Search)
//! A summary line per frame (state, v, omega, bearing) is printed to stdout.
//!
//! Usage:
//!   # default: the labeled vision fixtures (real recorded frames)
//!   cargo run -p planner --example steer_overlay -- --out /tmp/steer
//!   # or any directory of jpg/png frames (e.g. ffmpeg-extracted s1 video):
//!   cargo run -p planner --example steer_overlay -- --images /tmp/s1_frames --out /tmp/steer

use std::path::{Path, PathBuf};

use escape_core::PolarClearance;
use planner::{freespace_to_polar, ImageCal, NavDecision, NavParams, NavState, Navigator};
use vision::fixture::load_dir;
use vision::{segment_floor, Frame, Params};

const GREEN: [u8; 3] = [40, 230, 40];
const RED: [u8; 3] = [230, 40, 40];
const BLUE: [u8; 3] = [60, 120, 255];
const YELLOW: [u8; 3] = [255, 220, 0];

/// One frame to process: pixels plus the overlay rect to mask and column count.
struct Job {
    name: String,
    rgb: Vec<u8>,
    w: u32,
    h: u32,
    ignore_rect: Option<(u32, u32, u32, u32)>,
    n_columns: usize,
}

fn main() {
    let mut images: Option<PathBuf> = None;
    let mut out = PathBuf::from("/tmp/steer");
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--images" => images = Some(PathBuf::from(it.next().expect("--images DIR"))),
            "--out" => out = PathBuf::from(it.next().expect("--out DIR")),
            other => panic!("unknown arg {other}"),
        }
    }
    std::fs::create_dir_all(&out).unwrap();

    let jobs = match &images {
        Some(dir) => load_image_dir(dir),
        None => load_fixtures(),
    };

    // Known-floor prior (color + texture) for the seed gate, built from the
    // labeled fixtures — the same gate the production pipeline runs with.
    let prior = vision::fixture::build_floor_prior(&fixtures_root()).expect("build floor prior");
    let cal = ImageCal::default();
    let nav_params = NavParams::default();
    // One navigator across the sequence so search hysteresis is exercised.
    let mut nav = Navigator::new(nav_params.clone());

    println!("{:<22} {:<7} {:>7} {:>8} {:>8}", "frame", "state", "v_mm_s", "omega", "bearing");
    for job in &jobs {
        let params = Params {
            n_columns: job.n_columns,
            ignore_rect: job.ignore_rect,
            floor_prior: Some(prior.clone()),
            ..Default::default()
        };
        let fs = segment_floor(&Frame { width: job.w, height: job.h, rgb: &job.rgb }, &params);
        let polar = freespace_to_polar(&fs, &cal);
        let decision = nav.step(&polar);

        let mut canvas = job.rgb.clone();
        draw_overlay(&mut canvas, job.w, job.h, &fs, &polar, &decision, &nav_params);

        let stem = Path::new(&job.name).file_stem().unwrap().to_string_lossy();
        let path = out.join(format!("{stem}_steer.png"));
        image::save_buffer(&path, &canvas, job.w, job.h, image::ColorType::Rgb8).unwrap();

        let bearing = decision.chosen_bearing.map(|b| b.to_degrees());
        println!(
            "{:<22} {:<7} {:>7.0} {:>8.2} {:>8}",
            job.name,
            match decision.state {
                NavState::Drive => "DRIVE",
                NavState::Search => "SEARCH",
            },
            decision.twist.v_mm_s,
            decision.twist.omega_rad_s,
            bearing.map(|d| format!("{d:+.1}")).unwrap_or_else(|| "  --".into()),
        );
    }
    println!("\nwrote {} overlays to {}", jobs.len(), out.display());
}

fn fixtures_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../vision/tests/fixtures")
}

fn load_fixtures() -> Vec<Job> {
    let root = fixtures_root();
    let labels = load_dir(&root.join("labels")).expect("load fixture labels");
    labels
        .iter()
        .map(|l| {
            let img = image::open(root.join("images").join(&l.image)).unwrap().to_rgb8();
            let (w, h) = img.dimensions();
            Job {
                name: l.image.clone(),
                rgb: img.into_raw(),
                w,
                h,
                ignore_rect: l.ignore_rect.map(|[x, y, w, h]| (x, y, w, h)),
                n_columns: l.n_columns(),
            }
        })
        .collect()
}

fn load_image_dir(dir: &Path) -> Vec<Job> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            matches!(
                p.extension().and_then(|x| x.to_str()),
                Some("jpg") | Some("jpeg") | Some("png")
            )
        })
        .collect();
    paths.sort();
    paths
        .iter()
        .map(|p| {
            let img = image::open(p).unwrap().to_rgb8();
            let (w, h) = img.dimensions();
            Job {
                name: p.file_name().unwrap().to_string_lossy().into_owned(),
                rgb: img.into_raw(),
                w,
                h,
                ignore_rect: None,
                n_columns: Params::default().n_columns,
            }
        })
        .collect()
}

// ---- drawing (no font deps: shapes only; text goes to stdout) ----------------

fn put(buf: &mut [u8], w: u32, h: u32, x: i32, y: i32, c: [u8; 3]) {
    if x < 0 || y < 0 || x >= w as i32 || y >= h as i32 {
        return;
    }
    let i = ((y as u32 * w + x as u32) * 3) as usize;
    buf[i] = c[0];
    buf[i + 1] = c[1];
    buf[i + 2] = c[2];
}

/// Bresenham line.
fn line(buf: &mut [u8], w: u32, h: u32, mut x0: i32, mut y0: i32, x1: i32, y1: i32, c: [u8; 3]) {
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        put(buf, w, h, x0, y0, c);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

/// Stroke a line with a round nib of the given radius (0 = 1px). Orientation-
/// independent thickness, so diagonals read as boldly as verticals.
fn stroke(buf: &mut [u8], w: u32, h: u32, x0: i32, y0: i32, x1: i32, y1: i32, c: [u8; 3], r: i32) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                line(buf, w, h, x0 + dx, y0 + dy, x1 + dx, y1 + dy, c);
            }
        }
    }
}

/// Filled disk, for marking the robot origin.
fn disk(buf: &mut [u8], w: u32, h: u32, cx: i32, cy: i32, r: i32, c: [u8; 3]) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                put(buf, w, h, cx + dx, cy + dy, c);
            }
        }
    }
}

fn draw_overlay(
    buf: &mut [u8],
    w: u32,
    h: u32,
    fs: &vision::FreeSpace,
    polar: &PolarClearance,
    d: &NavDecision,
    p: &NavParams,
) {
    let n = fs.columns.len();
    // Floor boundary polyline: connect per-column boundary midpoints.
    let bx = |i: usize| ((i as f32 + 0.5) / n as f32 * w as f32) as i32;
    let by = |frac: f32| (h as f32 - frac * h as f32) as i32;
    for i in 1..n {
        stroke(
            buf, w, h,
            bx(i - 1), by(fs.columns[i - 1].free_frac),
            bx(i), by(fs.columns[i].free_frac),
            GREEN, 3,
        );
    }

    // Clearance fan from the robot origin (bottom-centre).
    let (ox, oy) = ((w / 2) as i32, (h - 1) as i32);
    let max_len = 0.9 * h as f32;
    for r in &polar.rays {
        let len = r.clearance * max_len;
        // +bearing = left = -x; up = -y.
        let ex = ox + (-r.bearing_rad.sin() * len) as i32;
        let ey = oy + (-r.bearing_rad.cos() * len) as i32;
        let col = if r.clearance >= p.safe_clearance && r.confidence >= p.min_confidence {
            GREEN
        } else {
            RED
        };
        stroke(buf, w, h, ox, oy, ex, ey, col, 2);
    }

    // Chosen heading (Drive) or search-spin indicator (Search).
    match d.state {
        NavState::Drive => {
            if let Some(b) = d.chosen_bearing {
                let len = 0.85 * h as f32;
                let ex = ox + (-b.sin() * len) as i32;
                let ey = oy + (-b.cos() * len) as i32;
                stroke(buf, w, h, ox, oy, ex, ey, BLUE, 5);
            }
        }
        NavState::Search => {
            // Short arrow at the bottom pointing the way we're spinning.
            let dir = d.twist.omega_rad_s.signum() as i32; // +1 left
            let len = (0.25 * w as f32) as i32;
            stroke(buf, w, h, ox, oy - 10, ox - dir * len, oy - 10, YELLOW, 5);
        }
    }
    disk(buf, w, h, ox, oy, 7, BLUE);
}
