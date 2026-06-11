//! Post-run analysis: replay a recorded `wander` run frame-by-frame with
//! overlays, so we can see exactly what vision detected and what the robot did.
//!
//! For each line in the run's `decisions.jsonl` we load the saved 640×360 frame,
//! re-run `vision::segment_floor` (deterministic, so it reproduces the boundary
//! the robot saw bit-for-bit), and draw: a green polyline for the floor boundary
//! (per-column free_frac); a clearance fan, one spoke per ray, green if drivable
//! else red; a decision arrow for the ACTUALLY-recorded action (blue heading for
//! Drive, yellow arc for Search — read from the log, not re-derived); and a top
//! state ribbon (green=Drive / yellow=Search).
//!
//! It also burns a per-frame numeric HUD into each frame (tiny embedded bitmap font,
//! so it needs no system fonts / ffmpeg text filters). ffmpeg then just stitches
//! the PNGs into a low-fps video for frame-stepping.
//!
//! Usage:
//!   cargo run -p planner --example replay_overlay -- --run ../wander1 --out /tmp/wander1_ov
//! then stitch (the tool prints the exact command):
//!   ffmpeg -y -framerate 6 -i /tmp/wander1_ov/%05d.png \
//!     -c:v libx264 -pix_fmt yuv420p /tmp/wander1_ov/overlay.mp4

// Throwaway drawing helpers take many positional args; not worth structs here.
#![allow(clippy::too_many_arguments)]

use std::path::{Path, PathBuf};

use escape_core::PolarClearance;
use planner::{freespace_to_polar, ImageCal, NavParams};
use vision::{segment_floor, Frame, Params};

const GREEN: [u8; 3] = [40, 230, 40];
const RED: [u8; 3] = [230, 40, 40];
const BLUE: [u8; 3] = [60, 120, 255];
const YELLOW: [u8; 3] = [255, 220, 0];
const FPS: u32 = 6;

/// One recorded decision (the fields `wander` logs per frame).
struct Decision {
    frame: String,
    state: String,
    v_mm_s: f64,
    omega_rad_s: f64,
    bearing_rad: Option<f32>,
    clearance: f32,
    t_s: f64,
}

fn main() {
    let mut run: Option<PathBuf> = None;
    let mut out = PathBuf::from("/tmp/wander_overlay");
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--run" => run = Some(PathBuf::from(it.next().expect("--run DIR"))),
            "--out" => out = PathBuf::from(it.next().expect("--out DIR")),
            other => panic!("unknown arg {other}"),
        }
    }
    let run = run.expect("--run DIR (a wander recording directory) is required");
    std::fs::create_dir_all(&out).unwrap();

    let cal = ImageCal::default();
    let nav = NavParams::default();
    // Default Params has the whole-frame block gate on, matching the live run.
    let base = Params::default();

    let decisions = load_decisions(&run.join("decisions.jsonl"));
    let mut out_idx = 0usize;
    let mut skipped = 0usize;
    let mut prev_t: Option<f64> = None;

    for d in &decisions {
        let img_path = run.join(&d.frame);
        let Ok(img) = image::open(&img_path) else {
            skipped += 1;
            continue;
        };
        let img = img.to_rgb8();
        let (w, h) = img.dimensions();
        let mut canvas = img.into_raw();

        let fs = segment_floor(&Frame { width: w, height: h, rgb: &canvas }, &base);
        let polar = freespace_to_polar(&fs, &cal);
        draw(&mut canvas, w, h, &fs, &polar, d, &nav);

        // A jump in t_s since the last recorded frame means the robot was busy
        // reversing/paused (a bump) — flag it on the HUD.
        let gap = prev_t.map(|p| d.t_s - p).unwrap_or(0.0);
        prev_t = Some(d.t_s);
        let fnum: String = d.frame.chars().filter(|c| c.is_ascii_digit()).collect();
        let fnum = fnum.trim_start_matches('0');
        let hud = format!(
            "F{} T{:.0} {} V{:.0} W{:+.1} C{:.2}{}",
            if fnum.is_empty() { "0" } else { fnum },
            d.t_s,
            d.state.to_uppercase(),
            d.v_mm_s,
            d.omega_rad_s,
            d.clearance,
            if gap > 1.2 { " BUMP" } else { "" },
        );
        draw_hud(&mut canvas, w, h, &hud);

        image::save_buffer(out.join(format!("{out_idx:05}.png")), &canvas, w, h, image::ColorType::Rgb8)
            .unwrap();
        out_idx += 1;
    }

    println!("rendered {out_idx} frames ({skipped} missing) to {}", out.display());
    println!(
        "stitch:\n  ffmpeg -y -framerate {FPS} -i {o}/%05d.png \
         -c:v libx264 -pix_fmt yuv420p {o}/overlay.mp4",
        o = out.display()
    );
}

fn load_decisions(path: &Path) -> Vec<Decision> {
    let text = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let v: serde_json::Value = serde_json::from_str(l).expect("parse decision line");
            Decision {
                frame: v["frame"].as_str().unwrap_or_default().to_string(),
                state: v["state"].as_str().unwrap_or("?").to_string(),
                v_mm_s: v["v_mm_s"].as_f64().unwrap_or(0.0),
                omega_rad_s: v["omega_rad_s"].as_f64().unwrap_or(0.0),
                bearing_rad: v["bearing_rad"].as_f64().map(|x| x as f32),
                clearance: v["clearance"].as_f64().unwrap_or(0.0) as f32,
                t_s: v["t_s"].as_f64().unwrap_or(0.0),
            }
        })
        .collect()
}

// ---- drawing ----------------------------------------------------------------

fn put(buf: &mut [u8], w: u32, h: u32, x: i32, y: i32, c: [u8; 3]) {
    if x < 0 || y < 0 || x >= w as i32 || y >= h as i32 {
        return;
    }
    let i = ((y as u32 * w + x as u32) * 3) as usize;
    buf[i] = c[0];
    buf[i + 1] = c[1];
    buf[i + 2] = c[2];
}

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

fn stroke(buf: &mut [u8], w: u32, h: u32, x0: i32, y0: i32, x1: i32, y1: i32, c: [u8; 3], r: i32) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                line(buf, w, h, x0 + dx, y0 + dy, x1 + dx, y1 + dy, c);
            }
        }
    }
}

fn disk(buf: &mut [u8], w: u32, h: u32, cx: i32, cy: i32, r: i32, c: [u8; 3]) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                put(buf, w, h, cx + dx, cy + dy, c);
            }
        }
    }
}

fn draw(
    buf: &mut [u8],
    w: u32,
    h: u32,
    fs: &vision::FreeSpace,
    polar: &PolarClearance,
    d: &Decision,
    p: &NavParams,
) {
    let n = fs.columns.len();
    let bx = |i: usize| ((i as f32 + 0.5) / n as f32 * w as f32) as i32;
    let by = |frac: f32| (h as f32 - frac * h as f32) as i32;
    for i in 1..n {
        stroke(
            buf, w, h,
            bx(i - 1), by(fs.columns[i - 1].free_frac),
            bx(i), by(fs.columns[i].free_frac),
            GREEN, 2,
        );
    }

    let (ox, oy) = ((w / 2) as i32, (h - 1) as i32);
    let max_len = 0.9 * h as f32;
    for r in &polar.rays {
        let len = r.clearance * max_len;
        let ex = ox + (-r.bearing_rad.sin() * len) as i32;
        let ey = oy + (-r.bearing_rad.cos() * len) as i32;
        let col = if r.clearance >= p.safe_clearance && r.confidence >= p.min_confidence {
            GREEN
        } else {
            RED
        };
        stroke(buf, w, h, ox, oy, ex, ey, col, 1);
    }

    // The recorded action.
    let is_drive = d.state.eq_ignore_ascii_case("drive");
    if is_drive {
        if let Some(b) = d.bearing_rad {
            let len = 0.85 * h as f32;
            let ex = ox + (-b.sin() * len) as i32;
            let ey = oy + (-b.cos() * len) as i32;
            stroke(buf, w, h, ox, oy, ex, ey, BLUE, 4);
        }
    } else {
        let dir = d.omega_rad_s.signum() as i32; // +1 left
        let len = (0.25 * w as f32) as i32;
        stroke(buf, w, h, ox, oy - 10, ox - dir * len, oy - 10, YELLOW, 4);
    }
    disk(buf, w, h, ox, oy, 6, BLUE);

    // State ribbon along the top.
    let ribbon = if is_drive { GREEN } else { YELLOW };
    for y in 0..6 {
        for x in 0..w as i32 {
            put(buf, w, h, x, y, ribbon);
        }
    }
}

// ---- HUD text via a tiny embedded 5×7 bitmap font ---------------------------

const SCALE: i32 = 2;

/// Draw the HUD string on a dark backdrop just under the top state ribbon.
fn draw_hud(buf: &mut [u8], w: u32, h: u32, text: &str) {
    let (x0, y0) = (4i32, 9i32);
    let cw = 6 * SCALE; // 5px glyph + 1px gap
    let bg_w = text.chars().count() as i32 * cw + 4;
    let bg_h = 7 * SCALE + 4;
    for y in (y0 - 2)..(y0 - 2 + bg_h) {
        for x in (x0 - 2)..(x0 - 2 + bg_w) {
            put(buf, w, h, x, y, [0, 0, 0]);
        }
    }
    let mut x = x0;
    for ch in text.chars() {
        draw_glyph(buf, w, h, x, y0, ch, [255, 255, 255]);
        x += cw;
    }
}

fn draw_glyph(buf: &mut [u8], w: u32, h: u32, x: i32, y: i32, ch: char, c: [u8; 3]) {
    let g = glyph(ch.to_ascii_uppercase());
    for (row, bits) in g.iter().enumerate() {
        for col in 0..5 {
            if bits & (0b10000 >> col) != 0 {
                for dy in 0..SCALE {
                    for dx in 0..SCALE {
                        put(buf, w, h, x + col * SCALE + dx, y + row as i32 * SCALE + dy, c);
                    }
                }
            }
        }
    }
}

/// 5×7 glyphs (one byte per row, low 5 bits, MSB = leftmost). Covers exactly the
/// characters the HUD emits; anything else renders blank.
fn glyph(ch: char) -> [u8; 7] {
    match ch {
        '0' => [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
        '1' => [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2' => [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
        '3' => [0b11111, 0b00010, 0b00100, 0b00010, 0b00001, 0b10001, 0b01110],
        '4' => [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        '5' => [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        '6' => [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        '7' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        '8' => [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        '9' => [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
        'A' => [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'B' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110],
        'C' => [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110],
        'D' => [0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110],
        'E' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111],
        'F' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000],
        'H' => [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'I' => [0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        'M' => [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001],
        'P' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000],
        'R' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
        'S' => [0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110],
        'T' => [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
        'U' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'V' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
        'W' => [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001],
        '.' => [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b00110],
        '-' => [0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000],
        '+' => [0b00000, 0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0b00000],
        _ => [0; 7], // space / unknown
    }
}
