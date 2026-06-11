//! Live camera ingest: an `rpicam-vid` subprocess piping raw YUV420 frames, plus
//! conversion to the RGB working resolution the vision crate expects.
//!
//! We capture at 1280×720 (tightly packed for raw YUV — multiples of 32×16, so
//! no stride-padding surprises) and 2× downsample to exactly 640×360, the fixed
//! resolution the floor prior was tuned at (memory: segment at 640×360). The
//! I420 chroma planes at 1280×720 are already 640×360, so the colour conversion
//! lands on the output grid with no chroma resampling.
//!
//! A reader thread keeps only the most recent frame, so a slow consumer never
//! falls behind a backed-up pipe — the loop always steers on the freshest view.

use std::io::{self, Read};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};

/// Capture resolution (well-aligned for raw YUV420).
pub const CAP_W: usize = 1280;
pub const CAP_H: usize = 720;
/// Vision working resolution (matches the floor prior).
pub const OUT_W: u32 = 640;
pub const OUT_H: u32 = 360;

const Y_SIZE: usize = CAP_W * CAP_H;
const C_W: usize = CAP_W / 2; // = 640, the chroma plane width
const C_SIZE: usize = (CAP_W / 2) * (CAP_H / 2); // = 640×360
const CAP_FRAME_BYTES: usize = Y_SIZE + 2 * C_SIZE;

pub struct Camera {
    child: Child,
    latest: Arc<Mutex<Option<Vec<u8>>>>,
    reader: Option<JoinHandle<()>>,
}

impl Camera {
    /// Spawn `rpicam-vid` streaming raw YUV420 to stdout at `fps`, and start the
    /// latest-frame reader thread.
    pub fn start(fps: u32) -> io::Result<Camera> {
        let mut child = Command::new("rpicam-vid")
            .args([
                "--codec", "yuv420",
                "--width", &CAP_W.to_string(),
                "--height", &CAP_H.to_string(),
                "--framerate", &fps.to_string(),
                "-t", "0", // run until killed
                "--nopreview",
                "--flush", // push each frame out promptly (lower latency)
                "-o", "-", // raw stream to stdout
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()?;

        let mut stdout = child
            .stdout
            .take()
            .ok_or_else(|| io::Error::other("rpicam-vid produced no stdout"))?;

        let latest = Arc::new(Mutex::new(None));
        let sink = latest.clone();
        let reader = thread::Builder::new().name("camera-reader".into()).spawn(move || {
            let mut buf = vec![0u8; CAP_FRAME_BYTES];
            // read_exact gives us consecutive fixed-size frames; on pipe close
            // (camera stopped / process killed) it errors and we exit.
            while stdout.read_exact(&mut buf).is_ok() {
                *sink.lock().unwrap() = Some(buf.clone());
            }
        })?;

        Ok(Camera { child, latest, reader: Some(reader) })
    }

    /// The most recent frame as 640×360 row-major RGB8, or `None` if no frame
    /// has arrived yet.
    pub fn latest_rgb(&self) -> Option<Vec<u8>> {
        let raw = self.latest.lock().unwrap().clone()?;
        Some(yuv420_to_rgb_downsampled(&raw))
    }

    pub fn stop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        if let Some(h) = self.reader.take() {
            let _ = h.join();
        }
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Convert a 1280×720 I420 frame to 640×360 RGB: average each 2×2 luma block and
/// pair it with the co-located chroma sample (BT.601). Panics if `raw` is the
/// wrong size — the reader only ever hands us full frames.
pub fn yuv420_to_rgb_downsampled(raw: &[u8]) -> Vec<u8> {
    assert_eq!(raw.len(), CAP_FRAME_BYTES, "unexpected YUV frame size");
    let y = &raw[..Y_SIZE];
    let u = &raw[Y_SIZE..Y_SIZE + C_SIZE];
    let v = &raw[Y_SIZE + C_SIZE..];

    let mut out = vec![0u8; (OUT_W * OUT_H * 3) as usize];
    for oy in 0..OUT_H as usize {
        for ox in 0..OUT_W as usize {
            let (x0, y0) = (2 * ox, 2 * oy);
            let yv = (y[y0 * CAP_W + x0] as u32
                + y[y0 * CAP_W + x0 + 1] as u32
                + y[(y0 + 1) * CAP_W + x0] as u32
                + y[(y0 + 1) * CAP_W + x0 + 1] as u32)
                / 4;
            let c = oy * C_W + ox; // chroma plane is exactly OUT_W×OUT_H
            let (yf, uf, vf) = (yv as f32, u[c] as f32 - 128.0, v[c] as f32 - 128.0);
            let o = (oy * OUT_W as usize + ox) * 3;
            out[o] = clamp8(yf + 1.402 * vf);
            out[o + 1] = clamp8(yf - 0.344_136 * uf - 0.714_136 * vf);
            out[o + 2] = clamp8(yf + 1.772 * uf);
        }
    }
    out
}

fn clamp8(x: f32) -> u8 {
    x.round().clamp(0.0, 255.0) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn solid_frame(yv: u8, uv: u8, vv: u8) -> Vec<u8> {
        let mut f = vec![yv; Y_SIZE];
        f.extend(std::iter::repeat(uv).take(C_SIZE));
        f.extend(std::iter::repeat(vv).take(C_SIZE));
        f
    }

    #[test]
    fn output_has_the_working_resolution() {
        let rgb = yuv420_to_rgb_downsampled(&solid_frame(128, 128, 128));
        assert_eq!(rgb.len(), (OUT_W * OUT_H * 3) as usize);
    }

    #[test]
    fn neutral_chroma_grey_round_trips() {
        // Y=128, U=V=128 → mid-grey on all channels.
        let rgb = yuv420_to_rgb_downsampled(&solid_frame(128, 128, 128));
        assert!(rgb.iter().all(|&c| (c as i32 - 128).abs() <= 1));
    }

    #[test]
    fn red_chroma_lights_the_red_channel() {
        // High V with neutral U pushes red up and blue down (BT.601).
        let rgb = yuv420_to_rgb_downsampled(&solid_frame(128, 128, 255));
        let (r, g, b) = (rgb[0], rgb[1], rgb[2]);
        assert!(r > g && r > b, "expected red-dominant, got ({r},{g},{b})");
    }
}
