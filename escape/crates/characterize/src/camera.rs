//! Camera capture via rpicam-apps subprocesses. The camera is mounted the
//! correct way up — no flips. Video (720p H.264) records continuously through
//! the motion phases; that footage is the entire calibration input (the floor
//! flowing under known ego-motion), so no stills are needed.

use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::Duration;

use crate::tts;

pub struct Camera {
    pub enabled: bool,
    out: PathBuf,
    child: Option<Child>,
}

impl Camera {
    pub fn new(enabled: bool, out: PathBuf) -> Camera {
        Camera { enabled, out, child: None }
    }

    pub fn start_video(&mut self) {
        if !self.enabled {
            return;
        }
        let video = self.out.join("video.h264");
        let spawn = Command::new("rpicam-vid")
            .args(["-t", "0", "--nopreview", "--width", "1280", "--height", "720", "--framerate", "30"])
            .arg("-o")
            .arg(&video)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();
        match spawn {
            Ok(mut c) => {
                // An unsupported flag or busy camera exits immediately — catch it.
                thread::sleep(Duration::from_millis(500));
                match c.try_wait() {
                    Ok(Some(status)) => {
                        tts::say(&format!("Warning: camera recorder exited early, status {status}. Continuing without video."));
                        self.enabled = false;
                    }
                    _ => {
                        self.child = Some(c);
                        tts::say("Camera recording.");
                    }
                }
            }
            Err(e) => {
                tts::say(&format!("Warning: camera failed to start: {e}. Continuing without video."));
                self.enabled = false;
            }
        }
    }

    pub fn stop_video(&mut self) {
        if let Some(mut c) = self.child.take() {
            let _ = c.kill(); // raw H.264 elementary stream survives a hard kill
            let _ = c.wait();
        }
    }
}
