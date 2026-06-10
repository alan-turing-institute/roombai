//! Spoken narration via the speaker daemon (CLAUDE.md): append one line to
//! /tmp/speak_queue.txt. Also printed to stdout so the console mirrors it.

use std::fs::OpenOptions;
use std::io::Write;

pub fn say(msg: &str) {
    println!("[say] {msg}");
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open("/tmp/speak_queue.txt") {
        let _ = writeln!(f, "{msg}");
    }
}
