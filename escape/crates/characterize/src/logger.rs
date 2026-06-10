//! JSONL session log: one `tick` line per sensor sample, `event` lines for
//! everything else. Timestamps are seconds since runner start, which is also
//! (approximately) video start + a logged offset event.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::Instant;

use serde_json::{json, Value};

use crate::telemetry::{Odom, Tick};

pub struct Logger {
    w: BufWriter<File>,
    t0: Instant,
}

impl Logger {
    pub fn new(path: &Path, t0: Instant) -> Logger {
        let f = File::create(path).expect("create session.jsonl");
        Logger { w: BufWriter::new(f), t0 }
    }

    fn emit(&mut self, v: Value) {
        let _ = writeln!(self.w, "{v}");
        let _ = self.w.flush();
    }

    pub fn event(&mut self, phase: &str, msg: &str, extra: Value) {
        let t = self.t0.elapsed().as_secs_f64();
        self.emit(json!({"t": t, "kind": "event", "phase": phase, "msg": msg, "extra": extra}));
    }

    pub fn tick(&mut self, phase: &str, tick: &Tick, odom: &Odom) {
        let t = self.t0.elapsed().as_secs_f64();
        let readings: serde_json::Map<String, Value> = tick
            .readings
            .iter()
            .map(|(k, v)| (k.to_string(), json!(v)))
            .collect();
        self.emit(json!({
            "t": t,
            "kind": "tick",
            "phase": phase,
            "readings": readings,
            "bump_l": tick.bump_left,
            "bump_r": tick.bump_right,
            "wheel_drop": tick.wheel_drop,
            "odom": {
                "enc_dist_mm": odom.enc_dist_mm,
                "enc_angle_deg": odom.enc_angle_deg,
                "i_dist_mm": odom.i_dist_mm,
                "i_angle_deg": odom.i_angle_deg,
            },
        }));
    }
}
