//! In-memory fake Roomba implementing Read + Write over the OI byte protocol,
//! used by `--dry-run` to validate the full session sequencing on a dev
//! machine. Simulates: differential-drive kinematics, 16-bit wrapping wheel
//! encoders, integrating distance/angle packets (reset on read), a wall
//! 1.7 m ahead of the start pose (bump + light-bump ramp), and a dark floor
//! tile band (cliff-signal reflectivity dip).

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::Instant;

use roomba_pilot::sensors;

use crate::telemetry::{MM_PER_TICK, WHEEL_SPAN_MM};

const WALL_X_MM: f64 = 1700.0;
const DARK_TILE_X_MM: (f64, f64) = (600.0, 900.0);

pub struct MockRoomba {
    rx: VecDeque<u8>, // bytes waiting to be read by the host
    cmd: Vec<u8>,     // partial command bytes from the host
    last: Instant,
    // pose & motion
    x: f64,
    y: f64,
    th: f64, // rad, +CCW; starts facing +x (toward the wall)
    vl: f64, // mm/s
    vr: f64,
    // sensors
    enc_l: f64, // ticks (wraps at u16 when reported)
    enc_r: f64,
    dist_i: f64,  // packet 19 accumulator, reset on read
    angle_i: f64, // packet 20 accumulator (deg), reset on read
    mode: u8,
}

impl MockRoomba {
    pub fn new() -> MockRoomba {
        MockRoomba {
            rx: VecDeque::new(),
            cmd: Vec::new(),
            last: Instant::now(),
            x: 0.0,
            y: 0.0,
            th: 0.0,
            vl: 0.0,
            vr: 0.0,
            enc_l: 1000.0,
            enc_r: 1000.0,
            dist_i: 0.0,
            angle_i: 0.0,
            mode: 1,
        }
    }

    fn advance(&mut self) {
        let dt = self.last.elapsed().as_secs_f64();
        self.last = Instant::now();
        if dt <= 0.0 {
            return;
        }
        let (mut vl, mut vr) = (self.vl, self.vr);
        // The wall blocks forward translation (wheels stall, encoders stop).
        if self.bumped() && (vl + vr) > 0.0 {
            vl = 0.0;
            vr = 0.0;
        }
        let v = (vl + vr) / 2.0;
        let w = (vr - vl) / WHEEL_SPAN_MM;
        self.x += v * dt * (self.th + w * dt / 2.0).cos();
        self.y += v * dt * (self.th + w * dt / 2.0).sin();
        self.th += w * dt;
        self.enc_l += vl * dt / MM_PER_TICK;
        self.enc_r += vr * dt / MM_PER_TICK;
        self.dist_i += v * dt;
        self.angle_i += w * dt * 180.0 / std::f64::consts::PI;
    }

    fn gap_mm(&self) -> f64 {
        // Distance from the bumper to the wall along +x, only meaningful when
        // roughly facing it.
        (WALL_X_MM - self.x).max(0.0)
    }

    fn facing_wall(&self) -> bool {
        self.th.cos() > 0.5
    }

    fn bumped(&self) -> bool {
        self.facing_wall() && self.gap_mm() <= 1.0
    }

    fn light_signal(&self, spread: f64) -> i32 {
        if !self.facing_wall() {
            return 0;
        }
        let g = self.gap_mm();
        if g < 250.0 {
            (((250.0 - g) * 8.0) * spread) as i32
        } else {
            0
        }
    }

    fn cliff_signal(&self) -> i32 {
        // Dark blue tile band reads less reflective than the light blue carpet.
        if self.x >= DARK_TILE_X_MM.0 && self.x <= DARK_TILE_X_MM.1 {
            1400
        } else {
            2200
        }
    }

    fn value(&mut self, id: u8) -> i32 {
        match id {
            7 => {
                if self.bumped() {
                    0x03
                } else {
                    0
                }
            }
            8 | 13 | 21 | 27 | 45 => 0,
            19 => {
                let v = self.dist_i as i32;
                self.dist_i -= v as f64;
                v
            }
            20 => {
                let v = self.angle_i as i32;
                self.angle_i -= v as f64;
                v
            }
            22 => 15_500,
            23 => -310,
            24 => 22,
            25 => 2200,
            26 => 2700,
            28..=31 => self.cliff_signal(),
            35 => self.mode as i32,
            43 => (self.enc_l as i64 as u16) as i32,
            44 => (self.enc_r as i64 as u16) as i32,
            46 | 51 => self.light_signal(0.3),      // outer left / right
            47 | 50 => self.light_signal(0.6),      // front left / right
            48 | 49 => self.light_signal(1.0),      // center pair
            _ => 0,
        }
    }

    fn handle_commands(&mut self) {
        loop {
            let Some(&op) = self.cmd.first() else { return };
            // opcode -> fixed argument byte count (None = variable, first arg is count)
            let fixed: Option<usize> = match op {
                128 | 131 | 132 | 133 | 134 | 135 | 136 | 143 => Some(0),
                138 | 142 | 150 => Some(1),
                137 | 145 => Some(4),
                139 => Some(3),
                148 | 149 => None,
                _ => {
                    // Unknown opcode: drop the byte rather than stalling forever.
                    self.cmd.remove(0);
                    continue;
                }
            };
            let (args_len, total) = match fixed {
                Some(n) => (n, 1 + n),
                None => {
                    if self.cmd.len() < 2 {
                        return; // wait for the count byte
                    }
                    let n = self.cmd[1] as usize;
                    (1 + n, 2 + n)
                }
            };
            if self.cmd.len() < total {
                return; // wait for the rest of the command
            }
            let args: Vec<u8> = self.cmd[1..1 + args_len].to_vec();
            self.cmd.drain(..total);
            self.execute(op, &args);
        }
    }

    fn execute(&mut self, op: u8, args: &[u8]) {
        match op {
            128 => self.mode = 1,
            131 => self.mode = 2,
            132 => self.mode = 3,
            137 => {
                let v = i16::from_be_bytes([args[0], args[1]]) as f64;
                let r = i16::from_be_bytes([args[2], args[3]]);
                self.advance();
                match r {
                    i16::MIN => {
                        self.vl = v;
                        self.vr = v;
                    }
                    1 => {
                        // CCW spin in place
                        self.vl = -v;
                        self.vr = v;
                    }
                    -1 => {
                        self.vl = v;
                        self.vr = -v;
                    }
                    _ => {
                        let r = r as f64;
                        self.vr = v * (r + WHEEL_SPAN_MM / 2.0) / r;
                        self.vl = v * (r - WHEEL_SPAN_MM / 2.0) / r;
                    }
                }
            }
            145 => {
                self.advance();
                self.vr = i16::from_be_bytes([args[0], args[1]]) as f64;
                self.vl = i16::from_be_bytes([args[2], args[3]]) as f64;
            }
            149 => {
                self.advance();
                for &id in &args[1..] {
                    let w = sensors::width(id).unwrap_or(0);
                    let val = self.value(id);
                    match w {
                        1 => self.rx.push_back(val as u8),
                        2 => {
                            let b = (val as i16 as u16).to_be_bytes();
                            self.rx.push_back(b[0]);
                            self.rx.push_back(b[1]);
                        }
                        _ => {}
                    }
                }
            }
            148 => {
                // Emit a burst of valid stream frames immediately (the real
                // robot emits one every 15 ms; for the dry run only framing
                // validity matters).
                self.advance();
                let ids: Vec<u8> = args[1..].to_vec();
                for _ in 0..33 {
                    let mut payload = Vec::new();
                    for &id in &ids {
                        payload.push(id);
                        let w = sensors::width(id).unwrap_or(0);
                        let val = self.value(id);
                        match w {
                            1 => payload.push(val as u8),
                            2 => payload.extend((val as i16 as u16).to_be_bytes()),
                            _ => {}
                        }
                    }
                    let mut f = vec![19u8, payload.len() as u8];
                    f.extend(payload);
                    let sum: u32 = f.iter().map(|&b| b as u32).sum();
                    f.push(((256 - (sum % 256)) % 256) as u8);
                    self.rx.extend(f);
                }
            }
            150 => {} // pause/resume stream: burst model has nothing to stop
            _ => {}
        }
    }
}

impl Write for MockRoomba {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.cmd.extend_from_slice(buf);
        self.handle_commands();
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Read for MockRoomba {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.advance();
        let n = buf.len().min(self.rx.len());
        for b in buf.iter_mut().take(n) {
            *b = self.rx.pop_front().unwrap();
        }
        Ok(n) // Ok(0) when empty -> read_exact fails like a serial timeout
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::Rig;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn query_returns_all_probed_packets() {
        let mut rig = Rig::new(MockRoomba::new());
        rig.wake_and_safe().unwrap();
        let r = rig.query(&[35, 43, 44, 19, 20]).unwrap();
        assert_eq!(r[0].value, 2); // SAFE mode
        assert_eq!(r.len(), 5);
    }

    #[test]
    fn driving_moves_encoders_forward() {
        let mut rig = Rig::new(MockRoomba::new());
        rig.wake_and_safe().unwrap();
        let before = rig.query(&[43]).unwrap()[0].value;
        rig.drive(200, i16::MIN).unwrap(); // straight, 200 mm/s
        thread::sleep(Duration::from_millis(120));
        let after = rig.query(&[43]).unwrap()[0].value;
        rig.stop().unwrap();
        let dticks = (after as u16).wrapping_sub(before as u16) as i16;
        assert!(dticks > 30, "expected forward ticks, got {dticks}");
    }

    #[test]
    fn stream_test_counts_frames() {
        let mut rig = Rig::new(MockRoomba::new());
        rig.wake_and_safe().unwrap();
        let frames = rig.stream_test(&[7, 43, 44], Duration::from_millis(200));
        assert_eq!(frames, 33);
    }
}
