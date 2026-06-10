//! Thin serial wrapper for the characterization runner.
//!
//! Reuses `roomba_pilot`'s pure protocol/sensor modules but owns its port
//! directly so it can drain garbage, probe unsupported packets with timeouts,
//! and test OI stream mode (opcode 148) — none of which `roomba_pilot::Robot`
//! exposes.

use std::io::{self, Read, Write};
use std::thread;
use std::time::{Duration, Instant};

use roomba_pilot::{protocol, sensors};

pub struct Rig<T: Read + Write> {
    port: T,
}

impl<T: Read + Write> Rig<T> {
    pub fn new(port: T) -> Self {
        Rig { port }
    }

    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()
    }

    /// START, settle, SAFE, settle, drain whatever the wake-up spewed.
    pub fn wake_and_safe(&mut self) -> io::Result<()> {
        self.send(&[protocol::START])?;
        thread::sleep(Duration::from_millis(300));
        self.send(&[protocol::SAFE])?;
        thread::sleep(Duration::from_millis(300));
        self.drain();
        Ok(())
    }

    /// SAFE mode re-entry after the OI kicked itself to passive (cliff/wheel-drop).
    pub fn reenter_safe(&mut self) -> io::Result<()> {
        self.send(&[protocol::START, protocol::SAFE])?;
        thread::sleep(Duration::from_millis(100));
        self.drain();
        Ok(())
    }

    pub fn passive(&mut self) -> io::Result<()> {
        self.send(&[protocol::START])
    }

    pub fn drive(&mut self, velocity_mm_s: i16, radius_mm: i16) -> io::Result<()> {
        self.send(&protocol::drive(velocity_mm_s, radius_mm))
    }

    pub fn stop(&mut self) -> io::Result<()> {
        self.send(&protocol::stop())
    }

    /// Query a list of packet ids, blocking until the full reply arrives.
    /// A read timeout (e.g. an unsupported packet returning nothing) is an Err.
    pub fn query(&mut self, ids: &[u8]) -> io::Result<Vec<sensors::Reading>> {
        let total: usize = ids.iter().map(|&i| sensors::width(i).unwrap_or(0)).sum();
        self.send(&protocol::query_list(ids))?;
        let mut buf = vec![0u8; total];
        self.port.read_exact(&mut buf)?;
        sensors::decode(ids, &buf)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("{e:?}")))
    }

    /// Discard any unread bytes (after a failed probe or paused stream).
    pub fn drain(&mut self) {
        let mut buf = [0u8; 256];
        loop {
            match self.port.read(&mut buf) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(_) => break,
            }
        }
    }

    /// Start OI stream mode for `ids`, collect for `dur`, pause the stream,
    /// and return how many checksum-valid frames arrived.
    pub fn stream_test(&mut self, ids: &[u8], dur: Duration) -> u32 {
        let mut req = vec![protocol::STREAM, ids.len() as u8];
        req.extend_from_slice(ids);
        if self.send(&req).is_err() {
            return 0;
        }
        let deadline = Instant::now() + dur;
        let mut data = Vec::new();
        let mut buf = [0u8; 512];
        while Instant::now() < deadline {
            match self.port.read(&mut buf) {
                Ok(0) => thread::sleep(Duration::from_millis(10)),
                Ok(n) => data.extend_from_slice(&buf[..n]),
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => continue,
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => continue,
                Err(_) => break,
            }
        }
        let _ = self.send(&[protocol::PAUSE_RESUME, 0]);
        thread::sleep(Duration::from_millis(100));
        self.drain();
        count_stream_frames(&data)
    }
}

/// Count valid OI stream frames: [19][len][payload...][checksum], where the
/// sum of every byte from the header through the checksum is 0 mod 256.
pub fn count_stream_frames(data: &[u8]) -> u32 {
    let mut count = 0u32;
    let mut i = 0usize;
    while i < data.len() {
        if data[i] == 19 && i + 1 < data.len() {
            let len = data[i + 1] as usize;
            let end = i + 2 + len + 1; // header + len byte + payload + checksum
            if end <= data.len() {
                let sum: u32 = data[i..end].iter().map(|&b| b as u32).sum();
                if sum % 256 == 0 {
                    count += 1;
                    i = end;
                    continue;
                }
            }
        }
        i += 1;
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(payload: &[u8]) -> Vec<u8> {
        let mut f = vec![19u8, payload.len() as u8];
        f.extend_from_slice(payload);
        let sum: u32 = f.iter().map(|&b| b as u32).sum();
        f.push(((256 - (sum % 256)) % 256) as u8);
        f
    }

    #[test]
    fn counts_valid_frames_and_skips_noise() {
        let mut data = vec![0xAA, 0x19]; // leading noise
        data.extend(frame(&[7, 0]));
        data.extend([19, 5, 1]); // truncated fake frame header
        data.extend(frame(&[35, 2]));
        assert_eq!(count_stream_frames(&data), 2);
    }

    #[test]
    fn rejects_bad_checksum() {
        let mut f = frame(&[7, 0]);
        *f.last_mut().unwrap() ^= 0xFF;
        assert_eq!(count_stream_frames(&f), 0);
    }
}
