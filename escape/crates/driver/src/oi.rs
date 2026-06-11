//! Low-level Open Interface transport.
//!
//! Owns a serial-like byte channel (`T: Read + Write`) and speaks the OI wire
//! format via `roomba_pilot`'s pure codecs. This is the only place that touches
//! raw bytes; everything above works in decoded values. Generic over the
//! transport so the control loop can be exercised against an in-memory fake in
//! tests (no robot, no serial port).

use std::collections::BTreeMap;
use std::io::{self, Read, Write};
use std::thread;
use std::time::Duration;

use roomba_pilot::{protocol, sensors};

pub struct Oi<T: Read + Write> {
    port: T,
}

impl<T: Read + Write> Oi<T> {
    pub fn new(port: T) -> Self {
        Oi { port }
    }

    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()
    }

    /// START → settle → SAFE → settle → drain the wake-up chatter. SAFE mode is
    /// required before the robot will accept drive commands.
    pub fn wake_and_safe(&mut self) -> io::Result<()> {
        self.send(&[protocol::START])?;
        thread::sleep(Duration::from_millis(300));
        self.send(&[protocol::SAFE])?;
        thread::sleep(Duration::from_millis(300));
        self.drain();
        Ok(())
    }

    /// Re-assert SAFE after the OI kicked itself to passive (cliff/wheel-drop).
    pub fn reenter_safe(&mut self) -> io::Result<()> {
        self.send(&[protocol::START, protocol::SAFE])?;
        thread::sleep(Duration::from_millis(50));
        self.drain();
        Ok(())
    }

    /// Drop back to passive mode (wheels off, charging allowed).
    pub fn passive(&mut self) -> io::Result<()> {
        self.send(&[protocol::START])
    }

    /// Independent wheel velocities, mm/s (clamped by the caller to ±500).
    pub fn drive_direct(&mut self, right_mm_s: i16, left_mm_s: i16) -> io::Result<()> {
        self.send(&protocol::drive_direct(right_mm_s, left_mm_s))
    }

    pub fn stop(&mut self) -> io::Result<()> {
        self.send(&protocol::stop())
    }

    /// Query a list of packet ids, blocking until the full fixed-width reply
    /// arrives. A read timeout (e.g. an unsupported packet) surfaces as an Err,
    /// which the caller treats as "this packet set is unsupported".
    ///
    /// Returns id → sign-extended value. Ids the caller didn't request are
    /// simply absent from the map.
    pub fn query(&mut self, ids: &[u8]) -> io::Result<BTreeMap<u8, i32>> {
        let total: usize = ids.iter().map(|&i| sensors::width(i).unwrap_or(0)).sum();
        self.send(&protocol::query_list(ids))?;
        let mut buf = vec![0u8; total];
        self.port.read_exact(&mut buf)?;
        let readings = sensors::decode(ids, &buf)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("{e:?}")))?;
        Ok(readings.into_iter().map(|r| (r.id, r.value)).collect())
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
}
