//! Robot command layer: wraps a byte transport (serial port in production,
//! an in-memory buffer in tests) and exposes high-level operations built from
//! the `protocol` and `sensors` modules.

use std::io::{self, Read, Write};

use crate::{protocol, sensors};

pub struct Robot<T: Read + Write> {
    port: T,
}

impl<T: Read + Write> Robot<T> {
    pub fn new(port: T) -> Self {
        Robot { port }
    }

    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()
    }

    pub fn enter_safe(&mut self) -> io::Result<()> {
        self.send(&protocol::enter_safe())
    }

    pub fn enter_full(&mut self) -> io::Result<()> {
        self.send(&protocol::enter_full())
    }

    pub fn start(&mut self) -> io::Result<()> {
        self.send(&protocol::start())
    }

    pub fn drive(&mut self, velocity_mm_s: i16, radius_mm: i16) -> io::Result<()> {
        self.send(&protocol::drive(velocity_mm_s, radius_mm))
    }

    pub fn drive_direct(&mut self, right_mm_s: i16, left_mm_s: i16) -> io::Result<()> {
        self.send(&protocol::drive_direct(right_mm_s, left_mm_s))
    }

    pub fn go(&mut self, cm_per_sec: f64, deg_per_sec: f64) -> io::Result<()> {
        self.send(&protocol::go(cm_per_sec, deg_per_sec))
    }

    pub fn stop(&mut self) -> io::Result<()> {
        self.send(&protocol::stop())
    }

    pub fn motors(&mut self, side_brush: i8, main_brush: i8, vacuum: i8) -> io::Result<()> {
        self.send(&protocol::motors(side_brush, main_brush, vacuum))
    }

    pub fn leds(&mut self, color: u8, intensity: u8, play: bool, advance: bool) -> io::Result<()> {
        self.send(&protocol::leds(color, intensity, play, advance))
    }

    pub fn seek_dock(&mut self) -> io::Result<()> {
        self.send(&protocol::seek_dock())
    }

    /// Request a list of sensors and block until the full reply is read back.
    pub fn query(&mut self, ids: &[u8]) -> io::Result<Vec<sensors::Reading>> {
        let total: usize = ids
            .iter()
            .map(|&i| sensors::width(i).unwrap_or(0))
            .sum();
        self.send(&protocol::query_list(ids))?;
        let mut buf = vec![0u8; total];
        self.port.read_exact(&mut buf)?;
        sensors::decode(ids, &buf)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("{:?}", e)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::{Reading, CURRENT, VOLTAGE};

    /// In-memory transport: records everything written, replays canned reads.
    struct Mock {
        written: Vec<u8>,
        to_read: io::Cursor<Vec<u8>>,
    }

    impl Mock {
        fn new(to_read: Vec<u8>) -> Self {
            Mock { written: Vec::new(), to_read: io::Cursor::new(to_read) }
        }
    }

    impl Write for Mock {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Read for Mock {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.to_read.read(buf)
        }
    }

    #[test]
    fn enter_safe_writes_start_then_safe() {
        let mut r = Robot::new(Mock::new(vec![]));
        r.enter_safe().unwrap();
        assert_eq!(r.port.written, vec![128, 131]);
    }

    #[test]
    fn go_forward_writes_drive_command() {
        let mut r = Robot::new(Mock::new(vec![]));
        r.go(20.0, 0.0).unwrap();
        // go() emits DRIVE_DIRECT (145) with equal wheel speeds for straight.
        assert_eq!(r.port.written, vec![145, 0x00, 0xC8, 0x00, 0xC8]);
    }

    #[test]
    fn query_sends_request_and_parses_reply() {
        // reply: voltage 0x3E80=16000, current 0xFF38=-200
        let mut r = Robot::new(Mock::new(vec![0x3E, 0x80, 0xFF, 0x38]));
        let readings = r.query(&[VOLTAGE, CURRENT]).unwrap();
        // request bytes: QUERY_LIST, count, ids
        assert_eq!(r.port.written, vec![149, 2, VOLTAGE, CURRENT]);
        assert_eq!(
            readings,
            vec![
                Reading { id: VOLTAGE, value: 16000 },
                Reading { id: CURRENT, value: -200 },
            ]
        );
    }
}
