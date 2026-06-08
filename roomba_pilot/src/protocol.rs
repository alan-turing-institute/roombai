//! Pure encoding/decoding of the iRobot Open Interface (OI) serial protocol.
//!
//! This is a faithful port of the wire format implemented in the reference
//! Python `create.py`. Everything here is pure (no I/O) so it can be unit
//! tested without the robot attached.

// ---- Opcodes (see create.py module-level constants) -------------------------
pub const START: u8 = 128;
pub const BAUD: u8 = 129;
pub const SAFE: u8 = 131;
pub const FULL: u8 = 132;
pub const POWER: u8 = 133;
pub const SPOT: u8 = 134;
pub const CLEAN: u8 = 135;
pub const MAX_DEMO: u8 = 136;
pub const DRIVE: u8 = 137;
pub const MOTORS: u8 = 138;
pub const LEDS: u8 = 139;
pub const SONG: u8 = 140;
pub const PLAY: u8 = 141;
pub const SENSORS: u8 = 142;
pub const FORCE_SEEKING_DOCK: u8 = 143;
pub const DRIVE_DIRECT: u8 = 145;
pub const STREAM: u8 = 148;
pub const QUERY_LIST: u8 = 149;
pub const PAUSE_RESUME: u8 = 150;

/// Radius value meaning "drive straight" (OI spec special value 0x8000).
pub const RADIUS_STRAIGHT: i16 = i16::MIN; // 0x8000
/// Radius value meaning "turn in place counter-clockwise".
pub const RADIUS_CCW: i16 = 1;
/// Radius value meaning "turn in place clockwise".
pub const RADIUS_CW: i16 = -1;

pub const WHEEL_SPAN_MM: f64 = 235.0;

// ---- Command encoders -------------------------------------------------------

/// `START` opcode -> passive mode.
pub fn start() -> Vec<u8> {
    vec![START]
}

/// Sequence to enter SAFE mode: START then SAFE.
pub fn enter_safe() -> Vec<u8> {
    vec![START, SAFE]
}

/// Sequence to enter FULL mode: START, SAFE, FULL.
pub fn enter_full() -> Vec<u8> {
    vec![START, SAFE, FULL]
}

/// DRIVE command: velocity in mm/s, radius in mm (or one of the RADIUS_* consts).
pub fn drive(velocity_mm_s: i16, radius_mm: i16) -> Vec<u8> {
    let v = velocity_mm_s.to_be_bytes();
    let r = radius_mm.to_be_bytes();
    vec![DRIVE, v[0], v[1], r[0], r[1]]
}

/// DRIVE_DIRECT command: independent wheel velocities in mm/s.
/// Wire order is right wheel first, then left (matches create.py).
pub fn drive_direct(right_mm_s: i16, left_mm_s: i16) -> Vec<u8> {
    let r = right_mm_s.to_be_bytes();
    let l = left_mm_s.to_be_bytes();
    vec![DRIVE_DIRECT, r[0], r[1], l[0], l[1]]
}

/// Stop both wheels (drive-direct 0,0).
pub fn stop() -> Vec<u8> {
    drive_direct(0, 0)
}

/// MOTORS command. Each arg: -1 reverse, 0 off, 1 forward. Vacuum has no reverse.
pub fn motors(side_brush: i8, main_brush: i8, vacuum: i8) -> Vec<u8> {
    let mut byte = 0u8;
    if main_brush < 0 {
        byte |= 16;
    }
    if side_brush < 0 {
        byte |= 8;
    }
    if main_brush != 0 {
        byte |= 4;
    }
    if vacuum != 0 {
        byte |= 2;
    }
    if side_brush > 0 {
        byte |= 1;
    }
    vec![MOTORS, byte]
}

/// LEDS command. `power_color` 0=green..255=red, `power_intensity` 0..255,
/// `play` and `advance` are on/off.
pub fn leds(power_color: u8, power_intensity: u8, play: bool, advance: bool) -> Vec<u8> {
    let bits = ((advance as u8) << 3) | ((play as u8) << 1);
    vec![LEDS, bits, power_color, power_intensity]
}

/// QUERY_LIST: request the given sensor packet ids.
pub fn query_list(ids: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(ids.len() + 2);
    out.push(QUERY_LIST);
    out.push(ids.len() as u8);
    out.extend_from_slice(ids);
    out
}

/// FORCE_SEEKING_DOCK.
pub fn seek_dock() -> Vec<u8> {
    vec![FORCE_SEEKING_DOCK]
}

/// Convert a high-level `go(cm/s, deg/s)` request into a DRIVE command,
/// mirroring create.py's `go`. Only pure-translation or pure-rotation are
/// supported here (matching how the daemon uses it).
pub fn go(cm_per_sec: f64, deg_per_sec: f64) -> Vec<u8> {
    if cm_per_sec == 0.0 {
        // pure rotation in place
        let rad_per_sec = deg_per_sec.to_radians();
        let vel_mm = (rad_per_sec.abs() * (WHEEL_SPAN_MM / 2.0)) as i16; // trunc toward zero, like int()
        let radius = if rad_per_sec >= 0.0 { RADIUS_CCW } else { RADIUS_CW };
        drive(vel_mm, radius)
    } else {
        // pure translation (deg_per_sec assumed 0)
        let vel_mm = (10.0 * cm_per_sec) as i16;
        drive(vel_mm, RADIUS_STRAIGHT)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_is_single_opcode() {
        assert_eq!(start(), vec![128]);
    }

    #[test]
    fn enter_safe_sends_start_then_safe() {
        assert_eq!(enter_safe(), vec![128, 131]);
    }

    #[test]
    fn enter_full_sends_start_safe_full() {
        assert_eq!(enter_full(), vec![128, 131, 132]);
    }

    #[test]
    fn drive_straight_forward() {
        // 200 mm/s straight: radius special 0x8000
        assert_eq!(drive(200, RADIUS_STRAIGHT), vec![137, 0x00, 0xC8, 0x80, 0x00]);
    }

    #[test]
    fn drive_reverse_is_twos_complement() {
        // -500 mm/s -> 0xFE0C
        assert_eq!(drive(-500, RADIUS_STRAIGHT), vec![137, 0xFE, 0x0C, 0x80, 0x00]);
    }

    #[test]
    fn drive_spin_ccw_and_cw() {
        assert_eq!(drive(100, RADIUS_CCW), vec![137, 0x00, 0x64, 0x00, 0x01]);
        assert_eq!(drive(100, RADIUS_CW), vec![137, 0x00, 0x64, 0xFF, 0xFF]);
    }

    #[test]
    fn drive_direct_right_first_then_left() {
        // create.py writes right high/low then left high/low.
        // right=-100 -> 0xFF9C, left=100 -> 0x0064
        assert_eq!(drive_direct(-100, 100), vec![145, 0xFF, 0x9C, 0x00, 0x64]);
    }

    #[test]
    fn stop_halts_both_wheels() {
        assert_eq!(stop(), vec![145, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn motors_all_off() {
        assert_eq!(motors(0, 0, 0), vec![138, 0]);
    }

    #[test]
    fn motors_all_forward() {
        // main!=0 ->4, vac!=0 ->2, side>0 ->1  => 7
        assert_eq!(motors(1, 1, 1), vec![138, 7]);
    }

    #[test]
    fn motors_brushes_reverse() {
        // main<0 ->16, side<0 ->8, main!=0 ->4 => 28
        assert_eq!(motors(-1, -1, 0), vec![138, 28]);
    }

    #[test]
    fn leds_encoding() {
        // bits = (advance<<3)|(play<<1) = 8|2 = 10
        assert_eq!(leds(0, 128, true, true), vec![139, 10, 0, 128]);
        assert_eq!(leds(255, 64, false, false), vec![139, 0, 255, 64]);
    }

    #[test]
    fn query_list_prefixes_count() {
        assert_eq!(query_list(&[25, 26]), vec![149, 2, 25, 26]);
    }

    #[test]
    fn seek_dock_opcode() {
        assert_eq!(seek_dock(), vec![143]);
    }

    #[test]
    fn go_forward_translates_cm_to_mm_straight() {
        // 20 cm/s -> 200 mm/s straight
        assert_eq!(go(20.0, 0.0), vec![137, 0x00, 0xC8, 0x80, 0x00]);
    }

    #[test]
    fn go_spin_in_place() {
        // go(0, 90): vel = radians(90)*117.5 = 184.6 -> 184 mm/s, CCW radius=1
        // 184 -> 0x00B8
        assert_eq!(go(0.0, 90.0), vec![137, 0x00, 0xB8, 0x00, 0x01]);
        // negative deg spins CW
        assert_eq!(go(0.0, -90.0), vec![137, 0x00, 0xB8, 0xFF, 0xFF]);
    }
}
