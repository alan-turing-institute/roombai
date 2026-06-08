//! Decoding of OI sensor query-list responses.
//!
//! Widths and signedness are ported from create.py's `SENSOR_DATA_WIDTH` and
//! the `sensorDataInterpreter` table.

// ---- Sensor packet ids we care about ----------------------------------------
pub const BUMPS_AND_WHEEL_DROPS: u8 = 7;
pub const WALL_IR_SENSOR: u8 = 8;
pub const VIRTUAL_WALL: u8 = 13;
pub const CHARGING_STATE: u8 = 21;
pub const VOLTAGE: u8 = 22;
pub const CURRENT: u8 = 23;
pub const BATTERY_TEMP: u8 = 24;
pub const BATTERY_CHARGE: u8 = 25;
pub const BATTERY_CAPACITY: u8 = 26;
pub const WALL_SIGNAL: u8 = 27;
pub const CLIFF_LEFT_SIGNAL: u8 = 28;
pub const CLIFF_FRONT_LEFT_SIGNAL: u8 = 29;
pub const CLIFF_FRONT_RIGHT_SIGNAL: u8 = 30;
pub const CLIFF_RIGHT_SIGNAL: u8 = 31;
pub const OI_MODE: u8 = 35;
pub const ENCODER_LEFT: u8 = 43;
pub const ENCODER_RIGHT: u8 = 44;
pub const LIGHTBUMP: u8 = 45;
pub const LIGHTBUMP_LEFT: u8 = 46;
pub const LIGHTBUMP_FRONT_LEFT: u8 = 47;
pub const LIGHTBUMP_CENTER_LEFT: u8 = 48;
pub const LIGHTBUMP_CENTER_RIGHT: u8 = 49;
pub const LIGHTBUMP_FRONT_RIGHT: u8 = 50;
pub const LIGHTBUMP_RIGHT: u8 = 51;

/// One decoded sensor reading. `value` is sign-extended into an i32.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reading {
    pub id: u8,
    pub value: i32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeError {
    UnknownSensor(u8),
    ShortPacket { id: u8, need: usize, have: usize },
}

/// Byte width of a sensor packet id (per SENSOR_DATA_WIDTH).
pub fn width(id: u8) -> Option<usize> {
    // SENSOR_DATA_WIDTH from create.py, indices 7..=51.
    const W: [u8; 52] = [
        0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 1, 2, 2, 2, 2, 2,
        2, 2, 1, 2, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2,
    ];
    match W.get(id as usize) {
        Some(0) | None => None,
        Some(&w) => Some(w as usize),
    }
}

/// Whether a sensor's value is two's-complement signed.
pub fn is_signed(id: u8) -> bool {
    // Per create.py's interpreter table: distance(19), angle(20), current(23)
    // use signed 2-byte; battery temp(24) uses signed 1-byte. All else unsigned.
    matches!(id, 19 | 20 | 23 | 24)
}

/// Decode a query-list response: `ids` is the exact list requested (same order
/// passed to `query_list`), `data` is the raw reply. Returns one Reading per id.
pub fn decode(ids: &[u8], data: &[u8]) -> Result<Vec<Reading>, DecodeError> {
    let mut out = Vec::with_capacity(ids.len());
    let mut off = 0usize;
    for &id in ids {
        let w = width(id).ok_or(DecodeError::UnknownSensor(id))?;
        if off + w > data.len() {
            return Err(DecodeError::ShortPacket {
                id,
                need: w,
                have: data.len() - off.min(data.len()),
            });
        }
        let value = match (w, is_signed(id)) {
            (1, false) => data[off] as i32,
            (1, true) => (data[off] as i8) as i32,
            (2, false) => i32::from(u16::from_be_bytes([data[off], data[off + 1]])),
            (2, true) => i32::from(i16::from_be_bytes([data[off], data[off + 1]])),
            _ => unreachable!("widths are only 1 or 2"),
        };
        out.push(Reading { id, value });
        off += w;
    }
    Ok(out)
}

/// Interpret the BUMPS_AND_WHEEL_DROPS byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Bumps {
    pub right_bump: bool,
    pub left_bump: bool,
    pub right_wheel_drop: bool,
    pub left_wheel_drop: bool,
    pub caster_drop: bool,
}

pub fn bumps(byte: i32) -> Bumps {
    Bumps {
        right_bump: byte & 0b00001 != 0,
        left_bump: byte & 0b00010 != 0,
        right_wheel_drop: byte & 0b00100 != 0,
        left_wheel_drop: byte & 0b01000 != 0,
        caster_drop: byte & 0b10000 != 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn widths_match_reference() {
        assert_eq!(width(VOLTAGE), Some(2));
        assert_eq!(width(BATTERY_CHARGE), Some(2));
        assert_eq!(width(BUMPS_AND_WHEEL_DROPS), Some(1));
        assert_eq!(width(OI_MODE), Some(1));
        assert_eq!(width(BATTERY_TEMP), Some(1));
        assert_eq!(width(ENCODER_LEFT), Some(2));
        assert_eq!(width(200), None);
    }

    #[test]
    fn signedness_matches_reference() {
        assert!(is_signed(CURRENT));
        assert!(is_signed(BATTERY_TEMP));
        assert!(!is_signed(VOLTAGE));
        assert!(!is_signed(OI_MODE));
    }

    #[test]
    fn decode_unsigned_two_byte_voltage() {
        // 0x3E80 = 16000 mV
        let r = decode(&[VOLTAGE], &[0x3E, 0x80]).unwrap();
        assert_eq!(r, vec![Reading { id: VOLTAGE, value: 16000 }]);
    }

    #[test]
    fn decode_signed_two_byte_current() {
        // 0xFF38 = -200 mA
        let r = decode(&[CURRENT], &[0xFF, 0x38]).unwrap();
        assert_eq!(r, vec![Reading { id: CURRENT, value: -200 }]);
    }

    #[test]
    fn decode_signed_one_byte_temp() {
        // 0xFB = -5 C
        let r = decode(&[BATTERY_TEMP], &[0xFB]).unwrap();
        assert_eq!(r, vec![Reading { id: BATTERY_TEMP, value: -5 }]);
    }

    #[test]
    fn decode_multiple_in_order() {
        let ids = [VOLTAGE, CURRENT, BATTERY_CHARGE];
        let data = [0x3E, 0x80, 0xFF, 0x38, 0x0A, 0x00];
        let r = decode(&ids, &data).unwrap();
        assert_eq!(
            r,
            vec![
                Reading { id: VOLTAGE, value: 16000 },
                Reading { id: CURRENT, value: -200 },
                Reading { id: BATTERY_CHARGE, value: 2560 },
            ]
        );
    }

    #[test]
    fn decode_mode_and_charging_state() {
        let r = decode(&[OI_MODE, CHARGING_STATE], &[3, 1]).unwrap();
        assert_eq!(r[0].value, 3); // full mode
        assert_eq!(r[1].value, 1);
    }

    #[test]
    fn decode_rejects_short_packet() {
        let err = decode(&[VOLTAGE], &[0x3E]).unwrap_err();
        assert_eq!(err, DecodeError::ShortPacket { id: VOLTAGE, need: 2, have: 1 });
    }

    #[test]
    fn decode_rejects_unknown_sensor() {
        assert_eq!(decode(&[200], &[0]).unwrap_err(), DecodeError::UnknownSensor(200));
    }

    #[test]
    fn bumps_byte_decoding() {
        // bit0 = right bump, bit1 = left bump
        assert_eq!(bumps(0b00001), Bumps { right_bump: true, ..Default::default() });
        assert_eq!(bumps(0b00010), Bumps { left_bump: true, ..Default::default() });
        assert_eq!(
            bumps(0b11111),
            Bumps {
                right_bump: true,
                left_bump: true,
                right_wheel_drop: true,
                left_wheel_drop: true,
                caster_drop: true,
            }
        );
    }
}
