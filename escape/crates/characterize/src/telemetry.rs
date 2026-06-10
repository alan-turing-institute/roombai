//! Per-tick sensor sampling and dual-source odometry accumulation.
//!
//! Everything is logged from both odometry sources (raw encoders 43/44 and the
//! integrating distance/angle packets 19/20) every tick; which source drives
//! the runner's own decisions is chosen once, from the probe results.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;
use std::io::{self, Read, Write};

use serde_json::{json, Value};

use crate::rig::Rig;

pub const MM_PER_TICK: f64 = 72.0 * PI / 508.8; // wheel dia 72 mm, 508.8 ticks/rev
pub const WHEEL_SPAN_MM: f64 = 235.0;

/// Packets sampled every control tick (intersected with what the 770 supports).
pub const TICK_IDS: &[u8] = &[
    7,  // bumps + wheel drops
    19, // integrated distance since last read (mm)
    20, // integrated angle since last read (deg)
    25, // battery charge
    28, 29, 30, 31, // cliff signals (floor reflectivity)
    35, // OI mode (detect safety fallback to passive)
    43, 44, // raw wheel encoders
    46, 47, 48, 49, 50, 51, // light bump signals
];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OdomSource {
    Encoders,
    Integrated,
    None,
}

pub struct Caps {
    pub supported: BTreeSet<u8>,
    pub probe_values: BTreeMap<u8, i32>,
    pub stream_frames: u32,
    pub source: OdomSource,
    pub tick_ids: Vec<u8>,
}

impl Caps {
    pub fn build(supported: BTreeSet<u8>, probe_values: BTreeMap<u8, i32>, stream_frames: u32) -> Self {
        let source = if supported.contains(&43) && supported.contains(&44) {
            OdomSource::Encoders
        } else if supported.contains(&19) && supported.contains(&20) {
            OdomSource::Integrated
        } else {
            OdomSource::None
        };
        let tick_ids: Vec<u8> = TICK_IDS.iter().copied().filter(|id| supported.contains(id)).collect();
        Caps { supported, probe_values, stream_frames, source, tick_ids }
    }

    pub fn to_json(&self) -> Value {
        json!({
            "supported": self.supported.iter().copied().collect::<Vec<u8>>(),
            "probe_values": self.probe_values.iter().map(|(k, v)| (k.to_string(), *v)).collect::<BTreeMap<_, _>>(),
            "stream_frames_in_2s": self.stream_frames,
            "odom_source": format!("{:?}", self.source),
        })
    }
}

/// Cumulative odometry from both sources, updated every tick.
#[derive(Default)]
pub struct Odom {
    prev_enc: Option<(i32, i32)>,
    pub enc_dist_mm: f64,
    pub enc_angle_deg: f64, // +CCW
    pub i_dist_mm: f64,
    pub i_angle_deg: f64,
}

/// Wrap-aware encoder delta (counts are 16-bit and roll over).
fn wrap16(cur: i32, prev: i32) -> i16 {
    (cur as u16).wrapping_sub(prev as u16) as i16
}

impl Odom {
    pub fn update(&mut self, readings: &BTreeMap<u8, i32>) {
        if let (Some(&l), Some(&r)) = (readings.get(&43), readings.get(&44)) {
            if let Some((pl, pr)) = self.prev_enc {
                let dl = wrap16(l, pl) as f64 * MM_PER_TICK;
                let dr = wrap16(r, pr) as f64 * MM_PER_TICK;
                self.enc_dist_mm += (dl + dr) / 2.0;
                self.enc_angle_deg += (dr - dl) / WHEEL_SPAN_MM * 180.0 / PI;
            }
            self.prev_enc = Some((l, r));
        }
        if let Some(&d) = readings.get(&19) {
            self.i_dist_mm += d as f64;
        }
        if let Some(&a) = readings.get(&20) {
            self.i_angle_deg += a as f64;
        }
    }

    pub fn snapshot(&self) -> OdomSnapshot {
        OdomSnapshot {
            enc_dist_mm: self.enc_dist_mm,
            enc_angle_deg: self.enc_angle_deg,
            i_dist_mm: self.i_dist_mm,
            i_angle_deg: self.i_angle_deg,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub struct OdomSnapshot {
    pub enc_dist_mm: f64,
    pub enc_angle_deg: f64,
    pub i_dist_mm: f64,
    pub i_angle_deg: f64,
}

pub struct Tick {
    pub readings: BTreeMap<u8, i32>,
    pub bump_left: bool,
    pub bump_right: bool,
    pub wheel_drop: bool,
    pub oi_mode: Option<i32>,
}

pub fn tick<T: Read + Write>(rig: &mut Rig<T>, caps: &Caps, odom: &mut Odom) -> io::Result<Tick> {
    let rs = rig.query(&caps.tick_ids)?;
    let readings: BTreeMap<u8, i32> = rs.iter().map(|r| (r.id, r.value)).collect();
    odom.update(&readings);
    let b7 = readings.get(&7).copied().unwrap_or(0);
    Ok(Tick {
        bump_right: b7 & 0x01 != 0,
        bump_left: b7 & 0x02 != 0,
        wheel_drop: b7 & 0x0C != 0,
        oi_mode: readings.get(&35).copied(),
        readings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap16_handles_rollover() {
        assert_eq!(wrap16(5, 65530), 11);
        assert_eq!(wrap16(65530, 5), -11);
        assert_eq!(wrap16(100, 90), 10);
    }

    #[test]
    fn encoder_spin_in_place_accumulates_angle_not_distance() {
        let mut o = Odom::default();
        o.update(&BTreeMap::from([(43u8, 1000i32), (44u8, 1000i32)]));
        // right wheel +100 ticks, left -100 ticks => pure CCW rotation
        o.update(&BTreeMap::from([(43u8, 900i32), (44u8, 1100i32)]));
        assert!(o.enc_dist_mm.abs() < 1e-9);
        let expected = 200.0 * MM_PER_TICK / WHEEL_SPAN_MM * 180.0 / PI;
        assert!((o.enc_angle_deg - expected).abs() < 1e-9);
        assert!(o.enc_angle_deg > 0.0); // CCW positive
    }

    #[test]
    fn integrated_packets_accumulate() {
        let mut o = Odom::default();
        o.update(&BTreeMap::from([(19u8, 50i32), (20u8, -10i32)]));
        o.update(&BTreeMap::from([(19u8, 25i32), (20u8, -5i32)]));
        assert_eq!(o.i_dist_mm, 75.0);
        assert_eq!(o.i_angle_deg, -15.0);
    }
}
