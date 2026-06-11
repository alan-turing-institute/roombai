//! The per-tick control decision, factored out of the I/O loop so it is
//! deterministic and unit-testable (SPEC §5: watchdog/reflex/timing tests with
//! no serial port). The threaded loop in `lib.rs` only does I/O and timing; all
//! policy lives here.

use std::collections::BTreeMap;
use std::time::Duration;

use escape_core::{Contact, Reflex, SensorFrame, Twist};
use roomba_pilot::sensors;

use crate::config::Config;
use crate::kinematics::{twist_to_wheels, MotionShaper};
use crate::odometry::{OdomSource, Odometry};
use crate::reflex::{self, ReflexLatch};

/// Decoded sensor values for one tick, lifted out of the raw id→value map so
/// the control logic never touches packet ids.
#[derive(Debug, Clone, Copy, Default)]
pub struct RawSensors {
    pub enc_left: Option<i32>,
    pub enc_right: Option<i32>,
    pub int_dist_mm: Option<f64>,
    pub int_angle_deg: Option<f64>,
    pub contact: Contact,
    pub light_bumps: [u16; 6],
    pub battery_charge_mah: Option<u16>,
    pub oi_mode: Option<u8>,
}

impl RawSensors {
    /// Pull the fields we care about out of a decoded query-list map. Missing
    /// ids stay `None`/`0`; unsigned packets are clamped at zero.
    pub fn from_readings(m: &BTreeMap<u8, i32>) -> Self {
        let u16_of = |id: u8| m.get(&id).map(|&v| v.max(0) as u16);
        let light = |id: u8| u16_of(id).unwrap_or(0);
        let b7 = m.get(&sensors::BUMPS_AND_WHEEL_DROPS).copied().unwrap_or(0);
        RawSensors {
            enc_left: m.get(&sensors::ENCODER_LEFT).copied(),
            enc_right: m.get(&sensors::ENCODER_RIGHT).copied(),
            int_dist_mm: m.get(&19).map(|&v| v as f64),
            int_angle_deg: m.get(&20).map(|&v| v as f64),
            contact: Contact {
                bump_right: b7 & 0x01 != 0,
                bump_left: b7 & 0x02 != 0,
                wheel_drop: b7 & 0x1C != 0, // either wheel or the caster
            },
            light_bumps: [light(46), light(47), light(48), light(49), light(50), light(51)],
            battery_charge_mah: u16_of(sensors::BATTERY_CHARGE),
            oi_mode: m.get(&sensors::OI_MODE).map(|&v| v as u8),
        }
    }
}

/// Result of advancing one tick: the wheel command to send and the frame to
/// publish.
pub struct Tick {
    pub wheels: (i16, i16),
    pub frame: SensorFrame,
}

/// Holds all per-tick control state (odometry, motion shaper, reflex latch).
pub struct ControlCore {
    cfg: Config,
    source: OdomSource,
    odom: Odometry,
    shaper: MotionShaper,
    latch: ReflexLatch,
    seq: u64,
}

impl ControlCore {
    pub fn new(cfg: Config, source: OdomSource) -> Self {
        ControlCore {
            odom: Odometry::new(cfg.odom_span_mm),
            shaper: MotionShaper::default(),
            latch: ReflexLatch::default(),
            seq: 0,
            source,
            cfg,
        }
    }

    /// Advance one control tick.
    ///
    /// - `desired`: the planner's latest setpoint.
    /// - `setpoint_age`: how long since that setpoint was set (watchdog input).
    /// - `raw`: this tick's decoded sensors.
    /// - `t_s`: seconds since loop start, stamped onto the frame.
    pub fn step(
        &mut self,
        desired: Twist,
        setpoint_age: Duration,
        raw: &RawSensors,
        t_s: f64,
    ) -> Tick {
        // 1. Watchdog: a stale setpoint means "no one is steering" → stop.
        let desired = if setpoint_age > self.cfg.watchdog { Twist::STOP } else { desired };

        // 2. Odometry from the chosen source.
        match self.source {
            OdomSource::Encoders => {
                if let (Some(l), Some(r)) = (raw.enc_left, raw.enc_right) {
                    self.odom.update_encoders(l, r);
                }
            }
            OdomSource::Integrated => {
                if let (Some(d), Some(a)) = (raw.int_dist_mm, raw.int_angle_deg) {
                    self.odom.update_integrated(d, a);
                }
            }
        }

        // 3. Reflexes: latch a new bump, then decide what's allowed.
        let fresh = reflex::detect(raw.contact);
        self.latch.observe(fresh);

        let mut allowed = desired;
        // While a bump is latched, refuse forward motion (reverse/turn ok).
        if self.latch.active().is_some() && allowed.v_mm_s > 0.0 {
            allowed.v_mm_s = 0.0;
        }
        // Wheel-drop is an unconditional hard stop for as long as it's live.
        if raw.contact.wheel_drop {
            allowed = Twist::STOP;
        }

        // 4. Smooth toward the allowed command and convert to wheels.
        let shaped = self.shaper.shape(
            allowed,
            self.cfg.dt(),
            self.cfg.max_lin_accel_mm_s2,
            self.cfg.max_ang_accel_rad_s2,
        );
        let wheels = twist_to_wheels(shaped, self.cfg.cmd_span_mm);

        // 5. The planner acknowledges a reflex by commanding recovery.
        self.latch.acknowledge_if_recovering(desired.v_mm_s);

        // 6. Report wheel-drop ahead of any latched bump in the frame.
        let reported = if raw.contact.wheel_drop {
            Some(Reflex::WheelDrop)
        } else {
            self.latch.active()
        };

        let frame = SensorFrame {
            t_s,
            seq: self.seq,
            pose: self.odom.pose(),
            pose_degraded: self.source.is_degraded(),
            contact: raw.contact,
            light_bumps: raw.light_bumps,
            battery_charge_mah: raw.battery_charge_mah,
            oi_mode: raw.oi_mode,
            reflex: reported,
        };
        self.seq += 1;

        Tick { wheels, frame }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config::default()
    }

    fn enc(core: &mut ControlCore, desired: Twist, l: i32, r: i32, t: f64) -> Tick {
        let raw = RawSensors { enc_left: Some(l), enc_right: Some(r), ..Default::default() };
        core.step(desired, Duration::ZERO, &raw, t)
    }

    #[test]
    fn fresh_forward_setpoint_drives_forward() {
        let mut core = ControlCore::new(cfg(), OdomSource::Encoders);
        enc(&mut core, Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, 1000, 1000, 0.0); // seed
        let t = enc(&mut core, Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, 1010, 1010, 0.05);
        // accel-limited: 150 mm/s² * 0.05 s = 7.5 mm/s after the first move tick.
        assert!(t.wheels.0 > 0 && t.wheels.1 > 0);
    }

    #[test]
    fn stale_setpoint_is_zeroed_by_watchdog() {
        let mut core = ControlCore::new(cfg(), OdomSource::Encoders);
        let raw = RawSensors { enc_left: Some(1000), enc_right: Some(1000), ..Default::default() };
        let stale = Duration::from_millis(500); // > 300 ms watchdog
        let t = core.step(Twist { v_mm_s: 250.0, omega_rad_s: 0.0 }, stale, &raw, 0.0);
        assert_eq!(t.wheels, (0, 0));
    }

    #[test]
    fn bump_suppresses_forward_but_allows_reverse() {
        let mut core = ControlCore::new(cfg(), OdomSource::Encoders);
        let bumped = RawSensors {
            contact: Contact { bump_left: true, bump_right: true, wheel_drop: false },
            ..Default::default()
        };
        // Planner still wants forward → wheels held at zero, reflex reported.
        let t = core.step(Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, Duration::ZERO, &bumped, 0.0);
        assert_eq!(t.wheels, (0, 0));
        assert!(matches!(t.frame.reflex, Some(Reflex::Bump { .. })));

        // Planner reverses → allowed (ramps negative) and latch clears.
        let clear = RawSensors::default();
        let t2 = core.step(Twist { v_mm_s: -100.0, omega_rad_s: 0.0 }, Duration::ZERO, &clear, 0.05);
        assert!(t2.wheels.0 < 0 && t2.wheels.1 < 0);
        assert!(t2.frame.reflex.is_none());
    }

    #[test]
    fn wheel_drop_forces_full_stop_regardless() {
        let mut core = ControlCore::new(cfg(), OdomSource::Encoders);
        // Build up some speed first.
        enc(&mut core, Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, 1000, 1000, 0.0);
        enc(&mut core, Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, 1100, 1100, 0.05);
        let dropped = RawSensors {
            contact: Contact { bump_left: false, bump_right: false, wheel_drop: true },
            ..Default::default()
        };
        let t = core.step(Twist { v_mm_s: 200.0, omega_rad_s: 0.0 }, Duration::ZERO, &dropped, 0.1);
        // Shaped toward zero; with a wheel drop the target is STOP.
        assert!(t.wheels.0 < 200 && t.wheels.1 < 200);
        assert!(matches!(t.frame.reflex, Some(Reflex::WheelDrop)));
    }

    #[test]
    fn integrated_source_flags_degraded_pose() {
        let mut core = ControlCore::new(cfg(), OdomSource::Integrated);
        let raw = RawSensors {
            int_dist_mm: Some(10.0),
            int_angle_deg: Some(0.0),
            ..Default::default()
        };
        let t = core.step(Twist::STOP, Duration::ZERO, &raw, 0.0);
        assert!(t.frame.pose_degraded);
        assert!((t.frame.pose.x_mm - 10.0).abs() < 1e-9);
    }

    #[test]
    fn seq_increments_each_tick() {
        let mut core = ControlCore::new(cfg(), OdomSource::Encoders);
        let a = enc(&mut core, Twist::STOP, 1000, 1000, 0.0);
        let b = enc(&mut core, Twist::STOP, 1000, 1000, 0.05);
        assert_eq!((a.frame.seq, b.frame.seq), (0, 1));
    }
}
