//! Reflex latch: the safety layer below the planner (SPEC §4.1).
//!
//! A bump latches and suppresses forward motion until the planner acknowledges
//! by commanding a recovery (a non-forward `Twist`). Wheel-drops are *not*
//! latched here — they gate motion in the control loop for exactly as long as
//! the drop is live (see `ControlCore`). Cliff sensing is intentionally not
//! wired up yet. This struct is pure; the loop feeds it decoded sensor state and
//! the planner's intent each tick.

use escape_core::{Contact, Reflex};

/// Detect a fresh latchable reflex (a bump) from this tick's sensors. `None`
/// means nothing new to latch.
pub fn detect(contact: Contact) -> Option<Reflex> {
    if contact.bump_left || contact.bump_right {
        return Some(Reflex::Bump { left: contact.bump_left, right: contact.bump_right });
    }
    None
}

/// Latched bump state.
#[derive(Debug, Clone, Copy, Default)]
pub struct ReflexLatch {
    active: Option<Reflex>,
}

impl ReflexLatch {
    /// Record a freshly-detected reflex (overwrites any prior latch with the
    /// newer cause). Call once per tick with `detect`'s result.
    pub fn observe(&mut self, fresh: Option<Reflex>) {
        if fresh.is_some() {
            self.active = fresh;
        }
    }

    /// Clear the latch if the planner's commanded `Twist` is a recovery — i.e.
    /// it is not driving forward (reverse, pure spin, or stop). This is how the
    /// planner "acknowledges" a reflex.
    pub fn acknowledge_if_recovering(&mut self, desired_v_mm_s: f64) {
        if desired_v_mm_s <= 0.0 {
            self.active = None;
        }
    }

    pub fn active(&self) -> Option<Reflex> {
        self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bump_is_detected_with_side() {
        let c = Contact { bump_left: true, bump_right: false, wheel_drop: false };
        assert_eq!(detect(c), Some(Reflex::Bump { left: true, right: false }));
    }

    #[test]
    fn nothing_latches_on_clear_floor() {
        assert_eq!(detect(Contact::default()), None);
    }

    #[test]
    fn latch_holds_until_recovery_command() {
        let mut latch = ReflexLatch::default();
        latch.observe(Some(Reflex::Bump { left: true, right: false }));
        assert!(latch.active().is_some());

        // Planner keeps commanding forward → stays latched.
        latch.acknowledge_if_recovering(120.0);
        assert!(latch.active().is_some());

        // Planner commands a reverse (recovery) → clears.
        latch.acknowledge_if_recovering(-80.0);
        assert!(latch.active().is_none());
    }

    #[test]
    fn pure_spin_counts_as_recovery() {
        let mut latch = ReflexLatch::default();
        latch.observe(Some(Reflex::Bump { left: false, right: true }));
        latch.acknowledge_if_recovering(0.0); // v=0, only turning
        assert!(latch.active().is_none());
    }
}
