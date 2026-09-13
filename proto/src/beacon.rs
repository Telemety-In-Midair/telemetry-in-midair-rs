//! When the next beacon goes out.
//!
//! The hardware loop passes every 10 ms and asks this once a pass. A
//! beacon is owed when the interval has run out - on the schedule that is
//! the node's turn coming up, one transmission per slot - and it is then
//! planned for a random instant inside the node's turn of the slot's
//! window, and sent when that instant arrives and nothing is arriving on
//! the air. The wait is the loop's, not the transmit's: a transmit that
//! waited inside itself would hold the receiver, the UART and the card
//! for most of a slot.
//!
//! Two things the planner has to get right that a first version did not.
//! A plan can go stale before its instant - the clock re-anchored on a
//! frame heard since, or a frame arriving held the transmit past its turn
//! - and then a fresh plan costs nothing where waiting for the next turn
//! inside the transmit cost a slot. And a plan that is still ahead is kept
//! across passes in slots that are not the node's: on an interval several
//! slots long it names the node's own next slot, and dropping it on the
//! first pass where nothing is owed cost a whole interval.
//!
//! Everything here is integer arithmetic on the clock the loop supplies,
//! so it runs on the host, and the state space tests walk every phase of
//! every slot against every pattern of holds and late passes.

use crate::ble;
use crate::hop::{Clock, Plan};

/// Transmissions a node sends between putting its own name on the air.
///
/// Counted in transmissions rather than seconds so the cost is a fixed
/// fraction of whatever air time the node was already configured to spend:
/// one frame in twenty, at any beacon interval, on a channel that is
/// shared. A receiver switched on beside a fleet therefore learns what
/// every node around it is called within twenty of their beacons, and a
/// fleet beaconing slowly does not pay for a name any more often than it
/// pays for a position.
pub const NAME_EVERY_TX: u32 = 20;

/// When a node announces its name ([`crate::lora::MSG_NAME`]) instead of
/// beaconing.
///
/// A name takes a whole turn, because the schedule allows one transmission
/// per turn - so the question this answers is which turns are worth one.
/// Three are: the first after boot, the first after a rename, and one in
/// [`NAME_EVERY_TX`] thereafter.
///
/// A rename is noticed by comparing labels rather than by being told. The
/// name is written over BLE, on the other core, and a flag set there would
/// have to survive a config write racing a transmission; the label the
/// last announcement carried is a fact this side already knows. It starts
/// empty, which is what makes the first transmission after boot a name.
///
/// A board that has never been named has nothing to announce and beacons
/// every turn, so naming a board is what costs the air time.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct NameCadence {
    /// Transmissions since the last one that carried the name.
    since: u32,
    /// The label that went out with it, zero-padded.
    announced: [u8; ble::NAME_FIELD_LEN],
}

impl Default for NameCadence {
    fn default() -> Self {
        Self::new()
    }
}

impl NameCadence {
    pub const fn new() -> Self {
        Self {
            since: 0,
            announced: [0; ble::NAME_FIELD_LEN],
        }
    }

    /// Whether the next transmission should carry `label`, the name the
    /// node is running under right now.
    pub fn due(&self, label: &str) -> bool {
        !label.is_empty() && (self.since >= NAME_EVERY_TX || self.announced() != label)
    }

    /// Record a transmission that carried the name.
    pub fn sent_name(&mut self, label: &str) {
        self.since = 0;
        self.announced = [0; ble::NAME_FIELD_LEN];
        let n = label.len().min(ble::NAME_LABEL_MAX);
        self.announced[..n].copy_from_slice(&label.as_bytes()[..n]);
    }

    /// Record a transmission that carried something else - a position, a
    /// ping, or one the radio refused.
    ///
    /// A failed transmit counts, for the reason the planner records one:
    /// a node whose radio will not key up must not come back to announce
    /// its name every turn at the expense of the positions.
    pub fn sent_other(&mut self) {
        self.since = self.since.saturating_add(1);
    }

    /// The label the last announcement carried, or `""` before there has
    /// been one.
    fn announced(&self) -> &str {
        let end = self
            .announced
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(ble::NAME_LABEL_MAX);
        core::str::from_utf8(&self.announced[..end]).unwrap_or("")
    }
}

/// What the loop does this pass.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Nothing, or not yet.
    Hold,
    /// Transmit now.
    Send,
}

/// The planner: what was last sent, what is planned next.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Planner {
    /// No beacon before this instant: a fleet powered up together must
    /// not transmit as one.
    first_at_ms: u64,
    /// Local `(start, end)` of the last transmission, as the radio timed
    /// it. What the interval is measured from; recorded for a failed
    /// transmit too, so a radio that will not key up is retried on the
    /// interval rather than on every pass.
    last: Option<(u64, u64)>,
    /// The instant the owed beacon is planned for, once the interval has
    /// run out; `None` while none is.
    at: Option<u64>,
}

impl Planner {
    /// A planner whose first beacon may go out at `first_at_ms`.
    pub const fn new(first_at_ms: u64) -> Self {
        Self {
            first_at_ms,
            last: None,
            at: None,
        }
    }

    /// Record a transmission's span, whatever it carried and whether or
    /// not it succeeded.
    pub fn sent(&mut self, span: (u64, u64)) {
        self.last = Some(span);
    }

    /// The last transmission's span.
    pub fn last(&self) -> Option<(u64, u64)> {
        self.last
    }

    /// The instant the next beacon is planned for, if one is.
    pub fn planned(&self) -> Option<u64> {
        self.at
    }

    /// One pass. `allowed` is whether the node may transmit at all right
    /// now - mode, role, radio, transfer and sleep gates - and
    /// `rx_in_progress` whether a frame is arriving. `interval_ms` is the
    /// beacon's period (0 silences it), `airtime_ms` the frame's time on
    /// air; the clock and the plan are the node's own.
    pub fn pass(
        &mut self,
        now_ms: u64,
        allowed: bool,
        rx_in_progress: bool,
        clock: &mut Clock,
        plan: &Plan,
        address: u8,
        interval_ms: u32,
        airtime_ms: u32,
    ) -> Step {
        let owed = allowed
            && interval_ms != 0
            && now_ms >= self.first_at_ms
            && clock.turn_due(plan, address, self.last.map(|(s, _)| s), interval_ms, now_ms);
        if !owed {
            // A turn that passed while something held the transmit is
            // gone; the next one is planned afresh when it comes. A plan
            // still ahead is kept: it names the node's own next slot, and
            // the slots between are nobody's turn to plan in.
            if self.at.is_some_and(|at| now_ms >= at) {
                self.at = None;
            }
            return Step::Hold;
        }
        if self.at.is_none() {
            self.at = Some(self.start(now_ms, clock, plan, address, interval_ms, airtime_ms));
        }
        let Some(at) = self.at else {
            return Step::Hold;
        };
        if now_ms < at || rx_in_progress {
            return Step::Hold;
        }
        // The plan can be stale by the time its instant arrives: the
        // clock re-anchored on a frame heard since, or a frame arriving
        // held the transmit past its turn. A fresh plan costs nothing.
        if clock.wait_for_window_ms(plan, address, interval_ms, now_ms, airtime_ms) > 0 {
            self.at = Some(self.start(now_ms, clock, plan, address, interval_ms, airtime_ms));
            return Step::Hold;
        }
        self.at = None;
        Step::Send
    }

    /// The instant to plan for from `now_ms`: a random point in the
    /// node's turn, of this slot if it is the node's and nothing has gone
    /// out in it yet, else of the next slot that is. One transmission per
    /// slot, whatever it carries.
    fn start(
        &self,
        now_ms: u64,
        clock: &mut Clock,
        plan: &Plan,
        address: u8,
        interval_ms: u32,
        airtime_ms: u32,
    ) -> u64 {
        let from = match self.last {
            Some((start, _)) if clock.slot(start) == clock.slot(now_ms) => {
                clock.next_slot_start_ms(now_ms)
            }
            _ => now_ms,
        };
        clock.tx_start(plan, address, interval_ms, from, airtime_ms)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radiocfg::RadioConfig;

    /// A named board says so with its first transmission, so a receiver
    /// already listening can put a name to the first position it hears.
    #[test]
    fn the_first_transmission_after_boot_is_the_name() {
        let mut c = NameCadence::new();
        assert!(c.due("sky-1"));
        c.sent_name("sky-1");
        assert!(!c.due("sky-1"));
    }

    /// And then one in twenty, so the name costs a fixed slice of whatever
    /// the node was already spending.
    #[test]
    fn a_name_goes_out_once_every_twenty_transmissions() {
        let mut c = NameCadence::new();
        c.sent_name("sky-1");
        for i in 1..NAME_EVERY_TX {
            c.sent_other();
            assert!(!c.due("sky-1"), "transmission {i}");
        }
        c.sent_other();
        assert!(c.due("sky-1"), "the twentieth is the name's");
        c.sent_name("sky-1");
        assert!(!c.due("sky-1"), "and the count starts again");
    }

    /// A rename reaches the air on the next transmission rather than
    /// twenty later: renaming a board is something an operator does while
    /// watching another board's screen.
    #[test]
    fn a_rename_is_announced_at_once() {
        let mut c = NameCadence::new();
        c.sent_name("sky-1");
        c.sent_other();
        assert!(!c.due("sky-1"));
        assert!(c.due("sky-2"));
        c.sent_name("sky-2");
        assert!(!c.due("sky-2"));
        assert!(c.due("sky-1"), "and back again");
    }

    /// A board that has never been named has nothing to say, so an unnamed
    /// fleet pays nothing for this at all.
    #[test]
    fn an_unnamed_board_never_announces() {
        let mut c = NameCadence::new();
        assert!(!c.due(""));
        for _ in 0..NAME_EVERY_TX * 2 {
            c.sent_other();
            assert!(!c.due(""));
        }
        // Naming it is what starts the announcements; clearing the name
        // stops them without waiting for anything.
        assert!(c.due("sky-1"));
        c.sent_name("sky-1");
        assert!(!c.due(""));
    }

    /// The longest label a board stores is remembered whole, or the next
    /// transmission would read as a rename and every turn would be one.
    #[test]
    fn the_longest_label_is_remembered() {
        let long = "a".repeat(ble::NAME_LABEL_MAX);
        let mut c = NameCadence::new();
        c.sent_name(&long);
        assert!(!c.due(&long));
        assert!(c.due(&long[..ble::NAME_LABEL_MAX - 1]), "one byte shorter is a rename");
    }

    /// A radio that will not key up must not turn every turn into a name
    /// announcement: a transmission counts whether or not it went out, the
    /// way the planner counts one.
    #[test]
    fn a_failed_transmission_still_counts() {
        let mut c = NameCadence::new();
        c.sent_name("sky-1");
        for _ in 0..NAME_EVERY_TX - 1 {
            c.sent_other();
        }
        assert!(!c.due("sky-1"));
    }

    fn setup() -> (Clock, Plan) {
        let cfg = RadioConfig::default();
        let plan = Plan::from_config(&cfg);
        let mut clock = Clock::new(plan.dwell_ms, 0, 1);
        clock.discipline_gps(0, 0);
        (clock, plan)
    }

    /// The ordinary second: owed at the slot boundary, planned into the
    /// turn, sent when the instant arrives, then not again in the slot.
    #[test]
    fn a_beacon_is_planned_into_the_turn_and_sent_once_a_slot() {
        let (mut clock, plan) = setup();
        let mut p = Planner::new(0);
        let mut sent_at = None;
        for t in (10_000..12_000).step_by(10) {
            if p.pass(t, true, false, &mut clock, &plan, 1, 1000, 289) == Step::Send {
                sent_at = Some(t);
                p.sent((t, t + 289));
                break;
            }
        }
        let t = sent_at.expect("a beacon went out");
        let phase = clock.phase_ms(t);
        assert!((100..=165).contains(&phase), "phase {phase}");
        // Not again in this slot.
        for t2 in (t..11_000).step_by(10) {
            assert_eq!(p.pass(t2, true, false, &mut clock, &plan, 1, 1000, 289), Step::Hold);
        }
        assert!(p.planned().is_none());
    }

    /// The regression: on a five-slot interval a pass that comes late in
    /// the node's slot plans the node's next slot, and the plan survives
    /// the passes in between rather than being dropped on the first slot
    /// that is nobody's turn.
    #[test]
    fn a_late_pass_costs_a_slot_not_an_interval() {
        let (mut clock, plan) = setup();
        let mut p = Planner::new(0);
        // Node 1 owns slots 0 mod 5. Its first pass in slot 10 lands at
        // phase 300, past its turn.
        assert_eq!(p.pass(10_300, true, false, &mut clock, &plan, 1, 5_000, 289), Step::Hold);
        let at = p.planned().expect("planned");
        assert_eq!(clock.slot(at) % 5, 0, "planned for the node's own slot");
        assert!(at >= 15_000 && at < 16_000, "{at}");
        // Passes through slots 11-14: nothing owed, plan kept.
        for t in (10_400..15_000).step_by(100) {
            assert_eq!(p.pass(t, true, false, &mut clock, &plan, 1, 5_000, 289), Step::Hold);
            assert_eq!(p.planned(), Some(at), "at {t}");
        }
        // And it goes out in slot 15 at the planned instant.
        let mut sent = None;
        for t in (15_000..16_000).step_by(10) {
            if p.pass(t, true, false, &mut clock, &plan, 1, 5_000, 289) == Step::Send {
                sent = Some(t);
                break;
            }
        }
        assert!(sent.is_some_and(|t| t >= at && t < at + 10), "{sent:?} for {at}");
    }

    /// A frame arriving holds the transmit; once it has passed the turn,
    /// the plan is remade rather than waited for inside the transmit.
    #[test]
    fn a_frame_arriving_holds_and_a_stale_plan_is_remade() {
        let (mut clock, plan) = setup();
        let mut p = Planner::new(0);
        p.pass(10_000, true, false, &mut clock, &plan, 1, 1000, 289);
        let at = p.planned().unwrap();
        // Held through its instant by a frame on the air.
        for t in (at..at + 200).step_by(10) {
            assert_eq!(p.pass(t, true, true, &mut clock, &plan, 1, 1000, 289), Step::Hold);
        }
        // The frame is gone but the turn is too: re-planned, not sent.
        let t = at + 200;
        assert_eq!(p.pass(t, true, false, &mut clock, &plan, 1, 1000, 289), Step::Hold);
        let again = p.planned().unwrap();
        assert!(again > t, "{again} > {t}");
        assert_eq!(clock.slot(again), clock.slot(t) + 1);
    }

    /// Nothing before the first-beacon stagger, nothing when not allowed,
    /// nothing at interval zero.
    #[test]
    fn the_gates_hold() {
        let (mut clock, plan) = setup();
        let mut p = Planner::new(20_000);
        for t in (10_000..20_000).step_by(100) {
            assert_eq!(p.pass(t, true, false, &mut clock, &plan, 1, 1000, 289), Step::Hold);
        }
        let mut p = Planner::new(0);
        for t in (10_000..12_000).step_by(100) {
            assert_eq!(p.pass(t, false, false, &mut clock, &plan, 1, 1000, 289), Step::Hold);
            assert_eq!(p.pass(t, true, false, &mut clock, &plan, 1, 0, 289), Step::Hold);
        }
        assert!(p.planned().is_none());
    }
}
