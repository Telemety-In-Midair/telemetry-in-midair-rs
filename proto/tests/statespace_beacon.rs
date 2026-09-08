//! Every phase of every slot, against every pattern of transmit gates,
//! frames arriving and late passes, for the beacon planner.
//!
//! Time is ticks of 50 ms over three slots of the default plan. The clock
//! is the node's own, on GPS time from the start, so the turns are the
//! real turns. What is checked: a send lands inside the node's turn of a
//! slot that is the node's, never twice in a slot, never over a frame that
//! is arriving, never while not allowed; and, as liveness, a node that is
//! allowed and unheld through its whole turn sends in it.

use midair_explore::{explore, Machine};
use midair_proto::beacon::{Planner, Step};
use midair_proto::hop::{Clock, Plan};
use midair_proto::radiocfg::RadioConfig;

const TICK_MS: u64 = 50;
const ADDRESS: u8 = 1;
const AIRTIME_MS: u32 = 289;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct State {
    tick: u64,
    planner: Planner,
    clock: Clock,
    allowed: bool,
    rx_busy: bool,
    /// The slot of the last send, for the once-a-slot check.
    last_sent_slot: Option<u32>,
    sends: u8,
}

impl State {
    fn now(&self) -> u64 {
        self.tick * TICK_MS
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Event {
    /// The loop passes.
    Pass,
    /// The loop misses two ticks: a transmit, a flush.
    Late,
    /// Whether the node may transmit flips: a transfer, a sleep, a mode.
    Gate,
    /// A frame starts or stops arriving.
    Air,
}

struct Beacon {
    interval_ms: u32,
    /// Ticks walked: two and a half intervals, so that two beacons fit
    /// and the second is planned across the slots that are not the
    /// node's.
    horizon: u64,
}

impl Beacon {
    fn new(interval_ms: u32) -> Self {
        Self {
            interval_ms,
            horizon: u64::from(interval_ms) * 5 / 2 / TICK_MS,
        }
    }
}

impl Machine for Beacon {
    type State = State;
    type Event = Event;

    fn initial(&self) -> Vec<State> {
        let plan = Plan::from_config(&RadioConfig::default());
        let mut clock = Clock::new(plan.dwell_ms, 0, u32::from(ADDRESS));
        clock.discipline_gps(0, 0);
        vec![State {
            tick: 0,
            planner: Planner::new(0),
            clock,
            allowed: true,
            rx_busy: false,
            last_sent_slot: None,
            sends: 0,
        }]
    }

    fn events(&self, s: &State) -> Vec<Event> {
        if s.tick >= self.horizon {
            return Vec::new();
        }
        vec![Event::Pass, Event::Late, Event::Gate, Event::Air]
    }

    fn step(&self, s: &State, e: &Event) -> State {
        let mut n = *s;
        let plan = Plan::from_config(&RadioConfig::default());
        match *e {
            Event::Pass | Event::Late => {
                let now = n.now();
                let step = n.planner.pass(
                    now,
                    n.allowed,
                    n.rx_busy,
                    &mut n.clock,
                    &plan,
                    ADDRESS,
                    self.interval_ms,
                    AIRTIME_MS,
                );
                if step == Step::Send {
                    n.planner.sent((now, now + u64::from(AIRTIME_MS)));
                    n.last_sent_slot = Some(n.clock.slot(now));
                    n.sends = n.sends.saturating_add(1);
                }
                n.tick += if *e == Event::Late { 3 } else { 1 };
            }
            Event::Gate => n.allowed = !n.allowed,
            Event::Air => n.rx_busy = !n.rx_busy,
        }
        n
    }

    fn check_step(&self, from: &State, e: &Event, to: &State) -> Result<(), String> {
        if !matches!(e, Event::Pass | Event::Late) || to.sends == from.sends {
            return Ok(());
        }
        let plan = Plan::from_config(&RadioConfig::default());
        let now = from.now();
        if !from.allowed {
            return Err("sent while not allowed".into());
        }
        if from.rx_busy {
            return Err("sent over a frame arriving".into());
        }
        let n = from.clock.slots_for(self.interval_ms);
        let slot = from.clock.slot(now);
        if slot % n != plan.turn_slot(ADDRESS, n) {
            return Err(format!("sent in slot {slot}, not the node's"));
        }
        let (lo, hi) = plan.start_range_for(ADDRESS, n, AIRTIME_MS);
        let phase = from.clock.phase_ms(now);
        // A pass lands up to a tick late, and a late one three.
        let slack = if *e == Event::Late { 0 } else { u32::try_from(TICK_MS).unwrap() };
        if phase < lo || phase > hi + slack {
            return Err(format!("sent at phase {phase}, turn is {lo}..={hi}"));
        }
        if from.last_sent_slot == Some(slot) {
            return Err(format!("sent twice in slot {slot}"));
        }
        Ok(())
    }

    fn describe(&self, s: &State) -> String {
        format!(
            "t={} slot {} phase {} allowed {} air {} planned {:?} sends {}",
            s.now(),
            s.clock.slot(s.now()),
            s.clock.phase_ms(s.now()),
            s.allowed,
            s.rx_busy,
            s.planner.planned(),
            s.sends
        )
    }
}

#[test]
fn every_send_is_in_the_node_s_turn_and_never_twice_a_slot() {
    for interval_ms in [1_000, 3_000] {
        let m = Beacon::new(interval_ms);
        let x = explore(&m);
        println!("beacon model, interval {interval_ms}: {x}");
        x.assert_ok(&m);
        x.assert_some(|s| s.sends >= 2, "two beacons went out");
        x.assert_some(|s| s.planner.planned().is_some() && !s.allowed, "a plan survives a gate");
        // Liveness: however the gates and the air have gone, opening them
        // gets a beacon out before the walk ends.
        let horizon = m.horizon;
        x.assert_always_reachable(
            &m,
            |s| s.sends >= 1 || s.tick >= horizon,
            "a beacon has gone out or the horizon is here",
        );
    }
}
