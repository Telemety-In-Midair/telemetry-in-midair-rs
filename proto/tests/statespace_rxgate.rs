//! Every sequence of interrupts the receive poll can read, at every phase
//! of the hold, against every slot boundary and transmit the loop can put
//! between them.
//!
//! The gate's job is to say how long a hop or a transmit has to wait for a
//! frame that may be arriving, and the two ways it can be wrong are
//! opposite: a hold that never ends pins the receiver, and a hold that
//! ends early lets a transmit trample a frame. The walk checks both at
//! every reachable point: a preamble-only hold lasts a header time and no
//! longer, a header's hold lasts a whole frame from the moment a frame is
//! first known to be arriving, a valid header seen after an earlier hold
//! lapsed starts a fresh one, and a hop is refused exactly while a frame
//! is inside its hold and for at most a slot.
//!
//! Time is ticks of 20 ms - two poll periods - over a horizon that outlasts
//! the longest hold, so a state is a tick and what the gate knows.

use midair_explore::{explore, Machine};
use midair_proto::rxgate::{Irq, RxGate, Seen, Stage};

/// One tick, ms: two poll periods, so a late poll is one skipped tick.
const TICK_MS: u64 = 20;
/// Ticks in the horizon: 600 ms, past the 452 ms longest frame.
const HORIZON: u64 = 30;
/// The default modulation's numbers, as the driver computes them.
const HEADER_MS: u32 = 186;
const MAX_FRAME_MS: u32 = 452;
const DWELL_MS: u32 = 1_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct State {
    tick: u64,
    gate: RxGate,
}

impl State {
    fn now(&self) -> u64 {
        self.tick * TICK_MS
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Event {
    /// The loop polls and reads these bits.
    Poll(Irq),
    /// The loop misses a tick: held by a card flush or a transmit.
    Skip,
    /// A slot boundary: the hop asks whether it may leave.
    Hop,
    /// The loop transmits, which it only does when nothing is arriving.
    Tx,
}

struct Gate;

/// Every combination of interrupt bits a single read can carry that
/// means anything different to the gate.
fn irqs() -> Vec<Irq> {
    let mut out = Vec::new();
    for preamble in [false, true] {
        for header_valid in [false, true] {
            for header_err in [false, true] {
                for (rx_done, crc_err) in [(false, false), (true, false), (true, true)] {
                    // A valid and an erroneous header in one read is not
                    // something the chip produces.
                    if header_valid && header_err {
                        continue;
                    }
                    out.push(Irq {
                        preamble,
                        header_valid,
                        header_err,
                        rx_done,
                        crc_err,
                    });
                }
            }
        }
    }
    out
}

impl Machine for Gate {
    type State = State;
    type Event = Event;

    fn initial(&self) -> Vec<State> {
        vec![State {
            tick: 0,
            gate: RxGate::new(HEADER_MS, MAX_FRAME_MS),
        }]
    }

    fn events(&self, s: &State) -> Vec<Event> {
        if s.tick >= HORIZON {
            return Vec::new();
        }
        let mut ev: Vec<Event> = irqs().into_iter().map(Event::Poll).collect();
        ev.push(Event::Skip);
        ev.push(Event::Hop);
        if !s.gate.in_progress(s.now()) {
            ev.push(Event::Tx);
        }
        ev
    }

    fn step(&self, s: &State, e: &Event) -> State {
        let mut n = *s;
        let now = n.now();
        match *e {
            Event::Poll(irq) => {
                n.gate.begin_poll(now);
                let _ = n.gate.observe(now, irq);
            }
            Event::Skip => {}
            Event::Hop => {
                let _ = n.gate.may_leave(now, DWELL_MS);
            }
            Event::Tx => n.gate.clear(),
        }
        n.tick += 1;
        n
    }

    fn check(&self, s: &State) -> Result<(), String> {
        let now = s.now();
        // No hold outlasts the longest frame, and a preamble alone is held
        // no longer than a header would take to follow it.
        if s.gate.in_progress(now) {
            let hold = s.gate.hold_ms();
            match s.gate.stage() {
                Some(Stage::Preamble) if hold != HEADER_MS => {
                    return Err(format!("a preamble held for {hold} ms"));
                }
                Some(Stage::Header) if hold != MAX_FRAME_MS => {
                    return Err(format!("a header held for {hold} ms"));
                }
                None => return Err("in progress with nothing seen".into()),
                _ => {}
            }
        }
        Ok(())
    }

    fn check_step(&self, from: &State, e: &Event, to: &State) -> Result<(), String> {
        let now = from.now();
        match *e {
            Event::Poll(irq) => {
                let seen = {
                    let mut g = from.gate;
                    g.begin_poll(now);
                    g.observe(now, irq)
                };
                if irq.rx_done {
                    if seen != (Seen::Packet { crc_ok: !irq.crc_err }) {
                        return Err("a packet end was not reported as one".into());
                    }
                    if to.gate.in_progress(now) {
                        return Err("a packet end left a hold behind".into());
                    }
                } else if irq.header_err {
                    if to.gate.in_progress(now) {
                        return Err("a header error left a hold behind".into());
                    }
                } else if irq.header_valid {
                    // A frame is known to be arriving: the hold reaches a
                    // whole frame ahead of wherever it is measured from,
                    // and if nothing was arriving before, from now.
                    if seen != Seen::Arriving {
                        return Err("a valid header was not reported as arriving".into());
                    }
                    if !from.gate.in_progress(now)
                        && !to.gate.in_progress(now + u64::from(MAX_FRAME_MS) - 1)
                    {
                        return Err("a valid header after a lapsed hold did not start a fresh one".into());
                    }
                    if to.gate.stage() != Some(Stage::Header) {
                        return Err("a valid header did not lift the stage".into());
                    }
                } else if irq.preamble {
                    if !to.gate.in_progress(now) {
                        return Err("a preamble started no hold".into());
                    }
                    // A re-fire on noise does not extend an existing hold.
                    if from.gate.in_progress(now) && from.gate.stage() == Some(Stage::Preamble) {
                        let end = |g: &RxGate| {
                            (0..=u64::from(MAX_FRAME_MS))
                                .find(|&d| !g.in_progress(now + d))
                                .unwrap_or(u64::MAX)
                        };
                        if end(&to.gate) != end(&from.gate) {
                            return Err("a repeated preamble extended the hold".into());
                        }
                    }
                } else if to.gate != {
                    let mut g = from.gate;
                    g.begin_poll(now);
                    g
                } {
                    return Err("an empty read changed what is known".into());
                }
            }
            Event::Hop => {
                let held = from.gate.in_progress(now);
                if held && to.gate != from.gate {
                    return Err("a hop cleared a frame inside its hold".into());
                }
                if !held && to.gate.in_progress(now) {
                    return Err("a hop left a lapsed hold in place".into());
                }
            }
            Event::Tx => {
                if from.gate.in_progress(now) {
                    return Err("a transmit was offered over a frame".into());
                }
            }
            Event::Skip => {}
        }
        Ok(())
    }

    fn describe(&self, s: &State) -> String {
        format!(
            "t={} ms stage {:?} in_progress {} late {}",
            s.now(),
            s.gate.stage(),
            s.gate.in_progress(s.now()),
            s.gate.poll_late()
        )
    }
}

#[test]
fn every_hold_is_bounded_and_every_frame_is_protected() {
    let m = Gate;
    let x = explore(&m);
    println!("receive gate model: {x}");
    x.assert_ok(&m);
    x.assert_some(|s| s.gate.stage() == Some(Stage::Header), "a header is held");
    x.assert_some(|s| s.gate.poll_late(), "a late poll is seen");
    x.assert_some(
        |s| s.gate.stage() == Some(Stage::Preamble) && !s.gate.in_progress(s.now()),
        "a preamble that was noise has lapsed and is still on the books",
    );
    // A hold always ends: from every state far enough from the horizon
    // to watch it, the receiver is free again.
    let last_start = HORIZON - u64::from(MAX_FRAME_MS).div_ceil(TICK_MS);
    x.assert_always_reachable(
        &m,
        |s| !s.gate.in_progress(s.now()) || s.tick > last_start,
        "nothing is arriving",
    );
}
