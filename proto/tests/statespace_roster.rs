//! Every sequence of reports, hand-outs, replays and passages of time the
//! remote-node roster can see, against a ledger of what it should hold.
//!
//! The roster promises three things: a report is handed out exactly once
//! unless a newer one from the same node overtakes it, a replay re-arms
//! every node still inside the TTL and drops the rest, and a full table
//! gives up the node that has gone quietest. The ledger here is the
//! obvious one-line version of each, and the walk holds the two together
//! after every event.
//!
//! Time moves in steps of the TTL, so a node recorded two steps ago has
//! expired and one recorded a step ago has not.

use midair_explore::{explore, explore_with, Limits, Machine};
use midair_proto::roster::{Report, Roster, Value, SLOTS, TTL_MS};

/// Steps of the TTL the model runs for.
const AGES: u64 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Node {
    /// The step it was last recorded at.
    at: u64,
    dirty: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct State {
    roster: Roster,
    /// What the roster should hold, by address.
    ledger: Vec<Option<Node>>,
    step: u64,
}

impl State {
    fn now(&self) -> u64 {
        self.step * TTL_MS
    }

    fn live(&self, at: u64) -> bool {
        self.now().saturating_sub(at * TTL_MS) < TTL_MS
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Event {
    Record(u8),
    Take,
    Replay,
    Age,
}

struct Table {
    nodes: u8,
}

fn report(src: u8) -> Report {
    let mut b = [0u8; midair_proto::ble::REMOTE_LEN];
    b[0] = src;
    Report::Position(b)
}

fn src_of(v: &Value) -> u8 {
    v.bytes()[0]
}

/// Every node the roster holds, read off a copy by replaying and
/// draining it.
fn srcs(roster: &Roster, now_ms: u64) -> Vec<u8> {
    let mut copy = *roster;
    copy.replay(now_ms);
    let mut out = Vec::new();
    while let Some(v) = copy.take_dirty(now_ms) {
        out.push(src_of(&v));
    }
    out
}

impl Machine for Table {
    type State = State;
    type Event = Event;

    fn initial(&self) -> Vec<State> {
        vec![State {
            roster: Roster::new(),
            ledger: vec![None; usize::from(self.nodes) + 1],
            step: 0,
        }]
    }

    fn events(&self, s: &State) -> Vec<Event> {
        let mut ev: Vec<Event> = (1..=self.nodes).map(Event::Record).collect();
        ev.push(Event::Take);
        ev.push(Event::Replay);
        if s.step < AGES {
            ev.push(Event::Age);
        }
        ev
    }

    fn step(&self, s: &State, e: &Event) -> State {
        let mut n = s.clone();
        match *e {
            Event::Record(src) => {
                n.roster.record(n.now(), report(src));
                n.expire();
                n.ledger[usize::from(src)] = Some(Node {
                    at: n.step,
                    dirty: true,
                });
                // A full table gives up the quietest node. Which of several
                // equally quiet ones is the roster's choice; the ledger
                // follows it and only insists it was one of the quietest.
                let held = n.ledger.iter().flatten().count();
                if held > SLOTS {
                    let kept = srcs(&n.roster, n.now());
                    let gone: Vec<u8> = (1..=self.nodes)
                        .filter(|&a| n.ledger[usize::from(a)].is_some() && !kept.contains(&a))
                        .collect();
                    assert_eq!(gone.len(), 1, "one node makes way, not {gone:?}");
                    let quietest = (1..=self.nodes)
                        .filter(|&a| a != src)
                        .filter_map(|a| n.ledger[usize::from(a)].map(|nd| nd.at))
                        .min()
                        .expect("something to evict");
                    let evicted = n.ledger[usize::from(gone[0])].unwrap();
                    assert_eq!(evicted.at, quietest, "the roster evicted {} which was not quietest", gone[0]);
                    n.ledger[usize::from(gone[0])] = None;
                }
            }
            Event::Take => {
                if let Some(v) = n.roster.take_dirty(n.now()) {
                    let src = usize::from(src_of(&v));
                    if let Some(nd) = n.ledger[src].as_mut() {
                        nd.dirty = false;
                    }
                }
            }
            Event::Replay => {
                n.roster.replay(n.now());
                n.expire();
                for nd in n.ledger.iter_mut().flatten() {
                    nd.dirty = true;
                }
            }
            Event::Age => n.step += 1,
        }
        n
    }

    fn check(&self, s: &State) -> Result<(), String> {
        let held: Vec<u8> = (1..s.ledger.len())
            .filter(|&a| s.ledger[a].is_some())
            .map(|a| a as u8)
            .collect();
        if held.len() > SLOTS {
            return Err("the ledger holds more than the table can".into());
        }
        if s.roster.len() != held.len() {
            return Err(format!(
                "roster holds {} nodes, ledger {}",
                s.roster.len(),
                held.len()
            ));
        }
        // What is dirty in the roster is what the ledger says is owed:
        // draining a copy hands out exactly those, oldest first.
        let mut copy = s.roster;
        let mut handed = Vec::new();
        while let Some(v) = copy.take_dirty(s.now()) {
            handed.push(src_of(&v));
        }
        let mut owed: Vec<(u64, u8)> = held
            .iter()
            .filter(|&&a| s.ledger[usize::from(a)].unwrap().dirty)
            .map(|&a| (s.ledger[usize::from(a)].unwrap().at, a))
            .collect();
        owed.sort();
        let owed: Vec<u8> = owed.into_iter().map(|(_, a)| a).collect();
        if handed.len() != owed.len() || !owed.iter().all(|a| handed.contains(a)) {
            return Err(format!("roster would hand out {handed:?}, ledger owes {owed:?}"));
        }
        // Oldest first: the ages handed out never go down. Nodes recorded
        // in the same step may come out in either order.
        let ages: Vec<u64> = handed
            .iter()
            .map(|&a| s.ledger[usize::from(a)].unwrap().at)
            .collect();
        if ages.windows(2).any(|w| w[0] > w[1]) {
            return Err(format!("not oldest first: {handed:?} at {ages:?}"));
        }
        Ok(())
    }

    fn check_step(&self, from: &State, e: &Event, to: &State) -> Result<(), String> {
        if let Event::Take = e {
            let before = from.ledger.iter().flatten().filter(|n| n.dirty).count();
            let after = to.ledger.iter().flatten().filter(|n| n.dirty).count();
            if before > 0 && after != before - 1 {
                return Err("a take handed out something other than one owed report".into());
            }
            if before == 0 && after != 0 {
                return Err("a take with nothing owed handed something out".into());
            }
        }
        Ok(())
    }

    fn describe(&self, s: &State) -> String {
        let held: Vec<String> = (1..s.ledger.len())
            .filter_map(|a| {
                s.ledger[a].map(|n| format!("{a}@{}{}", n.at, if n.dirty { "*" } else { "" }))
            })
            .collect();
        format!("step {} holding [{}]", s.step, held.join(" "))
    }
}

impl State {
    /// Expiry as the roster does it: at or past the TTL.
    fn expire(&mut self) {
        let now = self.now();
        for nd in self.ledger.iter_mut() {
            if let Some(n) = *nd {
                if now.saturating_sub(n.at * TTL_MS) >= TTL_MS {
                    *nd = None;
                }
            }
        }
    }
}

/// A fleet that fits the table: every report is handed out once, replays
/// re-arm the live ones, expiry forgets the rest.
#[test]
fn a_small_fleet_is_handed_out_exactly_once_each() {
    let m = Table { nodes: 3 };
    let x = explore(&m);
    println!("roster model, 3 nodes: {x}");
    x.assert_ok(&m);
    x.assert_some(|s| s.roster.len() == 3, "all three are held");
    x.assert_some(
        |s| s.step == AGES && s.roster.len() < 3 && s.ledger.iter().flatten().any(|n| !s.live(n.at)),
        "a node has aged out",
    );
}

/// A fleet one larger than the table, starting from a table already
/// full: the quietest node makes way, and everything the ledger says is
/// owed is still handed out once.
#[test]
fn a_full_table_evicts_the_quietest_node() {
    struct Full(Table);
    impl Machine for Full {
        type State = State;
        type Event = Event;
        fn initial(&self) -> Vec<State> {
            let mut s = self.0.initial().remove(0);
            for src in 1..=SLOTS as u8 {
                s = self.0.step(&s, &Event::Record(src));
            }
            // Half of them already handed out, so both kinds are in play.
            for _ in 0..SLOTS / 2 {
                s = self.0.step(&s, &Event::Take);
            }
            vec![s]
        }
        fn events(&self, s: &State) -> Vec<Event> {
            self.0.events(s)
        }
        fn step(&self, s: &State, e: &Event) -> State {
            self.0.step(s, e)
        }
        fn check(&self, s: &State) -> Result<(), String> {
            self.0.check(s)
        }
        fn check_step(&self, from: &State, e: &Event, to: &State) -> Result<(), String> {
            self.0.check_step(from, e, to)
        }
        fn describe(&self, s: &State) -> String {
            self.0.describe(s)
        }
    }
    let m = Full(Table {
        nodes: SLOTS as u8 + 1,
    });
    let x = explore_with(
        &m,
        Limits {
            max_states: 2_000_000,
            max_depth: 6,
        },
    );
    println!("roster model, {} nodes from a full table: {x}", SLOTS + 1);
    x.assert_ok(&m);
    x.assert_some(|s| s.roster.len() == SLOTS, "the table is full");
    x.assert_some(
        |s| s.ledger[usize::from(SLOTS as u8 + 1)].is_some() && s.ledger.iter().flatten().count() == SLOTS,
        "the ninth node has taken a slot from one of the first eight",
    );
}
