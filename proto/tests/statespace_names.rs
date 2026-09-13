//! Every sequence of name announcements, reports, hand-outs, replays and
//! passages of time the roster's name table can see, against a ledger of
//! what it should hold.
//!
//! The name table promises three things the report table does not, and
//! they are the whole reason names are kept beside reports rather than as
//! one more kind of report:
//!
//! - A name is handed out when it is *news* - first heard, or changed -
//!   and a node re-announcing what it is already called notifies nobody,
//!   however often it does so.
//! - A name outlives the reports it arrived between: a node going from a
//!   position to a ping and back keeps its name, and a name may arrive
//!   before that node has reported at all.
//! - A name ages by when its node was last *heard*, not by when it last
//!   announced - a name announcement is one transmission in twenty, so a
//!   name aged by its own arrival would be forgotten by a node that never
//!   went off the air.
//!
//! Time moves in steps of the TTL, as in the report model, so a node
//! touched two steps ago has expired and one touched a step ago has not.
//! A full table is not modeled here - the label dimension would multiply
//! the report model's nine-node walk - and eviction is held by the unit
//! test in the roster instead.

use midair_explore::{explore, Machine};
use midair_proto::ble;
use midair_proto::roster::{Report, Roster, SLOTS, TTL_MS};

/// Steps of the TTL the model runs for.
const AGES: u64 = 2;

/// The labels a node in the model can announce. Two is enough for a
/// rename: what matters is same-as-before against different-from-before.
const LABELS: [&str; 2] = ["sky-1", "sky-2"];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Named {
    /// Index into [`LABELS`].
    label: usize,
    /// The step this node was last heard from, by any transmission.
    at: u64,
    /// Owed to a central.
    dirty: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct State {
    roster: Roster,
    /// What the name table should hold, by address.
    ledger: Vec<Option<Named>>,
    step: u64,
}

impl State {
    fn now(&self) -> u64 {
        self.step * TTL_MS
    }

    /// Whether a node last heard from at `at` is still inside the TTL.
    fn live(&self, at: u64) -> bool {
        self.now().saturating_sub(at * TTL_MS) < TTL_MS
    }

    /// Expiry as the roster does it: at or past the TTL.
    fn expire(&mut self) {
        let now = self.now();
        for nd in self.ledger.iter_mut() {
            let stale = match *nd {
                Some(n) => now.saturating_sub(n.at * TTL_MS) >= TTL_MS,
                None => false,
            };
            if stale {
                *nd = None;
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Event {
    /// A node announces what it is called.
    Announce(u8, usize),
    /// A node reports a position, which says nothing about its name but
    /// keeps it alive.
    Report(u8),
    /// A central takes the next name owed to it.
    Take,
    /// A central connects, and is told the whole table.
    Replay,
    Age,
}

struct Names {
    nodes: u8,
}

fn position(src: u8) -> Report {
    let mut b = [0u8; ble::REMOTE_LEN];
    b[0] = src;
    Report::Position(b)
}

/// The label out of a handed-out value, read to its padding.
fn label_of(v: &[u8; ble::NODE_NAME_LEN]) -> String {
    let end = v[1..].iter().position(|&b| b == 0).unwrap_or(ble::NAME_LABEL_MAX);
    String::from_utf8(v[1..1 + end].to_vec()).expect("a label is ascii")
}

impl Machine for Names {
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
        let mut ev = Vec::new();
        for src in 1..=self.nodes {
            for label in 0..LABELS.len() {
                ev.push(Event::Announce(src, label));
            }
            ev.push(Event::Report(src));
        }
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
            Event::Announce(src, label) => {
                let news = n.roster.record_name(n.now(), src, LABELS[label]);
                n.expire();
                let entry = n.ledger[usize::from(src)];
                let renamed = entry.is_none_or(|nd| nd.label != label);
                n.ledger[usize::from(src)] = Some(Named {
                    label,
                    at: n.step,
                    // A name already handed out and re-announced unchanged
                    // stays handed out; anything else is owed.
                    dirty: renamed || entry.is_some_and(|nd| nd.dirty),
                });
                assert_eq!(
                    news, renamed,
                    "node {src} announcing {} reported news {news}",
                    LABELS[label]
                );
            }
            Event::Report(src) => {
                n.roster.record(n.now(), position(src));
                n.expire();
                // A report touches the name only by keeping it alive.
                if let Some(nd) = n.ledger[usize::from(src)].as_mut() {
                    nd.at = n.step;
                }
            }
            Event::Take => {
                if let Some(v) = n.roster.take_dirty_name() {
                    let src = usize::from(v[0]);
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
        // What the table says each node is called is what the ledger says.
        for src in 1..s.ledger.len() {
            let want = s.ledger[src].map(|nd| LABELS[nd.label]);
            let got = s.roster.name(src as u8);
            if want != got {
                return Err(format!("node {src} is named {got:?}, ledger says {want:?}"));
            }
        }
        // And what it would hand out is exactly what the ledger owes,
        // oldest first.
        let mut copy = s.roster;
        let mut handed = Vec::new();
        while let Some(v) = copy.take_dirty_name() {
            handed.push((v[0], label_of(&v)));
        }
        let mut owed: Vec<(u64, u8, String)> = (1..s.ledger.len())
            .filter_map(|a| s.ledger[a].map(|nd| (a as u8, nd)))
            .filter(|(_, nd)| nd.dirty)
            .map(|(a, nd)| (nd.at, a, LABELS[nd.label].to_string()))
            .collect();
        owed.sort();
        if handed.len() != owed.len()
            || !owed.iter().all(|(_, a, l)| handed.contains(&(*a, l.clone())))
        {
            return Err(format!("table would hand out {handed:?}, ledger owes {owed:?}"));
        }
        let ages: Vec<u64> = handed
            .iter()
            .map(|(a, _)| s.ledger[usize::from(*a)].unwrap().at)
            .collect();
        if ages.windows(2).any(|w| w[0] > w[1]) {
            return Err(format!("not oldest first: {handed:?} at {ages:?}"));
        }
        Ok(())
    }

    fn check_step(&self, from: &State, e: &Event, to: &State) -> Result<(), String> {
        // The promise a report table cannot make: a name that has been
        // handed out is not owed again by anything short of a rename or a
        // replay.
        if let Event::Announce(src, label) = *e {
            // Read against the announcement's own instant: a node that
            // fell out of the table while it was off the air is a node
            // being learned afresh, and that is news.
            let before = from.ledger[usize::from(src)].filter(|nd| to.live(nd.at));
            let after = to.ledger[usize::from(src)].expect("named by the announcement");
            if before.is_some_and(|nd| nd.label == label && !nd.dirty) && after.dirty {
                return Err(format!(
                    "node {src} re-announcing {} owes a notification",
                    LABELS[label]
                ));
            }
        }
        Ok(())
    }

    fn describe(&self, s: &State) -> String {
        let held: Vec<String> = (1..s.ledger.len())
            .filter_map(|a| {
                s.ledger[a].map(|n| {
                    format!("{a}={}@{}{}", LABELS[n.label], n.at, if n.dirty { "*" } else { "" })
                })
            })
            .collect();
        format!("step {} holding [{}]", s.step, held.join(" "))
    }
}

/// A fleet that fits the table: a name is handed out when it is news and
/// not otherwise, a report keeps it alive, a replay re-arms it and the
/// TTL forgets it.
#[test]
fn a_name_is_handed_out_when_it_is_news_and_not_again() {
    let m = Names { nodes: 3 };
    let x = explore(&m);
    println!("name model, 3 nodes: {x}");
    x.assert_ok(&m);
    assert!(m.nodes as usize <= SLOTS, "the table is not the constraint here");
    x.assert_some(
        |s| s.ledger.iter().flatten().count() == 3,
        "every node is named",
    );
    x.assert_some(
        |s| s.ledger.iter().flatten().any(|n| n.label == 1),
        "a node has been renamed",
    );
    x.assert_some(
        |s| s.ledger.iter().flatten().all(|n| !n.dirty) && s.ledger.iter().flatten().count() > 0,
        "every name has been handed out",
    );
    x.assert_some(
        |s| s.step == AGES && s.ledger.iter().flatten().count() < 3,
        "a node has aged out of the table",
    );
}
