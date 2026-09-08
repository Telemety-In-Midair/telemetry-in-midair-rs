//! Every order in which frames can arrive, repeat, expire and wrap at the
//! seen table and the repeat queue, against a ledger of what each should
//! hold.
//!
//! The table promises that a `(src, id)` pair is new once and seen until
//! its TTL from the last copy, that a full table gives up its oldest pair,
//! and that a reset forgets everything. The queue promises that a frame is
//! taken once, earliest due first, only when due, and that a full queue
//! drops rather than overwrites. Time moves in steps of half the TTL.

use midair_explore::{explore, Machine};
use midair_proto::dedup::{RepeatQueue, SeenTable};

/// Tables small enough to fill: three pairs of the four the model hears,
/// two of the repeats it queues.
const SEEN: usize = 3;
const REPEATS: usize = 2;

const TTL_MS: u64 = 1_000;
const STEP_MS: u64 = 500;
const STEPS: u64 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct Entry {
    src: u8,
    id: u8,
    at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct State {
    table: SeenTable<SEEN>,
    queue: RepeatQueue<REPEATS>,
    /// What the table should hold: the pairs inside their TTL.
    ledger: Vec<Entry>,
    /// What the queue should hold: frames by (due step, tag).
    queued: Vec<(u64, u8)>,
    step: u64,
}

impl State {
    fn now(&self) -> u64 {
        self.step * STEP_MS
    }

    fn live(&self, e: &Entry) -> bool {
        self.now().saturating_sub(e.at * STEP_MS) < TTL_MS
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Event {
    /// A frame from `src` with id `id` arrives.
    Hear(u8, u8),
    /// A repeat of a frame tagged `tag` is queued, due `ahead` steps on.
    Queue(u8, u64),
    /// The loop takes whatever repeat is due.
    Take,
    /// Half a TTL passes.
    Age,
}

struct Dedup;

impl Machine for Dedup {
    type State = State;
    type Event = Event;

    fn initial(&self) -> Vec<State> {
        vec![State {
            table: SeenTable::<SEEN>::new(TTL_MS),
            queue: RepeatQueue::<REPEATS>::new(),
            ledger: Vec::new(),
            queued: Vec::new(),
            step: 0,
        }]
    }

    fn events(&self, s: &State) -> Vec<Event> {
        let mut ev = Vec::new();
        // Two nodes, two ids each: enough for a duplicate, a wrap and a
        // stranger.
        for src in [1u8, 2] {
            for id in [0u8, 1] {
                ev.push(Event::Hear(src, id));
            }
        }
        for tag in [10u8, 11] {
            for ahead in [0u64, 1] {
                ev.push(Event::Queue(tag, ahead));
            }
        }
        ev.push(Event::Take);
        if s.step < STEPS {
            ev.push(Event::Age);
        }
        ev
    }

    fn step(&self, s: &State, e: &Event) -> State {
        let mut n = s.clone();
        let now = n.now();
        match *e {
            Event::Hear(src, id) => {
                let seen = n.table.mark(src, id, now);
                n.ledger.retain(|x| now.saturating_sub(x.at * STEP_MS) < TTL_MS);
                let known = n.ledger.iter().position(|x| x.src == src && x.id == id);
                assert_eq!(seen, known.is_some(), "table says seen {seen}, ledger {known:?}");
                match known {
                    Some(i) => n.ledger[i].at = n.step,
                    None => {
                        // A full table gives up its oldest pair; the model
                        // follows the table's choice and insists it was
                        // one of the oldest.
                        if n.ledger.len() == SEEN {
                            let oldest = n.ledger.iter().map(|e| e.at).min().unwrap();
                            let gone = n
                                .ledger
                                .iter()
                                .position(|e| !n.table.contains(e.src, e.id, now))
                                .expect("the table evicted someone");
                            assert_eq!(n.ledger[gone].at, oldest, "evicted a pair that was not oldest");
                            n.ledger.remove(gone);
                        }
                        n.ledger.push(Entry { src, id, at: n.step });
                    }
                }
                n.ledger.sort();
            }
            Event::Queue(tag, ahead) => {
                let due = n.step + ahead;
                let ok = n.queue.push(&[tag], due * STEP_MS);
                assert_eq!(ok, n.queued.len() < REPEATS, "queue accepted {ok}");
                if ok {
                    n.queued.push((due, tag));
                    n.queued.sort();
                }
            }
            Event::Take => {
                let got = n.queue.take_due(now).map(|(b, len)| b[..len].to_vec());
                let earliest = n
                    .queued
                    .iter()
                    .enumerate()
                    .filter(|(_, (due, _))| *due <= n.step)
                    .min_by_key(|(_, (due, _))| *due)
                    .map(|(i, _)| i);
                match (got, earliest) {
                    (None, None) => {}
                    (Some(frame), Some(i)) => {
                        let (due, tag) = n.queued.remove(i);
                        // Several may share the earliest due; the tag has
                        // to be one of theirs.
                        let candidates: Vec<u8> = std::iter::once(tag)
                            .chain(n.queued.iter().filter(|(d, _)| *d == due).map(|(_, t)| *t))
                            .collect();
                        assert!(candidates.contains(&frame[0]), "took {frame:?}");
                        // Put the ledger back the way the queue chose.
                        if frame[0] != tag {
                            let j = n.queued.iter().position(|(d, t)| *d == due && *t == frame[0]).unwrap();
                            n.queued[j] = (due, tag);
                        }
                    }
                    (got, earliest) => panic!("queue gave {got:?}, ledger expected {earliest:?}"),
                }
            }
            Event::Age => n.step += 1,
        }
        n
    }

    fn check(&self, s: &State) -> Result<(), String> {
        let now = s.now();
        for e in &s.ledger {
            if s.live(e) != s.table.contains(e.src, e.id, now) {
                return Err(format!("{e:?}: ledger live {}, table {}", s.live(e), s.table.contains(e.src, e.id, now)));
            }
        }
        if s.queue.len() != s.queued.len() {
            return Err(format!("queue holds {}, ledger {}", s.queue.len(), s.queued.len()));
        }
        if s.queue.due(now) != s.queued.iter().any(|(due, _)| *due <= s.step) {
            return Err("due disagrees".into());
        }
        Ok(())
    }

    fn describe(&self, s: &State) -> String {
        format!("step {} seen {:?} queued {:?}", s.step, s.ledger, s.queued)
    }
}

#[test]
fn every_arrival_order_is_deduplicated_and_every_repeat_taken_once() {
    let m = Dedup;
    let x = explore(&m);
    println!("dedup model: {x}");
    x.assert_ok(&m);
    x.assert_some(|s| s.ledger.len() == SEEN, "the table is full");
    x.assert_some(
        |s| s.ledger.iter().any(|e| !s.live(e)) && s.step == STEPS,
        "a pair has aged out",
    );
    x.assert_some(|s| s.queued.len() == REPEATS, "the queue is full");
}
