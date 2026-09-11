//! Exhaustive exploration of a finite state machine.
//!
//! A model is anything that implements [`Machine`]: a set of initial states,
//! the events that can happen in a state, what each event does, and the
//! invariants every reachable state has to satisfy. [`explore`] then walks
//! every state reachable from the initial ones, breadth first, checking the
//! invariants as it goes, and stops at the first violation with the shortest
//! sequence of events that produces it.
//!
//! Breadth first is what makes the trace short, and short is what makes a
//! trace readable: a violation eleven events deep is a bug report, one that
//! takes four hundred is noise. Every state is visited once, so a model with
//! a few thousand states runs in milliseconds and one with a million in a
//! few seconds.
//!
//! What this is not: a simulator with a clock. Time is abstracted into the
//! events that depend on it - "the budget expires", "the hold runs out" -
//! and a model that puts a millisecond counter in its state will not be
//! finite. The two-valued clock the firmware models use (an event happens
//! either before a deadline or at it) is the usual way through.
//!
//! Three kinds of check, each with the trace that explains a failure:
//!
//! - [`Machine::check`] and [`Machine::check_step`] are invariants, tested
//!   on every state and every transition during the walk.
//! - [`Explored::assert_always_reachable`] is liveness: from every reachable
//!   state, some state satisfying a predicate can still be reached. This is
//!   how "the board can always be reached over BLE again" is stated - a
//!   state from which no advertising state is reachable is a board gone
//!   dark for good.
//! - [`Explored::assert_some`] is coverage: at least one reachable state
//!   satisfies a predicate, so that an invariant about connected sessions
//!   cannot pass because no session was ever modeled.

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::fmt::{Debug, Display, Write as _};
use std::hash::Hash;

/// A state machine to explore.
///
/// Nondeterminism goes in [`events`](Self::events): a step that may end
/// two ways is two events, not one event with a random outcome. That is
/// what lets the explorer see both.
pub trait Machine {
    type State: Clone + Eq + Hash + Debug;
    type Event: Clone + Debug;

    /// The states exploration starts from.
    fn initial(&self) -> Vec<Self::State>;

    /// The events that can happen in `state`. An empty list is a dead end,
    /// which may be correct (a board asleep with no timer) or a bug.
    fn events(&self, state: &Self::State) -> Vec<Self::Event>;

    /// The state `event` leads to from `state`.
    fn step(&self, state: &Self::State, event: &Self::Event) -> Self::State;

    /// An invariant every reachable state must satisfy. `Err` names what
    /// was violated.
    fn check(&self, _state: &Self::State) -> Result<(), String> {
        Ok(())
    }

    /// A property of one transition, for what a state alone cannot say:
    /// that a request was not silently dropped, that an effect was emitted.
    fn check_step(
        &self,
        _from: &Self::State,
        _event: &Self::Event,
        _to: &Self::State,
    ) -> Result<(), String> {
        Ok(())
    }

    /// A one-line description of a state for a trace. Defaults to `Debug`.
    fn describe(&self, state: &Self::State) -> String {
        format!("{state:?}")
    }

    /// The lane an event is drawn in on a gantt of a trace: the component
    /// it belongs to. Defaults to one lane for everything.
    fn lane(&self, _event: &Self::Event) -> &'static str {
        "events"
    }
}

/// Bounds on an exploration, so a model with a mistake in it fails fast
/// rather than filling memory.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    /// Give up after this many distinct states.
    pub max_states: usize,
    /// Do not expand states deeper than this many events from an initial
    /// state. `usize::MAX` explores the whole space.
    pub max_depth: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_states: 2_000_000,
            max_depth: usize::MAX,
        }
    }
}

/// An invariant that did not hold.
#[derive(Clone, Debug)]
pub struct Violation<S, E> {
    /// What the check said.
    pub what: String,
    /// Index of the state the violation was found in - for a transition
    /// check, the state the offending event was taken from.
    pub state: usize,
    /// For a transition check, the event and the state it led to. Kept
    /// here rather than looked up, because the state it led to may have
    /// been reached first by some other path, whose trace would not show
    /// the step that broke the rule.
    pub step: Option<(E, S)>,
}

/// Everything an exploration found. States are numbered in the order they
/// were discovered, which for a breadth-first walk is by nondecreasing
/// depth - so the first state matching a predicate is one of the shallowest.
pub struct Explored<M: Machine> {
    states: Vec<M::State>,
    /// The state each one was first reached from, and by which event.
    parent: Vec<Option<(usize, M::Event)>>,
    depth: Vec<usize>,
    /// Successor indices per state, deduplicated.
    succ: Vec<Vec<usize>>,
    /// Whether a state's events were enumerated. A state at the depth
    /// limit, or left in the queue when the state limit hit, was not, and
    /// its empty successor list is ignorance rather than a dead end.
    expanded: Vec<bool>,
    transitions: usize,
    violation: Option<Violation<M::State, M::Event>>,
    truncated: bool,
}

/// Explore `machine` under the default [`Limits`].
pub fn explore<M: Machine>(machine: &M) -> Explored<M> {
    explore_with(machine, Limits::default())
}

/// Explore `machine`, stopping at the first invariant violation.
pub fn explore_with<M: Machine>(machine: &M, limits: Limits) -> Explored<M> {
    let mut out = Explored {
        states: Vec::new(),
        parent: Vec::new(),
        depth: Vec::new(),
        succ: Vec::new(),
        expanded: Vec::new(),
        transitions: 0,
        violation: None,
        truncated: false,
    };
    let mut index: HashMap<M::State, usize> = HashMap::new();
    let mut queue = VecDeque::new();

    for s in machine.initial() {
        if index.contains_key(&s) {
            continue;
        }
        let i = out.push(&mut index, s, None, 0);
        if let Err(what) = machine.check(&out.states[i]) {
            out.violation = Some(Violation {
                what,
                state: i,
                step: None,
            });
            return out;
        }
        queue.push_back(i);
    }

    while let Some(i) = queue.pop_front() {
        if out.depth[i] >= limits.max_depth {
            continue;
        }
        out.expanded[i] = true;
        let from = out.states[i].clone();
        for event in machine.events(&from) {
            let to = machine.step(&from, &event);
            out.transitions += 1;
            if let Err(what) = machine.check_step(&from, &event, &to) {
                out.violation = Some(Violation {
                    what,
                    state: i,
                    step: Some((event, to)),
                });
                return out;
            }
            let j = match index.get(&to) {
                Some(&j) => j,
                None => {
                    if out.states.len() >= limits.max_states {
                        out.truncated = true;
                        return out;
                    }
                    let j = out.push(&mut index, to, Some((i, event.clone())), out.depth[i] + 1);
                    if let Err(what) = machine.check(&out.states[j]) {
                        out.violation = Some(Violation {
                            what,
                            state: j,
                            step: None,
                        });
                        return out;
                    }
                    queue.push_back(j);
                    j
                }
            };
            if !out.succ[i].contains(&j) {
                out.succ[i].push(j);
            }
        }
    }
    out
}

impl<M: Machine> Explored<M> {
    fn push(
        &mut self,
        index: &mut HashMap<M::State, usize>,
        state: M::State,
        parent: Option<(usize, M::Event)>,
        depth: usize,
    ) -> usize {
        let i = self.states.len();
        index.insert(state.clone(), i);
        self.states.push(state);
        self.parent.push(parent);
        self.depth.push(depth);
        self.succ.push(Vec::new());
        self.expanded.push(false);
        i
    }

    /// Distinct states reached.
    pub fn len(&self) -> usize {
        self.states.len()
    }

    pub fn is_empty(&self) -> bool {
        self.states.is_empty()
    }

    /// Transitions taken, duplicates included.
    pub fn transitions(&self) -> usize {
        self.transitions
    }

    /// The deepest state, in events from an initial state.
    pub fn max_depth(&self) -> usize {
        self.depth.iter().copied().max().unwrap_or(0)
    }

    /// Whether the walk hit [`Limits::max_states`] before finishing. A
    /// truncated exploration proves nothing about the states it never saw.
    pub fn truncated(&self) -> bool {
        self.truncated
    }

    pub fn violation(&self) -> Option<&Violation<M::State, M::Event>> {
        self.violation.as_ref()
    }

    /// The violation as a report: the trace to where it was found, and for
    /// a transition check the step that broke the rule.
    pub fn format_violation(&self, machine: &M) -> Option<String> {
        let v = self.violation.as_ref()?;
        let mut out = format!(
            "state space violation: {}\n  after {} events:\n{}",
            v.what,
            self.depth[v.state] + usize::from(v.step.is_some()),
            self.format_trace(machine, v.state)
        );
        if let Some((event, to)) = &v.step {
            let n = self.depth[v.state] + 1;
            let _ = writeln!(out, "  {n:>3}. {event:?}");
            let _ = writeln!(out, "       -> {}", machine.describe(to));
        }
        Some(out)
    }

    /// Every state reached, in discovery order.
    pub fn states(&self) -> impl Iterator<Item = &M::State> {
        self.states.iter()
    }

    /// How many reachable states satisfy `pred`.
    pub fn count(&self, pred: impl Fn(&M::State) -> bool) -> usize {
        self.states.iter().filter(|s| pred(s)).count()
    }

    /// States with no enabled event. A state whose events were never
    /// enumerated - at the depth limit, or beyond the state limit - is not
    /// one.
    pub fn dead_ends(&self) -> Vec<usize> {
        (0..self.states.len())
            .filter(|&i| self.expanded[i] && self.succ[i].is_empty())
            .collect()
    }

    /// A histogram of the states by a label, for a coverage report.
    pub fn coverage(&self, label: impl Fn(&M::State) -> String) -> BTreeMap<String, usize> {
        let mut out = BTreeMap::new();
        for s in &self.states {
            *out.entry(label(s)).or_insert(0) += 1;
        }
        out
    }

    /// The events from an initial state to state `idx`, with the state each
    /// one leads to. The first entry is the initial state with no event.
    pub fn trace(&self, idx: usize) -> Vec<(Option<M::Event>, &M::State)> {
        let mut path = Vec::new();
        let mut at = idx;
        loop {
            match &self.parent[at] {
                Some((from, event)) => {
                    path.push((Some(event.clone()), &self.states[at]));
                    at = *from;
                }
                None => {
                    path.push((None, &self.states[at]));
                    break;
                }
            }
        }
        path.reverse();
        path
    }

    /// The trace to `idx`, one line per step.
    pub fn format_trace(&self, machine: &M, idx: usize) -> String {
        let mut out = String::new();
        for (n, (event, state)) in self.trace(idx).into_iter().enumerate() {
            match event {
                None => {
                    let _ = writeln!(out, "  {n:>3}. {}", machine.describe(state));
                }
                Some(e) => {
                    let _ = writeln!(out, "  {n:>3}. {e:?}");
                    let _ = writeln!(out, "       -> {}", machine.describe(state));
                }
            }
        }
        out
    }

    /// The trace to `idx` as a mermaid gantt: one lane per
    /// [`Machine::lane`], one unit of time per event, so a counterexample
    /// can be read against time by component.
    pub fn gantt(&self, machine: &M, idx: usize, title: &str) -> String {
        let mut out = String::new();
        let _ = writeln!(out, "gantt");
        let _ = writeln!(out, "    title {}", sanitize(title));
        let _ = writeln!(out, "    dateFormat x");
        let _ = writeln!(out, "    axisFormat %L");
        let mut lanes: BTreeMap<&'static str, Vec<(usize, String)>> = BTreeMap::new();
        for (n, (event, _)) in self.trace(idx).into_iter().enumerate() {
            if let Some(e) = event {
                lanes
                    .entry(machine.lane(&e))
                    .or_default()
                    .push((n, format!("{e:?}")));
            }
        }
        for (lane, events) in lanes {
            let _ = writeln!(out, "    section {}", sanitize(lane));
            for (n, label) in events {
                let _ = writeln!(out, "    {} :e{}, {}, 1ms", sanitize(&label), n, n);
            }
        }
        out
    }

    /// Panic with the shortest trace if an invariant failed or the walk was
    /// cut short. Returns `self` otherwise, so checks can be chained.
    pub fn assert_ok(&self, machine: &M) -> &Self {
        if let Some(report) = self.format_violation(machine) {
            panic!("{report}");
        }
        assert!(
            !self.truncated,
            "exploration truncated at {} states: raise Limits::max_states or shrink the model",
            self.states.len()
        );
        self
    }

    /// Coverage: at least one reachable state satisfies `pred`. Guards an
    /// invariant against passing vacuously.
    pub fn assert_some(&self, pred: impl Fn(&M::State) -> bool, what: &str) -> &Self {
        assert!(
            self.states.iter().any(|s| pred(s)),
            "no reachable state where {what}: the model never gets there, so nothing about it was checked"
        );
        self
    }

    /// Safety, checked after the walk: no reachable state satisfies `pred`.
    /// Fails with the trace to the shallowest one that does.
    pub fn assert_none(&self, machine: &M, pred: impl Fn(&M::State) -> bool, what: &str) -> &Self {
        if let Some(i) = (0..self.states.len()).find(|&i| pred(&self.states[i])) {
            panic!(
                "state space violation: {what}\n  after {} events:\n{}",
                self.depth[i],
                self.format_trace(machine, i)
            );
        }
        self
    }

    /// Liveness: from every reachable state, some state satisfying `pred`
    /// is still reachable. Fails with the trace to the shallowest state
    /// from which none is.
    ///
    /// Only states whose events were enumerated are held to it: a state
    /// at the depth limit has no recorded successors and cannot be a
    /// trap, only unknown. A truncated walk is refused outright, since
    /// what it never saw may be the way out.
    pub fn assert_always_reachable(
        &self,
        machine: &M,
        pred: impl Fn(&M::State) -> bool,
        what: &str,
    ) -> &Self {
        if let Some(i) = self.trap(pred) {
            panic!(
                "state space violation: from here nothing can reach a state where {what}\n  after {} events:\n{}",
                self.depth[i],
                self.format_trace(machine, i)
            );
        }
        self
    }

    /// The shallowest state from which no state satisfying `pred` can be
    /// reached, if there is one: what [`assert_always_reachable`] fails on,
    /// for a model that expects to find one.
    ///
    /// [`assert_always_reachable`]: Self::assert_always_reachable
    pub fn trap(&self, pred: impl Fn(&M::State) -> bool) -> Option<usize> {
        assert!(
            !self.truncated,
            "exploration truncated at {} states: liveness cannot be judged on a walk that stopped early",
            self.states.len()
        );
        let n = self.states.len();
        // Reverse edges, then a breadth-first walk back from every state
        // that satisfies the predicate.
        let mut pred_of: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (i, succ) in self.succ.iter().enumerate() {
            for &j in succ {
                pred_of[j].push(i);
            }
        }
        let mut can = vec![false; n];
        let mut queue = VecDeque::new();
        for (i, s) in self.states.iter().enumerate() {
            if pred(s) {
                can[i] = true;
                queue.push_back(i);
            }
        }
        while let Some(j) = queue.pop_front() {
            for &i in &pred_of[j] {
                if !can[i] {
                    can[i] = true;
                    queue.push_back(i);
                }
            }
        }
        (0..n).find(|&i| self.expanded[i] && !can[i])
    }
}

impl<M: Machine> Display for Explored<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} states, {} transitions, depth {}{}",
            self.states.len(),
            self.transitions,
            self.max_depth(),
            if self.truncated { " (truncated)" } else { "" }
        )
    }
}

/// Mermaid reads a colon as the start of a task's metadata and a comma as
/// a separator inside it, so neither may appear in a label.
fn sanitize(s: &str) -> String {
    s.replace(':', "=").replace(',', ";")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A counter that can go up, or reset, with a ceiling.
    struct Counter {
        ceiling: u32,
        /// Where an invariant is declared broken, if anywhere.
        bad: Option<u32>,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    struct Count(u32);

    #[derive(Clone, Copy, Debug)]
    enum Tick {
        Up,
        Reset,
    }

    impl Machine for Counter {
        type State = Count;
        type Event = Tick;

        fn initial(&self) -> Vec<Count> {
            vec![Count(0)]
        }

        fn events(&self, s: &Count) -> Vec<Tick> {
            let mut v = vec![Tick::Reset];
            if s.0 < self.ceiling {
                v.push(Tick::Up);
            }
            v
        }

        fn step(&self, s: &Count, e: &Tick) -> Count {
            match e {
                Tick::Up => Count(s.0 + 1),
                Tick::Reset => Count(0),
            }
        }

        fn check(&self, s: &Count) -> Result<(), String> {
            match self.bad {
                Some(b) if s.0 == b => Err(format!("count reached {b}")),
                _ => Ok(()),
            }
        }

        fn lane(&self, e: &Tick) -> &'static str {
            match e {
                Tick::Up => "up",
                Tick::Reset => "reset",
            }
        }
    }

    #[test]
    fn walks_every_state_once() {
        let m = Counter {
            ceiling: 5,
            bad: None,
        };
        let x = explore(&m);
        x.assert_ok(&m);
        assert_eq!(x.len(), 6);
        assert_eq!(x.max_depth(), 5);
        // Up from 0..5 and Reset from all six.
        assert_eq!(x.transitions(), 11);
        assert!(x.dead_ends().is_empty());
        assert_eq!(x.count(|s| s.0 > 2), 3);
        assert_eq!(x.to_string(), "6 states, 11 transitions, depth 5");
    }

    /// The trace to a violation is the shortest one: three ups, never a
    /// reset on the way.
    #[test]
    fn reports_the_shortest_trace() {
        let m = Counter {
            ceiling: 10,
            bad: Some(3),
        };
        let x = explore(&m);
        let v = x.violation().expect("count 3 is reachable");
        assert_eq!(v.what, "count reached 3");
        assert!(v.step.is_none());
        let trace = x.trace(v.state);
        assert_eq!(trace.len(), 4);
        assert!(trace[1..].iter().all(|(e, _)| matches!(e, Some(Tick::Up))));
        let text = x.format_trace(&m, v.state);
        assert!(text.contains("Count(0)"));
        assert!(text.contains("Count(3)"));
        let r = std::panic::catch_unwind(|| x.assert_ok(&m));
        assert!(r.is_err());
    }

    /// A transition check fails with the event that broke it.
    #[test]
    fn a_transition_check_names_its_event() {
        struct NoBigJump;
        impl Machine for NoBigJump {
            type State = Count;
            type Event = Tick;
            fn initial(&self) -> Vec<Count> {
                vec![Count(0)]
            }
            fn events(&self, _: &Count) -> Vec<Tick> {
                vec![Tick::Up, Tick::Reset]
            }
            fn step(&self, s: &Count, e: &Tick) -> Count {
                match e {
                    Tick::Up => Count(s.0 + 2),
                    Tick::Reset => Count(0),
                }
            }
            fn check_step(&self, from: &Count, _: &Tick, to: &Count) -> Result<(), String> {
                if to.0 > from.0 + 1 {
                    Err("jumped by more than one".into())
                } else {
                    Ok(())
                }
            }
        }
        let x = explore(&NoBigJump);
        let v = x.violation().expect("the first Up jumps by two");
        assert!(matches!(v.step, Some((Tick::Up, Count(2)))));
        assert_eq!(x.trace(v.state).len(), 1);
        let report = x.format_violation(&NoBigJump).unwrap();
        assert!(report.contains("after 1 events"), "{report}");
        assert!(report.ends_with("-> Count(2)\n"), "{report}");
    }

    /// Liveness: a trap state from which the counter can never reset is
    /// found, with the trace into it.
    #[test]
    fn a_trap_state_fails_liveness() {
        struct Trap;
        impl Machine for Trap {
            type State = Count;
            type Event = Tick;
            fn initial(&self) -> Vec<Count> {
                vec![Count(0)]
            }
            fn events(&self, s: &Count) -> Vec<Tick> {
                if s.0 >= 2 {
                    vec![Tick::Up]
                } else {
                    vec![Tick::Up, Tick::Reset]
                }
            }
            fn step(&self, s: &Count, e: &Tick) -> Count {
                match e {
                    Tick::Up => Count((s.0 + 1).min(2)),
                    Tick::Reset => Count(0),
                }
            }
        }
        let x = explore(&Trap);
        x.assert_ok(&Trap);
        x.assert_some(|s| s.0 == 2, "the trap is reached");
        let r = std::panic::catch_unwind(|| {
            x.assert_always_reachable(&Trap, |s| s.0 == 0, "the counter is at zero")
        });
        let err = match r {
            Err(e) => e,
            Ok(_) => panic!("liveness passed on a trap"),
        };
        let msg = *err.downcast::<String>().unwrap();
        assert!(msg.contains("after 2 events"), "{msg}");
        // A predicate that is reachable from everywhere passes.
        x.assert_always_reachable(&Trap, |s| s.0 == 2, "the trap");
        // And a state check after the fact finds the shallowest offender.
        let r = std::panic::catch_unwind(|| x.assert_none(&Trap, |s| s.0 == 2, "the trap is entered"));
        assert!(r.is_err());
    }

    /// A depth-bounded walk has frontier states with no successors, and
    /// they are unknown rather than traps: liveness holds them to nothing,
    /// and they are not dead ends.
    #[test]
    fn a_bounded_walk_does_not_mistake_its_frontier_for_a_trap() {
        let m = Counter {
            ceiling: 100,
            bad: None,
        };
        let x = explore_with(
            &m,
            Limits {
                max_states: 10_000,
                max_depth: 4,
            },
        );
        x.assert_ok(&m);
        assert!(x.dead_ends().is_empty());
        x.assert_always_reachable(&m, |s| s.0 == 0, "the counter is at zero");
        // A truncated walk is refused, not judged.
        let cut = explore_with(
            &m,
            Limits {
                max_states: 5,
                max_depth: usize::MAX,
            },
        );
        assert!(std::panic::catch_unwind(|| {
            cut.assert_always_reachable(&m, |s| s.0 == 0, "zero")
        })
        .is_err());
    }

    #[test]
    fn coverage_guards_against_a_vacuous_invariant() {
        let m = Counter {
            ceiling: 2,
            bad: None,
        };
        let x = explore(&m);
        x.assert_some(|s| s.0 == 2, "the ceiling is reached");
        let r = std::panic::catch_unwind(|| x.assert_some(|s| s.0 == 9, "nine"));
        assert!(r.is_err());
        let cov = x.coverage(|s| if s.0 == 0 { "zero".into() } else { "positive".into() });
        assert_eq!(cov["zero"], 1);
        assert_eq!(cov["positive"], 2);
    }

    #[test]
    fn a_limit_truncates_rather_than_running_away() {
        let m = Counter {
            ceiling: 1_000_000,
            bad: None,
        };
        let x = explore_with(
            &m,
            Limits {
                max_states: 100,
                max_depth: usize::MAX,
            },
        );
        assert!(x.truncated());
        assert_eq!(x.len(), 100);
        assert!(std::panic::catch_unwind(|| x.assert_ok(&m)).is_err());
        let shallow = explore_with(
            &m,
            Limits {
                max_states: 1_000,
                max_depth: 7,
            },
        );
        assert!(!shallow.truncated());
        assert_eq!(shallow.max_depth(), 7);
    }

    #[test]
    fn a_gantt_has_a_lane_per_component_and_no_stray_punctuation() {
        let m = Counter {
            ceiling: 3,
            bad: Some(2),
        };
        let x = explore(&m);
        let v = x.violation().unwrap();
        let g = x.gantt(&m, v.state, "two ups: a, b");
        assert!(g.starts_with("gantt\n    title two ups= a; b\n"));
        assert!(g.contains("section up"));
        assert!(!g.contains("section reset"));
        assert!(g.contains("Up :e1, 1, 1ms"));
        assert!(g.contains("Up :e2, 2, 1ms"));
    }
}
