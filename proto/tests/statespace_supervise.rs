//! Every way the two loops can stop, and whether the board comes back
//! with the right thing written down.
//!
//! The board model in `statespace_firmware.rs` walks what the firmware
//! *decides*: every ordering of connects, writes, passes and sleeps. It
//! cannot walk a loop that stops deciding, because a stop is the absence
//! of an event and a breadth-first walk over enabled events never
//! generates one. That is why a hang was invisible to it: the hardware
//! loop always took its next pass, the serve loop always answered its
//! next accept, and "the board is advertising" was reachable from every
//! state because every state had a next event. The model was faithful to
//! the policy and the policy has no failure in it.
//!
//! This model adds the failures and the mechanism that answers them. A
//! task is alive or stalled; a stalled task never beats again. The
//! monitor ticks, runs the real [`Supervisor::check`], and on a stall
//! writes the breadcrumb and resets - unless it blocks on the way (the
//! flash write it makes next needs a lock the dead core may hold), in
//! which case the hardware watchdog it stopped feeding resets the board
//! instead. The monitor can also stop on its own.
//!
//! Time is a tick of the monitor's period. A live task beats inside every
//! tick - that is what the firmware's heartbeats are - so silence in the
//! model is only ever a stalled task's, and a false alarm is provable
//! rather than merely unlikely.
//!
//! Checked in every state:
//!
//! - a stalled task is detected within its bound: no state has a task
//!   silent past its bound with the monitor alive and not already
//!   resetting;
//! - the watchdog fires within its bound: no state has it unfed past it;
//! - a stall crumb names a task that was in fact stalled, in the phase it
//!   stalled in;
//! - a reset the watchdog caused still carries the crumb the monitor
//!   wrote before it blocked;
//! - and from every state the board can come back with both loops alive.
//!
//! The same model with the supervisor removed fails the last one, in one
//! event, which is the trace the firmware could not print.

use midair_explore::{explore, Machine};
use midair_proto::supervise::{
    Phase, Stall, Supervisor, Task, Verdict, MONITOR_PERIOD_MS, WDT_TIMEOUT_MS,
};

/// One model tick, in the policy's milliseconds: ten monitor periods, so
/// the walk stays small while the bounds are still the policy's own.
const TICK_MS: u64 = MONITOR_PERIOD_MS as u64 * 10;

/// Ticks past which the watchdog fires: the first tick whose age exceeds
/// its timeout.
fn wdt_ticks() -> u8 {
    (WDT_TIMEOUT_MS as u64 / TICK_MS) as u8 + 1
}

fn bound_ticks(task: Task) -> u8 {
    (task.bound_ms() as u64 / TICK_MS) as u8 + 1
}

fn idx(task: Task) -> usize {
    match task {
        Task::Loop => 0,
        Task::Serve => 1,
    }
}

/// The phases each task can be doing when it stops. A few of each, so
/// the crumb has something to get wrong.
fn phases(task: Task) -> [Phase; 2] {
    match task {
        Task::Loop => [Phase::TxSend, Phase::Card],
        Task::Serve => [Phase::Advertise, Phase::Session],
    }
}

/// What a reset was caused by.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Cause {
    /// The monitor found a stall, wrote it down and reset.
    Monitor,
    /// The watchdog ran out.
    Watchdog,
}

/// What the boot after a reset found to write into the log.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Report {
    cause: Cause,
    crumb: Option<Stall>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct State {
    /// Ticks since each task last beat, saturated one past its bound.
    silent: [u8; 2],
    phase: [Phase; 2],
    dead: [bool; 2],
    monitor_alive: bool,
    /// Ticks since the watchdog was fed, saturated one past its bound.
    unfed: u8,
    /// The monitor found a stall this tick and is about to reset.
    resetting: bool,
    /// What is in RTC RAM: the stall the monitor wrote before doing
    /// anything else.
    crumb: Option<Stall>,
    /// What the boot wrote to the log, until the next event clears it.
    report: Option<Report>,
}

impl State {
    fn boot(cause: Option<Cause>, crumb: Option<Stall>) -> Self {
        Self {
            silent: [0; 2],
            phase: [Phase::Boot; 2],
            dead: [false; 2],
            monitor_alive: true,
            unfed: 0,
            resetting: false,
            crumb: None,
            report: cause.map(|cause| Report { cause, crumb }),
        }
    }

    /// The real supervisor, rebuilt from the model's relative times.
    fn supervisor(&self) -> (Supervisor, u64) {
        let now = TICK_MS * 8;
        let mut s = Supervisor::new(now);
        for task in Task::ALL {
            let i = idx(task);
            s.beat(task, self.phase[i], now - u64::from(self.silent[i]) * TICK_MS);
        }
        (s, now)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Event {
    /// A live task does something and says so.
    Work(Task, Phase),
    /// A task stops for good.
    Stall(Task),
    /// The monitor itself stops.
    MonitorStall,
    /// One monitor period: live tasks have beaten, the monitor checks and
    /// feeds or decides to reset, the watchdog counts.
    Tick,
    /// The monitor's reset lands.
    Reset,
    /// The monitor blocks after writing the crumb: the flash write it
    /// makes next needs a lock the dead core holds.
    MonitorBlocks,
}

struct Watched {
    supervised: bool,
}

impl Machine for Watched {
    type State = State;
    type Event = Event;

    fn initial(&self) -> Vec<State> {
        vec![State::boot(None, None)]
    }

    fn events(&self, s: &State) -> Vec<Event> {
        let mut ev = Vec::new();
        if s.resetting {
            ev.push(Event::Reset);
            ev.push(Event::MonitorBlocks);
            return ev;
        }
        for task in Task::ALL {
            if !s.dead[idx(task)] {
                for phase in phases(task) {
                    ev.push(Event::Work(task, phase));
                }
                ev.push(Event::Stall(task));
            }
        }
        if self.supervised && s.monitor_alive {
            ev.push(Event::MonitorStall);
        }
        ev.push(Event::Tick);
        ev
    }

    fn step(&self, s: &State, e: &Event) -> State {
        let mut s = *s;
        s.report = None;
        match *e {
            Event::Work(task, phase) => {
                s.phase[idx(task)] = phase;
                s.silent[idx(task)] = 0;
            }
            Event::Stall(task) => s.dead[idx(task)] = true,
            Event::MonitorStall => s.monitor_alive = false,
            Event::Tick => {
                for task in Task::ALL {
                    let i = idx(task);
                    s.silent[i] = if s.dead[i] {
                        (s.silent[i] + 1).min(bound_ticks(task))
                    } else {
                        0
                    };
                }
                if !self.supervised {
                    return s;
                }
                s.unfed = (s.unfed + 1).min(wdt_ticks());
                if s.unfed >= wdt_ticks() {
                    // The hardware fires. Whatever the monitor wrote before
                    // it stopped is still in RTC RAM.
                    return State::boot(Some(Cause::Watchdog), s.crumb);
                }
                if s.monitor_alive {
                    let (sup, now) = s.supervisor();
                    match sup.check(now) {
                        Verdict::Alive => s.unfed = 0,
                        Verdict::Stalled(stall) => {
                            // RTC RAM first, with no lock taken.
                            s.crumb = Some(stall);
                            s.resetting = true;
                        }
                    }
                }
            }
            Event::Reset => return State::boot(Some(Cause::Monitor), s.crumb),
            Event::MonitorBlocks => {
                s.resetting = false;
                s.monitor_alive = false;
            }
        }
        s
    }

    fn check(&self, s: &State) -> Result<(), String> {
        if !self.supervised {
            return Ok(());
        }
        if s.monitor_alive && !s.resetting {
            for task in Task::ALL {
                if u64::from(s.silent[idx(task)]) * TICK_MS > u64::from(task.bound_ms()) {
                    return Err(format!("{task:?} is silent past its bound and nobody noticed"));
                }
            }
        }
        if s.unfed >= wdt_ticks() {
            return Err("the watchdog is past its timeout and has not fired".into());
        }
        if let Some(stall) = s.crumb {
            if !s.dead[idx(stall.task)] {
                return Err(format!("a crumb names {:?}, which is alive", stall.task));
            }
            if s.phase[idx(stall.task)] != stall.phase {
                return Err(format!(
                    "a crumb says {:?} stopped in {:?}, it is in {:?}",
                    stall.task,
                    stall.phase,
                    s.phase[idx(stall.task)]
                ));
            }
        }
        Ok(())
    }

    fn check_step(&self, from: &State, e: &Event, to: &State) -> Result<(), String> {
        let Some(report) = to.report else {
            return Ok(());
        };
        // Every boot after a reset says what caused it and what was found.
        match (report.cause, report.crumb) {
            // The monitor reset: it names the task that had stalled.
            (Cause::Monitor, Some(stall)) => {
                if !from.dead[idx(stall.task)] {
                    return Err(format!("the log blames {:?}, which was alive", stall.task));
                }
            }
            (Cause::Monitor, None) => return Err("the monitor reset with nothing written".into()),
            // The watchdog: with a crumb when the monitor got that far,
            // without one only when the monitor itself was the stall.
            (Cause::Watchdog, Some(stall)) => {
                if from.crumb != Some(stall) {
                    return Err("the watchdog reset lost or changed the crumb".into());
                }
            }
            (Cause::Watchdog, None) => {
                if from.monitor_alive {
                    return Err("the watchdog fired under a live monitor".into());
                }
                if from.crumb.is_some() {
                    return Err("the watchdog reset dropped the crumb".into());
                }
            }
        }
        if *e != Event::Tick && *e != Event::Reset {
            return Err(format!("{e:?} is not a reset"));
        }
        Ok(())
    }

    fn describe(&self, s: &State) -> String {
        format!(
            "loop {} silent {} in {:?} | serve {} silent {} in {:?} | monitor {} unfed {} resetting {} | crumb {:?} | report {:?}",
            if s.dead[0] { "DEAD" } else { "alive" },
            s.silent[0],
            s.phase[0],
            if s.dead[1] { "DEAD" } else { "alive" },
            s.silent[1],
            s.phase[1],
            if s.monitor_alive { "alive" } else { "DEAD" },
            s.unfed,
            s.resetting,
            s.crumb,
            s.report
        )
    }

    fn lane(&self, e: &Event) -> &'static str {
        match e {
            Event::Work(Task::Loop, _) | Event::Stall(Task::Loop) => "hardware loop",
            Event::Work(Task::Serve, _) | Event::Stall(Task::Serve) => "ble serve loop",
            Event::MonitorStall | Event::MonitorBlocks | Event::Reset => "monitor",
            Event::Tick => "time",
        }
    }
}

fn both_alive(s: &State) -> bool {
    !s.dead[0] && !s.dead[1] && s.monitor_alive
}

/// The supervised board: every stall is found, named and recovered.
#[test]
fn every_stall_is_named_and_recovered() {
    let m = Watched { supervised: true };
    let x = explore(&m);
    println!("supervise model: {x}");
    x.assert_ok(&m);

    // Coverage: the invariants were about something.
    x.assert_some(
        |s| s.report.is_some_and(|r| r.cause == Cause::Monitor && r.crumb.is_some_and(|c| c.task == Task::Loop && c.phase == Phase::TxSend)),
        "the monitor reset a board whose loop stopped inside a transmit, and said so",
    );
    x.assert_some(
        |s| s.report.is_some_and(|r| r.cause == Cause::Watchdog && r.crumb.is_some()),
        "the monitor blocked after writing the crumb and the watchdog finished the reset",
    );
    x.assert_some(
        |s| s.report.is_some_and(|r| r.cause == Cause::Watchdog && r.crumb.is_none()),
        "the monitor itself stopped and the watchdog caught it",
    );
    x.assert_some(|s| s.dead[0] && s.dead[1], "both loops stopped");
    x.assert_some(
        |s| s.report.is_some_and(|r| r.crumb.is_some_and(|c| c.task == Task::Serve)),
        "the serve loop was the one named",
    );

    // Liveness: whatever stops, both loops come back.
    x.assert_always_reachable(&m, both_alive, "both loops and the monitor are alive");
}

/// The same board with nothing watching it: a stalled loop is a state
/// nothing leads out of. This is the trace the firmware could not print,
/// and the reason the board model never found the hang - it has no event
/// for a loop that stops, so it has no state like this one.
#[test]
fn without_a_supervisor_a_stall_is_forever() {
    let m = Watched { supervised: false };
    let x = explore(&m);
    println!("unsupervised model: {x}");
    x.assert_ok(&m);
    let stuck = x
        .trap(both_alive)
        .expect("an unsupervised stall recovered on its own, which no firmware can do");
    println!(
        "dark for good, in {} event(s):\n{}",
        x.trace(stuck).len() - 1,
        x.format_trace(&m, stuck)
    );
    println!("{}", x.gantt(&m, stuck, "A loop stops and nothing brings it back"));
}
