//! Who is alive, and what is written down when something stops.
//!
//! The firmware is two loops on two cores. Nothing in either can see the
//! other stop: a panic spins its core forever with whatever lock it held,
//! a driver waits on a line that never moves, and the other core carries
//! on until it needs that lock - then it stops too, and the board sits
//! with its LEDs frozen at whatever the last pass left them, answering
//! nobody. From the outside that is one symptom for every cause, and the
//! console that would have named the cause was not plugged in.
//!
//! The supervisor is the policy behind three mechanisms the firmware adds
//! for that:
//!
//! - **Heartbeats.** Each supervised task says what it is doing and that
//!   it is still doing it - at the top of every pass, and from inside any
//!   wait long enough to look like a stop. [`Supervisor::beat`].
//! - **A monitor** that reads them on a period and decides whether every
//!   task is inside its bound. [`Supervisor::check`]. A task that is not
//!   is a [`Stall`], which names the task and the phase it stopped in -
//!   the two things a reader needs and the two things a reset erases.
//! - **A hardware watchdog** the monitor feeds only while every task is
//!   inside its bound. It is what catches the monitor itself: a stall on
//!   the monitor's own core, or a monitor that blocked trying to write the
//!   stall to flash because the dead core holds the lock it needs.
//!
//! The order of what the monitor does on a stall is the part that matters
//! and the part the state space test checks: the stall goes into RTC RAM
//! first, with no lock taken, then to flash, then the board resets. If the
//! flash write blocks, the watchdog resets the board and the RTC copy is
//! still there for the next boot to write down.
//!
//! Everything here is pure. The firmware supplies the clock and the
//! atomics the heartbeats live in.

/// A task the monitor watches.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Task {
    /// The hardware loop: radio, GPS, card and panel, on the second core.
    Loop = 1,
    /// The BLE serve loop: advertising, the session, the duty cycle and
    /// the deep sleep, on the first core.
    Serve = 2,
}

impl Task {
    pub const ALL: [Task; 2] = [Task::Loop, Task::Serve];

    /// How long the task may go without a heartbeat before it is a stall.
    ///
    /// Each bound is set by the longest thing the task does *between*
    /// heartbeats, with room on top, not by how fast it normally runs.
    /// The loop beats inside a transmit and inside a GPS ack wait, so what
    /// is left is a card mount that retries on a failing card, a config
    /// apply that writes both stores, and a panel refresh. The serve loop
    /// beats inside every wait it has, so what is left is the controller
    /// coming up and a flash sector erase inside a bulk transfer.
    pub const fn bound_ms(self) -> u32 {
        match self {
            Task::Loop => 15_000,
            Task::Serve => 20_000,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Task::Loop => "hardware loop",
            Task::Serve => "ble serve loop",
        }
    }

    pub const fn as_wire(self) -> u8 {
        self as u8
    }

    pub const fn from_wire(v: u8) -> Option<Self> {
        match v {
            1 => Some(Task::Loop),
            2 => Some(Task::Serve),
            _ => None,
        }
    }

    const fn index(self) -> usize {
        match self {
            Task::Loop => 0,
            Task::Serve => 1,
        }
    }
}

/// What a task was doing when it last spoke. One flat set for both tasks,
/// so a stall record carries one byte and a reader needs one table.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Phase {
    /// Nothing recorded yet.
    #[default]
    Boot = 0,
    // -- the hardware loop
    /// Draining the request queue and carrying out the effects.
    Requests = 1,
    /// The receiver: sentences, settings, the hop clock.
    Gps = 2,
    /// Deciding whether a beacon is due.
    Beacon = 3,
    /// Inside a transmit, waiting for the radio to finish.
    TxSend = 4,
    /// Polling the radio for a frame.
    Receive = 5,
    // 6 was the SD card, which the firmware no longer drives.
    /// The J5 panel and the magnetometer.
    Panel = 7,
    /// The status line.
    Status = 8,
    /// Bringing the radio up from its config.
    RadioInit = 9,
    /// Parking the receiver, or waking it, or waiting for its ack.
    GpsCtl = 10,
    /// Reading a stored config and adopting it.
    ConfigLoad = 11,
    /// Applying a pushed config.
    ConfigApply = 12,
    // -- the serve loop
    /// Bringing the BLE controller and host up.
    BleInit = 13,
    /// Advertising, waiting for a central or the budget.
    Advertise = 14,
    /// A central connected; the attribute server is attaching.
    Attach = 15,
    /// A session is running.
    Session = 16,
    /// A config or bulk write is being applied for a central.
    Write = 17,
    /// The modem is down for the tracker's off period.
    BleDown = 18,
    /// Waiting for the hardware loop to park before a deep sleep.
    Park = 19,
    /// Entering deep sleep.
    Sleep = 20,
}

impl Phase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Phase::Boot => "boot",
            Phase::Requests => "requests",
            Phase::Gps => "gps",
            Phase::Beacon => "beacon",
            Phase::TxSend => "tx send",
            Phase::Receive => "receive",
            Phase::Panel => "panel",
            Phase::Status => "status",
            Phase::RadioInit => "radio init",
            Phase::GpsCtl => "gps control",
            Phase::ConfigLoad => "config load",
            Phase::ConfigApply => "config apply",
            Phase::BleInit => "ble init",
            Phase::Advertise => "advertise",
            Phase::Attach => "attach",
            Phase::Session => "session",
            Phase::Write => "write",
            Phase::BleDown => "ble down",
            Phase::Park => "park",
            Phase::Sleep => "sleep",
        }
    }

    pub const fn as_wire(self) -> u8 {
        self as u8
    }

    pub const fn from_wire(v: u8) -> Option<Self> {
        Some(match v {
            0 => Phase::Boot,
            1 => Phase::Requests,
            2 => Phase::Gps,
            3 => Phase::Beacon,
            4 => Phase::TxSend,
            5 => Phase::Receive,
            7 => Phase::Panel,
            8 => Phase::Status,
            9 => Phase::RadioInit,
            10 => Phase::GpsCtl,
            11 => Phase::ConfigLoad,
            12 => Phase::ConfigApply,
            13 => Phase::BleInit,
            14 => Phase::Advertise,
            15 => Phase::Attach,
            16 => Phase::Session,
            17 => Phase::Write,
            18 => Phase::BleDown,
            19 => Phase::Park,
            20 => Phase::Sleep,
            _ => return None,
        })
    }
}

/// How often the monitor reads the heartbeats and feeds the watchdog.
pub const MONITOR_PERIOD_MS: u32 = 500;

/// How long the hardware watchdog runs without a feed before it resets
/// the board.
///
/// Well past the longest task bound plus a monitor period, so the monitor
/// always gets to say *which* task stalled before the watchdog says only
/// that something did. Long enough to cover the boot - the flash reads,
/// the second core coming up, the controller's init - before the monitor
/// task is running to feed it.
pub const WDT_TIMEOUT_MS: u32 = 30_000;

/// A task that went quiet past its bound.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Stall {
    pub task: Task,
    /// What it said it was doing when it last spoke.
    pub phase: Phase,
    /// How long ago that was.
    pub silent_ms: u32,
}

/// What the monitor decided.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Verdict {
    /// Every task is inside its bound: feed the watchdog.
    Alive,
    /// Write it down and reset. Only the first stall found is reported;
    /// a second task stalled behind it is usually the first one's victim,
    /// waiting on a lock the first holds, and naming it would point the
    /// reader at the wrong core.
    Stalled(Stall),
}

/// The heartbeats, as the monitor reads them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Supervisor {
    seen_ms: [u64; Task::ALL.len()],
    phase: [Phase; Task::ALL.len()],
}

impl Supervisor {
    /// Every task counted as alive now. The boot beats for all of them,
    /// so a task that never starts is a stall from the boot line rather
    /// than one that is never noticed.
    pub const fn new(now_ms: u64) -> Self {
        Self {
            seen_ms: [now_ms; Task::ALL.len()],
            phase: [Phase::Boot; Task::ALL.len()],
        }
    }

    /// `task` is alive and in `phase`.
    pub fn beat(&mut self, task: Task, phase: Phase, now_ms: u64) {
        self.seen_ms[task.index()] = now_ms;
        self.phase[task.index()] = phase;
    }

    /// What `task` last said it was doing.
    pub fn phase(&self, task: Task) -> Phase {
        self.phase[task.index()]
    }

    /// How long since `task` last spoke.
    pub fn silent_ms(&self, task: Task, now_ms: u64) -> u32 {
        now_ms
            .saturating_sub(self.seen_ms[task.index()])
            .min(u32::MAX as u64) as u32
    }

    /// The monitor's decision at `now_ms`. Tasks are checked in the order
    /// of [`Task::ALL`], and the first past its bound is the verdict.
    pub fn check(&self, now_ms: u64) -> Verdict {
        for task in Task::ALL {
            let silent_ms = self.silent_ms(task, now_ms);
            if silent_ms > task.bound_ms() {
                return Verdict::Stalled(Stall {
                    task,
                    phase: self.phase(task),
                    silent_ms,
                });
            }
        }
        Verdict::Alive
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_supervisor_is_alive() {
        let s = Supervisor::new(1_000);
        assert_eq!(s.check(1_000), Verdict::Alive);
        assert_eq!(s.check(1_000 + Task::Loop.bound_ms() as u64), Verdict::Alive);
    }

    #[test]
    fn a_task_past_its_bound_is_a_stall_naming_its_phase() {
        let mut s = Supervisor::new(0);
        s.beat(Task::Loop, Phase::TxSend, 10_000);
        s.beat(Task::Serve, Phase::Advertise, 10_000);
        let at = 10_000 + Task::Loop.bound_ms() as u64 + 1;
        // The serve bound is longer, so only the loop is out.
        assert_eq!(
            s.check(at),
            Verdict::Stalled(Stall {
                task: Task::Loop,
                phase: Phase::TxSend,
                silent_ms: Task::Loop.bound_ms() + 1,
            })
        );
        s.beat(Task::Loop, Phase::Receive, at);
        assert_eq!(s.check(at), Verdict::Alive);
    }

    #[test]
    fn the_first_task_in_order_is_the_one_reported() {
        let mut s = Supervisor::new(0);
        // Both long gone: the loop is named, the serve loop is its victim.
        let at = Task::Serve.bound_ms() as u64 * 2;
        match s.check(at) {
            Verdict::Stalled(stall) => assert_eq!(stall.task, Task::Loop),
            v => panic!("{v:?}"),
        }
        s.beat(Task::Loop, Phase::Receive, at);
        match s.check(at) {
            Verdict::Stalled(stall) => assert_eq!(stall.task, Task::Serve),
            v => panic!("{v:?}"),
        }
    }

    #[test]
    fn the_watchdog_outlasts_every_bound_and_a_period() {
        for task in Task::ALL {
            assert!(task.bound_ms() + 2 * MONITOR_PERIOD_MS < WDT_TIMEOUT_MS);
        }
    }

    #[test]
    fn tasks_and_phases_round_trip_the_wire() {
        for task in Task::ALL {
            assert_eq!(Task::from_wire(task.as_wire()), Some(task));
        }
        assert_eq!(Task::from_wire(0), None);
        for v in 0..=u8::MAX {
            if let Some(p) = Phase::from_wire(v) {
                assert_eq!(p.as_wire(), v);
                assert!(!p.as_str().is_empty());
            }
        }
        assert_eq!(Phase::from_wire(Phase::Sleep.as_wire() + 1), None);
    }
}
