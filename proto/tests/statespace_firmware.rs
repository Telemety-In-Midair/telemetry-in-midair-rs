//! Every reachable state of the board around its two radios, and every
//! way the pieces that interrupt them can interleave.
//!
//! The pieces are the firmware's own: [`Serve`] decides what the BLE loop
//! does with a connect, a drop, an expiry, a nap or a moved mode;
//! [`Posture`] decides what the hardware task raises and lowers for each
//! [`Request`]; [`Requests`] is the queue between them; [`apply`] and
//! [`dispatch`] are what a config write does on the way in. What this
//! file adds is the glue the firmware keeps in `state.rs` and `main.rs` -
//! the sleep-now cell, the mode signal, the transfer lock, the transmit in
//! flight, the deep sleep and the wake - and a set of events that stand for
//! everything that can happen to a board: a central connects or drops, a
//! budget expires, a write lands over BLE or over the console, a beacon
//! starts and ends, a transfer begins and ends, the hardware loop takes a
//! pass, the chip sleeps and wakes.
//!
//! Time is two-valued. Every event happens either before the serve
//! budget's deadline or at it, which is all the policy ever asks; the
//! deadline itself is one of a handful of values the settings decide, so
//! the state stays finite.
//!
//! What is checked, in every state the walk reaches:
//!
//! - the posture is consistent with the settings it reports: a tracking
//!   board has its receiver and its radio exactly where the two override
//!   flags say, an idle board and a wake check have both down, a board
//!   parked for sleep has everything down;
//! - a sleeping board is fully parked - the park cannot be lost, skipped
//!   or undone by anything that arrives while it is happening;
//! - a listening node never transmits; a node in a transfer, or with a
//!   sleep pending, never starts a transmit;
//! - once the hardware loop has drained its requests, the mode it is in
//!   is the mode the settings report;
//! - and from every state the board can still be reached over BLE again,
//!   and can still be commanded into tracking - there is no way to leave
//!   it dark for good.

use midair_explore::{explore, Machine};
use midair_proto::ble::{self, Mode};
use midair_proto::bulk::Owner;
use midair_proto::posture::{Card, Effect, Gps, Posture, Radio, Request, Requests};
use midair_proto::session::{
    self, apply, boot_mode, dispatch, down_period, Accepted, Pass, Serve, Stored, Then,
    PFLAG_GPS_SLEEP, PFLAG_WIO_SLEEP,
};

/// Where the BLE side is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Ble {
    /// Advertising, waiting for a central or the budget.
    Advertising,
    /// A central is connected and the session arms are running.
    Connected,
    /// The modem is down for the tracker's off period.
    Down,
    /// A deep sleep was decided; the hardware loop is parking.
    Parking,
    /// The chip is in deep sleep.
    Asleep,
}

/// A config write, by what it asks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Write {
    Mode(Mode),
    GpsSleep(bool),
    WioSleep(bool),
    /// A nap of the configured cadence.
    SleepNow,
}

impl Write {
    fn bytes(self) -> Vec<u8> {
        match self {
            Write::Mode(m) => vec![ble::CFG_MODE, 1, m.as_wire()],
            Write::GpsSleep(on) => vec![ble::CFG_GPS_SLEEP, 1, on as u8],
            Write::WioSleep(on) => vec![ble::CFG_WIO_SLEEP, 1, on as u8],
            Write::SleepNow => {
                let mut v = vec![ble::CFG_SLEEP_NOW, 4];
                v.extend_from_slice(&0u32.to_le_bytes());
                v
            }
        }
    }
}

/// Which transport a write or a transfer came over.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Via {
    Ble,
    Usb,
}

/// What a completed transfer carried.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Kind {
    Config,
    Firmware,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Event {
    /// A central connected and the attribute server attached.
    Connect,
    /// A central tried and the handshake fizzled.
    ConnectFailed,
    /// The serve budget ran out with nobody interested.
    Expire,
    /// The central went away, or the session ended on its own.
    Disconnect,
    /// The session ended because a sleep was commanded during it.
    SessionEndsForSleep,
    /// The modem's off period is over.
    DownOver,
    /// A config write.
    Write(Via, Write),
    /// The hardware loop takes a pass: drains the requests, runs the
    /// effects.
    LoopPass,
    /// A beacon or a repeat starts.
    TxStart,
    /// It ends.
    TxEnd,
    /// A bulk transfer opens on this transport.
    TransferBegin(Via),
    /// It completes and verifies as this kind.
    TransferEnd(Kind),
    /// The park finished and the chip goes down.
    Sleep,
    /// The RTC timer fires.
    Wake,
}

/// The whole board.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Board {
    stored: Stored,
    serve: Serve,
    ble: Ble,
    posture: Posture,
    requests: Requests,
    /// The sleep-now cell: seconds a command asked the board to sleep
    /// for, until the serve loop acts on it.
    sleep_now: Option<u32>,
    /// The hardware loop finished the park.
    sleep_ready: bool,
    transfer: Option<Owner>,
    /// A LoRa transmit is on the air.
    tx: bool,
    /// The last request drained was one the posture ignored because the
    /// board was already parked - kept so a transition check can say so.
    ignored_while_parked: bool,
}

impl Board {
    fn cold_boot(stored: Stored) -> Self {
        let mut b = Self {
            stored,
            serve: Serve::new(0, &stored),
            ble: Ble::Advertising,
            posture: Posture::at_boot(Mode::Stored, &stored).0,
            requests: Requests::new(),
            sleep_now: None,
            sleep_ready: false,
            transfer: None,
            tx: false,
            ignored_while_parked: false,
        };
        b.boot(false);
        b
    }

    /// What every boot does: pick the mode from the persisted one and the
    /// wake cause, raise what it raises, start serving on its budget.
    fn boot(&mut self, woke_from_sleep: bool) {
        let mode = boot_mode(self.stored.mode, woke_from_sleep);
        self.stored.mode = mode;
        self.posture = Posture::at_boot(mode, &self.stored).0;
        self.requests = Requests::new();
        self.sleep_now = None;
        self.sleep_ready = false;
        self.transfer = None;
        self.tx = false;
        self.ignored_while_parked = false;
        self.serve = Serve::new(0, &self.stored);
        self.loop_top();
    }

    /// The top of the serve loop.
    fn loop_top(&mut self) {
        let (_, pass) = self.serve.pass(0, &self.stored);
        match pass {
            Pass::Advertise { .. } => self.ble = Ble::Advertising,
            Pass::Sleep { interval_s } => self.begin_sleep(interval_s),
            Pass::BleDown { .. } => self.ble = Ble::Down,
        }
    }

    /// `enter_deep_sleep`: clear the command that got here, ask the
    /// hardware loop to park, wait for it.
    fn begin_sleep(&mut self, _secs: u32) {
        self.sleep_now = None;
        self.sleep_ready = false;
        self.requests.push(Request::PrepareSleep);
        self.ble = Ble::Parking;
    }

    fn after(&mut self, then: Then) {
        match then {
            Then::Serve => self.ble = Ble::Connected,
            Then::Retry | Then::Continue => self.loop_top(),
            Then::Sleep(secs) => self.begin_sleep(secs),
            Then::Return => match down_period(&self.stored) {
                Some(_) => self.ble = Ble::Down,
                None => self.loop_top(),
            },
        }
    }

    /// The firmware's `apply_config`, on either transport.
    fn write(&mut self, via: Via, w: Write) {
        let outcome = apply(&mut self.stored, &w.bytes());
        let d = dispatch(outcome.action, &self.stored);
        if let Some(r) = d.request {
            self.requests.push(r);
        }
        if let Some(secs) = d.sleep_now {
            self.sleep_now = Some(secs);
        }
        let _ = via;
        // The signals wake whichever wait is running.
        match self.ble {
            Ble::Advertising => {
                if let Some(secs) = self.sleep_now {
                    let step = self.serve.on_accept(0, Accepted::SleepNow(secs), &self.stored);
                    self.after(step.then);
                } else if d.mode_signal {
                    let step = self.serve.on_accept(0, Accepted::ModeChanged, &self.stored);
                    self.after(step.then);
                }
            }
            Ble::Down => {
                if self.sleep_now.is_some() {
                    let secs = self.sleep_now.unwrap();
                    self.begin_sleep(secs);
                } else if d.mode_signal {
                    self.loop_top();
                }
            }
            // Connected: the session's own arm ends it, as a separate
            // event, since the ack has to leave first. Parking and asleep:
            // nothing is waiting on either signal.
            Ble::Connected | Ble::Parking | Ble::Asleep => {}
        }
    }

    fn end_session(&mut self) {
        // A transfer the phone was midway through does not outlive it.
        if self.transfer == Some(Owner::Ble) {
            self.transfer = None;
        }
        let then = self.serve.on_session_end(0, self.sleep_now.take(), &self.stored);
        self.after(then);
    }

    fn loop_pass(&mut self) {
        self.ignored_while_parked = false;
        while let Some(r) = self.requests.take() {
            let parked = self.posture.card == Card::Parked;
            let fx = self.posture.on(r, &self.stored);
            if parked && fx.is_empty() && !matches!(r, Request::PrepareSleep | Request::Reboot) {
                self.ignored_while_parked = true;
            }
            for e in fx {
                match e {
                    Effect::SleepReady => self.sleep_ready = true,
                    Effect::Reboot => {
                        // A reset: the persisted mode comes back, and the
                        // boot is a cold one.
                        self.stored.mode = self.stored.mode.persisted();
                        self.boot(false);
                        return;
                    }
                    _ => {}
                }
            }
        }
    }

    fn awake(&self) -> bool {
        !matches!(self.ble, Ble::Asleep)
    }

    fn bounded(&self) -> bool {
        self.stored.at_expiry() != session::Next::Advertise
    }
}

struct Firmware;

impl Machine for Firmware {
    type State = Board;
    type Event = Event;

    /// Every persisted mode, every override flag combination, on a bench
    /// board and on a deployed one.
    fn initial(&self) -> Vec<Board> {
        let mut out = Vec::new();
        for mode in [Mode::Stored, Mode::Tracking, Mode::Listening] {
            for flags in [0, PFLAG_GPS_SLEEP, PFLAG_WIO_SLEEP, PFLAG_GPS_SLEEP | PFLAG_WIO_SLEEP] {
                let bench = Stored {
                    mode,
                    flags,
                    ..Stored::new()
                };
                let deployed = Stored {
                    mode,
                    flags,
                    sleep_interval_s: 120,
                    adv_window_s: 15,
                    idle_timeout_s: 600,
                    ble_off_s: 30,
                    ble_on_s: 20,
                    ..Stored::new()
                };
                out.push(Board::cold_boot(bench));
                out.push(Board::cold_boot(deployed));
            }
        }
        out
    }

    fn events(&self, b: &Board) -> Vec<Event> {
        let mut ev = Vec::new();
        match b.ble {
            Ble::Advertising => {
                ev.push(Event::Connect);
                ev.push(Event::ConnectFailed);
                if b.bounded() {
                    ev.push(Event::Expire);
                }
            }
            Ble::Connected => {
                ev.push(Event::Disconnect);
                if b.sleep_now.is_some() {
                    ev.push(Event::SessionEndsForSleep);
                }
                for w in writes() {
                    ev.push(Event::Write(Via::Ble, w));
                }
                if b.transfer.is_none() {
                    ev.push(Event::TransferBegin(Via::Ble));
                }
            }
            Ble::Down => ev.push(Event::DownOver),
            Ble::Parking => {
                if b.sleep_ready {
                    ev.push(Event::Sleep);
                }
            }
            Ble::Asleep => ev.push(Event::Wake),
        }
        if b.awake() {
            // The console is alive whenever the chip is.
            for w in writes() {
                ev.push(Event::Write(Via::Usb, w));
            }
            if b.transfer.is_none() {
                ev.push(Event::TransferBegin(Via::Usb));
            }
            if b.transfer.is_some() {
                ev.push(Event::TransferEnd(Kind::Config));
                ev.push(Event::TransferEnd(Kind::Firmware));
            }
            // The loop cannot take a pass while it is inside a transmit.
            if !b.tx {
                ev.push(Event::LoopPass);
                let sleep_pending = b.sleep_now.is_some() || b.requests.sleep_pending();
                if b.posture.may_transmit(true, b.transfer.is_some(), sleep_pending) {
                    ev.push(Event::TxStart);
                }
            } else {
                ev.push(Event::TxEnd);
            }
        }
        ev
    }

    fn step(&self, b: &Board, e: &Event) -> Board {
        let mut b = *b;
        b.ignored_while_parked = false;
        match *e {
            Event::Connect => {
                let step = b.serve.on_accept(0, Accepted::Connected, &b.stored);
                if step.promote {
                    b.stored.mode = Mode::Idle;
                    b.requests.push(Request::Mode(Mode::Idle));
                }
                b.after(step.then);
            }
            Event::ConnectFailed => {
                let step = b.serve.on_accept(0, Accepted::Failed, &b.stored);
                if step.promote {
                    b.stored.mode = Mode::Idle;
                    b.requests.push(Request::Mode(Mode::Idle));
                }
                b.after(step.then);
            }
            Event::Expire => {
                let ends = b.serve.window().ends_ms();
                let step = b.serve.on_accept(ends, Accepted::Expired, &b.stored);
                b.after(step.then);
            }
            Event::Disconnect | Event::SessionEndsForSleep => b.end_session(),
            Event::DownOver => b.loop_top(),
            Event::Write(via, w) => b.write(via, w),
            Event::LoopPass => b.loop_pass(),
            Event::TxStart => b.tx = true,
            Event::TxEnd => b.tx = false,
            Event::TransferBegin(via) => {
                b.transfer = Some(match via {
                    Via::Ble => Owner::Ble,
                    Via::Usb => Owner::Usb,
                });
            }
            Event::TransferEnd(kind) => {
                b.transfer = None;
                b.requests.push(match kind {
                    Kind::Config => Request::ApplyConfig,
                    Kind::Firmware => Request::Reboot,
                });
            }
            Event::Sleep => b.ble = Ble::Asleep,
            Event::Wake => b.boot(true),
        }
        b
    }

    fn check(&self, b: &Board) -> Result<(), String> {
        // The posture rule is against the flags the loop has seen: a
        // write lands in the settings first and in the hardware on the
        // next pass, so it is checked once the loop has caught up.
        if b.requests.is_empty() {
            b.posture
                .consistent(&b.stored)
                .map_err(|e| format!("posture: {e}"))?;
        }
        if b.posture.card == Card::Parked
            && (b.posture.radio != Radio::Asleep || b.posture.gps != Gps::Parked)
        {
            return Err("parked for sleep with something still up".into());
        }
        if b.ble == Ble::Asleep {
            if !b.sleep_ready {
                return Err("asleep without the park having finished".into());
            }
            if b.posture.card != Card::Parked
                || b.posture.radio != Radio::Asleep
                || b.posture.gps != Gps::Parked
            {
                return Err("asleep with something still up".into());
            }
        }
        if b.sleep_ready && b.posture.card != Card::Parked {
            return Err("the park signaled done with the card not parked".into());
        }
        if b.tx && !(b.posture.live.transmits() && b.posture.radio_up()) {
            return Err("transmitting in a posture that must not".into());
        }
        if b.tx && b.posture.live == Mode::Listening {
            return Err("a listening node transmitted".into());
        }
        // Once the loop has caught up, the hardware is in the mode the
        // settings report - except while a store is on its way to the
        // sleep that carries it out.
        if b.requests.is_empty()
            && b.sleep_now.is_none()
            && matches!(b.ble, Ble::Advertising | Ble::Connected | Ble::Down)
            && b.posture.live != b.stored.mode
        {
            return Err(format!(
                "the loop is {:?} while the settings say {:?}",
                b.posture.live, b.stored.mode
            ));
        }
        if b.ble == Ble::Down && b.stored.mode != Mode::Tracking {
            return Err("the modem is down in a mode that has no off period".into());
        }
        Ok(())
    }

    fn check_step(&self, from: &Board, e: &Event, to: &Board) -> Result<(), String> {
        // A transmit never starts into a transfer or over a pending sleep.
        if *e == Event::TxStart
            && (from.transfer.is_some()
                || from.sleep_now.is_some()
                || from.requests.sleep_pending())
        {
            return Err("a transmit started over a transfer or a pending sleep".into());
        }
        // A request is never lost: what was pushed is drained by the next
        // pass, and the pass changes the posture or says why not.
        if *e == Event::LoopPass && !to.requests.is_empty() {
            return Err("a pass left a request behind".into());
        }
        Ok(())
    }

    fn describe(&self, b: &Board) -> String {
        format!(
            "ble {:?} | settings {:?} flags {:#x} | hw {:?} radio {:?} gps {:?} card {:?} | queue {} sleep_now {:?} ready {} transfer {:?} tx {}",
            b.ble,
            b.stored.mode,
            b.stored.flags,
            b.posture.live,
            b.posture.radio,
            b.posture.gps,
            b.posture.card,
            if b.requests.is_empty() { "empty" } else { "pending" },
            b.sleep_now,
            b.sleep_ready,
            b.transfer,
            b.tx
        )
    }

    fn lane(&self, e: &Event) -> &'static str {
        match e {
            Event::Connect
            | Event::ConnectFailed
            | Event::Expire
            | Event::Disconnect
            | Event::SessionEndsForSleep
            | Event::DownOver => "ble",
            Event::Write(Via::Ble, _) | Event::TransferBegin(Via::Ble) => "app",
            Event::Write(Via::Usb, _) | Event::TransferBegin(Via::Usb) => "usb",
            Event::TransferEnd(_) => "transfer",
            Event::LoopPass | Event::TxStart | Event::TxEnd => "hardware loop",
            Event::Sleep | Event::Wake => "power",
        }
    }
}

fn writes() -> [Write; 9] {
    [
        Write::Mode(Mode::Stored),
        Write::Mode(Mode::Idle),
        Write::Mode(Mode::Tracking),
        Write::Mode(Mode::Listening),
        Write::GpsSleep(true),
        Write::GpsSleep(false),
        Write::WioSleep(true),
        Write::WioSleep(false),
        Write::SleepNow,
    ]
}

/// The whole space, every invariant, every liveness property, and enough
/// coverage to know the invariants were not vacuous.
#[test]
fn every_reachable_board_state_is_sound() {
    let m = Firmware;
    let x = explore(&m);
    println!("firmware model: {x}");
    x.assert_ok(&m);

    // Coverage: the model reaches every posture and every BLE phase.
    for mode in [Mode::Stored, Mode::Idle, Mode::Tracking, Mode::Listening] {
        x.assert_some(|b| b.posture.live == mode, &format!("the hardware loop is {mode:?}"));
    }
    x.assert_some(|b| b.ble == Ble::Down, "the modem is down");
    x.assert_some(|b| b.ble == Ble::Asleep, "the chip is asleep");
    x.assert_some(
        |b| b.ble == Ble::Connected && b.posture.live == Mode::Stored,
        "a wake check is connected to before the promotion lands",
    );
    x.assert_some(
        |b| b.serve.mode() == Mode::Idle && b.stored.mode == Mode::Idle && b.posture.card == Card::Deferred,
        "a promoted wake check whose card is not up yet",
    );
    x.assert_some(
        |b| b.tx && b.ble == Ble::Connected && b.transfer.is_some(),
        "a transmit in flight while a transfer opens",
    );
    x.assert_some(
        |b| b.posture.radio == Radio::Standby && b.posture.gps == Gps::Awake,
        "the radio parked while the receiver runs",
    );
    x.assert_some(
        |b| b.ble == Ble::Parking && !b.requests.is_empty() && b.posture.card == Card::Parked,
        "a request arriving after the park",
    );

    // Liveness: nothing leaves the board dark for good, and nothing keeps
    // it from being put to work again.
    x.assert_always_reachable(&m, |b| b.ble == Ble::Advertising, "the board is advertising");
    x.assert_always_reachable(
        &m,
        |b| b.posture.live == Mode::Tracking && b.posture.radio_up() && b.requests.is_empty(),
        "the board is tracking with its radio up",
    );
    x.assert_always_reachable(&m, |b| b.ble == Ble::Asleep, "the board is asleep");
}

/// A request arriving between the park and the sleep is ignored rather
/// than acted on - the trace the rule exists for, checked explicitly.
#[test]
fn a_raise_after_the_park_is_ignored() {
    let m = Firmware;
    let x = explore(&m);
    x.assert_ok(&m);
    x.assert_some(
        |b| b.ignored_while_parked,
        "a pass that ignored a request because the board was parked",
    );
}
