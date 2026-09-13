//! The snapshot the hardware loop publishes and the BLE session reads.
//!
//! On the two-MCU board this data arrived over the UART link and the ESP
//! cached it, so a freshly connected app got the last known values instead
//! of waiting for the next report. One MCU keeps the cache for the same
//! reason - a central can connect between GPS epochs - but it is now a
//! plain shared cell rather than the far end of a framed protocol.
//!
//! Deliberately a snapshot and not a channel: every consumer wants the
//! *latest* position, never a backlog, and dropping intermediate values is
//! the correct behavior rather than a lossy compromise. The two exceptions
//! are the remote-node roster and the log line, and both are exceptions for
//! the same reason - a report from a node, or a status line, is news
//! exactly once, so losing one to a tick boundary loses the event itself.

use core::cell::{Cell, RefCell};
use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use midair_proto::session::ServeCommand;
use portable_atomic::{AtomicBool, Ordering};
use gps_proto::packet::PositionPacket;
use midair_proto::link::{Telemetry, LOG_MAX};
use midair_proto::radiocfg::RADIO_CONFIG_LEN;
use midair_proto::roster::{Report, Roster, Value};

struct Shared {
    /// `None` until the GPS has produced something, which is a real state:
    /// a central can connect before the first epoch.
    position: Cell<Option<PositionPacket>>,
    telemetry: Cell<Option<Telemetry>>,
    /// Set when the position changed since the BLE session last looked.
    position_dirty: Cell<bool>,
    /// Whether the radio is mid-transmission.
    ///
    /// On the old board this was a `RADIO_BUSY` link message, so the ESP
    /// could hold BLE notifications while the WIO had the air. Same chip,
    /// same reason - a LoRa transmit at 22 dBm next to a 2.4 GHz radio is
    /// a supply problem, not a protocol one - but it costs a bool instead
    /// of two frames.
    radio_busy: Cell<bool>,
    /// Position notify interval, which a config write can change. Session
    /// state, so unlike the sleep settings it resets with the board.
    notify_interval_ms: Cell<u32>,
    /// Whether the running config asks for verbose console logging.
    verbose: Cell<bool>,
    /// Whether a bulk transfer owns the board right now.
    transfer_active: Cell<bool>,
    /// The running radio config as its read-back blob, and whether one has
    /// been published yet - the all-zero placeholder decodes to `None`, so
    /// an app must not be handed it as if it were a config.
    radio_config: Cell<[u8; RADIO_CONFIG_LEN]>,
    radio_config_known: Cell<bool>,
    /// The latest report from each remote node heard over LoRa.
    roster: RefCell<Roster>,
    /// The board's BLE address, LSB first, resolved once at boot. Published
    /// here because the USB info query answers from a different task than
    /// the one that worked it out.
    ble_address: Cell<[u8; 6]>,
    /// Longest a single LoRa transmit can hold the hardware loop, from the
    /// running config. Published so the sleep path can wait out a beacon
    /// already in flight instead of guessing a constant that the slowest
    /// settings the config accepts would be forty times too small for.
    tx_worst_case_ms: Cell<u32>,
}

// SAFETY-adjacent note: every field is only ever touched inside
// `critical_section::with`, which is what makes the cells sound to share.
unsafe impl Sync for Shared {}

static SHARED: Mutex<Shared> = Mutex::new(Shared {
    position: Cell::new(None),
    telemetry: Cell::new(None),
    position_dirty: Cell::new(false),
    radio_busy: Cell::new(false),
    notify_interval_ms: Cell::new(gps_proto::packet::UPDATE_INTERVAL_DEFAULT_MS),
    // Talkative until a config says otherwise: the console costs nothing
    // when nothing is attached, and the window before the card is read is
    // exactly when a board that fails to come up needs to be saying
    // something.
    verbose: Cell::new(true),
    transfer_active: Cell::new(false),
    radio_config: Cell::new([0; RADIO_CONFIG_LEN]),
    radio_config_known: Cell::new(false),
    roster: RefCell::new(Roster::new()),
    ble_address: Cell::new([0; 6]),
    // The default config's SF12/BW500 beacon, until one is adopted.
    tx_worst_case_ms: Cell::new(1_000),
});

pub fn set_position(p: PositionPacket) {
    critical_section::with(|cs| {
        let s = SHARED.borrow(cs);
        s.position.set(Some(p));
        s.position_dirty.set(true);
    });
}

pub fn position() -> Option<PositionPacket> {
    critical_section::with(|cs| SHARED.borrow(cs).position.get())
}

/// The position, and whether it changed since the last call.
pub fn take_position() -> (Option<PositionPacket>, bool) {
    critical_section::with(|cs| {
        let s = SHARED.borrow(cs);
        (s.position.get(), s.position_dirty.replace(false))
    })
}

pub fn set_telemetry(t: Telemetry) {
    critical_section::with(|cs| SHARED.borrow(cs).telemetry.set(Some(t)));
}

pub fn telemetry() -> Option<Telemetry> {
    critical_section::with(|cs| SHARED.borrow(cs).telemetry.get())
}

pub fn set_radio_busy(busy: bool) {
    critical_section::with(|cs| SHARED.borrow(cs).radio_busy.set(busy));
}

pub fn radio_busy() -> bool {
    critical_section::with(|cs| SHARED.borrow(cs).radio_busy.get())
}

pub fn set_notify_interval_ms(ms: u32) {
    critical_section::with(|cs| SHARED.borrow(cs).notify_interval_ms.set(ms));
}

pub fn notify_interval_ms() -> u32 {
    critical_section::with(|cs| SHARED.borrow(cs).notify_interval_ms.get())
}

pub fn set_verbose(on: bool) {
    critical_section::with(|cs| SHARED.borrow(cs).verbose.set(on));
}

pub fn verbose() -> bool {
    critical_section::with(|cs| SHARED.borrow(cs).verbose.get())
}

/// Mark a bulk transfer as owning the board, which two things read.
///
/// The console goes quiet: esp-println and the USB reply frames share the
/// single USB Serial/JTAG IN FIFO with no arbitration between them, so
/// console text emitted from another task lands in the middle of a
/// transfer's ack frames and costs the host a retry per collision.
///
/// The beacon holds off: a 22 dBm LoRa transmit beside a 2.4 GHz radio is
/// a supply problem, and dropping the link mid-update is how a firmware
/// upload turns into a half-written slot.
pub fn set_transfer_active(active: bool) {
    critical_section::with(|cs| SHARED.borrow(cs).transfer_active.set(active));
}

pub fn transfer_active() -> bool {
    critical_section::with(|cs| SHARED.borrow(cs).transfer_active.get())
}

/// Publish the board's BLE address. Called once, before any task can be
/// asked for it.
pub fn set_ble_address(a: [u8; 6]) {
    critical_section::with(|cs| SHARED.borrow(cs).ble_address.set(a));
}

/// The board's BLE address, LSB first.
pub fn ble_address() -> [u8; 6] {
    critical_section::with(|cs| SHARED.borrow(cs).ble_address.get())
}

/// Publish the radio config an app can read back.
pub fn set_radio_config(blob: [u8; RADIO_CONFIG_LEN]) {
    critical_section::with(|cs| {
        let s = SHARED.borrow(cs);
        s.radio_config.set(blob);
        s.radio_config_known.set(true);
    });
    RADIO_CONFIG_SIGNAL.signal(());
}

/// The published radio config, or `None` before the radio is configured.
pub fn radio_config() -> Option<[u8; RADIO_CONFIG_LEN]> {
    critical_section::with(|cs| {
        let s = SHARED.borrow(cs);
        s.radio_config_known.get().then(|| s.radio_config.get())
    })
}

/// Raised whenever a new radio config is published, so a connected session
/// can republish it without polling.
pub static RADIO_CONFIG_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// ---------------------------------------------------------------------------
// Remote nodes
// ---------------------------------------------------------------------------

/// Raised when a node's report arrives (or the roster is replayed), so the
/// BLE session pushes it straight out instead of waiting for a tick.
pub static REMOTE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Record a report from a remote node.
pub fn record_remote(now_ms: u64, report: Report) {
    critical_section::with(|cs| SHARED.borrow(cs).roster.borrow_mut().record(now_ms, report));
    REMOTE_SIGNAL.signal(());
}

/// The next report waiting to go out, age stamped in. `None` once every
/// node's current report has been handed over.
pub fn take_remote(now_ms: u64) -> Option<Value> {
    critical_section::with(|cs| SHARED.borrow(cs).roster.borrow_mut().take_dirty(now_ms))
}

/// Re-arm every node still inside the roster's TTL, so a central that has
/// just connected receives the whole roster rather than only the next node
/// to report.
pub fn replay_remotes(now_ms: u64) {
    critical_section::with(|cs| SHARED.borrow(cs).roster.borrow_mut().replay(now_ms));
    REMOTE_SIGNAL.signal(());
}

/// How many remote nodes are currently remembered.
pub fn remote_count() -> usize {
    critical_section::with(|cs| SHARED.borrow(cs).roster.borrow().len())
}

// ---------------------------------------------------------------------------
// Status lines
// ---------------------------------------------------------------------------

/// Status lines waiting to reach a connected central.
///
/// Bounded and non-blocking: a board with nobody connected must not stall
/// the hardware loop on a channel nothing is draining, so the oldest line
/// is dropped instead.
pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, LOG_MAX>, 8> =
    Channel::new();

/// Queue one status line for the log characteristic. Called by
/// [`status_println!`](crate::status_println), which also prints it.
pub fn log_line(args: core::fmt::Arguments<'_>) {
    use core::fmt::Write as _;
    struct Sink(heapless::Vec<u8, LOG_MAX>);
    impl core::fmt::Write for Sink {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            // Truncate rather than fail: the head of a long line is worth
            // more to a reader than nothing at all.
            let room = self.0.capacity() - self.0.len();
            let take = s.len().min(room);
            let _ = self.0.extend_from_slice(&s.as_bytes()[..take]);
            Ok(())
        }
    }
    let mut sink = Sink(heapless::Vec::new());
    let _ = write!(sink, "{}", args);
    if sink.0.is_empty() {
        return;
    }
    // Full means nobody is draining; drop the oldest so the newest, which
    // is what an app connecting now wants, still gets in.
    if LOG_CHANNEL.try_send(sink.0.clone()).is_err() {
        let _ = LOG_CHANNEL.try_receive();
        let _ = LOG_CHANNEL.try_send(sink.0);
    }
}

/// Drop whatever is buffered, so a central that has just connected sees
/// live events rather than a stale backlog.
pub fn drain_log() {
    while LOG_CHANNEL.try_receive().is_ok() {}
}

// ---------------------------------------------------------------------------
// Requests from the BLE session to the hardware loop
// ---------------------------------------------------------------------------

/// A request from the BLE session or the host tools to the hardware loop.
///
/// The two-MCU build sent these over the link and waited for an ack. Same
/// chip, so this is a queue the loop drains on its next pass; the
/// characteristic is acked immediately because there is no longer a peer
/// that can fail to answer. What each one changes, and the rule the
/// result has to satisfy, is [`midair_proto::posture::Posture`], which the
/// state space tests walk exhaustively.
pub use midair_proto::posture::Request;

// ---------------------------------------------------------------------------
// Commands for the serve loop
// ---------------------------------------------------------------------------

/// Commands for the serve loop from a config write on either transport:
/// a nap, or a moved mode. One channel, one consumer - whichever wait the
/// loop is in takes the next one.
///
/// This replaced a single-slot signal per command, a cell beside one of
/// them and four helpers keeping the pair coherent. A queue of four is
/// plenty: the loop drains it inside a millisecond of any wait ending,
/// and a queue that is full means the loop is inside a deep sleep's park
/// and about to lose its RAM anyway.
static COMMANDS: Channel<CriticalSectionRawMutex, ServeCommand, 4> = Channel::new();

/// A sleep has been asked for and the chip has not gone down. Read by
/// the hardware loop, which declines to start a beacon it would then make
/// the sleep wait out - at the slowest settings the config accepts a
/// transmit runs to nearly ten seconds.
static SLEEP_ASKED: AtomicBool = AtomicBool::new(false);

/// Send the serve loop a command.
pub fn command(c: ServeCommand) {
    if let ServeCommand::SleepNow(_) = c {
        SLEEP_ASKED.store(true, Ordering::Relaxed);
    }
    if COMMANDS.try_send(c).is_err() {
        crate::qprintln!("serve: command queue full, {:?} dropped", c);
        crate::evlog::note(
            midair_proto::evlog::Kind::Note,
            format_args!("serve: command queue full, {:?} dropped", c),
        );
    }
}

/// The next command, when there is one.
pub async fn next_command() -> ServeCommand {
    COMMANDS.receive().await
}

/// Whether a sleep has been asked for and not yet entered. Never cleared:
/// every ask ends in a deep sleep, which is a reset, and a beacon that
/// started in between would only make the sleep wait it out.
pub fn sleep_asked() -> bool {
    SLEEP_ASKED.load(Ordering::Relaxed)
}

/// The node the compass should point at: `(src, position, age_s, rssi)`.
///
/// Read straight from the roster rather than cached, because the roster is
/// already the thing that knows which node was heard most recently and
/// duplicating that here would be a second answer to the same question.
pub fn compass_target(
    now_ms: u64,
) -> Option<(u8, gps_proto::packet::PositionPacket, u16, i16)> {
    critical_section::with(|cs| {
        let roster = SHARED.borrow(cs).roster.borrow();
        let (src, bytes, age_s, rssi) = roster.newest_position(now_ms)?;
        let packet = gps_proto::packet::PositionPacket::decode(&bytes)?;
        Some((src, packet, age_s, rssi))
    })
}

/// Publish the running config's transmit deadline, in ms.
pub fn set_tx_worst_case_ms(ms: u32) {
    critical_section::with(|cs| SHARED.borrow(cs).tx_worst_case_ms.set(ms));
}

/// The longest a beacon already in flight can keep the hardware loop from
/// answering a request.
pub fn tx_worst_case_ms() -> u32 {
    critical_section::with(|cs| SHARED.borrow(cs).tx_worst_case_ms.get())
}

/// Raised once the hardware loop has parked the radio for a deep sleep.
///
/// There is no rail to cut on this board, so the radio is the one load the
/// firmware can drop before sleeping - and the hardware loop owns it, so
/// the sleep path has to ask rather than reach for it.
pub static SLEEP_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The requests waiting for the hardware loop.
///
/// A set with one slot per kind rather than a channel: the channel this
/// replaced held four and dropped the fifth, and the fifth could be the
/// park before a deep sleep - which then happened over a radio still in
/// continuous receive. A newer request of a kind replaces an older one
/// still waiting, and [`Requests::take`] drains them in the order that is
/// correct whoever asked first: a mode before the overrides that sit on
/// top of it, a park after everything that raises.
static REQUESTS: Mutex<RefCell<midair_proto::posture::Requests>> =
    Mutex::new(RefCell::new(midair_proto::posture::Requests::new()));

pub fn request(r: Request) {
    critical_section::with(|cs| REQUESTS.borrow(cs).borrow_mut().push(r));
}

pub fn take_request() -> Option<Request> {
    critical_section::with(|cs| REQUESTS.borrow(cs).borrow_mut().take())
}

/// Whether a deep sleep is on its way: asked for and not yet acted on, or
/// acted on and waiting for the hardware loop to park. Either way a
/// transmit started now is one the sleep would have to wait out.
pub fn park_pending() -> bool {
    sleep_asked() || critical_section::with(|cs| REQUESTS.borrow(cs).borrow().sleep_pending())
}
