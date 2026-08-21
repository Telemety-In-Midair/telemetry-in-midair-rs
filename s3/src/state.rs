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

/// A request from the BLE session to the hardware loop.
///
/// The two-MCU build sent these over the link and waited for an ack. Same
/// chip, so this is a signal the loop picks up on its next pass; the
/// characteristic is acked immediately because there is no longer a peer
/// that can fail to answer.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Request {
    GpsSleep(bool),
    RadioStandby(bool),
    /// A new radio config was pushed over BLE or USB and verified. The
    /// hardware loop owns the radio, the GPS and the card, so it is what
    /// re-inits them and writes the file back.
    ApplyConfig,
    /// A firmware image landed in the inactive OTA slot. Reboot into it.
    Reboot,
    /// The board is about to deep sleep. Park what a sleeping board cannot
    /// use and raise [`SLEEP_READY`].
    PrepareSleep,
}

/// Raised once the hardware loop has parked the radio for a deep sleep.
///
/// There is no rail to cut on this board, so the radio is the one load the
/// firmware can drop before sleeping - and the hardware loop owns it, so
/// the sleep path has to ask rather than reach for it.
pub static SLEEP_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Requests are a short queue rather than a signal.
///
/// A `Signal` holds one value, so a config push arriving while a GPS sleep
/// request is still waiting would silently replace it. Four is ample: the
/// producers are a BLE session and a USB task, each answering one write at
/// a time, against a loop that drains the queue every 10 ms.
static REQUESTS: Channel<CriticalSectionRawMutex, Request, 4> = Channel::new();

pub fn request(r: Request) {
    // Dropping a request is better than blocking the GATT handler, and a
    // full queue means the hardware loop is wedged, which nothing here can
    // fix anyway.
    let _ = REQUESTS.try_send(r);
}

pub fn take_request() -> Option<Request> {
    REQUESTS.try_receive().ok()
}
