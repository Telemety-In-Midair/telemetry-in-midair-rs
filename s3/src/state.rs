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
//! the correct behavior rather than a lossy compromise.

use critical_section::Mutex;
use core::cell::Cell;
use gps_proto::packet::PositionPacket;
use midair_proto::link::Telemetry;

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
}

// SAFETY-adjacent note: every field is only ever touched inside
// `critical_section::with`, which is what makes the `Cell`s sound to share.
unsafe impl Sync for Shared {}

static SHARED: Mutex<Shared> = Mutex::new(Shared {
    position: Cell::new(None),
    telemetry: Cell::new(None),
    position_dirty: Cell::new(false),
    radio_busy: Cell::new(false),
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
}

static REQUEST: embassy_sync::signal::Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Request,
> = embassy_sync::signal::Signal::new();

pub fn request(r: Request) {
    REQUEST.signal(r);
}

pub fn take_request() -> Option<Request> {
    REQUEST.try_take()
}
