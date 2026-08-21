//! The one bulk transfer, and what a completed one sets in motion.
//!
//! Both transports - a BLE write on the bulk characteristic and a framed
//! `BULK` command on the USB console - land here, which is what makes "one
//! transfer at a time" a fact about the object rather than a flag someone
//! has to remember to check. The protocol itself is
//! [`midair_proto::bulk`], host-tested; this is the effects half.
//!
//! A completed config is parsed here, because a parse failure is the one
//! outcome the host has to hear about in the ack - a board that answered OK
//! and then quietly kept its old settings is the failure mode the whole
//! transfer exists to avoid. Applying it is not done here: the radio, the
//! GPS and the card belong to the hardware loop, so the parsed config is
//! left for it to pick up, and what actually happened is reported on the
//! status line (which reaches both the console and the log characteristic).

use core::cell::RefCell;
use critical_section::Mutex as CsMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use gps_proto::packet;
use midair_proto::ble;
use midair_proto::bulk::{self, Ack, Event, NoFirmware, Owner, Transfer};
use midair_proto::radiocfg::{self, RadioConfig};

use crate::state::{self, Request};

static TRANSFER: Mutex<CriticalSectionRawMutex, Transfer> = Mutex::new(Transfer::new());

/// A verified, parsed config waiting for the hardware loop to adopt it.
///
/// The raw text travels with the parsed form because the card copy has to
/// be the bytes the user wrote, not a re-rendering of what they parsed to.
struct Pending {
    cfg: Option<RadioConfig>,
    raw: [u8; bulk::CONFIG_MAX],
    len: usize,
}

static PENDING: CsMutex<RefCell<Pending>> = CsMutex::new(RefCell::new(Pending {
    cfg: None,
    raw: [0; bulk::CONFIG_MAX],
    len: 0,
}));

/// Take the config a transfer left, copying its text into `raw_out`.
/// Returns the parsed config and how many bytes of text came with it.
pub fn take_pending(raw_out: &mut [u8]) -> Option<(RadioConfig, usize)> {
    critical_section::with(|cs| {
        let mut p = PENDING.borrow(cs).borrow_mut();
        let cfg = p.cfg.take()?;
        let len = p.len.min(raw_out.len());
        raw_out[..len].copy_from_slice(&p.raw[..len]);
        Some((cfg, len))
    })
}

fn set_pending(cfg: RadioConfig, raw: &[u8]) {
    critical_section::with(|cs| {
        let mut p = PENDING.borrow(cs).borrow_mut();
        let len = raw.len().min(bulk::CONFIG_MAX);
        p.raw[..len].copy_from_slice(&raw[..len]);
        p.len = len;
        p.cfg = Some(cfg);
    });
}

/// Process one bulk op from `owner` and return the ack to send back.
pub async fn handle(owner: Owner, now_ms: u64, data: &[u8]) -> Ack {
    let mut t = TRANSFER.lock().await;
    // The flash is only needed for a firmware image; a board without it
    // installed refuses that kind rather than failing partway through.
    let done = crate::flash::with_flash(|f| t.handle(owner, now_ms, data, &mut f.ota_sink())).await;
    let (event, ack) = match done {
        Some(r) => r,
        None => t.handle(owner, now_ms, data, &mut NoFirmware),
    };
    // A transfer owns the board while it runs: the console stays off the
    // shared USB FIFO, and the beacon stays off the air so a 22 dBm
    // transmit does not brown out the link carrying the update.
    state::set_transfer_active(t.is_active());
    match event {
        Event::None => ack,
        Event::Config => match radiocfg::parse_bytes(t.bytes()) {
            Ok(cfg) => {
                set_pending(cfg, t.bytes());
                // Only now does a repeated END read as the success it was.
                t.mark_applied();
                state::request(Request::ApplyConfig);
                ack
            }
            Err(e) => {
                crate::status_println!("config: rejected, {:?}", e);
                packet::encode_ack(ble::ACK_ID_BULK, packet::ACK_BAD_VALUE, &[])
            }
        },
        Event::Firmware => {
            crate::status_println!("ota: image installed, rebooting into it");
            state::request(Request::Reboot);
            ack
        }
    }
}

/// Drop a transfer this transport owns, e.g. because the connection
/// carrying it went away. Returns whether there was one.
pub async fn abort(owner: Owner, now_ms: u64) -> bool {
    let mut t = TRANSFER.lock().await;
    if !t.is_active() {
        return false;
    }
    // Going through the protocol rather than reaching into the transfer,
    // so the ownership check and the sink cancel are the same ones every
    // other abort takes.
    let done = crate::flash::with_flash(|f| {
        t.handle(owner, now_ms, &[ble::OP_ABORT], &mut f.ota_sink())
    })
    .await;
    if done.is_none() {
        t.handle(owner, now_ms, &[ble::OP_ABORT], &mut NoFirmware);
    }
    let still = t.is_active();
    state::set_transfer_active(still);
    !still
}

/// Drop a transfer whose host has gone quiet, so a cable pulled mid-upload
/// does not hold the board off the air until someone power-cycles it.
pub async fn expire(now_ms: u64) {
    let mut t = TRANSFER.lock().await;
    if !t.is_active() {
        return;
    }
    let expired = crate::flash::with_flash(|f| t.expire(now_ms, &mut f.ota_sink()))
        .await
        .unwrap_or_else(|| t.expire(now_ms, &mut NoFirmware));
    if expired {
        state::set_transfer_active(false);
        crate::status_println!("bulk: transfer timed out, abandoned");
    }
}
