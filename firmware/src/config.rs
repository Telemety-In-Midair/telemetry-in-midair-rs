//! One place a settings write is applied, whichever transport carried it.
//!
//! The decision - what is legal, what it clamps to, what the ack says - is
//! [`midair_proto::session::apply`], the same host-tested policy the C6
//! ran, and what the result sets in motion is
//! [`midair_proto::session::dispatch`], host-tested too. What is left here
//! is carrying it out: storing the settings, queueing the request, raising
//! the signal, and the console line for each.
//!
//! Both the BLE config characteristic and the USB console's `CFG` command
//! land here, which is what makes a setting pushed from a bench and one
//! pushed from the app the same operation rather than two that have to be
//! kept in step.

use gps_proto::packet;
use midair_proto::ble::Mode;
use midair_proto::session::{apply as apply_write, dispatch, Action};

use crate::settings;
use crate::state;

/// Apply one settings write and return the ack it produced.
pub async fn apply_config(data: &[u8]) -> ([u8; packet::ACK_MAX_LEN], usize) {
    let mut stored = settings::get();
    let outcome = apply_write(&mut stored, data);
    settings::set(stored);
    if outcome.save {
        settings::save().await;
    }
    let d = dispatch(outcome.action, &stored);
    if let Some(r) = d.request {
        state::request(r);
    }
    if let Some(ms) = d.notify_interval_ms {
        state::set_notify_interval_ms(ms);
    }
    // Handed to the serve loop rather than acted on here: this function
    // runs inside the GATT session, and the ack it is building has not
    // been sent yet. The loop that owns the `Rtc` sleeps once the session
    // has finished paying out what it owes the central.
    if let Some(secs) = d.sleep_now {
        state::request_sleep_now(secs);
    }
    // Raised after the request is queued, so a serve loop that wakes on it
    // and re-budgets finds the hardware half already on its way.
    if d.mode_signal {
        state::MODE_SIGNAL.signal(());
    }
    match outcome.action {
        Action::GpsSleep(on) => qprintln!("config: gps {}", if on { "backup" } else { "wake" }),
        // No second MCU to put to sleep; the nearest thing is parking the
        // radio, which is what the WIO's soft sleep actually bought.
        Action::WioSleep(on) => qprintln!("config: radio {}", if on { "standby" } else { "up" }),
        // No host-controlled rail on this board - the GPS and SD sit
        // directly on +3V3. See BOARD-REVIEW.md in the board repo.
        Action::Rail(_) => qprintln!("config: rail control has no hardware here"),
        Action::NotifyInterval(ms) => qprintln!("config: notify interval set to {} ms", ms),
        Action::SleepInterval(secs) => qprintln!("config: sleep interval {} s", secs),
        Action::SleepNow(secs) => status_println!("config: sleep now for {} s", secs),
        Action::AdvWindow(secs) => qprintln!("config: advertising window {} s", secs),
        // Like the window: sampled when a window starts, so this one lands
        // at the next.
        Action::BleOn(secs) => qprintln!("config: BLE up {} s between off periods", secs),
        // Read by the duty-cycle loop in `main` at the end of the current
        // window, so a central that sets this keeps the connection it set
        // it over.
        Action::BleOff(secs) => match secs {
            0 => status_println!("config: BLE stays up between windows"),
            s => status_println!("config: BLE down {} s between windows", s),
        },
        // A store is a command that ends with the chip gone, so it goes to
        // the serve loop by the same route a nap does.
        Action::SetMode(Mode::Stored) => status_println!(
            "mode: stored, sleeping {} s per wake check",
            d.sleep_now.unwrap_or(0)
        ),
        Action::SetMode(other) => status_println!("mode: {}", other.as_str()),
        Action::IdleTimeout(secs) => match secs {
            0 => qprintln!("config: idle never stores itself"),
            s => qprintln!("config: idle timeout {} s", s),
        },
        // Nothing to drive: the serve loop rebuilds the scan response from
        // the stored name before every advertisement, so the new name goes
        // out with the next window - the one on the air now was handed to
        // the controller before this write arrived. The characteristic is
        // republished by the caller, which still holds the connection.
        Action::Name => status_println!("name: {}", settings::name()),
        Action::None => qprintln!("config: rejected write (status {})", outcome.ack[1]),
    }
    (outcome.ack, outcome.ack_len)
}
