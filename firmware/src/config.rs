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
use midair_proto::session::{apply as apply_write, dispatch, Action, ServeCommand, Zero};

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
    // been sent yet. A nap is entered once the session has finished paying
    // out what it owes the central; a moved mode is re-budgeted on the
    // loop's next pass. Sent after the request is queued, so a loop that
    // wakes on it finds the hardware half already on its way.
    if let Some(c) = d.command {
        state::command(c);
    }
    match outcome.action {
        Action::GpsSleep(on) => qprintln!("config: gps {}", if on { "backup" } else { "wake" }),
        Action::RadioStandby(on) => qprintln!("config: radio {}", if on { "standby" } else { "up" }),
        Action::NotifyInterval(ms) => qprintln!("config: notify interval set to {} ms", ms),
        Action::SleepNow(secs) => status_println!("config: sleep now for {} s", secs),
        // A duration, read by whichever loop owns it when it next decides:
        // a window at the next window, an off period at the end of the
        // one running, so a central that sets it keeps its connection.
        Action::Knob(knob, secs) => {
            let spec = knob.spec();
            match (spec.zero, secs) {
                (Zero::Off, 0) => status_println!("config: {} off", spec.name),
                _ => status_println!("config: {} {} s", spec.name, secs),
            }
        }
        // A store is a command that ends with the chip gone, so it goes to
        // the serve loop by the same route a nap does.
        Action::SetMode(Mode::Stored) => status_println!(
            "mode: stored, sleeping {} s per wake check",
            match d.command {
                Some(ServeCommand::SleepNow(s)) => s,
                _ => 0,
            }
        ),
        Action::SetMode(other) => status_println!("mode: {}", other.as_str()),
        // Nothing to drive: the serve loop rebuilds the scan response from
        // the stored name before every advertisement, so the new name goes
        // out with the next window - the one on the air now was handed to
        // the controller before this write arrived. The characteristic is
        // republished by the caller, which still holds the connection.
        Action::Name => status_println!("name: {}", settings::name()),
        // Handed to the hardware loop, which owns the radio, runs the
        // burst from its own pass and says so on the console - from there
        // rather than here, because a line printed ahead of the ack is one
        // the tool reading for that ack discards.
        Action::WakeNode { target, idle } => state::request_wake(target, idle),
        Action::None => qprintln!("config: rejected write (status {})", outcome.ack[1]),
    }
    (outcome.ack, outcome.ack_len)
}
