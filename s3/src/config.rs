//! One place a settings write is applied, whichever transport carried it.
//!
//! The decision - what is legal, what it clamps to, what the ack says - is
//! [`midair_proto::session::apply`], the same host-tested policy the C6
//! runs. What changed is the far side: an action that used to be a link
//! frame and a wait for the WIO's answer is now a signal to the hardware
//! loop, so the ack the policy built always holds.
//!
//! Both the BLE config characteristic and the USB console's `CFG` command
//! land here, which is what makes a setting pushed from a bench and one
//! pushed from the app the same operation rather than two that have to be
//! kept in step.

use gps_proto::packet;
use midair_proto::session::{apply as apply_write, Action};

use crate::state::{self, Request};
use crate::settings;

/// Apply one settings write and return the ack it produced.
pub async fn apply_config(data: &[u8]) -> ([u8; packet::ACK_MAX_LEN], usize) {
    let mut stored = settings::get();
    let outcome = apply_write(&mut stored, data);
    settings::set(stored);
    if outcome.save {
        settings::save().await;
    }
    match outcome.action {
        Action::GpsSleep(on) => state::request(Request::GpsSleep(on)),
        // No second MCU to put to sleep; the nearest thing is parking the
        // radio, which is what the WIO's soft sleep actually bought.
        Action::WioSleep(on) => state::request(Request::RadioStandby(on)),
        // No host-controlled rail on this board - the GPS and SD sit
        // directly on +3V3. See BOARD-REVIEW.md in the board repo.
        Action::Rail(_) => qprintln!("config: rail control has no hardware here"),
        Action::NotifyInterval(ms) => {
            state::set_notify_interval_ms(ms);
            qprintln!("config: notify interval set to {} ms", ms);
        }
        Action::SleepInterval(secs) => {
            qprintln!("config: sleep interval {} s", secs);
        }
        // Handed to the serve loop rather than acted on here: this function
        // runs inside the GATT session, and the ack it is building has not
        // been sent yet. The loop that owns the `Rtc` sleeps once the
        // session has finished paying out what it owes the central.
        Action::SleepNow(secs) => {
            status_println!("config: sleep now for {} s", secs);
            state::request_sleep_now(secs);
        }
        Action::AdvWindow(secs) => {
            qprintln!("config: advertising window {} s", secs);
        }
        // Read by the duty-cycle loop in `main` at the end of the current
        // window, so a central that sets this keeps the connection it set
        // it over.
        Action::BleOff(secs) => match secs {
            0 => status_println!("config: BLE stays up between windows"),
            s => status_println!("config: BLE down {} s between windows", s),
        },
        Action::None => {
            qprintln!("config: rejected write (status {})", outcome.ack[1]);
        }
    }
    (outcome.ack, outcome.ack_len)
}
