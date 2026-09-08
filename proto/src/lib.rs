//! Shared definitions for the telemetry-in-midair board.
//!
//! Two consumers depend on this crate so the wire formats cannot drift: the
//! Wio-S3 firmware (`s3/`) and host tests (`cargo test` in this directory).
//!
//! - [`bulk`]: the transfer a radio config or a firmware image arrives in,
//!   over either BLE or the USB console.
//! - [`cfgstore`]: the radio config's copy in the board's own flash, which
//!   is what a board with no card comes back on.
//! - [`link`]: the frame format. It was the UART protocol between the
//!   ESP32-C6 and the WIO-E5 on the two-MCU board; one module has nothing
//!   to link to, so what survives is the framing the host tools speak over
//!   USB and the bulk transfer that carries a radio config.
//! - [`lora`]: the LoRa over-air frame and the payloads it carries.
//! - [`hop`]: frequency hopping - the channel plan, the slot clock the
//!   network keeps in step, and the sync word a hopped frame carries.
//! - [`ble`]: BLE GATT extensions on top of the gps-proto service (extra
//!   characteristic UUIDs and config command ids).
//! - [`radiocfg`]: the radio TOML configuration file format and its parser.
//! - [`roster`]: the latest report from each remote node, and the BLE values
//!   the firmware serves from it.
//! - [`session`]: what a BLE config write changes, and the sleep/advertise
//!   cycle the firmware runs between visits - the serve loop's policy.
//! - [`posture`]: what the hardware task raises and lowers for each
//!   request, and the request set that carries them to it.
//! - [`rxgate`]: what the receiver has seen of an arriving frame, and how
//!   long a hop or a transmit has to wait for it.
//! - [`dedup`]: the table of frames seen and the queue of frames waiting to
//!   be repeated, which is what keeps a repeater from becoming a storm.
//! - [`beacon`]: when the next beacon goes out - the planner the hardware
//!   loop runs once a pass.
//!
//! The last three, with `hop` and `roster`, are the machines the state
//! space tests in `tests/` walk exhaustively; `midair-explore` is the
//! harness.
//!
//! The BLE position/ack protocol itself lives in the shared `gps-proto`
//! crate (re-exported here) so the existing gps-gui-rs app keeps working.

#![cfg_attr(not(any(test, feature = "std")), no_std)]

pub use gps_proto;

/// Every wire constant the host tools speak, as one JSON document. What
/// `tools/wire_consts.json` holds, and what a test in this crate holds it
/// to; see the `wire_consts` example.
#[cfg(any(test, feature = "std"))]
pub fn wire_consts_json() -> String {
    use crate::session::KNOBS;
    use gps_proto::packet;
    let mut knobs = Vec::new();
    for spec in &KNOBS {
        knobs.push(format!(
            "    {{\"name\": \"{}\", \"id\": {}, \"min\": {}, \"max\": {}, \"zero_is_off\": {}}}",
            spec.name,
            spec.id,
            spec.min,
            spec.max,
            matches!(spec.zero, session::Zero::Off)
        ));
    }
    format!(
        "{{\n\
  \"link\": {{\"sync\": {}, \"max_payload\": {}, \"ack\": {}}},\n\
  \"usb\": {{\"ping\": {}, \"bulk\": {}, \"bulk_ack\": {}, \"info\": {}, \"sleep\": {}, \"cfg\": {}}},\n\
  \"bulk\": {{\"begin\": {}, \"data\": {}, \"end\": {}, \"abort\": {}, \"kind_toml\": {}, \"kind_ota\": {}, \"data_max\": {}, \"config_max\": {}, \"ack_id\": {}}},\n\
  \"ack\": {{\"ok\": {}, \"unknown_id\": {}, \"bad_value\": {}, \"board_error\": {}, \"bad_state\": {}}},\n\
  \"cfg\": {{\"notify_interval_ms\": {}, \"radio_standby\": {}, \"gps_sleep\": {}, \"sleep_now\": {}, \"mode\": {}, \"name\": {}}},\n\
  \"knobs\": [\n{}\n  ]\n\
}}\n",
        link::SYNC,
        link::MAX_PAYLOAD,
        link::resp::ACK,
        link::usb::PING,
        link::usb::BULK,
        link::usb::BULK_ACK,
        link::usb::INFO,
        link::usb::SLEEP,
        link::usb::CFG,
        ble::OP_BEGIN,
        ble::OP_DATA,
        ble::OP_END,
        ble::OP_ABORT,
        ble::KIND_TOML,
        ble::KIND_OTA,
        ble::BULK_DATA_MAX,
        bulk::CONFIG_MAX,
        ble::ACK_ID_BULK,
        packet::ACK_OK,
        packet::ACK_UNKNOWN_ID,
        packet::ACK_BAD_VALUE,
        ble::ACK_BOARD_ERROR,
        ble::ACK_BAD_STATE,
        packet::CFG_UPDATE_INTERVAL_MS,
        ble::CFG_RADIO_STANDBY,
        ble::CFG_GPS_SLEEP,
        ble::CFG_SLEEP_NOW,
        ble::CFG_MODE,
        ble::CFG_NAME,
        knobs.join(",\n")
    )
}

pub mod beacon;
pub mod ble;
pub mod bulk;
pub mod cfgstore;
pub mod dedup;
pub mod geo;
pub mod hop;
pub mod link;
pub mod lora;
pub mod posture;
pub mod radiocfg;
pub mod roster;
pub mod rxgate;
pub mod session;

#[cfg(test)]
mod tests {
    /// `tools/wire_consts.json` is what the host tools read their wire
    /// constants from, and it is this crate's output. Regenerate with
    /// `cargo run --example wire_consts --features std >
    /// ../tools/wire_consts.json`.
    #[test]
    fn the_tools_wire_constants_are_current() {
        let shipped = include_str!("../../tools/wire_consts.json");
        assert!(
            shipped == super::wire_consts_json(),
            "tools/wire_consts.json is out of date - regenerate it with the wire_consts example"
        );
    }
}
