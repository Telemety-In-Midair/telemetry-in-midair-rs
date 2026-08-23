//! Shared definitions for the telemetry-in-midair board.
//!
//! Two consumers depend on this crate so the wire formats cannot drift: the
//! Wio-S3 firmware (`s3/`) and host tests (`cargo test` in this directory).
//!
//! - [`bulk`]: the transfer a radio config or a firmware image arrives in,
//!   over either BLE or the USB console.
//! - [`link`]: the frame format. It was the UART protocol between the
//!   ESP32-C6 and the WIO-E5 on the two-MCU board; one module has nothing
//!   to link to, so what survives is the framing the host tools speak over
//!   USB and the bulk transfer that carries a radio config.
//! - [`lora`]: the LoRa over-air frame and the payloads it carries.
//! - [`ble`]: BLE GATT extensions on top of the gps-proto service (extra
//!   characteristic UUIDs and config command ids).
//! - [`radiocfg`]: the radio TOML configuration file format and its parser.
//! - [`roster`]: the latest report from each remote node, and the BLE values
//!   the firmware serves from it.
//! - [`session`]: what a BLE config write changes, and the sleep/advertise
//!   cycle the firmware runs between visits.
//!
//! The BLE position/ack protocol itself lives in the shared `gps-proto`
//! crate (re-exported here) so the existing gps-gui-rs app keeps working.

#![cfg_attr(not(test), no_std)]

pub use gps_proto;

pub mod ble;
pub mod bulk;
pub mod geo;
pub mod link;
pub mod lora;
pub mod radiocfg;
pub mod roster;
pub mod session;
