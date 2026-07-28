//! BLE GATT extensions served by the ESP32-C6.
//!
//! The board reuses the gps-proto service and its position/config/ack
//! characteristics, so the existing gps-gui-rs app connects and streams
//! positions unchanged. This module adds board-specific characteristics
//! (continuing the same UUID sequence) and new config command ids on the
//! existing config characteristic.

/// Name the ESP32-C6 advertises under. The service UUID (which the app
/// filters scans by) stays `gps_proto::packet::SERVICE_UUID`.
pub const DEVICE_NAME: &str = "GPS-C6";

/// [`crate::link::Telemetry`] wire format, notify + read.
pub const TELEMETRY_UUID: &str = "c3a10005-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// Bulk transfer (radio TOML config / WIO firmware), write.
pub const BULK_UUID: &str = "c3a10006-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// Remote position: `[src u8, rssi i16le, PositionPacket 20B, age_s u16le]`,
/// notify + read. One notification per report received, not per tick.
pub const REMOTE_UUID: &str = "c3a10007-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// WIO status/log line (ASCII), notify + read. Carries the latest
/// [`crate::link::msg::LOG`] text, up to [`crate::link::LOG_MAX`] bytes.
pub const LOG_UUID: &str = "c3a10008-9f6e-4b2c-8f5a-2e32c3b1e5d0";

pub const TELEMETRY_UUID_U128: u128 = 0xc3a10005_9f6e_4b2c_8f5a_2e32c3b1e5d0;
pub const BULK_UUID_U128: u128 = 0xc3a10006_9f6e_4b2c_8f5a_2e32c3b1e5d0;
pub const REMOTE_UUID_U128: u128 = 0xc3a10007_9f6e_4b2c_8f5a_2e32c3b1e5d0;
pub const LOG_UUID_U128: u128 = 0xc3a10008_9f6e_4b2c_8f5a_2e32c3b1e5d0;

/// Smallest remote position value: src + rssi + packet.
///
/// This is the length a reader must accept, not the length the board sends
/// (see [`REMOTE_LEN_V2`]). It is deliberately unchanged from the layout
/// that had no age field, so a reader built against a newer version of this
/// crate still understands a board running older firmware.
pub const REMOTE_LEN: usize = 1 + 2 + gps_proto::packet::POSITION_PACKET_LEN;

/// Remote position value as the board sends it: [`REMOTE_LEN`] followed by
/// the age field.
///
/// The extra bytes are appended rather than inserted, so a reader that only
/// knows the shorter layout reads the same fields at the same offsets and
/// ignores the tail (`PositionPacket::decode` tolerates trailing bytes).
pub const REMOTE_LEN_V2: usize = REMOTE_LEN + 2;

/// Offset of the `age_s` field in a [`REMOTE_LEN_V2`] value.
pub const REMOTE_AGE_OFF: usize = REMOTE_LEN;

/// Remote node ping: `[src u8, rssi i16le, flags u8, uptime_s u16le,
/// age_s u16le]`, notify + read.
///
/// What a node reports in place of a position while it has no fix (see
/// [`crate::lora::Ping`]). Carried separately from [`REMOTE_UUID`] because a
/// ping is not a position: folding one into a position blob would mean
/// inventing coordinates for a node that explicitly has none.
///
/// `flags` is the [`crate::lora::PING_FLAG_GPS_PRESENT`] /
/// [`crate::lora::PING_FLAG_HAD_FIX`] byte as it travelled on the air.
pub const NODE_PING_UUID: &str = "c3a1000b-9f6e-4b2c-8f5a-2e32c3b1e5d0";
pub const NODE_PING_UUID_U128: u128 = 0xc3a1000b_9f6e_4b2c_8f5a_2e32c3b1e5d0;

/// Node ping value length: the [`crate::link::msg::PING`] payload plus the
/// same age field a remote position carries.
pub const NODE_PING_LEN: usize = crate::link::PING_LEN + 2;

/// Offset of the `age_s` field in a [`NODE_PING_LEN`] value.
pub const NODE_PING_AGE_OFF: usize = crate::link::PING_LEN;

/// How the `age_s` field on [`REMOTE_UUID`] and [`NODE_PING_UUID`] reads:
/// seconds since the board heard the report, saturating here.
///
/// Age is measured on the board's own clock rather than taken from the
/// report, because the position fields a sender spends air time on are
/// configurable and `tod_ms` is not among the defaults - a receiver would
/// have nothing to age a beacon by. A value is only ever notified once, on
/// arrival, so a non-zero age means the value was replayed to a central that
/// connected after the fact.
pub const AGE_MAX_S: u16 = u16::MAX;

// -- Config command ids (on the gps-proto config characteristic) -------------
//
// Ids 0x01-0x0F are reserved for gps-proto (0x01 = notify interval).
// Payload format is gps-proto's `[id, len, value]`; acks come back on the
// ack characteristic with the applied value.

/// Current device settings, readable so an app can populate its controls
/// on connect instead of assuming defaults, and notified whenever a value
/// changes (including changes the device makes itself, such as clamping a
/// requested interval).
pub const SETTINGS_UUID: &str = "c3a10009-9f6e-4b2c-8f5a-2e32c3b1e5d0";
pub const SETTINGS_UUID_U128: u128 = 0xc3a10009_9f6e_4b2c_8f5a_2e32c3b1e5d0;

/// The WIO-E5's current radio configuration (the `RADIO.CFG` settings) as one
/// [`crate::radiocfg::RadioConfig`] read-back blob, read + notify.
///
/// The config only ever travels *to* the board, so this is the sole way to
/// learn what a board is actually running - an app can populate its radio
/// editor from the board instead of a local file it has to hope matches. The
/// ESP never parses the config; it relays a snapshot the WIO encodes and
/// sends over the link ([`crate::link::msg::CONFIG`]), refreshed on connect,
/// on the link coming up, and whenever a new config is applied.
///
/// The value is [`crate::radiocfg::RADIO_CONFIG_LEN`] bytes. It reads back as
/// all-zero (layout version 0, which decodes to `None`) until the board has
/// reported one - which, if the GPS/LoRa rail is off, may not happen until a
/// connect powers the WIO.
pub const RADIO_CONFIG_UUID: &str = "c3a1000a-9f6e-4b2c-8f5a-2e32c3b1e5d0";
pub const RADIO_CONFIG_UUID_U128: u128 = 0xc3a1000a_9f6e_4b2c_8f5a_2e32c3b1e5d0;

/// Wire length of [`Settings`].
pub const SETTINGS_LEN: usize = 16;
/// Layout version in byte 0, so an app meeting a newer firmware can
/// reject the blob rather than misread it.
///
/// Version 2 dropped the separate stow interval: one wake-check interval
/// now covers every sleep the board does. Version 3 appended the
/// advertising window.
pub const SETTINGS_VERSION: u8 = 3;

pub const SFLAG_PWR_EN: u8 = 1 << 0;
pub const SFLAG_WIO_SLEEP: u8 = 1 << 1;
pub const SFLAG_GPS_SLEEP: u8 = 1 << 2;

/// Everything the config characteristic can set, in one readable blob.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Settings {
    /// GPS/LoRa rail enabled ([`CFG_PWR_EN`]).
    pub pwr_en: bool,
    /// WIO soft sleep ([`CFG_WIO_SLEEP`]).
    pub wio_sleep: bool,
    /// GPS backup mode ([`CFG_GPS_SLEEP`]).
    pub gps_sleep: bool,
    /// Wake-check interval ([`CFG_ESP_SLEEP_S`]), 0 = sleep disabled.
    pub sleep_interval_s: u32,
    /// Position notify interval in ms (gps-proto config id 0x01).
    pub notify_interval_ms: u32,
    /// Advertising window per wake check ([`CFG_ESP_ADV_WINDOW_S`]). Always
    /// the effective value, never 0.
    pub adv_window_s: u32,
}

impl Settings {
    pub fn encode(&self) -> [u8; SETTINGS_LEN] {
        let mut b = [0u8; SETTINGS_LEN];
        b[0] = SETTINGS_VERSION;
        let mut flags = 0u8;
        if self.pwr_en {
            flags |= SFLAG_PWR_EN;
        }
        if self.wio_sleep {
            flags |= SFLAG_WIO_SLEEP;
        }
        if self.gps_sleep {
            flags |= SFLAG_GPS_SLEEP;
        }
        b[1] = flags;
        // b[2..4] reserved, kept zero to word-align the u32s.
        b[4..8].copy_from_slice(&self.sleep_interval_s.to_le_bytes());
        b[8..12].copy_from_slice(&self.notify_interval_ms.to_le_bytes());
        b[12..16].copy_from_slice(&self.adv_window_s.to_le_bytes());
        b
    }

    /// Returns `None` for a short buffer or an unknown layout version.
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() < SETTINGS_LEN || b[0] != SETTINGS_VERSION {
            return None;
        }
        // The length check above makes the indexing infallible.
        let word = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        Some(Self {
            pwr_en: b[1] & SFLAG_PWR_EN != 0,
            wio_sleep: b[1] & SFLAG_WIO_SLEEP != 0,
            gps_sleep: b[1] & SFLAG_GPS_SLEEP != 0,
            sleep_interval_s: word(4),
            notify_interval_ms: word(8),
            adv_window_s: word(12),
        })
    }
}

/// `u8` 0/1: enable the GPS/LoRa power rail (AP2112K LDO on GPIO2).
/// 0 powers off both the WIO-E5 and the GPS entirely.
pub const CFG_PWR_EN: u8 = 0x10;
/// `u8` 0/1: 1 sends WIO_SLEEP(1) over the link; 0 wakes it (WIO_SLEEP(0),
/// with a reset pulse as fallback when the WIO does not ack).
pub const CFG_WIO_SLEEP: u8 = 0x11;
/// `u8` 0/1: GPS backup mode on/off (forwarded to the WIO).
pub const CFG_GPS_SLEEP: u8 = 0x12;
/// `u32` seconds: ESP deep-sleep wake-check interval. While set (non-zero)
/// the ESP deep-sleeps whenever no central is connected, waking every
/// interval to advertise for a short window. 0 disables sleep mode.
///
/// The board keeps this across a connect: reaching it does not clear the
/// interval, so an unattended tracker holds its cadence indefinitely and
/// the setting means the same thing whether or not anyone is looking. The
/// GPS/LoRa rail stays off through every sleep.
///
/// Note the wake is timed by the C6's uncalibrated RC slow clock, so the
/// interval drifts. It paces a wake-check, not a schedule.
pub const CFG_ESP_SLEEP_S: u8 = 0x13;

/// Clamp range for [`CFG_ESP_SLEEP_S`].
///
/// The ceiling is the worst case for reaching a sleeping board, since
/// deep sleep has no wake source but the timer - nothing over the air can
/// interrupt it. Five minutes keeps that wait short enough that a sleeping
/// board is always a wait rather than a lockout.
pub const ESP_SLEEP_MIN_S: u32 = 5;
pub const ESP_SLEEP_MAX_S: u32 = 5 * 60;

/// `u32` seconds: how long each sleep-mode wake check advertises before
/// going back to deep sleep. Only meaningful while [`CFG_ESP_SLEEP_S`] is
/// set; a board that never sleeps advertises continuously regardless.
///
/// This is the knob that sets the duty cycle, and so the average current:
/// advertising costs roughly two orders of magnitude more than deep sleep,
/// so at a fixed interval the window is what the draw is proportional to.
/// Shortening it buys battery life directly, at the cost of asking more of
/// whoever is trying to connect - the window has to overlap a phone's scan.
pub const CFG_ESP_ADV_WINDOW_S: u8 = 0x14;

/// Clamp range and default for [`CFG_ESP_ADV_WINDOW_S`].
///
/// The floor is not a comfortable connect time, it is the point below
/// which a window stops being worth waking for at all - a phone that only
/// scans intermittently can miss several 3 s windows in a row. The ceiling
/// exists because a window is time spent at full advertising current;
/// past a minute, shortening the interval is the better trade.
pub const ESP_ADV_MIN_S: u32 = 3;
pub const ESP_ADV_MAX_S: u32 = 60;
pub const ESP_ADV_DEFAULT_S: u32 = 15;

// -- Bulk transfer protocol (writes on [`BULK_UUID`]) -------------------------
//
// Each write is one op. Status comes back on the ack characteristic with
// id [`ACK_ID_BULK`]: status ACK_OK and the next expected seq as the value,
// or a non-zero status on error.

/// Bulk op: `[OP_BEGIN, kind u8, total_len u32le, crc32 u32le,
/// version u16le]` (version is 0 for TOML config).
pub const OP_BEGIN: u8 = 0x01;
/// Bulk op: `[OP_DATA, seq u16le, bytes...]`.
pub const OP_DATA: u8 = 0x02;
/// Bulk op: `[OP_END]`.
pub const OP_END: u8 = 0x03;
/// Bulk op: `[OP_ABORT]`.
pub const OP_ABORT: u8 = 0x04;

/// Bulk kind: radio TOML config, forwarded to the WIO and saved to SD.
pub const KIND_TOML: u8 = 1;
/// Bulk kind: WIO-E5 firmware image for the DFU partition.
pub const KIND_FIRMWARE: u8 = 2;

/// Ack id used for bulk transfer status on the ack characteristic.
pub const ACK_ID_BULK: u8 = 0x20;

/// Ack statuses beyond gps-proto's ACK_OK/ACK_UNKNOWN_ID/ACK_BAD_VALUE.
pub const ACK_WIO_ERROR: u8 = 0x10;
pub const ACK_WIO_TIMEOUT: u8 = 0x11;
pub const ACK_BAD_STATE: u8 = 0x12;

/// Max data bytes per OP_DATA write. Fits a 251-byte ATT payload after the
/// 3-byte op header while staying under the UART link chunk size.
pub const BULK_DATA_MAX: usize = crate::link::DATA_CHUNK;

#[cfg(test)]
mod tests {
    use gps_proto::str_eq;

    /// The trouble `#[gatt_service]` macro needs const u128s; make sure the
    /// string forms cannot drift from them (mirrors gps-proto's own test).
    #[test]
    fn uuid_strings_match_u128() {
        fn to_u128(s: &str) -> u128 {
            s.bytes()
                .filter(|&b| b != b'-')
                .fold(0u128, |v, b| v << 4 | (b as char).to_digit(16).unwrap() as u128)
        }
        assert_eq!(to_u128(super::TELEMETRY_UUID), super::TELEMETRY_UUID_U128);
        assert_eq!(to_u128(super::BULK_UUID), super::BULK_UUID_U128);
        assert_eq!(to_u128(super::REMOTE_UUID), super::REMOTE_UUID_U128);
        assert_eq!(to_u128(super::LOG_UUID), super::LOG_UUID_U128);
        assert_eq!(to_u128(super::SETTINGS_UUID), super::SETTINGS_UUID_U128);
        assert_eq!(to_u128(super::RADIO_CONFIG_UUID), super::RADIO_CONFIG_UUID_U128);
        assert_eq!(to_u128(super::NODE_PING_UUID), super::NODE_PING_UUID_U128);
        // Same service as the C3 beacon, different characteristic ids.
        assert!(str_eq(
            gps_proto::packet::SERVICE_UUID,
            "c3a10001-9f6e-4b2c-8f5a-2e32c3b1e5d0"
        ));
    }

    /// The age field is appended, never inserted: everything a reader that
    /// predates it knows about sits at the same offset, which is what lets
    /// an old app read a new board.
    #[test]
    fn age_field_is_appended() {
        assert_eq!(super::REMOTE_LEN, 23);
        assert_eq!(super::REMOTE_LEN_V2, super::REMOTE_LEN + 2);
        assert_eq!(super::REMOTE_AGE_OFF, super::REMOTE_LEN);
        assert_eq!(super::NODE_PING_LEN, crate::link::PING_LEN + 2);
        assert_eq!(super::NODE_PING_AGE_OFF, crate::link::PING_LEN);
    }

    #[test]
    fn settings_roundtrip() {
        let s = super::Settings {
            pwr_en: true,
            wio_sleep: false,
            gps_sleep: true,
            sleep_interval_s: 300,
            notify_interval_ms: 1000,
            adv_window_s: 15,
        };
        let bytes = s.encode();
        assert_eq!(bytes.len(), super::SETTINGS_LEN);
        assert_eq!(super::Settings::decode(&bytes), Some(s));
    }

    #[test]
    fn settings_defaults_roundtrip() {
        let s = super::Settings::default();
        assert_eq!(super::Settings::decode(&s.encode()), Some(s));
    }

    #[test]
    fn settings_rejects_short_and_wrong_version() {
        let good = super::Settings::default().encode();
        assert!(super::Settings::decode(&good[..super::SETTINGS_LEN - 1]).is_none());
        let mut bad = good;
        bad[0] = super::SETTINGS_VERSION + 1;
        assert!(super::Settings::decode(&bad).is_none());
    }

    /// A longer buffer must still decode: a future layout can only grow,
    /// and byte 0 is what gates compatibility.
    #[test]
    fn settings_tolerates_trailing_bytes() {
        let good = super::Settings::default().encode();
        let mut longer = [0u8; super::SETTINGS_LEN + 4];
        longer[..super::SETTINGS_LEN].copy_from_slice(&good);
        assert!(super::Settings::decode(&longer).is_some());
    }
}
