//! BLE GATT extensions served by the board's firmware.
//!
//! The board reuses the gps-proto service and its position/config/ack
//! characteristics, so the existing gps-gui-rs app connects and streams
//! positions unchanged. This module adds board-specific characteristics
//! (continuing the same UUID sequence) and new config command ids on the
//! existing config characteristic.

/// Name the board advertises under. The service UUID (which the app
/// filters scans by) stays `gps_proto::packet::SERVICE_UUID`, so this is
/// display text and renaming it does not break a scan.
pub const DEVICE_NAME: &str = "GPS-S3";

/// [`crate::link::Telemetry`] wire format, notify + read.
pub const TELEMETRY_UUID: &str = "c3a10005-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// Bulk transfer (radio TOML config), write.
pub const BULK_UUID: &str = "c3a10006-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// Remote position: `[src u8, rssi i16le, PositionPacket 20B, age_s u16le]`,
/// notify + read. One notification per report received, not per tick.
pub const REMOTE_UUID: &str = "c3a10007-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// Status/log line (ASCII), notify + read. Carries the latest line, up to
/// [`crate::link::LOG_MAX`] bytes.
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

/// Node ping value length: the [`crate::link::PING_LEN`] payload plus the
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

/// The board's current radio configuration (the `RADIO.CFG` settings) as one
/// [`crate::radiocfg::RadioConfig`] read-back blob, read + notify.
///
/// The config only ever travels *to* the board, so this is the sole way to
/// learn what a board is actually running - an app can populate its radio
/// editor from the board instead of a local file it has to hope matches.
/// Refreshed on connect and whenever a new config is applied.
///
/// The value is [`crate::radiocfg::RADIO_CONFIG_LEN`] bytes, and reads back
/// as all-zero (layout version 0, which decodes to `None`) until the radio
/// has been initialized.
pub const RADIO_CONFIG_UUID: &str = "c3a1000a-9f6e-4b2c-8f5a-2e32c3b1e5d0";
pub const RADIO_CONFIG_UUID_U128: u128 = 0xc3a1000a_9f6e_4b2c_8f5a_2e32c3b1e5d0;

/// Wire length of [`Settings`].
pub const SETTINGS_LEN: usize = 20;
/// Layout version in byte 0, so an app meeting a newer firmware can
/// reject the blob rather than misread it.
///
/// Version 2 dropped the separate stow interval: one wake-check interval
/// now covers every sleep the board does. Version 3 appended the
/// advertising window, version 4 the BLE off period.
pub const SETTINGS_VERSION: u8 = 4;

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
    /// Seconds the BLE controller stays powered down between advertising
    /// windows ([`CFG_BLE_OFF_S`]), 0 = never take it down.
    pub ble_off_s: u32,
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
        b[16..20].copy_from_slice(&self.ble_off_s.to_le_bytes());
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
            ble_off_s: word(16),
        })
    }
}

/// `u8` 0/1: enable the GPS/LoRa power rail.
///
/// The wio-s3-max-gps board has no such rail - the GPS and SD sit directly
/// on +3V3 - so its firmware accepts the write and logs that there is no
/// hardware behind it. Kept because a board respin could bring it back.
pub const CFG_PWR_EN: u8 = 0x10;
/// `u8` 0/1: 1 puts the radio into standby, 0 brings it back.
pub const CFG_WIO_SLEEP: u8 = 0x11;
/// `u8` 0/1: GPS backup mode on/off.
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
/// The floor exists only to keep a window from being no window at all: at
/// one second the board is genuinely advertising, and a central that scans
/// continuously (the app's "connect to sleeping") will catch it. It is not
/// a comfortable connect time - a phone that scans intermittently can miss
/// several one-second windows in a row - so a window that short is a
/// deliberate duty-cycle choice, or a bench setting for watching a whole
/// wake/advertise/sleep round go by in seconds. The ceiling exists because
/// a window is time spent at full advertising current; past a minute,
/// shortening the interval is the better trade.
pub const ESP_ADV_MIN_S: u32 = 1;
pub const ESP_ADV_MAX_S: u32 = 60;
pub const ESP_ADV_DEFAULT_S: u32 = 15;

/// `u32` seconds: how long the BLE controller stays powered down between
/// advertising windows. 0 disables it, which is the default and the old
/// behavior - a board that is not deep-sleeping advertises continuously.
///
/// This is the awake-state counterpart to [`CFG_ESP_SLEEP_S`], and on the
/// Wio-S3 it is the largest lever the firmware has. BLE measures **71 mA of
/// the board's 126**, and it cannot be reduced while the controller exists:
/// esp-radio does not implement the controller's modem sleep, so the PHY
/// stays up for as long as `BleConnector` is alive. Dropping the connector
/// calls `ble_deinit` and takes the PHY with it, which is what this
/// schedules.
///
/// Unlike deep sleep it costs no reset and stops nothing else: the LoRa
/// beacon keeps transmitting, the GPS keeps tracking and the card keeps
/// logging. What it costs is reachability - a board inside its off period
/// cannot be connected to, exactly as a sleeping one cannot, so the ceiling
/// is set the same way and for the same reason.
pub const CFG_BLE_OFF_S: u8 = 0x16;

/// Clamp range for [`CFG_BLE_OFF_S`], plus 0 to disable.
///
/// The ceiling is five minutes for the reason [`ESP_SLEEP_MAX_S`] is: it is
/// the worst case for how long the board is unreachable, and past a few
/// minutes that stops being a wait and starts being a lockout. The floor
/// keeps an off period long enough to be worth the controller teardown and
/// re-init, which is not free.
pub const BLE_OFF_MIN_S: u32 = 5;
pub const BLE_OFF_MAX_S: u32 = 5 * 60;

/// `u32` seconds: sleep *now*, for this long, and then come back.
///
/// A command, not a setting. Nothing about it is stored, it does not touch
/// the wake-check cadence, and the board resumes its configured behavior on
/// the far side - so a board with sleep mode off takes one nap and goes
/// back to advertising continuously.
///
/// This exists because every other way into deep sleep is indirect. The
/// wake-check interval sleeps a board when an advertising window expires
/// with nobody connected, which means the only way to put a board you are
/// *looking at* to sleep is to disconnect and wait out the window. That is
/// the wrong shape for the two cases that matter: packing a tracker away,
/// and confirming on a bench that sleep works at all.
///
/// The value is clamped to the [`ESP_SLEEP_MIN_S`]..=[`ESP_SLEEP_MAX_S`]
/// range the wake-check interval uses, with one exception: 0 means "for the
/// configured wake-check interval", falling back to [`SLEEP_NOW_DEFAULT_S`]
/// on a board that has sleep mode off and so has no cadence to borrow. The
/// ack echoes the resolved seconds, so an app can say how long the board
/// will be gone rather than repeating what it asked for.
///
/// The board acks before it sleeps and the link then drops. That
/// disconnect is the command working, not a failure.
pub const CFG_SLEEP_NOW: u8 = 0x15;

/// What [`CFG_SLEEP_NOW`] with a value of 0 resolves to when sleep mode is
/// off. Long enough to be an unmistakable sleep on a bench and short enough
/// that nobody is waiting on a board they put down by accident.
pub const SLEEP_NOW_DEFAULT_S: u32 = 60;

/// Resolve a [`CFG_SLEEP_NOW`] request into the seconds the board will
/// actually sleep for.
///
/// Shared rather than inlined at the one call site because the app shows
/// this number *before* the ack comes back - a control that says "sleeps
/// for 5 minutes" and a board that sleeps for 60 s would be the app's
/// arithmetic disagreeing with the firmware's, which is exactly the class
/// of drift the settings characteristic exists to prevent.
pub fn resolve_sleep_now(requested_s: u32, sleep_interval_s: u32) -> u32 {
    let secs = match (requested_s, sleep_interval_s) {
        (0, 0) => SLEEP_NOW_DEFAULT_S,
        (0, cadence) => cadence,
        (asked, _) => asked,
    };
    secs.clamp(ESP_SLEEP_MIN_S, ESP_SLEEP_MAX_S)
}

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

/// Bulk kind: radio TOML config, applied live and saved to SD.
pub const KIND_TOML: u8 = 1;
/// Bulk kind 2 was a WIO-E5 firmware image for its DFU partition, streamed
/// through the ESP32-C6. The Wio-S3 updates itself through ESP-IDF OTA, so
/// the kind is retired rather than reused - an old tool pushing an STM32
/// image at this firmware should be rejected, not misread.

/// Bulk kind: an ESP-IDF application image for the inactive OTA slot.
///
/// The image is written straight through to flash as it arrives rather than
/// buffered, and the next boot runs it (see [`crate::bulk::Sink`]). Kind 3
/// rather than 2 so the two firmware formats can never be confused: an STM32
/// image accepted here would be written into an app slot and bootlooped.
pub const KIND_OTA: u8 = 3;

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
            ble_off_s: 60,
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
