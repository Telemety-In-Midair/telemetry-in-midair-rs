//! BLE GATT extensions served by the board's firmware.
//!
//! The board reuses the gps-proto service and its position/config/ack
//! characteristics, so the existing gps-gui-rs app connects and streams
//! positions unchanged. This module adds board-specific characteristics
//! (continuing the same UUID sequence) and new config command ids on the
//! existing config characteristic.

/// Prefix every board's advertised name carries.
///
/// A named board advertises `<prefix>-<label>` ("ws3gps-sky-1"); one that
/// has never been named advertises `<prefix>-<xxxx>` from the tail of its
/// BLE address, so two boards out of the same box are already told apart in
/// a scan list. See [`advertised_name`].
///
/// The prefix is a firmware constant rather than part of the label, so that
/// a board cannot be named something unrecognizable: whatever it is called,
/// a generic scanner can be searched by this. It is not what an app filters
/// on - the service UUID is, and that travels in the advertisement rather
/// than the scan response - so a name is display text and renaming a board
/// cannot lose it.
pub const NAME_PREFIX: &str = "ws3gps";

/// Separator between the prefix and the label.
pub const NAME_SEP: u8 = b'-';

/// Bytes of label a board stores (see [`CFG_NAME`]).
///
/// The ceiling is not the air - a scan response holds 29 bytes of name -
/// but the GAP device-name characteristic, which trouble-host builds into a
/// fixed 22-byte string and refuses to build at all past it. Fifteen is
/// what is left of that after the prefix and its separator, and it is
/// longer than the place-and-number labels these boards get ("ground-1",
/// "sky-1").
pub const NAME_LABEL_MAX: usize = 15;

/// Bytes a stored label occupies.
///
/// One more than the longest label, which does two things: it keeps the
/// settings record a multiple of the flash write word, and it means a
/// full-length label is still followed by a zero - so the padding a reader
/// stops at is always there.
pub const NAME_FIELD_LEN: usize = NAME_LABEL_MAX + 1;

/// Longest name a board advertises: prefix, separator, label.
pub const NAME_MAX: usize = NAME_PREFIX.len() + 1 + NAME_LABEL_MAX;

/// Whether a label is one a board will store.
///
/// ASCII letters, digits, `-` and `_`, at most [`NAME_LABEL_MAX`] of them.
/// The charset is what a name can be without needing an escape somewhere:
/// it travels through a scan list, a console line and a log file, and a
/// space or a quote in it is a different amount of trouble in each. An
/// empty label is not valid here - clearing a name is a separate case that
/// [`CFG_NAME`] checks for first.
pub fn valid_label(label: &[u8]) -> bool {
    !label.is_empty()
        && label.len() <= NAME_LABEL_MAX
        && label
            .iter()
            .all(|b| b.is_ascii_alphanumeric() || *b == b'-' || *b == b'_')
}

/// The name a board with this label and address advertises under, written
/// into `buf`.
///
/// An empty (or invalid) label is an unnamed board, which falls back to the
/// tail of its BLE address. `addr` is LSB-first, as the controller takes
/// it, and the two bytes used are the two a scanner prints last - so the
/// name and the address a phone shows agree about which board this is.
///
/// Shared with the app rather than formatted at each end: the app strips
/// the prefix to show a label, and a board that built its name by another
/// rule would have the app displaying the wrong half of it.
pub fn advertised_name<'a>(label: &str, addr: &[u8; 6], buf: &'a mut [u8; NAME_MAX]) -> &'a str {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut n = 0;
    for &b in NAME_PREFIX.as_bytes() {
        buf[n] = b;
        n += 1;
    }
    buf[n] = NAME_SEP;
    n += 1;
    if valid_label(label.as_bytes()) {
        for &b in label.as_bytes() {
            buf[n] = b;
            n += 1;
        }
    } else {
        for &b in &[addr[1], addr[0]] {
            buf[n] = HEX[usize::from(b >> 4)];
            buf[n + 1] = HEX[usize::from(b & 0x0F)];
            n += 2;
        }
    }
    // Every byte written above came from ASCII, so this cannot fail; the
    // fallback keeps the function total rather than panicking on the air.
    core::str::from_utf8(&buf[..n]).unwrap_or(NAME_PREFIX)
}

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

/// The board's name as it advertises it (ASCII, up to [`NAME_MAX`] bytes),
/// read + notify.
///
/// The same string the scan response carries, so an app that connected to
/// a board need not have kept the scan around to know which one it is
/// talking to, and one that has just renamed a board sees the new name
/// without waiting for the next advertising window.
///
/// Read-only: a name is set through [`CFG_NAME`] on the config
/// characteristic, so every settings write a board takes goes through one
/// path and produces one ack.
pub const NAME_UUID: &str = "c3a1000c-9f6e-4b2c-8f5a-2e32c3b1e5d0";
pub const NAME_UUID_U128: u128 = 0xc3a1000c_9f6e_4b2c_8f5a_2e32c3b1e5d0;

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

/// Which node this board is on the LoRa network, and what it is called:
/// `[address u8, label zero-padded to [`NAME_FIELD_LEN`]]`, read + notify.
///
/// This is what lets an app name a remote node. A frame carries the
/// originating address and nothing else about the sender - there is no
/// room on a shared channel to spend a turn announcing a name that changes
/// once in a board's life - so the pairing is made where it costs nothing:
/// an app that has connected to a board once knows that address 3 is
/// `sky-1`, and can say so wherever it later hears node 3 reported, from
/// whichever board it happens to be connected to.
///
/// One value rather than the two it could be read from. The address is a
/// field of the radio config blob and the label is on [`NAME_UUID`], and
/// they change at different moments - a rename notifies one, a config push
/// the other - so an app joining them itself would sooner or later record
/// a name against the address that board had before the push.
///
/// Address 0 means the radio has not been configured yet and the board
/// cannot say which node it is; 0 is not an assignable address. An empty
/// label is a board that has never been named, which an app shows as the
/// address it already has.
pub const NODE_ID_UUID: &str = "c3a1000d-9f6e-4b2c-8f5a-2e32c3b1e5d0";
pub const NODE_ID_UUID_U128: u128 = 0xc3a1000d_9f6e_4b2c_8f5a_2e32c3b1e5d0;

/// Node id value length: the LoRa address and the padded label.
pub const NODE_ID_LEN: usize = 1 + NAME_FIELD_LEN;

/// The [`NODE_ID_UUID`] value for a board at `address` called `label`.
///
/// A label that is not one a board stores goes out empty rather than as
/// itself, which is [`crate::session::Stored::label`]'s rule: the value
/// says "unnamed", and an app falls back to the address.
pub fn node_id(address: u8, label: &str) -> [u8; NODE_ID_LEN] {
    let mut v = [0u8; NODE_ID_LEN];
    v[0] = address;
    if valid_label(label.as_bytes()) {
        v[1..1 + label.len()].copy_from_slice(label.as_bytes());
    }
    v
}

/// Read a [`NODE_ID_UUID`] value back as `(address, label)`.
///
/// `None` only for a value too short to be one. Trailing bytes are
/// tolerated, so a future layout can grow the way the remote-position one
/// did. The label stops at its padding and reads as `""` unless it is a
/// label a board would store - the same refusal as on the way out, because
/// this is the one string on the link that reaches an app's own display.
pub fn parse_node_id(v: &[u8]) -> Option<(u8, &str)> {
    let v = v.get(..NODE_ID_LEN)?;
    let end = v[1..]
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(NAME_LABEL_MAX);
    let label = match core::str::from_utf8(&v[1..1 + end]) {
        Ok(s) if valid_label(s.as_bytes()) => s,
        _ => "",
    };
    Some((v[0], label))
}

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

/// What the board is for right now.
///
/// This is the one setting an app actually means when it asks for a
/// tracker, or for a device that is going in a bag. It replaces the pair of
/// independent sleep flags ([`CFG_RADIO_STANDBY`], [`CFG_GPS_SLEEP`]) as the
/// thing an app sets: those stay, and are still the way to park one
/// subsystem and leave the other up, but neither of them decides what a
/// board does when it comes back from a flat cell, and this does.
///
/// Not to be confused with [`crate::session::Stored`], the settings record.
/// `Mode::Stored` is the device in storage; `session::Stored` is what
/// survives a sleep.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Everything the firmware can lower, lowered: deep sleep on the
    /// wake-check cadence, GPS in backup, radio in cold sleep, card
    /// unmounted, panel dark. The default, and where an unattended board
    /// spends most of its life.
    ///
    /// While a stored board is *awake* - the advertising window of one wake
    /// check - the mode still reads `Stored`, because that is what the
    /// window is for: asking whether anyone wants the board back.
    #[default]
    Stored,
    /// Awake and connectable, but not tracking: the GPS stays in backup and
    /// the radio stays down, so monitoring a stored object's configuration
    /// does not cost an acquisition.
    ///
    /// Deliberately transient. It always ends - by [`CFG_IDLE_TIMEOUT_S`]
    /// or by a command - and it is never what reaches flash (see
    /// [`Mode::persisted`]), because a board that came back from a reset
    /// still believing it was idle would sit at awake current with nobody
    /// coming.
    Idle,
    /// Tracking: GPS acquiring, positions out over LoRa, pings without a
    /// fix, card logging. Entered by command and left by command - there is
    /// no battery sense on this board, and a timeout that silently stopped
    /// tracking a flying object would be worse than a flat cell.
    Tracking,
    /// Listening: the node held next to the phone. GPS acquiring and the
    /// receiver up, so everything heard over LoRa is relayed and the phone
    /// can use this node's fix as its own - but nothing goes out on the
    /// air, and BLE stays up continuously so the phone connects at once.
    ///
    /// Entered and left by command, like tracking, and persisted the same
    /// way: a node that browns out in a pocket should come back listening,
    /// not go dark on the phone that was using it.
    Listening,
}

impl Mode {
    /// Decode the wire byte. `None` for a value this firmware does not
    /// know, which a config write rejects rather than guessing at.
    pub fn from_wire(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Stored),
            1 => Some(Self::Idle),
            2 => Some(Self::Tracking),
            3 => Some(Self::Listening),
            _ => None,
        }
    }

    pub fn as_wire(self) -> u8 {
        match self {
            Self::Stored => 0,
            Self::Idle => 1,
            Self::Tracking => 2,
            Self::Listening => 3,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Stored => "stored",
            Self::Idle => "idle",
            Self::Tracking => "tracking",
            Self::Listening => "listening",
        }
    }

    /// What this mode becomes on its way to flash.
    ///
    /// Only the commanded working modes are worth surviving a power cycle:
    /// a board that was tracking must come back tracking, one that was
    /// listening beside a phone must come back listening, and everything
    /// else must come back reachable. Idle is neither - it is "awake
    /// because someone might want me", and a reboot is exactly the moment
    /// nobody does - so it persists as [`Mode::Stored`] and the cold-boot
    /// rescue window brings it back to Idle anyway.
    pub fn persisted(self) -> Self {
        match self {
            Self::Idle => Self::Stored,
            other => other,
        }
    }

    /// Whether the GPS, the radio and the card come up with the board.
    pub fn tracks(self) -> bool {
        matches!(self, Self::Tracking | Self::Listening)
    }

    /// Whether the node puts anything on the air: beacons, pings and
    /// repeats. Listening is the mode that raises the radio and still says
    /// no here.
    pub fn transmits(self) -> bool {
        matches!(self, Self::Tracking)
    }
}

/// Wire length of [`Settings`].
pub const SETTINGS_LEN: usize = 28;
/// Layout version in byte 0, so an app meeting a newer firmware can
/// reject the blob rather than misread it.
///
/// Version 2 dropped the separate stow interval: one wake-check interval
/// now covers every sleep the board does. Version 3 appended the
/// advertising window, version 4 the BLE off period, version 5 the
/// [`Mode`] and its idle timeout, and version 6 the BLE on period.
pub const SETTINGS_VERSION: u8 = 6;

// Bit 0 was the GPS/LoRa rail of the two-MCU board, which this one does
// not have. Reserved: never set, ignored on the way in.
pub const SFLAG_RADIO_STANDBY: u8 = 1 << 1;
pub const SFLAG_GPS_SLEEP: u8 = 1 << 2;

/// Everything the config characteristic can set, in one readable blob.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Settings {
    /// The radio is parked in standby ([`CFG_RADIO_STANDBY`]).
    pub radio_standby: bool,
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
    /// windows ([`CFG_BLE_OFF_S`]), 0 = never take it down. Honored in
    /// [`Mode::Tracking`] only.
    pub ble_off_s: u32,
    /// What the board is doing right now ([`CFG_MODE`]).
    ///
    /// The live mode, not the persisted one: a board reporting
    /// [`Mode::Idle`] has [`Mode::Stored`] in its flash record, because idle
    /// is not a state a reboot may come back into.
    pub mode: Mode,
    /// Seconds [`Mode::Idle`] lasts before the board stores itself
    /// ([`CFG_IDLE_TIMEOUT_S`]), 0 = it never does.
    pub idle_timeout_s: u32,
    /// Seconds BLE stays up between off periods while tracking
    /// ([`CFG_BLE_ON_S`]). Always the effective value, never 0.
    pub ble_on_s: u32,
}

impl Settings {
    pub fn encode(&self) -> [u8; SETTINGS_LEN] {
        let mut b = [0u8; SETTINGS_LEN];
        b[0] = SETTINGS_VERSION;
        let mut flags = 0u8;
        if self.radio_standby {
            flags |= SFLAG_RADIO_STANDBY;
        }
        if self.gps_sleep {
            flags |= SFLAG_GPS_SLEEP;
        }
        b[1] = flags;
        // The mode takes one of the two bytes that were reserved to
        // word-align the u32s below, so it costs nothing and everything
        // after it keeps the offset an older reader knew.
        b[2] = self.mode.as_wire();
        b[8..12].copy_from_slice(&self.notify_interval_ms.to_le_bytes());
        // The durations sit where the knob table says, which is the one
        // place their layout is written down.
        for spec in &crate::session::KNOBS {
            let at = spec.wire_at;
            b[at..at + 4].copy_from_slice(&spec.wire_get(self).to_le_bytes());
        }
        b
    }

    /// Returns `None` for a short buffer or an unknown layout version.
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() < SETTINGS_LEN || b[0] != SETTINGS_VERSION {
            return None;
        }
        // The length check above makes the indexing infallible.
        let word = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let mut s = Self {
            radio_standby: b[1] & SFLAG_RADIO_STANDBY != 0,
            gps_sleep: b[1] & SFLAG_GPS_SLEEP != 0,
            notify_interval_ms: word(8),
            // An unknown mode byte is a board running firmware this reader
            // predates; report the safe one rather than refusing the whole
            // blob, which would take every other field with it.
            mode: Mode::from_wire(b[2]).unwrap_or_default(),
            ..Self::default()
        };
        for spec in &crate::session::KNOBS {
            spec.wire_set(&mut s, word(spec.wire_at));
        }
        Some(s)
    }
}

// 0x10 was the GPS/LoRa rail of the two-MCU board. This board has no rail
// - the GPS and SD sit directly on +3V3 - so the id is reserved and a
// write to it is refused as unknown, which is what it is.

/// `u8` 0/1: 1 puts the radio into standby, 0 brings it back to receive.
/// An override inside a tracking posture; outside one the mode has the
/// radio down already and the flag waits for the next tracking command.
pub const CFG_RADIO_STANDBY: u8 = 0x11;
/// `u8` 0/1: GPS backup mode on/off.
pub const CFG_GPS_SLEEP: u8 = 0x12;
/// `u32` seconds: ESP deep-sleep wake-check interval, i.e. the cadence
/// [`Mode::Stored`] runs on. 0 disables deep sleep.
///
/// This is a [`Mode::Stored`] knob and nothing else's. A stored board sleeps
/// this long, wakes into a check, advertises for
/// [`CFG_ESP_ADV_WINDOW_S`], and goes back down if nobody came; a board in
/// [`Mode::Idle`] uses it as the sleep it takes when its idle timeout runs
/// out; a board in [`Mode::Tracking`] ignores it entirely, because deep
/// sleep stops the beacon, the logging and the listening, and a tracker
/// doing that is not tracking.
///
/// 0 therefore means "never store this board *on its own*" rather than
/// merely "do not sleep": with no cadence to sleep on, the idle timeout has
/// nowhere to send it and it stays awake and reachable. That is the bench
/// setting, and it is what an unconfigured board does. A board explicitly
/// told `CFG_MODE stored` still sleeps - somebody asked for that one, so it
/// borrows [`ESP_SLEEP_MAX_S`] rather than treating the missing cadence as
/// a refusal.
///
/// The board keeps this across a connect: reaching it does not clear the
/// interval, so an unattended tracker holds its cadence indefinitely and
/// the setting means the same thing whether or not anyone is looking.
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
/// A [`Mode::Stored`] knob and nothing else's: the on-half of a tracker's
/// modem duty cycle is [`CFG_BLE_ON_S`], which used to share this value
/// and no longer does - a wake check and a tracker want different lengths
/// of window, and one number was always wrong for one of them.
///
/// This is the knob that sets the stored duty cycle, and so the average
/// current: advertising costs roughly two orders of magnitude more than
/// deep sleep, so at a fixed interval the window is what the draw is
/// proportional to. Shortening it buys battery life directly, at the cost
/// of asking more of whoever is trying to connect - the window has to
/// overlap a phone's scan.
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
/// This is a [`Mode::Tracking`] knob and nothing else's: it exists so a
/// tracker nobody is talking to does not pay 71 mA for reachability, and
/// the other two modes have no use for it - [`Mode::Idle`] exists to be
/// reachable, and [`Mode::Stored`] has no controller at all. Before the
/// mode existed the two duty cycles competed inside one state and deep
/// sleep silently won, which made this dead config on any board that had a
/// wake-check cadence set.
///
/// On the Wio-S3 it is the lever that takes BLE to zero. BLE measured **71
/// mA of the board's 126** with the PHY up continuously, which is what the
/// board did before the vendored esp-radio learned the controller's modem
/// sleep. Modem sleep now takes back whatever the gaps between
/// advertisements are worth - unmeasured, and the first thing to put on a
/// meter - but only destroying the controller takes the modem to nothing,
/// and that is what this schedules: dropping the connector calls
/// `ble_deinit` and takes the PHY with it.
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

/// `u8`: the board's [`Mode`] - 0 stored, 1 idle, 2 tracking, 3 listening.
///
/// The write an app makes when it means "this is a tracker now" or "this is
/// going in a bag". Each value is a whole posture rather than one
/// subsystem:
///
/// - **2, tracking.** The GPS comes up, the radio comes up, the card logs.
///   Persisted, so it survives a brownout on the object - which is the one
///   time it must.
/// - **1, idle.** GPS into backup, radio down, BLE up. Transient: the board
///   stores itself once [`CFG_IDLE_TIMEOUT_S`] runs out.
/// - **0, stored.** The board acks and then deep-sleeps on its wake-check
///   cadence, exactly as [`CFG_SLEEP_NOW`] does. The link drops; that
///   disconnect is the command working.
/// - **3, listening.** GPS up and the receiver up, nothing transmitted,
///   BLE up continuously. The node held next to the phone. Persisted.
///
/// A value outside 0..=3 is rejected rather than rounded, because every one
/// of them is a different amount of the board switched off.
pub const CFG_MODE: u8 = 0x17;

/// `u32` seconds: how long [`Mode::Idle`] lasts before the board stores
/// itself. 0 turns that off, and 0 is the default: an idle board stays
/// idle until something tells it otherwise.
///
/// Idle is the expensive state - BLE dominates it at around 90 mA, a figure
/// from before the vendored esp-radio learned the controller's modem sleep
/// and not re-taken since - so a board left idle by accident runs its cell
/// down. The timeout is the guard against that, for whoever wants one; it
/// is off by default because a board that stored itself while its owner
/// was still setting it up was the more common surprise.
///
/// Storing needs a cadence to wake on as well: with [`CFG_ESP_SLEEP_S`] at
/// 0 there is nowhere for the timeout to send the board, and it stays
/// awake and reachable whatever this says.
pub const CFG_IDLE_TIMEOUT_S: u8 = 0x18;

/// Clamp range for a non-zero [`CFG_IDLE_TIMEOUT_S`], and the value an app
/// offers when the timeout is switched on.
///
/// The floor is short enough to watch a whole promote/timeout/store cycle
/// go by on a bench and long enough that a phone which has just woken the
/// board can still finish connecting. The ceiling is an hour: past that the
/// timeout has stopped being a timeout and the board should have been told
/// to track.
pub const IDLE_TIMEOUT_MIN_S: u32 = 10;
pub const IDLE_TIMEOUT_MAX_S: u32 = 60 * 60;
pub const IDLE_TIMEOUT_DEFAULT_S: u32 = 10 * 60;

/// `u32` seconds: how long BLE stays up between off periods while
/// tracking - the on-half of the [`CFG_BLE_OFF_S`] duty cycle. 0 means
/// never configured and resolves to [`BLE_ON_DEFAULT_S`].
///
/// A [`Mode::Tracking`] knob and nothing else's. It was the advertising
/// window until the two were separated: a wake check wants the shortest
/// window a phone can still catch, and a tracker wants one long enough for
/// the phone to connect, read the roster and let go, and no single number
/// served both.
pub const CFG_BLE_ON_S: u8 = 0x1A;

/// Clamp range and default for [`CFG_BLE_ON_S`], the same bounds as the
/// advertising window and for the same reasons.
pub const BLE_ON_MIN_S: u32 = 1;
pub const BLE_ON_MAX_S: u32 = 60;
pub const BLE_ON_DEFAULT_S: u32 = 15;

/// ASCII label up to [`NAME_LABEL_MAX`] bytes: what this board is called.
///
/// The board advertises `<NAME_PREFIX>-<label>` from the next advertising
/// window on and reports the whole name on [`NAME_UUID`]. An empty value
/// clears the label, which returns the board to its address-derived name.
///
/// Persisted alongside the settings that decide reachability, and for the
/// same reason: a name has to survive a deep sleep and a flat cell, and it
/// has to be readable during a wake check, which mounts no card.
///
/// A label outside [`valid_label`] is rejected rather than sanitized. A
/// board answering to a name nobody asked for is worse than a write that
/// failed loudly, and the app cannot show the difference between the two
/// unless the board refuses.
///
/// The ack carries the stored length rather than the label, because an ack
/// is `gps_proto::packet::ACK_MAX_LEN` bytes and no name fits in one; a
/// length of 0 means the name was cleared. What the board is actually
/// called comes back on [`NAME_UUID`].
pub const CFG_NAME: u8 = 0x19;

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
// Bulk kind 2 was a WIO-E5 firmware image for its DFU partition, streamed
// through the ESP32-C6. The Wio-S3 updates itself through ESP-IDF OTA, so
// the kind is retired rather than reused - an old tool pushing an STM32
// image at this firmware should be rejected, not misread.

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
///
/// The board refused for a reason of its own rather than the op's: no
/// update slots, an image too big for the one it would go in, a flash
/// write that failed.
pub const ACK_BOARD_ERROR: u8 = 0x10;
// 0x11 was a timeout waiting on the two-MCU board's second chip. Reserved.
/// The op does not fit the transfer's state: another transport owns one,
/// or there is none for this op to act on.
pub const ACK_BAD_STATE: u8 = 0x12;

/// Max data bytes per OP_DATA write. Fits a 251-byte ATT payload after the
/// 3-byte op header while staying under the UART link chunk size.
pub const BULK_DATA_MAX: usize = 192;

/// Longest write any characteristic in this protocol takes: an `OP_DATA`
/// frame, which is [`BULK_DATA_MAX`] behind a three-byte
/// `[op, seq u16le]` header. Every other write - a config item, a bulk
/// begin or end - is shorter.
///
/// It exists so that a device can size the buffer it copies a write into
/// from the protocol rather than from a round number. A buffer that is
/// merely close enough truncates the tail of a full-length chunk silently,
/// and what the sender sees for that is a CRC failure at the end of a
/// transfer that looked fine.
pub const WRITE_MAX: usize = 3 + BULK_DATA_MAX;

/// Longest write the config characteristic takes: `[id, len, value]` with
/// the longest value any id defines, which is a [`CFG_NAME`] label.
///
/// It sizes the characteristic, so the attribute layer refuses anything
/// longer with an ATT error rather than handing the policy a name with its
/// tail missing. Raising it is backward compatible in the direction that
/// matters - every other config write is three to six bytes, and a central
/// that only ever sends those cannot tell the difference.
pub const CONFIG_WRITE_MAX: usize = 2 + NAME_LABEL_MAX;

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
        assert_eq!(to_u128(super::NAME_UUID), super::NAME_UUID_U128);
        assert_eq!(to_u128(super::NODE_ID_UUID), super::NODE_ID_UUID_U128);
        // Same service as the C3 beacon, different characteristic ids.
        assert!(str_eq(
            gps_proto::packet::SERVICE_UUID,
            "c3a10001-9f6e-4b2c-8f5a-2e32c3b1e5d0"
        ));
    }

    /// The pairing an app records: this board is node 3, and node 3 is
    /// called sky-1.
    #[test]
    fn node_id_round_trips() {
        let v = super::node_id(3, "sky-1");
        assert_eq!(v.len(), super::NODE_ID_LEN);
        assert_eq!(super::parse_node_id(&v), Some((3, "sky-1")));
        // The padding a reader stops at is always there: the field is one
        // byte longer than the longest label.
        assert_eq!(v[super::NODE_ID_LEN - 1], 0);

        let long = "a".repeat(super::NAME_LABEL_MAX);
        let v = super::node_id(255, &long);
        assert_eq!(super::parse_node_id(&v), Some((255, long.as_str())));
    }

    /// A board that has never been named says so, and is not mistaken for
    /// one called something unreadable. The app falls back to the address,
    /// which it has either way.
    #[test]
    fn an_unnamed_board_has_a_node_id_with_no_label() {
        assert_eq!(super::parse_node_id(&super::node_id(3, "")), Some((3, "")));
        // Anything that is not a label a board would store reads as
        // unnamed rather than as itself, on the way out and on the way in.
        assert_eq!(super::parse_node_id(&super::node_id(3, "sky 1")), Some((3, "")));
        let mut v = super::node_id(3, "sky-1");
        v[1] = b' ';
        assert_eq!(super::parse_node_id(&v), Some((3, "")));
        v[1] = 0xFF;
        assert_eq!(super::parse_node_id(&v), Some((3, "")));
    }

    /// Address 0 is not assignable, so it is what a board says before its
    /// radio is configured - an app must not record a name against it.
    #[test]
    fn address_zero_is_a_board_that_cannot_say_yet() {
        assert_eq!(super::parse_node_id(&super::node_id(0, "sky-1")), Some((0, "sky-1")));
    }

    /// Short is refused; longer is not, so the value can grow later the
    /// way the remote-position one did.
    #[test]
    fn node_id_tolerates_a_longer_value_and_refuses_a_short_one() {
        let v = super::node_id(7, "sky-1");
        assert_eq!(super::parse_node_id(&v[..super::NODE_ID_LEN - 1]), None);
        assert_eq!(super::parse_node_id(&[]), None);
        let mut longer = v.to_vec();
        longer.extend_from_slice(&[1, 2, 3]);
        assert_eq!(super::parse_node_id(&longer), Some((7, "sky-1")));
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
            radio_standby: false,
            gps_sleep: true,
            sleep_interval_s: 300,
            notify_interval_ms: 1000,
            adv_window_s: 15,
            ble_off_s: 60,
            mode: super::Mode::Tracking,
            idle_timeout_s: 600,
            ble_on_s: 20,
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

    /// Every mode survives the blob, and the byte it travels in is the one
    /// that used to be padding - so nothing else moved.
    #[test]
    fn settings_carry_the_mode() {
        for mode in [
            super::Mode::Stored,
            super::Mode::Idle,
            super::Mode::Tracking,
            super::Mode::Listening,
        ] {
            let s = super::Settings {
                mode,
                ..Default::default()
            };
            let bytes = s.encode();
            assert_eq!(bytes[2], mode.as_wire());
            assert_eq!(super::Settings::decode(&bytes), Some(s));
        }
    }

    /// The wire values are fixed: an app and a board disagreeing about
    /// which number means "tracking" is a board that stores itself when it
    /// was told to track.
    #[test]
    fn mode_wire_values_are_fixed() {
        assert_eq!(super::Mode::Stored.as_wire(), 0);
        assert_eq!(super::Mode::Idle.as_wire(), 1);
        assert_eq!(super::Mode::Tracking.as_wire(), 2);
        assert_eq!(super::Mode::Listening.as_wire(), 3);
        for m in [
            super::Mode::Stored,
            super::Mode::Idle,
            super::Mode::Tracking,
            super::Mode::Listening,
        ] {
            assert_eq!(super::Mode::from_wire(m.as_wire()), Some(m));
        }
        assert_eq!(super::Mode::from_wire(4), None);
        assert_eq!(super::Mode::default(), super::Mode::Stored);
    }

    /// Idle never reaches flash: a board that came back from a reset still
    /// believing it was idle would sit at awake current with nobody coming.
    /// The two commanded working modes do, so a brownout costs neither.
    #[test]
    fn idle_persists_as_stored() {
        assert_eq!(super::Mode::Idle.persisted(), super::Mode::Stored);
        assert_eq!(super::Mode::Stored.persisted(), super::Mode::Stored);
        assert_eq!(super::Mode::Tracking.persisted(), super::Mode::Tracking);
        assert_eq!(super::Mode::Listening.persisted(), super::Mode::Listening);
    }

    /// Listening raises what tracking raises and keys nothing up: it is
    /// the receiver half of a tracker, for the node beside the phone.
    #[test]
    fn listening_raises_the_radio_but_never_transmits() {
        assert!(super::Mode::Tracking.tracks());
        assert!(super::Mode::Listening.tracks());
        assert!(!super::Mode::Idle.tracks());
        assert!(!super::Mode::Stored.tracks());
        assert!(super::Mode::Tracking.transmits());
        assert!(!super::Mode::Listening.transmits());
        assert!(!super::Mode::Idle.transmits());
    }

    /// A named board is its label, an unnamed one is its address - and
    /// both start with the prefix a scanner is searched by.
    #[test]
    fn advertised_name_covers_named_and_unnamed() {
        // LSB-first, i.e. FF:C6:A1:53:50:47 as a scanner prints it.
        let addr = [0x47, 0x50, 0x53, 0xA1, 0xC6, 0xFF];
        let mut buf = [0u8; super::NAME_MAX];
        assert_eq!(super::advertised_name("sky-1", &addr, &mut buf), "ws3gps-sky-1");
        // The two bytes are the two the address ends with, in that order.
        assert_eq!(super::advertised_name("", &addr, &mut buf), "ws3gps-5047");
    }

    /// A rejected label does not leave the board nameless: the fallback is
    /// the same one an unnamed board gets, not an empty string.
    #[test]
    fn advertised_name_falls_back_on_a_bad_label() {
        let addr = [0x01, 0x02, 0, 0, 0, 0];
        let mut buf = [0u8; super::NAME_MAX];
        assert_eq!(super::advertised_name("has space", &addr, &mut buf), "ws3gps-0201");
    }

    /// The longest label a board stores still fits everything the name has
    /// to travel through.
    #[test]
    fn longest_name_fits_every_carrier() {
        let label = "x".repeat(super::NAME_LABEL_MAX);
        let mut buf = [0u8; super::NAME_MAX];
        let name = super::advertised_name(&label, &[0; 6], &mut buf);
        assert_eq!(name.len(), super::NAME_MAX);
        // A 31-byte scan response less the length and type bytes of the
        // complete-local-name AD structure.
        assert!(super::NAME_MAX <= 29);
        // trouble-host's GAP device name, which is a fixed-size string: a
        // name past it does not truncate, it fails the server build.
        assert!(super::NAME_MAX <= 22);
        // A full-length label still leaves the field zero-terminated.
        assert!(super::NAME_LABEL_MAX < super::NAME_FIELD_LEN);
    }

    #[test]
    fn labels_are_ascii_word_characters() {
        for good in ["a", "sky-1", "ground_2", "A1"] {
            assert!(super::valid_label(good.as_bytes()), "{good}");
        }
        for bad in ["", "has space", "quote\"", "caf\u{e9}", "a/b"] {
            assert!(!super::valid_label(bad.as_bytes()), "{bad}");
        }
        let too_long = "x".repeat(super::NAME_LABEL_MAX + 1);
        assert!(!super::valid_label(too_long.as_bytes()));
        assert!(super::valid_label(&too_long.as_bytes()[..super::NAME_LABEL_MAX]));
    }

    /// The config characteristic has to be able to carry the longest
    /// label, or a name write is truncated by the attribute layer before
    /// the policy ever sees it.
    #[test]
    fn config_write_max_holds_a_full_label() {
        assert_eq!(super::CONFIG_WRITE_MAX, 2 + super::NAME_LABEL_MAX);
        assert!(super::CONFIG_WRITE_MAX <= super::WRITE_MAX);
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
