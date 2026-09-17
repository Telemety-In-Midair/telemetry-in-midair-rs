//! What a BLE session does to the board: how a config write lands in the
//! settings, what the settings characteristic reports back, and when an
//! unattended board sleeps between advertising windows.
//!
//! These decisions live in the shared crate rather than inline in the
//! connection handler because they are the part where a mistake is
//! expensive: a board that mis-clamps an advertising window, or that lets a
//! retry restart its advertising budget, stops being reachable over the air
//! at all - and a `no_std` binary cannot be tested on the host.
//!
//! Everything here is pure. The firmware supplies the clock and performs
//! the effects ([`Action`]); nothing below touches a timer, flash or radio.
//!
//! The five durations an app can set - the wake-check cadence, the
//! advertising window, the modem's off and on periods, the idle timeout -
//! are described once, in [`KNOBS`]. That table is what says where each
//! one sits in the flash record and in the BLE settings blob, which config
//! id writes it, what its bounds are and what a stored zero means; the
//! record, the blob, the config write, the `[power]` section of the config
//! file and the firmware's RTC copy are all driven from it. Adding a
//! duration is one row and two struct fields.

use crate::ble::{self, Mode};
use crate::link;
use crate::lora;
use crate::posture::Request;
use gps_proto::packet;

// ---------------------------------------------------------------------------
// Settings that survive a sleep and a power cycle
// ---------------------------------------------------------------------------

// Bit 0 was the GPS/LoRa rail of the two-MCU board. Reserved.
/// The radio was asked into standby: an override inside a tracking
/// posture, honored whenever tracking is next commanded.
pub const PFLAG_RADIO_STANDBY: u32 = 1 << 1;
/// The GPS was asked to enter backup mode: the other override.
pub const PFLAG_GPS_SLEEP: u32 = 1 << 2;

/// One of the five durations.
///
/// The order is the order of [`KNOBS`], and the order the durations were
/// added to the record - which is what the record's version ladder reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Knob {
    /// Seconds between deep-sleep wake checks while stored, 0 = the board
    /// never stores itself on its own.
    SleepInterval,
    /// Seconds each wake check advertises for.
    AdvWindow,
    /// Seconds the modem stays down between windows while tracking, 0 =
    /// never take it down.
    BleOff,
    /// Seconds idle lasts before the board stores itself, 0 = never.
    IdleTimeout,
    /// Seconds the modem stays up between off periods while tracking.
    BleOn,
}

impl Knob {
    /// Every knob, in table order.
    pub const ALL: [Knob; 5] = [
        Knob::SleepInterval,
        Knob::AdvWindow,
        Knob::BleOff,
        Knob::IdleTimeout,
        Knob::BleOn,
    ];

    /// The knob's row of the table.
    pub fn spec(self) -> &'static KnobSpec {
        &KNOBS[self as usize]
    }

    /// The knob a config id writes, if it is one.
    pub fn from_id(id: u8) -> Option<Knob> {
        KNOBS.iter().find(|k| k.id == id).map(|k| k.knob)
    }

    /// The knob a config-file key names, if it is one.
    pub fn from_name(name: &str) -> Option<Knob> {
        KNOBS.iter().find(|k| k.name == name).map(|k| k.knob)
    }
}

/// What a stored zero means for a knob.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Zero {
    /// Zero is a setting - off - and is stored and read as zero.
    Off,
    /// Zero is "never configured": a write is clamped up to the floor so
    /// zero is never stored, and a record that holds one anyway reads as
    /// this default.
    Default(u32),
}

/// One row of the table: everything the code needs to know about a
/// duration, in one place.
pub struct KnobSpec {
    pub knob: Knob,
    /// The config-file key, which is also the name on the console.
    pub name: &'static str,
    /// The config characteristic id that writes it.
    pub id: u8,
    /// Bounds a non-zero value is clamped to.
    pub min: u32,
    pub max: u32,
    pub zero: Zero,
    /// The record version that appended it.
    pub since: u32,
    /// Byte offset in the flash record.
    pub record_at: usize,
    /// Byte offset in the BLE settings blob.
    pub wire_at: usize,
    /// One line for the config file and the app.
    pub doc: &'static str,
    get: fn(&Stored) -> u32,
    set: fn(&mut Stored, u32),
    wire_get: fn(&ble::Settings) -> u32,
    wire_set: fn(&mut ble::Settings, u32),
}

impl KnobSpec {
    /// The value a write of `asked` stores.
    pub fn clamp(&self, asked: u32) -> u32 {
        match self.zero {
            Zero::Off if asked == 0 => 0,
            _ => asked.clamp(self.min, self.max),
        }
    }

    /// The value a stored `raw` means.
    pub fn resolve(&self, raw: u32) -> u32 {
        match (self.zero, raw) {
            (Zero::Default(d), 0) => d,
            _ => raw,
        }
    }

    /// Whether `v` is a value a config file may carry: zero, or inside the
    /// bounds. A file is read once and edited by hand, so anything else is
    /// reported rather than clamped.
    pub fn accepts(&self, v: u64) -> bool {
        v == 0 || (u64::from(self.min)..=u64::from(self.max)).contains(&v)
    }

    pub fn get(&self, s: &Stored) -> u32 {
        (self.get)(s)
    }

    pub fn set(&self, s: &mut Stored, v: u32) {
        (self.set)(s, v)
    }

    pub fn wire_get(&self, w: &ble::Settings) -> u32 {
        (self.wire_get)(w)
    }

    pub fn wire_set(&self, w: &mut ble::Settings, v: u32) {
        (self.wire_set)(w, v)
    }
}

/// Where the BLE on period sits in the record: after the name, which is
/// where version 6 ended.
const BLE_ON_AT: usize = 32 + ble::NAME_FIELD_LEN;

/// The five durations.
pub const KNOBS: [KnobSpec; 5] = [
    KnobSpec {
        knob: Knob::SleepInterval,
        name: "sleep_interval_s",
        id: ble::CFG_ESP_SLEEP_S,
        min: ble::ESP_SLEEP_MIN_S,
        max: ble::ESP_SLEEP_MAX_S,
        zero: Zero::Off,
        since: 2,
        record_at: 8,
        wire_at: 4,
        doc: "Seconds between wake checks while the board is stored, 0 or 5-3600. 0 means the board never stores itself on its own: with no cadence to sleep on, the idle timeout has nowhere to send it and it stays awake and reachable. That is the bench setting. A board explicitly told to store itself still sleeps, on a 300 s default. Deep sleep takes the whole chip down rather than just the BLE controller, so it is the larger saving and the larger cost: the board stops beaconing, stops logging, and every wake is a full reset. With wake_enabled on in the radio config the radio keeps listening for a wake frame through the sleep, so this cadence is a backstop and an hour is a fine value; with it off, this is the only way back in. Ignored while tracking - a tracker that deep-sleeps is not tracking.",
        get: |s| s.sleep_interval_s,
        set: |s, v| s.sleep_interval_s = v,
        wire_get: |w| w.sleep_interval_s,
        wire_set: |w, v| w.sleep_interval_s = v,
    },
    KnobSpec {
        knob: Knob::AdvWindow,
        name: "adv_window_s",
        id: ble::CFG_ESP_ADV_WINDOW_S,
        min: ble::ESP_ADV_MIN_S,
        max: ble::ESP_ADV_MAX_S,
        zero: Zero::Default(ble::ESP_ADV_DEFAULT_S),
        since: 3,
        record_at: 16,
        wire_at: 12,
        doc: "Seconds each wake check advertises, 0 or 1-60. 0 means the firmware default of 15 s. This is the whole of the time a stored board is reachable, so it and sleep_interval_s together are what a phone has to catch. With sleep_interval_s at 0 the board advertises forever and the window never ends.",
        get: |s| s.adv_window_s,
        set: |s, v| s.adv_window_s = v,
        wire_get: |w| w.adv_window_s,
        wire_set: |w, v| w.adv_window_s = v,
    },
    KnobSpec {
        knob: Knob::BleOff,
        name: "ble_off_s",
        id: ble::CFG_BLE_OFF_S,
        min: ble::BLE_OFF_MIN_S,
        max: ble::BLE_OFF_MAX_S,
        zero: Zero::Off,
        since: 4,
        record_at: 20,
        wire_at: 16,
        doc: "Seconds the BLE controller is powered down between advertising windows, 0 or 5-300. This is the biggest single saving the firmware has: the controller is about 71 mA of the board's 126 and nothing but ending its lifetime reduces it, so the board drops to about 60 mA for this long and is unreachable over BLE while it does. LoRa, GPS and logging keep running throughout, so it is still a working tracker. 0 keeps BLE up continuously. Read while tracking only.",
        get: |s| s.ble_off_s,
        set: |s, v| s.ble_off_s = v,
        wire_get: |w| w.ble_off_s,
        wire_set: |w, v| w.ble_off_s = v,
    },
    KnobSpec {
        knob: Knob::IdleTimeout,
        name: "idle_timeout_s",
        id: ble::CFG_IDLE_TIMEOUT_S,
        min: ble::IDLE_TIMEOUT_MIN_S,
        max: ble::IDLE_TIMEOUT_MAX_S,
        zero: Zero::Off,
        since: 5,
        record_at: 28,
        wire_at: 20,
        doc: "Seconds the board stays reachable-but-not-tracking before it stores itself, 0 or 10-3600. 0 is the default and means it never does: an idle board stays idle until it is told otherwise. Idle is the expensive state - BLE dominates it at around 90 mA - so set this for a board that could be forgotten idle. It also needs sleep_interval_s: with no cadence to wake on the board cannot store itself and stays reachable.",
        get: |s| s.idle_timeout_s,
        set: |s, v| s.idle_timeout_s = v,
        wire_get: |w| w.idle_timeout_s,
        wire_set: |w, v| w.idle_timeout_s = v,
    },
    KnobSpec {
        knob: Knob::BleOn,
        name: "ble_on_s",
        id: ble::CFG_BLE_ON_S,
        min: ble::BLE_ON_MIN_S,
        max: ble::BLE_ON_MAX_S,
        zero: Zero::Default(ble::BLE_ON_DEFAULT_S),
        since: 7,
        record_at: BLE_ON_AT,
        wire_at: 24,
        doc: "Seconds BLE stays up between off periods while tracking, 0 or 1-60. 0 means the firmware default of 15 s. The on-half of the ble_off_s duty cycle: long enough for a phone to connect, read the roster and let go. Only read while tracking, and only with ble_off_s set.",
        get: |s| s.ble_on_s,
        set: |s, v| s.ble_on_s = v,
        wire_get: |w| w.ble_on_s,
        wire_set: |w, v| w.ble_on_s = v,
    },
];

/// Everything a config write can change that outlives the connection, in
/// the words the firmware keeps in RTC RAM and mirrors to flash.
///
/// The notify interval is deliberately not here: it is per-session state
/// that resets with the board (see [`Action::NotifyInterval`]).
///
/// The five durations are read through [`Stored::knob`], which resolves a
/// stored zero the way its [`KnobSpec`] says; the fields hold the raw
/// stored values.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Stored {
    pub sleep_interval_s: u32,
    /// `PFLAG_*` bits.
    pub flags: u32,
    pub adv_window_s: u32,
    pub ble_off_s: u32,
    /// What the board is doing, and - once [`Mode::persisted`] has had it -
    /// what it comes back doing after a flat cell.
    ///
    /// The RTC copy holds the live mode, [`Mode::Idle`] included; only
    /// [`Stored::encode_record`] normalizes it, so flash never says idle.
    pub mode: Mode,
    pub idle_timeout_s: u32,
    pub ble_on_s: u32,
    /// What the board is called ([`ble::CFG_NAME`]), zero-padded ASCII.
    ///
    /// All-zero is a board that has never been named, which advertises
    /// under its address instead - so this is the one field whose "never
    /// configured" value is visible to anyone holding a phone. Read it
    /// through [`Stored::label`], which is where the padding stops.
    ///
    /// It sits here rather than with the radio config because of when it is
    /// needed: a wake check advertises before anything has mounted the
    /// card, so a name kept on the card would be a name a sleeping board
    /// could not tell anyone.
    pub name: [u8; ble::NAME_FIELD_LEN],
}

impl Stored {
    pub const fn new() -> Self {
        Self {
            sleep_interval_s: 0,
            flags: 0,
            adv_window_s: 0,
            ble_off_s: 0,
            // The resting mode, not the one a board comes up in: a cold
            // boot turns this into [`Mode::Idle`] so a board that has just
            // been flashed, or has just come back from a flat cell, is
            // reachable before it stores itself. See [`boot_mode`].
            mode: Mode::Stored,
            idle_timeout_s: 0,
            ble_on_s: 0,
            name: [0; ble::NAME_FIELD_LEN],
        }
    }

    /// The stored label, or `""` for a board that has never been named.
    ///
    /// Padding stops at the first zero byte. Anything that is not a valid
    /// label reads as unnamed rather than as itself: the field is written
    /// by [`Stored::set_label`], which validates, so a value that fails
    /// here came from corrupted RTC RAM or a truncated record, and an
    /// address-derived name is the honest answer to that.
    pub fn label(&self) -> &str {
        let end = self
            .name
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(ble::NAME_LABEL_MAX);
        match core::str::from_utf8(&self.name[..end]) {
            Ok(s) if ble::valid_label(s.as_bytes()) => s,
            _ => "",
        }
    }

    /// Store a label, or clear it with an empty one. Returns whether the
    /// value was accepted; a rejected write leaves the old name in place.
    pub fn set_label(&mut self, label: &[u8]) -> bool {
        if !label.is_empty() && !ble::valid_label(label) {
            return false;
        }
        self.name = [0; ble::NAME_FIELD_LEN];
        self.name[..label.len()].copy_from_slice(label);
        true
    }

    pub fn radio_standby(&self) -> bool {
        self.flags & PFLAG_RADIO_STANDBY != 0
    }

    pub fn gps_sleep(&self) -> bool {
        self.flags & PFLAG_GPS_SLEEP != 0
    }

    /// A duration as the board uses it: a stored zero resolved the way the
    /// knob's row says.
    pub fn knob(&self, knob: Knob) -> u32 {
        let spec = knob.spec();
        spec.resolve(spec.get(self))
    }

    /// A duration as stored, zero included.
    pub fn knob_raw(&self, knob: Knob) -> u32 {
        knob.spec().get(self)
    }

    /// Store a duration, already clamped by [`KnobSpec::clamp`] or checked
    /// by [`KnobSpec::accepts`].
    pub fn set_knob(&mut self, knob: Knob, v: u32) {
        knob.spec().set(self, v)
    }

    /// How long a wake check advertises for. A stored 0 means never
    /// configured, not "do not advertise" - a zero window would leave a
    /// sleeping board unreachable by anything but a physical reset, so it
    /// resolves to the default instead.
    pub fn adv_window(&self) -> u32 {
        self.knob(Knob::AdvWindow)
    }

    /// How long [`Mode::Idle`] runs before the board stores itself, 0 for
    /// a board that never does. Unlike the advertising window a 0 here is
    /// a real setting and the default one: an idle board stays idle until
    /// something tells it otherwise, so nobody finds a board that stored
    /// itself while they were still setting it up.
    ///
    /// A non-zero timeout also needs a cadence: a board leaves Idle by
    /// deep-sleeping, and with `sleep_interval_s` at 0 there is nowhere for
    /// the timeout to send it, so it stays awake and reachable whatever
    /// this says.
    pub fn idle_timeout(&self) -> u32 {
        self.knob(Knob::IdleTimeout)
    }

    /// How long BLE stays up between off periods while tracking. A stored
    /// 0 means never configured and resolves to the default, exactly as the
    /// advertising window does and for the same reason: a zero-length on
    /// period is a tracker nobody can connect to.
    pub fn ble_on(&self) -> u32 {
        self.knob(Knob::BleOn)
    }

    /// The cadence a board that has been *told* to store itself sleeps on.
    ///
    /// `sleep_interval_s` when it is set. When it is not, an explicit
    /// `CFG_MODE stored` is still an unambiguous instruction - the same
    /// reading [`ble::resolve_sleep_now`] gives a commanded nap - so it
    /// borrows a default rather than being ignored. The passive path (an
    /// idle timeout) does not do this: nobody asked for that one, so a
    /// board with deep sleep off simply stays awake.
    pub fn sleep_cadence(&self) -> u32 {
        match self.sleep_interval_s {
            0 => ble::STORE_DEFAULT_S,
            s => s,
        }
    }

    /// How long the board advertises before [`Stored::at_expiry`] decides
    /// something, which is a different budget in each mode.
    ///
    /// A wake check gets the advertising window - long enough for a phone
    /// to catch it, short enough to be paid for every interval forever.
    /// Idle gets the whole idle timeout, because being reachable is the
    /// entire point of the state. Tracking gets the BLE on period, the
    /// on-half of the `ble_off_s` duty cycle.
    ///
    /// A mode whose budget never bites - listening, or idle with the
    /// timeout off - is given the advertising window to fill the field, and
    /// [`Stored::at_expiry`] is what says the number is never acted on.
    pub fn budget_s(&self) -> u32 {
        match self.mode {
            Mode::Stored | Mode::Listening => self.adv_window(),
            Mode::Tracking => self.ble_on(),
            Mode::Idle => match self.idle_timeout() {
                0 => self.adv_window(),
                s => s,
            },
        }
    }

    /// What happens when that budget runs out.
    ///
    /// Which duty cycle applies is a property of the mode rather than a
    /// race between two settings: before the mode existed both were tested
    /// in one place, deep sleep always won, and `ble_off_s` was dead config
    /// on any board that had a wake-check cadence.
    pub fn at_expiry(&self) -> Next {
        match self.mode {
            // A wake check always goes back down. The board is in this mode
            // because something put it here, so a cadence it does not have
            // is borrowed rather than treated as a refusal - otherwise a
            // board stored without one would wake once and then sit at
            // awake current forever, which is the opposite of what was
            // asked for.
            Mode::Stored => Next::Sleep {
                interval_s: self.sleep_cadence(),
            },
            // Idle is the passive path, and nobody asked for this one: with
            // the timeout off, or no cadence to sleep on, the board simply
            // keeps advertising, which is what a bench board and an
            // unconfigured board both want.
            Mode::Idle => match (self.idle_timeout(), self.sleep_interval_s) {
                (0, _) | (_, 0) => Next::FOREVER,
                (_, interval_s) => Next::Sleep { interval_s },
            },
            // A tracker never deep-sleeps on a cadence - that would stop
            // the beacon, the logging and the listening, which is the whole
            // job. What it can drop is the modem.
            Mode::Tracking => match self.ble_off_s {
                0 => Next::FOREVER,
                off_s => Next::BleDown { off_s },
            },
            // The node beside the phone exists to be connected to, so it
            // duty-cycles nothing: the phone has to be able to come back
            // whenever it likes.
            Mode::Listening => Next::FOREVER,
        }
    }

    fn set_flag(&mut self, flag: u32, on: bool) {
        if on {
            self.flags |= flag;
        } else {
            self.flags &= !flag;
        }
    }

    /// The settings characteristic value, which is the only way an app
    /// learns the board's current state on connect. Every duration goes out
    /// resolved, so an app never sees a "never configured" zero.
    pub fn settings(&self, notify_interval_ms: u32) -> ble::Settings {
        let mut w = ble::Settings {
            radio_standby: self.radio_standby(),
            gps_sleep: self.gps_sleep(),
            notify_interval_ms,
            mode: self.mode,
            ..ble::Settings::default()
        };
        for spec in &KNOBS {
            spec.wire_set(&mut w, self.knob(spec.knob));
        }
        w
    }

    /// Fold a config file's `[power]` section into these settings, and say
    /// whether that changed anything.
    ///
    /// Only the keys the file actually carried are applied - an absent one
    /// leaves the board's live value alone, which is the whole reason
    /// [`crate::radiocfg::PowerConfig`] holds an option per knob. The
    /// parser has already range-checked whatever is present, so there is
    /// nothing to clamp here.
    ///
    /// The return value is what decides whether the caller pays for a flash
    /// write: a board that reads the same card on every boot would otherwise
    /// rewrite the same record forever.
    pub fn adopt_power(&mut self, p: &crate::radiocfg::PowerConfig) -> bool {
        let before = *self;
        for spec in &KNOBS {
            if let Some(v) = p.get(spec.knob) {
                spec.set(self, v);
            }
        }
        *self != before
    }

    /// Encode the flash record. Erased-flash bytes past the end are none of
    /// this function's business; the crc covers what it wrote.
    pub fn encode_record(&self) -> [u8; RECORD_LEN] {
        let mut rec = [0u8; RECORD_LEN];
        rec[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&RECORD_VERSION.to_le_bytes());
        for spec in &KNOBS {
            let at = spec.record_at;
            rec[at..at + 4].copy_from_slice(&spec.get(self).to_le_bytes());
        }
        rec[12..16].copy_from_slice(&self.flags.to_le_bytes());
        // Normalized on the way out, and only here: the RTC copy may say
        // idle, but a board that came back from a reset still believing it
        // was idle would sit at awake current with nobody coming.
        rec[24..28].copy_from_slice(&(self.mode.persisted().as_wire() as u32).to_le_bytes());
        rec[32..32 + ble::NAME_FIELD_LEN].copy_from_slice(&self.name);
        let crc = link::crc32(&rec[0..RECORD_LEN - 4]);
        rec[RECORD_LEN - 4..RECORD_LEN].copy_from_slice(&crc.to_le_bytes());
        rec
    }

    /// Read a flash record back. `None` for anything that is not one: a
    /// short read, erased flash, another program's bytes, a version this
    /// build does not understand, or a record the crc rejects. The board
    /// then comes up unconfigured rather than acting on garbage.
    pub fn decode_record(rec: &[u8]) -> Option<Self> {
        if rec.len() < RECORD_LEN {
            return None;
        }
        let word = |i: usize| u32::from_le_bytes([rec[i], rec[i + 1], rec[i + 2], rec[i + 3]]);
        if word(0) != RECORD_MAGIC {
            return None;
        }
        // Every older version stops short and carries zeros for the fields
        // it predates, which read back as "never configured" and resolve to
        // defaults. Their trailing bytes are erased flash, so the crc has to
        // be checked where each version put it rather than where this one
        // does.
        let version = word(4);
        let crc_at = record_crc_at(version)?;
        if word(crc_at) != link::crc32(&rec[0..crc_at]) {
            return None;
        }
        let mut s = Self::new();
        for spec in &KNOBS {
            if spec.since <= version {
                spec.set(&mut s, word(spec.record_at));
            }
        }
        s.flags = word(12);
        // A mode this build does not know reads as the safe one: a board
        // that stores itself can be woken, where one that guessed at
        // tracking would run its cell down.
        if version >= 5 {
            s.mode = Mode::from_wire(word(24) as u8).unwrap_or_default();
        }
        if version >= 6 {
            s.name.copy_from_slice(&rec[32..32 + ble::NAME_FIELD_LEN]);
        }
        Some(s)
    }
}

/// Marks the record as this firmware's ("midA").
pub const RECORD_MAGIC: u32 = 0x6D69_6441;

/// Layout version of the flash record.
///
/// Version 2 dropped a separate stow interval; a version 1 record is
/// discarded rather than misread. Every version since has been a pure
/// append - the advertising window (3), the BLE off period (4), the mode
/// and its idle timeout (5), the name (6), the BLE on period (7) - so an
/// older record still reads: a board updated in the field keeps the cadence
/// it was left on instead of coming back advertising continuously. Which
/// knob a version carries is the `since` column of [`KNOBS`].
///
/// A record from before version 5 carries no mode, which reads as
/// [`Mode::Stored`]: an updated board comes back reachable (a cold boot
/// lands in [`Mode::Idle`] whatever the record says) and then stores itself
/// on the cadence it already had. A version 6 record also carries an idle
/// timeout that meant "the default" when it was 0 and means "off" now. That
/// is the change wanted: a board updated in the field stops storing itself
/// out of idle unless somebody had set a timeout.
pub const RECORD_VERSION: u32 = 7;

/// magic, version, sleep interval, flags, advertising window, BLE off
/// period, mode, idle timeout, name, BLE on period, crc32. Every field is a
/// `u32` or a multiple of one, so the length is already a multiple of the
/// flash write word.
pub const RECORD_LEN: usize = BLE_ON_AT + 8;

/// Where the crc sits in a record of `version`: the length that version's
/// layout had before it, or `None` for a version this build cannot read.
fn record_crc_at(version: u32) -> Option<usize> {
    Some(match version {
        2 => 16,
        3 => 20,
        4 => 24,
        5 => 32,
        6 => BLE_ON_AT,
        RECORD_VERSION => RECORD_LEN - 4,
        _ => return None,
    })
}

// ---------------------------------------------------------------------------
// Config characteristic writes
// ---------------------------------------------------------------------------

/// What the firmware still has to do once [`apply`] has folded a config
/// write into the settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Put the radio into standby (`true`) or bring it back.
    RadioStandby(bool),
    /// Put the GPS into (`true`) or out of backup mode.
    GpsSleep(bool),
    /// A duration is now this many seconds. Already stored and clamped;
    /// the loops read it when they next decide, so it takes effect at the
    /// next window, wake or re-arm rather than shortening the one running.
    Knob(Knob, u32),
    /// Deep sleep now, for this many seconds, then resume as configured.
    ///
    /// Unlike every other variant here this is a command rather than a
    /// settings change: nothing was stored, so nothing survives the sleep
    /// but the cadence the board already had. The firmware must get the
    /// ack out before it acts - the link does not survive the action.
    SleepNow(u32),
    /// Set the position notify interval, in ms, already clamped. Not
    /// persisted: it is per-session state.
    NotifyInterval(u32),
    /// Put the board into this mode - raise the tracker, lower it to idle,
    /// or store the board.
    ///
    /// Already folded into the settings, so the mode an app reads back is
    /// the one it asked for. [`Mode::Stored`] is a command like
    /// [`Action::SleepNow`] and has the same requirement: the ack must
    /// leave before the board acts, because the link does not survive it.
    SetMode(Mode),
    /// The board has been renamed - or, with an empty label, un-named and
    /// back to advertising under its address. Already stored.
    ///
    /// The firmware republishes the name characteristic on the strength of
    /// this, because the ack cannot carry the name: an ack has four value
    /// bytes and this one spends them on the stored length. The scan
    /// response catches up at the next advertising window, since the one
    /// running was handed to the controller before the write arrived.
    Name,
    /// Call `target` over LoRa: a burst of wake frames on its sentry
    /// preamble, ending at its answer. `tracking` asks it to come up
    /// tracking rather than idle. Nothing stored.
    WakeNode { target: u8, tracking: bool },
    /// Nothing to do - the write was rejected, and the ack says why.
    None,
}

/// The result of one write to the config characteristic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome {
    /// What the firmware must carry out.
    pub action: Action,
    /// Ack to notify back on the ack characteristic. Always
    /// [`packet::ACK_MAX_LEN`] bytes so it can be handed straight to a
    /// fixed-size characteristic; `ack_len` is what carries meaning.
    pub ack: [u8; packet::ACK_MAX_LEN],
    pub ack_len: usize,
    /// Whether the change has to reach flash to mean anything.
    ///
    /// Set for the settings that decide whether a board comes back at all
    /// after its battery goes flat - the durations, the mode, the name. The
    /// two override flags are not saved: they are re-applied on the next
    /// tracking command, and a board that cold-boots with its GPS running
    /// is the safer of the two failures.
    pub save: bool,
}

impl Outcome {
    /// The meaningful ack bytes.
    pub fn ack(&self) -> &[u8] {
        &self.ack[..self.ack_len]
    }

    fn new(action: Action, save: bool, id: u8, status: u8, value: &[u8]) -> Self {
        let (ack, ack_len) = packet::encode_ack(id, status, value);
        Self {
            action,
            ack,
            ack_len,
            save,
        }
    }

    fn reject(id: u8, status: u8) -> Self {
        Self::new(Action::None, false, id, status, &[])
    }
}

/// Apply one config characteristic write to the stored settings.
///
/// Board-specific ids are handled here; anything else falls through to the
/// gps-proto config protocol, so the app that predates this board keeps
/// working against it. Writes are applied before the hardware has moved -
/// the flag is set, the request is queued - because the settings
/// characteristic has to report what the app asked for, and the hardware
/// loop answers on its next pass rather than in this call.
pub fn apply(stored: &mut Stored, data: &[u8]) -> Outcome {
    if data.len() >= 2 {
        let id = data[0];
        let len = data[1] as usize;
        // A length running past the write leaves the value empty, which the
        // per-id checks below reject; only the flag ids default it.
        let value = data.get(2..2 + len).unwrap_or(&[]);
        if let Some(knob) = Knob::from_id(id) {
            let Ok(bytes) = <[u8; 4]>::try_from(value) else {
                return Outcome::reject(id, packet::ACK_BAD_VALUE);
            };
            // Clamped the way the knob's row says - a zero kept as "off"
            // for the knobs that have an off, brought up to the floor for
            // the ones a zero would leave unreachable - and echoed as
            // applied, so the app displays the effective setting rather
            // than the request.
            let secs = knob.spec().clamp(u32::from_le_bytes(bytes));
            stored.set_knob(knob, secs);
            return Outcome::new(
                Action::Knob(knob, secs),
                true,
                id,
                packet::ACK_OK,
                &secs.to_le_bytes(),
            );
        }
        match id {
            ble::CFG_RADIO_STANDBY => {
                let standby = value.first().copied().unwrap_or(0) != 0;
                stored.set_flag(PFLAG_RADIO_STANDBY, standby);
                return Outcome::new(
                    Action::RadioStandby(standby),
                    false,
                    id,
                    packet::ACK_OK,
                    &[standby as u8],
                );
            }
            ble::CFG_GPS_SLEEP => {
                let sleep = value.first().copied().unwrap_or(0) != 0;
                stored.set_flag(PFLAG_GPS_SLEEP, sleep);
                return Outcome::new(
                    Action::GpsSleep(sleep),
                    false,
                    id,
                    packet::ACK_OK,
                    &[sleep as u8],
                );
            }
            ble::CFG_MODE => {
                // Rejected rather than rounded: every value is a different
                // amount of the board switched off, and a write this
                // firmware does not understand is one it must not guess at.
                let Some(mode) = value.first().copied().and_then(Mode::from_wire) else {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                };
                stored.mode = mode;
                return Outcome::new(
                    // Saved, because this is the setting that decides what a
                    // board does when it comes back from a flat cell.
                    // `encode_record` is where idle stops being idle.
                    Action::SetMode(mode),
                    true,
                    id,
                    packet::ACK_OK,
                    &[mode.as_wire()],
                );
            }
            ble::CFG_NAME => {
                // An empty value clears the name; anything else has to be a
                // label the board can advertise. A rejected write leaves
                // the old name alone rather than half-applying one, so a
                // board never ends up between two names.
                if !stored.set_label(value) {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                }
                return Outcome::new(
                    Action::Name,
                    true,
                    id,
                    packet::ACK_OK,
                    &[value.len() as u8],
                );
            }
            ble::CFG_WAKE => {
                let Some(&target) = value.first() else {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                };
                let flags = value.get(1).copied().unwrap_or(0);
                let tracking = flags & lora::WAKE_FLAG_TRACKING != 0;
                return Outcome::new(
                    Action::WakeNode { target, tracking },
                    false,
                    id,
                    packet::ACK_OK,
                    &[target, flags],
                );
            }
            ble::CFG_SLEEP_NOW => {
                let Ok(bytes) = <[u8; 4]>::try_from(value) else {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                };
                // Deliberately leaves `stored` alone. A nap is not a
                // cadence, and a board told to sleep once must come back
                // to whatever it was doing - including advertising
                // continuously, if that is what it was configured for.
                let secs = ble::resolve_sleep_now(
                    u32::from_le_bytes(bytes),
                    stored.sleep_interval_s,
                );
                return Outcome::new(
                    Action::SleepNow(secs),
                    false,
                    id,
                    packet::ACK_OK,
                    &secs.to_le_bytes(),
                );
            }
            _ => {}
        }
    }

    match packet::parse_config(data) {
        Ok(packet::ConfigCommand::UpdateIntervalMs(ms)) => {
            let applied = packet::clamp_interval(ms);
            Outcome::new(
                Action::NotifyInterval(applied),
                false,
                packet::CFG_UPDATE_INTERVAL_MS,
                packet::ACK_OK,
                &applied.to_le_bytes(),
            )
        }
        Err(status) => Outcome::reject(data.first().copied().unwrap_or(0), status),
    }
}

// ---------------------------------------------------------------------------
// The wake / advertise / sleep cycle
// ---------------------------------------------------------------------------

/// How long the board keeps advertising after a central disconnects, before
/// the sleep-mode budget runs out.
///
/// Spent advertising rather than merely awake: the point is to let a phone
/// come straight back, which it cannot do if the board is up but not
/// discoverable.
pub const LINGER_S: u64 = 5;

/// Which mode this boot comes up in, from the persisted mode and the wake
/// cause.
///
/// Three flavors, and the difference between them is what the boot raises:
/// a wake check raises nothing, idle raises BLE and the card, tracking
/// raises everything.
///
/// The inversion worth noticing is the cold boot. The sleep flags were
/// deliberately not mirrored to flash because "a board that cold-boots with
/// its GPS running is the safer failure" - true for a tracker, and it
/// drains the cell of a device in a bag. Landing a cold boot in
/// [`Mode::Idle`] is safe in both senses: reachable, and with the GPS down.
/// Only an explicit stored `tracking` raises everything.
pub fn boot_mode(persisted: Mode, woke_from_sleep: bool) -> Mode {
    match (persisted, woke_from_sleep) {
        // Tracking survives the reset, whichever kind it was. A brownout on
        // the object is the one time it must. Listening likewise: the node
        // beside the phone must not go dark on it over a reset.
        (Mode::Tracking, _) => Mode::Tracking,
        (Mode::Listening, _) => Mode::Listening,
        // A timer wake with anything else stored is a wake check: raise
        // nothing, ask whether anyone wants the board, go back down.
        (_, true) => Mode::Stored,
        // Any other cold boot gets a rescue window - a board just flashed,
        // or one that came back from a flat cell, is reachable for an idle
        // timeout before it stores itself.
        (_, false) => Mode::Idle,
    }
}

/// What the board should do right now.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Next {
    /// Keep advertising. `bounded` says the budget ends in something, so
    /// the wait for a central is cut at [`Window::remaining_ms`]; unbounded
    /// is a board that advertises forever, and then the wait has no
    /// deadline.
    Advertise { bounded: bool },
    /// Deep sleep for this many seconds, which is the board going to
    /// [`Mode::Stored`].
    Sleep { interval_s: u32 },
    /// Take the BLE modem down for this many seconds and come back.
    ///
    /// [`Mode::Tracking`] only. Costs no reset and stops nothing else: the
    /// beacon keeps transmitting, the GPS keeps tracking and the card keeps
    /// logging. What it costs is reachability.
    BleDown { off_s: u32 },
}

impl Next {
    /// Advertise with no end: what a spent budget resolves to in a mode
    /// that has no duty cycle.
    pub const FOREVER: Next = Next::Advertise { bounded: false };

    /// Whether this is advertising, bounded or not.
    pub fn advertises(self) -> bool {
        matches!(self, Next::Advertise { .. })
    }
}

/// The advertising budget for one wake, held as a deadline rather than as a
/// per-attempt timeout.
///
/// Every path that restarts advertising - a failed advertise, a connection
/// attempt that errors out, a server that will not attach - asks this again,
/// and none of them may extend the window. Timing each attempt separately
/// let a central that kept failing to connect reset the budget forever, so
/// the board woke, drew its full advertising current and never slept again.
/// A deadline cannot be restarted by a retry; only a disconnect
/// ([`Window::linger`]) sets a new one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Window {
    ends_ms: u64,
}

impl Window {
    /// Start the budget for a wake that begins now.
    ///
    /// The window length is sampled once for the wake rather than read per
    /// attempt, so a window shortened over BLE takes effect on the next
    /// wake and cannot retroactively strand a board mid-window with its
    /// budget already spent.
    pub fn new(now_ms: u64, adv_window_s: u32) -> Self {
        Self {
            ends_ms: now_ms.saturating_add(adv_window_s as u64 * 1000),
        }
    }

    /// When the budget runs out, on the caller's monotonic millisecond
    /// clock.
    pub fn ends_ms(&self) -> u64 {
        self.ends_ms
    }

    /// How much of the budget is left, 0 once it is spent.
    pub fn remaining_ms(&self, now_ms: u64) -> u64 {
        self.ends_ms.saturating_sub(now_ms)
    }

    /// Advertise, or do whatever this mode does with a spent budget.
    ///
    /// The settings are passed in per call rather than captured, so a
    /// cadence or a mode changed over BLE takes effect at the next decision
    /// instead of at the next boot. Derived from `at_expiry` rather than
    /// from the settings directly, so a spent budget that resolves to "keep
    /// advertising" cannot arm a zero-length wait and spin.
    pub fn next(&self, now_ms: u64, stored: &Stored) -> Next {
        let at_expiry = stored.at_expiry();
        if self.remaining_ms(now_ms) > 0 {
            Next::Advertise {
                bounded: !at_expiry.advertises(),
            }
        } else {
            at_expiry
        }
    }

    /// Give the phone that just disconnected a chance to come straight
    /// back.
    pub fn linger(&mut self, now_ms: u64) {
        self.ends_ms = now_ms.saturating_add(LINGER_S * 1000);
    }

    /// A central tried to connect and the handshake did not complete.
    ///
    /// Extends rather than sets, which is the difference from
    /// [`linger`](Self::linger): an attempt early in a long window must not
    /// shorten it.
    ///
    /// Phones fizzle a first handshake routinely - the app's own reconnect
    /// loop exists for it - and the retry lands a second or two later.
    /// Without this the window can expire in between, and then the retry
    /// arrives at a board that has gone dark for `ble_off_s` or asleep for
    /// a whole cadence. From the hand holding the phone that is not "it
    /// will connect in a moment", it is "it did not connect", and the wait
    /// is as long as the duty cycle rather than as long as a handshake.
    ///
    /// [`Mode::Stored`] reaches this too, after the promotion has already
    /// re-armed the window on the idle budget: the extension is a `max`, so
    /// it changes nothing there.
    pub fn after_connect_attempt(&mut self, now_ms: u64) {
        self.ends_ms = self
            .ends_ms
            .max(now_ms.saturating_add(LINGER_S * 1000));
    }

    /// Re-arm the budget after a central disconnects.
    ///
    /// [`Mode::Idle`] restarts the whole timeout rather than spending what
    /// is left of it: the state exists to be reachable, and a phone that
    /// has just been talking to the board is the best evidence anyone has
    /// that someone still wants it. Every other mode gets the linger, which
    /// is spent advertising rather than merely awake - the point is to let
    /// the phone come straight back, which it cannot do if the board is up
    /// but not discoverable.
    pub fn after_disconnect(&mut self, now_ms: u64, stored: &Stored) {
        match stored.mode {
            Mode::Idle => *self = Window::new(now_ms, stored.budget_s()),
            _ => self.linger(now_ms),
        }
    }
}

// ---------------------------------------------------------------------------
// The serve loop's policy
// ---------------------------------------------------------------------------

/// A command for the serve loop, from a config write on either transport.
///
/// One channel with one consumer, whichever wait the loop is in - the wait
/// for a central, the connected session, the modem's off period. The loop
/// owns the advertising and the `Rtc`, so anything that ends a wait early
/// has to reach it here.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ServeCommand {
    /// Deep sleep now, for this long, once whatever is owed to a connected
    /// central has left. Already resolved and clamped.
    SleepNow(u32),
    /// The mode moved: a wait that belongs to the old mode's budget should
    /// end, and the next pass re-budgets on the new one.
    ModeChanged,
}

/// How waiting for a central ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Accepted {
    /// A central connected and the attribute server attached.
    Connected,
    /// A central started a connection and it did not complete, or the
    /// attribute server would not attach to one that did.
    Failed,
    /// The budget ran out with nobody interested.
    Expired,
    /// A command arrived while waiting.
    Command(ServeCommand),
}

/// What the firmware does after [`Serve::on_accept`] or
/// [`Serve::on_session_end`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Then {
    /// Run the connected session.
    Serve,
    /// Pause briefly, then advertise again.
    Retry,
    /// Deep sleep for this many seconds. Does not return.
    Sleep(u32),
    /// Drop the BLE stack for this many seconds and come back.
    BleDown(u32),
    /// Go straight back to the top of the loop.
    Continue,
}

/// What [`Serve::on_accept`] decided: whether the connect attempt promoted
/// a wake check to idle, and what to do next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Step {
    /// The board was a wake check and someone tried to connect, so it is
    /// now idle with the idle budget. The firmware records the mode and
    /// asks the hardware loop to raise what idle raises.
    pub promote: bool,
    pub then: Then,
}

/// The serve loop: advertise, accept one central, serve it, repeat, on a
/// budget the mode decides.
///
/// This is the policy half of the loop the firmware runs. The firmware
/// supplies the clock, the advertising and the connection; this decides
/// what each outcome means, and it is what the state space tests walk -
/// with every mode, every setting and every ordering of connects, drops,
/// commands and expiries - to show that the board is never left
/// unreachable.
///
/// What a spent budget means is a property of the mode, not a race between
/// two settings ([`Stored::at_expiry`]): a wake check sleeps again, idle
/// stores the board, tracking drops the modem for a while, listening
/// never expires. The budget is a deadline rather than a per-attempt
/// timeout, so no retry path can extend it ([`Window`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Serve {
    /// The mode the window was budgeted for. The live mode can move under
    /// the loop; the next pass notices and re-budgets.
    mode: Mode,
    window: Window,
}

impl Serve {
    /// Begin serving at `now_ms`, on the budget of the stored mode.
    pub fn new(now_ms: u64, stored: &Stored) -> Self {
        Self {
            mode: stored.mode,
            window: Window::new(now_ms, stored.budget_s()),
        }
    }

    /// The mode the current budget belongs to.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn window(&self) -> Window {
        self.window
    }

    /// How much of the budget is left.
    pub fn remaining_ms(&self, now_ms: u64) -> u64 {
        self.window.remaining_ms(now_ms)
    }

    /// The top of a pass: notice a mode that moved and budget on it, then
    /// say what to do. Returns whether the budget was restarted.
    ///
    /// A wake check's window and an idle timeout are the same deadline
    /// field holding two very different numbers, so a mode that moved gets
    /// a fresh budget on its own terms rather than the old one's deadline.
    pub fn pass(&mut self, now_ms: u64, stored: &Stored) -> (bool, Next) {
        let rebudgeted = stored.mode != self.mode;
        if rebudgeted {
            self.mode = stored.mode;
            self.window = Window::new(now_ms, stored.budget_s());
        }
        (rebudgeted, self.window.next(now_ms, stored))
    }

    /// What an accept outcome means.
    ///
    /// A connect during a wake check is a doorbell, not a leash: the
    /// attempt alone promotes the board to idle with the idle timeout
    /// armed, and the app can take its time - including reconnecting after
    /// a handshake that fizzled, which phones do routinely. On the attempt
    /// rather than on a completed session, because the failure mode of the
    /// stricter rule is a board that goes back down for five minutes over
    /// one bad handshake.
    ///
    /// `stored` is the settings as they were before any promotion; the
    /// idle budget is computed from them with the mode moved.
    pub fn on_accept(&mut self, now_ms: u64, accepted: Accepted, stored: &Stored) -> Step {
        let promote =
            self.mode == Mode::Stored && matches!(accepted, Accepted::Connected | Accepted::Failed);
        if promote {
            self.mode = Mode::Idle;
            let idle = Stored {
                mode: Mode::Idle,
                ..*stored
            };
            self.window = Window::new(now_ms, idle.budget_s());
        }
        let then = match accepted {
            Accepted::Connected => Then::Serve,
            Accepted::Failed => {
                // Held open for the retry rather than left to run out: a
                // phone that fizzled a handshake comes back within a
                // second or two, and before this it could find the board
                // dark for `ble_off_s` or asleep for a whole cadence.
                self.window.after_connect_attempt(now_ms);
                Then::Retry
            }
            Accepted::Expired => match stored.at_expiry() {
                Next::Sleep { interval_s } => Then::Sleep(interval_s),
                Next::BleDown { off_s } => Then::BleDown(off_s),
                Next::Advertise { .. } => Then::Continue,
            },
            Accepted::Command(ServeCommand::SleepNow(secs)) => Then::Sleep(secs),
            Accepted::Command(ServeCommand::ModeChanged) => Then::Continue,
        };
        Step { promote, then }
    }

    /// The session ended. Sleep if a command asked for it during the
    /// session, else re-arm the budget the way the mode does after a
    /// disconnect and advertise again.
    ///
    /// Checked after the session rather than inside it so the ack, the
    /// settings republish and the link teardown have all happened: the
    /// board is gone the moment a sleep starts, and anything still owed to
    /// the central has to have left first.
    pub fn on_session_end(&mut self, now_ms: u64, sleep_now: Option<u32>, stored: &Stored) -> Then {
        if let Some(secs) = sleep_now {
            return Then::Sleep(secs);
        }
        // Re-arm by advertising, not by idling: the point is to let the
        // phone come straight back, which it cannot do if the board is
        // awake but not discoverable.
        self.window.after_disconnect(now_ms, stored);
        Then::Continue
    }
}

/// What a command arriving during the modem's off period means: a nap
/// takes the board down at once, a moved mode ends the off period early
/// so the next pass can budget on the new mode.
pub fn during_ble_down(command: ServeCommand) -> Then {
    match command {
        ServeCommand::SleepNow(secs) => Then::Sleep(secs),
        ServeCommand::ModeChanged => Then::Continue,
    }
}

// ---------------------------------------------------------------------------
// From a config write to the two loops
// ---------------------------------------------------------------------------

/// What an [`Action`] sets in motion beyond the settings it already changed:
/// a request to the hardware loop, a command for the serve loop, a
/// per-session value.
///
/// The two loops are answered separately because they own different
/// things. The hardware half of a mode change is a request the hardware
/// loop picks up on its next pass; the budget half is a command to the
/// serve loop, which owns the advertising and the `Rtc`. A store is a
/// command that ends with the chip gone, so it goes to the serve loop by
/// the same route a nap does - the ack has to leave before the board acts,
/// and the link does not survive the action.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Dispatch {
    pub request: Option<Request>,
    pub command: Option<ServeCommand>,
    /// The position notify interval, which is session state rather than a
    /// stored setting.
    pub notify_interval_ms: Option<u32>,
    /// A node to call over LoRa: `(target, tracking)`.
    pub wake: Option<(u8, bool)>,
}

/// What `action` sets in motion, given the settings as they are after the
/// write that produced it.
pub fn dispatch(action: Action, stored: &Stored) -> Dispatch {
    let mut d = Dispatch::default();
    match action {
        Action::GpsSleep(on) => d.request = Some(Request::GpsSleep(on)),
        Action::RadioStandby(on) => d.request = Some(Request::RadioStandby(on)),
        Action::NotifyInterval(ms) => d.notify_interval_ms = Some(ms),
        Action::SleepNow(secs) => d.command = Some(ServeCommand::SleepNow(secs)),
        Action::SetMode(Mode::Stored) => {
            d.command = Some(ServeCommand::SleepNow(stored.sleep_cadence()));
        }
        Action::SetMode(mode) => {
            d.request = Some(Request::Mode(mode));
            d.command = Some(ServeCommand::ModeChanged);
        }
        Action::WakeNode { target, tracking } => d.wake = Some((target, tracking)),
        // Settings the loops read for themselves when they next decide.
        Action::Knob(..) | Action::Name | Action::None => {}
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Where each older layout put its crc: the length that version had.
    const V2_CRC_AT: usize = 16;
    const V3_CRC_AT: usize = 20;
    const V4_CRC_AT: usize = 24;
    const V5_CRC_AT: usize = 32;
    const V6_CRC_AT: usize = BLE_ON_AT;

    /// A config write in the wire format: `[id, len, value]`.
    fn write(id: u8, value: &[u8]) -> Vec<u8> {
        let mut v = vec![id, value.len() as u8];
        v.extend_from_slice(value);
        v
    }

    fn u32_write(id: u8, v: u32) -> Vec<u8> {
        write(id, &v.to_le_bytes())
    }

    /// The applied `u32` an ack echoes back.
    fn ack_u32(o: &Outcome) -> u32 {
        let a = packet::parse_ack(o.ack()).expect("ack parses");
        assert_eq!(a.status, packet::ACK_OK, "ack is an error, not a value");
        a.value_u32.expect("ack carries a u32")
    }

    fn ack_status(o: &Outcome) -> u8 {
        packet::parse_ack(o.ack()).expect("ack parses").status
    }

    fn ack_id(o: &Outcome) -> u8 {
        packet::parse_ack(o.ack()).expect("ack parses").id
    }

    /// A stored board on `interval_s`, i.e. what a wake check runs on.
    fn cadence(interval_s: u32) -> Stored {
        Stored {
            sleep_interval_s: interval_s,
            mode: Mode::Stored,
            ..Stored::new()
        }
    }

    /// An idle board with a timeout set and `interval_s` to store itself
    /// on. `0` is a board with deep sleep off, which stays idle however
    /// long the timeout is: there is nowhere for it to send the board.
    fn idle(interval_s: u32) -> Stored {
        Stored {
            sleep_interval_s: interval_s,
            idle_timeout_s: ble::IDLE_TIMEOUT_DEFAULT_S,
            mode: Mode::Idle,
            ..Stored::new()
        }
    }

    // -- settings ----------------------------------------------------------

    /// A board that has never been configured powers its GPS, never sleeps
    /// and advertises for the default window. Nothing stored may read as
    /// "off", because a board is unconfigured exactly when nobody has been
    /// able to tell it anything.
    #[test]
    fn an_unconfigured_board_is_awake_and_powered() {
        let s = Stored::new();
        assert_eq!(s, Stored::default());
        assert!(!s.radio_standby());
        assert!(!s.gps_sleep());
        assert_eq!(s.sleep_interval_s, 0);
        assert_eq!(s.adv_window(), ble::ESP_ADV_DEFAULT_S);
        assert_eq!(
            s.settings(packet::UPDATE_INTERVAL_DEFAULT_MS),
            ble::Settings {
                radio_standby: false,
                gps_sleep: false,
                sleep_interval_s: 0,
                notify_interval_ms: packet::UPDATE_INTERVAL_DEFAULT_MS,
                adv_window_s: ble::ESP_ADV_DEFAULT_S,
                ble_off_s: 0,
                mode: Mode::Stored,
                idle_timeout_s: 0,
                ble_on_s: ble::BLE_ON_DEFAULT_S,
            }
        );
    }

    /// The settings characteristic reports what is stored, over the wire an
    /// app can read it back from.
    #[test]
    fn settings_report_the_stored_state() {
        let mut s = Stored::new();
        s.set_flag(PFLAG_RADIO_STANDBY, true);
        s.sleep_interval_s = 120;
        s.adv_window_s = 30;

        let settings = s.settings(2_000);
        assert!(settings.radio_standby);
        assert!(!settings.gps_sleep);
        assert_eq!(settings.sleep_interval_s, 120);
        assert_eq!(settings.notify_interval_ms, 2_000);
        assert_eq!(settings.adv_window_s, 30);
        assert_eq!(ble::Settings::decode(&settings.encode()), Some(settings));
    }

    /// The window an app reads back is the one the board will actually use,
    /// never the "never configured" 0 - an app that echoed a 0 back would
    /// otherwise be asking for a window nobody can connect in.
    #[test]
    fn an_unset_window_reports_as_the_default() {
        let s = Stored::new();
        assert_eq!(s.adv_window_s, 0);
        assert_eq!(s.settings(1_000).adv_window_s, ble::ESP_ADV_DEFAULT_S);
    }


    // -- config writes -----------------------------------------------------



    /// The override flags are recorded as soon as they are asked for, before
    /// the hardware loop has moved, because the settings characteristic has
    /// to report what the app asked for.
    #[test]
    fn an_override_write_records_the_request_before_the_hardware_moves() {
        let mut s = Stored::new();
        let o = apply(&mut s, &write(ble::CFG_RADIO_STANDBY, &[1]));
        assert_eq!(o.action, Action::RadioStandby(true));
        assert!(s.radio_standby());
        assert_eq!(o.ack(), &[ble::CFG_RADIO_STANDBY, packet::ACK_OK, 1]);

        let o = apply(&mut s, &write(ble::CFG_GPS_SLEEP, &[1]));
        assert_eq!(o.action, Action::GpsSleep(true));
        assert!(s.gps_sleep());

        // And clearing them again.
        apply(&mut s, &write(ble::CFG_RADIO_STANDBY, &[0]));
        apply(&mut s, &write(ble::CFG_GPS_SLEEP, &[0]));
        assert!(!s.radio_standby());
        assert!(!s.gps_sleep());
    }

    /// The two override flags are the settings that are not written to
    /// flash: they are re-applied on the next tracking command, and a
    /// board that cold-boots with its GPS running is the safer failure.
    #[test]
    fn only_the_reachability_settings_reach_flash() {
        let mut s = Stored::new();
        assert!(!apply(&mut s, &write(ble::CFG_RADIO_STANDBY, &[1])).save);
        assert!(!apply(&mut s, &write(ble::CFG_GPS_SLEEP, &[1])).save);
        assert!(apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 60)).save);
        assert!(apply(&mut s, &u32_write(ble::CFG_ESP_ADV_WINDOW_S, 20)).save);
    }

    /// An interval the board cannot honor comes back as the one it will,
    /// so the app displays the effective setting rather than the request.
    #[test]
    fn a_sleep_interval_is_clamped_and_echoed() {
        let mut s = Stored::new();
        for (asked, applied) in [
            (1, ble::ESP_SLEEP_MIN_S),
            (u32::MAX, ble::ESP_SLEEP_MAX_S),
            (60, 60),
            (ble::ESP_SLEEP_MIN_S, ble::ESP_SLEEP_MIN_S),
            (ble::ESP_SLEEP_MAX_S, ble::ESP_SLEEP_MAX_S),
        ] {
            let o = apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, asked));
            assert_eq!(o.action, Action::Knob(Knob::SleepInterval, applied), "asked {}", asked);
            assert_eq!(ack_u32(&o), applied);
            assert_eq!(s.sleep_interval_s, applied);
        }
    }

    /// Zero is the one interval that is not clamped up: it means sleep is
    /// off, not "sleep as fast as you can".
    #[test]
    fn a_zero_sleep_interval_disables_sleep() {
        let mut s = Stored::new();
        apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 300));
        let o = apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 0));
        assert_eq!(o.action, Action::Knob(Knob::SleepInterval, 0));
        assert_eq!(ack_u32(&o), 0);
        assert_eq!(s.sleep_interval_s, 0);
    }

    /// The window is clamped in both directions including at zero, which is
    /// not an "off": a board that woke to a zero-length window could not be
    /// reached by anything short of a physical reset.
    #[test]
    fn an_advertising_window_is_always_clamped() {
        let mut s = Stored::new();
        for (asked, applied) in [
            (0, ble::ESP_ADV_MIN_S),
            (1, ble::ESP_ADV_MIN_S),
            (u32::MAX, ble::ESP_ADV_MAX_S),
            (20, 20),
        ] {
            let o = apply(&mut s, &u32_write(ble::CFG_ESP_ADV_WINDOW_S, asked));
            assert_eq!(o.action, Action::Knob(Knob::AdvWindow, applied), "asked {}", asked);
            assert_eq!(ack_u32(&o), applied);
            assert_eq!(s.adv_window(), applied);
        }
    }

    /// The two sleep settings are independent: setting a window must not
    /// disturb the interval, or an app writing both would have to care
    /// about the order.
    #[test]
    fn the_sleep_settings_do_not_disturb_each_other() {
        let mut s = Stored::new();
        apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 90));
        apply(&mut s, &u32_write(ble::CFG_ESP_ADV_WINDOW_S, 30));
        apply(&mut s, &write(ble::CFG_RADIO_STANDBY, &[1]));
        assert_eq!(s.sleep_interval_s, 90);
        assert_eq!(s.adv_window(), 30);
        assert!(s.radio_standby());
    }

    /// A malformed value changes nothing. Silently applying part of it
    /// would leave the board on a cadence the app never asked for.
    /// 0 is off rather than a floor, because a board that never takes BLE
    /// down is the reachable default and has to stay expressible.
    #[test]
    fn the_ble_off_period_clamps_but_keeps_zero() {
        let mut s = Stored::new();
        assert_eq!(s.ble_off_s, 0);
        for (asked, applied) in [
            (0u32, 0u32),
            (1, ble::BLE_OFF_MIN_S),
            (30, 30),
            (99_999, ble::BLE_OFF_MAX_S),
        ] {
            let o = apply(&mut s, &u32_write(ble::CFG_BLE_OFF_S, asked));
            assert_eq!(o.action, Action::Knob(Knob::BleOff, applied), "asked {}", asked);
            assert_eq!(s.ble_off_s, applied, "asked {}", asked);
            assert!(o.save);
        }
    }

    #[test]
    fn a_malformed_value_is_rejected_and_changes_nothing() {
        let mut s = Stored::new();
        apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 60));
        apply(&mut s, &u32_write(ble::CFG_ESP_ADV_WINDOW_S, 20));
        let before = s;

        for data in [
            write(ble::CFG_ESP_SLEEP_S, &[0, 0]),
            write(ble::CFG_ESP_ADV_WINDOW_S, &[1, 2, 3, 4, 5]),
            // A declared length that runs past the write itself.
            vec![ble::CFG_ESP_SLEEP_S, 4, 0, 0],
        ] {
            let o = apply(&mut s, &data);
            assert_eq!(o.action, Action::None);
            assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
            assert!(!o.save);
            assert_eq!(s, before, "rejected write changed the settings");
        }
    }

    /// The board still speaks the gps-proto config protocol it inherited,
    /// so the app that predates it keeps working.
    #[test]
    fn a_gps_proto_write_still_works() {
        let mut s = Stored::new();
        let before = s;
        let o = apply(&mut s, &u32_write(packet::CFG_UPDATE_INTERVAL_MS, 500));
        assert_eq!(o.action, Action::NotifyInterval(500));
        assert_eq!(ack_u32(&o), 500);
        assert!(!o.save, "the notify interval is per-session state");
        assert_eq!(s, before, "the notify interval is not a stored setting");

        // Clamped like any other value, and echoed as applied.
        let o = apply(&mut s, &u32_write(packet::CFG_UPDATE_INTERVAL_MS, 1));
        assert_eq!(
            o.action,
            Action::NotifyInterval(packet::UPDATE_INTERVAL_MIN_MS)
        );
        assert_eq!(ack_u32(&o), packet::UPDATE_INTERVAL_MIN_MS);
    }

    /// An id from a newer app is refused by id, not ignored, so the app can
    /// tell "this board is older" from "the write was lost".
    #[test]
    fn an_unknown_id_is_nacked_with_its_id() {
        let mut s = Stored::new();
        let before = s;
        let o = apply(&mut s, &u32_write(0x7F, 1));
        assert_eq!(o.action, Action::None);
        assert_eq!(ack_id(&o), 0x7F);
        assert_eq!(ack_status(&o), packet::ACK_UNKNOWN_ID);
        assert_eq!(s, before);
    }

    #[test]
    fn a_truncated_write_is_rejected() {
        let mut s = Stored::new();
        for data in [vec![], vec![ble::CFG_GPS_SLEEP]] {
            let o = apply(&mut s, &data);
            assert_eq!(o.action, Action::None);
            assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
        }
        assert_eq!(s, Stored::new());
    }

    // -- the flash record --------------------------------------------------

    #[test]
    fn a_record_roundtrips() {
        let s = Stored {
            sleep_interval_s: 300,
            flags: PFLAG_GPS_SLEEP,
            adv_window_s: 45,
            ble_off_s: 90,
            mode: Mode::Tracking,
            idle_timeout_s: 900,
            ble_on_s: 20,
            name: *b"sky-1\0\0\0\0\0\0\0\0\0\0\0",
        };
        let rec = s.encode_record();
        assert_eq!(rec.len(), RECORD_LEN);
        assert_eq!(Stored::decode_record(&rec), Some(s));
    }

    /// A board updated in the field keeps the cadence it was left on: the
    /// window was appended to the record, not inserted into it.
    #[test]
    fn a_version_2_record_still_reads() {
        let mut rec = [0xFFu8; RECORD_LEN]; // erased flash past the record
        rec[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&2u32.to_le_bytes());
        rec[8..12].copy_from_slice(&120u32.to_le_bytes());
        rec[12..16].copy_from_slice(&PFLAG_RADIO_STANDBY.to_le_bytes());
        let crc = link::crc32(&rec[0..V2_CRC_AT]);
        rec[V2_CRC_AT..V2_CRC_AT + 4].copy_from_slice(&crc.to_le_bytes());

        let s = Stored::decode_record(&rec).expect("a version 2 record still reads");
        assert_eq!(s.sleep_interval_s, 120);
        assert!(s.radio_standby());
        // No window in that layout, so the default applies.
        assert_eq!(s.adv_window_s, 0);
        assert_eq!(s.adv_window(), ble::ESP_ADV_DEFAULT_S);
        // Nor a BLE off period, which means the old behavior: never take
        // the controller down.
        assert_eq!(s.ble_off_s, 0);
    }

    /// The version this firmware replaced. A board updated in the field
    /// keeps the cadence and window it was left on rather than coming back
    /// unconfigured and advertising continuously.
    #[test]
    fn a_version_3_record_still_reads() {
        let mut rec = [0xFFu8; RECORD_LEN]; // erased flash past the record
        rec[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&3u32.to_le_bytes());
        rec[8..12].copy_from_slice(&120u32.to_le_bytes());
        rec[12..16].copy_from_slice(&PFLAG_RADIO_STANDBY.to_le_bytes());
        rec[16..20].copy_from_slice(&45u32.to_le_bytes());
        let crc = link::crc32(&rec[0..V3_CRC_AT]);
        rec[V3_CRC_AT..V3_CRC_AT + 4].copy_from_slice(&crc.to_le_bytes());

        let s = Stored::decode_record(&rec).expect("a version 3 record still reads");
        assert_eq!(s.sleep_interval_s, 120);
        assert!(s.radio_standby());
        assert_eq!(s.adv_window_s, 45);
        // The field that version predates reads as disabled, so an updated
        // board does not start going dark on its own.
        assert_eq!(s.ble_off_s, 0);
    }

    /// The version this mode work replaced. A board updated in the field
    /// keeps its cadence, window and off period, and reads as a stored
    /// board - which a cold boot then turns into the idle rescue window.
    #[test]
    fn a_version_4_record_still_reads() {
        let mut rec = [0xFFu8; RECORD_LEN]; // erased flash past the record
        rec[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&4u32.to_le_bytes());
        rec[8..12].copy_from_slice(&120u32.to_le_bytes());
        rec[12..16].copy_from_slice(&PFLAG_RADIO_STANDBY.to_le_bytes());
        rec[16..20].copy_from_slice(&45u32.to_le_bytes());
        rec[20..24].copy_from_slice(&30u32.to_le_bytes());
        let crc = link::crc32(&rec[0..V4_CRC_AT]);
        rec[V4_CRC_AT..V4_CRC_AT + 4].copy_from_slice(&crc.to_le_bytes());

        let s = Stored::decode_record(&rec).expect("a version 4 record still reads");
        assert_eq!(s.sleep_interval_s, 120);
        assert_eq!(s.adv_window_s, 45);
        assert_eq!(s.ble_off_s, 30);
        // No mode in that layout, and the safe one is what it reads as: an
        // updated board is reachable first and stores itself after.
        assert_eq!(s.mode, Mode::Stored);
        assert_eq!(s.idle_timeout_s, 0);
        assert_eq!(s.idle_timeout(), 0, "and it never stores itself out of idle");
        assert_eq!(boot_mode(s.mode, false), Mode::Idle);
    }

    /// A board updated in the field is unnamed rather than unreadable: the
    /// name was appended, so a version 5 record still carries every setting
    /// that decides reachability.
    #[test]
    fn a_version_5_record_still_reads() {
        let mut rec = [0xFFu8; RECORD_LEN]; // erased flash past the record
        rec[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&5u32.to_le_bytes());
        rec[8..12].copy_from_slice(&120u32.to_le_bytes());
        rec[12..16].copy_from_slice(&PFLAG_RADIO_STANDBY.to_le_bytes());
        rec[16..20].copy_from_slice(&45u32.to_le_bytes());
        rec[20..24].copy_from_slice(&30u32.to_le_bytes());
        rec[24..28].copy_from_slice(&(Mode::Tracking.as_wire() as u32).to_le_bytes());
        rec[28..32].copy_from_slice(&900u32.to_le_bytes());
        let crc = link::crc32(&rec[0..V5_CRC_AT]);
        rec[V5_CRC_AT..V5_CRC_AT + 4].copy_from_slice(&crc.to_le_bytes());

        let s = Stored::decode_record(&rec).expect("a version 5 record still reads");
        assert_eq!(s.sleep_interval_s, 120);
        assert_eq!(s.mode, Mode::Tracking);
        assert_eq!(s.idle_timeout_s, 900);
        // The erased bytes where the name now sits are not read as one.
        assert_eq!(s.label(), "");
        assert_eq!(s.name, [0; ble::NAME_FIELD_LEN]);
        assert_eq!(s.ble_on_s, 0);
    }

    /// A board updated in the field keeps its name and comes back on the
    /// default on period, which is the window it used to share.
    #[test]
    fn a_version_6_record_still_reads() {
        let mut rec = [0xFFu8; RECORD_LEN]; // erased flash past the record
        rec[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&6u32.to_le_bytes());
        rec[8..12].copy_from_slice(&120u32.to_le_bytes());
        rec[12..16].copy_from_slice(&0u32.to_le_bytes());
        rec[16..20].copy_from_slice(&45u32.to_le_bytes());
        rec[20..24].copy_from_slice(&30u32.to_le_bytes());
        rec[24..28].copy_from_slice(&(Mode::Tracking.as_wire() as u32).to_le_bytes());
        rec[28..32].copy_from_slice(&0u32.to_le_bytes());
        let mut name = [0u8; ble::NAME_FIELD_LEN];
        name[..5].copy_from_slice(b"sky-1");
        rec[32..32 + ble::NAME_FIELD_LEN].copy_from_slice(&name);
        let crc = link::crc32(&rec[0..V6_CRC_AT]);
        rec[V6_CRC_AT..V6_CRC_AT + 4].copy_from_slice(&crc.to_le_bytes());

        let s = Stored::decode_record(&rec).expect("a version 6 record still reads");
        assert_eq!(s.label(), "sky-1");
        assert_eq!(s.adv_window_s, 45);
        assert_eq!(s.ble_on_s, 0);
        assert_eq!(s.ble_on(), ble::BLE_ON_DEFAULT_S);
        // Written back, it is a version 7 record that reads the same.
        let again = Stored::decode_record(&s.encode_record()).expect("round trip");
        assert_eq!(again, s);
    }

    // -- the name ----------------------------------------------------------

    #[test]
    fn a_name_write_stores_a_label() {
        let mut s = Stored::new();
        assert_eq!(s.label(), "");

        let o = apply(&mut s, &write(ble::CFG_NAME, b"sky-1"));
        assert_eq!(o.action, Action::Name);
        assert_eq!(s.label(), "sky-1");
        // The ack spends its value on the length: no name fits in one.
        assert_eq!(o.ack(), &[ble::CFG_NAME, packet::ACK_OK, 5]);
        // A name has to survive a flat cell, so it is worth a flash write.
        assert!(o.save);
        assert_eq!(Stored::decode_record(&s.encode_record()), Some(s));
    }

    /// An empty value is the way back to an address-derived name, not a
    /// rejected write.
    #[test]
    fn an_empty_name_write_clears_the_label() {
        let mut s = Stored::new();
        apply(&mut s, &write(ble::CFG_NAME, b"ground-1"));
        let o = apply(&mut s, &write(ble::CFG_NAME, b""));
        assert_eq!(o.action, Action::Name);
        assert_eq!(ack_status(&o), packet::ACK_OK);
        assert_eq!(o.ack(), &[ble::CFG_NAME, packet::ACK_OK, 0]);
        assert_eq!(s.label(), "");
    }

    /// A label the board cannot advertise is refused, and refusing it
    /// leaves the board under the name it already answered to.
    #[test]
    fn a_bad_label_leaves_the_old_name() {
        let mut s = Stored::new();
        apply(&mut s, &write(ble::CFG_NAME, b"sky-1"));
        for bad in [b"has space".as_slice(), b"quote\"", "caf\u{e9}".as_bytes()] {
            let o = apply(&mut s, &write(ble::CFG_NAME, bad));
            assert_eq!(o.action, Action::None);
            assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
            assert!(!o.save);
            assert_eq!(s.label(), "sky-1");
        }
    }

    /// The longest label the record holds is one the board still reads
    /// back: there is no terminator to lose when the padding runs out.
    #[test]
    fn a_full_length_label_survives_the_record() {
        let label = "x".repeat(ble::NAME_LABEL_MAX);
        let mut s = Stored::new();
        let o = apply(&mut s, &write(ble::CFG_NAME, label.as_bytes()));
        assert_eq!(o.action, Action::Name);
        assert_eq!(s.label(), label);
        let back = Stored::decode_record(&s.encode_record()).expect("record");
        assert_eq!(back.label(), label);

        // One byte more is refused by the policy rather than truncated.
        let o = apply(&mut s, &write(ble::CFG_NAME, "y".repeat(ble::NAME_LABEL_MAX + 1).as_bytes()));
        assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
        assert_eq!(s.label(), label);
    }

    /// A name is a settings change, not a command: nothing else about the
    /// board moves with it.
    #[test]
    fn a_name_write_changes_nothing_else() {
        let mut s = Stored::new();
        apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 120));
        let before = s;
        apply(&mut s, &write(ble::CFG_NAME, b"ground-1"));
        assert_eq!(
            Stored {
                name: before.name,
                ..s
            },
            before
        );
    }

    /// Idle never reaches flash. The RTC copy carries it - that is what the
    /// settings characteristic reports and what the serve loop budgets on -
    /// but a record that said idle would bring a board back at awake
    /// current with nobody coming.
    #[test]
    fn a_record_never_says_idle() {
        let s = Stored {
            mode: Mode::Idle,
            ..Stored::new()
        };
        assert_eq!(s.mode, Mode::Idle, "the live copy keeps it");
        let back = Stored::decode_record(&s.encode_record()).expect("record reads back");
        assert_eq!(back.mode, Mode::Stored);

        // Tracking is the one mode that does survive, because a brownout on
        // the object is exactly when it must.
        let t = Stored {
            mode: Mode::Tracking,
            ..Stored::new()
        };
        let back = Stored::decode_record(&t.encode_record()).expect("record reads back");
        assert_eq!(back.mode, Mode::Tracking);
    }

    // -- modes -------------------------------------------------------------

    /// The three boot flavors. Tracking survives any reset; a timer wake is
    /// a wake check; every other cold boot gets the rescue window.
    #[test]
    fn a_boot_lands_in_the_right_mode() {
        assert_eq!(boot_mode(Mode::Tracking, true), Mode::Tracking);
        assert_eq!(boot_mode(Mode::Tracking, false), Mode::Tracking);
        assert_eq!(boot_mode(Mode::Stored, true), Mode::Stored);
        assert_eq!(boot_mode(Mode::Stored, false), Mode::Idle);
        // Idle cannot be persisted, but a garbled record that produced one
        // must still land somewhere sane.
        assert_eq!(boot_mode(Mode::Idle, false), Mode::Idle);
        assert_eq!(boot_mode(Mode::Idle, true), Mode::Stored);
    }

    /// A mode write is stored, acked with what it stored, and saved -
    /// because it is the setting that decides what the board does when it
    /// comes back from a flat cell.
    #[test]
    fn a_mode_write_persists_and_acts() {
        let mut s = Stored::new();
        let o = apply(&mut s, &write(ble::CFG_MODE, &[2]));
        assert_eq!(o.action, Action::SetMode(Mode::Tracking));
        assert!(o.save, "tracking must survive a brownout on the object");
        assert_eq!(s.mode, Mode::Tracking);
        assert_eq!(o.ack(), &[ble::CFG_MODE, packet::ACK_OK, 2]);

        let o = apply(&mut s, &write(ble::CFG_MODE, &[1]));
        assert_eq!(o.action, Action::SetMode(Mode::Idle));
        assert_eq!(s.mode, Mode::Idle);

        let o = apply(&mut s, &write(ble::CFG_MODE, &[0]));
        assert_eq!(o.action, Action::SetMode(Mode::Stored));
        assert_eq!(s.mode, Mode::Stored);
    }

    /// A mode this firmware does not know is rejected rather than rounded:
    /// each value is a different amount of the board switched off.
    #[test]
    fn an_unknown_mode_is_rejected() {
        let mut s = Stored {
            mode: Mode::Tracking,
            ..Stored::new()
        };
        for bad in [&[4u8][..], &[255][..], &[][..]] {
            let o = apply(&mut s, &write(ble::CFG_MODE, bad));
            assert_eq!(o.action, Action::None);
            assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
            assert_eq!(s.mode, Mode::Tracking, "a rejected write changed nothing");
        }
    }

    /// The idle timeout is off by default and 0 is how it is turned off
    /// again; a non-zero value clamps like every other interval.
    #[test]
    fn the_idle_timeout_is_off_by_default_and_clamps_when_set() {
        let mut s = Stored::new();
        assert_eq!(s.idle_timeout_s, 0);
        assert_eq!(s.idle_timeout(), 0);

        let o = apply(&mut s, &u32_write(ble::CFG_IDLE_TIMEOUT_S, 900));
        assert_eq!(o.action, Action::Knob(Knob::IdleTimeout, 900));
        assert!(o.save);
        assert_eq!(ack_u32(&o), 900);
        assert_eq!(s.idle_timeout(), 900);

        for (asked, applied) in [
            (1, ble::IDLE_TIMEOUT_MIN_S),
            (u32::MAX, ble::IDLE_TIMEOUT_MAX_S),
            (0, 0),
        ] {
            let o = apply(&mut s, &u32_write(ble::CFG_IDLE_TIMEOUT_S, asked));
            assert_eq!(ack_u32(&o), applied);
            assert_eq!(s.idle_timeout_s, applied);
        }

        // Off means off: an idle board with a cadence to sleep on still
        // keeps advertising however long it has been idle.
        let idle = Stored {
            mode: Mode::Idle,
            sleep_interval_s: 120,
            ..s
        };
        assert_eq!(idle.at_expiry(), Next::FOREVER);
        assert_eq!(
            Window::new(0, idle.budget_s()).next(3_600_000, &idle),
            Next::FOREVER
        );
    }

    /// The tracker's on period is its own setting, so a wake check and a
    /// tracker can want different lengths of window.
    #[test]
    fn the_ble_on_period_is_separate_from_the_advertising_window() {
        let mut s = Stored::new();
        assert_eq!(s.ble_on_s, 0);
        assert_eq!(s.ble_on(), ble::BLE_ON_DEFAULT_S);

        apply(&mut s, &u32_write(ble::CFG_ESP_ADV_WINDOW_S, 5));
        let o = apply(&mut s, &u32_write(ble::CFG_BLE_ON_S, 40));
        assert_eq!(o.action, Action::Knob(Knob::BleOn, 40));
        assert!(o.save);
        assert_eq!(ack_u32(&o), 40);
        assert_eq!(s.adv_window(), 5);
        assert_eq!(s.ble_on(), 40);
        assert_eq!(s.settings(1_000).ble_on_s, 40);

        for (asked, applied) in [
            (0, ble::BLE_ON_MIN_S),
            (u32::MAX, ble::BLE_ON_MAX_S),
        ] {
            let o = apply(&mut s, &u32_write(ble::CFG_BLE_ON_S, asked));
            assert_eq!(ack_u32(&o), applied);
        }

        // The wake check budgets on the window, the tracker on the on
        // period, and neither reads the other's.
        s.ble_on_s = 40;
        s.mode = Mode::Stored;
        assert_eq!(s.budget_s(), 5);
        s.mode = Mode::Tracking;
        assert_eq!(s.budget_s(), 40);
        // And the record carries it across a flat cell.
        let back = Stored::decode_record(&s.encode_record()).expect("round trip");
        assert_eq!(back.ble_on_s, 40);
    }

    /// Listening is the receiver half of a tracker for the node beside the
    /// phone: it never duty-cycles, never stores itself, and comes back
    /// listening after any reset.
    #[test]
    fn listening_never_expires_and_survives_a_reset() {
        let s = Stored {
            mode: Mode::Listening,
            sleep_interval_s: 120,
            ble_off_s: 30,
            idle_timeout_s: 60,
            ..Stored::new()
        };
        assert_eq!(s.at_expiry(), Next::FOREVER);
        let w = Window::new(0, s.budget_s());
        assert_eq!(w.next(3_600_000, &s), Next::FOREVER);
        assert_eq!(boot_mode(Mode::Listening, false), Mode::Listening);
        assert_eq!(boot_mode(Mode::Listening, true), Mode::Listening);
        let back = Stored::decode_record(&s.encode_record()).expect("round trip");
        assert_eq!(back.mode, Mode::Listening);

        let o = apply(&mut Stored::new(), &write(ble::CFG_MODE, &[Mode::Listening.as_wire()]));
        assert_eq!(o.action, Action::SetMode(Mode::Listening));
        assert!(o.save);
    }

    /// Which duty cycle applies is a property of the mode, not a race
    /// between two settings. The regression: `ble_off_s` used to be dead
    /// config on any board that had a wake-check cadence, because deep
    /// sleep was tested first and always won.
    #[test]
    fn each_mode_has_its_own_duty_cycle() {
        let both = Stored {
            sleep_interval_s: 120,
            ble_off_s: 30,
            ..Stored::new()
        };

        // A wake check goes back down on the cadence and ignores the modem
        // period - there is no controller to duty-cycle for a board that is
        // about to lose the whole chip.
        let wake = Stored {
            mode: Mode::Stored,
            ..both
        };
        assert_eq!(wake.at_expiry(), Next::Sleep { interval_s: 120 });
        assert_eq!(wake.budget_s(), wake.adv_window());

        // Idle stores itself, on the same cadence but its own budget - once
        // it has been given a timeout at all.
        let idle = Stored {
            mode: Mode::Idle,
            idle_timeout_s: 600,
            ..both
        };
        assert_eq!(idle.at_expiry(), Next::Sleep { interval_s: 120 });
        assert_eq!(idle.budget_s(), idle.idle_timeout());

        // Tracking takes the modem down and never the whole chip: deep
        // sleep would stop the beacon, the logging and the listening.
        let tracking = Stored {
            mode: Mode::Tracking,
            ..both
        };
        assert_eq!(tracking.at_expiry(), Next::BleDown { off_s: 30 });
        assert_eq!(tracking.budget_s(), tracking.ble_on());
    }

    /// With no cadence there is nowhere for an idle timeout to send the
    /// board, so it stays awake and reachable. That is the bench setting,
    /// and it is what an unconfigured board does.
    #[test]
    fn no_cadence_means_the_board_never_stores_itself() {
        let s = Stored {
            mode: Mode::Idle,
            sleep_interval_s: 0,
            idle_timeout_s: 60,
            ..Stored::new()
        };
        assert_eq!(s.at_expiry(), Next::FOREVER);
        let w = Window::new(0, s.budget_s());
        assert_eq!(w.next(60_000, &s), Next::FOREVER);
    }

    /// Being *told* to store the board is different: somebody asked for it,
    /// so a missing cadence is borrowed rather than read as a refusal.
    /// Without this a board stored without a cadence would wake once and
    /// then sit at awake current forever.
    #[test]
    fn a_stored_board_without_a_cadence_still_sleeps() {
        let s = Stored {
            mode: Mode::Stored,
            sleep_interval_s: 0,
            ..Stored::new()
        };
        assert_eq!(s.sleep_cadence(), ble::STORE_DEFAULT_S);
        assert_eq!(
            s.at_expiry(),
            Next::Sleep {
                interval_s: ble::STORE_DEFAULT_S
            }
        );
        assert_eq!(cadence(120).sleep_cadence(), 120);
        assert_eq!(
            cadence(120).at_expiry(),
            Next::Sleep { interval_s: 120 }
        );

        // And the way back out is the promotion, not the cadence: a wake
        // check that somebody connects to becomes idle, where a board with
        // no cadence stays.
        let promoted = Stored {
            mode: Mode::Idle,
            ..s
        };
        assert_eq!(promoted.at_expiry(), Next::FOREVER);
    }

    /// A disconnect in idle re-arms the whole timeout rather than the five
    /// second linger: the state exists to be reachable, and a phone that
    /// has just been talking to the board is the best evidence anyone still
    /// wants it.
    #[test]
    fn a_disconnect_in_idle_restarts_the_timeout() {
        let s = Stored {
            mode: Mode::Idle,
            sleep_interval_s: 120,
            idle_timeout_s: 600,
            ..Stored::new()
        };
        let mut w = Window::new(0, s.budget_s());
        let disconnected_at = 900_000;
        assert_eq!(
            w.next(disconnected_at, &s),
            Next::Sleep { interval_s: 120 },
            "the budget was already spent while connected"
        );
        w.after_disconnect(disconnected_at, &s);
        assert_eq!(w.remaining_ms(disconnected_at), 600_000);
        assert_eq!(
            w.next(disconnected_at + 600_000, &s),
            Next::Sleep { interval_s: 120 }
        );
    }

    /// A promotion: a wake check that somebody connected to becomes idle,
    /// with the idle budget rather than what was left of the advertising
    /// window. The connect itself is the doorbell.
    #[test]
    fn a_promotion_swaps_the_budget() {
        let mut s = Stored {
            sleep_interval_s: 300,
            adv_window_s: 15,
            idle_timeout_s: 600,
            ..Stored::new()
        };
        assert_eq!(boot_mode(s.mode, true), Mode::Stored);
        let w = Window::new(0, s.budget_s());
        assert_eq!(w.remaining_ms(0), 15_000);

        // Somebody connects at 8 s in.
        s.mode = Mode::Idle;
        let w = Window::new(8_000, s.budget_s());
        assert_eq!(w.remaining_ms(8_000), 600_000);
        assert_eq!(w.next(20_000, &s), Next::Advertise { bounded: true }, "the wake check would have slept");
        assert_eq!(
            w.next(608_000, &s),
            Next::Sleep { interval_s: 300 },
            "and then it stores itself"
        );
    }

    /// Anything that is not this firmware's record leaves the board
    /// unconfigured rather than acting on someone else's bytes.
    #[test]
    fn a_record_that_is_not_ours_is_ignored() {
        let good = Stored {
            sleep_interval_s: 60,
            adv_window_s: 15,
            ..Stored::new()
        }
        .encode_record();

        // Erased flash, and a partition that holds something else.
        assert_eq!(Stored::decode_record(&[0xFF; RECORD_LEN]), None);
        assert_eq!(Stored::decode_record(&[0x00; RECORD_LEN]), None);
        let mut foreign = good;
        foreign[0] ^= 0x01;
        assert_eq!(Stored::decode_record(&foreign), None);

        // A short read is not a record either.
        assert_eq!(Stored::decode_record(&good[..RECORD_LEN - 1]), None);

        // A version this build cannot read, in both directions.
        for version in [0u32, 1, RECORD_VERSION + 1] {
            let mut rec = good;
            rec[4..8].copy_from_slice(&version.to_le_bytes());
            assert_eq!(Stored::decode_record(&rec), None, "version {}", version);
        }

        // A single flipped bit anywhere in the payload fails the crc.
        for byte in 0..RECORD_LEN {
            let mut rec = good;
            rec[byte] ^= 0x08;
            assert_eq!(Stored::decode_record(&rec), None, "byte {}", byte);
        }
    }

    // -- the wake / advertise / sleep cycle --------------------------------

    /// With sleep off the board advertises indefinitely, whatever the
    /// budget says. That is an idle board with nowhere to be sent - the
    /// bench case, and the unconfigured one.
    #[test]
    fn sleep_off_means_the_board_never_sleeps() {
        let s = idle(0);
        let w = Window::new(0, 15);
        assert_eq!(w.next(0, &s), Next::FOREVER);
        assert_eq!(w.next(15_000, &s), Next::FOREVER);
        assert_eq!(w.next(u64::MAX, &s), Next::FOREVER);
    }

    /// The wake advertises for the configured window and then sleeps for
    /// the configured interval.
    #[test]
    fn a_spent_window_ends_in_deep_sleep() {
        let s = cadence(60);
        let w = Window::new(1_000, 15);
        assert_eq!(w.ends_ms(), 16_000);
        assert_eq!(w.next(1_000, &s), Next::Advertise { bounded: true });
        assert_eq!(w.remaining_ms(1_000), 15_000);
        assert_eq!(w.next(15_999, &s), Next::Advertise { bounded: true });
        assert_eq!(w.remaining_ms(15_999), 1);
        assert_eq!(w.next(16_000, &s), Next::Sleep { interval_s: 60 });
        assert_eq!(w.remaining_ms(16_000), 0);
        // A clock past the deadline is still a spent budget, not a wrap.
        assert_eq!(w.remaining_ms(u64::MAX), 0);
        assert_eq!(w.next(u64::MAX, &s), Next::Sleep { interval_s: 60 });
    }

    /// The regression the deadline exists for: a central that keeps failing
    /// to connect must not be able to hold the board awake. Every retry
    /// asks again, and the answer is still measured from the wake.
    #[test]
    fn retries_cannot_extend_the_window() {
        let s = cadence(30);
        let w = Window::new(0, 15);
        let mut now = 0;
        // A connect attempt that fails every 200 ms, as the firmware's
        // retry pause does.
        while w.next(now, &s).advertises() {
            now += 200;
            assert!(now <= 15_200, "the window never ended");
        }
        assert_eq!(w.next(now, &s), Next::Sleep { interval_s: 30 });
        assert_eq!(w.ends_ms(), 15_000, "the deadline moved");
    }

    /// A handshake that fizzled at the end of the window buys the same
    /// short hold a disconnect does, so the retry lands on a board that is
    /// still advertising rather than one that has gone down for a cadence.
    #[test]
    fn a_failed_attempt_holds_the_window_open() {
        let s = cadence(60);
        let mut w = Window::new(0, 15);
        // The phone tried at 14.9 s of a 15 s window and did not finish.
        let tried_at = 14_900;
        w.after_connect_attempt(tried_at);
        assert_eq!(w.next(tried_at, &s), Next::Advertise { bounded: true });
        assert_eq!(
            w.next(15_100, &s),
            Next::Advertise { bounded: true },
            "the window it tried in has expired, and it is still advertising"
        );
        assert_eq!(
            w.next(tried_at + LINGER_S * 1000, &s),
            Next::Sleep { interval_s: 60 },
            "and the hold is a hold, not a new window"
        );
    }

    /// The hold extends; it never shortens. An attempt in the first second
    /// of a ten minute idle timeout must not cut it to five seconds.
    #[test]
    fn a_failed_attempt_cannot_shorten_a_longer_window() {
        let s = Stored {
            mode: Mode::Idle,
            idle_timeout_s: 600,
            ..Stored::new()
        };
        let mut w = Window::new(0, s.budget_s());
        w.after_connect_attempt(1_000);
        assert_eq!(w.ends_ms(), 600_000);
    }

    /// A disconnect buys a short linger, so the phone can come straight
    /// back instead of waiting out a whole sleep interval.
    #[test]
    fn a_disconnect_lingers_then_sleeps() {
        let s = cadence(60);
        let mut w = Window::new(0, 15);
        // A session that outlasted the original window.
        let disconnected_at = 400_000;
        assert_eq!(
            w.next(disconnected_at, &s),
            Next::Sleep { interval_s: 60 },
            "the budget was already spent while connected"
        );
        w.after_disconnect(disconnected_at, &s);
        assert_eq!(w.next(disconnected_at, &s), Next::Advertise { bounded: true });
        assert_eq!(w.remaining_ms(disconnected_at), LINGER_S * 1000);
        assert_eq!(
            w.next(disconnected_at + LINGER_S * 1000, &s),
            Next::Sleep { interval_s: 60 }
        );
    }

    /// Sleep switched on mid-window takes effect at the next decision, not
    /// at the next boot - the settings are asked for every time.
    #[test]
    fn enabling_sleep_takes_effect_within_the_window() {
        let w = Window::new(0, 15);
        assert_eq!(w.next(20_000, &idle(0)), Next::FOREVER);
        assert_eq!(w.next(20_000, &idle(300)), Next::Sleep { interval_s: 300 });
    }

    /// The window a wake runs on is the one that was stored when it woke.
    #[test]
    fn the_window_comes_from_the_stored_setting() {
        let mut s = Stored::new();
        assert_eq!(
            Window::new(0, s.adv_window()).ends_ms(),
            ble::ESP_ADV_DEFAULT_S as u64 * 1000
        );
        apply(&mut s, &u32_write(ble::CFG_ESP_ADV_WINDOW_S, 45));
        assert_eq!(Window::new(0, s.adv_window()).ends_ms(), 45_000);
    }

    /// The whole cycle an unattended board runs: configured over BLE, woken
    /// on the interval, advertising to nobody, back to sleep.
    #[test]
    fn an_unattended_board_keeps_its_cadence() {
        let mut s = Stored::new();
        apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 120));
        apply(&mut s, &u32_write(ble::CFG_ESP_ADV_WINDOW_S, 10));

        // The settings survive the sleep by way of the flash record.
        let s = Stored::decode_record(&s.encode_record()).expect("record reads back");
        assert_eq!(s.sleep_interval_s, 120);

        // Reaching a wake check does not clear the interval, so the board
        // holds its cadence indefinitely rather than staying up after the
        // first visit.
        let mut now = 0;
        for _ in 0..3 {
            let w = Window::new(now, s.budget_s());
            assert_eq!(w.next(now, &s), Next::Advertise { bounded: true });
            now += 10_000;
            assert_eq!(w.next(now, &s), Next::Sleep { interval_s: 120 });
            now += 120_000;
        }
    }

    // -- sleep now ---------------------------------------------------------

    /// The whole point of the command: it sleeps a board that is connected
    /// and configured never to sleep, and it changes nothing about that
    /// configuration.
    #[test]
    fn sleep_now_is_a_command_and_not_a_setting() {
        let mut s = Stored::new();
        assert_eq!(s.sleep_interval_s, 0, "sleep mode is off");

        let before = s;
        let o = apply(&mut s, &u32_write(ble::CFG_SLEEP_NOW, 30));

        assert_eq!(o.action, Action::SleepNow(30));
        assert_eq!(ack_u32(&o), 30, "the ack says how long the board is gone");
        assert_eq!(s, before, "a nap stores nothing, not even the duration");
        assert!(!o.save, "and so nothing needs to reach flash");
    }

    /// 0 borrows the wake-check cadence, which is what makes "Sleep now"
    /// mean "start the next cycle early" on a board that already sleeps.
    #[test]
    fn sleep_now_zero_borrows_the_wake_check_interval() {
        let mut s = Stored::new();
        apply(&mut s, &u32_write(ble::CFG_ESP_SLEEP_S, 120));

        let o = apply(&mut s, &u32_write(ble::CFG_SLEEP_NOW, 0));
        assert_eq!(o.action, Action::SleepNow(120));
        assert_eq!(s.sleep_interval_s, 120, "and does not disturb it");
    }

    /// With no cadence to borrow there is still a defined answer, because
    /// the alternative is a button that does nothing on exactly the boards
    /// most likely to be sitting on a bench.
    #[test]
    fn sleep_now_zero_falls_back_when_sleep_mode_is_off() {
        let mut s = Stored::new();
        let o = apply(&mut s, &u32_write(ble::CFG_SLEEP_NOW, 0));
        assert_eq!(o.action, Action::SleepNow(ble::SLEEP_NOW_DEFAULT_S));
    }

    /// Clamped to the same range as the wake-check interval - including the
    /// floor, which the interval itself exempts 0 from. Here 0 already
    /// means something else, so nothing reaches the clamp as a zero.
    #[test]
    fn sleep_now_is_clamped_like_the_wake_check_interval() {
        let mut s = Stored::new();
        assert_eq!(
            apply(&mut s, &u32_write(ble::CFG_SLEEP_NOW, 1)).action,
            Action::SleepNow(ble::ESP_SLEEP_MIN_S)
        );
        assert_eq!(
            apply(&mut s, &u32_write(ble::CFG_SLEEP_NOW, 99_999)).action,
            Action::SleepNow(ble::ESP_SLEEP_MAX_S)
        );
    }

    /// A short write is rejected rather than read as some other duration.
    #[test]
    fn sleep_now_needs_a_full_u32() {
        let mut s = Stored::new();
        let o = apply(&mut s, &write(ble::CFG_SLEEP_NOW, &[30, 0]));
        assert_eq!(o.action, Action::None);
        assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
        assert_eq!(ack_id(&o), ble::CFG_SLEEP_NOW);
    }

    /// The resolver the app shares with the firmware agrees with what
    /// `apply` does, which is the only reason it is shared.
    #[test]
    fn the_app_can_predict_the_duration_before_the_ack() {
        for (asked, cadence) in [(0, 0), (0, 120), (30, 0), (1, 90), (99_999, 10)] {
            let mut s = Stored::new();
            s.sleep_interval_s = cadence;
            let predicted = ble::resolve_sleep_now(asked, cadence);
            assert_eq!(
                apply(&mut s, &u32_write(ble::CFG_SLEEP_NOW, asked)).action,
                Action::SleepNow(predicted),
                "asked {asked}, cadence {cadence}"
            );
        }
    }

    /// The file only speaks about what it mentions. A push that changes the
    /// beacon interval must not reset a duty cycle set from the app, and the
    /// no-change return is what keeps a board from rewriting the same flash
    /// record on every boot.
    #[test]
    fn an_absent_power_key_leaves_the_live_value_alone() {
        let mut stored = Stored {
            sleep_interval_s: 60,
            adv_window_s: 10,
            ble_off_s: 45,
            ..Stored::new()
        };
        let before = stored;

        assert!(!stored.adopt_power(&crate::radiocfg::PowerConfig::default()));
        assert_eq!(stored, before);

        // One key present changes that key and nothing else.
        let p = crate::radiocfg::PowerConfig::default().with(Knob::BleOff, 30);
        assert!(stored.adopt_power(&p));
        assert_eq!(stored.ble_off_s, 30);
        assert_eq!(stored.sleep_interval_s, 60);
        assert_eq!(stored.adv_window_s, 10);

        // Adopting the same file again is a no-op, so no flash write.
        assert!(!stored.adopt_power(&p));
    }

    /// A file that explicitly asks for zero turns the duty cycle off, rather
    /// than reading as "unset" - the distinction the `Option` exists for.
    #[test]
    fn an_explicit_zero_turns_the_duty_cycle_off() {
        let mut stored = Stored {
            ble_off_s: 45,
            ..Stored::new()
        };
        let p = crate::radiocfg::PowerConfig::default().with(Knob::BleOff, 0);
        assert!(stored.adopt_power(&p));
        assert_eq!(stored.ble_off_s, 0);
    }

    // -- the serve loop ----------------------------------------------------

    /// A tracker with an off period: the budget is the on period, expiry
    /// drops the modem, and a disconnect lingers.
    #[test]
    fn serve_runs_a_tracker_s_duty_cycle() {
        let s = Stored {
            mode: Mode::Tracking,
            ble_off_s: 30,
            ble_on_s: 20,
            ..Stored::new()
        };
        let mut serve = Serve::new(0, &s);
        assert_eq!(serve.mode(), Mode::Tracking);
        assert_eq!(serve.pass(0, &s), (false, Next::Advertise { bounded: true }));
        assert_eq!(serve.remaining_ms(0), 20_000);
        assert_eq!(
            serve.on_accept(5_000, Accepted::Connected, &s),
            Step {
                promote: false,
                then: Then::Serve
            }
        );
        // The session outlasts the budget; the disconnect buys a linger.
        assert_eq!(serve.on_session_end(60_000, None, &s), Then::Continue);
        assert_eq!(serve.remaining_ms(60_000), LINGER_S * 1000);
        assert_eq!(serve.pass(60_000, &s), (false, Next::Advertise { bounded: true }));
        assert_eq!(serve.pass(65_000, &s), (false, Next::BleDown { off_s: 30 }));
        assert_eq!(during_ble_down(ServeCommand::SleepNow(9)), Then::Sleep(9));
        assert_eq!(during_ble_down(ServeCommand::ModeChanged), Then::Continue);
        // Expiry while advertising says the same thing.
        let mut serve = Serve::new(0, &s);
        assert_eq!(serve.on_accept(20_000, Accepted::Expired, &s).then, Then::BleDown(30));
    }

    /// A wake check: a connect attempt, completed or not, promotes it to
    /// idle on the idle budget; nobody coming puts it back to sleep.
    #[test]
    fn serve_promotes_a_wake_check_on_the_attempt() {
        let s = Stored {
            mode: Mode::Stored,
            sleep_interval_s: 120,
            adv_window_s: 15,
            idle_timeout_s: 600,
            ..Stored::new()
        };
        for attempt in [Accepted::Connected, Accepted::Failed] {
            let mut serve = Serve::new(0, &s);
            let step = serve.on_accept(8_000, attempt, &s);
            assert!(step.promote, "{attempt:?}");
            assert_eq!(serve.mode(), Mode::Idle);
            assert_eq!(serve.remaining_ms(8_000), 600_000);
            // The firmware records the promotion; the next pass finds the
            // stored mode equal to the budgeted one and keeps the budget.
            let idle = Stored {
                mode: Mode::Idle,
                ..s
            };
            assert_eq!(serve.pass(20_000, &idle), (false, Next::Advertise { bounded: true }));
            assert_eq!(serve.pass(608_000, &idle), (false, Next::Sleep { interval_s: 120 }));
        }
        let mut serve = Serve::new(0, &s);
        assert_eq!(serve.on_accept(15_000, Accepted::Expired, &s).then, Then::Sleep(120));
        assert_eq!(serve.on_accept(3_000, Accepted::Command(ServeCommand::SleepNow(45)), &s).then, Then::Sleep(45));
    }

    /// A mode that moved under the loop re-budgets on its own terms.
    #[test]
    fn serve_rebudgets_when_the_mode_moves() {
        let mut s = Stored {
            mode: Mode::Idle,
            sleep_interval_s: 120,
            idle_timeout_s: 600,
            ..Stored::new()
        };
        let mut serve = Serve::new(0, &s);
        assert_eq!(serve.remaining_ms(0), 600_000);
        assert_eq!(serve.on_accept(1_000, Accepted::Command(ServeCommand::ModeChanged), &s).then, Then::Continue);
        s.mode = Mode::Tracking;
        let (rebudgeted, pass) = serve.pass(1_000, &s);
        assert!(rebudgeted);
        assert_eq!(pass, Next::Advertise { bounded: false });
        assert_eq!(serve.mode(), Mode::Tracking);
        assert_eq!(serve.remaining_ms(1_000), u64::from(ble::BLE_ON_DEFAULT_S) * 1000);
        // Listening never bounds its wait.
        s.mode = Mode::Listening;
        assert_eq!(serve.pass(1_000, &s).1, Next::Advertise { bounded: false });
    }

    /// A sleep asked for during the session is honored after it, and a
    /// failed attempt holds the window without extending a long one.
    #[test]
    fn serve_sleeps_after_the_session_that_asked() {
        let s = Stored {
            mode: Mode::Idle,
            idle_timeout_s: 600,
            ..Stored::new()
        };
        let mut serve = Serve::new(0, &s);
        assert_eq!(serve.on_session_end(5_000, Some(90), &s), Then::Sleep(90));
        let mut serve = Serve::new(0, &s);
        assert_eq!(serve.on_accept(1_000, Accepted::Failed, &s).then, Then::Retry);
        assert_eq!(serve.window().ends_ms(), 600_000, "an early failure cannot shorten idle");
    }

    // -- dispatch ----------------------------------------------------------

    /// Each action reaches the loop that owns it: overrides and modes to
    /// the hardware loop, a nap and a store to the serve loop, and the
    /// settings the loops read for themselves nowhere.
    #[test]
    fn dispatch_sends_each_action_to_the_loop_that_owns_it() {
        use crate::posture::Request;
        let s = Stored {
            sleep_interval_s: 120,
            ..Stored::new()
        };
        let none = Dispatch::default();
        assert_eq!(
            dispatch(Action::GpsSleep(true), &s),
            Dispatch {
                request: Some(Request::GpsSleep(true)),
                ..none
            }
        );
        assert_eq!(
            dispatch(Action::RadioStandby(false), &s),
            Dispatch {
                request: Some(Request::RadioStandby(false)),
                ..none
            }
        );
        assert_eq!(
            dispatch(Action::SetMode(Mode::Tracking), &s),
            Dispatch {
                request: Some(Request::Mode(Mode::Tracking)),
                command: Some(ServeCommand::ModeChanged),
                ..none
            }
        );
        // A store is a sleep on the cadence, not a hardware request: the
        // park happens on the way into the sleep.
        assert_eq!(
            dispatch(Action::SetMode(Mode::Stored), &s),
            Dispatch {
                command: Some(ServeCommand::SleepNow(120)),
                ..none
            }
        );
        assert_eq!(
            dispatch(Action::SetMode(Mode::Stored), &Stored::new()).command,
            Some(ServeCommand::SleepNow(ble::STORE_DEFAULT_S))
        );
        assert_eq!(
            dispatch(Action::SleepNow(30), &s),
            Dispatch {
                command: Some(ServeCommand::SleepNow(30)),
                ..none
            }
        );
        assert_eq!(
            dispatch(Action::NotifyInterval(500), &s),
            Dispatch {
                notify_interval_ms: Some(500),
                ..none
            }
        );
        for a in [
            Action::Knob(Knob::SleepInterval, 60),
            Action::Knob(Knob::AdvWindow, 10),
            Action::Knob(Knob::BleOff, 30),
            Action::Knob(Knob::BleOn, 20),
            Action::Knob(Knob::IdleTimeout, 0),
            Action::Name,
            Action::None,
        ] {
            assert_eq!(dispatch(a, &s), none, "{a:?}");
        }
    }

    // -- the knob table ----------------------------------------------------

    /// One row per knob, in the enum's order, with distinct ids and
    /// distinct, word-aligned offsets in both layouts - and the resolve and
    /// clamp rules the write path and the read path share.
    #[test]
    fn the_knob_table_is_consistent() {
        for (i, spec) in KNOBS.iter().enumerate() {
            assert_eq!(spec.knob as usize, i, "{:?} is out of order", spec.knob);
            assert_eq!(Knob::ALL[i], spec.knob);
            assert_eq!(Knob::from_id(spec.id), Some(spec.knob));
            assert_eq!(Knob::from_name(spec.name), Some(spec.knob));
            assert_eq!(spec.record_at % 4, 0);
            assert_eq!(spec.wire_at % 4, 0);
            assert!(spec.record_at + 4 <= RECORD_LEN - 4);
            assert!(spec.wire_at + 4 <= ble::SETTINGS_LEN);
            assert!(spec.min <= spec.max);
            for other in KNOBS.iter().skip(i + 1) {
                assert_ne!(spec.id, other.id);
                assert_ne!(spec.name, other.name);
                assert_ne!(spec.record_at, other.record_at);
                assert_ne!(spec.wire_at, other.wire_at);
            }
            // A write is clamped the way the zero rule says, and a read
            // resolves what a write could have stored.
            assert_eq!(spec.clamp(spec.max + 1), spec.max);
            assert_eq!(spec.clamp(spec.min), spec.min);
            match spec.zero {
                Zero::Off => {
                    assert_eq!(spec.clamp(0), 0);
                    assert_eq!(spec.resolve(0), 0);
                    assert!(spec.accepts(0));
                }
                Zero::Default(d) => {
                    assert_eq!(spec.clamp(0), spec.min);
                    assert_eq!(spec.resolve(0), d);
                    assert!((spec.min..=spec.max).contains(&d));
                }
            }
            assert!(!spec.accepts(u64::from(spec.max) + 1));
            // The accessors and the fields agree.
            let mut s = Stored::new();
            spec.set(&mut s, 42);
            assert_eq!(spec.get(&s), 42);
            assert_eq!(s.knob_raw(spec.knob), 42);
            let mut w = ble::Settings::default();
            spec.wire_set(&mut w, 7);
            assert_eq!(spec.wire_get(&w), 7);
        }
        // The fields the knobs sit beside in the record.
        assert_eq!(RECORD_LEN, 56);
    }

    /// Every knob is written by its id, echoed as clamped, and comes back
    /// resolved from the settings and intact from the record.
    #[test]
    fn every_knob_writes_reads_and_persists() {
        for spec in &KNOBS {
            let mut s = Stored::new();
            let o = apply(&mut s, &u32_write(spec.id, spec.min + 1));
            assert_eq!(o.action, Action::Knob(spec.knob, spec.min + 1));
            assert!(o.save);
            assert_eq!(ack_u32(&o), spec.min + 1);
            assert_eq!(s.knob(spec.knob), spec.min + 1);
            assert_eq!(spec.wire_get(&s.settings(1_000)), spec.min + 1);
            let back = Stored::decode_record(&s.encode_record()).expect("record");
            assert_eq!(back.knob(spec.knob), spec.min + 1);
            // A short value is refused by id.
            let o = apply(&mut s, &write(spec.id, &[1, 2]));
            assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
            assert_eq!(ack_id(&o), spec.id);
        }
    }

    /// The rail this board does not have is refused as the unknown id it
    /// now is, with nothing stored.
    #[test]
    fn the_old_rail_id_is_unknown() {
        let mut s = Stored::new();
        let o = apply(&mut s, &write(0x10, &[1]));
        assert_eq!(o.action, Action::None);
        assert_eq!(ack_status(&o), packet::ACK_UNKNOWN_ID);
        assert_eq!(s, Stored::new());
    }
}
