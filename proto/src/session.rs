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

use crate::ble::{self, Mode};
use crate::link;
use gps_proto::packet;

// ---------------------------------------------------------------------------
// Settings that survive a sleep and a power cycle
// ---------------------------------------------------------------------------

/// The GPS/LoRa rail is off.
///
/// Stored inverted so an all-zero [`Stored`] - a cold boot with nothing
/// saved - is a board that powers its GPS, which is what an unconfigured
/// board should do.
pub const PFLAG_PWR_OFF: u32 = 1 << 0;
/// The radio was asked into standby.
///
/// Named for the WIO-E5's soft sleep, which existed because the ESP32-C6
/// could not power the second MCU down mid-session. One MCU has one sleep
/// story, so what survives is the radio half: standby instead of receive.
pub const PFLAG_WIO_SLEEP: u32 = 1 << 1;
/// The GPS was asked to enter backup mode.
pub const PFLAG_GPS_SLEEP: u32 = 1 << 2;

/// Everything a config write can change that outlives the connection, in
/// the words the firmware keeps in RTC RAM and mirrors to flash.
///
/// The notify interval is deliberately not here: it is per-session state
/// that resets with the board (see [`Action::NotifyInterval`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stored {
    /// Deep-sleep wake-check interval in seconds, 0 = stay awake.
    pub sleep_interval_s: u32,
    /// `PFLAG_*` bits.
    pub flags: u32,
    /// Advertising window per wake check, 0 = never configured. Read it
    /// through [`Stored::adv_window`], which substitutes the default.
    pub adv_window_s: u32,
    /// How long the BLE controller stays powered down between advertising
    /// windows, 0 = never take it down.
    ///
    /// This is the awake-state counterpart to `sleep_interval_s`. Deep
    /// sleep takes the whole chip away and costs a full reset; this takes
    /// only the BLE modem away, which is 71 mA of the board's 126, and
    /// leaves the LoRa beacon and the GPS running. A board doing this is
    /// still a working tracker and is still logging - it just cannot be
    /// connected to until the next window.
    ///
    /// Honored in [`Mode::Tracking`] only: Idle exists to be reachable and
    /// Stored has no controller to duty-cycle.
    pub ble_off_s: u32,
    /// What the board is doing, and - once [`Mode::persisted`] has had it -
    /// what it comes back doing after a flat cell.
    ///
    /// The RTC copy holds the live mode, [`Mode::Idle`] included; only
    /// [`Stored::encode_record`] normalizes it, so flash never says idle.
    pub mode: Mode,
    /// How long [`Mode::Idle`] lasts before the board stores itself, 0 =
    /// never configured. Read it through [`Stored::idle_timeout`], which
    /// substitutes the default.
    pub idle_timeout_s: u32,
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
        }
    }

    pub fn pwr_en(&self) -> bool {
        self.flags & PFLAG_PWR_OFF == 0
    }

    pub fn wio_sleep(&self) -> bool {
        self.flags & PFLAG_WIO_SLEEP != 0
    }

    pub fn gps_sleep(&self) -> bool {
        self.flags & PFLAG_GPS_SLEEP != 0
    }

    /// How long a wake check advertises for. A stored 0 means never
    /// configured, not "do not advertise" - a zero window would leave a
    /// sleeping board unreachable by anything but a physical reset, so it
    /// resolves to the default instead.
    pub fn adv_window(&self) -> u32 {
        match self.adv_window_s {
            0 => ble::ESP_ADV_DEFAULT_S,
            s => s,
        }
    }

    /// How long [`Mode::Idle`] runs before the board stores itself. A
    /// stored 0 means never configured and resolves to the default, exactly
    /// as the advertising window does.
    ///
    /// There is no value here that means "never". A board leaves Idle by
    /// deep-sleeping, so a `sleep_interval_s` of 0 already is "never": with
    /// no cadence to sleep on there is nowhere for the timeout to send the
    /// board, and it stays awake and reachable. That is the bench setting,
    /// and it is what an unconfigured board does.
    pub fn idle_timeout(&self) -> u32 {
        match self.idle_timeout_s {
            0 => ble::IDLE_TIMEOUT_DEFAULT_S,
            s => s,
        }
    }

    /// The cadence a board that has been *told* to store itself sleeps on.
    ///
    /// `sleep_interval_s` when it is set. When it is not, an explicit
    /// `CFG_MODE stored` is still an unambiguous instruction - the same
    /// reading [`ble::resolve_sleep_now`] gives a commanded nap - so it
    /// borrows the ceiling rather than being ignored. The passive path (an
    /// idle timeout) does not do this: nobody asked for that one, so a
    /// board with deep sleep off simply stays awake.
    pub fn sleep_cadence(&self) -> u32 {
        match self.sleep_interval_s {
            0 => ble::ESP_SLEEP_MAX_S,
            s => s,
        }
    }

    /// How long the board advertises before [`Stored::at_expiry`] decides
    /// something, which is a different budget in each mode.
    ///
    /// A wake check gets the advertising window - long enough for a phone
    /// to catch it, short enough to be paid for every interval forever.
    /// Idle gets the whole idle timeout, because being reachable is the
    /// entire point of the state. Tracking gets the advertising window
    /// again, as the on-half of the `ble_off_s` duty cycle.
    pub fn budget_s(&self) -> u32 {
        match self.mode {
            Mode::Stored | Mode::Tracking => self.adv_window(),
            Mode::Idle => self.idle_timeout(),
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
            // no cadence to sleep on the board simply keeps advertising,
            // which is what a bench board and an unconfigured board both
            // want.
            Mode::Idle => match self.sleep_interval_s {
                0 => Next::Advertise,
                interval_s => Next::Sleep { interval_s },
            },
            // A tracker never deep-sleeps on a cadence - that would stop
            // the beacon, the logging and the listening, which is the whole
            // job. What it can drop is the modem.
            Mode::Tracking => match self.ble_off_s {
                0 => Next::Advertise,
                off_s => Next::BleDown { off_s },
            },
        }
    }

    /// Whether the GPS/LoRa rail comes up with the board.
    ///
    /// A sleep-interval wake check comes up dark whatever the app asked
    /// for: the question a wake check exists to ask is whether anyone wants
    /// the board back, which needs BLE only, so a wake nobody answers never
    /// pays for the GPS. A connect raises the rail afterwards. A cold boot
    /// follows the configured setting.
    ///
    /// The wio-s3-max-gps board has no such rail - the GPS and SD sit on
    /// +3V3 - so its firmware logs [`Action::Rail`] and does nothing. The
    /// policy stays here because it is tested, and a board respin could
    /// bring the switch back.
    pub fn rail_at_boot(&self, woke_from_sleep: bool) -> bool {
        !woke_from_sleep && self.pwr_en()
    }

    fn set_flag(&mut self, flag: u32, on: bool) {
        if on {
            self.flags |= flag;
        } else {
            self.flags &= !flag;
        }
    }

    /// The settings characteristic value, which is the only way an app
    /// learns the board's current state on connect.
    pub fn settings(&self, notify_interval_ms: u32) -> ble::Settings {
        ble::Settings {
            pwr_en: self.pwr_en(),
            wio_sleep: self.wio_sleep(),
            gps_sleep: self.gps_sleep(),
            sleep_interval_s: self.sleep_interval_s,
            notify_interval_ms,
            adv_window_s: self.adv_window(),
            ble_off_s: self.ble_off_s,
            mode: self.mode,
            idle_timeout_s: self.idle_timeout(),
        }
    }

    /// Fold a config file's `[power]` section into these settings, and say
    /// whether that changed anything.
    ///
    /// Only the keys the file actually carried are applied - an absent one
    /// leaves the board's live value alone, which is the whole reason
    /// [`crate::radiocfg::PowerConfig`] is optional field by field. The
    /// parser has already range-checked whatever is present, so there is
    /// nothing to clamp here.
    ///
    /// The return value is what decides whether the caller pays for a flash
    /// write: a board that reads the same card on every boot would otherwise
    /// rewrite the same record forever.
    pub fn adopt_power(&mut self, p: &crate::radiocfg::PowerConfig) -> bool {
        let before = *self;
        if let Some(s) = p.ble_off_s {
            self.ble_off_s = s;
        }
        if let Some(s) = p.adv_window_s {
            self.adv_window_s = s;
        }
        if let Some(s) = p.sleep_interval_s {
            self.sleep_interval_s = s;
        }
        if let Some(s) = p.idle_timeout_s {
            self.idle_timeout_s = s;
        }
        *self != before
    }

    /// Encode the flash record. Erased-flash bytes past the end are none of
    /// this function's business; the crc covers what it wrote.
    pub fn encode_record(&self) -> [u8; RECORD_LEN] {
        let mut rec = [0u8; RECORD_LEN];
        rec[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&RECORD_VERSION.to_le_bytes());
        rec[8..12].copy_from_slice(&self.sleep_interval_s.to_le_bytes());
        rec[12..16].copy_from_slice(&self.flags.to_le_bytes());
        rec[16..20].copy_from_slice(&self.adv_window_s.to_le_bytes());
        rec[20..24].copy_from_slice(&self.ble_off_s.to_le_bytes());
        // Normalized on the way out, and only here: the RTC copy may say
        // idle, but a board that came back from a reset still believing it
        // was idle would sit at awake current with nobody coming.
        rec[24..28].copy_from_slice(&(self.mode.persisted().as_wire() as u32).to_le_bytes());
        rec[28..32].copy_from_slice(&self.idle_timeout_s.to_le_bytes());
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
        let (crc_at, adv_window_s, ble_off_s, mode, idle_timeout_s) = match word(4) {
            2 => (V2_CRC_AT, 0, 0, Mode::Stored, 0),
            3 => (V3_CRC_AT, word(16), 0, Mode::Stored, 0),
            4 => (V4_CRC_AT, word(16), word(20), Mode::Stored, 0),
            RECORD_VERSION => (
                RECORD_LEN - 4,
                word(16),
                word(20),
                // A mode this build does not know reads as the safe one: a
                // board that stores itself can be woken, where one that
                // guessed at tracking would run its cell down.
                Mode::from_wire(word(24) as u8).unwrap_or_default(),
                word(28),
            ),
            _ => return None,
        };
        if word(crc_at) != link::crc32(&rec[0..crc_at]) {
            return None;
        }
        Some(Self {
            sleep_interval_s: word(8),
            flags: word(12),
            adv_window_s,
            ble_off_s,
            mode,
            idle_timeout_s,
        })
    }
}

/// Marks the record as this firmware's ("midA").
pub const RECORD_MAGIC: u32 = 0x6D69_6441;

/// Layout version of the flash record.
///
/// Version 2 dropped a separate stow interval; a version 1 record is
/// discarded rather than misread. Version 3 appended the advertising
/// window, version 4 the BLE off period, and version 5 the [`Mode`] and its
/// idle timeout. All are pure appends, so an older record still reads - a
/// board updated in the field keeps the cadence it was left on instead of
/// coming back advertising continuously.
///
/// A record from before version 5 carries no mode, which reads as
/// [`Mode::Stored`]: an updated board comes back reachable (a cold boot
/// lands in [`Mode::Idle`] whatever the record says) and then stores itself
/// on the cadence it already had.
pub const RECORD_VERSION: u32 = 5;

/// magic, version, sleep interval, flags, advertising window, BLE off
/// period, mode, idle timeout, crc32 - all `u32`, so the length is already
/// a multiple of the flash write word.
pub const RECORD_LEN: usize = 36;

/// Where the crc sits in a version 2 record, which is this layout minus its
/// last five words.
const V2_CRC_AT: usize = 16;
/// Where the crc sits in a version 3 record - this layout minus three.
const V3_CRC_AT: usize = 20;
/// Where the crc sits in a version 4 record - this layout minus two.
const V4_CRC_AT: usize = 24;

// ---------------------------------------------------------------------------
// Config characteristic writes
// ---------------------------------------------------------------------------

/// What the firmware still has to do once [`apply`] has folded a config
/// write into the settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Drive the GPS/LoRa rail to this level.
    Rail(bool),
    /// Put the radio into standby (`true`) or bring it back.
    ///
    /// This was a link frame and a wait for the WIO's answer, so the ack
    /// the policy built was provisional. Same-chip it is a signal the
    /// hardware loop picks up, and the ack always holds.
    WioSleep(bool),
    /// Put the GPS into (`true`) or out of backup mode.
    GpsSleep(bool),
    /// The deep-sleep wake-check interval is now this many seconds
    /// (0 = sleep off). Already stored and already clamped.
    SleepInterval(u32),
    /// The advertising window per wake check is now this many seconds.
    /// Takes effect on the next wake, not the window already running.
    AdvWindow(u32),
    /// The BLE controller now stays down for this many seconds between
    /// advertising windows (0 = never take it down). Takes effect at the
    /// end of the window already running, not immediately - a central that
    /// just set it keeps its connection.
    BleOff(u32),
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
    /// The idle timeout is now this many seconds. Already stored and
    /// clamped; it takes effect at the next re-arm rather than shortening
    /// the timeout already running.
    IdleTimeout(u32),
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
    /// after its battery goes flat - the sleep interval, the advertising
    /// window and the rail. The two WIO sleep flags are not saved: they are
    /// re-applied over the link when it next comes up, and a board that
    /// cold-boots with its GPS running is the safer of the two failures.
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
/// working against it. Writes are applied optimistically - the flag is set
/// before the WIO is asked - because the settings characteristic has to
/// report what the app asked for even while the WIO is unreachable, which
/// on this board is most of the time (the rail is off through every sleep).
pub fn apply(stored: &mut Stored, data: &[u8]) -> Outcome {
    if data.len() >= 2 {
        let id = data[0];
        let len = data[1] as usize;
        // A length running past the write leaves the value empty, which the
        // per-id checks below reject; only the flag ids default it.
        let value = data.get(2..2 + len).unwrap_or(&[]);
        match id {
            ble::CFG_PWR_EN => {
                // A bare write with no value means "on": the rail is what
                // makes the board a tracker, so the ambiguous case powers it.
                let on = value.first().copied().unwrap_or(1) != 0;
                stored.set_flag(PFLAG_PWR_OFF, !on);
                return Outcome::new(Action::Rail(on), true, id, packet::ACK_OK, &[on as u8]);
            }
            ble::CFG_WIO_SLEEP => {
                let sleep = value.first().copied().unwrap_or(0) != 0;
                stored.set_flag(PFLAG_WIO_SLEEP, sleep);
                return Outcome::new(
                    Action::WioSleep(sleep),
                    false,
                    id,
                    packet::ACK_OK,
                    &[sleep as u8],
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
            ble::CFG_ESP_SLEEP_S => {
                let Ok(bytes) = <[u8; 4]>::try_from(value) else {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                };
                let mut secs = u32::from_le_bytes(bytes);
                // 0 is "sleep off" rather than a very short interval, so it
                // is the one value that does not come up to the floor.
                if secs > 0 {
                    secs = secs.clamp(ble::ESP_SLEEP_MIN_S, ble::ESP_SLEEP_MAX_S);
                }
                stored.sleep_interval_s = secs;
                return Outcome::new(
                    Action::SleepInterval(secs),
                    true,
                    id,
                    packet::ACK_OK,
                    &secs.to_le_bytes(),
                );
            }
            ble::CFG_ESP_ADV_WINDOW_S => {
                let Ok(bytes) = <[u8; 4]>::try_from(value) else {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                };
                // Clamped unconditionally: unlike the sleep interval, 0 is
                // not an "off" here, so it comes up to the floor instead of
                // being stored as a window nobody could ever connect in.
                let secs = u32::from_le_bytes(bytes).clamp(ble::ESP_ADV_MIN_S, ble::ESP_ADV_MAX_S);
                stored.adv_window_s = secs;
                return Outcome::new(
                    Action::AdvWindow(secs),
                    true,
                    id,
                    packet::ACK_OK,
                    &secs.to_le_bytes(),
                );
            }
            ble::CFG_BLE_OFF_S => {
                let Ok(bytes) = <[u8; 4]>::try_from(value) else {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                };
                // 0 means off, like the sleep interval and unlike the
                // advertising window: a board that never takes BLE down is
                // the old behavior and has to stay reachable as one.
                let asked = u32::from_le_bytes(bytes);
                let secs = if asked == 0 {
                    0
                } else {
                    asked.clamp(ble::BLE_OFF_MIN_S, ble::BLE_OFF_MAX_S)
                };
                stored.ble_off_s = secs;
                return Outcome::new(
                    Action::BleOff(secs),
                    true,
                    id,
                    packet::ACK_OK,
                    &secs.to_le_bytes(),
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
            ble::CFG_IDLE_TIMEOUT_S => {
                let Ok(bytes) = <[u8; 4]>::try_from(value) else {
                    return Outcome::reject(id, packet::ACK_BAD_VALUE);
                };
                // Clamped unconditionally, like the advertising window and
                // unlike the two intervals where 0 means off: here 0 means
                // never configured, so storing it would be storing a
                // timeout that reads back as the default anyway.
                let secs = u32::from_le_bytes(bytes)
                    .clamp(ble::IDLE_TIMEOUT_MIN_S, ble::IDLE_TIMEOUT_MAX_S);
                stored.idle_timeout_s = secs;
                return Outcome::new(
                    Action::IdleTimeout(secs),
                    true,
                    id,
                    packet::ACK_OK,
                    &secs.to_le_bytes(),
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
        // the object is the one time it must.
        (Mode::Tracking, _) => Mode::Tracking,
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// Keep advertising. With a budget that bites, for at most
    /// [`Window::remaining_ms`] longer.
    Advertise,
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
    /// instead of at the next boot.
    pub fn next(&self, now_ms: u64, stored: &Stored) -> Next {
        if self.remaining_ms(now_ms) > 0 {
            Next::Advertise
        } else {
            stored.at_expiry()
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

#[cfg(test)]
mod tests {
    use super::*;

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

    /// An idle board with `interval_s` to store itself on. `0` is a board
    /// with deep sleep off, which is where "never store this board" lives.
    fn idle(interval_s: u32) -> Stored {
        Stored {
            sleep_interval_s: interval_s,
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
        assert!(s.pwr_en());
        assert!(!s.wio_sleep());
        assert!(!s.gps_sleep());
        assert_eq!(s.sleep_interval_s, 0);
        assert_eq!(s.adv_window(), ble::ESP_ADV_DEFAULT_S);
        assert_eq!(
            s.settings(packet::UPDATE_INTERVAL_DEFAULT_MS),
            ble::Settings {
                pwr_en: true,
                wio_sleep: false,
                gps_sleep: false,
                sleep_interval_s: 0,
                notify_interval_ms: packet::UPDATE_INTERVAL_DEFAULT_MS,
                adv_window_s: ble::ESP_ADV_DEFAULT_S,
                ble_off_s: 0,
                mode: Mode::Stored,
                idle_timeout_s: ble::IDLE_TIMEOUT_DEFAULT_S,
            }
        );
    }

    /// The settings characteristic reports what is stored, over the wire an
    /// app can read it back from.
    #[test]
    fn settings_report_the_stored_state() {
        let mut s = Stored::new();
        s.set_flag(PFLAG_PWR_OFF, true);
        s.set_flag(PFLAG_WIO_SLEEP, true);
        s.sleep_interval_s = 120;
        s.adv_window_s = 30;

        let settings = s.settings(2_000);
        assert!(!settings.pwr_en);
        assert!(settings.wio_sleep);
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

    /// A wake check comes up dark even with the rail configured on, so a
    /// wake nobody answers never pays for the WIO and the GPS.
    #[test]
    fn a_wake_check_comes_up_dark() {
        let s = Stored::new();
        assert!(s.pwr_en());
        assert!(!s.rail_at_boot(true), "a wake check must come up dark");
        assert!(s.rail_at_boot(false), "a cold boot follows the setting");

        let mut off = Stored::new();
        off.set_flag(PFLAG_PWR_OFF, true);
        assert!(!off.rail_at_boot(false));
        assert!(!off.rail_at_boot(true));
    }

    // -- config writes -----------------------------------------------------

    #[test]
    fn a_rail_write_drives_and_persists_it() {
        let mut s = Stored::new();
        let o = apply(&mut s, &write(ble::CFG_PWR_EN, &[0]));
        assert_eq!(o.action, Action::Rail(false));
        assert!(o.save, "the rail must survive a flat battery");
        assert!(!s.pwr_en());
        assert_eq!(o.ack(), &[ble::CFG_PWR_EN, packet::ACK_OK, 0]);

        let o = apply(&mut s, &write(ble::CFG_PWR_EN, &[1]));
        assert_eq!(o.action, Action::Rail(true));
        assert!(s.pwr_en());
        assert_eq!(o.ack(), &[ble::CFG_PWR_EN, packet::ACK_OK, 1]);
    }

    /// A valueless rail write powers the board rather than darkening it:
    /// the rail is what makes it a tracker.
    #[test]
    fn a_valueless_rail_write_powers_the_board() {
        let mut s = Stored::new();
        s.set_flag(PFLAG_PWR_OFF, true);
        let o = apply(&mut s, &write(ble::CFG_PWR_EN, &[]));
        assert_eq!(o.action, Action::Rail(true));
        assert!(s.pwr_en());
    }

    /// The sleep flags are recorded as soon as they are asked for, before
    /// the WIO has answered, because the settings characteristic has to
    /// report what the app asked for even while the WIO is unreachable.
    #[test]
    fn a_sleep_write_records_the_request_before_the_wio_answers() {
        let mut s = Stored::new();
        let o = apply(&mut s, &write(ble::CFG_WIO_SLEEP, &[1]));
        assert_eq!(o.action, Action::WioSleep(true));
        assert!(s.wio_sleep());
        assert_eq!(o.ack(), &[ble::CFG_WIO_SLEEP, packet::ACK_OK, 1]);

        let o = apply(&mut s, &write(ble::CFG_GPS_SLEEP, &[1]));
        assert_eq!(o.action, Action::GpsSleep(true));
        assert!(s.gps_sleep());

        // And clearing them again.
        apply(&mut s, &write(ble::CFG_WIO_SLEEP, &[0]));
        apply(&mut s, &write(ble::CFG_GPS_SLEEP, &[0]));
        assert!(!s.wio_sleep());
        assert!(!s.gps_sleep());
    }

    /// The WIO sleep flags are the two settings that are not written to
    /// flash: they are re-applied over the link when it comes up, and a
    /// board that cold-boots with its GPS running is the safer failure.
    #[test]
    fn only_the_reachability_settings_reach_flash() {
        let mut s = Stored::new();
        assert!(!apply(&mut s, &write(ble::CFG_WIO_SLEEP, &[1])).save);
        assert!(!apply(&mut s, &write(ble::CFG_GPS_SLEEP, &[1])).save);
        assert!(apply(&mut s, &write(ble::CFG_PWR_EN, &[0])).save);
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
            assert_eq!(o.action, Action::SleepInterval(applied), "asked {}", asked);
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
        assert_eq!(o.action, Action::SleepInterval(0));
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
            assert_eq!(o.action, Action::AdvWindow(applied), "asked {}", asked);
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
        apply(&mut s, &write(ble::CFG_WIO_SLEEP, &[1]));
        assert_eq!(s.sleep_interval_s, 90);
        assert_eq!(s.adv_window(), 30);
        assert!(s.wio_sleep());
        assert!(s.pwr_en());
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
            assert_eq!(o.action, Action::BleOff(applied), "asked {}", asked);
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
        for data in [vec![], vec![ble::CFG_PWR_EN]] {
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
            flags: PFLAG_PWR_OFF | PFLAG_GPS_SLEEP,
            adv_window_s: 45,
            ble_off_s: 90,
            mode: Mode::Tracking,
            idle_timeout_s: 900,
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
        rec[12..16].copy_from_slice(&PFLAG_WIO_SLEEP.to_le_bytes());
        let crc = link::crc32(&rec[0..V2_CRC_AT]);
        rec[V2_CRC_AT..V2_CRC_AT + 4].copy_from_slice(&crc.to_le_bytes());

        let s = Stored::decode_record(&rec).expect("a version 2 record still reads");
        assert_eq!(s.sleep_interval_s, 120);
        assert!(s.wio_sleep());
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
        rec[12..16].copy_from_slice(&PFLAG_WIO_SLEEP.to_le_bytes());
        rec[16..20].copy_from_slice(&45u32.to_le_bytes());
        let crc = link::crc32(&rec[0..V3_CRC_AT]);
        rec[V3_CRC_AT..V3_CRC_AT + 4].copy_from_slice(&crc.to_le_bytes());

        let s = Stored::decode_record(&rec).expect("a version 3 record still reads");
        assert_eq!(s.sleep_interval_s, 120);
        assert!(s.wio_sleep());
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
        rec[12..16].copy_from_slice(&PFLAG_WIO_SLEEP.to_le_bytes());
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
        assert_eq!(s.idle_timeout(), ble::IDLE_TIMEOUT_DEFAULT_S);
        assert_eq!(boot_mode(s.mode, false), Mode::Idle);
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
        for bad in [&[3u8][..], &[255][..], &[][..]] {
            let o = apply(&mut s, &write(ble::CFG_MODE, bad));
            assert_eq!(o.action, Action::None);
            assert_eq!(ack_status(&o), packet::ACK_BAD_VALUE);
            assert_eq!(s.mode, Mode::Tracking, "a rejected write changed nothing");
        }
    }

    /// The idle timeout clamps like the advertising window: 0 is "never
    /// configured" and reads back as the default, so there is no way to
    /// store a timeout nothing could fire.
    #[test]
    fn the_idle_timeout_clamps_and_defaults() {
        let mut s = Stored::new();
        assert_eq!(s.idle_timeout_s, 0);
        assert_eq!(s.idle_timeout(), ble::IDLE_TIMEOUT_DEFAULT_S);

        let o = apply(&mut s, &u32_write(ble::CFG_IDLE_TIMEOUT_S, 900));
        assert_eq!(o.action, Action::IdleTimeout(900));
        assert!(o.save);
        assert_eq!(ack_u32(&o), 900);
        assert_eq!(s.idle_timeout(), 900);

        for (asked, applied) in [
            (0, ble::IDLE_TIMEOUT_MIN_S),
            (1, ble::IDLE_TIMEOUT_MIN_S),
            (u32::MAX, ble::IDLE_TIMEOUT_MAX_S),
        ] {
            let o = apply(&mut s, &u32_write(ble::CFG_IDLE_TIMEOUT_S, asked));
            assert_eq!(ack_u32(&o), applied);
            assert_eq!(s.idle_timeout_s, applied);
        }
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

        // Idle stores itself, on the same cadence but its own budget.
        let idle = Stored {
            mode: Mode::Idle,
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
        assert_eq!(tracking.budget_s(), tracking.adv_window());
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
        assert_eq!(s.at_expiry(), Next::Advertise);
        let w = Window::new(0, s.budget_s());
        assert_eq!(w.next(60_000, &s), Next::Advertise);
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
        assert_eq!(s.sleep_cadence(), ble::ESP_SLEEP_MAX_S);
        assert_eq!(
            s.at_expiry(),
            Next::Sleep {
                interval_s: ble::ESP_SLEEP_MAX_S
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
        assert_eq!(promoted.at_expiry(), Next::Advertise);
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
        assert_eq!(w.next(20_000, &s), Next::Advertise, "the wake check would have slept");
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
        assert_eq!(w.next(0, &s), Next::Advertise);
        assert_eq!(w.next(15_000, &s), Next::Advertise);
        assert_eq!(w.next(u64::MAX, &s), Next::Advertise);
    }

    /// The wake advertises for the configured window and then sleeps for
    /// the configured interval.
    #[test]
    fn a_spent_window_ends_in_deep_sleep() {
        let s = cadence(60);
        let w = Window::new(1_000, 15);
        assert_eq!(w.ends_ms(), 16_000);
        assert_eq!(w.next(1_000, &s), Next::Advertise);
        assert_eq!(w.remaining_ms(1_000), 15_000);
        assert_eq!(w.next(15_999, &s), Next::Advertise);
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
        while let Next::Advertise = w.next(now, &s) {
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
        assert_eq!(w.next(tried_at, &s), Next::Advertise);
        assert_eq!(
            w.next(15_100, &s),
            Next::Advertise,
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
        assert_eq!(w.next(disconnected_at, &s), Next::Advertise);
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
        assert_eq!(w.next(20_000, &idle(0)), Next::Advertise);
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
            assert!(!s.rail_at_boot(true), "a wake check comes up dark");
            let w = Window::new(now, s.budget_s());
            assert_eq!(w.next(now, &s), Next::Advertise);
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
        let p = crate::radiocfg::PowerConfig {
            ble_off_s: Some(30),
            ..Default::default()
        };
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
        let p = crate::radiocfg::PowerConfig {
            ble_off_s: Some(0),
            ..Default::default()
        };
        assert!(stored.adopt_power(&p));
        assert_eq!(stored.ble_off_s, 0);
    }
}
