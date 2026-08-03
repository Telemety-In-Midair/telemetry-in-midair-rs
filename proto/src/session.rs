//! What a BLE session does to the board: how a config write lands in the
//! settings, what the settings characteristic reports back, and when an
//! unattended board sleeps between advertising windows.
//!
//! These decisions live in the shared crate rather than inline in the
//! ESP32-C6 connection handler because they are the part where a mistake is
//! expensive: a board that mis-clamps an advertising window, or that lets a
//! retry restart its advertising budget, stops being reachable over the air
//! at all - and a `no_std` binary cannot be tested on the host.
//!
//! Everything here is pure. The firmware supplies the clock and performs
//! the effects ([`Action`]); nothing below touches a timer, flash or radio.

use crate::ble;
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
/// The WIO-E5 was asked to enter soft sleep.
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
}

impl Stored {
    pub const fn new() -> Self {
        Self {
            sleep_interval_s: 0,
            flags: 0,
            adv_window_s: 0,
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

    /// Whether the GPS/LoRa rail comes up with the board.
    ///
    /// A sleep-interval wake check comes up dark whatever the app asked
    /// for: the question a wake check exists to ask is whether anyone wants
    /// the board back, which needs BLE only, so a wake nobody answers never
    /// pays for the WIO and the GPS. A connect raises the rail afterwards.
    /// A cold boot follows the configured setting.
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
        }
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
        let crc = link::crc32(&rec[0..20]);
        rec[20..24].copy_from_slice(&crc.to_le_bytes());
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
        // A version 2 record stops one word short and carries no window,
        // which reads back as the default. Its trailing bytes are erased
        // flash, so the crc has to be checked where that version put it.
        let (crc_at, adv_window_s) = match word(4) {
            2 => (V2_CRC_AT, 0),
            RECORD_VERSION => (RECORD_LEN - 4, word(RECORD_LEN - 8)),
            _ => return None,
        };
        if word(crc_at) != link::crc32(&rec[0..crc_at]) {
            return None;
        }
        Some(Self {
            sleep_interval_s: word(8),
            flags: word(12),
            adv_window_s,
        })
    }
}

/// Marks the record as this firmware's ("midA").
pub const RECORD_MAGIC: u32 = 0x6D69_6441;

/// Layout version of the flash record.
///
/// Version 2 dropped a separate stow interval; a version 1 record is
/// discarded rather than misread. Version 3 appended the advertising
/// window, which is a pure append, so a version 2 record still reads - a
/// board updated in the field keeps the cadence it was left on instead of
/// coming back advertising continuously.
pub const RECORD_VERSION: u32 = 3;

/// magic, version, sleep interval, flags, advertising window, crc32 - all
/// `u32`, so the length is already a multiple of the flash write word.
pub const RECORD_LEN: usize = 24;

/// Where the crc sits in a version 2 record, which is this layout minus its
/// last word.
const V2_CRC_AT: usize = 16;

// ---------------------------------------------------------------------------
// Config characteristic writes
// ---------------------------------------------------------------------------

/// What the firmware still has to do once [`apply`] has folded a config
/// write into the settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Drive the GPS/LoRa rail to this level.
    Rail(bool),
    /// Ask the WIO to enter (`true`) or leave soft sleep. The ack in the
    /// [`Outcome`] only holds if the WIO answers; a link failure replaces
    /// it, and a wake that times out is retried as a reset pulse.
    WioSleep(bool),
    /// Ask the WIO to put the GPS into (`true`) or out of backup mode.
    GpsSleep(bool),
    /// The deep-sleep wake-check interval is now this many seconds
    /// (0 = sleep off). Already stored and already clamped.
    SleepInterval(u32),
    /// The advertising window per wake check is now this many seconds.
    /// Takes effect on the next wake, not the window already running.
    AdvWindow(u32),
    /// Set the position notify interval, in ms, already clamped. Not
    /// persisted: it is per-session state.
    NotifyInterval(u32),
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

/// What the board should do right now.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// Keep advertising. With sleep enabled, for at most
    /// [`Window::remaining_ms`] longer.
    Advertise,
    /// Deep sleep for this many seconds.
    Sleep { interval_s: u32 },
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

    /// Advertise, or sleep because this wake's budget is spent.
    ///
    /// The interval is passed in per call rather than captured, so enabling
    /// or disabling sleep mode over BLE takes effect at the next decision
    /// instead of at the next boot.
    pub fn next(&self, now_ms: u64, sleep_interval_s: u32) -> Next {
        if sleep_interval_s > 0 && now_ms >= self.ends_ms {
            Next::Sleep {
                interval_s: sleep_interval_s,
            }
        } else {
            Next::Advertise
        }
    }

    /// Give the phone that just disconnected a chance to come straight
    /// back.
    pub fn linger(&mut self, now_ms: u64) {
        self.ends_ms = now_ms.saturating_add(LINGER_S * 1000);
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
    }

    /// Anything that is not this firmware's record leaves the board
    /// unconfigured rather than acting on someone else's bytes.
    #[test]
    fn a_record_that_is_not_ours_is_ignored() {
        let good = Stored {
            sleep_interval_s: 60,
            flags: 0,
            adv_window_s: 15,
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
    /// window says - the window only paces a wake check.
    #[test]
    fn sleep_off_means_the_board_never_sleeps() {
        let w = Window::new(0, 15);
        assert_eq!(w.next(0, 0), Next::Advertise);
        assert_eq!(w.next(15_000, 0), Next::Advertise);
        assert_eq!(w.next(u64::MAX, 0), Next::Advertise);
    }

    /// The wake advertises for the configured window and then sleeps for
    /// the configured interval.
    #[test]
    fn a_spent_window_ends_in_deep_sleep() {
        let w = Window::new(1_000, 15);
        assert_eq!(w.ends_ms(), 16_000);
        assert_eq!(w.next(1_000, 60), Next::Advertise);
        assert_eq!(w.remaining_ms(1_000), 15_000);
        assert_eq!(w.next(15_999, 60), Next::Advertise);
        assert_eq!(w.remaining_ms(15_999), 1);
        assert_eq!(w.next(16_000, 60), Next::Sleep { interval_s: 60 });
        assert_eq!(w.remaining_ms(16_000), 0);
        // A clock past the deadline is still a spent budget, not a wrap.
        assert_eq!(w.remaining_ms(u64::MAX), 0);
        assert_eq!(w.next(u64::MAX, 60), Next::Sleep { interval_s: 60 });
    }

    /// The regression the deadline exists for: a central that keeps failing
    /// to connect must not be able to hold the board awake. Every retry
    /// asks again, and the answer is still measured from the wake.
    #[test]
    fn retries_cannot_extend_the_window() {
        let w = Window::new(0, 15);
        let mut now = 0;
        // A connect attempt that fails every 200 ms, as the firmware's
        // retry pause does.
        while let Next::Advertise = w.next(now, 30) {
            now += 200;
            assert!(now <= 15_200, "the window never ended");
        }
        assert_eq!(w.next(now, 30), Next::Sleep { interval_s: 30 });
        assert_eq!(w.ends_ms(), 15_000, "the deadline moved");
    }

    /// A disconnect buys a short linger, so the phone can come straight
    /// back instead of waiting out a whole sleep interval.
    #[test]
    fn a_disconnect_lingers_then_sleeps() {
        let mut w = Window::new(0, 15);
        // A session that outlasted the original window.
        let disconnected_at = 400_000;
        assert_eq!(
            w.next(disconnected_at, 60),
            Next::Sleep { interval_s: 60 },
            "the budget was already spent while connected"
        );
        w.linger(disconnected_at);
        assert_eq!(w.next(disconnected_at, 60), Next::Advertise);
        assert_eq!(w.remaining_ms(disconnected_at), LINGER_S * 1000);
        assert_eq!(
            w.next(disconnected_at + LINGER_S * 1000, 60),
            Next::Sleep { interval_s: 60 }
        );
    }

    /// Sleep switched on mid-window takes effect at the next decision, not
    /// at the next boot - the interval is asked for every time.
    #[test]
    fn enabling_sleep_takes_effect_within_the_window() {
        let w = Window::new(0, 15);
        assert_eq!(w.next(20_000, 0), Next::Advertise);
        assert_eq!(w.next(20_000, 300), Next::Sleep { interval_s: 300 });
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
            let w = Window::new(now, s.adv_window());
            assert_eq!(w.next(now, s.sleep_interval_s), Next::Advertise);
            now += 10_000;
            assert_eq!(
                w.next(now, s.sleep_interval_s),
                Next::Sleep { interval_s: 120 }
            );
            now += 120_000;
        }
    }
}
