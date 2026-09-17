//! What the hardware task raises and lowers, and the requests that move it.
//!
//! The hardware task owns the radio, the GPS and the panel. Everything else
//! - the BLE session, the USB console, the serve loop - asks it to change
//! what is up and what is down through a [`Request`], and the task answers
//! on its next pass. What each request does depends on where the board
//! already is: a mode raises or lowers everything, an override parks one
//! subsystem inside a tracking posture, a sleep parks the lot.
//!
//! That decision used to be a `match` inside the task, next to the SPI and
//! UART calls it drove, and there it could only be checked on a bench with
//! a meter. Here it is a value: [`Posture`] is what is up, [`Posture::on`]
//! is what a request changes and the [`Effect`]s the task has to carry out
//! for it, and [`Posture::consistent`] is the rule that every reachable
//! posture must satisfy. The firmware runs the effects; the host runs every
//! sequence of requests there is and checks the rule after each.
//!
//! [`Requests`] is the queue between the two. It holds at most one request
//! of each kind - a newer mode replaces an older one still waiting, and a
//! sleep asked for twice is one sleep - so it can never overflow, and it is
//! drained in a fixed order that puts a mode before the overrides that sit
//! on top of it and a park after everything that raises.

use crate::ble::Mode;
use crate::radiocfg::Role;
use crate::session::Stored;

/// A request from the BLE session or the host tools to the hardware loop.
///
/// Acknowledged where it is made - there is no second chip that can fail
/// to answer - and picked up by the loop on its next pass.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Request {
    /// Park the GPS in backup (`true`) or wake it. An override inside a
    /// tracking posture; nothing elsewhere, where the mode decides.
    GpsSleep(bool),
    /// Put the radio in standby (`true`) or bring it back. An override
    /// inside a tracking posture, like [`Request::GpsSleep`].
    RadioStandby(bool),
    /// Raise or lower everything at once, because a mode is a posture
    /// rather than one subsystem. [`Mode::Stored`] never arrives this way:
    /// storing the board is a deep sleep, which is [`Request::PrepareSleep`]
    /// from the side that owns the sleep.
    Mode(Mode),
    /// A new radio config was pushed and verified. Re-init the radio and
    /// the node from it, write it back to flash.
    ApplyConfig,
    /// A firmware image landed in the inactive slot. Reboot into it.
    Reboot,
    /// The board is about to deep sleep. Park everything a sleeping board
    /// cannot use and say when it is done.
    PrepareSleep,
}

/// The requests waiting for the hardware loop.
///
/// A set rather than a queue, one slot per kind: the latest of a kind
/// wins, and a request repeated before the loop ran is one request. That
/// is what makes it impossible to lose one - the channel this replaced held
/// four and dropped the fifth on the floor, and the fifth could be the park
/// before a deep sleep.
///
/// [`take`](Requests::take) hands them out in a fixed order rather than in
/// arrival order, because the order that is correct does not depend on who
/// asked first: a mode decides the posture the overrides apply inside, a
/// config wants the stored one the mode may have just read, and a park has
/// to come after anything that raises or the board sleeps with it up.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Requests {
    mode: Option<Mode>,
    gps_sleep: Option<bool>,
    radio_standby: Option<bool>,
    apply_config: bool,
    prepare_sleep: bool,
    reboot: bool,
}

impl Requests {
    pub const fn new() -> Self {
        Self {
            mode: None,
            gps_sleep: None,
            radio_standby: None,
            apply_config: false,
            prepare_sleep: false,
            reboot: false,
        }
    }

    /// Queue a request. Never fails; a request of the same kind already
    /// waiting is replaced.
    pub fn push(&mut self, r: Request) {
        match r {
            Request::Mode(m) => self.mode = Some(m),
            Request::GpsSleep(on) => self.gps_sleep = Some(on),
            Request::RadioStandby(on) => self.radio_standby = Some(on),
            Request::ApplyConfig => self.apply_config = true,
            Request::PrepareSleep => self.prepare_sleep = true,
            Request::Reboot => self.reboot = true,
        }
    }

    /// The next request to carry out, in the order described above.
    pub fn take(&mut self) -> Option<Request> {
        if let Some(m) = self.mode.take() {
            return Some(Request::Mode(m));
        }
        if let Some(on) = self.gps_sleep.take() {
            return Some(Request::GpsSleep(on));
        }
        if let Some(on) = self.radio_standby.take() {
            return Some(Request::RadioStandby(on));
        }
        if core::mem::take(&mut self.apply_config) {
            return Some(Request::ApplyConfig);
        }
        if core::mem::take(&mut self.prepare_sleep) {
            return Some(Request::PrepareSleep);
        }
        if core::mem::take(&mut self.reboot) {
            return Some(Request::Reboot);
        }
        None
    }

    pub fn is_empty(&self) -> bool {
        self.mode.is_none()
            && self.gps_sleep.is_none()
            && self.radio_standby.is_none()
            && !self.apply_config
            && !self.prepare_sleep
            && !self.reboot
    }

    /// Whether a park is waiting, which is what keeps the loop from
    /// starting a transmit the sleep would then have to wait out.
    pub fn sleep_pending(&self) -> bool {
        self.prepare_sleep
    }
}

/// Where the radio is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Radio {
    /// Cold sleep, configuration lost; an init brings it back.
    Asleep,
    /// Configured and idle in standby: the [`Request::RadioStandby`]
    /// override.
    Standby,
    /// Initialized from the running config and, on a listening role, in
    /// continuous receive.
    Up,
    /// Duty-cycling on its own timers with the wake sync word, waiting
    /// for a wake frame while the chip sleeps. Only ever entered by a park,
    /// and from then on nothing may touch the radio over SPI: a transaction
    /// during its sleep phase ends the cycle.
    Sentry,
}

/// Where the GPS receiver is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Gps {
    /// Backup mode, or believed to be: the parked state a wake check
    /// inherits from the park before it.
    Parked,
    /// Acquiring or tracking, and being polled.
    Awake,
}

/// Whether the stored radio config has been read and adopted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Config {
    /// Not yet: a wake check does not read it and may never need to.
    Unread,
    /// Read from the board's flash and adopted, or found absent and the
    /// defaults adopted in its place.
    Read,
}

/// One thing the hardware task has to do for a request.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Effect {
    /// Read the stored config from flash and adopt it, then reconfigure
    /// the node from it.
    LoadConfig,
    /// Bring the receiver up: wake it from backup, or configure a receiver
    /// that is already running, and re-arm the settings retry.
    GpsUp,
    /// Put the receiver into backup.
    GpsPark,
    /// Tell the driver the receiver is in backup without touching it, for
    /// a wake check that inherits the park before it.
    GpsAssumeParked,
    /// Initialize the radio from the running config.
    RadioInit,
    /// Park the radio in standby.
    RadioStandby,
    /// Put the radio into cold sleep.
    RadioSleep,
    /// Arm the radio's sniff loop for a wake frame and leave it running.
    /// The task falls back to [`Effect::RadioSleep`] if the config's
    /// sentry cannot be armed.
    RadioSentry,
    /// Blank the status panel.
    PanelBlank,
    /// Adopt the pushed config: re-init the radio and the node from it and
    /// write it back. Followed by the radio effect that puts the radio back
    /// where the posture wants it, since the apply brings it up.
    ApplyConfig,
    /// Ask whether the receiver is talking when it should be parked, and
    /// say so - the one place a park that did not hold can be caught.
    CheckParkHeld,
    /// Everything is parked; the sleep may proceed.
    SleepReady,
    /// Reset into the new image.
    Reboot,
}

/// Most effects one request can produce.
const EFFECTS_MAX: usize = 8;

/// The effects of one request, in the order to carry them out.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Effects {
    items: [Option<Effect>; EFFECTS_MAX],
    len: usize,
}

impl Effects {
    fn push(&mut self, e: Effect) {
        // The longest sequence is a park, which is five; the array is sized
        // with room, so this cannot overflow short of a new effect being
        // added to the longest arm without the constant following.
        if self.len < EFFECTS_MAX {
            self.items[self.len] = Some(e);
            self.len += 1;
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn contains(&self, e: Effect) -> bool {
        self.iter().any(|x| x == e)
    }

    pub fn iter(&self) -> impl Iterator<Item = Effect> + '_ {
        self.items[..self.len].iter().flatten().copied()
    }
}

impl IntoIterator for Effects {
    type Item = Effect;
    type IntoIter = core::iter::Flatten<core::array::IntoIter<Option<Effect>, EFFECTS_MAX>>;

    fn into_iter(self) -> Self::IntoIter {
        self.items.into_iter().flatten()
    }
}

/// What is up on the board, and what mode it is up for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Posture {
    /// The mode the hardware is in. Moved by [`Request::Mode`], never by
    /// the task on its own.
    pub live: Mode,
    pub radio: Radio,
    pub gps: Gps,
    pub config: Config,
    /// Parked for a deep sleep: everything down, and staying down
    /// whatever arrives until the chip goes.
    pub parked: bool,
    /// Whether a park arms the radio as a sentry rather than sleeping it.
    /// The config's `wake_enabled`, once the config has been read; the
    /// default until then, so a wake check that has never read its config
    /// arms one and lets the task's own check of the config decide.
    pub sentry: bool,
}

impl Posture {
    /// What a boot into `boot` raises, and the effects that raise it.
    ///
    /// Three flavors. A wake check raises nothing and reads no config: it
    /// exists to ask whether anyone wants the board back, which needs BLE
    /// only. Idle reads the config so a config read-back answers, and
    /// parks the receiver a cold boot left acquiring. Tracking and
    /// listening raise everything, then honor the two overrides the
    /// settings carry.
    pub fn at_boot(boot: Mode, stored: &Stored) -> (Self, Effects) {
        let mut fx = Effects::default();
        let mut p = match boot {
            Mode::Stored => {
                fx.push(Effect::GpsAssumeParked);
                Self {
                    live: boot,
                    radio: Radio::Asleep,
                    gps: Gps::Parked,
                    config: Config::Unread,
                    parked: false,
                    sentry: true,
                }
            }
            Mode::Idle => {
                fx.push(Effect::LoadConfig);
                fx.push(Effect::GpsPark);
                fx.push(Effect::RadioSleep);
                Self {
                    live: boot,
                    radio: Radio::Asleep,
                    gps: Gps::Parked,
                    config: Config::Read,
                    parked: false,
                    sentry: true,
                }
            }
            Mode::Tracking | Mode::Listening => {
                fx.push(Effect::LoadConfig);
                fx.push(Effect::RadioInit);
                fx.push(Effect::GpsUp);
                Self {
                    live: boot,
                    radio: Radio::Up,
                    gps: Gps::Awake,
                    config: Config::Read,
                    parked: false,
                    sentry: true,
                }
            }
        };
        if boot.tracks() {
            p.apply_overrides(stored, &mut fx);
        }
        (p, fx)
    }

    /// Move the receiver and the radio to where the two override flags say
    /// a tracking posture keeps them.
    fn apply_overrides(&mut self, stored: &Stored, fx: &mut Effects) {
        let want_gps = if stored.gps_sleep() { Gps::Parked } else { Gps::Awake };
        self.move_gps(want_gps, fx);
        let want_radio = if stored.radio_standby() { Radio::Standby } else { Radio::Up };
        self.move_radio(want_radio, fx);
    }

    /// What the config, once read, says a park does with the radio.
    pub fn set_sentry(&mut self, on: bool) {
        self.sentry = on;
    }

    /// Read the stored config a wake check left unread.
    fn load(&mut self, fx: &mut Effects) {
        if self.config == Config::Unread {
            fx.push(Effect::LoadConfig);
            self.config = Config::Read;
        }
    }

    fn move_gps(&mut self, want: Gps, fx: &mut Effects) {
        if self.gps == want {
            return;
        }
        fx.push(match want {
            Gps::Awake => Effect::GpsUp,
            Gps::Parked => Effect::GpsPark,
        });
        self.gps = want;
    }

    fn move_radio(&mut self, want: Radio, fx: &mut Effects) {
        if self.radio == want {
            return;
        }
        match want {
            Radio::Up => fx.push(Effect::RadioInit),
            Radio::Asleep => fx.push(Effect::RadioSleep),
            // Standby is a configured radio parked between modes, so a
            // radio that lost its configuration in cold sleep is brought
            // up first, exactly as the boot path does.
            Radio::Standby => {
                if self.radio != Radio::Up {
                    fx.push(Effect::RadioInit);
                }
                fx.push(Effect::RadioStandby);
            }
            // Arming needs a configured radio, and a config to configure
            // it from: a wake check has read neither.
            Radio::Sentry => {
                if self.radio != Radio::Up && self.radio != Radio::Standby {
                    fx.push(Effect::RadioInit);
                }
                fx.push(Effect::RadioSentry);
            }
        }
        self.radio = want;
    }

    /// Apply one request and say what the task has to do for it.
    ///
    /// `stored` carries the two override flags, which decide where a mode
    /// that tracks leaves the receiver and the radio - so a board asked
    /// into tracking with the GPS parked comes up with it parked, and what
    /// the settings characteristic reports is what the hardware is doing.
    pub fn on(&mut self, r: Request, stored: &Stored) -> Effects {
        let mut fx = Effects::default();
        // A board parked for a deep sleep stays parked. The sleep follows
        // within a bounded wait and resets everything, so a mode or an
        // override that arrives in that window - from the console, since
        // the session that could have sent one is gone - would raise the
        // receiver or the radio for the sleep to happen over.
        if self.parked && !matches!(r, Request::PrepareSleep | Request::Reboot) {
            return fx;
        }
        match r {
            // The overrides only mean anything inside a tracking posture.
            // Elsewhere the mode has already parked both, and waking one
            // would leave a board that says it is idle drawing an
            // acquisition; the flag is still stored, and honored when
            // tracking is next commanded.
            Request::GpsSleep(park) => {
                if self.live.tracks() {
                    let want = if park { Gps::Parked } else { Gps::Awake };
                    self.move_gps(want, &mut fx);
                }
            }
            Request::RadioStandby(park) => {
                if self.live.tracks() {
                    let want = if park { Radio::Standby } else { Radio::Up };
                    self.move_radio(want, &mut fx);
                }
            }
            Request::Mode(m) => {
                self.live = m;
                // The config first: one that has not been read yet is the
                // one the radio is about to be initialized from.
                self.load(&mut fx);
                if m.tracks() {
                    self.apply_overrides(stored, &mut fx);
                } else {
                    self.move_gps(Gps::Parked, &mut fx);
                    self.move_radio(Radio::Asleep, &mut fx);
                }
            }
            Request::ApplyConfig => {
                self.load(&mut fx);
                fx.push(Effect::ApplyConfig);
                // The apply re-initializes the radio, which is the one
                // thing that brings it up. A board that was not using it
                // gets it put straight back: a config push is not a
                // request to start listening.
                match self.radio {
                    Radio::Up => {}
                    Radio::Standby => fx.push(Effect::RadioStandby),
                    Radio::Asleep => fx.push(Effect::RadioSleep),
                    // Unreachable in practice - a sentry only exists on a
                    // parked board, and a parked board takes no config -
                    // but if it were, the new config is what to listen on.
                    Radio::Sentry => fx.push(Effect::RadioSentry),
                }
            }
            Request::PrepareSleep => {
                // Everything a sleeping board cannot use, in the order that
                // loses the least when the sequence does not finish: the
                // two that cost current first, each bounded work, then the
                // panel.
                if self.gps == Gps::Parked {
                    fx.push(Effect::CheckParkHeld);
                }
                // Re-issued rather than skipped when already parked: a
                // reset leaves the driver believing the module is awake,
                // and the module is not obliged to agree with either.
                fx.push(Effect::GpsPark);
                self.gps = Gps::Parked;
                if self.sentry {
                    // The sentry listens on the config's carrier with the
                    // config's modulation, so the config has to be read
                    // and the radio initialized from it before the arm -
                    // which for a wake check is the first time either
                    // happens.
                    self.load(&mut fx);
                    self.move_radio(Radio::Sentry, &mut fx);
                } else {
                    fx.push(Effect::RadioSleep);
                    self.radio = Radio::Asleep;
                }
                fx.push(Effect::PanelBlank);
                fx.push(Effect::SleepReady);
                self.parked = true;
            }
            // The posture is not moved because the reset follows.
            Request::Reboot => fx.push(Effect::Reboot),
        }
        fx
    }

    /// Whether the radio is initialized: what makes the loop poll the
    /// receiver and plan a beacon.
    pub fn radio_up(&self) -> bool {
        self.radio == Radio::Up
    }

    /// Whether the receiver is awake: what makes the loop drain its
    /// sentences and push its settings.
    pub fn gps_awake(&self) -> bool {
        self.gps == Gps::Awake
    }

    /// Whether anything is up that the loop has to service at its full
    /// rate. A board with both down has nothing that moves faster than
    /// the panel.
    pub fn busy(&self) -> bool {
        self.radio_up() || self.gps_awake()
    }

    /// Whether the node may put a frame on the air right now: the mode and
    /// the role both transmit ([`on_air`]), the radio is up, no transfer
    /// owns the board and no sleep is waiting to park it.
    pub fn may_transmit(&self, role: Role, transfer_active: bool, sleep_pending: bool) -> bool {
        on_air(self.live, role).transmits && self.radio_up() && !transfer_active && !sleep_pending
    }
}

/// What a node does on the air, given both the words that describe it.
///
/// The role is the network's word and travels with the fleet in the radio
/// config: a leaf, a repeater, a node that only sends or only listens. The
/// mode is the device's word and lives in its settings: tracking or
/// listening beside a phone. The two overlap - a listening node and an
/// `rx_only` node both never transmit - and every combination of the four
/// modes and four roles is answered here, once, so the beacon gate, the
/// repeat gate and the receiver all read the same table.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct OnAir {
    /// Beacons and pings go out.
    pub transmits: bool,
    /// The receiver is armed.
    pub receives: bool,
    /// Frames still carrying hops are forwarded.
    pub repeats: bool,
}

/// The mode and the role, combined: each half of the air interface is used
/// only when both words allow it. A listening node is an `rx_only` node
/// for as long as it listens, whatever its role says; a stored or idle
/// node does nothing on the air at all.
pub fn on_air(mode: Mode, role: Role) -> OnAir {
    let raised = mode.tracks();
    let transmits = raised && mode.transmits() && role.transmits();
    OnAir {
        transmits,
        receives: raised && role.receives(),
        // A repeat is a transmission, so a node that may not transmit may
        // not repeat either, whatever its role.
        repeats: transmits && role.repeats(),
    }
}

impl Posture {
    /// The rule every reachable posture satisfies, against the override
    /// flags in `stored`. `Err` names the way it does not.
    ///
    /// A parked board has everything down whatever its mode; a wake check
    /// and an idle board have the receiver and the radio down; a tracking
    /// board has each of them exactly where its override flag says, and
    /// its config read.
    pub fn consistent(&self, stored: &Stored) -> Result<(), &'static str> {
        let down = self.radio == Radio::Asleep && self.gps == Gps::Parked;
        if self.parked {
            // A sentry is the one thing a parked board leaves running. It
            // is armed on the strength of a config the park may have had
            // to read in the same breath, so whether it was asked for is
            // the arm's own check, not this rule's: the task sleeps the
            // radio cold when the config it just read says no.
            let radio_down = match self.radio {
                Radio::Asleep => true,
                Radio::Sentry => self.config == Config::Read,
                Radio::Standby | Radio::Up => false,
            };
            return if radio_down && self.gps == Gps::Parked {
                Ok(())
            } else {
                Err("parked for sleep with something still up")
            };
        }
        if self.radio == Radio::Sentry {
            return Err("a sentry outside a park");
        }
        match self.live {
            Mode::Stored => {
                if down {
                    Ok(())
                } else {
                    Err("a wake check raised the receiver or the radio")
                }
            }
            Mode::Idle => {
                if !down {
                    Err("idle with the receiver or the radio up")
                } else if self.config != Config::Read {
                    Err("idle without the config read")
                } else {
                    Ok(())
                }
            }
            Mode::Tracking | Mode::Listening => {
                if self.config != Config::Read {
                    return Err("tracking without the config read");
                }
                let want_gps = if stored.gps_sleep() { Gps::Parked } else { Gps::Awake };
                if self.gps != want_gps {
                    return Err("the receiver is not where the gps_sleep flag says");
                }
                let want_radio = if stored.radio_standby() { Radio::Standby } else { Radio::Up };
                if self.radio != want_radio {
                    return Err("the radio is not where the wio_sleep flag says");
                }
                Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::{PFLAG_GPS_SLEEP, PFLAG_RADIO_STANDBY};

    fn flags(f: u32) -> Stored {
        Stored {
            flags: f,
            ..Stored::new()
        }
    }

    fn fx(e: Effects) -> Vec<Effect> {
        e.into_iter().collect()
    }

    // -- the request set ---------------------------------------------------

    /// One slot per kind: a newer mode replaces the older, a repeated park
    /// is one park, and nothing is ever refused.
    #[test]
    fn requests_coalesce_by_kind_and_never_overflow() {
        let mut q = Requests::new();
        assert!(q.is_empty());
        for _ in 0..100 {
            q.push(Request::Mode(Mode::Tracking));
            q.push(Request::Mode(Mode::Idle));
            q.push(Request::PrepareSleep);
            q.push(Request::GpsSleep(true));
            q.push(Request::GpsSleep(false));
        }
        assert!(q.sleep_pending());
        assert_eq!(q.take(), Some(Request::Mode(Mode::Idle)));
        assert_eq!(q.take(), Some(Request::GpsSleep(false)));
        assert_eq!(q.take(), Some(Request::PrepareSleep));
        assert_eq!(q.take(), None);
        assert!(q.is_empty());
        assert!(!q.sleep_pending());
    }

    /// The drain order is fixed, whatever the arrival order: mode, the
    /// two overrides, config, park, reboot.
    #[test]
    fn requests_drain_in_a_fixed_order() {
        let all = [
            Request::Reboot,
            Request::PrepareSleep,
            Request::ApplyConfig,
            Request::RadioStandby(true),
            Request::GpsSleep(true),
            Request::Mode(Mode::Tracking),
        ];
        let mut q = Requests::new();
        for r in all {
            q.push(r);
        }
        let mut out = Vec::new();
        while let Some(r) = q.take() {
            out.push(r);
        }
        assert_eq!(
            out,
            vec![
                Request::Mode(Mode::Tracking),
                Request::GpsSleep(true),
                Request::RadioStandby(true),
                Request::ApplyConfig,
                Request::PrepareSleep,
                Request::Reboot,
            ]
        );
    }

    // -- boot ---------------------------------------------------------------

    #[test]
    fn a_wake_check_raises_nothing_and_reads_no_config() {
        let (p, e) = Posture::at_boot(Mode::Stored, &Stored::new());
        assert_eq!(fx(e), vec![Effect::GpsAssumeParked]);
        assert_eq!(p.config, Config::Unread);
        assert!(!p.busy());
        assert_eq!(p.consistent(&Stored::new()), Ok(()));
    }

    #[test]
    fn idle_reads_the_config_and_parks_the_rest() {
        let (p, e) = Posture::at_boot(Mode::Idle, &Stored::new());
        assert_eq!(fx(e), vec![Effect::LoadConfig, Effect::GpsPark, Effect::RadioSleep]);
        assert_eq!((p.radio, p.gps, p.config), (Radio::Asleep, Gps::Parked, Config::Read));
        assert!(!p.busy());
    }

    /// Tracking raises everything, then honors each override flag the way
    /// the boot path always did.
    #[test]
    fn tracking_raises_everything_then_honors_the_overrides() {
        let (p, e) = Posture::at_boot(Mode::Tracking, &Stored::new());
        assert_eq!(fx(e), vec![Effect::LoadConfig, Effect::RadioInit, Effect::GpsUp]);
        assert!(p.radio_up() && p.gps_awake());

        let s = flags(PFLAG_GPS_SLEEP | PFLAG_RADIO_STANDBY);
        let (p, e) = Posture::at_boot(Mode::Listening, &s);
        assert_eq!(
            fx(e),
            vec![
                Effect::LoadConfig,
                Effect::RadioInit,
                Effect::GpsUp,
                Effect::GpsPark,
                Effect::RadioStandby
            ]
        );
        assert_eq!((p.radio, p.gps), (Radio::Standby, Gps::Parked));
        assert_eq!(p.consistent(&s), Ok(()));
        assert!(!p.busy());
    }

    // -- requests ----------------------------------------------------------

    /// The regression the flags exposed: a mode commanded on top of an
    /// override must leave the hardware where the settings say it is, or
    /// the app shows "GPS in backup" beside a receiver that is acquiring.
    #[test]
    fn a_mode_change_lands_on_the_flags() {
        let s = flags(PFLAG_GPS_SLEEP);
        let (mut p, _) = Posture::at_boot(Mode::Idle, &s);
        let e = p.on(Request::Mode(Mode::Tracking), &s);
        assert_eq!(fx(e), vec![Effect::RadioInit]);
        assert_eq!(p.gps, Gps::Parked, "the flag keeps it parked");
        assert_eq!(p.consistent(&s), Ok(()));

        let e = p.on(Request::Mode(Mode::Idle), &s);
        assert_eq!(fx(e), vec![Effect::RadioSleep]);
        assert_eq!(p.consistent(&s), Ok(()));
    }

    /// The other regression: waking the GPS while the radio is in standby
    /// used to leave a receiver acquiring that nothing polled.
    #[test]
    fn the_overrides_are_independent_inside_tracking() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        assert_eq!(fx(p.on(Request::RadioStandby(true), &s)), vec![Effect::RadioStandby]);
        assert!(!p.radio_up() && p.gps_awake() && p.busy());
        assert_eq!(fx(p.on(Request::GpsSleep(true), &s)), vec![Effect::GpsPark]);
        assert!(!p.busy());
        assert_eq!(fx(p.on(Request::GpsSleep(false), &s)), vec![Effect::GpsUp]);
        assert!(p.gps_awake() && !p.radio_up());
        assert_eq!(fx(p.on(Request::RadioStandby(false), &s)), vec![Effect::RadioInit]);
        assert!(p.radio_up());
        // Asked for what it already has: nothing to do.
        assert!(p.on(Request::RadioStandby(false), &s).is_empty());
        assert!(p.on(Request::GpsSleep(false), &s).is_empty());
    }

    /// Outside a tracking posture the overrides touch nothing: idle is
    /// idle, and a wake check stays dark.
    #[test]
    fn the_overrides_do_nothing_outside_tracking() {
        let s = Stored::new();
        for boot in [Mode::Stored, Mode::Idle] {
            let (mut p, _) = Posture::at_boot(boot, &s);
            let before = p;
            for r in [
                Request::GpsSleep(false),
                Request::GpsSleep(true),
                Request::RadioStandby(false),
                Request::RadioStandby(true),
            ] {
                assert!(p.on(r, &s).is_empty(), "{boot:?} {r:?}");
                assert_eq!(p, before);
            }
        }
    }

    /// A promotion reads the config the wake check left unread, and only
    /// that.
    #[test]
    fn a_promotion_reads_the_unread_config() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        assert_eq!(fx(p.on(Request::Mode(Mode::Idle), &s)), vec![Effect::LoadConfig]);
        assert_eq!(p.config, Config::Read);
        assert_eq!(p.consistent(&s), Ok(()));
        // And a tracking command straight from a wake check raises the lot.
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        assert_eq!(
            fx(p.on(Request::Mode(Mode::Tracking), &s)),
            vec![Effect::LoadConfig, Effect::GpsUp, Effect::RadioInit]
        );
        assert_eq!(p.consistent(&s), Ok(()));
    }

    /// A config apply brings the radio up as a side effect, so the posture
    /// puts it back where it was.
    #[test]
    fn a_config_apply_puts_the_radio_back() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Idle, &s);
        assert_eq!(fx(p.on(Request::ApplyConfig, &s)), vec![Effect::ApplyConfig, Effect::RadioSleep]);
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        assert_eq!(fx(p.on(Request::ApplyConfig, &s)), vec![Effect::ApplyConfig]);
        let st = flags(PFLAG_RADIO_STANDBY);
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &st);
        assert_eq!(fx(p.on(Request::ApplyConfig, &st)), vec![Effect::ApplyConfig, Effect::RadioStandby]);
        // On a wake check the stored config is read first, so the apply
        // lands on a node that knows what it had.
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        assert_eq!(
            fx(p.on(Request::ApplyConfig, &s)),
            vec![Effect::LoadConfig, Effect::ApplyConfig, Effect::RadioSleep]
        );
    }

    /// A park takes everything down in the order that loses the least, and
    /// ends by saying so.
    #[test]
    fn a_park_lowers_everything_and_signals() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        p.set_sentry(false);
        assert_eq!(
            fx(p.on(Request::PrepareSleep, &s)),
            vec![
                Effect::GpsPark,
                Effect::RadioSleep,
                Effect::PanelBlank,
                Effect::SleepReady
            ]
        );
        assert!(p.parked);
        assert_eq!(p.consistent(&s), Ok(()));
        // A receiver that should already be parked is probed first.
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        p.set_sentry(false);
        let e = p.on(Request::PrepareSleep, &s);
        assert_eq!(e.iter().next(), Some(Effect::CheckParkHeld));
        assert_eq!(e.len(), 5);
    }

    /// With the config asking for one, a park arms the radio as a sentry
    /// instead of sleeping it - from a radio that is up, directly; from a
    /// wake check that has read nothing, after reading the config and
    /// initializing the radio from it.
    #[test]
    fn a_park_arms_a_sentry_when_the_config_asks() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        assert_eq!(
            fx(p.on(Request::PrepareSleep, &s)),
            vec![
                Effect::GpsPark,
                Effect::RadioSentry,
                Effect::PanelBlank,
                Effect::SleepReady
            ]
        );
        assert!(p.parked);
        assert_eq!(p.radio, Radio::Sentry);
        assert_eq!(p.consistent(&s), Ok(()));

        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        assert_eq!(
            fx(p.on(Request::PrepareSleep, &s)),
            vec![
                Effect::CheckParkHeld,
                Effect::GpsPark,
                Effect::LoadConfig,
                Effect::RadioInit,
                Effect::RadioSentry,
                Effect::PanelBlank,
                Effect::SleepReady
            ]
        );
        assert_eq!(p.consistent(&s), Ok(()));

        // A radio parked in standby is configured already.
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        p.on(Request::RadioStandby(true), &s);
        let e = fx(p.on(Request::PrepareSleep, &s));
        assert!(!e.contains(&Effect::RadioInit), "{e:?}");
        assert!(e.contains(&Effect::RadioSentry));
    }

    /// A sentry belongs to a park and nowhere else.
    #[test]
    fn a_sentry_outside_a_park_is_inconsistent() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Idle, &s);
        p.radio = Radio::Sentry;
        assert!(p.consistent(&s).is_err());
        p.parked = true;
        assert_eq!(p.consistent(&s), Ok(()));
        // Never from a config that was not read: the arm needs one.
        p.config = Config::Unread;
        assert!(p.consistent(&s).is_err());
    }

    #[test]
    fn a_reboot_is_only_a_reboot() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        assert_eq!(fx(p.on(Request::Reboot, &s)), vec![Effect::Reboot]);
        assert!(!p.parked);
    }

    #[test]
    fn transmit_needs_a_transmitting_mode_and_a_free_board() {
        let s = Stored::new();
        let (p, _) = Posture::at_boot(Mode::Tracking, &s);
        assert!(p.may_transmit(Role::Leaf, false, false));
        assert!(!p.may_transmit(Role::RxOnly, false, false));
        assert!(!p.may_transmit(Role::Leaf, true, false));
        assert!(!p.may_transmit(Role::Leaf, false, true));
        let (p, _) = Posture::at_boot(Mode::Listening, &s);
        assert!(!p.may_transmit(Role::Leaf, false, false));
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        p.on(Request::RadioStandby(true), &s);
        assert!(!p.may_transmit(Role::Leaf, false, false));
    }

    /// Once parked for a sleep, a board stays parked: a mode that arrives
    /// in the window before the chip goes down must not raise anything
    /// for the sleep to happen over.
    #[test]
    fn a_parked_board_ignores_what_would_raise_it() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        p.on(Request::PrepareSleep, &s);
        for r in [
            Request::Mode(Mode::Tracking),
            Request::Mode(Mode::Idle),
            Request::GpsSleep(false),
            Request::RadioStandby(false),
            Request::ApplyConfig,
        ] {
            assert!(p.on(r, &s).is_empty(), "{r:?}");
            assert_eq!(p.consistent(&s), Ok(()), "{r:?}");
        }
        assert!(p.parked);
    }

    /// The whole mode x role matrix, written down: each half of the air is
    /// used only when both words allow it, and a repeat needs the right to
    /// transmit.
    #[test]
    fn the_mode_and_the_role_are_anded() {
        let roles = [Role::Leaf, Role::Repeater, Role::TxOnly, Role::RxOnly];
        for role in roles {
            for mode in [Mode::Stored, Mode::Idle] {
                assert_eq!(
                    on_air(mode, role),
                    OnAir { transmits: false, receives: false, repeats: false },
                    "{mode:?} {role:?}"
                );
            }
            let t = on_air(Mode::Tracking, role);
            assert_eq!(t.transmits, role.transmits(), "{role:?}");
            assert_eq!(t.receives, role.receives(), "{role:?}");
            assert_eq!(t.repeats, role.repeats(), "{role:?}");
            let l = on_air(Mode::Listening, role);
            assert!(!l.transmits && !l.repeats, "{role:?} listening");
            assert_eq!(l.receives, role.receives(), "{role:?} listening");
        }
        // The two ways to say "do not transmit" agree.
        assert_eq!(on_air(Mode::Listening, Role::Leaf), on_air(Mode::Tracking, Role::RxOnly));
    }
}
