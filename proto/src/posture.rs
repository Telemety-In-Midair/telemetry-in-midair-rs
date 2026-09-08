//! What the hardware task raises and lowers, and the requests that move it.
//!
//! The hardware task owns the radio, the GPS and the card. Everything else
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
    /// the node from it, write it back to the card and to flash.
    ApplyConfig,
    /// A firmware image landed in the inactive slot. Flush the card and
    /// reboot into it.
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
/// config wants the card the mode may have just mounted, and a park has to
/// come after anything that raises or the board sleeps with it up.
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

/// Where the card is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Card {
    /// Left off the bus: a wake check has not read it and may never.
    Deferred,
    /// Mounted, or trying to mount, and logging.
    Mounted,
    /// Flushed and unmounted for a deep sleep.
    Parked,
}

/// One thing the hardware task has to do for a request.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Effect {
    /// Read the card and adopt the stored config, then reconfigure the
    /// node from it.
    MountCard,
    /// Flush the buffered log lines and unmount the card.
    ParkCard,
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
        // The longest sequence is a park, which is six; the array is sized
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
    pub card: Card,
}

impl Posture {
    /// What a boot into `boot` raises, and the effects that raise it.
    ///
    /// Three flavors. A wake check raises nothing and reads no card: it
    /// exists to ask whether anyone wants the board back, which needs BLE
    /// only. Idle mounts the card so config reads and log pulls work, and
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
                    card: Card::Deferred,
                }
            }
            Mode::Idle => {
                fx.push(Effect::MountCard);
                fx.push(Effect::GpsPark);
                fx.push(Effect::RadioSleep);
                Self {
                    live: boot,
                    radio: Radio::Asleep,
                    gps: Gps::Parked,
                    card: Card::Mounted,
                }
            }
            Mode::Tracking | Mode::Listening => {
                fx.push(Effect::MountCard);
                fx.push(Effect::RadioInit);
                fx.push(Effect::GpsUp);
                Self {
                    live: boot,
                    radio: Radio::Up,
                    gps: Gps::Awake,
                    card: Card::Mounted,
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

    /// Bring up a card a wake check left off the bus.
    fn mount(&mut self, fx: &mut Effects) {
        if self.card == Card::Deferred {
            fx.push(Effect::MountCard);
            self.card = Card::Mounted;
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
                if self.radio == Radio::Asleep {
                    fx.push(Effect::RadioInit);
                }
                fx.push(Effect::RadioStandby);
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
        if self.card == Card::Parked && !matches!(r, Request::PrepareSleep | Request::Reboot) {
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
                // The card first: a config that has not been read yet is
                // the one the radio is about to be initialized from.
                self.mount(&mut fx);
                if m.tracks() {
                    self.apply_overrides(stored, &mut fx);
                } else {
                    self.move_gps(Gps::Parked, &mut fx);
                    self.move_radio(Radio::Asleep, &mut fx);
                }
            }
            Request::ApplyConfig => {
                self.mount(&mut fx);
                fx.push(Effect::ApplyConfig);
                // The apply re-initializes the radio, which is the one
                // thing that brings it up. A board that was not using it
                // gets it put straight back: a config push is not a
                // request to start listening.
                match self.radio {
                    Radio::Up => {}
                    Radio::Standby => fx.push(Effect::RadioStandby),
                    Radio::Asleep => fx.push(Effect::RadioSleep),
                }
            }
            Request::PrepareSleep => {
                // Everything a sleeping board cannot use, in the order that
                // loses the least when the sequence does not finish: the
                // three that cost current first, each bounded work, and
                // the card last because it alone can stall.
                if self.gps == Gps::Parked {
                    fx.push(Effect::CheckParkHeld);
                }
                // Re-issued rather than skipped when already parked: a
                // reset leaves the driver believing the module is awake,
                // and the module is not obliged to agree with either.
                fx.push(Effect::GpsPark);
                fx.push(Effect::RadioSleep);
                fx.push(Effect::PanelBlank);
                fx.push(Effect::ParkCard);
                fx.push(Effect::SleepReady);
                self.gps = Gps::Parked;
                self.radio = Radio::Asleep;
                self.card = Card::Parked;
            }
            Request::Reboot => {
                // The same hole as a deep sleep and the same fix: the
                // pending log buffer is RAM, and a reset is a reset. The
                // posture is not moved because the reset follows.
                fx.push(Effect::ParkCard);
                fx.push(Effect::Reboot);
            }
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
    /// its card mounted.
    pub fn consistent(&self, stored: &Stored) -> Result<(), &'static str> {
        let down = self.radio == Radio::Asleep && self.gps == Gps::Parked;
        if self.card == Card::Parked {
            return if down { Ok(()) } else { Err("parked for sleep with something still up") };
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
                } else if self.card != Card::Mounted {
                    Err("idle without the card")
                } else {
                    Ok(())
                }
            }
            Mode::Tracking | Mode::Listening => {
                if self.card != Card::Mounted {
                    return Err("tracking without the card");
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
    fn a_wake_check_raises_nothing_and_reads_no_card() {
        let (p, e) = Posture::at_boot(Mode::Stored, &Stored::new());
        assert_eq!(fx(e), vec![Effect::GpsAssumeParked]);
        assert_eq!(p.card, Card::Deferred);
        assert!(!p.busy());
        assert_eq!(p.consistent(&Stored::new()), Ok(()));
    }

    #[test]
    fn idle_mounts_the_card_and_parks_the_rest() {
        let (p, e) = Posture::at_boot(Mode::Idle, &Stored::new());
        assert_eq!(fx(e), vec![Effect::MountCard, Effect::GpsPark, Effect::RadioSleep]);
        assert_eq!((p.radio, p.gps, p.card), (Radio::Asleep, Gps::Parked, Card::Mounted));
        assert!(!p.busy());
    }

    /// Tracking raises everything, then honors each override flag the way
    /// the boot path always did.
    #[test]
    fn tracking_raises_everything_then_honors_the_overrides() {
        let (p, e) = Posture::at_boot(Mode::Tracking, &Stored::new());
        assert_eq!(fx(e), vec![Effect::MountCard, Effect::RadioInit, Effect::GpsUp]);
        assert!(p.radio_up() && p.gps_awake());

        let s = flags(PFLAG_GPS_SLEEP | PFLAG_RADIO_STANDBY);
        let (p, e) = Posture::at_boot(Mode::Listening, &s);
        assert_eq!(
            fx(e),
            vec![
                Effect::MountCard,
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

    /// A promotion mounts the card the wake check deferred, and only the
    /// card.
    #[test]
    fn a_promotion_mounts_the_deferred_card() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        assert_eq!(fx(p.on(Request::Mode(Mode::Idle), &s)), vec![Effect::MountCard]);
        assert_eq!(p.card, Card::Mounted);
        assert_eq!(p.consistent(&s), Ok(()));
        // And a tracking command straight from a wake check raises the lot.
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        assert_eq!(
            fx(p.on(Request::Mode(Mode::Tracking), &s)),
            vec![Effect::MountCard, Effect::GpsUp, Effect::RadioInit]
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
        // On a wake check the card comes up first, so the file has a card
        // to be written to.
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        assert_eq!(
            fx(p.on(Request::ApplyConfig, &s)),
            vec![Effect::MountCard, Effect::ApplyConfig, Effect::RadioSleep]
        );
    }

    /// A park takes everything down in the order that loses the least, and
    /// ends by saying so.
    #[test]
    fn a_park_lowers_everything_and_signals() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        assert_eq!(
            fx(p.on(Request::PrepareSleep, &s)),
            vec![
                Effect::GpsPark,
                Effect::RadioSleep,
                Effect::PanelBlank,
                Effect::ParkCard,
                Effect::SleepReady
            ]
        );
        assert_eq!(p.card, Card::Parked);
        assert_eq!(p.consistent(&s), Ok(()));
        // A receiver that should already be parked is probed first.
        let (mut p, _) = Posture::at_boot(Mode::Stored, &s);
        let e = p.on(Request::PrepareSleep, &s);
        assert_eq!(e.iter().next(), Some(Effect::CheckParkHeld));
        assert_eq!(e.len(), 6);
    }

    #[test]
    fn a_reboot_flushes_the_card_first() {
        let s = Stored::new();
        let (mut p, _) = Posture::at_boot(Mode::Tracking, &s);
        assert_eq!(fx(p.on(Request::Reboot, &s)), vec![Effect::ParkCard, Effect::Reboot]);
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
        assert_eq!(p.card, Card::Parked);
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
