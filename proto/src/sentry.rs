//! The policy for a radio that listens while the chip that owns it sleeps.
//!
//! The SX1262's `SetRxDutyCycle` is the only thing on the part that runs
//! without a host: the chip listens for `rxPeriod`, sleeps with its context
//! retained for `sleepPeriod`, and repeats until a packet completes, at
//! which point it raises `RxDone` on DIO1 and stops. The S3 registers that
//! pin as its wake source and goes to deep sleep. A *sentry* is that
//! arrangement, and this module is everything about it that can be decided
//! on a host: what the two periods have to be, how long a wake frame's
//! preamble must run to be caught by them, what a waker's burst looks like,
//! and when a configuration cannot be armed at all.
//!
//! # What the chip does, as measured
//!
//! Two of the receiver's settings decide whether a window opened at an
//! arbitrary point in a long preamble goes on to complete the reception,
//! and both persist in the chip across every mode change:
//!
//! - **`SetLoRaSymbNumTimeout` must be zero.** Set, it makes the modem
//!   demand the *end* of the preamble within that many symbols of the first
//!   chirp it sees, which a window opened mid-preamble can never satisfy.
//!   At eight symbols the measured wake rate was two percent, independent
//!   of preamble length; at zero it was every frame.
//! - **The window is `rxPeriod`.** With the symbol timeout at zero an empty
//!   window closes when `rxPeriod` expires; on preamble detection the
//!   chip restarts that timer at `2 * rxPeriod + sleepPeriod` and holds the
//!   receiver open for the header and the packet behind it.
//!
//! # The two bounds
//!
//! From those, the preamble a wake frame is sent with is bounded on both
//! sides:
//!
//! ```text
//! sleep + tcxo + 2 * detect  <=  Tpreamble  <=  2 * rx + sleep - Theader
//! ```
//!
//! Below the lower bound a preamble can fall entirely inside the gap
//! between two windows, or overlap one by less than the detector needs.
//! Above the upper bound the restarted timer expires before the header
//! arrives, and the frame is abandoned with the receiver awake - measured
//! at 62% with the bound exceeded by 120 ms, against 100% inside it.
//! Subtracting them, the sleep cancels:
//!
//! ```text
//! margin = 2 * rx - tcxo - Theader - 2 * detect
//! ```
//!
//! **The receive window buys margin; the sleep period buys current.** They
//! are independent, which is what makes the two config keys two keys. The
//! margin is what absorbs the drift of the chip's RC64k over a sleep: the
//! preamble is sent for the middle of the window, so half the margin is
//! the drift tolerated in either direction.
//!
//! Every number here is host-tested. If the model disagrees with hardware,
//! the model is wrong; it has been four times, and the fourth was the
//! symbol timeout.

use crate::radiocfg::RadioConfig;

/// One step of the chip's duty-cycle timers, nanoseconds: 1/64 kHz.
pub const DUTY_STEP_NS: u32 = 15_625;

/// Largest value either duty-cycle period field holds.
pub const DUTY_STEPS_MAX: u32 = 0x00FF_FFFF;

/// Longest preamble the packet parameters can name, in symbols.
pub const PREAMBLE_SYMBOLS_MAX: u32 = u16::MAX as u32;

/// Symbols the modem needs inside a window to detect a preamble.
///
/// Four is the figure the SX126x application notes use and the one the
/// bench arithmetic was checked against. The bounds below spend it twice
/// on the lower side - once for the detector and once because the
/// worst-case preamble starts just too late for the previous window - and
/// not at all on the upper, where it would only loosen a bound that is
/// measured to bite.
pub const DETECT_SYMBOLS: u32 = 4;

/// Symbols the LoRa explicit header occupies, which the chip must also
/// receive inside its restarted timer once it has heard a preamble.
pub const HEADER_SYMBOLS: u32 = 8;

/// Longest a frequency hopping system may hold one channel, microseconds.
/// A wake transmission on a hopping plan has to fit this or is refused.
pub const FHSS_DWELL_MAX_US: u32 = 400_000;

/// Frames a waker sends before giving up.
pub const WAKE_TRIES: u8 = 3;

/// How long a waker listens for an answer after each frame, ms.
///
/// The target's boot: measured at about 640 ms to a radio that can hear,
/// plus the wait for its turn in a slot, plus the answer's own air time.
/// Four seconds covers all three at the default modulation with room.
pub const WAKE_GAP_MS: u32 = 4_000;

/// Microseconds to duty-cycle steps, rounded to nearest, saturating.
pub fn duty_steps_from_us(us: u32) -> u32 {
    let ns = u64::from(us) * 1_000;
    let steps = (ns + u64::from(DUTY_STEP_NS) / 2) / u64::from(DUTY_STEP_NS);
    steps.min(u64::from(DUTY_STEPS_MAX)) as u32
}

/// Duty-cycle steps to microseconds.
pub fn duty_us_from_steps(steps: u32) -> u32 {
    ((u64::from(steps) * u64::from(DUTY_STEP_NS)) / 1_000) as u32
}

/// A sentry's two periods and the arithmetic that sizes a wake around them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Sentry {
    /// Receive window, microseconds: how long an empty window listens.
    pub rx_us: u32,
    /// Sleep between windows, microseconds.
    pub sleep_us: u32,
    /// Oscillator startup, microseconds. Added between the sleep and the
    /// receive phases, so it lengthens the cycle and the deaf gap alike.
    pub tcxo_us: u32,
    /// One LoRa symbol at the modulation the sentry listens on.
    pub t_sym_us: u32,
}

impl Sentry {
    /// A sentry for `cfg`'s modulation with the given periods.
    pub fn new(cfg: &RadioConfig, rx_us: u32, sleep_us: u32) -> Self {
        Self {
            rx_us,
            sleep_us,
            tcxo_us: u32::from(cfg.tcxo_startup_ms) * 1_000,
            t_sym_us: cfg.symbol_time_us().max(1),
        }
    }

    /// The sentry `cfg` asks for, from its two wake keys.
    pub fn from_config(cfg: &RadioConfig) -> Self {
        Self::new(
            cfg,
            u32::from(cfg.wake_rx_ms) * 1_000,
            u32::from(cfg.wake_sleep_ms) * 1_000,
        )
    }

    fn detect_us(&self) -> u32 {
        DETECT_SYMBOLS * self.t_sym_us
    }

    fn header_us(&self) -> u32 {
        HEADER_SYMBOLS * self.t_sym_us
    }

    /// One full cycle: window, sleep, and the oscillator restart between.
    pub fn cycle_us(&self) -> u32 {
        self.rx_us
            .saturating_add(self.sleep_us)
            .saturating_add(self.tcxo_us)
    }

    /// How long the chip holds the receiver open once it has detected a
    /// preamble: the restarted timer.
    pub fn hold_us(&self) -> u32 {
        self.rx_us.saturating_mul(2).saturating_add(self.sleep_us)
    }

    /// Shortest preamble every cycle samples with room to detect.
    pub fn preamble_min_us(&self) -> u32 {
        self.sleep_us
            .saturating_add(self.tcxo_us)
            .saturating_add(self.detect_us().saturating_mul(2))
    }

    /// Longest preamble whose header still lands inside the hold.
    pub fn preamble_max_us(&self) -> u32 {
        self.hold_us().saturating_sub(self.header_us())
    }

    /// Whether any preamble satisfies both bounds.
    pub fn feasible(&self) -> bool {
        self.preamble_min_us() <= self.preamble_max_us()
    }

    /// The room between the bounds, microseconds: what the drift of two
    /// free-running timebases has to stay inside.
    pub fn margin_us(&self) -> u32 {
        self.preamble_max_us().saturating_sub(self.preamble_min_us())
    }

    /// The preamble to transmit with, in symbols: the middle of the window,
    /// so the drift has half the margin on either side.
    pub fn preamble_symbols(&self) -> Option<u32> {
        if !self.feasible() {
            return None;
        }
        let mid = self.preamble_min_us() + self.margin_us() / 2;
        Some(mid / self.t_sym_us)
    }

    /// The drift, in parts per million of the sleep, that the margin
    /// tolerates in either direction.
    pub fn drift_tolerance_ppm(&self) -> u32 {
        if self.sleep_us == 0 {
            return 0;
        }
        (u64::from(self.margin_us() / 2) * 1_000_000 / u64::from(self.sleep_us)) as u32
    }

    /// Share of each cycle the receiver is powered, in parts per thousand.
    pub fn duty_permille(&self) -> u32 {
        let cycle = self.cycle_us();
        if cycle == 0 {
            return 0;
        }
        (u64::from(self.rx_us) * 1000 / u64::from(cycle)) as u32
    }

    /// Time on air of a wake transmission carrying `payload_len` bytes.
    pub fn wake_airtime_us(&self, cfg: &RadioConfig, payload_len: usize) -> Option<u32> {
        let syms = self.preamble_symbols()?;
        Some(cfg.time_on_air_preamble_us(payload_len, syms))
    }

    /// The register value for the receive half.
    pub fn rx_steps(&self) -> u32 {
        duty_steps_from_us(self.rx_us)
    }

    /// The register value for the sleep half.
    pub fn sleep_steps(&self) -> u32 {
        duty_steps_from_us(self.sleep_us)
    }
}

/// Why a sentry cannot be armed as asked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The config does not ask for one.
    Disabled,
    /// No preamble satisfies both bounds: the receive window is too short
    /// to cover the oscillator restart, the detector and the header between
    /// them. Lengthen `wake_rx_ms`; the sleep moves both bounds together.
    NoPreambleFits,
    /// The preamble the bounds allow does not fit the 16-bit field.
    PreambleTooLong,
    /// Either half-period overflows the chip's 24-bit timer.
    PeriodTooLong,
    /// The node is hopping and the wake transmission would hold one carrier
    /// past what a frequency hopping system may.
    DwellExceeded,
}

impl Refusal {
    pub fn as_str(self) -> &'static str {
        match self {
            Refusal::Disabled => "wake_enabled is off",
            Refusal::NoPreambleFits => "no preamble fits - lengthen wake_rx_ms",
            Refusal::PreambleTooLong => "the preamble does not fit the packet parameters",
            Refusal::PeriodTooLong => "a period overflows the chip's timer",
            Refusal::DwellExceeded => "hopping - a wake frame would hold one channel too long",
        }
    }
}

/// Whether `sentry` can be armed on a node running `cfg`, for a wake
/// transmission carrying `payload_len` bytes.
pub fn check(cfg: &RadioConfig, sentry: &Sentry, payload_len: usize) -> Result<(), Refusal> {
    if !cfg.wake_enabled {
        return Err(Refusal::Disabled);
    }
    if duty_steps_from_us(sentry.rx_us) >= DUTY_STEPS_MAX
        || duty_steps_from_us(sentry.sleep_us) >= DUTY_STEPS_MAX
    {
        return Err(Refusal::PeriodTooLong);
    }
    let Some(syms) = sentry.preamble_symbols() else {
        return Err(Refusal::NoPreambleFits);
    };
    if syms > PREAMBLE_SYMBOLS_MAX {
        return Err(Refusal::PreambleTooLong);
    }
    let airtime = sentry.wake_airtime_us(cfg, payload_len).unwrap_or(u32::MAX);
    if cfg.hop_channels > 1 && airtime > FHSS_DWELL_MAX_US {
        return Err(Refusal::DwellExceeded);
    }
    Ok(())
}

/// The sentry `cfg` asks for, checked. What both the sleeping board and
/// the waker compute, from the same config, so the preamble one sends is
/// the one the other listens for.
pub fn plan(cfg: &RadioConfig, payload_len: usize) -> Result<Sentry, Refusal> {
    let s = Sentry::from_config(cfg);
    check(cfg, &s, payload_len)?;
    Ok(s)
}

/// Shortest receive window that admits any preamble at all, microseconds.
///
/// Half of what the oscillator restart, two detections and the header cost
/// together. Below this a sentry cannot be woken at any sleep period.
pub fn min_rx_us(cfg: &RadioConfig) -> u32 {
    let t_sym = cfg.symbol_time_us().max(1);
    let tcxo = u32::from(cfg.tcxo_startup_ms) * 1_000;
    (tcxo + 2 * DETECT_SYMBOLS * t_sym + HEADER_SYMBOLS * t_sym).div_ceil(2)
}

/// What a waker does next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Put a wake frame on the air.
    Send,
    /// Listen for the answer.
    Wait,
    /// Finished: whether the target answered.
    Done { heard: bool },
}

/// A waker's burst: a bounded number of wake frames, each followed by a
/// listen for the target's answer.
///
/// Bounded, because a waker that never gives up is a transmitter that
/// never stops, and both the band and the waker's own battery care. A
/// frame is only sent when the previous listen produced nothing; the first
/// answer ends the burst.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Waker {
    pub target: u8,
    pub tracking: bool,
    pub nonce: u8,
    tries_left: u8,
    gap_ms: u32,
    /// When the current listen ends; `None` before the first frame and
    /// after the burst is over.
    listen_until_ms: Option<u64>,
    heard: bool,
    done: bool,
}

impl Waker {
    pub fn new(target: u8, tracking: bool, nonce: u8) -> Self {
        Self {
            target,
            tracking,
            nonce,
            tries_left: WAKE_TRIES,
            gap_ms: WAKE_GAP_MS,
            listen_until_ms: None,
            heard: false,
            done: false,
        }
    }

    /// The same, with the burst's shape given.
    pub fn with_shape(mut self, tries: u8, gap_ms: u32) -> Self {
        self.tries_left = tries;
        self.gap_ms = gap_ms;
        self
    }

    /// A frame arrived from `src`. An answer from the target - or from
    /// anyone, for a broadcast - ends the burst.
    pub fn heard(&mut self, src: u8) {
        if self.done {
            return;
        }
        if self.target == crate::lora::WAKE_BROADCAST || src == self.target {
            self.heard = true;
        }
    }

    /// A frame went out at `now_ms`; listen from here.
    pub fn sent(&mut self, now_ms: u64) {
        self.tries_left = self.tries_left.saturating_sub(1);
        self.listen_until_ms = Some(now_ms + u64::from(self.gap_ms));
    }

    /// What to do at `now_ms`.
    pub fn step(&mut self, now_ms: u64) -> Step {
        if self.done {
            return Step::Done { heard: self.heard };
        }
        if self.heard {
            self.done = true;
            return Step::Done { heard: true };
        }
        match self.listen_until_ms {
            Some(until) if now_ms < until => Step::Wait,
            _ if self.tries_left == 0 => {
                self.done = true;
                Step::Done { heard: false }
            }
            _ => Step::Send,
        }
    }

    pub fn is_done(&self) -> bool {
        self.done
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped default: SF12, BW500, one channel, 300 ms every 3 s.
    fn cfg() -> RadioConfig {
        RadioConfig::default()
    }

    #[test]
    fn duty_steps_round_trip_at_the_step() {
        assert_eq!(duty_steps_from_us(125), 8);
        assert_eq!(duty_us_from_steps(8), 125);
        assert_eq!(duty_steps_from_us(1_000_000), 64_000);
        assert_eq!(duty_us_from_steps(64_000), 1_000_000);
        assert_eq!(duty_steps_from_us(u32::MAX), DUTY_STEPS_MAX);
    }

    /// The bench's five arms, against a 165-symbol preamble (1 351 680 us
    /// at SF12/BW500). The three that woke on every frame are inside both
    /// bounds; the one that lost a third of them is over the upper bound;
    /// and the old arm, whose symbol timeout was the real fault, is not
    /// something this model can express - it is what the model forbids.
    #[test]
    fn the_bounds_sort_the_bench_arms_as_measured() {
        let preamble = 165 * cfg().symbol_time_us();
        let inside = |rx_ms: u32, sleep_ms: u32| {
            let s = Sentry::new(&cfg(), rx_ms * 1000, sleep_ms * 1000);
            preamble >= s.preamble_min_us() && preamble <= s.preamble_max_us()
        };
        assert!(inside(300, 900), "100% (29 of 29)");
        assert!(inside(250, 950), "100% (29 of 29)");
        assert!(!inside(150, 1000), "62% (18 of 29): the hold is too short");
        let short = Sentry::new(&cfg(), 150_000, 1_000_000);
        assert!(preamble > short.preamble_max_us());
        assert!(preamble >= short.preamble_min_us());
    }

    #[test]
    fn the_sleep_cancels_out_of_the_margin() {
        let a = Sentry::new(&cfg(), 300_000, 1_000_000);
        let b = Sentry::new(&cfg(), 300_000, 4_000_000);
        assert_eq!(a.margin_us(), b.margin_us());
        // 2 * 300 - 10 - 65.5 - 65.5 ms
        assert_eq!(a.margin_us(), 600_000 - 10_000 - 8 * 8_192 - 8 * 8_192);
        // And a longer window is the only thing that widens it.
        let c = Sentry::new(&cfg(), 400_000, 1_000_000);
        assert!(c.margin_us() > a.margin_us());
    }

    #[test]
    fn the_default_config_is_feasible_with_room_for_drift() {
        let s = plan(&cfg(), 8).expect("the shipped default arms");
        assert!(s.feasible());
        let syms = s.preamble_symbols().unwrap();
        let us = syms * s.t_sym_us;
        assert!(us >= s.preamble_min_us() && us <= s.preamble_max_us());
        // A percent-level RC oscillator over a 3 s sleep is tens of ms;
        // the default tolerates several percent.
        assert!(s.drift_tolerance_ppm() >= 40_000, "{} ppm", s.drift_tolerance_ppm());
        // And the receiver is off most of the time.
        assert!(s.duty_permille() < 120, "{} permille", s.duty_permille());
    }

    #[test]
    fn a_window_under_the_floor_admits_no_preamble() {
        let floor = min_rx_us(&cfg());
        let under = Sentry::new(&cfg(), floor - 1_000, 3_000_000);
        assert!(!under.feasible());
        assert_eq!(under.preamble_symbols(), None);
        let at = Sentry::new(&cfg(), floor, 3_000_000);
        assert!(at.feasible());
        let mut c = cfg();
        c.wake_rx_ms = ((floor - 1_000) / 1000) as u16;
        assert_eq!(plan(&c, 8), Err(Refusal::NoPreambleFits));
    }

    #[test]
    fn refusals_name_their_cause() {
        let mut c = cfg();
        c.wake_enabled = false;
        assert_eq!(plan(&c, 8), Err(Refusal::Disabled));
        let mut c = cfg();
        c.hop_channels = 50;
        assert_eq!(plan(&c, 8), Err(Refusal::DwellExceeded));
        // Neither overflow is reachable from the config keys at the
        // default modulation - the periods top out at 65 s, the timer at
        // 262 s and the preamble at 536 s - so both are checked on
        // sentries built by hand.
        let huge = Sentry::new(&cfg(), 300_000_000, 300_000_000);
        assert_eq!(check(&cfg(), &huge, 8), Err(Refusal::PeriodTooLong));
        // At SF7 a symbol is 256 us, so 65535 of them is under 17 s and a
        // 40 s preamble does not fit the field.
        let mut fast = cfg();
        fast.spreading_factor = 7;
        let long = Sentry::new(&fast, 20_000_000, 20_000_000);
        assert_eq!(check(&fast, &long, 8), Err(Refusal::PreambleTooLong));
    }

    #[test]
    fn a_waker_sends_listens_and_stops_on_the_first_answer() {
        let mut w = Waker::new(5, false, 1).with_shape(3, 1_000);
        assert_eq!(w.step(0), Step::Send);
        w.sent(0);
        assert_eq!(w.step(500), Step::Wait);
        w.heard(6);
        assert_eq!(w.step(600), Step::Wait, "someone else is not the answer");
        w.heard(5);
        assert_eq!(w.step(700), Step::Done { heard: true });
        assert!(w.is_done());
        assert_eq!(w.step(5_000), Step::Done { heard: true });
    }

    #[test]
    fn a_waker_gives_up_after_its_tries() {
        let mut w = Waker::new(5, true, 1).with_shape(2, 1_000);
        assert_eq!(w.step(0), Step::Send);
        w.sent(0);
        assert_eq!(w.step(1_000), Step::Send);
        w.sent(1_000);
        assert_eq!(w.step(1_999), Step::Wait);
        assert_eq!(w.step(2_000), Step::Done { heard: false });
        w.heard(5);
        assert_eq!(w.step(2_001), Step::Done { heard: false }, "too late");
    }

    #[test]
    fn a_broadcast_takes_any_answer() {
        let mut w = Waker::new(crate::lora::WAKE_BROADCAST, false, 9);
        assert_eq!(w.step(0), Step::Send);
        w.sent(0);
        w.heard(42);
        assert_eq!(w.step(1), Step::Done { heard: true });
    }
}
