//! Sentry timing: what a duty-cycled receiver, and the transmission meant
//! to catch it, have to satisfy between them.
//!
//! A receiver cycling receive and sleep on its own timers is deaf for most
//! of every cycle, so a transmission it is meant to hear cannot be an
//! ordinary frame. Its preamble has to be long enough that one receive
//! window falls inside it wherever the two happen to line up. That is one
//! inequality, and everything here is either that inequality, the chip's
//! register encoding for it, or the bench measurement that supplies its one
//! unknown.
//!
//! The unknown is what a receive window costs beyond the symbols it is
//! meant to hear. The chip restarts its oscillator on every window, and
//! whether that time is *added* to the commanded window or taken *out* of
//! it is a property of the part rather than something to assume: get it
//! backwards and every window in the design listens for less than it looks
//! like it does, which shows up as a sentry that misses preambles the
//! arithmetic says it should hear. [`sweep_floor`] turns a bench sweep into
//! that number and [`classify_charge`] reads the same run for which of the
//! two behaviors the part has.
//!
//! Pure, like the rest of the protocol crate: no timers, no registers, no
//! radio. The firmware supplies the measurements and performs the effects.

use crate::radiocfg::RadioConfig;

/// The SX126x duty-cycle timer step in nanoseconds: 15.625 us, the same
/// unit `SetRx`'s timeout counts in.
pub const DUTY_STEP_NS: u32 = 15_625;

/// Widest value the 24-bit `rxPeriod` and `sleepPeriod` fields carry, and
/// so the longest either half of a cycle can be - about 262 s.
pub const DUTY_STEPS_MAX: u32 = 0x00FF_FFFF;

/// Largest preamble the packet parameters can ask for: the field is 16 bits
/// of symbols.
pub const PREAMBLE_SYMBOLS_MAX: u32 = u16::MAX as u32;

/// Symbols the modem is told to count before it accepts that a signal is
/// really there (`SetLoRaSymbNumTimeout`).
///
/// Four is the usual floor for a LoRa preamble detection to be trustworthy
/// rather than noise, and it is what the receive window is sized around. A
/// larger value is a longer window for the same confidence, which on a
/// sentry is paid on every cycle for the life of the board.
///
/// **Zero is not a setting here.** It disables the modem's validation, and
/// on hardware that takes a duty cycle from waking on most frames to waking
/// on none: a non-zero value is also what makes the chip hold its receive
/// window open for the rest of a packet once it has validated one, so with
/// the check off the window ends at its own length and the chip sleeps
/// through a frame it had already heard the start of.
pub const DETECT_SYMBOLS: u8 = 4;

/// Longest one transmission may hold a single carrier when the node is
/// hopping, in microseconds.
///
/// A frequency hopping system may occupy one channel for 400 ms in any 20 s
/// under the 902-928 MHz rules; a single-carrier digital modulation system
/// has no such limit, which is why a wake preamble is a one-channel feature
/// and a hopping plan has to refuse it rather than shorten it.
pub const FHSS_DWELL_MAX_US: u32 = 400_000;

/// Convert microseconds to the chip's 15.625 us duty-cycle step, saturating
/// at the 24-bit field.
///
/// 1000/15625 reduces to 8/125, so this is exact in integer math for every
/// input rather than a rounded division through a float.
pub fn duty_steps_from_us(us: u32) -> u32 {
    let steps = u64::from(us) * 8 / 125;
    if steps > u64::from(DUTY_STEPS_MAX) {
        DUTY_STEPS_MAX
    } else {
        steps as u32
    }
}

/// Microseconds a duty-cycle step count stands for. The inverse of
/// [`duty_steps_from_us`] up to the truncation that function performs.
pub fn duty_us_from_steps(steps: u32) -> u32 {
    (u64::from(steps) * 125 / 8) as u32
}

/// Whether the chip's oscillator restart is added to the commanded receive
/// window or taken out of it.
///
/// The distinction is invisible from a datasheet reading and decides
/// whether a window listens for as long as it was asked to. It is read off
/// the interval between detections on a sentry pointed at a continuous
/// preamble: a cycle that measures longer than it was commanded by about
/// the overhead has the startup added, one that measures the commanded
/// length has it absorbed - and an absorbed startup means every window in
/// the design has to grow by that much.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TcxoCharge {
    /// The cycle runs long by roughly the overhead: the window listens for
    /// as long as it was commanded.
    Added,
    /// The cycle runs to the commanded length: the startup came out of the
    /// window, which listened for that much less.
    Absorbed,
    /// Neither, by more than the tolerance. Not a verdict - a reason to
    /// look at the run rather than to size a design from it.
    Unclear,
}

/// Read [`TcxoCharge`] off a measured cycle.
///
/// `commanded_us` is `rx + sleep` as asked for, `observed_us` the mean
/// interval between detections, and `overhead_us` the per-window cost from
/// [`sweep_floor`]. The tolerance is a quarter of the overhead, which is
/// wide enough for the chip's own RC timebase and narrow enough that the
/// two cases cannot both match.
pub fn classify_charge(commanded_us: u32, observed_us: u32, overhead_us: u32) -> TcxoCharge {
    // With no overhead to speak of there is nothing to tell apart, and the
    // honest answer is that the question did not arise.
    let tol = overhead_us / 4;
    if tol == 0 {
        return TcxoCharge::Unclear;
    }
    let added = commanded_us.saturating_add(overhead_us);
    if observed_us.abs_diff(added) <= tol {
        TcxoCharge::Added
    } else if observed_us.abs_diff(commanded_us) <= tol {
        TcxoCharge::Absorbed
    } else {
        TcxoCharge::Unclear
    }
}

/// Intervals that are one cycle apart, told from intervals that are not.
///
/// A source that is not on the air continuously leaves gaps, and a gap only
/// ever makes an interval *longer* - the receiver kept cycling, nothing was
/// there to hear. So the intervals worth measuring are the short ones, and
/// the long ones are the source rather than the receiver.
///
/// Without this a bursting source reads as a chip that stopped cycling,
/// which is a conclusion about the wrong end of the link and one that would
/// send the design off after a fault that is not there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Intervals {
    /// Mean of the intervals that sat within tolerance of one cycle.
    pub mean_us: u32,
    pub min_us: u32,
    pub max_us: u32,
    /// How many were one cycle apart - consecutive windows, both hearing.
    pub tight: u32,
    /// How many were longer, i.e. had a gap in them.
    pub loose: u32,
}

/// Summarize measured detection intervals against the commanded cycle.
///
/// An interval counts as tight when it is no more than half a cycle over
/// one: long enough to absorb the oscillator restart and the chip's own RC
/// timebase, short enough that a skipped window cannot hide in it. Returns
/// `None` when nothing was tight, which means either the cycle never ran or
/// the source was never on for two windows together - and those are told
/// apart by whether anything was detected at all.
pub fn summarize_intervals(intervals: &[u32], commanded_us: u32) -> Option<Intervals> {
    let limit = commanded_us.saturating_add(commanded_us / 2);
    let mut sum = 0u64;
    let mut tight = 0u32;
    let mut loose = 0u32;
    let mut min_us = u32::MAX;
    let mut max_us = 0u32;
    for &v in intervals {
        if v <= limit {
            tight += 1;
            sum += u64::from(v);
            min_us = min_us.min(v);
            max_us = max_us.max(v);
        } else {
            loose += 1;
        }
    }
    if tight == 0 {
        return None;
    }
    Some(Intervals {
        mean_us: (sum / u64::from(tight)) as u32,
        min_us,
        max_us,
        tight,
        loose,
    })
}

/// One step of a receive-window sweep: how many cycles ran at this window
/// length and how many of them detected the signal that was present
/// throughout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SweepStep {
    pub rx_us: u32,
    pub cycles: u32,
    pub detects: u32,
}

impl SweepStep {
    /// Whether every cycle at this window detected. Zero cycles is not a
    /// pass - nothing was tried.
    pub fn clean(&self) -> bool {
        self.cycles > 0 && self.detects == self.cycles
    }
}

/// What a sweep says the receive window really costs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Floor {
    /// Shortest window that detected on every cycle.
    pub rx_us: u32,
    /// What that window spends on something other than hearing symbols:
    /// the oscillator restart and whatever else the chip does per window.
    pub overhead_us: u32,
}

/// The shortest receive window a sweep found reliable, and the per-window
/// overhead it implies.
///
/// `steps` may arrive in any order. A window is only accepted if it and
/// every longer window in the sweep detected on every cycle: a short window
/// that happened to pass while a longer one failed is a run to repeat, not
/// a floor to design against, and taking the shortest clean step on its own
/// would quietly adopt the luckier of the two. Returns `None` when nothing
/// passed, or when the sweep is not clean from its top down.
///
/// The overhead is the found window less the symbols it was supposed to be
/// listening for. It comes back saturated at zero rather than negative: a
/// window shorter than its own symbol budget means the detection needed
/// fewer symbols than `detect_symbols` claims, which is a finding about the
/// modem and not a negative cost.
pub fn sweep_floor(steps: &[SweepStep], t_sym_us: u32, detect_symbols: u8) -> Option<Floor> {
    let top = steps.iter().map(|s| s.rx_us).max()?;
    // Walk down from the longest window, stopping at the first failure.
    // What is left below it may pass, but it passes underneath a length
    // that did not, so it is not a floor.
    let mut best: Option<u32> = None;
    let mut rx = top;
    loop {
        // Summed rather than taken singly, so a step run more than once
        // counts as the trials it actually was: two runs of twenty with one
        // miss between them is a step that did not pass.
        let step = steps
            .iter()
            .filter(|s| s.rx_us == rx)
            .fold(SweepStep { rx_us: rx, cycles: 0, detects: 0 }, |acc, s| {
                SweepStep {
                    rx_us: rx,
                    cycles: acc.cycles + s.cycles,
                    detects: acc.detects + s.detects,
                }
            });
        if !step.clean() {
            break;
        }
        best = Some(rx);
        // The next window down, if the sweep has one.
        match steps.iter().map(|s| s.rx_us).filter(|&v| v < rx).max() {
            Some(next) => rx = next,
            None => break,
        }
    }
    let rx_us = best?;
    let listening_us = u32::from(detect_symbols) * t_sym_us;
    Some(Floor {
        rx_us,
        overhead_us: rx_us.saturating_sub(listening_us),
    })
}

/// What the chip's own sleep timer runs at, against the commanded value.
///
/// The sleep half of a duty cycle is counted by the SX126x's RC64k, which
/// the datasheet calibrates against the crystal at power-on and on a
/// `Calibrate` command - and then does not specify. It is an RC oscillator,
/// so it drifts with temperature from wherever calibration left it, by an
/// amount the part does not commit to.
///
/// That matters because the preamble window has to absorb the error over a
/// whole sleep period: a sleep that runs 1% fast moves the instant a window
/// opens by 1% of the sleep, and the window is only a few tens of
/// milliseconds wide. So this is measured rather than assumed, and it is
/// what sets the receive window through [`min_rx_for_margin`].
///
/// Positive parts-per-million means the timer ran *long* - the measured
/// interval was more than the commanded one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RcRate {
    pub mean_ppm: i32,
    pub min_ppm: i32,
    pub max_ppm: i32,
    pub trials: u32,
}

impl RcRate {
    /// The largest error either side of nominal, in parts per million.
    ///
    /// What a margin must cover only if the commanded period is left
    /// uncorrected. Usually it should not be - see [`spread_ppm`](Self::spread_ppm).
    pub fn worst_abs_ppm(&self) -> u32 {
        self.min_ppm.unsigned_abs().max(self.max_ppm.unsigned_abs())
    }

    /// The part of the error a margin actually has to absorb: how far the
    /// trials sit from their own mean, rather than from nominal.
    ///
    /// The distinction is worth real current. An RC oscillator's error
    /// splits into an offset, which is the same every cycle and can simply
    /// be divided out of the period commanded, and a spread, which cannot.
    /// Sizing a receive window against the offset buys margin for a drift
    /// that does not happen, and the window is what the sentry's current is
    /// proportional to.
    ///
    /// What this cannot see is temperature. These trials run seconds apart
    /// on a board at one temperature, so the spread they show is short-term
    /// jitter; a board stored for weeks moves with its surroundings, and
    /// that is a longer measurement than this one.
    pub fn spread_ppm(&self) -> u32 {
        (self.max_ppm - self.mean_ppm)
            .unsigned_abs()
            .max((self.mean_ppm - self.min_ppm).unsigned_abs())
    }

    /// The period to command so the chip produces `want_us`.
    ///
    /// Divides the measured offset back out: a timer running long by its
    /// mean error is asked for proportionally less.
    pub fn correct_us(&self, want_us: u32) -> u32 {
        let scaled = i64::from(want_us) * 1_000_000 / (1_000_000 + i64::from(self.mean_ppm));
        scaled.clamp(0, i64::from(u32::MAX)) as u32
    }
}

/// Summarize measured timer intervals against what was asked for.
///
/// `measured_us` are the real elapsed times, by a clock that can be
/// trusted - the host's crystal - for a timeout commanded as
/// `commanded_us` and counted by the chip.
pub fn rc_rate(commanded_us: u32, measured_us: &[u32]) -> Option<RcRate> {
    if commanded_us == 0 || measured_us.is_empty() {
        return None;
    }
    let mut sum = 0i64;
    let mut min_ppm = i32::MAX;
    let mut max_ppm = i32::MIN;
    for &m in measured_us {
        // (measured - commanded) / commanded, in parts per million, with
        // the multiply first so integer division does not eat the result.
        let ppm = ((i64::from(m) - i64::from(commanded_us)) * 1_000_000
            / i64::from(commanded_us)) as i32;
        sum += i64::from(ppm);
        min_ppm = min_ppm.min(ppm);
        max_ppm = max_ppm.max(ppm);
    }
    Some(RcRate {
        mean_ppm: (sum / measured_us.len() as i64) as i32,
        min_ppm,
        max_ppm,
        trials: measured_us.len() as u32,
    })
}

/// How far a sleep of `sleep_us` can land off its nominal instant when the
/// timer counting it is wrong by `ppm_abs`, in microseconds.
///
/// The preamble window has to be at least this wide, or a wake that was
/// sized correctly on paper misses because the receiver woke at the wrong
/// moment.
pub fn drift_us(sleep_us: u32, ppm_abs: u32) -> u32 {
    ((u64::from(sleep_us) * u64::from(ppm_abs)) / 1_000_000) as u32
}

/// The receive window a sentry needs so its preamble window covers both the
/// timer's error over one sleep and the fixed costs of detection.
///
/// Inverts `margin = 2 * rxPeriod - (Theader + Tdetect + Ttcxo)`: the margin
/// wanted is the drift, so the window follows from it. This is the number
/// the whole design hangs on, because the receive window is the only lever
/// on margin and it is also what the sentry's current is proportional to.
pub fn min_rx_for_margin(
    cfg: &RadioConfig,
    detect_symbols: u8,
    tcxo_us: u32,
    sleep_us: u32,
    ppm_abs: u32,
    payload_len: usize,
) -> u32 {
    let fixed = min_rx_us(cfg, detect_symbols, tcxo_us, payload_len) * 2;
    // Two-sided: the sleep may run long or short, so the window has to hold
    // the drift in either direction.
    let need = fixed.saturating_add(drift_us(sleep_us, ppm_abs).saturating_mul(2));
    need.div_ceil(2)
}

/// Symbols the LoRa explicit header occupies, which the chip must also
/// receive inside its restarted timer once it has heard a preamble.
pub const HEADER_SYMBOLS: u32 = 8;

/// A sentry's two half-periods and the preamble window that reaches it.
///
/// The preamble is bounded at *both* ends, which is the whole difficulty:
///
/// - **Below**, because the receiver is deaf for the sleep phase and the
///   oscillator restart behind it, so a preamble shorter than that gap can
///   fall entirely inside one and never be sampled.
/// - **Above**, because of what the chip does when it *does* hear one. The
///   sniff loop stops its window timer on preamble detection and restarts
///   it at `2 * rxPeriod + sleepPeriod`; anything still arriving when that
///   expires is abandoned, and the packet is never received.
///
/// Both the Semtech datasheet and ST's RM0461, which documents the same die
/// as the SUBGHZ peripheral, write this bound the same way and count only
/// the header: `Tpreamble + Theader < 2 * rxPeriod + sleepPeriod`. The
/// restarted timer exists to find a header, not to finish a packet.
///
/// A payload term was briefly added here to explain a geometry that met the
/// documented bound and still failed on hardware. It was wrong twice over:
/// doubling the receive window afterwards changed nothing, which the theory
/// said it should, and RM0461 states the header-only form explicitly. What
/// made that geometry fail is still unexplained, and inventing a bound to
/// cover it only hid the fact.
///
/// So a longer preamble is not the safe direction. Past the upper bound the
/// wake stops working again, and it fails the same way it fails below the
/// lower one - silently, with the receiver awake and listening.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sentry {
    /// Receive window, microseconds: the symbols to be counted plus the
    /// measured per-window overhead.
    pub rx_us: u32,
    /// Sleep between windows, microseconds.
    pub sleep_us: u32,
    /// Symbols the modem counts before accepting a signal.
    pub detect_symbols: u8,
    /// Oscillator startup, microseconds. Added *between* the sleep and the
    /// receive phases rather than taken out of the window, so it lengthens
    /// the cycle and the deaf gap alike.
    pub tcxo_us: u32,
    /// Shortest preamble that can still be sampled by a window.
    pub preamble_min_us: u32,
    /// Longest preamble the chip's restarted timer will sit through.
    pub preamble_max_us: u32,
}

impl Sentry {
    /// Size a sentry for `cfg` from a sleep period, the per-window overhead
    /// a sweep measured, and the oscillator startup the config asks for.
    pub fn new(
        cfg: &RadioConfig,
        sleep_us: u32,
        overhead_us: u32,
        detect_symbols: u8,
        tcxo_us: u32,
        payload_len: usize,
    ) -> Self {
        // The payload's own air time, preamble excluded: `time_on_air` with
        // a zero-symbol preamble is the header and payload alone.
        let payload_us = cfg
            .time_on_air_preamble_us(payload_len, 0)
            .saturating_sub(cfg.time_on_air_preamble_us(0, 0));
        let t_sym_us = cfg.symbol_time_us().max(1);
        let detect_us = u32::from(detect_symbols).saturating_mul(t_sym_us);
        let header_us = HEADER_SYMBOLS.saturating_mul(t_sym_us);
        let rx_us = detect_us.saturating_add(overhead_us);
        // Only the header: the restarted timer is there to find one, and
        // both vendors' documents say so.
        let tail_us = header_us;
        let _ = payload_us;
        // Deaf for the sleep phase plus the oscillator restart behind it;
        // a window then needs its detect symbols inside what is left.
        let preamble_min_us = sleep_us
            .saturating_add(tcxo_us)
            .saturating_add(detect_us);
        // The chip's own post-detection timer, less the header it still has
        // to fit behind the preamble.
        let preamble_max_us = rx_us
            .saturating_mul(2)
            .saturating_add(sleep_us)
            .saturating_sub(tail_us);
        Self {
            rx_us,
            sleep_us,
            detect_symbols,
            tcxo_us,
            preamble_min_us,
            preamble_max_us,
        }
    }

    /// Whether a preamble exists that satisfies both bounds.
    ///
    /// It does only when the window is long enough to cover half of what
    /// detection and the header cost together - so the receive window, not
    /// the sleep, is what decides whether a sentry is possible at all.
    pub fn feasible(&self) -> bool {
        self.preamble_min_us <= self.preamble_max_us
    }

    /// The preamble to transmit with, in symbols: the middle of the window,
    /// so the drift of two free-running RC timebases has room either side.
    pub fn preamble_symbols(&self, cfg: &RadioConfig) -> Option<u32> {
        if !self.feasible() {
            return None;
        }
        let t_sym_us = cfg.symbol_time_us().max(1);
        let mid = self.preamble_min_us + (self.preamble_max_us - self.preamble_min_us) / 2;
        Some(mid / t_sym_us)
    }

    /// How much room the two bounds leave, microseconds. The margin every
    /// clock in the system has to stay inside.
    pub fn preamble_window_us(&self) -> u32 {
        self.preamble_max_us.saturating_sub(self.preamble_min_us)
    }

    /// One full cycle, microseconds: both phases and the oscillator restart
    /// the datasheet adds between them.
    pub fn cycle_us(&self) -> u32 {
        self.rx_us
            .saturating_add(self.sleep_us)
            .saturating_add(self.tcxo_us)
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
        let syms = self.preamble_symbols(cfg)?;
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
    /// No preamble satisfies both bounds: the receive window is too short to
    /// cover detection and the header between them. Lengthen `rx_us`;
    /// changing the sleep does not help, since it moves both bounds together.
    NoPreambleFits,
    /// The preamble the bounds allow does not fit the 16-bit field.
    PreambleTooLong,
    /// Either half-period overflows the chip's 24-bit timer.
    PeriodTooLong,
    /// The node is hopping and the wake transmission would hold one carrier
    /// past what a frequency hopping system may.
    DwellExceeded,
}

/// Whether `sentry` can be armed on a node running `cfg`, for a wake
/// transmission carrying `payload_len` bytes.
pub fn check(cfg: &RadioConfig, sentry: &Sentry, payload_len: usize) -> Result<(), Refusal> {
    let Some(syms) = sentry.preamble_symbols(cfg) else {
        return Err(Refusal::NoPreambleFits);
    };
    if syms > PREAMBLE_SYMBOLS_MAX {
        return Err(Refusal::PreambleTooLong);
    }
    if duty_steps_from_us(sentry.rx_us) >= DUTY_STEPS_MAX
        || duty_steps_from_us(sentry.sleep_us) >= DUTY_STEPS_MAX
    {
        return Err(Refusal::PeriodTooLong);
    }
    let airtime = sentry.wake_airtime_us(cfg, payload_len).unwrap_or(u32::MAX);
    if cfg.hop_channels > 1 && airtime > FHSS_DWELL_MAX_US {
        return Err(Refusal::DwellExceeded);
    }
    Ok(())
}

/// Shortest receive window that admits any preamble at all, microseconds.
///
/// Half of what detection and the header cost together, since the chip's
/// post-detection timer grants two receive windows for them. Below this a
/// sentry cannot be woken at any sleep period or preamble length.
pub fn min_rx_us(cfg: &RadioConfig, detect_symbols: u8, tcxo_us: u32, payload_len: usize) -> u32 {
    let t_sym_us = cfg.symbol_time_us().max(1);
    let detect_us = u32::from(detect_symbols) * t_sym_us;
    let header_us = HEADER_SYMBOLS * t_sym_us;
    let _ = payload_len;
    (tcxo_us + detect_us + header_us).div_ceil(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped default: SF12, BW500, one channel.
    fn cfg() -> RadioConfig {
        RadioConfig::default()
    }

    #[test]
    fn duty_steps_round_trip_at_the_step() {
        // One step is 15.625 us, so 125 us is exactly 8 of them - the
        // smallest whole-microsecond input that is also a whole number of
        // steps.
        assert_eq!(duty_steps_from_us(125), 8);
        assert_eq!(duty_us_from_steps(8), 125);
        assert_eq!(duty_steps_from_us(1_000_000), 64_000);
        assert_eq!(duty_us_from_steps(64_000), 1_000_000);
    }

    #[test]
    fn duty_steps_truncate_rather_than_wrap() {
        // Under one step is no steps, not a wrap to the top of the field.
        assert_eq!(duty_steps_from_us(15), 0);
        // Past the 24-bit field it saturates. 0xFFFFFF steps is ~262.1 s.
        assert_eq!(duty_steps_from_us(u32::MAX), DUTY_STEPS_MAX);
        // Just under the ceiling still encodes exactly, not at the clamp.
        assert_eq!(duty_steps_from_us(262_143_000), 16_777_152);
        assert!(duty_steps_from_us(262_143_000) < DUTY_STEPS_MAX);
    }

    #[test]
    fn a_gap_in_the_source_does_not_read_as_a_stopped_cycle() {
        // Nine intervals at the cycle, then the source went quiet for three
        // cycles, then nine more. The cycle is the tight ones; the gap is
        // about the transmitter and must not move the measurement.
        let commanded = 1_043_000;
        let mut v = [commanded + 9_000; 19];
        v[9] = commanded * 3;
        let i = summarize_intervals(&v, commanded).unwrap();
        assert_eq!(i.tight, 18);
        assert_eq!(i.loose, 1);
        assert_eq!(i.mean_us, commanded + 9_000);
        // And the verdict off that mean is still the right one.
        assert_eq!(
            classify_charge(commanded, i.mean_us, 10_000),
            TcxoCharge::Added
        );
    }

    #[test]
    fn nothing_within_a_cycle_is_no_measurement_at_all() {
        let commanded = 1_043_000;
        // Every interval is several cycles: no two consecutive windows ever
        // both heard, so there is no cycle here to report.
        assert_eq!(summarize_intervals(&[commanded * 4, commanded * 7], commanded), None);
        assert_eq!(summarize_intervals(&[], commanded), None);
    }

    #[test]
    fn half_a_cycle_over_is_the_edge_of_tight() {
        let commanded = 1_000_000;
        // A skipped window would show up as two cycles, which is past it.
        assert_eq!(summarize_intervals(&[1_500_000], commanded).unwrap().tight, 1);
        assert_eq!(summarize_intervals(&[1_500_001], commanded), None);
    }

    #[test]
    fn a_clean_sweep_gives_its_shortest_step() {
        let t_sym = cfg().symbol_time_us();
        let steps = [
            SweepStep { rx_us: 100_000, cycles: 40, detects: 40 },
            SweepStep { rx_us: 60_000, cycles: 40, detects: 40 },
            SweepStep { rx_us: 43_000, cycles: 40, detects: 40 },
        ];
        let f = sweep_floor(&steps, t_sym, DETECT_SYMBOLS).unwrap();
        assert_eq!(f.rx_us, 43_000);
        // 4 symbols at SF12/BW500 is 4 * 8192 us; the rest is overhead.
        assert_eq!(f.overhead_us, 43_000 - 4 * 8_192);
    }

    #[test]
    fn a_hole_in_the_sweep_is_not_stepped_over() {
        // 30 ms passed while 43 ms did not. Taking the shortest clean step
        // would adopt the luckier of the two and design every preamble in
        // the system against a window that is not reliable.
        let t_sym = cfg().symbol_time_us();
        let steps = [
            SweepStep { rx_us: 100_000, cycles: 40, detects: 40 },
            SweepStep { rx_us: 60_000, cycles: 40, detects: 40 },
            SweepStep { rx_us: 43_000, cycles: 40, detects: 37 },
            SweepStep { rx_us: 30_000, cycles: 40, detects: 40 },
        ];
        assert_eq!(sweep_floor(&steps, t_sym, DETECT_SYMBOLS).unwrap().rx_us, 60_000);
    }

    #[test]
    fn a_step_run_twice_counts_as_all_its_trials() {
        let t_sym = cfg().symbol_time_us();
        let steps = [
            SweepStep { rx_us: 60_000, cycles: 20, detects: 20 },
            SweepStep { rx_us: 43_000, cycles: 20, detects: 20 },
            // The same window, run again, with a miss in it. Taken singly
            // the first run would carry the step; summed it does not.
            SweepStep { rx_us: 43_000, cycles: 20, detects: 19 },
        ];
        assert_eq!(sweep_floor(&steps, t_sym, DETECT_SYMBOLS).unwrap().rx_us, 60_000);
    }

    #[test]
    fn a_sweep_that_never_detects_has_no_floor() {
        let t_sym = cfg().symbol_time_us();
        let steps = [
            SweepStep { rx_us: 100_000, cycles: 40, detects: 0 },
            SweepStep { rx_us: 60_000, cycles: 40, detects: 0 },
        ];
        assert_eq!(sweep_floor(&steps, t_sym, DETECT_SYMBOLS), None);
        assert_eq!(sweep_floor(&[], t_sym, DETECT_SYMBOLS), None);
        // A step nothing ran at is not a pass either.
        let none = [SweepStep { rx_us: 100_000, cycles: 0, detects: 0 }];
        assert_eq!(sweep_floor(&none, t_sym, DETECT_SYMBOLS), None);
    }

    #[test]
    fn a_window_under_its_symbol_budget_reports_no_negative_cost() {
        // Detection with less than four symbols' worth of window is a
        // finding about the modem, not an overhead below zero.
        let steps = [SweepStep { rx_us: 1_000, cycles: 10, detects: 10 }];
        let f = sweep_floor(&steps, 8_192, DETECT_SYMBOLS).unwrap();
        assert_eq!(f.overhead_us, 0);
    }

    #[test]
    fn the_two_tcxo_behaviors_are_told_apart() {
        // Commanded 1043 ms, overhead 10 ms.
        let commanded = 1_043_000;
        assert_eq!(
            classify_charge(commanded, commanded + 10_000, 10_000),
            TcxoCharge::Added
        );
        assert_eq!(
            classify_charge(commanded, commanded, 10_000),
            TcxoCharge::Absorbed
        );
        // Half a cycle out is neither, and must not be reported as either.
        assert_eq!(
            classify_charge(commanded, commanded + 500_000, 10_000),
            TcxoCharge::Unclear
        );
        // With no overhead there is nothing to distinguish.
        assert_eq!(classify_charge(commanded, commanded, 0), TcxoCharge::Unclear);
    }

    /// The board's configured oscillator startup, microseconds.
    const TCXO_US: u32 = 10_000;
    /// A wake frame's payload: the frame header and a short message.
    const WAKE_LEN: usize = 8;

    #[test]
    fn a_timer_that_runs_long_reads_positive() {
        // 1 s commanded, 1.01 s measured: the chip's timer is 1% slow, so
        // the interval it produced is 1% long.
        let r = rc_rate(1_000_000, &[1_010_000]).unwrap();
        assert_eq!(r.mean_ppm, 10_000);
        assert_eq!(r.worst_abs_ppm(), 10_000);
        // And short reads negative, with the same magnitude on the margin.
        let r = rc_rate(1_000_000, &[990_000]).unwrap();
        assert_eq!(r.mean_ppm, -10_000);
        assert_eq!(r.worst_abs_ppm(), 10_000);
    }

    #[test]
    fn a_steady_offset_is_corrected_rather_than_covered() {
        // What the bench measured: a large, very consistent error. Covering
        // it with margin would size the receive window for 1% of the sleep
        // when the cycle-to-cycle variation is a small fraction of that.
        let r = rc_rate(2_000_000, &[2_021_374, 2_021_314, 2_021_502]).unwrap();
        assert!(r.mean_ppm > 10_000, "a large offset");
        assert!(r.spread_ppm() < 100, "but a tiny spread");
        assert!(r.worst_abs_ppm() > 100 * r.spread_ppm());
        // Commanding the corrected period gets the interval actually wanted.
        let asked = r.correct_us(1_000_000);
        assert!(asked < 1_000_000, "a timer running long is asked for less");
        let produced = asked + drift_us(asked, r.mean_ppm.unsigned_abs());
        assert!(produced.abs_diff(1_000_000) < 1_000);
    }

    #[test]
    fn the_spread_is_what_the_window_has_to_pay_for() {
        let c = cfg();
        let r = rc_rate(2_000_000, &[2_021_374, 2_021_314, 2_021_502]).unwrap();
        let uncorrected = min_rx_for_margin(&c, DETECT_SYMBOLS, TCXO_US, 1_000_000, r.worst_abs_ppm(), WAKE_LEN);
        let corrected = min_rx_for_margin(&c, DETECT_SYMBOLS, TCXO_US, 1_000_000, r.spread_ppm(), WAKE_LEN);
        assert!(corrected < uncorrected);
        // And with the offset divided out the window is essentially the
        // fixed floor, which is the cheapest a sentry can be.
        assert!(corrected - min_rx_us(&c, DETECT_SYMBOLS, TCXO_US, WAKE_LEN) < 1_000);
    }

    #[test]
    fn the_margin_covers_the_worse_side_not_the_average() {
        // A timer that mostly runs fast but sometimes runs slow has to be
        // covered at its worst, in whichever direction that falls - an
        // average near zero would size the window for a drift that does
        // not happen.
        let r = rc_rate(1_000_000, &[1_020_000, 980_000, 1_001_000]).unwrap();
        assert!(r.mean_ppm.abs() < 5_000, "the average hides it");
        assert_eq!(r.worst_abs_ppm(), 20_000);
    }

    #[test]
    fn rc_rate_needs_something_to_measure() {
        assert_eq!(rc_rate(1_000_000, &[]), None);
        assert_eq!(rc_rate(0, &[1_000]), None);
    }

    #[test]
    fn drift_scales_with_the_sleep_it_is_measured_over() {
        // The reason a long sleep is not free: the same timer error costs
        // proportionally more margin the longer it is counting for.
        assert_eq!(drift_us(1_000_000, 10_000), 10_000);
        assert_eq!(drift_us(4_000_000, 10_000), 40_000);
    }

    #[test]
    fn a_drifting_timer_buys_its_margin_with_receive_window() {
        let c = cfg();
        let steady = min_rx_for_margin(&c, DETECT_SYMBOLS, TCXO_US, 1_000_000, 0, WAKE_LEN);
        let sloppy = min_rx_for_margin(&c, DETECT_SYMBOLS, TCXO_US, 1_000_000, 10_000, WAKE_LEN);
        // With a perfect timer the window is just the fixed costs.
        assert_eq!(steady, min_rx_us(&c, DETECT_SYMBOLS, TCXO_US, WAKE_LEN));
        // At 1% over a one-second sleep it has to grow by the drift.
        assert_eq!(sloppy, steady + drift_us(1_000_000, 10_000));
        // And a sentry built to it actually admits a preamble.
        let s = Sentry::new(&c, 1_000_000, sloppy - 4 * c.symbol_time_us(), DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        assert!(s.feasible());
        assert!(s.preamble_window_us() >= 2 * drift_us(1_000_000, 10_000));
    }

    #[test]
    fn the_preamble_is_bounded_at_both_ends() {
        let c = cfg();
        let t_sym = c.symbol_time_us();
        assert_eq!(t_sym, 8_192, "SF12 at BW500");
        let s = Sentry::new(&c, 1_000_000, 300_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        // Below: deaf for the sleep and the oscillator restart, then the
        // detect symbols have to fit in what is left of the preamble.
        assert_eq!(s.preamble_min_us, 1_000_000 + TCXO_US + 4 * t_sym);
        // Above: the chip's restarted timer, less the header behind it.
        // Above: the chip's restarted timer, less the header it still has
        // to find inside it. Only the header - both vendors say so.
        assert_eq!(s.preamble_max_us, 2 * s.rx_us + 1_000_000 - 8 * t_sym);
        assert!(s.feasible());
        let syms = s.preamble_symbols(&c).unwrap();
        assert!(syms * t_sym >= s.preamble_min_us);
        assert!(syms * t_sym <= s.preamble_max_us);
    }

    #[test]
    fn a_hundred_millisecond_window_is_within_the_documented_bound() {
        // Kept because the hardware disagrees with it, and that is the open
        // question rather than a bound to be adjusted until it matches.
        // Both vendors' documents make this geometry legal; on the bench it
        // produced two headers from thirty-nine detected preambles, and
        // doubling the window did not improve it.
        let c = cfg();
        const SYMBS: u8 = 8;
        let rx_100ms = 100_000 - u32::from(SYMBS) * c.symbol_time_us();
        let s = Sentry::new(&c, 1_000_000, rx_100ms, SYMBS, TCXO_US, WAKE_LEN);
        assert!(s.feasible(), "the documented bound admits this");
        assert_eq!(check(&c, &s, WAKE_LEN), Ok(()));
    }

    #[test]
    fn a_longer_preamble_is_not_the_safe_direction() {
        // The failure this arithmetic exists to prevent. A preamble sized
        // the intuitive way - long enough to span two whole cycles - sails
        // past the upper bound, and the chip abandons it before the packet
        // arrives. It fails exactly like one that is too short: silently,
        // with the receiver awake.
        let c = cfg();
        let s = Sentry::new(&c, 1_000_000, 300_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        let intuitive_us = 2 * s.sleep_us + s.rx_us;
        assert!(
            intuitive_us > s.preamble_max_us,
            "the obvious preamble must be over the bound, or this test proves nothing"
        );
    }

    #[test]
    fn the_receive_window_decides_whether_a_sentry_is_possible() {
        let c = cfg();
        let floor = min_rx_us(&c, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        // A window under the floor admits no preamble at any sleep period,
        // because the sleep moves both bounds together and cancels out.
        let tiny = Sentry::new(&c, 1_000_000, 0, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        assert!(tiny.rx_us < floor);
        assert!(!tiny.feasible());
        assert_eq!(check(&c, &tiny, 8), Err(Refusal::NoPreambleFits));
        let slow = Sentry::new(&c, 60_000_000, 0, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        assert!(!slow.feasible(), "a longer sleep cannot rescue a short window");
        // Widen the window past the floor and it becomes possible.
        let ok = Sentry::new(&c, 1_000_000, floor, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        assert!(ok.rx_us >= floor);
        assert!(ok.feasible());
    }

    #[test]
    fn a_wider_window_buys_margin_and_costs_current() {
        let c = cfg();
        let narrow = Sentry::new(&c, 1_000_000, 200_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        let wide = Sentry::new(&c, 1_000_000, 300_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        // The only lever on the margin two free-running RC clocks need.
        assert!(wide.preamble_window_us() > narrow.preamble_window_us());
        assert!(wide.duty_permille() > narrow.duty_permille());
    }

    #[test]
    fn the_oscillator_restart_lengthens_the_cycle() {
        // The datasheet adds the startup between the sleep and the receive
        // phases rather than taking it out of the window, so it shows up in
        // the cycle and in the deaf gap, not in the listening time.
        let c = cfg();
        let s = Sentry::new(&c, 1_000_000, 300_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        assert_eq!(s.cycle_us(), s.rx_us + s.sleep_us + TCXO_US);
        let none = Sentry::new(&c, 1_000_000, 300_000, DETECT_SYMBOLS, 0, WAKE_LEN);
        assert_eq!(s.rx_us, none.rx_us, "the window is unchanged");
        assert_eq!(s.preamble_min_us - none.preamble_min_us, TCXO_US);
    }

    #[test]
    fn a_longer_sleep_is_cheaper_and_needs_more_preamble() {
        let c = cfg();
        let fast = Sentry::new(&c, 1_000_000, 300_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        let slow = Sentry::new(&c, 4_000_000, 300_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        assert!(slow.duty_permille() < fast.duty_permille());
        assert!(slow.preamble_min_us > fast.preamble_min_us);
        assert!(slow.wake_airtime_us(&c, 8) > fast.wake_airtime_us(&c, 8));
    }

    #[test]
    fn one_carrier_takes_a_wake_preamble_and_a_hopping_plan_refuses_it() {
        let mut c = cfg();
        let s = Sentry::new(&c, 1_000_000, 300_000, DETECT_SYMBOLS, TCXO_US, WAKE_LEN);
        assert_eq!(c.hop_channels, 1);
        assert_eq!(check(&c, &s, 8), Ok(()));
        c.hop_channels = 50;
        assert!(s.wake_airtime_us(&c, 8).unwrap() > FHSS_DWELL_MAX_US);
        assert_eq!(check(&c, &s, 8), Err(Refusal::DwellExceeded));
    }

    #[test]
    fn the_register_values_are_the_periods_in_chip_steps() {
        let s = Sentry::new(&cfg(), 1_000_000, 300_000, DETECT_SYMBOLS, 10_000, WAKE_LEN);
        assert_eq!(s.sleep_steps(), 64_000);
        assert_eq!(s.rx_steps(), duty_steps_from_us(s.rx_us));
        // Whatever the rounding, the encoded pair is never longer than what
        // was asked for - a window that encodes long would eat the sleep.
        assert!(duty_us_from_steps(s.rx_steps()) <= s.rx_us);
        assert!(duty_us_from_steps(s.sleep_steps()) <= s.sleep_us);
    }
}
