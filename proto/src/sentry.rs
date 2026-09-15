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

/// A sentry's two half-periods and the preamble that catches it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sentry {
    /// Receive window, microseconds: the symbols to be counted plus the
    /// measured per-window overhead.
    pub rx_us: u32,
    /// Sleep between windows, microseconds. The knob: everything else here
    /// follows from it.
    pub sleep_us: u32,
    /// Symbols the modem counts before accepting a signal.
    pub detect_symbols: u8,
    /// Preamble a transmission needs to be certain of catching one window.
    pub preamble_symbols: u32,
}

impl Sentry {
    /// Size a sentry for `cfg` from a sleep period and the per-window
    /// overhead a sweep measured.
    ///
    /// The preamble condition is that a transmission must still be in
    /// preamble wherever a window opens, and must stay there long enough to
    /// be counted. Two sleeps plus a window covers that with a whole extra
    /// sleep of margin, which is deliberate: the two timebases are free
    /// running, one of them is the chip's RC, and a preamble that is a
    /// little too short fails as a wake that silently never arrives.
    pub fn new(cfg: &RadioConfig, sleep_us: u32, overhead_us: u32, detect_symbols: u8) -> Self {
        let t_sym_us = cfg.symbol_time_us();
        let rx_us = u32::from(detect_symbols)
            .saturating_mul(t_sym_us)
            .saturating_add(overhead_us);
        let need_us = sleep_us
            .saturating_mul(2)
            .saturating_add(rx_us);
        let preamble_symbols = need_us.div_ceil(t_sym_us.max(1));
        Self {
            rx_us,
            sleep_us,
            detect_symbols,
            preamble_symbols,
        }
    }

    /// One full cycle, microseconds.
    pub fn cycle_us(&self) -> u32 {
        self.rx_us.saturating_add(self.sleep_us)
    }

    /// Share of each cycle the receiver is powered, in parts per thousand.
    ///
    /// The figure the sentry's average current scales with: integer, so the
    /// same number comes out on the board as on the host.
    pub fn duty_permille(&self) -> u32 {
        let cycle = self.cycle_us();
        if cycle == 0 {
            return 0;
        }
        (u64::from(self.rx_us) * 1000 / u64::from(cycle)) as u32
    }

    /// Time on air of a wake transmission carrying `payload_len` bytes,
    /// microseconds - the long preamble and the frame behind it.
    pub fn wake_airtime_us(&self, cfg: &RadioConfig, payload_len: usize) -> u32 {
        cfg.time_on_air_preamble_us(payload_len, self.preamble_symbols)
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
    /// The preamble the sleep period demands does not fit the 16-bit
    /// packet-parameter field. Shorten the sleep.
    PreambleTooLong,
    /// Either half-period overflows the chip's 24-bit timer.
    PeriodTooLong,
    /// The node is hopping and the wake transmission would hold one carrier
    /// past what a frequency hopping system may.
    DwellExceeded,
}

/// Whether `sentry` can be armed on a node running `cfg`, for a wake
/// transmission carrying `payload_len` bytes.
///
/// The dwell check is the one that is a refusal rather than a clamp. A wake
/// preamble is seconds long by construction, which a single carrier may
/// hold and a hopping plan may not, and shortening it to fit would leave a
/// sentry that cannot be woken - a worse outcome than declining to arm one
/// and saying so.
pub fn check(cfg: &RadioConfig, sentry: &Sentry, payload_len: usize) -> Result<(), Refusal> {
    if sentry.preamble_symbols > PREAMBLE_SYMBOLS_MAX {
        return Err(Refusal::PreambleTooLong);
    }
    if duty_steps_from_us(sentry.rx_us) >= DUTY_STEPS_MAX
        || duty_steps_from_us(sentry.sleep_us) >= DUTY_STEPS_MAX
    {
        return Err(Refusal::PeriodTooLong);
    }
    if cfg.hop_channels > 1 && sentry.wake_airtime_us(cfg, payload_len) > FHSS_DWELL_MAX_US {
        return Err(Refusal::DwellExceeded);
    }
    Ok(())
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

    #[test]
    fn a_one_second_sleep_needs_a_two_second_preamble() {
        let c = cfg();
        let t_sym = c.symbol_time_us();
        assert_eq!(t_sym, 8_192, "SF12 at BW500");
        let s = Sentry::new(&c, 1_000_000, 10_000, DETECT_SYMBOLS);
        assert_eq!(s.rx_us, 4 * 8_192 + 10_000);
        // 2 * 1 s + the window, in whole symbols.
        let need = 2 * 1_000_000 + s.rx_us;
        assert_eq!(s.preamble_symbols, need.div_ceil(t_sym));
        // A preamble that covers two sleeps is a little over two seconds.
        assert!(s.preamble_symbols * t_sym >= need);
        assert!((2_000_000..2_200_000).contains(&(s.preamble_symbols * t_sym)));
    }

    #[test]
    fn the_duty_is_the_share_of_the_cycle_spent_listening() {
        let s = Sentry::new(&cfg(), 1_000_000, 10_000, DETECT_SYMBOLS);
        // ~43 ms in ~1043 ms.
        assert_eq!(s.cycle_us(), s.rx_us + 1_000_000);
        assert_eq!(s.duty_permille(), 41);
        // Sleeping twice as long halves it, near enough: the window is
        // unchanged, so the cycle it is a share of is what doubled.
        let slow = Sentry::new(&cfg(), 2_000_000, 10_000, DETECT_SYMBOLS);
        assert_eq!(slow.rx_us, s.rx_us);
        assert_eq!(slow.duty_permille(), 20);
    }

    #[test]
    fn a_longer_sleep_is_cheaper_and_costs_preamble() {
        let c = cfg();
        let fast = Sentry::new(&c, 1_000_000, 10_000, DETECT_SYMBOLS);
        let slow = Sentry::new(&c, 4_000_000, 10_000, DETECT_SYMBOLS);
        assert!(slow.duty_permille() < fast.duty_permille());
        assert!(slow.preamble_symbols > fast.preamble_symbols);
        assert!(slow.wake_airtime_us(&c, 8) > fast.wake_airtime_us(&c, 8));
    }

    #[test]
    fn an_absorbed_startup_makes_every_window_longer() {
        // The whole point of measuring: the same sleep period sized against
        // a measured 10 ms overhead and a measured 25 ms one are different
        // designs, and the second needs more preamble for the same sleep.
        let c = cfg();
        let cheap = Sentry::new(&c, 1_000_000, 10_000, DETECT_SYMBOLS);
        let dear = Sentry::new(&c, 1_000_000, 25_000, DETECT_SYMBOLS);
        assert_eq!(dear.rx_us - cheap.rx_us, 15_000);
        assert!(dear.preamble_symbols > cheap.preamble_symbols);
    }

    #[test]
    fn one_carrier_takes_a_wake_preamble_and_a_hopping_plan_refuses_it() {
        let mut c = cfg();
        let s = Sentry::new(&c, 1_000_000, 10_000, DETECT_SYMBOLS);
        // The default is one channel, so there is no dwell limit to break.
        assert_eq!(c.hop_channels, 1);
        assert_eq!(check(&c, &s, 8), Ok(()));
        // The same sentry on a hopping plan holds one carrier for seconds,
        // which the band does not allow.
        c.hop_channels = 50;
        assert!(s.wake_airtime_us(&c, 8) > FHSS_DWELL_MAX_US);
        assert_eq!(check(&c, &s, 8), Err(Refusal::DwellExceeded));
    }

    #[test]
    fn a_sleep_too_long_to_encode_is_refused_before_it_is_armed() {
        let c = cfg();
        // Past the 24-bit field at 15.625 us a step, which is ~262 s.
        let s = Sentry::new(&c, 300_000_000, 10_000, DETECT_SYMBOLS);
        assert!(matches!(
            check(&c, &s, 8),
            Err(Refusal::PeriodTooLong | Refusal::PreambleTooLong)
        ));
    }

    #[test]
    fn the_register_values_are_the_periods_in_chip_steps() {
        let s = Sentry::new(&cfg(), 1_000_000, 10_000, DETECT_SYMBOLS);
        assert_eq!(s.sleep_steps(), 64_000);
        assert_eq!(s.rx_steps(), duty_steps_from_us(s.rx_us));
        // Whatever the rounding, the encoded pair is never longer than what
        // was asked for - a window that encodes long would eat the sleep.
        assert!(duty_us_from_steps(s.rx_steps()) <= s.rx_us);
        assert!(duty_us_from_steps(s.sleep_steps()) <= s.sleep_us);
    }
}
