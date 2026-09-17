//! The bench probe for a duty-cycled receiver.
//!
//! One question decides whether a radio can usefully listen while the chip
//! that owns it is asleep: what a receive window costs beyond the symbols
//! it is meant to hear. The part restarts its oscillator on every window,
//! and whether that time is added to the commanded window or taken out of
//! it changes how long every window in such a design has to be. Reading it
//! off a datasheet is guessing; this measures it.
//!
//! The instrument is the radio. With a second board keying a continuous
//! preamble, every receive window that opens should detect - so DIO1
//! becomes a readout of the receiver's own schedule, and neither a scope
//! nor a current probe is needed for either half of the answer:
//!
//! - **Cadence.** Arm once and watch. The interval between detections is
//!   the real cycle, and comparing it against the commanded one says which
//!   way the oscillator restart is charged. Whether detections keep coming
//!   at all says whether the chip stays in the cycle after a reception,
//!   which decides whether a sleeping board has to re-arm on every wake.
//! - **Sweep.** Walk the window down until detection stops being reliable.
//!   The shortest window that still catches every cycle, less the symbols
//!   it was listening for, is the overhead - the number the design needs.
//!
//! The analysis is [`midair_proto::sentry`], host-tested, because picking
//! the wrong step out of a sweep gives a plausible number that is wrong,
//! and a wrong number here sizes every preamble in the system.
//!
//! Both halves need a source. See [`source`], and read what it says about
//! keying the PA before running it.

use embassy_time::{Duration, Instant, Timer};
use esp_println::println;
use midair_proto::sentry::{
    drift_us, min_rx_for_margin, min_rx_us, rc_rate, sweep_floor, SweepStep, DETECT_SYMBOLS,
};
use midair_proto::supervise::{Phase, Task};

use crate::radio::Sx1262Driver;
use crate::sx1262::irq;
use crate::watchdog;

/// The sentry both halves of the bench agree on.
///
/// One function rather than two constants, because the probe and the source
/// have to derive the same preamble from the same numbers - the receiver's
/// window and the transmitter's preamble are one choice, and two boards that
/// disagree about it produce a silence neither can explain.
fn geometry(radio: &Sx1262Driver<'_>) -> midair_proto::sentry::Sentry {
    let mut cfg = midair_proto::radiocfg::RadioConfig::default();
    cfg.spreading_factor = sf_for(radio.symbol_time_us());
    // A generous window on purpose. The measured timebase needs almost
    // none of it, and the point of a first run is to find out whether the
    // mechanism works at all rather than how cheaply it can be made to.
    let overhead = SENTRY_RX_US.saturating_sub(u32::from(SYMB_TIMEOUT) * radio.symbol_time_us());
    midair_proto::sentry::Sentry::new(&cfg, SENTRY_SLEEP_US, overhead, SYMB_TIMEOUT, TCXO_US, WAKE_PAYLOAD.len())
}

/// Symbols the modem is given to validate a signal, for both the chip and
/// the arithmetic that sizes the window around it.
///
/// One constant, not two: the window has to contain exactly the symbols the
/// modem validates on, so a receiver told to validate on eight and a
/// preamble sized for four disagree about the only number they share.
///
/// **It must not be zero.** Zero disables the check, and measured on
/// hardware that takes a duty cycle from waking on most frames to waking on
/// none - because a non-zero value is also what makes the chip hold the
/// window open "for the full duration of the packet" once it has validated.
/// With it off, the window simply ends at its own length and the chip
/// sleeps through the rest of a frame it had already heard the start of.
const SYMB_TIMEOUT: u8 = 8;

/// Whether the other board is keying a continuous carrier rather than
/// sending frames, i.e. whether it was built `iso-sentry-carrier`.
///
/// The two measurements want opposite signals and a run can only have one.
const PEER_IS_CARRIER: bool = false;

/// Nominal sleep of the sentry under test, microseconds. What the chip is
/// asked for is this corrected for its own timer.
const SENTRY_SLEEP_US: u32 = 1_000_000;

/// Receive window of the sentry under test, microseconds.
///
/// Two hundred rather than the hundred the first runs used. The chip's
/// restarted timer has to contain the whole packet and not just its header,
/// so at a hundred no preamble fits at all - measured as thirty-nine
/// detected preambles producing two headers, the two being those detected
/// late enough in the preamble that the rest of the frame still fit.
const SENTRY_RX_US: u32 = 200_000;

/// Sleep half of the cycle under test, microseconds.
///
/// Long enough that a window is a small part of the cycle, so a detection
/// is unambiguously one window's, and short enough that a sweep step is
/// seconds rather than minutes.
const SLEEP_US: u32 = 1_000_000;



/// Trials per sweep step. Forty is enough that a window which passes every
/// one is not passing by luck, and few enough that the whole ladder runs in
/// a couple of minutes.
const SWEEP_TRIALS: u32 = 15;

/// Window overheads to try, microseconds, longest first.
///
/// The ladder is in overhead rather than in absolute window length, so the
/// same sweep runs at any spreading factor: each step is the symbols the
/// modem has to count plus this much headroom, and the shortest step that
/// still detects every time *is* the overhead. Descending, because
/// [`sweep_floor`] only accepts a step whose longer neighbors all passed.
const OVERHEAD_LADDER_US: [u32; 12] = [
    400_000, 300_000, 250_000, 200_000, 150_000, 120_000, 100_000, 80_000, 60_000, 40_000,
    20_000, 0,
];

/// How long a single trial waits for a detection before calling it a miss,
/// as a multiple of the commanded cycle. Three cycles is generous for
/// something that should happen on the first.
const TRIAL_CYCLES: u32 = 2;

/// Longest the source will hold the PA on before standing down, seconds.
///
/// Keying with no end to it was a convenience, and it is the wrong default
/// for a thing that lives on a bench: a board left plugged in stays keyed
/// until somebody remembers it, which is exactly the state nobody is
/// watching. Twenty minutes covers a full probe run with room, and a run
/// that needs longer can be restarted deliberately.
const SOURCE_MAX_KEYED_S: u64 = 1_200;

/// The board's configured oscillator startup, microseconds. The datasheet
/// adds this between the sleep and receive phases of a duty cycle, so it
/// widens the deaf gap a preamble has to span.
const TCXO_US: u32 = 10_000;

/// Preamble lengths the source walks through, in symbols.
///
/// The one number in this design that has always been computed rather than
/// measured, from a model of the chip that has been wrong more than once.
/// So it is swept: wide enough to bracket whatever the real window is, and
/// each frame says which length it was sent with so a wake reports its own
/// cause.
const PREAMBLE_SWEEP: [u16; 14] = [
    60, 80, 100, 120, 130, 140, 150, 160, 180, 200, 230, 260, 290, 320,
];

/// Frames sent at each preamble length.
const FRAMES_PER_PREAMBLE: u32 = 4;

/// Tag byte in front of the preamble length a wake frame carries.
const WAKE_TAG: u8 = 0x57;

/// What a wake frame carries for the mechanism test. The payload is not the
/// point - a real packet is, because only a completed reception ends the
/// chip's sniff loop.
const WAKE_PAYLOAD: &[u8] = b"wake";

/// Seconds the source stays off the air before its first frame.
///
/// The probe opens by timing its own sleep oscillator, and that measurement
/// times a receive window against the host clock - so a window that hears a
/// preamble is no longer measuring an oscillator. Filtering the spoiled
/// trials is not enough, because a preamble can perturb the receive timer
/// without leaving an interrupt to filter on: measured beside live traffic
/// the same chip reads +10686 ppm and -4462 ppm in different runs, both
/// repeatably.
///
/// So the channel is left quiet for long enough that the measurement
/// finishes first. It costs one silent period per bench run and removes the
/// whole failure, which no amount of filtering did.
const QUIET_FIRST_S: u64 = 45;

/// Most transmit power the source will key at, dBm.
///
/// Two boards on a bench need milliwatts, and this is what makes keeping
/// the PA on indefinitely a bench convenience rather than a way to cook a
/// module. 0 dBm is 1 mW, which is still an enormous signal at that range.
const SOURCE_MAX_DBM: i8 = 10;

/// Poll period on DIO1, milliseconds.
///
/// The pin latches high until the interrupt is cleared, so this sets the
/// resolution of a timestamp and not whether a detection is seen at all.
/// Ten milliseconds against a cycle of about a second is one percent, which
/// is inside the chip's own RC timebase and so costs nothing real.
///
/// It is not set by the resolution wanted but by what the rest of the board
/// can afford. Every wait is a timer-queue operation under a critical
/// section this chip shares between its two cores, so a poll of one
/// millisecond puts a thousand of them a second against everything else
/// running - and the task that loses is the watchdog monitor on the other
/// core, which stops feeding and resets the board. Poll no faster than the
/// measurement needs.
const POLL_MS: u64 = 10;

/// Run the probe. Does not return.
///
/// Expects the radio already initialized from the running config - the
/// board has to be in a mode that brings it up - and a second board keying
/// a continuous preamble on the same settings throughout.
pub async fn probe(radio: &mut Sx1262Driver<'_>) -> ! {
    let t_sym = radio.symbol_time_us();
    let listening = u32::from(DETECT_SYMBOLS) * t_sym;
    println!("sentry probe: {} us/symbol, {} symbols to detect ({} us listening)",
        t_sym, DETECT_SYMBOLS, listening);
    println!("sentry probe: source must be keying a continuous preamble on the same settings");

    // The band first, because it decides whether the rest is worth
    // reading: a window that is usually already busy with a false
    // detection cannot be waiting for a frame.
    noise_survey(radio).await;

    // The chip's own timebase next: it needs no source, and what it
    // measures is what decides how wide every window below has to be.
    let rc = rc_timebase(radio).await;

    // The control first, while the chip is still in a state that answers.
    // A duty cycle leaves it asleep, so a control run afterwards measures
    // the ordering rather than the link - which is how the first version of
    // this reported a dead source that was transmitting perfectly well.
    if !hearing(radio).await {
        println!("sentry probe: STOPPING - nothing to measure against");
        loop {
            watchdog::beat(Task::Loop, Phase::Receive);
            Timer::after(Duration::from_millis(500)).await;
        }
    }

    // What the peer is transmitting decides what can be measured, and the
    // two want opposite signals: the sweep asks whether a window *detects*,
    // so it wants a carrier that is always there, and the wake test asks
    // whether one *receives*, so it wants frames. A run cannot do both, and
    // trying costs ten minutes proving the wrong one.
    if PEER_IS_CARRIER {
        let steps = sweep(radio, listening).await;
        println!("sentry probe:");
        match sweep_floor(&steps, t_sym, DETECT_SYMBOLS) {
            Some(f) => {
                println!(
                    "sentry probe: floor {} us, so {} us of overhead over {} symbols",
                    f.rx_us, f.overhead_us, DETECT_SYMBOLS
                );
                println!(
                    "sentry probe: a window is worth its length less {} us - size every one that way",
                    f.overhead_us
                );
            }
            // Either nothing detected at all, or a longer window failed
            // while a shorter one passed. Both are runs to repeat rather
            // than numbers to design against, and the table says which.
            None => println!(
                "sentry probe: NO FLOOR - nothing detected, or the sweep is not clean from the top"
            ),
        }
    } else {
        wake_test(radio, rc.as_ref()).await;
    }

    // Nothing else on this build has anything to do, and the loop is what
    // keeps the board's watchdog fed while the console output is read off.
    // Says so on a cadence for the reason `park` does: a finished run and a
    // wedged one look the same to a console attached after the fact, and
    // this run's whole output is the lines above.
    let mut said = Instant::now();
    loop {
        watchdog::beat(Task::Loop, Phase::Receive);
        if Instant::now() - said > Duration::from_secs(15) {
            println!("sentry probe: done, results above");
            said = Instant::now();
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// Arm a sentry the way the design means it to be armed, and wait to be
/// woken by a real frame.
///
/// This is the question the whole plan rests on, and the earlier version of
/// this probe could not ask it: the sniff loop leaves only on `RX_DONE`, so
/// a source keying a preamble that never becomes a packet can never wake
/// it, however long it keys for.
///
/// A reception ends the cycle and leaves the chip in standby, so every wake
/// is followed by a re-arm. That is not a workaround - it is what a sleeping
/// board will have to do on the far side of every wake.
async fn wake_test(radio: &mut Sx1262Driver<'_>, rc: Option<&midair_proto::sentry::RcRate>) {
    const WAKE_WAIT_S: u64 = 150;
    let g = geometry(radio);
    let Some(preamble) = g.preamble_symbols(&cfg_of(radio)) else {
        println!("sentry probe: no preamble fits this geometry - nothing to test");
        return;
    };
    // Corrected, so the sleep the chip actually takes is the one the
    // preamble was sized against.
    let (rx_cmd, sleep_cmd) = match rc {
        Some(r) => (r.correct_us(g.rx_us), r.correct_us(g.sleep_us)),
        None => (g.rx_us, g.sleep_us),
    };
    println!(
        "sentry probe: wake test, rx {} us sleep {} us (commanded {} / {}), symb timeout {}",
        g.rx_us, g.sleep_us, rx_cmd, sleep_cmd, SYMB_TIMEOUT
    );
    println!(
        "sentry probe: source must send frames with a {}-symbol preamble ({} us, window {} to {} us)",
        preamble,
        preamble * radio.symbol_time_us(),
        g.preamble_min_us,
        g.preamble_max_us
    );

    // Every arm is verified and every arm's outcome is recorded, because
    // the question is which of the two is failing: frames that are not
    // heard, or an arm after a reception that never took. A sentry whose
    // re-arm is eaten wakes once and then never again, which for a sleeping
    // board is a doorbell that works one time.
    let until = Instant::now() + Duration::from_secs(WAKE_WAIT_S);
    let mut woken = 0u32;
    let mut arms = 0u32;
    let mut arms_verified = 0u32;
    let mut preambles = 0u32;
    let mut headers = 0u32;
    let mut crc_errs = 0u32;
    // Which preamble lengths actually woke it, against the lengths sent.
    let mut hits = [0u32; PREAMBLE_SWEEP.len()];
    // Set by the first `arm!` before anything reads it.
    let mut armed_at;
    let mut said = Instant::now();

    // A closure would need the radio mutably twice; a small helper keeps it
    // readable and is used for the first arm and every re-arm alike, so the
    // two cannot drift apart.
    macro_rules! arm {
        () => {{
            // The chip is most likely in a retained sleep, where it accepts
            // nothing until an NSS edge wakes it - so the arm that follows
            // would otherwise be spent doing that and be lost.
            radio.wake_from_retained_sleep().await;
            // Every stage of a reception, not just its end. A wake that
            // does not happen is one of three different failures - a window
            // that never caught the preamble, a preamble that never became
            // a header, or a header whose packet failed - and they want
            // different fixes. All three fire while the chip is awake in a
            // window, so reading them cannot disturb the cycle.
            radio.arm_duty_cycle(
                rx_cmd,
                sleep_cmd,
                SYMB_TIMEOUT,
                irq::RX_DONE | irq::PREAMBLE_DETECTED | irq::HEADER_VALID | irq::CRC_ERR,
            );
            arms += 1;
            armed_at = Instant::now();
            // Inside the first receive window, so this read cannot disturb
            // the cycle it is checking.
            let (mode, err) = radio.health();
            if mode == "rx" {
                arms_verified += 1;
            } else {
                println!(
                    "sentry probe: ARM {} DID NOT TAKE - radio {} err 0x{:04X}",
                    arms, mode, err
                );
            }
        }};
    }

    arm!();
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        if radio.irq_pending() {
            let status = radio.take_irq();
            if status & irq::PREAMBLE_DETECTED != 0 {
                preambles += 1;
            }
            if status & irq::HEADER_VALID != 0 {
                headers += 1;
            }
            if status & irq::CRC_ERR != 0 {
                crc_errs += 1;
            }
            if status & irq::RX_DONE != 0 {
                woken += 1;
                // The frame says what preamble it was sent with, so a wake
                // reports its own cause. Read before the re-arm, which
                // rewrites the packet parameters.
                let mut buf = [0u8; 8];
                let n = radio.read_payload(&mut buf);
                let syms = if n >= 3 && buf[0] == WAKE_TAG {
                    u16::from(buf[1]) | (u16::from(buf[2]) << 8)
                } else {
                    0
                };
                if let Some(slot) = PREAMBLE_SWEEP.iter().position(|p| *p == syms) {
                    hits[slot] += 1;
                }
                println!(
                    "sentry probe: woken by a {} symbol preamble after {} ms",
                    syms,
                    (Instant::now() - armed_at).as_millis()
                );
                arm!();
            }
        }
        if Instant::now() - said > Duration::from_secs(20) {
            println!(
                "sentry probe: {} s left, {} wakes, {} arms ({} verified)",
                (until - Instant::now()).as_secs(),
                woken,
                arms,
                arms_verified
            );
            said = Instant::now();
        }
        Timer::after(Duration::from_millis(POLL_MS)).await;
    }

    radio.wake_from_retained_sleep().await;
    println!(
        "sentry probe: {} wakes from {} arms, {} of those arms verified rx",
        woken, arms, arms_verified
    );
    // Where the frames that did not wake it got to. Each step is a
    // different failure with a different fix, and the counts say which.
    println!(
        "sentry probe: {} preambles -> {} headers -> {} wakes ({} crc errors)",
        preambles, headers, woken, crc_errs
    );
    // The measurement this run exists for: the feasible preamble window as
    // the hardware reports it, rather than as the geometry predicts it.
    println!("sentry probe: wakes by preamble length -");
    let g = geometry(radio);
    for (i, p) in PREAMBLE_SWEEP.iter().enumerate() {
        let us = u32::from(*p) * radio.symbol_time_us();
        let predicted = us >= g.preamble_min_us && us <= g.preamble_max_us;
        println!(
            "sentry probe: {:>4} symbols ({:>7} us) {:>3} wakes  {}",
            p,
            us,
            hits[i],
            if predicted { "<- model says this should work" } else { "" }
        );
    }
    println!(
        "sentry probe: model predicts {} to {} us",
        g.preamble_min_us, g.preamble_max_us
    );

    if preambles == 0 {
        println!("sentry probe: STAGE windows are not catching the preamble - geometry or margin");
    } else if headers < preambles / 2 {
        println!("sentry probe: STAGE preambles caught but not becoming headers - the window closes too early");
    } else if woken < headers / 2 {
        println!("sentry probe: STAGE headers decoded but packets not completing - length or crc");
    } else {
        println!("sentry probe: STAGE receptions complete once started - the misses are earlier");
    }
    if arms_verified < arms {
        println!("sentry probe: VERDICT re-arms are being eaten - {} of {} did not take", arms - arms_verified, arms);
    } else if woken == 0 {
        println!("sentry probe: VERDICT every arm took and nothing was heard - not the re-arm");
    } else {
        println!("sentry probe: VERDICT every arm took; the misses are frames not heard, not arms lost");
    }
}

/// The running modulation as a config, for the shared arithmetic.
fn cfg_of(radio: &Sx1262Driver<'_>) -> midair_proto::radiocfg::RadioConfig {
    let mut cfg = midair_proto::radiocfg::RadioConfig::default();
    cfg.spreading_factor = sf_for(radio.symbol_time_us());
    cfg
}

/// Count false preamble detections across the band and both gain settings.
///
/// A duty-cycled receiver is only listening for a fraction of each cycle,
/// so it can afford that window to be spent on the signal it is waiting
/// for and not much else. A false detection is not free: the chip restarts
/// its timer and holds the receiver hunting a header that will never
/// arrive, so a channel busy enough will consume most windows before a real
/// frame lands.
///
/// Nothing may be transmitting while this runs, or it measures the source.
/// What it produces is the one number the wake design needs from the
/// environment - detections a second - for each carrier and gain, so a
/// quiet corner of the band can be picked rather than assumed.
async fn noise_survey(radio: &mut Sx1262Driver<'_>) {
    /// Seconds of continuous receive per condition.
    const DWELL_S: u64 = 6;
    /// Carriers to try, Hz. The 902-928 MHz band, sampled across.
    const CARRIERS: [u32; 7] = [
        903_000_000,
        907_000_000,
        911_000_000,
        915_000_000,
        919_000_000,
        923_000_000,
        927_000_000,
    ];

    let home = radio.carrier_hz();
    println!("sentry probe: noise survey, {} s a condition, nothing may be transmitting", DWELL_S);
    println!("sentry probe:      MHz  boost   detections  per second");
    let mut best = (u32::MAX, home, true);
    for boost in [true, false] {
        for hz in CARRIERS {
            radio.set_rx_boost(boost);
            radio.tune(hz);
            radio.arm_continuous_rx(irq::PREAMBLE_DETECTED);
            let until = Instant::now() + Duration::from_secs(DWELL_S);
            let mut seen = 0u32;
            while Instant::now() < until {
                watchdog::beat(Task::Loop, Phase::Receive);
                if radio.irq_pending() && radio.take_irq() & irq::PREAMBLE_DETECTED != 0 {
                    seen += 1;
                }
                Timer::after(Duration::from_millis(POLL_MS)).await;
            }
            // Tenths, so the table stays integer and still separates a
            // quiet carrier from a merely quieter one.
            let per_s_tenths = u64::from(seen) * 10 / DWELL_S;
            println!(
                "sentry probe: {:>8}  {:>5}   {:>10}  {}.{}",
                hz / 1_000_000,
                if boost { "on" } else { "off" },
                seen,
                per_s_tenths / 10,
                per_s_tenths % 10
            );
            if seen < best.0 {
                best = (seen, hz, boost);
            }
        }
    }
    println!(
        "sentry probe: quietest {} MHz with boost {} - {} in {} s",
        best.1 / 1_000_000,
        if best.2 { "on" } else { "off" },
        best.0,
        DWELL_S
    );
    // Put the radio back where the config wants it; the phases below are
    // about the link, not the band.
    radio.set_rx_boost(true);
    radio.tune(home);
}

/// Time the chip's own sleep timer against the host's crystal.
///
/// The receive timeout is counted in the same 15.625 us steps off the same
/// RC64k that times a duty cycle's sleep phase, so a timeout commanded and
/// then measured says what that oscillator is really running at. Nothing
/// has to be on the air for this, which is why it goes first.
///
/// The number matters because the preamble window has to absorb this error
/// over a whole sleep. The window is tens of milliseconds; one percent of a
/// one-second sleep is ten. So this is what sets the receive window, and
/// the receive window is what the sentry's current is proportional to.
async fn rc_timebase(radio: &mut Sx1262Driver<'_>) -> Option<midair_proto::sentry::RcRate> {
    /// Commanded timeout per trial, ms. Long enough that the fixed costs of
    /// arming and of the poll period are a small part of it.
    const RC_TIMEOUT_MS: u32 = 2_000;
    const RC_TRIALS: usize = 8;

    println!("sentry probe: timing the chip's RC64k, {} trials of {} ms", RC_TRIALS, RC_TIMEOUT_MS);
    let commanded_us = RC_TIMEOUT_MS * 1_000;
    let mut measured: heapless::Vec<u32, RC_TRIALS> = heapless::Vec::new();
    let mut spoiled = 0u32;
    for _ in 0..RC_TRIALS {
        // Everything that could end a receive window is latched, not just
        // the timeout, so a trial that was cut short by traffic can be told
        // from one that ran its length. Measuring only the timeout hides
        // the contaminated trials instead of discarding them, and a wrong
        // rate here mis-sizes every sleep that is corrected by it.
        radio.arm_rx_timeout(
            RC_TIMEOUT_MS,
            irq::TIMEOUT | irq::RX_DONE | irq::PREAMBLE_DETECTED | irq::HEADER_VALID,
        );
        let started = Instant::now();
        // Generous: a timer that runs very long must be measured, not cut
        // off at the value being checked.
        let give_up = started + Duration::from_millis(u64::from(RC_TIMEOUT_MS) * 3);
        let mut got = None;
        while Instant::now() < give_up {
            watchdog::beat(Task::Loop, Phase::Receive);
            if radio.irq_pending() {
                let at = Instant::now();
                let status = radio.take_irq();
                // A window that saw a signal was not timing the oscillator,
                // it was receiving. Discard it rather than average it in.
                if status & (irq::RX_DONE | irq::PREAMBLE_DETECTED | irq::HEADER_VALID) != 0 {
                    spoiled += 1;
                    break;
                }
                if status & irq::TIMEOUT != 0 {
                    got = Some((at - started).as_micros() as u32);
                    break;
                }
            }
            Timer::after(Duration::from_millis(POLL_MS)).await;
        }
        match got {
            Some(us) => {
                let _ = measured.push(us);
            }
            None => println!("sentry probe: a timeout never fired - the chip is not counting"),
        }
    }

    let Some(r) = rc_rate(commanded_us, &measured) else {
        println!("sentry probe: RC64k NOT MEASURED - no trial completed");
        return None;
    };
    println!(
        "sentry probe: RC64k {} clean trials ({} spoiled by traffic), mean {} ppm, min {} ppm, max {} ppm",
        r.trials, spoiled, r.mean_ppm, r.min_ppm, r.max_ppm
    );
    if r.trials < 3 {
        println!("sentry probe: RC64k UNRELIABLE - too few clean trials to correct with");
        return None;
    }
    // The offset and the spread are different things and only one of them
    // costs anything. An offset is the same every cycle and divides out of
    // the period commanded; the spread is what a window has to be wide
    // enough to absorb.
    let sleep = SLEEP_US;
    let spread = r.spread_ppm();
    println!(
        "sentry probe: offset {} ppm (correctable: command {} us for a {} us sleep)",
        r.mean_ppm,
        r.correct_us(sleep),
        sleep
    );
    println!(
        "sentry probe: spread {} ppm is {} us over a {} ms sleep - this is what needs margin",
        spread,
        drift_us(sleep, spread),
        sleep / 1000
    );
    let t_sym = radio.symbol_time_us();
    let floor = min_rx_us_for(t_sym);
    for (label, ppm) in [("corrected", spread), ("uncorrected", r.worst_abs_ppm())] {
        let needed = min_rx_for_margin_us(t_sym, sleep, ppm);
        println!(
            "sentry probe: {} - window >= {} us ({} over the {} us floor), {} permille duty",
            label,
            needed,
            needed.saturating_sub(floor),
            floor,
            (u64::from(needed) * 1000 / u64::from(needed + sleep + TCXO_US)) as u32
        );
    }
    // Said plainly, because it is the finding: these trials ran seconds
    // apart on a board at one temperature, and a stored board does not sit
    // at one temperature for a month.
    println!("sentry probe: NOTE the spread above is short-term only - temperature drift is unmeasured");
    Some(r)
}

/// The fixed floor, for the running modulation.
fn min_rx_us_for(t_sym_us: u32) -> u32 {
    let mut cfg = midair_proto::radiocfg::RadioConfig::default();
    cfg.spreading_factor = sf_for(t_sym_us);
    min_rx_us(&cfg, SYMB_TIMEOUT, TCXO_US, WAKE_PAYLOAD.len())
}

/// The window a measured drift demands, for the running modulation.
fn min_rx_for_margin_us(t_sym_us: u32, sleep_us: u32, ppm: u32) -> u32 {
    let mut cfg = midair_proto::radiocfg::RadioConfig::default();
    cfg.spreading_factor = sf_for(t_sym_us);
    min_rx_for_margin(&cfg, SYMB_TIMEOUT, TCXO_US, sleep_us, ppm, WAKE_PAYLOAD.len())
}

/// Recover the spreading factor from the symbol time at the default 500 kHz
/// bandwidth, so the arithmetic above is done on what the radio is running
/// rather than on the config default.
fn sf_for(t_sym_us: u32) -> u8 {
    // t_sym = 2^sf / bw, and at 500 kHz that is 2^sf * 2 us.
    let mut sf = 5u8;
    while sf < 12 && (1u32 << sf) * 2 < t_sym_us {
        sf += 1;
    }
    sf
}

/// Listen continuously for a few seconds and say whether anything is there.
///
/// The control for the whole run: it uses the same radio, the same
/// interrupt and the same settings as every measurement below, and differs
/// only in never sleeping. So a failure here is the link - a source that is
/// off, on other settings, or out of range - and a failure below it with
/// this passing is the receive window, which is the thing being measured.
async fn hearing(radio: &mut Sx1262Driver<'_>) -> bool {
    const LISTEN_S: u64 = 10;
    println!("sentry probe: listening continuously for {} s as a control", LISTEN_S);
    radio.arm_continuous_rx(irq::PREAMBLE_DETECTED);
    let until = Instant::now() + Duration::from_secs(LISTEN_S);
    let mut seen = 0u32;
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        if radio.irq_pending() {
            let status = radio.take_irq();
            if status & irq::PREAMBLE_DETECTED != 0 {
                seen += 1;
            }
        }
        Timer::after(Duration::from_millis(POLL_MS)).await;
    }
    let (mode, err) = radio.health();
    println!(
        "sentry probe: control saw {} preamble detections, radio {} err 0x{:04X}",
        seen, mode, err
    );
    if seen == 0 {
        println!("sentry probe: CONTROL FAILED - the source is not reachable on these settings");
        println!("sentry probe: check it is still keyed, and that both boards share frequency and modulation");
    }
    seen > 0
}

/// Walk the window down and count what still detects.
async fn sweep(radio: &mut Sx1262Driver<'_>, listening: u32) -> heapless::Vec<SweepStep, 16> {
    println!("sentry probe: sweep, {} trials a step", SWEEP_TRIALS);
    println!("sentry probe:  overhead_us     rx_us  detects");
    let mut steps: heapless::Vec<SweepStep, 16> = heapless::Vec::new();
    for overhead in OVERHEAD_LADDER_US {
        let rx_us = listening + overhead;
        let deadline =
            Duration::from_micros(u64::from(rx_us + SLEEP_US) * u64::from(TRIAL_CYCLES));
        let mut detects = 0;
        for _ in 0..SWEEP_TRIALS {
            // Re-armed per trial, so each one is an independent question:
            // does a window of this length catch a signal that is already
            // there?
            //
            // The wake first is not optional. Every trial but the first
            // arms a chip left mid-sleep by the one before, where it holds
            // BUSY high and accepts nothing until an NSS edge wakes it - so
            // the arm is spent waking it and the window never opens. Without
            // this the sweep reads 0 of 15 at every length including 432 ms,
            // on a board whose continuous receive hears the same carrier
            // perfectly well, and that reads as a receiver that cannot
            // detect rather than an arm that never landed.
            watchdog::beat(Task::Loop, Phase::Receive);
            radio.wake_from_retained_sleep().await;
            radio.arm_duty_cycle(rx_us, SLEEP_US, DETECT_SYMBOLS, irq::PREAMBLE_DETECTED);
            if wait_for_detect(radio, deadline).await.is_some() {
                detects += 1;
            }
        }
        println!(
            "sentry probe: {:>11}  {:>8}  {:>3}/{}",
            overhead, rx_us, detects, SWEEP_TRIALS
        );
        let _ = steps.push(SweepStep {
            rx_us,
            cycles: SWEEP_TRIALS,
            detects,
        });
    }
    steps
}

/// Wait for DIO1, clear what raised it, and return when it happened.
/// `None` if `deadline` passed with the pin quiet.
async fn wait_for_detect(radio: &mut Sx1262Driver<'_>, deadline: Duration) -> Option<Instant> {
    let give_up = Instant::now() + deadline;
    // Progress, so a run that stops says where it stopped. Without it a
    // reset mid-sweep is indistinguishable from one mid-wait, and the sweep
    // rows only print when a whole step is done.
    let mut said = Instant::now();
    loop {
        watchdog::beat(Task::Loop, Phase::Receive);
        if Instant::now() - said > Duration::from_secs(20) {
            println!("sentry probe: still waiting, {} s into a wait", (Instant::now() - give_up + deadline).as_secs());
            said = Instant::now();
        }
        if radio.irq_pending() {
            let at = Instant::now();
            let status = radio.take_irq();
            // A window that opened, counted its symbols and found nothing is
            // a timeout, not a detection - and on a sweep step that is too
            // short it is the expected outcome rather than an error.
            if status & irq::PREAMBLE_DETECTED != 0 {
                return Some(at);
            }
        }
        if Instant::now() >= give_up {
            return None;
        }
        Timer::after(Duration::from_millis(POLL_MS)).await;
    }
}

/// Key a continuous preamble, as the signal the probe measures against.
/// Does not return.
///
/// **This keys the PA and leaves it keyed**, for as long as the board is
/// powered. Two things make that acceptable rather than reckless, and both
/// are checked here rather than left to whoever is at the bench:
///
/// - **The power has to be low.** At the top of the range the PA is 127 mA
///   in a module that normally sees a third of a second at a time. At
///   [`SOURCE_MAX_DBM`] it is a fraction of that, and two boards a bench
///   apart need nothing more. A board configured higher is refused.
/// - **The antenna switch has to be right.** DIO2 switches it and DIO3
///   supplies it, so this only ever follows the ordinary initialization
///   that sets both, and the latched device errors are read before keying -
///   `XOSC_START` means the oscillator did not start, which on this module
///   means the switch is unpowered, and keying into an isolated port
///   destroys the part.
///
/// Continuous rather than burst because of what is downstream: a sweep
/// trial that lands while the source is quiet fails, and a source with an
/// off-period would put that failure into every step of the sweep at the
/// rate of its own duty cycle. The measurement would then be of the
/// transmitter, not the receiver.
pub async fn source(radio: &mut Sx1262Driver<'_>) -> ! {
    let dbm = radio.power_dbm();
    if dbm > SOURCE_MAX_DBM {
        println!(
            "sentry source: REFUSING to send at {} dBm - {} dBm or less",
            dbm, SOURCE_MAX_DBM
        );
        park().await
    }
    let err = radio.device_errors();
    if err != 0 {
        println!("sentry source: REFUSING to transmit, device errors 0x{:04X}", err);
        park().await
    }

    println!(
        "sentry source: quiet for {} s so the receiver can time its own oscillator first",
        QUIET_FIRST_S
    );
    hold(QUIET_FIRST_S).await;
    println!(
        "sentry source: sweeping {} preamble lengths, {} frames each at {} dBm",
        PREAMBLE_SWEEP.len(),
        FRAMES_PER_PREAMBLE,
        dbm
    );

    let until = Instant::now() + Duration::from_secs(SOURCE_MAX_KEYED_S);
    loop {
        for preamble in PREAMBLE_SWEEP {
            if Instant::now() >= until {
                println!("sentry source: budget spent, standing down");
                radio.standby();
                park().await
            }
            // The frame says what it was sent with, so a receiver woken by
            // it can report which length worked rather than leaving the two
            // ends to be matched up by wall clock.
            let payload = [WAKE_TAG, preamble as u8, (preamble >> 8) as u8];
            let mut ok = 0u32;
            for _ in 0..FRAMES_PER_PREAMBLE {
                watchdog::beat(Task::Loop, Phase::TxSend);
                if radio.send_wake(&payload, preamble).await.is_ok() {
                    ok += 1;
                }
                hold(1).await;
            }
            let (mode, err) = radio.health();
            if err != 0 {
                println!("sentry source: STANDING DOWN, radio latched 0x{:04X}", err);
                radio.standby();
                park().await
            }
            println!(
                "sentry source: {} symbols, {}/{} sent, radio {}, {} s left",
                preamble,
                ok,
                FRAMES_PER_PREAMBLE,
                mode,
                (until - Instant::now()).as_secs()
            );
        }
    }
}

/// Key a continuous preamble, as the signal the window sweep measures
/// against. Does not return.
///
/// The sweep asks whether a receive window *detects*, not whether it
/// receives, so a signal that is always present and never becomes a packet
/// is exactly right for it - every window that opens should detect, and the
/// ones that do not are the measurement.
///
/// The same guards as the frame source: the power is checked because this
/// holds the PA on, the device errors are read because DIO3 supplies the
/// antenna switch on this module, and the keying is bounded so a board left
/// plugged in does not transmit until somebody remembers it.
#[cfg(feature = "iso-sentry-carrier")]
pub async fn carrier(radio: &mut Sx1262Driver<'_>) -> ! {
    let dbm = radio.power_dbm();
    if dbm > SOURCE_MAX_DBM {
        println!(
            "sentry carrier: REFUSING to key at {} dBm - {} dBm or less",
            dbm, SOURCE_MAX_DBM
        );
        park().await
    }
    println!(
        "sentry carrier: quiet for {} s, then keyed at {} dBm for {} s",
        QUIET_FIRST_S, dbm, SOURCE_MAX_KEYED_S
    );
    hold(QUIET_FIRST_S).await;
    let err = radio.key_infinite_preamble();
    if err != 0 {
        println!("sentry carrier: REFUSING to key, device errors 0x{:04X}", err);
        radio.standby();
        park().await
    }
    let until = Instant::now() + Duration::from_secs(SOURCE_MAX_KEYED_S);
    let mut said = Instant::now();
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        if Instant::now() - said > Duration::from_secs(30) {
            let (mode, err) = radio.health();
            if err != 0 {
                println!("sentry carrier: STANDING DOWN, radio latched 0x{:04X}", err);
                radio.standby();
                park().await
            }
            println!(
                "sentry carrier: keyed at {} dBm, radio {}, {} s left",
                dbm,
                mode,
                (until - Instant::now()).as_secs()
            );
            said = Instant::now();
        }
        Timer::after(Duration::from_millis(200)).await;
    }
    println!("sentry carrier: standing down");
    radio.standby();
    park().await
}

/// Wait `secs`, keeping the heartbeat up across it.
async fn hold(secs: u64) {
    let until = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// Sit still, keeping the heartbeat up. For a source that declined to key.
///
/// It says so on a cadence rather than going quiet. A board that refused is
/// otherwise indistinguishable from a wedged one at the far end of a
/// console that was attached after the refusal was printed - and the
/// refusal is the more likely of the two, so it is the one that has to keep
/// being visible.
async fn park() -> ! {
    let mut said = Instant::now();
    loop {
        watchdog::beat(Task::Loop, Phase::Receive);
        if Instant::now() - said > Duration::from_secs(10) {
            println!("sentry source: parked, not transmitting");
            said = Instant::now();
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// Watch a duty cycle from outside the SPI bus. Does not return.
///
/// Every instrument in this module until now asked the chip a question, and
/// in a sniff loop the chip either will not answer or is ended by the
/// asking. BUSY is different: it is an input to the host, so reading it is
/// a pin read - no transaction, no NSS edge, nothing the radio can notice.
/// It is held high through a retained sleep and the startup behind it, and
/// released once the chip is awake, so watching it draws the shape of the
/// cycle from outside.
///
/// That answers the question the sweep could not reach: whether a receive
/// window is really as long as it was commanded. If a window commanded at
/// 200 ms is observed as twenty, nothing about preambles or margins was
/// ever going to matter.
///
/// It also mirrors BUSY and DIO1 onto the J1 header, so the same timing can
/// be taken by something that does not share this firmware's clock or its
/// bugs:
///
/// ```text
///   GPIO38   BUSY  (high = asleep or starting, low = awake)
///   GPIO39   DIO1  (high = an enabled interrupt is pending)
///   GPIO40   mark  (pulsed high for one pass when the sentry is armed)
/// ```
///
/// Those three are J1 header pins that nothing else claims; `main` parks
/// them as pulled-down inputs, and they are taken here the way the sleep
/// path takes the pads it has to hold.
#[cfg(feature = "iso-sentry-mirror")]
pub async fn mirror(radio: &mut Sx1262Driver<'_>) -> ! {
    use esp_hal::gpio::{Level, Output, OutputConfig};

    /// How long to watch before reporting, seconds.
    const WATCH_S: u64 = 30;
    /// Passes between heartbeats. The loop has no await in it, so the
    /// watchdog is fed by count rather than by time - and the monitor that
    /// would reset the board for a stall runs on the other core.
    const BEAT_EVERY: u32 = 20_000;

    // SAFETY: `main` parks these as inputs and nothing else ever claims
    // them; this build exists to drive them and does not return.
    let (mut out_busy, mut out_dio1, mut out_mark) = unsafe {
        (
            Output::new(esp_hal::peripherals::GPIO38::steal(), Level::Low, OutputConfig::default()),
            Output::new(esp_hal::peripherals::GPIO39::steal(), Level::Low, OutputConfig::default()),
            Output::new(esp_hal::peripherals::GPIO40::steal(), Level::Low, OutputConfig::default()),
        )
    };

    let g = geometry(radio);
    println!(
        "sentry mirror: rx {} us sleep {} us commanded, symb timeout {}",
        g.rx_us, g.sleep_us, SYMB_TIMEOUT
    );
    println!("sentry mirror: J1 GPIO38 = BUSY, GPIO39 = DIO1, GPIO40 = armed marker");

    radio.wake_from_retained_sleep().await;
    radio.arm_duty_cycle(g.rx_us, g.sleep_us, SYMB_TIMEOUT, irq::RX_DONE);
    out_mark.set_high();
    // Inside the first receive window, before the chip has slept once: the
    // one moment a status read is safe. Without it, a BUSY trace that looks
    // nothing like the commanded cycle could equally be a cycle that never
    // started, and those want opposite conclusions.
    {
        let (mode, err) = radio.health();
        println!("sentry mirror: armed, radio {} err 0x{:04X} (want rx)", mode, err);
    }

    // Timed here as well as mirrored, so a first answer needs no other
    // hardware - and so the two can be compared afterwards.
    let mut was_busy = radio.busy_high();
    let mut since = Instant::now();
    let mut lows: (u32, u32, u32, u64) = (0, u32::MAX, 0, 0); // n, min, max, sum
    let mut highs: (u32, u32, u32, u64) = (0, u32::MAX, 0, 0);
    let until = Instant::now() + Duration::from_secs(WATCH_S);
    let mut passes = 0u32;
    out_mark.set_low();

    while Instant::now() < until {
        let busy = radio.busy_high();
        out_busy.set_level(if busy { Level::High } else { Level::Low });
        out_dio1.set_level(if radio.irq_pending() { Level::High } else { Level::Low });
        if busy != was_busy {
            let held = (Instant::now() - since).as_micros() as u32;
            let bucket = if was_busy { &mut highs } else { &mut lows };
            bucket.0 += 1;
            bucket.1 = bucket.1.min(held);
            bucket.2 = bucket.2.max(held);
            bucket.3 += u64::from(held);
            was_busy = busy;
            since = Instant::now();
        }
        passes += 1;
        if passes % BEAT_EVERY == 0 {
            watchdog::beat(Task::Loop, Phase::Receive);
        }
    }

    let say = |name: &str, b: (u32, u32, u32, u64)| {
        if b.0 == 0 {
            println!("sentry mirror: {} never changed - the cycle is not running", name);
        } else {
            println!(
                "sentry mirror: {} x{}, min {} us, max {} us, mean {} us",
                name,
                b.0,
                b.1,
                b.2,
                (b.3 / u64::from(b.0)) as u32
            );
        }
    };
    println!("sentry mirror: after {} s -", WATCH_S);
    say("awake (BUSY low)", lows);
    say("asleep (BUSY high)", highs);
    println!(
        "sentry mirror: commanded rx {} us, sleep {} us - compare against awake and asleep above",
        g.rx_us, g.sleep_us
    );

    let mut said = Instant::now();
    loop {
        watchdog::beat(Task::Loop, Phase::Receive);
        if Instant::now() - said > Duration::from_secs(15) {
            println!("sentry mirror: done, results above; J1 pins still mirroring");
            said = Instant::now();
        }
        // Keep mirroring after the report so an external monitor can still
        // be attached and read the same cycle.
        for _ in 0..BEAT_EVERY {
            out_busy.set_level(if radio.busy_high() { Level::High } else { Level::Low });
            out_dio1.set_level(if radio.irq_pending() { Level::High } else { Level::Low });
        }
    }
}
