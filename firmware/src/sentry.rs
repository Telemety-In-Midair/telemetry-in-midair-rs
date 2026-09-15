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
    classify_charge, sweep_floor, SweepStep, TcxoCharge, DETECT_SYMBOLS,
};
use midair_proto::supervise::{Phase, Task};

use crate::radio::Sx1262Driver;
use crate::sx1262::irq;
use crate::watchdog;

/// Sleep half of the cycle under test, microseconds.
///
/// Long enough that a window is a small part of the cycle, so a detection
/// is unambiguously one window's, and short enough that a sweep step is
/// seconds rather than minutes.
const SLEEP_US: u32 = 1_000_000;

/// Detections to collect for the cadence measurement.
const CADENCE_SAMPLES: usize = 16;

/// Trials per sweep step. Forty is enough that a window which passes every
/// one is not passing by luck, and few enough that the whole ladder runs in
/// a couple of minutes.
const SWEEP_TRIALS: u32 = 40;

/// Window overheads to try, microseconds, longest first.
///
/// The ladder is in overhead rather than in absolute window length, so the
/// same sweep runs at any spreading factor: each step is the symbols the
/// modem has to count plus this much headroom, and the shortest step that
/// still detects every time *is* the overhead. Descending, because
/// [`sweep_floor`] only accepts a step whose longer neighbors all passed.
const OVERHEAD_LADDER_US: [u32; 10] = [
    50_000, 30_000, 20_000, 15_000, 12_000, 10_000, 8_000, 5_000, 2_000, 0,
];

/// How long a single trial waits for a detection before calling it a miss,
/// as a multiple of the commanded cycle. Three cycles is generous for
/// something that should happen on the first.
const TRIAL_CYCLES: u32 = 3;

/// Poll period on DIO1, milliseconds.
///
/// The pin latches high until the interrupt is cleared, so this sets the
/// resolution of a timestamp and not whether a detection is seen at all.
/// One millisecond against a cycle of about a second is a tenth of a
/// percent, which is well inside the chip's own RC timebase.
const POLL_MS: u64 = 1;

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

    cadence(radio, listening).await;
    let steps = sweep(radio, listening).await;

    println!("sentry probe:");
    match sweep_floor(&steps, t_sym, DETECT_SYMBOLS) {
        Some(f) => {
            println!(
                "sentry probe: floor {} us, so {} us of overhead over {} symbols",
                f.rx_us, f.overhead_us, DETECT_SYMBOLS
            );
            println!("sentry probe: size every window as {} symbols + {} us", DETECT_SYMBOLS, f.overhead_us);
        }
        // Either nothing detected at all, or a longer window failed while a
        // shorter one passed. Both are runs to repeat rather than numbers to
        // design against, and the table above says which happened.
        None => println!("sentry probe: NO FLOOR - nothing detected, or the sweep is not clean from the top"),
    }
    // Nothing else on this build has anything to do, and the loop is what
    // keeps the board's watchdog fed while the console output is read off.
    loop {
        watchdog::beat(Task::Loop, Phase::Receive);
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Arm once and time the detections that follow.
async fn cadence(radio: &mut Sx1262Driver<'_>, listening: u32) {
    // A generous window: this half is about the cycle, not about how short
    // a window can be, so nothing here should fail for want of listening
    // time.
    let rx_us = listening + OVERHEAD_LADDER_US[0];
    let commanded = rx_us + SLEEP_US;
    println!(
        "sentry probe: cadence, rx {} us sleep {} us, commanded cycle {} us",
        rx_us, SLEEP_US, commanded
    );
    radio.arm_duty_cycle(rx_us, SLEEP_US, DETECT_SYMBOLS, irq::PREAMBLE_DETECTED);

    let deadline = Duration::from_micros(u64::from(commanded) * u64::from(TRIAL_CYCLES));
    let mut last: Option<Instant> = None;
    let mut n = 0u32;
    let mut total_us = 0u64;
    let mut min_us = u32::MAX;
    let mut max_us = 0u32;
    for _ in 0..CADENCE_SAMPLES {
        // Deliberately no re-arm between samples: whether the chip keeps
        // cycling on its own is half of what this measures.
        let Some(at) = wait_for_detect(radio, deadline).await else {
            break;
        };
        if let Some(prev) = last {
            let us = (at - prev).as_micros() as u32;
            n += 1;
            total_us += u64::from(us);
            min_us = min_us.min(us);
            max_us = max_us.max(us);
        }
        last = Some(at);
    }

    if n == 0 {
        println!("sentry probe: cadence FAILED - fewer than two detections");
        println!("sentry probe: either the cycle is not running, or the chip left it after the first");
        return;
    }
    let mean = (total_us / u64::from(n)) as u32;
    println!(
        "sentry probe: {} intervals, mean {} us, min {} us, max {} us",
        n, mean, min_us, max_us
    );
    if (n as usize) < CADENCE_SAMPLES - 1 {
        // It stopped early. That is a finding rather than a failure: a chip
        // that leaves the cycle on a reception has to be re-armed by
        // whatever wakes up, which is a step a sleeping board must not skip.
        println!("sentry probe: stopped after {} - the chip does not stay in the cycle by itself", n + 1);
    }
    match classify_charge(commanded, mean, OVERHEAD_LADDER_US[0]) {
        TcxoCharge::Added => println!(
            "sentry probe: oscillator restart is ADDED - a window listens for as long as commanded"
        ),
        TcxoCharge::Absorbed => println!(
            "sentry probe: oscillator restart is ABSORBED - every window must grow by the overhead"
        ),
        TcxoCharge::Unclear => println!(
            "sentry probe: cadence UNCLEAR - {} us against a commanded {} us, look at the run",
            mean, commanded
        ),
    }
}

/// Walk the window down and count what still detects.
async fn sweep(radio: &mut Sx1262Driver<'_>, listening: u32) -> heapless::Vec<SweepStep, 16> {
    println!("sentry probe: sweep, {} trials a step", SWEEP_TRIALS);
    println!("sentry probe:  overhead_us     rx_us  detects");
    let mut steps = heapless::Vec::new();
    for overhead in OVERHEAD_LADDER_US {
        let rx_us = listening + overhead;
        let deadline =
            Duration::from_micros(u64::from(rx_us + SLEEP_US) * u64::from(TRIAL_CYCLES));
        let mut detects = 0;
        for _ in 0..SWEEP_TRIALS {
            // Re-armed per trial, so each one is an independent question:
            // does a window of this length catch a signal that is already
            // there? The cycle's own continuation is the other half's job.
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
    loop {
        watchdog::beat(Task::Loop, Phase::Receive);
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

/// Key a continuous preamble in bounded bursts, as the signal the probe
/// measures against. Does not return.
///
/// **This keys the PA and leaves it keyed.** On this board DIO2 switches
/// the antenna and DIO3 supplies that switch, so it only runs after the
/// ordinary initialization has set both - which is why it takes an
/// initialized driver and checks the latched device errors before every
/// burst rather than trusting the first one. `XOSC_START` latched means the
/// oscillator did not start, which on this module means the switch is
/// unpowered, and keying into an isolated port destroys the part.
///
/// Bursts rather than a truly continuous carrier: the PA is 127 mA in a
/// module that normally sees a third of a second at a time, and the boards
/// are centimeters apart on a bench. Turn the power down before running it.
pub async fn source(radio: &mut Sx1262Driver<'_>) -> ! {
    /// Seconds keyed, then seconds quiet.
    const BURST_S: u64 = 10;
    println!("sentry source: {} s keyed, {} s quiet, repeating", BURST_S, BURST_S);
    loop {
        let err = radio.key_infinite_preamble();
        if err != 0 {
            // Not a burst that failed - a radio that must not be keyed.
            println!("sentry source: REFUSING to key, device errors 0x{:04X}", err);
            radio.standby();
            loop {
                watchdog::beat(Task::Loop, Phase::Receive);
                Timer::after(Duration::from_millis(500)).await;
            }
        }
        println!("sentry source: keyed");
        hold(BURST_S).await;
        radio.standby();
        println!("sentry source: quiet");
        hold(BURST_S).await;
    }
}

/// Wait `secs`, keeping the heartbeat up across it.
async fn hold(secs: u64) {
    let until = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        Timer::after(Duration::from_millis(200)).await;
    }
}
