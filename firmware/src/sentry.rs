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
    classify_charge, summarize_intervals, sweep_floor, SweepStep, TcxoCharge, DETECT_SYMBOLS,
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

/// Longest the cadence half will spend collecting them, seconds.
///
/// It tolerates gaps rather than stopping at the first one - a source that
/// is not on the air is not a receiver that stopped cycling - so it needs a
/// budget of its own or a dead source would hold it forever.
const CADENCE_BUDGET_S: u64 = 90;

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

/// Longest the source will hold the PA on before standing down, seconds.
///
/// Keying with no end to it was a convenience, and it is the wrong default
/// for a thing that lives on a bench: a board left plugged in stays keyed
/// until somebody remembers it, which is exactly the state nobody is
/// watching. Twenty minutes covers a full probe run with room, and a run
/// that needs longer can be restarted deliberately.
const SOURCE_MAX_KEYED_S: u64 = 1_200;

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

    // The control first. Everything after it assumes a signal is reachable,
    // and without this a silent source and a duty cycle that never ran
    // produce the same "nothing detected" from every phase below.
    if !hearing(radio).await {
        println!("sentry probe: STOPPING - nothing to measure against");
        loop {
            watchdog::beat(Task::Loop, Phase::Receive);
            Timer::after(Duration::from_millis(500)).await;
        }
    }
    if !cadence(radio, listening).await {
        println!("sentry probe: STOPPING - the sweep is the same arm forty times over");
        loop {
            watchdog::beat(Task::Loop, Phase::Receive);
            Timer::after(Duration::from_millis(500)).await;
        }
    }
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

/// Arm once and time the detections that follow.
///
/// Gaps are tolerated rather than treated as the end of the run. A source
/// that stops transmitting and a chip that stops cycling look identical
/// from one timeout, and they are not the same finding - so this keeps
/// waiting and lets the interval lengths separate them afterwards.
async fn cadence(radio: &mut Sx1262Driver<'_>, listening: u32) -> bool {
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

    // What mode the chip actually went to, sampled across a cycle. The
    // command either took or it did not, and that is the difference between
    // a receive window too short to hear anything and a chip that never
    // entered the cycle at all - which nothing else here can tell apart.
    for i in 0..6 {
        Timer::after(Duration::from_millis(200)).await;
        let (mode, err) = radio.health();
        println!("sentry probe: armed +{} ms, radio {} err 0x{:04X}", (i + 1) * 200, mode, err);
    }

    // Per-wait deadline, and the budget for the whole collection.
    let wait = Duration::from_micros(u64::from(commanded) * u64::from(TRIAL_CYCLES));
    let give_up = Instant::now() + Duration::from_secs(CADENCE_BUDGET_S);
    let mut intervals: heapless::Vec<u32, CADENCE_SAMPLES> = heapless::Vec::new();
    let mut last: Option<Instant> = None;
    let mut detections = 0u32;
    while intervals.len() < CADENCE_SAMPLES && Instant::now() < give_up {
        // Deliberately no re-arm: whether the chip keeps cycling on its own
        // is half of what this measures, and re-arming would start the
        // cycle at its receive phase and measure the window instead.
        match wait_for_detect(radio, wait).await {
            Some(at) => {
                detections += 1;
                if let Some(prev) = last {
                    let _ = intervals.push((at - prev).as_micros() as u32);
                }
                last = Some(at);
            }
            // Nothing this time. The next detection's interval spans the
            // gap, which is what marks it as one.
            None => continue,
        }
    }

    println!("sentry probe: {} detections, {} intervals", detections, intervals.len());
    let Some(i) = summarize_intervals(&intervals, commanded) else {
        if detections == 0 {
            println!("sentry probe: cadence FAILED - nothing detected at all");
            println!("sentry probe: no signal, wrong settings, or the cycle never ran");
        } else {
            println!("sentry probe: cadence FAILED - detections, but never two a cycle apart");
            println!("sentry probe: the chip is not staying in the cycle, or the source is barely on");
        }
        return false;
    };
    println!(
        "sentry probe: {} at the cycle, {} over it (gaps in the source)",
        i.tight, i.loose
    );
    println!(
        "sentry probe: cycle mean {} us, min {} us, max {} us",
        i.mean_us, i.min_us, i.max_us
    );
    // Two consecutive windows both hearing is the chip cycling unaided; a
    // run of them is it keeping that up.
    println!(
        "sentry probe: the chip stays in the cycle by itself ({} consecutive pairs)",
        i.tight
    );
    match classify_charge(commanded, i.mean_us, OVERHEAD_LADDER_US[0]) {
        TcxoCharge::Added => println!(
            "sentry probe: oscillator restart is ADDED - a window listens for as long as commanded"
        ),
        TcxoCharge::Absorbed => println!(
            "sentry probe: oscillator restart is ABSORBED - every window must grow by the overhead"
        ),
        TcxoCharge::Unclear => println!(
            "sentry probe: cadence UNCLEAR - {} us against a commanded {} us, look at the run",
            i.mean_us, commanded
        ),
    }
    true
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
            "sentry source: REFUSING to key at {} dBm - {} dBm or less (board-config --set power_dbm=0)",
            dbm, SOURCE_MAX_DBM
        );
        park().await
    }
    let err = radio.key_infinite_preamble();
    if err != 0 {
        // Not a burst that failed - a radio that must not be keyed at all.
        println!("sentry source: REFUSING to key, device errors 0x{:04X}", err);
        radio.standby();
        park().await
    }
    println!(
        "sentry source: keyed at {} dBm, standing down after {} s",
        dbm, SOURCE_MAX_KEYED_S
    );
    let until = Instant::now() + Duration::from_secs(SOURCE_MAX_KEYED_S);
    let mut said = Instant::now();
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        if Instant::now() - said > Duration::from_secs(30) {
            // The errors are read to be acted on, not just printed. A radio
            // that latches one while keyed has something wrong with the
            // transmit path - on this module DIO3 also supplies the antenna
            // switch, so the failure that matters is the one where the PA is
            // driving an isolated port. Standing down on it is the whole
            // reason to look.
            let (mode, err) = radio.health();
            if err != 0 {
                println!("sentry source: STANDING DOWN, radio latched 0x{:04X} while keyed", err);
                radio.standby();
                park().await
            }
            let left = (until - Instant::now()).as_secs();
            // The power is on this line and not only on the one at boot,
            // because the boot line scrolls away and this is the fact worth
            // being able to confirm at any moment: a console attached
            // halfway through a run should still be able to say what the PA
            // is doing, without trusting that a config push earlier landed.
            println!(
                "sentry source: still keyed at {} dBm, radio {}, {} s left",
                dbm, mode, left
            );
            said = Instant::now();
        }
        Timer::after(Duration::from_millis(200)).await;
    }
    println!("sentry source: {} s up, standing down", SOURCE_MAX_KEYED_S);
    radio.standby();
    park().await
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
