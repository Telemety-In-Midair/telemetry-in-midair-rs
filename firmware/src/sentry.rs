//! The bench for a duty-cycled receiver.
//!
//! One question decides whether a radio can usefully listen while the chip
//! that owns it is asleep: does a receive window opened at an arbitrary
//! point in a long preamble go on to complete the reception? The chip's
//! own sniff loop, `SetRxDutyCycle`, is the only thing on the part that
//! runs without the host, and two of its settings decide the answer -
//! the symbol timeout and which event stops the window timer. Read off the
//! datasheet those two look like tuning; measured, they are the difference
//! between a wake rate of two percent and one that works.
//!
//! The instrument is the radio. A second board sends frames behind a
//! preamble longer than the receiver's whole cycle, each carrying a
//! sequence number, and the receiver is armed exactly as a sleeping board
//! would arm it: DIO1 carrying `RxDone` and nothing else, and not one SPI
//! transaction while the cycle runs. What it counts is what a sleeping
//! board would count - wakes - and the sequence numbers say which frames
//! were missed.
//!
//! Nothing is read over SPI while a cycle is running because the chip will
//! not answer during its sleep phase and the asking ends the cycle: a
//! falling edge on NSS wakes it into standby, and the loop does not resume.
//! Every earlier version of this probe read the interrupt register whenever
//! DIO1 rose, which with preamble detection routed there could end a sentry
//! on a noise burst. `BUSY` is the one thing that can be watched for free -
//! it is an input to the host - so the window lengths are taken from it.
//!
//! Both halves need the same radio settings at each end, and the source
//! keys a PA, so read what [`source`] says before running it.

use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_println::println;
use midair_proto::supervise::{Phase, Task};

use crate::radio::Sx1262Driver;
use crate::sx1262::irq;
use crate::watchdog;

/// Tag byte in front of the sequence number a wake frame carries.
const WAKE_TAG: u8 = 0x57;

/// Preamble the source sends every frame with, in symbols.
///
/// Sized against the trials below: it has to be longer than the longest
/// cycle so that some window always opens inside it, and shorter than the
/// shortest restarted timer less the header so that the packet behind it
/// still fits. At SF12/BW500 this is 1352 ms.
const SOURCE_PREAMBLE_SYMBOLS: u16 = 165;

/// Longest the source will transmit before standing down, seconds.
///
/// A board left plugged in stays keyed until somebody remembers it, which
/// is exactly the state nobody is watching. Twenty minutes covers a full
/// probe run with room, and a run that needs longer can be restarted
/// deliberately.
const SOURCE_MAX_KEYED_S: u64 = 1_200;

/// Most transmit power the source will key at, dBm. Two boards on a bench
/// need milliwatts; 0 dBm is 1 mW, which is still an enormous signal at
/// that range.
const SOURCE_MAX_DBM: i8 = 10;

/// Seconds the source stays off the air before its first frame, so a
/// probe flashed second still sees the start.
const QUIET_FIRST_S: u64 = 5;

/// Symbols a LoRa explicit header occupies.
const HEADER_SYMBOLS: u32 = 8;

/// Symbols the modem needs inside a window to detect a preamble.
const DETECT_SYMBOLS: u32 = 4;

/// The board's configured oscillator startup, microseconds. Added between
/// the sleep and receive phases of a duty cycle, so it lengthens the cycle.
const TCXO_US: u32 = 10_000;

/// Seconds the control listens for.
const CONTROL_S: u64 = 20;

/// Seconds each trial runs. Long enough for about thirty frames at the
/// source's cadence.
const TRIAL_S: u64 = 75;

/// Passes between heartbeats in the loops that have no await in them.
///
/// Those loops watch pins, and a wait on the timer queue is not free on
/// this chip: every one is a critical section shared between the two
/// cores, and a poll of a millisecond has starved the watchdog monitor on
/// the other core before. A loop with no await at all costs the other
/// core nothing, and the monitor is fed by count instead of by time.
const BEAT_EVERY: u32 = 20_000;

/// One way of arming the sniff loop.
struct Trial {
    name: &'static str,
    rx_us: u32,
    sleep_us: u32,
    symb_timeout: u8,
    stop_on_preamble: bool,
}

/// What is tried, in order.
///
/// The first pair is the question: the datasheet's sniff loop as written,
/// then with the window timer told to stop on the preamble rather than the
/// header. The third is the old configuration for comparison, whose
/// symbol timeout of eight predicts a wake only from a window that opened
/// in the last eight symbols of the preamble - about three percent. The
/// fourth deliberately breaks the upper bound: its restarted timer is
/// shorter than the preamble, so it should catch only the frames whose
/// preamble was mostly over when the window opened. The last is a cheaper
/// window inside both bounds.
const TRIALS: [Trial; 5] = [
    Trial {
        name: "symb 0, stop on header, rx 300 sleep 900",
        rx_us: 300_000,
        sleep_us: 900_000,
        symb_timeout: 0,
        stop_on_preamble: false,
    },
    Trial {
        name: "symb 0, stop on preamble, rx 300 sleep 900",
        rx_us: 300_000,
        sleep_us: 900_000,
        symb_timeout: 0,
        stop_on_preamble: true,
    },
    Trial {
        name: "symb 8, stop on header, rx 200 sleep 1000 (the old arm)",
        rx_us: 200_000,
        sleep_us: 1_000_000,
        symb_timeout: 8,
        stop_on_preamble: false,
    },
    Trial {
        name: "symb 0, stop on preamble, rx 150 sleep 1000 (timer too short)",
        rx_us: 150_000,
        sleep_us: 1_000_000,
        symb_timeout: 0,
        stop_on_preamble: true,
    },
    Trial {
        name: "symb 0, stop on preamble, rx 250 sleep 950",
        rx_us: 250_000,
        sleep_us: 950_000,
        symb_timeout: 0,
        stop_on_preamble: true,
    },
];

/// The J1 header pins that mirror the radio's lines for an external
/// monitor:
///
/// ```text
///   GPIO38   BUSY  (high = asleep or starting, low = awake)
///   GPIO39   DIO1  (high = an enabled interrupt is pending)
///   GPIO40   mark  (pulsed high when a trial arms)
/// ```
///
/// `main` parks them as pulled-down inputs and nothing else claims them.
struct J1 {
    busy: Output<'static>,
    dio1: Output<'static>,
    mark: Output<'static>,
}

impl J1 {
    fn take() -> Self {
        // SAFETY: `main` parks these as inputs and nothing else ever claims
        // them; the bench builds drive them and do not return.
        unsafe {
            Self {
                busy: Output::new(
                    esp_hal::peripherals::GPIO38::steal(),
                    Level::Low,
                    OutputConfig::default(),
                ),
                dio1: Output::new(
                    esp_hal::peripherals::GPIO39::steal(),
                    Level::Low,
                    OutputConfig::default(),
                ),
                mark: Output::new(
                    esp_hal::peripherals::GPIO40::steal(),
                    Level::Low,
                    OutputConfig::default(),
                ),
            }
        }
    }

    /// Prove the wires before measuring through them. Each pin is pulsed a
    /// different number of times, so a monitor on the far end can tell not
    /// only that a line is connected but which line it is - a swapped pair
    /// otherwise reads as a perfectly plausible measurement. DIO1
    /// especially: it only moves on a reception, so in a run with nothing
    /// transmitting a dead wire and a good one look exactly alike.
    async fn self_test(&mut self) {
        println!("sentry: J1 self-test - BUSY x1, DIO1 x2, MARK x3 pulses of 50 ms");
        for (pin, times) in [
            (&mut self.busy, 1u8),
            (&mut self.dio1, 2),
            (&mut self.mark, 3),
        ] {
            for _ in 0..times {
                pin.set_high();
                Timer::after(Duration::from_millis(50)).await;
                pin.set_low();
                Timer::after(Duration::from_millis(50)).await;
            }
            Timer::after(Duration::from_millis(200)).await;
        }
    }

    fn mirror(&mut self, radio: &Sx1262Driver<'_>) {
        self.busy
            .set_level(if radio.busy_high() { Level::High } else { Level::Low });
        self.dio1
            .set_level(if radio.irq_pending() { Level::High } else { Level::Low });
    }
}

/// Run the probe. Does not return.
///
/// Expects the radio already initialized from the running config - the
/// board has to be in a mode that brings it up - and a second board
/// running [`source`] on the same settings.
pub async fn probe(radio: &mut Sx1262Driver<'_>) -> ! {
    let mut j1 = J1::take();
    j1.self_test().await;

    let t_sym = radio.symbol_time_us();
    let preamble_us = u32::from(SOURCE_PREAMBLE_SYMBOLS) * t_sym;
    println!(
        "sentry probe: {} us/symbol, source preamble {} symbols = {} us",
        t_sym, SOURCE_PREAMBLE_SYMBOLS, preamble_us
    );

    // The control first, while the chip is in a state that answers. It
    // uses the same interrupt the trials are judged on, so a failure here
    // is the link - a source that is off or on other settings - and a
    // failure below it with this passing is the arm.
    let control = control(radio).await;
    if control == 0 {
        println!("sentry probe: STOPPING - nothing to measure against");
        park("sentry probe").await
    }

    println!("sentry probe: {} trials of {} s", TRIALS.len(), TRIAL_S);
    let mut wakes = [0u32; TRIALS.len()];
    let mut offered = [0u32; TRIALS.len()];
    for (i, t) in TRIALS.iter().enumerate() {
        let (w, o) = trial(radio, &mut j1, t, preamble_us).await;
        wakes[i] = w;
        offered[i] = o;
    }

    println!("sentry probe: RESULTS, {} s a trial, control {} in {} s", TRIAL_S, control, CONTROL_S);
    for (i, t) in TRIALS.iter().enumerate() {
        let pct = if offered[i] > 0 {
            wakes[i] * 100 / offered[i]
        } else {
            0
        };
        println!(
            "sentry probe:   {:>3}% ({:>2} of {:>2})  {}",
            pct, wakes[i], offered[i], t.name
        );
    }
    park("sentry probe").await
}

/// Listen without sleeping and count completed receptions.
async fn control(radio: &mut Sx1262Driver<'_>) -> u32 {
    println!("sentry probe: control - continuous receive for {} s", CONTROL_S);
    radio.wake_from_retained_sleep().await;
    radio.arm_continuous_rx(irq::RX_DONE | irq::CRC_ERR);
    let until = Instant::now() + Duration::from_secs(CONTROL_S);
    let mut got = 0u32;
    let mut crc = 0u32;
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        if radio.irq_pending() {
            let status = radio.take_irq();
            if status & irq::CRC_ERR != 0 {
                crc += 1;
            }
            if status & irq::RX_DONE != 0 {
                got += 1;
                let (seq, pre) = read_wake(radio);
                println!("sentry probe: control heard seq {} ({} symbols)", seq, pre);
                radio.arm_continuous_rx(irq::RX_DONE | irq::CRC_ERR);
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
    let (mode, err) = radio.health();
    println!(
        "sentry probe: CONTROL {} frames, {} crc errors in {} s, radio {} err 0x{:04X}",
        got, crc, CONTROL_S, mode, err
    );
    got
}

/// The sequence number and preamble length the frame in the buffer
/// carries, or zeros if it is something else.
fn read_wake(radio: &mut Sx1262Driver<'_>) -> (u16, u16) {
    let mut buf = [0u8; 8];
    let n = radio.read_payload(&mut buf);
    if n >= 5 && buf[0] == WAKE_TAG {
        (
            u16::from_le_bytes([buf[1], buf[2]]),
            u16::from_le_bytes([buf[3], buf[4]]),
        )
    } else {
        (0, 0)
    }
}

/// Arm one way and count what wakes it. Returns `(wakes, frames offered)`,
/// the second from the sequence numbers the wakes carried.
async fn trial(
    radio: &mut Sx1262Driver<'_>,
    j1: &mut J1,
    t: &Trial,
    preamble_us: u32,
) -> (u32, u32) {
    let t_sym = radio.symbol_time_us();
    // What the datasheet's two bounds say about this arm against the
    // source's preamble: the cycle a preamble has to span so that a window
    // opens inside it with room to detect, and the restarted timer the rest
    // of the frame has to fit in. Printed so a trial that fails a bound on
    // paper is not mistaken for one that failed on the chip.
    let cycle_us = t.rx_us + t.sleep_us + TCXO_US;
    let need_min = cycle_us + DETECT_SYMBOLS * t_sym;
    let hold_us = 2 * t.rx_us + t.sleep_us;
    let need_max = hold_us + DETECT_SYMBOLS * t_sym - HEADER_SYMBOLS * t_sym;
    println!("sentry probe: TRIAL {}", t.name);
    println!(
        "sentry probe:   cycle {} us, hold after detect {} us, preamble {} us must be in {}..{} - {}",
        cycle_us,
        hold_us,
        preamble_us,
        need_min,
        need_max,
        if preamble_us >= need_min && preamble_us <= need_max {
            "inside both bounds"
        } else if preamble_us < need_min {
            "SHORTER than the cycle"
        } else {
            "LONGER than the restarted timer"
        }
    );

    let mut wakes = 0u32;
    let mut arms = 0u32;
    // Which sequence numbers woke it, low byte, so the frames missed
    // between two wakes can be counted.
    let mut seen = [0u64; 4];
    let mut seq_lo = u16::MAX;
    let mut seq_hi = 0u16;
    // BUSY is high only through the transitions, so its low periods
    // alternate between the window and the sleep. Held windows - ones a
    // preamble extended - show up as low periods longer than the window
    // and shorter than the sleep.
    const GLITCH_US: u32 = 1_000;
    let mut lows = 0u32;
    let mut shortest = u32::MAX;
    let mut longest = 0u32;
    let mut held = 0u32;

    macro_rules! arm {
        () => {{
            radio.wake_from_retained_sleep().await;
            radio.arm_duty_cycle(
                t.rx_us,
                t.sleep_us,
                t.symb_timeout,
                t.stop_on_preamble,
                irq::RX_DONE,
            );
            arms += 1;
            j1.mark.set_high();
        }};
    }

    arm!();
    let mut armed_at = Instant::now();
    let until = Instant::now() + Duration::from_secs(TRIAL_S);
    let mut was_busy = radio.busy_high();
    let mut since = Instant::now();
    let mut passes = 0u32;
    let mut said = Instant::now();
    while Instant::now() < until {
        j1.mirror(radio);
        let busy = radio.busy_high();
        if busy != was_busy {
            if !was_busy {
                let held_us = (Instant::now() - since).as_micros() as u32;
                if held_us >= GLITCH_US {
                    lows += 1;
                    shortest = shortest.min(held_us);
                    longest = longest.max(held_us);
                    // A window is the shorter phase; the sleep is at least
                    // the commanded sleep less the timer's error.
                    let window_us = if t.symb_timeout == 0 {
                        t.rx_us
                    } else {
                        (u32::from(t.symb_timeout) * t_sym).min(t.rx_us)
                    };
                    if held_us > window_us + window_us / 2 && held_us < t.sleep_us * 9 / 10 {
                        held += 1;
                    }
                }
            }
            was_busy = busy;
            since = Instant::now();
        }
        // A reception is the only thing on DIO1, and it has ended the
        // cycle: the chip is in standby, so reading it disturbs nothing.
        if radio.irq_pending() {
            j1.mark.set_low();
            let status = radio.take_irq();
            if status & irq::RX_DONE != 0 {
                wakes += 1;
                let (seq, pre) = read_wake(radio);
                if seq != 0 {
                    seen[usize::from(seq & 0xFF) / 64] |= 1u64 << (seq % 64);
                    seq_lo = seq_lo.min(seq);
                    seq_hi = seq_hi.max(seq);
                }
                println!(
                    "sentry probe:   WOKEN by seq {} ({} symbols), {} ms after arming",
                    seq,
                    pre,
                    (Instant::now() - armed_at).as_millis()
                );
            } else {
                println!("sentry probe:   DIO1 without RxDone, irq 0x{:04X}", status);
            }
            arm!();
            armed_at = Instant::now();
            was_busy = radio.busy_high();
            since = Instant::now();
        }
        passes += 1;
        if passes % BEAT_EVERY == 0 {
            watchdog::beat(Task::Loop, Phase::Receive);
            if Instant::now() - said > Duration::from_secs(25) {
                println!(
                    "sentry probe:   {} s left, {} wakes",
                    (until - Instant::now()).as_secs(),
                    wakes
                );
                said = Instant::now();
            }
        }
    }
    j1.mark.set_low();
    // Out of the cycle, into somewhere known.
    radio.wake_from_retained_sleep().await;
    let (mode, err) = radio.health();

    let offered = if wakes > 0 {
        u32::from(seq_hi - seq_lo) + 1
    } else {
        0
    };
    let distinct: u32 = seen.iter().map(|w| w.count_ones()).sum();
    println!(
        "sentry probe:   {} wakes ({} distinct) from seq {}..{} = {} frames offered, {} arms, radio {} err 0x{:04X}",
        wakes, distinct, seq_lo, seq_hi, offered, arms, mode, err
    );
    if lows > 0 {
        println!(
            "sentry probe:   BUSY low periods x{}, shortest {} us, longest {} us, {} held windows",
            lows, shortest, longest, held
        );
    } else {
        println!("sentry probe:   BUSY never moved - the cycle did not run");
    }
    (wakes, offered)
}

/// Send wake frames, each behind [`SOURCE_PREAMBLE_SYMBOLS`] of preamble
/// and carrying its sequence number. Does not return.
///
/// Two things make keying a PA on a bench acceptable rather than reckless,
/// and both are checked here rather than left to whoever is at the bench:
///
/// - **The power has to be low.** At the top of the range the PA is 127 mA
///   in a module that normally sees a third of a second at a time. A board
///   configured above [`SOURCE_MAX_DBM`] is refused.
/// - **The antenna switch has to be right.** DIO2 switches it and DIO3
///   supplies it, so this only ever follows the ordinary initialization
///   that sets both, and the latched device errors are read before keying -
///   `XOSC_START` means the oscillator did not start, which on this module
///   means the switch is unpowered, and keying into an isolated port
///   destroys the part.
///
/// The gap between frames is jittered so the source's period and the
/// receiver's cycle cannot settle into a phase where every window lands on
/// the same part of every frame.
pub async fn source(radio: &mut Sx1262Driver<'_>) -> ! {
    let dbm = radio.power_dbm();
    if dbm > SOURCE_MAX_DBM {
        println!(
            "sentry source: REFUSING to send at {} dBm - {} dBm or less",
            dbm, SOURCE_MAX_DBM
        );
        park("sentry source").await
    }
    let err = radio.device_errors();
    if err != 0 {
        println!("sentry source: REFUSING to transmit, device errors 0x{:04X}", err);
        park("sentry source").await
    }

    println!("sentry source: quiet for {} s", QUIET_FIRST_S);
    hold(QUIET_FIRST_S).await;
    println!(
        "sentry source: frames with a {} symbol preamble ({} us) at {} dBm",
        SOURCE_PREAMBLE_SYMBOLS,
        u32::from(SOURCE_PREAMBLE_SYMBOLS) * radio.symbol_time_us(),
        dbm
    );

    let until = Instant::now() + Duration::from_secs(SOURCE_MAX_KEYED_S);
    let mut seq = 1u16;
    let mut sent = 0u32;
    let mut failed = 0u32;
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::TxSend);
        let payload = [
            WAKE_TAG,
            seq as u8,
            (seq >> 8) as u8,
            SOURCE_PREAMBLE_SYMBOLS as u8,
            (SOURCE_PREAMBLE_SYMBOLS >> 8) as u8,
        ];
        if radio.send_wake(&payload, SOURCE_PREAMBLE_SYMBOLS).await.is_ok() {
            sent += 1;
        } else {
            failed += 1;
        }
        if seq % 10 == 0 {
            let (mode, err) = radio.health();
            if err != 0 {
                println!("sentry source: STANDING DOWN, radio latched 0x{:04X}", err);
                radio.standby();
                park("sentry source").await
            }
            println!(
                "sentry source: seq {} sent, {} ok {} failed, radio {}, {} s left",
                seq,
                sent,
                failed,
                mode,
                (until - Instant::now()).as_secs()
            );
        }
        // 600 to 1400 ms, stepping through five values.
        let gap_ms = 600 + u64::from(seq % 5) * 200;
        hold_ms(gap_ms).await;
        seq = seq.wrapping_add(1).max(1);
    }
    println!("sentry source: budget spent, standing down");
    radio.standby();
    park("sentry source").await
}

/// Key a continuous preamble. Does not return.
///
/// For a receiver whose windows are being timed rather than counted: a
/// signal that is always present and never becomes a packet is what says
/// whether a window that detects something is held open by it. The same
/// guards as the frame source, for the same reasons.
#[cfg(feature = "iso-sentry-carrier")]
pub async fn carrier(radio: &mut Sx1262Driver<'_>) -> ! {
    let dbm = radio.power_dbm();
    if dbm > SOURCE_MAX_DBM {
        println!(
            "sentry carrier: REFUSING to key at {} dBm - {} dBm or less",
            dbm, SOURCE_MAX_DBM
        );
        park("sentry carrier").await
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
        park("sentry carrier").await
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
                park("sentry carrier").await
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
    park("sentry carrier").await
}

/// Wait `secs`, keeping the heartbeat up across it.
async fn hold(secs: u64) {
    hold_ms(secs * 1000).await
}

async fn hold_ms(ms: u64) {
    let until = Instant::now() + Duration::from_millis(ms);
    while Instant::now() < until {
        watchdog::beat(Task::Loop, Phase::Receive);
        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Sit still, keeping the heartbeat up, saying so on a cadence.
///
/// A finished run and a wedged one look the same to a console attached
/// after the fact, and the results are the lines above - so the fact that
/// there are results to scroll up to has to keep being said.
async fn park(who: &str) -> ! {
    let mut said = Instant::now();
    loop {
        watchdog::beat(Task::Loop, Phase::Receive);
        if Instant::now() - said > Duration::from_secs(15) {
            println!("{}: done, results above", who);
            said = Instant::now();
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// Time the sniff loop's phases off BUSY, with nothing transmitting, and
/// mirror the lines to J1. Does not return.
///
/// Sweeps the symbol timeout with the receive period opened wide, so the
/// symbol count alone decides how long an empty window stays open. What
/// it measured: the window is `SymbNum` symbol times exactly, and with the
/// count at zero it is the receive period.
#[cfg(feature = "iso-sentry-mirror")]
pub async fn mirror(radio: &mut Sx1262Driver<'_>) -> ! {
    const SYMB_SWEEP: [u8; 6] = [0, 4, 8, 16, 32, 64];
    const SWEEP_RX_US: u32 = 1_000_000;
    const SLEEP_US: u32 = 1_000_000;
    const PER_STEP_S: u64 = 12;
    const GLITCH_US: u32 = 1_000;

    let mut j1 = J1::take();
    j1.self_test().await;
    println!(
        "sentry mirror: sweeping the symbol timeout, rx ceiling {} us, sleep {} us",
        SWEEP_RX_US, SLEEP_US
    );
    println!("sentry mirror:  symbols   predicted_us   measured_us");

    for symbs in SYMB_SWEEP {
        radio.wake_from_retained_sleep().await;
        radio.arm_duty_cycle(SWEEP_RX_US, SLEEP_US, symbs, false, irq::RX_DONE);
        j1.mark.set_high();

        let mut was_busy = radio.busy_high();
        let mut since = Instant::now();
        let mut shortest = u32::MAX;
        let mut longest = 0u32;
        let until = Instant::now() + Duration::from_secs(PER_STEP_S);
        let mut passes = 0u32;
        while Instant::now() < until {
            j1.mirror(radio);
            let busy = radio.busy_high();
            if busy != was_busy {
                if !was_busy {
                    let held = (Instant::now() - since).as_micros() as u32;
                    if held >= GLITCH_US {
                        shortest = shortest.min(held);
                        longest = longest.max(held);
                    }
                }
                was_busy = busy;
                since = Instant::now();
            }
            passes += 1;
            if passes % BEAT_EVERY == 0 {
                watchdog::beat(Task::Loop, Phase::Receive);
            }
        }
        j1.mark.set_low();

        let predicted = if symbs == 0 {
            SWEEP_RX_US
        } else {
            (u32::from(symbs) * radio.symbol_time_us()).min(SWEEP_RX_US)
        };
        if shortest == u32::MAX {
            println!("sentry mirror: {:>8}   {:>12}   no transitions", symbs, predicted);
        } else {
            println!(
                "sentry mirror: {:>8}   {:>12}   {:>11}   (long phase {} us)",
                symbs, predicted, shortest, longest
            );
        }
    }
    radio.wake_from_retained_sleep().await;
    park("sentry mirror").await
}
