//! Deep sleep: park what a sleeping board cannot use, then go down.
//!
//! There is no rail to cut on this board - the GPS and the radio sit
//! directly on +3V3 - so the radio is the one load the firmware can
//! actually drop, and dropping it is worth doing: continuous RX is 5.5 mA
//! against the module's 9.3 uA asleep. The hardware loop owns the radio, so
//! this asks and waits rather than reaching for it.
//!
//! Or the radio is not dropped but left listening. A park that arms a
//! sentry leaves the SX1262 cycling its own receiver on its own timers,
//! and this registers the radio's DIO1 as a wake source beside the timer:
//! a completed wake frame then brings the chip back in the middle of an
//! interval that would otherwise have been the whole wait.
//!
//! The GPS goes down either way. `PrepareSleep` puts the receiver into
//! backup and holds the UART TX pad across the sleep, which together are
//! what make a sleeping board actually cheap: an M10 left acquiring is
//! around 30 mA against a chip that is otherwise in microamps, and a
//! sleeping S3 cannot use a fix anyway.

use embassy_time::{with_timeout, Duration};
use esp_hal::rtc_cntl::sleep::{Ext0WakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::Rtc;
use esp_println::println;
use midair_proto::evlog::Kind;
use midair_proto::posture::Request;
use midair_proto::supervise::{Phase, Task};

use crate::{event, evlog, settings, state, watchdog};

/// What the park is given on top of a transmit already in flight, ms.
///
/// The parts of the sequence take well under half a second between them;
/// the rest is room for the loop to be somewhere else - a config apply, a
/// panel refresh - when the request lands. Nothing is spent in the
/// ordinary case: this is a timeout rather than a delay, so the wait ends
/// when the park does.
const PARK_SLACK_MS: u64 = 1_500;

/// Park the radio, the GPS and the panel, then deep sleep for `interval_s`.
/// Does not return.
pub async fn enter_deep_sleep(rtc: &mut Rtc<'_>, interval_s: u32) -> ! {
    state::SLEEP_READY.reset();
    state::request(Request::PrepareSleep);
    // The loop notices within its 10 ms pass *unless* it is inside a
    // transmit, which awaits for the length of the frame on air - 289 ms at
    // the SF12/BW500 default and up to about 9.7 s at the slowest settings
    // the config accepts. A budget of one second here once slept over a
    // beacon with the SX1262 still in continuous receive: 5.7 mA for the
    // whole interval, and nothing visible afterwards because the wake
    // re-inits the radio anyway. The hardware loop also declines to start
    // a beacon while a sleep is pending, so this only has to cover one
    // already in flight.
    // The wait is bounded, but the bound can run past the serve loop's
    // heartbeat bound at the slowest settings, and it is a wait the loop
    // is meant to be in - so it is guarded rather than left to look like
    // a stall.
    let park = Duration::from_millis(u64::from(state::tx_worst_case_ms()) + PARK_SLACK_MS);
    let mut parked = watchdog::guarded(
        Task::Serve,
        Phase::Park,
        with_timeout(park, state::SLEEP_READY.wait()),
    )
    .await
    .is_ok();
    if !parked {
        // Once more before giving up on it. What is still awake is
        // whatever the hardware task had not reached: `PrepareSleep`
        // takes the receiver, the radio and the panel down before it
        // touches the card, so a first expiry is most likely the card
        // alone, mid-flush, and a second budget is what it needs.
        println!("sleep: park not finished, waiting once more");
        parked = watchdog::guarded(
            Task::Serve,
            Phase::Park,
            with_timeout(park, state::SLEEP_READY.wait()),
        )
        .await
        .is_ok();
    }
    if !parked {
        // Worth saying and worth counting: it means the sleep is about to
        // cost more than it should, and it is otherwise undetectable from
        // the far side. The count survives the sleep in RTC RAM and goes
        // out on the boot line and in the telemetry.
        settings::note_park_missed();
        event!(
            Kind::Sleep,
            "sleep: park did not finish in time, sleeping over it ({} missed since cold boot)",
            settings::parks_missed()
        );
    }
    // Whatever was noted and not yet written goes down now; the monitor
    // that would have written it is about to lose its RAM with the rest.
    watchdog::beat(Task::Serve, Phase::Sleep);
    evlog::flush().await;
    go_down(rtc, interval_s)
}

/// The last step of every deep sleep: hold the pads a sleeping board needs
/// held, register the wake sources, and go. Does not return.
///
/// Separate from the park above because one caller has no park to wait
/// for: a boot that a wake frame for some other node caused re-arms the
/// radio and comes straight here, from a boot that never started the
/// hardware task.
pub fn go_down(rtc: &mut Rtc<'_>, interval_s: u32) -> ! {
    // Hold what the sleeping board still needs held.
    //
    // The S3 releases every pad that is not explicitly held when the digital
    // domain drops (esp-hal clears `dg_pad_force_unhold` in its sleep prep),
    // and the SX1262 leaves sleep on a *falling* edge of NSS. A floating NSS
    // on an otherwise quiet board will produce one, so the radio that
    // `PrepareSleep` just put into cold sleep wakes itself back to STDBY_RC
    // and sits there for the whole interval - which is the same 5.7 mA the
    // parking was for. And a radio left as a sentry is ended by the same
    // edge: it wakes into standby and the cycle does not resume, so the
    // hold is what keeps the sentry listening at all. GPIO21 is inside the
    // S3's RTC GPIO range (0-21), so the pad hold reaches it.
    //
    // NRESET (GPIO7) is held for the sentry too. It is driven high by the
    // driver and released with everything else at sleep entry, and a reset
    // line drifting low would reset the radio into its power-up state -
    // deaf, with the antenna switch unpowered - with nothing anywhere to
    // say it happened.
    //
    // The pins belong to the SX1262 driver in the hardware task, so the
    // singletons are stolen for the holds. Nothing races: the driver is
    // parked and this function does not return.
    //
    // GPIO2 is UART1 TX into the M10's RX, and UART RX activity is one of
    // the receiver's two backup wake sources. A floating edge there undoes
    // the backup `PrepareSleep` just asked for and leaves the receiver
    // acquiring for the whole interval, which is the largest load a
    // sleeping board can carry. It is inside the RTC range too, so it is
    // held at the level the UART idles at.
    unsafe {
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO21::steal(), true);
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO7::steal(), true);
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO2::steal(), true);
    }

    settings::note_sleep(interval_s);
    // The RTC main timer keeps counting through the sleep - it is what the
    // wake source counts against - so a stamp here and a reading on the far
    // side is how long the wake itself took, less the interval asked for.
    // Nothing else on the board can measure that: every other clock stops.
    settings::note_sleep_at(rtc.time_since_boot().as_millis() as u32);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(interval_s as u64));
    match settings::sentry() {
        Some(s) => {
            println!(
                "deep sleep for {} s (mode {}, gps parked, radio listening {} ms every {} ms - DIO1 wakes)",
                interval_s,
                settings::get().mode.as_str(),
                s.rx_ms,
                s.sleep_ms
            );
            // DIO1 is GPIO9, inside the RTC range, and the radio holds it
            // high from `RxDone` until the interrupt is cleared - so a
            // frame that lands between the arm and this line is not lost:
            // the level is already there when the sleep begins, and the
            // chip comes straight back. The pin is the driver's, stolen
            // here for the same reason the held pads are.
            //
            // What this costs: `Ext0` keeps the RTC peripheral domain
            // powered through the sleep, tens of microamps against a
            // sentry average in the hundreds.
            let ext0 = Ext0WakeupSource::new(
                unsafe { esp_hal::peripherals::GPIO9::steal() },
                WakeupLevel::High,
            );
            rtc.sleep_deep(&[&timer, &ext0])
        }
        None => {
            println!(
                "deep sleep for {} s (mode {}, radio and gps parked)",
                interval_s,
                settings::get().mode.as_str()
            );
            rtc.sleep_deep(&[&timer])
        }
    }
}
