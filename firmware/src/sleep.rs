//! Deep sleep: park what a sleeping board cannot use, then go down.
//!
//! There is no rail to cut on this board - the GPS and SD sit directly on
//! +3V3 - so the radio is the one load the firmware can actually drop, and
//! dropping it is worth doing: continuous RX is 5.5 mA against the module's
//! 9.3 uA asleep. The hardware loop owns the radio, so this asks and waits
//! rather than reaching for it.
//!
//! The GPS goes with it. `PrepareSleep` puts the receiver into backup and
//! holds the UART TX pad across the sleep, which together are what make a
//! sleeping board actually cheap: an M10 left acquiring is around 30 mA
//! against a chip that is otherwise in microamps, and a sleeping S3 cannot
//! use a fix anyway.

use embassy_time::{with_timeout, Duration};
use esp_hal::rtc_cntl::sleep::TimerWakeupSource;
use esp_hal::rtc_cntl::Rtc;
use esp_println::println;
use midair_proto::posture::Request;

use crate::{settings, state};

/// What the park is given on top of a transmit already in flight, ms.
///
/// The bounded parts of the sequence take well under half a second; the
/// card's flush and unmount can stall on wear levelling for most of
/// another, and an expiry there was enough to sleep over a park that had
/// not finished. Nothing is spent in the ordinary case: this is a timeout
/// rather than a delay, so the wait ends when the park does.
const PARK_SLACK_MS: u64 = 1_500;

/// Park the radio, the GPS and the card, then deep sleep for `interval_s`.
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
    let park = Duration::from_millis(u64::from(state::tx_worst_case_ms()) + PARK_SLACK_MS);
    let mut parked = with_timeout(park, state::SLEEP_READY.wait()).await.is_ok();
    if !parked {
        // Once more before giving up on it. What is still awake is
        // whatever the hardware task had not reached: `PrepareSleep`
        // takes the receiver, the radio and the panel down before it
        // touches the card, so a first expiry is most likely the card
        // alone, mid-flush, and a second budget is what it needs.
        println!("sleep: park not finished, waiting once more");
        parked = with_timeout(park, state::SLEEP_READY.wait()).await.is_ok();
    }
    if !parked {
        // Worth saying and worth counting: it means the sleep is about to
        // cost more than it should, and it is otherwise undetectable from
        // the far side. The count survives the sleep in RTC RAM and goes
        // out on the boot line and in the telemetry.
        settings::note_park_missed();
        println!(
            "sleep: park did not finish in time, sleeping over it ({} missed since cold boot)",
            settings::parks_missed()
        );
    }

    // Hold what the sleeping board still needs held.
    //
    // The S3 releases every pad that is not explicitly held when the digital
    // domain drops (esp-hal clears `dg_pad_force_unhold` in its sleep prep),
    // and the SX1262 leaves sleep on a *falling* edge of NSS. A floating NSS
    // on an otherwise quiet board will produce one, so the radio that
    // `PrepareSleep` just put into cold sleep wakes itself back to STDBY_RC
    // and sits there for the whole interval - which is the same 5.7 mA the
    // parking was for. GPIO21 is inside the S3's RTC GPIO range (0-21), so
    // the pad hold reaches it.
    //
    // The pin belongs to the SX1262 driver in the hardware task, so the
    // singleton is stolen for the hold. Nothing races: the driver is parked
    // and this function does not return.
    //
    // SD CS (GPIO44) has the same problem and is not fixable the same way -
    // the S3's RTC pins stop at 21, so a digital pad needs the
    // `RTC_CNTL_DIG_PAD_HOLD` register that esp-hal 1.0 does not expose.
    //
    // GPIO2 is UART1 TX into the M10's RX, and UART RX activity is one of
    // the receiver's two backup wake sources. A floating edge there undoes
    // the backup `PrepareSleep` just asked for and leaves the receiver
    // acquiring for the whole interval, which is the largest load a
    // sleeping board can carry. It is inside the RTC range too, so it is
    // held at the level the UART idles at.
    unsafe {
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO21::steal(), true);
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO2::steal(), true);
    }

    settings::note_sleep(interval_s);
    println!(
        "deep sleep for {} s (mode {}, radio and gps parked)",
        interval_s,
        settings::get().mode.as_str()
    );
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(interval_s as u64));
    rtc.sleep_deep(&[&timer])
}
