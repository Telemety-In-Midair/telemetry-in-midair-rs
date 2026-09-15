//! Wio-S3 firmware for the wio-s3-max-gps board.
//!
//! One module holds BLE, LoRa and GPS. A config write is a request the
//! hardware loop picks up on its next pass, and a command the serve loop
//! picks up in whichever wait it is in.
//!
//! This file is the boot: the clock, the pins, the buses, the flash and the
//! settings, then the two loops. The BLE side is [`wio_s3_gps::ble`], the
//! hardware side [`wio_s3_gps::hardware`].
//!
//! - Reads a MAX-M10N on UART1 and folds NMEA into a position.
//! - Broadcasts that position over 915 MHz LoRa on the configured interval,
//!   or a [`lora::Ping`] while it has no fix, and hears every other node in
//!   range. A node configured as a repeater forwards what it hears.
//! - Keeps its radio config in its own flash, and reads it at boot.
//! - Serves the gps-proto GATT service, extended with telemetry, the
//!   remote-node roster, status lines and bulk transfer.
//! - Takes a radio config or a firmware image over BLE or the USB console.
//!
//! Pin assignments come from the board, not from preference:
//!
//! - D5 (GPIO43) and D2 (GPIO14) are **active low** - the LED anodes sit on
//!   +3V3 through R21/R20, so driving the pin low is what lights them. D5
//!   blinks on a LoRa receive, D2 on a transmit.
//! - GPIO43 is also UART0_TX, which is why the console is on the USB
//!   Serial/JTAG port (GPIO19/20 to the USB-C J3) instead. Expect the ROM
//!   bootloader's own log to flicker D5 on every reset.

#![no_std]
#![no_main]
// An isolation build leaves whole subsystems uncalled on purpose, and the
// warnings that produces are noise rather than signal.
#![cfg_attr(
    any(feature = "iso-no-ble", feature = "iso-no-app"),
    allow(dead_code, unused_imports, unused_variables, unused_mut)
)]

use embassy_executor::Spawner;
#[cfg(feature = "iso-no-ble")]
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::interconnect::{InputSignal, PeripheralInput};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::rtc_cntl::Rtc;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_println::println;
use midair_proto::ble::Mode;
use midair_proto::session;
#[cfg(feature = "dual-core")]
use static_cell::StaticCell;
use wio_s3_gps::gps::{Gps, BAUD as GPS_BAUD};
use wio_s3_gps::hardware::{self, LED_OFF};
use wio_s3_gps::radio::Sx1262Driver;
use wio_s3_gps::sx1262::Sx1262;
use wio_s3_gps::{flash, settings, state, status_println};

/// The SX1262 SPI clock. The chip takes up to 16 MHz; the bus here is
/// entirely inside the module, so this is conservative rather than tuned.
const LORA_SPI_HZ: u32 = 8_000_000;

/// Stack for the second core's executor thread. The hardware task's own
/// state lives in the task arena, but its future is built on this stack
/// before it is moved there, so the stack has to hold the whole of
/// `Hardware` once, beside the peripherals being moved into it; after that
/// it is what polling uses - the radio driver's frames and the console
/// formatting are the deep parts. 32 KiB overflowed at the spawn when the
/// constructor was an `async fn` holding the state twice.
#[cfg(feature = "dual-core")]
const APP_CORE_STACK: usize = 48 * 1024;

/// A value handed to the second core's start function, which has to be
/// `Send`.
///
/// The I2C driver behind the J5 panel is not, because it keeps a raw
/// pointer to its peripheral's state - a fact about the driver's shape,
/// not about which core may use it. Every peripheral moved this way is
/// used by exactly one task for the rest of the boot, on whichever core
/// that task runs, which is the condition `Send` is there to express.
#[cfg(feature = "dual-core")]
struct ToAppCore<T>(T);

#[cfg(feature = "dual-core")]
// SAFETY: see the type's doc: the value is moved once, to the one task
// that uses it, and never shared between cores.
unsafe impl<T> Send for ToAppCore<T> {}

#[cfg(feature = "dual-core")]
impl<T> ToAppCore<T> {
    /// Take the value back, on the core it was sent to.
    ///
    /// A method rather than a pattern on purpose: a closure that
    /// destructures the wrapper captures its fields one by one, and the
    /// wrapper's `Send` then covers none of them.
    fn into_inner(self) -> T {
        self.0
    }
}

/// Claim a pin as an SPI MISO that holds a level when nothing is driving it.
///
/// MISO is only driven while the peripheral's CS is low, which is a small
/// fraction of the time on both of this board's buses and none of it when
/// the SD slot is empty or the radio is asleep. The rest of the time the pad
/// floats, and esp-hal's `with_miso` is why it floats *and* has its input
/// buffer on: it applies `InputConfig::default()` (`Pull::None`)
/// unconditionally, overwriting anything configured beforehand. A floating
/// enabled input sits wherever leakage puts it, which can be mid-rail with
/// both halves of the buffer partly on - the same condition the parked
/// inputs below exist to remove, on two pins that parking cannot reach
/// because a peripheral already owns them.
///
/// `freeze` is the way through: a frozen signal makes the driver's own
/// `apply_input_config` a no-op, so the pull configured here survives being
/// handed to the SPI. Pulled up rather than down because idle-high is what
/// both an SD card's DO and the SX1262's MISO leave the line at.
fn miso_with_pullup<'d>(pin: impl PeripheralInput<'d>) -> InputSignal<'d> {
    let signal: InputSignal<'d> = pin.into();
    signal.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
    signal.set_input_enable(true);
    signal.freeze()
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // Not `CpuClock::max()`, which on the S3 is 240 MHz, and not ESP-IDF's
    // default of 160 either. Nothing here claims that headroom: the
    // hardware loop runs at 100 Hz, the GPS link is 9600 baud and the
    // radio sees one 8 MHz burst per beacon. The clock
    // is a standing cost the whole time the board is awake, and each step
    // down is on the order of 10 mA.
    //
    // 80 MHz is the floor rather than an arbitrary choice - esp-radio
    // refuses to start below it. Step back up to `_160MHz` if the BLE
    // controller misbehaves; that is the configuration it is validated at,
    // and this one is not.
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    // The idle task is substituted so the core's halted fraction is
    // measurable rather than assumed - see `idle`. The default hook does
    // the same `waiti`, just without counting it.
    esp_rtos::start_with_idle_hook(timg0.timer0, wio_s3_gps::idle::hook);

    // The hardware watchdog, armed here so it covers the boot as well as
    // everything after: a flash that does not answer, a second core that
    // never comes up, a controller that hangs in its init. From the moment
    // the monitor task runs it is fed only while every supervised task is
    // inside its bound. Its timeout is generous enough for the whole boot;
    // what it must never do is fire on a board that is merely busy.
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let mut wdt = timg1.wdt;
    wio_s3_gps::watchdog::arm(&mut wdt);
    // An isolation build leaves a loop out on purpose; a loop that never
    // starts must not read as one that stalled.
    #[cfg(feature = "iso-no-app")]
    wio_s3_gps::watchdog::unwatch(midair_proto::supervise::Task::Loop);
    #[cfg(feature = "iso-no-ble")]
    wio_s3_gps::watchdog::unwatch(midair_proto::supervise::Task::Serve);

    // D5 and D2. Both start dark so the first blink is visibly the
    // firmware's, not a leftover level from the ROM bootloader driving
    // UART0_TX - and so an unconfigured pin does not leave an LED biased
    // just under its forward voltage instead of held off.
    let d5 = Output::new(peripherals.GPIO43, LED_OFF, OutputConfig::default());
    let d2 = Output::new(peripherals.GPIO14, LED_OFF, OutputConfig::default());

    // Every pin the board brings out but the firmware does not use. A
    // CMOS input left floating sits wherever leakage puts it, which can be
    // mid-rail - both halves of the input buffer partly on, drawing current
    // and coupling noise into everything beside it. That is invisible at
    // 75 mA and is most of the budget in deep sleep.
    //
    // Pulled down rather than driven, because five of these leave the board:
    // GPIO38-41/47 on the J1 header. A pull is a defined state that still
    // yields to whatever a user wires up; an output would fight it. When a
    // real function claims one of these pins, it takes the pin from here -
    // as GPIO10 and GPIO11 have been, by the status display's I2C.
    let idle = InputConfig::default().with_pull(Pull::Down);
    let _parked = (
        // J1 header
        Input::new(peripherals.GPIO38, idle),
        Input::new(peripherals.GPIO39, idle),
        Input::new(peripherals.GPIO40, idle),
        Input::new(peripherals.GPIO41, idle),
        Input::new(peripherals.GPIO47, idle),
        // Module pads with nothing routed to them
        Input::new(peripherals.GPIO12, idle),
        Input::new(peripherals.GPIO13, idle),
        Input::new(peripherals.GPIO15, idle),
        Input::new(peripherals.GPIO16, idle),
        Input::new(peripherals.GPIO17, idle),
        Input::new(peripherals.GPIO18, idle),
        Input::new(peripherals.GPIO42, idle),
        Input::new(peripherals.GPIO48, idle),
    );
    // GPIO0 (BOOT) is deliberately absent: it has the board's pull-up and a
    // test point, and driving it would fight whoever holds it low to enter
    // the ROM loader. GPIO19/20 belong to the USB Serial/JTAG peripheral.

    println!("wio-s3-gps v{} up", env!("CARGO_PKG_VERSION"));

    // Was this a deep-sleep wake check, or a real boot? Read before
    // anything can clear it.
    let woke_from_sleep = matches!(
        esp_hal::system::wakeup_cause(),
        esp_hal::system::SleepSource::Timer
    );

    // Flash: the settings mirror and the OTA slots.
    flash::install(flash::Flash::new(peripherals.FLASH)).await;
    // Point ota-data at the running slot if it names none - the state
    // every USB flash leaves behind, and the one the slot arithmetic
    // cannot reason from.
    if flash::with_flash(|f| f.normalize_otadata()).await == Some(true) {
        println!("ota: ota-data initialized for the running slot");
    }
    // A freshly activated image is on probation until it says otherwise;
    // getting this far is the assertion. No-op on a board flashed with a
    // single-app partition table.
    if flash::with_flash(|f| f.confirm_boot()).await == Some(true) {
        println!("ota: running image confirmed");
    }
    // Booted and selected are read separately on purpose: the first comes
    // from the MMU and the second from ota-data, and a disagreement is the
    // bootloader having rolled an image back.
    match flash::with_flash(|f| (f.ota_available(), f.booted_slot(), f.selected_slot())).await {
        Some((true, booted, selected)) => {
            println!("ota: booted {:?}, ota-data selects {:?}", booted, selected);
            if booted != selected {
                println!("ota: WARNING - the bootloader did not run the selected image");
            }
        }
        _ => println!("ota: no update slots (single-app partition table)"),
    }
    // Only a deep-sleep wake keeps its RTC RAM copy; every other boot reads
    // flash and takes what it finds, so an erased flash is a board on
    // defaults rather than one that remembers its name through the erase.
    match settings::restore(woke_from_sleep).await {
        settings::Restored::Kept => {}
        settings::Restored::Flash(saved) => println!(
            "nvs: restored mode {}, sleep {} s, adv window {} s, flags {:#x}",
            saved.mode.as_str(),
            saved.sleep_interval_s,
            saved.adv_window(),
            saved.flags
        ),
        settings::Restored::Defaults { dropped_rtc } => println!(
            "nvs: nothing stored, settings are defaults{}",
            if dropped_rtc {
                " (an rtc copy from before the reset is dropped)"
            } else {
                ""
            }
        ),
    }
    // Why this boot happened and what the last one left - a panic, a
    // stall - go into the event log now, before anything that could fail
    // again is started, and the tail of the log goes to the console.
    wio_s3_gps::evlog::boot().await;

    // What this boot raises. Three flavors and one decision, taken here
    // because everything below - which peripherals are spoken to, whether
    // the config is read, what the serve loop budgets on - follows from it.
    //
    // The stored mode only ever says stored or tracking; idle is what a
    // cold boot turns "stored" into, so a board that has just been flashed,
    // or has just come back from a flat cell, is reachable for an idle
    // timeout before it stores itself. See `session::boot_mode`.
    let boot = session::boot_mode(settings::get().mode, woke_from_sleep);
    // The live mode, published so the serve loop, the settings
    // characteristic and the USB console all read the same answer. RTC RAM
    // only; idle never reaches flash.
    settings::set_mode(boot);
    // An isolation build exists to measure a board with everything running,
    // so it raises the tracker whatever the record says. Without this a
    // freshly flashed measurement board would come up idle - GPS parked,
    // radio down - and the reading would be of a different machine.
    #[cfg(any(feature = "iso-no-ble", feature = "iso-gps-backup"))]
    let boot = {
        let _ = boot;
        settings::set_mode(Mode::Tracking);
        Mode::Tracking
    };

    // Resolve and publish the BLE address before any task can be asked for
    // it over USB.
    let addr_bytes = wio_s3_gps::ble::address();
    state::set_ble_address(addr_bytes);

    if woke_from_sleep {
        // Counted, because a deep sleep is a full reset and from the console
        // a board that sleeps on its cadence and a board that resets in a
        // loop produce exactly the same boot banner. The wake number is what
        // tells them apart, and it is the first thing to read when the
        // question is whether sleep is working at all. The parks missed
        // beside it are the sleeps that cost more than they should have.
        let (n, asked_s) = settings::note_wake();
        status_println!(
            "woke from deep sleep #{} (slept {} s, parks missed {})",
            n,
            asked_s,
            settings::parks_missed()
        );
    } else {
        status_println!("cold boot (not a deep-sleep wake)");
    }
    status_println!(
        "mode {} - {}",
        boot.as_str(),
        match boot {
            Mode::Stored => "wake check, nothing raised",
            Mode::Idle => "reachable, gps in backup (CFG_MODE tracking to track)",
            Mode::Tracking => "gps and radio up",
            Mode::Listening => "gps and receiver up, nothing transmitted",
        }
    );

    // Everything below up to the `hardware_task` spawn is the application:
    // the LoRa radio, the GPS and the J5 panel. `iso-no-app`
    // drops the lot, which leaves BLE and the USB console - the closest
    // this board can get to the old two-MCU board's "ESP only, BLE
    // connected" reading. It does NOT power the GPS or the SX1262 down;
    // both are on the ungated +3V3 and keep running at their power-on
    // defaults.
    #[cfg(not(feature = "iso-no-app"))]
    {
    // Wio-S3 internal SX1262 wiring, from the module datasheet. Confirmed
    // against hardware - do not guess at these. An earlier build had three
    // of them wrong in a way that put an ESP push-pull output on a line the
    // SX1262 also drives, and it destroyed a board.
    //
    //   NSS  GPIO21    SCK  GPIO4    MOSI GPIO6    MISO GPIO5
    //   NRESET GPIO7   BUSY GPIO8    DIO1 GPIO9
    //   DIO2 goes to the SKY13453 RF switch inside the module, so it never
    //   reaches an ESP pin - it is the radio's own antenna control.
    let lora_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_hz(LORA_SPI_HZ))
            .with_mode(SpiMode::_0),
    )
    .expect("lora spi")
    .with_sck(peripherals.GPIO4)
    .with_mosi(peripherals.GPIO6)
    .with_miso(miso_with_pullup(peripherals.GPIO5));

    // BUSY and DIO1 both idle low, so a pull-down is the level they hold
    // anyway - and it is what makes an absent radio diagnosable. With no
    // pull, an unpowered or mis-wired SX1262 leaves BUSY floating, which
    // reads high as often as not and spends the driver's 50 ms busy timeout
    // on every single transaction. Pulled down it reads "not busy", the
    // transfer goes ahead, and the status byte comes back 0x00 - which is
    // exactly what `print_diagnostics` is written to recognize.
    let radio_irq_cfg = InputConfig::default().with_pull(Pull::Down);

    let lora = Sx1262Driver::new(Sx1262::new(
        lora_spi,
        // NSS and NRESET are ours to drive; BUSY and DIO1 are the radio's,
        // so they are inputs and nothing here may ever drive them.
        Output::new(peripherals.GPIO21, Level::High, OutputConfig::default()),
        Input::new(peripherals.GPIO8, radio_irq_cfg),
        Input::new(peripherals.GPIO9, radio_irq_cfg),
        Output::new(peripherals.GPIO7, Level::High, OutputConfig::default()),
    ));

    // Release the NSS pad hold that `enter_deep_sleep` set, now that the
    // pin has been reconfigured as the output that drives it.
    //
    // The hold outlives the sleep *and* the reset, which is the point - it
    // is what keeps NSS high while the digital domain is down, so a floating
    // edge cannot wake the SX1262 out of the sleep it was put into. The
    // order matters: reconfigure first, then release, or the pad glitches
    // through whatever state it had between the two. Unconditional because a cold boot's hold bit is
    // already clear, so releasing it is a write of the value it holds.
    unsafe {
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO21::steal(), false);
    }

    // GPS on UART1: GPIO1 is RX (module TX), GPIO2 is TX. 9600 8N1 is the
    // u-blox M10 factory default.
    //
    // The two halves go different ways. The receive half is an async
    // byte pump on this executor, which keeps draining the 128-byte FIFO
    // while the hardware loop is inside a transmit or a config apply; the
    // transmit half stays with the driver for the UBX commands.
    let gps_uart = Uart::new(
        peripherals.UART1,
        UartConfig::default().with_baudrate(GPS_BAUD),
    )
    .expect("gps uart")
    .with_rx(peripherals.GPIO1)
    .with_tx(peripherals.GPIO2);
    let (gps_rx, gps_tx) = gps_uart.split();
    let gps = Gps::new(gps_tx);
    spawner
        .spawn(wio_s3_gps::gps::pump(gps_rx.into_async()))
        .expect("spawn gps pump");

    // Release the TX pad hold that `enter_deep_sleep` set, in the same
    // order and for the same reason as NSS above: reconfigure first, then
    // release, or the pad glitches through whatever state it had in
    // between. UART RX activity is one of the M10's backup wake sources, so
    // an edge on this line while the digital domain is down is a receiver
    // that comes out of backup and acquires for the whole sleep interval.
    unsafe {
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO2::steal(), false);
    }

    // The microSD slot's four lines. The firmware does not drive the card
    // any more, so they are parked like the other unused pins: the three
    // the card drives or listens on pulled down, and its chip select
    // pulled up, which is a card deselected - a card in the slot then
    // leaves its data line in high impedance rather than driving it. All
    // three strapping pins among them are sampled at reset only, so a pull
    // after boot changes nothing about the next one.
    let _card_pins = (
        Input::new(peripherals.GPIO46, idle),
        Input::new(peripherals.GPIO45, idle),
        Input::new(peripherals.GPIO3, idle),
        Input::new(
            peripherals.GPIO44,
            InputConfig::default().with_pull(Pull::Up),
        ),
    );

    // The status display on J5. Optional hardware: a board with nothing on
    // that connector gets `None` and never mentions it again.
    //
    // Which of GPIO10/GPIO11 is SDA is not a board fact - the schematic
    // names those two nets `GPIO10` and `GPIO11` and nothing else - so both
    // orders are tried rather than one being picked and a reversed cable
    // looking like a dead panel. SDA on GPIO10 / SCL on GPIO11 is tried
    // first, so that is what a straight cable gets.
    let j5 = hardware::probe_j5(peripherals.I2C0).await;
    match &j5 {
        Some(j) => {
            match &j.oled {
                Some(o) => println!("oled: 128x32 at {:#04x}", o.address()),
                None => println!("oled: none on J5"),
            }
            match &j.compass {
                Some(c) => println!("compass: {} found", c.part().as_str()),
                None => println!("compass: none on J5"),
            }
        }
        None => println!("j5: nothing on the bus"),
    }

    // The hardware loop on the second core, with an executor of its own.
    //
    // Everything else - the BLE host, the USB console, the GPS byte pump
    // - stays on this one, beside the BLE controller's own thread. The
    // loop's blocking work is what this separates from the host: a
    // receive poll's SPI, a config apply's radio re-init, a panel
    // refresh, and on one core every millisecond of it was a millisecond
    // the host could not answer the phone or move a notification. The
    // other way round, the host's work
    // no longer lands inside the loop's 10 ms pass, which is what times a
    // received packet for the hop clock.
    //
    // What crosses between the cores is what crossed between the tasks
    // before: the `state` snapshot and its signals, all behind
    // critical sections, which on this chip are spinlocks that both cores
    // honor. Flash writes park the other core for their duration
    // (`FlashStorage::multicore_auto_park`), which is why the settings
    // and config saves are the rare events they are.
    #[cfg(feature = "dual-core")]
    {
        use esp_hal::interrupt::software::SoftwareInterruptControl;
        use esp_hal::system::Stack;

        static APP_STACK: StaticCell<Stack<APP_CORE_STACK>> = StaticCell::new();
        static APP_EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

        let sw = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let cold = !woke_from_sleep;
        let carried = ToAppCore((lora, gps, j5, d5, d2));
        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            sw.software_interrupt0,
            sw.software_interrupt1,
            APP_STACK.init(Stack::new()),
            move || {
                let (lora, gps, j5, d5, d2) = carried.into_inner();
                // The watchdog's warning lands on this core, so a first
                // core that has stopped cannot take it with it.
                wio_s3_gps::watchdog::warn_on_this_core();
                let executor = APP_EXECUTOR.init(esp_rtos::embassy::Executor::new());
                executor.run(|app| {
                    app.spawn(hardware::hardware_task(lora, gps, j5, d5, d2, boot, cold))
                        .expect("spawn hardware task");
                })
            },
        );
        println!(
            "hardware loop on the second core ({} B of state, {} KiB stack)",
            core::mem::size_of::<hardware::Hardware>(),
            APP_CORE_STACK / 1024
        );
    }
    #[cfg(not(feature = "dual-core"))]
    spawner
        .spawn(hardware::hardware_task(
            lora,
            gps,
            j5,
            d5,
            d2,
            boot,
            !woke_from_sleep,
        ))
        .expect("spawn hardware task");
    }

    // The LEDs are constructed above and stay at `LED_OFF`; without the
    // hardware task nothing owns them.
    #[cfg(feature = "iso-no-app")]
    let _ = (&d5, &d2);

    // The USB console: firmware text out, framed host commands in.
    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let (usb_rx, usb_tx) = usb.split();
    spawner
        .spawn(wio_s3_gps::usb::usb_task(usb_rx, usb_tx))
        .expect("spawn usb task");

    // The monitor: reads the heartbeats, feeds the watchdog while every
    // task is inside its bound, writes the event queue to flash, and
    // resets the board with the stall written down when a task is not.
    spawner
        .spawn(wio_s3_gps::watchdog::monitor_task(wdt))
        .expect("spawn monitor task");
    #[cfg(feature = "bench-hang")]
    spawner
        .spawn(wio_s3_gps::watchdog::bench_hang_task())
        .expect("spawn bench hang task");

    // `iso-no-ble` skips all of this: no `esp_radio::init`, so no PHY, no
    // controller and no advertising. The difference against the baseline is
    // what BLE actually costs on this board, which is the one number the
    // power investigation has never had.
    #[cfg(not(feature = "iso-no-ble"))]
    {
        println!("BLE-ADDR {}", wio_s3_gps::ble::fmt_address(&addr_bytes));
        let mut rtc = Rtc::new(peripherals.LPWR);
        // The first point after a wake where the RTC counter can be read.
        // Both readings go out, not just the difference: whether the counter
        // survives a deep sleep at all is the thing to confirm before
        // trusting any number derived from it, and two consecutive wakes
        // climbing is the confirmation.
        if woke_from_sleep {
            let now_ms = rtc.time_since_boot().as_millis() as u32;
            if let Some(stamp) = settings::sleep_stamp_ms() {
                let elapsed = now_ms.wrapping_sub(stamp);
                let asked_ms = settings::get().sleep_interval_s.saturating_mul(1000);
                status_println!(
                    "wake: rtc {} ms, slept from {} ms, elapsed {} ms over {} ms asked",
                    now_ms,
                    stamp,
                    elapsed,
                    asked_ms
                );
            }
        }
        wio_s3_gps::ble::duty_cycle(&mut rtc, addr_bytes).await
    }

    // Nothing left to do but hold the rail up and answer the console.
    #[cfg(feature = "iso-no-ble")]
    {
        status_println!("iso-no-ble: BLE stack not started");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
