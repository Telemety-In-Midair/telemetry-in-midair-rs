//! Wio-S3 firmware for the wio-s3-max-gps board.
//!
//! One module replaces the ESP32-C6 and WIO-E5 pair, so this binary holds
//! what both of them did: BLE, LoRa, GPS and SD. What it does *not* hold is
//! everything the split cost - there is no framed UART link, no heartbeat
//! proving it alive, no ack/retry around every command, and no second
//! firmware image to push. A config write that used to be a link frame and
//! a wait for an answer is now a signal the hardware loop picks up.
//!
//! - Reads a MAX-M10N on UART1 and folds NMEA into a position.
//! - Broadcasts that position over 915 MHz LoRa on the configured interval,
//!   or a [`lora::Ping`] while it has no fix, and hears every other node in
//!   range. A node configured as a repeater forwards what it hears.
//! - Logs own and remote positions to a FAT SD card, and reads `RADIO.CFG`
//!   from it at boot.
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

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either3};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::interconnect::{InputSignal, OutputSignal, PeripheralInput};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::rtc_cntl::sleep::TimerWakeupSource;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
// Renamed: `Mode` in this file is the board's mode, which appears on
// nearly every page of it.
use esp_hal::spi::Mode as SpiMode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_println::println;
use gps_proto::packet;
use midair_proto::bulk::{self, Owner};
use midair_proto::radiocfg::{self, RadioConfig};
use midair_proto::roster::{Report, Value};
use midair_proto::ble::{self, Mode};
use midair_proto::{link, lora, session};
use trouble_host::prelude::*;
use wio_s3_gps::gps::{Gps, BAUD as GPS_BAUD};
use wio_s3_gps::node::Node;
use wio_s3_gps::radio::Sx1262Driver;
use wio_s3_gps::sdlog::{SdLog, CONFIG_MAX};
use wio_s3_gps::state::{self, Request};
use wio_s3_gps::sx1262::Sx1262;
use wio_s3_gps::{flash, qprintln, settings, status_println, vprintln, xfer};

/// Status LEDs, cathodes on GPIO43 and GPIO14. Active low.
const LED_ON: Level = Level::Low;
const LED_OFF: Level = Level::High;

/// How long an activity LED stays lit for one packet.
const BLINK_MS: u32 = 20;

/// The SX1262 SPI clock. The chip takes up to 16 MHz; the bus here is
/// entirely inside the module, so this is conservative rather than tuned.
const LORA_SPI_HZ: u32 = 8_000_000;

/// SD card SPI clock. Cards must be initialized at 400 kHz or under, and
/// this never raises it afterwards - a flush is about a kilobyte every five
/// seconds, so the 25 ms it costs is not worth the reconfiguration.
const SD_SPI_HZ: u32 = 400_000;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

/// Build-time BLE address override, most-significant octet first (e.g.
/// "FF:C6:A1:53:50:47"). `Some` only when `BLE_ADDRESS` was set at build
/// time (build.rs validates and normalizes it, and emits nothing otherwise);
/// `None` derives a per-chip address from the eFuse MAC instead.
const BLE_ADDRESS_OVERRIDE: Option<&str> = option_env!("BLE_ADDRESS");

/// Claim a pin as an SPI MISO that holds a level when nothing is driving it.
///
/// MISO is only driven while the peripheral's CS is low, which is a small
/// fraction of the time on both of this board's buses and none of it when
/// the SD slot is empty or the radio is asleep. The rest of the time the pad
/// floats, and esp-hal's `with_miso` is why it floats *and* has its input
/// buffer on: it applies `InputConfig::default()` (`Pull::None`)
/// unconditionally, overwriting anything configured beforehand. A floating
/// enabled input sits wherever leakage puts it, which can be mid-rail with
/// both halves of the buffer partly on - the same condition the pin sweep in
/// `NOTES.md` went through the unrouted pads to remove, on two pins that
/// sweep could not reach because a peripheral already owned them.
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

/// `a` happened at or after deadline `b` in wrapping-u32 time.
fn due(now: u32, deadline: u32) -> bool {
    now.wrapping_sub(deadline) < 0x8000_0000
}

/// The board's BLE address as the LSB-first array `Address::random` expects.
///
/// With no build-time override it is derived from the chip's factory MAC in
/// eFuse, so every board is unique out of the box: the MAC's most-
/// significant octet gets its top two bits set, which is all a static-random
/// address requires, and the per-chip low bytes keep it distinct.
fn ble_address() -> [u8; 6] {
    match BLE_ADDRESS_OVERRIDE {
        Some(s) => {
            // build.rs has already validated the override, so a malformed
            // octet here can only be a bug; fall back to zero rather than
            // panicking on the device.
            let mut out = [0u8; 6];
            for (i, octet) in s.split(':').take(6).enumerate() {
                out[5 - i] = u8::from_str_radix(octet, 16).unwrap_or(0);
            }
            out
        }
        None => {
            // read_base_mac_address is MSB-first; reverse to LSB-first and
            // set the static-random bits on what becomes the MSB.
            let mac = esp_hal::efuse::Efuse::read_base_mac_address();
            let mut out = [0u8; 6];
            for i in 0..6 {
                out[i] = mac[5 - i];
            }
            out[5] |= 0xC0;
            out
        }
    }
}

/// Format an LSB-first address array as the MSB-first display string.
fn fmt_ble_address(a: &[u8; 6]) -> heapless::String<17> {
    use core::fmt::Write as _;
    let mut s = heapless::String::new();
    for i in (0..6).rev() {
        let _ = write!(s, "{:02X}", a[i]);
        if i != 0 {
            let _ = s.push(':');
        }
    }
    s
}

// ---------------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------------

#[gatt_server]
struct Server {
    gps: GpsService,
}

/// The gps-proto service, extended with the midair characteristics. UUIDs
/// come from the shared crates so firmware and app cannot drift - this is
/// the same service the C6 serves, so gps-gui-rs needs no changes.
#[gatt_service(uuid = packet::SERVICE_UUID_U128)]
struct GpsService {
    /// Position packets in the gps-proto wire format (local GPS).
    #[characteristic(uuid = packet::POSITION_UUID_U128, read, notify)]
    position: [u8; packet::POSITION_PACKET_LEN],
    /// Config commands: [id, len, value bytes].
    ///
    /// Sized from the protocol rather than from the longest numeric write,
    /// because one id carries a string: a name is `ble::NAME_LABEL_MAX`
    /// bytes behind its two-byte header, and a characteristic shorter than
    /// that would hand the policy a truncated label instead of refusing the
    /// write.
    #[characteristic(uuid = packet::CONFIG_UUID_U128, write)]
    config: heapless::Vec<u8, { ble::CONFIG_WRITE_MAX }>,
    /// Config/bulk acks: [id, status, applied value].
    #[characteristic(uuid = packet::ACK_UUID_U128, notify)]
    ack: [u8; packet::ACK_MAX_LEN],
    /// Radio telemetry in the midair-proto wire format.
    #[characteristic(uuid = ble::TELEMETRY_UUID_U128, read, notify)]
    telemetry: [u8; link::TELEMETRY_LEN],
    /// Bulk transfer ops: a radio config, or a firmware image.
    ///
    /// Sized from the protocol, so the attribute layer rejects an
    /// over-length write with an ATT error rather than accepting bytes the
    /// handler will refuse further in.
    #[characteristic(uuid = ble::BULK_UUID_U128, write)]
    bulk: heapless::Vec<u8, { ble::WRITE_MAX }>,
    /// Last remote position heard over LoRa.
    #[characteristic(uuid = ble::REMOTE_UUID_U128, read, notify)]
    remote: [u8; ble::REMOTE_LEN_V2],
    /// Last ping heard from a node with no fix.
    #[characteristic(uuid = ble::NODE_PING_UUID_U128, read, notify)]
    node_ping: [u8; ble::NODE_PING_LEN],
    /// Latest status/log line (ASCII text).
    ///
    /// The same bound the lines are built to. A characteristic smaller than
    /// that would not truncate them, it would drop them: `notify` fails on
    /// an over-length value and the logger arm has nowhere to report it.
    #[characteristic(uuid = ble::LOG_UUID_U128, read, notify)]
    log: heapless::Vec<u8, { link::LOG_MAX }>,
    /// Current power/sleep settings, so an app can populate its controls
    /// on connect instead of assuming defaults.
    #[characteristic(uuid = ble::SETTINGS_UUID_U128, read, notify)]
    settings: [u8; ble::SETTINGS_LEN],
    /// The current radio configuration, so an app can populate its radio
    /// editor from the board rather than a local file.
    #[characteristic(uuid = ble::RADIO_CONFIG_UUID_U128, read, notify)]
    radio_config: [u8; radiocfg::RADIO_CONFIG_LEN],
    /// What this board is called, as it advertises it.
    ///
    /// A connected app should not have to have kept the scan around to know
    /// which board it is talking to, and one that has just renamed a board
    /// sees the result here rather than waiting a window for the scan
    /// response to catch up.
    #[characteristic(uuid = ble::NAME_UUID_U128, read, notify)]
    name: heapless::Vec<u8, { ble::NAME_MAX }>,
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // Not `CpuClock::max()`, which on the S3 is 240 MHz, and not ESP-IDF's
    // default of 160 either. Nothing here claims that headroom: the
    // hardware loop runs at 100 Hz, the GPS link is 9600 baud, the SD bus
    // is 400 kHz and the radio sees one 8 MHz burst per beacon. The clock
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
    // Only a cold boot pays for the flash read - a wake check still holds
    // its RTC RAM copy.
    if let Some(saved) = settings::restore().await {
        println!(
            "nvs: restored mode {}, sleep {} s, adv window {} s, flags {:#x}",
            saved.mode.as_str(),
            saved.sleep_interval_s,
            saved.adv_window(),
            saved.flags
        );
    }

    // What this boot raises. Three flavors and one decision, taken here
    // because everything below - which peripherals are spoken to, whether
    // the card is mounted, what the serve loop budgets on - follows from it.
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
    let addr_bytes = ble_address();
    state::set_ble_address(addr_bytes);

    // Everything below up to the `hardware_task` spawn is the application:
    // the LoRa radio, the GPS, the card and the J5 panel. `iso-no-app`
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
    // SX1262 also drives, and it destroyed a board (see NOTES.md).
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
    // order is what the C6 firmware learned the hard way: reconfigure first,
    // then release, or the pad glitches through whatever state it had
    // between the two. Unconditional because a cold boot's hold bit is
    // already clear, so releasing it is a write of the value it holds.
    unsafe {
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO21::steal(), false);
    }

    // GPS on UART1: GPIO1 is RX (module TX), GPIO2 is TX. 9600 8N1 is the
    // u-blox M10 factory default.
    let gps_uart = Uart::new(
        peripherals.UART1,
        UartConfig::default().with_baudrate(GPS_BAUD),
    )
    .expect("gps uart")
    .with_rx(peripherals.GPIO1)
    .with_tx(peripherals.GPIO2);
    let gps = Gps::new(gps_uart);

    // Release the TX pad hold that `enter_deep_sleep` set, in the same
    // order and for the same reason as NSS above: reconfigure first, then
    // release, or the pad glitches through whatever state it had in
    // between. UART RX activity is one of the M10's backup wake sources, so
    // an edge on this line while the digital domain is down is a receiver
    // that comes out of backup and acquires for the whole sleep interval.
    unsafe {
        esp_hal::gpio::RtcPin::rtcio_pad_hold(&esp_hal::peripherals::GPIO2::steal(), false);
    }

    // microSD on SPI3. Three of these four lines are ESP32-S3 strapping
    // pins - see BOARD-REVIEW.md in the board repo; R17 on GPIO45 is DNP
    // for that reason.
    let sd_spi = Spi::new(
        peripherals.SPI3,
        SpiConfig::default()
            .with_frequency(Rate::from_hz(SD_SPI_HZ))
            .with_mode(SpiMode::_0),
    )
    .expect("sd spi")
    .with_sck(peripherals.GPIO46)
    .with_mosi(peripherals.GPIO45)
    .with_miso(miso_with_pullup(peripherals.GPIO3));
    let sd_cs = Output::new(peripherals.GPIO44, Level::High, OutputConfig::default());
    let sd_dev = embedded_hal_bus::spi::ExclusiveDevice::new(sd_spi, sd_cs, Delay::new())
        .expect("sd spi device");
    let sdlog = SdLog::new(embedded_sdmmc::SdCard::new(sd_dev, Delay::new()));

    // The status display on J5. Optional hardware: a board with nothing on
    // that connector gets `None` and never mentions it again.
    //
    // Which of GPIO10/GPIO11 is SDA is not a board fact - the schematic
    // names those two nets `GPIO10` and `GPIO11` and nothing else - so both
    // orders are tried rather than one being picked and a reversed cable
    // looking like a dead panel. SDA on GPIO10 / SCL on GPIO11 is tried
    // first, so that is what a straight cable gets.
    let j5 = probe_j5(peripherals.I2C0).await;
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

    spawner
        .spawn(hardware_task(
            lora,
            gps,
            sdlog,
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

    // `iso-no-ble` skips all of this: no `esp_radio::init`, so no PHY, no
    // controller and no advertising. The difference against the baseline is
    // what BLE actually costs on this board, which is the one number the
    // power investigation has never had.
    #[cfg(not(feature = "iso-no-ble"))]
    {
    println!("BLE-ADDR {}", fmt_ble_address(&addr_bytes));

    let mut rtc = Rtc::new(peripherals.LPWR);
    if woke_from_sleep {
        // Counted, because a deep sleep is a full reset and from the console
        // a board that sleeps on its cadence and a board that resets in a
        // loop produce exactly the same boot banner. The wake number is what
        // tells them apart, and it is the first thing to read when the
        // question is whether sleep is working at all.
        let (n, asked_s) = settings::note_wake();
        status_println!("woke from deep sleep #{} (slept {} s)", n, asked_s);
    } else {
        status_println!("cold boot (not a deep-sleep wake)");
    }
    status_println!(
        "mode {} - {}",
        boot.as_str(),
        match boot {
            Mode::Stored => "wake check, nothing raised",
            Mode::Idle => "reachable, gps in backup (CFG_MODE tracking to track)",
            Mode::Tracking => "gps, radio and card up",
        }
    );

    // The attribute table is built once and reused by every window.
    //
    // Not an optimization - a requirement. The `#[gatt_service]` macro backs
    // each characteristic with its own `static StaticCell`, so a second
    // `Server::new_with_config` panics ("already full, it can't be
    // initialized twice") rather than returning an error. It borrows nothing
    // from the per-window stack, so hoisting it costs nothing either.
    // The GAP device name is the one surface that cannot follow a rename:
    // the attribute table is built once (see above) and this string is
    // copied into it. A board renamed while running therefore advertises
    // and reports its new name immediately - the scan response and the name
    // characteristic are both rebuilt - and only the GAP characteristic,
    // which nothing here reads, catches up at the next boot.
    let boot_name = settings::name();
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: &boot_name,
        appearance: &appearance::sensor::GENERIC_SENSOR,
    }))
    .expect("gatt server");

    // The BLE duty cycle.
    //
    // BLE measures 71 mA of this board's 126. Two things cut into that, and
    // they cut into different parts of it.
    //
    // Modem sleep, set below, is the controller powering its own PHY down
    // in the gaps it knows about - between advertisements, and between the
    // connection events of a connection with nothing to say. It costs
    // nothing in reachability, because the controller is still counting and
    // still wakes for every event it promised. It cannot help while a
    // transfer is actually moving.
    //
    // The loop is the other one, and the only way to zero: `BleConnector`'s
    // `Drop` calls `ble_deinit` and takes the `PhyInitGuard` with it, so
    // the whole stack is built inside this loop and dropped at the bottom
    // of it. That does cost reachability - a board in a dark period cannot
    // be connected to at all.
    //
    // `serve` returns rather than advertising forever once the window is
    // spent and `ble_off_s` is set; with it at 0 it never returns and this
    // is the single pass the firmware always did.
    //
    // Unlike deep sleep this stops nothing else. The hardware task keeps
    // beaconing, the GPS keeps tracking and the card keeps logging - the
    // board stays a working tracker and only stops being connectable.
    let mut announced_modem_sleep = false;
    loop {
        {
            // TX power is 0 dBm rather than the +9 dBm default. Nine buys
            // nothing here: the module's 2.4 GHz pin goes to a test point
            // and stops, so range is whatever the stub couples either way,
            // and a +9 dBm burst is the worst current spike to put beside a
            // LoRa PA that can be keying 22 dBm at the same moment - the
            // conflict `state::radio_busy` exists to keep apart.
            //
            // Modem sleep is a local patch to the vendored esp-radio, which
            // ships it unimplemented; the sleep clock is left at its
            // default of the main crystal, which is the only low power
            // clock this board has (there is no 32.768 kHz part).
            let ble_config = esp_radio::ble::Config::default()
                .with_default_tx_power(esp_radio::ble::TxPower::N0)
                .with_modem_sleep(!cfg!(feature = "iso-ble-no-modem-sleep"));

            // SAFETY: the previous iteration's `BleConnector` was dropped at
            // the closing brace below, and nothing outside this block ever
            // holds `BT`. The first pass steals a peripheral that `main`
            // still owns and has never used, which is the same trade the
            // J5 probe makes.
            let bt = unsafe { esp_hal::peripherals::BT::steal() };

            // Declared before the stack so it outlives it: dropping the
            // connector needs the radio still initialized, and locals drop
            // in reverse declaration order.
            //
            // Neither of these may panic. They ran once per boot before the
            // duty cycle and now run every window - thousands of times a
            // day on a deployed board - so a transient failure has to cost
            // one window rather than the whole node. A tracker that stops
            // beaconing because its BLE modem would not come back up is a
            // worse outcome than one nobody can connect to.
            let radio = match esp_radio::init() {
                Ok(radio) => radio,
                Err(e) => {
                    qprintln!("radio init failed ({:?}), retrying", e);
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            };
            let transport =
                match esp_radio::ble::controller::BleConnector::new(&radio, bt, ble_config) {
                    Ok(t) => t,
                    Err(e) => {
                        qprintln!("ble connector failed ({:?}), retrying", e);
                        Timer::after(Duration::from_secs(1)).await;
                        continue;
                    }
                };
            // Said once, and from the crate rather than from the build
            // flags: this is what the controller was actually set up with.
            // A board that is quietly advertising with its PHY up all the
            // time looks exactly like one that is not, until a meter says
            // otherwise, and this is the cheaper way to ask.
            if !core::mem::replace(&mut announced_modem_sleep, true) {
                status_println!(
                    "ble modem sleep {}",
                    if esp_radio::ble::modem_sleep_active() {
                        "on"
                    } else {
                        "off"
                    }
                );
            }

            let controller = ExternalController::<_, 20>::new(transport);

            let mut resources: HostResources<
                DefaultPacketPool,
                CONNECTIONS_MAX,
                L2CAP_CHANNELS_MAX,
            > = HostResources::new();
            let stack = trouble_host::new(controller, &mut resources)
                .set_random_address(Address::random(addr_bytes));
            let Host {
                mut peripheral,
                mut runner,
                ..
            } = stack.build();

            // Seed the readable value so a central that reads immediately
            // after discovery cannot beat the first publish in
            // `gatt_session`. Re-seeded per window because the settings may
            // have moved while the modem was down.
            let _ = server.gps.settings.set(&server, &current_settings().encode());
            let _ = server.gps.name.set(&server, &name_value());

            let _ = select(
                async {
                    loop {
                        if runner.run().await.is_err() {
                            qprintln!("ble host error, restarting");
                            Timer::after(Duration::from_millis(200)).await;
                        }
                    }
                },
                serve(&mut peripheral, &server, &mut rtc),
            )
            .await;
        }

        // Everything above is dropped by here, `ble_deinit` included.
        let session::Next::BleDown { off_s } = settings::get().at_expiry() else {
            // `serve` only returns for a BLE-down period, so anything else
            // is a setting - or a mode - that moved while the modem was
            // coming down. Go straight back to advertising rather than
            // spinning on a zero-length wait.
            continue;
        };
        status_println!("ble down for {} s (lora and gps stay up)", off_s);
        // A board told to sleep - or told to change mode - over the USB
        // console must not have to wait out the whole dark period first.
        // The console is alive throughout, and while the modem is down it is
        // the only way in.
        let commanded = async { state::SLEEP_NOW_SIGNAL.wait().await };
        let remoded = async { state::MODE_SIGNAL.wait().await };
        match select3(
            Timer::after(Duration::from_secs(off_s as u64)),
            commanded,
            remoded,
        )
        .await
        {
            Either3::Second(()) => {
                let secs = state::take_sleep_now()
                    .unwrap_or_else(|| ble::resolve_sleep_now(0, settings::get().sleep_interval_s));
                status_println!("sleep on command: {} s, from a BLE-down period", secs);
                enter_deep_sleep(&mut rtc, secs).await;
            }
            // A mode that is no longer tracking has no BLE-down period to
            // sit out. Bring the modem straight back up; `serve` re-budgets
            // on the new mode.
            Either3::Third(()) => status_println!("mode changed, ending the BLE-down period early"),
            Either3::First(()) => {}
        }
        // Free heap alongside it, because the duty cycle turned a
        // once-per-boot allocation into a few thousand a day: the whole
        // trouble-host stack and the controller's queues are built and torn
        // down every window. A slow drift here across a soak is
        // fragmentation, and it is the failure this change is most likely
        // to introduce - it will not show up in a single cycle.
        qprintln!("ble back up ({} B heap free)", esp_alloc::HEAP.free());
    }
    }

    // The duty-cycle loop above never exits, so this is an `iso-no-ble`
    // build with nothing left to do but hold the rail up and answer the
    // console.
    #[cfg(feature = "iso-no-ble")]
    {
        status_println!("iso-no-ble: BLE stack not started");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}

/// Everything found on the J5 I2C bus, and the bus itself.
///
/// One bus, up to two devices, and the hardware loop owns all three - which
/// is what makes it safe for the display and the magnetometer to share a
/// controller with no arbitration: there is exactly one caller.
pub struct J5 {
    pub i2c: esp_hal::i2c::master::I2c<'static, esp_hal::Async>,
    pub oled: Option<wio_s3_gps::oled::Oled>,
    pub compass: Option<wio_s3_gps::compass::Compass>,
}

/// Try both SDA/SCL orders on J5 and return whichever finds a panel.
///
/// SDA on GPIO10 and SCL on GPIO11 goes first; the reverse is the fallback.
///
/// The I2C peripheral and the two pins are consumed by each attempt, so the
/// retry steals the singletons back. That is sound here and only here: this
/// runs once, before anything else has been handed either pin, and the
/// `I2c` from the failed attempt is dropped before the next is built.
async fn probe_j5(i2c0: esp_hal::peripherals::I2C0<'static>) -> Option<J5> {
    use esp_hal::i2c::master::{Config as I2cConfig, I2c};

    // 400 kHz: a 512-byte frame is about 11 ms of bus time at 400 kHz
    // against 44 ms at 100 kHz, and the refresh has to fit between radio
    // polls. Every SSD1306 module takes fast mode.
    let config = I2cConfig::default().with_frequency(Rate::from_khz(400));

    for swapped in [false, true] {
        // SAFETY: see the note above - single-threaded init, one live
        // borrow of each pin at a time.
        let (sda, scl): (OutputSignal<'static>, OutputSignal<'static>) = unsafe {
            if swapped {
                (
                    esp_hal::peripherals::GPIO11::steal().into(),
                    esp_hal::peripherals::GPIO10::steal().into(),
                )
            } else {
                (
                    esp_hal::peripherals::GPIO10::steal().into(),
                    esp_hal::peripherals::GPIO11::steal().into(),
                )
            }
        };
        let mut i2c = match I2c::new(unsafe { i2c0.clone_unchecked() }, config) {
            Ok(i2c) => i2c.with_sda(sda).with_scl(scl).into_async(),
            Err(_) => return None,
        };
        let oled = wio_s3_gps::oled::Oled::probe(&mut i2c).await;
        let compass = wio_s3_gps::compass::Compass::probe(&mut i2c).await;
        // Either device answering settles the wiring, so a board with a
        // magnetometer and no display still gets the right pin order.
        if oled.is_some() || compass.is_some() {
            if swapped {
                println!("j5: SDA/SCL swapped (SDA on GPIO11)");
            }
            return Some(J5 { i2c, oled, compass });
        }
    }
    None
}

/// The settings characteristic value: what an app reads on connect.
fn current_settings() -> ble::Settings {
    settings::get().settings(state::notify_interval_ms())
}

/// Park what a sleeping board cannot use, then deep sleep.
///
/// There is no rail to cut on this board - the GPS and SD sit directly on
/// +3V3 - so the radio is the one load the firmware can actually drop, and
/// dropping it is worth doing: continuous RX is 5.5 mA against the module's
/// 9.3 uA asleep. The hardware loop owns the radio, so this asks and waits
/// rather than reaching for it.
///
/// The GPS goes with it. `PrepareSleep` puts the receiver into backup and
/// holds the UART TX pad across the sleep, which together are what make a
/// sleeping board actually cheap: an M10 left acquiring is around 30 mA
/// against a chip that is otherwise in microamps, and a sleeping S3 cannot
/// use a fix anyway.
async fn enter_deep_sleep(rtc: &mut Rtc<'_>, interval_s: u32) -> ! {
    // A command that got this far has been acted on; nothing on the far
    // side of the sleep should find it still pending and sleep again.
    state::clear_sleep_now();
    state::SLEEP_READY.reset();
    state::request(Request::PrepareSleep);
    // The loop notices within its 10 ms pass *unless* it is inside a
    // transmit, which awaits for the length of the frame on air - 289 ms at
    // the SF12/BW500 default and up to about 9.7 s at the slowest settings
    // the config accepts. The old budget here was one second, so a sleep
    // that landed during a beacon timed out and slept with the SX1262 still
    // in continuous receive: 5.7 mA for the whole interval, and nothing
    // visible afterwards because the wake re-inits the radio anyway. The
    // hardware loop also declines to start a beacon while a sleep is
    // pending, so this only has to cover one already in flight.
    //
    // The second on top of that covers the park itself. It was 500 ms,
    // which is enough for the bounded parts of the sequence and not for the
    // card, whose flush and unmount can stall on wear levelling - and an
    // expiry there was enough to sleep over a park that had not finished.
    // Nothing is spent in the ordinary case: this is a timeout rather than
    // a delay, so the wait ends when the park does.
    let park = Duration::from_millis(u64::from(state::tx_worst_case_ms()) + 1_500);
    if with_timeout(park, state::SLEEP_READY.wait()).await.is_err() {
        // Worth saying: it means the sleep is about to cost more than it
        // should, and it is otherwise undetectable from the far side.
        //
        // What is still awake is whatever the hardware task had not reached.
        // `PrepareSleep` takes the receiver, the radio and the panel down
        // before it touches the card, so an expiry here is most likely the
        // card alone - which costs buffered log lines rather than current.
        // The expensive shape is a loop still inside a transmit that outran
        // the budget above, because that one reaches none of it.
        println!("sleep: park did not finish in time, sleeping over it");
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
    // singleton is stolen for the hold exactly as the C6 firmware does for
    // its rail and reset lines. Nothing races: the driver is parked and this
    // function does not return.
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

/// Advertise, accept one central, serve it, repeat.
///
/// What a spent budget means is a property of the mode, not a race between
/// two settings ([`session::Stored::at_expiry`]):
///
/// - **Stored** - a wake check. The budget is the advertising window, and
///   spending it is deep sleep again. Does not return.
/// - **Idle** - the budget is the idle timeout, and spending it stores the
///   board. Does not return.
/// - **Tracking** - the budget is the advertising window and spending it
///   returns, so the caller can drop the BLE stack for `ble_off_s` and call
///   this again. The board stays a working tracker throughout.
///
/// With no cadence to sleep on (`sleep_interval_s = 0`) the first two
/// simply keep advertising, which is what a bench board and an unconfigured
/// board both want.
///
/// The budget is a deadline rather than a per-attempt timeout, so no retry
/// path below can extend it - see [`session::Window`].
async fn serve<C: Controller>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    server: &Server<'_>,
    rtc: &mut Rtc<'_>,
) {
    let mut adv_data = [0u8; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids128(&[packet::SERVICE_UUID_U128.to_le_bytes()]),
        ],
        &mut adv_data,
    )
    .expect("adv data fits");
    let mut scan_data = [0u8; 31];

    let mut mode = settings::get().mode;
    let mut window = session::Window::new(Instant::now().as_millis(), settings::get().budget_s());

    loop {
        // Consumed before the read it stands for, so a write that lands
        // between the two is still pending on the next pass rather than
        // reset unseen.
        state::MODE_SIGNAL.reset();
        let stored = settings::get();
        // A mode that moved under the loop budgets differently - a wake
        // check's window and an idle timeout are the same deadline field
        // holding two very different numbers - so the budget restarts on the
        // new mode's terms rather than carrying the old one's deadline into
        // it.
        if stored.mode != mode {
            mode = stored.mode;
            window = session::Window::new(Instant::now().as_millis(), stored.budget_s());
            status_println!("mode {} ({} s budget)", mode.as_str(), stored.budget_s());
        }
        // Budget spent, whatever used it up.
        match window.next(Instant::now().as_millis(), &stored) {
            session::Next::Sleep { interval_s } => enter_deep_sleep(rtc, interval_s).await,
            // Cheaper exit than deep sleep and it stops nothing else: hand
            // control back so the caller can drop the stack. The board stays
            // awake and on the air over LoRa; only the modem goes.
            session::Next::BleDown { .. } => return,
            session::Next::Advertise => {}
        }
        // Built here rather than once above, so a board renamed during the
        // last connection advertises under the new name from this window on
        // rather than at the next boot.
        let name = settings::name();
        let scan_len = AdStructure::encode_slice(
            &[AdStructure::CompleteLocalName(name.as_bytes())],
            &mut scan_data,
        )
        .expect("scan data fits");
        qprintln!("advertising as {}", name);
        let advertiser = match peripheral
            .advertise(
                &AdvertisementParameters::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..adv_len],
                    scan_data: &scan_data[..scan_len],
                },
            )
            .await
        {
            Ok(a) => a,
            Err(_) => {
                qprintln!("advertise failed, retrying");
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };

        // Waiting for a central is also waiting for a `SLEEP_NOW`, which is
        // how a board told to sleep over the USB console with nobody
        // connected goes down without first having to be connected to. The
        // BLE path signals this too, but from inside a session, where
        // `gatt_session` is the arm that picks it up.
        //
        // `bounded` is whether the budget ends in anything. It does not for a board
        // that advertises forever - a bench board, or an unconfigured one -
        // and waiting on `accept` with no deadline is then the whole
        // intent. Derived from `at_expiry` rather than from the settings
        // directly, so a spent budget that resolves to "keep advertising"
        // cannot arm a zero-length timeout and spin.
        let bounded = stored.at_expiry() != session::Next::Advertise;
        let accepted = {
            let commanded = async {
                state::SLEEP_NOW_SIGNAL.wait().await;
            };
            // A mode written over USB or BLE while nothing is connected
            // would otherwise wait out the whole budget - which in idle is
            // ten minutes - before the loop noticed.
            let remoded = async {
                state::MODE_SIGNAL.wait().await;
            };
            let accept = async {
                if bounded {
                    let left =
                        Duration::from_millis(window.remaining_ms(Instant::now().as_millis()));
                    // `None` is the window expiring with nobody interested.
                    with_timeout(left, advertiser.accept()).await.ok()
                } else {
                    Some(advertiser.accept().await)
                }
            };
            select3(accept, commanded, remoded).await
        };

        // A connect during a wake check is a doorbell, not a leash.
        //
        // Before this, a held session was the only thing that kept a stored
        // board up: reaching one meant catching the window, connecting, and
        // then not letting go. Now the attempt alone promotes the board to
        // idle with the timeout armed, and the app can take its time -
        // including reconnecting after a handshake that fizzled, which
        // phones do routinely.
        //
        // On the attempt rather than on a completed session, because the
        // intent was unambiguous either way and the failure mode of the
        // stricter rule is a board that goes back down for five minutes
        // over one bad handshake. A promotion that turns out to be a
        // misfire costs one idle timeout of awake current, bounded and
        // small.
        if mode == Mode::Stored && matches!(accepted, Either3::First(Some(_))) {
            mode = Mode::Idle;
            settings::set_mode(mode);
            // Raises what a wake check left down: the card, so config reads
            // and log pulls work. The GPS and the radio stay parked - an
            // app looking at a stored object's settings should not cost an
            // acquisition.
            state::request(Request::Mode(mode));
            let stored = settings::get();
            window = session::Window::new(Instant::now().as_millis(), stored.budget_s());
            status_println!(
                "promoted to idle by a connect ({} s before it stores itself)",
                stored.budget_s()
            );
        }

        let conn = match accepted {
            Either3::First(Some(Ok(c))) => c,
            Either3::First(Some(Err(_))) => {
                // A central started a connection and it did not complete.
                // The pause keeps a repeated failure off a hot spin.
                //
                // The window is held open for the retry rather than left to
                // run out: a phone that fizzled a handshake comes back
                // within a second or two, and before this it could find the
                // board dark for `ble_off_s` or asleep for a whole cadence,
                // having tried at the wrong moment in a 15 s window.
                qprintln!("connect attempt failed, holding the window open");
                window.after_connect_attempt(Instant::now().as_millis());
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            // The budget ran out with nobody interested, so whatever this
            // mode does with a spent budget is what happens now.
            Either3::First(None) => match stored.at_expiry() {
                session::Next::Sleep { interval_s } => enter_deep_sleep(rtc, interval_s).await,
                _ => {
                    qprintln!("advertising window over");
                    return;
                }
            },
            // Told to sleep while advertising to nobody.
            Either3::Second(()) => {
                // The cell is always set before the signal is raised, so the
                // fallback is for a shape that should not occur - and it
                // resolves the same way a request of 0 would rather than
                // inventing a duration nothing else in the system uses.
                let secs = state::take_sleep_now()
                    .unwrap_or_else(|| ble::resolve_sleep_now(0, stored.sleep_interval_s));
                status_println!("sleep on command: {} s, from advertising", secs);
                enter_deep_sleep(rtc, secs).await
            }
            // A mode write. The top of the loop re-budgets on it.
            Either3::Third(()) => continue,
        };

        let Ok(conn) = conn.with_attribute_server(server) else {
            // A central connected and the attribute server did not attach -
            // the link dropped in between, or the stack is out of room. The
            // window is held open for the same reason a fizzled handshake
            // holds it: something was connecting, and letting the budget
            // expire here sends the board dark or asleep on a phone that is
            // about to try again.
            window.after_connect_attempt(Instant::now().as_millis());
            continue;
        };
        qprintln!("central connected");
        gatt_session(&conn, server).await;
        qprintln!("central disconnected");

        // A transfer the phone was midway through does not outlive it.
        xfer::abort(Owner::Ble, Instant::now().as_millis()).await;

        // The session may have ended because the app asked for a sleep
        // rather than because the phone went away. Checked here rather than
        // inside `gatt_session` so the ack, the settings republish and the
        // link teardown have all already happened - the board is gone the
        // moment this runs, and anything still owed to the central has to
        // have left first.
        if let Some(secs) = state::take_sleep_now() {
            status_println!("sleep on command: {} s", secs);
            enter_deep_sleep(rtc, secs).await;
        }

        // Re-arm by advertising, not by idling. The point is to let the
        // phone come straight back, which it cannot do if the board is
        // awake but not discoverable. Looping back re-advertises, and the
        // deadline check at the top sends the board wherever its mode sends
        // it when the budget runs out - a five second linger while
        // tracking, the whole timeout again while idle.
        window.after_disconnect(Instant::now().as_millis(), &settings::get());
    }
}

/// Refresh the settings characteristic and notify the central. Covers
/// changes the device makes on its own (a clamped interval, settings
/// restored from flash), not just ones the app asked for.
async fn publish_settings<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let value = current_settings().encode();
    if server.gps.settings.set(server, &value).is_err() {
        return;
    }
    // An unsubscribed central is not an error - the value stays readable.
    let _ = server.gps.settings.notify(conn, &value).await;
}

/// The name characteristic's value: what the board advertises under, as
/// the characteristic holds it.
fn name_value() -> heapless::Vec<u8, { ble::NAME_MAX }> {
    let name = settings::name();
    // Built to fit by construction - both buffers are `ble::NAME_MAX`.
    heapless::Vec::from_slice(name.as_bytes()).unwrap_or_default()
}

/// Refresh the name characteristic and notify the central. Called on
/// connect and after a config write, which is the only thing that renames a
/// board.
async fn publish_name<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let value = name_value();
    if server.gps.name.set(server, &value).is_err() {
        return;
    }
    let _ = server.gps.name.notify(conn, &value).await;
}

/// Refresh the radio-config characteristic. A no-op until the radio has
/// been configured, so the characteristic never carries the all-zero
/// placeholder as if it were a real config.
async fn publish_radio_config<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let Some(value) = state::radio_config() else {
        return;
    };
    if server.gps.radio_config.set(server, &value).is_err() {
        return;
    }
    let _ = server.gps.radio_config.notify(conn, &value).await;
}

/// Which characteristic a write landed on, sampled before the reply goes
/// out so the work can happen after it.
enum Wrote {
    Config,
    Bulk,
    Other,
}

/// One connection: publish what an app needs on arrival, then stream.
async fn gatt_session<P: PacketPool>(conn: &GattConnection<'_, '_, P>, server: &Server<'_>) {
    // Drop status lines buffered while disconnected so the central sees
    // live events, not a stale backlog.
    state::drain_log();
    // Cleared here rather than reacted to: the publish below covers
    // whatever is cached now.
    state::RADIO_CONFIG_SIGNAL.reset();

    // Publish before anything else, so an app can populate its controls
    // without waiting for a notify interval.
    publish_settings(server, conn).await;
    publish_name(server, conn).await;
    publish_radio_config(server, conn).await;

    // Hand the new central every node heard from recently. Their ages go
    // out with them, so a report from before this connection cannot be
    // read as a live one.
    state::replay_remotes(Instant::now().as_millis());

    let events = async {
        loop {
            match conn.next().await {
                GattConnectionEvent::Disconnected { .. } => break,
                GattConnectionEvent::Gatt { event } => {
                    // Copy the write out and let the attribute server finish
                    // the transaction before doing any of the work: a config
                    // apply reaches the SD card and a firmware chunk erases
                    // a flash sector, either of which is long enough that a
                    // central would otherwise see its write time out.
                    //
                    // Sized from the protocol rather than rounded: the
                    // longest write it defines is a bulk chunk behind its
                    // three-byte header. A buffer that merely fits today
                    // truncates silently the moment `BULK_DATA_MAX` moves,
                    // and a truncated chunk fails as a CRC error at the end
                    // of the transfer rather than as a write that was too
                    // long.
                    let mut data = [0u8; ble::WRITE_MAX];
                    let mut len = 0usize;
                    let mut wrote = Wrote::Other;
                    if let GattEvent::Write(w) = &event {
                        let d = w.data();
                        if d.len() > data.len() {
                            // Refused rather than clipped, and left as
                            // `Wrote::Other` so no handler sees half a
                            // frame. The central still gets its write
                            // acknowledged below - the transaction belongs
                            // to the attribute server - and then hears
                            // nothing back, which is what a clipped chunk
                            // did too, only silently.
                            qprintln!("ble write too long ({} B), ignored", d.len());
                        } else {
                            len = d.len();
                            data[..len].copy_from_slice(&d[..len]);
                            wrote = if w.handle() == server.gps.config.handle {
                                Wrote::Config
                            } else if w.handle() == server.gps.bulk.handle {
                                Wrote::Bulk
                            } else {
                                Wrote::Other
                            };
                        }
                    }
                    if let Ok(reply) = event.accept() {
                        reply.send().await;
                    }
                    match wrote {
                        Wrote::Config => {
                            let (ack, _) = wio_s3_gps::config::apply_config(&data[..len]).await;
                            let _ = server.gps.ack.notify(conn, &ack).await;
                            // The write may have changed something the
                            // settings characteristic reports, including
                            // through clamping.
                            publish_settings(server, conn).await;
                            // A rename is the one settings change the blob
                            // above does not carry: a name is a string, and
                            // the ack could only afford its length.
                            publish_name(server, conn).await;
                        }
                        Wrote::Bulk => {
                            let (ack, _) =
                                xfer::handle(Owner::Ble, Instant::now().as_millis(), &data[..len])
                                    .await;
                            let _ = server.gps.ack.notify(conn, &ack).await;
                        }
                        Wrote::Other => {}
                    }
                }
                _ => {}
            }
        }
    };

    let notifier = async {
        let mut next_notify = Instant::now();
        loop {
            Timer::at(next_notify).await;
            next_notify += Duration::from_millis(u64::from(state::notify_interval_ms()));
            // A tick that took longer than the interval - a slow notify, or
            // a shortened interval - would otherwise leave the deadline in
            // the past, and `Timer::at` on a past instant returns at once.
            // That is a spin, not a catch-up.
            next_notify = next_notify.max(Instant::now());

            // Hold notifications while the radio has the air. A LoRa
            // transmit at 22 dBm beside a 2.4 GHz radio is a supply
            // problem; on the two-MCU board this was a `RADIO_BUSY` link
            // message, here it is a bool.
            if state::radio_busy() {
                continue;
            }

            // `set` before each notify, so both of these answer a plain read
            // as well. They are declared `read`, and a central that never
            // subscribed would otherwise get the table's initial zeros - which
            // decode as a report rather than as nothing, since a telemetry
            // `secs_since_rx` of 0 reads as "just heard from" and not as
            // "never". The attribute table outlives the connection, so what is
            // written here is also what the next window opens holding.
            //
            // A failed `set` is not a failed link, so unlike a failed position
            // notify it does not end the notifier.
            let (position, dirty) = state::take_position();
            if let Some(p) = position
                && dirty
            {
                let v = p.encode();
                let _ = server.gps.position.set(server, &v);
                if server.gps.position.notify(conn, &v).await.is_err() {
                    break;
                }
            }
            if let Some(t) = state::telemetry() {
                let v = t.encode();
                let _ = server.gps.telemetry.set(server, &v);
                let _ = server.gps.telemetry.notify(conn, &v).await;
            }
        }
    };

    // Remote reports are pushed as they arrive rather than sampled on the
    // notify tick: a node's report is news exactly once, and two nodes
    // reporting inside one tick would mean only the second was ever sent.
    let remotes = async {
        loop {
            state::REMOTE_SIGNAL.wait().await;
            while state::radio_busy() {
                Timer::after(Duration::from_millis(100)).await;
            }
            while let Some(value) = state::take_remote(Instant::now().as_millis()) {
                // `set` first so the value is readable by a central that
                // never subscribed; an unsubscribed notify is not an error.
                match value {
                    Value::Position(v) => {
                        if server.gps.remote.set(server, &v).is_ok() {
                            let _ = server.gps.remote.notify(conn, &v).await;
                        }
                    }
                    Value::Ping(v) => {
                        if server.gps.node_ping.set(server, &v).is_ok() {
                            let _ = server.gps.node_ping.notify(conn, &v).await;
                        }
                    }
                }
            }
        }
    };

    // Stream status lines to the central as they arrive. Notify errors are
    // non-fatal (a central that never subscribed just misses them), so this
    // arm never ends the session on its own.
    let logger = async {
        loop {
            let line = state::LOG_CHANNEL.receive().await;
            let _ = server.gps.log.notify(conn, &line).await;
        }
    };

    // Republish the radio config whenever a new one is adopted, so an app's
    // radio editor updates without a reconnect.
    let config_pub = async {
        loop {
            state::RADIO_CONFIG_SIGNAL.wait().await;
            publish_radio_config(server, conn).await;
        }
    };

    // A `CFG_SLEEP_NOW` write ends the session, because the board is about
    // to stop being contactable and a connection left open would show the
    // phone a supervision timeout instead of a disconnect.
    //
    // The pause is what makes the ack useful. `events` sends the ack and
    // republishes the settings *after* it has already applied the write, so
    // the signal this waits on is raised while the notification that
    // explains it is still queued. Ending the session immediately would
    // drop that notification and leave the app with a write it never heard
    // back from, which is indistinguishable from a board that crashed.
    let commanded_sleep = async {
        state::SLEEP_NOW_SIGNAL.wait().await;
        Timer::after(Duration::from_millis(400)).await;
    };

    // Any arm ending (disconnect, a position notify that failed, or a
    // commanded sleep) ends the session.
    select3(
        select3(events, notifier, logger),
        select(config_pub, commanded_sleep),
        remotes,
    )
    .await;
}


// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------

/// An activity LED that is lit from the loop rather than by blocking in it.
///
/// The obvious light-it, `Timer::after(20ms)`, dark-it costs the receive
/// path twenty milliseconds it should be spending polling the radio, which
/// at the default settings is most of a packet.
struct Blinker {
    pin: Output<'static>,
    off_at: u32,
    lit: bool,
}

impl Blinker {
    fn new(pin: Output<'static>) -> Self {
        Self {
            pin,
            off_at: 0,
            lit: false,
        }
    }

    fn pulse(&mut self, now: u32) {
        self.pin.set_level(LED_ON);
        self.off_at = now.wrapping_add(BLINK_MS);
        self.lit = true;
    }

    fn update(&mut self, now: u32) {
        if self.lit && due(now, self.off_at) {
            self.pin.set_level(LED_OFF);
            self.lit = false;
        }
    }
}

/// How many times a settings push is retried before the loop stops asking.
const GPS_CFG_TRIES: u8 = 5;

/// Everything that is not BLE: the radio, the GPS and the card.
///
/// Owning them in one task is what removes the link protocol. The BLE
/// session never touches the hardware; it reads the snapshot this publishes
/// and signals back through [`state::Request`].
#[embassy_executor::task]
async fn hardware_task(
    lora: Sx1262Driver<'static>,
    mut gps: Gps<'static>,
    mut sdlog: SdLog<'static>,
    mut j5: Option<J5>,
    d5: Output<'static>,
    d2: Output<'static>,
    boot: Mode,
    cold: bool,
) {
    let mut rx_led = Blinker::new(d5);
    let mut tx_led = Blinker::new(d2);
    // Owned by this task rather than given one of its own, because the
    // panel has to be blanked *before* the board sleeps and this is where
    // `PrepareSleep` is answered. A separate task would need the ordering
    // negotiated; here it is a function call in the right place.
    let mut next_oled = 0u32;
    // What the board is doing, which is what decides how much of this task
    // runs. Moved by `Request::Mode`, never by this task on its own.
    let mut live = boot;

    // A wake check reads no card. It exists to ask whether anyone wants the
    // board back, and that question needs BLE and nothing else - so a wake
    // nobody answers never pays to mount a filesystem, and a promotion
    // reads the card then instead.
    let mut card_up = boot != Mode::Stored;
    if !card_up {
        sdlog.defer();
    }

    let mut cfg = RadioConfig::default();
    let mut cfg_loaded = false;
    if card_up {
        cfg_loaded = adopt_stored_config(&mut sdlog, &mut cfg, 0, cold).await;
    }

    let mut node = Node::new(lora, &cfg);

    // What this boot raises, which is the whole difference between the
    // three flavors.
    match boot {
        Mode::Tracking => {
            node.radio_mut().init(&cfg).await;
            if !node.radio_mut().print_diagnostics() {
                println!("radio did not answer - check the pin map in main");
            }
            gps.configure(&cfg.gps).await;
        }
        Mode::Idle => {
            // Reachable and nothing else. This is a cold boot, so the
            // receiver is at its power-on default and acquiring; park it.
            gps.park().await;
            node.radio_mut().sleep();
        }
        Mode::Stored => {
            // The park that caused this sleep left the receiver in an
            // open-ended backup, and a reset does not change that - so the
            // driver is told what the module is actually doing rather than
            // being left with its power-on assumption. Without it a
            // promotion straight to tracking would find `wake` a no-op, and
            // the receiver would stay in backup with nothing to notice it:
            // the settings retry is gated on a sentence having been seen,
            // and in backup there are none.
            gps.sleeping = true;
            // A wake check speaks to neither. `gps.configure` alone would
            // undo the sleep the last park asked for - UART RX is one of
            // the M10's backup wake sources, so the boot path would wake the
            // receiver on every single wake just to have the park path put
            // it back - and any SPI transaction pulls NSS low, which is the
            // edge the SX1262 leaves cold sleep on.
        }
    }

    // The settings that survive a deep sleep are re-applied here, because
    // the wake that restored them is a fresh boot to everything else.
    let stored = settings::get();
    // The isolation build asks unconditionally, because the point is to
    // measure the receiver rather than to honor a setting. V_BCKP is not
    // fed on this board, so the wake path is unproven - a power cycle is
    // the way back.
    #[cfg(feature = "iso-gps-backup")]
    {
        // Timed rather than open-ended, so the test recovers itself. Read
        // the meter across the gap: the drop is the receiver's own draw,
        // and the sentence counter climbing again on the far side is the
        // proof that backup mode is usable on a board whose V_BCKP is not
        // fed. If it never comes back, that is the answer too - and it
        // costs a power cycle rather than a board that cannot be recovered
        // without one.
        const ISO_GPS_BACKUP_MS: u32 = 20_000;
        gps.sleep_for(ISO_GPS_BACKUP_MS);
        status_println!(
            "iso-gps-backup: GPS in backup for {} s - watch the meter, then the nmea count",
            ISO_GPS_BACKUP_MS / 1000
        );
    }
    // The two manual overrides, which still mean what they always meant:
    // they park one subsystem where a mode parks all of them. Only a boot
    // that raised something can lower it again.
    if boot == Mode::Tracking {
        if stored.gps_sleep() {
            gps.park().await;
        }
        if stored.wio_sleep() {
            node.radio_mut().standby();
        }
    }
    let mut standby = boot != Mode::Tracking || stored.wio_sleep();

    match boot {
        Mode::Tracking => status_println!(
            "tracking: node {} ({}), {} Hz SF{} BW{}",
            cfg.address,
            cfg.role.as_str(),
            cfg.frequency_hz,
            cfg.spreading_factor,
            cfg.bandwidth_khz
        ),
        Mode::Idle => match stored.sleep_interval_s {
            // No cadence to sleep on, so the timeout has nowhere to send
            // it: this board stays reachable until something says
            // otherwise. The bench case, and the unconfigured one.
            0 => status_println!("idle: gps in backup, radio asleep, no sleep cadence set"),
            _ => status_println!(
                "idle: gps in backup, radio asleep, {} s before it stores itself",
                stored.idle_timeout()
            ),
        },
        Mode::Stored => status_println!(
            "wake check: nothing raised, {} s window then back down",
            stored.adv_window()
        ),
    }

    // Something on the panel before the first telemetry, so a board that
    // fails during init does not look like a board with a dead display. Not
    // on a wake check: the park path blanked the panel and the panel keeps
    // its own state across the sleep, so lighting it here would put the
    // display's current back into every wake.
    if boot != Mode::Stored
        && let Some(j) = j5.as_mut()
        && let Some(o) = j.oled.as_mut()
    {
        wio_s3_gps::oled::render(o, None, cfg.address);
        o.flush(&mut j.i2c).await;
    }

    let mut rx_count: u32 = 0;
    let mut tx_count: u32 = 0;
    let mut had_fix = false;
    // Whether a fix has ever held since boot, which is what separates a fix
    // lost from one never acquired in the no-fix ping below.
    let mut ever_had_fix = false;

    let boot = Instant::now().as_millis() as u32;
    // Stagger the first beacon so a fleet powered up together does not
    // transmit as one. Folded into eight slots rather than scaled by the
    // address itself, which made node 200 sit silent for three and a half
    // minutes after boot with nothing on the console to explain it.
    let mut next_beacon = boot
        .wrapping_add((cfg.address as u32 % 8) * 1_000)
        .wrapping_add(2_000);
    // The instant the owed beacon is planned for, once the interval has
    // run out; `None` while none is.
    let mut beacon_at: Option<u64> = None;
    let mut next_status = boot.wrapping_add(5_000);
    let mut idle_mark = wio_s3_gps::idle::entries();
    let mut idle_at = boot;
    let mut next_pos = boot;
    let mut next_gps_cfg = boot;
    let mut gps_cfg_tries: u8 = 0;
    // Last seen backup state, so a receiver that wakes on its own timer is
    // noticed. `sleep_for` ends with nothing sent to the host.
    let mut gps_was_sleeping = gps.sleeping;
    let mut gps_nmea_seen = false;
    let mut gps_checked = false;
    let gps_grace_until = boot.wrapping_add(5_000);

    loop {
        let now_ms = Instant::now().as_millis();
        let now = now_ms as u32;
        rx_led.update(now);
        tx_led.update(now);

        // ---- Requests from the BLE session and the host tools -----------
        while let Some(r) = state::take_request() {
            match r {
                Request::GpsSleep(true) => {
                    gps.park().await;
                    status_println!("gps: backup mode");
                }
                Request::GpsSleep(false) => {
                    gps.wake().await;
                    // `wake` marks the module unconfigured: backup mode
                    // loses the RAM layer the settings live in. Re-arm the
                    // retry so the loop pushes them again once the receiver
                    // is talking.
                    gps_cfg_tries = 0;
                    next_gps_cfg = now;
                    // Already accounted for; keep the self-wake detector
                    // below from reporting this one a second time.
                    gps_was_sleeping = false;
                    status_println!("gps: woken");
                }
                Request::RadioStandby(true) => {
                    node.radio_mut().standby();
                    standby = true;
                    status_println!("radio: standby");
                }
                Request::RadioStandby(false) => {
                    node.radio_mut().init(&cfg).await;
                    standby = false;
                    status_println!("radio: back from standby");
                }
                Request::Mode(m) => {
                    live = m;
                    // The card first: a config that has not been read yet
                    // is the one the radio is about to be initialized from.
                    if !card_up {
                        card_up = true;
                        cfg_loaded = adopt_stored_config(&mut sdlog, &mut cfg, now, false).await;
                        node.reconfigure(&cfg);
                    }
                    match m {
                        Mode::Tracking => {
                            gps.wake().await;
                            // `wake` marks the module unconfigured: backup
                            // mode loses the RAM layer the settings live in.
                            gps_cfg_tries = 0;
                            next_gps_cfg = now;
                            gps_was_sleeping = false;
                            node.radio_mut().init(&cfg).await;
                            standby = false;
                            status_println!(
                                "tracking: node {} ({}), gps and radio up",
                                cfg.address,
                                cfg.role.as_str()
                            );
                        }
                        // Idle, or the lowering half of a store - the sleep
                        // itself belongs to `PrepareSleep`, because deep
                        // sleep is entered from the side that owns the
                        // `Rtc`. Cold sleep rather than standby: nothing is
                        // going to use the radio, and `init` runs again
                        // whenever something does.
                        _ => {
                            gps.park().await;
                            gps_was_sleeping = true;
                            node.radio_mut().sleep();
                            standby = true;
                            status_println!("{}: gps in backup, radio asleep", m.as_str());
                        }
                    }
                }
                Request::ApplyConfig => {
                    if apply_radio_config(&mut node, &mut gps, &mut sdlog, &mut cfg, now).await {
                        cfg_loaded = true;
                        gps_cfg_tries = 0;
                        next_gps_cfg = now.wrapping_add(2_000);
                        // Applying a config re-inits the radio, which is
                        // the one thing that brings it back up. A board
                        // that was not using it - idle, or a standby the
                        // app asked for - gets it put straight back down,
                        // because a config push is not a request to start
                        // listening.
                        if standby {
                            node.radio_mut().sleep();
                        }
                    }
                }
                Request::PrepareSleep => {
                    // Everything a sleeping board cannot use, in the order
                    // that loses the least when the sequence does not
                    // finish - and it does not have to finish, because
                    // `enter_deep_sleep` waits a bounded time for the signal
                    // at the bottom and then sleeps anyway.
                    //
                    // So the three that cost current go first. Each is
                    // bounded work - a couple of UART bytes, an SPI command,
                    // an I2C frame - and each is milliamps for the whole
                    // interval if it is skipped, against a chip that is
                    // otherwise in microamps. The card goes last because it
                    // is the only one that can stall arbitrarily: a flush
                    // that lands on wear levelling is what expires the
                    // budget, and behind the other three that costs the
                    // buffered log lines rather than the sleep current the
                    // whole mode exists for.
                    //
                    // Did the last park hold?
                    //
                    // Free to ask here and nowhere else. Nothing polls the
                    // receiver while the board is in standby, so whatever
                    // is sitting in the UART FIFO at this point came from a
                    // receiver that was awake during a wake check - the
                    // ~10 mA failure that costs a whole sleep interval, and
                    // that is otherwise invisible, because the deep sleep
                    // reset this driver's idea of the module's state along
                    // with everything else.
                    if standby {
                        let before = gps.rx_bytes();
                        gps.poll();
                        if gps.rx_bytes() != before {
                            status_println!(
                                "gps: talking at park - the last park did not hold"
                            );
                        }
                    }
                    // The receiver is the largest load a sleeping board can
                    // carry, at around 30 mA against a chip that is
                    // otherwise in microamps - and a sleeping S3 cannot use
                    // a fix anyway. Re-issued on every park rather than
                    // assumed: a reset leaves this driver believing the
                    // module is awake, and the module is not obliged to
                    // agree with either of us.
                    gps.park().await;
                    // Cold sleep, not standby: `init` runs again on the
                    // wake, which is a full reset anyway.
                    node.radio_mut().sleep();
                    // The panel sits on the always-on +3V3, so without this
                    // it holds its last frame - and its current - for the
                    // whole sleep.
                    if let Some(j) = j5.as_mut()
                        && let Some(o) = j.oled.as_mut()
                    {
                        o.blank(&mut j.i2c).await;
                    }
                    // Last, per the order above. Deep sleep is a full reset,
                    // so the pending log buffer is otherwise simply gone -
                    // up to five fixes at the 1 Hz rate, and always the five
                    // at the end of the wake, which is the part a reader
                    // would use to work out where the board was when it went
                    // down. The unmount that follows leaves the FAT
                    // directory entry closed rather than trusting a sleeping
                    // card to have finished.
                    sdlog.park(now);
                    standby = true;
                    state::SLEEP_READY.signal(());
                }
                Request::Reboot => {
                    // Same hole as a deep sleep, and the same fix: the
                    // pending buffer is RAM, and a reset is a reset whether
                    // it is for an update or for a sleep.
                    sdlog.park(now);
                    // Long enough for the ack that asked for this to leave
                    // the USB FIFO or the BLE connection.
                    Timer::after(Duration::from_millis(500)).await;
                    esp_hal::system::software_reset();
                }
            }
        }

        // A bulk transfer whose host walked away must not hold the board
        // off the air; the USB task bounds its own read, and this covers a
        // transfer that arrived over BLE. Gated on the flag so the common
        // case does not queue behind whoever holds the transfer lock.
        if state::transfer_active() {
            xfer::expire(now_ms).await;
        }

        // The GPS, the beacon and the receiver, none of which a board
        // that is idle, storing itself or in a radio standby the app asked
        // for has any use for.
        //
        // What follows this block still runs. A board that is idle rather
        // than asleep is one somebody may be looking at, so the card keeps
        // flushing and mounting, telemetry stays fresh, the panel keeps its
        // frame and the status line keeps printing - and none of that costs
        // anything on a wake check, where the card is off the bus and the
        // panel is dark.
        if !standby {
            // ---- GPS ---------------------------------------------------------
            gps.poll();
            // A timed backup ends on the module's own clock, so the only signal
            // is that sentences started again - the driver clears its own flag
            // on the first one. Re-arm the config retry here, because the
            // settings do not survive backup and the budget may already be
            // spent from an earlier attempt.
            if gps_was_sleeping && !gps.sleeping {
                gps_cfg_tries = 0;
                next_gps_cfg = now;
                status_println!("gps: woke itself from backup, reconfiguring");
            }
            gps_was_sleeping = gps.sleeping;
            let fix = gps.has_fix();
            if fix != had_fix {
                had_fix = fix;
                if fix {
                    ever_had_fix = true;
                    status_println!("gps fix acquired ({} sats)", gps.packet().sats);
                } else {
                    status_println!("gps fix lost");
                }
            }
            if !gps_nmea_seen && gps.present() {
                gps_nmea_seen = true;
                status_println!("gps: NMEA up ({} bytes)", gps.rx_bytes());
            }
            // The hop clock's best reference. Taken whether or not there is
            // a fix, so a stale mark cannot be handed over later as if it
            // were fresh; used only with one.
            if let Some((tod_ms, at_ms)) = gps.take_time_mark()
                && gps.has_fix()
                && node.radio_mut().hop_discipline_gps(tod_ms, at_ms)
            {
                status_println!("hop: clock on gps time");
            }
            // Settings retry. Two things leave the module running its own
            // defaults while the firmware reports the ones it asked for: a
            // boot-time push that landed before the receiver had finished
            // starting, and a wake from backup mode, which cuts power to the
            // receiver core and takes the whole RAM configuration layer with it
            // - including the four NMEA sentences this firmware silences to fit
            // 9600 baud. Driving the retry off `configured` rather than off the
            // first sentence covers both, since `wake` clears it.
            if !gps.configured
                && !gps.sleeping
                && gps.present()
                && gps_cfg_tries < GPS_CFG_TRIES
                && due(now, next_gps_cfg)
            {
                next_gps_cfg = now.wrapping_add(2_000);
                gps_cfg_tries += 1;
                if gps.configure(&cfg.gps).await {
                    status_println!("gps: settings applied");
                } else if gps_cfg_tries == GPS_CFG_TRIES {
                    status_println!("gps: settings still not accepted, giving up");
                }
            }
            // Not while the receiver is in a backup this firmware asked for:
            // silence is the request working, and reporting it as a wiring or
            // baud fault sends whoever is measuring the GPS off after a bug
            // that is not there.
            if !gps_checked && !gps.sleeping && due(now, gps_grace_until) {
                gps_checked = true;
                if !gps.present() {
                    if gps.rx_bytes() == 0 {
                        status_println!("gps: silent on UART1 (power/wiring?)");
                    } else {
                        status_println!("gps: {} bytes but no NMEA (baud?)", gps.rx_bytes());
                    }
                }
            }
            if gps.take_updated() && due(now, next_pos) {
                next_pos = now.wrapping_add(1_000);
                let p = gps.packet();
                state::set_position(p);
                if gps.has_fix() {
                    sdlog.log_position(now, 0, 0, &p);
                }
            }

            // ---- LoRa beacon: a position, or a ping without a fix ------------
            //
            // Gated on the role here rather than inside the transmit, so a
            // receive-only node never claims the air for a broadcast it was
            // never going to send.
            //
            // One transmission per interval either way. A fix goes out as a
            // position; without one the slot carries a ping, so a node
            // searching for the sky is a node a receiver can hear rather than
            // one indistinguishable from out of range or dead. A ping is the
            // smaller of the two on air, so this cannot push a node past the
            // duty cycle its beacon already fits in.
            // The sleep gate is not politeness: a transmit awaits for the frame's
            // time on air, which at the slowest settings the config accepts is
            // nearly ten seconds, and a sleep that arrives just after one starts
            // waits all of it out. Declining to start one keeps the sleep path's
            // parking budget covering a beacon already in flight rather than one
            // this pass was about to begin.
            //
            // Two stages. The interval decides *that* a beacon is owed; on
            // a hopping network the slot clock then decides *when* in a slot
            // it goes, and that instant is planned here and waited for by
            // this loop rather than inside the transmit, where the wait would
            // hold the receiver off the air for up to a slot. On a single
            // channel the planned instant is now.
            //
            // The last gate is a frame arriving: keying up over it would
            // lose both, and the poll below will have delivered it by the
            // next pass. Bounded, since a preamble can be noise.
            let beacon_owed = cfg.role.transmits()
                && cfg.beacon_interval_s != 0
                && due(now, next_beacon)
                && !state::transfer_active()
                && !state::sleep_now_pending();
            if beacon_owed && beacon_at.is_none() {
                let len = node.frame_overhead()
                    + if gps.has_fix() {
                        lora::position_msg_len(cfg.beacon_fields)
                    } else {
                        lora::PING_MSG_LEN
                    };
                beacon_at = Some(node.radio_mut().tx_window_start(now_ms, len));
            }
            if beacon_owed
                && beacon_at.is_some_and(|at| now_ms >= at)
                && !node.radio().rx_in_progress(now_ms)
            {
                beacon_at = None;
                // The SX1262 does not reset with the MCU. If it browned out and
                // restarted on its own it is back at its power-up defaults -
                // antenna switch unpowered, DIO2 not switching - and keying up
                // into that ramps +22 dBm into an isolated port. Nothing about
                // it looks wrong from the counters, so this is the only place
                // it can be caught.
                if node.radio_mut().looks_reset() {
                    status_println!("radio restarted underneath us, re-initializing");
                    node.radio_mut().init(&cfg).await;
                }
                let payload_is_fix = gps.has_fix();
                state::set_radio_busy(true);
                tx_led.pulse(now);
                let sent = if payload_is_fix {
                    let (pos, n) = lora::encode_position(&gps.packet(), cfg.beacon_fields);
                    node.broadcast(&pos[..n]).await
                } else {
                    node.broadcast(
                        &lora::Ping {
                            uptime_s: (now_ms / 1_000).min(u16::MAX as u64) as u16,
                            gps_present: gps.present(),
                            had_fix: ever_had_fix,
                        }
                        .encode(),
                    )
                    .await
                };
                state::set_radio_busy(false);
                match sent {
                    Ok(()) => {
                        tx_count = tx_count.saturating_add(1);
                        vprintln!(
                            "beacon {} ({} ms on air)",
                            if payload_is_fix { "position" } else { "ping" },
                            cfg.beacon_airtime_us() / 1000
                        );
                    }
                    Err(e) => vprintln!("beacon TX failed: {:?}", e),
                }
                // Jitter on top of the interval so two nodes that happened to
                // line up do not stay lined up.
                //
                // Timed from after the transmit, not from the top of this pass:
                // the send awaited, and at the slowest settings the config
                // accepts that is nearly ten seconds. Measuring the interval
                // from a stale `now` would spend most of it inside the
                // transmission it is supposed to follow.
                let jitter = node.random(2_000);
                next_beacon = (Instant::now().as_millis() as u32)
                    .wrapping_add(cfg.beacon_interval_s as u32 * 1_000)
                    .wrapping_add(jitter);
            }

            // ---- LoRa receive -------------------------------------------------
            if let Some((src, stratum)) = node.take_sync_note() {
                status_println!("hop: clock from node {} (stratum {})", src, stratum);
            }
            if let Some(rx) = node.poll(now) {
                rx_count = rx_count.saturating_add(1);
                rx_led.pulse(now);
                if let Some(p) = lora::decode_position(rx.payload) {
                    vprintln!("position from node {} rssi {}", rx.src, rx.rssi);
                    let mut v = [0u8; ble::REMOTE_LEN];
                    v[0] = rx.src;
                    v[1..3].copy_from_slice(&rx.rssi.to_le_bytes());
                    v[3..].copy_from_slice(&p.encode());
                    state::record_remote(now_ms, Report::Position(v));
                    sdlog.log_position(now, rx.src, rx.rssi, &p);
                } else if let Some(ping) = lora::Ping::decode(rx.payload) {
                    // A node on the air with no fix to report. Nothing to log
                    // to SD - there is no position - but an app gets it as data
                    // as well as prose, so it can show the node as
                    // alive-without-a-fix rather than parse the line below.
                    let mut v = [0u8; link::PING_LEN];
                    v[0] = rx.src;
                    v[1..3].copy_from_slice(&rx.rssi.to_le_bytes());
                    v[3] = ping.flags();
                    v[4..6].copy_from_slice(&ping.uptime_s.to_le_bytes());
                    state::record_remote(now_ms, Report::Ping(v));
                    status_println!(
                        "node {} ping: rssi {}, up {}s, gps {}{}",
                        rx.src,
                        rx.rssi,
                        ping.uptime_s,
                        if ping.gps_present { "ok" } else { "silent" },
                        if ping.had_fix { ", fix lost" } else { "" }
                    );
                } else {
                    vprintln!(
                        "node {} sent {} bytes this build does not decode",
                        rx.src,
                        rx.payload.len()
                    );
                }
            }

            // ---- Repeat forwarding --------------------------------------------
            // Only a node configured as a repeater ever has one of these
            // queued; a leaf-only network never enters this branch.
            if node.repeat_due(now)
                && !state::transfer_active()
                && !node.radio().rx_in_progress(now_ms)
            {
                state::set_radio_busy(true);
                tx_led.pulse(now);
                let went = node.send_due_repeat(now).await;
                state::set_radio_busy(false);
                if went {
                    tx_count = tx_count.saturating_add(1);
                }
            }
        }

        // ---- Telemetry -----------------------------------------------------
        let secs_since_rx = match node.last_rx_ms() {
            Some(t) => (now.wrapping_sub(t) / 1000).min(0xFFFE) as u16,
            None => 0xFFFF,
        };
        let mut flags = 0u8;
        if sdlog.ready() {
            flags |= link::TELEM_FLAG_SD_OK;
        }
        if gps.has_fix() {
            flags |= link::TELEM_FLAG_GPS_FIX;
        }
        if cfg_loaded {
            flags |= link::TELEM_FLAG_CFG_LOADED;
        }
        if cfg.verbose {
            flags |= link::TELEM_FLAG_VERBOSE;
        }
        let (hop, hop_channel) = match node.radio().hop_status(now_ms) {
            Some((stratum, channel)) => (link::TELEM_HOP_ON | stratum, channel),
            None => (0, 0),
        };
        state::set_telemetry(link::Telemetry {
            last_rssi: node.last_rssi(),
            last_snr_cb: node.radio().last_snr_cb(),
            secs_since_rx,
            rx_count,
            tx_count,
            flags,
            sats: gps.packet().sats,
            hop,
            hop_channel,
        });

        sdlog.poll(now);

        // ---- Status display -------------------------------------------------
        // Twice a second. The fields underneath change about once a second,
        // so this is fast enough that a fix or a packet lands promptly and
        // slow enough that the 11 ms frame write is a couple of percent of
        // the loop. `flush` is a no-op when nothing changed.
        if j5.is_some() && live != Mode::Stored && due(now, next_oled) {
            next_oled = now.wrapping_add(500);
            let telemetry = state::telemetry();
            let own = gps.packet();
            let target = state::compass_target(now_ms);
            if let Some(j) = j5.as_mut() {
                // Sampled every refresh whether or not a panel is fitted:
                // the hard-iron calibration only improves by being fed, and
                // a board being carried is calibrating itself.
                if let Some(c) = j.compass.as_mut() {
                    c.sample(&mut j.i2c).await;
                }
                if let Some(o) = j.oled.as_mut() {
                    draw_screen(o, j.compass.as_ref(), telemetry, &own, target, cfg.address);
                    o.flush(&mut j.i2c).await;
                }
            }
        }

        // ---- Periodic status ------------------------------------------------
        // Without this a quiet radio and a quiet GPS look identical from the
        // console.
        if due(now, next_status) {
            // Idle fraction over the window just ended, not since boot: a
            // cumulative figure would average away exactly the thing worth
            // seeing, which is a core that stopped halting at some point.
            let idle_now = wio_s3_gps::idle::entries();
            let idle_hz = wio_s3_gps::idle::rate(idle_mark, idle_now, now.wrapping_sub(idle_at));
            idle_mark = idle_now;
            idle_at = now;
            next_status = now.wrapping_add(10_000);
            let (mode, err) = node.radio_mut().health();
            let (hop_ch, hop_stratum) = match node.radio().hop_status(now_ms) {
                Some((stratum, channel)) => (channel as i32, stratum as i32),
                None => (-1, -1),
            };
            status_println!(
                "t={}s radio {} err {:04x} rx {} tx {} hop ch {} s {} | gps {} nmea fix {} sats {} | sd {} | nodes {} | idle {} Hz",
                now_ms / 1000,
                mode,
                err,
                rx_count,
                tx_count,
                hop_ch,
                hop_stratum,
                gps.rx_sentences(),
                gps.has_fix() as u8,
                gps.packet().sats,
                if sdlog.ready() { "mounted" } else { "absent" },
                state::remote_count(),
                idle_hz
            );
            // Verbose only: break down what the radio heard but did not
            // deliver, so "a couple of random RXs" can be read as mostly
            // CRC failures (weak signal / parameter mismatch), duplicates,
            // or this node's own echoes rather than a mystery. Counts are
            // cumulative since boot.
            let d = node.rx_drops();
            vprintln!(
                "dropped: crc {} dup {} echo {} malformed {} oversize {} repeat-full {}",
                node.radio().rx_crc_errors(),
                d.duplicate,
                d.own_echo,
                d.malformed,
                node.radio().rx_oversize(),
                d.repeat_full
            );
        }

        // 100 Hz while there is a radio to poll and a UART to drain; a
        // fifth of that when there is neither, since the housekeeping above
        // has nothing that moves faster than the panel's 2 Hz.
        Timer::after(Duration::from_millis(if standby { 50 } else { 10 })).await;
    }
}

/// Choose and draw the screen: the compass when there is somewhere to
/// point, the status readout otherwise.
///
/// The compass needs both ends of a bearing - this node's fix and another
/// node's - so it appears exactly when it can be correct and the status
/// screen is what a board sees the rest of the time. That is a better trade
/// than alternating: a panel this size is read at a glance, and a glance
/// that lands on the wrong half of a rotation is worse than a screen that
/// only changes when the situation does.
fn draw_screen(
    oled: &mut wio_s3_gps::oled::Oled,
    compass: Option<&wio_s3_gps::compass::Compass>,
    telemetry: Option<link::Telemetry>,
    own: &packet::PositionPacket,
    target: Option<(u8, packet::PositionPacket, u16, i16)>,
    node_address: u8,
) {
    use wio_s3_gps::oled;

    let Some((node, remote, age_s, rssi)) = target else {
        oled::render(oled, telemetry, node_address);
        return;
    };
    if !own.has_fix() || !remote.has_fix() {
        oled::render(oled, telemetry, node_address);
        return;
    }

    let from = (own.lat_e7, own.lon_e7);
    let to = (remote.lat_e7, remote.lon_e7);

    // Heading, best source first. The magnetometer works standing still but
    // only once the board has been turned through a circle; GPS course is
    // always trustworthy but only exists while actually moving, so a walking
    // pace floor keeps a stationary receiver's wandering course out of it.
    const MOVING_CMS: u16 = 50;
    let heading = match compass.and_then(|c| c.heading_deg()) {
        Some(deg) => oled::Heading::Magnetic(deg as u16),
        None if own.speed_cms >= MOVING_CMS => {
            oled::Heading::Course((own.course_cdeg / 100).min(359))
        }
        None => oled::Heading::None,
    };

    let target = oled::Target {
        node,
        bearing_deg: midair_proto::geo::bearing_deg(from, to),
        distance_m: midair_proto::geo::distance_m(from, to),
        age_s,
        rssi,
    };
    let (fix, sats) = (own.has_fix(), own.sats);
    oled::render_compass(oled, &target, heading, fix, sats);
}

/// Read the stored config and adopt it.
///
/// A function rather than a stretch of `hardware_task` because a wake check
/// defers all of it: the card stays off the bus until the board knows it is
/// more than a check, and a promotion runs this then. Everything here is
/// about the *stored* config - the radio settings, the console verbosity,
/// the `[power]` section - so it costs nothing on a wake nobody answers.
///
/// Returns whether a config was adopted at all, which is what the
/// `CFG_LOADED` telemetry flag reports.
async fn adopt_stored_config(
    sdlog: &mut SdLog<'static>,
    cfg: &mut RadioConfig,
    now: u32,
    cold: bool,
) -> bool {
    sdlog.resume(now);
    // Give the card a chance to mount before its config is asked for.
    sdlog.poll(now);

    // Two stores, and the card wins. Editing `RADIO.CFG` on a computer has
    // to do what it looks like, so a card that says anything outranks the
    // backup - which exists for the board the card cannot answer for: one
    // that never had a card, or whose card has failed. Without it such a
    // board came back on firmware defaults and lost its address, the one
    // setting nothing can guess back.
    //
    // The buffer spans the awaits below rather than being scoped around the
    // card read, because both stores are read and written through it. It
    // costs a kilobyte of this task's future.
    let mut text = [0u8; CONFIG_MAX];
    let from_card = sdlog
        .read_config(&mut text)
        .and_then(|n| match radiocfg::parse_bytes(&text[..n]) {
            Ok(c) => Some((c, n)),
            Err(e) => {
                println!("config: RADIO.CFG invalid ({:?}), trying the backup", e);
                None
            }
        });
    let loaded = match from_card {
        Some((c, n)) => {
            println!("config: RADIO.CFG loaded (address {})", c.address);
            *cfg = c;
            // Keep the backup level with the card, so a card pulled or lost
            // later does not take the config with it. An unchanged card
            // costs a comparison rather than the two erases of a write,
            // which is the common case for every boot after the first.
            if !flash::with_flash(|f| f.save_config(&text[..n]))
                .await
                .unwrap_or(false)
            {
                println!("config: RADIO.CFG could not be backed up to flash");
            }
            true
        }
        None => match flash::with_flash(|f| f.load_config(&mut text)).await.flatten() {
            Some(n) => match radiocfg::parse_bytes(&text[..n]) {
                Ok(c) => {
                    println!("config: flash backup loaded (address {})", c.address);
                    *cfg = c;
                    true
                }
                // Only a config that parsed is ever written, so this is the
                // record disagreeing with a firmware that has since changed
                // what it accepts - not a bad push. Defaults, and say so.
                Err(e) => {
                    println!("config: flash backup invalid ({:?}), using defaults", e);
                    false
                }
            },
            None => {
                println!("config: none stored, using defaults");
                false
            }
        },
    };
    // Honor sd_enabled only now: the setting itself lives on the card, so
    // the card has to be read before it can say to stop using it.
    if !cfg.sd_enabled {
        println!("SD: disabled by config");
        sdlog.disable(now);
    }
    state::set_verbose(cfg.verbose);
    state::set_radio_config(cfg.encode());
    state::set_tx_worst_case_ms(cfg.tx_worst_case_ms());
    adopt_power(cfg, cold).await;
    loaded
}

/// Let a config file set the duty cycle.
///
/// `cold` gates this to a cold boot, and the distinction is the whole
/// design. The three settings in `[power]` are also kept in RTC RAM so they
/// survive a deep sleep, and an app can change them live over BLE - so
/// re-reading the card on every wake check would undo a live change once an
/// interval, forever. On a cold boot there is nothing live to undo and the
/// card is the only thing that outlives a reflash, so it wins.
///
/// A file that mentions none of the three changes nothing and costs no
/// flash write, which is the common case.
///
/// One thing this cannot catch up with: the advertising window of the very
/// first window is fixed before the card is mounted, so a `adv_window_s`
/// from the file takes effect from the second window on. `ble_off_s` is
/// re-read at the end of every window and has no such lag.
async fn adopt_power(cfg: &RadioConfig, cold: bool) {
    let asked = cfg.power;
    if !cold {
        if asked != Default::default() {
            qprintln!("config: [power] ignored on a wake, the live settings win");
        }
        return;
    }
    if !settings::adopt_power(&asked) {
        return;
    }
    let now = settings::get();
    status_println!(
        "config: [power] adopted - ble off {} s, adv window {} s, sleep interval {} s",
        now.ble_off_s,
        now.adv_window(),
        now.sleep_interval_s
    );
    // Mirrored to flash for the same reason a BLE write to any of these is:
    // they decide whether the board is reachable at all, and a board that
    // came back from a flat cell without them would advertise continuously
    // until it died again.
    settings::save().await;
}

/// Adopt a radio config that arrived over BLE or USB.
///
/// The transfer already parsed it - a config that would not parse never
/// reaches here, and the host was told so in the ack. What is left is the
/// hardware: the radio, the node's own addressing, the GPS, and the card
/// copy that has to survive a reboot.
async fn apply_radio_config(
    node: &mut Node<'static>,
    gps: &mut Gps<'static>,
    sdlog: &mut SdLog<'static>,
    cfg: &mut RadioConfig,
    now: u32,
) -> bool {
    let mut raw = [0u8; bulk::CONFIG_MAX];
    let Some((new_cfg, len)) = xfer::take_pending(&mut raw) else {
        return false;
    };
    let regps = new_cfg.gps != cfg.gps;
    *cfg = new_cfg;
    // Unlike at boot this is unconditional: a push is somebody deliberately
    // sending this file now, so it outranks whatever is live. It is also the
    // only way `[power]` reaches a board that is already running.
    adopt_power(cfg, true).await;
    node.radio_mut().init(cfg).await;
    node.reconfigure(cfg);
    state::set_verbose(cfg.verbose);
    state::set_radio_config(cfg.encode());
    state::set_tx_worst_case_ms(cfg.tx_worst_case_ms());
    // Both stores, because either one alone leaves a board that loses this
    // config at the next power cycle: a card can be absent or failed, and
    // the backup is behind whatever a computer last wrote to the card. A
    // write that reached neither has to be reported to the operator rather
    // than left in a console nobody is reading - it is the difference
    // between a config that is applied and one that is applied until the
    // next reboot.
    //
    // Written before `sd_enabled` is honored, and deliberately: a config
    // that turns the card off still has to be *on* the card, or the next
    // boot reads nothing there and comes up with the card enabled again.
    let on_card = sdlog.write_config(now, &raw[..len]);
    let in_flash = flash::with_flash(|f| f.save_config(&raw[..len]))
        .await
        .unwrap_or(false);
    if !cfg.sd_enabled {
        status_println!("SD: disabled by config");
        sdlog.disable(now);
    }
    status_println!(
        "config applied, node {} ({}), {}",
        cfg.address,
        cfg.role.as_str(),
        match (on_card, in_flash) {
            (true, true) => "saved to SD and flash",
            (true, false) => "saved to SD, NOT to flash",
            (false, true) => "saved to flash, NOT to SD",
            (false, false) => "NOT SAVED - lost on reboot",
        }
    );
    if regps && !gps.sleeping {
        if gps.configure(&cfg.gps).await {
            status_println!("gps reconfigured");
        } else {
            status_println!("gps did not accept settings");
        }
    }
    true
}
