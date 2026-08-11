//! Wio-S3 firmware for the wio-s3-max-gps board.
//!
//! The board replaces the ESP32-C6 and WIO-E5 pair with one Wio-S3 module
//! (ESP32-S3R8 plus an SX1262), so this crate will eventually hold what
//! both of the others did: BLE, LoRa, GPS, SD and power. So far it brings
//! up the radio and blinks. See `PORT-WIO-S3.md`.
//!
//! Pin assignments come from the board, not from preference:
//!
//! - D5 (GPIO43) and D2 (GPIO14) are **active low** - the LED anodes sit on
//!   +3V3 through R21/R20, so driving the pin low is what lights them.
//! - GPIO43 is also UART0_TX, which is why the console is on the USB
//!   Serial/JTAG port (GPIO19/20 to the USB-C J3) instead. Expect the ROM
//!   bootloader's own log to flicker D5 on every reset.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::delay::Delay;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_println::println;
use midair_proto::radiocfg::RadioConfig;
use wio_s3_gps::gps::{Gps, BAUD as GPS_BAUD};
use wio_s3_gps::radio::Sx1262Driver;
use wio_s3_gps::sdlog::SdLog;
use wio_s3_gps::sx1262::Sx1262;

/// Status LED, cathode on GPIO43. Active low.
const LED_ON: Level = Level::Low;
const LED_OFF: Level = Level::High;

/// The SX1262 SPI clock. The chip takes up to 16 MHz; the bus here is
/// entirely inside the module, so this is conservative rather than tuned.
const LORA_SPI_HZ: u32 = 8_000_000;

/// SD card SPI clock. Cards must be initialized at 400 kHz or under, and
/// this never raises it afterwards - a flush is about a kilobyte every five
/// seconds, so the 25 ms it costs is not worth the reconfiguration.
const SD_SPI_HZ: u32 = 400_000;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // D5. Starts dark so the first blink is visibly the firmware's, not a
    // leftover level from the ROM bootloader driving UART0_TX.
    let mut d5 = Output::new(peripherals.GPIO43, LED_OFF, OutputConfig::default());

    println!("wio-s3-gps v{} up", env!("CARGO_PKG_VERSION"));

    // ---------------------------------------------------------------
    // UNVERIFIED: the Wio-S3's internal ESP32-S3-to-SX1262 wiring is not
    // published in the module introduction, and searches for it return the
    // XIAO ESP32S3 + Wio-SX1262 kit, which is a different product with a
    // different map. GPIO4-10 is the only run of pins the module does not
    // bring out to a pad, and seven signals is exactly what the radio
    // needs, so that is the assumption below.
    //
    // Everything else in this crate is independent of these seven lines -
    // confirm them against the module datasheet or reference schematic and
    // this block is the only edit.
    // ---------------------------------------------------------------
    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_hz(LORA_SPI_HZ))
            .with_mode(Mode::_0),
    )
    .expect("lora spi")
    .with_sck(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO6)
    .with_miso(peripherals.GPIO7);

    let nss = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    let nrst = Output::new(peripherals.GPIO9, Level::High, OutputConfig::default());
    let busy = Input::new(peripherals.GPIO4, InputConfig::default());
    let dio1 = Input::new(peripherals.GPIO10, InputConfig::default());

    let mut lora = Sx1262Driver::new(Sx1262::new(spi, nss, busy, dio1, nrst));

    // Defaults until RADIO.CFG is readable, which needs the SD card port.
    let cfg = RadioConfig::default();
    lora.init(&cfg).await;

    if !lora.print_diagnostics() {
        println!("radio did not answer - check the pin map above");
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
    let mut gps = Gps::new(gps_uart);
    // The module may still be starting, in which case this goes
    // unacknowledged and the run loop re-pushes once it is talking.
    gps.configure(&cfg.gps).await;

    // microSD on SPI3. Three of these four lines are ESP32-S3 strapping
    // pins - see BOARD-REVIEW.md in the board repo; R17 on GPIO45 is DNP
    // for that reason.
    let sd_spi = Spi::new(
        peripherals.SPI3,
        SpiConfig::default()
            .with_frequency(Rate::from_hz(SD_SPI_HZ))
            .with_mode(Mode::_0),
    )
    .expect("sd spi")
    .with_sck(peripherals.GPIO46)
    .with_mosi(peripherals.GPIO45)
    .with_miso(peripherals.GPIO3);
    let sd_cs = Output::new(peripherals.GPIO44, Level::High, OutputConfig::default());
    let sd_dev = embedded_hal_bus::spi::ExclusiveDevice::new(sd_spi, sd_cs, Delay::new())
        .expect("sd spi device");
    let mut sdlog = SdLog::new(embedded_sdmmc::SdCard::new(sd_dev, Delay::new()));

    let mut buf = [0u8; 255];
    let mut ticks: u32 = 0;
    loop {
        d5.set_level(LED_ON);
        Timer::after(Duration::from_millis(50)).await;
        d5.set_level(LED_OFF);

        // 950 ms of listening, in slices, so a packet is noticed promptly
        // rather than once a second.
        for _ in 0..95 {
            if let Some((len, rssi)) = lora.poll_recv(&mut buf) {
                println!(
                    "rx {} bytes, {} dBm, snr {} cB",
                    len,
                    rssi,
                    lora.last_snr_cb()
                );
            }
            gps.poll();
            Timer::after(Duration::from_millis(10)).await;
        }

        ticks += 1;
        let now_ms = ticks.saturating_mul(1000);

        if gps.take_updated() {
            sdlog.log_position(now_ms, 0, 0, &gps.packet());
        }
        sdlog.poll(now_ms);

        if ticks % 10 == 0 {
            let (mode, err) = lora.health();
            println!(
                "alive {} s, radio {}, err 0x{:04X}, crc drops {}",
                ticks,
                mode,
                err,
                lora.rx_crc_errors()
            );
            println!(
                "gps {} sentences ({} bytes), fix {}, sd {}",
                gps.rx_sentences(),
                gps.rx_bytes(),
                gps.has_fix(),
                if sdlog.ready() { "mounted" } else { "absent" }
            );
        }
    }
}
