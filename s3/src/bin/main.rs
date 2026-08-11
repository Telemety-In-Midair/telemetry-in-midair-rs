//! Wio-S3 firmware skeleton for the wio-s3-max-gps board.
//!
//! The board replaces the ESP32-C6 and WIO-E5 pair with one Wio-S3 module
//! (ESP32-S3R8 plus an SX1262), so this crate will eventually hold what
//! both of the others did: BLE, LoRa, GPS, SD and power. Right now it
//! blinks and talks, which is what proves the toolchain and the flash path
//! before any of that is worth porting. See `PORT-WIO-S3.md`.
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
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

/// Status LED, cathode on GPIO43. Active low.
const LED_ON: Level = Level::Low;
const LED_OFF: Level = Level::High;

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

    let mut ticks: u32 = 0;
    loop {
        d5.set_level(LED_ON);
        Timer::after(Duration::from_millis(50)).await;
        d5.set_level(LED_OFF);
        Timer::after(Duration::from_millis(950)).await;

        ticks += 1;
        if ticks % 10 == 0 {
            println!("alive, {ticks} s");
        }
    }
}
