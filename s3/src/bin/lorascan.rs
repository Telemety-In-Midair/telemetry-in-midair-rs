//! Find the Wio-S3's internal SX1262 wiring by brute force.
//!
//! Seeed does not publish which ESP32-S3 GPIOs the module's SX1262 sits on,
//! and a wrong guess looks exactly like a dead radio: `GetStatus` returns
//! 0xFF because nothing drives MISO. The candidates are the pins the module
//! does *not* bring out to a pad - GPIO4-10 and GPIO21 - since an internal
//! signal has nowhere else to be. (GPIO26-32 are the flash interface and
//! GPIO33-37 the octal PSRAM on the R8 part; neither is available.)
//!
//! The oracle is a register write/read-back rather than `GetStatus`, so a
//! floating bus cannot pass: a stuck-high or stuck-low line returns the
//! same byte for both test values, and only a real chip stores and returns
//! two different ones.
//!
//! Run with `cargo run --release --bin lorascan`, then put the four winning
//! pins into `main.rs`.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

/// GPIOs the module keeps to itself, so the radio has to be on some of
/// these.
const CANDIDATES: &[u8] = &[4, 5, 6, 7, 8, 9, 10, 21];

/// A scratch register: the LoRa sync word, which nothing reads until the
/// driver sets it.
const SCRATCH_REG: u16 = 0x0740;

/// Two values with no bits in common, so a bus stuck either way fails both.
const PROBE_A: u8 = 0x55;
const PROBE_B: u8 = 0xAA;

esp_bootloader_esp_idf::esp_app_desc!();

/// Steal one GPIO by number. Typed singletons cannot be indexed at runtime,
/// and the scan needs to try every pin in every role.
fn pin(n: u8) -> AnyPin<'static> {
    use esp_hal::peripherals::*;
    unsafe {
        match n {
            4 => GPIO4::steal().into(),
            5 => GPIO5::steal().into(),
            6 => GPIO6::steal().into(),
            7 => GPIO7::steal().into(),
            8 => GPIO8::steal().into(),
            9 => GPIO9::steal().into(),
            10 => GPIO10::steal().into(),
            21 => GPIO21::steal().into(),
            _ => unreachable!(),
        }
    }
}

/// One write/read-back round trip on a candidate assignment.
fn probe(sck: u8, mosi: u8, miso: u8, nss: u8) -> bool {
    let spi = match Spi::new(
        unsafe { esp_hal::peripherals::SPI2::steal() },
        SpiConfig::default()
            .with_frequency(Rate::from_hz(2_000_000))
            .with_mode(Mode::_0),
    ) {
        Ok(s) => s,
        Err(_) => return false,
    };
    let mut spi = spi
        .with_sck(pin(sck))
        .with_mosi(pin(mosi))
        .with_miso(pin(miso));
    let mut cs = Output::new(pin(nss), Level::High, OutputConfig::default());
    let delay = Delay::new();

    let write = |spi: &mut Spi<'_, esp_hal::Blocking>, cs: &mut Output<'_>, v: u8| {
        cs.set_low();
        let _ = spi.write(&[0x0D, (SCRATCH_REG >> 8) as u8, SCRATCH_REG as u8, v]);
        cs.set_high();
        delay.delay_micros(200);
    };
    let read = |spi: &mut Spi<'_, esp_hal::Blocking>, cs: &mut Output<'_>| -> u8 {
        let mut buf = [
            0x1D,
            (SCRATCH_REG >> 8) as u8,
            SCRATCH_REG as u8,
            0x00,
            0x00,
        ];
        cs.set_low();
        let _ = spi.transfer(&mut buf);
        cs.set_high();
        delay.delay_micros(200);
        buf[4]
    };

    write(&mut spi, &mut cs, PROBE_A);
    if read(&mut spi, &mut cs) != PROBE_A {
        return false;
    }
    // A bus that echoes or floats passes the first test half the time and
    // this one never: it has to actually store what it was given.
    write(&mut spi, &mut cs, PROBE_B);
    read(&mut spi, &mut cs) == PROBE_B
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    Timer::after(Duration::from_millis(500)).await;
    println!("lorascan: {} candidate pins", CANDIDATES.len());

    let mut found: Option<(u8, u8, u8, u8)> = None;
    'outer: for &sck in CANDIDATES {
        for &mosi in CANDIDATES {
            if mosi == sck {
                continue;
            }
            for &miso in CANDIDATES {
                if miso == sck || miso == mosi {
                    continue;
                }
                for &nss in CANDIDATES {
                    if nss == sck || nss == mosi || nss == miso {
                        continue;
                    }
                    if probe(sck, mosi, miso, nss) {
                        println!(
                            "HIT  sck=GPIO{} mosi=GPIO{} miso=GPIO{} nss=GPIO{}",
                            sck, mosi, miso, nss
                        );
                        found = Some((sck, mosi, miso, nss));
                        break 'outer;
                    }
                }
            }
        }
    }

    let Some((sck, mosi, miso, nss)) = found else {
        println!("no combination answered.");
        println!("the radio may be held in reset - try grounding nothing and");
        println!("power-cycling, or widen CANDIDATES if the module differs.");
        loop {
            Timer::after(Duration::from_secs(5)).await;
        }
    };

    // BUSY is the giveaway among what is left: the SX1262 raises it while
    // it works and drops it when idle, so a command should move exactly one
    // of the remaining pins.
    println!("identifying BUSY among the rest...");
    let mut spi = Spi::new(
        unsafe { esp_hal::peripherals::SPI2::steal() },
        SpiConfig::default()
            .with_frequency(Rate::from_hz(2_000_000))
            .with_mode(Mode::_0),
    )
    .expect("spi")
    .with_sck(pin(sck))
    .with_mosi(pin(mosi))
    .with_miso(pin(miso));
    let mut cs = Output::new(pin(nss), Level::High, OutputConfig::default());

    for &c in CANDIDATES {
        if c == sck || c == mosi || c == miso || c == nss {
            continue;
        }
        let probe_in = Input::new(pin(c), InputConfig::default());
        let idle = probe_in.is_high();
        // Calibrate: a long job that holds BUSY high for well over a
        // millisecond.
        cs.set_low();
        let _ = spi.write(&[0x89, 0x7F]);
        cs.set_high();
        let mut moved = false;
        for _ in 0..50 {
            if probe_in.is_high() != idle {
                moved = true;
                break;
            }
            Delay::new().delay_micros(20);
        }
        println!(
            "  GPIO{}: idle {}, moved on Calibrate: {}{}",
            c,
            if idle { "high" } else { "low" },
            moved,
            if moved && !idle { "   <- BUSY" } else { "" }
        );
        Timer::after(Duration::from_millis(5)).await;
    }

    println!();
    println!("put these in main.rs:");
    println!("  .with_sck(peripherals.GPIO{})", sck);
    println!("  .with_mosi(peripherals.GPIO{})", mosi);
    println!("  .with_miso(peripherals.GPIO{})", miso);
    println!("  nss = GPIO{}", nss);
    println!("  busy/dio1/nrst: the three left over, BUSY marked above");

    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
