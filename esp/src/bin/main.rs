//! ESP32-C6 firmware for the telemetry-in-midair board.
//!
//! The C6 is the BLE face of the board: it serves the gps-proto GATT
//! service (so the existing gps-gui-rs app connects unchanged) extended
//! with telemetry, remote-position and bulk-transfer characteristics
//! (midair-proto). Position and status data come from the WIO-E5 over
//! UART0 (GPIO16 TX / GPIO17 RX) and are cached, so a freshly connected
//! central immediately receives the latest fix, LoRa RSSI and timestamps.
//!
//! Controls, all over BLE config writes:
//! - GPS/LoRa power rail (AP2112K LDO enable on GPIO2)
//! - WIO soft sleep / GPS backup mode (forwarded over the link; a stuck
//!   WIO is woken with a reset pulse on GPIO6)
//! - ESP deep sleep with a periodic wake-check: the C6 sleeps whenever no
//!   central is connected, waking every interval to advertise for a
//!   configurable window
//! - Radio TOML config and WIO firmware images pushed through the bulk
//!   characteristic and streamed over the UART link
//!
//! LED D2 (GPIO3): one long blink at the start of a sleep-interval wake,
//! short burst on config writes from the phone, fast toggling during a
//! firmware upload.
//!
//! Radio coordination: while the WIO flags a LoRa transmission the
//! notifier skips its ticks; while a bulk transfer runs the C6 flags
//! itself busy so the WIO defers its beacon.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::{Cell, RefCell};

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::partitions::{self, DataPartitionSubType, PartitionType};
use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig, RtcPin};
use esp_storage::{FlashStorage, FlashStorageError};
use esp_hal::rtc_cntl::sleep::TimerWakeupSource;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart, UartRx, UartTx};
use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use gps_proto::packet::{self, PositionPacket};
use midair_proto::ble;
use midair_proto::link::{self, cmd, msg, FrameBuf, FrameParser, Telemetry};
use midair_proto::radiocfg;
use trouble_host::prelude::*;

extern crate alloc;

/// Like [`println!`] but silent while a bulk transfer owns the USB link
/// (see [`console_busy`]).
macro_rules! qprintln {
    ($($arg:tt)*) => {
        if !console_busy() {
            ::esp_println::println!($($arg)*);
        }
    };
}

/// Like [`qprintln!`] but only for per-frame and per-heartbeat detail.
///
/// Gated twice. The `verbose` cargo feature decides whether these calls are
/// compiled in at all - it is on by default, and turning it off dead-code
/// eliminates them while still type-checking the arguments. Within such a
/// build, [`verbose_enabled`] is the runtime switch the `verbose` config key
/// drives, so the console can be quieted on a deployed board without
/// reflashing one.
macro_rules! vprintln {
    ($($arg:tt)*) => {
        if cfg!(feature = "verbose") && verbose_enabled() && !console_busy() {
            ::esp_println::println!($($arg)*);
        }
    };
}

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

/// Build-time BLE address override, most-significant octet first (e.g.
/// "FF:C6:A1:53:50:47"). `Some` only when `BLE_ADDRESS` was set at build
/// time (build.rs validates and normalizes it, and emits nothing otherwise);
/// `None` derives a per-chip address from the eFuse MAC instead.
const BLE_ADDRESS_OVERRIDE: Option<&str> = option_env!("BLE_ADDRESS");

/// The board's BLE address as the LSB-first array `Address::random` expects.
///
/// With no build-time override, it is derived from the chip's factory MAC in
/// eFuse so every board is unique out of the box: the MAC's most-significant
/// octet gets its top two bits set, which is all a static-random address
/// requires, and the per-chip low bytes keep it distinct.
fn ble_address() -> [u8; 6] {
    match BLE_ADDRESS_OVERRIDE {
        Some(s) => parse_ble_address(s),
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

/// Parse an MSB-first address string ("FF:C6:A1:53:50:47") into the
/// LSB-first array `Address::random` expects. build.rs has already validated
/// any override, so a malformed octet here can only be a bug; it falls back
/// to zero rather than panicking on the device.
fn parse_ble_address(s: &str) -> [u8; 6] {
    let mut out = [0u8; 6];
    for (i, octet) in s.split(':').take(6).enumerate() {
        out[5 - i] = u8::from_str_radix(octet, 16).unwrap_or(0);
    }
    out
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

/// The board's BLE address (LSB-first), published for the USB info query.
/// Set once at boot before the address is used.
static BLE_ADDR: Mutex<CriticalSectionRawMutex, Cell<[u8; 6]>> = Mutex::new(Cell::new([0; 6]));

/// How long to keep advertising after a disconnect (sleep mode active) so
/// the phone can come straight back before the C6 vanishes.
const SLEEP_LINGER_S: u64 = 5;

// ---------------------------------------------------------------------------
// Cached state (UART link task writes, BLE session samples)
// ---------------------------------------------------------------------------

static GPS_STATE: Mutex<CriticalSectionRawMutex, Cell<PositionPacket>> =
    Mutex::new(Cell::new(PositionPacket {
        lat_e7: 0,
        lon_e7: 0,
        alt_dm: 0,
        speed_cms: 0,
        course_cdeg: 0,
        flags: 0,
        sats: 0,
        tod_ms: 0,
    }));

static REMOTE_STATE: Mutex<CriticalSectionRawMutex, Cell<[u8; ble::REMOTE_LEN]>> =
    Mutex::new(Cell::new([0; ble::REMOTE_LEN]));

static TELEM_STATE: Mutex<CriticalSectionRawMutex, Cell<Telemetry>> = Mutex::new(Cell::new(
    Telemetry {
        last_rssi: 0,
        last_snr_cb: 0,
        secs_since_rx: 0xFFFF,
        rx_count: 0,
        tx_count: 0,
        flags: 0,
        sats: 0,
    },
));

/// Position notify interval, set over BLE. Survives disconnects but not
/// resets.
static NOTIFY_INTERVAL_MS: Mutex<CriticalSectionRawMutex, Cell<u32>> =
    Mutex::new(Cell::new(packet::UPDATE_INTERVAL_DEFAULT_MS));

/// Latest radio-config snapshot from the WIO ([`radiocfg::RadioConfig`] wire
/// form), served on the radio-config characteristic. The ESP never parses it
/// - it relays the bytes the WIO sends. Zeroed (layout version 0) until the
/// WIO first reports one, which the characteristic and its readers treat as
/// "not known yet" rather than a real config.
static RADIO_CONFIG_STATE: Mutex<CriticalSectionRawMutex, Cell<[u8; radiocfg::RADIO_CONFIG_LEN]>> =
    Mutex::new(Cell::new([0; radiocfg::RADIO_CONFIG_LEN]));

/// Set once the WIO has reported its config at least once.
static RADIO_CONFIG_KNOWN: AtomicBool = AtomicBool::new(false);

/// Pulsed when a fresh config arrives, so the connected session republishes
/// the characteristic (a mid-session change - the app pushing a new config -
/// is reflected without waiting for a reconnect).
static RADIO_CONFIG_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// `Instant::as_millis` deadline until which the WIO's radio is busy;
/// 0 = not busy. The BLE notifier defers its ticks while this is set.
static WIO_BUSY_UNTIL: Mutex<CriticalSectionRawMutex, Cell<u64>> = Mutex::new(Cell::new(0));

fn wio_busy() -> bool {
    WIO_BUSY_UNTIL.lock(|c| c.get()) > Instant::now().as_millis()
}

// ---------------------------------------------------------------------------
// Settings that survive deep sleep (RTC RAM with a magic for cold boots)
// ---------------------------------------------------------------------------

const PERSIST_MAGIC: u32 = 0x6D69_6461; // "mida"
const PFLAG_PWR_OFF: u32 = 1 << 0;
const PFLAG_WIO_SLEEP: u32 = 1 << 1;
const PFLAG_GPS_SLEEP: u32 = 1 << 2;

#[derive(Clone, Copy)]
struct Persist {
    sleep_interval_s: u32,
    flags: u32,
    /// 0 = never configured; read it through `adv_window_s`, which
    /// substitutes the default.
    adv_window_s: u32,
}

// These live in RTC fast RAM and are not reinitialized on a deep-sleep
// wake; the magic word gates cold-boot garbage. esp-hal's Persistable
// marker only covers atomics and primitives, hence three statics.
use portable_atomic::{AtomicBool, AtomicU32 as PersistU32, Ordering as PersistOrdering};
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PERSIST_MAGIC_WORD: PersistU32 = PersistU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PERSIST_INTERVAL: PersistU32 = PersistU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PERSIST_FLAGS: PersistU32 = PersistU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PERSIST_ADV_WINDOW: PersistU32 = PersistU32::new(0);

fn persist_get() -> Persist {
    if PERSIST_MAGIC_WORD.load(PersistOrdering::Relaxed) == PERSIST_MAGIC {
        Persist {
            sleep_interval_s: PERSIST_INTERVAL.load(PersistOrdering::Relaxed),
            flags: PERSIST_FLAGS.load(PersistOrdering::Relaxed),
            adv_window_s: PERSIST_ADV_WINDOW.load(PersistOrdering::Relaxed),
        }
    } else {
        Persist {
            sleep_interval_s: 0,
            flags: 0,
            adv_window_s: 0,
        }
    }
}

fn persist_update(f: impl FnOnce(&mut Persist)) {
    let mut p = persist_get();
    f(&mut p);
    PERSIST_INTERVAL.store(p.sleep_interval_s, PersistOrdering::Relaxed);
    PERSIST_FLAGS.store(p.flags, PersistOrdering::Relaxed);
    PERSIST_ADV_WINDOW.store(p.adv_window_s, PersistOrdering::Relaxed);
    PERSIST_MAGIC_WORD.store(PERSIST_MAGIC, PersistOrdering::Relaxed);
}

// ---------------------------------------------------------------------------
// Settings that survive a power cycle (flash, on top of the RTC RAM copy)
// ---------------------------------------------------------------------------

// RTC RAM is lost when the cell goes flat, so a sleeping board would come
// back with nothing configured and advertise at ~35 mA until it died
// again. The same two words are therefore mirrored into flash, which
// RTC RAM then caches: only a cold boot reads flash, so the wake-check
// path stays free of it.
//
// This claims the `nvs` data partition but does NOT use the ESP-IDF NVS
// key/value format - it is one fixed record at the partition start, and
// nothing else on this board reads the region. Saves are app-driven
// (rare), so rewriting the sector each time costs nothing in wear.

const NVS_MAGIC: u32 = 0x6D69_6441; // "midA"
/// Version 2 dropped the stow interval. A version 1 record fails the check
/// in `nvs_decode` and is ignored, which leaves the board awake rather than
/// reading the old stow word as something else.
///
/// Version 3 appended the advertising window. That one is a pure append, so
/// `nvs_decode` still reads a version 2 record rather than discarding it -
/// a board updated in the field keeps the cadence it was left on instead of
/// coming back advertising continuously.
const NVS_VERSION: u32 = 3;
/// magic, version, sleep, flags, adv window, crc32 - all u32, so the length
/// is already a multiple of the flash write word.
const NVS_RECORD_LEN: usize = 24;
/// Where the crc32 sits in a version 2 record, which is the version 3
/// layout minus its last word.
const NVS_V2_CRC_AT: usize = 16;

/// Resolved `nvs` partition offset, 0 = lookup failed (no partition table
/// or no such partition). Settings then degrade to RTC RAM only.
static NVS_OFFSET: PersistU32 = PersistU32::new(0);

static FLASH: Mutex<CriticalSectionRawMutex, RefCell<Option<FlashStorage<'static>>>> =
    Mutex::new(RefCell::new(None));

fn nvs_encode(p: &Persist) -> [u8; NVS_RECORD_LEN] {
    let mut rec = [0u8; NVS_RECORD_LEN];
    rec[0..4].copy_from_slice(&NVS_MAGIC.to_le_bytes());
    rec[4..8].copy_from_slice(&NVS_VERSION.to_le_bytes());
    rec[8..12].copy_from_slice(&p.sleep_interval_s.to_le_bytes());
    rec[12..16].copy_from_slice(&p.flags.to_le_bytes());
    rec[16..20].copy_from_slice(&p.adv_window_s.to_le_bytes());
    let crc = link::crc32(&rec[0..20]);
    rec[20..24].copy_from_slice(&crc.to_le_bytes());
    rec
}

fn nvs_decode(rec: &[u8; NVS_RECORD_LEN]) -> Option<Persist> {
    let word = |i: usize| u32::from_le_bytes(rec[i..i + 4].try_into().unwrap());
    if word(0) != NVS_MAGIC {
        return None;
    }
    // A version 2 record stops one word short and carries no window, which
    // reads back as the default. Its trailing bytes are erased flash, so
    // the crc has to be checked where that version put it.
    let (crc_at, adv_window_s) = match word(4) {
        2 => (NVS_V2_CRC_AT, 0),
        NVS_VERSION => (NVS_RECORD_LEN - 4, word(NVS_RECORD_LEN - 8)),
        _ => return None,
    };
    if word(crc_at) != link::crc32(&rec[0..crc_at]) {
        return None;
    }
    Some(Persist {
        sleep_interval_s: word(8),
        flags: word(12),
        adv_window_s,
    })
}

/// Read the saved settings. Called once on a cold boot; a deep-sleep wake
/// has a valid RTC RAM copy and never touches flash.
fn nvs_load() -> Option<Persist> {
    let offset = NVS_OFFSET.load(PersistOrdering::Relaxed);
    if offset == 0 {
        return None;
    }
    FLASH.lock(|f| {
        let mut f = f.borrow_mut();
        let flash = f.as_mut()?;
        let mut rec = [0u8; NVS_RECORD_LEN];
        ReadNorFlash::read(flash, offset, &mut rec).ok()?;
        nvs_decode(&rec)
    })
}

/// Persist the current settings. Erases the record's sector first, since
/// NOR flash only clears bits on write.
fn nvs_save() {
    let offset = NVS_OFFSET.load(PersistOrdering::Relaxed);
    if offset == 0 {
        return;
    }
    let rec = nvs_encode(&persist_get());
    let result = FLASH.lock(|f| {
        let mut f = f.borrow_mut();
        let Some(flash) = f.as_mut() else {
            return Err(FlashStorageError::IoError);
        };
        let sector = FlashStorage::SECTOR_SIZE;
        NorFlash::erase(flash, offset, offset + sector)?;
        NorFlash::write(flash, offset, &rec)
    });
    if result.is_err() {
        // Non-fatal: the RTC RAM copy still drives this power cycle, only
        // the survive-a-flat-battery guarantee is lost.
        println!("nvs: save failed, settings are volatile this session");
    }
}

/// Locate the `nvs` partition and adopt any saved settings. Flash reads
/// need the partition table, which is why this runs from `main` (a 3 KiB
/// buffer) rather than from the settings helpers.
fn nvs_init(flash_periph: esp_hal::peripherals::FLASH<'static>, table_buf: &mut [u8]) {
    let mut flash = FlashStorage::new(flash_periph);
    let offset = match partitions::read_partition_table(&mut flash, table_buf) {
        Ok(table) => match table.find_partition(PartitionType::Data(DataPartitionSubType::Nvs)) {
            Ok(Some(entry)) => entry.offset(),
            _ => {
                println!("nvs: no nvs partition, settings will not survive a power cycle");
                0
            }
        },
        Err(_) => {
            println!("nvs: no partition table, settings will not survive a power cycle");
            0
        }
    };
    NVS_OFFSET.store(offset, PersistOrdering::Relaxed);
    FLASH.lock(|f| f.borrow_mut().replace(flash));
}

/// Snapshot of everything the config characteristic can change, for the
/// readable settings characteristic. There is no other way for an app to
/// learn the device's current state on connect.
fn current_settings() -> ble::Settings {
    let p = persist_get();
    ble::Settings {
        pwr_en: p.flags & PFLAG_PWR_OFF == 0,
        wio_sleep: p.flags & PFLAG_WIO_SLEEP != 0,
        gps_sleep: p.flags & PFLAG_GPS_SLEEP != 0,
        sleep_interval_s: p.sleep_interval_s,
        notify_interval_ms: NOTIFY_INTERVAL_MS.lock(|c| c.get()),
        adv_window_s: adv_window_s(),
    }
}

/// Interval for the next deep sleep, 0 = stay awake and keep advertising.
fn next_sleep_interval_s() -> u32 {
    persist_get().sleep_interval_s
}

/// How long a wake check advertises for. A stored 0 means never
/// configured, not "do not advertise" - a zero window would leave a
/// sleeping board unreachable by anything but a physical reset, so it
/// resolves to the default instead.
fn adv_window_s() -> u32 {
    match persist_get().adv_window_s {
        0 => ble::ESP_ADV_DEFAULT_S,
        s => s,
    }
}

// ---------------------------------------------------------------------------
// Power / reset pins shared with the BLE handlers
// ---------------------------------------------------------------------------

static PWR_PIN: Mutex<CriticalSectionRawMutex, RefCell<Option<Output<'static>>>> =
    Mutex::new(RefCell::new(None));
static RST_PIN: Mutex<CriticalSectionRawMutex, RefCell<Option<Output<'static>>>> =
    Mutex::new(RefCell::new(None));

/// Current level of the GPS/LoRa rail, so tasks that need the WIO alive
/// (the heartbeat) can stay quiet while it is unpowered.
static RAIL_ON: AtomicBool = AtomicBool::new(false);

/// Drive the rail without touching the persisted setting. Used for the
/// states the firmware picks on its own - dark through deep sleep and
/// through a wake-check window - which must not overwrite what the app
/// last asked for.
fn drive_pwr(on: bool) {
    PWR_PIN.lock(|p| {
        if let Some(pin) = p.borrow_mut().as_mut() {
            if on {
                pin.set_high();
            } else {
                pin.set_low();
            }
        }
    });
    RAIL_ON.store(on, PersistOrdering::Relaxed);
}

/// Rail state the app asked for, i.e. what to restore on a connect.
fn pwr_configured_on() -> bool {
    persist_get().flags & PFLAG_PWR_OFF == 0
}

fn set_pwr_en(on: bool) {
    drive_pwr(on);
    persist_update(|p| {
        if on {
            p.flags &= !PFLAG_PWR_OFF;
        } else {
            p.flags |= PFLAG_PWR_OFF;
        }
    });
    nvs_save();
}

/// Hard-reset the WIO-E5 (NRST low pulse through the open-drain GPIO6).
async fn pulse_wio_reset() {
    RST_PIN.lock(|p| {
        if let Some(pin) = p.borrow_mut().as_mut() {
            pin.set_low();
        }
    });
    Timer::after(Duration::from_millis(20)).await;
    RST_PIN.lock(|p| {
        if let Some(pin) = p.borrow_mut().as_mut() {
            pin.set_high();
        }
    });
}

// ---------------------------------------------------------------------------
// LED D2 (GPIO3)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum Blink {
    /// One long blink: woke up on the sleep interval, advertising window
    /// is starting.
    Wake,
    /// Short burst: info/config received from the phone.
    Info,
    /// One immediate toggle: firmware chunk forwarded (fast blink while
    /// chunks stream).
    FwToggle,
    /// Force the LED off (end of firmware upload).
    Off,
}

static LED_CHANNEL: Channel<CriticalSectionRawMutex, Blink, 8> = Channel::new();

fn blink(b: Blink) {
    let _ = LED_CHANNEL.try_send(b);
}

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        match LED_CHANNEL.receive().await {
            Blink::Wake => {
                led.set_high();
                Timer::after(Duration::from_millis(500)).await;
                led.set_low();
            }
            Blink::Info => {
                for _ in 0..3 {
                    led.set_high();
                    Timer::after(Duration::from_millis(25)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(25)).await;
                }
            }
            Blink::FwToggle => led.toggle(),
            Blink::Off => led.set_low(),
        }
    }
}

// ---------------------------------------------------------------------------
// UART link to the WIO-E5
// ---------------------------------------------------------------------------

struct OutFrame {
    cmd: u8,
    payload: heapless::Vec<u8, 200>,
}

static OUT_CHANNEL: Channel<CriticalSectionRawMutex, OutFrame, 8> = Channel::new();

/// (is_ack, acked cmd, value-or-err) from the last ACK/NAK frame.
static ACK_SIGNAL: Signal<CriticalSectionRawMutex, (bool, u8, u16)> = Signal::new();

/// Serializes [`wio_request`] so a firmware transfer over USB and a config
/// write over BLE cannot race for the single [`ACK_SIGNAL`].
static LINK_LOCK: AsyncMutex<CriticalSectionRawMutex, ()> = AsyncMutex::new(());

/// Set while a bulk transfer (TOML or firmware) owns the WIO link, so the
/// BLE and USB paths cannot run one at the same time and corrupt the
/// firmware stream.
static FW_XFER_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Whether the config asks for verbose console logging.
///
/// The ESP never parses the config file - it forwards the bytes to the WIO
/// unread - so this tracks [`link::TELEM_FLAG_VERBOSE`] out of the WIO's
/// periodic telemetry. It starts true so a board boots talkative and stays
/// that way until the WIO says otherwise; that also means the console is
/// verbose during the window before the first heartbeat, which is exactly
/// when a board that fails to come up needs to be saying something.
static VERBOSE: AtomicBool = AtomicBool::new(true);

/// Whether verbose console lines should be emitted right now.
fn verbose_enabled() -> bool {
    VERBOSE.load(PersistOrdering::Relaxed)
}

/// Whether console output must stay quiet.
///
/// esp-println and the USB reply frames share the single 64-byte USB
/// Serial/JTAG IN FIFO with no arbitration between them, so console text
/// emitted from another task lands in the middle of a transfer's ack
/// frames and costs the host a 3 s retry per collision. During a bulk
/// transfer the frames win and everything discretionary goes silent.
fn console_busy() -> bool {
    FW_XFER_ACTIVE.load(PersistOrdering::Acquire)
}

/// Whether the WIO answered the last heartbeat ping. Cleared on boot and
/// whenever a ping goes unanswered, so the console reflects the live link
/// state (see [`heartbeat_task`]).
static LINK_UP: AtomicBool = AtomicBool::new(false);

/// Period between WIO heartbeat pings.
const HEARTBEAT_INTERVAL_S: u64 = 3;
/// How long to wait for the WIO's ping ack before treating the link as down.
const HEARTBEAT_TIMEOUT_MS: u64 = 500;

/// Latest WIO status/log lines awaiting delivery to the connected central.
/// The BLE characteristic value must match [`link::LOG_MAX`].
const _: () = assert!(link::LOG_MAX == 64);
static LOG_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 64>, 8> = Channel::new();

fn queue_frame(cmd: u8, payload: &[u8]) {
    let mut v = heapless::Vec::new();
    if v.extend_from_slice(payload).is_ok() {
        let _ = OUT_CHANNEL.try_send(OutFrame { cmd, payload: v });
    }
}

/// Send a command and wait for the WIO's ACK/NAK. Returns the ack value
/// or the midair-proto ble ack status to report.
async fn wio_request(cmd: u8, payload: &[u8], timeout_ms: u64) -> Result<u16, u8> {
    // Hold the link for the whole request/response so a concurrent caller
    // cannot consume this call's ACK.
    let _guard = LINK_LOCK.lock().await;
    ACK_SIGNAL.reset();
    let mut v = heapless::Vec::new();
    v.extend_from_slice(payload).map_err(|_| ble::ACK_BAD_STATE)?;
    OUT_CHANNEL.send(OutFrame { cmd, payload: v }).await;
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left == Duration::from_ticks(0) {
            return Err(ble::ACK_WIO_TIMEOUT);
        }
        match with_timeout(left, ACK_SIGNAL.wait()).await {
            Ok((true, c, value)) if c == cmd => return Ok(value),
            Ok((false, c, _err)) if c == cmd => return Err(ble::ACK_WIO_ERROR),
            Ok(_) => continue, // ack for something else: keep waiting
            Err(_) => return Err(ble::ACK_WIO_TIMEOUT),
        }
    }
}

/// Poll the WIO periodically so a dead or crashed link is visible on the
/// console instead of just silence. Each tick pings the WIO and waits for
/// its ack (fw version); up/down transitions are always logged, per-ping
/// detail only under the `verbose` feature. Skipped while a bulk transfer
/// owns the link (the transfer itself proves the link is alive).
#[embassy_executor::task]
async fn heartbeat_task() {
    let mut announced: Option<bool> = None;
    loop {
        Timer::after(Duration::from_secs(HEARTBEAT_INTERVAL_S)).await;
        if FW_XFER_ACTIVE.load(PersistOrdering::Acquire) {
            continue;
        }
        if !RAIL_ON.load(PersistOrdering::Relaxed) {
            // The WIO has no power - a ping could only ever time out, and
            // reporting that as a dead link would be misleading.
            continue;
        }
        let up = match wio_request(cmd::PING, &[], HEARTBEAT_TIMEOUT_MS).await {
            Ok(version) => {
                vprintln!("heartbeat: wio ack, fw v{}", version);
                true
            }
            Err(_) => {
                vprintln!("heartbeat: no ack from wio");
                false
            }
        };
        LINK_UP.store(up, PersistOrdering::Relaxed);
        if announced != Some(up) {
            announced = Some(up);
            if up {
                println!("wio link up");
                // Learn the config the WIO is running now the link is up, so
                // it is cached before any central connects - covers the ESP
                // having restarted under a WIO that was already running.
                queue_frame(cmd::CFG_READ, &[]);
            } else {
                println!("wio link down (no heartbeat ack)");
            }
        }
    }
}

/// Handle one complete frame from the WIO.
fn handle_link_frame(cmd_id: u8, payload: &[u8]) {
    vprintln!("wio frame: cmd=0x{:02x} len={}", cmd_id, payload.len());
    match cmd_id {
        msg::POSITION => {
            if payload.len() < 3 + packet::POSITION_PACKET_LEN {
                return;
            }
            let src = payload[0];
            if src == 0 {
                if let Some(p) = PositionPacket::decode(&payload[3..]) {
                    vprintln!(
                        "wio gps: fix={} sats={} lat_e7={} lon_e7={} alt_dm={} spd_cms={} tod={}ms",
                        (p.flags & packet::FLAG_FIX != 0) as u8,
                        p.sats,
                        p.lat_e7,
                        p.lon_e7,
                        p.alt_dm,
                        p.speed_cms,
                        p.tod_ms
                    );
                    GPS_STATE.lock(|c| c.set(p));
                }
            } else {
                let rssi = i16::from_le_bytes([payload[1], payload[2]]);
                if let Some(p) = PositionPacket::decode(&payload[3..]) {
                    vprintln!(
                        "wio remote node {}: rssi={} fix={} sats={} lat_e7={} lon_e7={}",
                        src,
                        rssi,
                        (p.flags & packet::FLAG_FIX != 0) as u8,
                        p.sats,
                        p.lat_e7,
                        p.lon_e7
                    );
                }
                let mut buf = [0u8; ble::REMOTE_LEN];
                buf.copy_from_slice(&payload[..ble::REMOTE_LEN]);
                REMOTE_STATE.lock(|c| c.set(buf));
            }
        }
        msg::STATUS => {
            if let Some(t) = Telemetry::decode(payload) {
                // Adopt the console verbosity the WIO's config asks for.
                // Done before the line below, so the heartbeat that carries
                // a change is already logged under the new setting.
                VERBOSE.store(t.flags & link::TELEM_FLAG_VERBOSE != 0, PersistOrdering::Relaxed);
                vprintln!(
                    "wio telem: sats={} fix={} rssi={} snr_cb={} rx={} tx={} flags=0x{:02x}",
                    t.sats,
                    (t.flags & link::TELEM_FLAG_GPS_FIX != 0) as u8,
                    t.last_rssi,
                    t.last_snr_cb,
                    t.rx_count,
                    t.tx_count,
                    t.flags
                );
                TELEM_STATE.lock(|c| c.set(t));
            }
        }
        msg::RADIO_BUSY => {
            let busy = payload.first() == Some(&1);
            let until = if busy {
                Instant::now().as_millis() + link::RADIO_BUSY_TIMEOUT_MS as u64
            } else {
                0
            };
            WIO_BUSY_UNTIL.lock(|c| c.set(until));
        }
        msg::LORA_RX => {
            // Non-position mesh traffic: just log it on the console.
            qprintln!("lora rx from {} ({} bytes)", payload.first().unwrap_or(&0), payload.len().saturating_sub(3));
        }
        msg::LOG => {
            // WIO status line: print to the USB console and hand it to the
            // BLE session to notify the connected central.
            let text = core::str::from_utf8(payload).unwrap_or("<non-utf8 status>");
            // Console only; the BLE notify below still goes out mid-transfer.
            qprintln!("wio: {}", text);
            let mut line = heapless::Vec::new();
            let n = payload.len().min(link::LOG_MAX);
            if line.extend_from_slice(&payload[..n]).is_ok() {
                let _ = LOG_CHANNEL.try_send(line);
            }
        }
        msg::CONFIG => {
            // The WIO's current radio config, relayed verbatim to the BLE
            // characteristic. Cache it and wake the session to republish.
            if payload.len() >= radiocfg::RADIO_CONFIG_LEN {
                let mut b = [0u8; radiocfg::RADIO_CONFIG_LEN];
                b.copy_from_slice(&payload[..radiocfg::RADIO_CONFIG_LEN]);
                RADIO_CONFIG_STATE.lock(|c| c.set(b));
                RADIO_CONFIG_KNOWN.store(true, PersistOrdering::Relaxed);
                RADIO_CONFIG_SIGNAL.signal(());
                vprintln!("wio config: {} bytes", payload.len());
            }
        }
        link::resp::ACK => {
            if payload.len() >= 3 {
                let value = u16::from_le_bytes([payload[1], payload[2]]);
                ACK_SIGNAL.signal((true, payload[0], value));
            }
        }
        link::resp::NAK
            if payload.len() >= 2 => {
                ACK_SIGNAL.signal((false, payload[0], payload[1] as u16));
            }
        _ => {}
    }
}

#[embassy_executor::task]
async fn link_task(mut rx: UartRx<'static, Async>, mut tx: UartTx<'static, Async>) {
    let mut parser = FrameParser::new();
    let mut out = FrameBuf::new();
    let mut buf = [0u8; 64];
    loop {
        match select(rx.read_async(&mut buf), OUT_CHANNEL.receive()).await {
            Either::First(Ok(n)) => {
                for &b in &buf[..n] {
                    if parser.feed(b) {
                        let f = parser.frame();
                        // Copy out so the parser can be fed again next loop.
                        let mut p = [0u8; link::MAX_PAYLOAD];
                        let len = f.payload.len();
                        p[..len].copy_from_slice(f.payload);
                        handle_link_frame(f.cmd, &p[..len]);
                    }
                }
            }
            Either::First(Err(_)) => {
                // RX error (overrun/noise): the frame parser resyncs on
                // the next valid frame by CRC.
            }
            Either::Second(frame) => {
                out.build(frame.cmd, &frame.payload);
                use embedded_io_async::Write as _;
                let _ = tx.write_all(out.as_bytes()).await;
                let _ = tx.flush_async().await;
            }
        }
    }
}

/// Firmware upload over the USB Serial/JTAG port. A host frames bulk ops
/// (the same wire format as the BLE bulk characteristic, [`link::usb`]) and
/// each is run through [`handle_bulk`] into the WIO, the ack framed back.
/// The console (esp-println) shares the port; the host's frame parser
/// resyncs past that text by sync byte + CRC.
/// Write one reply frame to the USB host.
///
/// The leading flush is what keeps the frame intact. esp-hal's async writer
/// stages bytes into the USB Serial/JTAG IN FIFO without checking that the
/// FIFO has room, so with a console packet still draining the hardware
/// silently drops whatever no longer fits - the host then sees a truncated
/// frame, waits out its timeout and retries the whole chunk. Only `flush`
/// tests `serial_in_ep_data_free`, so flushing first is how this waits for
/// the FIFO to actually be free.
async fn send_usb_frame(tx: &mut UsbSerialJtagTx<'static, Async>, bytes: &[u8]) {
    use embedded_io_async::Write as _;
    let _ = tx.flush().await;
    let _ = tx.write_all(bytes).await;
    let _ = tx.flush().await;
}

#[embassy_executor::task]
async fn usb_task(mut rx: UsbSerialJtagRx<'static, Async>, mut tx: UsbSerialJtagTx<'static, Async>) {
    use embedded_io_async::Read as _;
    let mut parser = FrameParser::new();
    let mut out = FrameBuf::new();
    let mut bulk: Option<BulkState> = None;
    let mut buf = [0u8; 64];
    loop {
        // While a transfer is mid-flight, bound the wait so a host that
        // vanished cannot wedge the shared transfer guard forever.
        let read = if bulk.is_some() {
            match with_timeout(Duration::from_secs(5), rx.read(&mut buf)).await {
                Ok(r) => r,
                Err(_) => {
                    bulk = None;
                    FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
                    queue_frame(cmd::FW_ABORT, &[]);
                    queue_frame(cmd::RADIO_BUSY, &[0]);
                    blink(Blink::Off);
                    println!("usb: firmware transfer timed out, aborted");
                    continue;
                }
            }
        } else {
            rx.read(&mut buf).await
        };
        let n = match read {
            Ok(n) => n,
            Err(_) => continue,
        };
        for &b in &buf[..n] {
            if !parser.feed(b) {
                continue;
            }
            // Copy the frame out so the parser (and its borrow) is free
            // across the awaits below.
            let cmd_id;
            let len;
            let mut p = [0u8; link::MAX_PAYLOAD];
            {
                let f = parser.frame();
                cmd_id = f.cmd;
                len = f.payload.len();
                p[..len].copy_from_slice(f.payload);
            }
            match cmd_id {
                link::usb::PING => {
                    out.build(link::resp::ACK, &[link::usb::PING, 1, 0]);
                    send_usb_frame(&mut tx, out.as_bytes()).await;
                }
                link::usb::INFO => {
                    let a = BLE_ADDR.lock(|c| c.get());
                    // MSB-first for display, matching the boot line.
                    let mut reply = [link::usb::INFO, 0, 0, 0, 0, 0, 0];
                    for i in 0..6 {
                        reply[1 + i] = a[5 - i];
                    }
                    out.build(link::resp::ACK, &reply);
                    send_usb_frame(&mut tx, out.as_bytes()).await;
                }
                link::usb::BULK => {
                    let (ack, alen) = handle_bulk(&p[..len], &mut bulk).await;
                    out.build(link::usb::BULK_ACK, &ack[..alen]);
                    send_usb_frame(&mut tx, out.as_bytes()).await;
                }
                _ => {}
            }
        }
    }
}

// ---------------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------------

#[gatt_server]
struct Server {
    gps: GpsService,
}

/// The gps-proto service, extended with the midair characteristics. UUIDs
/// come from the shared crates so firmware and app cannot drift.
#[gatt_service(uuid = packet::SERVICE_UUID_U128)]
struct GpsService {
    /// Position packets in the gps-proto wire format (local GPS).
    #[characteristic(uuid = packet::POSITION_UUID_U128, read, notify)]
    position: [u8; packet::POSITION_PACKET_LEN],
    /// Config commands: [id, len, value bytes].
    #[characteristic(uuid = packet::CONFIG_UUID_U128, write)]
    config: heapless::Vec<u8, 8>,
    /// Config/bulk acks: [id, status, applied value].
    #[characteristic(uuid = packet::ACK_UUID_U128, notify)]
    ack: [u8; packet::ACK_MAX_LEN],
    /// Link/radio telemetry in the midair-proto wire format.
    #[characteristic(uuid = ble::TELEMETRY_UUID_U128, read, notify)]
    telemetry: [u8; link::TELEMETRY_LEN],
    /// Bulk transfer ops (radio TOML / WIO firmware).
    #[characteristic(uuid = ble::BULK_UUID_U128, write)]
    bulk: heapless::Vec<u8, 200>,
    /// Last remote position heard over LoRa: [src, rssi i16le, packet].
    #[characteristic(uuid = ble::REMOTE_UUID_U128, read, notify)]
    remote: [u8; ble::REMOTE_LEN],
    /// Latest WIO status/log line (ASCII text).
    #[characteristic(uuid = ble::LOG_UUID_U128, read, notify)]
    log: heapless::Vec<u8, 64>,
    /// Current power/sleep settings, so an app can populate its controls
    /// on connect instead of assuming defaults.
    #[characteristic(uuid = ble::SETTINGS_UUID_U128, read, notify)]
    settings: [u8; ble::SETTINGS_LEN],
    /// The WIO's current radio configuration (RADIO.CFG settings), so an app
    /// can populate its radio editor from the board rather than a local file.
    /// All-zero until the WIO reports one (see [`RADIO_CONFIG_STATE`]).
    #[characteristic(uuid = ble::RADIO_CONFIG_UUID_U128, read, notify)]
    radio_config: [u8; radiocfg::RADIO_CONFIG_LEN],
}

// Default app descriptor required by the esp-idf 2nd-stage bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The C6's reclaimed boot-RAM region is exactly 64 KiB.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // Resolve and publish the BLE address before any task can be asked for it
    // over USB (see the info query in `usb_task`).
    let addr_bytes = ble_address();
    BLE_ADDR.lock(|c| c.set(addr_bytes));

    // Flash-backed settings. The table buffer is heap-scoped: it is 3 KiB,
    // wanted once, and too big to risk on the task stack.
    {
        let mut table_buf = alloc::vec![0u8; partitions::PARTITION_TABLE_MAX_LEN];
        nvs_init(peripherals.FLASH, &mut table_buf);
    }
    // A deep-sleep wake still holds its RTC RAM copy, so only a cold boot
    // (or a lost one) pays for the flash read.
    if PERSIST_MAGIC_WORD.load(PersistOrdering::Relaxed) != PERSIST_MAGIC {
        if let Some(saved) = nvs_load() {
            persist_update(|p| *p = saved);
            println!(
                "nvs: restored sleep {} s, adv window {} s, flags {:#x}",
                saved.sleep_interval_s,
                adv_window_s(),
                saved.flags
            );
        }
    }

    let persist = persist_get();
    let woke_from_sleep = matches!(
        esp_hal::system::wakeup_cause(),
        esp_hal::system::SleepSource::Timer
    );

    // Power rail first, then release the deep-sleep pad holds.
    //
    // A wake check comes up dark: the point of the interval is to ask
    // whether the app wants us back, which needs BLE only. The rail is
    // raised in `serve_task` if a central actually connects, so a wake
    // that nobody answers never pays for the WIO/GPS at all. A cold boot
    // follows whatever the app last configured.
    let rail_on = !woke_from_sleep && persist.flags & PFLAG_PWR_OFF == 0;
    let pwr_level = if rail_on { Level::High } else { Level::Low };
    let pwr = Output::new(peripherals.GPIO2, pwr_level, OutputConfig::default());
    let rst = Output::new(
        peripherals.GPIO6,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    );
    unsafe {
        // The Output drivers above own the pins; the holds from the last
        // deep sleep must be released after reconfiguration.
        esp_hal::peripherals::GPIO2::steal().rtcio_pad_hold(false);
        esp_hal::peripherals::GPIO6::steal().rtcio_pad_hold(false);
    }
    PWR_PIN.lock(|p| p.borrow_mut().replace(pwr));
    RST_PIN.lock(|p| p.borrow_mut().replace(rst));
    RAIL_ON.store(rail_on, PersistOrdering::Relaxed);

    let led = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    spawner.spawn(led_task(led)).expect("spawn led task");
    if woke_from_sleep {
        blink(Blink::Wake);
    }

    let mut rtc = Rtc::new(peripherals.LPWR);

    // WIO-E5 link on UART0 (GPIO16 TX / GPIO17 RX). Console output stays
    // on the USB Serial/JTAG port.
    let uart_config = UartConfig::default().with_baudrate(link::BAUD);
    let uart = Uart::new(peripherals.UART0, uart_config)
        .expect("uart init")
        .with_tx(peripherals.GPIO16)
        .with_rx(peripherals.GPIO17)
        .into_async();
    let (uart_rx, uart_tx) = uart.split();
    spawner
        .spawn(link_task(uart_rx, uart_tx))
        .expect("spawn link task");
    spawner
        .spawn(heartbeat_task())
        .expect("spawn heartbeat task");

    // USB Serial/JTAG: the console (esp-println) shares its TX; the RX half
    // lets a host push a WIO firmware image straight through the ESP.
    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let (usb_rx, usb_tx) = usb.split();
    spawner.spawn(usb_task(usb_rx, usb_tx)).expect("spawn usb task");

    let radio = esp_radio::init().expect("radio init");
    let transport =
        BleConnector::new(&radio, peripherals.BT, Default::default()).expect("ble connector");
    let controller = ExternalController::<_, 20>::new(transport);

    // Resolved and published above; print it in a fixed, parseable form.
    let address = Address::random(addr_bytes);
    println!("BLE-ADDR {}", fmt_ble_address(&addr_bytes));

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: ble::DEVICE_NAME,
        appearance: &appearance::sensor::GENERIC_SENSOR,
    }))
    .expect("gatt server");

    // Seed the readable value so a central that reads immediately after
    // discovery cannot beat the first publish in `gatt_session`.
    let _ = server
        .gps
        .settings
        .set(&server, &current_settings().encode());

    select(
        ble_host_task(&mut runner),
        serve_task(&mut peripheral, &server, &mut rtc),
    )
    .await;
    unreachable!()
}

/// Drive the BLE host stack.
async fn ble_host_task<C: Controller, P: PacketPool>(runner: &mut Runner<'_, C, P>) {
    loop {
        if runner.run().await.is_err() {
            qprintln!("ble host error, restarting");
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

/// Put the board to deep sleep for the configured interval. The GPS/LoRa
/// rail goes dark first: the WIO-E5 and MAX-M10 together draw on the order
/// of 30 mA, roughly a thousand times the sleeping C6, so leaving them up
/// would make the sleep interval pointless. GPIO2 (rail enable, now low)
/// and GPIO6 (WIO reset) are pad-held so the levels survive the sleep.
///
/// GPIO6 is held released rather than asserted: it is open-drain, and with
/// the rail dead the WIO is already held in reset by its own power-on
/// reset, so pulling the line down would only risk sinking through an
/// always-on pull-up.
fn enter_deep_sleep(rtc: &mut Rtc<'_>, interval_s: u32) -> ! {
    println!("deep sleep for {} s (gps/lora rail off)", interval_s);
    drive_pwr(false);
    unsafe {
        esp_hal::peripherals::GPIO2::steal().rtcio_pad_hold(true);
        esp_hal::peripherals::GPIO6::steal().rtcio_pad_hold(true);
    }
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(interval_s as u64));
    rtc.sleep_deep(&[&timer])
}

/// Advertise, accept one central at a time, serve it until disconnect.
/// With sleep mode active, a bounded advertising window ends in deep
/// sleep instead of advertising forever.
async fn serve_task<C: Controller>(
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
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(ble::DEVICE_NAME.as_bytes())],
        &mut scan_data,
    )
    .expect("scan data fits");

    // The advertising budget for this wake, held as a deadline rather than a
    // per-attempt timeout.
    //
    // Every path below that restarts advertising - a failed `advertise`, a
    // connection attempt that errors out, a server that will not attach -
    // loops back to the top, and none of them may extend the window. Timing
    // each attempt separately let a central that kept failing to connect reset
    // the budget forever, so the board woke, drew its full advertising current
    // and never slept again. A deadline cannot be restarted by a retry.
    // Sampled once for this wake rather than per iteration: a window shortened
    // over BLE takes effect on the next wake, so it cannot retroactively strand
    // a board mid-window with its budget already spent.
    let mut wake_ends = Instant::now() + Duration::from_secs(adv_window_s() as u64);

    loop {
        let sleep_interval = next_sleep_interval_s();
        // Budget spent, whatever used it up.
        if sleep_interval > 0 && Instant::now() >= wake_ends {
            enter_deep_sleep(rtc, sleep_interval);
        }
        qprintln!("advertising as {}", ble::DEVICE_NAME);
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

        let conn = if sleep_interval > 0 {
            let left = wake_ends.saturating_duration_since(Instant::now());
            match with_timeout(left, advertiser.accept()).await {
                Ok(Ok(c)) => c,
                Ok(Err(_)) => {
                    // A central started a connection and it did not complete.
                    // The pause keeps a repeated failure off a hot spin, and
                    // it comes out of the wake budget like everything else.
                    qprintln!("connect attempt failed");
                    Timer::after(Duration::from_millis(200)).await;
                    continue;
                }
                Err(_) => {
                    // Window expired with nobody interested.
                    enter_deep_sleep(rtc, sleep_interval);
                }
            }
        } else {
            match advertiser.accept().await {
                Ok(c) => c,
                Err(_) => {
                    qprintln!("connect attempt failed");
                    Timer::after(Duration::from_millis(200)).await;
                    continue;
                }
            }
        };

        let conn = match conn.with_attribute_server(server) {
            Ok(c) => c,
            Err(_) => continue,
        };
        qprintln!("central connected");

        // The rail is dark on every wake check; raise it now if that is
        // what the app configured. The WIO needs its boot time and the GPS
        // a cold TTFF from here, since both just came up from no power.
        if !RAIL_ON.load(PersistOrdering::Relaxed) && pwr_configured_on() {
            qprintln!("powering gps/lora rail for session");
            drive_pwr(true);
        }

        gatt_session(&conn, server).await;
        qprintln!("central disconnected");

        // Linger by advertising, not by idling. The point is to let the phone
        // come straight back, which it cannot do if the board is awake but not
        // discoverable - which is all the old `Timer::after` before sleeping
        // achieved. Looping back re-advertises, and the deadline check at the
        // top sends the board down when the linger runs out.
        wake_ends = Instant::now() + Duration::from_secs(SLEEP_LINGER_S);
    }
}

/// Refresh the settings characteristic and notify the central. Covers
/// changes the device makes on its own (a clamped interval), not just ones
/// the app asked for.
async fn publish_settings<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let value = current_settings().encode();
    if server.gps.settings.set(server, &value).is_err() {
        return;
    }
    // An unsubscribed central is not an error - the value stays readable.
    let _ = server.gps.settings.notify(conn, &value).await;
}

/// Refresh the radio-config characteristic and notify the central. A no-op
/// until the WIO has reported a config, so the characteristic never carries
/// the all-zero placeholder as if it were a real config.
async fn publish_radio_config<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    if !RADIO_CONFIG_KNOWN.load(PersistOrdering::Relaxed) {
        return;
    }
    let value = RADIO_CONFIG_STATE.lock(|c| c.get());
    if server.gps.radio_config.set(server, &value).is_err() {
        return;
    }
    let _ = server.gps.radio_config.notify(conn, &value).await;
}

/// One in-flight bulk transfer (TOML config or WIO firmware).
struct BulkState {
    kind: u8,
    crc32: u32,
    total: u32,
    received: u32,
    next_seq: u16,
}

/// Handle one connection: GATT events plus the notify ticker.
async fn gatt_session<P: PacketPool>(conn: &GattConnection<'_, '_, P>, server: &Server<'_>) {
    // Drop status lines buffered while disconnected so the central sees
    // live events, not a stale backlog.
    while LOG_CHANNEL.try_receive().is_ok() {}

    // Ask the WIO for a fresh config snapshot while it is reachable. If the
    // rail is off, a connect is about to raise it (see `serve_task`) and the
    // WIO's boot-time report arrives on its own, so skip the doomed request.
    // Clear any change signalled while disconnected; the publish below covers
    // whatever is cached now.
    RADIO_CONFIG_SIGNAL.reset();
    if RAIL_ON.load(PersistOrdering::Relaxed) {
        queue_frame(cmd::CFG_READ, &[]);
    }

    // The device may have changed things since the last session (a clamped
    // interval, settings restored from flash), so publish before serving.
    publish_settings(server, conn).await;
    publish_radio_config(server, conn).await;

    let events = async {
        let mut bulk: Option<BulkState> = None;
        loop {
            match conn.next().await {
                GattConnectionEvent::Disconnected { reason: _ } => break,
                GattConnectionEvent::Gatt { event } => {
                    let mut ack: Option<([u8; packet::ACK_MAX_LEN], usize)> = None;
                    if let GattEvent::Write(write) = &event {
                        if write.handle() == server.gps.config.handle {
                            ack = Some(apply_config(write.data()).await);
                            // Republish: the applied value may differ from
                            // the requested one after clamping.
                            publish_settings(server, conn).await;
                            blink(Blink::Info);
                        } else if write.handle() == server.gps.bulk.handle {
                            ack = Some(handle_bulk(write.data(), &mut bulk).await);
                        }
                    }
                    // Accepting lets the attribute server process the
                    // request (reads answered from the table, writes
                    // stored to it).
                    if let Ok(reply) = event.accept() {
                        reply.send().await;
                    }
                    // The ack confirms the setting/step actually applied.
                    if let Some((ack, len)) = ack {
                        let _ = server.gps.ack.notify(conn, &ack).await;
                        let _ = len;
                    }
                }
                _ => {}
            }
        }
        // Session over: make sure a dangling transfer does not leave the
        // WIO waiting or the busy flag set.
        if bulk.is_some() {
            FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
            queue_frame(cmd::FW_ABORT, &[]);
            queue_frame(cmd::RADIO_BUSY, &[0]);
            blink(Blink::Off);
        }
    };

    let notifier = async {
        let mut first = true;
        loop {
            if first {
                // Push the cached data right away so a fresh central sees
                // the latest fix/telemetry without waiting an interval
                // (notify also refreshes the read-property values).
                first = false;
                Timer::after(Duration::from_millis(200)).await;
            } else {
                let interval = NOTIFY_INTERVAL_MS.lock(|c| c.get());
                Timer::after(Duration::from_millis(u64::from(interval))).await;
            }
            // Defer discretionary notifications while the LoRa radio
            // transmits (both radios share the power budget).
            while wio_busy() {
                Timer::after(Duration::from_millis(100)).await;
            }
            let pos = GPS_STATE.lock(|c| c.get()).encode();
            if server.gps.position.notify(conn, &pos).await.is_err() {
                break;
            }
            let telem = TELEM_STATE.lock(|c| c.get()).encode();
            let _ = server.gps.telemetry.notify(conn, &telem).await;
            let remote = REMOTE_STATE.lock(|c| c.get());
            // src 0 means nothing heard yet.
            if remote[0] != 0 {
                let _ = server.gps.remote.notify(conn, &remote).await;
            }
        }
    };

    // Stream WIO status lines to the central as they arrive. Notify errors
    // are non-fatal (a central that never subscribed just misses them), so
    // this arm never ends the session on its own.
    let logger = async {
        loop {
            let line = LOG_CHANNEL.receive().await;
            let _ = server.gps.log.notify(conn, &line).await;
        }
    };

    // Republish the radio config whenever a fresh one arrives (the app
    // pushing a new config, or the WIO reporting after a connect powered it).
    // Never ends the session on its own, like the logger arm.
    let config_pub = async {
        loop {
            RADIO_CONFIG_SIGNAL.wait().await;
            publish_radio_config(server, conn).await;
        }
    };

    // Any arm ending (disconnect / position-notify failure) ends the
    // session.
    select(select3(events, notifier, logger), config_pub).await;
}

/// Apply a config write and build the ack to send back.
async fn apply_config(data: &[u8]) -> ([u8; packet::ACK_MAX_LEN], usize) {
    // Board-specific ids first; gps-proto ids as the fallback.
    if data.len() >= 2 {
        let id = data[0];
        let len = data[1] as usize;
        let value = data.get(2..2 + len).unwrap_or(&[]);
        match id {
            ble::CFG_PWR_EN => {
                let on = value.first().copied().unwrap_or(1) != 0;
                set_pwr_en(on);
                qprintln!("config: power rail {}", if on { "on" } else { "off" });
                return packet::encode_ack(id, packet::ACK_OK, &[on as u8]);
            }
            ble::CFG_WIO_SLEEP => {
                let sleep = value.first().copied().unwrap_or(0) != 0;
                persist_update(|p| {
                    if sleep {
                        p.flags |= PFLAG_WIO_SLEEP;
                    } else {
                        p.flags &= !PFLAG_WIO_SLEEP;
                    }
                });
                let result = wio_request(cmd::WIO_SLEEP, &[sleep as u8], 500).await;
                if result.is_err() && !sleep {
                    // Wake fallback: hard reset brings it back awake.
                    qprintln!("config: wio wake timed out, pulsing reset");
                    pulse_wio_reset().await;
                    return packet::encode_ack(id, packet::ACK_OK, &[0]);
                }
                return match result {
                    Ok(_) => packet::encode_ack(id, packet::ACK_OK, &[sleep as u8]),
                    Err(status) => packet::encode_ack(id, status, &[]),
                };
            }
            ble::CFG_GPS_SLEEP => {
                let sleep = value.first().copied().unwrap_or(0) != 0;
                persist_update(|p| {
                    if sleep {
                        p.flags |= PFLAG_GPS_SLEEP;
                    } else {
                        p.flags &= !PFLAG_GPS_SLEEP;
                    }
                });
                return match wio_request(cmd::GPS_SLEEP, &[sleep as u8], 500).await {
                    Ok(_) => packet::encode_ack(id, packet::ACK_OK, &[sleep as u8]),
                    Err(status) => packet::encode_ack(id, status, &[]),
                };
            }
            ble::CFG_ESP_SLEEP_S => {
                if let Ok(bytes) = <[u8; 4]>::try_from(value) {
                    let mut secs = u32::from_le_bytes(bytes);
                    if secs > 0 {
                        secs = secs.clamp(ble::ESP_SLEEP_MIN_S, ble::ESP_SLEEP_MAX_S);
                    }
                    persist_update(|p| p.sleep_interval_s = secs);
                    nvs_save();
                    qprintln!("config: esp sleep interval {} s", secs);
                    return packet::encode_ack(id, packet::ACK_OK, &secs.to_le_bytes());
                }
                return packet::encode_ack(id, packet::ACK_BAD_VALUE, &[]);
            }
            ble::CFG_ESP_ADV_WINDOW_S => {
                if let Ok(bytes) = <[u8; 4]>::try_from(value) {
                    // Clamped unconditionally: unlike the sleep interval, 0 is
                    // not an "off" here, so it comes up to the floor instead of
                    // being stored as a window nobody could ever connect in.
                    let secs =
                        u32::from_le_bytes(bytes).clamp(ble::ESP_ADV_MIN_S, ble::ESP_ADV_MAX_S);
                    persist_update(|p| p.adv_window_s = secs);
                    nvs_save();
                    qprintln!("config: advertising window {} s", secs);
                    return packet::encode_ack(id, packet::ACK_OK, &secs.to_le_bytes());
                }
                return packet::encode_ack(id, packet::ACK_BAD_VALUE, &[]);
            }
            _ => {}
        }
    }

    match packet::parse_config(data) {
        Ok(packet::ConfigCommand::UpdateIntervalMs(ms)) => {
            let applied = packet::clamp_interval(ms);
            NOTIFY_INTERVAL_MS.lock(|c| c.set(applied));
            qprintln!("config: notify interval set to {} ms", applied);
            packet::encode_ack(
                packet::CFG_UPDATE_INTERVAL_MS,
                packet::ACK_OK,
                &applied.to_le_bytes(),
            )
        }
        Err(status) => {
            let id = data.first().copied().unwrap_or(0);
            qprintln!("config: rejected write (status {})", status);
            packet::encode_ack(id, status, &[])
        }
    }
}

/// Handle one bulk characteristic write, forwarding to the WIO.
async fn handle_bulk(
    data: &[u8],
    bulk: &mut Option<BulkState>,
) -> ([u8; packet::ACK_MAX_LEN], usize) {
    let nak = |status: u8| packet::encode_ack(ble::ACK_ID_BULK, status, &[]);
    let Some(&op) = data.first() else {
        return nak(packet::ACK_BAD_VALUE);
    };
    match op {
        ble::OP_BEGIN => {
            if data.len() < 12 {
                return nak(packet::ACK_BAD_VALUE);
            }
            let kind = data[1];
            let total = u32::from_le_bytes(data[2..6].try_into().unwrap());
            let crc32 = u32::from_le_bytes(data[6..10].try_into().unwrap());
            let version = u16::from_le_bytes(data[10..12].try_into().unwrap());
            // Claim the WIO link: only one transfer (BLE or USB) at a time.
            if FW_XFER_ACTIVE.swap(true, PersistOrdering::AcqRel) {
                return nak(ble::ACK_BAD_STATE);
            }
            let result = match kind {
                ble::KIND_TOML => {
                    if total == 0 || total > 1024 {
                        FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
                        return nak(packet::ACK_BAD_VALUE);
                    }
                    wio_request(cmd::CFG_BEGIN, &(total as u16).to_le_bytes(), 1000).await
                }
                ble::KIND_FIRMWARE => {
                    let mut p = [0u8; 10];
                    p[0..4].copy_from_slice(&total.to_le_bytes());
                    p[4..8].copy_from_slice(&crc32.to_le_bytes());
                    p[8..10].copy_from_slice(&version.to_le_bytes());
                    wio_request(cmd::FW_BEGIN, &p, 1000).await
                }
                _ => {
                    FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
                    return nak(packet::ACK_BAD_VALUE);
                }
            };
            match result {
                Ok(_) => {
                    // Keep the LoRa side quiet during the transfer.
                    queue_frame(cmd::RADIO_BUSY, &[1]);
                    println!("bulk: begin kind={} total={}", kind, total);
                    *bulk = Some(BulkState {
                        kind,
                        crc32,
                        total,
                        received: 0,
                        next_seq: 0,
                    });
                    packet::encode_ack(ble::ACK_ID_BULK, packet::ACK_OK, &0u32.to_le_bytes())
                }
                Err(status) => {
                    FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
                    nak(status)
                }
            }
        }
        ble::OP_DATA => {
            let Some(state) = bulk.as_mut() else {
                return nak(ble::ACK_BAD_STATE);
            };
            if data.len() < 4 || data.len() - 3 > ble::BULK_DATA_MAX {
                return nak(packet::ACK_BAD_VALUE);
            }
            let seq = u16::from_le_bytes(data[1..3].try_into().unwrap());
            let chunk = &data[3..];
            if seq != state.next_seq {
                // Duplicate after a lost ack: re-ack; anything else is fatal.
                if seq.wrapping_add(1) == state.next_seq {
                    return packet::encode_ack(
                        ble::ACK_ID_BULK,
                        packet::ACK_OK,
                        &(state.next_seq as u32).to_le_bytes(),
                    );
                }
                return nak(packet::ACK_BAD_VALUE);
            }
            let link_cmd = if state.kind == ble::KIND_FIRMWARE {
                blink(Blink::FwToggle);
                cmd::FW_DATA
            } else {
                cmd::CFG_DATA
            };
            let mut p = [0u8; 2 + link::DATA_CHUNK];
            p[0..2].copy_from_slice(&seq.to_le_bytes());
            p[2..2 + chunk.len()].copy_from_slice(chunk);
            // Refresh the busy flag so it cannot expire mid-transfer.
            queue_frame(cmd::RADIO_BUSY, &[1]);
            match wio_request(link_cmd, &p[..2 + chunk.len()], 2000).await {
                Ok(next) => {
                    state.next_seq = next;
                    state.received += chunk.len() as u32;
                    packet::encode_ack(
                        ble::ACK_ID_BULK,
                        packet::ACK_OK,
                        &(next as u32).to_le_bytes(),
                    )
                }
                Err(status) => {
                    *bulk = None;
                    FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
                    queue_frame(cmd::RADIO_BUSY, &[0]);
                    blink(Blink::Off);
                    nak(status)
                }
            }
        }
        ble::OP_END => {
            // Inspect without consuming: a WIO timeout must leave the state
            // intact so the host can retry OP_END and reach the WIO again.
            let (received, total, kind, crc32) = match bulk.as_ref() {
                Some(s) => (s.received, s.total, s.kind, s.crc32),
                None => return nak(ble::ACK_BAD_STATE),
            };
            let finish = |bulk: &mut Option<BulkState>| {
                *bulk = None;
                FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
                queue_frame(cmd::RADIO_BUSY, &[0]);
                blink(Blink::Off);
            };
            if received != total {
                finish(bulk);
                return nak(packet::ACK_BAD_VALUE);
            }
            let result = if kind == ble::KIND_FIRMWARE {
                // CRC check over 112 KB of flash takes a moment.
                wio_request(cmd::FW_END, &[], 5000).await
            } else {
                wio_request(cmd::CFG_END, &crc32.to_le_bytes(), 2000).await
            };
            println!("bulk: end -> {:?}", result);
            match result {
                Ok(_) => {
                    finish(bulk);
                    packet::encode_ack(ble::ACK_ID_BULK, packet::ACK_OK, &[])
                }
                // Keep the transfer state + busy flag so a retried OP_END can
                // try the WIO again (the usb/BLE idle path clears it if the
                // host vanishes).
                Err(status) => nak(status),
            }
        }
        ble::OP_ABORT => {
            if bulk.take().is_some() {
                FW_XFER_ACTIVE.store(false, PersistOrdering::Release);
                queue_frame(cmd::FW_ABORT, &[]);
                queue_frame(cmd::RADIO_BUSY, &[0]);
                blink(Blink::Off);
            }
            packet::encode_ack(ble::ACK_ID_BULK, packet::ACK_OK, &[])
        }
        _ => nak(packet::ACK_BAD_VALUE),
    }
}
