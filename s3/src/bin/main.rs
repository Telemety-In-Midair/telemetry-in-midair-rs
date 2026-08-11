//! Wio-S3 firmware for the wio-s3-max-gps board.
//!
//! One module replaces the ESP32-C6 and WIO-E5 pair, so this binary holds
//! what both of them did: BLE, LoRa, GPS and SD. What it does *not* hold is
//! everything the split cost - there is no framed UART link, no heartbeat
//! proving it alive, no ack/retry around every command, and no second
//! firmware image to push. A config write that used to be a link frame and
//! a wait for an answer is now a signal the hardware loop picks up.
//!
//! Pin assignments come from the board, not from preference:
//!
//! - D5 (GPIO43) and D2 (GPIO14) are **active low** - the LED anodes sit on
//!   +3V3 through R21/R20, so driving the pin low is what lights them.
//! - GPIO43 is also UART0_TX, which is why the console is on the USB
//!   Serial/JTAG port (GPIO19/20 to the USB-C J3) instead. Expect the ROM
//!   bootloader's own log to flicker D5 on every reset.
//!
//! Not yet ported from the C6 build: bulk transfer (radio TOML over the
//! bulk characteristic), deep sleep with its nvs-backed settings, the
//! remote-node roster, and the USB console. See `PORT-WIO-S3.md`.

#![no_std]
#![no_main]

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_println::println;
use gps_proto::packet;
use midair_proto::radiocfg::RadioConfig;
use midair_proto::{ble, link, radiocfg, session};
use trouble_host::prelude::*;
use wio_s3_gps::gps::{Gps, BAUD as GPS_BAUD};
use wio_s3_gps::radio::Sx1262Driver;
use wio_s3_gps::sdlog::SdLog;
use wio_s3_gps::state::{self, Request};
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

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

/// How often a connected central is sent a fresh position and telemetry.
const NOTIFY_INTERVAL_MS: u32 = 1_000;

/// Power/sleep settings. The C6 keeps this in RTC RAM with an nvs backup so
/// it survives deep sleep and a flat cell; deep sleep is not ported yet, so
/// for now it lives only here and resets with the board.
static mut STORED: session::Stored = session::Stored::new();

#[allow(static_mut_refs)]
fn stored() -> &'static mut session::Stored {
    // Only the GATT session touches this, and there is one at a time.
    unsafe { &mut STORED }
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
    #[characteristic(uuid = packet::CONFIG_UUID_U128, write)]
    config: heapless::Vec<u8, 8>,
    /// Config/bulk acks: [id, status, applied value].
    #[characteristic(uuid = packet::ACK_UUID_U128, notify)]
    ack: [u8; packet::ACK_MAX_LEN],
    /// Radio telemetry in the midair-proto wire format.
    #[characteristic(uuid = ble::TELEMETRY_UUID_U128, read, notify)]
    telemetry: [u8; link::TELEMETRY_LEN],
    /// Bulk transfer ops. Declared so the service shape matches the C6's;
    /// the handler is not ported yet.
    #[characteristic(uuid = ble::BULK_UUID_U128, write)]
    bulk: heapless::Vec<u8, 200>,
    /// Last remote position heard over LoRa.
    #[characteristic(uuid = ble::REMOTE_UUID_U128, read, notify)]
    remote: [u8; ble::REMOTE_LEN_V2],
    /// Last ping heard from a node with no fix.
    #[characteristic(uuid = ble::NODE_PING_UUID_U128, read, notify)]
    node_ping: [u8; ble::NODE_PING_LEN],
    /// Latest status/log line (ASCII text).
    #[characteristic(uuid = ble::LOG_UUID_U128, read, notify)]
    log: heapless::Vec<u8, 128>,
    /// Current power/sleep settings, so an app can populate its controls
    /// on connect instead of assuming defaults.
    #[characteristic(uuid = ble::SETTINGS_UUID_U128, read, notify)]
    settings: [u8; ble::SETTINGS_LEN],
    /// The current radio configuration, so an app can populate its radio
    /// editor from the board rather than a local file.
    #[characteristic(uuid = ble::RADIO_CONFIG_UUID_U128, read, notify)]
    radio_config: [u8; radiocfg::RADIO_CONFIG_LEN],
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // D5. Starts dark so the first blink is visibly the firmware's, not a
    // leftover level from the ROM bootloader driving UART0_TX.
    let d5 = Output::new(peripherals.GPIO43, LED_OFF, OutputConfig::default());

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
    let lora_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_hz(LORA_SPI_HZ))
            .with_mode(Mode::_0),
    )
    .expect("lora spi")
    .with_sck(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO6)
    .with_miso(peripherals.GPIO7);

    let lora = Sx1262Driver::new(Sx1262::new(
        lora_spi,
        Output::new(peripherals.GPIO8, Level::High, OutputConfig::default()),
        Input::new(peripherals.GPIO4, InputConfig::default()),
        Input::new(peripherals.GPIO10, InputConfig::default()),
        Output::new(peripherals.GPIO9, Level::High, OutputConfig::default()),
    ));

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
    let sdlog = SdLog::new(embedded_sdmmc::SdCard::new(sd_dev, Delay::new()));

    spawner
        .spawn(hardware_task(lora, gps, sdlog, d5))
        .expect("spawn hardware task");

    // BLE. Same stack the C6 runs.
    let radio = esp_radio::init().expect("radio init");
    let transport =
        esp_radio::ble::controller::BleConnector::new(&radio, peripherals.BT, Default::default())
            .expect("ble connector");
    let controller = ExternalController::<_, 20>::new(transport);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources);
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
        .set(&server, &stored().settings(NOTIFY_INTERVAL_MS).encode());

    let _ = select(
        async {
            loop {
                if runner.run().await.is_err() {
                    Timer::after(Duration::from_millis(200)).await;
                }
            }
        },
        serve(&mut peripheral, &server),
    )
    .await;

    // Neither side of that select ever finishes.
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Advertise, accept one central, serve it, repeat.
///
/// The C6 wraps this in a `session::Window` so an unattended board can give
/// up and deep sleep. Deep sleep is not ported yet, so for now the board
/// advertises indefinitely - which is exactly what the C6 does with
/// `sleep_interval_s = 0`, its default.
async fn serve<C: Controller>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    server: &Server<'_>,
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

    loop {
        println!("advertising as {}", ble::DEVICE_NAME);
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
                println!("advertise failed, retrying");
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };
        let conn = match advertiser.accept().await {
            Ok(c) => c,
            Err(_) => {
                // A central started a connection and it did not complete.
                // The pause keeps a repeated failure off a hot spin.
                println!("connect attempt failed");
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
        };
        let Ok(conn) = conn.with_attribute_server(server) else {
            continue;
        };
        println!("central connected");
        gatt_session(&conn, server).await;
        println!("central disconnected");
    }
}

/// One connection: publish what an app needs on arrival, then stream.
async fn gatt_session<P: PacketPool>(conn: &GattConnection<'_, '_, P>, server: &Server<'_>) {
    // Publish before anything else, so an app can populate its controls
    // without waiting for a notify interval.
    let settings = stored().settings(NOTIFY_INTERVAL_MS).encode();
    let _ = server.gps.settings.set(server, &settings);
    let _ = server.gps.settings.notify(conn, &settings).await;

    let mut next_notify = Instant::now();

    loop {
        // Either the central said something, or the notify interval came
        // round. Nothing else can wake this.
        match select(conn.next(), Timer::at(next_notify)).await {
            Either::First(event) => match event {
                GattConnectionEvent::Disconnected { .. } => return,
                GattConnectionEvent::Gatt { event } => {
                    let is_config = matches!(&event, GattEvent::Write(w) if w.handle() == server.gps.config.handle);
                    let data_ack = if is_config {
                        let GattEvent::Write(w) = &event else {
                            unreachable!()
                        };
                        Some(apply_config(w.data()))
                    } else {
                        None
                    };
                    let _ = event.accept().map(|reply| reply.send());
                    if let Some((ack, _len)) = data_ack {
                        let _ = server.gps.ack.notify(conn, &ack).await;
                        // The write may have changed something the settings
                        // characteristic reports.
                        let settings = stored().settings(NOTIFY_INTERVAL_MS).encode();
                        let _ = server.gps.settings.set(server, &settings);
                        let _ = server.gps.settings.notify(conn, &settings).await;
                    }
                }
                _ => {}
            },
            Either::Second(_) => {
                next_notify += Duration::from_millis(NOTIFY_INTERVAL_MS as u64);

                // Hold notifications while the radio has the air. A LoRa
                // transmit at 22 dBm beside a 2.4 GHz radio is a supply
                // problem; on the two-MCU board this was a `RADIO_BUSY`
                // link message, here it is a bool.
                if state::radio_busy() {
                    continue;
                }

                let (position, dirty) = state::take_position();
                if let Some(p) = position
                    && dirty
                {
                    let _ = server.gps.position.notify(conn, &p.encode()).await;
                }
                if let Some(t) = state::telemetry() {
                    let _ = server.gps.telemetry.notify(conn, &t.encode()).await;
                }
            }
        }
    }
}

/// Apply a config-characteristic write.
///
/// The decision - what is legal, what it clamps to, what the ack says - is
/// [`session::apply`], the same host-tested policy the C6 runs. What
/// changed is the far side: an action that used to be a link frame and a
/// wait for the WIO's answer is now a signal to the hardware loop, so the
/// ack the policy built always holds.
fn apply_config(data: &[u8]) -> ([u8; packet::ACK_MAX_LEN], usize) {
    let outcome = session::apply(stored(), data);
    match outcome.action {
        session::Action::GpsSleep(on) => state::request(Request::GpsSleep(on)),
        // No second MCU to put to sleep; the nearest thing is parking the
        // radio, which is what the WIO's soft sleep actually bought.
        session::Action::WioSleep(on) => state::request(Request::RadioStandby(on)),
        // No host-controlled rail on this board - the GPS and SD sit
        // directly on +3V3. See BOARD-REVIEW.md in the board repo.
        session::Action::Rail(_) => println!("config: rail control has no hardware here"),
        _ => {}
    }
    (outcome.ack, outcome.ack_len)
}

/// Everything that is not BLE: the radio, the GPS and the card.
///
/// Owning them in one task is what removes the link protocol. The BLE
/// session never touches the hardware; it reads the snapshot this publishes
/// and signals back through [`state::Request`].
#[embassy_executor::task]
async fn hardware_task(
    mut lora: Sx1262Driver<'static>,
    mut gps: Gps<'static>,
    mut sdlog: SdLog<'static>,
    mut d5: Output<'static>,
) {
    let cfg = RadioConfig::default();
    lora.init(&cfg).await;
    if !lora.print_diagnostics() {
        println!("radio did not answer - check the pin map in main");
    }
    gps.configure(&cfg.gps).await;

    let mut buf = [0u8; 255];
    let mut rx_count: u32 = 0;
    let mut last_rssi: i16 = 0;
    let start = Instant::now();

    loop {
        if let Some(r) = state::take_request() {
            match r {
                Request::GpsSleep(true) => gps.sleep(),
                Request::GpsSleep(false) => gps.wake().await,
                Request::RadioStandby(true) => lora.standby(),
                Request::RadioStandby(false) => lora.init(&cfg).await,
            }
        }

        gps.poll();
        if gps.take_updated() {
            let p = gps.packet();
            state::set_position(p);
            sdlog.log_position(start.elapsed().as_millis() as u32, 0, 0, &p);
        }

        if let Some((len, rssi)) = lora.poll_recv(&mut buf) {
            rx_count = rx_count.saturating_add(1);
            last_rssi = rssi;
            println!("rx {} bytes, {} dBm", len, rssi);
            // Blink D5 on receive, as the WIO did on D6.
            d5.set_level(LED_ON);
            Timer::after(Duration::from_millis(20)).await;
            d5.set_level(LED_OFF);
        }

        state::set_telemetry(link::Telemetry {
            last_rssi,
            last_snr_cb: lora.last_snr_cb(),
            secs_since_rx: 0,
            rx_count,
            tx_count: 0,
            flags: 0,
            sats: gps.packet().sats,
        });

        sdlog.poll(start.elapsed().as_millis() as u32);
        Timer::after(Duration::from_millis(10)).await;
    }
}
