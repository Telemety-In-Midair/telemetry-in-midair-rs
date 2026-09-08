//! The BLE side: the GATT service, the duty cycle and the serve loop.
//!
//! The policy - what a connect, a fizzled handshake, an expiry, a nap or a
//! moved mode means on this mode's budget - is [`midair_proto::session`],
//! host-tested and walked exhaustively by the state space tests. What is
//! here is the advertising, the waiting and the effects: the trouble-host
//! stack, the characteristics, and the `Rtc` a deep sleep needs.

use bt_hci::controller::ExternalController;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use esp_hal::rtc_cntl::Rtc;
use gps_proto::packet;
use midair_proto::ble::{self, Mode};
use midair_proto::bulk::Owner;
use midair_proto::posture::Request;
use midair_proto::radiocfg;
use midair_proto::roster::Value;
use midair_proto::session::{self, Accepted, Next, ServeCommand, Then};
use midair_proto::link;
use trouble_host::prelude::*;

use crate::sleep::enter_deep_sleep;
use crate::{settings, state, xfer};

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

/// Build-time BLE address override, most-significant octet first (e.g.
/// "FF:C6:A1:53:50:47"). `Some` only when `BLE_ADDRESS` was set at build
/// time (build.rs validates and normalizes it, and emits nothing otherwise);
/// `None` derives a per-chip address from the eFuse MAC instead.
const BLE_ADDRESS_OVERRIDE: Option<&str> = option_env!("BLE_ADDRESS");

/// The board's BLE address as the LSB-first array `Address::random` expects.
///
/// With no build-time override it is derived from the chip's factory MAC in
/// eFuse, so every board is unique out of the box: the MAC's most-
/// significant octet gets its top two bits set, which is all a static-random
/// address requires, and the per-chip low bytes keep it distinct.
pub fn address() -> [u8; 6] {
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
pub fn fmt_address(a: &[u8; 6]) -> heapless::String<17> {
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
    ///
    /// A `Vec` rather than an array for the same reason the log line is:
    /// the blob is past the 32 bytes an array can be `Default` for, which
    /// the service macro needs, and always sent whole.
    #[characteristic(uuid = ble::RADIO_CONFIG_UUID_U128, read, notify)]
    radio_config: heapless::Vec<u8, { radiocfg::RADIO_CONFIG_LEN }>,
    /// What this board is called, as it advertises it.
    ///
    /// A connected app should not have to have kept the scan around to know
    /// which board it is talking to, and one that has just renamed a board
    /// sees the result here rather than waiting a window for the scan
    /// response to catch up.
    #[characteristic(uuid = ble::NAME_UUID_U128, read, notify)]
    name: heapless::Vec<u8, { ble::NAME_MAX }>,
}

/// The settings characteristic value: what an app reads on connect.
fn current_settings() -> ble::Settings {
    settings::get().settings(state::notify_interval_ms())
}

/// The BLE duty cycle. Never returns.
///
/// BLE measures 71 mA of this board's 126. Two things cut into that, and
/// they cut into different parts of it.
///
/// Modem sleep is the controller powering its own PHY down in the gaps it
/// knows about - between advertisements, and between the connection events
/// of a connection with nothing to say. It costs nothing in reachability,
/// because the controller is still counting and still wakes for every
/// event it promised. It cannot help while a transfer is actually moving.
///
/// The loop here is the other one, and the only way to zero:
/// `BleConnector`'s `Drop` calls `ble_deinit` and takes the `PhyInitGuard`
/// with it, so the whole stack is built inside the loop and dropped at the
/// bottom of it. That does cost reachability - a board in a dark period
/// cannot be connected to at all.
///
/// [`serve`] returns for a BLE-down period once the window is spent and
/// `ble_off_s` is set; with it at 0 it never returns and this is one pass.
///
/// Unlike deep sleep this stops nothing else. The hardware task keeps
/// beaconing, the GPS keeps tracking and the card keeps logging - the
/// board stays a working tracker and only stops being connectable.
pub async fn duty_cycle(rtc: &mut Rtc<'_>, addr_bytes: [u8; 6]) -> ! {
    // The attribute table is built once and reused by every window.
    //
    // Not an optimization - a requirement. The `#[gatt_service]` macro backs
    // each characteristic with its own `static StaticCell`, so a second
    // `Server::new_with_config` panics ("already full, it can't be
    // initialized twice") rather than returning an error. It borrows nothing
    // from the per-window stack, so hoisting it costs nothing either.
    //
    // The GAP device name is the one surface that cannot follow a rename:
    // this string is copied into the table. A board renamed while running
    // advertises and reports its new name immediately - the scan response
    // and the name characteristic are both rebuilt - and only the GAP
    // characteristic, which nothing here reads, catches up at the next
    // boot.
    let boot_name = settings::name();
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: &boot_name,
        appearance: &appearance::sensor::GENERIC_SENSOR,
    }))
    .expect("gatt server");

    let mut announced_modem_sleep = false;
    loop {
        let down = {
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
            // still owns and has never used.
            let bt = unsafe { esp_hal::peripherals::BT::steal() };

            // Declared before the stack so it outlives it: dropping the
            // connector needs the radio still initialized, and locals drop
            // in reverse declaration order.
            //
            // Neither of these may panic. They run every window - thousands
            // of times a day on a deployed board - so a transient failure
            // has to cost one window rather than the whole node. A tracker
            // that stops beaconing because its BLE modem would not come
            // back up is a worse outcome than one nobody can connect to.
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

            match select(
                async {
                    loop {
                        if runner.run().await.is_err() {
                            qprintln!("ble host error, restarting");
                            Timer::after(Duration::from_millis(200)).await;
                        }
                    }
                },
                serve(&mut peripheral, &server, rtc),
            )
            .await
            {
                Either::First(()) => None,
                Either::Second(off_s) => Some(off_s),
            }
        };

        // Everything above is dropped by here, `ble_deinit` included.
        //
        // `serve` only returns for a BLE-down period; the runner never
        // returns at all. Anything else goes straight back to advertising.
        let Some(off_s) = down else {
            continue;
        };
        status_println!("ble down for {} s (lora and gps stay up)", off_s);
        // A board told to sleep - or told to change mode - over the USB
        // console must not have to wait out the whole dark period first.
        // The console is alive throughout, and while the modem is down it is
        // the only way in.
        if let Either::Second(command) = select(
            Timer::after(Duration::from_secs(off_s as u64)),
            state::next_command(),
        )
        .await
        {
            match session::during_ble_down(command) {
                Then::Sleep(secs) => {
                    status_println!("sleep on command: {} s, from a BLE-down period", secs);
                    enter_deep_sleep(rtc, secs).await
                }
                // A mode that is no longer tracking has no BLE-down period
                // to sit out. Bring the modem straight back up; `serve`
                // re-budgets on the new mode.
                _ => status_println!("mode changed, ending the BLE-down period early"),
            }
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

/// Advertise, accept one central, serve it, repeat. Returns the length of
/// a BLE-down period when the budget ends in one; deep sleeps, and so does
/// not return, when it ends in a sleep.
///
/// What a spent budget means is a property of the mode, not a race between
/// two settings ([`session::Stored::at_expiry`]):
///
/// - **Stored** - a wake check. The budget is the advertising window, and
///   spending it is deep sleep again.
/// - **Idle** - the budget is the idle timeout, and spending it stores the
///   board.
/// - **Tracking** - the budget is the advertising window and spending it
///   returns, so the caller can drop the BLE stack for `ble_off_s` and call
///   this again. The board stays a working tracker throughout.
///
/// With no cadence to sleep on (`sleep_interval_s = 0`) the first two
/// simply keep advertising, which is what a bench board and an unconfigured
/// board both want.
async fn serve<C: Controller>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    server: &Server<'_>,
    rtc: &mut Rtc<'_>,
) -> u32 {
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

    let mut serve = session::Serve::new(Instant::now().as_millis(), &settings::get());

    loop {
        let stored = settings::get();
        let (rebudgeted, next) = serve.pass(Instant::now().as_millis(), &stored);
        if rebudgeted {
            status_println!("mode {} ({} s budget)", serve.mode().as_str(), stored.budget_s());
        }
        // `bounded` is whether the budget ends in anything. It does not for
        // a board that advertises forever - a bench board, or an
        // unconfigured one - and waiting on `accept` with no deadline is
        // then the whole intent.
        let bounded = match next {
            Next::Sleep { interval_s } => enter_deep_sleep(rtc, interval_s).await,
            // Cheaper exit than deep sleep and it stops nothing else: hand
            // control back so the caller can drop the stack. The board stays
            // awake and on the air over LoRa; only the modem goes.
            Next::BleDown { off_s } => return off_s,
            Next::Advertise { bounded } => bounded,
        };
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

        // Waiting for a central is also waiting for a command: a board told
        // to sleep over the USB console with nobody connected goes down
        // without first having to be connected to, and a mode written while
        // nothing is connected does not wait out the whole budget - ten
        // minutes in idle - before the loop notices.
        let accept = async {
            if bounded {
                let left = Duration::from_millis(serve.remaining_ms(Instant::now().as_millis()));
                // `None` is the window expiring with nobody interested.
                with_timeout(left, advertiser.accept()).await.ok()
            } else {
                Some(advertiser.accept().await)
            }
        };
        let accepted = select(accept, state::next_command()).await;

        let outcome = match &accepted {
            Either::First(Some(Ok(_))) => Accepted::Connected,
            Either::First(Some(Err(_))) => Accepted::Failed,
            Either::First(None) => Accepted::Expired,
            Either::Second(c) => Accepted::Command(*c),
        };
        let step = serve.on_accept(Instant::now().as_millis(), outcome, &stored);

        // A connect during a wake check is a doorbell, not a leash: the
        // attempt alone promotes the board to idle with the timeout armed,
        // and the app can take its time. Raises what a wake check left
        // down - the card, so config reads and log pulls work. The GPS and
        // the radio stay parked: an app looking at a stored object's
        // settings should not cost an acquisition.
        if step.promote {
            settings::set_mode(Mode::Idle);
            state::request(Request::Mode(Mode::Idle));
            status_println!(
                "promoted to idle by a connect ({} s before it stores itself)",
                settings::get().budget_s()
            );
        }

        let conn = match (accepted, step.then) {
            (Either::First(Some(Ok(c))), Then::Serve) => c,
            // The pause keeps a repeated failure off a hot spin.
            (_, Then::Retry) => {
                qprintln!("connect attempt failed, holding the window open");
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            (_, Then::Sleep(secs)) => {
                if matches!(outcome, Accepted::Command(_)) {
                    status_println!("sleep on command: {} s, from advertising", secs);
                }
                enter_deep_sleep(rtc, secs).await
            }
            (_, Then::BleDown(off_s)) => {
                qprintln!("advertising window over");
                return off_s;
            }
            // A mode write: the top of the loop re-budgets on it. `Serve`
            // without a connection cannot happen - the policy only says it
            // for a connect - and is answered the same way.
            (_, Then::Continue | Then::Serve) => continue,
        };

        let Ok(conn) = conn.with_attribute_server(server) else {
            // A central connected and the attribute server did not attach -
            // the link dropped in between, or the stack is out of room.
            // Held open like a fizzled handshake: something was connecting,
            // and letting the budget expire here sends the board dark or
            // asleep on a phone that is about to try again. Paused like one
            // too, so a stack that keeps refusing is not a hot spin.
            serve.on_accept(Instant::now().as_millis(), Accepted::Failed, &stored);
            qprintln!("attribute server did not attach, holding the window open");
            Timer::after(Duration::from_millis(200)).await;
            continue;
        };
        qprintln!("central connected");
        let nap = gatt_session(&conn, server).await;
        qprintln!("central disconnected");

        // A transfer the phone was midway through does not outlive it.
        xfer::abort(Owner::Ble, Instant::now().as_millis()).await;

        // The session may have ended because the app asked for a sleep
        // rather than because the phone went away. Checked here rather than
        // inside `gatt_session` so the ack, the settings republish and the
        // link teardown have all already happened - the board is gone the
        // moment this runs, and anything still owed to the central has to
        // have left first. Otherwise re-arm by advertising, not by idling:
        // the point is to let the phone come straight back.
        if let Then::Sleep(secs) =
            serve.on_session_end(Instant::now().as_millis(), nap, &settings::get())
        {
            status_println!("sleep on command: {} s", secs);
            enter_deep_sleep(rtc, secs).await;
        }
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
    let Some(blob) = state::radio_config() else {
        return;
    };
    // Cannot fail: the buffer is exactly the blob's length.
    let value = heapless::Vec::from_slice(&blob).unwrap_or_default();
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
/// Returns the nap a `CFG_SLEEP_NOW` asked for during the session, which
/// is what ended it.
async fn gatt_session<P: PacketPool>(conn: &GattConnection<'_, '_, P>, server: &Server<'_>) -> Option<u32> {
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
                            let (ack, _) = crate::config::apply_config(&data[..len]).await;
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
            // problem.
            //
            // Held, not skipped. This used to `continue` to the next
            // interval, which with a beacon every second and 289 ms on
            // air dropped up to half the position notifications - which
            // ones depended on where the tick fell in the slot, so the
            // phone saw a board that reported every second, or every
            // other second, or every third, for no visible reason.
            while state::radio_busy() {
                Timer::after(Duration::from_millis(5)).await;
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
    // phone a supervision timeout instead of a disconnect. A moved mode
    // does not: the session goes on, and the loop re-budgets after it.
    //
    // The pause is what makes the ack useful. `events` sends the ack and
    // republishes the settings *after* it has already applied the write, so
    // the command this waits on arrives while the notification that
    // explains it is still queued. Ending the session immediately would
    // drop that notification and leave the app with a write it never heard
    // back from, which is indistinguishable from a board that crashed.
    let commanded_sleep = async {
        loop {
            if let ServeCommand::SleepNow(secs) = state::next_command().await {
                Timer::after(Duration::from_millis(400)).await;
                return secs;
            }
        }
    };

    // Any arm ending (disconnect, a position notify that failed, or a
    // commanded sleep) ends the session.
    match select3(
        select3(events, notifier, logger),
        select(config_pub, commanded_sleep),
        remotes,
    )
    .await
    {
        Either3::Second(Either::Second(secs)) => Some(secs),
        _ => None,
    }
}
