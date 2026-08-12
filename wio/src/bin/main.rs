//! WIO-E5 firmware for the telemetry-in-midair board.
//!
//! - Reads the MAX-M10 GPS on USART1 (PB6 TX / PB7 RX, EXTINT on PB10).
//! - Broadcasts positions over 915 MHz LoRa and blinks D6 (PA9) on RX,
//!   D5 (PA10) on TX. The module's antenna switch is driven from PA4/PA5.
//!   Nodes are leaves by default and hear each other directly; one
//!   configured as a repeater extends that range.
//! - Logs own and remote positions to a FAT SD card (SPI1 + PA0 CS).
//! - Talks to the ESP32-C6 on USART2 (PA2 TX / PA3 RX): positions and
//!   status out; sleep/config/firmware commands in. Radio-busy flags run
//!   both ways so the two radios can avoid transmitting at once.
//! - Radio parameters come from `RADIO.CFG` on the SD card and/or a
//!   config pushed over the link (which is also saved back to SD).
//! - Accepts firmware images over the link into the DFU partition; the
//!   swap bootloader (see `bootloader/`) installs them on reboot.

#![no_std]
#![no_main]
#![warn(clippy::large_stack_frames)]

use panic_halt as _;

#[macro_use]
extern crate wio_e5_gps;

#[rtic::app(device = stm32wlxx_hal::pac, dispatchers = [DAC])]
mod app {
    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);

    use midair_proto::link::{self, cmd, msg, Telemetry};
    use midair_proto::lora;
    use midair_proto::radiocfg::{self, RadioConfig, Role};
    use rtt_target::{rprintln, rtt_init, set_print_channel};
    use stm32wlxx_hal::{
        gpio::{PortA, PortB},
        pac::{FLASH, IWDG},
        subghz::SubGhz,
    };
    use wio_e5_gps::cfgstore;
    use wio_e5_gps::cfgxfer::{CfgEvent, CfgTransfer};
    use wio_e5_gps::esplink::{self, EspLink, RxProducer, RxQueue};
    use wio_e5_gps::fwupdate::{FwEvent, FwUpdate};
    use wio_e5_gps::gps::Gps;
    use wio_e5_gps::leds::Leds;
    use wio_e5_gps::platform::{self, SYSCLK_HZ};
    use wio_e5_gps::radio::{RfSwitch, Sx1262Driver};
    use wio_e5_gps::sdcard::SdCard;
    use wio_e5_gps::sdlog::SdLog;
    use wio_e5_gps::watchdog;
    use wio_e5_gps::{status_println, Node, FIRMWARE_VERSION};

    /// `a` happened at or after deadline `b` in wrapping-u32 time.
    fn due(now: u32, deadline: u32) -> bool {
        now.wrapping_sub(deadline) < 0x8000_0000
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        node: Node<Sx1262Driver>,
        gps: Gps,
        sdlog: SdLog,
        esp: EspLink,
        leds: Leds,
        flash: FLASH,
        iwdg: IWDG,
        fw: FwUpdate,
        cfgxfer: CfgTransfer,
        cfg: RadioConfig,
        cfg_loaded: bool,
        /// Producer half of the ESP-link RX ring buffer (USART2 ISR side).
        esp_rx_prod: RxProducer,
    }

    #[init(local = [esp_rx_q: RxQueue = RxQueue::new()])]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let channels = rtt_init! {
            up: {
                0: { size: 1024, name: "Terminal" }
            }
        };
        set_print_channel(channels.up.0);
        rprintln!("wio-e5-gps v{} starting", FIRMWARE_VERSION);

        // DWT cycle counter drives platform::millis()/random().
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        let dp = cx.device;
        let mut rcc = dp.RCC;

        // 16 MHz for SD SPI throughput; before the monotonic starts.
        platform::raise_sysclk(&mut rcc);
        // The USARTs are clocked from HSI16 (see EspLink/Gps); enable it
        // before either UART is created or their first send hangs forever.
        platform::enable_hsi16(&mut rcc);
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let mut flash = dp.FLASH;

        // Watchdog first: a hang anywhere in init resets us, and the
        // bootloader reverts unconfirmed firmware.
        let iwdg = dp.IWDG;
        watchdog::start(&iwdg, 6_000);
        wio_e5_gps::boot_state::confirm_boot(&mut flash);

        let gpioa = PortA::split(dp.GPIOA, &mut rcc);
        let gpiob = PortB::split(dp.GPIOB, &mut rcc);

        // SD card + FAT (optional; retries in the loop when absent).
        let mut sdlog = cortex_m::interrupt::free(|cs| {
            SdLog::new(SdCard::new(
                dp.SPI1, gpiob.b3, gpiob.b4, gpiob.b5, gpioa.a0, &mut rcc, cs,
            ))
        });
        watchdog::feed(&iwdg);
        sdlog.poll(platform::millis());
        watchdog::feed(&iwdg);

        // Radio config: the SD file, else the flash backup, else defaults.
        // The card wins when both exist - pulling it to edit RADIO.CFG on a
        // computer has to do what it looks like it does - and the flash copy
        // is what carries a board with no card across a power cycle.
        let mut cfg_buf = [0u8; wio_e5_gps::sdlog::CONFIG_MAX];
        let from_sd = sdlog
            .read_config(&mut cfg_buf)
            .and_then(|n| match radiocfg::parse_bytes(&cfg_buf[..n]) {
                Ok(c) => Some(c),
                Err(e) => {
                    rprintln!("Config: RADIO.CFG invalid ({:?}), trying flash", e);
                    None
                }
            });
        let (cfg, cfg_loaded) = match from_sd {
            Some(c) => {
                rprintln!("Config: RADIO.CFG loaded (address {})", c.address);
                (c, true)
            }
            None => match cfgstore::read(&mut cfg_buf).and_then(|n| radiocfg::parse_bytes(&cfg_buf[..n]).ok()) {
                Some(c) => {
                    rprintln!("Config: flash backup loaded (address {})", c.address);
                    (c, true)
                }
                None => {
                    rprintln!("Config: none stored, using defaults");
                    (RadioConfig::default(), false)
                }
            },
        };
        // Honor sd_enabled only now: the setting itself lives on the card,
        // so the card has to be read before it can say to stop using it.
        if !cfg.sd_enabled {
            rprintln!("SD: disabled by config");
            sdlog.disable(platform::millis());
        }

        // SubGHz radio (integrated SX1262) and the module's antenna switch.
        let sg = SubGhz::new(dp.SPI3, &mut rcc);
        let rf_switch = cortex_m::interrupt::free(|cs| RfSwitch::new(gpioa.a4, gpioa.a5, cs));
        let mut radio = Sx1262Driver::new(sg, rf_switch);
        radio.init(&cfg);
        radio.print_diagnostics();
        watchdog::feed(&iwdg);

        // ESP-link RX ring buffer: the USART2 ISR fills it, the loop drains.
        let (esp_rx_prod, esp_rx_cons) = cx.local.esp_rx_q.split();

        // GPS on USART1, ESP link on USART2, activity LEDs.
        let (mut gps, esp, leds) = cortex_m::interrupt::free(|cs| {
            (
                Gps::new(dp.USART1, gpiob.b6, gpiob.b7, gpiob.b10, &mut rcc, cs),
                EspLink::new(dp.USART2, gpioa.a2, gpioa.a3, &mut rcc, cs, esp_rx_cons),
                Leds::new(gpioa.a9, gpioa.a10, cs),
            )
        });

        // The rail was just powered; the module is at factory 9600, matching
        // gps::BAUD, so push the configured GNSS/power settings now.
        if !gps.configure(&cfg.gps) {
            rprintln!("gps: settings not acknowledged (module absent or rejected)");
        }

        let node = Node::new(radio, &cfg);
        rprintln!(
            "Node {} ready ({})",
            cfg.address,
            match node.role() {
                Role::Leaf => "leaf",
                Role::Repeater => "repeater",
                Role::TxOnly => "transmit only",
                Role::RxOnly => "receive only",
            }
        );

        run::spawn().unwrap();

        (
            Shared {},
            Local {
                node,
                gps,
                sdlog,
                esp,
                leds,
                flash,
                iwdg,
                fw: FwUpdate::new(),
                cfgxfer: CfgTransfer::new(),
                cfg,
                cfg_loaded,
                esp_rx_prod,
            },
        )
    }

    /// USART2 (ESP link) RX interrupt: empty the hardware FIFO into the ring
    /// buffer so no byte is lost while the priority-1 run task is busy. Higher
    /// priority than run so it preempts the loop's GPS/SD/delay work.
    #[task(binds = USART2, priority = 2, local = [esp_rx_prod])]
    fn esp_rx(cx: esp_rx::Context) {
        esplink::drain_rx_isr(cx.local.esp_rx_prod);
    }

    #[task(local = [node, gps, sdlog, esp, leds, flash, iwdg, fw, cfgxfer, cfg, cfg_loaded], priority = 1)]
    async fn run(cx: run::Context) {
        let node = cx.local.node;
        let gps = cx.local.gps;
        let sdlog = cx.local.sdlog;
        let esp = cx.local.esp;
        let leds = cx.local.leds;
        let flash = cx.local.flash;
        let iwdg = cx.local.iwdg;
        let fw = cx.local.fw;
        let cfgxfer = cx.local.cfgxfer;
        let cfg = cx.local.cfg;
        let cfg_loaded = cx.local.cfg_loaded;

        let mut sleeping = false;
        let mut tx_count: u32 = 0;
        let mut rx_count: u32 = 0;
        // Track the GPS fix state so only its transitions are announced.
        let mut had_fix = false;
        // Whether a fix has ever held since boot, which is what separates a
        // fix lost from one never acquired in the no-fix ping below.
        let mut ever_had_fix = false;

        // Position report to the ESP at most once a second (the GPS fix
        // rate); the LoRa beacon runs on its own configured interval.
        let mut next_esp_pos: u32 = 0;
        // Stagger the first beacon so a fleet powered up together does not
        // transmit as one. Folded into eight slots rather than scaled by the
        // address itself, which made node 200 sit silent for three and a half
        // minutes after boot with nothing on the console to explain it. One
        // second apart is already wide against the air time of a beacon, and
        // the jitter added after each transmission keeps them apart from there.
        let mut next_beacon: u32 = platform::millis()
            .wrapping_add((cfg.address as u32 % 8) * 1_000)
            .wrapping_add(2_000);
        let mut next_status: u32 = platform::millis().wrapping_add(3_000);
        // While set, we flagged our radio busy to the ESP; clear at this time.
        let mut busy_clear_at: Option<u32> = None;

        // GPS presence: announce the first NMEA sentence, warn once if the
        // module is still silent after a grace period, and log a periodic
        // aliveness summary to RTT.
        let mut gps_nmea_seen = false;
        let mut gps_checked = false;
        let gps_grace_until = platform::millis().wrapping_add(5_000);
        let mut next_gps_log = platform::millis().wrapping_add(5_000);
        // Settings-push retries, and how many are spent before the loop stops
        // asking. Reset whenever something makes the module unconfigured again.
        const GPS_CFG_TRIES: u8 = 5;
        let mut gps_cfg_tries: u8 = 0;
        let mut next_gps_cfg: u32 = 0;

        status_println!(esp, "wio v{} up, node {}", FIRMWARE_VERSION, cfg.address);
        // Report the config we came up on, so an ESP already connected to a
        // central can serve it without waiting for a CFG_READ round-trip.
        esp.send(msg::CONFIG, &cfg.encode());

        loop {
            let now = platform::millis();
            leds.update(now);

            // ---- ESP link: drain and handle frames -----------------------
            while let Some((cmd_id, _len)) = esp.poll() {
                match cmd_id {
                    cmd::PING => {
                        esp.send_ack(cmd::PING, FIRMWARE_VERSION);
                    }
                    cmd::RADIO_BUSY => {
                        let busy = esp.payload().first() == Some(&1);
                        esp.set_peer_busy(busy, now);
                    }
                    cmd::WIO_SLEEP => {
                        let sleep = esp.payload().first() == Some(&1);
                        if sleep && !sleeping {
                            node.radio_mut().standby();
                            sleeping = true;
                            status_println!(esp, "soft sleep");
                        } else if !sleep && sleeping {
                            node.radio_mut().init(cfg);
                            sleeping = false;
                            status_println!(esp, "woke from soft sleep");
                        }
                        esp.send_ack(cmd::WIO_SLEEP, sleep as u16);
                    }
                    cmd::GPS_SLEEP => {
                        let sleep = esp.payload().first() == Some(&1);
                        if sleep {
                            gps.sleep();
                        } else {
                            // `wake` marks the module unconfigured: backup mode
                            // loses the RAM layer the settings live in. Re-arm
                            // the retry so the loop pushes them again once the
                            // receiver is talking.
                            gps.wake();
                            gps_cfg_tries = 0;
                            next_gps_cfg = now;
                        }
                        esp.send_ack(cmd::GPS_SLEEP, sleep as u16);
                    }
                    cmd::CFG_BEGIN => match cfgxfer.begin(esp.payload()) {
                        CfgEvent::Ack(seq) => esp.send_ack(cmd::CFG_BEGIN, seq),
                        CfgEvent::Error(e) => esp.send_nak(cmd::CFG_BEGIN, e),
                        CfgEvent::Complete | CfgEvent::Done => unreachable!(),
                    },
                    cmd::CFG_DATA => match cfgxfer.data(esp.payload()) {
                        CfgEvent::Ack(seq) => esp.send_ack(cmd::CFG_DATA, seq),
                        CfgEvent::Error(e) => esp.send_nak(cmd::CFG_DATA, e),
                        CfgEvent::Complete | CfgEvent::Done => unreachable!(),
                    },
                    cmd::CFG_END => {
                        // Applying one costs a radio re-init, a GPS settings
                        // push that waits up to 250 ms for an ack, an SD write
                        // and a flash page erase, all before the loop's own
                        // feed at the bottom.
                        watchdog::feed(iwdg);
                        // Bound the borrow of `cfgxfer` to this call so the
                        // arms below can take it mutably again.
                        let event = cfgxfer.end(esp.payload());
                        match event {
                            CfgEvent::Complete => match radiocfg::parse_bytes(cfgxfer.bytes()) {
                                Ok(new_cfg) => {
                                    let regps = new_cfg.gps != cfg.gps;
                                    *cfg = new_cfg;
                                    node.radio_mut().init(cfg);
                                    node.reconfigure(cfg);
                                    if regps && !gps.sleeping {
                                        // Re-arm the retry either way: a push
                                        // that is not acknowledged leaves the
                                        // module on its old settings, and the
                                        // loop is what gets it onto the new.
                                        gps_cfg_tries = 0;
                                        next_gps_cfg = now.wrapping_add(2_000);
                                        if gps.configure(&cfg.gps) {
                                            status_println!(esp, "gps reconfigured");
                                        } else {
                                            status_println!(esp, "gps did not accept settings");
                                        }
                                    }
                                    *cfg_loaded = true;
                                    // Both stores are best effort, but a config
                                    // that reached neither is gone at the next
                                    // power cycle - and that has to reach the
                                    // host, which sees only what goes over the
                                    // link. An RTT-only warning left a push
                                    // looking successful right up until a
                                    // reboot quietly restored the defaults.
                                    let sd_ok = sdlog.write_config(now, cfgxfer.bytes());
                                    let flash_ok = cfgstore::write(flash, cfgxfer.bytes());
                                    let stored = match (sd_ok, flash_ok) {
                                        (true, true) => "saved to SD and flash",
                                        (true, false) => "saved to SD only",
                                        (false, true) => "saved to flash only (no SD)",
                                        (false, false) => "NOT SAVED - lost on reboot",
                                    };
                                    status_println!(
                                        esp,
                                        "config applied, node {}, {}",
                                        cfg.address,
                                        stored
                                    );
                                    // Only now is a repeat of this END
                                    // answerable as the success it was, rather
                                    // than as a transfer that no longer exists.
                                    // A config that verified but would not
                                    // parse is never marked, so retrying that
                                    // one keeps failing - which is the truth.
                                    cfgxfer.mark_applied();
                                    esp.send_ack(cmd::CFG_END, 0);
                                    // Report the new config so a connected
                                    // app's view updates without a fresh read.
                                    esp.send(msg::CONFIG, &cfg.encode());
                                }
                                Err(_) => esp.send_nak(cmd::CFG_END, link::err::BAD_CONFIG),
                            },
                            // The host retried an END whose ack went missing;
                            // the config is already live and stored.
                            CfgEvent::Done => esp.send_ack(cmd::CFG_END, 0),
                            CfgEvent::Ack(seq) => esp.send_ack(cmd::CFG_END, seq),
                            CfgEvent::Error(e) => esp.send_nak(cmd::CFG_END, e),
                        }
                    }
                    cmd::CFG_READ => {
                        // Read-back: the config blob is the reply, not an ack.
                        esp.send(msg::CONFIG, &cfg.encode());
                    }
                    cmd::FW_BEGIN => match fw.begin(esp.payload(), now) {
                        FwEvent::Ack(seq) => {
                            status_println!(esp, "fw update: receiving image");
                            esp.send_ack(cmd::FW_BEGIN, seq);
                        }
                        FwEvent::Error(e) => esp.send_nak(cmd::FW_BEGIN, e),
                        FwEvent::Complete => unreachable!(),
                    },
                    cmd::FW_DATA => {
                        watchdog::feed(iwdg);
                        match fw.data(esp.payload(), flash, now) {
                            FwEvent::Ack(seq) => esp.send_ack(cmd::FW_DATA, seq),
                            FwEvent::Error(e) => esp.send_nak(cmd::FW_DATA, e),
                            FwEvent::Complete => unreachable!(),
                        }
                    }
                    cmd::FW_END => {
                        watchdog::feed(iwdg);
                        match fw.end(flash) {
                            FwEvent::Complete => {
                                esp.send_ack(cmd::FW_END, 0);
                                // Let the ack fully leave the UART before we
                                // reset, or the ESP never sees it and reports
                                // a timeout even though the swap is committed.
                                esp.flush_tx();
                                rprintln!("Rebooting into bootloader for swap");
                                cortex_m::peripheral::SCB::sys_reset();
                            }
                            FwEvent::Error(e) => esp.send_nak(cmd::FW_END, e),
                            FwEvent::Ack(_) => unreachable!(),
                        }
                    }
                    cmd::FW_ABORT => {
                        fw.abort();
                        esp.send_ack(cmd::FW_ABORT, 0);
                    }
                    _ => {}
                }
            }

            // A firmware transfer owns the loop: skip GPS/SD/radio work so
            // the link stays responsive and nothing else erases flash.
            //
            // Which is why it has to be able to end on its own. The ESP aborts
            // a stalled transfer after 5 s, but an ESP that resets mid-upload
            // never sends that abort, and the node would then sit off the air -
            // no beacon, no GPS, no logging - until someone power-cycled it.
            if fw.expire(now) {
                status_println!(esp, "fw update: abandoned, resuming normal operation");
            }
            if fw.is_active() {
                watchdog::feed(iwdg);
                Mono::delay(1_u32.millis()).await;
                continue;
            }

            if sleeping {
                // Soft sleep: only the ESP link stays alive (for WAKE).
                watchdog::feed(iwdg);
                Mono::delay(50_u32.millis()).await;
                continue;
            }

            // ---- GPS ------------------------------------------------------
            gps.poll();
            let fix = gps.has_fix();
            if fix != had_fix {
                had_fix = fix;
                if fix {
                    ever_had_fix = true;
                    status_println!(esp, "gps fix acquired ({} sats)", gps.packet().sats);
                } else {
                    status_println!(esp, "gps fix lost");
                }
            }
            // Presence: first sentence, then a one-shot warning if the
            // module never spoke, then a periodic RTT aliveness line.
            if !gps_nmea_seen && gps.present() {
                gps_nmea_seen = true;
                status_println!(esp, "gps: NMEA up ({} bytes)", gps.rx_bytes());
            }
            // Settings retry. Two things leave the module running its own
            // defaults while the firmware reports the ones it asked for: a
            // boot-time push that landed before the receiver had finished
            // starting, and a wake from backup mode, which cuts power to the
            // receiver core and takes the whole RAM configuration layer with it
            // - including the four NMEA sentences this firmware silences to fit
            // 9600 baud. Driving the retry off `configured` rather than off the
            // first sentence covers both, since `wake` clears it.
            //
            // A sentence is the earliest proof the module is listening, so that
            // gates the attempt; the tries are capped so a module that keeps
            // refusing does not talk over the console forever.
            if !gps.configured
                && !gps.sleeping
                && gps.present()
                && gps_cfg_tries < GPS_CFG_TRIES
                && due(now, next_gps_cfg)
            {
                next_gps_cfg = now.wrapping_add(2_000);
                gps_cfg_tries += 1;
                if gps.configure(&cfg.gps) {
                    status_println!(esp, "gps: settings applied");
                } else if gps_cfg_tries == GPS_CFG_TRIES {
                    status_println!(esp, "gps: settings still not accepted, giving up");
                }
            }
            if !gps_checked && due(now, gps_grace_until) {
                gps_checked = true;
                if !gps.present() {
                    if gps.rx_bytes() == 0 {
                        status_println!(esp, "gps: silent on USART1 (power/wiring?)");
                    } else {
                        status_println!(esp, "gps: {} bytes but no NMEA (baud?)", gps.rx_bytes());
                    }
                }
            }
            if due(now, next_gps_log) {
                next_gps_log = now.wrapping_add(5_000);
                rprintln!(
                    "gps: bytes={} nmea={} fix={} sats={}",
                    gps.rx_bytes(),
                    gps.rx_sentences(),
                    gps.has_fix() as u8,
                    gps.packet().sats
                );
            }
            if gps.take_updated() && due(now, next_esp_pos) {
                next_esp_pos = now.wrapping_add(1_000);
                let packet = gps.packet();
                let mut buf = [0u8; 3 + 20];
                buf[0] = 0; // src: local
                buf[1..3].copy_from_slice(&0i16.to_le_bytes());
                buf[3..].copy_from_slice(&packet.encode());
                esp.send(msg::POSITION, &buf);
                if gps.has_fix() {
                    sdlog.log_position(now, 0, 0, &packet);
                }
            }

            // ---- LoRa beacon: a position, or a ping without a fix ----------
            // Gated on the role here rather than inside the transmit, so a
            // receive-only node never claims the air with RADIO_BUSY for a
            // broadcast it was never going to send.
            //
            // One transmission per interval either way. A fix goes out as a
            // position; without one the slot carries a ping, so a node
            // searching for the sky is a node a receiver can hear rather
            // than one indistinguishable from out of range or dead. A ping
            // is the smaller of the two on air, so this cannot push a node
            // past the duty cycle its beacon already fits in.
            if cfg.role.transmits() && cfg.beacon_interval_s != 0 && due(now, next_beacon) {
                if esp.peer_busy(now) {
                    // ESP radio has the air: check again shortly.
                    next_beacon = now.wrapping_add(500);
                } else {
                    // Warn the ESP off the air for as long as the blocking
                    // transmit can hold the radio.
                    esp.send(msg::RADIO_BUSY, &[1]);
                    busy_clear_at = Some(now.wrapping_add(cfg.tx_poll_timeout_ms()));
                    let sent = if gps.has_fix() {
                        let (pos, n) = lora::encode_position(&gps.packet(), cfg.beacon_fields);
                        node.broadcast(&pos[..n])
                    } else {
                        node.broadcast(
                            &lora::Ping {
                                uptime_s: (now / 1_000).min(u16::MAX as u32) as u16,
                                gps_present: gps.present(),
                                had_fix: ever_had_fix,
                            }
                            .encode(),
                        )
                    };
                    match sent {
                        Ok(()) => tx_count += 1,
                        Err(e) => debug_println!("Beacon TX failed: {:?}", e),
                    }
                    let jitter = platform::random(0, 2_000) as u32;
                    next_beacon = now
                        .wrapping_add(cfg.beacon_interval_s as u32 * 1_000)
                        .wrapping_add(jitter);
                }
            }
            if let Some(t) = busy_clear_at
                && due(now, t) {
                    esp.send(msg::RADIO_BUSY, &[0]);
                    busy_clear_at = None;
                }

            // ---- LoRa receive -----------------------------------------------
            if let Some(rx) = node.poll(now) {
                rx_count += 1;
                if let Some(p) = lora::decode_position(rx.payload) {
                    debug_println!("Position from node {} rssi={}", rx.src, rx.rssi);
                    let mut buf = [0u8; 3 + 20];
                    buf[0] = rx.src;
                    buf[1..3].copy_from_slice(&rx.rssi.to_le_bytes());
                    buf[3..].copy_from_slice(&p.encode());
                    esp.send(msg::POSITION, &buf);
                    sdlog.log_position(now, rx.src, rx.rssi, &p);
                } else if let Some(ping) = lora::Ping::decode(rx.payload) {
                    // A node on the air with no fix to report. Nothing to log
                    // to SD - there is no position - but the ESP gets it as
                    // data as well as prose, so an app can show the node as
                    // alive-without-a-fix rather than have to parse the line
                    // below. The RSSI rides along either way, which is what
                    // makes a ping usable as a range check.
                    let mut buf = [0u8; link::PING_LEN];
                    buf[0] = rx.src;
                    buf[1..3].copy_from_slice(&rx.rssi.to_le_bytes());
                    buf[3] = ping.flags();
                    buf[4..6].copy_from_slice(&ping.uptime_s.to_le_bytes());
                    esp.send(msg::PING, &buf);
                    status_println!(
                        esp,
                        "node {} ping: rssi {}, up {}s, gps {}{}",
                        rx.src,
                        rx.rssi,
                        ping.uptime_s,
                        if ping.gps_present { "ok" } else { "silent" },
                        if ping.had_fix { ", fix lost" } else { "" }
                    );
                } else {
                    // Forward other payloads verbatim.
                    let mut buf = [0u8; 3 + lora::PAYLOAD_MAX];
                    let n = rx.payload.len().min(lora::PAYLOAD_MAX);
                    buf[0] = rx.src;
                    buf[1..3].copy_from_slice(&rx.rssi.to_le_bytes());
                    buf[3..3 + n].copy_from_slice(&rx.payload[..n]);
                    esp.send(msg::LORA_RX, &buf[..3 + n]);
                }
            }

            // ---- Repeat forwarding ------------------------------------------
            // Only a node configured as a repeater ever has one of these
            // queued; a leaf-only network never enters this branch.
            if node.repeat_due(now) && !esp.peer_busy(now) {
                esp.send(msg::RADIO_BUSY, &[1]);
                busy_clear_at = Some(now.wrapping_add(cfg.tx_poll_timeout_ms()));
                if node.send_due_repeat(now) {
                    tx_count += 1;
                }
            }

            // ---- Periodic status to the ESP ---------------------------------
            if due(now, next_status) {
                next_status = now.wrapping_add(5_000);
                let secs_since_rx = match node.last_rx_ms() {
                    Some(t) => {
                        let s = platform::millis().wrapping_sub(t) / 1000;
                        s.min(0xFFFE) as u16
                    }
                    None => 0xFFFF,
                };
                let mut flags = 0u8;
                if sdlog.ready() {
                    flags |= link::TELEM_FLAG_SD_OK;
                }
                if gps.has_fix() {
                    flags |= link::TELEM_FLAG_GPS_FIX;
                }
                if *cfg_loaded {
                    flags |= link::TELEM_FLAG_CFG_LOADED;
                }
                // The ESP has no copy of the config, so its console verbosity
                // rides along here and is refreshed with every heartbeat.
                if cfg.verbose {
                    flags |= link::TELEM_FLAG_VERBOSE;
                }
                let telem = Telemetry {
                    last_rssi: node.last_rssi(),
                    last_snr_cb: node.radio().last_snr_cb(),
                    secs_since_rx,
                    rx_count,
                    tx_count,
                    flags,
                    sats: gps.packet().sats,
                };
                esp.send(msg::STATUS, &telem.encode());

                // Verbose only: break down what the radio heard but did not
                // deliver, so "a couple of random RXs" can be read as mostly
                // CRC failures (weak signal / parameter mismatch), duplicates,
                // or this node's own echoes rather than a mystery. Counts are
                // cumulative since boot.
                if cfg.verbose {
                    let d = node.rx_drops();
                    let (mode, op_err) = node.radio_mut().health();
                    status_println!(
                        esp,
                        "radio {} err {:04x}; rx {} ok; dropped crc {} dup {} echo {} malformed {} oversize {}",
                        mode,
                        op_err,
                        rx_count,
                        node.radio().rx_crc_errors(),
                        d.duplicate,
                        d.own_echo,
                        d.malformed,
                        node.radio().rx_oversize(),
                    );
                }
            }

            // ---- SD housekeeping --------------------------------------------
            sdlog.poll(now);

            watchdog::feed(iwdg);
            Mono::delay(1_u32.millis()).await;
        }
    }
}
