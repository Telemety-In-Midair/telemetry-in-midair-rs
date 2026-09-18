//! Everything that is not BLE: the radio, the GPS and the panel, owned by
//! one task on the second core.
//!
//! Owning them in one task is what removes any link protocol between them.
//! The BLE session never touches the hardware; it reads the snapshot this
//! publishes and asks through [`state::request`]. What the board has up,
//! and what each request changes, is [`midair_proto::posture`]; when the
//! next beacon goes out is [`midair_proto::beacon`]; both are host-tested
//! and walked exhaustively by the state space tests, and this task carries
//! out the effects they name.

use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::{Level, Output};
use esp_hal::time::Rate;
use esp_println::println;
use gps_proto::packet;
use midair_proto::beacon::{Planner, Step};
use midair_proto::ble::{self, Mode};
use midair_proto::bulk;
use midair_proto::posture::{Effect, Effects, Posture, Radio, Request};
use midair_proto::session::Stored;
use midair_proto::radiocfg::{self, RadioConfig};
use midair_proto::evlog::Kind;
use midair_proto::roster::Report;
use midair_proto::sentry::{self, Step as WakeStep, Waker};
use midair_proto::supervise::{Phase, Task};
use midair_proto::{link, lora};

use crate::gps::Gps;
use crate::gpsctl::GpsWatch;
use crate::node::Node;
use crate::radio::Sx1262Driver;
use crate::{crumb, event, flash, oled, settings, state, watchdog, xfer};

/// Status LEDs, cathodes on GPIO43 and GPIO14. Active low: the anodes sit
/// on +3V3 through R21/R20, so driving the pin low is what lights them.
pub const LED_ON: Level = Level::Low;
pub const LED_OFF: Level = Level::High;

/// How long an activity LED stays lit for one packet.
const BLINK_MS: u64 = 20;

/// The longest gap between two passes of the hardware loop after which a
/// GPS time mark parsed on the second is not used to set the hop clock,
/// ms. The sentence arrived somewhere in that gap and was parsed at its
/// end; with a beacon or a config apply in between, that is hundreds of
/// milliseconds of error handed to every node that follows this one.
const LATE_PASS_MS: u64 = 40;

/// Between panel refreshes, ms. The fields underneath change about once a
/// second, so this is fast enough that a fix or a packet lands promptly
/// and slow enough that the 11 ms frame write is a couple of percent of
/// the loop.
const PANEL_MS: u64 = 500;

/// Between status lines, ms.
const STATUS_MS: u64 = 10_000;

/// An activity LED that is lit from the loop rather than by blocking in it.
///
/// The obvious light-it, `Timer::after(20ms)`, dark-it costs the receive
/// path twenty milliseconds it should be spending polling the radio, which
/// at the default settings is most of a packet.
struct Blinker {
    pin: Output<'static>,
    off_at_ms: u64,
    lit: bool,
}

impl Blinker {
    fn new(pin: Output<'static>) -> Self {
        Self {
            pin,
            off_at_ms: 0,
            lit: false,
        }
    }

    fn pulse(&mut self, now_ms: u64) {
        self.pin.set_level(LED_ON);
        self.off_at_ms = now_ms + BLINK_MS;
        self.lit = true;
    }

    fn update(&mut self, now_ms: u64) {
        if self.lit && now_ms >= self.off_at_ms {
            self.pin.set_level(LED_OFF);
            self.lit = false;
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
    pub oled: Option<oled::Oled>,
    pub compass: Option<crate::compass::Compass>,
}

/// Try both SDA/SCL orders on J5 and return whichever finds a panel.
///
/// SDA on GPIO10 and SCL on GPIO11 goes first; the reverse is the fallback.
/// Which of the two is SDA is not a board fact - the schematic names those
/// two nets `GPIO10` and `GPIO11` and nothing else - so both orders are
/// tried rather than one being picked and a reversed cable looking like a
/// dead panel.
///
/// The I2C peripheral and the two pins are consumed by each attempt, so the
/// retry steals the singletons back. That is sound here and only here: this
/// runs once, before anything else has been handed either pin, and the
/// `I2c` from the failed attempt is dropped before the next is built.
pub async fn probe_j5(i2c0: esp_hal::peripherals::I2C0<'static>) -> Option<J5> {
    use esp_hal::gpio::interconnect::OutputSignal;
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
        let oled = oled::Oled::probe(&mut i2c).await;
        let compass = crate::compass::Compass::probe(&mut i2c).await;
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

/// The hardware the loop owns, and what it keeps between passes.
pub struct Hardware {
    node: Node<'static>,
    gps: Gps<'static>,
    j5: Option<J5>,
    rx_led: Blinker,
    tx_led: Blinker,
    cfg: RadioConfig,
    /// Whether a stored config was adopted at all, which is what the
    /// `CFG_LOADED` telemetry flag reports.
    cfg_loaded: bool,
    /// Whether the next config load is the cold boot's, which is the one
    /// that adopts the record's `[power]` section: a deep-sleep wake keeps
    /// the live settings, and so does a promotion.
    cold: bool,
    watch: GpsWatch,
    posture: Posture,
    planner: Planner,
    rx_count: u32,
    tx_count: u32,
    next_panel_ms: u64,
    next_status_ms: u64,
    idle_mark: u64,
    idle_at_ms: u64,
    /// When the previous pass began, so a pass that comes late - after a
    /// transmit, a flush, a config apply - knows not to trust the arrival
    /// times it is about to assign.
    prev_pass_ms: u64,
    /// Nodes already reported for sharing this node's turn, one bit each,
    /// so the console says it once per node rather than once per frame.
    turn_warned: [u8; 32],
    /// A call to a sleeping node in progress: the burst of wake frames
    /// and the listens between them.
    waker: Option<Waker>,
    /// Tells one burst from the next in the frames it sends.
    wake_nonce: u8,
}

impl Hardware {
    /// Take the peripherals. Synchronous, and the struct literal is the
    /// return value, so the state is built once, in place: an `async fn`
    /// that owned the peripherals and returned `Self` kept them twice over
    /// in its future - the arguments and the value under construction -
    /// and the task's future, which is built on the second core's stack
    /// before it is moved into the arena, overflowed that stack.
    ///
    /// The stored config is read on the first effect that asks for it,
    /// which a wake check never issues: it exists to ask whether anyone
    /// wants the board back, and that question needs BLE and nothing
    /// else. `cold` is whether this is a cold boot rather than a
    /// deep-sleep wake; `posture` is what [`Posture::at_boot`] said the boot
    /// raises, whose effects [`boot`](Self::boot) then carries out.
    pub fn new(
        lora: Sx1262Driver<'static>,
        gps: Gps<'static>,
        j5: Option<J5>,
        d5: Output<'static>,
        d2: Output<'static>,
        posture: Posture,
        cold: bool,
    ) -> Self {
        let cfg = RadioConfig::default();
        let now_ms = Instant::now().as_millis();
        Self {
            node: Node::new(lora, &cfg),
            gps,
            j5,
            rx_led: Blinker::new(d5),
            tx_led: Blinker::new(d2),
            cfg,
            cfg_loaded: false,
            cold,
            watch: GpsWatch::new(now_ms),
            posture,
            // Stagger the first beacon so a fleet powered up together does
            // not transmit as one. Folded into eight slots rather than
            // scaled by the address itself, which made node 200 sit silent
            // for three and a half minutes after boot with nothing on the
            // console to explain it. The address is the config's, which is
            // not read yet, so the stagger is re-seeded when it is.
            planner: Planner::new(now_ms + 2_000),
            rx_count: 0,
            tx_count: 0,
            next_panel_ms: 0,
            next_status_ms: now_ms + 5_000,
            idle_mark: crate::idle::entries(),
            idle_at_ms: now_ms,
            prev_pass_ms: now_ms,
            turn_warned: [0; 32],
            waker: None,
            wake_nonce: 0,
        }
    }

    /// Raise what the boot raises: the effects [`Posture::at_boot`] named,
    /// which is the whole difference between the flavors - a wake check
    /// nothing, idle the card, tracking and listening everything, and the
    /// two override flags on top of that, which are the settings that
    /// survive a deep sleep re-applied because the wake that restored them
    /// is a fresh boot to everything else.
    // The probe builds below never return, so everything after them is
    // unreachable on purpose. Scoped to those features so the warning still
    // means something in the build that ships.
    #[cfg_attr(
        any(
            feature = "iso-sentry-probe",
            feature = "iso-sentry-source",
            feature = "iso-sentry-carrier",
            feature = "iso-sentry-mirror"
        ),
        allow(unreachable_code)
    )]
    pub async fn boot(&mut self, boot: Mode, boot_fx: Effects, stored: &Stored) {
        let now_ms = Instant::now().as_millis();
        for e in boot_fx {
            self.effect(e, now_ms).await;
        }
        // The second half of the wake timing. The console's `wake:` line
        // says how long the chip took to come back; this says how long after
        // that the radio was able to hear anything, and the two together are
        // what a transmission trying to reach a board that is waking up has
        // to outlast. Only when the boot actually raised the radio - a wake
        // check leaves it parked and has nothing to report.
        if self.posture.radio_up() {
            status_println!("boot: radio up {} ms into this boot", Instant::now().as_millis());
        }
        // A boot a wake frame caused answers the caller, so the waker's
        // burst ends at the first frame rather than at its last try. One
        // ping on the network's word, from a radio brought up for it if
        // the mode this boot came up in keeps the radio down, and put back
        // where the posture wants it afterwards.
        if let Some((caller, _)) = state::take_lora_wake() {
            let raised = !self.posture.radio_up();
            if raised {
                self.effect(Effect::RadioInit, now_ms).await;
            }
            state::set_radio_busy(true);
            watchdog::beat(Task::Loop, Phase::TxSend);
            let answered = self
                .node
                .broadcast(
                    &lora::Ping {
                        uptime_s: (now_ms / 1_000).min(u16::MAX as u64) as u16,
                        gps_present: self.gps.present(),
                        had_fix: false,
                    }
                    .encode(),
                    0,
                )
                .await;
            state::set_radio_busy(false);
            // The counters go out here as well as on the boot line: this
            // is the first line of a LoRa wake a host attached to the USB
            // port is likely to see, and a frame rejected for another node
            // costs a boot too short for the port to come up at all - so
            // the rejects are only ever readable from a wake like this one.
            let (wakes, rejects) = settings::lora_wakes();
            match answered {
                Ok(()) => {
                    self.tx_count = self.tx_count.saturating_add(1);
                    status_println!(
                        "wake: answered node {} (lora wakes {}, rejected {} since cold boot)",
                        caller, wakes, rejects
                    );
                }
                Err(e) => status_println!("wake: could not answer node {} ({:?})", caller, e),
            }
            if raised {
                match self.posture.radio {
                    Radio::Standby => self.node.radio_mut().standby(),
                    _ => self.node.radio_mut().sleep(),
                }
            }
        }

        // The isolation build asks unconditionally, because the point is
        // to measure the receiver rather than to honor a setting. V_BCKP
        // is not fed on this board, so the wake path is unproven - a
        // power cycle is the way back.
        #[cfg(feature = "iso-gps-backup")]
        {
            // Timed rather than open-ended, so the test recovers itself.
            // Read the meter across the gap: the drop is the receiver's
            // own draw, and the sentence counter climbing again on the far
            // side is the proof that backup mode is usable on a board
            // whose V_BCKP is not fed. If it never comes back, that is the
            // answer too - and it costs a power cycle rather than a board
            // that cannot be recovered without one.
            const ISO_GPS_BACKUP_MS: u32 = 20_000;
            self.gps.sleep_for(ISO_GPS_BACKUP_MS);
            status_println!(
                "iso-gps-backup: GPS in backup for {} s - watch the meter, then the nmea count",
                ISO_GPS_BACKUP_MS / 1000
            );
        }

        // Both halves take the board over rather than returning to the
        // loop: the probe needs the radio's whole attention and the source
        // needs the PA, and neither has anything to do with tracking.
        #[cfg(feature = "iso-sentry-probe")]
        crate::sentry::probe(self.node.radio_mut()).await;
        #[cfg(feature = "iso-sentry-source")]
        crate::sentry::source(self.node.radio_mut()).await;
        #[cfg(feature = "iso-sentry-carrier")]
        crate::sentry::carrier(self.node.radio_mut()).await;
        #[cfg(feature = "iso-sentry-mirror")]
        crate::sentry::mirror(self.node.radio_mut()).await;

        match boot {
            Mode::Tracking => status_println!(
                "tracking: node {} ({}), {} Hz SF{} BW{}",
                self.cfg.address,
                self.cfg.role.as_str(),
                self.cfg.frequency_hz,
                self.cfg.spreading_factor,
                self.cfg.bandwidth_khz
            ),
            Mode::Idle => match (stored.idle_timeout(), stored.sleep_interval_s) {
                // No timeout, or no cadence to sleep on, so there is
                // nowhere for idle to send the board: it stays reachable
                // until something says otherwise. The default, and the
                // bench case.
                (0, _) => status_println!("idle: gps in backup, radio asleep, stays idle"),
                (_, 0) => status_println!("idle: gps in backup, radio asleep, no sleep cadence set"),
                (timeout, _) => status_println!(
                    "idle: gps in backup, radio asleep, {} s before it stores itself",
                    timeout
                ),
            },
            Mode::Stored => status_println!(
                "wake check: nothing raised, {} s window then back down",
                stored.adv_window()
            ),
            Mode::Listening => status_println!(
                "listening: node {} receiving, nothing transmitted, ble up throughout",
                self.cfg.address
            ),
        }

        // Something on the panel before the first telemetry, so a board
        // that fails during init does not look like a board with a dead
        // display. Not on a wake check: the park path blanked the panel
        // and the panel keeps its own state across the sleep, so lighting
        // it here would put the display's current back into every wake.
        if boot != Mode::Stored
            && let Some(j) = self.j5.as_mut()
            && let Some(o) = j.oled.as_mut()
        {
            oled::render(o, None, self.cfg.address);
            o.flush(&mut j.i2c).await;
        }
    }

    /// One effect the posture named, carried out.
    async fn effect(&mut self, e: Effect, now_ms: u64) {
        match e {
            Effect::LoadConfig => {
                self.cfg_loaded = self.adopt_stored_config().await;
                self.cold = false;
                self.node.reconfigure(&self.cfg);
                self.planner = Planner::new(self.first_beacon_ms(now_ms));
                self.posture.set_sentry(self.cfg.wake_enabled);
            }
            Effect::GpsUp => {
                watchdog::beat(Task::Loop, Phase::GpsCtl);
                self.watch.configure(&mut self.gps, &self.cfg.gps).await
            }
            Effect::GpsPark => {
                watchdog::beat(Task::Loop, Phase::GpsCtl);
                self.gps.park().await;
                self.watch.parked();
                status_println!("gps: backup mode");
            }
            // The park that caused a sleep left the receiver in an
            // open-ended backup, and a reset does not change that - so the
            // driver is told what the module is actually doing rather than
            // being left with its power-on assumption. Without it a
            // promotion straight to tracking would find `wake` a no-op and
            // the receiver would stay in backup with nothing to notice it.
            Effect::GpsAssumeParked => {
                self.gps.sleeping = true;
                self.watch.parked();
            }
            Effect::RadioInit => {
                watchdog::beat(Task::Loop, Phase::RadioInit);
                self.node.radio_mut().init(&self.cfg).await;
                if !self.node.radio_mut().print_diagnostics() {
                    event!(Kind::Radio, "radio did not answer - check the pin map in main");
                }
            }
            Effect::RadioStandby => {
                self.node.radio_mut().standby();
                status_println!("radio: standby");
            }
            // Cold sleep rather than standby: nothing is going to use the
            // radio, and `init` runs again whenever something does.
            Effect::RadioSleep => {
                settings::clear_sentry();
                self.node.radio_mut().sleep()
            }
            // Leave the radio listening through the sleep. The config is
            // checked here and not only in the posture, because a wake
            // check's park decides on the sentry before the config load in
            // the same pass can say otherwise; a config that says no, or
            // asks for periods no preamble fits, gets a cold sleep and a
            // line saying why. After the arm nothing here touches the
            // radio again: the next effect is the panel, then the signal,
            // then the chip goes down.
            Effect::RadioSentry => {
                let payload = lora::HEADER_LEN + lora::WAKE_MSG_LEN;
                match sentry::plan(&self.cfg, payload) {
                    Ok(s) => {
                        self.node.radio_mut().arm_sentry(&s);
                        settings::note_sentry(s.rx_us / 1_000, s.sleep_us / 1_000, self.cfg.address);
                        status_println!(
                            "radio: sentry armed, {} ms every {} ms on {} Hz ({} permille, wake preamble {} symbols)",
                            s.rx_us / 1_000,
                            s.sleep_us / 1_000,
                            self.cfg.wake_carrier_hz(),
                            s.duty_permille(),
                            s.preamble_symbols().unwrap_or(0)
                        );
                    }
                    Err(r) => {
                        settings::clear_sentry();
                        self.node.radio_mut().sleep();
                        if r != sentry::Refusal::Disabled {
                            status_println!("radio: sentry refused ({}), sleeping cold", r.as_str());
                        }
                    }
                }
            }
            // The panel sits on the always-on +3V3, so without this it
            // holds its last frame - and its current - for the whole
            // sleep.
            Effect::PanelBlank => {
                if let Some(j) = self.j5.as_mut()
                    && let Some(o) = j.oled.as_mut()
                {
                    o.blank(&mut j.i2c).await;
                }
            }
            Effect::ApplyConfig => {
                watchdog::beat(Task::Loop, Phase::ConfigApply);
                if self.apply_radio_config(now_ms).await {
                    self.cfg_loaded = true;
                    self.watch.rearm(now_ms + 2_000);
                    self.posture.set_sentry(self.cfg.wake_enabled);
                }
            }
            // Did the last park hold? Free to ask here and nowhere else:
            // nothing polls the receiver while it is parked, so whatever
            // is in the UART FIFO came from a receiver that was awake -
            // the ~10 mA failure that costs a whole sleep interval, and
            // is otherwise invisible because the deep sleep reset this
            // driver's idea of the module's state along with everything
            // else.
            Effect::CheckParkHeld => {
                let before = self.gps.rx_bytes();
                self.gps.poll();
                if self.gps.rx_bytes() != before {
                    event!(Kind::Gps, "gps: talking at park - the last park did not hold");
                }
            }
            Effect::SleepReady => state::SLEEP_READY.signal(()),
            Effect::Reboot => {
                // Long enough for the ack that asked for this to leave the
                // USB FIFO or the BLE connection.
                Timer::after(Duration::from_millis(500)).await;
                crumb::mark_reset(crumb::Reason::Ota);
                esp_hal::system::software_reset();
            }
        }
    }

    /// No beacon before this: the boot stagger, from the config's address.
    fn first_beacon_ms(&self, now_ms: u64) -> u64 {
        now_ms + (u64::from(self.cfg.address) % 8) * 1_000 + 2_000
    }

    /// One pass of the loop. Returns how long to wait before the next:
    /// 100 Hz while there is a radio to poll or a UART to drain, a fifth
    /// of that when there is neither, since nothing else here moves faster
    /// than the panel's 2 Hz.
    pub async fn pass(&mut self) -> Duration {
        let now_ms = Instant::now().as_millis();
        let late_pass = now_ms.saturating_sub(self.prev_pass_ms) > LATE_PASS_MS;
        self.prev_pass_ms = now_ms;
        self.rx_led.update(now_ms);
        self.tx_led.update(now_ms);

        // The heartbeat: this pass is happening, and what it is doing is
        // set as it goes, so a stall names the stage it stopped in.
        watchdog::beat(Task::Loop, Phase::Requests);
        self.requests(now_ms).await;

        // A bulk transfer whose host walked away must not hold the board
        // off the air; the USB task bounds its own read, and this covers a
        // transfer that arrived over BLE. Gated on the flag so the common
        // case does not queue behind whoever holds the transfer lock.
        if state::transfer_active() {
            xfer::expire(now_ms).await;
        }

        // The receiver, while it is awake; the beacon and the radio, while
        // the radio is up. Each gated on its own state rather than both on
        // the radio's: a receiver woken while the radio sat in standby used
        // to acquire with nobody draining its sentences.
        //
        // What follows still runs. A board that is idle rather than asleep
        // is one somebody may be looking at, so telemetry stays fresh, the
        // panel keeps its frame and the status line keeps printing - and
        // none of that costs anything on a wake check, where the panel is
        // dark.
        if self.posture.gps_awake() {
            watchdog::beat(Task::Loop, Phase::Gps);
            self.gps(now_ms, late_pass).await;
        }
        if let Some((target, idle)) = state::take_wake_request() {
            self.start_wake(target, idle);
        }
        if self.posture.radio_up() {
            watchdog::beat(Task::Loop, Phase::Beacon);
            self.beacon(now_ms).await;
            watchdog::beat(Task::Loop, Phase::TxSend);
            self.wake(now_ms).await;
            watchdog::beat(Task::Loop, Phase::Receive);
            self.receive(now_ms);
            self.repeat(now_ms).await;
        }
        self.telemetry(now_ms);
        watchdog::beat(Task::Loop, Phase::Panel);
        self.panel(now_ms).await;
        watchdog::beat(Task::Loop, Phase::Status);
        self.status(now_ms);

        Duration::from_millis(if self.posture.busy() { 10 } else { 50 })
    }

    /// Requests from the BLE session and the host tools.
    async fn requests(&mut self, now_ms: u64) {
        while let Some(r) = state::take_request() {
            let fx = self.posture.on(r, &settings::get());
            for e in fx {
                self.effect(e, now_ms).await;
            }
            // Said from what the posture is, not from what was asked: a
            // mode commanded over an override flag lands on the flag, and a
            // board already parked for sleep ignores the request.
            match r {
                Request::Mode(m) if !self.posture.parked => status_println!(
                    "{}: node {} ({}), gps {}, radio {}",
                    m.as_str(),
                    self.cfg.address,
                    self.cfg.role.as_str(),
                    if self.posture.gps_awake() { "up" } else { "in backup" },
                    match self.posture.radio {
                        Radio::Up if m.transmits() => "up",
                        Radio::Up => "receiving, nothing transmitted",
                        Radio::Standby => "standby",
                        Radio::Asleep => "asleep",
                        Radio::Sentry => "listening as a sentry",
                    }
                ),
                Request::RadioStandby(false) if fx.contains(Effect::RadioInit) => {
                    status_println!("radio: back from standby")
                }
                _ => {}
            }
        }
    }

    /// The receiver: sentences, settings, the hop clock, the position.
    async fn gps(&mut self, now_ms: u64, late_pass: bool) {
        let seen = self.watch.pass(&mut self.gps, &self.cfg.gps, now_ms, late_pass).await;
        if let Some((tod_ms, at_ms)) = seen.time_mark
            && self.node.radio_mut().hop_discipline_gps(tod_ms, at_ms)
        {
            status_println!("hop: clock on gps time");
        }
        if let Some(p) = seen.position {
            state::set_position(p);
        }
    }

    /// The beacon: a position on the beacon interval, or a ping on the
    /// ping interval without a fix, so a node searching for the sky is a
    /// node a receiver can hear rather than one indistinguishable from out
    /// of range or dead. Which interval applies is decided by what the
    /// next transmission would carry, so a fix gained is reported as soon
    /// as the beacon interval allows, not when the slower ping would have.
    /// A ping is the smaller of the two on air, so this cannot push a node
    /// past the budget its beacon already fits in.
    async fn beacon(&mut self, now_ms: u64) {
        // Not while calling a node. The sentry being called locks on any
        // LoRa symbol that lands in its window, and this node's own beacon
        // just before its wake frame held that sentry through the frame -
        // measured as a call answered on its second try, not its first.
        if self.waker.is_some() {
            return;
        }
        let has_fix = self.gps.has_fix();
        let interval_ms = if self.cfg.beacon_interval_s == 0 {
            0
        } else if has_fix {
            u32::from(self.cfg.beacon_interval_s) * 1_000
        } else {
            u32::from(self.cfg.ping_interval_s) * 1_000
        };
        // The mode gate is what makes listening a mode rather than a role,
        // and the sleep gate is not politeness: a transmit awaits for the
        // frame's time on air, which at the slowest settings the config
        // accepts is nearly ten seconds, and a sleep that arrives just
        // after one starts waits all of it out.
        let allowed = self.posture.may_transmit(
            self.cfg.role,
            state::transfer_active(),
            state::park_pending(),
        );
        let frame_len = self.cfg.frame_overhead()
            + if has_fix {
                lora::position_msg_len(self.cfg.beacon_fields)
            } else {
                lora::PING_MSG_LEN
            };
        let airtime_ms = self.cfg.time_on_air_us(frame_len).div_ceil(1000);
        // The last gate is a frame arriving: keying up over it would lose
        // both, and the poll will have delivered it by the next pass.
        let rx_busy = self.node.radio().rx_in_progress(now_ms);
        let address = self.cfg.address;
        let step = {
            let (clock, plan) = self.node.radio_mut().schedule();
            self.planner
                .pass(now_ms, allowed, rx_busy, clock, plan, address, interval_ms, airtime_ms)
        };
        if step != Step::Send {
            return;
        }
        // The SX1262 does not reset with the MCU. If it browned out and
        // restarted on its own it is back at its power-up defaults -
        // antenna switch unpowered, DIO2 not switching - and keying up into
        // that ramps +22 dBm into an isolated port. Nothing about it looks
        // wrong from the counters, so this is the only place it can be
        // caught.
        if self.node.radio_mut().looks_reset() {
            event!(Kind::Radio, "radio restarted underneath us, re-initializing");
            self.node.radio_mut().init(&self.cfg).await;
        }
        state::set_radio_busy(true);
        watchdog::beat(Task::Loop, Phase::TxSend);
        self.tx_led.pulse(now_ms);
        let sent = if has_fix {
            let (pos, n) = lora::encode_position(&self.gps.packet(), self.cfg.beacon_fields);
            self.node.broadcast(&pos[..n], interval_ms).await
        } else {
            self.node
                .broadcast(
                    &lora::Ping {
                        uptime_s: (now_ms / 1_000).min(u16::MAX as u64) as u16,
                        gps_present: self.gps.present(),
                        had_fix: self.watch.ever_had_fix(),
                    }
                    .encode(),
                    interval_ms,
                )
                .await
        };
        state::set_radio_busy(false);
        match sent {
            Ok(()) => {
                self.tx_count = self.tx_count.saturating_add(1);
                vprintln!(
                    "beacon {} ({} ms on air)",
                    if has_fix { "position" } else { "ping" },
                    self.cfg.beacon_airtime_us() / 1000
                );
            }
            Err(e) => vprintln!("beacon TX failed: {:?}", e),
        }
        // As the radio timed it, so the interval runs from the slot it
        // started in. Recorded for a failed transmit too: a radio that
        // will not key up should be retried on the interval, not on every
        // pass.
        if let Some(span) = self.node.radio().last_tx_span() {
            self.planner.sent(span);
        }
    }

    /// What the radio heard.
    fn receive(&mut self, now_ms: u64) {
        if let Some((src, stratum)) = self.node.take_sync_note() {
            status_println!("hop: clock from node {} (stratum {})", src, stratum);
        }
        let mut heard_from = None;
        if let Some(rx) = self.node.poll(now_ms) {
            self.rx_count = self.rx_count.saturating_add(1);
            self.rx_led.pulse(now_ms);
            heard_from = Some(rx.src);
            // Anything from the node being called is its answer: the
            // woken board pings, but a beacon would do as well.
            if let Some(w) = self.waker.as_mut() {
                w.heard(rx.src);
            }
            if let Some(p) = lora::decode_position(rx.payload) {
                vprintln!("position from node {} rssi {}", rx.src, rx.rssi);
                let mut v = [0u8; ble::REMOTE_LEN];
                v[0] = rx.src;
                v[1..3].copy_from_slice(&rx.rssi.to_le_bytes());
                v[3..].copy_from_slice(&p.encode());
                state::record_remote(now_ms, Report::Position(v));
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
        // Two nodes in one turn overlap on the air every time they both
        // have something to say, and neither can hear the other to notice
        // - so it is reported from here, by a node that hears both. The
        // cure is an address that maps to a free turn, or an interval long
        // enough to have one.
        if let Some(src) = heard_from {
            let beacon_ms = u32::from(self.cfg.beacon_interval_s) * 1_000;
            let (byte, bit) = (usize::from(src / 8), 1u8 << (src % 8));
            if self.turn_warned[byte] & bit == 0
                && self.cfg.role.transmits()
                && self.node.radio().shares_turn(src, beacon_ms)
            {
                self.turn_warned[byte] |= bit;
                status_println!(
                    "hop: node {} shares this node's turn - renumber, or lengthen interval_s",
                    src
                );
            }
        }
    }

    /// Take a call to a sleeping node, if this node can make one.
    fn start_wake(&mut self, target: u8, idle: bool) {
        if !self.posture.radio_up() {
            status_println!("wake: cannot call node {} - the radio is not up", target);
            return;
        }
        if !self.cfg.role.transmits() {
            status_println!("wake: cannot call node {} - this node never transmits", target);
            return;
        }
        if self.waker.is_some() {
            status_println!("wake: a call is already in progress, node {} replaces it", target);
        }
        self.wake_nonce = self.wake_nonce.wrapping_add(1);
        self.waker = Some(Waker::new(target, idle, self.wake_nonce));
        status_println!(
            "wake: calling node {}{}",
            target,
            if idle { " to come up idle" } else { "" }
        );
    }

    /// One step of a call in progress: a wake frame when the burst is due
    /// one and the air is free, otherwise a listen.
    ///
    /// A wake frame is seconds on the air, so it is gated the way a beacon
    /// is - by a transfer, a pending sleep and a frame arriving - and
    /// skipped for this pass rather than refused when the gate is shut: the
    /// burst tries again on the next pass. Not by the mode, though: a
    /// listening node never beacons, but a call is an operator's explicit
    /// request, and a base station is the natural thing to make one from.
    async fn wake(&mut self, now_ms: u64) {
        let Some(mut w) = self.waker else {
            return;
        };
        match w.step(now_ms) {
            WakeStep::Wait => {}
            WakeStep::Done { heard } => {
                if heard {
                    status_println!("wake: node {} answered", w.target);
                } else {
                    status_println!(
                        "wake: no answer from node {} after {} tries",
                        w.target,
                        sentry::WAKE_TRIES
                    );
                }
                self.waker = None;
                return;
            }
            WakeStep::Send => {
                let allowed = self.posture.radio_up()
                    && self.cfg.role.transmits()
                    && !state::transfer_active()
                    && !state::park_pending();
                if !allowed || self.node.radio().rx_in_progress(now_ms) {
                    return;
                }
                let payload = lora::HEADER_LEN + lora::WAKE_MSG_LEN;
                let s = match sentry::plan(&self.cfg, payload) {
                    Ok(s) => s,
                    Err(r) => {
                        status_println!("wake: cannot call node {} - {}", w.target, r.as_str());
                        self.waker = None;
                        return;
                    }
                };
                // `plan` refused anything the preamble does not fit.
                let syms = s.preamble_symbols().unwrap_or(0) as u16;
                let frame = lora::Wake {
                    target: w.target,
                    idle: w.idle,
                    nonce: w.nonce,
                };
                state::set_radio_busy(true);
                watchdog::beat(Task::Loop, Phase::TxSend);
                self.tx_led.pulse(now_ms);
                let sent = self.node.send_wake(&frame, syms).await;
                state::set_radio_busy(false);
                w.sent(Instant::now().as_millis());
                match sent {
                    Ok(()) => {
                        self.tx_count = self.tx_count.saturating_add(1);
                        status_println!(
                            "wake: called node {} ({} symbol preamble, {} ms on air), listening",
                            w.target,
                            syms,
                            s.wake_airtime_us(&self.cfg, payload).unwrap_or(0) / 1_000
                        );
                    }
                    Err(e) => status_println!("wake: TX to node {} failed ({:?})", w.target, e),
                }
            }
        }
        self.waker = Some(w);
    }

    /// Repeat forwarding. Only a node configured as a repeater ever has
    /// one of these queued; a leaf-only network never enters this. A
    /// listening repeater hears them and drops them: nothing goes out on
    /// the air in that mode.
    async fn repeat(&mut self, now_ms: u64) {
        if !self.node.repeat_due(now_ms)
            || !self.posture.may_transmit(
                self.cfg.role,
                state::transfer_active(),
                state::park_pending(),
            )
            || self.node.radio().rx_in_progress(now_ms)
        {
            return;
        }
        state::set_radio_busy(true);
        watchdog::beat(Task::Loop, Phase::TxSend);
        self.tx_led.pulse(now_ms);
        let went = self.node.send_due_repeat(now_ms).await;
        state::set_radio_busy(false);
        if went {
            self.tx_count = self.tx_count.saturating_add(1);
        }
    }

    fn telemetry(&mut self, now_ms: u64) {
        let secs_since_rx = match self.node.last_rx_ms() {
            Some(t) => (now_ms.saturating_sub(t) / 1000).min(0xFFFE) as u16,
            None => 0xFFFF,
        };
        let mut flags = 0u8;
        if self.gps.has_fix() {
            flags |= link::TELEM_FLAG_GPS_FIX;
        }
        if self.cfg_loaded {
            flags |= link::TELEM_FLAG_CFG_LOADED;
        }
        if self.cfg.verbose {
            flags |= link::TELEM_FLAG_VERBOSE;
        }
        let (stratum, hop_channel) = self.node.radio().hop_status(now_ms);
        state::set_telemetry(link::Telemetry {
            last_rssi: self.node.last_rssi(),
            last_snr_cb: self.node.radio().last_snr_cb(),
            secs_since_rx,
            rx_count: self.rx_count,
            tx_count: self.tx_count,
            flags,
            sats: self.gps.packet().sats,
            hop: link::TELEM_HOP_ON | stratum,
            hop_channel,
            parks_missed: settings::parks_missed().min(u32::from(u8::MAX)) as u8,
            // Filled in by the notifier, which holds the connection this
            // measures; nothing on this side of the board can read it.
            ble_rssi: 0,
        });
    }

    /// The status display. `flush` is a no-op when nothing changed.
    async fn panel(&mut self, now_ms: u64) {
        if self.j5.is_none() || self.posture.live == Mode::Stored || now_ms < self.next_panel_ms {
            return;
        }
        self.next_panel_ms = now_ms + PANEL_MS;
        let telemetry = state::telemetry();
        let own = self.gps.packet();
        let target = state::compass_target(now_ms);
        if let Some(j) = self.j5.as_mut() {
            // Sampled every refresh whether or not a panel is fitted: the
            // hard-iron calibration only improves by being fed, and a
            // board being carried is calibrating itself.
            if let Some(c) = j.compass.as_mut() {
                c.sample(&mut j.i2c).await;
            }
            if let Some(o) = j.oled.as_mut() {
                draw_screen(o, j.compass.as_ref(), telemetry, &own, target, self.cfg.address);
                o.flush(&mut j.i2c).await;
            }
        }
    }

    /// The periodic status line. Without it a quiet radio and a quiet GPS
    /// look identical from the console.
    fn status(&mut self, now_ms: u64) {
        if now_ms < self.next_status_ms {
            return;
        }
        // Idle fraction over the window just ended, not since boot: a
        // cumulative figure would average away exactly the thing worth
        // seeing, which is a core that stopped halting at some point.
        let idle_now = crate::idle::entries();
        let idle_hz = crate::idle::rate(self.idle_mark, idle_now, now_ms - self.idle_at_ms);
        self.idle_mark = idle_now;
        self.idle_at_ms = now_ms;
        self.next_status_ms = now_ms + STATUS_MS;
        // Only asked of a radio that is configured. A radio in cold sleep
        // is woken into standby by the asking - the NSS edge is its wake -
        // and stays there at half a milliamp until the next init, which on
        // an idle board is never; and a radio left as a sentry is ended by
        // any transaction that lands in its sleep phase. Both are told
        // from the posture rather than read.
        let (mode, err) = match self.posture.radio {
            Radio::Asleep => ("asleep", 0),
            Radio::Sentry => ("sentry", 0),
            _ if self.posture.parked => ("parked", 0),
            _ => self.node.radio_mut().health(),
        };
        let (hop_stratum, hop_ch) = self.node.radio().hop_status(now_ms);
        // The heap beside the idle rate: the BLE duty cycle builds and
        // tears the whole stack down every window, and a free figure that
        // drifts down across a day is fragmentation - the failure that
        // shows up as a connect that never completes, hours in.
        status_println!(
            "t={}s radio {} err {:04x} rx {} tx {} hop ch {} s {} | gps {} nmea fix {} sats {} | nodes {} | idle {} Hz | heap {} B free",
            now_ms / 1000,
            mode,
            err,
            self.rx_count,
            self.tx_count,
            hop_ch,
            hop_stratum,
            self.gps.rx_sentences(),
            self.gps.has_fix() as u8,
            self.gps.packet().sats,
            state::remote_count(),
            idle_hz,
            esp_alloc::HEAP.free()
        );
        // Verbose only: break down what the radio heard but did not
        // deliver, so "a couple of random RXs" can be read as mostly CRC
        // failures (weak signal / parameter mismatch), duplicates, or this
        // node's own echoes rather than a mystery. Counts are cumulative
        // since boot.
        let d = self.node.rx_drops();
        vprintln!(
            "dropped: crc {} dup {} echo {} malformed {} oversize {} repeat-full {}",
            self.node.radio().rx_crc_errors(),
            d.duplicate,
            d.own_echo,
            d.malformed,
            self.node.radio().rx_oversize(),
            d.repeat_full
        );
    }

    /// Read the stored config and adopt it.
    ///
    /// A wake check defers this: the config is not needed until the radio
    /// is, and a promotion runs it then. Everything here is about the
    /// *stored* config - the radio settings, the console verbosity, the
    /// `[power]` section - so it costs nothing on a wake nobody answers.
    ///
    /// The store is the record in the board's own flash, which every push
    /// writes. A board that has never taken a push runs the firmware
    /// defaults, node address included.
    ///
    /// Returns whether a config was adopted at all.
    async fn adopt_stored_config(&mut self) -> bool {
        watchdog::beat(Task::Loop, Phase::ConfigLoad);
        let mut text = [0u8; bulk::CONFIG_MAX];
        let loaded = match flash::with_flash(|f| f.load_config(&mut text)).await.flatten() {
            Some(n) => match radiocfg::parse_bytes(&text[..n]) {
                Ok(c) => {
                    println!("config: flash record loaded (address {})", c.address);
                    self.cfg = c;
                    true
                }
                // Only a config that parsed is ever written, so this is
                // the record disagreeing with a firmware that has since
                // changed what it accepts - not a bad push. Defaults, and
                // say so.
                Err(e) => {
                    println!("config: flash record invalid ({:?}), using defaults", e);
                    false
                }
            },
            None => {
                println!("config: none stored, using defaults");
                false
            }
        };
        state::set_verbose(self.cfg.verbose);
        state::set_radio_config(self.cfg.encode());
        state::set_tx_worst_case_ms(self.cfg.tx_worst_case_ms());
        adopt_power(&self.cfg, self.cold).await;
        loaded
    }

    /// Adopt a radio config that arrived over BLE or USB.
    ///
    /// The transfer already parsed it - a config that would not parse
    /// never reaches here, and the host was told so in the ack. What is
    /// left is the hardware: the radio, the node's own addressing, the
    /// GPS, and the flash record that has to survive a reboot.
    async fn apply_radio_config(&mut self, now_ms: u64) -> bool {
        let mut raw = [0u8; bulk::CONFIG_MAX];
        let Some((new_cfg, len)) = xfer::take_pending(&mut raw) else {
            return false;
        };
        let regps = new_cfg.gps != self.cfg.gps;
        self.cfg = new_cfg;
        // Unlike at boot this is unconditional: a push is somebody
        // deliberately sending this file now, so it outranks whatever is
        // live. It is also the only way `[power]` reaches a board that is
        // already running.
        adopt_power(&self.cfg, true).await;
        self.node.radio_mut().init(&self.cfg).await;
        self.node.reconfigure(&self.cfg);
        state::set_verbose(self.cfg.verbose);
        state::set_radio_config(self.cfg.encode());
        state::set_tx_worst_case_ms(self.cfg.tx_worst_case_ms());
        // A write that did not reach flash has to be reported to the
        // operator rather than left in a console nobody is reading - it
        // is the difference between a config that is applied and one that
        // is applied until the next reboot.
        let in_flash = flash::with_flash(|f| f.save_config(&raw[..len]))
            .await
            .unwrap_or(false);
        event!(
            Kind::Transfer,
            "config applied, node {} ({}), {}",
            self.cfg.address,
            self.cfg.role.as_str(),
            if in_flash {
                "saved to flash"
            } else {
                "NOT SAVED - lost on reboot"
            }
        );
        if regps && !self.gps.sleeping {
            if self.gps.configure(&self.cfg.gps).await {
                status_println!("gps reconfigured");
            } else {
                status_println!("gps did not accept settings");
            }
        }
        let _ = now_ms;
        true
    }
}

/// Let a config file set the duty cycle.
///
/// `cold` gates this to a cold boot, and the distinction is the whole
/// design. The settings in `[power]` are also kept in RTC RAM so they
/// survive a deep sleep, and an app can change them live over BLE - so
/// re-reading the card on every wake check would undo a live change once an
/// interval, forever. On a cold boot there is nothing live to undo and the
/// card is the only thing that outlives a reflash, so it wins.
///
/// A file that mentions none of them changes nothing and costs no flash
/// write, which is the common case.
///
/// One thing this cannot catch up with: the advertising window of the very
/// first window is fixed before the card is mounted, so a `adv_window_s`
/// from the file takes effect from the second window on. `ble_off_s` is
/// re-read at the end of every window and has no such lag.
async fn adopt_power(cfg: &RadioConfig, cold: bool) {
    let asked = cfg.power;
    if !cold {
        if !asked.is_empty() {
            qprintln!("config: [power] ignored on a wake, the live settings win");
        }
        return;
    }
    if !settings::adopt_power(&asked) {
        return;
    }
    let now = settings::get();
    status_println!(
        "config: [power] adopted - ble off {} s, ble on {} s, adv window {} s, sleep interval {} s, idle timeout {} s",
        now.ble_off_s,
        now.ble_on(),
        now.adv_window(),
        now.sleep_interval_s,
        now.idle_timeout()
    );
    // Mirrored to flash for the same reason a BLE write to any of these is:
    // they decide whether the board is reachable at all, and a board that
    // came back from a flat cell without them would advertise continuously
    // until it died again.
    settings::save().await;
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
    panel: &mut oled::Oled,
    compass: Option<&crate::compass::Compass>,
    telemetry: Option<link::Telemetry>,
    own: &packet::PositionPacket,
    target: Option<(u8, packet::PositionPacket, u16, i16)>,
    node_address: u8,
) {
    let Some((node, remote, age_s, rssi)) = target else {
        oled::render(panel, telemetry, node_address);
        return;
    };
    if !own.has_fix() || !remote.has_fix() {
        oled::render(panel, telemetry, node_address);
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
    oled::render_compass(panel, &target, heading, fix, sats);
}

/// The hardware loop.
#[embassy_executor::task]
pub async fn hardware_task(
    lora: Sx1262Driver<'static>,
    gps: Gps<'static>,
    j5: Option<J5>,
    d5: Output<'static>,
    d2: Output<'static>,
    boot: Mode,
    cold: bool,
) {
    let stored = settings::get();
    let (posture, boot_fx) = Posture::at_boot(boot, &stored);
    let mut hw = Hardware::new(lora, gps, j5, d5, d2, posture, cold);
    hw.boot(boot, boot_fx, &stored).await;
    #[cfg(any(feature = "bench-panic", feature = "bench-stall"))]
    let fault_at = Instant::now() + Duration::from_secs(20);
    loop {
        #[cfg(feature = "bench-panic")]
        if Instant::now() >= fault_at {
            panic!("bench: a deliberate panic in the hardware loop");
        }
        #[cfg(feature = "bench-stall")]
        if Instant::now() >= fault_at {
            status_println!("bench: the hardware loop stops here on purpose");
            loop {
                core::hint::spin_loop();
            }
        }
        let wait = hw.pass().await;
        Timer::after(wait).await;
    }
}
