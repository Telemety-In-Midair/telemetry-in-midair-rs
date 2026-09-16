//! SX1262 radio driver, ported from the WIO-E5 firmware's `radio.rs`.
//!
//! The logic and every tuned value carry over unchanged - the WL's SubGHz
//! block is the same die and takes the same commands, so only the
//! transport moved (see [`crate::sx1262`]). Modulation parameters come
//! from a runtime [`RadioConfig`] rather than a compile-time preset, so
//! [`Sx1262Driver::init`] can be called again on a live radio after a
//! config push.
//!
//! Three things changed with the hardware:
//!
//! - The antenna switch is inside the module instead of on two MCU pins,
//!   so `RfPath` is gone. The radio drives it from DIO2 and supplies it
//!   from DIO3, both of which `init` fixes at the board's values rather
//!   than reading from the config.
//! - DIO1 is a real pin, so a receive poll costs a GPIO read rather than
//!   an SPI round trip until something actually arrives.
//! - Transmit waits are `.await` rather than a spin that has to feed the
//!   watchdog, so BLE and the rest of the firmware keep running through a
//!   slow beacon instead of being locked out for up to 10 s.
//!
//! Frequency hopping lives here too, because it is a property of where the
//! radio is tuned rather than of what the node says. Every config has a
//! hop plan ([`midair_proto::hop`]): the receiver retunes at every slot
//! boundary of the network's clock, a transmit is held to this node's turn
//! of the slot's window and stamped with that clock on its way out, and
//! every frame heard is offered to the clock as a reference.
//!
//! The default plan is one channel wide, so by default the retune is a
//! no-op and everything else - the clock, the turns, the sync word - is
//! what the plan is there for. When to transmit is decided above this
//! layer, by [`midair_proto::beacon`], against the clock and plan this
//! exposes through [`Sx1262Driver::schedule`].
//!
//! Two timing details the clock leans on. The sync word describes the
//! instant the preamble leaves the antenna, not the instant the command
//! was written: from a cold oscillator the two are `tcxo_startup_ms`
//! apart, which a follower would otherwise inherit as an error at every
//! stratum. And a poll that comes late - after a transmit, or a card
//! flush that stalled the loop - stamps whatever it reads with a time that
//! could be anywhere in the gap, so such a reading is not allowed to move
//! a clock that is already set.

use embassy_time::{Duration, Instant, Timer};
use esp_println::println;
use midair_proto::evlog::Kind;
use midair_proto::hop::{self, Offer, SyncWord};
use midair_proto::lora::FRAME_MAX;
use midair_proto::radiocfg::RadioConfig;
use midair_proto::rxgate::{Irq, RxGate, Seen};
use midair_proto::sentry;
use midair_proto::supervise::{Phase, Task};

use crate::sx1262::{dev_err, irq, mode, reg, FallbackMode, StandbyClk, Sx1262, RX_CONTINUOUS};

#[derive(Debug)]
pub enum Sx1262Error {
    Timeout,
}

/// Payload length written into the packet params before receiving.
///
/// With an explicit header this field is not the length of anything - the
/// header carries that - it is the largest payload the receiver will
/// accept. Every transmit has to narrow it to the size of the frame being
/// sent, so receiving means putting it back.
const RX_MAX_PAYLOAD: u8 = 255;

/// Time added to the header time before a preamble with no header behind
/// it is given up on, ms: one poll period each for seeing the preamble and
/// for seeing the header.
const HEADER_SLACK_MS: u32 = 20;

/// Lead from a transmit command to the preamble leaving the antenna when
/// the oscillator is already running, ms: the PLL lock and the PA ramp.
const TX_LEAD_WARM_MS: u32 = 1;

/// Lowest `SetDio3AsTcxoCtrl` trim the Wio-S3 may be driven at: 2.7 V.
///
/// DIO3 is not only the TCXO supply on this module. It is also the VDD of
/// the SKY13453-385LF antenna switch, specified 2.5 - 3.5 V, and the
/// switch's truth table calls any state outside that undefined. A config
/// naming a lower voltage - 1.8 V is the Wio-E5 value and was the default
/// here for a while - leaves the switch undefined while the PA transmits
/// into it, so it is raised to the board's own 3.3 V rather than honored.
const TCXO_TRIM_MIN: u8 = 0x5;
/// The trim this board wants: 3.3 V, inside the switch's window with room
/// on both sides.
const TCXO_TRIM_BOARD: u8 = 0x7;

/// Image calibration bounds for operation at `freq_hz`, as the `(f1, f2)`
/// byte pair `CalibrateImage` takes.
///
/// Image rejection is calibrated for a band, and calibrating for a band the
/// radio is not operating in throws away the rejection - which is
/// sensitivity, and so range. The config accepts anything from 150 to 960
/// MHz, well past the ISM bands the datasheet tabulates, so a frequency
/// outside all of them gets the 4 MHz-aligned window that brackets it
/// rather than the nearest named band.
fn image_band(freq_hz: u32) -> (u8, u8) {
    match freq_hz / 1_000_000 {
        902..=928 => (0xE1, 0xE9),
        863..=870 => (0xD7, 0xDB),
        779..=787 => (0xC1, 0xC5),
        470..=510 => (0x75, 0x81),
        430..=440 => (0x6B, 0x6F),
        mhz => {
            let f1 = (mhz / 4) as u8;
            (f1, f1 + 1)
        }
    }
}

/// What the schedule keeps between polls.
struct Hop {
    plan: hop::Plan,
    clock: hop::Clock,
    /// The slot the radio is tuned for, so a poll can tell a boundary has
    /// passed. `None` after an init, which tunes for the slot it ran in.
    rx_slot: Option<u32>,
    /// The carrier the radio is on, Hz. A slot boundary that maps to the
    /// same carrier - every one on the one-channel default - is noted
    /// without touching the radio.
    carrier_hz: u32,
}

pub struct Sx1262Driver<'d> {
    radio: Sx1262<'d>,
    /// The config the radio was last initialized from. Hopping needs the
    /// modulation to turn a frame length into a time on air.
    cfg: RadioConfig,
    hop: Hop,
    /// Whether [`init`](Self::init) has run: what decides whether the hop
    /// clock it finds is one worth keeping.
    configured: bool,
    /// What the receiver has seen of a frame that is arriving, and how
    /// long a hop or a transmit has to wait for it. Host-tested, and
    /// walked exhaustively by the state space tests.
    gate: RxGate,
    /// This node's address: which turn of a slot its transmissions take.
    address: u8,
    /// Whether the oscillator is kept running between modes. A listening
    /// node's radio is in receive nearly all the time, so standby with the
    /// oscillator up costs it nothing worth the `tcxo_startup_ms` a cold
    /// start puts in front of every transmit and every retune - and the
    /// same lead in every sync word it sends. A transmit-only node idles
    /// in standby for whole seconds and keeps the cold one.
    xosc: bool,
    /// Local time the last packet finished arriving, which with its length
    /// gives the instant it began - what a hop clock is measured against.
    last_rx_done_ms: u64,
    /// Local `(start, end)` of the last transmission, whatever it carried.
    /// The start places it in a hop slot, which no second transmission may
    /// share; the end is what a single-channel interval is measured from.
    last_tx: Option<(u64, u64)>,
    rx_active: bool,
    /// Whether the receiver is used at all. False on a transmit-only node,
    /// which idles in standby instead of continuous RX - that idle current
    /// is the whole reason the mode exists.
    listen: bool,
    /// SNR of the last received packet in centibels.
    last_snr_cb: i16,
    tx_poll_timeout_ms: u32,
    tx_chip_timeout_ms: u32,
    /// Packets delivered by the radio that failed the hardware CRC, since
    /// boot (saturating). A large count next to few good receptions points
    /// at a weak signal or a link-parameter mismatch rather than nothing on
    /// the air.
    rx_crc_errors: u32,
    /// Packets longer than the caller's buffer, dropped unread (saturating).
    rx_oversize: u32,
}

impl<'d> Sx1262Driver<'d> {
    /// Wrap the radio. Call [`init`](Self::init) before use.
    pub fn new(radio: Sx1262<'d>) -> Self {
        let cfg = RadioConfig::default();
        let plan = hop::Plan::from_config(&cfg);
        Self {
            radio,
            hop: Hop {
                plan,
                clock: hop::Clock::new(plan.dwell_ms, 0, 0),
                rx_slot: None,
                carrier_hz: 0,
            },
            configured: false,
            cfg,
            gate: RxGate::new(0, 0),
            address: 0,
            xosc: false,
            last_rx_done_ms: 0,
            last_tx: None,
            rx_active: false,
            listen: true,
            last_snr_cb: 0,
            tx_poll_timeout_ms: 250,
            tx_chip_timeout_ms: 750,
            rx_crc_errors: 0,
            rx_oversize: 0,
        }
    }

    /// Packets dropped since boot for a bad CRC. Saturates; never cleared
    /// except by reboot.
    pub fn rx_crc_errors(&self) -> u32 {
        self.rx_crc_errors
    }

    /// Packets dropped since boot for exceeding the receive buffer.
    pub fn rx_oversize(&self) -> u32 {
        self.rx_oversize
    }

    /// SNR of the last received packet in centibels (dB * 100).
    pub fn last_snr_cb(&self) -> i16 {
        self.last_snr_cb
    }

    /// Initialize (or re-initialize) the radio from `cfg`.
    pub async fn init(&mut self, cfg: &RadioConfig) {
        self.rx_active = false;
        self.listen = cfg.role.receives();
        self.xosc = self.listen;
        self.address = cfg.address;
        self.tx_poll_timeout_ms = cfg.tx_poll_timeout_ms();
        self.tx_chip_timeout_ms = cfg.tx_chip_timeout_ms();
        self.cfg = *cfg;
        // The two holds: a preamble for as long as a header would take to
        // follow it, a header for the longest frame the modulation allows.
        self.gate.retime(
            cfg.header_time_us().div_ceil(1000) + HEADER_SLACK_MS,
            cfg.time_on_air_us(FRAME_MAX).div_ceil(1000),
        );

        // The hop clock outlives a re-init when the slot length does: a
        // config push, a standby the app asked for, a radio that browned out
        // - none of them is a reason to forget what time the network keeps,
        // and forgetting it costs a rejoin of up to a cycle per node heard.
        let now = Instant::now().as_millis();
        let plan = hop::Plan::from_config(cfg);
        let clock = if self.configured && self.hop.clock.dwell_ms() == u32::from(plan.dwell_ms) {
            self.hop.clock
        } else {
            hop::Clock::new(plan.dwell_ms, now, u32::from(cfg.address))
        };
        self.hop = Hop {
            plan,
            clock,
            rx_slot: None,
            carrier_hz: 0,
        };
        self.configured = true;

        // A discrete radio does not reset with the MCU, so a re-init after
        // a wedge has to say so explicitly.
        self.radio.reset().await;
        self.radio.set_standby(StandbyClk::Rc);

        // DC-DC roughly halves RX/TX current, but only works on a board
        // with the SMPS inductor fitted, so it stays configurable.
        //
        // Clock detection has to be enabled before the SMPS is, not after,
        // so it is written unconditionally here rather than alongside the
        // mode below - it costs nothing on a board running the LDO.
        //
        // Read-modify-write: writing the whole register would enable clock
        // detection by clearing every other bit of the regulator's config.
        self.radio
            .modify_reg(reg::SMPS_C0, |v| v | reg::SMPS_CLK_DET_EN);
        self.radio.set_regulator_mode(cfg.dcdc_enabled);

        // DIO3 first, then DIO2: supply the antenna switch before handing
        // the radio its control line.
        //
        // Ordering matters less than it looks - DIO2 is low in STDBY_RC, so
        // the switch sees VCTL and VDD both at 0 either way, which its
        // datasheet specifies as a leakage condition rather than a damaging
        // one. But VCTL is only allowed to sit at or below VDD, and there
        // is no reason to write the two commands in the order that needs
        // that argument made.

        // DIO3 supplies the 32 MHz TCXO *and* the SKY13453-385LF's VDD, so
        // the trim is floored at the switch's 2.5 V minimum rather than
        // taken as given. The radio waits `tcxo_startup_ms` for both to
        // come up before it will use the clock.
        let trim = cfg.tcxo_volts.trim();
        let trim = if trim < TCXO_TRIM_MIN {
            println!(
                "config: tcxo_volts trim {} is under the RF switch's 2.5 V floor, using 3.3 V",
                trim
            );
            TCXO_TRIM_BOARD
        } else {
            trim
        };
        self.radio
            .set_tcxo_ctrl(trim, cfg.tcxo_startup_ms as u32);

        // The radio drives this board's antenna switch itself: DIO2 is the
        // SKY13453-385LF's VCTL. Unconditional, and deliberately not read
        // from the config - a config that said false would put the PA into
        // an isolated port on every transmission, and no setting a user can
        // reach should be able to ask for that.
        if !cfg.dio2_rf_switch {
            println!("config: dio2_rf_switch=false ignored, this board needs it on");
        }
        self.radio.set_dio2_as_rf_switch(true);

        // Recalibrate every block now that there is a 32 MHz clock. The
        // automatic calibration at power-up ran before the TCXO was
        // enabled, so the RC64k, RC13M, PLL, ADC and image results it
        // produced were all derived from a clock that was not running -
        // which shows up as frequency error and lost sensitivity rather
        // than as a failure. 0x7F selects every block.
        self.radio.calibrate(0x7F);

        let (f1, f2) = image_band(cfg.frequency_hz);
        self.radio.calibrate_image(f1, f2);

        self.radio.set_packet_type_lora();
        // The carrier is the current slot's; the poll moves it from there.
        // Tuned once here so a transmit-only node, which never polls,
        // still starts somewhere in the plan. On a one-channel plan every
        // slot's carrier is `frequency_hz`.
        let slot = self.hop.clock.slot(now);
        self.hop.rx_slot = Some(slot);
        self.hop.carrier_hz = self.hop.plan.frequency_for_slot(slot);
        self.radio.set_rf_frequency(self.hop.carrier_hz);

        // High-power PA, duty/hp_max per the +22 dBm datasheet preset; the
        // actual output level is set via the TX params below.
        self.radio.set_pa_config(0x04, 0x07);
        // Ramp time 0x04 = 200 us.
        //
        // Clamped rather than passed through. The TOML parser range-checks
        // this key, but `RadioConfig::decode` does not - it takes the byte
        // as an i8 - so a blob from anywhere else can name a level the HP
        // PA has no setting for. The datasheet gives -9..+22 and says
        // nothing about what the part does outside it, which is not a
        // question to answer with a PA.
        self.radio
            .set_tx_params(cfg.power_dbm.clamp(-9, 22), 0x04);

        // Applied after the PA is configured, since configuring it is what
        // this compensates for. The board's antenna is a connector and a
        // short wire, so the mismatch this guards the PA against is the
        // normal case rather than a fault.
        self.radio.modify_reg(reg::TX_CLAMP, |v| v | 0x1E);

        // Receive-side counterpart to the TX power above. Not covered by
        // warm-start retention, so it is rewritten on every entry to this
        // function rather than set once - which is what happens anyway,
        // since `init` is what brings the radio back from both sleep and a
        // config push.
        self.radio.write_reg(
            reg::RX_GAIN,
            if cfg.rx_boost {
                reg::RX_GAIN_BOOSTED
            } else {
                reg::RX_GAIN_POWER_SAVING
            },
        );

        let sf = cfg.spreading_factor.clamp(5, 12);
        let bw = match cfg.bandwidth_khz {
            62 => 0x03,
            250 => 0x05,
            500 => 0x06,
            _ => 0x04, // 125 kHz
        };
        let cr = match cfg.coding_rate {
            6 => 0x02,
            7 => 0x03,
            8 => 0x04,
            _ => 0x01,
        };
        self.radio.set_lora_mod_params(sf, bw, cr, cfg.ldro());

        // Bandwidth-dependent, and the modulation params are what carry the
        // bandwidth, so this follows them. Only 500 kHz wants the bit
        // clear; every other bandwidth wants it set, which is also the
        // reset value, so this only ever undoes itself after a config push
        // moved off 500.
        self.radio.modify_reg(reg::TX_MODULATION, |v| {
            if cfg.bandwidth_khz == 500 {
                v & !0x04
            } else {
                v | 0x04
            }
        });

        self.radio.set_lora_packet_params(RX_MAX_PAYLOAD);

        // Private (0x1424), not the public LoRaWAN word: on the public one
        // the receiver locks onto every LoRaWAN preamble in earshot, and
        // the time it spends failing to decode a frame that was never ours
        // is time it is not hearing the network. It also inflates the
        // CRC-error count the status line asks an operator to read as
        // signal quality.
        //
        // Nodes on different sync words cannot hear each other at all, so
        // this is a flag day: a fleet has to be reflashed together.
        let (msb, lsb) = reg::SYNC_WORD_PRIVATE;
        self.radio.write_reg(reg::LORA_SYNC_WORD_MSB, msb);
        self.radio.write_reg(reg::LORA_SYNC_WORD_LSB, lsb);

        self.radio.set_buffer_base_address(0x00, 0x00);
        // Preamble and header as well as the packet ends: they are how the
        // poll knows a frame is arriving, which is what holds a hop and a
        // transmit off a channel somebody is mid-sentence on.
        self.radio.set_dio_irq_params(
            irq::RX_DONE
                | irq::TX_DONE
                | irq::CRC_ERR
                | irq::TIMEOUT
                | irq::PREAMBLE_DETECTED
                | irq::HEADER_VALID
                | irq::HEADER_ERR,
        );
        // Where the chip lands after a packet: a listening node keeps the
        // oscillator running (see `xosc`), so its next receive or transmit
        // starts without the TCXO's startup in front of it.
        self.radio.set_rx_tx_fallback_mode(if self.xosc {
            FallbackMode::StandbyXosc
        } else {
            FallbackMode::StandbyRc
        });

        // Over-current protection: required for the HP PA to reach +22.
        self.radio.write_reg(reg::OCP, reg::OCP_140MA);

        // Drop the errors the power-up sequence latched before this function
        // had a chance to fix its cause.
        //
        // The automatic calibration at reset runs while DIO3 is still low,
        // so the TCXO has no supply, so the XOSC it needs never starts and
        // the chip latches XOSC_START_ERR (0x0020). It is not a fault - it
        // is the state this function exists to correct, and the recalibrate
        // above is the correction. Leaving it latched means the boot
        // diagnostic warns every single time, which trains an operator to
        // ignore the one register that would name a TCXO that really did
        // fail, a PLL that never locked, or a PA that would not ramp.
        //
        // Anything read after this point is real.
        self.radio.clear_device_errors();

        println!(
            "radio init: {} Hz SF{} BW{} CR4/{} {} dBm",
            cfg.frequency_hz,
            cfg.spreading_factor,
            cfg.bandwidth_khz,
            cfg.coding_rate,
            cfg.power_dbm
        );
        {
            let h = &self.hop;
            let (lo, hi) = h.plan.span_hz();
            println!(
                "radio hop: {} ch x {} kHz, {}-{} Hz, {} ms dwell, {} turns a slot (mine {}), stratum {}",
                h.plan.channels,
                h.plan.step_khz,
                lo,
                hi,
                h.plan.dwell_ms,
                h.plan.sub_slots,
                h.plan.sub_slot_of(cfg.address, 1),
                h.clock.stratum(now)
            );
            // Said once here rather than on every beacon. A frame longer
            // than the window still goes out - receivers hold their hop for
            // a frame in progress - but it is one channel occupied for
            // longer than a hop is meant to be, which is what the band
            // rules cap.
            let beacon_ms = cfg.beacon_airtime_us().div_ceil(1000);
            if !h.plan.fits(beacon_ms) {
                println!(
                    "config: the {} ms beacon does not fit the {} ms hop window; lengthen hop_dwell_ms or shorten the frame",
                    beacon_ms,
                    h.plan.window_ms()
                );
            }
        }
    }

    /// The hop clock's stratum and the channel the radio is on. What the
    /// telemetry and the status line report.
    pub fn hop_status(&self, now_ms: u64) -> (u8, u8) {
        let h = &self.hop;
        (h.clock.stratum(now_ms), h.plan.index_for_slot(h.clock.slot(now_ms)))
    }

    /// The schedule a transmit is planned against: the node's clock and
    /// the network's plan. What [`midair_proto::beacon::Planner`] takes.
    pub fn schedule(&mut self) -> (&mut hop::Clock, &hop::Plan) {
        (&mut self.hop.clock, &self.hop.plan)
    }

    /// Whether the receiver is mid-frame: a preamble or header has been
    /// seen and the packet end has not. A transmit started now would
    /// trample it, and on a hopping network the hop waits for it too.
    /// Bounded, since a preamble detection can be noise: by the time a
    /// header would take to follow it, and once a header has, by the
    /// longest frame the modulation allows.
    pub fn rx_in_progress(&self, now_ms: u64) -> bool {
        self.gate.in_progress(now_ms)
    }

    /// The local time a one-off frame of `frame_len` bytes - a repeat -
    /// should be sent, at or after `now_ms`: a random point in this node's
    /// turn of the current or next slot's window. One transmission per
    /// slot, whatever it carries: a beacon and a repeat in the same slot
    /// would be two visits' worth of air on one channel, which is the
    /// thing the band's hopping rule caps, so a slot that already had one
    /// plans for the next. Beacons are planned by
    /// [`midair_proto::beacon::Planner`] against [`schedule`](Self::schedule)
    /// by the same rule.
    pub fn repeat_start(&mut self, now_ms: u64, frame_len: usize) -> u64 {
        let airtime_ms = self.cfg.time_on_air_us(frame_len).div_ceil(1000);
        let h = &mut self.hop;
        let from = match self.last_tx {
            Some((start, _)) if h.clock.slot(start) == h.clock.slot(now_ms) => {
                h.clock.next_slot_start_ms(now_ms)
            }
            _ => now_ms,
        };
        h.clock.tx_start(&h.plan, self.address, 0, from, airtime_ms)
    }

    /// Whether node `other`, beaconing every `interval_ms`, takes the same
    /// turn as this node: the fleet has outgrown the plan's turns, or two
    /// addresses were chosen `turns` apart, and the two overlap on the air
    /// every time.
    pub fn shares_turn(&self, other: u8, interval_ms: u32) -> bool {
        let h = &self.hop;
        let n = h.clock.slots_for(interval_ms);
        other != self.address
            && h.plan.turn_slot(other, n) == h.plan.turn_slot(self.address, n)
            && h.plan.sub_slot_of(other, n) == h.plan.sub_slot_of(self.address, n)
    }

    /// The hop plan.
    pub fn hop_plan(&self) -> hop::Plan {
        self.hop.plan
    }

    /// Local `(start, end)` of the last transmission, if any.
    pub fn last_tx_span(&self) -> Option<(u64, u64)> {
        self.last_tx
    }

    /// Set the hop clock from the board's own GPS: `tod_ms` is the time of
    /// day a fix reported, `at_ms` when that report was parsed. Returns
    /// whether the clock was not already on GPS time, for the log.
    pub fn hop_discipline_gps(&mut self, tod_ms: u32, at_ms: u64) -> bool {
        let h = &mut self.hop;
        let was_gps = h.clock.from_gps(at_ms);
        h.clock.discipline_gps(tod_ms, at_ms);
        !was_gps
    }

    /// Offer the clock a frame just received: `word` is what it carried,
    /// `src` who sent it, `my_addr` this node's address for the tie-break,
    /// and `frame_len` its length, which with the modulation places the
    /// start of the transmission at a known instant before the packet end
    /// the poll timestamped.
    pub fn hop_heard(&mut self, word: SyncWord, src: u8, my_addr: u8, frame_len: usize) -> Offer {
        let h = &mut self.hop;
        // The packet end this poll timestamped could be anywhere in the
        // gap since the last poll, so it is not a reference for a clock
        // that already has one - the next frame from the same sender
        // re-anchors it in any case. A clock with nothing takes it anyway:
        // a slot that is roughly right finds the network, and the error
        // is corrected by the first well-timed frame.
        if self.gate.poll_late() && h.clock.synced() {
            return Offer::Kept;
        }
        let toa = u64::from(self.cfg.time_on_air_us(frame_len).div_ceil(1000));
        let tx_start = self.last_rx_done_ms.saturating_sub(toa);
        h.clock.offer(word, src, my_addr, tx_start, self.last_rx_done_ms)
    }

    /// Move the receiver to the current slot's channel if the slot has
    /// changed the carrier, unless a frame is arriving on the one it is on.
    ///
    /// A slot boundary that keeps the carrier - every one of them on the
    /// one-channel default, and a cycle boundary that happens to repeat
    /// on a wider plan - is only noted: taking the receiver through
    /// standby to retune it to the frequency it is already on cost it a
    /// millisecond of deafness a second, and a preamble that landed in
    /// that millisecond was a frame lost.
    ///
    /// The hold is what lets a frame cross a slot boundary - a receiver
    /// whose clock runs a little ahead would otherwise leave mid-packet.
    /// It is bounded by what has been seen of the frame (see
    /// [`RxGate`]) and by one slot, whichever is shorter: a preamble that
    /// was noise would otherwise pin the receiver on a channel the network
    /// has left.
    fn hop_tick(&mut self, now_ms: u64) {
        let clk = self.standby_clk();
        // The plan's carrier for this slot is one permutation of the
        // channels, computed once here and compared against the carrier
        // the radio is on.
        let (slot, carrier, same, dwell_ms) = {
            let h = &self.hop;
            let slot = h.clock.slot(now_ms);
            if h.rx_slot == Some(slot) {
                return;
            }
            let carrier = h.plan.frequency_for_slot(slot);
            (slot, carrier, carrier == h.carrier_hz, u32::from(h.plan.dwell_ms))
        };
        if !same && !self.gate.may_leave(now_ms, dwell_ms) {
            return;
        }
        let h = &mut self.hop;
        h.rx_slot = Some(slot);
        if same {
            return;
        }
        h.carrier_hz = carrier;
        self.radio.set_standby(clk);
        self.radio.set_rf_frequency(carrier);
        // Re-armed on the new channel by the poll, which is the one caller.
        self.rx_active = false;
    }

    /// The standby the radio is parked in between modes: with the
    /// oscillator running on a node that will be back in receive within
    /// the millisecond, cold on one that will not.
    fn standby_clk(&self) -> StandbyClk {
        if self.xosc {
            StandbyClk::Xosc
        } else {
            StandbyClk::Rc
        }
    }

    /// Milliseconds from a transmit command to the preamble leaving the
    /// antenna: the oscillator's startup when it is cold, the PLL and the
    /// ramp either way. What the sync word is stamped for.
    fn tx_lead_ms(&self) -> u32 {
        if self.xosc {
            TX_LEAD_WARM_MS
        } else {
            u32::from(self.cfg.tcxo_startup_ms) + TX_LEAD_WARM_MS
        }
    }

    /// The radio's current mode and any latched operational error.
    ///
    /// Packet counters cannot show a radio that is sitting somewhere it
    /// should not be. A node beaconing every 20 s spends almost all of its
    /// time in `rx`, so anything else on a periodic status line - `tx` in
    /// particular - is a radio burning current between transmissions rather
    /// than listening.
    pub fn health(&mut self) -> (&'static str, u16) {
        let name = match self.chip_mode() {
            mode::STDBY_RC => "standby",
            mode::STDBY_XOSC => "standby-xosc",
            mode::FS => "fs",
            mode::RX => "rx",
            mode::TX => "tx",
            _ => "?",
        };
        (name, self.radio.device_errors())
    }

    /// The chip mode from bits 6:4 of the status byte.
    fn chip_mode(&mut self) -> u8 {
        (self.radio.status() >> 4) & 0x07
    }

    /// Whether the radio looks like it restarted underneath the firmware.
    ///
    /// The SX1262 is a separate chip with its own supply, so it can brown
    /// out and come back without the MCU noticing. What it comes back as is
    /// the problem: DIO2 and DIO3 return to their power-up defaults, which
    /// on this board means the antenna switch is unpowered and not
    /// switching, and the next transmit ramps +22 dBm into an isolated
    /// port. Nothing about that looks wrong from the packet counters.
    ///
    /// Two signals, either of which is enough:
    ///
    /// - `XOSC_START_ERR` latched again. [`init`](Self::init) clears it on
    ///   the way out, precisely so that seeing it afterwards means the chip
    ///   ran its power-up calibration a second time.
    /// - A listening node whose radio is not in RX. Continuous receive is
    ///   armed once and persists; standby is where a reset leaves it.
    pub fn looks_reset(&mut self) -> bool {
        let status = self.radio.status();
        // Nothing on the bus at all - not a restart, but equally not a
        // radio that should be keyed up.
        if status == 0x00 || status == 0xFF {
            return true;
        }
        if self.radio.device_errors() & dev_err::XOSC_START != 0 {
            return true;
        }
        self.listen && self.rx_active && (status >> 4) & 0x07 != mode::RX
    }

    /// Whether this node's receiver is enabled at all.
    pub fn listens(&self) -> bool {
        self.listen
    }

    /// Print radio diagnostics. Returns `true` if the radio responds.
    pub fn print_diagnostics(&mut self) -> bool {
        let status = self.radio.status();
        // An unwired or dead radio floats the bus; both ends of the range
        // mean nothing answered rather than a mode that exists.
        if status == 0x00 || status == 0xFF {
            println!("WARNING: radio not responding (status 0x{:02X})", status);
            return false;
        }
        let (mode, err) = self.health();
        println!("radio status 0x{:02X}, mode {}", status, mode);
        if err != 0 {
            println!("WARNING: radio op error 0x{:04X}", err);
            self.radio.clear_device_errors();
        }
        true
    }

    /// Arm the chip's own receive/sleep cycle, with `mask` routed to DIO1.
    ///
    /// The periods are in microseconds and are encoded here, so a caller
    /// works in the units the arithmetic is done in rather than in the
    /// chip's 15.625 us steps. Packet parameters are restored first for the
    /// same reason [`enter_rx`](Self::enter_rx) restores them: a transmit
    /// leaves them carrying its own length.
    ///
    /// `rx_active` stays false. This is not continuous receive, and a poll
    /// that decided the receiver needed re-arming would take the chip
    /// straight back out of the cycle.
    pub fn arm_duty_cycle(&mut self, rx_us: u32, sleep_us: u32, detect_symbols: u8, mask: u16) {
        // Both halves are counted by the chip's RC64k, so both are what the
        // caller asked for only if the caller corrected for its offset.
        // Nothing here does that on the caller's behalf - the correction
        // needs a measurement this driver does not hold.
        self.rx_active = false;
        self.radio.set_standby(StandbyClk::Rc);
        // The receiver gain is outside the chip's warm-start retention set,
        // so without this the boost set at init survives only until the
        // first sleep phase and every window after it listens deafer.
        self.radio.set_retention_list(&[reg::RX_GAIN]);
        self.radio.set_lora_packet_params(RX_MAX_PAYLOAD);
        self.radio.set_lora_symb_num_timeout(detect_symbols);
        self.radio.set_dio_irq_params(mask);
        self.radio.clear_irq_status(irq::ALL);
        self.radio.set_rx_duty_cycle(
            sentry::duty_steps_from_us(rx_us),
            sentry::duty_steps_from_us(sleep_us),
        );
    }

    /// Arm plain continuous receive with `mask` routed to DIO1.
    ///
    /// The control for a duty-cycled arm: same radio, same signal, same
    /// interrupt, but listening the whole time. A source that cannot be
    /// heard by this either is not on the air or is not on these settings,
    /// and that is a different problem from a receive window that is too
    /// short - which the two otherwise report identically.
    pub fn arm_continuous_rx(&mut self, mask: u16) {
        self.rx_active = false;
        self.radio.set_standby(StandbyClk::Rc);
        self.radio.set_lora_packet_params(RX_MAX_PAYLOAD);
        self.radio.set_dio_irq_params(mask);
        self.radio.clear_irq_status(irq::ALL);
        self.radio.set_rx(RX_CONTINUOUS);
    }

    /// Transmit `data` behind a preamble of `preamble_syms`, and wait for it
    /// to leave. No hop clock, no slot, no turn.
    ///
    /// A wake frame is not network traffic. It goes on a rendezvous channel
    /// to a receiver with no clock at all - that is what makes it a wake -
    /// so the slot machinery [`send`](Self::send) exists for has nothing to
    /// say about it, and the preamble runs to hundreds of symbols where the
    /// network uses eight.
    pub async fn send_wake(&mut self, data: &[u8], preamble_syms: u16) -> Result<(), Sx1262Error> {
        self.rx_active = false;
        self.gate.clear();
        let clk = self.standby_clk();
        self.radio.set_standby(clk);
        self.radio.clear_irq_status(irq::ALL);
        self.radio.write_buffer(0x00, data);
        self.radio
            .set_lora_packet_params_preamble(data.len() as u8, preamble_syms);
        self.radio.set_dio_irq_params(irq::TX_DONE | irq::TIMEOUT);
        // The preamble alone can be seconds, so the chip timeout is taken
        // from the airtime rather than from the constant a network frame
        // uses.
        let airtime_ms = self
            .cfg
            .time_on_air_preamble_us(data.len(), u32::from(preamble_syms))
            .div_ceil(1000);
        let budget = airtime_ms + self.tx_chip_timeout_ms;
        self.radio.set_tx(crate::sx1262::timeout_from_millis(budget));

        let deadline = Instant::now() + Duration::from_millis(u64::from(budget));
        while Instant::now() < deadline {
            crate::watchdog::beat(Task::Loop, Phase::TxSend);
            if self.radio.irq_pending() {
                let status = self.radio.irq_status();
                self.radio.clear_irq_status(irq::ALL);
                if status & irq::TX_DONE != 0 {
                    return Ok(());
                }
                if status & irq::TIMEOUT != 0 {
                    return Err(Sx1262Error::Timeout);
                }
            }
            Timer::after(Duration::from_millis(10)).await;
        }
        Err(Sx1262Error::Timeout)
    }

    /// Arm one receive window of `timeout_ms`, with `mask` routed to DIO1.
    ///
    /// The instrument for the chip's own timebase. This timeout is counted
    /// in the same 15.625 us steps off the same RC64k that times a duty
    /// cycle's sleep, so timing it against the host's crystal measures the
    /// oscillator that decides how wide a wake preamble has to be - and it
    /// needs no second board, because the answer does not depend on hearing
    /// anything.
    pub fn arm_rx_timeout(&mut self, timeout_ms: u32, mask: u16) {
        self.rx_active = false;
        self.radio.set_standby(StandbyClk::Rc);
        self.radio.set_lora_packet_params(RX_MAX_PAYLOAD);
        self.radio.set_dio_irq_params(mask);
        self.radio.clear_irq_status(irq::ALL);
        self.radio.set_rx(crate::sx1262::timeout_from_millis(timeout_ms));
    }

    /// Bring the chip back from a retained sleep so the next command lands.
    ///
    /// A duty cycle spends most of its time in sleep with BUSY held high,
    /// where the part accepts nothing and is woken only by a falling edge
    /// on NSS. The first transaction after that is therefore spent waking
    /// it, and whatever it carried is lost - which is why an arm issued
    /// straight after a duty cycle appears to do nothing and the status
    /// afterwards reads as an absent radio.
    ///
    /// Sends a cheap command for its NSS edge alone, gives the part its
    /// documented wake-up time, and then puts it somewhere known.
    pub async fn wake_from_retained_sleep(&mut self) {
        // The status byte is the cheapest transaction there is, and its
        // value is not wanted - only the edge that carries it.
        let _ = self.radio.status();
        Timer::after(Duration::from_millis(2)).await;
        self.radio.set_standby(StandbyClk::Rc);
        let _ = self.radio.status();
    }

    /// Whether the radio is asserting DIO1.
    pub fn irq_pending(&self) -> bool {
        self.radio.irq_pending()
    }

    /// Read the pending interrupt bits and clear exactly those.
    pub fn take_irq(&mut self) -> u16 {
        let status = self.radio.irq_status();
        if status != 0 {
            self.radio.clear_irq_status(status);
        }
        status
    }

    /// Key the PA on a continuous preamble until [`standby`](Self::standby).
    ///
    /// Only useful as an instrument: it gives a duty-cycled receiver
    /// somewhere a signal is always present, so every window that opens
    /// should detect and the receiver's DIO1 reports its own schedule.
    ///
    /// Returns the latched device errors, which the caller is expected to
    /// check before keying again - `XOSC_START` here means DIO3 is not
    /// supplying the antenna switch, and transmitting into an isolated port
    /// is what destroys this module.
    pub fn key_infinite_preamble(&mut self) -> u16 {
        let err = self.radio.device_errors();
        if err == 0 {
            self.rx_active = false;
            self.radio.set_standby(StandbyClk::Rc);
            self.radio.set_tx_infinite_preamble();
        }
        err
    }

    /// Retune the receiver, leaving everything else alone.
    ///
    /// For surveying a band rather than for operating in one: the hop plan
    /// owns the carrier during normal running and will move it back at its
    /// next slot.
    pub fn tune(&mut self, hz: u32) {
        self.radio.set_standby(StandbyClk::Rc);
        self.radio.set_rf_frequency(hz);
        self.hop.carrier_hz = hz;
    }

    /// The carrier the receiver is on.
    pub fn carrier_hz(&self) -> u32 {
        self.cfg.frequency_hz
    }

    /// Boosted receive gain, or the chip's power-saving default.
    ///
    /// Boost is about two decibels of sensitivity, and sensitivity is not
    /// free on a duty cycle: a receiver that hears more also raises more
    /// false preamble detections, and each of those holds a window open
    /// hunting a header that will never arrive.
    pub fn set_rx_boost(&mut self, on: bool) {
        self.radio.set_standby(StandbyClk::Rc);
        self.radio.write_reg(
            reg::RX_GAIN,
            if on { reg::RX_GAIN_BOOSTED } else { reg::RX_GAIN_POWER_SAVING },
        );
    }

    /// Latched operational errors, as the chip reports them.
    pub fn device_errors(&mut self) -> u16 {
        self.radio.device_errors()
    }

    /// Transmit power the radio is configured for, dBm.
    pub fn power_dbm(&self) -> i8 {
        self.cfg.power_dbm
    }

    /// One LoRa symbol at the running configuration, microseconds.
    pub fn symbol_time_us(&self) -> u32 {
        self.cfg.symbol_time_us()
    }

    /// Put the radio into standby (used for the soft-sleep state).
    pub fn standby(&mut self) {
        self.rx_active = false;
        self.radio.set_standby(StandbyClk::Rc);
    }

    /// Put the radio into cold sleep. [`init`](Self::init) must run again
    /// before further use.
    pub fn sleep(&mut self) {
        self.standby();
        self.radio.set_sleep();
    }

    /// Arm continuous receive, restoring the maximum acceptable payload
    /// length first.
    ///
    /// That restore is the whole reason this is a function rather than a
    /// `set_rx` call at each site: a transmit leaves the packet params
    /// carrying the length of the frame it just sent, and re-entering
    /// receive with that still in place caps the receiver at the size of
    /// this node's own last transmission. After a 7-byte no-fix ping that
    /// is shorter than every position frame on the network, so a node that
    /// lost its fix would also go deaf to everyone else's.
    ///
    /// Packet params are configuration, so the radio is put back in standby
    /// to take them - the caller may be re-arming from continuous RX after
    /// dropping an oversize packet.
    fn enter_rx(&mut self) {
        let clk = self.standby_clk();
        self.radio.set_standby(clk);
        self.radio.set_lora_packet_params(RX_MAX_PAYLOAD);
        self.radio.set_rx(RX_CONTINUOUS);
        self.rx_active = true;
    }

    /// Poll for a received packet. `Some((len, rssi_dbm))` when one landed.
    pub fn poll_recv(&mut self, buf: &mut [u8]) -> Option<(usize, i16)> {
        // A transmit-only node never arms the receiver: this is the call
        // that would otherwise enter continuous RX and hold it there.
        if !self.listen {
            return None;
        }

        let now_ms = Instant::now().as_millis();
        self.gate.begin_poll(now_ms);
        self.hop_tick(now_ms);

        if !self.rx_active {
            self.enter_rx();
        }

        // The cheap check first. DIO1 carries exactly the IRQs enabled in
        // `init`, so a quiet channel costs one GPIO read per poll instead
        // of an SPI transaction.
        if !self.radio.irq_pending() {
            return None;
        }

        let status = self.radio.irq_status();
        // A preamble or a header means a frame is on its way: the gate
        // notes it, so a hop or a transmit waits for it, and says when the
        // packet behind it has landed.
        let seen = self.gate.observe(
            now_ms,
            Irq {
                preamble: status & irq::PREAMBLE_DETECTED != 0,
                header_valid: status & irq::HEADER_VALID != 0,
                header_err: status & irq::HEADER_ERR != 0,
                rx_done: status & irq::RX_DONE != 0,
                crc_err: status & irq::CRC_ERR != 0,
            },
        );
        let crc_ok = match seen {
            Seen::Nothing | Seen::Arriving => {
                // Clear what was read and only that - which also covers a
                // TxDone or a timeout left over from a transmit whose clear
                // did not land. Clearing everything would take an RxDone
                // that landed between the read and the clear with it, and
                // the packet behind it would sit unread in the buffer until
                // the next one overwrote it.
                self.radio.clear_irq_status(status);
                return None;
            }
            Seen::Packet { crc_ok } => crc_ok,
        };

        self.last_rx_done_ms = now_ms;

        // The SX126x raises RxDone alongside the CRC error when a packet
        // arrives corrupt, and the payload is still sitting in the buffer.
        // Nothing above this layer checksums, so a corrupt packet handed up
        // would be parsed as a real frame - drop it here.
        self.radio.clear_irq_status(irq::ALL);

        if !crc_ok {
            self.rx_crc_errors = self.rx_crc_errors.saturating_add(1);
            return None;
        }

        let (len_u8, offset) = self.radio.rx_buffer_status();
        let len = len_u8 as usize;

        if len > buf.len() {
            self.rx_oversize = self.rx_oversize.saturating_add(1);
            self.rx_active = false;
            return None;
        }

        self.radio.read_buffer(offset, &mut buf[..len]);

        let (rssi, snr_quarter_db) = self.radio.lora_packet_status();
        self.last_snr_cb = i16::from(snr_quarter_db) * 25;

        // Stay in RX - continuous mode persists.
        Some((len, rssi))
    }

    /// Transmit one packet, returning once the radio reports it sent.
    ///
    /// The packet goes out on the current slot's channel, inside this
    /// node's turn of it for a frame sent every `interval_ms` (0 for a
    /// one-off), and if `sync_at` names where the frame keeps its sync
    /// word the clock's reading at the instant the preamble leaves the
    /// antenna is written there - which is why the buffer is mutable. The
    /// caller is expected to have planned the instant - the beacon planner
    /// or [`repeat_start`](Self::repeat_start) - and the wait here is the
    /// last resort for a clock that moved in between, bounded by one slot.
    pub async fn send(
        &mut self,
        data: &mut [u8],
        sync_at: Option<usize>,
        interval_ms: u32,
    ) -> Result<(), Sx1262Error> {
        self.rx_active = false;
        // Whatever was arriving is lost to the transmit; nothing waits for
        // it afterwards.
        self.gate.clear();

        let airtime_ms = self.cfg.time_on_air_us(data.len()).div_ceil(1000);
        {
            let h = &self.hop;
            let now_ms = Instant::now().as_millis();
            let wait =
                h.clock
                    .wait_for_window_ms(&h.plan, self.address, interval_ms, now_ms, airtime_ms);
            if wait > 0 {
                Timer::after(Duration::from_millis(u64::from(wait))).await;
            }
        }

        let clk = self.standby_clk();
        self.radio.set_standby(clk);
        // The stamp and the record are both for the instant the preamble
        // starts, which is the command instant plus the lead the chip
        // takes to get there.
        let tx_start_ms = Instant::now().as_millis() + u64::from(self.tx_lead_ms());
        {
            let h = &mut self.hop;
            let slot = h.clock.slot(tx_start_ms);
            h.carrier_hz = h.plan.frequency_for_slot(slot);
            self.radio.set_rf_frequency(h.carrier_hz);
            // The receiver comes back up on this channel after TxDone; the
            // next poll moves it if the slot has changed by then.
            h.rx_slot = Some(slot);
            if let Some(off) = sync_at
                && let Some(word) = data.get_mut(off..off + hop::SYNC_LEN)
            {
                word.copy_from_slice(&h.clock.word_at(tx_start_ms).to_bytes());
            }
        }
        self.radio.clear_irq_status(irq::ALL);
        self.radio.write_buffer(0x00, data);

        // Packet params must carry the actual payload length and the full
        // LoRa parameter set, or TxDone never fires. `enter_rx` puts the
        // length back afterwards.
        self.radio.set_lora_packet_params(data.len() as u8);

        self.radio
            .set_tx(crate::sx1262::timeout_from_millis(self.tx_chip_timeout_ms));

        // Wait for TxDone or the chip's own timeout.
        //
        // The deadline reaches 9.7 s at the slowest settings the config
        // accepts (SF12/BW62.5). On the WL this was a spin that had to feed
        // the watchdog to survive; here it awaits, so BLE, GPS and the link
        // all keep running through a slow beacon.
        let start = Instant::now();
        let deadline = Duration::from_millis(self.tx_poll_timeout_ms as u64);
        let result = loop {
            // A transmit is the one place the loop legitimately sits for
            // seconds, so it says so once a millisecond.
            crate::watchdog::beat(Task::Loop, Phase::TxSend);
            if Instant::now() - start > deadline {
                crate::event!(
                    Kind::Radio,
                    "TX timeout (no TxDone after {} ms)",
                    self.tx_poll_timeout_ms
                );
                self.radio.clear_irq_status(irq::ALL);
                break Err(Sx1262Error::Timeout);
            }
            if self.radio.irq_pending() {
                let status = self.radio.irq_status();
                let tx_done = status & irq::TX_DONE != 0;
                let timed_out = status & irq::TIMEOUT != 0;
                if tx_done || timed_out {
                    self.radio.clear_irq_status(irq::ALL);
                    break if tx_done {
                        Ok(())
                    } else {
                        Err(Sx1262Error::Timeout)
                    };
                }
            }
            Timer::after(Duration::from_millis(1)).await;
        };

        self.last_tx = Some((tx_start_ms, Instant::now().as_millis()));

        // Re-enter continuous RX immediately: the node is deaf while it
        // transmits, so every millisecond spent out of RX after TxDone is
        // another chance to miss someone else's transmission. A
        // transmit-only node has nothing to miss and drops back to standby
        // instead.
        if self.listen {
            self.enter_rx();
        } else {
            self.radio.set_standby(StandbyClk::Rc);
        }

        result
    }

    /// Whether a poll's timestamps came too late to place anything.
    pub fn poll_was_late(&self) -> bool {
        self.gate.poll_late()
    }

    pub fn max_packet_len(&self) -> usize {
        255
    }
}
