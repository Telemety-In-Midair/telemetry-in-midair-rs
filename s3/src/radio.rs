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

use embassy_time::{Duration, Instant, Timer};
use esp_println::println;
use midair_proto::radiocfg::RadioConfig;

use crate::sx1262::{irq, reg, FallbackMode, StandbyClk, Sx1262, RX_CONTINUOUS};

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

pub struct Sx1262Driver<'d> {
    radio: Sx1262<'d>,
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
        Self {
            radio,
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
        self.tx_poll_timeout_ms = cfg.tx_poll_timeout_ms();
        self.tx_chip_timeout_ms = cfg.tx_chip_timeout_ms();

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

        // The radio drives this board's antenna switch itself: DIO2 is the
        // SKY13453-385LF's VCTL. Unconditional, and deliberately not read
        // from the config - a config that said false would put the PA into
        // an isolated port on every transmission, and no setting a user can
        // reach should be able to ask for that.
        if !cfg.dio2_rf_switch {
            println!("config: dio2_rf_switch=false ignored, this board needs it on");
        }
        self.radio.set_dio2_as_rf_switch(true);

        // DIO3 supplies the 32 MHz TCXO *and* that same antenna switch's
        // VDD, so it is floored at the switch's 2.5 V minimum rather than
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
        self.radio.set_rf_frequency(cfg.frequency_hz);

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
        self.radio
            .set_dio_irq_params(irq::RX_DONE | irq::TX_DONE | irq::CRC_ERR | irq::TIMEOUT);
        self.radio.set_rx_tx_fallback_mode(FallbackMode::StandbyRc);

        // Over-current protection: required for the HP PA to reach +22.
        self.radio.write_reg(reg::OCP, reg::OCP_140MA);

        println!(
            "radio init: {} Hz SF{} BW{} CR4/{} {} dBm",
            cfg.frequency_hz,
            cfg.spreading_factor,
            cfg.bandwidth_khz,
            cfg.coding_rate,
            cfg.power_dbm
        );
    }

    /// The radio's current mode and any latched operational error.
    ///
    /// Packet counters cannot show a radio that is sitting somewhere it
    /// should not be. A node beaconing every 20 s spends almost all of its
    /// time in `rx`, so anything else on a periodic status line - `tx` in
    /// particular - is a radio burning current between transmissions rather
    /// than listening.
    pub fn health(&mut self) -> (&'static str, u16) {
        let mode = match (self.radio.status() >> 4) & 0x07 {
            0x02 => "standby",
            0x03 => "standby-xosc",
            0x04 => "fs",
            0x05 => "rx",
            0x06 => "tx",
            _ => "?",
        };
        (mode, self.radio.device_errors())
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
        self.radio.set_standby(StandbyClk::Rc);
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
        if status & irq::RX_DONE == 0 {
            return None;
        }

        // The SX126x raises RxDone alongside the CRC error when a packet
        // arrives corrupt, and the payload is still sitting in the buffer.
        // Nothing above this layer checksums, so a corrupt packet handed up
        // would be parsed as a real frame - drop it here.
        let crc_bad = status & irq::CRC_ERR != 0;
        self.radio.clear_irq_status(irq::ALL);

        if crc_bad {
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
    pub async fn send(&mut self, data: &[u8]) -> Result<(), Sx1262Error> {
        self.rx_active = false;

        self.radio.set_standby(StandbyClk::Rc);
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
            if Instant::now() - start > deadline {
                println!("TX timeout (no TxDone after {} ms)", self.tx_poll_timeout_ms);
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

    pub fn max_packet_len(&self) -> usize {
        255
    }
}
