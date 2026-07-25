//! SX1262 radio driver implementing [`PacketRadio`] via the STM32WLE5
//! SubGHz peripheral.
//!
//! Unlike the compile-time preset scheme of the long-range-radio nodes,
//! modulation parameters here come from a runtime [`RadioConfig`] (SD card
//! TOML or pushed over the ESP link) so [`Sx1262Driver::init`] can be
//! called again to re-configure a live radio.

use midair_proto::radiocfg::RadioConfig;

use crate::platform;

/// A packet-oriented radio interface.
pub trait PacketRadio {
    /// Error type for radio operations.
    type Error: core::fmt::Debug;

    /// Poll for a received packet (non-blocking).
    ///
    /// If a packet is available, write it into `buf` and return
    /// `Ok(Some((bytes_written, rssi_dbm)))`.
    fn poll_recv(&mut self, buf: &mut [u8]) -> Result<Option<(usize, i16)>, Self::Error>;

    /// Transmit a raw packet. Blocks until transmission completes.
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Maximum packet size in bytes.
    fn max_packet_len(&self) -> usize;
}

use stm32wlxx_hal::spi::{SgMiso, SgMosi};
use stm32wlxx_hal::subghz::{
    CalibrateImage, CfgIrq, CodingRate, FallbackMode, HeaderType, Irq, LoRaBandwidth,
    LoRaModParams, LoRaPacketParams, LoRaSyncWord, Ocp, PMode, PaConfig, PaSel, PacketType,
    RampTime, RegMode, RfFreq, SleepCfg, SpreadingFactor, StandbyClk, SubGhz, TcxoMode, TcxoTrim,
    Timeout, TxParams,
};

/// Errors from the SubGHz radio.
#[derive(Debug)]
pub enum Sx1262Error {
    Radio,
    Timeout,
}

/// SetRx timeout value that selects continuous RX. On the SX126x the SetRx
/// timeout doubles as a mode select: 0x000000 is single mode - the receiver
/// stays on only until it decodes one packet, then drops to the fallback
/// mode - while 0xFFFFFF keeps it in RX across packets. A node arms RX once
/// and expects to keep hearing the network, so it must be the latter; single
/// mode would leave a node that rarely transmits deaf after its first packet.
const RX_CONTINUOUS: Timeout = Timeout::from_raw(0x00FF_FFFF);

/// SubGHz radio driver that implements [`PacketRadio`].
///
/// On the STM32WLE5 the SX1262 is integrated - the [`SubGhz`] peripheral
/// handles the internal SPI3 interface, BUSY signal, and DIO lines.
pub struct Sx1262Driver {
    radio: SubGhz<SgMiso, SgMosi>,
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

impl Sx1262Driver {
    /// Create a new SubGHz radio driver. Call [`init`](Self::init) before use.
    pub fn new(radio: SubGhz<SgMiso, SgMosi>) -> Self {
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

    /// Packets dropped since boot for a bad CRC and for overrunning the
    /// caller's buffer. Both saturate; neither is cleared except by reboot.
    pub fn rx_crc_errors(&self) -> u32 {
        self.rx_crc_errors
    }

    /// Packets dropped since boot for exceeding the receive buffer.
    pub fn rx_oversize(&self) -> u32 {
        self.rx_oversize
    }

    /// Initialize (or re-initialize) the radio from `cfg`.
    ///
    /// # Panics
    ///
    /// Panics if the radio fails to respond during initialization.
    pub fn init(&mut self, cfg: &RadioConfig) {
        debug_println!("Initialising SubGHz radio...");
        self.rx_active = false;
        self.listen = cfg.role.receives();
        self.tx_poll_timeout_ms = cfg.tx_poll_timeout_ms();
        self.tx_chip_timeout_ms = cfg.tx_chip_timeout_ms();

        self.radio.set_standby(StandbyClk::Rc).expect("set_standby");

        // DC-DC roughly halves RX/TX current, but only works on a board
        // with the SMPS inductor fitted, so it stays configurable.
        self.radio
            .set_regulator_mode(if cfg.dcdc_enabled {
                RegMode::Smps
            } else {
                RegMode::Ldo
            })
            .ok();

        // The radio powers the 32 MHz TCXO itself and waits for it to
        // settle before using the clock - the same job the SX1262 does with
        // SetDio3AsTcxoCtrl, which this chip exposes as SetTcxoMode because
        // the radio is on-die and there is no external DIO3 to name.
        let trim = match cfg.tcxo_volts.trim() {
            0x0 => TcxoTrim::Volts1pt6,
            0x1 => TcxoTrim::Volts1pt7,
            0x2 => TcxoTrim::Volts1pt8,
            0x3 => TcxoTrim::Volts2pt2,
            0x4 => TcxoTrim::Volts2pt4,
            0x5 => TcxoTrim::Volts2pt7,
            0x6 => TcxoTrim::Volts3pt0,
            _ => TcxoTrim::Volts3pt3,
        };
        self.radio
            .set_tcxo_mode(
                &TcxoMode::new()
                    .set_txco_trim(trim)
                    .set_timeout(Timeout::from_millis_sat(cfg.tcxo_startup_ms as u32)),
            )
            .expect("set_tcxo_mode");

        let band = if cfg.frequency_hz >= 900_000_000 {
            CalibrateImage::ISM_902_928
        } else if cfg.frequency_hz >= 860_000_000 {
            CalibrateImage::ISM_863_870
        } else {
            CalibrateImage::ISM_430_440
        };
        self.radio.calibrate_image(band).expect("calibrate_image");

        self.radio
            .set_packet_type(PacketType::LoRa)
            .expect("set_packet_type");

        self.radio
            .set_rf_frequency(&RfFreq::from_frequency(cfg.frequency_hz))
            .expect("set_rf_frequency");

        // High-power PA, duty/hp_max per the +22 dBm datasheet preset;
        // the actual output level is set via TxParams below.
        self.radio
            .set_pa_config(
                &PaConfig::new()
                    .set_pa_duty_cycle(0x04)
                    .set_hp_max(0x07)
                    .set_pa(PaSel::Hp),
            )
            .expect("set_pa_config");

        self.radio
            .set_tx_params(
                &TxParams::new()
                    .set_power(cfg.power_dbm as u8)
                    .set_ramp_time(RampTime::Micros200),
            )
            .expect("set_tx_params");

        // Receive-side counterpart to the TX power above. The RxGain
        // register is not covered by warm-start retention, so it has to be
        // rewritten on every entry to this function rather than set once -
        // which is what happens anyway, since `init` is what brings the
        // radio back from both soft sleep and a config push.
        self.radio
            .set_rx_gain(if cfg.rx_boost {
                PMode::Boost
            } else {
                PMode::PowerSaving
            })
            .expect("set_rx_gain");

        let sf = match cfg.spreading_factor {
            5 => SpreadingFactor::Sf5,
            6 => SpreadingFactor::Sf6,
            7 => SpreadingFactor::Sf7,
            8 => SpreadingFactor::Sf8,
            9 => SpreadingFactor::Sf9,
            10 => SpreadingFactor::Sf10,
            11 => SpreadingFactor::Sf11,
            _ => SpreadingFactor::Sf12,
        };
        let bw = match cfg.bandwidth_khz {
            62 => LoRaBandwidth::Bw62,
            250 => LoRaBandwidth::Bw250,
            500 => LoRaBandwidth::Bw500,
            _ => LoRaBandwidth::Bw125,
        };
        let cr = match cfg.coding_rate {
            6 => CodingRate::Cr46,
            7 => CodingRate::Cr47,
            8 => CodingRate::Cr48,
            _ => CodingRate::Cr45,
        };
        self.radio
            .set_lora_mod_params(
                &LoRaModParams::new()
                    .set_sf(sf)
                    .set_bw(bw)
                    .set_cr(cr)
                    .set_ldro_en(cfg.ldro()),
            )
            .expect("set_lora_mod_params");

        self.radio
            .set_lora_packet_params(
                &LoRaPacketParams::new()
                    .set_preamble_len(8)
                    .set_header_type(HeaderType::Variable)
                    .set_payload_len(255)
                    .set_crc_en(true)
                    .set_invert_iq(false),
            )
            .expect("set_lora_packet_params");

        self.radio
            .set_lora_sync_word(LoRaSyncWord::Public)
            .expect("set_lora_sync_word");

        self.radio
            .set_buffer_base_address(0x00, 0x00)
            .expect("set_buffer_base_address");

        self.radio
            .set_irq_cfg(
                &CfgIrq::new()
                    .irq_enable_all(Irq::RxDone)
                    .irq_enable_all(Irq::TxDone)
                    .irq_enable_all(Irq::Err)
                    .irq_enable_all(Irq::Timeout),
            )
            .expect("set_irq_cfg");

        self.radio
            .set_tx_rx_fallback_mode(FallbackMode::Standby)
            .ok();

        // Over-current protection: required for the HP PA to reach +22.
        self.radio.set_pa_ocp(Ocp::Max140m).ok();
        debug_println!(
            "SubGHz init: {} Hz SF{} BW{} CR4/{} {} dBm",
            cfg.frequency_hz,
            cfg.spreading_factor,
            cfg.bandwidth_khz,
            cfg.coding_rate,
            cfg.power_dbm
        );
    }

    /// Print radio diagnostics. Returns `true` if the radio responds.
    pub fn print_diagnostics(&mut self) -> bool {
        match self.radio.status() {
            Ok(s) => {
                debug_println!("Radio status: {:?}", s);
                true
            }
            Err(_) => {
                rtt_target::rprintln!("WARNING: Radio not responding!");
                false
            }
        }
    }

    /// SNR of the last received packet in centibels (dB * 100).
    pub fn last_snr_cb(&self) -> i16 {
        self.last_snr_cb
    }

    /// Put the radio into standby (used for the soft-sleep state).
    pub fn standby(&mut self) {
        self.rx_active = false;
        self.radio.set_standby(StandbyClk::Rc).ok();
        self.wait_on_busy();
    }

    /// Put the radio into cold sleep. [`init`](Self::init) must run again
    /// before further use.
    pub fn sleep(&mut self) {
        self.standby();
        unsafe {
            self.radio.set_sleep(SleepCfg::default()).ok();
        }
    }

    /// Poll the RFBUSYS bit to wait for the radio to be ready.
    ///
    /// The SX126x silently ignores SPI commands sent while BUSY is high, so
    /// this must be called after every set_standby/set_tx/set_rx before the
    /// next command or IRQ poll.
    fn wait_on_busy(&self) {
        while unsafe {
            (*stm32wlxx_hal::pac::PWR::ptr())
                .sr2
                .read()
                .rfbusys()
                .bit_is_set()
        } {}
    }
}

impl PacketRadio for Sx1262Driver {
    type Error = Sx1262Error;

    fn poll_recv(&mut self, buf: &mut [u8]) -> Result<Option<(usize, i16)>, Self::Error> {
        // A transmit-only node never arms the receiver: this is the call
        // that would otherwise enter continuous RX and hold it there.
        if !self.listen {
            return Ok(None);
        }

        // Enter continuous RX if not already listening.
        if !self.rx_active {
            self.radio
                .set_rx(RX_CONTINUOUS)
                .map_err(|_| Sx1262Error::Radio)?;
            self.wait_on_busy();
            self.rx_active = true;
        }

        let (_, irq) = self.radio.irq_status().map_err(|_| Sx1262Error::Radio)?;

        if irq & Irq::RxDone.mask() == 0 {
            return Ok(None);
        }

        // The SX126x raises RxDone alongside Err when a packet arrives with
        // a bad CRC, and the payload is still sitting in the buffer. Nothing
        // above this layer checksums, so a corrupt packet handed up would be
        // parsed as a real frame - drop it here.
        let crc_bad = irq & Irq::Err.mask() != 0;

        let _ = self.radio.clear_irq_status(0xFFFF);

        if crc_bad {
            self.rx_crc_errors = self.rx_crc_errors.saturating_add(1);
            debug_println!("Dropped packet with bad CRC");
            return Ok(None);
        }

        let (_, len_u8, offset) = self
            .radio
            .rx_buffer_status()
            .map_err(|_| Sx1262Error::Radio)?;
        let len = len_u8 as usize;

        if len > buf.len() {
            self.rx_oversize = self.rx_oversize.saturating_add(1);
            self.rx_active = false;
            return Ok(None);
        }

        self.radio
            .read_buffer(offset, &mut buf[..len])
            .map_err(|_| Sx1262Error::Radio)?;

        let pkt_status = self
            .radio
            .lora_packet_status()
            .map_err(|_| Sx1262Error::Radio)?;
        let rssi = pkt_status.rssi_pkt().to_integer();
        // snr_pkt() is Ratio<i16> with denominator 4 (quarter dB).
        self.last_snr_cb = *pkt_status.snr_pkt().numer() * 25;

        crate::leds::note_rx();

        // Stay in RX - continuous mode persists.
        Ok(Some((len, rssi)))
    }

    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.rx_active = false;

        self.radio
            .set_standby(StandbyClk::Rc)
            .map_err(|_| Sx1262Error::Radio)?;
        self.wait_on_busy();

        let _ = self.radio.clear_irq_status(0xFFFF);

        self.radio
            .write_buffer(0x00, data)
            .map_err(|_| Sx1262Error::Radio)?;

        // Packet params must carry the actual payload length and the full
        // LoRa parameter set, or TxDone never fires.
        self.radio
            .set_lora_packet_params(
                &LoRaPacketParams::new()
                    .set_preamble_len(8)
                    .set_header_type(HeaderType::Variable)
                    .set_payload_len(data.len() as u8)
                    .set_crc_en(true)
                    .set_invert_iq(false),
            )
            .map_err(|_| Sx1262Error::Radio)?;

        self.radio
            .set_tx(Timeout::from_millis_sat(self.tx_chip_timeout_ms))
            .map_err(|_| Sx1262Error::Radio)?;
        self.wait_on_busy();

        crate::leds::note_tx();

        // Poll IRQ for TxDone/Timeout.
        let start_ms = platform::millis();
        let result = loop {
            let elapsed = platform::millis().wrapping_sub(start_ms);
            if elapsed > self.tx_poll_timeout_ms {
                debug_println!("TX timeout (no TxDone after {} ms)", self.tx_poll_timeout_ms);
                let _ = self.radio.clear_irq_status(0xFFFF);
                break Err(Sx1262Error::Timeout);
            }
            if let Ok((_, irq)) = self.radio.irq_status() {
                let tx_done = irq & Irq::TxDone.mask() != 0;
                let timeout = irq & Irq::Timeout.mask() != 0;
                if tx_done || timeout {
                    let _ = self.radio.clear_irq_status(0xFFFF);
                    break if tx_done {
                        Ok(())
                    } else {
                        Err(Sx1262Error::Timeout)
                    };
                }
            }
        };

        // Re-enter continuous RX immediately: the node is deaf while it
        // transmits, so every millisecond spent out of RX after TxDone is
        // another chance to miss someone else's transmission. A transmit-only
        // node has nothing to miss and drops back to standby instead.
        if self.listen {
            if self.radio.set_rx(RX_CONTINUOUS).is_ok() {
                self.wait_on_busy();
                self.rx_active = true;
            }
        } else {
            self.radio.set_standby(StandbyClk::Rc).ok();
            self.wait_on_busy();
        }

        result
    }

    fn max_packet_len(&self) -> usize {
        255
    }
}
