//! SX1262 command layer over SPI.
//!
//! The STM32WLE5 this firmware is ported from carries the same radio die
//! on an internal SPI, so `stm32wlxx-hal`'s `subghz` module and this file
//! speak the same command set - only the transport differs. That is what
//! makes [`crate::radio`] a near-verbatim port: the typed HAL structs it
//! used compile down to these byte sequences.
//!
//! Two things the WL could not do turn up here. DIO1 is a real pin, so a
//! receive poll can check a GPIO before paying for an SPI round trip; and
//! DIO2 can drive the antenna switch, which on the WL was an MCU GPIO job
//! (see `dio2_rf_switch` in the radio config).
//!
//! Every command must be sent with BUSY low - the chip silently ignores
//! anything shifted in while it is high - which is what [`Sx1262::xfer`]
//! guarantees for all of them.

use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::{Input, Output};
use esp_hal::spi::master::Spi;
use esp_hal::Blocking;

/// Commands, from the SX1261/2 datasheet's opcode table.
mod op {
    pub const SET_SLEEP: u8 = 0x84;
    pub const SET_STANDBY: u8 = 0x80;
    pub const SET_TX: u8 = 0x83;
    pub const SET_RX: u8 = 0x82;
    pub const SET_RX_DUTY_CYCLE: u8 = 0x94;
    pub const SET_LORA_SYMB_NUM_TIMEOUT: u8 = 0xA0;
    pub const SET_TX_INFINITE_PREAMBLE: u8 = 0xD1;
    pub const SET_RX_TX_FALLBACK_MODE: u8 = 0x93;
    pub const SET_REGULATOR_MODE: u8 = 0x96;
    pub const CALIBRATE: u8 = 0x89;
    pub const CALIBRATE_IMAGE: u8 = 0x98;
    pub const SET_PA_CONFIG: u8 = 0x95;
    pub const WRITE_REGISTER: u8 = 0x0D;
    pub const READ_REGISTER: u8 = 0x1D;
    pub const WRITE_BUFFER: u8 = 0x0E;
    pub const READ_BUFFER: u8 = 0x1E;
    pub const SET_DIO_IRQ_PARAMS: u8 = 0x08;
    pub const GET_IRQ_STATUS: u8 = 0x12;
    pub const CLEAR_IRQ_STATUS: u8 = 0x02;
    pub const SET_DIO2_AS_RF_SWITCH_CTRL: u8 = 0x9D;
    pub const SET_DIO3_AS_TCXO_CTRL: u8 = 0x97;
    pub const SET_RF_FREQUENCY: u8 = 0x86;
    pub const SET_PACKET_TYPE: u8 = 0x8A;
    pub const SET_TX_PARAMS: u8 = 0x8E;
    pub const SET_MODULATION_PARAMS: u8 = 0x8B;
    pub const SET_PACKET_PARAMS: u8 = 0x8C;
    pub const SET_BUFFER_BASE_ADDRESS: u8 = 0x8F;
    pub const GET_STATUS: u8 = 0xC0;
    pub const GET_RX_BUFFER_STATUS: u8 = 0x13;
    pub const GET_PACKET_STATUS: u8 = 0x14;
    pub const GET_DEVICE_ERRORS: u8 = 0x17;
    pub const CLEAR_DEVICE_ERRORS: u8 = 0x07;
}

/// Configuration registers this driver touches by address, because they
/// have no command of their own.
pub mod reg {
    /// SMPS control 0. Bit 6 enables clock detection, which has to be on
    /// before the SMPS is selected; every other bit belongs to the
    /// regulator and must be preserved.
    pub const SMPS_C0: u16 = 0x0916;
    /// [`SMPS_C0`] clock-detection enable.
    pub const SMPS_CLK_DET_EN: u8 = 1 << 6;

    /// TX clamp configuration. Bits 4:1 all set improves the PA's tolerance
    /// of an antenna mismatch (datasheet, "Better resistance of the SX1262
    /// Tx to antenna mismatch").
    pub const TX_CLAMP: u16 = 0x08D8;

    /// TX modulation configuration. Bit 2 must be cleared for a 500 kHz
    /// LoRa bandwidth and set for every other bandwidth (datasheet,
    /// "Modulation quality with 500 kHz LoRa bandwidth").
    pub const TX_MODULATION: u16 = 0x0889;

    /// Receiver gain. Not covered by warm-start retention, so it has to be
    /// rewritten on every init.
    pub const RX_GAIN: u16 = 0x08AC;
    /// Boosted gain: roughly +2 dB of sensitivity for a few mA.
    ///
    /// The WL firmware wrote 0x97 here, by way of its HAL's `PMode::Boost`.
    /// The datasheet documents only these two values, so the port writes
    /// what is documented.
    pub const RX_GAIN_BOOSTED: u8 = 0x96;
    /// Power-saving gain, the chip's power-up value.
    pub const RX_GAIN_POWER_SAVING: u8 = 0x94;

    /// Over-current protection, in 2.5 mA steps.
    pub const OCP: u16 = 0x08E7;
    /// 140 mA, required for the HP PA to reach +22 dBm.
    pub const OCP_140MA: u8 = 0x38;

    /// LoRa sync word, high then low byte.
    pub const LORA_SYNC_WORD_MSB: u16 = 0x0740;
    pub const LORA_SYNC_WORD_LSB: u16 = 0x0741;
    /// Private network (0x1424), not the public LoRaWAN word.
    pub const SYNC_WORD_PRIVATE: (u8, u8) = (0x14, 0x24);
}

/// Latched operational errors, as returned by `GetDeviceErrors`.
///
/// These are the only report of a radio that reached a healthy-looking mode
/// without working: a TCXO that never started, a calibration or PLL lock
/// that failed, a PA that would not ramp.
pub mod dev_err {
    pub const RC64K_CALIB: u16 = 1 << 0;
    pub const RC13M_CALIB: u16 = 1 << 1;
    pub const PLL_CALIB: u16 = 1 << 2;
    pub const ADC_CALIB: u16 = 1 << 3;
    pub const IMG_CALIB: u16 = 1 << 4;
    /// The 32 MHz oscillator did not start. On this board that means DIO3
    /// is not supplying the TCXO - which is also the antenna switch's VDD,
    /// so it is a transmit-into-an-isolated-port condition, not a range one.
    pub const XOSC_START: u16 = 1 << 5;
    pub const PLL_LOCK: u16 = 1 << 6;
    pub const PA_RAMP: u16 = 1 << 8;
}

/// Chip modes, as they appear in bits 6:4 of the status byte.
pub mod mode {
    pub const STDBY_RC: u8 = 0x02;
    pub const STDBY_XOSC: u8 = 0x03;
    pub const FS: u8 = 0x04;
    pub const RX: u8 = 0x05;
    pub const TX: u8 = 0x06;
}

/// IRQ bits, as returned by `GetIrqStatus`.
pub mod irq {
    pub const TX_DONE: u16 = 1 << 0;
    pub const RX_DONE: u16 = 1 << 1;
    /// A LoRa preamble has been detected: a frame may be arriving.
    pub const PREAMBLE_DETECTED: u16 = 1 << 2;
    /// The explicit header decoded, so a payload is on its way.
    pub const HEADER_VALID: u16 = 1 << 4;
    /// The header failed its CRC; the radio has given up on this frame.
    pub const HEADER_ERR: u16 = 1 << 5;
    pub const CRC_ERR: u16 = 1 << 6;
    pub const TIMEOUT: u16 = 1 << 9;
    /// Every bit, for a blanket clear.
    pub const ALL: u16 = 0xFFFF;
}

/// `SetStandby` argument.
#[derive(Clone, Copy)]
pub enum StandbyClk {
    /// 13 MHz RC oscillator. Cheaper, and what configuration runs on.
    Rc = 0,
    /// The 32 MHz crystal or TCXO.
    Xosc = 1,
}

/// `SetRxTxFallbackMode` argument: where the chip lands after a packet.
#[derive(Clone, Copy)]
pub enum FallbackMode {
    Fs = 0x40,
    StandbyXosc = 0x30,
    StandbyRc = 0x20,
}

/// How long [`Sx1262::wait_on_busy`] waits before deciding the radio is not
/// going to answer.
///
/// Comfortably past the longest BUSY the chip produces: a full `Calibrate`
/// of every block takes about 3.5 ms, and startup after NRESET is similar.
const BUSY_TIMEOUT_MS: u64 = 50;

/// Timeout value that selects continuous RX.
///
/// On the SX126x the `SetRx` timeout doubles as a mode select: 0x000000 is
/// single mode - the receiver stays on only until it decodes one packet,
/// then drops to the fallback mode - while 0xFFFFFF keeps it in RX across
/// packets. A node arms RX once and expects to keep hearing the network,
/// so it must be the latter; single mode would leave a node that rarely
/// transmits deaf after its first packet.
pub const RX_CONTINUOUS: u32 = 0x00FF_FFFF;

/// Preamble every frame on the network is sent with, in symbols.
///
/// Mirrors the protocol crate's `PREAMBLE_SYMBOLS`, which is what the
/// airtime arithmetic uses; the two have to agree or every computed time on
/// air is wrong by the difference.
pub const NETWORK_PREAMBLE_SYMBOLS: u16 = 8;

/// Convert milliseconds to the chip's 15.625 us timeout step, saturating
/// at the 24-bit field.
pub fn timeout_from_millis(ms: u32) -> u32 {
    ms.saturating_mul(64).min(0x00FF_FFFF)
}

/// The radio, and the four lines that are not the SPI bus.
pub struct Sx1262<'d> {
    spi: Spi<'d, Blocking>,
    nss: Output<'d>,
    busy: Input<'d>,
    dio1: Input<'d>,
    nrst: Output<'d>,
}

impl<'d> Sx1262<'d> {
    pub fn new(
        spi: Spi<'d, Blocking>,
        nss: Output<'d>,
        busy: Input<'d>,
        dio1: Input<'d>,
        nrst: Output<'d>,
    ) -> Self {
        Self {
            spi,
            nss,
            busy,
            dio1,
            nrst,
        }
    }

    /// Pulse NRST and wait for the chip to come back.
    ///
    /// The WL had no equivalent - its radio reset with the MCU. Here a
    /// wedged radio can be recovered without power-cycling the board, and
    /// a cold start needs this before the first command lands.
    pub async fn reset(&mut self) {
        self.nrst.set_low();
        Timer::after(Duration::from_millis(2)).await;
        self.nrst.set_high();
        Timer::after(Duration::from_millis(5)).await;
        self.wait_on_busy();
    }

    /// Whether the radio is asserting DIO1, i.e. one of the enabled IRQs
    /// is pending.
    ///
    /// A receive poll checks this before spending an SPI round trip on
    /// `GetIrqStatus`, which is most of what an idle listening node does.
    pub fn irq_pending(&self) -> bool {
        self.dio1.is_high()
    }

    /// Spin until the radio releases BUSY, or [`BUSY_TIMEOUT_MS`] passes.
    ///
    /// The SX126x silently ignores commands sent while BUSY is high, so
    /// this runs before every transaction.
    ///
    /// The timeout is what makes a dead radio diagnosable. BUSY is an input
    /// with no pull, so a radio that is absent, held in reset or wired to
    /// the wrong pad floats this line - and an unbounded spin here hangs
    /// the firmware inside `init`, before the diagnostic that would have
    /// named the problem ever runs. Giving up and letting the transaction
    /// proceed produces a garbage status byte instead, which
    /// [`crate::radio::Sx1262Driver::print_diagnostics`] reports as a radio
    /// that is not responding.
    fn wait_on_busy(&self) {
        let deadline = Instant::now() + Duration::from_millis(BUSY_TIMEOUT_MS);
        while self.busy.is_high() {
            if Instant::now() > deadline {
                return;
            }
        }
    }

    /// One SPI transaction with NSS held across it, replacing `buf` with
    /// what the radio shifted back.
    fn xfer(&mut self, buf: &mut [u8]) {
        self.wait_on_busy();
        self.nss.set_low();
        let _ = self.spi.transfer(buf);
        self.nss.set_high();
    }

    /// A command with arguments and no reply.
    fn cmd(&mut self, opcode: u8, args: &[u8]) {
        self.wait_on_busy();
        self.nss.set_low();
        let _ = self.spi.write(&[opcode]);
        if !args.is_empty() {
            let _ = self.spi.write(args);
        }
        self.nss.set_high();
    }

    /// A command that returns `out.len()` bytes after the status byte.
    fn cmd_read(&mut self, opcode: u8, out: &mut [u8]) {
        self.wait_on_busy();
        self.nss.set_low();
        // Opcode, then one NOP during which the chip returns its status,
        // then the payload.
        let _ = self.spi.write(&[opcode, 0x00]);
        let _ = self.spi.read(out);
        self.nss.set_high();
    }

    pub fn read_reg(&mut self, addr: u16) -> u8 {
        let mut buf = [
            op::READ_REGISTER,
            (addr >> 8) as u8,
            addr as u8,
            0x00,
            0x00,
        ];
        self.xfer(&mut buf);
        buf[4]
    }

    pub fn write_reg(&mut self, addr: u16, value: u8) {
        self.cmd(
            op::WRITE_REGISTER,
            &[(addr >> 8) as u8, addr as u8, value],
        );
    }

    /// Read-modify-write, for the registers where the other bits matter.
    pub fn modify_reg(&mut self, addr: u16, f: impl FnOnce(u8) -> u8) {
        let v = self.read_reg(addr);
        self.write_reg(addr, f(v));
    }

    pub fn write_buffer(&mut self, offset: u8, data: &[u8]) {
        self.wait_on_busy();
        self.nss.set_low();
        let _ = self.spi.write(&[op::WRITE_BUFFER, offset]);
        let _ = self.spi.write(data);
        self.nss.set_high();
    }

    pub fn read_buffer(&mut self, offset: u8, out: &mut [u8]) {
        self.wait_on_busy();
        self.nss.set_low();
        let _ = self.spi.write(&[op::READ_BUFFER, offset, 0x00]);
        let _ = self.spi.read(out);
        self.nss.set_high();
    }

    pub fn set_standby(&mut self, clk: StandbyClk) {
        self.cmd(op::SET_STANDBY, &[clk as u8]);
    }

    /// Cold sleep. Configuration is lost; a full init has to run again.
    pub fn set_sleep(&mut self) {
        // Cold start, no RTC wake: everything this driver sets is rewritten
        // by `init` anyway, and warm start would keep the radio's retention
        // regulator alive for nothing.
        self.cmd(op::SET_SLEEP, &[0x00]);
    }

    pub fn set_regulator_mode(&mut self, smps: bool) {
        self.cmd(op::SET_REGULATOR_MODE, &[u8::from(smps)]);
    }

    /// `SetDIO3AsTCXOCtrl`: the radio powers the TCXO and waits for it to
    /// settle before using the clock.
    pub fn set_tcxo_ctrl(&mut self, trim: u8, startup_ms: u32) {
        let t = timeout_from_millis(startup_ms);
        self.cmd(
            op::SET_DIO3_AS_TCXO_CTRL,
            &[trim & 0x07, (t >> 16) as u8, (t >> 8) as u8, t as u8],
        );
    }

    /// `SetDIO2AsRfSwitchCtrl`: let the radio drive an external antenna
    /// switch from DIO2 as it changes between TX and RX.
    pub fn set_dio2_as_rf_switch(&mut self, enable: bool) {
        self.cmd(op::SET_DIO2_AS_RF_SWITCH_CTRL, &[u8::from(enable)]);
    }

    pub fn calibrate(&mut self, blocks: u8) {
        self.cmd(op::CALIBRATE, &[blocks]);
    }

    pub fn calibrate_image(&mut self, f1: u8, f2: u8) {
        self.cmd(op::CALIBRATE_IMAGE, &[f1, f2]);
    }

    pub fn set_packet_type_lora(&mut self) {
        self.cmd(op::SET_PACKET_TYPE, &[0x01]);
    }

    pub fn set_rf_frequency(&mut self, hz: u32) {
        // freq_reg = hz * 2^25 / 32 MHz, in u64 so the shift cannot wrap.
        let raw = (((hz as u64) << 25) / 32_000_000) as u32;
        self.cmd(op::SET_RF_FREQUENCY, &raw.to_be_bytes());
    }

    /// High-power PA, `duty`/`hp_max` per the datasheet's +22 dBm preset.
    /// The output level itself comes from [`Self::set_tx_params`].
    pub fn set_pa_config(&mut self, duty: u8, hp_max: u8) {
        // device_sel 0 = SX1262, pa_lut is always 0x01.
        self.cmd(op::SET_PA_CONFIG, &[duty, hp_max, 0x00, 0x01]);
    }

    pub fn set_tx_params(&mut self, power_dbm: i8, ramp_time: u8) {
        self.cmd(op::SET_TX_PARAMS, &[power_dbm as u8, ramp_time]);
    }

    pub fn set_lora_mod_params(&mut self, sf: u8, bw: u8, cr: u8, ldro: bool) {
        self.cmd(op::SET_MODULATION_PARAMS, &[sf, bw, cr, u8::from(ldro)]);
    }

    /// The LoRa packet params this firmware always uses, for a payload of
    /// `payload_len` bytes: an explicit header, the hardware CRC on, and no
    /// IQ inversion, behind the network's ordinary preamble.
    ///
    /// `RadioConfig::time_on_air_us` computes air time from these same
    /// fixed choices, so the two have to agree.
    pub fn set_lora_packet_params(&mut self, payload_len: u8) {
        self.set_lora_packet_params_preamble(payload_len, NETWORK_PREAMBLE_SYMBOLS);
    }

    /// The same, with the preamble length given.
    ///
    /// Only a transmission meant for a receiver that is asleep most of the
    /// time needs this: its preamble has to span that receiver's whole
    /// cycle, so it runs to thousands of symbols where everything else on
    /// the network uses eight.
    pub fn set_lora_packet_params_preamble(&mut self, payload_len: u8, preamble_syms: u16) {
        self.cmd(
            op::SET_PACKET_PARAMS,
            &[
                (preamble_syms >> 8) as u8,
                preamble_syms as u8,
                0x00, // variable length, i.e. explicit header
                payload_len,
                0x01, // CRC on
                0x00, // no IQ inversion
            ],
        );
    }

    pub fn set_buffer_base_address(&mut self, tx: u8, rx: u8) {
        self.cmd(op::SET_BUFFER_BASE_ADDRESS, &[tx, rx]);
    }

    /// Enable `mask` and route the same bits to DIO1, so [`Self::irq_pending`]
    /// answers for them.
    pub fn set_dio_irq_params(&mut self, mask: u16) {
        let m = mask.to_be_bytes();
        self.cmd(
            op::SET_DIO_IRQ_PARAMS,
            &[m[0], m[1], m[0], m[1], 0, 0, 0, 0],
        );
    }

    pub fn set_rx_tx_fallback_mode(&mut self, mode: FallbackMode) {
        self.cmd(op::SET_RX_TX_FALLBACK_MODE, &[mode as u8]);
    }

    pub fn set_rx(&mut self, timeout: u32) {
        self.cmd(
            op::SET_RX,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
        );
    }

    /// Cycle receive and sleep on the chip's own timer, both periods in
    /// the same 15.625 us step `SetRx` counts in.
    ///
    /// The one command on this part that runs without the host: the chip
    /// listens for `rx_steps`, sleeps with its configuration retained for
    /// `sleep_steps`, and repeats until something takes it out - so the MCU
    /// can be in deep sleep for whole cycles at a time and still be woken by
    /// a frame. The chip leaves the cycle by itself on a reception, landing
    /// in the fallback mode with the packet in its buffer.
    ///
    /// A transmission is only heard if its preamble spans a whole cycle, so
    /// the two periods and the preamble of whatever is meant to be heard are
    /// one choice and not three.
    pub fn set_rx_duty_cycle(&mut self, rx_steps: u32, sleep_steps: u32) {
        self.cmd(
            op::SET_RX_DUTY_CYCLE,
            &[
                (rx_steps >> 16) as u8,
                (rx_steps >> 8) as u8,
                rx_steps as u8,
                (sleep_steps >> 16) as u8,
                (sleep_steps >> 8) as u8,
                sleep_steps as u8,
            ],
        );
    }

    /// How many LoRa symbols the modem must count before it accepts that a
    /// signal is really there. 0 disables the check.
    ///
    /// What makes a duty-cycled receive window cheap: without it a window
    /// that caught noise stays in receive for the whole of its timeout, and
    /// with it the modem gives up after a few symbols and goes back to
    /// sleep. Larger is more confident and costs that much more window on
    /// every cycle for the life of the board.
    pub fn set_lora_symb_num_timeout(&mut self, symbols: u8) {
        self.cmd(op::SET_LORA_SYMB_NUM_TIMEOUT, &[symbols]);
    }

    /// Transmit preamble symbols continuously until a `SetStandby`.
    ///
    /// A test mode, and the instrument for a duty-cycled receiver: with a
    /// preamble always on the air, every receive window that opens should
    /// detect, so the receiver's DIO1 becomes a readout of its own schedule.
    ///
    /// It keys the PA and leaves it keyed. On this board DIO2 switches the
    /// antenna and DIO3 supplies that switch, so this must only ever follow
    /// the ordinary initialization that sets both - keying with either wrong
    /// transmits into an isolated port, which destroys the module rather
    /// than shortening its range.
    pub fn set_tx_infinite_preamble(&mut self) {
        self.cmd(op::SET_TX_INFINITE_PREAMBLE, &[]);
    }

    pub fn set_tx(&mut self, timeout: u32) {
        self.cmd(
            op::SET_TX,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
        );
    }

    pub fn irq_status(&mut self) -> u16 {
        let mut out = [0u8; 2];
        self.cmd_read(op::GET_IRQ_STATUS, &mut out);
        u16::from_be_bytes(out)
    }

    pub fn clear_irq_status(&mut self, mask: u16) {
        self.cmd(op::CLEAR_IRQ_STATUS, &mask.to_be_bytes());
    }

    /// `(payload_len, buffer_offset)` of the packet just received.
    pub fn rx_buffer_status(&mut self) -> (u8, u8) {
        let mut out = [0u8; 2];
        self.cmd_read(op::GET_RX_BUFFER_STATUS, &mut out);
        (out[0], out[1])
    }

    /// `(rssi_dbm, snr_quarter_db)` for the packet just received.
    pub fn lora_packet_status(&mut self) -> (i16, i8) {
        let mut out = [0u8; 3];
        self.cmd_read(op::GET_PACKET_STATUS, &mut out);
        // RssiPkt is -rssi/2 dBm; SnrPkt is two's complement quarter-dB.
        let rssi = -((out[0] as i16) / 2);
        (rssi, out[1] as i8)
    }

    /// The raw status byte. Bits 6:4 are the chip mode.
    ///
    /// Unlike the other getters this has no payload after the status: the
    /// status *is* the answer, returned during the second byte, so it
    /// cannot go through [`Self::cmd_read`].
    pub fn status(&mut self) -> u8 {
        let mut buf = [op::GET_STATUS, 0x00];
        self.xfer(&mut buf);
        buf[1]
    }

    /// Latched operational errors.
    ///
    /// The status byte reports the mode the radio is in, not whether it got
    /// there intact. This is the only thing that names a TCXO that never
    /// started, a calibration or PLL lock that failed, or a PA that would
    /// not ramp - a radio that came up deaf for any of those still reports
    /// a perfectly healthy standby.
    pub fn device_errors(&mut self) -> u16 {
        let mut out = [0u8; 2];
        self.cmd_read(op::GET_DEVICE_ERRORS, &mut out);
        u16::from_be_bytes(out)
    }

    pub fn clear_device_errors(&mut self) {
        self.cmd(op::CLEAR_DEVICE_ERRORS, &[0x00, 0x00]);
    }
}
