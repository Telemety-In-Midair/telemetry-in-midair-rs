//! MAX-M10 GPS receiver on USART1: PB7 = RX (module TX -> MCU), PB6 = TX
//! (MCU -> module, used for UBX power commands), 9600 8N1 factory default.
//! PB10 drives the module's EXTINT pin for wake-from-backup.
//!
//! NMEA parsing is shared with the ESP32-C3 beacon via gps-proto: RMC owns
//! the fix flag, position, motion and time; GGA contributes altitude and
//! satellite count. The folded state is a wire-ready [`PositionPacket`],
//! whose fix-derived fields are zeroed when the fix drops rather than left
//! holding their last values.

use cortex_m::interrupt::CriticalSection;
use gps_proto::nmea::{self, Sentence};
use gps_proto::packet::{PositionPacket, FLAG_FIX};
use midair_proto::radiocfg::GpsConfig;
use stm32wlxx_hal::{
    embedded_hal::serial::{Read, Write},
    gpio::{pins, Output, OutputArgs, PinState},
    pac,
    uart::{self, Uart1},
};

/// NMEA line rate - the u-blox M10 factory default.
pub const BAUD: u32 = 9_600;

/// Longest NMEA sentence: 82 characters including "$" and CRLF.
const NMEA_MAX: usize = 82;

/// Bytes drained from USART1 per [`poll`](Gps::poll) call. A fix at the 1 Hz
/// default emits a few hundred bytes a second, so the main loop's ~1 kHz
/// polling never approaches this in normal use; the cap exists for the
/// opposite case, where no module is attached and the floating RX pin streams
/// noise. Bounding the drain keeps that from monopolizing the loop and
/// starving the radio receive, so a node with no GPS still hears the network.
const DRAIN_BUDGET: usize = 512;

/// Largest UBX payload this driver builds. The CFG-VALSET frame it emits
/// (4-byte header + fifteen key/value pairs) is well under this.
const UBX_MAX_PAYLOAD: usize = 128;

// UBX-CFG-VALSET configuration keys (u-blox M10, protocol 34.x). The high
// nibble region encodes the value size: 0x10.. = L (1 byte), 0x20.. = U1/E1
// (1 byte), 0x30.. = U2 (2 bytes).
const CFG_SIGNAL_GPS_ENA: u32 = 0x1031_001F;
const CFG_SIGNAL_SBAS_ENA: u32 = 0x1031_0020;
const CFG_SIGNAL_GAL_ENA: u32 = 0x1031_0021;
const CFG_SIGNAL_BDS_ENA: u32 = 0x1031_0022;
const CFG_SIGNAL_QZSS_ENA: u32 = 0x1031_0024;
const CFG_SIGNAL_GLO_ENA: u32 = 0x1031_0025;
const CFG_PM_OPERATEMODE: u32 = 0x20D0_0001;
const CFG_RATE_MEAS: u32 = 0x3021_0001;
const CFG_NAVSPG_DYNMODEL: u32 = 0x2011_0021;

// CFG-MSGOUT-NMEA_ID_*_UART1: per-sentence output rate in navigation
// epochs, 0 to turn the sentence off.
const CFG_MSGOUT_RMC: u32 = 0x2091_00AC;
const CFG_MSGOUT_GGA: u32 = 0x2091_00BB;
const CFG_MSGOUT_GLL: u32 = 0x2091_00CA;
const CFG_MSGOUT_GSA: u32 = 0x2091_00C0;
const CFG_MSGOUT_GSV: u32 = 0x2091_00C5;
const CFG_MSGOUT_VTG: u32 = 0x2091_00B1;

/// How long [`Gps::configure`] waits for the module to acknowledge.
const ACK_TIMEOUT_MS: u32 = 250;

pub struct Gps {
    uart: Uart1<pins::B7, pins::B6>,
    extint: Output<pins::B10>,
    line: [u8; NMEA_MAX],
    len: usize,
    in_line: bool,
    /// Folded position state, wire-ready.
    packet: PositionPacket,
    /// Set when an accepted sentence updated `packet` since the last take.
    updated: bool,
    /// Whether the module was put into backup mode.
    pub sleeping: bool,
    /// Whether the module acknowledged the last [`configure`](Self::configure).
    /// False after a push that landed before the module had finished
    /// starting, which is the usual outcome at boot.
    pub configured: bool,
    /// Total bytes read from USART1 since boot (saturating). Presence check:
    /// 0 means nothing on the wire (unpowered / miswired / RX pin).
    rx_bytes: u32,
    /// Total valid NMEA sentences parsed since boot (saturating). >0 means
    /// the module is talking at the expected baud.
    rx_sentences: u32,
}

impl Gps {
    pub fn new(
        usart1: pac::USART1,
        b6: pins::B6,
        b7: pins::B7,
        b10: pins::B10,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        // HSI16 keeps the baud rate exact and independent of the MSI
        // system clock.
        let uart = Uart1::new(usart1, BAUD, uart::Clk::Hsi16, rcc)
            .enable_rx(b7, cs)
            .enable_tx(b6, cs);
        const ARGS: OutputArgs = OutputArgs {
            level: PinState::Low,
            ..OutputArgs::new()
        };
        Self {
            uart,
            extint: Output::new(b10, &ARGS, cs),
            line: [0; NMEA_MAX],
            len: 0,
            in_line: false,
            packet: PositionPacket::default(),
            updated: false,
            sleeping: false,
            configured: false,
            rx_bytes: 0,
            rx_sentences: 0,
        }
    }

    /// Latest folded position snapshot.
    pub fn packet(&self) -> PositionPacket {
        self.packet
    }

    /// Bytes read from the module since boot. 0 = nothing on USART1.
    pub fn rx_bytes(&self) -> u32 {
        self.rx_bytes
    }

    /// Valid NMEA sentences parsed since boot.
    pub fn rx_sentences(&self) -> u32 {
        self.rx_sentences
    }

    /// Whether the module has produced at least one valid NMEA sentence,
    /// i.e. it is powered, wired and talking at the expected baud. A fix is
    /// a separate question ([`has_fix`](Self::has_fix)).
    pub fn present(&self) -> bool {
        self.rx_sentences > 0
    }

    /// Whether the position state changed since the last call.
    pub fn take_updated(&mut self) -> bool {
        core::mem::take(&mut self.updated)
    }

    pub fn has_fix(&self) -> bool {
        self.packet.flags & FLAG_FIX != 0
    }

    /// Clear sticky UART error flags (overrun keeps erroring until
    /// acknowledged in ICR). Overruns are expected: other work in the
    /// main loop can block longer than one character time.
    fn clear_errors(&mut self) {
        // The HAL keeps the register block private; USART1 is owned by
        // `self.uart` so this access is exclusive.
        unsafe {
            let usart1 = &*pac::USART1::PTR;
            usart1.icr.write(|w| {
                w.orecf()
                    .set_bit()
                    .fecf()
                    .set_bit()
                    .pecf()
                    .set_bit()
                    .ncf()
                    .set_bit()
            });
        }
    }

    /// Drain the UART and fold complete sentences into the position
    /// state. Call every main-loop iteration.
    pub fn poll(&mut self) {
        for _ in 0..DRAIN_BUDGET {
            let byte = match self.uart.read() {
                Ok(b) => b,
                Err(nb::Error::WouldBlock) => return,
                Err(nb::Error::Other(_)) => {
                    // Lost bytes: drop the partial line and resync.
                    self.clear_errors();
                    self.in_line = false;
                    self.len = 0;
                    continue;
                }
            };
            self.rx_bytes = self.rx_bytes.saturating_add(1);
            match byte {
                b'$' => {
                    self.line[0] = b'$';
                    self.len = 1;
                    self.in_line = true;
                }
                b'\r' | b'\n' => {
                    if self.in_line
                        && let Ok(s) = core::str::from_utf8(&self.line[..self.len]) {
                            crate::debug_println!("gps: {}", s);
                            if let Some(sentence) = nmea::parse(s) {
                                self.rx_sentences = self.rx_sentences.saturating_add(1);
                                self.fold(sentence);
                                self.updated = true;
                            }
                        }
                    self.in_line = false;
                    self.len = 0;
                }
                _ if self.in_line => {
                    if self.len < NMEA_MAX {
                        self.line[self.len] = byte;
                        self.len += 1;
                    } else {
                        // Overlong garbage: resync on the next '$'.
                        self.in_line = false;
                        self.len = 0;
                    }
                }
                _ => {}
            }
        }
    }

    fn fold(&mut self, s: Sentence) {
        let p = &mut self.packet;
        match s {
            Sentence::Rmc(rmc) => {
                if rmc.valid {
                    if let (Some(lat), Some(lon)) = (rmc.lat_e7, rmc.lon_e7) {
                        p.lat_e7 = lat;
                        p.lon_e7 = lon;
                        p.flags |= FLAG_FIX;
                    }
                } else {
                    p.flags &= !FLAG_FIX;
                    clear_fix_fields(p);
                }
                if let Some(v) = rmc.speed_cms {
                    p.speed_cms = v;
                }
                if let Some(v) = rmc.course_cdeg {
                    p.course_cdeg = v;
                }
                if let Some(v) = rmc.tod_ms {
                    p.tod_ms = v;
                }
            }
            Sentence::Gga(gga) => {
                if let Some(v) = gga.alt_dm {
                    p.alt_dm = v;
                }
                if let Some(v) = gga.sats {
                    p.sats = v;
                }
            }
        }
    }

    fn write_all(&mut self, bytes: &[u8]) {
        for &b in bytes {
            let _ = nb::block!(self.uart.write(b));
        }
        let _ = nb::block!(self.uart.flush());
    }

    /// Frame and send a UBX message: `B5 62 class id len_lo len_hi payload
    /// ck_a ck_b`, with the 8-bit Fletcher checksum over class..payload.
    /// Payloads longer than [`UBX_MAX_PAYLOAD`] are dropped (never happens
    /// for the frames this driver builds).
    fn ubx(&mut self, class: u8, id: u8, payload: &[u8]) {
        if payload.len() > UBX_MAX_PAYLOAD {
            return;
        }
        let mut frame = [0u8; 8 + UBX_MAX_PAYLOAD];
        frame[0] = 0xB5;
        frame[1] = 0x62;
        frame[2] = class;
        frame[3] = id;
        frame[4] = payload.len() as u8;
        frame[5] = (payload.len() >> 8) as u8;
        frame[6..6 + payload.len()].copy_from_slice(payload);
        let (mut ck_a, mut ck_b) = (0u8, 0u8);
        for &b in &frame[2..6 + payload.len()] {
            ck_a = ck_a.wrapping_add(b);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        frame[6 + payload.len()] = ck_a;
        frame[7 + payload.len()] = ck_b;
        self.write_all(&frame[..8 + payload.len()]);
    }

    /// Apply a [`GpsConfig`] with a single UBX-CFG-VALSET (RAM layer). The
    /// WIO controls the module's power rail, so it re-runs at every boot;
    /// the RAM layer is enough and avoids wearing battery-backed storage.
    /// No-op while the module is in backup mode.
    ///
    /// Returns whether the module acknowledged. A rejected frame - an
    /// unsupported key, or a constellation set the receiver cannot track
    /// concurrently - otherwise leaves it quietly running its old settings
    /// while the firmware reports the ones it asked for.
    pub fn configure(&mut self, cfg: &GpsConfig) -> bool {
        if self.sleeping {
            return false;
        }
        // VALSET payload: version(0), layers(bit0 = RAM), reserved[2], then
        // key/value pairs. Value width is encoded in the key id (bits 30:28:
        // 0x1 = 1 byte, 0x3 = 2 bytes).
        let mut p = [0u8; UBX_MAX_PAYLOAD];
        let mut n = 0usize;
        p[1] = 0x01; // layers = RAM
        n += 4;
        let mut put = |key: u32, val: &[u8]| {
            p[n..n + 4].copy_from_slice(&key.to_le_bytes());
            n += 4;
            p[n..n + val.len()].copy_from_slice(val);
            n += val.len();
        };
        // CFG-SIGNAL-*_ENA (L, 1 byte): constellation enables.
        put(CFG_SIGNAL_GPS_ENA, &[cfg.gps_enabled as u8]);
        put(CFG_SIGNAL_SBAS_ENA, &[cfg.sbas_enabled as u8]);
        put(CFG_SIGNAL_GAL_ENA, &[cfg.galileo_enabled as u8]);
        put(CFG_SIGNAL_BDS_ENA, &[cfg.beidou_enabled as u8]);
        put(CFG_SIGNAL_QZSS_ENA, &[cfg.qzss_enabled as u8]);
        put(CFG_SIGNAL_GLO_ENA, &[cfg.glonass_enabled as u8]);
        // CFG-PM-OPERATEMODE (E1, 1 byte).
        put(CFG_PM_OPERATEMODE, &[cfg.power_mode.operate_mode()]);
        // CFG-RATE-MEAS (U2, 2 bytes, ms).
        put(CFG_RATE_MEAS, &cfg.meas_rate_ms.to_le_bytes());
        // CFG-NAVSPG-DYNMODEL (E1, 1 byte).
        put(CFG_NAVSPG_DYNMODEL, &[cfg.dyn_model.dynmodel()]);
        // NMEA output (U1, 1 byte each): keep the two sentences this
        // firmware reads and silence the four the module also sends by
        // default. At 9600 baud there are 960 bytes a second to spend, and
        // with several constellations enabled GSV alone can exceed that in
        // one epoch - which pushes RMC and GGA behind sentences nothing
        // parses, and shows up as the USART overruns `poll` treats as
        // routine rather than as anything obviously wrong.
        put(CFG_MSGOUT_RMC, &[1]);
        put(CFG_MSGOUT_GGA, &[1]);
        put(CFG_MSGOUT_GLL, &[0]);
        put(CFG_MSGOUT_GSA, &[0]);
        put(CFG_MSGOUT_GSV, &[0]);
        put(CFG_MSGOUT_VTG, &[0]);
        self.ubx(0x06, 0x8A, &p[..n]);
        self.configured = self.wait_ack(0x06, 0x8A);
        self.configured
    }

    /// Wait for the module to acknowledge the message `class`/`id`.
    ///
    /// The receiver keeps emitting NMEA while it answers, so this scans the
    /// byte stream for a UBX-ACK frame rather than expecting the reply to
    /// arrive on its own. Bytes consumed here are bytes the NMEA parser does
    /// not see, which costs at most one epoch of position.
    fn wait_ack(&mut self, class: u8, id: u8) -> bool {
        // B5 62 05 <01 ack | 00 nak> 02 00 <class> <id>, then the checksum.
        const ACK: usize = 3;
        let expected: [u8; 8] = [0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, class, id];
        let mut matched = 0usize;

        let start = crate::platform::millis();
        while crate::platform::millis().wrapping_sub(start) < ACK_TIMEOUT_MS {
            let byte = match self.uart.read() {
                Ok(b) => b,
                Err(nb::Error::WouldBlock) => continue,
                Err(nb::Error::Other(_)) => {
                    self.clear_errors();
                    matched = 0;
                    continue;
                }
            };
            self.rx_bytes = self.rx_bytes.saturating_add(1);
            // The ack/nak byte is the one position where both values are a
            // valid frame; a NAK is an answer, so stop on it either way.
            if matched == ACK && byte == 0x00 {
                crate::debug_println!("gps: config rejected (NAK)");
                return false;
            }
            if byte == expected[matched] {
                matched += 1;
                if matched == expected.len() {
                    return true;
                }
            } else {
                // A mismatch can itself be the start of the next frame.
                matched = (byte == expected[0]) as usize;
            }
        }
        crate::debug_println!("gps: no ack for config");
        false
    }

    /// Put the module into backup mode (UBX-RXM-PMREQ, indefinite, wake
    /// on EXTINT or UART RX activity).
    pub fn sleep(&mut self) {
        // UART RX is one of the wake sources below, so sending the request to
        // a module that is already in backup would wake it just to put it back.
        if self.sleeping {
            return;
        }
        // Version-0 16-byte payload: version, reserved[3], duration (0 =
        // until wake source), flags (bit1 = backup), wakeupSources
        // (bit3 = uartrx, bit5 = extint0).
        let mut payload = [0u8; 16];
        payload[8..12].copy_from_slice(&2u32.to_le_bytes()); // flags: backup
        payload[12..16].copy_from_slice(&((1u32 << 3) | (1u32 << 5)).to_le_bytes());
        self.ubx(0x02, 0x41, &payload); // class RXM, id PMREQ
        self.extint.set_level_low();
        self.sleeping = true;
        self.packet.flags &= !FLAG_FIX;
        clear_fix_fields(&mut self.packet);
    }

    /// Wake the module from backup: EXTINT pulse plus UART traffic.
    ///
    /// Clears [`configured`](Self::configured), because backup mode cuts power
    /// to the receiver core and the RAM configuration layer - the only one
    /// [`configure`](Self::configure) writes - does not survive it. The module
    /// comes back on its BBR/flash defaults, which means the four NMEA
    /// sentences this firmware silences are talking again and the measurement
    /// rate is whatever the factory set. The caller re-pushes the settings once
    /// the module is producing sentences again; it is not done here because the
    /// receiver needs a moment to start and would not acknowledge yet.
    pub fn wake(&mut self) {
        if !self.sleeping {
            return;
        }
        self.extint.set_level_high();
        // Hold EXTINT high a few ms so the edge is registered.
        cortex_m::asm::delay(crate::platform::SYSCLK_HZ / 1000 * 5);
        self.extint.set_level_low();
        self.write_all(&[0xFF, 0xFF]);
        self.sleeping = false;
        self.configured = false;
    }
}

/// Zero the fields that only mean anything while a fix holds.
///
/// Each is written from an optional NMEA field, and the module leaves those
/// fields empty once the fix drops - GGA stops carrying altitude, RMC stops
/// carrying speed and course. Without this the last good values would sit in
/// the packet indefinitely, so anything that reads it without checking
/// [`FLAG_FIX`] sees an altitude from minutes or hours ago as if it were
/// current.
///
/// `tod_ms` is deliberately not cleared: the receiver keeps decoding time
/// from the satellites it still tracks, so time stays valid across a fix
/// loss and RMC keeps carrying it.
fn clear_fix_fields(p: &mut PositionPacket) {
    p.lat_e7 = 0;
    p.lon_e7 = 0;
    p.alt_dm = 0;
    p.speed_cms = 0;
    p.course_cdeg = 0;
    p.sats = 0;
}
