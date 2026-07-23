//! Framed UART protocol between the ESP32-C6 and the WIO-E5.
//!
//! Frame format (same scheme as the long-range-radio basestation link):
//!   `[SYNC 0xAA] [LEN_LO] [LEN_HI] [CMD] [PAYLOAD: LEN bytes] [CRC8]`
//!
//! LEN is the payload length only (0-256). CRC8 covers CMD + PAYLOAD.
//! Both directions use the same framing; command ids are split by
//! direction so a device never confuses an echo for a request.

/// Frame sync byte.
pub const SYNC: u8 = 0xAA;

/// Maximum payload size per frame.
pub const MAX_PAYLOAD: usize = 256;

/// UART baud rate on the ESP <-> WIO link.
pub const BAUD: u32 = 115_200;

// -- Command ids: ESP32-C6 -> WIO-E5 (0x01-0x3F) ---------------------------

pub mod cmd {
    /// No payload. WIO answers with [`super::resp::ACK`] (value = fw version).
    pub const PING: u8 = 0x01;

    /// `[flag u8]` - 1: the ESP radio (BLE) is busy, the WIO should defer
    /// discretionary LoRa transmissions; 0: clear. The flag expires on the
    /// WIO after [`super::RADIO_BUSY_TIMEOUT_MS`] in case the clear is lost.
    pub const RADIO_BUSY: u8 = 0x02;

    /// `[flag u8]` - 1: WIO enters soft sleep (radio to standby, GPS and SD
    /// idle, slow loop); 0: wake back up.
    pub const WIO_SLEEP: u8 = 0x03;

    /// `[flag u8]` - 1: put the GPS into backup mode (UBX-RXM-PMREQ);
    /// 0: wake it (EXTINT pulse + UART traffic).
    pub const GPS_SLEEP: u8 = 0x04;

    // Radio TOML config transfer (applied on END, also saved to SD).
    /// `[total_len u16le]` - start a config transfer.
    pub const CFG_BEGIN: u8 = 0x10;
    /// `[seq u16le, bytes...]` - config file data, in order.
    pub const CFG_DATA: u8 = 0x11;
    /// `[crc32 u32le]` - end of config; WIO verifies, parses and applies.
    pub const CFG_END: u8 = 0x12;
    /// No payload. Ask the WIO to report its current radio configuration; it
    /// answers with [`super::msg::CONFIG`] (there is no ACK - the config blob
    /// is the reply). Sent when the ESP wants the value fresh, e.g. on a BLE
    /// connect or when the link first comes up.
    pub const CFG_READ: u8 = 0x13;

    // Firmware update (written into the WIO DFU partition; the swap
    // bootloader installs it on the reboot that follows FW_END).
    /// `[size u32le, crc32 u32le, version u16le]`.
    pub const FW_BEGIN: u8 = 0x20;
    /// `[seq u16le, bytes...]` - firmware data, in order.
    pub const FW_DATA: u8 = 0x21;
    /// No payload. WIO verifies the CRC, marks the swap and reboots.
    pub const FW_END: u8 = 0x22;
    /// No payload. Abandon an in-progress transfer.
    pub const FW_ABORT: u8 = 0x23;
}

// -- Command ids: WIO-E5 -> ESP32-C6 (0x40-0x7F) ---------------------------

pub mod msg {
    /// `[src u8, rssi i16le, PositionPacket 20B]` - a position report.
    /// `src` 0 is the local GPS; other values are the addresses of nodes
    /// whose broadcast we received (rssi is then the LoRa RSSI in dBm).
    pub const POSITION: u8 = 0x40;

    /// [`super::Telemetry`] wire format - periodic link/radio status.
    pub const STATUS: u8 = 0x41;

    /// `[flag u8]` - 1: LoRa TX in progress or imminent, the ESP should
    /// defer discretionary BLE traffic; 0: clear. Expires like RADIO_BUSY.
    pub const RADIO_BUSY: u8 = 0x42;

    /// `[src u8, rssi i16le, payload...]` - a received LoRa payload that is
    /// not a position, forwarded verbatim.
    pub const LORA_RX: u8 = 0x43;

    /// `[text: ASCII bytes]` - a human-readable status/log line. The ESP
    /// prints it to its console and notifies it over BLE (no ACK). Payload
    /// is at most [`super::LOG_MAX`] bytes.
    pub const LOG: u8 = 0x44;

    /// [`crate::radiocfg::RadioConfig::encode`] blob - the WIO's current radio
    /// configuration. The ESP caches it and serves it on the BLE radio-config
    /// characteristic ([`crate::ble::RADIO_CONFIG_UUID`]) without parsing it.
    /// Sent unprompted at boot and after a new config is applied, and in
    /// reply to [`super::cmd::CFG_READ`], so the ESP re-learns it whether the
    /// change was local to the WIO or the ESP restarted under a running WIO.
    pub const CONFIG: u8 = 0x45;
}

/// Host <-> ESP32-C6 commands carried over the ESP's USB Serial/JTAG port,
/// framed identically to the ESP <-> WIO link. They let a computer push a
/// WIO firmware image straight through the ESP without BLE.
pub mod usb {
    /// Host -> ESP, no payload. ESP answers [`super::resp::ACK`]
    /// (`[PING, 1, 0]`) so a tool can confirm it found the firmware.
    pub const PING: u8 = 0x50;
    /// Host -> ESP, `[bulk op bytes]` - one bulk op in the [`crate::ble`]
    /// wire format (`OP_BEGIN`/`OP_DATA`/`OP_END`/`OP_ABORT`). The ESP runs
    /// it through the same path as a BLE bulk write and replies with
    /// [`BULK_ACK`].
    pub const BULK: u8 = 0x51;
    /// ESP -> host, `[id, status, value...]` - the gps-proto ack bytes the
    /// bulk op produced (status 0 = OK).
    pub const BULK_ACK: u8 = 0x52;
    /// Host -> ESP, no payload. ESP answers [`super::resp::ACK`]
    /// (`[INFO, addr[0]..addr[5]]`) with its BLE address most-significant
    /// octet first, so a tool can read a board's address on demand rather
    /// than having to catch the one line it prints at boot.
    pub const INFO: u8 = 0x53;
}

/// Responses (either direction, follow a command).
pub mod resp {
    /// `[cmd u8, value u16le]` - command accepted. `value` is command
    /// specific (fw version for PING, next expected seq for *_DATA).
    pub const ACK: u8 = 0x81;
    /// `[cmd u8, err u8]` - command failed.
    pub const NAK: u8 = 0x82;
}

/// NAK error codes.
pub mod err {
    pub const BAD_FRAME: u8 = 0x01;
    pub const BAD_SIZE: u8 = 0x02;
    pub const BAD_SEQ: u8 = 0x03;
    pub const CRC_MISMATCH: u8 = 0x04;
    pub const FLASH_ERROR: u8 = 0x05;
    pub const INVALID_STATE: u8 = 0x06;
    pub const BAD_CONFIG: u8 = 0x07;
    pub const SD_ERROR: u8 = 0x08;
}

/// A radio-busy flag from the peer expires after this long without a
/// refresh, so a lost "clear" frame cannot wedge the other side.
pub const RADIO_BUSY_TIMEOUT_MS: u32 = 3_000;

/// Data bytes per FW_DATA/CFG_DATA frame. Sized well below [`MAX_PAYLOAD`]
/// so a frame plus response turnaround stays short at 115200 baud.
pub const DATA_CHUNK: usize = 192;

/// Maximum bytes in a [`msg::LOG`] status line (and the matching BLE
/// characteristic value). Longer lines are truncated at the source.
pub const LOG_MAX: usize = 64;

// -- Telemetry (WIO -> ESP -> BLE) ------------------------------------------

/// Set in [`Telemetry::flags`] when the SD card is initialized and logging.
pub const TELEM_FLAG_SD_OK: u8 = 0x01;
/// Set when the GPS currently has a fix.
pub const TELEM_FLAG_GPS_FIX: u8 = 0x02;
/// Set when the radio config was loaded from SD/UART (not defaults).
pub const TELEM_FLAG_CFG_LOADED: u8 = 0x04;
/// Set when the config asks for verbose console logging.
///
/// The ESP never sees the config file - it streams the bytes through to the
/// WIO without parsing them - so the WIO relays this one setting back as a
/// flag bit. Riding on the periodic telemetry rather than a command of its
/// own means the ESP re-learns it every few seconds, so a bit lost to a
/// garbled frame corrects itself and a mid-session config push takes effect
/// without either side tracking whether it was delivered.
pub const TELEM_FLAG_VERBOSE: u8 = 0x08;

pub const TELEMETRY_LEN: usize = 16;

/// Periodic WIO status, also served over BLE (see [`crate::ble`]).
///
/// Layout (little-endian): `last_rssi: i16, last_snr_cb: i16,
/// secs_since_rx: u16, rx_count: u32, tx_count: u32, flags: u8, sats: u8`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Telemetry {
    /// RSSI of the last received LoRa packet (dBm), 0 if none yet.
    pub last_rssi: i16,
    /// SNR of the last received LoRa packet in centibels (quarter-dB * 25).
    pub last_snr_cb: i16,
    /// Seconds since the last LoRa RX; 0xFFFF = never.
    pub secs_since_rx: u16,
    /// LoRa packets received since boot.
    pub rx_count: u32,
    /// LoRa packets transmitted since boot.
    pub tx_count: u32,
    /// TELEM_FLAG_* bits.
    pub flags: u8,
    /// Satellites used in the current GPS fix.
    pub sats: u8,
}

impl Telemetry {
    pub fn encode(&self) -> [u8; TELEMETRY_LEN] {
        let mut b = [0u8; TELEMETRY_LEN];
        b[0..2].copy_from_slice(&self.last_rssi.to_le_bytes());
        b[2..4].copy_from_slice(&self.last_snr_cb.to_le_bytes());
        b[4..6].copy_from_slice(&self.secs_since_rx.to_le_bytes());
        b[6..10].copy_from_slice(&self.rx_count.to_le_bytes());
        b[10..14].copy_from_slice(&self.tx_count.to_le_bytes());
        b[14] = self.flags;
        b[15] = self.sats;
        b
    }

    /// Extra trailing bytes are tolerated; short input is rejected.
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() < TELEMETRY_LEN {
            return None;
        }
        Some(Self {
            last_rssi: i16::from_le_bytes(b[0..2].try_into().ok()?),
            last_snr_cb: i16::from_le_bytes(b[2..4].try_into().ok()?),
            secs_since_rx: u16::from_le_bytes(b[4..6].try_into().ok()?),
            rx_count: u32::from_le_bytes(b[6..10].try_into().ok()?),
            tx_count: u32::from_le_bytes(b[10..14].try_into().ok()?),
            flags: b[14],
            sats: b[15],
        })
    }
}

// -- CRC-8 ------------------------------------------------------------------

/// CRC-8 with polynomial 0x07 (CRC-8/ITU) over CMD + PAYLOAD.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// CRC-32 (IEEE, reflected) used for config and firmware transfer
/// integrity. Matches the standard zlib/`crc32fast` value.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

// -- Frame parser (byte-at-a-time state machine) -----------------------------

#[derive(Clone, Copy)]
enum ParseState {
    Sync,
    LenLo,
    LenHi,
    Data, // collects CMD + PAYLOAD
    Crc,
}

/// A parsed frame ready for processing.
pub struct Frame<'a> {
    pub cmd: u8,
    pub payload: &'a [u8],
}

/// Incremental frame parser. Feed bytes one at a time via [`feed`];
/// when it returns `true`, read the frame with [`frame`].
///
/// [`feed`]: FrameParser::feed
/// [`frame`]: FrameParser::frame
pub struct FrameParser {
    state: ParseState,
    /// Buffer holding CMD + PAYLOAD.
    buf: [u8; MAX_PAYLOAD + 1],
    /// Total expected bytes in buf (1 cmd + len payload).
    expected: usize,
    /// Current write position in buf.
    pos: usize,
}

impl Default for FrameParser {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameParser {
    pub const fn new() -> Self {
        Self {
            state: ParseState::Sync,
            buf: [0u8; MAX_PAYLOAD + 1],
            expected: 0,
            pos: 0,
        }
    }

    /// Feed a single byte. Returns `true` when a complete frame with a
    /// valid CRC has been received.
    pub fn feed(&mut self, byte: u8) -> bool {
        match self.state {
            ParseState::Sync => {
                if byte == SYNC {
                    self.state = ParseState::LenLo;
                }
            }
            ParseState::LenLo => {
                self.expected = byte as usize;
                self.state = ParseState::LenHi;
            }
            ParseState::LenHi => {
                self.expected |= (byte as usize) << 8;
                if self.expected > MAX_PAYLOAD {
                    self.state = ParseState::Sync;
                } else {
                    self.expected += 1; // +1 for the CMD byte
                    self.pos = 0;
                    self.state = ParseState::Data;
                }
            }
            ParseState::Data => {
                if self.pos < self.expected {
                    self.buf[self.pos] = byte;
                    self.pos += 1;
                }
                if self.pos >= self.expected {
                    self.state = ParseState::Crc;
                }
            }
            ParseState::Crc => {
                let computed = crc8(&self.buf[..self.expected]);
                self.state = ParseState::Sync;
                if computed == byte {
                    return true;
                }
                // CRC mismatch: discard the frame silently.
            }
        }
        false
    }

    /// The last parsed frame. Only valid immediately after [`feed`]
    /// returned `true`.
    ///
    /// [`feed`]: FrameParser::feed
    pub fn frame(&self) -> Frame<'_> {
        Frame {
            cmd: self.buf[0],
            payload: &self.buf[1..self.expected],
        }
    }
}

// -- Frame builder ------------------------------------------------------------

/// Max on-wire frame size: 1 sync + 2 len + 1 cmd + payload + 1 crc.
pub const MAX_FRAME: usize = 5 + MAX_PAYLOAD;

/// Scratch buffer for building outgoing frames.
pub struct FrameBuf {
    pub buf: [u8; MAX_FRAME],
    pub len: usize,
}

impl Default for FrameBuf {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameBuf {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME],
            len: 0,
        }
    }

    /// Build a frame with the given command and payload.
    pub fn build(&mut self, cmd: u8, payload: &[u8]) -> &[u8] {
        let plen = payload.len().min(MAX_PAYLOAD);
        self.buf[0] = SYNC;
        self.buf[1] = plen as u8;
        self.buf[2] = (plen >> 8) as u8;
        self.buf[3] = cmd;
        self.buf[4..4 + plen].copy_from_slice(&payload[..plen]);
        let crc_end = 4 + plen;
        self.buf[crc_end] = crc8(&self.buf[3..crc_end]);
        self.len = crc_end + 1;
        self.as_bytes()
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_roundtrip() {
        let mut out = FrameBuf::new();
        let bytes = out.build(cmd::FW_DATA, &[1, 2, 3, 4]);

        let mut parser = FrameParser::new();
        let mut got = None;
        for &b in bytes {
            if parser.feed(b) {
                let f = parser.frame();
                got = Some((f.cmd, f.payload.to_vec()));
            }
        }
        assert_eq!(got, Some((cmd::FW_DATA, vec![1, 2, 3, 4])));
    }

    #[test]
    fn frame_resync_after_garbage() {
        let mut out = FrameBuf::new();
        let bytes = out.build(msg::POSITION, &[9; 23]);

        let mut parser = FrameParser::new();
        let mut hits = 0;
        // Garbage, then a complete corrupted frame (bad crc), then a good
        // frame. The corrupted frame is fully consumed (including its bad
        // CRC byte) before the good frame's sync arrives.
        for &b in [0x00, 0xAA, 0x02, 0x00, 0x55, 0x66, 0x77, 0x00].iter().chain(bytes) {
            if parser.feed(b) {
                hits += 1;
                assert_eq!(parser.frame().cmd, msg::POSITION);
                assert_eq!(parser.frame().payload.len(), 23);
            }
        }
        assert_eq!(hits, 1);
    }

    #[test]
    fn oversized_len_rejected() {
        let mut parser = FrameParser::new();
        // LEN = 0x0FFF > MAX_PAYLOAD: parser must fall back to Sync and
        // then accept a valid frame.
        for b in [SYNC, 0xFF, 0x0F] {
            assert!(!parser.feed(b));
        }
        let mut out = FrameBuf::new();
        let bytes = out.build(cmd::PING, &[]);
        let mut ok = false;
        for &b in bytes {
            ok |= parser.feed(b);
        }
        assert!(ok);
        assert_eq!(parser.frame().cmd, cmd::PING);
        assert!(parser.frame().payload.is_empty());
    }

    #[test]
    fn telemetry_roundtrip() {
        let t = Telemetry {
            last_rssi: -97,
            last_snr_cb: -25,
            secs_since_rx: 12,
            rx_count: 100_000,
            tx_count: 42,
            flags: TELEM_FLAG_SD_OK | TELEM_FLAG_GPS_FIX | TELEM_FLAG_VERBOSE,
            sats: 11,
        };
        let b = t.encode();
        assert_eq!(Telemetry::decode(&b), Some(t));
        assert_eq!(Telemetry::decode(&b[..TELEMETRY_LEN - 1]), None);
        let mut longer = b.to_vec();
        longer.push(0xAB);
        assert_eq!(Telemetry::decode(&longer), Some(t));
    }

    #[test]
    fn crc32_known_value() {
        // Standard IEEE CRC-32 of "123456789".
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }
}
