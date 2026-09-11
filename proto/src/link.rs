//! The framed command protocol, and the bulk transfer built on it.
//!
//! This was the UART link between the ESP32-C6 and the WIO-E5. The Wio-S3
//! board has one MCU and no link, but the host tools speak the same framing
//! over USB (module `usb`) to push a radio config, so the codec outlived
//! its transport. Command ids are still grouped by the direction they had.
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

// The two-MCU command sets that used to sit here - `cmd` (ESP32-C6 ->
// WIO-E5, 0x01-0x3F) and `msg` (WIO-E5 -> ESP32-C6, 0x40-0x7F) - are gone
// with the link itself. Every one of them was a round trip between two
// chips: PING to prove the link alive, RADIO_BUSY to negotiate who owned
// the air, WIO_SLEEP because the ESP could not power the WIO down, and the
// CFG_*/FW_* transfers. On one MCU those are a function call, a mutex, one
// sleep story, and ESP-IDF OTA. The id ranges stay unallocated so a board
// still running the old firmware cannot be half-understood by a new tool.

/// Host <-> board commands carried over the USB Serial/JTAG port, framed as
/// above. They let a computer push a radio config without BLE.
pub mod usb {
    /// Host -> board, no payload. The board answers [`super::resp::ACK`]
    /// (`[PING, 1, 0]`) so a tool can confirm it found the firmware.
    pub const PING: u8 = 0x50;
    /// Host -> board, `[bulk op bytes]` - one bulk op in the [`crate::ble`]
    /// wire format (`OP_BEGIN`/`OP_DATA`/`OP_END`/`OP_ABORT`). The board
    /// runs it through the same path as a BLE bulk write and replies with
    /// [`BULK_ACK`].
    pub const BULK: u8 = 0x51;
    /// Board -> host, `[id, status, value...]` - the gps-proto ack bytes
    /// the bulk op produced (status 0 = OK).
    pub const BULK_ACK: u8 = 0x52;
    /// Host -> board, no payload. The board answers [`super::resp::ACK`]
    /// (`[INFO, addr[0]..addr[5]]`) with its BLE address most-significant
    /// octet first, so a tool can read a board's address on demand rather
    /// than having to catch the one line it prints at boot.
    pub const INFO: u8 = 0x53;
    /// Host -> board, `[secs u32le]` - deep sleep now for that long, with
    /// the same meaning as the BLE `CFG_SLEEP_NOW` write (0 borrows the
    /// configured wake-check cadence). The board answers
    /// [`super::resp::ACK`] (`[SLEEP, secs u16le]`, saturating, so a tool
    /// can say when to expect the port back) and then goes, which drops the
    /// USB device - the port vanishing is the command working.
    pub const SLEEP: u8 = 0x54;
    /// Host -> board, `[id, len, value...]` - one settings write in the
    /// same wire format the BLE config characteristic takes, run through
    /// the same [`crate::session::apply`]. The board answers
    /// [`super::resp::ACK`] (`[CFG, ack bytes...]`) carrying the gps-proto
    /// ack the write produced, so a tool sees the clamped value the board
    /// actually stored rather than the one it asked for.
    ///
    /// Generic on purpose. Every settings id is reachable from a bench with
    /// nothing but a USB cable, which is what a power measurement needs -
    /// the alternative is a phone for every knob.
    pub const CFG: u8 = 0x55;
    /// Host -> board, no payload. The board forgets everything it stores
    /// about itself - the settings record, the name and the radio config
    /// backup in `nvs`, and the RTC RAM copy of the settings that a reset
    /// alone does not clear - answers [`super::resp::ACK`]
    /// (`[WIPE, ok u8]`, 1 when the flash records were erased) and then
    /// restarts, which drops the USB device. The card is not touched: a
    /// `RADIO.CFG` on it is read again at the boot that follows, as it is
    /// at every cold boot.
    ///
    /// Exists because a full flash erase is not a full reset on this chip:
    /// the settings live in RTC RAM as well as flash, and RTC RAM survives
    /// every reset short of a power cycle. This command clears both.
    pub const WIPE: u8 = 0x56;
    /// Host -> board, `[index u16le]` - read one record of the event log
    /// (see [`crate::evlog`]), newest first: index 0 is the last thing
    /// the board wrote down. The board answers [`super::resp::ACK`]
    /// (`[EVLOG, count u16le, index u16le, record...]`) with the record's
    /// bytes, or with no record bytes past the end of the log, so a tool
    /// reads until the reply comes back short. `count` is how many
    /// records the log holds. An index of [`EVLOG_ERASE`] erases the log
    /// instead, and the reply carries a count of 0.
    pub const EVLOG: u8 = 0x57;
    /// The [`EVLOG`] index that erases the log rather than reading it.
    pub const EVLOG_ERASE: u16 = 0xFFFF;
}

/// Responses, following a command.
///
/// There is one. The old link had a NAK with eight error codes beside it;
/// on this board every failure is a gps-proto ack status inside an `ACK`
/// frame, so a tool reads one vocabulary. The id 0x82 stays unallocated.
pub mod resp {
    /// `[cmd u8, value...]` - command accepted. What follows the command
    /// id is command specific: the firmware version for `PING`, the ack
    /// bytes for `CFG` and `BULK`.
    pub const ACK: u8 = 0x81;
}

/// Maximum bytes in a status/log line (and the matching BLE characteristic
/// value). Longer lines are truncated at the source.
///
/// The longest line the firmware builds is the verbose radio breakdown, which
/// runs to about 115 bytes once the drop counters reach six digits; 64 cut
/// it mid-word. A central that never negotiates its ATT MTU up still only
/// sees the first MTU - 3 bytes of whatever arrives.
pub const LOG_MAX: usize = 128;

/// Length of a node-ping report: src + rssi + the two on-air ping fields,
/// laid out `[src u8, rssi i16le, flags u8, uptime_s u16le]`.
///
/// Not [`crate::lora::PING_MSG_LEN`], which is the length of the ping as it
/// travels over LoRa - this one has the receiver's src/rssi in front of it
/// and no message tag.
pub const PING_LEN: usize = 1 + 2 + 1 + 2;

// -- Telemetry (served over BLE) --------------------------------------------

/// Set in [`Telemetry::flags`] when the SD card is initialized and logging.
pub const TELEM_FLAG_SD_OK: u8 = 0x01;
/// Set when the GPS currently has a fix.
pub const TELEM_FLAG_GPS_FIX: u8 = 0x02;
/// Set when a stored radio config was adopted - off the card, out of the
/// flash backup, or pushed - rather than firmware defaults.
pub const TELEM_FLAG_CFG_LOADED: u8 = 0x04;
/// Set when the config asks for verbose console logging.
///
/// A flag bit rather than a value an app can read back: on the two-MCU
/// board the BLE half never parsed the config file, so this was the one
/// setting the radio half relayed. It stays because it is still the only
/// place a connected app learns whether the console is verbose.
pub const TELEM_FLAG_VERBOSE: u8 = 0x08;

/// Set in [`Telemetry::hop`] when the radio is frequency hopping. The low
/// nibble is then the hop clock's stratum: 0 on the board's own GPS time,
/// [`crate::hop::STRATUM_MAX`] free-running with nothing to follow.
pub const TELEM_HOP_ON: u8 = 0x80;
/// Mask of the stratum in [`Telemetry::hop`].
pub const TELEM_HOP_STRATUM: u8 = 0x0F;

/// Length of the telemetry blob before [`Telemetry::ble_rssi`] was
/// appended: the least a decoder accepts, so a board on that firmware still
/// reports everything it has.
pub const TELEMETRY_LEN_V1: usize = 19;
pub const TELEMETRY_LEN: usize = TELEMETRY_LEN_V1 + 1;

/// Periodic radio/GPS status, served over BLE (see [`crate::ble`]).
///
/// Layout (little-endian): `last_rssi: i16, last_snr_cb: i16,
/// secs_since_rx: u16, rx_count: u32, tx_count: u32, flags: u8, sats: u8,
/// hop: u8, hop_channel: u8, parks_missed: u8, ble_rssi: i8`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
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
    /// [`TELEM_HOP_ON`] plus the hop clock's stratum, 0 when not hopping.
    pub hop: u8,
    /// Index of the channel the receiver is on right now, 0 when not
    /// hopping. Watching it change is the cheapest proof the radio hops.
    pub hop_channel: u8,
    /// Deep sleeps entered before the hardware loop finished parking,
    /// since the last cold boot, saturating. Each one is a sleep interval
    /// spent with the receiver, or the radio, still drawing - the one
    /// power failure that is otherwise invisible from the far side.
    pub parks_missed: u8,
    /// The BLE link's signal as the board's controller measures it, in
    /// dBm; 0 when there is no reading. A connected board stops
    /// advertising, so this is the only signal reading a central can have
    /// of it - a scan sees nothing.
    pub ble_rssi: i8,
}

impl Telemetry {
    /// The hop clock's stratum, or `None` when the radio is not hopping.
    pub fn hop_stratum(&self) -> Option<u8> {
        (self.hop & TELEM_HOP_ON != 0).then_some(self.hop & TELEM_HOP_STRATUM)
    }

    pub fn encode(&self) -> [u8; TELEMETRY_LEN] {
        let mut b = [0u8; TELEMETRY_LEN];
        b[0..2].copy_from_slice(&self.last_rssi.to_le_bytes());
        b[2..4].copy_from_slice(&self.last_snr_cb.to_le_bytes());
        b[4..6].copy_from_slice(&self.secs_since_rx.to_le_bytes());
        b[6..10].copy_from_slice(&self.rx_count.to_le_bytes());
        b[10..14].copy_from_slice(&self.tx_count.to_le_bytes());
        b[14] = self.flags;
        b[15] = self.sats;
        b[16] = self.hop;
        b[17] = self.hop_channel;
        b[18] = self.parks_missed;
        b[19] = self.ble_rssi as u8;
        b
    }

    /// Extra trailing bytes are tolerated; input shorter than
    /// [`TELEMETRY_LEN_V1`] is rejected. A blob of exactly that length is
    /// firmware from before the link RSSI, which reads back as no reading.
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() < TELEMETRY_LEN_V1 {
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
            hop: b[16],
            hop_channel: b[17],
            parks_missed: b[18],
            ble_rssi: b.get(19).map(|&v| v as i8).unwrap_or(0),
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
        let bytes = out.build(usb::BULK, &[1, 2, 3, 4]);

        let mut parser = FrameParser::new();
        let mut got = None;
        for &b in bytes {
            if parser.feed(b) {
                let f = parser.frame();
                got = Some((f.cmd, f.payload.to_vec()));
            }
        }
        assert_eq!(got, Some((usb::BULK, vec![1, 2, 3, 4])));
    }

    #[test]
    fn frame_resync_after_garbage() {
        let mut out = FrameBuf::new();
        let bytes = out.build(usb::BULK_ACK, &[9; 23]);

        let mut parser = FrameParser::new();
        let mut hits = 0;
        // Garbage, then a complete corrupted frame (bad crc), then a good
        // frame. The corrupted frame is fully consumed (including its bad
        // CRC byte) before the good frame's sync arrives.
        for &b in [0x00, 0xAA, 0x02, 0x00, 0x55, 0x66, 0x77, 0x00].iter().chain(bytes) {
            if parser.feed(b) {
                hits += 1;
                assert_eq!(parser.frame().cmd, usb::BULK_ACK);
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
        let bytes = out.build(usb::PING, &[]);
        let mut ok = false;
        for &b in bytes {
            ok |= parser.feed(b);
        }
        assert!(ok);
        assert_eq!(parser.frame().cmd, usb::PING);
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
            hop: TELEM_HOP_ON | 3,
            hop_channel: 27,
            parks_missed: 3,
            ble_rssi: -63,
        };
        let b = t.encode();
        assert_eq!(Telemetry::decode(&b), Some(t));
        assert_eq!(t.hop_stratum(), Some(3));
        assert_eq!(Telemetry { hop: 0, ..t }.hop_stratum(), None);
        assert_eq!(Telemetry::decode(&b[..TELEMETRY_LEN_V1 - 1]), None);
        // A board from before the link RSSI sends the shorter blob, and
        // everything it does send still arrives.
        assert_eq!(
            Telemetry::decode(&b[..TELEMETRY_LEN_V1]),
            Some(Telemetry { ble_rssi: 0, ..t })
        );
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
