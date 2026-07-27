//! The LoRa over-air frame and the payload formats it carries.
//!
//! Every transmission is a message: one 3-byte header followed by an
//! application payload. There is no addressing beyond the originator and
//! no routing state - a node either repeats a frame or it does not, which
//! is all a leaf/repeater topology needs.
//!
//! ```text
//! [0] src        originating node address, 1-255
//! [1] id         originator's sequence number, wraps at 256
//! [2] hops_left  remaining retransmissions; 0 = nobody repeats this
//! [3..] payload  1..=PAYLOAD_MAX application bytes
//! ```
//!
//! `(src, id)` identifies a frame for as long as it is in flight, which is
//! what lets a receiver drop duplicates and a repeater avoid looping.
//!
//! There is no checksum here: the SX126x transmits LoRa packets with its
//! hardware CRC enabled and the driver discards frames that fail it, so a
//! software check would only repeat work already done in the radio.
//!
//! Payloads are sent at their true length - nothing is padded - so shrinking
//! a payload format shortens the air time it costs.

use gps_proto::packet::{PositionPacket, FLAG_FIX};

/// Frame header length: `[src, id, hops_left]`.
pub const HEADER_LEN: usize = 3;

/// Largest application payload carried in one frame. Matches the payload
/// space the ESP link reserves for a forwarded frame.
pub const PAYLOAD_MAX: usize = 32;

/// Largest encoded frame.
pub const FRAME_MAX: usize = HEADER_LEN + PAYLOAD_MAX;

/// A decoded over-air frame borrowing its payload from the receive buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    /// Address of the node that originated the frame (never a repeater's).
    pub src: u8,
    /// Originator's sequence number.
    pub id: u8,
    /// Retransmissions still permitted. A repeater forwards only when this
    /// is non-zero, and decrements it on the way out.
    pub hops_left: u8,
    /// Application payload.
    pub payload: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Encoded length of this frame.
    pub fn encoded_len(&self) -> usize {
        HEADER_LEN + self.payload.len()
    }

    /// Write the frame into `out`, returning the number of bytes written.
    ///
    /// Returns `None` if the payload is empty, longer than [`PAYLOAD_MAX`],
    /// or `out` is too small.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let n = self.encoded_len();
        if self.payload.is_empty() || self.payload.len() > PAYLOAD_MAX || out.len() < n {
            return None;
        }
        out[0] = self.src;
        out[1] = self.id;
        out[2] = self.hops_left;
        out[HEADER_LEN..n].copy_from_slice(self.payload);
        Some(n)
    }

    /// Decode a received packet.
    ///
    /// Returns `None` for a runt, an empty payload, or a source address of
    /// 0 - the last of which is not a legal address and so marks the packet
    /// as something other than one of ours.
    pub fn decode(bytes: &'a [u8]) -> Option<Frame<'a>> {
        if bytes.len() <= HEADER_LEN || bytes[0] == 0 {
            return None;
        }
        Some(Frame {
            src: bytes[0],
            id: bytes[1],
            hops_left: bytes[2],
            payload: &bytes[HEADER_LEN..],
        })
    }
}

/// Position transmission: `[POSITION tag] [field mask] [selected fields]`.
///
/// The sender picks which fields to spend air time on (see
/// `RadioConfig::beacon_fields`) and stamps its choice into the mask, so the
/// frame describes its own layout and two nodes configured differently still
/// understand each other.
///
/// Tag 0x50 was the earlier fixed 20-byte layout and is deliberately not
/// reused: firmware of either vintage rejects the other's tag outright
/// rather than reading a mask out of a latitude.
pub const MSG_POSITION: u8 = 0x51;

/// Field mask bits, in the order the fields appear in the payload.
pub const FIELD_LAT: u8 = 1 << 0;
pub const FIELD_LON: u8 = 1 << 1;
pub const FIELD_ALT: u8 = 1 << 2;
pub const FIELD_SPEED: u8 = 1 << 3;
pub const FIELD_COURSE: u8 = 1 << 4;
pub const FIELD_SATS: u8 = 1 << 5;
pub const FIELD_TIME: u8 = 1 << 6;

/// Every field this format can carry.
pub const FIELDS_ALL: u8 =
    FIELD_LAT | FIELD_LON | FIELD_ALT | FIELD_SPEED | FIELD_COURSE | FIELD_SATS | FIELD_TIME;

/// Position and nothing else - the default beacon payload.
pub const FIELDS_DEFAULT: u8 = FIELD_LAT | FIELD_LON;

/// Fields without which a position transmission is not one.
pub const FIELDS_REQUIRED: u8 = FIELD_LAT | FIELD_LON;

/// Longest encoded position message: tag + mask + every field.
pub const POSITION_MSG_MAX: usize = 2 + 4 + 4 + 2 + 2 + 2 + 1 + 4;

/// Wire width of each field present in `mask`.
const fn fields_len(mask: u8) -> usize {
    let mut n = 0;
    if mask & FIELD_LAT != 0 {
        n += 4;
    }
    if mask & FIELD_LON != 0 {
        n += 4;
    }
    if mask & FIELD_ALT != 0 {
        n += 2;
    }
    if mask & FIELD_SPEED != 0 {
        n += 2;
    }
    if mask & FIELD_COURSE != 0 {
        n += 2;
    }
    if mask & FIELD_SATS != 0 {
        n += 1;
    }
    if mask & FIELD_TIME != 0 {
        n += 4;
    }
    n
}

/// Encoded length of a position message carrying `mask`.
pub const fn position_msg_len(mask: u8) -> usize {
    2 + fields_len(mask)
}

/// Encode a position transmission carrying the fields in `mask`, returning the
/// buffer and the used length. Bits outside [`FIELDS_ALL`] are ignored.
pub fn encode_position(p: &PositionPacket, mask: u8) -> ([u8; POSITION_MSG_MAX], usize) {
    let mask = mask & FIELDS_ALL;
    let mut b = [0u8; POSITION_MSG_MAX];
    b[0] = MSG_POSITION;
    b[1] = mask;
    let mut n = 2;
    let mut put = |bytes: &[u8]| {
        b[n..n + bytes.len()].copy_from_slice(bytes);
        n += bytes.len();
    };
    if mask & FIELD_LAT != 0 {
        put(&p.lat_e7.to_le_bytes());
    }
    if mask & FIELD_LON != 0 {
        put(&p.lon_e7.to_le_bytes());
    }
    if mask & FIELD_ALT != 0 {
        put(&p.alt_dm.to_le_bytes());
    }
    if mask & FIELD_SPEED != 0 {
        put(&p.speed_cms.to_le_bytes());
    }
    if mask & FIELD_COURSE != 0 {
        put(&p.course_cdeg.to_le_bytes());
    }
    if mask & FIELD_SATS != 0 {
        put(&[p.sats]);
    }
    if mask & FIELD_TIME != 0 {
        put(&p.tod_ms.to_le_bytes());
    }
    (b, n)
}

/// Decode a position transmission, or `None` if the payload is something else
/// or is shorter than its own mask claims.
///
/// Fields the sender left out come back zeroed. [`FLAG_FIX`] is always set:
/// a node only beacons while it has a fix, so receiving one is the proof,
/// and the flag costs nothing to reconstruct here.
pub fn decode_position(data: &[u8]) -> Option<PositionPacket> {
    if data.first() != Some(&MSG_POSITION) || data.len() < 2 {
        return None;
    }
    let mask = data[1] & FIELDS_ALL;
    let body = data.get(2..2 + fields_len(mask))?;

    let mut n = 0;
    let mut take = |len: usize| {
        let s = &body[n..n + len];
        n += len;
        s
    };
    let mut p = PositionPacket {
        flags: FLAG_FIX,
        ..PositionPacket::default()
    };
    if mask & FIELD_LAT != 0 {
        p.lat_e7 = i32::from_le_bytes(take(4).try_into().ok()?);
    }
    if mask & FIELD_LON != 0 {
        p.lon_e7 = i32::from_le_bytes(take(4).try_into().ok()?);
    }
    if mask & FIELD_ALT != 0 {
        p.alt_dm = i16::from_le_bytes(take(2).try_into().ok()?);
    }
    if mask & FIELD_SPEED != 0 {
        p.speed_cms = u16::from_le_bytes(take(2).try_into().ok()?);
    }
    if mask & FIELD_COURSE != 0 {
        p.course_cdeg = u16::from_le_bytes(take(2).try_into().ok()?);
    }
    if mask & FIELD_SATS != 0 {
        p.sats = take(1)[0];
    }
    if mask & FIELD_TIME != 0 {
        p.tod_ms = u32::from_le_bytes(take(4).try_into().ok()?);
    }
    Some(p)
}

/// Ping transmission: `[PING tag] [flags] [uptime_s u16le]`.
///
/// What a node puts on the air in place of a position while it has no fix.
/// A silent node is indistinguishable from one out of range or one that is
/// dead, so a node with nothing to report says so instead: a receiver then
/// knows the node is alive, and the two flags say why it has no position
/// yet - a receiver still searching, one that never came up at all, or a
/// fix that was held and lost.
///
/// Four bytes, so a ping costs less air time than the leanest position, and
/// a node that never sees the sky is cheaper on the channel than a fixed
/// one rather than more expensive.
pub const MSG_PING: u8 = 0x52;

/// Encoded length of a ping message. Fixed - every field is always present.
pub const PING_MSG_LEN: usize = 4;

/// Set in [`Ping::flags`] when the GPS module is producing NMEA.
pub const PING_FLAG_GPS_PRESENT: u8 = 1 << 0;
/// Set when the sender has held a fix at some point since it booted.
pub const PING_FLAG_HAD_FIX: u8 = 1 << 1;

/// A node reporting that it is on the air without a position to send.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ping {
    /// Seconds since the sender booted, saturating at [`u16::MAX`] (18 h).
    /// Read against a previous ping it also shows a node that rebooted.
    pub uptime_s: u16,
    /// The GPS module is talking (at least one NMEA sentence parsed). Clear
    /// means a silent module - usually an unpowered rail or wiring, not a
    /// receiver that cannot find the sky.
    pub gps_present: bool,
    /// The sender has had a fix since boot, so this ping is a fix lost
    /// rather than one never acquired.
    pub had_fix: bool,
}

impl Ping {
    /// Flag byte as it travels on the air.
    pub fn flags(&self) -> u8 {
        let mut f = 0;
        if self.gps_present {
            f |= PING_FLAG_GPS_PRESENT;
        }
        if self.had_fix {
            f |= PING_FLAG_HAD_FIX;
        }
        f
    }

    /// Encode the ping as a frame payload.
    pub fn encode(&self) -> [u8; PING_MSG_LEN] {
        let mut b = [0u8; PING_MSG_LEN];
        b[0] = MSG_PING;
        b[1] = self.flags();
        b[2..4].copy_from_slice(&self.uptime_s.to_le_bytes());
        b
    }

    /// Decode a ping, or `None` if the payload is something else or short.
    /// Unknown flag bits are ignored, so a future sender that sets one is
    /// still understood here.
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.first() != Some(&MSG_PING) || data.len() < PING_MSG_LEN {
            return None;
        }
        Some(Self {
            uptime_s: u16::from_le_bytes([data[2], data[3]]),
            gps_present: data[1] & PING_FLAG_GPS_PRESENT != 0,
            had_fix: data[1] & PING_FLAG_HAD_FIX != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gps_proto::packet::FLAG_FIX;

    fn sample() -> PositionPacket {
        PositionPacket {
            lat_e7: 481_173_000,
            lon_e7: -1_226_760_000,
            alt_dm: 1234,
            speed_cms: 0,
            course_cdeg: 0,
            flags: FLAG_FIX,
            sats: 7,
            tod_ms: 1000,
        }
    }

    #[test]
    fn position_roundtrip_all_fields() {
        let p = sample();
        let (enc, n) = encode_position(&p, FIELDS_ALL);
        assert_eq!(n, POSITION_MSG_MAX);
        assert_eq!(decode_position(&enc[..n]), Some(p));
        assert_eq!(decode_position(b"hello"), None);
        // A tagged payload shorter than its own mask claims is rejected
        // rather than decoded from whatever follows it in the buffer.
        assert_eq!(decode_position(&enc[..n - 1]), None);
        assert_eq!(decode_position(&enc[..1]), None);
    }

    /// The default beacon is position only, and that is what the air time
    /// saving rests on: 10 payload bytes against the 21 of a full packet.
    #[test]
    fn default_fields_carry_position_only() {
        let (enc, n) = encode_position(&sample(), FIELDS_DEFAULT);
        assert_eq!(n, 10);
        assert_eq!(position_msg_len(FIELDS_DEFAULT), 10);
        let got = decode_position(&enc[..n]).unwrap();
        assert_eq!((got.lat_e7, got.lon_e7), (481_173_000, -1_226_760_000));
        // Everything not selected comes back zeroed, not stale or garbage.
        assert_eq!(got.alt_dm, 0);
        assert_eq!(got.sats, 0);
        assert_eq!(got.tod_ms, 0);
        // Receiving a beacon at all is proof the sender had a fix.
        assert!(got.has_fix());
    }

    #[test]
    fn each_field_costs_its_own_width() {
        let base = position_msg_len(FIELDS_DEFAULT);
        for (bit, width) in [
            (FIELD_ALT, 2),
            (FIELD_SPEED, 2),
            (FIELD_COURSE, 2),
            (FIELD_SATS, 1),
            (FIELD_TIME, 4),
        ] {
            let (_, n) = encode_position(&sample(), FIELDS_DEFAULT | bit);
            assert_eq!(n, base + width, "field {bit:#04x}");
        }
    }

    /// A sender that adds a field and one that does not are both decodable
    /// by the same receiver - the mask travels with the frame.
    #[test]
    fn mixed_senders_interoperate() {
        let (lean, ln) = encode_position(&sample(), FIELDS_DEFAULT);
        let (rich, rn) = encode_position(&sample(), FIELDS_DEFAULT | FIELD_ALT);
        assert_eq!(decode_position(&lean[..ln]).unwrap().alt_dm, 0);
        assert_eq!(decode_position(&rich[..rn]).unwrap().alt_dm, 1234);
    }

    /// 0x50 was the old fixed 20-byte layout. A frame from that firmware
    /// must be refused, not read as a mask plus fields.
    #[test]
    fn old_position_tag_is_not_decoded() {
        let mut old = [0u8; 21];
        old[0] = 0x50;
        old[1..].copy_from_slice(&sample().encode());
        assert_eq!(decode_position(&old), None);
    }

    #[test]
    fn ping_roundtrip() {
        let p = Ping { uptime_s: 4321, gps_present: true, had_fix: false };
        let enc = p.encode();
        assert_eq!(enc.len(), PING_MSG_LEN);
        assert_eq!(Ping::decode(&enc), Some(p));

        // Each flag travels on its own bit.
        let none = Ping { uptime_s: 0, gps_present: false, had_fix: false };
        assert_eq!(none.flags(), 0);
        assert_eq!(Ping::decode(&none.encode()), Some(none));
        let both = Ping { uptime_s: u16::MAX, gps_present: true, had_fix: true };
        assert_eq!(Ping::decode(&both.encode()), Some(both));

        // Set bits this build does not know are ignored, not a rejection.
        let mut future = both.encode();
        future[1] |= 0x80;
        assert_eq!(Ping::decode(&future), Some(both));
    }

    /// The two payload kinds must never decode as each other: a receiver
    /// tries both, and a position read as a ping would report an alive node
    /// with no position when it had one.
    #[test]
    fn ping_and_position_do_not_alias() {
        let (pos, n) = encode_position(&sample(), FIELDS_DEFAULT);
        assert_eq!(Ping::decode(&pos[..n]), None);
        assert_eq!(decode_position(&Ping::default().encode()), None);
        // Runts and other payloads are refused rather than read short.
        assert_eq!(Ping::decode(&Ping::default().encode()[..3]), None);
        assert_eq!(Ping::decode(b""), None);
        assert_eq!(Ping::decode(b"hello"), None);
    }

    /// A ping is cheaper on the air than the leanest position, which is what
    /// makes reporting a missing fix free of a duty-cycle argument.
    #[test]
    fn ping_is_smaller_than_a_position() {
        assert!(PING_MSG_LEN < position_msg_len(FIELDS_REQUIRED));
    }

    #[test]
    fn frame_roundtrip() {
        let (payload, plen) = encode_position(&sample(), FIELDS_ALL);
        let payload = &payload[..plen];
        let frame = Frame {
            src: 3,
            id: 42,
            hops_left: 1,
            payload,
        };
        let mut buf = [0u8; FRAME_MAX];
        let n = frame.encode(&mut buf).unwrap();
        // 3 header + 21 payload; no padding to a fixed size.
        assert_eq!(n, 24);
        assert_eq!(Frame::decode(&buf[..n]), Some(frame));
    }

    #[test]
    fn frame_rejects_malformed() {
        let mut buf = [0u8; FRAME_MAX];
        // Header only, no payload.
        assert_eq!(Frame::decode(&[1, 2, 3]), None);
        assert_eq!(Frame::decode(&[1, 2]), None);
        assert_eq!(Frame::decode(&[]), None);
        // Source address 0 is not assignable.
        assert_eq!(Frame::decode(&[0, 1, 1, 0x50]), None);
        // Empty and oversized payloads do not encode.
        let empty = Frame { src: 1, id: 0, hops_left: 0, payload: &[] };
        assert_eq!(empty.encode(&mut buf), None);
        let big = [0u8; PAYLOAD_MAX + 1];
        let over = Frame { src: 1, id: 0, hops_left: 0, payload: &big };
        assert_eq!(over.encode(&mut buf), None);
        // Exactly full fits.
        let full = Frame { src: 1, id: 0, hops_left: 0, payload: &big[..PAYLOAD_MAX] };
        assert_eq!(full.encode(&mut buf), Some(FRAME_MAX));
    }

    #[test]
    fn hops_survive_a_repeat() {
        let (payload, plen) = encode_position(&sample(), FIELDS_ALL);
        let mut buf = [0u8; FRAME_MAX];
        let n = Frame { src: 7, id: 9, hops_left: 2, payload: &payload[..plen] }
            .encode(&mut buf)
            .unwrap();

        // What a repeater does: decode, decrement, re-encode. The origin
        // address and id must survive so the next hop still dedups on them.
        let recv = Frame::decode(&buf[..n]).unwrap();
        let mut out = [0u8; FRAME_MAX];
        let n2 = Frame { hops_left: recv.hops_left - 1, ..recv }
            .encode(&mut out)
            .unwrap();
        let hop2 = Frame::decode(&out[..n2]).unwrap();
        assert_eq!((hop2.src, hop2.id, hop2.hops_left), (7, 9, 1));
        assert_eq!(decode_position(hop2.payload), Some(sample()));
    }
}
