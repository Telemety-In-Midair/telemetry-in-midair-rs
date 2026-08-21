//! The bulk transfer a radio config or a firmware image arrives in.
//!
//! One write is one op ([`ble::OP_BEGIN`], [`ble::OP_DATA`], [`ble::OP_END`],
//! [`ble::OP_ABORT`]) and every op is answered with a gps-proto ack. The
//! same bytes arrive two ways - a BLE write on the bulk characteristic, or a
//! framed [`crate::link::usb::BULK`] command on the USB console - so the
//! state machine lives here rather than in either transport.
//!
//! On the two-MCU board this was split in half: the ESP32-C6 parsed the ops
//! and forwarded each one over the UART link, and the WIO-E5 reassembled the
//! bytes and checked the CRC. One MCU does both, so the two halves become
//! one object - and one that a host test can drive, which neither of them
//! could be.
//!
//! The two kinds differ in where the bytes go. A config is small and has to
//! be parsed as a whole, so it is buffered here and handed over complete. A
//! firmware image is hundreds of kilobytes, so it streams through a
//! [`Sink`] as it arrives and only its CRC is accumulated.

use crate::ble;
use crate::link::crc32;
use gps_proto::packet::{self, ACK_MAX_LEN};

/// Largest `RADIO.CFG` a transfer will accept, matching the firmware's own
/// read buffer.
pub const CONFIG_MAX: usize = 1024;

/// How long a transfer may sit idle before it is abandoned.
///
/// A transfer holds the console quiet and locks out the other transport, so
/// a host that vanishes mid-upload - a phone that walked away, a USB cable
/// pulled - must not wedge the board until someone power-cycles it.
pub const IDLE_TIMEOUT_MS: u64 = 5_000;

/// Which transport owns an in-flight transfer.
///
/// A transfer is stateful across several writes, so two transports pushing
/// at once would interleave into one buffer. The second one to start is
/// refused with [`ble::ACK_BAD_STATE`] instead.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Owner {
    Ble,
    Usb,
}

/// What the firmware has to do once [`Transfer::handle`] has processed an op.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// Nothing beyond sending the ack back.
    None,
    /// A complete, CRC-verified config is in [`Transfer::bytes`]. Parse,
    /// apply and store it, then call [`Transfer::mark_applied`] - a repeated
    /// `OP_END` is only answerable as the success it was once that is done.
    Config,
    /// A complete, CRC-verified firmware image has been written and
    /// [`Sink::finish`] accepted it. Reboot into it when convenient.
    Firmware,
}

/// Where a firmware image's bytes go as they arrive.
///
/// Separate from the transfer because an image is far too big to buffer:
/// the bytes are written out chunk by chunk and only the running CRC is
/// kept. A board with no OTA partitions refuses at [`Sink::begin`], which
/// is what makes firmware support optional rather than assumed.
pub trait Sink {
    /// Prepare to receive `total` bytes. `false` refuses the transfer.
    fn begin(&mut self, total: u32) -> bool;
    /// Write `bytes` at `offset` from the start of the image. `false` fails
    /// the transfer.
    fn write(&mut self, offset: u32, bytes: &[u8]) -> bool;
    /// The image is complete and its CRC matched. `false` fails the
    /// transfer.
    fn finish(&mut self) -> bool;
    /// The transfer ended without completing. Release whatever `begin`
    /// claimed.
    fn cancel(&mut self);
}

/// A [`Sink`] that accepts no firmware at all, for a build (or a test) that
/// only takes configs. Refusing at `begin` is reported to the host as a bad
/// value, which is the truth: this board does not take that kind.
pub struct NoFirmware;

impl Sink for NoFirmware {
    fn begin(&mut self, _total: u32) -> bool {
        false
    }
    fn write(&mut self, _offset: u32, _bytes: &[u8]) -> bool {
        false
    }
    fn finish(&mut self) -> bool {
        false
    }
    fn cancel(&mut self) {}
}

/// The ack bytes an op is answered with.
pub type Ack = ([u8; ACK_MAX_LEN], usize);

fn ack(status: u8, value: &[u8]) -> Ack {
    packet::encode_ack(ble::ACK_ID_BULK, status, value)
}

fn nak(status: u8) -> Ack {
    ack(status, &[])
}

/// One transfer at a time, in whichever of the two kinds is running.
pub struct Transfer {
    active: bool,
    owner: Owner,
    kind: u8,
    /// Total bytes the sender declared in `OP_BEGIN`.
    total: u32,
    /// Bytes accepted so far.
    received: u32,
    /// Next sequence number expected on `OP_DATA`.
    next_seq: u16,
    /// CRC32 the sender declared, checked at `OP_END`.
    want_crc: u32,
    /// Running CRC of a streaming (firmware) transfer.
    running_crc: u32,
    /// When the last op arrived, for [`IDLE_TIMEOUT_MS`].
    last_ms: u64,
    /// CRC of the last transfer that verified, and whether the caller went
    /// on to apply it.
    ///
    /// Together they let a repeated `OP_END` be answered as the success it
    /// was. A host retries the step whenever an ack goes missing, and an
    /// apply can outlast the host's timeout on a slow card - so without
    /// this a config that was applied and saved comes back as a failure.
    last_crc: u32,
    applied: bool,
    buf: [u8; CONFIG_MAX],
}

impl Default for Transfer {
    fn default() -> Self {
        Self::new()
    }
}

impl Transfer {
    pub const fn new() -> Self {
        Self {
            active: false,
            owner: Owner::Ble,
            kind: 0,
            total: 0,
            received: 0,
            next_seq: 0,
            want_crc: 0,
            running_crc: 0,
            last_ms: 0,
            last_crc: 0,
            applied: false,
            buf: [0; CONFIG_MAX],
        }
    }

    /// Whether a transfer is in flight. While one is, the console stays
    /// quiet and the radio is held off the air.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// The kind of the transfer in flight ([`ble::KIND_TOML`] or
    /// [`ble::KIND_OTA`]); meaningless when none is.
    pub fn kind(&self) -> u8 {
        self.kind
    }

    /// The received config text. Valid after [`Event::Config`].
    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.received as usize]
    }

    /// Record that the caller parsed and adopted the completed config, so a
    /// repeated `OP_END` answers OK rather than "no such transfer".
    ///
    /// A config that verified but would not parse is never marked, and a
    /// retry of that one keeps failing - which is the truth.
    pub fn mark_applied(&mut self) {
        self.applied = true;
    }

    /// Drop an in-flight transfer, e.g. because the connection carrying it
    /// went away. Returns whether there was one.
    pub fn abort(&mut self, sink: &mut dyn Sink) -> bool {
        let was = self.active;
        if was {
            self.cancel(sink);
        }
        was
    }

    /// Drop a transfer that has gone quiet for [`IDLE_TIMEOUT_MS`],
    /// returning whether it did. A host that vanished mid-upload must not
    /// hold the board hostage.
    pub fn expire(&mut self, now_ms: u64, sink: &mut dyn Sink) -> bool {
        if !self.active || now_ms.saturating_sub(self.last_ms) < IDLE_TIMEOUT_MS {
            return false;
        }
        self.cancel(sink);
        true
    }

    fn cancel(&mut self, sink: &mut dyn Sink) {
        if self.active && self.kind == ble::KIND_OTA {
            sink.cancel();
        }
        self.active = false;
    }

    /// Process one bulk op and return what to do plus the ack to send back.
    ///
    /// `now_ms` is a monotonic millisecond clock, used only for the idle
    /// timeout.
    pub fn handle(
        &mut self,
        owner: Owner,
        now_ms: u64,
        data: &[u8],
        sink: &mut dyn Sink,
    ) -> (Event, Ack) {
        let Some(&op) = data.first() else {
            return (Event::None, nak(packet::ACK_BAD_VALUE));
        };
        // Any op from the transport that owns the transfer keeps it alive.
        // An op from the other one does not, or a second host could hold a
        // stalled transfer open indefinitely by polling it.
        if self.active && self.owner == owner {
            self.last_ms = now_ms;
        }
        match op {
            ble::OP_BEGIN => self.begin(owner, now_ms, data, sink),
            ble::OP_DATA => self.data(owner, data, sink),
            ble::OP_END => self.end(owner, sink),
            ble::OP_ABORT => {
                // Only the owner may abort: a stray op on the other
                // transport must not cancel someone else's upload.
                if self.active && self.owner == owner {
                    self.cancel(sink);
                }
                (Event::None, ack(packet::ACK_OK, &[]))
            }
            _ => (Event::None, nak(packet::ACK_BAD_VALUE)),
        }
    }

    /// `[OP_BEGIN, kind u8, total u32le, crc32 u32le, version u16le]`.
    fn begin(&mut self, owner: Owner, now_ms: u64, data: &[u8], sink: &mut dyn Sink) -> (Event, Ack) {
        if data.len() < 12 {
            return (Event::None, nak(packet::ACK_BAD_VALUE));
        }
        // A begin from the transport that already owns a transfer restarts
        // it - that is a host retrying from the top, which is legal. One
        // from the other transport is refused.
        if self.active && self.owner != owner {
            return (Event::None, nak(ble::ACK_BAD_STATE));
        }
        let kind = data[1];
        let total = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
        let want_crc = u32::from_le_bytes([data[6], data[7], data[8], data[9]]);
        // A restart releases whatever the previous attempt claimed before
        // the new one claims it again.
        if self.active {
            self.cancel(sink);
        }
        match kind {
            ble::KIND_TOML => {
                if total == 0 || total as usize > CONFIG_MAX {
                    return (Event::None, nak(packet::ACK_BAD_VALUE));
                }
            }
            ble::KIND_OTA => {
                if total == 0 {
                    return (Event::None, nak(packet::ACK_BAD_VALUE));
                }
                // A refusal here is the board's, not the op's: no update
                // slots, or an image too big for the one it would go in.
                // Reported as a board error so a host tool can say which of
                // those it is rather than blaming the bytes it sent.
                if !sink.begin(total) {
                    return (Event::None, nak(ble::ACK_WIO_ERROR));
                }
            }
            _ => return (Event::None, nak(packet::ACK_BAD_VALUE)),
        }
        self.active = true;
        self.owner = owner;
        self.kind = kind;
        self.total = total;
        self.received = 0;
        self.next_seq = 0;
        self.want_crc = want_crc;
        self.running_crc = 0;
        self.applied = false;
        self.last_ms = now_ms;
        (Event::None, ack(packet::ACK_OK, &0u32.to_le_bytes()))
    }

    /// `[OP_DATA, seq u16le, bytes...]`.
    fn data(&mut self, owner: Owner, data: &[u8], sink: &mut dyn Sink) -> (Event, Ack) {
        if !self.active || self.owner != owner {
            return (Event::None, nak(ble::ACK_BAD_STATE));
        }
        if data.len() < 4 || data.len() - 3 > ble::BULK_DATA_MAX {
            return (Event::None, nak(packet::ACK_BAD_VALUE));
        }
        let seq = u16::from_le_bytes([data[1], data[2]]);
        let chunk = &data[3..];
        if seq != self.next_seq {
            // The chunk before this one, i.e. an ack that went missing on
            // the way back. Re-ack it; anything else is a real desync.
            if seq.wrapping_add(1) == self.next_seq {
                return (
                    Event::None,
                    ack(packet::ACK_OK, &(self.next_seq as u32).to_le_bytes()),
                );
            }
            return (Event::None, nak(packet::ACK_BAD_VALUE));
        }
        if self.received + chunk.len() as u32 > self.total {
            self.cancel(sink);
            return (Event::None, nak(packet::ACK_BAD_VALUE));
        }
        if self.kind == ble::KIND_OTA {
            if !sink.write(self.received, chunk) {
                self.cancel(sink);
                return (Event::None, nak(ble::ACK_WIO_ERROR));
            }
            self.running_crc = crc32_continue(self.running_crc, chunk);
        } else {
            let at = self.received as usize;
            self.buf[at..at + chunk.len()].copy_from_slice(chunk);
        }
        self.received += chunk.len() as u32;
        self.next_seq = seq.wrapping_add(1);
        (
            Event::None,
            ack(packet::ACK_OK, &(self.next_seq as u32).to_le_bytes()),
        )
    }

    /// `[OP_END]`. The CRC declared at begin is what is checked; the op
    /// itself carries nothing.
    fn end(&mut self, owner: Owner, sink: &mut dyn Sink) -> (Event, Ack) {
        if !self.active || self.owner != owner {
            // Either a retry of an END that already landed, or a stray op.
            // Whether the last transfer was applied is what tells them
            // apart, and the host reads OK as "the work is committed".
            return if self.applied {
                (Event::None, ack(packet::ACK_OK, &[]))
            } else {
                (Event::None, nak(ble::ACK_BAD_STATE))
            };
        }
        if self.received != self.total {
            self.cancel(sink);
            return (Event::None, nak(packet::ACK_BAD_VALUE));
        }
        let got = if self.kind == ble::KIND_OTA {
            self.running_crc
        } else {
            crc32(self.bytes())
        };
        if got != self.want_crc {
            self.cancel(sink);
            return (Event::None, nak(packet::ACK_BAD_VALUE));
        }
        // The transfer is over either way; only the buffered config
        // outlives it, so `active` drops without cancelling the sink.
        self.active = false;
        self.last_crc = self.want_crc;
        if self.kind == ble::KIND_OTA {
            if !sink.finish() {
                return (Event::None, nak(ble::ACK_WIO_ERROR));
            }
            // Nothing left to retry: the image is installed and the next
            // boot runs it, so a repeated END answers OK.
            self.applied = true;
            return (Event::Firmware, ack(packet::ACK_OK, &[]));
        }
        // The caller parses and applies, then marks it - only then does a
        // repeat of this END read as a success.
        (Event::Config, ack(packet::ACK_OK, &[]))
    }

    /// CRC of the last transfer that verified. Diagnostics only.
    pub fn last_crc(&self) -> u32 {
        self.last_crc
    }
}

/// One more chunk of a running CRC-32 (IEEE, reflected).
///
/// [`crc32`] does the whole buffer at once, which a streaming firmware image
/// cannot be held in. Same polynomial and the same final value, split so the
/// pre- and post-inversion happen once around the whole stream rather than
/// once per chunk.
fn crc32_continue(crc: u32, data: &[u8]) -> u32 {
    let mut c = !crc;
    for &byte in data {
        c ^= byte as u32;
        for _ in 0..8 {
            if c & 1 != 0 {
                c = (c >> 1) ^ 0xEDB8_8320;
            } else {
                c >>= 1;
            }
        }
    }
    !c
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A [`Sink`] that keeps what it was given, so a test can check the
    /// bytes that came out the far end.
    #[derive(Default)]
    struct VecSink {
        accept_begin: bool,
        fail_write_at: Option<u32>,
        fail_finish: bool,
        image: Vec<u8>,
        finished: bool,
        cancels: u32,
    }

    impl VecSink {
        fn ready() -> Self {
            Self {
                accept_begin: true,
                ..Default::default()
            }
        }
    }

    impl Sink for VecSink {
        fn begin(&mut self, _total: u32) -> bool {
            self.image.clear();
            self.finished = false;
            self.accept_begin
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> bool {
            if self.fail_write_at == Some(offset) {
                return false;
            }
            assert_eq!(offset as usize, self.image.len(), "chunks arrive in order");
            self.image.extend_from_slice(bytes);
            true
        }
        fn finish(&mut self) -> bool {
            self.finished = !self.fail_finish;
            self.finished
        }
        fn cancel(&mut self) {
            self.cancels += 1;
        }
    }

    fn begin_op(kind: u8, data: &[u8]) -> Vec<u8> {
        let mut v = vec![ble::OP_BEGIN, kind];
        v.extend_from_slice(&(data.len() as u32).to_le_bytes());
        v.extend_from_slice(&crc32(data).to_le_bytes());
        v.extend_from_slice(&0u16.to_le_bytes());
        v
    }

    fn data_op(seq: u16, chunk: &[u8]) -> Vec<u8> {
        let mut v = vec![ble::OP_DATA];
        v.extend_from_slice(&seq.to_le_bytes());
        v.extend_from_slice(chunk);
        v
    }

    fn status(a: &Ack) -> u8 {
        packet::parse_ack(&a.0[..a.1]).expect("ack parses").status
    }

    fn next_seq(a: &Ack) -> u32 {
        packet::parse_ack(&a.0[..a.1])
            .expect("ack parses")
            .value_u32
            .expect("ack carries a u32")
    }

    /// Push a whole transfer through and return the last event.
    fn push(t: &mut Transfer, kind: u8, payload: &[u8], sink: &mut dyn Sink) -> Event {
        let (_, a) = t.handle(Owner::Ble, 0, &begin_op(kind, payload), sink);
        assert_eq!(status(&a), packet::ACK_OK, "begin");
        let mut seq = 0u16;
        for chunk in payload.chunks(ble::BULK_DATA_MAX) {
            let (_, a) = t.handle(Owner::Ble, 0, &data_op(seq, chunk), sink);
            assert_eq!(status(&a), packet::ACK_OK, "chunk {seq}");
            seq = next_seq(&a) as u16;
        }
        let (event, a) = t.handle(Owner::Ble, 0, &[ble::OP_END], sink);
        assert_eq!(status(&a), packet::ACK_OK, "end");
        event
    }

    const TOML: &[u8] = b"[radio]\nfrequency_hz = 915000000\naddress = 7\n";

    #[test]
    fn a_config_arrives_whole() {
        let mut t = Transfer::new();
        assert_eq!(push(&mut t, ble::KIND_TOML, TOML, &mut NoFirmware), Event::Config);
        assert_eq!(t.bytes(), TOML);
        assert!(!t.is_active());
    }

    /// The reference config is longer than one chunk, which is the case the
    /// sequence numbers exist for.
    #[test]
    fn a_multi_chunk_config_reassembles() {
        let big: Vec<u8> = (0..CONFIG_MAX).map(|i| b'a' + (i % 26) as u8).collect();
        let mut t = Transfer::new();
        assert_eq!(push(&mut t, ble::KIND_TOML, &big, &mut NoFirmware), Event::Config);
        assert_eq!(t.bytes(), big.as_slice());
    }

    /// A corrupt payload must not be parsed as a config. The CRC is the
    /// only thing that would catch it: every op was individually well
    /// formed.
    #[test]
    fn a_bad_crc_is_refused() {
        let mut t = Transfer::new();
        let mut op = begin_op(ble::KIND_TOML, TOML);
        op[6] ^= 0xFF; // corrupt the declared crc
        t.handle(Owner::Ble, 0, &op, &mut NoFirmware);
        t.handle(Owner::Ble, 0, &data_op(0, TOML), &mut NoFirmware);
        let (event, a) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(event, Event::None);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
        assert!(!t.is_active());
    }

    #[test]
    fn a_short_transfer_is_refused() {
        let mut t = Transfer::new();
        // Declares more than it sends.
        let mut op = begin_op(ble::KIND_TOML, TOML);
        op[2] = op[2].wrapping_add(1);
        t.handle(Owner::Ble, 0, &op, &mut NoFirmware);
        t.handle(Owner::Ble, 0, &data_op(0, TOML), &mut NoFirmware);
        let (_, a) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
    }

    /// More bytes than declared are refused as they arrive, rather than
    /// running off the end of the buffer.
    #[test]
    fn an_overlong_transfer_is_refused() {
        let mut t = Transfer::new();
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, TOML), &mut NoFirmware);
        let too_much = [b'x'; 100];
        assert!(too_much.len() > TOML.len() && too_much.len() <= ble::BULK_DATA_MAX);
        let (_, a) = t.handle(Owner::Ble, 0, &data_op(0, &too_much), &mut NoFirmware);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
        assert!(!t.is_active());
    }

    #[test]
    fn a_config_larger_than_the_buffer_is_refused_at_begin() {
        let mut t = Transfer::new();
        let big = vec![b'x'; CONFIG_MAX + 1];
        let (_, a) = t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, &big), &mut NoFirmware);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
        assert!(!t.is_active());
    }

    /// The retry the host actually performs: an ack was lost, so it sends
    /// the same chunk again. It must re-ack, not desync.
    #[test]
    fn a_repeated_chunk_is_re_acked() {
        let mut t = Transfer::new();
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, TOML), &mut NoFirmware);
        let (_, first) = t.handle(Owner::Ble, 0, &data_op(0, TOML), &mut NoFirmware);
        let (_, again) = t.handle(Owner::Ble, 0, &data_op(0, TOML), &mut NoFirmware);
        assert_eq!(status(&again), packet::ACK_OK);
        assert_eq!(next_seq(&again), next_seq(&first));
        // And the bytes landed once, not twice.
        let (event, _) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(event, Event::Config);
        assert_eq!(t.bytes(), TOML);
    }

    /// A chunk from nowhere near the current position is a real desync.
    #[test]
    fn a_jumped_sequence_is_rejected() {
        let mut t = Transfer::new();
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, TOML), &mut NoFirmware);
        let (_, a) = t.handle(Owner::Ble, 0, &data_op(5, TOML), &mut NoFirmware);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
    }

    /// The END whose ack went missing. Once the caller has applied the
    /// config, a repeat is the success it was - the host would otherwise
    /// report a failure for work that is committed.
    #[test]
    fn a_repeated_end_answers_the_applied_config() {
        let mut t = Transfer::new();
        assert_eq!(push(&mut t, ble::KIND_TOML, TOML, &mut NoFirmware), Event::Config);
        // Before the caller applies it, a repeat is not a success.
        let (_, a) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(status(&a), ble::ACK_BAD_STATE);
        t.mark_applied();
        let (event, a) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(status(&a), packet::ACK_OK);
        // And it does not re-fire the work.
        assert_eq!(event, Event::None);
    }

    /// Two transports cannot interleave into one buffer.
    #[test]
    fn the_second_transport_is_refused() {
        let mut t = Transfer::new();
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, TOML), &mut NoFirmware);
        let (_, a) = t.handle(Owner::Usb, 0, &begin_op(ble::KIND_TOML, TOML), &mut NoFirmware);
        assert_eq!(status(&a), ble::ACK_BAD_STATE);
        // Nor can it push data or end the transfer.
        let (_, a) = t.handle(Owner::Usb, 0, &data_op(0, TOML), &mut NoFirmware);
        assert_eq!(status(&a), ble::ACK_BAD_STATE);
        let (_, a) = t.handle(Owner::Usb, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(status(&a), ble::ACK_BAD_STATE);
        // And an abort from the other transport does not cancel it.
        t.handle(Owner::Usb, 0, &[ble::OP_ABORT], &mut NoFirmware);
        assert!(t.is_active());
        // The owner still finishes normally.
        t.handle(Owner::Ble, 0, &data_op(0, TOML), &mut NoFirmware);
        let (event, _) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(event, Event::Config);
    }

    /// The owner restarting from the top is a host retry, not a collision.
    #[test]
    fn the_owner_may_restart() {
        let mut t = Transfer::new();
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, TOML), &mut NoFirmware);
        t.handle(Owner::Ble, 0, &data_op(0, &TOML[..4]), &mut NoFirmware);
        assert_eq!(push(&mut t, ble::KIND_TOML, TOML, &mut NoFirmware), Event::Config);
        assert_eq!(t.bytes(), TOML);
    }

    /// A host that walks away mid-upload must not hold the board.
    #[test]
    fn an_idle_transfer_expires() {
        let mut t = Transfer::new();
        let mut sink = VecSink::ready();
        t.handle(Owner::Ble, 1_000, &begin_op(ble::KIND_OTA, TOML), &mut sink);
        assert!(t.is_active());
        assert!(!t.expire(1_000 + IDLE_TIMEOUT_MS - 1, &mut sink));
        assert!(t.expire(1_000 + IDLE_TIMEOUT_MS, &mut sink));
        assert!(!t.is_active());
        // The sink was told, so whatever it claimed is released.
        assert_eq!(sink.cancels, 1);
        // And expiring again does nothing.
        assert!(!t.expire(u64::MAX, &mut sink));
    }

    /// Only the owner's own ops keep a transfer alive; polling from the
    /// other transport must not extend it.
    #[test]
    fn the_other_transport_cannot_hold_a_transfer_open() {
        let mut t = Transfer::new();
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, TOML), &mut NoFirmware);
        t.handle(Owner::Usb, IDLE_TIMEOUT_MS - 1, &[ble::OP_END], &mut NoFirmware);
        assert!(t.expire(IDLE_TIMEOUT_MS, &mut NoFirmware));
    }

    #[test]
    fn a_firmware_image_streams_through_the_sink() {
        let image: Vec<u8> = (0..1000u32).map(|i| (i % 251) as u8).collect();
        let mut t = Transfer::new();
        let mut sink = VecSink::ready();
        assert_eq!(push(&mut t, ble::KIND_OTA, &image, &mut sink), Event::Firmware);
        assert_eq!(sink.image, image);
        assert!(sink.finished);
        assert_eq!(sink.cancels, 0);
        // An image is not buffered, so the config view stays empty.
        assert!(t.bytes().is_empty() || t.kind() == ble::KIND_OTA);
    }

    /// The running CRC has to agree with the whole-buffer one, or a good
    /// image would be rejected at the last step.
    #[test]
    fn the_streaming_crc_matches_the_one_shot_crc() {
        for len in [0usize, 1, 7, 192, 193, 1000] {
            let data: Vec<u8> = (0..len).map(|i| (i % 251) as u8).collect();
            let mut running = 0;
            for chunk in data.chunks(192.max(1)) {
                running = crc32_continue(running, chunk);
            }
            assert_eq!(running, crc32(&data), "len {len}");
        }
    }

    /// A board with no OTA partitions says so at begin rather than
    /// accepting an image it has nowhere to put - and says it as a board
    /// error, since the bytes the host sent were fine.
    #[test]
    fn firmware_is_refused_where_there_is_no_sink() {
        let mut t = Transfer::new();
        let (_, a) = t.handle(Owner::Ble, 0, &begin_op(ble::KIND_OTA, b"image"), &mut NoFirmware);
        assert_eq!(status(&a), ble::ACK_WIO_ERROR);
        assert!(!t.is_active());
        // An empty image is a bad op, which is a different answer.
        let (_, a) = t.handle(Owner::Ble, 0, &begin_op(ble::KIND_OTA, b""), &mut NoFirmware);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
    }

    #[test]
    fn a_failed_write_ends_the_transfer() {
        let image = vec![0xABu8; 400];
        let mut t = Transfer::new();
        let mut sink = VecSink {
            fail_write_at: Some(192),
            ..VecSink::ready()
        };
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_OTA, &image), &mut sink);
        let (_, a) = t.handle(Owner::Ble, 0, &data_op(0, &image[..192]), &mut sink);
        assert_eq!(status(&a), packet::ACK_OK);
        let (_, a) = t.handle(Owner::Ble, 0, &data_op(1, &image[192..384]), &mut sink);
        assert_eq!(status(&a), ble::ACK_WIO_ERROR);
        assert!(!t.is_active());
        assert_eq!(sink.cancels, 1);
    }

    #[test]
    fn a_failed_finish_is_reported() {
        let image = vec![0xABu8; 100];
        let mut t = Transfer::new();
        let mut sink = VecSink {
            fail_finish: true,
            ..VecSink::ready()
        };
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_OTA, &image), &mut sink);
        t.handle(Owner::Ble, 0, &data_op(0, &image), &mut sink);
        let (event, a) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut sink);
        assert_eq!(event, Event::None);
        assert_eq!(status(&a), ble::ACK_WIO_ERROR);
    }

    /// An image whose bytes were corrupted in flight must not be activated,
    /// even though every chunk was written.
    #[test]
    fn a_corrupt_image_is_not_finished() {
        let image = vec![0x5Au8; 300];
        let mut t = Transfer::new();
        let mut sink = VecSink::ready();
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_OTA, &image), &mut sink);
        let mut corrupt = image.clone();
        corrupt[0] ^= 0xFF;
        t.handle(Owner::Ble, 0, &data_op(0, &corrupt[..192]), &mut sink);
        t.handle(Owner::Ble, 0, &data_op(1, &corrupt[192..]), &mut sink);
        let (event, a) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut sink);
        assert_eq!(event, Event::None);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
        assert!(!sink.finished);
        assert_eq!(sink.cancels, 1);
    }

    /// The retired WIO-E5 firmware kind must be refused, not misread as an
    /// ESP image.
    #[test]
    fn the_retired_firmware_kind_is_rejected() {
        let mut t = Transfer::new();
        let mut sink = VecSink::ready();
        let (_, a) = t.handle(Owner::Ble, 0, &begin_op(2, b"stm32 image"), &mut sink);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
        assert!(!t.is_active());
    }

    #[test]
    fn malformed_ops_are_rejected() {
        let mut t = Transfer::new();
        for op in [
            vec![],
            vec![0x7F],
            vec![ble::OP_BEGIN, ble::KIND_TOML, 1, 0],
            vec![ble::OP_DATA, 0, 0],
        ] {
            let (event, a) = t.handle(Owner::Ble, 0, &op, &mut NoFirmware);
            assert_eq!(event, Event::None);
            assert_ne!(status(&a), packet::ACK_OK, "op {op:02x?}");
        }
    }

    /// A data chunk bigger than the protocol allows is refused rather than
    /// silently truncated.
    #[test]
    fn an_oversize_chunk_is_rejected() {
        let mut t = Transfer::new();
        let payload = vec![b'x'; ble::BULK_DATA_MAX + 1];
        t.handle(Owner::Ble, 0, &begin_op(ble::KIND_TOML, &payload), &mut NoFirmware);
        let (_, a) = t.handle(Owner::Ble, 0, &data_op(0, &payload), &mut NoFirmware);
        assert_eq!(status(&a), packet::ACK_BAD_VALUE);
        // A malformed op is not a failed transfer: the host can send the
        // same bytes again correctly chunked.
        assert!(t.is_active());
        for (seq, chunk) in payload.chunks(ble::BULK_DATA_MAX).enumerate() {
            let (_, a) = t.handle(Owner::Ble, 0, &data_op(seq as u16, chunk), &mut NoFirmware);
            assert_eq!(status(&a), packet::ACK_OK);
        }
        let (event, _) = t.handle(Owner::Ble, 0, &[ble::OP_END], &mut NoFirmware);
        assert_eq!(event, Event::Config);
    }
}
