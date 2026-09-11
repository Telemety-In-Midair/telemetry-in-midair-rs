//! The event log: what the board writes down about itself, in its own
//! flash, so a reset is not the end of the story.
//!
//! A board that stops in a field has no console attached. What it can
//! keep is a short line per thing worth knowing - why it booted, what it
//! panicked on, which task stalled and in what phase, a radio that
//! restarted underneath the firmware, a BLE controller that would not
//! come up - in a ring of fixed records that a host tool reads back over
//! USB and the boot prints the tail of.
//!
//! ```text
//! offset  size  field
//! 0       2     magic
//! 2       1     kind
//! 3       1     length of the text
//! 4       4     sequence number, one up per record across every boot
//! 8       4     uptime when written, seconds
//! 12      2     boot number, one up per boot
//! 14      2     reserved, written as zero
//! 16      108   the text, zero-padded
//! 124     4     crc32 of the 124 bytes before it
//! ```
//!
//! The ring ([`Ring`]) is append-only in erased flash: a record is one
//! program of its slot, never a read-modify-write of the sector around
//! it, and a sector is erased once, as the ring enters it, which takes the
//! oldest records with it. The head is found at boot by the highest
//! sequence number, and a slot that is not blank when the ring reaches it
//! - a program that was interrupted - is skipped rather than written
//! over, since flash bits only clear and a second program on top of a
//! partial one is a record no crc will pass.

use crate::link::crc32;

/// One record in flash. A multiple of the flash word, and small enough
/// that a USB frame carries one whole.
pub const RECORD_LEN: usize = 128;
/// magic, kind, length, seq, uptime, boot, reserved.
pub const HEADER_LEN: usize = 16;
/// The crc, at the end.
const CRC_LEN: usize = 4;
/// Longest text a record carries.
pub const TEXT_MAX: usize = RECORD_LEN - HEADER_LEN - CRC_LEN;

/// Marks a slot as a record ("EL"). Erased flash reads 0xFFFF there,
/// which is also how a blank slot is told from a record.
pub const MAGIC: u16 = 0x4C45;

/// What a record is about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Kind {
    /// A boot: the reset reason, and what the boot before it left behind.
    Boot = 1,
    /// A panic, with its message and location. Written at the boot after
    /// it, from the copy the panic handler left in RTC RAM.
    Panic = 2,
    /// A task past its heartbeat bound: the task and the phase it stopped
    /// in. Written at the boot after the reset, from RTC RAM.
    Stall = 3,
    /// The BLE stack: a controller that would not init, a host error, a
    /// restart.
    Ble = 4,
    /// The LoRa radio: a chip that restarted underneath the firmware, or
    /// one that does not answer.
    Radio = 5,
    /// The GPS: a receiver that went silent, or a park that did not hold.
    Gps = 6,
    /// Deep sleep: a park that did not finish before the chip went down.
    Sleep = 7,
    /// A bulk transfer: one abandoned, one rejected, an image installed.
    Transfer = 8,
    /// Anything else worth a line.
    Note = 9,
}

impl Kind {
    pub const ALL: [Kind; 9] = [
        Kind::Boot,
        Kind::Panic,
        Kind::Stall,
        Kind::Ble,
        Kind::Radio,
        Kind::Gps,
        Kind::Sleep,
        Kind::Transfer,
        Kind::Note,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Kind::Boot => "boot",
            Kind::Panic => "panic",
            Kind::Stall => "stall",
            Kind::Ble => "ble",
            Kind::Radio => "radio",
            Kind::Gps => "gps",
            Kind::Sleep => "sleep",
            Kind::Transfer => "transfer",
            Kind::Note => "note",
        }
    }

    pub const fn as_wire(self) -> u8 {
        self as u8
    }

    pub const fn from_wire(v: u8) -> Option<Self> {
        Some(match v {
            1 => Kind::Boot,
            2 => Kind::Panic,
            3 => Kind::Stall,
            4 => Kind::Ble,
            5 => Kind::Radio,
            6 => Kind::Gps,
            7 => Kind::Sleep,
            8 => Kind::Transfer,
            9 => Kind::Note,
            _ => return None,
        })
    }
}

/// One event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Record {
    pub kind: Kind,
    pub seq: u32,
    pub uptime_s: u32,
    pub boot: u16,
    text: [u8; TEXT_MAX],
    len: u8,
}

impl Record {
    /// A record carrying `text`, truncated to [`TEXT_MAX`] and cut at a
    /// character boundary so what is stored is still a string.
    pub fn new(kind: Kind, seq: u32, uptime_s: u32, boot: u16, text: &str) -> Self {
        let mut buf = [0u8; TEXT_MAX];
        let mut len = text.len().min(TEXT_MAX);
        while !text.is_char_boundary(len) {
            len -= 1;
        }
        buf[..len].copy_from_slice(&text.as_bytes()[..len]);
        Self {
            kind,
            seq,
            uptime_s,
            boot,
            text: buf,
            len: len as u8,
        }
    }

    /// The text, as written.
    pub fn text(&self) -> &str {
        core::str::from_utf8(&self.text[..usize::from(self.len)]).unwrap_or("")
    }

    pub fn encode(&self) -> [u8; RECORD_LEN] {
        let mut out = [0u8; RECORD_LEN];
        out[0..2].copy_from_slice(&MAGIC.to_le_bytes());
        out[2] = self.kind.as_wire();
        out[3] = self.len;
        out[4..8].copy_from_slice(&self.seq.to_le_bytes());
        out[8..12].copy_from_slice(&self.uptime_s.to_le_bytes());
        out[12..14].copy_from_slice(&self.boot.to_le_bytes());
        out[HEADER_LEN..HEADER_LEN + TEXT_MAX].copy_from_slice(&self.text);
        let crc = crc32(&out[..RECORD_LEN - CRC_LEN]);
        out[RECORD_LEN - CRC_LEN..].copy_from_slice(&crc.to_le_bytes());
        out
    }

    /// `None` for a blank slot, another kind of bytes, a kind this build
    /// does not know, or a crc that does not match.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < RECORD_LEN {
            return None;
        }
        let bytes = &bytes[..RECORD_LEN];
        if u16::from_le_bytes([bytes[0], bytes[1]]) != MAGIC {
            return None;
        }
        let crc = u32::from_le_bytes(bytes[RECORD_LEN - CRC_LEN..].try_into().ok()?);
        if crc != crc32(&bytes[..RECORD_LEN - CRC_LEN]) {
            return None;
        }
        let kind = Kind::from_wire(bytes[2])?;
        let len = bytes[3];
        if usize::from(len) > TEXT_MAX {
            return None;
        }
        let mut text = [0u8; TEXT_MAX];
        text.copy_from_slice(&bytes[HEADER_LEN..HEADER_LEN + TEXT_MAX]);
        core::str::from_utf8(&text[..usize::from(len)]).ok()?;
        Some(Self {
            kind,
            seq: u32::from_le_bytes(bytes[4..8].try_into().ok()?),
            uptime_s: u32::from_le_bytes(bytes[8..12].try_into().ok()?),
            boot: u16::from_le_bytes([bytes[12], bytes[13]]),
            text,
            len,
        })
    }

    /// The sequence and boot numbers of the record in `bytes`, if there is
    /// one. What the boot scan reads from every slot: the header is
    /// checked and the crc is, so a partial record does not decide where
    /// the head is.
    pub fn info_of(bytes: &[u8]) -> Option<(u32, u16)> {
        Self::decode(bytes).map(|r| (r.seq, r.boot))
    }

    /// Whether a slot's bytes are erased flash.
    pub fn is_blank(bytes: &[u8]) -> bool {
        bytes.iter().all(|&b| b == 0xFF)
    }
}

/// Where the next record goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Head {
    /// The slot the next record is written to.
    pub slot: usize,
    /// Its sequence number.
    pub next_seq: u32,
    /// The boot number the next boot record carries: one past the newest
    /// record's, or 1 on an empty log.
    pub next_boot: u16,
    /// How many records the log holds.
    pub count: usize,
}

/// The ring's arithmetic: slots, sectors, where the head is and which
/// sector has to be erased before a write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ring {
    /// Record slots in the whole region.
    pub slots: usize,
    /// Slots per erase sector.
    pub per_sector: usize,
}

impl Ring {
    /// A ring over `region_len` bytes of flash erased `sector_len` at a
    /// time. `None` if the region is smaller than one sector, or the
    /// sector smaller than one record.
    pub const fn new(region_len: usize, sector_len: usize) -> Option<Self> {
        if sector_len < RECORD_LEN || region_len < sector_len {
            return None;
        }
        let per_sector = sector_len / RECORD_LEN;
        let sectors = region_len / sector_len;
        Some(Self {
            slots: sectors * per_sector,
            per_sector,
        })
    }

    /// Byte offset of a slot inside the region.
    pub const fn offset(&self, slot: usize) -> u32 {
        (slot * RECORD_LEN) as u32
    }

    /// The sector a slot lies in.
    pub const fn sector_of(&self, slot: usize) -> usize {
        slot / self.per_sector
    }

    /// Whether writing `slot` means erasing its sector first: the slot is
    /// the first of the sector, so the ring is entering it.
    pub const fn erase_before(&self, slot: usize) -> bool {
        slot % self.per_sector == 0
    }

    /// Find the head from what the slots hold. `info_of(slot)` is the
    /// record's sequence and boot numbers, or `None` for a blank or
    /// invalid slot; `blank(slot)` says whether the slot is erased flash.
    /// Both are read per slot rather than taken as a table so the caller
    /// need not hold the whole region in RAM.
    ///
    /// The head is the slot after the newest record, moved forward past
    /// anything not blank in the same sector - an interrupted program, or
    /// another program's bytes - up to the next sector boundary, where an
    /// erase makes the question moot.
    pub fn locate(
        &self,
        info_of: impl Fn(usize) -> Option<(u32, u16)>,
        blank: impl Fn(usize) -> bool,
    ) -> Head {
        let mut newest: Option<(usize, u32, u16)> = None;
        let mut count = 0usize;
        for slot in 0..self.slots {
            if let Some((seq, boot)) = info_of(slot) {
                count += 1;
                if newest.is_none_or(|(_, s, _)| seq > s) {
                    newest = Some((slot, seq, boot));
                }
            }
        }
        let Some((slot, seq, boot)) = newest else {
            return Head {
                slot: 0,
                next_seq: 1,
                next_boot: 1,
                count: 0,
            };
        };
        let mut head = (slot + 1) % self.slots;
        while !self.erase_before(head) && !blank(head) {
            head = (head + 1) % self.slots;
        }
        Head {
            slot: head,
            next_seq: seq.wrapping_add(1).max(1),
            next_boot: boot.wrapping_add(1).max(1),
            count,
        }
    }

    /// The slot holding the `i`-th newest record, counting back from a
    /// head, or `None` past the ring's capacity.
    pub const fn newest(&self, head: usize, i: usize) -> Option<usize> {
        if i >= self.slots {
            return None;
        }
        Some((head + self.slots - 1 - i) % self.slots)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rec(seq: u32, text: &str) -> Record {
        Record::new(Kind::Note, seq, 42, 7, text)
    }

    #[test]
    fn a_record_round_trips() {
        let r = Record::new(Kind::Panic, 9, 1234, 3, "panicked at src/ble.rs:10:5: boom");
        let bytes = r.encode();
        assert_eq!(bytes.len(), RECORD_LEN);
        let back = Record::decode(&bytes).expect("decodes");
        assert_eq!(back, r);
        assert_eq!(back.text(), "panicked at src/ble.rs:10:5: boom");
        assert_eq!(back.kind, Kind::Panic);
        assert_eq!(back.seq, 9);
        assert_eq!(back.uptime_s, 1234);
        assert_eq!(back.boot, 3);
        assert_eq!(Record::info_of(&bytes), Some((9, 3)));
    }

    #[test]
    fn text_is_cut_at_a_character_boundary() {
        let long: String = "ab".repeat(100);
        let r = rec(1, &long);
        assert_eq!(r.text().len(), TEXT_MAX);
        // A multibyte character straddling the cut is dropped whole.
        let mut s = "x".repeat(TEXT_MAX - 1);
        s.push('\u{e9}');
        let r = rec(1, &s);
        assert_eq!(r.text().len(), TEXT_MAX - 1);
        assert!(Record::decode(&r.encode()).is_some());
    }

    #[test]
    fn blank_corrupt_and_foreign_slots_do_not_decode() {
        let blank = [0xFFu8; RECORD_LEN];
        assert!(Record::is_blank(&blank));
        assert!(Record::decode(&blank).is_none());
        let mut bytes = rec(5, "hello").encode();
        bytes[20] ^= 1;
        assert!(Record::decode(&bytes).is_none());
        let mut bytes = rec(5, "hello").encode();
        bytes[2] = 0x7F;
        assert!(Record::decode(&bytes).is_none());
        let short = &rec(5, "hello").encode()[..RECORD_LEN - 1];
        assert!(Record::decode(short).is_none());
    }

    #[test]
    fn every_kind_round_trips_the_wire() {
        for k in Kind::ALL {
            assert_eq!(Kind::from_wire(k.as_wire()), Some(k));
            assert!(!k.as_str().is_empty());
        }
        assert_eq!(Kind::from_wire(0), None);
    }

    #[test]
    fn the_ring_is_sized_from_the_region() {
        let ring = Ring::new(65_536, 4096).expect("fits");
        assert_eq!(ring.per_sector, 32);
        assert_eq!(ring.slots, 512);
        assert_eq!(ring.sector_of(33), 1);
        assert!(ring.erase_before(32));
        assert!(!ring.erase_before(33));
        assert_eq!(ring.offset(3), 384);
        assert!(Ring::new(100, 4096).is_none());
        assert!(Ring::new(4096, 64).is_none());
    }

    /// A slot table for the ring tests: `Some(seq)` for a record, `None`
    /// blank, and a separate set of slots that are neither. Every record
    /// is from boot 4.
    struct Slots {
        seq: Vec<Option<u32>>,
        garbage: Vec<usize>,
    }

    impl Slots {
        fn head(&self, ring: &Ring) -> Head {
            ring.locate(
                |s| self.seq[s].map(|seq| (seq, 4)),
                |s| self.seq[s].is_none() && !self.garbage.contains(&s),
            )
        }
    }

    #[test]
    fn an_empty_log_starts_at_the_first_slot() {
        let ring = Ring::new(4096 * 2, 4096).unwrap();
        let slots = Slots {
            seq: vec![None; ring.slots],
            garbage: vec![],
        };
        assert_eq!(
            slots.head(&ring),
            Head {
                slot: 0,
                next_seq: 1,
                next_boot: 1,
                count: 0
            }
        );
    }

    #[test]
    fn the_head_follows_the_newest_record() {
        let ring = Ring::new(4096 * 2, 4096).unwrap();
        let mut seq = vec![None; ring.slots];
        for (i, s) in seq.iter_mut().enumerate().take(10) {
            *s = Some(i as u32 + 1);
        }
        let slots = Slots {
            seq,
            garbage: vec![],
        };
        let h = slots.head(&ring);
        assert_eq!((h.slot, h.next_seq, h.next_boot, h.count), (10, 11, 5, 10));
    }

    #[test]
    fn the_head_wraps_and_skips_a_partial_slot() {
        let ring = Ring::new(4096 * 2, 4096).unwrap();
        // The ring has wrapped: sector 1 holds seqs 100..131, sector 0
        // holds the newer 132..140, then a slot that was half programmed.
        let mut seq = vec![None; ring.slots];
        for i in 0..32 {
            seq[32 + i] = Some(100 + i as u32);
        }
        for i in 0..9 {
            seq[i] = Some(132 + i as u32);
        }
        let slots = Slots {
            seq,
            garbage: vec![9],
        };
        let h = slots.head(&ring);
        assert_eq!((h.slot, h.next_seq), (10, 141));
        // The newest record is the last one written; counting back runs
        // through sector 0 and wraps into the end of sector 1.
        assert_eq!(ring.newest(h.slot, 0), Some(9));
        assert_eq!(ring.newest(h.slot, 1), Some(8));
        assert_eq!(ring.newest(h.slot, 9), Some(0));
        assert_eq!(ring.newest(h.slot, 10), Some(63));
        assert_eq!(ring.newest(h.slot, ring.slots), None);
    }

    #[test]
    fn a_partial_slot_at_the_end_of_a_sector_moves_the_head_to_the_next() {
        let ring = Ring::new(4096 * 2, 4096).unwrap();
        let mut seq = vec![None; ring.slots];
        seq[30] = Some(5);
        let slots = Slots {
            seq,
            garbage: vec![31],
        };
        let h = slots.head(&ring);
        assert_eq!(h.slot, 32);
        assert!(ring.erase_before(h.slot));
    }

    #[test]
    fn the_last_slot_wraps_to_the_first() {
        let ring = Ring::new(4096 * 2, 4096).unwrap();
        let mut seq = vec![None; ring.slots];
        seq[63] = Some(1000);
        let slots = Slots {
            seq,
            garbage: vec![],
        };
        assert_eq!(slots.head(&ring).slot, 0);
    }
}
