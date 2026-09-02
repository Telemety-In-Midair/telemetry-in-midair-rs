//! The radio config's copy in the board's own flash.
//!
//! The card is the store: `RADIO.CFG` is what a person edits, and it wins at
//! boot so that pulling the card to change it on a computer does what it
//! looks like. This is the copy that answers the case the card cannot - a
//! board with no card, or one whose card has failed - where a pushed config
//! was live until the next power cycle and then gone, taking the node
//! address with it. That is the one setting nothing can guess back: two
//! senders sharing an address are mutually deaf, each dropping the other's
//! broadcasts as an echo of its own.
//!
//! What is kept is the config *text* rather than the parsed form, for the
//! same reason the card copy is: the bytes somebody wrote survive a firmware
//! that later learns a new key, where a re-rendering of what this build
//! understood would quietly drop it.
//!
//! ```text
//! offset  size  field
//! 0       4     magic
//! 4       4     version
//! 8       4     length of the text
//! 12      4     crc32 of the text
//! 16      len   the text
//! ```
//!
//! The length and the crc sit in front of the text, so a header and a text
//! that were never written together cannot be read as a pair. That is the
//! interrupted write: the record goes down in one write, of a whole flash
//! sector that was erased first, so a power loss part way through leaves a
//! header in front of bytes that are half the new text and half nothing.
//! The crc refuses it, and the board comes up on the card or on defaults
//! rather than on a config that is partly somebody else's.

use crate::bulk;
use crate::link;

/// Marks the record as this firmware's ("midC"). One letter off the
/// settings record's "midA", which shares the partition.
pub const MAGIC: u32 = 0x6D69_6443;

/// Layout version of the backup record.
///
/// Unlike the settings record there is no ladder of older ones to read: a
/// version this build does not know reads as nothing stored, and the board
/// falls back to the card. The text behind the header is a config file,
/// which already carries its own compatibility - an unknown key is ignored
/// and an absent one keeps its default - so a format change here would have
/// to be a change to the framing rather than to the settings, and there is
/// nothing in the framing left to add.
pub const VERSION: u32 = 1;

/// magic, version, text length, crc32 of the text.
pub const HEADER_LEN: usize = 16;

/// Longest config text a record holds, which is the longest one the firmware
/// reads off a card or accepts in a transfer.
pub const TEXT_MAX: usize = bulk::CONFIG_MAX;

/// Header plus the longest text: what the flash has to have room for.
pub const RECORD_MAX: usize = HEADER_LEN + TEXT_MAX;

/// What sits in front of the stored text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// Bytes of config text behind this header.
    pub len: usize,
    /// crc32 over those bytes.
    pub crc: u32,
}

impl Header {
    /// The header `text` is stored behind, or `None` for text too long to
    /// store.
    ///
    /// Empty text is a header like any other, deliberately: an empty config
    /// file is a valid one meaning "every setting at its default", and a
    /// board that refused to store it would restore the *previous* config
    /// at the next boot - silently undoing a push rather than honoring it.
    /// A record for no text and no record at all are different things here.
    pub fn for_text(text: &[u8]) -> Option<Self> {
        if text.len() > TEXT_MAX {
            return None;
        }
        Some(Self {
            len: text.len(),
            crc: link::crc32(text),
        })
    }

    pub fn encode(&self) -> [u8; HEADER_LEN] {
        let mut hdr = [0u8; HEADER_LEN];
        hdr[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        hdr[4..8].copy_from_slice(&VERSION.to_le_bytes());
        hdr[8..12].copy_from_slice(&(self.len as u32).to_le_bytes());
        hdr[12..16].copy_from_slice(&self.crc.to_le_bytes());
        hdr
    }

    /// Read a header back. `None` for anything that is not one: a short
    /// read, erased flash, another program's bytes, a version this build
    /// does not understand, or a length no transfer could have produced.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < HEADER_LEN {
            return None;
        }
        let word = |i: usize| {
            u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]])
        };
        if word(0) != MAGIC || word(4) != VERSION {
            return None;
        }
        let len = word(8) as usize;
        if len > TEXT_MAX {
            return None;
        }
        Some(Self {
            len,
            crc: word(12),
        })
    }

    /// Whether `text` is the text this header was written for.
    pub fn matches(&self, text: &[u8]) -> bool {
        text.len() == self.len && link::crc32(text) == self.crc
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CONFIG: &[u8] = b"address = 3\nrole = \"repeater\"\n";

    /// The ordinary path: what was written comes back, and says it is what
    /// was written.
    #[test]
    fn a_record_round_trips() {
        let header = Header::for_text(CONFIG).expect("a short config fits");
        let read = Header::decode(&header.encode()).expect("its own header decodes");
        assert_eq!(read, header);
        assert_eq!(read.len, CONFIG.len());
        assert!(read.matches(CONFIG));
    }

    /// An empty config is a real config - every setting at its default -
    /// so it is storable, and it is not the same as having stored nothing.
    /// A board that treated the two alike would answer a push of an empty
    /// file by restoring whatever the last one said.
    #[test]
    fn an_empty_config_is_a_record_and_not_an_absent_one() {
        let header = Header::for_text(b"").expect("an empty config is storable");
        let read = Header::decode(&header.encode()).expect("decodes");
        assert_eq!(read.len, 0);
        assert!(read.matches(b""));
        assert!(!read.matches(CONFIG));
    }

    /// Erased flash is what a board that has never stored a config reads,
    /// and it has to read as nothing rather than as a config of 0xFFFF...
    /// bytes.
    #[test]
    fn erased_flash_is_not_a_record() {
        assert_eq!(Header::decode(&[0xFF; HEADER_LEN]), None);
    }

    /// So is a zeroed region, which is what another program's untouched
    /// allocation looks like.
    #[test]
    fn zeros_are_not_a_record() {
        assert_eq!(Header::decode(&[0x00; HEADER_LEN]), None);
    }

    /// A short read cannot be completed by guessing at the rest.
    #[test]
    fn a_truncated_header_is_not_a_record() {
        let hdr = Header::for_text(CONFIG).unwrap().encode();
        assert_eq!(Header::decode(&hdr[..HEADER_LEN - 1]), None);
    }

    /// A future layout reads as nothing stored, so the board falls back to
    /// the card rather than to a misread record. Downgrading a board is the
    /// case: the record it left behind is not this build's.
    #[test]
    fn an_unknown_version_is_refused() {
        let mut hdr = Header::for_text(CONFIG).unwrap().encode();
        hdr[4..8].copy_from_slice(&(VERSION + 1).to_le_bytes());
        assert_eq!(Header::decode(&hdr), None);
    }

    /// A length past what any transfer could have delivered is corruption,
    /// whatever else the header says - and the reader would otherwise ask
    /// the flash for bytes past the record.
    #[test]
    fn an_impossible_length_is_refused() {
        let mut hdr = Header::for_text(CONFIG).unwrap().encode();
        hdr[8..12].copy_from_slice(&((TEXT_MAX + 1) as u32).to_le_bytes());
        assert_eq!(Header::decode(&hdr), None);
        // And nothing that long can be written in the first place.
        assert_eq!(Header::for_text(&[b'#'; TEXT_MAX + 1]), None);
        assert!(Header::for_text(&[b'#'; TEXT_MAX]).is_some());
    }

    /// The interrupted write: an old header in front of new text. Both
    /// halves are individually valid, and the pair has to be refused - this
    /// is the whole reason the crc is in front of the text rather than
    /// behind it.
    #[test]
    fn a_header_from_a_different_text_does_not_match() {
        let old = Header::for_text(b"address = 1\n").unwrap();
        assert!(!old.matches(CONFIG));
    }

    /// A single flipped byte, with the length unchanged, is what a crc is
    /// for: nothing about the framing notices it.
    #[test]
    fn a_corrupted_text_does_not_match() {
        let header = Header::for_text(CONFIG).unwrap();
        let mut corrupt = CONFIG.to_vec();
        corrupt[0] ^= 0x20;
        assert_eq!(corrupt.len(), header.len);
        assert!(!header.matches(&corrupt));
    }
}
