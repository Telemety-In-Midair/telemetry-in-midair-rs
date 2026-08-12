//! Radio-config transfer receiver (CFG_BEGIN / CFG_DATA / CFG_END).
//!
//! Collects TOML text pushed over the ESP link into a static buffer,
//! verifies the CRC32, and hands the bytes to the caller for parsing,
//! application and SD storage.

use core::cell::UnsafeCell;

use midair_proto::link::{crc32, err};

/// Largest config file accepted (matches [`crate::sdlog::CONFIG_MAX`]).
pub const CONFIG_MAX: usize = 1024;

struct StaticBuf(UnsafeCell<[u8; CONFIG_MAX]>);
unsafe impl Sync for StaticBuf {}
// Single RTIC task, single core: access is exclusive.
static CONFIG_BUF: StaticBuf = StaticBuf(UnsafeCell::new([0; CONFIG_MAX]));

pub enum CfgEvent {
    /// Step accepted; ack with the next expected seq.
    Ack(u16),
    /// Step failed; NAK with this error code.
    Error(u8),
    /// Transfer complete and CRC-verified; the payload is in
    /// [`CfgTransfer::bytes`]. Apply it, then call
    /// [`CfgTransfer::mark_applied`] and ack.
    Complete,
    /// A repeat of an END that already completed and was applied. Ack it
    /// without doing the work a second time.
    Done,
}

pub struct CfgTransfer {
    total: usize,
    received: usize,
    next_seq: u16,
    active: bool,
    /// CRC of the last transfer that verified, with whether the caller went on
    /// to apply it.
    ///
    /// Together they let a repeated CFG_END be answered as the success it was.
    /// The uploader retries the step whenever an ack goes missing - and the ESP
    /// gives up on this one after 2 s, which the apply can outlast on a slow
    /// card - so without this a config that was applied and saved comes back to
    /// the host as a failure.
    last_crc: u32,
    applied: bool,
}

impl CfgTransfer {
    pub const fn new() -> Self {
        Self {
            total: 0,
            received: 0,
            next_seq: 0,
            active: false,
            last_crc: 0,
            applied: false,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    /// The received config text (valid after [`CfgEvent::Complete`]).
    pub fn bytes(&self) -> &[u8] {
        let buf = unsafe { &*CONFIG_BUF.0.get() };
        &buf[..self.received]
    }

    /// CFG_BEGIN: `[total_len u16le]`.
    pub fn begin(&mut self, payload: &[u8]) -> CfgEvent {
        if payload.len() < 2 {
            return CfgEvent::Error(err::BAD_FRAME);
        }
        let total = u16::from_le_bytes(payload[0..2].try_into().unwrap()) as usize;
        if total == 0 || total > CONFIG_MAX {
            return CfgEvent::Error(err::BAD_SIZE);
        }
        self.total = total;
        self.received = 0;
        self.next_seq = 0;
        self.active = true;
        self.applied = false;
        CfgEvent::Ack(0)
    }

    /// Record that the caller parsed and adopted the completed transfer, so a
    /// repeated CFG_END for it answers [`CfgEvent::Done`] rather than
    /// `INVALID_STATE`. A config that verified but would not parse is never
    /// marked, and a retry of that one keeps failing, which is the truth.
    pub fn mark_applied(&mut self) {
        self.applied = true;
    }

    /// CFG_DATA: `[seq u16le, bytes...]`.
    pub fn data(&mut self, payload: &[u8]) -> CfgEvent {
        if !self.active {
            return CfgEvent::Error(err::INVALID_STATE);
        }
        if payload.len() < 3 {
            return CfgEvent::Error(err::BAD_FRAME);
        }
        let seq = u16::from_le_bytes(payload[0..2].try_into().unwrap());
        let data = &payload[2..];
        if seq.wrapping_add(1) == self.next_seq {
            return CfgEvent::Ack(self.next_seq); // duplicate: re-ack
        }
        if seq != self.next_seq {
            return CfgEvent::Error(err::BAD_SEQ);
        }
        if self.received + data.len() > self.total {
            self.active = false;
            return CfgEvent::Error(err::BAD_SIZE);
        }
        let buf = unsafe { &mut *CONFIG_BUF.0.get() };
        buf[self.received..self.received + data.len()].copy_from_slice(data);
        self.received += data.len();
        self.next_seq = seq.wrapping_add(1);
        CfgEvent::Ack(self.next_seq)
    }

    /// CFG_END: `[crc32 u32le]`.
    pub fn end(&mut self, payload: &[u8]) -> CfgEvent {
        if payload.len() < 4 {
            return CfgEvent::Error(err::BAD_FRAME);
        }
        let want = u32::from_le_bytes(payload[0..4].try_into().unwrap());
        if !self.active {
            // Either a retry of the END that already landed, or a stray frame.
            // The CRC is what tells the two apart: it names the transfer.
            return if self.applied && self.last_crc == want {
                CfgEvent::Done
            } else {
                CfgEvent::Error(err::INVALID_STATE)
            };
        }
        self.active = false;
        if self.received != self.total {
            return CfgEvent::Error(err::BAD_SIZE);
        }
        if crc32(self.bytes()) != want {
            return CfgEvent::Error(err::CRC_MISMATCH);
        }
        self.last_crc = want;
        CfgEvent::Complete
    }
}

impl Default for CfgTransfer {
    fn default() -> Self {
        Self::new()
    }
}
