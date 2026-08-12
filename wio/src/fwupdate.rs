//! Firmware-update receiver: UART-fed DFU staging.
//!
//! The ESP32-C6 streams a new application image over the link
//! (FW_BEGIN / FW_DATA / FW_END). Chunks are buffered into a 2 KB page
//! buffer and written into the DFU partition; on a CRC-verified FW_END
//! the boot state is set to SWAP_PENDING and the caller reboots into the
//! swap bootloader, which installs the image power-fail-safely. If the
//! new firmware never confirms boot, the bootloader reverts.

use core::cell::UnsafeCell;

use stm32wlxx_hal::flash::{AlignedAddr, Flash, Page};
use stm32wlxx_hal::pac;

use midair_proto::link::{crc32, err};

const FLASH_BASE: u32 = 0x0800_0000;
const PAGE_SIZE: u32 = 2048;

/// DFU staging partition: pages 64-119 (56 data pages, matching ACTIVE).
const DFU_BASE: u32 = FLASH_BASE + 64 * PAGE_SIZE; // 0x0802_0000
const DFU_PAGE_START: u8 = 64;

/// Maximum firmware size (ACTIVE partition = 112 KB = 56 pages).
const MAX_FW_SIZE: u32 = 56 * PAGE_SIZE;

const PAGE_BUF_SIZE: usize = PAGE_SIZE as usize;

// Static page buffer: a 2 KB buffer inside the receiver struct would
// land on the stack during init and risk overflow (single RTIC task,
// single core, so the UnsafeCell access is exclusive).
struct StaticPageBuf(UnsafeCell<[u8; PAGE_BUF_SIZE]>);
unsafe impl Sync for StaticPageBuf {}
static PAGE_BUF: StaticPageBuf = StaticPageBuf(UnsafeCell::new([0xFF; PAGE_BUF_SIZE]));

/// A transfer with no frame for this long is abandoned.
///
/// An active transfer owns the main loop - no GPS, no radio, no SD - so a host
/// that stops halfway takes the node off the air until someone power-cycles it.
/// The ESP gives up after 5 s of its own, but an ESP that resets mid-transfer
/// never sends the abort, and that is the case this covers. Long enough that no
/// legitimate gap between chunks comes close.
const IDLE_TIMEOUT_MS: u32 = 30_000;

struct Receiving {
    size: u32,
    crc32: u32,
    next_seq: u16,
    received: u32,
    current_page: u16,
    page_offset: u16,
    /// When the last frame for this transfer arrived, for [`IDLE_TIMEOUT_MS`].
    last_ms: u32,
}

pub enum FwEvent {
    /// Nothing special; ack with the next expected seq.
    Ack(u16),
    /// Transfer failed; NAK with this error code.
    Error(u8),
    /// Image verified and swap requested - ack, then reboot.
    Complete,
}

/// UART firmware-update receiver state machine.
pub struct FwUpdate {
    state: Option<Receiving>,
}

impl FwUpdate {
    pub const fn new() -> Self {
        Self { state: None }
    }

    pub fn is_active(&self) -> bool {
        self.state.is_some()
    }

    /// Progress as (received_bytes, total_bytes).
    pub fn progress(&self) -> Option<(u32, u32)> {
        self.state.as_ref().map(|rx| (rx.received, rx.size))
    }

    pub fn abort(&mut self) {
        self.state = None;
    }

    /// Drop a transfer that has gone quiet, returning whether one was dropped.
    /// Call every main-loop iteration; see [`IDLE_TIMEOUT_MS`].
    pub fn expire(&mut self, now_ms: u32) -> bool {
        let stale = match &self.state {
            Some(rx) => now_ms.wrapping_sub(rx.last_ms) >= IDLE_TIMEOUT_MS,
            None => false,
        };
        if stale {
            self.state = None;
        }
        stale
    }

    fn page_buf(&self) -> &[u8; PAGE_BUF_SIZE] {
        unsafe { &*PAGE_BUF.0.get() }
    }

    fn page_buf_mut(&mut self) -> &mut [u8; PAGE_BUF_SIZE] {
        unsafe { &mut *PAGE_BUF.0.get() }
    }

    /// FW_BEGIN: `[size u32le, crc32 u32le, version u16le]`.
    pub fn begin(&mut self, payload: &[u8], now_ms: u32) -> FwEvent {
        if payload.len() < 10 {
            return FwEvent::Error(err::BAD_FRAME);
        }
        let size = u32::from_le_bytes(payload[0..4].try_into().unwrap());
        let crc = u32::from_le_bytes(payload[4..8].try_into().unwrap());
        let version = u16::from_le_bytes(payload[8..10].try_into().unwrap());
        if size == 0 || size > MAX_FW_SIZE {
            return FwEvent::Error(err::BAD_SIZE);
        }
        rtt_target::rprintln!("FW: begin size={} crc={:08x} v{}", size, crc, version);
        self.page_buf_mut().fill(0xFF);
        self.state = Some(Receiving {
            size,
            crc32: crc,
            next_seq: 0,
            received: 0,
            current_page: 0,
            page_offset: 0,
            last_ms: now_ms,
        });
        FwEvent::Ack(0)
    }

    /// FW_DATA: `[seq u16le, bytes...]`.
    pub fn data(&mut self, payload: &[u8], flash: &mut pac::FLASH, now_ms: u32) -> FwEvent {
        let Some(rx) = self.state.as_mut() else {
            return FwEvent::Error(err::INVALID_STATE);
        };
        if payload.len() < 3 {
            return FwEvent::Error(err::BAD_FRAME);
        }
        // Any well-formed frame proves the host is still there, a retransmit
        // included, so the idle deadline moves before the sequence checks.
        rx.last_ms = now_ms;
        let (next_seq, received, size) = (rx.next_seq, rx.received, rx.size);
        let seq = u16::from_le_bytes(payload[0..2].try_into().unwrap());
        let data = &payload[2..];

        // Duplicate (retransmit after a lost ack): re-ack, don't rewrite.
        if seq.wrapping_add(1) == next_seq {
            return FwEvent::Ack(next_seq);
        }
        if seq != next_seq {
            return FwEvent::Error(err::BAD_SEQ);
        }
        if received + data.len() as u32 > size {
            self.state = None;
            return FwEvent::Error(err::BAD_SIZE);
        }

        // Copy into the page buffer, flushing full pages to flash.
        let mut remaining = data;
        while !remaining.is_empty() {
            let rx = self.state.as_ref().unwrap();
            let offset = rx.page_offset as usize;
            let space = PAGE_BUF_SIZE - offset;
            let n = remaining.len().min(space);
            self.page_buf_mut()[offset..offset + n].copy_from_slice(&remaining[..n]);
            remaining = &remaining[n..];

            let rx = self.state.as_mut().unwrap();
            rx.page_offset += n as u16;
            rx.received += n as u32;
            if rx.page_offset as usize == PAGE_BUF_SIZE {
                let page_idx = DFU_PAGE_START + rx.current_page as u8;
                if !self.write_page(flash, page_idx) {
                    self.state = None;
                    return FwEvent::Error(err::FLASH_ERROR);
                }
                let rx = self.state.as_mut().unwrap();
                rx.current_page += 1;
                rx.page_offset = 0;
                self.page_buf_mut().fill(0xFF);
            }
        }

        let rx = self.state.as_mut().unwrap();
        rx.next_seq = seq.wrapping_add(1);
        FwEvent::Ack(rx.next_seq)
    }

    /// FW_END: flush the tail page, verify CRC32, request the swap.
    ///
    /// Every failure leaves the transfer in place, so a host that retries the
    /// step - which the uploader does when an ack goes missing - gets the same
    /// answer instead of `INVALID_STATE` for a transfer this call had quietly
    /// destroyed. `FW_ABORT` and the idle deadline are what clear a transfer
    /// that ends badly; only success consumes it here.
    pub fn end(&mut self, flash: &mut pac::FLASH) -> FwEvent {
        let Some(rx) = &self.state else {
            return FwEvent::Error(err::INVALID_STATE);
        };
        if rx.received != rx.size {
            rtt_target::rprintln!("FW: end with missing data (next seq {})", rx.next_seq);
            return FwEvent::Error(err::BAD_SIZE);
        }
        let (size, want_crc, page_offset, current_page) =
            (rx.size, rx.crc32, rx.page_offset, rx.current_page);
        if page_offset > 0 && !self.write_page(flash, DFU_PAGE_START + current_page as u8) {
            return FwEvent::Error(err::FLASH_ERROR);
        }
        // Flash is memory-mapped, so the shared CRC runs over it directly.
        let staged = unsafe { core::slice::from_raw_parts(DFU_BASE as *const u8, size as usize) };
        let computed = crc32(staged);
        if computed != want_crc {
            rtt_target::rprintln!("FW: crc mismatch {:08x} != {:08x}", computed, want_crc);
            return FwEvent::Error(err::CRC_MISMATCH);
        }
        self.state = None;
        crate::boot_state::request_swap(flash);
        rtt_target::rprintln!("FW: image verified, swap requested");
        FwEvent::Complete
    }

    fn write_page(&self, flash_periph: &mut pac::FLASH, page_idx: u8) -> bool {
        let mut flash = Flash::unlock(flash_periph);
        let page = unsafe { Page::from_index_unchecked(page_idx) };
        if unsafe { flash.page_erase(page) }.is_err() {
            return false;
        }
        let addr = unsafe { AlignedAddr::new_unchecked(page.addr()) };
        unsafe { flash.program_bytes(self.page_buf(), addr) }.is_ok()
        // Flash locked on drop.
    }
}

impl Default for FwUpdate {
    fn default() -> Self {
        Self::new()
    }
}
