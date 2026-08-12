#![no_std]
#![no_main]

use panic_halt as _;

use core::ptr;

// ── Flash / partition constants ──────────────────────────────────────────────

/// STM32WLE5 flash page size (2 KB).
const PAGE_SIZE: u32 = 2048;

/// Base address of internal flash.
const FLASH_BASE: u32 = 0x0800_0000;

/// ACTIVE partition: pages 8–63  (112 KB, 56 pages).
const ACTIVE_BASE: u32 = FLASH_BASE + 8 * PAGE_SIZE; // 0x0800_4000
const ACTIVE_PAGES: u32 = 56;

/// DFU partition: pages 64–120  (114 KB, 57 pages — last page is scratch).
const DFU_BASE: u32 = FLASH_BASE + 64 * PAGE_SIZE; // 0x0802_0000
const SCRATCH_PAGE: u32 = DFU_BASE + 56 * PAGE_SIZE; // page 120

/// Boot-state partition: page 121 (0x0803_C800).
const STATE_ADDR: u32 = FLASH_BASE + 121 * PAGE_SIZE;

// Magic words written to STATE_ADDR.
const BOOT_OK: u32 = 0x4F4B_4F4B;
const SWAP_PENDING: u32 = 0x5357_4150;
const SWAP_COMPLETE: u32 = 0x444F_4E45;
const REVERT_PENDING: u32 = 0x5245_5654;

// Progress word sits right after the state magic (STATE_ADDR + 4).
// Stores the next page index to swap (0..ACTIVE_PAGES).
// 0xFFFF_FFFF (erased flash) means "start from 0".
const PROGRESS_ADDR: u32 = STATE_ADDR + 4;

// ── STM32WLE5 flash register addresses ──────────────────────────────────────

const FLASH_REGS: u32 = 0x5800_4000;
const FLASH_KEYR: *mut u32 = (FLASH_REGS + 0x08) as *mut u32;
const FLASH_SR: *mut u32 = (FLASH_REGS + 0x10) as *mut u32;
const FLASH_CR: *mut u32 = (FLASH_REGS + 0x14) as *mut u32;

const FLASH_SR_BSY: u32 = 1 << 16;
const FLASH_SR_EOP: u32 = 1 << 0;
const FLASH_CR_PER: u32 = 1 << 1; // page erase
const FLASH_CR_STRT: u32 = 1 << 16;
const FLASH_CR_PG: u32 = 1 << 0; // programming
const FLASH_CR_LOCK: u32 = 1 << 31;
const FLASH_CR_PNB_SHIFT: u32 = 3; // page number bits [10:3]

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

// ── Low-level flash helpers ─────────────────────────────────────────────────

/// Wait until flash is not busy.
#[inline(never)]
fn flash_wait() {
    unsafe {
        while ptr::read_volatile(FLASH_SR) & FLASH_SR_BSY != 0 {}
    }
}

/// Unlock the flash for programming/erase. No-op if already unlocked.
fn flash_unlock() {
    unsafe {
        if ptr::read_volatile(FLASH_CR) & FLASH_CR_LOCK != 0 {
            ptr::write_volatile(FLASH_KEYR, FLASH_KEY1);
            ptr::write_volatile(FLASH_KEYR, FLASH_KEY2);
        }
    }
}

/// Lock flash.
fn flash_lock() {
    unsafe {
        let cr = ptr::read_volatile(FLASH_CR);
        ptr::write_volatile(FLASH_CR, cr | FLASH_CR_LOCK);
    }
}

/// Erase a single flash page by absolute page number (0–127).
fn flash_erase_page(page: u32) {
    unsafe {
        flash_wait();
        // Clear EOP
        ptr::write_volatile(FLASH_SR, FLASH_SR_EOP);
        // Set PER + page number + STRT
        let cr = FLASH_CR_PER | (page << FLASH_CR_PNB_SHIFT) | FLASH_CR_STRT;
        ptr::write_volatile(FLASH_CR, cr);
        flash_wait();
        // Clear PER
        ptr::write_volatile(FLASH_CR, 0);
    }
}

/// Program `len` bytes from `src` to flash address `dst`.
/// Both `dst` and `len` must be 8-byte (double-word) aligned.
///
/// `src` has no alignment requirement: the words are assembled from bytes
/// rather than read through a `*const u32`. That cast was undefined behavior
/// for the byte arrays `write_state` builds on the stack, which carry no
/// alignment at all - and the compiler is free to answer it with an `ldrd`,
/// which faults. Nothing to lose by assembling instead, since LLVM still emits
/// a plain load where it can prove the alignment, and a fault here would be in
/// the code that writes the boot state, with no way back.
fn flash_program(dst: u32, src: &[u8]) {
    assert!(dst % 8 == 0);
    assert!(src.len() % 8 == 0);
    unsafe {
        let mut offset = 0u32;
        while (offset as usize) < src.len() {
            flash_wait();
            // Set PG bit
            ptr::write_volatile(FLASH_CR, FLASH_CR_PG);

            // Write low word then high word (must be two consecutive 32-bit writes)
            let w = &src[offset as usize..offset as usize + 8];
            let lo = u32::from_le_bytes([w[0], w[1], w[2], w[3]]);
            let hi = u32::from_le_bytes([w[4], w[5], w[6], w[7]]);

            let dst_ptr = dst + offset;
            ptr::write_volatile(dst_ptr as *mut u32, lo);
            ptr::write_volatile((dst_ptr + 4) as *mut u32, hi);

            flash_wait();
            // Clear PG
            ptr::write_volatile(FLASH_CR, 0);
            offset += 8;
        }
    }
}

/// Read a 32-bit word from a flash address.
fn flash_read_u32(addr: u32) -> u32 {
    unsafe { ptr::read_volatile(addr as *const u32) }
}

// ── Page-level copy helpers ─────────────────────────────────────────────────

/// Page buffer in RAM used for copying. Must be 8-byte aligned.
#[repr(align(8))]
struct PageBuf([u8; PAGE_SIZE as usize]);

static mut PAGE_BUF: PageBuf = PageBuf([0u8; PAGE_SIZE as usize]);

/// Absolute page number for an address.
fn page_of(addr: u32) -> u32 {
    (addr - FLASH_BASE) / PAGE_SIZE
}

/// Copy one flash page from `src_addr` to `dst_addr`, erasing dst first.
fn copy_page(src_addr: u32, dst_addr: u32) {
    unsafe {
        // Read source page into RAM buffer
        let buf_ptr = &raw mut PAGE_BUF;
        ptr::copy_nonoverlapping(
            src_addr as *const u8,
            (*buf_ptr).0.as_mut_ptr(),
            PAGE_SIZE as usize,
        );
        // Erase destination page
        flash_erase_page(page_of(dst_addr));
        // Program from buffer
        flash_program(dst_addr, &(*buf_ptr).0);
    }
}

// ── State helpers ───────────────────────────────────────────────────────────

fn read_state() -> u32 {
    flash_read_u32(STATE_ADDR)
}

fn read_progress() -> u32 {
    flash_read_u32(PROGRESS_ADDR)
}

/// Write a new state (and optionally progress) to the state page.
/// Erases the state page first, then programs the magic word + progress.
fn write_state(state: u32, progress: u32) {
    flash_erase_page(page_of(STATE_ADDR));
    // We need to write 8 bytes (state + progress) as one double-word.
    let dw: [u8; 8] = {
        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&state.to_le_bytes());
        buf[4..8].copy_from_slice(&progress.to_le_bytes());
        buf
    };
    flash_program(STATE_ADDR, &dw);
}

// ── Swap logic ──────────────────────────────────────────────────────────────

/// Perform a power-fail-safe swap of ACTIVE ↔ DFU.
///
/// For each page i (starting from `start_page`):
///   1. Copy ACTIVE[i] → scratch
///   2. Copy DFU[i]    → ACTIVE[i]
///   3. Copy scratch    → DFU[i]
///   4. Update progress to i+1
fn swap_partitions(start_page: u32) {
    for i in start_page..ACTIVE_PAGES {
        let active_addr = ACTIVE_BASE + i * PAGE_SIZE;
        let dfu_addr = DFU_BASE + i * PAGE_SIZE;

        // ACTIVE[i] → scratch
        copy_page(active_addr, SCRATCH_PAGE);
        // DFU[i] → ACTIVE[i]
        copy_page(dfu_addr, active_addr);
        // scratch → DFU[i]
        copy_page(SCRATCH_PAGE, dfu_addr);

        // Persist progress so we can resume after power loss
        write_state(SWAP_PENDING, i + 1);
    }
}

/// Revert: copy DFU (which contains old firmware after a swap) back to ACTIVE.
fn revert_partitions(start_page: u32) {
    for i in start_page..ACTIVE_PAGES {
        let active_addr = ACTIVE_BASE + i * PAGE_SIZE;
        let dfu_addr = DFU_BASE + i * PAGE_SIZE;

        copy_page(dfu_addr, active_addr);
        write_state(REVERT_PENDING, i + 1);
    }
}

// ── Jump to application ─────────────────────────────────────────────────────

/// RAM bounds for the app's initial stack pointer (64 KB at 0x2000_0000).
const RAM_START: u32 = 0x2000_0000;
const RAM_END: u32 = RAM_START + 64 * 1024;

/// Whether the ACTIVE partition holds a plausible vector table: an initial
/// stack pointer inside RAM and a reset vector inside the ACTIVE flash range
/// with the Thumb bit set. Guards against jumping into an erased (all-0xFF)
/// or half-written application, which would fault into a lockup.
fn app_valid() -> bool {
    let sp = flash_read_u32(ACTIVE_BASE);
    let reset = flash_read_u32(ACTIVE_BASE + 4);
    (RAM_START..=RAM_END).contains(&sp)
        && (reset & 1) == 1
        && (ACTIVE_BASE..DFU_BASE).contains(&(reset & !1))
}

/// Set VTOR and jump to the application in the ACTIVE partition.
///
/// # Safety
/// The ACTIVE partition must contain a valid vector table.
unsafe fn jump_to_app() -> ! {
    let vtor = ACTIVE_BASE;
    let sp = ptr::read_volatile(vtor as *const u32);
    let reset = ptr::read_volatile((vtor + 4) as *const u32);

    // Set VTOR to application vector table
    const SCB_VTOR: *mut u32 = 0xE000_ED08 as *mut u32;
    ptr::write_volatile(SCB_VTOR, vtor);

    // Barriers
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Set MSP and jump via inline assembly
    core::arch::asm!(
        "msr MSP, {sp}",
        "bx {entry}",
        sp = in(reg) sp,
        entry = in(reg) reset,
        options(noreturn),
    )
}

// ── Entry point ─────────────────────────────────────────────────────────────

#[cortex_m_rt::entry]
fn main() -> ! {
    flash_unlock();

    let state = read_state();
    let progress = read_progress();
    let start = if progress == 0xFFFF_FFFF { 0 } else { progress };

    match state {
        SWAP_PENDING => {
            swap_partitions(start);
            write_state(SWAP_COMPLETE, 0xFFFF_FFFF);
        }
        REVERT_PENDING => {
            revert_partitions(start);
            write_state(BOOT_OK, 0xFFFF_FFFF);
        }
        SWAP_COMPLETE => {
            // New firmware booted but never confirmed → revert
            revert_partitions(0);
            write_state(BOOT_OK, 0xFFFF_FFFF);
        }
        _ => {
            // BOOT_OK or erased (0xFFFF_FFFF) → normal boot
        }
    }

    flash_lock();

    // With no valid application (e.g. the bootloader was just flashed on a
    // freshly erased chip, before the app), wait quietly for a reflash
    // instead of jumping into erased flash and locking up.
    if !app_valid() {
        loop {
            cortex_m::asm::wfi();
        }
    }

    unsafe { jump_to_app() }
}
