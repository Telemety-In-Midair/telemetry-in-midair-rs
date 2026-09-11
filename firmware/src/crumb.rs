//! What a dying board leaves for the next boot, in RTC RAM.
//!
//! A panic handler and a stall reset both run on a board that may be
//! half broken: the other core may be holding the lock the console and
//! the flash need, the heap may be what failed, the stack may be what
//! overflowed. So what is written here is written with atomics and
//! nothing else - no lock, no allocation, no formatting into anything but
//! a fixed byte array - and it survives the reset that follows because
//! RTC fast RAM survives every reset short of a power cycle. The boot
//! after reads it, writes it into the event log with the flash awake and
//! the locks free, and clears it.
//!
//! The magic word and the checksum are what separate a crumb from what
//! the die held at power-on and from a crumb a reset cut in half.

use core::fmt::Write as _;

use midair_proto::evlog::TEXT_MAX;
use midair_proto::supervise::{Phase, Stall, Task};
use portable_atomic::{AtomicU32, AtomicU8, Ordering};

/// Marks the block as a crumb ("midb"). One letter off the settings
/// block's "mida", which shares the RTC RAM.
const MAGIC: u32 = 0x6D69_6462;

/// Backtrace program counters kept with a panic.
pub const PCS_MAX: usize = 8;

const KIND_NONE: u32 = 0;
const KIND_PANIC: u32 = 1;
const KIND_STALL: u32 = 2;

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static MAGIC_WORD: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static KIND: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static UPTIME_S: AtomicU32 = AtomicU32::new(0);
/// A stall: the task in the low byte, the phase above it.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static WHO: AtomicU32 = AtomicU32::new(0);
/// A stall: how long the task had been silent.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static SILENT_MS: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static TEXT_LEN: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static TEXT: [AtomicU8; TEXT_MAX] = [const { AtomicU8::new(0) }; TEXT_MAX];
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PCS: [AtomicU32; PCS_MAX] = [const { AtomicU32::new(0) }; PCS_MAX];
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static SUM: AtomicU32 = AtomicU32::new(0);
/// Why the board last reset itself, when it did so on purpose. Separate
/// from the crumb, since a commanded reboot leaves no crumb and a stall
/// that was logged before the reset clears its crumb but keeps its reason.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static RESET_REASON: AtomicU32 = AtomicU32::new(0);

/// Why the firmware reset the board, when it was the firmware that did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Reason {
    /// A panic, or a CPU exception, which the HAL turns into one.
    Panic = 1,
    /// The monitor found a task past its bound.
    Stall = 2,
    /// A firmware image was installed and the board rebooted into it.
    Ota = 3,
    /// A wipe over the console.
    Wipe = 4,
}

impl Reason {
    pub const fn as_str(self) -> &'static str {
        match self {
            Reason::Panic => "panic",
            Reason::Stall => "stall",
            Reason::Ota => "ota reboot",
            Reason::Wipe => "wipe",
        }
    }

    const fn from_word(v: u32) -> Option<Self> {
        Some(match v {
            1 => Reason::Panic,
            2 => Reason::Stall,
            3 => Reason::Ota,
            4 => Reason::Wipe,
            _ => return None,
        })
    }
}

/// What the previous boot left.
#[derive(Clone, Debug)]
pub enum Crumb {
    Panic {
        uptime_s: u32,
        /// `file:line:column: message`, cut to fit.
        text: heapless::String<TEXT_MAX>,
        pcs: heapless::Vec<u32, PCS_MAX>,
    },
    Stall {
        uptime_s: u32,
        stall: Stall,
    },
}

/// A `fmt::Write` over the text cells. Truncates rather than fails: the
/// head of a panic message is worth more than none of it.
struct Sink(usize);

impl core::fmt::Write for Sink {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.0 >= TEXT_MAX {
                break;
            }
            TEXT[self.0].store(b, Ordering::Relaxed);
            self.0 += 1;
        }
        Ok(())
    }
}

fn checksum(len: usize) -> u32 {
    let mut sum = KIND
        .load(Ordering::Relaxed)
        .wrapping_mul(31)
        .wrapping_add(UPTIME_S.load(Ordering::Relaxed))
        .wrapping_mul(31)
        .wrapping_add(WHO.load(Ordering::Relaxed))
        .wrapping_mul(31)
        .wrapping_add(SILENT_MS.load(Ordering::Relaxed))
        .wrapping_mul(31)
        .wrapping_add(len as u32);
    for cell in TEXT.iter().take(len.min(TEXT_MAX)) {
        sum = sum.wrapping_mul(31).wrapping_add(u32::from(cell.load(Ordering::Relaxed)));
    }
    for pc in &PCS {
        sum = sum.wrapping_mul(31).wrapping_add(pc.load(Ordering::Relaxed));
    }
    sum ^ MAGIC
}

/// Seal what was written: the checksum, then the magic word. A reset
/// between the two leaves a block the next boot refuses.
fn seal(len: usize) {
    TEXT_LEN.store(len as u32, Ordering::Relaxed);
    SUM.store(checksum(len), Ordering::Relaxed);
    MAGIC_WORD.store(MAGIC, Ordering::Release);
}

/// Record a panic. Atomics only: this runs on a board that may hold no
/// other promise.
///
/// The text is cut at [`TEXT_MAX`] bytes wherever that falls; a character
/// split by the cut is read back as `?` by [`take`], which keeps every
/// byte it hands on as printable ASCII. Nothing here may index past the
/// cells, since a panic in the panic handler is a reset with nothing
/// written.
pub fn record_panic(info: &core::panic::PanicInfo<'_>, pcs: &[u32], uptime_s: u32) {
    MAGIC_WORD.store(0, Ordering::Relaxed);
    KIND.store(KIND_PANIC, Ordering::Relaxed);
    UPTIME_S.store(uptime_s, Ordering::Relaxed);
    WHO.store(0, Ordering::Relaxed);
    SILENT_MS.store(0, Ordering::Relaxed);
    for (cell, pc) in PCS.iter().zip(pcs.iter().copied().chain(core::iter::repeat(0))) {
        cell.store(pc, Ordering::Relaxed);
    }
    let mut sink = Sink(0);
    if let Some(loc) = info.location() {
        let _ = write!(sink, "{}:{}:{}: ", loc.file(), loc.line(), loc.column());
    }
    let _ = write!(sink, "{}", info.message());
    seal(sink.0.min(TEXT_MAX));
    RESET_REASON.store(Reason::Panic as u32, Ordering::Relaxed);
}

/// Record a stall the monitor found.
pub fn record_stall(stall: &Stall, uptime_s: u32) {
    MAGIC_WORD.store(0, Ordering::Relaxed);
    KIND.store(KIND_STALL, Ordering::Relaxed);
    UPTIME_S.store(uptime_s, Ordering::Relaxed);
    WHO.store(
        u32::from(stall.task.as_wire()) | (u32::from(stall.phase.as_wire()) << 8),
        Ordering::Relaxed,
    );
    SILENT_MS.store(stall.silent_ms, Ordering::Relaxed);
    for cell in &PCS {
        cell.store(0, Ordering::Relaxed);
    }
    seal(0);
    RESET_REASON.store(Reason::Stall as u32, Ordering::Relaxed);
}

/// Say why the board is about to reset itself, for a reset that leaves
/// no crumb.
pub fn mark_reset(reason: Reason) {
    RESET_REASON.store(reason as u32, Ordering::Relaxed);
}

/// Drop the crumb, keeping the reason: a stall that reached the log
/// before the reset does not need writing twice.
pub fn clear() {
    MAGIC_WORD.store(0, Ordering::Relaxed);
    KIND.store(KIND_NONE, Ordering::Relaxed);
}

/// The reason the last reset was the firmware's own, if it was. Cleared
/// by the read.
pub fn take_reset_reason() -> Option<Reason> {
    let r = Reason::from_word(RESET_REASON.swap(0, Ordering::Relaxed));
    r
}

/// The crumb the previous boot left, if there is one. Cleared by the
/// read, so a crumb is written to the log once.
pub fn take() -> Option<Crumb> {
    if MAGIC_WORD.load(Ordering::Acquire) != MAGIC {
        return None;
    }
    MAGIC_WORD.store(0, Ordering::Relaxed);
    let len = (TEXT_LEN.load(Ordering::Relaxed) as usize).min(TEXT_MAX);
    if SUM.load(Ordering::Relaxed) != checksum(len) {
        return None;
    }
    let uptime_s = UPTIME_S.load(Ordering::Relaxed);
    match KIND.load(Ordering::Relaxed) {
        KIND_PANIC => {
            let mut text = heapless::String::new();
            for cell in TEXT.iter().take(len) {
                let b = cell.load(Ordering::Relaxed);
                // Kept printable: the log is read as text, and a byte
                // that is not is more likely corruption than message.
                let c = if (0x20..0x7F).contains(&b) { b as char } else { '?' };
                let _ = text.push(c);
            }
            let mut pcs = heapless::Vec::new();
            for cell in &PCS {
                let pc = cell.load(Ordering::Relaxed);
                if pc == 0 {
                    break;
                }
                let _ = pcs.push(pc);
            }
            Some(Crumb::Panic {
                uptime_s,
                text,
                pcs,
            })
        }
        KIND_STALL => {
            let who = WHO.load(Ordering::Relaxed);
            let task = Task::from_wire(who as u8)?;
            let phase = Phase::from_wire((who >> 8) as u8).unwrap_or_default();
            Some(Crumb::Stall {
                uptime_s,
                stall: Stall {
                    task,
                    phase,
                    silent_ms: SILENT_MS.load(Ordering::Relaxed),
                },
            })
        }
        _ => None,
    }
}
