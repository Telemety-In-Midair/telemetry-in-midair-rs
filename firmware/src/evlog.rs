//! The event log: what the board writes down about itself, in flash.
//!
//! The record and the ring are [`midair_proto::evlog`]; the partition
//! and the programming are [`crate::flash`]. What is here is how an
//! event gets from wherever it happens to the flash, and what the boot
//! does with what the last one left.
//!
//! An event is noted from wherever it happens - a status line in the
//! hardware loop, a failed init in the serve loop - into a queue, and
//! the monitor task writes the queue to flash between its checks. That
//! keeps the flash write, which parks the other core and holds
//! interrupts off for a millisecond or forty, out of every task but the
//! one whose job it is; it also means a line noted by a task about to
//! die still reaches the flash, as long as the monitor is alive to write
//! it. What the monitor cannot write - because it is the monitor that
//! found the stall and is about to reset, or because the panic handler
//! runs with nothing to await on - goes through RTC RAM
//! ([`crate::crumb`]) and is written here at the next boot.

use core::fmt::Write as _;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;
use esp_hal::rtc_cntl::SocResetReason;
use esp_println::println;
pub use midair_proto::evlog::{Kind, Record, TEXT_MAX};
use midair_proto::supervise::Stall;

use crate::{crumb, flash};

/// A line waiting for the monitor to write it.
struct Queued {
    kind: Kind,
    uptime_s: u32,
    text: heapless::String<TEXT_MAX>,
}

/// Bounded and non-blocking: nothing that notes an event may wait on the
/// flash, so a full queue drops its oldest line.
static QUEUE: Channel<CriticalSectionRawMutex, Queued, 8> = Channel::new();

/// How many of the newest records the boot prints.
const TAIL: usize = 6;

fn uptime_s() -> u32 {
    Instant::now().as_secs() as u32
}

/// Note an event. Sync, lock-free past the queue's own critical section,
/// and never waits: what it costs the caller is the formatting.
pub fn note(kind: Kind, args: core::fmt::Arguments<'_>) {
    let mut text: heapless::String<TEXT_MAX> = heapless::String::new();
    // Truncate rather than fail: a `write!` into a full heapless string
    // errors, and the head of the line is what a reader wants.
    struct Sink<'a>(&'a mut heapless::String<TEXT_MAX>);
    impl core::fmt::Write for Sink<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let room = self.0.capacity() - self.0.len();
            let mut take = s.len().min(room);
            while !s.is_char_boundary(take) {
                take -= 1;
            }
            let _ = self.0.push_str(&s[..take]);
            Ok(())
        }
    }
    let _ = write!(Sink(&mut text), "{}", args);
    let q = Queued {
        kind,
        uptime_s: uptime_s(),
        text,
    };
    if let Err(embassy_sync::channel::TrySendError::Full(q)) = QUEUE.try_send(q) {
        let _ = QUEUE.try_receive();
        let _ = QUEUE.try_send(q);
    }
}

/// Print a status line and note it in the log, as one event.
#[macro_export]
macro_rules! event {
    ($kind:expr, $($arg:tt)*) => {{
        $crate::status_println!($($arg)*);
        $crate::evlog::note($kind, format_args!($($arg)*));
    }};
}

/// Write what is queued to flash. Called by the monitor between checks.
pub async fn flush() {
    while let Ok(q) = QUEUE.try_receive() {
        let _ = flash::with_flash(|f| f.evlog_append(q.kind, q.uptime_s, &q.text)).await;
    }
}

/// Write a stall straight to flash, ahead of the queue. Returns whether
/// it landed.
pub async fn write_stall(stall: &Stall, uptime_s: u32) -> bool {
    let mut text: heapless::String<TEXT_MAX> = heapless::String::new();
    let _ = write!(
        text,
        "{} silent {} s in {}",
        stall.task.as_str(),
        stall.silent_ms / 1000,
        stall.phase.as_str()
    );
    flash::with_flash(|f| f.evlog_append(Kind::Stall, uptime_s, &text).is_some())
        .await
        .unwrap_or(false)
}

/// The reset reason as a word for the boot record.
fn reason_str(reason: Option<SocResetReason>) -> &'static str {
    match reason {
        Some(SocResetReason::ChipPowerOn) => "power on",
        Some(SocResetReason::CoreSw | SocResetReason::CpuSw) => "software",
        Some(SocResetReason::CoreDeepSleep) => "deep sleep wake",
        Some(
            SocResetReason::CoreMwdt0
            | SocResetReason::CoreMwdt1
            | SocResetReason::CpuMwdt0
            | SocResetReason::CpuMwdt1,
        ) => "hardware watchdog",
        Some(
            SocResetReason::CoreRtcWdt | SocResetReason::CpuRtcWdt | SocResetReason::SysRtcWdt,
        ) => "rtc watchdog",
        Some(SocResetReason::SysBrownOut) => "brownout",
        Some(SocResetReason::SysSuperWdt) => "super watchdog",
        Some(SocResetReason::SysClkGlitch) => "clock glitch",
        Some(SocResetReason::CorePwrGlitch) => "power glitch",
        Some(SocResetReason::CoreEfuseCrc) => "efuse crc",
        Some(SocResetReason::CoreUsbUart | SocResetReason::CoreUsbJtag) => "usb",
        None => "unknown",
    }
}

/// Open the log, write down what the last boot left and why this one
/// happened, and print the tail. Called once from `main`, after the
/// flash is installed and before anything that could fail is started.
pub async fn boot() {
    let opened = flash::with_flash(|f| f.evlog_open()).await.flatten();
    let Some(head) = opened else {
        println!("evlog: no log partition, events go to the console only");
        // Say what the crumb said anyway; it is the one place it can be
        // seen on such a board.
        if let Some(c) = crumb::take() {
            println!("evlog: last boot left: {}", describe(&c));
        }
        return;
    };

    let reason = esp_hal::system::reset_reason();
    let own = crumb::take_reset_reason();
    let mut text: heapless::String<TEXT_MAX> = heapless::String::new();
    let _ = write!(text, "reset: {}", reason_str(reason));
    if let Some(own) = own {
        let _ = write!(text, " ({})", own.as_str());
    }
    let _ = write!(text, ", fw {}", env!("CARGO_PKG_VERSION"));
    let boot_seq = flash::with_flash(|f| f.evlog_append(Kind::Boot, 0, &text))
        .await
        .flatten();

    if let Some(c) = crumb::take() {
        let line = describe(&c);
        let (kind, at) = match &c {
            crumb::Crumb::Panic { uptime_s, .. } => (Kind::Panic, *uptime_s),
            crumb::Crumb::Stall { uptime_s, .. } => (Kind::Stall, *uptime_s),
        };
        let _ = flash::with_flash(|f| f.evlog_append(kind, at, &line)).await;
        println!("evlog: last boot left: {}", line);
    }

    println!(
        "evlog: {} records, boot #{}{}",
        head.count + 1,
        head.next_boot,
        match boot_seq {
            Some(_) => "",
            None => " (the boot record did not land)",
        }
    );
    for i in 0..TAIL {
        let Some(r) = flash::with_flash(|f| f.evlog_read(i)).await.flatten() else {
            break;
        };
        println!("  {}", format(&r));
    }
}

/// One line for a crumb: what the log carries for it.
fn describe(c: &crumb::Crumb) -> heapless::String<TEXT_MAX> {
    let mut line: heapless::String<TEXT_MAX> = heapless::String::new();
    match c {
        crumb::Crumb::Panic { text, pcs, .. } => {
            let _ = line.push_str(text);
            // The frames go after the message, and only as many as fit:
            // the message names the line, the frames name the caller.
            for pc in pcs {
                let mut hex: heapless::String<12> = heapless::String::new();
                let _ = write!(hex, " {:x}", pc);
                if line.push_str(&hex).is_err() {
                    break;
                }
            }
        }
        crumb::Crumb::Stall { stall, .. } => {
            let _ = write!(
                line,
                "{} silent {} s in {}",
                stall.task.as_str(),
                stall.silent_ms / 1000,
                stall.phase.as_str()
            );
        }
    }
    line
}

/// A record as the console shows it.
pub fn format(r: &Record) -> heapless::String<{ TEXT_MAX + 40 }> {
    let mut s = heapless::String::new();
    let _ = write!(
        s,
        "#{} b{} +{}s {}: {}",
        r.seq,
        r.boot,
        r.uptime_s,
        r.kind.as_str(),
        r.text()
    );
    s
}

/// The `i`-th newest record.
pub async fn read(i: usize) -> Option<Record> {
    flash::with_flash(|f| f.evlog_read(i)).await.flatten()
}

/// How many records the log holds.
pub async fn count() -> usize {
    flash::with_flash(|f| f.evlog_count()).await.unwrap_or(0)
}

/// Erase the log.
pub async fn erase() -> bool {
    flash::with_flash(|f| f.evlog_erase()).await.unwrap_or(false)
}
