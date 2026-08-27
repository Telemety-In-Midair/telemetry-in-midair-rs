//! Idle accounting, so "does the core actually halt?" is a number.
//!
//! esp-rtos runs a lowest-priority idle task whose whole body is
//! `loop { waiti 0 }` - the Xtensa halt-until-interrupt instruction, which
//! gates the CPU clock until something needs servicing. Whether that path
//! is *reached* is a different question from whether it exists: one future
//! that returns `Poll::Ready` in a loop, or one driver that polls a status
//! register instead of awaiting an interrupt, and the executor never runs
//! out of work, the idle task never gets scheduled, and the core spins at
//! full current with nothing to show for it. That failure is invisible
//! without a meter and indistinguishable from "this chip is just thirsty".
//!
//! `esp_rtos::start_with_idle_hook` takes the idle task's body, so this
//! substitutes one that brackets each `waiti` with a timestamp. What comes
//! out is the fraction of wall time the core spent halted, which is the
//! direct answer rather than an inference from current.
//!
//! Cost is one systimer read either side of each halt. At the wake rates
//! this firmware produces - a 100 Hz poll loop plus interrupts - that is
//! well under a microsecond per millisecond of wall time.

use esp_hal::time::Instant;
use portable_atomic::{AtomicU64, Ordering};

/// Microseconds spent halted, cumulative since boot.
static IDLE_US: AtomicU64 = AtomicU64::new(0);
/// Times the core came out of `waiti`, cumulative since boot.
static WAKEUPS: AtomicU64 = AtomicU64::new(0);

/// The idle task body. Installed by `main` via
/// [`esp_rtos::start_with_idle_hook`].
///
/// Never returns: this *is* the idle task, not something called from it.
pub extern "C" fn hook() -> ! {
    loop {
        let entered = Instant::now();
        // SAFETY: `waiti 0` halts until an interrupt of any priority. It
        // needs no operands and touches no state; the only requirement is
        // that interrupts are enabled, which they are in task context.
        unsafe { core::arch::asm!("waiti 0") };
        let halted = Instant::now()
            .duration_since_epoch()
            .as_micros()
            .saturating_sub(entered.duration_since_epoch().as_micros());
        IDLE_US.fetch_add(halted, Ordering::Relaxed);
        WAKEUPS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Cumulative microseconds halted, and wakeup count.
pub fn totals() -> (u64, u64) {
    (
        IDLE_US.load(Ordering::Relaxed),
        WAKEUPS.load(Ordering::Relaxed),
    )
}

/// Percent of wall time the core spent halted over a window, and the wake
/// rate in hertz across it.
///
/// Take `totals()` at each end of the window and pass both, along with how
/// long the window was. Returns `(idle %, wakeups per second)`.
///
/// A healthy board here is high nineties. Anything low means something is
/// polling rather than awaiting, and the wake rate says which: a rate near
/// the 100 Hz loop is the loop, and a much higher one is a driver spinning
/// on a status register.
pub fn window(before: (u64, u64), after: (u64, u64), elapsed_us: u64) -> (u32, u32) {
    if elapsed_us == 0 {
        return (0, 0);
    }
    let idle = after.0.saturating_sub(before.0).min(elapsed_us);
    let wakes = after.1.saturating_sub(before.1);
    (
        ((idle * 100) / elapsed_us) as u32,
        ((wakes * 1_000_000) / elapsed_us) as u32,
    )
}
