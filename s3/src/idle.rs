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
//! substitutes one that counts entries before halting. The rate that comes
//! out answers the question directly - above zero means the core reaches
//! `waiti` - without inferring it from a current reading.
//!
//! It is a rate and not a percentage on purpose. esp-rtos documents that
//! the idle hook's context is not preserved: when an interrupt makes a task
//! ready the scheduler switches away and discards the idle context, so
//! nothing after the `waiti` ever executes. A first version of this
//! timestamped both sides of the halt and reported a flat `idle 0% 0 Hz`,
//! because the second timestamp was unreachable code. Measuring the halted
//! *fraction* needs the exit time, and the exit is only observable from the
//! scheduler's context switch, which is not exposed.
//!
//! Cost is one relaxed increment per idle entry.

use portable_atomic::{AtomicU64, Ordering};

/// Times the idle task has been entered, cumulative since boot.
///
/// Entries, not wakeups. esp-rtos documents that "the idle hook's context
/// is not preserved": when an interrupt makes a task ready the scheduler
/// switches away and **discards** the idle context, so execution never
/// resumes after the `waiti`. Anything written past that instruction is
/// dead code, which is why this is a count taken on the way in rather than
/// a duration measured across the halt.
static ENTRIES: AtomicU64 = AtomicU64::new(0);

/// The idle task body. Installed by `main` via
/// [`esp_rtos::start_with_idle_hook`].
///
/// Never returns: this *is* the idle task, not something called from it.
pub extern "C" fn hook() -> ! {
    loop {
        ENTRIES.fetch_add(1, Ordering::Relaxed);
        // SAFETY: `waiti 0` halts until an interrupt of any priority. It
        // needs no operands and touches no state; the only requirement is
        // that interrupts are enabled, which they are in task context.
        //
        // Execution may never continue past here - see `ENTRIES`. Do not
        // add accounting below this line expecting it to run.
        unsafe { core::arch::asm!("waiti 0") };
    }
}

/// Cumulative idle-task entries.
pub fn entries() -> u64 {
    ENTRIES.load(Ordering::Relaxed)
}

/// Idle entries per second across a window.
///
/// **Above zero means the core is reaching `waiti` and halting**, which is
/// the question worth answering; zero means something is polling instead of
/// awaiting and the core never stops. The rate itself is a coarse read on
/// what is doing the waking - in the same order as the 100 Hz hardware loop
/// is expected, far above it means a driver spinning on a status register.
///
/// This is deliberately not a percentage. Getting a halted *fraction* would
/// need the exit time, and the discarded idle context means the exit is not
/// observable from here; it would have to come from the scheduler's context
/// switch, which esp-rtos does not expose. A rate that answers "yes it
/// halts" honestly beats a percentage that would have to be invented.
pub fn rate(before: u64, after: u64, elapsed_ms: u32) -> u32 {
    if elapsed_ms == 0 {
        return 0;
    }
    ((after.saturating_sub(before) * 1_000) / u64::from(elapsed_ms)) as u32
}
