//! The heartbeats, the monitor and the hardware watchdog behind it.
//!
//! The policy - which tasks are watched, how long each may go quiet,
//! what is written down - is [`midair_proto::supervise`], host-tested
//! and walked exhaustively. What is here is the firmware's half: the
//! atomics the heartbeats land in, the task that reads them, and the
//! timer-group watchdog it feeds.
//!
//! A heartbeat is a task saying what it is doing and that it still is.
//! The two loops beat at the top of every pass and set their phase as
//! they go; a wait that is allowed to last - advertising with nobody
//! interested, a connected phone with nothing to say, the modem's off
//! period, the park before a sleep - is wrapped in [`guarded`], which
//! beats on the task's behalf once a second for as long as the wait
//! runs. A wait that is *not* allowed to last - a notify, a flash write,
//! the controller coming up - is left uncovered on purpose: those are
//! what a stall looks like.
//!
//! The monitor runs on the first core. If that core stops, or if the
//! monitor blocks trying to write a stall to flash because the dead core
//! holds the lock the write needs, nothing feeds the watchdog and the
//! watchdog resets the board. The crumb is written before either.

use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::peripherals::TIMG1;
use esp_hal::timer::timg::{MwdtStage, Wdt};
use midair_proto::supervise::{
    Phase, Stall, Supervisor, Task, Verdict, MONITOR_PERIOD_MS, WDT_TIMEOUT_MS,
};
use portable_atomic::{AtomicU32, AtomicU8, Ordering};

use crate::{crumb, evlog};

/// When each task last beat, as the low word of the millisecond clock.
/// A word rather than the whole clock so a beat is one store with no
/// lock behind it; the age is a wrapping difference, so the wrap at
/// forty-nine days costs nothing.
static SEEN_MS: [AtomicU32; Task::ALL.len()] = [const { AtomicU32::new(0) }; Task::ALL.len()];
static PHASE: [AtomicU8; Task::ALL.len()] = [const { AtomicU8::new(0) }; Task::ALL.len()];

/// Tasks this build does not run, one bit each by task index. An
/// isolation build leaves a whole loop out on purpose, and a loop that
/// was never started is not one that stalled.
static UNWATCHED: AtomicU8 = AtomicU8::new(0);

/// When the oldest session operation still in flight started, as the low
/// word of the clock, or 0 for none; and how many are in flight. Two arms
/// of the session - the notifier and the write handler - can each have
/// one going, so the count is what says when the last one finished and
/// the time is the first one's, which is the one that is stuck if any is.
/// What lets the session's own heartbeat stop when the BLE stack has
/// stopped answering underneath a connection that is still, as far as
/// the controller knows, up.
static SESSION_BUSY_SINCE: AtomicU32 = AtomicU32::new(0);
static SESSION_OPS: AtomicU8 = AtomicU8::new(0);

/// How long a session operation may take before the session's heartbeat
/// stops on its behalf. A notify completes inside a connection interval
/// or fails; a config write reaches the card and the flash; a bulk
/// chunk erases a sector. Seconds is far outside all of them.
const SESSION_OP_MS: u32 = 5_000;

const fn index(task: Task) -> usize {
    match task {
        Task::Loop => 0,
        Task::Serve => 1,
    }
}

fn now_ms32() -> u32 {
    Instant::now().as_millis() as u32
}

/// `task` is alive and in `phase`. Two stores; safe from anywhere.
pub fn beat(task: Task, phase: Phase) {
    let i = index(task);
    PHASE[i].store(phase.as_wire(), Ordering::Relaxed);
    SEEN_MS[i].store(now_ms32(), Ordering::Relaxed);
}

/// This build never starts `task`: leave it out of the check.
pub fn unwatch(task: Task) {
    UNWATCHED.fetch_or(1 << index(task), Ordering::Relaxed);
}

fn watched(task: Task) -> bool {
    UNWATCHED.load(Ordering::Relaxed) & (1 << index(task)) == 0
}

/// The session has started something that must finish.
pub fn session_busy() {
    if SESSION_OPS.fetch_add(1, Ordering::Relaxed) == 0 {
        SESSION_BUSY_SINCE.store(now_ms32().max(1), Ordering::Relaxed);
    }
}

/// It finished.
pub fn session_free() {
    let ops = SESSION_OPS.load(Ordering::Relaxed);
    if ops <= 1 {
        SESSION_OPS.store(0, Ordering::Relaxed);
        SESSION_BUSY_SINCE.store(0, Ordering::Relaxed);
    } else {
        SESSION_OPS.store(ops - 1, Ordering::Relaxed);
    }
}

/// Nothing is in flight: the session is starting, or it ended with an
/// arm cancelled mid-operation, which never gets to say it finished.
pub fn session_reset() {
    SESSION_OPS.store(0, Ordering::Relaxed);
    SESSION_BUSY_SINCE.store(0, Ordering::Relaxed);
}

/// Whether a session operation has been in flight longer than one may.
fn session_stuck() -> bool {
    let since = SESSION_BUSY_SINCE.load(Ordering::Relaxed);
    since != 0 && now_ms32().wrapping_sub(since) > SESSION_OP_MS
}

async fn tick_forever(task: Task, phase: Phase) {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        if task == Task::Serve && phase == Phase::Session && session_stuck() {
            continue;
        }
        beat(task, phase);
    }
}

/// Run `fut` as a wait `task` is allowed to spend any length of time in,
/// beating for it once a second until it completes.
pub async fn guarded<F: Future>(task: Task, phase: Phase, fut: F) -> F::Output {
    beat(task, phase);
    match select(fut, tick_forever(task, phase)).await {
        Either::First(out) => out,
        Either::Second(()) => unreachable!(),
    }
}

/// The heartbeats as the policy reads them: rebuilt from the ages, so
/// the policy sees a clock that never wraps.
fn snapshot() -> (Supervisor, u64) {
    let now32 = now_ms32();
    let now = u64::from(u32::MAX) + u64::from(now32);
    let mut s = Supervisor::new(now);
    for task in Task::ALL {
        if !watched(task) {
            continue;
        }
        let i = index(task);
        let age = u64::from(now32.wrapping_sub(SEEN_MS[i].load(Ordering::Relaxed)));
        let phase = Phase::from_wire(PHASE[i].load(Ordering::Relaxed)).unwrap_or_default();
        s.beat(task, phase, now - age);
    }
    (s, now)
}

/// Arm the hardware watchdog. Called once, early in the boot, so it
/// covers the boot too; the monitor feeds it from then on.
pub fn arm(wdt: &mut Wdt<TIMG1<'static>>) {
    wdt.set_timeout(
        MwdtStage::Stage0,
        esp_hal::time::Duration::from_millis(u64::from(WDT_TIMEOUT_MS)),
    );
    wdt.enable();
}

/// Read the heartbeats every period; feed the watchdog while every task
/// is inside its bound, reset the board with the stall written down when
/// one is not. Between checks, what other tasks queued for the event log
/// is written to flash from here, so no task does its own flash write
/// for a log line.
#[embassy_executor::task]
pub async fn monitor_task(mut wdt: Wdt<TIMG1<'static>>) {
    for task in Task::ALL {
        beat(task, Phase::Boot);
    }
    loop {
        Timer::after(Duration::from_millis(u64::from(MONITOR_PERIOD_MS))).await;
        let (sup, now) = snapshot();
        match sup.check(now) {
            Verdict::Alive => {
                wdt.feed();
                evlog::flush().await;
            }
            Verdict::Stalled(stall) => stalled(stall).await,
        }
    }
}

/// A task is past its bound. RTC RAM first, with no lock; then the
/// console and the flash, either of which may block on a lock the dead
/// core holds - and then the watchdog, unfed, finishes the job with the
/// crumb intact.
async fn stalled(stall: Stall) -> ! {
    let uptime_s = Instant::now().as_secs() as u32;
    crumb::record_stall(&stall, uptime_s);
    qprintln!(
        "watchdog: {} silent {} s in {}, resetting",
        stall.task.as_str(),
        stall.silent_ms / 1000,
        stall.phase.as_str()
    );
    if evlog::write_stall(&stall, uptime_s).await {
        // In the log already; the boot after must not write it twice.
        crumb::clear();
    }
    Timer::after(Duration::from_millis(100)).await;
    esp_hal::system::software_reset()
}
