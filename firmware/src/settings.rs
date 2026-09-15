//! Power and sleep settings, kept where a deep sleep cannot lose them.
//!
//! What the settings mean, what a config write does to them and when the
//! board sleeps all live in `midair_proto::session`, which is host-tested.
//! This is only where they are kept, and there are two layers of that:
//!
//! - **RTC fast RAM**, which survives deep sleep. A wake check therefore
//!   costs no flash read at all, which matters because the wake check is
//!   the thing that runs every interval forever.
//! - **The `nvs` partition**, which survives a flat cell. RTC RAM does not,
//!   and a sleeping board that came back unconfigured would advertise
//!   continuously until it died again - so the record is mirrored to flash
//!   whenever a write changes something that decides reachability.
//!
//! The magic word is what separates a real RTC RAM copy from whatever was
//! in that memory at a cold boot. The copy is trusted across a deep sleep
//! and nothing else: RTC RAM also survives every reset short of a power
//! cycle - the reset button, a panic, the reset a flashing tool issues -
//! so a boot that is not a deep-sleep wake reads flash and takes what it
//! finds, including nothing. Otherwise a board whose flash had just been
//! erased came back up on the copy, name and all, and wrote it straight
//! back into the flash that had been cleared.

use midair_proto::ble::Mode;
use midair_proto::session::{Stored, KNOBS};
use portable_atomic::{AtomicU32, AtomicU8, Ordering};

/// Marks the RTC RAM copy as ours ("mida").
const MAGIC: u32 = 0x6D69_6461;

// esp-hal's `Persistable` marker only covers atomics and primitives, hence
// a static per field rather than one struct.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static MAGIC_WORD: AtomicU32 = AtomicU32::new(0);
/// The five durations, one word each in the order of the knob table -
/// which is the one place their set is written down.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static DURATIONS: [AtomicU32; KNOBS.len()] = [const { AtomicU32::new(0) }; KNOBS.len()];
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static FLAGS: AtomicU32 = AtomicU32::new(0);
/// The *live* mode, which is the one difference between this copy and the
/// flash record: RTC RAM may say idle, flash never does.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static MODE: AtomicU32 = AtomicU32::new(0);
/// The board's name, zero-padded, a byte per cell.
///
/// Kept here for the reason the advertising window is: a wake check
/// advertises before anything has mounted the card or read the flash, and
/// the name is what a scan list shows. Bytes rather than packed words
/// because that is the shape both the record and the advertisement want.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static NAME: [AtomicU8; midair_proto::ble::NAME_FIELD_LEN] =
    [const { AtomicU8::new(0) }; midair_proto::ble::NAME_FIELD_LEN];

/// How many deep sleeps this board has woken from since its last cold
/// boot, and the seconds it was last told to sleep for.
///
/// Pure instrumentation, and it earns its two words: a deep sleep is a full
/// reset, so from the console a board that sleeps and a board that resets in
/// a loop produce the same boot banner. The counter is what separates them,
/// and it is the first thing to look at when the question is whether sleep
/// is working at all.
///
/// Zeroed by [`set`] when it stamps the magic word, which is the only thing
/// that makes them readable: RTC RAM is not zero-initialized, so before
/// that they hold whatever was in the die.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static WAKE_COUNT: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static LAST_SLEEP_S: AtomicU32 = AtomicU32::new(0);
/// The RTC counter as the last deep sleep was entered, in milliseconds.
///
/// The RTC main timer is what the sleep's own wake source counts against,
/// so unlike every other clock on the board it keeps running through the
/// sleep. Stamping it on the way down and reading it on the way up gives
/// the one number the wake path has never had: how long the wake itself
/// takes, which is what anything trying to reach a sleeping board has to
/// outlast.
///
/// Milliseconds rather than the counter's microseconds, and a u32, because
/// the persistent statics are one word each; it wraps after 49 days of RTC
/// uptime and the read is a wrapping subtraction, so a wrap costs one
/// nonsense reading rather than a wrong number forever.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static SLEEP_AT_MS: AtomicU32 = AtomicU32::new(0);

/// Deep sleeps entered before the hardware loop finished parking, since
/// the last cold boot. Each one is an interval spent with the receiver or
/// the radio still drawing, which nothing else reports.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PARKS_MISSED: AtomicU32 = AtomicU32::new(0);

/// The current settings. An unconfigured board reads back
/// [`Stored::new`], which is awake, powered and never sleeping.
pub fn get() -> Stored {
    if MAGIC_WORD.load(Ordering::Relaxed) == MAGIC {
        let mut s = Stored {
            flags: FLAGS.load(Ordering::Relaxed),
            // A word that is not a mode can only be corruption, and the
            // safe reading of it is the mode a board can be woken out of.
            mode: Mode::from_wire(MODE.load(Ordering::Relaxed) as u8).unwrap_or_default(),
            name: core::array::from_fn(|i| NAME[i].load(Ordering::Relaxed)),
            ..Stored::new()
        };
        for spec in &KNOBS {
            spec.set(&mut s, DURATIONS[spec.knob as usize].load(Ordering::Relaxed));
        }
        s
    } else {
        Stored::new()
    }
}

pub fn set(s: Stored) {
    // The instrumentation words come with the magic word or not at all.
    //
    // Persistent RTC RAM is not zero-initialized, and `note_sleep` stamps
    // the magic word without touching these two - so on the first sleep
    // after a cold boot the word says "this block is ours" while
    // `WAKE_COUNT` still holds whatever was in that memory. The wake on the
    // far side then reports a nonsense count, which is exactly the number
    // the console tells a reader to trust first: a board that sleeps and a
    // board that resets in a loop are told apart by it.
    if MAGIC_WORD.load(Ordering::Relaxed) != MAGIC {
        WAKE_COUNT.store(0, Ordering::Relaxed);
        LAST_SLEEP_S.store(0, Ordering::Relaxed);
        PARKS_MISSED.store(0, Ordering::Relaxed);
    }
    for spec in &KNOBS {
        DURATIONS[spec.knob as usize].store(spec.get(&s), Ordering::Relaxed);
    }
    FLAGS.store(s.flags, Ordering::Relaxed);
    MODE.store(u32::from(s.mode.as_wire()), Ordering::Relaxed);
    for (cell, b) in NAME.iter().zip(s.name) {
        cell.store(b, Ordering::Relaxed);
    }
    MAGIC_WORD.store(MAGIC, Ordering::Relaxed);
}

/// Set the live mode without touching flash.
///
/// The boot path and the wake-check promotion both use this. Neither is a
/// settings change an app made, and neither may cost a flash write: a
/// promotion happens every time somebody connects to a stored board, and
/// the mode it moves to is one that never reaches flash anyway.
pub fn set_mode(mode: Mode) {
    let mut stored = get();
    stored.mode = mode;
    set(stored);
}

/// The live mode.
pub fn mode() -> Mode {
    get().mode
}

/// Adopt the `[power]` section of a config file, returning whether anything
/// changed. The policy - which keys apply and which are absent - is
/// [`Stored::adopt_power`], so it is host-tested rather than decided here.
///
/// The caller decides *when* this is allowed to run, and the answer is a
/// cold boot only. On a deep-sleep wake the RTC copy may hold a duty cycle
/// an app set live, and re-reading the card would undo it every interval.
pub fn adopt_power(p: &midair_proto::radiocfg::PowerConfig) -> bool {
    let mut stored = get();
    if !stored.adopt_power(p) {
        return false;
    }
    set(stored);
    true
}

/// Note the sleep that is about to happen, so the wake on the far side can
/// report it. Called immediately before `sleep_deep`.
pub fn note_sleep(interval_s: u32) {
    LAST_SLEEP_S.store(interval_s, Ordering::Relaxed);
    // A board that has never taken a config write has no magic word, and
    // without one the wake on the far side reads as a cold boot and does
    // not count. Stamping it needs the settings written alongside, not the
    // word on its own: persistent RTC RAM is not zero-initialized, so a
    // magic word in front of never-written interval/flags words would hand
    // the next boot cold-boot garbage as if it were a stored config.
    // `set(get())` resolves to the RTC copy or to `Stored::new`, and writes
    // whichever it was.
    set(get());
}

/// Count a wake from deep sleep and return `(wake number, seconds asked
/// for)`. Called once at boot, and only when the wake cause says timer.
pub fn note_wake() -> (u32, u32) {
    let n = WAKE_COUNT.load(Ordering::Relaxed).saturating_add(1);
    WAKE_COUNT.store(n, Ordering::Relaxed);
    (n, LAST_SLEEP_S.load(Ordering::Relaxed))
}

/// Wakes counted since the last cold boot.
pub fn wake_count() -> u32 {
    if MAGIC_WORD.load(Ordering::Relaxed) == MAGIC {
        WAKE_COUNT.load(Ordering::Relaxed)
    } else {
        0
    }
}

/// Count a deep sleep entered over a park that did not finish.
pub fn note_park_missed() {
    // Stamped alongside the settings for the reason `note_sleep` stamps
    // them: a count in front of no magic word reads as garbage next boot.
    set(get());
    PARKS_MISSED.store(PARKS_MISSED.load(Ordering::Relaxed).saturating_add(1), Ordering::Relaxed);
}

/// Stamp the RTC counter as a deep sleep is entered.
pub fn note_sleep_at(rtc_ms: u32) {
    // Stamped alongside the settings for the reason the counters are: a
    // stamp in front of no magic word reads as garbage on the next boot.
    set(get());
    SLEEP_AT_MS.store(rtc_ms, Ordering::Relaxed);
}

/// The stamp the last deep sleep left, or `None` if this boot did not come
/// through one that took a stamp.
pub fn sleep_stamp_ms() -> Option<u32> {
    if MAGIC_WORD.load(Ordering::Relaxed) == MAGIC {
        Some(SLEEP_AT_MS.load(Ordering::Relaxed))
    } else {
        None
    }
}

/// Parks missed since the last cold boot.
pub fn parks_missed() -> u32 {
    if MAGIC_WORD.load(Ordering::Relaxed) == MAGIC {
        PARKS_MISSED.load(Ordering::Relaxed)
    } else {
        0
    }
}

/// Whether RTC RAM holds no copy.
fn is_cold() -> bool {
    MAGIC_WORD.load(Ordering::Relaxed) != MAGIC
}

/// Drop the RTC RAM copy. The next `get` reads defaults and the next `set`
/// stamps a fresh copy, instrumentation zeroed.
fn clear() {
    MAGIC_WORD.store(0, Ordering::Relaxed);
}

/// What [`restore`] found to start the boot from.
#[derive(Clone, Copy, Debug)]
pub enum Restored {
    /// A deep-sleep wake with its RTC RAM copy intact: nothing was read.
    Kept,
    /// Flash held a record, and it is the settings now.
    Flash(Stored),
    /// Flash held nothing, so the settings are defaults. `dropped_rtc` says
    /// an RTC RAM copy from before the reset was there and was discarded
    /// with them - which is what a flash erase followed by a reset looks
    /// like from here.
    Defaults { dropped_rtc: bool },
}

/// Adopt whatever reached flash last, unless this is a deep-sleep wake
/// with its RTC RAM copy intact - the one case the copy is trusted over
/// flash, since it may hold a mode an app set live that never goes to
/// flash. Any other boot is a cold one as far as the settings go, whatever
/// RTC RAM happens to remember.
pub async fn restore(woke_from_sleep: bool) -> Restored {
    if woke_from_sleep && !is_cold() {
        return Restored::Kept;
    }
    let had_rtc = !is_cold();
    match crate::flash::with_flash(|f| f.load_settings()).await.flatten() {
        Some(saved) => {
            set(saved);
            Restored::Flash(saved)
        }
        None => {
            clear();
            Restored::Defaults {
                dropped_rtc: had_rtc,
            }
        }
    }
}

/// Forget everything this board stores about itself: the RTC RAM copy,
/// and the settings record and the config backup in flash. Returns
/// whether the flash records were erased; the RTC copy is gone either
/// way. The caller restarts the board, which is what makes the loops
/// forget what they had read.
pub async fn wipe() -> bool {
    clear();
    crate::flash::with_flash(|f| f.wipe()).await.unwrap_or(false)
}

/// Mirror the current settings to flash. Best effort: see
/// [`crate::flash::Flash::save_settings`].
pub async fn save() {
    let stored = get();
    let ok = crate::flash::with_flash(|f| f.save_settings(&stored))
        .await
        .unwrap_or(false);
    if !ok {
        crate::qprintln!("nvs: save failed, settings are volatile this session");
    }
}

/// The name this board advertises under: the stored label behind the
/// firmware's prefix, or the board's address if it has never been named.
///
/// Built here rather than at each use so the scan response, the GAP name,
/// the name characteristic and the console line cannot disagree about what
/// the board is called.
pub fn name() -> heapless::String<{ midair_proto::ble::NAME_MAX }> {
    let stored = get();
    let mut buf = [0u8; midair_proto::ble::NAME_MAX];
    let name = midair_proto::ble::advertised_name(
        stored.label(),
        &crate::state::ble_address(),
        &mut buf,
    );
    // The name is built to fit by construction; an empty string would be an
    // unnamed advertisement, which is worse than a wrong one.
    heapless::String::try_from(name).unwrap_or_default()
}
