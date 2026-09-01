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
//! in that memory at a cold boot; only a cold boot pays for the flash read.

use midair_proto::ble::Mode;
use midair_proto::session::Stored;
use portable_atomic::{AtomicU32, Ordering};

/// Marks the RTC RAM copy as ours ("mida").
const MAGIC: u32 = 0x6D69_6461;

// esp-hal's `Persistable` marker only covers atomics and primitives, hence
// a static per field rather than one struct.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static MAGIC_WORD: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static INTERVAL: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static FLAGS: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static ADV_WINDOW: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static BLE_OFF: AtomicU32 = AtomicU32::new(0);
/// The *live* mode, which is the one difference between this copy and the
/// flash record: RTC RAM may say idle, flash never does.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static MODE: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static IDLE_TIMEOUT: AtomicU32 = AtomicU32::new(0);

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

/// The current settings. An unconfigured board reads back
/// [`Stored::new`], which is awake, powered and never sleeping.
pub fn get() -> Stored {
    if MAGIC_WORD.load(Ordering::Relaxed) == MAGIC {
        Stored {
            sleep_interval_s: INTERVAL.load(Ordering::Relaxed),
            flags: FLAGS.load(Ordering::Relaxed),
            adv_window_s: ADV_WINDOW.load(Ordering::Relaxed),
            ble_off_s: BLE_OFF.load(Ordering::Relaxed),
            // A word that is not a mode can only be corruption, and the
            // safe reading of it is the mode a board can be woken out of.
            mode: Mode::from_wire(MODE.load(Ordering::Relaxed) as u8).unwrap_or_default(),
            idle_timeout_s: IDLE_TIMEOUT.load(Ordering::Relaxed),
        }
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
    }
    INTERVAL.store(s.sleep_interval_s, Ordering::Relaxed);
    FLAGS.store(s.flags, Ordering::Relaxed);
    ADV_WINDOW.store(s.adv_window_s, Ordering::Relaxed);
    BLE_OFF.store(s.ble_off_s, Ordering::Relaxed);
    MODE.store(u32::from(s.mode.as_wire()), Ordering::Relaxed);
    IDLE_TIMEOUT.store(s.idle_timeout_s, Ordering::Relaxed);
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

/// Whether RTC RAM holds no copy, i.e. this is a cold boot rather than a
/// deep-sleep wake.
pub fn is_cold() -> bool {
    MAGIC_WORD.load(Ordering::Relaxed) != MAGIC
}

/// On a cold boot, adopt whatever reached flash last. Returns what was
/// restored, if anything.
pub async fn restore() -> Option<Stored> {
    if !is_cold() {
        return None;
    }
    let saved = crate::flash::with_flash(|f| f.load_settings()).await??;
    set(saved);
    Some(saved)
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
