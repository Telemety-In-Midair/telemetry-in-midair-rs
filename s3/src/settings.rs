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

use midair_proto::session::Stored;
use portable_atomic::{AtomicU32, Ordering};

/// Marks the RTC RAM copy as ours ("mida").
const MAGIC: u32 = 0x6D69_6461;

// esp-hal's `Persistable` marker only covers atomics and primitives, hence
// four statics rather than one struct.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static MAGIC_WORD: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static INTERVAL: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static FLAGS: AtomicU32 = AtomicU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static ADV_WINDOW: AtomicU32 = AtomicU32::new(0);

/// The current settings. An unconfigured board reads back
/// [`Stored::new`], which is awake, powered and never sleeping.
pub fn get() -> Stored {
    if MAGIC_WORD.load(Ordering::Relaxed) == MAGIC {
        Stored {
            sleep_interval_s: INTERVAL.load(Ordering::Relaxed),
            flags: FLAGS.load(Ordering::Relaxed),
            adv_window_s: ADV_WINDOW.load(Ordering::Relaxed),
        }
    } else {
        Stored::new()
    }
}

pub fn set(s: Stored) {
    INTERVAL.store(s.sleep_interval_s, Ordering::Relaxed);
    FLAGS.store(s.flags, Ordering::Relaxed);
    ADV_WINDOW.store(s.adv_window_s, Ordering::Relaxed);
    MAGIC_WORD.store(MAGIC, Ordering::Relaxed);
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
