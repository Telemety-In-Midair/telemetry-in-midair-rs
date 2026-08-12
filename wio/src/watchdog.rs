//! Independent Watchdog (IWDG) driver for boot safety.
//!
//! The IWDG runs from the LSI oscillator (~32 kHz) and is independent of the
//! main system clock.  Once started it **cannot be stopped** — this is by
//! design and ensures that a stuck firmware will always reset.
//!
//! Usage:
//! 1. Call [`start`] early in `init`, **before** `confirm_boot()`.
//! 2. Call [`feed`] periodically in the main loop (must be called at least
//!    once per timeout period).
//!
//! If the new firmware never calls `confirm_boot` or hangs, the watchdog
//! fires a reset.  The bootloader sees `SWAP_COMPLETE` (app didn't confirm)
//! and reverts to the previous firmware.

use stm32wlxx_hal::pac;

// IWDG key register magic values
const KEY_ENABLE: u16 = 0xCCCC;
const KEY_RELOAD: u16 = 0xAAAA;
const KEY_UNLOCK: u16 = 0x5555;

/// Start the IWDG with the given timeout in milliseconds.
///
/// The LSI clock is ~32 kHz.  With prescaler /64 the counter ticks at
/// ~500 Hz, giving a maximum timeout of ~8 s (reload 0xFFF = 4095).
///
/// Common values:
/// - 5000 ms → reload ≈ 2500
/// - 8000 ms → reload ≈ 4000
///
/// Clamps to the hardware maximum of 4095.
pub fn start(iwdg: &pac::IWDG, timeout_ms: u32) {
    // Compute reload value: LSI ≈ 32 kHz, prescaler = 64 → tick = 500 Hz
    let ticks = (timeout_ms * 500) / 1000;
    let reload = ticks.min(4095) as u16;

    // Enable IWDG
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_ENABLE) });
    // Unlock PR and RLR
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_UNLOCK) });
    // Prescaler = /64 (PR = 4)
    iwdg.pr.write(|w| w.pr().bits(4));
    // Reload value
    iwdg.rlr.write(|w| w.rl().bits(reload));
    // Wait for prescaler (PVU) and reload (RVU) updates only.
    // Checking all bits would also wait on WVU (window value update),
    // which may never clear if WINR was not written, causing a hang.
    while iwdg.sr.read().bits() & 0x03 != 0 {}
    // Initial reload
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });
}

/// Feed (reload) the watchdog counter, preventing a reset.
///
/// Must be called at least once per timeout period.
#[inline]
pub fn feed(iwdg: &pac::IWDG) {
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });
}

/// Feed the watchdog without holding the peripheral.
///
/// The key register is write-only and the reload key is idempotent, so this
/// cannot disturb whatever the owner of the [`pac::IWDG`] is doing, and a
/// write before [`start`] is simply ignored.
///
/// It exists for the bounded wait loops that sit below the main loop and
/// cannot reach the peripheral: a blocking LoRa transmit and an SD block
/// transfer. Both carry their own deadline, so feeding from inside them says
/// "still waiting on something that will time out", not "trust me". Loops
/// with no deadline of their own - `wait_on_busy` in particular - are
/// deliberately left unfed, because a radio that never releases BUSY is
/// exactly what the watchdog is for.
#[inline]
pub fn feed_now() {
    // SAFETY: a single write of the reload key to a write-only register.
    unsafe {
        (*pac::IWDG::PTR).kr.write(|w| w.key().bits(KEY_RELOAD));
    }
}
