//! Platform helper functions for the STM32WLE5.

use stm32wlxx_hal::pac;

/// System clock frequency in Hz. The MSI is raised from the 4 MHz reset
/// default to 16 MHz early in init (see [`raise_sysclk`]) for SD SPI
/// throughput and FAT/NMEA processing headroom; 16 MHz still needs zero
/// flash wait states.
pub const SYSCLK_HZ: u32 = 16_000_000;

/// Raise the MSI clock to 16 MHz. Must be called before the SysTick
/// monotonic is started and before any clock-derived peripheral setup.
pub fn raise_sysclk(rcc: &mut pac::RCC) {
    rcc.cr
        .modify(|_, w| w.msirgsel().set_bit().msirange().range16m());
    while rcc.cr.read().msirdy().bit_is_clear() {}
}

/// Enable the 16 MHz HSI oscillator and wait until it is ready.
///
/// Both USARTs (GPS on USART1, ESP link on USART2) select HSI16 as their
/// kernel clock so the baud rate stays exact regardless of the MSI sysclk.
/// `Uart::new` only selects the source - it does not turn the oscillator
/// on - so this must run before either UART is created. Without it the
/// USART kernel clock is dead: `TXE` never asserts and the first blocking
/// send spins forever (then the watchdog resets the chip).
pub fn enable_hsi16(rcc: &mut pac::RCC) {
    rcc.cr.modify(|_, w| w.hsion().set_bit());
    while rcc.cr.read().hsirdy().is_not_ready() {}
}

/// Milliseconds since boot (via DWT cycle counter).
///
/// Uses wrapping subtraction to handle the DWT counter rollover (~268 s at
/// 16 MHz), producing a monotonically-increasing u32 that wraps only at
/// ~49 days. Enable the cycle counter in init before calling this:
/// ```ignore
/// cx.core.DCB.enable_trace();
/// cx.core.DWT.enable_cycle_counter();
/// ```
pub fn millis() -> u32 {
    use core::sync::atomic::{AtomicU32, Ordering};

    const TICKS_PER_MS: u32 = SYSCLK_HZ / 1000;

    static PREV_CYCLES: AtomicU32 = AtomicU32::new(0);
    static ACCUM_MS: AtomicU32 = AtomicU32::new(0);
    static LEFTOVER: AtomicU32 = AtomicU32::new(0);

    let current = cortex_m::peripheral::DWT::cycle_count();
    let prev = PREV_CYCLES.swap(current, Ordering::Relaxed);
    let elapsed = current.wrapping_sub(prev);
    let leftover = LEFTOVER.load(Ordering::Relaxed);
    let total = leftover.saturating_add(elapsed);
    let new_ms = total / TICKS_PER_MS;
    LEFTOVER.store(total % TICKS_PER_MS, Ordering::Relaxed);
    ACCUM_MS.fetch_add(new_ms, Ordering::Relaxed).wrapping_add(new_ms)
}

/// Random number in `[min, max)`.
///
/// Uses a simple xorshift32 PRNG seeded from the DWT cycle counter.
pub fn random(min: i32, max: i32) -> i32 {
    use core::sync::atomic::{AtomicU32, Ordering};

    static STATE: AtomicU32 = AtomicU32::new(0);

    let mut s = STATE.load(Ordering::Relaxed);
    if s == 0 {
        s = cortex_m::peripheral::DWT::cycle_count();
        if s == 0 {
            s = 1;
        }
    }
    // xorshift32
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    STATE.store(s, Ordering::Relaxed);

    if max <= min {
        return min;
    }
    // Reduce in unsigned space. Going through `(s as i32).unsigned_abs()`
    // returns 2^31 for exactly one state value, which casts back to a negative
    // i32 and, since `%` keeps the sign of its left operand, produced a result
    // below `min` - a repeat jitter that reads as already due.
    let span = max.wrapping_sub(min) as u32;
    min.wrapping_add((s % span) as i32)
}
