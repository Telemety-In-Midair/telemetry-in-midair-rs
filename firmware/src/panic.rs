//! The panic handler: write it down, say it, reset.
//!
//! The one the backtrace crate ships prints the message and the frames
//! and then spins its core forever with interrupts off. On this board
//! that is the worst of the options. A core that spins holding a
//! critical section takes the other core with it the next time that
//! core needs one - which the hardware loop does every pass and the BLE
//! host does every event - so a panic on either core is, within
//! milliseconds, both cores stopped, the LEDs frozen at whatever the
//! last pass set, and a board that answers nobody until somebody finds
//! it and pulls the battery. And the message went to a console nothing
//! was reading.
//!
//! This one writes the message, the location and the top of the
//! backtrace into RTC RAM first, with atomics and nothing else, because
//! nothing else can be trusted at this point: the heap may be what
//! failed, the lock the console needs may be what the other core holds.
//! Then it prints, which may block on that lock - the watchdog resets
//! the board if it does, and the crumb is already safe. Then it resets,
//! and the boot after writes the crumb into the event log.
//!
//! The HAL routes CPU exceptions - a bad load, an illegal instruction, a
//! write to the stack guard - through `panic!` as well, so all of them
//! arrive here.

use esp_hal::time::{Duration, Instant};
use esp_println::println;
use portable_atomic::{AtomicBool, Ordering};

use crate::crumb;

/// A panic inside the handler - the print, most likely - goes straight
/// to the reset rather than around again.
static PANICKING: AtomicBool = AtomicBool::new(false);

/// How long the console is given to drain before the reset. The USB
/// Serial/JTAG FIFO is 64 bytes and the host reads it in milliseconds
/// when there is a host; when there is none nothing is lost by waiting.
const DRAIN_MS: u64 = 250;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    if PANICKING.swap(true, Ordering::SeqCst) {
        esp_hal::system::software_reset();
    }
    let uptime_s = Instant::now().duration_since_epoch().as_secs() as u32;

    // The frames first: capturing them walks this core's own stack and
    // register window and touches nothing shared.
    let backtrace = esp_backtrace::Backtrace::capture();
    let mut pcs = [0u32; crumb::PCS_MAX];
    for (slot, frame) in pcs.iter_mut().zip(backtrace.frames()) {
        *slot = frame.program_counter() as u32;
    }
    crumb::record_panic(info, &pcs, uptime_s);

    println!("");
    println!("====================== PANIC ======================");
    println!("{}", info);
    println!("");
    println!("Backtrace:");
    for frame in backtrace.frames() {
        println!("0x{:x}", frame.program_counter());
    }
    println!("");
    println!("resetting; the boot after this one logs it");

    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(DRAIN_MS) {}
    esp_hal::system::software_reset()
}
