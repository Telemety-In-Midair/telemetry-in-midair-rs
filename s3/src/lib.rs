#![no_std]

/// Print to the console *and* queue the same line for the BLE log
/// characteristic, so a connected app sees what the console sees.
///
/// This was the WIO-E5's `status_println!`, which put the line on the UART
/// link for the ESP to forward. One MCU sends it to both ends directly.
#[macro_export]
macro_rules! status_println {
    ($($arg:tt)*) => {{
        if !$crate::state::transfer_active() {
            ::esp_println::println!($($arg)*);
        }
        $crate::state::log_line(format_args!($($arg)*));
    }};
}

/// Console only, and silent while a bulk transfer owns the USB port (see
/// [`state::set_transfer_active`](crate::state::set_transfer_active)).
#[macro_export]
macro_rules! qprintln {
    ($($arg:tt)*) => {{
        if !$crate::state::transfer_active() {
            ::esp_println::println!($($arg)*);
        }
    }};
}

/// Per-event detail, on top of the events that are always logged.
///
/// Gated at runtime on the config's `verbose` key, so a deployed board can
/// be quieted without a reflash - and so a board that has not read its card
/// yet is talkative, which is exactly when one that fails to come up needs
/// to be saying something.
#[macro_export]
macro_rules! vprintln {
    ($($arg:tt)*) => {{
        if $crate::state::verbose() && !$crate::state::transfer_active() {
            ::esp_println::println!($($arg)*);
        }
    }};
}

pub mod flash;
pub mod gps;
pub mod node;
pub mod oled;
pub mod radio;
pub mod sdlog;
pub mod settings;
pub mod state;
pub mod sx1262;
pub mod usb;
pub mod xfer;
