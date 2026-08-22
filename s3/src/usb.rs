//! The USB console: firmware text out, framed commands in.
//!
//! The host tools (`pixi run wio-config`) speak the same framed protocol
//! over this port that an app speaks over BLE - see `midair_proto::link`,
//! module `usb`. That framing was the ESP32-C6's UART link to the WIO-E5 as
//! well as its console; one module has nothing to link to, so what survives
//! is the console half, and this is the board end of it.
//!
//! esp-println writes to the same port with no arbitration, so console text
//! from another task would land in the middle of a reply frame and cost the
//! host a retry per collision. That is what
//! [`state::set_transfer_active`](crate::state::set_transfer_active) is
//! for; the frames win and everything discretionary goes quiet.

use embassy_time::{with_timeout, Duration, Instant};
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;
use midair_proto::bulk::Owner;
use midair_proto::link::{self, FrameBuf, FrameParser};

use crate::state;
use crate::xfer;

/// Version reported in the `PING` ack, so a host tool can tell which
/// protocol it is talking to before it starts a transfer.
pub const FIRMWARE_VERSION: u16 = 1;

/// Write a frame, waiting for the shared IN FIFO both before and after.
///
/// The USB Serial/JTAG peripheral has one 64-byte IN FIFO, and a write that
/// does not fit is silently truncated rather than blocking - `write` only
/// tests whether the endpoint is free. Flushing first is how this waits for
/// it actually to be.
async fn send_frame(tx: &mut UsbSerialJtagTx<'static, Async>, bytes: &[u8]) {
    use embedded_io_async::Write as _;
    let _ = tx.flush().await;
    let _ = tx.write_all(bytes).await;
    let _ = tx.flush().await;
}

#[embassy_executor::task]
pub async fn usb_task(
    mut rx: UsbSerialJtagRx<'static, Async>,
    mut tx: UsbSerialJtagTx<'static, Async>,
) {
    use embedded_io_async::Read as _;
    let mut parser = FrameParser::new();
    let mut out = FrameBuf::new();
    let mut buf = [0u8; 64];
    loop {
        // While a transfer is in flight, bound the wait: a host that
        // vanished must not leave the board quiet and off the air, and the
        // read is the only thing that would otherwise block forever.
        let read = if state::transfer_active() {
            match with_timeout(Duration::from_secs(1), rx.read(&mut buf)).await {
                Ok(r) => r,
                Err(_) => {
                    xfer::expire(Instant::now().as_millis()).await;
                    continue;
                }
            }
        } else {
            rx.read(&mut buf).await
        };
        // The read cannot fail (its error type is `Infallible`); a zero
        // length is the case worth skipping.
        let n = read.unwrap_or(0);
        if n == 0 {
            continue;
        }

        for &b in &buf[..n] {
            if !parser.feed(b) {
                continue;
            }
            // Copy the frame out so the parser (and its borrow of the
            // parser's buffer) is free across the awaits below.
            let cmd;
            let len;
            let mut payload = [0u8; link::MAX_PAYLOAD];
            {
                let f = parser.frame();
                cmd = f.cmd;
                len = f.payload.len();
                payload[..len].copy_from_slice(f.payload);
            }
            match cmd {
                link::usb::PING => {
                    let v = FIRMWARE_VERSION.to_le_bytes();
                    out.build(link::resp::ACK, &[link::usb::PING, v[0], v[1]]);
                    send_frame(&mut tx, out.as_bytes()).await;
                }
                link::usb::INFO => {
                    let a = state::ble_address();
                    // Most-significant octet first, so it prints directly
                    // and matches the boot line.
                    let mut reply = [link::usb::INFO, 0, 0, 0, 0, 0, 0];
                    for i in 0..6 {
                        reply[1 + i] = a[5 - i];
                    }
                    out.build(link::resp::ACK, &reply);
                    send_frame(&mut tx, out.as_bytes()).await;
                }
                link::usb::SLEEP => {
                    // Same meaning as the BLE `CFG_SLEEP_NOW` write, and the
                    // same resolver, so a board naps for the length the tool
                    // printed rather than one the firmware worked out
                    // separately. A short payload is a nap of 0, which
                    // resolves to the configured cadence.
                    let asked = payload
                        .get(..4)
                        .and_then(|b| <[u8; 4]>::try_from(b).ok())
                        .map(u32::from_le_bytes)
                        .unwrap_or(0);
                    let secs = midair_proto::ble::resolve_sleep_now(
                        asked,
                        crate::settings::get().sleep_interval_s,
                    );
                    // The ack goes out before the request, because the
                    // request takes the port down with it.
                    let v = (secs.min(u32::from(u16::MAX)) as u16).to_le_bytes();
                    out.build(link::resp::ACK, &[link::usb::SLEEP, v[0], v[1]]);
                    send_frame(&mut tx, out.as_bytes()).await;
                    state::request_sleep_now(secs);
                }
                link::usb::BULK => {
                    let (ack, alen) =
                        xfer::handle(Owner::Usb, Instant::now().as_millis(), &payload[..len]).await;
                    out.build(link::usb::BULK_ACK, &ack[..alen]);
                    send_frame(&mut tx, out.as_bytes()).await;
                }
                _ => {}
            }
        }
    }
}
