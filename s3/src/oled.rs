//! A 0.91" 128x32 SSD1306 on I2C: fix, satellites, and what the radio last
//! heard.
//!
//! The board brings GPIO10 and GPIO11 out on the J5 JST-SH along with GND
//! and +3V3, which is a Qwiic/STEMMA-shaped connector without being one -
//! the schematic labels those two nets `GPIO10` and `GPIO11` and nothing
//! else, so *which* is SDA and which is SCL is this firmware's choice
//! rather than the board's. Rather than pick one and make a wrong cable
//! look like a dead display, [`Oled::probe`] tries both.
//!
//! Written in-repo for the same reason the SX1262 driver is: what a status
//! readout needs is a framebuffer, a font and four lines of text, and that
//! is less code than the configuration surface of a general display stack.
//!
//! **This costs power.** A 128x32 panel is 5-15 mA depending on how many
//! pixels are lit, on a board whose whole awake budget is under scrutiny -
//! so the layout leaves most of the panel dark, and the hardware loop
//! blanks it before deep sleep. It is not free and it is not nothing.

use esp_hal::i2c::master::I2c;
use esp_hal::Async;
use midair_proto::link::Telemetry;

/// Both addresses the 0.91" modules ship with. The silkscreen usually says
/// 0x3C; a few boards jumper 0x3D.
const ADDRESSES: [u8; 2] = [0x3C, 0x3D];

pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 32;
/// One byte per 8 vertical pixels, so four pages of 128 columns.
const PAGES: usize = HEIGHT / 8;
const FRAME: usize = WIDTH * PAGES;

/// Glyph cell: a 5x7 font with one column of spacing.
const CELL_W: usize = 6;
/// Characters per line, and lines per screen.
pub const COLS: usize = WIDTH / CELL_W;
pub const ROWS: usize = PAGES;

/// Control byte prefixes. Bit 6 is D/C#: clear selects the command
/// register, set selects display RAM.
const CTRL_CMD: u8 = 0x00;
const CTRL_DATA: u8 = 0x40;

/// Panel initialization for the 128x32 part.
///
/// The two that are specific to this geometry rather than copied from the
/// datasheet's generic sequence are the multiplex ratio (0x1F = 32 rows,
/// against 0x3F for a 128x64) and the COM pin configuration (0x02 for
/// sequential pins with no remap; a 128x64 wants 0x12). Getting either
/// wrong on a 128x32 shows the frame doubled or interlaced rather than
/// blank, which is worth knowing because it looks like a framebuffer bug.
const INIT: &[u8] = &[
    0xAE, // display off while it is configured
    0xD5, 0x80, // clock: divide by 1, oscillator at its reset frequency
    0xA8, 0x1F, // multiplex ratio = 32 rows
    0xD3, 0x00, // no display offset
    0x40, // start line 0
    0x8D, 0x14, // charge pump on - without this the panel stays dark
    0x20, 0x00, // horizontal addressing, so a frame is one linear write
    0xA1, // segment remap: column 127 maps to SEG0
    0xC8, // COM scan direction reversed
    0xDA, 0x02, // COM pin configuration for 32 rows
    0x81, 0x8F, // contrast
    0xD9, 0xF1, // precharge period
    0xDB, 0x40, // VCOMH deselect level
    0xA4, // follow display RAM, rather than forcing every pixel on
    0xA6, // normal, not inverted
    0x2E, // scrolling off
    0xAF, // display on
];

/// 5x7 glyphs for ASCII 0x20..=0x7E, five column bytes each, LSB at the top
/// of the cell. Anything outside the range renders as a space.
const FONT: &[u8; 95 * 5] = &[
    0x00, 0x00, 0x00, 0x00, 0x00, // space
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x14, 0x08, 0x3E, 0x08, 0x14, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x09, 0x01, // F
    0x3E, 0x41, 0x49, 0x49, 0x7A, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x0C, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x3F, 0x40, 0x38, 0x40, 0x3F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x07, 0x08, 0x70, 0x08, 0x07, // Y
    0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x7F, 0x41, 0x41, 0x00, // [
    0x02, 0x04, 0x08, 0x10, 0x20, // backslash
    0x00, 0x41, 0x41, 0x7F, 0x00, // ]
    0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x01, 0x02, 0x04, 0x00, // `
    0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x0C, 0x52, 0x52, 0x52, 0x3E, // g
    0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x20, 0x40, 0x44, 0x3D, 0x00, // j
    0x7F, 0x10, 0x28, 0x44, 0x00, // k
    0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x7C, 0x14, 0x14, 0x14, 0x08, // p
    0x08, 0x14, 0x14, 0x18, 0x7C, // q
    0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x0C, 0x50, 0x50, 0x50, 0x3C, // y
    0x44, 0x64, 0x54, 0x4C, 0x44, // z
    0x00, 0x08, 0x36, 0x41, 0x00, // {
    0x00, 0x00, 0x7F, 0x00, 0x00, // |
    0x00, 0x41, 0x36, 0x08, 0x00, // }
    0x08, 0x04, 0x08, 0x10, 0x08, // ~
];

/// The panel, its address, and the frame being built for it.
pub struct Oled {
    i2c: I2c<'static, Async>,
    address: u8,
    buf: [u8; FRAME],
    /// The frame the panel is currently showing, so an unchanged screen
    /// costs no bus time. The refresh runs at twice the rate the underlying
    /// fields change, so a fix or a packet lands promptly - and about half
    /// of those passes have nothing to send.
    sent: [u8; FRAME],
    /// Whether `sent` describes the panel. False after an init, when what
    /// the panel holds is not known.
    sent_valid: bool,
    /// Whether the panel is powered up. Blanking for sleep clears it, so a
    /// redraw afterwards knows to turn it back on.
    on: bool,
}

impl Oled {
    /// Look for a panel on `i2c`, at either address. `None` when nothing
    /// answers, which is the normal state of a board with no display on J5.
    pub async fn probe(mut i2c: I2c<'static, Async>) -> Option<Self> {
        for address in ADDRESSES {
            // A zero-length write is the cheapest address poll: it drives
            // the address byte and reads the ack bit without leaving the
            // panel in a state that matters if something else is there.
            if i2c.write_async(address, &[]).await.is_ok() {
                let mut oled = Self {
                    i2c,
                    address,
                    buf: [0; FRAME],
                    sent: [0; FRAME],
                    sent_valid: false,
                    on: false,
                };
                oled.init().await.ok()?;
                return Some(oled);
            }
        }
        None
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    async fn cmds(&mut self, bytes: &[u8]) -> Result<(), ()> {
        // One control byte then the command stream; the panel keeps
        // interpreting bytes as commands until the transaction ends.
        let mut frame = [0u8; 32];
        for chunk in bytes.chunks(frame.len() - 1) {
            frame[0] = CTRL_CMD;
            frame[1..1 + chunk.len()].copy_from_slice(chunk);
            self.i2c
                .write_async(self.address, &frame[..1 + chunk.len()])
                .await
                .map_err(|_| ())?;
        }
        Ok(())
    }

    async fn init(&mut self) -> Result<(), ()> {
        self.cmds(INIT).await?;
        self.on = true;
        // Nothing is known about display RAM across an init, so the next
        // flush has to send a whole frame rather than trust `sent`.
        self.sent_valid = false;
        Ok(())
    }

    /// Blank the panel and stop its charge pump.
    ///
    /// Called before deep sleep. The display sits on the always-on +3V3
    /// rail, so without this it holds its last frame - and its current -
    /// for the whole time the S3 is asleep, which on a board whose sleep
    /// is already dominated by the GPS is the last thing worth adding to
    /// it.
    pub async fn blank(&mut self) {
        if !self.on {
            return;
        }
        // 0xAE is display off; 0x8D 0x10 stops the charge pump, which is
        // what actually takes the panel current down rather than just
        // clearing the pixels.
        let _ = self.cmds(&[0xAE, 0x8D, 0x10]).await;
        self.on = false;
    }

    fn clear(&mut self) {
        self.buf = [0; FRAME];
    }

    /// Draw `text` at a character cell. Out-of-range rows are dropped and
    /// text past the right edge is clipped rather than wrapped: every
    /// caller here is writing a fixed-width status field, so a line that
    /// grew is a line to shorten, not one to fold.
    fn text(&mut self, col: usize, row: usize, text: &str) {
        if row >= ROWS {
            return;
        }
        let base = row * WIDTH;
        for (i, ch) in text.bytes().enumerate() {
            let cell = col + i;
            if cell >= COLS {
                return;
            }
            let glyph = match ch {
                0x20..=0x7E => ((ch - 0x20) as usize) * 5,
                _ => 0,
            };
            let x = cell * CELL_W;
            for c in 0..5 {
                self.buf[base + x + c] = FONT[glyph + c];
            }
            // The sixth column is the inter-character gap.
            self.buf[base + x + 5] = 0x00;
        }
    }

    /// Push the frame if it differs from what the panel is showing.
    pub async fn flush(&mut self) {
        if !self.on && self.init().await.is_err() {
            return;
        }
        if self.sent_valid && self.buf == self.sent {
            return;
        }
        // Window the whole panel, then stream it. Horizontal addressing
        // wraps column to page for us, so the frame is one linear run.
        if self
            .cmds(&[0x21, 0, (WIDTH - 1) as u8, 0x22, 0, (PAGES - 1) as u8])
            .await
            .is_err()
        {
            return;
        }
        let mut frame = [0u8; 65];
        for chunk in self.buf.chunks(frame.len() - 1) {
            frame[0] = CTRL_DATA;
            frame[1..1 + chunk.len()].copy_from_slice(chunk);
            if self
                .i2c
                .write_async(self.address, &frame[..1 + chunk.len()])
                .await
                .is_err()
            {
                // A panel unplugged mid-run: stop, and let the next pass
                // try again rather than spending the rest of the frame on
                // a bus nothing is answering. `sent` is deliberately left
                // invalid, because half a frame reached the panel and the
                // next pass must not skip itself on a comparison against
                // what was only partly written.
                self.sent_valid = false;
                return;
            }
        }
        self.sent = self.buf;
        self.sent_valid = true;
    }
}

/// Seconds-since as something that fits four characters.
///
/// `secs_since_rx` carries 0xFFFF for "no packet since boot", which has to
/// read as never rather than as a very large number of seconds - the two
/// mean opposite things about whether the radio is working.
fn ago(secs: u16) -> heapless::String<8> {
    use core::fmt::Write as _;
    let mut s = heapless::String::new();
    if secs == u16::MAX {
        let _ = write!(s, "never");
    } else if secs < 60 {
        let _ = write!(s, "{}s", secs);
    } else if secs < 3600 {
        let _ = write!(s, "{}m", secs / 60);
    } else {
        let _ = write!(s, "{}h", secs / 3600);
    }
    s
}

/// Render the status screen from the snapshot the hardware loop publishes.
///
/// Four lines of twenty-one characters, right-aligned on the value so the
/// numbers sit in a column and a changing digit does not move the label.
/// Nothing is drawn from live hardware here: this reads the same
/// [`Telemetry`] the BLE session notifies, so what the panel says and what
/// the app says cannot disagree.
pub fn render(oled: &mut Oled, telemetry: Option<Telemetry>, node_address: u8) {
    use core::fmt::Write as _;
    oled.clear();

    let Some(t) = telemetry else {
        oled.text(0, 0, "wio-s3-gps");
        oled.text(0, 2, "starting...");
        return;
    };

    let mut line: heapless::String<{ COLS }> = heapless::String::new();

    // Fix and satellites - the two that answer "is the GPS working".
    let fix = t.flags & midair_proto::link::TELEM_FLAG_GPS_FIX != 0;
    let _ = write!(
        line,
        "GPS {}  {:>2} sat",
        if fix { "FIX " } else { "----" },
        t.sats
    );
    oled.text(0, 0, &line);

    // Signal strength of the last packet heard.
    line.clear();
    if t.secs_since_rx == u16::MAX {
        let _ = write!(line, "RSSI     --  dBm");
    } else {
        let _ = write!(line, "RSSI {:>4} dBm", t.last_rssi);
    }
    oled.text(0, 1, &line);

    // How long ago that was, which is what turns the RSSI above from a
    // reading into a live one.
    line.clear();
    let _ = write!(line, "SEEN {:>6}", ago(t.secs_since_rx).as_str());
    oled.text(0, 2, &line);

    // Counters and this node's own address, so two boards on a bench are
    // told apart without connecting to either.
    line.clear();
    let _ = write!(
        line,
        "n{:<3} rx{:<5} tx{}",
        node_address,
        t.rx_count.min(9999),
        t.tx_count.min(9999)
    );
    oled.text(0, 3, &line);
}
