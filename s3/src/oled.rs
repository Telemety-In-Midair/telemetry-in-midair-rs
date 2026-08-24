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

/// Column 0 of the panel in the controller's RAM. Zero on an SSD1306,
/// which is what a 0.91" module carries; an SH1106 centers a 128-column
/// panel in 132 columns of RAM and needs 2, which is what a display that
/// works but is shifted two pixels sideways is telling you.
const COL_OFFSET: u8 = 0;

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
    0x20, 0x02, // page addressing: see `write_page` for why not horizontal
    0xA1, // segment remap: column 127 maps to SEG0
    0xC8, // COM scan direction reversed
    0xDA, 0x02, // COM pin configuration for 32 rows
    0x81, 0x8F, // contrast
    0xD9, 0xF1, // precharge period
    0xDB, 0x40, // VCOMH deselect level
    0xA4, // follow display RAM, rather than forcing every pixel on
    0xA6, // normal, not inverted
    0x2E, // scrolling off
];
// Display on is deliberately not in that list: `init` clears display RAM
// first, so the panel never shows whatever the RAM happened to hold at
// power-up.
const DISPLAY_ON: &[u8] = &[0xAF];

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
///
/// The I2C bus is *not* owned here. It is shared with the magnetometer in
/// [`crate::compass`], so every method that talks to the panel borrows it
/// from the hardware loop, which is the single caller and therefore the
/// thing that guarantees no two transactions overlap.
pub struct Oled {
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
    pub async fn probe(i2c: &mut I2c<'static, Async>) -> Option<Self> {
        for address in ADDRESSES {
            // A zero-length write is the cheapest address poll: it drives
            // the address byte and reads the ack bit without leaving the
            // panel in a state that matters if something else is there.
            if i2c.write_async(address, &[]).await.is_ok() {
                let mut oled = Self {
                    address,
                    buf: [0; FRAME],
                    sent: [0; FRAME],
                    sent_valid: false,
                    on: false,
                };
                oled.init(i2c).await.ok()?;
                return Some(oled);
            }
        }
        None
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    async fn cmds(&mut self, i2c: &mut I2c<'static, Async>, bytes: &[u8]) -> Result<(), ()> {
        // One control byte then the command stream; the panel keeps
        // interpreting bytes as commands until the transaction ends.
        let mut frame = [0u8; 32];
        for chunk in bytes.chunks(frame.len() - 1) {
            frame[0] = CTRL_CMD;
            frame[1..1 + chunk.len()].copy_from_slice(chunk);
            i2c.write_async(self.address, &frame[..1 + chunk.len()])
                .await
                .map_err(|_| ())?;
        }
        Ok(())
    }

    async fn init(&mut self, i2c: &mut I2c<'static, Async>) -> Result<(), ()> {
        self.cmds(i2c, INIT).await?;
        self.clear_ram(i2c).await?;
        self.cmds(i2c, DISPLAY_ON).await?;
        self.on = true;
        // Nothing is known about display RAM across an init, so the next
        // flush has to send a whole frame rather than trust `sent`.
        self.sent_valid = false;
        Ok(())
    }

    /// Point the write pointer at column `col` of `page`.
    ///
    /// Page addressing rather than the addressing-mode registers, because
    /// the column start is split across two commands here but works the
    /// same on every SSD1306-compatible controller, including the SH1106
    /// that some modules carry, which has no horizontal addressing mode at
    /// all and quietly drops the window commands.
    async fn goto(&mut self, i2c: &mut I2c<'static, Async>, page: u8, col: u8) -> Result<(), ()> {
        let col = col + COL_OFFSET;
        self.cmds(i2c, &[0xB0 | page, col & 0x0F, 0x10 | (col >> 4)])
            .await
    }

    /// Zero every page the controller has, not just the four this panel
    /// shows.
    ///
    /// A 128x64 controller driven at 32 rows still holds whatever landed in
    /// the pages below, and RAM is not defined at power-up - so anything not
    /// written here is free to appear as speckle. Done once per init, before
    /// the display is turned on.
    async fn clear_ram(&mut self, i2c: &mut I2c<'static, Async>) -> Result<(), ()> {
        // 132 columns, the widest RAM row any of these controllers has.
        let mut frame = [0u8; 1 + 66];
        frame[0] = CTRL_DATA;
        for page in 0..8 {
            self.goto(i2c, page, 0).await?;
            for _ in 0..2 {
                i2c.write_async(self.address, &frame)
                    .await
                    .map_err(|_| ())?;
            }
        }
        Ok(())
    }

    /// Send one page: 128 bytes from `buf` to the row of the panel that
    /// `page` addresses.
    async fn write_page(&mut self, i2c: &mut I2c<'static, Async>, page: usize) -> Result<(), ()> {
        self.goto(i2c, page as u8, 0).await?;
        let mut frame = [0u8; 65];
        let end = (page + 1) * WIDTH;
        let mut off = page * WIDTH;
        while off < end {
            let n = (end - off).min(frame.len() - 1);
            frame[0] = CTRL_DATA;
            frame[1..1 + n].copy_from_slice(&self.buf[off..off + n]);
            i2c.write_async(self.address, &frame[..1 + n])
                .await
                .map_err(|_| ())?;
            off += n;
        }
        Ok(())
    }

    /// Blank the panel and stop its charge pump.
    ///
    /// Called before deep sleep. The display sits on the always-on +3V3
    /// rail, so without this it holds its last frame - and its current -
    /// for the whole time the S3 is asleep, which on a board whose sleep
    /// is already dominated by the GPS is the last thing worth adding to
    /// it.
    pub async fn blank(&mut self, i2c: &mut I2c<'static, Async>) {
        if !self.on {
            return;
        }
        // 0xAE is display off; 0x8D 0x10 stops the charge pump, which is
        // what actually takes the panel current down rather than just
        // clearing the pixels.
        let _ = self.cmds(i2c, &[0xAE, 0x8D, 0x10]).await;
        self.on = false;
    }

    fn clear(&mut self) {
        self.buf = [0; FRAME];
    }

    /// Set one pixel. Out-of-range coordinates are dropped rather than
    /// wrapping, so a shape that runs off the panel is clipped instead of
    /// reappearing on the far side.
    fn pixel(&mut self, x: i32, y: i32) {
        if x < 0 || y < 0 || x >= WIDTH as i32 || y >= HEIGHT as i32 {
            return;
        }
        let (x, y) = (x as usize, y as usize);
        // Page-major layout: one byte spans eight rows, bit 0 at the top.
        self.buf[(y / 8) * WIDTH + x] |= 1 << (y % 8);
    }

    /// Bresenham, so a line is drawn without a division per pixel.
    fn line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32) {
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let (mut x, mut y) = (x0, y0);
        let mut err = dx + dy;
        loop {
            self.pixel(x, y);
            if x == x1 && y == y1 {
                return;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
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
    pub async fn flush(&mut self, i2c: &mut I2c<'static, Async>) {
        if !self.on && self.init(i2c).await.is_err() {
            return;
        }
        if self.sent_valid && self.buf == self.sent {
            return;
        }
        // Per page, and only the pages that changed: a status readout
        // usually redraws one line, which is a quarter of the bus time of a
        // whole frame.
        for page in 0..PAGES {
            let range = page * WIDTH..(page + 1) * WIDTH;
            if self.sent_valid && self.buf[range.clone()] == self.sent[range] {
                continue;
            }
            if self.write_page(i2c, page).await.is_err() {
                // A panel unplugged mid-run: stop, and let the next pass
                // try again rather than spending the rest of the frame on
                // a bus nothing is answering. `sent` is deliberately left
                // invalid, because part of a frame reached the panel and
                // the next pass must not skip itself on a comparison
                // against what was only partly written.
                self.sent_valid = false;
                return;
            }
        }
        self.sent = self.buf;
        self.sent_valid = true;
    }
}

/// Center and radius of the compass rose, in the leftmost 32 pixels.
const ROSE_CX: i32 = 15;
const ROSE_CY: i32 = 15;
const ROSE_R: i32 = 14;

/// Draw the rose and an arrow at `rel_deg` clockwise from straight up.
///
/// "Up" is the direction the operator is facing, which is what makes this a
/// compass rather than a diagram: the four ticks are ahead, right, behind
/// and left, and the arrow is where the other node is from where you stand.
/// Feeding this a true bearing without subtracting the heading gives an
/// arrow that is correct only while facing north.
fn draw_rose(oled: &mut Oled, rel_deg: f32) {
    let rad = rel_deg * core::f32::consts::PI / 180.0;
    // Screen y grows downward, so "up" is -y and a clockwise bearing is
    // (sin, -cos) rather than the (cos, sin) of a math-convention angle.
    let (sin, cos) = (libm::sinf(rad), libm::cosf(rad));
    let tip_x = ROSE_CX + (sin * ROSE_R as f32) as i32;
    let tip_y = ROSE_CY - (cos * ROSE_R as f32) as i32;

    // Four ticks instead of a full circle: the outline would be most of the
    // lit pixels on this screen, and lit pixels are current.
    for (dx, dy) in [(0, -1), (1, 0), (0, 1), (-1, 0)] {
        let x = ROSE_CX + dx * ROSE_R;
        let y = ROSE_CY + dy * ROSE_R;
        oled.pixel(x, y);
        oled.pixel(x - dy, y - dx);
        oled.pixel(x + dy, y + dx);
    }

    oled.line(ROSE_CX, ROSE_CY, tip_x, tip_y);
    // Two barbs, 150 degrees back from the way the arrow points, so the
    // head reads as a head at five pixels rather than as a blob.
    for spread in [150.0f32, -150.0] {
        let a = rad + spread * core::f32::consts::PI / 180.0;
        let bx = tip_x + (libm::sinf(a) * 5.0) as i32;
        let by = tip_y - (libm::cosf(a) * 5.0) as i32;
        oled.line(tip_x, tip_y, bx, by);
    }
}

/// Distance in the shortest form that stays honest about its precision.
fn distance_text(meters: f32) -> heapless::String<8> {
    use core::fmt::Write as _;
    let mut s = heapless::String::new();
    if meters < 1000.0 {
        let _ = write!(s, "{}m", meters as u32);
    } else if meters < 10_000.0 {
        // Two decimals below 10 km, where a person can act on 10 m.
        let _ = write!(s, "{}.{:02}km", meters as u32 / 1000, (meters as u32 % 1000) / 10);
    } else {
        let _ = write!(s, "{}km", meters as u32 / 1000);
    }
    s
}

/// Where the heading came from, which decides what the arrow means.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Heading {
    /// From the magnetometer. The arrow is relative to where you face.
    Magnetic(u16),
    /// From GPS course over ground, which only exists while moving. The
    /// arrow is relative to the way you are travelling.
    Course(u16),
    /// No heading available. The arrow shows the true bearing and the
    /// operator has to supply the rotation.
    None,
}

impl Heading {
    fn degrees(self) -> f32 {
        match self {
            Heading::Magnetic(d) | Heading::Course(d) => f32::from(d),
            Heading::None => 0.0,
        }
    }

    /// One character, because there are fifteen columns and this has to be
    /// on screen: the arrow means three different things and a reader
    /// cannot tell which from the arrow.
    fn marker(self) -> &'static str {
        match self {
            Heading::Magnetic(_) => "M",
            Heading::Course(_) => "G",
            Heading::None => "T",
        }
    }
}

/// What the compass screen is drawn from.
pub struct Target {
    /// Address of the node being pointed at.
    pub node: u8,
    /// True bearing to it, degrees from north.
    pub bearing_deg: f32,
    pub distance_m: f32,
    /// Seconds since that node's position was heard.
    pub age_s: u16,
    pub rssi: i16,
}

/// Rose on the left, the numbers on the right.
///
/// Drawn instead of the status screen whenever there is a node to point at
/// and a fix to point from. The right-hand column keeps the satellite count
/// and the radio numbers, so switching screens does not cost the reader the
/// things the status screen existed to show.
pub fn render_compass(
    oled: &mut Oled,
    target: &Target,
    heading: Heading,
    fix: bool,
    sats: u8,
) {
    use core::fmt::Write as _;
    oled.clear();

    let rel = midair_proto::geo::relative_bearing_deg(target.bearing_deg, heading.degrees());
    draw_rose(oled, rel);

    // Cell 6 starts at x=36, leaving fifteen columns.
    const TEXT_COL: usize = 6;
    let mut line: heapless::String<16> = heapless::String::new();

    let _ = write!(
        line,
        "n{} {} {:03}",
        target.node,
        midair_proto::geo::compass_point(rel),
        rel as u16
    );
    oled.text(TEXT_COL, 0, &line);

    line.clear();
    let _ = write!(line, "{}", distance_text(target.distance_m).as_str());
    oled.text(TEXT_COL, 1, &line);

    line.clear();
    let _ = write!(line, "{}dB {}", target.rssi, ago(target.age_s).as_str());
    oled.text(TEXT_COL, 2, &line);

    line.clear();
    let _ = write!(
        line,
        "{} {:>2}sat {}",
        if fix { "FIX" } else { "---" },
        sats,
        heading.marker()
    );
    oled.text(TEXT_COL, 3, &line);
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
