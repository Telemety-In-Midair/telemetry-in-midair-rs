//! A 3-axis magnetometer on the display's I2C bus, as a heading.
//!
//! Two parts are recognized, both by address, because they are what the
//! cheap "GY-271" breakout ships as and the silkscreen usually does not say
//! which one is fitted:
//!
//! - **QMC5883L** at 0x0D. Continuous mode, data at 0x00, little-endian.
//! - **HMC5883L** at 0x1E. Continuous mode, data at 0x03, big-endian, and
//!   the axis order is X, **Z**, Y rather than X, Y, Z - a difference that
//!   produces a heading which looks plausible and is wrong, so it is worth
//!   naming rather than leaving to the register map.
//!
//! What this is not: tilt-compensated. A magnetometer alone measures the
//! field in the *board's* frame, so tipping the board mixes the vertical
//! component of the Earth's field into the horizontal axes and swings the
//! heading. Holding it level is the requirement, and
//! [`Compass::heading_deg`] does not pretend otherwise. Correcting it needs
//! an accelerometer, which is a different part than the one this supports.

use esp_hal::i2c::master::I2c;
use esp_hal::Async;
use libm::atan2f;
use midair_proto::geo::normalize_deg;

const QMC5883L_ADDR: u8 = 0x0D;
const HMC5883L_ADDR: u8 = 0x1E;

/// Which of the two is on the bus. They share a footprint and a breakout
/// but not a register map.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Part {
    Qmc5883l,
    Hmc5883l,
}

impl Part {
    pub fn as_str(self) -> &'static str {
        match self {
            Part::Qmc5883l => "QMC5883L",
            Part::Hmc5883l => "HMC5883L",
        }
    }

    fn address(self) -> u8 {
        match self {
            Part::Qmc5883l => QMC5883L_ADDR,
            Part::Hmc5883l => HMC5883L_ADDR,
        }
    }
}

/// Running per-axis extremes, which is the whole of the hard-iron
/// correction.
///
/// A magnetometer sitting next to a LoRa PA, an SD card and a battery does
/// not read a field centered on zero - the board's own iron offsets it, and
/// an uncorrected reading traces a circle somewhere off-origin, which
/// becomes a heading that is right in two places and wrong everywhere else.
/// Subtracting the midpoint of each axis's observed range re-centers it.
///
/// This calibrates by being turned: the extremes are only correct once the
/// board has been rotated through a full circle, so [`Compass::calibrated`]
/// reports whether that has happened and the display says so rather than
/// showing a confident heading built from a quarter turn.
#[derive(Clone, Copy)]
struct Extremes {
    min: [i16; 3],
    max: [i16; 3],
    seen: bool,
}

impl Extremes {
    const fn new() -> Self {
        Self {
            min: [i16::MAX; 3],
            max: [i16::MIN; 3],
            seen: false,
        }
    }

    fn feed(&mut self, raw: [i16; 3]) {
        for (axis, reading) in raw.into_iter().enumerate() {
            self.min[axis] = self.min[axis].min(reading);
            self.max[axis] = self.max[axis].max(reading);
        }
        self.seen = true;
    }

    /// Centered reading, as floats so the division does not quantize.
    fn correct(&self, raw: [i16; 3]) -> [f32; 3] {
        let mut out = [0.0; 3];
        for i in 0..3 {
            let offset = (i32::from(self.min[i]) + i32::from(self.max[i])) as f32 / 2.0;
            out[i] = raw[i] as f32 - offset;
        }
        out
    }

    /// Whether both horizontal axes have swung far enough for the midpoint
    /// to mean anything.
    ///
    /// The threshold is in raw counts and deliberately loose: it is asking
    /// "has this been turned around", not "is this well calibrated". A
    /// board rotated through a full circle produces a span of several
    /// thousand counts on both axes; one sitting still produces noise.
    fn usable(&self) -> bool {
        const SPAN_MIN: i32 = 600;
        self.seen
            && (0..2).all(|i| i32::from(self.max[i]) - i32::from(self.min[i]) >= SPAN_MIN)
    }
}

pub struct Compass {
    part: Part,
    cal: Extremes,
    /// Last good raw reading, so a dropped I2C transaction shows the
    /// previous heading rather than snapping the arrow to north.
    last: Option<[i16; 3]>,
}

impl Compass {
    /// Look for a magnetometer on the bus already built for the display.
    ///
    /// Takes the I2C by reference because the display owns it: one bus, two
    /// devices, and the hardware loop is the only caller so there is never
    /// a second transaction in flight.
    pub async fn probe(i2c: &mut I2c<'static, Async>) -> Option<Self> {
        for part in [Part::Qmc5883l, Part::Hmc5883l] {
            if i2c.write_async(part.address(), &[]).await.is_err() {
                continue;
            }
            let ok = match part {
                // Soft reset, then set/reset period 0x01 (the datasheet
                // says always write this), then control: continuous mode,
                // 200 Hz, 8 gauss, 512x oversampling.
                Part::Qmc5883l => {
                    i2c.write_async(part.address(), &[0x0A, 0x80]).await.is_ok()
                        && i2c.write_async(part.address(), &[0x0B, 0x01]).await.is_ok()
                        && i2c.write_async(part.address(), &[0x09, 0x1D]).await.is_ok()
                }
                // Config A: 8-sample average, 15 Hz. Config B: gain 1.3 Ga.
                // Mode: continuous.
                Part::Hmc5883l => {
                    i2c.write_async(part.address(), &[0x00, 0x70]).await.is_ok()
                        && i2c.write_async(part.address(), &[0x01, 0x20]).await.is_ok()
                        && i2c.write_async(part.address(), &[0x02, 0x00]).await.is_ok()
                }
            };
            if ok {
                return Some(Self {
                    part,
                    cal: Extremes::new(),
                    last: None,
                });
            }
        }
        None
    }

    pub fn part(&self) -> Part {
        self.part
    }

    /// Whether the board has been turned enough for the heading to mean
    /// anything.
    pub fn calibrated(&self) -> bool {
        self.cal.usable()
    }

    /// Read the field and fold it into the calibration. Call it on every
    /// display refresh; the calibration improves as the board is moved.
    pub async fn sample(&mut self, i2c: &mut I2c<'static, Async>) {
        let (reg, len) = match self.part {
            Part::Qmc5883l => (0x00u8, 6usize),
            Part::Hmc5883l => (0x03u8, 6usize),
        };
        let mut buf = [0u8; 6];
        if i2c
            .write_read_async(self.part.address(), &[reg], &mut buf[..len])
            .await
            .is_err()
        {
            return;
        }
        let raw = match self.part {
            // Little-endian, X Y Z.
            Part::Qmc5883l => [
                i16::from_le_bytes([buf[0], buf[1]]),
                i16::from_le_bytes([buf[2], buf[3]]),
                i16::from_le_bytes([buf[4], buf[5]]),
            ],
            // Big-endian, and the order on the wire is X, Z, Y.
            Part::Hmc5883l => [
                i16::from_be_bytes([buf[0], buf[1]]),
                i16::from_be_bytes([buf[4], buf[5]]),
                i16::from_be_bytes([buf[2], buf[3]]),
            ],
        };
        // An all-zero read is what a part that has not finished its first
        // conversion returns, and it would drag both axes' minima to zero
        // and poison the calibration permanently.
        if raw == [0, 0, 0] {
            return;
        }
        self.cal.feed(raw);
        self.last = Some(raw);
    }

    /// Heading in degrees clockwise from magnetic north, or `None` until
    /// the board has been turned through enough of a circle to trust it.
    ///
    /// Magnetic, not true: no declination is applied. Over the distances
    /// this points across, the bearing and the heading are both wrong by
    /// the same declination and it cancels out of the *relative* bearing
    /// the arrow is drawn from - which is the only thing displayed.
    pub fn heading_deg(&self) -> Option<f32> {
        if !self.cal.usable() {
            return None;
        }
        let v = self.cal.correct(self.last?);
        // atan2(y, x) measures counterclockwise from +X; a compass measures
        // clockwise from +Y (north), which is the swapped-argument,
        // negated-y form below.
        Some(normalize_deg(atan2f(-v[1], v[0]) * 180.0 / core::f32::consts::PI))
    }
}
