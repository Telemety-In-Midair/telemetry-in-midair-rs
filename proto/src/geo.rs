//! Bearing and distance between two fixes, and what to call a direction.
//!
//! This is what turns the remote-node roster from a list into something you
//! can walk towards: given this node's fix and another node's, which way is
//! it and how far. It lives here rather than in the firmware because it is
//! arithmetic with edge cases that are painful to find on hardware -
//! the antimeridian, the poles, two nodes at the same spot - and trivial to
//! pin down in a test.
//!
//! Everything is in the `1e-7` degree integers the position packet carries,
//! and everything comes back in degrees or meters.

use libm::{atan2f, cosf, sinf, sqrtf};

/// Degrees per `1e-7`-degree unit.
const E7: f32 = 1e-7;
/// Mean Earth radius, meters. The figure the haversine convention uses.
const EARTH_R_M: f32 = 6_371_000.0;

const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

/// Initial great-circle bearing from one fix to another, degrees clockwise
/// from true north, in `0.0..360.0`.
///
/// "Initial" is the honest word: a great-circle track's bearing changes as
/// you walk it. Over the ranges LoRa covers the difference is far below what
/// a 32-pixel arrow can show, but the distinction is why this is not simply
/// the angle of a straight line on a flat map.
pub fn bearing_deg(from: (i32, i32), to: (i32, i32)) -> f32 {
    let lat1 = from.0 as f32 * E7 * DEG_TO_RAD;
    let lat2 = to.0 as f32 * E7 * DEG_TO_RAD;
    // Longitude difference, wrapped into -180..180 so a pair straddling the
    // antimeridian is a short step east rather than most of the way around
    // the world westward.
    let dlon = wrap180(dlon_deg(from.1, to.1)) * DEG_TO_RAD;

    let y = sinf(dlon) * cosf(lat2);
    let x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);
    normalize_deg(atan2f(y, x) * RAD_TO_DEG)
}

/// Great-circle distance in meters, by the haversine formula.
///
/// Haversine rather than the cheaper equirectangular approximation because
/// the expensive part is the same handful of trig calls either way, and
/// haversine does not degrade at high latitude or over long baselines - both
/// of which a repeater on a hill can produce.
pub fn distance_m(from: (i32, i32), to: (i32, i32)) -> f32 {
    let lat1 = from.0 as f32 * E7 * DEG_TO_RAD;
    let lat2 = to.0 as f32 * E7 * DEG_TO_RAD;
    let dlat = (lat2 - lat1) * 0.5;
    let dlon = wrap180(dlon_deg(from.1, to.1)) * DEG_TO_RAD * 0.5;

    let sin_dlat = sinf(dlat);
    let sin_dlon = sinf(dlon);
    let a = sin_dlat * sin_dlat + cosf(lat1) * cosf(lat2) * sin_dlon * sin_dlon;
    // Clamped before the square root: rounding can push `a` a hair past 1
    // for antipodal points, and `sqrt` of a negative would be NaN.
    2.0 * EARTH_R_M * atan2f(sqrtf(a.clamp(0.0, 1.0)), sqrtf((1.0 - a).clamp(0.0, 1.0)))
}

/// Where a bearing sits relative to the way you are facing: 0 is straight
/// ahead, 90 is to your right.
///
/// This is the number an arrow on a display is drawn from. Feeding it a true
/// bearing and forgetting the heading is the classic way to build a compass
/// that is correct only when the operator happens to face north.
pub fn relative_bearing_deg(bearing_deg: f32, heading_deg: f32) -> f32 {
    normalize_deg(bearing_deg - heading_deg)
}

/// Fold any angle into `0.0..360.0`.
pub fn normalize_deg(deg: f32) -> f32 {
    let mut d = deg % 360.0;
    if d < 0.0 {
        d += 360.0;
    }
    // A tiny negative input leaves `d` at exactly 360.0 after the add.
    if d >= 360.0 {
        d = 0.0;
    }
    d
}

/// Longitude difference in degrees, computed in `i64`.
///
/// The subtraction must not happen in `i32`: two fixes either side of the
/// antimeridian are about +1.8e9 and -1.8e9 in `1e-7` degrees, and the
/// 3.6e9 between them is past `i32::MAX`. In release that wraps silently
/// and puts the bearing 180 degrees out - an arrow pointing exactly the
/// wrong way, on the one part of the world where nobody is checking.
fn dlon_deg(from_lon_e7: i32, to_lon_e7: i32) -> f32 {
    (i64::from(to_lon_e7) - i64::from(from_lon_e7)) as f32 * E7
}

/// Fold a longitude difference into `-180.0..=180.0`.
fn wrap180(deg: f32) -> f32 {
    let mut d = deg % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d < -180.0 {
        d += 360.0;
    }
    d
}

/// The 16-point compass name for a bearing: `"N"`, `"NNE"`, ... `"NNW"`.
///
/// Sixteen points rather than eight because the extra three characters cost
/// nothing on the display and halve the ambiguity: "NE" spans 45 degrees,
/// which at a kilometer is most of a field.
pub fn compass_point(deg: f32) -> &'static str {
    const POINTS: [&str; 16] = [
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW",
        "NW", "NNW",
    ];
    // +11.25 so each name is centered on its point rather than starting at
    // it: due north has to read "N" from 348.75 through 11.25, not from 0
    // through 22.5.
    let idx = ((normalize_deg(deg) + 11.25) / 22.5) as usize % 16;
    POINTS[idx]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Degrees to the packet's 1e-7 integers.
    fn p(lat: f64, lon: f64) -> (i32, i32) {
        ((lat * 1e7) as i32, (lon * 1e7) as i32)
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn the_cardinal_directions_come_out_cardinal() {
        let here = p(45.0, 9.0);
        assert!(close(bearing_deg(here, p(46.0, 9.0)), 0.0, 0.1), "north");
        assert!(close(bearing_deg(here, p(45.0, 10.0)), 90.0, 0.5), "east");
        assert!(close(bearing_deg(here, p(44.0, 9.0)), 180.0, 0.1), "south");
        assert!(close(bearing_deg(here, p(45.0, 8.0)), 270.0, 0.5), "west");
    }

    /// A degree of latitude is about 111 km anywhere; a degree of longitude
    /// is that times the cosine of the latitude. Both are worth pinning,
    /// because getting the cosine on the wrong term is a mistake that looks
    /// right at the equator.
    #[test]
    fn distances_match_the_textbook_figures() {
        assert!(
            close(distance_m(p(0.0, 0.0), p(1.0, 0.0)), 111_195.0, 200.0),
            "a degree of latitude"
        );
        assert!(
            close(distance_m(p(0.0, 0.0), p(0.0, 1.0)), 111_195.0, 200.0),
            "a degree of longitude at the equator"
        );
        assert!(
            close(distance_m(p(60.0, 0.0), p(60.0, 1.0)), 55_597.0, 200.0),
            "a degree of longitude at 60 north is half that"
        );
    }

    /// Two nodes in the same place must not produce a NaN bearing or a
    /// distance that is anything but zero - the display would show whatever
    /// a NaN formats as, forever, on the one board sitting next to you.
    #[test]
    fn a_node_at_your_own_position_is_finite() {
        let here = p(51.5, -0.12);
        assert_eq!(distance_m(here, here), 0.0);
        let b = bearing_deg(here, here);
        assert!(b.is_finite() && (0.0..360.0).contains(&b), "bearing {b}");
    }

    /// The antimeridian is one step east, not 359 degrees of the long way
    /// round. This is the case a plain longitude subtraction gets wrong.
    #[test]
    fn crossing_the_antimeridian_is_a_short_step() {
        let west = p(0.0, 179.9);
        let east = p(0.0, -179.9);
        assert!(close(bearing_deg(west, east), 90.0, 0.5), "eastward");
        assert!(close(bearing_deg(east, west), 270.0, 0.5), "westward");
        assert!(
            distance_m(west, east) < 30_000.0,
            "about 22 km, not most of the equator"
        );
    }

    /// The whole point of the heading: the same bearing points to different
    /// parts of the display depending on which way the operator faces.
    #[test]
    fn relative_bearing_turns_with_the_operator() {
        assert_eq!(relative_bearing_deg(90.0, 0.0), 90.0, "facing north");
        assert_eq!(relative_bearing_deg(90.0, 90.0), 0.0, "facing it");
        assert_eq!(relative_bearing_deg(0.0, 90.0), 270.0, "it is behind left");
        assert_eq!(relative_bearing_deg(10.0, 350.0), 20.0, "across the wrap");
    }

    /// Each name is centered on its point, so due north reads "N" from
    /// either side of the wrap rather than "NNW" on one of them.
    #[test]
    fn compass_names_are_centered_on_their_point() {
        assert_eq!(compass_point(0.0), "N");
        assert_eq!(compass_point(359.0), "N");
        assert_eq!(compass_point(11.0), "N");
        assert_eq!(compass_point(12.0), "NNE");
        assert_eq!(compass_point(45.0), "NE");
        assert_eq!(compass_point(180.0), "S");
        assert_eq!(compass_point(270.0), "W");
        assert_eq!(compass_point(348.75), "N");
    }

    #[test]
    fn normalize_covers_both_directions_and_the_boundary() {
        assert_eq!(normalize_deg(0.0), 0.0);
        assert_eq!(normalize_deg(360.0), 0.0);
        assert_eq!(normalize_deg(-1.0), 359.0);
        assert_eq!(normalize_deg(720.5), 0.5);
        assert_eq!(normalize_deg(-721.0), 359.0);
    }
}
