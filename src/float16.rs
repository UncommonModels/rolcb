//! IEEE 754 half-precision floats, as used for Traction speeds.
//!
//! Traction speeds are in scale meters/second. The sign bit carries the
//! direction (set = reverse), so `-0.0` means "stopped, facing reverse".
//!
//! The *Traction Control Standard* sends speeds as a 16-bit IEEE 754
//! "binary16" value: 1 sign bit, 5 exponent bits (bias 15) and 10 mantissa
//! bits, big-endian on the wire. That gives roughly three significant
//! decimal digits, which is plenty for a model train: 126 scale mph
//! (56.3 m/s) round-trips to within 0.1%.
//!
//! ```text
//!  15  14   10 9          0
//! +---+-------+------------+
//! | S |  exp  |  mantissa  |      S = 1 → reverse, S = 0 → forward
//! +---+-------+------------+
//! ```
//!
//! Because direction lives in the sign bit, `0x0000` (+0.0) and `0x8000`
//! (-0.0) are both "stopped" but remember different directions. Rust's
//! `f32` preserves the sign of zero, so the conversions here do too. Most
//! code should use [`traction::Speed`](crate::traction::Speed), which wraps
//! these functions with direction-aware helpers.
//!
//! This module is a small, dependency-free converter that works in
//! `no_std`. It rounds to nearest-even, produces subnormals for tiny
//! values, saturates to infinity on overflow and preserves NaN.
//!
//! ```
//! use rolcb::float16::{from_f32, to_f32};
//!
//! assert_eq!(from_f32(1.0), 0x3C00);
//! assert_eq!(from_f32(-2.0), 0xC000);
//! assert_eq!(from_f32(-0.0), 0x8000);   // stopped, reverse
//! assert_eq!(to_f32(0x3800), 0.5);
//! assert!(to_f32(0x8000).is_sign_negative());
//! assert_eq!(from_f32(1.0e6), 0x7C00);  // too big: +infinity
//! ```

/// Convert an `f32` to IEEE half-float bits (round-to-nearest-even).
///
/// Values larger than 65504 become infinity (`0x7C00`/`0xFC00`), values
/// smaller than the smallest subnormal become signed zero, NaN stays NaN.
///
/// ```
/// assert_eq!(rolcb::float16::from_f32(0.5), 0x3800);
/// assert_eq!(rolcb::float16::from_f32(65504.0), 0x7BFF);
/// ```
pub fn from_f32(v: f32) -> u16 {
    let bits = v.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exp = ((bits >> 23) & 0xFF) as i32;
    let mant = bits & 0x007F_FFFF;

    if exp == 0xFF {
        // Inf / NaN
        return sign | 0x7C00 | if mant != 0 { 0x0200 } else { 0 };
    }
    let e = exp - 127 + 15;
    if e >= 0x1F {
        return sign | 0x7C00; // overflow -> inf
    }
    if e <= 0 {
        // Subnormal or zero.
        if e < -10 {
            return sign;
        }
        let m = mant | 0x0080_0000;
        let shift = (14 - e) as u32;
        let half = m >> shift;
        let rem = m & ((1 << shift) - 1);
        let halfway = 1 << (shift - 1);
        let round = rem > halfway || (rem == halfway && (half & 1) == 1);
        return sign | (half + round as u32) as u16;
    }
    let half_mant = mant >> 13;
    let rem = mant & 0x1FFF;
    let mut h = ((e as u32) << 10) | half_mant;
    if rem > 0x1000 || (rem == 0x1000 && (half_mant & 1) == 1) {
        h += 1; // may carry into exponent, which is correct
    }
    sign | h as u16
}

/// Convert IEEE half-float bits to `f32`.
///
/// Every half-float value is exactly representable as an `f32`, so this
/// conversion is lossless (including subnormals, signed zero, infinities
/// and NaN).
///
/// ```
/// assert_eq!(rolcb::float16::to_f32(0xC000), -2.0);
/// assert!(rolcb::float16::to_f32(0xFFFF).is_nan());
/// ```
pub fn to_f32(h: u16) -> f32 {
    let sign = ((h & 0x8000) as u32) << 16;
    let exp = ((h >> 10) & 0x1F) as u32;
    let mant = (h & 0x03FF) as u32;
    let bits = match exp {
        0 if mant == 0 => sign,
        0 => {
            // Subnormal: normalise.
            let mut e = 127 - 15 + 1;
            let mut m = mant;
            while m & 0x0400 == 0 {
                m <<= 1;
                e -= 1;
            }
            sign | ((e as u32) << 23) | ((m & 0x03FF) << 13)
        }
        0x1F => sign | 0x7F80_0000 | (mant << 13),
        _ => sign | ((exp + 127 - 15) << 23) | (mant << 13),
    };
    f32::from_bits(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_values() {
        assert_eq!(from_f32(0.0), 0x0000);
        assert_eq!(from_f32(-0.0), 0x8000);
        assert_eq!(from_f32(1.0), 0x3C00);
        assert_eq!(from_f32(-2.0), 0xC000);
        assert_eq!(from_f32(0.5), 0x3800);
        assert_eq!(from_f32(65504.0), 0x7BFF);
        assert_eq!(from_f32(1.0e6), 0x7C00);
        assert_eq!(to_f32(0x3C00), 1.0);
        assert_eq!(to_f32(0xC000), -2.0);
        assert!(to_f32(0x8000).is_sign_negative());
        assert_eq!(to_f32(0x0001), 5.960_464_5e-8);
    }

    #[test]
    fn round_trip_speeds() {
        // 0..=126 mph in m/s survive within half-float precision.
        for mph in 0..=126 {
            let mps = mph as f32 * 0.44704;
            let back = to_f32(from_f32(mps));
            assert!(
                (back - mps).abs() <= mps * 0.001 + 1e-6,
                "{mph}: {mps} -> {back}"
            );
        }
    }

    /// Every non-NaN half survives half → f32 → half exactly, keeping the
    /// sign of zero; every NaN stays NaN.
    #[test]
    fn exhaustive_round_trip() {
        for h in 0..=u16::MAX {
            let f = to_f32(h);
            if f.is_nan() {
                assert_eq!(h & 0x7C00, 0x7C00, "{h:04X}");
                assert_ne!(h & 0x03FF, 0, "{h:04X}");
                assert!(to_f32(from_f32(f)).is_nan());
                continue;
            }
            assert_eq!(from_f32(f), h, "{h:04X} -> {f}");
        }
    }

    #[test]
    fn boundaries() {
        // Largest finite, smallest normal, largest/smallest subnormal.
        assert_eq!(to_f32(0x7BFF), 65504.0);
        assert_eq!(to_f32(0x0400), 6.103_515_6e-5);
        assert_eq!(to_f32(0x03FF), 1023.0 / 16_777_216.0);
        assert_eq!(to_f32(0x0001), 5.960_464_5e-8);
        assert_eq!(to_f32(0x7C00), f32::INFINITY);
        assert_eq!(to_f32(0xFC00), f32::NEG_INFINITY);
        assert_eq!(from_f32(f32::INFINITY), 0x7C00);
        assert_eq!(from_f32(f32::NEG_INFINITY), 0xFC00);
        assert!(to_f32(from_f32(f32::NAN)).is_nan());
        // Overflow: 65520 is the halfway point to infinity and rounds up.
        assert_eq!(from_f32(65519.0), 0x7BFF);
        assert_eq!(from_f32(65520.0), 0x7C00);
        assert_eq!(from_f32(-1.0e9), 0xFC00);
        // Underflow: half of the smallest subnormal rounds to even (zero);
        // just above it rounds up.
        assert_eq!(from_f32(2.980_232_2e-8), 0x0000);
        assert_eq!(from_f32(2.980_233e-8), 0x0001);
        assert_eq!(from_f32(-2.980_232_2e-8), 0x8000);
        assert_eq!(from_f32(1.0e-10), 0x0000);
        assert_eq!(from_f32(f32::MIN_POSITIVE), 0x0000);
        // Subnormal → normal carry.
        assert_eq!(from_f32(6.102e-5), 0x0400);
        assert_eq!(from_f32(6.099e-5), 0x03FF);
        // Round to nearest even in the normal range: 1 + 2^-11 is halfway
        // between 0x3C00 and 0x3C01 and rounds to the even one.
        assert_eq!(from_f32(1.0 + 1.0 / 2048.0), 0x3C00);
        assert_eq!(from_f32(1.0 + 3.0 / 2048.0), 0x3C02);
        // Mantissa overflow carries into the exponent.
        assert_eq!(from_f32(2.0 - 1.0 / 4096.0), 0x4000);
    }
}
