//! Mathematical utilities for Detour navigation mesh operations
//!
//!
//! These are simple wrappers around standard math functions for consistency
//! with the C++ API naming conventions.

/// Returns the absolute value of a floating point number
#[inline]
pub fn dt_math_fabsf(x: f32) -> f32 {
    x.abs()
}

/// Returns the square root of a floating point number
#[inline]
pub fn dt_math_sqrtf(x: f32) -> f32 {
    x.sqrt()
}

/// Returns the largest integer less than or equal to a floating point number
#[inline]
pub fn dt_math_floorf(x: f32) -> f32 {
    x.floor()
}

/// Returns the smallest integer greater than or equal to a floating point number
#[inline]
pub fn dt_math_ceilf(x: f32) -> f32 {
    x.ceil()
}

/// Returns the cosine of an angle in radians
#[inline]
pub fn dt_math_cosf(x: f32) -> f32 {
    x.cos()
}

/// Returns the sine of an angle in radians
#[inline]
pub fn dt_math_sinf(x: f32) -> f32 {
    x.sin()
}

/// Returns the arctangent of y/x in radians
#[inline]
pub fn dt_math_atan2f(y: f32, x: f32) -> f32 {
    y.atan2(x)
}

/// Checks if a floating point value is finite (not NaN or infinity)
#[inline]
pub fn dt_math_isfinite(x: f32) -> bool {
    x.is_finite()
}

/// Additional math utilities not in original DetourMath.h but useful
/// Returns the minimum of two f32 values
#[inline]
pub fn dt_math_minf(a: f32, b: f32) -> f32 {
    a.min(b)
}

/// Returns the maximum of two f32 values
#[inline]
pub fn dt_math_maxf(a: f32, b: f32) -> f32 {
    a.max(b)
}

/// Clamps a f32 value between min and max
#[inline]
pub fn dt_math_clampf(x: f32, min: f32, max: f32) -> f32 {
    x.clamp(min, max)
}

/// Linear interpolation between two f32 values
#[inline]
pub fn dt_math_lerpf(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Returns the fractional part of a floating point number
#[inline]
pub fn dt_math_fractf(x: f32) -> f32 {
    x.fract()
}

/// Converts degrees to radians
#[inline]
pub fn dt_math_deg_to_rad(deg: f32) -> f32 {
    deg.to_radians()
}

/// Converts radians to degrees
#[inline]
pub fn dt_math_rad_to_deg(rad: f32) -> f32 {
    rad.to_degrees()
}

/// Returns the sign of a floating point number (-1.0, 0.0, or 1.0)
#[inline]
pub fn dt_math_signf(x: f32) -> f32 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_math() {
        assert_eq!(dt_math_fabsf(-5.0), 5.0);
        assert_eq!(dt_math_fabsf(5.0), 5.0);

        assert_eq!(dt_math_sqrtf(4.0), 2.0);
        assert_eq!(dt_math_sqrtf(9.0), 3.0);

        assert_eq!(dt_math_floorf(3.7), 3.0);
        assert_eq!(dt_math_floorf(-3.7), -4.0);

        assert_eq!(dt_math_ceilf(3.3), 4.0);
        assert_eq!(dt_math_ceilf(-3.3), -3.0);
    }

    #[test]
    fn test_trig() {
        let pi = std::f32::consts::PI;

        assert!((dt_math_cosf(0.0) - 1.0).abs() < 0.0001);
        assert!((dt_math_sinf(0.0) - 0.0).abs() < 0.0001);

        assert!((dt_math_cosf(pi / 2.0) - 0.0).abs() < 0.0001);
        assert!((dt_math_sinf(pi / 2.0) - 1.0).abs() < 0.0001);

        assert_eq!(dt_math_atan2f(0.0, 1.0), 0.0);
        assert!((dt_math_atan2f(1.0, 0.0) - pi / 2.0).abs() < 0.0001);
    }

    #[test]
    fn test_finite() {
        assert!(dt_math_isfinite(0.0));
        assert!(dt_math_isfinite(1.0));
        assert!(dt_math_isfinite(-1.0));
        assert!(!dt_math_isfinite(f32::INFINITY));
        assert!(!dt_math_isfinite(f32::NEG_INFINITY));
        assert!(!dt_math_isfinite(f32::NAN));
    }

    #[test]
    fn test_additional_utils() {
        assert_eq!(dt_math_minf(3.0, 5.0), 3.0);
        assert_eq!(dt_math_maxf(3.0, 5.0), 5.0);

        assert_eq!(dt_math_clampf(5.0, 0.0, 10.0), 5.0);
        assert_eq!(dt_math_clampf(-5.0, 0.0, 10.0), 0.0);
        assert_eq!(dt_math_clampf(15.0, 0.0, 10.0), 10.0);

        assert_eq!(dt_math_lerpf(0.0, 10.0, 0.5), 5.0);
        assert_eq!(dt_math_lerpf(0.0, 10.0, 0.0), 0.0);
        assert_eq!(dt_math_lerpf(0.0, 10.0, 1.0), 10.0);

        assert!((dt_math_fractf(3.7) - 0.7).abs() < 0.0001);
        // Note: Rust's fract() preserves the sign, so -3.7.fract() = -0.7
        assert!((dt_math_fractf(-3.7) - (-0.7)).abs() < 0.0001);

        assert_eq!(dt_math_signf(5.0), 1.0);
        assert_eq!(dt_math_signf(-5.0), -1.0);
        assert_eq!(dt_math_signf(0.0), 0.0);
    }

    #[test]
    fn test_angle_conversion() {
        let pi = std::f32::consts::PI;

        assert!((dt_math_deg_to_rad(180.0) - pi).abs() < 0.0001);
        assert!((dt_math_deg_to_rad(90.0) - pi / 2.0).abs() < 0.0001);

        assert!((dt_math_rad_to_deg(pi) - 180.0).abs() < 0.0001);
        assert!((dt_math_rad_to_deg(pi / 2.0) - 90.0).abs() < 0.0001);
    }
}
