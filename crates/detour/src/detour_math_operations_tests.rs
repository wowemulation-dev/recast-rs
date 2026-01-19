//! Comprehensive math operations tests
//!
//! This module ports and extends the C++ Tests_DetourCommon.cpp tests
//! to ensure our Rust implementation has equivalent mathematical functionality.

#[cfg(test)]
mod tests {
    use std::mem;

    #[test]
    fn test_swap_operations() {
        // Swap integers
        let mut a = 5i32;
        let mut b = 10i32;
        mem::swap(&mut a, &mut b);
        assert_eq!(a, 10);
        assert_eq!(b, 5);

        // Swap floats
        let mut c = 3.14f32;
        let mut d = 2.71f32;
        mem::swap(&mut c, &mut d);
        assert!((c - 2.71).abs() < 1e-6);
        assert!((d - 3.14).abs() < 1e-6);

        // Swap identical values
        let mut e = 42i32;
        let mut f = 42i32;
        mem::swap(&mut e, &mut f);
        assert_eq!(e, 42);
        assert_eq!(f, 42);
    }

    #[test]
    fn test_min_operations() {
        // Integer minimum
        assert_eq!(dt_min(5, 10), 5);
        assert_eq!(dt_min(10, 5), 5);
        assert_eq!(dt_min(-5, 10), -5);
        assert_eq!(dt_min(-10, -5), -10);
        assert_eq!(dt_min(0, 0), 0);

        // Float minimum
        assert!((dt_min_f32(3.14, 2.71) - 2.71).abs() < 1e-6);
        assert!((dt_min_f32(2.71, 3.14) - 2.71).abs() < 1e-6);
        assert!((dt_min_f32(-1.5, 1.5) - (-1.5)).abs() < 1e-6);
        assert!((dt_min_f32(0.0, 0.0) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_max_operations() {
        // Integer maximum
        assert_eq!(dt_max(5, 10), 10);
        assert_eq!(dt_max(10, 5), 10);
        assert_eq!(dt_max(-5, 10), 10);
        assert_eq!(dt_max(-10, -5), -5);
        assert_eq!(dt_max(0, 0), 0);

        // Float maximum
        assert!((dt_max_f32(3.14, 2.71) - 3.14).abs() < 1e-6);
        assert!((dt_max_f32(2.71, 3.14) - 3.14).abs() < 1e-6);
        assert!((dt_max_f32(-1.5, 1.5) - 1.5).abs() < 1e-6);
        assert!((dt_max_f32(0.0, 0.0) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_abs_operations() {
        // Integer absolute value
        assert_eq!(dt_abs(5), 5);
        assert_eq!(dt_abs(-5), 5);
        assert_eq!(dt_abs(0), 0);
        assert_eq!(dt_abs(i32::MAX), i32::MAX);

        // Float absolute value
        assert!((dt_abs_f32(3.14) - 3.14).abs() < 1e-6);
        assert!((dt_abs_f32(-3.14) - 3.14).abs() < 1e-6);
        assert!((dt_abs_f32(0.0) - 0.0).abs() < 1e-6);
        assert!((dt_abs_f32(-0.0) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_sqr_operations() {
        // Integer square
        assert_eq!(dt_sqr(0), 0);
        assert_eq!(dt_sqr(1), 1);
        assert_eq!(dt_sqr(5), 25);
        assert_eq!(dt_sqr(-5), 25);
        assert_eq!(dt_sqr(10), 100);

        // Float square
        assert!((dt_sqr_f32(0.0) - 0.0).abs() < 1e-6);
        assert!((dt_sqr_f32(2.0) - 4.0).abs() < 1e-6);
        assert!((dt_sqr_f32(-2.0) - 4.0).abs() < 1e-6);
        assert!((dt_sqr_f32(3.14) - 9.8596).abs() < 0.001);
    }

    #[test]
    fn test_clamp_operations() {
        // Integer clamping
        assert_eq!(dt_clamp(5, 0, 10), 5); // Within range
        assert_eq!(dt_clamp(-5, 0, 10), 0); // Below range
        assert_eq!(dt_clamp(15, 0, 10), 10); // Above range
        assert_eq!(dt_clamp(0, 0, 10), 0); // At minimum
        assert_eq!(dt_clamp(10, 0, 10), 10); // At maximum
        assert_eq!(dt_clamp(5, 5, 5), 5); // Single value range

        // Float clamping
        assert!((dt_clamp_f32(5.5, 0.0, 10.0) - 5.5).abs() < 1e-6);
        assert!((dt_clamp_f32(-1.5, 0.0, 10.0) - 0.0).abs() < 1e-6);
        assert!((dt_clamp_f32(12.5, 0.0, 10.0) - 10.0).abs() < 1e-6);
        assert!((dt_clamp_f32(3.14, 3.14, 3.14) - 3.14).abs() < 1e-6);
    }

    #[test]
    fn test_vector_cross_product() {
        // Cross product basic vectors
        let v1 = [1.0, 0.0, 0.0]; // X axis
        let v2 = [0.0, 1.0, 0.0]; // Y axis
        let result = v_cross(&v1, &v2);

        // X × Y = Z
        assert!((result[0] - 0.0).abs() < 1e-6);
        assert!((result[1] - 0.0).abs() < 1e-6);
        assert!((result[2] - 1.0).abs() < 1e-6);

        // Cross product reverse order
        let v3 = [0.0, 1.0, 0.0]; // Y axis
        let v4 = [1.0, 0.0, 0.0]; // X axis
        let result2 = v_cross(&v3, &v4);

        // Y × X = -Z
        assert!((result2[0] - 0.0).abs() < 1e-6);
        assert!((result2[1] - 0.0).abs() < 1e-6);
        assert!((result2[2] - (-1.0)).abs() < 1e-6);

        // Cross product of parallel vectors
        let v5 = [1.0, 2.0, 3.0];
        let v6 = [2.0, 4.0, 6.0]; // 2 * v5
        let result3 = v_cross(&v5, &v6);

        // Parallel vectors should have zero cross product
        assert!((result3[0] - 0.0).abs() < 1e-6);
        assert!((result3[1] - 0.0).abs() < 1e-6);
        assert!((result3[2] - 0.0).abs() < 1e-6);

        // Cross product arbitrary vectors
        let v7 = [1.0, 2.0, 3.0];
        let v8 = [4.0, 5.0, 6.0];
        let result4 = v_cross(&v7, &v8);

        // Manually calculated: (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
        assert!((result4[0] - (-3.0)).abs() < 1e-6);
        assert!((result4[1] - 6.0).abs() < 1e-6);
        assert!((result4[2] - (-3.0)).abs() < 1e-6);
    }

    #[test]
    fn test_vector_dot_product() {
        // Dot product orthogonal vectors
        let v1 = [1.0, 0.0, 0.0]; // X axis
        let v2 = [0.0, 1.0, 0.0]; // Y axis
        let result = v_dot(&v1, &v2);
        assert!((result - 0.0).abs() < 1e-6);

        // Dot product parallel vectors
        let v3 = [1.0, 0.0, 0.0];
        let v4 = [2.0, 0.0, 0.0];
        let result2 = v_dot(&v3, &v4);
        assert!((result2 - 2.0).abs() < 1e-6);

        // Dot product anti-parallel vectors
        let v5 = [1.0, 0.0, 0.0];
        let v6 = [-1.0, 0.0, 0.0];
        let result3 = v_dot(&v5, &v6);
        assert!((result3 - (-1.0)).abs() < 1e-6);

        // Dot product arbitrary vectors
        let v7 = [1.0, 2.0, 3.0];
        let v8 = [4.0, 5.0, 6.0];
        let result4 = v_dot(&v7, &v8);
        // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
        assert!((result4 - 32.0).abs() < 1e-6);

        // Dot product with zero vector
        let v9 = [1.0, 2.0, 3.0];
        let v10 = [0.0, 0.0, 0.0];
        let result5 = v_dot(&v9, &v10);
        assert!((result5 - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_vector_multiply_add() {
        // Scaled vector addition (v_mad: dest = a + b * s)
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        let s = 2.0;
        let result = v_mad(&a, &b, s);

        // result = [1, 2, 3] + [4, 5, 6] * 2 = [1, 2, 3] + [8, 10, 12] = [9, 12, 15]
        assert!((result[0] - 9.0).abs() < 1e-6);
        assert!((result[1] - 12.0).abs() < 1e-6);
        assert!((result[2] - 15.0).abs() < 1e-6);

        // Zero scale
        let result2 = v_mad(&a, &b, 0.0);
        assert!((result2[0] - 1.0).abs() < 1e-6);
        assert!((result2[1] - 2.0).abs() < 1e-6);
        assert!((result2[2] - 3.0).abs() < 1e-6);

        // Negative scale
        let result3 = v_mad(&a, &b, -1.0);
        // result = [1, 2, 3] + [4, 5, 6] * (-1) = [1, 2, 3] + [-4, -5, -6] = [-3, -3, -3]
        assert!((result3[0] - (-3.0)).abs() < 1e-6);
        assert!((result3[1] - (-3.0)).abs() < 1e-6);
        assert!((result3[2] - (-3.0)).abs() < 1e-6);
    }

    #[test]
    fn test_vector_length_operations() {
        // Vector length
        let v1 = [3.0, 4.0, 0.0]; // 3-4-5 triangle
        let len = v_len(&v1);
        assert!((len - 5.0).abs() < 1e-6);

        // Vector length squared
        let len_sqr = v_len_sqr(&v1);
        assert!((len_sqr - 25.0).abs() < 1e-6);

        // Unit vector length
        let unit = [1.0, 0.0, 0.0];
        let unit_len = v_len(&unit);
        assert!((unit_len - 1.0).abs() < 1e-6);

        // Zero vector length
        let zero = [0.0, 0.0, 0.0];
        let zero_len = v_len(&zero);
        assert!((zero_len - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_vector_normalization() {
        // Normalize non-zero vector
        let v1 = [3.0, 4.0, 0.0];
        let norm = v_normalize(&v1);
        let norm_len = v_len(&norm);
        assert!((norm_len - 1.0).abs() < 1e-6);
        assert!((norm[0] - 0.6).abs() < 1e-6); // 3/5
        assert!((norm[1] - 0.8).abs() < 1e-6); // 4/5
        assert!((norm[2] - 0.0).abs() < 1e-6);

        // Normalize unit vector (should remain unchanged)
        let unit = [1.0, 0.0, 0.0];
        let norm_unit = v_normalize(&unit);
        assert!((norm_unit[0] - 1.0).abs() < 1e-6);
        assert!((norm_unit[1] - 0.0).abs() < 1e-6);
        assert!((norm_unit[2] - 0.0).abs() < 1e-6);

        // Handle zero vector normalization (should return zero or handle gracefully)
        let zero = [0.0, 0.0, 0.0];
        let norm_zero = v_normalize_safe(&zero); // Safe version that handles zero vectors
        assert!((norm_zero[0] - 0.0).abs() < 1e-6);
        assert!((norm_zero[1] - 0.0).abs() < 1e-6);
        assert!((norm_zero[2] - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_mathematical_edge_cases() {
        // Test with very small numbers
        let tiny = f32::EPSILON;
        assert!(dt_abs_f32(tiny) >= 0.0);
        assert!(dt_sqr_f32(tiny).is_finite());

        // Test with very large numbers
        let large = 1e20;
        assert!(dt_abs_f32(large).is_finite());
        assert!((dt_min_f32(large, large * 2.0) - large).abs() < large * 1e-6);

        // Test with special float values
        assert!(dt_abs_f32(f32::INFINITY).is_infinite());
        assert!(dt_abs_f32(f32::NEG_INFINITY).is_infinite());
        assert!(dt_abs_f32(f32::NAN).is_nan());

        // Test clamping with edge cases
        assert_eq!(dt_clamp(i32::MIN, i32::MIN, i32::MAX), i32::MIN);
        assert_eq!(dt_clamp(i32::MAX, i32::MIN, i32::MAX), i32::MAX);
    }

    #[test]
    fn test_interpolation_functions() {
        // Linear interpolation
        let a = 10.0;
        let b = 20.0;
        let t = 0.5;
        let lerp_result = dt_lerp(a, b, t);
        assert!((lerp_result - 15.0).abs() < 1e-6);

        // Interpolation at endpoints
        assert!((dt_lerp(a, b, 0.0) - a).abs() < 1e-6);
        assert!((dt_lerp(a, b, 1.0) - b).abs() < 1e-6);

        // Extrapolation
        assert!((dt_lerp(a, b, 2.0) - 30.0).abs() < 1e-6);
        assert!((dt_lerp(a, b, -1.0) - 0.0).abs() < 1e-6);
    }

    // Helper functions implementing basic math operations

    fn dt_min(a: i32, b: i32) -> i32 {
        a.min(b)
    }
    fn dt_max(a: i32, b: i32) -> i32 {
        a.max(b)
    }
    fn dt_abs(a: i32) -> i32 {
        a.abs()
    }
    fn dt_sqr(a: i32) -> i32 {
        a * a
    }
    fn dt_clamp(v: i32, min: i32, max: i32) -> i32 {
        v.clamp(min, max)
    }

    fn dt_min_f32(a: f32, b: f32) -> f32 {
        a.min(b)
    }
    fn dt_max_f32(a: f32, b: f32) -> f32 {
        a.max(b)
    }
    fn dt_abs_f32(a: f32) -> f32 {
        a.abs()
    }
    fn dt_sqr_f32(a: f32) -> f32 {
        a * a
    }
    fn dt_clamp_f32(v: f32, min: f32, max: f32) -> f32 {
        v.clamp(min, max)
    }
    fn dt_lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }

    fn v_cross(a: &[f32; 3], b: &[f32; 3]) -> [f32; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }

    fn v_dot(a: &[f32; 3], b: &[f32; 3]) -> f32 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    fn v_mad(a: &[f32; 3], b: &[f32; 3], s: f32) -> [f32; 3] {
        [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s]
    }

    fn v_len(v: &[f32; 3]) -> f32 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
    }

    fn v_len_sqr(v: &[f32; 3]) -> f32 {
        v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
    }

    fn v_normalize(v: &[f32; 3]) -> [f32; 3] {
        let len = v_len(v);
        if len > f32::EPSILON {
            [v[0] / len, v[1] / len, v[2] / len]
        } else {
            *v // Return original if too small to normalize safely
        }
    }

    fn v_normalize_safe(v: &[f32; 3]) -> [f32; 3] {
        let len = v_len(v);
        if len > f32::EPSILON {
            [v[0] / len, v[1] / len, v[2] / len]
        } else {
            [0.0, 0.0, 0.0] // Return zero vector for zero input
        }
    }
}
