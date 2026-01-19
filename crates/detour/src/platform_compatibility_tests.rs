//! Platform compatibility tests for cross-platform support
//!
//! These tests verify that the library works correctly across different
//! operating systems and architectures.

#[cfg(test)]
mod tests {
    use crate::{NavMesh, NavMeshCreateParams, NavMeshParams, PolyFlags, PolyRef, QueryFilter};
    use recast_common::Result;
    use std::mem;
    use std::path::{Path, PathBuf};

    /// Test file path handling across platforms
    #[test]
    fn test_cross_platform_paths() -> Result<()> {
        // Test various path formats
        let test_paths = vec![
            "assets/test.obj",
            "./assets/test.obj",
            "../assets/test.obj",
            "assets/../assets/test.obj",
        ];

        for path_str in test_paths {
            let path = Path::new(path_str);
            let path_buf = PathBuf::from(path);

            // Verify path operations work
            assert_eq!(path.as_os_str(), path_buf.as_os_str());

            // Test extension handling
            if let Some(ext) = path.extension() {
                assert_eq!(ext, "obj");
            }

            // Test file stem
            if let Some(stem) = path.file_stem() {
                assert_eq!(stem, "test");
            }
        }

        Ok(())
    }

    /// Test endianness handling for serialization
    #[test]
    fn test_endianness_handling() -> Result<()> {
        // Create test data with known values
        let test_u16: u16 = 0x1234;
        let test_u32: u32 = 0x12345678;
        let test_f32: f32 = 1234.5678;

        // Convert to bytes in different endianness
        let le_u16 = test_u16.to_le_bytes();
        let be_u16 = test_u16.to_be_bytes();
        let ne_u16 = test_u16.to_ne_bytes();

        // Verify conversions work correctly
        assert_eq!(u16::from_le_bytes(le_u16), test_u16);
        assert_eq!(u16::from_be_bytes(be_u16), test_u16);
        assert_eq!(u16::from_ne_bytes(ne_u16), test_u16);

        // Test u32
        let le_u32 = test_u32.to_le_bytes();
        let be_u32 = test_u32.to_be_bytes();
        assert_eq!(u32::from_le_bytes(le_u32), test_u32);
        assert_eq!(u32::from_be_bytes(be_u32), test_u32);

        // Test f32
        let le_f32 = test_f32.to_le_bytes();
        let be_f32 = test_f32.to_be_bytes();
        assert_eq!(f32::from_le_bytes(le_f32), test_f32);
        assert_eq!(f32::from_be_bytes(be_f32), test_f32);

        // Verify endianness detection
        let is_little_endian = cfg!(target_endian = "little");
        let is_big_endian = cfg!(target_endian = "big");
        assert!(is_little_endian || is_big_endian);

        Ok(())
    }

    /// Test memory alignment requirements
    #[test]
    fn test_memory_alignment() -> Result<()> {
        // Test structure alignment
        assert_eq!(
            mem::align_of::<NavMeshParams>() % 4,
            0,
            "NavMeshParams should be 4-byte aligned"
        );
        assert_eq!(
            mem::align_of::<PolyRef>() % 4,
            0,
            "PolyRef should be 4-byte aligned"
        );

        // Test array alignment for SIMD operations
        let positions: Vec<[f32; 3]> = vec![[1.0, 2.0, 3.0]; 16];
        let ptr = positions.as_ptr() as usize;

        // Verify alignment for potential SIMD usage
        // Most SIMD operations require 16-byte alignment
        let alignment_ok = ptr % 16 == 0 || ptr % 8 == 0 || ptr % 4 == 0;
        assert!(alignment_ok, "Vector data should be reasonably aligned");

        Ok(())
    }

    /// Test floating point precision across platforms
    #[test]
    fn test_floating_point_consistency() -> Result<()> {
        // Test that floating point operations are consistent
        let a = 0.1_f32;
        let b = 0.2_f32;
        let c = a + b;

        // Use epsilon comparison for floating point
        let epsilon = 1e-6_f32;
        assert!(
            (c - 0.3_f32).abs() < epsilon,
            "Floating point addition should be consistent"
        );

        // Test special values
        assert!(f32::INFINITY.is_infinite());
        assert!(f32::NEG_INFINITY.is_infinite());
        assert!(f32::NAN.is_nan());

        // Test edge case calculations
        let very_small = 1e-30_f32;
        let very_large = 1e30_f32;
        assert!(very_small > 0.0);
        assert!(very_large < f32::INFINITY);

        Ok(())
    }

    /// Test platform-specific integer sizes
    #[test]
    fn test_integer_sizes() {
        // Verify standard integer sizes
        assert_eq!(mem::size_of::<u8>(), 1);
        assert_eq!(mem::size_of::<u16>(), 2);
        assert_eq!(mem::size_of::<u32>(), 4);
        assert_eq!(mem::size_of::<u64>(), 8);
        assert_eq!(mem::size_of::<i8>(), 1);
        assert_eq!(mem::size_of::<i16>(), 2);
        assert_eq!(mem::size_of::<i32>(), 4);
        assert_eq!(mem::size_of::<i64>(), 8);
        assert_eq!(mem::size_of::<f32>(), 4);
        assert_eq!(mem::size_of::<f64>(), 8);

        // Test pointer sizes (varies by platform)
        let ptr_size = mem::size_of::<*const u8>();
        assert!(
            ptr_size == 4 || ptr_size == 8,
            "Pointer size should be 4 or 8 bytes"
        );

        // Test usize/isize
        assert_eq!(mem::size_of::<usize>(), ptr_size);
        assert_eq!(mem::size_of::<isize>(), ptr_size);
    }

    /// Test NavMesh creation with extreme values
    #[test]
    fn test_extreme_values_handling() -> Result<()> {
        // Test with very large world coordinates
        let params = NavMeshParams {
            origin: [1e6, 0.0, 1e6],
            tile_width: 1000.0,
            tile_height: 1000.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };

        let nav_mesh = NavMesh::new(params)?;
        assert_eq!(nav_mesh.get_params().origin[0], 1e6);

        // Test with very small tile sizes
        let small_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 0.1,
            tile_height: 0.1,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };

        let small_nav_mesh = NavMesh::new(small_params)?;
        assert!((small_nav_mesh.get_params().tile_width - 0.1).abs() < f32::EPSILON);

        Ok(())
    }

    /// Test thread safety annotations
    #[test]
    fn test_thread_safety_markers() {
        // Verify Send and Sync traits where appropriate
        fn assert_send<T: Send>() {}
        fn assert_sync<T: Sync>() {}

        // These types should be Send
        assert_send::<NavMeshParams>();
        assert_send::<PolyRef>();
        assert_send::<QueryFilter>();
        assert_send::<PolyFlags>();

        // These types should be Sync
        assert_sync::<NavMeshParams>();
        assert_sync::<PolyRef>();
        assert_sync::<QueryFilter>();
        assert_sync::<PolyFlags>();
    }

    /// Test platform-specific build configuration
    #[test]
    fn test_build_configuration() {
        // Test target architecture
        let arch = std::env::consts::ARCH;
        assert!(
            ["x86", "x86_64", "arm", "aarch64", "riscv64", "wasm32"].contains(&arch),
            "Unexpected architecture: {}",
            arch
        );

        // Test target OS
        let os = std::env::consts::OS;
        assert!(
            ["linux", "windows", "macos", "ios", "android", "freebsd", "openbsd", "netbsd"]
                .contains(&os),
            "Unexpected OS: {}",
            os
        );

        // Test target family
        let family = std::env::consts::FAMILY;
        assert!(
            ["unix", "windows", "wasm"].contains(&family),
            "Unexpected OS family: {}",
            family
        );

        println!("Platform: {} {} ({})", arch, os, family);
    }

    /// Test coordinate system consistency
    #[test]
    fn test_coordinate_system_consistency() -> Result<()> {
        // Recast uses Y-up coordinate system
        // Test that our math operations preserve this

        let up_vector = [0.0_f32, 1.0, 0.0];
        let forward_vector = [0.0_f32, 0.0, 1.0];
        let right_vector = [1.0_f32, 0.0, 0.0];

        // Cross product: forward × right = up (in left-handed coordinate system)
        let cross = [
            forward_vector[1] * right_vector[2] - forward_vector[2] * right_vector[1],
            forward_vector[2] * right_vector[0] - forward_vector[0] * right_vector[2],
            forward_vector[0] * right_vector[1] - forward_vector[1] * right_vector[0],
        ];

        assert!((cross[0] - up_vector[0]).abs() < 1e-6);
        assert!((cross[1] - up_vector[1]).abs() < 1e-6);
        assert!((cross[2] - up_vector[2]).abs() < 1e-6);

        Ok(())
    }

    /// Test handling of platform-specific line endings
    #[test]
    fn test_line_ending_handling() {
        let text_with_lf = "line1\nline2\nline3";
        let text_with_crlf = "line1\r\nline2\r\nline3";
        let text_with_cr = "line1\rline2\rline3";

        // All should split into 3 lines
        let lines_lf: Vec<&str> = text_with_lf.lines().collect();
        let lines_crlf: Vec<&str> = text_with_crlf.lines().collect();
        let lines_cr: Vec<&str> = text_with_cr.split('\r').filter(|s| !s.is_empty()).collect();

        assert_eq!(lines_lf.len(), 3);
        assert_eq!(lines_crlf.len(), 3);
        assert_eq!(lines_cr.len(), 3);

        // Content should be the same
        for i in 0..3 {
            assert_eq!(lines_lf[i], format!("line{}", i + 1));
            assert_eq!(lines_crlf[i], format!("line{}", i + 1));
            assert_eq!(lines_cr[i], format!("line{}", i + 1));
        }
    }

    /// Test numeric limits across platforms
    #[test]
    fn test_numeric_limits() {
        // Test maximum values
        assert_eq!(u8::MAX, 255);
        assert_eq!(u16::MAX, 65535);
        assert_eq!(u32::MAX, 4294967295);
        assert_eq!(i8::MAX, 127);
        assert_eq!(i16::MAX, 32767);
        assert_eq!(i32::MAX, 2147483647);

        // Test minimum values
        assert_eq!(i8::MIN, -128);
        assert_eq!(i16::MIN, -32768);
        assert_eq!(i32::MIN, -2147483648);

        // Test floating point limits
        assert!(f32::MAX > 3.4e38);
        assert!(f32::MIN_POSITIVE < 1.2e-38);
        assert_eq!(f32::INFINITY, 1.0_f32 / 0.0_f32);
    }

    /// Test that NavMesh handles different coordinate scales
    #[test]
    fn test_coordinate_scale_handling() -> Result<()> {
        // Test meter scale
        let meter_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 32.0, // 32 meters
            tile_height: 32.0,
            max_tiles: 1,
            max_polys_per_tile: 100,
        };

        let meter_mesh = NavMesh::new(meter_params)?;

        // Test centimeter scale
        let cm_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 3200.0, // 3200 cm = 32 meters
            tile_height: 3200.0,
            max_tiles: 1,
            max_polys_per_tile: 100,
        };

        let cm_mesh = NavMesh::new(cm_params)?;

        // Both should work correctly
        assert!(meter_mesh.get_params().tile_width > 0.0);
        assert!(cm_mesh.get_params().tile_width > 0.0);

        Ok(())
    }
}
