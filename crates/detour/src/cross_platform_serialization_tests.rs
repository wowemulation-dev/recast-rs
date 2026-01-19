//! Cross-platform serialization tests
//!
//! These tests verify that navigation mesh data can be correctly
//! serialized and deserialized across different platforms.

#[cfg(test)]
mod tests {
    use crate::binary_format::{DT_NAVMESH_MAGIC, DT_NAVMESH_VERSION};
    use crate::{NavMeshParams, PolyFlags, PolyRef};
    use byteorder::{BigEndian, LittleEndian, ReadBytesExt, WriteBytesExt};
    use recast_common::Result;
    use std::io::Cursor;

    /// Test that we can write and read data in little-endian format
    #[test]
    fn test_little_endian_serialization() -> Result<()> {
        let mut buffer = Vec::new();

        // Write test data in little-endian
        buffer.write_u32::<LittleEndian>(DT_NAVMESH_MAGIC)?;
        buffer.write_u32::<LittleEndian>(DT_NAVMESH_VERSION)?;
        buffer.write_f32::<LittleEndian>(123.456)?;
        buffer.write_i32::<LittleEndian>(-42)?;

        // Read it back
        let mut cursor = Cursor::new(&buffer);
        assert_eq!(cursor.read_u32::<LittleEndian>()?, DT_NAVMESH_MAGIC);
        assert_eq!(cursor.read_u32::<LittleEndian>()?, DT_NAVMESH_VERSION);
        assert!((cursor.read_f32::<LittleEndian>()? - 123.456).abs() < f32::EPSILON);
        assert_eq!(cursor.read_i32::<LittleEndian>()?, -42);

        Ok(())
    }

    /// Test that we can write and read data in big-endian format
    #[test]
    fn test_big_endian_serialization() -> Result<()> {
        let mut buffer = Vec::new();

        // Write test data in big-endian
        buffer.write_u32::<BigEndian>(DT_NAVMESH_MAGIC)?;
        buffer.write_u32::<BigEndian>(DT_NAVMESH_VERSION)?;
        buffer.write_f32::<BigEndian>(123.456)?;
        buffer.write_i32::<BigEndian>(-42)?;

        // Read it back
        let mut cursor = Cursor::new(&buffer);
        assert_eq!(cursor.read_u32::<BigEndian>()?, DT_NAVMESH_MAGIC);
        assert_eq!(cursor.read_u32::<BigEndian>()?, DT_NAVMESH_VERSION);
        assert!((cursor.read_f32::<BigEndian>()? - 123.456).abs() < f32::EPSILON);
        assert_eq!(cursor.read_i32::<BigEndian>()?, -42);

        Ok(())
    }

    /// Test cross-endian compatibility
    #[test]
    fn test_cross_endian_compatibility() -> Result<()> {
        // Write in little-endian
        let mut le_buffer = Vec::new();
        le_buffer.write_u32::<LittleEndian>(0x12345678)?;
        le_buffer.write_u16::<LittleEndian>(0xABCD)?;

        // Write same data in big-endian
        let mut be_buffer = Vec::new();
        be_buffer.write_u32::<BigEndian>(0x12345678)?;
        be_buffer.write_u16::<BigEndian>(0xABCD)?;

        // Verify the byte representation is different
        assert_ne!(le_buffer, be_buffer);

        // But reading with correct endianness gives same values
        let mut le_cursor = Cursor::new(&le_buffer);
        let le_u32 = le_cursor.read_u32::<LittleEndian>()?;
        let le_u16 = le_cursor.read_u16::<LittleEndian>()?;

        let mut be_cursor = Cursor::new(&be_buffer);
        let be_u32 = be_cursor.read_u32::<BigEndian>()?;
        let be_u16 = be_cursor.read_u16::<BigEndian>()?;

        assert_eq!(le_u32, be_u32);
        assert_eq!(le_u16, be_u16);

        Ok(())
    }

    /// Test PolyRef serialization consistency
    #[test]
    fn test_polyref_serialization() -> Result<()> {
        let poly_refs = vec![
            PolyRef::new(0),
            PolyRef::new(1),
            PolyRef::new(0xFFFFFFFF),
            PolyRef::new(0x12345678),
        ];

        for poly_ref in poly_refs {
            // Test little-endian
            let mut le_buffer = Vec::new();
            le_buffer.write_u32::<LittleEndian>(poly_ref.id())?;

            let mut le_cursor = Cursor::new(&le_buffer);
            let le_read = PolyRef::new(le_cursor.read_u32::<LittleEndian>()?);
            assert_eq!(poly_ref, le_read);

            // Test big-endian
            let mut be_buffer = Vec::new();
            be_buffer.write_u32::<BigEndian>(poly_ref.id())?;

            let mut be_cursor = Cursor::new(&be_buffer);
            let be_read = PolyRef::new(be_cursor.read_u32::<BigEndian>()?);
            assert_eq!(poly_ref, be_read);
        }

        Ok(())
    }

    /// Test floating point array serialization
    #[test]
    fn test_float_array_serialization() -> Result<()> {
        let positions: Vec<[f32; 3]> = vec![
            [1.0, 2.0, 3.0],
            [-1.0, -2.0, -3.0],
            [1e10, 1e-10, 0.0],
            [f32::MIN_POSITIVE, f32::MAX, -f32::MIN_POSITIVE],
        ];

        // Write in little-endian
        let mut buffer = Vec::new();
        for pos in &positions {
            for &val in pos {
                buffer.write_f32::<LittleEndian>(val)?;
            }
        }

        // Read back
        let mut cursor = Cursor::new(&buffer);
        for original_pos in &positions {
            let mut read_pos = [0.0_f32; 3];
            for val in &mut read_pos {
                *val = cursor.read_f32::<LittleEndian>()?;
            }

            // Compare with epsilon for floating point
            for i in 0..3 {
                let diff = (original_pos[i] - read_pos[i]).abs();
                let max_val = original_pos[i].abs().max(read_pos[i].abs());
                let relative_error = if max_val > 0.0 { diff / max_val } else { diff };
                assert!(
                    relative_error < 1e-6 || diff < 1e-6,
                    "Float mismatch at index {}: {} vs {}",
                    i,
                    original_pos[i],
                    read_pos[i]
                );
            }
        }

        Ok(())
    }

    /// Test struct padding and alignment in serialization
    #[test]
    fn test_struct_padding() -> Result<()> {
        #[repr(C)]
        struct TestStruct {
            a: u8,
            b: u32,
            c: u16,
            d: u8,
        }

        // Size should include padding
        let size = std::mem::size_of::<TestStruct>();
        assert!(size >= 1 + 4 + 2 + 1); // Minimum size without padding
        assert_eq!(size % 4, 0); // Should be aligned to 4 bytes

        // Test manual serialization with explicit padding
        let test = TestStruct {
            a: 0x12,
            b: 0x34567890,
            c: 0xABCD,
            d: 0xEF,
        };

        let mut buffer = Vec::new();
        buffer.write_u8(test.a)?;
        // Add padding to align b to 4-byte boundary
        while buffer.len() % 4 != 0 {
            buffer.write_u8(0)?;
        }
        buffer.write_u32::<LittleEndian>(test.b)?;
        buffer.write_u16::<LittleEndian>(test.c)?;
        buffer.write_u8(test.d)?;
        // Add final padding
        while buffer.len() % 4 != 0 {
            buffer.write_u8(0)?;
        }

        // Verify alignment
        assert_eq!(buffer.len() % 4, 0);

        Ok(())
    }

    /// Test serialization of navigation mesh parameters
    #[test]
    fn test_navmesh_params_serialization() -> Result<()> {
        let params = NavMeshParams {
            origin: [100.0, 200.0, 300.0],
            tile_width: 32.0,
            tile_height: 32.0,
            max_tiles: 128,
            max_polys_per_tile: 256,
        };

        // Serialize
        let mut buffer = Vec::new();
        for &val in &params.origin {
            buffer.write_f32::<LittleEndian>(val)?;
        }
        buffer.write_f32::<LittleEndian>(params.tile_width)?;
        buffer.write_f32::<LittleEndian>(params.tile_height)?;
        buffer.write_i32::<LittleEndian>(params.max_tiles)?;
        buffer.write_i32::<LittleEndian>(params.max_polys_per_tile)?;

        // Deserialize
        let mut cursor = Cursor::new(&buffer);
        let mut read_origin = [0.0_f32; 3];
        for val in &mut read_origin {
            *val = cursor.read_f32::<LittleEndian>()?;
        }
        let read_params = NavMeshParams {
            origin: read_origin,
            tile_width: cursor.read_f32::<LittleEndian>()?,
            tile_height: cursor.read_f32::<LittleEndian>()?,
            max_tiles: cursor.read_i32::<LittleEndian>()?,
            max_polys_per_tile: cursor.read_i32::<LittleEndian>()?,
        };

        // Compare
        assert_eq!(params.origin, read_params.origin);
        assert_eq!(params.tile_width, read_params.tile_width);
        assert_eq!(params.tile_height, read_params.tile_height);
        assert_eq!(params.max_tiles, read_params.max_tiles);
        assert_eq!(params.max_polys_per_tile, read_params.max_polys_per_tile);

        Ok(())
    }

    /// Test bitflags serialization
    #[test]
    fn test_bitflags_serialization() -> Result<()> {
        let flags_values = vec![
            PolyFlags::WALK,
            PolyFlags::SWIM,
            PolyFlags::DOOR,
            PolyFlags::WALK | PolyFlags::SWIM,
            PolyFlags::all(),
            PolyFlags::empty(),
        ];

        for flags in flags_values {
            // Serialize as u16
            let mut buffer = Vec::new();
            buffer.write_u16::<LittleEndian>(flags.bits())?;

            // Deserialize
            let mut cursor = Cursor::new(&buffer);
            let bits = cursor.read_u16::<LittleEndian>()?;
            let read_flags = PolyFlags::from_bits(bits).unwrap();

            assert_eq!(flags, read_flags);
        }

        Ok(())
    }

    /// Test that magic numbers are correct for endianness
    #[test]
    fn test_magic_number_endianness() {
        // DT_NAVMESH_MAGIC should be 'DNAV' (0x4E415644 in big-endian)
        let magic_bytes = DT_NAVMESH_MAGIC.to_le_bytes();
        assert_eq!(magic_bytes, [b'D', b'N', b'A', b'V']);

        // Verify it's different in big-endian
        let be_bytes = DT_NAVMESH_MAGIC.to_be_bytes();
        assert_ne!(be_bytes, [b'D', b'N', b'A', b'V']);
    }
}
