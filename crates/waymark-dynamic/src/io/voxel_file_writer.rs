use crate::io::{
    VERSION_COMPRESSION_LZ4, VERSION_EXPORTER_RECAST4J, VOXEL_FILE_MAGIC, VoxelCompressor,
    VoxelFile,
};
use std::io::{Error, ErrorKind, Result as IoResult, Write};

pub struct VoxelFileWriter {
    compressor: Option<Box<dyn VoxelCompressor>>,
}

impl Default for VoxelFileWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl VoxelFileWriter {
    pub fn new() -> Self {
        VoxelFileWriter { compressor: None }
    }

    pub fn with_compressor(compressor: Box<dyn VoxelCompressor>) -> Self {
        VoxelFileWriter {
            compressor: Some(compressor),
        }
    }

    pub fn write<W: Write>(
        &self,
        writer: &mut W,
        file: &VoxelFile,
        use_compression: bool,
    ) -> IoResult<()> {
        // Write magic number (big-endian)
        writer.write_all(&VOXEL_FILE_MAGIC.to_be_bytes())?;

        // Calculate version with compression and exporter flags
        let mut version = file.version;
        version |= VERSION_EXPORTER_RECAST4J;
        if use_compression && self.compressor.is_some() {
            version |= VERSION_COMPRESSION_LZ4;
        }

        // Write version (big-endian)
        writer.write_all(&version.to_be_bytes())?;

        // Write walkable parameters
        writer.write_all(&file.walkable_radius.to_be_bytes())?;
        writer.write_all(&file.walkable_height.to_be_bytes())?;
        writer.write_all(&file.walkable_climb.to_be_bytes())?;
        writer.write_all(&file.walkable_slope_angle.to_be_bytes())?;
        writer.write_all(&file.cell_size.to_be_bytes())?;
        writer.write_all(&file.max_simplification_error.to_be_bytes())?;
        writer.write_all(&file.max_edge_len.to_be_bytes())?;
        writer.write_all(&file.min_region_area.to_be_bytes())?;

        // Write recast4j specific fields
        writer.write_all(&file.region_merge_area.to_be_bytes())?;
        writer.write_all(&file.verts_per_poly.to_be_bytes())?;
        writer.write_all(&[if file.build_mesh_detail { 1u8 } else { 0u8 }])?;
        writer.write_all(&file.detail_sample_distance.to_be_bytes())?;
        writer.write_all(&file.detail_sample_max_error.to_be_bytes())?;

        // Write tile configuration
        writer.write_all(&[if file.use_tiles { 1u8 } else { 0u8 }])?;
        writer.write_all(&file.tile_size_x.to_be_bytes())?;
        writer.write_all(&file.tile_size_z.to_be_bytes())?;

        // Write rotation
        writer.write_all(&file.rotation.x.to_be_bytes())?;
        writer.write_all(&file.rotation.y.to_be_bytes())?;
        writer.write_all(&file.rotation.z.to_be_bytes())?;

        // Write bounds
        for bound in &file.bounds {
            writer.write_all(&bound.to_be_bytes())?;
        }

        // Write tiles
        writer.write_all(&(file.tiles.len() as i32).to_be_bytes())?;

        for tile in &file.tiles {
            writer.write_all(&tile.tile_x.to_be_bytes())?;
            writer.write_all(&tile.tile_z.to_be_bytes())?;
            writer.write_all(&tile.width.to_be_bytes())?;
            writer.write_all(&tile.depth.to_be_bytes())?;
            writer.write_all(&tile.border_size.to_be_bytes())?;

            // Write bounds
            writer.write_all(&tile.bounds_min.x.to_be_bytes())?;
            writer.write_all(&tile.bounds_min.y.to_be_bytes())?;
            writer.write_all(&tile.bounds_min.z.to_be_bytes())?;
            writer.write_all(&tile.bounds_max.x.to_be_bytes())?;
            writer.write_all(&tile.bounds_max.y.to_be_bytes())?;
            writer.write_all(&tile.bounds_max.z.to_be_bytes())?;

            writer.write_all(&tile.cell_size.to_be_bytes())?;
            writer.write_all(&tile.cell_height.to_be_bytes())?;

            // Handle voxel data compression
            let voxel_data = if use_compression {
                if let Some(ref compressor) = self.compressor {
                    compressor.compress(&tile.span_data)?
                } else {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "Compression requested but no compressor provided",
                    ));
                }
            } else {
                tile.span_data.clone()
            };

            writer.write_all(&(voxel_data.len() as i32).to_be_bytes())?;
            writer.write_all(&voxel_data)?;
        }

        Ok(())
    }
}
