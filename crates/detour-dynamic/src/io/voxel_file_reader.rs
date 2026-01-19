use std::io::{Read, Result as IoResult, Error, ErrorKind};
use glam::Vec3;
use crate::io::{VoxelFile, VoxelTile, VoxelCompressor, VOXEL_FILE_MAGIC, VERSION_EXPORTER_MASK, VERSION_COMPRESSION_MASK, VERSION_COMPRESSION_LZ4};

pub struct VoxelFileReader {
    compressor: Option<Box<dyn VoxelCompressor>>,
}

impl Default for VoxelFileReader {
    fn default() -> Self {
        Self::new()
    }
}

impl VoxelFileReader {
    pub fn new() -> Self {
        VoxelFileReader {
            compressor: None,
        }
    }

    pub fn with_compressor(compressor: Box<dyn VoxelCompressor>) -> Self {
        VoxelFileReader {
            compressor: Some(compressor),
        }
    }

    pub fn read<R: Read>(&self, reader: &mut R) -> IoResult<VoxelFile> {
        let mut buffer = [0u8; 4];
        
        // Read and validate magic number
        reader.read_exact(&mut buffer)?;
        let mut magic = u32::from_be_bytes(buffer);
        let mut big_endian = true;
        
        if magic != VOXEL_FILE_MAGIC {
            magic = u32::from_le_bytes(buffer);
            if magic != VOXEL_FILE_MAGIC {
                return Err(Error::new(ErrorKind::InvalidData, "Invalid voxel file magic"));
            }
            big_endian = false;
        }

        let mut file = VoxelFile::new();

        // Read version
        reader.read_exact(&mut buffer)?;
        file.version = if big_endian {
            u32::from_be_bytes(buffer)
        } else {
            u32::from_le_bytes(buffer)
        };

        let is_exported_from_astar = (file.version & VERSION_EXPORTER_MASK) == 0;
        let compression = (file.version & VERSION_COMPRESSION_MASK) == VERSION_COMPRESSION_LZ4;

        // Read walkable parameters
        file.walkable_radius = self.read_f32(reader, big_endian)?;
        file.walkable_height = self.read_f32(reader, big_endian)?;
        file.walkable_climb = self.read_f32(reader, big_endian)?;
        file.walkable_slope_angle = self.read_f32(reader, big_endian)?;
        file.cell_size = self.read_f32(reader, big_endian)?;
        file.max_simplification_error = self.read_f32(reader, big_endian)?;
        file.max_edge_len = self.read_f32(reader, big_endian)?;
        file.min_region_area = self.read_f32(reader, big_endian)?;

        if !is_exported_from_astar {
            file.region_merge_area = self.read_f32(reader, big_endian)?;
            file.verts_per_poly = self.read_i32(reader, big_endian)?;
            file.build_mesh_detail = self.read_u8(reader)? != 0;
            file.detail_sample_distance = self.read_f32(reader, big_endian)?;
            file.detail_sample_max_error = self.read_f32(reader, big_endian)?;
        } else {
            file.region_merge_area = 6.0 * file.min_region_area;
            file.verts_per_poly = 6;
            file.build_mesh_detail = true;
            file.detail_sample_distance = file.max_edge_len * 0.5;
            file.detail_sample_max_error = file.max_simplification_error * 0.8;
        }

        file.use_tiles = self.read_u8(reader)? != 0;
        file.tile_size_x = self.read_i32(reader, big_endian)?;
        file.tile_size_z = self.read_i32(reader, big_endian)?;
        
        file.rotation.x = self.read_f32(reader, big_endian)?;
        file.rotation.y = self.read_f32(reader, big_endian)?;
        file.rotation.z = self.read_f32(reader, big_endian)?;
        
        for i in 0..6 {
            file.bounds[i] = self.read_f32(reader, big_endian)?;
        }

        if is_exported_from_astar {
            // Bounds are saved as center + size, convert to min + max
            file.bounds[0] -= 0.5 * file.bounds[3];
            file.bounds[1] -= 0.5 * file.bounds[4];
            file.bounds[2] -= 0.5 * file.bounds[5];
            file.bounds[3] += file.bounds[0];
            file.bounds[4] += file.bounds[1];
            file.bounds[5] += file.bounds[2];
        }

        // Read tiles
        let tile_count = self.read_i32(reader, big_endian)?;
        for _ in 0..tile_count {
            let tile_x = self.read_i32(reader, big_endian)?;
            let tile_z = self.read_i32(reader, big_endian)?;
            let width = self.read_i32(reader, big_endian)?;
            let depth = self.read_i32(reader, big_endian)?;
            let border_size = self.read_i32(reader, big_endian)?;
            
            let mut bounds_min = Vec3::new(
                self.read_f32(reader, big_endian)?,
                self.read_f32(reader, big_endian)?,
                self.read_f32(reader, big_endian)?,
            );
            
            let mut bounds_max = Vec3::new(
                self.read_f32(reader, big_endian)?,
                self.read_f32(reader, big_endian)?,
                self.read_f32(reader, big_endian)?,
            );

            if is_exported_from_astar {
                // Bounds are local, make them global
                bounds_min.x += file.bounds[0];
                bounds_min.y += file.bounds[1];
                bounds_min.z += file.bounds[2];
                bounds_max.x += file.bounds[0];
                bounds_max.y += file.bounds[1];
                bounds_max.z += file.bounds[2];
            }

            let cell_size = self.read_f32(reader, big_endian)?;
            let cell_height = self.read_f32(reader, big_endian)?;
            let voxel_size = self.read_i32(reader, big_endian)? as usize;

            let mut voxel_data = vec![0u8; voxel_size];
            reader.read_exact(&mut voxel_data)?;

            if compression {
                if let Some(ref compressor) = self.compressor {
                    voxel_data = compressor.decompress(&voxel_data)?;
                } else {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "Compressed voxel file requires compressor",
                    ));
                }
            }

            let tile = VoxelTile::new(
                tile_x,
                tile_z,
                width,
                depth,
                bounds_min,
                bounds_max,
                cell_size,
                cell_height,
                border_size,
                voxel_data,
            );

            file.add_tile(tile);
        }

        Ok(file)
    }

    fn read_f32<R: Read>(&self, reader: &mut R, big_endian: bool) -> IoResult<f32> {
        let mut buffer = [0u8; 4];
        reader.read_exact(&mut buffer)?;
        Ok(if big_endian {
            f32::from_be_bytes(buffer)
        } else {
            f32::from_le_bytes(buffer)
        })
    }

    fn read_i32<R: Read>(&self, reader: &mut R, big_endian: bool) -> IoResult<i32> {
        let mut buffer = [0u8; 4];
        reader.read_exact(&mut buffer)?;
        Ok(if big_endian {
            i32::from_be_bytes(buffer)
        } else {
            i32::from_le_bytes(buffer)
        })
    }

    fn read_u8<R: Read>(&self, reader: &mut R) -> IoResult<u8> {
        let mut buffer = [0u8; 1];
        reader.read_exact(&mut buffer)?;
        Ok(buffer[0])
    }
}