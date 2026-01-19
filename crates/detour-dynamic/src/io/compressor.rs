use std::io::Result as IoResult;

/// Trait for compression and decompression of voxel tile data
pub trait VoxelCompressor {
    /// Compress the input data
    fn compress(&self, data: &[u8]) -> IoResult<Vec<u8>>;
    
    /// Decompress the input data
    fn decompress(&self, data: &[u8]) -> IoResult<Vec<u8>>;
}

/// LZ4 compressor implementation using lz4_flex
pub struct Lz4Compressor;

impl VoxelCompressor for Lz4Compressor {
    fn compress(&self, data: &[u8]) -> IoResult<Vec<u8>> {
        Ok(lz4_flex::compress_prepend_size(data))
    }

    fn decompress(&self, data: &[u8]) -> IoResult<Vec<u8>> {
        lz4_flex::decompress_size_prepended(data)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }
}