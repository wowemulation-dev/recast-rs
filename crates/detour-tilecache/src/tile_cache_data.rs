//! Tile cache data structures and serialization
//!
//! This module defines the data structures used for tile caching
//! and provides serialization/deserialization functionality.

use crate::error::TileCacheError;

/// Magic number for tile cache data
const TILECACHE_MAGIC: u32 = 0x4C494554; // 'TILE' in ASCII

/// Version number for tile cache data format
const TILECACHE_VERSION: u32 = 1;

/// Tile cache layer header
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct TileCacheLayerHeader {
    /// Magic number for validation
    pub(crate) magic: u32,
    /// Version of the tile cache format
    pub(crate) version: u32,
    /// Tile position X
    pub(crate) tx: i32,
    /// Tile position Y
    pub(crate) ty: i32,
    /// Tile layer
    pub(crate) tlayer: i32,
    /// Bounding box minimum
    pub(crate) bmin: [f32; 3],
    /// Bounding box maximum
    pub(crate) bmax: [f32; 3],
    /// Height of the layer (in cells)
    pub(crate) hmin: u16,
    /// Height of the layer (in cells)
    pub(crate) hmax: u16,
    /// Width of the layer (in cells)
    pub(crate) width: u8,
    /// Height of the layer (in cells)
    pub(crate) height: u8,
    /// Minimum region id
    pub(crate) minx: u8,
    /// Maximum region id
    pub(crate) maxx: u8,
    /// Minimum region id
    pub(crate) miny: u8,
    /// Maximum region id
    pub(crate) maxy: u8,
}

impl Default for TileCacheLayerHeader {
    fn default() -> Self {
        Self::new()
    }
}

impl TileCacheLayerHeader {
    /// Creates a new tile cache layer header
    pub fn new() -> Self {
        Self {
            magic: TILECACHE_MAGIC,
            version: TILECACHE_VERSION,
            tx: 0,
            ty: 0,
            tlayer: 0,
            bmin: [0.0; 3],
            bmax: [0.0; 3],
            hmin: 0,
            hmax: 0,
            width: 0,
            height: 0,
            minx: 0,
            maxx: 0,
            miny: 0,
            maxy: 0,
        }
    }

    /// Validates the header
    pub fn validate(&self) -> Result<(), TileCacheError> {
        if self.magic != TILECACHE_MAGIC {
            return Err(TileCacheError::InvalidParam);
        }
        if self.version != TILECACHE_VERSION {
            return Err(TileCacheError::InvalidParam);
        }
        Ok(())
    }

    /// Serializes the header to bytes
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(56);

        // Write magic and version
        bytes.extend_from_slice(&self.magic.to_le_bytes());
        bytes.extend_from_slice(&self.version.to_le_bytes());

        // Write tile position
        bytes.extend_from_slice(&self.tx.to_le_bytes());
        bytes.extend_from_slice(&self.ty.to_le_bytes());
        bytes.extend_from_slice(&self.tlayer.to_le_bytes());

        // Write bounding box
        for i in 0..3 {
            bytes.extend_from_slice(&self.bmin[i].to_le_bytes());
        }
        for i in 0..3 {
            bytes.extend_from_slice(&self.bmax[i].to_le_bytes());
        }

        // Write dimensions
        bytes.extend_from_slice(&self.hmin.to_le_bytes());
        bytes.extend_from_slice(&self.hmax.to_le_bytes());
        bytes.push(self.width);
        bytes.push(self.height);
        bytes.push(self.minx);
        bytes.push(self.maxx);
        bytes.push(self.miny);
        bytes.push(self.maxy);

        bytes
    }

    /// Deserializes the header from bytes
    pub fn from_bytes(data: &[u8]) -> Result<Self, TileCacheError> {
        if data.len() < 54 {
            return Err(TileCacheError::InvalidParam);
        }

        let mut offset = 0;

        // Read magic and version
        let magic = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;
        let version = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;

        // Read tile position
        let tx = i32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;
        let ty = i32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;
        let tlayer = i32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;

        // Read bounding box
        let mut bmin = [0.0f32; 3];
        for item in &mut bmin {
            *item = f32::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
            ]);
            offset += 4;
        }
        let mut bmax = [0.0f32; 3];
        for item in &mut bmax {
            *item = f32::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
            ]);
            offset += 4;
        }

        // Read dimensions
        let hmin = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        let hmax = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        let width = data[offset];
        offset += 1;
        let height = data[offset];
        offset += 1;
        let minx = data[offset];
        offset += 1;
        let maxx = data[offset];
        offset += 1;
        let miny = data[offset];
        offset += 1;
        let maxy = data[offset];

        let header = Self {
            magic,
            version,
            tx,
            ty,
            tlayer,
            bmin,
            bmax,
            hmin,
            hmax,
            width,
            height,
            minx,
            maxx,
            miny,
            maxy,
        };

        header.validate()?;
        Ok(header)
    }

    /// Returns the magic number.
    pub fn magic(&self) -> u32 {
        self.magic
    }

    /// Returns the format version.
    pub fn version(&self) -> u32 {
        self.version
    }

    /// Returns the tile X position.
    pub fn tx(&self) -> i32 {
        self.tx
    }

    /// Returns the tile Y position.
    pub fn ty(&self) -> i32 {
        self.ty
    }

    /// Returns the tile layer index.
    pub fn tlayer(&self) -> i32 {
        self.tlayer
    }

    /// Returns the bounding box minimum.
    pub fn bmin(&self) -> [f32; 3] {
        self.bmin
    }

    /// Returns the bounding box maximum.
    pub fn bmax(&self) -> [f32; 3] {
        self.bmax
    }

    /// Returns the minimum height (in cells).
    pub fn hmin(&self) -> u16 {
        self.hmin
    }

    /// Returns the maximum height (in cells).
    pub fn hmax(&self) -> u16 {
        self.hmax
    }

    /// Returns the layer width (in cells).
    pub fn width(&self) -> u8 {
        self.width
    }

    /// Returns the layer height (in cells).
    pub fn height(&self) -> u8 {
        self.height
    }

    /// Returns the minimum X region bound.
    pub fn minx(&self) -> u8 {
        self.minx
    }

    /// Returns the maximum X region bound.
    pub fn maxx(&self) -> u8 {
        self.maxx
    }

    /// Returns the minimum Y region bound.
    pub fn miny(&self) -> u8 {
        self.miny
    }

    /// Returns the maximum Y region bound.
    pub fn maxy(&self) -> u8 {
        self.maxy
    }
}

/// Compressed tile cache layer data
#[derive(Debug, Clone)]
pub struct TileCacheLayer {
    /// Header information
    pub header: TileCacheLayerHeader,
    /// Compressed region data
    pub regons: Vec<u8>,
    /// Compressed area data
    pub areas: Vec<u8>,
    /// Compressed connection data
    pub cons: Vec<u8>,
}

impl TileCacheLayer {
    /// Creates a new tile cache layer
    pub fn new(header: TileCacheLayerHeader) -> Self {
        Self {
            header,
            regons: Vec::new(),
            areas: Vec::new(),
            cons: Vec::new(),
        }
    }

    /// Serializes the layer to bytes (uncompressed)
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        // Write header
        bytes.extend_from_slice(&self.header.to_bytes());

        // Write data lengths
        bytes.extend_from_slice(&(self.regons.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&(self.areas.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&(self.cons.len() as u32).to_le_bytes());

        // Write data
        bytes.extend_from_slice(&self.regons);
        bytes.extend_from_slice(&self.areas);
        bytes.extend_from_slice(&self.cons);

        bytes
    }

    /// Deserializes the layer from bytes (uncompressed)
    pub fn from_bytes(data: &[u8]) -> Result<Self, TileCacheError> {
        if data.len() < 66 {
            // Header (54) + 3 lengths (12)
            return Err(TileCacheError::InvalidParam);
        }

        let header = TileCacheLayerHeader::from_bytes(data)?;
        let mut offset = 54; // Header size

        // Read data lengths
        let regons_len = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]) as usize;
        offset += 4;
        let areas_len = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]) as usize;
        offset += 4;
        let cons_len = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]) as usize;
        offset += 4;

        // Validate lengths
        if offset + regons_len + areas_len + cons_len > data.len() {
            return Err(TileCacheError::InvalidParam);
        }

        // Read data
        let regons = data[offset..offset + regons_len].to_vec();
        offset += regons_len;
        let areas = data[offset..offset + areas_len].to_vec();
        offset += areas_len;
        let cons = data[offset..offset + cons_len].to_vec();

        Ok(Self {
            header,
            regons,
            areas,
            cons,
        })
    }
}

/// Tile cache builder configuration
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct TileCacheBuilderConfig {
    /// Cell size
    pub cs: f32,
    /// Cell height
    pub ch: f32,
    /// Agent height
    pub walkable_height: i32,
    /// Agent radius
    pub walkable_radius: i32,
    /// Agent max climb
    pub walkable_climb: i32,
    /// Maximum edge length
    pub max_edge_len: f32,
    /// Maximum simplification error
    pub max_simplification_error: f32,
    /// Minimum region size
    pub min_region_area: i32,
    /// Region merge area
    pub merge_region_area: i32,
    /// Maximum vertices per polygon
    pub max_verts_per_poly: i32,
}

impl Default for TileCacheBuilderConfig {
    fn default() -> Self {
        Self {
            cs: 0.3,
            ch: 0.2,
            walkable_height: 20,
            walkable_radius: 6,
            walkable_climb: 9,
            max_edge_len: 12.0,
            max_simplification_error: 1.3,
            min_region_area: 8,
            merge_region_area: 20,
            max_verts_per_poly: 6,
        }
    }
}
