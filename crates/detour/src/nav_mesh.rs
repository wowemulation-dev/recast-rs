//! Navigation mesh implementation for Detour
//!
//! The navigation mesh is the core data structure for pathfinding in Detour.
//! It consists of polygons connected in a graph-like structure.

use std::collections::HashMap;

use super::bvh_tree::{Aabb, BVHItem, BVHTree};
use super::{
    MAX_VERTS_PER_POLY, NavMeshFlags, NavMeshParams, PolyFlags, PolyRef, PolyType, QueryFilter,
    Status,
};
use recast::{MESH_NULL_IDX, PolyMesh, PolyMeshDetail};
use recast_common::{Error, Result};

/// Maximum number of nodes in the navigation mesh node pool
#[allow(dead_code)]
const DT_MAX_NODES: usize = 4096;

/// Number of bits for polygon id
const DT_POLY_BITS: u32 = 16;
/// Number of bits for tile id
const DT_TILE_BITS: u32 = 10;
/// Number of bits for salt
const DT_SALT_BITS: u32 = 6;

/// Maximum polygon id value (16 bits)
const DT_POLY_MASK: u32 = (1 << DT_POLY_BITS) - 1;
/// Maximum tile id value (10 bits)
const DT_TILE_MASK: u32 = (1 << DT_TILE_BITS) - 1;
/// Maximum salt value (6 bits)
const DT_SALT_MASK: u32 = (1 << DT_SALT_BITS) - 1;

/// Extracts the polygon id from a PolyRef
#[inline]
pub(crate) fn decode_poly_id(reference: PolyRef) -> u32 {
    reference.id() & DT_POLY_MASK
}

/// Extracts the tile id from a PolyRef
#[inline]
pub(crate) fn decode_tile_id(reference: PolyRef) -> u32 {
    (reference.id() >> DT_POLY_BITS) & DT_TILE_MASK
}

/// Extracts the salt value from a PolyRef
#[inline]
fn decode_poly_id_salt(reference: PolyRef) -> u32 {
    (reference.id() >> (DT_POLY_BITS + DT_TILE_BITS)) & DT_SALT_MASK
}

/// Creates a PolyRef from tile and polygon ids
#[allow(dead_code)]
#[inline]
pub fn encode_poly_ref(tile_id: u32, poly_id: u32) -> PolyRef {
    PolyRef::new(((tile_id & DT_TILE_MASK) << DT_POLY_BITS) | (poly_id & DT_POLY_MASK))
}

/// Creates a PolyRef from salt, tile and polygon ids
#[inline]
pub fn encode_poly_ref_with_salt(salt: u32, tile_id: u32, poly_id: u32) -> PolyRef {
    PolyRef::new(
        ((salt & DT_SALT_MASK) << (DT_POLY_BITS + DT_TILE_BITS))
            | ((tile_id & DT_TILE_MASK) << DT_POLY_BITS)
            | (poly_id & DT_POLY_MASK),
    )
}

/// Decodes a PolyRef into tile and polygon ids
/// Note: The returned tile_id is 1-based to avoid PolyRef(0)
#[inline]
pub fn decode_poly_ref(reference: PolyRef) -> (u32, u32) {
    let tile_id = decode_tile_id(reference);
    let poly_id = decode_poly_id(reference);
    (tile_id, poly_id)
}

/// Decodes a PolyRef into salt, tile and polygon ids
#[inline]
pub fn decode_poly_ref_full(reference: PolyRef) -> (u32, u32, u32) {
    let salt = decode_poly_id_salt(reference);
    let tile_id = decode_tile_id(reference);
    let poly_id = decode_poly_id(reference);
    (salt, tile_id, poly_id)
}

/// Converts a 1-based tile ID to a 0-based tile index
#[inline]
pub(crate) fn tile_id_to_index(tile_id: u32) -> Option<usize> {
    if tile_id == 0 {
        None
    } else {
        Some((tile_id - 1) as usize)
    }
}

/// Link between two polygons
#[derive(Debug, Clone, Copy)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Link {
    /// Reference to the connected polygon
    pub reference: PolyRef,
    /// Index of the neighbor polygon
    pub neighbor_index: u8,
    /// Edge index of the connection
    pub edge_index: u8,
    /// Direction flags for the connection
    pub side: u8,
    /// Boundary flag (0 = internal, 1 = tile boundary)
    pub boundary_flag: u8,
    /// Index of the next link in the linked list (None if last)
    pub next: Option<u32>,
}

impl Link {
    /// Creates a new link
    pub fn new(
        reference: PolyRef,
        neighbor_index: u8,
        edge_index: u8,
        side: u8,
        boundary_flag: u8,
    ) -> Self {
        Self {
            reference,
            neighbor_index,
            edge_index,
            side,
            boundary_flag,
            next: None,
        }
    }

    /// Creates a new link with a specified next link
    pub fn new_with_next(
        reference: PolyRef,
        neighbor_index: u8,
        edge_index: u8,
        side: u8,
        boundary_flag: u8,
        next: Option<u32>,
    ) -> Self {
        Self {
            reference,
            neighbor_index,
            edge_index,
            side,
            boundary_flag,
            next,
        }
    }
}

/// Polygon in the navigation mesh
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Poly {
    /// First link index
    pub first_link: Option<usize>,
    /// Vertices of the polygon (indices into the navigation mesh vertex array)
    pub verts: [u16; MAX_VERTS_PER_POLY],
    /// Neighboring polygon references for each edge
    pub neighbors: [u16; MAX_VERTS_PER_POLY],
    /// Flags for the polygon
    pub flags: PolyFlags,
    /// Number of vertices in the polygon
    pub vert_count: u8,
    /// Area ID of the polygon
    pub area: u8,
    /// Polygon type
    pub poly_type: PolyType,
}

impl Poly {
    /// Creates a new polygon
    pub fn new(area: u8, poly_type: PolyType, flags: PolyFlags) -> Self {
        Self {
            first_link: None,
            verts: [0; MAX_VERTS_PER_POLY],
            neighbors: [0; MAX_VERTS_PER_POLY],
            flags,
            vert_count: 0,
            area,
            poly_type,
        }
    }
}

/// Mesh tile in the navigation mesh
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct MeshTile {
    /// Salt value for the tile
    pub salt: u32,
    /// Tile location (x, y, layer)
    pub header: Option<TileHeader>,
    /// Polygons in the tile
    pub polys: Vec<Poly>,
    /// Vertices in the tile [x,y,z,...]
    pub verts: Vec<f32>,
    /// Links between polygons
    pub links: Vec<Link>,
    /// Detailed mesh data
    pub detail_meshes: Vec<PolyDetail>,
    /// Detailed mesh vertices [x,y,z,...]
    pub detail_verts: Vec<f32>,
    /// Detailed mesh triangle indices
    pub detail_tris: Vec<u8>,
    /// Bounding volume tree root node
    pub bvh_root: Option<usize>,
    /// Bounding volume tree nodes
    pub bvh_nodes: Vec<BVNode>,
    /// Off-mesh connections
    pub off_mesh_connections: Vec<OffMeshConnection>,
    /// Tile flags
    pub flags: u8,
    /// Next free tile in the linked list (used for memory management)
    pub next: Option<usize>,
}

impl Default for MeshTile {
    fn default() -> Self {
        Self::new()
    }
}

impl MeshTile {
    /// Creates a new empty mesh tile
    pub fn new() -> Self {
        Self {
            salt: 0,
            header: None,
            polys: Vec::new(),
            verts: Vec::new(),
            links: Vec::new(),
            detail_meshes: Vec::new(),
            detail_verts: Vec::new(),
            detail_tris: Vec::new(),
            bvh_root: None,
            bvh_nodes: Vec::new(),
            off_mesh_connections: Vec::new(),
            flags: 0,
            next: None,
        }
    }
}

/// Tile header information
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct TileHeader {
    /// Tile position (x, y, layer)
    pub x: i32,
    pub y: i32,
    pub layer: i32,
    /// User defined data
    pub user_id: u32,
    /// Size of the tile data
    pub data_size: usize,
    /// Bounding box of the tile
    pub bmin: [f32; 3],
    pub bmax: [f32; 3],
    /// Number of polys in the tile
    pub poly_count: i32,
    /// Number of vertices in the tile
    pub vert_count: i32,
    /// Number of links in the tile
    pub max_links: i32,
    /// Number of detail meshes in the tile
    pub detail_mesh_count: i32,
    /// Number of detail vertices in the tile
    pub detail_vert_count: i32,
    /// Number of detail triangles in the tile
    pub detail_tri_count: i32,
    /// Number of BVH nodes in the tile
    pub bvh_node_count: i32,
    /// Number of off-mesh connections in the tile
    pub off_mesh_connection_count: i32,
    /// BVH quantization factor for this tile
    pub bv_quant_factor: f32,
}

impl TileHeader {
    /// Creates a new tile header
    pub fn new(x: i32, y: i32, layer: i32) -> Self {
        Self {
            x,
            y,
            layer,
            user_id: 0,
            data_size: 0,
            bmin: [0.0; 3],
            bmax: [0.0; 3],
            poly_count: 0,
            vert_count: 0,
            max_links: 0,
            detail_mesh_count: 0,
            detail_vert_count: 0,
            detail_tri_count: 0,
            bvh_node_count: 0,
            off_mesh_connection_count: 0,
            bv_quant_factor: 0.0,
        }
    }
}

/// Detailed mesh for a polygon
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct PolyDetail {
    /// Index of the vertex in the detail_verts array
    pub vert_base: u32,
    /// Index of the triangle in the detail_tris array
    pub tri_base: u32,
    /// Number of vertices
    pub vert_count: u8,
    /// Number of triangles
    pub tri_count: u8,
}

impl Default for PolyDetail {
    fn default() -> Self {
        Self::new()
    }
}

impl PolyDetail {
    /// Creates a new polygon detail
    pub fn new() -> Self {
        Self {
            vert_base: 0,
            tri_base: 0,
            vert_count: 0,
            tri_count: 0,
        }
    }
}

/// Bounding volume node using quantized coordinates for memory efficiency
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct BVNode {
    /// Quantized bounding box min coordinates (16-bit)
    pub bmin: [u16; 3],
    /// Quantized bounding box max coordinates (16-bit)
    pub bmax: [u16; 3],
    /// Index of the left and right children
    /// Negative values indicate leaf nodes with indices into the polygon array
    pub i: i32,
}

/// Quantized bounds for efficient comparison
#[derive(Debug, Clone, Copy)]
pub struct QuantBounds {
    pub bmin: [u16; 3],
    pub bmax: [u16; 3],
}

impl Default for BVNode {
    fn default() -> Self {
        Self::new()
    }
}

impl BVNode {
    /// Creates a new bounding volume node
    pub fn new() -> Self {
        Self {
            bmin: [0; 3],
            bmax: [0; 3],
            i: 0,
        }
    }

    /// Quantizes floating point coordinates to 16-bit integers
    pub fn quantize_bounds(
        bmin: &[f32; 3],
        bmax: &[f32; 3],
        tile_bmin: &[f32; 3],
        tile_bmax: &[f32; 3],
    ) -> (QuantBounds, f32) {
        // Calculate quantization factor (16-bit range / tile dimensions)
        let tile_size = [
            tile_bmax[0] - tile_bmin[0],
            tile_bmax[1] - tile_bmin[1],
            tile_bmax[2] - tile_bmin[2],
        ];

        let quant_factor = 1.0 / tile_size.iter().fold(f32::MIN, |a, &b| a.max(b)) * 65535.0;

        let mut qbounds = QuantBounds {
            bmin: [0; 3],
            bmax: [0; 3],
        };

        for i in 0..3 {
            qbounds.bmin[i] = ((bmin[i] - tile_bmin[i]) * quant_factor).clamp(0.0, 65535.0) as u16;
            qbounds.bmax[i] = ((bmax[i] - tile_bmin[i]) * quant_factor).clamp(0.0, 65535.0) as u16;
        }

        (qbounds, quant_factor)
    }

    /// Checks if two quantized bounds overlap
    pub fn overlap_quant_bounds(
        amin: &[u16; 3],
        amax: &[u16; 3],
        bmin: &[u16; 3],
        bmax: &[u16; 3],
    ) -> bool {
        !(amin[0] > bmax[0]
            || amax[0] < bmin[0]
            || amin[1] > bmax[1]
            || amax[1] < bmin[1]
            || amin[2] > bmax[2]
            || amax[2] < bmin[2])
    }
}

/// Off-mesh connection
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct OffMeshConnection {
    /// Connection position (start/end) \[sx,sy,sz,ex,ey,ez\]
    pub pos: [f32; 6],
    /// Connection radius
    pub radius: f32,
    /// Polygon reference
    pub poly: PolyRef,
    /// Flags
    pub flags: PolyFlags,
    /// Area ID
    pub area: u8,
    /// Direction (bidir = 0, A->B = 1, B->A = 2)
    pub dir: u8,
    /// User ID
    pub user_id: u32,
}

impl Default for OffMeshConnection {
    fn default() -> Self {
        Self::new()
    }
}

impl OffMeshConnection {
    /// Creates a new off-mesh connection
    pub fn new() -> Self {
        Self {
            pos: [0.0; 6],
            radius: 0.0,
            poly: PolyRef::new(0),
            flags: PolyFlags::empty(),
            area: 0,
            dir: 0,
            user_id: 0,
        }
    }

    /// Creates a new off-mesh connection with specified parameters
    pub fn new_with_params(
        start_pos: [f32; 3],
        end_pos: [f32; 3],
        radius: f32,
        flags: PolyFlags,
        area: u8,
        dir: u8,
        user_id: u32,
    ) -> Self {
        let mut pos = [0.0; 6];
        pos[0] = start_pos[0];
        pos[1] = start_pos[1];
        pos[2] = start_pos[2];
        pos[3] = end_pos[0];
        pos[4] = end_pos[1];
        pos[5] = end_pos[2];

        Self {
            pos,
            radius,
            poly: PolyRef::new(0),
            flags,
            area,
            dir,
            user_id,
        }
    }

    /// Gets the start position of the connection
    pub fn start_pos(&self) -> [f32; 3] {
        [self.pos[0], self.pos[1], self.pos[2]]
    }

    /// Gets the end position of the connection
    pub fn end_pos(&self) -> [f32; 3] {
        [self.pos[3], self.pos[4], self.pos[5]]
    }

    /// Checks if the connection is bidirectional
    pub fn is_bidirectional(&self) -> bool {
        self.dir == 0
    }

    /// Checks if traversal from start to end is allowed
    pub fn allows_start_to_end(&self) -> bool {
        self.dir == 0 || self.dir == 1
    }

    /// Checks if traversal from end to start is allowed
    pub fn allows_end_to_start(&self) -> bool {
        self.dir == 0 || self.dir == 2
    }

    /// Gets the distance between start and end positions
    pub fn get_length(&self) -> f32 {
        let start = self.start_pos();
        let end = self.end_pos();
        let dx = end[0] - start[0];
        let dy = end[1] - start[1];
        let dz = end[2] - start[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Navigation mesh structure
#[derive(Debug)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct NavMesh {
    /// Navigation mesh parameters
    params: NavMeshParams,
    /// Origin of the navigation mesh
    origin: [f32; 3],
    /// Tile width and height
    tile_width: f32,
    tile_height: f32,
    /// Maximum number of tiles
    max_tiles: i32,
    /// Maximum number of polygons per tile
    max_polys_per_tile: i32,
    /// Tile to world scale
    #[allow(dead_code)]
    tile_to_world_scale: f32,
    /// Tiles in the navigation mesh
    tiles: Vec<Option<MeshTile>>,
    /// Next available tile ID
    next_free: Option<usize>,
    /// Tile hash lookup
    #[cfg_attr(
        feature = "serialization",
        serde(
            serialize_with = "serialize_pos_lookup",
            deserialize_with = "deserialize_pos_lookup"
        )
    )]
    pos_lookup: HashMap<(i32, i32, i32), usize>,
    /// Navigation mesh flags
    flags: NavMeshFlags,
    /// Salt for generating unique references
    salt: u32,
    /// Global BVH tree for all polygons (not serialized)
    #[cfg_attr(feature = "serialization", serde(skip))]
    global_bvh: Option<BVHTree>,
}

impl NavMesh {
    /// Creates a new navigation mesh
    pub fn new(params: NavMeshParams) -> Result<Self> {
        // Validate parameters
        if params.origin[0].is_infinite()
            || params.origin[0].is_nan()
            || params.origin[1].is_infinite()
            || params.origin[1].is_nan()
            || params.origin[2].is_infinite()
            || params.origin[2].is_nan()
        {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.tile_width <= 0.0 || params.tile_height <= 0.0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.max_tiles <= 0 || params.max_polys_per_tile <= 0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.max_tiles >= (1 << DT_TILE_BITS) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.max_polys_per_tile >= (1 << DT_POLY_BITS) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Allocate tiles
        let mut tiles = Vec::with_capacity(params.max_tiles as usize);
        for i in 0..params.max_tiles as usize {
            let mut tile = MeshTile::new();
            tile.salt = 1;
            tile.next = Some(i + 1);
            tiles.push(Some(tile));
        }

        // Set last tile's next to None
        if let Some(Some(tile)) = tiles.last_mut() {
            tile.next = None;
        }

        // Tile hash lookup
        let pos_lookup = HashMap::new();

        Ok(Self {
            origin: params.origin,
            tile_width: params.tile_width,
            tile_height: params.tile_height,
            max_tiles: params.max_tiles,
            max_polys_per_tile: params.max_polys_per_tile,
            params,
            tile_to_world_scale: 1.0, // Will be calculated during initialization
            tiles,
            next_free: Some(0),
            pos_lookup,
            flags: NavMeshFlags::empty(),
            salt: 1,
            global_bvh: None,
        })
    }

    /// Initializes the navigation mesh with tiles
    pub fn init(&mut self, nav_data: &[u8]) -> Result<()> {
        use super::binary_format::{DT_NAVMESH_MAGIC, DT_NAVMESH_VERSION, load_tile_from_binary};
        use byteorder::{NativeEndian, ReadBytesExt};
        use std::io::Cursor;

        if nav_data.len() < 4 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let mut cursor = Cursor::new(nav_data);

        // Check if this is a multi-tile format by looking for magic number
        let magic = cursor.read_u32::<NativeEndian>()?;
        cursor.set_position(0);

        if magic == DT_NAVMESH_MAGIC {
            // Multi-tile format with header
            cursor.set_position(4); // Skip magic we already read
            let version = cursor.read_u32::<NativeEndian>()?;
            if version != DT_NAVMESH_VERSION {
                return Err(Error::Detour(Status::WrongVersion.to_string()));
            }

            let tile_count = cursor.read_i32::<NativeEndian>()?;

            // Read NavMeshParams
            let mut origin = [0.0; 3];
            for v in &mut origin {
                *v = cursor.read_f32::<NativeEndian>()?;
            }
            let tile_width = cursor.read_f32::<NativeEndian>()?;
            let tile_height = cursor.read_f32::<NativeEndian>()?;
            let max_tiles = cursor.read_i32::<NativeEndian>()?;
            let max_polys_per_tile = cursor.read_i32::<NativeEndian>()?;

            // Re-initialize with new parameters
            self.params = NavMeshParams {
                origin,
                tile_width,
                tile_height,
                max_tiles,
                max_polys_per_tile,
            };

            // Clear existing tiles and resize
            self.tiles.clear();
            self.tiles.resize(max_tiles as usize, None);
            self.pos_lookup.clear();
            self.next_free = None;
            // Reset free list to all tiles
            for i in 0..max_tiles as usize {
                self.tiles[i] = Some(MeshTile {
                    salt: 1,
                    next: if i < max_tiles as usize - 1 {
                        Some(i + 1)
                    } else {
                        None
                    },
                    ..MeshTile::default()
                });
            }
            self.next_free = Some(0);

            // Load each tile
            for _ in 0..tile_count {
                let tile_size = cursor.read_u32::<NativeEndian>()? as usize;
                let pos = cursor.position() as usize;

                if pos + tile_size > nav_data.len() {
                    return Err(Error::Detour(Status::InvalidParam.to_string()));
                }

                let tile_data = &nav_data[pos..pos + tile_size];
                let tile = load_tile_from_binary(tile_data)?;
                self.add_mesh_tile(tile)?;

                cursor.set_position((pos + tile_size) as u64);
            }
        } else {
            // Single tile format - parse as C++ does
            let tile = load_tile_from_binary(nav_data)?;

            // Extract parameters from tile header
            let header = tile
                .header
                .as_ref()
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

            // Re-initialize with parameters from tile
            self.params = NavMeshParams {
                origin: header.bmin,
                tile_width: header.bmax[0] - header.bmin[0],
                tile_height: header.bmax[2] - header.bmin[2],
                max_tiles: 1,
                max_polys_per_tile: header.poly_count,
            };

            // Clear existing tiles
            self.tiles.clear();
            self.tiles.resize(1, None);
            self.pos_lookup.clear();
            self.next_free = Some(0);
            // Initialize single tile slot
            self.tiles[0] = Some(MeshTile {
                salt: 1,
                next: None,
                ..MeshTile::default()
            });

            // Add the tile
            self.add_mesh_tile(tile)?;
        }

        Ok(())
    }

    /// Adds a tile to the navigation mesh from raw data
    pub fn add_tile(
        &mut self,
        data: &[u8],
        flags: u8,
        _last_ref: Option<PolyRef>,
    ) -> Result<PolyRef> {
        // Parse tile data from binary format
        let mut tile = super::binary_format::load_tile_from_binary(data)?;

        // Set tile flags
        tile.flags = flags;

        // Add the tile to the navigation mesh
        self.add_mesh_tile(tile)
    }

    /// Adds a MeshTile directly to the navigation mesh
    pub fn add_mesh_tile(&mut self, mut tile: MeshTile) -> Result<PolyRef> {
        // Extract tile coordinates from header
        let header = tile
            .header
            .as_ref()
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        let tile_x = header.x;
        let tile_y = header.y;
        let tile_layer = header.layer;

        // Check if we have a free tile slot
        let tile_idx = self.allocate_tile()?;

        // Update the next free pointer
        if let Some(Some(existing_tile)) = self.tiles.get(tile_idx) {
            self.next_free = existing_tile.next;
        }

        // Use the salt from the free tile slot (set during remove_tile)
        // The salt was already set when the tile was added to the free list
        let tile_salt = if let Some(Some(existing_tile)) = self.tiles.get(tile_idx) {
            existing_tile.salt
        } else {
            self.salt
        };
        tile.salt = tile_salt;

        // Clear the tile's next pointer since it's now in use
        tile.next = None;

        // Set tile flags
        tile.flags = 1; // Default to walkable

        // Add tile to position lookup
        self.pos_lookup
            .insert((tile_x, tile_y, tile_layer), tile_idx);

        // Store the tile
        self.tiles[tile_idx] = Some(tile);

        // Build BVH tree for spatial queries (after tile is stored)
        self.build_tile_bvh(tile_idx)?;

        // Generate polygon reference for the first polygon in the tile
        // Add 1 to tile_idx to avoid creating PolyRef(0) which is invalid
        // Include the salt in the reference
        let poly_ref =
            encode_poly_ref_with_salt(tile_salt & DT_SALT_MASK, (tile_idx + 1) as u32, 0);

        // Update internal link references to use the correct tile ID
        self.update_internal_link_references(tile_idx)?;

        // Connect tiles (build links between adjacent tiles)
        self.connect_tile(tile_idx)?;

        Ok(poly_ref)
    }

    /// Adds a tile to the navigation mesh from create parameters
    pub fn add_tile_from_params(
        &mut self,
        params: &super::NavMeshCreateParams,
        tile_x: i32,
        tile_y: i32,
        tile_layer: i32,
    ) -> Result<PolyRef> {
        // Check if we have a free tile slot
        let tile_idx = match self.next_free {
            Some(idx) => idx,
            None => return Err(Error::Detour(Status::OutOfMemory.to_string())),
        };

        // Get next free info and update it
        let next_free = {
            let tile = match self.tiles.get_mut(tile_idx) {
                Some(Some(tile)) => tile,
                _ => return Err(Error::Detour(Status::Failure.to_string())),
            };
            tile.next
        };
        self.next_free = next_free;

        // Create tile header
        let mut header = TileHeader::new(tile_x, tile_y, tile_layer);
        header.bmin = params.bmin;
        header.bmax = params.bmax;
        header.poly_count = params.poly_count;
        header.vert_count = params.vert_count;
        header.detail_mesh_count = params.poly_count;
        header.detail_vert_count = params.detail_vert_count;
        header.detail_tri_count = params.detail_tri_count;
        header.max_links = params.poly_count * 6; // Assuming max 6 links per polygon

        // Prepare tile data
        let verts = params.verts.clone();

        // Convert polygons
        let mut polys = Vec::new();
        let mut poly_idx = 0;
        for i in 0..params.poly_count as usize {
            // Get polygon properties
            let area = params.poly_areas.get(i).copied().unwrap_or(0);
            let flags = params.poly_flags.get(i).copied().unwrap_or(PolyFlags::WALK);
            let mut poly = Poly::new(area, PolyType::Ground, flags);

            // Count vertices for this polygon
            let mut vert_count = 0;
            for j in 0..params.nvp as usize {
                let idx = poly_idx + j;
                if idx < params.polys.len() {
                    let vert_idx = params.polys[idx];
                    if vert_idx != MESH_NULL_IDX {
                        poly.verts[vert_count] = vert_idx;
                        vert_count += 1;
                    }
                }
            }

            // Read polygon neighbors
            for j in 0..params.nvp as usize {
                let idx = poly_idx + params.nvp as usize + j;
                if idx < params.polys.len() {
                    poly.neighbors[j] = params.polys[idx];
                } else {
                    poly.neighbors[j] = MESH_NULL_IDX;
                }
            }

            poly_idx += params.nvp as usize * 2;

            // Set polygon vertex count
            poly.vert_count = vert_count as u8;

            polys.push(poly);
        }

        // Copy detail meshes
        let mut detail_meshes = Vec::new();
        for i in 0..params.poly_count as usize {
            let base_idx = i * 4;
            if base_idx + 3 < params.detail_meshes.len() {
                detail_meshes.push(PolyDetail {
                    vert_base: params.detail_meshes[base_idx],
                    tri_base: params.detail_meshes[base_idx + 1],
                    vert_count: params.detail_meshes[base_idx + 2] as u8,
                    tri_count: params.detail_meshes[base_idx + 3] as u8,
                });
            }
        }

        // Copy detail vertices and triangles
        let detail_verts = params.detail_verts.clone();
        let detail_tris = params.detail_tris.clone();

        // Handle off-mesh connections
        let off_mesh_vert_base = params.vert_count as usize;
        let off_mesh_poly_base = params.poly_count as usize;

        // Add off-mesh connection vertices to the vertex array
        let mut all_verts = verts.clone();
        for i in 0..params.off_mesh_con_count as usize {
            let v_idx = i * 6;
            if v_idx + 5 < params.off_mesh_con_verts.len() {
                // Add start vertex
                all_verts.push(params.off_mesh_con_verts[v_idx]);
                all_verts.push(params.off_mesh_con_verts[v_idx + 1]);
                all_verts.push(params.off_mesh_con_verts[v_idx + 2]);
                // Add end vertex
                all_verts.push(params.off_mesh_con_verts[v_idx + 3]);
                all_verts.push(params.off_mesh_con_verts[v_idx + 4]);
                all_verts.push(params.off_mesh_con_verts[v_idx + 5]);
            }
        }

        // Create off-mesh connection polygons
        for i in 0..params.off_mesh_con_count as usize {
            let area = params.off_mesh_con_areas.get(i).copied().unwrap_or(0);
            let flags = params
                .off_mesh_con_flags
                .get(i)
                .copied()
                .unwrap_or(PolyFlags::WALK);
            let mut poly = Poly::new(area, PolyType::OffMeshConnection, flags);

            // Off-mesh connections have exactly 2 vertices
            poly.vert_count = 2;
            poly.verts[0] = (off_mesh_vert_base + i * 2) as u16;
            poly.verts[1] = (off_mesh_vert_base + i * 2 + 1) as u16;

            polys.push(poly);
        }

        // Create off-mesh connection structs
        let mut off_mesh_connections = Vec::new();
        for i in 0..params.off_mesh_con_count as usize {
            let v_idx = i * 6;
            if v_idx + 5 < params.off_mesh_con_verts.len() {
                let mut con = OffMeshConnection::new();

                // Copy vertex positions
                con.pos[0] = params.off_mesh_con_verts[v_idx];
                con.pos[1] = params.off_mesh_con_verts[v_idx + 1];
                con.pos[2] = params.off_mesh_con_verts[v_idx + 2];
                con.pos[3] = params.off_mesh_con_verts[v_idx + 3];
                con.pos[4] = params.off_mesh_con_verts[v_idx + 4];
                con.pos[5] = params.off_mesh_con_verts[v_idx + 5];

                con.radius = params.off_mesh_con_rad.get(i).copied().unwrap_or(0.0);
                con.flags = params
                    .off_mesh_con_flags
                    .get(i)
                    .copied()
                    .unwrap_or(PolyFlags::WALK);
                con.area = params.off_mesh_con_areas.get(i).copied().unwrap_or(0);
                con.dir = params.off_mesh_con_dir.get(i).copied().unwrap_or(0);
                con.user_id = params.off_mesh_con_user_id.get(i).copied().unwrap_or(0);

                // Polygon reference will be set after tile is finalized
                con.poly = PolyRef::new(0);

                off_mesh_connections.push(con);
            }
        }

        // Update header counts
        header.poly_count = params.poly_count + params.off_mesh_con_count;
        header.vert_count = params.vert_count + params.off_mesh_con_count * 2;
        header.off_mesh_connection_count = params.off_mesh_con_count;

        // Build BVH tree if requested
        if params.build_bv_tree {
            self.build_tile_bvh(tile_idx)?;
        }

        // Now update the tile with all the data
        let tile_salt = {
            let tile = match self.tiles.get_mut(tile_idx) {
                Some(Some(tile)) => tile,
                _ => return Err(Error::Detour(Status::Failure.to_string())),
            };

            tile.header = Some(header);
            tile.salt += 1;
            tile.flags = 0;
            tile.verts = all_verts;
            tile.polys = polys;
            tile.detail_meshes = detail_meshes;
            tile.detail_verts = detail_verts;
            tile.detail_tris = detail_tris;
            tile.off_mesh_connections = off_mesh_connections;

            // Build internal links between polygons
            crate::nav_mesh_builder::NavMeshBuilder::build_internal_links_for_tile(tile)?;

            // Return the salt for use in encoding
            tile.salt
        };

        // Add to position lookup
        self.pos_lookup
            .insert((tile_x, tile_y, tile_layer), tile_idx);

        // Create polygon reference for the first polygon in the tile
        // Add 1 to tile_idx to avoid creating PolyRef(0) which is invalid
        // Include the salt in the reference
        let actual_tile_id = (tile_idx + 1) as u32;
        let poly_ref = encode_poly_ref_with_salt(tile_salt & DT_SALT_MASK, actual_tile_id, 0);

        // Update internal link references to use the correct tile ID
        self.update_internal_link_references(tile_idx)?;

        // Update off-mesh connection polygon references
        if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
            for (i, con) in tile.off_mesh_connections.iter_mut().enumerate() {
                let poly_idx = off_mesh_poly_base + i;
                con.poly = encode_poly_ref_with_salt(
                    tile_salt & DT_SALT_MASK,
                    actual_tile_id,
                    poly_idx as u32,
                );
            }
        }

        // Create links between ground polygons and off-mesh connections
        self.connect_off_mesh_connections_for_tile(tile_idx, actual_tile_id, params)?;

        // Collect and apply external links to adjacent tiles
        let link_requests = crate::nav_mesh_builder::NavMeshBuilder::connect_external_links(
            self, tile_x, tile_y, tile_layer,
        )?;
        self.apply_external_links(link_requests)?;

        Ok(poly_ref)
    }

    /// Builds a BVH tree for a tile
    fn build_tile_bvh(&mut self, tile_idx: usize) -> Result<()> {
        // Get mutable reference to the tile
        let tile = self
            .tiles
            .get_mut(tile_idx)
            .and_then(|opt| opt.as_mut())
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Clear existing BVH nodes
        tile.bvh_nodes.clear();

        // Get number of polygons
        let poly_count = tile.polys.len();
        if poly_count == 0 {
            return Ok(());
        }

        // Calculate bounding boxes for all polygons
        let mut poly_bounds: Vec<([f32; 3], [f32; 3])> = Vec::with_capacity(poly_count);

        for poly in &tile.polys {
            let mut bmin = [f32::MAX; 3];
            let mut bmax = [f32::MIN; 3];

            // Calculate polygon bounds
            for j in 0..poly.vert_count as usize {
                let v_idx = poly.verts[j] as usize * 3;
                if v_idx + 2 < tile.verts.len() {
                    for k in 0..3 {
                        bmin[k] = bmin[k].min(tile.verts[v_idx + k]);
                        bmax[k] = bmax[k].max(tile.verts[v_idx + k]);
                    }
                }
            }

            poly_bounds.push((bmin, bmax));
        }

        // Build the BVH tree recursively
        let mut items: Vec<usize> = (0..poly_count).collect();
        Self::build_bvh_recursive_for_tile(tile, &poly_bounds, &mut items, 0, poly_count)?;

        // Update tile header with BVH node count
        if let Some(header) = tile.header.as_mut() {
            header.bvh_node_count = tile.bvh_nodes.len() as i32;
        }

        Ok(())
    }

    /// Recursively builds a BVH tree for the given range of polygons
    fn build_bvh_recursive_for_tile(
        tile: &mut MeshTile,
        poly_bounds: &[([f32; 3], [f32; 3])],
        items: &mut [usize],
        item_start: usize,
        item_count: usize,
    ) -> Result<i32> {
        if item_count == 0 {
            return Ok(-1);
        }

        let node_idx = tile.bvh_nodes.len() as i32;

        // Calculate bounds for all items in this node
        let mut node_bmin = [f32::MAX; 3];
        let mut node_bmax = [f32::MIN; 3];

        for i in 0..item_count {
            let poly_idx = items[item_start + i];
            let (bmin, bmax) = poly_bounds[poly_idx];
            for k in 0..3 {
                node_bmin[k] = node_bmin[k].min(bmin[k]);
                node_bmax[k] = node_bmax[k].max(bmax[k]);
            }
        }

        // Get tile header for quantization
        let header = tile
            .header
            .as_ref()
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Quantize bounds
        let (quant_bounds, _) =
            BVNode::quantize_bounds(&node_bmin, &node_bmax, &header.bmin, &header.bmax);

        // Create node
        let mut node = BVNode {
            bmin: quant_bounds.bmin,
            bmax: quant_bounds.bmax,
            i: 0,
        };

        // If we have only one or two items, create a leaf node
        if item_count <= 2 {
            // Leaf node: encode polygon indices
            // For one item: i = -(poly_idx + 1)
            // For two items: i = (poly_idx0 << 16) | poly_idx1
            if item_count == 1 {
                node.i = -(items[item_start] as i32 + 1);
            } else {
                let idx0 = items[item_start] as i32;
                let idx1 = items[item_start + 1] as i32;
                node.i = (idx0 << 16) | (idx1 & 0xFFFF);
            }
            tile.bvh_nodes.push(node);
        } else {
            // Internal node: subdivide
            tile.bvh_nodes.push(node); // Reserve space

            // Find the best axis to split along
            let mut axis = 0;
            let mut max_span = node_bmax[0] - node_bmin[0];
            for k in 1..3 {
                let span = node_bmax[k] - node_bmin[k];
                if span > max_span {
                    axis = k;
                    max_span = span;
                }
            }

            // Sort items along the chosen axis
            let items_slice = &mut items[item_start..item_start + item_count];
            items_slice.sort_by(|&a, &b| {
                let a_center = (poly_bounds[a].0[axis] + poly_bounds[a].1[axis]) * 0.5;
                let b_center = (poly_bounds[b].0[axis] + poly_bounds[b].1[axis]) * 0.5;
                a_center
                    .partial_cmp(&b_center)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            // Split items in half
            let split = item_count / 2;

            // Build children
            let left_child =
                Self::build_bvh_recursive_for_tile(tile, poly_bounds, items, item_start, split)?;
            let right_child = Self::build_bvh_recursive_for_tile(
                tile,
                poly_bounds,
                items,
                item_start + split,
                item_count - split,
            )?;

            // Update node with children indices
            // Encode as: (left_child << 16) | (right_child & 0xFFFF)
            tile.bvh_nodes[node_idx as usize].i =
                ((left_child & 0xFFFF) << 16) | (right_child & 0xFFFF);
        }

        Ok(node_idx)
    }

    /// Gets the tile index from tile coordinates
    pub fn get_tile_index(&self, tile_x: i32, tile_y: i32, tile_layer: i32) -> Option<usize> {
        self.pos_lookup.get(&(tile_x, tile_y, tile_layer)).copied()
    }

    /// Connects a tile to its neighboring tiles by building links
    pub fn connect_tile(&mut self, tile_idx: usize) -> Result<()> {
        // Get tile header info for the tile we're connecting
        let (tile_x, tile_y, tile_layer) = {
            let tile = self
                .tiles
                .get(tile_idx)
                .and_then(|t| t.as_ref())
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

            let header = tile
                .header
                .as_ref()
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

            (header.x, header.y, header.layer)
        };

        // Connect to adjacent tiles (4-connected: N, S, E, W)
        let adjacent_coords = [
            (tile_x, tile_y + 1, tile_layer), // North
            (tile_x, tile_y - 1, tile_layer), // South
            (tile_x + 1, tile_y, tile_layer), // East
            (tile_x - 1, tile_y, tile_layer), // West
        ];

        for (adj_x, adj_y, adj_layer) in adjacent_coords {
            if let Some(&adj_tile_idx) = self.pos_lookup.get(&(adj_x, adj_y, adj_layer)) {
                // Connect the current tile to the adjacent tile
                self.connect_two_tiles(tile_idx, adj_tile_idx)?;
                // And the adjacent tile back to the current tile
                self.connect_two_tiles(adj_tile_idx, tile_idx)?;
            }
        }

        Ok(())
    }

    /// Updates internal link references to use the correct tile ID and salt
    fn update_internal_link_references(&mut self, tile_idx: usize) -> Result<()> {
        let tile_salt = if let Some(Some(tile)) = self.tiles.get(tile_idx) {
            tile.salt
        } else {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        };

        // Update all internal link references in this tile
        if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
            for link in &mut tile.links {
                // Check if this is an internal link (boundary_flag == 0)
                if link.boundary_flag == 0 {
                    let (_, poly_id) = decode_poly_ref(link.reference);
                    // Update the reference to use the correct tile ID and salt
                    link.reference = encode_poly_ref_with_salt(
                        tile_salt & DT_SALT_MASK,
                        (tile_idx + 1) as u32,
                        poly_id,
                    );
                }
            }
        }

        Ok(())
    }

    /// Connects two specific tiles by building links between their boundary polygons
    fn connect_two_tiles(&mut self, tile_a_idx: usize, tile_b_idx: usize) -> Result<()> {
        // Get immutable references to both tiles' data first
        let (tile_a_bounds, _tile_b_bounds, _tile_width, _tile_height, tile_a_coord, tile_b_coord) = {
            let tiles = &self.tiles;

            let tile_a = tiles
                .get(tile_a_idx)
                .and_then(|t| t.as_ref())
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;
            let tile_b = tiles
                .get(tile_b_idx)
                .and_then(|t| t.as_ref())
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

            let header_a = tile_a
                .header
                .as_ref()
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;
            let header_b = tile_b
                .header
                .as_ref()
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

            // Determine which edge they share
            let dx = header_b.x - header_a.x;
            let dy = header_b.y - header_a.y;

            // Only connect directly adjacent tiles
            if dx.abs() + dy.abs() != 1 || header_a.layer != header_b.layer {
                return Ok(());
            }

            (
                (header_a.bmin, header_a.bmax),
                (header_b.bmin, header_b.bmax),
                self.params.tile_width,
                self.params.tile_height,
                (header_a.x, header_a.y),
                (header_b.x, header_b.y),
            )
        };

        // Determine the shared edge direction
        let (axis, direction) = {
            let dx = tile_b_coord.0 - tile_a_coord.0;
            let dy = tile_b_coord.1 - tile_a_coord.1;

            if dx != 0 {
                // Tiles are aligned along X axis
                (0, dx)
            } else {
                // Tiles are aligned along Z axis
                (2, dy)
            }
        };

        // Find candidate polygons on the boundary
        let epsilon = 0.001f32;
        let mut connections: Vec<(usize, usize, u8, u8)> = Vec::new();

        // Get polygon data for both tiles
        {
            let tiles = &self.tiles;
            let tile_a = tiles[tile_a_idx].as_ref().unwrap();
            let tile_b = tiles[tile_b_idx].as_ref().unwrap();

            // Check each polygon in tile A
            for (poly_a_idx, poly_a) in tile_a.polys.iter().enumerate() {
                // Find edges on the boundary
                for edge_a in 0..poly_a.vert_count as usize {
                    let va0 = poly_a.verts[edge_a] as usize * 3;
                    let va1 = poly_a.verts[(edge_a + 1) % poly_a.vert_count as usize] as usize * 3;

                    if va0 + 2 >= tile_a.verts.len() || va1 + 2 >= tile_a.verts.len() {
                        continue;
                    }

                    let v0 = [
                        tile_a.verts[va0],
                        tile_a.verts[va0 + 1],
                        tile_a.verts[va0 + 2],
                    ];
                    let v1 = [
                        tile_a.verts[va1],
                        tile_a.verts[va1 + 1],
                        tile_a.verts[va1 + 2],
                    ];

                    // Check if edge is on the boundary
                    let boundary_pos = if direction > 0 {
                        tile_a_bounds.1[axis]
                    } else {
                        tile_a_bounds.0[axis]
                    };

                    let on_boundary = (v0[axis] - boundary_pos).abs() < epsilon
                        && (v1[axis] - boundary_pos).abs() < epsilon;

                    if !on_boundary {
                        continue;
                    }

                    // Now check polygons in tile B for matching edges
                    for (poly_b_idx, poly_b) in tile_b.polys.iter().enumerate() {
                        for edge_b in 0..poly_b.vert_count as usize {
                            let vb0 = poly_b.verts[edge_b] as usize * 3;
                            let vb1 = poly_b.verts[(edge_b + 1) % poly_b.vert_count as usize]
                                as usize
                                * 3;

                            if vb0 + 2 >= tile_b.verts.len() || vb1 + 2 >= tile_b.verts.len() {
                                continue;
                            }

                            let vb0_pos = [
                                tile_b.verts[vb0],
                                tile_b.verts[vb0 + 1],
                                tile_b.verts[vb0 + 2],
                            ];
                            let vb1_pos = [
                                tile_b.verts[vb1],
                                tile_b.verts[vb1 + 1],
                                tile_b.verts[vb1 + 2],
                            ];

                            // Check if edges overlap
                            let other_axis = if axis == 0 { 2 } else { 0 };

                            // Project edges onto the shared axis
                            let a_min = v0[other_axis].min(v1[other_axis]);
                            let a_max = v0[other_axis].max(v1[other_axis]);
                            let b_min = vb0_pos[other_axis].min(vb1_pos[other_axis]);
                            let b_max = vb0_pos[other_axis].max(vb1_pos[other_axis]);

                            // Check for overlap
                            let overlap = a_min < b_max - epsilon && b_min < a_max - epsilon;

                            // Check if heights are compatible
                            let height_compatible = (v0[1] - vb0_pos[1]).abs() < 2.0
                                || (v0[1] - vb1_pos[1]).abs() < 2.0
                                || (v1[1] - vb0_pos[1]).abs() < 2.0
                                || (v1[1] - vb1_pos[1]).abs() < 2.0;

                            if overlap && height_compatible {
                                connections.push((
                                    poly_a_idx,
                                    poly_b_idx,
                                    edge_a as u8,
                                    edge_b as u8,
                                ));
                            }
                        }
                    }
                }
            }
        }

        // Get salt values for both tiles
        let tile_a_salt = self
            .tiles
            .get(tile_a_idx)
            .and_then(|opt| opt.as_ref())
            .map(|t| t.salt)
            .unwrap_or(0);
        let tile_b_salt = self
            .tiles
            .get(tile_b_idx)
            .and_then(|opt| opt.as_ref())
            .map(|t| t.salt)
            .unwrap_or(0);

        // Now create the actual links
        for (poly_a_idx, poly_b_idx, edge_a, edge_b) in connections {
            // Create link from tile A to tile B
            let poly_ref_b = encode_poly_ref_with_salt(
                tile_b_salt & DT_SALT_MASK,
                (tile_b_idx + 1) as u32,
                poly_b_idx as u32,
            );
            self.add_link_to_polygon(tile_a_idx, poly_a_idx, poly_ref_b, edge_a, 0, 1)?;

            // Create link from tile B to tile A
            let poly_ref_a = encode_poly_ref_with_salt(
                tile_a_salt & DT_SALT_MASK,
                (tile_a_idx + 1) as u32,
                poly_a_idx as u32,
            );
            self.add_link_to_polygon(tile_b_idx, poly_b_idx, poly_ref_a, edge_b, 0, 1)?;
        }

        Ok(())
    }

    /// Adds a link to a polygon in a tile
    fn add_link_to_polygon(
        &mut self,
        tile_idx: usize,
        poly_idx: usize,
        neighbor_ref: PolyRef,
        edge: u8,
        side: u8,
        boundary_flag: u8,
    ) -> Result<()> {
        let tile = self
            .tiles
            .get_mut(tile_idx)
            .and_then(|t| t.as_mut())
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        if poly_idx >= tile.polys.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Allocate a new link
        let link_idx = tile.links.len();
        let link = Link::new(neighbor_ref, poly_idx as u8, edge, side, boundary_flag);
        tile.links.push(link);

        // Update the polygon's link list
        let poly = &mut tile.polys[poly_idx];
        if let Some(first_link) = poly.first_link {
            // Add to the beginning of the link list
            tile.links[link_idx].next = Some(first_link as u32);
        }
        poly.first_link = Some(link_idx);

        Ok(())
    }

    /// Removes a tile from the navigation mesh
    pub fn remove_tile(&mut self, reference: PolyRef) -> Result<()> {
        // Extract tile ID from the reference
        let tile_id = decode_tile_id(reference);

        // Convert to tile index
        let tile_idx =
            tile_id_to_index(tile_id).ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Check if the tile index is valid
        if tile_idx >= self.max_tiles as usize {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }
        let tile = self
            .tiles
            .get_mut(tile_idx)
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        if tile.is_none() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Remove the tile
        let old_tile = tile.take().unwrap();

        // Remove tile from the hash lookup
        if let Some(header) = &old_tile.header {
            self.pos_lookup.remove(&(header.x, header.y, header.layer));
        }

        // Create a new tile for the free list
        let mut new_tile = MeshTile::new();
        new_tile.salt = self.salt;
        self.salt += 1;

        // Add to free list
        new_tile.next = self.next_free;
        self.next_free = Some(tile_idx);

        // Re-insert the tile
        self.tiles[tile_idx] = Some(new_tile);

        Ok(())
    }

    /// Allocates a tile from the free list
    fn allocate_tile(&mut self) -> Result<usize> {
        if let Some(tile_idx) = self.next_free {
            // Get the next free tile
            if let Some(Some(tile)) = self.tiles.get(tile_idx) {
                self.next_free = tile.next;
            }
            Ok(tile_idx)
        } else {
            Err(Error::Detour(Status::Failure.to_string()))
        }
    }

    /// Sets a tile at a specific index in the navigation mesh
    pub fn set_tile_at_index(&mut self, tile_idx: usize, mut tile: MeshTile) -> Result<()> {
        // Validate index
        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Extract tile coordinates from header
        let header = tile
            .header
            .as_ref()
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        let tile_x = header.x;
        let tile_y = header.y;
        let tile_layer = header.layer;

        // Remove existing tile at this index if present
        if let Some(Some(existing_tile)) = self.tiles.get(tile_idx) {
            // Remove from position lookup
            if let Some(existing_header) = &existing_tile.header {
                self.pos_lookup.remove(&(
                    existing_header.x,
                    existing_header.y,
                    existing_header.layer,
                ));
            }
        }

        // Set the salt for the new tile
        if let Some(Some(existing_tile)) = self.tiles.get(tile_idx) {
            tile.salt = existing_tile.salt;
        } else {
            tile.salt = self.salt;
        }

        // Clear the tile's next pointer since it's now in use
        tile.next = None;

        // Set default tile flags if not set
        if tile.flags == 0 {
            tile.flags = 1; // Default to walkable
        }

        // Add tile to position lookup
        self.pos_lookup
            .insert((tile_x, tile_y, tile_layer), tile_idx);

        // Store the tile
        self.tiles[tile_idx] = Some(tile);

        // Build BVH tree for spatial queries
        self.build_tile_bvh(tile_idx)?;

        // Connect tiles (build links between adjacent tiles)
        self.connect_tile(tile_idx)?;

        Ok(())
    }

    /// Gets all tiles that overlap the given bounds
    pub fn get_tiles_in_bounds(&self, bmin: &[f32; 3], bmax: &[f32; 3]) -> Result<Vec<&MeshTile>> {
        let mut result = Vec::new();

        // Calculate tile coordinates for the bounds
        let tw = self.params.tile_width;
        let th = self.params.tile_height;

        let min_x = ((bmin[0] - self.params.origin[0]) / tw).floor() as i32;
        let max_x = ((bmax[0] - self.params.origin[0]) / tw).floor() as i32;
        let min_y = ((bmin[2] - self.params.origin[2]) / th).floor() as i32;
        let max_y = ((bmax[2] - self.params.origin[2]) / th).floor() as i32;

        // Iterate through all tiles and check if they overlap
        for tile in self.tiles.iter().flatten() {
            if let Some(header) = &tile.header {
                if header.x >= min_x && header.x <= max_x && header.y >= min_y && header.y <= max_y
                {
                    result.push(tile);
                }
            }
        }

        Ok(result)
    }

    /// Gets the base polygon reference for a tile
    pub fn get_poly_ref_base(&self, tile: &MeshTile) -> u32 {
        // Find the tile index by comparing pointers
        let tile_idx = self
            .tiles
            .iter()
            .position(|t| t.as_ref().map(|mt| std::ptr::eq(mt, tile)).unwrap_or(false));

        if let Some(idx) = tile_idx {
            // Encode the tile reference with salt
            let tile_id = (idx + 1) as u32; // tile IDs are 1-based
            encode_poly_ref_with_salt(tile.salt & DT_SALT_MASK, tile_id, 0).id()
        } else {
            0
        }
    }

    /// Gets the bounding box of a polygon
    pub fn get_poly_bounds(&self, tile: &MeshTile, poly: &Poly) -> Result<([f32; 3], [f32; 3])> {
        let mut bmin = [f32::MAX; 3];
        let mut bmax = [f32::MIN; 3];

        for i in 0..poly.vert_count as usize {
            let vert_idx = poly.verts[i] as usize;
            if vert_idx * 3 + 2 >= tile.verts.len() {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }

            let x = tile.verts[vert_idx * 3];
            let y = tile.verts[vert_idx * 3 + 1];
            let z = tile.verts[vert_idx * 3 + 2];

            bmin[0] = bmin[0].min(x);
            bmin[1] = bmin[1].min(y);
            bmin[2] = bmin[2].min(z);

            bmax[0] = bmax[0].max(x);
            bmax[1] = bmax[1].max(y);
            bmax[2] = bmax[2].max(z);
        }

        Ok((bmin, bmax))
    }

    /// Finds the closest point on a polygon to the given position
    pub fn closest_point_on_poly(
        &self,
        reference: PolyRef,
        pos: &[f32; 3],
    ) -> Result<([f32; 3], bool)> {
        // Extract tile and poly IDs from the reference
        let tile_id = decode_tile_id(reference);
        let poly_id = decode_poly_id(reference);

        // Convert to tile index
        let tile_idx =
            tile_id_to_index(tile_id).ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Check if the tile index is valid
        if tile_idx >= self.max_tiles as usize {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Get the tile
        let tile = match &self.tiles.get(tile_idx) {
            Some(Some(tile)) => tile,
            _ => {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }
        };

        // Check if the poly ID is valid
        if poly_id >= tile.polys.len() as u32 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Get the polygon
        let poly = &tile.polys[poly_id as usize];

        // Handle off-mesh connections separately
        if poly.poly_type == PolyType::OffMeshConnection {
            // For off-mesh connections, return the position as-is
            return Ok((*pos, false));
        }

        // Get polygon vertices
        let mut verts = Vec::with_capacity(poly.vert_count as usize * 3);
        for i in 0..poly.vert_count as usize {
            let vert_idx = poly.verts[i] as usize;
            if vert_idx * 3 + 2 >= tile.verts.len() {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }
            verts.extend_from_slice(&tile.verts[vert_idx * 3..(vert_idx + 1) * 3]);
        }

        // Check if point is inside polygon using robust point-in-polygon test
        let is_inside = Self::point_in_polygon_robust(pos, &verts, poly.vert_count as usize);

        if is_inside {
            // Point is inside - interpolate height and return position
            let mut closest = *pos;

            // Use polygon height calculation for accurate Y coordinate
            if let Some(height) = self.get_poly_height(tile, poly, pos)? {
                closest[1] = height;
            } else {
                // Fallback to average height of polygon vertices
                closest[1] =
                    Self::interpolate_polygon_height(&verts, pos, poly.vert_count as usize);
            }

            Ok((closest, true))
        } else {
            // Point is outside - find closest point on polygon boundary
            let closest_point =
                Self::closest_point_on_polygon_boundary(pos, &verts, poly.vert_count as usize);
            Ok((closest_point, false))
        }
    }

    /// Finds the polygon nearest to the specified point
    pub fn find_nearest_poly(
        &self,
        _pos: &[f32; 3],
        _ext_bounds: &[f32; 3],
    ) -> Result<(PolyRef, [f32; 3])> {
        // TODO: Implement polygon search using BVH trees
        Ok((PolyRef::new(0), [0.0; 3]))
    }

    /// Gets the tile and polygon for a reference
    pub fn get_tile_and_poly_by_ref(&self, reference: PolyRef) -> Result<(&MeshTile, &Poly)> {
        // Extract tile and poly IDs from the reference
        let tile_id = decode_tile_id(reference);
        let poly_id = decode_poly_id(reference);

        // Convert to tile index
        let tile_idx =
            tile_id_to_index(tile_id).ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Check if the tile index is valid
        if tile_idx >= self.max_tiles as usize {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Get the tile
        let tile = match &self.tiles.get(tile_idx) {
            Some(Some(tile)) => tile,
            _ => return Err(Error::Detour(Status::InvalidParam.to_string())),
        };

        // Check if the poly ID is valid
        if poly_id >= tile.polys.len() as u32 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Get the polygon
        let poly = &tile.polys[poly_id as usize];

        Ok((tile, poly))
    }

    /// Gets the tile and polygon for a reference (mutable version)
    pub fn get_tile_and_poly_by_ref_mut(
        &mut self,
        reference: PolyRef,
    ) -> Result<(&mut MeshTile, &mut Poly)> {
        // Extract tile and poly IDs from the reference
        let tile_id = decode_tile_id(reference);
        let poly_id = decode_poly_id(reference);

        // Convert to tile index
        let tile_idx =
            tile_id_to_index(tile_id).ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Check if the tile index is valid
        if tile_idx >= self.max_tiles as usize {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Get the tile
        let tile = match self.tiles.get_mut(tile_idx) {
            Some(Some(tile)) => tile,
            _ => return Err(Error::Detour(Status::InvalidParam.to_string())),
        };

        // Check if the poly ID is valid
        if poly_id >= tile.polys.len() as u32 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Can't return direct mutable references to both tile and poly because of Rust's borrowing rules
        // This would require a more complex solution
        // For now, using a workaround with unsafe
        // SAFETY: This is unsafe because we're creating multiple mutable references to the same data
        // However, it's safe in this context because we're only returning references to disjoint parts of the data
        unsafe {
            let tile_ptr = tile as *mut MeshTile;
            let poly_ptr = (*tile_ptr).polys.as_mut_ptr().add(poly_id as usize);

            Ok((&mut *tile_ptr, &mut *poly_ptr))
        }
    }

    /// Gets the tile at the specified coordinates
    pub fn get_tile_at(&self, x: i32, y: i32, layer: i32) -> Option<&MeshTile> {
        if let Some(&tile_idx) = self.pos_lookup.get(&(x, y, layer)) {
            if let Some(Some(tile)) = self.tiles.get(tile_idx) {
                return Some(tile);
            }
        }

        None
    }

    /// Gets all tiles in the navigation mesh
    pub fn get_all_tiles(&self) -> Vec<&MeshTile> {
        self.tiles
            .iter()
            .filter_map(|tile_opt| tile_opt.as_ref())
            .collect()
    }

    /// Gets the navigation mesh parameters
    pub fn get_params(&self) -> &NavMeshParams {
        &self.params
    }

    /// Gets the maximum number of tiles
    pub fn get_max_tiles(&self) -> i32 {
        self.max_tiles
    }

    /// Gets a tile by index (for debugging)
    #[cfg(test)]
    pub fn get_tile_by_index(&self, index: usize) -> Option<&MeshTile> {
        self.tiles.get(index).and_then(|t| t.as_ref())
    }

    /// Gets the maximum number of polygons per tile
    pub fn get_max_polys_per_tile(&self) -> i32 {
        self.max_polys_per_tile
    }

    /// Gets the origin of the navigation mesh
    pub fn get_origin(&self) -> [f32; 3] {
        self.origin
    }

    /// Builds the global BVH tree for all polygons in the navigation mesh
    pub fn build_bvh_tree(&mut self) -> Result<()> {
        let mut items = Vec::new();

        // Collect all polygons from all tiles
        for (tile_idx, tile_opt) in self.tiles.iter().enumerate() {
            if let Some(tile) = tile_opt {
                // Get the base reference for this tile
                let base_ref = self.get_poly_ref_base(tile);
                if base_ref != 0 {
                    // Decode to get tile components
                    let (salt, tile_id, _) = decode_poly_ref_full(PolyRef::new(base_ref));

                    // Add regular polygons
                    for (poly_idx, poly) in tile.polys.iter().enumerate() {
                        // Skip off-mesh connections
                        if poly.poly_type == PolyType::OffMeshConnection {
                            continue;
                        }

                        // Calculate polygon bounds
                        let mut bounds = Aabb::empty();
                        for j in 0..poly.vert_count as usize {
                            let v_idx = poly.verts[j] as usize;
                            if v_idx * 3 + 2 < tile.verts.len() {
                                let vertex = [
                                    tile.verts[v_idx * 3],
                                    tile.verts[v_idx * 3 + 1],
                                    tile.verts[v_idx * 3 + 2],
                                ];
                                bounds.expand_point(&vertex);
                            }
                        }

                        if bounds.is_valid() {
                            let poly_ref =
                                encode_poly_ref_with_salt(salt, tile_id, poly_idx as u32);
                            items.push(BVHItem { poly_ref, bounds });
                        }
                    }
                }

                // Add off-mesh connections
                for (conn_idx, connection) in tile.off_mesh_connections.iter().enumerate() {
                    let start = [connection.pos[0], connection.pos[1], connection.pos[2]];
                    let end = [connection.pos[3], connection.pos[4], connection.pos[5]];

                    let mut bounds = Aabb::empty();
                    bounds.expand_point(&start);
                    bounds.expand_point(&end);

                    // Expand bounds by radius
                    bounds.min[0] -= connection.radius;
                    bounds.min[1] -= connection.radius;
                    bounds.min[2] -= connection.radius;
                    bounds.max[0] += connection.radius;
                    bounds.max[1] += connection.radius;
                    bounds.max[2] += connection.radius;

                    let poly_ref = self.generate_off_mesh_poly_ref(tile_idx, conn_idx);
                    items.push(BVHItem { poly_ref, bounds });
                }
            }
        }

        // Build the BVH tree
        let mut bvh = BVHTree::new();
        bvh.build(items);
        self.global_bvh = Some(bvh);

        Ok(())
    }

    /// Queries polygons that overlap the specified bounds
    pub fn query_polygons(
        &self,
        bmin: &[f32; 3],
        bmax: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<Vec<PolyRef>> {
        // If we have a BVH tree, use it for efficient queries
        if let Some(bvh) = &self.global_bvh {
            let query_bounds = Aabb::new(*bmin, *bmax);
            let candidates = bvh.query(&query_bounds);

            // Filter candidates based on the query filter
            let mut result = Vec::new();
            for poly_ref in candidates {
                if let Ok((tile, poly)) = self.get_tile_and_poly_by_ref(poly_ref) {
                    if filter.pass_filter(poly_ref, tile, poly) {
                        result.push(poly_ref);
                    }
                }
            }
            return Ok(result);
        }

        // Fallback to brute force search
        let mut result = Vec::new();

        // Convert bounds to tile coordinates
        let imin = ((bmin[0] - self.origin[0]) / self.tile_width).floor() as i32;
        let imax = ((bmax[0] - self.origin[0]) / self.tile_width).floor() as i32;
        let jmin = ((bmin[2] - self.origin[2]) / self.tile_height).floor() as i32;
        let jmax = ((bmax[2] - self.origin[2]) / self.tile_height).floor() as i32;

        // Query all tiles that overlap the bounds
        for y in jmin..=jmax {
            for x in imin..=imax {
                // Check all layers at this position
                for layer in 0..self.max_tiles {
                    if let Some(tile) = self.get_tile_at(x, y, layer) {
                        // Query polygons in this tile
                        let tile_polys =
                            self.query_polygons_in_tile_internal(tile, bmin, bmax, filter)?;
                        result.extend(tile_polys);

                        // Query off-mesh connections in this tile
                        let off_mesh_polys =
                            self.query_off_mesh_connections_in_tile(tile, bmin, bmax, filter)?;
                        result.extend(off_mesh_polys);
                    }
                }
            }
        }

        Ok(result)
    }

    /// Queries polygons in a single tile
    ///
    /// This is a more efficient version of `query_polygons` when you know which tile to search.
    /// It only searches within the specified tile, avoiding the overhead of multi-tile queries.
    ///
    /// # Arguments
    ///
    /// * `tile_x` - The X coordinate of the tile
    /// * `tile_y` - The Y coordinate of the tile
    /// * `bmin` - The minimum bounds of the search area
    /// * `bmax` - The maximum bounds of the search area
    /// * `filter` - The polygon filter to apply
    ///
    /// # Returns
    ///
    /// A vector of polygon references that overlap the search area and pass the filter
    ///
    /// # Example
    ///
    /// ```ignore
    /// // Query polygons in a specific tile
    /// let tile_x = 5;
    /// let tile_y = 3;
    /// let bmin = [10.0, 0.0, 10.0];
    /// let bmax = [20.0, 5.0, 20.0];
    /// let polys = nav_mesh.query_polygons_in_tile(tile_x, tile_y, &bmin, &bmax, &filter)?;
    /// ```
    pub fn query_polygons_in_tile(
        &self,
        tile_x: i32,
        tile_y: i32,
        bmin: &[f32; 3],
        bmax: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<Vec<PolyRef>> {
        // Get the tile at the specified coordinates
        match self.get_tile_at(tile_x, tile_y, 0) {
            Some(tile) => self.query_polygons_in_tile_internal(tile, bmin, bmax, filter),
            None => Ok(Vec::new()), // No tile at this location
        }
    }

    /// Internal helper for querying polygons in a tile
    fn query_polygons_in_tile_internal(
        &self,
        tile: &MeshTile,
        qmin: &[f32; 3],
        qmax: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<Vec<PolyRef>> {
        let mut result = Vec::new();

        // Get the base reference for this tile
        let base_ref = self.get_poly_ref_base(tile);
        if base_ref == 0 {
            return Ok(result); // No valid reference, return empty
        }

        // Decode to get tile components
        let (salt, tile_id, _) = decode_poly_ref_full(PolyRef::new(base_ref));

        // If the tile has a BVH tree, use it for efficient queries
        if let Some(root_idx) = tile.bvh_root {
            if !tile.bvh_nodes.is_empty() {
                // Get tile header for quantization factor
                let header = match &tile.header {
                    Some(h) => h,
                    None => {
                        // Fallback to linear search if no header
                        return self.query_polygons_in_tile_linear(
                            tile, qmin, qmax, filter, salt, tile_id,
                        );
                    }
                };

                // Quantize query bounds for efficient comparison
                let (q_bounds, _) = BVNode::quantize_bounds(qmin, qmax, &header.bmin, &header.bmax);

                // Use BVH tree traversal
                let mut stack = vec![root_idx];

                while let Some(node_idx) = stack.pop() {
                    if node_idx >= tile.bvh_nodes.len() {
                        continue;
                    }

                    let node = &tile.bvh_nodes[node_idx];

                    // Check if node bounds overlap query bounds using quantized comparison
                    if !BVNode::overlap_quant_bounds(
                        &q_bounds.bmin,
                        &q_bounds.bmax,
                        &node.bmin,
                        &node.bmax,
                    ) {
                        continue;
                    }

                    // Check node type
                    if node.i < 0 {
                        // Leaf node with single polygon
                        let poly_idx = (-node.i - 1) as usize;
                        if poly_idx < tile.polys.len() {
                            let poly = &tile.polys[poly_idx];

                            // Skip off-mesh connections
                            if poly.poly_type != PolyType::OffMeshConnection {
                                let poly_ref =
                                    encode_poly_ref_with_salt(salt, tile_id, poly_idx as u32);
                                if filter.pass_filter(poly_ref, tile, poly) {
                                    result.push(poly_ref);
                                }
                            }
                        }
                    } else if node.i >= 0 && node.i < (1 << 16) {
                        // Internal node - add children to stack
                        let child_idx = node.i as usize;
                        if child_idx + 1 < tile.bvh_nodes.len() {
                            stack.push(child_idx + 1);
                            stack.push(child_idx);
                        }
                    } else {
                        // Leaf node with two polygons
                        let poly_idx0 = ((node.i >> 16) & 0xFFFF) as usize;
                        let poly_idx1 = (node.i & 0xFFFF) as usize;

                        // Check first polygon
                        if poly_idx0 < tile.polys.len() {
                            let poly = &tile.polys[poly_idx0];
                            if poly.poly_type != PolyType::OffMeshConnection {
                                let poly_ref =
                                    encode_poly_ref_with_salt(salt, tile_id, poly_idx0 as u32);
                                if filter.pass_filter(poly_ref, tile, poly) {
                                    result.push(poly_ref);
                                }
                            }
                        }

                        // Check second polygon
                        if poly_idx1 < tile.polys.len() {
                            let poly = &tile.polys[poly_idx1];
                            if poly.poly_type != PolyType::OffMeshConnection {
                                let poly_ref =
                                    encode_poly_ref_with_salt(salt, tile_id, poly_idx1 as u32);
                                if filter.pass_filter(poly_ref, tile, poly) {
                                    result.push(poly_ref);
                                }
                            }
                        }
                    }
                }

                return Ok(result);
            }
        }

        // Fallback to linear search if no BVH tree is available
        self.query_polygons_in_tile_linear(tile, qmin, qmax, filter, salt, tile_id)
    }

    /// Linear search through polygons in a tile (fallback when no BVH available)
    fn query_polygons_in_tile_linear(
        &self,
        tile: &MeshTile,
        qmin: &[f32; 3],
        qmax: &[f32; 3],
        filter: &QueryFilter,
        salt: u32,
        tile_id: u32,
    ) -> Result<Vec<PolyRef>> {
        let mut result = Vec::new();

        for (i, poly) in tile.polys.iter().enumerate() {
            // Skip off-mesh connections (they are handled separately)
            if poly.poly_type == PolyType::OffMeshConnection {
                continue;
            }

            // Create poly ref with proper salt
            let poly_ref = encode_poly_ref_with_salt(salt, tile_id, i as u32);

            // Apply filter
            if !filter.pass_filter(poly_ref, tile, poly) {
                continue;
            }

            // Calculate polygon bounds
            let mut poly_bmin = [f32::MAX; 3];
            let mut poly_bmax = [f32::MIN; 3];

            for j in 0..poly.vert_count as usize {
                let v_idx = poly.verts[j] as usize;
                if v_idx * 3 + 2 >= tile.verts.len() {
                    continue; // Skip invalid vertex indices
                }

                poly_bmin[0] = poly_bmin[0].min(tile.verts[v_idx * 3]);
                poly_bmin[1] = poly_bmin[1].min(tile.verts[v_idx * 3 + 1]);
                poly_bmin[2] = poly_bmin[2].min(tile.verts[v_idx * 3 + 2]);

                poly_bmax[0] = poly_bmax[0].max(tile.verts[v_idx * 3]);
                poly_bmax[1] = poly_bmax[1].max(tile.verts[v_idx * 3 + 1]);
                poly_bmax[2] = poly_bmax[2].max(tile.verts[v_idx * 3 + 2]);
            }

            // Check if polygon bounds overlap query bounds
            if Self::overlap_bounds(qmin, qmax, &poly_bmin, &poly_bmax) {
                result.push(poly_ref);
            }
        }

        Ok(result)
    }

    /// Queries off-mesh connections in a single tile
    fn query_off_mesh_connections_in_tile(
        &self,
        tile: &MeshTile,
        qmin: &[f32; 3],
        qmax: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<Vec<PolyRef>> {
        let mut result = Vec::new();

        // Check each off-mesh connection in the tile
        for connection in tile.off_mesh_connections.iter() {
            // Apply filter - treat off-mesh connection as a polygon for filtering
            if !filter.include_flags.intersects(connection.flags)
                || filter.exclude_flags.intersects(connection.flags)
            {
                continue;
            }

            // Check if either endpoint is within the query bounds
            let start_pos = connection.start_pos();
            let end_pos = connection.end_pos();

            let start_in_bounds = Self::point_in_bounds(&start_pos, qmin, qmax);
            let end_in_bounds = Self::point_in_bounds(&end_pos, qmin, qmax);

            // Also check if the connection passes through the query bounds
            let intersects_bounds = start_in_bounds
                || end_in_bounds
                || Self::segment_intersects_bounds(&start_pos, &end_pos, qmin, qmax);

            if intersects_bounds {
                result.push(connection.poly);
            }
        }

        Ok(result)
    }

    /// Checks if a point is within the given bounds
    fn point_in_bounds(point: &[f32; 3], bmin: &[f32; 3], bmax: &[f32; 3]) -> bool {
        point[0] >= bmin[0]
            && point[0] <= bmax[0]
            && point[1] >= bmin[1]
            && point[1] <= bmax[1]
            && point[2] >= bmin[2]
            && point[2] <= bmax[2]
    }

    /// Checks if a line segment intersects with the given bounds
    fn segment_intersects_bounds(
        start: &[f32; 3],
        end: &[f32; 3],
        bmin: &[f32; 3],
        bmax: &[f32; 3],
    ) -> bool {
        // Simple Aabb vs line segment intersection test
        // This is a conservative test - it may return true for some non-intersecting cases

        let min_x = start[0].min(end[0]);
        let max_x = start[0].max(end[0]);
        let min_y = start[1].min(end[1]);
        let max_y = start[1].max(end[1]);
        let min_z = start[2].min(end[2]);
        let max_z = start[2].max(end[2]);

        // Check if the line segment's bounding box overlaps with the query bounds
        !(max_x < bmin[0]
            || min_x > bmax[0]
            || max_y < bmin[1]
            || min_y > bmax[1]
            || max_z < bmin[2]
            || min_z > bmax[2])
    }

    /// Checks if two axis-aligned bounding boxes overlap
    fn overlap_bounds(amin: &[f32; 3], amax: &[f32; 3], bmin: &[f32; 3], bmax: &[f32; 3]) -> bool {
        !(amin[0] > bmax[0]
            || amax[0] < bmin[0]
            || amin[1] > bmax[1]
            || amax[1] < bmin[1]
            || amin[2] > bmax[2]
            || amax[2] < bmin[2])
    }

    /// Robust point-in-polygon test using ray casting algorithm
    fn point_in_polygon_robust(point: &[f32; 3], verts: &[f32], vert_count: usize) -> bool {
        const EPSILON: f32 = 1e-6;

        let px = point[0];
        let pz = point[2];
        let mut inside = false;

        // Ray casting algorithm - cast a ray from point to infinity and count intersections
        let mut j = vert_count - 1;
        for i in 0..vert_count {
            let vi_x = verts[i * 3];
            let vi_z = verts[i * 3 + 2];
            let vj_x = verts[j * 3];
            let vj_z = verts[j * 3 + 2];

            // Check if ray crosses this edge
            if (vi_z > pz) != (vj_z > pz) {
                // Calculate intersection point
                let t = (pz - vi_z) / (vj_z - vi_z);
                let intersection_x = vi_x + t * (vj_x - vi_x);

                // If intersection is to the right of point, count it
                if px < intersection_x + EPSILON {
                    inside = !inside;
                }
            }

            j = i;
        }

        inside
    }

    /// Interpolates height within a polygon using barycentric coordinates
    fn interpolate_polygon_height(verts: &[f32], point: &[f32; 3], vert_count: usize) -> f32 {
        if vert_count == 0 {
            return point[1];
        }

        // For simple case, use average height of vertices
        // TODO: Could be improved with proper barycentric interpolation for triangulated polygons
        let mut total_height = 0.0;
        for i in 0..vert_count {
            total_height += verts[i * 3 + 1];
        }

        total_height / vert_count as f32
    }

    /// Finds the closest point on the polygon boundary to the given point
    fn closest_point_on_polygon_boundary(
        point: &[f32; 3],
        verts: &[f32],
        vert_count: usize,
    ) -> [f32; 3] {
        let mut closest_point = *point;
        let mut min_distance_sqr = f32::MAX;

        // Check each edge of the polygon
        for i in 0..vert_count {
            let j = (i + 1) % vert_count;

            let v1 = [verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2]];
            let v2 = [verts[j * 3], verts[j * 3 + 1], verts[j * 3 + 2]];

            // Find closest point on this edge segment
            let edge_closest = Self::closest_point_on_segment_3d(point, &v1, &v2);

            // Calculate distance squared
            let dx = edge_closest[0] - point[0];
            let dy = edge_closest[1] - point[1];
            let dz = edge_closest[2] - point[2];
            let dist_sqr = dx * dx + dy * dy + dz * dz;

            if dist_sqr < min_distance_sqr {
                min_distance_sqr = dist_sqr;
                closest_point = edge_closest;
            }
        }

        closest_point
    }

    /// Finds the closest point on a 3D line segment
    fn closest_point_on_segment_3d(
        point: &[f32; 3],
        seg_start: &[f32; 3],
        seg_end: &[f32; 3],
    ) -> [f32; 3] {
        let dx = seg_end[0] - seg_start[0];
        let dy = seg_end[1] - seg_start[1];
        let dz = seg_end[2] - seg_start[2];

        let px = point[0] - seg_start[0];
        let py = point[1] - seg_start[1];
        let pz = point[2] - seg_start[2];

        let seg_length_sqr = dx * dx + dy * dy + dz * dz;

        if seg_length_sqr < 1e-6 {
            // Degenerate segment - return start point
            return *seg_start;
        }

        // Calculate parameter t along the segment [0, 1]
        let t = ((px * dx + py * dy + pz * dz) / seg_length_sqr).clamp(0.0, 1.0);

        // Return interpolated point
        [
            seg_start[0] + t * dx,
            seg_start[1] + t * dy,
            seg_start[2] + t * dz,
        ]
    }

    /// Gets the height of a polygon at the specified position
    pub fn get_poly_height(
        &self,
        tile: &MeshTile,
        poly: &Poly,
        pos: &[f32; 3],
    ) -> Result<Option<f32>> {
        // Off-mesh connections don't have detail polygons
        if poly.poly_type == PolyType::OffMeshConnection {
            return Ok(None);
        }

        // Get polygon vertices
        let mut verts = Vec::with_capacity(poly.vert_count as usize * 3);
        for i in 0..poly.vert_count as usize {
            let v_idx = poly.verts[i] as usize;
            verts.extend_from_slice(&tile.verts[v_idx * 3..(v_idx + 1) * 3]);
        }

        // Check if point is inside polygon (2D check)
        if !Self::point_in_polygon_2d(pos, &verts, poly.vert_count as usize) {
            return Ok(None);
        }

        // Get polygon detail - for now use the polygon's position in the tile
        // In the original implementation, poly.index would point to the detail mesh
        // Since we don't have that field, we'll need to find the polygon's index
        let poly_ptr = poly as *const Poly;
        let first_poly_ptr = tile.polys.as_ptr();
        let ip = unsafe { poly_ptr.offset_from(first_poly_ptr) as usize };

        if ip >= tile.detail_meshes.len() {
            return Ok(None);
        }

        let pd = &tile.detail_meshes[ip];

        // Find height from detail triangles
        for j in 0..pd.tri_count as usize {
            let t_base = (pd.tri_base + j as u32) as usize;
            if t_base * 4 + 3 >= tile.detail_tris.len() {
                continue;
            }

            let t = &tile.detail_tris[t_base * 4..(t_base + 1) * 4];

            let mut v: [[f32; 3]; 3] = [[0.0; 3]; 3];
            for k in 0..3 {
                if t[k] < poly.vert_count {
                    let v_idx = poly.verts[t[k] as usize] as usize;
                    v[k] = [
                        tile.verts[v_idx * 3],
                        tile.verts[v_idx * 3 + 1],
                        tile.verts[v_idx * 3 + 2],
                    ];
                } else {
                    let dv_idx = (pd.vert_base + (t[k] - poly.vert_count) as u32) as usize;
                    if dv_idx * 3 + 2 < tile.detail_verts.len() {
                        v[k] = [
                            tile.detail_verts[dv_idx * 3],
                            tile.detail_verts[dv_idx * 3 + 1],
                            tile.detail_verts[dv_idx * 3 + 2],
                        ];
                    }
                }
            }

            // Calculate height on triangle
            if let Some(h) = Self::closest_height_on_triangle(pos, &v[0], &v[1], &v[2]) {
                return Ok(Some(h));
            }
        }

        // If we couldn't find height from triangles, interpolate from polygon vertices
        if poly.vert_count >= 3 {
            let v0_idx = poly.verts[0] as usize;
            let v0 = [
                tile.verts[v0_idx * 3],
                tile.verts[v0_idx * 3 + 1],
                tile.verts[v0_idx * 3 + 2],
            ];
            return Ok(Some(v0[1])); // Return first vertex height as fallback
        }

        Ok(None)
    }

    /// Checks if a point is inside a polygon (2D check)
    fn point_in_polygon_2d(pt: &[f32; 3], verts: &[f32], nverts: usize) -> bool {
        let mut c = false;
        let x = pt[0];
        let z = pt[2];

        for i in 0..nverts {
            let j = (i + 1) % nverts;

            let vi_x = verts[i * 3];
            let vi_z = verts[i * 3 + 2];
            let vj_x = verts[j * 3];
            let vj_z = verts[j * 3 + 2];

            if ((vi_z > z) != (vj_z > z)) && (x < (vj_x - vi_x) * (z - vi_z) / (vj_z - vi_z) + vi_x)
            {
                c = !c;
            }
        }

        c
    }

    /// Finds the closest point's height on a triangle
    fn closest_height_on_triangle(
        p: &[f32; 3],
        a: &[f32; 3],
        b: &[f32; 3],
        c: &[f32; 3],
    ) -> Option<f32> {
        let v0 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let v1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v2 = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];

        // Calculate barycentric coordinates
        let dot00 = v0[0] * v0[0] + v0[2] * v0[2];
        let dot01 = v0[0] * v1[0] + v0[2] * v1[2];
        let dot02 = v0[0] * v2[0] + v0[2] * v2[2];
        let dot11 = v1[0] * v1[0] + v1[2] * v1[2];
        let dot12 = v1[0] * v2[0] + v1[2] * v2[2];

        let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        // Check if point is in triangle
        if u >= 0.0 && v >= 0.0 && (u + v) <= 1.0 {
            let h = a[1] + v0[1] * u + v1[1] * v;
            Some(h)
        } else {
            None
        }
    }

    /// Finds a link from the given polygon to a neighbor through the specified edge
    pub fn find_link<'a>(&self, tile: &'a MeshTile, poly: &Poly, edge: u8) -> Option<&'a Link> {
        // Check if the edge is valid
        if edge >= poly.vert_count {
            return None;
        }

        // Search through the polygon's links using proper linked list traversal
        let mut link_idx = poly.first_link;
        while let Some(idx) = link_idx {
            if idx >= tile.links.len() {
                break;
            }

            let link = &tile.links[idx];
            if link.edge_index == edge {
                return Some(link);
            }

            // Move to the next link in the linked list
            link_idx = link.next.map(|n| n as usize);
        }

        None
    }

    /// Gets all links for a given polygon
    pub fn get_polygon_links<'a>(&self, tile: &'a MeshTile, poly: &Poly) -> Vec<&'a Link> {
        let mut links = Vec::new();
        let mut link_idx = poly.first_link;

        while let Some(idx) = link_idx {
            if idx >= tile.links.len() {
                break;
            }

            let link = &tile.links[idx];
            links.push(link);

            // Move to the next link in the linked list
            link_idx = link.next.map(|n| n as usize);
        }

        links
    }

    /// Adds a link to a polygon in a tile (for mesh building)
    /// This is typically used during navigation mesh construction
    pub fn add_link_to_tile_polygon(
        tile: &mut MeshTile,
        poly_idx: usize,
        link: Link,
    ) -> Result<()> {
        if poly_idx >= tile.polys.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Find a free slot in the links array or add a new one
        let link_idx = tile.links.len();
        tile.links.push(link);

        // Update the polygon's link chain
        let poly = &mut tile.polys[poly_idx];
        if let Some(first_link) = poly.first_link {
            // Insert at the beginning of the linked list
            tile.links[link_idx].next = Some(first_link as u32);
        }
        poly.first_link = Some(link_idx);

        Ok(())
    }

    /// Removes all links for a polygon
    /// This is typically used during navigation mesh updates
    pub fn clear_polygon_links(tile: &mut MeshTile, poly_idx: usize) -> Result<()> {
        if poly_idx >= tile.polys.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Clear the polygon's first link pointer
        tile.polys[poly_idx].first_link = None;

        // Note: In a more sophisticated implementation, we would also
        // add the freed links to a free list for reuse
        Ok(())
    }

    /// Counts the number of links for a polygon
    pub fn count_polygon_links(&self, tile: &MeshTile, poly: &Poly) -> usize {
        let mut count = 0;
        let mut link_idx = poly.first_link;

        while let Some(idx) = link_idx {
            if idx >= tile.links.len() {
                break;
            }

            count += 1;
            let link = &tile.links[idx];
            link_idx = link.next.map(|n| n as usize);
        }

        count
    }

    /// Gets the tile width
    pub fn get_tile_width(&self) -> f32 {
        self.tile_width
    }

    /// Gets the tile height
    pub fn get_tile_height(&self) -> f32 {
        self.tile_height
    }

    /// Checks if a polygon reference is valid
    pub fn is_valid_poly_ref(&self, reference: PolyRef) -> bool {
        if reference.id() == 0 {
            return false;
        }

        let salt = decode_poly_id_salt(reference);
        let tile_id = decode_tile_id(reference);
        let poly_id = decode_poly_id(reference);

        // Convert to tile index
        let tile_idx = match tile_id_to_index(tile_id) {
            Some(idx) => idx,
            None => return false,
        };

        if tile_idx >= self.max_tiles as usize {
            return false;
        }

        match &self.tiles.get(tile_idx) {
            Some(Some(tile)) => {
                // Check salt matches
                if salt != (tile.salt & DT_SALT_MASK) {
                    return false;
                }

                if poly_id >= tile.polys.len() as u32 {
                    return false;
                }

                true
            }
            _ => false,
        }
    }

    /// Builds a navigation mesh from Recast data
    pub fn build_from_recast(
        params: NavMeshParams,
        poly_mesh: &PolyMesh,
        detail_mesh: &PolyMeshDetail,
        flags: NavMeshFlags,
    ) -> Result<Self> {
        // Create a new navigation mesh
        let mut nav_mesh = Self::new(params)?;

        // Set flags
        nav_mesh.flags = flags;

        // Create a single tile for the entire mesh
        let mut tile = MeshTile::new();

        // Set up tile header
        tile.header = Some(TileHeader {
            x: 0,
            y: 0,
            layer: 0,
            user_id: 0,
            data_size: 0, // Will be calculated later
            bmin: [poly_mesh.bmin.x, poly_mesh.bmin.y, poly_mesh.bmin.z],
            bmax: [poly_mesh.bmax.x, poly_mesh.bmax.y, poly_mesh.bmax.z],
            poly_count: poly_mesh.poly_count as i32,
            vert_count: poly_mesh.vert_count as i32,
            max_links: poly_mesh.poly_count as i32 * 6, // Max 6 links per polygon
            detail_mesh_count: detail_mesh.poly_count as i32,
            detail_vert_count: detail_mesh.vert_count as i32,
            detail_tri_count: detail_mesh.tri_count as i32,
            bvh_node_count: 0, // No BVH tree for now
            off_mesh_connection_count: 0,
            bv_quant_factor: 0.0,
        });

        // Convert Recast vertices to Detour format (from integer to float coordinates)
        tile.verts.reserve(poly_mesh.vert_count * 3);
        for i in 0..poly_mesh.vert_count {
            let x = poly_mesh.bmin.x + (poly_mesh.vertices[i * 3] as f32) * poly_mesh.cs;
            let y = poly_mesh.bmin.y + (poly_mesh.vertices[i * 3 + 1] as f32) * poly_mesh.ch;
            let z = poly_mesh.bmin.z + (poly_mesh.vertices[i * 3 + 2] as f32) * poly_mesh.cs;
            tile.verts.push(x);
            tile.verts.push(y);
            tile.verts.push(z);
        }

        // Convert Recast polygons to Detour format
        tile.polys.reserve(poly_mesh.poly_count);
        for i in 0..poly_mesh.poly_count {
            let poly_start = i * poly_mesh.max_verts_per_poly;
            let mut poly = Poly::new(
                poly_mesh.areas[i],
                PolyType::Ground,
                PolyFlags::WALK, // Default to walkable
            );

            // Copy vertices
            let mut vert_count = 0;
            for j in 0..poly_mesh.max_verts_per_poly {
                let v = poly_mesh.polys[poly_start + j];
                if v == MESH_NULL_IDX {
                    break;
                }
                poly.verts[j] = v;
                vert_count += 1;
            }
            poly.vert_count = vert_count;

            // Initialize neighbors to 0 (will be calculated later)
            for j in 0..MAX_VERTS_PER_POLY {
                poly.neighbors[j] = 0;
            }

            tile.polys.push(poly);
        }

        // Calculate neighbor connections and create links between polygons
        // For a single-tile mesh, we check for shared edges between polygons
        // We'll create the base PolyRef using tile_idx (which will be allocated below)
        // For now, we'll calculate it the same way get_poly_ref_base would
        let base = 1u32 << DT_POLY_BITS; // tile_idx 0 maps to tile_id 1

        // Structure to hold neighbor information temporarily
        struct NeighborInfo {
            poly_i: usize,
            poly_j: usize,
            edge_i: usize,
            edge_j: usize,
        }
        let mut neighbors: Vec<NeighborInfo> = Vec::new();

        // Find all neighbor connections first
        for i in 0..tile.polys.len() {
            for j in i + 1..tile.polys.len() {
                // Get polygon vertex counts
                let poly_i_vert_count = tile.polys[i].vert_count as usize;
                let poly_j_vert_count = tile.polys[j].vert_count as usize;

                // Check each edge of polygon i
                for k in 0..poly_i_vert_count {
                    let k_next = (k + 1) % poly_i_vert_count;
                    let vi = tile.polys[i].verts[k];
                    let vj = tile.polys[i].verts[k_next];

                    // Check if polygon j shares this edge (in reverse order)
                    for l in 0..poly_j_vert_count {
                        let l_next = (l + 1) % poly_j_vert_count;
                        if tile.polys[j].verts[l] == vj && tile.polys[j].verts[l_next] == vi {
                            // Found shared edge!
                            neighbors.push(NeighborInfo {
                                poly_i: i,
                                poly_j: j,
                                edge_i: k,
                                edge_j: l,
                            });
                            break;
                        }
                    }
                }
            }
        }

        // Now create links for all neighbor connections
        for neighbor in neighbors {
            // Create link from polygon i to polygon j
            let link_i_to_j = Link {
                reference: PolyRef::new(base | neighbor.poly_j as u32),
                neighbor_index: neighbor.poly_j as u8,
                edge_index: neighbor.edge_i as u8,
                side: 0xff,       // Internal edge
                boundary_flag: 0, // Internal connection
                next: None,
            };

            // Create link from polygon j to polygon i
            let link_j_to_i = Link {
                reference: PolyRef::new(base | neighbor.poly_i as u32),
                neighbor_index: neighbor.poly_i as u8,
                edge_index: neighbor.edge_j as u8,
                side: 0xff,       // Internal edge
                boundary_flag: 0, // Internal connection
                next: None,
            };

            // Add links to tile
            let link_i_idx = tile.links.len();
            tile.links.push(link_i_to_j);

            let link_j_idx = tile.links.len();
            tile.links.push(link_j_to_i);

            // Update first_link pointers
            if let Some(prev_first) = tile.polys[neighbor.poly_i].first_link {
                tile.links[link_i_idx].next = Some(prev_first as u32);
            }
            tile.polys[neighbor.poly_i].first_link = Some(link_i_idx);

            if let Some(prev_first) = tile.polys[neighbor.poly_j].first_link {
                tile.links[link_j_idx].next = Some(prev_first as u32);
            }
            tile.polys[neighbor.poly_j].first_link = Some(link_j_idx);

            // Also set neighbors for compatibility
            tile.polys[neighbor.poly_i].neighbors[neighbor.edge_i] = (neighbor.poly_j + 1) as u16;
            tile.polys[neighbor.poly_j].neighbors[neighbor.edge_j] = (neighbor.poly_i + 1) as u16;
        }

        // Copy detail mesh data if available
        if detail_mesh.vert_count > 0 {
            // Copy detail vertices
            tile.detail_verts = detail_mesh.vertices.clone();

            // Copy detail mesh info for each polygon
            tile.detail_meshes.reserve(detail_mesh.poly_count);
            for i in 0..detail_mesh.poly_count {
                let detail = PolyDetail {
                    vert_base: 0, // Will be calculated
                    tri_base: detail_mesh.poly_start[i] as u32,
                    vert_count: 0, // Will be calculated
                    tri_count: detail_mesh.poly_tri_count[i] as u8,
                };
                tile.detail_meshes.push(detail);
            }

            // Copy detail triangles
            tile.detail_tris.reserve(detail_mesh.tri_count * 4);
            for i in 0..detail_mesh.tri_count {
                for j in 0..3 {
                    tile.detail_tris
                        .push(detail_mesh.triangles[i * 3 + j] as u8);
                }
                tile.detail_tris.push(0); // Padding for alignment
            }
        }

        // Allocate a tile from the free list
        let tile_idx = nav_mesh.allocate_tile()?;

        // Add tile to the navigation mesh
        nav_mesh.tiles[tile_idx] = Some(tile);

        // Add to position lookup
        nav_mesh.pos_lookup.insert((0, 0, 0), tile_idx);

        // Build the BVH tree for efficient queries
        nav_mesh.build_bvh_tree()?;

        Ok(nav_mesh)
    }

    /// Saves the navigation mesh to a file in JSON format
    #[cfg(feature = "serialization")]
    pub fn save_to_json<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        let json = serde_json::to_string_pretty(self)
            .map_err(|_e| Error::Detour(Status::Failure.to_string()))?;

        std::fs::write(path, json).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        Ok(())
    }

    /// Loads a navigation mesh from a JSON file
    #[cfg(feature = "serialization")]
    pub fn load_from_json<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let json = std::fs::read_to_string(path)
            .map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        let nav_mesh =
            serde_json::from_str(&json).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        Ok(nav_mesh)
    }

    /// Saves the navigation mesh to a file in C++ compatible binary format
    pub fn save_to_cpp_binary<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        // For now, only support single-tile meshes
        if self.tiles.len() != 1 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Find the first valid tile
        let tile = self
            .tiles
            .iter()
            .find_map(|t| t.as_ref())
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        let data = super::binary_format::save_tile_to_binary(tile)?;
        std::fs::write(path, data).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        Ok(())
    }

    /// Loads a navigation mesh from C++ compatible binary format
    pub fn load_from_cpp_binary<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let data = std::fs::read(path).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        let tile = super::binary_format::load_tile_from_binary(&data)?;

        // Extract parameters from tile
        let header = tile
            .header
            .as_ref()
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        let params = NavMeshParams {
            origin: header.bmin,
            tile_width: header.bmax[0] - header.bmin[0],
            tile_height: header.bmax[2] - header.bmin[2],
            max_tiles: 1,
            max_polys_per_tile: header.poly_count,
        };

        // Store values we need from header before moving tile
        let tile_x = header.x;
        let tile_y = header.y;
        let tile_layer = header.layer;

        let mut nav_mesh = NavMesh::new(params)?;

        // Allocate a tile slot
        let tile_idx = nav_mesh.allocate_tile()?;
        nav_mesh.tiles[tile_idx] = Some(tile);

        // Add to position lookup
        nav_mesh
            .pos_lookup
            .insert((tile_x, tile_y, tile_layer), tile_idx);

        // Build BVH tree
        nav_mesh.build_bvh_tree()?;

        Ok(nav_mesh)
    }

    /// Loads a navigation mesh from a binary file (C++ compatible format)
    pub fn load_from_binary<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let data = std::fs::read(path).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        super::binary_format::load_nav_mesh_from_binary(&data)
    }

    /// Saves the navigation mesh to a binary file (C++ compatible format)
    pub fn save_to_binary<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        let data = super::binary_format::save_nav_mesh_to_binary(self)?;
        std::fs::write(path, data).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(())
    }

    /// Serializes the navigation mesh to JSON bytes
    #[cfg(feature = "serialization")]
    pub fn to_json_bytes(&self) -> Result<Vec<u8>> {
        let json =
            serde_json::to_vec(self).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(json)
    }

    /// Deserializes a navigation mesh from JSON bytes
    #[cfg(feature = "serialization")]
    pub fn from_json_bytes(data: &[u8]) -> Result<Self> {
        let nav_mesh =
            serde_json::from_slice(data).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(nav_mesh)
    }

    /// Serializes the navigation mesh to binary bytes
    #[cfg(feature = "serialization")]
    pub fn to_binary_bytes(&self) -> Result<Vec<u8>> {
        let data =
            postcard::to_allocvec(self).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(data)
    }

    /// Deserializes a navigation mesh from binary bytes
    #[cfg(feature = "serialization")]
    pub fn from_binary_bytes(data: &[u8]) -> Result<Self> {
        let nav_mesh =
            postcard::from_bytes(data).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(nav_mesh)
    }

    /// Adds an off-mesh connection to the navigation mesh
    #[allow(clippy::too_many_arguments)]
    pub fn add_off_mesh_connection(
        &mut self,
        start_pos: [f32; 3],
        end_pos: [f32; 3],
        radius: f32,
        flags: PolyFlags,
        area: u8,
        direction: u8,
        user_id: u32,
    ) -> Result<PolyRef> {
        // Find tiles that could contain the start or end positions
        let start_tile_pos = self.world_to_tile(start_pos);
        let _end_tile_pos = self.world_to_tile(end_pos);

        // For now, add the connection to the tile containing the start position
        // In a full implementation, we might need to handle connections spanning multiple tiles
        let tile_idx = self.get_tile_index_at(start_tile_pos.0, start_tile_pos.1, 0);

        if let Some(tile_idx) = tile_idx {
            // Get the connection count first
            let connection_count = if let Some(Some(tile)) = self.tiles.get(tile_idx) {
                tile.off_mesh_connections.len()
            } else {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            };

            // Generate the polygon reference before mutable borrow
            let poly_ref = self.generate_off_mesh_poly_ref(tile_idx, connection_count);

            // Now get mutable access to the tile
            if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
                // Create the off-mesh connection
                let mut connection = OffMeshConnection::new_with_params(
                    start_pos, end_pos, radius, flags, area, direction, user_id,
                );
                connection.poly = poly_ref;

                // Add the connection to the tile
                tile.off_mesh_connections.push(connection);

                // Update the tile header count
                if let Some(header) = &mut tile.header {
                    header.off_mesh_connection_count += 1;
                }

                return Ok(poly_ref);
            }
        }

        // If we couldn't find a suitable tile, create a new tile or return an error
        Err(Error::Detour(Status::InvalidParam.to_string()))
    }

    /// Removes an off-mesh connection by its polygon reference
    pub fn remove_off_mesh_connection(&mut self, connection_ref: PolyRef) -> Result<()> {
        // Find the tile and connection index from the polygon reference
        let (tile_idx, connection_idx) = self.decode_off_mesh_ref(connection_ref)?;

        if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
            if connection_idx < tile.off_mesh_connections.len() {
                tile.off_mesh_connections.remove(connection_idx);

                // Update the tile header count
                if let Some(header) = &mut tile.header {
                    header.off_mesh_connection_count =
                        header.off_mesh_connection_count.saturating_sub(1);
                }

                return Ok(());
            }
        }

        Err(Error::Detour(Status::NotFound.to_string()))
    }

    /// Gets all off-mesh connections in the navigation mesh
    pub fn get_all_off_mesh_connections(&self) -> Vec<&OffMeshConnection> {
        let mut connections = Vec::new();
        for tile in self.tiles.iter().flatten() {
            for connection in &tile.off_mesh_connections {
                connections.push(connection);
            }
        }
        connections
    }

    /// Finds off-mesh connections near a given position
    pub fn find_off_mesh_connections_near(
        &self,
        pos: [f32; 3],
        radius: f32,
    ) -> Vec<&OffMeshConnection> {
        let mut nearby_connections = Vec::new();

        for tile in self.tiles.iter().flatten() {
            for connection in &tile.off_mesh_connections {
                // Check distance to start position
                let start_pos = connection.start_pos();
                let start_dist = self.distance_3d(pos, start_pos);

                // Check distance to end position
                let end_pos = connection.end_pos();
                let end_dist = self.distance_3d(pos, end_pos);

                // Include if either endpoint is within radius
                if start_dist <= radius + connection.radius
                    || end_dist <= radius + connection.radius
                {
                    nearby_connections.push(connection);
                }
            }
        }

        nearby_connections
    }

    /// Gets an off-mesh connection by its polygon reference
    pub fn get_off_mesh_connection(&self, connection_ref: PolyRef) -> Result<&OffMeshConnection> {
        let (tile_idx, connection_idx) = self.decode_off_mesh_ref(connection_ref)?;

        if let Some(Some(tile)) = self.tiles.get(tile_idx) {
            if let Some(connection) = tile.off_mesh_connections.get(connection_idx) {
                return Ok(connection);
            }
        }

        Err(Error::Detour(Status::NotFound.to_string()))
    }

    /// Converts world coordinates to tile coordinates
    fn world_to_tile(&self, pos: [f32; 3]) -> (i32, i32) {
        let tx = ((pos[0] - self.origin[0]) / self.tile_width).floor() as i32;
        let tz = ((pos[2] - self.origin[2]) / self.tile_height).floor() as i32;
        (tx, tz)
    }

    /// Gets the tile index at specific tile coordinates
    fn get_tile_index_at(&self, tx: i32, ty: i32, layer: i32) -> Option<usize> {
        self.pos_lookup.get(&(tx, ty, layer)).copied()
    }

    /// Generates a polygon reference for an off-mesh connection
    fn generate_off_mesh_poly_ref(&self, tile_idx: usize, connection_idx: usize) -> PolyRef {
        // Use a special encoding for off-mesh connections
        // High bit set to distinguish from regular polygons
        let ref_val = 0x80000000 | ((tile_idx as u32) << 16) | (connection_idx as u32);
        PolyRef::new(ref_val)
    }

    /// Decodes an off-mesh connection reference to tile and connection indices
    fn decode_off_mesh_ref(&self, connection_ref: PolyRef) -> Result<(usize, usize)> {
        let ref_val = connection_ref.id();

        // Check if this is an off-mesh connection reference (high bit set)
        if (ref_val & 0x80000000) == 0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let tile_idx = ((ref_val >> 16) & 0x7FFF) as usize;
        let connection_idx = (ref_val & 0xFFFF) as usize;

        Ok((tile_idx, connection_idx))
    }

    /// Calculates 3D distance between two points
    fn distance_3d(&self, a: [f32; 3], b: [f32; 3]) -> f32 {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Checks if a polygon reference refers to an off-mesh connection
    pub fn is_off_mesh_connection(&self, poly_ref: PolyRef) -> bool {
        (poly_ref.id() & 0x80000000) != 0
    }

    /// Helper method for testing - creates a test tile
    #[cfg(test)]
    pub fn create_test_tile(&mut self, x: i32, y: i32, layer: i32) -> Result<()> {
        // Find the next free tile
        let tile_idx = if let Some(free_idx) = self.next_free {
            // Update next_free to point to the next tile in the free list
            if let Some(Some(tile)) = self.tiles.get(free_idx) {
                self.next_free = tile.next;
            } else {
                self.next_free = None;
            }
            free_idx
        } else {
            return Err(Error::Detour(Status::BufferTooSmall.to_string()));
        };

        // Update the existing tile instead of creating a new one
        if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
            tile.header = Some(TileHeader::new(x, y, layer));
            tile.next = None; // No longer in free list
            // Clear any existing data
            tile.polys.clear();
            tile.verts.clear();
            tile.links.clear();
            tile.detail_meshes.clear();
            tile.detail_verts.clear();
            tile.detail_tris.clear();
            tile.off_mesh_connections.clear();
            tile.bvh_nodes.clear();
            tile.bvh_root = None;
            tile.flags = 0;
        } else {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Add to position lookup
        self.pos_lookup.insert((x, y, layer), tile_idx);

        Ok(())
    }

    /// Sets the flags for a specific polygon
    ///
    /// Updates the traversal flags for a polygon at runtime. This allows dynamic modification
    /// of navigation behavior without rebuilding the entire navigation mesh.
    ///
    /// # Arguments
    /// * `reference` - Valid polygon reference obtained from navigation queries
    /// * `flags` - New flags to set (e.g., WALKABLE, SWIM, DOOR)
    ///
    /// # Returns
    /// * `Ok(())` - Success
    /// * `Err(Error::Detour)` - Invalid polygon reference or tile not found
    ///
    /// # Examples
    /// ```
    /// # use recast_navigation::detour::{NavMesh, PolyFlags};
    /// # let mut nav_mesh = NavMesh::new();
    /// # let poly_ref = nav_mesh.get_poly_ref_base(); // Assuming this exists
    /// // Mark a polygon as non-walkable (e.g., for temporary obstacles)
    /// nav_mesh.set_poly_flags(poly_ref, PolyFlags::empty())?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    ///
    /// # Performance
    /// Time complexity: O(1)
    /// Space complexity: O(1)
    pub fn set_poly_flags(&mut self, reference: PolyRef, flags: PolyFlags) -> Result<()> {
        if !reference.is_valid() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        let (tile_id, poly_id) = decode_poly_ref(reference);
        let tile_idx = match tile_id_to_index(tile_id) {
            Some(idx) => idx,
            None => return Err(Error::Detour(Status::InvalidParam.to_string())),
        };

        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
            if poly_id as usize >= tile.polys.len() {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }
            tile.polys[poly_id as usize].flags = flags;
            Ok(())
        } else {
            Err(Error::Detour(Status::InvalidParam.to_string()))
        }
    }

    /// Gets the flags for a specific polygon
    ///
    /// Retrieves the current traversal flags for a polygon. These flags determine
    /// how navigation queries interact with the polygon.
    ///
    /// # Arguments
    /// * `reference` - Valid polygon reference obtained from navigation queries
    ///
    /// # Returns
    /// * `Ok(PolyFlags)` - Current flags for the polygon
    /// * `Err(Error::Detour)` - Invalid polygon reference or tile not found
    ///
    /// # Examples
    /// ```
    /// # use recast_navigation::detour::{NavMesh, PolyFlags};
    /// # let nav_mesh = NavMesh::new();
    /// # let poly_ref = nav_mesh.get_poly_ref_base(); // Assuming this exists
    /// let flags = nav_mesh.get_poly_flags(poly_ref)?;
    /// if flags.contains(PolyFlags::WALKABLE) {
    ///     println!("Polygon is walkable");
    /// }
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    ///
    /// # Performance
    /// Time complexity: O(1)
    /// Space complexity: O(1)
    pub fn get_poly_flags(&self, reference: PolyRef) -> Result<PolyFlags> {
        if !reference.is_valid() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        let (tile_id, poly_id) = decode_poly_ref(reference);
        let tile_idx = match tile_id_to_index(tile_id) {
            Some(idx) => idx,
            None => return Err(Error::Detour(Status::InvalidParam.to_string())),
        };

        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if let Some(Some(tile)) = self.tiles.get(tile_idx) {
            if poly_id as usize >= tile.polys.len() {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }
            Ok(tile.polys[poly_id as usize].flags)
        } else {
            Err(Error::Detour(Status::InvalidParam.to_string()))
        }
    }

    /// Sets the area for a specific polygon
    ///
    /// Updates the area type for a polygon at runtime. Area types are used for
    /// pathfinding cost calculation via QueryFilter implementations.
    ///
    /// # Arguments
    /// * `reference` - Valid polygon reference obtained from navigation queries
    /// * `area` - New area type ID (0-255, where 0 typically means unwalkable)
    ///
    /// # Returns
    /// * `Ok(())` - Success
    /// * `Err(Error::Detour)` - Invalid polygon reference or tile not found
    ///
    /// # Examples
    /// ```
    /// # use recast_navigation::detour::NavMesh;
    /// # let mut nav_mesh = NavMesh::new();
    /// # let poly_ref = nav_mesh.get_poly_ref_base(); // Assuming this exists
    /// // Set polygon to grass area type (higher movement cost)
    /// nav_mesh.set_poly_area(poly_ref, 2)?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    ///
    /// # Performance
    /// Time complexity: O(1)
    /// Space complexity: O(1)
    pub fn set_poly_area(&mut self, reference: PolyRef, area: u8) -> Result<()> {
        if !reference.is_valid() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        let (tile_id, poly_id) = decode_poly_ref(reference);
        let tile_idx = match tile_id_to_index(tile_id) {
            Some(idx) => idx,
            None => return Err(Error::Detour(Status::InvalidParam.to_string())),
        };

        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
            if poly_id as usize >= tile.polys.len() {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }
            tile.polys[poly_id as usize].area = area;
            Ok(())
        } else {
            Err(Error::Detour(Status::InvalidParam.to_string()))
        }
    }

    /// Gets the area for a specific polygon
    ///
    /// Retrieves the current area type for a polygon. Area types determine
    /// movement costs during pathfinding calculations.
    ///
    /// # Arguments
    /// * `reference` - Valid polygon reference obtained from navigation queries
    ///
    /// # Returns
    /// * `Ok(u8)` - Area type ID for the polygon (0-255)
    /// * `Err(Error::Detour)` - Invalid polygon reference or tile not found
    ///
    /// # Examples
    /// ```
    /// # use recast_navigation::detour::NavMesh;
    /// # let nav_mesh = NavMesh::new();
    /// # let poly_ref = nav_mesh.get_poly_ref_base(); // Assuming this exists
    /// let area = nav_mesh.get_poly_area(poly_ref)?;
    /// match area {
    ///     0 => println!("Unwalkable area"),
    ///     1 => println!("Ground area"),
    ///     2 => println!("Grass area"),
    ///     _ => println!("Custom area type: {}", area),
    /// }
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    ///
    /// # Performance
    /// Time complexity: O(1)
    /// Space complexity: O(1)
    pub fn get_poly_area(&self, reference: PolyRef) -> Result<u8> {
        if !reference.is_valid() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        let (tile_id, poly_id) = decode_poly_ref(reference);
        let tile_idx = match tile_id_to_index(tile_id) {
            Some(idx) => idx,
            None => return Err(Error::Detour(Status::InvalidParam.to_string())),
        };

        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if let Some(Some(tile)) = self.tiles.get(tile_idx) {
            if poly_id as usize >= tile.polys.len() {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }
            Ok(tile.polys[poly_id as usize].area)
        } else {
            Err(Error::Detour(Status::InvalidParam.to_string()))
        }
    }

    /// Extracts the salt value from a polygon reference
    pub fn decode_poly_ref_salt(&self, reference: PolyRef) -> u32 {
        decode_poly_id_salt(reference)
    }

    /// Extracts the tile index from a polygon reference
    pub fn decode_poly_ref_tile(&self, reference: PolyRef) -> u32 {
        decode_tile_id(reference)
    }

    /// Extracts the polygon index from a polygon reference
    pub fn decode_poly_ref_poly(&self, reference: PolyRef) -> u32 {
        decode_poly_id(reference)
    }

    /// Gets the endpoints of an off-mesh connection, ordered by the traversal direction
    ///
    /// Off-mesh connections are stored as special 2-vertex polygons. This method returns
    /// the endpoints in the correct order based on the previous polygon reference.
    pub fn get_off_mesh_connection_poly_end_points(
        &self,
        prev_ref: PolyRef,
        poly_ref: PolyRef,
    ) -> Result<([f32; 3], [f32; 3])> {
        if !poly_ref.is_valid() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Decode the polygon reference
        let (salt, tile_id, poly_id) = decode_poly_ref_full(poly_ref);
        let tile_idx =
            tile_id_to_index(tile_id).ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let tile = self
            .tiles
            .get(tile_idx)
            .and_then(|opt| opt.as_ref())
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Check salt matches
        if salt != (tile.salt & DT_SALT_MASK) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if poly_id as usize >= tile.polys.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let poly = &tile.polys[poly_id as usize];

        // Make sure this is an off-mesh connection polygon
        if poly.poly_type != PolyType::OffMeshConnection {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Off-mesh connections have exactly 2 vertices
        if poly.vert_count != 2 {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Figure out which way to hand out the vertices
        let mut idx0 = 0;
        let mut idx1 = 1;

        // Find link that points to first vertex
        if let Some(first_link) = poly.first_link {
            let mut link_idx = first_link;
            loop {
                if link_idx >= tile.links.len() {
                    break;
                }
                let link = &tile.links[link_idx];
                if link.edge_index == 0 {
                    if link.reference != prev_ref {
                        idx0 = 1;
                        idx1 = 0;
                    }
                    break;
                }
                if let Some(next) = link.next {
                    link_idx = next as usize;
                } else {
                    break;
                }
            }
        }

        // Get the vertex positions
        let v0_idx = poly.verts[idx0] as usize * 3;
        let v1_idx = poly.verts[idx1] as usize * 3;

        if v0_idx + 2 >= tile.verts.len() || v1_idx + 2 >= tile.verts.len() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        let start_pos = [
            tile.verts[v0_idx],
            tile.verts[v0_idx + 1],
            tile.verts[v0_idx + 2],
        ];
        let end_pos = [
            tile.verts[v1_idx],
            tile.verts[v1_idx + 1],
            tile.verts[v1_idx + 2],
        ];

        Ok((start_pos, end_pos))
    }

    /// Gets the off-mesh connection data associated with a polygon reference
    pub fn get_off_mesh_connection_by_ref(&self, reference: PolyRef) -> Result<&OffMeshConnection> {
        if !reference.is_valid() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Decode the polygon reference
        let (salt, tile_id, poly_id) = decode_poly_ref_full(reference);
        let tile_idx =
            tile_id_to_index(tile_id).ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let tile = self
            .tiles
            .get(tile_idx)
            .and_then(|opt| opt.as_ref())
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Check salt matches
        if salt != (tile.salt & DT_SALT_MASK) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if poly_id as usize >= tile.polys.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let poly = &tile.polys[poly_id as usize];

        // Make sure this is an off-mesh connection polygon
        if poly.poly_type != PolyType::OffMeshConnection {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Find the corresponding off-mesh connection
        // In the C++ version, off-mesh connections are stored after regular polygons
        // We need to search through the off_mesh_connections vector to find the one
        // with matching poly reference
        for connection in &tile.off_mesh_connections {
            if connection.poly == reference {
                return Ok(connection);
            }
        }

        Err(Error::Detour(Status::Failure.to_string()))
    }

    /// Gets the size in bytes required to store the tile state
    pub fn get_tile_state_size(&self, tile: &MeshTile) -> usize {
        // Size of TileState header
        let header_size = std::mem::size_of::<u32>() * 3; // magic, version, ref

        // Size of poly states (flags: u16, area: u8, aligned to 4 bytes)
        let poly_state_size = 4; // 2 + 1 + 1 padding
        let total_poly_size = poly_state_size * tile.polys.len();

        header_size + total_poly_size
    }

    /// Stores the non-structural tile state (polygon flags and areas)
    pub fn store_tile_state(&self, tile: &MeshTile, data: &mut [u8]) -> Result<usize> {
        const DT_NAVMESH_STATE_MAGIC: u32 = 0x5354544E; // 'STTN'
        const DT_NAVMESH_STATE_VERSION: u32 = 1;

        let required_size = self.get_tile_state_size(tile);
        if data.len() < required_size {
            return Err(Error::Detour(Status::BufferTooSmall.to_string()));
        }

        let mut offset = 0;

        // Write header
        // Magic number
        data[offset..offset + 4].copy_from_slice(&DT_NAVMESH_STATE_MAGIC.to_le_bytes());
        offset += 4;

        // Version
        data[offset..offset + 4].copy_from_slice(&DT_NAVMESH_STATE_VERSION.to_le_bytes());
        offset += 4;

        // Tile reference - we need to find the tile index
        let tile_ref = if let Some(tile_idx) = self
            .tiles
            .iter()
            .position(|t| t.as_ref().map(|mt| std::ptr::eq(mt, tile)).unwrap_or(false))
        {
            encode_poly_ref_with_salt(tile.salt & DT_SALT_MASK, (tile_idx + 1) as u32, 0)
        } else {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        };
        data[offset..offset + 4].copy_from_slice(&tile_ref.id().to_le_bytes());
        offset += 4;

        // Write poly states
        for poly in &tile.polys {
            // Flags (u16)
            data[offset..offset + 2].copy_from_slice(&poly.flags.bits().to_le_bytes());
            offset += 2;

            // Area (u8)
            data[offset] = poly.area;
            offset += 1;

            // Padding (u8)
            data[offset] = 0;
            offset += 1;
        }

        Ok(required_size)
    }

    /// Restores the non-structural tile state (polygon flags and areas)
    pub fn restore_tile_state(&mut self, tile: &mut MeshTile, data: &[u8]) -> Result<()> {
        const DT_NAVMESH_STATE_MAGIC: u32 = 0x5354544E; // 'STTN'
        const DT_NAVMESH_STATE_VERSION: u32 = 1;

        let required_size = self.get_tile_state_size(tile);
        if data.len() < required_size {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let mut offset = 0;

        // Read and verify header
        // Magic number
        let magic = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;
        if magic != DT_NAVMESH_STATE_MAGIC {
            return Err(Error::Detour(Status::WrongMagic.to_string()));
        }

        // Version
        let version = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;
        if version != DT_NAVMESH_STATE_VERSION {
            return Err(Error::Detour(Status::WrongVersion.to_string()));
        }

        // Tile reference - verify it matches
        let stored_ref = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        offset += 4;

        // Find current tile reference
        let current_ref = if let Some(tile_idx) = self
            .tiles
            .iter()
            .position(|t| t.as_ref().map(|mt| std::ptr::eq(mt, tile)).unwrap_or(false))
        {
            encode_poly_ref_with_salt(tile.salt & DT_SALT_MASK, (tile_idx + 1) as u32, 0).id()
        } else {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        };

        if stored_ref != current_ref {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Restore poly states
        for poly in &mut tile.polys {
            // Flags (u16)
            let flags = u16::from_le_bytes([data[offset], data[offset + 1]]);
            poly.flags = PolyFlags::from_bits(flags).unwrap_or(PolyFlags::empty());
            offset += 2;

            // Area (u8)
            poly.area = data[offset];
            offset += 1;

            // Skip padding (u8)
            offset += 1;
        }

        Ok(())
    }

    /// Calculates the tile grid location for the specified world position
    pub fn calc_tile_loc(&self, pos: &[f32; 3]) -> (i32, i32) {
        let tx = ((pos[0] - self.origin[0]) / self.tile_width).floor() as i32;
        let ty = ((pos[2] - self.origin[2]) / self.tile_height).floor() as i32;
        (tx, ty)
    }

    /// Gets all tiles at the specified grid location (all layers)
    pub fn get_tiles_at(&self, x: i32, y: i32, max_tiles: usize) -> Vec<&MeshTile> {
        let mut tiles = Vec::with_capacity(max_tiles.min(256));

        // Search all layers at this location
        for layer in 0..256 {
            if let Some(tile_idx) = self.pos_lookup.get(&(x, y, layer)) {
                if let Some(Some(tile)) = self.tiles.get(*tile_idx) {
                    tiles.push(tile);
                    if tiles.len() >= max_tiles {
                        break;
                    }
                }
            } else {
                // No more layers at this location
                break;
            }
        }

        tiles
    }

    /// Gets the tile reference at the specified grid location
    pub fn get_tile_ref_at(&self, x: i32, y: i32, layer: i32) -> Option<PolyRef> {
        if let Some(&tile_idx) = self.pos_lookup.get(&(x, y, layer)) {
            if let Some(Some(tile)) = self.tiles.get(tile_idx) {
                // Return a reference to the first polygon in the tile
                return Some(encode_poly_ref_with_salt(
                    tile.salt & DT_SALT_MASK,
                    (tile_idx + 1) as u32,
                    0,
                ));
            }
        }
        None
    }

    /// Gets the reference for a tile
    pub fn get_tile_ref(&self, tile: &MeshTile) -> Option<PolyRef> {
        // Find the tile index by comparing pointers
        self.tiles
            .iter()
            .position(|t| t.as_ref().map(|mt| std::ptr::eq(mt, tile)).unwrap_or(false))
            .map(|tile_idx| {
                encode_poly_ref_with_salt(tile.salt & DT_SALT_MASK, (tile_idx + 1) as u32, 0)
            })
    }

    /// Gets a tile by its reference
    pub fn get_tile_by_ref(&self, reference: PolyRef) -> Option<&MeshTile> {
        if !reference.is_valid() {
            return None;
        }

        let (salt, tile_id, _) = decode_poly_ref_full(reference);
        let tile_idx = tile_id_to_index(tile_id)?;

        if tile_idx >= self.tiles.len() {
            return None;
        }

        self.tiles
            .get(tile_idx)
            .and_then(|opt| opt.as_ref())
            .filter(|tile| salt == (tile.salt & DT_SALT_MASK))
    }

    /// Gets a tile by index
    pub fn get_tile(&self, index: usize) -> Option<&MeshTile> {
        if index >= self.tiles.len() {
            return None;
        }

        self.tiles.get(index).and_then(|opt| opt.as_ref())
    }

    /// Gets the PolyRef for a specific polygon in a tile
    pub fn get_poly_ref(&self, tile: &MeshTile, poly: &Poly) -> Result<PolyRef> {
        // Find the base reference for the tile
        let base_ref = self.get_poly_ref_base(tile);
        if base_ref == 0 {
            return Err(Error::Detour(
                "Failed to get tile base reference".to_string(),
            ));
        }

        // Find the polygon index within the tile
        let poly_idx = tile
            .polys
            .iter()
            .position(|p| std::ptr::eq(p, poly))
            .ok_or_else(|| Error::Detour("Polygon not found in tile".to_string()))?;

        // Combine base ref with polygon index
        Ok(PolyRef::new(base_ref | (poly_idx as u32)))
    }

    /// Applies a collection of external link requests to the navigation mesh
    /// This is the deferred link creation phase that follows the idiomatic Rust pattern
    pub fn apply_external_links(
        &mut self,
        requests: Vec<crate::ExternalLinkRequest>,
    ) -> Result<()> {
        for request in requests {
            self.create_external_link(request)?;
        }
        Ok(())
    }

    /// Connects off-mesh connections to ground polygons in a tile
    fn connect_off_mesh_connections_for_tile(
        &mut self,
        tile_idx: usize,
        actual_tile_id: u32,
        params: &super::NavMeshCreateParams,
    ) -> Result<()> {
        // Get tile data first
        let tile_data = {
            let tile = self
                .tiles
                .get(tile_idx)
                .and_then(|t| t.as_ref())
                .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

            let mut connections = Vec::new();
            for off_mesh_con in &tile.off_mesh_connections {
                connections.push((
                    off_mesh_con.poly,
                    off_mesh_con.start_pos(),
                    off_mesh_con.end_pos(),
                    off_mesh_con.radius,
                ));
            }

            let mut polys_to_check = Vec::new();
            for (i, poly) in tile.polys.iter().enumerate() {
                if poly.poly_type == PolyType::Ground {
                    // Get polygon bounds
                    let mut poly_verts = Vec::new();
                    for j in 0..poly.vert_count as usize {
                        let v_idx = poly.verts[j] as usize * 3;
                        if v_idx + 2 < tile.verts.len() {
                            poly_verts.push([
                                tile.verts[v_idx],
                                tile.verts[v_idx + 1],
                                tile.verts[v_idx + 2],
                            ]);
                        }
                    }
                    polys_to_check.push((i, poly_verts));
                }
            }

            (connections, polys_to_check)
        };

        // Now check which ground polygons should connect to off-mesh connections
        // And create bidirectional links
        for (con_idx, (con_poly_ref, start_pos, end_pos, radius)) in tile_data.0.iter().enumerate()
        {
            let mut start_poly_idx = None;
            let mut end_poly_idx = None;

            // Find ground polygons near start and end positions
            for (ground_poly_idx, poly_verts) in &tile_data.1 {
                let mut near_start = false;
                let mut near_end = false;

                // Check if any vertex of the ground polygon is within radius of start or end
                for vert in poly_verts.iter() {
                    let dist_to_start = ((vert[0] - start_pos[0]).powi(2)
                        + (vert[1] - start_pos[1]).powi(2)
                        + (vert[2] - start_pos[2]).powi(2))
                    .sqrt();
                    let dist_to_end = ((vert[0] - end_pos[0]).powi(2)
                        + (vert[1] - end_pos[1]).powi(2)
                        + (vert[2] - end_pos[2]).powi(2))
                    .sqrt();

                    if dist_to_start <= *radius {
                        near_start = true;
                    }
                    if dist_to_end <= *radius {
                        near_end = true;
                    }
                }

                if near_start && start_poly_idx.is_none() {
                    start_poly_idx = Some(*ground_poly_idx);
                }
                if near_end && end_poly_idx.is_none() {
                    end_poly_idx = Some(*ground_poly_idx);
                }
            }

            // Get the off-mesh connection polygon index (it's after all ground polygons)
            let off_mesh_poly_idx = params.poly_count as usize + con_idx;

            // CRITICAL: Create links for off-mesh connection pathfinding
            // The off-mesh connection polygon needs links to BOTH start and end ground polygons

            // Link from off-mesh connection to START polygon (for knowing where it starts)
            if let Some(start_idx) = start_poly_idx {
                let start_poly_ref = encode_poly_ref_with_salt(
                    self.tiles[tile_idx].as_ref().unwrap().salt & DT_SALT_MASK,
                    actual_tile_id,
                    start_idx as u32,
                );
                self.add_link_to_polygon(tile_idx, off_mesh_poly_idx, start_poly_ref, 0, 0xFF, 0)?;
            }

            // IMPORTANT: Link from off-mesh connection to END polygon (for pathfinding continuation)
            if let Some(end_idx) = end_poly_idx {
                let end_poly_ref = encode_poly_ref_with_salt(
                    self.tiles[tile_idx].as_ref().unwrap().salt & DT_SALT_MASK,
                    actual_tile_id,
                    end_idx as u32,
                );
                self.add_link_to_polygon(tile_idx, off_mesh_poly_idx, end_poly_ref, 1, 0xFF, 0)?;
            }

            // Create links FROM ground polygons TO off-mesh connection
            if let Some(start_idx) = start_poly_idx {
                // Link from start ground polygon to off-mesh connection
                self.add_link_to_polygon(tile_idx, start_idx, *con_poly_ref, 0xFF, 0xFF, 0)?;
            }

            if let Some(end_idx) = end_poly_idx {
                // For bidirectional connections, also link from end ground polygon to off-mesh connection
                if let Some(Some(tile)) = self.tiles.get(tile_idx) {
                    if con_idx < tile.off_mesh_connections.len() {
                        let connection = &tile.off_mesh_connections[con_idx];
                        if connection.allows_end_to_start() {
                            self.add_link_to_polygon(
                                tile_idx,
                                end_idx,
                                *con_poly_ref,
                                0xFF,
                                0xFF,
                                0,
                            )?;
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Creates a single external link between tiles following existing patterns
    fn create_external_link(&mut self, request: crate::ExternalLinkRequest) -> Result<()> {
        let (tx, ty, layer) = request.source_tile;

        // Find the tile index using the position lookup
        let tile_idx = self
            .pos_lookup
            .get(&(tx, ty, layer))
            .copied()
            .ok_or_else(|| Error::Detour("Source tile not found".to_string()))?;

        // Get mutable access to the source tile using the established pattern
        if let Some(Some(tile)) = self.tiles.get_mut(tile_idx) {
            // Check if polygon index is valid
            if request.source_poly >= tile.polys.len() {
                return Err(Error::Detour("Invalid source polygon index".to_string()));
            }

            // Create the external link
            let link = Link::new(
                request.target_poly_ref,
                0, // neighbor_index (not used for external links)
                request.source_edge,
                request.side,
                1, // boundary_flag: 1 = external link
            );

            // Add link to the tile's link vector
            let link_idx = tile.links.len();
            tile.links.push(link);

            // Update the polygon's link chain using the established pattern
            let poly = &mut tile.polys[request.source_poly];
            if poly.first_link.is_none() {
                poly.first_link = Some(link_idx);
            } else {
                // Find the last link in the chain and append
                let mut current_idx = poly.first_link.unwrap();
                while let Some(next_idx) = tile.links[current_idx].next {
                    current_idx = next_idx as usize;
                }
                tile.links[current_idx].next = Some(link_idx as u32);
            }
        } else {
            return Err(Error::Detour("Source tile not accessible".to_string()));
        }

        Ok(())
    }
}

/// Custom serializer for pos_lookup HashMap with tuple keys
#[cfg(feature = "serialization")]
fn serialize_pos_lookup<S>(
    pos_lookup: &HashMap<(i32, i32, i32), usize>,
    serializer: S,
) -> std::result::Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    use serde::ser::SerializeMap;
    let mut map = serializer.serialize_map(Some(pos_lookup.len()))?;
    for ((x, y, layer), value) in pos_lookup {
        let key = format!("{},{},{}", x, y, layer);
        map.serialize_entry(&key, value)?;
    }
    map.end()
}

/// Type alias for position lookup map
#[allow(dead_code)]
type PosLookupMap = HashMap<(i32, i32, i32), usize>;

/// Custom deserializer for pos_lookup HashMap with tuple keys
#[cfg(feature = "serialization")]
fn deserialize_pos_lookup<'de, D>(deserializer: D) -> std::result::Result<PosLookupMap, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::de::{self, MapAccess, Visitor};
    use std::fmt;

    struct PosLookupVisitor;

    impl<'de> Visitor<'de> for PosLookupVisitor {
        type Value = HashMap<(i32, i32, i32), usize>;

        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_str("a map with string keys in format 'x,y,layer'")
        }

        fn visit_map<V>(self, mut map: V) -> std::result::Result<Self::Value, V::Error>
        where
            V: MapAccess<'de>,
        {
            let mut result = HashMap::new();

            while let Some(key) = map.next_key::<String>()? {
                let value = map.next_value::<usize>()?;

                let parts: Vec<&str> = key.split(',').collect();
                if parts.len() != 3 {
                    return Err(de::Error::invalid_value(
                        de::Unexpected::Str(&key),
                        &"string in format 'x,y,layer'",
                    ));
                }

                let x = parts[0].parse::<i32>().map_err(|_| {
                    de::Error::invalid_value(de::Unexpected::Str(parts[0]), &"integer")
                })?;
                let y = parts[1].parse::<i32>().map_err(|_| {
                    de::Error::invalid_value(de::Unexpected::Str(parts[1]), &"integer")
                })?;
                let layer = parts[2].parse::<i32>().map_err(|_| {
                    de::Error::invalid_value(de::Unexpected::Str(parts[2]), &"integer")
                })?;

                result.insert((x, y, layer), value);
            }

            Ok(result)
        }
    }

    deserializer.deserialize_map(PosLookupVisitor)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_poly_ref_encoding() {
        let tile_id = 42;
        let poly_id = 123;

        let reference = encode_poly_ref(tile_id, poly_id);

        assert_eq!(decode_tile_id(reference), tile_id);
        assert_eq!(decode_poly_id(reference), poly_id);
    }

    #[test]
    fn test_nav_mesh_creation() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params.clone());

        assert!(nav_mesh.is_ok());

        let nav_mesh = nav_mesh.unwrap();

        assert_eq!(nav_mesh.get_max_tiles(), params.max_tiles);
        assert_eq!(nav_mesh.get_max_polys_per_tile(), params.max_polys_per_tile);
        assert_eq!(nav_mesh.get_origin(), params.origin);
        assert_eq!(nav_mesh.get_tile_width(), params.tile_width);
        assert_eq!(nav_mesh.get_tile_height(), params.tile_height);
    }

    #[test]
    fn test_invalid_params() {
        // Test invalid origin
        let params = NavMeshParams {
            origin: [f32::INFINITY, 0.0, 0.0], // Use INFINITY directly
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params);
        assert!(nav_mesh.is_err());

        // Test invalid tile size
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: -1.0, // Negative tile width
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params);
        assert!(nav_mesh.is_err());

        // Test invalid max_tiles
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 0, // Zero max_tiles
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params);
        assert!(nav_mesh.is_err());

        // Test invalid max_polys_per_tile
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 0, // Zero max_polys_per_tile
        };

        let nav_mesh = NavMesh::new(params);
        assert!(nav_mesh.is_err());
    }

    #[test]
    #[cfg(feature = "serialization")]
    fn test_navmesh_serialization_json() {
        let params = NavMeshParams {
            origin: [1.0, 2.0, 3.0],
            tile_width: 50.0,
            tile_height: 50.0,
            max_tiles: 64,
            max_polys_per_tile: 512,
        };

        let nav_mesh = NavMesh::new(params.clone()).unwrap();

        // Test JSON serialization
        let json_bytes = nav_mesh.to_json_bytes().unwrap();
        let restored_nav_mesh = NavMesh::from_json_bytes(&json_bytes).unwrap();

        // Verify that the restored nav mesh has the same parameters
        assert_eq!(restored_nav_mesh.get_origin(), params.origin);
        assert_eq!(restored_nav_mesh.get_tile_width(), params.tile_width);
        assert_eq!(restored_nav_mesh.get_tile_height(), params.tile_height);
        assert_eq!(restored_nav_mesh.get_max_tiles(), params.max_tiles);
        assert_eq!(
            restored_nav_mesh.get_max_polys_per_tile(),
            params.max_polys_per_tile
        );
    }

    #[test]
    #[cfg(feature = "serialization")]
    fn test_navmesh_serialization_binary() {
        let params = NavMeshParams {
            origin: [1.0, 2.0, 3.0],
            tile_width: 50.0,
            tile_height: 50.0,
            max_tiles: 64,
            max_polys_per_tile: 512,
        };

        let nav_mesh = NavMesh::new(params.clone()).unwrap();

        // Test binary serialization
        let binary_bytes = nav_mesh.to_binary_bytes().unwrap();
        let restored_nav_mesh = NavMesh::from_binary_bytes(&binary_bytes).unwrap();

        // Verify that the restored nav mesh has the same parameters
        assert_eq!(restored_nav_mesh.get_origin(), params.origin);
        assert_eq!(restored_nav_mesh.get_tile_width(), params.tile_width);
        assert_eq!(restored_nav_mesh.get_tile_height(), params.tile_height);
        assert_eq!(restored_nav_mesh.get_max_tiles(), params.max_tiles);
        assert_eq!(
            restored_nav_mesh.get_max_polys_per_tile(),
            params.max_polys_per_tile
        );
    }

    #[test]
    #[cfg(feature = "serialization")]
    fn test_navmesh_file_serialization() {
        use tempfile::NamedTempFile;

        let params = NavMeshParams {
            origin: [1.0, 2.0, 3.0],
            tile_width: 50.0,
            tile_height: 50.0,
            max_tiles: 64,
            max_polys_per_tile: 512,
        };

        let nav_mesh = NavMesh::new(params.clone()).unwrap();

        // Test JSON file serialization
        let json_file = NamedTempFile::new().unwrap();
        nav_mesh.save_to_json(json_file.path()).unwrap();
        let restored_from_json = NavMesh::load_from_json(json_file.path()).unwrap();

        assert_eq!(restored_from_json.get_origin(), params.origin);
        assert_eq!(restored_from_json.get_max_tiles(), params.max_tiles);

        // Test binary file serialization
        let binary_file = NamedTempFile::new().unwrap();
        nav_mesh.save_to_binary(binary_file.path()).unwrap();
        let restored_from_binary = NavMesh::load_from_binary(binary_file.path()).unwrap();

        assert_eq!(restored_from_binary.get_origin(), params.origin);
        assert_eq!(restored_from_binary.get_max_tiles(), params.max_tiles);
    }

    #[test]
    fn test_off_mesh_connections() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create a simple tile
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Add an off-mesh connection
        let connection_ref = nav_mesh
            .add_off_mesh_connection(
                [10.0, 0.0, 10.0], // start
                [20.0, 0.0, 20.0], // end
                2.0,               // radius
                PolyFlags::WALK,   // flags
                1,                 // area
                0,                 // bidirectional
                42,                // user_id
            )
            .unwrap();

        // Verify the connection was added
        assert!(connection_ref.is_valid());
        assert!(nav_mesh.is_off_mesh_connection(connection_ref));

        // Get the connection and verify its properties
        let connection = nav_mesh.get_off_mesh_connection(connection_ref).unwrap();
        assert_eq!(connection.start_pos(), [10.0, 0.0, 10.0]);
        assert_eq!(connection.end_pos(), [20.0, 0.0, 20.0]);
        assert_eq!(connection.radius, 2.0);
        assert_eq!(connection.area, 1);
        assert_eq!(connection.user_id, 42);
        assert!(connection.is_bidirectional());
        assert!(connection.allows_start_to_end());
        assert!(connection.allows_end_to_start());

        // Test connection length
        let expected_length = (10.0_f32 * 10.0 + 0.0 * 0.0 + 10.0 * 10.0).sqrt();
        assert!((connection.get_length() - expected_length).abs() < 1e-6);

        // Test finding connections near a position
        let nearby = nav_mesh.find_off_mesh_connections_near([15.0, 0.0, 15.0], 6.0);
        assert_eq!(nearby.len(), 1);

        let far_away = nav_mesh.find_off_mesh_connections_near([100.0, 0.0, 100.0], 5.0);
        assert_eq!(far_away.len(), 0);

        // Test getting all connections
        let all_connections = nav_mesh.get_all_off_mesh_connections();
        assert_eq!(all_connections.len(), 1);

        // Remove the connection
        nav_mesh.remove_off_mesh_connection(connection_ref).unwrap();

        // Verify it was removed
        assert!(nav_mesh.get_off_mesh_connection(connection_ref).is_err());
        assert_eq!(nav_mesh.get_all_off_mesh_connections().len(), 0);
    }

    #[test]
    fn test_off_mesh_connection_directions() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create a simple tile
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Test unidirectional A->B
        let a_to_b_ref = nav_mesh
            .add_off_mesh_connection(
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                1.0,
                PolyFlags::WALK,
                1,
                1,
                1,
            )
            .unwrap();

        let a_to_b = nav_mesh.get_off_mesh_connection(a_to_b_ref).unwrap();
        assert!(!a_to_b.is_bidirectional());
        assert!(a_to_b.allows_start_to_end());
        assert!(!a_to_b.allows_end_to_start());

        // Test unidirectional B->A
        let b_to_a_ref = nav_mesh
            .add_off_mesh_connection(
                [0.0, 0.0, 10.0],
                [10.0, 0.0, 10.0],
                1.0,
                PolyFlags::WALK,
                1,
                2,
                2,
            )
            .unwrap();

        let b_to_a = nav_mesh.get_off_mesh_connection(b_to_a_ref).unwrap();
        assert!(!b_to_a.is_bidirectional());
        assert!(!b_to_a.allows_start_to_end());
        assert!(b_to_a.allows_end_to_start());

        // Test bidirectional
        let bidir_ref = nav_mesh
            .add_off_mesh_connection(
                [0.0, 0.0, 20.0],
                [10.0, 0.0, 20.0],
                1.0,
                PolyFlags::WALK,
                1,
                0,
                3,
            )
            .unwrap();

        let bidir = nav_mesh.get_off_mesh_connection(bidir_ref).unwrap();
        assert!(bidir.is_bidirectional());
        assert!(bidir.allows_start_to_end());
        assert!(bidir.allows_end_to_start());
    }

    #[test]
    #[cfg(feature = "serialization")]
    fn test_off_mesh_connection_serialization() {
        use tempfile::NamedTempFile;

        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create a simple tile
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Add an off-mesh connection
        let _connection_ref = nav_mesh
            .add_off_mesh_connection(
                [10.0, 0.0, 10.0],
                [20.0, 0.0, 20.0],
                2.0,
                PolyFlags::WALK,
                1,
                0,
                42,
            )
            .unwrap();

        // Test binary serialization first (more robust with floating point values)
        let binary_bytes = nav_mesh.to_binary_bytes().unwrap();
        let restored_nav_mesh = NavMesh::from_binary_bytes(&binary_bytes).unwrap();
        assert_eq!(restored_nav_mesh.get_all_off_mesh_connections().len(), 1);

        // Test JSON serialization with better error reporting
        println!("Testing JSON serialization...");
        match nav_mesh.to_json_bytes() {
            Ok(json_bytes) => {
                println!("JSON serialization successful, {} bytes", json_bytes.len());
                match NavMesh::from_json_bytes(&json_bytes) {
                    Ok(restored_nav_mesh) => {
                        assert_eq!(restored_nav_mesh.get_all_off_mesh_connections().len(), 1);
                        println!("JSON round-trip successful");
                    }
                    Err(e) => {
                        println!("JSON deserialization failed: {:?}", e);
                        panic!("JSON deserialization should work now");
                    }
                }
            }
            Err(e) => {
                println!("JSON serialization failed: {:?}", e);
                panic!("JSON serialization should work now");
            }
        }

        // Test JSON file serialization
        let json_file = NamedTempFile::new().unwrap();
        nav_mesh.save_to_json(json_file.path()).unwrap();
        let restored_from_json_file = NavMesh::load_from_json(json_file.path()).unwrap();
        assert_eq!(
            restored_from_json_file.get_all_off_mesh_connections().len(),
            1
        );
        println!("JSON file serialization successful");

        let binary_file = NamedTempFile::new().unwrap();
        nav_mesh.save_to_binary(binary_file.path()).unwrap();
        let restored_from_binary_file = NavMesh::load_from_binary(binary_file.path()).unwrap();
        assert_eq!(
            restored_from_binary_file
                .get_all_off_mesh_connections()
                .len(),
            1
        );

        // Verify the connection properties are preserved
        let original_connections = nav_mesh.get_all_off_mesh_connections();
        let restored_connections = restored_from_binary_file.get_all_off_mesh_connections();

        assert_eq!(
            original_connections[0].start_pos(),
            restored_connections[0].start_pos()
        );
        assert_eq!(
            original_connections[0].end_pos(),
            restored_connections[0].end_pos()
        );
        assert_eq!(
            original_connections[0].radius,
            restored_connections[0].radius
        );
        assert_eq!(original_connections[0].area, restored_connections[0].area);
        assert_eq!(
            original_connections[0].user_id,
            restored_connections[0].user_id
        );
    }

    #[test]
    fn test_point_in_polygon_robust() {
        // Test with a simple square polygon
        let square_verts = vec![
            0.0, 0.0, 0.0, // vertex 0
            10.0, 0.0, 0.0, // vertex 1
            10.0, 0.0, 10.0, // vertex 2
            0.0, 0.0, 10.0, // vertex 3
        ];

        // Point inside the square
        assert!(NavMesh::point_in_polygon_robust(
            &[5.0, 0.0, 5.0],
            &square_verts,
            4
        ));

        // Point outside the square
        assert!(!NavMesh::point_in_polygon_robust(
            &[15.0, 0.0, 5.0],
            &square_verts,
            4
        ));
        assert!(!NavMesh::point_in_polygon_robust(
            &[-5.0, 0.0, 5.0],
            &square_verts,
            4
        ));
        assert!(!NavMesh::point_in_polygon_robust(
            &[5.0, 0.0, 15.0],
            &square_verts,
            4
        ));
        assert!(!NavMesh::point_in_polygon_robust(
            &[5.0, 0.0, -5.0],
            &square_verts,
            4
        ));

        // Points near the boundary (edge cases - ray casting can be sensitive to exact boundaries)
        assert!(NavMesh::point_in_polygon_robust(
            &[0.1, 0.0, 5.0],
            &square_verts,
            4
        )); // near left edge
        assert!(NavMesh::point_in_polygon_robust(
            &[5.0, 0.0, 0.1],
            &square_verts,
            4
        )); // near bottom edge

        // Points exactly on boundary may not be reliably inside due to floating point precision
        // This is consistent with many ray-casting implementations

        // Test with a triangle
        let triangle_verts = vec![
            0.0, 0.0, 0.0, // vertex 0
            10.0, 0.0, 0.0, // vertex 1
            5.0, 0.0, 10.0, // vertex 2
        ];

        // Point inside the triangle
        assert!(NavMesh::point_in_polygon_robust(
            &[5.0, 0.0, 3.0],
            &triangle_verts,
            3
        ));

        // Point outside the triangle
        assert!(!NavMesh::point_in_polygon_robust(
            &[15.0, 0.0, 3.0],
            &triangle_verts,
            3
        ));
        assert!(!NavMesh::point_in_polygon_robust(
            &[5.0, 0.0, 15.0],
            &triangle_verts,
            3
        ));
    }

    #[test]
    fn test_closest_point_on_segment_3d() {
        // Test horizontal segment
        let start = [0.0, 0.0, 0.0];
        let end = [10.0, 0.0, 0.0];

        // Point on the segment
        let point = [5.0, 5.0, 0.0];
        let closest = NavMesh::closest_point_on_segment_3d(&point, &start, &end);
        assert_eq!(closest, [5.0, 0.0, 0.0]);

        // Point before the segment
        let point = [-5.0, 0.0, 0.0];
        let closest = NavMesh::closest_point_on_segment_3d(&point, &start, &end);
        assert_eq!(closest, [0.0, 0.0, 0.0]);

        // Point after the segment
        let point = [15.0, 0.0, 0.0];
        let closest = NavMesh::closest_point_on_segment_3d(&point, &start, &end);
        assert_eq!(closest, [10.0, 0.0, 0.0]);

        // Degenerate segment
        let start = [5.0, 5.0, 5.0];
        let end = [5.0, 5.0, 5.0];
        let point = [10.0, 10.0, 10.0];
        let closest = NavMesh::closest_point_on_segment_3d(&point, &start, &end);
        assert_eq!(closest, [5.0, 5.0, 5.0]);
    }

    #[test]
    fn test_interpolate_polygon_height() {
        // Test with a flat polygon
        let flat_verts = vec![
            0.0, 5.0, 0.0, // All vertices at height 5.0
            10.0, 5.0, 0.0, 10.0, 5.0, 10.0, 0.0, 5.0, 10.0,
        ];

        let height = NavMesh::interpolate_polygon_height(&flat_verts, &[5.0, 0.0, 5.0], 4);
        assert!((height - 5.0).abs() < 1e-6);

        // Test with sloped polygon
        let sloped_verts = vec![
            0.0, 0.0, 0.0, // Height 0.0
            10.0, 10.0, 0.0, // Height 10.0
            10.0, 20.0, 10.0, // Height 20.0
            0.0, 15.0, 10.0, // Height 15.0
        ];

        let height = NavMesh::interpolate_polygon_height(&sloped_verts, &[5.0, 0.0, 5.0], 4);
        let expected_height = (0.0 + 10.0 + 20.0 + 15.0) / 4.0; // Average = 11.25
        assert!((height - expected_height).abs() < 1e-6);

        // Test with empty polygon
        let empty_verts = vec![];
        let height = NavMesh::interpolate_polygon_height(&empty_verts, &[5.0, 7.0, 5.0], 0);
        assert!((height - 7.0).abs() < 1e-6); // Should return point's Y coordinate
    }

    #[test]
    fn test_closest_point_on_polygon_boundary() {
        // Test with a simple square
        let square_verts = vec![
            0.0, 0.0, 0.0, // vertex 0
            10.0, 0.0, 0.0, // vertex 1
            10.0, 0.0, 10.0, // vertex 2
            0.0, 0.0, 10.0, // vertex 3
        ];

        // Point outside, closest to bottom edge
        let point = [5.0, 5.0, -5.0];
        let closest = NavMesh::closest_point_on_polygon_boundary(&point, &square_verts, 4);
        assert!((closest[0] - 5.0).abs() < 1e-6);
        assert!((closest[1] - 0.0).abs() < 1e-6);
        assert!((closest[2] - 0.0).abs() < 1e-6);

        // Point outside, closest to right edge
        let point = [15.0, 0.0, 5.0];
        let closest = NavMesh::closest_point_on_polygon_boundary(&point, &square_verts, 4);
        assert!((closest[0] - 10.0).abs() < 1e-6);
        assert!((closest[1] - 0.0).abs() < 1e-6);
        assert!((closest[2] - 5.0).abs() < 1e-6);

        // Point outside, closest to corner
        let point = [15.0, 0.0, 15.0];
        let closest = NavMesh::closest_point_on_polygon_boundary(&point, &square_verts, 4);
        assert!((closest[0] - 10.0).abs() < 1e-6);
        assert!((closest[1] - 0.0).abs() < 1e-6);
        assert!((closest[2] - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_link_management_system() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create a test tile and add polygons
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            let mut poly1 = Poly::new(1, PolyType::Ground, PolyFlags::WALK);
            let mut poly2 = Poly::new(2, PolyType::Ground, PolyFlags::WALK);
            poly1.vert_count = 4; // Square polygon
            poly2.vert_count = 4; // Square polygon
            tile.polys.push(poly1);
            tile.polys.push(poly2);
        }

        // Test 1: Initially, polygons should have no links
        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 0);
            assert!(nav_mesh.get_polygon_links(tile, poly).is_empty());
            assert!(nav_mesh.find_link(tile, poly, 0).is_none());
        }

        // Test 2: Add a link to polygon 0
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            let link1 = Link::new(
                PolyRef::new(123), // reference to neighbor
                1,                 // neighbor_index
                0,                 // edge_index
                1,                 // side
                0,                 // boundary_flag
            );
            NavMesh::add_link_to_tile_polygon(tile, 0, link1).unwrap();
        }

        // Verify the link was added
        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 1);
            let links = nav_mesh.get_polygon_links(tile, poly);
            assert_eq!(links.len(), 1);
            assert_eq!(links[0].reference.id(), 123);
            assert_eq!(links[0].edge_index, 0);

            // Test 3: Find link by edge
            let found_link = nav_mesh.find_link(tile, poly, 0);
            assert!(found_link.is_some());
            let found_link = found_link.unwrap();
            assert_eq!(found_link.reference.id(), 123);
            assert_eq!(found_link.edge_index, 0);

            // Test 4: Find non-existent link
            assert!(nav_mesh.find_link(tile, poly, 1).is_none());
        }

        // Test 5: Add a second link to the same polygon
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            let link2 = Link::new(
                PolyRef::new(456), // reference to neighbor
                2,                 // neighbor_index
                1,                 // edge_index
                2,                 // side
                1,                 // boundary_flag
            );
            NavMesh::add_link_to_tile_polygon(tile, 0, link2).unwrap();
        }

        // Verify both links exist
        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 2);
            let links = nav_mesh.get_polygon_links(tile, poly);
            assert_eq!(links.len(), 2);

            // The newer link should be first (inserted at head of list)
            assert_eq!(links[0].reference.id(), 456);
            assert_eq!(links[0].edge_index, 1);
            assert_eq!(links[1].reference.id(), 123);
            assert_eq!(links[1].edge_index, 0);

            // Test 6: Find both links by their edges
            let link_edge_0 = nav_mesh.find_link(tile, poly, 0);
            assert!(link_edge_0.is_some());
            assert_eq!(link_edge_0.unwrap().reference.id(), 123);

            let link_edge_1 = nav_mesh.find_link(tile, poly, 1);
            assert!(link_edge_1.is_some());
            assert_eq!(link_edge_1.unwrap().reference.id(), 456);
        }

        // Test 7: Add a third link
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            let link3 = Link::new(PolyRef::new(789), 3, 2, 3, 0);
            NavMesh::add_link_to_tile_polygon(tile, 0, link3).unwrap();
        }

        // Verify all three links
        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 3);
            let links = nav_mesh.get_polygon_links(tile, poly);
            assert_eq!(links.len(), 3);

            // Test proper linked list order (most recent first)
            assert_eq!(links[0].reference.id(), 789); // Most recent
            assert_eq!(links[1].reference.id(), 456); // Middle
            assert_eq!(links[2].reference.id(), 123); // First added
        }

        // Test 8: Clear all links
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            NavMesh::clear_polygon_links(tile, 0).unwrap();
        }

        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 0);
            assert!(nav_mesh.get_polygon_links(tile, poly).is_empty());
            assert!(nav_mesh.find_link(tile, poly, 0).is_none());
            assert!(nav_mesh.find_link(tile, poly, 1).is_none());
            assert!(nav_mesh.find_link(tile, poly, 2).is_none());
        }

        // Test 9: Test error handling - invalid polygon index
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            let invalid_link = Link::new(PolyRef::new(999), 0, 0, 0, 0);
            assert!(NavMesh::add_link_to_tile_polygon(tile, 999, invalid_link).is_err());
            assert!(NavMesh::clear_polygon_links(tile, 999).is_err());
        }
    }

    #[test]
    fn test_link_management_edge_cases() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Create a test polygon with high vertex count
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            let mut poly = Poly::new(1, PolyType::Ground, PolyFlags::WALK);
            poly.vert_count = 6; // Hexagon
            tile.polys.push(poly);
        }

        // Test 1: Find link with edge index equal to vertex count (should return None)
        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert!(nav_mesh.find_link(tile, poly, 6).is_none()); // Edge index == vert_count
            assert!(nav_mesh.find_link(tile, poly, 7).is_none()); // Edge index > vert_count
        }

        // Test 2: Add links for all edges of the polygon
        for edge in 0..6 {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            let link = Link::new(
                PolyRef::new(100 + edge as u32),
                edge as u8,
                edge as u8,
                1,
                0,
            );
            NavMesh::add_link_to_tile_polygon(tile, 0, link).unwrap();
        }

        // Verify all links can be found
        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 6);

            for edge in 0..6 {
                let found_link = nav_mesh.find_link(tile, poly, edge as u8);
                assert!(found_link.is_some());
                assert_eq!(found_link.unwrap().reference.id(), 100 + edge as u32);
                assert_eq!(found_link.unwrap().edge_index, edge as u8);
            }
        }

        // Test 3: Test with empty tile (no polygons)
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            tile.polys.clear();

            // Should not crash when accessing empty polygon array
            let empty_link = Link::new(PolyRef::new(1), 0, 0, 0, 0);
            assert!(NavMesh::add_link_to_tile_polygon(tile, 0, empty_link).is_err());
        }
    }

    #[test]
    fn test_link_traversal_robustness() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Set up the test scenario
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();

            // Create test polygon
            let mut poly = Poly::new(1, PolyType::Ground, PolyFlags::WALK);
            poly.vert_count = 4; // Square polygon
            tile.polys.push(poly);

            // Manually create a chain of links to test traversal
            // Add links in a specific order to test the linked list behavior
            let link1 = Link::new_with_next(PolyRef::new(100), 0, 0, 0, 0, None);
            let link2 = Link::new_with_next(PolyRef::new(200), 1, 1, 1, 0, None);
            let link3 = Link::new_with_next(PolyRef::new(300), 2, 2, 2, 0, None);

            // Add to tile links array and set up linked list manually
            tile.links.push(link1); // index 0
            tile.links.push(link2); // index 1
            tile.links.push(link3); // index 2

            // Set up linked list: 2 -> 1 -> 0 -> None
            tile.links[2].next = Some(1);
            tile.links[1].next = Some(0);
            tile.links[0].next = None;

            // Set polygon's first link to point to the head of the chain
            tile.polys[0].first_link = Some(2);
        }

        // Test traversal finds all links
        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            let links = nav_mesh.get_polygon_links(tile, poly);
            assert_eq!(links.len(), 3);

            // Should traverse in order: 2 -> 1 -> 0
            assert_eq!(links[0].reference.id(), 300); // link3 (index 2)
            assert_eq!(links[1].reference.id(), 200); // link2 (index 1)
            assert_eq!(links[2].reference.id(), 100); // link1 (index 0)

            // Test that find_link works correctly with the chain
            assert_eq!(
                nav_mesh.find_link(tile, poly, 0).unwrap().reference.id(),
                100
            );
            assert_eq!(
                nav_mesh.find_link(tile, poly, 1).unwrap().reference.id(),
                200
            );
            assert_eq!(
                nav_mesh.find_link(tile, poly, 2).unwrap().reference.id(),
                300
            );

            // Test count is correct
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 3);
        }

        // Test with corrupted link index (out of bounds)
        {
            let tile = nav_mesh.tiles.get_mut(0).unwrap().as_mut().unwrap();
            tile.polys[0].first_link = Some(999); // Invalid index
        }

        {
            let tile = nav_mesh.tiles.first().unwrap().as_ref().unwrap();
            let poly = &tile.polys[0];
            assert_eq!(nav_mesh.count_polygon_links(tile, poly), 0);
            assert!(nav_mesh.get_polygon_links(tile, poly).is_empty());
            assert!(nav_mesh.find_link(tile, poly, 0).is_none());
        }
    }
}
