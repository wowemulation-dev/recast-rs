use super::VoxelTile;
use glam::Vec3;

pub const VOXEL_FILE_MAGIC: u32 =
    (b'V' as u32) << 24 | (b'O' as u32) << 16 | (b'X' as u32) << 8 | (b'L' as u32);
pub const VERSION_EXPORTER_MASK: u32 = 0xF000;
pub const VERSION_COMPRESSION_MASK: u32 = 0x0F00;
pub const VERSION_EXPORTER_RECAST4J: u32 = 0x1000;
pub const VERSION_COMPRESSION_LZ4: u32 = 0x0100;

#[derive(Debug)]
pub struct VoxelFile {
    pub version: u32,
    pub partition: i32,
    pub filter_low_hanging_obstacles: bool,
    pub filter_ledge_spans: bool,
    pub filter_walkable_low_height_spans: bool,
    pub walkable_radius: f32,
    pub walkable_height: f32,
    pub walkable_climb: f32,
    pub walkable_slope_angle: f32,
    pub cell_size: f32,
    pub max_simplification_error: f32,
    pub max_edge_len: f32,
    pub min_region_area: f32,
    pub region_merge_area: f32,
    pub verts_per_poly: i32,
    pub build_mesh_detail: bool,
    pub detail_sample_distance: f32,
    pub detail_sample_max_error: f32,
    pub use_tiles: bool,
    pub tile_size_x: i32,
    pub tile_size_z: i32,
    pub rotation: Vec3,
    pub bounds: [f32; 6],
    pub tiles: Vec<VoxelTile>,
}

impl Default for VoxelFile {
    fn default() -> Self {
        Self::new()
    }
}

impl VoxelFile {
    pub fn new() -> Self {
        VoxelFile {
            version: 1,
            partition: 0, // Watershed partitioning
            filter_low_hanging_obstacles: true,
            filter_ledge_spans: true,
            filter_walkable_low_height_spans: true,
            walkable_radius: 0.0,
            walkable_height: 0.0,
            walkable_climb: 0.0,
            walkable_slope_angle: 0.0,
            cell_size: 0.0,
            max_simplification_error: 0.0,
            max_edge_len: 0.0,
            min_region_area: 0.0,
            region_merge_area: 0.0,
            verts_per_poly: 6,
            build_mesh_detail: true,
            detail_sample_distance: 0.0,
            detail_sample_max_error: 0.0,
            use_tiles: false,
            tile_size_x: 0,
            tile_size_z: 0,
            rotation: Vec3::ZERO,
            bounds: [0.0; 6],
            tiles: Vec::new(),
        }
    }

    pub fn add_tile(&mut self, tile: VoxelTile) {
        self.tiles.push(tile);
    }
}
