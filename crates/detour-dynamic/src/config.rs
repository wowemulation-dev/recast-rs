use glam::Vec3;

#[derive(Debug, Clone)]
pub struct DynamicNavMeshConfig {
    pub use_tiles: bool,
    pub tile_size_x: i32,
    pub tile_size_z: i32,
    pub cell_size: f32,
    pub cell_height: f32,
    pub world_min: Vec3,
    pub world_max: Vec3,
    pub partition: i32,
    pub walkable_area: u8,
    pub walkable_height: f32,
    pub walkable_slope_angle: f32,
    pub walkable_radius: f32,
    pub walkable_climb: f32,
    pub min_region_area: f32,
    pub region_merge_area: f32,
    pub max_edge_len: f32,
    pub max_simplification_error: f32,
    pub verts_per_poly: i32,
    pub build_detail_mesh: bool,
    pub detail_sample_distance: f32,
    pub detail_sample_max_error: f32,
    pub filter_low_hanging_obstacles: bool,
    pub filter_ledge_spans: bool,
    pub filter_walkable_low_height_spans: bool,
    pub enable_checkpoints: bool,
    pub keep_intermediate_results: bool,
}

impl DynamicNavMeshConfig {
    pub fn new(use_tiles: bool, tile_size_x: i32, tile_size_z: i32, cell_size: f32) -> Self {
        DynamicNavMeshConfig {
            use_tiles,
            tile_size_x,
            tile_size_z,
            cell_size,
            cell_height: 0.2,
            world_min: Vec3::new(-10.0, -5.0, -10.0),
            world_max: Vec3::new(10.0, 5.0, 10.0),
            partition: 0, // Watershed by default
            walkable_area: 1,
            walkable_height: 2.0,
            walkable_slope_angle: 45.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            min_region_area: 8.0,
            region_merge_area: 20.0,
            max_edge_len: 12.0,
            max_simplification_error: 1.3,
            verts_per_poly: 6,
            build_detail_mesh: true,
            detail_sample_distance: 6.0,
            detail_sample_max_error: 1.0,
            filter_low_hanging_obstacles: true,
            filter_ledge_spans: true,
            filter_walkable_low_height_spans: true,
            enable_checkpoints: true,
            keep_intermediate_results: false,
        }
    }

    pub fn with_walkable_height(mut self, walkable_height: f32) -> Self {
        self.walkable_height = walkable_height;
        self
    }

    pub fn with_walkable_radius(mut self, walkable_radius: f32) -> Self {
        self.walkable_radius = walkable_radius;
        self
    }

    pub fn with_walkable_climb(mut self, walkable_climb: f32) -> Self {
        self.walkable_climb = walkable_climb;
        self
    }

    pub fn with_walkable_slope_angle(mut self, walkable_slope_angle: f32) -> Self {
        self.walkable_slope_angle = walkable_slope_angle;
        self
    }

    pub fn with_min_region_area(mut self, min_region_area: f32) -> Self {
        self.min_region_area = min_region_area;
        self
    }

    pub fn with_region_merge_area(mut self, region_merge_area: f32) -> Self {
        self.region_merge_area = region_merge_area;
        self
    }

    pub fn with_max_edge_len(mut self, max_edge_len: f32) -> Self {
        self.max_edge_len = max_edge_len;
        self
    }

    pub fn with_max_simplification_error(mut self, max_simplification_error: f32) -> Self {
        self.max_simplification_error = max_simplification_error;
        self
    }

    pub fn with_verts_per_poly(mut self, verts_per_poly: i32) -> Self {
        self.verts_per_poly = verts_per_poly;
        self
    }

    pub fn with_detail_mesh(mut self, build_detail_mesh: bool) -> Self {
        self.build_detail_mesh = build_detail_mesh;
        self
    }

    pub fn with_detail_sample_distance(mut self, detail_sample_distance: f32) -> Self {
        self.detail_sample_distance = detail_sample_distance;
        self
    }

    pub fn with_detail_sample_max_error(mut self, detail_sample_max_error: f32) -> Self {
        self.detail_sample_max_error = detail_sample_max_error;
        self
    }

    pub fn with_checkpoints(mut self, enable_checkpoints: bool) -> Self {
        self.enable_checkpoints = enable_checkpoints;
        self
    }

    pub fn with_keep_intermediate_results(mut self, keep_intermediate_results: bool) -> Self {
        self.keep_intermediate_results = keep_intermediate_results;
        self
    }

    pub fn with_world_bounds(mut self, world_min: Vec3, world_max: Vec3) -> Self {
        self.world_min = world_min;
        self.world_max = world_max;
        self
    }

    pub fn with_cell_height(mut self, cell_height: f32) -> Self {
        self.cell_height = cell_height;
        self
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.cell_size <= 0.0 {
            return Err("Cell size must be positive".to_string());
        }
        if self.cell_height <= 0.0 {
            return Err("Cell height must be positive".to_string());
        }
        if self.walkable_height <= 0.0 {
            return Err("Walkable height must be positive".to_string());
        }
        if self.walkable_radius < 0.0 {
            return Err("Walkable radius cannot be negative".to_string());
        }
        if self.walkable_climb < 0.0 {
            return Err("Walkable climb cannot be negative".to_string());
        }
        if self.world_min.x >= self.world_max.x
            || self.world_min.y >= self.world_max.y
            || self.world_min.z >= self.world_max.z
        {
            return Err("World bounds invalid: min must be less than max".to_string());
        }
        Ok(())
    }
}

impl Default for DynamicNavMeshConfig {
    fn default() -> Self {
        Self::new(true, 32, 32, 0.3).with_world_bounds(
            Vec3::new(-100.0, -10.0, -100.0),
            Vec3::new(100.0, 10.0, 100.0),
        )
    }
}
