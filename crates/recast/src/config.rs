//! Configuration for the Recast navigation mesh generation process

use glam::Vec3;

use crate::error::ConfigError;

/// Configuration parameters for Recast navigation mesh generation
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct RecastConfig {
    /// The width of the field along the x-axis
    pub width: i32,
    /// The height of the field along the z-axis
    pub height: i32,

    /// The width/depth resolution of the field (cell size)
    pub cs: f32,
    /// The height resolution of the field (cell height)
    pub ch: f32,

    /// The minimum bounds of the field's AABB
    pub bmin: Vec3,
    /// The maximum bounds of the field's AABB
    pub bmax: Vec3,

    /// The maximum slope in degrees that is considered walkable
    pub walkable_slope_angle: f32,
    /// Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable
    pub walkable_height: i32,
    /// The maximum height between walkable layers
    pub walkable_climb: i32,
    /// The distance to erode/shrink the walkable area from obstacles
    pub walkable_radius: i32,

    /// The maximum allowed length for contour edges along the border of the mesh
    pub max_edge_len: i32,
    /// The maximum distance a simplified contour's border edges should deviate from the original raw contour
    pub max_simplification_error: f32,
    /// The minimum number of cells allowed to form isolated island areas
    pub min_region_area: i32,
    /// Any regions with an area smaller than this value will be merged with larger regions if possible
    pub merge_region_area: i32,

    /// The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process
    pub max_vertices_per_polygon: i32,

    /// Sets the sampling distance to use when generating the detail mesh
    pub detail_sample_dist: f32,
    /// The maximum distance the detail mesh surface should deviate from the heightfield data
    pub detail_sample_max_error: f32,

    /// The size of the border to add around the heightfield
    pub border_size: i32,
}

impl Default for RecastConfig {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            cs: 0.3,
            ch: 0.2,
            bmin: Vec3::ZERO,
            bmax: Vec3::ZERO,
            walkable_slope_angle: 45.0,
            walkable_height: 2,
            walkable_climb: 1,
            walkable_radius: 1,
            max_edge_len: 12,
            max_simplification_error: 1.3,
            min_region_area: 8,
            merge_region_area: 20,
            max_vertices_per_polygon: 6,
            detail_sample_dist: 6.0,
            detail_sample_max_error: 1.0,
            border_size: 0,
        }
    }
}

impl RecastConfig {
    /// Creates a new RecastConfig with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Calculates and sets the grid size based on the provided AABB
    pub fn calculate_grid_size(&mut self, bmin: Vec3, bmax: Vec3) {
        self.bmin = bmin;
        self.bmax = bmax;

        // Match C++ rcCalcGridSize: round-to-nearest via (int)(x + 0.5f)
        self.width = ((bmax.x - bmin.x) / self.cs + 0.5) as i32;
        self.height = ((bmax.z - bmin.z) / self.cs + 0.5) as i32;
    }

    /// Validates the configuration parameters
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.width <= 0 || self.height <= 0 {
            return Err(ConfigError::InvalidGridSize {
                width: self.width,
                height: self.height,
            });
        }

        if self.cs <= 0.0 || self.ch <= 0.0 {
            return Err(ConfigError::InvalidCellDimensions {
                cs: self.cs,
                ch: self.ch,
            });
        }

        if self.walkable_slope_angle < 0.0 || self.walkable_slope_angle > 90.0 {
            return Err(ConfigError::InvalidWalkableSlope {
                angle: self.walkable_slope_angle,
            });
        }

        if self.max_vertices_per_polygon < 3 {
            return Err(ConfigError::TooFewVertsPerPoly {
                count: self.max_vertices_per_polygon,
            });
        }

        Ok(())
    }
}
