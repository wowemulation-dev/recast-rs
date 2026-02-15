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

    /// Sets the cell size (width/depth resolution).
    pub fn with_cell_size(mut self, cs: f32) -> Self {
        self.cs = cs;
        self
    }

    /// Sets the cell height (vertical resolution).
    pub fn with_cell_height(mut self, ch: f32) -> Self {
        self.ch = ch;
        self
    }

    /// Sets the AABB bounds and recalculates grid size.
    pub fn with_bounds(mut self, bmin: Vec3, bmax: Vec3) -> Self {
        self.calculate_grid_size(bmin, bmax);
        self
    }

    /// Sets the maximum walkable slope angle in degrees.
    pub fn with_walkable_slope_angle(mut self, angle: f32) -> Self {
        self.walkable_slope_angle = angle;
        self
    }

    /// Sets the minimum floor-to-ceiling height (in voxels).
    pub fn with_walkable_height(mut self, height: i32) -> Self {
        self.walkable_height = height;
        self
    }

    /// Sets the maximum traversable ledge height (in voxels).
    pub fn with_walkable_climb(mut self, climb: i32) -> Self {
        self.walkable_climb = climb;
        self
    }

    /// Sets the distance to erode walkable area from obstacles (in voxels).
    pub fn with_walkable_radius(mut self, radius: i32) -> Self {
        self.walkable_radius = radius;
        self
    }

    /// Sets the maximum allowed length for contour edges along the border.
    pub fn with_max_edge_len(mut self, len: i32) -> Self {
        self.max_edge_len = len;
        self
    }

    /// Sets the maximum distance a simplified contour may deviate from the raw contour.
    pub fn with_max_simplification_error(mut self, error: f32) -> Self {
        self.max_simplification_error = error;
        self
    }

    /// Sets the minimum number of cells for isolated island areas.
    pub fn with_min_region_area(mut self, area: i32) -> Self {
        self.min_region_area = area;
        self
    }

    /// Sets the region area threshold below which regions are merged.
    pub fn with_merge_region_area(mut self, area: i32) -> Self {
        self.merge_region_area = area;
        self
    }

    /// Sets the maximum number of vertices per polygon.
    pub fn with_max_vertices_per_polygon(mut self, count: i32) -> Self {
        self.max_vertices_per_polygon = count;
        self
    }

    /// Sets the sampling distance for the detail mesh.
    pub fn with_detail_sample_dist(mut self, dist: f32) -> Self {
        self.detail_sample_dist = dist;
        self
    }

    /// Sets the maximum error for the detail mesh surface.
    pub fn with_detail_sample_max_error(mut self, error: f32) -> Self {
        self.detail_sample_max_error = error;
        self
    }

    /// Sets the border size around the heightfield.
    pub fn with_border_size(mut self, size: i32) -> Self {
        self.border_size = size;
        self
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
