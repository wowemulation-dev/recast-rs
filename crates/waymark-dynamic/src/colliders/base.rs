//! Base collider implementation matching 's DtCollider abstract class

use glam::Vec3;

/// Base collider data matching 's DtCollider
#[derive(Debug, Clone, PartialEq)]
pub struct ColliderBase {
    /// Area type for navigation mesh generation
    pub area: i32,
    /// Flag merge threshold
    pub flag_merge_threshold: f32,
    /// Cached bounds for this collider
    pub bounds: [f32; 6],
}

impl ColliderBase {
    /// Create a new collider base
    pub fn new(area: i32, flag_merge_threshold: f32, bounds: [f32; 6]) -> Self {
        Self {
            area,
            flag_merge_threshold,
            bounds,
        }
    }

    /// Get the bounds as min/max vectors
    pub fn bounds_as_vectors(&self) -> (Vec3, Vec3) {
        let min = Vec3::new(self.bounds[0], self.bounds[1], self.bounds[2]);
        let max = Vec3::new(self.bounds[3], self.bounds[4], self.bounds[5]);
        (min, max)
    }

    /// Compute bounds from vertices
    pub fn compute_bounds(vertices: &[f32]) -> [f32; 6] {
        if vertices.is_empty() {
            return [0.0; 6];
        }

        let mut bounds = [
            vertices[0],
            vertices[1],
            vertices[2],
            vertices[0],
            vertices[1],
            vertices[2],
        ];

        for i in (3..vertices.len()).step_by(3) {
            bounds[0] = bounds[0].min(vertices[i]);
            bounds[1] = bounds[1].min(vertices[i + 1]);
            bounds[2] = bounds[2].min(vertices[i + 2]);
            bounds[3] = bounds[3].max(vertices[i]);
            bounds[4] = bounds[4].max(vertices[i + 1]);
            bounds[5] = bounds[5].max(vertices[i + 2]);
        }

        bounds
    }
}

/// Default area type for walkable surfaces
pub const SAMPLE_POLYAREA_TYPE_GROUND: i32 = 0;
/// Default area type for water surfaces  
pub const SAMPLE_POLYAREA_TYPE_WATER: i32 = 1;
/// Default area type for roads
pub const SAMPLE_POLYAREA_TYPE_ROAD: i32 = 2;
/// Default area type for doors
pub const SAMPLE_POLYAREA_TYPE_DOOR: i32 = 3;
/// Default area type for grass
pub const SAMPLE_POLYAREA_TYPE_GRASS: i32 = 4;
/// Default area type for jump areas
pub const SAMPLE_POLYAREA_TYPE_JUMP: i32 = 5;

impl Default for ColliderBase {
    fn default() -> Self {
        Self {
            area: SAMPLE_POLYAREA_TYPE_GROUND,
            flag_merge_threshold: 1.0,
            bounds: [0.0; 6],
        }
    }
}
