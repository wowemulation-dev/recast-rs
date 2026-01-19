//! Capsule collider implementation matching

use super::{Collider, ColliderType, base::ColliderBase};
use glam::Vec3;
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};

/// A capsule collider (cylinder with hemispherical caps) matching 's DtCapsuleCollider
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CapsuleCollider {
    /// Base collider data
    #[serde(skip)]
    pub base: ColliderBase,
    /// Start point of the capsule's central axis
    pub start: Vec3,
    /// End point of the capsule's central axis
    pub end: Vec3,
    /// Radius of the capsule
    pub radius: f32,
}

impl CapsuleCollider {
    /// Create a new capsule collider matching 's constructor
    pub fn new(start: Vec3, end: Vec3, radius: f32, area: i32, flag_merge_threshold: f32) -> Self {
        let bounds = Self::compute_bounds(&start, &end, radius);
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            start,
            end,
            radius,
        }
    }

    /// Create with default area and flag merge threshold for backwards compatibility
    pub fn new_simple(start: Vec3, end: Vec3, radius: f32) -> Self {
        Self::new(start, end, radius, 0, 1.0)
    }

    fn compute_bounds(start: &Vec3, end: &Vec3, radius: f32) -> [f32; 6] {
        [
            start.x.min(end.x) - radius,
            start.y.min(end.y) - radius,
            start.z.min(end.z) - radius,
            start.x.max(end.x) + radius,
            start.y.max(end.y) + radius,
            start.z.max(end.z) + radius,
        ]
    }

    /// Check if a point is inside this capsule
    pub fn contains_point(&self, point: &Vec3) -> bool {
        let distance = self.distance_to_line_segment(point);
        distance <= self.radius
    }

    /// Calculate distance from a point to the line segment
    fn distance_to_line_segment(&self, point: &Vec3) -> f32 {
        let segment = self.end - self.start;
        let segment_length_sq = segment.length_squared();

        if segment_length_sq < 1e-6 {
            // Degenerate case: start and end are the same
            return (point - self.start).length();
        }

        let t = ((point - self.start).dot(segment) / segment_length_sq).clamp(0.0, 1.0);
        let projection = self.start + segment * t;
        (point - projection).length()
    }
}

impl Collider for CapsuleCollider {
    fn bounds(&self) -> (Vec3, Vec3) {
        self.base.bounds_as_vectors()
    }

    fn contains_point(&self, point: &Vec3) -> bool {
        self.contains_point(point)
    }

    fn rasterize(
        &self,
        heightfield: &mut Heightfield,
        cell_size: f32,
        cell_height: f32,
        world_min: &Vec3,
    ) -> Result<()> {
        // In , this calls RcFilledVolumeRasterization.RasterizeCapsule
        // For now, we'll use a simple voxel-based approach
        // TODO: Implement proper capsule rasterization in recast crate

        let (aabb_min, aabb_max) = self.bounds();

        // Convert bounds to grid coordinates
        let min_x = ((aabb_min.x - world_min.x) / cell_size).floor() as i32;
        let max_x = ((aabb_max.x - world_min.x) / cell_size).ceil() as i32;
        let min_z = ((aabb_min.z - world_min.z) / cell_size).floor() as i32;
        let max_z = ((aabb_max.z - world_min.z) / cell_size).ceil() as i32;

        // Clamp to heightfield bounds
        let grid_min_x = min_x.max(0);
        let grid_max_x = max_x.min(heightfield.width - 1);
        let grid_min_z = min_z.max(0);
        let grid_max_z = max_z.min(heightfield.height - 1);

        // Rasterize each cell
        for z in grid_min_z..=grid_max_z {
            for x in grid_min_x..=grid_max_x {
                let world_x = world_min.x + (x as f32 + 0.5) * cell_size;
                let world_z = world_min.z + (z as f32 + 0.5) * cell_size;

                // Check multiple points vertically
                let min_y = aabb_min.y;
                let max_y = aabb_max.y;
                let y_steps = ((max_y - min_y) / cell_height).ceil() as i32 + 1;

                for i in 0..y_steps {
                    let world_y = min_y + i as f32 * cell_height;
                    let test_point = Vec3::new(world_x, world_y, world_z);

                    if self.contains_point(&test_point) {
                        let span_min = ((world_y - world_min.y) / cell_height).floor() as i16;
                        let span_max = span_min;
                        heightfield.add_span(x, z, span_min, span_max, self.base.area as u8)?;
                    }
                }
            }
        }

        Ok(())
    }

    fn collider_type(&self) -> ColliderType {
        ColliderType::Capsule
    }

    fn clone_box(&self) -> Box<dyn Collider> {
        Box::new(self.clone())
    }

    fn area(&self) -> i32 {
        self.base.area
    }

    fn flag_merge_threshold(&self) -> f32 {
        self.base.flag_merge_threshold
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}
