//! Sphere collider implementation matching 

use super::{base::ColliderBase, utils, Collider, ColliderType};
use glam::Vec3;
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};

/// A spherical collider matching 's DtSphereCollider
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SphereCollider {
    /// Base collider data
    #[serde(skip)]
    pub base: ColliderBase,
    /// Center of the sphere in world coordinates
    pub center: Vec3,
    /// Radius of the sphere
    pub radius: f32,
}

impl SphereCollider {
    /// Create a new sphere collider matching 's constructor
    pub fn new(center: Vec3, radius: f32, area: i32, flag_merge_threshold: f32) -> Self {
        let bounds = Self::compute_bounds(&center, radius);
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            center,
            radius,
        }
    }
    
    /// Create with default area and flag merge threshold for backwards compatibility
    pub fn new_simple(center: Vec3, radius: f32) -> Self {
        Self::new(center, radius, 0, 1.0)
    }
    
    /// Compute bounds for the sphere
    fn compute_bounds(center: &Vec3, radius: f32) -> [f32; 6] {
        [
            center.x - radius,
            center.y - radius,
            center.z - radius,
            center.x + radius,
            center.y + radius,
            center.z + radius,
        ]
    }

    /// Check if a point is inside this sphere
    pub fn contains_point(&self, point: &Vec3) -> bool {
        let distance_sq = (point - self.center).length_squared();
        distance_sq <= self.radius * self.radius
    }
}

impl Collider for SphereCollider {
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
        let (aabb_min, aabb_max) = self.bounds();

        // Calculate grid bounds to check
        let (min_x, min_y, min_z) = utils::world_to_grid(&aabb_min, world_min, cell_size);
        let (max_x, max_y, max_z) = utils::world_to_grid(&aabb_max, world_min, cell_size);

        // Ensure we're within heightfield bounds
        let grid_min_x = min_x.max(0);
        let grid_max_x = max_x.min(heightfield.width - 1);
        let grid_min_z = min_z.max(0);
        let grid_max_z = max_z.min(heightfield.height - 1);

        // For each cell in the bounding box, check if the sphere intersects it
        for z in grid_min_z..=grid_max_z {
            for x in grid_min_x..=grid_max_x {
                // Calculate world position of cell center
                let world_x = world_min.x + (x as f32 + 0.5) * cell_size;
                let world_z = world_min.z + (z as f32 + 0.5) * cell_size;

            // Can have multiple heights within the sphere's Y range
                let y_start = (min_y as f32 * cell_height + world_min.y).max(world_min.y);
                let y_end = (max_y as f32 * cell_height + world_min.y)
                    .min(world_min.y + heightfield.height as f32 * cell_height);

                let mut y = y_start;
                while y <= y_end {
                    let test_point = Vec3::new(world_x, y, world_z);

                    if self.contains_point(&test_point) {
                        // Convert Y coordinate to heightfield layer
                        let h = ((y - world_min.y) / cell_height).floor() as i32;
                        if h >= 0 {
                            // Mark this cell as solid by adding a span with the configured area
                            heightfield.add_span(x, z, h as i16, h as i16, self.base.area as u8)?;
                        }
                    }

                    y += cell_height;
                }
            }
        }

        Ok(())
    }

    fn collider_type(&self) -> ColliderType {
        ColliderType::Sphere
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere_contains_point() {
        let sphere = SphereCollider::new(Vec3::new(0.0, 0.0, 0.0), 5.0, 1, 1.0);

        // Point at center
        assert!(sphere.contains_point(&Vec3::new(0.0, 0.0, 0.0)));

        // Point inside
        assert!(sphere.contains_point(&Vec3::new(2.0, 2.0, 2.0)));

        // Point on surface
        assert!(sphere.contains_point(&Vec3::new(5.0, 0.0, 0.0)));

        // Point outside
        assert!(!sphere.contains_point(&Vec3::new(6.0, 0.0, 0.0)));
        assert!(!sphere.contains_point(&Vec3::new(4.0, 4.0, 4.0)));
    }

    #[test]
    fn test_sphere_bounds() {
        let sphere = SphereCollider::new(Vec3::new(10.0, 5.0, -2.0), 3.0, 1, 1.0);
        let (min, max) = sphere.bounds();

        assert_eq!(min, Vec3::new(7.0, 2.0, -5.0));
        assert_eq!(max, Vec3::new(13.0, 8.0, 1.0));
    }
}
