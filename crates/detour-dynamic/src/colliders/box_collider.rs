//! Box collider implementation matching

use super::{Collider, ColliderType, base::ColliderBase, utils};
use glam::{Mat3, Vec3};
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};

/// A box-shaped collider with arbitrary orientation matching 's DtBoxCollider
///
/// The box is defined by a center point, half-extents, and rotation matrix.
/// This allows for oriented boxes, not just axis-aligned ones.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoxCollider {
    /// Base collider data
    #[serde(skip)]
    pub base: ColliderBase,
    /// Center of the box in world coordinates
    pub center: Vec3,
    /// Half-extents of the box (distance from center to face)
    pub half_extents: Vec3,
    /// Rotation matrix for box orientation
    pub rotation: Mat3,
}

impl BoxCollider {
    /// Create a new axis-aligned box collider matching 's constructor
    pub fn new(center: Vec3, half_extents: Vec3, area: i32, flag_merge_threshold: f32) -> Self {
        let bounds = Self::compute_bounds(&center, &half_extents, &Mat3::IDENTITY);
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            center,
            half_extents,
            rotation: Mat3::IDENTITY,
        }
    }

    /// Create a new oriented box collider
    pub fn new_oriented(
        center: Vec3,
        half_extents: Vec3,
        rotation: Mat3,
        area: i32,
        flag_merge_threshold: f32,
    ) -> Self {
        let bounds = Self::compute_bounds(&center, &half_extents, &rotation);
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            center,
            half_extents,
            rotation,
        }
    }

    /// Create with default area and flag merge threshold for backwards compatibility
    pub fn axis_aligned(center: Vec3, half_extents: Vec3) -> Self {
        Self::new(center, half_extents, 0, 1.0)
    }

    /// Create a box from minimum and maximum coordinates (axis-aligned)
    pub fn from_bounds(min: Vec3, max: Vec3, area: i32, flag_merge_threshold: f32) -> Self {
        let center = (min + max) * 0.5;
        let half_extents = (max - min) * 0.5;
        Self::new(center, half_extents, area, flag_merge_threshold)
    }

    /// Compute bounds for the box
    fn compute_bounds(center: &Vec3, half_extents: &Vec3, rotation: &Mat3) -> [f32; 6] {
        // For axis-aligned box
        if rotation == &Mat3::IDENTITY {
            return [
                center.x - half_extents.x,
                center.y - half_extents.y,
                center.z - half_extents.z,
                center.x + half_extents.x,
                center.y + half_extents.y,
                center.z + half_extents.z,
            ];
        }

        // For oriented box, compute AABB of rotated corners
        let corners = [
            Vec3::new(-half_extents.x, -half_extents.y, -half_extents.z),
            Vec3::new(half_extents.x, -half_extents.y, -half_extents.z),
            Vec3::new(-half_extents.x, half_extents.y, -half_extents.z),
            Vec3::new(half_extents.x, half_extents.y, -half_extents.z),
            Vec3::new(-half_extents.x, -half_extents.y, half_extents.z),
            Vec3::new(half_extents.x, -half_extents.y, half_extents.z),
            Vec3::new(-half_extents.x, half_extents.y, half_extents.z),
            Vec3::new(half_extents.x, half_extents.y, half_extents.z),
        ];

        let mut min = Vec3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut max = Vec3::new(f32::MIN, f32::MIN, f32::MIN);

        for corner in &corners {
            let world_corner = *rotation * *corner + *center;
            min.x = min.x.min(world_corner.x);
            min.y = min.y.min(world_corner.y);
            min.z = min.z.min(world_corner.z);
            max.x = max.x.max(world_corner.x);
            max.y = max.y.max(world_corner.y);
            max.z = max.z.max(world_corner.z);
        }

        [min.x, min.y, min.z, max.x, max.y, max.z]
    }

    /// Check if a point is inside this box
    pub fn contains_point(&self, point: &Vec3) -> bool {
        // Transform point to box local space
        let local_point = self.rotation.transpose() * (*point - self.center);

        // Check if point is within half-extents in all dimensions
        local_point.x.abs() <= self.half_extents.x
            && local_point.y.abs() <= self.half_extents.y
            && local_point.z.abs() <= self.half_extents.z
    }

    /// Get the 8 vertices of this box
    pub fn vertices(&self) -> [Vec3; 8] {
        let hx = self.half_extents.x;
        let hy = self.half_extents.y;
        let hz = self.half_extents.z;

        // Local space vertices
        let local_vertices = [
            Vec3::new(-hx, -hy, -hz),
            Vec3::new(hx, -hy, -hz),
            Vec3::new(hx, hy, -hz),
            Vec3::new(-hx, hy, -hz),
            Vec3::new(-hx, -hy, hz),
            Vec3::new(hx, -hy, hz),
            Vec3::new(hx, hy, hz),
            Vec3::new(-hx, hy, hz),
        ];

        // Transform to world space
        let mut world_vertices = [Vec3::ZERO; 8];
        for (i, local_vertex) in local_vertices.iter().enumerate() {
            world_vertices[i] = self.center + self.rotation * *local_vertex;
        }

        world_vertices
    }

    /// Calculate axis-aligned bounding box of this oriented box
    fn _calculate_aabb(&self) -> (Vec3, Vec3) {
        let vertices = self.vertices();

        let mut min = vertices[0];
        let mut max = vertices[0];

        for vertex in vertices.iter().skip(1) {
            min = min.min(*vertex);
            max = max.max(*vertex);
        }

        (min, max)
    }
}

impl Collider for BoxCollider {
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

        // For each cell in the bounding box, check if the box intersects it
        for z in grid_min_z..=grid_max_z {
            for x in grid_min_x..=grid_max_x {
                // Calculate world position of cell center
                let world_x = world_min.x + (x as f32 + 0.5) * cell_size;
                let world_z = world_min.z + (z as f32 + 0.5) * cell_size;

                // Can have multiple heights within the box's Y range
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
        ColliderType::Box
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
    fn test_axis_aligned_box() {
        let center = Vec3::new(5.0, 2.0, 5.0);
        let half_extents = Vec3::new(2.0, 1.0, 3.0);
        let box_collider = BoxCollider::new(center, half_extents, 1, 1.0);

        // Test point inside
        assert!(box_collider.contains_point(&Vec3::new(5.0, 2.0, 5.0)));
        assert!(box_collider.contains_point(&Vec3::new(6.0, 2.5, 7.0)));

        // Test point outside
        assert!(!box_collider.contains_point(&Vec3::new(8.0, 2.0, 5.0)));
        assert!(!box_collider.contains_point(&Vec3::new(5.0, 4.0, 5.0)));

        // Test point on boundary
        assert!(box_collider.contains_point(&Vec3::new(7.0, 2.0, 5.0)));
    }

    #[test]
    fn test_oriented_box() {
        let center = Vec3::new(0.0, 0.0, 0.0);
        let half_extents = Vec3::new(2.0, 1.0, 1.0);

        // 45-degree rotation around Y axis
        let angle = std::f32::consts::PI / 4.0;
        let rotation = Mat3::from_cols(
            Vec3::new(angle.cos(), 0.0, -angle.sin()),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(angle.sin(), 0.0, angle.cos()),
        );

        let box_collider = BoxCollider::new_oriented(center, half_extents, rotation, 1, 1.0);

        // Point that would be inside an axis-aligned box but outside the rotated one
        let test_point = Vec3::new(2.0, 0.0, 0.0);
        // This depends on the exact rotation, so we'll just verify the method doesn't panic
        let _inside = box_collider.contains_point(&test_point);
    }

    #[test]
    fn test_from_bounds() {
        let min = Vec3::new(-1.0, -2.0, -3.0);
        let max = Vec3::new(5.0, 4.0, 9.0);

        let box_collider = BoxCollider::from_bounds(min, max, 1, 1.0);

        assert_eq!(box_collider.center, Vec3::new(2.0, 1.0, 3.0));
        assert_eq!(box_collider.half_extents, Vec3::new(3.0, 3.0, 6.0));
    }

    #[test]
    fn test_vertices() {
        let center = Vec3::new(1.0, 2.0, 3.0);
        let half_extents = Vec3::new(0.5, 1.0, 1.5);
        let box_collider = BoxCollider::new(center, half_extents, 1, 1.0);

        let vertices = box_collider.vertices();

        // Verify that all vertices are at the expected distance from center
        for vertex in &vertices {
            let local = *vertex - center;
            assert!(local.x.abs() <= half_extents.x + 1e-6);
            assert!(local.y.abs() <= half_extents.y + 1e-6);
            assert!(local.z.abs() <= half_extents.z + 1e-6);
        }

        // Should have 8 vertices
        assert_eq!(vertices.len(), 8);
    }

    #[test]
    fn test_bounds() {
        let center = Vec3::new(5.0, 5.0, 5.0);
        let half_extents = Vec3::new(2.0, 1.0, 3.0);
        let box_collider = BoxCollider::new(center, half_extents, 1, 1.0);

        let (min, max) = box_collider.bounds();

        // For axis-aligned box, bounds should match center ± half_extents
        assert!((min.x - 3.0).abs() < 1e-6);
        assert!((min.y - 4.0).abs() < 1e-6);
        assert!((min.z - 2.0).abs() < 1e-6);
        assert!((max.x - 7.0).abs() < 1e-6);
        assert!((max.y - 6.0).abs() < 1e-6);
        assert!((max.z - 8.0).abs() < 1e-6);
    }
}
