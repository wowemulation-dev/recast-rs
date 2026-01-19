//! Collider types for dynamic obstacles

pub mod base;
pub mod box_collider;
pub mod capsule_collider;
pub mod composite_collider;
pub mod convex_trimesh_collider;
pub mod cylinder_collider;
pub mod sphere_collider;
pub mod trimesh_collider;

use glam::Vec3;
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};

// Re-export collider types
pub use base::{ColliderBase, SAMPLE_POLYAREA_TYPE_GROUND, SAMPLE_POLYAREA_TYPE_WATER, 
              SAMPLE_POLYAREA_TYPE_ROAD, SAMPLE_POLYAREA_TYPE_DOOR,
              SAMPLE_POLYAREA_TYPE_GRASS, SAMPLE_POLYAREA_TYPE_JUMP};
pub use box_collider::BoxCollider;
pub use capsule_collider::CapsuleCollider;
pub use composite_collider::CompositeCollider;
pub use convex_trimesh_collider::ConvexTrimeshCollider;
pub use cylinder_collider::CylinderCollider;
pub use sphere_collider::SphereCollider;
pub use trimesh_collider::TrimeshCollider;

/// Trait for objects that can be rasterized into a heightfield as obstacles
/// Matches 's IDtCollider interface
pub trait Collider: std::any::Any + Send + Sync {
    fn bounds(&self) -> (Vec3, Vec3);

    /// Check if a point is inside this collider
    fn contains_point(&self, point: &Vec3) -> bool;

    ///
    /// This method should mark voxels that are inside the collider as solid,
    /// effectively creating obstacles in the navigation mesh.
    ///
    /// # Arguments
    ///
    /// * `heightfield` - The heightfield to rasterize into
    /// * `cell_size` - Size of each cell in world units
    /// * `cell_height` - Height of each cell in world units
    /// * `world_min` - World space minimum bounds
    fn rasterize(
        &self,
        heightfield: &mut Heightfield,
        cell_size: f32,
        cell_height: f32,
        world_min: &Vec3,
    ) -> Result<()>;

    /// Get a unique identifier for this collider type
    fn collider_type(&self) -> ColliderType;

    /// Clone this collider as a boxed trait object
    fn clone_box(&self) -> Box<dyn Collider>;

    /// Returns self as &dyn Any for downcasting
    /// This is needed because trait upcasting coercion is not yet stable
    fn as_any(&self) -> &dyn std::any::Any;

    fn area(&self) -> i32 {
        0 // Default walkable area
    }

    fn flag_merge_threshold(&self) -> f32 {
        1.0 // Default merge threshold
    }
}

/// Types of colliders supported
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ColliderType {
    Box,
    Cylinder,
    Sphere,
    Capsule,
    Trimesh,
    ConvexTrimesh,
    Composite,
}

/// Serializable collider data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SerializableCollider {
    Box(BoxCollider),
    Cylinder(CylinderCollider),
    Sphere(SphereCollider),
    Capsule(CapsuleCollider),
    Trimesh(TrimeshCollider),
    ConvexTrimesh(ConvexTrimeshCollider),
    Composite(CompositeCollider),
}

impl SerializableCollider {
    /// Convert to a boxed collider trait object
    pub fn into_collider(self) -> Box<dyn Collider> {
        match self {
            SerializableCollider::Box(c) => Box::new(c),
            SerializableCollider::Cylinder(c) => Box::new(c),
            SerializableCollider::Sphere(c) => Box::new(c),
            SerializableCollider::Capsule(c) => Box::new(c),
            SerializableCollider::Trimesh(c) => Box::new(c),
            SerializableCollider::ConvexTrimesh(c) => Box::new(c),
            SerializableCollider::Composite(c) => Box::new(c),
        }
    }

    /// Create from a collider trait object
    pub fn from_collider(collider: &dyn Collider) -> Option<Self> {
        let any = collider.as_any();
        match collider.collider_type() {
            ColliderType::Box => any
                .downcast_ref::<BoxCollider>()
                .map(|box_collider| SerializableCollider::Box(box_collider.clone())),
            ColliderType::Cylinder => any
                .downcast_ref::<CylinderCollider>()
                .map(|cylinder_collider| SerializableCollider::Cylinder(cylinder_collider.clone())),
            ColliderType::Sphere => any
                .downcast_ref::<SphereCollider>()
                .map(|sphere_collider| SerializableCollider::Sphere(sphere_collider.clone())),
            ColliderType::Capsule => any
                .downcast_ref::<CapsuleCollider>()
                .map(|capsule_collider| SerializableCollider::Capsule(capsule_collider.clone())),
            ColliderType::Trimesh => any
                .downcast_ref::<TrimeshCollider>()
                .map(|trimesh_collider| SerializableCollider::Trimesh(trimesh_collider.clone())),
            ColliderType::ConvexTrimesh => any
                .downcast_ref::<ConvexTrimeshCollider>()
                .map(|convex_trimesh_collider| SerializableCollider::ConvexTrimesh(convex_trimesh_collider.clone())),
            ColliderType::Composite => any
                .downcast_ref::<CompositeCollider>()
                .map(|composite_collider| {
                    SerializableCollider::Composite(composite_collider.clone())
                }),
        }
    }
}

impl Clone for Box<dyn Collider> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

/// Utility functions for collider operations
pub mod utils {
    use super::*;

    /// Check if a point is inside an axis-aligned bounding box
    pub fn point_in_aabb(point: &Vec3, min: &Vec3, max: &Vec3) -> bool {
        point.x >= min.x
            && point.x <= max.x
            && point.y >= min.y
            && point.y <= max.y
            && point.z >= min.z
            && point.z <= max.z
    }

    /// Expand an AABB by a given margin
    pub fn expand_aabb(
        min: &Vec3,
        max: &Vec3,
        margin: f32,
    ) -> (Vec3, Vec3) {
        let margin_vec = Vec3::new(margin, margin, margin);
        (min - margin_vec, max + margin_vec)
    }

    /// Calculate the intersection of two AABBs
    pub fn intersect_aabb(
        min1: &Vec3,
        max1: &Vec3,
        min2: &Vec3,
        max2: &Vec3,
    ) -> Option<(Vec3, Vec3)> {
        let min = Vec3::new(min1.x.max(min2.x), min1.y.max(min2.y), min1.z.max(min2.z));
        let max = Vec3::new(max1.x.min(max2.x), max1.y.min(max2.y), max1.z.min(max2.z));

        if min.x <= max.x && min.y <= max.y && min.z <= max.z {
            Some((min, max))
        } else {
            None
        }
    }

    /// Convert world coordinates to heightfield grid coordinates
    pub fn world_to_grid(
        world_pos: &Vec3,
        world_min: &Vec3,
        cell_size: f32,
    ) -> (i32, i32, i32) {
        let x = ((world_pos.x - world_min.x) / cell_size).floor() as i32;
        let y = ((world_pos.y - world_min.y) / cell_size).floor() as i32;
        let z = ((world_pos.z - world_min.z) / cell_size).floor() as i32;
        (x, y, z)
    }

    /// Convert heightfield grid coordinates to world coordinates
    pub fn grid_to_world(
        grid_x: i32,
        grid_y: i32,
        grid_z: i32,
        world_min: &Vec3,
        cell_size: f32,
    ) -> Vec3 {
        Vec3::new(
            world_min.x + grid_x as f32 * cell_size,
            world_min.y + grid_y as f32 * cell_size,
            world_min.z + grid_z as f32 * cell_size,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_utils() {
        let min = Vec3::new(0.0, 0.0, 0.0);
        let max = Vec3::new(10.0, 10.0, 10.0);

        // Point inside
        assert!(utils::point_in_aabb(
            &Vec3::new(5.0, 5.0, 5.0),
            &min,
            &max
        ));

        // Point outside
        assert!(!utils::point_in_aabb(
            &Vec3::new(15.0, 5.0, 5.0),
            &min,
            &max
        ));

        // Point on boundary
        assert!(utils::point_in_aabb(
            &Vec3::new(10.0, 10.0, 10.0),
            &min,
            &max
        ));
    }

    #[test]
    fn test_expand_aabb() {
        let min = Vec3::new(0.0, 0.0, 0.0);
        let max = Vec3::new(10.0, 10.0, 10.0);

        let (new_min, new_max) = utils::expand_aabb(&min, &max, 1.0);

        assert_eq!(new_min, Vec3::new(-1.0, -1.0, -1.0));
        assert_eq!(new_max, Vec3::new(11.0, 11.0, 11.0));
    }

    #[test]
    fn test_intersect_aabb() {
        let min1 = Vec3::new(0.0, 0.0, 0.0);
        let max1 = Vec3::new(10.0, 10.0, 10.0);
        let min2 = Vec3::new(5.0, 5.0, 5.0);
        let max2 = Vec3::new(15.0, 15.0, 15.0);

        let intersection = utils::intersect_aabb(&min1, &max1, &min2, &max2);
        assert!(intersection.is_some());

        let (int_min, int_max) = intersection.unwrap();
        assert_eq!(int_min, Vec3::new(5.0, 5.0, 5.0));
        assert_eq!(int_max, Vec3::new(10.0, 10.0, 10.0));

        // No intersection
        let min3 = Vec3::new(20.0, 20.0, 20.0);
        let max3 = Vec3::new(30.0, 30.0, 30.0);

        let no_intersection = utils::intersect_aabb(&min1, &max1, &min3, &max3);
        assert!(no_intersection.is_none());
    }

    #[test]
    fn test_coordinate_conversion() {
        let world_min = Vec3::new(-10.0, -5.0, -10.0);
        let cell_size = 0.5;

        let world_pos = Vec3::new(0.0, 0.0, 0.0);
        let (gx, gy, gz) = utils::world_to_grid(&world_pos, &world_min, cell_size);

        assert_eq!(gx, 20);
        assert_eq!(gy, 10);
        assert_eq!(gz, 20);

        let converted_back = utils::grid_to_world(gx, gy, gz, &world_min, cell_size);
        assert!((converted_back.x - (-10.0 + 20.0 * 0.5)).abs() < 0.001);
        assert!((converted_back.y - (-5.0 + 10.0 * 0.5)).abs() < 0.001);
        assert!((converted_back.z - (-10.0 + 20.0 * 0.5)).abs() < 0.001);
    }
}
