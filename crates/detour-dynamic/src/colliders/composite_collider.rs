//! Composite collider implementation matching

use super::{Collider, ColliderType};
use glam::Vec3;
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};
use std::fmt;

/// A composite collider containing multiple child colliders matching 's DtCompositeCollider
///
/// Note: Unlike other colliders, CompositeCollider doesn't have base fields because it
/// aggregates other colliders rather than defining its own area/flagMergeThreshold
pub struct CompositeCollider {
    /// Child colliders
    pub colliders: Vec<Box<dyn Collider>>,
}

impl CompositeCollider {
    /// Create a new empty composite collider
    pub fn new() -> Self {
        Self {
            colliders: Vec::new(),
        }
    }

    /// Add a collider to this composite
    pub fn add_collider(&mut self, collider: Box<dyn Collider>) {
        self.colliders.push(collider);
    }

    /// Create from a vector of colliders
    pub fn from_colliders(colliders: Vec<Box<dyn Collider>>) -> Self {
        Self { colliders }
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.colliders.is_empty()
    }

    /// Get number of child colliders
    pub fn len(&self) -> usize {
        self.colliders.len()
    }

    /// Check if a point is inside any of the child colliders
    pub fn contains_point(&self, point: &Vec3) -> bool {
        self.colliders
            .iter()
            .any(|collider| collider.contains_point(point))
    }
}

impl Clone for CompositeCollider {
    fn clone(&self) -> Self {
        Self {
            colliders: self.colliders.clone(),
        }
    }
}

impl fmt::Debug for CompositeCollider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CompositeCollider")
            .field(
                "colliders",
                &format!("[{} collider(s)]", self.colliders.len()),
            )
            .finish()
    }
}

impl Serialize for CompositeCollider {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use super::SerializableCollider;
        let serializable_colliders: Vec<SerializableCollider> = self
            .colliders
            .iter()
            .filter_map(|c| SerializableCollider::from_collider(c.as_ref()))
            .collect();
        serializable_colliders.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for CompositeCollider {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use super::SerializableCollider;
        let serializable_colliders: Vec<SerializableCollider> = Vec::deserialize(deserializer)?;
        let colliders = serializable_colliders
            .into_iter()
            .map(|c| c.into_collider())
            .collect();
        Ok(Self { colliders })
    }
}

impl Default for CompositeCollider {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for CompositeCollider {
    fn eq(&self, other: &Self) -> bool {
        if self.colliders.len() != other.colliders.len() {
            return false;
        }

        // This is a simplified comparison - in practice, we'd need more sophisticated
        // comparison for trait objects
        self.colliders.len() == other.colliders.len()
    }
}

impl Collider for CompositeCollider {
    fn bounds(&self) -> (Vec3, Vec3) {
        if self.colliders.is_empty() {
            return (Vec3::ZERO, Vec3::ZERO);
        }

        let (mut min, mut max) = self.colliders[0].bounds();

        for collider in &self.colliders[1..] {
            let (c_min, c_max) = collider.bounds();
            min = min.min(c_min);
            max = max.max(c_max);
        }

        (min, max)
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
        // Rasterize all child colliders
        for collider in &self.colliders {
            collider.rasterize(heightfield, cell_size, cell_height, world_min)?;
        }
        Ok(())
    }

    fn collider_type(&self) -> ColliderType {
        ColliderType::Composite
    }

    fn clone_box(&self) -> Box<dyn Collider> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::colliders::{BoxCollider, SphereCollider};

    #[test]
    fn test_composite_collider() {
        let mut composite = CompositeCollider::new();
        assert!(composite.is_empty());
        assert_eq!(composite.len(), 0);

        // Create a box collider
        let box_collider =
            BoxCollider::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0), 0, 1.0);
        composite.add_collider(Box::new(box_collider));

        // Add a sphere collider
        let sphere_collider = SphereCollider::new(Vec3::new(5.0, 5.0, 5.0), 2.0, 0, 1.0);
        composite.add_collider(Box::new(sphere_collider));

        assert!(!composite.is_empty());
        assert_eq!(composite.len(), 2);

        // Test bounds calculation
        let (min, max) = composite.bounds();
        assert_eq!(min, Vec3::new(-1.0, -1.0, -1.0));
        assert_eq!(max, Vec3::new(7.0, 7.0, 7.0));
    }

    #[test]
    fn test_empty_composite() {
        let composite = CompositeCollider::new();
        let (min, max) = composite.bounds();
        assert_eq!(min, Vec3::ZERO);
        assert_eq!(max, Vec3::ZERO);
    }
}
