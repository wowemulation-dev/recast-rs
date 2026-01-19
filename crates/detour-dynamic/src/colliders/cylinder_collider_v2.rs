//! Cylinder collider implementation matching 

use super::{base::ColliderBase, Collider, ColliderType};
use glam::Vec3;
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CylinderCollider {
    /// Base collider data
    #[serde(skip)]
    pub base: ColliderBase,
    /// Start point of the cylinder axis
    pub start: Vec3,
    /// End point of the cylinder axis
    pub end: Vec3,
    /// Radius of the cylinder
    pub radius: f32,
}

impl CylinderCollider {
    /// Create a new cylinder collider matching 's constructor
    pub fn new(
        start: Vec3,
        end: Vec3,
        radius: f32,
        area: i32,
        flag_merge_threshold: f32,
    ) -> Self {
        let bounds = Self::compute_bounds(&start, &end, radius);
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            start,
            end,
            radius,
        }
    }
    
    /// Compute bounds for a cylinder
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
}

impl Collider for CylinderCollider {
    fn bounds(&self) -> (Vec3, Vec3) {
        self.base.bounds_as_vectors()
    }

    fn contains_point(&self, point: &Vec3) -> bool {
        // Calculate the closest point on the cylinder axis to the query point
        let axis = self.end - self.start;
        let axis_len_sq = axis.length_squared();
        
        if axis_len_sq < 1e-6 {
            // Degenerate cylinder (point)
            let dist_sq = (point - self.start).length_squared();
            return dist_sq <= self.radius * self.radius;
        }
        
        // Project point onto axis
        let t = ((point - self.start).dot(axis) / axis_len_sq).clamp(0.0, 1.0);
        let closest_point = self.start + axis * t;
        
        // Check distance from axis
        let dist_sq = (point - closest_point).length_squared();
        dist_sq <= self.radius * self.radius
    }

    fn rasterize(
        &self,
        heightfield: &mut Heightfield,
        _cell_size: f32,
        cell_height: f32,
        _world_min: &Vec3,
    ) -> Result<()> {
        // In , this calls RcFilledVolumeRasterization.RasterizeCylinder
        // We need to implement cylinder rasterization in the recast crate
        // For now, we'll use a simplified approach
        
        let flag_merge_threshold = (self.base.flag_merge_threshold / cell_height).floor() as i16;
        
        // Rasterize the cylinder into the heightfield
        recast::rasterize_cylinder(
            heightfield,
            &[self.start.x, self.start.y, self.start.z],
            &[self.end.x, self.end.y, self.end.z],
            self.radius,
            self.base.area as u8,
            flag_merge_threshold,
        )?;
        
        Ok(())
    }

    fn collider_type(&self) -> ColliderType {
        ColliderType::Cylinder
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