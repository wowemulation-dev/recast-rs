//! Convex triangle mesh collider implementation matching 

use super::{base::ColliderBase, Collider, ColliderType};
use glam::Vec3;
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};

///
/// This collider type is optimized for convex meshes and uses specialized
/// rasterization for better performance with convex hulls.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConvexTrimeshCollider {
    /// Base collider data
    #[serde(skip)]
    pub base: ColliderBase,
    /// Vertices of the triangle mesh (flattened array of x, y, z coordinates)
    pub vertices: Vec<f32>,
    /// Triangle indices (groups of 3)
    pub triangles: Vec<i32>,
}

impl ConvexTrimeshCollider {
    /// Create a new convex trimesh collider matching 's constructor
    pub fn new(
        vertices: Vec<f32>,
        triangles: Vec<i32>,
        area: i32,
        flag_merge_threshold: f32,
    ) -> Self {
        let bounds = ColliderBase::compute_bounds(&vertices);
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            vertices,
            triangles,
        }
    }
    
    /// Create with explicit bounds
    pub fn with_bounds(
        vertices: Vec<f32>,
        triangles: Vec<i32>,
        bounds: [f32; 6],
        area: i32,
        flag_merge_threshold: f32,
    ) -> Self {
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            vertices,
            triangles,
        }
    }
}

impl Collider for ConvexTrimeshCollider {
    fn bounds(&self) -> (Vec3, Vec3) {
        self.base.bounds_as_vectors()
    }

    fn contains_point(&self, point: &Vec3) -> bool {
        // For a convex mesh, we can use a simpler algorithm
        // Check if point is inside the bounding box first
        let (min, max) = self.bounds();
        if point.x < min.x || point.x > max.x ||
           point.y < min.y || point.y > max.y ||
           point.z < min.z || point.z > max.z {
            return false;
        }
        
        // For actual convex test, we'd need to check if the point is on the 
        // correct side of all faces. For now, use bounding box as approximation.
        true
    }

    fn rasterize(
        &self,
        heightfield: &mut Heightfield,
        _cell_size: f32,
        cell_height: f32,
        _world_min: &Vec3,
    ) -> Result<()> {
        // In , this calls RcFilledVolumeRasterization.RasterizeConvex
        // This is optimized for convex meshes
        let flag_merge_threshold = (self.base.flag_merge_threshold / cell_height).floor() as i16;
        
        // For now, we'll rasterize each triangle individually
        // A proper implementation would use the convex hull optimization
        for i in (0..self.triangles.len()).step_by(3) {
            let i0 = self.triangles[i] as usize * 3;
            let i1 = self.triangles[i + 1] as usize * 3;
            let i2 = self.triangles[i + 2] as usize * 3;
            
            if i0 + 2 < self.vertices.len() && 
               i1 + 2 < self.vertices.len() && 
               i2 + 2 < self.vertices.len() {
                let v0 = [self.vertices[i0], self.vertices[i0 + 1], self.vertices[i0 + 2]];
                let v1 = [self.vertices[i1], self.vertices[i1 + 1], self.vertices[i1 + 2]];
                let v2 = [self.vertices[i2], self.vertices[i2 + 1], self.vertices[i2 + 2]];
                
                // Convert arrays to glam vectors
                let v0_vec = Vec3::new(v0[0], v0[1], v0[2]);
                let v1_vec = Vec3::new(v1[0], v1[1], v1[2]);
                let v2_vec = Vec3::new(v2[0], v2[1], v2[2]);
                
                // Rasterize the triangle
                recast::rasterize_triangle(
                    &v0_vec,
                    &v1_vec,
                    &v2_vec,
                    self.base.area as u8,
                    heightfield,
                    flag_merge_threshold as i32,
                )?;
            }
        }
        
        Ok(())
    }

    fn collider_type(&self) -> ColliderType {
        ColliderType::ConvexTrimesh
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
    fn test_convex_trimesh_collider() {
        // Create a simple triangle
        let vertices = vec![
            0.0, 0.0, 0.0,  // Vertex 0
            1.0, 0.0, 0.0,  // Vertex 1
            0.5, 1.0, 0.0,  // Vertex 2
        ];
        let triangles = vec![0, 1, 2];
        
        let collider = ConvexTrimeshCollider::new(vertices, triangles, 1, 1.0);
        
        let (min, max) = collider.bounds();
        assert_eq!(min.x, 0.0);
        assert_eq!(min.y, 0.0);
        assert_eq!(min.z, 0.0);
        assert_eq!(max.x, 1.0);
        assert_eq!(max.y, 1.0);
        assert_eq!(max.z, 0.0);
    }
}