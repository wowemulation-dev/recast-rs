//! Triangle mesh collider implementation matching 

use super::{base::ColliderBase, Collider, ColliderType};
use glam::Vec3;
use recast::Heightfield;
use recast_common::Result;
use serde::{Deserialize, Serialize};

/// A triangle mesh collider matching 's DtTrimeshCollider
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimeshCollider {
    /// Base collider data
    #[serde(skip)]
    pub base: ColliderBase,
    /// Vertices of the triangle mesh
    pub vertices: Vec<Vec3>,
    /// Triangle indices (groups of 3)
    pub indices: Vec<u32>,
}

impl TrimeshCollider {
    /// Create a new triangle mesh collider matching 's constructor
    pub fn new(vertices: Vec<Vec3>, indices: Vec<u32>, area: i32, flag_merge_threshold: f32) -> Self {
        let bounds = Self::compute_bounds(&vertices);
        Self {
            base: ColliderBase::new(area, flag_merge_threshold, bounds),
            vertices,
            indices,
        }
    }
    
    /// Create with default area and flag merge threshold for backwards compatibility
    pub fn new_simple(vertices: Vec<Vec3>, indices: Vec<u32>) -> Self {
        Self::new(vertices, indices, 0, 1.0)
    }
    
    /// Compute bounds for the trimesh
    fn compute_bounds(vertices: &[Vec3]) -> [f32; 6] {
        if vertices.is_empty() {
            return [0.0; 6];
        }
        
        let mut min_x = vertices[0].x;
        let mut min_y = vertices[0].y;
        let mut min_z = vertices[0].z;
        let mut max_x = vertices[0].x;
        let mut max_y = vertices[0].y;
        let mut max_z = vertices[0].z;
        
        for vertex in vertices.iter().skip(1) {
            min_x = min_x.min(vertex.x);
            min_y = min_y.min(vertex.y);
            min_z = min_z.min(vertex.z);
            max_x = max_x.max(vertex.x);
            max_y = max_y.max(vertex.y);
            max_z = max_z.max(vertex.z);
        }
        
        [min_x, min_y, min_z, max_x, max_y, max_z]
    }

    /// Get triangles as triplets of vertices
    pub fn triangles(
        &self,
    ) -> impl Iterator<Item = (Vec3, Vec3, Vec3)> + '_ {
        self.indices.chunks(3).filter_map(|chunk| {
            if chunk.len() == 3 {
                let i0 = chunk[0] as usize;
                let i1 = chunk[1] as usize;
                let i2 = chunk[2] as usize;

                if i0 < self.vertices.len() && i1 < self.vertices.len() && i2 < self.vertices.len()
                {
                    Some((self.vertices[i0], self.vertices[i1], self.vertices[i2]))
                } else {
                    None
                }
            } else {
                None
            }
        })
    }

    /// Check if a point is inside this trimesh (approximation using bounding box)
    /// Note: This is a simplified implementation using the bounding box as an approximation
    /// A full implementation would require complex point-in-mesh testing
    pub fn contains_point(&self, point: &Vec3) -> bool {
        let (min, max) = self.bounds();
        point.x >= min.x
            && point.x <= max.x
            && point.y >= min.y
            && point.y <= max.y
            && point.z >= min.z
            && point.z <= max.z
    }
}

impl Collider for TrimeshCollider {
    fn bounds(&self) -> (Vec3, Vec3) {
        self.base.bounds_as_vectors()
    }

    fn contains_point(&self, point: &Vec3) -> bool {
        self.contains_point(point)
    }

    fn rasterize(
        &self,
        heightfield: &mut Heightfield,
        _cell_size: f32,
        cell_height: f32,
        _world_min: &Vec3,
    ) -> Result<()> {
        // In , this calls RcFilledVolumeRasterization.RasterizeTrimesh
        // Calculate flag merge threshold in heightfield units
        let flag_merge_threshold = (self.base.flag_merge_threshold / cell_height).floor() as i32;
        
        // Rasterize each triangle
        for triangle in self.triangles() {
            let v0 = triangle.0;
            let v1 = triangle.1;
            let v2 = triangle.2;
            
            // Use Recast's rasterize_triangle function
            recast::rasterize_triangle(
                &v0,
                &v1,
                &v2,
                self.base.area as u8,
                heightfield,
                flag_merge_threshold,
            )?;
        }
        
        Ok(())
    }

    fn collider_type(&self) -> ColliderType {
        ColliderType::Trimesh
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
