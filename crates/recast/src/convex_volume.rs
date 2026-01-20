//! Convex volume support for Recast navigation mesh generation
//!
//! Convex volumes allow modification of the navigation mesh generation process
//! in specific areas. They can be used to mark areas with special properties,
//! exclude areas from navigation, or modify walkability.

use glam::Vec3;
use recast_common::{Error, Result};
use std::rc::Rc;

/// Maximum number of vertices in a convex volume
pub const MAX_CONVEX_VOLUME_VERTS: usize = 32;

/// A convex volume used to modify navigation mesh generation
#[derive(Debug, Clone)]
pub struct ConvexVolume {
    /// Vertices defining the convex hull (in world space)
    pub vertices: Vec<Vec3>,
    /// Minimum height of the volume
    pub min_height: f32,
    /// Maximum height of the volume
    pub max_height: f32,
    /// Area type to apply within this volume
    pub area_type: u8,
}

impl ConvexVolume {
    /// Creates a new convex volume from vertices
    pub fn new(
        vertices: Vec<Vec3>,
        min_height: f32,
        max_height: f32,
        area_type: u8,
    ) -> Result<Self> {
        if vertices.len() < 3 {
            return Err(Error::InvalidMesh(
                "Convex volume requires at least 3 vertices".to_string(),
            ));
        }

        if vertices.len() > MAX_CONVEX_VOLUME_VERTS {
            return Err(Error::InvalidMesh(format!(
                "Convex volume has too many vertices: {} (max: {})",
                vertices.len(),
                MAX_CONVEX_VOLUME_VERTS
            )));
        }

        if min_height > max_height {
            return Err(Error::InvalidMesh(
                "Convex volume min_height > max_height".to_string(),
            ));
        }

        // Ensure vertices form a convex polygon
        if !Self::is_convex(&vertices) {
            return Err(Error::InvalidMesh(
                "Vertices do not form a convex polygon".to_string(),
            ));
        }

        Ok(Self {
            vertices,
            min_height,
            max_height,
            area_type,
        })
    }

    /// Creates a box-shaped convex volume
    pub fn from_box(center: Vec3, half_extents: Vec3, area_type: u8) -> Result<Self> {
        let min_x = center.x - half_extents.x;
        let max_x = center.x + half_extents.x;
        let min_z = center.z - half_extents.z;
        let max_z = center.z + half_extents.z;

        // Create vertices in counter-clockwise order when viewed from above
        let vertices = vec![
            Vec3::new(min_x, center.y, min_z),
            Vec3::new(min_x, center.y, max_z),
            Vec3::new(max_x, center.y, max_z),
            Vec3::new(max_x, center.y, min_z),
        ];

        let min_height = center.y - half_extents.y;
        let max_height = center.y + half_extents.y;

        Self::new(vertices, min_height, max_height, area_type)
    }

    /// Creates a cylinder-shaped convex volume (approximated as a polygon)
    pub fn from_cylinder(
        center: Vec3,
        radius: f32,
        height: f32,
        segments: usize,
        area_type: u8,
    ) -> Result<Self> {
        if segments < 3 {
            return Err(Error::InvalidMesh(
                "Cylinder requires at least 3 segments".to_string(),
            ));
        }

        if segments > MAX_CONVEX_VOLUME_VERTS {
            return Err(Error::InvalidMesh(format!(
                "Cylinder has too many segments: {} (max: {})",
                segments, MAX_CONVEX_VOLUME_VERTS
            )));
        }

        let mut vertices = Vec::with_capacity(segments);
        let angle_step = 2.0 * std::f32::consts::PI / segments as f32;

        for i in 0..segments {
            let angle = i as f32 * angle_step;
            let x = center.x + radius * angle.cos();
            let z = center.z + radius * angle.sin();
            vertices.push(Vec3::new(x, center.y, z));
        }

        let min_height = center.y - height * 0.5;
        let max_height = center.y + height * 0.5;

        Self::new(vertices, min_height, max_height, area_type)
    }

    /// Checks if a point is inside the convex volume
    pub fn contains_point(&self, point: Vec3) -> bool {
        // Check height bounds first
        if point.y < self.min_height || point.y > self.max_height {
            return false;
        }

        // Check if point is inside the convex polygon (2D test in XZ plane)
        self.point_in_polygon_2d(point.x, point.z)
    }

    /// Checks if a 2D point is inside the convex polygon
    fn point_in_polygon_2d(&self, x: f32, z: f32) -> bool {
        // Use ray-casting algorithm (same as C++ pointInPoly)
        let mut inside = false;
        let n = self.vertices.len();
        let mut j = n - 1;

        for i in 0..n {
            let vi = &self.vertices[i];
            let vj = &self.vertices[j];

            if (vi.z > z) == (vj.z > z) {
                j = i;
                continue;
            }

            if x >= (vj.x - vi.x) * (z - vi.z) / (vj.z - vi.z) + vi.x {
                j = i;
                continue;
            }

            inside = !inside;
            j = i;
        }

        inside
    }

    /// Checks if a set of vertices forms a convex polygon
    fn is_convex(vertices: &[Vec3]) -> bool {
        let n = vertices.len();
        if n < 3 {
            return false;
        }

        let mut sign = 0.0;

        for i in 0..n {
            let j = (i + 1) % n;
            let k = (i + 2) % n;

            let v1 = &vertices[i];
            let v2 = &vertices[j];
            let v3 = &vertices[k];

            // Calculate cross product of edges
            let dx1 = v2.x - v1.x;
            let dz1 = v2.z - v1.z;
            let dx2 = v3.x - v2.x;
            let dz2 = v3.z - v2.z;

            let cross = dx1 * dz2 - dz1 * dx2;

            if cross != 0.0 {
                if sign == 0.0 {
                    sign = cross;
                } else if (cross > 0.0) != (sign > 0.0) {
                    // Different signs mean non-convex
                    return false;
                }
            }
        }

        true
    }

    /// Gets the bounding box of the convex volume
    pub fn get_bounds(&self) -> (Vec3, Vec3) {
        if self.vertices.is_empty() {
            return (Vec3::ZERO, Vec3::ZERO);
        }

        let mut min_bounds = Vec3::new(f32::MAX, self.min_height, f32::MAX);
        let mut max_bounds = Vec3::new(f32::MIN, self.max_height, f32::MIN);

        for vertex in &self.vertices {
            min_bounds.x = min_bounds.x.min(vertex.x);
            min_bounds.z = min_bounds.z.min(vertex.z);
            max_bounds.x = max_bounds.x.max(vertex.x);
            max_bounds.z = max_bounds.z.max(vertex.z);
        }

        (min_bounds, max_bounds)
    }

    /// Clips a polygon against this convex volume
    pub fn clip_polygon(&self, polygon: &[Vec3], clipped: &mut Vec<Vec3>) -> bool {
        clipped.clear();

        if polygon.is_empty() {
            return false;
        }

        // Start with the input polygon
        let mut temp_poly: Vec<Vec3> = polygon.to_vec();

        // Clip against each edge of the convex volume
        for i in 0..self.vertices.len() {
            if temp_poly.is_empty() {
                return false;
            }

            let j = (i + 1) % self.vertices.len();
            let edge_start = &self.vertices[i];
            let edge_end = &self.vertices[j];

            // Clip temp_poly against this edge
            let mut new_poly = Vec::new();
            self.clip_polygon_by_edge(&temp_poly, edge_start, edge_end, &mut new_poly);

            temp_poly = new_poly;
        }

        // Store the result
        *clipped = temp_poly;
        !clipped.is_empty()
    }

    /// Clips a polygon against a single edge using Sutherland-Hodgman algorithm
    fn clip_polygon_by_edge(
        &self,
        polygon: &[Vec3],
        edge_start: &Vec3,
        edge_end: &Vec3,
        output: &mut Vec<Vec3>,
    ) {
        output.clear();

        if polygon.is_empty() {
            return;
        }

        // Edge vector
        let edge_dx = edge_end.x - edge_start.x;
        let edge_dz = edge_end.z - edge_start.z;

        for i in 0..polygon.len() {
            let j = (i + 1) % polygon.len();
            let v1 = &polygon[i];
            let v2 = &polygon[j];

            // Check which side of the edge each vertex is on
            let side1 = (v1.x - edge_start.x) * edge_dz - (v1.z - edge_start.z) * edge_dx;
            let side2 = (v2.x - edge_start.x) * edge_dz - (v2.z - edge_start.z) * edge_dx;

            if side1 >= 0.0 {
                // v1 is inside
                output.push(*v1);

                if side2 < 0.0 {
                    // v2 is outside, add intersection point
                    if let Some(intersection) =
                        self.line_edge_intersection(v1, v2, edge_start, edge_end)
                    {
                        output.push(intersection);
                    }
                }
            } else if side2 >= 0.0 {
                // v1 is outside, v2 is inside, add intersection point
                if let Some(intersection) =
                    self.line_edge_intersection(v1, v2, edge_start, edge_end)
                {
                    output.push(intersection);
                }
            }
        }
    }

    /// Finds the intersection point between a line segment and an edge
    fn line_edge_intersection(
        &self,
        p1: &Vec3,
        p2: &Vec3,
        edge_start: &Vec3,
        edge_end: &Vec3,
    ) -> Option<Vec3> {
        let dx1 = p2.x - p1.x;
        let dz1 = p2.z - p1.z;
        let dx2 = edge_end.x - edge_start.x;
        let dz2 = edge_end.z - edge_start.z;

        let denominator = dx1 * dz2 - dz1 * dx2;

        if denominator.abs() < f32::EPSILON {
            // Lines are parallel
            return None;
        }

        let dx3 = edge_start.x - p1.x;
        let dz3 = edge_start.z - p1.z;

        let t = (dx3 * dz2 - dz3 * dx2) / denominator;

        if (0.0..=1.0).contains(&t) {
            // Interpolate between p1 and p2
            Some(Vec3::new(
                p1.x + t * dx1,
                p1.y + t * (p2.y - p1.y),
                p1.z + t * dz1,
            ))
        } else {
            None
        }
    }
}

/// Collection of convex volumes for batch processing
#[derive(Debug, Clone, Default)]
pub struct ConvexVolumeSet {
    /// List of convex volumes
    pub volumes: Vec<ConvexVolume>,
}

impl ConvexVolumeSet {
    /// Creates a new empty convex volume set
    pub fn new() -> Self {
        Self {
            volumes: Vec::new(),
        }
    }

    /// Adds a convex volume to the set
    pub fn add_volume(&mut self, volume: ConvexVolume) {
        self.volumes.push(volume);
    }

    /// Finds all volumes that contain the given point
    pub fn find_volumes_at_point(&self, point: Vec3) -> Vec<usize> {
        let mut result = Vec::new();

        for (idx, volume) in self.volumes.iter().enumerate() {
            if volume.contains_point(point) {
                result.push(idx);
            }
        }

        result
    }

    /// Gets the highest priority area type at the given point
    pub fn get_area_at_point(&self, point: Vec3, default_area: u8) -> u8 {
        let mut area = default_area;

        // Later volumes in the list have higher priority
        for volume in &self.volumes {
            if volume.contains_point(point) {
                area = volume.area_type;
            }
        }

        area
    }

    /// Applies convex volumes to modify area types in a heightfield
    pub fn apply_to_heightfield(
        &self,
        heightfield: &mut super::heightfield::Heightfield,
    ) -> Result<()> {
        let cell_size = heightfield.cs;
        let cell_height = heightfield.ch;
        let origin = heightfield.bmin;

        // Process each cell in the heightfield
        for z in 0..heightfield.height {
            for x in 0..heightfield.width {
                // Calculate world position of cell center
                let world_x = origin.x + (x as f32 + 0.5) * cell_size;
                let world_z = origin.z + (z as f32 + 0.5) * cell_size;

                // Get the span column for this cell
                if let Some(Some(span_ref)) = heightfield.spans.get(&(x, z)) {
                    // Process each span in the column
                    let mut current_span = Some(Rc::clone(span_ref));

                    while let Some(span_rc) = current_span {
                        let mut span = span_rc.borrow_mut();

                        // Calculate world height of span top
                        let world_y = origin.y + span.max as f32 * cell_height;
                        let point = Vec3::new(world_x, world_y, world_z);

                        // Apply area from convex volumes
                        span.area = self.get_area_at_point(point, span.area);

                        // Move to next span
                        current_span = span.next.as_ref().map(Rc::clone);
                    }
                }
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convex_volume_creation() {
        // Test valid convex volume
        let vertices = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 10.0),
            Vec3::new(0.0, 0.0, 10.0),
        ];

        let volume = ConvexVolume::new(vertices, 0.0, 5.0, 1);
        assert!(volume.is_ok());

        // Test too few vertices
        let vertices = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0)];

        let volume = ConvexVolume::new(vertices, 0.0, 5.0, 1);
        assert!(volume.is_err());
    }

    #[test]
    fn test_box_volume() {
        let center = Vec3::new(5.0, 2.5, 5.0);
        let half_extents = Vec3::new(5.0, 2.5, 5.0);

        let volume = ConvexVolume::from_box(center, half_extents, 2).unwrap();

        // Test points inside
        assert!(volume.contains_point(Vec3::new(5.0, 2.5, 5.0))); // Center
        assert!(volume.contains_point(Vec3::new(1.0, 1.0, 1.0))); // Corner
        assert!(volume.contains_point(Vec3::new(9.0, 4.0, 9.0))); // Opposite corner

        // Test points outside
        assert!(!volume.contains_point(Vec3::new(-1.0, 2.5, 5.0))); // Outside X
        assert!(!volume.contains_point(Vec3::new(5.0, 6.0, 5.0))); // Above
        assert!(!volume.contains_point(Vec3::new(5.0, -1.0, 5.0))); // Below
    }

    #[test]
    fn test_cylinder_volume() {
        let center = Vec3::new(0.0, 5.0, 0.0);
        let radius = 10.0;
        let height = 10.0;

        let volume = ConvexVolume::from_cylinder(center, radius, height, 8, 3).unwrap();

        // Test center point
        assert!(volume.contains_point(center));

        // Test point at edge of cylinder
        assert!(volume.contains_point(Vec3::new(9.0, 5.0, 0.0)));

        // Test point outside cylinder
        assert!(!volume.contains_point(Vec3::new(15.0, 5.0, 0.0)));

        // Test height bounds
        assert!(volume.contains_point(Vec3::new(0.0, 0.1, 0.0)));
        assert!(volume.contains_point(Vec3::new(0.0, 9.9, 0.0)));
        assert!(!volume.contains_point(Vec3::new(0.0, -0.1, 0.0)));
        assert!(!volume.contains_point(Vec3::new(0.0, 10.1, 0.0)));
    }

    #[test]
    fn test_convex_check() {
        // Test convex polygon
        let convex_verts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 2.0),
            Vec3::new(0.0, 0.0, 2.0),
        ];
        assert!(ConvexVolume::is_convex(&convex_verts));

        // Test non-convex polygon (L-shape)
        let non_convex_verts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 2.0),
            Vec3::new(0.0, 0.0, 2.0),
        ];
        assert!(!ConvexVolume::is_convex(&non_convex_verts));
    }

    #[test]
    fn test_volume_set() {
        let mut set = ConvexVolumeSet::new();

        // Add overlapping volumes with different area types
        let vol1 =
            ConvexVolume::from_box(Vec3::new(5.0, 0.0, 5.0), Vec3::new(5.0, 5.0, 5.0), 1).unwrap();

        let vol2 =
            ConvexVolume::from_box(Vec3::new(7.0, 0.0, 7.0), Vec3::new(3.0, 5.0, 3.0), 2).unwrap();

        set.add_volume(vol1);
        set.add_volume(vol2);

        // Test non-overlapping point
        assert_eq!(set.get_area_at_point(Vec3::new(1.0, 0.0, 1.0), 0), 1);

        // Test overlapping point (should get area from vol2 since it's added later)
        assert_eq!(set.get_area_at_point(Vec3::new(7.0, 0.0, 7.0), 0), 2);

        // Test point outside all volumes
        assert_eq!(set.get_area_at_point(Vec3::new(20.0, 0.0, 20.0), 0), 0);
    }
}
