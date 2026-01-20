//! Mesh utilities for triangle mesh handling

use crate::Result;
use glam::Vec3;

#[cfg(feature = "std")]
use std::fs::File;
#[cfg(feature = "std")]
use std::io::{BufRead, BufReader};
#[cfg(feature = "std")]
use std::path::Path;

/// A simple triangle mesh
#[derive(Debug, Clone, Default)]
pub struct TriMesh {
    /// The vertices of the mesh as a flat array of [x, y, z] coordinates
    pub vertices: Vec<f32>,
    /// The indices of the mesh, 3 per triangle
    pub indices: Vec<i32>,
    /// The number of vertices in the mesh
    pub vert_count: usize,
    /// The number of triangles in the mesh
    pub tri_count: usize,
}

impl TriMesh {
    /// Creates a new empty triangle mesh
    pub fn new() -> Self {
        Self::default()
    }

    /// Loads a mesh from an OBJ file
    ///
    /// This method is only available when the `std` feature is enabled.
    #[cfg(feature = "std")]
    pub fn from_obj<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let mut mesh = Self::new();

        for line in reader.lines() {
            let line = line?;
            Self::parse_obj_line(&line, &mut mesh)?;
        }

        Ok(mesh)
    }

    /// Parses OBJ content from a string
    ///
    /// This method is WASM-compatible and can parse OBJ data that has been
    /// loaded into memory through other means (e.g., fetch API in browser).
    ///
    /// # Example
    ///
    /// ```
    /// use recast_common::TriMesh;
    ///
    /// let obj_content = r#"
    /// v 0.0 0.0 0.0
    /// v 1.0 0.0 0.0
    /// v 0.5 1.0 0.0
    /// f 1 2 3
    /// "#;
    ///
    /// let mesh = TriMesh::from_obj_str(obj_content).unwrap();
    /// assert_eq!(mesh.vert_count, 3);
    /// assert_eq!(mesh.tri_count, 1);
    /// ```
    pub fn from_obj_str(content: &str) -> Result<Self> {
        let mut mesh = Self::new();

        for line in content.lines() {
            Self::parse_obj_line(line, &mut mesh)?;
        }

        Ok(mesh)
    }

    /// Parses a single line from an OBJ file
    fn parse_obj_line(line: &str, mesh: &mut Self) -> Result<()> {
        let mut tokens = line.split_whitespace();

        match tokens.next() {
            Some("v") => {
                // Vertex
                let x = tokens
                    .next()
                    .ok_or_else(|| {
                        crate::Error::InvalidMesh(
                            "Invalid vertex: missing x coordinate".to_string(),
                        )
                    })?
                    .parse::<f32>()
                    .map_err(|_| {
                        crate::Error::InvalidMesh(
                            "Invalid vertex: x coordinate is not a number".to_string(),
                        )
                    })?;

                let y = tokens
                    .next()
                    .ok_or_else(|| {
                        crate::Error::InvalidMesh(
                            "Invalid vertex: missing y coordinate".to_string(),
                        )
                    })?
                    .parse::<f32>()
                    .map_err(|_| {
                        crate::Error::InvalidMesh(
                            "Invalid vertex: y coordinate is not a number".to_string(),
                        )
                    })?;

                let z = tokens
                    .next()
                    .ok_or_else(|| {
                        crate::Error::InvalidMesh(
                            "Invalid vertex: missing z coordinate".to_string(),
                        )
                    })?
                    .parse::<f32>()
                    .map_err(|_| {
                        crate::Error::InvalidMesh(
                            "Invalid vertex: z coordinate is not a number".to_string(),
                        )
                    })?;

                mesh.vertices.push(x);
                mesh.vertices.push(y);
                mesh.vertices.push(z);
                mesh.vert_count += 1;
            }
            Some("f") => {
                // Face
                let mut face_indices = Vec::new();

                for token in tokens {
                    let index_str = token.split('/').next().ok_or_else(|| {
                        crate::Error::InvalidMesh("Invalid face: missing vertex index".to_string())
                    })?;

                    let index = index_str.parse::<i32>().map_err(|_| {
                        crate::Error::InvalidMesh(
                            "Invalid face: vertex index is not a number".to_string(),
                        )
                    })? - 1; // OBJ indices are 1-based

                    face_indices.push(index);
                }

                if face_indices.len() < 3 {
                    return Err(crate::Error::InvalidMesh(
                        "Invalid face: less than 3 vertices".to_string(),
                    ));
                }

                // Triangulate the face if it has more than 3 vertices (simple fan triangulation)
                for i in 1..(face_indices.len() - 1) {
                    mesh.indices.push(face_indices[0]);
                    mesh.indices.push(face_indices[i]);
                    mesh.indices.push(face_indices[i + 1]);
                    mesh.tri_count += 1;
                }
            }
            _ => {
                // Skip other lines (normals, texture coordinates, comments, etc.)
            }
        }

        Ok(())
    }

    /// Calculates the axis-aligned bounding box of the mesh
    pub fn calculate_bounds(&self) -> (Vec3, Vec3) {
        if self.vert_count == 0 {
            return (Vec3::ZERO, Vec3::ZERO);
        }

        let mut bmin = Vec3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut bmax = Vec3::new(f32::MIN, f32::MIN, f32::MIN);

        for i in 0..self.vert_count {
            let x = self.vertices[i * 3];
            let y = self.vertices[i * 3 + 1];
            let z = self.vertices[i * 3 + 2];

            bmin.x = bmin.x.min(x);
            bmin.y = bmin.y.min(y);
            bmin.z = bmin.z.min(z);

            bmax.x = bmax.x.max(x);
            bmax.y = bmax.y.max(y);
            bmax.z = bmax.z.max(z);
        }

        (bmin, bmax)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_obj_str_simple_triangle() {
        let obj = r#"
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.5 1.0 0.0
f 1 2 3
"#;
        let mesh = TriMesh::from_obj_str(obj).unwrap();
        assert_eq!(mesh.vert_count, 3);
        assert_eq!(mesh.tri_count, 1);
        assert_eq!(mesh.vertices.len(), 9);
        assert_eq!(mesh.indices.len(), 3);
    }

    #[test]
    fn test_from_obj_str_quad_triangulation() {
        let obj = r#"
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 1.0 1.0 0.0
v 0.0 1.0 0.0
f 1 2 3 4
"#;
        let mesh = TriMesh::from_obj_str(obj).unwrap();
        assert_eq!(mesh.vert_count, 4);
        assert_eq!(mesh.tri_count, 2); // Quad triangulated into 2 triangles
        assert_eq!(mesh.indices.len(), 6);
    }

    #[test]
    fn test_from_obj_str_with_texture_coords() {
        // OBJ with texture coordinates (v/vt format)
        let obj = r#"
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.5 1.0 0.0
vt 0.0 0.0
vt 1.0 0.0
vt 0.5 1.0
f 1/1 2/2 3/3
"#;
        let mesh = TriMesh::from_obj_str(obj).unwrap();
        assert_eq!(mesh.vert_count, 3);
        assert_eq!(mesh.tri_count, 1);
    }

    #[test]
    fn test_from_obj_str_with_normals() {
        // OBJ with normals (v/vt/vn format)
        let obj = r#"
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.5 1.0 0.0
vn 0.0 0.0 1.0
f 1//1 2//1 3//1
"#;
        let mesh = TriMesh::from_obj_str(obj).unwrap();
        assert_eq!(mesh.vert_count, 3);
        assert_eq!(mesh.tri_count, 1);
    }

    #[test]
    fn test_from_obj_str_skips_comments() {
        let obj = r#"
# This is a comment
v 0.0 0.0 0.0
# Another comment
v 1.0 0.0 0.0
v 0.5 1.0 0.0
f 1 2 3
"#;
        let mesh = TriMesh::from_obj_str(obj).unwrap();
        assert_eq!(mesh.vert_count, 3);
        assert_eq!(mesh.tri_count, 1);
    }

    #[test]
    fn test_from_obj_str_invalid_vertex() {
        let obj = "v 0.0 0.0"; // Missing z coordinate
        let result = TriMesh::from_obj_str(obj);
        assert!(result.is_err());
    }

    #[test]
    fn test_from_obj_str_invalid_face() {
        let obj = r#"
v 0.0 0.0 0.0
v 1.0 0.0 0.0
f 1 2
"#;
        let result = TriMesh::from_obj_str(obj);
        assert!(result.is_err());
    }
}
