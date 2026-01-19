//! Mesh utilities for 

use crate::Result;
use glam::Vec3;
use std::fs::File;
use std::io::{self, BufRead, BufReader};
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
    pub fn from_obj<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let mut mesh = Self::new();

        for line in reader.lines() {
            let line = line?;
            let mut tokens = line.split_whitespace();

            match tokens.next() {
                Some("v") => {
                    // Vertex
                    let x = tokens
                        .next()
                        .ok_or_else(|| {
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid vertex: missing x coordinate",
                            )
                        })?
                        .parse::<f32>()
                        .map_err(|_| {
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid vertex: x coordinate is not a number",
                            )
                        })?;

                    let y = tokens
                        .next()
                        .ok_or_else(|| {
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid vertex: missing y coordinate",
                            )
                        })?
                        .parse::<f32>()
                        .map_err(|_| {
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid vertex: y coordinate is not a number",
                            )
                        })?;

                    let z = tokens
                        .next()
                        .ok_or_else(|| {
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid vertex: missing z coordinate",
                            )
                        })?
                        .parse::<f32>()
                        .map_err(|_| {
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid vertex: z coordinate is not a number",
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
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid face: missing vertex index",
                            )
                        })?;

                        let index = index_str.parse::<i32>().map_err(|_| {
                            io::Error::new(
                                io::ErrorKind::InvalidData,
                                "Invalid face: vertex index is not a number",
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
                    // Skip other lines (normals, texture coordinates, etc.)
                }
            }
        }

        Ok(mesh)
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
