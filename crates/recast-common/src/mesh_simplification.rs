//! Mesh simplification and preprocessing tools for improving performance
//!
//! This module provides various algorithms to reduce mesh complexity before rasterization,
//! which can significantly improve navigation mesh generation performance.

use super::mesh::TriMesh;
use crate::Result;
use glam::Vec3;
use std::collections::{HashMap, HashSet};

/// Configuration for mesh simplification operations
#[derive(Debug, Clone)]
pub struct SimplificationConfig {
    /// Target reduction ratio (0.0 = no reduction, 1.0 = remove everything)
    pub reduction_ratio: f32,
    /// Maximum allowed error tolerance for simplification
    pub max_error: f32,
    /// Whether to preserve mesh boundaries during simplification
    pub preserve_boundaries: bool,
    /// Whether to preserve topology (no holes created)
    pub preserve_topology: bool,
    /// Minimum triangle area threshold (smaller triangles removed)
    pub min_triangle_area: f32,
    /// Vertex welding threshold for duplicate removal
    pub vertex_weld_threshold: f32,
}

impl Default for SimplificationConfig {
    fn default() -> Self {
        Self {
            reduction_ratio: 0.5,         // 50% reduction
            max_error: 0.01,              // 1cm error tolerance
            preserve_boundaries: true,    // Keep mesh boundaries intact
            preserve_topology: true,      // Don't create holes
            min_triangle_area: 0.0001,    // Remove very small triangles
            vertex_weld_threshold: 0.001, // 1mm welding threshold
        }
    }
}

/// Result of a mesh simplification operation
#[derive(Debug)]
pub struct SimplificationResult {
    /// The simplified mesh
    pub mesh: TriMesh,
    /// Statistics about the simplification
    pub stats: SimplificationStats,
}

/// Statistics from mesh simplification
#[derive(Debug, Default)]
pub struct SimplificationStats {
    /// Original vertex count
    pub original_vertices: usize,
    /// Original triangle count
    pub original_triangles: usize,
    /// Final vertex count after simplification
    pub final_vertices: usize,
    /// Final triangle count after simplification
    pub final_triangles: usize,
    /// Number of vertices welded (merged)
    pub vertices_welded: usize,
    /// Number of degenerate triangles removed
    pub degenerate_removed: usize,
    /// Number of small triangles removed
    pub small_triangles_removed: usize,
    /// Maximum error introduced by simplification
    pub max_error_introduced: f32,
}

impl SimplificationStats {
    /// Calculates the reduction percentage achieved
    pub fn vertex_reduction_percent(&self) -> f32 {
        if self.original_vertices == 0 {
            return 0.0;
        }
        ((self.original_vertices - self.final_vertices) as f32 / self.original_vertices as f32)
            * 100.0
    }

    /// Calculates the triangle reduction percentage achieved
    pub fn triangle_reduction_percent(&self) -> f32 {
        if self.original_triangles == 0 {
            return 0.0;
        }
        ((self.original_triangles - self.final_triangles) as f32 / self.original_triangles as f32)
            * 100.0
    }
}

/// Edge in the mesh for edge collapse operations
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct Edge {
    v0: usize,
    v1: usize,
}

impl Edge {
    fn new(v0: usize, v1: usize) -> Self {
        // Ensure consistent ordering
        if v0 < v1 {
            Self { v0, v1 }
        } else {
            Self { v0: v1, v1: v0 }
        }
    }
}

/// Quadric error metric for vertex positioning
#[derive(Debug, Clone, Default)]
struct QuadricError {
    /// 4x4 quadric matrix flattened to 10 unique values (symmetric)
    matrix: [f64; 10],
}

impl QuadricError {
    /// Creates a new quadric from a triangle plane
    fn from_triangle(v0: &Vec3, v1: &Vec3, v2: &Vec3) -> Self {
        // Calculate triangle normal and plane equation ax + by + cz + d = 0
        let edge1 = *v1 - *v0;
        let edge2 = *v2 - *v0;
        let normal = edge1.cross(edge2).normalize();

        let a = normal.x as f64;
        let b = normal.y as f64;
        let c = normal.z as f64;
        let d = -(a * v0.x as f64 + b * v0.y as f64 + c * v0.z as f64);

        // Build quadric matrix from plane equation
        let mut matrix = [0.0; 10];
        matrix[0] = a * a; // a²
        matrix[1] = a * b; // ab
        matrix[2] = a * c; // ac
        matrix[3] = a * d; // ad
        matrix[4] = b * b; // b²
        matrix[5] = b * c; // bc
        matrix[6] = b * d; // bd
        matrix[7] = c * c; // c²
        matrix[8] = c * d; // cd
        matrix[9] = d * d; // d²

        Self { matrix }
    }

    /// Adds another quadric to this one
    fn add(&mut self, other: &QuadricError) {
        for i in 0..10 {
            self.matrix[i] += other.matrix[i];
        }
    }

    /// Calculates the error for a given vertex position
    fn error(&self, v: &Vec3) -> f64 {
        let x = v.x as f64;
        let y = v.y as f64;
        let z = v.z as f64;

        // Evaluate quadric: v^T * Q * v
        self.matrix[0] * x * x
            + 2.0 * self.matrix[1] * x * y
            + 2.0 * self.matrix[2] * x * z
            + 2.0 * self.matrix[3] * x
            + self.matrix[4] * y * y
            + 2.0 * self.matrix[5] * y * z
            + 2.0 * self.matrix[6] * y
            + self.matrix[7] * z * z
            + 2.0 * self.matrix[8] * z
            + self.matrix[9]
    }
}

/// Mesh simplifier with multiple algorithms
pub struct MeshSimplifier {
    config: SimplificationConfig,
}

impl MeshSimplifier {
    /// Creates a new mesh simplifier with the given configuration
    pub fn new(config: SimplificationConfig) -> Self {
        Self { config }
    }

    /// Creates a new mesh simplifier with default configuration
    pub fn with_defaults() -> Self {
        Self::new(SimplificationConfig::default())
    }

    /// Simplifies a mesh using the configured algorithm
    pub fn simplify(&self, mesh: &TriMesh) -> Result<SimplificationResult> {
        let mut stats = SimplificationStats {
            original_vertices: mesh.vert_count,
            original_triangles: mesh.tri_count,
            ..Default::default()
        };

        if mesh.tri_count == 0 {
            return Ok(SimplificationResult {
                mesh: mesh.clone(),
                stats,
            });
        }

        // Step 1: Preprocess mesh (weld vertices, remove degenerates)
        let mut simplified_mesh = self.preprocess_mesh(mesh, &mut stats)?;

        // Step 2: Apply main simplification algorithm
        if self.config.reduction_ratio > 0.0 {
            simplified_mesh = self.edge_collapse_simplification(&simplified_mesh, &mut stats)?;
        }

        // Step 3: Post-process (remove small triangles, final cleanup)
        simplified_mesh = self.postprocess_mesh(&simplified_mesh, &mut stats)?;

        stats.final_vertices = simplified_mesh.vert_count;
        stats.final_triangles = simplified_mesh.tri_count;

        Ok(SimplificationResult {
            mesh: simplified_mesh,
            stats,
        })
    }

    /// Preprocesses the mesh by welding vertices and removing degenerate triangles
    fn preprocess_mesh(&self, mesh: &TriMesh, stats: &mut SimplificationStats) -> Result<TriMesh> {
        // Step 1: Weld nearby vertices
        let (vertices, vertex_mapping) = self.weld_vertices(&mesh.vertices, mesh.vert_count)?;
        stats.vertices_welded = mesh.vert_count - vertices.len() / 3;

        // Step 2: Remap indices and remove degenerate triangles
        let mut indices = Vec::new();
        let mut triangle_count = 0;

        for i in 0..mesh.tri_count {
            let base = i * 3;
            let i0 = vertex_mapping[mesh.indices[base] as usize];
            let i1 = vertex_mapping[mesh.indices[base + 1] as usize];
            let i2 = vertex_mapping[mesh.indices[base + 2] as usize];

            // Skip degenerate triangles
            if i0 != i1 && i1 != i2 && i0 != i2 {
                // Check for valid triangle area
                let v0 = Vec3::new(vertices[i0 * 3], vertices[i0 * 3 + 1], vertices[i0 * 3 + 2]);
                let v1 = Vec3::new(vertices[i1 * 3], vertices[i1 * 3 + 1], vertices[i1 * 3 + 2]);
                let v2 = Vec3::new(vertices[i2 * 3], vertices[i2 * 3 + 1], vertices[i2 * 3 + 2]);

                let area = Self::triangle_area(&v0, &v1, &v2);
                if area >= self.config.min_triangle_area {
                    indices.push(i0 as i32);
                    indices.push(i1 as i32);
                    indices.push(i2 as i32);
                    triangle_count += 1;
                } else {
                    stats.small_triangles_removed += 1;
                }
            } else {
                stats.degenerate_removed += 1;
            }
        }

        let vert_count = vertices.len() / 3;
        Ok(TriMesh {
            vertices,
            indices,
            vert_count,
            tri_count: triangle_count,
        })
    }

    /// Welds nearby vertices together
    fn weld_vertices(&self, vertices: &[f32], vert_count: usize) -> Result<(Vec<f32>, Vec<usize>)> {
        let mut welded_vertices = Vec::new();
        let mut vertex_mapping = vec![0; vert_count];

        for i in 0..vert_count {
            let v = Vec3::new(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);

            // Try to find an existing vertex to weld with
            let mut found = false;
            for (j, existing_idx) in (0..welded_vertices.len() / 3).enumerate() {
                let existing = Vec3::new(
                    welded_vertices[existing_idx * 3],
                    welded_vertices[existing_idx * 3 + 1],
                    welded_vertices[existing_idx * 3 + 2],
                );

                if (v - existing).length() <= self.config.vertex_weld_threshold {
                    vertex_mapping[i] = j;
                    found = true;
                    break;
                }
            }

            if !found {
                let new_idx = welded_vertices.len() / 3;
                welded_vertices.push(v.x);
                welded_vertices.push(v.y);
                welded_vertices.push(v.z);
                vertex_mapping[i] = new_idx;
            }
        }

        Ok((welded_vertices, vertex_mapping))
    }

    /// Applies edge collapse simplification using quadric error metrics
    fn edge_collapse_simplification(
        &self,
        mesh: &TriMesh,
        stats: &mut SimplificationStats,
    ) -> Result<TriMesh> {
        if mesh.tri_count == 0 {
            return Ok(mesh.clone());
        }

        // Calculate target triangle count
        let target_triangles =
            ((mesh.tri_count as f32) * (1.0 - self.config.reduction_ratio)) as usize;
        if target_triangles >= mesh.tri_count {
            return Ok(mesh.clone());
        }

        // Build adjacency information
        let mut edge_to_triangles: HashMap<Edge, Vec<usize>> = HashMap::new();
        let mut vertex_to_triangles: HashMap<usize, Vec<usize>> = HashMap::new();

        for i in 0..mesh.tri_count {
            let base = i * 3;
            let v0 = mesh.indices[base] as usize;
            let v1 = mesh.indices[base + 1] as usize;
            let v2 = mesh.indices[base + 2] as usize;

            // Add edges
            for &(va, vb) in &[(v0, v1), (v1, v2), (v2, v0)] {
                let edge = Edge::new(va, vb);
                edge_to_triangles.entry(edge).or_default().push(i);
                vertex_to_triangles.entry(va).or_default().push(i);
                vertex_to_triangles.entry(vb).or_default().push(i);
            }
        }

        // Calculate quadric error metrics for each vertex
        let mut vertex_quadrics = vec![QuadricError::default(); mesh.vert_count];

        for i in 0..mesh.tri_count {
            let base = i * 3;
            let v0_idx = mesh.indices[base] as usize;
            let v1_idx = mesh.indices[base + 1] as usize;
            let v2_idx = mesh.indices[base + 2] as usize;

            let v0 = Vec3::new(
                mesh.vertices[v0_idx * 3],
                mesh.vertices[v0_idx * 3 + 1],
                mesh.vertices[v0_idx * 3 + 2],
            );
            let v1 = Vec3::new(
                mesh.vertices[v1_idx * 3],
                mesh.vertices[v1_idx * 3 + 1],
                mesh.vertices[v1_idx * 3 + 2],
            );
            let v2 = Vec3::new(
                mesh.vertices[v2_idx * 3],
                mesh.vertices[v2_idx * 3 + 1],
                mesh.vertices[v2_idx * 3 + 2],
            );

            let quadric = QuadricError::from_triangle(&v0, &v1, &v2);
            vertex_quadrics[v0_idx].add(&quadric);
            vertex_quadrics[v1_idx].add(&quadric);
            vertex_quadrics[v2_idx].add(&quadric);
        }

        // Build priority queue of edges to collapse
        let mut edge_costs: Vec<(f64, Edge, Vec3)> = Vec::new();

        for edge in edge_to_triangles.keys() {
            if self.config.preserve_boundaries {
                // Skip boundary edges (edges with only one adjacent triangle)
                if edge_to_triangles[edge].len() < 2 {
                    continue;
                }
            }

            let v0 = Vec3::new(
                mesh.vertices[edge.v0 * 3],
                mesh.vertices[edge.v0 * 3 + 1],
                mesh.vertices[edge.v0 * 3 + 2],
            );
            let v1 = Vec3::new(
                mesh.vertices[edge.v1 * 3],
                mesh.vertices[edge.v1 * 3 + 1],
                mesh.vertices[edge.v1 * 3 + 2],
            );

            // Calculate optimal collapse position (midpoint for simplicity)
            let collapse_pos = (v0 + v1) * 0.5;

            // Calculate collapse cost using quadric error
            let mut combined_quadric = vertex_quadrics[edge.v0].clone();
            combined_quadric.add(&vertex_quadrics[edge.v1]);
            let cost = combined_quadric.error(&collapse_pos);

            edge_costs.push((cost, edge.clone(), collapse_pos));
        }

        // Sort by cost (lowest first)
        edge_costs.sort_by(|a, b| a.0.total_cmp(&b.0));

        // Perform edge collapses until target is reached
        let mut active_triangles: HashSet<usize> = (0..mesh.tri_count).collect();
        let mut vertex_positions: Vec<Vec3> = (0..mesh.vert_count)
            .map(|i| {
                Vec3::new(
                    mesh.vertices[i * 3],
                    mesh.vertices[i * 3 + 1],
                    mesh.vertices[i * 3 + 2],
                )
            })
            .collect();

        let mut collapsed_edges = 0;
        let max_collapses = mesh.tri_count - target_triangles;

        for (cost, edge, collapse_pos) in edge_costs {
            if collapsed_edges >= max_collapses {
                break;
            }

            if cost > self.config.max_error as f64 {
                break;
            }

            // Check if edge is still valid (both vertices exist)
            if edge.v0 >= vertex_positions.len() || edge.v1 >= vertex_positions.len() {
                continue;
            }

            // Remove triangles that use this edge
            if let Some(triangles) = edge_to_triangles.get(&edge) {
                for &tri_idx in triangles {
                    active_triangles.remove(&tri_idx);
                }
                collapsed_edges += triangles.len();
            }

            // Update vertex position
            vertex_positions[edge.v0] = collapse_pos;

            // Mark second vertex as collapsed (reuse index for first vertex)
            // This is a simplified approach - a full implementation would need
            // more sophisticated vertex merging

            stats.max_error_introduced = stats.max_error_introduced.max(cost as f32);
        }

        // Rebuild mesh from active triangles
        let mut new_vertices = Vec::new();
        let mut new_indices = Vec::new();
        let mut vertex_map = HashMap::new();
        let mut next_vertex_idx: i32 = 0;

        for &tri_idx in &active_triangles {
            let base = tri_idx * 3;
            let mut triangle_indices: Vec<i32> = Vec::new();

            for offset in 0..3 {
                let old_idx = mesh.indices[base + offset] as usize;

                if let Some(&new_idx) = vertex_map.get(&old_idx) {
                    triangle_indices.push(new_idx);
                } else {
                    let pos = &vertex_positions[old_idx];
                    new_vertices.push(pos.x);
                    new_vertices.push(pos.y);
                    new_vertices.push(pos.z);
                    vertex_map.insert(old_idx, next_vertex_idx);
                    triangle_indices.push(next_vertex_idx);
                    next_vertex_idx += 1;
                }
            }

            // Add triangle if it's still valid
            if triangle_indices[0] != triangle_indices[1]
                && triangle_indices[1] != triangle_indices[2]
                && triangle_indices[0] != triangle_indices[2]
            {
                new_indices.push(triangle_indices[0]);
                new_indices.push(triangle_indices[1]);
                new_indices.push(triangle_indices[2]);
            }
        }

        let vert_count = new_vertices.len() / 3;
        let tri_count = new_indices.len() / 3;
        Ok(TriMesh {
            vertices: new_vertices,
            indices: new_indices,
            vert_count,
            tri_count,
        })
    }

    /// Post-processes the mesh with final cleanup
    fn postprocess_mesh(&self, mesh: &TriMesh, stats: &mut SimplificationStats) -> Result<TriMesh> {
        // Remove any remaining degenerate or small triangles
        let mut new_indices = Vec::new();
        let mut triangle_count = 0;

        for i in 0..mesh.tri_count {
            let base = i * 3;
            let i0 = mesh.indices[base] as usize;
            let i1 = mesh.indices[base + 1] as usize;
            let i2 = mesh.indices[base + 2] as usize;

            if i0 != i1 && i1 != i2 && i0 != i2 {
                let v0 = Vec3::new(
                    mesh.vertices[i0 * 3],
                    mesh.vertices[i0 * 3 + 1],
                    mesh.vertices[i0 * 3 + 2],
                );
                let v1 = Vec3::new(
                    mesh.vertices[i1 * 3],
                    mesh.vertices[i1 * 3 + 1],
                    mesh.vertices[i1 * 3 + 2],
                );
                let v2 = Vec3::new(
                    mesh.vertices[i2 * 3],
                    mesh.vertices[i2 * 3 + 1],
                    mesh.vertices[i2 * 3 + 2],
                );

                let area = Self::triangle_area(&v0, &v1, &v2);
                if area >= self.config.min_triangle_area {
                    new_indices.push(mesh.indices[base]);
                    new_indices.push(mesh.indices[base + 1]);
                    new_indices.push(mesh.indices[base + 2]);
                    triangle_count += 1;
                } else {
                    stats.small_triangles_removed += 1;
                }
            } else {
                stats.degenerate_removed += 1;
            }
        }

        Ok(TriMesh {
            vertices: mesh.vertices.clone(),
            indices: new_indices,
            vert_count: mesh.vert_count,
            tri_count: triangle_count,
        })
    }

    /// Calculates the area of a triangle
    fn triangle_area(v0: &Vec3, v1: &Vec3, v2: &Vec3) -> f32 {
        let edge1 = *v1 - *v0;
        let edge2 = *v2 - *v0;
        edge1.cross(edge2).length() * 0.5
    }
}

/// Utility functions for mesh preprocessing
impl MeshSimplifier {
    /// Analyzes mesh complexity and recommends simplification parameters
    pub fn analyze_mesh(mesh: &TriMesh) -> MeshComplexityAnalysis {
        let mut analysis = MeshComplexityAnalysis::default();

        if mesh.tri_count == 0 {
            return analysis;
        }

        analysis.vertex_count = mesh.vert_count;
        analysis.triangle_count = mesh.tri_count;

        // Calculate triangle areas
        let mut total_area = 0.0;
        let mut min_area = f32::MAX;
        let mut max_area = f32::MIN;
        let mut small_triangle_count = 0;

        for i in 0..mesh.tri_count {
            let base = i * 3;
            let v0 = Vec3::new(
                mesh.vertices[mesh.indices[base] as usize * 3],
                mesh.vertices[mesh.indices[base] as usize * 3 + 1],
                mesh.vertices[mesh.indices[base] as usize * 3 + 2],
            );
            let v1 = Vec3::new(
                mesh.vertices[mesh.indices[base + 1] as usize * 3],
                mesh.vertices[mesh.indices[base + 1] as usize * 3 + 1],
                mesh.vertices[mesh.indices[base + 1] as usize * 3 + 2],
            );
            let v2 = Vec3::new(
                mesh.vertices[mesh.indices[base + 2] as usize * 3],
                mesh.vertices[mesh.indices[base + 2] as usize * 3 + 1],
                mesh.vertices[mesh.indices[base + 2] as usize * 3 + 2],
            );

            let area = Self::triangle_area(&v0, &v1, &v2);
            total_area += area;
            min_area = min_area.min(area);
            max_area = max_area.max(area);

            if area < 0.001 {
                // Very small triangles
                small_triangle_count += 1;
            }
        }

        analysis.average_triangle_area = total_area / mesh.tri_count as f32;
        analysis.min_triangle_area = min_area;
        analysis.max_triangle_area = max_area;
        analysis.small_triangle_ratio = small_triangle_count as f32 / mesh.tri_count as f32;

        // Calculate mesh density (triangles per unit area)
        let (bmin, bmax) = mesh.calculate_bounds();
        let bbox_area = (bmax.x - bmin.x) * (bmax.z - bmin.z);
        if bbox_area > 0.0 {
            analysis.triangle_density = mesh.tri_count as f32 / bbox_area;
        }

        // Recommend simplification based on analysis
        analysis.recommended_reduction = analysis.calculate_recommended_reduction();

        analysis
    }
}

/// Analysis of mesh complexity for recommending simplification parameters
#[derive(Debug, Default)]
pub struct MeshComplexityAnalysis {
    pub vertex_count: usize,
    pub triangle_count: usize,
    pub average_triangle_area: f32,
    pub min_triangle_area: f32,
    pub max_triangle_area: f32,
    pub small_triangle_ratio: f32,
    pub triangle_density: f32,
    pub recommended_reduction: f32,
}

impl MeshComplexityAnalysis {
    /// Calculates recommended reduction ratio based on mesh characteristics
    fn calculate_recommended_reduction(&self) -> f32 {
        let mut reduction: f32 = 0.0;

        // High triangle count suggests aggressive reduction
        if self.triangle_count > 100_000 {
            reduction += 0.7;
        } else if self.triangle_count > 50_000 {
            reduction += 0.5;
        } else if self.triangle_count > 10_000 {
            reduction += 0.3;
        }

        // High density of small triangles suggests reduction
        if self.small_triangle_ratio > 0.3 {
            reduction += 0.3;
        } else if self.small_triangle_ratio > 0.1 {
            reduction += 0.2;
        }

        // High triangle density suggests reduction
        if self.triangle_density > 1000.0 {
            reduction += 0.2;
        } else if self.triangle_density > 500.0 {
            reduction += 0.1;
        }

        reduction.min(0.8) // Cap at 80% reduction
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_mesh() -> TriMesh {
        // Create a simple pyramid mesh
        let mut mesh = TriMesh::new();

        // Vertices (pyramid with square base)
        let vertices = vec![
            0.0, 0.0, 0.0, // 0: base corner
            1.0, 0.0, 0.0, // 1: base corner
            1.0, 0.0, 1.0, // 2: base corner
            0.0, 0.0, 1.0, // 3: base corner
            0.5, 1.0, 0.5, // 4: apex
        ];

        // Triangles
        let indices = vec![
            // Base (2 triangles)
            0, 1, 2, 0, 2, 3, // Sides (4 triangles)
            0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0,
        ];

        mesh.vertices = vertices;
        mesh.indices = indices;
        mesh.vert_count = 5;
        mesh.tri_count = 6;

        mesh
    }

    #[test]
    fn test_mesh_simplification_basic() {
        let simplifier = MeshSimplifier::with_defaults();
        let mesh = create_test_mesh();

        let result = simplifier.simplify(&mesh).unwrap();

        // Should have some reduction
        assert!(result.stats.final_triangles <= result.stats.original_triangles);
        assert!(result.stats.final_vertices <= result.stats.original_vertices);
    }

    #[test]
    fn test_vertex_welding() {
        let config = SimplificationConfig {
            vertex_weld_threshold: 0.1,
            reduction_ratio: 0.0, // No edge collapse, just welding
            ..Default::default()
        };
        let simplifier = MeshSimplifier::new(config);

        let mut mesh = create_test_mesh();

        // Add duplicate vertices
        mesh.vertices.extend_from_slice(&[0.05, 0.0, 0.0]); // Close to vertex 0
        mesh.vert_count += 1;

        let result = simplifier.simplify(&mesh).unwrap();

        assert!(result.stats.vertices_welded > 0);
    }

    #[test]
    fn test_mesh_analysis() {
        let mesh = create_test_mesh();
        let analysis = MeshSimplifier::analyze_mesh(&mesh);

        assert_eq!(analysis.vertex_count, 5);
        assert_eq!(analysis.triangle_count, 6);
        assert!(analysis.average_triangle_area > 0.0);
        assert!(analysis.recommended_reduction >= 0.0);
    }

    #[test]
    fn test_degenerate_triangle_removal() {
        let simplifier = MeshSimplifier::with_defaults();

        let mut mesh = TriMesh::new();
        mesh.vertices = vec![
            0.0, 0.0, 0.0, // 0
            1.0, 0.0, 0.0, // 1
            0.0, 1.0, 0.0, // 2
        ];
        mesh.indices = vec![
            0, 1, 2, // Valid triangle
            0, 0, 1, // Degenerate (repeated vertex)
        ];
        mesh.vert_count = 3;
        mesh.tri_count = 2;

        let result = simplifier.simplify(&mesh).unwrap();

        assert!(result.stats.degenerate_removed > 0);
        assert_eq!(result.stats.final_triangles, 1);
    }

    #[test]
    fn test_small_triangle_removal() {
        let config = SimplificationConfig {
            min_triangle_area: 0.1, // Remove triangles smaller than 0.1
            reduction_ratio: 0.0,   // No edge collapse
            ..Default::default()
        };
        let simplifier = MeshSimplifier::new(config);

        let mut mesh = TriMesh::new();
        mesh.vertices = vec![
            0.0, 0.0, 0.0, // 0
            1.0, 0.0, 0.0, // 1
            0.0, 1.0, 0.0, // 2
            0.01, 0.01, 0.0, // 3 (very close to 0)
        ];
        mesh.indices = vec![
            0, 1, 2, // Large triangle
            0, 1, 3, // Very small triangle
        ];
        mesh.vert_count = 4;
        mesh.tri_count = 2;

        let result = simplifier.simplify(&mesh).unwrap();

        assert!(result.stats.small_triangles_removed > 0);
    }
}
