//! Mesh merging operations for combining multiple navigation meshes
//!
//! This module provides functionality to merge multiple polygon meshes into a single
//! unified navigation mesh, handling vertex welding, polygon connectivity, and topology.

use super::polymesh::{PolyMesh, MESH_NULL_IDX};
use glam::Vec3;
use recast_common::{Error, Result};
use std::collections::HashMap;

/// Configuration for mesh merging operations
#[derive(Debug, Clone)]
pub struct MeshMergeConfig {
    /// Maximum distance for vertex welding (vertices closer than this are merged)
    pub weld_threshold: f32,
    /// Whether to remove duplicate polygons
    pub remove_duplicates: bool,
    /// Whether to merge coplanar adjacent polygons
    pub merge_coplanar: bool,
    /// Tolerance for coplanarity testing
    pub coplanar_tolerance: f32,
}

impl Default for MeshMergeConfig {
    fn default() -> Self {
        Self {
            weld_threshold: 0.001,
            remove_duplicates: true,
            merge_coplanar: false,
            coplanar_tolerance: 0.01,
        }
    }
}

/// Result of a mesh merge operation
#[derive(Debug)]
pub struct MergeResult {
    /// The merged polygon mesh
    pub mesh: PolyMesh,
    /// Statistics about the merge operation
    pub stats: MergeStats,
}

/// Statistics from a mesh merge operation
#[derive(Debug, Default)]
pub struct MergeStats {
    /// Number of input meshes
    pub input_mesh_count: usize,
    /// Total input vertices before merging
    pub input_vertex_count: usize,
    /// Total input polygons before merging
    pub input_polygon_count: usize,
    /// Output vertices after merging
    pub output_vertex_count: usize,
    /// Output polygons after merging
    pub output_polygon_count: usize,
    /// Number of vertices welded (merged)
    pub vertices_welded: usize,
    /// Number of duplicate polygons removed
    pub duplicates_removed: usize,
    /// Number of coplanar polygons merged
    pub coplanar_merged: usize,
}

/// Mesh merger for combining multiple polygon meshes
pub struct MeshMerger {
    config: MeshMergeConfig,
}

impl MeshMerger {
    /// Creates a new mesh merger with the given configuration
    pub fn new(config: MeshMergeConfig) -> Self {
        Self { config }
    }

    /// Creates a new mesh merger with default configuration
    pub fn with_defaults() -> Self {
        Self::new(MeshMergeConfig::default())
    }

    /// Merges multiple polygon meshes into a single mesh
    pub fn merge_meshes(&self, meshes: &[&PolyMesh]) -> Result<MergeResult> {
        if meshes.is_empty() {
            return Err(Error::NavMeshGeneration(
                "Cannot merge empty mesh list".to_string(),
            ));
        }

        let mut stats = MergeStats {
            input_mesh_count: meshes.len(),
            ..Default::default()
        };

        // Collect input statistics
        for mesh in meshes {
            stats.input_vertex_count += mesh.vert_count;
            stats.input_polygon_count += mesh.poly_count;
        }

        // Validate mesh compatibility
        self.validate_mesh_compatibility(meshes)?;

        // Create the merged mesh structure
        let mut merged_mesh = self.create_base_mesh(meshes[0])?;

        // Step 1: Collect and weld vertices
        let vertex_mapping =
            self.collect_and_weld_vertices(meshes, &mut merged_mesh, &mut stats)?;

        // Step 2: Merge polygons using the vertex mapping
        self.merge_polygons(meshes, &mut merged_mesh, &vertex_mapping, &mut stats)?;

        // Step 3: Remove duplicate polygons if requested
        if self.config.remove_duplicates {
            self.remove_duplicate_polygons(&mut merged_mesh, &mut stats)?;
        }

        // Step 4: Merge coplanar polygons if requested
        if self.config.merge_coplanar {
            self.merge_coplanar_polygons(&mut merged_mesh, &mut stats)?;
        }

        // Update final statistics
        stats.output_vertex_count = merged_mesh.vert_count;
        stats.output_polygon_count = merged_mesh.poly_count;

        Ok(MergeResult {
            mesh: merged_mesh,
            stats,
        })
    }

    /// Validates that meshes are compatible for merging
    fn validate_mesh_compatibility(&self, meshes: &[&PolyMesh]) -> Result<()> {
        if meshes.is_empty() {
            return Ok(());
        }

        let reference = meshes[0];

        for (i, mesh) in meshes.iter().enumerate().skip(1) {
            // Check cell size compatibility
            if (mesh.cs - reference.cs).abs() > f32::EPSILON {
                return Err(Error::NavMeshGeneration(format!(
                    "Mesh {} has incompatible cell size: {} vs {}",
                    i, mesh.cs, reference.cs
                )));
            }

            // Check cell height compatibility
            if (mesh.ch - reference.ch).abs() > f32::EPSILON {
                return Err(Error::NavMeshGeneration(format!(
                    "Mesh {} has incompatible cell height: {} vs {}",
                    i, mesh.ch, reference.ch
                )));
            }

            // Check max vertices per polygon compatibility
            if mesh.max_verts_per_poly != reference.max_verts_per_poly {
                return Err(Error::NavMeshGeneration(format!(
                    "Mesh {} has incompatible max vertices per polygon: {} vs {}",
                    i, mesh.max_verts_per_poly, reference.max_verts_per_poly
                )));
            }
        }

        Ok(())
    }

    /// Creates the base mesh structure for merging
    fn create_base_mesh(&self, reference: &PolyMesh) -> Result<PolyMesh> {
        let mut merged = PolyMesh::new(reference.max_verts_per_poly, reference.border_size);

        // Copy basic properties from reference mesh
        merged.cs = reference.cs;
        merged.ch = reference.ch;
        merged.bmin = reference.bmin;
        merged.bmax = reference.bmax;

        Ok(merged)
    }

    /// Collects vertices from all meshes and welds nearby vertices
    fn collect_and_weld_vertices(
        &self,
        meshes: &[&PolyMesh],
        merged_mesh: &mut PolyMesh,
        stats: &mut MergeStats,
    ) -> Result<Vec<Vec<u16>>> {
        let mut all_vertices: Vec<Vec3> = Vec::new();
        let mut vertex_mappings = Vec::new();

        // Collect all vertices with their source mesh information
        for mesh in meshes.iter() {
            let mut mesh_mapping = Vec::new();

            for vert_idx in 0..mesh.vert_count {
                let base_idx = vert_idx * 3;
                let vertex = Vec3::new(
                    mesh.vertices[base_idx] as f32,
                    mesh.vertices[base_idx + 1] as f32,
                    mesh.vertices[base_idx + 2] as f32,
                );

                // Try to find an existing vertex to weld with
                let mut found_existing = false;
                for (existing_idx, existing_vertex) in all_vertices.iter().enumerate() {
                    let dx = vertex[0] - existing_vertex[0];
                    let dy = vertex[1] - existing_vertex[1];
                    let dz = vertex[2] - existing_vertex[2];
                    let distance = (dx * dx + dy * dy + dz * dz).sqrt();
                    if distance <= self.config.weld_threshold {
                        // Weld with existing vertex
                        mesh_mapping.push(existing_idx as u16);
                        stats.vertices_welded += 1;
                        found_existing = true;
                        break;
                    }
                }

                if !found_existing {
                    // Add as new vertex
                    let new_idx = all_vertices.len();
                    all_vertices.push(vertex);
                    mesh_mapping.push(new_idx as u16);
                }
            }

            vertex_mappings.push(mesh_mapping);
        }

        // Store the welded vertices in the merged mesh
        merged_mesh.vert_count = all_vertices.len();
        merged_mesh.vertices = Vec::with_capacity(merged_mesh.vert_count * 3);

        for vertex in &all_vertices {
            merged_mesh.vertices.push(vertex.x as u16);
            merged_mesh.vertices.push(vertex.y as u16);
            merged_mesh.vertices.push(vertex.z as u16);
        }

        // Update bounding box to encompass all vertices
        if !all_vertices.is_empty() {
            let first = &all_vertices[0];
            let mut min_bounds = *first;
            let mut max_bounds = *first;

            for vertex in &all_vertices[1..] {
                min_bounds[0] = min_bounds[0].min(vertex[0]);
                min_bounds[1] = min_bounds[1].min(vertex[1]);
                min_bounds[2] = min_bounds[2].min(vertex[2]);

                max_bounds[0] = max_bounds[0].max(vertex[0]);
                max_bounds[1] = max_bounds[1].max(vertex[1]);
                max_bounds[2] = max_bounds[2].max(vertex[2]);
            }

            merged_mesh.bmin = min_bounds;
            merged_mesh.bmax = max_bounds;
        }

        Ok(vertex_mappings)
    }

    /// Merges polygons from all input meshes using the vertex mapping
    fn merge_polygons(
        &self,
        meshes: &[&PolyMesh],
        merged_mesh: &mut PolyMesh,
        vertex_mappings: &[Vec<u16>],
        _stats: &mut MergeStats,
    ) -> Result<()> {
        // Pre-allocate capacity for polygons
        let total_polys: usize = meshes.iter().map(|m| m.poly_count).sum();
        let nvp = merged_mesh.max_verts_per_poly;

        merged_mesh.polys.reserve(total_polys * nvp);
        merged_mesh.regs.reserve(total_polys);
        merged_mesh.areas.reserve(total_polys);

        // Copy polygons from each mesh, updating vertex indices
        for (mesh_idx, mesh) in meshes.iter().enumerate() {
            let vertex_mapping = &vertex_mappings[mesh_idx];

            for poly_idx in 0..mesh.poly_count {
                // Extract polygon vertices
                let poly_base = poly_idx * nvp;
                let mut remapped_vertices = Vec::new();

                for vert_offset in 0..nvp {
                    let vert_idx = mesh.polys[poly_base + vert_offset];
                    if vert_idx == MESH_NULL_IDX {
                        break;
                    }

                    // Remap vertex index to merged mesh space
                    let remapped_idx = vertex_mapping[vert_idx as usize];
                    remapped_vertices.push(remapped_idx);
                }

                // Add the polygon with remapped vertices
                if !remapped_vertices.is_empty() {
                    let region = mesh.regs[poly_idx];
                    let area = mesh.areas[poly_idx];

                    // Add polygon data directly since add_polygon doesn't exist
                    for &vertex_id in &remapped_vertices {
                        merged_mesh.polys.push(vertex_id);
                    }
                    // Pad with MESH_NULL_IDX if needed
                    while merged_mesh.polys.len() % nvp != 0 {
                        merged_mesh.polys.push(super::polymesh::MESH_NULL_IDX);
                    }
                    merged_mesh.regs.push(region);
                    merged_mesh.areas.push(area);
                    merged_mesh.poly_count += 1;
                }
            }
        }

        Ok(())
    }

    /// Removes duplicate polygons from the merged mesh
    fn remove_duplicate_polygons(
        &self,
        merged_mesh: &mut PolyMesh,
        stats: &mut MergeStats,
    ) -> Result<()> {
        let nvp = merged_mesh.max_verts_per_poly;
        let mut unique_polygons = HashMap::new();
        let mut keep_indices = Vec::new();

        // Identify unique polygons
        for poly_idx in 0..merged_mesh.poly_count {
            let poly_base = poly_idx * nvp;

            // Extract and normalize polygon (sort vertices to handle different winding)
            let mut poly_verts = Vec::new();
            for vert_offset in 0..nvp {
                let vert_idx = merged_mesh.polys[poly_base + vert_offset];
                if vert_idx == MESH_NULL_IDX {
                    break;
                }
                poly_verts.push(vert_idx);
            }

            // Sort vertices to create a canonical representation
            poly_verts.sort_unstable();
            let poly_key = (
                poly_verts.clone(),
                merged_mesh.regs[poly_idx],
                merged_mesh.areas[poly_idx],
            );

            match unique_polygons.entry(poly_key) {
                std::collections::hash_map::Entry::Occupied(_) => {
                    stats.duplicates_removed += 1;
                }
                std::collections::hash_map::Entry::Vacant(entry) => {
                    entry.insert(poly_idx);
                    keep_indices.push(poly_idx);
                }
            }
        }

        // Rebuild polygon arrays with only unique polygons
        if stats.duplicates_removed > 0 {
            let mut new_polys = Vec::new();
            let mut new_regions = Vec::new();
            let mut new_areas = Vec::new();

            for &keep_idx in &keep_indices {
                let poly_base = keep_idx * nvp;

                // Copy polygon vertices
                for vert_offset in 0..nvp {
                    new_polys.push(merged_mesh.polys[poly_base + vert_offset]);
                }

                // Copy metadata
                new_regions.push(merged_mesh.regs[keep_idx]);
                new_areas.push(merged_mesh.areas[keep_idx]);
            }

            merged_mesh.polys = new_polys;
            merged_mesh.regs = new_regions;
            merged_mesh.areas = new_areas;
            merged_mesh.poly_count = keep_indices.len();
        }

        Ok(())
    }

    /// Merges coplanar adjacent polygons (simplified implementation)
    fn merge_coplanar_polygons(
        &self,
        _merged_mesh: &mut PolyMesh,
        _stats: &mut MergeStats,
    ) -> Result<()> {
        // TODO: Implement coplanar polygon merging
        // This is a complex operation that requires:
        // 1. Building polygon adjacency information
        // 2. Computing polygon normals and checking coplanarity
        // 3. Merging adjacent coplanar polygons while maintaining topology
        // 4. Ensuring the resulting polygons don't exceed max_verts_per_poly

        // For now, this is a placeholder that does nothing
        Ok(())
    }
}

/// Utility functions for mesh operations
impl MeshMerger {
    /// Translates a mesh by the given offset
    pub fn translate_mesh(mesh: &mut PolyMesh, offset: Vec3) -> Result<()> {
        for i in 0..mesh.vert_count {
            let base_idx = i * 3;
            mesh.vertices[base_idx] = (mesh.vertices[base_idx] as f32 + offset[0]) as u16;
            mesh.vertices[base_idx + 1] = (mesh.vertices[base_idx + 1] as f32 + offset[1]) as u16;
            mesh.vertices[base_idx + 2] = (mesh.vertices[base_idx + 2] as f32 + offset[2]) as u16;
        }

        // Update bounding box
        mesh.bmin += offset;
        mesh.bmax += offset;

        Ok(())
    }

    /// Scales a mesh by the given factors
    pub fn scale_mesh(mesh: &mut PolyMesh, scale: Vec3) -> Result<()> {
        if scale.x <= 0.0 || scale.y <= 0.0 || scale.z <= 0.0 {
            return Err(Error::NavMeshGeneration(
                "Scale factors must be positive".to_string(),
            ));
        }

        for i in 0..mesh.vert_count {
            let base_idx = i * 3;
            mesh.vertices[base_idx] = (mesh.vertices[base_idx] as f32 * scale[0]) as u16;
            mesh.vertices[base_idx + 1] = (mesh.vertices[base_idx + 1] as f32 * scale[1]) as u16;
            mesh.vertices[base_idx + 2] = (mesh.vertices[base_idx + 2] as f32 * scale[2]) as u16;
        }

        // Update bounding box and cell sizes
        mesh.bmin[0] *= scale[0];
        mesh.bmin[1] *= scale[1];
        mesh.bmin[2] *= scale[2];
        mesh.bmax[0] *= scale[0];
        mesh.bmax[1] *= scale[1];
        mesh.bmax[2] *= scale[2];

        mesh.cs *= scale[0].min(scale[2]); // Use smaller horizontal scale for cell size
        mesh.ch *= scale[1];

        Ok(())
    }

    /// Creates a copy of a mesh with transformed vertices
    /// Applies scale first, then translation (standard transform order)
    pub fn transform_mesh(
        mesh: &PolyMesh,
        translation: Vec3,
        scale: Vec3,
    ) -> Result<PolyMesh> {
        let mut transformed = mesh.clone();
        Self::scale_mesh(&mut transformed, scale)?;
        Self::translate_mesh(&mut transformed, translation)?;
        Ok(transformed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn create_test_mesh(
        vertices: Vec<(i32, i32, i32)>,
        polygons: Vec<Vec<u16>>,
        regions: Vec<u16>,
        areas: Vec<u8>,
    ) -> PolyMesh {
        let mut mesh = PolyMesh::new(6, 0);

        // Set basic properties
        mesh.bmin = Vec3::new(0.0, 0.0, 0.0);
        mesh.bmax = Vec3::new(10.0, 10.0, 10.0);
        mesh.cs = 1.0;
        mesh.ch = 1.0;

        // Add vertices
        mesh.nverts = vertices.len();
        mesh.vert_count = vertices.len();
        mesh.verts = Vec::with_capacity(vertices.len() * 3);
        mesh.vertices = Vec::with_capacity(vertices.len() * 3);
        for (x, y, z) in vertices {
            mesh.verts.push(x as u16);
            mesh.verts.push(y as u16);
            mesh.verts.push(z as u16);
            mesh.vertices.push(x as u16);
            mesh.vertices.push(y as u16);
            mesh.vertices.push(z as u16);
        }

        // Add polygons
        mesh.npolys = polygons.len();
        mesh.poly_count = polygons.len();
        mesh.maxpolys = polygons.len();
        mesh.polys = vec![MESH_NULL_IDX; polygons.len() * mesh.nvp * 2];
        mesh.regs = regions;
        mesh.areas = areas;
        mesh.flags = vec![0; polygons.len()];

        for (i, poly) in polygons.iter().enumerate() {
            let base = i * mesh.nvp * 2;
            for (j, &v) in poly.iter().enumerate() {
                if j < mesh.nvp {
                    mesh.polys[base + j] = v;
                }
            }
            // Fill unused vertices with MESH_NULL_IDX
            for j in poly.len()..mesh.nvp {
                mesh.polys[base + j] = MESH_NULL_IDX;
            }
            // Fill neighbor data with MESH_NULL_IDX
            for j in 0..mesh.nvp {
                mesh.polys[base + mesh.nvp + j] = MESH_NULL_IDX;
            }
        }

        mesh
    }

    #[test]
    fn test_mesh_merger_basic() {
        let merger = MeshMerger::with_defaults();

        // Create two simple meshes
        let mesh1 = create_test_mesh(
            vec![(0, 0, 0), (2, 0, 0), (1, 0, 2)],
            vec![vec![0, 1, 2]],
            vec![1],
            vec![1],
        );

        let mesh2 = create_test_mesh(
            vec![(3, 0, 0), (5, 0, 0), (4, 0, 2)],
            vec![vec![0, 1, 2]],
            vec![2],
            vec![1],
        );

        let result = merger.merge_meshes(&[&mesh1, &mesh2]).unwrap();

        // Should have combined vertices and polygons
        assert_eq!(result.mesh.vert_count, 6);
        assert_eq!(result.mesh.poly_count, 2);
        assert_eq!(result.stats.input_mesh_count, 2);
        assert_eq!(result.stats.input_vertex_count, 6);
        assert_eq!(result.stats.input_polygon_count, 2);
    }

    #[test]
    fn test_vertex_welding() {
        let config = MeshMergeConfig {
            weld_threshold: 1.5, // Large threshold to force welding
            ..Default::default()
        };
        let merger = MeshMerger::new(config);

        // Create two meshes with nearby vertices that should be welded
        let mesh1 = create_test_mesh(
            vec![(0, 0, 0), (2, 0, 0), (1, 0, 2)],
            vec![vec![0, 1, 2]],
            vec![1],
            vec![1],
        );

        let mesh2 = create_test_mesh(
            vec![(1, 0, 0), (3, 0, 0), (2, 0, 2)], // First vertex close to mesh1's second vertex
            vec![vec![0, 1, 2]],
            vec![2],
            vec![1],
        );

        let result = merger.merge_meshes(&[&mesh1, &mesh2]).unwrap();

        // Should have fewer vertices due to welding
        assert!(result.mesh.vert_count < 6);
        assert!(result.stats.vertices_welded > 0);
    }

    #[test]
    fn test_duplicate_removal() {
        let merger = MeshMerger::with_defaults();

        // Create two meshes with identical polygons
        let mesh1 = create_test_mesh(
            vec![(0, 0, 0), (2, 0, 0), (1, 0, 2)],
            vec![vec![0, 1, 2]],
            vec![1],
            vec![1],
        );

        let mesh2 = create_test_mesh(
            vec![(0, 0, 0), (2, 0, 0), (1, 0, 2)], // Identical vertices
            vec![vec![0, 1, 2]],                   // Identical polygon
            vec![1],                               // Same region
            vec![1],                               // Same area
        );

        let result = merger.merge_meshes(&[&mesh1, &mesh2]).unwrap();

        // Should have removed one duplicate polygon
        assert_eq!(result.mesh.poly_count, 1);
        assert!(result.stats.duplicates_removed > 0);
    }

    #[test]
    fn test_mesh_compatibility_validation() {
        let merger = MeshMerger::with_defaults();

        let mesh1 = create_test_mesh(
            vec![(0, 0, 0), (2, 0, 0), (1, 0, 2)],
            vec![vec![0, 1, 2]],
            vec![1],
            vec![1],
        );

        let mut mesh2 = create_test_mesh(
            vec![(3, 0, 0), (5, 0, 0), (4, 0, 2)],
            vec![vec![0, 1, 2]],
            vec![2],
            vec![1],
        );

        // Make meshes incompatible
        mesh2.cs = 2.0; // Different cell size

        let result = merger.merge_meshes(&[&mesh1, &mesh2]);
        assert!(result.is_err());
    }

    #[test]
    fn test_mesh_transformation() {
        let mesh = create_test_mesh(
            vec![(0, 0, 0), (2, 0, 0), (1, 0, 2)],
            vec![vec![0, 1, 2]],
            vec![1],
            vec![1],
        );

        let translation = Vec3::new(5.0, 3.0, 1.0);
        let scale = Vec3::new(2.0, 1.5, 2.0);

        let transformed = MeshMerger::transform_mesh(&mesh, translation, scale).unwrap();

        // Check first vertex transformation
        assert_eq!(transformed.vertices[0], 5); // 0 * 2.0 + 5.0
        assert_eq!(transformed.vertices[1], 3); // 0 * 1.5 + 3.0
        assert_eq!(transformed.vertices[2], 1); // 0 * 2.0 + 1.0

        // Check second vertex transformation
        assert_eq!(transformed.vertices[3], 9); // 2 * 2.0 + 5.0
        assert_eq!(transformed.vertices[4], 3); // 0 * 1.5 + 3.0
        assert_eq!(transformed.vertices[5], 1); // 0 * 2.0 + 1.0
    }

    #[test]
    fn test_empty_mesh_list() {
        let merger = MeshMerger::with_defaults();
        let result = merger.merge_meshes(&[]);
        assert!(result.is_err());
    }
}

/// Creates a deep copy of a polygon mesh
/// This is a utility function that matches the C++ rcCopyPolyMesh
pub fn copy_poly_mesh(mesh: &PolyMesh) -> PolyMesh {
    mesh.copy()
}
