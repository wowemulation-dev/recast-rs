//! Test mesh creation helpers that match C++ test patterns
//!
//! This module provides navigation mesh creation functions that exactly match
//! the C++ test patterns to ensure proper test mesh initialization.

use crate::nav_mesh_builder::NavMeshBuilder;
use crate::{NavMesh, NavMeshCreateParams, NavMeshParams, PolyFlags};
use recast_common::Result;

/// Creates a minimal navigation mesh matching C++ createMinimalNavMesh()
///
/// This creates a simple 2x2 quad polygon at the origin, exactly matching
/// the C++ test pattern for basic functionality testing.
pub fn create_minimal_test_navmesh() -> Result<NavMesh> {
    // Create NavMeshParams matching C++ pattern
    let nav_mesh_params = NavMeshParams {
        origin: [0.0, 0.0, 0.0],
        tile_width: 1.0, // Small tile for minimal mesh
        tile_height: 1.0,
        max_tiles: 1,
        max_polys_per_tile: 1,
    };

    let mut nav_mesh = NavMesh::new(nav_mesh_params.clone())?;

    // Create vertex data (in voxel coordinates, like C++)
    // 4 vertices forming a 2x2 square at origin
    let verts = vec![
        0.0, 0.0, 0.0, // Vertex 0: (0,0,0)
        2.0, 0.0, 0.0, // Vertex 1: (2,0,0)
        2.0, 0.0, 2.0, // Vertex 2: (2,0,2)
        0.0, 0.0, 2.0, // Vertex 3: (0,0,2)
    ];

    // Polygon data: 4 vertices + 2 padding (nvp=6 format)
    let polys = vec![
        0, 1, 2, 3, // Vertex indices (counter-clockwise)
        0, 0, // Padding for nvp=6
    ];

    let poly_flags = vec![PolyFlags::WALK];
    let poly_areas = vec![0u8];

    // Critical: Convert voxel coordinates to world coordinates
    let cs = 0.3f32; // Cell size
    let ch = 0.2f32; // Cell height

    let tile_params = NavMeshCreateParams {
        nav_mesh_params: nav_mesh_params,
        verts,
        vert_count: 4,
        polys,
        poly_flags,
        poly_areas,
        poly_count: 1,
        nvp: 6, // Max vertices per polygon

        // Detail mesh (empty for minimal test)
        detail_meshes: vec![],
        detail_verts: vec![],
        detail_vert_count: 0,
        detail_tris: vec![],
        detail_tri_count: 0,

        // Off-mesh connections (none)
        off_mesh_con_verts: vec![],
        off_mesh_con_rad: vec![],
        off_mesh_con_flags: vec![],
        off_mesh_con_areas: vec![],
        off_mesh_con_dir: vec![],
        off_mesh_con_user_id: vec![],
        off_mesh_con_count: 0,

        // Bounding box in WORLD coordinates (voxel * cs)
        bmin: [0.0, 0.0, 0.0],
        bmax: [2.0 * cs, 1.0, 2.0 * cs], // [0.6, 1.0, 0.6]

        // Agent parameters matching C++ tests
        walkable_height: 2.0,
        walkable_radius: 0.6,
        walkable_climb: 0.9,
        cs,
        ch,
        build_bv_tree: true,
    };

    let tile = NavMeshBuilder::build_tile(&tile_params)?;
    nav_mesh.add_mesh_tile(tile)?;

    Ok(nav_mesh)
}

/// Creates a complex navigation mesh with multiple connected polygons
///
/// This creates a 3x3 grid of connected polygons for multi-polygon testing,
/// matching the C++ createComplexNavMesh() pattern.
pub fn create_complex_test_navmesh() -> Result<NavMesh> {
    let nav_mesh_params = NavMeshParams {
        origin: [0.0, 0.0, 0.0],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 1,
        max_polys_per_tile: 9,
    };

    let mut nav_mesh = NavMesh::new(nav_mesh_params.clone())?;

    // Create 4x4 vertex grid for 3x3 polygons
    let mut verts = Vec::new();
    for z in 0..4 {
        for x in 0..4 {
            verts.push(x as f32); // X coordinate (voxel units)
            verts.push(0.0); // Y coordinate
            verts.push(z as f32); // Z coordinate (voxel units)
        }
    }

    // Create 9 polygons (3x3 grid)
    let mut polys = Vec::new();
    let mut poly_flags = Vec::new();
    let mut poly_areas = Vec::new();

    for z in 0..3 {
        for x in 0..3 {
            let base_idx = (z * 4 + x) as u16;

            // Polygon vertices (counter-clockwise)
            polys.extend_from_slice(&[
                base_idx,     // bottom-left
                base_idx + 1, // bottom-right
                base_idx + 5, // top-right
                base_idx + 4, // top-left
                0,
                0, // Padding
            ]);

            poly_flags.push(PolyFlags::WALK);
            poly_areas.push(0u8);
        }
    }

    let cs = 0.3f32;
    let ch = 0.2f32;

    let tile_params = NavMeshCreateParams {
        nav_mesh_params: nav_mesh_params,
        verts,
        vert_count: 16,
        polys,
        poly_flags,
        poly_areas,
        poly_count: 9,
        nvp: 6,

        // Detail mesh (empty)
        detail_meshes: vec![],
        detail_verts: vec![],
        detail_vert_count: 0,
        detail_tris: vec![],
        detail_tri_count: 0,

        // Off-mesh connections (none)
        off_mesh_con_verts: vec![],
        off_mesh_con_rad: vec![],
        off_mesh_con_flags: vec![],
        off_mesh_con_areas: vec![],
        off_mesh_con_dir: vec![],
        off_mesh_con_user_id: vec![],
        off_mesh_con_count: 0,

        // Bounding box in world coordinates
        bmin: [0.0, 0.0, 0.0],
        bmax: [3.0 * cs, 1.0, 3.0 * cs], // [0.9, 1.0, 0.9]

        walkable_height: 2.0,
        walkable_radius: 0.6,
        walkable_climb: 0.9,
        cs,
        ch,
        build_bv_tree: true,
    };

    let tile = NavMeshBuilder::build_tile(&tile_params)?;
    nav_mesh.add_mesh_tile(tile)?;

    Ok(nav_mesh)
}

/// Creates a larger rectangular mesh for stress testing
pub fn create_large_test_navmesh() -> Result<NavMesh> {
    let nav_mesh_params = NavMeshParams {
        origin: [0.0, 0.0, 0.0],
        tile_width: 30.0,
        tile_height: 30.0,
        max_tiles: 1,
        max_polys_per_tile: 1,
    };

    let mut nav_mesh = NavMesh::new(nav_mesh_params.clone())?;

    // Large rectangular polygon
    let verts = vec![
        0.0, 0.0, 0.0, // (0,0,0)
        100.0, 0.0, 0.0, // (100,0,0)
        100.0, 0.0, 100.0, // (100,0,100)
        0.0, 0.0, 100.0, // (0,0,100)
    ];

    let polys = vec![0, 1, 2, 3, 0, 0];
    let poly_flags = vec![PolyFlags::WALK];
    let poly_areas = vec![0u8];

    let cs = 0.3f32;
    let ch = 0.2f32;

    let tile_params = NavMeshCreateParams {
        nav_mesh_params: nav_mesh_params,
        verts,
        vert_count: 4,
        polys,
        poly_flags,
        poly_areas,
        poly_count: 1,
        nvp: 6,

        detail_meshes: vec![],
        detail_verts: vec![],
        detail_vert_count: 0,
        detail_tris: vec![],
        detail_tri_count: 0,

        off_mesh_con_verts: vec![],
        off_mesh_con_rad: vec![],
        off_mesh_con_flags: vec![],
        off_mesh_con_areas: vec![],
        off_mesh_con_dir: vec![],
        off_mesh_con_user_id: vec![],
        off_mesh_con_count: 0,

        // Large bounding box
        bmin: [0.0, 0.0, 0.0],
        bmax: [100.0 * cs, 1.0, 100.0 * cs], // [30.0, 1.0, 30.0]

        walkable_height: 2.0,
        walkable_radius: 0.6,
        walkable_climb: 0.9,
        cs,
        ch,
        build_bv_tree: true,
    };

    let tile = NavMeshBuilder::build_tile(&tile_params)?;
    nav_mesh.add_mesh_tile(tile)?;

    Ok(nav_mesh)
}

/// Gets a valid test position within the minimal mesh bounds
pub fn get_test_position_minimal() -> [f32; 3] {
    [0.3, 0.0, 0.3] // Center of minimal mesh (world coordinates)
}

/// Gets a valid test position within the complex mesh bounds
pub fn get_test_position_complex() -> [f32; 3] {
    [0.45, 0.0, 0.45] // Center of complex mesh (world coordinates)
}

/// Gets a valid test position within the large mesh bounds
pub fn get_test_position_large() -> [f32; 3] {
    [15.0, 0.0, 15.0] // Center of large mesh (world coordinates)
}

/// Gets appropriate search extents for test queries
pub fn get_test_extents() -> [f32; 3] {
    [1.0, 1.0, 1.0] // Reasonable search extents
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{NavMeshQuery, QueryFilter};

    #[test]
    fn test_minimal_mesh_creation() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let extents = get_test_extents();

        // Should find a polygon at the test position
        let result = query.find_nearest_poly(&center, &extents, &filter);
        assert!(result.is_ok(), "Should find polygon in minimal mesh");

        let (poly_ref, _pos) = result?;
        assert!(poly_ref.is_valid(), "Found polygon should be valid");

        Ok(())
    }

    #[test]
    fn test_complex_mesh_creation() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_complex();
        let extents = get_test_extents();

        // Should find a polygon at the test position
        let result = query.find_nearest_poly(&center, &extents, &filter);
        assert!(result.is_ok(), "Should find polygon in complex mesh");

        let (poly_ref, _pos) = result?;
        assert!(poly_ref.is_valid(), "Found polygon should be valid");

        Ok(())
    }
}
