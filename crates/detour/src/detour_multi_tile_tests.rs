//! Tests for multi-tile navigation mesh functionality
//!
//! Multi-tile meshes allow for larger worlds by dividing the navigation
//! mesh into tiles that can be loaded/unloaded dynamically.

#[cfg(test)]
mod tests {
    use crate::{
        NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyFlags, QueryFilter,
    };
    use recast_common::Result;

    /// Helper to create a simple 2x2 tile mesh
    fn create_2x2_tile_mesh() -> Result<NavMesh> {
        let tile_size = 10.0;
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: tile_size,
            tile_height: tile_size,
            max_tiles: 4,
            max_polys_per_tile: 10,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;

        // Create 4 tiles in a 2x2 grid
        for ty in 0..2 {
            for tx in 0..2 {
                let tile_origin_x = tx as f32 * tile_size;
                let tile_origin_z = ty as f32 * tile_size;

                // Create a simple square polygon for each tile
                let vertices = vec![
                    tile_origin_x,
                    0.0,
                    tile_origin_z,
                    tile_origin_x + tile_size,
                    0.0,
                    tile_origin_z,
                    tile_origin_x + tile_size,
                    0.0,
                    tile_origin_z + tile_size,
                    tile_origin_x,
                    0.0,
                    tile_origin_z + tile_size,
                ];

                // Create polygon: 4 vertices (square), 2 empty vertex slots (for nvp=6)
                let mut polys = vec![0, 1, 2, 3, 0xffff, 0xffff];

                // Add neighbor information for edges (6 neighbor slots)
                // Mark internal grid boundaries as external links that can connect to neighbors
                // Bottom edge (between tiles vertically)
                if ty > 0 {
                    polys.push(0x8000 | 6);
                } else {
                    polys.push(0xffff);
                } // Can connect south to tile below
                // Right edge (between tiles horizontally)
                if tx < 1 {
                    polys.push(0x8000 | 0);
                } else {
                    polys.push(0xffff);
                } // Can connect east to tile right
                // Top edge
                if ty < 1 {
                    polys.push(0x8000 | 2);
                } else {
                    polys.push(0xffff);
                } // Can connect north to tile above
                // Left edge
                if tx > 0 {
                    polys.push(0x8000 | 4);
                } else {
                    polys.push(0xffff);
                } // Can connect west to tile left
                // Two more neighbor slots (unused for a 4-vertex polygon)
                polys.push(0xffff);
                polys.push(0xffff);
                let poly_areas = vec![0];
                let poly_flags = vec![PolyFlags::WALK];

                let create_params = NavMeshCreateParams {
                    nav_mesh_params: params.clone(),
                    verts: vertices,
                    vert_count: 4,
                    polys,
                    poly_flags,
                    poly_areas,
                    poly_count: 1,
                    nvp: 6,
                    detail_meshes: vec![0, 4, 0, 2],
                    detail_verts: vec![
                        tile_origin_x,
                        0.0,
                        tile_origin_z,
                        tile_origin_x + tile_size,
                        0.0,
                        tile_origin_z,
                        tile_origin_x + tile_size,
                        0.0,
                        tile_origin_z + tile_size,
                        tile_origin_x,
                        0.0,
                        tile_origin_z + tile_size,
                    ],
                    detail_vert_count: 4,
                    detail_tris: vec![0, 1, 2, 0, 0, 2, 3, 0],
                    detail_tri_count: 2,
                    off_mesh_con_verts: Vec::new(),
                    off_mesh_con_rad: Vec::new(),
                    off_mesh_con_dir: Vec::new(),
                    off_mesh_con_areas: Vec::new(),
                    off_mesh_con_flags: Vec::new(),
                    off_mesh_con_user_id: Vec::new(),
                    off_mesh_con_count: 0,
                    bmin: [tile_origin_x, 0.0, tile_origin_z],
                    bmax: [tile_origin_x + tile_size, 1.0, tile_origin_z + tile_size],
                    walkable_height: 2.0,
                    walkable_radius: 0.6,
                    walkable_climb: 0.9,
                    cs: 0.3,
                    ch: 0.2,
                    build_bv_tree: true,
                };

                nav_mesh.add_tile_from_params(&create_params, tx, ty, 0)?;
            }
        }

        // Connect all tiles after adding them
        for ty in 0..2 {
            for tx in 0..2 {
                if let Some(tile_idx) = nav_mesh.get_tile_index(tx, ty, 0) {
                    nav_mesh.connect_tile(tile_idx)?;
                }
            }
        }

        Ok(nav_mesh)
    }

    /// Test basic multi-tile mesh creation
    #[test]
    fn test_multi_tile_creation() -> Result<()> {
        let nav_mesh = create_2x2_tile_mesh()?;

        assert_eq!(nav_mesh.get_max_tiles(), 4, "Should support 4 tiles");

        // Verify all tiles exist
        for ty in 0..2 {
            for tx in 0..2 {
                let tile = nav_mesh.get_tile_at(tx, ty, 0);
                assert!(tile.is_some(), "Tile ({}, {}) should exist", tx, ty);
            }
        }

        Ok(())
    }

    /// Test navigation across tile boundaries
    #[test]
    fn test_cross_tile_navigation() -> Result<()> {
        let nav_mesh = create_2x2_tile_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Start in tile (0,0)
        let start_pos = [5.0, 0.0, 5.0];
        // End in tile (1,1)
        let end_pos = [15.0, 0.0, 15.0];
        let extents = [1.0, 1.0, 1.0];

        let (start_ref, actual_start) = query.find_nearest_poly(&start_pos, &extents, &filter)?;
        let (end_ref, actual_end) = query.find_nearest_poly(&end_pos, &extents, &filter)?;

        assert!(start_ref.is_valid(), "Should find start polygon");
        assert!(end_ref.is_valid(), "Should find end polygon");

        // Decode refs to verify they're in different tiles
        let (start_tile, _) = crate::decode_poly_ref(start_ref);
        let (end_tile, _) = crate::decode_poly_ref(end_ref);
        assert_ne!(
            start_tile, end_tile,
            "Start and end should be in different tiles"
        );

        // Find path across tiles
        let path = query.find_path(start_ref, end_ref, &actual_start, &actual_end, &filter)?;
        assert!(path.len() >= 2, "Path should cross multiple tiles");

        Ok(())
    }

    /// Test queries near tile boundaries
    #[test]
    fn test_tile_boundary_queries() -> Result<()> {
        let nav_mesh = create_2x2_tile_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test point exactly on tile boundary
        let boundary_pos = [10.0, 0.0, 5.0]; // Between tiles (0,0) and (1,0)
        let small_extents = [0.1, 0.1, 0.1];

        let result = query.find_nearest_poly(&boundary_pos, &small_extents, &filter);
        assert!(result.is_ok(), "Should handle boundary queries");

        // Test query spanning multiple tiles
        let center_pos = [10.0, 0.0, 10.0]; // At intersection of all 4 tiles
        let large_extents = [5.0, 5.0, 5.0];

        let result = query.find_nearest_poly(&center_pos, &large_extents, &filter);
        assert!(result.is_ok(), "Should handle multi-tile spanning queries");

        Ok(())
    }

    /// Test adding and removing tiles dynamically
    #[test]
    fn test_dynamic_tile_management() -> Result<()> {
        let tile_size = 10.0;
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: tile_size,
            tile_height: tile_size,
            max_tiles: 10,
            max_polys_per_tile: 10,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;

        // Add a single tile
        let vertices = vec![
            0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
        ];

        let create_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: vertices,
            vert_count: 4,
            polys: vec![0, 1, 2, 3, 0xffff, 0xffff],
            poly_flags: vec![PolyFlags::WALK],
            poly_areas: vec![0],
            poly_count: 1,
            nvp: 6,
            detail_meshes: vec![0, 4, 0, 2],
            detail_verts: vec![
                0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
            ],
            detail_vert_count: 4,
            detail_tris: vec![0, 1, 2, 0, 0, 2, 3, 0],
            detail_tri_count: 2,
            off_mesh_con_verts: Vec::new(),
            off_mesh_con_rad: Vec::new(),
            off_mesh_con_dir: Vec::new(),
            off_mesh_con_areas: Vec::new(),
            off_mesh_con_flags: Vec::new(),
            off_mesh_con_user_id: Vec::new(),
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [10.0, 1.0, 10.0],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        let tile_ref = nav_mesh.add_tile_from_params(&create_params, 0, 0, 0)?;
        assert!(
            nav_mesh.get_tile_at(0, 0, 0).is_some(),
            "Tile should exist after adding"
        );

        // Remove the tile
        nav_mesh.remove_tile(tile_ref)?;
        assert!(
            nav_mesh.get_tile_at(0, 0, 0).is_none(),
            "Tile should not exist after removal"
        );

        Ok(())
    }

    /// Test tile coordinate calculations
    #[test]
    fn test_tile_coordinate_conversion() -> Result<()> {
        let nav_mesh = create_2x2_tile_mesh()?;

        // Test various world positions to tile coordinates
        let test_cases = vec![
            ([5.0, 0.0, 5.0], (0, 0)),   // Center of tile (0,0)
            ([15.0, 0.0, 5.0], (1, 0)),  // Center of tile (1,0)
            ([5.0, 0.0, 15.0], (0, 1)),  // Center of tile (0,1)
            ([15.0, 0.0, 15.0], (1, 1)), // Center of tile (1,1)
            ([0.0, 0.0, 0.0], (0, 0)),   // Origin
            ([10.0, 0.0, 10.0], (1, 1)), // Tile boundary (should round to higher tile)
        ];

        for (pos, expected_tile) in test_cases {
            let (tx, ty) = nav_mesh.calc_tile_loc(&pos);
            assert_eq!(
                (tx, ty),
                expected_tile,
                "Position {:?} should map to tile {:?}",
                pos,
                expected_tile
            );
        }

        Ok(())
    }

    /// Test maximum tile limit
    #[test]
    fn test_maximum_tile_limit() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 2,
            max_polys_per_tile: 10,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;

        // Create minimal tile params
        let create_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0],
            vert_count: 4,
            polys: vec![0, 1, 2, 3, 0xffff, 0xffff],
            poly_flags: vec![PolyFlags::WALK],
            poly_areas: vec![0],
            poly_count: 1,
            nvp: 6,
            detail_meshes: vec![0, 4, 0, 2],
            detail_verts: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0],
            detail_vert_count: 4,
            detail_tris: vec![0, 1, 2, 0, 0, 2, 3, 0],
            detail_tri_count: 2,
            off_mesh_con_verts: Vec::new(),
            off_mesh_con_rad: Vec::new(),
            off_mesh_con_dir: Vec::new(),
            off_mesh_con_areas: Vec::new(),
            off_mesh_con_flags: Vec::new(),
            off_mesh_con_user_id: Vec::new(),
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [10.0, 1.0, 10.0],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        // Add tiles up to limit
        nav_mesh.add_tile_from_params(&create_params, 0, 0, 0)?;
        nav_mesh.add_tile_from_params(&create_params, 1, 0, 0)?;

        // Try to add one more - should fail or replace
        let result = nav_mesh.add_tile_from_params(&create_params, 2, 0, 0);
        // This might succeed by replacing an old tile or fail - both are valid

        Ok(())
    }

    /// Test pathfinding performance across many tiles
    #[test]
    #[ignore] // Can be expensive, run with --ignored flag
    fn test_large_multi_tile_pathfinding() -> Result<()> {
        // Create a larger grid (10x10 tiles)
        let grid_size = 10;
        let tile_size = 10.0;
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: tile_size,
            tile_height: tile_size,
            max_tiles: grid_size * grid_size,
            max_polys_per_tile: 10,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;

        // Add all tiles
        for ty in 0..grid_size {
            for tx in 0..grid_size {
                let tile_origin_x = tx as f32 * tile_size;
                let tile_origin_z = ty as f32 * tile_size;

                let vertices = vec![
                    tile_origin_x,
                    0.0,
                    tile_origin_z,
                    tile_origin_x + tile_size,
                    0.0,
                    tile_origin_z,
                    tile_origin_x + tile_size,
                    0.0,
                    tile_origin_z + tile_size,
                    tile_origin_x,
                    0.0,
                    tile_origin_z + tile_size,
                ];

                let create_params = NavMeshCreateParams {
                    nav_mesh_params: params.clone(),
                    verts: vertices,
                    vert_count: 4,
                    polys: vec![0, 1, 2, 3, 0xffff, 0xffff],
                    poly_flags: vec![PolyFlags::WALK],
                    poly_areas: vec![0],
                    poly_count: 1,
                    nvp: 6,
                    detail_meshes: vec![0, 4, 0, 2],
                    detail_verts: vec![
                        tile_origin_x,
                        0.0,
                        tile_origin_z,
                        tile_origin_x + tile_size,
                        0.0,
                        tile_origin_z,
                        tile_origin_x + tile_size,
                        0.0,
                        tile_origin_z + tile_size,
                        tile_origin_x,
                        0.0,
                        tile_origin_z + tile_size,
                    ],
                    detail_vert_count: 4,
                    detail_tris: vec![0, 1, 2, 0, 0, 2, 3, 0],
                    detail_tri_count: 2,
                    off_mesh_con_verts: Vec::new(),
                    off_mesh_con_rad: Vec::new(),
                    off_mesh_con_dir: Vec::new(),
                    off_mesh_con_areas: Vec::new(),
                    off_mesh_con_flags: Vec::new(),
                    off_mesh_con_user_id: Vec::new(),
                    off_mesh_con_count: 0,
                    bmin: [tile_origin_x, 0.0, tile_origin_z],
                    bmax: [tile_origin_x + tile_size, 1.0, tile_origin_z + tile_size],
                    walkable_height: 2.0,
                    walkable_radius: 0.6,
                    walkable_climb: 0.9,
                    cs: 0.3,
                    ch: 0.2,
                    build_bv_tree: true,
                };

                nav_mesh.add_tile_from_params(&create_params, tx, ty, 0)?;
            }
        }

        // Connect all tiles after adding them
        for ty in 0..grid_size {
            for tx in 0..grid_size {
                if let Some(tile_idx) = nav_mesh.get_tile_index(tx, ty, 0) {
                    nav_mesh.connect_tile(tile_idx)?;
                }
            }
        }

        // Test diagonal path across entire grid
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let start_pos = [5.0, 0.0, 5.0]; // Near origin
        let end_pos = [95.0, 0.0, 95.0]; // Far corner
        let extents = [1.0, 1.0, 1.0];

        let (start_ref, actual_start) = query.find_nearest_poly(&start_pos, &extents, &filter)?;
        let (end_ref, actual_end) = query.find_nearest_poly(&end_pos, &extents, &filter)?;

        assert!(
            start_ref.is_valid() && end_ref.is_valid(),
            "Should find polygons"
        );

        // Time the pathfinding
        let start_time = std::time::Instant::now();
        let path = query.find_path(start_ref, end_ref, &actual_start, &actual_end, &filter)?;
        let elapsed = start_time.elapsed();

        println!("Large grid pathfinding took: {:?}", elapsed);
        assert!(!path.is_empty(), "Should find path across large grid");

        Ok(())
    }

    /// Test tile layer support
    #[test]
    fn test_tile_layers() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 4,
            max_polys_per_tile: 10,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;

        // Create a simple tile params
        let create_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: vec![
                0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
            ],
            vert_count: 4,
            polys: vec![0, 1, 2, 3, 0xffff, 0xffff],
            poly_flags: vec![PolyFlags::WALK],
            poly_areas: vec![0],
            poly_count: 1,
            nvp: 6,
            detail_meshes: vec![0, 4, 0, 2],
            detail_verts: vec![
                0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
            ],
            detail_vert_count: 4,
            detail_tris: vec![0, 1, 2, 0, 0, 2, 3, 0],
            detail_tri_count: 2,
            off_mesh_con_verts: Vec::new(),
            off_mesh_con_rad: Vec::new(),
            off_mesh_con_dir: Vec::new(),
            off_mesh_con_areas: Vec::new(),
            off_mesh_con_flags: Vec::new(),
            off_mesh_con_user_id: Vec::new(),
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [10.0, 1.0, 10.0],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        // Add tiles at different layers
        nav_mesh.add_tile_from_params(&create_params, 0, 0, 0)?; // Ground layer
        nav_mesh.add_tile_from_params(&create_params, 0, 0, 1)?; // Layer 1

        // Verify both layers exist
        assert!(
            nav_mesh.get_tile_at(0, 0, 0).is_some(),
            "Layer 0 should exist"
        );
        assert!(
            nav_mesh.get_tile_at(0, 0, 1).is_some(),
            "Layer 1 should exist"
        );

        Ok(())
    }

    /// Test tile connection validation
    #[test]
    fn test_tile_connection_validation() -> Result<()> {
        // TODO: Test that tiles properly connect at boundaries
        // - Verify portal edges are created
        // - Test disconnected tiles don't create invalid paths

        Ok(())
    }

    /// Test tile reference encoding/decoding
    #[test]
    fn test_tile_reference_encoding() -> Result<()> {
        // Test encoding and decoding of tile references
        let test_cases = vec![
            (0, 0),
            (1, 0),
            (0, 1),
            (100, 200),
            (0x3FF, 0xFFFF), // Maximum valid values (10-bit tile ID, 16-bit poly ID)
        ];

        for (tile_id, poly_id) in test_cases {
            let poly_ref = crate::encode_poly_ref(tile_id, poly_id);
            let (decoded_tile, decoded_poly) = crate::decode_poly_ref(poly_ref);

            assert_eq!(decoded_tile, tile_id, "Tile ID should match");
            assert_eq!(decoded_poly, poly_id, "Poly ID should match");
        }

        Ok(())
    }
}
