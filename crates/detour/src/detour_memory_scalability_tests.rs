//! Memory usage and scalability tests for Detour navigation mesh
//!
//! These tests verify that the navigation mesh implementation:
//! - Scales well with increasing data sizes
//! - Manages memory efficiently
//! - Cleans up resources properly
//! - Handles large-scale scenarios

#[cfg(test)]
mod tests {
    use crate::{
        NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyFlags, QueryFilter,
    };
    use recast_common::Result;
    use std::mem;

    /// Helper to get approximate memory size of a type
    fn get_memory_size<T>() -> usize {
        mem::size_of::<T>()
    }

    /// Helper to create a large navigation mesh with specified tile count
    fn create_large_navmesh(tiles_per_side: i32) -> Result<NavMesh> {
        let tile_size = 32.0;
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: tile_size,
            tile_height: tile_size,
            max_tiles: tiles_per_side * tiles_per_side,
            max_polys_per_tile: 256,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;

        // Create tiles in a grid
        for ty in 0..tiles_per_side {
            for tx in 0..tiles_per_side {
                let tile_origin_x = tx as f32 * tile_size;
                let tile_origin_z = ty as f32 * tile_size;

                // Create a 4x4 polygon grid per tile
                let polys_per_side = 4;
                let poly_size = tile_size / polys_per_side as f32;

                let mut vertices = Vec::new();
                for y in 0..=polys_per_side {
                    for x in 0..=polys_per_side {
                        vertices.extend_from_slice(&[
                            tile_origin_x + x as f32 * poly_size,
                            0.0,
                            tile_origin_z + y as f32 * poly_size,
                        ]);
                    }
                }

                let mut polys = Vec::new();
                let mut poly_areas = Vec::new();
                let mut poly_flags = Vec::new();

                for y in 0..polys_per_side {
                    for x in 0..polys_per_side {
                        let base = y * (polys_per_side + 1) + x;
                        polys.extend_from_slice(&[
                            base as u16,
                            (base + 1) as u16,
                            (base + polys_per_side + 2) as u16,
                            (base + polys_per_side + 1) as u16,
                            0xffff,
                            0xffff,
                        ]);
                        poly_areas.push(0);
                        poly_flags.push(PolyFlags::WALK);
                    }
                }

                let poly_count = polys_per_side * polys_per_side;
                let vert_count = ((polys_per_side + 1) * (polys_per_side + 1)) as i32;

                // Simple detail mesh
                let detail_meshes: Vec<u32> = (0..poly_count)
                    .flat_map(|i| vec![i as u32 * 4, 4, i as u32 * 2, 2])
                    .collect();

                let detail_verts = vertices.clone();
                let detail_tris: Vec<u8> = (0..poly_count)
                    .flat_map(|_| vec![0, 1, 2, 0, 0, 2, 3, 0])
                    .collect();

                let create_params = NavMeshCreateParams {
                    nav_mesh_params: params.clone(),
                    verts: vertices,
                    vert_count,
                    polys,
                    poly_flags,
                    poly_areas,
                    poly_count,
                    nvp: 6,
                    detail_meshes,
                    detail_verts,
                    detail_vert_count: vert_count,
                    detail_tris,
                    detail_tri_count: poly_count * 2,
                    off_mesh_con_verts: Vec::new(),
                    off_mesh_con_rad: Vec::new(),
                    off_mesh_con_dir: Vec::new(),
                    off_mesh_con_areas: Vec::new(),
                    off_mesh_con_flags: Vec::new(),
                    off_mesh_con_user_id: Vec::new(),
                    off_mesh_con_count: 0,
                    bmin: [tile_origin_x, -1.0, tile_origin_z],
                    bmax: [tile_origin_x + tile_size, 5.0, tile_origin_z + tile_size],
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

        Ok(nav_mesh)
    }

    /// Test memory usage with increasing tile counts
    #[test]
    fn test_memory_scaling_with_tiles() -> Result<()> {
        let tile_counts = vec![1, 2, 4, 8, 16];
        let mut memory_sizes = Vec::new();

        for tiles_per_side in tile_counts {
            let nav_mesh = create_large_navmesh(tiles_per_side)?;

            // Get basic size estimate (this is just the struct size, not heap allocations)
            let base_size = get_memory_size::<NavMesh>();

            // Count tiles and polygons
            let mut tile_count = 0;
            let mut poly_count = 0;

            for ty in 0..tiles_per_side {
                for tx in 0..tiles_per_side {
                    if let Some(tile) = nav_mesh.get_tile_at(tx, ty, 0) {
                        tile_count += 1;
                        poly_count += tile.polys.len() as i32;
                    }
                }
            }

            memory_sizes.push((tiles_per_side, tile_count, poly_count));

            println!(
                "Grid {}x{}: {} tiles, {} polygons, base size: {} bytes",
                tiles_per_side, tiles_per_side, tile_count, poly_count, base_size
            );
        }

        // Verify scaling is roughly quadratic (as expected for 2D grid)
        for i in 1..memory_sizes.len() {
            let (prev_size, prev_tiles, prev_polys) = memory_sizes[i - 1];
            let (curr_size, curr_tiles, curr_polys) = memory_sizes[i];

            let size_ratio = (curr_size * curr_size) as f32 / (prev_size * prev_size) as f32;
            let tile_ratio = curr_tiles as f32 / prev_tiles as f32;

            // Allow some variance due to overhead
            assert!(
                (tile_ratio - size_ratio).abs() < 0.1,
                "Tile count should scale quadratically: expected ratio ~{}, got {}",
                size_ratio,
                tile_ratio
            );
        }

        Ok(())
    }

    /// Test query performance with large meshes
    #[test]
    fn test_query_scalability() -> Result<()> {
        let nav_mesh = create_large_navmesh(10)?; // 100 tiles
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test queries at different distances
        let test_cases = vec![
            ([5.0, 0.0, 5.0], [10.0, 0.0, 10.0]),   // Short distance
            ([5.0, 0.0, 5.0], [50.0, 0.0, 50.0]),   // Medium distance
            ([5.0, 0.0, 5.0], [150.0, 0.0, 150.0]), // Long distance
            ([5.0, 0.0, 5.0], [315.0, 0.0, 315.0]), // Very long distance
        ];

        for (start_pos, end_pos) in test_cases {
            let ext = [5.0, 5.0, 5.0];

            // Find nearest polygons
            let (start_ref, _) = query.find_nearest_poly(&start_pos, &ext, &filter)?;
            let (end_ref, _) = query.find_nearest_poly(&end_pos, &ext, &filter)?;

            if start_ref.is_valid() && end_ref.is_valid() {
                // Test pathfinding
                let path = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &filter)?;

                println!(
                    "Path from {:?} to {:?}: {} polygons",
                    start_pos,
                    end_pos,
                    path.len()
                );

                // Paths should exist for reachable destinations
                assert!(
                    !path.is_empty(),
                    "Should find path between reachable points"
                );

                // Test straight path generation
                if !path.is_empty() {
                    let straight_path = query.find_straight_path(&start_pos, &end_pos, &path)?;
                    assert!(
                        !straight_path.waypoints.is_empty(),
                        "Should generate straight path"
                    );
                }
            }
        }

        Ok(())
    }

    /// Test memory cleanup and tile removal
    #[test]
    fn test_memory_cleanup() -> Result<()> {
        let tiles_per_side = 5;
        let tile_size = 32.0;
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: tile_size,
            tile_height: tile_size,
            max_tiles: tiles_per_side * tiles_per_side,
            max_polys_per_tile: 16,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;
        let mut tile_refs = Vec::new();

        // Add tiles
        for ty in 0..tiles_per_side {
            for tx in 0..tiles_per_side {
                let vertices = vec![
                    tx as f32 * tile_size,
                    0.0,
                    ty as f32 * tile_size,
                    (tx + 1) as f32 * tile_size,
                    0.0,
                    ty as f32 * tile_size,
                    (tx + 1) as f32 * tile_size,
                    0.0,
                    (ty + 1) as f32 * tile_size,
                    tx as f32 * tile_size,
                    0.0,
                    (ty + 1) as f32 * tile_size,
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
                        tx as f32 * tile_size,
                        0.0,
                        ty as f32 * tile_size,
                        (tx + 1) as f32 * tile_size,
                        0.0,
                        ty as f32 * tile_size,
                        (tx + 1) as f32 * tile_size,
                        0.0,
                        (ty + 1) as f32 * tile_size,
                        tx as f32 * tile_size,
                        0.0,
                        (ty + 1) as f32 * tile_size,
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
                    bmin: [tx as f32 * tile_size, 0.0, ty as f32 * tile_size],
                    bmax: [
                        (tx + 1) as f32 * tile_size,
                        1.0,
                        (ty + 1) as f32 * tile_size,
                    ],
                    walkable_height: 2.0,
                    walkable_radius: 0.6,
                    walkable_climb: 0.9,
                    cs: 0.3,
                    ch: 0.2,
                    build_bv_tree: true,
                };

                let tile_ref = nav_mesh.add_tile_from_params(&create_params, tx, ty, 0)?;
                tile_refs.push((tx, ty, tile_ref));
            }
        }

        // Verify all tiles were added
        assert_eq!(tile_refs.len(), (tiles_per_side * tiles_per_side) as usize);

        // Remove tiles one by one
        for (tx, ty, tile_ref) in tile_refs {
            nav_mesh.remove_tile(tile_ref)?;

            // Verify tile is gone
            assert!(
                nav_mesh.get_tile_at(tx, ty, 0).is_none(),
                "Tile at ({}, {}) should be removed",
                tx,
                ty
            );
        }

        // Verify all tiles are removed
        let mut remaining_tiles = 0;
        for ty in 0..tiles_per_side {
            for tx in 0..tiles_per_side {
                if nav_mesh.get_tile_at(tx, ty, 0).is_some() {
                    remaining_tiles += 1;
                }
            }
        }
        assert_eq!(remaining_tiles, 0, "All tiles should be removed");

        Ok(())
    }

    /// Test maximum path length handling
    #[test]
    fn test_maximum_path_length() -> Result<()> {
        let nav_mesh = create_large_navmesh(20)?; // 400 tiles
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Try to find a very long path
        let start_pos = [5.0, 0.0, 5.0];
        let end_pos = [635.0, 0.0, 635.0]; // Far corner
        let ext = [5.0, 5.0, 5.0];

        let (start_ref, _) = query.find_nearest_poly(&start_pos, &ext, &filter)?;
        let (end_ref, _) = query.find_nearest_poly(&end_pos, &ext, &filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            // Test with limited iterations using sliced pathfinding
            query.init_sliced_find_path(start_ref, end_ref, &start_pos, &end_pos, &filter, 0)?;

            let mut total_iters = 0;
            let max_iters_per_update = 100;
            let max_total_iters = 10000;

            loop {
                let (done_iters, state) = query.update_sliced_find_path(max_iters_per_update)?;
                total_iters += done_iters;

                match state {
                    crate::SlicedPathState::InProgress => {
                        if total_iters > max_total_iters {
                            println!("Path search stopped after {} iterations", total_iters);
                            break;
                        }
                    }
                    crate::SlicedPathState::Success => {
                        println!("Path found after {} iterations", total_iters);
                        break;
                    }
                    crate::SlicedPathState::Failed => {
                        println!("Path search failed after {} iterations", total_iters);
                        break;
                    }
                    crate::SlicedPathState::PartialPath => {
                        println!("Partial path found after {} iterations", total_iters);
                        break;
                    }
                }
            }

            // Finalize the path
            let path = query.finalize_sliced_find_path(10000)?;
            println!("Final path length: {} polygons", path.len());

            // Very long paths should still be found
            assert!(!path.is_empty(), "Should find path even for long distances");
        }

        Ok(())
    }

    /// Test memory usage with many small queries
    #[test]
    fn test_many_small_queries() -> Result<()> {
        let nav_mesh = create_large_navmesh(5)?;
        let filter = QueryFilter::default();

        // Create many queries
        let query_count = 100;
        let mut queries = Vec::new();

        for _ in 0..query_count {
            queries.push(NavMeshQuery::new(&nav_mesh));
        }

        // Perform operations on all queries
        for (i, query) in queries.iter_mut().enumerate() {
            let pos = [(i % 10) as f32 * 10.0, 0.0, (i / 10) as f32 * 10.0];
            let ext = [5.0, 5.0, 5.0];

            let result = query.find_nearest_poly(&pos, &ext, &filter);
            assert!(result.is_ok(), "Query {} should succeed", i);
        }

        // Queries should be independent
        assert_eq!(queries.len(), query_count);

        Ok(())
    }

    /// Test tile replacement and memory reuse
    #[test]
    fn test_tile_replacement() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 4,
            max_polys_per_tile: 10,
        };

        let mut nav_mesh = NavMesh::new(params.clone())?;

        // Add initial tile
        let vertices = vec![
            0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
        ];

        let create_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: vertices.clone(),
            vert_count: 4,
            polys: vec![0, 1, 2, 3, 0xffff, 0xffff],
            poly_flags: vec![PolyFlags::WALK],
            poly_areas: vec![0],
            poly_count: 1,
            nvp: 6,
            detail_meshes: vec![0, 4, 0, 2],
            detail_verts: vertices.clone(),
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

        let initial_ref = nav_mesh.add_tile_from_params(&create_params, 0, 0, 0)?;

        // Replace the tile multiple times
        for i in 0..10 {
            // Modify the data slightly
            let mut modified_params = create_params.clone();
            modified_params.poly_areas[0] = (i % 4) as u8;

            // Remove old tile
            nav_mesh.remove_tile(initial_ref)?;

            // Add new tile
            let new_ref = nav_mesh.add_tile_from_params(&modified_params, 0, 0, 0)?;

            // Verify new tile exists
            assert!(
                nav_mesh.get_tile_at(0, 0, 0).is_some(),
                "Tile should exist after replacement {}",
                i
            );

            // The tile ref might change
            if i < 9 {
                nav_mesh.remove_tile(new_ref)?;
            }
        }

        Ok(())
    }

    /// Test bounds query scalability
    #[test]
    fn test_bounds_query_scalability() -> Result<()> {
        let nav_mesh = create_large_navmesh(10)?;
        let filter = QueryFilter::default();

        // Test queries of different sizes
        let query_sizes = vec![
            1.0,   // Very small
            10.0,  // Small
            50.0,  // Medium
            100.0, // Large
            200.0, // Very large
        ];

        for size in query_sizes {
            let center = [160.0, 0.0, 160.0]; // Center of 10x10 grid
            let bounds_min = [center[0] - size, -10.0, center[2] - size];
            let bounds_max = [center[0] + size, 10.0, center[2] + size];

            let polygons = nav_mesh.query_polygons(&bounds_min, &bounds_max, &filter)?;

            println!(
                "Bounds query size {}: found {} polygons",
                size * 2.0,
                polygons.len()
            );

            // Larger queries should find more polygons
            if size > 10.0 {
                assert!(!polygons.is_empty(), "Large queries should find polygons");
            }
        }

        Ok(())
    }

    /// Test stress scenario with maximum configuration
    #[test]
    #[ignore] // This test is expensive, run with --ignored
    fn test_stress_maximum_configuration() -> Result<()> {
        // Create a very large mesh (50x50 = 2500 tiles)
        let tiles_per_side = 50;
        let nav_mesh = create_large_navmesh(tiles_per_side)?;

        // Create multiple queries
        let mut queries: Vec<NavMeshQuery> =
            (0..10).map(|_| NavMeshQuery::new(&nav_mesh)).collect();

        let filter = QueryFilter::default();

        // Perform many operations
        for i in 0..100 {
            let query_idx = i % queries.len();
            let query = &mut queries[query_idx];

            // Random positions across the large mesh
            let pos1 = [
                ((i * 7) % (tiles_per_side as usize)) as f32 * 32.0 + 16.0,
                0.0,
                ((i * 11) % (tiles_per_side as usize)) as f32 * 32.0 + 16.0,
            ];
            let pos2 = [
                (((i * 13 + 25) % (tiles_per_side as usize)) as f32) * 32.0 + 16.0,
                0.0,
                (((i * 17 + 25) % (tiles_per_side as usize)) as f32) * 32.0 + 16.0,
            ];

            let ext = [5.0, 5.0, 5.0];

            // Find nearest polygons
            if let Ok((start_ref, _)) = query.find_nearest_poly(&pos1, &ext, &filter) {
                if let Ok((end_ref, _)) = query.find_nearest_poly(&pos2, &ext, &filter) {
                    if start_ref.is_valid() && end_ref.is_valid() {
                        // Try pathfinding
                        match query.find_path(start_ref, end_ref, &pos1, &pos2, &filter) {
                            Ok(path) => {
                                // Generate straight path if path exists
                                if !path.is_empty() && path.len() < 1000 {
                                    let _ = query.find_straight_path(&pos1, &pos2, &path);
                                }
                            }
                            Err(_) => {
                                // Some paths might not exist, that's ok
                            }
                        }
                    }
                }
            }
        }

        println!("Stress test completed successfully");
        Ok(())
    }
}
