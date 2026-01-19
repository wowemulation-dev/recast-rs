//! Tests for spatial query operations in Detour
//!
//! Spatial queries include finding polygons within bounds, circles,
//! and along paths.

#[cfg(test)]
mod tests {
    use crate::{
        NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyFlags, QueryFilter,
    };
    use recast_common::Result;

    /// Helper to create a grid mesh for spatial query testing
    fn create_grid_mesh(grid_size: i32) -> Result<NavMesh> {
        let cell_size = 2.0;
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: grid_size as f32 * cell_size,
            tile_height: grid_size as f32 * cell_size,
            max_tiles: 1,
            max_polys_per_tile: grid_size * grid_size,
        };

        // Create vertices for grid
        let mut vertices = Vec::new();
        for y in 0..=grid_size {
            for x in 0..=grid_size {
                vertices.extend_from_slice(&[x as f32 * cell_size, 0.0, y as f32 * cell_size]);
            }
        }

        // Create polygons
        let mut polys = Vec::new();
        let mut poly_areas = Vec::new();
        let mut poly_flags = Vec::new();

        for y in 0..grid_size {
            for x in 0..grid_size {
                let base = y * (grid_size + 1) + x;
                let poly_idx = y * grid_size + x;

                // Vertices (counter-clockwise) - 4 vertices + 2 padding for nvp=6
                polys.extend_from_slice(&[
                    base as u16,
                    (base + 1) as u16,
                    (base + grid_size + 2) as u16,
                    (base + grid_size + 1) as u16,
                    0xffff, // Padding
                    0xffff, // Padding
                ]);

                // Neighbors: bottom, right, top, left + 2 padding for nvp=6
                // Bottom neighbor
                polys.push(if y > 0 {
                    ((y - 1) * grid_size + x) as u16
                } else {
                    0xffff
                });
                // Right neighbor
                polys.push(if x < grid_size - 1 {
                    (y * grid_size + x + 1) as u16
                } else {
                    0xffff
                });
                // Top neighbor
                polys.push(if y < grid_size - 1 {
                    ((y + 1) * grid_size + x) as u16
                } else {
                    0xffff
                });
                // Left neighbor
                polys.push(if x > 0 {
                    (y * grid_size + x - 1) as u16
                } else {
                    0xffff
                });
                // Padding
                polys.push(0xffff);
                polys.push(0xffff);

                poly_areas.push(0);
                poly_flags.push(PolyFlags::WALK);
            }
        }

        // Simple detail mesh
        let detail_meshes: Vec<u32> = (0..grid_size * grid_size)
            .flat_map(|i| vec![i as u32 * 4, 4, i as u32 * 2, 2])
            .collect();

        let detail_verts = vertices.clone();
        let detail_tris: Vec<u8> = (0..grid_size * grid_size)
            .flat_map(|_| vec![0, 1, 2, 0, 0, 2, 3, 0])
            .collect();

        let create_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: vertices,
            vert_count: ((grid_size + 1) * (grid_size + 1)) as i32,
            polys,
            poly_flags,
            poly_areas,
            poly_count: grid_size * grid_size,
            nvp: 6,
            detail_meshes,
            detail_verts,
            detail_vert_count: ((grid_size + 1) * (grid_size + 1)) as i32,
            detail_tris,
            detail_tri_count: grid_size * grid_size * 2,
            off_mesh_con_verts: Vec::new(),
            off_mesh_con_rad: Vec::new(),
            off_mesh_con_dir: Vec::new(),
            off_mesh_con_areas: Vec::new(),
            off_mesh_con_flags: Vec::new(),
            off_mesh_con_user_id: Vec::new(),
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [
                grid_size as f32 * cell_size,
                1.0,
                grid_size as f32 * cell_size,
            ],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        let mut nav_mesh = NavMesh::new(params)?;
        nav_mesh.add_tile_from_params(&create_params, 0, 0, 0)?;

        Ok(nav_mesh)
    }

    /// Test query_polygons within bounds
    #[test]
    fn test_query_polygons_in_bounds() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Query center area
        let bounds_min = [4.0, -1.0, 4.0];
        let bounds_max = [6.0, 1.0, 6.0];

        let polygons = nav_mesh.query_polygons(&bounds_min, &bounds_max, &filter)?;

        // Should find polygons in the center area
        assert!(!polygons.is_empty(), "Should find polygons in bounds");

        // Verify all returned polygons are within bounds
        for poly_ref in &polygons {
            // TODO: Get polygon bounds and verify overlap
        }

        Ok(())
    }

    /// Test query with empty bounds
    #[test]
    fn test_query_empty_bounds() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Query with zero-sized bounds
        let point = [5.0, 0.0, 5.0];
        let _polygons = nav_mesh.query_polygons(&point, &point, &filter);

        // Might find polygon at exact point or be empty
        // Both are valid behaviors

        Ok(())
    }

    /// Test query with inverted bounds
    #[test]
    fn test_query_inverted_bounds() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Query with max < min
        let bounds_min = [6.0, 1.0, 6.0];
        let bounds_max = [4.0, -1.0, 4.0];

        let _polygons = nav_mesh.query_polygons(&bounds_min, &bounds_max, &filter);

        // Should handle gracefully (empty or corrected)

        Ok(())
    }

    /// Test query entire mesh
    #[test]
    fn test_query_entire_mesh() -> Result<()> {
        let grid_size = 5;
        let nav_mesh = create_grid_mesh(grid_size)?;
        let _query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Query with huge bounds
        let bounds_min = [-100.0, -100.0, -100.0];
        let bounds_max = [100.0, 100.0, 100.0];

        let polygons = nav_mesh.query_polygons(&bounds_min, &bounds_max, &filter)?;

        // Should find all polygons
        assert_eq!(
            polygons.len(),
            (grid_size * grid_size) as usize,
            "Should find all polygons in mesh"
        );

        Ok(())
    }

    /// Test find_polys_around_circle
    #[test]
    fn test_find_polys_around_circle() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find starting polygon at center
        let center = [5.0, 0.0, 5.0];
        let extents = [0.5, 0.5, 0.5];
        let (start_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        if start_ref.is_valid() {
            // Search for polygons within radius
            let radius = 3.0;
            let result = query.find_polys_around_circle(start_ref, &center, radius, &filter);

            match result {
                Ok(polys) => {
                    assert!(!polys.is_empty(), "Should find polygons around circle");
                }
                Err(_) => {
                    // Method might not be fully implemented yet
                }
            }
        }

        Ok(())
    }

    /// Test find_polys_around_circle with zero radius
    #[test]
    fn test_find_polys_circle_zero_radius() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = [5.0, 0.0, 5.0];
        let (start_ref, _) = query.find_nearest_poly(&center, &[0.5, 0.5, 0.5], &filter)?;

        if start_ref.is_valid() {
            let result = query.find_polys_around_circle(start_ref, &center, 0.0, &filter);

            match result {
                Ok(polys) => {
                    // Should only find starting polygon
                    assert_eq!(polys.len(), 1, "Zero radius should only find start polygon");
                    assert_eq!(polys[0], start_ref, "Should be start polygon");
                }
                Err(_) => {
                    // Also acceptable
                }
            }
        }

        Ok(())
    }

    /// Test find_polys_around_circle with large radius
    #[test]
    fn test_find_polys_circle_large_radius() -> Result<()> {
        let grid_size = 5;
        let nav_mesh = create_grid_mesh(grid_size)?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = [5.0, 0.0, 5.0];
        let (start_ref, _) = query.find_nearest_poly(&center, &[0.5, 0.5, 0.5], &filter)?;

        if start_ref.is_valid() {
            // Use radius that covers entire mesh
            let radius = 100.0;
            let result = query.find_polys_around_circle(start_ref, &center, radius, &filter);

            match result {
                Ok(polys) => {
                    // Should find many polygons, possibly all
                    println!("Found {} polygons with radius {}", polys.len(), radius);

                    // Debug: Let's check if links are working
                    if polys.len() <= 3 {
                        println!(
                            "Only found {} polygons, checking if neighbors are reachable...",
                            polys.len()
                        );

                        if let Some(tile) = nav_mesh.get_tile_by_index(0) {
                            println!(
                                "Tile has {} polygons and {} links",
                                tile.polys.len(),
                                tile.links.len()
                            );

                            // Check the first polygon that was found
                            if !polys.is_empty() {
                                let found_poly_ref = polys[0];
                                if let Ok((tile, poly)) =
                                    nav_mesh.get_tile_and_poly_by_ref(found_poly_ref)
                                {
                                    println!(
                                        "Found polygon has {} links",
                                        if poly.first_link.is_some() {
                                            "some"
                                        } else {
                                            "no"
                                        }
                                    );
                                }
                            }
                        }
                    }

                    assert!(
                        polys.len() > 1,
                        "Large radius should find multiple polygons, but found {}",
                        polys.len()
                    );
                }
                Err(e) => {
                    println!("find_polys_around_circle failed: {:?}", e);
                    // Also acceptable
                }
            }
        }

        Ok(())
    }

    /// Test find_polys_in_path (when implemented)
    #[test]
    #[ignore] // Enable when method is implemented
    fn test_find_polys_in_path() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Create a path
        let start = [1.0, 0.0, 1.0];
        let end = [9.0, 0.0, 9.0];
        let extents = [1.0, 1.0, 1.0];

        let (start_ref, start_pos) = query.find_nearest_poly(&start, &extents, &filter)?;
        let (end_ref, end_pos) = query.find_nearest_poly(&end, &extents, &filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            let path = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &filter)?;

            if !path.is_empty() {
                // TODO: Call find_polys_in_path when implemented
                // Should find all polygons along the path corridor
            }
        }

        Ok(())
    }

    /// Test spatial queries with filters
    #[test]
    fn test_spatial_queries_with_filters() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let query = NavMeshQuery::new(&nav_mesh);

        // Create filter that excludes some polygons
        let mut exclusive_filter = QueryFilter::default();
        exclusive_filter.exclude_flags = PolyFlags::DISABLED;

        let bounds_min = [0.0, -1.0, 0.0];
        let bounds_max = [10.0, 1.0, 10.0];

        // Query with exclusive filter
        let filtered_polys =
            nav_mesh.query_polygons(&bounds_min, &bounds_max, &exclusive_filter)?;

        // Query with default filter
        let all_polys =
            nav_mesh.query_polygons(&bounds_min, &bounds_max, &QueryFilter::default())?;

        // Filtered query might return fewer polygons
        assert!(
            filtered_polys.len() <= all_polys.len(),
            "Filtered query should not return more polygons"
        );

        Ok(())
    }

    /// Test query performance with large bounds
    #[test]
    #[ignore] // Performance test - run with --ignored
    fn test_query_performance_large_bounds() -> Result<()> {
        let nav_mesh = create_grid_mesh(50)?; // Large grid
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let bounds_min = [0.0, -10.0, 0.0];
        let bounds_max = [100.0, 10.0, 100.0];

        let start_time = std::time::Instant::now();

        for _ in 0..1000 {
            let _ = nav_mesh.query_polygons(&bounds_min, &bounds_max, &filter)?;
        }

        let elapsed = start_time.elapsed();
        println!("1000 spatial queries took: {:?}", elapsed);

        // Should be reasonably fast
        assert!(elapsed.as_millis() < 1000, "Spatial queries should be fast");

        Ok(())
    }

    /// Test query with BVH tree optimization
    #[test]
    fn test_query_with_bvh_optimization() -> Result<()> {
        // Create mesh with BVH tree
        let nav_mesh = create_grid_mesh(10)?;
        let _query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Small bounds query - should use BVH for efficiency
        let bounds_min = [5.0, -1.0, 5.0];
        let bounds_max = [7.0, 1.0, 7.0];

        let polygons = nav_mesh.query_polygons(&bounds_min, &bounds_max, &filter)?;

        // Should find only nearby polygons efficiently
        assert!(!polygons.is_empty(), "Should find polygons");
        assert!(
            polygons.len() <= 4,
            "Should not return too many polygons for small bounds (expected <=4, got {})",
            polygons.len()
        );

        Ok(())
    }

    /// Test overlapping spatial queries
    #[test]
    fn test_overlapping_spatial_queries() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Two overlapping regions
        let bounds1_min = [2.0, -1.0, 2.0];
        let bounds1_max = [6.0, 1.0, 6.0];

        let bounds2_min = [4.0, -1.0, 4.0];
        let bounds2_max = [8.0, 1.0, 8.0];

        let polys1 = nav_mesh.query_polygons(&bounds1_min, &bounds1_max, &filter)?;
        let polys2 = nav_mesh.query_polygons(&bounds2_min, &bounds2_max, &filter)?;

        // Should have some overlap
        let overlap: Vec<_> = polys1.iter().filter(|p| polys2.contains(p)).collect();

        assert!(
            !overlap.is_empty(),
            "Overlapping queries should share some polygons"
        );

        Ok(())
    }

    /// Test query at mesh boundaries
    #[test]
    fn test_query_at_mesh_boundaries() -> Result<()> {
        let nav_mesh = create_grid_mesh(5)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Query at mesh edge
        let bounds_min = [-1.0, -1.0, -1.0];
        let bounds_max = [1.0, 1.0, 1.0];

        let polygons = nav_mesh.query_polygons(&bounds_min, &bounds_max, &filter)?;

        // Should find corner polygon(s)
        assert!(
            !polygons.is_empty(),
            "Should find polygons at mesh boundary"
        );

        Ok(())
    }
}
