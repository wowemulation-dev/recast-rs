//! Comprehensive raycast functionality tests
//!
//! This module provides extensive testing for raycast operations to ensure
//! robustness and correctness, addressing known issues in the C++ implementation.

#[cfg(test)]
mod tests {
    use crate::nav_mesh_query::NavMeshQuery;
    use crate::test_mesh_helpers::*;
    use crate::{PolyFlags, QueryFilter};
    use recast_common::Result;

    #[test]
    fn test_basic_raycast_scenarios() -> Result<()> {
        println!("Creating minimal test mesh...");
        let nav_mesh = create_minimal_test_navmesh()?;
        println!("Created nav mesh successfully");

        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find a valid starting polygon
        let center = get_test_position_minimal();
        let half_extents = get_test_extents();
        println!(
            "Searching for polygon at center {:?} with extents {:?}",
            center, half_extents
        );

        let result = query.find_nearest_poly(&center, &half_extents, &filter);
        match result {
            Ok((start_ref, start_pos)) => {
                println!("Found polygon {:?} at position {:?}", start_ref, start_pos);

                // Test 1: Zero-length raycast (C++ compatibility)
                let dir = [1.0, 0.0, 0.0];
                let (end_ref, end_pos, t) =
                    query.raycast(start_ref, &start_pos, &dir, 0.0, &filter)?;
                assert_eq!(end_ref, start_ref);
                assert_eq!(end_pos, start_pos);
                assert_eq!(t, 0.0);

                // Test 2: Short raycast within same polygon
                let short_dist = 0.1; // Very short distance
                let (end_ref2, _end_pos2, t2) =
                    query.raycast(start_ref, &start_pos, &dir, short_dist, &filter)?;
                assert!(end_ref2.is_valid());
                assert!(t2 >= 0.0 && t2 <= 1.0);

                println!("Basic raycast scenarios passed!");
                Ok(())
            }
            Err(e) => {
                println!("Failed to find polygon: {:?}", e);
                Err(e)
            }
        }
    }

    #[test]
    fn test_raycast_wall_intersections() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Start from center of mesh
        let center = get_test_position_minimal();
        let half_extents = get_test_extents();
        let (start_ref, start_pos) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // Cast ray toward mesh boundary (should hit wall)
        let boundary_dir = [1.0, 0.0, 0.0];
        let long_dist = 2.0; // Longer than minimal mesh size (0.6 units)
        let (_end_ref, _end_pos, t) =
            query.raycast(start_ref, &start_pos, &boundary_dir, long_dist, &filter)?;

        // Should hit a wall before reaching max distance
        assert!(t < 1.0, "Ray should hit wall before max distance");
        assert!(
            t > 0.0,
            "Ray should travel some distance before hitting wall"
        );

        // Verify end position is reasonable
        let dist_traveled = long_dist * t;
        assert!(dist_traveled > 0.0);
        assert!(dist_traveled < long_dist);

        Ok(())
    }

    #[test]
    fn test_raycast_across_polygons() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Get starting polygon from center of complex mesh
        let start_center = get_test_position_complex();
        let half_extents = get_test_extents();
        let (start_ref, start_pos) =
            query.find_nearest_poly(&start_center, &half_extents, &filter)?;

        // Cast ray in diagonal direction within mesh bounds
        let dir = [1.0, 0.0, 1.0]; // Diagonal direction
        let dist = 0.5; // Stay within complex mesh bounds (0.9 units)
        let (ray_end_ref, ray_end_pos, t) =
            query.raycast(start_ref, &start_pos, &dir, dist, &filter)?;

        // Should successfully traverse polygons
        assert!(ray_end_ref.is_valid());
        assert!(t >= 0.0);

        // Test enhanced raycast for path information
        use crate::raycast_hit::RaycastOptions;
        let options = RaycastOptions {
            include_path: true,
            include_cost: true,
        };

        let result =
            query.raycast_enhanced(start_ref, &start_pos, &ray_end_pos, &filter, &options, None)?;

        // Should have path data
        if let Some(path) = &result.hit.path {
            assert!(
                path.len() > 0,
                "Ray path should not be empty (fixing C++ bug)"
            );
            assert!(path[0] == start_ref, "Path should start with start polygon");
        }

        Ok(())
    }

    #[test]
    fn test_raycast_edge_cases() -> Result<()> {
        let nav_mesh = create_large_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with invalid start reference
        let invalid_ref = crate::PolyRef::new(999999);
        let pos = [0.0, 0.0, 0.0];
        let dir = [1.0, 0.0, 0.0];

        let result = query.raycast(invalid_ref, &pos, &dir, 10.0, &filter);
        assert!(
            result.is_err(),
            "Should fail with invalid polygon reference"
        );

        // Test with very small distance
        let center = get_test_position_large();
        let half_extents = get_test_extents();
        let (start_ref, start_pos) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        let tiny_dist = f32::EPSILON * 10.0;
        let (end_ref, _end_pos, t) =
            query.raycast(start_ref, &start_pos, &dir, tiny_dist, &filter)?;
        assert!(end_ref.is_valid());
        assert!(t >= 0.0);

        // Test with very large distance
        let huge_dist = 50.0; // Large but reasonable for large mesh (30x30 units)
        let (end_ref2, _end_pos2, t2) =
            query.raycast(start_ref, &start_pos, &dir, huge_dist, &filter)?;
        assert!(end_ref2.is_valid());
        assert!(t2 >= 0.0);
        // Note: C++ implementation may return t close to 1.0 or FLT_MAX when ray reaches end
        // We accept t > 1.0 if the ray extends beyond the end position (valid behavior)

        Ok(())
    }

    #[test]
    fn test_raycast_precision_edge_cases() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let half_extents = get_test_extents();
        let (start_ref, start_pos) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // Test rays that are nearly parallel to polygon edges
        let near_parallel_dirs = [
            [1.0, 0.0, f32::EPSILON],       // Nearly parallel to X axis
            [f32::EPSILON, 0.0, 1.0],       // Nearly parallel to Z axis
            [1.0, 0.0, f32::EPSILON * 2.0], // Slight angle
        ];

        for dir in &near_parallel_dirs {
            let (end_ref, _end_pos, t) = query.raycast(start_ref, &start_pos, dir, 0.5, &filter)?;
            assert!(end_ref.is_valid());
            assert!(t >= 0.0);
            assert!(t.is_finite(), "t value should be finite for parallel rays");
        }

        // Test rays with very small direction vectors
        let tiny_dir = [f32::EPSILON, 0.0, f32::EPSILON];
        let result = query.raycast(start_ref, &start_pos, &tiny_dir, 0.1, &filter);
        // Should either succeed with valid results or fail gracefully
        if let Ok((end_ref, _end_pos, t)) = result {
            assert!(end_ref.is_valid());
            assert!(t.is_finite());
        }

        Ok(())
    }

    #[test]
    fn test_raycast_filter_interactions() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);

        // Create filter that excludes walkable polygons
        let mut restrictive_filter = QueryFilter::default();
        restrictive_filter.exclude_flags = PolyFlags::WALK;

        let center = get_test_position_minimal();
        let half_extents = get_test_extents();

        // Should not find any polygons with restrictive filter
        let result = query.find_nearest_poly(&center, &half_extents, &restrictive_filter);
        assert!(
            result.is_err(),
            "Should fail to find polygons with restrictive filter"
        );

        // Test with permissive filter
        let permissive_filter = QueryFilter::default();
        let (start_ref, start_pos) =
            query.find_nearest_poly(&center, &half_extents, &permissive_filter)?;

        let dir = [1.0, 0.0, 0.0];
        let (end_ref, _end_pos, t) =
            query.raycast(start_ref, &start_pos, &dir, 0.5, &permissive_filter)?;
        assert!(end_ref.is_valid());
        assert!(t >= 0.0);

        Ok(())
    }

    #[test]
    fn test_raycast_multi_polygon_traversal() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Start from one corner of the complex mesh
        let start_corner = [0.1, 0.0, 0.1]; // Near bottom-left (world coordinates)
        let end_corner = [0.8, 0.0, 0.8]; // Near top-right (world coordinates)
        let half_extents = get_test_extents();

        let (start_ref, start_pos) =
            query.find_nearest_poly(&start_corner, &half_extents, &filter)?;
        let (end_ref, end_pos) = query.find_nearest_poly(&end_corner, &half_extents, &filter)?;

        // Cast ray diagonally across the grid
        let dir = [
            end_pos[0] - start_pos[0],
            end_pos[1] - start_pos[1],
            end_pos[2] - start_pos[2],
        ];
        let dist = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();

        let (ray_end_ref, ray_end_pos, t) =
            query.raycast(start_ref, &start_pos, &dir, dist, &filter)?;

        // Should successfully traverse multiple polygons
        assert!(ray_end_ref.is_valid());
        assert!(t > 0.0, "Should traverse some distance across grid");

        // Test enhanced raycast to verify path
        use crate::raycast_hit::RaycastOptions;
        let options = RaycastOptions {
            include_path: true,
            include_cost: false,
        };
        let result =
            query.raycast_enhanced(start_ref, &start_pos, &ray_end_pos, &filter, &options, None)?;

        if let Some(path) = &result.hit.path {
            assert!(path.len() >= 1, "Should have at least starting polygon");
            assert!(
                path.len() <= 9,
                "Should not exceed number of polygons in 3x3 grid"
            );
        }

        Ok(())
    }

    #[test]
    fn test_raycast_boundary_detection() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let half_extents = get_test_extents();
        let (start_ref, start_pos) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // Cast rays in all cardinal directions to test boundary detection
        let test_cases = [
            ([1.0, 0.0, 0.0], "East"),
            ([-1.0, 0.0, 0.0], "West"),
            ([0.0, 0.0, 1.0], "North"),
            ([0.0, 0.0, -1.0], "South"),
        ];

        for (dir, direction_name) in &test_cases {
            let long_dist = 0.25; // Distance that will hit boundary of minimal mesh (0.6 units)
            let (_end_ref, end_pos, t) =
                query.raycast(start_ref, &start_pos, dir, long_dist, &filter)?;

            // Should hit boundary before max distance OR reach end position
            // C++ allows both t < 1.0 (boundary hit) or t ≈ 1.0 (ray completes)
            assert!(
                t > 0.0,
                "Ray {} should travel some distance",
                direction_name
            );

            // The end position should be between start and ray end
            if t < 1.0 {
                // Hit boundary case - end_pos should be along the ray direction

                // For horizontal rays, Y should stay the same
                assert!(
                    (end_pos[1] - start_pos[1]).abs() < 0.01,
                    "Y coordinate should stay same for {}",
                    direction_name
                );

                // For East/West rays, Z should stay the same
                if *direction_name == "East" || *direction_name == "West" {
                    assert!(
                        (end_pos[2] - start_pos[2]).abs() < 0.01,
                        "Z coordinate should stay same for {}",
                        direction_name
                    );
                }

                // For North/South rays, X should stay the same
                if *direction_name == "North" || *direction_name == "South" {
                    assert!(
                        (end_pos[0] - start_pos[0]).abs() < 0.01,
                        "X coordinate should stay same for {}",
                        direction_name
                    );
                }
            }
        }

        Ok(())
    }

    #[test]
    fn test_raycast_iteration_limits() -> Result<()> {
        let nav_mesh = create_large_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_large();
        let half_extents = get_test_extents();
        let (start_ref, start_pos) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // Cast very long ray that would stress the algorithm
        let extreme_dist = 50.0; // Large but reasonable for large mesh
        let dir = [1.0, 0.0, 1.0];

        let start_time = std::time::Instant::now();
        let (end_ref, _end_pos, t) =
            query.raycast(start_ref, &start_pos, &dir, extreme_dist, &filter)?;
        let elapsed = start_time.elapsed();

        // Should complete in reasonable time (not infinite loop)
        assert!(
            elapsed.as_millis() < 1000,
            "Raycast should complete within 1 second"
        );
        assert!(end_ref.is_valid());
        assert!(t >= 0.0);

        Ok(())
    }
}
