//! Edge case tests for Detour navigation mesh operations
//!
//! This module tests boundary conditions, numerical edge cases, and
//! unusual but valid scenarios that might occur in production use.

#[cfg(test)]
mod tests {
    use crate::test_mesh_helpers::create_minimal_test_navmesh;
    use crate::{
        nav_mesh::encode_poly_ref_with_salt, NavMesh, NavMeshParams, NavMeshQuery, PolyFlags,
        PolyRef, QueryFilter, MAX_VERTS_PER_POLY,
    };
    use recast_common::Result;

    /// Test operations on an empty navigation mesh
    #[test]
    fn test_empty_navmesh_operations() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 100,
        };

        let nav_mesh = NavMesh::new(params)?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test find_nearest_poly on empty mesh
        let pos = [5.0, 0.0, 5.0];
        let extents = [1.0, 1.0, 1.0];
        let result = query.find_nearest_poly(&pos, &extents, &filter);

        // Should succeed but return invalid poly ref
        match result {
            Ok((poly_ref, _)) => assert!(
                !poly_ref.is_valid(),
                "Empty mesh should return invalid poly ref"
            ),
            Err(_) => {} // Also acceptable
        }

        // Test pathfinding on empty mesh
        let invalid_ref = PolyRef::from(0);
        let path_result = query.find_path(invalid_ref, invalid_ref, &pos, &pos, &filter);
        assert!(
            path_result.is_err(),
            "Pathfinding on empty mesh should fail"
        );

        Ok(())
    }

    /// Test operations with single polygon mesh
    #[test]
    fn test_single_polygon_mesh() -> Result<()> {
        // Create a mesh with just one triangle polygon
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find the single polygon
        let center = [0.3, 0.0, 0.3];
        let extents = [0.5, 0.5, 0.5];
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;
        assert!(poly_ref.is_valid(), "Should find the single polygon");

        // Path from polygon to itself should succeed with single node
        let path = query.find_path(poly_ref, poly_ref, &center, &center, &filter)?;
        assert_eq!(path.len(), 1, "Path within same polygon should have 1 node");
        assert_eq!(path[0], poly_ref, "Path should contain the polygon");

        // Raycast within single polygon
        let start = [0.2, 0.0, 0.2];
        let end = [0.4, 0.0, 0.4];
        let (hit_ref, _, t) = query.raycast(poly_ref, &start, &end, 1.0, &filter)?;
        assert_eq!(hit_ref, poly_ref, "Should stay in same polygon");
        // Allow small floating point differences
        assert!(
            (t - 1.0).abs() < 0.01 || t <= 1.0,
            "Should reach full distance or hit boundary (t={:.6})",
            t
        );

        Ok(())
    }

    /// Test queries at exact polygon boundaries
    #[test]
    fn test_polygon_boundary_queries() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test point exactly on polygon edge
        let edge_point = [0.3, 0.0, 0.0]; // On bottom edge
        let tiny_extent = [0.01, 0.01, 0.01];

        let result = query.find_nearest_poly(&edge_point, &tiny_extent, &filter);
        assert!(result.is_ok(), "Should handle edge point query");

        // Test point exactly on polygon vertex
        let vertex_point = [0.0, 0.0, 0.0]; // Bottom-left vertex
        let result = query.find_nearest_poly(&vertex_point, &tiny_extent, &filter);
        assert!(result.is_ok(), "Should handle vertex point query");

        Ok(())
    }

    /// Test maximum polygon size (MAX_VERTS_PER_POLY vertices)
    #[test]
    fn test_maximum_polygon_vertices() -> Result<()> {
        // This tests that we handle polygons with the maximum allowed vertices
        // In practice, this would require a custom mesh builder
        assert_eq!(
            MAX_VERTS_PER_POLY, 6,
            "Maximum vertices per polygon should be 6"
        );

        // TODO: When we have proper mesh building, create a hexagon polygon
        // and test operations on it

        Ok(())
    }

    /// Test degenerate polygon cases
    #[test]
    fn test_degenerate_polygons() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Test with zero-sized search extent
        let pos = [0.3, 0.0, 0.3];
        let zero_extent = [0.0, 0.0, 0.0];
        let filter = QueryFilter::default();

        let result = query.find_nearest_poly(&pos, &zero_extent, &filter);
        // Zero extent should still work if point is inside polygon
        assert!(result.is_ok(), "Zero extent search should work");

        Ok(())
    }

    /// Test extremely long paths
    #[test]
    fn test_extremely_long_paths() -> Result<()> {
        // With our minimal mesh, we can't create truly long paths
        // This is a placeholder for when we have multi-tile support
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // For now, just verify path finding doesn't panic with distant points
        let start = [0.1, 0.0, 0.1];
        let end = [0.5, 0.0, 0.5];
        let extents = [1.0, 1.0, 1.0];

        let (start_ref, start_pos) = query.find_nearest_poly(&start, &extents, &filter)?;
        let (end_ref, end_pos) = query.find_nearest_poly(&end, &extents, &filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            let result = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &filter);
            assert!(
                result.is_ok(),
                "Path finding should handle any valid points"
            );
        }

        Ok(())
    }

    /// Test numerical precision edge cases
    #[test]
    fn test_numerical_precision_edge_cases() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with very small epsilon differences
        let base_pos = [0.3, 0.0, 0.3];
        let epsilon = f32::EPSILON;
        let nearby_pos = [base_pos[0] + epsilon, base_pos[1], base_pos[2] + epsilon];

        let extents = [0.1, 0.1, 0.1];
        let (ref1, _) = query.find_nearest_poly(&base_pos, &extents, &filter)?;
        let (ref2, _) = query.find_nearest_poly(&nearby_pos, &extents, &filter)?;

        assert_eq!(ref1, ref2, "Epsilon differences should find same polygon");

        // Test with maximum float values (within reason)
        let large_pos = [1000.0, 0.0, 1000.0];
        let large_extents = [1000.0, 1000.0, 1000.0];
        let result = query.find_nearest_poly(&large_pos, &large_extents, &filter);
        assert!(result.is_ok(), "Should handle large coordinate values");

        Ok(())
    }

    /// Test raycast edge cases
    #[test]
    fn test_raycast_edge_cases() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = [0.3, 0.0, 0.3];
        let extents = [0.1, 0.1, 0.1];
        let (poly_ref, pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Test zero-length raycast
        let (hit_ref, hit_pos, t) = query.raycast(poly_ref, &pos, &pos, 0.0, &filter)?;
        assert_eq!(
            hit_ref, poly_ref,
            "Zero-length ray should stay in same polygon"
        );
        assert_eq!(t, 0.0, "Zero-length ray should have t=0");

        // Test ray parallel to polygon edge
        let start = [0.1, 0.0, 0.0];
        let end = [0.5, 0.0, 0.0]; // Along bottom edge
        let result = query.raycast(poly_ref, &start, &end, 1.0, &filter);
        assert!(result.is_ok(), "Parallel ray should be handled");

        // Test ray with negative direction components
        let backward_end = [-0.1, 0.0, -0.1];
        let result = query.raycast(poly_ref, &center, &backward_end, 1.0, &filter);
        assert!(result.is_ok(), "Backward ray should be handled");

        Ok(())
    }

    /// Test invalid polygon references
    #[test]
    fn test_invalid_polygon_references() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with null reference
        let null_ref = PolyRef::from(0);
        let pos = [0.0, 0.0, 0.0];

        let result = query.find_path(null_ref, null_ref, &pos, &pos, &filter);
        assert!(result.is_err(), "Null references should fail");

        // Test with non-existent polygon ID
        let invalid_ref = encode_poly_ref_with_salt(1, 0, 999);
        let result = query.find_path(invalid_ref, invalid_ref, &pos, &pos, &filter);
        assert!(result.is_err(), "Invalid polygon ID should fail");

        // Test closest_point_on_poly with invalid ref
        let result = query.closest_point_on_poly(null_ref, &pos);
        assert!(
            result.is_err(),
            "Invalid ref should fail for closest_point_on_poly"
        );

        Ok(())
    }

    /// Test query filter edge cases
    #[test]
    fn test_query_filter_edge_cases() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Test with filter that excludes all areas
        let mut exclusive_filter = QueryFilter::default();
        exclusive_filter.exclude_flags = PolyFlags::all(); // All flags set

        let pos = [0.3, 0.0, 0.3];
        let extents = [0.5, 0.5, 0.5];
        let result = query.find_nearest_poly(&pos, &extents, &exclusive_filter);

        // Should either fail or return invalid poly
        match result {
            Ok((poly_ref, _)) => {
                assert!(!poly_ref.is_valid(), "Exclusive filter should find nothing")
            }
            Err(_) => {} // Also acceptable
        }

        // Test with maximum area cost
        let mut expensive_filter = QueryFilter::default();
        expensive_filter.area_cost[0] = f32::MAX;

        // Should still work but with high cost
        let result = query.find_nearest_poly(&pos, &extents, &expensive_filter);
        assert!(result.is_ok(), "High cost filter should still work");

        Ok(())
    }

    /// Test concurrent query operations (basic thread safety)
    #[test]
    fn test_concurrent_queries() -> Result<()> {
        use std::sync::Arc;
        use std::thread;

        let nav_mesh = Arc::new(create_minimal_test_navmesh()?);
        let mut handles = vec![];

        // Spawn multiple threads doing queries
        for i in 0..10 {
            let mesh_clone = Arc::clone(&nav_mesh);
            let handle = thread::spawn(move || {
                let mut query = NavMeshQuery::new(&mesh_clone);
                let filter = QueryFilter::default();

                let pos = [0.3 + (i as f32 * 0.01), 0.0, 0.3];
                let extents = [0.1, 0.1, 0.1];

                for _ in 0..100 {
                    let result = query.find_nearest_poly(&pos, &extents, &filter);
                    assert!(result.is_ok(), "Concurrent query should succeed");
                }
            });
            handles.push(handle);
        }

        // Wait for all threads
        for handle in handles {
            handle.join().expect("Thread should complete");
        }

        Ok(())
    }

    /// Test pathfinding with obstacles (when implemented)
    #[test]
    #[ignore] // TODO: Enable when dynamic obstacles are implemented
    fn test_pathfinding_with_dynamic_obstacles() -> Result<()> {
        // Placeholder for dynamic obstacle testing
        Ok(())
    }

    /// Test mesh modification edge cases (when implemented)
    #[test]
    #[ignore] // TODO: Enable when mesh modification is implemented
    fn test_mesh_modification_edge_cases() -> Result<()> {
        // Placeholder for testing adding/removing tiles
        Ok(())
    }
}
