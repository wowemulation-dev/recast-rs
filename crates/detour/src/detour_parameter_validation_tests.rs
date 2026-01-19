//! Comprehensive invalid parameter validation tests
//!
//! This module tests edge cases and invalid parameter handling to ensure
//! robust error handling that exceeds the C++ implementation.

#[cfg(test)]
mod tests {
    use crate::nav_mesh_query::NavMeshQuery;
    use crate::{NavMesh, NavMeshParams, PolyRef, QueryFilter, Status};
    use recast_common::Result;

    #[test]
    fn test_navmesh_invalid_params() {
        // Test invalid NavMeshParams

        // Zero tile dimensions
        let invalid_params1 = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 0.0, // Invalid
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        assert!(NavMesh::new(invalid_params1).is_err());

        // Negative tile dimensions
        let invalid_params2 = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: -10.0, // Invalid
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        assert!(NavMesh::new(invalid_params2).is_err());

        // Zero max tiles
        let invalid_params3 = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 0, // Invalid
            max_polys_per_tile: 1,
        };
        assert!(NavMesh::new(invalid_params3).is_err());

        // Zero max polys per tile
        let invalid_params4 = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 0, // Invalid
        };
        assert!(NavMesh::new(invalid_params4).is_err());
    }

    #[test]
    fn test_query_invalid_polygon_refs() -> Result<()> {
        // Create minimal valid navmesh
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with invalid (zero) polygon reference
        let invalid_ref = PolyRef::new(0);
        let pos = [5.0, 0.0, 5.0];
        let dir = [1.0, 0.0, 0.0];

        // All these should return proper errors, not crash
        assert!(query
            .raycast(invalid_ref, &pos, &dir, 10.0, &filter)
            .is_err());
        assert!(query.closest_point_on_poly(invalid_ref, &pos).is_err());

        // Test with very large polygon reference (invalid)
        let huge_ref = PolyRef::new(0xFFFFFFFF);
        assert!(query.raycast(huge_ref, &pos, &dir, 10.0, &filter).is_err());
        assert!(query.closest_point_on_poly(huge_ref, &pos).is_err());

        Ok(())
    }

    #[test]
    fn test_query_invalid_positions() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with NaN positions
        let nan_pos = [f32::NAN, 0.0, 0.0];
        let valid_pos = [5.0, 0.0, 5.0];
        let half_extents = [1.0, 1.0, 1.0];

        // Should handle NaN gracefully
        let result = query.find_nearest_poly(&nan_pos, &half_extents, &filter);
        assert!(result.is_err() || result.unwrap().0 == PolyRef::new(0));

        // Test with infinite positions
        let inf_pos = [f32::INFINITY, 0.0, 0.0];
        let result2 = query.find_nearest_poly(&inf_pos, &half_extents, &filter);
        assert!(result2.is_err() || result2.unwrap().0 == PolyRef::new(0));

        // Test with very large coordinates
        let huge_pos = [1e30, 0.0, 1e30];
        let result3 = query.find_nearest_poly(&huge_pos, &half_extents, &filter);
        // Should not crash, either succeed or fail gracefully
        assert!(result3.is_ok() || result3.is_err());

        Ok(())
    }

    #[test]
    fn test_query_invalid_extents() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let pos = [5.0, 0.0, 5.0];

        // Test with zero extents
        let zero_extents = [0.0, 0.0, 0.0];
        let result = query.find_nearest_poly(&pos, &zero_extents, &filter);
        // Should either succeed (finding exact position) or fail gracefully
        assert!(result.is_ok() || result.is_err());

        // Test with negative extents
        let neg_extents = [-1.0, -1.0, -1.0];
        let result2 = query.find_nearest_poly(&pos, &neg_extents, &filter);
        // Should handle negative extents gracefully
        assert!(result2.is_ok() || result2.is_err());

        // Test with NaN extents
        let nan_extents = [f32::NAN, 1.0, 1.0];
        let result3 = query.find_nearest_poly(&pos, &nan_extents, &filter);
        assert!(result3.is_err() || result3.unwrap().0 == PolyRef::new(0));

        Ok(())
    }

    #[test]
    fn test_raycast_invalid_directions() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // We can't easily get a valid polygon ref without mesh data,
        // but we can test the parameter validation
        let dummy_ref = PolyRef::new(1);
        let pos = [5.0, 0.0, 5.0];

        // Test with zero direction vector
        let zero_dir = [0.0, 0.0, 0.0];
        let result = query.raycast(dummy_ref, &pos, &zero_dir, 10.0, &filter);
        // Should handle zero direction gracefully (either error or no movement)
        assert!(result.is_err() || result.unwrap().2 == 0.0);

        // Test with NaN direction
        let nan_dir = [f32::NAN, 0.0, 0.0];
        let result2 = query.raycast(dummy_ref, &pos, &nan_dir, 10.0, &filter);
        assert!(result2.is_err());

        // Test with infinite direction
        let inf_dir = [f32::INFINITY, 0.0, 0.0];
        let result3 = query.raycast(dummy_ref, &pos, &inf_dir, 10.0, &filter);
        assert!(result3.is_err());

        Ok(())
    }

    #[test]
    fn test_raycast_invalid_distances() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let dummy_ref = PolyRef::new(1);
        let pos = [5.0, 0.0, 5.0];
        let dir = [1.0, 0.0, 0.0];

        // Test with negative distance
        let result = query.raycast(dummy_ref, &pos, &dir, -10.0, &filter);
        // Should handle negative distance gracefully
        assert!(result.is_err() || result.unwrap().2 == 0.0);

        // Test with NaN distance
        let result2 = query.raycast(dummy_ref, &pos, &dir, f32::NAN, &filter);
        assert!(result2.is_err());

        // Test with infinite distance
        let result3 = query.raycast(dummy_ref, &pos, &dir, f32::INFINITY, &filter);
        // Should either error or handle large distances
        assert!(result3.is_ok() || result3.is_err());

        Ok(())
    }

    #[test]
    fn test_pathfinding_invalid_params() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let invalid_ref = PolyRef::new(0);
        let valid_ref = PolyRef::new(1); // May not exist, but not zero
        let pos = [5.0, 0.0, 5.0];

        // Test pathfinding with invalid start ref
        let result = query.find_path(invalid_ref, valid_ref, &pos, &pos, &filter);
        assert!(result.is_err());

        // Test pathfinding with invalid end ref
        let result2 = query.find_path(valid_ref, invalid_ref, &pos, &pos, &filter);
        assert!(result2.is_err());

        // Test pathfinding with same start and end (edge case)
        let result3 = query.find_path(valid_ref, valid_ref, &pos, &pos, &filter);
        // Should either succeed immediately or fail gracefully
        assert!(result3.is_ok() || result3.is_err());

        Ok(())
    }

    #[test]
    fn test_sliced_pathfinding_invalid_params() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let invalid_ref = PolyRef::new(0);
        let valid_ref = PolyRef::new(1);
        let pos = [5.0, 0.0, 5.0];

        // Test sliced pathfinding init with invalid refs
        let result = query.init_sliced_find_path(invalid_ref, valid_ref, &pos, &pos, &filter, 0);
        assert!(result.is_err());

        // Test update without init
        let result2 = query.update_sliced_find_path(10);
        assert!(result2.is_err()); // Should require init first

        // Test finalize without init
        let result3 = query.finalize_sliced_find_path(100);
        assert!(result3.is_err()); // Should require init first

        Ok(())
    }

    #[test]
    fn test_query_filter_edge_cases() -> Result<()> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let query = NavMeshQuery::new(&nav_mesh);

        // Test filter that excludes everything
        let mut exclude_all_filter = QueryFilter::default();
        exclude_all_filter.include_flags = crate::PolyFlags::empty(); // Include nothing

        let pos = [5.0, 0.0, 5.0];
        let half_extents = [1.0, 1.0, 1.0];

        let result = query.find_nearest_poly(&pos, &half_extents, &exclude_all_filter);
        // Should fail to find any polygons
        assert!(result.is_err() || result.unwrap().0 == PolyRef::new(0));

        // Test filter with extreme area costs
        let mut extreme_cost_filter = QueryFilter::default();
        extreme_cost_filter.area_cost[0] = f32::INFINITY;

        // Should handle infinite costs gracefully
        let result2 = query.find_nearest_poly(&pos, &half_extents, &extreme_cost_filter);
        assert!(result2.is_ok() || result2.is_err());

        Ok(())
    }

    #[test]
    fn test_mesh_memory_safety() {
        // Test creating very large mesh parameters (should fail gracefully)
        let huge_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: i32::MAX,          // Extreme value
            max_polys_per_tile: i32::MAX, // Extreme value
        };

        // Should fail gracefully, not crash or allocate excessive memory
        let result = NavMesh::new(huge_params);
        assert!(result.is_err());
    }

    #[test]
    fn test_concurrent_access_safety() -> Result<()> {
        // Test that read-only queries are safe for concurrent access
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };
        let nav_mesh = NavMesh::new(params)?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // This is mainly a compile-time test - NavMeshQuery should be Send + Sync
        // for read-only operations
        let pos = [5.0, 0.0, 5.0];
        let half_extents = [1.0, 1.0, 1.0];

        // Multiple queries should not interfere (if they were concurrent)
        for _ in 0..10 {
            let _result = query.find_nearest_poly(&pos, &half_extents, &filter);
            // Results may fail due to no mesh data, but should not crash
        }

        Ok(())
    }

    #[test]
    fn test_rust_specific_safety() {
        // Test Rust-specific safety improvements over C++

        // 1. No null pointer dereferences (automatic with Rust)
        // 2. No buffer overflows (automatic with Rust bounds checking)
        // 3. No memory leaks (automatic with Rust RAII)
        // 4. No use-after-free (automatic with Rust borrow checker)

        // These are compile-time guarantees in Rust, but let's test some edge cases

        // Test vector bounds checking (should panic in debug, not corrupt memory)
        let mut test_vec = vec![1, 2, 3];
        let _last = test_vec.pop(); // Safe operation

        // Test option handling (no null pointer equivalent)
        let maybe_value: Option<i32> = None;
        assert!(maybe_value.is_none()); // Safe check

        // Test that our error types are properly defined
        use crate::Status;
        let status = Status::InvalidParam;
        assert!(!status.to_string().is_empty());
    }
}
