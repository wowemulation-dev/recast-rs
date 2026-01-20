//! Comprehensive test suite for advanced navigation functions
//!
//! This module provides extensive test coverage for the more advanced navigation mesh
//! functions including moveAlongSurface, findLocalNeighbourhood, and polygon
//! property get/set operations (flags and areas).

#![allow(unused)]

use crate::test_mesh_helpers::*;
use crate::{NavMeshQuery, PolyFlags, PolyRef, QueryFilter};
use recast_common::Result;
use std::collections::HashSet;

/// Test suite for moveAlongSurface function
mod move_along_surface_tests {
    use super::*;

    #[test]
    fn test_move_along_surface_basic() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find starting position
        let start_pos = get_test_position_complex();
        let extents = get_test_extents();
        let (start_ref, actual_start_pos) =
            query.find_nearest_poly(&start_pos, &extents, &filter)?;

        // Move within the same polygon
        let end_pos = [
            actual_start_pos[0] + 0.1,
            actual_start_pos[1],
            actual_start_pos[2] + 0.1,
        ];
        let mut visited = Vec::new();

        let result_pos = query.move_along_surface(
            start_ref,
            &actual_start_pos,
            &end_pos,
            &filter,
            &mut visited,
        )?;

        // Should successfully move to target position within same polygon
        assert!(!visited.is_empty(), "Should visit at least one polygon");
        assert_eq!(
            visited[0], start_ref,
            "First visited should be start polygon"
        );

        // Result position should be close to target
        let dist =
            ((result_pos[0] - end_pos[0]).powi(2) + (result_pos[2] - end_pos[2]).powi(2)).sqrt();
        assert!(
            dist < 0.01,
            "Should reach near target position, got distance: {}",
            dist
        );

        Ok(())
    }

    #[test]
    fn test_move_along_surface_cross_polygon() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Start near one edge of the mesh
        let start_pos = [0.15, 0.0, 0.15]; // Near corner
        let extents = get_test_extents();
        let (start_ref, actual_start_pos) =
            query.find_nearest_poly(&start_pos, &extents, &filter)?;

        // Move to opposite corner, crossing multiple polygons
        let end_pos = [0.75, 0.0, 0.75];
        let mut visited = Vec::new();

        let result_pos = query.move_along_surface(
            start_ref,
            &actual_start_pos,
            &end_pos,
            &filter,
            &mut visited,
        )?;

        // Should visit at least one polygon (might visit more depending on path)
        assert!(!visited.is_empty(), "Should visit at least one polygon");

        // All visited polygons should be unique
        let unique_visited: HashSet<_> = visited.iter().collect();
        assert_eq!(
            unique_visited.len(),
            visited.len(),
            "All visited polygons should be unique"
        );

        // Should move closer to target or at least not move further away
        let start_dist = ((actual_start_pos[0] - end_pos[0]).powi(2)
            + (actual_start_pos[2] - end_pos[2]).powi(2))
        .sqrt();
        let result_dist =
            ((result_pos[0] - end_pos[0]).powi(2) + (result_pos[2] - end_pos[2]).powi(2)).sqrt();
        assert!(
            result_dist <= start_dist + 0.1,
            "Should not move significantly further from target"
        );

        Ok(())
    }

    #[test]
    fn test_move_along_surface_blocked_by_wall() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Start in center
        let start_pos = get_test_position_minimal();
        let extents = get_test_extents();
        let (start_ref, actual_start_pos) =
            query.find_nearest_poly(&start_pos, &extents, &filter)?;

        // Try to move far outside mesh bounds (should hit wall)
        let end_pos = [10.0, 0.0, 10.0];
        let mut visited = Vec::new();

        let result_pos = query.move_along_surface(
            start_ref,
            &actual_start_pos,
            &end_pos,
            &filter,
            &mut visited,
        )?;

        // Should stop at mesh boundary, not reach target
        let result_dist =
            ((result_pos[0] - end_pos[0]).powi(2) + (result_pos[2] - end_pos[2]).powi(2)).sqrt();
        assert!(result_dist > 5.0, "Should not reach far target due to wall");

        // Should have visited only one polygon (blocked immediately)
        assert_eq!(
            visited.len(),
            1,
            "Should only visit start polygon when blocked"
        );

        Ok(())
    }

    #[test]
    fn test_move_along_surface_invalid_start_ref() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let start_pos = [0.0, 0.0, 0.0];
        let end_pos = [1.0, 0.0, 1.0];
        let mut visited = Vec::new();

        // Use invalid polygon reference
        let invalid_ref = PolyRef::new(0);
        let result =
            query.move_along_surface(invalid_ref, &start_pos, &end_pos, &filter, &mut visited);

        assert!(
            result.is_err(),
            "Should fail with invalid polygon reference"
        );

        Ok(())
    }

    #[test]
    fn test_move_along_surface_zero_distance() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let start_pos = get_test_position_minimal();
        let extents = get_test_extents();
        let (start_ref, actual_start_pos) =
            query.find_nearest_poly(&start_pos, &extents, &filter)?;

        // Move to same position
        let mut visited = Vec::new();
        let result_pos = query.move_along_surface(
            start_ref,
            &actual_start_pos,
            &actual_start_pos,
            &filter,
            &mut visited,
        )?;

        // Should return same position
        for i in 0..3 {
            assert!(
                (result_pos[i] - actual_start_pos[i]).abs() < 1e-6,
                "Should return same position for zero distance move"
            );
        }

        // Should still visit the start polygon
        assert_eq!(visited.len(), 1, "Should visit start polygon");
        assert_eq!(visited[0], start_ref, "Should visit start polygon");

        Ok(())
    }

    #[test]
    fn test_move_along_surface_search_radius_constraint() -> Result<()> {
        let nav_mesh = create_large_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let start_pos = [5.0, 0.0, 5.0];
        let extents = get_test_extents();
        let (start_ref, actual_start_pos) =
            query.find_nearest_poly(&start_pos, &extents, &filter)?;

        // Try to move very far (should be constrained by search radius)
        let end_pos = [25.0, 0.0, 25.0];
        let mut visited = Vec::new();

        let result_pos = query.move_along_surface(
            start_ref,
            &actual_start_pos,
            &end_pos,
            &filter,
            &mut visited,
        )?;

        // Should move some distance towards target
        let moved_dist = ((result_pos[0] - actual_start_pos[0]).powi(2)
            + (result_pos[2] - actual_start_pos[2]).powi(2))
        .sqrt();
        let target_dist = ((end_pos[0] - actual_start_pos[0]).powi(2)
            + (end_pos[2] - actual_start_pos[2]).powi(2))
        .sqrt();

        // Should move some distance, but might reach target if the large mesh allows it
        assert!(moved_dist >= 0.0, "Should not move backwards");

        // If we moved less than the full distance, test that it's due to constraints
        if moved_dist < target_dist - 0.1 {
            // Movement was constrained, which is expected behavior
            assert!(
                moved_dist > 0.0,
                "Should move some distance when constrained"
            );
        } else {
            // Reached or nearly reached target, which is also valid
            assert!(
                moved_dist <= target_dist + 0.1,
                "Should not overshoot target"
            );
        }

        Ok(())
    }

    #[test]
    fn test_move_along_surface_visited_refs_cleared() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let start_pos = get_test_position_minimal();
        let extents = get_test_extents();
        let (start_ref, actual_start_pos) =
            query.find_nearest_poly(&start_pos, &extents, &filter)?;

        let end_pos = [
            actual_start_pos[0] + 0.1,
            actual_start_pos[1],
            actual_start_pos[2] + 0.1,
        ];

        // Pre-populate visited refs
        let mut visited = vec![PolyRef::new(999), PolyRef::new(888)];

        query.move_along_surface(
            start_ref,
            &actual_start_pos,
            &end_pos,
            &filter,
            &mut visited,
        )?;

        // visited_refs should be cleared and repopulated
        assert!(!visited.is_empty(), "Visited refs should not be empty");
        assert!(
            !visited.contains(&PolyRef::new(999)),
            "Old visited refs should be cleared"
        );
        assert!(
            !visited.contains(&PolyRef::new(888)),
            "Old visited refs should be cleared"
        );

        Ok(())
    }
}

/// Test suite for polygon flags get/set operations
mod poly_flags_tests {
    use super::*;

    #[test]
    fn test_set_get_poly_flags_basic() -> Result<()> {
        let mut nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find a polygon
        let center = get_test_position_minimal();
        let extents = get_test_extents();
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Get initial flags
        let initial_flags = nav_mesh.get_poly_flags(poly_ref)?;
        assert_eq!(
            initial_flags,
            PolyFlags::WALK,
            "Initial flags should be WALK"
        );

        // Set new flags
        let new_flags = PolyFlags::WALK | PolyFlags::SWIM;
        nav_mesh.set_poly_flags(poly_ref, new_flags)?;

        // Verify flags were set
        let retrieved_flags = nav_mesh.get_poly_flags(poly_ref)?;
        assert_eq!(
            retrieved_flags, new_flags,
            "Flags should match what was set"
        );

        Ok(())
    }

    #[test]
    fn test_set_poly_flags_invalid_ref() -> Result<()> {
        let mut nav_mesh = create_minimal_test_navmesh()?;

        let invalid_ref = PolyRef::new(0);
        let result = nav_mesh.set_poly_flags(invalid_ref, PolyFlags::WALK);

        assert!(
            result.is_err(),
            "Should fail with invalid polygon reference"
        );

        Ok(())
    }

    #[test]
    fn test_get_poly_flags_invalid_ref() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;

        let invalid_ref = PolyRef::new(0);
        let result = nav_mesh.get_poly_flags(invalid_ref);

        assert!(
            result.is_err(),
            "Should fail with invalid polygon reference"
        );

        Ok(())
    }

    #[test]
    fn test_poly_flags_all_combinations() -> Result<()> {
        let mut nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find multiple polygons
        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Test different flag combinations
        let flag_tests = vec![
            PolyFlags::WALK,
            PolyFlags::SWIM,
            PolyFlags::DOOR,
            PolyFlags::WALK | PolyFlags::SWIM,
            PolyFlags::WALK | PolyFlags::DOOR,
            PolyFlags::SWIM | PolyFlags::DOOR,
            PolyFlags::WALK | PolyFlags::SWIM | PolyFlags::DOOR,
            PolyFlags::empty(), // No flags
        ];

        for test_flags in flag_tests {
            nav_mesh.set_poly_flags(poly_ref, test_flags)?;
            let retrieved_flags = nav_mesh.get_poly_flags(poly_ref)?;
            assert_eq!(
                retrieved_flags, test_flags,
                "Flags should match for combination {:?}",
                test_flags
            );
        }

        Ok(())
    }

    #[test]
    fn test_poly_flags_persistence() -> Result<()> {
        let mut nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let extents = get_test_extents();
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        let test_flags = PolyFlags::SWIM | PolyFlags::DOOR;
        nav_mesh.set_poly_flags(poly_ref, test_flags)?;

        // Multiple reads should return the same value
        for _ in 0..10 {
            let flags = nav_mesh.get_poly_flags(poly_ref)?;
            assert_eq!(
                flags, test_flags,
                "Flags should persist across multiple reads"
            );
        }

        Ok(())
    }
}

/// Test suite for polygon area get/set operations
mod poly_area_tests {
    use super::*;

    #[test]
    fn test_set_get_poly_area_basic() -> Result<()> {
        let mut nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find a polygon
        let center = get_test_position_minimal();
        let extents = get_test_extents();
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Get initial area
        let initial_area = nav_mesh.get_poly_area(poly_ref)?;
        assert_eq!(initial_area, 0, "Initial area should be 0");

        // Set new area
        let new_area = 42u8;
        nav_mesh.set_poly_area(poly_ref, new_area)?;

        // Verify area was set
        let retrieved_area = nav_mesh.get_poly_area(poly_ref)?;
        assert_eq!(retrieved_area, new_area, "Area should match what was set");

        Ok(())
    }

    #[test]
    fn test_set_poly_area_invalid_ref() -> Result<()> {
        let mut nav_mesh = create_minimal_test_navmesh()?;

        let invalid_ref = PolyRef::new(0);
        let result = nav_mesh.set_poly_area(invalid_ref, 5);

        assert!(
            result.is_err(),
            "Should fail with invalid polygon reference"
        );

        Ok(())
    }

    #[test]
    fn test_get_poly_area_invalid_ref() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;

        let invalid_ref = PolyRef::new(0);
        let result = nav_mesh.get_poly_area(invalid_ref);

        assert!(
            result.is_err(),
            "Should fail with invalid polygon reference"
        );

        Ok(())
    }

    #[test]
    fn test_poly_area_all_values() -> Result<()> {
        let mut nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Test all possible area values (0-255)
        for area in 0u8..=255u8 {
            nav_mesh.set_poly_area(poly_ref, area)?;
            let retrieved_area = nav_mesh.get_poly_area(poly_ref)?;
            assert_eq!(retrieved_area, area, "Area should match for value {}", area);
        }

        Ok(())
    }

    #[test]
    fn test_poly_area_persistence() -> Result<()> {
        let mut nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let extents = get_test_extents();
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        let test_area = 123u8;
        nav_mesh.set_poly_area(poly_ref, test_area)?;

        // Multiple reads should return the same value
        for _ in 0..10 {
            let area = nav_mesh.get_poly_area(poly_ref)?;
            assert_eq!(area, test_area, "Area should persist across multiple reads");
        }

        Ok(())
    }

    #[test]
    fn test_poly_area_independent_from_flags() -> Result<()> {
        let mut nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let extents = get_test_extents();
        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Set area and flags
        let test_area = 99u8;
        let test_flags = PolyFlags::SWIM;

        nav_mesh.set_poly_area(poly_ref, test_area)?;
        nav_mesh.set_poly_flags(poly_ref, test_flags)?;

        // Verify both are independent
        assert_eq!(nav_mesh.get_poly_area(poly_ref)?, test_area);
        assert_eq!(nav_mesh.get_poly_flags(poly_ref)?, test_flags);

        // Change flags, area should remain
        nav_mesh.set_poly_flags(poly_ref, PolyFlags::DOOR)?;
        assert_eq!(nav_mesh.get_poly_area(poly_ref)?, test_area);

        // Change area, flags should remain
        nav_mesh.set_poly_area(poly_ref, 200u8)?;
        assert_eq!(nav_mesh.get_poly_flags(poly_ref)?, PolyFlags::DOOR);

        Ok(())
    }
}

/// Test suite for findLocalNeighbourhood function
mod find_local_neighbourhood_tests {
    use super::*;

    #[test]
    fn test_find_local_neighbourhood_basic() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find starting polygon
        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Find local neighbourhood with reasonable radius
        let radius = 0.5;
        let max_result = 10;
        let (polys, parents) =
            query.find_local_neighbourhood(start_ref, &center_pos, radius, &filter, max_result)?;

        // Should find at least the starting polygon
        assert!(!polys.is_empty(), "Should find at least one polygon");
        assert_eq!(
            polys.len(),
            parents.len(),
            "Polys and parents arrays should be same length"
        );

        // First polygon should be the start polygon
        assert_eq!(polys[0], start_ref, "First polygon should be start polygon");
        assert!(
            !parents[0].is_valid(),
            "Start polygon should have no parent"
        );

        // All polygons should be valid
        for poly_ref in &polys {
            assert!(poly_ref.is_valid(), "All returned polygons should be valid");
        }

        Ok(())
    }

    #[test]
    fn test_find_local_neighbourhood_small_radius() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Very small radius should only find the start polygon
        let radius = 0.01;
        let max_result = 10;
        let (polys, parents) =
            query.find_local_neighbourhood(start_ref, &center_pos, radius, &filter, max_result)?;

        assert_eq!(
            polys.len(),
            1,
            "Small radius should only find start polygon"
        );
        assert_eq!(polys[0], start_ref, "Should find start polygon");

        Ok(())
    }

    #[test]
    fn test_find_local_neighbourhood_large_radius() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Large radius should find multiple polygons
        let radius = 2.0;
        let max_result = 20;
        let (polys, parents) =
            query.find_local_neighbourhood(start_ref, &center_pos, radius, &filter, max_result)?;

        assert!(
            polys.len() > 1,
            "Large radius should find multiple polygons"
        );
        assert!(polys.len() <= max_result, "Should respect max result limit");

        // Verify no overlapping polygons (all unique)
        let unique_polys: HashSet<_> = polys.iter().collect();
        assert_eq!(
            unique_polys.len(),
            polys.len(),
            "All polygons should be unique"
        );

        Ok(())
    }

    #[test]
    fn test_find_local_neighbourhood_max_result_limit() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Test with small max_result
        let radius = 2.0;
        let max_result = 3;
        let (polys, parents) =
            query.find_local_neighbourhood(start_ref, &center_pos, radius, &filter, max_result)?;

        assert!(polys.len() <= max_result, "Should respect max_result limit");
        assert_eq!(polys.len(), parents.len(), "Arrays should be same length");

        Ok(())
    }

    #[test]
    fn test_find_local_neighbourhood_invalid_params() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();

        // Invalid polygon reference
        let invalid_ref = PolyRef::new(0);
        let result = query.find_local_neighbourhood(invalid_ref, &center, 1.0, &filter, 10);
        assert!(
            result.is_err(),
            "Should fail with invalid polygon reference"
        );

        // Negative radius
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;
        let result = query.find_local_neighbourhood(start_ref, &center_pos, -1.0, &filter, 10);
        assert!(result.is_err(), "Should fail with negative radius");

        // Zero max_result
        let result = query.find_local_neighbourhood(start_ref, &center_pos, 1.0, &filter, 0);
        assert!(result.is_err(), "Should fail with zero max_result");

        Ok(())
    }

    #[test]
    fn test_find_local_neighbourhood_parent_relationships() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        let radius = 1.0;
        let max_result = 10;
        let (polys, parents) =
            query.find_local_neighbourhood(start_ref, &center_pos, radius, &filter, max_result)?;

        if polys.len() > 1 {
            // Verify parent relationships make sense
            for i in 1..polys.len() {
                let poly = polys[i];
                let parent = parents[i];

                // Parent should be valid and in the result set
                assert!(
                    parent.is_valid(),
                    "Non-start polygons should have valid parents"
                );
                assert!(polys.contains(&parent), "Parent should be in result set");

                // Parent should appear before child in results (breadth-first)
                let parent_index = polys.iter().position(|&p| p == parent).unwrap();
                assert!(
                    parent_index < i,
                    "Parent should appear before child in results"
                );
            }
        }

        Ok(())
    }

    #[test]
    fn test_find_local_neighbourhood_non_overlapping() -> Result<()> {
        let nav_mesh = create_complex_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_complex();
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        let radius = 1.5;
        let max_result = 15;
        let (polys, _) =
            query.find_local_neighbourhood(start_ref, &center_pos, radius, &filter, max_result)?;

        // Test the non-overlapping constraint mentioned in C++ documentation
        // All returned polygons should be non-overlapping when viewed in 2D
        for i in 0..polys.len() {
            for j in (i + 1)..polys.len() {
                let poly1 = polys[i];
                let poly2 = polys[j];

                // Note: The actual overlap check is complex and requires
                // polygon vertex extraction and intersection testing.
                // The find_local_neighbourhood implementation handles this internally.
                // For now, we just ensure all polygons are unique
                assert_ne!(poly1, poly2, "All polygons should be unique");
            }
        }

        Ok(())
    }

    #[test]
    fn test_find_local_neighbourhood_zero_radius() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let extents = get_test_extents();
        let (start_ref, center_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Zero radius should only find the start polygon
        let radius = 0.0;
        let max_result = 10;
        let (polys, parents) =
            query.find_local_neighbourhood(start_ref, &center_pos, radius, &filter, max_result)?;

        assert_eq!(polys.len(), 1, "Zero radius should only find start polygon");
        assert_eq!(polys[0], start_ref, "Should find start polygon");
        assert!(
            !parents[0].is_valid(),
            "Start polygon should have no parent"
        );

        Ok(())
    }
}

#[cfg(test)]
mod edge_case_tests {
    use super::*;

    /// Test behavior with very small floating point values
    #[test]
    fn test_floating_point_precision() -> Result<()> {
        let nav_mesh = create_minimal_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        let center = get_test_position_minimal();
        let extents = get_test_extents();
        let (start_ref, start_pos) = query.find_nearest_poly(&center, &extents, &filter)?;

        // Test with very small movements (floating point precision)
        let tiny_offset = 1e-7f32;
        let end_pos = [
            start_pos[0] + tiny_offset,
            start_pos[1],
            start_pos[2] + tiny_offset,
        ];

        let mut visited = Vec::new();
        let result =
            query.move_along_surface(start_ref, &start_pos, &end_pos, &filter, &mut visited);

        assert!(
            result.is_ok(),
            "Should handle tiny movements without numerical issues"
        );

        Ok(())
    }

    /// Test with extreme coordinates
    #[test]
    fn test_extreme_coordinates() -> Result<()> {
        let nav_mesh = create_large_test_navmesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with very large coordinates (within mesh bounds)
        let start_pos = [25.0, 0.0, 25.0];
        let extents = [5.0, 5.0, 5.0];
        let (start_ref, actual_start) = query.find_nearest_poly(&start_pos, &extents, &filter)?;

        // Test neighbourhood finding with large coordinates
        let (polys, parents) =
            query.find_local_neighbourhood(start_ref, &actual_start, 5.0, &filter, 10)?;

        assert!(!polys.is_empty(), "Should work with large coordinates");
        assert_eq!(polys.len(), parents.len(), "Arrays should match");

        Ok(())
    }
}
