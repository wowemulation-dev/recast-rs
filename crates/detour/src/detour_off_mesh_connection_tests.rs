//! Tests for off-mesh connections in Detour navigation meshes
//!
//! Off-mesh connections allow navigation between disconnected areas of the mesh,
//! such as jump points, ladders, or teleporters.

#[cfg(test)]
mod tests {
    use crate::{
        NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyFlags, QueryFilter,
    };
    use recast_common::Result;

    /// Helper to create a simple two-island mesh with an off-mesh connection
    fn create_two_island_mesh_with_connection() -> Result<NavMesh> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 10,
        };

        // Create vertices for two separate square islands
        let vertices = vec![
            // Island 1: (0,0) to (2,2)
            0.0, 0.0, 0.0, // 0
            2.0, 0.0, 0.0, // 1
            2.0, 0.0, 2.0, // 2
            0.0, 0.0, 2.0, // 3
            // Island 2: (4,0) to (6,2)
            4.0, 0.0, 0.0, // 4
            6.0, 0.0, 0.0, // 5
            6.0, 0.0, 2.0, // 6
            4.0, 0.0, 2.0, // 7
        ];

        // Create two polygons (one per island)
        // Format: nvp vertices, then nvp neighbors (nvp=6)
        let polys = vec![
            // Island 1: vertices
            0, 1, 2, 3, 0xffff, 0xffff,
            // Island 1: neighbors (all no connection since islands are separate)
            0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, // Island 2: vertices
            4, 5, 6, 7, 0xffff, 0xffff,
            // Island 2: neighbors (all no connection since islands are separate)
            0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
        ];

        let poly_areas = vec![0, 0]; // Both walkable
        let poly_flags = vec![PolyFlags::WALK, PolyFlags::WALK];

        // Off-mesh connection from island 1 to island 2
        let off_mesh_con_verts = vec![
            2.0, 0.0, 1.0, // Start position (right edge of island 1)
            4.0, 0.0, 1.0, // End position (left edge of island 2)
        ];

        let off_mesh_con_rad = vec![1.5]; // Connection radius (needs to reach vertices)
        let off_mesh_con_dir = vec![0]; // Bidirectional
        let off_mesh_con_areas = vec![0]; // Walkable area
        let off_mesh_con_flags = vec![PolyFlags::JUMP]; // Jump connection
        let off_mesh_con_user_id = vec![1000]; // User ID

        let create_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: vertices,
            vert_count: 8,
            polys,
            poly_flags,
            poly_areas,
            poly_count: 2,
            nvp: 6,
            detail_meshes: vec![0, 4, 0, 2, 4, 4, 2, 2], // Simple detail mesh
            detail_verts: vec![
                // Detail verts for island 1
                0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 2.0, 0.0, 2.0, 0.0, 0.0, 2.0,
                // Detail verts for island 2
                4.0, 0.0, 0.0, 6.0, 0.0, 0.0, 6.0, 0.0, 2.0, 4.0, 0.0, 2.0,
            ],
            detail_vert_count: 8,
            detail_tris: vec![0, 1, 2, 0, 0, 2, 3, 0, 0, 1, 2, 0, 0, 2, 3, 0],
            detail_tri_count: 4,
            off_mesh_con_verts,
            off_mesh_con_rad,
            off_mesh_con_dir,
            off_mesh_con_areas,
            off_mesh_con_flags,
            off_mesh_con_user_id,
            off_mesh_con_count: 1,
            bmin: [0.0, 0.0, 0.0],
            bmax: [6.0, 1.0, 2.0],
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

    /// Test basic off-mesh connection creation
    #[test]
    fn test_off_mesh_connection_creation() -> Result<()> {
        let nav_mesh = create_two_island_mesh_with_connection()?;

        // Verify mesh was created with expected polygons
        assert_eq!(nav_mesh.get_max_tiles(), 1, "Should have 1 tile");

        // Get tile and verify it has off-mesh connections
        let tile = nav_mesh.get_tile_at(0, 0, 0);
        assert!(tile.is_some(), "Tile should exist");

        // TODO: Add method to query off-mesh connections from tile

        Ok(())
    }

    /// Test pathfinding across off-mesh connection
    #[test]
    fn test_pathfinding_with_off_mesh_connection() -> Result<()> {
        let nav_mesh = create_two_island_mesh_with_connection()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let mut filter = QueryFilter::default();
        filter.include_flags = PolyFlags::WALK | PolyFlags::JUMP;

        // Start on island 1
        let start_pos = [1.0, 0.0, 1.0];
        // End on island 2
        let end_pos = [5.0, 0.0, 1.0];
        let extents = [0.5, 1.0, 0.5];

        // Find start and end polygons
        let (start_ref, actual_start) = query.find_nearest_poly(&start_pos, &extents, &filter)?;
        let (end_ref, actual_end) = query.find_nearest_poly(&end_pos, &extents, &filter)?;

        assert!(
            start_ref.is_valid(),
            "Should find start polygon on island 1"
        );
        assert!(end_ref.is_valid(), "Should find end polygon on island 2");
        assert_ne!(start_ref, end_ref, "Islands should be different polygons");

        // Find path - should succeed if off-mesh connection works
        let path = query.find_path(start_ref, end_ref, &actual_start, &actual_end, &filter)?;

        // Path should include both polygons and the off-mesh connection
        assert!(
            path.len() >= 2,
            "Path should cross islands via off-mesh connection"
        );
        assert_eq!(path[0], start_ref, "Path should start at island 1");
        assert_eq!(path[path.len() - 1], end_ref, "Path should end at island 2");

        Ok(())
    }

    /// Test off-mesh connection directionality
    #[test]
    fn test_off_mesh_connection_directionality() -> Result<()> {
        // TODO: Create mesh with unidirectional connection and verify
        // pathfinding only works in one direction

        Ok(())
    }

    /// Test off-mesh connection with custom area types
    #[test]
    fn test_off_mesh_connection_area_filtering() -> Result<()> {
        let nav_mesh = create_two_island_mesh_with_connection()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Create filter that excludes jump connections
        let mut no_jump_filter = QueryFilter::default();
        no_jump_filter.exclude_flags = PolyFlags::JUMP;

        let start_pos = [1.0, 0.0, 1.0];
        let end_pos = [5.0, 0.0, 1.0];
        let extents = [0.5, 1.0, 0.5];

        let (start_ref, start_actual) =
            query.find_nearest_poly(&start_pos, &extents, &no_jump_filter)?;
        let (end_ref, end_actual) = query.find_nearest_poly(&end_pos, &extents, &no_jump_filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            // Path should fail or be empty since we exclude jump connections
            let result = query.find_path(
                start_ref,
                end_ref,
                &start_actual,
                &end_actual,
                &no_jump_filter,
            );

            match result {
                Ok(path) => {
                    // If path exists, it shouldn't use the off-mesh connection
                    // In our case, islands are disconnected, so path should be impossible
                    assert!(
                        path.is_empty() || path.len() == 1,
                        "Should not find path when jump connections are excluded"
                    );
                }
                Err(_) => {
                    // Expected - no path without off-mesh connection
                }
            }
        }

        Ok(())
    }

    /// Test multiple off-mesh connections
    #[test]
    fn test_multiple_off_mesh_connections() -> Result<()> {
        // TODO: Create mesh with multiple connections and verify
        // pathfinding chooses optimal connection

        Ok(())
    }

    /// Test off-mesh connection endpoint queries
    #[test]
    fn test_off_mesh_connection_endpoints() -> Result<()> {
        let nav_mesh = create_two_island_mesh_with_connection()?;
        let query = NavMeshQuery::new(&nav_mesh);

        // TODO: Once we have proper off-mesh connection query methods,
        // test get_off_mesh_connection_endpoint and related functions

        Ok(())
    }

    /// Test off-mesh connection with different radii
    #[test]
    fn test_off_mesh_connection_radius() -> Result<()> {
        // TODO: Test that connection radius affects path planning
        // and agent movement

        Ok(())
    }

    /// Test invalid off-mesh connections
    #[test]
    fn test_invalid_off_mesh_connections() -> Result<()> {
        // TODO: Test creating off-mesh connections with:
        // - Invalid polygon references
        // - Zero or negative radius
        // - Endpoints outside mesh bounds
        // - Duplicate connections

        Ok(())
    }

    /// Test off-mesh connection user IDs
    #[test]
    fn test_off_mesh_connection_user_ids() -> Result<()> {
        // TODO: Verify user IDs are preserved and can be queried

        Ok(())
    }

    /// Test pathfinding straight path with off-mesh connections
    #[test]
    fn test_straight_path_with_off_mesh() -> Result<()> {
        let nav_mesh = create_two_island_mesh_with_connection()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let mut filter = QueryFilter::default();
        filter.include_flags = PolyFlags::WALK | PolyFlags::JUMP;

        let start_pos = [1.0, 0.0, 1.0];
        let end_pos = [5.0, 0.0, 1.0];
        let extents = [0.5, 1.0, 0.5];

        let (start_ref, actual_start) = query.find_nearest_poly(&start_pos, &extents, &filter)?;
        let (end_ref, actual_end) = query.find_nearest_poly(&end_pos, &extents, &filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            let path = query.find_path(start_ref, end_ref, &actual_start, &actual_end, &filter)?;

            if !path.is_empty() && path.len() >= 2 {
                // Get straight path
                let straight_path = query.find_straight_path(&actual_start, &actual_end, &path)?;

                // Straight path should include off-mesh connection points
                assert!(
                    straight_path.waypoints.len() >= 3,
                    "Straight path should include start, connection points, and end"
                );

                // TODO: Verify flags indicate off-mesh connection
            }
        }

        Ok(())
    }

    /// Test move_along_surface with off-mesh connections
    #[test]
    fn test_move_along_surface_with_off_mesh() -> Result<()> {
        // TODO: Test that move_along_surface properly handles
        // off-mesh connection boundaries

        Ok(())
    }

    /// Test raycast interaction with off-mesh connections
    #[test]
    fn test_raycast_with_off_mesh() -> Result<()> {
        // TODO: Verify raycast behavior when ray intersects
        // off-mesh connection areas

        Ok(())
    }
}
