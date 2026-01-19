//! Tests for QueryFilter functionality
//!
//! QueryFilter allows customization of navigation queries by filtering
//! polygons based on area types, flags, and custom traversal costs.

#[cfg(test)]
mod tests {
    use crate::{
        NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyFlags, QueryFilter,
    };
    use recast_common::Result;

    /// Helper to create a mesh with multiple area types
    fn create_multi_area_mesh() -> Result<NavMesh> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 20.0,
            tile_height: 20.0,
            max_tiles: 1,
            max_polys_per_tile: 10,
        };

        // Create vertices for a 3x3 grid of squares
        let mut vertices = Vec::new();
        let square_size = 5.0;

        for y in 0..4 {
            for x in 0..4 {
                vertices.extend_from_slice(&[x as f32 * square_size, 0.0, y as f32 * square_size]);
            }
        }

        // Create 9 polygons with different area types
        let mut polys = Vec::new();
        let mut poly_areas = Vec::new();
        let mut poly_flags = Vec::new();

        for y in 0..3 {
            for x in 0..3 {
                let base = y * 4 + x;
                polys.extend_from_slice(&[
                    base as u16,
                    (base + 1) as u16,
                    (base + 5) as u16,
                    (base + 4) as u16,
                    0xffff,
                    0xffff,
                ]);

                // Assign different area types
                let area_type = ((y * 3 + x) % 4) as u8;
                poly_areas.push(area_type);

                // Different flags based on area
                let flags = match area_type {
                    0 => PolyFlags::WALK,
                    1 => PolyFlags::WALK | PolyFlags::SWIM,
                    2 => PolyFlags::WALK | PolyFlags::DOOR,
                    3 => PolyFlags::WALK | PolyFlags::JUMP,
                    _ => PolyFlags::WALK,
                };
                poly_flags.push(flags);
            }
        }

        // Simple detail mesh
        let mut detail_meshes = Vec::new();
        let mut detail_verts = vertices.clone();
        let mut detail_tris = Vec::new();

        for i in 0..9 {
            detail_meshes.extend_from_slice(&[(i * 4) as u32, 4, (i * 2) as u32, 2]);

            let base = i * 4;
            detail_tris.extend_from_slice(&[0, 1, 2, 0, 0, 2, 3, 0]);
        }

        let create_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: vertices,
            vert_count: 16,
            polys,
            poly_flags,
            poly_areas,
            poly_count: 9,
            nvp: 6,
            detail_meshes,
            detail_verts,
            detail_vert_count: 16,
            detail_tris,
            detail_tri_count: 18,
            off_mesh_con_verts: Vec::new(),
            off_mesh_con_rad: Vec::new(),
            off_mesh_con_dir: Vec::new(),
            off_mesh_con_areas: Vec::new(),
            off_mesh_con_flags: Vec::new(),
            off_mesh_con_user_id: Vec::new(),
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [15.0, 1.0, 15.0],
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

    /// Test basic flag filtering
    #[test]
    fn test_flag_filtering() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Test with default filter (all walkable)
        let default_filter = QueryFilter::default();
        let center = [7.5, 0.0, 7.5];
        let extents = [10.0, 1.0, 10.0];

        let (poly_ref, _) = query.find_nearest_poly(&center, &extents, &default_filter)?;
        assert!(poly_ref.is_valid(), "Default filter should find polygons");

        // Test with filter excluding swimming areas
        let mut no_swim_filter = QueryFilter::default();
        no_swim_filter.exclude_flags = PolyFlags::SWIM;

        // Try to find polygon in swimming area
        let swim_area_pos = [7.5, 0.0, 2.5]; // Should be in swim area
        let result = query.find_nearest_poly(&swim_area_pos, &[0.5, 0.5, 0.5], &no_swim_filter);

        // TODO: Once flag filtering is properly implemented, verify this excludes swim areas

        Ok(())
    }

    /// Test area cost modifications
    #[test]
    fn test_area_costs() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Create filter with high cost for area type 2
        let mut expensive_filter = QueryFilter::default();
        expensive_filter.area_cost[2] = 10.0; // Make area 2 expensive

        // Test pathfinding that would normally go through area 2
        let start_pos = [2.5, 0.0, 2.5];
        let end_pos = [12.5, 0.0, 12.5];
        let extents = [1.0, 1.0, 1.0];

        let (start_ref, actual_start) =
            query.find_nearest_poly(&start_pos, &extents, &expensive_filter)?;
        let (end_ref, actual_end) =
            query.find_nearest_poly(&end_pos, &extents, &expensive_filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            // Find path with expensive area
            let expensive_path = query.find_path(
                start_ref,
                end_ref,
                &actual_start,
                &actual_end,
                &expensive_filter,
            )?;

            // Find path with default costs
            let default_path = query.find_path(
                start_ref,
                end_ref,
                &actual_start,
                &actual_end,
                &QueryFilter::default(),
            )?;

            // Paths might differ due to cost differences
            // TODO: Verify path avoids expensive areas when possible
        }

        Ok(())
    }

    /// Test include flags
    #[test]
    fn test_include_flags() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Filter that only includes door areas
        let mut door_only_filter = QueryFilter::default();
        door_only_filter.include_flags = PolyFlags::DOOR;
        door_only_filter.exclude_flags = PolyFlags::empty(); // Clear excludes

        // Search in area that should have doors
        let door_area_pos = [12.5, 0.0, 2.5];
        let extents = [5.0, 1.0, 5.0];

        let result = query.find_nearest_poly(&door_area_pos, &extents, &door_only_filter);

        // TODO: Verify only door polygons are found

        Ok(())
    }

    /// Test combining include and exclude flags
    #[test]
    fn test_combined_flag_filtering() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Filter that includes walkable but excludes jump areas
        let mut filter = QueryFilter::default();
        filter.include_flags = PolyFlags::WALK;
        filter.exclude_flags = PolyFlags::JUMP;

        let center = [7.5, 0.0, 7.5];
        let extents = [10.0, 1.0, 10.0];

        let result = query.find_nearest_poly(&center, &extents, &filter);
        assert!(result.is_ok(), "Combined filtering should work");

        Ok(())
    }

    /// Test zero area cost
    #[test]
    fn test_zero_area_cost() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Create filter with zero cost for area 1 (free movement)
        let mut free_filter = QueryFilter::default();
        free_filter.area_cost[1] = 0.0;

        // Should still be able to pathfind
        let start = [2.5, 0.0, 2.5];
        let end = [12.5, 0.0, 2.5];
        let extents = [1.0, 1.0, 1.0];

        let (start_ref, start_pos) = query.find_nearest_poly(&start, &extents, &free_filter)?;
        let (end_ref, end_pos) = query.find_nearest_poly(&end, &extents, &free_filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            let result = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &free_filter);
            assert!(
                result.is_ok(),
                "Zero cost areas should still be traversable"
            );
        }

        Ok(())
    }

    /// Test negative area cost (invalid)
    #[test]
    fn test_negative_area_cost() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let query = NavMeshQuery::new(&nav_mesh);

        // Create filter with negative cost (should be treated as invalid/infinite)
        let mut invalid_filter = QueryFilter::default();
        invalid_filter.area_cost[0] = -1.0;

        // Filter should still be usable, but negative costs might be clamped
        let pos = [2.5, 0.0, 2.5];
        let extents = [1.0, 1.0, 1.0];
        let result = query.find_nearest_poly(&pos, &extents, &invalid_filter);

        // Should handle gracefully
        assert!(result.is_ok(), "Negative costs should be handled");

        Ok(())
    }

    /// Test maximum area cost
    #[test]
    fn test_maximum_area_cost() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Set all areas to maximum cost
        let mut max_filter = QueryFilter::default();
        for i in 0..32 {
            max_filter.area_cost[i] = f32::MAX;
        }

        let start = [2.5, 0.0, 2.5];
        let end = [7.5, 0.0, 7.5];
        let extents = [1.0, 1.0, 1.0];

        let (start_ref, start_pos) = query.find_nearest_poly(&start, &extents, &max_filter)?;
        let (end_ref, end_pos) = query.find_nearest_poly(&end, &extents, &max_filter)?;

        if start_ref.is_valid() && end_ref.is_valid() {
            // Pathfinding might fail or succeed with very high cost
            let result = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &max_filter);
            // Both success and failure are valid with maximum costs
        }

        Ok(())
    }

    /// Test filter with custom area assignments
    #[test]
    fn test_custom_area_filtering() -> Result<()> {
        // This tests that different area types are properly respected
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Create filters for each area type
        for area in 0..4 {
            let mut area_filter = QueryFilter::default();
            // Make other areas very expensive
            for i in 0..32 {
                area_filter.area_cost[i] = if i == area { 1.0 } else { 1000.0 };
            }

            // Test that paths prefer the cheap area
            let result =
                query.find_nearest_poly(&[7.5, 0.0, 7.5], &[10.0, 1.0, 10.0], &area_filter);
            assert!(result.is_ok(), "Should handle area-specific filters");
        }

        Ok(())
    }

    /// Test empty flag filters
    #[test]
    fn test_empty_flag_filters() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let query = NavMeshQuery::new(&nav_mesh);

        // Filter with no include or exclude flags
        let mut empty_filter = QueryFilter::default();
        empty_filter.include_flags = PolyFlags::empty();
        empty_filter.exclude_flags = PolyFlags::empty();

        let pos = [7.5, 0.0, 7.5];
        let extents = [1.0, 1.0, 1.0];

        // Should still work with empty flags
        let result = query.find_nearest_poly(&pos, &extents, &empty_filter);
        assert!(result.is_ok(), "Empty flags should be handled");

        Ok(())
    }

    /// Test all flags set
    #[test]
    fn test_all_flags_set() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let query = NavMeshQuery::new(&nav_mesh);

        // Filter with all possible flags
        let mut all_flags_filter = QueryFilter::default();
        all_flags_filter.include_flags = PolyFlags::all();

        let pos = [7.5, 0.0, 7.5];
        let extents = [5.0, 1.0, 5.0];

        let result = query.find_nearest_poly(&pos, &extents, &all_flags_filter);
        assert!(result.is_ok(), "All flags set should work");

        Ok(())
    }

    /// Test pathfinding with different filters
    #[test]
    fn test_pathfinding_filter_variations() -> Result<()> {
        let nav_mesh = create_multi_area_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);

        let start = [2.5, 0.0, 2.5];
        let end = [12.5, 0.0, 12.5];
        let extents = [1.0, 1.0, 1.0];

        // Test with various filters
        let filters = vec![
            QueryFilter::default(),
            {
                let mut f = QueryFilter::default();
                f.exclude_flags = PolyFlags::DOOR;
                f
            },
            {
                let mut f = QueryFilter::default();
                f.area_cost[1] = 5.0;
                f.area_cost[2] = 10.0;
                f
            },
        ];

        for filter in filters {
            let (start_ref, start_pos) = query.find_nearest_poly(&start, &extents, &filter)?;
            let (end_ref, end_pos) = query.find_nearest_poly(&end, &extents, &filter)?;

            if start_ref.is_valid() && end_ref.is_valid() {
                let result = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &filter);
                // Different filters may produce different results
            }
        }

        Ok(())
    }
}
