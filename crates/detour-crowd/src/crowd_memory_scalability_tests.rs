//! Memory usage and scalability tests for Detour Crowd simulation
//!
//! These tests verify that the crowd simulation:
//! - Scales well with increasing agent counts
//! - Manages memory efficiently
//! - Handles stress scenarios

#[cfg(test)]
mod tests {
    use crate::{AgentParams, Crowd, GridAgent, ProximityGrid, RVOConfig, UpdateFlags};
    use detour::{
        NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyFlags, QueryFilter,
    };
    use recast_common::Result;
    use std::mem;

    /// Helper to create a simple square navmesh for crowd testing
    fn create_test_navmesh(size: f32) -> Result<NavMesh> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: size,
            tile_height: size,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };

        let vertices = vec![
            0.0, 0.0, 0.0, size, 0.0, 0.0, size, 0.0, size, 0.0, 0.0, size,
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
            detail_verts: vertices,
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
            bmax: [size, 1.0, size],
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

    /// Test crowd scaling with increasing agent counts
    #[test]
    fn test_crowd_agent_scaling() -> Result<()> {
        let nav_mesh = create_test_navmesh(100.0)?;
        let agent_counts = vec![10, 50, 100, 500, 1000];

        for count in agent_counts {
            let mut crowd = Crowd::new(&nav_mesh, count, 2.0);

            let params = AgentParams {
                radius: 0.6,
                height: 2.0,
                max_acceleration: 8.0,
                max_speed: 3.5,
                collision_query_range: 12.0,
                path_optimization_range: 30.0,
                separate: true,
                update_flags: UpdateFlags::default(),
                obstacle_avoidance_type: 3,
                query_filter_type: 0,
                user_data: None,
                use_rvo: true,
                rvo_config: RVOConfig::default(),
            };

            // Add agents in a grid pattern
            let grid_size = (count as f32).sqrt().ceil() as usize;
            let spacing = 90.0 / grid_size as f32;

            let mut added_count = 0;
            for i in 0..count {
                let x = (i % grid_size) as f32 * spacing + 5.0;
                let z = (i / grid_size) as f32 * spacing + 5.0;
                let pos = [x, 0.0, z];

                if crowd.add_agent(pos, params.clone()).is_ok() {
                    added_count += 1;
                }
            }

            println!(
                "Agent count {}: added {} agents ({}% success)",
                count,
                added_count,
                (added_count * 100) / count
            );

            // Verify most agents were added successfully
            assert!(
                added_count >= count * 90 / 100,
                "Should add at least 90% of requested agents"
            );

            // Test update performance
            let start = std::time::Instant::now();
            crowd.update(0.033)?;
            let elapsed = start.elapsed();

            println!("Update with {} agents took {:?}", added_count, elapsed);

            // Basic memory size check
            let crowd_size = mem::size_of_val(&crowd);
            println!("Crowd struct size: {} bytes", crowd_size);
        }

        Ok(())
    }

    /// Test proximity grid scalability
    #[test]
    fn test_proximity_grid_scaling() -> Result<()> {
        let cell_sizes = vec![1.0, 2.0, 5.0, 10.0];
        let agent_counts = vec![100, 500, 1000, 5000];

        for cell_size in cell_sizes {
            for &agent_count in &agent_counts {
                let mut grid = ProximityGrid::new(cell_size);

                // Add agents randomly distributed
                for i in 0..agent_count {
                    let x = (i * 7 % 100) as f32;
                    let z = (i * 11 % 100) as f32;

                    let agent = GridAgent {
                        id: i,
                        pos: [x, 0.0, z],
                        radius: 0.6,
                    };

                    grid.update_agent(agent);
                }

                // Test query performance
                let start = std::time::Instant::now();
                let mut total_neighbors = 0;

                for _ in 0..100 {
                    let query_pos = [50.0, 0.0, 50.0];
                    let neighbors = grid.query_agents(query_pos, 10.0);
                    total_neighbors += neighbors.len();
                }

                let elapsed = start.elapsed();

                println!(
                    "Grid (cell_size={}, agents={}): 100 queries took {:?}, avg neighbors: {}",
                    cell_size,
                    agent_count,
                    elapsed,
                    total_neighbors / 100
                );
            }
        }

        Ok(())
    }

    /// Test agent add/remove cycles
    #[test]
    fn test_agent_lifecycle_memory() -> Result<()> {
        let nav_mesh = create_test_navmesh(50.0)?;
        let mut crowd = Crowd::new(&nav_mesh, 100, 2.0);

        let params = AgentParams {
            radius: 0.6,
            height: 2.0,
            max_acceleration: 8.0,
            max_speed: 3.5,
            collision_query_range: 12.0,
            path_optimization_range: 30.0,
            separate: true,
            update_flags: UpdateFlags::default(),
            obstacle_avoidance_type: 3,
            query_filter_type: 0,
            user_data: None,
            use_rvo: true,
            rvo_config: RVOConfig::default(),
        };

        // Perform many add/remove cycles
        for cycle in 0..100 {
            let mut agent_ids = Vec::new();

            // Add 10 agents
            for i in 0..10 {
                let pos = [
                    (i * 5) as f32 % 45.0 + 2.5,
                    0.0,
                    (cycle * 3 + i * 7) as f32 % 45.0 + 2.5,
                ];

                if let Ok(id) = crowd.add_agent(pos, params.clone()) {
                    agent_ids.push(id);
                }
            }

            // Update a few times
            for _ in 0..5 {
                crowd.update(0.033)?;
            }

            // Remove all agents
            for id in agent_ids {
                crowd.remove_agent(id)?;
            }
        }

        // Verify no memory leaks by checking agent count
        let active_count = crowd.get_active_agent_count();
        assert_eq!(active_count, 0, "All agents should be removed after cycles");

        Ok(())
    }

    /// Test maximum agent configuration
    ///
    /// This test is ignored by default because it depends on CI machine
    /// performance. Run manually with: cargo test -- --ignored
    #[test]
    #[ignore = "Performance test - too variable for CI"]
    fn test_maximum_agent_configuration() -> Result<()> {
        let nav_mesh = create_test_navmesh(200.0)?;
        let max_agents = 5000;
        let mut crowd = Crowd::new(&nav_mesh, max_agents, 3.0);

        // Create diverse agent parameters
        let param_variants = vec![
            AgentParams {
                radius: 0.4,
                height: 1.8,
                max_acceleration: 10.0,
                max_speed: 5.0,
                collision_query_range: 10.0,
                path_optimization_range: 20.0,
                separate: true,
                update_flags: UpdateFlags::default(),
                obstacle_avoidance_type: 3,
                query_filter_type: 0,
                user_data: None,
                use_rvo: true,
                rvo_config: RVOConfig::default(),
            },
            AgentParams {
                radius: 0.8,
                height: 2.5,
                max_acceleration: 5.0,
                max_speed: 2.0,
                collision_query_range: 15.0,
                path_optimization_range: 30.0,
                separate: true,
                update_flags: UpdateFlags::default(),
                obstacle_avoidance_type: 2,
                query_filter_type: 0,
                user_data: None,
                use_rvo: false,
                rvo_config: RVOConfig::default(),
            },
            AgentParams {
                radius: 1.2,
                height: 3.0,
                max_acceleration: 3.0,
                max_speed: 1.5,
                collision_query_range: 20.0,
                path_optimization_range: 40.0,
                separate: false,
                update_flags: UpdateFlags::default(),
                obstacle_avoidance_type: 1,
                query_filter_type: 0,
                user_data: None,
                use_rvo: true,
                rvo_config: RVOConfig::default(),
            },
        ];

        // Add as many agents as possible
        let mut added_count = 0;
        let grid_size = (max_agents as f32).sqrt().ceil() as usize;
        let spacing = 195.0 / grid_size as f32;

        for i in 0..max_agents {
            let x = (i % grid_size) as f32 * spacing + 2.5;
            let z = (i / grid_size) as f32 * spacing + 2.5;
            let pos = [x, 0.0, z];

            let params = &param_variants[i % param_variants.len()];

            if crowd.add_agent(pos, params.clone()).is_ok() {
                added_count += 1;
            } else {
                break;
            }
        }

        println!(
            "Maximum configuration: added {} agents out of {} requested",
            added_count, max_agents
        );

        // Set random targets for agents
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        let ext = [5.0, 5.0, 5.0];

        for i in 0..added_count.min(100) {
            let target = [
                (i * 17 % 190) as f32 + 5.0,
                0.0,
                (i * 23 % 190) as f32 + 5.0,
            ];

            if let Ok((poly_ref, closest)) = query.find_nearest_poly(&target, &ext, &filter) {
                crowd.request_move_target(i, poly_ref, closest).ok();
            }
        }

        // Measure update performance
        let start = std::time::Instant::now();
        let update_count = 10;

        for _ in 0..update_count {
            crowd.update(0.033)?;
        }

        let elapsed = start.elapsed();
        let avg_update = elapsed / update_count;

        println!(
            "Average update time with {} agents: {:?}",
            added_count, avg_update
        );

        // Verify update time is reasonable (less than 200ms per update for 5000 agents)
        assert!(
            avg_update.as_millis() < 200,
            "Update time should be reasonable even with many agents"
        );

        Ok(())
    }

    /// Test crowd state persistence
    #[test]
    fn test_crowd_state_memory() -> Result<()> {
        let nav_mesh = create_test_navmesh(50.0)?;
        let mut crowd = Crowd::new(&nav_mesh, 50, 2.0);

        let params = AgentParams {
            radius: 0.6,
            height: 2.0,
            max_acceleration: 8.0,
            max_speed: 3.5,
            collision_query_range: 12.0,
            path_optimization_range: 30.0,
            separate: true,
            update_flags: UpdateFlags::default(),
            obstacle_avoidance_type: 3,
            query_filter_type: 0,
            user_data: Some(42),
            use_rvo: true,
            rvo_config: RVOConfig::default(),
        };

        // Add agents and set various states
        let mut agent_ids = Vec::new();
        for i in 0..20 {
            let pos = [(i * 2) as f32 + 5.0, 0.0, (i * 3) as f32 % 45.0 + 2.5];
            if let Ok(id) = crowd.add_agent(pos, params.clone()) {
                agent_ids.push(id);
            }
        }

        // Set different targets for agents
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        let ext = [2.0, 2.0, 2.0];

        for (i, &id) in agent_ids.iter().enumerate() {
            let target = [45.0 - (i as f32 * 2.0), 0.0, 45.0 - (i as f32 * 1.5)];
            if let Ok((poly_ref, closest)) = query.find_nearest_poly(&target, &ext, &filter) {
                crowd.request_move_target(id, poly_ref, closest)?;
            }
        }

        // Update multiple times and check state persistence
        for update in 0..50 {
            crowd.update(0.033)?;

            // Periodically check agent states
            if update % 10 == 0 {
                let active_count = crowd.get_active_agent_count();
                println!("Update {}: {} active agents", update, active_count);

                // Verify agents still exist
                for &id in &agent_ids {
                    assert!(
                        crowd.get_agent(id).is_some(),
                        "Agent {} should still exist",
                        id
                    );
                }
            }
        }

        Ok(())
    }

    /// Test proximity grid memory with dense agent clusters
    #[test]
    fn test_proximity_grid_dense_clusters() -> Result<()> {
        let mut grid = ProximityGrid::new(2.0);

        // Create dense clusters of agents
        let clusters = vec![
            (25.0, 25.0, 500), // 500 agents at (25, 25)
            (75.0, 25.0, 300), // 300 agents at (75, 25)
            (25.0, 75.0, 400), // 400 agents at (25, 75)
            (75.0, 75.0, 200), // 200 agents at (75, 75)
        ];

        let mut agent_id = 0;
        for (cx, cz, count) in clusters {
            for i in 0..count {
                // Place agents in a tight cluster with some randomness
                let angle = (i as f32 * 2.0 * std::f32::consts::PI) / count as f32;
                let radius = (i % 10) as f32 * 0.5;
                let x = cx + angle.cos() * radius;
                let z = cz + angle.sin() * radius;

                let agent = GridAgent {
                    id: agent_id,
                    pos: [x, 0.0, z],
                    radius: 0.6,
                };

                grid.update_agent(agent);
                agent_id += 1;
            }
        }

        println!("Added {} agents in 4 dense clusters", agent_id);

        // Test queries in different areas
        let query_positions = vec![
            [25.0, 0.0, 25.0],   // Center of cluster 1
            [75.0, 0.0, 75.0],   // Center of cluster 4
            [50.0, 0.0, 50.0],   // Between clusters
            [0.0, 0.0, 0.0],     // Empty area
            [100.0, 0.0, 100.0], // Outside clusters
        ];

        for pos in query_positions {
            let neighbors = grid.query_agents(pos, 10.0);
            println!("Query at {:?}: found {} neighbors", pos, neighbors.len());
        }

        Ok(())
    }

    /// Stress test with rapid agent state changes
    #[test]
    #[ignore] // This test is expensive, run with --ignored
    fn test_stress_rapid_state_changes() -> Result<()> {
        let nav_mesh = create_test_navmesh(100.0)?;
        let mut crowd = Crowd::new(&nav_mesh, 200, 2.0);

        let params = AgentParams {
            radius: 0.5,
            height: 2.0,
            max_acceleration: 15.0,
            max_speed: 6.0,
            collision_query_range: 8.0,
            path_optimization_range: 15.0,
            separate: true,
            update_flags: UpdateFlags::default(),
            obstacle_avoidance_type: 3,
            query_filter_type: 0,
            user_data: None,
            use_rvo: true,
            rvo_config: RVOConfig::default(),
        };

        // Add agents
        let mut agent_ids = Vec::new();
        for i in 0..150 {
            let pos = [
                (i % 10) as f32 * 9.0 + 5.0,
                0.0,
                (i / 10) as f32 * 6.0 + 5.0,
            ];
            if let Ok(id) = crowd.add_agent(pos, params.clone()) {
                agent_ids.push(id);
            }
        }

        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        let ext = [5.0, 5.0, 5.0];

        // Rapid state changes
        let start = std::time::Instant::now();
        let iterations = 1000;

        for iter in 0..iterations {
            // Update crowd
            crowd.update(0.016)?; // 60 FPS

            // Change random agent targets
            if iter % 5 == 0 {
                for &id in agent_ids.iter().skip(iter % 10).step_by(10) {
                    let target = [
                        (iter * 7 % 90) as f32 + 5.0,
                        0.0,
                        (iter * 11 % 90) as f32 + 5.0,
                    ];

                    if let Ok((poly_ref, closest)) = query.find_nearest_poly(&target, &ext, &filter)
                    {
                        crowd.request_move_target(id, poly_ref, closest).ok();
                    }
                }
            }

            // Occasionally change some agents' targets to a nearby location
            if iter % 20 == 0 {
                for &id in agent_ids.iter().skip(iter % 5).step_by(20) {
                    // Set a stationary target to effectively stop the agent
                    let stop_pos = [
                        45.0, 0.0, 45.0, // Center of the navmesh
                    ];
                    if let Ok((poly_ref, closest)) =
                        query.find_nearest_poly(&stop_pos, &ext, &filter)
                    {
                        crowd.request_move_target(id, poly_ref, closest).ok();
                    }
                }
            }
        }

        let elapsed = start.elapsed();
        let avg_update = elapsed / iterations as u32;

        println!(
            "Stress test: {} iterations in {:?}, avg {:?}/update",
            iterations, elapsed, avg_update
        );

        // Verify performance is acceptable
        assert!(
            avg_update.as_micros() < 20000, // Less than 20ms per update
            "Rapid state changes should maintain good performance"
        );

        Ok(())
    }
}
