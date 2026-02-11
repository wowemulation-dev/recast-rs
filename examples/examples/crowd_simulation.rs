//! Crowd simulation example.
//!
//! Demonstrates multi-agent crowd simulation with RVO collision avoidance.
//! Adds 5 agents at different positions, sets targets, and runs a 100-frame
//! update loop printing agent positions periodically.
//!
//! Run with:
//! ```sh
//! cargo run -p recast-rs-examples --example crowd_simulation
//! ```

use detour::{NavMeshQuery, QueryFilter};
use detour_crowd::{AgentParams, Crowd};
use glam::Vec3;

use recast_rs_examples::common;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (nav_mesh, _poly_mesh, _detail_mesh) =
        common::build_navmesh_from_obj("examples/data/nav_test.obj")?;

    // First, snap candidate positions to the navmesh so agents start on valid polygons
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let candidate_positions = [
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
        [0.0, 0.0, 5.0],
        [5.0, 0.0, 5.0],
        [2.5, 0.0, 2.5],
    ];

    let mut snapped_positions = Vec::new();
    for pos in &candidate_positions {
        let (_poly_ref, snapped) = query.find_nearest_poly(pos, &extent, &filter)?;
        snapped_positions.push(snapped);
    }

    // Create crowd manager
    let max_agents = 10;
    let max_agent_radius = 0.6;
    let mut crowd = Crowd::new(&nav_mesh, max_agents, max_agent_radius);

    // Enable RVO collision avoidance
    crowd.enable_rvo(None);

    // Agent parameters
    let params = {
        let mut p = AgentParams::default();
        p.radius = 0.6;
        p.height = 2.0;
        p.max_speed = 3.5;
        p.max_acceleration = 8.0;
        p
    };

    // Add 5 agents at snapped positions
    let mut agent_ids = Vec::new();
    for pos in &snapped_positions {
        let id = crowd.add_agent(Vec3::from(*pos), params.clone())?;
        agent_ids.push(id);
        println!(
            "Added agent {} at ({:.2}, {:.2}, {:.2})",
            id, pos[0], pos[1], pos[2]
        );
    }

    println!();
    println!("Active agents: {}", crowd.get_active_agent_count());

    // Set a shared target
    let target_pos = [20.0, 0.0, 10.0];
    let (target_ref, snapped_target) = query.find_nearest_poly(&target_pos, &extent, &filter)?;

    println!(
        "Target: ({:.1}, {:.1}, {:.1}) -> snapped to ({:.2}, {:.2}, {:.2})",
        target_pos[0],
        target_pos[1],
        target_pos[2],
        snapped_target[0],
        snapped_target[1],
        snapped_target[2]
    );

    for &agent_id in &agent_ids {
        crowd.request_move_target(agent_id, target_ref, Vec3::from(snapped_target))?;
    }

    // Run simulation: 100 frames at 60 FPS
    let dt = 1.0 / 60.0;
    let total_frames = 100;

    println!();
    println!("Running {} frames at dt={:.4}s...", total_frames, dt);
    println!();

    for frame in 0..total_frames {
        crowd.update(dt)?;

        // Print every 25 frames
        if frame % 25 == 0 || frame == total_frames - 1 {
            println!("Frame {}:", frame);
            for &agent_id in &agent_ids {
                if let Some(agent) = crowd.get_agent(agent_id) {
                    let pos = agent.get_pos();
                    let vel = agent.get_vel();
                    let speed = (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]).sqrt();
                    println!(
                        "  Agent {}: pos=({:.2}, {:.2}, {:.2}) speed={:.2}",
                        agent_id, pos[0], pos[1], pos[2], speed
                    );
                }
            }
        }
    }

    // Print final distances to target
    println!();
    println!("Final distances to target:");
    for &agent_id in &agent_ids {
        if let Some(agent) = crowd.get_agent(agent_id) {
            let pos = agent.get_pos();
            let dx = pos[0] - snapped_target[0];
            let dy = pos[1] - snapped_target[1];
            let dz = pos[2] - snapped_target[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            println!("  Agent {}: {:.2}", agent_id, dist);
        }
    }

    Ok(())
}
