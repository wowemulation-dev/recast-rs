//! Benchmarks for crowd simulation updates.
//!
//! Measures `Crowd::update` with varying agent counts (10, 50, 100)
//! and with RVO collision avoidance enabled.

use criterion::{Criterion, criterion_group, criterion_main};
use detour::{NavMesh, NavMeshFlags, NavMeshParams, NavMeshQuery, QueryFilter};
use detour_crowd::{AgentParams, Crowd};
use glam::Vec3;
use recast::{RecastBuilder, RecastConfig};
use recast_common::TriMesh;

const TEST_DATA: &str = "../../test-data/meshes";

fn build_navmesh() -> NavMesh {
    let path = format!("{TEST_DATA}/nav_test.obj");
    let mesh = TriMesh::from_obj(&path).expect("failed to load test mesh");
    let (bmin, bmax) = mesh.calculate_bounds();
    let mut config = RecastConfig {
        cs: 0.3,
        ch: 0.2,
        walkable_slope_angle: 45.0,
        walkable_height: 2,
        walkable_climb: 1,
        walkable_radius: 1,
        max_edge_len: 12,
        max_simplification_error: 1.3,
        min_region_area: 8,
        merge_region_area: 20,
        max_vertices_per_polygon: 6,
        detail_sample_dist: 6.0,
        detail_sample_max_error: 1.0,
        ..Default::default()
    };
    config.calculate_grid_size(bmin, bmax);

    let builder = RecastBuilder::new(config);
    let (poly_mesh, detail_mesh) = builder
        .build_mesh(&mesh.vertices, &mesh.indices)
        .expect("failed to build mesh");

    let params = NavMeshParams {
        origin: [bmin.x, bmin.y, bmin.z],
        tile_width: bmax.x - bmin.x,
        tile_height: bmax.z - bmin.z,
        max_tiles: 1,
        max_polys_per_tile: poly_mesh.npolys as i32,
    };

    NavMesh::build_from_recast(params, &poly_mesh, &detail_mesh, NavMeshFlags::empty())
        .expect("failed to build navmesh")
}

/// Set up a crowd with the given number of agents, each with a target.
fn setup_crowd(nav_mesh: &NavMesh, num_agents: usize, enable_rvo: bool) -> Crowd<'_> {
    let mut crowd = Crowd::new(nav_mesh, num_agents + 10, 0.6);

    if enable_rvo {
        crowd.enable_rvo(None);
    }

    let query = NavMeshQuery::new(nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let params = AgentParams {
        radius: 0.6,
        height: 2.0,
        max_acceleration: 8.0,
        max_speed: 3.5,
        ..Default::default()
    };

    // Place agents in a grid pattern and give each a target
    for i in 0..num_agents {
        let row = i / 10;
        let col = i % 10;
        let x = 5.0 + col as f32 * 2.0;
        let z = -10.0 + row as f32 * 2.0;
        let pos = Vec3::new(x, 0.0, z);

        if let Ok((_, snapped_pos)) = query.find_nearest_poly(pos, extent, &filter) {
            if let Ok(agent_id) = crowd.add_agent(snapped_pos, params.clone()) {
                // Target: opposite side of the mesh
                let target = Vec3::new(50.0 - x, 0.0, -z);
                if let Ok((target_ref, snapped_target)) =
                    query.find_nearest_poly(target, extent, &filter)
                {
                    let _ = crowd.request_move_target(agent_id, target_ref, snapped_target);
                }
            }
        }
    }

    // Run a few warm-up frames so agents have active corridors
    for _ in 0..5 {
        let _ = crowd.update(1.0 / 60.0);
    }

    crowd
}

fn bench_crowd_update(c: &mut Criterion) {
    let mut group = c.benchmark_group("crowd_update");

    let nav_mesh = build_navmesh();

    for count in [10, 50, 100] {
        group.bench_function(format!("{count}_agents"), |b| {
            let mut crowd = setup_crowd(&nav_mesh, count, false);
            b.iter(|| crowd.update(1.0 / 60.0).unwrap());
        });
    }

    group.bench_function("100_agents_rvo", |b| {
        let mut crowd = setup_crowd(&nav_mesh, 100, true);
        b.iter(|| crowd.update(1.0 / 60.0).unwrap());
    });

    group.finish();
}

criterion_group!(benches, bench_crowd_update);
criterion_main!(benches);
