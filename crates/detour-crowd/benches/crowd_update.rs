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
    let mut config = RecastConfig::default();
    config.cs = 0.3;
    config.ch = 0.2;
    config.walkable_slope_angle = 45.0;
    config.walkable_height = 2;
    config.walkable_climb = 1;
    config.walkable_radius = 1;
    config.max_edge_len = 12;
    config.max_simplification_error = 1.3;
    config.min_region_area = 8;
    config.merge_region_area = 20;
    config.max_vertices_per_polygon = 6;
    config.detail_sample_dist = 6.0;
    config.detail_sample_max_error = 1.0;
    config.calculate_grid_size(bmin, bmax);

    let builder = RecastBuilder::new(config);
    let (poly_mesh, detail_mesh) = builder
        .build_mesh(&mesh.vertices, &mesh.indices)
        .expect("failed to build mesh");

    let params = {
        let mut p = NavMeshParams::default();
        p.origin = [bmin.x, bmin.y, bmin.z];
        p.tile_width = bmax.x - bmin.x;
        p.tile_height = bmax.z - bmin.z;
        p.max_tiles = 1;
        p.max_polys_per_tile = poly_mesh.poly_count() as i32;
        p
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
    let extent = [5.0, 10.0, 5.0];

    let params = {
        let mut p = AgentParams::default();
        p.radius = 0.6;
        p.height = 2.0;
        p.max_acceleration = 8.0;
        p.max_speed = 3.5;
        p
    };

    // Place agents in a grid pattern and give each a target
    for i in 0..num_agents {
        let row = i / 10;
        let col = i % 10;
        let x = 5.0 + col as f32 * 2.0;
        let z = -10.0 + row as f32 * 2.0;
        let pos = [x, 0.0, z];

        if let Ok((_, snapped_pos)) = query.find_nearest_poly(&pos, &extent, &filter) {
            if let Ok(agent_id) = crowd.add_agent(Vec3::from(snapped_pos), params.clone()) {
                // Target: opposite side of the mesh
                let target = [50.0 - x, 0.0, -z];
                if let Ok((target_ref, snapped_target)) =
                    query.find_nearest_poly(&target, &extent, &filter)
                {
                    let _ =
                        crowd.request_move_target(agent_id, target_ref, Vec3::from(snapped_target));
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
