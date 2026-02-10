//! Benchmarks for spatial queries on navigation meshes.
//!
//! Measures find_nearest_poly, raycast, find_distance_to_wall,
//! move_along_surface, and find_polys_around_circle using nav_test.obj.

use criterion::{Criterion, criterion_group, criterion_main};
use detour::{NavMesh, NavMeshFlags, NavMeshParams, NavMeshQuery, PolyRef, QueryFilter};
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

/// Find a valid starting polygon and position on the navmesh.
fn find_start(nav_mesh: &NavMesh) -> (PolyRef, [f32; 3]) {
    let query = NavMeshQuery::new(nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];
    // Position verified in integration tests (nav_test.obj)
    query
        .find_nearest_poly(&[0.0, 0.0, 0.0], &extent, &filter)
        .expect("no start poly")
}

fn bench_find_nearest_poly(c: &mut Criterion) {
    let mut group = c.benchmark_group("find_nearest_poly");

    let nav_mesh = build_navmesh();

    // Benchmark with positions spread across the mesh
    let positions: Vec<[f32; 3]> = (0..20)
        .map(|i| {
            let t = i as f32 / 20.0;
            let x = -20.0 + t * 60.0;
            let z = -40.0 + t * 60.0;
            [x, 0.0, z]
        })
        .collect();

    group.bench_function("20_positions", |b| {
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        let extent = [5.0, 10.0, 5.0];
        b.iter(|| {
            for pos in &positions {
                let _ = query.find_nearest_poly(pos, &extent, &filter);
            }
        });
    });

    group.finish();
}

fn bench_raycast(c: &mut Criterion) {
    let mut group = c.benchmark_group("raycast");

    let nav_mesh = build_navmesh();
    let (start_ref, start_pos) = find_start(&nav_mesh);

    // Raycast in multiple directions from a known position
    let directions: Vec<[f32; 3]> = (0..16)
        .map(|i| {
            let angle = (i as f32) * std::f32::consts::TAU / 16.0;
            [angle.cos(), 0.0, angle.sin()]
        })
        .collect();

    group.bench_function("16_directions", |b| {
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        b.iter(|| {
            for dir in &directions {
                let _ = query.raycast(start_ref, &start_pos, dir, 100.0, &filter);
            }
        });
    });

    group.finish();
}

fn bench_find_distance_to_wall(c: &mut Criterion) {
    let mut group = c.benchmark_group("find_distance_to_wall");

    let nav_mesh = build_navmesh();
    let (start_ref, start_pos) = find_start(&nav_mesh);

    group.bench_function("single_query", |b| {
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        b.iter(|| {
            query
                .find_distance_to_wall(start_ref, &start_pos, 50.0, &filter)
                .unwrap()
        });
    });

    group.finish();
}

fn bench_move_along_surface(c: &mut Criterion) {
    let mut group = c.benchmark_group("move_along_surface");

    let nav_mesh = build_navmesh();
    let (start_ref, start_pos) = find_start(&nav_mesh);

    // Move in multiple directions
    let targets: Vec<[f32; 3]> = (0..8)
        .map(|i| {
            let angle = (i as f32) * std::f32::consts::TAU / 8.0;
            [
                start_pos[0] + angle.cos() * 5.0,
                start_pos[1],
                start_pos[2] + angle.sin() * 5.0,
            ]
        })
        .collect();

    group.bench_function("8_directions", |b| {
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        b.iter(|| {
            for target in &targets {
                let mut visited = Vec::new();
                let _ =
                    query.move_along_surface(start_ref, &start_pos, target, &filter, &mut visited);
            }
        });
    });

    group.finish();
}

fn bench_find_polys_around_circle(c: &mut Criterion) {
    let mut group = c.benchmark_group("find_polys_around_circle");

    let nav_mesh = build_navmesh();
    let (start_ref, start_pos) = find_start(&nav_mesh);

    for radius in [5.0, 15.0, 30.0] {
        group.bench_function(format!("radius_{}", radius as i32), |b| {
            let query = NavMeshQuery::new(&nav_mesh);
            let filter = QueryFilter::default();
            b.iter(|| {
                query
                    .find_polys_around_circle(start_ref, &start_pos, radius, &filter)
                    .unwrap()
            });
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_find_nearest_poly,
    bench_raycast,
    bench_find_distance_to_wall,
    bench_move_along_surface,
    bench_find_polys_around_circle
);
criterion_main!(benches);
