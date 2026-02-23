//! Benchmarks for pathfinding queries on navigation meshes.
//!
//! Measures find_path, find_straight_path, and sliced pathfinding
//! using nav_test.obj navmesh with known-good query positions from
//! integration test reference data.
//!
//! Note: find_path currently returns 1-polygon paths due to a known
//! polygon link setup issue in build_from_recast. These benchmarks
//! establish a baseline for when that bug is fixed.

use criterion::{Criterion, criterion_group, criterion_main};
use glam::Vec3;
use landmark::{RecastBuilder, RecastConfig};
use landmark_common::TriMesh;
use waymark::{NavMesh, NavMeshFlags, NavMeshParams, NavMeshQuery, PolyRef, QueryFilter};

const TEST_DATA: &str = "../../test-data/meshes";

fn build_navmesh(name: &str) -> NavMesh {
    let path = format!("{TEST_DATA}/{name}");
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

    let mut params = NavMeshParams::default();
    params.origin = [bmin.x, bmin.y, bmin.z];
    params.tile_width = bmax.x - bmin.x;
    params.tile_height = bmax.z - bmin.z;
    params.max_tiles = 1;
    params.max_polys_per_tile = poly_mesh.poly_count() as i32;

    NavMesh::build_from_recast(params, &poly_mesh, &detail_mesh, NavMeshFlags::empty())
        .expect("failed to build navmesh")
}

/// Snap a position to the navmesh, returning the poly ref and snapped position.
fn snap_to_navmesh(nav_mesh: &NavMesh, pos: [f32; 3]) -> (PolyRef, [f32; 3]) {
    let query = NavMeshQuery::new(nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);
    let (poly_ref, snapped) = query
        .find_nearest_poly(Vec3::from(pos), extent, &filter)
        .expect("position not on navmesh");
    (poly_ref, snapped.to_array())
}

fn bench_find_path(c: &mut Criterion) {
    let mut group = c.benchmark_group("find_path");

    let nav_mesh = build_navmesh("nav_test.obj");

    // Positions verified in integration tests
    let (start_ref, start_pos) = snap_to_navmesh(&nav_mesh, [0.0, 0.0, 0.0]);
    let (end_ref, end_pos) = snap_to_navmesh(&nav_mesh, [16.0, 0.0, -7.0]);

    group.bench_function("nav_test", |b| {
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        b.iter(|| {
            query
                .find_path(
                    start_ref,
                    end_ref,
                    Vec3::from(start_pos),
                    Vec3::from(end_pos),
                    &filter,
                )
                .unwrap()
        });
    });

    group.finish();
}

fn bench_find_straight_path(c: &mut Criterion) {
    let mut group = c.benchmark_group("find_straight_path");

    let nav_mesh = build_navmesh("nav_test.obj");

    let (start_ref, start_pos) = snap_to_navmesh(&nav_mesh, [0.0, 0.0, 0.0]);
    let (end_ref, end_pos) = snap_to_navmesh(&nav_mesh, [16.0, 0.0, -7.0]);

    // Pre-compute the poly path for the funnel algorithm benchmark
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let poly_path = query
        .find_path(
            start_ref,
            end_ref,
            Vec3::from(start_pos),
            Vec3::from(end_pos),
            &filter,
        )
        .expect("failed to find path");

    group.bench_function("nav_test", |b| {
        let query = NavMeshQuery::new(&nav_mesh);
        b.iter(|| {
            query
                .find_straight_path(Vec3::from(start_pos), Vec3::from(end_pos), &poly_path)
                .unwrap()
        });
    });

    group.finish();
}

fn bench_sliced_find_path(c: &mut Criterion) {
    let mut group = c.benchmark_group("sliced_find_path");

    let nav_mesh = build_navmesh("nav_test.obj");

    let (start_ref, start_pos) = snap_to_navmesh(&nav_mesh, [0.0, 0.0, 0.0]);
    let (end_ref, end_pos) = snap_to_navmesh(&nav_mesh, [16.0, 0.0, -7.0]);

    group.bench_function("nav_test", |b| {
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();
        b.iter(|| {
            query
                .init_sliced_find_path(
                    start_ref,
                    end_ref,
                    Vec3::from(start_pos),
                    Vec3::from(end_pos),
                    &filter,
                    0,
                )
                .unwrap();
            loop {
                let (_, state) = query.update_sliced_find_path(100).unwrap();
                if state != waymark::SlicedPathState::InProgress {
                    break;
                }
            }
            query.finalize_sliced_find_path(256).unwrap()
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_find_path,
    bench_find_straight_path,
    bench_sliced_find_path
);
criterion_main!(benches);
