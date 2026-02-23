//! Integration tests for detour pathfinding and spatial queries.
//!
//! Tests build navmeshes from test OBJ files using recast, then validate
//! spatial queries and pathfinding against reference data from
//! `test-data/reference/*.json`.

use glam::Vec3;
use landmark::{RecastBuilder, RecastConfig};
use landmark_common::TriMesh;
use waymark::{NavMesh, NavMeshFlags, NavMeshParams, NavMeshQuery, QueryFilter};

const TEST_DATA: &str = "../../test-data";

fn test_config() -> RecastConfig {
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
    config
}

fn build_navmesh(obj_name: &str) -> NavMesh {
    let path = format!("{}/meshes/{}", TEST_DATA, obj_name);
    let mesh = TriMesh::from_obj(&path).expect("failed to load mesh");
    let (bmin, bmax) = mesh.calculate_bounds();
    let mut config = test_config();
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

// -- NavMesh construction --

#[test]
fn nav_test_navmesh_builds() {
    let _nav_mesh = build_navmesh("nav_test.obj");
}

#[test]
fn dungeon_navmesh_builds() {
    let _nav_mesh = build_navmesh("dungeon.obj");
}

#[test]
fn bridge_navmesh_builds() {
    let _nav_mesh = build_navmesh("bridge.obj");
}

// -- Spatial queries: find_nearest_poly --

#[test]
fn nav_test_find_nearest_poly_origin() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let result = query.find_nearest_poly(Vec3::new(0.0, 0.0, 0.0), extent, &filter);
    assert!(
        result.is_ok(),
        "find_nearest_poly should succeed near origin"
    );

    let (poly_ref, snapped) = result.unwrap();
    assert!(poly_ref.is_valid(), "returned poly_ref should be valid");

    assert!((snapped.x - 0.0000).abs() < 0.01);
    assert!((snapped.y - (-2.2695)).abs() < 0.01);
    assert!((snapped.z - (-2.2001)).abs() < 0.01);
}

#[test]
fn nav_test_find_nearest_poly_q1() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let (poly_ref, snapped) = query
        .find_nearest_poly(Vec3::new(5.0, 0.0, 0.0), extent, &filter)
        .unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped.x - 4.8514).abs() < 0.01);
    assert!((snapped.y - (-2.2695)).abs() < 0.01);
    assert!((snapped.z - (-2.3777)).abs() < 0.01);
}

#[test]
fn nav_test_find_nearest_poly_q3() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let (poly_ref, snapped) = query
        .find_nearest_poly(Vec3::new(16.0, 0.0, -7.0), extent, &filter)
        .unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped.x - 16.0).abs() < 0.01);
    assert!((snapped.y - (-2.2695)).abs() < 0.01);
    assert!((snapped.z - (-7.0)).abs() < 0.01);
}

#[test]
fn dungeon_find_nearest_poly_center() {
    let nav_mesh = build_navmesh("dungeon.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let result = query.find_nearest_poly(Vec3::new(12.145, 20.087, -40.575), extent, &filter);
    assert!(result.is_ok());

    let (poly_ref, snapped) = result.unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped.x - 12.1450).abs() < 0.01);
    assert!((snapped.y - 10.3074).abs() < 0.01);
    assert!((snapped.z - (-40.5750)).abs() < 0.01);
}

#[test]
fn bridge_find_nearest_poly_center() {
    let nav_mesh = build_navmesh("bridge.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let result = query.find_nearest_poly(Vec3::new(-0.239, 1.806, -0.287), extent, &filter);
    assert!(result.is_ok());

    let (poly_ref, snapped) = result.unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped.x - (-0.2390)).abs() < 0.01);
    assert!((snapped.y - 4.5951).abs() < 0.01);
    assert!((snapped.z - (-0.2870)).abs() < 0.01);
}

// -- Pathfinding: find_path --

#[test]
fn nav_test_find_path_returns_result() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let (start_ref, start_pos) = query
        .find_nearest_poly(Vec3::new(5.0, 0.0, 0.0), extent, &filter)
        .unwrap();
    let (end_ref, end_pos) = query
        .find_nearest_poly(Vec3::new(20.0, 0.0, 0.0), extent, &filter)
        .unwrap();

    let path = query
        .find_path(start_ref, end_ref, start_pos, end_pos, &filter)
        .unwrap();
    assert!(!path.is_empty());
    assert!(path.len() > 1, "path should traverse multiple polygons");
}

#[test]
fn nav_test_pathfinding_straight_path() {
    // Validates multi-polygon pathfinding with straight path (funnel algorithm).
    // Uses two points known to be on the Rust navmesh and far enough apart
    // to produce a multi-polygon path with intermediate waypoints.
    let nav_mesh = build_navmesh("nav_test.obj");
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    // (0,0,0) snaps to ~(0, -2.27, -2.2) and (16,0,-7) snaps to ~(16, -2.27, -7)
    let (start_ref, start_pos) = query
        .find_nearest_poly(Vec3::new(0.0, 0.0, 0.0), extent, &filter)
        .unwrap();
    let (end_ref, end_pos) = query
        .find_nearest_poly(Vec3::new(16.0, 0.0, -7.0), extent, &filter)
        .unwrap();

    let path = query
        .find_path(start_ref, end_ref, start_pos, end_pos, &filter)
        .unwrap();

    assert!(path.len() > 1, "path should traverse multiple polygons");

    let straight = query.find_straight_path(start_pos, end_pos, &path).unwrap();

    assert!(
        straight.waypoints.len() >= 2,
        "straight path should have at least start and end waypoints, got {}",
        straight.waypoints.len()
    );

    // First waypoint should be near the start position
    let first = straight.waypoints[0];
    assert!(
        (first.x - start_pos.x).abs() < 0.1,
        "first waypoint x={} should be near start x={}",
        first.x,
        start_pos.x
    );
    assert!(
        (first.z - start_pos.z).abs() < 0.1,
        "first waypoint z={} should be near start z={}",
        first.z,
        start_pos.z
    );

    // Last waypoint should be near the end position
    let last = straight.waypoints[straight.waypoints.len() - 1];
    assert!(
        (last.x - end_pos.x).abs() < 0.1,
        "last waypoint x={} should be near end x={}",
        last.x,
        end_pos.x
    );
    assert!(
        (last.z - end_pos.z).abs() < 0.1,
        "last waypoint z={} should be near end z={}",
        last.z,
        end_pos.z
    );
}

#[test]
fn dungeon_pathfinding_multi_polygon() {
    // C++ reference: start=[22.6,10.2,-45.9], end=[6.5,10.2,-18.3], 6 waypoints
    let nav_mesh = build_navmesh("dungeon.obj");
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let (start_ref, start_pos) = query
        .find_nearest_poly(Vec3::new(22.6, 10.2, -45.9), extent, &filter)
        .unwrap();
    let (end_ref, end_pos) = query
        .find_nearest_poly(Vec3::new(6.5, 10.2, -18.3), extent, &filter)
        .unwrap();

    let path = query
        .find_path(start_ref, end_ref, start_pos, end_pos, &filter)
        .unwrap();

    assert!(
        path.len() > 1,
        "dungeon path should traverse multiple polygons, got {}",
        path.len()
    );

    let straight = query.find_straight_path(start_pos, end_pos, &path).unwrap();

    assert!(
        straight.waypoints.len() >= 2,
        "dungeon straight path should have at least 2 waypoints, got {}",
        straight.waypoints.len()
    );
}

#[test]
fn bridge_pathfinding_multi_polygon() {
    // C++ reference: start=[0,3,5], end=[0,3,-5], 2 waypoints
    let nav_mesh = build_navmesh("bridge.obj");
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = Vec3::new(5.0, 10.0, 5.0);

    let (start_ref, start_pos) = query
        .find_nearest_poly(Vec3::new(0.0, 3.0, 5.0), extent, &filter)
        .unwrap();
    let (end_ref, end_pos) = query
        .find_nearest_poly(Vec3::new(0.0, 3.0, -5.0), extent, &filter)
        .unwrap();

    let path = query
        .find_path(start_ref, end_ref, start_pos, end_pos, &filter)
        .unwrap();

    assert!(
        path.len() > 1,
        "bridge path should traverse multiple polygons, got {}",
        path.len()
    );

    let straight = query.find_straight_path(start_pos, end_pos, &path).unwrap();

    assert!(
        straight.waypoints.len() >= 2,
        "bridge straight path should have at least 2 waypoints, got {}",
        straight.waypoints.len()
    );

    // Bridge is small — C++ produces exactly 2 waypoints (start + end).
    // Verify the start and end waypoints are near the query positions.
    let first = straight.waypoints[0];
    let last = straight.waypoints[straight.waypoints.len() - 1];

    assert!(
        (first.x - start_pos.x).abs() < 0.1 && (first.z - start_pos.z).abs() < 0.1,
        "first waypoint should be near start"
    );
    assert!(
        (last.x - end_pos.x).abs() < 0.1 && (last.z - end_pos.z).abs() < 0.1,
        "last waypoint should be near end"
    );
}
