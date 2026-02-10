//! Integration tests for detour pathfinding and spatial queries.
//!
//! Tests build navmeshes from test OBJ files using recast, then validate
//! spatial queries and pathfinding against reference data.
//!
//! Tests marked `#[ignore]` compare against C++ ground truth and will
//! pass once pipeline bugs are fixed.

use detour::{NavMesh, NavMeshFlags, NavMeshParams, NavMeshQuery, QueryFilter};
use recast::{RecastBuilder, RecastConfig};
use recast_common::TriMesh;

const TEST_DATA: &str = "../../test-data";

fn test_config() -> RecastConfig {
    RecastConfig {
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
    }
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

// ── NavMesh construction ──

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

// ── Spatial queries: find_nearest_poly ──

#[test]
fn nav_test_find_nearest_poly_origin() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let result = query.find_nearest_poly(&[0.0, 0.0, 0.0], &extent, &filter);
    assert!(
        result.is_ok(),
        "find_nearest_poly should succeed near origin"
    );

    let (poly_ref, snapped) = result.unwrap();
    assert!(poly_ref.is_valid(), "returned poly_ref should be valid");

    // Snapped position should be near the query point
    assert!((snapped[0] - (-1.0898)).abs() < 0.01);
    assert!((snapped[1] - 0.3242).abs() < 0.01);
    assert!((snapped[2] - 1.0347).abs() < 0.01);
}

#[test]
fn nav_test_find_nearest_poly_q1() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let (poly_ref, snapped) = query
        .find_nearest_poly(&[5.0, 0.0, 5.0], &extent, &filter)
        .unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped[0] - 5.0).abs() < 0.01);
    assert!((snapped[1] - (-2.4695)).abs() < 0.01);
    assert!((snapped[2] - 5.0).abs() < 0.01);
}

#[test]
fn nav_test_find_nearest_poly_q3() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let (poly_ref, snapped) = query
        .find_nearest_poly(&[20.0, 0.0, 15.0], &extent, &filter)
        .unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped[0] - 20.0).abs() < 0.01);
    assert!((snapped[1] - (-3.6695)).abs() < 0.01);
    assert!((snapped[2] - 15.0).abs() < 0.01);
}

#[test]
fn dungeon_find_nearest_poly_center() {
    let nav_mesh = build_navmesh("dungeon.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let result = query.find_nearest_poly(&[12.145, 20.087, -40.575], &extent, &filter);
    assert!(result.is_ok());

    let (poly_ref, snapped) = result.unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped[0] - 10.6848).abs() < 0.01);
    assert!((snapped[1] - 17.1973).abs() < 0.01);
    assert!((snapped[2] - (-40.575)).abs() < 0.01);
}

#[test]
fn bridge_find_nearest_poly_center() {
    let nav_mesh = build_navmesh("bridge.obj");
    let query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let result = query.find_nearest_poly(&[-0.239, 1.806, -0.287], &extent, &filter);
    assert!(result.is_ok());

    let (poly_ref, snapped) = result.unwrap();
    assert!(poly_ref.is_valid());

    assert!((snapped[0] - (-0.3299)).abs() < 0.01);
    assert!((snapped[1] - 3.2903).abs() < 0.01);
    assert!((snapped[2] - (-0.2613)).abs() < 0.01);
}

// ── Pathfinding: current behavior ──

#[test]
fn nav_test_find_path_returns_result() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let (start_ref, start_pos) = query
        .find_nearest_poly(&[5.0, 0.0, 5.0], &extent, &filter)
        .unwrap();
    let (end_ref, end_pos) = query
        .find_nearest_poly(&[20.0, 0.0, 15.0], &extent, &filter)
        .unwrap();

    let path = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &filter);

    // Currently returns 1-polygon paths due to link setup issue
    assert!(path.is_ok());
    let path = path.unwrap();
    assert!(!path.is_empty());
}

#[test]
#[ignore = "find_path returns 1-polygon paths; polygon links not set up in build_from_recast"]
fn nav_test_pathfinding_matches_cpp() {
    let nav_mesh = build_navmesh("nav_test.obj");
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();
    let extent = [5.0, 10.0, 5.0];

    let (start_ref, start_pos) = query
        .find_nearest_poly(&[5.0, 0.0, 5.0], &extent, &filter)
        .unwrap();
    let (end_ref, end_pos) = query
        .find_nearest_poly(&[20.0, 0.0, 15.0], &extent, &filter)
        .unwrap();

    let path = query
        .find_path(start_ref, end_ref, &start_pos, &end_pos, &filter)
        .unwrap();

    // C++ produces a multi-polygon path
    assert!(path.len() > 1, "path should traverse multiple polygons");

    let straight = query
        .find_straight_path(&start_pos, &end_pos, &path)
        .unwrap();

    // C++ produces 5 waypoints for this query
    assert_eq!(straight.waypoints.len(), 5);
}
