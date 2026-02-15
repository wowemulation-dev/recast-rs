//! Integration tests for the recast navmesh generation pipeline.
//!
//! Tests validate mesh loading and navmesh generation output against
//! reference data from `test-data/reference/`. Reference data includes
//! both C++ RecastNavigation CLI ground truth and current Rust output.
//!
//! Tests marked `#[ignore]` compare against C++ ground truth and will
//! pass once pipeline bugs are fixed.

use recast::{RecastBuilder, RecastConfig};
use recast_common::TriMesh;

/// Path to test-data relative to the workspace root.
/// Integration tests run from the crate directory, so we need to go up two levels.
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

fn build_mesh(
    obj_name: &str,
) -> (
    TriMesh,
    recast::PolyMesh,
    recast::PolyMeshDetail,
    RecastConfig,
) {
    let path = format!("{}/meshes/{}", TEST_DATA, obj_name);
    let mesh = TriMesh::from_obj(&path).expect("failed to load mesh");
    let (bmin, bmax) = mesh.calculate_bounds();
    let mut config = test_config();
    config.calculate_grid_size(bmin, bmax);
    let builder = RecastBuilder::new(config.clone());
    let (poly_mesh, detail_mesh) = builder
        .build_mesh(&mesh.vertices, &mesh.indices)
        .expect("failed to build mesh");
    (mesh, poly_mesh, detail_mesh, config)
}

// ── nav_test.obj ──

#[test]
fn nav_test_mesh_loading() {
    let path = format!("{}/meshes/nav_test.obj", TEST_DATA);
    let mesh = TriMesh::from_obj(&path).unwrap();
    assert_eq!(mesh.vert_count, 884);
    assert_eq!(mesh.tri_count, 1612);
}

#[test]
fn nav_test_grid_size() {
    let (_, _, _, config) = build_mesh("nav_test.obj");
    assert_eq!(config.width, 305);
    assert_eq!(config.height, 258);
}

#[test]
fn nav_test_generation_rust_output() {
    let (_, poly_mesh, detail_mesh, _) = build_mesh("nav_test.obj");

    // Current Rust output (regression test)
    // After hole merging fix — all metrics match C++ exactly for nav_test
    assert_eq!(poly_mesh.poly_count(), 537); // C++ 537 (exact)
    assert_eq!(poly_mesh.vert_count(), 1197); // C++ 1197 (exact)
    assert_eq!(detail_mesh.vert_count(), 2228); // C++ 2228 (exact)
    assert_eq!(detail_mesh.tri_count(), 1172); // C++ 1172 (exact)
}

#[test]
fn nav_test_generation_matches_cpp() {
    let (_, poly_mesh, _, _) = build_mesh("nav_test.obj");

    // C++ RecastCLI ground truth — now exact match
    assert_eq!(poly_mesh.poly_count(), 537);
    assert_eq!(poly_mesh.vert_count(), 1197);
}

// ── dungeon.obj ──

#[test]
fn dungeon_mesh_loading() {
    let path = format!("{}/meshes/dungeon.obj", TEST_DATA);
    let mesh = TriMesh::from_obj(&path).unwrap();
    assert_eq!(mesh.vert_count, 5101);
    assert_eq!(mesh.tri_count, 10133);
}

#[test]
fn dungeon_grid_size() {
    let (_, _, _, config) = build_mesh("dungeon.obj");
    assert_eq!(config.width, 248);
    assert_eq!(config.height, 330);
}

#[test]
fn dungeon_generation_rust_output() {
    let (_, poly_mesh, detail_mesh, _) = build_mesh("dungeon.obj");

    // Current Rust output (regression test)
    // After area border flag fix (Bug #29)
    assert_eq!(poly_mesh.poly_count(), 216); // C++ 217 (0.995x)
    assert_eq!(poly_mesh.vert_count(), 452); // C++ 452 (exact)
    assert_eq!(detail_mesh.vert_count(), 866); // C++ 868 (0.998x)
    assert_eq!(detail_mesh.tri_count(), 434); // C++ 434 (exact)
}

#[test]
#[ignore = "Rust produces ~0.98x polygons vs C++; remaining pipeline differences"]
fn dungeon_generation_matches_cpp() {
    let (_, poly_mesh, _, _) = build_mesh("dungeon.obj");

    // C++ RecastCLI ground truth
    assert_eq!(poly_mesh.poly_count(), 217);
    assert_eq!(poly_mesh.vert_count(), 452);
}

// ── bridge.obj ──

#[test]
fn bridge_mesh_loading() {
    let path = format!("{}/meshes/bridge.obj", TEST_DATA);
    let mesh = TriMesh::from_obj(&path).unwrap();
    assert_eq!(mesh.vert_count, 29);
    assert_eq!(mesh.tri_count, 54);
}

#[test]
fn bridge_grid_size() {
    let (_, _, _, config) = build_mesh("bridge.obj");
    // Grid size now matches C++ rcCalcGridSize using round-to-nearest
    assert_eq!(config.width, 16);
    assert_eq!(config.height, 137);
}

#[test]
fn bridge_generation_rust_output() {
    let (_, poly_mesh, detail_mesh, _) = build_mesh("bridge.obj");

    // Current Rust output (regression test)
    // After detail mesh restructure and grid fix
    assert_eq!(poly_mesh.poly_count(), 8);
    assert_eq!(poly_mesh.vert_count(), 18);
    assert_eq!(detail_mesh.vert_count(), 32);
    assert_eq!(detail_mesh.tri_count(), 16);
}

#[test]
#[ignore = "bridge matches C++ polygon count"]
fn bridge_generation_matches_cpp() {
    let (_, poly_mesh, _, _) = build_mesh("bridge.obj");

    // C++ RecastCLI ground truth
    assert_eq!(poly_mesh.poly_count(), 8);
    assert_eq!(poly_mesh.vert_count(), 18);
}

// ── Bounds validation ──

#[test]
fn nav_test_bounds() {
    let (_, poly_mesh, _, _) = build_mesh("nav_test.obj");

    let bmin = poly_mesh.bmin();
    let bmax = poly_mesh.bmax();
    assert!((bmin.x - (-28.889317)).abs() < 0.01);
    assert!((bmin.y - (-4.869517)).abs() < 0.01);
    assert!((bmin.z - (-46.300152)).abs() < 0.01);
    assert!((bmax.x - 62.494808).abs() < 0.01);
    // bmax.y includes walkable_height * ch adjustment (matching C++)
    assert!((bmax.y - 17.410753).abs() < 0.01);
    assert!((bmax.z - 31.053141).abs() < 0.01);
}

// ── Build does not panic ──

#[test]
fn all_meshes_build_without_error() {
    for obj in &["nav_test.obj", "dungeon.obj", "bridge.obj"] {
        let (_, _, _, _) = build_mesh(obj);
    }
}
