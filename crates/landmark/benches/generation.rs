//! Benchmarks for the navmesh generation pipeline.
//!
//! Measures `RecastBuilder::build_mesh` for three mesh sizes:
//! - bridge.obj: ~30 triangles (small)
//! - nav_test.obj: ~1,600 triangles (medium)
//! - dungeon.obj: ~10,000 triangles (large)

use criterion::{Criterion, criterion_group, criterion_main};
use landmark::{RecastBuilder, RecastConfig};
use landmark_common::TriMesh;

const TEST_DATA: &str = "../../test-data/meshes";

fn default_config(mesh: &TriMesh) -> RecastConfig {
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
    config
}

fn load_mesh(name: &str) -> TriMesh {
    let path = format!("{TEST_DATA}/{name}");
    TriMesh::from_obj(&path).expect("failed to load test mesh")
}

fn bench_build_mesh(c: &mut Criterion) {
    let mut group = c.benchmark_group("navmesh_generation");

    let bridge = load_mesh("bridge.obj");
    let bridge_config = default_config(&bridge);
    group.bench_function("bridge_54tri", |b| {
        b.iter(|| {
            let builder = RecastBuilder::new(bridge_config.clone());
            builder
                .build_mesh(&bridge.vertices, &bridge.indices)
                .unwrap()
        });
    });

    let nav_test = load_mesh("nav_test.obj");
    let nav_test_config = default_config(&nav_test);
    group.bench_function("nav_test_1612tri", |b| {
        b.iter(|| {
            let builder = RecastBuilder::new(nav_test_config.clone());
            builder
                .build_mesh(&nav_test.vertices, &nav_test.indices)
                .unwrap()
        });
    });

    let dungeon = load_mesh("dungeon.obj");
    let dungeon_config = default_config(&dungeon);
    group.bench_function("dungeon_10133tri", |b| {
        b.iter(|| {
            let builder = RecastBuilder::new(dungeon_config.clone());
            builder
                .build_mesh(&dungeon.vertices, &dungeon.indices)
                .unwrap()
        });
    });

    group.finish();
}

criterion_group!(benches, bench_build_mesh);
criterion_main!(benches);
