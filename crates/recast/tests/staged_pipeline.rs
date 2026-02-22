//! Staged pipeline tests that verify intermediate results at each step.
//!
//! The navmesh generation pipeline has two categories of stages:
//!
//! **Deterministic stages** (Phase 1) — should produce identical results across
//! implementations given the same input:
//! - Heightfield rasterization
//! - Span filtering (low-hanging obstacles, ledge, low-height)
//! - Compact heightfield construction
//! - Walkable area erosion
//! - Distance field computation
//!
//! **Implementation-dependent stages** (Phase 2) — produce valid but potentially
//! different results due to heuristic tie-breaking in flood-fill, region merging,
//! contour simplification, and polygon construction:
//! - Watershed region building
//! - Contour generation
//! - Polygon mesh building
//! - Detail mesh generation
//!
//! Phase 1 tests assert exact values. Phase 2 tests assert structural properties
//! and regression values (which may change if the algorithm is improved).
//!
//! These tests were created by porting test methodology from the rerecast project
//! (github.com/janhohenheim/rerecast), which validates against C++ Recast reference
//! data at each pipeline stage.

use recast::{
    CompactHeightfield, ContourSet, Heightfield, PolyMesh, PolyMeshDetail, RecastConfig, SPAN_NULL,
    erode_walkable_area, rasterize_triangle,
};
use recast_common::TriMesh;

/// Path to test-data relative to the workspace root.
const TEST_DATA: &str = "../../test-data";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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

fn load_mesh(name: &str) -> (TriMesh, RecastConfig) {
    let path = format!("{}/meshes/{}", TEST_DATA, name);
    let mesh = TriMesh::from_obj(&path).expect("failed to load mesh");
    let (bmin, bmax) = mesh.calculate_bounds();
    let mut config = test_config();
    config.calculate_grid_size(bmin, bmax);
    (mesh, config)
}

/// Rasterize triangles into a heightfield WITHOUT filtering.
/// This separates rasterization from filtering so each stage can be
/// verified independently.
fn rasterize_without_filtering(
    mesh: &TriMesh,
    config: &RecastConfig,
) -> Heightfield {
    use glam::Vec3;

    let mut hf = Heightfield::new(
        config.width,
        config.height,
        config.bmin,
        config.bmax,
        config.cs,
        config.ch,
    );

    let walkable_slope_threshold =
        (config.walkable_slope_angle * std::f32::consts::PI / 180.0).cos();

    let num_triangles = mesh.indices.len() / 3;
    for i in 0..num_triangles {
        let idx0 = mesh.indices[i * 3] as usize;
        let idx1 = mesh.indices[i * 3 + 1] as usize;
        let idx2 = mesh.indices[i * 3 + 2] as usize;

        let v0 = Vec3::new(
            mesh.vertices[idx0 * 3],
            mesh.vertices[idx0 * 3 + 1],
            mesh.vertices[idx0 * 3 + 2],
        );
        let v1 = Vec3::new(
            mesh.vertices[idx1 * 3],
            mesh.vertices[idx1 * 3 + 1],
            mesh.vertices[idx1 * 3 + 2],
        );
        let v2 = Vec3::new(
            mesh.vertices[idx2 * 3],
            mesh.vertices[idx2 * 3 + 1],
            mesh.vertices[idx2 * 3 + 2],
        );

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let cross = e1.cross(e2);
        if cross.length() < f32::EPSILON {
            continue;
        }
        let normal = cross.normalize();
        let area = if normal.y > walkable_slope_threshold {
            1u8
        } else {
            0u8
        };

        rasterize_triangle(&v0, &v1, &v2, area, &mut hf, 1)
            .expect("rasterize_triangle failed");
    }

    hf
}

/// Count total and walkable spans in a heightfield.
fn count_spans(hf: &Heightfield) -> (u64, u64) {
    let mut total = 0u64;
    let mut walkable = 0u64;
    for z in 0..hf.height() {
        for x in 0..hf.width() {
            let mut si = hf.column_first_span(x, z);
            while si != SPAN_NULL {
                let s = hf.span(si);
                total += 1;
                if s.area != 0 {
                    walkable += 1;
                }
                si = s.next;
            }
        }
    }
    (total, walkable)
}

// ===========================================================================
// nav_test.obj — Phase 1: Deterministic stages (exact match with C++)
// ===========================================================================

mod nav_test {
    use super::*;

    fn setup() -> (TriMesh, RecastConfig) {
        load_mesh("nav_test.obj")
    }

    // -- Stage 1: Heightfield rasterization --

    #[test]
    fn stage1_rasterization_span_counts() {
        let (mesh, config) = setup();
        let hf = rasterize_without_filtering(&mesh, &config);

        let (total, walkable) = count_spans(&hf);

        // These values must match C++ rcRasterizeTriangles output exactly.
        // Total spans = all voxel columns created by triangle rasterization.
        // Walkable spans = spans whose triangle normal exceeds the slope threshold.
        assert_eq!(total, 120183, "total span count after rasterization");
        assert_eq!(walkable, 62983, "walkable span count after rasterization");
    }

    // -- Stage 1b: Individual filter steps --

    #[test]
    fn stage1b_filter_low_hanging_obstacles() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);

        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();

        let (_, walkable) = count_spans(&hf);
        // Low-hanging obstacle filter should not change walkable count for
        // this mesh (no low-hanging obstacles present).
        assert_eq!(walkable, 62983, "walkable after low-hanging filter");
    }

    #[test]
    fn stage1b_filter_ledge_spans() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);

        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();

        let (_, walkable) = count_spans(&hf);
        assert_eq!(walkable, 56928, "walkable after ledge filter");
    }

    #[test]
    fn stage1b_filter_walkable_low_height() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);

        // Apply all three filters in order
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let (total, walkable) = count_spans(&hf);
        // C++ reference: 56689 walkable spans after all filters.
        assert_eq!(total, 120183, "total spans unchanged by filtering");
        assert_eq!(walkable, 56689, "walkable after all filters (C++ reference)");
    }

    // -- Stage 2: Compact heightfield construction --

    #[test]
    fn stage2_compact_heightfield() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();

        assert_eq!(chf.span_count(), 56689, "compact heightfield span count");
        assert_eq!(
            chf.walkable_span_count(),
            56689,
            "compact heightfield walkable span count"
        );
        assert_eq!(chf.max_height(), 101, "compact heightfield max height");

        // Verify neighbor connections
        let mut connected = 0usize;
        let mut total_links = 0usize;
        for span in chf.spans() {
            let mut has_con = false;
            for dir in 0..4 {
                if span.con[dir] != 63 {
                    total_links += 1;
                    has_con = true;
                }
            }
            if has_con {
                connected += 1;
            }
        }
        assert_eq!(connected, 56628, "connected spans");
        assert_eq!(total_links, 214562, "total neighbor links");
    }

    // -- Stage 2b: Erosion --

    #[test]
    fn stage2b_erosion() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();

        let pre_erosion = chf.areas().iter().filter(|&&a| a != 0).count();
        assert_eq!(pre_erosion, 56689, "walkable before erosion");

        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();

        let post_erosion = chf.areas().iter().filter(|&&a| a != 0).count();
        assert_eq!(post_erosion, 47257, "walkable after erosion");
        assert_eq!(pre_erosion - post_erosion, 9432, "eroded span count");
    }

    // -- Stage 3: Distance field --

    #[test]
    fn stage3_distance_field() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();

        chf.build_distance_field().unwrap();

        assert_eq!(chf.max_distance(), 48, "max distance");

        let boundary = chf.dist().iter().filter(|&&d| d == 0).count();
        let interior = chf.dist().iter().filter(|&&d| d > 0 && d < u16::MAX).count();
        assert_eq!(boundary, 15853, "boundary spans (dist=0)");
        assert_eq!(interior, 40836, "interior spans (dist>0)");
    }

    // -- Stage 4+: Pipeline completion (Phase 2 — structural properties) --

    #[test]
    fn stage4_regions_properties() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();
        chf.build_distance_field().unwrap();

        chf.build_regions(
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

        // Structural properties that must hold regardless of implementation
        let assigned = chf.spans().iter().filter(|s| s.reg > 0).count();
        assert!(assigned > 0, "at least some spans should be assigned to regions");

        let normal_regions: std::collections::HashSet<u16> = chf
            .spans()
            .iter()
            .filter(|s| s.reg > 0 && s.reg < 0x8000)
            .map(|s| s.reg)
            .collect();
        assert!(
            normal_regions.len() >= 50,
            "should have substantial region count (got {})",
            normal_regions.len()
        );

        // All assigned spans should have region >= 1
        for span in chf.spans() {
            if span.reg > 0 {
                assert!(span.reg >= 1, "invalid region id");
            }
        }

        // Regression values — these may change if the watershed algorithm improves
        assert_eq!(chf.max_regions(), 249, "max_regions (regression)");
        assert_eq!(normal_regions.len(), 147, "normal region count (regression)");
        assert_eq!(assigned, 47027, "assigned spans (regression)");
    }

    // -- Full pipeline completion --

    #[test]
    fn full_pipeline_produces_valid_navmesh() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();
        chf.build_distance_field().unwrap();
        chf.build_regions(
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

        let contour_set = ContourSet::build_from_compact_heightfield(
            &chf,
            config.max_simplification_error,
            config.max_edge_len,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

        assert!(
            !contour_set.contours.is_empty(),
            "contour set should have contours"
        );

        let poly_mesh = PolyMesh::build_from_contour_set(
            &contour_set,
            config.max_vertices_per_polygon as usize,
        )
        .unwrap();

        // C++ reference: 537 polys, 1197 verts
        assert_eq!(poly_mesh.poly_count(), 537, "polygon count (C++ match)");
        assert_eq!(poly_mesh.vert_count(), 1197, "vertex count (C++ match)");

        let detail_sample_dist = if config.detail_sample_dist < 0.9 {
            0.0
        } else {
            config.cs * config.detail_sample_dist
        };
        let detail_sample_max_error = config.ch * config.detail_sample_max_error;
        let detail = PolyMeshDetail::build_from_poly_mesh(
            &poly_mesh,
            &chf,
            detail_sample_dist,
            detail_sample_max_error,
        )
        .unwrap();

        assert!(detail.vert_count() > 0, "detail mesh should have vertices");
        assert!(detail.tri_count() > 0, "detail mesh should have triangles");

        // C++ reference: 2228 verts, 1172 tris
        assert_eq!(detail.vert_count(), 2228, "detail vert count (C++ match)");
        assert_eq!(detail.tri_count(), 1172, "detail tri count (C++ match)");
    }
}

// ===========================================================================
// dungeon.obj — Phase 1: Deterministic stages
// ===========================================================================

mod dungeon {
    use super::*;

    fn setup() -> (TriMesh, RecastConfig) {
        load_mesh("dungeon.obj")
    }

    #[test]
    fn stage1_rasterization_span_counts() {
        let (mesh, config) = setup();
        let hf = rasterize_without_filtering(&mesh, &config);

        let (total, walkable) = count_spans(&hf);
        assert_eq!(total, 52106, "total span count");
        // Pre-filtering walkable count (filtering reduces this to 20648)
        assert_eq!(walkable, 26719, "walkable span count before filtering");
    }

    #[test]
    fn stage2_compact_heightfield() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();

        assert_eq!(chf.span_count(), 20648, "compact heightfield span count");
        assert_eq!(chf.max_height(), 201, "compact heightfield max height");
    }

    #[test]
    fn stage2b_erosion() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();

        let post_erosion = chf.areas().iter().filter(|&&a| a != 0).count();
        assert_eq!(post_erosion, 16335, "walkable after erosion");
    }

    #[test]
    fn stage3_distance_field() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();
        chf.build_distance_field().unwrap();

        assert_eq!(chf.max_distance(), 54, "max distance");
    }

    #[test]
    fn full_pipeline_produces_valid_navmesh() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();
        chf.build_distance_field().unwrap();
        chf.build_regions(
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

        let contour_set = ContourSet::build_from_compact_heightfield(
            &chf,
            config.max_simplification_error,
            config.max_edge_len,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

        let poly_mesh = PolyMesh::build_from_contour_set(
            &contour_set,
            config.max_vertices_per_polygon as usize,
        )
        .unwrap();

        // C++ reference: 217 polys, 452 verts
        assert_eq!(poly_mesh.poly_count(), 217, "polygon count (C++ match)");
        assert_eq!(poly_mesh.vert_count(), 452, "vertex count (C++ match)");

        let detail_sample_dist = config.cs * config.detail_sample_dist;
        let detail_sample_max_error = config.ch * config.detail_sample_max_error;
        let detail = PolyMeshDetail::build_from_poly_mesh(
            &poly_mesh,
            &chf,
            detail_sample_dist,
            detail_sample_max_error,
        )
        .unwrap();

        assert_eq!(detail.vert_count(), 868, "detail vert count (C++ match)");
        assert_eq!(detail.tri_count(), 434, "detail tri count (C++ match)");
    }
}

// ===========================================================================
// bridge.obj — Phase 1: Deterministic stages
// ===========================================================================

mod bridge {
    use super::*;

    fn setup() -> (TriMesh, RecastConfig) {
        load_mesh("bridge.obj")
    }

    #[test]
    fn stage1_rasterization_span_counts() {
        let (mesh, config) = setup();
        let hf = rasterize_without_filtering(&mesh, &config);

        let (total, walkable) = count_spans(&hf);
        assert_eq!(total, 3529, "total span count");
        // Pre-filtering walkable count (filtering reduces this to 1224)
        assert_eq!(walkable, 2060, "walkable span count before filtering");
    }

    #[test]
    fn stage2_compact_heightfield() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();

        assert_eq!(chf.span_count(), 1224, "compact heightfield span count");
        assert_eq!(chf.max_height(), 27, "compact heightfield max height");
    }

    #[test]
    fn stage2b_erosion() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();

        let post_erosion = chf.areas().iter().filter(|&&a| a != 0).count();
        assert_eq!(post_erosion, 825, "walkable after erosion");
    }

    #[test]
    fn stage3_distance_field() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();
        chf.build_distance_field().unwrap();

        assert_eq!(chf.max_distance(), 10, "max distance");
    }

    #[test]
    fn full_pipeline_produces_valid_navmesh() {
        let (mesh, config) = setup();
        let mut hf = rasterize_without_filtering(&mesh, &config);
        hf.filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
            .unwrap();
        hf.filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
            .unwrap();
        hf.filter_walkable_low_height_spans(config.walkable_height as i16)
            .unwrap();

        let mut chf = CompactHeightfield::build_from_heightfield(
            &hf,
            config.walkable_height as i32,
            config.walkable_climb as i32,
        )
        .unwrap();
        erode_walkable_area(&mut chf, config.walkable_radius as i32).unwrap();
        chf.build_distance_field().unwrap();
        chf.build_regions(
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

        let contour_set = ContourSet::build_from_compact_heightfield(
            &chf,
            config.max_simplification_error,
            config.max_edge_len,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

        let poly_mesh = PolyMesh::build_from_contour_set(
            &contour_set,
            config.max_vertices_per_polygon as usize,
        )
        .unwrap();

        // C++ reference: 8 polys, 18 verts
        assert_eq!(poly_mesh.poly_count(), 8, "polygon count (C++ match)");
        assert_eq!(poly_mesh.vert_count(), 18, "vertex count (C++ match)");

        let detail_sample_dist = config.cs * config.detail_sample_dist;
        let detail_sample_max_error = config.ch * config.detail_sample_max_error;
        let detail = PolyMeshDetail::build_from_poly_mesh(
            &poly_mesh,
            &chf,
            detail_sample_dist,
            detail_sample_max_error,
        )
        .unwrap();

        assert_eq!(detail.vert_count(), 32, "detail vert count");
        assert_eq!(detail.tri_count(), 16, "detail tri count");
    }
}
