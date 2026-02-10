//! Pipeline diagnostic tests that dump intermediate values at each stage.
//!
//! These tests help identify exactly where the Rust pipeline diverges from C++.
//! Run with: cargo test -p recast --test pipeline_diagnostic -- --nocapture

use recast::{
    CompactHeightfield, ContourSet, PolyMesh, PolyMeshDetail, RecastBuilder, RecastConfig,
    erode_walkable_area,
};
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

fn run_pipeline_diagnostic(obj_name: &str) {
    let path = format!("{}/meshes/{}", TEST_DATA, obj_name);
    let mesh = TriMesh::from_obj(&path).expect("failed to load mesh");
    let (bmin, bmax) = mesh.calculate_bounds();
    let mut config = test_config();
    config.calculate_grid_size(bmin, bmax);

    println!();
    println!("============================================================");
    println!("  PIPELINE DIAGNOSTIC: {}", obj_name);
    println!("============================================================");
    println!();
    println!(
        "Config: cs={} ch={} walkable_height={} walkable_climb={} walkable_radius={}",
        config.cs, config.ch, config.walkable_height, config.walkable_climb, config.walkable_radius
    );
    println!(
        "  max_edge_len={} max_simplification_error={} min_region_area={} merge_region_area={}",
        config.max_edge_len,
        config.max_simplification_error,
        config.min_region_area,
        config.merge_region_area
    );
    println!();

    // Stage 0: Input
    println!("--- Stage 0: Input Mesh ---");
    println!("  vertices: {}", mesh.vert_count);
    println!("  triangles: {}", mesh.tri_count);
    println!(
        "  bounds: min=[{:.3}, {:.3}, {:.3}] max=[{:.3}, {:.3}, {:.3}]",
        bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z
    );
    println!("  grid: {}x{}", config.width, config.height);
    println!();

    // Stage 1: Heightfield (rasterization)
    println!("--- Stage 1: Heightfield ---");
    let builder = RecastBuilder::new(config.clone());
    let heightfield = builder
        .build_heightfield(&mesh.vertices, &mesh.indices)
        .expect("failed to build heightfield");

    let mut total_spans = 0u64;
    let mut walkable_spans = 0u64;
    let mut cells_with_spans = 0u64;
    for (_coord, span_opt) in &heightfield.spans {
        if let Some(first_span_rc) = span_opt {
            cells_with_spans += 1;
            let mut current = Some(first_span_rc.clone());
            while let Some(span_rc) = current {
                let span = span_rc.borrow();
                total_spans += 1;
                if span.area != 0 {
                    walkable_spans += 1;
                }
                current = span.next.clone();
            }
        }
    }
    println!("  total_spans: {}", total_spans);
    println!("  walkable_spans: {}", walkable_spans);
    println!("  non_walkable_spans: {}", total_spans - walkable_spans);
    println!(
        "  cells_with_spans: {} / {}",
        cells_with_spans,
        config.width as u64 * config.height as u64
    );
    println!();

    // Stage 2: Compact Heightfield
    println!("--- Stage 2: Compact Heightfield ---");
    let mut chf = CompactHeightfield::build_from_heightfield(
        &heightfield,
        config.walkable_height as i32,
        config.walkable_climb as i32,
    )
    .expect("failed to build compact heightfield");

    println!("  span_count: {}", chf.span_count);
    println!("  walkable_span_count: {}", chf.walkable_span_count);
    println!("  max_height: {}", chf.max_height);

    // Count connections (63 = RC_NOT_CONNECTED)
    let mut connected_spans = 0usize;
    let mut total_connections = 0usize;
    for span in &chf.spans {
        let mut has_con = false;
        for dir in 0..4 {
            if span.con[dir] != 63 {
                total_connections += 1;
                has_con = true;
            }
        }
        if has_con {
            connected_spans += 1;
        }
    }
    println!(
        "  connected_spans: {} / {}",
        connected_spans, chf.span_count
    );
    println!("  total_con4_links: {}", total_connections);
    println!("  linked_list_connections: {}", chf.connections.len());
    println!();

    // Stage 2b: Erosion (matching C++ rcErodeWalkableArea)
    println!(
        "--- Stage 2b: Erosion (walkable_radius={}) ---",
        config.walkable_radius
    );
    let pre_erosion_walkable = chf.areas.iter().filter(|&&a| a != 0).count();
    println!("  walkable_before_erosion: {}", pre_erosion_walkable);
    if config.walkable_radius > 0 {
        erode_walkable_area(&mut chf, config.walkable_radius as i32)
            .expect("failed to erode walkable area");
    }
    let post_erosion_walkable = chf.areas.iter().filter(|&&a| a != 0).count();
    println!("  walkable_after_erosion: {}", post_erosion_walkable);
    println!(
        "  eroded_spans: {}",
        pre_erosion_walkable - post_erosion_walkable
    );
    println!();

    // Stage 3: Distance Field
    println!("--- Stage 3: Distance Field ---");
    chf.build_distance_field()
        .expect("failed to build distance field");
    println!("  max_distance: {}", chf.max_distance);

    let zero_dist = chf.dist.iter().filter(|&&d| d == 0).count();
    let nonzero_dist = chf.dist.iter().filter(|&&d| d > 0 && d < u16::MAX).count();
    println!("  boundary_spans (dist=0): {}", zero_dist);
    println!("  interior_spans (dist>0): {}", nonzero_dist);
    println!();

    // Stage 4: Regions (watershed)
    println!("--- Stage 4: Regions (Watershed) ---");
    chf.build_regions(
        config.border_size,
        config.min_region_area,
        config.merge_region_area,
    )
    .expect("failed to build regions");

    println!("  max_regions: {}", chf.max_regions);

    // Count spans per region
    let mut region_span_counts = std::collections::HashMap::new();
    for span in &chf.spans {
        if span.reg > 0 {
            *region_span_counts.entry(span.reg).or_insert(0u32) += 1;
        }
    }
    let unassigned = chf.spans.iter().filter(|s| s.reg == 0).count();
    let assigned = chf.spans.iter().filter(|s| s.reg > 0).count();
    let border_regs = region_span_counts.keys().filter(|&&r| r >= 0x8000).count();
    let normal_regs = region_span_counts
        .keys()
        .filter(|&&r| r > 0 && r < 0x8000)
        .count();

    println!("  normal_regions: {}", normal_regs);
    println!("  border_regions: {}", border_regs);
    println!("  assigned_spans: {}", assigned);
    println!("  unassigned_spans: {}", unassigned);

    // Region size distribution
    let mut sizes: Vec<u32> = region_span_counts
        .iter()
        .filter(|&(r, _)| *r > 0 && *r < 0x8000)
        .map(|(_, &c)| c)
        .collect();
    sizes.sort_unstable();
    if !sizes.is_empty() {
        println!(
            "  region_sizes: min={} median={} max={} total={}",
            sizes[0],
            sizes[sizes.len() / 2],
            sizes[sizes.len() - 1],
            sizes.iter().sum::<u32>()
        );
    }
    println!();

    // Stage 5: Contours
    println!("--- Stage 5: Contour Set ---");
    let contour_set = ContourSet::build_from_compact_heightfield(
        &chf,
        config.max_simplification_error,
        config.max_edge_len,
        config.min_region_area,
        config.merge_region_area,
    )
    .expect("failed to build contours");

    println!("  contour_count: {}", contour_set.contours.len());
    let total_contour_verts: usize = contour_set.contours.iter().map(|c| c.vertices.len()).sum();
    let min_verts = contour_set
        .contours
        .iter()
        .map(|c| c.vertices.len())
        .min()
        .unwrap_or(0);
    let max_verts = contour_set
        .contours
        .iter()
        .map(|c| c.vertices.len())
        .max()
        .unwrap_or(0);
    println!("  total_contour_vertices: {}", total_contour_verts);
    println!(
        "  vertices_per_contour: min={} max={} avg={:.1}",
        min_verts,
        max_verts,
        if contour_set.contours.is_empty() {
            0.0
        } else {
            total_contour_verts as f64 / contour_set.contours.len() as f64
        }
    );
    println!();

    // Stage 6: PolyMesh
    println!("--- Stage 6: Polygon Mesh ---");
    let poly_mesh =
        PolyMesh::build_from_contour_set(&contour_set, config.max_vertices_per_polygon as usize)
            .expect("failed to build poly mesh");

    println!("  nverts: {}", poly_mesh.nverts);
    println!("  npolys: {}", poly_mesh.npolys);
    println!("  maxpolys: {}", poly_mesh.maxpolys);
    println!();

    // Stage 7: PolyMeshDetail
    println!("--- Stage 7: Detail Mesh ---");
    let detail_mesh = PolyMeshDetail::build_from_poly_mesh(
        &poly_mesh,
        &chf,
        config.detail_sample_dist,
        config.detail_sample_max_error,
    )
    .expect("failed to build detail mesh");

    println!("  vert_count: {}", detail_mesh.vert_count);
    println!("  tri_count: {}", detail_mesh.tri_count);
    println!();

    println!("--- Summary ---");
    println!(
        "  {} -> {} spans -> {} regions -> {} contours -> {} polys ({} verts)",
        obj_name,
        chf.span_count,
        normal_regs,
        contour_set.contours.len(),
        poly_mesh.npolys,
        poly_mesh.nverts
    );
    println!();
}

#[test]
fn pipeline_diagnostic_nav_test() {
    run_pipeline_diagnostic("nav_test.obj");
}

#[test]
fn pipeline_diagnostic_dungeon() {
    run_pipeline_diagnostic("dungeon.obj");
}

#[test]
fn pipeline_diagnostic_bridge() {
    run_pipeline_diagnostic("bridge.obj");
}
