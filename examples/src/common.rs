use detour::{NavMesh, NavMeshFlags, NavMeshParams};
use recast::{PolyMesh, PolyMeshDetail, RecastBuilder, RecastConfig};
use recast_common::TriMesh;

/// Build a navmesh from an OBJ file with default config.
///
/// Returns the NavMesh, PolyMesh, and PolyMeshDetail for use in examples.
/// The default config values match the CLI tool defaults.
pub fn build_navmesh_from_obj(
    path: &str,
) -> Result<(NavMesh, PolyMesh, PolyMeshDetail), Box<dyn std::error::Error>> {
    let mesh = TriMesh::from_obj(path)?;

    println!(
        "Loaded mesh: {} vertices, {} triangles",
        mesh.vert_count, mesh.tri_count
    );

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

    println!("Grid size: {}x{}", config.width, config.height);

    let builder = RecastBuilder::new(config);
    let (poly_mesh, detail_mesh) = builder.build_mesh(&mesh.vertices, &mesh.indices)?;

    println!(
        "Built navmesh: {} vertices, {} polygons",
        poly_mesh.nverts, poly_mesh.npolys
    );

    let params = NavMeshParams {
        origin: [bmin.x, bmin.y, bmin.z],
        tile_width: bmax.x - bmin.x,
        tile_height: bmax.z - bmin.z,
        max_tiles: 1,
        max_polys_per_tile: poly_mesh.npolys as i32,
    };

    let nav_mesh =
        NavMesh::build_from_recast(params, &poly_mesh, &detail_mesh, NavMeshFlags::empty())?;

    Ok((nav_mesh, poly_mesh, detail_mesh))
}

/// Print statistics about a PolyMesh and PolyMeshDetail.
pub fn print_stats(poly_mesh: &PolyMesh, detail_mesh: &PolyMeshDetail) {
    println!("--- Navmesh Statistics ---");
    println!("  Polygons:        {}", poly_mesh.npolys);
    println!("  Vertices:        {}", poly_mesh.nverts);
    println!("  Max verts/poly:  {}", poly_mesh.nvp);
    println!("  Detail vertices: {}", detail_mesh.vert_count);
    println!("  Detail triangles:{}", detail_mesh.tri_count);
    println!(
        "  Bounds: ({:.1}, {:.1}, {:.1}) - ({:.1}, {:.1}, {:.1})",
        poly_mesh.bmin.x,
        poly_mesh.bmin.y,
        poly_mesh.bmin.z,
        poly_mesh.bmax.x,
        poly_mesh.bmax.y,
        poly_mesh.bmax.z,
    );
}
