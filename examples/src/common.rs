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

    println!("Grid size: {}x{}", config.width, config.height);

    let builder = RecastBuilder::new(config);
    let (poly_mesh, detail_mesh) = builder.build_mesh(&mesh.vertices, &mesh.indices)?;

    println!(
        "Built navmesh: {} vertices, {} polygons",
        poly_mesh.vert_count(),
        poly_mesh.poly_count()
    );

    let params = {
        let mut p = NavMeshParams::default();
        p.origin = [bmin.x, bmin.y, bmin.z];
        p.tile_width = bmax.x - bmin.x;
        p.tile_height = bmax.z - bmin.z;
        p.max_tiles = 1;
        p.max_polys_per_tile = poly_mesh.poly_count() as i32;
        p
    };

    let nav_mesh =
        NavMesh::build_from_recast(params, &poly_mesh, &detail_mesh, NavMeshFlags::empty())?;

    Ok((nav_mesh, poly_mesh, detail_mesh))
}

/// Print statistics about a PolyMesh and PolyMeshDetail.
pub fn print_stats(poly_mesh: &PolyMesh, detail_mesh: &PolyMeshDetail) {
    println!("--- Navmesh Statistics ---");
    println!("  Polygons:        {}", poly_mesh.poly_count());
    println!("  Vertices:        {}", poly_mesh.vert_count());
    println!("  Max verts/poly:  {}", poly_mesh.max_verts_per_poly());
    println!("  Detail vertices: {}", detail_mesh.vert_count());
    println!("  Detail triangles:{}", detail_mesh.tri_count());
    let bmin = poly_mesh.bmin();
    let bmax = poly_mesh.bmax();
    println!(
        "  Bounds: ({:.1}, {:.1}, {:.1}) - ({:.1}, {:.1}, {:.1})",
        bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z,
    );
}
