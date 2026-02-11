//! Pathfinding example.
//!
//! Demonstrates end-to-end: OBJ -> navmesh -> A* path -> straight path waypoints.
//! Shows how to use NavMeshQuery with find_nearest_poly, find_path, and
//! find_straight_path.
//!
//! Run with:
//! ```sh
//! cargo run -p recast-rs-examples --example pathfinding
//! ```

use detour::{NavMeshQuery, QueryFilter};
use glam::Vec3;

use recast_rs_examples::common;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (nav_mesh, _poly_mesh, _detail_mesh) =
        common::build_navmesh_from_obj("examples/data/nav_test.obj")?;

    // Create a query object for pathfinding
    let mut query = NavMeshQuery::new(&nav_mesh);

    // Default filter includes all walkable polygons
    let filter = QueryFilter::default();

    // Search extents: how far from the query point to search for a polygon
    let extent = Vec3::new(2.0, 4.0, 2.0);

    // Define start and end positions on the mesh.
    // These positions are on opposite sides of nav_test.obj, forcing a longer path.
    let start_pos = Vec3::new(-20.0, 0.0, -20.0);
    let end_pos = Vec3::new(50.0, 0.0, 20.0);

    println!();
    println!("Finding path from {:?} to {:?}", start_pos, end_pos);

    // Snap positions to the nearest polygon on the navmesh
    let (start_ref, closest_start) = query.find_nearest_poly(start_pos, extent, &filter)?;
    let (end_ref, closest_end) = query.find_nearest_poly(end_pos, extent, &filter)?;

    println!("Start polygon: {:?} at {:?}", start_ref, closest_start);
    println!("End polygon:   {:?} at {:?}", end_ref, closest_end);

    // A* search: find a corridor of polygons from start to end
    let poly_path = query.find_path(start_ref, end_ref, closest_start, closest_end, &filter)?;
    println!("Polygon path: {} polygons", poly_path.len());

    // Funnel algorithm: convert polygon corridor to world-space waypoints
    let straight_path = query.find_straight_path(closest_start, closest_end, &poly_path)?;

    println!("Straight path: {} waypoints", straight_path.waypoints.len());
    println!();

    // Print each waypoint
    for (i, waypoint) in straight_path.waypoints.iter().enumerate() {
        println!(
            "  Waypoint {}: ({:.2}, {:.2}, {:.2})",
            i, waypoint[0], waypoint[1], waypoint[2]
        );
    }

    Ok(())
}
