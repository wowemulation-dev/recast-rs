//! CLI utility for 

use anyhow::{anyhow, Context, Result};
use clap::{Parser, Subcommand};
use glam::Vec3;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

use detour::{NavMesh, NavMeshFlags, NavMeshParams, NavMeshQuery, QueryFilter};
use recast::{RecastBuilder, RecastConfig};
use recast_common::TriMesh;

/// A CLI utility for Recast and Detour navigation mesh generation and pathfinding
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    #[clap(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Build a navigation mesh from an input mesh
    Build {
        /// Input mesh file (OBJ format)
        #[clap(long, value_parser)]
        input: PathBuf,

        /// Output navigation mesh file
        #[clap(long, value_parser)]
        output: PathBuf,

        /// Cell size (horizontal resolution)
        #[clap(long, default_value = "0.3")]
        cs: f32,

        /// Cell height (vertical resolution)
        #[clap(long, default_value = "0.2")]
        ch: f32,

        /// Maximum slope in degrees that is considered walkable
        #[clap(long, default_value = "45.0")]
        walkable_slope_angle: f32,

        /// Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable
        #[clap(long, default_value = "2")]
        walkable_height: i32,

        /// The maximum height between walkable layers
        #[clap(long, default_value = "1")]
        walkable_climb: i32,

        /// The distance to erode/shrink the walkable area from obstacles
        #[clap(long, default_value = "1")]
        walkable_radius: i32,

        /// The maximum allowed length for contour edges along the border of the mesh
        #[clap(long, default_value = "12")]
        max_edge_len: i32,

        /// The maximum distance a simplified contour's border edges should deviate from the original raw contour
        #[clap(long, default_value = "1.3")]
        max_simplification_error: f32,

        /// The minimum number of cells allowed to form isolated island areas
        #[clap(long, default_value = "8")]
        min_region_area: i32,

        /// Any regions with an area smaller than this value will be merged with larger regions if possible
        #[clap(long, default_value = "20")]
        merge_region_area: i32,

        /// The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process
        #[clap(long, default_value = "6")]
        max_vertices_per_polygon: i32,

        /// Sets the sampling distance to use when generating the detail mesh
        #[clap(long, default_value = "6.0")]
        detail_sample_dist: f32,

        /// The maximum distance the detail mesh surface should deviate from the heightfield data
        #[clap(long, default_value = "1.0")]
        detail_sample_max_error: f32,
    },

    /// Find a path on a navigation mesh
    FindPath {
        /// Input navigation mesh file
        #[clap(long, value_parser)]
        mesh: PathBuf,

        /// Start position (x,y,z)
        #[clap(long, value_parser = parse_vector)]
        start: Vec3,

        /// End position (x,y,z)
        #[clap(long, value_parser = parse_vector)]
        end: Vec3,

        /// Output path file
        #[clap(long, value_parser)]
        output: Option<PathBuf>,
    },
}

/// Parse a comma-separated vector
fn parse_vector(s: &str) -> Result<Vec3, String> {
    let parts: Vec<&str> = s.split(',').collect();

    if parts.len() != 3 {
        return Err(format!(
            "Vector must have 3 components, got {}",
            parts.len()
        ));
    }

    let x = parts[0].parse::<f32>().map_err(|e| e.to_string())?;
    let y = parts[1].parse::<f32>().map_err(|e| e.to_string())?;
    let z = parts[2].parse::<f32>().map_err(|e| e.to_string())?;

    Ok(Vec3::new(x, y, z))
}

fn main() -> Result<()> {
    let args = Args::parse();

    match args.command {
        Commands::Build {
            input,
            output,
            cs,
            ch,
            walkable_slope_angle,
            walkable_height,
            walkable_climb,
            walkable_radius,
            max_edge_len,
            max_simplification_error,
            min_region_area,
            merge_region_area,
            max_vertices_per_polygon,
            detail_sample_dist,
            detail_sample_max_error,
        } => build_mesh(
            &input,
            &output,
            cs,
            ch,
            walkable_slope_angle,
            walkable_height,
            walkable_climb,
            walkable_radius,
            max_edge_len,
            max_simplification_error,
            min_region_area,
            merge_region_area,
            max_vertices_per_polygon,
            detail_sample_dist,
            detail_sample_max_error,
        ),
        Commands::FindPath {
            mesh,
            start,
            end,
            output,
        } => find_path(&mesh, start, end, output.as_deref()),
    }
}

/// Build a navigation mesh from an input mesh
#[allow(clippy::too_many_arguments)]
fn build_mesh(
    input: &Path,
    output: &Path,
    cs: f32,
    ch: f32,
    walkable_slope_angle: f32,
    walkable_height: i32,
    walkable_climb: i32,
    walkable_radius: i32,
    max_edge_len: i32,
    max_simplification_error: f32,
    min_region_area: i32,
    merge_region_area: i32,
    max_vertices_per_polygon: i32,
    detail_sample_dist: f32,
    detail_sample_max_error: f32,
) -> Result<()> {
    println!("Loading mesh from {}...", input.display());

    // Load the mesh
    let mesh = TriMesh::from_obj(input).map_err(|e| anyhow!("Failed to load mesh: {}", e))?;

    println!(
        "Mesh loaded: {} vertices, {} triangles",
        mesh.vert_count, mesh.tri_count
    );

    // Calculate mesh bounds
    let (bmin, bmax) = mesh.calculate_bounds();

    println!("Mesh bounds: min={:?}, max={:?}", bmin, bmax);

    // Create Recast configuration
    let mut config = RecastConfig {
        cs,
        ch,
        walkable_slope_angle,
        walkable_height,
        walkable_climb,
        walkable_radius,
        max_edge_len,
        max_simplification_error,
        min_region_area,
        merge_region_area,
        max_vertices_per_polygon,
        detail_sample_dist,
        detail_sample_max_error,
        ..Default::default()
    };

    // Calculate grid size
    config.calculate_grid_size(bmin, bmax);

    println!("Building navigation mesh...");
    println!("Grid size: {}x{}", config.width, config.height);

    // Build the navigation mesh
    let builder = RecastBuilder::new(config);
    let (poly_mesh, poly_mesh_detail) = builder
        .build_mesh(&mesh.vertices, &mesh.indices)
        .map_err(|e| anyhow!("Failed to build navigation mesh: {}", e))?;

    println!(
        "Navigation mesh built: {} vertices, {} polygons",
        poly_mesh.vert_count, poly_mesh.poly_count
    );

    // Convert to Detour format
    let params = NavMeshParams {
        origin: [bmin.x, bmin.y, bmin.z],
        tile_width: (bmax.x - bmin.x),
        tile_height: (bmax.z - bmin.z),
        max_tiles: 1,
        max_polys_per_tile: poly_mesh.poly_count as i32,
    };

    let nav_mesh =
        NavMesh::build_from_recast(params, &poly_mesh, &poly_mesh_detail, NavMeshFlags::empty())
            .map_err(|e| anyhow!("Failed to convert to NavMesh: {:?}", e))?;

    println!("Navigation mesh conversion complete");

    // Save the navigation mesh to a file
    println!("Saving navigation mesh to {}...", output.display());

    // Determine format based on file extension
    if let Some(extension) = output.extension().and_then(|ext| ext.to_str()) {
        match extension.to_lowercase().as_str() {
            "json" => {
                nav_mesh
                    .save_to_json(output)
                    .map_err(|e| anyhow!("Failed to save as JSON: {:?}", e))?;
                println!("Saved navigation mesh as JSON");
            }
            "bin" | "navmesh" => {
                nav_mesh
                    .save_to_binary(output)
                    .map_err(|e| anyhow!("Failed to save as binary: {:?}", e))?;
                println!("Saved navigation mesh as binary");
            }
            _ => {
                // Default to binary format
                nav_mesh
                    .save_to_binary(output)
                    .map_err(|e| anyhow!("Failed to save as binary: {:?}", e))?;
                println!("Saved navigation mesh as binary (default format)");
            }
        }
    } else {
        // No extension, default to binary
        nav_mesh
            .save_to_binary(output)
            .map_err(|e| anyhow!("Failed to save as binary: {:?}", e))?;
        println!("Saved navigation mesh as binary (default format)");
    }

    Ok(())
}

/// Find a path on a navigation mesh
fn find_path(
    mesh_path: &Path,
    start: Vec3,
    end: Vec3,
    output: Option<&Path>,
) -> Result<()> {
    println!("Loading navigation mesh from {}...", mesh_path.display());

    // Load the navigation mesh from a file
    let nav_mesh = if let Some(extension) = mesh_path.extension().and_then(|ext| ext.to_str()) {
        match extension.to_lowercase().as_str() {
            "json" => NavMesh::load_from_json(mesh_path)
                .map_err(|e| anyhow!("Failed to load JSON navigation mesh: {:?}", e))?,
            "bin" | "navmesh" => NavMesh::load_from_binary(mesh_path)
                .map_err(|e| anyhow!("Failed to load binary navigation mesh: {:?}", e))?,
            _ => {
                // Try binary first, then JSON as fallback
                NavMesh::load_from_binary(mesh_path)
                    .or_else(|_| NavMesh::load_from_json(mesh_path))
                    .map_err(|e| {
                        anyhow!(
                            "Failed to load navigation mesh (tried both binary and JSON): {:?}",
                            e
                        )
                    })?
            }
        }
    } else {
        // No extension, try binary first, then JSON as fallback
        NavMesh::load_from_binary(mesh_path)
            .or_else(|_| NavMesh::load_from_json(mesh_path))
            .map_err(|e| {
                anyhow!(
                    "Failed to load navigation mesh (tried both binary and JSON): {:?}",
                    e
                )
            })?
    };

    println!("Successfully loaded navigation mesh");

    println!("Finding path from {:?} to {:?}...", start, end);

    // Create a query
    let mut query = NavMeshQuery::new(&nav_mesh);

    // Find nearest polygons to start and end points
    let start_pos = [start.x, start.y, start.z];
    let end_pos = [end.x, end.y, end.z];
    let ext = [2.0, 4.0, 2.0]; // Search extents

    // Create a default filter
    let filter = QueryFilter::default();

    // Find nearest polygons to start and end positions
    let (start_ref, closest_start) = query
        .find_nearest_poly(&start_pos, &ext, &filter)
        .map_err(|e| anyhow!("Failed to find start polygon: {:?}", e))?;

    let (end_ref, closest_end) = query
        .find_nearest_poly(&end_pos, &ext, &filter)
        .map_err(|e| anyhow!("Failed to find end polygon: {:?}", e))?;

    println!(
        "Found start polygon: {:?} at {:?}",
        start_ref, closest_start
    );
    println!("Found end polygon: {:?} at {:?}", end_ref, closest_end);

    // Find the path
    let path = query
        .find_path(start_ref, end_ref, &closest_start, &closest_end, &filter)
        .map_err(|e| anyhow!("Failed to find path: {:?}", e))?;

    println!("Found path with {} polygons", path.len());

    // Convert polygon path to straight path
    let straight_path = query
        .find_straight_path(&closest_start, &closest_end, &path)
        .map_err(|e| anyhow!("Failed to find straight path: {:?}", e))?;

    println!(
        "Generated straight path with {} waypoints",
        straight_path.waypoints.len()
    );

    // Output the path
    if let Some(output_path) = output {
        println!("Saving path to {}...", output_path.display());

        let mut file = File::create(output_path)
            .with_context(|| format!("Failed to create output file: {}", output_path.display()))?;

        // Write the path
        writeln!(file, "# Path from {:?} to {:?}", start, end)?;
        writeln!(file, "# {} waypoints", straight_path.waypoints.len())?;

        for waypoint in &straight_path.waypoints {
            writeln!(file, "{},{},{}", waypoint[0], waypoint[1], waypoint[2])?;
        }
    } else {
        // Print the path to stdout
        println!("Path:");
        for (i, waypoint) in straight_path.waypoints.iter().enumerate() {
            println!("{}: {},{},{}", i, waypoint[0], waypoint[1], waypoint[2]);
        }
    }

    Ok(())
}
