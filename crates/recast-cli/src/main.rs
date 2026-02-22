//! CLI utility for Recast navigation mesh generation and benchmarking

use anyhow::{Context, Result, anyhow};
use clap::{Parser, Subcommand};
use glam::Vec3;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use detour::{NavMesh, NavMeshFlags, NavMeshParams, NavMeshQuery, QueryFilter};
use recast::{
    CompactHeightfield, ContourSet, Heightfield, PolyMesh, PolyMeshDetail, RecastBuilder,
    RecastConfig, erode_walkable_area, rasterize_triangle,
};
use recast_common::TriMesh;

/// A CLI utility for Recast and Detour navigation mesh generation and pathfinding
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    #[clap(subcommand)]
    command: Commands,
}

/// Shared build configuration parameters
#[derive(Parser, Debug, Clone)]
struct BuildParams {
    /// Cell size (horizontal resolution)
    #[clap(long, default_value = "0.3")]
    cs: f32,

    /// Cell height (vertical resolution)
    #[clap(long, default_value = "0.2")]
    ch: f32,

    /// Maximum slope in degrees that is considered walkable
    #[clap(long, default_value = "45.0")]
    walkable_slope_angle: f32,

    /// Minimum floor to 'ceiling' height that will still allow the floor area to be considered
    /// walkable
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

    /// The maximum distance a simplified contour's border edges should deviate from the original
    /// raw contour
    #[clap(long, default_value = "1.3")]
    max_simplification_error: f32,

    /// The minimum number of cells allowed to form isolated island areas
    #[clap(long, default_value = "8")]
    min_region_area: i32,

    /// Any regions with an area smaller than this value will be merged with larger regions if
    /// possible
    #[clap(long, default_value = "20")]
    merge_region_area: i32,

    /// The maximum number of vertices allowed for polygons generated during the contour to polygon
    /// conversion process
    #[clap(long, default_value = "6")]
    max_vertices_per_polygon: i32,

    /// Sets the sampling distance to use when generating the detail mesh
    #[clap(long, default_value = "6.0")]
    detail_sample_dist: f32,

    /// The maximum distance the detail mesh surface should deviate from the heightfield data
    #[clap(long, default_value = "1.0")]
    detail_sample_max_error: f32,
}

impl BuildParams {
    fn to_config(&self, bmin: Vec3, bmax: Vec3) -> RecastConfig {
        let mut config = RecastConfig::default();
        config.cs = self.cs;
        config.ch = self.ch;
        config.walkable_slope_angle = self.walkable_slope_angle;
        config.walkable_height = self.walkable_height;
        config.walkable_climb = self.walkable_climb;
        config.walkable_radius = self.walkable_radius;
        config.max_edge_len = self.max_edge_len;
        config.max_simplification_error = self.max_simplification_error;
        config.min_region_area = self.min_region_area;
        config.merge_region_area = self.merge_region_area;
        config.max_vertices_per_polygon = self.max_vertices_per_polygon;
        config.detail_sample_dist = self.detail_sample_dist;
        config.detail_sample_max_error = self.detail_sample_max_error;
        config.calculate_grid_size(bmin, bmax);
        config
    }
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

        /// Print per-stage build timings to stderr
        #[clap(long)]
        timings: bool,

        #[clap(flatten)]
        params: BuildParams,
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

    /// Benchmark navigation mesh generation
    Bench {
        /// Input mesh file (OBJ format)
        #[clap(long, value_parser)]
        input: PathBuf,

        /// Number of iterations
        #[clap(long, default_value = "10")]
        iterations: u32,

        #[clap(flatten)]
        params: BuildParams,
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
            timings,
            params,
        } => build_mesh(&input, &output, &params, timings),
        Commands::FindPath {
            mesh,
            start,
            end,
            output,
        } => find_path(&mesh, start, end, output.as_deref()),
        Commands::Bench {
            input,
            iterations,
            params,
        } => bench(&input, &params, iterations),
    }
}

// ---------------------------------------------------------------------------
// Per-stage timing infrastructure
// ---------------------------------------------------------------------------

/// Collected per-stage durations from a single pipeline run.
#[derive(Clone, Default)]
struct StageTimes {
    rasterize: Duration,
    filter: Duration,
    build_compact: Duration,
    erode_area: Duration,
    build_regions: Duration,
    build_contours: Duration,
    build_polymesh: Duration,
    build_polymesh_detail: Duration,
    total: Duration,
}

impl StageTimes {
    fn print(&self) {
        let total_us = self.total.as_micros() as f64;
        let pct = |d: Duration| -> f64 {
            if total_us > 0.0 {
                d.as_micros() as f64 * 100.0 / total_us
            } else {
                0.0
            }
        };
        let ms = |d: Duration| -> f64 { d.as_micros() as f64 / 1000.0 };

        eprintln!("Build Times");
        eprintln!(
            "  Rasterize:              {:>8.2}ms  ({:>5.1}%)",
            ms(self.rasterize),
            pct(self.rasterize)
        );
        eprintln!(
            "  Filter:                 {:>8.2}ms  ({:>5.1}%)",
            ms(self.filter),
            pct(self.filter)
        );
        eprintln!(
            "  Build Compact:          {:>8.2}ms  ({:>5.1}%)",
            ms(self.build_compact),
            pct(self.build_compact)
        );
        eprintln!(
            "  Erode Area:             {:>8.2}ms  ({:>5.1}%)",
            ms(self.erode_area),
            pct(self.erode_area)
        );
        eprintln!(
            "  Build Regions:          {:>8.2}ms  ({:>5.1}%)",
            ms(self.build_regions),
            pct(self.build_regions)
        );
        eprintln!(
            "  Build Contours:         {:>8.2}ms  ({:>5.1}%)",
            ms(self.build_contours),
            pct(self.build_contours)
        );
        eprintln!(
            "  Build Polymesh:         {:>8.2}ms  ({:>5.1}%)",
            ms(self.build_polymesh),
            pct(self.build_polymesh)
        );
        eprintln!(
            "  Build Polymesh Detail:  {:>8.2}ms  ({:>5.1}%)",
            ms(self.build_polymesh_detail),
            pct(self.build_polymesh_detail)
        );
        eprintln!("  ===                     {:>8.2}ms  TOTAL", ms(self.total));
    }
}

/// Run the full Recast pipeline with per-stage timing.
///
/// Returns `(PolyMesh, PolyMeshDetail, StageTimes)`.
fn build_pipeline_timed(
    config: &RecastConfig,
    vertices: &[f32],
    indices: &[i32],
) -> Result<(PolyMesh, PolyMeshDetail, StageTimes), anyhow::Error> {
    let mut times = StageTimes::default();
    let total_start = Instant::now();

    // -- Rasterize triangles --------------------------------------------------
    let t = Instant::now();
    let mut heightfield = Heightfield::new(
        config.width,
        config.height,
        config.bmin,
        config.bmax,
        config.cs,
        config.ch,
    );
    let walkable_slope_threshold =
        (config.walkable_slope_angle * std::f32::consts::PI / 180.0).cos();

    let num_triangles = indices.len() / 3;
    for i in 0..num_triangles {
        let idx0 = indices[i * 3] as usize;
        let idx1 = indices[i * 3 + 1] as usize;
        let idx2 = indices[i * 3 + 2] as usize;

        let v0 = Vec3::new(
            vertices[idx0 * 3],
            vertices[idx0 * 3 + 1],
            vertices[idx0 * 3 + 2],
        );
        let v1 = Vec3::new(
            vertices[idx1 * 3],
            vertices[idx1 * 3 + 1],
            vertices[idx1 * 3 + 2],
        );
        let v2 = Vec3::new(
            vertices[idx2 * 3],
            vertices[idx2 * 3 + 1],
            vertices[idx2 * 3 + 2],
        );

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let cross = e1.cross(e2);
        if cross.length() < f32::EPSILON {
            continue;
        }
        let normal = cross.normalize();
        let area = if normal.y > walkable_slope_threshold {
            1
        } else {
            0
        };

        rasterize_triangle(&v0, &v1, &v2, area, &mut heightfield, 1)
            .map_err(|e| anyhow!("rasterize: {e}"))?;
    }
    times.rasterize = t.elapsed();

    // -- Filter ---------------------------------------------------------------
    let t = Instant::now();
    heightfield
        .filter_low_hanging_walkable_obstacles(config.walkable_climb as i16)
        .map_err(|e| anyhow!("filter low hanging: {e}"))?;
    heightfield
        .filter_ledge_spans(config.walkable_height as i16, config.walkable_climb as i16)
        .map_err(|e| anyhow!("filter ledge: {e}"))?;
    heightfield
        .filter_walkable_low_height_spans(config.walkable_height as i16)
        .map_err(|e| anyhow!("filter low height: {e}"))?;
    times.filter = t.elapsed();

    // -- Build Compact Heightfield --------------------------------------------
    let t = Instant::now();
    let mut compact_heightfield = CompactHeightfield::build_from_heightfield(
        &heightfield,
        config.walkable_height as i32,
        config.walkable_climb as i32,
    )
    .map_err(|e| anyhow!("build compact: {e}"))?;
    times.build_compact = t.elapsed();

    // -- Erode Area -----------------------------------------------------------
    let t = Instant::now();
    if config.walkable_radius > 0 {
        erode_walkable_area(&mut compact_heightfield, config.walkable_radius as i32)
            .map_err(|e| anyhow!("erode area: {e}"))?;
    }
    times.erode_area = t.elapsed();

    // -- Build Regions (includes distance field) ------------------------------
    let t = Instant::now();
    compact_heightfield
        .build_regions(
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        )
        .map_err(|e| anyhow!("build regions: {e}"))?;
    times.build_regions = t.elapsed();

    // -- Build Contours -------------------------------------------------------
    let t = Instant::now();
    let contour_set = ContourSet::build_from_compact_heightfield(
        &compact_heightfield,
        config.max_simplification_error,
        config.max_edge_len,
        config.min_region_area,
        config.merge_region_area,
    )
    .map_err(|e| anyhow!("build contours: {e}"))?;
    times.build_contours = t.elapsed();

    // -- Build Polymesh -------------------------------------------------------
    let t = Instant::now();
    let poly_mesh =
        PolyMesh::build_from_contour_set(&contour_set, config.max_vertices_per_polygon as usize)
            .map_err(|e| anyhow!("build polymesh: {e}"))?;
    times.build_polymesh = t.elapsed();

    // -- Build Polymesh Detail ------------------------------------------------
    let t = Instant::now();
    let detail_sample_dist = if config.detail_sample_dist < 0.9 {
        0.0
    } else {
        config.cs * config.detail_sample_dist
    };
    let detail_sample_max_error = config.ch * config.detail_sample_max_error;

    let poly_mesh_detail = PolyMeshDetail::build_from_poly_mesh(
        &poly_mesh,
        &compact_heightfield,
        detail_sample_dist,
        detail_sample_max_error,
    )
    .map_err(|e| anyhow!("build polymesh detail: {e}"))?;
    times.build_polymesh_detail = t.elapsed();

    times.total = total_start.elapsed();

    Ok((poly_mesh, poly_mesh_detail, times))
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/// Load mesh, compute bounds, build config.
fn load_and_configure(
    input: &Path,
    params: &BuildParams,
) -> Result<(TriMesh, Vec3, Vec3, RecastConfig)> {
    let mesh = TriMesh::from_obj(input).map_err(|e| anyhow!("Failed to load mesh: {}", e))?;
    let (bmin, bmax) = mesh.calculate_bounds();
    let config = params.to_config(bmin, bmax);
    Ok((mesh, bmin, bmax, config))
}

/// Build a navigation mesh from an input mesh
fn build_mesh(input: &Path, output: &Path, params: &BuildParams, timings: bool) -> Result<()> {
    eprintln!("Loading mesh from {}...", input.display());

    let (mesh, bmin, bmax, config) = load_and_configure(input, params)?;

    eprintln!(
        "Mesh loaded: {} vertices, {} triangles",
        mesh.vert_count, mesh.tri_count
    );
    eprintln!("Mesh bounds: min={:?}, max={:?}", bmin, bmax);
    eprintln!("Grid size: {}x{}", config.width, config.height);
    eprintln!("Building navigation mesh...");

    let (poly_mesh, poly_mesh_detail, stage_times) = if timings {
        build_pipeline_timed(&config, &mesh.vertices, &mesh.indices)?
    } else {
        let builder = RecastBuilder::new(config.clone());
        let t = Instant::now();
        let (pm, pmd) = builder
            .build_mesh(&mesh.vertices, &mesh.indices)
            .map_err(|e| anyhow!("Failed to build navigation mesh: {}", e))?;
        let total = t.elapsed();
        let times = StageTimes {
            total,
            ..Default::default()
        };
        (pm, pmd, times)
    };

    eprintln!(
        "Navigation mesh built: {} vertices, {} polygons",
        poly_mesh.vert_count(),
        poly_mesh.poly_count()
    );

    if timings {
        stage_times.print();
    } else {
        eprintln!(
            "Total build time: {:.2}ms",
            stage_times.total.as_micros() as f64 / 1000.0
        );
    }

    let mut nav_params = NavMeshParams::default();
    nav_params.origin = [bmin.x, bmin.y, bmin.z];
    nav_params.tile_width = bmax.x - bmin.x;
    nav_params.tile_height = bmax.z - bmin.z;
    nav_params.max_tiles = 1;
    nav_params.max_polys_per_tile = poly_mesh.poly_count() as i32;

    let nav_mesh = NavMesh::build_from_recast(
        nav_params,
        &poly_mesh,
        &poly_mesh_detail,
        NavMeshFlags::empty(),
    )
    .map_err(|e| anyhow!("Failed to convert to NavMesh: {:?}", e))?;

    eprintln!("Saving navigation mesh to {}...", output.display());
    save_nav_mesh(&nav_mesh, output)?;

    Ok(())
}

/// Benchmark navigation mesh generation
fn bench(input: &Path, params: &BuildParams, iterations: u32) -> Result<()> {
    eprintln!("Loading mesh from {}...", input.display());

    let (mesh, _bmin, _bmax, config) = load_and_configure(input, params)?;

    eprintln!(
        "Mesh loaded: {} vertices, {} triangles",
        mesh.vert_count, mesh.tri_count
    );
    eprintln!("Grid size: {}x{}", config.width, config.height);
    eprintln!("Running {} iterations...", iterations);
    eprintln!();

    let mut durations = Vec::with_capacity(iterations as usize);
    let mut last_times = StageTimes::default();

    for i in 0..iterations {
        let (_pm, _pmd, times) = build_pipeline_timed(&config, &mesh.vertices, &mesh.indices)?;
        eprint!(
            "  [{:>3}/{}] {:.2}ms\r",
            i + 1,
            iterations,
            times.total.as_micros() as f64 / 1000.0
        );
        durations.push(times.total);
        last_times = times;
    }
    eprintln!();

    durations.sort();
    let n = durations.len() as f64;
    let sum: Duration = durations.iter().sum();
    let mean = sum.as_micros() as f64 / n;
    let min = durations.first().unwrap().as_micros() as f64;
    let max = durations.last().unwrap().as_micros() as f64;
    let median = if durations.len() % 2 == 0 {
        let a = durations[durations.len() / 2 - 1].as_micros() as f64;
        let b = durations[durations.len() / 2].as_micros() as f64;
        (a + b) / 2.0
    } else {
        durations[durations.len() / 2].as_micros() as f64
    };
    let variance = durations
        .iter()
        .map(|d| {
            let diff = d.as_micros() as f64 - mean;
            diff * diff
        })
        .sum::<f64>()
        / n;
    let stddev = variance.sqrt();

    let ms = |us: f64| us / 1000.0;

    eprintln!("Results ({} iterations):", iterations);
    eprintln!("  Min:    {:>8.2}ms", ms(min));
    eprintln!("  Max:    {:>8.2}ms", ms(max));
    eprintln!("  Mean:   {:>8.2}ms", ms(mean));
    eprintln!("  Median: {:>8.2}ms", ms(median));
    eprintln!("  StdDev: {:>8.2}ms", ms(stddev));
    eprintln!();

    eprintln!("Per-stage breakdown (last iteration):");
    last_times.print();

    Ok(())
}

/// Save a NavMesh to a file, choosing format based on extension.
fn save_nav_mesh(nav_mesh: &NavMesh, output: &Path) -> Result<()> {
    let ext = output
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase();

    match ext.as_str() {
        "json" => {
            nav_mesh
                .save_to_json(output)
                .map_err(|e| anyhow!("Failed to save as JSON: {:?}", e))?;
            eprintln!("Saved navigation mesh as JSON");
        }
        _ => {
            nav_mesh
                .save_to_binary(output)
                .map_err(|e| anyhow!("Failed to save as binary: {:?}", e))?;
            eprintln!("Saved navigation mesh as binary");
        }
    }
    Ok(())
}

/// Find a path on a navigation mesh
fn find_path(mesh_path: &Path, start: Vec3, end: Vec3, output: Option<&Path>) -> Result<()> {
    println!("Loading navigation mesh from {}...", mesh_path.display());

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

    let mut query = NavMeshQuery::new(&nav_mesh);
    let start_pos = Vec3::new(start.x, start.y, start.z);
    let end_pos = Vec3::new(end.x, end.y, end.z);
    let ext = Vec3::new(2.0, 4.0, 2.0); // Search extents

    let filter = QueryFilter::default();

    let (start_ref, closest_start) = query
        .find_nearest_poly(start_pos, ext, &filter)
        .map_err(|e| anyhow!("Failed to find start polygon: {:?}", e))?;

    let (end_ref, closest_end) = query
        .find_nearest_poly(end_pos, ext, &filter)
        .map_err(|e| anyhow!("Failed to find end polygon: {:?}", e))?;

    println!(
        "Found start polygon: {:?} at {:?}",
        start_ref, closest_start
    );
    println!("Found end polygon: {:?} at {:?}", end_ref, closest_end);

    let path = query
        .find_path(start_ref, end_ref, closest_start, closest_end, &filter)
        .map_err(|e| anyhow!("Failed to find path: {:?}", e))?;

    println!("Found path with {} polygons", path.len());

    let straight_path = query
        .find_straight_path(closest_start, closest_end, &path)
        .map_err(|e| anyhow!("Failed to find straight path: {:?}", e))?;

    println!(
        "Generated straight path with {} waypoints",
        straight_path.waypoints.len()
    );

    if let Some(output_path) = output {
        println!("Saving path to {}...", output_path.display());

        let mut file = File::create(output_path)
            .with_context(|| format!("Failed to create output file: {}", output_path.display()))?;

        writeln!(file, "# Path from {:?} to {:?}", start, end)?;
        writeln!(file, "# {} waypoints", straight_path.waypoints.len())?;

        for waypoint in &straight_path.waypoints {
            writeln!(file, "{},{},{}", waypoint[0], waypoint[1], waypoint[2])?;
        }
    } else {
        println!("Path:");
        for (i, waypoint) in straight_path.waypoints.iter().enumerate() {
            println!("{}: {},{},{}", i, waypoint[0], waypoint[1], waypoint[2]);
        }
    }

    Ok(())
}
