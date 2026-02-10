//! Basic navmesh generation example.
//!
//! Demonstrates the Recast generation pipeline: load OBJ mesh,
//! configure parameters, build navmesh, and print statistics.
//!
//! Run with:
//! ```sh
//! cargo run -p recast-rs-examples --example basic_navmesh
//! ```

use std::time::Instant;

use recast_rs_examples::common;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();

    let (_nav_mesh, poly_mesh, detail_mesh) =
        common::build_navmesh_from_obj("examples/data/nav_test.obj")?;

    let elapsed = start.elapsed();

    println!();
    common::print_stats(&poly_mesh, &detail_mesh);
    println!();
    println!("Build time: {:.2?}", elapsed);

    Ok(())
}
