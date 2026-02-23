//! Tile cache obstacles example.
//!
//! Demonstrates dynamic obstacle management with TileCache: create a tile
//! cache, add cylinder and box obstacles, update, remove an obstacle, and
//! update again. Shows obstacle lifecycle management.
//!
//! Run with:
//! ```sh
//! cargo run -p recast-rs-examples --example tilecache_obstacles
//! ```

use waymark_tilecache::{TileCache, TileCacheParams};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create tile cache parameters matching the navmesh config
    let params = {
        let mut p = TileCacheParams::default();
        p.width = 100;
        p.height = 100;
        p.max_obstacles = 32;
        p
    };

    let mut tile_cache = TileCache::new(params)?;
    tile_cache.init()?;

    println!("Created tile cache: {}x{} tiles", 100, 100);
    println!("Max obstacles: 32");
    println!();

    // Add a cylinder obstacle
    let cylinder_ref = tile_cache.add_obstacle([10.0, 0.0, 10.0], 2.0, 3.0)?;
    println!(
        "Added cylinder obstacle (ref={}): pos=(10, 0, 10), radius=2.0, height=3.0",
        cylinder_ref
    );

    // Add a box obstacle
    let box_ref = tile_cache.add_box_obstacle([20.0, 0.0, 20.0], [24.0, 3.0, 24.0])?;
    println!(
        "Added box obstacle (ref={}): min=(20, 0, 20), max=(24, 3, 24)",
        box_ref
    );

    // Add an oriented box obstacle
    let oriented_ref =
        tile_cache.add_oriented_box_obstacle([30.0, 0.0, 15.0], [2.0, 1.5, 3.0], 0.785)?;
    println!(
        "Added oriented box obstacle (ref={}): center=(30, 0, 15), half=(2, 1.5, 3), rot=0.785",
        oriented_ref
    );

    println!();
    println!("Obstacle count: {}", tile_cache.get_obstacle_count());

    // Update tile cache to process pending obstacles
    tile_cache.update()?;
    println!("Updated tile cache (obstacles processed)");

    // Remove the cylinder obstacle
    tile_cache.remove_obstacle(cylinder_ref)?;
    println!();
    println!("Removed cylinder obstacle (ref={})", cylinder_ref);

    // Update again to process the removal
    tile_cache.update()?;
    println!("Updated tile cache (removal processed)");
    println!("Obstacle count: {}", tile_cache.get_obstacle_count());

    // Remove remaining obstacles
    tile_cache.remove_obstacle(box_ref)?;
    tile_cache.remove_obstacle(oriented_ref)?;
    tile_cache.update()?;
    println!();
    println!("Removed all remaining obstacles");
    println!("Obstacle count: {}", tile_cache.get_obstacle_count());

    Ok(())
}
