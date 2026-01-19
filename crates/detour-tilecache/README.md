# detour-tilecache

Dynamic obstacle management and tile caching for navigation meshes.

[![Crates.io](https://img.shields.io/crates/v/detour-tilecache.svg)](https://crates.io/crates/detour-tilecache)
[![Documentation](https://docs.rs/detour-tilecache/badge.svg)](https://docs.rs/detour-tilecache)
[![License](https://img.shields.io/crates/l/detour-tilecache.svg)](../LICENSE-MIT)

## Overview

DetourTileCache provides tile-based navigation mesh management with support for
runtime obstacle addition and removal. It enables dynamic environments where
obstacles can change without regenerating the entire navmesh.

This is a Rust port of the DetourTileCache component from [RecastNavigation][recast-cpp].

[recast-cpp]: https://github.com/recastnavigation/recastnavigation

## Features

- **Tile Caching**: Compressed storage of navigation mesh tiles
- **Dynamic Obstacles**: Add/remove obstacles at runtime
- **Obstacle Types**: Cylinder, box, and oriented box shapes
- **Incremental Updates**: Rebuild only affected tiles
- **LZ4 Compression**: Efficient tile storage
- **Streaming Support**: Handle large worlds through tile streaming

## Optional Features

- `serialization` - Save/load tile cache state via Serde

## Example

```rust
use detour_tilecache::{TileCache, TileCacheParams};
use glam::Vec3;

// Configure tile cache
let params = TileCacheParams {
    tile_width: 48,
    tile_height: 48,
    max_obstacles: 128,
    max_layers: 32,
    ..Default::default()
};

// Create tile cache
let mut tile_cache = TileCache::new(&params)?;

// Add a cylinder obstacle (e.g., a tree or pillar)
let obstacle_id = tile_cache.add_cylinder_obstacle(
    &Vec3::new(10.0, 0.0, 10.0),  // position
    2.0,                               // radius
    4.0,                               // height
)?;

// Add a box obstacle (e.g., a crate)
let box_id = tile_cache.add_box_obstacle(
    &Vec3::new(15.0, 0.0, 15.0),   // min corner
    &Vec3::new(17.0, 2.0, 17.0),   // max corner
)?;

// Update affected tiles
tile_cache.update()?;

// Later, remove an obstacle
tile_cache.remove_obstacle(obstacle_id)?;
tile_cache.update()?;
```

## Obstacle Types

| Type | Description | Use Case |
|------|-------------|----------|
| Cylinder | Circular base with height | Trees, pillars, characters |
| Box | Axis-aligned box | Crates, furniture |
| Oriented Box | Rotated box | Vehicles, angled objects |

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
