# waymark-tilecache

Dynamic obstacle management and tile caching for navigation meshes.

[![Crates.io](https://img.shields.io/crates/v/waymark-tilecache.svg)](https://crates.io/crates/waymark-tilecache)
[![Documentation](https://docs.rs/waymark-tilecache/badge.svg)](https://docs.rs/waymark-tilecache)
[![License](https://img.shields.io/crates/l/waymark-tilecache.svg)](../LICENSE-MIT)
[![WASM](https://img.shields.io/badge/WASM-compatible-green.svg)](https://webassembly.org/)

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

```rust,ignore
use waymark_tilecache::{TileCache, TileCacheParams};

// Configure tile cache
let mut params = TileCacheParams::default();
params.width = 48;
params.height = 48;
params.max_obstacles = 128;

// Create tile cache
let mut tile_cache = TileCache::new(params)?;

// Add a cylinder obstacle (e.g., a tree or pillar)
let obstacle_id = tile_cache.add_obstacle(
    [10.0, 0.0, 10.0],  // position
    2.0,                 // radius
    4.0,                 // height
)?;

// Add a box obstacle (e.g., a crate)
let box_id = tile_cache.add_box_obstacle(
    [15.0, 0.0, 15.0],  // min corner
    [17.0, 2.0, 17.0],  // max corner
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

## WASM Support

This crate is fully compatible with WebAssembly. Build for WASM with:

```bash
cargo build --target wasm32-unknown-unknown -p waymark-tilecache
```

LZ4 compression uses `lz4_flex`, a pure Rust implementation that works on WASM.
Serialization works with in-memory buffers; file-based save/load is not
available on WASM.

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
