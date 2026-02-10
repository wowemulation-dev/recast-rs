# detour-tilecache

Dynamic obstacle management with compressed tile storage.

**Lines**: ~2,700 | **WASM**: Yes | **Depends on**: `recast-common`, `recast`, `detour`

## Overview

The `detour-tilecache` crate provides runtime obstacle management. Obstacles
can be added or removed without regenerating the entire navigation mesh. Only
affected tiles are rebuilt.

## Main Types

### TileCache

The main cache managing tiles and obstacles.

```rust,ignore
use detour_tilecache::{TileCache, TileCacheParams};
use glam::Vec3;

let params = TileCacheParams {
    tile_width: 48,
    tile_height: 48,
    max_obstacles: 128,
    ..Default::default()
};
let mut tile_cache = TileCache::new(&params)?;

// Add obstacles
let cylinder = tile_cache.add_cylinder_obstacle(
    &Vec3::new(10.0, 0.0, 10.0),
    2.0,   // radius
    4.0,   // height
)?;

let box_obs = tile_cache.add_box_obstacle(
    &Vec3::new(5.0, 0.0, 5.0),   // min
    &Vec3::new(7.0, 3.0, 7.0),   // max
)?;

let oriented_box = tile_cache.add_oriented_box_obstacle(
    &Vec3::new(0.0, 0.0, 0.0),   // center
    &Vec3::new(2.0, 1.5, 1.0),   // half extents
    0.785,                         // rotation (radians)
)?;

// Process changes (rebuilds affected tiles)
tile_cache.update()?;

// Remove obstacle
tile_cache.remove_obstacle(cylinder)?;
tile_cache.update()?;
```

### TileCacheBuilder

Builds compressed tile data from heightfield layers.

### TileCacheLayer

Tile layer data for caching. Tile data is compressed with LZ4 (`lz4_flex`)
for memory efficiency.

## Obstacle Types

- **Cylinder**: Position, radius, height
- **Box**: Axis-aligned min/max bounds
- **Oriented Box**: Center, half extents, rotation angle

## Compression

Tile data is compressed with `lz4_flex`, a pure Rust LZ4 implementation.
This makes it WASM-compatible without C dependencies.

## Feature Flags

| Feature | Default | Description |
|---------|---------|-------------|
| `serialization` | No | Save/load tile cache data (serde, postcard) |
