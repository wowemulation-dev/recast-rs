# waymark-tilecache

Dynamic obstacle management with compressed tile storage.

**Lines**: ~2,700 | **WASM**: Yes | **Depends on**: `landmark-common`, `landmark`, `waymark`

## Overview

The `waymark-tilecache` crate provides runtime obstacle management. Obstacles
can be added or removed without regenerating the entire navigation mesh. Only
affected tiles are rebuilt.

## Main Types

### TileCache

The main cache managing tiles and obstacles.

```rust,ignore
use waymark_tilecache::{TileCache, TileCacheParams};

let mut params = TileCacheParams::default();
params.width = 48;
params.height = 48;
params.max_obstacles = 128;
let mut tile_cache = TileCache::new(params)?;

// Add a cylinder obstacle
let cylinder = tile_cache.add_obstacle(
    [10.0, 0.0, 10.0],  // position
    2.0,                 // radius
    4.0,                 // height
)?;

// Add a box obstacle
let box_obs = tile_cache.add_box_obstacle(
    [5.0, 0.0, 5.0],    // min
    [7.0, 3.0, 7.0],    // max
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
