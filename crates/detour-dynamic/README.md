# detour-dynamic

Dynamic navigation mesh generation with real-time obstacle support.

[![Crates.io](https://img.shields.io/crates/v/detour-dynamic.svg)](https://crates.io/crates/detour-dynamic)
[![Documentation](https://docs.rs/detour-dynamic/badge.svg)](https://docs.rs/detour-dynamic)
[![License](https://img.shields.io/crates/l/detour-dynamic.svg)](../LICENSE-MIT)
[![WASM](https://img.shields.io/badge/WASM-compatible-green.svg)](https://webassembly.org/)

## Overview

DetourDynamic provides dynamic navigation mesh generation capabilities,
allowing real-time modification of navigation meshes by adding and removing
obstacles. This is useful for games with destructible environments, moving
platforms, or procedurally generated content.

## Features

- **Dynamic Obstacle Management**: Add and remove colliders at runtime
- **Incremental Updates**: Only rebuild affected tiles for performance
- **Async Processing**: Non-blocking navmesh updates (WASM-compatible)
- **Multiple Collider Types**: Box, cylinder, sphere, trimesh, and composites
- **Checkpoint System**: Efficient state management for incremental rebuilds
- **Voxel-based Queries**: Precise raycasting against heightfield data
- **Serialization**: Save and load dynamic navmesh state

## Optional Features

- `tokio` - Use Tokio runtime for async operations (not WASM-compatible)

## Collider Types

| Collider | Description |
|----------|-------------|
| `BoxCollider` | Axis-aligned or oriented box |
| `CylinderCollider` | Vertical cylinder |
| `SphereCollider` | Sphere shape |
| `CapsuleCollider` | Capsule (cylinder with hemispherical caps) |
| `ConvexCollider` | Convex hull from points |
| `TrimeshCollider` | Arbitrary triangle mesh |
| `CompositeCollider` | Combine multiple colliders |

## Example

```rust
use detour_dynamic::{DynamicNavMesh, DynamicNavMeshConfig};
use detour_dynamic::colliders::BoxCollider;
use glam::Vec3;
use std::sync::Arc;

// Create configuration
let config = DynamicNavMeshConfig {
    world_min: Vec3::new(-50.0, -1.0, -50.0),
    world_max: Vec3::new(50.0, 10.0, 50.0),
    cell_size: 0.3,
    cell_height: 0.2,
    walkable_height: 2.0,
    walkable_radius: 0.6,
    walkable_climb: 0.9,
    walkable_slope_angle: 45.0,
    ..Default::default()
};

// Create dynamic navmesh
let mut navmesh = DynamicNavMesh::with_tile_grid(config, 4, 4)?;

// Add a dynamic obstacle
let collider = Arc::new(BoxCollider::new(
    Vec3::new(5.0, 0.0, 5.0),   // center
    Vec3::new(1.0, 2.0, 1.0),   // half extents
    1,                              // area type
    1.0,                            // flag merge threshold
));
navmesh.add_collider(collider)?;

// Rebuild affected tiles
navmesh.rebuild_dirty_tiles()?;

// Query the navmesh
let query = navmesh.create_query()?;
```

## Async Updates

Async operations use runtime-agnostic primitives (`async-lock`, `futures-lite`)
and work on all platforms including WASM:

```rust
// Works with any async runtime (Tokio, async-std, wasm-bindgen-futures, etc.)
navmesh.build_async().await?;
navmesh.update_async().await?;
```

On native platforms with Tokio:

```rust
use tokio::runtime::Runtime;

let rt = Runtime::new()?;
rt.block_on(async {
    navmesh.build_async().await?;
    Ok::<_, Box<dyn std::error::Error>>(())
})?;
```

## WASM Support

This crate is fully compatible with WebAssembly. Build for WASM with:

```bash
cargo build --target wasm32-unknown-unknown -p detour-dynamic
```

Async operations work on WASM using `wasm-bindgen-futures` or similar runtimes.
The `tokio` feature is not available on WASM.

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
