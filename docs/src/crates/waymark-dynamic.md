# waymark-dynamic

Dynamic navigation mesh generation with async support.

**Lines**: ~6,000 | **WASM**: Yes | **Depends on**: `landmark-common`, `landmark`, `waymark`, `waymark-tilecache`

## Overview

The `waymark-dynamic` crate extends beyond the C++ original. It provides
runtime navmesh modification through colliders, async tile rebuilding, and
a checkpoint system for incremental updates.

## Main Types

### DynamicNavMesh

The main entry point for dynamic navmesh management.

```rust,ignore
use waymark_dynamic::{DynamicNavMesh, DynamicNavMeshConfig};
use waymark_dynamic::colliders::BoxCollider;
use glam::Vec3;
use std::sync::Arc;

let config = DynamicNavMeshConfig {
    world_min: Vec3::new(-20.0, -1.0, -20.0),
    world_max: Vec3::new(20.0, 5.0, 20.0),
    cell_size: 0.3,
    cell_height: 0.2,
    walkable_height: 2.0,
    walkable_radius: 0.6,
    walkable_climb: 0.9,
    walkable_slope_angle: 45.0,
    ..Default::default()
};

let mut dynamic_navmesh = DynamicNavMesh::with_tile_grid(config, 3, 3)?;

// Add a collider
let collider = Arc::new(BoxCollider::new(
    Vec3::new(5.0, 0.0, 5.0),
    Vec3::new(1.0, 2.0, 1.0),
    0,    // area type
    1.0,  // flag merge threshold
));
dynamic_navmesh.add_collider(collider)?;

// Rebuild affected tiles
dynamic_navmesh.update()?;
```

### Collider Types

Seven collider types implement the `Collider` trait:

| Type | Description |
|------|-------------|
| `BoxCollider` | Axis-aligned or oriented box |
| `CylinderCollider` | Vertical cylinder |
| `SphereCollider` | Sphere |
| `CapsuleCollider` | Capsule (cylinder with hemispherical caps) |
| `TrimeshCollider` | Arbitrary triangle mesh |
| `ConvexCollider` | Convex triangle mesh |
| `CompositeCollider` | Group of multiple colliders |

All colliders define an axis-aligned bounding box for spatial queries and a
method to rasterize into heightfield tiles.

### Checkpoint System

Save and restore tile state for incremental rebuilds:

```rust,ignore
use waymark_dynamic::CheckpointManager;

let mut checkpoint_mgr = CheckpointManager::new();

// Save checkpoint
let checkpoint = checkpoint_mgr.create_checkpoint(&dynamic_navmesh)?;

// Restore checkpoint (only dirty tiles are rebuilt)
checkpoint_mgr.restore_checkpoint(&mut dynamic_navmesh, &checkpoint)?;
```

### VoxelQuery

Raycasting against heightfield data:

```rust,ignore
use waymark_dynamic::VoxelQuery;

let voxel_query = VoxelQuery::from_single_heightfield(origin, tile_width, tile_depth);

if let Some(hit) = voxel_query.raycast(start, end) {
    println!("Hit at {:?}", hit.position);
}
```

### Job System

Async collider operations via `async-lock` and `futures-lite` (runtime-agnostic,
works on WASM):

- `ColliderAdditionJob`: Async collider addition
- `ColliderRemovalJob`: Async collider removal
- `JobProcessor`: Processes pending jobs

### I/O

- `VoxelFile`: Read/write voxel data with LZ4 compression
- `VoxelTile`: Individual tile voxel data

## Feature Flags

| Feature | Default | Description |
|---------|---------|-------------|
| `tokio` | No | Tokio runtime integration (not WASM-compatible) |

Without the `tokio` feature, async operations use `async-lock` and
`futures-lite`, which are runtime-agnostic and WASM-compatible.
