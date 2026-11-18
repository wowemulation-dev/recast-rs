# landmark

Navigation mesh generation from 3D triangle meshes.

[![Crates.io](https://img.shields.io/crates/v/landmark.svg)](https://crates.io/crates/landmark)
[![Documentation](https://docs.rs/landmark/badge.svg)](https://docs.rs/landmark)
[![License](https://img.shields.io/crates/l/landmark.svg)](../LICENSE-MIT)
[![WASM](https://img.shields.io/badge/WASM-compatible-green.svg)](https://webassembly.org/)

## Overview

`landmark` generates navigation meshes from input geometry. It voxelizes
triangle meshes into heightfields, filters non-walkable areas, and produces
polygon meshes suitable for pathfinding.

Rust port of the Recast component from [RecastNavigation][recast-cpp].

[recast-cpp]: https://github.com/recastnavigation/recastnavigation

## Features

- **Voxelization**: Convert triangle meshes to solid heightfields
- **Area Marking**: Define walkable, non-walkable, and custom area types
- **Region Building**: Watershed and monotone partitioning algorithms
- **Contour Generation**: Extract walkable region boundaries
- **Polygon Mesh**: Generate navigation polygon meshes
- **Detail Mesh**: Create detailed height meshes for accurate positioning

## WASM Support

This crate is fully compatible with WebAssembly. Build for WASM with:

```bash
cargo build --target wasm32-unknown-unknown -p landmark
```

Timing uses `web-time` for cross-platform compatibility.

## Example

```rust,ignore
use landmark::{RecastBuilder, RecastConfig};

// Configure the build
let mut config = RecastConfig::default();
config.cs = 0.3;
config.ch = 0.2;
config.walkable_slope_angle = 45.0;
config.walkable_height = 2;   // voxel units (i32)
config.walkable_climb = 1;
config.walkable_radius = 1;
config.calculate_grid_size(bounds_min, bounds_max);

// Build the full pipeline in one call
let builder = RecastBuilder::new(config);
let (poly_mesh, detail_mesh) = builder.build_mesh(&vertices, &indices)?;
```

## Pipeline Stages

1. **Heightfield** - Voxelize input triangles
2. **CompactHeightfield** - Compress and prepare for region building
3. **ContourSet** - Extract region contours
4. **PolyMesh** - Generate navigation polygons
5. **PolyMeshDetail** - Add height detail (optional)

## Test Data

Test meshes in `../../test-data/meshes/`:

| File | Vertices | Triangles |
|------|----------|-----------|
| `nav_test.obj` | 884 | 1612 |
| `dungeon.obj` | 5101 | 10133 |
| `bridge.obj` | 29 | 54 |

Reference output in `../../test-data/reference/` contains expected JSON values for validation.

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
