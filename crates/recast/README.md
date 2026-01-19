# recast

Navigation mesh generation from 3D triangle meshes.

[![Crates.io](https://img.shields.io/crates/v/recast.svg)](https://crates.io/crates/recast)
[![Documentation](https://docs.rs/recast/badge.svg)](https://docs.rs/recast)
[![License](https://img.shields.io/crates/l/recast.svg)](../LICENSE-MIT)

## Overview

Recast generates navigation meshes from input geometry. It voxelizes triangle
meshes into heightfields, filters non-walkable areas, and produces polygon
meshes suitable for pathfinding.

This is a Rust port of the Recast component from [RecastNavigation][recast-cpp].

[recast-cpp]: https://github.com/recastnavigation/recastnavigation

## Features

- **Voxelization**: Convert triangle meshes to solid heightfields
- **Area Marking**: Define walkable, non-walkable, and custom area types
- **Region Building**: Watershed and monotone partitioning algorithms
- **Contour Generation**: Extract walkable region boundaries
- **Polygon Mesh**: Generate navigation polygon meshes
- **Detail Mesh**: Create detailed height meshes for accurate positioning

## Optional Features

- `parallel` - Multi-threaded mesh generation using Rayon

## Example

```rust
use recast::{Heightfield, CompactHeightfield, ContourSet, PolyMesh, RecastConfig};

// Configure the build
let config = RecastConfig {
    cell_size: 0.3,
    cell_height: 0.2,
    walkable_slope_angle: 45.0,
    walkable_height: 2.0,
    walkable_climb: 0.9,
    walkable_radius: 0.6,
    ..Default::default()
};

// Build the navmesh pipeline
let mut heightfield = Heightfield::new(&config, &bounds_min, &bounds_max)?;
heightfield.rasterize_triangles(&vertices, &indices, area_ids)?;

let compact = CompactHeightfield::build(&heightfield, &config)?;
let contours = ContourSet::build(&compact, &config)?;
let poly_mesh = PolyMesh::build(&contours, &config)?;
```

## Pipeline Stages

1. **Heightfield** - Voxelize input triangles
2. **CompactHeightfield** - Compress and prepare for region building
3. **ContourSet** - Extract region contours
4. **PolyMesh** - Generate navigation polygons
5. **PolyMeshDetail** - Add height detail (optional)

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
