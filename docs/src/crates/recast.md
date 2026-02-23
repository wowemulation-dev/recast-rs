# landmark

Navigation mesh generation from input triangle meshes.

**Lines**: ~13,400 | **WASM**: Yes | **Depends on**: `landmark-common`

## Overview

The `landmark` crate implements the full navmesh generation pipeline. It takes
input triangle geometry and produces polygon meshes for pathfinding.

See [Processing Pipeline](../architecture/pipeline.md) for the stage-by-stage
description.

## Main Types

### RecastBuilder

Entry point for navmesh generation. Drives the full pipeline.

```rust,ignore
use landmark::{RecastBuilder, RecastConfig};

let config = RecastConfig::default();
let builder = RecastBuilder::new(config);
let (poly_mesh, detail_mesh) = builder.build_mesh(&vertices, &indices)?;
```

Available build methods:

- `build_mesh()`: Full pipeline from triangles to polygon mesh
- `build_heightfield()`: Build only the heightfield stage
- `build_layered_heightfield()`: Multi-story heightfield generation
- `build_mesh_with_volumes()`: Build with convex volume area overrides
- `build_and_merge_from_multiple_inputs()`: Build and merge multiple meshes

### RecastConfig

Configuration parameters for mesh generation. Defaults are suitable for
human-sized agents at 1 unit = 1 meter scale. See
[Getting Started](../getting-started.md) for parameter descriptions.

### Pipeline Stages

| Type | Description |
|------|-------------|
| `Heightfield` | Voxel grid with vertical spans |
| `CompactHeightfield` | Compressed spans with neighbor connections |
| `ContourSet` | Walkable region boundary polygons |
| `PolyMesh` | Navigation polygon mesh with adjacency |
| `PolyMeshDetail` | Height detail mesh for accurate sampling |
| `LayeredHeightfield` | Multi-story heightfield layers |

### Area Marking

Functions for modifying walkable areas in the heightfield:

- `mark_box_area()`: Mark area inside an axis-aligned box
- `mark_cylinder_area()`: Mark area inside a cylinder
- `mark_convex_poly_area()`: Mark area inside a convex polygon
- `erode_walkable_area()`: Shrink walkable area by agent radius
- `median_filter_walkable_area()`: Smooth area boundaries

### Region Building

Three partitioning algorithms:

- `build_regions_watershed()`: Best region shapes, flood-fill based
- `build_regions_monotone()`: Faster, axis-aligned regions
- `build_layer_regions()`: Height-based partitioning for multi-story

### Mesh Merging

- `MeshMerger`: Merges multiple `PolyMesh` instances
- `MeshMergeConfig`: Configuration for merge behavior
- `copy_poly_mesh()`: Deep copy a polygon mesh

### Context

- `RecastContext`: Logging and performance timing
- `LogLevel`: Debug, Warning, Error
- `TimerCategory`: Performance categories for profiling
