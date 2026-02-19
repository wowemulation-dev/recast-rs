# detour

Pathfinding and navigation queries on navigation meshes.

**Lines**: ~24,000 | **WASM**: Yes | **Depends on**: `recast-common`, `recast`

## Overview

The `detour` crate is the largest in the workspace. It provides the NavMesh
data structure, A* pathfinding, spatial queries, and serialization. This crate
has 15 dedicated test modules.

## Main Types

### NavMesh

Multi-tile navigation mesh with BVH spatial indexing and off-mesh connections.

```rust,ignore
use detour::{NavMesh, NavMeshParams, NavMeshFlags};

let params = NavMeshParams {
    origin: [0.0, 0.0, 0.0],
    tile_width: 100.0,
    tile_height: 100.0,
    max_tiles: 16,
    max_polys_per_tile: 256,
};

let nav_mesh = NavMesh::build_from_recast(
    params, &poly_mesh, &detail_mesh, NavMeshFlags::empty()
)?;
```

Key components:
- **`MeshTile`**: Individual tile containing polygons, vertices, and BVH
- **`Poly`**: Navigation polygon with flags and area type
- **`BVNode`**: Bounding volume hierarchy node for spatial queries
- **`Link`**: Connection between polygons (same tile or cross-tile)
- **`OffMeshConnection`**: Links between non-adjacent polygons

### NavMeshQuery

Pathfinding and spatial query engine.

```rust,ignore
use detour::{NavMeshQuery, QueryFilter};

let mut query = NavMeshQuery::new(&nav_mesh);
let filter = QueryFilter::default();

// Find nearest polygon to a world position
let (poly_ref, closest_pos) = query.find_nearest_poly(pos, extents, &filter)?;

// Find a path (returns polygon references)
let path = query.find_path(start_ref, end_ref, start_pos, end_pos, &filter)?;

// Convert to waypoints via funnel algorithm
let straight_path = query.find_straight_path(start_pos, end_pos, &path)?;
```

Available queries:
- `find_path()`: A* pathfinding returning polygon references
- `find_straight_path()`: Funnel algorithm converting polygon path to waypoints
- `find_nearest_poly()`: Find closest polygon to a world position
- `find_polys_around_circle()`: Find polygons within a radius
- `find_random_point()`: Random point on the navmesh
- `find_distance_to_wall()`: Distance to nearest navmesh boundary
- `move_along_surface()`: Constrained movement along the navmesh
- `raycast()`: Line-of-sight query against the navmesh

### Sliced Pathfinding

For distributing A* work across frames:

```rust,ignore
use detour::SlicedPathfindingQuery;

let mut sliced = SlicedPathfindingQuery::new();
sliced.init_sliced_find_path(&query, start_ref, end_ref, &start, &end, &filter)?;

// Process a limited number of iterations per frame
loop {
    let status = sliced.update_sliced_find_path(&query, 10)?;
    if status.is_complete() {
        break;
    }
}

let path = sliced.finalize_sliced_find_path(&query)?;
```

### Hierarchical Pathfinding

Cluster-based pathfinding for large meshes:

- `HierarchicalPathfinder`: Cluster-level pathfinding
- `HierarchicalConfig`: Cluster size and configuration
- `Cluster`, `Portal`, `ClusterConnection`: Graph structures

### QueryFilter

Controls polygon traversability and costs:

```rust,ignore
use detour::{QueryFilter, PolyFlags};

let mut filter = QueryFilter::default();
filter.include_flags = PolyFlags::WALK | PolyFlags::SWIM;
filter.exclude_flags = PolyFlags::DISABLED;
filter.area_cost[0] = 1.0;  // Ground
filter.area_cost[1] = 2.0;  // Shallow water
filter.area_cost[2] = 5.0;  // Deep water
```

### PolyRef

Polygon reference encoding salt, tile index, and polygon index:

```rust,ignore
use detour::{PolyRef, encode_poly_ref, decode_poly_ref};

let poly_ref = encode_poly_ref(salt, tile_index, poly_index);
let (salt, tile, poly) = decode_poly_ref(poly_ref);
```

### PolyFlags

Bitflags describing polygon attributes: `WALK`, `SWIM`, `DOOR`, `JUMP`,
`DISABLED`, `CLIMB`.

## Feature Flags

| Feature | Default | Description |
|---------|---------|-------------|
| `serialization` | No | Save/load NavMesh (JSON, binary, postcard) |

### Serialization Formats

With the `serialization` feature enabled:

- **Binary**: C++ compatible format via `binary_format` module
- **JSON**: Human-readable via serde_json
- **Postcard**: Compact binary via postcard (no_std compatible)

```rust,ignore
// Save
nav_mesh.save_to_binary("mesh.bin")?;
nav_mesh.save_to_json("mesh.json")?;

// Load
let mesh = NavMesh::load_from_binary("mesh.bin")?;
let mesh = NavMesh::load_from_json("mesh.json")?;
```
