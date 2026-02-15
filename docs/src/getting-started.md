# Getting Started

## Installation

Add the crates you need to your `Cargo.toml`:

```toml
[dependencies]
recast = "0.1"
detour = "0.1"
```

Optional crates:

```toml
[dependencies]
detour-crowd = "0.1"       # Multi-agent crowd simulation
detour-tilecache = "0.1"   # Dynamic obstacle management
detour-dynamic = "0.1"     # Dynamic navmesh generation
```

## Feature Flags

| Crate | Feature | Dependencies | Default |
|-------|---------|--------------|---------|
| recast-common | `std` | standard library | Yes |
| detour | `serialization` | serde, serde_json, postcard, byteorder | No |
| detour-tilecache | `serialization` | serde, serde_json, postcard | No |
| detour-dynamic | `tokio` | tokio (spawn_blocking) | No |

Enable features in `Cargo.toml`:

```toml
[dependencies]
detour = { version = "0.1", features = ["serialization"] }
```

## Basic Usage

The typical workflow is:

1. Load or define input geometry (triangle mesh)
2. Generate a navigation mesh with Recast
3. Query paths with Detour

```rust,ignore
use recast::{RecastBuilder, RecastConfig};
use detour::{NavMesh, NavMeshQuery, QueryFilter};

// Configure the navigation mesh generation
let config = RecastConfig::default();
let builder = RecastBuilder::new(config);

// Build from triangle data (flat f32 arrays: [x,y,z, x,y,z, ...])
let (poly_mesh, detail_mesh) = builder.build_mesh(&vertices, &indices)?;

// Create a NavMesh for pathfinding
let nav_mesh = NavMesh::from_poly_mesh(&poly_mesh, &detail_mesh)?;

// Find a path
let mut query = NavMeshQuery::new(&nav_mesh);
let filter = QueryFilter::default();
let path = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &filter)?;
```

## RecastConfig Parameters

The `RecastConfig` struct controls the navigation mesh generation. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cs` | 0.3 | Cell size (horizontal voxel resolution) |
| `ch` | 0.2 | Cell height (vertical voxel resolution) |
| `walkable_slope_angle` | 45.0 | Maximum walkable slope in degrees |
| `walkable_height` | 2 | Minimum ceiling height (in voxels) |
| `walkable_climb` | 1 | Maximum step height (in voxels) |
| `walkable_radius` | 1 | Agent radius for erosion (in voxels) |
| `max_edge_len` | 12 | Maximum contour edge length |
| `max_simplification_error` | 1.3 | Maximum contour simplification error |
| `min_region_area` | 8 | Minimum region size (in voxels) |
| `merge_region_area` | 20 | Region merge threshold |
| `max_vertices_per_polygon` | 6 | Maximum vertices per polygon |
| `detail_sample_dist` | 6.0 | Detail mesh sampling distance |
| `detail_sample_max_error` | 1.0 | Detail mesh max height error |

Smaller `cs` and `ch` values produce more accurate meshes but increase memory
usage and build time. `walkable_height`, `walkable_climb`, and `walkable_radius`
define the agent's physical properties in voxel units.

## Platform Support

- Linux: x86_64 (glibc, musl), aarch64 (glibc, musl), armv7 (glibc, musl)
- macOS: aarch64, x86_64
- Windows: x86_64 (MSVC, GNU)
- WebAssembly: wasm32-unknown-unknown (library crates only)

See the [WebAssembly guide](guides/wasm.md) for WASM-specific instructions.
