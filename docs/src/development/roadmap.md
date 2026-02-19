# Resolution Roadmap

Items are ordered by priority: critical blockers first, then high-severity
issues, then improvements.

## Phase 1: Library Safety (Critical) -- COMPLETE

These issues must be resolved before crates.io publication.

### 1.1 Eliminate `unwrap()`/`expect()` from Library Code -- COMPLETE

> **Status**: Reduced from 45 to 2. The 2 remaining are in detour-dynamic
> job processing (`collider_removal_job.rs`, `dynamic_tile_job.rs`).

**Problem**: 45 `unwrap()`/`expect()` calls in non-test library code
(verified count: 20 in detour, 13 in detour-tilecache, 9 in recast, 2 in
detour-crowd, 1 doctest in recast-common). A library that panics on
recoverable errors is not usable.

**Approach**:

- Audit every `unwrap()`/`expect()` call in non-test code
- Categorize each as: (a) provably infallible, (b) should return `Result`,
  or (c) should use a default value
- For category (a): replace with `unwrap_or_else(|| unreachable!())` or
  restructure to avoid the `Option`/`Result` entirely
- For category (b): propagate the error with `?`
- For category (c): use `unwrap_or()`, `unwrap_or_default()`, or
  `unwrap_or_else()`

**Priority by crate** (based on risk, not just count):

1. `detour-tilecache` -- 13 panics on resource exhaustion in `tile_cache.rs`
2. `detour` -- 20 calls, includes A\* open list pop at `nav_mesh_query.rs:438`
3. `recast` -- 9 calls, includes cell index unwraps in `watershed.rs`
4. `detour-crowd` -- 2 calls
5. `recast-common` -- 1 doctest in `mesh.rs`

**Verification**: `grep -rn 'unwrap()\|expect(' crates/*/src/ --include='*.rs'`
filtered to exclude `#[cfg(test)]` modules should return zero results.

### 1.2 Add Structured Error Types -- COMPLETE

> **Status**: Per-crate error types implemented. The old catch-all `Error`
> enum has been removed from recast-common. Each crate owns its errors:
> `MeshError`, `ConfigError`/`BuildError`/`ConvexVolumeError`, `DetourError`,
> `CrowdError`, `TileCacheError`, `DynamicError`.

**Problem**: The workspace `Error` enum in `recast-common/src/lib.rs` has
6 variants. 5 of 6 use bare `String` as payload. The `Pathfinding` variant
is unused (zero occurrences in the codebase). The `detour` crate has a
well-designed `Status` enum with 22 variants (`InvalidParam`, `OutOfMemory`,
`PathInvalid`, etc.) but converts these to strings via `.to_string()` before
wrapping in `Error::Detour(String)`. This pattern appears 242 times,
destroying type information that callers need for error handling.

**Error site counts (non-test code)**:

| Pattern | Count | Location |
|---------|-------|----------|
| `Error::Detour(Status::InvalidParam.to_string())` | 159 | detour, detour-crowd, detour-tilecache |
| `Error::Detour(Status::Failure.to_string())` | 45 | detour, detour-crowd, detour-tilecache |
| `Error::NavMeshGeneration(String)` | 26 | recast |
| `Error::InvalidMesh(String)` | 23 | recast, detour-crowd, detour-tilecache |
| `Error::Detour(Status::NotFound.to_string())` | 13 | detour, detour-tilecache |
| `Error::Detour(Status::PathInvalid.to_string())` | 7 | detour, detour-crowd |
| `Error::Detour(Status::OutOfMemory.to_string())` | 7 | detour, detour-tilecache |
| `Error::Recast(String)` | 7 | recast, detour-dynamic |
| `Error::Detour(Status::WrongVersion.to_string())` | 4 | detour |
| `Error::Detour(Status::WrongMagic.to_string())` | 3 | detour |
| `Error::Detour(Status::BufferTooSmall.to_string())` | 3 | detour |
| `Error::Detour(Status::InProgress.to_string())` | 1 | detour |
| `Error::Detour(ad-hoc string)` | 3 | detour (nav_mesh.rs, nav_mesh_query.rs) |
| `Error::Pathfinding(String)` | 0 | unused |
| **Total** | **301** | |

#### Approach

Each crate defines its own error type. The workspace `Error` in
`recast-common` is removed. Functions return crate-specific
`Result<T, CrateError>`.

#### Step 1: `recast-common` -- Remove catch-all Error

Delete the current `Error` enum. Replace with:

```rust
// recast-common/src/error.rs

/// Error for mesh I/O operations (std-only)
#[derive(thiserror::Error, Debug)]
pub enum MeshError {
    #[error("vertex array length {len} is not a multiple of 3")]
    VertexArrayNotTripled { len: usize },

    #[error("index array length {len} is not a multiple of 3")]
    IndexArrayNotTripled { len: usize },

    #[error("triangle index out of bounds: ({i0}, {i1}, {i2}), vertex count: {vertex_count}")]
    TriangleIndexOutOfBounds {
        i0: usize,
        i1: usize,
        i2: usize,
        vertex_count: usize,
    },

    #[cfg(feature = "std")]
    #[error(transparent)]
    Io(#[from] std::io::Error),
}
```

Remove: `InvalidMesh(String)`, `NavMeshGeneration(String)`,
`Pathfinding(String)`, `Recast(String)`, `Detour(String)`.

#### Step 2: `recast` -- Per-stage error types

The 58 error sites in the recast crate fall into these categories:

```rust
// recast/src/error.rs

/// Error during recast configuration validation
#[derive(thiserror::Error, Debug)]
pub enum ConfigError {
    #[error("grid dimensions out of range: width={width}, height={height}")]
    InvalidGridSize { width: i32, height: i32 },

    #[error("cell size or height must be positive: cs={cs}, ch={ch}")]
    InvalidCellDimensions { cs: f32, ch: f32 },

    #[error("walkable slope angle out of range: {angle}")]
    InvalidWalkableSlope { angle: f32 },

    #[error("max vertices per polygon must be >= 3, got {count}")]
    TooFewVertsPerPoly { count: i32 },
}

/// Error during navmesh generation
#[derive(thiserror::Error, Debug)]
pub enum BuildError {
    #[error(transparent)]
    Config(#[from] ConfigError),

    // Capacity limits
    #[error("too many vertices: {count} (max: {max})")]
    TooManyVertices { count: usize, max: usize },

    #[error("too many polygons: {count} (max: {max})")]
    TooManyPolygons { count: usize, max: usize },

    #[error("too many edges")]
    TooManyEdges,

    #[error("region ID overflow")]
    RegionIdOverflow,

    #[error("layer overflow at ({x}, {y}): too many overlapping walkable platforms")]
    LayerOverflow { x: i32, y: i32 },

    // Grid/cell errors
    #[error("span position out of bounds: ({x}, {y})")]
    SpanOutOfBounds { x: i32, y: i32 },

    #[error("invalid span height: min ({min}) > max ({max})")]
    InvalidSpanHeight { min: u32, max: u32 },

    #[error("cell not found: ({x}, {y})")]
    CellNotFound { x: i32, y: i32 },

    #[error("cell index out of bounds: {index}")]
    CellIndexOutOfBounds { index: usize },

    // Polygon/triangle errors
    #[error("polygon must have at least 3 vertices")]
    DegeneratePolygon,

    #[error("cannot merge empty mesh list")]
    EmptyMeshList,

    #[error("incompatible mesh at index {index}: {detail}")]
    IncompatibleMesh { index: usize, detail: &'static str },

    #[error("scale factors must be positive")]
    InvalidScaleFactors,
}

/// Error during convex volume creation
#[derive(thiserror::Error, Debug)]
pub enum ConvexVolumeError {
    #[error("requires at least {min} vertices, got {count}")]
    TooFewVertices { count: usize, min: usize },

    #[error("too many vertices: {count} (max: {max})")]
    TooManyVertices { count: usize, max: usize },

    #[error("min_height ({min}) exceeds max_height ({max})")]
    InvalidHeight { min: f32, max: f32 },

    #[error("vertices do not form a convex polygon")]
    NotConvex,
}
```

**Migration**: Each `Error::NavMeshGeneration(format!(...))` becomes a
specific `BuildError` variant. Each `Error::InvalidMesh(...)` in config.rs
becomes a `ConfigError` variant. Each `Error::InvalidMesh(...)` in
convex_volume.rs becomes a `ConvexVolumeError` variant.

#### Step 3: `detour` -- Promote Status to error type

The `Status` enum already has the right categories. Make it implement
`std::error::Error` and use it directly:

```rust
// detour/src/error.rs
use crate::status::Status;

/// Error from detour operations
#[derive(thiserror::Error, Debug)]
pub enum DetourError {
    #[error("invalid parameter")]
    InvalidParam,

    #[error("operation failed")]
    Failure,

    #[error("out of memory")]
    OutOfMemory,

    #[error("path not found")]
    PathNotFound,

    #[error("not found")]
    NotFound,

    #[error("buffer too small")]
    BufferTooSmall,

    #[error("query in progress")]
    InProgress,

    #[error("wrong magic number")]
    WrongMagic,

    #[error("wrong version")]
    WrongVersion,

    #[error("data corrupted")]
    DataCorrupted,

    #[error("navmesh build failed")]
    Build(#[from] recast::BuildError),

    #[cfg(feature = "serialization")]
    #[error("serialization failed: {0}")]
    Serialization(#[source] Box<dyn std::error::Error + Send + Sync>),

    #[cfg(feature = "serialization")]
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

impl From<Status> for DetourError {
    fn from(status: Status) -> Self {
        match status {
            Status::InvalidParam => DetourError::InvalidParam,
            Status::OutOfMemory => DetourError::OutOfMemory,
            Status::PathInvalid => DetourError::PathNotFound,
            Status::NotFound => DetourError::NotFound,
            Status::BufferTooSmall => DetourError::BufferTooSmall,
            Status::InProgress => DetourError::InProgress,
            Status::WrongMagic => DetourError::WrongMagic,
            Status::WrongVersion => DetourError::WrongVersion,
            Status::DataCorrupted => DetourError::DataCorrupted,
            _ => DetourError::Failure,
        }
    }
}
```

**Migration**: Every `Error::Detour(Status::X.to_string())` becomes
`DetourError::from(Status::X)` or just `DetourError::X` directly. The 3
ad-hoc strings in nav_mesh.rs and nav_mesh_query.rs map to existing variants:

- `"Polygon not found in tile"` -> `DetourError::NotFound`
- `"End node not found in explored nodes"` -> `DetourError::NotFound`
- `"Invalid source polygon index"` -> `DetourError::InvalidParam`

The `Serialization` variant wraps serde/postcard errors that are currently
discarded by `.map_err(|_| Error::Detour(Status::Failure.to_string()))`.
This recovers the original error information.

#### Step 4: `detour-crowd` -- Crowd-specific error type

```rust
// detour-crowd/src/error.rs

#[derive(thiserror::Error, Debug)]
pub enum CrowdError {
    #[error("invalid parameter")]
    InvalidParam,

    #[error("agent not found: {index}")]
    AgentNotFound { index: usize },

    #[error("path corridor failed")]
    CorridorFailed,

    #[error("RVO computation failed: {0}")]
    Rvo(&'static str),

    #[error(transparent)]
    Detour(#[from] detour::DetourError),
}
```

**Migration**: The 2 `Error::InvalidMesh(...)` calls in rvo.rs become
`CrowdError::Rvo(...)`. The `Error::Detour(Status::InvalidParam...)` calls
become `CrowdError::InvalidParam`.

#### Step 5: `detour-tilecache` -- TileCache-specific error type

```rust
// detour-tilecache/src/error.rs

#[derive(thiserror::Error, Debug)]
pub enum TileCacheError {
    #[error("invalid parameter")]
    InvalidParam,

    #[error("out of memory: {resource}")]
    OutOfMemory { resource: &'static str },

    #[error("tile not found: ({x}, {y})")]
    TileNotFound { x: i32, y: i32 },

    #[error("obstacle not found")]
    ObstacleNotFound,

    #[error("invalid region data size")]
    InvalidRegionData,

    #[error("invalid area data size")]
    InvalidAreaData,

    #[error(transparent)]
    Detour(#[from] detour::DetourError),

    #[cfg(feature = "serialization")]
    #[error("serialization failed: {0}")]
    Serialization(#[source] Box<dyn std::error::Error + Send + Sync>),

    #[cfg(feature = "serialization")]
    #[error(transparent)]
    Io(#[from] std::io::Error),
}
```

**Migration**: The 13 `unwrap()` calls on free list exhaustion become
`TileCacheError::OutOfMemory { resource: "tiles" }` or
`"obstacles"`. The 2 `Error::InvalidMesh(...)` calls in
tile_cache_builder.rs become `InvalidRegionData` and `InvalidAreaData`.

#### Step 6: `detour-dynamic` -- Dynamic-specific error type

```rust
// detour-dynamic/src/error.rs

#[derive(thiserror::Error, Debug)]
pub enum DynamicError {
    #[error("invalid span data at cell ({x}, {y}): {detail}")]
    InvalidSpanData { x: i32, y: i32, detail: String },

    #[error("invalid partition type")]
    InvalidPartitionType,

    #[error("job queue full")]
    JobQueueFull,

    #[error(transparent)]
    Config(#[from] recast::ConfigError),

    #[error(transparent)]
    Build(#[from] recast::BuildError),

    #[error(transparent)]
    Detour(#[from] detour::DetourError),

    #[error(transparent)]
    Io(#[from] std::io::Error),
}
```

**Migration**: The 4 span parsing errors in dynamic_tile.rs become
`DynamicError::InvalidSpanData { ... }`. The 2 job queue errors become
`DynamicError::JobQueueFull`. The config validation call becomes
`DynamicError::Config(...)`.

#### Execution order

1. Define new error types in each crate (add `error.rs` modules)
2. Update function signatures crate by crate, bottom-up:
   recast-common -> recast -> detour -> detour-crowd -> detour-tilecache ->
   detour-dynamic
3. Delete old `Error` enum from recast-common
4. Update recast-cli to handle new error types (use `anyhow` to collect)

#### Verification

```bash
# No String-only error variants remain
grep -rn 'Error::.*String)' crates/*/src/ --include='*.rs' | \
  grep -v '#\[cfg(test)\]' | grep -v 'mod tests' | grep -v '///'

# No .to_string() on Status
grep -rn 'Status::.*\.to_string()' crates/*/src/ --include='*.rs' | \
  grep -v '#\[cfg(test)\]' | grep -v 'mod tests'

# No unused Pathfinding variant
grep -rn 'Pathfinding' crates/*/src/ --include='*.rs'

# All tests pass
cargo fmt --all && cargo lint && cargo test-all
```

### 1.3 Enable `-D warnings` Locally -- COMPLETE

> **Status**: Enabled in `.cargo/config.toml`. All warnings resolved.

**Problem**: `rustflags = ["-D", "warnings"]` was commented out in
`.cargo/config.toml`. CI enforces this via environment variable, but local
development allowed warnings to accumulate.

**Action**: Uncommented the line in `.cargo/config.toml`. Fixed all resulting
warnings.

## Phase 2: Usability (High) -- MOSTLY COMPLETE

These issues block practical adoption. Sections 2.1-2.3 are complete.
Section 2.4 (crates.io publication) is pending.

### 2.1 Add Worked Examples -- COMPLETE

> **Status**: 5 examples implemented in `examples/examples/`:
> `basic_navmesh.rs`, `pathfinding.rs`, `crowd_simulation.rs`,
> `tilecache_obstacles.rs`, `serialization.rs`. Shared helpers in
> `examples/src/common.rs`.

**Problem**: Zero examples. Users cannot evaluate the library without writing
code from scratch. rerecast ships 4 examples (Bevy integrations). DotRecast
ships a full interactive demo. recast-navigation-js has 5 example projects
and 22 Storybook stories.

#### Structure

Examples live in a workspace-level `examples/` crate to avoid adding
dependencies to individual library crates:

```text
examples/
  Cargo.toml          # [[example]] entries, depends on all workspace crates
  data/
    nav_test.obj      # 113K, from C++ RecastDemo (same file used by DotRecast)
  src/
    common.rs         # Shared helpers: build_navmesh_from_obj(), print_stats()
  examples/
    basic_navmesh.rs
    pathfinding.rs
    crowd_simulation.rs
    tilecache_obstacles.rs
    serialization.rs
```

Add `"examples"` to the workspace `members` list in root `Cargo.toml`.

The `examples/Cargo.toml` depends on workspace crates:

```toml
[package]
name = "recast-rs-examples"
version.workspace = true
edition.workspace = true
publish = false

[dependencies]
recast-common = { workspace = true }
recast = { workspace = true }
detour = { workspace = true, features = ["serialization"] }
detour-crowd = { workspace = true }
detour-tilecache = { workspace = true, features = ["serialization"] }
glam = { workspace = true }

[[example]]
name = "basic_navmesh"
path = "examples/basic_navmesh.rs"

# ... one entry per example
```

#### Example 1: `basic_navmesh.rs`

Demonstrates the Recast generation pipeline. No pathfinding.

**API calls** (in order):

1. `TriMesh::from_obj("examples/data/nav_test.obj")` -- load input geometry
2. `mesh.calculate_bounds()` -- get AABB for config
3. `RecastConfig { cs: 0.3, ch: 0.2, walkable_slope_angle: 45.0, ... }`
4. `config.calculate_grid_size(bmin, bmax)` -- compute grid dimensions
5. `RecastBuilder::new(config).build_mesh(&mesh.vertices, &mesh.indices)`
   -- returns `(PolyMesh, PolyMeshDetail)`
6. Print: grid size, vertex count, polygon count, build time

**Expected output** (approximate, for `nav_test.obj` with default config):

```text
Loaded mesh: 1713 vertices, 3424 triangles
Grid size: 78x67
Built navmesh: ~200 vertices, ~100 polygons
```

#### Example 2: `pathfinding.rs`

Demonstrates end-to-end: OBJ -> navmesh -> path -> waypoints.

**API calls** (in order):

1. Build navmesh using the same flow as Example 1
2. `NavMeshParams { origin, tile_width, tile_height, max_tiles: 1, ... }`
3. `NavMesh::build_from_recast(params, &poly_mesh, &detail, NavMeshFlags::empty())`
4. `NavMeshQuery::new(&nav_mesh)`
5. `query.find_nearest_poly(&start_pos, &extent, &filter)` -- snap to mesh
6. `query.find_nearest_poly(&end_pos, &extent, &filter)` -- snap to mesh
7. `query.find_path(start_ref, end_ref, &start, &end, &filter)` -- A\* search
8. `query.find_straight_path(&start, &end, &path)` -- funnel algorithm
9. Print: polygon path length, waypoint coordinates

**Points to demonstrate**:
- `QueryFilter::default()` for basic filtering
- `set_query_extent([2.0, 4.0, 2.0])` for search radius
- How `find_path` returns polygon refs and `find_straight_path` converts
  to world-space waypoints

#### Example 3: `crowd_simulation.rs`

Demonstrates multi-agent crowd simulation.

**API calls** (in order):

1. Build navmesh (reuse helper from `common.rs`)
2. `Crowd::new(&nav_mesh)` -- create crowd manager
3. `AgentParams { radius: 0.6, height: 2.0, max_speed: 3.5, max_acceleration: 8.0, ... }`
4. `crowd.add_agent(position, params)` -- add 5 agents at different positions
5. `crowd.request_move_target(agent_id, target_ref, target_pos)` -- set targets
6. Loop 100 frames:
   - `crowd.update(dt)` with `dt = 1.0/60.0`
   - Every 10 frames: print agent positions and velocities
7. Print: final agent positions, distances to targets

**Points to demonstrate**:
- `UpdateFlags` for enabling/disabling features
- RVO collision avoidance (`crowd.enable_rvo()`)
- Agent state checking (`agent.get_state()`, `agent.is_active()`)

#### Example 4: `tilecache_obstacles.rs`

Demonstrates dynamic obstacle management.

**API calls** (in order):

1. Build navmesh and tilecache
2. `TileCacheParams { origin, cs, ch, width, height, max_obstacles: 32 }`
3. `TileCache::new(params)` + `tile_cache.init(...)` + `attach_to_nav_mesh()`
4. Add cylinder obstacle: `tile_cache.add_obstacle(position, radius, height)`
5. Add box obstacle: `tile_cache.add_box_obstacle(bmin, bmax)`
6. `tile_cache.update()` -- rebuild affected tiles
7. Find path with obstacle, print waypoints (path should route around)
8. `tile_cache.remove_obstacle(obstacle_ref)` -- remove obstacle
9. `tile_cache.update()` -- rebuild
10. Find path again, print waypoints (shorter path without obstacle)

#### Example 5: `serialization.rs`

Demonstrates save/load in multiple formats. Requires `serialization` feature.

**API calls** (in order):

1. Build navmesh (reuse helper)
2. `nav_mesh.to_json_bytes()` -- serialize to JSON bytes
3. `NavMesh::from_json_bytes(&json_bytes)` -- deserialize
4. `nav_mesh.to_binary_bytes()` -- serialize to postcard binary
5. `NavMesh::from_binary_bytes(&binary_bytes)` -- deserialize
6. Print: JSON size, binary size, verify round-trip (compare polygon counts)

**Points to demonstrate**:
- JSON format is human-readable, larger
- Binary (postcard) format is compact, faster
- Both round-trip correctly
- File-based save/load with `save_to_json(path)` / `load_from_json(path)`

#### Shared helpers: `common.rs`

```rust
/// Build a navmesh from an OBJ file with default config.
/// Returns (NavMesh, PolyMesh, PolyMeshDetail) for use in examples.
pub fn build_navmesh_from_obj(path: &str) -> Result<(NavMesh, PolyMesh, PolyMeshDetail)> {
    // 1. Load TriMesh
    // 2. Calculate bounds
    // 3. Create RecastConfig with defaults from CLI tool
    // 4. RecastBuilder::new(config).build_mesh(...)
    // 5. NavMesh::build_from_recast(...)
    // Return all three for examples that need intermediate data
}
```

The default config values match the CLI tool defaults:
`cs=0.3, ch=0.2, walkable_slope_angle=45.0, walkable_height=2,
walkable_climb=1, walkable_radius=1, max_edge_len=12,
max_simplification_error=1.3, min_region_area=8, merge_region_area=20,
max_vertices_per_polygon=6, detail_sample_dist=6.0,
detail_sample_max_error=1.0`.

#### Verification

```bash
cargo run -p recast-rs-examples --example basic_navmesh
cargo run -p recast-rs-examples --example pathfinding
cargo run -p recast-rs-examples --example crowd_simulation
cargo run -p recast-rs-examples --example tilecache_obstacles
cargo run -p recast-rs-examples --example serialization
```

All five must compile and run without errors. Output should include
non-zero polygon counts and valid path waypoints.

### 2.2 Add Test Fixtures -- COMPLETE

> **Status**: Test fixtures exist in `test-data/meshes/` with 3 OBJ files
> (nav_test.obj, dungeon.obj, bridge.obj). Integration tests in
> `crates/recast/tests/` and `crates/detour/tests/` validate against C++
> reference output. 447 tests total (416 unit + 27 integration + 4 tokio).

**Problem**: Tests do not validate against known-good reference output. The
C++ RecastDemo ships 4 test meshes (dungeon.obj, nav_test.obj,
undulating.obj, world.obj). DotRecast ships 6 OBJ meshes, 5 pre-built binary
navmeshes, and 2 voxel files. recast-rs has zero test fixtures.

#### Available reference data

From C++ `RecastDemo/Bin/Meshes/`:

| File | Size | Description |
|------|------|-------------|
| `nav_test.obj` | 113 KB | Simple terrain, 1,713 vertices, 3,424 triangles |
| `dungeon.obj` | 382 KB | Multi-room dungeon, ~6,400 triangles |
| `undulating.obj` | 148 KB | Rolling terrain, ~5,000 triangles |

From DotRecast `resources/` (in addition to the above):

| File | Size | Description |
|------|------|-------------|
| `bridge.obj` | 1.5 KB | Minimal mesh, 87 lines, ~30 triangles |
| `house.obj` | 153 KB | Single building, ~2,600 triangles |
| `convex.obj` | 7.4 KB | Convex shape validation, ~120 triangles |
| `dungeon_all_tiles_navmesh.bin` | 36 KB | Pre-built navmesh from dungeon.obj |
| `all_tiles_navmesh.bin` | 70 KB | Pre-built navmesh from nav_test.obj |

Do **not** include `world.obj` (807 KB) or voxel files (8-11 MB) -- too
large for the repository.

#### Structure

```text
test-data/
  meshes/
    nav_test.obj        # 113 KB -- primary test mesh
    dungeon.obj         # 382 KB -- complex test mesh
    bridge.obj          # 1.5 KB -- minimal test mesh
  reference/
    nav_test.json       # Expected navmesh output (polygon count, bounds, etc.)
    dungeon.json        # Expected navmesh output
    README.md           # How reference data was generated
```

The `reference/` directory contains JSON files with expected values, not
full navmesh dumps. This keeps the files small and version-control friendly.

#### Reference data format

Each `reference/*.json` file contains values generated by running the C++
RecastDemo with default parameters:

```json
{
  "config": {
    "cs": 0.3, "ch": 0.2,
    "walkable_slope_angle": 45.0,
    "walkable_height": 2, "walkable_climb": 1, "walkable_radius": 1
  },
  "input": {
    "vertex_count": 1713,
    "triangle_count": 3424
  },
  "output": {
    "grid_width": 78,
    "grid_height": 67,
    "poly_count": 118,
    "vert_count": 236,
    "detail_vert_count": 472,
    "bounds_min": [0.0, -0.1, 0.0],
    "bounds_max": [23.1, 5.3, 19.9]
  },
  "pathfinding": {
    "start": [5.0, 0.0, 5.0],
    "end": [20.0, 0.0, 15.0],
    "path_poly_count": 12,
    "straight_path_waypoint_count": 8
  }
}
```

**How to generate reference data**:

1. Build C++ RecastDemo from the reference repository
2. Load each mesh with the default parameters listed above
3. Record: grid size, polygon count, vertex count, bounds
4. Run `findPath` between two known points, record polygon count and
   waypoint count
5. Store in `reference/*.json`

Alternatively, run the recast-rs CLI tool and cross-validate against
DotRecast output for the same mesh and parameters. DotRecast test files
(`AbstractDetourTest.cs`, `FindNearestPolyTest.cs`, `FindPathTest.cs`)
contain hardcoded expected values that can be used as ground truth.

#### Integration tests

Create `tests/integration/` in the workspace root (or within relevant
crates) with tests that:

```rust
#[test]
fn test_nav_test_mesh_generation() {
    let mesh = TriMesh::from_obj("../../test-data/meshes/nav_test.obj").unwrap();
    assert_eq!(mesh.vert_count, 1713);
    assert_eq!(mesh.tri_count, 3424);

    let config = default_test_config(&mesh);
    let builder = RecastBuilder::new(config);
    let (poly_mesh, _detail) = builder.build_mesh(&mesh.vertices, &mesh.indices).unwrap();

    // Validate against reference
    let reference: Reference = load_reference("nav_test.json");
    assert_eq!(poly_mesh.poly_count, reference.output.poly_count);
    assert_eq!(poly_mesh.vert_count, reference.output.vert_count);
}

#[test]
fn test_nav_test_pathfinding() {
    let nav_mesh = build_test_navmesh("nav_test.obj");
    let mut query = NavMeshQuery::new(&nav_mesh);
    let filter = QueryFilter::default();

    let (start_ref, _) = query.find_nearest_poly(&[5.0, 0.0, 5.0], &EXTENT, &filter).unwrap();
    let (end_ref, _) = query.find_nearest_poly(&[20.0, 0.0, 15.0], &EXTENT, &filter).unwrap();
    let path = query.find_path(start_ref, end_ref, &[5.0, 0.0, 5.0], &[20.0, 0.0, 15.0], &filter).unwrap();

    let reference: Reference = load_reference("nav_test.json");
    assert_eq!(path.len(), reference.pathfinding.path_poly_count);
}
```

**Tolerance**: Use exact comparison for counts (polygon count, vertex count).
Use `assert_approx_eq` with epsilon 0.01 for floating-point positions. Path
waypoint counts may vary by +/- 1 due to floating-point differences between
C++ and Rust math.

#### Verification

```bash
cargo test -p recast --test integration
cargo test -p detour --test integration
```

### 2.3 Add Benchmarks -- COMPLETE

> **Status**: Benchmark directories exist in `crates/recast/benches/`,
> `crates/detour/benches/`, and `crates/detour-crowd/benches/`. Uses
> criterion. Flamegraph profiling support added via cargo aliases.

**Problem**: No performance data. Cannot measure regressions or compare
against C++ FFI alternatives. DotRecast has BenchmarkDotNet benchmarks for
vector operations and priority queues. The C++ original has
`Bench_rcVector.cpp`.

#### Framework

Use `criterion` (the standard Rust benchmarking framework). Add it as a
workspace dev-dependency:

```toml
# Cargo.toml (workspace)
[workspace.dependencies]
criterion = { version = "0.5", features = ["html_reports"] }
```

#### Structure

Each crate with benchmarks gets a `benches/` directory:

```text
crates/
  recast/
    benches/
      generation.rs     # Heightfield, compact, contour, polymesh
  detour/
    benches/
      pathfinding.rs    # find_path, find_straight_path, sliced
      spatial_queries.rs # find_nearest_poly, raycast, find_distance_to_wall
  detour-crowd/
    benches/
      crowd_update.rs   # crowd.update() with varying agent counts
```

Add to each crate's `Cargo.toml`:

```toml
[dev-dependencies]
criterion = { workspace = true }

[[bench]]
name = "generation"
harness = false
```

#### Benchmark specifications

**`recast/benches/generation.rs`** -- Navmesh generation pipeline:

```rust
use criterion::{criterion_group, criterion_main, Criterion, BenchmarkId};

fn bench_build_mesh(c: &mut Criterion) {
    let mut group = c.benchmark_group("navmesh_generation");

    // Small: bridge.obj (~30 triangles)
    let bridge = load_mesh("bridge.obj");
    group.bench_function("bridge_30tri", |b| {
        b.iter(|| {
            let config = default_config(&bridge);
            RecastBuilder::new(config).build_mesh(&bridge.vertices, &bridge.indices)
        });
    });

    // Medium: nav_test.obj (~3,400 triangles)
    let nav_test = load_mesh("nav_test.obj");
    group.bench_function("nav_test_3400tri", |b| {
        b.iter(|| {
            let config = default_config(&nav_test);
            RecastBuilder::new(config).build_mesh(&nav_test.vertices, &nav_test.indices)
        });
    });

    // Large: dungeon.obj (~6,400 triangles)
    let dungeon = load_mesh("dungeon.obj");
    group.bench_function("dungeon_6400tri", |b| {
        b.iter(|| {
            let config = default_config(&dungeon);
            RecastBuilder::new(config).build_mesh(&dungeon.vertices, &dungeon.indices)
        });
    });

    group.finish();
}
```

Input meshes: bridge.obj (1.5 KB), nav_test.obj (113 KB), dungeon.obj
(382 KB). Store in `test-data/meshes/` (shared with test fixtures).

**`detour/benches/pathfinding.rs`** -- Path queries:

| Benchmark | Setup | Measure |
|-----------|-------|---------|
| `find_path_short` | 3x3 grid navmesh, adjacent polys | `find_path` between neighbors |
| `find_path_medium` | nav_test.obj navmesh, ~10 poly path | `find_path` across mesh |
| `find_path_long` | dungeon.obj navmesh, ~30 poly path | `find_path` end-to-end |
| `find_straight_path` | Pre-computed poly path | `find_straight_path` funnel |
| `sliced_find_path` | dungeon.obj navmesh, 100 iterations | `init_sliced_find_path` + `update_sliced_find_path` |

**`detour/benches/spatial_queries.rs`** -- Spatial queries:

| Benchmark | Setup | Measure |
|-----------|-------|---------|
| `find_nearest_poly` | nav_test.obj navmesh | `find_nearest_poly` at 100 random positions |
| `raycast` | nav_test.obj navmesh | `raycast` from center to 100 directions |
| `find_distance_to_wall` | nav_test.obj navmesh | `find_distance_to_wall` at 100 positions |
| `move_along_surface` | nav_test.obj navmesh | `move_along_surface` at 100 positions |
| `find_polys_around_circle` | nav_test.obj navmesh | `find_polys_around_circle` varying radii |

**`detour-crowd/benches/crowd_update.rs`** -- Crowd simulation:

| Benchmark | Setup | Measure |
|-----------|-------|---------|
| `crowd_10_agents` | nav_test.obj, 10 agents with targets | `crowd.update(1.0/60.0)` |
| `crowd_50_agents` | nav_test.obj, 50 agents with targets | `crowd.update(1.0/60.0)` |
| `crowd_100_agents` | nav_test.obj, 100 agents with targets | `crowd.update(1.0/60.0)` |
| `crowd_100_agents_rvo` | Same + RVO enabled | `crowd.update(1.0/60.0)` |

Each crowd benchmark should use `criterion::BatchSize::SmallInput` and
create a fresh Crowd per iteration to avoid state accumulation.

#### Shared benchmark helpers

Create `test-data/src/lib.rs` as a shared test-data crate (or use
`include_str!` with relative paths):

```rust
/// Load a mesh from the test-data directory.
pub fn load_test_mesh(name: &str) -> TriMesh {
    let path = format!("{}/meshes/{}", env!("CARGO_MANIFEST_DIR"), name);
    TriMesh::from_obj(&path).expect("failed to load test mesh")
}

/// Build a NavMesh from a test mesh with default config.
pub fn build_test_navmesh(name: &str) -> NavMesh {
    let mesh = load_test_mesh(name);
    // ... same as examples/common.rs
}
```

#### Verification

```bash
cargo bench -p recast
cargo bench -p detour
cargo bench -p detour-crowd

# Verify all benchmarks compile and run at least one iteration
cargo bench -- --test
```

### 2.4 Publish to crates.io -- PENDING

> **Status**: Phase 1 blockers (error types, unwrap elimination) are
> resolved. Publication checklist items below still need to be completed.

#### Pre-publication checklist

**Metadata** (already present in workspace `Cargo.toml` but verify):

| Field | Status | Value |
|-------|--------|-------|
| `version` | Set | `0.1.0` |
| `edition` | Set | `2024` |
| `rust-version` | Set | `1.92` |
| `license` | Set | `MIT OR Apache-2.0` |
| `repository` | Set | GitHub URL |
| `homepage` | Set | GitHub URL |
| `keywords` | Set | `game, pathfinding, navmesh, recast, detour` |
| `categories` | Set | `game-development, algorithms` |
| `description` | **Missing** | Each crate needs its own `description` field |
| `readme` | **Missing** | Each crate should point to workspace README |

**Per-crate descriptions** (add to each crate's `Cargo.toml`):

| Crate | Description |
|-------|-------------|
| `recast-common` | `Shared types and utilities for the recast-rs navigation mesh library` |
| `recast` | `Navigation mesh generation from triangle meshes (Rust port of Recast)` |
| `detour` | `Pathfinding and spatial queries on navigation meshes (Rust port of Detour)` |
| `detour-crowd` | `Multi-agent crowd simulation with collision avoidance` |
| `detour-tilecache` | `Dynamic obstacle management with compressed tile storage` |
| `detour-dynamic` | `Dynamic navigation mesh generation with async support` |

**Documentation coverage**: Run `cargo doc --workspace --no-deps` with
`RUSTDOCFLAGS="-D warnings"`. Fix any missing doc comments on public items.
Priority: public functions and structs that appear in examples.

#### Publish order

Publish in dependency order. Each crate must be published and available on
crates.io before its dependents can be published:

1. `recast-common` (no workspace deps)
2. `recast` (depends on recast-common)
3. `detour` (depends on recast-common, recast)
4. `detour-crowd` (depends on recast-common, detour)
5. `detour-tilecache` (depends on recast-common, recast, detour)
6. `detour-dynamic` (depends on recast-common, recast, detour,
   detour-tilecache)

Do **not** publish `recast-cli` or `recast-rs-examples`.

#### Dry run

```bash
# Verify each crate passes dry-run (run in order)
cargo publish --dry-run -p recast-common
cargo publish --dry-run -p recast
cargo publish --dry-run -p detour
cargo publish --dry-run -p detour-crowd
cargo publish --dry-run -p detour-tilecache
cargo publish --dry-run -p detour-dynamic
```

Fix any issues (missing fields, path dependencies without version, etc.)
before actual publication.

#### Post-publication

- Tag the release: `git tag -a v0.1.0 -m "Initial crates.io release"`
- Update CHANGELOG.md with publication date
- Verify each crate page on crates.io shows correct metadata and README

## Phase 3: API Quality (Medium) -- IN PROGRESS

These issues improve the developer experience. All are breaking API changes
that must happen before 1.0 publication.

### 3.1 Replace C-Style Output Parameters -- PARTIALLY COMPLETE

> **Status**: `detour-crowd` migrated to `Vec3` parameters (30 public
> methods). `NavMeshQuery` `Vec3` migration is pending.

**Problem**: Some functions use C-style output parameter patterns instead of
returning owned values. The original roadmap claimed `find_path` was one of
these, but that is incorrect -- most `NavMeshQuery` methods already return
`Result<Vec<T>>` or `Result<(T, U)>`.

**Actual remaining C-style output parameters** (verified by codebase audit):

#### Public functions with `&mut Vec<T>` output parameters (2)

```rust
// crates/detour/src/bvh_tree.rs:118
pub fn query(&self, query_bounds: &Aabb, results: &mut Vec<PolyRef>)

// crates/recast/src/convex_volume.rs:225
pub fn clip_polygon(&self, polygon: &[Vec3], clipped: &mut Vec<Vec3>) -> bool
```

**Target**: Return `Vec<T>` instead of taking `&mut Vec<T>`:

```rust
pub fn query(&self, query_bounds: &Aabb) -> Vec<PolyRef>
pub fn clip_polygon(&self, polygon: &[Vec3]) -> Option<Vec<Vec3>>
```

#### Public function with `&mut [u8]` output buffer (1)

```rust
// crates/detour/src/nav_mesh.rs:3738
pub fn store_tile_state(&self, tile: &MeshTile, data: &mut [u8]) -> Result<usize>
```

Note: `binary_format::store_tile_state` already returns `Result<Vec<u8>>`.
The `NavMesh` method should match:

```rust
pub fn store_tile_state(&self, tile: &MeshTile) -> Result<Vec<u8>>
```

#### Public function with output parameter in return position (1)

```rust
// crates/detour/src/nav_mesh_query.rs:1198
pub fn move_along_surface(
    &self, start_ref: PolyRef, start_pos: &[f32; 3], end_pos: &[f32; 3],
    filter: &QueryFilter, visited_refs: &mut Vec<PolyRef>,
) -> Result<[f32; 3]>
```

**Target**: Return a struct or tuple instead of mixing return + output param:

```rust
pub fn move_along_surface(
    &self, start_ref: PolyRef, start_pos: Vec3, end_pos: Vec3,
    filter: &QueryFilter,
) -> Result<MoveAlongSurfaceResult>

pub struct MoveAlongSurfaceResult {
    pub position: Vec3,
    pub visited: Vec<PolyRef>,
}
```

#### Private functions (not user-facing, lower priority)

4 private functions in `recast` use `&mut Vec<T>` output parameters. These
are internal algorithm helpers translated from C++ and do not affect the
public API. Refactor opportunistically.

#### `detour_common.rs` vector utilities (12 public functions)

The `detour_common` module contains 12 public functions that take
`&mut [f32; 3]` as output parameters, directly translating C++ vector
utilities:

```rust
// All in crates/detour/src/detour_common.rs
pub fn dt_vcross(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3])
pub fn dt_vmad(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3], s: f32)
pub fn dt_vlerp(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3], t: f32)
pub fn dt_vadd(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3])
pub fn dt_vsub(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3])
pub fn dt_vscale(dest: &mut [f32; 3], v: &[f32; 3], t: f32)
pub fn dt_vmin(dest: &mut [f32; 3], v: &[f32; 3])
pub fn dt_vmax(dest: &mut [f32; 3], v: &[f32; 3])
pub fn dt_vset(dest: &mut [f32; 3], x: f32, y: f32, z: f32)
pub fn dt_vcopy(dest: &mut [f32; 3], a: &[f32; 3])
pub fn dt_vnormalize(v: &mut [f32; 3])
pub fn dt_calc_poly_center(tc: &mut [f32; 3], idx: &[u16], nidx: usize, verts: &[f32])
```

These should be replaced with `glam::Vec3` operations. Most are one-liners
that `glam` already provides (`Vec3::cross`, `Vec3::lerp`, operator
overloads). After migration, the `detour_common` module can be removed or
reduced to functions that have no `glam` equivalent (like
`dt_calc_poly_center`).

#### `&[f32; 3]` parameter migration

Beyond output parameters, **235 function parameters** across the workspace
use `&[f32; 3]` where `Vec3` would be more idiomatic:

| Crate | `&[f32; 3]` parameter count |
|-------|-----------------------------|
| detour | 170 |
| detour-crowd | 40 |
| recast | 21 |
| detour-tilecache | 3 |
| recast-common | 1 |
| detour-dynamic | 0 |

This is a large migration. Approach:

1. Start with `detour_common.rs` -- replace 12 functions with `Vec3`
2. Update `NavMeshQuery` public methods to accept `Vec3` instead of
   `&[f32; 3]`
3. Update `Crowd` public methods
4. Leave internal/private functions for later -- they can keep `[f32; 3]`
   internally and convert at boundaries

**Migration strategy**: Add `Vec3` overloads first (with `_v3` suffix or
separate impl block), deprecate `&[f32; 3]` versions, remove deprecated
versions before 1.0.

### 3.2 Reduce Public Field Exposure

**Problem**: 22 structs expose 5 or more public fields, preventing future
internal changes without API breakage.

#### Inventory by category

**Configuration structs** (9 structs, user-constructed):

| Struct | Crate | Fields | Has Default | Notes |
|--------|-------|--------|-------------|-------|
| DynamicNavMeshConfig | detour-dynamic | 26 | Yes | Has 16 `with_*()` methods |
| NavMeshCreateParams | detour | 25 | No | Largest user-facing struct |
| RecastConfig | recast | 18 | Yes | Core pipeline config |
| AgentParams | detour-crowd | 13 | Yes | Agent behavior params |
| DtObstacleAvoidanceParams | detour-crowd | 10 | Yes | RVO tuning |
| TileCacheBuilderConfig | detour-tilecache | 10 | Yes | Tile cache build config |
| FormationConfig | detour-crowd | 7 | No | Group movement params |
| RVOConfig | detour-crowd | 6 | Yes | Velocity obstacle params |
| NavMeshParams | detour | 5 | No | Mesh initialization |

**Action for configuration structs**: These are intentionally user-visible.
Keep fields public but add builder patterns (section 3.3). Add
`#[non_exhaustive]` to allow future field additions without breaking changes.

**Data model structs** (13 structs, internal representations):

| Struct | Crate | Fields | Category |
|--------|-------|--------|----------|
| PolyMesh | recast | 18 | Pipeline output (4 legacy duplicates) |
| CompactHeightfield | recast | 15 | Pipeline intermediate |
| TileHeader | detour | 15 | Tile metadata |
| TileCacheLayerHeader | detour-tilecache | 14 | Tile cache metadata |
| MeshTile | detour | 12 | Tile data |
| Formation | detour-crowd | 9 | Crowd formation |
| PolyMeshDetail | recast | 7 | Pipeline output |
| Poly | detour | 7 | Polygon definition |
| Heightfield | recast | 6 | Pipeline intermediate |
| Link | detour | 6 | Graph edge |
| DtObstacleCircle | detour-crowd | 6 | Algorithm internal |
| FormationAgent | detour-crowd | 6 | Formation member |
| RVOAgent | detour-crowd | 5 | Algorithm internal |

**Action for data model structs**: Make fields private, add accessor methods.
Prioritize by exposure risk:

**Priority 1 -- Pipeline output structs** (cross-crate boundaries):

`PolyMesh` is consumed by `detour` to build `NavMesh`. Its 4 legacy
duplicate fields must be removed first:

```rust
// Remove these 4 fields from PolyMesh:
pub vert_count: usize,       // duplicate of nverts
pub poly_count: usize,       // duplicate of npolys
pub vertices: Vec<u16>,      // duplicate of verts
pub max_verts_per_poly: usize, // duplicate of nvp
```

Then make remaining fields private with accessors:

```rust
pub struct PolyMesh {
    // All fields private
    verts: Vec<u16>,
    polys: Vec<u16>,
    // ...
}

impl PolyMesh {
    pub fn verts(&self) -> &[u16] { &self.verts }
    pub fn polys(&self) -> &[u16] { &self.polys }
    pub fn vert_count(&self) -> usize { self.nverts }
    pub fn poly_count(&self) -> usize { self.npolys }
    pub fn max_verts_per_poly(&self) -> usize { self.nvp }
    pub fn bounds(&self) -> (Vec3, Vec3) { (self.bmin, self.bmax) }
    pub fn cell_size(&self) -> f32 { self.cs }
    pub fn cell_height(&self) -> f32 { self.ch }
    // ...
}
```

`PolyMeshDetail` follows the same pattern. Both are consumed by
`NavMeshBuilder::build_from_recast()` in detour -- update that call site.

**Priority 2 -- Tile structures** (serialization boundaries):

`MeshTile`, `TileHeader`, and `TileCacheLayerHeader` are involved in
serialization. Make fields private but provide:
- Read-only accessors for all fields
- `pub(crate)` mutable access for internal construction
- Serde derives remain on the struct (not affected by field visibility)

**Priority 3 -- Algorithm internals** (low exposure risk):

`Link`, `Poly`, `DtObstacleCircle`, `RVOAgent`, `FormationAgent` are used
within single crates. Make fields `pub(crate)` instead of `pub`. No accessor
methods needed since all consumers are in the same crate.

**Priority 4 -- Intermediate pipeline structs**:

`Heightfield` and `CompactHeightfield` are consumed within the `recast`
crate and by `detour-tilecache`. Make fields `pub(crate)` where possible,
add accessors for cross-crate access.

#### Migration approach

1. Remove PolyMesh legacy duplicates (4 fields) -- find and update all
   references across the workspace
2. Add `#[non_exhaustive]` to all configuration structs
3. Convert Priority 3 structs to `pub(crate)` fields (crate-internal only,
   no API breakage outside the crate)
4. Add accessors to Priority 1 structs, make fields private
5. Add accessors to Priority 2 structs, make fields private
6. Convert Priority 4 structs last

Use `find_referencing_symbols` to identify all field access sites before
making any field private. Each struct conversion is a separate commit.

### 3.3 Add Builder Patterns for Configuration

**Problem**: Configuration structs have many fields (up to 28) with no
guided construction. Users must know all fields and their valid ranges.

#### Existing builder infrastructure

The codebase already has three "builder" types:

| Builder | Crate | Pattern | Fields |
|---------|-------|---------|--------|
| RecastBuilder | recast | Wraps RecastConfig, drives pipeline | 0 own fields |
| NavMeshBuilder | detour | Zero-sized, static methods only | 0 fields |
| TileCacheBuilder | detour-tilecache | Wraps TileCacheBuilderConfig | 0 own fields |

`DynamicNavMeshConfig` already has 16 `with_*()` fluent methods. This is the
pattern to follow for other config structs.

#### Builder specifications

**RecastConfigBuilder** (new, for `RecastConfig` with 18 fields):

```rust
pub struct RecastConfigBuilder {
    config: RecastConfig,
}

impl RecastConfigBuilder {
    /// Start with sensible defaults matching the C++ RecastDemo.
    pub fn new() -> Self {
        Self { config: RecastConfig::default() }
    }

    // Geometry bounds (required -- no sensible default)
    pub fn bounds(mut self, bmin: Vec3, bmax: Vec3) -> Self { ... }

    // Voxelization
    pub fn cell_size(mut self, cs: f32) -> Self { ... }
    pub fn cell_height(mut self, ch: f32) -> Self { ... }

    // Agent parameters (in voxel units)
    pub fn walkable_slope_angle(mut self, degrees: f32) -> Self { ... }
    pub fn walkable_height(mut self, voxels: i32) -> Self { ... }
    pub fn walkable_climb(mut self, voxels: i32) -> Self { ... }
    pub fn walkable_radius(mut self, voxels: i32) -> Self { ... }

    // Region building
    pub fn min_region_area(mut self, voxels: i32) -> Self { ... }
    pub fn merge_region_area(mut self, voxels: i32) -> Self { ... }

    // Mesh simplification
    pub fn max_edge_len(mut self, voxels: i32) -> Self { ... }
    pub fn max_simplification_error(mut self, error: f32) -> Self { ... }
    pub fn max_vertices_per_polygon(mut self, n: i32) -> Self { ... }

    // Detail mesh
    pub fn detail_sample_dist(mut self, dist: f32) -> Self { ... }
    pub fn detail_sample_max_error(mut self, error: f32) -> Self { ... }

    // Tiling
    pub fn border_size(mut self, size: i32) -> Self { ... }

    /// Validate and build. Returns error if bounds are not set or
    /// parameters are out of range.
    pub fn build(self) -> Result<RecastConfig> {
        // Calls existing config.validate()
        // Calls config.calculate_grid_size(bmin, bmax) to set width/height
    }
}
```

Default values (from existing `Default` impl):

| Field | Default | Unit |
|-------|---------|------|
| cs | 0.3 | world units |
| ch | 0.2 | world units |
| walkable_slope_angle | 45.0 | degrees |
| walkable_height | 2 | voxels |
| walkable_climb | 1 | voxels |
| walkable_radius | 1 | voxels |
| max_edge_len | 12 | voxels |
| max_simplification_error | 1.3 | world units |
| min_region_area | 8 | voxels^2 |
| merge_region_area | 20 | voxels^2 |
| max_vertices_per_polygon | 6 | count |
| detail_sample_dist | 6.0 | world units |
| detail_sample_max_error | 1.0 | world units |
| border_size | 0 | voxels |

**NavMeshCreateParamsBuilder** (new, for `NavMeshCreateParams` with 25
fields):

This struct has the most fields and the most complex construction. Fields
fall into 4 groups:

1. **Required mesh data** (from Recast output): verts, polys, poly_flags,
   poly_areas, detail meshes
2. **Optional off-mesh connections**: 6 parallel arrays + count
3. **Agent properties**: walkable_height, walkable_radius, walkable_climb
4. **Grid properties**: bmin, bmax, cs, ch, build_bv_tree

```rust
pub struct NavMeshCreateParamsBuilder {
    params: NavMeshCreateParams,
}

impl NavMeshCreateParamsBuilder {
    /// Create from Recast pipeline output. Extracts mesh data, bounds,
    /// cell size, and cell height from PolyMesh and PolyMeshDetail.
    pub fn from_recast(
        poly_mesh: &PolyMesh,
        detail_mesh: &PolyMeshDetail,
        nav_mesh_params: NavMeshParams,
    ) -> Self { ... }

    // Agent properties
    pub fn walkable_height(mut self, h: f32) -> Self { ... }
    pub fn walkable_radius(mut self, r: f32) -> Self { ... }
    pub fn walkable_climb(mut self, c: f32) -> Self { ... }

    // Off-mesh connections (optional)
    pub fn add_off_mesh_connection(
        mut self,
        start: Vec3, end: Vec3,
        radius: f32, bidirectional: bool,
        area: u8, flags: u16, user_id: u32,
    ) -> Self { ... }

    // BVH
    pub fn build_bv_tree(mut self, build: bool) -> Self { ... }

    pub fn build(self) -> Result<NavMeshCreateParams> { ... }
}
```

The `from_recast` constructor eliminates the need to manually copy 15+
fields from `PolyMesh` and `PolyMeshDetail`. This is where most user errors
occur today.

**AgentParamsBuilder** (new, for `AgentParams` with 13 fields):

```rust
impl AgentParamsBuilder {
    pub fn new() -> Self { Self { params: AgentParams::default() } }

    pub fn radius(mut self, r: f32) -> Self { ... }
    pub fn height(mut self, h: f32) -> Self { ... }
    pub fn max_speed(mut self, s: f32) -> Self { ... }
    pub fn max_acceleration(mut self, a: f32) -> Self { ... }
    pub fn collision_query_range(mut self, r: f32) -> Self { ... }
    pub fn path_optimization_range(mut self, r: f32) -> Self { ... }
    pub fn separation(mut self, enabled: bool) -> Self { ... }
    pub fn obstacle_avoidance(mut self, quality: u8) -> Self { ... }
    pub fn rvo(mut self, config: RVOConfig) -> Self { ... }

    pub fn build(self) -> AgentParams { ... }
}
```

Default values (from existing `Default` impl):

| Field | Default |
|-------|---------|
| radius | 0.6 |
| height | 2.0 |
| max_acceleration | 8.0 |
| max_speed | 3.5 |
| collision_query_range | 12.0 |
| path_optimization_range | 30.0 |
| separate | true |
| obstacle_avoidance_type | 3 (best quality) |
| use_rvo | true |

**DynamicNavMeshConfig** (extend existing):

Already has 16 `with_*()` methods. Add the missing ones and a `build()`
method with validation:

```rust
// Missing with_*() methods to add:
pub fn with_world_bounds(mut self, min: Vec3, max: Vec3) -> Self { ... }
pub fn with_partition(mut self, partition: PartitionType) -> Self { ... }
pub fn with_walkable_area(mut self, area: u8) -> Self { ... }
pub fn with_keep_intermediate_results(mut self, keep: bool) -> Self { ... }

/// Validate configuration. Returns error if world bounds are invalid
/// or parameters are out of range.
pub fn validate(&self) -> Result<()> { ... }
```

#### Implementation order

1. `RecastConfigBuilder` -- most commonly used, highest impact
2. `NavMeshCreateParamsBuilder` with `from_recast()` -- eliminates the
   most error-prone manual construction
3. `AgentParamsBuilder` -- frequently configured per-agent
4. Extend `DynamicNavMeshConfig` `with_*()` coverage

Each builder is a separate commit. Existing direct struct construction
continues to work (builders are additive, not replacing). Add
`#[non_exhaustive]` to config structs in the same commit as their builder.

## Phase 4: Ecosystem (Lower Priority) -- NOT STARTED

These items improve adoption and broaden the target audience. They are
independent of each other and can be worked in any order.

### 4.1 Interactive Demo

DotRecast and the C++ original both ship interactive demos. An interactive
demo serves two purposes: visual debugging during development, and a
showcase for potential users.

#### Reference demo comparison

| Project | Lines | GUI | Rendering | Tools |
|---------|-------|-----|-----------|-------|
| C++ RecastDemo | 13,600 | SDL3 + ImGui | OpenGL | 8 sample tools |
| DotRecast | 8,800 | Silk.NET + ImGui | OpenGL 3.3 | 11 tools |
| rerecast | 408 | Bevy + Gizmos | Bevy native | 4 examples |
| namigator MapViewerSDL | 1,570 | SDL3 | OpenGL 3.3 | Camera + pathfinding |

#### C++ RecastDemo tools (8)

1. Tile Edit -- modify tile-based navmeshes
2. Tile Highlight -- visual tile debugging
3. Temp Obstacle -- dynamic obstacle add/remove
4. NavMesh Tester -- A\* pathfinding with waypoints
5. NavMesh Prune -- remove unnecessary polygons
6. Off-Mesh Connection -- create jump links
7. Convex Volume -- draw and test area volumes
8. Crowd -- multi-agent simulation

#### Recommended approach: `egui` + `three-d`

**Why not Bevy**: A demo application should not require a full game engine.
Bevy adds ~50 crates to the dependency tree and imposes an ECS architecture.
The demo is a standalone tool, not a game.

**Why `egui` + `three-d`**: Both are pure Rust, support WASM + native,
and `three-d` has built-in `egui` integration. Total added dependencies
are smaller than Bevy. `egui` matches the ImGui immediate-mode pattern used
by the C++ and DotRecast demos.

#### Demo crate structure

```text
crates/recast-demo/
├── Cargo.toml
├── src/
│   ├── main.rs               # Window setup, event loop
│   ├── app.rs                 # Application state, tool switching
│   ├── renderer.rs            # three-d scene setup, camera
│   ├── debug_draw.rs          # NavMesh polygon rendering
│   ├── input_geometry.rs      # OBJ loading and display
│   ├── tools/
│   │   ├── mod.rs             # Tool trait, registry
│   │   ├── navmesh_tester.rs  # Pathfinding queries
│   │   ├── crowd.rs           # Crowd simulation
│   │   ├── temp_obstacle.rs   # TileCache obstacles
│   │   └── convex_volume.rs   # Area marking
│   └── ui/
│       ├── mod.rs             # egui layout
│       ├── settings_panel.rs  # RecastConfig controls
│       ├── stats_panel.rs     # Build timing, poly counts
│       └── log_panel.rs       # RecastContext log output
└── assets/
    └── nav_test.obj           # Default test mesh
```

#### Implementation phases

**Phase A -- Minimal viewer** (~800 lines):
- Window with `three-d` + `egui`
- Load OBJ file, display wireframe
- `RecastConfig` controls in egui panel
- Build navmesh on button press
- Render navmesh polygons (color by area)
- Display build statistics (timing, vertex/polygon counts)

**Phase B -- Pathfinding tool** (~400 lines):
- Click to place start/end positions
- Run `find_path` + `find_straight_path`
- Render path as line segments with waypoint markers
- `QueryFilter` controls (area costs, include/exclude flags)

**Phase C -- Crowd tool** (~500 lines):
- Place agents by clicking
- Set targets by right-clicking
- Animate `crowd.update(dt)` each frame
- Render agent positions, velocities, target lines
- `AgentParams` controls per agent

**Phase D -- Obstacle tool** (~300 lines):
- Add/remove cylinder, box, oriented box obstacles
- Trigger `TileCache` rebuild
- Visualize obstacle shapes and affected tiles

**Phase E -- WASM target** (~200 lines):
- `three-d` and `egui` both support WebGL/WASM
- Add `wasm-bindgen` entry point
- File loading via browser file picker or drag-and-drop
- Deploy as static HTML page (GitHub Pages or similar)

**Estimated total: 2,000-2,500 lines** (smaller than C++ or DotRecast
because `egui` and `three-d` handle rendering and UI, and the library
already has a clean API).

#### Dependencies

```toml
[dependencies]
three-d = "0.19"       # 3D rendering + windowing + egui integration
egui = "0.31"           # Immediate mode UI
recast = { path = "../recast" }
detour = { path = "../detour" }
detour-crowd = { path = "../detour-crowd" }
detour-tilecache = { path = "../detour-tilecache" }
recast-common = { path = "../recast-common" }
```

The demo crate is a `[[bin]]` target, not a library. It does not need to
compile to WASM as a library crate (Phase E adds WASM via `wasm-bindgen`
separately). Exclude it from workspace WASM CI checks.

### 4.2 `no_std` Support

#### Feasibility per crate

| Crate | Difficulty | Effort | Key blockers |
|-------|-----------|--------|-------------|
| recast-common | Easy | 2-3h | HashMap in mesh_simplification, std::io in Error |
| recast | Medium | 6-8h | HashMap (6), BinaryHeap (1), format! in logging |
| detour | Hard | 16-20h | std::io in binary_format, std::fs for persistence, HashMap (9) |
| detour-crowd | Medium | 4-6h | HashMap (4); blocked by detour |
| detour-tilecache | Medium-Hard | 8-10h | std::fs for persistence, HashMap (3) |
| detour-dynamic | Hard | 12-16h | std::sync::Arc, AtomicU64, mpsc channels |

**Total estimated effort: 48-63 hours across all crates.**

#### std type usage inventory

| std type | recast-common | recast | detour | detour-crowd | detour-tilecache | detour-dynamic |
|----------|--------------|--------|--------|-------------|-----------------|---------------|
| HashMap | 4 | 6 | 9 | 4 | 3 | 4 |
| HashSet | 1 | 1 | 1 | 0 | 1 | 0 |
| BinaryHeap | 0 | 1 | 1 | 0 | 0 | 0 |
| VecDeque | 0 | 0 | 3 | 0 | 0 | 0 |
| std::io | 1 | 0 | 5 | 0 | 0 | 1 |
| std::fs | 1 | 0 | 6 | 0 | 4 | 0 |
| std::sync | 0 | 0 | 0 | 0 | 0 | 12 |
| format! | few | 148 | many | some | few | some |

#### Dependency compatibility

All major dependencies support `no_std`:
- `glam` 0.31: `default-features = false` (uses `libm`)
- `thiserror` 2.0: `default-features = false`
- `log` 0.4: always `no_std`
- `bitflags` 2.10: `default-features = false`
- `ordered-float` 5.1: core-only
- `postcard` 1.1: `feature = "alloc"`
- `lz4_flex` 0.12: pure Rust
- `byteorder` 1.5: always `no_std`
- `async-lock` 3.4: `no_std` + alloc
- `futures-lite` 2.6: `no_std` + alloc

Problematic:
- `tokio`: std-only (already optional behind `tokio` feature)
- `web-time`: platform-specific, needs verification

#### Collection replacement strategy

`HashMap`/`HashSet` have no `alloc` equivalent. Options:

1. **Replace with `BTreeMap`/`BTreeSet`**: O(log n) vs O(1), acceptable for
   mesh generation where n is small (< 10,000 typically)
2. **Add `hashbrown` dependency**: Provides `no_std` HashMap/HashSet with
   identical API. Adds one dependency but preserves O(1) performance.
3. **Feature-gate**: Use `hashbrown` by default in `no_std`, `std::collections`
   otherwise.

Recommendation: Option 3 (feature-gated `hashbrown`). The performance
difference matters in `detour`'s pathfinding hot path.

#### Implementation order

**Phase 1 -- recast-common** (easy, 2-3 hours):

```rust
// lib.rs
#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

// Replace std imports
use alloc::vec::Vec;
use alloc::string::String;
use alloc::boxed::Box;

// Feature-gate:
#[cfg(feature = "std")]
use std::collections::{HashMap, HashSet};
#[cfg(not(feature = "std"))]
use hashbrown::{HashMap, HashSet};
```

- Already has `std` feature flag
- File I/O already behind `#[cfg(feature = "std")]`
- `Error::Io` variant already behind `std` feature

**Phase 2 -- recast** (medium, 6-8 hours):

- Replace `HashMap`/`HashSet`/`BinaryHeap` with feature-gated imports
- `BinaryHeap` is in `alloc::collections` (no replacement needed)
- `VecDeque` is in `alloc::collections` (no replacement needed)
- Gate `format!` usage in non-test code behind `std` or use `write!`
  to a pre-allocated buffer
- `web-time` usage in `RecastContext`: gate timing behind `std` feature,
  provide no-op timing in `no_std` mode

**Phase 3 -- detour** (hard, 16-20 hours):

The main blocker is `binary_format.rs` which uses `std::io::{Read, Write,
Cursor}`. Options:

1. Gate all serialization behind `std` feature (simplest)
2. Abstract Read/Write behind custom traits that work with `&[u8]` buffers
3. Use `embedded-io` crate for no_std I/O traits

Recommendation: Option 1 for initial release. Serialization is already
behind a `serialization` feature flag -- make it require `std` as well.
The `postcard` format works with `&[u8]` and could be made `no_std`
compatible separately.

File persistence (`nav_mesh.rs` save/load) must be `std`-only regardless.

**Phase 4 -- detour-crowd** (medium, 4-6 hours):

Blocked by detour. Once detour has `no_std` support, crowd is
straightforward: replace 4 `HashMap` instances, gate test utilities.

**Phase 5 -- detour-tilecache** (medium-hard, 8-10 hours):

Similar to detour: gate file persistence behind `std`, replace collections.
`lz4_flex` compression already works without `std`.

**Phase 6 -- detour-dynamic** (hard, 12-16 hours):

`std::sync::Arc` can be replaced with `alloc::sync::Arc`.
`AtomicU64` requires `core::sync::atomic` (available on most targets but
not all). `mpsc` channels have no `alloc` equivalent -- either gate async
job processing behind `std` or add `crossbeam-channel` (which is `no_std`
compatible with `alloc`).

Recommendation: Gate async features behind `std`. The primary `no_std`
use case is embedded systems that would not use async job processing.

#### What to prioritize

The highest-value `no_std` targets are `recast-common` and `recast`.
These enable navmesh generation on embedded and bare-metal targets.
`detour` is the next priority for pathfinding on embedded. The crowd,
tilecache, and dynamic crates are lower priority -- embedded systems
rarely need crowd simulation or dynamic obstacles.

#### How rerecast does it

rerecast uses:
- `#![no_std]` + `extern crate alloc;`
- `libm` for math functions (via `glam` with `default-features = false`)
- Feature flags separating `std` from `default_no_std`
- No `HashMap` usage (uses `Vec`-based lookups)

The key difference: rerecast is Recast-only (~4,000 lines). It does not
have Detour's serialization complexity or Crowd/TileCache features.

### 4.3 Framework Integrations

#### Existing Rust navmesh ecosystem

| Crate | Scope | Bevy version | Notes |
|-------|-------|-------------|-------|
| rerecast / bevy_rerecast | Generation + pathfinding | 0.17 | Recast-only, no Detour |
| oxidized_navigation | Generation + pathfinding | 0.15+ | Custom implementation |
| vleue_navigator | Pathfinding only | 0.13-0.18 | Polyanya algorithm, no generation |
| landmass | Full movement system | 0.15+ | A\*, steering, collision avoidance |

None of these provide the full Recast+Detour+Crowd+TileCache stack.

#### Bevy integration: `bevy-recast`

**Scope**: ~400-500 lines, separate crate outside the main workspace.

**Design based on rerecast's architecture** (which works well):

```text
bevy-recast/
├── Cargo.toml
├── src/
│   ├── lib.rs              # Plugin group, prelude
│   ├── settings.rs         # NavmeshSettings component (wraps RecastConfig)
│   ├── generator.rs        # Async generation via AsyncComputeTaskPool
│   ├── navmesh_resource.rs # Navmesh asset/resource wrapper
│   ├── pathfinding.rs      # Query wrapper, path request events
│   ├── backends/
│   │   ├── mod.rs          # Backend trait
│   │   └── mesh3d.rs       # Bevy Mesh3d -> TriMesh conversion
│   └── debug.rs            # Optional gizmo visualization
```

**Plugin architecture** (following rerecast's pattern):

```rust
pub struct RecastPlugin;

impl Plugin for RecastPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NavmeshQueue>()
           .init_resource::<NavmeshTaskQueue>()
           .add_event::<NavmeshReady>()
           .add_systems(PostUpdate, (
               drain_queue_into_tasks,
               poll_tasks,
           ).chain());
    }
}
```

**Key types**:

| Type | Purpose |
|------|---------|
| `NavmeshSettings` | Component wrapping `RecastConfig` + agent params |
| `NavmeshHandle` | Handle to built `NavMesh` (Bevy `Asset`) |
| `NavmeshReady` | Event fired when generation completes |
| `FindPathRequest` | Event to request A\* pathfinding |
| `FindPathResult` | Event containing path waypoints |
| `ExcludeFromNavmesh` | Marker component to exclude entities |

**Backend system** (geometry source abstraction):

```rust
// Users provide a system that converts world geometry to TriMesh
pub trait NavmeshApp {
    fn set_navmesh_backend<M>(
        &mut self,
        system: impl IntoSystem<In<NavmeshSettings>, TriMesh, M>,
    ) -> &mut Self;
}
```

Built-in backends:
- `Mesh3dBackend`: Queries `Mesh3d` + `GlobalTransform` components
- Physics backends (optional features): Avian3D, Rapier3D collider conversion

**Pathfinding integration**:

```rust
// Option A: Event-based (async, non-blocking)
app.add_event::<FindPathRequest>();
app.add_event::<FindPathResult>();

// Option B: System parameter (synchronous, per-frame)
fn my_system(navmesh: Res<NavmeshHandle>, query: NavmeshQuery) {
    let path = query.find_path(start, end, &filter)?;
}
```

Recommendation: Provide both. Event-based for complex queries, system
parameter for simple per-frame queries.

**Bevy version**: Target the latest stable Bevy (currently 0.17). Bevy's
API changes frequently -- expect maintenance burden per major version.

#### macroquad integration

macroquad has no plugin system. The integration is a utility module, not a
crate:

```rust
// ~150-200 lines
pub struct NavMeshContext {
    navmesh: Option<NavMesh>,
    query: Option<NavMeshQuery<'static>>,
}

impl NavMeshContext {
    pub fn build(&mut self, mesh: &TriMesh, config: &RecastConfig) -> Result<()> { ... }
    pub fn find_path(&self, start: Vec3, end: Vec3) -> Option<Vec<Vec3>> { ... }
    pub fn draw_navmesh(&self) { ... }  // Uses macroquad draw_triangle_3d
    pub fn draw_path(&self, path: &[Vec3]) { ... }
}
```

This could be a standalone example rather than a published crate. The scope
is small enough to include in the `examples/` directory.

#### Implementation order

1. Bevy integration first (largest user base in Rust gamedev)
2. macroquad example second (demonstrates non-ECS usage)
3. Other frameworks as requested by users

#### Separate repository vs monorepo

Framework integrations should be **separate repositories/crates**:
- Different release cadence (Bevy versions change often)
- Different dependencies (heavy framework deps should not pollute the core)
- Different maintainers possible
- rerecast uses this pattern (`bevy_rerecast` is in the rerecast monorepo
  but could be separate)

For recast-rs: start with the integration in-tree (easier development),
split to a separate repo if the maintenance burden grows.

### 4.4 Reduce Unsafe Code

#### Complete inventory

**16 unsafe blocks** across 3 files (plus 2 `unsafe impl`):

| File | Items | Category |
|------|-------|----------|
| detour/src/node_pool.rs | 12 | Raw pointer priority queue + Send/Sync |
| detour/src/nav_mesh.rs | 2 | Disjoint mutable refs, pointer offset |
| detour-dynamic/src/dynamic_tile.rs | 4 | Unchecked voxel span parsing |

#### detour/src/node_pool.rs (12 items)

The `DtNodeQueue` is a binary min-heap that stores `*mut DtNode` raw
pointers. It has:

- 2 `unsafe impl` (Send, Sync) at lines 443-444
- 1 pointer arithmetic in `get_node_idx` (line 231-232)
- 1 `get_unchecked` in `top()` (line 326)
- 1 raw pointer deref in `pop()` (line 347)
- 2 raw pointer derefs in `bubble_up()` (lines 390, 394)
- 3 raw pointer derefs in `trickle_down()` (lines 409, 421-422, 429)
- 2 raw pointer derefs in `push()` (accessing node.total)

**Problem**: The `unsafe impl Send` and `unsafe impl Sync` are **invalid**.
`DtNodeQueue` stores `Vec<*mut DtNode>` where the pointers reference nodes
owned by `DtNodePool`. If the pool is shared across threads, mutable
access through the raw pointers creates data races. The safety comment
("only used within the lifetime of the nodes") does not establish
thread-safety -- it establishes lifetime validity, which is a different
property.

**Safe replacement: Index-based priority queue**

Replace `Vec<*mut DtNode>` with `Vec<usize>` (indices into the node pool's
`nodes` vector). This eliminates all raw pointer operations:

```rust
pub struct DtNodeQueue {
    heap: Vec<usize>,  // indices into DtNodePool::nodes
    size: usize,
}

impl DtNodeQueue {
    pub fn push(&mut self, pool: &DtNodePool, node_idx: usize) {
        let cost = pool.nodes[node_idx].total;
        self.heap.push(node_idx);
        self.size += 1;
        self.bubble_up(pool, self.size - 1, node_idx);
    }

    pub fn pop(&mut self, pool: &DtNodePool) -> Option<usize> {
        if self.size == 0 { return None; }
        let result = self.heap[0];
        self.size -= 1;
        if self.size > 0 {
            self.heap[0] = self.heap[self.size];
            self.trickle_down(pool, 0, self.heap[0]);
        }
        self.heap.truncate(self.size);
        Some(result)
    }

    fn bubble_up(&mut self, pool: &DtNodePool, mut i: usize, node_idx: usize) {
        let cost = pool.nodes[node_idx].total;
        while i > 0 {
            let parent = (i - 1) / 2;
            let parent_cost = pool.nodes[self.heap[parent]].total;
            if cost >= parent_cost { break; }
            self.heap[i] = self.heap[parent];
            i = parent;
        }
        self.heap[i] = node_idx;
    }

    // trickle_down follows the same pattern
}
```

This change:
- Removes all 10 `unsafe` expression blocks
- Removes both `unsafe impl Send/Sync` (no longer needed -- `Vec<usize>`
  is automatically Send + Sync)
- Makes the code simpler and easier to audit
- May have a small performance cost from bounds checking (benchmark to
  verify -- the compiler may elide checks when indices are proven in-range)

**Effort**: 4-8 hours (refactor DtNodeQueue + update all call sites in
NavMeshQuery that use the queue)

#### detour/src/nav_mesh.rs (2 items)

**Item 1: `get_tile_and_poly_by_ref_mut`** (line 1887)

Creates two `&mut` references to overlapping memory: `&mut MeshTile` and
`&mut Poly` where the poly is inside the tile's `polys` vec. The SAFETY
comment claims disjoint references, but this is **incorrect** -- the
references overlap.

This can cause undefined behavior if the caller modifies `tile.polys`
(e.g., push/remove) while holding the `&mut Poly` reference, since
reallocation would invalidate the poly pointer.

**Safe replacement -- return indices**:

```rust
pub fn get_tile_and_poly_indices(
    &self,
    reference: PolyRef,
) -> Result<(usize, usize)> {
    let (salt, tile_idx, poly_idx) = self.decode_poly_id(reference);
    // ... validation ...
    Ok((tile_idx, poly_idx))
}

// Callers access via:
let (tile_idx, poly_idx) = nav_mesh.get_tile_and_poly_indices(ref)?;
let poly = &mut nav_mesh.tiles[tile_idx].as_mut().unwrap().polys[poly_idx];
```

**Alternative -- view struct**:

```rust
pub struct TilePolyMut<'a> {
    tile: &'a mut MeshTile,
    poly_idx: usize,
}

impl<'a> TilePolyMut<'a> {
    pub fn tile(&self) -> &MeshTile { self.tile }
    pub fn poly(&self) -> &Poly { &self.tile.polys[self.poly_idx] }
    pub fn poly_mut(&mut self) -> &mut Poly { &mut self.tile.polys[self.poly_idx] }
}
```

**Effort**: 3-6 hours (find all call sites with `find_referencing_symbols`,
update each to use indices or the view struct)

**Item 2: Pointer offset calculation** (line 2513)

Computes a polygon's index via `ptr.offset_from(base)`. Safe because both
pointers originate from the same `tile.polys` allocation, but fragile.

**Safe replacement**: Pass the poly index directly instead of computing it
from pointer arithmetic. This requires changing the call site to pass
`poly_idx: usize` instead of `poly: &Poly`.

**Effort**: 1-2 hours

#### detour-dynamic/src/dynamic_tile.rs (4 items)

Four `get_unchecked` calls in `reconstruct_heightfield()` (lines 191-250)
for parsing voxel span data. All are preceded by bounds checks:

```rust
// Pattern in all 4 blocks:
if position + required_bytes > span_data.len() {
    return Err(...);
}
let value = unsafe {
    i32::from_le_bytes([
        *span_data.get_unchecked(position),
        *span_data.get_unchecked(position + 1),
        // ...
    ])
};
```

These are **safe** (bounds are validated) and **performance-motivated**
(voxel parsing is a hot path). The compiler may not be able to prove the
bounds check eliminates the need for per-access checks.

**Safe replacement**:

```rust
let bytes: [u8; 4] = span_data[position..position + 4]
    .try_into()
    .map_err(|_| Error::Recast("truncated span data".into()))?;
let value = i32::from_le_bytes(bytes);
```

Or using a cursor:

```rust
let mut cursor = &span_data[position..];
let smin = i32::from_le_bytes(cursor[..4].try_into().unwrap());
cursor = &cursor[4..];
```

**Effort**: 1-2 hours. Benchmark the safe version first -- if the
performance difference is negligible (likely on modern CPUs with branch
prediction), replace unconditionally.

#### Priority and implementation order

1. **nav_mesh.rs `get_tile_and_poly_by_ref_mut`** -- correctness issue
   (undefined behavior). Fix first. (3-6 hours)
2. **node_pool.rs Send/Sync** -- remove invalid trait impls immediately.
   Can be done independently of the full queue refactor. (1 hour)
3. **node_pool.rs index-based queue** -- eliminates 10 unsafe blocks.
   Largest single improvement. (4-8 hours)
4. **nav_mesh.rs pointer offset** -- small fix, do alongside item 1.
   (1-2 hours)
5. **dynamic_tile.rs unchecked access** -- safe, low priority. Benchmark
   safe version; replace if no regression. (1-2 hours)

#### Benchmarking requirement

Before replacing unsafe code in hot paths (node_pool.rs priority queue,
dynamic_tile.rs voxel parsing), add benchmarks (Phase 2.3) that cover:

- `NavMeshQuery::find_path` with varying path lengths (exercises the
  priority queue)
- `DynamicNavMesh` voxel reconstruction (exercises span parsing)

Compare before/after to ensure no regression above 5%. If regression
exceeds 5%, keep the unsafe version with improved SAFETY documentation.

## Verification

After completing each phase, run:

```bash
# Phase 1 verification
grep -rn 'unwrap()\|expect(' crates/*/src/ --include='*.rs' | grep -v '#\[cfg(test)\]' | grep -v 'mod tests'
cargo fmt --all && cargo lint && cargo test-all
cargo check --workspace --no-default-features

# Phase 2 verification
cargo run -p recast-rs-examples --example basic_navmesh
cargo run -p recast-rs-examples --example pathfinding
cargo run -p recast-rs-examples --example crowd_simulation
cargo run -p recast-rs-examples --example tilecache_obstacles
cargo run -p recast-rs-examples --example serialization
cargo bench -- --test   # verify benchmarks compile and run
cargo test -p recast --test integration
cargo test -p detour --test integration
cargo publish --dry-run -p recast-common

# Phase 3 verification
# 3.1: Verify no C-style output parameters remain in public API
grep -rn 'pub fn.*&mut Vec<' crates/*/src/ --include='*.rs' | grep -v 'cfg(test)' | grep -v 'mod tests'
grep -rn 'pub fn.*dest:.*&mut \[f32' crates/*/src/ --include='*.rs'
# 3.1: Verify detour_common.rs vector functions removed or converted to Vec3
test ! -f crates/detour/src/detour_common.rs || grep -c 'pub fn dt_v' crates/detour/src/detour_common.rs
# 3.2: Verify legacy PolyMesh fields removed
grep -c 'vert_count\|poly_count\|vertices\|max_verts_per_poly' crates/recast/src/polymesh.rs
# 3.2: Verify #[non_exhaustive] on config structs
grep -c 'non_exhaustive' crates/recast/src/config.rs crates/detour/src/lib.rs crates/detour-crowd/src/crowd.rs
# 3.3: Verify builders exist and compile
cargo doc --workspace --no-deps
cargo fmt --all && cargo lint && cargo test-all

# Phase 4 verification
# 4.1: Verify demo builds and runs
cargo build -p recast-demo
cargo run -p recast-demo -- --help
# 4.2: Verify no_std builds
cargo build -p recast-common --no-default-features --target thumbv7em-none-eabihf
cargo build -p recast --no-default-features --target thumbv7em-none-eabihf
# 4.3: Verify framework integration compiles
cargo build -p bevy-recast
# 4.4: Verify zero unsafe blocks remain (or only justified ones)
grep -rn 'unsafe' crates/*/src/ --include='*.rs' | grep -v '#\[cfg(test)\]' | grep -v 'mod tests' | grep -v '// SAFETY:'
cargo fmt --all && cargo lint && cargo test-all
```
