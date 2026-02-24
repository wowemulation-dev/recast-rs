# Resolution Roadmap

Items are ordered by priority: critical blockers first, then high-severity
issues, then improvements.

## Phase 1: Library Safety (Critical) -- COMPLETE

These issues were resolved for the v0.1.0 release.

### 1.1 Eliminate `unwrap()`/`expect()` from Library Code -- COMPLETE

> **Status**: Reduced from 45 to 2. The 2 remaining are in waymark-dynamic
> job processing (`collider_removal_job.rs`, `dynamic_tile_job.rs`).

**Problem**: 45 `unwrap()`/`expect()` calls in non-test library code
(verified count: 20 in waymark, 13 in waymark-tilecache, 9 in landmark, 2 in
waymark-crowd, 1 doctest in landmark-common). A library that panics on
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

1. `waymark-tilecache` -- 13 panics on resource exhaustion in `tile_cache.rs`
2. `waymark` -- 20 calls, includes A\* open list pop at `nav_mesh_query.rs:438`
3. `landmark` -- 9 calls, includes cell index unwraps in `watershed.rs`
4. `waymark-crowd` -- 2 calls
5. `landmark-common` -- 1 doctest in `mesh.rs`

**Verification**: `grep -rn 'unwrap()\|expect(' crates/*/src/ --include='*.rs'`
filtered to exclude `#[cfg(test)]` modules should return zero results.

### 1.2 Add Structured Error Types -- COMPLETE

> **Status**: Per-crate error types implemented. The old catch-all `Error`
> enum has been removed from landmark-common. Each crate owns its errors:
> `MeshError`, `ConfigError`/`BuildError`/`ConvexVolumeError`, `DetourError`,
> `CrowdError`, `TileCacheError`, `DynamicError`.

**Problem**: The workspace `Error` enum in `landmark-common/src/lib.rs` has
6 variants. 5 of 6 use bare `String` as payload. The `Pathfinding` variant
is unused (zero occurrences in the codebase). The `waymark` crate has a
well-designed `Status` enum with 22 variants (`InvalidParam`, `OutOfMemory`,
`PathInvalid`, etc.) but converts these to strings via `.to_string()` before
wrapping in `Error::Detour(String)`. This pattern appears 242 times,
destroying type information that callers need for error handling.

**Error site counts (non-test code)**:

| Pattern | Count | Location |
|---------|-------|----------|
| `Error::Detour(Status::InvalidParam.to_string())` | 159 | waymark, waymark-crowd, waymark-tilecache |
| `Error::Detour(Status::Failure.to_string())` | 45 | waymark, waymark-crowd, waymark-tilecache |
| `Error::NavMeshGeneration(String)` | 26 | landmark |
| `Error::InvalidMesh(String)` | 23 | landmark, waymark-crowd, waymark-tilecache |
| `Error::Detour(Status::NotFound.to_string())` | 13 | waymark, waymark-tilecache |
| `Error::Detour(Status::PathInvalid.to_string())` | 7 | waymark, waymark-crowd |
| `Error::Detour(Status::OutOfMemory.to_string())` | 7 | waymark, waymark-tilecache |
| `Error::Recast(String)` | 7 | landmark, waymark-dynamic |
| `Error::Detour(Status::WrongVersion.to_string())` | 4 | waymark |
| `Error::Detour(Status::WrongMagic.to_string())` | 3 | waymark |
| `Error::Detour(Status::BufferTooSmall.to_string())` | 3 | waymark |
| `Error::Detour(Status::InProgress.to_string())` | 1 | waymark |
| `Error::Detour(ad-hoc string)` | 3 | waymark (nav_mesh.rs, nav_mesh_query.rs) |
| `Error::Pathfinding(String)` | 0 | unused |
| **Total** | **301** | |

#### Approach

Each crate defines its own error type. The workspace `Error` in
`landmark-common` is removed. Functions return crate-specific
`Result<T, CrateError>`.

#### Step 1: `landmark-common` -- Remove catch-all Error

Delete the current `Error` enum. Replace with:

```rust
// landmark-common/src/error.rs

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

#### Step 2: `landmark` -- Per-stage error types

The 58 error sites in the landmark crate fall into these categories:

```rust
// landmark/src/error.rs

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

#### Step 3: `waymark` -- Promote Status to error type

The `Status` enum already has the right categories. Make it implement
`std::error::Error` and use it directly:

```rust
// waymark/src/error.rs
use crate::status::Status;

/// Error from waymark operations
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
    Build(#[from] landmark::BuildError),

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

#### Step 4: `waymark-crowd` -- Crowd-specific error type

```rust
// waymark-crowd/src/error.rs

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
    Detour(#[from] waymark::DetourError),
}
```

**Migration**: The 2 `Error::InvalidMesh(...)` calls in rvo.rs become
`CrowdError::Rvo(...)`. The `Error::Detour(Status::InvalidParam...)` calls
become `CrowdError::InvalidParam`.

#### Step 5: `waymark-tilecache` -- TileCache-specific error type

```rust
// waymark-tilecache/src/error.rs

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
    Detour(#[from] waymark::DetourError),

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

#### Step 6: `waymark-dynamic` -- Dynamic-specific error type

```rust
// waymark-dynamic/src/error.rs

#[derive(thiserror::Error, Debug)]
pub enum DynamicError {
    #[error("invalid span data at cell ({x}, {y}): {detail}")]
    InvalidSpanData { x: i32, y: i32, detail: String },

    #[error("invalid partition type")]
    InvalidPartitionType,

    #[error("job queue full")]
    JobQueueFull,

    #[error(transparent)]
    Config(#[from] landmark::ConfigError),

    #[error(transparent)]
    Build(#[from] landmark::BuildError),

    #[error(transparent)]
    Detour(#[from] waymark::DetourError),

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
   landmark-common -> landmark -> waymark -> waymark-crowd -> waymark-tilecache ->
   waymark-dynamic
3. Delete old `Error` enum from landmark-common
4. Update landmark-cli to handle new error types (use `anyhow` to collect)

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

These issues block practical adoption. Sections 2.1-2.3 are complete and
merged to main. Section 2.4 (crates.io publication) is pending.

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
landmark-common = { workspace = true }
landmark = { workspace = true }
waymark = { workspace = true, features = ["serialization"] }
waymark-crowd = { workspace = true }
waymark-tilecache = { workspace = true, features = ["serialization"] }
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
> `crates/landmark/tests/` and `crates/waymark/tests/` validate against C++
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
cargo test -p landmark --test integration
cargo test -p waymark --test integration
```

### 2.3 Add Benchmarks -- COMPLETE

> **Status**: Benchmark directories exist in `crates/landmark/benches/`,
> `crates/waymark/benches/`, and `crates/waymark-crowd/benches/`. Uses
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
  landmark/
    benches/
      generation.rs     # Heightfield, compact, contour, polymesh
  waymark/
    benches/
      pathfinding.rs    # find_path, find_straight_path, sliced
      spatial_queries.rs # find_nearest_poly, raycast, find_distance_to_wall
  waymark-crowd/
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

**`landmark/benches/generation.rs`** -- Navmesh generation pipeline:

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

**`waymark/benches/pathfinding.rs`** -- Path queries:

| Benchmark | Setup | Measure |
|-----------|-------|---------|
| `find_path_short` | 3x3 grid navmesh, adjacent polys | `find_path` between neighbors |
| `find_path_medium` | nav_test.obj navmesh, ~10 poly path | `find_path` across mesh |
| `find_path_long` | dungeon.obj navmesh, ~30 poly path | `find_path` end-to-end |
| `find_straight_path` | Pre-computed poly path | `find_straight_path` funnel |
| `sliced_find_path` | dungeon.obj navmesh, 100 iterations | `init_sliced_find_path` + `update_sliced_find_path` |

**`waymark/benches/spatial_queries.rs`** -- Spatial queries:

| Benchmark | Setup | Measure |
|-----------|-------|---------|
| `find_nearest_poly` | nav_test.obj navmesh | `find_nearest_poly` at 100 random positions |
| `raycast` | nav_test.obj navmesh | `raycast` from center to 100 directions |
| `find_distance_to_wall` | nav_test.obj navmesh | `find_distance_to_wall` at 100 positions |
| `move_along_surface` | nav_test.obj navmesh | `move_along_surface` at 100 positions |
| `find_polys_around_circle` | nav_test.obj navmesh | `find_polys_around_circle` varying radii |

**`waymark-crowd/benches/crowd_update.rs`** -- Crowd simulation:

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
cargo bench -p landmark
cargo bench -p waymark
cargo bench -p waymark-crowd

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
| `landmark-common` | `Shared types and utilities for the recast-rs navigation mesh library` |
| `landmark` | `Navigation mesh generation from triangle meshes (Rust port of Recast)` |
| `waymark` | `Pathfinding and spatial queries on navigation meshes (Rust port of Detour)` |
| `waymark-crowd` | `Multi-agent crowd simulation with collision avoidance` |
| `waymark-tilecache` | `Dynamic obstacle management with compressed tile storage` |
| `waymark-dynamic` | `Dynamic navigation mesh generation with async support` |

**Documentation coverage**: Run `cargo doc --workspace --no-deps` with
`RUSTDOCFLAGS="-D warnings"`. Fix any missing doc comments on public items.
Priority: public functions and structs that appear in examples.

#### Publish order

Publish in dependency order. Each crate must be published and available on
crates.io before its dependents can be published:

1. `landmark-common` (no workspace deps)
2. `landmark` (depends on landmark-common)
3. `waymark` (depends on landmark-common, landmark)
4. `waymark-crowd` (depends on landmark-common, waymark)
5. `waymark-tilecache` (depends on landmark-common, landmark, waymark)
6. `waymark-dynamic` (depends on landmark-common, landmark, waymark,
   waymark-tilecache)

Do **not** publish `landmark-cli` or `recast-rs-examples`.

#### Dry run

```bash
# Verify each crate passes dry-run (run in order)
cargo publish --dry-run -p landmark-common
cargo publish --dry-run -p landmark
cargo publish --dry-run -p waymark
cargo publish --dry-run -p waymark-crowd
cargo publish --dry-run -p waymark-tilecache
cargo publish --dry-run -p waymark-dynamic
```

Fix any issues (missing fields, path dependencies without version, etc.)
before actual publication.

#### Post-publication

- Tag the release: `git tag -a v0.1.0 -m "Initial crates.io release"`
- Update CHANGELOG.md with publication date
- Verify each crate page on crates.io shows correct metadata and README

## Phase 3: API Quality (Medium) -- MOSTLY COMPLETE

These issues improve the developer experience. Sections 3.1 and 3.2 are
complete. Section 3.3 is partially complete (fluent methods added, separate
builder structs pending).

### 3.1 Replace C-Style Output Parameters -- MOSTLY COMPLETE

> **Status**: `waymark-crowd` migrated to `Vec3` (30 public methods).
> `NavMeshQuery` migrated to `Vec3` (all public methods). 14 C-style
> `detour_common` vector functions with `&mut` output params removed.
> `bvh_tree::query`, `store_tile_state`, and `move_along_surface`
> converted to return values. 12 read-only `detour_common` utility
> functions remain (they use `&[f32]` input params, not output params).

**Problem**: Some functions use C-style output parameter patterns instead of
returning owned values.

**Completed work**:

- `NavMeshQuery`: all public methods migrated from `&[f32; 3]` to `Vec3`
- `waymark-crowd`: 30 public methods migrated from `[f32; 3]` to `Vec3`
- `waymark`: struct fields migrated from `[f32; 3]` to `Vec3`
- `detour_common`: 14 C-style vector functions with `&mut [f32; 3]` output
  parameters removed (`dt_vcross`, `dt_vmad`, `dt_vlerp`, `dt_vadd`,
  `dt_vsub`, `dt_vscale`, `dt_vmin`, `dt_vmax`, `dt_vset`, `dt_vcopy`,
  `dt_vnormalize`, `dt_calc_poly_center`, and 2 others)
- `bvh_tree::query`: returns `Vec<PolyRef>` instead of taking `&mut Vec`
- `store_tile_state`: returns `Result<Vec<u8>>` instead of taking `&mut [u8]`
- `move_along_surface`: returns `MoveAlongSurfaceResult` struct instead of
  mixing return value with `&mut Vec` output parameter
- C-style output parameters in detour converted to return values

**Remaining work**:

- 12 read-only utility functions in `detour_common.rs` still use `&[f32]`
  and `&[f32; 3]` input parameters (not output params). These are internal
  helpers (`dt_vdot`, `dt_vlen`, `dt_vdist`, etc.) used by internal code.
  They could be replaced with `glam::Vec3` operations but this is low
  priority since they do not affect the public API.
- `recast/src/triangle_utils.rs`: `vcopy` still takes `&mut [f32; 3]`
  (internal helper)
- `sliced_pathfinding.rs`: `get_path_mut` returns `&mut Vec<PolyRef>`
  (intentional mutable access, not a C-style output pattern)

### 3.2 Reduce Public Field Exposure -- COMPLETE

> **Status**: All data model structs have private fields with accessors.
> Configuration structs have `#[non_exhaustive]` and `Default` impls.
> 11 config structs marked `#[non_exhaustive]`.

**Completed work**:

- Added `#[non_exhaustive]` to 11 configuration structs with `Default` impls
- Made `Heightfield` and `CompactHeightfield` fields private, added accessors
- Made `PolyMesh` and `PolyMeshDetail` fields private, added accessors
- Made `MeshTile`, `TileHeader`, `TileCacheLayerHeader` fields private,
  added accessors
- Made `Link`, `Poly` fields private (pub(crate))
- Made `DtObstacleCircle`, `RVOAgent`, `FormationAgent` fields private

### 3.3 Add Builder Patterns for Configuration -- PARTIALLY COMPLETE

> **Status**: `RecastConfig` has 15 fluent `with_*` methods.
> `DynamicNavMeshConfig` has 16 `with_*` methods. Separate `Builder`
> structs with validation have not been created.

**Completed work**:

- `RecastConfig`: 15 `with_*()` fluent methods (`with_cell_size`,
  `with_cell_height`, `with_bounds`, `with_walkable_slope_angle`,
  `with_walkable_height`, `with_walkable_climb`, `with_walkable_radius`,
  `with_max_edge_len`, `with_max_simplification_error`,
  `with_min_region_area`, `with_merge_region_area`,
  `with_max_vertices_per_polygon`, `with_detail_sample_dist`,
  `with_detail_sample_max_error`, `with_border_size`)
- `DynamicNavMeshConfig`: 16 `with_*()` fluent methods (pre-existing)
- All configuration structs have `#[non_exhaustive]` and `Default` impls
  (from section 3.2)

**Remaining work**:

- `NavMeshCreateParamsBuilder` with `from_recast()` constructor that
  eliminates manual field copying from `PolyMesh`/`PolyMeshDetail`
- `AgentParamsBuilder` for `waymark-crowd` `AgentParams`
- Validating `build()` methods that check parameter ranges before
  constructing config structs

## Phase 4: Ecosystem (Lower Priority) -- PARTIALLY COMPLETE

These items improve adoption and broaden the target audience. They are
independent of each other and can be worked in any order. Section 4.4
(unsafe code) is complete.

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
landmark = { path = "../landmark" }
waymark = { path = "../waymark" }
waymark-crowd = { path = "../waymark-crowd" }
waymark-tilecache = { path = "../waymark-tilecache" }
landmark-common = { path = "../landmark-common" }
```

The demo crate is a `[[bin]]` target, not a library. It does not need to
compile to WASM as a library crate (Phase E adds WASM via `wasm-bindgen`
separately). Exclude it from workspace WASM CI checks.

### 4.2 `no_std` Support

#### Feasibility per crate

| Crate | Difficulty | Effort | Key blockers |
|-------|-----------|--------|-------------|
| landmark-common | Easy | 2-3h | HashMap in mesh_simplification, std::io in Error |
| landmark | Medium | 6-8h | HashMap (6), BinaryHeap (1), format! in logging |
| waymark | Hard | 16-20h | std::io in binary_format, std::fs for persistence, HashMap (9) |
| waymark-crowd | Medium | 4-6h | HashMap (4); blocked by waymark |
| waymark-tilecache | Medium-Hard | 8-10h | std::fs for persistence, HashMap (3) |
| waymark-dynamic | Hard | 12-16h | std::sync::Arc, AtomicU64, mpsc channels |

**Total estimated effort: 48-63 hours across all crates.**

#### std type usage inventory

| std type | landmark-common | landmark | waymark | waymark-crowd | waymark-tilecache | waymark-dynamic |
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
difference matters in `waymark`'s pathfinding hot path.

#### Implementation order

**Phase 1 -- landmark-common** (easy, 2-3 hours):

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

**Phase 2 -- landmark** (medium, 6-8 hours):

- Replace `HashMap`/`HashSet`/`BinaryHeap` with feature-gated imports
- `BinaryHeap` is in `alloc::collections` (no replacement needed)
- `VecDeque` is in `alloc::collections` (no replacement needed)
- Gate `format!` usage in non-test code behind `std` or use `write!`
  to a pre-allocated buffer
- `web-time` usage in `RecastContext`: gate timing behind `std` feature,
  provide no-op timing in `no_std` mode

**Phase 3 -- waymark** (hard, 16-20 hours):

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

**Phase 4 -- waymark-crowd** (medium, 4-6 hours):

Blocked by waymark. Once waymark has `no_std` support, crowd is
straightforward: replace 4 `HashMap` instances, gate test utilities.

**Phase 5 -- waymark-tilecache** (medium-hard, 8-10 hours):

Similar to waymark: gate file persistence behind `std`, replace collections.
`lz4_flex` compression already works without `std`.

**Phase 6 -- waymark-dynamic** (hard, 12-16 hours):

`std::sync::Arc` can be replaced with `alloc::sync::Arc`.
`AtomicU64` requires `core::sync::atomic` (available on most targets but
not all). `mpsc` channels have no `alloc` equivalent -- either gate async
job processing behind `std` or add `crossbeam-channel` (which is `no_std`
compatible with `alloc`).

Recommendation: Gate async features behind `std`. The primary `no_std`
use case is embedded systems that would not use async job processing.

#### What to prioritize

The highest-value `no_std` targets are `landmark-common` and `landmark`.
These enable navmesh generation on embedded and bare-metal targets.
`waymark` is the next priority for pathfinding on embedded. The crowd,
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

### 4.4 Reduce Unsafe Code -- COMPLETE

> **Status**: All unsafe code removed from the workspace. Zero `unsafe`
> blocks remain in library code.

**Completed work**:

- `node_pool.rs`: replaced raw pointer priority queue (`Vec<*mut DtNode>`)
  with index-based queue (`Vec<usize>`). Removed all 10 `unsafe` blocks
  and both invalid `unsafe impl Send/Sync`.
- `nav_mesh.rs`: removed `get_tile_and_poly_by_ref_mut` overlapping
  `&mut` references and pointer offset calculation. Replaced with
  index-based access.
- `dynamic_tile.rs`: replaced 4 `get_unchecked` calls with safe slice
  indexing. No measurable performance regression.

## Verification

After completing each phase, run:

```bash
# Phase 1 verification (COMPLETE)
grep -rn 'unwrap()\|expect(' crates/*/src/ --include='*.rs' | grep -v '#\[cfg(test)\]' | grep -v 'mod tests'
cargo fmt --all && cargo lint && cargo test-all
cargo check --workspace --no-default-features

# Phase 2 verification (2.1-2.3 COMPLETE, 2.4 PENDING)
cargo run -p recast-rs-examples --example basic_navmesh
cargo run -p recast-rs-examples --example pathfinding
cargo run -p recast-rs-examples --example crowd_simulation
cargo run -p recast-rs-examples --example tilecache_obstacles
cargo run -p recast-rs-examples --example serialization
cargo bench -- --test   # verify benchmarks compile and run
cargo test -p landmark --test integration
cargo test -p waymark --test integration
cargo publish --dry-run -p landmark-common

# Phase 3 verification (3.1-3.2 COMPLETE, 3.3 PARTIAL)
# 3.2: Verify #[non_exhaustive] on config structs
grep -c 'non_exhaustive' crates/landmark/src/config.rs crates/waymark/src/lib.rs crates/waymark-crowd/src/crowd.rs
# 3.3: Verify builders exist and compile
cargo doc --workspace --no-deps
cargo fmt --all && cargo lint && cargo test-all

# Phase 4 verification (4.4 COMPLETE, 4.1-4.3 NOT STARTED)
# 4.1: Verify demo builds and runs
cargo build -p recast-demo
cargo run -p recast-demo -- --help
# 4.2: Verify no_std builds
cargo build -p landmark-common --no-default-features --target thumbv7em-none-eabihf
cargo build -p landmark --no-default-features --target thumbv7em-none-eabihf
# 4.3: Verify framework integration compiles
cargo build -p bevy-recast
# 4.4: Verify zero unsafe blocks (COMPLETE)
grep -rn 'unsafe' crates/*/src/ --include='*.rs' | grep -v '#\[cfg(test)\]' | grep -v 'mod tests'
cargo fmt --all && cargo lint && cargo test-all
```
