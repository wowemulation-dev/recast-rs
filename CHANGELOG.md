# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added nextest configuration with profiles for development, CI, local, and release testing
- Added flamegraph profiling support via `cargo-flamegraph` with cargo aliases
- Added performance profiling documentation in `docs/src/development/profiling.md`
- Added cargo aliases for nextest testing and flamegraph generation
- Added pipeline accuracy comparison to README with Mermaid diagram and test mesh tables

#### recast-common

- Added `std` feature flag (enabled by default) to gate file I/O operations
- Added `TriMesh::from_obj_str()` for parsing OBJ content from strings (WASM-compatible)

### Fixed

#### recast

- Fixed 22 pipeline bugs bringing navmesh output within 1-2% of C++ reference
- Fixed heightfield normal check: `normal.y.abs()` to `normal.y` (Bug #9)
- Fixed erosion walkable area step size and `con[4]` indexing (Bugs #10-#11)
- Fixed `expand_regions` termination: `stack.retain()` broke convergence (Bug #12)
- Fixed `merge_small_regions`: picked largest neighbor instead of smallest (Bug #13)
- Fixed `distance_pt_seg`: returned `sqrt` instead of squared distance (Bug #14)
- Fixed `ContourVertex.region`: was `u16`, truncating `RC_BORDER_VERTEX` flag; changed
  to `i32` (Bug #15)
- Fixed contour simplification mask: `0x30000` to `0x2ffff` (Bug #16)
- Fixed `get_corner_height`: used 8-dir linked list instead of 4-dir `span.con[dir]`
  (Bug #17)
- Fixed 4-dir/8-dir direction mismatch in monotone/layer region building (Bug #19)
- Fixed grid size calculation: `ceil()` to round-to-nearest matching C++
  `rcCalcGridSize` (Bug #20)
- Fixed detail sample parameter scaling: apply `cs`/`ch` at call site (Bug #21a)
- Fixed rasterization x0 boundary clipping and span bounds checking
- Rewrote detail mesh generation to match C++ per-polygon architecture: local vertex
  buffers, `triangulate_hull` (shortest-perimeter ear heuristic), `delaunay_hull` for
  interior sampling, 4-value triangle format (Bug #21b)
- Fixed height patch population: region-based scan with connection-following BFS instead
  of vertex-seeding with grid-based flood fill (Bug #22)
- Implemented `remove_edge_vertices` (was stubbed): `can_remove_vertex`,
  `remove_vertex`, `triangulate_raw`

#### detour

- Fixed missing Recast-to-Detour direction remapping in `build_tile_data` (Bug #18)

### Changed

- Updated CI workflow to use nextest with `--profile ci` for faster test execution
- Added profiling artifacts to `.gitignore` (`*.svg`, `perf*.data`, `perf*.old`)
- Updated markdownlint configuration: added `$schema`, disabled MD024 for
  Keep a Changelog format, disabled MD060, expanded allowed HTML elements and
  code fence languages
- Added `.markdownlint-cli2.jsonc` config and `.markdownlintignore`
- Renamed `mise.toml` to `.mise.toml` (dotfile convention)
- Upgraded `glam` from 0.29 to 0.31
- Upgraded `bitflags` from 2.9 to 2.10
- Upgraded `ordered-float` from 5.0 to 5.1
- Upgraded `tokio` from 1.45 to 1.49
- Upgraded `lz4_flex` from 0.11 to 0.12
- Upgraded `tempfile` from 3.20 to 3.24
- Updated `bytes` to 1.11.1 (fixes RUSTSEC-2026-0007)
- Updated `socket2` to 0.6.2

#### recast-common

- Replaced catch-all `Error` enum with per-crate error types: `MeshError` in
  recast-common, `ConfigError`/`BuildError`/`ConvexVolumeError` in recast,
  `DetourError` in detour, `CrowdError` in detour-crowd, `TileCacheError` in
  detour-tilecache, `DynamicError` in detour-dynamic
- Moved `Error::Io` variant behind `std` feature for WASM compatibility
- Moved `TriMesh::from_obj()` behind `std` feature for WASM compatibility

#### recast

- Replaced `std::time::Instant` with `web-time` crate for WASM compatibility
- `RecastContext` timing now works on both native and WASM targets
- Renamed direction helpers: `get_dir_offset_y` to `get_dir_offset_z`, cardinal
  names to axis labels

#### detour

- Verified WASM compatibility (file I/O already feature-gated behind `serialization`)
- Added to CI WASM compilation checks

#### detour-crowd

- Migrated 30 public methods from `[f32; 3]` arrays to `Vec3` parameters and returns
- Verified WASM compatibility (no WASM-incompatible dependencies)
- Added to CI WASM compilation checks

#### detour-tilecache

- Replaced `lz4` crate with `lz4_flex` (pure Rust, no C dependencies)
- File I/O functions already feature-gated behind `serialization`
- In-memory serialization methods work on WASM
- Added to CI WASM compilation checks

#### detour-dynamic

- Replaced `tokio::sync::RwLock` with `async-lock::RwLock` (runtime-agnostic)
- Replaced `tokio::task::yield_now()` with `futures-lite::future::yield_now()`
- Made `tokio` an optional dependency (feature flag: `tokio`)
- Async methods now work on WASM without tokio dependency
- Added to CI WASM compilation checks

### Removed

- Removed unused `rayon` dependency and `parallel` feature from recast crate
- Removed unused `bytemuck` dependency from recast-common and recast crates
- Removed unused `criterion` workspace dependency (no benchmarks exist)
- Removed unused `anyhow` dependency from recast-common crate
- Removed unused `Zlib` license allowance from `deny.toml`
- Removed catch-all `Error` enum from recast-common (replaced by per-crate types)

## [0.1.0] - 2026-01-19

Initial release. This is a Rust port of [RecastNavigation][recast-cpp].

### Added

#### recast-common

- Error types for the workspace (`Error`, `Result`)
- Vector math utilities using glam
- Triangle mesh processing (`TriMesh`)
- Geometry utilities (bounds calculation, grid sizing)
- Debug visualization support

#### recast

- Heightfield voxelization from triangle meshes
- Compact heightfield generation
- Area marking (walkable, non-walkable, custom areas)
- Distance field and region building (watershed, monotone)
- Contour generation with simplification
- Polygon mesh generation (`PolyMesh`)
- Detail mesh generation (`PolyMeshDetail`)
- Configurable build parameters (`RecastConfig`)
- Context for logging and timing (`RecastContext`)

#### detour

- Navigation mesh data structure (`NavMesh`)
- A* pathfinding (`NavMeshQuery::find_path`)
- Path straightening with funnel algorithm (`find_straight_path`)
- Raycasting for line-of-sight queries
- Spatial queries (nearest poly, random point, poly height)
- Query filters for area costs and flags
- Off-mesh connection support
- Multi-tile navigation mesh support
- BVH tree for spatial indexing
- Optional serialization via `serialization` feature

#### detour-crowd

- Crowd simulation manager (`Crowd`)
- Agent management with configurable parameters
- Path corridor for agent navigation state
- RVO-based collision avoidance (`DtObstacleAvoidanceQuery`)
- Local boundary detection (`DtLocalBoundary`)
- Proximity grid for neighbor queries
- Formation and group behavior support
- Behavior tree system for agent AI

#### detour-tilecache

- Tile cache for streaming large worlds (`TileCache`)
- Dynamic obstacle management
- Cylinder, box, and oriented box obstacles
- LZ4 compression for tile storage
- Incremental tile rebuilding
- Optional serialization via `serialization` feature

#### detour-dynamic

- Dynamic navigation mesh generation (`DynamicNavMesh`)
- Multiple collider types:
  - `BoxCollider` (axis-aligned and oriented)
  - `CylinderCollider`
  - `SphereCollider`
  - `CapsuleCollider`
  - `ConvexCollider`
  - `TrimeshCollider`
  - `CompositeCollider`
- Async tile rebuilding via Tokio
- Checkpoint system for incremental updates
- Voxel-based raycasting queries
- Collider serialization support

#### recast-cli

- `build` command for navmesh generation from OBJ files
- `find-path` command for pathfinding queries
- Configurable build parameters via command-line flags

### Notes

- Rust 2024 edition (requires Rust 1.92+)
- Uses glam for vector math (WASM SIMD compatible)
- Uses postcard for binary serialization (no_std compatible)
- WASM compatible (simd128 support via glam)

[Unreleased]: https://github.com/wowemulation-dev/recast-rs/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/wowemulation-dev/recast-rs/releases/tag/v0.1.0
[recast-cpp]: https://github.com/recastnavigation/recastnavigation
