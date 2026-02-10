# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added mdbook documentation site in `docs/`

### Removed

- Removed unused `rayon` dependency and `parallel` feature from recast crate
- Removed unused `bytemuck` dependency from recast-common and recast crates
- Removed unused `criterion` workspace dependency (no benchmarks exist)
- Removed unused `anyhow` dependency from recast-common crate
- Removed unused `Zlib` license allowance from `deny.toml`

### Changed

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

- Added `std` feature flag (enabled by default) to gate file I/O operations
- Moved `Error::Io` variant behind `std` feature for WASM compatibility
- Moved `TriMesh::from_obj()` behind `std` feature for WASM compatibility

#### recast

- Replaced `std::time::Instant` with `web-time` crate for WASM compatibility
- `RecastContext` timing now works on both native and WASM targets

#### detour

- Verified WASM compatibility (file I/O already feature-gated behind `serialization`)
- Added to CI WASM compilation checks

#### detour-crowd

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

### Added

#### recast-common

- Added `TriMesh::from_obj_str()` for parsing OBJ content from strings (WASM-compatible)

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
