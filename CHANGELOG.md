# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-02-23

Initial release. Rust port of [RecastNavigation][recast-cpp] with 33 pipeline
bug fixes achieving exact output match with C++ for all three test meshes.

### Added

#### landmark-common

- Error types for the workspace (`MeshError`)
- Vector math utilities using glam
- Triangle mesh processing (`TriMesh`)
- `TriMesh::from_obj_str()` for parsing OBJ content from strings (WASM-compatible)
- Geometry utilities (bounds calculation, grid sizing)
- Debug visualization support
- `std` feature flag (enabled by default) to gate file I/O operations

#### landmark

- Heightfield voxelization from triangle meshes
- Compact heightfield generation
- Area marking (walkable, non-walkable, custom areas)
- Distance field and region building (watershed, monotone)
- Contour generation with simplification
- Polygon mesh generation (`PolyMesh`)
- Detail mesh generation (`PolyMeshDetail`)
- Configurable build parameters (`RecastConfig`)
- Fluent `with_*` builder methods on `RecastConfig`
- Context for logging and timing (`RecastContext`)
- Navmesh generation benchmarks

#### waymark

- Navigation mesh data structure (`NavMesh`)
- A* pathfinding (`NavMeshQuery::find_path`)
- Path straightening with funnel algorithm (`find_straight_path`)
- Raycasting for line-of-sight queries
- Spatial queries (nearest poly, random point, poly height)
- Query filters for area costs and flags
- Off-mesh connection support
- Multi-tile navigation mesh support
- BVH tree for spatial indexing
- Optional serialization via `serialization` feature (JSON, binary, postcard)
- Pathfinding and spatial query benchmarks

#### waymark-crowd

- Crowd simulation manager (`Crowd`)
- Agent management with configurable parameters
- Path corridor for agent navigation state
- RVO-based collision avoidance (`DtObstacleAvoidanceQuery`)
- Local boundary detection (`DtLocalBoundary`)
- Proximity grid for neighbor queries
- Formation and group behavior support
- Behavior tree system for agent AI
- Crowd simulation benchmarks

#### waymark-tilecache

- Tile cache for streaming large worlds (`TileCache`)
- Dynamic obstacle management
- Cylinder, box, and oriented box obstacles
- LZ4 compression for tile storage (pure Rust via `lz4_flex`)
- Incremental tile rebuilding
- Optional serialization via `serialization` feature

#### waymark-dynamic

- Dynamic navigation mesh generation (`DynamicNavMesh`)
- Multiple collider types:
  - `BoxCollider` (axis-aligned and oriented)
  - `CylinderCollider`
  - `SphereCollider`
  - `CapsuleCollider`
  - `ConvexCollider`
  - `TrimeshCollider`
  - `CompositeCollider`
- Async tile rebuilding (runtime-agnostic via `async-lock`/`futures-lite`)
- Optional Tokio integration via `tokio` feature
- Checkpoint system for incremental updates
- Voxel-based raycasting queries
- Collider serialization support

#### landmark-cli

- `build` command for navmesh generation from OBJ files
- `find-path` command for pathfinding queries
- `--timings` flag to display build timing information
- `bench` subcommand for performance measurement

#### Testing and CI

- Integration tests with C++ cross-validation reference data for nav_test, dungeon,
  and bridge meshes
- Staged pipeline tests for intermediate verification of heightfield, compact
  heightfield, contours, and polymesh stages
- Nextest configuration with profiles for development, CI, local, and release testing
- Examples crate with 5 examples: basic_navmesh, pathfinding, crowd_simulation,
  tilecache_obstacles, serialization
- mdbook documentation site with development guides
- Flamegraph profiling support

### Notes

- Rust 2024 edition (requires Rust 1.92+)
- All library crates compile to `wasm32-unknown-unknown`
- Uses glam for vector math (WASM SIMD compatible)
- Uses postcard for binary serialization (no_std compatible)
- Per-crate error types (no catch-all error enum)
- Public API uses `glam::Vec3` instead of `[f32; 3]` arrays
- 460 tests passing, exact C++ output match for all three test meshes

[Unreleased]: https://github.com/wowemulation-dev/recast-rs/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/wowemulation-dev/recast-rs/releases/tag/v0.1.0
[recast-cpp]: https://github.com/recastnavigation/recastnavigation
