# TODO

Actionable checklist distilled from the
[Resolution Roadmap](docs/src/development/roadmap.md). Each top-level item
maps to a roadmap section. Sub-items are sized as individual PRs or work
sessions. Counts and file paths are from the roadmap; verify before starting.

---

## Phase 1: Library Safety (Critical)

Must be resolved before crates.io publication.

### 1.1 Eliminate `unwrap()`/`expect()` from Library Code

45 calls in non-test library code. Categorize each as infallible, should
return `Result`, or should use a default. See roadmap section 1.1 for
approach.

Verify: `grep -rn 'unwrap()\|expect(' crates/*/src/ --include='*.rs'`
filtered to exclude `#[cfg(test)]` modules should return zero results.

- [x] `detour-tilecache` -- 13 panics on resource exhaustion in `tile_cache.rs`
- [x] `detour` -- 20 calls, includes A* open list pop at `nav_mesh_query.rs:438`
- [x] `recast` -- 9 calls, includes cell index unwraps in `watershed.rs`
- [x] `detour-crowd` -- 2 calls
- [x] `recast-common` -- 1 doctest in `mesh.rs`

### 1.2 Add Structured Error Types

301 error sites use `String` payloads. The `detour` crate's `Status` enum
(22 variants) is converted to strings via `.to_string()` 242 times,
destroying type information. Each crate gets its own error type; the
workspace `Error` in `recast-common` is removed. See roadmap section 1.2
for full type designs.

Execution order (bottom-up):

- [x] `recast-common` -- Replace catch-all `Error` with `MeshError`
- [x] `recast` -- Add `ConfigError`, `BuildError`, `ConvexVolumeError` (58 error sites)
- [x] `detour` -- Add `DetourError`, promote `Status` to error type (242 sites)
- [x] `detour-crowd` -- Add `CrowdError` (2 `InvalidMesh` + `InvalidParam` calls)
- [x] `detour-tilecache` -- Add `TileCacheError` (13 `unwrap` + 2 `InvalidMesh`)
- [x] `detour-dynamic` -- Add `DynamicError` (4 span parsing + 2 job queue errors)
- [x] Delete old `Error` enum from `recast-common`, update `recast-cli` to use `anyhow`

### 1.3 Enable `-D warnings` Locally

- [x] Uncomment `rustflags = ["-D", "warnings"]` in `.cargo/config.toml` and fix any resulting warnings

---

## Phase 2: Usability (High)

Blocks practical adoption.

### 2.1 Add Worked Examples

Zero examples exist. Create `examples/` workspace crate with shared
helpers and 5 examples. See roadmap section 2.1 for API call sequences
and expected output.

- [x] Create `examples/` crate, add to workspace members, add `data/nav_test.obj` (113 KB)
- [x] Write shared helpers in `src/common.rs` (`build_navmesh_from_obj`, `print_stats`)
- [x] `basic_navmesh.rs` -- Recast generation pipeline, print stats
- [x] `pathfinding.rs` -- OBJ to navmesh to A* path to waypoints
- [x] `crowd_simulation.rs` -- Multi-agent crowd with RVO, 100-frame loop
- [x] `tilecache_obstacles.rs` -- Add/remove obstacles, path before/after
- [x] `serialization.rs` -- JSON and postcard round-trip (requires `serialization` feature)

### 2.2 Add Test Fixtures

No reference data. Tests do not validate against known-good output. See
roadmap section 2.2 for reference data format and integration test design.

- [x] Copy test meshes to `test-data/meshes/` (nav_test.obj 113 KB, dungeon.obj 382 KB, bridge.obj 1.5 KB)
- [x] Generate reference data in `test-data/reference/` (JSON with expected counts, bounds, path results)
- [x] Write integration tests validating recast output and detour pathfinding against reference data

### 2.3 Add Benchmarks

No performance data. Use `criterion` with meshes from `test-data/meshes/`.
See roadmap section 2.3 for benchmark specifications.

- [x] `recast/benches/generation.rs` -- build_mesh for bridge (30 tri), nav_test (3,400 tri), dungeon (6,400 tri)
- [x] `detour/benches/pathfinding.rs` -- find_path (short/medium/long), find_straight_path, sliced
- [x] `detour/benches/spatial_queries.rs` -- find_nearest_poly, raycast, find_distance_to_wall, move_along_surface
- [x] `detour-crowd/benches/crowd_update.rs` -- crowd.update with 10/50/100 agents, with/without RVO

### 2.4 Publish to crates.io

Blocked by: 1.1, 1.2, 1.3

See roadmap section 2.4 for per-crate descriptions and publish order.

- [x] Add `description` and `readme` fields to each crate's `Cargo.toml`
- [x] Fix doc coverage: `RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps`
- [x] Dry-run all crates in dependency order: recast-common, recast, detour, detour-crowd, detour-tilecache, detour-dynamic

---

## Phase 3: API Quality (Medium)

Breaking API changes that must happen before 1.0.

### 3.1 Replace C-Style Output Parameters

4 public functions use C-style output params, 12 `detour_common.rs`
functions use `&mut [f32; 3]`, 235 parameters across the workspace use
`&[f32; 3]` instead of `Vec3`. See roadmap section 3.1 for targets and
migration strategy.

- [x] Convert 4 public C-style output params: `bvh_tree::query`, `convex_volume::clip_polygon`, `nav_mesh::store_tile_state`, `nav_mesh_query::move_along_surface`
- [x] Replace 12 `detour_common.rs` vector functions with `glam::Vec3` operations
- [x] Migrate `NavMeshQuery` public methods from `&[f32; 3]` to `Vec3` (170 params in detour)
- [x] Migrate `Crowd` public methods from `&[f32; 3]` to `Vec3` (40 params in detour-crowd)

### 3.2 Reduce Public Field Exposure

22 structs expose 5+ public fields. See roadmap section 3.2 for full
inventory and priority tiers.

- [x] Priority 1 -- Pipeline outputs: Remove 4 legacy PolyMesh duplicates, add accessors to `PolyMesh` and `PolyMeshDetail`, make fields private
- [x] Priority 2 -- Tile structures: Make `MeshTile`, `TileHeader`, `TileCacheLayerHeader` fields private, add accessors
- [x] Priority 3 -- Algorithm internals: Convert `Link`, `Poly`, `DtObstacleCircle`, `RVOAgent`, `FormationAgent` fields to `pub(crate)`
- [x] Priority 4 -- Intermediate pipeline: Convert `Heightfield`, `CompactHeightfield` fields; add `#[non_exhaustive]` to all 9 config structs

### 3.3 Add Builder Patterns for Configuration

Config structs have up to 28 fields with no guided construction.
`DynamicNavMeshConfig` already has 16 `with_*()` methods; follow that
pattern. See roadmap section 3.3 for builder specifications and defaults.

- [ ] `RecastConfigBuilder` -- 18-field config, validating `build()` method
- [ ] `NavMeshCreateParamsBuilder` -- 25-field config with `from_recast()` constructor
- [ ] `AgentParamsBuilder` -- 13-field config for crowd agents
- [ ] Extend `DynamicNavMeshConfig` -- add missing `with_*()` methods and `validate()`

---

## Phase 4: Ecosystem (Lower Priority)

Independent items that improve adoption.

### 4.1 Interactive Demo

C++ and DotRecast both ship interactive demos. Use `egui` + `three-d`
(not Bevy). See roadmap section 4.1 for crate structure and phase
breakdown. Estimated 2,000-2,500 lines.

- [ ] Phase A -- Minimal viewer: window, OBJ wireframe, RecastConfig panel, build + render navmesh (~800 lines)
- [ ] Phase B -- Pathfinding tool: click start/end, find_path + render path (~400 lines)
- [ ] Phase C -- Crowd tool: place agents, set targets, animate updates (~500 lines)
- [ ] Phase D -- Obstacle tool: add/remove obstacles, TileCache rebuild (~300 lines)
- [ ] Phase E -- WASM target: wasm-bindgen entry point, browser file picker (~200 lines)

### 4.2 `no_std` Support

48-63 hours estimated across all crates. Feature-gated `hashbrown` for
`no_std` HashMap/HashSet. See roadmap section 4.2 for per-crate analysis,
std type inventory, and implementation order.

- [ ] `recast-common` -- Easy (2-3h): already has `std` feature, gate HashMap/HashSet
- [ ] `recast` -- Medium (6-8h): gate HashMap/HashSet/BinaryHeap, format! usage, web-time timing
- [ ] `detour` -- Hard (16-20h): gate serialization + file persistence behind `std`, replace collections
- [ ] `detour-crowd` -- Medium (4-6h): replace 4 HashMap instances; blocked by detour
- [ ] `detour-tilecache` -- Medium-Hard (8-10h): gate file persistence, replace collections
- [ ] `detour-dynamic` -- Hard (12-16h): gate async features behind `std`, replace Arc/AtomicU64/mpsc

### 4.3 Framework Integrations

Separate crates, separate release cadence. See roadmap section 4.3 for
design and type specifications.

- [ ] `bevy-recast` -- Plugin with NavmeshSettings, async generation, path request events, gizmo debug (~400-500 lines)
- [ ] macroquad example -- NavMeshContext utility, standalone example (~150-200 lines)

### 4.4 Reduce Unsafe Code

18 unsafe items across 3 files (16 expression blocks + 2 `unsafe impl`).
See roadmap section 4.4 for safe replacements and benchmarking
requirements. Benchmark hot-path replacements before merging.

- [ ] `nav_mesh.rs` `get_tile_and_poly_by_ref_mut` -- correctness issue, return indices or view struct (3-6h)
- [ ] `node_pool.rs` Send/Sync -- remove invalid `unsafe impl` (1h); then refactor to index-based queue (4-8h, 12 items)
- [ ] `nav_mesh.rs` pointer offset -- pass poly index instead of computing from pointer arithmetic (1-2h)
- [ ] `dynamic_tile.rs` `get_unchecked` -- 4 blocks, replace with safe slice indexing, benchmark first (1-2h)
