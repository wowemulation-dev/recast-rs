# Resolution Roadmap

This document describes how to resolve the issues identified in the
[Port Assessment](assessment.md). Items are ordered by priority: critical
blockers first, then high-severity issues, then improvements.

## Phase 1: Library Safety (Critical)

These issues must be resolved before crates.io publication.

### 1.1 Eliminate `unwrap()`/`expect()` from Library Code

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

### 1.2 Add Structured Error Types

**Problem**: `Error::Recast(String)` and similar variants lose type
information. Callers cannot match on error kinds without parsing strings.

**What rerecast does right**:

```rust
// rerecast: structured, matchable
pub enum RasterizationError {
    PolygonDivisionError(#[from] PolygonDivisionError),
    SpanInsertionError(#[from] SpanInsertionError),
}

pub enum HeightfieldBuilderError {
    ColumnCountTooLarge { width: f32, height: f32 },
}
```

**Target**: Replace string-based variants with structured enums per crate.
Each crate should define its own error type that implements
`std::error::Error`. The workspace `Error` in `recast-common` should become
a thin wrapper or be removed in favor of crate-specific errors.

**Verification**: No `Error` variant should contain a bare `String` as its
only payload.

### 1.3 Enable `-D warnings` Locally

**Problem**: `rustflags = ["-D", "warnings"]` is commented out in
`.cargo/config.toml`. CI enforces this via environment variable, but local
development allows warnings to accumulate.

**Action**: Uncomment the line in `.cargo/config.toml`. Fix any resulting
warnings.

## Phase 2: Usability (High)

These issues block practical adoption.

### 2.1 Add Worked Examples

**Problem**: Zero examples. Users cannot evaluate the library without writing
code from scratch.

**Target**: Create a top-level `examples/` directory with at minimum:

| Example | Description |
|---------|-------------|
| `basic_navmesh.rs` | Load OBJ, generate navmesh, print stats |
| `pathfinding.rs` | Build navmesh, find path, print waypoints |
| `crowd_simulation.rs` | Set up agents, step simulation, print positions |
| `dynamic_obstacles.rs` | Add/remove tilecache obstacles |
| `serialization.rs` | Save and load a navmesh (requires `serialization` feature) |

Each example should be self-contained, compile with `cargo run --example`,
and include comments explaining the steps.

### 2.2 Add Test Fixtures

**Problem**: Tests do not validate against known-good reference output. The
C++ repo and DotRecast both ship test meshes and expected results.

**Action**:

- Copy or recreate test OBJ meshes (dungeon, house, simple terrain)
- Generate reference navmesh data from the C++ implementation
- Add integration tests that build navmeshes from these meshes and compare
  output (polygon count, vertex positions, path results) against reference
  values
- Store fixtures in a `tests/fixtures/` or `test-data/` directory

### 2.3 Add Benchmarks

**Problem**: No performance data. Cannot measure regressions or compare
against C++ FFI alternatives.

**Action**:

- Create a `benches/` directory using `criterion` or `divan`
- Benchmark key operations:
  - Heightfield rasterization (varying triangle counts)
  - Navmesh generation (small, medium, large meshes)
  - A\* pathfinding (short, medium, long paths)
  - Spatial queries (find_nearest_poly, raycast)
  - Crowd simulation step (varying agent counts)

### 2.4 Publish to crates.io

**Blocked by**: Phase 1 completion.

**Action**:

- Finalize crate metadata (description, keywords, categories, repository URL)
- Ensure all public items have doc comments
- Run `cargo publish --dry-run` for each crate
- Publish in dependency order: recast-common, recast, detour, detour-crowd,
  detour-tilecache, detour-dynamic

## Phase 3: API Quality (Medium)

These issues improve the developer experience.

### 3.1 Replace C-Style Output Parameters

**Problem**: Functions like `find_path` use output arrays and count pointers
instead of returning owned collections.

**Current**:

```rust
fn find_path(&mut self, start: PolyRef, end: PolyRef,
    start_pos: &[f32], end_pos: &[f32], filter: &QueryFilter,
    path: &mut [PolyRef], path_count: &mut usize) -> Status;
```

**Target**:

```rust
fn find_path(&mut self, start: PolyRef, end: PolyRef,
    start_pos: Vec3, end_pos: Vec3,
    filter: &QueryFilter) -> Result<Vec<PolyRef>>;
```

This is a breaking API change. Do it before 1.0.

### 3.2 Reduce Public Field Exposure

**Problem**: Structs like `Node`, `Poly`, `MeshTile` expose all fields
publicly. This prevents future internal changes without breaking the API.

**Action**: Make fields private, add accessor methods. Use `#[non_exhaustive]`
on public structs where appropriate.

### 3.3 Add Builder Patterns for Configuration

**Problem**: `RecastConfig` is a flat struct with 18+ public fields. rerecast
uses a `ConfigBuilder` with documented methods and validation.

**Action**: Add a `RecastConfigBuilder` with sensible defaults, validation,
and doc comments explaining each parameter's effect.

## Phase 4: Ecosystem (Lower Priority)

### 4.1 Interactive Demo

DotRecast and the C++ original both ship interactive demos. An `egui`-based
demo would demonstrate all crate capabilities and serve as a visual debugging
tool. This is significant work (~2,000-5,000 lines) but high impact for
adoption.

### 4.2 `no_std` Support

rerecast supports `no_std` via `libm`. For embedded or bare-metal targets,
recast-rs could add `no_std` support to `recast-common` and `recast`. The
`detour` family crates use `Vec` and `BinaryHeap` extensively, making
`no_std` harder there.

### 4.3 Framework Integrations

Bevy, macroquad, or other Rust game framework integrations would lower the
adoption barrier. These should be separate crates (like rerecast's
`bevy_rerecast`) to avoid coupling.

### 4.4 Reduce Unsafe Code

The 10+ `unsafe` blocks in `detour/src/node_pool.rs` implement a raw pointer
priority queue. Investigate replacing with safe alternatives (e.g., `slotmap`
or `Vec`-based priority queue with generation indices). Benchmark before and
after to ensure no regression.

## Verification

After completing each phase, run:

```bash
# Phase 1 verification
grep -rn 'unwrap()\|expect(' crates/*/src/ --include='*.rs' | grep -v '#\[cfg(test)\]' | grep -v 'mod tests'
cargo fmt --all && cargo lint && cargo test-all
cargo check --workspace --no-default-features

# Phase 2 verification
cargo run --example basic_navmesh
cargo run --example pathfinding
cargo bench
cargo publish --dry-run -p recast-common

# Phase 3 verification
cargo doc --workspace --no-deps
cargo fmt --all && cargo lint && cargo test-all
```
