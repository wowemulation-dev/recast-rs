# Port Assessment

This is an assessment of recast-rs compared to the original C++
RecastNavigation and three re-implementations: DotRecast (.NET),
recast-navigation-js (TypeScript/WASM), and rerecast (Rust).

All claims in this document have been verified by auditing the source code.
Specific file paths, line numbers, and counts are provided where relevant.

## Summary

recast-rs is a functionally complete algorithm port with pipeline output
within 1-2% of the C++ reference. The algorithms work. Surrounding
infrastructure (documentation, examples, API migration) is partially
complete but ongoing.

## Strengths

### Scope

No other Rust crate covers the full Recast+Detour stack. rerecast is
Recast-only (no pathfinding). recast-navigation-js wraps C++ via Emscripten.
recast-rs is the only native Rust implementation covering all 5 C++ modules
plus `detour-dynamic` (7 collider types, async, checkpoints), which has no
C++ equivalent.

### Pipeline Accuracy

22 pipeline bugs fixed, bringing navmesh output within 1-2% of C++:

| Mesh | Polygons (Rust/C++) | Detail Verts (Rust/C++) | Ratio |
|------|---------------------|-------------------------|-------|
| nav_test | 530 / 537 | 2,207 / 2,228 | 0.99x |
| dungeon | 213 / 217 | 865 / 868 | 1.00x |
| bridge | 8 / 8 | 32 / 32 | exact |

### WASM

All 6 library crates compile directly to `wasm32-unknown-unknown`, verified
in CI. No Emscripten, no runtime initialization, no async `init()` ceremony.
recast-navigation-js requires async WASM module loading. DotRecast needs the
.NET runtime.

### CI Rigor

10 cross-compilation targets, cargo-nextest, cargo-llvm-cov, cargo-deny,
WASM verification, `-D warnings`. Better CI pipeline than DotRecast and far
better than rerecast or recast-navigation-js.

### Test Count

447 tests (443 `#[test]` + 4 `#[tokio::test]`) with 15 dedicated test
modules in detour alone. Integration tests validate against C++ reference
output.

**Verified test counts by crate:**

| Crate | Unit | Integration | `#[tokio::test]` | Total |
|-------|------|-------------|-------------------|-------|
| recast-common | 16 | 0 | 0 | 16 |
| recast | 69 | 17 | 0 | 86 |
| detour | 247 | 10 | 0 | 257 |
| detour-crowd | 39 | 0 | 0 | 39 |
| detour-tilecache | 9 | 0 | 0 | 9 |
| detour-dynamic | 36 | 0 | 4 | 40 |
| recast-cli | 0 | 0 | 0 | 0 |
| **Total** | **416** | **27** | **4** | **447** |

## Weaknesses

### Compared to C++ Original

- No demo application (C++ has full OpenGL+ImGui interactive demo with 11
  sample tools)
- No 64-bit PolyRef option (`DT_POLYREF64`)
- No pluggable allocators
- NavMeshQuery public API not yet migrated to `Vec3` parameters

### Compared to DotRecast (.NET)

- DotRecast: 1,234 commits over 3 years vs ~65 commits over ~6 months
- DotRecast ships a full ImGui demo with 11 interactive tools
- DotRecast has BenchmarkDotNet benchmarks
- DotRecast is published on NuGet (9 packages) with real users
- DotRecast has a `Detour.Extras` module (A\* import, jump links)

### Compared to recast-navigation-js

- recast-navigation-js has a live demo site, 22+ Storybook stories, 5 example
  projects
- Three.js and PlayCanvas framework integrations
- Version 0.43.0 with stable releases
- Better documentation and onboarding experience

### Compared to rerecast (Rust)

- rerecast uses builder patterns properly (`ConfigBuilder` with fluent API)
- rerecast has zero `unsafe` blocks in core code
- rerecast is published on crates.io with users
- rerecast has 4 worked examples including Bevy integration
- rerecast uses `slotmap` for cache-friendly storage
- rerecast supports `no_std` with `libm`

## Technical Debt

### unwrap()/expect() in Library Code

**Verified count: 2 in non-test code** (both in detour-dynamic job
processing). Down from 45 after Phase 1.1 cleanup.

| Crate | Non-test count | Location |
|-------|----------------|----------|
| detour-dynamic | 2 | `collider_removal_job.rs`, `dynamic_tile_job.rs` |
| All others | 0 | |

### Error Types

Per-crate error types implemented (Phase 1.2 complete). Each crate owns
its errors:

| Crate | Error Types |
|-------|-------------|
| recast-common | `MeshError` |
| recast | `ConfigError`, `BuildError`, `ConvexVolumeError` |
| detour | `DetourError` |
| detour-crowd | `CrowdError` |
| detour-tilecache | `TileCacheError` |
| detour-dynamic | `DynamicError` |

The old catch-all `Error` enum with string payloads has been removed.

### Unsafe Code

**Verified count: 16 unsafe blocks** across 3 files. All can be replaced
with safe code.

| File | Blocks | `unsafe impl` | Has SAFETY comment |
|------|--------|---------------|-------------------|
| `detour/src/node_pool.rs` | 10 | 2 | 1 of 12 |
| `detour/src/nav_mesh.rs` | 2 | 0 | 1 of 2 |
| `detour-dynamic/src/dynamic_tile.rs` | 4 | 0 | 0 (inline notes only) |

### C-Style API Patterns

**4 public functions** use C-style output parameters (`&mut Vec<T>` or
`&mut [u8]` as output buffers): `BvhTree::query`, `ConvexVolume::clip_polygon`,
`NavMesh::store_tile_state`, `NavMeshQuery::move_along_surface`.

The `detour-crowd` crate has been migrated to `Vec3` parameters (30 public
methods). `NavMeshQuery` migration is pending.

### Infrastructure Status

| Item | Status | Notes |
|------|--------|-------|
| Examples directory | 5 examples | `examples/examples/` |
| Runnable doc-tests | 6 exist | Across detour and recast-common |
| Test fixtures | Present | `test-data/meshes/` with 3 OBJ files |
| Reference validation | Present | Integration tests compare against C++ output |
| Benchmarks | 3 crates | `recast/`, `detour/`, `detour-crowd/` benches |
| crates.io publication | Not published | Metadata is structurally complete |
| `no_std` support | None | All crates use `std::collections` |

## Scorecard

| Dimension | Score | Notes |
|-----------|-------|-------|
| Algorithm completeness | 9/10 | Full port + detour-dynamic extension |
| Pipeline accuracy | 9/10 | Within 1-2% of C++ on all test meshes |
| API idiomaticness | 5/10 | detour-crowd migrated to Vec3; NavMeshQuery pending |
| Error handling | 8/10 | Per-crate error types; 2 unwraps remain |
| Documentation | 5/10 | mdbook, READMEs, 5 examples; no demo |
| Testing | 8/10 | 447 tests, integration tests, benchmarks, reference validation |
| Ecosystem readiness | 2/10 | Not published, no framework integrations |
| Developer experience | 4/10 | Examples exist but no demo application |
| CI/Build quality | 9/10 | Cross-platform, WASM verified, coverage |
| Production readiness | 5/10 | Pipeline accurate, API not yet stable |

## What to Preserve

The algorithm port itself is solid. 56,000 lines across 97 files covering all
5 C++ modules is significant work. Pipeline output matches C++ within 1-2%.
The `detour-dynamic` crate adds value beyond the original. WASM support is
better than any alternative. The CI setup is thorough.

See [Resolution Roadmap](roadmap.md) for the plan to address remaining issues.
