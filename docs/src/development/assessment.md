# Port Assessment

This is an honest assessment of recast-rs compared to the original C++
RecastNavigation and three re-implementations: DotRecast (.NET),
recast-navigation-js (TypeScript/WASM), and rerecast (Rust).

All claims in this document have been verified by auditing the source code.
Specific file paths, line numbers, and counts are provided where relevant.

## Summary

recast-rs is a functionally complete algorithm port that is not yet a usable
product. The algorithms are present. The surrounding infrastructure --
documentation, examples, error handling discipline, tooling, validation -- is
not.

## Strengths

### Scope

No other Rust crate covers the full Recast+Detour stack. rerecast is
Recast-only (no pathfinding). recast-navigation-js wraps C++ via Emscripten.
recast-rs is the only native Rust implementation covering all 5 C++ modules
plus `detour-dynamic` (7 collider types, async, checkpoints), which has no
C++ equivalent.

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

421 tests (417 `#[test]` + 4 `#[tokio::test]`, 430 at runtime with all
features) with 15 dedicated test modules in detour alone. More tests than
the C++ original for the Detour module. More tests than recast-navigation-js
(~9 tests) by two orders of magnitude.

**Verified test counts by crate:**

| Crate | `#[test]` | `#[tokio::test]` | Total |
|-------|-----------|-------------------|-------|
| recast-common | 16 | 0 | 16 |
| recast | 70 | 0 | 70 |
| detour | 247 | 0 | 247 |
| detour-crowd | 39 | 0 | 39 |
| detour-tilecache | 9 | 0 | 9 |
| detour-dynamic | 36 | 4 | 40 |
| recast-cli | 0 | 0 | 0 |
| **Total** | **417** | **4** | **421** |

## Weaknesses

### Compared to C++ Original

- No demo application (C++ has full OpenGL+ImGui interactive demo with 11
  sample tools)
- No benchmarks (C++ has `Bench_rcVector.cpp`; criterion was previously
  declared in recast-rs but removed as unused)
- No 64-bit PolyRef option (`DT_POLYREF64`)
- No pluggable allocators
- 45 `unwrap()`/`expect()` calls in non-test library code (see
  [Technical Debt](#technical-debt) for details)

### Compared to DotRecast (.NET)

- DotRecast: 1,234 commits over 3 years vs 42 commits over ~6 months
- DotRecast ships a full ImGui demo with 11 interactive tools
- DotRecast has BenchmarkDotNet benchmarks
- DotRecast has test fixtures (OBJ meshes, binary navmesh data, voxel files)
- DotRecast is published on NuGet (9 packages) with real users
- DotRecast has a `Detour.Extras` module (A\* import, jump links)

### Compared to recast-navigation-js

- recast-navigation-js has a live demo site, 22+ Storybook stories, 5 example
  projects
- Three.js and PlayCanvas framework integrations
- Version 0.43.0 with stable releases
- Better documentation and onboarding experience

### Compared to rerecast (Rust)

- rerecast has fine-grained error types (`HeightfieldBuilderError`,
  `RasterizationError`, `SpanInsertionError`) vs catch-all `Error` enum with
  string messages
- rerecast uses builder patterns properly (`ConfigBuilder` with fluent API)
- rerecast has zero `unsafe` blocks in core code
- rerecast is published on crates.io with users
- rerecast has 4 worked examples including Bevy integration
- rerecast uses `slotmap` for cache-friendly storage
- rerecast supports `no_std` with `libm`

## Technical Debt

### unwrap()/expect() in Library Code

**Verified count: 45 in non-test code** (44 production + 1 doctest). The
349 additional occurrences in test code are acceptable.

| Crate | Non-test count | Highest-risk location |
|-------|----------------|-----------------------|
| detour | 20 | `nav_mesh_query.rs:438` -- A\* open list pop |
| detour-tilecache | 13 | `tile_cache.rs:302` -- free list exhaustion |
| recast | 9 | `watershed.rs:174` -- cell index unwrap |
| detour-crowd | 2 | |
| recast-common | 1 | doctest in `mesh.rs` |
| detour-dynamic | 0 | |

The 13 occurrences in `detour-tilecache/src/tile_cache.rs` are the most
dangerous: they panic on resource exhaustion (`self.next_free.unwrap()`,
`self.next_free_obstacle.unwrap()`) instead of returning errors.

### String-Based Error Types

The workspace `Error` enum in `recast-common/src/lib.rs` has 6 variants.
5 of 6 use bare `String` as payload. 1 variant (`Pathfinding`) is defined
but never used anywhere in the codebase.

The `detour` crate has a well-defined `Status` enum with 22 variants
(`InvalidParam`, `OutOfMemory`, `PathInvalid`, etc.) but converts these
to strings via `.to_string()` before wrapping in `Error::Detour(String)`.
This pattern appears 242 times, destroying type information.

### Unsafe Code

**Verified count: 18 unsafe items** (16 expression blocks + 2
`unsafe impl`) across 3 files. All can be replaced with safe code.

| File | Blocks | `unsafe impl` | Has SAFETY comment |
|------|--------|---------------|-------------------|
| `detour/src/node_pool.rs` | 10 | 2 | 1 of 12 |
| `detour/src/nav_mesh.rs` | 2 | 0 | 1 of 2 (flawed) |
| `detour-dynamic/src/dynamic_tile.rs` | 4 | 0 | 0 (inline notes only) |

The `nav_mesh.rs` unsafe block at line 1887 claims disjoint mutable
references but creates two `&mut` borrows of overlapping memory
(`&mut MeshTile` overlaps with `&mut Poly` inside its `polys` vec).

The 4 blocks in `dynamic_tile.rs` bypass bounds checks that have already
been performed. Modern compilers can typically prove the safe versions
need no checks.

### C-Style API Patterns

**8 functions** use C-style output parameters (`&mut [T]` + `&mut usize`
count). Most are in private/internal functions mirroring C++ algorithms.
2 are in public detour APIs.

**25 structs** have 5 or more public fields (14 in detour, 11 in recast).
The most exposed:

- `NavMeshCreateParams`: 27 public fields
- `RecastConfig`: 17 public fields
- `PolyMesh`: 17 public fields (4 are redundant "legacy" duplicates)
- `TileHeader`: 16 public fields
- `CompactHeightfield`: 16 public fields

### Missing Infrastructure

| Item | Status | Evidence |
|------|--------|---------|
| Examples directory | Missing | 0 `examples/` dirs, 0 `[[example]]` in Cargo.toml |
| Runnable doc-tests | 1 exists | `TriMesh::from_obj_str` in `mesh.rs:57` |
| Test fixtures | Missing | 0 `.obj`, `.bin`, `.voxels` files in repo |
| Reference validation | Missing | No tests compare output against C++ results |
| Benchmarks | Missing | 0 `benches/` dirs, criterion was removed as unused |
| crates.io publication | Not published | Metadata is structurally complete |
| `no_std` support | None | 0 `#![no_std]` attributes; all crates use `std::collections` |

## Scorecard

| Dimension | Score | Notes |
|-----------|-------|-------|
| Algorithm completeness | 9/10 | Full port + detour-dynamic extension |
| API idiomaticness | 4/10 | Mechanical C++ translation, 8 C-style functions, 25 over-exposed structs |
| Error handling | 5/10 | 45 non-test unwraps (not ~180); string-based errors with 242 `.to_string()` conversions |
| Documentation | 3/10 | READMEs and mdbook, no examples or demos |
| Testing | 6/10 | 421 tests, but no fixtures or reference validation |
| Ecosystem readiness | 2/10 | Not published, no framework integrations |
| Developer experience | 3/10 | No examples, no demo, hard to evaluate |
| CI/Build quality | 9/10 | Cross-platform, WASM verified, coverage |
| Production readiness | 4/10 | Fewer panics than initially estimated, but unstable API |

## What to Preserve

The algorithm port itself is solid. 56,000 lines across 97 files covering all
5 C++ modules is significant work. The `detour-dynamic` crate adds value
beyond the original. WASM support is better than any alternative. The CI setup
is thorough. The foundation is there -- it needs the product work around it.

See [Resolution Roadmap](roadmap.md) for the plan to address these issues.
