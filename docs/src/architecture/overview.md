# Architecture Overview

## Workspace Layout

```text
recast-rs/
├── crates/
│   ├── recast-common/      # Shared utilities, math, error types
│   ├── recast/             # Navigation mesh generation
│   ├── detour/             # Pathfinding and navigation queries
│   ├── detour-crowd/       # Multi-agent crowd simulation
│   ├── detour-tilecache/   # Dynamic obstacle management
│   ├── detour-dynamic/     # Dynamic navmesh support
│   └── recast-cli/         # Command-line tool
├── docs/                   # This mdbook documentation
├── .github/workflows/      # CI: ci.yml, cross-build.yml
└── .cargo/config.toml      # Aliases, build settings
```

Unit tests are inline `#[cfg(test)]` modules. Integration tests in `crates/recast/tests/` and `crates/detour/tests/`. Examples in `examples/`. Benchmarks in per-crate `benches/` directories.

## C++ Module Mapping

| C++ Module | Rust Crate | C++ Prefix | Rust Module |
|------------|------------|------------|-------------|
| Recast | `recast` | `rc` | `recast::` |
| Detour | `detour` | `dt` | `detour::` |
| DetourCrowd | `detour-crowd` | `dt` | `detour_crowd::` |
| DetourTileCache | `detour-tilecache` | `dt` | `detour_tilecache::` |
| DebugUtils | `recast-common::debug` | `du` | `recast_common::debug::` |
| (none) | `detour-dynamic` | - | `detour_dynamic::` |

The `detour-dynamic` crate has no C++ equivalent. It extends the original
library with 7 collider types, async operations, and a checkpoint system.

## Key Dependencies

| Purpose | Crate | Notes |
|---------|-------|-------|
| Vector math | `glam` 0.31 | WASM-compatible, game-oriented |
| Error types | `thiserror` 2.0 | Library error derivation |
| Bit flags | `bitflags` 2.10 | PolyFlags, NavMeshFlags |
| Byte order | `byteorder` 1.5 | C++ binary format compatibility |
| Logging | `log` 0.4 | Logging facade |
| Float ordering | `ordered-float` 5.1 | Ordered floats for collections |
| Timing | `web-time` 1.1 | WASM-compatible Instant |
| Async (WASM) | `async-lock` 3.4 + `futures-lite` 2.6 | Runtime-agnostic |
| Serialization | `serde` 1.0 + `postcard` 1.1 | Optional feature |
| Compression | `lz4_flex` 0.12 | Pure Rust LZ4, WASM-compatible |

## Type Mappings from C++

| C++ | Rust |
|-----|------|
| `float` | `f32` |
| `int` | `i32` |
| `unsigned int` | `u32` |
| `unsigned short` | `u16` |
| `unsigned char` | `u8` |
| `float[3]` | `glam::Vec3` or `[f32; 3]` |
| Raw pointer + size | `&[T]` slice |
| Output parameter | Return value or `&mut T` |
| `rcAlloc`/`rcFree` | `Box<T>` or `Vec<T>` |
| `rcContext*` | `RecastContext` + `log` crate |
| Return bool + output param | Return `Result<T, E>` |
| Null pointer check | `Option<T>` |
| Virtual methods | Trait objects |
| `dtPolyQuery` | `PolyQuery` trait |
