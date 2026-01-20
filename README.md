# recast-rs

A Rust port of [RecastNavigation](https://github.com/recastnavigation/recastnavigation).

[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue)](LICENSE-MIT)
[![Rust Version](https://img.shields.io/badge/rust-1.92+-orange.svg)](https://www.rust-lang.org)
[![WASM Compatible](https://img.shields.io/badge/WASM-compatible-green.svg)](https://webassembly.org/)

> **Note**: This port is developed for the [WoW Emulation project][wowemu] and
> has not been used outside of that context. The API may change as the project
> matures.

[wowemu]: https://github.com/wowemulation-dev

## Overview

This library provides navigation mesh generation and pathfinding for games.
It is a Rust 2024 edition port of Mikko Mononen's RecastNavigation C++ library.

## Workspace Structure

| Crate | Description | WASM |
|-------|-------------|------|
| `recast-common` | Shared utilities, math, error types | Yes |
| `recast` | Navigation mesh generation | Yes |
| `detour` | Pathfinding and navigation queries | Yes |
| `detour-crowd` | Multi-agent crowd simulation | Yes |
| `detour-tilecache` | Dynamic obstacle management | Yes |
| `detour-dynamic` | Dynamic navmesh support | Yes |
| `recast-cli` | Command-line tool | No |

### Crate Dependencies

- **recast-common**: Base crate with no workspace dependencies
- **recast**: Depends on `recast-common`
- **detour**: Depends on `recast-common` and `recast`
- **detour-crowd**: Depends on `recast-common` and `detour`
- **detour-tilecache**: Depends on `recast-common`, `recast`, and `detour`
- **detour-dynamic**: Depends on `recast-common`, `recast`, `detour`, and `detour-tilecache`

## Features

### Recast - Navigation Mesh Generation

- Voxelization pipeline (heightfield, compact heightfield, contours, polygon mesh)
- Area marking with traversal costs
- Multi-tile mesh generation
- OBJ mesh loading

### Detour - Pathfinding

- A* pathfinding
- Funnel algorithm for path straightening
- Raycast for line-of-sight queries
- Spatial queries (nearest point, random point, polygon height)
- Off-mesh connections
- Multi-tile navigation

### DetourCrowd - Multi-Agent Simulation

- Agent management
- Collision avoidance
- Path following
- Proximity grid for spatial indexing

### DetourTileCache - Dynamic Obstacles

- Runtime obstacle management (cylinder, box, oriented box)
- Tile regeneration
- Compressed tile storage

## Usage

Add the crates you need to your `Cargo.toml`:

```toml
[dependencies]
recast = "0.1"
detour = "0.1"
detour-crowd = "0.1"  # Optional: crowd simulation
```

### Example

```rust
use recast::{RecastBuilder, RecastConfig};
use detour::{NavMesh, NavMeshQuery, QueryFilter};

// Build a navigation mesh
let config = RecastConfig::default();
let builder = RecastBuilder::new(config);
let (poly_mesh, detail_mesh) = builder.build_from_vertices(&vertices, &indices)?;

// Create a NavMesh for pathfinding
let nav_mesh = NavMesh::from_poly_mesh(&poly_mesh, &detail_mesh)?;

// Find a path
let mut query = NavMeshQuery::new(&nav_mesh);
let filter = QueryFilter::default();
let path = query.find_path(start_ref, end_ref, &start_pos, &end_pos, &filter)?;
```

## Building

```bash
# Build all crates
cargo build --workspace

# Run tests
cargo test --workspace

# Run benchmarks
cargo bench

# Build with optimizations
cargo build --release
```

### Feature Flags

- `serialization` - Save/load navigation meshes
- `parallel` - Multi-threaded mesh generation (not WASM-compatible)
- `tokio` - Tokio runtime integration for `detour-dynamic` (not WASM-compatible)

### Platform Support

- Linux (x86_64, musl)
- macOS (x86_64, aarch64)
- Windows (x86_64)
- WebAssembly (wasm32-unknown-unknown)

## WebAssembly Support

All library crates support WebAssembly (`wasm32-unknown-unknown`). Build for WASM with:

```bash
cargo build --target wasm32-unknown-unknown -p recast -p detour
```

### WASM-Compatible Features

| Feature | Native | WASM | Notes |
|---------|--------|------|-------|
| Mesh generation | Yes | Yes | Full support |
| Pathfinding | Yes | Yes | Full support |
| Crowd simulation | Yes | Yes | Full support |
| Dynamic obstacles | Yes | Yes | Full support |
| Async operations | Yes | Yes | Runtime-agnostic via `async-lock` |
| File I/O | Yes | No | Use `std` feature to disable |
| Parallel processing | Yes | No | Disable `parallel` feature |
| Serialization | Yes | Yes | In-memory only on WASM |

### WASM Usage Notes

- **recast-common**: Disable file I/O with `default-features = false`
- **detour**: Serialization works with in-memory buffers
- **detour-tilecache**: Uses pure Rust LZ4 (`lz4_flex`)
- **detour-dynamic**: Async via `async-lock` and `futures-lite` (no tokio required)

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))

## Acknowledgments

- Mikko Mononen for [RecastNavigation](https://github.com/recastnavigation/recastnavigation)
- [DotRecast](https://github.com/ikpil/DotRecast) for implementation reference
- [rerecast](https://github.com/janhohenheim/rerecast) for Rust patterns

## Resources

- [Original RecastNavigation](https://github.com/recastnavigation/recastnavigation)
- [Recast Navigation Documentation](https://recastnav.com/)
- [Digesting Duck Blog](http://digestingduck.blogspot.com/) - Navigation mesh concepts by Mikko Mononen
