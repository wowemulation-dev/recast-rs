# WebAssembly

All library crates compile to `wasm32-unknown-unknown`. CI verifies this on
every commit.

## Building for WASM

```bash
# Build specific crates for WASM
cargo build --target wasm32-unknown-unknown -p landmark -p waymark

# Build all library crates
cargo build --target wasm32-unknown-unknown \
    -p landmark-common -p landmark -p waymark \
    -p waymark-crowd -p waymark-tilecache -p waymark-dynamic
```

The `landmark-cli` crate does not compile to WASM (it uses file I/O and `clap`).

## Feature Compatibility

| Feature | Native | WASM | Notes |
|---------|--------|------|-------|
| Mesh generation | Yes | Yes | Full support |
| Pathfinding | Yes | Yes | Full support |
| Crowd simulation | Yes | Yes | Full support |
| Dynamic obstacles | Yes | Yes | Full support |
| Async operations | Yes | Yes | Runtime-agnostic via `async-lock` |
| File I/O | Yes | No | Disable `std` feature |
| Serialization | Yes | Yes | In-memory only on WASM |

## Crate-Specific Notes

### landmark-common

Disable the `std` feature to remove file I/O dependencies:

```toml
[dependencies]
landmark-common = { version = "0.1", default-features = false }
```

This removes:
- `TriMesh::from_obj()` (use `TriMesh::from_obj_str()` instead)
- `MeshError::Io` variant

### waymark

Serialization works on WASM using in-memory buffers. File-based save/load
methods require native targets.

### waymark-tilecache

Uses `lz4_flex` (pure Rust) instead of system LZ4. No C dependencies,
works on WASM without configuration.

### waymark-dynamic

Async operations use `async-lock` and `futures-lite` by default. These are
runtime-agnostic and work on WASM without tokio.

The `tokio` feature flag enables tokio integration but is **not**
WASM-compatible:

```toml
# Native only - do not use with WASM
waymark-dynamic = { version = "0.1", features = ["tokio"] }
```

### Timing

The `landmark` crate uses `web-time` instead of `std::time::Instant`. This
provides WASM-compatible timing via `performance.now()` on web targets.

## Dependency Summary

WASM-incompatible features are isolated behind feature flags:

| Feature | Crate | WASM-compatible |
|---------|-------|-----------------|
| `std` | landmark-common | No (file I/O) |
| `tokio` | waymark-dynamic | No (tokio runtime) |
| `serialization` | waymark, waymark-tilecache | Yes (in-memory) |
