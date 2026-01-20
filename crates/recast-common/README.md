# recast-common

Common utilities and types shared by the recast-rs workspace crates.

[![Crates.io](https://img.shields.io/crates/v/recast-common.svg)](https://crates.io/crates/recast-common)
[![Documentation](https://docs.rs/recast-common/badge.svg)](https://docs.rs/recast-common)
[![License](https://img.shields.io/crates/l/recast-common.svg)](../LICENSE-MIT)
[![WASM](https://img.shields.io/badge/WASM-compatible-green.svg)](https://webassembly.org/)

## Overview

This crate provides foundational types and utilities used by `recast`, `detour`,
and related crates. It is not intended for direct use by end users.

## Contents

- **Error Types**: Common error definitions for the workspace
- **Math Utilities**: Vector operations and geometric calculations
- **Mesh Utilities**: Triangle mesh processing helpers
- **Debug Utilities**: Visualization and debugging support

## Features

- `std` (default) - Enables file I/O operations (`TriMesh::from_obj()`)

## WASM Support

This crate is fully compatible with WebAssembly. For WASM builds, file I/O is
automatically disabled. Use `TriMesh::from_obj_str()` to parse OBJ content from
strings instead of files.

## Usage

This crate is an internal dependency of the recast-rs workspace. Add the
specific crate you need instead:

```toml
[dependencies]
recast = "0.1"      # For navmesh generation
detour = "0.1"      # For pathfinding
```

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
