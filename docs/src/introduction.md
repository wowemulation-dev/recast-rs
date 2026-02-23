# Introduction

recast-rs is a Rust port of [RecastNavigation](https://github.com/recastnavigation/recastnavigation),
providing navigation mesh generation and pathfinding for games and simulations.

The library uses Rust 2024 edition and targets Rust 1.92+. All library crates
compile to `wasm32-unknown-unknown`.

> **Note**: This port is developed for the [WoW Emulation project](https://github.com/wowemulation-dev)
> and has not been used outside that context. The API may change as the project
> matures.

## What is RecastNavigation?

RecastNavigation is Mikko Mononen's C++ library for navigation mesh generation
and pathfinding. It is used in many game engines and middleware. The library
consists of two main parts:

- **Recast**: Generates navigation meshes from input geometry by voxelizing
  triangles into a heightfield, then extracting walkable regions as polygons.
- **Detour**: Provides pathfinding on the generated navigation meshes using
  A* search and the funnel algorithm.

recast-rs ports all five C++ modules and adds a sixth (`waymark-dynamic`) that
is not present in the original.

## Workspace Crates

See the [Architecture Overview](architecture/overview.md) for the workspace layout and
[Crate Dependencies](architecture/crate-dependencies.md) for the dependency graph.

The workspace contains 7 crates: `landmark-common`, `landmark`, `waymark`, `waymark-crowd`,
`waymark-tilecache`, `waymark-dynamic`, and `landmark-cli`. All library crates compile to
WASM (see [WebAssembly](guides/wasm.md)).

## License

Dual-licensed under MIT or Apache-2.0.

## Acknowledgments

- Mikko Mononen for [RecastNavigation](https://github.com/recastnavigation/recastnavigation)
- [DotRecast](https://github.com/ikpil/DotRecast) for implementation reference
- [rerecast](https://github.com/janhohenheim/rerecast) for Rust patterns
