# Crate Dependencies

## Dependency Graph

```text
recast-common          (base crate, no workspace deps)
    │
    ├── recast         (depends on recast-common)
    │       │
    │       ├── detour          (depends on recast-common, recast)
    │       │       │
    │       │       ├── detour-crowd     (depends on recast-common, detour)
    │       │       │
    │       │       └── detour-tilecache (depends on recast-common, recast, detour)
    │       │               │
    │       │               └── detour-dynamic   (depends on recast-common, recast,
    │       │                                      detour, detour-tilecache)
    │       │
    │       └───────────────────┘
    │
    └── recast-cli     (depends on all workspace crates)
```

## recast-common

Base crate with no workspace dependencies. Provides:

- `Vec3` type alias (`glam::Vec3`)
- `Error` and `Result` types
- `TriMesh` for OBJ loading
- Geometry utilities
- Debug visualization primitives

## recast

Depends on `recast-common`. Provides the navmesh generation pipeline from
input triangles to polygon meshes.

## detour

Depends on `recast-common` and `recast`. Provides pathfinding, spatial queries,
and NavMesh data structures.

## detour-crowd

Depends on `recast-common` and `detour`. Provides multi-agent simulation.
Does not depend on `recast` since it operates on pre-built NavMesh data.

## detour-tilecache

Depends on `recast-common`, `recast`, and `detour`. Needs `recast` for tile
regeneration when obstacles change.

## detour-dynamic

Depends on `recast-common`, `recast`, `detour`, and `detour-tilecache`.
The most dependent crate, combining mesh generation, pathfinding, and tile
caching for dynamic navmesh updates.

## recast-cli

Binary crate depending on all workspace crates. Not published as a library.
