# Crate Dependencies

## Dependency Graph

```text
landmark-common          (base crate, no workspace deps)
    │
    ├── landmark         (depends on landmark-common)
    │       │
    │       ├── waymark          (depends on landmark-common, landmark)
    │       │       │
    │       │       ├── waymark-crowd     (depends on landmark-common, waymark)
    │       │       │
    │       │       ├── waymark-tilecache (depends on landmark-common, landmark, waymark)
    │       │       │
    │       │       └── waymark-dynamic   (depends on landmark-common, landmark, waymark)
    │       │
    └── landmark-cli     (depends on landmark-common, landmark, waymark)
```

## landmark-common

Base crate with no workspace dependencies. Provides:

- `Vec3` type alias (`glam::Vec3`)
- `MeshError` type
- `TriMesh` for OBJ loading
- Geometry utilities
- Debug visualization primitives

## landmark

Depends on `landmark-common`. Provides the navmesh generation pipeline from
input triangles to polygon meshes.

## waymark

Depends on `landmark-common` and `landmark`. Provides pathfinding, spatial queries,
and NavMesh data structures.

## waymark-crowd

Depends on `landmark-common` and `waymark`. Provides multi-agent simulation.
Does not depend on `landmark` since it operates on pre-built NavMesh data.

## waymark-tilecache

Depends on `landmark-common`, `landmark`, and `waymark`. Needs `landmark` for tile
regeneration when obstacles change.

## waymark-dynamic

Depends on `landmark-common`, `landmark`, and `waymark`. Combines mesh
generation and pathfinding for dynamic navmesh updates.

## landmark-cli

Binary crate depending on `landmark-common`, `landmark`, and `waymark`. Not published.
