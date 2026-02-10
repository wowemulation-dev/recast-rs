# recast-common

Foundational types and utilities shared across all crates.

**Lines**: ~2,800 | **WASM**: Yes | **Dependencies**: none (workspace)

## Modules

### Math and Geometry

- **`Vec3`**: Type alias for `glam::Vec3`, used as the 3D position type
- **2D/3D geometry**: Bounding box calculations, grid sizing, area computation
- **Convex hull**: Graham's scan algorithm
- **Ray-triangle intersection**: Moller-Trumbore algorithm
- **Vector operations**: Cross product, dot product, distance, normalization

### Mesh

- **`TriMesh`**: Triangle mesh with OBJ file loading
  - `from_obj(path)`: Load from file (requires `std` feature)
  - `from_obj_str(content)`: Parse from string (WASM-compatible)
  - `calculate_bounds()`: Compute axis-aligned bounding box
- **Mesh simplification**: Quadric error metrics (edge collapse)

### Debug Visualization

- **`DebugDraw`**: Primitives for rendering (lines, triangles, text, spheres,
  bounds, arrows)
- **`DebugVisualize`** trait: Implement on types to enable debug rendering
- **Color schemes**: For polygon flags, agent states, and formations

### Error Types

```rust,ignore
pub enum Error {
    InvalidMesh(String),
    NavMeshGeneration(String),
    Pathfinding(String),
    Recast(String),
    Io(std::io::Error),    // behind `std` feature
    Detour(String),
}

pub type Result<T> = std::result::Result<T, Error>;
```

## Feature Flags

| Feature | Default | Description |
|---------|---------|-------------|
| `std` | Yes | Enables file I/O (`from_obj`, `Error::Io`) |
| `async` | No | Enables tokio integration |

For WASM builds, disable `std`:

```toml
[dependencies]
recast-common = { version = "0.1", default-features = false }
```
