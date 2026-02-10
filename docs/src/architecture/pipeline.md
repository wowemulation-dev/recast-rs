# Processing Pipeline

## Recast: Navmesh Generation

The Recast pipeline converts input triangle meshes into navigation polygon
meshes through a series of stages. Each stage transforms the data into a
form closer to the final polygon mesh.

```text
Input Triangles
      │
      ▼
┌─────────────┐
│ Heightfield │  Voxelizes triangles into vertical spans
└──────┬──────┘
       │
       ▼
┌──────────────────┐
│ CompactHeightfield│  Compresses spans, builds connections
└────────┬─────────┘
         │
         ▼
┌────────────────────┐
│ Distance Field +   │  Watershed, monotone, or layer
│ Region Building    │  partitioning of walkable areas
└────────┬───────────┘
         │
         ▼
┌──────────────┐
│ Contour Set  │  Extracts walkable region contours
└──────┬───────┘
       │
       ▼
┌──────────┐
│ PolyMesh │  Generates navigation polygons with adjacency
└────┬─────┘
     │
     ▼
┌────────────────┐
│ PolyMeshDetail │  Adds height detail for accurate sampling
└────────────────┘
```

### Stage Details

**Heightfield**: Rasterizes each input triangle into the voxel grid. Each
column in the grid contains a list of spans representing solid and open space.
Filtering passes remove low-hanging obstacles, ledge spans, and areas with
insufficient headroom.

**CompactHeightfield**: Compresses the span data and builds neighbor connections
between adjacent open spans. This is the working representation for the
remaining pipeline stages.

**Distance Field and Regions**: Computes the distance from each cell to the
nearest boundary, then partitions walkable cells into regions. Three algorithms
are available:
- **Watershed**: produces the best region shapes, uses flood fill from local maxima
- **Monotone**: faster, produces elongated regions aligned to axes
- **Layer**: used for multi-story buildings, partitions by height layers

**Contour Set**: Traces the boundaries of each region to produce simplified
contour polygons. The `max_simplification_error` parameter controls how closely
contours follow the voxel boundaries.

**PolyMesh**: Triangulates contours and merges triangles into convex polygons
(up to `max_vertices_per_polygon` vertices). Builds polygon adjacency for
navigation.

**PolyMeshDetail**: Samples height data from the compact heightfield to add
sub-polygon height detail. This allows accurate height queries on the
navigation mesh.

### Multi-Tile Generation

For large worlds, the pipeline supports tiled generation:

1. Divide the world into a grid of tiles
2. Run the pipeline independently for each tile
3. Merge tiles into a single NavMesh with cross-tile polygon links

The `RecastBuilder` provides `build_layered_heightfield()` for multi-story
support and `build_and_merge_from_multiple_inputs()` for merging.

## Detour: Pathfinding

### NavMesh Structure

The NavMesh is organized as a collection of tiles, each containing:

- **Polygons**: Convex navigation polygons with adjacency links
- **BVH Tree**: Bounding volume hierarchy for spatial queries within a tile
- **Off-Mesh Connections**: Links between non-adjacent polygons (jump points,
  ladders, teleporters)

Polygon references (`PolyRef`) encode three components:
- **Salt**: Generation counter for validity checking
- **Tile index**: Which tile contains the polygon
- **Polygon index**: Index within the tile

### NavMeshQuery

The query system provides:

- **A* Pathfinding** (`find_path`): Finds a path as a sequence of polygon
  references from start to end
- **Funnel Algorithm** (`find_straight_path`): Converts a polygon path into
  waypoints by finding the shortest path through the polygon corridor
- **Raycast**: Line-of-sight queries against the navigation mesh
- **Spatial Queries**: `find_nearest_poly`, `find_polys_around_circle`,
  `find_random_point`, `find_distance_to_wall`, `move_along_surface`
- **Sliced Pathfinding**: `init_sliced_find_path` / `update_sliced_find_path`
  for distributing A* work across frames
- **Hierarchical Pathfinding**: Cluster-based queries for large meshes

### QueryFilter

The `QueryFilter` controls which polygons are traversable and at what cost:

- **Include flags**: Only traverse polygons matching these flags
- **Exclude flags**: Skip polygons matching these flags
- **Area costs**: Per-area traversal cost multipliers (32 areas supported)

Default configuration includes `WALK` polygons and excludes `DISABLED` polygons.
