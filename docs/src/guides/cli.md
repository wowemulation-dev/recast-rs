# CLI Tool

The `landmark-cli` crate provides a command-line tool for navmesh generation
and pathfinding.

## Installation

```bash
cargo install --path crates/landmark-cli
```

Or build from the workspace:

```bash
cargo build -p landmark-cli --release
```

## Commands

### build

Generate a navigation mesh from an OBJ mesh file.

```bash
landmark-cli build --input mesh.obj --output navmesh.bin
```

Output format is determined by file extension:
- `.json`: JSON format
- `.bin`, `.navmesh`: Binary format (C++ compatible)
- Other/none: Binary format (default)

All `RecastConfig` parameters are available as flags:

```bash
landmark-cli build \
    --input mesh.obj \
    --output navmesh.bin \
    --cs 0.3 \
    --ch 0.2 \
    --walkable-slope-angle 45.0 \
    --walkable-height 2 \
    --walkable-climb 1 \
    --walkable-radius 1 \
    --max-edge-len 12 \
    --max-simplification-error 1.3 \
    --min-region-area 8 \
    --merge-region-area 20 \
    --max-vertices-per-polygon 6 \
    --detail-sample-dist 6.0 \
    --detail-sample-max-error 1.0
```

### find-path

Find a path between two positions on a navigation mesh.

```bash
landmark-cli find-path --mesh navmesh.bin --start 0,0,0 --end 10,0,10
```

Positions are comma-separated `x,y,z` values.

Optional output file:

```bash
landmark-cli find-path --mesh navmesh.bin --start 0,0,0 --end 10,0,10 --output path.csv
```

The output file contains one waypoint per line in `x,y,z` format.

## Workflow

A typical workflow:

```bash
# 1. Generate navmesh from level geometry
landmark-cli build --input level.obj --output level.bin

# 2. Query paths
landmark-cli find-path --mesh level.bin --start 0,1,0 --end 50,1,30

# 3. Adjust parameters and rebuild
landmark-cli build --input level.obj --output level.bin --cs 0.2 --walkable-radius 2
```
