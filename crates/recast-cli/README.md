# recast-cli

Command-line tool for navigation mesh generation and pathfinding.

[![Crates.io](https://img.shields.io/crates/v/recast-cli.svg)](https://crates.io/crates/recast-cli)
[![License](https://img.shields.io/crates/l/recast-cli.svg)](../LICENSE-MIT)

## Overview

`recast-cli` provides command-line access to Recast navigation mesh generation
and Detour pathfinding. It can build navmeshes from OBJ files and find paths
on existing meshes.

## Installation

```bash
cargo install recast-cli
```

Or build from source:

```bash
cargo build --release -p recast-cli
```

## Commands

### build

Build a navigation mesh from an OBJ input file.

```bash
recast-cli build --input terrain.obj --output terrain.nav
```

**Options:**

| Option | Default | Description |
|--------|---------|-------------|
| `--input` | required | Input mesh file (OBJ format) |
| `--output` | required | Output navigation mesh file |
| `--cs` | 0.3 | Cell size (horizontal resolution) |
| `--ch` | 0.2 | Cell height (vertical resolution) |
| `--walkable-slope-angle` | 45.0 | Maximum walkable slope in degrees |
| `--walkable-height` | 2 | Minimum ceiling height for walkable areas |
| `--walkable-climb` | 1 | Maximum step height |
| `--walkable-radius` | 1 | Agent radius for obstacle erosion |
| `--max-edge-len` | 12 | Maximum contour edge length |
| `--max-simplification-error` | 1.3 | Contour simplification tolerance |
| `--min-region-area` | 8 | Minimum region size (cells) |
| `--merge-region-area` | 20 | Region merge threshold (cells) |
| `--max-vertices-per-polygon` | 6 | Maximum polygon vertices |
| `--detail-sample-dist` | 6.0 | Detail mesh sample distance |
| `--detail-sample-max-error` | 1.0 | Detail mesh height tolerance |

### find-path

Find a path between two points on a navigation mesh.

```bash
recast-cli find-path --mesh terrain.nav --start 0,0,0 --end 10,0,10
```

**Options:**

| Option | Default | Description |
|--------|---------|-------------|
| `--mesh` | required | Input navigation mesh file |
| `--start` | required | Start position (x,y,z) |
| `--end` | required | End position (x,y,z) |
| `--output` | stdout | Output file for path data |

## Examples

Build a navmesh with custom agent size:

```bash
recast-cli build \
    --input level.obj \
    --output level.nav \
    --walkable-height 3 \
    --walkable-radius 2 \
    --walkable-climb 1
```

Find a path and save to file:

```bash
recast-cli find-path \
    --mesh level.nav \
    --start 5.0,0.0,5.0 \
    --end 45.0,0.0,45.0 \
    --output path.json
```

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
