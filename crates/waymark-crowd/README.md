# waymark-crowd

Multi-agent crowd simulation on navigation meshes.

[![Crates.io](https://img.shields.io/crates/v/waymark-crowd.svg)](https://crates.io/crates/waymark-crowd)
[![Documentation](https://docs.rs/waymark-crowd/badge.svg)](https://docs.rs/waymark-crowd)
[![License](https://img.shields.io/crates/l/waymark-crowd.svg)](../LICENSE-MIT)
[![WASM](https://img.shields.io/badge/WASM-compatible-green.svg)](https://webassembly.org/)

## Overview

DetourCrowd provides crowd simulation for managing multiple agents navigating
on a shared navigation mesh. It handles local steering, collision avoidance,
and path following for groups of agents.

This is a Rust port of the DetourCrowd component from [RecastNavigation][recast-cpp].

[recast-cpp]: https://github.com/recastnavigation/recastnavigation

## Features

- **Agent Management**: Create and update multiple navigation agents
- **Collision Avoidance**: RVO-based (Reciprocal Velocity Obstacles) avoidance
- **Path Following**: Smooth path corridor following with local steering
- **Proximity Queries**: Spatial grid for efficient neighbor lookups
- **Formation Support**: Group movement patterns and formations
- **Behavior System**: Customizable agent behaviors

## Example

```rust,ignore
use waymark_crowd::{Crowd, AgentParams};
use glam::Vec3;

// Create a crowd simulation
let mut crowd = Crowd::new(&nav_mesh, 128, 0.6);

// Configure agent parameters
let mut params = AgentParams::default();
params.radius = 0.6;
params.height = 2.0;
params.max_acceleration = 8.0;
params.max_speed = 3.5;

// Add agents (positions are Vec3)
let agent_id = crowd.add_agent(start_pos, params)?;

// Set movement target
crowd.request_move_target(agent_id, target_poly, target_pos)?;

// Update simulation each frame
crowd.update(delta_time)?;

// Get agent state
if let Some(agent) = crowd.get_agent(agent_id) {
    let position = agent.get_pos();
    let velocity = agent.get_vel();
}
```

## Components

| Component | Description |
|-----------|-------------|
| `Crowd` | Main simulation manager |
| `PathCorridor` | Manages agent path state |
| `LocalBoundary` | Local obstacle detection |
| `ObstacleAvoidance` | RVO collision avoidance |
| `ProximityGrid` | Spatial indexing for neighbors |

## WASM Support

This crate is fully compatible with WebAssembly. Build for WASM with:

```bash
cargo build --target wasm32-unknown-unknown -p waymark-crowd
```

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
