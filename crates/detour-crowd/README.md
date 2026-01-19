# detour-crowd

Multi-agent crowd simulation on navigation meshes.

[![Crates.io](https://img.shields.io/crates/v/detour-crowd.svg)](https://crates.io/crates/detour-crowd)
[![Documentation](https://docs.rs/detour-crowd/badge.svg)](https://docs.rs/detour-crowd)
[![License](https://img.shields.io/crates/l/detour-crowd.svg)](../LICENSE-MIT)

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

```rust
use detour_crowd::{Crowd, CrowdConfig, AgentParams};

// Create a crowd simulation
let config = CrowdConfig::default();
let mut crowd = Crowd::new(&nav_mesh, config)?;

// Configure agent parameters
let params = AgentParams {
    radius: 0.6,
    height: 2.0,
    max_acceleration: 8.0,
    max_speed: 3.5,
    collision_query_range: 12.0,
    path_optimization_range: 30.0,
    ..Default::default()
};

// Add agents
let agent_id = crowd.add_agent(&start_pos, &params)?;

// Set movement target
crowd.request_move_target(agent_id, target_poly, &target_pos)?;

// Update simulation each frame
crowd.update(delta_time)?;

// Get agent state
let agent = crowd.get_agent(agent_id)?;
let position = agent.position();
let velocity = agent.velocity();
```

## Components

| Component | Description |
|-----------|-------------|
| `Crowd` | Main simulation manager |
| `PathCorridor` | Manages agent path state |
| `LocalBoundary` | Local obstacle detection |
| `ObstacleAvoidance` | RVO collision avoidance |
| `ProximityGrid` | Spatial indexing for neighbors |

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](../../LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](../../LICENSE-APACHE))
