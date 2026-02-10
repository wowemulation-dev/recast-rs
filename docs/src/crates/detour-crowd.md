# detour-crowd

Multi-agent crowd simulation on navigation meshes.

**Lines**: ~7,100 | **WASM**: Yes | **Depends on**: `recast-common`, `detour`

## Overview

The `detour-crowd` crate manages multiple agents navigating on a shared
navigation mesh. It handles local steering, collision avoidance, and path
following.

## Main Types

### Crowd

The simulation manager. Creates agents, sets targets, and steps the
simulation.

```rust,ignore
use detour_crowd::{Crowd, CrowdConfig, AgentParams};

let config = CrowdConfig::default();
let mut crowd = Crowd::new(&nav_mesh, config)?;

// Add an agent
let params = AgentParams {
    radius: 0.6,
    height: 2.0,
    max_acceleration: 8.0,
    max_speed: 3.5,
    ..Default::default()
};
let agent_id = crowd.add_agent(&start_pos, &params)?;

// Set target
crowd.request_move_target(agent_id, target_poly, &target_pos)?;

// Step simulation
crowd.update(delta_time)?;

// Read agent state
let agent = crowd.get_agent(agent_id)?;
let position = agent.position();
let velocity = agent.velocity();
```

### PathCorridor

Manages per-agent navigation state. Maintains a corridor of polygons from the
agent's current position to the target, and smooths movement along the path.

### DtObstacleAvoidanceQuery

RVO-based (Reciprocal Velocity Obstacles) collision avoidance. Agents compute
velocities that avoid collisions with nearby agents while progressing toward
their targets.

### ProximityGrid

Spatial indexing grid for neighbor queries. Used internally by the crowd
system to find nearby agents for collision avoidance.

### DtLocalBoundary

Detects nearby navmesh boundaries and obstacles for local steering.

### Formations

Group movement patterns. Assign agents to formations and set group targets:

```rust,ignore
use detour_crowd::Formation;

let formation = Formation::new(/* ... */);
crowd.assign_formation(agent_id, &formation)?;
```

### Behaviors

Customizable steering behaviors for agent AI. Agents can have different
behavior profiles affecting how they steer and avoid obstacles.
