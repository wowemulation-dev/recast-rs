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
use detour_crowd::{Crowd, AgentParams};
use glam::Vec3;

let mut crowd = Crowd::new(&nav_mesh, 128, 0.6);

// Add an agent
let mut params = AgentParams::default();
params.radius = 0.6;
params.height = 2.0;
params.max_acceleration = 8.0;
params.max_speed = 3.5;
let agent_id = crowd.add_agent(start_pos, params)?;

// Set target
crowd.request_move_target(agent_id, target_poly, target_pos)?;

// Step simulation
crowd.update(delta_time)?;

// Read agent state
if let Some(agent) = crowd.get_agent(agent_id) {
    let position = agent.get_pos();
    let velocity = agent.get_vel();
}
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
