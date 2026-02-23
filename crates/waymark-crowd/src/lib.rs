//! Multi-agent crowd simulation on navigation meshes
//!
//! This crate provides crowd simulation capabilities for managing multiple agents
//! navigating on a shared navigation mesh. It handles local steering, collision
//! avoidance, and path following for groups of agents.
//!
//! # Features
//!
//! - **Agent Management**: Create and manage multiple navigation agents
//! - **Collision Avoidance**: RVO-based (Reciprocal Velocity Obstacles) avoidance
//! - **Path Following**: Smooth path corridor following with local steering
//! - **Proximity Queries**: Spatial grid for efficient neighbor lookups
//! - **Formation Support**: Group movement patterns and formations
//!
//! # Example
//!
//! ```no_run
//! use waymark_crowd::{Crowd, AgentParams};
//! use waymark::{NavMesh, NavMeshParams, PolyRef};
//! use glam::Vec3;
//!
//! # fn example() -> Result<(), Box<dyn std::error::Error>> {
//! let nav_mesh = NavMesh::new(NavMeshParams::default())?;
//! let mut crowd = Crowd::new(&nav_mesh, 25, 0.6);
//!
//! let params = AgentParams::default()
//!     .with_radius(0.6)
//!     .with_height(2.0)
//!     .with_max_acceleration(8.0)
//!     .with_max_speed(3.5);
//! let agent_id = crowd.add_agent(Vec3::new(0.0, 0.0, 0.0), params)?;
//!
//! crowd.request_move_target(agent_id, PolyRef::new(1), Vec3::new(10.0, 0.0, 10.0))?;
//!
//! crowd.update(1.0 / 60.0)?;
//! # Ok(())
//! # }
//! ```
//!
//! # Architecture
//!
//! The crowd system consists of several components:
//!
//! - [`Crowd`]: Main simulation manager
//! - [`PathCorridor`]: Manages agent path state
//! - [`DtLocalBoundary`]: Handles local obstacle detection
//! - [`DtObstacleAvoidanceQuery`]: RVO-based collision avoidance
//! - [`ProximityGrid`]: Spatial indexing for neighbor queries

pub mod crowd;
pub mod crowd_behaviors;
pub mod error;
pub mod formation;
pub mod local_boundary;
pub mod obstacle_avoidance;
pub mod path_corridor;
pub mod proximity_grid;
pub mod rvo;

pub use crowd::*;
pub use crowd_behaviors::*;
pub use error::CrowdError;
pub use formation::*;
pub use local_boundary::*;
pub use obstacle_avoidance::*;
pub use path_corridor::*;
pub use proximity_grid::*;
pub use rvo::*;

#[cfg(test)]
mod crowd_memory_scalability_tests;
