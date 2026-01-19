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
//! ```rust,ignore
//! use detour_crowd::{Crowd, CrowdConfig, AgentParams};
//! use detour::NavMesh;
//!
//! // Create a crowd simulation
//! let config = CrowdConfig::default();
//! let mut crowd = Crowd::new(&nav_mesh, config)?;
//!
//! // Add an agent
//! let params = AgentParams {
//!     radius: 0.6,
//!     height: 2.0,
//!     max_acceleration: 8.0,
//!     max_speed: 3.5,
//!     ..Default::default()
//! };
//! let agent_id = crowd.add_agent(&start_pos, &params)?;
//!
//! // Set agent target
//! crowd.request_move_target(agent_id, target_poly, &target_pos)?;
//!
//! // Update simulation
//! crowd.update(delta_time)?;
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
pub mod formation;
pub mod local_boundary;
pub mod obstacle_avoidance;
pub mod path_corridor;
pub mod proximity_grid;
pub mod rvo;

pub use crowd::*;
pub use crowd_behaviors::*;
pub use formation::*;
pub use local_boundary::*;
pub use obstacle_avoidance::*;
pub use path_corridor::*;
pub use proximity_grid::*;
pub use rvo::*;

#[cfg(test)]
mod crowd_memory_scalability_tests;
