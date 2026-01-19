//! Crowd management for Detour
//!
//! This module contains the crowd management functionality, which
//! allows for efficient pathfinding and movement of multiple agents.

use std::f32;

use super::PathCorridor;
use super::formation::{FormationConfig, FormationManager, FormationRole};
use super::proximity_grid::{GridAgent, ProximityGrid};
use super::rvo::{
    RVOConfig, RVOSimulator, position_3d_to_2d, velocity_2d_to_3d, velocity_3d_to_2d,
};
use detour::{NavMesh, NavMeshQuery, PolyRef, QueryFilter, Status};
use recast_common::{Error, Result};

/// Maximum number of agents in a crowd
#[allow(dead_code)]
const MAX_CROWD_AGENTS: usize = 512;

/// The maximum number of neighbors that a crowd agent can take into account
/// for steering decisions.
pub const DT_CROWDAGENT_MAX_NEIGHBOURS: usize = 6;

/// The maximum number of corners a crowd agent will look ahead in the path.
/// This value is used for sizing the crowd agent corner buffers.
/// Due to the behavior of the crowd manager, the actual number of useful
/// corners will be one less than this number.
pub const DT_CROWDAGENT_MAX_CORNERS: usize = 4;

/// The maximum number of crowd avoidance configurations supported by the
/// crowd manager.
pub const DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS: usize = 8;

/// The maximum number of query filter types supported by the crowd manager.
pub const DT_CROWD_MAX_QUERY_FILTER_TYPE: usize = 16;

/// Default radius of an agent
const DEFAULT_AGENT_RADIUS: f32 = 0.6;

/// Default height of an agent
const DEFAULT_AGENT_HEIGHT: f32 = 2.0;

/// Default max acceleration of an agent
const DEFAULT_AGENT_MAX_ACCELERATION: f32 = 8.0;

/// Default max speed of an agent
const DEFAULT_AGENT_MAX_SPEED: f32 = 3.5;

/// The type of navigation mesh polygon the agent is currently traversing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CrowdAgentState {
    /// The agent is not in a valid state.
    #[default]
    Invalid = 0,
    /// The agent is traversing a normal navigation mesh polygon.
    Walking = 1,
    /// The agent is traversing an off-mesh connection.
    OffMesh = 2,
}

/// Move request state for agents
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MoveRequestState {
    #[default]
    None = 0,
    Failed,
    Valid,
    Requesting,
    WaitingForQueue,
    WaitingForPath,
    Velocity,
}

/// Agent state in the crowd
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgentState {
    /// Agent is invalid (not active)
    Invalid,
    /// Agent is waiting for a path
    Waiting,
    /// Agent is traversing a path
    Active,
    /// Agent has reached its destination
    Completed,
    /// Agent failed to find a path
    Failed,
}

/// Update flags for crowd agents (matching C++ UpdateFlags)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct UpdateFlags(pub u8);

impl UpdateFlags {
    pub const ANTICIPATE_TURNS: UpdateFlags = UpdateFlags(1);
    pub const OBSTACLE_AVOIDANCE: UpdateFlags = UpdateFlags(2);
    pub const SEPARATION: UpdateFlags = UpdateFlags(4);
    pub const OPTIMIZE_VIS: UpdateFlags = UpdateFlags(8);
    pub const OPTIMIZE_TOPO: UpdateFlags = UpdateFlags(16);

    /// Check if flags contain the specified flag
    pub fn contains(&self, other: UpdateFlags) -> bool {
        self.0 & other.0 != 0
    }
}

impl Default for UpdateFlags {
    fn default() -> Self {
        UpdateFlags(
            UpdateFlags::ANTICIPATE_TURNS.0
                | UpdateFlags::OBSTACLE_AVOIDANCE.0
                | UpdateFlags::SEPARATION.0,
        )
    }
}

// Legacy AgentParams for backwards compatibility
/// Agent parameters for crowd
#[derive(Debug, Clone)]
pub struct AgentParams {
    /// Radius of the agent
    pub radius: f32,
    /// Height of the agent
    pub height: f32,
    /// Maximum acceleration of the agent
    pub max_acceleration: f32,
    /// Maximum speed of the agent
    pub max_speed: f32,
    /// Collision query range
    pub collision_query_range: f32,
    /// Path optimization range
    pub path_optimization_range: f32,
    /// Whether the agent can move through other agents
    pub separate: bool,
    /// Update flags for the agent
    pub update_flags: UpdateFlags,
    /// Obstacle avoidance type (0 = disabled, 1-3 quality levels)
    pub obstacle_avoidance_type: i32,
    /// Query filter type for the agent
    pub query_filter_type: i32,
    /// User data for the agent
    pub user_data: Option<usize>,
    /// Whether to use RVO for collision avoidance
    pub use_rvo: bool,
    /// RVO configuration for this agent
    pub rvo_config: RVOConfig,
}

impl Default for AgentParams {
    fn default() -> Self {
        Self {
            radius: DEFAULT_AGENT_RADIUS,
            height: DEFAULT_AGENT_HEIGHT,
            max_acceleration: DEFAULT_AGENT_MAX_ACCELERATION,
            max_speed: DEFAULT_AGENT_MAX_SPEED,
            collision_query_range: 12.0,
            path_optimization_range: 30.0,
            separate: true,
            update_flags: UpdateFlags::default(),
            obstacle_avoidance_type: 3, // Best quality
            query_filter_type: 0,
            user_data: None,
            use_rvo: true, // Enable RVO by default
            rvo_config: RVOConfig::default(),
        }
    }
}

// Legacy CrowdAgent for backwards compatibility
/// Agent in the crowd (legacy)
#[derive(Debug, Clone)]
pub struct CrowdAgent {
    /// Agent parameters
    params: AgentParams,
    /// Current state of the agent
    state: AgentState,
    /// Current position of the agent
    pos: [f32; 3],
    /// Desired velocity of the agent
    desired_vel: [f32; 3],
    /// Current velocity of the agent
    vel: [f32; 3],
    /// Current acceleration of the agent
    accel: [f32; 3],
    /// Target position of the agent
    target: [f32; 3],
    /// Target polygon reference
    target_ref: PolyRef,
    /// Path corridor for the agent
    corridor: PathCorridor,
    /// Whether the agent needs a target update
    target_updated: bool,
    /// Whether the agent is active
    active: bool,
    /// Agent ID
    id: usize,
    /// RVO agent ID (if using RVO)
    rvo_agent_id: Option<usize>,
}

impl CrowdAgent {
    /// Creates a new crowd agent
    pub fn new(params: AgentParams, id: usize) -> Self {
        Self {
            params,
            state: AgentState::Invalid,
            pos: [0.0, 0.0, 0.0],
            desired_vel: [0.0, 0.0, 0.0],
            vel: [0.0, 0.0, 0.0],
            accel: [0.0, 0.0, 0.0],
            target: [0.0, 0.0, 0.0],
            target_ref: PolyRef::new(0),
            corridor: PathCorridor::new(),
            target_updated: false,
            active: false,
            id,
            rvo_agent_id: None,
        }
    }

    /// Resets the agent
    pub fn reset(&mut self) {
        self.state = AgentState::Invalid;
        self.pos = [0.0, 0.0, 0.0];
        self.desired_vel = [0.0, 0.0, 0.0];
        self.vel = [0.0, 0.0, 0.0];
        self.accel = [0.0, 0.0, 0.0];
        self.target = [0.0, 0.0, 0.0];
        self.target_ref = PolyRef::new(0);
        self.corridor = PathCorridor::new();
        self.target_updated = false;
        self.active = false;
    }

    /// Gets the agent's position
    pub fn get_pos(&self) -> [f32; 3] {
        self.pos
    }

    /// Gets the agent's velocity
    pub fn get_vel(&self) -> [f32; 3] {
        self.vel
    }

    /// Gets the agent's target
    pub fn get_target(&self) -> [f32; 3] {
        self.target
    }

    /// Gets the agent's state
    pub fn get_state(&self) -> AgentState {
        self.state
    }

    /// Gets the agent's parameters
    pub fn get_params(&self) -> &AgentParams {
        &self.params
    }

    /// Gets the agent's ID
    pub fn get_id(&self) -> usize {
        self.id
    }

    /// Gets whether the agent is active
    pub fn is_active(&self) -> bool {
        self.active
    }
}

/// Legacy crowd manager for Detour
#[derive(Debug)]
pub struct Crowd<'a> {
    /// Reference to the navigation mesh
    /// TODO: Currently stored but not used directly - access through query instead
    #[allow(dead_code)]
    nav_mesh: &'a NavMesh,
    /// Navigation mesh query
    query: NavMeshQuery<'a>,
    /// Maximum number of agents
    max_agents: usize,
    /// Current agents
    agents: Vec<Option<CrowdAgent>>,
    /// Active agents
    active_agents: Vec<usize>,
    /// Query filters
    filters: Vec<QueryFilter>,
    /// Time delta for updates
    delta_time: f32,
    /// Maximum agent radius
    max_agent_radius: f32,
    /// RVO simulator for collision avoidance
    rvo_simulator: Option<RVOSimulator>,
    /// Formation manager for organizing agents into formations
    formation_manager: FormationManager,
    /// Proximity grid for efficient spatial queries
    proximity_grid: ProximityGrid,
}

impl<'a> Crowd<'a> {
    /// Creates a new crowd manager
    pub fn new(nav_mesh: &'a NavMesh, max_agents: usize, max_agent_radius: f32) -> Self {
        let query = NavMeshQuery::new(nav_mesh);

        // Create agent slots
        let mut agents = Vec::with_capacity(max_agents);
        for _i in 0..max_agents {
            agents.push(None);
        }

        // Create a default query filter
        let default_filter = QueryFilter::default();

        // Calculate appropriate cell size for proximity grid
        // Use 2x the max agent radius to ensure good spatial distribution
        let grid_cell_size = (max_agent_radius * 4.0).max(2.0);

        Self {
            nav_mesh,
            query,
            max_agents,
            agents,
            active_agents: Vec::new(),
            filters: vec![default_filter],
            delta_time: 0.1,
            max_agent_radius,
            rvo_simulator: None,
            formation_manager: FormationManager::new(),
            proximity_grid: ProximityGrid::new(grid_cell_size),
        }
    }

    /// Adds an agent to the crowd
    pub fn add_agent(&mut self, pos: [f32; 3], params: AgentParams) -> Result<usize> {
        // Find a free slot
        let mut slot = usize::MAX;
        for i in 0..self.max_agents {
            if self.agents[i].is_none() {
                slot = i;
                break;
            }
        }

        if slot == usize::MAX {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Find nearest polygon to position
        let ext = [
            params.radius * 2.0,
            params.height * 0.5,
            params.radius * 2.0,
        ];

        let filter = &self.filters[0]; // Use default filter
        let (nearest_ref, nearest_pos) = match self.query.find_nearest_poly(&pos, &ext, filter) {
            Ok((reference, position)) => (reference, position),
            Err(_) => {
                // Try with larger extents
                let ext = [params.radius * 3.0, params.height, params.radius * 3.0];
                match self.query.find_nearest_poly(&pos, &ext, filter) {
                    Ok((reference, position)) => (reference, position),
                    Err(_) => return Err(Error::Detour(Status::InvalidParam.to_string())),
                }
            }
        };

        // Update max agent radius before moving params
        self.max_agent_radius = self.max_agent_radius.max(params.radius);

        // Create the agent
        let mut agent = CrowdAgent::new(params.clone(), slot);

        // Initialize the agent
        agent.state = AgentState::Active;
        agent.pos = nearest_pos;
        agent.desired_vel = [0.0, 0.0, 0.0];
        agent.vel = [0.0, 0.0, 0.0];
        agent.active = true;

        // Init path corridor
        agent.corridor.reset(nearest_ref, nearest_pos);

        // Handle RVO integration
        if params.use_rvo {
            // Initialize RVO simulator if needed
            if self.rvo_simulator.is_none() {
                self.rvo_simulator = Some(RVOSimulator::new());
            }

            if let Some(rvo_sim) = &mut self.rvo_simulator {
                let rvo_pos_2d = position_3d_to_2d(&nearest_pos);
                let rvo_agent_id = rvo_sim.add_agent_with_config(rvo_pos_2d, params.rvo_config);
                agent.rvo_agent_id = Some(rvo_agent_id);
            }
        }

        // Add the agent to the proximity grid
        let grid_agent = GridAgent {
            id: slot,
            pos: nearest_pos,
            radius: params.radius,
        };
        let _ = self.proximity_grid.update_agent(grid_agent);

        // Add the agent to the crowd
        self.agents[slot] = Some(agent);
        self.active_agents.push(slot);

        Ok(slot)
    }

    /// Removes an agent from the crowd
    pub fn remove_agent(&mut self, agent_idx: usize) -> Result<()> {
        if agent_idx >= self.max_agents {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if self.agents[agent_idx].is_none() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Handle RVO agent removal
        if let Some(agent) = &self.agents[agent_idx] {
            if let Some(rvo_agent_id) = agent.rvo_agent_id {
                if let Some(rvo_sim) = &mut self.rvo_simulator {
                    let _ = rvo_sim.remove_agent(rvo_agent_id);
                }
            }
        }

        // Remove from proximity grid
        self.proximity_grid.remove_agent(agent_idx);

        // Remove from active agents
        if let Some(pos) = self.active_agents.iter().position(|&idx| idx == agent_idx) {
            self.active_agents.swap_remove(pos);
        }

        // Reset the agent slot
        self.agents[agent_idx] = None;

        // Update max agent radius
        if self.max_agent_radius > DEFAULT_AGENT_RADIUS {
            self.max_agent_radius = DEFAULT_AGENT_RADIUS;

            // Recalculate max agent radius
            for agent in self.agents.iter().flatten() {
                self.max_agent_radius = self.max_agent_radius.max(agent.params.radius);
            }
        }

        Ok(())
    }

    /// Requests a move target for an agent
    pub fn request_move_target(
        &mut self,
        agent_idx: usize,
        target_ref: PolyRef,
        target_pos: [f32; 3],
    ) -> Result<()> {
        if agent_idx >= self.max_agents {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let agent = match self.agents[agent_idx].as_mut() {
            Some(agent) => agent,
            None => return Err(Error::Detour(Status::InvalidParam.to_string())),
        };

        // Initialize request
        agent.target = target_pos;
        agent.target_ref = target_ref;
        agent.target_updated = true;

        // If the agent is inactive, activate it
        if !agent.active {
            agent.active = true;
            self.active_agents.push(agent_idx);
        }

        Ok(())
    }

    /// Updates all agents in the crowd
    pub fn update(&mut self, delta_time: f32) -> Result<()> {
        self.delta_time = delta_time;

        // Reset dynamic avoidance grid
        // TODO: Implement dynamic avoidance grid

        // Update all active agents
        for &agent_idx in &self.active_agents.clone() {
            // Find paths for agents that need them
            if let Some(agent) = self.agents[agent_idx].as_ref() {
                if agent.target_updated {
                    let filter_idx = agent.params.query_filter_type as usize;
                    let filter = self.get_filter(filter_idx)?.clone();

                    // Check if we have a valid path
                    if agent.corridor.get_path().is_empty() {
                        continue; // Skip this agent if no valid path
                    }

                    let target_ref = agent.target_ref;
                    let target_pos = agent.target;

                    // Update agent after path finding
                    if let Some(agent) = self.agents[agent_idx].as_mut() {
                        agent.target_updated = false;

                        match agent.corridor.find_path(
                            &mut self.query,
                            target_ref,
                            target_pos,
                            &filter,
                        ) {
                            Ok(()) => {
                                agent.state = AgentState::Active;
                            }
                            Err(_) => {
                                agent.state = AgentState::Failed;
                            }
                        }
                    }
                }
            }

            // Update path corridor for active agents
            if let Some(agent) = self.agents[agent_idx].as_ref() {
                if agent.state == AgentState::Active && !agent.corridor.get_path().is_empty() {
                    let filter_idx = agent.params.query_filter_type as usize;
                    let filter = self.get_filter(filter_idx)?.clone();
                    let agent_pos = agent.pos;

                    // Update corridor advance in a separate scope to avoid borrow issues
                    if let Some(agent) = self.agents[agent_idx].as_mut() {
                        // First try to optimize the path corridor
                        if let Err(e) = agent.corridor.optimize_path(&mut self.query, &filter) {
                            // If optimization fails, log but continue - it's not critical
                            eprintln!(
                                "DEBUG: Corridor optimization failed for agent {}: {:?}",
                                agent_idx, e
                            );
                        }

                        // Advance the corridor towards the target
                        match agent.corridor.advance(agent_pos, &mut self.query, &filter) {
                            Ok(reached_target) => {
                                if reached_target {
                                    // Agent has reached their target
                                    agent.state = AgentState::Completed;
                                    agent.target_ref = PolyRef::new(0);
                                }
                            }
                            Err(e) => {
                                // If corridor advance fails, try to recover by moving position only
                                eprintln!(
                                    "DEBUG: Corridor advance failed for agent {}: {:?}",
                                    agent_idx, e
                                );

                                // Fallback: just update corridor position without advancing
                                if let Err(e2) = agent.corridor.move_position(
                                    agent_pos,
                                    &mut self.query,
                                    &filter,
                                ) {
                                    eprintln!(
                                        "DEBUG: Corridor move_position fallback also failed for agent {}: {:?}",
                                        agent_idx, e2
                                    );
                                    // If both fail, mark agent as failed
                                    agent.state = AgentState::Failed;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Update formations
        self.update_formations(delta_time)?;

        // Calculate steering (includes formation steering)
        if let Err(e) = self.calculate_steering() {
            eprintln!("DEBUG: calculate_steering failed: {:?}", e);
            return Err(e);
        }

        // Calculate velocity
        if let Err(e) = self.calculate_velocity(delta_time) {
            eprintln!("DEBUG: calculate_velocity failed: {:?}", e);
            return Err(e);
        }

        // Move agents
        if let Err(e) = self.move_agents(delta_time) {
            eprintln!("DEBUG: move_agents failed: {:?}", e);
            return Err(e);
        }

        // Clean up inactive agents
        self.purge_inactive_agents();

        Ok(())
    }

    /// Calculates steering for all active agents
    fn calculate_steering(&mut self) -> Result<()> {
        // Collect agent data first to avoid borrow checker issues
        let active_agents = self.active_agents.clone();

        for &agent_idx in &active_agents {
            // Skip if agent doesn't exist or isn't active
            let should_process = self.agents[agent_idx]
                .as_ref()
                .map(|a| a.state == AgentState::Active)
                .unwrap_or(false);

            if !should_process {
                continue;
            }

            // Get agent data
            let agent_data = self.agents[agent_idx]
                .as_ref()
                .map(|a| {
                    (
                        a.corridor.get_path().to_vec(),
                        a.pos,
                        a.params.radius,
                        a.params.max_speed,
                        a.corridor.get_target(),
                    )
                })
                .unwrap();

            let (path, agent_pos, _agent_radius, max_speed, target_pos) = agent_data;

            // Calculate steering direction
            let mut steer_dir = [0.0; 3];

            // Calculate steer direction based on the remaining path
            if !path.is_empty() {
                // Simple steering: move directly towards the target
                steer_dir[0] = target_pos[0] - agent_pos[0];
                steer_dir[1] = target_pos[1] - agent_pos[1];
                steer_dir[2] = target_pos[2] - agent_pos[2];

                // Normalize direction
                let dist = (steer_dir[0] * steer_dir[0]
                    + steer_dir[1] * steer_dir[1]
                    + steer_dir[2] * steer_dir[2])
                    .sqrt();
                if dist > 0.0001 {
                    steer_dir[0] /= dist;
                    steer_dir[1] /= dist;
                    steer_dir[2] /= dist;
                }
            }

            // Apply formation steering if agent is in a formation
            let formation_positions = self.formation_manager.get_formation_positions();
            if let Some(formation_pos) = formation_positions.get(&agent_idx) {
                // Calculate formation steering
                let formation_steer = [
                    formation_pos[0] - agent_pos[0],
                    formation_pos[1] - agent_pos[1],
                    formation_pos[2] - agent_pos[2],
                ];

                let formation_dist = (formation_steer[0] * formation_steer[0]
                    + formation_steer[1] * formation_steer[1]
                    + formation_steer[2] * formation_steer[2])
                    .sqrt();

                if formation_dist > 0.001 {
                    // Normalize formation steering
                    let formation_steer_normalized = [
                        formation_steer[0] / formation_dist,
                        formation_steer[1] / formation_dist,
                        formation_steer[2] / formation_dist,
                    ];

                    // Blend path steering with formation steering
                    let formation_weight = 0.6; // Formation has moderate influence
                    let path_weight = 1.0 - formation_weight;

                    steer_dir[0] = path_weight * steer_dir[0]
                        + formation_weight * formation_steer_normalized[0];
                    steer_dir[1] = path_weight * steer_dir[1]
                        + formation_weight * formation_steer_normalized[1];
                    steer_dir[2] = path_weight * steer_dir[2]
                        + formation_weight * formation_steer_normalized[2];

                    // Renormalize blended steering
                    let blended_length = (steer_dir[0] * steer_dir[0]
                        + steer_dir[1] * steer_dir[1]
                        + steer_dir[2] * steer_dir[2])
                        .sqrt();
                    if blended_length > 0.001 {
                        steer_dir[0] /= blended_length;
                        steer_dir[1] /= blended_length;
                        steer_dir[2] /= blended_length;
                    }
                }
            }

            // Set desired velocity based on steering direction and speed
            if let Some(agent) = self.agents[agent_idx].as_mut() {
                agent.desired_vel[0] = steer_dir[0] * max_speed;
                agent.desired_vel[1] = steer_dir[1] * max_speed;
                agent.desired_vel[2] = steer_dir[2] * max_speed;
            }
        }

        Ok(())
    }

    /// Calculates velocity for all active agents
    fn calculate_velocity(&mut self, delta_time: f32) -> Result<()> {
        // Do collision avoidance between agents
        self.perform_collision_avoidance(delta_time)?;

        // Update each agent's velocity
        for &agent_idx in &self.active_agents {
            let agent = match self.agents[agent_idx].as_mut() {
                Some(agent) => agent,
                None => continue,
            };

            if agent.state != AgentState::Active {
                continue;
            }

            // Calculate acceleration
            let mut accel = [0.0; 3];

            // Acceleration from desired velocity
            accel[0] = (agent.desired_vel[0] - agent.vel[0]) * agent.params.max_acceleration;
            accel[1] = (agent.desired_vel[1] - agent.vel[1]) * agent.params.max_acceleration;
            accel[2] = (agent.desired_vel[2] - agent.vel[2]) * agent.params.max_acceleration;

            // Limit acceleration
            let accel_sq = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
            if accel_sq > agent.params.max_acceleration * agent.params.max_acceleration {
                let accel_mag = accel_sq.sqrt();
                accel[0] = accel[0] * agent.params.max_acceleration / accel_mag;
                accel[1] = accel[1] * agent.params.max_acceleration / accel_mag;
                accel[2] = accel[2] * agent.params.max_acceleration / accel_mag;
            }

            // Update agent's acceleration
            agent.accel = accel;
        }

        Ok(())
    }

    /// Moves all active agents
    fn move_agents(&mut self, delta_time: f32) -> Result<()> {
        for &agent_idx in &self.active_agents {
            let agent = match self.agents[agent_idx].as_mut() {
                Some(agent) => agent,
                None => continue,
            };

            if agent.state != AgentState::Active {
                continue;
            }

            // Update velocity
            agent.vel[0] += agent.accel[0] * delta_time;
            agent.vel[1] += agent.accel[1] * delta_time;
            agent.vel[2] += agent.accel[2] * delta_time;

            // Limit velocity
            let vel_sq = agent.vel[0] * agent.vel[0]
                + agent.vel[1] * agent.vel[1]
                + agent.vel[2] * agent.vel[2];
            if vel_sq > agent.params.max_speed * agent.params.max_speed {
                let vel_mag = vel_sq.sqrt();
                agent.vel[0] = agent.vel[0] * agent.params.max_speed / vel_mag;
                agent.vel[1] = agent.vel[1] * agent.params.max_speed / vel_mag;
                agent.vel[2] = agent.vel[2] * agent.params.max_speed / vel_mag;
            }

            // Calculate new position
            let new_pos = [
                agent.pos[0] + agent.vel[0] * delta_time,
                agent.pos[1] + agent.vel[1] * delta_time,
                agent.pos[2] + agent.vel[2] * delta_time,
            ];

            // Update agent position temporarily
            agent.pos = new_pos;
        }

        // Update path corridors in a separate pass to avoid borrow issues
        let active_agents = self.active_agents.clone();
        for &agent_idx in &active_agents {
            if let Some(agent) = self.agents[agent_idx].as_ref() {
                if agent.state == AgentState::Active
                    && agent.params.update_flags.contains(UpdateFlags::SEPARATION)
                {
                    let filter_idx = agent.params.query_filter_type as usize;
                    let filter = self.get_filter(filter_idx)?.clone();
                    let new_pos = agent.pos;

                    if let Some(agent) = self.agents[agent_idx].as_mut() {
                        agent
                            .corridor
                            .move_position(new_pos, &mut self.query, &filter)?;
                        agent.pos = agent.corridor.get_pos();
                    }
                }
            }
        }

        // Update all agent positions in the proximity grid after movement
        self.update_proximity_grid()?;

        Ok(())
    }

    /// Performs collision avoidance between agents
    fn perform_collision_avoidance(&mut self, delta_time: f32) -> Result<()> {
        // Check if we should use RVO for collision avoidance
        let use_rvo = self.rvo_simulator.is_some() && self.has_rvo_agents();

        if use_rvo {
            self.perform_rvo_collision_avoidance(delta_time)?;
        } else {
            self.perform_basic_collision_avoidance()?;
        }

        Ok(())
    }

    /// Checks if any agents are using RVO
    fn has_rvo_agents(&self) -> bool {
        self.active_agents.iter().any(|&agent_idx| {
            if let Some(agent) = &self.agents[agent_idx] {
                agent.params.use_rvo
            } else {
                false
            }
        })
    }

    /// Performs RVO-based collision avoidance
    fn perform_rvo_collision_avoidance(&mut self, delta_time: f32) -> Result<()> {
        // Synchronize crowd agents with RVO agents
        for &agent_idx in &self.active_agents {
            if let Some(agent) = &mut self.agents[agent_idx] {
                if agent.state != AgentState::Active || !agent.params.use_rvo {
                    continue;
                }

                if let Some(rvo_agent_id) = agent.rvo_agent_id {
                    if let Some(rvo_sim) = &mut self.rvo_simulator {
                        if let Some(rvo_agent) = rvo_sim.get_agent_mut(rvo_agent_id) {
                            // Update RVO agent position and preferred velocity
                            rvo_agent.position = position_3d_to_2d(&agent.pos);
                            rvo_agent.velocity = velocity_3d_to_2d(&agent.vel);
                            rvo_agent.pref_velocity = velocity_3d_to_2d(&agent.desired_vel);
                        }
                    }
                }
            }
        }

        // Run RVO simulation step
        if let Some(rvo_sim) = &mut self.rvo_simulator {
            rvo_sim.step(delta_time)?;
        }

        // Apply RVO results back to crowd agents
        for &agent_idx in &self.active_agents {
            if let Some(agent) = &mut self.agents[agent_idx] {
                if agent.state != AgentState::Active || !agent.params.use_rvo {
                    continue;
                }

                if let Some(rvo_agent_id) = agent.rvo_agent_id {
                    if let Some(rvo_sim) = &self.rvo_simulator {
                        if let Some(rvo_agent) = rvo_sim.get_agent(rvo_agent_id) {
                            // Apply RVO velocity to desired velocity
                            let new_vel_3d = velocity_2d_to_3d(&rvo_agent.get_new_velocity());
                            agent.desired_vel = new_vel_3d;
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Performs basic collision avoidance using proximity grid for efficiency
    fn perform_basic_collision_avoidance(&mut self) -> Result<()> {
        // Skip if there are too few agents
        if self.active_agents.len() <= 1 {
            return Ok(());
        }

        // For each active agent, avoid collisions with nearby agents
        for i in 0..self.active_agents.len() {
            let agent_idx_i = self.active_agents[i];

            let agent_i = match self.agents[agent_idx_i].as_ref() {
                Some(agent) => agent,
                None => continue,
            };

            if agent_i.state != AgentState::Active {
                continue;
            }

            // Skip if agent doesn't need separation
            if !agent_i.params.separate
                || !agent_i
                    .params
                    .update_flags
                    .contains(UpdateFlags::SEPARATION)
            {
                continue;
            }

            // Use proximity grid to find nearby agents efficiently
            let query_radius = agent_i.params.collision_query_range;
            let nearby_agents = self.proximity_grid.query_agents(agent_i.pos, query_radius);

            // Calculate a separation vector from nearby agents
            let mut sep = [0.0, 0.0, 0.0];
            let mut weight = 0.0;

            for grid_agent in nearby_agents {
                // Skip self
                if grid_agent.id == agent_idx_i {
                    continue;
                }

                // Get the actual agent data
                let agent_j = match self.agents.get(grid_agent.id).and_then(|a| a.as_ref()) {
                    Some(agent) => agent,
                    None => continue,
                };

                if agent_j.state != AgentState::Active {
                    continue;
                }

                // Calculate distance between agents
                let dist_x = agent_i.pos[0] - agent_j.pos[0];
                let dist_y = agent_i.pos[1] - agent_j.pos[1];
                let dist_z = agent_i.pos[2] - agent_j.pos[2];

                let dist_sq = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;

                // Skip if too far away
                let min_dist = agent_i.params.radius + agent_j.params.radius;
                if dist_sq > min_dist * min_dist * 4.0 {
                    continue;
                }

                // Calculate separation vector
                let dist = dist_sq.sqrt();
                let weight_j = if dist < 0.0001 { 1.0 } else { 1.0 / dist };

                sep[0] += dist_x * weight_j;
                sep[1] += dist_y * weight_j;
                sep[2] += dist_z * weight_j;
                weight += weight_j;
            }

            // Apply separation to desired velocity
            if weight > 0.0001 {
                // Normalize separation vector
                sep[0] /= weight;
                sep[1] /= weight;
                sep[2] /= weight;

                // Get agent (mutable) again
                let agent_i = match self.agents[agent_idx_i].as_mut() {
                    Some(agent) => agent,
                    None => continue,
                };

                // Blend with desired velocity
                let blend = 0.5;
                agent_i.desired_vel[0] = agent_i.desired_vel[0] * (1.0 - blend) + sep[0] * blend;
                agent_i.desired_vel[1] = agent_i.desired_vel[1] * (1.0 - blend) + sep[1] * blend;
                agent_i.desired_vel[2] = agent_i.desired_vel[2] * (1.0 - blend) + sep[2] * blend;

                // Normalize desired velocity
                let desired_vel_sq = agent_i.desired_vel[0] * agent_i.desired_vel[0]
                    + agent_i.desired_vel[1] * agent_i.desired_vel[1]
                    + agent_i.desired_vel[2] * agent_i.desired_vel[2];

                if desired_vel_sq > 0.0001 {
                    let desired_vel_mag = desired_vel_sq.sqrt();
                    agent_i.desired_vel[0] =
                        agent_i.desired_vel[0] * agent_i.params.max_speed / desired_vel_mag;
                    agent_i.desired_vel[1] =
                        agent_i.desired_vel[1] * agent_i.params.max_speed / desired_vel_mag;
                    agent_i.desired_vel[2] =
                        agent_i.desired_vel[2] * agent_i.params.max_speed / desired_vel_mag;
                }
            }
        }

        Ok(())
    }

    /// Purges inactive agents from the active list
    fn purge_inactive_agents(&mut self) {
        let mut i = 0;
        while i < self.active_agents.len() {
            let agent_idx = self.active_agents[i];

            let active = match self.agents[agent_idx].as_ref() {
                Some(agent) => agent.active,
                None => false,
            };

            if !active {
                self.active_agents.swap_remove(i);
            } else {
                i += 1;
            }
        }
    }

    /// Gets a query filter by index
    fn get_filter(&self, idx: usize) -> Result<&QueryFilter> {
        if idx >= self.filters.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        Ok(&self.filters[idx])
    }

    /// Gets an agent by index
    pub fn get_agent(&self, idx: usize) -> Option<&CrowdAgent> {
        if idx >= self.agents.len() {
            return None;
        }

        match &self.agents[idx] {
            Some(agent) => Some(agent),
            None => None,
        }
    }

    /// Gets an agent by index (mutable)
    pub fn get_agent_mut(&mut self, idx: usize) -> Option<&mut CrowdAgent> {
        if idx >= self.agents.len() {
            return None;
        }

        match &mut self.agents[idx] {
            Some(agent) => Some(agent),
            None => None,
        }
    }

    /// Gets the number of active agents
    pub fn get_active_agent_count(&self) -> usize {
        self.active_agents.len()
    }

    /// Adds a query filter
    pub fn add_query_filter(&mut self, filter: QueryFilter) -> usize {
        self.filters.push(filter);
        self.filters.len() - 1
    }

    /// Gets a query filter by index (mutable)
    pub fn get_filter_mut(&mut self, idx: usize) -> Result<&mut QueryFilter> {
        if idx >= self.filters.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        Ok(&mut self.filters[idx])
    }

    /// Enables RVO collision avoidance for the crowd
    pub fn enable_rvo(&mut self, default_config: Option<RVOConfig>) {
        let config = default_config.unwrap_or_default();
        self.rvo_simulator = Some(RVOSimulator::new_with_config(config));
    }

    /// Disables RVO collision avoidance for the crowd
    pub fn disable_rvo(&mut self) {
        // Remove RVO agent IDs from all agents
        for agent in self.agents.iter_mut().flatten() {
            agent.rvo_agent_id = None;
        }
        self.rvo_simulator = None;
    }

    /// Gets a reference to the RVO simulator (if enabled)
    pub fn get_rvo_simulator(&self) -> Option<&RVOSimulator> {
        self.rvo_simulator.as_ref()
    }

    /// Gets a mutable reference to the RVO simulator (if enabled)
    pub fn get_rvo_simulator_mut(&mut self) -> Option<&mut RVOSimulator> {
        self.rvo_simulator.as_mut()
    }

    /// Checks if RVO is enabled for this crowd
    pub fn is_rvo_enabled(&self) -> bool {
        self.rvo_simulator.is_some()
    }

    // Formation Management Methods

    /// Creates a new formation with the specified configuration
    pub fn create_formation(&mut self, config: FormationConfig) -> usize {
        self.formation_manager.create_formation(config)
    }

    /// Adds an agent to a formation with the specified role
    pub fn add_agent_to_formation(
        &mut self,
        formation_id: usize,
        agent_id: usize,
        role: FormationRole,
    ) -> Result<()> {
        if agent_id >= self.max_agents || self.agents[agent_id].is_none() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }
        self.formation_manager
            .add_agent_to_formation(formation_id, agent_id, role)
    }

    /// Removes an agent from their current formation
    pub fn remove_agent_from_formation(&mut self, agent_id: usize) -> Result<()> {
        self.formation_manager.remove_agent_from_formation(agent_id)
    }

    /// Gets the formation ID for an agent (if any)
    pub fn get_agent_formation(&self, agent_id: usize) -> Option<usize> {
        self.formation_manager.get_agent_formation(agent_id)
    }

    /// Sets a target destination for a formation
    pub fn set_formation_target(&mut self, formation_id: usize, target: [f32; 3]) -> Result<()> {
        self.formation_manager
            .set_formation_target(formation_id, target)
    }

    /// Gets the formation manager for direct access
    pub fn get_formation_manager(&self) -> &FormationManager {
        &self.formation_manager
    }

    /// Gets the formation manager for direct access (mutable)
    pub fn get_formation_manager_mut(&mut self) -> &mut FormationManager {
        &mut self.formation_manager
    }

    /// Dissolves a formation, removing all agents from it
    pub fn dissolve_formation(&mut self, formation_id: usize) -> Result<()> {
        self.formation_manager.dissolve_formation(formation_id)
    }

    /// Gets the number of active formations
    pub fn get_formation_count(&self) -> usize {
        self.formation_manager.get_formation_count()
    }

    /// Updates formations and applies formation steering
    fn update_formations(&mut self, delta_time: f32) -> Result<()> {
        // Collect current agent positions
        let mut agent_positions = std::collections::HashMap::new();
        for (idx, agent_opt) in self.agents.iter().enumerate() {
            if let Some(agent) = agent_opt {
                if agent.active {
                    agent_positions.insert(idx, agent.pos);
                }
            }
        }

        // Update the formation manager
        self.formation_manager.update(&agent_positions, delta_time);

        Ok(())
    }

    /// Updates all agent positions in the proximity grid
    fn update_proximity_grid(&mut self) -> Result<()> {
        for &agent_idx in &self.active_agents {
            if let Some(agent) = &self.agents[agent_idx] {
                if agent.state == AgentState::Active {
                    let grid_agent = GridAgent {
                        id: agent_idx,
                        pos: agent.pos,
                        radius: agent.params.radius,
                    };
                    let _ = self.proximity_grid.update_agent(grid_agent);
                }
            }
        }
        Ok(())
    }

    /// Gets a reference to the proximity grid for inspection
    pub fn get_proximity_grid(&self) -> &ProximityGrid {
        &self.proximity_grid
    }

    /// Gets proximity grid statistics for performance monitoring
    pub fn get_proximity_grid_stats(&self) -> super::proximity_grid::ProximityGridStats {
        self.proximity_grid.get_stats()
    }

    /// Queries for agents near a position using the proximity grid
    pub fn query_nearby_agents(&self, pos: [f32; 3], radius: f32) -> Vec<usize> {
        self.proximity_grid
            .query_agents(pos, radius)
            .into_iter()
            .map(|agent| agent.id)
            .collect()
    }

    /// Optimizes the proximity grid by removing empty cells
    pub fn optimize_proximity_grid(&mut self) {
        self.proximity_grid.optimize();
    }
}

/// Calculates the squared distance between two points
#[allow(dead_code)]
fn distance_sq(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];

    dx * dx + dy * dy + dz * dz
}

#[cfg(test)]
mod tests {
    use super::*;
    use detour::NavMeshParams;

    #[test]
    fn test_create_crowd() {
        // Create a simple navigation mesh
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params).unwrap();

        // Create a crowd
        let max_agents = 100;
        let max_agent_radius = 1.0;

        let crowd = Crowd::new(&nav_mesh, max_agents, max_agent_radius);

        assert_eq!(crowd.get_active_agent_count(), 0);
    }

    // More tests would be required for the full functionality, but would need a complex
    // navigation mesh setup to test properly.
}
