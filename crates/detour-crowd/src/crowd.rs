//! Crowd management for Detour
//!
//! This module contains the crowd management functionality, which
//! allows for efficient pathfinding and movement of multiple agents.

use std::f32;

use glam::Vec3;

use super::PathCorridor;
use super::formation::{FormationConfig, FormationManager, FormationRole};
use super::proximity_grid::{GridAgent, ProximityGrid};
use super::rvo::{
    RVOConfig, RVOSimulator, position_3d_to_2d, velocity_2d_to_3d, velocity_3d_to_2d,
};
use crate::error::CrowdError;
use detour::{NavMesh, NavMeshQuery, PolyRef, QueryFilter};

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

/// Agent parameters for crowd
#[derive(Debug, Clone)]
#[non_exhaustive]
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

impl AgentParams {
    pub fn with_radius(mut self, radius: f32) -> Self {
        self.radius = radius;
        self
    }

    pub fn with_height(mut self, height: f32) -> Self {
        self.height = height;
        self
    }

    pub fn with_max_acceleration(mut self, accel: f32) -> Self {
        self.max_acceleration = accel;
        self
    }

    pub fn with_max_speed(mut self, speed: f32) -> Self {
        self.max_speed = speed;
        self
    }

    pub fn with_collision_query_range(mut self, range: f32) -> Self {
        self.collision_query_range = range;
        self
    }

    pub fn with_path_optimization_range(mut self, range: f32) -> Self {
        self.path_optimization_range = range;
        self
    }

    pub fn with_separate(mut self, separate: bool) -> Self {
        self.separate = separate;
        self
    }

    pub fn with_update_flags(mut self, flags: UpdateFlags) -> Self {
        self.update_flags = flags;
        self
    }

    /// Quality type ranges from 0 (low) to 3 (high).
    pub fn with_obstacle_avoidance_type(mut self, avoidance_type: i32) -> Self {
        self.obstacle_avoidance_type = avoidance_type;
        self
    }

    pub fn with_query_filter_type(mut self, filter_type: i32) -> Self {
        self.query_filter_type = filter_type;
        self
    }

    pub fn with_user_data(mut self, data: Option<usize>) -> Self {
        self.user_data = data;
        self
    }

    pub fn with_use_rvo(mut self, use_rvo: bool) -> Self {
        self.use_rvo = use_rvo;
        self
    }

    pub fn with_rvo_config(mut self, config: RVOConfig) -> Self {
        self.rvo_config = config;
        self
    }
}

/// Agent in the crowd (legacy)
#[derive(Debug, Clone)]
pub struct CrowdAgent {
    params: AgentParams,
    state: AgentState,
    pos: [f32; 3],
    desired_vel: [f32; 3],
    vel: [f32; 3],
    accel: [f32; 3],
    target: [f32; 3],
    target_ref: PolyRef,
    corridor: PathCorridor,
    target_updated: bool,
    active: bool,
    id: usize,
    rvo_agent_id: Option<usize>,
}

impl CrowdAgent {
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

    pub fn get_pos(&self) -> Vec3 {
        Vec3::from(self.pos)
    }

    pub fn get_vel(&self) -> Vec3 {
        Vec3::from(self.vel)
    }

    pub fn get_target(&self) -> Vec3 {
        Vec3::from(self.target)
    }

    pub fn get_state(&self) -> AgentState {
        self.state
    }

    pub fn get_params(&self) -> &AgentParams {
        &self.params
    }

    pub fn get_id(&self) -> usize {
        self.id
    }

    pub fn is_active(&self) -> bool {
        self.active
    }
}

/// Legacy crowd manager for Detour
#[derive(Debug)]
pub struct Crowd<'a> {
    // TODO: Currently stored but not used directly - access through query instead
    #[allow(dead_code)]
    nav_mesh: &'a NavMesh,
    query: NavMeshQuery<'a>,
    max_agents: usize,
    agents: Vec<Option<CrowdAgent>>,
    active_agents: Vec<usize>,
    filters: Vec<QueryFilter>,
    delta_time: f32,
    max_agent_radius: f32,
    rvo_simulator: Option<RVOSimulator>,
    formation_manager: FormationManager,
    proximity_grid: ProximityGrid,
}

impl<'a> Crowd<'a> {
    pub fn new(nav_mesh: &'a NavMesh, max_agents: usize, max_agent_radius: f32) -> Self {
        let query = NavMeshQuery::new(nav_mesh);

        let mut agents = Vec::with_capacity(max_agents);
        for _i in 0..max_agents {
            agents.push(None);
        }

        let default_filter = QueryFilter::default();

        // 4x max agent radius gives good spatial distribution for the proximity grid
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

    pub fn add_agent(&mut self, pos: Vec3, params: AgentParams) -> Result<usize, CrowdError> {
        let mut slot = usize::MAX;
        for i in 0..self.max_agents {
            if self.agents[i].is_none() {
                slot = i;
                break;
            }
        }

        if slot == usize::MAX {
            return Err(CrowdError::InvalidParam);
        }

        let ext = Vec3::new(
            params.radius * 2.0,
            params.height * 0.5,
            params.radius * 2.0,
        );

        let filter = &self.filters[0]; // Use default filter
        let (nearest_ref, nearest_pos) = match self.query.find_nearest_poly(pos, ext, filter) {
            Ok((reference, position)) => (reference, position),
            Err(_) => {
                // Try with larger extents
                let ext = Vec3::new(params.radius * 3.0, params.height, params.radius * 3.0);
                match self.query.find_nearest_poly(pos, ext, filter) {
                    Ok((reference, position)) => (reference, position),
                    Err(_) => return Err(CrowdError::InvalidParam),
                }
            }
        };

        self.max_agent_radius = self.max_agent_radius.max(params.radius);

        let mut agent = CrowdAgent::new(params.clone(), slot);
        agent.state = AgentState::Active;
        agent.pos = nearest_pos.to_array();
        agent.desired_vel = [0.0, 0.0, 0.0];
        agent.vel = [0.0, 0.0, 0.0];
        agent.active = true;

        agent.corridor.reset(nearest_ref, nearest_pos);

        if params.use_rvo {
            if self.rvo_simulator.is_none() {
                self.rvo_simulator = Some(RVOSimulator::new());
            }

            if let Some(rvo_sim) = &mut self.rvo_simulator {
                let rvo_pos_2d = position_3d_to_2d(nearest_pos);
                let rvo_agent_id = rvo_sim.add_agent_with_config(rvo_pos_2d, params.rvo_config);
                agent.rvo_agent_id = Some(rvo_agent_id);
            }
        }

        let grid_agent = GridAgent {
            id: slot,
            pos: nearest_pos.to_array(),
            radius: params.radius,
        };
        let _ = self.proximity_grid.update_agent(grid_agent);

        self.agents[slot] = Some(agent);
        self.active_agents.push(slot);

        Ok(slot)
    }

    pub fn remove_agent(&mut self, agent_idx: usize) -> Result<(), CrowdError> {
        if agent_idx >= self.max_agents {
            return Err(CrowdError::AgentNotFound { index: agent_idx });
        }

        if self.agents[agent_idx].is_none() {
            return Err(CrowdError::AgentNotFound { index: agent_idx });
        }

        if let Some(agent) = &self.agents[agent_idx] {
            if let Some(rvo_agent_id) = agent.rvo_agent_id {
                if let Some(rvo_sim) = &mut self.rvo_simulator {
                    let _ = rvo_sim.remove_agent(rvo_agent_id);
                }
            }
        }

        self.proximity_grid.remove_agent(agent_idx);

        if let Some(pos) = self.active_agents.iter().position(|&idx| idx == agent_idx) {
            self.active_agents.swap_remove(pos);
        }

        self.agents[agent_idx] = None;

        if self.max_agent_radius > DEFAULT_AGENT_RADIUS {
            self.max_agent_radius = DEFAULT_AGENT_RADIUS;

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
        target_pos: Vec3,
    ) -> Result<(), CrowdError> {
        let target_pos = target_pos.to_array();
        if agent_idx >= self.max_agents {
            return Err(CrowdError::AgentNotFound { index: agent_idx });
        }

        let agent = match self.agents[agent_idx].as_mut() {
            Some(agent) => agent,
            None => {
                return Err(CrowdError::AgentNotFound { index: agent_idx });
            }
        };

        agent.target = target_pos;
        agent.target_ref = target_ref;
        agent.target_updated = true;

        if !agent.active {
            agent.active = true;
            self.active_agents.push(agent_idx);
        }

        Ok(())
    }

    /// Updates all agents in the crowd
    pub fn update(&mut self, delta_time: f32) -> Result<(), CrowdError> {
        self.delta_time = delta_time;

        // TODO: Implement dynamic avoidance grid

        for &agent_idx in &self.active_agents.clone() {
            if let Some(agent) = self.agents[agent_idx].as_ref() {
                if agent.target_updated {
                    let filter_idx = agent.params.query_filter_type as usize;
                    let filter = self.get_filter(filter_idx)?.clone();

                    if agent.corridor.get_path().is_empty() {
                        continue;
                    }

                    let target_ref = agent.target_ref;
                    let target_pos = agent.target;

                    if let Some(agent) = self.agents[agent_idx].as_mut() {
                        agent.target_updated = false;

                        match agent.corridor.find_path(
                            &mut self.query,
                            target_ref,
                            Vec3::from(target_pos),
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

            if let Some(agent) = self.agents[agent_idx].as_ref() {
                if agent.state == AgentState::Active && !agent.corridor.get_path().is_empty() {
                    let filter_idx = agent.params.query_filter_type as usize;
                    let filter = self.get_filter(filter_idx)?.clone();
                    let agent_pos = agent.pos;

                    // Separate scope to avoid borrow issues with self.agents and self.query
                    if let Some(agent) = self.agents[agent_idx].as_mut() {
                        if let Err(e) = agent.corridor.optimize_path(&mut self.query, &filter) {
                            // If optimization fails, log but continue - it's not critical
                            eprintln!(
                                "DEBUG: Corridor optimization failed for agent {}: {:?}",
                                agent_idx, e
                            );
                        }

                        match agent.corridor.advance(
                            Vec3::from(agent_pos),
                            &mut self.query,
                            &filter,
                        ) {
                            Ok(reached_target) => {
                                if reached_target {
                                    agent.state = AgentState::Completed;
                                    agent.target_ref = PolyRef::new(0);
                                }
                            }
                            Err(e) => {
                                eprintln!(
                                    "DEBUG: Corridor advance failed for agent {}: {:?}",
                                    agent_idx, e
                                );

                                if let Err(e2) = agent.corridor.move_position(
                                    Vec3::from(agent_pos),
                                    &mut self.query,
                                    &filter,
                                ) {
                                    eprintln!(
                                        "DEBUG: Corridor move_position fallback also failed for agent {}: {:?}",
                                        agent_idx, e2
                                    );
                                    agent.state = AgentState::Failed;
                                }
                            }
                        }
                    }
                }
            }
        }

        self.update_formations(delta_time)?;

        if let Err(e) = self.calculate_steering() {
            eprintln!("DEBUG: calculate_steering failed: {:?}", e);
            return Err(e);
        }

        if let Err(e) = self.calculate_velocity(delta_time) {
            eprintln!("DEBUG: calculate_velocity failed: {:?}", e);
            return Err(e);
        }

        if let Err(e) = self.move_agents(delta_time) {
            eprintln!("DEBUG: move_agents failed: {:?}", e);
            return Err(e);
        }

        self.purge_inactive_agents();

        Ok(())
    }

    /// Calculates steering for all active agents
    fn calculate_steering(&mut self) -> Result<(), CrowdError> {
        // Clone needed to avoid borrowing self.active_agents while mutating self.agents
        let active_agents = self.active_agents.clone();

        for &agent_idx in &active_agents {
            let Some(agent) = self.agents[agent_idx].as_ref() else {
                continue;
            };
            if agent.state != AgentState::Active {
                continue;
            }

            let path = agent.corridor.get_path().to_vec();
            let agent_pos = agent.pos;
            let max_speed = agent.params.max_speed;
            let target_pos = agent.corridor.get_target();

            let mut steer_dir = [0.0; 3];

            if !path.is_empty() {
                steer_dir[0] = target_pos[0] - agent_pos[0];
                steer_dir[1] = target_pos[1] - agent_pos[1];
                steer_dir[2] = target_pos[2] - agent_pos[2];

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

            let formation_positions = self.formation_manager.get_formation_positions();
            if let Some(formation_pos) = formation_positions.get(&agent_idx) {
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
                    let formation_steer_normalized = [
                        formation_steer[0] / formation_dist,
                        formation_steer[1] / formation_dist,
                        formation_steer[2] / formation_dist,
                    ];

                    let formation_weight = 0.6;
                    let path_weight = 1.0 - formation_weight;

                    steer_dir[0] = path_weight * steer_dir[0]
                        + formation_weight * formation_steer_normalized[0];
                    steer_dir[1] = path_weight * steer_dir[1]
                        + formation_weight * formation_steer_normalized[1];
                    steer_dir[2] = path_weight * steer_dir[2]
                        + formation_weight * formation_steer_normalized[2];

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

            if let Some(agent) = self.agents[agent_idx].as_mut() {
                agent.desired_vel[0] = steer_dir[0] * max_speed;
                agent.desired_vel[1] = steer_dir[1] * max_speed;
                agent.desired_vel[2] = steer_dir[2] * max_speed;
            }
        }

        Ok(())
    }

    /// Calculates velocity for all active agents
    fn calculate_velocity(&mut self, delta_time: f32) -> Result<(), CrowdError> {
        self.perform_collision_avoidance(delta_time)?;

        for &agent_idx in &self.active_agents {
            let agent = match self.agents[agent_idx].as_mut() {
                Some(agent) => agent,
                None => continue,
            };

            if agent.state != AgentState::Active {
                continue;
            }

            let mut accel = [0.0; 3];
            accel[0] = (agent.desired_vel[0] - agent.vel[0]) * agent.params.max_acceleration;
            accel[1] = (agent.desired_vel[1] - agent.vel[1]) * agent.params.max_acceleration;
            accel[2] = (agent.desired_vel[2] - agent.vel[2]) * agent.params.max_acceleration;

            let accel_sq = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
            if accel_sq > agent.params.max_acceleration * agent.params.max_acceleration {
                let accel_mag = accel_sq.sqrt();
                accel[0] = accel[0] * agent.params.max_acceleration / accel_mag;
                accel[1] = accel[1] * agent.params.max_acceleration / accel_mag;
                accel[2] = accel[2] * agent.params.max_acceleration / accel_mag;
            }

            agent.accel = accel;
        }

        Ok(())
    }

    /// Moves all active agents
    fn move_agents(&mut self, delta_time: f32) -> Result<(), CrowdError> {
        for &agent_idx in &self.active_agents {
            let agent = match self.agents[agent_idx].as_mut() {
                Some(agent) => agent,
                None => continue,
            };

            if agent.state != AgentState::Active {
                continue;
            }

            agent.vel[0] += agent.accel[0] * delta_time;
            agent.vel[1] += agent.accel[1] * delta_time;
            agent.vel[2] += agent.accel[2] * delta_time;

            let vel_sq = agent.vel[0] * agent.vel[0]
                + agent.vel[1] * agent.vel[1]
                + agent.vel[2] * agent.vel[2];
            if vel_sq > agent.params.max_speed * agent.params.max_speed {
                let vel_mag = vel_sq.sqrt();
                agent.vel[0] = agent.vel[0] * agent.params.max_speed / vel_mag;
                agent.vel[1] = agent.vel[1] * agent.params.max_speed / vel_mag;
                agent.vel[2] = agent.vel[2] * agent.params.max_speed / vel_mag;
            }

            let new_pos = [
                agent.pos[0] + agent.vel[0] * delta_time,
                agent.pos[1] + agent.vel[1] * delta_time,
                agent.pos[2] + agent.vel[2] * delta_time,
            ];

            agent.pos = new_pos;
        }

        // Separate pass: self.agents and self.query can't be borrowed together above
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
                        agent.corridor.move_position(
                            Vec3::from(new_pos),
                            &mut self.query,
                            &filter,
                        )?;
                        agent.pos = agent.corridor.get_pos().to_array();
                    }
                }
            }
        }

        self.update_proximity_grid()?;

        Ok(())
    }

    /// Performs collision avoidance between agents
    fn perform_collision_avoidance(&mut self, delta_time: f32) -> Result<(), CrowdError> {
        let use_rvo = self.rvo_simulator.is_some() && self.has_rvo_agents();

        if use_rvo {
            self.perform_rvo_collision_avoidance(delta_time)?;
        } else {
            self.perform_basic_collision_avoidance()?;
        }

        Ok(())
    }

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
    fn perform_rvo_collision_avoidance(&mut self, delta_time: f32) -> Result<(), CrowdError> {
        for &agent_idx in &self.active_agents {
            if let Some(agent) = &mut self.agents[agent_idx] {
                if agent.state != AgentState::Active || !agent.params.use_rvo {
                    continue;
                }

                if let Some(rvo_agent_id) = agent.rvo_agent_id {
                    if let Some(rvo_sim) = &mut self.rvo_simulator {
                        if let Some(rvo_agent) = rvo_sim.get_agent_mut(rvo_agent_id) {
                            rvo_agent.position = position_3d_to_2d(Vec3::from(agent.pos));
                            rvo_agent.velocity = velocity_3d_to_2d(Vec3::from(agent.vel));
                            rvo_agent.pref_velocity =
                                velocity_3d_to_2d(Vec3::from(agent.desired_vel));
                        }
                    }
                }
            }
        }

        if let Some(rvo_sim) = &mut self.rvo_simulator {
            rvo_sim.step(delta_time)?;
        }

        for &agent_idx in &self.active_agents {
            if let Some(agent) = &mut self.agents[agent_idx] {
                if agent.state != AgentState::Active || !agent.params.use_rvo {
                    continue;
                }

                if let Some(rvo_agent_id) = agent.rvo_agent_id {
                    if let Some(rvo_sim) = &self.rvo_simulator {
                        if let Some(rvo_agent) = rvo_sim.get_agent(rvo_agent_id) {
                            let new_vel_3d = velocity_2d_to_3d(&rvo_agent.get_new_velocity());
                            agent.desired_vel = new_vel_3d.to_array();
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Performs basic collision avoidance using proximity grid for efficiency
    fn perform_basic_collision_avoidance(&mut self) -> Result<(), CrowdError> {
        if self.active_agents.len() <= 1 {
            return Ok(());
        }

        for i in 0..self.active_agents.len() {
            let agent_idx_i = self.active_agents[i];

            let agent_i = match self.agents[agent_idx_i].as_ref() {
                Some(agent) => agent,
                None => continue,
            };

            if agent_i.state != AgentState::Active {
                continue;
            }

            if !agent_i.params.separate
                || !agent_i
                    .params
                    .update_flags
                    .contains(UpdateFlags::SEPARATION)
            {
                continue;
            }

            let query_radius = agent_i.params.collision_query_range;
            let nearby_agents = self
                .proximity_grid
                .query_agents(Vec3::from(agent_i.pos), query_radius);

            let mut sep = [0.0, 0.0, 0.0];
            let mut weight = 0.0;

            for grid_agent in nearby_agents {
                if grid_agent.id == agent_idx_i {
                    continue;
                }

                let agent_j = match self.agents.get(grid_agent.id).and_then(|a| a.as_ref()) {
                    Some(agent) => agent,
                    None => continue,
                };

                if agent_j.state != AgentState::Active {
                    continue;
                }

                let dist_x = agent_i.pos[0] - agent_j.pos[0];
                let dist_y = agent_i.pos[1] - agent_j.pos[1];
                let dist_z = agent_i.pos[2] - agent_j.pos[2];

                let dist_sq = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;

                let min_dist = agent_i.params.radius + agent_j.params.radius;
                if dist_sq > min_dist * min_dist * 4.0 {
                    continue;
                }

                let dist = dist_sq.sqrt();
                let weight_j = if dist < 0.0001 { 1.0 } else { 1.0 / dist };

                sep[0] += dist_x * weight_j;
                sep[1] += dist_y * weight_j;
                sep[2] += dist_z * weight_j;
                weight += weight_j;
            }

            if weight > 0.0001 {
                sep[0] /= weight;
                sep[1] /= weight;
                sep[2] /= weight;

                let agent_i = match self.agents[agent_idx_i].as_mut() {
                    Some(agent) => agent,
                    None => continue,
                };

                let blend = 0.5;
                agent_i.desired_vel[0] = agent_i.desired_vel[0] * (1.0 - blend) + sep[0] * blend;
                agent_i.desired_vel[1] = agent_i.desired_vel[1] * (1.0 - blend) + sep[1] * blend;
                agent_i.desired_vel[2] = agent_i.desired_vel[2] * (1.0 - blend) + sep[2] * blend;

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

    fn get_filter(&self, idx: usize) -> Result<&QueryFilter, CrowdError> {
        if idx >= self.filters.len() {
            return Err(CrowdError::InvalidParam);
        }

        Ok(&self.filters[idx])
    }

    pub fn get_agent(&self, idx: usize) -> Option<&CrowdAgent> {
        if idx >= self.agents.len() {
            return None;
        }

        match &self.agents[idx] {
            Some(agent) => Some(agent),
            None => None,
        }
    }

    pub fn get_agent_mut(&mut self, idx: usize) -> Option<&mut CrowdAgent> {
        if idx >= self.agents.len() {
            return None;
        }

        match &mut self.agents[idx] {
            Some(agent) => Some(agent),
            None => None,
        }
    }

    pub fn get_active_agent_count(&self) -> usize {
        self.active_agents.len()
    }

    pub fn add_query_filter(&mut self, filter: QueryFilter) -> usize {
        self.filters.push(filter);
        self.filters.len() - 1
    }

    pub fn get_filter_mut(&mut self, idx: usize) -> Result<&mut QueryFilter, CrowdError> {
        if idx >= self.filters.len() {
            return Err(CrowdError::InvalidParam);
        }

        Ok(&mut self.filters[idx])
    }

    pub fn enable_rvo(&mut self, default_config: Option<RVOConfig>) {
        let config = default_config.unwrap_or_default();
        self.rvo_simulator = Some(RVOSimulator::new_with_config(config));
    }

    pub fn disable_rvo(&mut self) {
        for agent in self.agents.iter_mut().flatten() {
            agent.rvo_agent_id = None;
        }
        self.rvo_simulator = None;
    }

    pub fn get_rvo_simulator(&self) -> Option<&RVOSimulator> {
        self.rvo_simulator.as_ref()
    }

    pub fn get_rvo_simulator_mut(&mut self) -> Option<&mut RVOSimulator> {
        self.rvo_simulator.as_mut()
    }

    pub fn is_rvo_enabled(&self) -> bool {
        self.rvo_simulator.is_some()
    }

    pub fn create_formation(&mut self, config: FormationConfig) -> usize {
        self.formation_manager.create_formation(config)
    }

    pub fn add_agent_to_formation(
        &mut self,
        formation_id: usize,
        agent_id: usize,
        role: FormationRole,
    ) -> Result<(), CrowdError> {
        if agent_id >= self.max_agents || self.agents[agent_id].is_none() {
            return Err(CrowdError::AgentNotFound { index: agent_id });
        }
        self.formation_manager
            .add_agent_to_formation(formation_id, agent_id, role)
    }

    pub fn remove_agent_from_formation(&mut self, agent_id: usize) -> Result<(), CrowdError> {
        self.formation_manager.remove_agent_from_formation(agent_id)
    }

    pub fn get_agent_formation(&self, agent_id: usize) -> Option<usize> {
        self.formation_manager.get_agent_formation(agent_id)
    }

    pub fn set_formation_target(
        &mut self,
        formation_id: usize,
        target: Vec3,
    ) -> Result<(), CrowdError> {
        self.formation_manager
            .set_formation_target(formation_id, target)
    }

    pub fn get_formation_manager(&self) -> &FormationManager {
        &self.formation_manager
    }

    pub fn get_formation_manager_mut(&mut self) -> &mut FormationManager {
        &mut self.formation_manager
    }

    pub fn dissolve_formation(&mut self, formation_id: usize) -> Result<(), CrowdError> {
        self.formation_manager.dissolve_formation(formation_id)
    }

    pub fn get_formation_count(&self) -> usize {
        self.formation_manager.get_formation_count()
    }

    /// Updates formations and applies formation steering
    fn update_formations(&mut self, delta_time: f32) -> Result<(), CrowdError> {
        let mut agent_positions = std::collections::HashMap::new();
        for (idx, agent_opt) in self.agents.iter().enumerate() {
            if let Some(agent) = agent_opt {
                if agent.active {
                    agent_positions.insert(idx, agent.pos);
                }
            }
        }

        self.formation_manager.update(&agent_positions, delta_time);

        Ok(())
    }

    fn update_proximity_grid(&mut self) -> Result<(), CrowdError> {
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

    pub fn get_proximity_grid(&self) -> &ProximityGrid {
        &self.proximity_grid
    }

    pub fn get_proximity_grid_stats(&self) -> super::proximity_grid::ProximityGridStats {
        self.proximity_grid.get_stats()
    }

    pub fn query_nearby_agents(&self, pos: Vec3, radius: f32) -> Vec<usize> {
        self.proximity_grid
            .query_agents(pos, radius)
            .into_iter()
            .map(|agent| agent.id)
            .collect()
    }

    pub fn optimize_proximity_grid(&mut self) {
        self.proximity_grid.optimize();
    }
}

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
        let params = {
            let mut p = NavMeshParams::default();
            p.tile_width = 100.0;
            p.tile_height = 100.0;
            p.max_tiles = 128;
            p.max_polys_per_tile = 1024;
            p
        };

        let nav_mesh = NavMesh::new(params).unwrap();

        let max_agents = 100;
        let max_agent_radius = 1.0;

        let crowd = Crowd::new(&nav_mesh, max_agents, max_agent_radius);

        assert_eq!(crowd.get_active_agent_count(), 0);
    }

    // More tests would be required for the full functionality, but would need a complex
    // navigation mesh setup to test properly.
}
