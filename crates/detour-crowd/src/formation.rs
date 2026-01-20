//! Crowd formation management for Detour
//!
//! This module provides functionality for managing groups of agents in specific
//! formations, including formation types, leadership roles, and dynamic formation
//! behaviors.

use std::collections::HashMap;
use std::f32::consts::PI;

use detour::Status;
use recast_common::{Error, Result};

/// Maximum number of agents in a single formation
const MAX_FORMATION_SIZE: usize = 64;

/// Formation types supported by the formation management system
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FormationType {
    /// Single file line formation
    Line,
    /// Two-column formation
    Column,
    /// Wedge/V formation
    Wedge,
    /// Circular formation
    Circle,
    /// Box/rectangle formation
    Box,
    /// Diamond formation
    Diamond,
    /// Custom formation (user-defined positions)
    Custom,
}

/// Formation state for tracking formation behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormationState {
    /// Formation is forming (agents moving to positions)
    Forming,
    /// Formation is stable and maintaining shape
    Stable,
    /// Formation is moving as a cohesive unit
    Moving,
    /// Formation is adapting to environmental constraints
    Adapting,
    /// Formation is breaking apart
    Disbanding,
}

/// Formation configuration parameters
#[derive(Debug, Clone)]
pub struct FormationConfig {
    /// Type of formation
    pub formation_type: FormationType,
    /// Spacing between agents in the formation
    pub agent_spacing: f32,
    /// How tightly agents maintain formation positions
    pub cohesion_strength: f32,
    /// How quickly agents adapt to formation changes
    pub adaptation_speed: f32,
    /// Maximum distance from formation center before rejoining
    pub max_distance_threshold: f32,
    /// Whether the formation should automatically adapt to obstacles
    pub auto_adapt: bool,
    /// Priority of formation maintenance vs individual navigation
    pub formation_priority: f32,
}

impl Default for FormationConfig {
    fn default() -> Self {
        Self {
            formation_type: FormationType::Line,
            agent_spacing: 2.0,
            cohesion_strength: 0.8,
            adaptation_speed: 1.0,
            max_distance_threshold: 10.0,
            auto_adapt: true,
            formation_priority: 0.7,
        }
    }
}

/// Role of an agent within a formation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormationRole {
    /// Leader sets the direction and pace of the formation
    Leader,
    /// Followers maintain relative positions to the leader
    Follower,
    /// Scouts move ahead to check for obstacles
    Scout,
    /// Guards protect the flanks or rear of the formation
    Guard,
}

/// Individual agent's formation data
#[derive(Debug, Clone)]
pub struct FormationAgent {
    /// Agent ID in the crowd
    pub agent_id: usize,
    /// Role within the formation
    pub role: FormationRole,
    /// Target position relative to formation center
    pub relative_position: [f32; 3],
    /// Current formation position (world coordinates)
    pub formation_position: [f32; 3],
    /// Weight of this agent's influence on formation movement
    pub influence_weight: f32,
    /// How long agent has been out of formation
    pub out_of_formation_time: f32,
}

impl FormationAgent {
    /// Creates a new formation agent
    pub fn new(agent_id: usize, role: FormationRole, relative_position: [f32; 3]) -> Self {
        Self {
            agent_id,
            role,
            relative_position,
            formation_position: [0.0, 0.0, 0.0],
            influence_weight: match role {
                FormationRole::Leader => 1.0,
                FormationRole::Follower => 0.5,
                FormationRole::Scout => 0.3,
                FormationRole::Guard => 0.7,
            },
            out_of_formation_time: 0.0,
        }
    }
}

/// A formation of agents that move together
#[derive(Debug)]
pub struct Formation {
    /// Unique identifier for this formation
    pub id: usize,
    /// Formation configuration
    pub config: FormationConfig,
    /// Current state of the formation
    pub state: FormationState,
    /// Agents in this formation
    pub agents: Vec<FormationAgent>,
    /// Center position of the formation
    pub center: [f32; 3],
    /// Current heading direction of the formation
    pub heading: [f32; 3],
    /// Target destination for the formation
    pub target: Option<[f32; 3]>,
    /// Movement speed of the formation
    pub speed: f32,
    /// Time the formation has existed
    pub age: f32,
}

impl Formation {
    /// Creates a new formation
    pub fn new(id: usize, config: FormationConfig) -> Self {
        Self {
            id,
            config,
            state: FormationState::Forming,
            agents: Vec::new(),
            center: [0.0, 0.0, 0.0],
            heading: [0.0, 0.0, 1.0], // Default forward direction
            target: None,
            speed: 0.0,
            age: 0.0,
        }
    }

    /// Adds an agent to the formation
    pub fn add_agent(&mut self, agent_id: usize, role: FormationRole) -> Result<()> {
        if self.agents.len() >= MAX_FORMATION_SIZE {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Calculate relative position based on formation type and agent count
        let relative_position = self.calculate_formation_position(self.agents.len(), role)?;

        let formation_agent = FormationAgent::new(agent_id, role, relative_position);
        self.agents.push(formation_agent);

        // If this is the first agent and it's a leader, use its position as center
        if self.agents.len() == 1 && role == FormationRole::Leader {
            // Center will be updated when we have actual agent position
        }

        Ok(())
    }

    /// Removes an agent from the formation
    pub fn remove_agent(&mut self, agent_id: usize) -> Result<()> {
        let initial_len = self.agents.len();
        self.agents.retain(|agent| agent.agent_id != agent_id);

        if self.agents.len() == initial_len {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // If formation is now empty, mark for disbanding
        if self.agents.is_empty() {
            self.state = FormationState::Disbanding;
        }

        Ok(())
    }

    /// Calculates the formation position for a new agent
    fn calculate_formation_position(&self, index: usize, _role: FormationRole) -> Result<[f32; 3]> {
        let spacing = self.config.agent_spacing;

        match self.config.formation_type {
            FormationType::Line => {
                // Single file line
                let x = 0.0;
                let z = -(index as f32) * spacing;
                Ok([x, 0.0, z])
            }
            FormationType::Column => {
                // Two-column formation
                let x = if index % 2 == 0 {
                    -spacing * 0.5
                } else {
                    spacing * 0.5
                };
                let z = -(index as f32 / 2.0).floor() * spacing;
                Ok([x, 0.0, z])
            }
            FormationType::Wedge => {
                // V-shaped wedge formation
                if index == 0 {
                    Ok([0.0, 0.0, 0.0]) // Leader at the point
                } else {
                    let side = if index % 2 == 1 { -1.0 } else { 1.0 };
                    let row = (index as f32 / 2.0).ceil();
                    let x = side * row * spacing * 0.7;
                    let z = -row * spacing;
                    Ok([x, 0.0, z])
                }
            }
            FormationType::Circle => {
                // Circular formation
                if index == 0 {
                    Ok([0.0, 0.0, 0.0]) // Leader at center
                } else if self.agents.len() <= 1 {
                    // Handle edge case where we only have one agent
                    Ok([spacing, 0.0, 0.0])
                } else {
                    let angle = (index - 1) as f32 * 2.0 * PI / (self.agents.len() - 1) as f32;
                    let radius = spacing * 2.0;
                    let x = radius * angle.cos();
                    let z = radius * angle.sin();
                    Ok([x, 0.0, z])
                }
            }
            FormationType::Box => {
                // Rectangular box formation
                let cols = (((self.agents.len() as f32).sqrt()).ceil() as usize).max(1);
                let row = index / cols;
                let col = index % cols;
                let x = (col as f32 - cols as f32 * 0.5) * spacing;
                let z = -(row as f32) * spacing;
                Ok([x, 0.0, z])
            }
            FormationType::Diamond => {
                // Diamond formation
                if index == 0 {
                    Ok([0.0, 0.0, 0.0]) // Leader at front
                } else if index <= 2 {
                    let side = if index == 1 { -1.0 } else { 1.0 };
                    Ok([side * spacing, 0.0, -spacing])
                } else {
                    Ok([0.0, 0.0, -spacing * 2.0]) // Rear guard
                }
            }
            FormationType::Custom => {
                // Custom formation positions would be set externally
                Ok([0.0, 0.0, -(index as f32) * spacing])
            }
        }
    }

    /// Updates the formation center based on agent positions
    pub fn update_center(&mut self, agent_positions: &HashMap<usize, [f32; 3]>) {
        if self.agents.is_empty() {
            return;
        }

        let mut center = [0.0, 0.0, 0.0];
        let mut total_weight = 0.0;

        for formation_agent in &self.agents {
            if let Some(pos) = agent_positions.get(&formation_agent.agent_id) {
                let weight = formation_agent.influence_weight;
                center[0] += pos[0] * weight;
                center[1] += pos[1] * weight;
                center[2] += pos[2] * weight;
                total_weight += weight;
            }
        }

        if total_weight > 0.0 {
            center[0] /= total_weight;
            center[1] /= total_weight;
            center[2] /= total_weight;
            self.center = center;
        }
    }

    /// Updates formation positions for all agents
    pub fn update_formation_positions(&mut self) {
        let rotation_matrix = self.calculate_rotation_matrix();

        for agent in &mut self.agents {
            // Transform relative position by formation heading
            agent.formation_position = [
                self.center[0]
                    + rotation_matrix[0][0] * agent.relative_position[0]
                    + rotation_matrix[0][2] * agent.relative_position[2],
                self.center[1] + agent.relative_position[1],
                self.center[2]
                    + rotation_matrix[2][0] * agent.relative_position[0]
                    + rotation_matrix[2][2] * agent.relative_position[2],
            ];
        }
    }

    /// Calculates rotation matrix for formation heading
    fn calculate_rotation_matrix(&self) -> [[f32; 3]; 3] {
        let heading = self.heading;
        let angle = heading[2].atan2(heading[0]);
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        [[cos_a, 0.0, -sin_a], [0.0, 1.0, 0.0], [sin_a, 0.0, cos_a]]
    }

    /// Sets the target destination for the formation
    pub fn set_target(&mut self, target: [f32; 3]) {
        self.target = Some(target);
        self.state = FormationState::Moving;
    }

    /// Updates the formation heading towards the target
    pub fn update_heading(&mut self, dt: f32) {
        if let Some(target) = self.target {
            let direction = [
                target[0] - self.center[0],
                target[1] - self.center[1],
                target[2] - self.center[2],
            ];

            let length = (direction[0] * direction[0]
                + direction[1] * direction[1]
                + direction[2] * direction[2])
                .sqrt();
            if length > 0.01 {
                // Smoothly update heading
                let new_heading = [
                    direction[0] / length,
                    direction[1] / length,
                    direction[2] / length,
                ];

                let adaptation_rate = self.config.adaptation_speed * dt;
                self.heading[0] =
                    self.heading[0] * (1.0 - adaptation_rate) + new_heading[0] * adaptation_rate;
                self.heading[1] =
                    self.heading[1] * (1.0 - adaptation_rate) + new_heading[1] * adaptation_rate;
                self.heading[2] =
                    self.heading[2] * (1.0 - adaptation_rate) + new_heading[2] * adaptation_rate;

                // Normalize heading
                let heading_length = (self.heading[0] * self.heading[0]
                    + self.heading[1] * self.heading[1]
                    + self.heading[2] * self.heading[2])
                    .sqrt();
                if heading_length > 0.01 {
                    self.heading[0] /= heading_length;
                    self.heading[1] /= heading_length;
                    self.heading[2] /= heading_length;
                }
            }
        }
    }

    /// Gets the leader agent (if any)
    pub fn get_leader(&self) -> Option<&FormationAgent> {
        self.agents
            .iter()
            .find(|agent| agent.role == FormationRole::Leader)
    }

    /// Gets all followers in the formation
    pub fn get_followers(&self) -> Vec<&FormationAgent> {
        self.agents
            .iter()
            .filter(|agent| agent.role == FormationRole::Follower)
            .collect()
    }

    /// Checks if the formation is cohesive (all agents in position)
    pub fn is_cohesive(&self, agent_positions: &HashMap<usize, [f32; 3]>) -> bool {
        let threshold = self.config.max_distance_threshold;

        for agent in &self.agents {
            if let Some(pos) = agent_positions.get(&agent.agent_id) {
                let distance = calculate_distance(pos, &agent.formation_position);
                if distance > threshold {
                    return false;
                }
            }
        }

        true
    }

    /// Updates formation state based on current conditions
    pub fn update_state(&mut self, agent_positions: &HashMap<usize, [f32; 3]>, dt: f32) {
        self.age += dt;

        match self.state {
            FormationState::Forming => {
                if self.is_cohesive(agent_positions) {
                    self.state = FormationState::Stable;
                }
            }
            FormationState::Stable => {
                if self.target.is_some() {
                    self.state = FormationState::Moving;
                } else if !self.is_cohesive(agent_positions) {
                    self.state = FormationState::Adapting;
                }
            }
            FormationState::Moving => {
                if let Some(target) = self.target {
                    let distance_to_target = calculate_distance(&self.center, &target);
                    if distance_to_target < 2.0 {
                        self.target = None;
                        self.state = FormationState::Stable;
                    } else if !self.is_cohesive(agent_positions) {
                        self.state = FormationState::Adapting;
                    }
                }
            }
            FormationState::Adapting => {
                if self.is_cohesive(agent_positions) {
                    self.state = if self.target.is_some() {
                        FormationState::Moving
                    } else {
                        FormationState::Stable
                    };
                }
            }
            FormationState::Disbanding => {
                // Formation will be removed by the manager
            }
        }
    }
}

/// Manager for handling multiple formations
#[derive(Debug)]
pub struct FormationManager {
    /// All active formations
    formations: HashMap<usize, Formation>,
    /// Next formation ID to assign
    next_formation_id: usize,
    /// Agent to formation mapping
    agent_to_formation: HashMap<usize, usize>,
}

impl FormationManager {
    /// Creates a new formation manager
    pub fn new() -> Self {
        Self {
            formations: HashMap::new(),
            next_formation_id: 1,
            agent_to_formation: HashMap::new(),
        }
    }

    /// Creates a new formation
    pub fn create_formation(&mut self, config: FormationConfig) -> usize {
        let formation_id = self.next_formation_id;
        self.next_formation_id += 1;

        let formation = Formation::new(formation_id, config);
        self.formations.insert(formation_id, formation);

        formation_id
    }

    /// Adds an agent to a formation
    pub fn add_agent_to_formation(
        &mut self,
        formation_id: usize,
        agent_id: usize,
        role: FormationRole,
    ) -> Result<()> {
        // Remove agent from any existing formation first
        self.remove_agent_from_formation(agent_id)?;

        let formation = self
            .formations
            .get_mut(&formation_id)
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        formation.add_agent(agent_id, role)?;
        self.agent_to_formation.insert(agent_id, formation_id);

        Ok(())
    }

    /// Removes an agent from their current formation
    pub fn remove_agent_from_formation(&mut self, agent_id: usize) -> Result<()> {
        if let Some(formation_id) = self.agent_to_formation.remove(&agent_id) {
            if let Some(formation) = self.formations.get_mut(&formation_id) {
                formation.remove_agent(agent_id)?;

                // Remove formation if it's disbanding
                if formation.state == FormationState::Disbanding {
                    self.formations.remove(&formation_id);
                }
            }
        }
        Ok(())
    }

    /// Gets the formation ID for an agent
    pub fn get_agent_formation(&self, agent_id: usize) -> Option<usize> {
        self.agent_to_formation.get(&agent_id).copied()
    }

    /// Gets a formation by ID
    pub fn get_formation(&self, formation_id: usize) -> Option<&Formation> {
        self.formations.get(&formation_id)
    }

    /// Gets a mutable formation by ID
    pub fn get_formation_mut(&mut self, formation_id: usize) -> Option<&mut Formation> {
        self.formations.get_mut(&formation_id)
    }

    /// Sets a target for a formation
    pub fn set_formation_target(&mut self, formation_id: usize, target: [f32; 3]) -> Result<()> {
        let formation = self
            .formations
            .get_mut(&formation_id)
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        formation.set_target(target);
        Ok(())
    }

    /// Updates all formations
    pub fn update(&mut self, agent_positions: &HashMap<usize, [f32; 3]>, dt: f32) {
        let mut formations_to_remove = Vec::new();

        for (formation_id, formation) in &mut self.formations {
            formation.update_center(agent_positions);
            formation.update_heading(dt);
            formation.update_formation_positions();
            formation.update_state(agent_positions, dt);

            if formation.state == FormationState::Disbanding {
                formations_to_remove.push(*formation_id);
            }
        }

        // Remove disbanding formations
        for formation_id in formations_to_remove {
            if let Some(formation) = self.formations.remove(&formation_id) {
                for agent in formation.agents {
                    self.agent_to_formation.remove(&agent.agent_id);
                }
            }
        }
    }

    /// Gets formation positions for agents (returns desired formation positions)
    pub fn get_formation_positions(&self) -> HashMap<usize, [f32; 3]> {
        let mut positions = HashMap::new();

        for formation in self.formations.values() {
            for agent in &formation.agents {
                positions.insert(agent.agent_id, agent.formation_position);
            }
        }

        positions
    }

    /// Gets all formation IDs
    pub fn get_formation_ids(&self) -> Vec<usize> {
        self.formations.keys().copied().collect()
    }

    /// Gets formation count
    pub fn get_formation_count(&self) -> usize {
        self.formations.len()
    }

    /// Dissolves a formation (removes all agents and deletes formation)
    pub fn dissolve_formation(&mut self, formation_id: usize) -> Result<()> {
        let formation = self
            .formations
            .remove(&formation_id)
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Remove agent mappings
        for agent in formation.agents {
            self.agent_to_formation.remove(&agent.agent_id);
        }

        Ok(())
    }
}

impl Default for FormationManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculates the distance between two 3D points
fn calculate_distance(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_formation_creation() {
        let config = FormationConfig::default();
        let formation = Formation::new(1, config);

        assert_eq!(formation.id, 1);
        assert_eq!(formation.state, FormationState::Forming);
        assert!(formation.agents.is_empty());
    }

    #[test]
    fn test_add_agent_to_formation() {
        let config = FormationConfig::default();
        let mut formation = Formation::new(1, config);

        assert!(formation.add_agent(0, FormationRole::Leader).is_ok());
        assert_eq!(formation.agents.len(), 1);
        assert_eq!(formation.agents[0].role, FormationRole::Leader);
    }

    #[test]
    fn test_formation_manager() {
        let mut manager = FormationManager::new();
        let config = FormationConfig::default();

        let formation_id = manager.create_formation(config);
        assert_eq!(formation_id, 1);

        assert!(
            manager
                .add_agent_to_formation(formation_id, 0, FormationRole::Leader)
                .is_ok()
        );
        assert_eq!(manager.get_agent_formation(0), Some(formation_id));
    }

    #[test]
    fn test_formation_positions() {
        let config = FormationConfig {
            formation_type: FormationType::Line,
            agent_spacing: 2.0,
            ..Default::default()
        };
        let mut formation = Formation::new(1, config);

        formation.add_agent(0, FormationRole::Leader).unwrap();
        formation.add_agent(1, FormationRole::Follower).unwrap();

        // Check that positions are calculated correctly
        assert_eq!(formation.agents[0].relative_position, [0.0, 0.0, 0.0]);
        assert_eq!(formation.agents[1].relative_position, [0.0, 0.0, -2.0]);
    }

    #[test]
    fn test_formation_types() {
        for formation_type in [
            FormationType::Line,
            FormationType::Column,
            FormationType::Wedge,
            FormationType::Circle,
            FormationType::Box,
            FormationType::Diamond,
        ] {
            let config = FormationConfig {
                formation_type,
                ..Default::default()
            };
            let mut formation = Formation::new(1, config);

            // Should be able to add agents to any formation type
            assert!(formation.add_agent(0, FormationRole::Leader).is_ok());
            assert!(formation.add_agent(1, FormationRole::Follower).is_ok());
        }
    }
}
