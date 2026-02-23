//! Crowd formation management for Detour
//!
//! This module provides functionality for managing groups of agents in specific
//! formations, including formation types, leadership roles, and dynamic formation
//! behaviors.

use std::collections::HashMap;
use std::f32::consts::PI;

use glam::Vec3;

use crate::error::CrowdError;

/// Maximum number of agents in a single formation
const MAX_FORMATION_SIZE: usize = 64;

/// Formation types supported by the formation management system
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FormationType {
    Line,
    Column,
    Wedge,
    Circle,
    Box,
    Diamond,
    /// User-defined positions.
    Custom,
}

/// Formation state for tracking formation behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormationState {
    Forming,
    Stable,
    Moving,
    /// Adapting to environmental constraints.
    Adapting,
    Disbanding,
}

#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct FormationConfig {
    pub formation_type: FormationType,
    pub agent_spacing: f32,
    /// How tightly agents maintain formation positions (0.0..1.0).
    pub cohesion_strength: f32,
    pub adaptation_speed: f32,
    /// Maximum distance from formation center before rejoining.
    pub max_distance_threshold: f32,
    pub auto_adapt: bool,
    /// Priority of formation maintenance vs individual navigation (0.0..1.0).
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormationRole {
    Leader,
    Follower,
    Scout,
    Guard,
}

#[derive(Debug, Clone)]
pub struct FormationAgent {
    pub(crate) agent_id: usize,
    pub(crate) role: FormationRole,
    /// Position relative to formation center.
    pub(crate) relative_position: [f32; 3],
    /// World-space formation position.
    pub(crate) formation_position: [f32; 3],
    pub(crate) influence_weight: f32,
    #[allow(dead_code)]
    pub(crate) out_of_formation_time: f32,
}

impl FormationAgent {
    pub fn new(agent_id: usize, role: FormationRole, relative_position: Vec3) -> Self {
        let relative_position = relative_position.to_array();
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

#[derive(Debug)]
pub struct Formation {
    pub id: usize,
    pub config: FormationConfig,
    pub state: FormationState,
    pub agents: Vec<FormationAgent>,
    pub center: [f32; 3],
    pub heading: [f32; 3],
    pub target: Option<[f32; 3]>,
    pub speed: f32,
    pub age: f32,
}

impl Formation {
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

    pub fn add_agent(&mut self, agent_id: usize, role: FormationRole) -> Result<(), CrowdError> {
        if self.agents.len() >= MAX_FORMATION_SIZE {
            return Err(CrowdError::InvalidParam);
        }

        let relative_position = self.calculate_formation_position(self.agents.len(), role)?;

        let formation_agent = FormationAgent::new(agent_id, role, Vec3::from(relative_position));
        self.agents.push(formation_agent);

        if self.agents.len() == 1 && role == FormationRole::Leader {}

        Ok(())
    }

    pub fn remove_agent(&mut self, agent_id: usize) -> Result<(), CrowdError> {
        let initial_len = self.agents.len();
        self.agents.retain(|agent| agent.agent_id != agent_id);

        if self.agents.len() == initial_len {
            return Err(CrowdError::AgentNotFound { index: agent_id });
        }

        if self.agents.is_empty() {
            self.state = FormationState::Disbanding;
        }

        Ok(())
    }

    fn calculate_formation_position(
        &self,
        index: usize,
        _role: FormationRole,
    ) -> Result<[f32; 3], CrowdError> {
        let spacing = self.config.agent_spacing;

        match self.config.formation_type {
            FormationType::Line => {
                let x = 0.0;
                let z = -(index as f32) * spacing;
                Ok([x, 0.0, z])
            }
            FormationType::Column => {
                let x = if index % 2 == 0 {
                    -spacing * 0.5
                } else {
                    spacing * 0.5
                };
                let z = -(index as f32 / 2.0).floor() * spacing;
                Ok([x, 0.0, z])
            }
            FormationType::Wedge => {
                if index == 0 {
                    Ok([0.0, 0.0, 0.0])
                } else {
                    let side = if index % 2 == 1 { -1.0 } else { 1.0 };
                    let row = (index as f32 / 2.0).ceil();
                    let x = side * row * spacing * 0.7;
                    let z = -row * spacing;
                    Ok([x, 0.0, z])
                }
            }
            FormationType::Circle => {
                if index == 0 {
                    Ok([0.0, 0.0, 0.0])
                } else if self.agents.len() <= 1 {
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
                let cols = (((self.agents.len() as f32).sqrt()).ceil() as usize).max(1);
                let row = index / cols;
                let col = index % cols;
                let x = (col as f32 - cols as f32 * 0.5) * spacing;
                let z = -(row as f32) * spacing;
                Ok([x, 0.0, z])
            }
            FormationType::Diamond => {
                if index == 0 {
                    Ok([0.0, 0.0, 0.0])
                } else if index <= 2 {
                    let side = if index == 1 { -1.0 } else { 1.0 };
                    Ok([side * spacing, 0.0, -spacing])
                } else {
                    Ok([0.0, 0.0, -spacing * 2.0])
                }
            }
            FormationType::Custom => Ok([0.0, 0.0, -(index as f32) * spacing]),
        }
    }

    /// Recalculates center as influence-weighted average of agent positions.
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

    /// Transforms relative positions by heading rotation into world-space formation positions.
    pub fn update_formation_positions(&mut self) {
        let rotation_matrix = self.calculate_rotation_matrix();

        for agent in &mut self.agents {
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

    fn calculate_rotation_matrix(&self) -> [[f32; 3]; 3] {
        let heading = self.heading;
        let angle = heading[2].atan2(heading[0]);
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        [[cos_a, 0.0, -sin_a], [0.0, 1.0, 0.0], [sin_a, 0.0, cos_a]]
    }

    pub fn set_target(&mut self, target: Vec3) {
        let target = target.to_array();
        self.target = Some(target);
        self.state = FormationState::Moving;
    }

    /// Smoothly rotates heading towards the target at the configured adaptation speed.
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

    pub fn get_leader(&self) -> Option<&FormationAgent> {
        self.agents
            .iter()
            .find(|agent| agent.role == FormationRole::Leader)
    }

    pub fn get_followers(&self) -> Vec<&FormationAgent> {
        self.agents
            .iter()
            .filter(|agent| agent.role == FormationRole::Follower)
            .collect()
    }

    /// Returns true if all agents are within `max_distance_threshold` of their formation positions.
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
            FormationState::Disbanding => {}
        }
    }
}

#[derive(Debug)]
pub struct FormationManager {
    formations: HashMap<usize, Formation>,
    next_formation_id: usize,
    agent_to_formation: HashMap<usize, usize>,
}

impl FormationManager {
    pub fn new() -> Self {
        Self {
            formations: HashMap::new(),
            next_formation_id: 1,
            agent_to_formation: HashMap::new(),
        }
    }

    pub fn create_formation(&mut self, config: FormationConfig) -> usize {
        let formation_id = self.next_formation_id;
        self.next_formation_id += 1;

        let formation = Formation::new(formation_id, config);
        self.formations.insert(formation_id, formation);

        formation_id
    }

    pub fn add_agent_to_formation(
        &mut self,
        formation_id: usize,
        agent_id: usize,
        role: FormationRole,
    ) -> Result<(), CrowdError> {
        self.remove_agent_from_formation(agent_id)?;

        let formation = self
            .formations
            .get_mut(&formation_id)
            .ok_or(CrowdError::InvalidParam)?;

        formation.add_agent(agent_id, role)?;
        self.agent_to_formation.insert(agent_id, formation_id);

        Ok(())
    }

    pub fn remove_agent_from_formation(&mut self, agent_id: usize) -> Result<(), CrowdError> {
        if let Some(formation_id) = self.agent_to_formation.remove(&agent_id) {
            if let Some(formation) = self.formations.get_mut(&formation_id) {
                formation.remove_agent(agent_id)?;

                if formation.state == FormationState::Disbanding {
                    self.formations.remove(&formation_id);
                }
            }
        }
        Ok(())
    }

    pub fn get_agent_formation(&self, agent_id: usize) -> Option<usize> {
        self.agent_to_formation.get(&agent_id).copied()
    }

    pub fn get_formation(&self, formation_id: usize) -> Option<&Formation> {
        self.formations.get(&formation_id)
    }

    pub fn get_formation_mut(&mut self, formation_id: usize) -> Option<&mut Formation> {
        self.formations.get_mut(&formation_id)
    }

    pub fn set_formation_target(
        &mut self,
        formation_id: usize,
        target: Vec3,
    ) -> Result<(), CrowdError> {
        let formation = self
            .formations
            .get_mut(&formation_id)
            .ok_or(CrowdError::InvalidParam)?;

        formation.set_target(target);
        Ok(())
    }

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

        for formation_id in formations_to_remove {
            if let Some(formation) = self.formations.remove(&formation_id) {
                for agent in formation.agents {
                    self.agent_to_formation.remove(&agent.agent_id);
                }
            }
        }
    }

    pub fn get_formation_positions(&self) -> HashMap<usize, [f32; 3]> {
        let mut positions = HashMap::new();

        for formation in self.formations.values() {
            for agent in &formation.agents {
                positions.insert(agent.agent_id, agent.formation_position);
            }
        }

        positions
    }

    pub fn get_formation_ids(&self) -> Vec<usize> {
        self.formations.keys().copied().collect()
    }

    pub fn get_formation_count(&self) -> usize {
        self.formations.len()
    }

    pub fn dissolve_formation(&mut self, formation_id: usize) -> Result<(), CrowdError> {
        let formation = self
            .formations
            .remove(&formation_id)
            .ok_or(CrowdError::InvalidParam)?;

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

            assert!(formation.add_agent(0, FormationRole::Leader).is_ok());
            assert!(formation.add_agent(1, FormationRole::Follower).is_ok());
        }
    }
}
