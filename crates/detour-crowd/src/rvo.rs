//! Reciprocal Velocity Obstacles (RVO) implementation for crowd simulation
//!
//! This module implements the RVO algorithm for realistic collision avoidance
//! between agents in a crowd. RVO ensures that agents can smoothly avoid
//! collisions with each other while maintaining their desired motion.

use recast_common::Result;

/// Configuration parameters for RVO
#[derive(Debug, Clone)]
pub struct RVOConfig {
    /// Time horizon for collision avoidance
    pub time_horizon: f32,
    /// Time horizon for obstacle avoidance
    pub time_horizon_obs: f32,
    /// Maximum neighbors to consider
    pub max_neighbors: usize,
    /// Neighbor distance
    pub neighbor_dist: f32,
    /// Maximum speed
    pub max_speed: f32,
    /// Agent radius
    pub radius: f32,
}

impl Default for RVOConfig {
    fn default() -> Self {
        Self {
            time_horizon: 2.0,
            time_horizon_obs: 2.0,
            max_neighbors: 10,
            neighbor_dist: 15.0,
            max_speed: 2.0,
            radius: 0.5,
        }
    }
}

/// RVO Agent representation
#[derive(Debug, Clone)]
pub struct RVOAgent {
    /// Agent ID
    pub id: usize,
    /// Current position
    pub position: [f32; 2],
    /// Current velocity
    pub velocity: [f32; 2],
    /// Preferred velocity (desired direction and speed)
    pub pref_velocity: [f32; 2],
    /// Agent configuration
    pub config: RVOConfig,
    /// ORCA lines (optimal reciprocal collision avoidance)
    orca_lines: Vec<ORCALine>,
    /// New velocity (computed by RVO)
    new_velocity: [f32; 2],
}

impl RVOAgent {
    /// Creates a new RVO agent
    pub fn new(id: usize, position: [f32; 2], config: RVOConfig) -> Self {
        Self {
            id,
            position,
            velocity: [0.0, 0.0],
            pref_velocity: [0.0, 0.0],
            config,
            orca_lines: Vec::new(),
            new_velocity: [0.0, 0.0],
        }
    }

    /// Sets the preferred velocity for this agent
    pub fn set_pref_velocity(&mut self, pref_vel: [f32; 2]) {
        self.pref_velocity = pref_vel;
    }

    /// Gets the computed new velocity
    pub fn get_new_velocity(&self) -> [f32; 2] {
        self.new_velocity
    }

    /// Updates the agent's position and velocity
    pub fn update(&mut self, dt: f32) {
        self.velocity = self.new_velocity;
        self.position[0] += self.velocity[0] * dt;
        self.position[1] += self.velocity[1] * dt;
    }
}

/// ORCA (Optimal Reciprocal Collision Avoidance) line
#[derive(Debug, Clone)]
struct ORCALine {
    /// Point on the line
    point: [f32; 2],
    /// Direction of the line (normalized)
    direction: [f32; 2],
}

impl ORCALine {
    /// Creates a new ORCA line
    fn new(point: [f32; 2], direction: [f32; 2]) -> Self {
        Self { point, direction }
    }
}

/// RVO Simulator for managing multiple agents
#[derive(Debug)]
pub struct RVOSimulator {
    /// All agents in the simulation
    agents: Vec<RVOAgent>,
    /// Default configuration for new agents
    default_config: RVOConfig,
}

impl RVOSimulator {
    /// Creates a new RVO simulator
    pub fn new() -> Self {
        Self {
            agents: Vec::new(),
            default_config: RVOConfig::default(),
        }
    }

    /// Creates a new RVO simulator with custom default configuration
    pub fn new_with_config(config: RVOConfig) -> Self {
        Self {
            agents: Vec::new(),
            default_config: config,
        }
    }

    /// Adds an agent to the simulation
    pub fn add_agent(&mut self, position: [f32; 2]) -> usize {
        let id = self.agents.len();
        let agent = RVOAgent::new(id, position, self.default_config.clone());
        self.agents.push(agent);
        id
    }

    /// Adds an agent with custom configuration
    pub fn add_agent_with_config(&mut self, position: [f32; 2], config: RVOConfig) -> usize {
        let id = self.agents.len();
        let agent = RVOAgent::new(id, position, config);
        self.agents.push(agent);
        id
    }

    /// Sets the preferred velocity for an agent
    pub fn set_agent_pref_velocity(&mut self, agent_id: usize, pref_vel: [f32; 2]) -> Result<()> {
        if agent_id >= self.agents.len() {
            return Err(recast_common::Error::InvalidMesh(
                "Invalid agent ID".to_string(),
            ));
        }
        self.agents[agent_id].set_pref_velocity(pref_vel);
        Ok(())
    }

    /// Gets an agent by ID
    pub fn get_agent(&self, agent_id: usize) -> Option<&RVOAgent> {
        self.agents.get(agent_id)
    }

    /// Gets a mutable reference to an agent by ID
    pub fn get_agent_mut(&mut self, agent_id: usize) -> Option<&mut RVOAgent> {
        self.agents.get_mut(agent_id)
    }

    /// Gets the number of agents
    pub fn get_num_agents(&self) -> usize {
        self.agents.len()
    }

    /// Performs one simulation step
    pub fn step(&mut self, dt: f32) -> Result<()> {
        // Clear ORCA lines for all agents
        for agent in &mut self.agents {
            agent.orca_lines.clear();
        }

        // Compute ORCA lines for each agent
        for i in 0..self.agents.len() {
            self.compute_orca_lines(i)?;
        }

        // Solve linear programs to find new velocities
        for i in 0..self.agents.len() {
            self.solve_linear_program(i);
        }

        // Update agent positions
        for agent in &mut self.agents {
            agent.update(dt);
        }

        Ok(())
    }

    /// Computes ORCA lines for a specific agent
    fn compute_orca_lines(&mut self, agent_idx: usize) -> Result<()> {
        let agent = &self.agents[agent_idx];
        let agent_pos = agent.position;
        let agent_vel = agent.velocity;
        let agent_radius = agent.config.radius;
        let time_horizon = agent.config.time_horizon;
        let max_neighbors = agent.config.max_neighbors;
        let neighbor_dist = agent.config.neighbor_dist;

        // Find neighboring agents
        let mut neighbors = Vec::new();
        for (i, other) in self.agents.iter().enumerate() {
            if i == agent_idx {
                continue;
            }

            let dist_sq = distance_sq(&agent_pos, &other.position);
            if dist_sq < neighbor_dist * neighbor_dist {
                neighbors.push((i, dist_sq));
            }
        }

        // Sort neighbors by distance and limit to max_neighbors
        neighbors.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        neighbors.truncate(max_neighbors);

        // Compute ORCA lines for each neighbor
        let mut orca_lines = Vec::new();

        for (neighbor_idx, _) in neighbors {
            let neighbor = &self.agents[neighbor_idx];
            let relative_pos = [
                neighbor.position[0] - agent_pos[0],
                neighbor.position[1] - agent_pos[1],
            ];
            let relative_vel = [
                agent_vel[0] - neighbor.velocity[0],
                agent_vel[1] - neighbor.velocity[1],
            ];

            let dist_sq = dot(&relative_pos, &relative_pos);
            let combined_radius = agent_radius + neighbor.config.radius;
            let combined_radius_sq = combined_radius * combined_radius;

            let mut line = ORCALine::new([0.0, 0.0], [0.0, 0.0]);

            if dist_sq > combined_radius_sq {
                // No collision. Compute ORCA line.
                let w = [
                    relative_vel[0] - relative_pos[0] / time_horizon,
                    relative_vel[1] - relative_pos[1] / time_horizon,
                ];

                // Vector from cutoff center to relative velocity
                let w_length_sq = dot(&w, &w);
                let dot_product1 = dot(&w, &relative_pos);

                if dot_product1 < 0.0
                    && dot_product1 * dot_product1 > combined_radius_sq * w_length_sq
                {
                    // Project on cutoff circle
                    let w_length = w_length_sq.sqrt();
                    let unit_w = [w[0] / w_length, w[1] / w_length];

                    line.direction = [unit_w[1], -unit_w[0]];
                    let u = [
                        unit_w[0] * combined_radius / time_horizon,
                        unit_w[1] * combined_radius / time_horizon,
                    ];
                    line.point = [agent_vel[0] + 0.5 * u[0], agent_vel[1] + 0.5 * u[1]];
                } else {
                    // Project on legs
                    let leg = (dist_sq - combined_radius_sq).sqrt();

                    if determinant(&relative_pos, &w) > 0.0 {
                        // Project on left leg
                        line.direction = [
                            relative_pos[0] * leg - relative_pos[1] * combined_radius,
                            relative_pos[0] * combined_radius + relative_pos[1] * leg,
                        ];
                    } else {
                        // Project on right leg
                        line.direction = [
                            relative_pos[0] * leg + relative_pos[1] * combined_radius,
                            -relative_pos[0] * combined_radius + relative_pos[1] * leg,
                        ];
                    }

                    let dot_product2 = dot(&relative_vel, &line.direction);
                    let u = [
                        dot_product2 * line.direction[0] - relative_vel[0],
                        dot_product2 * line.direction[1] - relative_vel[1],
                    ];
                    line.point = [agent_vel[0] + 0.5 * u[0], agent_vel[1] + 0.5 * u[1]];
                }
            } else {
                // Collision. Project on cutoff circle of time timeStep
                let inv_time_step = 1.0 / time_horizon;

                // Vector from cutoff center to relative velocity
                let w = [
                    relative_vel[0] - relative_pos[0] * inv_time_step,
                    relative_vel[1] - relative_pos[1] * inv_time_step,
                ];

                let w_length = dot(&w, &w).sqrt();
                let unit_w = [w[0] / w_length, w[1] / w_length];

                line.direction = [unit_w[1], -unit_w[0]];
                let u = [
                    unit_w[0] * combined_radius * inv_time_step,
                    unit_w[1] * combined_radius * inv_time_step,
                ];
                line.point = [agent_vel[0] + 0.5 * u[0], agent_vel[1] + 0.5 * u[1]];
            }

            orca_lines.push(line);
        }

        // Update the agent's ORCA lines
        self.agents[agent_idx].orca_lines = orca_lines;

        Ok(())
    }

    /// Solves the linear program to find the optimal velocity for an agent
    fn solve_linear_program(&mut self, agent_idx: usize) {
        let agent = &self.agents[agent_idx];
        let pref_vel = agent.pref_velocity;
        let max_speed = agent.config.max_speed;
        let orca_lines = agent.orca_lines.clone();

        let mut optimal_velocity = pref_vel;

        // Clamp preferred velocity to max speed
        let pref_speed_sq = dot(&pref_vel, &pref_vel);
        if pref_speed_sq > max_speed * max_speed {
            let pref_speed = pref_speed_sq.sqrt();
            optimal_velocity = [
                pref_vel[0] * max_speed / pref_speed,
                pref_vel[1] * max_speed / pref_speed,
            ];
        }

        // Check if optimal velocity satisfies all ORCA constraints
        for line in &orca_lines {
            if determinant(
                &line.direction,
                &[
                    optimal_velocity[0] - line.point[0],
                    optimal_velocity[1] - line.point[1],
                ],
            ) > 0.0
            {
                // Velocity is on the wrong side of the line, project onto line
                let dot_product =
                    dot(&optimal_velocity, &line.direction) - dot(&line.point, &line.direction);
                optimal_velocity = [
                    optimal_velocity[0] - dot_product * line.direction[0],
                    optimal_velocity[1] - dot_product * line.direction[1],
                ];
            }
        }

        // Ensure velocity is within speed limit
        let speed_sq = dot(&optimal_velocity, &optimal_velocity);
        if speed_sq > max_speed * max_speed {
            let speed = speed_sq.sqrt();
            optimal_velocity = [
                optimal_velocity[0] * max_speed / speed,
                optimal_velocity[1] * max_speed / speed,
            ];
        }

        self.agents[agent_idx].new_velocity = optimal_velocity;
    }

    /// Gets all agent positions (useful for rendering/debugging)
    pub fn get_agent_positions(&self) -> Vec<[f32; 2]> {
        self.agents.iter().map(|agent| agent.position).collect()
    }

    /// Gets all agent velocities (useful for rendering/debugging)
    pub fn get_agent_velocities(&self) -> Vec<[f32; 2]> {
        self.agents.iter().map(|agent| agent.velocity).collect()
    }

    /// Removes an agent from the simulation
    pub fn remove_agent(&mut self, agent_id: usize) -> Result<()> {
        if agent_id >= self.agents.len() {
            return Err(recast_common::Error::InvalidMesh(
                "Invalid agent ID".to_string(),
            ));
        }

        self.agents.remove(agent_id);

        // Update agent IDs to maintain consistency
        for (i, agent) in self.agents.iter_mut().enumerate() {
            agent.id = i;
        }

        Ok(())
    }

    /// Clears all agents from the simulation
    pub fn clear(&mut self) {
        self.agents.clear();
    }
}

impl Default for RVOSimulator {
    fn default() -> Self {
        Self::new()
    }
}

/// Computes the dot product of two 2D vectors
fn dot(a: &[f32; 2], b: &[f32; 2]) -> f32 {
    a[0] * b[0] + a[1] * b[1]
}

/// Computes the determinant of two 2D vectors
fn determinant(a: &[f32; 2], b: &[f32; 2]) -> f32 {
    a[0] * b[1] - a[1] * b[0]
}

/// Computes the squared distance between two 2D points
fn distance_sq(a: &[f32; 2], b: &[f32; 2]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    dx * dx + dy * dy
}

/// Normalizes a 2D vector
#[allow(dead_code)]
fn normalize(v: &mut [f32; 2]) -> f32 {
    let length = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if length > 0.0001 {
        v[0] /= length;
        v[1] /= length;
    }
    length
}

/// Converts a 3D velocity to 2D (ignoring Y component)
pub fn velocity_3d_to_2d(vel_3d: &[f32; 3]) -> [f32; 2] {
    [vel_3d[0], vel_3d[2]]
}

/// Converts a 2D velocity to 3D (setting Y component to 0)
pub fn velocity_2d_to_3d(vel_2d: &[f32; 2]) -> [f32; 3] {
    [vel_2d[0], 0.0, vel_2d[1]]
}

/// Converts a 3D position to 2D (ignoring Y component)
pub fn position_3d_to_2d(pos_3d: &[f32; 3]) -> [f32; 2] {
    [pos_3d[0], pos_3d[2]]
}

/// Converts a 2D position to 3D (setting Y component to input value)
pub fn position_2d_to_3d(pos_2d: &[f32; 2], y: f32) -> [f32; 3] {
    [pos_2d[0], y, pos_2d[1]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rvo_simulator_creation() {
        let sim = RVOSimulator::new();
        assert_eq!(sim.get_num_agents(), 0);
    }

    #[test]
    fn test_add_agent() {
        let mut sim = RVOSimulator::new();
        let agent_id = sim.add_agent([0.0, 0.0]);
        assert_eq!(agent_id, 0);
        assert_eq!(sim.get_num_agents(), 1);

        let agent = sim.get_agent(agent_id).unwrap();
        assert_eq!(agent.position, [0.0, 0.0]);
    }

    #[test]
    fn test_set_pref_velocity() {
        let mut sim = RVOSimulator::new();
        let agent_id = sim.add_agent([0.0, 0.0]);

        sim.set_agent_pref_velocity(agent_id, [1.0, 0.0]).unwrap();

        let agent = sim.get_agent(agent_id).unwrap();
        assert_eq!(agent.pref_velocity, [1.0, 0.0]);
    }

    #[test]
    fn test_simulation_step() {
        let mut sim = RVOSimulator::new();
        let agent1 = sim.add_agent([0.0, 0.0]);
        let agent2 = sim.add_agent([2.0, 0.0]);

        // Set agents moving towards each other
        sim.set_agent_pref_velocity(agent1, [1.0, 0.0]).unwrap();
        sim.set_agent_pref_velocity(agent2, [-1.0, 0.0]).unwrap();

        // Run one simulation step
        sim.step(0.1).unwrap();

        // Agents should avoid collision
        let agent1_pos = sim.get_agent(agent1).unwrap().position;
        let agent2_pos = sim.get_agent(agent2).unwrap().position;

        // They should have moved but not collided
        assert!(agent1_pos[0] > 0.0);
        assert!(agent2_pos[0] < 2.0);

        let distance = ((agent2_pos[0] - agent1_pos[0]).powi(2)
            + (agent2_pos[1] - agent1_pos[1]).powi(2))
        .sqrt();
        assert!(distance > 1.0); // Should maintain safe distance
    }

    #[test]
    fn test_remove_agent() {
        let mut sim = RVOSimulator::new();
        let agent1 = sim.add_agent([0.0, 0.0]);
        let _agent2 = sim.add_agent([1.0, 0.0]);

        assert_eq!(sim.get_num_agents(), 2);

        sim.remove_agent(agent1).unwrap();
        assert_eq!(sim.get_num_agents(), 1);

        // Remaining agent should have ID 0
        let remaining_agent = sim.get_agent(0).unwrap();
        assert_eq!(remaining_agent.position, [1.0, 0.0]);
    }

    #[test]
    fn test_coordinate_conversion() {
        let pos_3d = [1.0, 2.0, 3.0];
        let pos_2d = position_3d_to_2d(&pos_3d);
        assert_eq!(pos_2d, [1.0, 3.0]);

        let pos_3d_back = position_2d_to_3d(&pos_2d, 2.0);
        assert_eq!(pos_3d_back, [1.0, 2.0, 3.0]);

        let vel_3d = [0.5, 1.0, 1.5];
        let vel_2d = velocity_3d_to_2d(&vel_3d);
        assert_eq!(vel_2d, [0.5, 1.5]);

        let vel_3d_back = velocity_2d_to_3d(&vel_2d);
        assert_eq!(vel_3d_back, [0.5, 0.0, 1.5]);
    }

    #[test]
    fn test_dot_product() {
        let a = [1.0, 2.0];
        let b = [3.0, 4.0];
        assert_eq!(dot(&a, &b), 11.0);
    }

    #[test]
    fn test_determinant() {
        let a = [1.0, 2.0];
        let b = [3.0, 4.0];
        assert_eq!(determinant(&a, &b), -2.0);
    }

    #[test]
    fn test_distance_squared() {
        let a = [0.0, 0.0];
        let b = [3.0, 4.0];
        assert_eq!(distance_sq(&a, &b), 25.0);
    }

    #[test]
    fn test_normalize() {
        let mut v = [3.0, 4.0];
        let length = normalize(&mut v);
        assert!((length - 5.0).abs() < 0.001);
        assert!((v[0] - 0.6).abs() < 0.001);
        assert!((v[1] - 0.8).abs() < 0.001);
    }
}
