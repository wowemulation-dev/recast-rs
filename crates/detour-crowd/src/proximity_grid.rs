//! Proximity grid implementation for efficient spatial queries in crowd simulation
//!
//! This module provides a 2D spatial grid that enables fast neighbor queries
//! for agents in a crowd simulation. Instead of O(n²) distance checks,
//! the grid provides O(1) cell lookup and only checks agents in nearby cells.

use std::collections::HashMap;

/// Default cell size for the proximity grid (in world units)
const DEFAULT_CELL_SIZE: f32 = 4.0;

/// Maximum number of agents per grid cell (for performance)
const MAX_AGENTS_PER_CELL: usize = 32;

/// Agent data stored in the proximity grid
#[derive(Debug, Clone, Copy)]
pub struct GridAgent {
    /// Agent ID
    pub id: usize,
    /// Agent position (only X and Z are used for 2D grid)
    pub pos: [f32; 3],
    /// Agent radius for collision detection
    pub radius: f32,
}

/// Grid cell coordinates
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct GridCoord {
    x: i32,
    z: i32,
}

impl GridCoord {
    /// Creates new grid coordinates
    fn new(x: i32, z: i32) -> Self {
        Self { x, z }
    }

    /// Converts world position to grid coordinates
    fn from_world_pos(pos: [f32; 3], cell_size: f32) -> Self {
        Self {
            x: (pos[0] / cell_size).floor() as i32,
            z: (pos[2] / cell_size).floor() as i32,
        }
    }

    /// Gets the 9 neighboring cells (including this cell)
    fn get_neighbors(&self) -> Vec<GridCoord> {
        let mut neighbors = Vec::with_capacity(9);
        for dx in -1..=1 {
            for dz in -1..=1 {
                neighbors.push(GridCoord::new(self.x + dx, self.z + dz));
            }
        }
        neighbors
    }
}

/// Grid cell containing agents
#[derive(Debug, Clone)]
struct GridCell {
    /// Agents in this cell
    agents: Vec<GridAgent>,
}

impl GridCell {
    /// Creates a new empty grid cell
    fn new() -> Self {
        Self { agents: Vec::new() }
    }

    /// Adds an agent to the cell
    fn add_agent(&mut self, agent: GridAgent) -> bool {
        if self.agents.len() >= MAX_AGENTS_PER_CELL {
            return false; // Cell is full
        }
        self.agents.push(agent);
        true
    }

    /// Removes an agent from the cell
    fn remove_agent(&mut self, agent_id: usize) -> bool {
        if let Some(pos) = self.agents.iter().position(|a| a.id == agent_id) {
            self.agents.swap_remove(pos);
            true
        } else {
            false
        }
    }

    /// Gets all agents in the cell
    fn get_agents(&self) -> &[GridAgent] {
        &self.agents
    }

    /// Checks if the cell is empty
    fn is_empty(&self) -> bool {
        self.agents.is_empty()
    }
}

/// Proximity grid for efficient spatial queries
#[derive(Debug)]
pub struct ProximityGrid {
    /// Grid cells indexed by coordinates
    cells: HashMap<GridCoord, GridCell>,
    /// Cell size in world units
    cell_size: f32,
    /// Agent position cache for efficient updates
    agent_positions: HashMap<usize, GridCoord>,
    /// Grid bounds for optimization
    min_coord: GridCoord,
    max_coord: GridCoord,
    /// Total number of agents in the grid
    agent_count: usize,
}

impl ProximityGrid {
    /// Creates a new proximity grid
    pub fn new(cell_size: f32) -> Self {
        Self {
            cells: HashMap::new(),
            cell_size: cell_size.max(0.1), // Minimum cell size to avoid division by zero
            agent_positions: HashMap::new(),
            min_coord: GridCoord::new(0, 0),
            max_coord: GridCoord::new(0, 0),
            agent_count: 0,
        }
    }

    /// Creates a new proximity grid with default cell size
    pub fn new_default() -> Self {
        Self::new(DEFAULT_CELL_SIZE)
    }

    /// Adds or updates an agent in the grid
    pub fn update_agent(&mut self, agent: GridAgent) -> bool {
        // Remove agent from old position if it exists
        if let Some(old_coord) = self.agent_positions.get(&agent.id) {
            if let Some(cell) = self.cells.get_mut(old_coord) {
                cell.remove_agent(agent.id);
                // Remove empty cells to save memory
                if cell.is_empty() {
                    self.cells.remove(old_coord);
                }
            }
        } else {
            // New agent
            self.agent_count += 1;
        }

        // Calculate new grid position
        let new_coord = GridCoord::from_world_pos(agent.pos, self.cell_size);

        // Add agent to new position
        let cell = self.cells.entry(new_coord).or_insert_with(GridCell::new);
        if !cell.add_agent(agent) {
            // Cell is full, agent couldn't be added
            return false;
        }

        // Update agent position cache
        self.agent_positions.insert(agent.id, new_coord);

        // Update grid bounds
        if self.agent_count == 1 {
            self.min_coord = new_coord;
            self.max_coord = new_coord;
        } else {
            self.min_coord.x = self.min_coord.x.min(new_coord.x);
            self.min_coord.z = self.min_coord.z.min(new_coord.z);
            self.max_coord.x = self.max_coord.x.max(new_coord.x);
            self.max_coord.z = self.max_coord.z.max(new_coord.z);
        }

        true
    }

    /// Removes an agent from the grid
    pub fn remove_agent(&mut self, agent_id: usize) -> bool {
        if let Some(coord) = self.agent_positions.remove(&agent_id) {
            if let Some(cell) = self.cells.get_mut(&coord) {
                let removed = cell.remove_agent(agent_id);
                // Remove empty cells to save memory
                if cell.is_empty() {
                    self.cells.remove(&coord);
                }
                if removed {
                    self.agent_count -= 1;
                }
                removed
            } else {
                false
            }
        } else {
            false
        }
    }

    /// Queries for agents near a given position
    pub fn query_agents(&self, pos: [f32; 3], radius: f32) -> Vec<GridAgent> {
        let mut result = Vec::new();
        let center_coord = GridCoord::from_world_pos(pos, self.cell_size);

        // Calculate how many cells we need to check based on radius
        let cell_radius = ((radius / self.cell_size).ceil() as i32).max(1);

        // Check all cells within the radius
        for dx in -cell_radius..=cell_radius {
            for dz in -cell_radius..=cell_radius {
                let coord = GridCoord::new(center_coord.x + dx, center_coord.z + dz);

                if let Some(cell) = self.cells.get(&coord) {
                    for &agent in cell.get_agents() {
                        // Calculate actual distance to filter agents
                        let dist_sqr = distance_squared_2d(pos, agent.pos);
                        let combined_radius = radius + agent.radius;

                        if dist_sqr <= combined_radius * combined_radius {
                            result.push(agent);
                        }
                    }
                }
            }
        }

        result
    }

    /// Queries for agents in a specific cell
    pub fn query_cell(&self, pos: [f32; 3]) -> Vec<GridAgent> {
        let coord = GridCoord::from_world_pos(pos, self.cell_size);

        if let Some(cell) = self.cells.get(&coord) {
            cell.get_agents().to_vec()
        } else {
            Vec::new()
        }
    }

    /// Gets all agents within the immediate neighboring cells (3x3 grid)
    pub fn query_neighbors(&self, pos: [f32; 3]) -> Vec<GridAgent> {
        let mut result = Vec::new();
        let center_coord = GridCoord::from_world_pos(pos, self.cell_size);

        for neighbor_coord in center_coord.get_neighbors() {
            if let Some(cell) = self.cells.get(&neighbor_coord) {
                result.extend_from_slice(cell.get_agents());
            }
        }

        result
    }

    /// Clears all agents from the grid
    pub fn clear(&mut self) {
        self.cells.clear();
        self.agent_positions.clear();
        self.agent_count = 0;
        self.min_coord = GridCoord::new(0, 0);
        self.max_coord = GridCoord::new(0, 0);
    }

    /// Gets the total number of agents in the grid
    pub fn get_agent_count(&self) -> usize {
        self.agent_count
    }

    /// Gets the number of active cells
    pub fn get_cell_count(&self) -> usize {
        self.cells.len()
    }

    /// Gets the cell size
    pub fn get_cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Gets grid statistics for debugging
    pub fn get_stats(&self) -> ProximityGridStats {
        let mut min_agents_per_cell = usize::MAX;
        let mut max_agents_per_cell = 0;
        let mut total_agents_in_cells = 0;

        for cell in self.cells.values() {
            let count = cell.agents.len();
            min_agents_per_cell = min_agents_per_cell.min(count);
            max_agents_per_cell = max_agents_per_cell.max(count);
            total_agents_in_cells += count;
        }

        if self.cells.is_empty() {
            min_agents_per_cell = 0;
        }

        ProximityGridStats {
            total_agents: self.agent_count,
            active_cells: self.cells.len(),
            min_agents_per_cell,
            max_agents_per_cell,
            avg_agents_per_cell: if self.cells.is_empty() {
                0.0
            } else {
                total_agents_in_cells as f32 / self.cells.len() as f32
            },
            cell_size: self.cell_size,
            grid_width: (self.max_coord.x - self.min_coord.x + 1) as usize,
            grid_height: (self.max_coord.z - self.min_coord.z + 1) as usize,
        }
    }

    /// Optimizes the grid by removing empty cells and compacting data
    pub fn optimize(&mut self) {
        // Remove empty cells
        self.cells.retain(|_, cell| !cell.is_empty());

        // Rebuild bounds
        if self.agent_count == 0 {
            self.min_coord = GridCoord::new(0, 0);
            self.max_coord = GridCoord::new(0, 0);
        } else {
            let mut first = true;
            for &coord in self.cells.keys() {
                if first {
                    self.min_coord = coord;
                    self.max_coord = coord;
                    first = false;
                } else {
                    self.min_coord.x = self.min_coord.x.min(coord.x);
                    self.min_coord.z = self.min_coord.z.min(coord.z);
                    self.max_coord.x = self.max_coord.x.max(coord.x);
                    self.max_coord.z = self.max_coord.z.max(coord.z);
                }
            }
        }
    }
}

/// Statistics for the proximity grid
#[derive(Debug, Clone)]
pub struct ProximityGridStats {
    /// Total number of agents
    pub total_agents: usize,
    /// Number of active cells
    pub active_cells: usize,
    /// Minimum agents per cell
    pub min_agents_per_cell: usize,
    /// Maximum agents per cell
    pub max_agents_per_cell: usize,
    /// Average agents per cell
    pub avg_agents_per_cell: f32,
    /// Cell size in world units
    pub cell_size: f32,
    /// Grid width in cells
    pub grid_width: usize,
    /// Grid height in cells
    pub grid_height: usize,
}

/// Calculates squared distance in 2D (ignoring Y axis)
#[inline]
fn distance_squared_2d(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dz = a[2] - b[2];
    dx * dx + dz * dz
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_coord_from_world_pos() {
        let coord = GridCoord::from_world_pos([4.5, 0.0, 7.2], 2.0);
        assert_eq!(coord.x, 2);
        assert_eq!(coord.z, 3);

        let coord = GridCoord::from_world_pos([-1.5, 0.0, -3.7], 2.0);
        assert_eq!(coord.x, -1);
        assert_eq!(coord.z, -2);
    }

    #[test]
    fn test_grid_neighbors() {
        let coord = GridCoord::new(1, 1);
        let neighbors = coord.get_neighbors();
        assert_eq!(neighbors.len(), 9);
        assert!(neighbors.contains(&GridCoord::new(0, 0)));
        assert!(neighbors.contains(&GridCoord::new(1, 1)));
        assert!(neighbors.contains(&GridCoord::new(2, 2)));
    }

    #[test]
    fn test_proximity_grid_basic() {
        let mut grid = ProximityGrid::new(2.0);

        // Add some agents
        let agent1 = GridAgent {
            id: 1,
            pos: [1.0, 0.0, 1.0],
            radius: 0.5,
        };
        let agent2 = GridAgent {
            id: 2,
            pos: [3.0, 0.0, 3.0],
            radius: 0.5,
        };
        let agent3 = GridAgent {
            id: 3,
            pos: [10.0, 0.0, 10.0],
            radius: 0.5,
        };

        assert!(grid.update_agent(agent1));
        assert!(grid.update_agent(agent2));
        assert!(grid.update_agent(agent3));

        assert_eq!(grid.get_agent_count(), 3);

        // Query near agent1
        let nearby = grid.query_agents([1.0, 0.0, 1.0], 3.0);
        assert_eq!(nearby.len(), 2); // Should find agent1 and agent2

        // Query near agent3
        let nearby = grid.query_agents([10.0, 0.0, 10.0], 3.0);
        assert_eq!(nearby.len(), 1); // Should only find agent3
    }

    #[test]
    fn test_agent_removal() {
        let mut grid = ProximityGrid::new(2.0);

        let agent1 = GridAgent {
            id: 1,
            pos: [1.0, 0.0, 1.0],
            radius: 0.5,
        };
        let agent2 = GridAgent {
            id: 2,
            pos: [1.5, 0.0, 1.5],
            radius: 0.5,
        };

        grid.update_agent(agent1);
        grid.update_agent(agent2);
        assert_eq!(grid.get_agent_count(), 2);

        // Remove agent1
        assert!(grid.remove_agent(1));
        assert_eq!(grid.get_agent_count(), 1);

        // Try to remove agent1 again
        assert!(!grid.remove_agent(1));
        assert_eq!(grid.get_agent_count(), 1);
    }

    #[test]
    fn test_agent_update() {
        let mut grid = ProximityGrid::new(2.0);

        let agent1 = GridAgent {
            id: 1,
            pos: [1.0, 0.0, 1.0],
            radius: 0.5,
        };

        grid.update_agent(agent1);
        assert_eq!(grid.get_agent_count(), 1);

        // Update agent position
        let agent1_moved = GridAgent {
            id: 1,
            pos: [5.0, 0.0, 5.0],
            radius: 0.5,
        };
        grid.update_agent(agent1_moved);
        assert_eq!(grid.get_agent_count(), 1); // Still only one agent

        // Query at old position
        let nearby = grid.query_agents([1.0, 0.0, 1.0], 1.0);
        assert_eq!(nearby.len(), 0); // Should not find the agent

        // Query at new position
        let nearby = grid.query_agents([5.0, 0.0, 5.0], 1.0);
        assert_eq!(nearby.len(), 1); // Should find the agent
    }

    #[test]
    fn test_grid_stats() {
        let mut grid = ProximityGrid::new(2.0);

        // Add agents in same cell
        for i in 0..3 {
            let agent = GridAgent {
                id: i,
                pos: [1.0 + i as f32 * 0.1, 0.0, 1.0],
                radius: 0.5,
            };
            grid.update_agent(agent);
        }

        let stats = grid.get_stats();
        assert_eq!(stats.total_agents, 3);
        assert_eq!(stats.active_cells, 1);
        assert_eq!(stats.max_agents_per_cell, 3);
        assert_eq!(stats.cell_size, 2.0);
    }

    #[test]
    fn test_query_neighbors() {
        let mut grid = ProximityGrid::new(2.0);

        // Add agents in different cells
        let positions = [
            [1.0, 0.0, 1.0],   // Cell (0, 0)
            [3.0, 0.0, 1.0],   // Cell (1, 0)
            [1.0, 0.0, 3.0],   // Cell (0, 1)
            [3.0, 0.0, 3.0],   // Cell (1, 1)
            [10.0, 0.0, 10.0], // Cell (5, 5) - far away
        ];

        for (i, &pos) in positions.iter().enumerate() {
            let agent = GridAgent {
                id: i,
                pos,
                radius: 0.5,
            };
            grid.update_agent(agent);
        }

        // Query neighbors of position in cell (0, 0)
        let neighbors = grid.query_neighbors([1.0, 0.0, 1.0]);
        assert!(neighbors.len() >= 4); // Should find the 4 nearby agents
        assert!(neighbors.len() <= 4); // Should not find the far agent
    }

    #[test]
    fn test_grid_optimization() {
        let mut grid = ProximityGrid::new(2.0);

        // Add and remove agents to create empty cells
        for i in 0..5 {
            let agent = GridAgent {
                id: i,
                pos: [i as f32 * 3.0, 0.0, 0.0],
                radius: 0.5,
            };
            grid.update_agent(agent);
        }

        // Remove some agents
        grid.remove_agent(1);
        grid.remove_agent(3);

        let stats_before = grid.get_stats();
        grid.optimize();
        let stats_after = grid.get_stats();

        assert_eq!(stats_after.total_agents, 3);
        assert!(stats_after.active_cells <= stats_before.active_cells);
    }
}
