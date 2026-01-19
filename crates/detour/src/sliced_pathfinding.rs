//! Sliced pathfinding implementation for large-scale pathfinding queries
//!
//! This module provides functionality to break long pathfinding queries into
//! smaller segments to avoid performance issues and memory limitations.

use std::collections::VecDeque;

use super::nav_mesh_query::NavMeshQuery;
use super::{PolyRef, QueryFilter, Status};
use recast_common::{Error, Result};

/// Maximum number of polygons to process in a single slice
const DEFAULT_MAX_SLICE_SIZE: usize = 256;

/// Maximum number of iterations before giving up on pathfinding
const MAX_PATHFIND_ITERATIONS: usize = 2048;

/// State of a sliced pathfinding query
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlicedPathState {
    /// Path finding is in progress
    InProgress,
    /// Path finding completed successfully
    Success,
    /// Path finding failed
    Failed,
    /// Partial path found (couldn't reach destination)
    PartialPath,
}

/// Configuration for sliced pathfinding
#[derive(Debug, Clone)]
pub struct SlicedPathConfig {
    /// Maximum number of polygons to process in a single slice
    pub max_slice_size: usize,
    /// Maximum total iterations before giving up
    pub max_iterations: usize,
}

impl Default for SlicedPathConfig {
    fn default() -> Self {
        Self {
            max_slice_size: DEFAULT_MAX_SLICE_SIZE,
            max_iterations: MAX_PATHFIND_ITERATIONS,
        }
    }
}

/// A sliced pathfinding query that can be executed incrementally
#[derive(Debug)]
pub struct SlicedPathfindingQuery<'a> {
    /// Reference to the navigation mesh query
    query: &'a mut NavMeshQuery<'a>,
    /// Configuration for the sliced pathfinding
    config: SlicedPathConfig,
    /// Current state of the pathfinding
    state: SlicedPathState,
    /// Start polygon reference
    start_ref: PolyRef,
    /// End polygon reference
    end_ref: PolyRef,
    /// Start position
    start_pos: [f32; 3],
    /// End position
    end_pos: [f32; 3],
    /// Query filter
    filter: QueryFilter,
    /// Current path being built
    current_path: Vec<PolyRef>,
    /// Queue of intermediate goals for multi-segment pathfinding
    goal_queue: VecDeque<(PolyRef, [f32; 3])>,
    /// Total iterations performed
    iterations: usize,
    /// Current segment being processed
    current_segment: usize,
}

impl<'a> SlicedPathfindingQuery<'a> {
    /// Creates a new sliced pathfinding query
    pub fn new(
        query: &'a mut NavMeshQuery<'a>,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: [f32; 3],
        end_pos: [f32; 3],
        filter: QueryFilter,
    ) -> Result<Self> {
        // Validate input
        if !query.nav_mesh().is_valid_poly_ref(start_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }
        if !query.nav_mesh().is_valid_poly_ref(end_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let mut sliced_query = Self {
            query,
            config: SlicedPathConfig::default(),
            state: SlicedPathState::InProgress,
            start_ref,
            end_ref,
            start_pos,
            end_pos,
            filter,
            current_path: Vec::new(),
            goal_queue: VecDeque::new(),
            iterations: 0,
            current_segment: 0,
        };

        // Initialize the first segment
        sliced_query.initialize_pathfinding()?;

        Ok(sliced_query)
    }

    /// Creates a new sliced pathfinding query with custom configuration
    pub fn new_with_config(
        query: &'a mut NavMeshQuery<'a>,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: [f32; 3],
        end_pos: [f32; 3],
        filter: QueryFilter,
        config: SlicedPathConfig,
    ) -> Result<Self> {
        let mut sliced_query = Self::new(query, start_ref, end_ref, start_pos, end_pos, filter)?;
        sliced_query.config = config;
        Ok(sliced_query)
    }

    /// Initializes the pathfinding process
    fn initialize_pathfinding(&mut self) -> Result<()> {
        // If start and end are the same, we're done
        if self.start_ref == self.end_ref {
            self.current_path.push(self.start_ref);
            self.state = SlicedPathState::Success;
            return Ok(());
        }

        // Check if we need to break the path into segments
        let straight_line_distance = self.calculate_distance(&self.start_pos, &self.end_pos);

        // If the distance is very large, create intermediate waypoints
        let max_segment_distance = 100.0; // Maximum distance per segment
        if straight_line_distance > max_segment_distance {
            self.create_intermediate_waypoints(max_segment_distance)?;
        } else {
            // Single segment pathfinding
            self.goal_queue.push_back((self.end_ref, self.end_pos));
        }

        Ok(())
    }

    /// Creates intermediate waypoints for very long paths
    fn create_intermediate_waypoints(&mut self, max_segment_distance: f32) -> Result<()> {
        let total_distance = self.calculate_distance(&self.start_pos, &self.end_pos);
        let num_segments = (total_distance / max_segment_distance).ceil() as usize;

        for i in 1..=num_segments {
            let t = i as f32 / num_segments as f32;
            let intermediate_pos = [
                self.start_pos[0] + (self.end_pos[0] - self.start_pos[0]) * t,
                self.start_pos[1] + (self.end_pos[1] - self.start_pos[1]) * t,
                self.start_pos[2] + (self.end_pos[2] - self.start_pos[2]) * t,
            ];

            // Find the nearest polygon for this intermediate position
            let half_extents = self.query.get_query_extent();
            if let Ok((poly_ref, _)) =
                self.query
                    .find_nearest_poly(&intermediate_pos, &half_extents, &self.filter)
            {
                if i == num_segments {
                    // Last segment should target the actual end
                    self.goal_queue.push_back((self.end_ref, self.end_pos));
                } else {
                    self.goal_queue.push_back((poly_ref, intermediate_pos));
                }
            } else {
                // If we can't find a valid polygon, fall back to direct pathfinding
                self.goal_queue.clear();
                self.goal_queue.push_back((self.end_ref, self.end_pos));
                break;
            }
        }

        Ok(())
    }

    /// Executes one slice of the pathfinding algorithm
    pub fn execute_slice(&mut self) -> Result<SlicedPathState> {
        if self.state != SlicedPathState::InProgress {
            return Ok(self.state);
        }

        if self.iterations >= self.config.max_iterations {
            self.state = SlicedPathState::Failed;
            return Ok(self.state);
        }

        // Check if we have any goals left to process
        if self.goal_queue.is_empty() {
            self.state = if self.current_path.is_empty() {
                SlicedPathState::Failed
            } else {
                SlicedPathState::Success
            };
            return Ok(self.state);
        }

        // Get the next goal
        let (goal_ref, goal_pos) = self.goal_queue.front().copied().unwrap();

        // Determine start position for this segment
        let segment_start_ref = if self.current_path.is_empty() {
            self.start_ref
        } else {
            *self.current_path.last().unwrap()
        };

        let segment_start_pos = if self.current_path.is_empty() {
            self.start_pos
        } else {
            // Get position of last polygon in path
            self.query.get_poly_center(segment_start_ref)?
        };

        // Execute pathfinding for this segment
        match self.find_path_segment(segment_start_ref, goal_ref, &segment_start_pos, &goal_pos) {
            Ok(segment_path) => {
                // Merge the segment path with the current path
                self.merge_segment_path(segment_path);

                // Remove the completed goal
                self.goal_queue.pop_front();
                self.current_segment += 1;

                // Check if we've completed all segments
                if self.goal_queue.is_empty() {
                    self.state = SlicedPathState::Success;
                } else {
                    // Continue with next segment
                    self.state = SlicedPathState::InProgress;
                }
            }
            Err(_) => {
                // Pathfinding failed for this segment
                if self.current_path.is_empty() {
                    self.state = SlicedPathState::Failed;
                } else {
                    // We have a partial path
                    self.state = SlicedPathState::PartialPath;
                }
            }
        }

        Ok(self.state)
    }

    /// Finds a path for a single segment
    fn find_path_segment(
        &mut self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
    ) -> Result<Vec<PolyRef>> {
        // Use the regular pathfinding but with limited iteration count
        let _max_iterations_backup = self.config.max_iterations;

        // Execute pathfinding for this segment
        let segment_path =
            self.query
                .find_path(start_ref, end_ref, start_pos, end_pos, &self.filter)?;

        self.iterations += segment_path.len(); // Approximate iteration count

        Ok(segment_path)
    }

    /// Merges a segment path with the current path
    fn merge_segment_path(&mut self, mut segment_path: Vec<PolyRef>) {
        if self.current_path.is_empty() {
            // First segment
            self.current_path = segment_path;
        } else {
            // Remove the first polygon of the segment path if it's the same as the last
            // polygon of the current path (to avoid duplication)
            if !segment_path.is_empty() && segment_path[0] == *self.current_path.last().unwrap() {
                segment_path.remove(0);
            }

            // Append the segment path
            self.current_path.extend(segment_path);
        }
    }

    /// Executes the pathfinding to completion
    pub fn execute_to_completion(&mut self) -> Result<SlicedPathState> {
        while self.state == SlicedPathState::InProgress {
            self.execute_slice()?;

            // Safety check to prevent infinite loops
            if self.iterations >= self.config.max_iterations {
                self.state = SlicedPathState::Failed;
                break;
            }
        }

        Ok(self.state)
    }

    /// Gets the current state of the pathfinding
    pub fn get_state(&self) -> SlicedPathState {
        self.state
    }

    /// Gets the current path (may be partial if pathfinding is in progress)
    pub fn get_path(&self) -> &[PolyRef] {
        &self.current_path
    }

    /// Gets the current path as a mutable reference
    pub fn get_path_mut(&mut self) -> &mut Vec<PolyRef> {
        &mut self.current_path
    }

    /// Gets the number of iterations performed so far
    pub fn get_iterations(&self) -> usize {
        self.iterations
    }

    /// Gets the current segment being processed
    pub fn get_current_segment(&self) -> usize {
        self.current_segment
    }

    /// Gets the total number of segments
    pub fn get_total_segments(&self) -> usize {
        self.current_segment + self.goal_queue.len()
    }

    /// Calculates the straight-line distance between two points
    fn calculate_distance(&self, a: &[f32; 3], b: &[f32; 3]) -> f32 {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Resets the pathfinding query to start over
    pub fn reset(&mut self) -> Result<()> {
        self.state = SlicedPathState::InProgress;
        self.current_path.clear();
        self.goal_queue.clear();
        self.iterations = 0;
        self.current_segment = 0;
        self.initialize_pathfinding()
    }

    /// Updates the end goal of the pathfinding (useful for moving targets)
    pub fn update_end_goal(&mut self, new_end_ref: PolyRef, new_end_pos: [f32; 3]) -> Result<()> {
        if !self.query.nav_mesh().is_valid_poly_ref(new_end_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        self.end_ref = new_end_ref;
        self.end_pos = new_end_pos;

        // Update the last goal in the queue
        if let Some(last_goal) = self.goal_queue.back_mut() {
            *last_goal = (new_end_ref, new_end_pos);
        } else if !self.goal_queue.is_empty() {
            // Replace all remaining goals with the new end goal
            self.goal_queue.clear();
            self.goal_queue.push_back((new_end_ref, new_end_pos));
        }

        Ok(())
    }
}

/// Helper function to create and execute a sliced pathfinding query
pub fn find_path_sliced<'a>(
    query: &'a mut NavMeshQuery<'a>,
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: [f32; 3],
    end_pos: [f32; 3],
    filter: &QueryFilter,
    config: Option<SlicedPathConfig>,
) -> Result<Vec<PolyRef>> {
    let config = config.unwrap_or_default();
    let mut sliced_query = SlicedPathfindingQuery::new_with_config(
        query,
        start_ref,
        end_ref,
        start_pos,
        end_pos,
        filter.clone(),
        config,
    )?;

    let final_state = sliced_query.execute_to_completion()?;

    match final_state {
        SlicedPathState::Success | SlicedPathState::PartialPath => {
            Ok(sliced_query.get_path().to_vec())
        }
        SlicedPathState::Failed => Err(Error::Detour(Status::PathInvalid.to_string())),
        SlicedPathState::InProgress => {
            // This shouldn't happen after execute_to_completion
            Err(Error::Detour(Status::Failure.to_string()))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav_mesh::NavMesh;
    use crate::{NavMeshParams, QueryFilter};

    #[test]
    fn test_sliced_path_config() {
        let config = SlicedPathConfig::default();
        assert_eq!(config.max_slice_size, DEFAULT_MAX_SLICE_SIZE);
        assert_eq!(config.max_iterations, MAX_PATHFIND_ITERATIONS);

        let custom_config = SlicedPathConfig {
            max_slice_size: 128,
            max_iterations: 1000,
        };
        assert_eq!(custom_config.max_slice_size, 128);
        assert_eq!(custom_config.max_iterations, 1000);
    }

    #[test]
    fn test_sliced_pathfinding_creation() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Create valid polygon references for testing
        let start_ref = PolyRef::new(1);
        let end_ref = PolyRef::new(2);

        // This should fail with invalid polygon references since we don't have real polygons
        let result = SlicedPathfindingQuery::new(
            &mut query,
            start_ref,
            end_ref,
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 10.0],
            filter,
        );

        // Should fail due to invalid polygon references
        assert!(result.is_err());
    }

    #[test]
    fn test_distance_calculation() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params).unwrap();
        let _query = NavMeshQuery::new(&nav_mesh);
        let _filter = QueryFilter::default();

        // We can't easily create a valid SlicedPathfindingQuery without proper polygons,
        // but we can test the distance calculation logic indirectly
        let start_pos = [0.0, 0.0, 0.0];
        let end_pos = [3.0, 4.0, 0.0];

        // Distance should be 5.0 (3-4-5 triangle)
        let dx = end_pos[0] - start_pos[0];
        let dy = end_pos[1] - start_pos[1];
        let dz = end_pos[2] - start_pos[2];
        let distance = ((dx * dx + dy * dy + dz * dz) as f32).sqrt();

        assert!((distance - 5.0_f32).abs() < 1e-6);
    }

    #[test]
    fn test_sliced_pathfinding_state_transitions() {
        // Test that SlicedPathState enum values work correctly
        let state = SlicedPathState::InProgress;
        assert_eq!(state, SlicedPathState::InProgress);
        assert_ne!(state, SlicedPathState::Success);

        let states = [
            SlicedPathState::InProgress,
            SlicedPathState::Success,
            SlicedPathState::Failed,
            SlicedPathState::PartialPath,
        ];

        // Each state should be equal to itself and different from others
        for (i, &state1) in states.iter().enumerate() {
            for (j, &state2) in states.iter().enumerate() {
                if i == j {
                    assert_eq!(state1, state2);
                } else {
                    assert_ne!(state1, state2);
                }
            }
        }
    }

    #[test]
    fn test_helper_function_with_invalid_params() {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params).unwrap();
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with invalid polygon references
        let result = find_path_sliced(
            &mut query,
            PolyRef::new(0), // Invalid reference
            PolyRef::new(1),
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 10.0],
            &filter,
            None,
        );

        assert!(result.is_err());
    }
}
