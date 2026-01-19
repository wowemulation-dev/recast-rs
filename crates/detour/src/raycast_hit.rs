//! Raycast hit information for Detour
//!
//! This module provides the RaycastHit structure that matches the C++ dtRaycastHit

use super::PolyRef;

/// Options for raycast behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RaycastOptions {
    /// Whether to include the cost of the path
    pub include_cost: bool,
    /// Whether to include the visited polygons in the result
    pub include_path: bool,
}

impl Default for RaycastOptions {
    fn default() -> Self {
        Self {
            include_cost: false,
            include_path: true,
        }
    }
}

/// Provides information about raycast hit
/// Matches C++ dtRaycastHit structure
#[derive(Debug, Clone)]
pub struct RaycastHit {
    /// The hit parameter (distance along ray as fraction of max_dist)
    /// Set to f32::MAX if no wall hit
    pub t: f32,

    /// The normal of the nearest wall hit [(x, y, z)]
    pub hit_normal: [f32; 3],

    /// The index of the edge on the final polygon where the wall was hit
    pub hit_edge_index: i32,

    /// The visited polygon path (optional)
    pub path: Option<Vec<PolyRef>>,

    /// The cost of the path until hit (optional)
    pub path_cost: Option<f32>,
}

impl RaycastHit {
    /// Creates a new RaycastHit with no hit (t = f32::MAX)
    pub fn no_hit() -> Self {
        Self {
            t: f32::MAX,
            hit_normal: [0.0, 0.0, 0.0],
            hit_edge_index: -1,
            path: None,
            path_cost: None,
        }
    }

    /// Creates a new RaycastHit with a wall hit
    pub fn wall_hit(t: f32, normal: [f32; 3], edge_index: i32) -> Self {
        Self {
            t,
            hit_normal: normal,
            hit_edge_index: edge_index,
            path: None,
            path_cost: None,
        }
    }

    /// Checks if this represents a wall hit
    pub fn hit_wall(&self) -> bool {
        self.t < f32::MAX
    }

    /// Sets the path information
    pub fn with_path(mut self, path: Vec<PolyRef>) -> Self {
        self.path = Some(path);
        self
    }

    /// Sets the path cost
    pub fn with_cost(mut self, cost: f32) -> Self {
        self.path_cost = Some(cost);
        self
    }

    /// Gets the path count
    pub fn path_count(&self) -> usize {
        self.path.as_ref().map(|p| p.len()).unwrap_or(0)
    }
}

/// Result of a raycast query
#[derive(Debug, Clone)]
pub struct RaycastResult {
    /// The polygon reference where the ray ends
    pub end_ref: PolyRef,

    /// The position where the ray ends
    pub end_pos: [f32; 3],

    /// Hit information
    pub hit: RaycastHit,
}

impl RaycastResult {
    /// Creates a new raycast result
    pub fn new(end_ref: PolyRef, end_pos: [f32; 3], hit: RaycastHit) -> Self {
        Self {
            end_ref,
            end_pos,
            hit,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raycast_hit_no_hit() {
        let hit = RaycastHit::no_hit();
        assert_eq!(hit.t, f32::MAX);
        assert!(!hit.hit_wall());
        assert_eq!(hit.path_count(), 0);
    }

    #[test]
    fn test_raycast_hit_wall() {
        let hit = RaycastHit::wall_hit(0.5, [0.0, 1.0, 0.0], 2);
        assert_eq!(hit.t, 0.5);
        assert!(hit.hit_wall());
        assert_eq!(hit.hit_edge_index, 2);
        assert_eq!(hit.hit_normal, [0.0, 1.0, 0.0]);
    }

    #[test]
    fn test_raycast_hit_with_path() {
        let path = vec![PolyRef::new(1), PolyRef::new(2), PolyRef::new(3)];
        let hit = RaycastHit::wall_hit(0.7, [1.0, 0.0, 0.0], 1)
            .with_path(path.clone())
            .with_cost(15.5);

        assert_eq!(hit.path_count(), 3);
        assert_eq!(hit.path, Some(path));
        assert_eq!(hit.path_cost, Some(15.5));
    }
}
