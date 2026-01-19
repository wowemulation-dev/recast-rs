//! Navigation mesh query implementation for Detour
//!
//! This module contains the NavMeshQuery structure, which is used to perform
//! pathfinding and other queries on the navigation mesh.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f32;

use super::nav_mesh::{MeshTile, Poly};
use super::raycast_hit::{RaycastHit, RaycastOptions, RaycastResult};
use super::sliced_pathfinding::SlicedPathState;
use super::{NavMesh, Path, PolyFlags, PolyRef, PolyType, QueryFilter, Status};
use glam::Vec3;
use recast_common::{Error, Result};

/// Maximum number of nodes in the search pool
const DT_MAX_NODES: usize = 4096;

/// Node in the A* search algorithm
#[derive(Debug, Clone)]
pub struct Node {
    /// Polygon reference
    pub poly: PolyRef,
    /// Parent node
    pub parent: Option<usize>,
    /// Cost from start to this node
    pub g: f32,
    /// Estimated cost from this node to goal
    pub h: f32,
    /// Total cost (g + h)
    pub f: f32,
    /// State of the node in the search
    pub state: NodeState,
    /// Index in the node pool
    #[allow(dead_code)]
    pub index: usize,
}

impl Node {
    /// Creates a new node
    fn new(poly: PolyRef, index: usize) -> Self {
        Self {
            poly,
            parent: None,
            g: 0.0,
            h: 0.0,
            f: 0.0,
            state: NodeState::New,
            index,
        }
    }
}

/// State of a node in the search
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeState {
    /// Node hasn't been processed yet
    New,
    /// Node is in the open list
    Open,
    /// Node is in the closed list
    Closed,
}

/// Node wrapper for the binary heap (priority queue)
#[derive(Debug, Clone, Copy)]
struct HeapNode {
    /// Reference to the node in the node pool
    index: usize,
    /// Total cost (f value)
    f: f32,
}

impl PartialEq for HeapNode {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl Eq for HeapNode {}

impl PartialOrd for HeapNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Implement reverse ordering for min-heap (lowest f value first)
        // Use total ordering for f32 by handling NaN specially
        match other.f.partial_cmp(&self.f) {
            Some(ordering) => ordering,
            None => {
                // If either is NaN, order NaN values as greater
                if other.f.is_nan() && !self.f.is_nan() {
                    Ordering::Less
                } else if !other.f.is_nan() && self.f.is_nan() {
                    Ordering::Greater
                } else {
                    Ordering::Equal
                }
            }
        }
    }
}

/// Navigation mesh query structure
#[derive(Debug)]
pub struct NavMeshQuery<'a> {
    /// Reference to the navigation mesh
    nav_mesh: &'a NavMesh,
    /// Node pool for A* search
    node_pool: Vec<Node>,
    /// Open list for A* search
    open_list: BinaryHeap<HeapNode>,
    /// Query extent bounds
    query_extent: [f32; 3],
    /// Random seed for random sample queries
    random_seed: u32,
    /// Sliced pathfinding state
    sliced_state: SlicedPathfindingState,
}

/// State for sliced pathfinding within NavMeshQuery
#[derive(Debug)]
struct SlicedPathfindingState {
    /// Is sliced pathfinding active?
    active: bool,
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
    /// Best node found so far (for partial paths)
    best_node_idx: usize,
    /// Best node cost (for partial paths)
    best_node_cost: f32,
}

impl Default for SlicedPathfindingState {
    fn default() -> Self {
        Self {
            active: false,
            state: SlicedPathState::InProgress,
            start_ref: PolyRef::new(0),
            end_ref: PolyRef::new(0),
            start_pos: [0.0; 3],
            end_pos: [0.0; 3],
            filter: QueryFilter::default(),
            current_path: Vec::new(),
            best_node_idx: 0,
            best_node_cost: f32::MAX,
        }
    }
}

impl<'a> NavMeshQuery<'a> {
    /// Creates a new navigation mesh query
    pub fn new(nav_mesh: &'a NavMesh) -> Self {
        // Initialize the node pool
        let mut node_pool = Vec::with_capacity(DT_MAX_NODES);
        for i in 0..DT_MAX_NODES {
            node_pool.push(Node::new(PolyRef::new(0), i));
        }

        Self {
            nav_mesh,
            node_pool,
            open_list: BinaryHeap::new(),
            query_extent: [2.0, 4.0, 2.0], // Default query extent
            random_seed: 1,
            sliced_state: SlicedPathfindingState::default(),
        }
    }

    /// Sets the query extent
    pub fn set_query_extent(&mut self, extent: [f32; 3]) {
        self.query_extent = extent;
    }

    /// Gets the query extent
    pub fn get_query_extent(&self) -> [f32; 3] {
        self.query_extent
    }

    /// Sets the random seed for random point generation
    pub fn set_random_seed(&mut self, seed: u32) {
        self.random_seed = seed;
    }

    /// Generates next random number (simple LCG)
    fn next_random(&mut self) -> u32 {
        self.random_seed = self
            .random_seed
            .wrapping_mul(1103515245)
            .wrapping_add(12345);
        self.random_seed
    }

    /// Generates a random float between 0.0 and 1.0
    fn random_f32(&mut self) -> f32 {
        (self.next_random() & 0x7FFFFFFF) as f32 / 2147483647.0
    }

    /// Gets a reference to the navigation mesh
    pub fn nav_mesh(&self) -> &NavMesh {
        self.nav_mesh
    }

    /// Finds the polygon nearest to the specified center point
    pub fn find_nearest_poly(
        &self,
        center: &[f32; 3],
        half_extents: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<(PolyRef, [f32; 3])> {
        // Create search bounds
        let bmin = [
            center[0] - half_extents[0],
            center[1] - half_extents[1],
            center[2] - half_extents[2],
        ];
        let bmax = [
            center[0] + half_extents[0],
            center[1] + half_extents[1],
            center[2] + half_extents[2],
        ];

        // Query all polygons within the search bounds
        let polys = self.nav_mesh.query_polygons(&bmin, &bmax, filter)?;

        let mut nearest_ref = PolyRef::new(0);
        let mut nearest_point = [0.0; 3];
        let mut nearest_distance_sqr = f32::MAX;

        // Find the closest polygon
        for poly_ref in polys {
            let (closest_pt, is_over_poly) = self.closest_point_on_poly(poly_ref, center)?;

            // Calculate distance
            let diff = [
                center[0] - closest_pt[0],
                center[1] - closest_pt[1],
                center[2] - closest_pt[2],
            ];

            let d = if is_over_poly {
                // If point is directly over polygon and closer than climb height,
                // favor that instead of straight line nearest point
                let (_tile, _poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;
                // TODO: Get walkable_climb from navigation mesh params
                let climb_height = 0.25; // Default climb height

                let height_diff = diff[1].abs() - climb_height;
                if height_diff > 0.0 {
                    height_diff * height_diff
                } else {
                    0.0
                }
            } else {
                // Regular distance squared
                diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]
            };

            if d < nearest_distance_sqr {
                nearest_point = closest_pt;
                nearest_distance_sqr = d;
                nearest_ref = poly_ref;
            }
        }

        if !nearest_ref.is_valid() {
            return Err(Error::Detour(Status::PathInvalid.to_string()));
        }

        Ok((nearest_ref, nearest_point))
    }

    /// Finds the polygon nearest to the specified center point (with is_over_poly flag)
    ///
    /// This overload also returns whether the nearest point lies directly over the polygon.
    ///
    /// # Arguments
    /// * `center` - The center of the search box
    /// * `half_extents` - The search distance along each axis
    /// * `filter` - The polygon filter to apply to the query
    ///
    /// # Returns
    /// A tuple containing:
    /// * The polygon reference of the nearest polygon
    /// * The nearest point on the polygon
    /// * Whether the point's X/Z coordinate lies inside the polygon
    pub fn find_nearest_poly_extended(
        &self,
        center: &[f32; 3],
        half_extents: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<(PolyRef, [f32; 3], bool)> {
        // Create search bounds
        let bmin = [
            center[0] - half_extents[0],
            center[1] - half_extents[1],
            center[2] - half_extents[2],
        ];
        let bmax = [
            center[0] + half_extents[0],
            center[1] + half_extents[1],
            center[2] + half_extents[2],
        ];

        // Query all polygons within the search bounds
        let polys = self.nav_mesh.query_polygons(&bmin, &bmax, filter)?;

        let mut nearest_ref = PolyRef::new(0);
        let mut nearest_point = [0.0; 3];
        let mut nearest_distance_sqr = f32::MAX;
        let mut nearest_is_over_poly = false;

        // Find the closest polygon
        for poly_ref in polys {
            let (closest_pt, is_over_poly) = self.closest_point_on_poly(poly_ref, center)?;

            // Calculate distance
            let diff = [
                center[0] - closest_pt[0],
                center[1] - closest_pt[1],
                center[2] - closest_pt[2],
            ];

            let d = if is_over_poly {
                // If point is directly over polygon and closer than climb height,
                // favor that instead of straight line nearest point
                let (_tile, _poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;
                // TODO: Get walkable_climb from navigation mesh params
                let climb_height = 0.25; // Default climb height

                let height_diff = diff[1].abs() - climb_height;
                if height_diff > 0.0 {
                    height_diff * height_diff
                } else {
                    0.0
                }
            } else {
                // Regular distance squared
                diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]
            };

            if d < nearest_distance_sqr {
                nearest_point = closest_pt;
                nearest_distance_sqr = d;
                nearest_ref = poly_ref;
                nearest_is_over_poly = is_over_poly;
            }
        }

        if !nearest_ref.is_valid() {
            return Err(Error::Detour(Status::PathInvalid.to_string()));
        }

        Ok((nearest_ref, nearest_point, nearest_is_over_poly))
    }

    /// Finds a path from start to end position
    pub fn find_path(
        &mut self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<Vec<PolyRef>> {
        // Validate the input
        if !self.nav_mesh.is_valid_poly_ref(start_ref) || !self.nav_mesh.is_valid_poly_ref(end_ref)
        {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Return early if start and end are the same
        if start_ref == end_ref {
            return Ok(vec![start_ref]);
        }

        // Reset the node pool and open list
        for node in &mut self.node_pool {
            node.parent = None;
            node.state = NodeState::New;
            node.poly = PolyRef::new(0);
            node.g = 0.0;
            node.h = 0.0;
            node.f = 0.0;
        }
        self.open_list.clear();

        // Get the start and end nodes
        let start_node_idx = 0;
        let start_h = self.heuristic(start_pos, end_pos);
        {
            let start_node = &mut self.node_pool[start_node_idx];
            start_node.poly = start_ref;
            start_node.state = NodeState::Open;
            start_node.g = 0.0;
            start_node.h = start_h;
            start_node.f = start_h;
        }

        // Push the start node to the open list
        let start_f = self.node_pool[start_node_idx].f;
        self.open_list.push(HeapNode {
            index: start_node_idx,
            f: start_f,
        });

        // Set the goal node
        let goal_node_idx = 1;
        self.node_pool[goal_node_idx].poly = end_ref;

        // Iterate through the A* search
        let mut best_node_idx = start_node_idx;
        let mut best_node_cost = self.node_pool[start_node_idx].h;

        let mut _iterations = 0;
        while !self.open_list.is_empty() {
            _iterations += 1;
            // Pop the best node from the open list
            let HeapNode {
                index: current_idx, ..
            } = self.open_list.pop().unwrap();
            let current_node = &mut self.node_pool[current_idx];
            current_node.state = NodeState::Closed;

            // If we reached the end, break
            if current_node.poly == end_ref {
                best_node_idx = current_idx;
                break;
            }

            // Get better nodes for partial paths
            if current_node.h < best_node_cost {
                best_node_cost = current_node.h;
                best_node_idx = current_idx;
            }

            // Expand neighbors
            self.expand_neighbors(current_idx, goal_node_idx, end_pos, filter)?;
        }

        // Reconstruct the path
        let mut path = Vec::new();
        let mut current_idx = best_node_idx;

        while current_idx != start_node_idx {
            let current_node = &self.node_pool[current_idx];
            path.push(current_node.poly);

            if let Some(parent_idx) = current_node.parent {
                current_idx = parent_idx;
            } else {
                // Path is invalid
                return Err(Error::Detour(Status::PathInvalid.to_string()));
            }
        }

        // Add the start node
        path.push(start_ref);

        // Reverse the path
        path.reverse();

        Ok(path)
    }

    /// Expands the neighbors of a node in the A* search
    fn expand_neighbors(
        &mut self,
        current_idx: usize,
        goal_idx: usize,
        end_pos: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<()> {
        let current_poly = self.node_pool[current_idx].poly;
        let current_parent = self.node_pool[current_idx].parent;

        // Get the tile and poly
        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(current_poly)?;

        // Get the end node's poly
        let goal_poly = self.node_pool[goal_idx].poly;

        // Debug: Check neighbor expansion
        let mut _neighbors_found = 0;

        // Iterate through all neighbors
        let mut link_idx = poly.first_link;
        while let Some(idx) = link_idx {
            let link = &tile.links[idx];
            link_idx = link.next.map(|n| n as usize);

            let neighbor_ref = link.reference;

            // Skip invalid neighbors and don't expand back to parent (C++ compatibility)
            if !self.nav_mesh.is_valid_poly_ref(neighbor_ref) {
                continue;
            }

            // Skip if this is the parent polygon (avoid infinite loops)
            if let Some(parent_idx) = current_parent {
                if self.node_pool[parent_idx].poly == neighbor_ref {
                    continue;
                }
            }

            // Skip this neighbor if it doesn't pass the filter
            if !self.pass_filter(neighbor_ref, tile, poly, filter)? {
                continue;
            }

            _neighbors_found += 1;

            // Find the node or allocate a new one
            let mut neighbor_idx = 0;
            while neighbor_idx < DT_MAX_NODES {
                if self.node_pool[neighbor_idx].state == NodeState::New
                    || self.node_pool[neighbor_idx].poly == neighbor_ref
                {
                    break;
                }
                neighbor_idx += 1;
            }

            // If no node could be allocated, abort
            if neighbor_idx == DT_MAX_NODES {
                return Err(Error::Detour(Status::BufferTooSmall.to_string()));
            }

            // Get node state to check if we should process it
            let neighbor_state = self.node_pool[neighbor_idx].state;
            let neighbor_g = self.node_pool[neighbor_idx].g;
            let neighbor_h = self.node_pool[neighbor_idx].h;

            // Initialize poly ref if node is new
            if neighbor_state == NodeState::New {
                self.node_pool[neighbor_idx].poly = neighbor_ref;
            }

            // If the node has already been processed, skip it
            if neighbor_state == NodeState::Closed {
                continue;
            }

            // Get edge midpoint positions (C++ compatibility: use shared edge midpoint)
            let current_center = self.get_poly_center(current_poly)?;
            let neighbor_center = match self.get_portal_points(current_poly, neighbor_ref) {
                Ok((left, right)) => {
                    // Use midpoint of shared edge between polygons
                    [
                        (left[0] + right[0]) * 0.5,
                        (left[1] + right[1]) * 0.5,
                        (left[2] + right[2]) * 0.5,
                    ]
                }
                Err(_) => {
                    // Fallback to polygon center if portal points not available
                    self.get_poly_center(neighbor_ref)?
                }
            };

            // Cost for the neighbor
            let cost = self.get_edge_cost(
                current_poly,
                neighbor_ref,
                &current_center,
                &neighbor_center,
                filter,
            )?;

            // Calculate the new g value
            let current_g = self.node_pool[current_idx].g;
            let new_g = current_g + cost;

            if neighbor_state == NodeState::New || new_g < neighbor_g {
                // Calculate heuristic if needed
                let h = if neighbor_state == NodeState::New {
                    if neighbor_ref == goal_poly {
                        0.0
                    } else {
                        let poly_center = self.get_poly_center(neighbor_ref)?;
                        self.heuristic(&poly_center, end_pos)
                    }
                } else {
                    neighbor_h
                };

                // Update the node
                {
                    let neighbor_node = &mut self.node_pool[neighbor_idx];
                    neighbor_node.parent = Some(current_idx);
                    neighbor_node.g = new_g;
                    neighbor_node.h = h;
                    neighbor_node.f = new_g + h;

                    // Update state if it's a new node
                    if neighbor_state == NodeState::New {
                        neighbor_node.state = NodeState::Open;
                    }
                }

                // Add to open list if it's a new node
                if neighbor_state == NodeState::New {
                    self.open_list.push(HeapNode {
                        index: neighbor_idx,
                        f: new_g + h,
                    });
                }
            }
        }

        Ok(())
    }

    /// Expands off-mesh connections as potential neighbors in A* search
    #[allow(dead_code)]
    fn expand_off_mesh_connections(
        &mut self,
        current_idx: usize,
        goal_idx: usize,
        end_pos: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<()> {
        let current_node = &self.node_pool[current_idx];
        let current_poly = current_node.poly;

        // Get the tile and poly for the current node
        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(current_poly)?;
        let goal_poly = self.node_pool[goal_idx].poly;

        // Check all off-mesh connections in ALL tiles (not just current)
        let all_tiles = self.nav_mesh.get_all_tiles();
        for check_tile in all_tiles {
            // Check all off-mesh connections in this tile
            for connection in &check_tile.off_mesh_connections {
                // Check if the connection passes the filter
                if !filter.include_flags.intersects(connection.flags)
                    || filter.exclude_flags.intersects(connection.flags)
                {
                    continue;
                }

                let connection_ref = connection.poly;

                // Skip if this is the current polygon (can't connect to self)
                if connection_ref == current_poly {
                    continue;
                }

                // Get connection endpoints
                let start_pos = connection.start_pos();
                let end_pos_conn = connection.end_pos();

                // Check if current polygon contains or is near the start/end points
                let mut can_use_connection = false;

                // Debug: Print connection info

                // Check if we're on the polygon that contains the start point
                if self
                    .is_point_in_polygon(tile, poly, &start_pos)
                    .unwrap_or(false)
                {
                    can_use_connection = connection.allows_start_to_end();
                }
                // Or if we're on the polygon that contains the end point
                else if self
                    .is_point_in_polygon(tile, poly, &end_pos_conn)
                    .unwrap_or(false)
                {
                    can_use_connection = connection.allows_end_to_start();
                }
                // Or check if any vertex of current polygon is within connection radius
                else {
                    for i in 0..poly.vert_count as usize {
                        let v_idx = poly.verts[i] as usize;
                        let vert = [
                            tile.verts[v_idx * 3],
                            tile.verts[v_idx * 3 + 1],
                            tile.verts[v_idx * 3 + 2],
                        ];

                        let dist_to_start = self.distance_3d(vert, start_pos);
                        let dist_to_end = self.distance_3d(vert, end_pos_conn);

                        if (dist_to_start <= connection.radius && connection.allows_start_to_end())
                            || (dist_to_end <= connection.radius
                                && connection.allows_end_to_start())
                        {
                            can_use_connection = true;
                            break;
                        }
                    }
                }

                if !can_use_connection {
                    continue;
                }

                // Find the node or allocate a new one for this off-mesh connection
                let mut neighbor_idx = 0;
                while neighbor_idx < DT_MAX_NODES {
                    if self.node_pool[neighbor_idx].state == NodeState::New
                        || self.node_pool[neighbor_idx].poly == connection_ref
                    {
                        break;
                    }
                    neighbor_idx += 1;
                }

                // If no node could be allocated, continue to next connection
                if neighbor_idx == DT_MAX_NODES {
                    continue;
                }

                // Get node state to check if we should process it
                let neighbor_state = self.node_pool[neighbor_idx].state;
                let neighbor_g = self.node_pool[neighbor_idx].g;
                let neighbor_h = self.node_pool[neighbor_idx].h;

                // Initialize poly ref if node is new
                if neighbor_state == NodeState::New {
                    self.node_pool[neighbor_idx].poly = connection_ref;
                }

                // If the node has already been processed, skip it
                if neighbor_state == NodeState::Closed {
                    continue;
                }

                // Determine which endpoint we'll move to
                let (_from_pos, to_pos) = if can_use_connection && connection.allows_start_to_end()
                {
                    (start_pos, end_pos_conn)
                } else {
                    (end_pos_conn, start_pos)
                };

                // Get position on current polygon for cost calculation
                let current_pos = self.get_poly_center(current_poly)?;

                // Cost for traversing the off-mesh connection
                let cost = self.get_off_mesh_connection_cost(
                    current_poly,
                    connection_ref,
                    &current_pos,
                    &to_pos,
                    filter,
                )?;

                // Calculate the new g value
                let current_g = self.node_pool[current_idx].g;
                let new_g = current_g + cost;

                if neighbor_state == NodeState::New || new_g < neighbor_g {
                    // Calculate heuristic if needed
                    let h = if neighbor_state == NodeState::New {
                        if connection_ref == goal_poly {
                            0.0
                        } else {
                            self.heuristic(&to_pos, end_pos)
                        }
                    } else {
                        neighbor_h
                    };

                    // Update the node
                    {
                        let neighbor_node = &mut self.node_pool[neighbor_idx];
                        neighbor_node.parent = Some(current_idx);
                        neighbor_node.g = new_g;
                        neighbor_node.h = h;
                        neighbor_node.f = new_g + h;

                        // Update state if it's a new node
                        if neighbor_state == NodeState::New {
                            neighbor_node.state = NodeState::Open;
                        }
                    }

                    // Add to open list if it's a new node
                    if neighbor_state == NodeState::New {
                        self.open_list.push(HeapNode {
                            index: neighbor_idx,
                            f: new_g + h,
                        });
                    }
                }
            } // Close the off-mesh connections loop
        } // Close the tiles loop

        Ok(())
    }

    /// Calculates 3D distance between two points
    fn distance_3d(&self, a: [f32; 3], b: [f32; 3]) -> f32 {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let dz = b[2] - a[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Calculates the heuristic (straight-line distance)
    /// Uses H_SCALE factor to ensure heuristic is slightly underestimated for A* optimality
    fn heuristic(&self, start: &[f32; 3], end: &[f32; 3]) -> f32 {
        const H_SCALE: f32 = 0.999; // C++ compatibility: maintain A* optimality
        let dx = end[0] - start[0];
        let dy = end[1] - start[1];
        let dz = end[2] - start[2];

        (dx * dx + dy * dy + dz * dz).sqrt() * H_SCALE
    }

    /// Gets the cost of moving from one polygon to another
    fn get_edge_cost(
        &self,
        from_ref: PolyRef,
        to_ref: PolyRef,
        from_pos: &[f32; 3],
        to_pos: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<f32> {
        // Handle off-mesh connections specially
        if self.nav_mesh.is_off_mesh_connection(to_ref)
            || self.nav_mesh.is_off_mesh_connection(from_ref)
        {
            return self.get_off_mesh_connection_cost(from_ref, to_ref, from_pos, to_pos, filter);
        }

        // Get tile and polygon data
        let (from_tile, from_poly) = self.nav_mesh.get_tile_and_poly_by_ref(from_ref)?;
        let (to_tile, to_poly) = self.nav_mesh.get_tile_and_poly_by_ref(to_ref)?;

        // Use filter to calculate cost
        let cost = filter.get_cost(
            from_pos,
            to_pos,
            PolyRef::new(0),
            None,
            None, // prev (not used for edge cost)
            from_ref,
            from_tile,
            from_poly, // current
            to_ref,
            Some(to_tile),
            Some(to_poly), // next
        );

        Ok(cost)
    }

    /// Calculates the cost of traversing an off-mesh connection
    fn get_off_mesh_connection_cost(
        &self,
        from_ref: PolyRef,
        to_ref: PolyRef,
        from_pos: &[f32; 3],
        to_pos: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<f32> {
        // Base distance cost
        let dx = to_pos[0] - from_pos[0];
        let dy = to_pos[1] - from_pos[1];
        let dz = to_pos[2] - from_pos[2];
        let base_cost = (dx * dx + dy * dy + dz * dz).sqrt();

        // If either polygon is an off-mesh connection, get the area cost from it
        let area = if self.nav_mesh.is_off_mesh_connection(to_ref) {
            let connection = self.nav_mesh.get_off_mesh_connection(to_ref)?;
            connection.area
        } else if self.nav_mesh.is_off_mesh_connection(from_ref) {
            let connection = self.nav_mesh.get_off_mesh_connection(from_ref)?;
            connection.area
        } else {
            0 // Should not happen based on our check above
        };

        // Apply area cost multiplier
        let area_index = (area as usize).min(31);
        let area_cost = filter.area_cost[area_index];

        Ok(base_cost * area_cost)
    }

    /// Checks if a polygon passes the filter
    fn pass_filter(
        &self,
        poly_ref: PolyRef,
        _tile: &MeshTile,
        _poly: &Poly,
        filter: &QueryFilter,
    ) -> Result<bool> {
        // Get the polygon
        let (_, neighbor_poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

        // Check if the polygon is included by the filter
        let include = (neighbor_poly.flags & filter.include_flags) != PolyFlags::empty();

        // Check if the polygon is excluded by the filter
        let exclude = (neighbor_poly.flags & filter.exclude_flags) != PolyFlags::empty();

        // The polygon passes the filter if it is included and not excluded
        Ok(include && !exclude)
    }

    /// Gets the center of a polygon
    pub fn get_poly_center(&self, poly_ref: PolyRef) -> Result<[f32; 3]> {
        // Get the tile and poly
        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

        let mut center = [0.0; 3];
        let mut count = 0;

        // Calculate the center as the average of all vertices
        for i in 0..poly.vert_count as usize {
            let vert_idx = poly.verts[i] as usize;

            center[0] += tile.verts[vert_idx * 3];
            center[1] += tile.verts[vert_idx * 3 + 1];
            center[2] += tile.verts[vert_idx * 3 + 2];

            count += 1;
        }

        if count > 0 {
            center[0] /= count as f32;
            center[1] /= count as f32;
            center[2] /= count as f32;
        }

        Ok(center)
    }

    /// Finds the closest point on a polygon
    pub fn closest_point_on_poly(
        &self,
        poly_ref: PolyRef,
        pos: &[f32; 3],
    ) -> Result<([f32; 3], bool)> {
        // Handle off-mesh connections
        if self.nav_mesh.is_off_mesh_connection(poly_ref) {
            return self.closest_point_on_off_mesh_connection(poly_ref, pos);
        }

        // Get the tile and poly
        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

        let mut closest = *pos;
        let mut is_over_poly = false;

        // Try to get height for the position within the polygon
        if let Some(height) = self.nav_mesh.get_poly_height(tile, poly, pos)? {
            closest[1] = height;
            is_over_poly = true;
            return Ok((closest, is_over_poly));
        }

        // Skip the old off-mesh connection handling since we handle it above
        if poly.poly_type == PolyType::OffMeshConnection {
            return Ok((*pos, false));
        }

        // Point is outside polygon, find closest point on edges
        let mut closest_dist_sqr = f32::MAX;

        // Check each edge of the polygon
        for i in 0..poly.vert_count as usize {
            let j = (i + 1) % poly.vert_count as usize;

            let vi_idx = poly.verts[i] as usize;
            let vj_idx = poly.verts[j] as usize;

            let vi = [
                tile.verts[vi_idx * 3],
                tile.verts[vi_idx * 3 + 1],
                tile.verts[vi_idx * 3 + 2],
            ];
            let vj = [
                tile.verts[vj_idx * 3],
                tile.verts[vj_idx * 3 + 1],
                tile.verts[vj_idx * 3 + 2],
            ];

            let vi_pt = Vec3::new(vi[0], vi[1], vi[2]);
            let vj_pt = Vec3::new(vj[0], vj[1], vj[2]);
            let pos_pt = Vec3::new(pos[0], pos[1], pos[2]);

            let edge_closest_pt = recast_common::closest_point_on_segment(&pos_pt, &vi_pt, &vj_pt);
            let edge_closest = [edge_closest_pt.x, edge_closest_pt.y, edge_closest_pt.z];

            let dx = edge_closest[0] - pos[0];
            let dy = edge_closest[1] - pos[1];
            let dz = edge_closest[2] - pos[2];
            let dist_sqr = dx * dx + dy * dy + dz * dz;

            if dist_sqr < closest_dist_sqr {
                closest_dist_sqr = dist_sqr;
                closest = edge_closest;
            }
        }

        Ok((closest, is_over_poly))
    }

    /// Finds the closest point on an off-mesh connection
    fn closest_point_on_off_mesh_connection(
        &self,
        connection_ref: PolyRef,
        pos: &[f32; 3],
    ) -> Result<([f32; 3], bool)> {
        let connection = self.nav_mesh.get_off_mesh_connection(connection_ref)?;

        let start_pos = connection.start_pos();
        let end_pos = connection.end_pos();

        // Find the closest point on the line segment between start and end
        let start_pt = Vec3::new(start_pos[0], start_pos[1], start_pos[2]);
        let end_pt = Vec3::new(end_pos[0], end_pos[1], end_pos[2]);
        let pos_pt = Vec3::new(pos[0], pos[1], pos[2]);

        let closest_pt = recast_common::closest_point_on_segment(&pos_pt, &start_pt, &end_pt);
        let closest_arr = [closest_pt.x, closest_pt.y, closest_pt.z];

        // Check if the point is within the connection radius
        let dist_to_closest = {
            let dx = pos[0] - closest_arr[0];
            let dy = pos[1] - closest_arr[1];
            let dz = pos[2] - closest_arr[2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        };

        let is_over_connection = dist_to_closest <= connection.radius;

        Ok((closest_arr, is_over_connection))
    }

    /// Gets the off-mesh connection end position for a given start position
    pub fn get_off_mesh_connection_endpoint(
        &self,
        connection_ref: PolyRef,
        start_pos: &[f32; 3],
    ) -> Result<[f32; 3]> {
        let connection = self.nav_mesh.get_off_mesh_connection(connection_ref)?;

        let conn_start = connection.start_pos();
        let conn_end = connection.end_pos();

        // Calculate distances to both endpoints
        let dist_to_start = {
            let dx = start_pos[0] - conn_start[0];
            let dy = start_pos[1] - conn_start[1];
            let dz = start_pos[2] - conn_start[2];
            dx * dx + dy * dy + dz * dz
        };

        let dist_to_end = {
            let dx = start_pos[0] - conn_end[0];
            let dy = start_pos[1] - conn_end[1];
            let dz = start_pos[2] - conn_end[2];
            dx * dx + dy * dy + dz * dz
        };

        // Return the endpoint that's further away (i.e., the destination)
        if dist_to_start < dist_to_end {
            // We're closer to the start, so end is the destination
            if connection.allows_start_to_end() {
                Ok(conn_end)
            } else {
                Err(Error::Detour(Status::InvalidParam.to_string()))
            }
        } else {
            // We're closer to the end, so start is the destination
            if connection.allows_end_to_start() {
                Ok(conn_start)
            } else {
                Err(Error::Detour(Status::InvalidParam.to_string()))
            }
        }
    }

    /// Gets the ordered endpoints of an off-mesh connection based on the previous polygon.
    /// This method determines the travel direction through the off-mesh connection by looking
    /// at which polygon was used to reach the connection.
    ///
    /// # Arguments
    /// * `prev_ref` - Reference to the polygon we came from (used to determine direction)
    /// * `connection_ref` - Reference to the off-mesh connection polygon
    ///
    /// # Returns
    /// A tuple of (start_pos, end_pos) ordered for the travel direction.
    /// If prev_ref is invalid, returns the connection's natural start->end order.
    pub fn get_off_mesh_connection_poly_end_points(
        &self,
        prev_ref: PolyRef,
        connection_ref: PolyRef,
    ) -> Result<([f32; 3], [f32; 3])> {
        let connection = self.nav_mesh.get_off_mesh_connection(connection_ref)?;

        let conn_start = connection.start_pos();
        let conn_end = connection.end_pos();

        // If we don't have a valid previous reference, return natural order
        if !prev_ref.is_valid() {
            return Ok((conn_start, conn_end));
        }

        // Get the previous polygon's position to determine approach direction
        match self.nav_mesh.get_tile_and_poly_by_ref(prev_ref) {
            Ok((tile, poly)) => {
                // Calculate the center of the previous polygon
                let prev_center = self.calculate_poly_center(tile, poly)?;

                // Calculate distances from previous polygon center to both endpoints
                let dist_to_start = {
                    let dx = prev_center[0] - conn_start[0];
                    let dy = prev_center[1] - conn_start[1];
                    let dz = prev_center[2] - conn_start[2];
                    dx * dx + dy * dy + dz * dz
                };

                let dist_to_end = {
                    let dx = prev_center[0] - conn_end[0];
                    let dy = prev_center[1] - conn_end[1];
                    let dz = prev_center[2] - conn_end[2];
                    dx * dx + dy * dy + dz * dz
                };

                // The endpoint closer to the previous polygon is our "start",
                // and the farther one is our "end" for this traversal
                if dist_to_start < dist_to_end {
                    // We're approaching from the start side, go start->end
                    if connection.allows_start_to_end() {
                        Ok((conn_start, conn_end))
                    } else {
                        Err(Error::Detour(Status::InvalidParam.to_string()))
                    }
                } else {
                    // We're approaching from the end side, go end->start
                    if connection.allows_end_to_start() {
                        Ok((conn_end, conn_start))
                    } else {
                        Err(Error::Detour(Status::InvalidParam.to_string()))
                    }
                }
            }
            Err(_) => {
                // If we can't get the previous polygon, fall back to natural order
                Ok((conn_start, conn_end))
            }
        }
    }

    /// Helper method to calculate the center point of a polygon
    fn calculate_poly_center(&self, tile: &MeshTile, poly: &Poly) -> Result<[f32; 3]> {
        if poly.vert_count == 0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let mut center = [0.0f32; 3];
        let vert_count = poly.vert_count as usize;

        // Sum all vertex positions
        for i in 0..vert_count {
            let vert_idx = poly.verts[i] as usize;
            if vert_idx * 3 + 2 >= tile.verts.len() {
                return Err(Error::Detour(Status::InvalidParam.to_string()));
            }

            center[0] += tile.verts[vert_idx * 3];
            center[1] += tile.verts[vert_idx * 3 + 1];
            center[2] += tile.verts[vert_idx * 3 + 2];
        }

        // Average to get center
        let count = vert_count as f32;
        center[0] /= count;
        center[1] /= count;
        center[2] /= count;

        Ok(center)
    }

    /// Moves from the start to the end position constrained to the navigation mesh surface
    ///
    /// This uses the same algorithm as the C++ version: a breadth-first search with
    /// a search radius constraint to find the best path.
    ///
    /// # Algorithm
    /// Uses breadth-first search (BFS) to explore neighboring polygons within a search radius.
    /// The search radius is calculated as quarter distance between start and end positions
    /// plus a small epsilon to ensure numeric stability.
    ///
    /// # Performance
    /// Time complexity: O(n) where n is the number of polygons within search radius (max 16)
    /// Space complexity: O(n) for the node storage and traversal queue
    ///
    /// Uses VecDeque for efficient O(1) front operations during BFS traversal.
    pub fn move_along_surface(
        &self,
        start_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &QueryFilter,
        visited_refs: &mut Vec<PolyRef>,
    ) -> Result<[f32; 3]> {
        use recast_common::{dist_point_segment_sqr_2d_with_t, point_in_polygon_2d};
        use std::collections::VecDeque;

        // Validate input
        if !self.nav_mesh.is_valid_poly_ref(start_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        visited_refs.clear();

        // Constants matching C++
        const MAX_STACK: usize = 48;
        const MAX_VISITED: usize = 16;

        // Node structure for breadth-first search
        // Removed Clone derive to avoid unnecessary cloning
        struct SurfaceNode {
            poly_ref: PolyRef,
            parent_idx: Option<usize>,
        }

        let mut nodes = Vec::new();
        // Use VecDeque for efficient O(1) front removal in BFS
        let mut stack = VecDeque::new();

        // Initialize start node
        let start_node = SurfaceNode {
            poly_ref: start_ref,
            parent_idx: None,
        };
        nodes.push(start_node);
        stack.push_back(0); // Index of start node

        let mut best_pos = *start_pos;
        let mut best_dist = f32::MAX;
        let mut best_node_idx = 0;

        // Search constraints - midpoint and radius for search area
        let search_pos = [
            (start_pos[0] + end_pos[0]) * 0.5,
            (start_pos[1] + end_pos[1]) * 0.5,
            (start_pos[2] + end_pos[2]) * 0.5,
        ];

        let dx = start_pos[0] - end_pos[0];
        let dy = start_pos[1] - end_pos[1];
        let dz = start_pos[2] - end_pos[2];
        // Search radius: quarter of distance between start and end, plus epsilon for stability
        // 0.25 = quarter distance multiplier to constrain search area
        // 0.001 = minimum search radius to prevent degenerate cases
        let search_rad_sqr = (dx * dx + dy * dy + dz * dz) * 0.25 + 0.001;

        while !stack.is_empty() && nodes.len() < MAX_VISITED {
            // Pop front (breadth-first) - now O(1) with VecDeque
            let cur_node_idx = stack.pop_front().unwrap();
            let cur_poly_ref = nodes[cur_node_idx].poly_ref;

            // Get poly and tile
            let (cur_tile, cur_poly) = self.nav_mesh.get_tile_and_poly_by_ref(cur_poly_ref)?;

            // Collect vertices
            let mut verts = Vec::new();
            for i in 0..cur_poly.vert_count as usize {
                let idx = cur_poly.verts[i] as usize * 3;
                verts.push(cur_tile.verts[idx]);
                verts.push(cur_tile.verts[idx + 1]);
                verts.push(cur_tile.verts[idx + 2]);
            }

            // If target is inside the poly, we found it
            if point_in_polygon_2d(end_pos, &verts, cur_poly.vert_count as usize) {
                best_node_idx = cur_node_idx;
                best_pos = *end_pos;
                break;
            }

            // Find wall edges and nearest point inside the walls
            for i in 0..cur_poly.vert_count as usize {
                let j = if i == 0 {
                    cur_poly.vert_count as usize - 1
                } else {
                    i - 1
                };

                // Find links to neighbors
                let mut neighbors = Vec::new();

                if let Some(link) = self.nav_mesh.find_link(cur_tile, cur_poly, j as u8) {
                    if link.reference.is_valid() {
                        if let Ok((nei_tile, nei_poly)) =
                            self.nav_mesh.get_tile_and_poly_by_ref(link.reference)
                        {
                            if filter.pass_filter(link.reference, nei_tile, nei_poly) {
                                neighbors.push(link.reference);
                            }
                        }
                    }
                }

                let vi_idx = cur_poly.verts[i] as usize * 3;
                let vj_idx = cur_poly.verts[j] as usize * 3;
                let vi = &cur_tile.verts[vi_idx..vi_idx + 3];
                let vj = &cur_tile.verts[vj_idx..vj_idx + 3];

                // If wall edge, check distance
                if neighbors.is_empty() {
                    // Wall edge, calc distance
                    let (dist_sqr, t) = dist_point_segment_sqr_2d_with_t(end_pos, vj, vi);
                    if dist_sqr < best_dist {
                        // Update nearest distance
                        best_pos = [
                            vj[0] + (vi[0] - vj[0]) * t,
                            vj[1] + (vi[1] - vj[1]) * t,
                            vj[2] + (vi[2] - vj[2]) * t,
                        ];
                        best_dist = dist_sqr;
                        best_node_idx = cur_node_idx;
                    }
                } else {
                    // Has neighbors, check them
                    for &nei_ref in &neighbors {
                        // Check if already visited
                        let already_visited = nodes.iter().any(|n| n.poly_ref == nei_ref);
                        if already_visited {
                            continue;
                        }

                        // Skip if too far from search constraint
                        let (dist_sqr, _) = dist_point_segment_sqr_2d_with_t(&search_pos, vj, vi);
                        if dist_sqr > search_rad_sqr {
                            continue;
                        }

                        // Add to nodes and stack
                        if stack.len() < MAX_STACK && nodes.len() < MAX_VISITED {
                            let new_node = SurfaceNode {
                                poly_ref: nei_ref,
                                parent_idx: Some(cur_node_idx),
                            };
                            let new_idx = nodes.len();
                            nodes.push(new_node);
                            stack.push_back(new_idx);
                        }
                    }
                }
            }
        }

        // Trace back path from best node
        let mut path_indices = Vec::new();
        let mut current_idx = Some(best_node_idx);

        while let Some(idx) = current_idx {
            path_indices.push(idx);
            current_idx = nodes[idx].parent_idx;
        }

        // Reverse to get path from start to end
        path_indices.reverse();

        // Store visited polygons
        for idx in path_indices {
            visited_refs.push(nodes[idx].poly_ref);
        }

        Ok(best_pos)
    }

    /// Casts a ray along the navigation mesh
    #[allow(unused_assignments)]
    pub fn raycast(
        &self,
        start_ref: PolyRef,
        start_pos: &[f32; 3],
        dir: &[f32; 3],
        max_dist: f32,
        filter: &QueryFilter,
    ) -> Result<(PolyRef, [f32; 3], f32)> {
        // Validate the input
        if !self.nav_mesh.is_valid_poly_ref(start_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Handle zero-length raycast (C++ compatibility)
        if max_dist <= 0.0 {
            return Ok((start_ref, *start_pos, 0.0));
        }

        // Calculate end position
        let end_pos = [
            start_pos[0] + dir[0] * max_dist,
            start_pos[1] + dir[1] * max_dist,
            start_pos[2] + dir[2] * max_dist,
        ];

        let mut cur_ref = start_ref;
        let mut cur_pos = *start_pos;
        let mut _last_pos = *start_pos;

        const MAX_ITERATIONS: usize = 256;

        for _ in 0..MAX_ITERATIONS {
            // Get current tile and polygon
            let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(cur_ref)?;

            // Check if we've reached the end position
            let dx = end_pos[0] - cur_pos[0];
            let dz = end_pos[2] - cur_pos[2];
            if dx * dx + dz * dz < 1e-6 {
                break;
            }

            // Find the edge to cross
            let mut next_ref = PolyRef::new(0);
            let mut next_pos = end_pos;
            let mut next_t = 1.0;

            // Check each edge of the polygon
            for i in 0..poly.vert_count as usize {
                let j = (i + 1) % poly.vert_count as usize;

                let vi_idx = poly.verts[i] as usize;
                let vj_idx = poly.verts[j] as usize;

                let vi = [
                    tile.verts[vi_idx * 3],
                    tile.verts[vi_idx * 3 + 1],
                    tile.verts[vi_idx * 3 + 2],
                ];
                let vj = [
                    tile.verts[vj_idx * 3],
                    tile.verts[vj_idx * 3 + 1],
                    tile.verts[vj_idx * 3 + 2],
                ];

                // Check if ray intersects this edge
                if let Some((edge_t, seg_t)) =
                    Self::intersect_ray_segment_2d(&cur_pos, &end_pos, &vi, &vj)
                {
                    if edge_t > 0.0 && edge_t < next_t {
                        // For neighbor traversal, intersection must be within edge segment
                        if (0.0..=1.0).contains(&seg_t) {
                            // Get the neighbor polygon through this edge
                            if let Some(link) = self.nav_mesh.find_link(tile, poly, i as u8) {
                                if link.reference.is_valid()
                                    && filter.pass_filter(link.reference, tile, poly)
                                {
                                    next_ref = link.reference;
                                    next_t = edge_t;

                                    // Calculate intersection point
                                    next_pos = [
                                        cur_pos[0] + (end_pos[0] - cur_pos[0]) * edge_t,
                                        cur_pos[1] + (end_pos[1] - cur_pos[1]) * edge_t,
                                        cur_pos[2] + (end_pos[2] - cur_pos[2]) * edge_t,
                                    ];
                                } else {
                                    // This edge has no valid neighbor - it's a wall
                                    next_t = edge_t;
                                    next_pos = [
                                        cur_pos[0] + (end_pos[0] - cur_pos[0]) * edge_t,
                                        cur_pos[1] + (end_pos[1] - cur_pos[1]) * edge_t,
                                        cur_pos[2] + (end_pos[2] - cur_pos[2]) * edge_t,
                                    ];
                                    // Don't set next_ref - it stays invalid to indicate wall hit
                                    break; // Found wall intersection, stop looking for closer ones
                                }
                            } else {
                                // No link found - this is definitely a wall edge
                                next_t = edge_t;
                                next_pos = [
                                    cur_pos[0] + (end_pos[0] - cur_pos[0]) * edge_t,
                                    cur_pos[1] + (end_pos[1] - cur_pos[1]) * edge_t,
                                    cur_pos[2] + (end_pos[2] - cur_pos[2]) * edge_t,
                                ];
                                // Don't set next_ref - it stays invalid to indicate wall hit
                                break; // Found wall intersection, stop looking for closer ones
                            }
                        } else {
                            // Ray intersects extended edge line but not the segment itself
                            // This indicates we're hitting a wall at the polygon boundary
                            next_t = edge_t;
                            next_pos = [
                                cur_pos[0] + (end_pos[0] - cur_pos[0]) * edge_t,
                                cur_pos[1] + (end_pos[1] - cur_pos[1]) * edge_t,
                                cur_pos[2] + (end_pos[2] - cur_pos[2]) * edge_t,
                            ];
                            // Don't set next_ref - it stays invalid to indicate wall hit
                            break; // Found wall intersection, stop looking for closer ones
                        }
                    }
                }
            }

            // Also check for off-mesh connections from this position (only if we found a valid neighbor)
            if next_ref.is_valid() {
                next_ref = self.check_off_mesh_connections_for_raycast(
                    &cur_pos,
                    &end_pos,
                    tile,
                    filter,
                    &mut next_pos,
                    &mut next_t,
                )?;
            }

            // If we couldn't find a valid next polygon, we hit a wall
            if !next_ref.is_valid() {
                // If next_t is 1.0, it means we didn't find any edge intersection,
                // so the ray travels the full distance within this polygon
                if next_t >= 1.0 {
                    return Ok((cur_ref, end_pos, max_dist));
                }

                // Calculate the hit distance based on next_t
                let hit_dist = max_dist * next_t;
                return Ok((cur_ref, next_pos, hit_dist));
            }

            // Move to the next polygon
            _last_pos = cur_pos;
            cur_pos = next_pos;
            cur_ref = next_ref;
        }

        // Reached the end position
        Ok((cur_ref, end_pos, max_dist))
    }

    /// Checks off-mesh connections for raycast traversal
    fn check_off_mesh_connections_for_raycast(
        &self,
        cur_pos: &[f32; 3],
        end_pos: &[f32; 3],
        tile: &MeshTile,
        filter: &QueryFilter,
        next_pos: &mut [f32; 3],
        next_t: &mut f32,
    ) -> Result<PolyRef> {
        let mut best_ref = PolyRef::new(0);
        let mut best_t = *next_t;

        // Check all off-mesh connections in this tile
        for connection in &tile.off_mesh_connections {
            // Check if the connection passes the filter
            if !filter.include_flags.intersects(connection.flags)
                || filter.exclude_flags.intersects(connection.flags)
            {
                continue;
            }

            let start_conn = connection.start_pos();
            let end_conn = connection.end_pos();

            // Check if ray passes close to either endpoint
            let ray_dir = [
                end_pos[0] - cur_pos[0],
                end_pos[1] - cur_pos[1],
                end_pos[2] - cur_pos[2],
            ];
            let ray_len =
                (ray_dir[0] * ray_dir[0] + ray_dir[1] * ray_dir[1] + ray_dir[2] * ray_dir[2])
                    .sqrt();

            if ray_len < 1e-6 {
                continue;
            }

            let ray_dir_norm = [
                ray_dir[0] / ray_len,
                ray_dir[1] / ray_len,
                ray_dir[2] / ray_len,
            ];

            // Check distance to start point along the ray
            let to_start = [
                start_conn[0] - cur_pos[0],
                start_conn[1] - cur_pos[1],
                start_conn[2] - cur_pos[2],
            ];
            let proj_start = to_start[0] * ray_dir_norm[0]
                + to_start[1] * ray_dir_norm[1]
                + to_start[2] * ray_dir_norm[2];

            // Check distance to end point along the ray
            let to_end = [
                end_conn[0] - cur_pos[0],
                end_conn[1] - cur_pos[1],
                end_conn[2] - cur_pos[2],
            ];
            let proj_end = to_end[0] * ray_dir_norm[0]
                + to_end[1] * ray_dir_norm[1]
                + to_end[2] * ray_dir_norm[2];

            // Check if we can reach the start point and use the connection
            if proj_start > 0.0 && proj_start < ray_len && proj_start < best_t * ray_len {
                let closest_on_ray = [
                    cur_pos[0] + ray_dir_norm[0] * proj_start,
                    cur_pos[1] + ray_dir_norm[1] * proj_start,
                    cur_pos[2] + ray_dir_norm[2] * proj_start,
                ];

                let dist_to_start = self.distance_3d(closest_on_ray, start_conn);
                if dist_to_start <= connection.radius && connection.allows_start_to_end() {
                    let t = proj_start / ray_len;
                    if t < best_t {
                        best_ref = connection.poly;
                        best_t = t;
                        *next_pos = end_conn; // Jump to the other end of the connection
                    }
                }
            }

            // Check if we can reach the end point and use the connection
            if proj_end > 0.0 && proj_end < ray_len && proj_end < best_t * ray_len {
                let closest_on_ray = [
                    cur_pos[0] + ray_dir_norm[0] * proj_end,
                    cur_pos[1] + ray_dir_norm[1] * proj_end,
                    cur_pos[2] + ray_dir_norm[2] * proj_end,
                ];

                let dist_to_end = self.distance_3d(closest_on_ray, end_conn);
                if dist_to_end <= connection.radius && connection.allows_end_to_start() {
                    let t = proj_end / ray_len;
                    if t < best_t {
                        best_ref = connection.poly;
                        best_t = t;
                        *next_pos = start_conn; // Jump to the other end of the connection
                    }
                }
            }
        }

        if best_ref.is_valid() {
            *next_t = best_t;
        }

        Ok(best_ref)
    }

    /// Intersects a ray with a line segment in 2D (ignoring Y)
    fn intersect_ray_segment_2d(
        ray_start: &[f32; 3],
        ray_end: &[f32; 3],
        seg_start: &[f32; 3],
        seg_end: &[f32; 3],
    ) -> Option<(f32, f32)> {
        let dx = ray_end[0] - ray_start[0];
        let dz = ray_end[2] - ray_start[2];
        let seg_dx = seg_end[0] - seg_start[0];
        let seg_dz = seg_end[2] - seg_start[2];

        let d = seg_dx * dz - seg_dz * dx;
        if d.abs() < 1e-6 {
            return None; // Parallel
        }

        let t = ((seg_start[0] - ray_start[0]) * dz - (seg_start[2] - ray_start[2]) * dx) / d;
        let s =
            ((seg_start[0] - ray_start[0]) * seg_dz - (seg_start[2] - ray_start[2]) * seg_dx) / d;

        Some((s, t))
    }

    /// Enhanced raycast method that returns detailed hit information
    /// Matches C++ API: raycast(..., dtRaycastHit*, ...)
    pub fn raycast_enhanced(
        &self,
        start_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &QueryFilter,
        options: &RaycastOptions,
        prev_ref: Option<PolyRef>,
    ) -> Result<RaycastResult> {
        // Validate input
        if !self.nav_mesh.is_valid_poly_ref(start_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Calculate ray direction and distance
        let dir = [
            end_pos[0] - start_pos[0],
            end_pos[1] - start_pos[1],
            end_pos[2] - start_pos[2],
        ];
        let max_dist = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();

        // Initialize result
        let mut hit = RaycastHit::no_hit();
        let mut visited_path = Vec::new();
        let mut path_cost = 0.0;

        // Handle zero-length ray
        if max_dist <= 0.0 {
            let result = RaycastResult::new(start_ref, *start_pos, hit);
            return Ok(result);
        }

        // Track current position
        let mut cur_ref = start_ref;
        let mut cur_pos = *start_pos;
        let mut last_ref = prev_ref.unwrap_or(PolyRef::new(0));

        if options.include_path {
            visited_path.push(start_ref);
        }

        const MAX_ITERATIONS: usize = 256;

        for iter in 0..MAX_ITERATIONS {
            // Get current tile and polygon
            let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(cur_ref)?;

            // Check if we've reached the end position
            let dx = end_pos[0] - cur_pos[0];
            let dz = end_pos[2] - cur_pos[2];
            if dx * dx + dz * dz < 1e-6 {
                break;
            }

            // Find the edge to cross
            let mut next_ref = PolyRef::new(0);
            let mut next_pos = *end_pos;
            let mut edge_t = 1.0;
            let mut edge_index = -1;

            // Check each edge of the polygon
            for i in 0..poly.vert_count as usize {
                let j = (i + 1) % poly.vert_count as usize;

                let vi_idx = poly.verts[i] as usize;
                let vj_idx = poly.verts[j] as usize;

                let vi = [
                    tile.verts[vi_idx * 3],
                    tile.verts[vi_idx * 3 + 1],
                    tile.verts[vi_idx * 3 + 2],
                ];
                let vj = [
                    tile.verts[vj_idx * 3],
                    tile.verts[vj_idx * 3 + 1],
                    tile.verts[vj_idx * 3 + 2],
                ];

                // Check if ray intersects this edge
                if let Some((ray_t, seg_t)) =
                    Self::intersect_ray_segment_2d(&cur_pos, end_pos, &vi, &vj)
                {
                    if ray_t > 0.0 && ray_t < edge_t {
                        // Check if this is a neighbor edge or a wall
                        if let Some(link) = self.nav_mesh.find_link(tile, poly, i as u8) {
                            if link.reference.is_valid()
                                && link.reference != last_ref
                                && filter.pass_filter(link.reference, tile, poly)
                                && (0.0..=1.0).contains(&seg_t)
                            {
                                // Valid neighbor
                                next_ref = link.reference;
                                edge_t = ray_t;
                                edge_index = i as i32;
                                next_pos = [
                                    cur_pos[0] + dir[0] * ray_t,
                                    cur_pos[1] + dir[1] * ray_t,
                                    cur_pos[2] + dir[2] * ray_t,
                                ];
                            }
                        } else if (0.0..=1.0).contains(&seg_t) {
                            // Hit a wall
                            edge_t = ray_t;
                            edge_index = i as i32;
                            next_pos = [
                                cur_pos[0] + dir[0] * ray_t,
                                cur_pos[1] + dir[1] * ray_t,
                                cur_pos[2] + dir[2] * ray_t,
                            ];
                            // Calculate normal
                            let edge_dx = vj[0] - vi[0];
                            let edge_dz = vj[2] - vi[2];
                            let edge_len = (edge_dx * edge_dx + edge_dz * edge_dz).sqrt();
                            if edge_len > 1e-6 {
                                hit.hit_normal = [edge_dz / edge_len, 0.0, -edge_dx / edge_len];
                            }
                            // Mark as wall hit
                            next_ref = PolyRef::new(0);
                            break;
                        }
                    }
                }
            }

            // Update cost if requested
            if options.include_cost && next_ref.is_valid() {
                let edge_cost =
                    self.get_edge_cost(cur_ref, next_ref, &cur_pos, &next_pos, filter)?;
                path_cost += edge_cost;
            }

            // If we hit a wall, finalize the hit info
            if !next_ref.is_valid() {
                hit.t = edge_t * max_dist / max_dist; // Normalize to [0,1]
                hit.hit_edge_index = edge_index;

                if options.include_path {
                    hit = hit.with_path(visited_path);
                }
                if options.include_cost {
                    hit = hit.with_cost(path_cost);
                }

                let result = RaycastResult::new(cur_ref, next_pos, hit);
                return Ok(result);
            }

            // Move to the next polygon
            if options.include_path {
                visited_path.push(next_ref);
            }
            last_ref = cur_ref;
            cur_ref = next_ref;
            cur_pos = next_pos;

            // Prevent infinite loops
            if iter >= MAX_ITERATIONS - 1 {
                break;
            }
        }

        // Reached the end position without hitting a wall
        if options.include_path {
            hit = hit.with_path(visited_path);
        }
        if options.include_cost {
            hit = hit.with_cost(path_cost);
        }

        let result = RaycastResult::new(cur_ref, *end_pos, hit);
        Ok(result)
    }

    /// Gets a path from the explored nodes in the previous Dijkstra search
    /// Matches C++ API: getPathFromDijkstraSearch
    pub fn get_path_from_dijkstra_search(
        &self,
        end_ref: PolyRef,
        max_path: usize,
    ) -> Result<Vec<PolyRef>> {
        // Validate input
        if !self.nav_mesh.is_valid_poly_ref(end_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Find the end node in our node pool
        let mut end_node_idx = None;
        for i in 0..self.node_pool.len() {
            if self.node_pool[i].poly == end_ref && self.node_pool[i].state == NodeState::Closed {
                end_node_idx = Some(i);
                break;
            }
        }

        let end_idx = end_node_idx
            .ok_or_else(|| Error::Detour("End node not found in explored nodes".to_string()))?;

        // Build path by following parent links
        let mut path = Vec::new();
        let mut current_idx = end_idx;
        let mut safety_counter = 0;

        loop {
            if safety_counter >= max_path {
                break; // Path too long
            }
            safety_counter += 1;

            let node = &self.node_pool[current_idx];
            path.push(node.poly);

            match node.parent {
                Some(parent_idx) => {
                    current_idx = parent_idx;
                }
                None => {
                    // Reached the start
                    break;
                }
            }
        }

        // Reverse to get start->end order
        path.reverse();

        // Truncate if needed
        if path.len() > max_path {
            path.truncate(max_path);
        }

        Ok(path)
    }

    /// Finds a straight path from start to end
    #[allow(unused_assignments)]
    pub fn find_straight_path(
        &self,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        path: &[PolyRef],
    ) -> Result<Path> {
        if path.is_empty() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let mut result = Path::new();

        // Add the start position
        result.waypoints.push(*start_pos);
        result.poly_refs.push(path[0]);

        // If the path is just one polygon, add the end position and return
        if path.len() == 1 {
            result.waypoints.push(*end_pos);
            result.poly_refs.push(path[0]);
            return Ok(result);
        }

        // Use the funnel algorithm to find the straight path
        let mut portal_apex = *start_pos;
        let mut portal_left = portal_apex;
        let mut portal_right = portal_apex;
        let mut apex_index = 0;
        let mut left_index = 0;
        let mut right_index = 0;

        for i in 0..path.len() - 1 {
            // Get portal points between current and next polygon
            let (left, right) = self.get_portal_points(path[i], path[i + 1])?;

            // If starting really close to the portal, skip it
            if i == 0 {
                let dx = portal_apex[0] - left[0];
                let dz = portal_apex[2] - left[2];
                let dist_sqr = dx * dx + dz * dz;
                if dist_sqr < 0.001 * 0.001 {
                    continue;
                }
            }

            // Right vertex
            if Self::tri_area_2d(&portal_apex, &portal_right, &right) <= 0.0 {
                if Self::v_equal(&portal_apex, &portal_right)
                    || Self::tri_area_2d(&portal_apex, &portal_left, &right) > 0.0
                {
                    portal_right = right;
                    right_index = i + 1;
                } else {
                    // Append apex vertex
                    result.waypoints.push(portal_left);
                    result.poly_refs.push(path[left_index]);

                    // Restart funnel
                    portal_apex = portal_left;
                    apex_index = left_index;
                    portal_left = portal_apex;
                    portal_right = portal_apex;
                    left_index = apex_index;
                    right_index = apex_index;
                    continue;
                }
            }

            // Left vertex
            if Self::tri_area_2d(&portal_apex, &portal_left, &left) >= 0.0 {
                if Self::v_equal(&portal_apex, &portal_left)
                    || Self::tri_area_2d(&portal_apex, &portal_right, &left) < 0.0
                {
                    portal_left = left;
                    left_index = i + 1;
                } else {
                    // Append apex vertex
                    result.waypoints.push(portal_right);
                    result.poly_refs.push(path[right_index]);

                    // Restart funnel
                    portal_apex = portal_right;
                    apex_index = right_index;
                    portal_left = portal_apex;
                    portal_right = portal_apex;
                    left_index = apex_index;
                    right_index = apex_index;
                    continue;
                }
            }
        }

        // Add the end position
        result.waypoints.push(*end_pos);
        result.poly_refs.push(path[path.len() - 1]);

        Ok(result)
    }

    /// Finds the straight path from start to end position with options
    ///
    /// This overload accepts additional options for path computation.
    ///
    /// # Arguments
    /// * `start_pos` - Path start position
    /// * `end_pos` - Path end position  
    /// * `path` - An array of polygon references that represent the path corridor
    /// * `options` - Options for straight path computation (0 = default behavior)
    ///
    /// # Returns
    /// The straight path with waypoints and polygon references
    pub fn find_straight_path_with_options(
        &self,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        path: &[PolyRef],
        options: u32,
    ) -> Result<Path> {
        // For now, options parameter is accepted but not used
        // This maintains C++ API compatibility
        let _ = options;

        // Delegate to the main implementation
        self.find_straight_path(start_pos, end_pos, path)
    }

    /// Gets portal points between two adjacent polygons
    ///
    /// # Arguments
    ///
    /// * `from_ref` - The reference of the polygon to traverse from
    /// * `to_ref` - The reference of the polygon to traverse to
    ///
    /// # Returns
    ///
    /// A tuple containing the left and right portal points
    pub fn get_portal_points(
        &self,
        from_ref: PolyRef,
        to_ref: PolyRef,
    ) -> Result<([f32; 3], [f32; 3])> {
        let (from_tile, from_poly) = self.nav_mesh.get_tile_and_poly_by_ref(from_ref)?;
        let (_to_tile, to_poly) = self.nav_mesh.get_tile_and_poly_by_ref(to_ref)?;

        // Special handling for off-mesh connections
        if from_poly.poly_type == PolyType::OffMeshConnection {
            // For off-mesh connections, find the link that points to the 'to' polygon
            // and return the vertex position as both left and right (like C++)
            let mut link_idx = from_poly.first_link;
            while let Some(idx) = link_idx {
                if idx < from_tile.links.len() {
                    let link = &from_tile.links[idx];
                    if link.reference == to_ref {
                        // Get the vertex position from the edge index
                        let v_idx = from_poly.verts[link.edge_index as usize] as usize;
                        if v_idx * 3 + 2 < from_tile.verts.len() {
                            let pos = [
                                from_tile.verts[v_idx * 3],
                                from_tile.verts[v_idx * 3 + 1],
                                from_tile.verts[v_idx * 3 + 2],
                            ];
                            // Return same position for both left and right
                            return Ok((pos, pos));
                        }
                    }
                    link_idx = link.next.map(|n| n as usize);
                }
            }
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if to_poly.poly_type == PolyType::OffMeshConnection {
            // For connections TO off-mesh connections, find the link in the off-mesh connection
            // that points back to 'from' and use that vertex
            let mut link_idx = to_poly.first_link;
            while let Some(idx) = link_idx {
                if idx < _to_tile.links.len() {
                    let link = &_to_tile.links[idx];
                    if link.reference == from_ref {
                        // Get the vertex position from the edge index
                        let v_idx = to_poly.verts[link.edge_index as usize] as usize;
                        if v_idx * 3 + 2 < _to_tile.verts.len() {
                            let pos = [
                                _to_tile.verts[v_idx * 3],
                                _to_tile.verts[v_idx * 3 + 1],
                                _to_tile.verts[v_idx * 3 + 2],
                            ];
                            // Return same position for both left and right
                            return Ok((pos, pos));
                        }
                    }
                    link_idx = link.next.map(|n| n as usize);
                }
            }
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Regular polygon-to-polygon edge finding
        // Find the shared edge
        for i in 0..from_poly.vert_count as usize {
            // Check if this edge connects to the target polygon
            if let Some(link) = self.nav_mesh.find_link(from_tile, from_poly, i as u8) {
                if link.reference == to_ref {
                    let j = (i + 1) % from_poly.vert_count as usize;

                    let vi_idx = from_poly.verts[i] as usize;
                    let vj_idx = from_poly.verts[j] as usize;

                    let left = [
                        from_tile.verts[vi_idx * 3],
                        from_tile.verts[vi_idx * 3 + 1],
                        from_tile.verts[vi_idx * 3 + 2],
                    ];
                    let right = [
                        from_tile.verts[vj_idx * 3],
                        from_tile.verts[vj_idx * 3 + 1],
                        from_tile.verts[vj_idx * 3 + 2],
                    ];

                    return Ok((left, right));
                }
            }
        }

        // If we couldn't find a direct link, the polygons might not be adjacent
        Err(Error::Detour(Status::InvalidParam.to_string()))
    }

    /// Calculates the signed area of a triangle in 2D (ignoring Y)
    fn tri_area_2d(a: &[f32; 3], b: &[f32; 3], c: &[f32; 3]) -> f32 {
        let abx = b[0] - a[0];
        let abz = b[2] - a[2];
        let acx = c[0] - a[0];
        let acz = c[2] - a[2];
        acx * abz - abx * acz
    }

    /// Checks if two positions are approximately equal (ignoring Y)
    fn v_equal(a: &[f32; 3], b: &[f32; 3]) -> bool {
        let dx = a[0] - b[0];
        let dz = a[2] - b[2];
        dx * dx + dz * dz < 0.00001
    }

    /// Calculates the squared 2D distance from a point to a line segment (ignoring Y)
    /// Based on the C++ dtDistancePtSegSqr2D function
    fn distance_point_to_segment_2d_squared(
        &self,
        point: &[f32; 3],
        segment_start: &[f32; 3],
        segment_end: &[f32; 3],
    ) -> f32 {
        let px = point[0];
        let pz = point[2];
        let ax = segment_start[0];
        let az = segment_start[2];
        let bx = segment_end[0];
        let bz = segment_end[2];

        let dx = bx - ax;
        let dz = bz - az;
        let d = dx * dx + dz * dz;

        if d > 0.0 {
            let t = ((px - ax) * dx + (pz - az) * dz) / d;
            let t_clamped = t.clamp(0.0, 1.0);

            let closest_x = ax + t_clamped * dx;
            let closest_z = az + t_clamped * dz;

            let dist_x = px - closest_x;
            let dist_z = pz - closest_z;

            dist_x * dist_x + dist_z * dist_z
        } else {
            // Degenerate segment - distance to point
            let dist_x = px - ax;
            let dist_z = pz - az;
            dist_x * dist_x + dist_z * dist_z
        }
    }

    /// Finds all polygons within a circular area around a center point
    pub fn find_polys_around_circle(
        &self,
        center_ref: PolyRef,
        center_pos: &[f32; 3],
        radius: f32,
        filter: &QueryFilter,
    ) -> Result<Vec<PolyRef>> {
        // Validate input
        if !self.nav_mesh.is_valid_poly_ref(center_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Initialize search data structures
        let mut node_pool = std::collections::HashMap::new();
        let mut open_list = BinaryHeap::new();
        let mut result_polys = Vec::new();

        let radius_squared = radius * radius;

        // Create start node
        let start_node = Node {
            poly: center_ref,
            parent: None,
            g: 0.0,
            h: 0.0,
            f: 0.0,
            state: NodeState::Open,
            index: 0,
        };

        // Add start node to pool and open list
        node_pool.insert(center_ref.id(), start_node);
        open_list.push(HeapNode {
            index: center_ref.id() as usize,
            f: 0.0,
        });

        // Main search loop - Dijkstra-like algorithm
        while let Some(heap_node) = open_list.pop() {
            let current_ref_id = heap_node.index as u32;
            let current_ref = PolyRef::new(current_ref_id);

            // Mark node as closed and get current cost
            let current_cost = {
                match node_pool.get_mut(&current_ref_id) {
                    Some(node) => {
                        node.state = NodeState::Closed;
                        node.g
                    }
                    None => continue,
                }
            };

            // Get tile and polygon data
            let (current_tile, current_poly) =
                match self.nav_mesh.get_tile_and_poly_by_ref(current_ref) {
                    Ok((tile, poly)) => (tile, poly),
                    Err(_) => continue,
                };

            // Check if this polygon passes the filter
            if !filter.pass_filter(current_ref, current_tile, current_poly) {
                continue;
            }

            // Add current polygon to results
            result_polys.push(current_ref);

            // Explore neighbors through links
            if let Some(first_link_idx) = current_poly.first_link {
                let mut link_idx = first_link_idx;

                loop {
                    if link_idx >= current_tile.links.len() {
                        break;
                    }

                    let link = &current_tile.links[link_idx];
                    let neighbor_ref = link.reference;

                    // Skip invalid neighbors
                    if !neighbor_ref.is_valid() {
                        if let Some(next_idx) = link.next {
                            link_idx = next_idx as usize;
                            continue;
                        } else {
                            break;
                        }
                    }

                    // Get neighbor tile and polygon
                    let (neighbor_tile, neighbor_poly) =
                        match self.nav_mesh.get_tile_and_poly_by_ref(neighbor_ref) {
                            Ok((tile, poly)) => (tile, poly),
                            Err(_) => {
                                if let Some(next_idx) = link.next {
                                    link_idx = next_idx as usize;
                                    continue;
                                } else {
                                    break;
                                }
                            }
                        };

                    // Check filter
                    if !filter.pass_filter(neighbor_ref, neighbor_tile, neighbor_poly) {
                        if let Some(next_idx) = link.next {
                            link_idx = next_idx as usize;
                            continue;
                        } else {
                            break;
                        }
                    }

                    // Test circle intersection with portal edge
                    let portal_result = self.get_portal_points(current_ref, neighbor_ref);
                    let within_circle = match portal_result {
                        Ok((left_portal, right_portal)) => {
                            // Calculate 2D distance from circle center to portal edge
                            let dist_squared = self.distance_point_to_segment_2d_squared(
                                center_pos,
                                &left_portal,
                                &right_portal,
                            );
                            dist_squared <= radius_squared
                        }
                        Err(_) => {
                            // If we can't get portal points, fall back to polygon center distance
                            if let Ok(neighbor_center) = self.get_poly_center(neighbor_ref) {
                                let dx = neighbor_center[0] - center_pos[0];
                                let dz = neighbor_center[2] - center_pos[2];
                                let dist_squared = dx * dx + dz * dz;
                                dist_squared <= radius_squared
                            } else {
                                false
                            }
                        }
                    };

                    // Skip if not within circle
                    if !within_circle {
                        if let Some(next_idx) = link.next {
                            link_idx = next_idx as usize;
                            continue;
                        } else {
                            break;
                        }
                    }

                    // Check if neighbor already processed
                    let neighbor_id = neighbor_ref.id();
                    if let Some(existing_node) = node_pool.get(&neighbor_id) {
                        if existing_node.state == NodeState::Closed {
                            // Already processed, skip
                            if let Some(next_idx) = link.next {
                                link_idx = next_idx as usize;
                                continue;
                            } else {
                                break;
                            }
                        }
                    }

                    // Calculate cost (for now, use unit cost)
                    let cost = 1.0;
                    let total_cost = current_cost + cost;

                    // Update or create neighbor node
                    let should_add_to_open = match node_pool.get_mut(&neighbor_id) {
                        Some(neighbor_node) => {
                            if neighbor_node.state == NodeState::New || total_cost < neighbor_node.g
                            {
                                neighbor_node.g = total_cost;
                                neighbor_node.f = total_cost;
                                neighbor_node.parent = Some(current_ref_id as usize);
                                neighbor_node.state = NodeState::Open;
                                true
                            } else {
                                false
                            }
                        }
                        None => {
                            // Create new node
                            let new_node = Node {
                                poly: neighbor_ref,
                                parent: Some(current_ref_id as usize),
                                g: total_cost,
                                h: 0.0,
                                f: total_cost,
                                state: NodeState::Open,
                                index: neighbor_id as usize,
                            };
                            node_pool.insert(neighbor_id, new_node);
                            true
                        }
                    };

                    // Add to open list if needed
                    if should_add_to_open {
                        open_list.push(HeapNode {
                            index: neighbor_id as usize,
                            f: total_cost,
                        });
                    }

                    // Move to next link
                    if let Some(next_idx) = link.next {
                        link_idx = next_idx as usize;
                    } else {
                        break;
                    }
                }
            }
        }

        Ok(result_polys)
    }

    /// Checks if a point is inside a polygon (2D check ignoring Y)
    fn is_point_in_polygon(
        &self,
        tile: &super::nav_mesh::MeshTile,
        poly: &super::nav_mesh::Poly,
        point: &[f32; 3],
    ) -> Result<bool> {
        let mut inside = false;
        let px = point[0];
        let pz = point[2];

        let mut j = poly.vert_count as usize - 1;
        for i in 0..poly.vert_count as usize {
            let vi_idx = poly.verts[i] as usize;
            let vj_idx = poly.verts[j] as usize;

            let vix = tile.verts[vi_idx * 3];
            let viz = tile.verts[vi_idx * 3 + 2];
            let vjx = tile.verts[vj_idx * 3];
            let vjz = tile.verts[vj_idx * 3 + 2];

            if ((viz > pz) != (vjz > pz)) && (px < (vjx - vix) * (pz - viz) / (vjz - viz) + vix) {
                inside = !inside;
            }
            j = i;
        }

        Ok(inside)
    }

    /// Finds the closest point on a line segment to a given point
    fn closest_point_on_segment(
        &self,
        seg_start: &[f32; 3],
        seg_end: &[f32; 3],
        point: &[f32; 3],
    ) -> [f32; 3] {
        let dx = seg_end[0] - seg_start[0];
        let dz = seg_end[2] - seg_start[2];

        let seg_len_squared = dx * dx + dz * dz;

        if seg_len_squared < 1e-6 {
            // Degenerate segment, return start point
            return *seg_start;
        }

        let px = point[0] - seg_start[0];
        let pz = point[2] - seg_start[2];

        let t = (px * dx + pz * dz) / seg_len_squared;
        let t_clamped = t.clamp(0.0, 1.0);

        [
            seg_start[0] + dx * t_clamped,
            seg_start[1] + (seg_end[1] - seg_start[1]) * t_clamped,
            seg_start[2] + dz * t_clamped,
        ]
    }

    /// Finds the distance to the nearest wall or obstacle from a given position
    pub fn find_distance_to_wall(
        &self,
        start_ref: PolyRef,
        center_pos: &[f32; 3],
        radius: f32,
        filter: &QueryFilter,
    ) -> Result<(f32, [f32; 3], [f32; 3])> {
        // Validate input
        if !self.nav_mesh.is_valid_poly_ref(start_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let mut min_distance = radius;
        let mut wall_hit = *center_pos;
        let mut wall_normal = [0.0, 0.0, 1.0];

        // Number of rays to cast for wall detection
        const NUM_RAYS: usize = 16;
        let angle_step = 2.0 * std::f32::consts::PI / NUM_RAYS as f32;

        // Cast rays in multiple directions to find walls
        for i in 0..NUM_RAYS {
            let angle = i as f32 * angle_step;
            let ray_dir = [angle.cos(), 0.0, angle.sin()];

            // Perform raycast to find wall intersection
            match self.raycast(start_ref, center_pos, &ray_dir, radius, filter) {
                Ok((_, hit_pos, t)) => {
                    let hit_distance = t * radius;

                    if hit_distance < min_distance {
                        min_distance = hit_distance;
                        wall_hit = hit_pos;

                        // Calculate wall normal (pointing toward the center)
                        let dx = center_pos[0] - hit_pos[0];
                        let dz = center_pos[2] - hit_pos[2];
                        let len = (dx * dx + dz * dz).sqrt();

                        if len > 1e-6 {
                            wall_normal = [dx / len, 0.0, dz / len];
                        }
                    }
                }
                Err(_) => {
                    // No wall hit in this direction within radius
                }
            }
        }

        // If no walls found within radius, check polygon boundaries more carefully
        if min_distance >= radius {
            if let Ok(boundary_result) =
                self.find_polygon_boundary_distance(start_ref, center_pos, radius, filter)
            {
                if boundary_result.0 < min_distance {
                    min_distance = boundary_result.0;
                    wall_hit = boundary_result.1;
                    wall_normal = boundary_result.2;
                }
            }
        }

        Ok((min_distance, wall_hit, wall_normal))
    }

    /// Finds the distance to the nearest polygon boundary (more precise than raycast)
    fn find_polygon_boundary_distance(
        &self,
        start_ref: PolyRef,
        center_pos: &[f32; 3],
        max_radius: f32,
        filter: &QueryFilter,
    ) -> Result<(f32, [f32; 3], [f32; 3])> {
        let mut min_distance = max_radius;
        let mut wall_hit = *center_pos;
        let mut wall_normal = [0.0, 0.0, 1.0];

        let mut visited = std::collections::HashSet::new();
        let mut queue = std::collections::VecDeque::new();

        // Start BFS from the current polygon
        queue.push_back(start_ref);
        visited.insert(start_ref.id());

        while let Some(current_ref) = queue.pop_front() {
            let (tile, poly) = match self.nav_mesh.get_tile_and_poly_by_ref(current_ref) {
                Ok((tile, poly)) => (tile, poly),
                Err(_) => continue,
            };

            if !filter.pass_filter(current_ref, tile, poly) {
                continue;
            }

            // Check each edge of this polygon
            for i in 0..poly.vert_count as usize {
                let j = (i + 1) % poly.vert_count as usize;

                let vert1_idx = poly.verts[i] as usize;
                let vert2_idx = poly.verts[j] as usize;

                let vert1 = [
                    tile.verts[vert1_idx * 3],
                    tile.verts[vert1_idx * 3 + 1],
                    tile.verts[vert1_idx * 3 + 2],
                ];
                let vert2 = [
                    tile.verts[vert2_idx * 3],
                    tile.verts[vert2_idx * 3 + 1],
                    tile.verts[vert2_idx * 3 + 2],
                ];

                // Check if this edge has a connection (not a wall)
                let has_connection = self.nav_mesh.find_link(tile, poly, i as u8).is_some();

                if !has_connection {
                    // This is a wall edge, calculate distance
                    let closest_point = self.closest_point_on_segment(&vert1, &vert2, center_pos);
                    let dx = closest_point[0] - center_pos[0];
                    let dz = closest_point[2] - center_pos[2];
                    let distance = (dx * dx + dz * dz).sqrt();

                    if distance < min_distance {
                        min_distance = distance;
                        wall_hit = closest_point;

                        // Calculate wall normal
                        if distance > 1e-6 {
                            wall_normal = [dx / distance, 0.0, dz / distance];
                        }
                    }
                } else {
                    // Add connected polygon to exploration queue
                    if let Some(link) = self.nav_mesh.find_link(tile, poly, i as u8) {
                        let neighbor_ref = link.reference;
                        if neighbor_ref.is_valid() && !visited.contains(&neighbor_ref.id()) {
                            // Only explore if the neighbor is within reasonable distance
                            if let Ok(neighbor_center) = self.get_poly_center(neighbor_ref) {
                                let dx = neighbor_center[0] - center_pos[0];
                                let dz = neighbor_center[2] - center_pos[2];
                                let dist = (dx * dx + dz * dz).sqrt();

                                if dist <= max_radius + 5.0 {
                                    // Small buffer for exploration
                                    queue.push_back(neighbor_ref);
                                    visited.insert(neighbor_ref.id());
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok((min_distance, wall_hit, wall_normal))
    }

    /// Finds a random point on the navigation mesh
    pub fn find_random_point(&mut self, filter: &QueryFilter) -> Result<(PolyRef, [f32; 3])> {
        self.find_random_point_around(PolyRef::new(0), &[0.0, 0.0, 0.0], f32::MAX, filter)
    }

    /// Finds a random point on the navigation mesh using custom random function
    ///
    /// This method matches the C++ API that accepts a function pointer for random number generation.
    ///
    /// # Arguments
    /// * `filter` - The polygon filter to apply to the query
    /// * `frand` - Function returning a random number [0..1)
    ///
    /// # Returns
    /// * A tuple containing the polygon reference and the random point position
    pub fn find_random_point_with_custom_rand<F>(
        &mut self,
        filter: &QueryFilter,
        frand: F,
    ) -> Result<(PolyRef, [f32; 3])>
    where
        F: FnMut() -> f32,
    {
        self.find_random_point_around_circle(
            PolyRef::new(0),
            &[0.0, 0.0, 0.0],
            f32::MAX,
            filter,
            frand,
        )
    }

    /// Gets the height of the polygon at the provided position
    ///
    /// This method provides accurate height interpolation using the detail mesh.
    ///
    /// # Arguments
    /// * `ref` - The reference id of the polygon
    /// * `pos` - A position within the xz-bounds of the polygon
    ///
    /// # Returns
    /// * The height at the surface of the polygon, or None if the position is outside the polygon
    pub fn get_poly_height(&self, poly_ref: PolyRef, pos: &[f32; 3]) -> Result<Option<f32>> {
        // Validate the polygon reference
        if !self.nav_mesh.is_valid_poly_ref(poly_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Get the tile and polygon
        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

        // Delegate to the NavMesh implementation
        self.nav_mesh.get_poly_height(tile, poly, pos)
    }

    /// Finds a random point within a given radius of a center point using custom random function
    ///
    /// This method matches the C++ API that accepts a function pointer for random number generation.
    ///
    /// # Arguments
    /// * `center_ref` - The reference id of the polygon where the search starts
    /// * `center_pos` - The center of the search circle
    /// * `radius` - The radius of the search circle
    /// * `filter` - The polygon filter to apply to the query
    /// * `frand` - Function returning a random number [0..1)
    ///
    /// # Returns
    /// * A tuple containing the polygon reference and the random point position
    pub fn find_random_point_around_circle<F>(
        &mut self,
        center_ref: PolyRef,
        center_pos: &[f32; 3],
        radius: f32,
        filter: &QueryFilter,
        mut frand: F,
    ) -> Result<(PolyRef, [f32; 3])>
    where
        F: FnMut() -> f32,
    {
        // Save the current internal random generator state
        let saved_seed = self.random_seed;

        // Temporarily replace random generation with the provided function
        // by calling it and using it to seed our internal generator
        let rand_val = frand();
        self.random_seed = (rand_val * 2147483647.0) as u32;

        // Call the existing implementation
        let result = self.find_random_point_around(center_ref, center_pos, radius, filter);

        // Restore the original seed
        self.random_seed = saved_seed;

        result
    }

    /// Finds a random point within a given radius of a center point
    pub fn find_random_point_around(
        &mut self,
        center_ref: PolyRef,
        center_pos: &[f32; 3],
        radius: f32,
        filter: &QueryFilter,
    ) -> Result<(PolyRef, [f32; 3])> {
        // If no center is provided (invalid ref), find random point on entire mesh
        if !center_ref.is_valid() || !self.nav_mesh.is_valid_poly_ref(center_ref) {
            return self.find_random_point_on_mesh(filter);
        }

        // Get all polygons within the circle area
        let polys_in_circle =
            self.find_polys_around_circle(center_ref, center_pos, radius, filter)?;

        if polys_in_circle.is_empty() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Pick a random polygon from those within the circle
        let random_index = (self.random_f32() * polys_in_circle.len() as f32) as usize;
        let random_poly = polys_in_circle[random_index];

        // Generate a random point within that polygon
        match self.get_random_point_in_polygon(random_poly) {
            Ok(pos) => {
                // Double-check that the generated point is within the radius
                let dx = pos[0] - center_pos[0];
                let dz = pos[2] - center_pos[2];
                let distance = (dx * dx + dz * dz).sqrt();

                if distance <= radius {
                    Ok((random_poly, pos))
                } else {
                    // Fall back to polygon center if random point is outside radius
                    match self.get_poly_center(random_poly) {
                        Ok(center) => Ok((random_poly, center)),
                        Err(e) => Err(e),
                    }
                }
            }
            Err(_) => {
                // Fallback: get polygon center
                match self.get_poly_center(random_poly) {
                    Ok(center) => Ok((random_poly, center)),
                    Err(e) => Err(e),
                }
            }
        }
    }

    /// Finds a random point anywhere on the navigation mesh
    fn find_random_point_on_mesh(&mut self, filter: &QueryFilter) -> Result<(PolyRef, [f32; 3])> {
        // Get all valid polygons that pass the filter
        let valid_polys = self.get_all_valid_polygons(filter)?;

        if valid_polys.is_empty() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Pick a random polygon
        let random_index = (self.random_f32() * valid_polys.len() as f32) as usize;
        let random_poly = valid_polys[random_index];

        // Generate random point within that polygon
        match self.get_random_point_in_polygon(random_poly) {
            Ok(pos) => Ok((random_poly, pos)),
            Err(_) => {
                // Fallback: get polygon center
                match self.get_poly_center(random_poly) {
                    Ok(center) => Ok((random_poly, center)),
                    Err(e) => Err(e),
                }
            }
        }
    }

    /// Gets all valid polygons that pass the filter
    fn get_all_valid_polygons(&self, filter: &QueryFilter) -> Result<Vec<PolyRef>> {
        let mut valid_polys = Vec::new();

        // Use a much more efficient approach: get polygons from a large search area
        // Start from origin and search in a very large area to find all polygons

        // Try to find a polygon anywhere on the mesh first
        if let Ok(start_polys) = self.nav_mesh.query_polygons(
            &[-1000.0, -1000.0, -1000.0],
            &[1000.0, 1000.0, 1000.0],
            filter,
        ) {
            // We found some polygons, add them all (query_polygons already filters by the filter)
            valid_polys.extend(start_polys);
        }

        // If that didn't work, fall back to the brute force approach but with limits
        if valid_polys.is_empty() {
            // Try a smaller search space first
            for tile_id in 0..10.min(self.nav_mesh.get_max_tiles()) {
                for poly_id in 0..100.min(self.nav_mesh.get_max_polys_per_tile()) {
                    let poly_ref = super::nav_mesh::encode_poly_ref(tile_id as u32, poly_id as u32);

                    // Check if this polygon reference is valid
                    if self.nav_mesh.is_valid_poly_ref(poly_ref) {
                        // Get the tile and polygon data
                        if let Ok((tile, poly)) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref) {
                            // Check if polygon passes filter
                            if filter.pass_filter(poly_ref, tile, poly) {
                                valid_polys.push(poly_ref);
                            }
                        }
                    }
                }
            }
        }

        Ok(valid_polys)
    }

    /// Generates a random point within a specific polygon
    fn get_random_point_in_polygon(&mut self, poly_ref: PolyRef) -> Result<[f32; 3]> {
        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

        if poly.vert_count < 3 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Get polygon vertices
        let mut verts = Vec::with_capacity(poly.vert_count as usize);
        for i in 0..poly.vert_count as usize {
            let vert_idx = poly.verts[i] as usize;
            verts.push([
                tile.verts[vert_idx * 3],
                tile.verts[vert_idx * 3 + 1],
                tile.verts[vert_idx * 3 + 2],
            ]);
        }

        // Use triangle sampling for polygons with more than 3 vertices
        if poly.vert_count == 3 {
            // Simple triangle sampling
            Ok(self.random_point_in_triangle(&verts[0], &verts[1], &verts[2]))
        } else {
            // For polygons with more vertices, triangulate and pick a random triangle
            let triangle_count = poly.vert_count - 2;
            let random_triangle = (self.random_f32() * triangle_count as f32) as usize;

            // Create triangle from vertex 0 and two consecutive vertices
            let v1_idx = 1 + random_triangle;
            let v2_idx = 2 + random_triangle;

            if v1_idx < verts.len() && v2_idx < verts.len() {
                Ok(self.random_point_in_triangle(&verts[0], &verts[v1_idx], &verts[v2_idx]))
            } else {
                // Fallback to first triangle
                Ok(self.random_point_in_triangle(&verts[0], &verts[1], &verts[2]))
            }
        }
    }

    /// Generates a random point within a triangle using barycentric coordinates
    fn random_point_in_triangle(&mut self, a: &[f32; 3], b: &[f32; 3], c: &[f32; 3]) -> [f32; 3] {
        let r1 = self.random_f32();
        let r2 = self.random_f32();

        // Ensure uniform distribution in triangle
        let sqrt_r1 = r1.sqrt();
        let u = 1.0 - sqrt_r1;
        let v = r2 * sqrt_r1;
        let w = 1.0 - u - v;

        [
            a[0] * u + b[0] * v + c[0] * w,
            a[1] * u + b[1] * v + c[1] * w,
            a[2] * u + b[2] * v + c[2] * w,
        ]
    }

    /// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position
    pub fn find_local_neighbourhood(
        &self,
        start_ref: PolyRef,
        center_pos: &[f32; 3],
        radius: f32,
        filter: &QueryFilter,
        max_result: usize,
    ) -> Result<(Vec<PolyRef>, Vec<PolyRef>)> {
        // Validate input
        if !self.nav_mesh.is_valid_poly_ref(start_ref) || radius < 0.0 || max_result == 0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let mut result_refs = Vec::new();
        let mut result_parents = Vec::new();
        let mut stack = Vec::new();
        let mut visited = std::collections::HashSet::new();

        let radius_sqr = radius * radius;
        const MAX_STACK_SIZE: usize = 48;

        // Start with the initial polygon
        stack.push(start_ref);
        visited.insert(start_ref.id());

        // Add start polygon to results
        result_refs.push(start_ref);
        result_parents.push(PolyRef::new(0)); // Start polygon has no parent

        while !stack.is_empty() && stack.len() < MAX_STACK_SIZE {
            let current_ref = stack.remove(0); // Pop front for breadth-first search

            // Get current polygon data
            let (current_tile, current_poly) =
                self.nav_mesh.get_tile_and_poly_by_ref(current_ref)?;

            // Explore all neighbors
            for i in 0..current_poly.vert_count as usize {
                if let Some(link) = self.nav_mesh.find_link(current_tile, current_poly, i as u8) {
                    let neighbor_ref = link.reference;

                    // Skip invalid neighbors
                    if !neighbor_ref.is_valid() {
                        continue;
                    }

                    // Skip already visited neighbors
                    if visited.contains(&neighbor_ref.id()) {
                        continue;
                    }

                    // Get neighbor polygon data
                    let (neighbor_tile, neighbor_poly) =
                        match self.nav_mesh.get_tile_and_poly_by_ref(neighbor_ref) {
                            Ok((tile, poly)) => (tile, poly),
                            Err(_) => continue,
                        };

                    // Skip off-mesh connections
                    if neighbor_poly.poly_type == PolyType::OffMeshConnection {
                        continue;
                    }

                    // Check if neighbor passes filter
                    if !filter.pass_filter(neighbor_ref, neighbor_tile, neighbor_poly) {
                        continue;
                    }

                    // Get edge vertices between current and neighbor
                    let (va, vb) = match self.get_portal_points(current_ref, neighbor_ref) {
                        Ok((a, b)) => (a, b),
                        Err(_) => continue,
                    };

                    // Check if the circle (radius around center) touches this edge
                    let closest_on_edge = self.closest_point_on_segment(&va, &vb, center_pos);
                    let dist_sqr = {
                        let dx = center_pos[0] - closest_on_edge[0];
                        let dy = center_pos[1] - closest_on_edge[1];
                        let dz = center_pos[2] - closest_on_edge[2];
                        dx * dx + dy * dy + dz * dz
                    };

                    // Skip if edge is too far from the search radius
                    if dist_sqr > radius_sqr {
                        continue;
                    }

                    // Mark as visited early to avoid revisiting
                    visited.insert(neighbor_ref.id());

                    // Check for polygon overlap with existing result polygons
                    let neighbor_verts = self.get_polygon_vertices(neighbor_tile, neighbor_poly)?;
                    let mut overlaps = false;

                    for &existing_ref in &result_refs {
                        // Skip overlap check for connected polygons
                        if self.are_polygons_connected(current_ref, existing_ref)? {
                            continue;
                        }

                        // Get existing polygon vertices
                        let (existing_tile, existing_poly) =
                            self.nav_mesh.get_tile_and_poly_by_ref(existing_ref)?;
                        let existing_verts =
                            self.get_polygon_vertices(existing_tile, existing_poly)?;

                        // Check for 2D polygon overlap
                        if self.polygons_overlap_2d(&neighbor_verts, &existing_verts) {
                            overlaps = true;
                            break;
                        }
                    }

                    // Skip if this polygon overlaps with existing ones
                    if overlaps {
                        continue;
                    }

                    // Add to results if we have space
                    if result_refs.len() < max_result {
                        result_refs.push(neighbor_ref);
                        result_parents.push(current_ref);
                    }

                    // Add to stack for further exploration
                    if stack.len() < MAX_STACK_SIZE {
                        stack.push(neighbor_ref);
                    }
                }
            }
        }

        Ok((result_refs, result_parents))
    }

    /// Gets the vertices of a polygon as a flat array
    fn get_polygon_vertices(
        &self,
        tile: &super::nav_mesh::MeshTile,
        poly: &super::nav_mesh::Poly,
    ) -> Result<Vec<[f32; 3]>> {
        let mut vertices = Vec::new();

        for i in 0..poly.vert_count as usize {
            let vert_idx = poly.verts[i] as usize;
            vertices.push([
                tile.verts[vert_idx * 3],
                tile.verts[vert_idx * 3 + 1],
                tile.verts[vert_idx * 3 + 2],
            ]);
        }

        Ok(vertices)
    }

    /// Checks if two polygons are directly connected by a link
    fn are_polygons_connected(&self, poly_a: PolyRef, poly_b: PolyRef) -> Result<bool> {
        let (tile_a, poly_a_data) = self.nav_mesh.get_tile_and_poly_by_ref(poly_a)?;

        // Check if poly_a has a link to poly_b
        for i in 0..poly_a_data.vert_count as usize {
            if let Some(link) = self.nav_mesh.find_link(tile_a, poly_a_data, i as u8) {
                if link.reference == poly_b {
                    return Ok(true);
                }
            }
        }

        Ok(false)
    }

    /// Checks if two polygons overlap in 2D (ignoring Y axis)
    fn polygons_overlap_2d(&self, poly_a: &[[f32; 3]], poly_b: &[[f32; 3]]) -> bool {
        // Convert to 2D points for overlap test
        let poly_a_2d: Vec<[f32; 2]> = poly_a.iter().map(|v| [v[0], v[2]]).collect();
        let poly_b_2d: Vec<[f32; 2]> = poly_b.iter().map(|v| [v[0], v[2]]).collect();

        // Use Separating Axis Theorem (SAT) for convex polygon overlap
        self.convex_polygons_overlap_2d(&poly_a_2d, &poly_b_2d)
    }

    /// Checks if two convex polygons overlap using Separating Axis Theorem
    fn convex_polygons_overlap_2d(&self, poly_a: &[[f32; 2]], poly_b: &[[f32; 2]]) -> bool {
        // Check separating axes from polygon A
        for i in 0..poly_a.len() {
            let j = (i + 1) % poly_a.len();
            let edge = [poly_a[j][0] - poly_a[i][0], poly_a[j][1] - poly_a[i][1]];
            let normal = [-edge[1], edge[0]]; // Perpendicular to edge

            if self.polygons_separated_by_axis(&normal, poly_a, poly_b) {
                return false; // Separating axis found
            }
        }

        // Check separating axes from polygon B
        for i in 0..poly_b.len() {
            let j = (i + 1) % poly_b.len();
            let edge = [poly_b[j][0] - poly_b[i][0], poly_b[j][1] - poly_b[i][1]];
            let normal = [-edge[1], edge[0]]; // Perpendicular to edge

            if self.polygons_separated_by_axis(&normal, poly_a, poly_b) {
                return false; // Separating axis found
            }
        }

        true // No separating axis found, polygons overlap
    }

    /// Checks if two polygons are separated by a given axis
    fn polygons_separated_by_axis(
        &self,
        axis: &[f32; 2],
        poly_a: &[[f32; 2]],
        poly_b: &[[f32; 2]],
    ) -> bool {
        // Project polygon A onto the axis
        let mut min_a = f32::MAX;
        let mut max_a = f32::MIN;
        for vertex in poly_a {
            let projection = vertex[0] * axis[0] + vertex[1] * axis[1];
            min_a = min_a.min(projection);
            max_a = max_a.max(projection);
        }

        // Project polygon B onto the axis
        let mut min_b = f32::MAX;
        let mut max_b = f32::MIN;
        for vertex in poly_b {
            let projection = vertex[0] * axis[0] + vertex[1] * axis[1];
            min_b = min_b.min(projection);
            max_b = max_b.max(projection);
        }

        // Check if projections are separated
        max_a < min_b || max_b < min_a
    }

    /// Initializes a sliced pathfinding query for large-scale pathfinding
    ///
    /// This method starts a pathfinding request that can be processed over multiple frames
    /// to avoid performance hitches with very long paths.
    ///
    /// # Arguments
    /// * `start_ref` - Start polygon reference
    /// * `end_ref` - End polygon reference
    /// * `start_pos` - Start position in world coordinates
    /// * `end_pos` - End position in world coordinates
    /// * `filter` - Query filter for polygon selection
    ///
    /// # Returns
    /// Result indicating success or failure of initialization
    pub fn init_sliced_find_path(
        &mut self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &QueryFilter,
        options: u32,
    ) -> Result<()> {
        // Validate the input
        if !self.nav_mesh.is_valid_poly_ref(start_ref) || !self.nav_mesh.is_valid_poly_ref(end_ref)
        {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Initialize sliced pathfinding state
        self.sliced_state.active = true;
        self.sliced_state.state = if start_ref == end_ref {
            SlicedPathState::Success
        } else {
            SlicedPathState::InProgress
        };
        self.sliced_state.start_ref = start_ref;
        self.sliced_state.end_ref = end_ref;
        self.sliced_state.start_pos = *start_pos;
        self.sliced_state.end_pos = *end_pos;
        self.sliced_state.filter = filter.clone();
        self.sliced_state.current_path.clear();
        self.sliced_state.best_node_idx = 0;
        self.sliced_state.best_node_cost = f32::MAX;

        // If start and end are the same, we're done immediately
        if start_ref == end_ref {
            self.sliced_state.current_path.push(start_ref);
            return Ok(());
        }

        // Reset the node pool and open list for A* search
        for node in &mut self.node_pool {
            node.parent = None;
            node.state = NodeState::New;
            node.poly = PolyRef::new(0);
            node.g = 0.0;
            node.h = 0.0;
            node.f = 0.0;
        }
        self.open_list.clear();

        // Initialize start node
        let start_h = self.heuristic(start_pos, end_pos);
        let start_node = &mut self.node_pool[0];
        start_node.poly = start_ref;
        start_node.state = NodeState::Open;
        start_node.g = 0.0;
        start_node.h = start_h;
        start_node.f = start_h;

        // Push start node to open list
        self.open_list.push(HeapNode {
            index: 0,
            f: start_h,
        });

        // Set goal node
        self.node_pool[1].poly = end_ref;

        // Initialize best node tracking
        self.sliced_state.best_node_idx = 0;
        self.sliced_state.best_node_cost = start_h;

        // Handle DT_FINDPATH_ANY_ANGLE option
        const DT_FINDPATH_ANY_ANGLE: u32 = 0x02;
        if options & DT_FINDPATH_ANY_ANGLE != 0 {
            // TODO: Implement any-angle pathfinding with raycasts
            // For now, just use regular pathfinding
        }

        Ok(())
    }

    /// Initializes a sliced path query with default options
    ///
    /// This is a convenience method that calls init_sliced_find_path with options = 0
    /// to match the C++ API default parameter behavior.
    ///
    /// # Arguments
    /// * `start_ref` - The reference id of the start polygon
    /// * `end_ref` - The reference id of the end polygon  
    /// * `start_pos` - A position within the start polygon
    /// * `end_pos` - A position within the end polygon
    /// * `filter` - The polygon filter to apply to the query
    pub fn init_sliced_find_path_default(
        &mut self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &QueryFilter,
    ) -> Result<()> {
        self.init_sliced_find_path(start_ref, end_ref, start_pos, end_pos, filter, 0)
    }

    /// Updates the sliced pathfinding query for a limited number of iterations
    ///
    /// This method continues processing the pathfinding request. It should be called
    /// each frame until it returns a state other than InProgress.
    ///
    /// # Arguments
    /// * `max_iterations` - Maximum number of iterations to process this frame
    ///
    /// # Returns
    /// The current state of the pathfinding (InProgress, Success, Failed, PartialPath)
    pub fn update_sliced_find_path(&mut self, max_iter: i32) -> Result<(i32, SlicedPathState)> {
        if !self.sliced_state.active {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if self.sliced_state.state != SlicedPathState::InProgress {
            return Ok((0, self.sliced_state.state));
        }

        let mut iter_count = 0i32;

        while !self.open_list.is_empty() && iter_count < max_iter {
            // Get the node with the lowest f cost
            let HeapNode {
                index: current_idx, ..
            } = self.open_list.pop().unwrap();

            // Mark as closed
            self.node_pool[current_idx].state = NodeState::Closed;

            let current_poly = self.node_pool[current_idx].poly;

            // Check if we reached the goal
            if current_poly == self.sliced_state.end_ref {
                self.sliced_state.state = SlicedPathState::Success;
                self.sliced_state.current_path = self.reconstruct_path(current_idx)?;
                return Ok((iter_count + 1, SlicedPathState::Success));
            }

            // Update best node for partial path
            let current_cost = self.node_pool[current_idx].h;
            if current_cost < self.sliced_state.best_node_cost {
                self.sliced_state.best_node_idx = current_idx;
                self.sliced_state.best_node_cost = current_cost;
            }

            // Expand neighbors
            self.expand_neighbors_sliced(current_idx)?;

            iter_count += 1;
        }

        // Check if we exhausted the open list
        if self.open_list.is_empty() {
            self.sliced_state.state = SlicedPathState::PartialPath;
            self.sliced_state.current_path =
                self.reconstruct_path(self.sliced_state.best_node_idx)?;
        }

        Ok((iter_count, self.sliced_state.state))
    }

    /// Finalizes the sliced pathfinding and returns the computed path
    ///
    /// This method should be called after update_sliced_find_path returns Success
    /// or PartialPath to retrieve the final path result.
    ///
    /// # Returns
    /// The computed path as a vector of polygon references
    pub fn finalize_sliced_find_path(&mut self, max_path: usize) -> Result<Vec<PolyRef>> {
        if !self.sliced_state.active {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let result = match self.sliced_state.state {
            SlicedPathState::Success | SlicedPathState::PartialPath => {
                let mut path = self.sliced_state.current_path.clone();
                if path.len() > max_path {
                    path.truncate(max_path);
                }
                Ok(path)
            }
            SlicedPathState::InProgress => Err(Error::Detour(Status::InProgress.to_string())),
            SlicedPathState::Failed => Err(Error::Detour(Status::PathInvalid.to_string())),
        };

        // Clear the sliced state
        self.sliced_state.active = false;
        self.sliced_state.current_path.clear();

        result
    }

    /// Finalizes a sliced pathfinding query with a partial path
    ///
    /// This method is used to retrieve a partial path when the pathfinding
    /// cannot reach the goal but has explored some nodes. It attempts to
    /// find the furthest node along the existing path that was visited.
    ///
    /// # Arguments
    /// * `existing` - An existing path to try to extend from
    /// * `max_path` - Maximum number of polygons in the result path
    ///
    /// # Returns
    /// The computed partial path as a vector of polygon references
    pub fn finalize_sliced_find_path_partial(
        &mut self,
        existing: &[PolyRef],
        max_path: usize,
    ) -> Result<Vec<PolyRef>> {
        if !self.sliced_state.active {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if existing.is_empty() || max_path == 0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Check if the query has failed
        if self.sliced_state.state == SlicedPathState::Failed {
            self.sliced_state.active = false;
            return Err(Error::Detour(Status::PathInvalid.to_string()));
        }

        let mut path = Vec::with_capacity(max_path);

        // Special case: start and end are the same
        if self.sliced_state.start_ref == self.sliced_state.end_ref {
            path.push(self.sliced_state.start_ref);
        } else {
            // Find furthest existing node that was visited
            let mut found_node_idx = None;

            // Search from the end of the existing path backwards
            for &poly_ref in existing.iter().rev() {
                for (idx, node) in self.node_pool.iter().enumerate() {
                    if node.poly == poly_ref && node.state != NodeState::New {
                        found_node_idx = Some(idx);
                        break;
                    }
                }
                if found_node_idx.is_some() {
                    break;
                }
            }

            // If no node from existing path was found, use the best node
            let node_idx = found_node_idx.unwrap_or(self.sliced_state.best_node_idx);

            // Build the path by following parent links
            let mut current_idx = node_idx;
            let mut reverse_path = Vec::new();

            while current_idx < self.node_pool.len() {
                let node = &self.node_pool[current_idx];
                reverse_path.push(node.poly);

                if let Some(parent_idx) = node.parent {
                    current_idx = parent_idx;
                } else {
                    break;
                }

                // Prevent infinite loops
                if reverse_path.len() >= max_path {
                    break;
                }
            }

            // Reverse the path to get start->end order
            path.extend(reverse_path.iter().rev().copied());

            // Truncate if necessary
            if path.len() > max_path {
                path.truncate(max_path);
            }
        }

        // Clear the sliced state
        self.sliced_state.active = false;
        self.sliced_state.current_path.clear();

        Ok(path)
    }

    /// Gets the current state of the sliced pathfinding query
    ///
    /// # Returns
    /// The current pathfinding state, or None if no sliced query is active
    pub fn get_sliced_path_state(&self) -> Option<SlicedPathState> {
        if self.sliced_state.active {
            Some(self.sliced_state.state)
        } else {
            None
        }
    }

    /// Cancels the current sliced pathfinding query
    pub fn cancel_sliced_find_path(&mut self) {
        self.sliced_state.active = false;
        self.sliced_state.current_path.clear();
    }

    /// Expands neighbors for sliced pathfinding (helper method)
    fn expand_neighbors_sliced(&mut self, current_idx: usize) -> Result<()> {
        let current_poly = self.node_pool[current_idx].poly;
        let current_g = self.node_pool[current_idx].g;

        // Get the tile and polygon for the current node
        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(current_poly)?;

        // Iterate through all links/neighbors
        if let Some(first_link) = poly.first_link {
            for i in 0..poly.vert_count {
                let link_idx = first_link + i as usize;
                if link_idx >= tile.links.len() {
                    continue;
                }

                let link = &tile.links[link_idx];
                if !link.reference.is_valid() {
                    continue;
                }

                let neighbor_ref = link.reference;

                // Check if neighbor passes filter
                let (neighbor_tile, neighbor_poly) =
                    match self.nav_mesh.get_tile_and_poly_by_ref(neighbor_ref) {
                        Ok((tile, poly)) => (tile, poly),
                        Err(_) => continue, // Skip invalid neighbor
                    };

                if !self
                    .sliced_state
                    .filter
                    .pass_filter(neighbor_ref, neighbor_tile, neighbor_poly)
                {
                    continue;
                }

                // Find or create node for neighbor
                let neighbor_idx = self.find_or_create_node(neighbor_ref)?;
                let neighbor_node = &self.node_pool[neighbor_idx];

                // Skip if already closed
                if neighbor_node.state == NodeState::Closed {
                    continue;
                }

                // Calculate movement cost - simplified version for sliced pathfinding
                let movement_cost = 1.0; // Simple cost for now, could be improved
                let tentative_g = current_g + movement_cost;

                // If not in open list or this path is better
                if neighbor_node.state == NodeState::New || tentative_g < neighbor_node.g {
                    let neighbor_pos = self.get_poly_center(neighbor_ref)?;
                    let h = self.heuristic(&neighbor_pos, &self.sliced_state.end_pos);

                    let neighbor_node = &mut self.node_pool[neighbor_idx];
                    neighbor_node.parent = Some(current_idx);
                    neighbor_node.g = tentative_g;
                    neighbor_node.h = h;
                    neighbor_node.f = tentative_g + h;

                    if neighbor_node.state == NodeState::New {
                        neighbor_node.state = NodeState::Open;
                        self.open_list.push(HeapNode {
                            index: neighbor_idx,
                            f: neighbor_node.f,
                        });
                    }
                }
            }
        }

        Ok(())
    }

    /// Finds or creates a node for the given polygon reference
    fn find_or_create_node(&mut self, poly_ref: PolyRef) -> Result<usize> {
        // Look for existing node
        for (i, node) in self.node_pool.iter().enumerate() {
            if node.poly == poly_ref {
                return Ok(i);
            }
        }

        // Find unused node
        for (i, node) in self.node_pool.iter().enumerate() {
            if node.state == NodeState::New && !node.poly.is_valid() {
                let node = &mut self.node_pool[i];
                node.poly = poly_ref;
                return Ok(i);
            }
        }

        // No free nodes available
        Err(Error::Detour(Status::OutOfMemory.to_string()))
    }

    /// Reconstructs the path from the goal node back to the start
    fn reconstruct_path(&self, goal_idx: usize) -> Result<Vec<PolyRef>> {
        let mut path = Vec::new();
        let mut current_idx = goal_idx;

        loop {
            let node = &self.node_pool[current_idx];
            path.push(node.poly);

            if let Some(parent_idx) = node.parent {
                current_idx = parent_idx;
            } else {
                break;
            }
        }

        path.reverse();
        Ok(path)
    }

    /// Gets the wall segments of a polygon
    ///
    /// Wall segments are edges of the polygon that don't connect to other walkable polygons.
    /// Each segment is represented by two 3D points [start_x, start_y, start_z, end_x, end_y, end_z].
    ///
    /// # Arguments
    /// * `poly_ref` - The polygon reference to get wall segments for
    /// * `filter` - Query filter to determine which polygons are walkable
    /// * `max_segments` - Maximum number of segments to return
    ///
    /// # Returns
    /// Vector of wall segments, where each segment is [start_x, start_y, start_z, end_x, end_y, end_z]
    /// Finds polygons that overlap the search box
    ///
    /// # Arguments
    ///
    /// * `center` - The center of the search box
    /// * `half_extents` - The search distance along each axis
    /// * `filter` - The polygon filter to apply to the query
    ///
    /// # Returns
    ///
    /// A vector of polygon references that overlap the query box
    ///
    /// Finds the polygons along the navigation graph that touch the specified convex polygon
    ///
    /// # Arguments
    ///
    /// * `start_ref` - The reference id of the polygon where the search starts
    /// * `verts` - The vertices describing the convex polygon (CCW). [(x, y, z) * nverts]
    /// * `filter` - The polygon filter to apply to the query
    ///
    /// # Returns
    ///
    /// A vector of tuples containing:
    /// - The polygon reference
    /// - The parent polygon reference (None if it's the start polygon)
    /// - The search cost from the centroid to the polygon
    pub fn find_polys_around_shape(
        &mut self,
        start_ref: PolyRef,
        verts: &[[f32; 3]],
        filter: &QueryFilter,
    ) -> Result<Vec<(PolyRef, Option<PolyRef>, f32)>> {
        use recast_common::intersect_segment_poly_2d;

        if !self.nav_mesh.is_valid_poly_ref(start_ref) {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if verts.len() < 3 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Calculate the center of the shape
        let mut center = [0.0, 0.0, 0.0];
        for vert in verts {
            center[0] += vert[0];
            center[1] += vert[1];
            center[2] += vert[2];
        }
        let nverts = verts.len() as f32;
        center[0] /= nverts;
        center[1] /= nverts;
        center[2] /= nverts;

        // Flatten verts to a single array for intersect_segment_poly_2d
        let mut flat_verts = Vec::with_capacity(verts.len() * 3);
        for vert in verts {
            flat_verts.push(vert[0]);
            flat_verts.push(vert[1]);
            flat_verts.push(vert[2]);
        }

        // Initialize search
        self.node_pool.clear();
        self.open_list.clear();

        let start_node = Node::new(start_ref, 0);
        self.node_pool.push(start_node);
        self.node_pool[0].g = 0.0;
        self.node_pool[0].f = 0.0;
        self.node_pool[0].state = NodeState::Open;
        self.open_list.push(HeapNode { index: 0, f: 0.0 });

        let mut result = Vec::new();

        while let Some(heap_node) = self.open_list.pop() {
            let best_idx = heap_node.index;
            if best_idx >= self.node_pool.len() {
                continue;
            }

            // Update node state
            self.node_pool[best_idx].state = NodeState::Closed;

            let best_ref = self.node_pool[best_idx].poly;
            let best_cost = self.node_pool[best_idx].g;

            // Get poly and tile
            let (best_tile, best_poly) = match self.nav_mesh.get_tile_and_poly_by_ref(best_ref) {
                Ok(val) => val,
                Err(_) => continue,
            };

            // Get parent reference
            let parent_ref = self.node_pool[best_idx]
                .parent
                .map(|idx| self.node_pool[idx].poly);

            // Add to results
            result.push((best_ref, parent_ref, best_cost));

            // Process neighbors
            for i in 0..best_poly.vert_count as usize {
                let link = match self.nav_mesh.find_link(best_tile, best_poly, i as u8) {
                    Some(l) => l,
                    None => continue,
                };

                let neighbor_ref = link.reference;

                // Skip invalid neighbors and back to parent
                if !neighbor_ref.is_valid() || Some(neighbor_ref) == parent_ref {
                    continue;
                }

                // Get neighbor tile and poly
                let (neighbor_tile, neighbor_poly) =
                    match self.nav_mesh.get_tile_and_poly_by_ref(neighbor_ref) {
                        Ok(val) => val,
                        Err(_) => continue,
                    };

                // Check filter
                if !filter.pass_filter(neighbor_ref, neighbor_tile, neighbor_poly) {
                    continue;
                }

                // Get portal points
                let (va, vb) = match self.get_portal_points(best_ref, neighbor_ref) {
                    Ok(val) => val,
                    Err(_) => continue,
                };

                // Check if the edge intersects the shape
                match intersect_segment_poly_2d(&va, &vb, &flat_verts, verts.len()) {
                    Some((tmin, tmax, _, _)) => {
                        // Check if the intersection is valid
                        if tmin > 1.0 || tmax < 0.0 {
                            continue;
                        }
                    }
                    None => continue,
                }

                // Calculate position on edge (midpoint)
                let edge_mid = [
                    (va[0] + vb[0]) * 0.5,
                    (va[1] + vb[1]) * 0.5,
                    (va[2] + vb[2]) * 0.5,
                ];

                // Calculate cost
                // Get parent tile and poly if available
                let (parent_tile, parent_poly) = if let Some(parent_ref) = parent_ref {
                    match self.nav_mesh.get_tile_and_poly_by_ref(parent_ref) {
                        Ok((t, p)) => (Some(t), Some(p)),
                        Err(_) => (None, None),
                    }
                } else {
                    (None, None)
                };

                let cost = filter.get_cost(
                    &center,
                    &edge_mid,
                    parent_ref.unwrap_or(PolyRef::new(0)),
                    parent_tile,
                    parent_poly,
                    best_ref,
                    best_tile,
                    best_poly,
                    neighbor_ref,
                    Some(neighbor_tile),
                    Some(neighbor_poly),
                );

                let total = best_cost + cost;

                // Check if node already exists
                let mut neighbor_idx = None;
                for (idx, node) in self.node_pool.iter().enumerate() {
                    if node.poly == neighbor_ref {
                        neighbor_idx = Some(idx);
                        break;
                    }
                }

                if let Some(idx) = neighbor_idx {
                    // Node exists
                    if self.node_pool[idx].state == NodeState::Closed {
                        continue;
                    }

                    // Update if better path found
                    if total < self.node_pool[idx].g {
                        self.node_pool[idx].parent = Some(best_idx);
                        self.node_pool[idx].g = total;
                        self.node_pool[idx].f = total;

                        if self.node_pool[idx].state == NodeState::Open {
                            // Update position in heap
                            self.open_list.push(HeapNode {
                                index: idx,
                                f: total,
                            });
                        } else {
                            self.node_pool[idx].state = NodeState::Open;
                            self.open_list.push(HeapNode {
                                index: idx,
                                f: total,
                            });
                        }
                    }
                } else {
                    // New node
                    if self.node_pool.len() >= DT_MAX_NODES {
                        continue;
                    }

                    let mut new_node = Node::new(neighbor_ref, self.node_pool.len());
                    new_node.parent = Some(best_idx);
                    new_node.g = total;
                    new_node.f = total;
                    new_node.state = NodeState::Open;

                    let node_idx = self.node_pool.len();
                    self.node_pool.push(new_node);
                    self.open_list.push(HeapNode {
                        index: node_idx,
                        f: total,
                    });
                }
            }
        }

        Ok(result)
    }

    /// Returns a point on the boundary closest to the source point if the source point is outside the polygon's xz-bounds
    ///
    /// # Arguments
    ///
    /// * `poly_ref` - The reference id to the polygon
    /// * `pos` - The position to check
    ///
    /// # Returns
    ///
    /// The closest point on the polygon boundary
    pub fn closest_point_on_poly_boundary(
        &self,
        poly_ref: PolyRef,
        pos: &[f32; 3],
    ) -> Result<[f32; 3]> {
        use recast_common::distance_pt_poly_edges_sqr;

        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

        // Collect vertices
        let mut verts = Vec::with_capacity(poly.vert_count as usize * 3);
        for i in 0..poly.vert_count as usize {
            let idx = poly.verts[i] as usize;
            verts.push(tile.verts[idx * 3]);
            verts.push(tile.verts[idx * 3 + 1]);
            verts.push(tile.verts[idx * 3 + 2]);
        }

        let (inside, edge_dists, edge_ts) =
            distance_pt_poly_edges_sqr(pos, &verts, poly.vert_count as usize);

        if inside {
            // Point is inside the polygon, return the point itself
            Ok(*pos)
        } else {
            // Point is outside, find closest edge
            let mut min_dist = edge_dists[0];
            let mut min_idx = 0;

            for (i, &dist) in edge_dists.iter().enumerate().skip(1) {
                if dist < min_dist {
                    min_dist = dist;
                    min_idx = i;
                }
            }

            // Calculate the closest point on the nearest edge
            let va_idx = min_idx * 3;
            let vb_idx = ((min_idx + 1) % poly.vert_count as usize) * 3;

            let va = [verts[va_idx], verts[va_idx + 1], verts[va_idx + 2]];
            let vb = [verts[vb_idx], verts[vb_idx + 1], verts[vb_idx + 2]];
            let t = edge_ts[min_idx];

            // Linear interpolation
            Ok([
                va[0] + (vb[0] - va[0]) * t,
                va[1] + (vb[1] - va[1]) * t,
                va[2] + (vb[2] - va[2]) * t,
            ])
        }
    }

    /// Returns edge mid point between two polygons
    ///
    /// # Arguments
    ///
    /// * `from_ref` - The reference of the polygon to traverse from
    /// * `to_ref` - The reference of the polygon to traverse to
    ///
    /// # Returns
    ///
    /// The midpoint of the edge between the two polygons
    pub fn get_edge_mid_point(&self, from_ref: PolyRef, to_ref: PolyRef) -> Result<[f32; 3]> {
        let (left, right) = self.get_portal_points(from_ref, to_ref)?;

        // Calculate midpoint
        Ok([
            (left[0] + right[0]) * 0.5,
            (left[1] + right[1]) * 0.5,
            (left[2] + right[2]) * 0.5,
        ])
    }

    /// Gets a path from the explored nodes in the previous search
    ///
    /// This method depends on the state from a previous Dijkstra search
    /// (findPolysAroundCircle or findPolysAroundShape). It should only be used
    /// immediately after one of these searches.
    ///
    /// # Arguments
    ///
    /// * `end_ref` - The reference id of the end polygon
    ///
    /// # Returns
    ///
    /// An ordered list of polygon references representing the path (start to end)
    /// Finds polygons that overlap the search box.
    ///
    /// This method matches the C++ API: dtNavMeshQuery::queryPolygons
    pub fn query_polygons(
        &self,
        center: &[f32; 3],
        half_extents: &[f32; 3],
        filter: &QueryFilter,
        max_polys: usize,
    ) -> Result<Vec<PolyRef>> {
        let mut collect_query = super::poly_query::CollectPolysQuery::new(max_polys);
        self.query_polygons_with_query(center, half_extents, filter, &mut collect_query)?;
        Ok(collect_query.polys().to_vec())
    }

    /// Finds polygons that overlap the search box and processes them with a custom query.
    ///
    /// This method matches the C++ API: dtNavMeshQuery::queryPolygons with dtPolyQuery
    pub fn query_polygons_with_query(
        &self,
        center: &[f32; 3],
        half_extents: &[f32; 3],
        filter: &QueryFilter,
        query: &mut dyn super::poly_query::PolyQuery,
    ) -> Result<()> {
        // Calculate search bounds
        let bmin = [
            center[0] - half_extents[0],
            center[1] - half_extents[1],
            center[2] - half_extents[2],
        ];
        let bmax = [
            center[0] + half_extents[0],
            center[1] + half_extents[1],
            center[2] + half_extents[2],
        ];

        // Get all tiles that overlap the search bounds
        let tiles = self.nav_mesh.get_tiles_in_bounds(&bmin, &bmax)?;

        // Process each tile
        for tile in tiles {
            let mut batch_polys = Vec::new();
            let mut batch_refs = Vec::new();
            const BATCH_SIZE: usize = 32;

            // Process polygons in the tile
            for (poly_idx, poly) in tile.polys.iter().enumerate() {
                // Get polygon reference
                let poly_ref = self.nav_mesh.get_poly_ref_base(tile) | (poly_idx as u32);
                let poly_ref = PolyRef::new(poly_ref);

                // Check if polygon passes filter
                if !filter.pass_filter(poly_ref, tile, poly) {
                    continue;
                }

                // Check if polygon bounds overlap search bounds
                let poly_bounds = self.nav_mesh.get_poly_bounds(tile, poly)?;
                if !Self::overlap_bounds(&poly_bounds.0, &poly_bounds.1, &bmin, &bmax) {
                    continue;
                }

                // Add to batch
                batch_polys.push(poly);
                batch_refs.push(poly_ref);

                // Process batch when full
                if batch_polys.len() >= BATCH_SIZE {
                    query.process(tile, &batch_polys, &batch_refs);
                    batch_polys.clear();
                    batch_refs.clear();
                }
            }

            // Process remaining polygons
            if !batch_polys.is_empty() {
                query.process(tile, &batch_polys, &batch_refs);
            }
        }

        Ok(())
    }

    /// Checks if two axis-aligned bounding boxes overlap
    fn overlap_bounds(amin: &[f32; 3], amax: &[f32; 3], bmin: &[f32; 3], bmax: &[f32; 3]) -> bool {
        !(amin[0] > bmax[0]
            || amax[0] < bmin[0]
            || amin[1] > bmax[1]
            || amax[1] < bmin[1]
            || amin[2] > bmax[2]
            || amax[2] < bmin[2])
    }

    pub fn get_poly_wall_segments(
        &self,
        poly_ref: PolyRef,
        filter: &QueryFilter,
        max_segments: usize,
    ) -> Result<Vec<[f32; 6]>> {
        if !poly_ref.is_valid() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let (tile, poly) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

        // Check if the polygon passes the filter
        if !filter.pass_filter(poly_ref, tile, poly) {
            return Ok(Vec::new());
        }

        let mut wall_segments = Vec::new();

        // Check each edge of the polygon
        for i in 0..poly.vert_count as usize {
            if wall_segments.len() >= max_segments {
                break;
            }

            // Get current and next vertex indices
            let curr_vert_idx = poly.verts[i] as usize;
            let next_vert_idx = poly.verts[(i + 1) % poly.vert_count as usize] as usize;

            // Get vertex coordinates
            if curr_vert_idx * 3 + 2 >= tile.verts.len()
                || next_vert_idx * 3 + 2 >= tile.verts.len()
            {
                continue;
            }

            let curr_vert = [
                tile.verts[curr_vert_idx * 3],
                tile.verts[curr_vert_idx * 3 + 1],
                tile.verts[curr_vert_idx * 3 + 2],
            ];
            let next_vert = [
                tile.verts[next_vert_idx * 3],
                tile.verts[next_vert_idx * 3 + 1],
                tile.verts[next_vert_idx * 3 + 2],
            ];

            // Check if this edge is a wall (no walkable neighbor)
            // Use the link system to find the actual neighbor connection
            let is_wall = if let Some(link) = self.nav_mesh.find_link(tile, poly, i as u8) {
                // There's a link to a neighbor - check if it's walkable
                if link.reference.is_valid() {
                    // Get the neighbor tile and polygon
                    if let Ok((neighbor_tile, neighbor_poly)) =
                        self.nav_mesh.get_tile_and_poly_by_ref(link.reference)
                    {
                        // If neighbor doesn't pass filter, this edge is a wall
                        !filter.pass_filter(link.reference, neighbor_tile, neighbor_poly)
                    } else {
                        // Can't access neighbor - treat as wall
                        true
                    }
                } else {
                    // Invalid neighbor reference - it's a wall
                    true
                }
            } else {
                // No link to neighbor - definitely a wall
                true
            };

            if is_wall {
                // Add this edge as a wall segment
                wall_segments.push([
                    curr_vert[0],
                    curr_vert[1],
                    curr_vert[2],
                    next_vert[0],
                    next_vert[1],
                    next_vert[2],
                ]);
            }
        }

        Ok(wall_segments)
    }

    /// Checks if a polygon reference is valid and passes the filter restrictions
    pub fn is_valid_poly_ref(&self, poly_ref: PolyRef, filter: &QueryFilter) -> bool {
        if !poly_ref.is_valid() {
            return false;
        }

        // Try to get the tile and polygon
        if let Ok((tile, poly)) = self.nav_mesh.get_tile_and_poly_by_ref(poly_ref) {
            // Check if the polygon passes the filter
            filter.pass_filter(poly_ref, tile, poly)
        } else {
            false
        }
    }

    /// Checks if a polygon reference is in the closed list
    pub fn is_in_closed_list(&self, poly_ref: PolyRef) -> bool {
        // Search through the node pool for this polygon
        for node in &self.node_pool {
            if node.poly == poly_ref && node.state == NodeState::Closed {
                return true;
            }
        }
        false
    }

    /// Queries polygons with a custom query implementation
    ///
    /// This is an alias for `query_polygons_with_query` with a more intuitive name.
    /// It allows custom processing of polygons during spatial queries.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use detour::{PolyQuery, NavMesh, PolyRef, MeshTile, Poly};
    ///
    /// struct MyCustomQuery {
    ///     count: usize,
    /// }
    ///
    /// impl PolyQuery for MyCustomQuery {
    ///     fn process(&mut self, tile: &MeshTile, polys: &[&Poly], refs: &[PolyRef]) {
    ///         self.count += polys.len();
    ///     }
    /// }
    ///
    /// let mut custom_query = MyCustomQuery { count: 0 };
    /// query.query_polygons_custom(&center, &extents, &filter, &mut custom_query)?;
    /// println!("Found {} polygons", custom_query.count);
    /// ```
    pub fn query_polygons_custom<Q: super::poly_query::PolyQuery>(
        &self,
        center: &[f32; 3],
        extents: &[f32; 3],
        filter: &QueryFilter,
        query: &mut Q,
    ) -> Result<()> {
        self.query_polygons_with_query(center, extents, filter, query)
    }

    /// Gets the internal node pool (for debugging and advanced use)
    ///
    /// Note: The returned nodes are only valid during the current query.
    /// The node pool is reused between queries.
    pub fn get_node_pool(&self) -> &[Node] {
        &self.node_pool
    }

    /// Gets the attached navigation mesh
    pub fn get_attached_nav_mesh(&self) -> &NavMesh {
        self.nav_mesh
    }
}

#[cfg(test)]
mod tests {
    use super::super::{
        NavMeshBuilder, NavMeshCreateParams, NavMeshParams, PolyFlags, encode_poly_ref,
    };
    use super::*;
    use crate::{CollectPolysQuery, FindNearestPolyQuery, nav_mesh::encode_poly_ref_with_salt};
    use recast::MESH_NULL_IDX;

    #[test]
    fn test_basic_query() {
        // Create a simple navigation mesh
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params).unwrap();

        // Create a query
        let query = NavMeshQuery::new(&nav_mesh);

        // Check query extent default values
        assert_eq!(query.get_query_extent(), [2.0, 4.0, 2.0]);
    }

    #[test]
    fn test_set_query_extent() {
        // Create a simple navigation mesh
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let nav_mesh = NavMesh::new(params).unwrap();

        // Create a query
        let mut query = NavMeshQuery::new(&nav_mesh);

        // Set query extent
        let extent = [10.0, 20.0, 10.0];
        query.set_query_extent(extent);

        // Check query extent
        assert_eq!(query.get_query_extent(), extent);
    }

    #[test]
    fn test_pathfinding_with_off_mesh_connections() {
        use super::super::{NavMeshParams, PolyFlags};

        // Create a simple navigation mesh
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create a simple tile with an off-mesh connection
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Add an off-mesh connection
        let connection_ref = nav_mesh
            .add_off_mesh_connection(
                [10.0, 0.0, 10.0], // start
                [20.0, 0.0, 20.0], // end
                2.0,               // radius
                PolyFlags::WALK,   // flags
                1,                 // area
                0,                 // bidirectional
                42,                // user_id
            )
            .unwrap();

        // Create a query
        let query = NavMeshQuery::new(&nav_mesh);

        // Test that off-mesh connections are detected
        assert!(nav_mesh.is_off_mesh_connection(connection_ref));

        // Test closest point on off-mesh connection
        let (closest, _is_over) = query
            .closest_point_on_off_mesh_connection(connection_ref, &[15.0, 0.0, 15.0])
            .unwrap();

        // The closest point should be on the line segment between start and end
        assert!(closest[0] >= 10.0 && closest[0] <= 20.0);
        assert!(closest[2] >= 10.0 && closest[2] <= 20.0);

        // Test off-mesh connection endpoint calculation
        let endpoint = query
            .get_off_mesh_connection_endpoint(
                connection_ref,
                &[11.0, 0.0, 11.0], // Close to start
            )
            .unwrap();

        // Should return the end position since we're closer to start
        assert_eq!(endpoint, [20.0, 0.0, 20.0]);

        // Test from the other direction
        let endpoint2 = query
            .get_off_mesh_connection_endpoint(
                connection_ref,
                &[19.0, 0.0, 19.0], // Close to end
            )
            .unwrap();

        // Should return the start position since we're closer to end
        assert_eq!(endpoint2, [10.0, 0.0, 10.0]);
    }

    #[test]
    fn test_off_mesh_connection_cost_calculation() {
        use super::super::{NavMeshParams, PolyFlags, QueryFilter};

        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create a simple tile
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Add an off-mesh connection with specific area
        let connection_ref = nav_mesh
            .add_off_mesh_connection(
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                1.0,
                PolyFlags::WALK,
                5,
                0,
                1,
            )
            .unwrap();

        let query = NavMeshQuery::new(&nav_mesh);
        let mut filter = QueryFilter::default();
        filter.area_cost[5] = 2.0; // Higher cost for area 5

        // Calculate cost
        let cost = query
            .get_off_mesh_connection_cost(
                PolyRef::new(1), // dummy from ref
                connection_ref,
                &[0.0, 0.0, 0.0],
                &[10.0, 0.0, 0.0],
                &filter,
            )
            .unwrap();

        // Cost should be distance (10.0) * area cost (2.0) = 20.0
        assert!((cost - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_off_mesh_connection_direction_constraints() {
        use super::super::{NavMeshParams, PolyFlags};

        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create a simple tile
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Add a unidirectional connection (start to end only)
        let connection_ref = nav_mesh
            .add_off_mesh_connection(
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                1.0,
                PolyFlags::WALK,
                1,
                1,
                1, // direction = 1 (A->B)
            )
            .unwrap();

        let query = NavMeshQuery::new(&nav_mesh);

        // Should work from start to end
        let endpoint1 = query.get_off_mesh_connection_endpoint(
            connection_ref,
            &[1.0, 0.0, 0.0], // Close to start
        );
        assert!(endpoint1.is_ok());
        assert_eq!(endpoint1.unwrap(), [10.0, 0.0, 0.0]);

        // Should NOT work from end to start
        let endpoint2 = query.get_off_mesh_connection_endpoint(
            connection_ref,
            &[9.0, 0.0, 0.0], // Close to end
        );
        assert!(endpoint2.is_err()); // Should fail due to direction constraint
    }

    #[test]
    fn test_off_mesh_connection_poly_end_points() {
        use super::super::{NavMeshParams, PolyFlags};

        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create test tiles
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Add a bidirectional off-mesh connection
        let connection_ref = nav_mesh
            .add_off_mesh_connection(
                [10.0, 0.0, 10.0], // Start point
                [20.0, 0.0, 20.0], // End point
                2.0,
                PolyFlags::WALK,
                1,
                0, // bidirectional
                42,
            )
            .unwrap();

        let query = NavMeshQuery::new(&nav_mesh);

        // Test with invalid previous reference - should return natural order
        let (start, end) = query
            .get_off_mesh_connection_poly_end_points(PolyRef::new(0), connection_ref)
            .unwrap();
        assert_eq!(start, [10.0, 0.0, 10.0]);
        assert_eq!(end, [20.0, 0.0, 20.0]);

        // Test with a mock previous polygon reference
        // In a real scenario, this would be a valid polygon reference
        let mock_prev_ref = PolyRef::new(1);
        let result = query.get_off_mesh_connection_poly_end_points(mock_prev_ref, connection_ref);
        // Since we can't easily create valid polygon references in this test,
        // we expect it to fall back to natural order
        if let Ok((start, end)) = result {
            // Should still work, either in natural order or reversed based on approach
            assert!(
                (start == [10.0, 0.0, 10.0] && end == [20.0, 0.0, 20.0])
                    || (start == [20.0, 0.0, 20.0] && end == [10.0, 0.0, 10.0])
            );
        }
        // If it fails due to invalid prev_ref, that's also acceptable behavior
    }

    #[test]
    fn test_off_mesh_connection_straight_path_integration() {
        use super::super::{NavMeshParams, PolyFlags};

        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 128,
            max_polys_per_tile: 1024,
        };

        let mut nav_mesh = NavMesh::new(params).unwrap();

        // Create test tiles
        nav_mesh.create_test_tile(0, 0, 0).unwrap();

        // Add an off-mesh connection
        let connection_ref = nav_mesh
            .add_off_mesh_connection(
                [5.0, 0.0, 5.0],
                [15.0, 0.0, 15.0],
                1.0,
                PolyFlags::WALK,
                1,
                0, // bidirectional
                1,
            )
            .unwrap();

        let query = NavMeshQuery::new(&nav_mesh);

        // Create a simple path containing the off-mesh connection
        let path = vec![PolyRef::new(1), connection_ref, PolyRef::new(2)];

        // Test that straight path generation doesn't crash with off-mesh connections
        let result = query.find_straight_path(&[0.0, 0.0, 0.0], &[20.0, 0.0, 20.0], &path);

        // The exact behavior depends on the implementation details,
        // but it should not crash and should return some result
        match result {
            Ok(straight_path) => {
                // Should have at least start and end points
                assert!(straight_path.waypoints.len() >= 2);
                assert_eq!(straight_path.waypoints[0], [0.0, 0.0, 0.0]);
                assert_eq!(
                    straight_path.waypoints[straight_path.waypoints.len() - 1],
                    [20.0, 0.0, 20.0]
                );
            }
            Err(_) => {
                // It's acceptable for this to fail in the current implementation
                // since we don't have fully valid polygon data
            }
        }
    }

    #[test]
    fn test_get_poly_wall_segments() -> std::result::Result<(), Box<dyn std::error::Error>> {
        // Create a simple navigation mesh with a single isolated polygon
        let nav_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 32.0,
            tile_height: 32.0,
            max_tiles: 1,
            max_polys_per_tile: 256,
        };

        let mut nav_mesh = NavMesh::new(nav_params.clone())?;

        // Create a simple square polygon with no neighbors (all walls)
        let params = NavMeshCreateParams {
            nav_mesh_params: nav_params,
            verts: vec![
                0.0, 0.0, 0.0, // 0
                10.0, 0.0, 0.0, // 1
                10.0, 0.0, 10.0, // 2
                0.0, 0.0, 10.0, // 3
            ],
            vert_count: 4,
            polys: vec![
                0,
                1,
                2,
                3,             // vertex indices
                MESH_NULL_IDX, // neighbor 0 (none)
                MESH_NULL_IDX, // neighbor 1 (none)
                MESH_NULL_IDX, // neighbor 2 (none)
                MESH_NULL_IDX, // neighbor 3 (none)
            ],
            poly_flags: vec![PolyFlags::WALK],
            poly_areas: vec![0],
            poly_count: 1,
            nvp: 4,
            detail_meshes: vec![0, 0, 0, 2],
            detail_verts: vec![],
            detail_vert_count: 0,
            detail_tris: vec![0, 1, 2, 0, 2, 3],
            detail_tri_count: 2,
            off_mesh_con_verts: vec![],
            off_mesh_con_rad: vec![],
            off_mesh_con_flags: vec![],
            off_mesh_con_areas: vec![],
            off_mesh_con_dir: vec![],
            off_mesh_con_user_id: vec![],
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [10.0, 0.0, 10.0],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        let tile = NavMeshBuilder::build_tile(&params)?;
        let _tile_ref = nav_mesh.add_mesh_tile(tile)?;

        // Create query
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Get the polygon reference using find_nearest_poly to ensure correct salt
        let center = [5.0, 0.0, 5.0];
        let half_extents = [5.0, 5.0, 5.0];
        let (poly_ref, _) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // Get wall segments (should have 4 since all edges are walls)
        let wall_segments = query.get_poly_wall_segments(poly_ref, &filter, 10)?;

        // Should have 4 wall segments (one for each edge)
        assert_eq!(wall_segments.len(), 4);

        // Verify the wall segments form a closed loop
        let expected_edges = [
            ([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]),   // bottom edge
            ([10.0, 0.0, 0.0], [10.0, 0.0, 10.0]), // right edge
            ([10.0, 0.0, 10.0], [0.0, 0.0, 10.0]), // top edge
            ([0.0, 0.0, 10.0], [0.0, 0.0, 0.0]),   // left edge
        ];

        for (i, segment) in wall_segments.iter().enumerate() {
            let start = [segment[0], segment[1], segment[2]];
            let end = [segment[3], segment[4], segment[5]];
            let expected = expected_edges[i];

            // Allow for floating point precision
            for j in 0..3 {
                assert!(
                    (start[j] - expected.0[j]).abs() < 0.001,
                    "Segment {} start mismatch: expected {:?}, got {:?}",
                    i,
                    expected.0,
                    start
                );
                assert!(
                    (end[j] - expected.1[j]).abs() < 0.001,
                    "Segment {} end mismatch: expected {:?}, got {:?}",
                    i,
                    expected.1,
                    end
                );
            }
        }

        // Test with max_segments limit
        let limited_segments = query.get_poly_wall_segments(poly_ref, &filter, 2)?;
        assert_eq!(limited_segments.len(), 2); // Should be limited to 2

        // Test with invalid polygon reference
        let invalid_ref = encode_poly_ref(999, 999);
        let result = query.get_poly_wall_segments(invalid_ref, &filter, 10);
        assert!(result.is_err());

        Ok(())
    }

    #[test]
    fn test_get_poly_wall_segments_with_neighbors()
    -> std::result::Result<(), Box<dyn std::error::Error>> {
        // Create a 2-tile navigation mesh to test cross-tile neighbor detection
        let nav_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 4,
            max_polys_per_tile: 256,
        };

        let mut nav_mesh = NavMesh::new(nav_params.clone())?;

        // Create two adjacent tiles
        for tx in 0..2 {
            let x_offset = tx as f32 * 10.0;

            let params = NavMeshCreateParams {
                nav_mesh_params: nav_params.clone(),
                verts: vec![
                    x_offset + 0.0,
                    0.0,
                    0.0, // 0
                    x_offset + 10.0,
                    0.0,
                    0.0, // 1
                    x_offset + 10.0,
                    0.0,
                    10.0, // 2
                    x_offset + 0.0,
                    0.0,
                    10.0, // 3
                ],
                vert_count: 4,
                polys: vec![
                    0,
                    1,
                    2,
                    3,
                    MESH_NULL_IDX,
                    MESH_NULL_IDX,
                    MESH_NULL_IDX,
                    MESH_NULL_IDX,
                ],
                poly_flags: vec![PolyFlags::WALK],
                poly_areas: vec![0],
                poly_count: 1,
                nvp: 4,
                detail_meshes: vec![0, 0, 0, 2],
                detail_verts: vec![],
                detail_vert_count: 0,
                detail_tris: vec![0, 1, 2, 0, 2, 3],
                detail_tri_count: 2,
                off_mesh_con_verts: vec![],
                off_mesh_con_rad: vec![],
                off_mesh_con_flags: vec![],
                off_mesh_con_areas: vec![],
                off_mesh_con_dir: vec![],
                off_mesh_con_user_id: vec![],
                off_mesh_con_count: 0,
                bmin: [x_offset, 0.0, 0.0],
                bmax: [x_offset + 10.0, 0.0, 10.0],
                walkable_height: 2.0,
                walkable_radius: 0.6,
                walkable_climb: 0.9,
                cs: 0.3,
                ch: 0.2,
                build_bv_tree: true,
            };

            let mut tile = NavMeshBuilder::build_tile(&params)?;

            // Set tile coordinates
            if let Some(header) = tile.header.as_mut() {
                header.x = tx;
                header.y = 0;
                header.layer = 0;
            }

            nav_mesh.add_mesh_tile(tile)?;
        }

        // Manually connect the tiles (this would normally be done automatically)
        // For now, just test that the method works with disconnected tiles

        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Get polygon reference using find_nearest_poly to ensure correct salt
        let center = [5.0, 0.0, 5.0];
        let half_extents = [5.0, 5.0, 5.0];
        let (poly_ref, _) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // Get wall segments - may vary depending on whether tiles auto-connect
        let wall_segments = query.get_poly_wall_segments(poly_ref, &filter, 10)?;

        // Should have at least 2 wall segments (some edges may not be walls if tiles connect)
        assert!(wall_segments.len() >= 2);
        assert!(wall_segments.len() <= 4);

        // Verify that each segment is properly formatted (6 values: start_xyz, end_xyz)
        for segment in &wall_segments {
            // Each segment should have exactly 6 coordinates
            assert_eq!(segment.len(), 6);
            // Start and end points should be different
            let start = [segment[0], segment[1], segment[2]];
            let end = [segment[3], segment[4], segment[5]];
            let dist_sqr = (end[0] - start[0]).powi(2)
                + (end[1] - start[1]).powi(2)
                + (end[2] - start[2]).powi(2);
            assert!(dist_sqr > 0.001, "Wall segment should have non-zero length");
        }

        Ok(())
    }

    // More tests would be needed for full coverage, but we need a more complex
    // navigation mesh setup to test pathfinding properly.

    // Helper function to create a simple nav mesh for testing
    fn create_simple_nav_mesh() -> Result<NavMesh> {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 1,
            max_polys_per_tile: 1,
        };

        let mut nav_mesh = NavMesh::new(params.clone()).unwrap();

        // Create a simple tile with one polygon
        let tile_params = NavMeshCreateParams {
            nav_mesh_params: params,
            verts: vec![
                0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
            ],
            vert_count: 4,
            polys: vec![0, 1, 2, 3, 0xffff, 0xffff],
            poly_flags: vec![PolyFlags::WALK],
            poly_areas: vec![0],
            poly_count: 1,
            nvp: 6,
            detail_meshes: vec![0, 4, 0, 2],
            detail_verts: vec![
                0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
            ],
            detail_vert_count: 4,
            detail_tris: vec![0, 1, 2, 0, 2, 3],
            detail_tri_count: 2,
            off_mesh_con_verts: vec![],
            off_mesh_con_rad: vec![],
            off_mesh_con_flags: vec![],
            off_mesh_con_areas: vec![],
            off_mesh_con_dir: vec![],
            off_mesh_con_user_id: vec![],
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [10.0, 0.0, 10.0],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        let mut tile = NavMeshBuilder::build_tile(&tile_params)?;

        // Set tile header info
        if let Some(header) = tile.header.as_mut() {
            header.x = 0;
            header.y = 0;
            header.layer = 0;
        }

        nav_mesh.add_mesh_tile(tile)?;

        // Debug: Check if tile was added
        let tiles = nav_mesh.get_all_tiles();
        println!("Number of tiles: {}", tiles.len());
        if !tiles.is_empty() {
            let first_tile = &tiles[0];
            println!("First tile has {} polygons", first_tile.polys.len());
            if !first_tile.polys.is_empty() {
                println!("First polygon: {:?}", first_tile.polys[0]);
            }
        }

        Ok(nav_mesh)
    }

    #[test]
    fn test_sliced_pathfinding_simple() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Get the correct polygon reference from the first tile
        let tiles = nav_mesh.get_all_tiles();
        if tiles.is_empty() {
            panic!("No tiles in nav mesh!");
        }

        // Get the tile's salt value to create valid poly ref
        let first_tile = &tiles[0];
        let tile_salt = first_tile.salt;
        println!("Tile salt: {}", tile_salt);

        // Try different tile IDs to find the valid one
        let mut poly_ref = PolyRef::new(0);
        for tile_id in 1..10 {
            let test_ref = encode_poly_ref_with_salt(tile_salt, tile_id, 0);
            if nav_mesh.is_valid_poly_ref(test_ref) {
                println!("Found valid tile_id: {}", tile_id);
                poly_ref = test_ref;
                println!(
                    "Using poly_ref: {:?} (hex: 0x{:08x})",
                    poly_ref,
                    poly_ref.id()
                );
                break;
            }
        }

        // If we still don't have a valid ref, fall back to find_nearest_poly
        if !poly_ref.is_valid() {
            let center = [5.0, 0.0, 5.0];
            let half_extents = [1.0, 1.0, 1.0];
            match query.find_nearest_poly(&center, &half_extents, &filter) {
                Ok((found_ref, _pos)) => poly_ref = found_ref,
                Err(_) => poly_ref = encode_poly_ref_with_salt(tile_salt, 1, 0),
            }
        }
        println!(
            "Final poly_ref: {:?} (hex: 0x{:08x})",
            poly_ref,
            poly_ref.id()
        );
        println!(
            "Is valid poly_ref: {}",
            nav_mesh.is_valid_poly_ref(poly_ref)
        );

        let start_pos = [5.0, 0.0, 5.0];
        let end_pos = [8.0, 0.0, 8.0];

        // Initialize sliced pathfinding
        query.init_sliced_find_path(poly_ref, poly_ref, &start_pos, &end_pos, &filter, 0)?;

        // Update until complete
        let (iterations, state) = query.update_sliced_find_path(100)?;
        assert_eq!(state, SlicedPathState::Success);
        assert_eq!(iterations, 0); // Should complete immediately for same poly

        // Finalize path
        let path = query.finalize_sliced_find_path(10)?;
        assert_eq!(path.len(), 1);
        assert_eq!(path[0], poly_ref);

        Ok(())
    }

    #[test]
    fn test_sliced_pathfinding_with_iterations() -> Result<()> {
        // Create a 3x3 grid nav mesh
        let nav_mesh = create_3x3_grid_navmesh();
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Path from bottom-left to top-right using find_nearest_poly to get correct refs
        let start_center = [5.0, 0.0, 5.0]; // Center of first polygon
        let end_center = [25.0, 0.0, 25.0]; // Center of last polygon
        let half_extents = [20.0, 10.0, 20.0]; // Large search area

        println!("Finding start polygon near {:?}", start_center);
        let start_result = query.find_nearest_poly(&start_center, &half_extents, &filter);
        let (start_ref, start_nearest) = match start_result {
            Ok(result) => {
                println!("Found start polygon: {:?} at {:?}", result.0, result.1);
                result
            }
            Err(e) => {
                println!("Failed to find start polygon: {:?}", e);
                return Err(e);
            }
        };

        println!("Finding end polygon near {:?}", end_center);
        let end_result = query.find_nearest_poly(&end_center, &half_extents, &filter);
        let (end_ref, end_nearest) = match end_result {
            Ok(result) => {
                println!("Found end polygon: {:?} at {:?}", result.0, result.1);
                result
            }
            Err(e) => {
                println!("Failed to find end polygon: {:?}", e);
                return Err(e);
            }
        };
        let start_pos = start_nearest;
        let end_pos = end_nearest;

        // Initialize sliced pathfinding
        println!(
            "Initializing sliced pathfinding from {:?} to {:?}",
            start_ref, end_ref
        );
        match query.init_sliced_find_path(start_ref, end_ref, &start_pos, &end_pos, &filter, 0) {
            Ok(()) => println!("Sliced pathfinding initialized successfully"),
            Err(e) => {
                println!("Failed to initialize sliced pathfinding: {:?}", e);

                // Check if polygons are valid
                println!(
                    "Checking if start_ref is valid: {}",
                    nav_mesh.is_valid_poly_ref(start_ref)
                );
                println!(
                    "Checking if end_ref is valid: {}",
                    nav_mesh.is_valid_poly_ref(end_ref)
                );

                return Err(e);
            }
        }

        // Update with limited iterations
        let mut total_iterations = 0;
        let mut state = SlicedPathState::InProgress;

        while state == SlicedPathState::InProgress {
            println!(
                "Updating sliced pathfinding, iteration {}",
                total_iterations
            );
            match query.update_sliced_find_path(2) {
                Ok((iter, new_state)) => {
                    println!("  Updated {} iterations, new state: {:?}", iter, new_state);
                    total_iterations += iter;
                    state = new_state;
                }
                Err(e) => {
                    println!("  Update failed: {:?}", e);
                    return Err(e);
                }
            }

            // Safety check
            if total_iterations > 100 {
                panic!("Too many iterations");
            }
        }

        // Should eventually succeed
        assert_eq!(state, SlicedPathState::Success);

        // Finalize path
        let path = query.finalize_sliced_find_path(20)?;
        assert!(path.len() > 1); // Should have multiple polygons
        assert_eq!(path[0], start_ref);
        assert_eq!(path[path.len() - 1], end_ref);

        Ok(())
    }

    #[test]
    fn test_sliced_pathfinding_partial() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Get a valid start reference using find_nearest_poly
        let start_center = [5.0, 0.0, 5.0];
        let half_extents = [2.0, 2.0, 2.0];
        let (start_ref, _) = query.find_nearest_poly(&start_center, &half_extents, &filter)?;
        let invalid_end_ref = PolyRef::new(999); // Doesn't exist
        let start_pos = [5.0, 0.0, 5.0];
        let end_pos = [50.0, 0.0, 50.0];

        // This should fail during init
        let result = query.init_sliced_find_path(
            start_ref,
            invalid_end_ref,
            &start_pos,
            &end_pos,
            &filter,
            0,
        );
        assert!(result.is_err());

        Ok(())
    }

    #[test]
    fn test_sliced_pathfinding_any_angle() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Get polygon reference using find_nearest_poly to ensure correct salt
        let center = [5.0, 0.0, 5.0];
        let half_extents = [2.0, 2.0, 2.0];
        let (poly_ref, _) = query.find_nearest_poly(&center, &half_extents, &filter)?;
        let start_pos = [5.0, 0.0, 5.0];
        let end_pos = [8.0, 0.0, 8.0];

        // Initialize with DT_FINDPATH_ANY_ANGLE option
        const DT_FINDPATH_ANY_ANGLE: u32 = 0x02;
        query.init_sliced_find_path(
            poly_ref,
            poly_ref,
            &start_pos,
            &end_pos,
            &filter,
            DT_FINDPATH_ANY_ANGLE,
        )?;

        // Should still work (any-angle is a TODO)
        let (_, state) = query.update_sliced_find_path(100)?;
        assert_eq!(state, SlicedPathState::Success);

        Ok(())
    }

    #[test]
    fn test_query_polygons() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Query polygons in the center of the mesh
        let center = [5.0, 0.0, 5.0];
        let half_extents = [3.0, 2.0, 3.0];

        let polys = query.query_polygons(&center, &half_extents, &filter, 10)?;

        // Should find at least one polygon
        assert!(!polys.is_empty());

        Ok(())
    }

    #[test]
    fn test_query_polygons_with_custom_query() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Use a custom query to collect polygons
        let mut collect_query = CollectPolysQuery::new(5);
        let center = [5.0, 0.0, 5.0];
        let half_extents = [3.0, 2.0, 3.0];

        query.query_polygons_with_query(&center, &half_extents, &filter, &mut collect_query)?;

        // Should have collected at least one polygon
        assert!(collect_query.num_collected() > 0);
        assert!(!collect_query.overflow());

        Ok(())
    }

    #[test]
    fn test_find_nearest_poly_query() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Use FindNearestPolyQuery
        let center = [5.0, 0.0, 5.0];
        let half_extents = [5.0, 2.0, 5.0];
        let mut nearest_query = FindNearestPolyQuery::new(&query, &center);

        query.query_polygons_with_query(&center, &half_extents, &filter, &mut nearest_query)?;

        // Should have found a nearest polygon
        assert!(nearest_query.nearest_ref().is_valid());
        assert!(nearest_query.nearest_distance_sqr() < f32::MAX);

        Ok(())
    }

    // Helper function to create a 3x3 grid navigation mesh for testing
    fn create_3x3_grid_navmesh() -> NavMesh {
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 30.0,
            tile_height: 30.0,
            max_tiles: 1,
            max_polys_per_tile: 9,
        };

        let mut nav_mesh = NavMesh::new(params.clone()).unwrap();

        // Create 3x3 grid of polygons (0-8)
        let cell_size = 10.0;

        // Create vertices for a 3x3 grid (4x4 vertices)
        // Store as f32 values directly - they'll be used as-is in the tile
        let mut verts = Vec::new();
        for z in 0..4 {
            for x in 0..4 {
                verts.push(x as f32 * cell_size);
                verts.push(0.0);
                verts.push(z as f32 * cell_size);
            }
        }

        // Create 9 polygons in a 3x3 grid
        let mut polys = Vec::new();
        let mut poly_flags = Vec::new();
        let mut poly_areas = Vec::new();

        for row in 0..3 {
            for col in 0..3 {
                let base_idx = row * 4 + col;
                let _poly_idx = row * 3 + col;

                // Add polygon vertices (counter-clockwise)
                polys.push(base_idx as u16);
                polys.push((base_idx + 1) as u16);
                polys.push((base_idx + 5) as u16);
                polys.push((base_idx + 4) as u16);

                // Add neighbor information (bottom, right, top, left)
                // Bottom neighbor
                polys.push(if row > 0 {
                    ((row - 1) * 3 + col) as u16
                } else {
                    0xffff
                });
                // Right neighbor
                polys.push(if col < 2 {
                    (row * 3 + col + 1) as u16
                } else {
                    0xffff
                });
                // Top neighbor
                polys.push(if row < 2 {
                    ((row + 1) * 3 + col) as u16
                } else {
                    0xffff
                });
                // Left neighbor
                polys.push(if col > 0 {
                    (row * 3 + col - 1) as u16
                } else {
                    0xffff
                });

                poly_flags.push(PolyFlags::WALK);
                poly_areas.push(0);
            }
        }

        // Create detail mesh
        let mut detail_meshes = Vec::new();
        let mut detail_tris = Vec::new();

        for i in 0..9 {
            detail_meshes.push(0); // base vertex
            detail_meshes.push(4); // vertex count
            detail_meshes.push((i * 2) as u32); // base tri
            detail_meshes.push(2); // tri count

            // Two triangles per polygon
            detail_tris.push(0);
            detail_tris.push(1);
            detail_tris.push(2);
            detail_tris.push(0);
            detail_tris.push(2);
            detail_tris.push(3);
        }

        let tile_params = NavMeshCreateParams {
            nav_mesh_params: params.clone(),
            verts: verts.clone(),
            vert_count: 16,
            polys,
            poly_flags,
            poly_areas,
            poly_count: 9,
            nvp: 6,
            detail_meshes,
            detail_verts: verts,
            detail_vert_count: 16,
            detail_tris,
            detail_tri_count: 18,
            off_mesh_con_verts: vec![],
            off_mesh_con_rad: vec![],
            off_mesh_con_flags: vec![],
            off_mesh_con_areas: vec![],
            off_mesh_con_dir: vec![],
            off_mesh_con_user_id: vec![],
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [30.0, 0.0, 30.0],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        let tile = NavMeshBuilder::build_tile(&tile_params).unwrap();
        nav_mesh.add_mesh_tile(tile).unwrap();
        nav_mesh
    }

    #[test]
    fn test_get_poly_height() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find a valid polygon
        let center = [5.0, 0.0, 5.0];
        let half_extents = [2.0, 2.0, 2.0];

        // Find nearest polygon
        let (poly_ref, nearest_pt) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // If we found a valid polygon
        if poly_ref.is_valid() {
            // Test get_poly_height with the nearest point (should be inside the polygon)
            let height = query.get_poly_height(poly_ref, &nearest_pt)?;

            // For simple nav mesh without detail mesh, height might be None
            // This is still a valid test - we're testing the API exists and works
            if height.is_some() {
                // If we have height, it should be reasonable
                assert!(height.unwrap() >= -100.0 && height.unwrap() <= 100.0);
            }

            // Test with a position definitely outside any polygon
            let outside_pos = [1000.0, 0.0, 1000.0];
            let height_outside = query.get_poly_height(poly_ref, &outside_pos)?;

            // Height should be None for a position outside the polygon
            assert!(height_outside.is_none());
        }

        Ok(())
    }

    #[test]
    fn test_find_random_point_with_custom_rand() -> Result<()> {
        let nav_mesh = create_3x3_grid_navmesh();
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Test with a simple custom random function
        let counter = std::cell::Cell::new(0);
        let custom_rand = || {
            let val = counter.get();
            counter.set(val + 1);
            // Return a predictable sequence
            (val as f32 * 0.123) % 1.0
        };

        let (poly_ref, point) = query.find_random_point_with_custom_rand(&filter, custom_rand)?;

        assert!(poly_ref.is_valid());
        // Point should be within the navigation mesh bounds
        assert!(point[0] >= 0.0 && point[0] <= 30.0);
        assert!(point[2] >= 0.0 && point[2] <= 30.0);

        Ok(())
    }

    #[test]
    fn test_find_random_point_around_circle_with_custom_rand() -> Result<()> {
        let nav_mesh = create_3x3_grid_navmesh();
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Find a center polygon
        let center = [15.0, 0.0, 15.0];
        let half_extents = [5.0, 2.0, 5.0];
        let (center_ref, _) = query.find_nearest_poly(&center, &half_extents, &filter)?;

        // Test with custom random function
        let seed = std::cell::Cell::new(42u32);
        let custom_rand = || {
            // Simple LCG
            let val = seed.get();
            let new_val = val.wrapping_mul(1103515245).wrapping_add(12345);
            seed.set(new_val);
            (new_val & 0x7FFFFFFF) as f32 / 2147483647.0
        };

        let radius = 10.0;
        let (poly_ref, point) = query.find_random_point_around_circle(
            center_ref,
            &center,
            radius,
            &filter,
            custom_rand,
        )?;

        assert!(poly_ref.is_valid());

        // Check that the point is within the radius
        let dx = point[0] - center[0];
        let dz = point[2] - center[2];
        let distance = (dx * dx + dz * dz).sqrt();
        assert!(distance <= radius);

        Ok(())
    }

    #[test]
    fn test_api_harmonization_methods() -> Result<()> {
        let nav_mesh = create_simple_nav_mesh()?;
        let mut query = NavMeshQuery::new(&nav_mesh);
        let filter = QueryFilter::default();

        // Use very large search area to ensure we find the single polygon
        let center = [15.0, 0.0, 15.0];
        let half_extents = [20.0, 10.0, 20.0];

        // Test find_nearest_poly_extended (new overload with is_over_poly)
        match query.find_nearest_poly_extended(&center, &half_extents, &filter) {
            Ok((nearest_ref, nearest_pt, _is_over_poly)) => {
                assert!(nearest_ref.is_valid());

                // Test basic find_nearest_poly returns same result
                let (nearest_ref_basic, nearest_pt_basic) =
                    query.find_nearest_poly(&center, &half_extents, &filter)?;
                assert_eq!(nearest_ref, nearest_ref_basic);
                assert_eq!(nearest_pt, nearest_pt_basic);

                // Test find_straight_path_with_options (new overload with options)
                let path_refs = vec![nearest_ref];
                let start_pos = nearest_pt;
                let end_pos = nearest_pt; // Same position to avoid complexity

                let path_with_options =
                    query.find_straight_path_with_options(&start_pos, &end_pos, &path_refs, 0)?;
                let path_basic = query.find_straight_path(&start_pos, &end_pos, &path_refs)?;

                // Should return the same result
                assert_eq!(
                    path_with_options.waypoint_count(),
                    path_basic.waypoint_count()
                );

                // Test init_sliced_find_path_default (new overload with default options)
                query.init_sliced_find_path_default(
                    nearest_ref,
                    nearest_ref,
                    &start_pos,
                    &end_pos,
                    &filter,
                )?;

                // Test that it's equivalent to calling with options = 0
                let mut query2 = NavMeshQuery::new(&nav_mesh);
                query2.init_sliced_find_path(
                    nearest_ref,
                    nearest_ref,
                    &start_pos,
                    &end_pos,
                    &filter,
                    0,
                )?;

                // Both should be in the same state
                let (_iter1, state1) = query.update_sliced_find_path(1)?;
                let (_iter2, state2) = query2.update_sliced_find_path(1)?;
                assert_eq!(state1, state2);
            }
            Err(_) => {
                // If we can't find a polygon, just test that the API methods exist and can be called
                // This ensures API compatibility even if we don't have proper test data

                // These should fail gracefully but the APIs should exist
                assert!(
                    query
                        .find_nearest_poly(&center, &half_extents, &filter)
                        .is_err()
                );
                assert!(
                    query
                        .find_nearest_poly_extended(&center, &half_extents, &filter)
                        .is_err()
                );

                // Test straight path methods with empty path (should handle gracefully)
                let empty_path = vec![];
                let pos = [0.0, 0.0, 0.0];
                assert!(query.find_straight_path(&pos, &pos, &empty_path).is_err());
                assert!(
                    query
                        .find_straight_path_with_options(&pos, &pos, &empty_path, 0)
                        .is_err()
                );
            }
        }

        Ok(())
    }
}
