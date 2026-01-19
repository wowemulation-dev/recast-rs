//! Path corridor implementation for Detour
//!
//! This module contains the PathCorridor structure, which is used to
//! represent and maintain a path corridor for smooth path following.

use std::f32;

use detour::{NavMeshQuery, PolyRef, QueryFilter, Status};
use recast_common::{Error, Result};

/// Maximum number of polygons in a path corridor
#[allow(dead_code)]
const MAX_PATH_POLYS: usize = 256;

/// Path corridor for smooth path following
#[derive(Debug, Clone)]
pub struct PathCorridor {
    /// Current position in the corridor
    pos: [f32; 3],
    /// Target position in the corridor
    target: [f32; 3],
    /// Path polygon references
    path: Vec<PolyRef>,
}

impl Default for PathCorridor {
    fn default() -> Self {
        Self::new()
    }
}

impl PathCorridor {
    /// Creates a new path corridor
    pub fn new() -> Self {
        Self {
            pos: [0.0; 3],
            target: [0.0; 3],
            path: Vec::new(),
        }
    }

    /// Allocates the corridor's path buffer
    pub fn init(&mut self, max_path: usize) -> bool {
        self.path.reserve(max_path);
        true
    }

    /// Resets the path corridor
    pub fn reset(&mut self, ref_value: PolyRef, pos: [f32; 3]) {
        self.pos = pos;
        self.target = pos;

        self.path.clear();

        if ref_value.is_valid() {
            self.path.push(ref_value);
        }
    }

    /// Generates a path to a target
    pub fn find_path(
        &mut self,
        query: &mut NavMeshQuery,
        target_ref: PolyRef,
        target: [f32; 3],
        filter: &QueryFilter,
    ) -> Result<()> {
        // Validate input
        if self.path.is_empty() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        if !target_ref.is_valid() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Make sure start polygon is still valid
        if !query.nav_mesh().is_valid_poly_ref(self.path[0]) {
            self.path.clear();
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Connect with straight line path
        let mut new_path = Vec::new();
        query
            .find_path(self.path[0], target_ref, &self.pos, &target, filter)
            .map(|path| new_path = path)?;

        // If path is empty, just return an error
        if new_path.is_empty() {
            return Err(Error::Detour(Status::PathInvalid.to_string()));
        }

        // Set the path and target
        self.path = new_path;
        self.target = target;

        Ok(())
    }

    /// Optimizes the path using a funnel algorithm
    pub fn optimize_path(&mut self, query: &mut NavMeshQuery, filter: &QueryFilter) -> Result<()> {
        if self.path.len() <= 2 {
            return Ok(());
        }

        // Simplify the path
        let mut opt_path = Vec::new();
        let mut visited = Vec::new();

        // Start with the current position
        opt_path.push(self.path[0]);

        // Move along the path
        let mut current_idx = 0;
        while current_idx < self.path.len() - 1 {
            // Find the furthest visible polygon
            let mut furthest_idx = current_idx + 1;

            for i in current_idx + 2..self.path.len() {
                let _result = query.move_along_surface(
                    self.path[current_idx],
                    &self.pos,
                    &self.target,
                    filter,
                    &mut visited,
                )?;

                // If we can reach the target directly, we're done
                if visited.contains(&self.path[i]) {
                    furthest_idx = i;
                } else {
                    break;
                }
            }

            // Add the furthest visible polygon to the optimized path
            opt_path.push(self.path[furthest_idx]);

            // Move to the furthest visible polygon
            current_idx = furthest_idx;
        }

        // Update the path
        self.path = opt_path;

        Ok(())
    }

    /// Moves the corridor position
    pub fn move_position(
        &mut self,
        new_pos: [f32; 3],
        query: &mut NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<()> {
        if self.path.is_empty() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Check if the new position is still within the first polygon
        let (closest, inside) = query
            .nav_mesh()
            .closest_point_on_poly(self.path[0], &new_pos)?;

        if inside {
            // We're still in the first polygon, use the closest position which has the correct height
            self.pos = closest;
            return Ok(());
        }

        // Find the new position by moving along the surface
        let mut visited = Vec::new();
        let result =
            query.move_along_surface(self.path[0], &self.pos, &new_pos, filter, &mut visited)?;

        // Update the position
        self.pos = result;

        // Adjust the path
        if visited.len() > 1 {
            // Check if we're now in a later polygon in the path
            let mut in_path = false;
            let mut path_idx = 0;

            for i in 1..self.path.len() {
                if self.path[i] == visited[visited.len() - 1] {
                    in_path = true;
                    path_idx = i;
                    break;
                }
            }

            if in_path {
                // Remove earlier polygons from the path
                self.path.drain(0..path_idx);
            } else {
                // We moved outside the path, so we need to rebuild it
                self.path = visited;
            }
        }

        // Make sure the path is still valid
        if self.path.is_empty() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        Ok(())
    }

    /// Advances the corridor towards the target position
    pub fn advance(
        &mut self,
        new_pos: [f32; 3],
        query: &mut NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<bool> {
        if self.path.is_empty() {
            return Ok(false);
        }

        // Update the corridor position
        self.move_position(new_pos, query, filter)?;

        // Check if we reached the target
        if self.path.len() == 1 {
            // Check if we're at the target
            let dist_sq = distance_sq(&self.pos, &self.target);

            // If we're close enough to the target, we've reached it
            if dist_sq < 0.01 * 0.01 {
                return Ok(true);
            }
        }

        // Find the furthest polygon we can reach in the path
        let mut furthest_valid_poly_idx = 0;

        for i in 1..self.path.len() {
            // Try to move along the surface to each polygon
            let mut visited = Vec::new();
            let _result = query.move_along_surface(
                self.path[0],
                &self.pos,
                &self.target,
                filter,
                &mut visited,
            )?;

            // Check if we can reach the polygon
            if visited.contains(&self.path[i]) {
                furthest_valid_poly_idx = i;
            } else {
                break;
            }
        }

        // Remove visited polygons from the path
        if furthest_valid_poly_idx > 0 {
            self.path.drain(0..furthest_valid_poly_idx);
        }

        // If we've reached the target (last polygon remaining)
        if self.path.len() == 1 {
            // Check if we're at the target
            let dist_sq = distance_sq(&self.pos, &self.target);

            // If we're close enough to the target, we've reached it
            if dist_sq < 0.01 * 0.01 {
                return Ok(true);
            }
        }

        // Still following the path
        Ok(false)
    }

    /// Shortens the path by removing polygons from the front
    pub fn shorten(
        &mut self,
        new_pos: [f32; 3],
        query: &mut NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<()> {
        if self.path.is_empty() {
            return Err(Error::Detour(Status::Failure.to_string()));
        }

        // Check if we can discard any polygons
        let mut furthest_valid_poly_idx = 0;

        for i in self.path.len() - 1..0 {
            // Check if we can reach the target from this polygon
            let mut visited = Vec::new();
            let _result = query.move_along_surface(
                self.path[i],
                &new_pos,
                &self.target,
                filter,
                &mut visited,
            )?;

            // Check if we can reach the end polygon
            if visited.contains(&self.path[self.path.len() - 1]) {
                furthest_valid_poly_idx = i;
                break;
            }
        }

        // Remove unnecessary polygons from the path
        if furthest_valid_poly_idx > 0 {
            self.path.drain(0..furthest_valid_poly_idx);
            self.pos = new_pos;
        }

        Ok(())
    }

    /// Gets the current position in the corridor
    pub fn get_pos(&self) -> [f32; 3] {
        self.pos
    }

    /// Gets the target position in the corridor
    pub fn get_target(&self) -> [f32; 3] {
        self.target
    }

    /// Gets the path of polygon references
    pub fn get_path(&self) -> &[PolyRef] {
        &self.path
    }

    /// Gets the length of the path
    pub fn get_path_count(&self) -> usize {
        self.path.len()
    }

    /// Finds the corners in the corridor from the position toward the target
    pub fn find_corners(
        &self,
        corner_verts: &mut Vec<[f32; 3]>,
        corner_flags: &mut Vec<u8>,
        corner_polys: &mut Vec<PolyRef>,
        max_corners: usize,
        navquery: &NavMeshQuery,
        _filter: &QueryFilter,
    ) -> Result<usize> {
        if self.path.is_empty() {
            return Ok(0);
        }

        corner_verts.clear();
        corner_flags.clear();
        corner_polys.clear();

        // Use the straight path functionality to find corners
        let straight_path = navquery.find_straight_path(&self.pos, &self.target, &self.path)?;

        let corner_count = straight_path.waypoints.len().min(max_corners);

        for i in 0..corner_count {
            corner_verts.push(straight_path.waypoints[i]);
            corner_flags.push(0); // Default flag
            if i < straight_path.poly_refs.len() {
                corner_polys.push(straight_path.poly_refs[i]);
            } else {
                corner_polys.push(PolyRef::new(0));
            }
        }

        Ok(corner_count)
    }

    /// Attempts to optimize the path if the specified point is visible from the current position
    pub fn optimize_path_visibility(
        &mut self,
        next: &[f32; 3],
        path_optimization_range: f32,
        navquery: &NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<()> {
        if self.path.len() < 3 {
            return Ok(());
        }

        // Check if we can see the next point
        let dir = [
            next[0] - self.pos[0],
            next[1] - self.pos[1],
            next[2] - self.pos[2],
        ];
        let max_dist = distance(&self.pos, next);

        let result = navquery.raycast(self.path[0], &self.pos, &dir, max_dist, filter);

        if let Ok((_hit_ref, _hit_pos, t)) = result {
            if t > 0.99 * max_dist {
                // We can see the target, optimize the path
                let dist = distance(&self.pos, next);
                if dist < path_optimization_range {
                    // Use a simple optimization: if we can reach the next point directly,
                    // remove intermediate polygons (simplified approach)
                    if self.path.len() > 2 {
                        // Keep first and last polygon only for now
                        let last_poly = self.path[self.path.len() - 1];
                        self.path.truncate(1);
                        self.path.push(last_poly);
                    }
                }
            }
        }

        Ok(())
    }

    /// Attempts to optimize the path using a local area search
    pub fn optimize_path_topology(
        &mut self,
        navquery: &mut NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<bool> {
        if self.path.len() < 5 {
            return Ok(false);
        }

        // Try to find a shorter path for the middle section
        let start_idx = 1;
        let end_idx = self.path.len() - 2;

        if end_idx <= start_idx {
            return Ok(false);
        }

        let start_ref = self.path[start_idx];
        let end_ref = self.path[end_idx];

        // Get positions for start and end (simplified approach)
        let start_pos = self.pos;
        let end_pos = self.target;

        // Try to find a direct path
        let new_path = navquery.find_path(start_ref, end_ref, &start_pos, &end_pos, filter)?;

        if new_path.len() < (end_idx - start_idx + 1) {
            // The new path is shorter, use it
            self.path.splice(start_idx..=end_idx, new_path);
            return Ok(true);
        }

        Ok(false)
    }

    /// Handles movement over off-mesh connections
    pub fn move_over_offmesh_connection(
        &mut self,
        offmesh_con_ref: PolyRef,
        refs: &mut [PolyRef; 2],
        start_pos: &mut [f32; 3],
        end_pos: &mut [f32; 3],
        _navquery: &NavMeshQuery,
    ) -> Result<bool> {
        if !offmesh_con_ref.is_valid() || self.path.is_empty() {
            return Ok(false);
        }

        // Find the off-mesh connection in our path
        let mut con_idx = None;
        for (i, &poly_ref) in self.path.iter().enumerate() {
            if poly_ref == offmesh_con_ref {
                con_idx = Some(i);
                break;
            }
        }

        let con_idx = match con_idx {
            Some(idx) => idx,
            None => return Ok(false),
        };

        // Set up the connection references
        refs[0] = if con_idx > 0 {
            self.path[con_idx - 1]
        } else {
            PolyRef::new(0)
        };
        refs[1] = if con_idx < self.path.len() - 1 {
            self.path[con_idx + 1]
        } else {
            PolyRef::new(0)
        };

        // For now, use current position as connection points (simplified)
        *start_pos = self.pos;
        *end_pos = self.target;

        // Move the position to the end of the connection
        self.pos = *end_pos;

        // Remove the off-mesh connection from the path
        self.path.remove(con_idx);

        Ok(true)
    }

    /// Fixes the path start position
    pub fn fix_path_start(&mut self, safe_ref: PolyRef, safe_pos: &[f32; 3]) -> Result<bool> {
        if !safe_ref.is_valid() {
            return Ok(false);
        }

        self.pos = *safe_pos;

        if self.path.is_empty() || self.path[0] != safe_ref {
            self.path.insert(0, safe_ref);
            return Ok(true);
        }

        Ok(false)
    }

    /// Trims invalid path segments
    pub fn trim_invalid_path(
        &mut self,
        safe_ref: PolyRef,
        safe_pos: &[f32; 3],
        navquery: &NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<bool> {
        // Find the first invalid polygon
        let mut first_invalid = None;
        for (i, &poly_ref) in self.path.iter().enumerate() {
            if !navquery.nav_mesh().is_valid_poly_ref(poly_ref) {
                first_invalid = Some(i);
                break;
            }

            // Check if polygon passes filter
            let nav_mesh = navquery.nav_mesh();
            if let Ok((tile, poly)) = nav_mesh.get_tile_and_poly_by_ref(poly_ref) {
                if !filter.pass_filter(poly_ref, tile, poly) {
                    first_invalid = Some(i);
                    break;
                }
            }
        }

        if let Some(invalid_idx) = first_invalid {
            // Trim the path at the invalid polygon
            self.path.truncate(invalid_idx);

            // If we trimmed everything, use the safe reference
            if self.path.is_empty() && safe_ref.is_valid() {
                self.path.push(safe_ref);
                self.pos = *safe_pos;
            }

            return Ok(true);
        }

        Ok(false)
    }

    /// Checks if the corridor path is valid
    pub fn is_valid(
        &self,
        max_look_ahead: usize,
        navquery: &NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<bool> {
        let check_count = self.path.len().min(max_look_ahead);

        for i in 0..check_count {
            let poly_ref = self.path[i];

            if !navquery.nav_mesh().is_valid_poly_ref(poly_ref) {
                return Ok(false);
            }

            // Check if polygon passes filter
            let nav_mesh = navquery.nav_mesh();
            if let Ok((tile, poly)) = nav_mesh.get_tile_and_poly_by_ref(poly_ref) {
                if !filter.pass_filter(poly_ref, tile, poly) {
                    return Ok(false);
                }
            } else {
                return Ok(false);
            }
        }

        Ok(true)
    }

    /// Moves the target position
    pub fn move_target_position(
        &mut self,
        npos: &[f32; 3],
        navquery: &mut NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<bool> {
        if self.path.is_empty() {
            return Ok(false);
        }

        // Find the nearest polygon to the new target position
        let half_extents = [2.0, 4.0, 2.0];
        let (nearest_ref, nearest_point) =
            navquery.find_nearest_poly(npos, &half_extents, filter)?;

        if !nearest_ref.is_valid() {
            return Ok(false);
        }

        self.target = nearest_point;

        // Update the path if needed
        let last_poly = self.path[self.path.len() - 1];
        if last_poly != nearest_ref {
            // Try to extend the path to the new target
            let path_to_target =
                navquery.find_path(last_poly, nearest_ref, &self.target, npos, filter)?;

            if !path_to_target.is_empty() {
                // Remove the last polygon (it's duplicated) and extend
                self.path.pop();
                self.path.extend(path_to_target);
            }
        }

        Ok(true)
    }

    /// Loads a new path and target into the corridor
    pub fn set_corridor(&mut self, target: &[f32; 3], polys: &[PolyRef], npath: usize) {
        self.target = *target;
        self.path.clear();
        self.path
            .extend_from_slice(&polys[..npath.min(polys.len())]);
    }

    /// Gets the first polygon in the corridor
    pub fn get_first_poly(&self) -> PolyRef {
        if self.path.is_empty() {
            PolyRef::new(0)
        } else {
            self.path[0]
        }
    }

    /// Gets the last polygon in the corridor
    pub fn get_last_poly(&self) -> PolyRef {
        if self.path.is_empty() {
            PolyRef::new(0)
        } else {
            self.path[self.path.len() - 1]
        }
    }
}

/// Calculates the squared distance between two points
fn distance_sq(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];

    dx * dx + dy * dy + dz * dz
}

/// Calculates the distance between two points
fn distance(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    distance_sq(a, b).sqrt()
}

/// Merges a corridor path when the start position has moved
///
/// This function is used to merge a path when the agent has moved from the start of the corridor.
/// It finds the furthest common polygon between the current path and the visited polygons,
/// then reconstructs the path to include the visited portion.
///
/// Note: This is part of the complete PathCorridor API matching C++ dtMergeCorridorStartMoved.
/// Currently unused but kept for API completeness.
#[allow(dead_code)]
pub fn merge_corridor_start_moved(
    path: &mut [PolyRef],
    npath: usize,
    max_path: usize,
    visited: &[PolyRef],
    nvisited: usize,
) -> usize {
    let mut furthest_path = None;
    let mut furthest_visited = None;

    // Find furthest common polygon
    for i in (0..npath).rev() {
        let mut found = false;
        for j in (0..nvisited).rev() {
            if path[i] == visited[j] {
                furthest_path = Some(i);
                furthest_visited = Some(j);
                found = true;
            }
        }
        if found {
            break;
        }
    }

    // If no intersection found, just return current path
    let (furthest_path, furthest_visited) = match (furthest_path, furthest_visited) {
        (Some(fp), Some(fv)) => (fp, fv),
        _ => return npath,
    };

    // Concatenate paths
    // Adjust beginning of the buffer to include the visited
    let req = nvisited - furthest_visited;
    let orig = (furthest_path + 1).min(npath);
    let mut size = npath.saturating_sub(orig);

    if req + size > max_path {
        size = max_path - req;
    }

    if size > 0 {
        // Move existing path to make room
        for i in (0..size).rev() {
            path[req + i] = path[orig + i];
        }
    }

    // Store visited (in reverse order)
    let n = req.min(max_path);
    for i in 0..n {
        path[i] = visited[(nvisited - 1) - i];
    }

    req + size
}

/// Merges a corridor path when the end position has moved
///
/// This function is used to merge a path when the target has moved from the end of the corridor.
/// It finds the furthest common polygon between the current path and the visited polygons,
/// then appends the new portion to the existing path.
#[allow(dead_code)]
pub fn merge_corridor_end_moved(
    path: &mut [PolyRef],
    npath: usize,
    max_path: usize,
    visited: &[PolyRef],
    nvisited: usize,
) -> usize {
    let mut furthest_path = None;
    let mut furthest_visited = None;

    // Find furthest common polygon (searching from start)
    for (i, &path_poly) in path.iter().take(npath).enumerate() {
        let mut found = false;
        for j in (0..nvisited).rev() {
            if path_poly == visited[j] {
                furthest_path = Some(i);
                furthest_visited = Some(j);
                found = true;
            }
        }
        if found {
            break;
        }
    }

    // If no intersection found, just return current path
    let (furthest_path, furthest_visited) = match (furthest_path, furthest_visited) {
        (Some(fp), Some(fv)) => (fp, fv),
        _ => return npath,
    };

    // Concatenate paths
    let ppos = furthest_path + 1;
    let vpos = furthest_visited + 1;
    let count = (nvisited - vpos).min(max_path - ppos);

    if count > 0 {
        // Copy the new portion
        path[ppos..(ppos + count)].copy_from_slice(&visited[vpos..(vpos + count)]);
    }

    ppos + count
}

/// Merges a corridor path after finding a shortcut
///
/// This function is used when a raycast or similar operation has found a shortcut in the path.
/// It merges the shortcut with the existing path, maintaining continuity.
#[allow(dead_code)]
pub fn merge_corridor_start_shortcut(
    path: &mut [PolyRef],
    npath: usize,
    max_path: usize,
    visited: &[PolyRef],
    nvisited: usize,
) -> usize {
    let mut furthest_path = None;
    let mut furthest_visited = None;

    // Find furthest common polygon
    for i in (0..npath).rev() {
        let mut found = false;
        for j in (0..nvisited).rev() {
            if path[i] == visited[j] {
                furthest_path = Some(i);
                furthest_visited = Some(j);
                found = true;
            }
        }
        if found {
            break;
        }
    }

    // If no intersection found, just return current path
    let (furthest_path, furthest_visited) = match (furthest_path, furthest_visited) {
        (Some(fp), Some(fv)) => (fp, fv),
        _ => return npath,
    };

    // Concatenate paths
    // Adjust beginning of the buffer to include the visited
    let req = furthest_visited;
    if req == 0 {
        return npath;
    }

    let orig = furthest_path;
    let mut size = npath.saturating_sub(orig);

    if req + size > max_path {
        size = max_path - req;
    }

    if size > 0 {
        // Move existing path to make room
        for i in (0..size).rev() {
            path[req + i] = path[orig + i];
        }
    }

    // Store visited
    path[..req].copy_from_slice(&visited[..req]);

    req + size
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_corridor() {
        let corridor = PathCorridor::new();

        assert_eq!(corridor.get_pos(), [0.0, 0.0, 0.0]);
        assert_eq!(corridor.get_target(), [0.0, 0.0, 0.0]);
        assert_eq!(corridor.get_path_count(), 0);
    }

    #[test]
    fn test_reset_corridor() {
        let mut corridor = PathCorridor::new();

        let start_pos = [10.0, 5.0, 10.0];
        let start_ref = PolyRef::new(42);

        corridor.reset(start_ref, start_pos);

        assert_eq!(corridor.get_pos(), start_pos);
        assert_eq!(corridor.get_target(), start_pos);
        assert_eq!(corridor.get_path_count(), 1);
        assert_eq!(corridor.get_path()[0], start_ref);
    }

    // More tests would be required for the full functionality, but would need a complex
    // navigation mesh setup to test properly.
}
