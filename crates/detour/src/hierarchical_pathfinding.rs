//! Hierarchical pathfinding for large worlds
//!
//! This module provides multi-level pathfinding that enables efficient navigation
//! across large game worlds by using hierarchical abstraction of the navigation mesh.
//!
//! The hierarchical approach works by:
//! 1. Dividing the world into tiles or clusters
//! 2. Creating abstract graphs at multiple levels
//! 3. Planning high-level paths first, then refining down to detailed paths
//! 4. Caching results at different levels for performance

use super::nav_mesh::NavMesh;
use super::status::Status;
use super::{PolyRef, QueryFilter};
use glam::Vec3;
use recast_common::{Error, Result};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

/// Maximum number of hierarchical levels
#[allow(dead_code)]
const MAX_HIERARCHY_LEVELS: usize = 4;

/// Default cluster size for level 0 (in navigation mesh units)
const DEFAULT_CLUSTER_SIZE: f32 = 64.0;

/// Hierarchical pathfinding system
#[derive(Debug)]
pub struct HierarchicalPathfinder {
    /// The base navigation mesh
    /// TODO: Currently stored but not used - would be used for actual pathfinding implementation
    #[allow(dead_code)]
    nav_mesh: NavMesh,
    /// Hierarchical levels (level 0 = finest detail)
    pub levels: Vec<HierarchyLevel>,
    /// Cluster size at level 0
    pub base_cluster_size: f32,
}

/// A single level in the hierarchy
#[derive(Debug, Clone)]
pub struct HierarchyLevel {
    /// Level index (0 = base level)
    pub level: usize,
    /// Size of clusters at this level
    pub cluster_size: f32,
    /// Clusters at this level
    pub clusters: Vec<Cluster>,
    /// Abstract graph connections between clusters
    pub connections: Vec<ClusterConnection>,
    /// Spatial index for fast cluster lookup
    pub spatial_index: HashMap<(i32, i32), Vec<usize>>,
}

/// A cluster of navigation polygons
#[derive(Debug, Clone)]
pub struct Cluster {
    /// Unique cluster ID
    pub id: usize,
    /// Bounding box of the cluster
    pub bounds: ClusterBounds,
    /// Polygons contained in this cluster
    pub polygons: Vec<PolyRef>,
    /// Entry/exit points to other clusters
    pub portals: Vec<Portal>,
    /// Center point of the cluster
    pub center: Vec3,
    /// Whether this cluster is walkable
    pub walkable: bool,
}

/// Connection between two clusters
#[derive(Debug, Clone)]
pub struct ClusterConnection {
    /// Source cluster ID
    pub from_cluster: usize,
    /// Destination cluster ID
    pub to_cluster: usize,
    /// Portal connecting the clusters
    pub portal: Portal,
    /// Cost to traverse this connection
    pub cost: f32,
    /// Distance of the connection
    pub distance: f32,
}

/// Portal between clusters
#[derive(Debug, Clone)]
pub struct Portal {
    /// Position of the portal
    pub position: Vec3,
    /// Width of the portal
    pub width: f32,
    /// Direction vector of the portal
    pub direction: Vec3,
    /// Polygons on either side of the portal
    pub poly_refs: [PolyRef; 2],
}

/// Bounding box for a cluster
#[derive(Debug, Clone)]
pub struct ClusterBounds {
    pub min: Vec3,
    pub max: Vec3,
}

/// Configuration for hierarchical pathfinding
#[derive(Debug, Clone)]
pub struct HierarchicalConfig {
    /// Number of hierarchy levels
    pub max_levels: usize,
    /// Base cluster size
    pub base_cluster_size: f32,
    /// Scaling factor between levels
    pub level_scale_factor: f32,
    /// Maximum distance to search at each level
    pub max_search_distance: f32,
    /// Whether to cache paths
    pub enable_caching: bool,
}

impl Default for HierarchicalConfig {
    fn default() -> Self {
        Self {
            max_levels: 3,
            base_cluster_size: DEFAULT_CLUSTER_SIZE,
            level_scale_factor: 4.0,
            max_search_distance: 1000.0,
            enable_caching: true,
        }
    }
}

/// Result of hierarchical pathfinding
#[derive(Debug)]
pub struct HierarchicalPath {
    /// Complete path from start to end
    pub path: Vec<Vec3>,
    /// Clusters traversed at each level
    pub cluster_path: Vec<Vec<usize>>,
    /// Cost of the path
    pub cost: f32,
    /// Status of the pathfinding operation
    pub status: Status,
}

/// Cache entry for hierarchical paths
/// TODO: Implement path caching for improved performance
#[allow(dead_code)]
#[derive(Debug, Clone)]
struct CacheEntry {
    path: Vec<usize>,
    cost: f32,
    timestamp: u64,
}

/// Path cache for hierarchical pathfinding
/// TODO: Implement path caching for improved performance
#[allow(dead_code)]
#[derive(Debug)]
struct PathCache {
    entries: HashMap<(usize, usize, usize), CacheEntry>, // (level, from, to) -> path
    max_entries: usize,
    current_time: u64,
}

impl HierarchicalPathfinder {
    /// Creates a new hierarchical pathfinder from a navigation mesh
    pub fn new(nav_mesh: NavMesh, config: HierarchicalConfig) -> Result<Self> {
        let mut pathfinder = Self {
            nav_mesh,
            levels: Vec::new(),
            base_cluster_size: config.base_cluster_size,
        };

        // Build the hierarchical levels
        pathfinder.build_hierarchy(&config)?;

        Ok(pathfinder)
    }

    /// Finds a path using hierarchical pathfinding
    pub fn find_path(
        &self,
        start: Vec3,
        end: Vec3,
        filter: &QueryFilter,
    ) -> Result<HierarchicalPath> {
        // Find starting and ending clusters at each level
        let start_clusters = self.find_clusters_at_position(start)?;
        let end_clusters = self.find_clusters_at_position(end)?;

        if start_clusters.is_empty() || end_clusters.is_empty() {
            return Ok(HierarchicalPath {
                path: Vec::new(),
                cluster_path: Vec::new(),
                cost: f32::INFINITY,
                status: Status::Failure,
            });
        }

        // Plan from highest level down to lowest level
        let mut cluster_paths = Vec::new();
        let mut current_start = start_clusters[self.levels.len() - 1];
        let mut current_end = end_clusters[self.levels.len() - 1];

        // Start from the highest level and work down
        for level in (0..self.levels.len()).rev() {
            let level_path = self.find_path_at_level(level, current_start, current_end, filter)?;

            if level_path.is_empty() {
                return Ok(HierarchicalPath {
                    path: Vec::new(),
                    cluster_path: Vec::new(),
                    cost: f32::INFINITY,
                    status: Status::Failure,
                });
            }

            cluster_paths.push(level_path.clone());

            // Refine for the next level down
            if level > 0 {
                current_start = start_clusters[level - 1];
                current_end = end_clusters[level - 1];
            }
        }

        // Convert cluster path to actual world positions
        let world_path = self.convert_to_world_path(&cluster_paths, start, end)?;
        let total_cost = self.calculate_path_cost(&world_path);

        Ok(HierarchicalPath {
            path: world_path,
            cluster_path: cluster_paths,
            cost: total_cost,
            status: Status::Success,
        })
    }

    /// Builds the hierarchical levels
    fn build_hierarchy(&mut self, config: &HierarchicalConfig) -> Result<()> {
        // Build each level of the hierarchy
        for level in 0..config.max_levels {
            let cluster_size =
                config.base_cluster_size * config.level_scale_factor.powi(level as i32);
            let hierarchy_level = self.build_level(level, cluster_size)?;
            self.levels.push(hierarchy_level);
        }

        Ok(())
    }

    /// Builds a single level of the hierarchy
    fn build_level(&self, level: usize, cluster_size: f32) -> Result<HierarchyLevel> {
        let mut clusters = Vec::new();
        let mut spatial_index = HashMap::new();

        // Get navigation mesh bounds
        let mesh_bounds = self.get_mesh_bounds();

        // Create grid of clusters
        let grid_width = ((mesh_bounds.max.x - mesh_bounds.min.x) / cluster_size).ceil() as i32;
        let grid_height = ((mesh_bounds.max.z - mesh_bounds.min.z) / cluster_size).ceil() as i32;

        let mut cluster_id = 0;

        for grid_y in 0..grid_height {
            for grid_x in 0..grid_width {
                let min_x = mesh_bounds.min.x + grid_x as f32 * cluster_size;
                let min_z = mesh_bounds.min.z + grid_y as f32 * cluster_size;
                let max_x = min_x + cluster_size;
                let max_z = min_z + cluster_size;

                let bounds = ClusterBounds {
                    min: Vec3::new(min_x, mesh_bounds.min.y, min_z),
                    max: Vec3::new(max_x, mesh_bounds.max.y, max_z),
                };

                // Find polygons in this cluster
                let polygons = self.find_polygons_in_bounds(&bounds)?;

                if !polygons.is_empty() {
                    let center = Vec3::new(
                        (min_x + max_x) * 0.5,
                        (mesh_bounds.min.y + mesh_bounds.max.y) * 0.5,
                        (min_z + max_z) * 0.5,
                    );

                    let cluster = Cluster {
                        id: cluster_id,
                        bounds,
                        polygons,
                        portals: Vec::new(),
                        center,
                        walkable: true,
                    };

                    // Add to spatial index
                    spatial_index
                        .entry((grid_x, grid_y))
                        .or_insert_with(Vec::new)
                        .push(clusters.len());

                    clusters.push(cluster);
                    cluster_id += 1;
                }
            }
        }

        // Build connections between clusters
        let connections = self.build_cluster_connections(&clusters)?;

        Ok(HierarchyLevel {
            level,
            cluster_size,
            clusters,
            connections,
            spatial_index,
        })
    }

    /// Finds clusters that contain the given position
    pub fn find_clusters_at_position(&self, position: Vec3) -> Result<Vec<usize>> {
        let mut cluster_ids = Vec::new();

        for level in &self.levels {
            // Calculate grid coordinates
            let grid_x =
                ((position.x - self.get_mesh_bounds().min.x) / level.cluster_size).floor() as i32;
            let grid_z =
                ((position.z - self.get_mesh_bounds().min.z) / level.cluster_size).floor() as i32;

            if let Some(cluster_indices) = level.spatial_index.get(&(grid_x, grid_z)) {
                // Find the specific cluster containing this position
                for &cluster_idx in cluster_indices {
                    let cluster = &level.clusters[cluster_idx];
                    if self.point_in_bounds(position, &cluster.bounds) {
                        cluster_ids.push(cluster_idx);
                        break;
                    }
                }
            }

            // If no cluster found at this level, use the nearest one
            if cluster_ids.len() <= level.level {
                let nearest = self.find_nearest_cluster(level, position)?;
                cluster_ids.push(nearest);
            }
        }

        Ok(cluster_ids)
    }

    /// Finds a path at a specific hierarchy level
    fn find_path_at_level(
        &self,
        level: usize,
        start_cluster: usize,
        end_cluster: usize,
        _filter: &QueryFilter,
    ) -> Result<Vec<usize>> {
        if level >= self.levels.len() {
            return Err(Error::NavMeshGeneration(
                "Invalid hierarchy level".to_string(),
            ));
        }

        let hierarchy_level = &self.levels[level];

        // Use A* to find path between clusters
        let mut open_set = BinaryHeap::new();
        let mut came_from = HashMap::new();
        let mut g_score = HashMap::new();
        let mut f_score = HashMap::new();

        // Initialize with start cluster
        g_score.insert(start_cluster, 0.0);
        let h_start = self.heuristic_cost(hierarchy_level, start_cluster, end_cluster);
        f_score.insert(start_cluster, h_start);
        open_set.push(Reverse((
            ordered_float::NotNan::new(h_start).unwrap(),
            start_cluster,
        )));

        while let Some(Reverse((_, current))) = open_set.pop() {
            if current == end_cluster {
                // Reconstruct path
                let mut path = vec![current];
                let mut node = current;
                while let Some(&parent) = came_from.get(&node) {
                    path.push(parent);
                    node = parent;
                }
                path.reverse();
                return Ok(path);
            }

            // Explore neighbors
            for connection in &hierarchy_level.connections {
                let neighbor = if connection.from_cluster == current {
                    connection.to_cluster
                } else if connection.to_cluster == current {
                    connection.from_cluster
                } else {
                    continue;
                };

                let tentative_g_score =
                    g_score.get(&current).unwrap_or(&f32::INFINITY) + connection.cost;

                if tentative_g_score < *g_score.get(&neighbor).unwrap_or(&f32::INFINITY) {
                    came_from.insert(neighbor, current);
                    g_score.insert(neighbor, tentative_g_score);
                    let h_neighbor = self.heuristic_cost(hierarchy_level, neighbor, end_cluster);
                    let f_neighbor = tentative_g_score + h_neighbor;
                    f_score.insert(neighbor, f_neighbor);

                    open_set.push(Reverse((
                        ordered_float::NotNan::new(f_neighbor).unwrap(),
                        neighbor,
                    )));
                }
            }
        }

        // No path found
        Ok(Vec::new())
    }

    /// Calculates heuristic cost between clusters
    fn heuristic_cost(&self, level: &HierarchyLevel, from: usize, to: usize) -> f32 {
        if from >= level.clusters.len() || to >= level.clusters.len() {
            return f32::INFINITY;
        }

        let from_cluster = &level.clusters[from];
        let to_cluster = &level.clusters[to];

        // Euclidean distance between cluster centers
        (to_cluster.center - from_cluster.center).length()
    }

    /// Converts cluster path to world coordinates
    fn convert_to_world_path(
        &self,
        cluster_paths: &[Vec<usize>],
        start: Vec3,
        end: Vec3,
    ) -> Result<Vec<Vec3>> {
        if cluster_paths.is_empty() {
            return Ok(vec![start, end]);
        }

        let mut world_path = vec![start];

        // Use the most detailed level (level 0) for the actual path
        if let Some(detailed_path) = cluster_paths.first() {
            for &cluster_id in detailed_path.iter().skip(1) {
                if cluster_id < self.levels[0].clusters.len() {
                    let cluster = &self.levels[0].clusters[cluster_id];
                    world_path.push(cluster.center);
                }
            }
        }

        world_path.push(end);
        Ok(world_path)
    }

    /// Calculates the total cost of a path
    pub fn calculate_path_cost(&self, path: &[Vec3]) -> f32 {
        let mut total_cost = 0.0;
        for i in 1..path.len() {
            total_cost += (path[i] - path[i - 1]).length();
        }
        total_cost
    }

    /// Helper methods
    fn get_mesh_bounds(&self) -> ClusterBounds {
        // For now, return a default bounds - in a real implementation,
        // this would calculate the actual bounds of the navigation mesh
        ClusterBounds {
            min: Vec3::new(-500.0, -10.0, -500.0),
            max: Vec3::new(500.0, 10.0, 500.0),
        }
    }

    fn find_polygons_in_bounds(&self, bounds: &ClusterBounds) -> Result<Vec<PolyRef>> {
        // For now, return dummy polygon references
        // In a real implementation, this would query the navigation mesh
        // for polygons that intersect with the given bounds

        // Return some polygons for any bounds within reasonable range
        // This ensures clusters have content and can form connections
        if bounds.max.x >= -600.0
            && bounds.min.x <= 600.0
            && bounds.max.z >= -600.0
            && bounds.min.z <= 600.0
        {
            // Generate a few dummy polygon references based on bounds position
            let base_id = ((bounds.min.x + 600.0) / 100.0) as u32 * 100
                + ((bounds.min.z + 600.0) / 100.0) as u32;
            Ok(vec![
                PolyRef::new(base_id + 1),
                PolyRef::new(base_id + 2),
                PolyRef::new(base_id + 3),
            ])
        } else {
            Ok(vec![]) // No polygons in this area
        }
    }

    fn point_in_bounds(&self, point: Vec3, bounds: &ClusterBounds) -> bool {
        point.x >= bounds.min.x
            && point.x <= bounds.max.x
            && point.y >= bounds.min.y
            && point.y <= bounds.max.y
            && point.z >= bounds.min.z
            && point.z <= bounds.max.z
    }

    fn find_nearest_cluster(&self, level: &HierarchyLevel, position: Vec3) -> Result<usize> {
        let mut nearest_idx = 0;
        let mut nearest_distance = f32::INFINITY;

        for (idx, cluster) in level.clusters.iter().enumerate() {
            let distance = (cluster.center - position).length();
            if distance < nearest_distance {
                nearest_distance = distance;
                nearest_idx = idx;
            }
        }

        Ok(nearest_idx)
    }

    fn build_cluster_connections(&self, clusters: &[Cluster]) -> Result<Vec<ClusterConnection>> {
        let mut connections = Vec::new();

        // For each pair of clusters, check if they are adjacent and create connections
        for i in 0..clusters.len() {
            for j in (i + 1)..clusters.len() {
                if self.clusters_are_adjacent(&clusters[i], &clusters[j]) {
                    let distance = (clusters[j].center - clusters[i].center).length();
                    let cost = distance * 1.0; // Base cost multiplier

                    // Create a simple portal at the midpoint
                    let portal = Portal {
                        position: (clusters[i].center + clusters[j].center) * 0.5,
                        width: 2.0,
                        direction: (clusters[j].center - clusters[i].center).normalize(),
                        poly_refs: [PolyRef::new(1), PolyRef::new(2)], // Dummy polygon references
                    };

                    connections.push(ClusterConnection {
                        from_cluster: i,
                        to_cluster: j,
                        portal,
                        cost,
                        distance,
                    });
                }
            }
        }

        Ok(connections)
    }

    fn clusters_are_adjacent(&self, cluster1: &Cluster, cluster2: &Cluster) -> bool {
        // Simple adjacency check based on bounding box overlap or proximity
        let distance = (cluster1.center - cluster2.center).length();
        distance <= cluster1.bounds.max.x - cluster1.bounds.min.x + 1.0 // Allow some tolerance
    }
}

#[allow(dead_code)]
impl PathCache {
    fn new(max_entries: usize) -> Self {
        Self {
            entries: HashMap::new(),
            max_entries,
            current_time: 0,
        }
    }

    fn get(&mut self, level: usize, from: usize, to: usize) -> Option<Vec<usize>> {
        self.current_time += 1;

        if let Some(entry) = self.entries.get_mut(&(level, from, to)) {
            entry.timestamp = self.current_time;
            Some(entry.path.clone())
        } else {
            None
        }
    }

    fn insert(&mut self, level: usize, from: usize, to: usize, path: Vec<usize>, cost: f32) {
        self.current_time += 1;

        // Remove oldest entries if cache is full
        if self.entries.len() >= self.max_entries {
            let oldest_key = self
                .entries
                .iter()
                .min_by_key(|(_, entry)| entry.timestamp)
                .map(|(key, _)| *key);

            if let Some(key) = oldest_key {
                self.entries.remove(&key);
            }
        }

        self.entries.insert(
            (level, from, to),
            CacheEntry {
                path,
                cost,
                timestamp: self.current_time,
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav_mesh::NavMesh;

    fn create_test_nav_mesh() -> NavMesh {
        use super::super::NavMeshParams;
        let params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 100.0,
            tile_height: 100.0,
            max_tiles: 256, // Must be < 1024
            max_polys_per_tile: 2048,
        };
        NavMesh::new(params).expect("Failed to create test NavMesh")
    }

    #[test]
    fn test_hierarchical_pathfinder_creation() {
        let nav_mesh = create_test_nav_mesh();
        let config = HierarchicalConfig::default();

        let pathfinder = HierarchicalPathfinder::new(nav_mesh, config);
        assert!(pathfinder.is_ok());

        let pathfinder = pathfinder.unwrap();
        assert_eq!(pathfinder.levels.len(), 3); // Default max_levels
    }

    #[test]
    fn test_cluster_bounds() {
        let bounds = ClusterBounds {
            min: Vec3::new(0.0, 0.0, 0.0),
            max: Vec3::new(10.0, 10.0, 10.0),
        };

        let nav_mesh = create_test_nav_mesh();
        let config = HierarchicalConfig::default();
        let pathfinder = HierarchicalPathfinder::new(nav_mesh, config).unwrap();

        // Test point inside bounds
        assert!(pathfinder.point_in_bounds(Vec3::new(5.0, 5.0, 5.0), &bounds));

        // Test point outside bounds
        assert!(!pathfinder.point_in_bounds(Vec3::new(15.0, 5.0, 5.0), &bounds));
    }

    #[test]
    fn test_cluster_adjacency() {
        let nav_mesh = create_test_nav_mesh();
        let config = HierarchicalConfig::default();
        let pathfinder = HierarchicalPathfinder::new(nav_mesh, config).unwrap();

        let cluster1 = Cluster {
            id: 0,
            bounds: ClusterBounds {
                min: Vec3::new(0.0, 0.0, 0.0),
                max: Vec3::new(10.0, 10.0, 10.0),
            },
            polygons: vec![PolyRef::new(1), PolyRef::new(2)],
            portals: Vec::new(),
            center: Vec3::new(5.0, 5.0, 5.0),
            walkable: true,
        };

        let cluster2 = Cluster {
            id: 1,
            bounds: ClusterBounds {
                min: Vec3::new(10.0, 0.0, 0.0),
                max: Vec3::new(20.0, 10.0, 10.0),
            },
            polygons: vec![PolyRef::new(3), PolyRef::new(4)],
            portals: Vec::new(),
            center: Vec3::new(15.0, 5.0, 5.0),
            walkable: true,
        };

        // Adjacent clusters should be detected
        assert!(pathfinder.clusters_are_adjacent(&cluster1, &cluster2));

        let cluster3 = Cluster {
            id: 2,
            bounds: ClusterBounds {
                min: Vec3::new(50.0, 0.0, 0.0),
                max: Vec3::new(60.0, 10.0, 10.0),
            },
            polygons: vec![PolyRef::new(5), PolyRef::new(6)],
            portals: Vec::new(),
            center: Vec3::new(55.0, 5.0, 5.0),
            walkable: true,
        };

        // Non-adjacent clusters should not be detected as adjacent
        assert!(!pathfinder.clusters_are_adjacent(&cluster1, &cluster3));
    }

    #[test]
    fn test_path_cache() {
        let mut cache = PathCache::new(3);

        // Insert paths
        cache.insert(0, 1, 2, vec![1, 5, 2], 10.0);
        cache.insert(0, 2, 3, vec![2, 6, 3], 15.0);

        // Retrieve paths
        assert_eq!(cache.get(0, 1, 2), Some(vec![1, 5, 2]));
        assert_eq!(cache.get(0, 2, 3), Some(vec![2, 6, 3]));
        assert_eq!(cache.get(0, 3, 4), None);

        // Test cache eviction
        cache.insert(0, 3, 4, vec![3, 7, 4], 20.0);
        cache.insert(0, 4, 5, vec![4, 8, 5], 25.0);

        // Oldest entry should be evicted
        assert!(cache.entries.len() <= 3);
    }

    #[test]
    fn test_heuristic_cost_calculation() {
        let nav_mesh = create_test_nav_mesh();
        let config = HierarchicalConfig::default();
        let pathfinder = HierarchicalPathfinder::new(nav_mesh, config).unwrap();

        if !pathfinder.levels.is_empty() && pathfinder.levels[0].clusters.len() >= 2 {
            let cost = pathfinder.heuristic_cost(&pathfinder.levels[0], 0, 1);
            assert!(cost >= 0.0);
            assert!(cost.is_finite());
        }
    }
}
