//! Dynamic navigation mesh implementation
//!
//! This module provides the main orchestrator for dynamic navigation mesh generation,
//! managing tiles, colliders, and the job processing system.

use crate::colliders::Collider;
use crate::config::DynamicNavMeshConfig;
use crate::dynamic_tile::{DynamicTile, DynamicTileManager};
use crate::io::{VoxelFile, VoxelTile};
use crate::jobs::{JobProcessor, DynamicTileJob};
use crate::voxel_query::{VoxelQuery, VoxelRaycastHit};
use detour::{NavMesh, NavMeshParams};
use glam::Vec3;
use recast_common::Result;
use recast::Heightfield;
use std::collections::HashMap;
use std::sync::{Arc, atomic::{AtomicU64, Ordering}};
use std::sync::mpsc::{self, Receiver, Sender};

/// Main dynamic navigation mesh manager
///
/// This is the primary interface for dynamic navigation mesh generation and management.
/// It orchestrates tiles, colliders, jobs, and provides high-level API for runtime updates.
pub struct DynamicNavMesh {
    /// Configuration for dynamic mesh generation
    config: DynamicNavMeshConfig,
    /// Tile manager for handling dynamic tiles
    _tile_manager: DynamicTileManager,
    /// Job processor for handling updates
    job_processor: JobProcessor,
    /// Current navigation mesh (built from tiles)
    nav_mesh: Option<NavMesh>,
    /// Navigation mesh parameters for Detour
    nav_mesh_params: NavMeshParams,
    /// Global voxel query system
    voxel_query: Option<VoxelQuery>,
    current_collider_id: AtomicU64,
    /// Update counter for change tracking
    update_counter: u64,
    /// Dirty flag indicating nav mesh needs rebuilding
    dirty: bool,
    tiles: HashMap<i64, DynamicTile>,
    job_sender: Sender<Box<dyn DynamicTileJob>>,
    job_receiver: Receiver<Box<dyn DynamicTileJob>>,
}

impl DynamicNavMesh {
    /// Create a new dynamic navigation mesh
    pub fn new(config: DynamicNavMeshConfig) -> Result<Self> {
        config.validate().map_err(recast_common::Error::Recast)?;

        let tile_manager = DynamicTileManager::new(config.clone());
        let job_processor = JobProcessor::new();
        
        let (job_sender, job_receiver) = mpsc::channel();

        // Initialize navigation mesh parameters following  pattern
        let nav_mesh_params = NavMeshParams {
            origin: [config.world_min.x, config.world_min.y, config.world_min.z],
            tile_width: if config.use_tiles {
                config.cell_size * config.tile_size_x as f32
            } else {
                config.world_max.x - config.world_min.x
            },
            tile_height: if config.use_tiles {
                config.cell_size * config.tile_size_z as f32
            } else {
                config.world_max.z - config.world_min.z
            },
            max_tiles: 1024, // Default max tiles
            max_polys_per_tile: 0x8000, // 32768 polys per tile
        };

        Ok(Self {
            config,
            _tile_manager: tile_manager,
            job_processor,
            nav_mesh: None,
            nav_mesh_params,
            voxel_query: None,
            current_collider_id: AtomicU64::new(0),
            update_counter: 0,
            dirty: true,
            tiles: HashMap::new(),
            job_sender,
            job_receiver,
        })
    }

    pub fn from_voxel_file(voxel_file: VoxelFile) -> Result<Self> {
        // Create config from voxel file parameters
        let mut config = DynamicNavMeshConfig::new(
            voxel_file.use_tiles,
            voxel_file.tile_size_x,
            voxel_file.tile_size_z,
            voxel_file.cell_size,
        );
        
        // Set all parameters from voxel file
        config = config
            .with_walkable_height(voxel_file.walkable_height)
            .with_walkable_radius(voxel_file.walkable_radius)
            .with_walkable_climb(voxel_file.walkable_climb)
            .with_walkable_slope_angle(voxel_file.walkable_slope_angle)
            .with_max_simplification_error(voxel_file.max_simplification_error)
            .with_max_edge_len(voxel_file.max_edge_len)
            .with_min_region_area(voxel_file.min_region_area)
            .with_region_merge_area(voxel_file.region_merge_area)
            .with_detail_mesh(voxel_file.build_mesh_detail)
            .with_detail_sample_distance(voxel_file.detail_sample_distance)
            .with_detail_sample_max_error(voxel_file.detail_sample_max_error)
            .with_world_bounds(
                Vec3::new(voxel_file.bounds[0], voxel_file.bounds[1], voxel_file.bounds[2]),
                Vec3::new(voxel_file.bounds[3], voxel_file.bounds[4], voxel_file.bounds[5]),
            );
        
        // Set navigation mesh parameters from voxel file
        let nav_mesh_params = NavMeshParams {
            origin: [voxel_file.bounds[0], voxel_file.bounds[1], voxel_file.bounds[2]],
            tile_width: if voxel_file.use_tiles {
                voxel_file.cell_size * voxel_file.tile_size_x as f32
            } else {
                voxel_file.bounds[3] - voxel_file.bounds[0]
            },
            tile_height: if voxel_file.use_tiles {
                voxel_file.cell_size * voxel_file.tile_size_z as f32
            } else {
                voxel_file.bounds[5] - voxel_file.bounds[2]
            },
            max_tiles: voxel_file.tiles.len() as i32,
            max_polys_per_tile: 0x8000,
        };
        
        let tile_manager = DynamicTileManager::new(config.clone());
        let job_processor = JobProcessor::new();
        let (job_sender, job_receiver) = mpsc::channel();
        
        let mut dynamic_navmesh = Self {
            config,
            _tile_manager: tile_manager,
            job_processor,
            nav_mesh: None,
            nav_mesh_params,
            voxel_query: None,
            current_collider_id: AtomicU64::new(0),
            update_counter: 0,
            dirty: true,
            tiles: HashMap::new(),
            job_sender,
            job_receiver,
        };
        
        for voxel_tile in voxel_file.tiles {
            dynamic_navmesh.add_voxel_tile(voxel_tile)?;
        }
        
        Ok(dynamic_navmesh)
    }
    
    /// Add a tile from voxel data
    fn add_voxel_tile(&mut self, voxel_tile: VoxelTile) -> Result<()> {
        // Create dynamic tile from voxel tile
        let tile = DynamicTile::from_voxel(voxel_tile, self.config.clone())?;
        
        let key = Self::lookup_key(tile.tile_x(), tile.tile_z());
        self.tiles.insert(key, tile);
        
        Ok(())
    }

    /// Create a dynamic navigation mesh with an initial tile grid
    pub fn with_tile_grid(
        config: DynamicNavMeshConfig,
        grid_width: i32,
        grid_height: i32,
    ) -> Result<Self> {
        let mut dynamic_navmesh = Self::new(config)?;
        dynamic_navmesh.initialize_tile_grid(grid_width, grid_height)?;
        Ok(dynamic_navmesh)
    }

    /// Initialize a grid of tiles covering the world bounds
    pub fn initialize_tile_grid(&mut self, grid_width: i32, grid_height: i32) -> Result<()> {
        let world_size_x = self.config.world_max.x - self.config.world_min.x;
        let world_size_z = self.config.world_max.z - self.config.world_min.z;

        let tile_size_x = world_size_x / grid_width as f32;
        let tile_size_z = world_size_z / grid_height as f32;

        for y in 0..grid_height {
            for x in 0..grid_width {
                let min_x = self.config.world_min.x + x as f32 * tile_size_x;
                let max_x = self.config.world_min.x + (x + 1) as f32 * tile_size_x;
                let min_z = self.config.world_min.z + y as f32 * tile_size_z;
                let max_z = self.config.world_min.z + (y + 1) as f32 * tile_size_z;

                let bounds_min = Vec3::new(min_x, self.config.world_min.y, min_z);
                let bounds_max = Vec3::new(max_x, self.config.world_max.y, max_z);

                // Note: In a real async implementation, this would be awaited
                // For now, we'll need to implement a sync version of add_tile
                self.add_tile_sync(x, y, bounds_min, bounds_max)?;
            }
        }

        Ok(())
    }

    /// Add a tile synchronously (helper for initialization)
    fn add_tile_sync(
        &mut self,
        x: i32,
        y: i32,
        bounds_min: Vec3,
        bounds_max: Vec3,
    ) -> Result<()> {
        let tile = DynamicTile::new(x, y, bounds_min, bounds_max, self.config.clone());
        let key = Self::lookup_key(x, y);
        self.tiles.insert(key, tile);
        Ok(())
    }

    pub fn add_collider(&mut self, collider: Arc<dyn Collider>) -> Result<u64> {
        let collider_id = self.current_collider_id.fetch_add(1, Ordering::SeqCst);
        
        let (collider_min, collider_max) = collider.bounds();
        let bounds = [
            collider_min.x, collider_min.y, collider_min.z,
            collider_max.x, collider_max.y, collider_max.z,
        ];

        let affected_tile_refs = self.get_tiles_by_bounds(&bounds);
        let affected_tiles: Vec<(i32, i32)> = affected_tile_refs.iter()
            .map(|tile| (tile.tile_x(), tile.tile_z()))
            .collect();

        let job = crate::jobs::ColliderAdditionJob::new(
            collider_id,
            collider,
            affected_tiles.into_iter().collect(),
        );
        
        self.job_sender.send(Box::new(job)).map_err(|_| {
            recast_common::Error::Recast("Failed to queue collider addition job".to_string())
        })?;

        Ok(collider_id)
    }

    pub fn remove_collider(&mut self, collider_id: u64) -> Result<()> {
        let affected_tile_refs = self.get_tiles_by_collider(collider_id);
        let affected_tiles: Vec<(i32, i32)> = affected_tile_refs.iter()
            .map(|tile| (tile.tile_x(), tile.tile_z()))
            .collect();

        let job = crate::jobs::ColliderRemovalJob::new(
            collider_id,
            affected_tiles.into_iter().collect(),
        );
        
        self.job_sender.send(Box::new(job)).map_err(|_| {
            recast_common::Error::Recast("Failed to queue collider removal job".to_string())
        })?;

        Ok(())
    }

    pub fn update(&mut self) -> Result<bool> {
        let affected_tile_coords = self.process_queue()?;
        
        if !affected_tile_coords.is_empty() {
            self.dirty = true;
            self.update_counter += 1;
            
            self.rebuild_affected_tiles(&affected_tile_coords)?;
            
            return self.update_nav_mesh();
        }
        
        Ok(false)
    }
    
    /// Rebuild specific affected tiles
    fn rebuild_affected_tiles(&mut self, _affected_tile_coords: &[(i32, i32)]) -> Result<()> {
        // For simplicity, rebuild all tiles that need rebuilding
        // In a real implementation, we'd only rebuild the specific affected tiles
        let tile_keys: Vec<i64> = self.tiles.keys().copied().collect();
        
        for key in tile_keys {
            if let Some(tile) = self.tiles.get_mut(&key) {
                if tile.needs_rebuild() {
                    tile.build()?;
                }
            }
        }
        
        Ok(())
    }

    /// Force a rebuild of specific tiles
    pub fn rebuild_tiles(
        &mut self,
        tile_coords: Vec<(i32, i32)>,
        force_full_rebuild: bool,
    ) -> Result<()> {
        for coords in tile_coords {
            self.job_processor.rebuild_tile(coords, force_full_rebuild);
        }
        Ok(())
    }

    /// Update the navigation mesh from all tiles
    pub fn update_nav_mesh(&mut self) -> Result<bool> {
        if !self.dirty {
            return Ok(false);
        }

        self.dirty = false;

        // Check if we have any built tiles with mesh data
        let has_built_tiles = self.tiles.values().any(|tile| tile.is_built() && tile.has_mesh_data());

        if has_built_tiles {
            // Create a new navigation mesh
            let mut nav_mesh = NavMesh::new(self.nav_mesh_params.clone())?;

            // Add all tiles to the navigation mesh
            for tile in self.tiles.values() {
                // Only add tiles that have been successfully built
                if tile.is_built() {
                    tile.add_to_nav_mesh(&mut nav_mesh, &self.nav_mesh_params)?;
                }
            }

            // Replace the old navigation mesh with the new one
            self.nav_mesh = Some(nav_mesh);
        } else {
            // No tiles with mesh data, clear the nav mesh
            self.nav_mesh = None;
        }

        log::info!(
            "Navigation mesh updated (update {}, tiles: {})",
            self.update_counter,
            self.tiles.len()
        );
        Ok(true)
    }

    /// Get the current navigation mesh
    pub fn nav_mesh(&self) -> Option<&NavMesh> {
        self.nav_mesh.as_ref()
    }

    /// Perform a voxel raycast query
    pub fn voxel_raycast(&self, start: Vec3, end: Vec3) -> Option<VoxelRaycastHit> {
        if let Some(ref voxel_query) = self.voxel_query {
            voxel_query.raycast(start, end)
        } else {
            None
        }
    }

    /// Initialize the global voxel query system
    pub fn initialize_voxel_query(&mut self) -> Result<()> {
        self.voxel_query = Some(self.create_voxel_query());
        Ok(())
    }

    /// Find tiles that are affected by a bounding box
    fn _find_affected_tiles(&self, min: &Vec3, max: &Vec3) -> Vec<(i32, i32)> {
        let mut affected_tiles = Vec::new();

        // Calculate tile size (assuming uniform grid)
        let world_size_x = self.config.world_max.x - self.config.world_min.x;
        let world_size_z = self.config.world_max.z - self.config.world_min.z;

        // For simplicity, assume a 10x10 grid
        let grid_size = 10;
        let tile_size_x = world_size_x / grid_size as f32;
        let tile_size_z = world_size_z / grid_size as f32;

        // Calculate tile range that overlaps with the bounding box
        let min_tile_x = ((min.x - self.config.world_min.x) / tile_size_x).floor() as i32;
        let max_tile_x = ((max.x - self.config.world_min.x) / tile_size_x).ceil() as i32;
        let min_tile_z = ((min.z - self.config.world_min.z) / tile_size_z).floor() as i32;
        let max_tile_z = ((max.z - self.config.world_min.z) / tile_size_z).ceil() as i32;

        for z in min_tile_z.max(0)..=max_tile_z.min(grid_size - 1) {
            for x in min_tile_x.max(0)..=max_tile_x.min(grid_size - 1) {
                affected_tiles.push((x, z));
            }
        }

        affected_tiles
    }

    /// Get statistics about the dynamic navigation mesh
    pub fn get_statistics(&self) -> DynamicNavMeshStatistics {
        let dirty_tiles = self.tiles.values()
            .filter(|t| t.needs_rebuild())
            .count();
        
        let memory_usage = self.tiles.values()
            .map(|t| t.memory_usage())
            .sum();

        DynamicNavMeshStatistics {
            total_tiles: self.tiles.len(),
            dirty_tiles,
            pending_jobs: self.job_processor.pending_job_count(),
            update_counter: self.update_counter,
            memory_usage,
        }
    }

    /// Check if the navigation mesh has pending updates
    pub fn has_pending_updates(&self) -> bool {
        !self.job_processor.is_idle()
    }

    /// Get the configuration used by this dynamic navigation mesh
    pub fn config(&self) -> &DynamicNavMeshConfig {
        &self.config
    }
    
    fn lookup_key(x: i32, z: i32) -> i64 {
        ((z as i64) << 32) | (x as i64)
    }
    
    fn get_tile_at(&self, x: i32, z: i32) -> Option<&DynamicTile> {
        self.tiles.get(&Self::lookup_key(x, z))
    }
    
    /// Get mutable tile at specific coordinates
    fn get_tile_at_mut(&mut self, x: i32, z: i32) -> Option<&mut DynamicTile> {
        self.tiles.get_mut(&Self::lookup_key(x, z))
    }
    
    fn get_tiles_by_bounds(&self, bounds: &[f32; 6]) -> Vec<&DynamicTile> {
        let minx = ((bounds[0] - self.nav_mesh_params.origin[0]) / self.nav_mesh_params.tile_width).floor() as i32 - 1;
        let minz = ((bounds[2] - self.nav_mesh_params.origin[2]) / self.nav_mesh_params.tile_height).floor() as i32 - 1;
        let maxx = ((bounds[3] - self.nav_mesh_params.origin[0]) / self.nav_mesh_params.tile_width).floor() as i32 + 1;
        let maxz = ((bounds[5] - self.nav_mesh_params.origin[2]) / self.nav_mesh_params.tile_height).floor() as i32 + 1;
        
        let mut result_tiles = Vec::new();
        for z in minz..=maxz {
            for x in minx..=maxx {
                if let Some(tile) = self.get_tile_at(x, z) {
                    if self.intersects_xz(tile, bounds) {
                        result_tiles.push(tile);
                    }
                }
            }
        }
        
        result_tiles
    }
    
    fn intersects_xz(&self, tile: &DynamicTile, bounds: &[f32; 6]) -> bool {
        tile.bounds_min.x <= bounds[3] && tile.bounds_max.x >= bounds[0] &&
        tile.bounds_min.z <= bounds[5] && tile.bounds_max.z >= bounds[2]
    }
    
    fn get_tiles_by_collider(&self, collider_id: u64) -> Vec<&DynamicTile> {
        self.tiles.values()
            .filter(|tile| tile.active_colliders.contains_key(&collider_id))
            .collect()
    }
    
    fn process_queue(&mut self) -> Result<Vec<(i32, i32)>> {
        let jobs = self.consume_queue();
        let mut affected_tile_coords = Vec::new();
        
        for job in jobs {
            // Collect affected tile coordinates
            for &(tile_x, tile_z) in job.affected_tiles() {
                affected_tile_coords.push((tile_x, tile_z));
                
                // Process job on tile
                if let Some(tile) = self.get_tile_at_mut(tile_x, tile_z) {
                    job.process(tile)?;
                }
            }
        }
        
        Ok(affected_tile_coords)
    }
    
    fn consume_queue(&mut self) -> Vec<Box<dyn DynamicTileJob>> {
        let mut jobs = Vec::new();
        while let Ok(job) = self.job_receiver.try_recv() {
            jobs.push(job);
        }
        jobs
    }
    
    
    pub fn create_voxel_query(&self) -> VoxelQuery {
        // For now, create a simple provider that returns None
        // In a real implementation, this would need a more sophisticated approach
        // to safely share heightfield data (e.g., using Arc<Mutex<>> or similar)
        let provider = Box::new(|_tile_x: i32, _tile_z: i32| -> Option<Heightfield> {
            None // Placeholder - would need proper synchronization for real heightfield access
        });
        
        VoxelQuery::new(
            Vec3::new(
                self.nav_mesh_params.origin[0], 
                self.nav_mesh_params.origin[1], 
                self.nav_mesh_params.origin[2]
            ),
            self.nav_mesh_params.tile_width,
            self.nav_mesh_params.tile_height,
            provider,
        )
    }
    
    pub fn set_nav_mesh(&mut self, nav_mesh: NavMesh) {
        self.nav_mesh = Some(nav_mesh);
        self.dirty = false;
    }
    
    pub fn voxel_tiles(&self) -> Vec<VoxelTile> {
        // This would return VoxelTile data from our dynamic tiles
        // For now, return empty vector as placeholder
        Vec::new()
    }
    
    pub fn recast_results(&self) -> Vec<String> {
        // This would return RcBuilderResult data from our dynamic tiles
        // For now, return empty vector as placeholder
        Vec::new()
    }
    
    pub fn build(&mut self) -> Result<()> {
        self.process_queue()?;
        self.rebuild_all_tiles()
    }
    
    /// Rebuild all tiles
    fn rebuild_all_tiles(&mut self) -> Result<()> {
        let tile_keys: Vec<i64> = self.tiles.keys().copied().collect();
        
        for key in tile_keys {
            if let Some(tile) = self.tiles.get_mut(&key) {
                if tile.needs_rebuild() {
                    tile.build()?;
                    self.dirty = true;
                }
            }
        }
        
        self.update_nav_mesh()?;
        Ok(())
    }
    
    /// This performs a full build of all tiles with optimized performance
    pub async fn build_async(&mut self) -> Result<bool> {
        // Process all pending jobs first (using new queue system)
        let _affected_coords = self.process_queue()?;
        
        // Get tiles that need rebuilding
        let tiles_to_build: Vec<i64> = self.tiles.iter()
            .filter_map(|(key, tile)| if tile.needs_rebuild() { Some(*key) } else { None })
            .collect();
        
        if tiles_to_build.is_empty() {
            return Ok(false);
        }
        
        println!("🏗️  Building {} tiles...", tiles_to_build.len());
        let start_time = std::time::Instant::now();
        
        // Process tiles in smaller batches to avoid memory issues and provide progress
        const BATCH_SIZE: usize = 10; // Process 10 tiles at a time
        let mut total_built = 0;
        
        for (batch_num, batch_keys) in tiles_to_build.chunks(BATCH_SIZE).enumerate() {
            println!("📦 Processing batch {} of {} ({} tiles)", 
                batch_num + 1, 
                tiles_to_build.len().div_ceil(BATCH_SIZE),
                batch_keys.len()
            );
            
            let mut batch_built = 0;
            for &key in batch_keys {
                if let Some(tile) = self.tiles.get_mut(&key) {
                    let tile_start = std::time::Instant::now();
                    match tile.build() {
                        Ok(built) => {
                            if built {
                                let tile_time = tile_start.elapsed();
                                println!("  ✅ Tile ({}, {}) built in {:?}", 
                                    tile.tile_x(), tile.tile_z(), tile_time);
                                batch_built += 1;
                                total_built += 1;
                            } else {
                                println!("  ⏭️  Tile ({}, {}) skipped (no changes)", 
                                    tile.tile_x(), tile.tile_z());
                            }
                        }
                        Err(e) => {
                            println!("  ❌ Tile ({}, {}) failed: {:?}", 
                                tile.tile_x(), tile.tile_z(), e);
                        }
                    }
                }
                
                // Yield control periodically to prevent blocking
                if total_built % 5 == 0 {
                    tokio::task::yield_now().await;
                }
            }
            
            println!("📦 Batch {} complete: {}/{} tiles built", 
                batch_num + 1, batch_built, batch_keys.len());
        }
        
        let total_time = start_time.elapsed();
        println!("🎉 Build complete: {}/{} tiles built in {:?}", 
            total_built, tiles_to_build.len(), total_time);
        
        if total_built > 0 {
            self.dirty = true;
            println!("🔄 Updating navigation mesh...");
            let nav_start = std::time::Instant::now();
            self.update_nav_mesh()?;
            let nav_time = nav_start.elapsed();
            println!("✅ Navigation mesh updated in {:?}", nav_time);
        }
        
        Ok(total_built > 0)
    }
    
    /// This performs an incremental update of modified tiles
    ///
    /// Note: In Rust, we execute the update synchronously within an async context.
    /// For true parallelism, tiles would need to be thread-safe (Send + Sync).
    pub async fn update_async(&mut self) -> Result<bool> {
        // Simply delegate to the synchronous update method
        // This implements pattern where Update(TaskFactory) processes tiles
        self.update()
    }
}

/// Statistics about the dynamic navigation mesh
#[derive(Debug, Clone)]
pub struct DynamicNavMeshStatistics {
    /// Total number of tiles
    pub total_tiles: usize,
    /// Number of dirty tiles needing rebuild
    pub dirty_tiles: usize,
    /// Number of pending jobs in the queue
    pub pending_jobs: usize,
    /// Update counter (increments with each change)
    pub update_counter: u64,
    /// Approximate memory usage in bytes
    pub memory_usage: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_dynamic_navmesh() {
        let config = DynamicNavMeshConfig::default();
        let result = DynamicNavMesh::new(config);
        assert!(result.is_ok());
    }
}
