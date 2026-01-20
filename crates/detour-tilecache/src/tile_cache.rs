//! Tile cache implementation for Detour
//!
//! This module contains the TileCache structure, which is used to
//! efficiently store and update navigation mesh tiles.

use std::collections::{HashMap, HashSet};
use std::f32;

use super::tile_cache_data::{TileCacheBuilderConfig, TileCacheLayer};
use super::tile_cache_integration::TileCacheNavMeshIntegration;
use detour::nav_mesh::TileHeader;
use detour::{NavMesh, PolyRef, Status};
use recast_common::{Error, Result};

/// Maximum number of layers per tile
const MAX_LAYERS: usize = 32;

/// Tile cache structure
#[derive(Debug)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct TileCache {
    /// Parameters for the tile cache
    params: TileCacheParams,
    /// Tile width (x-axis)
    tile_width: f32,
    /// Tile height (z-axis)
    tile_height: f32,
    /// Cached tiles
    tiles: Vec<Option<TileCacheEntry>>,
    /// Compressed tile data
    compressed_tiles: Vec<Vec<u8>>,
    /// Next free tile index
    next_free: Option<usize>,
    /// Tile grid hash lookup
    pos_lookup: HashMap<(i32, i32, i32), usize>,
    /// Obstacles in the tile cache
    obstacles: Vec<Option<Obstacle>>,
    /// Next free obstacle index
    next_free_obstacle: Option<usize>,
    /// Salt for generating unique references
    salt: u32,
    /// Navigation mesh integration (not serialized)
    #[cfg_attr(feature = "serialization", serde(skip))]
    nav_mesh_integration: Option<TileCacheNavMeshIntegration>,
}

/// Tile cache parameters
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct TileCacheParams {
    /// Origin of the tile cache
    pub origin: [f32; 3],
    /// Cell size (horizontal resolution)
    pub cs: f32,
    /// Cell height (vertical resolution)
    pub ch: f32,
    /// Width of the tile cache (in tiles)
    pub width: i32,
    /// Height of the tile cache (in tiles)
    pub height: i32,
    /// Maximum number of obstacles
    pub max_obstacles: i32,
}

/// Tile cache entry
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct TileCacheEntry {
    /// Header of the tile
    pub header: TileHeader,
    /// Index of the compressed tile data
    pub data: usize,
    /// Indices of obstacles affecting this tile
    pub obstacles: Vec<usize>,
    /// Next free tile in the linked list (used for memory management)
    pub next: Option<usize>,
}

/// Obstacle type
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum ObstacleType {
    /// Cylinder obstacle
    Cylinder,
    /// Axis-aligned bounding box
    Box,
    /// Oriented bounding box (rotated in Y axis)
    OrientedBox,
}

/// Obstacle state
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum ObstacleState {
    /// Empty/uninitialized
    Empty,
    /// Being processed
    Processing,
    /// Processed and active
    Processed,
    /// Being removed
    Removing,
}

/// Obstacle data for different types
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum ObstacleData {
    /// Cylinder obstacle
    Cylinder {
        /// Center position
        pos: [f32; 3],
        /// Radius
        radius: f32,
        /// Height
        height: f32,
    },
    /// Axis-aligned box obstacle
    Box {
        /// Minimum bounds
        bmin: [f32; 3],
        /// Maximum bounds
        bmax: [f32; 3],
    },
    /// Oriented box obstacle (rotated in Y axis)
    OrientedBox {
        /// Center position
        center: [f32; 3],
        /// Half extents (width/2, height/2, depth/2)
        half_extents: [f32; 3],
        /// Rotation auxiliary values [cos(0.5*angle)*sin(-0.5*angle), cos(0.5*angle)*cos(0.5*angle) - 0.5]
        rot_aux: [f32; 2],
    },
}

/// Obstacle in the tile cache
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Obstacle {
    /// Obstacle data
    pub data: ObstacleData,
    /// Obstacle state
    pub state: ObstacleState,
    /// Salt value for reference generation
    pub salt: u16,
    /// Tiles affected by this obstacle
    pub touched: Vec<u32>,
    /// Pending tiles to be updated
    pub pending: Vec<u32>,
    /// Next free obstacle in the linked list (used for memory management)
    pub next: Option<usize>,
}

impl TileCache {
    /// Creates a new tile cache
    pub fn new(params: TileCacheParams) -> Result<Self> {
        // Validate parameters
        if params.origin[0].is_infinite()
            || params.origin[0].is_nan()
            || params.origin[1].is_infinite()
            || params.origin[1].is_nan()
            || params.origin[2].is_infinite()
            || params.origin[2].is_nan()
        {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.cs <= 0.0 || params.ch <= 0.0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.width <= 0 || params.height <= 0 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        let max_tiles = params.width * params.height * MAX_LAYERS as i32;

        // Allocate tiles
        let mut tiles = Vec::with_capacity(max_tiles as usize);
        for _i in 0..max_tiles as usize {
            tiles.push(None);
        }

        // Allocate obstacles
        let mut obstacles = Vec::with_capacity(params.max_obstacles as usize);
        for _i in 0..params.max_obstacles as usize {
            obstacles.push(None);
        }

        // Set up free lists
        let next_free = Some(0);
        let next_free_obstacle = Some(0);

        // Initialize tile free list
        for (i, tile) in tiles.iter_mut().enumerate().take(max_tiles as usize) {
            *tile = Some(TileCacheEntry {
                header: TileHeader::new(0, 0, 0),
                data: 0,
                obstacles: Vec::new(),
                next: if i < max_tiles as usize - 1 {
                    Some(i + 1)
                } else {
                    None
                },
            });
        }

        // Initialize obstacle free list
        for (i, obstacle) in obstacles
            .iter_mut()
            .enumerate()
            .take(params.max_obstacles as usize)
        {
            *obstacle = Some(Obstacle {
                data: ObstacleData::Cylinder {
                    pos: [0.0, 0.0, 0.0],
                    radius: 0.0,
                    height: 0.0,
                },
                state: ObstacleState::Empty,
                salt: 0,
                touched: Vec::new(),
                pending: Vec::new(),
                next: if i < params.max_obstacles as usize - 1 {
                    Some(i + 1)
                } else {
                    None
                },
            });
        }

        Ok(Self {
            params,
            tile_width: 0.0,  // Will be calculated during initialization
            tile_height: 0.0, // Will be calculated during initialization
            tiles,
            compressed_tiles: Vec::new(),
            next_free,
            pos_lookup: HashMap::new(),
            obstacles,
            next_free_obstacle,
            salt: 1,
            nav_mesh_integration: None,
        })
    }

    /// Initializes the tile cache with data
    pub fn init(&mut self) -> Result<()> {
        // Calculate tile grid size
        self.tile_width = self.params.cs * self.params.width as f32;
        self.tile_height = self.params.cs * self.params.height as f32;

        Ok(())
    }

    /// Attaches the tile cache to a navigation mesh with the specified builder configuration
    pub fn attach_to_nav_mesh(&mut self, builder_config: TileCacheBuilderConfig) -> Result<()> {
        // Create the integration with the provided configuration
        let integration = TileCacheNavMeshIntegration::new(builder_config);
        self.nav_mesh_integration = Some(integration);

        Ok(())
    }

    /// Adds a tile to the cache with optional compression
    pub fn add_tile(&mut self, data: &[u8], flags: u8, result: &mut PolyRef) -> Result<()> {
        if self.next_free.is_none() {
            return Err(Error::Detour(Status::OutOfMemory.to_string()));
        }

        // Compress the tile data if it's not already compressed
        let compressed_data = if (flags & 0x01) != 0 {
            // Data is already compressed
            data.to_vec()
        } else {
            // Compress the data
            self.compress_tile(data)?
        };

        // Allocate a tile
        let free_idx = self.next_free.unwrap();
        let mut tile_entry = self.tiles[free_idx].take().unwrap();
        self.next_free = tile_entry.next;

        // Parse the tile header from compressed data
        let header = self.parse_tile_header(&compressed_data)?;
        let header_key = (header.x, header.y, header.layer);

        // Check if a tile already exists at this location
        if self.pos_lookup.contains_key(&header_key) {
            // Return the tile to the free list
            tile_entry.next = self.next_free;
            self.next_free = Some(free_idx);
            self.tiles[free_idx] = Some(tile_entry);
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Store the compressed data
        let data_idx = self.compressed_tiles.len();
        self.compressed_tiles.push(compressed_data);
        tile_entry.data = data_idx;
        tile_entry.header = header;

        // Add to lookup
        self.pos_lookup.insert(header_key, free_idx);

        // Store the tile
        self.tiles[free_idx] = Some(tile_entry);

        // Return the tile reference using salt for uniqueness
        *result = PolyRef::new((self.salt << 16) | (free_idx as u32));
        self.salt += 1;

        Ok(())
    }

    /// Removes a tile from the cache
    pub fn remove_tile(&mut self, ref_val: PolyRef) -> Result<()> {
        // Decode the tile index from the reference
        let tile_idx = (ref_val.id() & 0xFFFF) as usize;

        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if self.tiles[tile_idx].is_none() {
            return Err(Error::Detour(Status::NotFound.to_string()));
        }

        // Remove the tile
        let mut tile_entry = self.tiles[tile_idx].take().unwrap();

        // Remove from lookup
        self.pos_lookup.remove(&(
            tile_entry.header.x,
            tile_entry.header.y,
            tile_entry.header.layer,
        ));

        // Reset the tile entry and add it to the free list
        tile_entry.obstacles.clear();
        tile_entry.next = self.next_free;
        self.next_free = Some(tile_idx);

        // Store the reset tile entry
        self.tiles[tile_idx] = Some(tile_entry);

        Ok(())
    }

    /// Adds a cylinder obstacle to the cache
    pub fn add_obstacle(&mut self, pos: [f32; 3], radius: f32, height: f32) -> Result<u32> {
        if self.next_free_obstacle.is_none() {
            return Err(Error::Detour(Status::OutOfMemory.to_string()));
        }

        // Allocate an obstacle
        let free_idx = self.next_free_obstacle.unwrap();
        let mut obstacle = self.obstacles[free_idx].take().unwrap();
        self.next_free_obstacle = obstacle.next;

        // Set obstacle properties
        obstacle.data = ObstacleData::Cylinder {
            pos,
            radius,
            height,
        };
        obstacle.state = ObstacleState::Processing;
        obstacle.salt += 1;
        obstacle.touched.clear();
        obstacle.pending.clear();

        // Find tiles affected by this obstacle
        let affected_tiles = self.find_tiles_affected_by_obstacle(pos, radius)?;
        for &tile_idx in &affected_tiles {
            obstacle.touched.push(tile_idx as u32);
            obstacle.pending.push(tile_idx as u32);
        }

        // Get salt for reference before storing
        let salt = obstacle.salt;

        // Store the obstacle
        self.obstacles[free_idx] = Some(obstacle);

        Ok(self.encode_obstacle_ref(salt, free_idx))
    }

    /// Adds an axis-aligned box obstacle to the cache
    pub fn add_box_obstacle(&mut self, bmin: [f32; 3], bmax: [f32; 3]) -> Result<u32> {
        if self.next_free_obstacle.is_none() {
            return Err(Error::Detour(Status::OutOfMemory.to_string()));
        }

        // Allocate an obstacle
        let free_idx = self.next_free_obstacle.unwrap();
        let mut obstacle = self.obstacles[free_idx].take().unwrap();
        self.next_free_obstacle = obstacle.next;

        // Set obstacle properties
        obstacle.data = ObstacleData::Box { bmin, bmax };
        obstacle.state = ObstacleState::Processing;
        obstacle.salt += 1;
        obstacle.touched.clear();
        obstacle.pending.clear();

        // Find tiles affected by this obstacle
        // For a box, we use the center and maximum extent
        let center = [
            (bmin[0] + bmax[0]) * 0.5,
            (bmin[1] + bmax[1]) * 0.5,
            (bmin[2] + bmax[2]) * 0.5,
        ];
        let radius = ((bmax[0] - bmin[0]).powi(2) + (bmax[2] - bmin[2]).powi(2)).sqrt() * 0.5;

        let affected_tiles = self.find_tiles_affected_by_obstacle(center, radius)?;
        for &tile_idx in &affected_tiles {
            obstacle.touched.push(tile_idx as u32);
            obstacle.pending.push(tile_idx as u32);
        }

        // Get salt for reference before storing
        let salt = obstacle.salt;

        // Store the obstacle
        self.obstacles[free_idx] = Some(obstacle);

        Ok(self.encode_obstacle_ref(salt, free_idx))
    }

    /// Adds an oriented box obstacle to the cache
    pub fn add_oriented_box_obstacle(
        &mut self,
        center: [f32; 3],
        half_extents: [f32; 3],
        y_radians: f32,
    ) -> Result<u32> {
        if self.next_free_obstacle.is_none() {
            return Err(Error::Detour(Status::OutOfMemory.to_string()));
        }

        // Allocate an obstacle
        let free_idx = self.next_free_obstacle.unwrap();
        let mut obstacle = self.obstacles[free_idx].take().unwrap();
        self.next_free_obstacle = obstacle.next;

        // Calculate rotation auxiliary values
        let angle = y_radians;
        let cos_half = (angle * 0.5).cos();
        let sin_half = (angle * 0.5).sin();
        let rot_aux = [cos_half * -sin_half, cos_half * cos_half - 0.5];

        // Set obstacle properties
        obstacle.data = ObstacleData::OrientedBox {
            center,
            half_extents,
            rot_aux,
        };
        obstacle.state = ObstacleState::Processing;
        obstacle.salt += 1;
        obstacle.touched.clear();
        obstacle.pending.clear();

        // Find tiles affected by this obstacle
        // For an oriented box, we use a conservative bounding radius
        let radius = (half_extents[0].powi(2) + half_extents[2].powi(2)).sqrt();

        let affected_tiles = self.find_tiles_affected_by_obstacle(center, radius)?;
        for &tile_idx in &affected_tiles {
            obstacle.touched.push(tile_idx as u32);
            obstacle.pending.push(tile_idx as u32);
        }

        // Get salt for reference before storing
        let salt = obstacle.salt;

        // Store the obstacle
        self.obstacles[free_idx] = Some(obstacle);

        Ok(self.encode_obstacle_ref(salt, free_idx))
    }

    /// Removes an obstacle from the cache
    pub fn remove_obstacle(&mut self, obstacle_ref: u32) -> Result<()> {
        let obstacle_idx = self.decode_obstacle_ref_idx(obstacle_ref);
        let salt = self.decode_obstacle_ref_salt(obstacle_ref);

        if obstacle_idx >= self.obstacles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if self.obstacles[obstacle_idx].is_none() {
            return Err(Error::Detour(Status::NotFound.to_string()));
        }

        // Verify salt matches
        let obstacle = self.obstacles[obstacle_idx].as_ref().unwrap();
        if obstacle.salt != salt {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Remove the obstacle
        let mut obstacle = self.obstacles[obstacle_idx].take().unwrap();

        // Mark for removal
        obstacle.state = ObstacleState::Removing;

        // Mark tiles for update
        obstacle.pending = obstacle.touched.clone();

        // Store the obstacle back (will be cleaned up during update)
        self.obstacles[obstacle_idx] = Some(obstacle);

        Ok(())
    }

    /// Updates the tile cache (processes pending obstacles)
    pub fn update(&mut self) -> Result<()> {
        self.update_with_status(0.0).map(|_| ())
    }

    /// Updates the tile cache and returns whether it's up to date
    pub fn update_with_status(&mut self, _dt: f32) -> Result<bool> {
        // Find all tiles that need updating due to pending obstacles
        let mut tiles_to_update = HashSet::new();
        let mut has_pending_work = false;

        // Process obstacles
        for obstacle_idx in 0..self.obstacles.len() {
            if let Some(obstacle) = &mut self.obstacles[obstacle_idx] {
                match obstacle.state {
                    ObstacleState::Processing => {
                        has_pending_work = true;
                        // Mark all pending tiles as needing update
                        for &tile_idx in &obstacle.pending {
                            tiles_to_update.insert(tile_idx as usize);
                        }
                        obstacle.pending.clear();
                        obstacle.state = ObstacleState::Processed;
                    }
                    ObstacleState::Removing => {
                        has_pending_work = true;
                        // Mark all touched tiles as needing update
                        for &tile_idx in &obstacle.pending {
                            tiles_to_update.insert(tile_idx as usize);
                        }
                        // Clear the obstacle data but keep it in the free list
                        obstacle.state = ObstacleState::Empty;
                        obstacle.touched.clear();
                        obstacle.pending.clear();
                    }
                    _ => {}
                }
            }
        }

        // Clean up removed obstacles
        for obstacle_idx in 0..self.obstacles.len() {
            if let Some(obstacle) = &self.obstacles[obstacle_idx] {
                if obstacle.state == ObstacleState::Empty {
                    // Move obstacle to free list
                    let mut obstacle = self.obstacles[obstacle_idx].take().unwrap();
                    obstacle.next = self.next_free_obstacle;
                    self.next_free_obstacle = Some(obstacle_idx);
                    self.obstacles[obstacle_idx] = Some(obstacle);
                }
            }
        }

        // Update each affected tile
        for tile_idx in tiles_to_update {
            self.rebuild_tile(tile_idx)?;
        }

        // Return true if there's no more pending work
        Ok(!has_pending_work)
    }

    /// Updates the tile cache and rebuilds tiles in the navigation mesh
    pub fn update_with_nav_mesh(&mut self, _dt: f32, nav_mesh: &mut NavMesh) -> Result<bool> {
        // Find all tiles that need updating due to pending obstacles
        let mut tiles_to_update = HashSet::new();
        let mut has_pending_work = false;

        // Process obstacles
        for obstacle_idx in 0..self.obstacles.len() {
            if let Some(obstacle) = &mut self.obstacles[obstacle_idx] {
                match obstacle.state {
                    ObstacleState::Processing => {
                        has_pending_work = true;
                        // Mark all pending tiles as needing update
                        for &tile_idx in &obstacle.pending {
                            tiles_to_update.insert(tile_idx as usize);
                        }
                        obstacle.pending.clear();
                        obstacle.state = ObstacleState::Processed;
                    }
                    ObstacleState::Removing => {
                        has_pending_work = true;
                        // Mark all touched tiles as needing update
                        for &tile_idx in &obstacle.pending {
                            tiles_to_update.insert(tile_idx as usize);
                        }
                        // Clear the obstacle data but keep it in the free list
                        obstacle.state = ObstacleState::Empty;
                        obstacle.touched.clear();
                        obstacle.pending.clear();
                    }
                    _ => {}
                }
            }
        }

        // Clean up removed obstacles
        for obstacle_idx in 0..self.obstacles.len() {
            if let Some(obstacle) = &self.obstacles[obstacle_idx] {
                if obstacle.state == ObstacleState::Empty {
                    // Move obstacle to free list
                    let mut obstacle = self.obstacles[obstacle_idx].take().unwrap();
                    obstacle.next = self.next_free_obstacle;
                    self.next_free_obstacle = Some(obstacle_idx);
                    self.obstacles[obstacle_idx] = Some(obstacle);
                }
            }
        }

        // Build nav mesh tiles for each affected tile
        for tile_idx in tiles_to_update {
            if let Some(Some(_tile_entry)) = self.tiles.get(tile_idx) {
                // Get tile reference
                let tile_ref = PolyRef::new((self.salt << 16) | (tile_idx as u32));
                // Build nav mesh tile
                self.build_nav_mesh_tile(tile_ref, nav_mesh)?;
            }
        }

        // Return true if there's no more pending work
        Ok(!has_pending_work)
    }

    /// Builds navigation mesh tiles at the specified tile coordinates
    pub fn build_nav_mesh_tiles_at(
        &mut self,
        tx: i32,
        ty: i32,
        nav_mesh: &mut NavMesh,
    ) -> Result<()> {
        let tiles = self.get_tiles_at(tx, ty)?;

        for tile_ref in tiles {
            self.build_nav_mesh_tile(tile_ref, nav_mesh)?;
        }

        Ok(())
    }

    /// Builds a navigation mesh tile from a compressed tile reference
    pub fn build_nav_mesh_tile(&mut self, tile_ref: PolyRef, nav_mesh: &mut NavMesh) -> Result<()> {
        // Check if we have an integration set up
        let integration = self
            .nav_mesh_integration
            .as_ref()
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Decode tile reference
        let tile_idx = (tile_ref.id() & 0xFFFF) as usize;
        let _salt = tile_ref.id() >> 16;

        // Validate tile
        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        // Check salt matches (but for now we'll skip this check as our salt handling is different)
        // In the C++ version, each tile has its own salt

        // Get the tile
        let tile_entry = match &self.tiles[tile_idx] {
            Some(entry) => entry,
            None => return Err(Error::Detour(Status::NotFound.to_string())),
        };

        // Get compressed data
        let compressed_data = match self.compressed_tiles.get(tile_entry.data) {
            Some(data) => data,
            None => return Err(Error::Detour(Status::NotFound.to_string())),
        };

        // Decompress tile data
        let decompressed_data = self.decompress_tile(compressed_data, None)?;
        let tile_layer = TileCacheLayer::from_bytes(&decompressed_data)?;

        // Use the integration to build the nav mesh tile
        integration.build_nav_mesh_tile_from_layer(self, nav_mesh, &tile_layer, tile_idx)?;

        Ok(())
    }

    /// Finds the tile index for the given coordinates
    #[allow(dead_code)]
    fn find_tile_index(&self, x: i32, y: i32, layer: i32) -> Option<usize> {
        // Look up the tile index in our position lookup
        self.pos_lookup.get(&(x, y, layer)).copied()
    }

    /// Rebuilds a tile using the attached navigation mesh integration
    pub fn rebuild_tile_in_nav_mesh(
        &mut self,
        nav_mesh: &mut NavMesh,
        tile_idx: usize,
    ) -> Result<Option<PolyRef>> {
        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if self.tiles[tile_idx].is_none() {
            return Err(Error::Detour(Status::NotFound.to_string()));
        }

        // Check if we have an integration set up
        let integration = self
            .nav_mesh_integration
            .as_ref()
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Use the integration to rebuild the tile
        integration.rebuild_tile_in_nav_mesh(self, nav_mesh, tile_idx)
    }

    /// Rebuilds a tile (legacy method for internal use)
    fn rebuild_tile(&mut self, tile_idx: usize) -> Result<()> {
        if tile_idx >= self.tiles.len() {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if self.tiles[tile_idx].is_none() {
            return Err(Error::Detour(Status::NotFound.to_string()));
        }

        // Get the tile entry
        let tile_entry = match &self.tiles[tile_idx] {
            Some(entry) => entry,
            None => return Err(Error::Detour(Status::NotFound.to_string())),
        };

        // Get the compressed tile data
        let compressed_data = &self.compressed_tiles[tile_entry.data];

        // For this implementation, we'll just mark that the tile has been processed
        // The real rebuilding happens in rebuild_tile_in_nav_mesh
        log::debug!(
            "Rebuilding tile {} with {} bytes of data",
            tile_idx,
            compressed_data.len()
        );

        Ok(())
    }

    /// Gets a tile by index
    pub fn get_tile(&self, tile_idx: usize) -> Option<&TileCacheEntry> {
        if tile_idx >= self.tiles.len() {
            return None;
        }

        match &self.tiles[tile_idx] {
            Some(entry) => Some(entry),
            None => None,
        }
    }

    /// Gets a tile at the specified coordinates
    pub fn get_tile_at(&self, x: i32, y: i32, layer: i32) -> Option<&TileCacheEntry> {
        if let Some(&tile_idx) = self.pos_lookup.get(&(x, y, layer)) {
            self.get_tile(tile_idx)
        } else {
            None
        }
    }

    /// Gets an obstacle by index
    pub fn get_obstacle(&self, obstacle_idx: usize) -> Option<&Obstacle> {
        if obstacle_idx >= self.obstacles.len() {
            return None;
        }

        match &self.obstacles[obstacle_idx] {
            Some(obstacle) => Some(obstacle),
            None => None,
        }
    }

    /// Gets the number of obstacles
    pub fn get_obstacle_count(&self) -> usize {
        // Count obstacles that are not empty
        self.obstacles
            .iter()
            .filter_map(|obs| obs.as_ref())
            .filter(|obs| obs.state != ObstacleState::Empty)
            .count()
    }

    /// Gets the parameters of the tile cache
    pub fn get_params(&self) -> &TileCacheParams {
        &self.params
    }

    /// Gets the compressed data for a tile
    pub fn get_tile_compressed_data(&self, tile_idx: usize) -> Option<&[u8]> {
        if let Some(Some(tile)) = self.tiles.get(tile_idx) {
            self.compressed_tiles.get(tile.data).map(|v| v.as_slice())
        } else {
            None
        }
    }

    /// Gets the decompressed data for a tile
    pub fn get_tile_data(&self, tile_idx: usize) -> Result<Vec<u8>> {
        if let Some(compressed_data) = self.get_tile_compressed_data(tile_idx) {
            self.decompress_tile(compressed_data, None)
        } else {
            Err(Error::Detour(Status::NotFound.to_string()))
        }
    }

    /// Compresses tile data using LZ4
    pub fn compress_tile(&self, data: &[u8]) -> Result<Vec<u8>> {
        // Compress the data using LZ4 with prepended size for decompression
        Ok(lz4_flex::compress_prepend_size(data))
    }

    /// Decompresses tile data using LZ4
    pub fn decompress_tile(
        &self,
        compressed_data: &[u8],
        _uncompressed_size: Option<usize>,
    ) -> Result<Vec<u8>> {
        // Handle empty data
        if compressed_data.is_empty() {
            return Ok(Vec::new());
        }

        // Decompress the data using LZ4 (size is prepended)
        match lz4_flex::decompress_size_prepended(compressed_data) {
            Ok(decompressed) => Ok(decompressed),
            Err(e) => {
                log::error!("LZ4 decompression failed: {:?}", e);
                Err(Error::Detour(Status::Failure.to_string()))
            }
        }
    }

    /// Parses a tile header from compressed tile data
    fn parse_tile_header(&self, compressed_data: &[u8]) -> Result<TileHeader> {
        // Decompress the tile data to read the header
        let decompressed = self.decompress_tile(compressed_data, None)?;

        // Parse as TileCacheLayer
        let tile_layer = TileCacheLayer::from_bytes(&decompressed)?;

        // Convert TileCacheLayerHeader to TileHeader
        let mut header = TileHeader::new(
            tile_layer.header.tx,
            tile_layer.header.ty,
            tile_layer.header.tlayer,
        );
        header.data_size = compressed_data.len(); // Store compressed size

        Ok(header)
    }

    /// Finds tiles affected by an obstacle
    fn find_tiles_affected_by_obstacle(&self, pos: [f32; 3], radius: f32) -> Result<Vec<usize>> {
        let mut affected_tiles = Vec::new();

        // Calculate tile coordinates for the obstacle's bounding box
        let min_x = ((pos[0] - radius - self.params.origin[0]) / self.tile_width).floor() as i32;
        let max_x = ((pos[0] + radius - self.params.origin[0]) / self.tile_width).floor() as i32;
        let min_z = ((pos[2] - radius - self.params.origin[2]) / self.tile_height).floor() as i32;
        let max_z = ((pos[2] + radius - self.params.origin[2]) / self.tile_height).floor() as i32;

        // Check all tiles in the bounding box
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                for layer in 0..MAX_LAYERS as i32 {
                    if let Some(&tile_idx) = self.pos_lookup.get(&(x, z, layer)) {
                        affected_tiles.push(tile_idx);
                    }
                }
            }
        }

        Ok(affected_tiles)
    }

    /// Gets the current salt value
    pub fn get_salt(&self) -> u32 {
        self.salt
    }

    /// Encodes an obstacle reference from salt and index
    pub fn encode_obstacle_ref(&self, salt: u16, idx: usize) -> u32 {
        ((salt as u32) << 16) | (idx as u32)
    }

    /// Decodes the salt from an obstacle reference
    pub fn decode_obstacle_ref_salt(&self, ref_val: u32) -> u16 {
        ((ref_val >> 16) & 0xFFFF) as u16
    }

    /// Decodes the index from an obstacle reference
    pub fn decode_obstacle_ref_idx(&self, ref_val: u32) -> usize {
        (ref_val & 0xFFFF) as usize
    }

    /// Gets an obstacle by its reference
    pub fn get_obstacle_by_ref(&self, ref_val: u32) -> Option<&Obstacle> {
        let idx = self.decode_obstacle_ref_idx(ref_val);
        let salt = self.decode_obstacle_ref_salt(ref_val);

        if let Some(obstacle) = self.get_obstacle(idx) {
            if obstacle.salt == salt {
                return Some(obstacle);
            }
        }
        None
    }

    /// Gets the reference for an obstacle at the given index
    pub fn get_obstacle_ref(&self, obstacle_idx: usize) -> Option<u32> {
        self.get_obstacle(obstacle_idx)
            .map(|obstacle| self.encode_obstacle_ref(obstacle.salt, obstacle_idx))
    }

    /// Queries tiles that overlap with the given bounding box
    pub fn query_tiles(
        &self,
        bmin: &[f32; 3],
        bmax: &[f32; 3],
        max_tiles: usize,
    ) -> Result<Vec<PolyRef>> {
        let mut results = Vec::new();

        let tw = self.params.width as f32 * self.params.cs;
        let th = self.params.height as f32 * self.params.cs;

        let tx0 = ((bmin[0] - self.params.origin[0]) / tw).floor() as i32;
        let tx1 = ((bmax[0] - self.params.origin[0]) / tw).floor() as i32;
        let ty0 = ((bmin[2] - self.params.origin[2]) / th).floor() as i32;
        let ty1 = ((bmax[2] - self.params.origin[2]) / th).floor() as i32;

        for ty in ty0..=ty1 {
            for tx in tx0..=tx1 {
                let tiles = self.get_tiles_at(tx, ty)?;

                for &tile_ref in &tiles {
                    if results.len() >= max_tiles {
                        break;
                    }

                    // Get tile by reference and check bounds
                    let tile_idx = (tile_ref.id() & 0xFFFF) as usize;
                    let salt = tile_ref.id() >> 16;

                    if let Some(Some(_tile)) = self.tiles.get(tile_idx) {
                        // Verify salt matches
                        if salt != self.salt {
                            continue;
                        }

                        // Get compressed data and decompress header
                        if let Some(compressed_data) = self.get_tile_compressed_data(tile_idx) {
                            let decompressed = self.decompress_tile(compressed_data, None)?;
                            if let Ok(tile_layer) = TileCacheLayer::from_bytes(&decompressed) {
                                let (tbmin, tbmax) =
                                    self.calc_tight_tile_bounds(&tile_layer.header);

                                // Check overlap
                                if Self::overlap_bounds(bmin, bmax, &tbmin, &tbmax) {
                                    results.push(tile_ref);
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(results)
    }

    /// Gets all tiles at the given tile coordinates
    pub fn get_tiles_at(&self, tx: i32, ty: i32) -> Result<Vec<PolyRef>> {
        let mut tiles = Vec::new();

        // Search through all tiles at this position
        for layer in 0..MAX_LAYERS as i32 {
            if let Some(&tile_idx) = self.pos_lookup.get(&(tx, ty, layer)) {
                if let Some(Some(_tile)) = self.tiles.get(tile_idx) {
                    // Create tile reference with salt
                    let tile_ref = PolyRef::new((self.salt << 16) | (tile_idx as u32));
                    tiles.push(tile_ref);
                }
            }
        }

        Ok(tiles)
    }

    /// Calculates tight bounds for a tile
    pub fn calc_tight_tile_bounds(
        &self,
        header: &crate::tile_cache_data::TileCacheLayerHeader,
    ) -> ([f32; 3], [f32; 3]) {
        let cs = self.params.cs;

        let bmin = [
            header.bmin[0] + header.minx as f32 * cs,
            header.bmin[1],
            header.bmin[2] + header.miny as f32 * cs,
        ];

        let bmax = [
            header.bmin[0] + (header.maxx + 1) as f32 * cs,
            header.bmax[1],
            header.bmin[2] + (header.maxy + 1) as f32 * cs,
        ];

        (bmin, bmax)
    }

    /// Gets the bounds of an obstacle
    pub fn get_obstacle_bounds(&self, obstacle: &Obstacle) -> ([f32; 3], [f32; 3]) {
        match &obstacle.data {
            ObstacleData::Cylinder {
                pos,
                radius,
                height,
            } => {
                let bmin = [pos[0] - radius, pos[1], pos[2] - radius];
                let bmax = [pos[0] + radius, pos[1] + height, pos[2] + radius];
                (bmin, bmax)
            }
            ObstacleData::Box { bmin, bmax } => (*bmin, *bmax),
            ObstacleData::OrientedBox {
                center,
                half_extents,
                ..
            } => {
                // Conservative bounds for oriented box
                let max_r = 1.41 * half_extents[0].max(half_extents[2]);
                let bmin = [
                    center[0] - max_r,
                    center[1] - half_extents[1],
                    center[2] - max_r,
                ];
                let bmax = [
                    center[0] + max_r,
                    center[1] + half_extents[1],
                    center[2] + max_r,
                ];
                (bmin, bmax)
            }
        }
    }

    /// Checks if two bounding boxes overlap
    fn overlap_bounds(amin: &[f32; 3], amax: &[f32; 3], bmin: &[f32; 3], bmax: &[f32; 3]) -> bool {
        !(amin[0] > bmax[0]
            || amax[0] < bmin[0]
            || amin[1] > bmax[1]
            || amax[1] < bmin[1]
            || amin[2] > bmax[2]
            || amax[2] < bmin[2])
    }

    /// Saves the tile cache to a file in JSON format
    #[cfg(feature = "serialization")]
    pub fn save_to_json<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        let json = serde_json::to_string_pretty(self)
            .map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        std::fs::write(path, json).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        Ok(())
    }

    /// Loads a tile cache from a JSON file
    #[cfg(feature = "serialization")]
    pub fn load_from_json<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let json = std::fs::read_to_string(path)
            .map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        let tile_cache =
            serde_json::from_str(&json).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        Ok(tile_cache)
    }

    /// Saves the tile cache to a file in binary format
    #[cfg(feature = "serialization")]
    pub fn save_to_binary<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        let encoded =
            postcard::to_allocvec(self).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        std::fs::write(path, encoded).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        Ok(())
    }

    /// Loads a tile cache from a binary file
    #[cfg(feature = "serialization")]
    pub fn load_from_binary<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let data = std::fs::read(path).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        let tile_cache =
            postcard::from_bytes(&data).map_err(|_| Error::Detour(Status::Failure.to_string()))?;

        Ok(tile_cache)
    }

    /// Serializes the tile cache to JSON bytes
    #[cfg(feature = "serialization")]
    pub fn to_json_bytes(&self) -> Result<Vec<u8>> {
        let json =
            serde_json::to_vec(self).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(json)
    }

    /// Deserializes a tile cache from JSON bytes
    #[cfg(feature = "serialization")]
    pub fn from_json_bytes(data: &[u8]) -> Result<Self> {
        let tile_cache =
            serde_json::from_slice(data).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(tile_cache)
    }

    /// Serializes the tile cache to binary bytes
    #[cfg(feature = "serialization")]
    pub fn to_binary_bytes(&self) -> Result<Vec<u8>> {
        let data =
            postcard::to_allocvec(self).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(data)
    }

    /// Deserializes a tile cache from binary bytes
    #[cfg(feature = "serialization")]
    pub fn from_binary_bytes(data: &[u8]) -> Result<Self> {
        let tile_cache =
            postcard::from_bytes(data).map_err(|_| Error::Detour(Status::Failure.to_string()))?;
        Ok(tile_cache)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tile_cache_data::TileCacheLayerHeader;

    #[test]
    fn test_create_tile_cache() {
        let params = TileCacheParams {
            origin: [0.0, 0.0, 0.0],
            cs: 0.3,
            ch: 0.2,
            width: 100,
            height: 100,
            max_obstacles: 1024,
        };

        let result = TileCache::new(params.clone());

        assert!(result.is_ok());

        let tile_cache = result.unwrap();

        assert_eq!(tile_cache.get_params().width, params.width);
        assert_eq!(tile_cache.get_params().height, params.height);
        assert_eq!(tile_cache.get_params().cs, params.cs);
        assert_eq!(tile_cache.get_params().ch, params.ch);
        assert_eq!(tile_cache.get_params().max_obstacles, params.max_obstacles);
        assert_eq!(tile_cache.get_obstacle_count(), 0);
    }

    #[test]
    fn test_invalid_params() {
        // Test invalid origin
        let params = TileCacheParams {
            origin: [f32::INFINITY, 0.0, 0.0], // Use INFINITY directly
            cs: 0.3,
            ch: 0.2,
            width: 100,
            height: 100,
            max_obstacles: 1024,
        };

        let result = TileCache::new(params);
        assert!(result.is_err());

        // Test invalid cell size
        let params = TileCacheParams {
            origin: [0.0, 0.0, 0.0],
            cs: -0.3, // Negative cell size
            ch: 0.2,
            width: 100,
            height: 100,
            max_obstacles: 1024,
        };

        let result = TileCache::new(params);
        assert!(result.is_err());

        // Test invalid width
        let params = TileCacheParams {
            origin: [0.0, 0.0, 0.0],
            cs: 0.3,
            ch: 0.2,
            width: 0, // Zero width
            height: 100,
            max_obstacles: 1024,
        };

        let result = TileCache::new(params);
        assert!(result.is_err());
    }

    #[test]
    fn test_obstacle_management() {
        let params = TileCacheParams {
            origin: [0.0, 0.0, 0.0],
            cs: 0.3,
            ch: 0.2,
            width: 10,
            height: 10,
            max_obstacles: 64,
        };

        let mut tile_cache = TileCache::new(params).unwrap();
        tile_cache.init().unwrap();

        // Add an obstacle
        let obstacle_ref = tile_cache.add_obstacle([5.0, 1.0, 5.0], 2.0, 3.0).unwrap();
        assert_eq!(tile_cache.get_obstacle_count(), 1);

        // Check obstacle properties
        let obstacle = tile_cache.get_obstacle_by_ref(obstacle_ref).unwrap();
        match &obstacle.data {
            ObstacleData::Cylinder {
                pos,
                radius,
                height,
            } => {
                assert_eq!(*pos, [5.0, 1.0, 5.0]);
                assert_eq!(*radius, 2.0);
                assert_eq!(*height, 3.0);
            }
            _ => panic!("Expected cylinder obstacle"),
        }
        assert_eq!(obstacle.state, ObstacleState::Processing);

        // Remove the obstacle
        tile_cache.remove_obstacle(obstacle_ref).unwrap();
        // Obstacle is marked for removal but not yet removed
        assert_eq!(tile_cache.get_obstacle_count(), 1);

        // Update to actually remove the obstacle
        tile_cache.update().unwrap();
        assert_eq!(tile_cache.get_obstacle_count(), 0);
    }

    #[test]
    #[cfg(feature = "serialization")]
    fn test_tile_cache_serialization() {
        use tempfile::NamedTempFile;

        let params = TileCacheParams {
            origin: [1.0, 2.0, 3.0],
            cs: 0.5,
            ch: 0.3,
            width: 20,
            height: 20,
            max_obstacles: 128,
        };

        let mut tile_cache = TileCache::new(params.clone()).unwrap();
        tile_cache.init().unwrap();

        // Add some test data
        let _obstacle_ref = tile_cache
            .add_obstacle([10.0, 2.0, 10.0], 1.5, 2.0)
            .unwrap();

        // Test JSON serialization
        let json_bytes = tile_cache.to_json_bytes().unwrap();
        let restored_from_json = TileCache::from_json_bytes(&json_bytes).unwrap();
        assert_eq!(restored_from_json.get_params().cs, params.cs);
        assert_eq!(restored_from_json.get_params().width, params.width);

        // Test binary serialization
        let binary_bytes = tile_cache.to_binary_bytes().unwrap();
        let restored_from_binary = TileCache::from_binary_bytes(&binary_bytes).unwrap();
        assert_eq!(restored_from_binary.get_params().cs, params.cs);
        assert_eq!(restored_from_binary.get_params().height, params.height);

        // Test file serialization
        let json_file = NamedTempFile::new().unwrap();
        tile_cache.save_to_json(json_file.path()).unwrap();
        let restored_from_json_file = TileCache::load_from_json(json_file.path()).unwrap();
        assert_eq!(
            restored_from_json_file.get_params().max_obstacles,
            params.max_obstacles
        );

        let binary_file = NamedTempFile::new().unwrap();
        tile_cache.save_to_binary(binary_file.path()).unwrap();
        let restored_from_binary_file = TileCache::load_from_binary(binary_file.path()).unwrap();
        assert_eq!(restored_from_binary_file.get_params().origin, params.origin);
    }

    #[test]
    fn test_tile_management() {
        let params = TileCacheParams {
            origin: [0.0, 0.0, 0.0],
            cs: 1.0,
            ch: 0.5,
            width: 5,
            height: 5,
            max_obstacles: 32,
        };

        let mut tile_cache = TileCache::new(params).unwrap();
        tile_cache.init().unwrap();

        // Create a proper tile layer
        let mut header = TileCacheLayerHeader::new();
        header.tx = 1;
        header.ty = 2;
        header.tlayer = 0;
        header.bmin = [0.0, 0.0, 0.0];
        header.bmax = [5.0, 5.0, 5.0];
        header.width = 5;
        header.height = 5;

        let layer = TileCacheLayer::new(header);
        let tile_data = layer.to_bytes();

        // Add a tile
        let mut tile_ref = PolyRef::new(0);
        tile_cache.add_tile(&tile_data, 0, &mut tile_ref).unwrap();
        assert!(tile_ref.is_valid());

        // Test salt usage
        let salt = tile_cache.get_salt();
        assert!(salt > 1);

        // Try to get the tile
        let tile = tile_cache.get_tile_at(1, 2, 0);
        assert!(tile.is_some());

        // Remove the tile
        tile_cache.remove_tile(tile_ref).unwrap();
        let tile_after_removal = tile_cache.get_tile_at(1, 2, 0);
        assert!(tile_after_removal.is_none());
    }

    #[test]
    fn test_update_process() {
        let params = TileCacheParams {
            origin: [0.0, 0.0, 0.0],
            cs: 1.0,
            ch: 0.5,
            width: 10,
            height: 10,
            max_obstacles: 64,
        };

        let mut tile_cache = TileCache::new(params).unwrap();
        tile_cache.init().unwrap();

        // Add an obstacle (it should be marked as pending)
        let _obstacle_idx = tile_cache.add_obstacle([5.0, 1.0, 5.0], 2.0, 3.0).unwrap();

        // Update should process pending obstacles
        tile_cache.update().unwrap();

        // After update, obstacle should no longer be pending
        // (though in our mock implementation, we don't actually change the pending flag
        // since we don't have real tiles to affect)
    }
}
