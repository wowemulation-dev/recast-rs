//! Dynamic obstacle management and tile caching for navigation meshes
//!
//! This crate provides tile-based navigation mesh management with support for
//! runtime obstacle addition and removal. It enables dynamic environments where
//! obstacles can be added or removed without regenerating the entire navmesh.
//!
//! # Features
//!
//! - **Tile Caching**: Compressed storage of navigation mesh tiles
//! - **Dynamic Obstacles**: Add/remove cylinder, box, and oriented box obstacles
//! - **Incremental Updates**: Rebuild only affected tiles when obstacles change
//! - **Compression**: LZ4 compression for efficient tile storage
//! - **Streaming**: Support for large worlds through tile streaming
//!
//! # Example
//!
//! ```rust,ignore
//! use detour_tilecache::{TileCache, TileCacheParams, ObstacleRequest};
//! use glam::Vec3;
//!
//! // Create tile cache with configuration
//! let params = TileCacheParams {
//!     tile_width: 48,
//!     tile_height: 48,
//!     max_obstacles: 128,
//!     ..Default::default()
//! };
//! let mut tile_cache = TileCache::new(&params)?;
//!
//! // Add a cylinder obstacle
//! let obstacle_id = tile_cache.add_cylinder_obstacle(
//!     &Vec3::new(10.0, 0.0, 10.0),  // position
//!     2.0,                               // radius
//!     4.0,                               // height
//! )?;
//!
//! // Update affected tiles
//! tile_cache.update()?;
//!
//! // Remove the obstacle later
//! tile_cache.remove_obstacle(obstacle_id)?;
//! tile_cache.update()?;
//! ```
//!
//! # Architecture
//!
//! The tile cache system manages navigation data in tiles:
//!
//! - [`TileCache`]: Main cache managing tiles and obstacles
//! - [`TileCacheBuilder`]: Builds compressed tile data
//! - [`TileCacheLayer`]: Tile layer data for caching

pub mod tile_cache;
pub mod tile_cache_builder;
pub mod tile_cache_data;
pub mod tile_cache_integration;

pub use tile_cache::*;
pub use tile_cache_builder::*;
pub use tile_cache_data::*;
pub use tile_cache_integration::*;
