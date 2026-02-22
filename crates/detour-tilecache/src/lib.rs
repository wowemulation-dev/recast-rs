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
//! ```no_run
//! use detour_tilecache::{TileCache, TileCacheParams};
//!
//! # fn example() -> Result<(), Box<dyn std::error::Error>> {
//! let mut params = TileCacheParams::default();
//! params.max_obstacles = 128;
//! let mut tile_cache = TileCache::new(params)?;
//!
//! // Add a cylinder obstacle
//! let obstacle_id = tile_cache.add_obstacle(
//!     [10.0, 0.0, 10.0],  // position
//!     2.0,                 // radius
//!     4.0,                 // height
//! )?;
//!
//! tile_cache.update()?;
//!
//! // Remove the obstacle later
//! tile_cache.remove_obstacle(obstacle_id)?;
//! tile_cache.update()?;
//! # Ok(())
//! # }
//! ```
//!
//! # Architecture
//!
//! The tile cache system manages navigation data in tiles:
//!
//! - [`TileCache`]: Main cache managing tiles and obstacles
//! - [`TileCacheBuilder`]: Builds compressed tile data
//! - [`TileCacheLayer`]: Tile layer data for caching

pub mod error;
pub mod tile_cache;
pub mod tile_cache_builder;
pub mod tile_cache_data;
pub mod tile_cache_integration;

pub use error::TileCacheError;
pub use tile_cache::*;
pub use tile_cache_builder::*;
pub use tile_cache_data::*;
pub use tile_cache_integration::*;
