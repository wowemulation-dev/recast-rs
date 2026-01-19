//! Dynamic Navigation Mesh Generation
//!
//! This crate provides dynamic navigation mesh generation capabilities, allowing for
//! real-time modification of navigation meshes by adding and removing obstacles.

// Allow unused code in tests - test code often has intentionally unused variables
// for demonstration or future use
#![cfg_attr(test, allow(unused))]
//! This is particularly useful for games with destructible environments, moving
//! platforms, or procedurally generated content.
//!
//! # Features
//!
//! - **Dynamic Obstacle Management**: Add and remove colliders at runtime
//! - **Incremental Updates**: Only rebuild affected tiles for performance
//! - **Async Processing**: Non-blocking navmesh updates using Tokio
//! - **Multiple Collider Types**: Box, cylinder, sphere, trimesh, and composite colliders
//! - **Checkpoint System**: Efficient state management for incremental rebuilds
//! - **Voxel-based Queries**: Precise raycasting against heightfield data
//! - **Serialization**: Save and load dynamic navmesh state
//!
//! # Example
//!
//! ```rust,no_run
//! use detour_dynamic::{DynamicNavMesh, DynamicNavMeshConfig, VoxelQuery};
//! use detour_dynamic::colliders::BoxCollider;
//! use glam::Vec3;
//! use std::sync::Arc;
//!
//! # fn example() -> Result<(), Box<dyn std::error::Error>> {
//! // Create configuration
//! let config = DynamicNavMeshConfig {
//!     world_min: Vec3::new(-20.0, -1.0, -20.0),
//!     world_max: Vec3::new(20.0, 5.0, 20.0),
//!     cell_size: 0.3,
//!     cell_height: 0.2,
//!     walkable_height: 2.0,
//!     walkable_radius: 0.6,
//!     walkable_climb: 0.9,
//!     walkable_slope_angle: 45.0,
//!     ..Default::default()
//! };
//!
//! // Create dynamic navmesh with tile grid
//! let mut dynamic_navmesh = DynamicNavMesh::with_tile_grid(config, 3, 3)?;
//!
//! // Add a dynamic obstacle (area=0 for walkable, flag_merge_threshold=1.0)
//! let box_collider = Arc::new(BoxCollider::new(
//!     Vec3::new(5.0, 0.0, 5.0),  // center
//!     Vec3::new(1.0, 2.0, 1.0),  // half extents
//!     0,                          // area type
//!     1.0,                        // flag merge threshold
//! ));
//! dynamic_navmesh.add_collider(box_collider)?;
//!
//! // Update navmesh to process changes
//! dynamic_navmesh.update()?;
//!
//! // Perform voxel raycasting using a simple heightfield query
//! let voxel_query = VoxelQuery::from_single_heightfield(
//!     Vec3::new(-10.0, 0.0, -10.0),  // origin
//!     10.0,                           // tile width
//!     10.0,                           // tile depth
//! );
//!
//! if let Some(hit) = voxel_query.raycast(
//!     Vec3::new(0.0, 1.0, 0.0),  // start
//!     Vec3::new(5.0, 1.0, 5.0),  // end
//! ) {
//!     println!("Hit obstacle at {:?}", hit.position);
//! }
//!
//! // Get statistics about the dynamic navmesh
//! let stats = dynamic_navmesh.get_statistics();
//! println!("Memory usage: {} bytes", stats.memory_usage);
//! # Ok(())
//! # }
//! ```

pub mod checkpoint;
pub mod colliders;
pub mod config;
pub mod dynamic_navmesh;
pub mod dynamic_tile;
pub mod io;
pub mod jobs;
pub mod voxel_query;

// Re-export main types
pub use checkpoint::{CheckpointManager, DynamicTileCheckpoint};
pub use config::DynamicNavMeshConfig;
pub use dynamic_navmesh::{DynamicNavMesh, DynamicNavMeshStatistics};
pub use dynamic_tile::{DynamicTile, DynamicTileManager, TileCheckpoint, TileStatus};

// Re-export collider types
pub use colliders::{
    BoxCollider, CapsuleCollider, Collider, CompositeCollider, CylinderCollider, SphereCollider,
    TrimeshCollider,
};

// Re-export I/O types
pub use io::{VoxelFile, VoxelTile};

// Re-export voxel query
pub use voxel_query::{VoxelQuery, VoxelRaycastHit};

// Re-export job system
pub use jobs::{ColliderAdditionJob, ColliderRemovalJob, DynamicTileJob, JobProcessor};

/// Version of the dynamic navmesh format
pub const DYNAMIC_NAVMESH_VERSION: u32 = 1;

/// Magic number for dynamic navmesh files
pub const DYNAMIC_NAVMESH_MAGIC: u32 = 0x444E4D48; // "DNMH"
