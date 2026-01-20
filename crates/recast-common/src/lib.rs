//! Common utilities and data structures used by both Recast and Detour

pub mod debug;
mod geometry;
mod math;
mod mesh;
mod mesh_simplification;
mod vector;

pub use geometry::*;
pub use math::*;
pub use mesh::*;
pub use mesh_simplification::*;
pub use vector::*;

/// Represents a 3D position
pub type Vec3 = glam::Vec3;

/// Error types for the library
#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("invalid input mesh: {0}")]
    InvalidMesh(String),

    #[error("navigation mesh generation failed: {0}")]
    NavMeshGeneration(String),

    #[error("pathfinding failed: {0}")]
    Pathfinding(String),

    #[error("recast error: {0}")]
    Recast(String),

    #[cfg(feature = "std")]
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("detour error: {0}")]
    Detour(String),
}

/// Result type for  operations
pub type Result<T> = std::result::Result<T, Error>;
