//! Common utilities and data structures used by both Recast and Detour

pub mod debug;
mod error;
mod geometry;
mod math;
mod mesh;
mod mesh_simplification;
mod vector;

pub use error::*;
pub use geometry::*;
pub use math::*;
pub use mesh::*;
pub use mesh_simplification::*;
pub use vector::*;

/// Represents a 3D position
pub type Vec3 = glam::Vec3;
