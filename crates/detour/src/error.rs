//! Error types for the detour crate
//!
//! These types replace the generic `recast_common::Error` variants with structured
//! errors that carry context about what went wrong during pathfinding and
//! navigation mesh operations.

use crate::status::Status;

/// Error from detour operations
#[derive(thiserror::Error, Debug)]
pub enum DetourError {
    #[error("invalid parameter")]
    InvalidParam,

    #[error("operation failed")]
    Failure,

    #[error("out of memory")]
    OutOfMemory,

    #[error("path not found")]
    PathNotFound,

    #[error("not found")]
    NotFound,

    #[error("buffer too small")]
    BufferTooSmall,

    #[error("query in progress")]
    InProgress,

    #[error("wrong magic number")]
    WrongMagic,

    #[error("wrong version")]
    WrongVersion,

    #[error("data corrupted")]
    DataCorrupted,

    #[error("navmesh build failed")]
    Build(#[from] recast::BuildError),

    #[cfg(feature = "serialization")]
    #[error("serialization failed: {0}")]
    Serialization(#[source] Box<dyn std::error::Error + Send + Sync>),

    #[cfg(feature = "serialization")]
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

impl From<Status> for DetourError {
    fn from(status: Status) -> Self {
        match status {
            Status::InvalidParam => DetourError::InvalidParam,
            Status::OutOfMemory => DetourError::OutOfMemory,
            Status::PathInvalid => DetourError::PathNotFound,
            Status::NotFound => DetourError::NotFound,
            Status::BufferTooSmall => DetourError::BufferTooSmall,
            Status::InProgress => DetourError::InProgress,
            Status::WrongMagic => DetourError::WrongMagic,
            Status::WrongVersion => DetourError::WrongVersion,
            Status::DataCorrupted => DetourError::DataCorrupted,
            _ => DetourError::Failure,
        }
    }
}

// Bridge impl: allow `?` to propagate DetourError into functions
// that still return `recast_common::Result<T>`.
impl From<DetourError> for recast_common::Error {
    fn from(e: DetourError) -> Self {
        recast_common::Error::Detour(e.to_string())
    }
}
