//! Error types for the detour-tilecache crate

/// Error from tile cache operations
#[derive(thiserror::Error, Debug)]
pub enum TileCacheError {
    #[error("invalid parameter")]
    InvalidParam,

    #[error("out of memory: {resource}")]
    OutOfMemory { resource: &'static str },

    #[error("tile not found: ({x}, {y})")]
    TileNotFound { x: i32, y: i32 },

    #[error("obstacle not found")]
    ObstacleNotFound,

    #[error("invalid region data size")]
    InvalidRegionData,

    #[error("invalid area data size")]
    InvalidAreaData,

    #[error(transparent)]
    Build(#[from] recast::BuildError),

    #[error(transparent)]
    Detour(#[from] detour::DetourError),

    #[cfg(feature = "serialization")]
    #[error("serialization failed: {0}")]
    Serialization(#[source] Box<dyn std::error::Error + Send + Sync>),

    #[cfg(feature = "serialization")]
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

// Bridge impl
impl From<TileCacheError> for recast_common::Error {
    fn from(e: TileCacheError) -> Self {
        recast_common::Error::Detour(e.to_string())
    }
}
