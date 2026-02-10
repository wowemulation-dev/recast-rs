//! Error types for the detour-dynamic crate

/// Error from dynamic navmesh operations
#[derive(thiserror::Error, Debug)]
pub enum DynamicError {
    #[error("invalid span data at cell ({x}, {y}): {detail}")]
    InvalidSpanData { x: i32, y: i32, detail: String },

    #[error("invalid partition type")]
    InvalidPartitionType,

    #[error("job queue full")]
    JobQueueFull,

    #[error("invalid config: {0}")]
    InvalidConfig(String),

    #[error(transparent)]
    Config(#[from] recast::ConfigError),

    #[error(transparent)]
    Build(#[from] recast::BuildError),

    #[error(transparent)]
    Detour(#[from] detour::DetourError),

    #[error(transparent)]
    Io(#[from] std::io::Error),

    /// Wraps legacy `recast_common::Error` from functions not yet migrated.
    #[error("recast: {0}")]
    Recast(#[from] recast_common::Error),
}

// Bridge impl
impl From<DynamicError> for recast_common::Error {
    fn from(e: DynamicError) -> Self {
        recast_common::Error::Detour(e.to_string())
    }
}
