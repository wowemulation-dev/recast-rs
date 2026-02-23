//! Error types for the waymark-dynamic crate

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
    Config(#[from] landmark::ConfigError),

    #[error(transparent)]
    Build(#[from] landmark::BuildError),

    #[error(transparent)]
    Detour(#[from] waymark::DetourError),

    #[error(transparent)]
    Io(#[from] std::io::Error),
}
