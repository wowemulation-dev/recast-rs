//! Error types for the detour-crowd crate

/// Error from crowd simulation operations
#[derive(thiserror::Error, Debug)]
pub enum CrowdError {
    #[error("invalid parameter")]
    InvalidParam,

    #[error("agent not found: {index}")]
    AgentNotFound { index: usize },

    #[error("path corridor failed")]
    CorridorFailed,

    #[error("RVO computation failed: {0}")]
    Rvo(&'static str),

    #[error(transparent)]
    Detour(#[from] waymark::DetourError),
}
