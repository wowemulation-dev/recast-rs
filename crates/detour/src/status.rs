//! Status codes for Detour operations

/// Result type for Detour operations
pub type Result<T> = std::result::Result<T, Status>;

/// Status enum for Detour operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation was successful
    Success,
    /// Operation failed due to an unknown reason
    Failure,
    /// Provided parameter was invalid
    InvalidParam,
    /// Operation ran out of memory
    OutOfMemory,
    /// Operation timed out
    Timeout,
    /// Operation was interrupted
    Interrupted,
    /// Pathfinding failed; no valid path found
    PathInvalid,
    /// Navigation mesh data is invalid
    NavMeshInvalid,
    /// Pathfinding operation exceeded buffer capacity
    BufferTooSmall,
    /// Query already in progress
    InProgress,
    /// Value already exists
    AlreadyExists,
    /// Value does not exist or is not found
    NotFound,
    /// Operation partially succeeded
    PartialResult,
    /// Navigation mesh tile is out of bounds
    TileOutOfBounds,
    /// Agent is in an invalid state
    AgentInvalid,
    /// Specified agent does not exist
    AgentNotFound,
    /// Crowd manager encountered an error
    CrowdError,
    /// Path corridor operation failed
    CorridorError,
    /// Local neighborhood operation failed
    NeighborhoodError,
    /// Input data is corrupted or invalid
    DataCorrupted,
    /// Tile cache operation failed
    TileCacheError,
    /// File has wrong magic number
    WrongMagic,
    /// File has wrong version
    WrongVersion,
}

impl Status {
    /// Checks if the status is a failure
    pub fn is_failure(&self) -> bool {
        *self != Status::Success && *self != Status::PartialResult
    }

    /// Checks if the status is a success
    pub fn is_success(&self) -> bool {
        *self == Status::Success || *self == Status::PartialResult
    }

    /// Converts status to a result
    pub fn to_result<T>(self, value: T) -> Result<T> {
        if self.is_success() {
            Ok(value)
        } else {
            Err(self)
        }
    }
}

impl std::error::Error for Status {}

impl std::fmt::Display for Status {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Status::Success => write!(f, "Success"),
            Status::Failure => write!(f, "Failure"),
            Status::InvalidParam => write!(f, "Invalid parameter"),
            Status::OutOfMemory => write!(f, "Out of memory"),
            Status::Timeout => write!(f, "Operation timed out"),
            Status::Interrupted => write!(f, "Operation interrupted"),
            Status::PathInvalid => write!(f, "Invalid path"),
            Status::NavMeshInvalid => write!(f, "Invalid navigation mesh"),
            Status::BufferTooSmall => write!(f, "Buffer too small"),
            Status::InProgress => write!(f, "Operation in progress"),
            Status::AlreadyExists => write!(f, "Value already exists"),
            Status::NotFound => write!(f, "Value not found"),
            Status::PartialResult => write!(f, "Partial result"),
            Status::TileOutOfBounds => write!(f, "Tile out of bounds"),
            Status::AgentInvalid => write!(f, "Invalid agent"),
            Status::AgentNotFound => write!(f, "Agent not found"),
            Status::CrowdError => write!(f, "Crowd error"),
            Status::CorridorError => write!(f, "Path corridor error"),
            Status::NeighborhoodError => write!(f, "Neighborhood error"),
            Status::DataCorrupted => write!(f, "Data corrupted"),
            Status::TileCacheError => write!(f, "Tile cache error"),
            Status::WrongMagic => write!(f, "Wrong magic number"),
            Status::WrongVersion => write!(f, "Wrong version"),
        }
    }
}

impl From<std::io::Error> for Status {
    fn from(_: std::io::Error) -> Self {
        Status::Failure
    }
}

/// Macros for handling Detour status results
/// Return early with an error if the expression evaluates to `Err`
#[macro_export]
macro_rules! dtry {
    ($expr:expr) => {
        match $expr {
            Ok(val) => val,
            Err(err) => return Err(err),
        }
    };
}

/// Return early with `Status::Failure` if the expression evaluates to `false`
#[macro_export]
macro_rules! dtassert {
    ($expr:expr) => {
        if !$expr {
            return Err($crate::detour::Status::Failure);
        }
    };
    ($expr:expr, $status:expr) => {
        if !$expr {
            return Err($status);
        }
    };
}

/// Return early with `Status::InvalidParam` if the expression evaluates to `None`
#[macro_export]
macro_rules! dtunwrap {
    ($expr:expr) => {
        match $expr {
            Some(val) => val,
            None => return Err($crate::detour::Status::InvalidParam),
        }
    };
    ($expr:expr, $status:expr) => {
        match $expr {
            Some(val) => val,
            None => return Err($status),
        }
    };
}
