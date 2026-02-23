//! C++ compatible status codes for Detour operations
//!
//! This module provides a C++ compatible status system using bitmasks,
//! following the exact structure from DetourStatus.h

use std::fmt;

/// Status type for Detour operations (matches C++ dtStatus)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DtStatus(pub u32);

// High level status flags
impl DtStatus {
    /// Operation failed
    pub const DT_FAILURE: u32 = 1u32 << 31;
    /// Operation succeeded
    pub const DT_SUCCESS: u32 = 1u32 << 30;
    /// Operation still in progress
    pub const DT_IN_PROGRESS: u32 = 1u32 << 29;

    // Detail information for status
    pub const DT_STATUS_DETAIL_MASK: u32 = 0x0ffffff;
    /// Input data is not recognized
    pub const DT_WRONG_MAGIC: u32 = 1 << 0;
    /// Input data is in wrong version
    pub const DT_WRONG_VERSION: u32 = 1 << 1;
    /// Operation ran out of memory
    pub const DT_OUT_OF_MEMORY: u32 = 1 << 2;
    /// An input parameter was invalid
    pub const DT_INVALID_PARAM: u32 = 1 << 3;
    /// Result buffer for the query was too small to store all results
    pub const DT_BUFFER_TOO_SMALL: u32 = 1 << 4;
    /// Query ran out of nodes during search
    pub const DT_OUT_OF_NODES: u32 = 1 << 5;
    /// Query did not reach the end location, returning best guess
    pub const DT_PARTIAL_RESULT: u32 = 1 << 6;
    /// A tile has already been assigned to the given x,y coordinate
    pub const DT_ALREADY_OCCUPIED: u32 = 1 << 7;

    /// Creates a new status with the given flags
    pub const fn new(flags: u32) -> Self {
        Self(flags)
    }

    /// Creates a success status
    pub const fn success() -> Self {
        Self(Self::DT_SUCCESS)
    }

    /// Creates a failure status
    pub const fn failure() -> Self {
        Self(Self::DT_FAILURE)
    }

    /// Creates a failure status with detail
    pub const fn failure_detail(detail: u32) -> Self {
        Self(Self::DT_FAILURE | detail)
    }

    /// Creates a success status with detail
    pub const fn success_detail(detail: u32) -> Self {
        Self(Self::DT_SUCCESS | detail)
    }

    /// Creates an in-progress status
    pub const fn in_progress() -> Self {
        Self(Self::DT_IN_PROGRESS)
    }

    /// Returns true if status is success
    pub fn is_success(&self) -> bool {
        (self.0 & Self::DT_SUCCESS) != 0
    }

    /// Returns true if status is failure
    pub fn is_failure(&self) -> bool {
        (self.0 & Self::DT_FAILURE) != 0
    }

    /// Returns true if status is in progress
    pub fn is_in_progress(&self) -> bool {
        (self.0 & Self::DT_IN_PROGRESS) != 0
    }

    /// Returns true if specific detail is set
    pub fn has_detail(&self, detail: u32) -> bool {
        (self.0 & detail) != 0
    }

    /// Gets the detail mask
    pub fn detail(&self) -> u32 {
        self.0 & Self::DT_STATUS_DETAIL_MASK
    }
}

impl fmt::Display for DtStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_success() {
            write!(f, "Success")?;
        } else if self.is_failure() {
            write!(f, "Failure")?;
        } else if self.is_in_progress() {
            write!(f, "In Progress")?;
        }

        // Add detail information
        let mut details = Vec::new();
        if self.has_detail(Self::DT_WRONG_MAGIC) {
            details.push("Wrong Magic");
        }
        if self.has_detail(Self::DT_WRONG_VERSION) {
            details.push("Wrong Version");
        }
        if self.has_detail(Self::DT_OUT_OF_MEMORY) {
            details.push("Out of Memory");
        }
        if self.has_detail(Self::DT_INVALID_PARAM) {
            details.push("Invalid Param");
        }
        if self.has_detail(Self::DT_BUFFER_TOO_SMALL) {
            details.push("Buffer Too Small");
        }
        if self.has_detail(Self::DT_OUT_OF_NODES) {
            details.push("Out of Nodes");
        }
        if self.has_detail(Self::DT_PARTIAL_RESULT) {
            details.push("Partial Result");
        }
        if self.has_detail(Self::DT_ALREADY_OCCUPIED) {
            details.push("Already Occupied");
        }

        if !details.is_empty() {
            write!(f, " ({})", details.join(", "))?;
        }

        Ok(())
    }
}

impl Default for DtStatus {
    fn default() -> Self {
        Self::success()
    }
}

impl std::error::Error for DtStatus {}

/// Helper functions matching C++ API
pub fn dt_status_succeed(status: DtStatus) -> bool {
    status.is_success()
}

pub fn dt_status_failed(status: DtStatus) -> bool {
    status.is_failure()
}

pub fn dt_status_in_progress(status: DtStatus) -> bool {
    status.is_in_progress()
}

pub fn dt_status_detail(status: DtStatus, detail: u32) -> bool {
    status.has_detail(detail)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_status_flags() {
        let success = DtStatus::success();
        assert!(success.is_success());
        assert!(!success.is_failure());

        let failure = DtStatus::failure();
        assert!(!failure.is_success());
        assert!(failure.is_failure());

        let in_progress = DtStatus::in_progress();
        assert!(in_progress.is_in_progress());
        assert!(!in_progress.is_success());
        assert!(!in_progress.is_failure());
    }

    #[test]
    fn test_status_with_detail() {
        let failure_oom = DtStatus::failure_detail(DtStatus::DT_OUT_OF_MEMORY);
        assert!(failure_oom.is_failure());
        assert!(failure_oom.has_detail(DtStatus::DT_OUT_OF_MEMORY));
        assert!(!failure_oom.has_detail(DtStatus::DT_INVALID_PARAM));

        let success_partial = DtStatus::success_detail(DtStatus::DT_PARTIAL_RESULT);
        assert!(success_partial.is_success());
        assert!(success_partial.has_detail(DtStatus::DT_PARTIAL_RESULT));
    }

    #[test]
    fn test_multiple_details() {
        let status = DtStatus::new(
            DtStatus::DT_FAILURE | DtStatus::DT_INVALID_PARAM | DtStatus::DT_BUFFER_TOO_SMALL,
        );
        assert!(status.is_failure());
        assert!(status.has_detail(DtStatus::DT_INVALID_PARAM));
        assert!(status.has_detail(DtStatus::DT_BUFFER_TOO_SMALL));
        assert!(!status.has_detail(DtStatus::DT_OUT_OF_MEMORY));
    }
}
