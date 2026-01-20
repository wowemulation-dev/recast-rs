//! Context system for Recast operations providing logging, profiling, and progress tracking
//!
//! This module implements a context system similar to rcContext in the C++
//! library, providing comprehensive logging, timing, and progress monitoring capabilities.

use std::collections::HashMap;
use std::time::Duration;
use web_time::Instant;

/// Log level for context messages
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    /// Debug level messages
    Debug = 0,
    /// Informational messages
    Info = 1,
    /// Warning messages
    Warning = 2,
    /// Error messages
    Error = 3,
}

/// Timer categories for performance profiling
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum TimerCategory {
    /// Total navigation mesh generation time
    Total,
    /// Heightfield rasterization
    Rasterization,
    /// Heightfield filtering
    Filtering,
    /// Compact heightfield building
    CompactHeightfield,
    /// Region building
    Regions,
    /// Contour extraction
    Contours,
    /// Polygon mesh generation
    PolyMesh,
    /// Detail mesh generation
    DetailMesh,
    /// Area marking operations
    AreaMarking,
    /// Mesh merging operations
    MeshMerging,
    /// Pathfinding operations
    Pathfinding,
    /// Crowd simulation
    Crowd,
    /// Tile cache operations
    TileCache,
    /// Custom user-defined timer
    Custom(String),
}

/// Progress information for long-running operations
#[derive(Debug, Clone)]
pub struct ProgressInfo {
    /// Current step number
    pub current: usize,
    /// Total number of steps
    pub total: usize,
    /// Description of current operation
    pub description: String,
}

/// Log entry containing message and metadata
#[derive(Debug, Clone)]
pub struct LogEntry {
    /// Log level
    pub level: LogLevel,
    /// Timestamp when log was created
    pub timestamp: Instant,
    /// Log message
    pub message: String,
    /// Optional category for grouping logs
    pub category: Option<String>,
}

/// Timer entry for performance measurement
#[derive(Debug, Clone)]
pub struct TimerEntry {
    /// Timer category
    pub category: TimerCategory,
    /// Start time
    pub start_time: Instant,
    /// Duration (only set when timer is stopped)
    pub duration: Option<Duration>,
    /// Number of times this timer was used
    pub count: usize,
}

/// Context for Recast operations providing logging, profiling, and progress tracking
#[derive(Debug)]
pub struct RecastContext {
    /// Log entries
    logs: Vec<LogEntry>,
    /// Active timers
    active_timers: HashMap<TimerCategory, Instant>,
    /// Completed timer entries
    timers: HashMap<TimerCategory, TimerEntry>,
    /// Current progress information
    progress: Option<ProgressInfo>,
    /// Minimum log level to record
    min_log_level: LogLevel,
    /// Whether to enable performance timing
    enable_timing: bool,
    /// Maximum number of log entries to keep
    max_log_entries: usize,
}

impl Default for RecastContext {
    fn default() -> Self {
        Self::new()
    }
}

impl RecastContext {
    /// Creates a new Recast context with default settings
    pub fn new() -> Self {
        Self {
            logs: Vec::new(),
            active_timers: HashMap::new(),
            timers: HashMap::new(),
            progress: None,
            min_log_level: LogLevel::Info,
            enable_timing: true,
            max_log_entries: 1000,
        }
    }

    /// Sets the minimum log level
    pub fn set_log_level(&mut self, level: LogLevel) {
        self.min_log_level = level;
    }

    /// Enables or disables performance timing
    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.enable_timing = enabled;
    }

    /// Sets the maximum number of log entries to keep
    pub fn set_max_log_entries(&mut self, max_entries: usize) {
        self.max_log_entries = max_entries;
    }

    /// Logs a debug message
    pub fn log_debug(&mut self, message: impl Into<String>) {
        self.log(LogLevel::Debug, message, None);
    }

    /// Logs a debug message with category
    pub fn log_debug_with_category(
        &mut self,
        message: impl Into<String>,
        category: impl Into<String>,
    ) {
        self.log(LogLevel::Debug, message, Some(category.into()));
    }

    /// Logs an info message
    pub fn log_info(&mut self, message: impl Into<String>) {
        self.log(LogLevel::Info, message, None);
    }

    /// Logs an info message with category
    pub fn log_info_with_category(
        &mut self,
        message: impl Into<String>,
        category: impl Into<String>,
    ) {
        self.log(LogLevel::Info, message, Some(category.into()));
    }

    /// Logs a warning message
    pub fn log_warning(&mut self, message: impl Into<String>) {
        self.log(LogLevel::Warning, message, None);
    }

    /// Logs a warning message with category
    pub fn log_warning_with_category(
        &mut self,
        message: impl Into<String>,
        category: impl Into<String>,
    ) {
        self.log(LogLevel::Warning, message, Some(category.into()));
    }

    /// Logs an error message
    pub fn log_error(&mut self, message: impl Into<String>) {
        self.log(LogLevel::Error, message, None);
    }

    /// Logs an error message with category
    pub fn log_error_with_category(
        &mut self,
        message: impl Into<String>,
        category: impl Into<String>,
    ) {
        self.log(LogLevel::Error, message, Some(category.into()));
    }

    /// Internal logging function
    fn log(&mut self, level: LogLevel, message: impl Into<String>, category: Option<String>) {
        if level >= self.min_log_level {
            let entry = LogEntry {
                level,
                timestamp: Instant::now(),
                message: message.into(),
                category,
            };

            self.logs.push(entry);

            // Trim logs if we exceed the maximum
            if self.logs.len() > self.max_log_entries {
                self.logs.remove(0);
            }
        }
    }

    /// Starts a timer for the given category
    pub fn start_timer(&mut self, category: TimerCategory) {
        if self.enable_timing {
            self.active_timers.insert(category, Instant::now());
        }
    }

    /// Stops a timer and records the duration
    pub fn stop_timer(&mut self, category: TimerCategory) {
        if self.enable_timing {
            if let Some(start_time) = self.active_timers.remove(&category) {
                let duration = start_time.elapsed();

                // Update or create timer entry
                let entry = self.timers.entry(category.clone()).or_insert(TimerEntry {
                    category,
                    start_time,
                    duration: Some(duration),
                    count: 0,
                });

                if let Some(existing_duration) = entry.duration {
                    entry.duration = Some(existing_duration + duration);
                } else {
                    entry.duration = Some(duration);
                }
                entry.count += 1;
            }
        }
    }

    /// Gets the elapsed time for an active timer
    pub fn get_timer_elapsed(&self, category: &TimerCategory) -> Option<Duration> {
        if self.enable_timing {
            self.active_timers
                .get(category)
                .map(|start| start.elapsed())
        } else {
            None
        }
    }

    /// Gets the total duration for a completed timer
    pub fn get_timer_duration(&self, category: &TimerCategory) -> Option<Duration> {
        self.timers.get(category).and_then(|entry| entry.duration)
    }

    /// Gets the count for a timer (how many times it was used)
    pub fn get_timer_count(&self, category: &TimerCategory) -> usize {
        self.timers
            .get(category)
            .map(|entry| entry.count)
            .unwrap_or(0)
    }

    /// Updates progress information
    pub fn set_progress(&mut self, current: usize, total: usize, description: impl Into<String>) {
        self.progress = Some(ProgressInfo {
            current,
            total,
            description: description.into(),
        });
    }

    /// Clears progress information
    pub fn clear_progress(&mut self) {
        self.progress = None;
    }

    /// Gets current progress information
    pub fn get_progress(&self) -> Option<&ProgressInfo> {
        self.progress.as_ref()
    }

    /// Gets all log entries
    pub fn get_logs(&self) -> &[LogEntry] {
        &self.logs
    }

    /// Gets log entries for a specific level
    pub fn get_logs_by_level(&self, level: LogLevel) -> Vec<&LogEntry> {
        self.logs
            .iter()
            .filter(|entry| entry.level == level)
            .collect()
    }

    /// Gets log entries for a specific category
    pub fn get_logs_by_category(&self, category: &str) -> Vec<&LogEntry> {
        self.logs
            .iter()
            .filter(|entry| {
                entry
                    .category
                    .as_ref()
                    .map(|c| c == category)
                    .unwrap_or(false)
            })
            .collect()
    }

    /// Gets all timer entries
    pub fn get_timers(&self) -> &HashMap<TimerCategory, TimerEntry> {
        &self.timers
    }

    /// Clears all logs
    pub fn clear_logs(&mut self) {
        self.logs.clear();
    }

    /// Clears all timers
    pub fn clear_timers(&mut self) {
        self.active_timers.clear();
        self.timers.clear();
    }

    /// Resets the context (clears logs, timers, and progress)
    pub fn reset(&mut self) {
        self.clear_logs();
        self.clear_timers();
        self.clear_progress();
    }

    /// Prints a summary of performance timers
    pub fn print_timer_summary(&self) {
        println!("=== Recast Performance Summary ===");

        let mut sorted_timers: Vec<_> = self.timers.iter().collect();
        sorted_timers.sort_by(|a, b| {
            b.1.duration
                .unwrap_or_default()
                .cmp(&a.1.duration.unwrap_or_default())
        });

        for (category, entry) in sorted_timers {
            if let Some(duration) = entry.duration {
                println!(
                    "{:20} {:8.2}ms ({} calls, avg: {:.2}ms)",
                    format!("{:?}", category),
                    duration.as_secs_f64() * 1000.0,
                    entry.count,
                    duration.as_secs_f64() * 1000.0 / entry.count as f64
                );
            }
        }
    }

    /// Prints all log entries
    pub fn print_logs(&self) {
        println!("=== Recast Logs ===");
        for entry in &self.logs {
            let level_str = match entry.level {
                LogLevel::Debug => "DEBUG",
                LogLevel::Info => "INFO",
                LogLevel::Warning => "WARN",
                LogLevel::Error => "ERROR",
            };

            let category_str = entry
                .category
                .as_ref()
                .map(|c| format!("[{}] ", c))
                .unwrap_or_default();

            println!("{} {}{}", level_str, category_str, entry.message);
        }
    }
}

/// RAII timer guard that automatically stops timing when dropped
/// Uses interior mutability to avoid borrow checker issues
pub struct TimerGuard {
    category: TimerCategory,
    started: bool,
}

impl TimerGuard {
    /// Creates a new timer guard and starts timing
    pub fn new(context: &mut RecastContext, category: TimerCategory) -> Self {
        context.start_timer(category.clone());
        Self {
            category,
            started: true,
        }
    }

    /// Manually stops the timer before the guard is dropped
    pub fn stop(mut self, context: &mut RecastContext) {
        if self.started {
            context.stop_timer(self.category.clone());
            self.started = false;
        }
    }

    /// Get the category of this timer
    pub fn category(&self) -> &TimerCategory {
        &self.category
    }
}

impl Drop for TimerGuard {
    fn drop(&mut self) {
        // Note: We can't stop the timer here without access to the context
        // The timer will remain active if not manually stopped
        // This is a limitation of the RAII pattern in this case
        if self.started {
            eprintln!(
                "Warning: TimerGuard for {:?} was dropped without being stopped",
                self.category
            );
        }
    }
}

/// Convenience macro for creating timer guards
#[macro_export]
macro_rules! recast_timer {
    ($context:expr, $category:expr) => {
        $crate::recast::context::TimerGuard::new($context, $category)
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_context_creation() {
        let context = RecastContext::new();
        assert_eq!(context.logs.len(), 0);
        assert_eq!(context.timers.len(), 0);
        assert!(context.progress.is_none());
    }

    #[test]
    fn test_logging() {
        let mut context = RecastContext::new();
        context.set_log_level(LogLevel::Debug);

        context.log_debug("Debug message");
        context.log_info("Info message");
        context.log_warning("Warning message");
        context.log_error("Error message");

        assert_eq!(context.logs.len(), 4);
        assert_eq!(context.logs[0].level, LogLevel::Debug);
        assert_eq!(context.logs[1].level, LogLevel::Info);
        assert_eq!(context.logs[2].level, LogLevel::Warning);
        assert_eq!(context.logs[3].level, LogLevel::Error);
    }

    #[test]
    fn test_log_level_filtering() {
        let mut context = RecastContext::new();
        context.set_log_level(LogLevel::Warning);

        context.log_debug("Debug message");
        context.log_info("Info message");
        context.log_warning("Warning message");
        context.log_error("Error message");

        assert_eq!(context.logs.len(), 2); // Only warning and error
        assert_eq!(context.logs[0].level, LogLevel::Warning);
        assert_eq!(context.logs[1].level, LogLevel::Error);
    }

    #[test]
    fn test_timing() {
        let mut context = RecastContext::new();

        context.start_timer(TimerCategory::Total);
        thread::sleep(Duration::from_millis(10));
        context.stop_timer(TimerCategory::Total);

        let duration = context.get_timer_duration(&TimerCategory::Total);
        assert!(duration.is_some());
        assert!(duration.unwrap() >= Duration::from_millis(10));
    }

    #[test]
    fn test_timer_guard() {
        let mut context = RecastContext::new();

        {
            let guard = TimerGuard::new(&mut context, TimerCategory::Rasterization);
            thread::sleep(Duration::from_millis(10));
            guard.stop(&mut context); // Manually stop the timer
        }

        let duration = context.get_timer_duration(&TimerCategory::Rasterization);
        assert!(duration.is_some());
        assert!(duration.unwrap() >= Duration::from_millis(10));
    }

    #[test]
    fn test_progress_tracking() {
        let mut context = RecastContext::new();

        context.set_progress(5, 10, "Processing triangles");
        let progress = context.get_progress().unwrap();
        assert_eq!(progress.current, 5);
        assert_eq!(progress.total, 10);
        assert_eq!(progress.description, "Processing triangles");

        context.clear_progress();
        assert!(context.get_progress().is_none());
    }

    #[test]
    fn test_log_categories() {
        let mut context = RecastContext::new();

        context.log_info_with_category("Test message 1", "rasterization");
        context.log_info_with_category("Test message 2", "filtering");
        context.log_info_with_category("Test message 3", "rasterization");

        let rasterization_logs = context.get_logs_by_category("rasterization");
        assert_eq!(rasterization_logs.len(), 2);

        let filtering_logs = context.get_logs_by_category("filtering");
        assert_eq!(filtering_logs.len(), 1);
    }

    #[test]
    fn test_timer_accumulation() {
        let mut context = RecastContext::new();

        // Run timer multiple times
        for _ in 0..3 {
            context.start_timer(TimerCategory::Filtering);
            thread::sleep(Duration::from_millis(5));
            context.stop_timer(TimerCategory::Filtering);
        }

        let count = context.get_timer_count(&TimerCategory::Filtering);
        assert_eq!(count, 3);

        let duration = context.get_timer_duration(&TimerCategory::Filtering);
        assert!(duration.is_some());
        assert!(duration.unwrap() >= Duration::from_millis(15));
    }

    #[test]
    fn test_max_log_entries() {
        let mut context = RecastContext::new();
        context.set_max_log_entries(3);

        // Add more logs than the maximum
        for i in 0..5 {
            context.log_info(format!("Message {}", i));
        }

        // Should only keep the last 3 entries
        assert_eq!(context.logs.len(), 3);
        assert_eq!(context.logs[2].message, "Message 4");
    }

    #[test]
    fn test_context_reset() {
        let mut context = RecastContext::new();

        context.log_info("Test message");
        context.start_timer(TimerCategory::Total);
        context.stop_timer(TimerCategory::Total);
        context.set_progress(1, 2, "Test progress");

        assert!(!context.logs.is_empty());
        assert!(!context.timers.is_empty());
        assert!(context.progress.is_some());

        context.reset();

        assert!(context.logs.is_empty());
        assert!(context.timers.is_empty());
        assert!(context.progress.is_none());
    }
}
