//! Navigation-specific visualization traits and data structures
//!
//! This module provides traits and data structures for visualization that can be
//! implemented by higher-level crates (detour, recast) without creating circular dependencies.

use super::{Color, DebugConfig, DebugDraw};

/// Trait for types that can be visualized in debug mode
pub trait DebugVisualize {
    /// Draws debug visualization for this type
    fn debug_draw(&self, debug_draw: &mut DebugDraw, config: &DebugConfig);
}

/// Visualization helper for polygon flags
#[derive(Debug, Clone, Copy)]
pub struct PolyFlagColors;

impl PolyFlagColors {
    /// Default walkable area color
    pub const WALK: Color = Color::GREEN;

    /// Water/swim area color
    pub const SWIM: Color = Color::BLUE;

    /// Door area color
    pub const DOOR: Color = Color::ORANGE;

    /// Jump area color
    pub const JUMP: Color = Color::YELLOW;

    /// Disabled area color
    pub const DISABLED: Color = Color::RED;

    /// Climbable area color
    pub const CLIMB: Color = Color::PURPLE;

    /// Default/unknown area color
    pub const DEFAULT: Color = Color::GRAY;
}

/// Agent state visualization colors
#[derive(Debug, Clone, Copy)]
pub struct AgentStateColors;

impl AgentStateColors {
    /// Active agent color
    pub const ACTIVE: Color = Color::GREEN;

    /// Waiting agent color
    pub const WAITING: Color = Color::YELLOW;

    /// Completed agent color
    pub const COMPLETED: Color = Color::BLUE;

    /// Failed agent color
    pub const FAILED: Color = Color::RED;

    /// Invalid agent color
    pub const INVALID: Color = Color::GRAY;
}

/// Formation state visualization colors
#[derive(Debug, Clone, Copy)]
pub struct FormationStateColors;

impl FormationStateColors {
    /// Forming formation color
    pub const FORMING: Color = Color::YELLOW;

    /// Stable formation color
    pub const STABLE: Color = Color::GREEN;

    /// Moving formation color
    pub const MOVING: Color = Color::BLUE;

    /// Adapting formation color
    pub const ADAPTING: Color = Color::ORANGE;

    /// Disbanding formation color
    pub const DISBANDING: Color = Color::RED;
}

/// Formation role visualization colors
#[derive(Debug, Clone, Copy)]
pub struct FormationRoleColors;

impl FormationRoleColors {
    /// Leader role color
    pub const LEADER: Color = Color::RED;

    /// Follower role color
    pub const FOLLOWER: Color = Color::GREEN;

    /// Scout role color
    pub const SCOUT: Color = Color::YELLOW;

    /// Guard role color
    pub const GUARD: Color = Color::BLUE;
}

/// Utility to create debug colors for different purposes
pub struct DebugColors;

impl DebugColors {
    /// Generate a unique color based on an ID
    pub fn unique_color(id: usize) -> Color {
        let r = ((id * 137) % 255) as f32 / 255.0;
        let g = ((id * 211) % 255) as f32 / 255.0;
        let b = ((id * 97) % 255) as f32 / 255.0;
        Color::rgb(r, g, b)
    }

    /// Generate a gradient color between two colors
    pub fn gradient(start: Color, end: Color, t: f32) -> Color {
        let t = t.clamp(0.0, 1.0);
        Color::new(
            start.r + (end.r - start.r) * t,
            start.g + (end.g - start.g) * t,
            start.b + (end.b - start.b) * t,
            start.a + (end.a - start.a) * t,
        )
    }

    /// Generate a heat map color (blue -> green -> yellow -> red)
    pub fn heat_map(value: f32) -> Color {
        let value = value.clamp(0.0, 1.0);

        if value < 0.25 {
            // Blue to green
            let t = value * 4.0;
            Self::gradient(Color::BLUE, Color::GREEN, t)
        } else if value < 0.5 {
            // Green to yellow
            let t = (value - 0.25) * 4.0;
            Self::gradient(Color::GREEN, Color::YELLOW, t)
        } else if value < 0.75 {
            // Yellow to orange
            let t = (value - 0.5) * 4.0;
            Self::gradient(Color::YELLOW, Color::ORANGE, t)
        } else {
            // Orange to red
            let t = (value - 0.75) * 4.0;
            Self::gradient(Color::ORANGE, Color::RED, t)
        }
    }
}

/// Visualization info for navigation mesh debugging
#[derive(Debug, Clone)]
pub struct NavMeshDebugInfo {
    /// Total number of tiles
    pub tile_count: usize,
    /// Total number of polygons
    pub poly_count: usize,
    /// Total number of vertices
    pub vert_count: usize,
    /// Bounds of the navigation mesh
    pub bounds_min: [f32; 3],
    pub bounds_max: [f32; 3],
}

/// Visualization info for crowd debugging
#[derive(Debug, Clone)]
pub struct CrowdDebugInfo {
    /// Number of active agents
    pub active_agents: usize,
    /// Total agent capacity
    pub max_agents: usize,
    /// Number of formations
    pub formation_count: usize,
    /// Average velocity of all agents
    pub avg_velocity: f32,
}

/// Visualization info for pathfinding debugging
#[derive(Debug, Clone)]
pub struct PathfindingDebugInfo {
    /// Number of nodes explored
    pub nodes_explored: usize,
    /// Length of the found path
    pub path_length: f32,
    /// Time taken to find path (in milliseconds)
    pub search_time_ms: f32,
    /// Whether a complete path was found
    pub path_found: bool,
}
