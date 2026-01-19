//! Debug visualization utilities for recast-navigation
//!
//! This module provides utilities for visualizing navigation meshes, paths,
//! crowd behaviors, and other debugging information in a renderer-agnostic way.

mod visualization;

pub use visualization::*;

/// Color representation for debug visualization
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    /// Creates a new color
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Creates a color from RGB values (alpha = 1.0)
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self::new(r, g, b, 1.0)
    }

    /// Creates a color from RGBA bytes
    pub const fn from_rgba_bytes(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self::new(
            r as f32 / 255.0,
            g as f32 / 255.0,
            b as f32 / 255.0,
            a as f32 / 255.0,
        )
    }

    /// Creates a color from RGB bytes (alpha = 1.0)
    pub const fn from_rgb_bytes(r: u8, g: u8, b: u8) -> Self {
        Self::from_rgba_bytes(r, g, b, 255)
    }
}

/// Common debug colors
impl Color {
    pub const WHITE: Color = Color::rgb(1.0, 1.0, 1.0);
    pub const BLACK: Color = Color::rgb(0.0, 0.0, 0.0);
    pub const RED: Color = Color::rgb(1.0, 0.0, 0.0);
    pub const GREEN: Color = Color::rgb(0.0, 1.0, 0.0);
    pub const BLUE: Color = Color::rgb(0.0, 0.0, 1.0);
    pub const YELLOW: Color = Color::rgb(1.0, 1.0, 0.0);
    pub const CYAN: Color = Color::rgb(0.0, 1.0, 1.0);
    pub const MAGENTA: Color = Color::rgb(1.0, 0.0, 1.0);
    pub const ORANGE: Color = Color::rgb(1.0, 0.5, 0.0);
    pub const PURPLE: Color = Color::rgb(0.5, 0.0, 1.0);
    pub const GRAY: Color = Color::rgb(0.5, 0.5, 0.5);
    pub const LIGHT_GRAY: Color = Color::rgb(0.8, 0.8, 0.8);
    pub const DARK_GRAY: Color = Color::rgb(0.3, 0.3, 0.3);
}

/// Debug vertex with position and color
#[derive(Debug, Clone, Copy)]
pub struct DebugVertex {
    pub position: [f32; 3],
    pub color: Color,
}

impl DebugVertex {
    /// Creates a new debug vertex
    pub const fn new(position: [f32; 3], color: Color) -> Self {
        Self { position, color }
    }
}

/// Debug line for rendering
#[derive(Debug, Clone)]
pub struct DebugLine {
    pub start: [f32; 3],
    pub end: [f32; 3],
    pub color: Color,
    pub thickness: f32,
}

impl DebugLine {
    /// Creates a new debug line
    pub fn new(start: [f32; 3], end: [f32; 3], color: Color) -> Self {
        Self {
            start,
            end,
            color,
            thickness: 1.0,
        }
    }

    /// Creates a debug line with custom thickness
    pub fn with_thickness(start: [f32; 3], end: [f32; 3], color: Color, thickness: f32) -> Self {
        Self {
            start,
            end,
            color,
            thickness,
        }
    }
}

/// Debug triangle for rendering
#[derive(Debug, Clone)]
pub struct DebugTriangle {
    pub vertices: [DebugVertex; 3],
    pub wireframe: bool,
}

impl DebugTriangle {
    /// Creates a new debug triangle
    pub fn new(v0: DebugVertex, v1: DebugVertex, v2: DebugVertex) -> Self {
        Self {
            vertices: [v0, v1, v2],
            wireframe: false,
        }
    }

    /// Creates a wireframe debug triangle
    pub fn wireframe(v0: DebugVertex, v1: DebugVertex, v2: DebugVertex) -> Self {
        Self {
            vertices: [v0, v1, v2],
            wireframe: true,
        }
    }

    /// Creates a triangle with single color
    pub fn colored(p0: [f32; 3], p1: [f32; 3], p2: [f32; 3], color: Color) -> Self {
        Self::new(
            DebugVertex::new(p0, color),
            DebugVertex::new(p1, color),
            DebugVertex::new(p2, color),
        )
    }
}

/// Debug text for rendering
#[derive(Debug, Clone)]
pub struct DebugText {
    pub position: [f32; 3],
    pub text: String,
    pub color: Color,
    pub size: f32,
}

impl DebugText {
    /// Creates a new debug text
    pub fn new(position: [f32; 3], text: String, color: Color) -> Self {
        Self {
            position,
            text,
            color,
            size: 12.0,
        }
    }

    /// Creates debug text with custom size
    pub fn with_size(position: [f32; 3], text: String, color: Color, size: f32) -> Self {
        Self {
            position,
            text,
            color,
            size,
        }
    }
}

/// Debug circle for rendering
#[derive(Debug, Clone)]
pub struct DebugCircle {
    pub center: [f32; 3],
    pub radius: f32,
    pub color: Color,
    pub segments: u32,
}

impl DebugCircle {
    /// Creates a new debug circle
    pub fn new(center: [f32; 3], radius: f32, color: Color) -> Self {
        Self {
            center,
            radius,
            color,
            segments: 32,
        }
    }

    /// Creates a debug circle with custom segment count
    pub fn with_segments(center: [f32; 3], radius: f32, color: Color, segments: u32) -> Self {
        Self {
            center,
            radius,
            color,
            segments,
        }
    }
}

/// Debug sphere for rendering
#[derive(Debug, Clone)]
pub struct DebugSphere {
    pub center: [f32; 3],
    pub radius: f32,
    pub color: Color,
    pub wireframe: bool,
}

impl DebugSphere {
    /// Creates a new debug sphere
    pub fn new(center: [f32; 3], radius: f32, color: Color) -> Self {
        Self {
            center,
            radius,
            color,
            wireframe: false,
        }
    }

    /// Creates a wireframe debug sphere
    pub fn wireframe(center: [f32; 3], radius: f32, color: Color) -> Self {
        Self {
            center,
            radius,
            color,
            wireframe: true,
        }
    }
}

/// Debug bounding box for rendering
#[derive(Debug, Clone)]
pub struct DebugBounds {
    pub min: [f32; 3],
    pub max: [f32; 3],
    pub color: Color,
    pub wireframe: bool,
}

impl DebugBounds {
    /// Creates a new debug bounding box
    pub fn new(min: [f32; 3], max: [f32; 3], color: Color) -> Self {
        Self {
            min,
            max,
            color,
            wireframe: true,
        }
    }

    /// Creates a filled debug bounding box
    pub fn filled(min: [f32; 3], max: [f32; 3], color: Color) -> Self {
        Self {
            min,
            max,
            color,
            wireframe: false,
        }
    }
}

/// Debug arrow for rendering
#[derive(Debug, Clone)]
pub struct DebugArrow {
    pub start: [f32; 3],
    pub end: [f32; 3],
    pub color: Color,
    pub head_size: f32,
}

impl DebugArrow {
    /// Creates a new debug arrow
    pub fn new(start: [f32; 3], end: [f32; 3], color: Color) -> Self {
        Self {
            start,
            end,
            color,
            head_size: 0.5,
        }
    }

    /// Creates a debug arrow with custom head size
    pub fn with_head_size(start: [f32; 3], end: [f32; 3], color: Color, head_size: f32) -> Self {
        Self {
            start,
            end,
            color,
            head_size,
        }
    }
}

/// Collection of debug drawing primitives
#[derive(Debug, Default)]
pub struct DebugDraw {
    pub lines: Vec<DebugLine>,
    pub triangles: Vec<DebugTriangle>,
    pub text: Vec<DebugText>,
    pub circles: Vec<DebugCircle>,
    pub spheres: Vec<DebugSphere>,
    pub bounds: Vec<DebugBounds>,
    pub arrows: Vec<DebugArrow>,
}

impl DebugDraw {
    /// Creates a new debug draw collection
    pub fn new() -> Self {
        Self::default()
    }

    /// Clears all debug drawing primitives
    pub fn clear(&mut self) {
        self.lines.clear();
        self.triangles.clear();
        self.text.clear();
        self.circles.clear();
        self.spheres.clear();
        self.bounds.clear();
        self.arrows.clear();
    }

    /// Adds a debug line
    pub fn line(&mut self, start: [f32; 3], end: [f32; 3], color: Color) {
        self.lines.push(DebugLine::new(start, end, color));
    }

    /// Adds a debug line with thickness
    pub fn thick_line(&mut self, start: [f32; 3], end: [f32; 3], color: Color, thickness: f32) {
        self.lines
            .push(DebugLine::with_thickness(start, end, color, thickness));
    }

    /// Adds a debug triangle
    pub fn triangle(&mut self, p0: [f32; 3], p1: [f32; 3], p2: [f32; 3], color: Color) {
        self.triangles
            .push(DebugTriangle::colored(p0, p1, p2, color));
    }

    /// Adds a wireframe debug triangle
    pub fn wireframe_triangle(&mut self, p0: [f32; 3], p1: [f32; 3], p2: [f32; 3], color: Color) {
        let mut tri = DebugTriangle::colored(p0, p1, p2, color);
        tri.wireframe = true;
        self.triangles.push(tri);
    }

    /// Adds debug text
    pub fn text(&mut self, position: [f32; 3], text: impl Into<String>, color: Color) {
        self.text.push(DebugText::new(position, text.into(), color));
    }

    /// Adds debug text with size
    pub fn sized_text(
        &mut self,
        position: [f32; 3],
        text: impl Into<String>,
        color: Color,
        size: f32,
    ) {
        self.text
            .push(DebugText::with_size(position, text.into(), color, size));
    }

    /// Adds a debug circle
    pub fn circle(&mut self, center: [f32; 3], radius: f32, color: Color) {
        self.circles.push(DebugCircle::new(center, radius, color));
    }

    /// Adds a debug sphere
    pub fn sphere(&mut self, center: [f32; 3], radius: f32, color: Color) {
        self.spheres.push(DebugSphere::new(center, radius, color));
    }

    /// Adds a wireframe debug sphere
    pub fn wireframe_sphere(&mut self, center: [f32; 3], radius: f32, color: Color) {
        self.spheres
            .push(DebugSphere::wireframe(center, radius, color));
    }

    /// Adds debug bounding box
    pub fn bounds(&mut self, min: [f32; 3], max: [f32; 3], color: Color) {
        self.bounds.push(DebugBounds::new(min, max, color));
    }

    /// Adds filled debug bounding box
    pub fn filled_bounds(&mut self, min: [f32; 3], max: [f32; 3], color: Color) {
        self.bounds.push(DebugBounds::filled(min, max, color));
    }

    /// Adds a debug arrow
    pub fn arrow(&mut self, start: [f32; 3], end: [f32; 3], color: Color) {
        self.arrows.push(DebugArrow::new(start, end, color));
    }

    /// Adds a debug arrow with custom head size
    pub fn sized_arrow(&mut self, start: [f32; 3], end: [f32; 3], color: Color, head_size: f32) {
        self.arrows
            .push(DebugArrow::with_head_size(start, end, color, head_size));
    }

    /// Gets the total number of debug primitives
    pub fn primitive_count(&self) -> usize {
        self.lines.len()
            + self.triangles.len()
            + self.text.len()
            + self.circles.len()
            + self.spheres.len()
            + self.bounds.len()
            + self.arrows.len()
    }

    /// Checks if the debug draw is empty
    pub fn is_empty(&self) -> bool {
        self.primitive_count() == 0
    }
}

/// Debug visualization configuration
#[derive(Debug, Clone)]
pub struct DebugConfig {
    /// Whether to show navigation mesh polygons
    pub show_nav_mesh: bool,
    /// Whether to show navigation mesh bounds
    pub show_nav_mesh_bounds: bool,
    /// Whether to show agent paths
    pub show_paths: bool,
    /// Whether to show agent positions
    pub show_agents: bool,
    /// Whether to show agent velocities
    pub show_velocities: bool,
    /// Whether to show formations
    pub show_formations: bool,
    /// Whether to show RVO collision circles
    pub show_rvo_circles: bool,
    /// Whether to show obstacle bounds
    pub show_obstacles: bool,
    /// Whether to show tile boundaries
    pub show_tile_boundaries: bool,
    /// Whether to show off-mesh connections
    pub show_off_mesh_connections: bool,
    /// Scale factor for velocity arrows
    pub velocity_scale: f32,
    /// Agent radius visualization scale
    pub agent_radius_scale: f32,
    /// Text size for labels
    pub text_size: f32,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            show_nav_mesh: true,
            show_nav_mesh_bounds: false,
            show_paths: true,
            show_agents: true,
            show_velocities: true,
            show_formations: true,
            show_rvo_circles: false,
            show_obstacles: true,
            show_tile_boundaries: false,
            show_off_mesh_connections: true,
            velocity_scale: 1.0,
            agent_radius_scale: 1.0,
            text_size: 12.0,
        }
    }
}
