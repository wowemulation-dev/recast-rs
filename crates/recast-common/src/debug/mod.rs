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
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Alpha defaults to 1.0.
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self::new(r, g, b, 1.0)
    }

    pub const fn from_rgba_bytes(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self::new(
            r as f32 / 255.0,
            g as f32 / 255.0,
            b as f32 / 255.0,
            a as f32 / 255.0,
        )
    }

    /// Alpha defaults to 1.0.
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
    pub fn new(start: [f32; 3], end: [f32; 3], color: Color) -> Self {
        Self {
            start,
            end,
            color,
            thickness: 1.0,
        }
    }

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
    pub fn new(v0: DebugVertex, v1: DebugVertex, v2: DebugVertex) -> Self {
        Self {
            vertices: [v0, v1, v2],
            wireframe: false,
        }
    }

    pub fn wireframe(v0: DebugVertex, v1: DebugVertex, v2: DebugVertex) -> Self {
        Self {
            vertices: [v0, v1, v2],
            wireframe: true,
        }
    }

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
    pub fn new(position: [f32; 3], text: String, color: Color) -> Self {
        Self {
            position,
            text,
            color,
            size: 12.0,
        }
    }

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
    pub fn new(center: [f32; 3], radius: f32, color: Color) -> Self {
        Self {
            center,
            radius,
            color,
            segments: 32,
        }
    }

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
    pub fn new(center: [f32; 3], radius: f32, color: Color) -> Self {
        Self {
            center,
            radius,
            color,
            wireframe: false,
        }
    }

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
    pub fn new(min: [f32; 3], max: [f32; 3], color: Color) -> Self {
        Self {
            min,
            max,
            color,
            wireframe: true,
        }
    }

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
    pub fn new(start: [f32; 3], end: [f32; 3], color: Color) -> Self {
        Self {
            start,
            end,
            color,
            head_size: 0.5,
        }
    }

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
    pub fn new() -> Self {
        Self::default()
    }

    pub fn clear(&mut self) {
        self.lines.clear();
        self.triangles.clear();
        self.text.clear();
        self.circles.clear();
        self.spheres.clear();
        self.bounds.clear();
        self.arrows.clear();
    }

    pub fn line(&mut self, start: [f32; 3], end: [f32; 3], color: Color) {
        self.lines.push(DebugLine::new(start, end, color));
    }

    pub fn thick_line(&mut self, start: [f32; 3], end: [f32; 3], color: Color, thickness: f32) {
        self.lines
            .push(DebugLine::with_thickness(start, end, color, thickness));
    }

    pub fn triangle(&mut self, p0: [f32; 3], p1: [f32; 3], p2: [f32; 3], color: Color) {
        self.triangles
            .push(DebugTriangle::colored(p0, p1, p2, color));
    }

    pub fn wireframe_triangle(&mut self, p0: [f32; 3], p1: [f32; 3], p2: [f32; 3], color: Color) {
        let mut tri = DebugTriangle::colored(p0, p1, p2, color);
        tri.wireframe = true;
        self.triangles.push(tri);
    }

    pub fn text(&mut self, position: [f32; 3], text: impl Into<String>, color: Color) {
        self.text.push(DebugText::new(position, text.into(), color));
    }

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

    pub fn circle(&mut self, center: [f32; 3], radius: f32, color: Color) {
        self.circles.push(DebugCircle::new(center, radius, color));
    }

    pub fn sphere(&mut self, center: [f32; 3], radius: f32, color: Color) {
        self.spheres.push(DebugSphere::new(center, radius, color));
    }

    pub fn wireframe_sphere(&mut self, center: [f32; 3], radius: f32, color: Color) {
        self.spheres
            .push(DebugSphere::wireframe(center, radius, color));
    }

    pub fn bounds(&mut self, min: [f32; 3], max: [f32; 3], color: Color) {
        self.bounds.push(DebugBounds::new(min, max, color));
    }

    pub fn filled_bounds(&mut self, min: [f32; 3], max: [f32; 3], color: Color) {
        self.bounds.push(DebugBounds::filled(min, max, color));
    }

    pub fn arrow(&mut self, start: [f32; 3], end: [f32; 3], color: Color) {
        self.arrows.push(DebugArrow::new(start, end, color));
    }

    pub fn sized_arrow(&mut self, start: [f32; 3], end: [f32; 3], color: Color, head_size: f32) {
        self.arrows
            .push(DebugArrow::with_head_size(start, end, color, head_size));
    }

    pub fn primitive_count(&self) -> usize {
        self.lines.len()
            + self.triangles.len()
            + self.text.len()
            + self.circles.len()
            + self.spheres.len()
            + self.bounds.len()
            + self.arrows.len()
    }

    pub fn is_empty(&self) -> bool {
        self.primitive_count() == 0
    }
}

/// Debug visualization configuration
#[derive(Debug, Clone)]
pub struct DebugConfig {
    pub show_nav_mesh: bool,
    pub show_nav_mesh_bounds: bool,
    pub show_paths: bool,
    pub show_agents: bool,
    pub show_velocities: bool,
    pub show_formations: bool,
    pub show_rvo_circles: bool,
    pub show_obstacles: bool,
    pub show_tile_boundaries: bool,
    pub show_off_mesh_connections: bool,
    pub velocity_scale: f32,
    pub agent_radius_scale: f32,
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
