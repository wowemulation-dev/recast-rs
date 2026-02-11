//! Error types for the recast crate
//!
//! These types replace the generic `recast_common::Error` variants with structured
//! errors that carry context about what went wrong during navmesh generation.

/// Error during recast configuration validation
#[derive(thiserror::Error, Debug)]
pub enum ConfigError {
    #[error("grid dimensions out of range: width={width}, height={height}")]
    InvalidGridSize { width: i32, height: i32 },

    #[error("cell size or height must be positive: cs={cs}, ch={ch}")]
    InvalidCellDimensions { cs: f32, ch: f32 },

    #[error("walkable slope angle out of range: {angle}")]
    InvalidWalkableSlope { angle: f32 },

    #[error("max vertices per polygon must be >= 3, got {count}")]
    TooFewVertsPerPoly { count: i32 },
}

/// Error during navmesh generation
#[derive(thiserror::Error, Debug)]
pub enum BuildError {
    #[error(transparent)]
    Config(#[from] ConfigError),

    #[error("too many vertices: {count} (max: {max})")]
    TooManyVertices { count: usize, max: usize },

    #[error("too many polygons: {count} (max: {max})")]
    TooManyPolygons { count: usize, max: usize },

    #[error("too many edges")]
    TooManyEdges,

    #[error("region ID overflow")]
    RegionIdOverflow,

    #[error("layer overflow at ({x}, {y}): too many overlapping walkable platforms")]
    LayerOverflow { x: i32, y: i32 },

    #[error("span position out of bounds: ({x}, {y})")]
    SpanOutOfBounds { x: i32, y: i32 },

    #[error("invalid span height: min ({min}) > max ({max})")]
    InvalidSpanHeight { min: u32, max: u32 },

    #[error("cell not found: ({x}, {y})")]
    CellNotFound { x: i32, y: i32 },

    #[error("cell index out of bounds: {index}")]
    CellIndexOutOfBounds { index: usize },

    #[error("polygon must have at least 3 vertices")]
    DegeneratePolygon,

    #[error("cannot merge empty mesh list")]
    EmptyMeshList,

    #[error("incompatible mesh at index {index}: {detail}")]
    IncompatibleMesh { index: usize, detail: &'static str },

    #[error("scale factors must be positive")]
    InvalidScaleFactors,

    #[error("vertex array length must be a multiple of 3")]
    VertexArrayNotTripled,

    #[error("index array length must be a multiple of 3")]
    IndexArrayNotTripled,

    #[error("triangle index out of bounds: ({i0}, {i1}, {i2}), max: {max}")]
    TriangleIndexOutOfBounds {
        i0: usize,
        i1: usize,
        i2: usize,
        max: usize,
    },

    #[error("heightfield missing column at ({x}, {z})")]
    MissingColumn { x: i32, z: i32 },

    #[error("empty spans in {context}")]
    EmptySpans { context: &'static str },

    #[error("triangle must have exactly 3 vertices")]
    InvalidTriangle,

    #[error("detail mesh generation failed")]
    DetailMeshFailed,
}

/// Error during convex volume creation
#[derive(thiserror::Error, Debug)]
pub enum ConvexVolumeError {
    #[error("requires at least {min} vertices, got {count}")]
    TooFewVertices { count: usize, min: usize },

    #[error("too many vertices: {count} (max: {max})")]
    TooManyVertices { count: usize, max: usize },

    #[error("min_height ({min}) exceeds max_height ({max})")]
    InvalidHeight { min: f32, max: f32 },

    #[error("vertices do not form a convex polygon")]
    NotConvex,
}
