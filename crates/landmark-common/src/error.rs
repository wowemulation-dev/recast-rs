/// Error for mesh I/O and structural validation
#[derive(thiserror::Error, Debug)]
pub enum MeshError {
    #[error("OBJ parse error: {0}")]
    ObjParse(String),

    #[error("vertex array length {len} is not a multiple of 3")]
    VertexArrayNotTripled { len: usize },

    #[error("index array length {len} is not a multiple of 3")]
    IndexArrayNotTripled { len: usize },

    #[error("triangle index out of bounds: ({i0}, {i1}, {i2}), vertex count: {vertex_count}")]
    TriangleIndexOutOfBounds {
        i0: usize,
        i1: usize,
        i2: usize,
        vertex_count: usize,
    },

    #[cfg(feature = "std")]
    #[error(transparent)]
    Io(#[from] std::io::Error),
}
