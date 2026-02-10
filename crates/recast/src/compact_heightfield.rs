//! Compact heightfield representation for Recast
//!
//! The compact heightfield is a more memory-efficient representation
//! of the heightfield that also includes information about walkable spans
//! and connections between spans.

use glam::Vec3;

use super::heightfield::Heightfield;
use super::watershed;
use crate::error::BuildError;

/// A compact cell in the heightfield
#[derive(Debug, Clone)]
pub struct CompactCell {
    /// Index of the first span in the cell (or None if the cell has no spans)
    pub index: Option<usize>,
    /// Number of spans in the cell
    pub count: usize,
}

impl CompactCell {
    /// Creates a new compact cell
    pub fn new(index: Option<usize>, count: usize) -> Self {
        Self { index, count }
    }
}

/// A compact span in the heightfield
#[derive(Debug, Clone)]
pub struct CompactSpan {
    /// The minimum height of the span
    pub min: i16,
    /// The maximum height of the span
    pub max: i16,
    /// Area ID (0 = not walkable)
    pub area: u8,
    /// Region ID (0 = not in region)
    pub reg: u16,
    /// Index of the first connection (or None if the span has no connections)
    pub first_connection: Option<usize>,
    /// Y coordinate of the span
    pub y: i32,
    /// Height of the span (max - min)
    pub h: u8,
    /// Connections to neighbor spans in 4 directions (63 = not connected)
    pub con: [usize; 4],
}

impl CompactSpan {
    /// Creates a new compact span
    pub fn new(min: i16, max: i16, area: u8, y: i32) -> Self {
        Self {
            min,
            max,
            area,
            reg: 0, // Initialize with no region
            first_connection: None,
            y,
            h: (max - min) as u8,
            con: [63; 4], // 63 = RC_NOT_CONNECTED
        }
    }
}

/// A connection between spans
#[derive(Debug, Clone)]
pub struct CompactConnection {
    /// Reference to the span that is connected to
    pub ref_span: usize,
    /// Direction of the connection (0-7)
    pub dir: u8,
    /// Next connection for the current span (or None if this is the last connection)
    pub next: Option<usize>,
}

impl CompactConnection {
    /// Creates a new compact connection
    pub fn new(ref_span: usize, dir: u8, next: Option<usize>) -> Self {
        Self {
            ref_span,
            dir,
            next,
        }
    }
}

/// Direction constants for connections
#[allow(dead_code)]
pub const DIR_NONE: u8 = 0xff;
#[allow(dead_code)]
pub const DIR_NW: u8 = 0;
#[allow(dead_code)]
pub const DIR_N: u8 = 1;
#[allow(dead_code)]
pub const DIR_NE: u8 = 2;
#[allow(dead_code)]
pub const DIR_E: u8 = 3;
#[allow(dead_code)]
pub const DIR_SE: u8 = 4;
#[allow(dead_code)]
pub const DIR_S: u8 = 5;
#[allow(dead_code)]
pub const DIR_SW: u8 = 6;
#[allow(dead_code)]
pub const DIR_W: u8 = 7;

/// Offset in x for each 8-direction
#[allow(dead_code)]
pub const DIR_OFFSET_X: [i32; 8] = [-1, 0, 1, 1, 1, 0, -1, -1];
/// Offset in z for each 8-direction
#[allow(dead_code)]
pub const DIR_OFFSET_Z: [i32; 8] = [-1, -1, -1, 0, 1, 1, 1, 0];

/// Region constants
/// Border region flag (region is on the border of the walkable area)
#[allow(dead_code)]
pub const RC_BORDER_REG: u16 = 0x8000;
/// Multiple regions flag (polygon touches multiple regions)
#[allow(dead_code)]
pub const RC_MULTIPLE_REGS: u16 = 0;
/// Border vertex flag
#[allow(dead_code)]
pub const RC_BORDER_VERTEX: u32 = 0x10000;
/// Area border flag
#[allow(dead_code)]
pub const RC_AREA_BORDER: u32 = 0x20000;

/// Compact heightfield structure
#[derive(Debug, Clone)]
pub struct CompactHeightfield {
    /// Width of the heightfield along the x-axis
    pub width: i32,
    /// Height (depth) of the heightfield along the z-axis
    pub height: i32,

    /// The minimum bounds of the heightfield's AABB
    pub bmin: Vec3,
    /// The maximum bounds of the heightfield's AABB
    pub bmax: Vec3,

    /// Cell size (horizontal resolution)
    pub cs: f32,
    /// Cell height (vertical resolution)
    pub ch: f32,

    /// Grid of compact cells
    pub cells: Vec<CompactCell>,
    /// Array of compact spans
    pub spans: Vec<CompactSpan>,
    /// Array of connections between spans
    pub connections: Vec<CompactConnection>,
    /// Array of area IDs for each span
    pub areas: Vec<u8>,
    /// Array of distance values per span (for watershed)
    pub dist: Vec<u16>,

    /// Number of spans that are walkable
    pub walkable_span_count: usize,
    /// Number of spans that have been assigned to an area
    pub walkable_area_count: usize,
    /// Cell count (width * height)
    pub cell_count: usize,
    /// Span count (total number of spans)
    pub span_count: usize,
    /// Maximum height in the heightfield
    pub max_height: u16,
    /// Maximum distance value in distance field
    pub max_distance: u16,
    /// Maximum region id
    pub max_regions: u16,
}

#[allow(dead_code)]
impl CompactHeightfield {
    /// Builds a compact heightfield from a regular heightfield.
    ///
    /// Only walkable spans (area != 0) are included, matching C++ `rcBuildCompactHeightfield`.
    /// `walkable_height` and `walkable_climb` are used for neighbor connection criteria.
    pub fn build_from_heightfield(
        heightfield: &Heightfield,
        walkable_height: i32,
        walkable_climb: i32,
    ) -> Result<Self, BuildError> {
        let width = heightfield.width;
        let height = heightfield.height;
        let bmin = heightfield.bmin;
        let mut bmax = heightfield.bmax;
        let cs = heightfield.cs;
        let ch = heightfield.ch;

        // C++ adds walkable_height * ch to bmax.y
        bmax.y += walkable_height as f32 * ch;

        const MAX_HEIGHT: i32 = 0xffff;

        // Count walkable spans only (matching C++)
        let mut span_count = 0;
        for column in heightfield.spans.values() {
            let mut current = column.clone();
            while let Some(span_rc) = current {
                let span = span_rc.borrow();
                if span.area != 0 {
                    span_count += 1;
                }
                current = span.next.clone();
            }
        }

        // Allocate memory for the compact heightfield
        let mut cells = Vec::with_capacity((width * height) as usize);
        let mut spans = Vec::with_capacity(span_count);
        let connections = Vec::new(); // Will be filled in later
        let mut areas = Vec::with_capacity(span_count);

        // Build the compact heightfield
        let mut walkable_span_count = 0;
        let mut max_height = 0u16;

        for z in 0..height {
            for x in 0..width {
                let column = heightfield
                    .spans
                    .get(&(x, z))
                    .ok_or_else(|| BuildError::MissingColumn { x, z })?;

                // If the column has no spans, add an empty cell
                if column.is_none() {
                    cells.push(CompactCell::new(None, 0));
                    continue;
                }

                // Count walkable spans in this cell
                let mut walkable_count = 0;
                let mut current = column.clone();
                while let Some(span_rc) = current {
                    let span = span_rc.borrow();
                    if span.area != 0 {
                        walkable_count += 1;
                    }
                    current = span.next.clone();
                }

                if walkable_count == 0 {
                    cells.push(CompactCell::new(None, 0));
                    current = column.clone();
                    // Still need to advance past this column
                    continue;
                }

                // Add the cell
                cells.push(CompactCell::new(Some(spans.len()), walkable_count));

                // Add only walkable spans (matching C++ rcBuildCompactHeightfield)
                current = column.clone();
                while let Some(span_rc) = current {
                    let span = span_rc.borrow();

                    if span.area != 0 {
                        // C++ convention: y = smax (top of solid = walkable surface)
                        let bot = span.max as i32;
                        // Air gap: from top of this solid to bottom of next solid above
                        let top = if let Some(ref next) = span.next {
                            next.borrow().min as i32
                        } else {
                            MAX_HEIGHT
                        };
                        let air_height = (top - bot).clamp(0, 255);

                        spans.push(CompactSpan {
                            min: span.min,
                            max: span.max,
                            area: span.area,
                            reg: 0,
                            first_connection: None,
                            y: bot,
                            h: air_height as u8,
                            con: [63; 4], // RC_NOT_CONNECTED
                        });
                        areas.push(span.area);

                        // Track max height
                        max_height = max_height.max(span.max as u16);
                        walkable_span_count += 1;
                    }

                    current = span.next.clone();
                }
            }
        }

        // Create distance field array
        let dist = vec![0u16; spans.len()];

        // Create the compact heightfield
        let mut chf = CompactHeightfield {
            width,
            height,
            bmin,
            bmax,
            cs,
            ch,
            cells,
            spans,
            connections,
            areas,
            dist,
            walkable_span_count,
            walkable_area_count: 0,
            cell_count: (width * height) as usize,
            span_count,
            max_height,
            max_distance: 0,
            max_regions: 0,
        };

        // Build connections between spans
        chf.build_connections(walkable_height, walkable_climb)?;

        Ok(chf)
    }

    /// Builds connections between spans.
    ///
    /// Matches C++ `rcBuildCompactHeightfield` connection logic:
    /// - Only 4 cardinal directions (W=0, S=1, E=2, N=3)
    /// - Air space overlap must be >= walkable_height
    /// - Step height between walkable surfaces must be <= walkable_climb
    /// - Takes first valid connection per direction (not best)
    fn build_connections(
        &mut self,
        walkable_height: i32,
        walkable_climb: i32,
    ) -> Result<(), BuildError> {
        let width = self.width;
        let height = self.height;

        // C++ 4-direction offsets: W=0, S=1, E=2, N=3
        let dir_offset_x: [i32; 4] = [-1, 0, 1, 0];
        let dir_offset_z: [i32; 4] = [0, 1, 0, -1];

        // Map 4-dir to 8-dir for linked list storage
        let dir4_to_dir8: [u8; 4] = [7, 5, 3, 1]; // W=7, S=5, E=3, N=1

        const MAX_LAYERS: usize = 62; // RC_NOT_CONNECTED - 1

        // Process cells to find connections (matching C++ exactly)
        let num_cells = (width * height) as usize;
        let mut connections_per_span: Vec<Vec<(usize, u8)>> = vec![Vec::new(); self.spans.len()];

        for cell_idx in 0..num_cells {
            let cell = &self.cells[cell_idx];

            if let Some(first_span_idx) = cell.index {
                let x = (cell_idx as i32) % width;
                let z = (cell_idx as i32) / width;

                for s in 0..cell.count {
                    let span_idx = first_span_idx + s;

                    // Skip unwalkable spans
                    if self.areas[span_idx] == 0 {
                        continue;
                    }

                    let span_y = self.spans[span_idx].y;
                    let span_h = self.spans[span_idx].h as i32;

                    // Check 4 cardinal directions
                    for dir in 0..4usize {
                        // Initialize as not connected
                        self.spans[span_idx].con[dir] = 63; // RC_NOT_CONNECTED

                        let nx = x + dir_offset_x[dir];
                        let nz = z + dir_offset_z[dir];

                        // Skip out-of-bounds neighbors
                        if nx < 0 || nz < 0 || nx >= width || nz >= height {
                            continue;
                        }

                        let neighbor_cell_idx = (nz * width + nx) as usize;
                        let neighbor_cell = &self.cells[neighbor_cell_idx];

                        if let Some(first_neighbor_idx) = neighbor_cell.index {
                            for ns in 0..neighbor_cell.count {
                                let neighbor_idx = first_neighbor_idx + ns;
                                let neighbor_span = &self.spans[neighbor_idx];

                                // C++ connection criteria:
                                // bot = max(span.y, neighbor.y)
                                // top = min(span.y + span.h, neighbor.y + neighbor.h)
                                // (top - bot) >= walkableHeight AND
                                // abs(neighbor.y - span.y) <= walkableClimb
                                let bot = span_y.max(neighbor_span.y);
                                let top =
                                    (span_y + span_h).min(neighbor_span.y + neighbor_span.h as i32);

                                if (top - bot) >= walkable_height
                                    && (neighbor_span.y - span_y).abs() <= walkable_climb
                                {
                                    // C++ stores layer index (offset within neighbor cell)
                                    let layer_index = ns;
                                    if layer_index > MAX_LAYERS {
                                        continue;
                                    }
                                    self.spans[span_idx].con[dir] = layer_index;

                                    // Also store in linked list for get_neighbor compatibility
                                    connections_per_span[span_idx]
                                        .push((neighbor_idx, dir4_to_dir8[dir]));
                                    break; // C++ takes first valid match
                                }
                            }
                        }
                    }
                }
            }
        }

        // Build the linked list connections
        let mut span_to_first_connection = vec![None; self.spans.len()];

        // Estimate connections: average ~4 per walkable span
        let estimated_connections = self.spans.len() * 4;
        self.connections = Vec::with_capacity(estimated_connections);

        for (span_idx, span_connections) in connections_per_span.iter().enumerate() {
            let mut first_connection = None;

            for &(neighbor_span_idx, dir8) in span_connections.iter().rev() {
                let connection = CompactConnection::new(neighbor_span_idx, dir8, first_connection);
                let connection_idx = self.connections.len();
                self.connections.push(connection);
                first_connection = Some(connection_idx);
            }

            span_to_first_connection[span_idx] = first_connection;
        }

        // Update spans with first connection indices
        for (span_idx, first_connection) in span_to_first_connection.iter().enumerate() {
            self.spans[span_idx].first_connection = *first_connection;
        }

        Ok(())
    }

    /// Helper to find which cell a span belongs to
    fn get_cell_index_for_span(&self, span_idx: usize) -> Option<usize> {
        for (cell_idx, cell) in self.cells.iter().enumerate() {
            if let Some(first_idx) = cell.index {
                if span_idx >= first_idx && span_idx < first_idx + cell.count {
                    return Some(cell_idx);
                }
            }
        }
        None
    }

    /// Counts the number of spans in each area and assigns area IDs
    pub fn mark_areas(&mut self) -> Result<(), BuildError> {
        let mut max_area_id = 0;
        let mut area_counts = vec![0; 256]; // Area IDs are u8, so max 256 areas

        // Count spans in each area
        for &area in &self.areas {
            if area != 0 {
                area_counts[area as usize] += 1;
                max_area_id = max_area_id.max(area);
            }
        }

        // Count the number of areas with spans
        let mut area_count = 0;
        for area_id in 1..=max_area_id {
            if area_counts[area_id as usize] > 0 {
                area_count += 1;
            }
        }

        self.walkable_area_count = area_count;

        Ok(())
    }

    /// Gets the span at a specific position
    pub fn get_span_at(&self, x: i32, z: i32, y: i32) -> Option<&CompactSpan> {
        if x < 0 || z < 0 || x >= self.width || z >= self.height {
            return None;
        }

        let cell_idx = (z * self.width + x) as usize;
        let cell = &self.cells[cell_idx];

        if let Some(first_span_idx) = cell.index {
            for s in 0..cell.count {
                let span_idx = first_span_idx + s;
                let span = &self.spans[span_idx];

                // Check if the span contains the specified height
                if span.min <= y as i16 && span.max >= y as i16 {
                    return Some(span);
                }
            }
        }

        None
    }

    /// Gets the neighboring span in a specific direction
    pub fn get_neighbor(&self, span_idx: usize, dir: u8) -> Option<usize> {
        let span = &self.spans[span_idx];
        let mut connection_idx = span.first_connection;

        while let Some(conn_idx) = connection_idx {
            let connection = &self.connections[conn_idx];

            if connection.dir == dir {
                return Some(connection.ref_span);
            }

            connection_idx = connection.next;
        }

        None
    }

    /// Builds the distance field for the compact heightfield
    /// This implements the C++ rcBuildDistanceField algorithm
    pub fn build_distance_field(&mut self) -> Result<(), BuildError> {
        let span_count = self.spans.len();

        // Initialize distance field if not already done
        if self.dist.len() != span_count {
            self.dist = vec![0xffffu16; span_count];
        }

        // Use a working buffer for calculations
        let mut src = vec![0xffffu16; span_count];
        src.copy_from_slice(&self.dist);
        let mut dst = vec![0u16; span_count];

        // Calculate distance field using two-pass algorithm
        let max_dist = self.calculate_distance_field(&mut src)?;

        // Apply box blur for smoothing
        if self.box_blur(1, &mut src, &mut dst)? {
            // If box_blur returns true, result is in dst
            self.dist.copy_from_slice(&dst);
        } else {
            // Result is in src
            self.dist.copy_from_slice(&src);
        }

        // Store max distance
        self.max_distance = max_dist;

        Ok(())
    }

    /// Calculates the distance field using the C++ two-pass algorithm.
    ///
    /// Uses cardinal directions (W=7, S=5, E=3, N=1 in 8-dir system) for propagation,
    /// with diagonal checks via two consecutive cardinal hops.
    fn calculate_distance_field(&self, src: &mut [u16]) -> Result<u16, BuildError> {
        let w = self.width;
        let h = self.height;

        // 8-dir constants for cardinal directions
        const DIR_W: u8 = 7;
        const DIR_S: u8 = 5;
        const DIR_E: u8 = 3;
        const DIR_N: u8 = 1;

        // Initialize all distances to infinity
        for distance in src.iter_mut() {
            *distance = 0xffff;
        }

        // Mark boundary cells (spans that don't have all 4 neighbors with same area)
        for y in 0..h {
            for x in 0..w {
                let cell_idx = (y * w + x) as usize;
                let cell = &self.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let span_idx = first_span_idx + s;

                        let mut neighbor_count = 0;

                        // Check all 4 cardinal directions using correct 8-dir indices
                        for dir in [DIR_N, DIR_E, DIR_S, DIR_W] {
                            if let Some(neighbor_idx) = self.get_neighbor(span_idx, dir) {
                                if self.areas[neighbor_idx] == self.areas[span_idx] {
                                    neighbor_count += 1;
                                }
                            }
                        }

                        // If doesn't have 4 neighbors with same area, it's a boundary
                        if neighbor_count != 4 {
                            src[span_idx] = 0;
                        }
                    }
                }
            }
        }

        // Pass 1: Forward pass (top-left to bottom-right)
        // C++ checks West(0) and North(3) as cardinal, then diagonals NW and NE
        for y in 0..h {
            for x in 0..w {
                let cell_idx = (y * w + x) as usize;
                let cell = &self.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let span_idx = first_span_idx + s;

                        // Check West (cardinal: +2)
                        if let Some(west_idx) = self.get_neighbor(span_idx, DIR_W) {
                            if src[west_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[west_idx].saturating_add(2);
                            }

                            // NW diagonal: from West neighbor, go North (+3)
                            if let Some(nw_idx) = self.get_neighbor(west_idx, DIR_N) {
                                if src[nw_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[nw_idx].saturating_add(3);
                                }
                            }
                        }

                        // Check North (cardinal: +2)
                        if let Some(north_idx) = self.get_neighbor(span_idx, DIR_N) {
                            if src[north_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[north_idx].saturating_add(2);
                            }

                            // NE diagonal: from North neighbor, go East (+3)
                            if let Some(ne_idx) = self.get_neighbor(north_idx, DIR_E) {
                                if src[ne_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[ne_idx].saturating_add(3);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Pass 2: Backward pass (bottom-right to top-left)
        // C++ checks East(2) and South(1) as cardinal, then diagonals SE and SW
        for y in (0..h).rev() {
            for x in (0..w).rev() {
                let cell_idx = (y * w + x) as usize;
                let cell = &self.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let span_idx = first_span_idx + s;

                        // Check East (cardinal: +2)
                        if let Some(east_idx) = self.get_neighbor(span_idx, DIR_E) {
                            if src[east_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[east_idx].saturating_add(2);
                            }

                            // SE diagonal: from East neighbor, go South (+3)
                            if let Some(se_idx) = self.get_neighbor(east_idx, DIR_S) {
                                if src[se_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[se_idx].saturating_add(3);
                                }
                            }
                        }

                        // Check South (cardinal: +2)
                        if let Some(south_idx) = self.get_neighbor(span_idx, DIR_S) {
                            if src[south_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[south_idx].saturating_add(2);
                            }

                            // SW diagonal: from South neighbor, go West (+3)
                            if let Some(sw_idx) = self.get_neighbor(south_idx, DIR_W) {
                                if src[sw_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[sw_idx].saturating_add(3);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Find maximum distance
        let max_dist = src.iter().copied().max().unwrap_or(0);

        Ok(max_dist)
    }

    /// Applies box blur to smooth the distance field.
    /// Returns true if result is in dst, false if result is in src.
    ///
    /// Diagonals are reached via two consecutive cardinal hops, matching C++.
    fn box_blur(
        &self,
        threshold: u16,
        src: &mut [u16],
        dst: &mut [u16],
    ) -> Result<bool, BuildError> {
        let w = self.width;
        let h = self.height;
        let thr = threshold * 2;

        // Cardinal directions in 8-dir system
        // C++ order: dir 0=W, 1=S, 2=E, 3=N
        // 8-dir:     W=7, S=5, E=3, N=1
        let cardinal_dirs: [u8; 4] = [7, 5, 3, 1];
        // For each cardinal dir[i], the next cardinal dir[(i+1)&3] to reach diagonal:
        // W(7) → S(5) = SW, S(5) → E(3) = SE, E(3) → N(1) = NE, N(1) → W(7) = NW
        let next_cardinal: [u8; 4] = [5, 3, 1, 7];

        for y in 0..h {
            for x in 0..w {
                let cell_idx = (y * w + x) as usize;
                let cell = &self.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let span_idx = first_span_idx + s;
                        let cd = src[span_idx];

                        if cd <= thr {
                            dst[span_idx] = cd;
                            continue;
                        }

                        let mut d = cd as i32;
                        let mut neighbor_count = 1; // Count self

                        // Check 4 cardinal + 4 diagonal neighbors
                        for i in 0..4 {
                            let dir = cardinal_dirs[i];
                            if let Some(neighbor_idx) = self.get_neighbor(span_idx, dir) {
                                d += src[neighbor_idx] as i32;
                                neighbor_count += 1;

                                // Diagonal: from cardinal neighbor, go next cardinal direction
                                let next_dir = next_cardinal[i];
                                if let Some(diag_idx) = self.get_neighbor(neighbor_idx, next_dir) {
                                    d += src[diag_idx] as i32;
                                    neighbor_count += 1;
                                }
                            }
                        }

                        dst[span_idx] = (d / neighbor_count) as u16;
                    }
                }
            }
        }

        Ok(true) // Result is in dst
    }

    /// Gets the neighbor connection for a span in a specific direction
    /// Returns None if no connection exists
    /// Direction: 0=W, 1=S, 2=E, 3=N (4-direction system)
    pub fn get_neighbor_connection(&self, span_idx: usize, direction: usize) -> Option<usize> {
        if direction >= 4 {
            return None;
        }
        // Map 4-direction to 8-direction: W=7, S=5, E=3, N=1
        let dir8 = match direction {
            0 => 7, // West
            1 => 5, // South
            2 => 3, // East
            3 => 1, // North
            _ => return None,
        };
        self.get_neighbor(span_idx, dir8)
    }

    /// Gets the X offset for a direction
    pub fn get_dir_offset_x(&self, direction: usize) -> i32 {
        match direction {
            0 => -1, // West
            1 => 0,  // South
            2 => 1,  // East
            3 => 0,  // North
            _ => 0,
        }
    }

    /// Gets the Y (Z) offset for a direction
    pub fn get_dir_offset_y(&self, direction: usize) -> i32 {
        match direction {
            0 => 0,  // West
            1 => 1,  // South
            2 => 0,  // East
            3 => -1, // North
            _ => 0,
        }
    }

    /// Builds regions using watershed partitioning
    pub fn build_regions(
        &mut self,
        border_size: i32,
        min_region_area: i32,
        merge_region_area: i32,
    ) -> Result<(), BuildError> {
        // Use the proper watershed algorithm
        watershed::build_regions_watershed(self, border_size, min_region_area, merge_region_area)
    }

    /// Marks border regions around the edges of the heightfield
    fn mark_border_regions(
        &self,
        border_size: i32,
        region_ids: &mut [u16],
        next_region_id: &mut u16,
    ) -> Result<(), BuildError> {
        let w = self.width;
        let h = self.height;
        let bw = w.min(border_size);
        let bh = h.min(border_size);

        // Mark left and right borders
        self.paint_rect_region(0, bw, 0, h, *next_region_id | RC_BORDER_REG, region_ids)?;
        *next_region_id += 1;
        self.paint_rect_region(w - bw, w, 0, h, *next_region_id | RC_BORDER_REG, region_ids)?;
        *next_region_id += 1;

        // Mark top and bottom borders
        self.paint_rect_region(0, w, 0, bh, *next_region_id | RC_BORDER_REG, region_ids)?;
        *next_region_id += 1;
        self.paint_rect_region(0, w, h - bh, h, *next_region_id | RC_BORDER_REG, region_ids)?;
        *next_region_id += 1;

        Ok(())
    }

    /// Paints a rectangular region with the specified region ID
    fn paint_rect_region(
        &self,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
        region_id: u16,
        region_ids: &mut [u16],
    ) -> Result<(), BuildError> {
        for y in min_y..max_y {
            for x in min_x..max_x {
                let cell_idx = (y * self.width + x) as usize;
                if cell_idx < self.cells.len() {
                    let cell = &self.cells[cell_idx];

                    if let Some(first_span_idx) = cell.index {
                        for s in 0..cell.count {
                            let span_idx = first_span_idx + s;
                            if span_idx < region_ids.len() && self.areas[span_idx] != 0 {
                                region_ids[span_idx] = region_id;
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Flood fills a region starting from the given span
    fn flood_fill_region(
        &self,
        seed_idx: usize,
        region_id: u16,
        region_ids: &mut [u16],
    ) -> Result<(), BuildError> {
        let mut stack = vec![seed_idx];
        region_ids[seed_idx] = region_id;

        while let Some(span_idx) = stack.pop() {
            // Check all 4 cardinal neighbors
            // Using 8-direction constants: N=1, E=3, S=5, W=7
            for dir in [1u8, 3u8, 5u8, 7u8] {
                if let Some(neighbor_idx) = self.get_neighbor(span_idx, dir) {
                    // Only expand to walkable spans of same area that aren't assigned yet
                    if self.areas[neighbor_idx] != 0
                        && self.areas[neighbor_idx] == self.areas[span_idx]
                        && region_ids[neighbor_idx] == 0
                    {
                        region_ids[neighbor_idx] = region_id;
                        stack.push(neighbor_idx);
                    }
                }
            }
        }

        Ok(())
    }

    /// Filters out regions smaller than minimum area
    fn filter_small_regions(
        &self,
        region_ids: &mut [u16],
        min_region_area: i32,
    ) -> Result<(), BuildError> {
        // Count region areas
        let max_region = region_ids.iter().copied().max().unwrap_or(0) & !RC_BORDER_REG;
        let mut region_areas = vec![0i32; (max_region + 1) as usize];

        for &region_id in region_ids.iter() {
            let clean_id = region_id & !RC_BORDER_REG;
            if clean_id > 0 && (clean_id as usize) < region_areas.len() {
                region_areas[clean_id as usize] += 1;
            }
        }

        // Remove small regions (but keep border regions)
        for region_id in region_ids.iter_mut() {
            let clean_id = *region_id & !RC_BORDER_REG;
            if clean_id > 0 &&
               (*region_id & RC_BORDER_REG) == 0 && // Not a border region
               (clean_id as usize) < region_areas.len() &&
               region_areas[clean_id as usize] < min_region_area
            {
                *region_id = 0;
            }
        }

        Ok(())
    }

    /// Merges small regions with neighboring regions
    fn merge_small_regions(
        &self,
        region_ids: &mut [u16],
        merge_region_area: i32,
    ) -> Result<(), BuildError> {
        // This is a simplified version - the full C++ implementation is much more complex
        let max_region = region_ids.iter().copied().max().unwrap_or(0) & !RC_BORDER_REG;
        let mut region_areas = vec![0i32; (max_region + 1) as usize];

        // Count region areas
        for &region_id in region_ids.iter() {
            let clean_id = region_id & !RC_BORDER_REG;
            if clean_id > 0 && (clean_id as usize) < region_areas.len() {
                region_areas[clean_id as usize] += 1;
            }
        }

        // Find and merge small regions
        for region_id in 1..=max_region {
            if (region_id as usize) < region_areas.len()
                && region_areas[region_id as usize] > 0
                && region_areas[region_id as usize] < merge_region_area
            {
                // Find best neighbor to merge with
                if let Some(best_neighbor) =
                    self.find_best_merge_neighbor(region_ids, region_id, &region_areas)?
                {
                    // Merge regions
                    for span_region in region_ids.iter_mut() {
                        if (*span_region & !RC_BORDER_REG) == region_id {
                            *span_region = best_neighbor;
                        }
                    }

                    // Update area counts
                    if (best_neighbor as usize) < region_areas.len() {
                        region_areas[best_neighbor as usize] += region_areas[region_id as usize];
                    }
                    region_areas[region_id as usize] = 0;
                }
            }
        }

        Ok(())
    }

    /// Finds the best neighboring region to merge with
    fn find_best_merge_neighbor(
        &self,
        region_ids: &[u16],
        region_id: u16,
        region_areas: &[i32],
    ) -> Result<Option<u16>, BuildError> {
        use std::collections::HashMap;

        let mut neighbor_borders: HashMap<u16, i32> = HashMap::new();

        // Count border lengths with each neighbor
        for (span_idx, &span_region) in region_ids.iter().enumerate() {
            if (span_region & !RC_BORDER_REG) != region_id {
                continue;
            }

            // Check all 4 cardinal neighbors
            // Using 8-direction constants: N=1, E=3, S=5, W=7
            for dir in [1u8, 3u8, 5u8, 7u8] {
                if let Some(neighbor_idx) = self.get_neighbor(span_idx, dir) {
                    let neighbor_region = region_ids[neighbor_idx] & !RC_BORDER_REG;

                    if neighbor_region != region_id && neighbor_region != 0 {
                        *neighbor_borders.entry(neighbor_region).or_insert(0) += 1;
                    }
                }
            }
        }

        // Find neighbor with best score (longest border * region area)
        let mut best_neighbor = None;
        let mut best_score = 0i32;

        for (&neighbor_region, &border_length) in &neighbor_borders {
            let neighbor_area = region_areas
                .get(neighbor_region as usize)
                .copied()
                .unwrap_or(0);
            let score = border_length * (neighbor_area + 1); // +1 to avoid zero

            if score > best_score {
                best_score = score;
                best_neighbor = Some(neighbor_region);
            }
        }

        Ok(best_neighbor)
    }

    /// Compacts region IDs to remove gaps
    fn compact_region_ids(&self, region_ids: &mut [u16]) -> Result<(), BuildError> {
        use std::collections::HashMap;

        // Find all unique region IDs (excluding border flag)
        let mut unique_regions: Vec<u16> =
            region_ids.iter().map(|&id| id & !RC_BORDER_REG).collect();
        unique_regions.sort_unstable();
        unique_regions.dedup();

        // Create mapping from old region ID to new compact ID
        let mut region_mapping = HashMap::new();
        let mut new_region_id = 1u16;

        for &old_region_id in &unique_regions {
            if old_region_id > 0 {
                region_mapping.insert(old_region_id, new_region_id);
                new_region_id += 1;
            }
        }

        // Apply mapping while preserving border flags
        for region_id in region_ids.iter_mut() {
            let is_border = (*region_id & RC_BORDER_REG) != 0;
            let clean_id = *region_id & !RC_BORDER_REG;

            if let Some(&new_id) = region_mapping.get(&clean_id) {
                *region_id = new_id | (if is_border { RC_BORDER_REG } else { 0 });
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compact_heightfield_creation() {
        // Create a simple heightfield
        let width = 3;
        let height = 3;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(3.0, 3.0, 3.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        // Add some spans
        // Cell (0, 0): two spans with air gap of 2 (>= walkable_height)
        heightfield.add_span(0, 0, 0, 2, 1).unwrap();
        heightfield.add_span(0, 0, 4, 6, 1).unwrap();

        // Cell (1, 0)
        heightfield.add_span(1, 0, 0, 2, 1).unwrap();

        // Cell (0, 1)
        heightfield.add_span(0, 1, 0, 2, 1).unwrap();

        // Build compact heightfield
        let chf = CompactHeightfield::build_from_heightfield(&heightfield, 2, 1).unwrap();

        // Check the compact heightfield properties
        assert_eq!(chf.width, width);
        assert_eq!(chf.height, height);
        assert_eq!(chf.cells.len(), (width * height) as usize);
        assert_eq!(chf.spans.len(), 4); // 4 spans total
        assert!(!chf.connections.is_empty()); // There should be some connections
    }

    #[test]
    fn test_get_span_at() {
        // Create a simple heightfield
        let width = 3;
        let height = 3;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(3.0, 3.0, 3.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        // Add a span to cell (1, 1)
        heightfield.add_span(1, 1, 0, 2, 1).unwrap();

        // Build compact heightfield
        let chf = CompactHeightfield::build_from_heightfield(&heightfield, 2, 1).unwrap();

        // Check if we can get the span at (1, 1, 1)
        let span = chf.get_span_at(1, 1, 1);
        assert!(span.is_some());
        let span = span.unwrap();
        assert_eq!(span.min, 0);
        assert_eq!(span.max, 2);
        assert_eq!(span.area, 1);

        // Check a position with no span
        let span = chf.get_span_at(0, 0, 0);
        assert!(span.is_none());
    }

    #[test]
    fn test_get_neighbor() {
        // Create a simple heightfield with two adjacent cells
        let width = 2;
        let height = 1;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(2.0, 1.0, 1.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        // Add spans to cells (0, 0) and (1, 0)
        heightfield.add_span(0, 0, 0, 1, 1).unwrap();
        heightfield.add_span(1, 0, 0, 1, 1).unwrap();

        // Build compact heightfield
        let chf = CompactHeightfield::build_from_heightfield(&heightfield, 2, 1).unwrap();

        // Check if the spans are connected
        let span0_idx = 0; // First span at (0, 0)
        let neighbor_idx = chf.get_neighbor(span0_idx, DIR_E);
        assert!(neighbor_idx.is_some());
        let neighbor_idx = neighbor_idx.unwrap();
        assert_eq!(neighbor_idx, 1); // Should be the span at (1, 0)
    }
}
