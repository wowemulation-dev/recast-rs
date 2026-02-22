//! Compact heightfield representation for Recast
//!
//! The compact heightfield is a more memory-efficient representation
//! of the heightfield that also includes information about walkable spans
//! and connections between spans.

use glam::Vec3;

use super::heightfield::{Heightfield, SPAN_NULL};
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

/// Not connected sentinel for `CompactSpan::con` entries.
pub const RC_NOT_CONNECTED: u8 = 63;

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
    /// Y coordinate of the span
    pub y: i32,
    /// Height of the span (max - min)
    pub h: u8,
    /// Connections to neighbor spans in 4 directions.
    /// Each entry is a layer index within the neighbor cell, or
    /// [`RC_NOT_CONNECTED`] (63) if no connection exists.
    pub con: [u8; 4],
}

impl CompactSpan {
    /// Creates a new compact span
    pub fn new(min: i16, max: i16, area: u8, y: i32) -> Self {
        Self {
            min,
            max,
            area,
            reg: 0,
            y,
            h: (max - min) as u8,
            con: [RC_NOT_CONNECTED; 4],
        }
    }
}

/// Direction system for compact heightfield connections.
///
/// Recast uses a Y-up coordinate system where the ground plane is XZ.
/// The C++ code uses numbered directions 0-3 with axis-label comments.
///
/// ## 4-direction system (`con[4]` array, matches C++ `rcGetCon`)
///
/// ```text
///   dir 0 = -X   offset (-1,  0)
///   dir 1 = +Z   offset ( 0, +1)
///   dir 2 = +X   offset (+1,  0)
///   dir 3 = -Z   offset ( 0, -1)
/// ```
///
/// Offsets are returned by [`get_dir_offset_x`](CompactHeightfield::get_dir_offset_x)
/// and [`get_dir_offset_z`](CompactHeightfield::get_dir_offset_z)
/// (C++ `rcGetDirOffsetX` / `rcGetDirOffsetY`).
///
/// Rotation: `(dir + 1) & 3` = clockwise, `(dir + 3) & 3` = counter-clockwise.

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
    pub(crate) width: i32,
    /// Height (depth) of the heightfield along the z-axis
    pub(crate) height: i32,

    /// The minimum bounds of the heightfield's AABB
    pub(crate) bmin: Vec3,
    /// The maximum bounds of the heightfield's AABB
    pub(crate) bmax: Vec3,

    /// Cell size (horizontal resolution)
    pub(crate) cs: f32,
    /// Cell height (vertical resolution)
    pub(crate) ch: f32,

    /// Grid of compact cells
    pub(crate) cells: Vec<CompactCell>,
    /// Array of compact spans
    pub(crate) spans: Vec<CompactSpan>,
    /// Reverse mapping: span_idx → cell_idx (for resolving neighbors)
    pub(crate) span_cells: Vec<u32>,
    /// Array of area IDs for each span
    pub(crate) areas: Vec<u8>,
    /// Array of distance values per span (for watershed)
    pub(crate) dist: Vec<u16>,

    /// Number of spans that are walkable
    pub(crate) walkable_span_count: usize,
    /// Number of spans that have been assigned to an area
    pub(crate) walkable_area_count: usize,
    /// Cell count (width * height)
    pub(crate) cell_count: usize,
    /// Span count (total number of spans)
    pub(crate) span_count: usize,
    /// Maximum height in the heightfield
    pub(crate) max_height: u16,
    /// Maximum distance value in distance field
    pub(crate) max_distance: u16,
    /// Maximum region id
    pub(crate) max_regions: u16,
}

#[allow(dead_code)]
impl CompactHeightfield {
    pub fn width(&self) -> i32 {
        self.width
    }

    pub fn height(&self) -> i32 {
        self.height
    }

    pub fn bmin(&self) -> Vec3 {
        self.bmin
    }

    pub fn bmax(&self) -> Vec3 {
        self.bmax
    }

    /// Cell size in the xz-plane (horizontal voxel resolution).
    pub fn cs(&self) -> f32 {
        self.cs
    }

    /// Cell height along the y-axis (vertical voxel resolution).
    pub fn ch(&self) -> f32 {
        self.ch
    }

    pub fn cells(&self) -> &[CompactCell] {
        &self.cells
    }

    pub fn cells_mut(&mut self) -> &mut [CompactCell] {
        &mut self.cells
    }

    pub fn spans(&self) -> &[CompactSpan] {
        &self.spans
    }

    pub fn spans_mut(&mut self) -> &mut [CompactSpan] {
        &mut self.spans
    }

    pub fn span_cells(&self) -> &[u32] {
        &self.span_cells
    }

    pub fn areas(&self) -> &[u8] {
        &self.areas
    }

    /// Distance field values per span, computed by `build_distance_field`.
    pub fn dist(&self) -> &[u16] {
        &self.dist
    }

    pub fn walkable_span_count(&self) -> usize {
        self.walkable_span_count
    }

    pub fn walkable_area_count(&self) -> usize {
        self.walkable_area_count
    }

    pub fn cell_count(&self) -> usize {
        self.cell_count
    }

    /// Returns the total span count.
    pub fn span_count(&self) -> usize {
        self.span_count
    }

    /// Returns the maximum height in the heightfield.
    pub fn max_height(&self) -> u16 {
        self.max_height
    }

    /// Returns the maximum distance value.
    pub fn max_distance(&self) -> u16 {
        self.max_distance
    }

    /// Returns the maximum region ID.
    pub fn max_regions(&self) -> u16 {
        self.max_regions
    }

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
        for z in 0..height {
            for x in 0..width {
                let mut si = heightfield.column_first_span(x, z);
                while si != SPAN_NULL {
                    let s = heightfield.span(si);
                    if s.area != 0 {
                        span_count += 1;
                    }
                    si = s.next;
                }
            }
        }

        // Allocate memory for the compact heightfield
        let mut cells = Vec::with_capacity((width * height) as usize);
        let mut spans = Vec::with_capacity(span_count);
        let mut span_cells = Vec::with_capacity(span_count);
        let mut areas = Vec::with_capacity(span_count);

        // Build the compact heightfield
        let mut walkable_span_count = 0;
        let mut max_height = 0u16;

        for z in 0..height {
            for x in 0..width {
                let first_si = heightfield.column_first_span(x, z);

                // If the column has no spans, add an empty cell
                if first_si == SPAN_NULL {
                    cells.push(CompactCell::new(None, 0));
                    continue;
                }

                // Count walkable spans in this cell
                let mut walkable_count = 0;
                let mut si = first_si;
                while si != SPAN_NULL {
                    let s = heightfield.span(si);
                    if s.area != 0 {
                        walkable_count += 1;
                    }
                    si = s.next;
                }

                if walkable_count == 0 {
                    cells.push(CompactCell::new(None, 0));
                    continue;
                }

                // Add the cell
                let cell_idx = cells.len() as u32;
                cells.push(CompactCell::new(Some(spans.len()), walkable_count));

                // Add only walkable spans (matching C++ rcBuildCompactHeightfield)
                si = first_si;
                while si != SPAN_NULL {
                    let s = heightfield.span(si);

                    if s.area != 0 {
                        // C++ convention: y = smax (top of solid = walkable surface)
                        let bot = s.max as i32;
                        // Air gap: from top of this solid to bottom of next solid above
                        let top = if s.next != SPAN_NULL {
                            heightfield.span(s.next).min as i32
                        } else {
                            MAX_HEIGHT
                        };
                        let air_height = (top - bot).clamp(0, 255);

                        spans.push(CompactSpan {
                            min: s.min,
                            max: s.max,
                            area: s.area,
                            reg: 0,
                            y: bot,
                            h: air_height as u8,
                            con: [RC_NOT_CONNECTED; 4],
                        });
                        span_cells.push(cell_idx);
                        areas.push(s.area);

                        // Track max height
                        max_height = max_height.max(s.max as u16);
                        walkable_span_count += 1;
                    }

                    si = s.next;
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
            span_cells,
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
    /// Builds connections between spans.
    ///
    /// Matches C++ `rcBuildCompactHeightfield` connection logic:
    /// - Only 4 cardinal directions (W=0, S=1, E=2, N=3)
    /// - Air space overlap must be >= walkable_height
    /// - Step height between walkable surfaces must be <= walkable_climb
    /// - Takes first valid connection per direction (not best)
    /// - Stores layer index in `con[4]`; resolved via `get_neighbor` or
    ///   `get_neighbor_connection`
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

        const MAX_LAYERS: u8 = RC_NOT_CONNECTED - 1;

        let num_cells = (width * height) as usize;

        for cell_idx in 0..num_cells {
            let cell = &self.cells[cell_idx];

            if let Some(first_span_idx) = cell.index {
                let x = (cell_idx as i32) % width;
                let z = (cell_idx as i32) / width;

                for s in 0..cell.count {
                    let span_idx = first_span_idx + s;

                    if self.areas[span_idx] == 0 {
                        continue;
                    }

                    let span_y = self.spans[span_idx].y;
                    let span_h = self.spans[span_idx].h as i32;

                    for dir in 0..4usize {
                        self.spans[span_idx].con[dir] = RC_NOT_CONNECTED;

                        let nx = x + dir_offset_x[dir];
                        let nz = z + dir_offset_z[dir];

                        if nx < 0 || nz < 0 || nx >= width || nz >= height {
                            continue;
                        }

                        let neighbor_cell_idx = (nz * width + nx) as usize;
                        let neighbor_cell = &self.cells[neighbor_cell_idx];

                        if let Some(first_neighbor_idx) = neighbor_cell.index {
                            for ns in 0..neighbor_cell.count {
                                let neighbor_idx = first_neighbor_idx + ns;
                                let neighbor_span = &self.spans[neighbor_idx];

                                let bot = span_y.max(neighbor_span.y);
                                let top =
                                    (span_y + span_h).min(neighbor_span.y + neighbor_span.h as i32);

                                if (top - bot) >= walkable_height
                                    && (neighbor_span.y - span_y).abs() <= walkable_climb
                                {
                                    if ns > MAX_LAYERS as usize {
                                        continue;
                                    }
                                    self.spans[span_idx].con[dir] = ns as u8;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
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

    /// Gets the neighboring span in a specific 8-direction.
    ///
    /// Maps 8-dir to 4-dir internally (only cardinal dirs 1,3,5,7 are valid).
    /// Returns `None` if direction is diagonal or not connected.
    pub fn get_neighbor(&self, span_idx: usize, dir: u8) -> Option<usize> {
        // Map 8-dir to 4-dir: 7→0(-X), 5→1(+Z), 3→2(+X), 1→3(-Z)
        let dir4 = match dir {
            7 => 0,
            5 => 1,
            3 => 2,
            1 => 3,
            _ => return None, // Diagonal dirs not supported
        };
        self.get_con(span_idx, dir4)
    }

    /// Gets the neighboring span using the 4-direction `con[4]` array directly.
    ///
    /// This is the primary neighbor lookup, matching C++ `rcGetCon`. Returns
    /// the absolute span index of the neighbor, or `None` if not connected.
    ///
    /// Recovers (x,z) from `span_cells` via integer division. For hot loops
    /// that already have (x,z) in scope, use [`get_con_xy`](Self::get_con_xy)
    /// to avoid the modular arithmetic.
    pub fn get_con(&self, span_idx: usize, dir: usize) -> Option<usize> {
        let layer = self.spans[span_idx].con[dir];
        if layer == RC_NOT_CONNECTED {
            return None;
        }
        let cell_idx = self.span_cells[span_idx] as usize;
        let x = (cell_idx % self.width as usize) as i32;
        let z = (cell_idx / self.width as usize) as i32;
        let nx = x + Self::dir_offset_x(dir);
        let nz = z + Self::dir_offset_z(dir);
        let ncell = &self.cells[(nz * self.width + nx) as usize];
        ncell.index.map(|idx| idx + layer as usize)
    }

    /// Gets the neighboring span when (x,z) are already known.
    ///
    /// Avoids the `% width` / `/ width` integer division in [`get_con`](Self::get_con).
    /// Use this in hot loops that iterate `for z in 0..h { for x in 0..w { ... } }`.
    #[inline(always)]
    pub fn get_con_xy(&self, span_idx: usize, dir: usize, x: i32, z: i32) -> Option<usize> {
        let layer = self.spans[span_idx].con[dir];
        if layer == RC_NOT_CONNECTED {
            return None;
        }
        let nx = x + Self::dir_offset_x(dir);
        let nz = z + Self::dir_offset_z(dir);
        let ncell = &self.cells[(nz * self.width + nx) as usize];
        ncell.index.map(|idx| idx + layer as usize)
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
    /// Uses the 4-dir `con[4]` array for propagation, with diagonal
    /// checks via two consecutive cardinal hops.
    fn calculate_distance_field(&self, src: &mut [u16]) -> Result<u16, BuildError> {
        let w = self.width;
        let h = self.height;

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
                        for dir in 0..4 {
                            if let Some(neighbor_idx) = self.get_con_xy(span_idx, dir, x, y) {
                                if self.areas[neighbor_idx] == self.areas[span_idx] {
                                    neighbor_count += 1;
                                }
                            }
                        }

                        if neighbor_count != 4 {
                            src[span_idx] = 0;
                        }
                    }
                }
            }
        }

        // Pass 1: Forward pass (top-left to bottom-right)
        // C++ checks dir 0 (-X) and dir 3 (-Z), then diagonals (-X,-Z) and (-Z,+X)
        for y in 0..h {
            for x in 0..w {
                let cell_idx = (y * w + x) as usize;
                let cell = &self.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let span_idx = first_span_idx + s;

                        // dir 0 = -X (cardinal: +2)
                        if let Some(neg_x_idx) = self.get_con_xy(span_idx, 0, x, y) {
                            if src[neg_x_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[neg_x_idx].saturating_add(2);
                            }
                            // (-X,-Z) diagonal: from -X neighbor, go dir 3 (-Z)
                            // Neighbor is at (x-1, y), so pass those coords
                            if let Some(diag_idx) = self.get_con_xy(neg_x_idx, 3, x - 1, y) {
                                if src[diag_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[diag_idx].saturating_add(3);
                                }
                            }
                        }

                        // dir 3 = -Z (cardinal: +2)
                        if let Some(neg_z_idx) = self.get_con_xy(span_idx, 3, x, y) {
                            if src[neg_z_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[neg_z_idx].saturating_add(2);
                            }
                            // (-Z,+X) diagonal: from -Z neighbor, go dir 2 (+X)
                            // Neighbor is at (x, y-1), so pass those coords
                            if let Some(diag_idx) = self.get_con_xy(neg_z_idx, 2, x, y - 1) {
                                if src[diag_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[diag_idx].saturating_add(3);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Pass 2: Backward pass (bottom-right to top-left)
        // C++ checks dir 2 (+X) and dir 1 (+Z), then diagonals (+X,+Z) and (+Z,-X)
        for y in (0..h).rev() {
            for x in (0..w).rev() {
                let cell_idx = (y * w + x) as usize;
                let cell = &self.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let span_idx = first_span_idx + s;

                        // dir 2 = +X (cardinal: +2)
                        if let Some(pos_x_idx) = self.get_con_xy(span_idx, 2, x, y) {
                            if src[pos_x_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[pos_x_idx].saturating_add(2);
                            }
                            // (+X,+Z) diagonal: from +X neighbor, go dir 1 (+Z)
                            // Neighbor is at (x+1, y), so pass those coords
                            if let Some(diag_idx) = self.get_con_xy(pos_x_idx, 1, x + 1, y) {
                                if src[diag_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[diag_idx].saturating_add(3);
                                }
                            }
                        }

                        // dir 1 = +Z (cardinal: +2)
                        if let Some(pos_z_idx) = self.get_con_xy(span_idx, 1, x, y) {
                            if src[pos_z_idx].saturating_add(2) < src[span_idx] {
                                src[span_idx] = src[pos_z_idx].saturating_add(2);
                            }
                            // (+Z,-X) diagonal: from +Z neighbor, go dir 0 (-X)
                            // Neighbor is at (x, y+1), so pass those coords
                            if let Some(diag_idx) = self.get_con_xy(pos_z_idx, 0, x, y + 1) {
                                if src[diag_idx].saturating_add(3) < src[span_idx] {
                                    src[span_idx] = src[diag_idx].saturating_add(3);
                                }
                            }
                        }
                    }
                }
            }
        }

        let max_dist = src.iter().copied().max().unwrap_or(0);
        Ok(max_dist)
    }

    /// Applies box blur to smooth the distance field.
    /// Returns true if result is in dst, false if result is in src.
    ///
    /// Matches C++ `boxBlur`: always uses a 3x3 kernel (divides by 9),
    /// padding missing neighbors with the center value.
    fn box_blur(
        &self,
        threshold: u16,
        src: &mut [u16],
        dst: &mut [u16],
    ) -> Result<bool, BuildError> {
        let w = self.width;
        let h = self.height;
        let thr = threshold * 2;

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

                        // Check 4 cardinal + 4 diagonal neighbors (C++ 3x3 kernel)
                        // For each dir, diagonal is reached by going (dir+1)&3 from
                        // the cardinal neighbor.
                        for dir in 0..4usize {
                            if let Some(neighbor_idx) = self.get_con_xy(span_idx, dir, x, y) {
                                d += src[neighbor_idx] as i32;

                                let next_dir = (dir + 1) & 3;
                                let nx = x + Self::dir_offset_x(dir);
                                let nz = y + Self::dir_offset_z(dir);
                                if let Some(diag_idx) =
                                    self.get_con_xy(neighbor_idx, next_dir, nx, nz)
                                {
                                    d += src[diag_idx] as i32;
                                } else {
                                    d += cd as i32;
                                }
                            } else {
                                d += cd as i32 * 2;
                            }
                        }

                        dst[span_idx] = ((d + 5) / 9) as u16;
                    }
                }
            }
        }

        Ok(true) // Result is in dst
    }

    /// Gets the neighbor span for a 4-direction index (0-3).
    ///
    /// Equivalent to `get_con` but kept for API compatibility.
    pub fn get_neighbor_connection(&self, span_idx: usize, direction: usize) -> Option<usize> {
        if direction >= 4 {
            return None;
        }
        self.get_con(span_idx, direction)
    }

    /// Gets the X-axis offset for a 4-direction index.
    ///
    /// Matches C++ `rcGetDirOffsetX`. See [`get_dir_offset_z`](Self::get_dir_offset_z).
    pub fn get_dir_offset_x(&self, direction: usize) -> i32 {
        Self::dir_offset_x(direction)
    }

    /// Gets the Z-axis offset for a 4-direction index.
    ///
    /// C++ names this `rcGetDirOffsetY` because it returns the grid's second
    /// axis, but the grid lies on the XZ plane so the offset is along Z.
    pub fn get_dir_offset_z(&self, direction: usize) -> i32 {
        Self::dir_offset_z(direction)
    }

    /// Static X-axis offset for a 4-direction index. Avoids &self borrow.
    fn dir_offset_x(direction: usize) -> i32 {
        const OFFSETS: [i32; 4] = [-1, 0, 1, 0];
        OFFSETS[direction]
    }

    /// Static Z-axis offset for a 4-direction index. Avoids &self borrow.
    fn dir_offset_z(direction: usize) -> i32 {
        const OFFSETS: [i32; 4] = [0, 1, 0, -1];
        OFFSETS[direction]
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
            for dir in 0..4 {
                if let Some(neighbor_idx) = self.get_con(span_idx, dir) {
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
        let mut neighbor_borders = vec![0i32; region_areas.len()];

        // Count border lengths with each neighbor
        for (span_idx, &span_region) in region_ids.iter().enumerate() {
            if (span_region & !RC_BORDER_REG) != region_id {
                continue;
            }

            for dir in 0..4 {
                if let Some(neighbor_idx) = self.get_con(span_idx, dir) {
                    let neighbor_region = region_ids[neighbor_idx] & !RC_BORDER_REG;

                    if neighbor_region != region_id && neighbor_region != 0 {
                        neighbor_borders[neighbor_region as usize] += 1;
                    }
                }
            }
        }

        // Find neighbor with best score (longest border * region area)
        let mut best_neighbor = None;
        let mut best_score = 0i32;

        for (neighbor_region, &border_length) in neighbor_borders.iter().enumerate() {
            if border_length == 0 {
                continue;
            }
            let neighbor_area = region_areas.get(neighbor_region).copied().unwrap_or(0);
            let score = border_length * (neighbor_area + 1); // +1 to avoid zero

            if score > best_score {
                best_score = score;
                best_neighbor = Some(neighbor_region as u16);
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
        // Verify spans have connections (at least one con[] != RC_NOT_CONNECTED)
        assert!(
            chf.spans
                .iter()
                .any(|s| s.con.iter().any(|&c| c != RC_NOT_CONNECTED))
        );
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
        let neighbor_idx = chf.get_con(span0_idx, 2); // dir 2 = +X (east)
        assert!(neighbor_idx.is_some());
        let neighbor_idx = neighbor_idx.unwrap();
        assert_eq!(neighbor_idx, 1); // Should be the span at (1, 0)
    }
}
