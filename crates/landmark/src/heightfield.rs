//! Heightfield representation for Recast
//!
//! The heightfield is the first data structure in the Recast pipeline.
//! It's a 2D grid of height spans that represents a voxelized 3D environment.
//!
//! Spans are stored in a flat arena (`Vec<SpanEntry>`) indexed by `SpanIndex`.
//! Each grid cell has a `columns` entry pointing to the first span in that
//! column (or `SPAN_NULL` if empty). This replaces the previous
//! `HashMap<(i32,i32), Option<Rc<RefCell<Span>>>>` for O(1) hashing-free
//! lookups and cache-friendly iteration.

use glam::Vec3;

use crate::error::BuildError;

/// Index into the span arena. `SPAN_NULL` means no span.
pub type SpanIndex = u32;

/// Sentinel value indicating "no span" (analogous to a null pointer).
pub const SPAN_NULL: SpanIndex = u32::MAX;

/// Represents cell bounds in XZ plane
struct CellBounds {
    min_x: f32,
    max_x: f32,
    min_z: f32,
    max_z: f32,
}

/// A span in the heightfield, representing a vertical segment of space.
///
/// Stored in the `Heightfield`'s arena. Linked via `next` indices.
#[derive(Debug, Clone, Copy)]
pub struct SpanEntry {
    /// The minimum height of the span
    pub min: i16,
    /// The maximum height of the span
    pub max: i16,
    /// Area ID (0 = not walkable)
    pub area: u8,
    /// Next span in the column, or `SPAN_NULL` if this is the last span
    pub next: SpanIndex,
}

/// Iterator over spans in a single column.
pub struct ColumnSpanIter<'a> {
    arena: &'a [SpanEntry],
    current: SpanIndex,
}

impl<'a> Iterator for ColumnSpanIter<'a> {
    type Item = (SpanIndex, &'a SpanEntry);

    fn next(&mut self) -> Option<Self::Item> {
        if self.current == SPAN_NULL {
            return None;
        }
        let idx = self.current;
        let entry = &self.arena[idx as usize];
        self.current = entry.next;
        Some((idx, entry))
    }
}

/// Heightfield structure holding a grid of span columns.
///
/// Uses a flat `columns` vec indexed by `x + z * width` and an arena-allocated
/// span storage for O(1) lookups without hashing.
#[derive(Debug)]
pub struct Heightfield {
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

    /// One entry per grid cell (width * height). Value is the index of the
    /// first span in that column, or `SPAN_NULL` if the column is empty.
    pub(crate) columns: Vec<SpanIndex>,

    /// Arena of all spans. Grows as spans are added. Freed spans are recycled
    /// via `free_list`.
    pub(crate) spans: Vec<SpanEntry>,

    /// Head of the freelist for recycled span slots.
    free_list: SpanIndex,
}

impl Heightfield {
    /// Creates a new empty heightfield
    pub fn new(width: i32, height: i32, bmin: Vec3, bmax: Vec3, cs: f32, ch: f32) -> Self {
        let num_cells = (width * height) as usize;
        Self {
            width,
            height,
            bmin,
            bmax,
            cs,
            ch,
            columns: vec![SPAN_NULL; num_cells],
            spans: Vec::new(),
            free_list: SPAN_NULL,
        }
    }

    // ── Column index helpers ──────────────────────────────────────────

    /// Converts (x, z) to the flat column index.
    #[inline]
    fn col_index(&self, x: i32, z: i32) -> usize {
        (x + z * self.width) as usize
    }

    // ── Arena allocation ──────────────────────────────────────────────

    /// Allocates a span in the arena (reusing a freelist slot if available).
    pub(crate) fn alloc_span(&mut self, entry: SpanEntry) -> SpanIndex {
        if self.free_list != SPAN_NULL {
            let idx = self.free_list;
            self.free_list = self.spans[idx as usize].next;
            self.spans[idx as usize] = entry;
            idx
        } else {
            let idx = self.spans.len() as SpanIndex;
            self.spans.push(entry);
            idx
        }
    }

    /// Returns a span slot to the freelist for reuse.
    pub(crate) fn free_span(&mut self, idx: SpanIndex) {
        self.spans[idx as usize].next = self.free_list;
        self.free_list = idx;
    }

    // ── Public accessors ──────────────────────────────────────────────

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

    /// Gets the first span index for column (x, z), or `SPAN_NULL` if empty.
    #[inline]
    pub fn column_first_span(&self, x: i32, z: i32) -> SpanIndex {
        self.columns[self.col_index(x, z)]
    }

    /// Gets a span by index.
    #[inline]
    pub fn span(&self, index: SpanIndex) -> &SpanEntry {
        &self.spans[index as usize]
    }

    /// Gets a mutable reference to a span by index.
    #[inline]
    pub fn span_mut(&mut self, index: SpanIndex) -> &mut SpanEntry {
        &mut self.spans[index as usize]
    }

    /// Iterates all spans in a column.
    pub fn column_spans(&self, x: i32, z: i32) -> ColumnSpanIter<'_> {
        ColumnSpanIter {
            arena: &self.spans,
            current: self.columns[self.col_index(x, z)],
        }
    }

    /// Access the full columns slice (for bulk iteration).
    pub fn columns(&self) -> &[SpanIndex] {
        &self.columns
    }

    /// Access the full spans arena (for bulk iteration).
    pub fn spans_arena(&self) -> &[SpanEntry] {
        &self.spans
    }

    // ── Span insertion ────────────────────────────────────────────────

    /// Add a span to the heightfield
    pub fn add_span(
        &mut self,
        x: i32,
        z: i32,
        min: i16,
        max: i16,
        area: u8,
    ) -> Result<(), BuildError> {
        if x < 0 || x >= self.width || z < 0 || z >= self.height {
            return Err(BuildError::SpanOutOfBounds { x, y: z });
        }

        if min > max {
            return Err(BuildError::InvalidSpanHeight {
                min: min as u32,
                max: max as u32,
            });
        }

        let col_idx = self.col_index(x, z);
        let new_idx = self.alloc_span(SpanEntry {
            min,
            max,
            area,
            next: SPAN_NULL,
        });

        let first = self.columns[col_idx];
        if first == SPAN_NULL {
            self.columns[col_idx] = new_idx;
            return Ok(());
        }

        // Walk the column to find insertion point (sorted by min height)
        let mut prev = SPAN_NULL;
        let mut cur = first;

        while cur != SPAN_NULL {
            let cur_span = self.spans[cur as usize];

            // Check for overlap with same area - merge if so
            if cur_span.area == area && cur_span.max >= min && max >= cur_span.min {
                let merged_min = cur_span.min.min(min);
                let merged_max = cur_span.max.max(max);
                self.spans[cur as usize].min = merged_min;
                self.spans[cur as usize].max = merged_max;

                // Free the new span since we merged into cur
                self.free_span(new_idx);

                // Check if we can merge with the next span too
                let next = self.spans[cur as usize].next;
                if next != SPAN_NULL {
                    let next_span = self.spans[next as usize];
                    if next_span.area == area && merged_max >= next_span.min {
                        let final_max = merged_max.max(next_span.max);
                        let next_next = next_span.next;
                        self.spans[cur as usize].max = final_max;
                        self.spans[cur as usize].next = next_next;
                        self.free_span(next);
                    }
                }

                return Ok(());
            }

            // If new span is below current span, insert before
            if max < cur_span.min {
                self.spans[new_idx as usize].next = cur;
                if prev == SPAN_NULL {
                    self.columns[col_idx] = new_idx;
                } else {
                    self.spans[prev as usize].next = new_idx;
                }
                return Ok(());
            }

            prev = cur;
            cur = cur_span.next;
        }

        // Add at end
        self.spans[new_idx as usize].next = SPAN_NULL;
        if prev != SPAN_NULL {
            self.spans[prev as usize].next = new_idx;
        } else {
            self.columns[col_idx] = new_idx;
        }

        Ok(())
    }

    /// Get the height range of the heightfield
    pub fn get_height_range(&self) -> (i16, i16) {
        let mut min_height = i16::MAX;
        let mut max_height = i16::MIN;

        for &entry in &self.spans {
            min_height = min_height.min(entry.min);
            max_height = max_height.max(entry.max);
        }

        (min_height, max_height)
    }

    /// Filters spans in the heightfield based on walkable criteria
    pub fn filter_walkable(
        &mut self,
        walkable_height: i16,
        walkable_climb: i16,
    ) -> Result<(), BuildError> {
        for z in 0..self.height {
            for x in 0..self.width {
                let mut si = self.columns[self.col_index(x, z)];
                while si != SPAN_NULL {
                    let span = self.spans[si as usize];
                    let span_height = span.max - span.min;

                    if span_height < walkable_height {
                        self.spans[si as usize].area = 0;
                    }

                    if span.next != SPAN_NULL {
                        let next = self.spans[span.next as usize];
                        let climb = next.min as i32 - span.max as i32;
                        if climb > walkable_climb as i32 {
                            self.spans[si as usize].area = 0;
                        }
                    }

                    si = span.next;
                }
            }
        }
        Ok(())
    }

    /// Filters low hanging walkable obstacles that the agent can step over.
    /// Implements the C++ rcFilterLowHangingWalkableObstacles algorithm.
    pub fn filter_low_hanging_walkable_obstacles(
        &mut self,
        walkable_climb: i16,
    ) -> Result<(), BuildError> {
        for z in 0..self.height {
            for x in 0..self.width {
                let mut si = self.columns[self.col_index(x, z)];
                let mut previous_walkable = false;
                let mut previous_area = 0u8;
                let mut previous_max = 0i16;

                while si != SPAN_NULL {
                    let span = self.spans[si as usize];
                    let walkable = span.area != 0;

                    if !walkable && previous_walkable {
                        if (span.max as i32 - previous_max as i32) <= walkable_climb as i32 {
                            self.spans[si as usize].area = previous_area;
                        }
                    }

                    previous_walkable = walkable;
                    previous_area = self.spans[si as usize].area;
                    previous_max = span.max;

                    si = span.next;
                }
            }
        }
        Ok(())
    }

    /// Filters ledge spans - marks spans adjacent to ledges as unwalkable.
    /// Implements the C++ rcFilterLedgeSpans algorithm.
    pub fn filter_ledge_spans(
        &mut self,
        walkable_height: i16,
        walkable_climb: i16,
    ) -> Result<(), BuildError> {
        const MAX_HEIGHTFIELD_HEIGHT: i32 = 0xffff;
        let dir_offsets = [(0, -1), (1, 0), (0, 1), (-1, 0)];

        for z in 0..self.height {
            for x in 0..self.width {
                let mut si = self.columns[self.col_index(x, z)];

                while si != SPAN_NULL {
                    let span = self.spans[si as usize];

                    if span.area == 0 {
                        si = span.next;
                        continue;
                    }

                    let floor = span.max as i32;
                    let ceiling = if span.next != SPAN_NULL {
                        self.spans[span.next as usize].min as i32
                    } else {
                        MAX_HEIGHTFIELD_HEIGHT
                    };

                    let mut lowest_neighbor_floor_diff = MAX_HEIGHTFIELD_HEIGHT;
                    let mut lowest_traversable_floor = span.max as i32;
                    let mut highest_traversable_floor = span.max as i32;
                    let mut is_ledge = false;

                    for &(dx, dz) in &dir_offsets {
                        let nx = x + dx;
                        let nz = z + dz;

                        if nx < 0 || nz < 0 || nx >= self.width || nz >= self.height {
                            lowest_neighbor_floor_diff = -walkable_climb as i32 - 1;
                            is_ledge = true;
                            break;
                        }

                        let mut nsi = self.columns[self.col_index(nx, nz)];

                        let mut neighbor_ceiling = if nsi != SPAN_NULL {
                            self.spans[nsi as usize].min as i32
                        } else {
                            MAX_HEIGHTFIELD_HEIGHT
                        };

                        if (ceiling.min(neighbor_ceiling) - floor) >= walkable_height as i32 {
                            lowest_neighbor_floor_diff = -walkable_climb as i32 - 1;
                            is_ledge = true;
                            break;
                        }

                        while nsi != SPAN_NULL {
                            let ns = self.spans[nsi as usize];
                            let neighbor_floor = ns.max as i32;
                            neighbor_ceiling = if ns.next != SPAN_NULL {
                                self.spans[ns.next as usize].min as i32
                            } else {
                                MAX_HEIGHTFIELD_HEIGHT
                            };

                            if (ceiling.min(neighbor_ceiling) - floor.max(neighbor_floor))
                                < walkable_height as i32
                            {
                                nsi = ns.next;
                                continue;
                            }

                            let neighbor_floor_diff = neighbor_floor - floor;
                            lowest_neighbor_floor_diff =
                                lowest_neighbor_floor_diff.min(neighbor_floor_diff);

                            if neighbor_floor_diff.abs() <= walkable_climb as i32 {
                                lowest_traversable_floor =
                                    lowest_traversable_floor.min(neighbor_floor);
                                highest_traversable_floor =
                                    highest_traversable_floor.max(neighbor_floor);
                            } else if neighbor_floor_diff < -walkable_climb as i32 {
                                break;
                            }

                            nsi = ns.next;
                        }
                    }

                    if is_ledge
                        || lowest_neighbor_floor_diff < -walkable_climb as i32
                        || (highest_traversable_floor - lowest_traversable_floor)
                            > walkable_climb as i32
                    {
                        self.spans[si as usize].area = 0;
                    }

                    si = span.next;
                }
            }
        }
        Ok(())
    }

    /// Filters walkable spans that have insufficient clearance above them.
    /// Implements the C++ rcFilterWalkableLowHeightSpans algorithm.
    pub fn filter_walkable_low_height_spans(
        &mut self,
        walkable_height: i16,
    ) -> Result<(), BuildError> {
        const MAX_HEIGHTFIELD_HEIGHT: i32 = 0xffff;

        for z in 0..self.height {
            for x in 0..self.width {
                let mut si = self.columns[self.col_index(x, z)];

                while si != SPAN_NULL {
                    let span = self.spans[si as usize];
                    let floor = span.max as i32;
                    let ceiling = if span.next != SPAN_NULL {
                        self.spans[span.next as usize].min as i32
                    } else {
                        MAX_HEIGHTFIELD_HEIGHT
                    };

                    if (ceiling - floor) < walkable_height as i32 {
                        self.spans[si as usize].area = 0;
                    }

                    si = span.next;
                }
            }
        }
        Ok(())
    }

    /// Marks border spans (spans at the edge of the heightfield)
    #[allow(dead_code)]
    fn mark_border_spans(&mut self) -> Result<(), BuildError> {
        for z in 0..self.height {
            for x in 0..self.width {
                if x == 0 || x == self.width - 1 || z == 0 || z == self.height - 1 {
                    let mut si = self.columns[self.col_index(x, z)];
                    while si != SPAN_NULL {
                        self.spans[si as usize].area = 0;
                        si = self.spans[si as usize].next;
                    }
                }
            }
        }
        Ok(())
    }

    /// Applies median filter to remove noise in walkable areas
    pub fn median_filter_walkable_area(&mut self) -> Result<(), BuildError> {
        let width = self.width;
        let height = self.height;

        // Create a copy of area values for reading, indexed by column
        let num_cols = (width * height) as usize;
        let mut column_areas: Vec<Vec<u8>> = vec![Vec::new(); num_cols];

        for z in 0..height {
            for x in 0..width {
                let mut si = self.columns[self.col_index(x, z)];
                let col_idx = (z * width + x) as usize;

                while si != SPAN_NULL {
                    column_areas[col_idx].push(self.spans[si as usize].area);
                    si = self.spans[si as usize].next;
                }
            }
        }

        // Apply median filter
        for z in 1..height - 1 {
            for x in 1..width - 1 {
                let mut si = self.columns[self.col_index(x, z)];
                let mut span_index = 0;

                while si != SPAN_NULL {
                    let mut neighbor_areas = Vec::new();

                    for dz in -1..=1 {
                        for dx in -1..=1 {
                            let nx = x + dx;
                            let nz = z + dz;
                            let col_idx = (nz * width + nx) as usize;
                            if let Some(&area) = column_areas[col_idx].get(span_index) {
                                neighbor_areas.push(area);
                            }
                        }
                    }

                    if neighbor_areas.len() >= 5 {
                        neighbor_areas.sort();
                        let median_area = neighbor_areas[neighbor_areas.len() / 2];
                        let current_area = self.spans[si as usize].area;
                        if current_area != median_area {
                            let median_count =
                                neighbor_areas.iter().filter(|&&a| a == median_area).count();
                            if median_count > neighbor_areas.len() / 2 {
                                self.spans[si as usize].area = median_area;
                            }
                        }
                    }

                    si = self.spans[si as usize].next;
                    span_index += 1;
                }
            }
        }

        Ok(())
    }

    /// Erodes the walkable area by the specified radius
    pub fn erode_walkable_area(&mut self, radius: i32) -> Result<(), BuildError> {
        if radius <= 0 {
            return Ok(());
        }

        let width = self.width;
        let height = self.height;

        // Build column-to-arena-index mapping for O(1) ordinal lookups
        let num_cols = (width * height) as usize;
        let mut column_spans: Vec<Vec<u32>> = vec![Vec::new(); num_cols];

        for z in 0..height {
            for x in 0..width {
                let mut si = self.columns[self.col_index(x, z)];
                let col_idx = (z * width + x) as usize;

                while si != SPAN_NULL {
                    column_spans[col_idx].push(si);
                    si = self.spans[si as usize].next;
                }
            }
        }

        // Find all non-walkable spans
        let mut non_walkable: Vec<(i32, i32, usize)> = Vec::new();

        for z in 0..height {
            for x in 0..width {
                let col_idx = (z * width + x) as usize;
                for (span_index, &si) in column_spans[col_idx].iter().enumerate() {
                    if self.spans[si as usize].area == 0 {
                        non_walkable.push((x, z, span_index));
                    }
                }
            }
        }

        // Erode walkable areas near non-walkable spans
        let mut to_erode = vec![false; self.spans.len()];

        for &(x, z, span_index) in &non_walkable {
            for dz in -radius..=radius {
                for dx in -radius..=radius {
                    let nx = x + dx;
                    let nz = z + dz;

                    if nx < 0 || nx >= width || nz < 0 || nz >= height {
                        continue;
                    }

                    let dist_sq = dx * dx + dz * dz;
                    if dist_sq <= radius * radius {
                        let col_idx = (nz * width + nx) as usize;
                        if let Some(&si) = column_spans[col_idx].get(span_index) {
                            to_erode[si as usize] = true;
                        }
                    }
                }
            }
        }

        // Apply erosion
        for (si, &erode) in to_erode.iter().enumerate() {
            if erode {
                self.spans[si].area = 0;
            }
        }

        Ok(())
    }

    /// Rasterizes a triangle into the heightfield
    pub fn rasterize_triangle(&mut self, verts: &[Vec3; 3], area: u8) -> Result<(), BuildError> {
        if verts.len() != 3 {
            return Err(BuildError::InvalidTriangle);
        }

        // Convert vertices to cell coordinates
        let mut c_verts = [Vec3::ZERO; 3];
        for i in 0..3 {
            c_verts[i].x = (verts[i].x - self.bmin.x) / self.cs;
            c_verts[i].y = (verts[i].y - self.bmin.y) / self.ch;
            c_verts[i].z = (verts[i].z - self.bmin.z) / self.cs;
        }

        // Calculate triangle bounds in cell coordinates
        let mut min_x = self.width;
        let mut min_z = self.height;
        let mut max_x = 0;
        let mut max_z = 0;

        for v in &c_verts {
            min_x = min_x.min(v.x as i32);
            min_z = min_z.min(v.z as i32);
            max_x = max_x.max(v.x as i32);
            max_z = max_z.max(v.z as i32);
        }

        // Clip bounds to heightfield dimensions
        min_x = min_x.max(0);
        min_z = min_z.max(0);
        max_x = max_x.min(self.width - 1);
        max_z = max_z.min(self.height - 1);

        // Rasterize the triangle using conservative rasterization
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                let bounds = CellBounds {
                    min_x: self.bmin.x + (x as f32) * self.cs,
                    max_x: self.bmin.x + ((x + 1) as f32) * self.cs,
                    min_z: self.bmin.z + (z as f32) * self.cs,
                    max_z: self.bmin.z + ((z + 1) as f32) * self.cs,
                };

                if !self.triangle_overlaps_cell_xz(&verts[0], &verts[1], &verts[2], &bounds) {
                    continue;
                }

                let pt = Vec3::new(
                    self.bmin.x + (x as f32 + 0.5) * self.cs,
                    0.0,
                    self.bmin.z + (z as f32 + 0.5) * self.cs,
                );

                let y = self.interpolate_height(&pt, &verts[0], &verts[1], &verts[2]);
                let h = ((y - self.bmin.y) / self.ch) as i16;
                let span_height = 10;
                let max_height = h.saturating_add(span_height);
                self.add_span(x, z, h, max_height, area)?;
            }
        }

        Ok(())
    }

    /// Checks if a point is inside a triangle (XZ plane projection)
    fn point_in_triangle_xz(&self, p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> bool {
        let v0x = c.x - a.x;
        let v0z = c.z - a.z;
        let v1x = b.x - a.x;
        let v1z = b.z - a.z;
        let v2x = p.x - a.x;
        let v2z = p.z - a.z;

        let dot00 = v0x * v0x + v0z * v0z;
        let dot01 = v0x * v1x + v0z * v1z;
        let dot02 = v0x * v2x + v0z * v2z;
        let dot11 = v1x * v1x + v1z * v1z;
        let dot12 = v1x * v2x + v1z * v2z;

        let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0)
    }

    /// Checks if a triangle overlaps a cell in the XZ plane
    fn triangle_overlaps_cell_xz(
        &self,
        v0: &Vec3,
        v1: &Vec3,
        v2: &Vec3,
        bounds: &CellBounds,
    ) -> bool {
        let tri_min_x = v0.x.min(v1.x).min(v2.x);
        let tri_max_x = v0.x.max(v1.x).max(v2.x);
        let tri_min_z = v0.z.min(v1.z).min(v2.z);
        let tri_max_z = v0.z.max(v1.z).max(v2.z);

        if tri_max_x < bounds.min_x
            || tri_min_x > bounds.max_x
            || tri_max_z < bounds.min_z
            || tri_min_z > bounds.max_z
        {
            return false;
        }

        let corners = [
            Vec3::new(bounds.min_x, 0.0, bounds.min_z),
            Vec3::new(bounds.max_x, 0.0, bounds.min_z),
            Vec3::new(bounds.max_x, 0.0, bounds.max_z),
            Vec3::new(bounds.min_x, 0.0, bounds.max_z),
        ];

        for corner in &corners {
            if self.point_in_triangle_xz(corner, v0, v1, v2) {
                return true;
            }
        }

        if v0.x >= bounds.min_x
            && v0.x <= bounds.max_x
            && v0.z >= bounds.min_z
            && v0.z <= bounds.max_z
        {
            return true;
        }
        if v1.x >= bounds.min_x
            && v1.x <= bounds.max_x
            && v1.z >= bounds.min_z
            && v1.z <= bounds.max_z
        {
            return true;
        }
        if v2.x >= bounds.min_x
            && v2.x <= bounds.max_x
            && v2.z >= bounds.min_z
            && v2.z <= bounds.max_z
        {
            return true;
        }

        if self.edge_intersects_cell_xz(v0, v1, bounds)
            || self.edge_intersects_cell_xz(v1, v2, bounds)
            || self.edge_intersects_cell_xz(v2, v0, bounds)
        {
            return true;
        }

        false
    }

    /// Checks if an edge intersects a cell in the XZ plane
    fn edge_intersects_cell_xz(&self, p0: &Vec3, p1: &Vec3, bounds: &CellBounds) -> bool {
        let dx = p1.x - p0.x;
        let dz = p1.z - p0.z;

        if dx.abs() > f32::EPSILON {
            let t = (bounds.min_x - p0.x) / dx;
            if (0.0..=1.0).contains(&t) {
                let z = p0.z + t * dz;
                if z >= bounds.min_z && z <= bounds.max_z {
                    return true;
                }
            }
        }

        if dx.abs() > f32::EPSILON {
            let t = (bounds.max_x - p0.x) / dx;
            if (0.0..=1.0).contains(&t) {
                let z = p0.z + t * dz;
                if z >= bounds.min_z && z <= bounds.max_z {
                    return true;
                }
            }
        }

        if dz.abs() > f32::EPSILON {
            let t = (bounds.min_z - p0.z) / dz;
            if (0.0..=1.0).contains(&t) {
                let x = p0.x + t * dx;
                if x >= bounds.min_x && x <= bounds.max_x {
                    return true;
                }
            }
        }

        if dz.abs() > f32::EPSILON {
            let t = (bounds.max_z - p0.z) / dz;
            if (0.0..=1.0).contains(&t) {
                let x = p0.x + t * dx;
                if x >= bounds.min_x && x <= bounds.max_x {
                    return true;
                }
            }
        }

        false
    }

    /// Interpolates the height at a point inside a triangle
    fn interpolate_height(&self, p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> f32 {
        let v0x = c.x - a.x;
        let v0z = c.z - a.z;
        let v1x = b.x - a.x;
        let v1z = b.z - a.z;
        let v2x = p.x - a.x;
        let v2z = p.z - a.z;

        let dot00 = v0x * v0x + v0z * v0z;
        let dot01 = v0x * v1x + v0z * v1z;
        let dot02 = v0x * v2x + v0z * v2z;
        let dot11 = v1x * v1x + v1z * v1z;
        let dot12 = v1x * v2x + v1z * v2z;

        let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
        let w = 1.0 - u - v;

        a.y * w + b.y * v + c.y * u
    }

    /// Marks spans within an axis-aligned box with the specified area ID
    pub fn mark_box_area(
        &mut self,
        bmin: &[f32; 3],
        bmax: &[f32; 3],
        area_id: u8,
    ) -> Result<(), BuildError> {
        let min_x = ((bmin[0] - self.bmin.x) / self.cs).floor() as i32;
        let min_z = ((bmin[2] - self.bmin.z) / self.cs).floor() as i32;
        let max_x = ((bmax[0] - self.bmin.x) / self.cs).ceil() as i32;
        let max_z = ((bmax[2] - self.bmin.z) / self.cs).ceil() as i32;

        let min_x = min_x.max(0);
        let min_z = min_z.max(0);
        let max_x = max_x.min(self.width - 1);
        let max_z = max_z.min(self.height - 1);

        let min_y = ((bmin[1] - self.bmin.y) / self.ch).floor() as i16;
        let max_y = ((bmax[1] - self.bmin.y) / self.ch).ceil() as i16;

        for z in min_z..=max_z {
            for x in min_x..=max_x {
                let mut si = self.columns[self.col_index(x, z)];
                while si != SPAN_NULL {
                    let span = &mut self.spans[si as usize];
                    if span.min <= max_y && span.max >= min_y {
                        span.area = area_id;
                    }
                    si = span.next;
                }
            }
        }
        Ok(())
    }

    /// Marks spans within a cylinder with the specified area ID
    pub fn mark_cylinder_area(
        &mut self,
        pos: &[f32; 3],
        radius: f32,
        height: f32,
        area_id: u8,
    ) -> Result<(), BuildError> {
        let min_x = ((pos[0] - radius - self.bmin.x) / self.cs).floor() as i32;
        let min_z = ((pos[2] - radius - self.bmin.z) / self.cs).floor() as i32;
        let max_x = ((pos[0] + radius - self.bmin.x) / self.cs).ceil() as i32;
        let max_z = ((pos[2] + radius - self.bmin.z) / self.cs).ceil() as i32;

        let min_x = min_x.max(0);
        let min_z = min_z.max(0);
        let max_x = max_x.min(self.width - 1);
        let max_z = max_z.min(self.height - 1);

        let min_y = ((pos[1] - self.bmin.y) / self.ch).floor() as i16;
        let max_y = ((pos[1] + height - self.bmin.y) / self.ch).ceil() as i16;

        let radius_sq = radius * radius;

        for z in min_z..=max_z {
            for x in min_x..=max_x {
                let cell_x = self.bmin.x + (x as f32 + 0.5) * self.cs;
                let cell_z = self.bmin.z + (z as f32 + 0.5) * self.cs;
                let dx = cell_x - pos[0];
                let dz = cell_z - pos[2];
                let dist_sq = dx * dx + dz * dz;

                if dist_sq <= radius_sq {
                    let mut si = self.columns[self.col_index(x, z)];
                    while si != SPAN_NULL {
                        let span = &mut self.spans[si as usize];
                        if span.min <= max_y && span.max >= min_y {
                            span.area = area_id;
                        }
                        si = span.next;
                    }
                }
            }
        }
        Ok(())
    }

    /// Marks spans within a convex polygon with the specified area ID
    pub fn mark_convex_poly_area(
        &mut self,
        verts: &[f32],
        nverts: usize,
        min_y: f32,
        max_y: f32,
        area_id: u8,
    ) -> Result<(), BuildError> {
        if nverts < 3 {
            return Err(BuildError::DegeneratePolygon);
        }

        let mut poly_min_x = verts[0];
        let mut poly_max_x = verts[0];
        let mut poly_min_z = verts[2];
        let mut poly_max_z = verts[2];

        for i in 1..nverts {
            let x = verts[i * 3];
            let z = verts[i * 3 + 2];
            poly_min_x = poly_min_x.min(x);
            poly_max_x = poly_max_x.max(x);
            poly_min_z = poly_min_z.min(z);
            poly_max_z = poly_max_z.max(z);
        }

        let min_x = ((poly_min_x - self.bmin.x) / self.cs).floor() as i32;
        let min_z = ((poly_min_z - self.bmin.z) / self.cs).floor() as i32;
        let max_x = ((poly_max_x - self.bmin.x) / self.cs).ceil() as i32;
        let max_z = ((poly_max_z - self.bmin.z) / self.cs).ceil() as i32;

        let min_x = min_x.max(0);
        let min_z = min_z.max(0);
        let max_x = max_x.min(self.width - 1);
        let max_z = max_z.min(self.height - 1);

        let span_min_y = ((min_y - self.bmin.y) / self.ch).floor() as i16;
        let span_max_y = ((max_y - self.bmin.y) / self.ch).ceil() as i16;

        for z in min_z..=max_z {
            for x in min_x..=max_x {
                let cell_x = self.bmin.x + (x as f32 + 0.5) * self.cs;
                let cell_z = self.bmin.z + (z as f32 + 0.5) * self.cs;

                if Self::point_in_poly_2d(cell_x, cell_z, verts, nverts) {
                    let mut si = self.columns[self.col_index(x, z)];
                    while si != SPAN_NULL {
                        let span = &mut self.spans[si as usize];
                        if span.min <= span_max_y && span.max >= span_min_y {
                            span.area = area_id;
                        }
                        si = span.next;
                    }
                }
            }
        }
        Ok(())
    }

    /// Tests if a point is inside a 2D convex polygon
    fn point_in_poly_2d(px: f32, pz: f32, verts: &[f32], nverts: usize) -> bool {
        let mut sign = 0i32;

        for i in 0..nverts {
            let j = (i + 1) % nverts;

            let v0x = verts[i * 3];
            let v0z = verts[i * 3 + 2];
            let v1x = verts[j * 3];
            let v1z = verts[j * 3 + 2];

            let edge_x = v1x - v0x;
            let edge_z = v1z - v0z;
            let to_point_x = px - v0x;
            let to_point_z = pz - v0z;
            let cross = edge_x * to_point_z - edge_z * to_point_x;

            if i == 0 {
                sign = if cross >= 0.0 { 1 } else { -1 };
            } else {
                let curr_sign = if cross >= 0.0 { 1 } else { -1 };
                if curr_sign != sign {
                    return false;
                }
            }
        }

        true
    }

    /// Gets the count of walkable spans in the heightfield
    pub fn get_span_count(&self) -> i32 {
        let mut count = 0;

        for z in 0..self.height {
            for x in 0..self.width {
                let mut si = self.columns[self.col_index(x, z)];
                while si != SPAN_NULL {
                    if self.spans[si as usize].area != 0 {
                        count += 1;
                    }
                    si = self.spans[si as usize].next;
                }
            }
        }

        count
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn test_heightfield_creation() {
        let width = 10;
        let height = 10;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(10.0, 10.0, 10.0);
        let cs = 1.0;
        let ch = 1.0;

        let heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        assert_eq!(heightfield.width, width);
        assert_eq!(heightfield.height, height);
        assert_eq!(heightfield.columns.len(), (width * height) as usize);
    }

    #[test]
    fn test_add_span() {
        let width = 10;
        let height = 10;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(10.0, 10.0, 10.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        let x = 5;
        let z = 5;
        let min = 3;
        let max = 6;
        let area = 1;

        heightfield.add_span(x, z, min, max, area).unwrap();

        // Check that the span was added correctly
        let si = heightfield.column_first_span(x, z);
        assert_ne!(si, SPAN_NULL);

        let span = heightfield.span(si);
        assert_eq!(span.min, min);
        assert_eq!(span.max, max);
        assert_eq!(span.area, area);
        assert_eq!(span.next, SPAN_NULL);
    }

    #[test]
    fn test_rasterize_triangle() {
        let width = 10;
        let height = 10;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(10.0, 10.0, 10.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        let triangle = [
            Vec3::new(2.0, 5.0, 2.0),
            Vec3::new(8.0, 5.0, 2.0),
            Vec3::new(5.0, 5.0, 8.0),
        ];

        let area = 1;
        heightfield.rasterize_triangle(&triangle, area).unwrap();

        // Check a few specific cells that should be inside the triangle
        assert_ne!(heightfield.column_first_span(5, 5), SPAN_NULL);
        assert_ne!(heightfield.column_first_span(2, 2), SPAN_NULL);
        assert_ne!(heightfield.column_first_span(7, 2), SPAN_NULL);
        assert_ne!(heightfield.column_first_span(5, 6), SPAN_NULL);

        // Check a cell outside the triangle
        assert_eq!(heightfield.column_first_span(0, 0), SPAN_NULL);
    }

    #[test]
    fn test_mark_box_area() {
        let width = 10;
        let height = 10;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(10.0, 10.0, 10.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 10, 1).unwrap();
            }
        }

        let box_min = [2.0, 2.0, 2.0];
        let box_max = [5.0, 8.0, 5.0];
        heightfield.mark_box_area(&box_min, &box_max, 5).unwrap();

        // Check that spans within the box have area 5
        for z in 2..5 {
            for x in 2..5 {
                let si = heightfield.column_first_span(x, z);
                assert_ne!(si, SPAN_NULL);
                assert_eq!(heightfield.span(si).area, 5);
            }
        }

        // Check that spans outside the box still have area 1
        let si = heightfield.column_first_span(0, 0);
        assert_ne!(si, SPAN_NULL);
        assert_eq!(heightfield.span(si).area, 1);
    }

    #[test]
    fn test_mark_cylinder_area() {
        let width = 10;
        let height = 10;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(10.0, 10.0, 10.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 10, 1).unwrap();
            }
        }

        let pos = [5.0, 2.0, 5.0];
        let radius = 2.5;
        let cylinder_height = 6.0;
        heightfield
            .mark_cylinder_area(&pos, radius, cylinder_height, 6)
            .unwrap();

        // Check center should be marked
        let si = heightfield.column_first_span(5, 5);
        assert_ne!(si, SPAN_NULL);
        assert_eq!(heightfield.span(si).area, 6);

        // Check corner should not be marked (outside radius)
        let si = heightfield.column_first_span(2, 2);
        assert_ne!(si, SPAN_NULL);
        assert_eq!(heightfield.span(si).area, 1);
    }

    #[test]
    fn test_mark_convex_poly_area() {
        let width = 10;
        let height = 10;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(10.0, 10.0, 10.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 10, 1).unwrap();
            }
        }

        let verts = vec![
            3.0, 0.0, 3.0, // vertex 0
            7.0, 0.0, 3.0, // vertex 1
            7.0, 0.0, 7.0, // vertex 2
            3.0, 0.0, 7.0, // vertex 3
        ];

        heightfield
            .mark_convex_poly_area(&verts, 4, 2.0, 8.0, 7)
            .unwrap();

        // Check center should be marked
        let si = heightfield.column_first_span(5, 5);
        assert_ne!(si, SPAN_NULL);
        assert_eq!(heightfield.span(si).area, 7);

        // Check outside should not be marked
        let si = heightfield.column_first_span(1, 1);
        assert_ne!(si, SPAN_NULL);
        assert_eq!(heightfield.span(si).area, 1);
    }

    #[test]
    fn test_filter_low_hanging_walkable_obstacles() {
        let width = 5;
        let height = 5;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(5.0, 10.0, 5.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        heightfield.add_span(2, 2, 0, 5, 1).unwrap(); // ground
        heightfield.add_span(2, 2, 6, 8, 0).unwrap(); // low obstacle
        heightfield.add_span(2, 2, 9, 15, 1).unwrap(); // walkable above

        let walkable_climb = 3;
        heightfield
            .filter_low_hanging_walkable_obstacles(walkable_climb)
            .unwrap();

        // Check that all spans are now walkable
        let mut si = heightfield.column_first_span(2, 2);
        let mut count = 0;
        while si != SPAN_NULL {
            assert_eq!(heightfield.span(si).area, 1);
            si = heightfield.span(si).next;
            count += 1;
        }
        assert_eq!(count, 3);
    }

    #[test]
    fn test_erode_walkable_area() {
        let width = 5;
        let height = 5;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(5.0, 10.0, 5.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        for z in 0..height {
            for x in 0..width {
                if x == 2 && z == 2 {
                    heightfield.add_span(x, z, 0, 5, 0).unwrap();
                } else {
                    heightfield.add_span(x, z, 0, 5, 1).unwrap();
                }
            }
        }

        let radius = 1;
        heightfield.erode_walkable_area(radius).unwrap();

        for z in 0..height {
            for x in 0..width {
                let si = heightfield.column_first_span(x, z);
                if si != SPAN_NULL {
                    let span = heightfield.span(si);
                    let dist_sq = (x - 2) * (x - 2) + (z - 2) * (z - 2);

                    if dist_sq <= radius * radius {
                        assert_eq!(span.area, 0, "Span at ({}, {}) should be eroded", x, z);
                    } else {
                        assert_eq!(
                            span.area, 1,
                            "Span at ({}, {}) should remain walkable",
                            x, z
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_median_filter_walkable_area() {
        let width = 5;
        let height = 5;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(5.0, 10.0, 5.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        for z in 0..height {
            for x in 0..width {
                let area = if x == 2 && z == 2 { 0 } else { 1 };
                heightfield.add_span(x, z, 0, 5, area).unwrap();
            }
        }

        heightfield.median_filter_walkable_area().unwrap();

        let si = heightfield.column_first_span(2, 2);
        assert_ne!(si, SPAN_NULL);
        assert_eq!(heightfield.span(si).area, 1);
    }
}
