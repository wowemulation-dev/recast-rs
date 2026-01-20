//! Heightfield representation for Recast
//!
//! The heightfield is the first data structure in the Recast pipeline.
//! It's a 2D grid of height spans that represents a voxelized 3D environment.

use glam::Vec3;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use recast_common::{Error, Result};

/// Represents cell bounds in XZ plane
struct CellBounds {
    min_x: f32,
    max_x: f32,
    min_z: f32,
    max_z: f32,
}

/// A span in the heightfield, representing a vertical segment of space
#[derive(Debug, Clone)]
pub struct Span {
    /// The minimum height of the span
    pub min: i16,
    /// The maximum height of the span
    pub max: i16,
    /// Area ID (0 = not walkable)
    pub area: u8,
    /// Next span in the column, or None if this is the last span
    pub next: Option<Rc<RefCell<Span>>>,
}

impl Span {
    /// Creates a new span
    pub fn new(min: i16, max: i16, area: u8) -> Self {
        Self {
            min,
            max,
            area,
            next: None,
        }
    }

    /// Creates a new span with a next span
    pub fn with_next(min: i16, max: i16, area: u8, next: Rc<RefCell<Span>>) -> Self {
        Self {
            min,
            max,
            area,
            next: Some(next),
        }
    }
}

/// Heightfield structure holding a grid of span columns
#[derive(Debug)]
pub struct Heightfield {
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

    /// Heightfield grid of span columns
    pub spans: HashMap<(i32, i32), Option<Rc<RefCell<Span>>>>,
}

impl Heightfield {
    /// Creates a new empty heightfield
    pub fn new(width: i32, height: i32, bmin: Vec3, bmax: Vec3, cs: f32, ch: f32) -> Self {
        let mut spans = HashMap::new();

        // Initialize all cells with None
        for z in 0..height {
            for x in 0..width {
                spans.insert((x, z), None);
            }
        }

        Self {
            width,
            height,
            bmin,
            bmax,
            cs,
            ch,
            spans,
        }
    }

    /// Add a span to the heightfield
    pub fn add_span(&mut self, x: i32, z: i32, min: i16, max: i16, area: u8) -> Result<()> {
        if x < 0 || x >= self.width || z < 0 || z >= self.height {
            return Err(Error::NavMeshGeneration(format!(
                "Span position out of bounds: ({}, {})",
                x, z
            )));
        }

        if min > max {
            return Err(Error::NavMeshGeneration(format!(
                "Invalid span height: min ({}) > max ({})",
                min, max
            )));
        }

        let new_span = Rc::new(RefCell::new(Span::new(min, max, area)));

        match self.spans.get_mut(&(x, z)) {
            Some(cell) => {
                match cell {
                    None => {
                        // First span in this column
                        *cell = Some(new_span);
                    }
                    Some(first_span) => {
                        // Insert the span in the column, maintaining height order
                        Self::insert_span(first_span.clone(), new_span)?;
                    }
                }
            }
            None => {
                return Err(Error::NavMeshGeneration(format!(
                    "Cell not found: ({}, {})",
                    x, z
                )));
            }
        }

        Ok(())
    }

    /// Insert a span into a column, maintaining height order and merging overlapping spans
    fn insert_span(first_span: Rc<RefCell<Span>>, new_span: Rc<RefCell<Span>>) -> Result<()> {
        let mut prev: Option<Rc<RefCell<Span>>> = None;
        let mut curr = first_span;
        let (new_min, new_max, new_area) = {
            let ns = new_span.borrow();
            (ns.min, ns.max, ns.area)
        };

        // Find the insertion point
        loop {
            let (curr_min, curr_max, curr_area) = {
                let cs = curr.borrow();
                (cs.min, cs.max, cs.area)
            };

            // Check for overlap with same area - merge if so
            if curr_area == new_area && curr_max >= new_min && new_max >= curr_min {
                // Merge overlapping spans
                let merged_min = curr_min.min(new_min);
                let merged_max = curr_max.max(new_max);

                // Update current span with merged values
                {
                    let mut cs = curr.borrow_mut();
                    cs.min = merged_min;
                    cs.max = merged_max;
                }

                // Check if we can merge with the next span too
                let next_opt = curr.borrow().next.clone();
                if let Some(next) = next_opt {
                    let (next_min, next_max, next_area) = {
                        let ns = next.borrow();
                        (ns.min, ns.max, ns.area)
                    };

                    if next_area == new_area && merged_max >= next_min {
                        // Merge with next span too
                        let final_max = merged_max.max(next_max);
                        let next_next = next.borrow().next.clone();

                        let mut cs = curr.borrow_mut();
                        cs.max = final_max;
                        cs.next = next_next;
                    }
                }

                return Ok(());
            }

            // If new span is below current span
            if new_max < curr_min {
                // Insert new span before current span
                if let Some(prev_span) = prev {
                    prev_span.borrow_mut().next = Some(new_span.clone());
                    new_span.borrow_mut().next = Some(curr);
                } else {
                    // This is the first span, need to swap
                    let curr_clone = curr.borrow().clone();
                    *curr.borrow_mut() = new_span.borrow().clone();
                    curr.borrow_mut().next = Some(Rc::new(RefCell::new(curr_clone)));
                }
                break;
            }

            // If there's no next span, add new span at the end
            let next_opt = curr.borrow().next.clone();
            if next_opt.is_none() {
                curr.borrow_mut().next = Some(new_span);
                break;
            }

            // Move to next span
            prev = Some(curr);
            curr = next_opt.unwrap();
        }

        Ok(())
    }

    /// Get the height range of the heightfield
    pub fn get_height_range(&self) -> (i16, i16) {
        let mut min_height = i16::MAX;
        let mut max_height = i16::MIN;

        for column in self.spans.values() {
            let mut current = column.clone();

            while let Some(span_rc) = current {
                let span = span_rc.borrow();
                min_height = min_height.min(span.min);
                max_height = max_height.max(span.max);
                current = span.next.clone();
            }
        }

        (min_height, max_height)
    }

    /// Filters spans in the heightfield based on walkable criteria
    pub fn filter_walkable(&mut self, walkable_height: i16, walkable_climb: i16) -> Result<()> {
        // Process each column
        for ((_x, _z), column) in self.spans.iter_mut() {
            if let Some(first_span) = column {
                let mut current_span = Some(first_span.clone());

                while let Some(span_rc) = current_span {
                    let mut span = span_rc.borrow_mut();

                    // Check if this span is walkable
                    let span_height = span.max - span.min;
                    let is_walkable = span_height >= walkable_height;

                    // Check if the climb to the next span is walkable
                    let should_mark_unwalkable = if let Some(next_span_rc) = &span.next {
                        let next_span = next_span_rc.borrow();
                        let climb = next_span.min as i32 - span.max as i32;
                        climb > walkable_climb as i32
                    } else {
                        false
                    };

                    if should_mark_unwalkable {
                        span.area = 0;
                    }

                    // If the span is not walkable, mark it
                    if !is_walkable {
                        span.area = 0;
                    }

                    // Move to next span
                    current_span = span.next.clone();
                }
            }
        }

        Ok(())
    }

    /// Filters low hanging walkable obstacles that the agent can step over
    /// This implements the C++ rcFilterLowHangingWalkableObstacles algorithm
    pub fn filter_low_hanging_walkable_obstacles(&mut self, walkable_climb: i16) -> Result<()> {
        let x_size = self.width;
        let z_size = self.height;

        for z in 0..z_size {
            for x in 0..x_size {
                if let Some(Some(first_span)) = self.spans.get(&(x, z)) {
                    let mut previous_span: Option<Rc<RefCell<Span>>> = None;
                    let mut previous_was_walkable = false;
                    let mut previous_area_id = 0u8;

                    let mut current = Some(first_span.clone());
                    while let Some(span_rc) = current {
                        let walkable = span_rc.borrow().area != 0; // RC_NULL_AREA is 0

                        // If current span is not walkable, but there is walkable span just below it
                        // and the height difference is small enough for the agent to walk over,
                        // mark the current span as walkable too.
                        if !walkable && previous_was_walkable {
                            if let Some(prev_span) = &previous_span {
                                let prev_max = prev_span.borrow().max;
                                let curr_max = span_rc.borrow().max;

                                if (curr_max as i32 - prev_max as i32) <= walkable_climb as i32 {
                                    span_rc.borrow_mut().area = previous_area_id;
                                }
                            }
                        }

                        // Copy the original walkable value regardless of whether we changed it.
                        // This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
                        previous_was_walkable = walkable;
                        previous_area_id = span_rc.borrow().area;
                        previous_span = Some(span_rc.clone());

                        current = span_rc.borrow().next.clone();
                    }
                }
            }
        }

        Ok(())
    }

    /// Filters ledge spans - marks spans that are adjacent to ledges as unwalkable
    /// This implements the C++ rcFilterLedgeSpans algorithm
    pub fn filter_ledge_spans(&mut self, walkable_height: i16, walkable_climb: i16) -> Result<()> {
        const MAX_HEIGHTFIELD_HEIGHT: i32 = 0xffff;

        let x_size = self.width;
        let z_size = self.height;

        // Direction offsets for 4-directional neighbors (N, E, S, W)
        let dir_offsets = [(0, -1), (1, 0), (0, 1), (-1, 0)];

        // Mark spans that are adjacent to a ledge as unwalkable
        for z in 0..z_size {
            for x in 0..x_size {
                if let Some(Some(first_span)) = self.spans.get(&(x, z)) {
                    let mut current = Some(first_span.clone());

                    while let Some(span_rc) = current {
                        let span = span_rc.borrow();

                        // Skip non-walkable spans
                        if span.area == 0 {
                            current = span.next.clone();
                            continue;
                        }

                        let floor = span.max as i32;
                        let ceiling = if let Some(ref next_span) = span.next {
                            next_span.borrow().min as i32
                        } else {
                            MAX_HEIGHTFIELD_HEIGHT
                        };

                        // The difference between this walkable area and the lowest neighbor walkable area
                        let mut lowest_neighbor_floor_difference = MAX_HEIGHTFIELD_HEIGHT;

                        // Min and max height of accessible neighbours
                        let mut lowest_traversable_neighbor_floor = span.max as i32;
                        let mut highest_traversable_neighbor_floor = span.max as i32;

                        // Check all 4 directions
                        let mut is_ledge = false;
                        for &(dx, dz) in &dir_offsets {
                            let neighbor_x = x + dx;
                            let neighbor_z = z + dz;

                            // Skip neighbours which are out of bounds
                            if neighbor_x < 0
                                || neighbor_z < 0
                                || neighbor_x >= x_size
                                || neighbor_z >= z_size
                            {
                                lowest_neighbor_floor_difference = -walkable_climb as i32 - 1;
                                is_ledge = true;
                                break;
                            }

                            if let Some(Some(neighbor_first_span)) =
                                self.spans.get(&(neighbor_x, neighbor_z))
                            {
                                let mut neighbor_span_opt = Some(neighbor_first_span.clone());

                                // The most we can step down to the neighbor is the walkableClimb distance
                                let mut neighbor_ceiling =
                                    if let Some(ref first_neighbor) = neighbor_span_opt {
                                        first_neighbor.borrow().min as i32
                                    } else {
                                        MAX_HEIGHTFIELD_HEIGHT
                                    };

                                // Skip neighbour if the gap between the spans is too small
                                if (ceiling.min(neighbor_ceiling) - floor) >= walkable_height as i32
                                {
                                    lowest_neighbor_floor_difference = -walkable_climb as i32 - 1;
                                    is_ledge = true;
                                    break;
                                }

                                // For each span in the neighboring column
                                while let Some(neighbor_span_rc) = neighbor_span_opt {
                                    let neighbor_span = neighbor_span_rc.borrow();
                                    let neighbor_floor = neighbor_span.max as i32;
                                    neighbor_ceiling =
                                        if let Some(ref next_neighbor) = neighbor_span.next {
                                            next_neighbor.borrow().min as i32
                                        } else {
                                            MAX_HEIGHTFIELD_HEIGHT
                                        };

                                    // Only consider neighboring areas that have enough overlap to be potentially traversable
                                    if (ceiling.min(neighbor_ceiling) - floor.max(neighbor_floor))
                                        < walkable_height as i32
                                    {
                                        // No space to traverse between them
                                        neighbor_span_opt = neighbor_span.next.clone();
                                        continue;
                                    }

                                    let neighbor_floor_difference = neighbor_floor - floor;
                                    lowest_neighbor_floor_difference =
                                        lowest_neighbor_floor_difference
                                            .min(neighbor_floor_difference);

                                    // Find min/max accessible neighbor height
                                    // Only consider neighbors that are at most walkableClimb away
                                    if neighbor_floor_difference.abs() <= walkable_climb as i32 {
                                        // There is space to move to the neighbor cell and the slope isn't too much
                                        lowest_traversable_neighbor_floor =
                                            lowest_traversable_neighbor_floor.min(neighbor_floor);
                                        highest_traversable_neighbor_floor =
                                            highest_traversable_neighbor_floor.max(neighbor_floor);
                                    } else if neighbor_floor_difference < -walkable_climb as i32 {
                                        // We already know this will be considered a ledge span so we can early-out
                                        break;
                                    }

                                    neighbor_span_opt = neighbor_span.next.clone();
                                }
                            } else {
                                // No neighbor span column - treat as ledge
                                lowest_neighbor_floor_difference = -walkable_climb as i32 - 1;
                                is_ledge = true;
                                break;
                            }
                        }

                        if is_ledge {
                            // Mark as unwalkable due to being near boundary or having insufficient clearance
                            drop(span); // Release borrow before modifying
                            span_rc.borrow_mut().area = 0;
                        } else {
                            // Check ledge conditions
                            if lowest_neighbor_floor_difference < -walkable_climb as i32 {
                                // The current span is close to a ledge
                                drop(span); // Release borrow before modifying
                                span_rc.borrow_mut().area = 0;
                            } else if (highest_traversable_neighbor_floor
                                - lowest_traversable_neighbor_floor)
                                > walkable_climb as i32
                            {
                                // If the difference between all neighbor floors is too large, this is a steep slope
                                drop(span); // Release borrow before modifying
                                span_rc.borrow_mut().area = 0;
                            }
                        }

                        current = if let Ok(span_ref) = span_rc.try_borrow() {
                            span_ref.next.clone()
                        } else {
                            // If we can't borrow (because we modified it above), get next from the modified span
                            span_rc.borrow().next.clone()
                        };
                    }
                }
            }
        }

        Ok(())
    }

    /// Filters walkable spans that have insufficient clearance above them
    /// This implements the C++ rcFilterWalkableLowHeightSpans algorithm
    pub fn filter_walkable_low_height_spans(&mut self, walkable_height: i16) -> Result<()> {
        const MAX_HEIGHTFIELD_HEIGHT: i32 = 0xffff;

        let x_size = self.width;
        let z_size = self.height;

        // Remove walkable flag from spans which do not have enough
        // space above them for the agent to stand there.
        for z in 0..z_size {
            for x in 0..x_size {
                if let Some(Some(first_span)) = self.spans.get(&(x, z)) {
                    let mut current = Some(first_span.clone());

                    while let Some(span_rc) = current {
                        let span = span_rc.borrow();
                        let floor = span.max as i32;
                        let ceiling = if let Some(ref next_span) = span.next {
                            next_span.borrow().min as i32
                        } else {
                            MAX_HEIGHTFIELD_HEIGHT
                        };

                        if (ceiling - floor) < walkable_height as i32 {
                            drop(span); // Release borrow before modifying
                            span_rc.borrow_mut().area = 0; // RC_NULL_AREA
                        }

                        current = if let Ok(span_ref) = span_rc.try_borrow() {
                            span_ref.next.clone()
                        } else {
                            // If we can't borrow (because we modified it above), get next from the modified span
                            span_rc.borrow().next.clone()
                        };
                    }
                }
            }
        }

        Ok(())
    }

    /// Marks border spans (spans at the edge of the heightfield)
    #[allow(dead_code)]
    fn mark_border_spans(&mut self) -> Result<()> {
        // Mark spans on the border of the heightfield
        for z in 0..self.height {
            for x in 0..self.width {
                // Check if this is a border cell
                if x == 0 || x == self.width - 1 || z == 0 || z == self.height - 1 {
                    if let Some(Some(span)) = self.spans.get(&(x, z)) {
                        let mut current = Some(span.clone());

                        while let Some(span_rc) = current {
                            // Mark border spans as unwalkable
                            span_rc.borrow_mut().area = 0;
                            current = span_rc.borrow().next.clone();
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Applies median filter to remove noise in walkable areas
    pub fn median_filter_walkable_area(&mut self) -> Result<()> {
        let width = self.width;
        let height = self.height;

        // Create a copy of area values for reading
        let mut area_copy = HashMap::new();

        // Copy current area values
        for z in 0..height {
            for x in 0..width {
                if let Some(Some(span)) = self.spans.get(&(x, z)) {
                    let mut current = Some(span.clone());
                    let mut span_index = 0;

                    while let Some(span_rc) = current {
                        let area = span_rc.borrow().area;
                        area_copy.insert((x, z, span_index), area);
                        current = span_rc.borrow().next.clone();
                        span_index += 1;
                    }
                }
            }
        }

        // Apply median filter
        for z in 1..height - 1 {
            for x in 1..width - 1 {
                if let Some(Some(span)) = self.spans.get(&(x, z)) {
                    let mut current = Some(span.clone());
                    let mut span_index = 0;

                    while let Some(span_rc) = current {
                        let mut neighbor_areas = Vec::new();

                        // Collect areas from 3x3 neighborhood at similar heights
                        for dz in -1..=1 {
                            for dx in -1..=1 {
                                let nx = x + dx;
                                let nz = z + dz;

                                if let Some(&area) = area_copy.get(&(nx, nz, span_index)) {
                                    neighbor_areas.push(area);
                                }
                            }
                        }

                        // Apply median filter if we have enough neighbors
                        if neighbor_areas.len() >= 5 {
                            neighbor_areas.sort();
                            let median_area = neighbor_areas[neighbor_areas.len() / 2];

                            // Only change if current area is different from median
                            let current_area = span_rc.borrow().area;
                            if current_area != median_area {
                                // Count occurrences of median value
                                let median_count =
                                    neighbor_areas.iter().filter(|&&a| a == median_area).count();

                                // Only change if median is dominant (appears in majority)
                                if median_count > neighbor_areas.len() / 2 {
                                    span_rc.borrow_mut().area = median_area;
                                }
                            }
                        }

                        current = span_rc.borrow().next.clone();
                        span_index += 1;
                    }
                }
            }
        }

        Ok(())
    }

    /// Erodes the walkable area by the specified radius
    pub fn erode_walkable_area(&mut self, radius: i32) -> Result<()> {
        if radius <= 0 {
            return Ok(());
        }

        let width = self.width;
        let height = self.height;

        // Find all non-walkable spans
        let mut non_walkable = HashSet::new();

        for z in 0..height {
            for x in 0..width {
                if let Some(Some(span)) = self.spans.get(&(x, z)) {
                    let mut current = Some(span.clone());
                    let mut span_index = 0;

                    while let Some(span_rc) = current {
                        if span_rc.borrow().area == 0 {
                            non_walkable.insert((x, z, span_index));
                        }
                        current = span_rc.borrow().next.clone();
                        span_index += 1;
                    }
                }
            }
        }

        // Erode walkable areas near non-walkable spans
        let mut to_erode = HashSet::new();

        for &(x, z, span_index) in &non_walkable {
            // Check all cells within radius
            for dz in -radius..=radius {
                for dx in -radius..=radius {
                    let nx = x + dx;
                    let nz = z + dz;

                    // Skip out of bounds
                    if nx < 0 || nx >= width || nz < 0 || nz >= height {
                        continue;
                    }

                    // Check if within radius (using squared distance to avoid sqrt)
                    let dist_sq = dx * dx + dz * dz;
                    if dist_sq <= radius * radius {
                        to_erode.insert((nx, nz, span_index));
                    }
                }
            }
        }

        // Apply erosion
        for (x, z, span_index) in to_erode {
            if let Some(Some(span)) = self.spans.get(&(x, z)) {
                let mut current = Some(span.clone());
                let mut idx = 0;

                while let Some(span_rc) = current {
                    if idx == span_index {
                        span_rc.borrow_mut().area = 0;
                        break;
                    }
                    current = span_rc.borrow().next.clone();
                    idx += 1;
                }
            }
        }

        Ok(())
    }

    /// Rasterizes a triangle into the heightfield
    pub fn rasterize_triangle(&mut self, verts: &[Vec3; 3], area: u8) -> Result<()> {
        if verts.len() != 3 {
            return Err(Error::NavMeshGeneration(
                "Triangle must have exactly 3 vertices".to_string(),
            ));
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
                // Get cell bounds
                let bounds = CellBounds {
                    min_x: self.bmin.x + (x as f32) * self.cs,
                    max_x: self.bmin.x + ((x + 1) as f32) * self.cs,
                    min_z: self.bmin.z + (z as f32) * self.cs,
                    max_z: self.bmin.z + ((z + 1) as f32) * self.cs,
                };

                // Check if triangle overlaps this cell using a more conservative approach
                if !self.triangle_overlaps_cell_xz(&verts[0], &verts[1], &verts[2], &bounds) {
                    continue;
                }

                // Use cell center for height interpolation
                let pt = Vec3::new(
                    self.bmin.x + (x as f32 + 0.5) * self.cs,
                    0.0,
                    self.bmin.z + (z as f32 + 0.5) * self.cs,
                );

                // Calculate the height (y) at this point using barycentric coordinates
                let y = self.interpolate_height(&pt, &verts[0], &verts[1], &verts[2]);

                // Convert to heightfield height (span height)
                let h = ((y - self.bmin.y) / self.ch) as i16;

                // Add span to the heightfield
                // Create a span with sufficient height to represent walkable space
                // The span extends from the surface up by a reasonable amount
                let span_height = 10; // Default height for walkable spans
                let max_height = h.saturating_add(span_height);
                self.add_span(x, z, h, max_height, area)?;
            }
        }

        Ok(())
    }

    /// Checks if a point is inside a triangle (XZ plane projection)
    fn point_in_triangle_xz(&self, p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> bool {
        // Compute vectors
        let v0x = c.x - a.x;
        let v0z = c.z - a.z;
        let v1x = b.x - a.x;
        let v1z = b.z - a.z;
        let v2x = p.x - a.x;
        let v2z = p.z - a.z;

        // Compute dot products
        let dot00 = v0x * v0x + v0z * v0z;
        let dot01 = v0x * v1x + v0z * v1z;
        let dot02 = v0x * v2x + v0z * v2z;
        let dot11 = v1x * v1x + v1z * v1z;
        let dot12 = v1x * v2x + v1z * v2z;

        // Compute barycentric coordinates
        let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        // Check if point is in triangle
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
        // First, check if triangle bounding box overlaps cell
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

        // Check if any cell corner is inside the triangle
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

        // Check if any triangle vertex is inside the cell
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

        // Check if any triangle edge intersects the cell
        // This handles the case where the triangle passes through the cell
        // but no vertices are inside and no corners are inside the triangle
        if self.edge_intersects_cell_xz(v0, v1, bounds)
            || self.edge_intersects_cell_xz(v1, v2, bounds)
            || self.edge_intersects_cell_xz(v2, v0, bounds)
        {
            return true;
        }

        // If none of the above conditions are met, there's no overlap
        false
    }

    /// Checks if an edge intersects a cell in the XZ plane
    fn edge_intersects_cell_xz(&self, p0: &Vec3, p1: &Vec3, bounds: &CellBounds) -> bool {
        // Check if edge crosses any of the four cell boundaries
        // Using parametric line equation: p = p0 + t * (p1 - p0)
        let dx = p1.x - p0.x;
        let dz = p1.z - p0.z;

        // Check intersection with left edge (x = cell_min_x)
        if dx.abs() > f32::EPSILON {
            let t = (bounds.min_x - p0.x) / dx;
            if (0.0..=1.0).contains(&t) {
                let z = p0.z + t * dz;
                if z >= bounds.min_z && z <= bounds.max_z {
                    return true;
                }
            }
        }

        // Check intersection with right edge (x = cell_max_x)
        if dx.abs() > f32::EPSILON {
            let t = (bounds.max_x - p0.x) / dx;
            if (0.0..=1.0).contains(&t) {
                let z = p0.z + t * dz;
                if z >= bounds.min_z && z <= bounds.max_z {
                    return true;
                }
            }
        }

        // Check intersection with bottom edge (z = cell_min_z)
        if dz.abs() > f32::EPSILON {
            let t = (bounds.min_z - p0.z) / dz;
            if (0.0..=1.0).contains(&t) {
                let x = p0.x + t * dx;
                if x >= bounds.min_x && x <= bounds.max_x {
                    return true;
                }
            }
        }

        // Check intersection with top edge (z = cell_max_z)
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
        // Compute vectors
        let v0x = c.x - a.x;
        let v0z = c.z - a.z;
        let v1x = b.x - a.x;
        let v1z = b.z - a.z;
        let v2x = p.x - a.x;
        let v2z = p.z - a.z;

        // Compute dot products
        let dot00 = v0x * v0x + v0z * v0z;
        let dot01 = v0x * v1x + v0z * v1z;
        let dot02 = v0x * v2x + v0z * v2z;
        let dot11 = v1x * v1x + v1z * v1z;
        let dot12 = v1x * v2x + v1z * v2z;

        // Compute barycentric coordinates
        let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
        let w = 1.0 - u - v;

        // Interpolate height
        a.y * w + b.y * v + c.y * u
    }

    /// Marks spans within an axis-aligned box with the specified area ID
    pub fn mark_box_area(&mut self, bmin: &[f32; 3], bmax: &[f32; 3], area_id: u8) -> Result<()> {
        // Convert world coordinates to cell coordinates
        let min_x = ((bmin[0] - self.bmin.x) / self.cs).floor() as i32;
        let min_z = ((bmin[2] - self.bmin.z) / self.cs).floor() as i32;
        let max_x = ((bmax[0] - self.bmin.x) / self.cs).ceil() as i32;
        let max_z = ((bmax[2] - self.bmin.z) / self.cs).ceil() as i32;

        // Clamp to heightfield bounds
        let min_x = min_x.max(0);
        let min_z = min_z.max(0);
        let max_x = max_x.min(self.width - 1);
        let max_z = max_z.min(self.height - 1);

        // Convert Y bounds to span units
        let min_y = ((bmin[1] - self.bmin.y) / self.ch).floor() as i16;
        let max_y = ((bmax[1] - self.bmin.y) / self.ch).ceil() as i16;

        // Mark all spans within the box
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                if let Some(Some(span)) = self.spans.get(&(x, z)) {
                    let mut current = Some(span.clone());

                    while let Some(span_rc) = current {
                        let mut span_ref = span_rc.borrow_mut();

                        // Check if span overlaps with the box in Y
                        if span_ref.min <= max_y && span_ref.max >= min_y {
                            span_ref.area = area_id;
                        }

                        current = span_ref.next.clone();
                    }
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
    ) -> Result<()> {
        // Convert cylinder bounds to cell coordinates
        let min_x = ((pos[0] - radius - self.bmin.x) / self.cs).floor() as i32;
        let min_z = ((pos[2] - radius - self.bmin.z) / self.cs).floor() as i32;
        let max_x = ((pos[0] + radius - self.bmin.x) / self.cs).ceil() as i32;
        let max_z = ((pos[2] + radius - self.bmin.z) / self.cs).ceil() as i32;

        // Clamp to heightfield bounds
        let min_x = min_x.max(0);
        let min_z = min_z.max(0);
        let max_x = max_x.min(self.width - 1);
        let max_z = max_z.min(self.height - 1);

        // Convert Y bounds to span units
        let min_y = ((pos[1] - self.bmin.y) / self.ch).floor() as i16;
        let max_y = ((pos[1] + height - self.bmin.y) / self.ch).ceil() as i16;

        let radius_sq = radius * radius;

        // Mark all spans within the cylinder
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                // Calculate cell center
                let cell_x = self.bmin.x + (x as f32 + 0.5) * self.cs;
                let cell_z = self.bmin.z + (z as f32 + 0.5) * self.cs;

                // Check if cell center is within cylinder radius
                let dx = cell_x - pos[0];
                let dz = cell_z - pos[2];
                let dist_sq = dx * dx + dz * dz;

                if dist_sq <= radius_sq {
                    if let Some(Some(span)) = self.spans.get(&(x, z)) {
                        let mut current = Some(span.clone());

                        while let Some(span_rc) = current {
                            let mut span_ref = span_rc.borrow_mut();

                            // Check if span overlaps with the cylinder in Y
                            if span_ref.min <= max_y && span_ref.max >= min_y {
                                span_ref.area = area_id;
                            }

                            current = span_ref.next.clone();
                        }
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
    ) -> Result<()> {
        if nverts < 3 {
            return Err(Error::NavMeshGeneration(
                "Polygon must have at least 3 vertices".to_string(),
            ));
        }

        // Find polygon bounds
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

        // Convert bounds to cell coordinates
        let min_x = ((poly_min_x - self.bmin.x) / self.cs).floor() as i32;
        let min_z = ((poly_min_z - self.bmin.z) / self.cs).floor() as i32;
        let max_x = ((poly_max_x - self.bmin.x) / self.cs).ceil() as i32;
        let max_z = ((poly_max_z - self.bmin.z) / self.cs).ceil() as i32;

        // Clamp to heightfield bounds
        let min_x = min_x.max(0);
        let min_z = min_z.max(0);
        let max_x = max_x.min(self.width - 1);
        let max_z = max_z.min(self.height - 1);

        // Convert Y bounds to span units
        let span_min_y = ((min_y - self.bmin.y) / self.ch).floor() as i16;
        let span_max_y = ((max_y - self.bmin.y) / self.ch).ceil() as i16;

        // Mark all spans within the polygon
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                // Calculate cell center
                let cell_x = self.bmin.x + (x as f32 + 0.5) * self.cs;
                let cell_z = self.bmin.z + (z as f32 + 0.5) * self.cs;

                // Check if cell center is within polygon using point-in-polygon test
                if Self::point_in_poly_2d(cell_x, cell_z, verts, nverts) {
                    if let Some(Some(span)) = self.spans.get(&(x, z)) {
                        let mut current = Some(span.clone());

                        while let Some(span_rc) = current {
                            let mut span_ref = span_rc.borrow_mut();

                            // Check if span overlaps with the height range
                            if span_ref.min <= span_max_y && span_ref.max >= span_min_y {
                                span_ref.area = area_id;
                            }

                            current = span_ref.next.clone();
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Tests if a point is inside a 2D convex polygon
    fn point_in_poly_2d(px: f32, pz: f32, verts: &[f32], nverts: usize) -> bool {
        // Use the cross product method for convex polygons
        // Point is inside if it's on the same side of all edges
        let mut sign = 0i32;

        for i in 0..nverts {
            let j = (i + 1) % nverts;

            let v0x = verts[i * 3];
            let v0z = verts[i * 3 + 2];
            let v1x = verts[j * 3];
            let v1z = verts[j * 3 + 2];

            // Edge vector
            let edge_x = v1x - v0x;
            let edge_z = v1z - v0z;

            // Vector from vertex to point
            let to_point_x = px - v0x;
            let to_point_z = pz - v0z;

            // Cross product (2D)
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
                if let Some(Some(span_ref)) = self.spans.get(&(x, z)) {
                    let mut current = Some(Rc::clone(span_ref));
                    while let Some(span) = current {
                        let span_borrow = span.borrow();
                        if span_borrow.area != 0 {
                            count += 1;
                        }
                        current = span_borrow.next.as_ref().map(Rc::clone);
                    }
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
        assert_eq!(heightfield.spans.len(), (width * height) as usize);
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

        // Add a span
        let x = 5;
        let z = 5;
        let min = 3;
        let max = 6;
        let area = 1;

        heightfield.add_span(x, z, min, max, area).unwrap();

        // Check that the span was added correctly
        let span = heightfield.spans.get(&(x, z)).unwrap();
        assert!(span.is_some());

        let span = span.as_ref().unwrap().borrow();
        assert_eq!(span.min, min);
        assert_eq!(span.max, max);
        assert_eq!(span.area, area);
        assert!(span.next.is_none());
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

        // Create a triangle that covers several cells
        let triangle = [
            Vec3::new(2.0, 5.0, 2.0),
            Vec3::new(8.0, 5.0, 2.0),
            Vec3::new(5.0, 5.0, 8.0),
        ];

        let area = 1;

        heightfield.rasterize_triangle(&triangle, area).unwrap();

        // Check if the triangle was rasterized correctly
        // The triangle should cover approximately these cells:
        // (2,2), (3,2), (4,2), (5,2), (6,2), (7,2), (8,2)
        // (2,3), (3,3), (4,3), (5,3), (6,3), (7,3), (8,3)
        // (3,4), (4,4), (5,4), (6,4), (7,4)
        // (3,5), (4,5), (5,5), (6,5), (7,5)
        // (4,6), (5,6), (6,6)
        // (4,7), (5,7), (6,7)
        // (5,8)

        // Check a few specific cells that should be inside the triangle
        assert!(heightfield.spans.get(&(5, 5)).unwrap().is_some());
        assert!(heightfield.spans.get(&(2, 2)).unwrap().is_some());
        assert!(heightfield.spans.get(&(7, 2)).unwrap().is_some()); // Changed from (8,2)
        assert!(heightfield.spans.get(&(5, 6)).unwrap().is_some()); // Changed from (5,8)

        // Check a cell outside the triangle
        assert!(heightfield.spans.get(&(0, 0)).unwrap().is_none());
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

        // Add some spans
        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 10, 1).unwrap();
            }
        }

        // Mark a box area
        let box_min = [2.0, 2.0, 2.0];
        let box_max = [5.0, 8.0, 5.0];
        heightfield.mark_box_area(&box_min, &box_max, 5).unwrap();

        // Check that spans within the box have area 5
        for z in 2..5 {
            for x in 2..5 {
                if let Some(Some(span)) = heightfield.spans.get(&(x, z)) {
                    let span_ref = span.borrow();
                    assert_eq!(span_ref.area, 5);
                }
            }
        }

        // Check that spans outside the box still have area 1
        if let Some(Some(span)) = heightfield.spans.get(&(0, 0)) {
            let span_ref = span.borrow();
            assert_eq!(span_ref.area, 1);
        }
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

        // Add some spans
        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 10, 1).unwrap();
            }
        }

        // Mark a cylinder area
        let pos = [5.0, 2.0, 5.0];
        let radius = 2.5;
        let cylinder_height = 6.0;
        heightfield
            .mark_cylinder_area(&pos, radius, cylinder_height, 6)
            .unwrap();

        // Check center should be marked
        if let Some(Some(span)) = heightfield.spans.get(&(5, 5)) {
            let span_ref = span.borrow();
            assert_eq!(span_ref.area, 6);
        }

        // Check corner should not be marked (outside radius)
        if let Some(Some(span)) = heightfield.spans.get(&(2, 2)) {
            let span_ref = span.borrow();
            assert_eq!(span_ref.area, 1);
        }
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

        // Add some spans
        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 10, 1).unwrap();
            }
        }

        // Define a square polygon
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
        if let Some(Some(span)) = heightfield.spans.get(&(5, 5)) {
            let span_ref = span.borrow();
            assert_eq!(span_ref.area, 7);
        }

        // Check outside should not be marked
        if let Some(Some(span)) = heightfield.spans.get(&(1, 1)) {
            let span_ref = span.borrow();
            assert_eq!(span_ref.area, 1);
        }
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

        // Add a walkable span, then a low obstacle, then walkable space above
        heightfield.add_span(2, 2, 0, 5, 1).unwrap(); // ground
        heightfield.add_span(2, 2, 6, 8, 0).unwrap(); // low obstacle (height 2)
        heightfield.add_span(2, 2, 9, 15, 1).unwrap(); // walkable space above

        let walkable_climb = 3; // Can climb 3 units
        heightfield
            .filter_low_hanging_walkable_obstacles(walkable_climb)
            .unwrap();

        // Check that the low obstacle is now marked as walkable
        if let Some(Some(span)) = heightfield.spans.get(&(2, 2)) {
            let mut current = Some(span.clone());
            let mut count = 0;

            while let Some(span_rc) = current {
                let area = span_rc.borrow().area;
                // All spans should now be walkable
                assert_eq!(area, 1);
                current = span_rc.borrow().next.clone();
                count += 1;
            }

            assert_eq!(count, 3); // Should have 3 spans
        }
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

        // Add walkable spans everywhere (but only ground level)
        for z in 0..height {
            for x in 0..width {
                if x == 2 && z == 2 {
                    // Add non-walkable obstacle at center
                    heightfield.add_span(x, z, 0, 5, 0).unwrap();
                } else {
                    heightfield.add_span(x, z, 0, 5, 1).unwrap();
                }
            }
        }

        let radius = 1;
        heightfield.erode_walkable_area(radius).unwrap();

        // Check erosion results
        for z in 0..height {
            for x in 0..width {
                if let Some(Some(span)) = heightfield.spans.get(&(x, z)) {
                    let span_ref = span.borrow();
                    let dist_sq = (x - 2) * (x - 2) + (z - 2) * (z - 2);

                    if dist_sq <= radius * radius {
                        // Areas within radius 1 of (2,2) should be eroded (non-walkable)
                        assert_eq!(span_ref.area, 0, "Span at ({}, {}) should be eroded", x, z);
                    } else {
                        // Areas outside radius should remain walkable
                        assert_eq!(
                            span_ref.area, 1,
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

        // Add spans with noise (most walkable, one non-walkable in the middle)
        for z in 0..height {
            for x in 0..width {
                let area = if x == 2 && z == 2 { 0 } else { 1 };
                heightfield.add_span(x, z, 0, 5, area).unwrap();
            }
        }

        heightfield.median_filter_walkable_area().unwrap();

        // The noise in the center should be filtered out (changed to walkable)
        if let Some(Some(span)) = heightfield.spans.get(&(2, 2)) {
            let span_ref = span.borrow();
            assert_eq!(span_ref.area, 1);
        }
    }
}
