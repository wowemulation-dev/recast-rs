//! Distance field generation for improved region building
//!
//! This module implements distance field generation and watershed-based
//! region building algorithms for better quality navigation mesh generation.

use super::compact_heightfield::CompactHeightfield;
use recast_common::Result;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

/// Distance field entry for priority queue
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DistanceEntry {
    distance: u16,
    span_idx: usize,
}

impl Ord for DistanceEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap
        other.distance.cmp(&self.distance)
    }
}

impl PartialOrd for DistanceEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Builds a distance field from boundary spans
pub fn build_distance_field(chf: &CompactHeightfield, boundary_flags: &[u8]) -> Result<Vec<u16>> {
    let span_count = chf.spans.len();
    let mut distances = vec![0u16; span_count];
    let mut queue = BinaryHeap::new();

    // Initialize with boundary spans (distance 0)
    for span_idx in 0..span_count {
        let span = &chf.spans[span_idx];

        // Skip non-walkable spans
        if span.area == 0 {
            distances[span_idx] = 0;
            continue;
        }

        // Check if this is a boundary span
        if boundary_flags[span_idx] != 0 {
            distances[span_idx] = 0;
            queue.push(DistanceEntry {
                distance: 0,
                span_idx,
            });
        } else {
            distances[span_idx] = u16::MAX;
        }
    }

    // Dijkstra's algorithm to compute distances
    while let Some(entry) = queue.pop() {
        let current_dist = entry.distance;
        let span_idx = entry.span_idx;

        // Skip if we've found a better distance already
        if current_dist > distances[span_idx] {
            continue;
        }

        // Check all 4 cardinal neighbors
        // Using 8-direction constants: N=1, E=3, S=5, W=7
        for dir in [1u8, 3u8, 5u8, 7u8] {
            if let Some(neighbor_idx) = chf.get_neighbor(span_idx, dir) {
                let neighbor_span = &chf.spans[neighbor_idx];

                // Skip non-walkable neighbors
                if neighbor_span.area == 0 {
                    continue;
                }

                let new_dist = current_dist.saturating_add(2); // Use distance 2 to avoid overflow issues

                if new_dist < distances[neighbor_idx] {
                    distances[neighbor_idx] = new_dist;
                    queue.push(DistanceEntry {
                        distance: new_dist,
                        span_idx: neighbor_idx,
                    });
                }
            }
        }
    }

    Ok(distances)
}

/// Builds regions using watershed partitioning based on distance field
pub fn build_regions_watershed(
    chf: &CompactHeightfield,
    boundary_flags: &[u8],
    min_region_area: i32,
    merge_region_area: i32,
) -> Result<Vec<u16>> {
    let span_count = chf.spans.len();

    // Build distance field
    let distances = build_distance_field(chf, boundary_flags)?;

    // Initialize region IDs
    let mut region_ids = vec![0u16; span_count];
    let mut current_region_id = 1u16;

    // Create sorted list of spans by distance (furthest first)
    let mut span_distances: Vec<(usize, u16)> = distances
        .iter()
        .enumerate()
        .filter(|(idx, dist)| {
            // Only include walkable spans with valid distance
            chf.spans[*idx].area != 0 && **dist != 0 && **dist != u16::MAX
        })
        .map(|(idx, dist)| (idx, *dist))
        .collect();

    // Sort by distance (descending - furthest from boundary first)
    span_distances.sort_by(|a, b| b.1.cmp(&a.1));

    // Watershed algorithm: assign regions starting from highest distance points
    for (span_idx, _distance) in span_distances {
        // Skip if already assigned
        if region_ids[span_idx] != 0 {
            continue;
        }

        // Try to merge with existing neighboring region
        let neighbor_region = find_neighbor_region(chf, span_idx, &region_ids)?;

        if neighbor_region != 0 {
            // Merge with existing region using flood fill
            watershed_flood_fill(chf, span_idx, neighbor_region, &distances, &mut region_ids)?;
        } else {
            // Create new region
            watershed_flood_fill(
                chf,
                span_idx,
                current_region_id,
                &distances,
                &mut region_ids,
            )?;
            current_region_id += 1;
        }
    }

    // Post-process regions
    filter_small_regions(&mut region_ids, min_region_area);

    if merge_region_area > 0 {
        merge_small_regions_watershed(chf, &mut region_ids, merge_region_area)?;
    }

    compact_region_ids(&mut region_ids)?;

    Ok(region_ids)
}

/// Finds a neighboring region ID for watershed merging
fn find_neighbor_region(
    chf: &CompactHeightfield,
    span_idx: usize,
    region_ids: &[u16],
) -> Result<u16> {
    // Check all 4 cardinal neighbors for existing regions
    // Using 8-direction constants: N=1, E=3, S=5, W=7
    for dir in [1u8, 3u8, 5u8, 7u8] {
        if let Some(neighbor_idx) = chf.get_neighbor(span_idx, dir) {
            let neighbor_region = region_ids[neighbor_idx];
            if neighbor_region != 0 {
                return Ok(neighbor_region);
            }
        }
    }

    Ok(0) // No neighboring region found
}

/// Watershed flood fill that respects distance gradients
fn watershed_flood_fill(
    chf: &CompactHeightfield,
    seed_idx: usize,
    region_id: u16,
    distances: &[u16],
    region_ids: &mut [u16],
) -> Result<()> {
    let mut queue = VecDeque::new();
    queue.push_back(seed_idx);
    region_ids[seed_idx] = region_id;

    while let Some(span_idx) = queue.pop_front() {
        let current_distance = distances[span_idx];

        // Check all 4 cardinal neighbors
        // Using 8-direction constants: N=1, E=3, S=5, W=7
        for dir in [1u8, 3u8, 5u8, 7u8] {
            if let Some(neighbor_idx) = chf.get_neighbor(span_idx, dir) {
                let neighbor_span = &chf.spans[neighbor_idx];

                // Skip non-walkable or already assigned spans
                if neighbor_span.area == 0 || region_ids[neighbor_idx] != 0 {
                    continue;
                }

                let neighbor_distance = distances[neighbor_idx];

                // Only flow to neighbors at same or lower distance (watershed property)
                if neighbor_distance <= current_distance {
                    region_ids[neighbor_idx] = region_id;
                    queue.push_back(neighbor_idx);
                }
            }
        }
    }

    Ok(())
}

/// Filters out regions smaller than minimum area
fn filter_small_regions(region_ids: &mut [u16], min_region_area: i32) {
    // Count region areas
    let max_region = region_ids.iter().copied().max().unwrap_or(0);
    let mut region_areas = vec![0i32; max_region as usize + 1];

    for &region_id in region_ids.iter() {
        if region_id > 0 && region_id <= max_region {
            region_areas[region_id as usize] += 1;
        }
    }

    // Remove small regions
    for region_id in region_ids.iter_mut() {
        if *region_id > 0 && region_areas[*region_id as usize] < min_region_area {
            *region_id = 0;
        }
    }
}

/// Merges small regions with neighboring regions in watershed algorithm
fn merge_small_regions_watershed(
    chf: &CompactHeightfield,
    region_ids: &mut [u16],
    merge_region_area: i32,
) -> Result<()> {
    // Count region areas
    let max_region = region_ids.iter().copied().max().unwrap_or(0);
    let mut region_areas = vec![0i32; max_region as usize + 1];

    for &region_id in region_ids.iter() {
        if region_id > 0 && region_id <= max_region {
            region_areas[region_id as usize] += 1;
        }
    }

    // Find regions that need merging
    for region_id in 1..=max_region {
        if region_areas[region_id as usize] > 0
            && region_areas[region_id as usize] < merge_region_area
        {
            // Find best neighbor to merge with
            let best_neighbor =
                find_best_merge_neighbor(chf, region_ids, region_id, &region_areas)?;

            if best_neighbor != 0 {
                // Merge regions
                for span_region in region_ids.iter_mut() {
                    if *span_region == region_id {
                        *span_region = best_neighbor;
                    }
                }

                // Update area counts
                region_areas[best_neighbor as usize] += region_areas[region_id as usize];
                region_areas[region_id as usize] = 0;
            }
        }
    }

    Ok(())
}

/// Finds the best neighboring region to merge with
fn find_best_merge_neighbor(
    chf: &CompactHeightfield,
    region_ids: &[u16],
    region_id: u16,
    region_areas: &[i32],
) -> Result<u16> {
    use std::collections::HashMap;

    let mut neighbor_borders: HashMap<u16, i32> = HashMap::new();

    // Count border lengths with each neighbor
    for (span_idx, &span_region) in region_ids.iter().enumerate() {
        if span_region != region_id {
            continue;
        }

        // Check all 4 cardinal neighbors
        // Using 8-direction constants: N=1, E=3, S=5, W=7
        for dir in [1u8, 3u8, 5u8, 7u8] {
            if let Some(neighbor_idx) = chf.get_neighbor(span_idx, dir) {
                let neighbor_region = region_ids[neighbor_idx];

                if neighbor_region != region_id && neighbor_region != 0 {
                    *neighbor_borders.entry(neighbor_region).or_insert(0) += 1;
                }
            }
        }
    }

    // Find neighbor with longest border (prefer larger regions)
    let mut best_neighbor = 0u16;
    let mut best_score = 0i32;

    for (&neighbor_region, &border_length) in &neighbor_borders {
        // Score = border_length * region_area (prefer larger regions with longer borders)
        let neighbor_area = region_areas
            .get(neighbor_region as usize)
            .copied()
            .unwrap_or(0);
        let score = border_length * (neighbor_area + 1); // +1 to avoid zero

        if score > best_score {
            best_score = score;
            best_neighbor = neighbor_region;
        }
    }

    Ok(best_neighbor)
}

/// Compacts region IDs to remove gaps
fn compact_region_ids(region_ids: &mut [u16]) -> Result<()> {
    use std::collections::HashMap;

    // Find all unique region IDs
    let mut unique_regions: Vec<u16> = region_ids.to_vec();
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

    // Apply mapping
    for region_id in region_ids.iter_mut() {
        if let Some(&new_id) = region_mapping.get(region_id) {
            *region_id = new_id;
        }
    }

    Ok(())
}

/// Sweep span structure for monotone region building
#[derive(Debug, Clone, Copy)]
struct SweepSpan {
    /// Row ID
    rid: u16,
    /// Region ID
    id: u16,
    /// Number of samples
    ns: u16,
    /// Neighbor ID
    nei: u16,
}

/// Null neighbor constant
const RC_NULL_NEI: u16 = 0xffff;
/// Border region flag
const RC_BORDER_REG: u16 = 0x8000;

/// Paints a rectangular region with the given ID
fn paint_rect_region(
    min_x: i32,
    max_x: i32,
    min_y: i32,
    max_y: i32,
    region_id: u16,
    chf: &CompactHeightfield,
    src_reg: &mut [u16],
) {
    for y in min_y..max_y {
        for x in min_x..max_x {
            let cell_idx = x + y * chf.width;
            if let Some(cell) = chf.cells.get(cell_idx as usize) {
                if let Some(start_idx) = cell.index {
                    #[allow(clippy::needless_range_loop)]
                    for i in start_idx..start_idx + cell.count {
                        if let Some(span) = chf.spans.get(i) {
                            if span.area != 0 {
                                src_reg[i] = region_id;
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Builds regions using monotone partitioning
pub fn build_regions_monotone(
    chf: &CompactHeightfield,
    border_size: i32,
    min_region_area: i32,
    merge_region_area: i32,
) -> Result<Vec<u16>> {
    let w = chf.width;
    let h = chf.height;
    let mut id = 1u16;

    let mut src_reg = vec![0u16; chf.spans.len()];

    let nsweeps = w.max(h) as usize;
    let mut sweeps = vec![
        SweepSpan {
            rid: 0,
            id: 0,
            ns: 0,
            nei: 0
        };
        nsweeps
    ];

    // Mark border regions
    if border_size > 0 {
        // Make sure border will not overflow
        let bw = w.min(border_size);
        let bh = h.min(border_size);

        // Paint regions
        paint_rect_region(0, bw, 0, h, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
        paint_rect_region(w - bw, w, 0, h, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
        paint_rect_region(0, w, 0, bh, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
        paint_rect_region(0, w, h - bh, h, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
    }

    // border_size is not stored in the heightfield

    let mut prev = vec![0i32; (id + 1) as usize];

    // Sweep one line at a time
    for y in border_size..h - border_size {
        // Collect spans from this row
        prev.resize((id + 1) as usize, 0);
        prev.fill(0);
        let mut rid = 1u16;

        for x in border_size..w - border_size {
            let cell_idx = (x + y * w) as usize;
            if let Some(cell) = chf.cells.get(cell_idx) {
                if let Some(start_idx) = cell.index {
                    for i in start_idx..start_idx + cell.count {
                        if let Some(span) = chf.spans.get(i) {
                            if span.area == 0 {
                                continue;
                            }

                            // Check -x direction
                            let mut previd = 0u16;
                            if let Some(neighbor_idx) = chf.get_neighbor(i, 0) {
                                if (src_reg[neighbor_idx] & RC_BORDER_REG) == 0
                                    && chf.spans[neighbor_idx].area == span.area
                                {
                                    previd = src_reg[neighbor_idx];
                                }
                            }

                            if previd == 0 {
                                previd = rid;
                                rid += 1;
                                if (previd as usize) < sweeps.len() {
                                    sweeps[previd as usize].rid = previd;
                                    sweeps[previd as usize].ns = 0;
                                    sweeps[previd as usize].nei = 0;
                                }
                            }

                            // Check -y direction
                            if let Some(neighbor_idx) = chf.get_neighbor(i, 3) {
                                if src_reg[neighbor_idx] != 0
                                    && (src_reg[neighbor_idx] & RC_BORDER_REG) == 0
                                    && chf.spans[neighbor_idx].area == span.area
                                {
                                    let nr = src_reg[neighbor_idx];
                                    if (previd as usize) < sweeps.len()
                                        && (nr as usize) < prev.len()
                                    {
                                        if sweeps[previd as usize].nei == 0
                                            || sweeps[previd as usize].nei == nr
                                        {
                                            sweeps[previd as usize].nei = nr;
                                            sweeps[previd as usize].ns += 1;
                                            prev[nr as usize] += 1;
                                        } else {
                                            sweeps[previd as usize].nei = RC_NULL_NEI;
                                        }
                                    }
                                }
                            }

                            src_reg[i] = previd;
                        }
                    }
                }
            }
        }

        // Create unique ID
        for i in 1..rid as usize {
            if i < sweeps.len() {
                if sweeps[i].nei != RC_NULL_NEI
                    && sweeps[i].nei != 0
                    && (sweeps[i].nei as usize) < prev.len()
                    && prev[sweeps[i].nei as usize] == sweeps[i].ns as i32
                {
                    sweeps[i].id = sweeps[i].nei;
                } else {
                    sweeps[i].id = id;
                    id += 1;
                }
            }
        }

        // Remap IDs
        for x in border_size..w - border_size {
            let cell_idx = (x + y * w) as usize;
            if let Some(cell) = chf.cells.get(cell_idx) {
                if let Some(start_idx) = cell.index {
                    for i in start_idx..start_idx + cell.count {
                        if src_reg[i] > 0
                            && src_reg[i] < rid
                            && (src_reg[i] as usize) < sweeps.len()
                        {
                            src_reg[i] = sweeps[src_reg[i] as usize].id;
                        }
                    }
                }
            }
        }
    }

    // Filter out small regions
    filter_small_regions(&mut src_reg, min_region_area);

    // Merge small regions if requested
    if merge_region_area > 0 {
        merge_small_regions_monotone(chf, &mut src_reg, merge_region_area)?;
    }

    // Compact region IDs
    compact_region_ids(&mut src_reg)?;

    Ok(src_reg)
}

/// Builds layered regions for better handling of overlapping areas
pub fn build_layer_regions(
    chf: &CompactHeightfield,
    border_size: i32,
    min_region_area: i32,
) -> Result<Vec<u16>> {
    let w = chf.width;
    let h = chf.height;
    let mut id = 1u16;

    let mut src_reg = vec![0u16; chf.spans.len()];

    let nsweeps = w.max(h) as usize;
    let mut sweeps = vec![
        SweepSpan {
            rid: 0,
            id: 0,
            ns: 0,
            nei: 0
        };
        nsweeps
    ];

    // Mark border regions
    if border_size > 0 {
        // Make sure border will not overflow
        let bw = w.min(border_size);
        let bh = h.min(border_size);

        // Paint regions
        paint_rect_region(0, bw, 0, h, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
        paint_rect_region(w - bw, w, 0, h, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
        paint_rect_region(0, w, 0, bh, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
        paint_rect_region(0, w, h - bh, h, id | RC_BORDER_REG, chf, &mut src_reg);
        id += 1;
    }

    // border_size is not stored in the heightfield

    let mut prev = vec![0i32; (id + 1) as usize];

    // Sweep one line at a time
    for y in border_size..h - border_size {
        // Collect spans from this row
        prev.resize((id + 1) as usize, 0);
        prev.fill(0);
        let mut rid = 1u16;

        for x in border_size..w - border_size {
            let cell_idx = (x + y * w) as usize;
            if let Some(cell) = chf.cells.get(cell_idx) {
                if let Some(start_idx) = cell.index {
                    for i in start_idx..start_idx + cell.count {
                        if let Some(span) = chf.spans.get(i) {
                            if span.area == 0 {
                                continue;
                            }

                            // Check -x direction
                            let mut previd = 0u16;
                            if let Some(neighbor_idx) = chf.get_neighbor(i, 0) {
                                if (src_reg[neighbor_idx] & RC_BORDER_REG) == 0
                                    && chf.spans[neighbor_idx].area == span.area
                                {
                                    previd = src_reg[neighbor_idx];
                                }
                            }

                            if previd == 0 {
                                previd = rid;
                                rid += 1;
                                if (previd as usize) < sweeps.len() {
                                    sweeps[previd as usize].rid = previd;
                                    sweeps[previd as usize].ns = 0;
                                    sweeps[previd as usize].nei = 0;
                                }
                            }

                            // Check -y direction
                            if let Some(neighbor_idx) = chf.get_neighbor(i, 3) {
                                if src_reg[neighbor_idx] != 0
                                    && (src_reg[neighbor_idx] & RC_BORDER_REG) == 0
                                    && chf.spans[neighbor_idx].area == span.area
                                {
                                    let nr = src_reg[neighbor_idx];
                                    if (previd as usize) < sweeps.len()
                                        && (nr as usize) < prev.len()
                                    {
                                        if sweeps[previd as usize].nei == 0
                                            || sweeps[previd as usize].nei == nr
                                        {
                                            sweeps[previd as usize].nei = nr;
                                            sweeps[previd as usize].ns += 1;
                                            prev[nr as usize] += 1;
                                        } else {
                                            sweeps[previd as usize].nei = RC_NULL_NEI;
                                        }
                                    }
                                }
                            }

                            src_reg[i] = previd;
                        }
                    }
                }
            }
        }

        // Create unique ID
        for i in 1..rid as usize {
            if i < sweeps.len() {
                if sweeps[i].nei != RC_NULL_NEI
                    && sweeps[i].nei != 0
                    && (sweeps[i].nei as usize) < prev.len()
                    && prev[sweeps[i].nei as usize] == sweeps[i].ns as i32
                {
                    sweeps[i].id = sweeps[i].nei;
                } else {
                    sweeps[i].id = id;
                    id += 1;
                }
            }
        }

        // Remap IDs
        for x in border_size..w - border_size {
            let cell_idx = (x + y * w) as usize;
            if let Some(cell) = chf.cells.get(cell_idx) {
                if let Some(start_idx) = cell.index {
                    for i in start_idx..start_idx + cell.count {
                        if src_reg[i] > 0
                            && src_reg[i] < rid
                            && (src_reg[i] as usize) < sweeps.len()
                        {
                            src_reg[i] = sweeps[src_reg[i] as usize].id;
                        }
                    }
                }
            }
        }
    }

    // Filter and merge regions for layers
    merge_and_filter_layer_regions(chf, &mut src_reg, min_region_area)?;

    Ok(src_reg)
}

/// Merges small regions for monotone region building
fn merge_small_regions_monotone(
    chf: &CompactHeightfield,
    region_ids: &mut [u16],
    merge_region_area: i32,
) -> Result<()> {
    // Similar to merge_small_regions_watershed but for monotone regions
    merge_small_regions_watershed(chf, region_ids, merge_region_area)
}

/// Merges and filters regions for layer building
fn merge_and_filter_layer_regions(
    _chf: &CompactHeightfield,
    region_ids: &mut [u16],
    min_region_area: i32,
) -> Result<()> {
    // First, count region sizes
    let max_region = region_ids.iter().copied().max().unwrap_or(0);
    let mut region_areas = vec![0i32; max_region as usize + 1];

    for &region_id in region_ids.iter() {
        if region_id > 0 && region_id <= max_region {
            region_areas[region_id as usize] += 1;
        }
    }

    // Filter out small regions
    filter_small_regions(region_ids, min_region_area);

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compact_heightfield::CompactHeightfield;
    use crate::heightfield::Heightfield;
    use glam::Vec3;

    #[test]
    fn test_distance_field_generation() {
        // Create a simple compact heightfield
        let width = 5;
        let height = 5;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(5.0, 5.0, 5.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        // Add walkable spans everywhere
        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 5, 1).unwrap();
            }
        }

        let chf = CompactHeightfield::build_from_heightfield(&heightfield).unwrap();
        let boundary_flags = vec![0u8; chf.spans.len()];

        // Mark border spans as boundaries
        let mut boundary_flags = boundary_flags;
        // For simplicity, mark first and last spans as boundaries
        if !boundary_flags.is_empty() {
            boundary_flags[0] = 1;
            let last_idx = boundary_flags.len() - 1;
            boundary_flags[last_idx] = 1;
        }

        let distances = build_distance_field(&chf, &boundary_flags).unwrap();

        // Distance field should have been generated
        assert_eq!(distances.len(), chf.spans.len());

        // Boundary spans should have distance 0
        assert_eq!(distances[0], 0);
        if distances.len() > 1 {
            assert_eq!(distances[distances.len() - 1], 0);
        }
    }

    #[test]
    fn test_monotone_region_building() {
        // Create a simple compact heightfield
        let width = 5;
        let height = 5;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(5.0, 5.0, 5.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        // Add walkable spans everywhere
        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 5, 1).unwrap();
            }
        }

        let chf = CompactHeightfield::build_from_heightfield(&heightfield).unwrap();

        let region_ids = build_regions_monotone(&chf, 0, 1, 0).unwrap();

        // Should have generated regions
        assert_eq!(region_ids.len(), chf.spans.len());

        // At least some spans should be assigned to regions
        let assigned_count = region_ids.iter().filter(|&&id| id > 0).count();
        assert!(
            assigned_count > 0,
            "Some spans should be assigned to regions"
        );
    }

    #[test]
    fn test_layer_region_building() {
        // Create a simple compact heightfield
        let width = 5;
        let height = 5;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(5.0, 5.0, 5.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        // Add walkable spans everywhere
        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 5, 1).unwrap();
            }
        }

        let chf = CompactHeightfield::build_from_heightfield(&heightfield).unwrap();

        let region_ids = build_layer_regions(&chf, 0, 1).unwrap();

        // Should have generated regions
        assert_eq!(region_ids.len(), chf.spans.len());

        // At least some spans should be assigned to regions
        let assigned_count = region_ids.iter().filter(|&&id| id > 0).count();
        assert!(
            assigned_count > 0,
            "Some spans should be assigned to regions"
        );
    }

    #[test]
    fn test_watershed_region_building() {
        // Create a simple compact heightfield
        let width = 5;
        let height = 5;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(5.0, 5.0, 5.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);

        // Add walkable spans everywhere
        for z in 0..height {
            for x in 0..width {
                heightfield.add_span(x, z, 0, 5, 1).unwrap();
            }
        }

        let chf = CompactHeightfield::build_from_heightfield(&heightfield).unwrap();

        // Create proper boundary flags (mark edge spans as boundaries)
        let mut boundary_flags = vec![0u8; chf.spans.len()];

        // For testing, mark first and last few spans as boundaries to create distance gradient
        let boundary_count = chf.spans.len().min(4);
        for (i, span) in chf.spans.iter().enumerate().take(boundary_count) {
            if span.area != 0 {
                boundary_flags[i] = 1;
            }
        }

        // Also mark some at the end
        let start_idx = chf.spans.len().saturating_sub(boundary_count);
        for (i, span) in chf.spans.iter().enumerate().skip(start_idx) {
            if span.area != 0 {
                boundary_flags[i] = 1;
            }
        }

        let region_ids = build_regions_watershed(&chf, &boundary_flags, 1, 0).unwrap();

        // Should have generated regions
        assert_eq!(region_ids.len(), chf.spans.len());

        // Interior walkable spans should be assigned to regions (boundaries may not be)
        let mut assigned_count = 0;
        for (idx, &region_id) in region_ids.iter().enumerate() {
            if chf.spans[idx].area != 0 && boundary_flags[idx] == 0 && region_id > 0 {
                assigned_count += 1;
            }
        }

        // At least some interior spans should be assigned
        assert!(
            assigned_count > 0,
            "Some interior spans should be assigned to regions"
        );
    }
}
