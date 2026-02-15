//! Watershed partitioning algorithm for region building
//! Following the exact C++ implementation from RecastRegion.cpp

use super::compact_heightfield::CompactHeightfield;
use crate::error::BuildError;

/// Border region flag (matches C++ RC_BORDER_REG)
pub const RC_BORDER_REG: u16 = 0x8000;

/// Null area constant
pub const RC_NULL_AREA: u8 = 0;

/// Not connected constant
pub const RC_NOT_CONNECTED: usize = 63;

/// Level stack entry for watershed algorithm
#[derive(Debug, Clone, Copy)]
struct LevelStackEntry {
    x: i32,
    y: i32,
    index: usize,
}

impl LevelStackEntry {
    fn new(x: i32, y: i32, index: usize) -> Self {
        Self { x, y, index }
    }
}

/// Region structure for merging
#[allow(dead_code)]
struct Region {
    span_count: i32,
    id: u16,
    area_type: u8,
    remap: bool,
    visited: bool,
    overlap: bool,
    connects_to_border: bool,
    ymin: u16,
    ymax: u16,
    connections: Vec<usize>,
    floors: Vec<usize>,
}

impl Region {
    fn new(id: u16) -> Self {
        Self {
            span_count: 0,
            id,
            area_type: 0,
            remap: false,
            visited: false,
            overlap: false,
            connects_to_border: false,
            ymin: 0xffff,
            ymax: 0,
            connections: Vec::new(),
            floors: Vec::new(),
        }
    }
}

/// Sorts cells by level for watershed algorithm
fn sort_cells_by_level(
    start_level: u16,
    chf: &CompactHeightfield,
    src_reg: &[u16],
    nb_stacks: usize,
    stacks: &mut [Vec<LevelStackEntry>],
    log_levels_per_stack: u16,
) {
    let w = chf.width;
    let h = chf.height;
    let start_level = start_level >> log_levels_per_stack;

    // Clear all stacks
    for stack in stacks.iter_mut() {
        stack.clear();
    }

    // Put all cells in the level range into the appropriate stacks
    let mut _total_added = 0;
    let mut _skipped_null_area = 0;
    let mut _skipped_has_region = 0;
    let mut _skipped_out_of_range = 0;

    for y in 0..h {
        for x in 0..w {
            let cell_idx = (y * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(first_span_idx) = cell.index {
                for s in 0..cell.count {
                    let i = first_span_idx + s;

                    if chf.areas[i] == RC_NULL_AREA {
                        _skipped_null_area += 1;
                        continue;
                    }
                    if src_reg[i] != 0 {
                        _skipped_has_region += 1;
                        continue;
                    }

                    let level = chf.dist[i] >> log_levels_per_stack;
                    let s_id = start_level.saturating_sub(level) as usize;

                    if s_id < nb_stacks {
                        stacks[s_id].push(LevelStackEntry::new(x, y, i));
                        _total_added += 1;
                    } else {
                        _skipped_out_of_range += 1;
                    }
                }
            }
        }
    }
}

/// Appends one stack to another, filtering out assigned regions
fn append_stacks(
    src_stack: &[LevelStackEntry],
    dst_stack: &mut Vec<LevelStackEntry>,
    src_reg: &[u16],
) {
    for &entry in src_stack {
        if entry.index < src_reg.len() && src_reg[entry.index] == 0 {
            dst_stack.push(entry);
        }
    }
}

/// Resolves a cell index for a neighbor in the given direction.
///
/// When `span.con[dir]` indicates a connection exists, the neighbor cell's index
/// should always be `Some`. A `None` value here would indicate a malformed compact
/// heightfield. Returns `None` in that case so callers can skip the neighbor.
fn resolve_neighbor_index(
    chf: &CompactHeightfield,
    x: i32,
    y: i32,
    dir: usize,
    con_offset: usize,
) -> Option<usize> {
    let w = chf.width;
    let ax = x + chf.get_dir_offset_x(dir);
    let ay = y + chf.get_dir_offset_z(dir);
    let cell = &chf.cells[(ay * w + ax) as usize];
    Some(cell.index? + con_offset)
}

/// Flood fills a region starting from a seed
#[allow(clippy::too_many_arguments)]
fn flood_region(
    x: i32,
    y: i32,
    i: usize,
    level: u16,
    r: u16,
    chf: &CompactHeightfield,
    src_reg: &mut [u16],
    src_dist: &mut [u16],
    stack: &mut Vec<LevelStackEntry>,
) -> bool {
    let area = chf.areas[i];

    // Flood fill mark region
    stack.clear();
    stack.push(LevelStackEntry::new(x, y, i));
    src_reg[i] = r;
    src_dist[i] = 0;

    let lev = level.saturating_sub(2);
    let mut count = 0;

    while let Some(back) = stack.pop() {
        let cx = back.x;
        let cy = back.y;
        let ci = back.index;

        let span = &chf.spans[ci];

        // Check if any of the neighbours already have a valid region set
        let mut ar = 0u16;

        // Check all 4 directions
        for dir in 0..4 {
            if span.con[dir] != RC_NOT_CONNECTED {
                let Some(ai) = resolve_neighbor_index(chf, cx, cy, dir, span.con[dir]) else {
                    continue;
                };

                if chf.areas[ai] != area {
                    continue;
                }

                let nr = src_reg[ai];
                if (nr & RC_BORDER_REG) != 0 {
                    // Do not take borders into account
                    continue;
                }

                if nr != 0 && nr != r {
                    ar = nr;
                    break;
                }

                // Check diagonal
                let neighbor_span = &chf.spans[ai];
                let dir2 = (dir + 1) & 0x3;

                if neighbor_span.con[dir2] != RC_NOT_CONNECTED {
                    let ax = cx + chf.get_dir_offset_x(dir);
                    let ay = cy + chf.get_dir_offset_z(dir);
                    let Some(ai2) =
                        resolve_neighbor_index(chf, ax, ay, dir2, neighbor_span.con[dir2])
                    else {
                        continue;
                    };

                    if chf.areas[ai2] != area {
                        continue;
                    }

                    let nr2 = src_reg[ai2];
                    if nr2 != 0 && nr2 != r {
                        ar = nr2;
                        break;
                    }
                }
            }
        }

        if ar != 0 {
            src_reg[ci] = 0;
            continue;
        }

        count += 1;

        // Expand neighbours
        for dir in 0..4 {
            if span.con[dir] != RC_NOT_CONNECTED {
                let Some(ai) = resolve_neighbor_index(chf, cx, cy, dir, span.con[dir]) else {
                    continue;
                };

                if chf.areas[ai] != area {
                    continue;
                }

                if chf.dist[ai] >= lev && src_reg[ai] == 0 {
                    src_reg[ai] = r;
                    src_dist[ai] = 0;
                    stack.push(LevelStackEntry::new(
                        cx + chf.get_dir_offset_x(dir),
                        cy + chf.get_dir_offset_z(dir),
                        ai,
                    ));
                }
            }
        }
    }

    count > 0
}

/// Expands regions by assigning unassigned cells to nearby existing regions.
///
/// Matches C++ `expandRegions`. Items are marked with `usize::MAX` when assigned
/// but kept in the stack (not removed) so the termination check works correctly.
fn expand_regions(
    max_iter: i32,
    level: u16,
    chf: &CompactHeightfield,
    src_reg: &mut [u16],
    src_dist: &mut [u16],
    stack: &mut Vec<LevelStackEntry>,
    fill_stack: bool,
) {
    let w = chf.width;
    let h = chf.height;

    if fill_stack {
        // Find cells revealed by the raised level
        stack.clear();
        for y in 0..h {
            for x in 0..w {
                let cell_idx = (y * w + x) as usize;
                let cell = &chf.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let i = first_span_idx + s;
                        if chf.dist[i] >= level && src_reg[i] == 0 && chf.areas[i] != RC_NULL_AREA {
                            stack.push(LevelStackEntry::new(x, y, i));
                        }
                    }
                }
            }
        }
    } else {
        // Mark all cells which already have a region
        for j in 0..stack.len() {
            if src_reg[stack[j].index] != 0 {
                stack[j] = LevelStackEntry::new(stack[j].x, stack[j].y, usize::MAX);
            }
        }
    }

    let mut dirty_entries = Vec::new();
    let mut iter = 0;

    while !stack.is_empty() {
        let mut failed = 0;
        dirty_entries.clear();

        for j in 0..stack.len() {
            let x = stack[j].x;
            let y = stack[j].y;
            let i = stack[j].index;

            if i == usize::MAX {
                failed += 1;
                continue;
            }

            let mut r = src_reg[i];
            let mut d2 = 0xffffu16;
            let area = chf.areas[i];
            let span = &chf.spans[i];

            // Check all 4 directions
            for dir in 0..4 {
                if span.con[dir] == RC_NOT_CONNECTED {
                    continue;
                }

                let Some(ai) = resolve_neighbor_index(chf, x, y, dir, span.con[dir]) else {
                    continue;
                };

                if chf.areas[ai] != area {
                    continue;
                }

                if src_reg[ai] > 0 && (src_reg[ai] & RC_BORDER_REG) == 0 {
                    if src_dist[ai] + 2 < d2 {
                        r = src_reg[ai];
                        d2 = src_dist[ai] + 2;
                    }
                }
            }

            if r != 0 {
                stack[j] = LevelStackEntry::new(x, y, usize::MAX); // mark as used
                dirty_entries.push((i, r, d2));
            } else {
                failed += 1;
            }
        }

        // Copy results back
        for &(idx, reg, dist) in &dirty_entries {
            src_reg[idx] = reg;
            src_dist[idx] = dist;
        }

        if failed == stack.len() {
            break;
        }

        if level > 0 {
            iter += 1;
            if iter >= max_iter {
                break;
            }
        }
    }
}

/// Builds regions using watershed partitioning
pub fn build_regions_watershed(
    chf: &mut CompactHeightfield,
    border_size: i32,
    min_region_area: i32,
    merge_region_area: i32,
) -> Result<(), BuildError> {
    let w = chf.width;
    let h = chf.height;
    let span_count = chf.spans.len();

    // Build distance field
    chf.build_distance_field()?;

    // Allocate working buffers
    let mut src_reg = vec![0u16; span_count];
    let mut src_dist = vec![0u16; span_count];

    const LOG_NB_STACKS: u16 = 3;
    const NB_STACKS: usize = 1 << LOG_NB_STACKS;
    let mut lvl_stacks: [Vec<LevelStackEntry>; NB_STACKS] = Default::default();
    let mut stack = Vec::with_capacity(256);

    let mut region_id = 1u16;
    // C++ uses unsigned wrapping: (maxDist+1) & ~1. Use wrapping_add for same behavior.
    let mut level = chf.max_distance.wrapping_add(1) & !1;

    // Expansion iterations - controls how much watershed "overflows"
    let expand_iters = 8;

    // Mark border regions
    if border_size > 0 {
        let bw = w.min(border_size);
        let bh = h.min(border_size);

        // Paint border regions
        paint_rect_region(0, bw, 0, h, region_id | RC_BORDER_REG, chf, &mut src_reg)?;
        region_id += 1;
        paint_rect_region(
            w - bw,
            w,
            0,
            h,
            region_id | RC_BORDER_REG,
            chf,
            &mut src_reg,
        )?;
        region_id += 1;
        paint_rect_region(0, w, 0, bh, region_id | RC_BORDER_REG, chf, &mut src_reg)?;
        region_id += 1;
        paint_rect_region(
            0,
            w,
            h - bh,
            h,
            region_id | RC_BORDER_REG,
            chf,
            &mut src_reg,
        )?;
        region_id += 1;
    }

    let mut s_id = -1i32;

    // Debug output

    // Main watershed loop
    while level > 0 {
        level = level.saturating_sub(2);
        s_id = (s_id + 1) & (NB_STACKS as i32 - 1);

        if s_id == 0 {
            sort_cells_by_level(level, chf, &src_reg, NB_STACKS, &mut lvl_stacks, 1);
        } else {
            // Copy leftovers from last level
            let (left, right) = lvl_stacks.split_at_mut(s_id as usize);
            append_stacks(&left[s_id as usize - 1], &mut right[0], &src_reg);
        }

        // Expand current regions
        expand_regions(
            expand_iters,
            level,
            chf,
            &mut src_reg,
            &mut src_dist,
            &mut lvl_stacks[s_id as usize],
            false,
        );

        // Mark new regions with IDs
        for j in 0..lvl_stacks[s_id as usize].len() {
            let current = lvl_stacks[s_id as usize][j];
            let x = current.x;
            let y = current.y;
            let i = current.index;

            if i < src_reg.len()
                && src_reg[i] == 0
                && flood_region(
                    x,
                    y,
                    i,
                    level,
                    region_id,
                    chf,
                    &mut src_reg,
                    &mut src_dist,
                    &mut stack,
                )
            {
                if region_id == 0xFFFF {
                    return Err(BuildError::RegionIdOverflow.into());
                }
                region_id += 1;
            }
        }
    }

    // Expand current regions until no empty connected cells found
    expand_regions(
        expand_iters * 8,
        0,
        chf,
        &mut src_reg,
        &mut src_dist,
        &mut stack,
        true,
    );

    // Store max regions
    chf.max_regions = region_id;

    // Filter and merge regions
    merge_and_filter_regions(
        min_region_area,
        merge_region_area,
        region_id,
        chf,
        &mut src_reg,
    )?;

    // Write results back to spans
    for (i, span) in chf.spans.iter_mut().enumerate() {
        span.reg = src_reg[i];
    }

    Ok(())
}

/// Paints a rectangular region
fn paint_rect_region(
    min_x: i32,
    max_x: i32,
    min_y: i32,
    max_y: i32,
    region_id: u16,
    chf: &CompactHeightfield,
    region_ids: &mut [u16],
) -> Result<(), BuildError> {
    for y in min_y..max_y {
        for x in min_x..max_x {
            let cell_idx = (y * chf.width + x) as usize;
            if cell_idx < chf.cells.len() {
                let cell = &chf.cells[cell_idx];

                if let Some(first_span_idx) = cell.index {
                    for s in 0..cell.count {
                        let span_idx = first_span_idx + s;
                        if span_idx < region_ids.len() && chf.areas[span_idx] != RC_NULL_AREA {
                            region_ids[span_idx] = region_id;
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

/// Merges and filters regions, matching C++ `mergeAndFilterRegions`.
fn merge_and_filter_regions(
    min_region_area: i32,
    merge_region_area: i32,
    max_region_id: u16,
    chf: &CompactHeightfield,
    src_reg: &mut [u16],
) -> Result<(), BuildError> {
    let w = chf.width;
    let h = chf.height;
    let nreg = max_region_id as usize + 1;

    // Construct region table
    let mut regions = Vec::with_capacity(nreg);
    for i in 0..nreg {
        regions.push(Region::new(i as u16));
    }

    // Find edge of a region and find connections around the contour.
    for y in 0..h {
        for x in 0..w {
            let cell_idx = (y * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(first_idx) = cell.index {
                for s in 0..cell.count {
                    let i = first_idx + s;
                    let r = src_reg[i] as usize;
                    if r == 0 || r >= nreg {
                        continue;
                    }

                    let reg = &mut regions[r];
                    reg.span_count += 1;

                    // Update floors (other regions at the same cell, different height)
                    for s2 in 0..cell.count {
                        let j = first_idx + s2;
                        if i == j {
                            continue;
                        }
                        let floor_id = src_reg[j] as usize;
                        if floor_id == 0 || floor_id >= nreg {
                            continue;
                        }
                        if floor_id == r {
                            reg.overlap = true;
                        }
                        if !reg.floors.contains(&floor_id) {
                            reg.floors.push(floor_id);
                        }
                    }

                    // Have found contour already for this region
                    if !reg.connections.is_empty() {
                        continue;
                    }

                    reg.area_type = chf.areas[i];

                    // Check if this cell is next to a solid edge (region boundary)
                    let mut ndir = -1i32;
                    for dir in 0..4 {
                        if is_solid_edge(chf, src_reg, x, y, i, dir) {
                            ndir = dir as i32;
                            break;
                        }
                    }

                    if ndir != -1 {
                        // Walk around the contour to find all the neighbours
                        walk_contour(x, y, i, ndir as usize, chf, src_reg, &mut reg.connections);
                    }
                }
            }
        }
    }

    // Remove too small regions using flood-fill through connected regions (C++ approach)
    for i in 0..nreg {
        if regions[i].id == 0 || (regions[i].id & RC_BORDER_REG) != 0 {
            continue;
        }
        if regions[i].span_count == 0 {
            continue;
        }
        if regions[i].visited {
            continue;
        }

        // Flood-fill through connected regions to accumulate total span count
        let mut connects_to_border = false;
        let mut span_count = 0i32;
        let mut stack = Vec::new();
        let mut trace = Vec::new();

        regions[i].visited = true;
        stack.push(i);

        while let Some(ri) = stack.pop() {
            span_count += regions[ri].span_count;
            trace.push(ri);

            for j in 0..regions[ri].connections.len() {
                let conn = regions[ri].connections[j];
                if conn & (RC_BORDER_REG as usize) != 0 {
                    connects_to_border = true;
                    continue;
                }
                if conn >= nreg {
                    continue;
                }
                if regions[conn].visited {
                    continue;
                }
                if regions[conn].id == 0 || (regions[conn].id & RC_BORDER_REG) != 0 {
                    continue;
                }
                stack.push(conn);
                regions[conn].visited = true;
            }
        }

        // If the accumulated regions size is too small, remove the group.
        // Don't remove areas connected to tile borders.
        if span_count < min_region_area && !connects_to_border {
            for &ri in &trace {
                regions[ri].span_count = 0;
                regions[ri].id = 0;
            }
        }
    }

    // Apply region removals to spans
    for reg in src_reg.iter_mut() {
        let r = *reg as usize;
        if r > 0 && r < nreg && regions[r].id == 0 {
            *reg = 0;
        }
    }

    // Merge regions if requested
    if merge_region_area > 0 {
        merge_small_regions(&mut regions, min_region_area, merge_region_area, src_reg)?;
    }

    // Compact region IDs
    compact_region_ids(&regions, src_reg)?;

    Ok(())
}

/// Checks if a span edge in a given direction is a region boundary.
///
/// Matches C++ `isSolidEdge`.
fn is_solid_edge(
    chf: &CompactHeightfield,
    src_reg: &[u16],
    x: i32,
    y: i32,
    i: usize,
    dir: usize,
) -> bool {
    let span = &chf.spans[i];
    let mut r = 0u16;
    if span.con[dir] != RC_NOT_CONNECTED {
        if let Some(ai) = resolve_neighbor_index(chf, x, y, dir, span.con[dir]) {
            r = src_reg[ai];
        }
    }
    r != src_reg[i]
}

/// Walks around the contour of a region to find all neighboring region IDs.
///
/// Matches C++ `walkContour`. Traces the boundary clockwise, recording each
/// neighboring region ID as it changes. Removes adjacent duplicates.
fn walk_contour(
    mut x: i32,
    mut y: i32,
    mut i: usize,
    mut dir: usize,
    chf: &CompactHeightfield,
    src_reg: &[u16],
    cont: &mut Vec<usize>,
) {
    let start_dir = dir;
    let start_i = i;

    let span = &chf.spans[i];
    let mut cur_reg = 0u16;
    if span.con[dir] != RC_NOT_CONNECTED {
        if let Some(ai) = resolve_neighbor_index(chf, x, y, dir, span.con[dir]) {
            cur_reg = src_reg[ai];
        }
    }
    cont.push(cur_reg as usize);

    let mut iter = 0;
    while iter < 40000 {
        iter += 1;
        let span = &chf.spans[i];

        if is_solid_edge(chf, src_reg, x, y, i, dir) {
            // At a region boundary edge
            let mut r = 0u16;
            if span.con[dir] != RC_NOT_CONNECTED {
                if let Some(ai) = resolve_neighbor_index(chf, x, y, dir, span.con[dir]) {
                    r = src_reg[ai];
                }
            }
            if r != cur_reg {
                cur_reg = r;
                cont.push(cur_reg as usize);
            }

            dir = (dir + 1) & 0x3; // Rotate CW
        } else {
            // Move to neighbor cell
            let nx = x + chf.get_dir_offset_x(dir);
            let nz = y + chf.get_dir_offset_z(dir);
            if span.con[dir] == RC_NOT_CONNECTED {
                return; // Should not happen
            }
            let Some(ni) = resolve_neighbor_index(chf, x, y, dir, span.con[dir]) else {
                return;
            };
            x = nx;
            y = nz;
            i = ni;
            dir = (dir + 3) & 0x3; // Rotate CCW
        }

        if start_i == i && start_dir == dir {
            break;
        }
    }

    // Remove adjacent duplicates
    if cont.len() > 1 {
        let mut j = 0;
        while j < cont.len() {
            let nj = (j + 1) % cont.len();
            if cont[j] == cont[nj] {
                cont.remove(j);
            } else {
                j += 1;
            }
        }
    }
}

/// Checks if a region is connected to the border (has a neighbor with ID 0).
///
/// Matches C++ `isRegionConnectedToBorder`.
fn is_region_connected_to_border(reg: &Region) -> bool {
    for &conn in &reg.connections {
        if conn == 0 {
            return true;
        }
    }
    false
}

/// Replaces all occurrences of `old_id` with `new_id` in a region's connection
/// and floor lists, then removes adjacent duplicates.
///
/// Matches C++ `replaceNeighbour`.
fn replace_neighbour(reg: &mut Region, old_id: u16, new_id: u16) {
    let mut nei_changed = false;
    for conn in &mut reg.connections {
        if *conn == old_id as usize {
            *conn = new_id as usize;
            nei_changed = true;
        }
    }
    for floor in &mut reg.floors {
        if *floor == old_id as usize {
            *floor = new_id as usize;
        }
    }
    if nei_changed {
        remove_adjacent_neighbours(reg);
    }
}

/// Checks if region `a` can be merged with region `b`.
///
/// Matches C++ `canMergeWithRegion`. Returns false if:
/// - Area types differ
/// - Regions share more than 1 connection edge (would create non-simple region)
/// - Region `b` appears in region `a`'s floor list (vertical overlap)
fn can_merge_with_region(a: &Region, b: &Region) -> bool {
    if a.area_type != b.area_type {
        return false;
    }
    // Count how many times b.id appears in a's connections
    let n = a
        .connections
        .iter()
        .filter(|&&c| c == b.id as usize)
        .count();
    if n > 1 {
        return false;
    }
    // Check floors — if b is a floor of a, they overlap vertically
    if a.floors.contains(&(b.id as usize)) {
        return false;
    }
    true
}

/// Removes adjacent duplicate entries from a region's connection list.
///
/// Matches C++ `removeAdjacentNeighbours`.
fn remove_adjacent_neighbours(reg: &mut Region) {
    let mut i = 0;
    while i < reg.connections.len() && reg.connections.len() > 1 {
        let ni = if i + 1 < reg.connections.len() {
            i + 1
        } else {
            0
        };
        if reg.connections[i] == reg.connections[ni] {
            // Remove the duplicate at position ni
            reg.connections.remove(ni);
            // Don't advance i since we need to recheck at the same position
            if ni == 0 {
                // Removed the first element, adjust i
                if i > 0 {
                    i -= 1;
                }
            }
        } else {
            i += 1;
        }
    }
}

/// Adds a unique floor region ID.
fn add_unique_floor_region(reg: &mut Region, floor_id: usize) {
    if !reg.floors.contains(&floor_id) {
        reg.floors.push(floor_id);
    }
}

/// Merges region `b` into region `a` by splicing connection lists.
///
/// Matches C++ `mergeRegions`. Returns true if the merge succeeded.
fn merge_regions(a: &mut Region, b: &mut Region) -> bool {
    let aid = a.id as usize;
    let bid = b.id as usize;

    // Find insertion point on A (where B appears in A's connections)
    let insa = a.connections.iter().position(|&c| c == bid);
    let Some(insa) = insa else {
        return false;
    };

    // Find insertion point on B (where A appears in B's connections)
    let insb = b.connections.iter().position(|&c| c == aid);
    let Some(insb) = insb else {
        return false;
    };

    // Duplicate A's connections
    let acon = a.connections.clone();
    let bcon = &b.connections;

    // Build merged connection list
    a.connections.clear();

    // Add A's connections starting after the insertion point, skipping the B entry
    let na = acon.len();
    for i in 0..na.saturating_sub(1) {
        a.connections.push(acon[(insa + 1 + i) % na]);
    }

    // Add B's connections starting after the insertion point, skipping the A entry
    let nb = bcon.len();
    for i in 0..nb.saturating_sub(1) {
        a.connections.push(bcon[(insb + 1 + i) % nb]);
    }

    remove_adjacent_neighbours(a);

    // Merge floors
    for &floor in &b.floors {
        add_unique_floor_region(a, floor);
    }

    a.span_count += b.span_count;
    b.span_count = 0;
    b.connections.clear();

    true
}

/// Merges small regions with neighbors.
///
/// Matches the C++ merge loop in `mergeAndFilterRegions`:
/// - Checks `canMergeWithRegion` before merging (area type, connection count, floors)
/// - Uses `mergeRegions` for proper connection topology splicing
/// - Selects the smallest neighbor (C++ behavior)
fn merge_small_regions(
    regions: &mut [Region],
    _min_region_area: i32,
    merge_region_area: i32,
    src_reg: &mut [u16],
) -> Result<(), BuildError> {
    let nreg = regions.len();

    loop {
        let mut merge_count = 0;

        for i in 0..nreg {
            if regions[i].id == 0 || (regions[i].id & RC_BORDER_REG) != 0 {
                continue;
            }
            if regions[i].overlap {
                continue;
            }
            if regions[i].span_count == 0 {
                continue;
            }

            // C++ condition: skip if large AND connected to border.
            if regions[i].span_count > merge_region_area
                && is_region_connected_to_border(&regions[i])
            {
                continue;
            }

            // Find smallest neighbor that can be merged (C++ behavior).
            let mut smallest = i32::MAX;
            let mut merge_id = regions[i].id;

            for j in 0..regions[i].connections.len() {
                let conn = regions[i].connections[j];
                if conn & (RC_BORDER_REG as usize) != 0 {
                    continue;
                }
                if conn >= nreg {
                    continue;
                }
                let mreg = &regions[conn];
                if mreg.id == 0 || (mreg.id & RC_BORDER_REG) != 0 || mreg.overlap {
                    continue;
                }
                if mreg.span_count < smallest
                    && can_merge_with_region(&regions[i], mreg)
                    && can_merge_with_region(mreg, &regions[i])
                {
                    smallest = mreg.span_count;
                    merge_id = mreg.id;
                }
            }

            if merge_id != regions[i].id {
                let old_id = regions[i].id;
                let target_idx = merge_id as usize;

                // Merge using proper connection topology splicing
                // Split borrows: we need mutable access to two different elements
                let (target, source) = if target_idx < i {
                    let (left, right) = regions.split_at_mut(i);
                    (&mut left[target_idx], &mut right[0])
                } else {
                    let (left, right) = regions.split_at_mut(target_idx);
                    (&mut left[i], &mut right[0])
                };

                // The merge function expects (target, source) where target = merge_id, source = i
                let merged = if target_idx < i {
                    merge_regions(target, source)
                } else {
                    merge_regions(source, target)
                };

                if merged {
                    // Fixup regions pointing to current region
                    for j in 0..nreg {
                        if regions[j].id == 0 || (regions[j].id & RC_BORDER_REG) != 0 {
                            continue;
                        }
                        if regions[j].id == old_id {
                            regions[j].id = merge_id;
                        }
                        replace_neighbour(&mut regions[j], old_id, merge_id);
                    }

                    // Update spans
                    for reg in src_reg.iter_mut() {
                        if *reg == old_id {
                            *reg = merge_id;
                        }
                    }

                    merge_count += 1;
                }
            }
        }

        if merge_count == 0 {
            break;
        }
    }

    Ok(())
}

/// Compacts region IDs to remove gaps
fn compact_region_ids(regions: &[Region], src_reg: &mut [u16]) -> Result<(), BuildError> {
    // Build remapping table
    let mut remap = vec![0u16; regions.len()];
    let mut new_id = 1u16;

    for (i, reg) in regions.iter().enumerate() {
        if reg.id == 0 {
            continue;
        }

        if (reg.id & RC_BORDER_REG) != 0 {
            remap[i] = reg.id;
        } else {
            remap[i] = new_id;
            new_id += 1;
        }
    }

    // Remap regions
    for span_reg in src_reg.iter_mut() {
        let reg_id = *span_reg;
        if reg_id < remap.len() as u16 {
            *span_reg = remap[reg_id as usize];
        }
    }

    Ok(())
}
