//! Watershed partitioning algorithm for region building
//! Following the exact C++ implementation from RecastRegion.cpp

use super::compact_heightfield::CompactHeightfield;
use recast_common::{Error, Result};

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
    let w = chf.width;
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
                let ax = cx + chf.get_dir_offset_x(dir);
                let ay = cy + chf.get_dir_offset_y(dir);
                let ai = chf.cells[(ay * w + ax) as usize].index.unwrap() + span.con[dir];

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
                    let ax2 = ax + chf.get_dir_offset_x(dir2);
                    let ay2 = ay + chf.get_dir_offset_y(dir2);
                    let ai2 = chf.cells[(ay2 * w + ax2) as usize].index.unwrap()
                        + neighbor_span.con[dir2];

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
                let ax = cx + chf.get_dir_offset_x(dir);
                let ay = cy + chf.get_dir_offset_y(dir);
                let ai = chf.cells[(ay * w + ax) as usize].index.unwrap() + span.con[dir];

                if chf.areas[ai] != area {
                    continue;
                }

                if chf.dist[ai] >= lev && src_reg[ai] == 0 {
                    src_reg[ai] = r;
                    src_dist[ai] = 0;
                    stack.push(LevelStackEntry::new(ax, ay, ai));
                }
            }
        }
    }

    count > 0
}

/// Expands regions
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

    let mut iter = 0;
    let mut dirty_entries = Vec::new();

    while !stack.is_empty() {
        let mut failed = 0;
        dirty_entries.clear();
        let mut mark_indices = Vec::new();

        for (j, entry) in stack.iter().enumerate() {
            let x = entry.x;
            let y = entry.y;
            let i = entry.index;

            if i == usize::MAX {
                failed += 1;
                continue;
            }

            let mut r = src_reg[i];
            let mut d2 = 0xffff;
            let area = chf.areas[i];
            let span = &chf.spans[i];

            // Check all 4 directions
            for dir in 0..4 {
                if span.con[dir] == RC_NOT_CONNECTED {
                    continue;
                }

                let ax = x + chf.get_dir_offset_x(dir);
                let ay = y + chf.get_dir_offset_y(dir);
                let ai = chf.cells[(ay * w + ax) as usize].index.unwrap() + span.con[dir];

                if chf.areas[ai] != area {
                    continue;
                }

                if src_reg[ai] > 0 && (src_reg[ai] & RC_BORDER_REG) == 0 && src_dist[ai] + 2 < d2 {
                    r = src_reg[ai];
                    d2 = src_dist[ai] + 2;
                }
            }

            if r != 0 {
                // Mark as assigned and add to dirty list
                mark_indices.push(j);
                dirty_entries.push((i, r, d2));
            } else {
                failed += 1;
            }
        }

        // Update marked entries
        for j in mark_indices {
            stack[j] = LevelStackEntry::new(stack[j].x, stack[j].y, usize::MAX);
        }

        // Copy results back
        for &(idx, reg, dist) in &dirty_entries {
            src_reg[idx] = reg;
            src_dist[idx] = dist;
        }

        // Remove processed items
        stack.retain(|e| e.index != usize::MAX);

        if failed * 3 >= stack.len() * 2 {
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
) -> Result<()> {
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
    let mut level = (chf.max_distance + 1) & !1;

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
                    return Err(Error::Recast("Region ID overflow".to_string()));
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
) -> Result<()> {
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

/// Merges and filters regions
fn merge_and_filter_regions(
    min_region_area: i32,
    merge_region_area: i32,
    max_region_id: u16,
    chf: &CompactHeightfield,
    src_reg: &mut [u16],
) -> Result<()> {
    let span_count = chf.spans.len();

    // Construct region table
    let mut regions = Vec::with_capacity(max_region_id as usize);
    for i in 0..max_region_id {
        regions.push(Region::new(i));
    }

    // Find region connections and overlapping regions
    let _overlaps: Vec<u16> = Vec::new();
    for span_idx in 0..span_count {
        let reg_id = src_reg[span_idx];
        if reg_id == 0 || (reg_id & RC_BORDER_REG) != 0 {
            continue;
        }

        let reg = &mut regions[reg_id as usize];
        reg.span_count += 1;

        // Update region bounds
        let y = chf.spans[span_idx].y as u16;
        reg.ymin = reg.ymin.min(y);
        reg.ymax = reg.ymax.max(y);

        // Collect neighbors
        for dir in 0..4 {
            if let Some(neighbor_idx) = chf.get_neighbor_connection(span_idx, dir) {
                let neighbor_reg = src_reg[neighbor_idx];
                if neighbor_reg == reg_id || neighbor_reg == 0 {
                    continue;
                }

                if (neighbor_reg & RC_BORDER_REG) != 0 {
                    reg.connects_to_border = true;
                    continue;
                }

                // Add neighbor connection
                if !reg.connections.contains(&(neighbor_reg as usize)) {
                    reg.connections.push(neighbor_reg as usize);
                }
            }
        }
    }

    // Remove small regions
    let mut regions_to_remove = Vec::new();
    for (i, reg) in regions.iter().enumerate().take(max_region_id as usize) {
        if reg.id == 0 || (reg.id & RC_BORDER_REG) != 0 {
            continue;
        }

        if reg.span_count == 0 {
            continue;
        }

        if reg.span_count < min_region_area && !reg.connects_to_border {
            // Mark for removal
            regions_to_remove.push((i, reg.id));
        }
    }

    // Apply removals
    for (i, old_id) in regions_to_remove {
        regions[i].id = 0;

        // Reassign all spans
        for reg in src_reg.iter_mut().take(span_count) {
            if *reg == old_id {
                *reg = 0;
            }
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

/// Merges small regions with neighbors
fn merge_small_regions(
    regions: &mut [Region],
    _min_region_area: i32,
    merge_region_area: i32,
    src_reg: &mut [u16],
) -> Result<()> {
    // Find regions to merge
    let mut merge_count;

    loop {
        merge_count = 0;

        // Collect merge operations first to avoid borrow issues
        let mut merges = Vec::new();

        for i in 0..regions.len() {
            let reg = &regions[i];
            if reg.id == 0 || (reg.id & RC_BORDER_REG) != 0 {
                continue;
            }

            if reg.span_count == 0 {
                continue;
            }

            // Check if region should be merged
            if reg.span_count < merge_region_area && !reg.connects_to_border {
                // Find best merge candidate
                let mut best_neighbor_id = 0;
                let mut best_neighbor_size = 0;

                for &neighbor_id in &reg.connections {
                    if neighbor_id < regions.len() {
                        let neighbor = &regions[neighbor_id];
                        if neighbor.id == 0 || (neighbor.id & RC_BORDER_REG) != 0 {
                            continue;
                        }

                        if neighbor.span_count > best_neighbor_size {
                            best_neighbor_id = neighbor.id;
                            best_neighbor_size = neighbor.span_count;
                        }
                    }
                }

                // Record merge operation
                if best_neighbor_id > 0 {
                    merges.push((i, reg.id, best_neighbor_id, reg.span_count));
                    merge_count += 1;
                }
            }
        }

        // Apply merges
        for (region_idx, old_id, best_neighbor_id, span_count) in merges {
            // Update spans
            for reg in &mut *src_reg {
                if *reg == old_id {
                    *reg = best_neighbor_id;
                }
            }

            // Update region info
            regions[region_idx].id = 0;

            // Find and update the best neighbor
            for region in &mut *regions {
                if region.id == best_neighbor_id {
                    region.span_count += span_count;
                    break;
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
fn compact_region_ids(regions: &[Region], src_reg: &mut [u16]) -> Result<()> {
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
