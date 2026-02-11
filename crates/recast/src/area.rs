//! Area marking operations for Recast
//!
//! This module provides functions to mark and filter areas in the navigation mesh,
//! following the exact C++ implementation.

use super::compact_heightfield::CompactHeightfield;
use crate::error::BuildError;

const EPSILON: f32 = 1e-6;

/// Sorts the given data in-place using insertion sort
/// Matches C++ insertSort
fn insert_sort(data: &mut [u8]) {
    for value_index in 1..data.len() {
        let value = data[value_index];
        let mut insertion_index = value_index as i32 - 1;

        while insertion_index >= 0 && data[insertion_index as usize] > value {
            data[insertion_index as usize + 1] = data[insertion_index as usize];
            insertion_index -= 1;
        }

        data[(insertion_index + 1) as usize] = value;
    }
}

/// Checks if a point is contained within a polygon
/// Matches C++ pointInPoly
fn point_in_poly(num_verts: usize, verts: &[f32], point: &[f32]) -> bool {
    let mut in_poly = false;
    let mut j = num_verts - 1;

    for i in 0..num_verts {
        let vi = &verts[i * 3..];
        let vj = &verts[j * 3..];

        if (vi[2] > point[2]) == (vj[2] > point[2]) {
            j = i;
            continue;
        }

        if point[0] < (vj[0] - vi[0]) * (point[2] - vi[2]) / (vj[2] - vi[2]) + vi[0] {
            in_poly = !in_poly;
        }

        j = i;
    }

    in_poly
}

/// Normalizes the vector if the length is greater than zero
/// Matches C++ rcVsafeNormalize
fn safe_normalize(v: &mut [f32; 3]) {
    let sq_mag = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if sq_mag > EPSILON {
        let inverse_mag = 1.0 / sq_mag.sqrt();
        v[0] *= inverse_mag;
        v[1] *= inverse_mag;
        v[2] *= inverse_mag;
    }
}

/// Vector subtraction for 3D vectors
fn vsub(dest: &mut [f32; 3], a: &[f32], b: &[f32]) {
    dest[0] = a[0] - b[0];
    dest[1] = a[1] - b[1];
    dest[2] = a[2] - b[2];
}

/// Erodes walkable area by specified radius
/// Matches C++ `rcErodeWalkableArea` - uses `con[4]` connections directly
pub fn erode_walkable_area(
    chf: &mut CompactHeightfield,
    erosion_radius: i32,
) -> Result<(), BuildError> {
    let w = chf.width;
    let h = chf.height;
    let span_count = chf.spans.len();

    const NOT_CONNECTED: usize = 63; // RC_NOT_CONNECTED

    // Allocate distance buffer
    let mut dist = vec![0xff_u8; span_count];

    // Mark boundary cells (spans with missing or non-walkable neighbors)
    for z in 0..h {
        for x in 0..w {
            let cell_idx = (z * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(index) = cell.index {
                for s in 0..cell.count {
                    let span_idx = index + s;

                    if chf.areas[span_idx] == 0 {
                        dist[span_idx] = 0;
                        continue;
                    }

                    let span = &chf.spans[span_idx];

                    // Check all 4 cardinal directions
                    let mut neighbor_count = 0;
                    for dir in 0..4 {
                        if span.con[dir] != NOT_CONNECTED {
                            let nx = x + chf.get_dir_offset_x(dir);
                            let nz = z + chf.get_dir_offset_z(dir);

                            if nx >= 0 && nz >= 0 && nx < w && nz < h {
                                let ncell_idx = (nz * w + nx) as usize;
                                let ncell = &chf.cells[ncell_idx];

                                if let Some(nindex) = ncell.index {
                                    let ni = nindex + span.con[dir];
                                    if chf.areas[ni] != 0 {
                                        neighbor_count += 1;
                                        continue;
                                    }
                                }
                            }
                        }
                        // Missing or non-walkable neighbor
                        break;
                    }

                    if neighbor_count != 4 {
                        dist[span_idx] = 0;
                    }
                }
            }
        }
    }

    // Helper to resolve neighbor span index using con[dir]
    // Returns None if not connected or out of bounds
    let resolve_neighbor = |span_idx: usize, x: i32, z: i32, dir: usize| -> Option<usize> {
        let span = &chf.spans[span_idx];
        if span.con[dir] == NOT_CONNECTED {
            return None;
        }
        let nx = x + chf.get_dir_offset_x(dir);
        let nz = z + chf.get_dir_offset_z(dir);
        if nx < 0 || nz < 0 || nx >= w || nz >= h {
            return None;
        }
        let ncell = &chf.cells[(nz * w + nx) as usize];
        ncell.index.map(|ni| ni + span.con[dir])
    };

    // Pass 1 - forward propagation (top-left to bottom-right)
    for z in 0..h {
        for x in 0..w {
            let cell_idx = (z * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(index) = cell.index {
                for s in 0..cell.count {
                    let span_idx = index + s;

                    // Direction 0 (-X): +2
                    if let Some(ax) = resolve_neighbor(span_idx, x, z, 0) {
                        let nd = dist[ax].saturating_add(2);
                        if nd < dist[span_idx] {
                            dist[span_idx] = nd;
                        }

                        // Diagonal: dir 0 (-X) then dir 3 (-Z): +3
                        if let Some(bx) = resolve_neighbor(
                            ax,
                            x + chf.get_dir_offset_x(0),
                            z + chf.get_dir_offset_z(0),
                            3,
                        ) {
                            let nd = dist[bx].saturating_add(3);
                            if nd < dist[span_idx] {
                                dist[span_idx] = nd;
                            }
                        }
                    }

                    // Direction 3 (-Z): +2
                    if let Some(ax) = resolve_neighbor(span_idx, x, z, 3) {
                        let nd = dist[ax].saturating_add(2);
                        if nd < dist[span_idx] {
                            dist[span_idx] = nd;
                        }

                        // Diagonal: dir 3 (-Z) then dir 2 (+X): +3
                        if let Some(bx) = resolve_neighbor(
                            ax,
                            x + chf.get_dir_offset_x(3),
                            z + chf.get_dir_offset_z(3),
                            2,
                        ) {
                            let nd = dist[bx].saturating_add(3);
                            if nd < dist[span_idx] {
                                dist[span_idx] = nd;
                            }
                        }
                    }
                }
            }
        }
    }

    // Pass 2 - backward propagation (bottom-right to top-left)
    for z in (0..h).rev() {
        for x in (0..w).rev() {
            let cell_idx = (z * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(index) = cell.index {
                for s in 0..cell.count {
                    let span_idx = index + s;

                    // Direction 2 (+X): +2
                    if let Some(ax) = resolve_neighbor(span_idx, x, z, 2) {
                        let nd = dist[ax].saturating_add(2);
                        if nd < dist[span_idx] {
                            dist[span_idx] = nd;
                        }

                        // Diagonal: dir 2 (+X) then dir 1 (+Z): +3
                        if let Some(bx) = resolve_neighbor(
                            ax,
                            x + chf.get_dir_offset_x(2),
                            z + chf.get_dir_offset_z(2),
                            1,
                        ) {
                            let nd = dist[bx].saturating_add(3);
                            if nd < dist[span_idx] {
                                dist[span_idx] = nd;
                            }
                        }
                    }

                    // Direction 1 (+Z): +2
                    if let Some(ax) = resolve_neighbor(span_idx, x, z, 1) {
                        let nd = dist[ax].saturating_add(2);
                        if nd < dist[span_idx] {
                            dist[span_idx] = nd;
                        }

                        // Diagonal: dir 1 (+Z) then dir 0 (-X): +3
                        if let Some(bx) = resolve_neighbor(
                            ax,
                            x + chf.get_dir_offset_x(1),
                            z + chf.get_dir_offset_z(1),
                            0,
                        ) {
                            let nd = dist[bx].saturating_add(3);
                            if nd < dist[span_idx] {
                                dist[span_idx] = nd;
                            }
                        }
                    }
                }
            }
        }
    }

    // Mark spans that are too close to boundaries
    let min_dist = (erosion_radius * 2) as u8;
    for i in 0..span_count {
        if dist[i] < min_dist {
            chf.areas[i] = 0;
        }
    }

    Ok(())
}

/// Applies median filter to walkable areas
/// Matches C++ rcMedianFilterWalkableArea exactly
pub fn median_filter_walkable_area(chf: &mut CompactHeightfield) -> Result<(), BuildError> {
    let w = chf.width;
    let h = chf.height;
    let span_count = chf.spans.len();

    // Allocate temporary areas buffer
    let mut areas = vec![0xff_u8; span_count];

    for z in 0..h {
        for x in 0..w {
            let cell_idx = (z * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(index) = cell.index {
                for s in 0..cell.count {
                    let span_idx = index + s;
                    let _span = &chf.spans[span_idx];

                    if chf.areas[span_idx] == 0 {
                        areas[span_idx] = chf.areas[span_idx];
                        continue;
                    }

                    let mut neighbor_areas = [chf.areas[span_idx]; 9];

                    // Check all 8 neighbors (4 cardinal + 4 diagonal)
                    // Cardinal directions: N=1, E=3, S=5, W=7
                    // Diagonal directions: NE=2, SE=4, SW=6, NW=0
                    let cardinal_dirs = [1u8, 3u8, 5u8, 7u8];
                    for (i, &dir) in cardinal_dirs.iter().enumerate() {
                        if let Some(ax_idx) = chf.get_neighbor(span_idx, dir) {
                            if chf.areas[ax_idx] != 0 {
                                neighbor_areas[i * 2] = chf.areas[ax_idx];
                            }

                            // Get the next diagonal direction
                            let diag_dir = (dir + 1) % 8;
                            if let Some(bx_idx) = chf.get_neighbor(ax_idx, diag_dir) {
                                if chf.areas[bx_idx] != 0 {
                                    neighbor_areas[i * 2 + 1] = chf.areas[bx_idx];
                                }
                            }
                        }
                    }

                    insert_sort(&mut neighbor_areas);
                    areas[span_idx] = neighbor_areas[4];
                }
            }
        }
    }

    // Copy filtered areas back
    chf.areas.copy_from_slice(&areas);

    Ok(())
}

/// Marks spans within a box area
/// Matches C++ rcMarkBoxArea exactly
pub fn mark_box_area(
    chf: &mut CompactHeightfield,
    bmin: &[f32; 3],
    bmax: &[f32; 3],
    area_id: u8,
) -> Result<(), BuildError> {
    let w = chf.width;
    let h = chf.height;

    // Find the footprint of the box area in grid cell coordinates
    let min_x = ((bmin[0] - chf.bmin.x) / chf.cs) as i32;
    let min_y = ((bmin[1] - chf.bmin.y) / chf.ch) as i32;
    let min_z = ((bmin[2] - chf.bmin.z) / chf.cs) as i32;
    let max_x = ((bmax[0] - chf.bmin.x) / chf.cs) as i32;
    let max_y = ((bmax[1] - chf.bmin.y) / chf.ch) as i32;
    let max_z = ((bmax[2] - chf.bmin.z) / chf.cs) as i32;

    // Early-out if the box is outside the bounds of the grid
    if max_x < 0 || min_x >= w || max_z < 0 || min_z >= h {
        return Ok(());
    }

    // Clamp relevant bound coordinates to the grid
    let min_x = min_x.max(0);
    let max_x = max_x.min(w - 1);
    let min_z = min_z.max(0);
    let max_z = max_z.min(h - 1);

    // Mark relevant cells
    for z in min_z..=max_z {
        for x in min_x..=max_x {
            let cell_idx = (z * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(index) = cell.index {
                for s in 0..cell.count {
                    let span_idx = index + s;
                    let span = &chf.spans[span_idx];

                    // Skip if the span is outside the box extents
                    if span.y < min_y || span.y > max_y {
                        continue;
                    }

                    // Skip if the span has been removed
                    if chf.areas[span_idx] == 0 {
                        continue;
                    }

                    // Mark the span
                    chf.areas[span_idx] = area_id;
                }
            }
        }
    }

    Ok(())
}

/// Marks spans within a convex polygon area
/// Matches C++ rcMarkConvexPolyArea exactly
pub fn mark_convex_poly_area(
    chf: &mut CompactHeightfield,
    verts: &[f32],
    num_verts: usize,
    min_y: f32,
    max_y: f32,
    area_id: u8,
) -> Result<(), BuildError> {
    let w = chf.width;
    let h = chf.height;

    // Compute the bounding box of the polygon
    let mut bmin = [verts[0], min_y, verts[2]];
    let mut bmax = [verts[0], max_y, verts[2]];

    for i in 1..num_verts {
        let v = &verts[i * 3..];
        bmin[0] = bmin[0].min(v[0]);
        bmin[2] = bmin[2].min(v[2]);
        bmax[0] = bmax[0].max(v[0]);
        bmax[2] = bmax[2].max(v[2]);
    }

    // Compute the grid footprint of the polygon
    let minx = ((bmin[0] - chf.bmin.x) / chf.cs) as i32;
    let miny = ((bmin[1] - chf.bmin.y) / chf.ch) as i32;
    let minz = ((bmin[2] - chf.bmin.z) / chf.cs) as i32;
    let maxx = ((bmax[0] - chf.bmin.x) / chf.cs) as i32;
    let maxy = ((bmax[1] - chf.bmin.y) / chf.ch) as i32;
    let maxz = ((bmax[2] - chf.bmin.z) / chf.cs) as i32;

    // Early-out if the polygon lies entirely outside the grid
    if maxx < 0 || minx >= w || maxz < 0 || minz >= h {
        return Ok(());
    }

    // Clamp the polygon footprint to the grid
    let minx = minx.max(0);
    let maxx = maxx.min(w - 1);
    let minz = minz.max(0);
    let maxz = maxz.min(h - 1);

    // Mark cells within the polygon
    for z in minz..=maxz {
        for x in minx..=maxx {
            let cell_idx = (z * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            if let Some(index) = cell.index {
                for s in 0..cell.count {
                    let span_idx = index + s;
                    let span = &chf.spans[span_idx];

                    // Skip if span is removed
                    if chf.areas[span_idx] == 0 {
                        continue;
                    }

                    // Skip if y extents don't overlap
                    if span.y < miny || span.y > maxy {
                        continue;
                    }

                    let point = [
                        chf.bmin.x + (x as f32 + 0.5) * chf.cs,
                        0.0,
                        chf.bmin.z + (z as f32 + 0.5) * chf.cs,
                    ];

                    if point_in_poly(num_verts, verts, &point) {
                        chf.areas[span_idx] = area_id;
                    }
                }
            }
        }
    }

    Ok(())
}

/// Offsets a polygon by the specified amount
/// Matches C++ rcOffsetPoly exactly
pub fn offset_poly(
    verts: &[f32],
    num_verts: usize,
    offset: f32,
    out_verts: &mut Vec<f32>,
    max_out_verts: usize,
) -> usize {
    const MITER_LIMIT: f32 = 1.20;

    let mut num_out_verts = 0;

    for vert_index in 0..num_verts {
        // Grab three vertices of the polygon
        let vert_index_a = (vert_index + num_verts - 1) % num_verts;
        let vert_index_b = vert_index;
        let vert_index_c = (vert_index + 1) % num_verts;

        let vert_a = &verts[vert_index_a * 3..];
        let vert_b = &verts[vert_index_b * 3..];
        let vert_c = &verts[vert_index_c * 3..];

        // From A to B on the x/z plane
        let mut prev_segment_dir = [0f32; 3];
        vsub(&mut prev_segment_dir, vert_b, vert_a);
        prev_segment_dir[1] = 0.0; // Squash onto x/z plane
        safe_normalize(&mut prev_segment_dir);

        // From B to C on the x/z plane
        let mut curr_segment_dir = [0f32; 3];
        vsub(&mut curr_segment_dir, vert_c, vert_b);
        curr_segment_dir[1] = 0.0; // Squash onto x/z plane
        safe_normalize(&mut curr_segment_dir);

        // The y component of the cross product
        let cross =
            curr_segment_dir[0] * prev_segment_dir[2] - prev_segment_dir[0] * curr_segment_dir[2];

        // CCW perpendicular vector to AB
        let prev_segment_norm_x = -prev_segment_dir[2];
        let prev_segment_norm_z = prev_segment_dir[0];

        // CCW perpendicular vector to BC
        let curr_segment_norm_x = -curr_segment_dir[2];
        let curr_segment_norm_z = curr_segment_dir[0];

        // Average the two segment normals
        let mut corner_miter_x = (prev_segment_norm_x + curr_segment_norm_x) * 0.5;
        let mut corner_miter_z = (prev_segment_norm_z + curr_segment_norm_z) * 0.5;
        let corner_miter_sq_mag = corner_miter_x * corner_miter_x + corner_miter_z * corner_miter_z;

        // Check if should bevel
        let bevel = corner_miter_sq_mag * MITER_LIMIT * MITER_LIMIT < 1.0;

        // Scale the corner miter
        if corner_miter_sq_mag > EPSILON {
            let scale = 1.0 / corner_miter_sq_mag;
            corner_miter_x *= scale;
            corner_miter_z *= scale;
        }

        if bevel && cross < 0.0 {
            // Generate two bevel vertices
            if num_out_verts + 2 > max_out_verts {
                return 0;
            }

            let d = (1.0
                - (prev_segment_dir[0] * curr_segment_dir[0]
                    + prev_segment_dir[2] * curr_segment_dir[2]))
                * 0.5;

            out_verts.resize((num_out_verts + 2) * 3, 0.0);
            out_verts[num_out_verts * 3] =
                vert_b[0] + (-prev_segment_norm_x + prev_segment_dir[0] * d) * offset;
            out_verts[num_out_verts * 3 + 1] = vert_b[1];
            out_verts[num_out_verts * 3 + 2] =
                vert_b[2] + (-prev_segment_norm_z + prev_segment_dir[2] * d) * offset;
            num_out_verts += 1;

            out_verts[num_out_verts * 3] =
                vert_b[0] + (-curr_segment_norm_x - curr_segment_dir[0] * d) * offset;
            out_verts[num_out_verts * 3 + 1] = vert_b[1];
            out_verts[num_out_verts * 3 + 2] =
                vert_b[2] + (-curr_segment_norm_z - curr_segment_dir[2] * d) * offset;
            num_out_verts += 1;
        } else {
            if num_out_verts + 1 > max_out_verts {
                return 0;
            }

            out_verts.resize((num_out_verts + 1) * 3, 0.0);
            out_verts[num_out_verts * 3] = vert_b[0] - corner_miter_x * offset;
            out_verts[num_out_verts * 3 + 1] = vert_b[1];
            out_verts[num_out_verts * 3 + 2] = vert_b[2] - corner_miter_z * offset;
            num_out_verts += 1;
        }
    }

    num_out_verts
}

/// Marks spans within a cylinder area
/// Matches C++ rcMarkCylinderArea exactly
pub fn mark_cylinder_area(
    chf: &mut CompactHeightfield,
    position: &[f32; 3],
    radius: f32,
    height: f32,
    area_id: u8,
) -> Result<(), BuildError> {
    let w = chf.width;
    let h = chf.height;

    // Compute the bounding box of the cylinder
    let cylinder_bb_min = [position[0] - radius, position[1], position[2] - radius];
    let cylinder_bb_max = [
        position[0] + radius,
        position[1] + height,
        position[2] + radius,
    ];

    // Compute the grid footprint of the cylinder
    let minx = ((cylinder_bb_min[0] - chf.bmin.x) / chf.cs) as i32;
    let miny = ((cylinder_bb_min[1] - chf.bmin.y) / chf.ch) as i32;
    let minz = ((cylinder_bb_min[2] - chf.bmin.z) / chf.cs) as i32;
    let maxx = ((cylinder_bb_max[0] - chf.bmin.x) / chf.cs) as i32;
    let maxy = ((cylinder_bb_max[1] - chf.bmin.y) / chf.ch) as i32;
    let maxz = ((cylinder_bb_max[2] - chf.bmin.z) / chf.cs) as i32;

    // Early-out if the cylinder is completely outside the grid bounds
    if maxx < 0 || minx >= w || maxz < 0 || minz >= h {
        return Ok(());
    }

    // Clamp the cylinder bounds to the grid
    let minx = minx.max(0);
    let maxx = maxx.min(w - 1);
    let minz = minz.max(0);
    let maxz = maxz.min(h - 1);

    let radius_sq = radius * radius;

    for z in minz..=maxz {
        for x in minx..=maxx {
            let cell_idx = (z * w + x) as usize;
            let cell = &chf.cells[cell_idx];

            let cell_x = chf.bmin.x + (x as f32 + 0.5) * chf.cs;
            let cell_z = chf.bmin.z + (z as f32 + 0.5) * chf.cs;
            let delta_x = cell_x - position[0];
            let delta_z = cell_z - position[2];

            // Skip this column if it's too far from the center
            if delta_x * delta_x + delta_z * delta_z >= radius_sq {
                continue;
            }

            if let Some(index) = cell.index {
                for s in 0..cell.count {
                    let span_idx = index + s;
                    let span = &chf.spans[span_idx];

                    // Skip if span is removed
                    if chf.areas[span_idx] == 0 {
                        continue;
                    }

                    // Mark if y extents overlap
                    if span.y >= miny && span.y <= maxy {
                        chf.areas[span_idx] = area_id;
                    }
                }
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_insert_sort() {
        let mut data = vec![5, 2, 8, 1, 9, 3, 7, 4, 6];
        insert_sort(&mut data);
        assert_eq!(data, vec![1, 2, 3, 4, 5, 6, 7, 8, 9]);
    }

    #[test]
    fn test_point_in_poly() {
        // Define a square polygon
        let verts = vec![0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0.0, 5.0, 0.0, 0.0, 5.0];

        // Test point inside
        assert!(point_in_poly(4, &verts, &[2.5, 0.0, 2.5]));

        // Test point outside
        assert!(!point_in_poly(4, &verts, &[6.0, 0.0, 3.0]));

        // Test point on edge - edge behavior depends on implementation
        // The ray-casting algorithm can return either true or false for edge points
        // depending on how ties are broken. Our implementation returns true.
        assert!(point_in_poly(4, &verts, &[0.0, 0.0, 2.5]));
    }

    #[test]
    fn test_safe_normalize() {
        let mut v = [3.0, 0.0, 4.0];
        safe_normalize(&mut v);
        let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        assert!((mag - 1.0).abs() < 0.0001);

        // Test zero vector
        let mut v_zero = [0.0, 0.0, 0.0];
        safe_normalize(&mut v_zero);
        assert_eq!(v_zero, [0.0, 0.0, 0.0]);
    }
}
