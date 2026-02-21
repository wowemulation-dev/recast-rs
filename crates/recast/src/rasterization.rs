//! Triangle rasterization for Recast
//!
//! This module provides functions to rasterize triangles into heightfields,
//! following the exact C++ implementation from RecastRasterization.cpp.

use super::heightfield::{Heightfield, SPAN_NULL, SpanEntry, SpanIndex};
use crate::error::BuildError;
use glam::Vec3;

/// Axis enum for polygon clipping
#[derive(Debug, Clone, Copy)]
enum Axis {
    X = 0,
    #[allow(dead_code)]
    Y = 1,
    Z = 2,
}

/// Adds a span to the heightfield at the specified position
/// Matches C++ rcAddSpan exactly
pub fn add_span(
    heightfield: &mut Heightfield,
    x: i32,
    z: i32,
    span_min: i16,
    span_max: i16,
    area_id: u8,
    flag_merge_threshold: i32,
) -> Result<(), BuildError> {
    // Check bounds
    if x < 0 || z < 0 || x >= heightfield.width || z >= heightfield.height {
        return Ok(()); // Silently ignore out of bounds, like C++
    }

    // Add span using internal implementation
    add_span_internal(
        heightfield,
        x,
        z,
        span_min,
        span_max,
        area_id,
        flag_merge_threshold,
    )
}

/// Internal implementation of span addition with merging logic.
/// Matches C++ addSpan static function.
fn add_span_internal(
    heightfield: &mut Heightfield,
    x: i32,
    z: i32,
    span_min: i16,
    span_max: i16,
    area_id: u8,
    flag_merge_threshold: i32,
) -> Result<(), BuildError> {
    let col_idx = (x + z * heightfield.width) as usize;
    let first = heightfield.columns[col_idx];

    // If column is empty, create first span
    if first == SPAN_NULL {
        let idx = heightfield.alloc_span(SpanEntry {
            min: span_min,
            max: span_max,
            area: area_id,
            next: SPAN_NULL,
        });
        heightfield.columns[col_idx] = idx;
        return Ok(());
    }

    // Find position where to insert the span
    let mut prev: SpanIndex = SPAN_NULL;
    let mut current = first;

    while current != SPAN_NULL {
        let cur = heightfield.spans[current as usize];

        // No overlap: new span is entirely below current
        if span_max < cur.min {
            let new_idx = heightfield.alloc_span(SpanEntry {
                min: span_min,
                max: span_max,
                area: area_id,
                next: current,
            });
            if prev != SPAN_NULL {
                heightfield.spans[prev as usize].next = new_idx;
            } else {
                heightfield.columns[col_idx] = new_idx;
            }
            return Ok(());
        }

        // No overlap: new span is entirely above current
        if span_min > cur.max {
            prev = current;
            current = cur.next;
            continue;
        }

        // Spans overlap, merge them
        let merged_min = span_min.min(cur.min);
        let mut merged_max = span_max.max(cur.max);

        // Determine merged area - match C++ logic exactly
        let mut merged_area = area_id;
        if (merged_max as i32 - cur.max as i32).abs() <= flag_merge_threshold {
            merged_area = merged_area.max(cur.area);
        }

        // Check if we need to merge with following spans
        let mut next_span = cur.next;
        while next_span != SPAN_NULL {
            let ns = heightfield.spans[next_span as usize];
            if ns.min > merged_max {
                break;
            }

            // Merge with next span
            merged_max = merged_max.max(ns.max);

            if (merged_max as i32 - ns.max as i32).abs() <= flag_merge_threshold {
                merged_area = merged_area.max(ns.area);
            }

            let following = ns.next;
            // Free the consumed span
            heightfield.free_span(next_span);
            next_span = following;
        }

        // Update current span with merged values
        heightfield.spans[current as usize].min = merged_min;
        heightfield.spans[current as usize].max = merged_max;
        heightfield.spans[current as usize].area = merged_area;
        heightfield.spans[current as usize].next = next_span;

        return Ok(());
    }

    // Add at end of list
    if prev != SPAN_NULL {
        let new_idx = heightfield.alloc_span(SpanEntry {
            min: span_min,
            max: span_max,
            area: area_id,
            next: SPAN_NULL,
        });
        heightfield.spans[prev as usize].next = new_idx;
    }

    Ok(())
}

/// Divides a convex polygon by an axis-aligned line
/// Matches C++ dividePoly exactly
fn divide_poly(
    in_verts: &[f32],
    out_verts1: &mut Vec<f32>,
    out_verts2: &mut Vec<f32>,
    axis_offset: f32,
    axis: Axis,
) {
    out_verts1.clear();
    out_verts2.clear();

    let axis_idx = axis as usize;
    let n = in_verts.len() / 3;

    if n == 0 {
        return;
    }

    // Determine side of each vertex
    let mut sides = Vec::with_capacity(n);
    for i in 0..n {
        let v = &in_verts[i * 3..];
        if v[axis_idx] < axis_offset {
            sides.push(-1);
        } else if v[axis_idx] > axis_offset {
            sides.push(1);
        } else {
            sides.push(0);
        }
    }

    // Clip polygon
    for i in 0..n {
        let j = (i + 1) % n;
        let vi = &in_verts[i * 3..(i + 1) * 3];
        let vj = &in_verts[j * 3..(j + 1) * 3];
        let si = sides[i];
        let sj = sides[j];

        if si == 0 {
            // Vertex on split line
            out_verts1.extend_from_slice(vi);
            out_verts2.extend_from_slice(vi);
        } else if si < 0 {
            // Vertex on negative side
            out_verts1.extend_from_slice(vi);
            if sj > 0 {
                // Edge crosses split line
                let t = (axis_offset - vi[axis_idx]) / (vj[axis_idx] - vi[axis_idx]);
                let mut intersection = [0.0; 3];
                for k in 0..3 {
                    intersection[k] = vi[k] + t * (vj[k] - vi[k]);
                }
                out_verts1.extend_from_slice(&intersection);
                out_verts2.extend_from_slice(&intersection);
            }
        } else {
            // Vertex on positive side
            out_verts2.extend_from_slice(vi);
            if sj < 0 {
                // Edge crosses split line
                let t = (axis_offset - vi[axis_idx]) / (vj[axis_idx] - vi[axis_idx]);
                let mut intersection = [0.0; 3];
                for k in 0..3 {
                    intersection[k] = vi[k] + t * (vj[k] - vi[k]);
                }
                out_verts1.extend_from_slice(&intersection);
                out_verts2.extend_from_slice(&intersection);
            }
        }
    }
}

/// Rasterizes a triangle into the heightfield
/// Matches C++ rasterizeTri exactly
fn rasterize_tri(
    v0: &[f32],
    v1: &[f32],
    v2: &[f32],
    area_id: u8,
    heightfield: &mut Heightfield,
    flag_merge_threshold: i32,
) -> Result<(), BuildError> {
    let inverse_cell_size = 1.0 / heightfield.cs;
    let inverse_cell_height = 1.0 / heightfield.ch;

    // Calculate triangle bounding box
    let mut tri_min = [v0[0], v0[1], v0[2]];
    let mut tri_max = [v0[0], v0[1], v0[2]];

    for i in 0..3 {
        tri_min[i] = tri_min[i].min(v1[i]).min(v2[i]);
        tri_max[i] = tri_max[i].max(v1[i]).max(v2[i]);
    }

    // Clip to heightfield bounds
    let hf_min = [heightfield.bmin.x, heightfield.bmin.y, heightfield.bmin.z];
    let hf_max = [heightfield.bmax.x, heightfield.bmax.y, heightfield.bmax.z];

    if !overlap_bounds(&tri_min, &tri_max, &hf_min, &hf_max) {
        return Ok(());
    }

    // Calculate footprint in grid coordinates
    let w = heightfield.width;
    let h = heightfield.height;
    let by = heightfield.bmax.y - heightfield.bmin.y;

    let z0 = ((tri_min[2] - heightfield.bmin.z) * inverse_cell_size) as i32;
    let z1 = ((tri_max[2] - heightfield.bmin.z) * inverse_cell_size) as i32;

    // Clamp z: use -1 rather than 0 to cut the polygon properly at the start of the tile
    let z0 = z0.clamp(-1, h - 1);
    let z1 = z1.clamp(0, h - 1);

    // Build initial triangle vertex list
    let in_verts = vec![
        v0[0], v0[1], v0[2], v1[0], v1[1], v1[2], v2[0], v2[1], v2[2],
    ];

    let mut out_verts1 = Vec::with_capacity(21);
    let mut out_verts2 = Vec::with_capacity(21);
    let mut in_row = Vec::with_capacity(21);

    // Initial polygon vertices
    let mut p1 = in_verts.clone();

    // Clip triangle to each grid cell
    for z in z0..=z1 {
        // Clip polygon to row. Store the remaining polygon as well
        let cz_max = heightfield.bmin.z + (z + 1) as f32 * heightfield.cs;
        divide_poly(&p1, &mut in_row, &mut out_verts1, cz_max, Axis::Z);

        // Swap for next iteration
        p1 = out_verts1.clone();

        let nv_row = in_row.len() / 3;
        if nv_row < 3 || z < 0 {
            continue;
        }

        // Find X-axis bounds of the row polygon (matches C++)
        let mut min_x = in_row[0];
        let mut max_x = in_row[0];
        for vert in 1..nv_row {
            min_x = min_x.min(in_row[vert * 3]);
            max_x = max_x.max(in_row[vert * 3]);
        }

        let x0 = ((min_x - heightfield.bmin.x) * inverse_cell_size) as i32;
        let x1 = ((max_x - heightfield.bmin.x) * inverse_cell_size) as i32;
        // Use -1 to cut the polygon properly at the start of the tile
        let x0 = x0.clamp(-1, w - 1);
        let x1 = x1.clamp(0, w - 1);

        // Initialize row polygon for X clipping
        let mut p2 = in_row.clone();

        // Process each column
        for x in x0..=x1 {
            // Clip polygon to column
            let cx_max = heightfield.bmin.x + (x + 1) as f32 * heightfield.cs;
            divide_poly(&p2, &mut out_verts2, &mut out_verts1, cx_max, Axis::X);

            // Swap for next iteration
            p2 = out_verts1.clone();

            let n = out_verts2.len() / 3;
            if n < 3 {
                continue;
            }

            // Skip the pre-clip cell (matches C++ `if (x < 0) continue;`)
            if x < 0 {
                continue;
            }

            // Find min/max Y in the clipped polygon
            let mut span_min_f = out_verts2[1];
            let mut span_max_f = out_verts2[1];

            for i in 1..n {
                let y = out_verts2[i * 3 + 1];
                span_min_f = span_min_f.min(y);
                span_max_f = span_max_f.max(y);
            }
            span_min_f -= heightfield.bmin.y;
            span_max_f -= heightfield.bmin.y;

            // Skip spans outside the heightfield bounding box
            if span_max_f < 0.0 || span_min_f > by {
                continue;
            }

            // Clamp to heightfield bounding box
            span_min_f = span_min_f.clamp(0.0, by);
            span_max_f = span_max_f.clamp(0.0, by);

            // Snap to heightfield height grid (matches C++ clamping)
            let span_min = (span_min_f * inverse_cell_height)
                .floor()
                .clamp(0.0, 8191.0) as i16;
            let span_max = (span_max_f * inverse_cell_height)
                .ceil()
                .clamp((span_min + 1) as f32, 8191.0) as i16;

            // Add span
            add_span_internal(
                heightfield,
                x,
                z,
                span_min,
                span_max,
                area_id,
                flag_merge_threshold,
            )?;
        }
    }

    Ok(())
}

/// Checks if two bounding boxes overlap
/// Matches C++ overlapBounds
fn overlap_bounds(a_min: &[f32], a_max: &[f32], b_min: &[f32], b_max: &[f32]) -> bool {
    a_min[0] <= b_max[0]
        && a_max[0] >= b_min[0]
        && a_min[1] <= b_max[1]
        && a_max[1] >= b_min[1]
        && a_min[2] <= b_max[2]
        && a_max[2] >= b_min[2]
}

/// Rasterizes a single triangle into the heightfield
/// Matches C++ rcRasterizeTriangle exactly
pub fn rasterize_triangle(
    v0: &Vec3,
    v1: &Vec3,
    v2: &Vec3,
    area_id: u8,
    heightfield: &mut Heightfield,
    flag_merge_threshold: i32,
) -> Result<(), BuildError> {
    let v0_arr = [v0.x, v0.y, v0.z];
    let v1_arr = [v1.x, v1.y, v1.z];
    let v2_arr = [v2.x, v2.y, v2.z];

    rasterize_tri(
        &v0_arr,
        &v1_arr,
        &v2_arr,
        area_id,
        heightfield,
        flag_merge_threshold,
    )
}

/// Rasterizes multiple indexed triangles into the heightfield
/// Matches C++ rcRasterizeTriangles with int indices
pub fn rasterize_triangles(
    verts: &[f32],
    tris: &[i32],
    tri_area_ids: &[u8],
    num_tris: usize,
    heightfield: &mut Heightfield,
    flag_merge_threshold: i32,
) -> Result<(), BuildError> {
    for i in 0..num_tris {
        let v0_idx = tris[i * 3] as usize;
        let v1_idx = tris[i * 3 + 1] as usize;
        let v2_idx = tris[i * 3 + 2] as usize;

        let v0 = &verts[v0_idx * 3..(v0_idx + 1) * 3];
        let v1 = &verts[v1_idx * 3..(v1_idx + 1) * 3];
        let v2 = &verts[v2_idx * 3..(v2_idx + 1) * 3];

        let area_id = tri_area_ids[i];

        rasterize_tri(v0, v1, v2, area_id, heightfield, flag_merge_threshold)?;
    }

    Ok(())
}

/// Rasterizes multiple indexed triangles with u16 indices
/// Matches C++ rcRasterizeTriangles with unsigned short indices
pub fn rasterize_triangles_u16(
    verts: &[f32],
    tris: &[u16],
    tri_area_ids: &[u8],
    num_tris: usize,
    heightfield: &mut Heightfield,
    flag_merge_threshold: i32,
) -> Result<(), BuildError> {
    for i in 0..num_tris {
        let v0_idx = tris[i * 3] as usize;
        let v1_idx = tris[i * 3 + 1] as usize;
        let v2_idx = tris[i * 3 + 2] as usize;

        let v0 = &verts[v0_idx * 3..(v0_idx + 1) * 3];
        let v1 = &verts[v1_idx * 3..(v1_idx + 1) * 3];
        let v2 = &verts[v2_idx * 3..(v2_idx + 1) * 3];

        let area_id = tri_area_ids[i];

        rasterize_tri(v0, v1, v2, area_id, heightfield, flag_merge_threshold)?;
    }

    Ok(())
}

/// Rasterizes a triangle soup (sequential vertices)
/// Matches C++ rcRasterizeTriangles for triangle soup
pub fn rasterize_triangle_soup(
    verts: &[f32],
    tri_area_ids: &[u8],
    num_tris: usize,
    heightfield: &mut Heightfield,
    flag_merge_threshold: i32,
) -> Result<(), BuildError> {
    for (i, &area_id) in tri_area_ids.iter().take(num_tris).enumerate() {
        let base_idx = i * 9; // 3 vertices * 3 floats
        let v0 = &verts[base_idx..base_idx + 3];
        let v1 = &verts[base_idx + 3..base_idx + 6];
        let v2 = &verts[base_idx + 6..base_idx + 9];

        rasterize_tri(v0, v1, v2, area_id, heightfield, flag_merge_threshold)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn test_add_span_simple() {
        let mut heightfield = Heightfield::new(
            10,
            10,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 10.0, 10.0),
            1.0,
            1.0,
        );

        add_span(&mut heightfield, 5, 5, 10, 20, 1, 1).unwrap();

        let si = heightfield.column_first_span(5, 5);
        assert_ne!(si, SPAN_NULL);
        let span = heightfield.span(si);
        assert_eq!(span.min, 10);
        assert_eq!(span.max, 20);
        assert_eq!(span.area, 1);
    }

    #[test]
    fn test_add_span_merge() {
        let mut heightfield = Heightfield::new(
            10,
            10,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 10.0, 10.0),
            1.0,
            1.0,
        );

        add_span(&mut heightfield, 5, 5, 10, 20, 1, 1).unwrap();
        add_span(&mut heightfield, 5, 5, 15, 25, 2, 1).unwrap();

        let si = heightfield.column_first_span(5, 5);
        assert_ne!(si, SPAN_NULL);
        let span = heightfield.span(si);
        assert_eq!(span.min, 10);
        assert_eq!(span.max, 25);
        assert_eq!(span.area, 2);
    }

    #[test]
    fn test_rasterize_triangle() {
        let mut heightfield = Heightfield::new(
            10,
            10,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 10.0, 10.0),
            1.0,
            1.0,
        );

        // Rasterize a triangle
        let v0 = Vec3::new(2.0, 0.0, 2.0);
        let v1 = Vec3::new(8.0, 0.0, 2.0);
        let v2 = Vec3::new(5.0, 2.0, 8.0);

        rasterize_triangle(&v0, &v1, &v2, 1, &mut heightfield, 1).unwrap();

        // Check that some spans were created
        let has_spans = heightfield.columns().iter().any(|&si| si != SPAN_NULL);
        assert!(has_spans);
    }
}
