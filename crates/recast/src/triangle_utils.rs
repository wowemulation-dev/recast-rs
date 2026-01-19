//! Triangle utilities for Recast
//!
//! This module provides triangle-related utilities following the exact C++ implementation.

use glam::Vec3;

use crate::heightfield::Heightfield;

/// Walkable area ID (matches C++ RC_WALKABLE_AREA)
pub const RC_WALKABLE_AREA: u8 = 63;

/// Null area ID (matches C++ RC_NULL_AREA)
pub const RC_NULL_AREA: u8 = 0;

/// Calculates the normal of a triangle (matches C++ calcTriNormal)
fn calc_tri_normal(v0: &[f32], v1: &[f32], v2: &[f32]) -> [f32; 3] {
    let e0 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    let e1 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

    let mut n = [
        e0[1] * e1[2] - e0[2] * e1[1],
        e0[2] * e1[0] - e0[0] * e1[2],
        e0[0] * e1[1] - e0[1] * e1[0],
    ];

    // Normalize
    let d = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if d > 0.0 {
        let inv_d = 1.0 / d;
        n[0] *= inv_d;
        n[1] *= inv_d;
        n[2] *= inv_d;
    }

    n
}

/// Marks triangles as walkable based on slope angle
/// Matches C++ rcMarkWalkableTriangles exactly
pub fn mark_walkable_triangles(
    walkable_slope_angle: f32,
    verts: &[f32],
    tris: &[i32],
    num_tris: usize,
    tri_area_ids: &mut [u8],
) {
    let walkable_thr = (walkable_slope_angle / 180.0 * std::f32::consts::PI).cos();

    for (i, area_id) in tri_area_ids.iter_mut().take(num_tris).enumerate() {
        let t = i * 3;
        let v0 = &verts[(tris[t] * 3) as usize..((tris[t] + 1) * 3) as usize];
        let v1 = &verts[(tris[t + 1] * 3) as usize..((tris[t + 1] + 1) * 3) as usize];
        let v2 = &verts[(tris[t + 2] * 3) as usize..((tris[t + 2] + 1) * 3) as usize];

        let norm = calc_tri_normal(v0, v1, v2);
        // Check if the face is walkable
        if norm[1] > walkable_thr {
            *area_id = RC_WALKABLE_AREA;
        }
    }
}

/// Clears triangles with steep slopes (marks as unwalkable)
/// Matches C++ rcClearUnwalkableTriangles exactly
pub fn clear_unwalkable_triangles(
    walkable_slope_angle: f32,
    verts: &[f32],
    tris: &[i32],
    num_tris: usize,
    tri_area_ids: &mut [u8],
) {
    let walkable_thr = (walkable_slope_angle / 180.0 * std::f32::consts::PI).cos();

    for (i, area_id) in tri_area_ids.iter_mut().take(num_tris).enumerate() {
        let t = i * 3;
        let v0 = &verts[(tris[t] * 3) as usize..((tris[t] + 1) * 3) as usize];
        let v1 = &verts[(tris[t + 1] * 3) as usize..((tris[t + 1] + 1) * 3) as usize];
        let v2 = &verts[(tris[t + 2] * 3) as usize..((tris[t + 2] + 1) * 3) as usize];

        let norm = calc_tri_normal(v0, v1, v2);
        // Check if the face is walkable
        if norm[1] <= walkable_thr {
            *area_id = RC_NULL_AREA;
        }
    }
}

/// Calculates the bounding box of a set of vertices
/// Matches C++ rcCalcBounds exactly
pub fn calc_bounds(verts: &[f32]) -> ([f32; 3], [f32; 3]) {
    let mut min_bounds = [f32::MAX; 3];
    let mut max_bounds = [f32::MIN; 3];

    let n_verts = verts.len() / 3;
    for i in 0..n_verts {
        let v = &verts[i * 3..(i + 1) * 3];
        min_bounds[0] = min_bounds[0].min(v[0]);
        min_bounds[1] = min_bounds[1].min(v[1]);
        min_bounds[2] = min_bounds[2].min(v[2]);
        max_bounds[0] = max_bounds[0].max(v[0]);
        max_bounds[1] = max_bounds[1].max(v[1]);
        max_bounds[2] = max_bounds[2].max(v[2]);
    }

    (min_bounds, max_bounds)
}

/// Calculates grid size from bounds and cell size
/// Matches C++ rcCalcGridSize exactly
pub fn calc_grid_size(min_bounds: &[f32; 3], max_bounds: &[f32; 3], cell_size: f32) -> (i32, i32) {
    let size_x = ((max_bounds[0] - min_bounds[0]) / cell_size + 0.5) as i32;
    let size_z = ((max_bounds[2] - min_bounds[2]) / cell_size + 0.5) as i32;
    (size_x, size_z)
}

// Vector math utilities

/// Cross product of two 3D vectors (matches C++ rcVcross)
pub fn vcross(v1: &[f32; 3], v2: &[f32; 3]) -> [f32; 3] {
    [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ]
}

/// Dot product of two 3D vectors (matches C++ rcVdot)
pub fn vdot(v1: &[f32; 3], v2: &[f32; 3]) -> f32 {
    v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
}

/// Vector multiply-add: v1 + v2 * s (matches C++ rcVmad)
pub fn vmad(v1: &[f32; 3], v2: &[f32; 3], s: f32) -> [f32; 3] {
    [v1[0] + v2[0] * s, v1[1] + v2[1] * s, v1[2] + v2[2] * s]
}

/// Vector addition (matches C++ rcVadd)
pub fn vadd(v1: &[f32; 3], v2: &[f32; 3]) -> [f32; 3] {
    [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]
}

/// Vector subtraction (matches C++ rcVsub)
pub fn vsub(v1: &[f32; 3], v2: &[f32; 3]) -> [f32; 3] {
    [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]
}

/// Component-wise minimum (matches C++ rcVmin)
pub fn vmin(v1: &[f32; 3], v2: &[f32; 3]) -> [f32; 3] {
    [v1[0].min(v2[0]), v1[1].min(v2[1]), v1[2].min(v2[2])]
}

/// Component-wise maximum (matches C++ rcVmax)
pub fn vmax(v1: &[f32; 3], v2: &[f32; 3]) -> [f32; 3] {
    [v1[0].max(v2[0]), v1[1].max(v2[1]), v1[2].max(v2[2])]
}

/// Copy vector (matches C++ rcVcopy)
pub fn vcopy(dest: &mut [f32; 3], src: &[f32; 3]) {
    dest[0] = src[0];
    dest[1] = src[1];
    dest[2] = src[2];
}

/// Distance between two points (matches C++ rcVdist)
pub fn vdist(v1: &[f32; 3], v2: &[f32; 3]) -> f32 {
    let dx = v2[0] - v1[0];
    let dy = v2[1] - v1[1];
    let dz = v2[2] - v1[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Squared distance between two points (matches C++ rcVdistSqr)
pub fn vdist_sqr(v1: &[f32; 3], v2: &[f32; 3]) -> f32 {
    let dx = v2[0] - v1[0];
    let dy = v2[1] - v1[1];
    let dz = v2[2] - v1[2];
    dx * dx + dy * dy + dz * dz
}

/// Normalize vector (matches C++ rcVnormalize)
pub fn vnormalize(v: &mut [f32; 3]) {
    let d = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if d > 0.0 {
        let inv_d = 1.0 / d;
        v[0] *= inv_d;
        v[1] *= inv_d;
        v[2] *= inv_d;
    }
}

/// Get direction offset X (matches C++ rcGetDirOffsetX)
pub fn get_dir_offset_x(dir: u8) -> i32 {
    const OFFSETS: [i32; 4] = [-1, 0, 1, 0];
    OFFSETS[(dir & 0x03) as usize]
}

/// Get direction offset Y/Z (matches C++ rcGetDirOffsetY)
pub fn get_dir_offset_y(dir: u8) -> i32 {
    const OFFSETS: [i32; 4] = [0, 1, 0, -1];
    OFFSETS[(dir & 0x03) as usize]
}

/// Get direction from offset (matches C++ rcGetDirForOffset)
pub fn get_dir_for_offset(x: i32, y: i32) -> Option<u8> {
    const DIRS: [(i32, i32, u8); 4] = [(-1, 0, 0), (0, 1, 1), (1, 0, 2), (0, -1, 3)];

    for &(dx, dy, dir) in &DIRS {
        if dx == x && dy == y {
            return Some(dir);
        }
    }
    None
}

// Conversion utilities for glam types

/// Convert glam Vector3 to array
pub fn vec3_to_array(v: &Vec3) -> [f32; 3] {
    [v.x, v.y, v.z]
}

/// Convert array to glam Vector3
pub fn array_to_vec3(a: &[f32; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

/// Calculate bounds for glam vectors
pub fn calc_bounds_vec3(verts: &[Vec3]) -> (Vec3, Vec3) {
    let mut min_bounds = Vec3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max_bounds = Vec3::new(f32::MIN, f32::MIN, f32::MIN);

    for v in verts {
        min_bounds.x = min_bounds.x.min(v.x);
        min_bounds.y = min_bounds.y.min(v.y);
        min_bounds.z = min_bounds.z.min(v.z);
        max_bounds.x = max_bounds.x.max(v.x);
        max_bounds.y = max_bounds.y.max(v.y);
        max_bounds.z = max_bounds.z.max(v.z);
    }

    (min_bounds, max_bounds)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calc_tri_normal() {
        // Counter-clockwise winding for upward-facing normal
        let v0 = [0.0, 0.0, 0.0];
        let v1 = [0.0, 0.0, 1.0];
        let v2 = [1.0, 0.0, 0.0];

        let normal = calc_tri_normal(&v0, &v1, &v2);

        // Normal should point up (0, 1, 0)
        assert!((normal[0] - 0.0).abs() < 0.001);
        assert!((normal[1] - 1.0).abs() < 0.001);
        assert!((normal[2] - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_mark_walkable_triangles() {
        let verts = vec![
            0.0, 0.0, 0.0, // v0
            0.0, 0.0, 1.0, // v1
            1.0, 0.0, 0.0, // v2
            0.0, 1.0, 0.0, // v3 (for steep triangle)
        ];

        let tris = vec![
            0, 1, 2, // Flat triangle (counter-clockwise for upward normal)
            0, 3, 1, // Steep triangle (vertical)
        ];

        let mut tri_area_ids = vec![0, 0];

        // Mark triangles with slope less than 45 degrees as walkable
        mark_walkable_triangles(45.0, &verts, &tris, 2, &mut tri_area_ids);

        // First triangle should be walkable
        assert_eq!(tri_area_ids[0], RC_WALKABLE_AREA);
        // Second triangle should not be walkable (vertical)
        assert_eq!(tri_area_ids[1], 0);
    }

    #[test]
    fn test_calc_bounds() {
        let verts = vec![-1.0, 0.0, -1.0, 1.0, 2.0, 1.0, 0.0, -1.0, 0.5];

        let (min_bounds, max_bounds) = calc_bounds(&verts);

        assert_eq!(min_bounds, [-1.0, -1.0, -1.0]);
        assert_eq!(max_bounds, [1.0, 2.0, 1.0]);
    }

    #[test]
    fn test_calc_grid_size() {
        let min_bounds = [0.0, 0.0, 0.0];
        let max_bounds = [10.0, 5.0, 10.0];
        let cell_size = 0.5;

        let (size_x, size_z) = calc_grid_size(&min_bounds, &max_bounds, cell_size);

        assert_eq!(size_x, 20);
        assert_eq!(size_z, 20);
    }

    #[test]
    fn test_vector_math() {
        let v1 = [1.0, 2.0, 3.0];
        let v2 = [4.0, 5.0, 6.0];

        // Test dot product
        let dot = vdot(&v1, &v2);
        assert_eq!(dot, 32.0); // 1*4 + 2*5 + 3*6

        // Test cross product
        let cross = vcross(&v1, &v2);
        assert_eq!(cross, [-3.0, 6.0, -3.0]);

        // Test distance
        let dist = vdist(&v1, &v2);
        assert!((dist - 5.196).abs() < 0.001);

        // Test normalization
        let mut v = [3.0, 4.0, 0.0];
        vnormalize(&mut v);
        assert!((v[0] - 0.6).abs() < 0.001);
        assert!((v[1] - 0.8).abs() < 0.001);
        assert!((v[2] - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_direction_offsets() {
        assert_eq!(get_dir_offset_x(0), -1);
        assert_eq!(get_dir_offset_y(0), 0);

        assert_eq!(get_dir_offset_x(1), 0);
        assert_eq!(get_dir_offset_y(1), 1);

        assert_eq!(get_dir_offset_x(2), 1);
        assert_eq!(get_dir_offset_y(2), 0);

        assert_eq!(get_dir_offset_x(3), 0);
        assert_eq!(get_dir_offset_y(3), -1);

        // Test get_dir_for_offset
        assert_eq!(get_dir_for_offset(-1, 0), Some(0));
        assert_eq!(get_dir_for_offset(0, 1), Some(1));
        assert_eq!(get_dir_for_offset(1, 0), Some(2));
        assert_eq!(get_dir_for_offset(0, -1), Some(3));
        assert_eq!(get_dir_for_offset(2, 2), None);
    }
}

/// Gets the count of walkable spans in the heightfield
/// This is a utility function that matches the C++ rcGetHeightFieldSpanCount
pub fn get_heightfield_span_count(heightfield: &Heightfield) -> i32 {
    heightfield.get_span_count()
}
