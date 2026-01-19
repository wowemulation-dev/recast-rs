//! Common utilities for Detour navigation mesh operations
//!

use recast_common;

/// Returns the minimum of two values
#[inline]
pub fn dt_min<T: PartialOrd>(a: T, b: T) -> T {
    if a < b {
        a
    } else {
        b
    }
}

/// Returns the maximum of two values
#[inline]
pub fn dt_max<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else {
        b
    }
}

/// Returns the absolute value
#[inline]
pub fn dt_abs<T: PartialOrd + std::ops::Neg<Output = T> + Default>(a: T) -> T {
    if a < T::default() {
        -a
    } else {
        a
    }
}

/// Returns the square of the value
#[inline]
pub fn dt_sqr<T: std::ops::Mul<Output = T> + Copy>(a: T) -> T {
    a * a
}

/// Clamps the value to the specified range
#[inline]
pub fn dt_clamp<T: PartialOrd>(v: T, mn: T, mx: T) -> T {
    if v < mn {
        mn
    } else if v > mx {
        mx
    } else {
        v
    }
}

/// Swaps the values of two parameters
#[inline]
pub fn dt_swap<T>(a: &mut T, b: &mut T) {
    std::mem::swap(a, b);
}

/// Derives the cross product of two vectors (v1 x v2)
#[inline]
pub fn dt_vcross(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3]) {
    dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
    dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
    dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

/// Derives the dot product of two vectors (v1 . v2)
#[inline]
pub fn dt_vdot(v1: &[f32; 3], v2: &[f32; 3]) -> f32 {
    v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
}

/// Performs a scaled vector addition (v1 + (v2 * s))
#[inline]
pub fn dt_vmad(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3], s: f32) {
    dest[0] = v1[0] + v2[0] * s;
    dest[1] = v1[1] + v2[1] * s;
    dest[2] = v1[2] + v2[2] * s;
}

/// Performs a linear interpolation between two vectors (v1 toward v2)
#[inline]
pub fn dt_vlerp(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3], t: f32) {
    dest[0] = v1[0] + (v2[0] - v1[0]) * t;
    dest[1] = v1[1] + (v2[1] - v1[1]) * t;
    dest[2] = v1[2] + (v2[2] - v1[2]) * t;
}

/// Performs a vector addition (v1 + v2)
#[inline]
pub fn dt_vadd(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3]) {
    dest[0] = v1[0] + v2[0];
    dest[1] = v1[1] + v2[1];
    dest[2] = v1[2] + v2[2];
}

/// Performs a vector subtraction (v1 - v2)
#[inline]
pub fn dt_vsub(dest: &mut [f32; 3], v1: &[f32; 3], v2: &[f32; 3]) {
    dest[0] = v1[0] - v2[0];
    dest[1] = v1[1] - v2[1];
    dest[2] = v1[2] - v2[2];
}

/// Scales the vector by the specified value (v * t)
#[inline]
pub fn dt_vscale(dest: &mut [f32; 3], v: &[f32; 3], t: f32) {
    dest[0] = v[0] * t;
    dest[1] = v[1] * t;
    dest[2] = v[2] * t;
}

/// Selects the minimum value of each element from the specified vectors
#[inline]
pub fn dt_vmin(mn: &mut [f32; 3], v: &[f32; 3]) {
    mn[0] = dt_min(mn[0], v[0]);
    mn[1] = dt_min(mn[1], v[1]);
    mn[2] = dt_min(mn[2], v[2]);
}

/// Selects the maximum value of each element from the specified vectors
#[inline]
pub fn dt_vmax(mx: &mut [f32; 3], v: &[f32; 3]) {
    mx[0] = dt_max(mx[0], v[0]);
    mx[1] = dt_max(mx[1], v[1]);
    mx[2] = dt_max(mx[2], v[2]);
}

/// Sets the vector elements to the specified values
#[inline]
pub fn dt_vset(dest: &mut [f32; 3], x: f32, y: f32, z: f32) {
    dest[0] = x;
    dest[1] = y;
    dest[2] = z;
}

/// Performs a vector copy
#[inline]
pub fn dt_vcopy(dest: &mut [f32; 3], a: &[f32; 3]) {
    dest[0] = a[0];
    dest[1] = a[1];
    dest[2] = a[2];
}

/// Derives the scalar length of the vector
#[inline]
pub fn dt_vlen(v: &[f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Derives the square of the scalar length of the vector
#[inline]
pub fn dt_vlen_sqr(v: &[f32; 3]) -> f32 {
    v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
}

/// Returns the distance between two points
#[inline]
pub fn dt_vdist(v1: &[f32; 3], v2: &[f32; 3]) -> f32 {
    let dx = v2[0] - v1[0];
    let dy = v2[1] - v1[1];
    let dz = v2[2] - v1[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Returns the square of the distance between two points
#[inline]
pub fn dt_vdist_sqr(v1: &[f32; 3], v2: &[f32; 3]) -> f32 {
    let dx = v2[0] - v1[0];
    let dy = v2[1] - v1[1];
    let dz = v2[2] - v1[2];
    dx * dx + dy * dy + dz * dz
}

/// Derives the distance between the specified points on the xz-plane
#[inline]
pub fn dt_vdist_2d(v1: &[f32], v2: &[f32]) -> f32 {
    let dx = v2[0] - v1[0];
    let dz = v2[2] - v1[2];
    (dx * dx + dz * dz).sqrt()
}

/// Derives the square of the distance between the specified points on the xz-plane
#[inline]
pub fn dt_vdist_2d_sqr(v1: &[f32], v2: &[f32]) -> f32 {
    let dx = v2[0] - v1[0];
    let dz = v2[2] - v1[2];
    dx * dx + dz * dz
}

/// Normalizes the vector
#[inline]
pub fn dt_vnormalize(v: &mut [f32; 3]) {
    let d = 1.0 / (dt_sqr(v[0]) + dt_sqr(v[1]) + dt_sqr(v[2])).sqrt();
    v[0] *= d;
    v[1] *= d;
    v[2] *= d;
}

/// Performs a 'sloppy' colocation check of the specified points
#[inline]
pub fn dt_vequal(p0: &[f32; 3], p1: &[f32; 3]) -> bool {
    const THR: f32 = 1.0 / 16384.0;
    let d = dt_vdist_sqr(p0, p1);
    d < (THR * THR)
}

/// Checks that the specified vector's components are all finite
#[inline]
pub fn dt_visfinite(v: &[f32; 3]) -> bool {
    v[0].is_finite() && v[1].is_finite() && v[2].is_finite()
}

/// Checks that the specified vector's 2D components are finite
#[inline]
pub fn dt_visfinite_2d(v: &[f32]) -> bool {
    v[0].is_finite() && v[2].is_finite()
}

/// Derives the dot product of two vectors on the xz-plane
#[inline]
pub fn dt_vdot_2d(u: &[f32], v: &[f32]) -> f32 {
    u[0] * v[0] + u[2] * v[2]
}

/// Derives the xz-plane 2D perp product of the two vectors (uz*vx - ux*vz)
#[inline]
pub fn dt_vperp_2d(u: &[f32], v: &[f32]) -> f32 {
    u[2] * v[0] - u[0] * v[2]
}

/// Derives the signed xz-plane area of the triangle ABC
#[inline]
pub fn dt_tri_area_2d(a: &[f32], b: &[f32], c: &[f32]) -> f32 {
    recast_common::tri_area_2d(a, b, c)
}

/// Determines if two axis-aligned bounding boxes overlap
#[inline]
pub fn dt_overlap_bounds(
    amin: &[f32; 3],
    amax: &[f32; 3],
    bmin: &[f32; 3],
    bmax: &[f32; 3],
) -> bool {
    recast_common::overlap_bounds(amin, amax, bmin, bmax)
}

/// Determines if two quantized axis-aligned bounding boxes overlap
#[inline]
pub fn dt_overlap_quant_bounds(
    amin: &[u16; 3],
    amax: &[u16; 3],
    bmin: &[u16; 3],
    bmax: &[u16; 3],
) -> bool {
    !(amin[0] > bmax[0]
        || amax[0] < bmin[0]
        || amin[1] > bmax[1]
        || amax[1] < bmin[1]
        || amin[2] > bmax[2]
        || amax[2] < bmin[2])
}

/// Derives the closest point on a triangle from the specified reference point
pub fn dt_closest_pt_point_triangle(
    closest: &mut [f32; 3],
    p: &[f32; 3],
    a: &[f32; 3],
    b: &[f32; 3],
    c: &[f32; 3],
) {
    // Check if P in vertex region outside A
    let mut ab = [0.0f32; 3];
    let mut ac = [0.0f32; 3];
    let mut ap = [0.0f32; 3];
    dt_vsub(&mut ab, b, a);
    dt_vsub(&mut ac, c, a);
    dt_vsub(&mut ap, p, a);

    let d1 = dt_vdot(&ab, &ap);
    let d2 = dt_vdot(&ac, &ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        dt_vcopy(closest, a);
        return;
    }

    // Check if P in vertex region outside B
    let mut bp = [0.0f32; 3];
    dt_vsub(&mut bp, p, b);
    let d3 = dt_vdot(&ab, &bp);
    let d4 = dt_vdot(&ac, &bp);
    if d3 >= 0.0 && d4 <= d3 {
        dt_vcopy(closest, b);
        return;
    }

    // Check if P in edge region of AB, if so return projection of P onto AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        dt_vmad(closest, a, &ab, v);
        return;
    }

    // Check if P in vertex region outside C
    let mut cp = [0.0f32; 3];
    dt_vsub(&mut cp, p, c);
    let d5 = dt_vdot(&ab, &cp);
    let d6 = dt_vdot(&ac, &cp);
    if d6 >= 0.0 && d5 <= d6 {
        dt_vcopy(closest, c);
        return;
    }

    // Check if P in edge region of AC, if so return projection of P onto AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        dt_vmad(closest, a, &ac, w);
        return;
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let mut bc = [0.0f32; 3];
        dt_vsub(&mut bc, c, b);
        dt_vmad(closest, b, &bc, w);
        return;
    }

    // P inside face region. Compute Q through barycentric coordinates (u,v,w)
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    // First compute a + ab * v
    let mut temp = [0.0f32; 3];
    dt_vmad(&mut temp, a, &ab, v);
    // Then add ac * w to get the final result
    dt_vmad(closest, &temp, &ac, w);
}

/// Derives the y-axis height of the closest point on the triangle from the specified reference point
pub fn dt_closest_height_point_triangle(
    p: &[f32; 3],
    a: &[f32; 3],
    b: &[f32; 3],
    c: &[f32; 3],
) -> Option<f32> {
    let mut closest = [0.0f32; 3];
    dt_closest_pt_point_triangle(&mut closest, p, a, b, c);

    // Check if the closest point is within reasonable bounds
    let dx = p[0] - closest[0];
    let dz = p[2] - closest[2];
    let dist_sqr = dx * dx + dz * dz;

    // If the point is too far away horizontally, the height isn't meaningful
    if dist_sqr > 1000000.0 {
        return None;
    }

    Some(closest[1])
}

/// Finds the intersection of a 2D segment with a 2D polygon
pub fn dt_intersect_segment_poly_2d(
    p0: &[f32],
    p1: &[f32],
    verts: &[f32],
    nverts: usize,
) -> Option<(f32, f32, i32, i32)> {
    recast_common::intersect_segment_poly_2d(p0, p1, verts, nverts)
}

/// Check if two 2D segments intersect
pub fn dt_intersect_seg_seg_2d(
    ap: &[f32],
    aq: &[f32],
    bp: &[f32],
    bq: &[f32],
) -> Option<(f32, f32)> {
    // Use 2D math directly since we only need x,z components
    let dx1 = aq[0] - ap[0];
    let dz1 = aq[2] - ap[2];
    let dx2 = bq[0] - bp[0];
    let dz2 = bq[2] - bp[2];

    let denom = dz1 * dx2 - dx1 * dz2;

    if denom.abs() < 1e-6 {
        return None;
    }

    let dx3 = bp[0] - ap[0];
    let dz3 = bp[2] - ap[2];

    let s = (dz3 * dx2 - dx3 * dz2) / denom;
    let t = (dz3 * dx1 - dx3 * dz1) / denom;

    if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
        Some((s, t))
    } else {
        None
    }
}

/// Determines if the specified point is inside the convex polygon on the xz-plane
#[inline]
pub fn dt_point_in_polygon(pt: &[f32], verts: &[f32], nverts: usize) -> bool {
    recast_common::point_in_polygon_2d(pt, verts, nverts)
}

/// Finds the squared distance from a point to polygon edges
pub fn dt_distance_pt_poly_edges_sqr(
    pt: &[f32],
    verts: &[f32],
    nverts: usize,
) -> (bool, Vec<f32>, Vec<f32>) {
    recast_common::distance_pt_poly_edges_sqr(pt, verts, nverts)
}

/// Distance squared from a point to a segment
#[inline]
pub fn dt_dist_pt_seg_sqr_2d(pt: &[f32], p: &[f32], q: &[f32]) -> (f32, f32) {
    recast_common::dist_point_segment_sqr_2d_with_t(pt, p, q)
}

/// Derives the centroid of a convex polygon
pub fn dt_calc_poly_center(tc: &mut [f32; 3], idx: &[u16], nidx: usize, verts: &[f32]) {
    tc[0] = 0.0;
    tc[1] = 0.0;
    tc[2] = 0.0;

    for i in 0..nidx {
        let v = &verts[(idx[i] as usize) * 3..];
        tc[0] += v[0];
        tc[1] += v[1];
        tc[2] += v[2];
    }

    let scale = 1.0 / nidx as f32;
    tc[0] *= scale;
    tc[1] *= scale;
    tc[2] *= scale;
}

/// Determines if the two convex polygons overlap on the xz-plane
pub fn dt_overlap_poly_poly_2d(polya: &[f32], npolya: usize, polyb: &[f32], npolyb: usize) -> bool {
    const EPS: f32 = 1e-4;

    // Use separating axis theorem
    for i in 0..npolya {
        let va = &polya[i * 3..];
        let vb = &polya[((i + 1) % npolya) * 3..];

        let n = [va[2] - vb[2], 0.0, vb[0] - va[0]];
        let (amin, amax) = project_poly_axis(&n, polya, npolya);
        let (bmin, bmax) = project_poly_axis(&n, polyb, npolyb);

        if !overlap_range(amin, amax, bmin, bmax, EPS) {
            return false;
        }
    }

    for i in 0..npolyb {
        let va = &polyb[i * 3..];
        let vb = &polyb[((i + 1) % npolyb) * 3..];

        let n = [va[2] - vb[2], 0.0, vb[0] - va[0]];
        let (amin, amax) = project_poly_axis(&n, polya, npolya);
        let (bmin, bmax) = project_poly_axis(&n, polyb, npolyb);

        if !overlap_range(amin, amax, bmin, bmax, EPS) {
            return false;
        }
    }

    true
}

// Helper functions
fn project_poly_axis(axis: &[f32], poly: &[f32], npoly: usize) -> (f32, f32) {
    let mut min = f32::MAX;
    let mut max = f32::NEG_INFINITY;

    for i in 0..npoly {
        let v = &poly[i * 3..];
        let d = axis[0] * v[0] + axis[2] * v[2];
        min = min.min(d);
        max = max.max(d);
    }

    (min, max)
}

fn overlap_range(amin: f32, amax: f32, bmin: f32, bmax: f32, eps: f32) -> bool {
    (amin + eps) > bmax || (amax - eps) < bmin
}

/// Generates a random point inside a convex polygon
pub fn dt_random_point_in_convex_poly(
    pts: &[f32],
    npts: usize,
    areas: &mut [f32],
    s: f32,
    t: f32,
    out: &mut [f32; 3],
) {
    // Calculate triangle areas
    let mut acc = 0.0;
    for i in 0..(npts - 2) {
        let a = &pts[0..3];
        let b = &pts[(i + 1) * 3..(i + 2) * 3];
        let c = &pts[(i + 2) * 3..(i + 3) * 3];
        areas[i] = dt_tri_area_2d(a, b, c).abs() * 0.5;
        acc += areas[i];
    }

    // Find triangle
    let mut tri = 0;
    let mut u = s * acc;
    let mut sum = 0.0;

    for (i, area) in areas.iter().take(npts - 2).enumerate() {
        sum += area;
        if u <= sum {
            tri = i;
            u = (u - (sum - area)) / area;
            break;
        }
    }

    // Calculate point
    let a = &pts[0..3];
    let b = &pts[(tri + 1) * 3..(tri + 2) * 3];
    let c = &pts[(tri + 2) * 3..(tri + 3) * 3];

    let v = t.sqrt();
    let a_weight = 1.0 - v;
    let b_weight = v * (1.0 - u);
    let c_weight = v * u;

    out[0] = a[0] * a_weight + b[0] * b_weight + c[0] * c_weight;
    out[1] = a[1] * a_weight + b[1] * b_weight + c[1] * c_weight;
    out[2] = a[2] * a_weight + b[2] * b_weight + c[2] * c_weight;
}

/// Returns the next power of 2
#[inline]
pub fn dt_next_pow2(mut v: u32) -> u32 {
    if v == 0 {
        return 0;
    }
    v -= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v + 1
}

/// Returns the integer logarithm base 2
#[inline]
pub fn dt_ilog2(mut v: u32) -> u32 {
    let mut r = if v > 0xffff { 16 } else { 0 };
    v >>= r;

    let shift = if v > 0xff { 8 } else { 0 };
    v >>= shift;
    r |= shift;

    let shift = if v > 0xf { 4 } else { 0 };
    v >>= shift;
    r |= shift;

    let shift = if v > 0x3 { 2 } else { 0 };
    v >>= shift;
    r |= shift;

    r | (v >> 1)
}

/// Aligns value to 4-byte boundary
#[inline]
pub fn dt_align4(x: i32) -> i32 {
    (x + 3) & !3
}

/// Returns the opposite tile direction
#[inline]
pub fn dt_opposite_tile(side: i32) -> i32 {
    (side + 4) & 0x7
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_min_max() {
        assert_eq!(dt_min(5, 3), 3);
        assert_eq!(dt_max(5, 3), 5);
        assert_eq!(dt_min(-5, -3), -5);
        assert_eq!(dt_max(-5, -3), -3);
    }

    #[test]
    fn test_abs() {
        assert_eq!(dt_abs(5), 5);
        assert_eq!(dt_abs(-5), 5);
        assert_eq!(dt_abs(0), 0);
    }

    #[test]
    fn test_sqr() {
        assert_eq!(dt_sqr(3), 9);
        assert_eq!(dt_sqr(-3), 9);
        assert_eq!(dt_sqr(0), 0);
    }

    #[test]
    fn test_clamp() {
        assert_eq!(dt_clamp(5, 0, 10), 5);
        assert_eq!(dt_clamp(-5, 0, 10), 0);
        assert_eq!(dt_clamp(15, 0, 10), 10);
    }

    #[test]
    fn test_vector_operations() {
        let v1 = [1.0, 2.0, 3.0];
        let v2 = [4.0, 5.0, 6.0];
        let mut result = [0.0; 3];

        // Test cross product
        dt_vcross(&mut result, &v1, &v2);
        assert_eq!(result, [-3.0, 6.0, -3.0]);

        // Test dot product
        assert_eq!(dt_vdot(&v1, &v2), 32.0);

        // Test vector addition
        dt_vadd(&mut result, &v1, &v2);
        assert_eq!(result, [5.0, 7.0, 9.0]);

        // Test vector subtraction
        dt_vsub(&mut result, &v2, &v1);
        assert_eq!(result, [3.0, 3.0, 3.0]);

        // Test vector scaling
        dt_vscale(&mut result, &v1, 2.0);
        assert_eq!(result, [2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_distance() {
        let v1 = [0.0, 0.0, 0.0];
        let v2 = [3.0, 4.0, 0.0];

        assert_eq!(dt_vdist(&v1, &v2), 5.0);
        assert_eq!(dt_vdist_sqr(&v1, &v2), 25.0);
        assert_eq!(dt_vdist_2d(&v1, &v2), 3.0);
        assert_eq!(dt_vdist_2d_sqr(&v1, &v2), 9.0);
    }

    #[test]
    fn test_closest_point_triangle() {
        let p = [0.5, 0.0, 0.5];
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 0.0, 1.0];
        let mut closest = [0.0; 3];

        dt_closest_pt_point_triangle(&mut closest, &p, &a, &b, &c);

        // The closest point should be inside the triangle
        assert!((closest[0] - 0.5).abs() < 0.001);
        assert!((closest[1] - 0.0).abs() < 0.001);
        assert!((closest[2] - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_next_pow2() {
        assert_eq!(dt_next_pow2(0), 0);
        assert_eq!(dt_next_pow2(1), 1);
        assert_eq!(dt_next_pow2(2), 2);
        assert_eq!(dt_next_pow2(3), 4);
        assert_eq!(dt_next_pow2(5), 8);
        assert_eq!(dt_next_pow2(17), 32);
    }

    #[test]
    fn test_ilog2() {
        assert_eq!(dt_ilog2(1), 0);
        assert_eq!(dt_ilog2(2), 1);
        assert_eq!(dt_ilog2(4), 2);
        assert_eq!(dt_ilog2(8), 3);
        assert_eq!(dt_ilog2(16), 4);
        assert_eq!(dt_ilog2(17), 4);
        assert_eq!(dt_ilog2(32), 5);
    }
}
