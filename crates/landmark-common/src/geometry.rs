//! 2D geometry operations for
//!
//! This module provides 2D geometric operations primarily used for navigation mesh
//! generation and queries. Most operations work on the XZ plane (Y-up coordinate system).

use glam::Vec3;

/// Returns twice the signed area.
/// The sign indicates winding order:
/// - Positive: clockwise (looking down Y axis)
/// - Negative: counter-clockwise (looking down Y axis)
/// - Zero: collinear points
#[inline]
pub fn tri_area_2d(a: &[f32], b: &[f32], c: &[f32]) -> f32 {
    let abx = b[0] - a[0];
    let abz = b[2] - a[2];
    let acx = c[0] - a[0];
    let acz = c[2] - a[2];
    acx * abz - abx * acz
}

/// Vec3 variant of [`tri_area_2d`].
#[inline]
pub fn tri_area_2d_vec3(a: &Vec3, b: &Vec3, c: &Vec3) -> f32 {
    let abx = b.x - a.x;
    let abz = b.z - a.z;
    let acx = c.x - a.x;
    let acz = c.z - a.z;
    acx * abz - abx * acz
}

#[inline]
pub fn left(a: &[f32], b: &[f32], c: &[f32]) -> bool {
    tri_area_2d(a, b, c) < 0.0
}

#[inline]
pub fn left_on(a: &[f32], b: &[f32], c: &[f32]) -> bool {
    tri_area_2d(a, b, c) <= 0.0
}

#[inline]
pub fn right(a: &[f32], b: &[f32], c: &[f32]) -> bool {
    tri_area_2d(a, b, c) > 0.0
}

#[inline]
pub fn right_on(a: &[f32], b: &[f32], c: &[f32]) -> bool {
    tri_area_2d(a, b, c) >= 0.0
}

#[inline]
pub fn collinear(a: &[f32], b: &[f32], c: &[f32], eps: f32) -> bool {
    tri_area_2d(a, b, c).abs() <= eps
}

/// Assumes the points are collinear.
#[inline]
pub fn between(a: &[f32], b: &[f32], c: &[f32]) -> bool {
    if a[0] != c[0] {
        (a[0] <= b[0] && b[0] <= c[0]) || (c[0] <= b[0] && b[0] <= a[0])
    } else {
        (a[2] <= b[2] && b[2] <= c[2]) || (c[2] <= b[2] && b[2] <= a[2])
    }
}

#[inline]
pub fn overlap_bounds(amin: &[f32; 3], amax: &[f32; 3], bmin: &[f32; 3], bmax: &[f32; 3]) -> bool {
    amin[0] <= bmax[0]
        && amax[0] >= bmin[0]
        && amin[1] <= bmax[1]
        && amax[1] >= bmin[1]
        && amin[2] <= bmax[2]
        && amax[2] >= bmin[2]
}

#[inline]
pub fn overlap_bounds_2d(amin: &[f32], amax: &[f32], bmin: &[f32], bmax: &[f32]) -> bool {
    amin[0] <= bmax[0] && amax[0] >= bmin[0] && amin[2] <= bmax[2] && amax[2] >= bmin[2]
}

#[inline]
pub fn dist_sqr_2d(a: &[f32], b: &[f32]) -> f32 {
    let dx = b[0] - a[0];
    let dz = b[2] - a[2];
    dx * dx + dz * dz
}

#[inline]
pub fn dist_2d(a: &[f32], b: &[f32]) -> f32 {
    dist_sqr_2d(a, b).sqrt()
}

#[inline]
pub fn dist_sqr_2d_vec3(a: &Vec3, b: &Vec3) -> f32 {
    let dx = b.x - a.x;
    let dz = b.z - a.z;
    dx * dx + dz * dz
}

#[inline]
pub fn dist_2d_vec3(a: &Vec3, b: &Vec3) -> f32 {
    dist_sqr_2d_vec3(a, b).sqrt()
}

pub fn dist_point_segment_sqr_2d(p: &[f32], a: &[f32], b: &[f32]) -> f32 {
    let dx = b[0] - a[0];
    let dz = b[2] - a[2];
    let px = p[0] - a[0];
    let pz = p[2] - a[2];

    let d = dx * dx + dz * dz;
    if d < f32::EPSILON {
        // Segment is a point
        return px * px + pz * pz;
    }

    let t = (px * dx + pz * dz) / d;

    if t < 0.0 {
        // Before segment start
        px * px + pz * pz
    } else if t > 1.0 {
        // After segment end
        let px = p[0] - b[0];
        let pz = p[2] - b[2];
        px * px + pz * pz
    } else {
        // On segment
        let qx = a[0] + t * dx;
        let qz = a[2] + t * dz;
        let px = p[0] - qx;
        let pz = p[2] - qz;
        px * px + pz * pz
    }
}

#[inline]
pub fn dist_point_segment_2d(p: &[f32], a: &[f32], b: &[f32]) -> f32 {
    dist_point_segment_sqr_2d(p, a, b).sqrt()
}

pub fn closest_point_on_segment_2d(p: &[f32], a: &[f32], b: &[f32]) -> [f32; 3] {
    let dx = b[0] - a[0];
    let dz = b[2] - a[2];
    let px = p[0] - a[0];
    let pz = p[2] - a[2];

    let d = dx * dx + dz * dz;
    if d < f32::EPSILON {
        // Segment is a point
        return [a[0], a[1], a[2]];
    }

    let t = (px * dx + pz * dz) / d;
    let t = t.clamp(0.0, 1.0);

    [a[0] + t * dx, a[1], a[2] + t * dz]
}

/// Returns true if the segments intersect, including touching at endpoints.
pub fn intersect_segments_2d(a1: &[f32], a2: &[f32], b1: &[f32], b2: &[f32]) -> bool {
    let d1 = tri_area_2d(b1, b2, a1);
    let d2 = tri_area_2d(b1, b2, a2);
    let d3 = tri_area_2d(a1, a2, b1);
    let d4 = tri_area_2d(a1, a2, b2);

    if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
        return true;
    }

    // Check for touching endpoints
    if d1.abs() < f32::EPSILON && between(b1, a1, b2) {
        return true;
    }
    if d2.abs() < f32::EPSILON && between(b1, a2, b2) {
        return true;
    }
    if d3.abs() < f32::EPSILON && between(a1, b1, a2) {
        return true;
    }
    if d4.abs() < f32::EPSILON && between(a1, b2, a2) {
        return true;
    }

    false
}

/// Uses the winding number algorithm.
pub fn point_in_polygon_2d(p: &[f32], verts: &[f32], nverts: usize) -> bool {
    let mut winding = 0;

    for i in 0..nverts {
        let j = (i + 1) % nverts;
        let v1 = &verts[i * 3..];
        let v2 = &verts[j * 3..];

        if v1[2] <= p[2] {
            if v2[2] > p[2] && left(v1, v2, p) {
                winding += 1;
            }
        } else if v2[2] <= p[2] && right(v1, v2, p) {
            winding -= 1;
        }
    }

    winding != 0
}

#[inline]
pub fn perp_2d(x1: f32, z1: f32, x2: f32, z2: f32) -> f32 {
    x1 * z2 - z1 * x2
}

pub fn vec_perp_2d(a: &[f32], b: &[f32]) -> f32 {
    perp_2d(a[0], a[2], b[0], b[2])
}

/// Returns `(tmin, tmax, seg_min, seg_max)`:
/// - `tmin`/`tmax` - normalized distance along segment where it enters/exits polygon
/// - `seg_min`/`seg_max` - index of polygon edge where segment enters/exits
///
/// Returns `None` if no intersection. Only X and Z components of vertices are used.
pub fn intersect_segment_poly_2d(
    p0: &[f32],
    p1: &[f32],
    verts: &[f32],
    nverts: usize,
) -> Option<(f32, f32, i32, i32)> {
    const EPS: f32 = 0.000001;

    let mut tmin = 0.0;
    let mut tmax = 1.0;
    let mut seg_min = -1;
    let mut seg_max = -1;

    let dir = [p1[0] - p0[0], 0.0, p1[2] - p0[2]];

    for i in 0..nverts {
        let j = if i == 0 { nverts - 1 } else { i - 1 };

        let edge = [
            verts[i * 3] - verts[j * 3],
            0.0,
            verts[i * 3 + 2] - verts[j * 3 + 2],
        ];
        let diff = [p0[0] - verts[j * 3], 0.0, p0[2] - verts[j * 3 + 2]];

        let n = vec_perp_2d(&edge, &diff);
        let d = vec_perp_2d(&dir, &edge);

        if d.abs() < EPS {
            // Segment is nearly parallel to this edge
            if n < 0.0 {
                return None;
            } else {
                continue;
            }
        }

        let t = n / d;
        if d < 0.0 {
            // Segment is entering across this edge
            if t > tmin {
                tmin = t;
                seg_min = j as i32;
                // Segment enters after leaving polygon
                if tmin > tmax {
                    return None;
                }
            }
        } else {
            // Segment is leaving across this edge
            if t < tmax {
                tmax = t;
                seg_max = j as i32;
                // Segment leaves before entering polygon
                if tmax < tmin {
                    return None;
                }
            }
        }
    }

    Some((tmin, tmax, seg_min, seg_max))
}

/// Returns `(distance_squared, t)` where `t` is the parametric position on the segment.
pub fn dist_point_segment_sqr_2d_with_t(p: &[f32], a: &[f32], b: &[f32]) -> (f32, f32) {
    let dx = b[0] - a[0];
    let dz = b[2] - a[2];
    let dpx = p[0] - a[0];
    let dpz = p[2] - a[2];

    let d = dx * dx + dz * dz;
    let mut t = if d > 0.0 {
        (dpx * dx + dpz * dz) / d
    } else {
        0.0
    };

    // Clamp t to [0, 1]
    t = t.clamp(0.0, 1.0);

    let px = a[0] + t * dx;
    let pz = a[2] + t * dz;

    let dx2 = p[0] - px;
    let dz2 = p[2] - pz;

    (dx2 * dx2 + dz2 * dz2, t)
}

/// Returns `(inside, edge_dists, edge_ts)`:
/// - `inside` - true if the point is inside the polygon
/// - `edge_dists` - squared distance to each edge
/// - `edge_ts` - parametric position on each edge
pub fn distance_pt_poly_edges_sqr(
    pt: &[f32],
    verts: &[f32],
    nverts: usize,
) -> (bool, Vec<f32>, Vec<f32>) {
    let mut edge_dists = vec![0.0; nverts];
    let mut edge_ts = vec![0.0; nverts];

    // Point in polygon test using ray casting
    let mut inside = false;
    for i in 0..nverts {
        let j = if i == 0 { nverts - 1 } else { i - 1 };

        let vi = &verts[i * 3..];
        let vj = &verts[j * 3..];

        // Ray casting test for point in polygon
        if ((vi[2] > pt[2]) != (vj[2] > pt[2]))
            && (pt[0] < (vj[0] - vi[0]) * (pt[2] - vi[2]) / (vj[2] - vi[2]) + vi[0])
        {
            inside = !inside;
        }

        // Calculate distance to edge
        let (dist_sqr, t) = dist_point_segment_sqr_2d_with_t(pt, vj, vi);
        edge_dists[j] = dist_sqr;
        edge_ts[j] = t;
    }

    (inside, edge_dists, edge_ts)
}

#[inline]
pub fn overlap_segment_1d(amin: f32, amax: f32, bmin: f32, bmax: f32) -> f32 {
    let min = amin.max(bmin);
    let max = amax.min(bmax);
    (max - min).max(0.0)
}

pub fn project_poly_2d(axis: &[f32; 2], verts: &[f32], nverts: usize) -> (f32, f32) {
    let mut min = f32::MAX;
    let mut max = f32::MIN;

    for i in 0..nverts {
        let v = &verts[i * 3..];
        let proj = v[0] * axis[0] + v[2] * axis[1];
        min = min.min(proj);
        max = max.max(proj);
    }

    (min, max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tri_area_2d() {
        // Counter-clockwise triangle (negative in Y-up system)
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 0.0, 1.0];
        assert!(tri_area_2d(&a, &b, &c) < 0.0);

        // Clockwise triangle (positive in Y-up system)
        assert!(tri_area_2d(&a, &c, &b) > 0.0);

        // Collinear points
        let d = [2.0, 0.0, 0.0];
        assert_eq!(tri_area_2d(&a, &b, &d), 0.0);
    }

    #[test]
    fn test_orientation() {
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c_left = [0.5, 0.0, 1.0];
        let c_right = [0.5, 0.0, -1.0];
        let c_on = [0.5, 0.0, 0.0];

        assert!(left(&a, &b, &c_left));
        assert!(!left(&a, &b, &c_right));
        assert!(!left(&a, &b, &c_on));

        assert!(left_on(&a, &b, &c_left));
        assert!(!left_on(&a, &b, &c_right));
        assert!(left_on(&a, &b, &c_on));

        assert!(!right(&a, &b, &c_left));
        assert!(right(&a, &b, &c_right));
        assert!(!right(&a, &b, &c_on));
    }

    #[test]
    fn test_overlap_bounds() {
        let amin = [0.0, 0.0, 0.0];
        let amax = [2.0, 2.0, 2.0];
        let bmin = [1.0, 1.0, 1.0];
        let bmax = [3.0, 3.0, 3.0];
        let cmin = [3.0, 3.0, 3.0];
        let cmax = [4.0, 4.0, 4.0];

        assert!(overlap_bounds(&amin, &amax, &bmin, &bmax));
        assert!(!overlap_bounds(&amin, &amax, &cmin, &cmax));
    }

    #[test]
    fn test_dist_2d() {
        let a = [0.0, 0.0, 0.0];
        let b = [3.0, 0.0, 4.0];

        assert_eq!(dist_sqr_2d(&a, &b), 25.0);
        assert_eq!(dist_2d(&a, &b), 5.0);
    }
}
