//! Math utilities for

use glam::Vec3;
use std::f32::consts::PI;

/// Calculates the cross product of two 2D vectors [(x1,y1), (x2,y2)]
#[inline]
pub fn cross_2d(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    x1 * y2 - y1 * x2
}

/// Calculates the dot product of two 2D vectors [(x1,y1), (x2,y2)]
#[inline]
pub fn dot_2d(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    x1 * x2 + y1 * y2
}

/// Converts a floating point value to an integer by rounding
#[inline]
pub fn float_to_int(value: f32) -> i32 {
    value.round() as i32
}

/// Calculates the area of a triangle
pub fn triangle_area(a: &Vec3, b: &Vec3, c: &Vec3) -> f32 {
    let ab = *b - *a;
    let ac = *c - *a;
    let cross = ab.cross(ac);
    cross.length() * 0.5
}

/// Checks if a point is inside a triangle
pub fn point_in_triangle(p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> bool {
    // Compute vectors
    let v0 = *c - *a;
    let v1 = *b - *a;
    let v2 = *p - *a;

    // Compute dot products
    let dot00 = v0.dot(v0);
    let dot01 = v0.dot(v1);
    let dot02 = v0.dot(v2);
    let dot11 = v1.dot(v1);
    let dot12 = v1.dot(v2);

    // Compute barycentric coordinates
    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is in triangle
    (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0)
}

/// Converts degrees to radians
#[inline]
pub fn deg_to_rad(deg: f32) -> f32 {
    deg * PI / 180.0
}

/// Converts radians to degrees
#[inline]
pub fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / PI
}

/// Clamps a value between min and max
#[inline]
pub fn clamp<T: PartialOrd>(v: T, min: T, max: T) -> T {
    if v < min {
        min
    } else if v > max {
        max
    } else {
        v
    }
}

/// Square a value (x²)
#[inline]
pub fn sqr<T: std::ops::Mul<Output = T> + Copy>(x: T) -> T {
    x * x
}

/// Linear interpolation between two values
#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Get the next power of 2 greater than or equal to x
#[inline]
pub fn next_pow2(x: u32) -> u32 {
    if x == 0 {
        return 1;
    }
    let mut n = x - 1;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n + 1
}

/// Integer log base 2
#[inline]
pub fn ilog2(x: u32) -> u32 {
    if x == 0 {
        return 0;
    }
    31 - x.leading_zeros()
}

/// Align value to 4-byte boundary
#[inline]
pub fn align4(x: usize) -> usize {
    (x + 3) & !3
}

/// Component-wise minimum of two vectors
#[inline]
pub fn vmin(a: &Vec3, b: &Vec3) -> Vec3 {
    a.min(*b)
}

/// Component-wise maximum of two vectors
#[inline]
pub fn vmax(a: &Vec3, b: &Vec3) -> Vec3 {
    a.max(*b)
}

/// Component-wise minimum assignment
#[inline]
pub fn vmin_mut(dest: &mut Vec3, v: &Vec3) {
    *dest = dest.min(*v);
}

/// Component-wise maximum assignment
#[inline]
pub fn vmax_mut(dest: &mut Vec3, v: &Vec3) {
    *dest = dest.max(*v);
}

/// Vector multiply-add: dest = v1 + v2 * s
#[inline]
pub fn vmad(v1: &Vec3, v2: &Vec3, s: f32) -> Vec3 {
    *v1 + *v2 * s
}

/// Vector linear interpolation
#[inline]
pub fn vlerp(v1: &Vec3, v2: &Vec3, t: f32) -> Vec3 {
    v1.lerp(*v2, t)
}

/// Set vector components
#[inline]
pub fn vset(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z)
}

/// Zero out a vector (set all components to 0)
#[inline]
pub fn vzero(dest: &mut Vec3) {
    *dest = Vec3::ZERO;
}

/// Create a zero vector
#[inline]
pub fn vzero_new() -> Vec3 {
    Vec3::ZERO
}

/// Copy vector components from source to destination
#[inline]
pub fn vcopy(dest: &mut Vec3, src: &Vec3) {
    *dest = *src;
}

/// Scale a vector in place
#[inline]
pub fn vscale(dest: &mut Vec3, scale: f32) {
    *dest *= scale;
}

/// Add two vectors and store result in destination
#[inline]
pub fn vadd(dest: &mut Vec3, a: &Vec3, b: &Vec3) {
    *dest = *a + *b;
}

/// Subtract two vectors and store result in destination
#[inline]
pub fn vsub(dest: &mut Vec3, a: &Vec3, b: &Vec3) {
    *dest = *a - *b;
}

/// Normalize a vector in place
#[inline]
pub fn vnormalize(dest: &mut Vec3) {
    let len = dest.length();
    if len > 0.0 {
        *dest /= len;
    }
}

/// Find the closest point on a triangle to a given point
pub fn closest_point_on_triangle(p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> Vec3 {
    // Check if P in vertex region outside A
    let ab = *b - *a;
    let ac = *c - *a;
    let ap = *p - *a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a; // barycentric coordinates (1,0,0)
    }

    // Check if P in vertex region outside B
    let bp = *p - *b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b; // barycentric coordinates (0,1,0)
    }

    // Check if P in edge region of AB, if so return projection of P onto AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return *a + ab * v; // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    let cp = *p - *c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c; // barycentric coordinates (0,0,1)
    }

    // Check if P in edge region of AC, if so return projection of P onto AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return *a + ac * w; // barycentric coordinates (1-w,0,w)
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return *b + (*c - *b) * w; // barycentric coordinates (0,1-w,w)
    }

    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    *a + ab * v + ac * w // = u*a + v*b + w*c, u = va * denom = 1.0-v-w
}

/// Calculate the squared distance from a point to a triangle
pub fn distance_point_triangle_squared(p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> f32 {
    let closest = closest_point_on_triangle(p, a, b, c);
    (*p - closest).length_squared()
}

/// Check if a cylinder overlaps with a line segment
pub fn overlap_cylinder_segment(
    start: &Vec3,
    end: &Vec3,
    center: &Vec3,
    radius: f32,
    height: f32,
) -> bool {
    let d = *end - *start;
    let m = *start - *center;

    // Project the segment onto the cylinder's base plane (XZ plane)
    let d_xz = Vec3::new(d.x, 0.0, d.z);
    let m_xz = Vec3::new(m.x, 0.0, m.z);

    let a = d_xz.dot(d_xz);

    // Handle vertical segments (when a is zero or very small)
    if a < f32::EPSILON {
        // For vertical segments, just check if the XZ position is inside the cylinder
        let dist_sq = m_xz.x * m_xz.x + m_xz.z * m_xz.z;
        if dist_sq <= radius * radius {
            // Check if any part of the vertical segment is within the cylinder height
            let y_min = start.y.min(end.y);
            let y_max = start.y.max(end.y);
            return y_max >= center.y && y_min <= center.y + height;
        }
        return false;
    }

    let b = 2.0 * m_xz.dot(d_xz);
    let c = m_xz.dot(m_xz) - radius * radius;

    let discriminant = b * b - 4.0 * a * c;

    // No intersection with infinite cylinder
    if discriminant < 0.0 {
        return false;
    }

    // Check intersection points
    let sqrt_discriminant = discriminant.sqrt();
    let t1 = (-b - sqrt_discriminant) / (2.0 * a);
    let t2 = (-b + sqrt_discriminant) / (2.0 * a);

    // Check if any intersection point is within the segment and cylinder height
    for &t in &[t1, t2] {
        if (0.0..=1.0).contains(&t) {
            let y = start.y + t * d.y;
            if y >= center.y && y <= center.y + height {
                return true;
            }
        }
    }

    // Check if the segment endpoints are inside the cylinder
    let check_point_in_cylinder = |p: &Vec3| -> bool {
        let dx = p.x - center.x;
        let dz = p.z - center.z;
        let dist_sq = dx * dx + dz * dz;
        dist_sq <= radius * radius && p.y >= center.y && p.y <= center.y + height
    };

    check_point_in_cylinder(start) || check_point_in_cylinder(end)
}

/// Compute the 2D convex hull of a set of points using Graham's scan algorithm
pub fn convex_hull_2d(points: &mut [(f32, f32)]) -> Vec<(f32, f32)> {
    let n = points.len();
    if n < 3 {
        return points.to_vec();
    }

    // Find the bottom-most point (and left-most if tied)
    let mut bottom = 0;
    for i in 1..n {
        if points[i].1 < points[bottom].1
            || (points[i].1 == points[bottom].1 && points[i].0 < points[bottom].0)
        {
            bottom = i;
        }
    }
    points.swap(0, bottom);

    let pivot = points[0];

    // Sort points by polar angle with respect to pivot
    points[1..].sort_by(|a, b| {
        let angle_a = (a.1 - pivot.1).atan2(a.0 - pivot.0);
        let angle_b = (b.1 - pivot.1).atan2(b.0 - pivot.0);
        angle_a
            .partial_cmp(&angle_b)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Helper function to check if three points make a counter-clockwise turn
    let ccw = |p1: (f32, f32), p2: (f32, f32), p3: (f32, f32)| -> f32 {
        (p2.0 - p1.0) * (p3.1 - p1.1) - (p2.1 - p1.1) * (p3.0 - p1.0)
    };

    // Build the convex hull
    let mut hull = Vec::new();
    hull.push(points[0]);
    hull.push(points[1]);

    for &point in &points[2..n] {
        // Remove points that make a clockwise turn
        while hull.len() > 1 {
            let len = hull.len();
            if ccw(hull[len - 2], hull[len - 1], point) <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(point);
    }

    hull
}
