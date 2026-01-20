//! Vector utilities for

use glam::Vec3;

/// Calculates the distance between two points
#[inline]
pub fn distance(a: &Vec3, b: &Vec3) -> f32 {
    (*b - *a).length()
}

/// Calculates the squared distance between two points
#[inline]
pub fn distance_squared(a: &Vec3, b: &Vec3) -> f32 {
    (*b - *a).length_squared()
}

/// Calculates the distance between a point and a line segment
pub fn distance_point_segment(p: &Vec3, a: &Vec3, b: &Vec3) -> f32 {
    let ab = *b - *a;
    let ap = *p - *a;

    let ab_len_sq = ab.length_squared();

    // If the line segment is just a point, return distance to that point
    if ab_len_sq < f32::EPSILON {
        return ap.length();
    }

    // Calculate projection of ap onto ab
    let t = ap.dot(ab) / ab_len_sq;

    if t < 0.0 {
        // Projection is beyond 'a', use distance to 'a'
        ap.length()
    } else if t > 1.0 {
        // Projection is beyond 'b', use distance to 'b'
        (*p - *b).length()
    } else {
        // Projection is on the segment, calculate perpendicular distance
        let projection = *a + ab * t;
        (*p - projection).length()
    }
}

/// Calculates the squared distance between a point and a line segment
pub fn distance_point_segment_squared(p: &Vec3, a: &Vec3, b: &Vec3) -> f32 {
    let ab = *b - *a;
    let ap = *p - *a;

    let ab_len_sq = ab.length_squared();

    // If the line segment is just a point, return distance to that point
    if ab_len_sq < f32::EPSILON {
        return ap.length_squared();
    }

    // Calculate projection of ap onto ab
    let t = ap.dot(ab) / ab_len_sq;

    if t < 0.0 {
        // Projection is beyond 'a', use distance to 'a'
        ap.length_squared()
    } else if t > 1.0 {
        // Projection is beyond 'b', use distance to 'b'
        (*p - *b).length_squared()
    } else {
        // Projection is on the segment, calculate perpendicular distance
        let projection = *a + ab * t;
        (*p - projection).length_squared()
    }
}

/// Finds the closest point on a line segment to a given point
pub fn closest_point_on_segment(p: &Vec3, a: &Vec3, b: &Vec3) -> Vec3 {
    let ab = *b - *a;
    let ap = *p - *a;

    let ab_len_sq = ab.length_squared();

    // If the line segment is just a point, return that point
    if ab_len_sq < f32::EPSILON {
        return *a;
    }

    // Calculate projection of ap onto ab
    let t = ap.dot(ab) / ab_len_sq;

    if t < 0.0 {
        // Projection is beyond 'a'
        *a
    } else if t > 1.0 {
        // Projection is beyond 'b'
        *b
    } else {
        // Projection is on the segment
        *a + ab * t
    }
}

/// Checks if a ray intersects a triangle
pub fn ray_triangle_intersection(
    origin: &Vec3,
    direction: &Vec3,
    v0: &Vec3,
    v1: &Vec3,
    v2: &Vec3,
) -> Option<f32> {
    // Möller–Trumbore algorithm
    let edge1 = *v1 - *v0;
    let edge2 = *v2 - *v0;

    let h = direction.cross(edge2);
    let a = edge1.dot(h);

    // This ray is parallel to the triangle
    if a.abs() < f32::EPSILON {
        return None;
    }

    let f = 1.0 / a;
    let s = *origin - *v0;
    let u = f * s.dot(h);

    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(edge1);
    let v = f * direction.dot(q);

    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    // At this stage we can compute t to find out where the intersection point is on the line
    let t = f * edge2.dot(q);

    if t > f32::EPSILON {
        return Some(t);
    }

    // No hit
    None
}
