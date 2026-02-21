//! Detail mesh generation for Recast
//!
//! This module generates detailed meshes from polygon meshes, adding height
//! detail from the compact heightfield. Matches the C++ `rcBuildPolyMeshDetail`
//! and `buildPolyDetail` implementation.

use super::compact_heightfield::CompactHeightfield;
use super::polymesh::{MESH_NULL_IDX, PolyMesh, RC_MULTIPLE_REGS};
use super::triangle_utils::{get_dir_for_offset, get_dir_offset_x, get_dir_offset_z};
use crate::error::BuildError;

/// Unset height marker (matches C++ RC_UNSET_HEIGHT)
const RC_UNSET_HEIGHT: u16 = 0xffff;

/// Max vertices in local per-polygon buffer (matches C++ MAX_VERTS)
const MAX_VERTS: usize = 127;

/// Max vertices per edge sample (matches C++ MAX_VERTS_PER_EDGE)
const MAX_VERTS_PER_EDGE: usize = 32;

/// Max triangles per polygon (matches C++ MAX_TRIS)
const MAX_TRIS: usize = 255;

/// Edge value: undefined face (matches C++ EV_UNDEF)
const EV_UNDEF: i32 = -1;

/// Edge value: hull boundary (matches C++ EV_HULL)
const EV_HULL: i32 = -2;

/// Height patch for storing sampled heights
#[derive(Debug)]
struct HeightPatch {
    data: Vec<u16>,
    xmin: i32,
    ymin: i32,
    width: i32,
    height: i32,
}

impl HeightPatch {
    fn new() -> Self {
        Self {
            data: Vec::new(),
            xmin: 0,
            ymin: 0,
            width: 0,
            height: 0,
        }
    }
}

/// A detailed polygon mesh
#[derive(Debug, Clone)]
pub struct PolyMeshDetail {
    /// Vertices of the mesh [x,y,z,...]
    pub(crate) vertices: Vec<f32>,
    /// Triangle indices of the mesh, grouped by polygon
    /// Triangles are stored as 4 consecutive values: 3 indices + 1 flags byte
    pub(crate) triangles: Vec<u32>,
    /// Number of vertices in the mesh
    pub(crate) vert_count: usize,
    /// Number of triangles in the mesh
    pub(crate) tri_count: usize,
    /// Number of parent polygons for each submesh
    pub(crate) poly_count: usize,
    /// Starting indices in the triangles array for each polygon submesh
    pub(crate) poly_start: Vec<usize>,
    /// Number of triangles in each polygon submesh
    pub(crate) poly_tri_count: Vec<usize>,
}

impl Default for PolyMeshDetail {
    fn default() -> Self {
        Self::new()
    }
}

// ── Helper math functions (matches C++ static helpers) ──────────────────────

/// 2D dot product (XZ plane)
fn vdot2(a: &[f32], b: &[f32]) -> f32 {
    a[0] * b[0] + a[2] * b[2]
}

/// 2D squared distance (XZ plane)
fn vdist_sq2(p: &[f32], q: &[f32]) -> f32 {
    let dx = q[0] - p[0];
    let dz = q[2] - p[2];
    dx * dx + dz * dz
}

/// 2D distance (XZ plane)
fn vdist2(p: &[f32], q: &[f32]) -> f32 {
    vdist_sq2(p, q).sqrt()
}

/// 2D cross product (matches C++ vcross2)
fn vcross2(p1: &[f32], p2: &[f32], p3: &[f32]) -> f32 {
    let u1 = p2[0] - p1[0];
    let v1 = p2[2] - p1[2];
    let u2 = p3[0] - p1[0];
    let v2 = p3[2] - p1[2];
    u1 * v2 - v1 * u2
}

/// 3D distance from point to segment (matches C++ distancePtSeg)
fn distance_pt_seg(pt: &[f32], p: &[f32], q: &[f32]) -> f32 {
    let pqx = q[0] - p[0];
    let pqy = q[1] - p[1];
    let pqz = q[2] - p[2];
    let dx = pt[0] - p[0];
    let dy = pt[1] - p[1];
    let dz = pt[2] - p[2];
    let d = pqx * pqx + pqy * pqy + pqz * pqz;
    let mut t = pqx * dx + pqy * dy + pqz * dz;
    if d > 0.0 {
        t /= d;
    }
    t = t.clamp(0.0, 1.0);

    let rx = p[0] + t * pqx - pt[0];
    let ry = p[1] + t * pqy - pt[1];
    let rz = p[2] + t * pqz - pt[2];
    rx * rx + ry * ry + rz * rz
}

/// 2D distance from point to segment (matches C++ distancePtSeg2d)
fn distance_pt_seg_2d(pt: &[f32], p: &[f32], q: &[f32]) -> f32 {
    let pqx = q[0] - p[0];
    let pqz = q[2] - p[2];
    let dx = pt[0] - p[0];
    let dz = pt[2] - p[2];
    let d = pqx * pqx + pqz * pqz;
    let mut t = pqx * dx + pqz * dz;
    if d > 0.0 {
        t /= d;
    }
    t = t.clamp(0.0, 1.0);

    let rx = p[0] + t * pqx - pt[0];
    let rz = p[2] + t * pqz - pt[2];
    rx * rx + rz * rz
}

/// Calculate distance from point to triangle (matches C++ distPtTri)
fn dist_pt_tri(p: &[f32], a: &[f32], b: &[f32], c: &[f32]) -> f32 {
    let v0 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let v1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v2 = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];

    let dot00 = vdot2(&v0, &v0);
    let dot01 = vdot2(&v0, &v1);
    let dot02 = vdot2(&v0, &v2);
    let dot11 = vdot2(&v1, &v1);
    let dot12 = vdot2(&v1, &v2);

    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is in triangle
    let eps: f32 = 1e-4;
    if u >= -eps && v >= -eps && (u + v) <= 1.0 + eps {
        let y = a[1] + v0[1] * u + v1[1] * v;
        return (y - p[1]).abs();
    }
    f32::MAX
}

/// Distance from point to triangle mesh (matches C++ distToTriMesh)
/// Returns -1 if point doesn't project onto any triangle
fn dist_to_tri_mesh(p: &[f32], verts: &[f32], tris: &[i32]) -> f32 {
    let ntris = tris.len() / 4;
    let mut dmin = f32::MAX;
    for i in 0..ntris {
        let va = &verts[tris[i * 4] as usize * 3..];
        let vb = &verts[tris[i * 4 + 1] as usize * 3..];
        let vc = &verts[tris[i * 4 + 2] as usize * 3..];
        let d = dist_pt_tri(p, va, vb, vc);
        if d < dmin {
            dmin = d;
        }
    }
    if dmin == f32::MAX { -1.0 } else { dmin }
}

/// Signed distance from point to polygon boundary (matches C++ distToPoly)
/// Returns negative if inside the polygon
fn dist_to_poly(nvert: usize, verts: &[f32], p: &[f32]) -> f32 {
    let mut dmin = f32::MAX;
    let mut c = false;
    let mut j = nvert - 1;
    for i in 0..nvert {
        let vi = &verts[i * 3..];
        let vj = &verts[j * 3..];
        if ((vi[2] > p[2]) != (vj[2] > p[2]))
            && (p[0] < (vj[0] - vi[0]) * (p[2] - vi[2]) / (vj[2] - vi[2]) + vi[0])
        {
            c = !c;
        }
        dmin = dmin.min(distance_pt_seg_2d(p, vj, vi));
        j = i;
    }
    if c { -dmin } else { dmin }
}

/// Calculate minimum extent of polygon (matches C++ polyMinExtent)
fn poly_min_extent(verts: &[f32], nverts: usize) -> f32 {
    let mut min_dist = f32::MAX;
    for i in 0..nverts {
        let ni = (i + 1) % nverts;
        let p1 = &verts[i * 3..];
        let p2 = &verts[ni * 3..];
        let mut max_edge_dist: f32 = 0.0;
        for j in 0..nverts {
            if j == i || j == ni {
                continue;
            }
            let d = distance_pt_seg_2d(&verts[j * 3..], p1, p2);
            max_edge_dist = max_edge_dist.max(d);
        }
        min_dist = min_dist.min(max_edge_dist);
    }
    min_dist.sqrt()
}

/// Circumcircle of three points (matches C++ circumCircle)
fn circum_circle(p1: &[f32], p2: &[f32], p3: &[f32], c: &mut [f32; 3], r: &mut f32) -> bool {
    const EPS: f32 = 1e-6;

    let v1 = [0.0f32, 0.0, 0.0];
    let v2 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
    let v3 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]];

    let cp = vcross2(&v1, &v2, &v3);
    if cp.abs() > EPS {
        let v1_sq = vdot2(&v1, &v1);
        let v2_sq = vdot2(&v2, &v2);
        let v3_sq = vdot2(&v3, &v3);

        c[0] = (v1_sq * (v2[2] - v3[2]) + v2_sq * (v3[2] - v1[2]) + v3_sq * (v1[2] - v2[2]))
            / (2.0 * cp);
        c[1] = 0.0;
        c[2] = (v1_sq * (v3[0] - v2[0]) + v2_sq * (v1[0] - v3[0]) + v3_sq * (v2[0] - v1[0]))
            / (2.0 * cp);

        *r = vdist2(c, &v1);

        c[0] += p1[0];
        c[1] += p1[1];
        c[2] += p1[2];

        true
    } else {
        false
    }
}

/// Jitter X for sample position (matches C++ getJitterX)
fn get_jitter_x(i: i32) -> f32 {
    ((i.wrapping_mul(0x8da6b343_u32 as i32) as u32 & 0xffff) as f32 / 65535.0 * 2.0) - 1.0
}

/// Jitter Y for sample position (matches C++ getJitterY)
fn get_jitter_y(i: i32) -> f32 {
    ((i.wrapping_mul(0xd8163841_u32 as i32) as u32 & 0xffff) as f32 / 65535.0 * 2.0) - 1.0
}

/// Get height from height patch (matches C++ getHeight)
fn get_height(fx: f32, fy: f32, _fz: f32, ics: f32, ch: f32, radius: i32, hp: &HeightPatch) -> u16 {
    let mut ix = (fx * ics + 0.01).floor() as i32;
    let mut iz = (_fz * ics + 0.01).floor() as i32;
    ix = (ix - hp.xmin).clamp(0, hp.width - 1);
    iz = (iz - hp.ymin).clamp(0, hp.height - 1);

    let mut h = hp.data[(ix + iz * hp.width) as usize];
    if h == RC_UNSET_HEIGHT {
        // Spiral search for valid height
        let mut x = 1i32;
        let mut z = 0i32;
        let mut dx = 1i32;
        let mut dz = 0i32;
        let max_size = radius * 2 + 1;
        let max_iter = max_size * max_size - 1;

        let mut next_ring_iter_start = 8;
        let mut next_ring_iters = 16;

        let mut dmin = f32::MAX;
        for i in 0..max_iter {
            let nx = ix + x;
            let nz = iz + z;

            if nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height {
                let nh = hp.data[(nx + nz * hp.width) as usize];
                if nh != RC_UNSET_HEIGHT {
                    let d = (nh as f32 * ch - fy).abs();
                    if d < dmin {
                        h = nh;
                        dmin = d;
                    }
                }
            }

            if i + 1 == next_ring_iter_start {
                if h != RC_UNSET_HEIGHT {
                    break;
                }
                next_ring_iter_start += next_ring_iters;
                next_ring_iters += 8;
            }

            if (x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1 - z)) {
                let tmp = dx;
                dx = -dz;
                dz = tmp;
            }
            x += dx;
            z += dz;
        }
    }
    h
}

// ── Edge management for Delaunay triangulation ──────────────────────────────

/// Find edge in edge list (matches C++ findEdge)
fn find_edge(edges: &[i32], nedges: usize, s: i32, t: i32) -> i32 {
    for i in 0..nedges {
        let e = &edges[i * 4..];
        if (e[0] == s && e[1] == t) || (e[0] == t && e[1] == s) {
            return i as i32;
        }
    }
    EV_UNDEF
}

/// Add edge to edge list (matches C++ addEdge)
fn add_edge(
    edges: &mut [i32],
    nedges: &mut usize,
    max_edges: usize,
    s: i32,
    t: i32,
    l: i32,
    r: i32,
) -> i32 {
    if *nedges >= max_edges {
        return EV_UNDEF;
    }

    let e = find_edge(edges, *nedges, s, t);
    if e == EV_UNDEF {
        let idx = *nedges;
        edges[idx * 4] = s;
        edges[idx * 4 + 1] = t;
        edges[idx * 4 + 2] = l;
        edges[idx * 4 + 3] = r;
        *nedges += 1;
        idx as i32
    } else {
        EV_UNDEF
    }
}

/// Update left face of an edge (matches C++ updateLeftFace)
fn update_left_face(edges: &mut [i32], e_idx: usize, s: i32, t: i32, f: i32) {
    let e = &mut edges[e_idx * 4..e_idx * 4 + 4];
    if e[0] == s && e[1] == t && e[2] == EV_UNDEF {
        e[2] = f;
    } else if e[1] == s && e[0] == t && e[3] == EV_UNDEF {
        e[3] = f;
    }
}

/// Check if two 2D segments overlap (matches C++ overlapSegSeg2d)
fn overlap_seg_seg_2d(a: &[f32], b: &[f32], c: &[f32], d: &[f32]) -> bool {
    let a1 = vcross2(a, b, d);
    let a2 = vcross2(a, b, c);
    if a1 * a2 < 0.0 {
        let a3 = vcross2(c, d, a);
        let a4 = a3 + a2 - a1;
        if a3 * a4 < 0.0 {
            return true;
        }
    }
    false
}

/// Check if edge overlaps any existing edges (matches C++ overlapEdges)
fn overlap_edges(pts: &[f32], edges: &[i32], nedges: usize, s1: i32, t1: i32) -> bool {
    for i in 0..nedges {
        let s0 = edges[i * 4];
        let t0 = edges[i * 4 + 1];
        // Same or connected edges do not overlap
        if s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1 {
            continue;
        }
        if overlap_seg_seg_2d(
            &pts[s0 as usize * 3..],
            &pts[t0 as usize * 3..],
            &pts[s1 as usize * 3..],
            &pts[t1 as usize * 3..],
        ) {
            return true;
        }
    }
    false
}

/// Complete a facet of the Delaunay triangulation (matches C++ completeFacet)
fn complete_facet(
    pts: &[f32],
    npts: usize,
    edges: &mut Vec<i32>,
    nedges: &mut usize,
    max_edges: usize,
    nfaces: &mut usize,
    e: usize,
) {
    const EPS: f32 = 1e-5;

    let (s, t) = {
        let edge = &edges[e * 4..e * 4 + 4];
        if edge[2] == EV_UNDEF {
            (edge[0], edge[1])
        } else if edge[3] == EV_UNDEF {
            (edge[1], edge[0])
        } else {
            return; // Edge already completed
        }
    };

    // Find best point on left of edge
    let mut pt = npts as i32;
    let mut c = [0.0f32; 3];
    let mut r: f32 = -1.0;
    for u in 0..npts as i32 {
        if u == s || u == t {
            continue;
        }
        if vcross2(
            &pts[s as usize * 3..],
            &pts[t as usize * 3..],
            &pts[u as usize * 3..],
        ) > EPS
        {
            if r < 0.0 {
                pt = u;
                circum_circle(
                    &pts[s as usize * 3..],
                    &pts[t as usize * 3..],
                    &pts[u as usize * 3..],
                    &mut c,
                    &mut r,
                );
                continue;
            }
            let d = vdist2(&c, &pts[u as usize * 3..]);
            let tol = 0.001;
            if d > r * (1.0 + tol) {
                continue;
            } else if d < r * (1.0 - tol) {
                pt = u;
                circum_circle(
                    &pts[s as usize * 3..],
                    &pts[t as usize * 3..],
                    &pts[u as usize * 3..],
                    &mut c,
                    &mut r,
                );
            } else {
                if overlap_edges(pts, edges, *nedges, s, u) {
                    continue;
                }
                if overlap_edges(pts, edges, *nedges, t, u) {
                    continue;
                }
                pt = u;
                circum_circle(
                    &pts[s as usize * 3..],
                    &pts[t as usize * 3..],
                    &pts[u as usize * 3..],
                    &mut c,
                    &mut r,
                );
            }
        }
    }

    if pt < npts as i32 {
        update_left_face(edges, e, s, t, *nfaces as i32);

        let e2 = find_edge(edges, *nedges, pt, s);
        if e2 == EV_UNDEF {
            add_edge(edges, nedges, max_edges, pt, s, *nfaces as i32, EV_UNDEF);
        } else {
            update_left_face(edges, e2 as usize, pt, s, *nfaces as i32);
        }

        let e3 = find_edge(edges, *nedges, t, pt);
        if e3 == EV_UNDEF {
            add_edge(edges, nedges, max_edges, t, pt, *nfaces as i32, EV_UNDEF);
        } else {
            update_left_face(edges, e3 as usize, t, pt, *nfaces as i32);
        }

        *nfaces += 1;
    } else {
        update_left_face(edges, e, s, t, EV_HULL);
    }
}

/// Delaunay triangulation from hull (matches C++ delaunayHull)
fn delaunay_hull(
    npts: usize,
    pts: &[f32],
    nhull: usize,
    hull: &[i32],
    tris: &mut Vec<i32>,
    edges: &mut Vec<i32>,
) {
    let mut nfaces = 0usize;
    let mut nedges = 0usize;
    let max_edges = npts * 10;
    edges.resize(max_edges * 4, 0);

    let mut j = nhull - 1;
    for i in 0..nhull {
        add_edge(
            edges,
            &mut nedges,
            max_edges,
            hull[j],
            hull[i],
            EV_HULL,
            EV_UNDEF,
        );
        j = i;
    }

    let mut current_edge = 0;
    while current_edge < nedges {
        if edges[current_edge * 4 + 2] == EV_UNDEF {
            complete_facet(
                pts,
                npts,
                edges,
                &mut nedges,
                max_edges,
                &mut nfaces,
                current_edge,
            );
        }
        if edges[current_edge * 4 + 3] == EV_UNDEF {
            complete_facet(
                pts,
                npts,
                edges,
                &mut nedges,
                max_edges,
                &mut nfaces,
                current_edge,
            );
        }
        current_edge += 1;
    }

    // Create tris
    tris.resize(nfaces * 4, -1);

    for i in 0..nedges {
        let e = &edges[i * 4..i * 4 + 4];
        if e[3] >= 0 {
            let t_idx = e[3] as usize * 4;
            if tris[t_idx] == -1 {
                tris[t_idx] = e[0];
                tris[t_idx + 1] = e[1];
            } else if tris[t_idx] == e[1] {
                tris[t_idx + 2] = e[0];
            } else if tris[t_idx + 1] == e[0] {
                tris[t_idx + 2] = e[1];
            }
        }
        if e[2] >= 0 {
            let t_idx = e[2] as usize * 4;
            if tris[t_idx] == -1 {
                tris[t_idx] = e[1];
                tris[t_idx + 1] = e[0];
            } else if tris[t_idx] == e[0] {
                tris[t_idx + 2] = e[1];
            } else if tris[t_idx + 1] == e[1] {
                tris[t_idx + 2] = e[0];
            }
        }
    }

    // Remove dangling faces
    let mut i = 0;
    while i < tris.len() / 4 {
        let t = i * 4;
        if tris[t] == -1 || tris[t + 1] == -1 || tris[t + 2] == -1 {
            let last = tris.len() - 4;
            tris[t] = tris[last];
            tris[t + 1] = tris[last + 1];
            tris[t + 2] = tris[last + 2];
            tris[t + 3] = tris[last + 3];
            tris.truncate(tris.len() - 4);
        } else {
            i += 1;
        }
    }
}

/// Triangulate hull using shortest-perimeter ear heuristic (matches C++ triangulateHull)
fn triangulate_hull(verts: &[f32], nhull: usize, hull: &[i32], nin: usize, tris: &mut Vec<i32>) {
    let mut start = 0i32;
    let mut left = 1i32;
    let mut right = nhull as i32 - 1;

    // Start from an ear with shortest perimeter
    let mut dmin = f32::MAX;
    for i in 0..nhull as i32 {
        if hull[i as usize] >= nin as i32 {
            continue;
        }
        let pi = if i - 1 >= 0 { i - 1 } else { nhull as i32 - 1 };
        let ni = if i + 1 < nhull as i32 { i + 1 } else { 0 };
        let pv = &verts[hull[pi as usize] as usize * 3..];
        let cv = &verts[hull[i as usize] as usize * 3..];
        let nv = &verts[hull[ni as usize] as usize * 3..];
        let d = vdist2(pv, cv) + vdist2(cv, nv) + vdist2(nv, pv);
        if d < dmin {
            start = i;
            left = ni;
            right = pi;
            dmin = d;
        }
    }

    // Add first triangle
    tris.push(hull[start as usize]);
    tris.push(hull[left as usize]);
    tris.push(hull[right as usize]);
    tris.push(0); // flags

    // Triangulate by moving left or right based on shorter perimeter
    while {
        let nleft = if left + 1 < nhull as i32 { left + 1 } else { 0 };
        nleft != right
    } {
        let nleft = if left + 1 < nhull as i32 { left + 1 } else { 0 };
        let nright = if right - 1 >= 0 {
            right - 1
        } else {
            nhull as i32 - 1
        };

        let cvleft = &verts[hull[left as usize] as usize * 3..];
        let nvleft = &verts[hull[nleft as usize] as usize * 3..];
        let cvright = &verts[hull[right as usize] as usize * 3..];
        let nvright = &verts[hull[nright as usize] as usize * 3..];
        let dleft = vdist2(cvleft, nvleft) + vdist2(nvleft, cvright);
        let dright = vdist2(cvright, nvright) + vdist2(cvleft, nvright);

        if dleft < dright {
            tris.push(hull[left as usize]);
            tris.push(hull[nleft as usize]);
            tris.push(hull[right as usize]);
            tris.push(0);
            left = nleft;
        } else {
            tris.push(hull[left as usize]);
            tris.push(hull[nright as usize]);
            tris.push(hull[right as usize]);
            tris.push(0);
            right = nright;
        }
    }
}

/// Check if edge a-b is on the hull (matches C++ onHull)
fn on_hull(a: i32, b: i32, nhull: usize, hull: &[i32]) -> bool {
    if a as usize >= nhull || b as usize >= nhull {
        return false;
    }
    let mut j = nhull - 1;
    for i in 0..nhull {
        if a == hull[j] && b == hull[i] {
            return true;
        }
        j = i;
    }
    false
}

/// Set triangle edge flags (matches C++ setTriFlags)
fn set_tri_flags(tris: &mut [i32], nhull: usize, hull: &[i32]) {
    const DETAIL_EDGE_BOUNDARY: i32 = 0x1;
    let ntris = tris.len() / 4;
    for i in 0..ntris {
        let a = tris[i * 4];
        let b = tris[i * 4 + 1];
        let c = tris[i * 4 + 2];
        let mut flags = 0u16;
        if on_hull(a, b, nhull, hull) {
            flags |= DETAIL_EDGE_BOUNDARY as u16;
        }
        if on_hull(b, c, nhull, hull) {
            flags |= (DETAIL_EDGE_BOUNDARY as u16) << 2;
        }
        if on_hull(c, a, nhull, hull) {
            flags |= (DETAIL_EDGE_BOUNDARY as u16) << 4;
        }
        tris[i * 4 + 3] = flags as i32;
    }
}

// ── buildPolyDetail (matches C++ buildPolyDetail) ───────────────────────────

/// Build detail mesh for a single polygon (matches C++ buildPolyDetail)
#[allow(clippy::too_many_arguments)]
fn build_poly_detail(
    input: &[f32], // Polygon boundary vertices (world-relative, nin*3 floats)
    nin: usize,    // Number of boundary vertices
    sample_dist: f32,
    sample_max_error: f32,
    height_search_radius: i32,
    chf: &CompactHeightfield,
    hp: &HeightPatch,
    verts: &mut [f32], // Output: local vertex buffer (MAX_VERTS*3)
    nverts: &mut usize,
    tris: &mut Vec<i32>,
    edges: &mut Vec<i32>,
    samples: &mut Vec<i32>,
) -> bool {
    let cs = chf.cs;
    let ics = 1.0 / cs;

    // Phase 1: Copy boundary vertices
    *nverts = nin;
    for i in 0..nin {
        verts[i * 3] = input[i * 3];
        verts[i * 3 + 1] = input[i * 3 + 1];
        verts[i * 3 + 2] = input[i * 3 + 2];
    }

    tris.clear();

    let mut hull = vec![0i32; MAX_VERTS];
    let mut nhull = 0usize;

    // Phase 2: Edge tessellation
    let mut edge = [0.0f32; MAX_VERTS_PER_EDGE * 3];

    if sample_dist > 0.0 {
        let mut j = nin - 1;
        for i in 0..nin {
            let mut vj_idx = j;
            let mut vi_idx = i;
            let mut swapped = false;

            // Lexicographic sort to ensure consistent edge ordering
            if (input[vj_idx * 3] - input[vi_idx * 3]).abs() < 1e-6 {
                if input[vj_idx * 3 + 2] > input[vi_idx * 3 + 2] {
                    std::mem::swap(&mut vj_idx, &mut vi_idx);
                    swapped = true;
                }
            } else if input[vj_idx * 3] > input[vi_idx * 3] {
                std::mem::swap(&mut vj_idx, &mut vi_idx);
                swapped = true;
            }

            let vj = &input[vj_idx * 3..vj_idx * 3 + 3];
            let vi = &input[vi_idx * 3..vi_idx * 3 + 3];

            let dx = vi[0] - vj[0];
            let dy = vi[1] - vj[1];
            let dz = vi[2] - vj[2];
            let d = (dx * dx + dz * dz).sqrt();
            let mut nn = 1 + (d / sample_dist).floor() as usize;
            if nn >= MAX_VERTS_PER_EDGE {
                nn = MAX_VERTS_PER_EDGE - 1;
            }
            if *nverts + nn >= MAX_VERTS {
                nn = MAX_VERTS - 1 - *nverts;
            }

            for k in 0..=nn {
                let u = k as f32 / nn as f32;
                let pos = &mut edge[k * 3..k * 3 + 3];
                pos[0] = vj[0] + dx * u;
                pos[1] = vj[1] + dy * u;
                pos[2] = vj[2] + dz * u;
                pos[1] = get_height(
                    pos[0],
                    pos[1],
                    pos[2],
                    ics,
                    chf.ch,
                    height_search_radius,
                    hp,
                ) as f32
                    * chf.ch;
            }

            // Simplify edge samples (Douglas-Peucker style)
            let mut idx = [0i32; MAX_VERTS_PER_EDGE];
            idx[0] = 0;
            idx[1] = nn as i32;
            let mut nidx = 2usize;

            let mut k = 0;
            while k < nidx - 1 {
                let a = idx[k] as usize;
                let b = idx[k + 1] as usize;
                let va = &edge[a * 3..a * 3 + 3];
                let vb = &edge[b * 3..b * 3 + 3];

                let mut maxd: f32 = 0.0;
                let mut maxi: i32 = -1;
                for m in (a + 1)..b {
                    let dev = distance_pt_seg(&edge[m * 3..m * 3 + 3], va, vb);
                    if dev > maxd {
                        maxd = dev;
                        maxi = m as i32;
                    }
                }

                if maxi != -1 && maxd > sample_max_error * sample_max_error {
                    // Insert point
                    for m in (k + 1..nidx).rev() {
                        idx[m + 1] = idx[m];
                    }
                    idx[k + 1] = maxi;
                    nidx += 1;
                } else {
                    k += 1;
                }
            }

            hull[nhull] = j as i32;
            nhull += 1;

            // Add new vertices
            if swapped {
                for k in (1..nidx - 1).rev() {
                    let src = idx[k] as usize * 3;
                    verts[*nverts * 3] = edge[src];
                    verts[*nverts * 3 + 1] = edge[src + 1];
                    verts[*nverts * 3 + 2] = edge[src + 2];
                    hull[nhull] = *nverts as i32;
                    nhull += 1;
                    *nverts += 1;
                }
            } else {
                for k in 1..nidx - 1 {
                    let src = idx[k] as usize * 3;
                    verts[*nverts * 3] = edge[src];
                    verts[*nverts * 3 + 1] = edge[src + 1];
                    verts[*nverts * 3 + 2] = edge[src + 2];
                    hull[nhull] = *nverts as i32;
                    nhull += 1;
                    *nverts += 1;
                }
            }

            j = i;
        }
    }

    // If no edge tessellation, hull is just the original boundary
    if nhull == 0 {
        for i in 0..nin {
            hull[i] = i as i32;
        }
        nhull = nin;
    }

    // Check polygon minimum extent for sliver detection
    if sample_dist > 0.0 {
        let min_extent = poly_min_extent(input, nin);
        if min_extent < sample_dist * 2.0 {
            triangulate_hull(verts, nhull, &hull, nin, tris);
            set_tri_flags(tris, nhull, &hull);
            return true;
        }
    }

    // Phase 3: Initial triangulation
    triangulate_hull(verts, nhull, &hull, nin, tris);

    if tris.is_empty() {
        return true;
    }

    // Phase 4: Interior sampling
    if sample_dist > 0.0 {
        // Create sample locations in a grid
        let mut bmin = [input[0], input[1], input[2]];
        let mut bmax = [input[0], input[1], input[2]];
        for i in 1..nin {
            bmin[0] = bmin[0].min(input[i * 3]);
            bmin[1] = bmin[1].min(input[i * 3 + 1]);
            bmin[2] = bmin[2].min(input[i * 3 + 2]);
            bmax[0] = bmax[0].max(input[i * 3]);
            bmax[1] = bmax[1].max(input[i * 3 + 1]);
            bmax[2] = bmax[2].max(input[i * 3 + 2]);
        }

        let x0 = (bmin[0] / sample_dist).floor() as i32;
        let x1 = (bmax[0] / sample_dist).ceil() as i32;
        let z0 = (bmin[2] / sample_dist).floor() as i32;
        let z1 = (bmax[2] / sample_dist).ceil() as i32;

        samples.clear();
        for z in z0..z1 {
            for x in x0..x1 {
                let pt = [
                    x as f32 * sample_dist,
                    (bmax[1] + bmin[1]) * 0.5,
                    z as f32 * sample_dist,
                ];
                // Make sure samples are not too close to the edges
                if dist_to_poly(nin, input, &pt) > -sample_dist / 2.0 {
                    continue;
                }
                samples.push(x);
                samples.push(
                    get_height(pt[0], pt[1], pt[2], ics, chf.ch, height_search_radius, hp) as i32,
                );
                samples.push(z);
                samples.push(0); // not added flag
            }
        }

        // Add samples starting from the one with most error
        let nsamples = samples.len() / 4;
        for _iter in 0..nsamples {
            if *nverts >= MAX_VERTS {
                break;
            }

            // Find sample with most error
            let mut bestpt = [0.0f32; 3];
            let mut bestd: f32 = 0.0;
            let mut besti: i32 = -1;

            for si in 0..nsamples {
                let s = &samples[si * 4..si * 4 + 4];
                if s[3] != 0 {
                    continue; // skip added
                }
                let pt = [
                    s[0] as f32 * sample_dist + get_jitter_x(si as i32) * cs * 0.1,
                    s[1] as f32 * chf.ch,
                    s[2] as f32 * sample_dist + get_jitter_y(si as i32) * cs * 0.1,
                ];
                let d = dist_to_tri_mesh(&pt, &verts[..(*nverts * 3)], tris);
                if d < 0.0 {
                    continue;
                }
                if d > bestd {
                    bestd = d;
                    besti = si as i32;
                    bestpt = pt;
                }
            }

            if bestd <= sample_max_error || besti == -1 {
                break;
            }

            // Mark sample as added
            samples[besti as usize * 4 + 3] = 1;

            // Add the new sample point
            verts[*nverts * 3] = bestpt[0];
            verts[*nverts * 3 + 1] = bestpt[1];
            verts[*nverts * 3 + 2] = bestpt[2];
            *nverts += 1;

            // Rebuild triangulation
            edges.clear();
            tris.clear();
            delaunay_hull(*nverts, verts, nhull, &hull, tris, edges);
        }
    }

    let ntris = tris.len() / 4;
    if ntris > MAX_TRIS {
        tris.truncate(MAX_TRIS * 4);
    }

    set_tri_flags(tris, nhull, &hull);
    true
}

// ── PolyMeshDetail impl ────────────────────────────────────────────────────

impl PolyMeshDetail {
    /// Creates a new empty detailed polygon mesh
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            vert_count: 0,
            tri_count: 0,
            poly_count: 0,
            poly_start: Vec::new(),
            poly_tri_count: Vec::new(),
        }
    }

    /// Returns the detail vertices as a flat `[x, y, z, ...]` slice.
    pub fn vertices(&self) -> &[f32] {
        &self.vertices
    }

    /// Returns the triangle indices as a flat slice (4 values per tri: v0, v1, v2, flags).
    pub fn triangles(&self) -> &[u32] {
        &self.triangles
    }

    /// Returns the number of detail vertices.
    pub fn vert_count(&self) -> usize {
        self.vert_count
    }

    /// Returns the number of detail triangles.
    pub fn tri_count(&self) -> usize {
        self.tri_count
    }

    /// Returns the number of parent polygons.
    pub fn poly_count(&self) -> usize {
        self.poly_count
    }

    /// Returns the starting triangle index for each polygon submesh.
    pub fn poly_start(&self) -> &[usize] {
        &self.poly_start
    }

    /// Returns the triangle count for each polygon submesh.
    pub fn poly_tri_count(&self) -> &[usize] {
        &self.poly_tri_count
    }

    /// Builds a detailed polygon mesh from a polygon mesh and compact heightfield
    /// (matches C++ rcBuildPolyMeshDetail)
    pub fn build_from_poly_mesh(
        poly_mesh: &PolyMesh,
        chf: &CompactHeightfield,
        sample_dist: f32,
        sample_max_error: f32,
    ) -> Result<Self, BuildError> {
        if poly_mesh.nverts == 0 || poly_mesh.npolys == 0 {
            return Ok(Self::new());
        }

        let nvp = poly_mesh.nvp;
        let cs = poly_mesh.cs;
        let ch = poly_mesh.ch;
        let orig = poly_mesh.bmin;
        let border_size = poly_mesh.border_size;
        let height_search_radius = 1.max((poly_mesh.max_edge_error + 0.999) as i32);

        // Allocate output
        let mut dmesh = Self::new();
        dmesh.poly_count = poly_mesh.npolys;
        dmesh.poly_start.resize(poly_mesh.npolys + 1, 0);
        dmesh.poly_tri_count.resize(poly_mesh.npolys, 0);

        // Temp buffers (reused across polygons)
        let mut verts = vec![0.0f32; MAX_VERTS * 3];
        let mut tris: Vec<i32> = Vec::with_capacity(MAX_TRIS * 4);
        let mut edges: Vec<i32> = Vec::new();
        let mut samples: Vec<i32> = Vec::new();
        let mut poly = vec![0.0f32; nvp * 3]; // polygon boundary verts in world-relative coords
        let mut hp = HeightPatch::new();

        // Calculate bounds for each polygon (for height patch)
        let mut bounds = vec![0i32; poly_mesh.npolys * 4];
        for i in 0..poly_mesh.npolys {
            let p = &poly_mesh.polys[i * nvp * 2..];
            let mut xmin = chf.width;
            let mut xmax = 0;
            let mut zmin = chf.height;
            let mut zmax = 0;

            for j in 0..nvp {
                if p[j] == MESH_NULL_IDX {
                    break;
                }
                let v = p[j] as usize * 3;
                xmin = xmin.min(poly_mesh.verts[v] as i32);
                xmax = xmax.max(poly_mesh.verts[v] as i32);
                zmin = zmin.min(poly_mesh.verts[v + 2] as i32);
                zmax = zmax.max(poly_mesh.verts[v + 2] as i32);
            }

            xmin = (xmin - 1).max(0);
            xmax = (xmax + 1).min(chf.width - 1);
            zmin = (zmin - 1).max(0);
            zmax = (zmax + 1).min(chf.height - 1);

            bounds[i * 4] = xmin;
            bounds[i * 4 + 1] = xmax;
            bounds[i * 4 + 2] = zmin;
            bounds[i * 4 + 3] = zmax;
        }

        // Process each polygon
        for i in 0..poly_mesh.npolys {
            let p = &poly_mesh.polys[i * nvp * 2..];

            // Build polygon boundary vertices in cell-relative coordinates
            let mut npoly = 0;
            for j in 0..nvp {
                if p[j] == MESH_NULL_IDX {
                    break;
                }
                let v = p[j] as usize * 3;
                poly[j * 3] = poly_mesh.verts[v] as f32 * cs;
                poly[j * 3 + 1] = poly_mesh.verts[v + 1] as f32 * ch;
                poly[j * 3 + 2] = poly_mesh.verts[v + 2] as f32 * cs;
                npoly += 1;
            }

            // Get height data for this polygon
            hp.xmin = bounds[i * 4];
            hp.ymin = bounds[i * 4 + 2];
            hp.width = bounds[i * 4 + 1] - bounds[i * 4];
            hp.height = bounds[i * 4 + 3] - bounds[i * 4 + 2];
            Self::get_height_data(
                chf,
                &poly_mesh.polys[i * nvp * 2..],
                nvp,
                &poly_mesh.verts,
                &mut hp,
                poly_mesh.regs[i],
                border_size,
            )?;

            // Build detail for this polygon
            let mut nverts = 0;
            let ok = build_poly_detail(
                &poly[..npoly * 3],
                npoly,
                sample_dist,
                sample_max_error,
                height_search_radius,
                chf,
                &hp,
                &mut verts,
                &mut nverts,
                &mut tris,
                &mut edges,
                &mut samples,
            );

            if !ok {
                return Err(BuildError::DetailMeshFailed);
            }

            // Move detail verts to world space
            for j in 0..nverts {
                verts[j * 3] += orig.x;
                verts[j * 3 + 1] += orig.y + chf.ch;
                verts[j * 3 + 2] += orig.z;
            }
            // Offset poly too (for flag checking)
            for j in 0..npoly {
                poly[j * 3] += orig.x;
                poly[j * 3 + 1] += orig.y;
                poly[j * 3 + 2] += orig.z;
            }

            // Store detail submesh
            let ntris = tris.len() / 4;

            dmesh.poly_start[i] = dmesh.tri_count;
            dmesh.poly_tri_count[i] = ntris;

            // Store vertices
            for j in 0..nverts {
                dmesh.vertices.push(verts[j * 3]);
                dmesh.vertices.push(verts[j * 3 + 1]);
                dmesh.vertices.push(verts[j * 3 + 2]);
            }
            dmesh.vert_count += nverts;

            // Store triangles (indices are local to this polygon's vertex range)
            for j in 0..ntris {
                dmesh.triangles.push(tris[j * 4] as u32);
                dmesh.triangles.push(tris[j * 4 + 1] as u32);
                dmesh.triangles.push(tris[j * 4 + 2] as u32);
                dmesh.triangles.push(tris[j * 4 + 3] as u32);
            }
            dmesh.tri_count += ntris;
        }

        // Final poly_start sentinel
        dmesh.poly_start[poly_mesh.npolys] = dmesh.tri_count;

        Ok(dmesh)
    }

    /// Get height data for a polygon (matches C++ getHeightData).
    ///
    /// Two paths:
    /// 1. If region != RC_MULTIPLE_REGS: scan height patch for matching region spans,
    ///    identify border seeds, BFS with span connections.
    /// 2. If empty (no matching region): seed from polygon center via DFS,
    ///    then BFS with span connections.
    fn get_height_data(
        chf: &CompactHeightfield,
        poly: &[u16],
        nvp: usize,
        verts: &[u16],
        hp: &mut HeightPatch,
        region: u16,
        border_size: i32,
    ) -> Result<(), BuildError> {
        const RC_NOT_CONNECTED: u8 = 63;

        // Allocate height data
        if hp.width <= 0 || hp.height <= 0 {
            hp.data.clear();
            return Ok(());
        }
        hp.data = vec![RC_UNSET_HEIGHT; (hp.width * hp.height) as usize];

        // Count polygon vertices
        let npoly = poly
            .iter()
            .take(nvp)
            .position(|&x| x == MESH_NULL_IDX)
            .unwrap_or(nvp);

        if npoly == 0 {
            return Ok(());
        }

        let bs = border_size;

        // Queue stores (cx, cz, span_index) triples in chf coordinates
        let mut queue: Vec<i32> = Vec::new();
        let mut empty = true;

        if region != RC_MULTIPLE_REGS {
            // Path 1: Scan entire height patch for spans with matching region.
            // Store their heights directly, and identify border spans as BFS seeds.
            for hy in 0..hp.height {
                let y = hp.ymin + hy + bs;
                for hx in 0..hp.width {
                    let x = hp.xmin + hx + bs;
                    let cell_idx = (x + y * chf.width) as usize;
                    if cell_idx >= chf.cells.len() {
                        continue;
                    }
                    let cell = &chf.cells[cell_idx];
                    if let Some(cell_index) = cell.index {
                        for si in 0..cell.count {
                            let i = cell_index + si;
                            let s = &chf.spans[i];
                            if s.reg == region {
                                // Store height
                                hp.data[(hx + hy * hp.width) as usize] = s.max as u16;
                                empty = false;

                                // Check if this span borders a different region
                                let mut border = false;
                                for dir in 0..4usize {
                                    if s.con[dir] != RC_NOT_CONNECTED {
                                        let ax = x + get_dir_offset_x(dir as u8);
                                        let ay = y + get_dir_offset_z(dir as u8);
                                        let ai = chf.cells[(ax + ay * chf.width) as usize]
                                            .index
                                            .unwrap()
                                            + s.con[dir] as usize;
                                        if chf.spans[ai].reg != region {
                                            border = true;
                                            break;
                                        }
                                    }
                                }
                                if border {
                                    queue.push(x);
                                    queue.push(y);
                                    queue.push(i as i32);
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }

        // If no matching region found (rare, or RC_MULTIPLE_REGS),
        // seed from polygon center
        if empty {
            Self::seed_from_poly_center(chf, poly, npoly, verts, bs, hp, &mut queue);
        }

        // BFS from seeds using span connections (matches C++ FIFO queue)
        let mut head: usize = 0;
        const RETRACT_SIZE: usize = 256;

        while head * 3 < queue.len() {
            let cx = queue[head * 3];
            let cy = queue[head * 3 + 1];
            let ci = queue[head * 3 + 2];
            head += 1;

            // Queue compaction (matches C++ retract optimization)
            if head >= RETRACT_SIZE {
                let remaining = queue.len() - RETRACT_SIZE * 3;
                if remaining > 0 {
                    queue.copy_within(RETRACT_SIZE * 3.., 0);
                }
                queue.truncate(remaining);
                head = 0;
            }

            let cs = &chf.spans[ci as usize];
            for dir in 0..4usize {
                if cs.con[dir] == RC_NOT_CONNECTED {
                    continue;
                }

                let ax = cx + get_dir_offset_x(dir as u8);
                let ay = cy + get_dir_offset_z(dir as u8);
                let hx = ax - hp.xmin - bs;
                let hy = ay - hp.ymin - bs;

                if (hx as u32) >= (hp.width as u32) || (hy as u32) >= (hp.height as u32) {
                    continue;
                }

                let hp_idx = (hx + hy * hp.width) as usize;
                if hp.data[hp_idx] != RC_UNSET_HEIGHT {
                    continue;
                }

                let ai =
                    chf.cells[(ax + ay * chf.width) as usize].index.unwrap() + cs.con[dir] as usize;
                hp.data[hp_idx] = chf.spans[ai].max as u16;

                queue.push(ax);
                queue.push(ay);
                queue.push(ai as i32);
            }
        }

        Ok(())
    }

    /// Seed height data from polygon center (matches C++ seedArrayWithPolyCenter).
    ///
    /// DFS from nearest vertex toward polygon center following span connections,
    /// then set that center span as the BFS seed.
    fn seed_from_poly_center(
        chf: &CompactHeightfield,
        poly: &[u16],
        npoly: usize,
        verts: &[u16],
        bs: i32,
        hp: &mut HeightPatch,
        queue: &mut Vec<i32>,
    ) {
        const RC_NOT_CONNECTED: u8 = 63;

        // 9 offsets: center + 8 neighbors
        const OFFSET: [(i32, i32); 9] = [
            (0, 0),
            (-1, -1),
            (0, -1),
            (1, -1),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
        ];

        // Find cell closest to a poly vertex
        let mut start_cell_x = 0i32;
        let mut start_cell_y = 0i32;
        let mut start_span_index: i32 = -1;
        let mut dmin = RC_UNSET_HEIGHT as i32;

        'outer: for j in 0..npoly {
            for &(ox, oy) in &OFFSET {
                if dmin <= 0 {
                    break 'outer;
                }
                let ax = verts[poly[j] as usize * 3] as i32 + ox;
                let ay = verts[poly[j] as usize * 3 + 1] as i32;
                let az = verts[poly[j] as usize * 3 + 2] as i32 + oy;
                if ax < hp.xmin
                    || ax >= hp.xmin + hp.width
                    || az < hp.ymin
                    || az >= hp.ymin + hp.height
                {
                    continue;
                }

                let cell_idx = ((ax + bs) + (az + bs) * chf.width) as usize;
                if cell_idx >= chf.cells.len() {
                    continue;
                }
                let cell = &chf.cells[cell_idx];
                if let Some(cell_index) = cell.index {
                    for si in 0..cell.count {
                        let i = cell_index + si;
                        let d = (ay - chf.spans[i].max as i32).abs();
                        if d < dmin {
                            start_cell_x = ax;
                            start_cell_y = az;
                            start_span_index = i as i32;
                            dmin = d;
                        }
                    }
                }
            }
        }

        if start_span_index < 0 {
            return;
        }

        // Find center of the polygon
        let mut pcx = 0i32;
        let mut pcy = 0i32;
        for j in 0..npoly {
            pcx += verts[poly[j] as usize * 3] as i32;
            pcy += verts[poly[j] as usize * 3 + 2] as i32;
        }
        pcx /= npoly as i32;
        pcy /= npoly as i32;

        // DFS to move toward the center (matches C++ seedArrayWithPolyCenter)
        let mut stack: Vec<i32> = Vec::new();
        stack.push(start_cell_x);
        stack.push(start_cell_y);
        stack.push(start_span_index);

        let mut dirs = [0u8, 1, 2, 3];
        // Use hp.data as visited marker (0 = visited, 0xff = unvisited)
        for val in hp.data.iter_mut() {
            *val = 0;
        }

        let mut cx = -1i32;
        let mut cy = -1i32;
        let mut ci = -1i32;

        loop {
            if stack.len() < 3 {
                break;
            }

            ci = stack.pop().unwrap();
            cy = stack.pop().unwrap();
            cx = stack.pop().unwrap();

            if cx == pcx && cy == pcy {
                break;
            }

            // Prefer direction toward center
            let direct_dir = if cx == pcx {
                get_dir_for_offset(0, if pcy > cy { 1 } else { -1 }).unwrap_or(0)
            } else {
                get_dir_for_offset(if pcx > cx { 1 } else { -1 }, 0).unwrap_or(0)
            };

            // Push the direct dir last so we start with it on next iteration
            let dd_idx = dirs.iter().position(|&d| d == direct_dir).unwrap_or(3);
            dirs.swap(dd_idx, 3);

            let cs_span = &chf.spans[ci as usize];
            for i in 0..4 {
                let dir = dirs[i] as usize;
                if cs_span.con[dir] == RC_NOT_CONNECTED {
                    continue;
                }

                let new_x = cx + get_dir_offset_x(dir as u8);
                let new_y = cy + get_dir_offset_z(dir as u8);

                let hpx = new_x - hp.xmin;
                let hpy = new_y - hp.ymin;
                if hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height {
                    continue;
                }

                if hp.data[(hpx + hpy * hp.width) as usize] != 0 {
                    continue;
                }

                hp.data[(hpx + hpy * hp.width) as usize] = 1;

                let cell_idx = ((new_x + bs) + (new_y + bs) * chf.width) as usize;
                let new_ci = chf.cells[cell_idx].index.unwrap() + cs_span.con[dir] as usize;
                stack.push(new_x);
                stack.push(new_y);
                stack.push(new_ci as i32);
            }

            // Restore dirs order
            dirs.swap(dd_idx, 3);
        }

        queue.clear();
        // Seeds are given in chf coordinates (with border offset)
        queue.push(cx + bs);
        queue.push(cy + bs);
        queue.push(ci);

        // Reset hp.data to unset and store the center span's height
        for val in hp.data.iter_mut() {
            *val = RC_UNSET_HEIGHT;
        }
        if cx >= hp.xmin && cy >= hp.ymin && cx < hp.xmin + hp.width && cy < hp.ymin + hp.height {
            hp.data[((cx - hp.xmin) + (cy - hp.ymin) * hp.width) as usize] =
                chf.spans[ci as usize].max as u16;
        }
    }

    /// Merges multiple detail meshes into a single detail mesh
    /// (matches C++ rcMergePolyMeshDetails)
    pub fn merge_detail_meshes(meshes: &[&PolyMeshDetail]) -> Result<Self, BuildError> {
        if meshes.is_empty() {
            return Err(BuildError::EmptyMeshList);
        }

        let mut max_verts = 0;
        let mut max_tris = 0;
        let mut max_meshes = 0;

        for mesh in meshes {
            max_verts += mesh.vert_count;
            max_tris += mesh.tri_count;
            max_meshes += mesh.poly_count;
        }

        let mut merged = Self::new();
        merged.vertices.reserve(max_verts * 3);
        merged.triangles.reserve(max_tris * 4);
        merged.poly_start.reserve(max_meshes + 1);
        merged.poly_tri_count.reserve(max_meshes);
        merged.poly_start.push(0);

        for mesh in meshes {
            let vert_base = merged.vert_count as u32;

            for poly_idx in 0..mesh.poly_count {
                let tri_start = mesh.poly_start[poly_idx];
                let tri_end = mesh.poly_start[poly_idx + 1];
                let num_tris = tri_end - tri_start;

                merged.poly_tri_count.push(mesh.poly_tri_count[poly_idx]);
                let last_start = merged.poly_start.last().copied().unwrap_or(0);
                merged.poly_start.push(last_start + num_tris);
                merged.poly_count += 1;

                for tri_idx in tri_start..tri_end {
                    let base_tri = tri_idx * 4;
                    // Copy vertex indices (offset by vert_base) + flags
                    for k in 0..3 {
                        merged
                            .triangles
                            .push(mesh.triangles[base_tri + k] + vert_base);
                    }
                    merged.triangles.push(mesh.triangles[base_tri + 3]); // flags
                }
                merged.tri_count += num_tris;
            }

            for k in 0..mesh.vert_count * 3 {
                merged.vertices.push(mesh.vertices[k]);
            }
            merged.vert_count += mesh.vert_count;
        }

        Ok(merged)
    }
}

/// Merges multiple detail meshes into a single detail mesh
pub fn merge_poly_mesh_details(meshes: &[&PolyMeshDetail]) -> Result<PolyMeshDetail, BuildError> {
    PolyMeshDetail::merge_detail_meshes(meshes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::heightfield::Heightfield;
    use crate::polymesh::PolyMesh;
    use glam::Vec3;

    #[test]
    fn test_triangulate_hull_simple() {
        // 4 vertices forming a square
        let verts = [
            0.0f32, 0.0, 0.0, // 0
            10.0, 0.0, 0.0, // 1
            10.0, 0.0, 10.0, // 2
            0.0, 0.0, 10.0, // 3
        ];
        let hull = [0i32, 1, 2, 3];
        let mut tris = Vec::new();

        triangulate_hull(&verts, 4, &hull, 4, &mut tris);

        // Should produce 2 triangles (4 ints each)
        assert_eq!(tris.len() / 4, 2);
    }

    #[test]
    fn test_create_detail_mesh() {
        let max_verts_per_poly = 6;
        let border_size = 0;
        let mut poly_mesh = PolyMesh::new(max_verts_per_poly, border_size);

        poly_mesh.bmin = Vec3::new(0.0, 0.0, 0.0);
        poly_mesh.bmax = Vec3::new(10.0, 10.0, 10.0);
        poly_mesh.cs = 1.0;
        poly_mesh.ch = 1.0;

        poly_mesh.verts = vec![0, 0, 0, 10, 0, 0, 10, 0, 10, 0, 0, 10];
        poly_mesh.nverts = 4;

        poly_mesh.npolys = 1;
        poly_mesh.maxpolys = 1;
        poly_mesh.polys = vec![MESH_NULL_IDX; poly_mesh.nvp * 2];
        poly_mesh.regs = vec![1];
        poly_mesh.areas = vec![1];
        poly_mesh.flags = vec![0];

        for (i, &v) in [0u16, 1, 2, 3].iter().enumerate() {
            if i < poly_mesh.nvp {
                poly_mesh.polys[i] = v;
            }
        }
        for i in 4..poly_mesh.nvp {
            poly_mesh.polys[i] = MESH_NULL_IDX;
        }
        for i in 0..poly_mesh.nvp {
            poly_mesh.polys[poly_mesh.nvp + i] = MESH_NULL_IDX;
        }

        let width = 10;
        let height = 10;
        let bmin = Vec3::new(0.0, 0.0, 0.0);
        let bmax = Vec3::new(10.0, 10.0, 10.0);
        let cs = 1.0;
        let ch = 1.0;

        let mut heightfield = Heightfield::new(width, height, bmin, bmax, cs, ch);
        for x in 0..width {
            for z in 0..height {
                heightfield.add_span(x, z, 0, 1, 1).unwrap();
            }
        }

        let chf = CompactHeightfield::build_from_heightfield(&heightfield, 2, 1).unwrap();

        let sample_dist = 0.0;
        let sample_max_error = 1.0;

        let detail_mesh =
            PolyMeshDetail::build_from_poly_mesh(&poly_mesh, &chf, sample_dist, sample_max_error)
                .unwrap();

        assert_eq!(detail_mesh.poly_count, 1);
        assert_eq!(detail_mesh.vert_count, 4);
        assert!(detail_mesh.tri_count > 0);
    }

    #[test]
    fn test_merge_detail_meshes() {
        let mut mesh1 = PolyMeshDetail::new();
        mesh1.vertices = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        mesh1.vert_count = 3;
        mesh1.triangles = vec![0, 1, 2, 0]; // 4 values per tri now
        mesh1.tri_count = 1;
        mesh1.poly_count = 1;
        mesh1.poly_start = vec![0, 1];
        mesh1.poly_tri_count = vec![1];

        let mut mesh2 = PolyMeshDetail::new();
        mesh2.vertices = vec![2.0, 0.0, 0.0, 3.0, 0.0, 0.0, 2.0, 0.0, 1.0];
        mesh2.vert_count = 3;
        mesh2.triangles = vec![0, 1, 2, 0]; // 4 values per tri now
        mesh2.tri_count = 1;
        mesh2.poly_count = 1;
        mesh2.poly_start = vec![0, 1];
        mesh2.poly_tri_count = vec![1];

        let merged = PolyMeshDetail::merge_detail_meshes(&[&mesh1, &mesh2]).unwrap();

        assert_eq!(merged.vert_count, 6);
        assert_eq!(merged.tri_count, 2);
        assert_eq!(merged.poly_count, 2);
        assert_eq!(merged.vertices.len(), 18);
        assert_eq!(merged.triangles.len(), 8); // 2 tris * 4 values each

        // First triangle uses first set of vertices
        assert_eq!(merged.triangles[0], 0);
        assert_eq!(merged.triangles[1], 1);
        assert_eq!(merged.triangles[2], 2);
        // Second triangle uses second set (offset by 3)
        assert_eq!(merged.triangles[4], 3);
        assert_eq!(merged.triangles[5], 4);
        assert_eq!(merged.triangles[6], 5);
    }
}
