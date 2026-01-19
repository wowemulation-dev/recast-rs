use detour::{NavMeshQuery, PolyRef, QueryFilter};
use recast_common::Error;

const MAX_LOCAL_SEGS: usize = 8;
const MAX_LOCAL_POLYS: usize = 16;

#[derive(Debug, Clone, Copy)]
struct Segment {
    s: [f32; 6], // Segment start/end (start_x, start_y, start_z, end_x, end_y, end_z)
    d: f32,      // Distance for pruning
}

impl Default for Segment {
    fn default() -> Self {
        Self {
            s: [0.0; 6],
            d: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DtLocalBoundary {
    center: [f32; 3],
    segs: [Segment; MAX_LOCAL_SEGS],
    n_segs: usize,
    polys: [PolyRef; MAX_LOCAL_POLYS],
    n_polys: usize,
}

impl DtLocalBoundary {
    pub fn new() -> Self {
        Self {
            center: [0.0; 3],
            segs: [Segment::default(); MAX_LOCAL_SEGS],
            n_segs: 0,
            polys: [PolyRef::new(0); MAX_LOCAL_POLYS],
            n_polys: 0,
        }
    }

    pub fn reset(&mut self) {
        self.center = [0.0; 3];
        self.n_segs = 0;
        self.n_polys = 0;
    }

    pub fn update(
        &mut self,
        poly_ref: PolyRef,
        pos: &[f32; 3],
        collision_query_range: f32,
        navquery: &NavMeshQuery,
        filter: &QueryFilter,
    ) -> Result<(), Error> {
        if !poly_ref.is_valid() {
            return Ok(());
        }

        self.center = *pos;
        self.n_segs = 0;
        self.n_polys = 0;

        // First, find all nearby polygons
        let half_extents = [
            collision_query_range,
            collision_query_range,
            collision_query_range,
        ];
        let nearby_polys = navquery.query_polygons(pos, &half_extents, filter, MAX_LOCAL_POLYS)?;

        // Store the polygons for validation
        self.n_polys = nearby_polys.len().min(MAX_LOCAL_POLYS);
        for (i, &poly) in nearby_polys.iter().take(self.n_polys).enumerate() {
            self.polys[i] = poly;
        }

        // Extract boundary segments from nearby polygons
        let nav_mesh = navquery.nav_mesh();
        for &poly_ref in &nearby_polys {
            let (tile, poly) = nav_mesh.get_tile_and_poly_by_ref(poly_ref)?;

            // Get polygon vertices
            let mut verts = Vec::new();
            for &v_idx in &poly.verts[..poly.vert_count as usize] {
                let vertex_idx = v_idx as usize;
                if vertex_idx * 3 + 2 < tile.verts.len() {
                    verts.push([
                        tile.verts[vertex_idx * 3],
                        tile.verts[vertex_idx * 3 + 1],
                        tile.verts[vertex_idx * 3 + 2],
                    ]);
                }
            }

            // Process each edge
            for i in 0..verts.len() {
                let j = (i + 1) % verts.len();
                let va = verts[i];
                let vb = verts[j];

                // Check if this edge is a boundary (not shared with another polygon)
                if self.is_boundary_edge(tile, poly, i, &nearby_polys, navquery)? {
                    // Calculate distance from center to segment
                    let dist = self.distance_point_to_segment_2d(pos, &va, &vb);

                    if dist < collision_query_range {
                        self.add_segment(dist, &[va[0], va[1], va[2], vb[0], vb[1], vb[2]]);
                    }
                }
            }
        }

        Ok(())
    }

    pub fn is_valid(&self, navquery: &NavMeshQuery, filter: &QueryFilter) -> Result<bool, Error> {
        if self.n_polys == 0 {
            return Ok(false);
        }

        // Check if all stored polygons are still valid
        for i in 0..self.n_polys {
            let poly_ref = self.polys[i];
            if !poly_ref.is_valid() {
                return Ok(false);
            }

            // Check if polygon still passes the filter
            let nav_mesh = navquery.nav_mesh();
            if let Ok((tile, poly)) = nav_mesh.get_tile_and_poly_by_ref(poly_ref) {
                if !filter.pass_filter(poly_ref, tile, poly) {
                    return Ok(false);
                }
            } else {
                return Ok(false);
            }
        }

        Ok(true)
    }

    pub fn get_center(&self) -> &[f32; 3] {
        &self.center
    }

    pub fn get_segment_count(&self) -> usize {
        self.n_segs
    }

    pub fn get_segment(&self, i: usize) -> Option<&[f32; 6]> {
        if i >= self.n_segs {
            None
        } else {
            Some(&self.segs[i].s)
        }
    }

    fn add_segment(&mut self, dist: f32, s: &[f32; 6]) {
        // Insert segment in sorted order (closest first)
        let mut insert_idx = self.n_segs;

        // Find insertion point
        for i in 0..self.n_segs {
            if dist < self.segs[i].d {
                insert_idx = i;
                break;
            }
        }

        // Shift segments if needed
        if insert_idx < MAX_LOCAL_SEGS {
            // Shift existing segments
            let shift_count = (self.n_segs - insert_idx).min(MAX_LOCAL_SEGS - insert_idx - 1);
            for i in (insert_idx..insert_idx + shift_count).rev() {
                if i + 1 < MAX_LOCAL_SEGS {
                    self.segs[i + 1] = self.segs[i];
                }
            }

            // Insert new segment
            self.segs[insert_idx] = Segment { s: *s, d: dist };

            // Update count
            self.n_segs = (self.n_segs + 1).min(MAX_LOCAL_SEGS);
        }
    }

    fn is_boundary_edge(
        &self,
        tile: &detour::MeshTile,
        poly: &detour::Poly,
        edge_idx: usize,
        _nearby_polys: &[PolyRef],
        _navquery: &NavMeshQuery,
    ) -> Result<bool, Error> {
        // Get the neighbor link for this edge
        if edge_idx >= poly.vert_count as usize {
            return Ok(true); // Invalid edge index, treat as boundary
        }

        // Check if this edge has a neighbor polygon
        // In the navigation mesh, edges store neighbor information
        let neighbor = poly.neighbors[edge_idx];

        // If neighbor is 0 (MESH_NULL_IDX), this is a boundary edge
        if neighbor == 0 || neighbor == 0xFFFF {
            return Ok(true);
        }

        // If the edge has a link to another polygon, check if it's internal or external
        // Internal links (within the same tile) have the high bit clear
        // External links (to other tiles) have the high bit set
        if (neighbor & 0x8000) != 0 {
            // External link - need to check if the linked tile/polygon exists
            // For now, we'll consider external links as boundaries for local boundary purposes
            return Ok(true);
        }

        // Internal link - check if the linked polygon is valid
        let linked_poly_idx = (neighbor & 0x7FFF) as usize;
        if linked_poly_idx >= tile.polys.len() {
            return Ok(true); // Invalid link, treat as boundary
        }

        // The edge connects to another polygon, so it's not a boundary
        Ok(false)
    }

    fn distance_point_to_segment_2d(&self, pt: &[f32; 3], a: &[f32; 3], b: &[f32; 3]) -> f32 {
        let pqx = b[0] - a[0];
        let pqz = b[2] - a[2];
        let dx = pt[0] - a[0];
        let dz = pt[2] - a[2];
        let d = pqx * pqx + pqz * pqz;
        let mut t = pqx * dx + pqz * dz;

        if d > 0.0 {
            t /= d;
        }

        t = t.clamp(0.0, 1.0);

        let dx = a[0] + t * pqx - pt[0];
        let dz = a[2] + t * pqz - pt[2];
        (dx * dx + dz * dz).sqrt()
    }
}

impl Default for DtLocalBoundary {
    fn default() -> Self {
        Self::new()
    }
}
