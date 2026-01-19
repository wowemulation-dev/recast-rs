//! Polygon query interface for Detour
//!
//! This module provides the PolyQuery trait that matches the C++ dtPolyQuery interface

use super::{MeshTile, NavMeshQuery, Poly, PolyRef};

/// Provides custom polygon query behavior.
/// Used by NavMeshQuery::query_polygons.
///
/// This trait is equivalent to the C++ dtPolyQuery class.
pub trait PolyQuery {
    /// Called for each batch of unique polygons touched by the search area.
    /// This can be called multiple times for a single query.
    ///
    /// # Arguments
    /// * `tile` - The tile containing the polygons
    /// * `polys` - Array of polygons in the batch
    /// * `refs` - Array of polygon references corresponding to the polygons
    fn process(&mut self, tile: &MeshTile, polys: &[&Poly], refs: &[PolyRef]);
}

/// Collects polygons within a search area.
///
/// This is equivalent to the C++ dtCollectPolysQuery class.
pub struct CollectPolysQuery {
    polys: Vec<PolyRef>,
    max_polys: usize,
    overflow: bool,
}

impl CollectPolysQuery {
    /// Creates a new CollectPolysQuery with the specified maximum capacity.
    pub fn new(max_polys: usize) -> Self {
        Self {
            polys: Vec::with_capacity(max_polys.min(1024)), // Cap initial allocation
            max_polys,
            overflow: false,
        }
    }

    /// Returns the collected polygon references.
    pub fn polys(&self) -> &[PolyRef] {
        &self.polys
    }

    /// Returns the number of collected polygons.
    pub fn num_collected(&self) -> usize {
        self.polys.len()
    }

    /// Returns true if more polygons were available than could be collected.
    pub fn overflow(&self) -> bool {
        self.overflow
    }

    /// Clears the collected polygons.
    pub fn clear(&mut self) {
        self.polys.clear();
        self.overflow = false;
    }
}

impl PolyQuery for CollectPolysQuery {
    fn process(&mut self, _tile: &MeshTile, _polys: &[&Poly], refs: &[PolyRef]) {
        let remaining = self.max_polys.saturating_sub(self.polys.len());

        if remaining < refs.len() {
            self.overflow = true;
            self.polys.extend_from_slice(&refs[..remaining]);
        } else {
            self.polys.extend_from_slice(refs);
        }
    }
}

/// Finds the nearest polygon to a point.
///
/// This is equivalent to the C++ dtFindNearestPolyQuery class.
pub struct FindNearestPolyQuery<'a> {
    query: &'a NavMeshQuery<'a>,
    center: [f32; 3],
    nearest_distance_sqr: f32,
    nearest_ref: PolyRef,
    nearest_point: [f32; 3],
    over_poly: bool,
}

impl<'a> FindNearestPolyQuery<'a> {
    /// Creates a new FindNearestPolyQuery.
    pub fn new(query: &'a NavMeshQuery<'a>, center: &[f32; 3]) -> Self {
        Self {
            query,
            center: *center,
            nearest_distance_sqr: f32::MAX,
            nearest_ref: PolyRef::new(0),
            nearest_point: [0.0; 3],
            over_poly: false,
        }
    }

    /// Returns the nearest polygon reference found.
    pub fn nearest_ref(&self) -> PolyRef {
        self.nearest_ref
    }

    /// Returns the nearest point found.
    pub fn nearest_point(&self) -> &[f32; 3] {
        &self.nearest_point
    }

    /// Returns true if the nearest point is directly over the polygon.
    pub fn is_over_poly(&self) -> bool {
        self.over_poly
    }

    /// Returns the squared distance to the nearest polygon.
    pub fn nearest_distance_sqr(&self) -> f32 {
        self.nearest_distance_sqr
    }
}

impl<'a> PolyQuery for FindNearestPolyQuery<'a> {
    fn process(&mut self, _tile: &MeshTile, _polys: &[&Poly], refs: &[PolyRef]) {
        for &poly_ref in refs {
            // Get closest point on this polygon
            if let Ok((closest_pt, is_over_poly)) =
                self.query.closest_point_on_poly(poly_ref, &self.center)
            {
                // Calculate squared distance
                let diff = [
                    self.center[0] - closest_pt[0],
                    self.center[1] - closest_pt[1],
                    self.center[2] - closest_pt[2],
                ];

                let d = if is_over_poly {
                    // If point is directly over polygon and closer than climb height,
                    // favor that instead of straight line nearest point
                    let climb_height = 0.25; // TODO: Get from nav mesh params
                    let height_diff = diff[1].abs() - climb_height;
                    if height_diff > 0.0 {
                        height_diff * height_diff
                    } else {
                        0.0
                    }
                } else {
                    // Regular squared distance
                    diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]
                };

                if d < self.nearest_distance_sqr {
                    self.nearest_distance_sqr = d;
                    self.nearest_ref = poly_ref;
                    self.nearest_point = closest_pt;
                    self.over_poly = is_over_poly;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collect_polys_query() {
        let mut query = CollectPolysQuery::new(10);

        // Test basic collection
        let refs = vec![PolyRef::new(1), PolyRef::new(2), PolyRef::new(3)];
        query.process(&MeshTile::default(), &[], &refs);

        assert_eq!(query.num_collected(), 3);
        assert!(!query.overflow());
        assert_eq!(query.polys(), &refs);

        // Test overflow
        let more_refs: Vec<_> = (4..20).map(|i| PolyRef::new(i)).collect();
        query.process(&MeshTile::default(), &[], &more_refs);

        assert_eq!(query.num_collected(), 10);
        assert!(query.overflow());
    }

    #[test]
    fn test_collect_polys_clear() {
        let mut query = CollectPolysQuery::new(10);

        let refs = vec![PolyRef::new(1), PolyRef::new(2)];
        query.process(&MeshTile::default(), &[], &refs);
        assert_eq!(query.num_collected(), 2);

        query.clear();
        assert_eq!(query.num_collected(), 0);
        assert!(!query.overflow());
    }
}
