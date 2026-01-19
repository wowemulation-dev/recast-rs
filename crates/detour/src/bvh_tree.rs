//! Bounding Volume Hierarchy (BVH) tree for efficient spatial queries
//!
//! This module implements a BVH tree to accelerate polygon queries in the navigation mesh.
//! It provides O(log n) query performance instead of O(n) brute force searches.

use super::PolyRef;

/// Axis-aligned bounding box
#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    /// Minimum bounds
    pub min: [f32; 3],
    /// Maximum bounds  
    pub max: [f32; 3],
}

impl Aabb {
    /// Creates a new Aabb from min and max points
    pub fn new(min: [f32; 3], max: [f32; 3]) -> Self {
        Self { min, max }
    }

    /// Creates an empty Aabb (invalid bounds)
    pub fn empty() -> Self {
        Self {
            min: [f32::MAX; 3],
            max: [f32::MIN; 3],
        }
    }

    /// Checks if this Aabb is valid
    pub fn is_valid(&self) -> bool {
        self.min[0] <= self.max[0] && self.min[1] <= self.max[1] && self.min[2] <= self.max[2]
    }

    /// Expands this Aabb to include another Aabb
    pub fn expand(&mut self, other: &Aabb) {
        self.min[0] = self.min[0].min(other.min[0]);
        self.min[1] = self.min[1].min(other.min[1]);
        self.min[2] = self.min[2].min(other.min[2]);

        self.max[0] = self.max[0].max(other.max[0]);
        self.max[1] = self.max[1].max(other.max[1]);
        self.max[2] = self.max[2].max(other.max[2]);
    }

    /// Expands this Aabb to include a point
    pub fn expand_point(&mut self, point: &[f32; 3]) {
        self.min[0] = self.min[0].min(point[0]);
        self.min[1] = self.min[1].min(point[1]);
        self.min[2] = self.min[2].min(point[2]);

        self.max[0] = self.max[0].max(point[0]);
        self.max[1] = self.max[1].max(point[1]);
        self.max[2] = self.max[2].max(point[2]);
    }

    /// Checks if this Aabb overlaps with another Aabb
    pub fn overlaps(&self, other: &Aabb) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
            && self.min[2] <= other.max[2]
            && self.max[2] >= other.min[2]
    }

    /// Gets the center of the Aabb
    pub fn center(&self) -> [f32; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    /// Gets the surface area of the Aabb
    pub fn surface_area(&self) -> f32 {
        let dx = self.max[0] - self.min[0];
        let dy = self.max[1] - self.min[1];
        let dz = self.max[2] - self.min[2];
        2.0 * (dx * dy + dy * dz + dz * dx)
    }
}

/// Item stored in the BVH tree
#[derive(Debug, Clone)]
pub struct BVHItem {
    /// Polygon reference
    pub poly_ref: PolyRef,
    /// Bounding box of the polygon
    pub bounds: Aabb,
}

/// Node in the BVH tree
#[derive(Debug, Clone)]
pub enum BVHNode {
    /// Leaf node containing items
    Leaf { bounds: Aabb, items: Vec<BVHItem> },
    /// Internal node with two children
    Internal {
        bounds: Aabb,
        left: Box<BVHNode>,
        right: Box<BVHNode>,
    },
}

impl BVHNode {
    /// Gets the bounds of this node
    pub fn bounds(&self) -> &Aabb {
        match self {
            BVHNode::Leaf { bounds, .. } => bounds,
            BVHNode::Internal { bounds, .. } => bounds,
        }
    }

    /// Queries all items that overlap with the given bounds
    pub fn query(&self, query_bounds: &Aabb, results: &mut Vec<PolyRef>) {
        if !self.bounds().overlaps(query_bounds) {
            return;
        }

        match self {
            BVHNode::Leaf { items, .. } => {
                for item in items {
                    if item.bounds.overlaps(query_bounds) {
                        results.push(item.poly_ref);
                    }
                }
            }
            BVHNode::Internal { left, right, .. } => {
                left.query(query_bounds, results);
                right.query(query_bounds, results);
            }
        }
    }
}

/// Bounding Volume Hierarchy tree for spatial queries
#[derive(Debug)]
pub struct BVHTree {
    /// Root node of the tree
    root: Option<BVHNode>,
    /// Maximum items per leaf node
    max_leaf_size: usize,
}

impl Default for BVHTree {
    fn default() -> Self {
        Self::new()
    }
}

impl BVHTree {
    /// Creates a new empty BVH tree
    pub fn new() -> Self {
        Self {
            root: None,
            max_leaf_size: 4,
        }
    }

    /// Builds the BVH tree from a list of items
    pub fn build(&mut self, items: Vec<BVHItem>) {
        if items.is_empty() {
            self.root = None;
            return;
        }

        self.root = Some(self.build_node(items));
    }

    /// Recursively builds a BVH node
    fn build_node(&self, mut items: Vec<BVHItem>) -> BVHNode {
        // Calculate bounds for all items
        let mut bounds = Aabb::empty();
        for item in &items {
            bounds.expand(&item.bounds);
        }

        // If we have few enough items, create a leaf
        if items.len() <= self.max_leaf_size {
            return BVHNode::Leaf { bounds, items };
        }

        // Find the best axis to split on
        let center = bounds.center();
        let mut best_axis = 0;
        let mut best_split = 0.0;
        let mut best_cost = f32::MAX;

        for (axis, &center_val) in center.iter().enumerate() {
            // Try different split positions
            let mut splits = vec![center_val];

            // Also try splitting at item centers
            for item in &items {
                splits.push(item.bounds.center()[axis]);
            }

            for &split in &splits {
                let (left_items, right_items) = partition_items(&items, axis, split);

                if left_items.is_empty() || right_items.is_empty() {
                    continue;
                }

                // Calculate SAH cost
                let cost = self.calculate_sah_cost(&left_items, &right_items, &bounds);

                if cost < best_cost {
                    best_cost = cost;
                    best_axis = axis;
                    best_split = split;
                }
            }
        }

        // Partition items using the best split
        let (left_items, right_items) = partition_items(&items, best_axis, best_split);

        // If partition failed, fall back to median split
        let (left_items, right_items) = if left_items.is_empty() || right_items.is_empty() {
            items.sort_by(|a, b| {
                a.bounds.center()[best_axis]
                    .partial_cmp(&b.bounds.center()[best_axis])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let split_index = items.len() / 2;
            let right = items.split_off(split_index);
            (items, right)
        } else {
            (left_items, right_items)
        };

        // Recursively build children
        let left = Box::new(self.build_node(left_items));
        let right = Box::new(self.build_node(right_items));

        BVHNode::Internal {
            bounds,
            left,
            right,
        }
    }

    /// Calculates the Surface Area Heuristic (SAH) cost for a split
    fn calculate_sah_cost(
        &self,
        left_items: &[BVHItem],
        right_items: &[BVHItem],
        parent_bounds: &Aabb,
    ) -> f32 {
        let mut left_bounds = Aabb::empty();
        for item in left_items {
            left_bounds.expand(&item.bounds);
        }

        let mut right_bounds = Aabb::empty();
        for item in right_items {
            right_bounds.expand(&item.bounds);
        }

        let parent_area = parent_bounds.surface_area();
        let left_area = left_bounds.surface_area();
        let right_area = right_bounds.surface_area();

        // SAH cost formula
        let traversal_cost = 1.0;
        let intersection_cost = 1.0;

        traversal_cost
            + intersection_cost
                * ((left_items.len() as f32 * left_area / parent_area)
                    + (right_items.len() as f32 * right_area / parent_area))
    }

    /// Queries all items that overlap with the given bounds
    pub fn query(&self, query_bounds: &Aabb) -> Vec<PolyRef> {
        let mut results = Vec::new();

        if let Some(root) = &self.root {
            root.query(query_bounds, &mut results);
        }

        results
    }

    /// Clears the tree
    #[allow(dead_code)]
    pub fn clear(&mut self) {
        self.root = None;
    }
}

/// Partitions items along an axis at a split position
fn partition_items(items: &[BVHItem], axis: usize, split: f32) -> (Vec<BVHItem>, Vec<BVHItem>) {
    let mut left = Vec::new();
    let mut right = Vec::new();

    for item in items {
        if item.bounds.center()[axis] < split {
            left.push(item.clone());
        } else {
            right.push(item.clone());
        }
    }

    (left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_overlap() {
        let aabb1 = Aabb::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let aabb2 = Aabb::new([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
        let aabb3 = Aabb::new([2.0, 2.0, 2.0], [3.0, 3.0, 3.0]);

        assert!(aabb1.overlaps(&aabb2));
        assert!(aabb2.overlaps(&aabb1));
        assert!(!aabb1.overlaps(&aabb3));
        assert!(!aabb3.overlaps(&aabb1));
    }

    #[test]
    fn test_bvh_query() {
        let mut tree = BVHTree::new();

        // Create some test items
        let items = vec![
            BVHItem {
                poly_ref: PolyRef::new(1),
                bounds: Aabb::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]),
            },
            BVHItem {
                poly_ref: PolyRef::new(2),
                bounds: Aabb::new([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]),
            },
            BVHItem {
                poly_ref: PolyRef::new(3),
                bounds: Aabb::new([0.0, 1.0, 0.0], [1.0, 2.0, 1.0]),
            },
            BVHItem {
                poly_ref: PolyRef::new(4),
                bounds: Aabb::new([5.0, 5.0, 5.0], [6.0, 6.0, 6.0]),
            },
        ];

        tree.build(items);

        // Query overlapping region
        let query_bounds = Aabb::new([0.5, 0.5, 0.0], [1.5, 1.5, 1.0]);
        let results = tree.query(&query_bounds);

        assert_eq!(results.len(), 3);
        assert!(results.contains(&PolyRef::new(1)));
        assert!(results.contains(&PolyRef::new(2)));
        assert!(results.contains(&PolyRef::new(3)));
        assert!(!results.contains(&PolyRef::new(4)));
    }
}
