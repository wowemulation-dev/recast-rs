use glam::Vec3;
use recast::Heightfield;

/// Result of a voxel raycast operation
#[derive(Debug, Clone)]
pub struct VoxelRaycastHit {
    /// The hit parameter (t) along the ray where intersection occurred
    pub t: f32,
    /// The world position of the hit
    pub position: Vec3,
    /// The cell coordinates where the hit occurred
    pub cell_x: i32,
    /// The cell coordinates where the hit occurred  
    pub cell_z: i32,
}

/// Voxel raycast system using the Fast Voxel Traversal Algorithm
///
/// Based on "A Fast Voxel Traversal Algorithm for Ray Tracing"
/// by John Amanatides and Andrew Woo
pub struct VoxelQuery {
    /// Origin point of the voxel grid
    origin: Vec3,
    /// Width of each tile
    tile_width: f32,
    /// Depth of each tile
    tile_depth: f32,
    /// Provider function for heightfield data
    heightfield_provider: Box<dyn Fn(i32, i32) -> Option<Heightfield>>,
}

impl VoxelQuery {
    /// Creates a new voxel query system
    pub fn new(
        origin: Vec3,
        tile_width: f32,
        tile_depth: f32,
        heightfield_provider: Box<dyn Fn(i32, i32) -> Option<Heightfield>>,
    ) -> Self {
        VoxelQuery {
            origin,
            tile_width,
            tile_depth,
            heightfield_provider,
        }
    }

    /// Creates a voxel query from a heightfield reference
    pub fn from_single_heightfield(origin: Vec3, tile_width: f32, tile_depth: f32) -> Self {
        // Simple provider that returns None for all tiles (will be updated when heightfield is set)
        let provider = Box::new(|_x: i32, _z: i32| -> Option<Heightfield> { None });

        VoxelQuery::new(origin, tile_width, tile_depth, provider)
    }

    /// Perform raycast using voxel heightfields
    ///
    /// Returns Some(hit) if an intersection is found, None otherwise
    pub fn raycast(&self, start: Vec3, end: Vec3) -> Option<VoxelRaycastHit> {
        self.traverse_tiles(start, end)
    }

    /// Traverse tiles using the voxel traversal algorithm
    fn traverse_tiles(&self, start: Vec3, end: Vec3) -> Option<VoxelRaycastHit> {
        let rel_start_x = start.x - self.origin.x;
        let rel_start_z = start.z - self.origin.z;

        let sx = (rel_start_x / self.tile_width).floor() as i32;
        let sz = (rel_start_z / self.tile_depth).floor() as i32;
        let ex = ((end.x - self.origin.x) / self.tile_width).floor() as i32;
        let ez = ((end.z - self.origin.z) / self.tile_depth).floor() as i32;

        let dx = ex - sx;
        let dz = ez - sz;

        let step_x = if dx < 0 { -1 } else { 1 };
        let step_z = if dz < 0 { -1 } else { 1 };

        let x_rem = (self.tile_width + (rel_start_x % self.tile_width)) % self.tile_width;
        let z_rem = (self.tile_depth + (rel_start_z % self.tile_depth)) % self.tile_depth;

        let tx = end.x - start.x;
        let tz = end.z - start.z;

        let x_offset = if tx < 0.0 {
            x_rem
        } else {
            self.tile_width - x_rem
        }
        .abs();
        let z_offset = if tz < 0.0 {
            z_rem
        } else {
            self.tile_depth - z_rem
        }
        .abs();

        let tx_abs = tx.abs();
        let tz_abs = tz.abs();

        let mut t_max_x = if tx_abs > 0.0 {
            x_offset / tx_abs
        } else {
            f32::INFINITY
        };
        let mut t_max_z = if tz_abs > 0.0 {
            z_offset / tz_abs
        } else {
            f32::INFINITY
        };

        let t_delta_x = if tx_abs > 0.0 {
            self.tile_width / tx_abs
        } else {
            f32::INFINITY
        };
        let t_delta_z = if tz_abs > 0.0 {
            self.tile_depth / tz_abs
        } else {
            f32::INFINITY
        };

        let mut current_x = sx;
        let mut current_z = sz;
        let mut t = 0.0f32;

        loop {
            let next_t = (1.0f32).min(t_max_x.min(t_max_z));

            if let Some(hit) =
                self.traverse_heightfield(current_x, current_z, start, end, t, next_t)
            {
                return Some(hit);
            }

            // Check if we've reached the end tile
            if (if dx > 0 {
                current_x >= ex
            } else {
                current_x <= ex
            }) && (if dz > 0 {
                current_z >= ez
            } else {
                current_z <= ez
            }) {
                break;
            }

            // Move to the next tile
            if t_max_x < t_max_z {
                t = t_max_x;
                t_max_x += t_delta_x;
                current_x += step_x;
            } else {
                t = t_max_z;
                t_max_z += t_delta_z;
                current_z += step_z;
            }
        }

        None
    }

    /// Traverse a specific heightfield tile
    fn traverse_heightfield(
        &self,
        tile_x: i32,
        tile_z: i32,
        start: Vec3,
        end: Vec3,
        t_min: f32,
        t_max: f32,
    ) -> Option<VoxelRaycastHit> {
        let heightfield = (self.heightfield_provider)(tile_x, tile_z)?;

        let tx = end.x - start.x;
        let ty = end.y - start.y;
        let tz = end.z - start.z;

        let entry = Vec3::new(
            start.x + t_min * tx,
            start.y + t_min * ty,
            start.z + t_min * tz,
        );

        let exit = Vec3::new(
            start.x + t_max * tx,
            start.y + t_max * ty,
            start.z + t_max * tz,
        );

        let rel_start_x = entry.x - heightfield.bmin.x;
        let rel_start_z = entry.z - heightfield.bmin.z;

        let sx = (rel_start_x / heightfield.cs).floor() as i32;
        let sz = (rel_start_z / heightfield.cs).floor() as i32;
        let ex = ((exit.x - heightfield.bmin.x) / heightfield.cs).floor() as i32;
        let ez = ((exit.z - heightfield.bmin.z) / heightfield.cs).floor() as i32;

        let dx = ex - sx;
        let dz = ez - sz;

        let step_x = if dx < 0 { -1 } else { 1 };
        let step_z = if dz < 0 { -1 } else { 1 };

        let x_rem = (heightfield.cs + (rel_start_x % heightfield.cs)) % heightfield.cs;
        let z_rem = (heightfield.cs + (rel_start_z % heightfield.cs)) % heightfield.cs;

        let x_offset = if tx < 0.0 {
            x_rem
        } else {
            heightfield.cs - x_rem
        }
        .abs();
        let z_offset = if tz < 0.0 {
            z_rem
        } else {
            heightfield.cs - z_rem
        }
        .abs();

        let tx_abs = tx.abs();
        let tz_abs = tz.abs();

        let mut t_max_x = if tx_abs > 0.0 {
            x_offset / tx_abs
        } else {
            f32::INFINITY
        };
        let mut t_max_z = if tz_abs > 0.0 {
            z_offset / tz_abs
        } else {
            f32::INFINITY
        };

        let t_delta_x = if tx_abs > 0.0 {
            heightfield.cs / tx_abs
        } else {
            f32::INFINITY
        };
        let t_delta_z = if tz_abs > 0.0 {
            heightfield.cs / tz_abs
        } else {
            f32::INFINITY
        };

        let mut current_x = sx;
        let mut current_z = sz;
        let mut t = 0.0f32;

        loop {
            // Check if current cell is within heightfield bounds
            if current_x >= 0
                && current_x < heightfield.width
                && current_z >= 0
                && current_z < heightfield.height
            {
                let y1 = start.y + ty * (t_min + t) - heightfield.bmin.y;
                let y2 = start.y + ty * (t_min + t_max_x.min(t_max_z)) - heightfield.bmin.y;
                let y_min = (y1.min(y2) / heightfield.ch).floor();
                let y_max = (y1.max(y2) / heightfield.ch).ceil();

                // Check spans in this cell
                if let Some(Some(span_rc)) = heightfield.spans.get(&(current_x, current_z)) {
                    let mut current_span = Some(span_rc.clone());
                    while let Some(span) = current_span {
                        let span_ref = span.borrow();
                        let span_min = span_ref.min as f32;
                        let span_max = span_ref.max as f32;

                        // Check if ray intersects this span
                        if span_min <= y_max && span_max >= y_min {
                            let hit_t = (t_min + t).min(1.0);
                            let hit_pos = Vec3::new(
                                start.x + hit_t * tx,
                                start.y + hit_t * ty,
                                start.z + hit_t * tz,
                            );

                            return Some(VoxelRaycastHit {
                                t: hit_t,
                                position: hit_pos,
                                cell_x: current_x,
                                cell_z: current_z,
                            });
                        }

                        current_span = span_ref.next.clone();
                    }
                }
            }

            // Check if we've reached the end of this tile
            if (if dx > 0 {
                current_x >= ex
            } else {
                current_x <= ex
            }) && (if dz > 0 {
                current_z >= ez
            } else {
                current_z <= ez
            }) {
                break;
            }

            // Move to next cell
            if t_max_x < t_max_z {
                t = t_max_x;
                t_max_x += t_delta_x;
                current_x += step_x;
            } else {
                t = t_max_z;
                t_max_z += t_delta_z;
                current_z += step_z;
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_heightfield() -> Heightfield {
        let mut hf = Heightfield::new(
            4,
            4,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(4.0, 2.0, 4.0),
            1.0,
            0.1,
        );

        // Add some test spans
        let _ = hf.add_span(1, 1, 0, 10, 1); // Ground
        let _ = hf.add_span(2, 2, 0, 20, 1); // Higher ground

        hf
    }

    #[test]
    fn test_voxel_query_creation() {
        let query = VoxelQuery::from_single_heightfield(Vec3::new(0.0, 0.0, 0.0), 4.0, 4.0);

        assert_eq!(query.origin, Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(query.tile_width, 4.0);
        assert_eq!(query.tile_depth, 4.0);
    }

    #[test]
    fn test_raycast_hit() {
        let query = VoxelQuery::from_single_heightfield(Vec3::new(0.0, 0.0, 0.0), 4.0, 4.0);

        // Ray that should hit the heightfield
        let start = Vec3::new(1.5, 2.0, 1.5);
        let end = Vec3::new(1.5, -1.0, 1.5);

        // Since we have no heightfield data, this should miss
        let hit = query.raycast(start, end);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_miss() {
        let query = VoxelQuery::from_single_heightfield(Vec3::new(0.0, 0.0, 0.0), 4.0, 4.0);

        // Ray that should miss (no heightfield data)
        let start = Vec3::new(0.5, 2.0, 0.5);
        let end = Vec3::new(0.5, -1.0, 0.5);

        let hit = query.raycast(start, end);
        assert!(hit.is_none());
    }
}
