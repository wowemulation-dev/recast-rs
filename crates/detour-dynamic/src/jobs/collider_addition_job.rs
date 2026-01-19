use super::DynamicTileJob;
use crate::colliders::Collider;
use crate::dynamic_tile::DynamicTile;
use recast_common::Result;
use std::collections::HashSet;
use std::sync::Arc;

/// Job for adding a collider to dynamic tiles
pub struct ColliderAdditionJob {
    /// ID of the collider being added
    collider_id: u64,
    /// The collider to add
    collider: Arc<dyn Collider>,
    /// Tiles affected by this collider
    affected_tiles: HashSet<(i32, i32)>,
}

impl ColliderAdditionJob {
    /// Create a new collider addition job
    pub fn new(
        collider_id: u64,
        collider: Arc<dyn Collider>,
        affected_tiles: HashSet<(i32, i32)>,
    ) -> Self {
        ColliderAdditionJob {
            collider_id,
            collider,
            affected_tiles,
        }
    }
}

impl DynamicTileJob for ColliderAdditionJob {
    fn affected_tiles(&self) -> &HashSet<(i32, i32)> {
        &self.affected_tiles
    }

    fn process(&self, tile: &mut DynamicTile) -> Result<()> {
        // Add the collider to this tile
        tile.add_collider(self.collider_id, self.collider.clone())?;
        
        // Rasterize colliders into the heightfield if it exists
        if tile.heightfield.is_some() {
            tile.rasterize_colliders()?;
        }
        
        Ok(())
    }

    fn description(&self) -> String {
        format!("Add collider {} to {} tiles", self.collider_id, self.affected_tiles.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DynamicNavMeshConfig, DynamicTile, colliders::BoxCollider};
    use glam::Vec3;

    #[test]
    fn test_collider_addition_job_creation() {
        let collider = Arc::new(BoxCollider::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 1.0),
            0,
            1.0,
        ));
        
        let affected_tiles = HashSet::from([(0, 0), (1, 0)]);
        let job = ColliderAdditionJob::new(1, collider, affected_tiles.clone());
        
        assert_eq!(job.collider_id, 1);
        assert_eq!(job.affected_tiles(), &affected_tiles);
        assert_eq!(job.description(), "Add collider 1 to 2 tiles");
    }

    #[test]
    fn test_collider_addition_job_process() {
        let config = DynamicNavMeshConfig::default();
        let mut tile = DynamicTile::new(
            0, 0,
            Vec3::new(-5.0, -5.0, -5.0),
            Vec3::new(5.0, 5.0, 5.0),
            config,
        );

        let collider = Arc::new(BoxCollider::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 1.0),
            0,
            1.0,
        ));
        
        let affected_tiles = HashSet::from([(0, 0)]);
        let job = ColliderAdditionJob::new(1, collider, affected_tiles);
        
        // Process the job
        let result = job.process(&mut tile);
        assert!(result.is_ok());
        
        // Verify the collider was added
        assert!(tile.active_colliders.contains_key(&1));
        assert_eq!(tile.active_colliders.len(), 1);
    }
}