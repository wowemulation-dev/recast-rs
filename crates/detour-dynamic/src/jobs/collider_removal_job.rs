use super::DynamicTileJob;
use crate::dynamic_tile::DynamicTile;
use recast_common::Result;
use std::collections::HashSet;

/// Job for removing a collider from dynamic tiles
pub struct ColliderRemovalJob {
    /// ID of the collider being removed
    collider_id: u64,
    /// Tiles affected by this collider removal
    affected_tiles: HashSet<(i32, i32)>,
}

impl ColliderRemovalJob {
    /// Create a new collider removal job
    pub fn new(collider_id: u64, affected_tiles: HashSet<(i32, i32)>) -> Self {
        ColliderRemovalJob {
            collider_id,
            affected_tiles,
        }
    }
}

impl DynamicTileJob for ColliderRemovalJob {
    fn affected_tiles(&self) -> &HashSet<(i32, i32)> {
        &self.affected_tiles
    }

    fn process(&self, tile: &mut DynamicTile) -> Result<()> {
        // Remove the collider from this tile
        let removed = tile.remove_collider(self.collider_id);
        
        if removed {
            // If we removed a collider and there's a heightfield, we might want to
            // rebuild it from scratch or restore from a checkpoint
            if tile.heightfield.is_some() {
                // For now, we'll clear the navigation mesh data to force a rebuild
                // In a more sophisticated implementation, we could restore from a checkpoint
                // that was taken before this collider was added
                tile.clear_navmesh_data();
                tile.mark_dirty();
            }
        }
        
        Ok(())
    }

    fn description(&self) -> String {
        format!("Remove collider {} from {} tiles", self.collider_id, self.affected_tiles.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DynamicNavMeshConfig, DynamicTile, colliders::BoxCollider};
    use glam::Vec3;
    use std::sync::Arc;

    #[test]
    fn test_collider_removal_job_creation() {
        let affected_tiles = HashSet::from([(0, 0), (1, 0)]);
        let job = ColliderRemovalJob::new(1, affected_tiles.clone());
        
        assert_eq!(job.collider_id, 1);
        assert_eq!(job.affected_tiles(), &affected_tiles);
        assert_eq!(job.description(), "Remove collider 1 from 2 tiles");
    }

    #[test]
    fn test_collider_removal_job_process() {
        let config = DynamicNavMeshConfig::default();
        let mut tile = DynamicTile::new(
            0, 0,
            Vec3::new(-5.0, -5.0, -5.0),
            Vec3::new(5.0, 5.0, 5.0),
            config,
        );

        // First add a collider
        let collider = Arc::new(BoxCollider::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 1.0),
            0,
            1.0,
        ));
        tile.add_collider(1, collider).unwrap();
        assert!(tile.active_colliders.contains_key(&1));
        
        // Now create and process a removal job
        let affected_tiles = HashSet::from([(0, 0)]);
        let job = ColliderRemovalJob::new(1, affected_tiles);
        
        // Process the job
        let result = job.process(&mut tile);
        assert!(result.is_ok());
        
        // Verify the collider was removed
        assert!(!tile.active_colliders.contains_key(&1));
        assert_eq!(tile.active_colliders.len(), 0);
    }

    #[test]
    fn test_collider_removal_job_nonexistent_collider() {
        let config = DynamicNavMeshConfig::default();
        let mut tile = DynamicTile::new(
            0, 0,
            Vec3::new(-5.0, -5.0, -5.0),
            Vec3::new(5.0, 5.0, 5.0),
            config,
        );

        // Try to remove a collider that doesn't exist
        let affected_tiles = HashSet::from([(0, 0)]);
        let job = ColliderRemovalJob::new(999, affected_tiles);
        
        // Process the job - should succeed even if collider doesn't exist
        let result = job.process(&mut tile);
        assert!(result.is_ok());
        
        // Verify no colliders exist
        assert_eq!(tile.active_colliders.len(), 0);
    }
}