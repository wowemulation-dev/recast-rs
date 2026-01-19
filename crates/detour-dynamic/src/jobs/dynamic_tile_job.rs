use super::{ColliderAdditionJob, ColliderRemovalJob};
use crate::{colliders::Collider, dynamic_tile::DynamicTile};
use recast_common::Result;
use std::collections::HashSet;
use std::sync::Arc;

/// Interface for jobs that modify dynamic tiles
pub trait DynamicTileJob {
    /// Get the tiles affected by this job
    fn affected_tiles(&self) -> &HashSet<(i32, i32)>;

    /// Process this job on the given tile
    fn process(&self, tile: &mut DynamicTile) -> Result<()>;

    /// Get a description of this job for debugging
    fn description(&self) -> String;
}

/// Job processor for handling dynamic tile jobs
pub struct JobProcessor {
    /// Queue of pending jobs
    job_queue: Vec<Box<dyn DynamicTileJob>>,
}

impl JobProcessor {
    /// Create a new job processor
    pub fn new() -> Self {
        JobProcessor {
            job_queue: Vec::new(),
        }
    }

    /// Add a job to the queue
    pub fn enqueue_job(&mut self, job: Box<dyn DynamicTileJob>) {
        self.job_queue.push(job);
    }

    /// Process all queued jobs that affect the given tiles
    pub fn process_jobs_for_tiles(
        &mut self,
        tiles: &mut std::collections::HashMap<(i32, i32), DynamicTile>,
    ) -> Result<usize> {
        let mut processed_count = 0;

        // Process jobs in order
        for job in self.job_queue.drain(..) {
            for &tile_coords in job.affected_tiles() {
                if let Some(tile) = tiles.get_mut(&tile_coords) {
                    job.process(tile)?;
                    processed_count += 1;
                }
            }
        }

        Ok(processed_count)
    }

    /// Get the number of queued jobs
    pub fn job_count(&self) -> usize {
        self.job_queue.len()
    }

    /// Clear all queued jobs
    pub fn clear_jobs(&mut self) {
        self.job_queue.clear();
    }

    /// Get descriptions of all queued jobs for debugging
    pub fn job_descriptions(&self) -> Vec<String> {
        self.job_queue.iter().map(|job| job.description()).collect()
    }

    /// Add a collider addition job to the queue
    pub fn add_collider(
        &mut self,
        collider_id: u64,
        collider: Arc<dyn Collider>,
        affected_tiles: Vec<(i32, i32)>,
    ) {
        let job =
            ColliderAdditionJob::new(collider_id, collider, affected_tiles.into_iter().collect());
        self.enqueue_job(Box::new(job));
    }

    /// Add a collider removal job to the queue
    pub fn remove_collider(&mut self, collider_id: u64, affected_tiles: Vec<(i32, i32)>) {
        let job = ColliderRemovalJob::new(collider_id, affected_tiles.into_iter().collect());
        self.enqueue_job(Box::new(job));
    }

    /// Process all queued jobs on the provided tiles
    pub fn process_all_jobs_on_tiles(
        &mut self,
        tiles: &mut std::collections::HashMap<(i32, i32), DynamicTile>,
    ) -> Result<usize> {
        let mut processed_count = 0;

        // Process jobs in order
        for job in self.job_queue.drain(..) {
            for &tile_coords in job.affected_tiles() {
                if let Some(tile) = tiles.get_mut(&tile_coords) {
                    job.process(tile)?;
                    processed_count += 1;
                }
            }
        }

        Ok(processed_count)
    }

    /// Check if there are pending jobs
    pub fn pending_job_count(&self) -> usize {
        self.job_queue.len()
    }

    /// Check if the job processor is idle (no pending jobs)
    pub fn is_idle(&self) -> bool {
        self.job_queue.is_empty()
    }

    /// Add a tile rebuild job
    pub fn rebuild_tile(&mut self, coords: (i32, i32), force_full_rebuild: bool) {
        // For now, we'll implement this as a simple marker
        // In a full implementation, this would be a specific rebuild job type
        log::info!(
            "Tile rebuild requested for {:?}, full_rebuild: {}",
            coords,
            force_full_rebuild
        );
    }
}

impl Default for JobProcessor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DynamicNavMeshConfig, DynamicTile};
    use glam::Vec3;

    struct TestJob {
        affected_tiles: HashSet<(i32, i32)>,
        description: String,
    }

    impl DynamicTileJob for TestJob {
        fn affected_tiles(&self) -> &HashSet<(i32, i32)> {
            &self.affected_tiles
        }

        fn process(&self, _tile: &mut DynamicTile) -> Result<()> {
            // Test job does nothing
            Ok(())
        }

        fn description(&self) -> String {
            self.description.clone()
        }
    }

    fn create_test_config() -> DynamicNavMeshConfig {
        DynamicNavMeshConfig::new(true, 32, 32, 0.3)
            .with_world_bounds(Vec3::new(-10.0, -5.0, -10.0), Vec3::new(10.0, 5.0, 10.0))
    }

    #[test]
    fn test_job_processor_creation() {
        let processor = JobProcessor::new();
        assert_eq!(processor.job_count(), 0);
    }

    #[test]
    fn test_job_enqueue() {
        let mut processor = JobProcessor::new();
        let job = TestJob {
            affected_tiles: HashSet::from([(0, 0)]),
            description: "Test job".to_string(),
        };

        processor.enqueue_job(Box::new(job));
        assert_eq!(processor.job_count(), 1);
    }

    #[test]
    fn test_job_processing() {
        let mut processor = JobProcessor::new();
        let config = create_test_config();

        // Create a test tile
        let mut tiles = std::collections::HashMap::new();
        tiles.insert(
            (0, 0),
            DynamicTile::new(
                0,
                0,
                Vec3::new(-1.0, -1.0, -1.0),
                Vec3::new(1.0, 1.0, 1.0),
                config.clone(),
            ),
        );

        // Add a test job
        let job = TestJob {
            affected_tiles: HashSet::from([(0, 0)]),
            description: "Test job".to_string(),
        };
        processor.enqueue_job(Box::new(job));

        // Process jobs
        let processed = processor.process_jobs_for_tiles(&mut tiles).unwrap();
        assert_eq!(processed, 1);
        assert_eq!(processor.job_count(), 0);
    }

    #[test]
    fn test_job_descriptions() {
        let mut processor = JobProcessor::new();

        let job1 = TestJob {
            affected_tiles: HashSet::from([(0, 0)]),
            description: "Job 1".to_string(),
        };
        let job2 = TestJob {
            affected_tiles: HashSet::from([(1, 1)]),
            description: "Job 2".to_string(),
        };

        processor.enqueue_job(Box::new(job1));
        processor.enqueue_job(Box::new(job2));

        let descriptions = processor.job_descriptions();
        assert_eq!(descriptions.len(), 2);
        assert_eq!(descriptions[0], "Job 1");
        assert_eq!(descriptions[1], "Job 2");
    }
}
