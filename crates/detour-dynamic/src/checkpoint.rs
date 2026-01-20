use recast::Heightfield;
use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

/// Checkpoint system for saving and restoring heightfield states
///
/// This system allows for efficient incremental updates by saving snapshots
/// of heightfield state before applying collider modifications. When colliders
/// are removed, the system can restore the heightfield to a previous clean state
/// rather than rebuilding from scratch.
#[derive(Debug)]
pub struct DynamicTileCheckpoint {
    /// Snapshot of the heightfield state
    pub heightfield: Heightfield,
    /// Set of collider IDs that were active when this checkpoint was created
    pub colliders: HashSet<u64>,
}

impl DynamicTileCheckpoint {
    /// Creates a new checkpoint from a heightfield and active colliders
    pub fn new(heightfield: &Heightfield, colliders: HashSet<u64>) -> Self {
        DynamicTileCheckpoint {
            heightfield: Self::clone_heightfield(heightfield),
            colliders,
        }
    }

    /// Deep clones a heightfield including all span data
    fn clone_heightfield(source: &Heightfield) -> Heightfield {
        let mut clone = Heightfield::new(
            source.width,
            source.height,
            source.bmin,
            source.bmax,
            source.cs,
            source.ch,
        );

        // Clone all spans
        for z in 0..source.height {
            for x in 0..source.width {
                if let Some(Some(span_rc)) = source.spans.get(&(x, z)) {
                    Self::clone_span_chain(&mut clone, x, z, span_rc.clone());
                }
            }
        }

        clone
    }

    /// Recursively clones a chain of spans
    fn clone_span_chain(target: &mut Heightfield, x: i32, z: i32, span: Rc<RefCell<recast::Span>>) {
        let span_borrow = span.borrow();
        let _ = target.add_span(x, z, span_borrow.min, span_borrow.max, span_borrow.area);

        // Clone next span in the chain if it exists
        if let Some(next_span) = &span_borrow.next {
            Self::clone_span_chain(target, x, z, next_span.clone());
        }
    }

    /// Checks if this checkpoint can be used for the given set of colliders
    ///
    /// A checkpoint can be used if the active colliders are a subset of
    /// the colliders that were active when the checkpoint was created.
    pub fn is_valid_for(&self, active_colliders: &HashSet<u64>) -> bool {
        active_colliders.is_subset(&self.colliders)
    }

    /// Gets the memory footprint of this checkpoint in bytes
    pub fn memory_usage(&self) -> usize {
        let mut size = std::mem::size_of::<Self>();

        // Add heightfield memory
        size += std::mem::size_of::<Heightfield>();

        // Estimate span memory (this is approximate)
        size += self.heightfield.spans.len() * std::mem::size_of::<(i32, i32)>();
        size += self.heightfield.spans.len() * 64; // Approximate span memory

        // Add collider set memory
        size += self.colliders.len() * std::mem::size_of::<u64>();

        size
    }
}

/// Manager for checkpoint operations
#[derive(Debug)]
pub struct CheckpointManager {
    /// Maximum number of checkpoints to keep
    max_checkpoints: usize,
    /// Checkpoints stored in chronological order (newest first)
    checkpoints: Vec<DynamicTileCheckpoint>,
}

impl CheckpointManager {
    /// Creates a new checkpoint manager
    pub fn new(max_checkpoints: usize) -> Self {
        CheckpointManager {
            max_checkpoints,
            checkpoints: Vec::new(),
        }
    }

    /// Creates a new checkpoint and adds it to the manager
    pub fn create_checkpoint(&mut self, heightfield: &Heightfield, colliders: HashSet<u64>) {
        let checkpoint = DynamicTileCheckpoint::new(heightfield, colliders);

        // Add to front (newest first)
        self.checkpoints.insert(0, checkpoint);

        // Remove oldest checkpoints if we exceed the limit
        if self.checkpoints.len() > self.max_checkpoints {
            self.checkpoints.truncate(self.max_checkpoints);
        }
    }

    /// Finds the best checkpoint for the given active colliders
    ///
    /// Returns the newest checkpoint that can be used for the given colliders,
    /// or None if no suitable checkpoint exists.
    pub fn find_best_checkpoint(
        &self,
        active_colliders: &HashSet<u64>,
    ) -> Option<&DynamicTileCheckpoint> {
        self.checkpoints
            .iter()
            .find(|checkpoint| checkpoint.is_valid_for(active_colliders))
    }

    /// Clears all checkpoints
    pub fn clear(&mut self) {
        self.checkpoints.clear();
    }

    /// Gets the number of stored checkpoints
    pub fn checkpoint_count(&self) -> usize {
        self.checkpoints.len()
    }

    /// Gets the total memory usage of all checkpoints in bytes
    pub fn total_memory_usage(&self) -> usize {
        self.checkpoints.iter().map(|c| c.memory_usage()).sum()
    }

    /// Removes checkpoints that are no longer useful
    ///
    /// This removes checkpoints where all colliders in the checkpoint
    /// are also present in newer checkpoints, making the old checkpoint redundant.
    pub fn cleanup_redundant_checkpoints(&mut self) {
        if self.checkpoints.len() <= 1 {
            return;
        }

        let mut to_remove = Vec::new();

        for (i, checkpoint) in self.checkpoints.iter().enumerate() {
            // Check if any newer checkpoint contains all colliders from this one
            let is_redundant = self.checkpoints[0..i]
                .iter()
                .any(|newer| checkpoint.colliders.is_subset(&newer.colliders));

            if is_redundant {
                to_remove.push(i);
            }
        }

        // Remove in reverse order to maintain indices
        for &index in to_remove.iter().rev() {
            self.checkpoints.remove(index);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn create_test_heightfield() -> Heightfield {
        let mut hf = Heightfield::new(
            2,
            2,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 1.0, 2.0),
            1.0,
            0.1,
        );
        let _ = hf.add_span(0, 0, 0, 10, 1);
        let _ = hf.add_span(1, 1, 0, 10, 1);
        hf
    }

    #[test]
    fn test_checkpoint_creation() {
        let hf = create_test_heightfield();
        let colliders = HashSet::from([1, 2, 3]);

        let checkpoint = DynamicTileCheckpoint::new(&hf, colliders.clone());

        assert_eq!(checkpoint.colliders, colliders);
        assert_eq!(checkpoint.heightfield.width, hf.width);
        assert_eq!(checkpoint.heightfield.height, hf.height);
    }

    #[test]
    fn test_checkpoint_validity() {
        let hf = create_test_heightfield();
        let checkpoint_colliders = HashSet::from([1, 2, 3]);
        let checkpoint = DynamicTileCheckpoint::new(&hf, checkpoint_colliders);

        // Valid: subset of checkpoint colliders
        assert!(checkpoint.is_valid_for(&HashSet::from([1, 2])));
        assert!(checkpoint.is_valid_for(&HashSet::from([3])));
        assert!(checkpoint.is_valid_for(&HashSet::new()));

        // Invalid: contains colliders not in checkpoint
        assert!(!checkpoint.is_valid_for(&HashSet::from([1, 2, 3, 4])));
        assert!(!checkpoint.is_valid_for(&HashSet::from([4])));
    }

    #[test]
    fn test_checkpoint_manager() {
        let mut manager = CheckpointManager::new(3);
        let hf = create_test_heightfield();

        // Add some checkpoints
        manager.create_checkpoint(&hf, HashSet::from([1]));
        manager.create_checkpoint(&hf, HashSet::from([1, 2]));
        manager.create_checkpoint(&hf, HashSet::from([1, 2, 3]));

        assert_eq!(manager.checkpoint_count(), 3);

        // Add one more - should remove the oldest
        manager.create_checkpoint(&hf, HashSet::from([1, 2, 3, 4]));
        assert_eq!(manager.checkpoint_count(), 3);

        // Find best checkpoint
        let best = manager.find_best_checkpoint(&HashSet::from([1, 2]));
        assert!(best.is_some());
        assert!(best.unwrap().colliders.contains(&1));
        assert!(best.unwrap().colliders.contains(&2));
    }
}
