//! Node pool and queue implementations for pathfinding
//!

use super::PolyRef;

/// Node flags for pathfinding state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeFlags(u8);

impl NodeFlags {
    pub const OPEN: NodeFlags = NodeFlags(0x01);
    pub const CLOSED: NodeFlags = NodeFlags(0x02);
    pub const PARENT_DETACHED: NodeFlags = NodeFlags(0x04);

    pub fn contains(&self, flag: NodeFlags) -> bool {
        self.0 & flag.0 != 0
    }

    pub fn insert(&mut self, flag: NodeFlags) {
        self.0 |= flag.0;
    }

    pub fn remove(&mut self, flag: NodeFlags) {
        self.0 &= !flag.0;
    }
}

/// Node index type
pub type NodeIndex = u16;

/// Null node index constant
pub const DT_NULL_IDX: NodeIndex = NodeIndex::MAX;

/// Maximum states per node
pub const DT_MAX_STATES_PER_NODE: usize = 4;

/// Node in the pathfinding graph
#[derive(Debug, Clone)]
pub struct DtNode {
    /// Position of the node
    pub pos: [f32; 3],
    /// Cost from previous node to current node
    pub cost: f32,
    /// Total cost up to the node
    pub total: f32,
    /// Index to parent node
    pub pidx: NodeIndex,
    /// Extra state information (0-3)
    pub state: u8,
    /// Node flags
    pub flags: NodeFlags,
    /// Polygon ref the node corresponds to
    pub id: PolyRef,
}

impl DtNode {
    /// Creates a new node
    pub fn new(id: PolyRef) -> Self {
        Self {
            pos: [0.0; 3],
            cost: 0.0,
            total: 0.0,
            pidx: DT_NULL_IDX,
            state: 0,
            flags: NodeFlags(0),
            id,
        }
    }
}

/// Node pool for efficient node management during pathfinding
pub struct DtNodePool {
    /// Node storage
    nodes: Vec<DtNode>,
    /// First node index for each hash bucket
    first: Vec<NodeIndex>,
    /// Next node index in hash chain
    next: Vec<NodeIndex>,
    /// Maximum number of nodes
    max_nodes: usize,
    /// Hash table size
    hash_size: usize,
    /// Current node count
    node_count: usize,
}

impl DtNodePool {
    /// Creates a new node pool
    pub fn new(max_nodes: usize, hash_size: usize) -> Self {
        let mut nodes = Vec::with_capacity(max_nodes);
        for _ in 0..max_nodes {
            nodes.push(DtNode::new(PolyRef::new(0)));
        }

        Self {
            nodes,
            first: vec![DT_NULL_IDX; hash_size],
            next: vec![DT_NULL_IDX; max_nodes],
            max_nodes,
            hash_size,
            node_count: 0,
        }
    }

    /// Clears the node pool
    pub fn clear(&mut self) {
        self.first.fill(DT_NULL_IDX);
        self.next.fill(DT_NULL_IDX);
        self.node_count = 0;
    }

    /// Gets or allocates a node for the given polygon ref and state
    pub fn get_node(&mut self, id: PolyRef, state: u8) -> Option<&mut DtNode> {
        // First check if node exists
        let hash = Self::hash_ref(id) & (self.hash_size - 1);
        let mut idx = self.first[hash];
        let mut found_idx = None;

        while idx != DT_NULL_IDX {
            let node_idx = idx as usize;
            if node_idx < self.nodes.len() {
                if self.nodes[node_idx].id == id && self.nodes[node_idx].state == state {
                    found_idx = Some(node_idx);
                    break;
                }
                idx = self.next[node_idx];
            } else {
                break;
            }
        }

        // If found, return it
        if let Some(idx) = found_idx {
            return Some(&mut self.nodes[idx]);
        }

        // Allocate new node
        if self.node_count >= self.max_nodes {
            return None;
        }

        let idx = self.node_count;
        self.node_count += 1;

        // Init node
        let node = &mut self.nodes[idx];
        node.pidx = DT_NULL_IDX;
        node.cost = 0.0;
        node.total = 0.0;
        node.id = id;
        node.state = state;
        node.flags = NodeFlags(0);

        // Add to hash table
        let hash = Self::hash_ref(id) & (self.hash_size - 1);
        self.next[idx] = self.first[hash];
        self.first[hash] = idx as NodeIndex;

        Some(&mut self.nodes[idx])
    }

    /// Finds a node with the given polygon ref and state
    pub fn find_node(&self, id: PolyRef, state: u8) -> Option<&DtNode> {
        let hash = Self::hash_ref(id) & (self.hash_size - 1);
        let mut idx = self.first[hash];

        while idx != DT_NULL_IDX {
            let node_idx = idx as usize;
            if node_idx < self.nodes.len() {
                let node = &self.nodes[node_idx];
                if node.id == id && node.state == state {
                    return Some(node);
                }
                idx = self.next[node_idx];
            } else {
                break;
            }
        }

        None
    }

    /// Finds a mutable node with the given polygon ref and state
    #[allow(dead_code)]
    fn find_node_mut(&mut self, id: PolyRef, state: u8) -> Option<&mut DtNode> {
        let hash = Self::hash_ref(id) & (self.hash_size - 1);
        let mut idx = self.first[hash];

        while idx != DT_NULL_IDX {
            let node_idx = idx as usize;
            if node_idx < self.nodes.len() {
                if self.nodes[node_idx].id == id && self.nodes[node_idx].state == state {
                    return Some(&mut self.nodes[node_idx]);
                }
                idx = self.next[node_idx];
            } else {
                break;
            }
        }

        None
    }

    /// Finds all nodes with the given polygon ref
    pub fn find_nodes(&self, id: PolyRef, max_nodes: usize) -> Vec<&DtNode> {
        let mut result = Vec::new();
        let hash = Self::hash_ref(id) & (self.hash_size - 1);
        let mut idx = self.first[hash];

        while idx != DT_NULL_IDX && result.len() < max_nodes {
            let node_idx = idx as usize;
            if node_idx < self.nodes.len() {
                let node = &self.nodes[node_idx];
                if node.id == id {
                    result.push(node);
                }
                idx = self.next[node_idx];
            } else {
                break;
            }
        }

        result
    }

    /// Gets the index of a node
    pub fn get_node_idx(&self, node: &DtNode) -> NodeIndex {
        let ptr = node as *const DtNode;
        let base = self.nodes.as_ptr();

        if ptr >= base && ptr < unsafe { base.add(self.nodes.len()) } {
            let offset = unsafe { ptr.offset_from(base) } as usize;
            (offset + 1) as NodeIndex
        } else {
            0
        }
    }

    /// Gets a node at the given index
    pub fn get_node_at_idx(&self, idx: NodeIndex) -> Option<&DtNode> {
        if idx == 0 {
            return None;
        }

        let node_idx = (idx - 1) as usize;
        if node_idx < self.nodes.len() {
            Some(&self.nodes[node_idx])
        } else {
            None
        }
    }

    /// Gets a mutable node at the given index
    pub fn get_node_at_idx_mut(&mut self, idx: NodeIndex) -> Option<&mut DtNode> {
        if idx == 0 {
            return None;
        }

        let node_idx = (idx - 1) as usize;
        if node_idx < self.nodes.len() {
            Some(&mut self.nodes[node_idx])
        } else {
            None
        }
    }

    /// Gets memory used by the pool
    pub fn get_mem_used(&self) -> usize {
        std::mem::size_of::<Self>()
            + std::mem::size_of::<DtNode>() * self.max_nodes
            + std::mem::size_of::<NodeIndex>() * self.max_nodes
            + std::mem::size_of::<NodeIndex>() * self.hash_size
    }

    /// Gets the maximum number of nodes
    pub fn get_max_nodes(&self) -> usize {
        self.max_nodes
    }

    /// Gets the hash size
    pub fn get_hash_size(&self) -> usize {
        self.hash_size
    }

    /// Gets the current node count
    pub fn get_node_count(&self) -> usize {
        self.node_count
    }

    /// Hashes a polygon ref
    fn hash_ref(id: PolyRef) -> usize {
        let a = id.id() as usize;
        a ^ (a >> 16)
    }
}

/// Priority queue for nodes during pathfinding
pub struct DtNodeQueue {
    /// Heap storage
    heap: Vec<*mut DtNode>,
    /// Capacity
    capacity: usize,
    /// Current size
    size: usize,
}

impl DtNodeQueue {
    /// Creates a new node queue
    pub fn new(capacity: usize) -> Self {
        Self {
            heap: Vec::with_capacity(capacity + 1),
            capacity,
            size: 0,
        }
    }

    /// Clears the queue
    pub fn clear(&mut self) {
        self.size = 0;
        self.heap.clear();
    }

    /// Gets the top node (minimum cost)
    pub fn top(&self) -> Option<&DtNode> {
        if self.size > 0 {
            unsafe { Some(&**self.heap.get_unchecked(0)) }
        } else {
            None
        }
    }

    /// Pops the top node
    pub fn pop(&mut self) -> Option<&mut DtNode> {
        if self.size == 0 {
            return None;
        }

        let result = self.heap[0];
        self.size -= 1;

        if self.size > 0 {
            let last = self.heap[self.size];
            self.trickle_down(0, last);
        }

        self.heap.truncate(self.size);
        unsafe { Some(&mut *result) }
    }

    /// Pushes a node onto the queue
    pub fn push(&mut self, node: &mut DtNode) {
        if self.size >= self.capacity {
            return;
        }

        self.heap.push(node as *mut DtNode);
        self.bubble_up(self.size, node as *mut DtNode);
        self.size += 1;
    }

    /// Modifies a node's position in the queue
    pub fn modify(&mut self, node: &DtNode) {
        let node_ptr = node as *const DtNode as *mut DtNode;

        for i in 0..self.size {
            if self.heap[i] == node_ptr {
                self.bubble_up(i, node_ptr);
                return;
            }
        }
    }

    /// Checks if the queue is empty
    pub fn empty(&self) -> bool {
        self.size == 0
    }

    /// Gets memory used by the queue
    pub fn get_mem_used(&self) -> usize {
        std::mem::size_of::<Self>() + std::mem::size_of::<*mut DtNode>() * (self.capacity + 1)
    }

    /// Gets the capacity
    pub fn get_capacity(&self) -> usize {
        self.capacity
    }

    /// Bubbles a node up the heap
    fn bubble_up(&mut self, mut i: usize, node: *mut DtNode) {
        let node_total = unsafe { (*node).total };

        while i > 0 {
            let parent = (i - 1) / 2;
            let parent_total = unsafe { (*self.heap[parent]).total };

            if node_total >= parent_total {
                break;
            }

            self.heap[i] = self.heap[parent];
            i = parent;
        }

        self.heap[i] = node;
    }

    /// Trickles a node down the heap
    fn trickle_down(&mut self, mut i: usize, node: *mut DtNode) {
        let node_total = unsafe { (*node).total };

        loop {
            let child1 = 2 * i + 1;
            if child1 >= self.size {
                break;
            }

            let child2 = child1 + 1;
            let mut min_child = child1;

            if child2 < self.size {
                let child1_total = unsafe { (*self.heap[child1]).total };
                let child2_total = unsafe { (*self.heap[child2]).total };

                if child2_total < child1_total {
                    min_child = child2;
                }
            }

            let min_child_total = unsafe { (*self.heap[min_child]).total };
            if node_total <= min_child_total {
                break;
            }

            self.heap[i] = self.heap[min_child];
            i = min_child;
        }

        self.heap[i] = node;
    }
}

// Safety: The raw pointers in DtNodeQueue are only used within the lifetime of the nodes
unsafe impl Send for DtNodeQueue {}
unsafe impl Sync for DtNodeQueue {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_pool() {
        let mut pool = DtNodePool::new(16, 8);

        // Test allocation
        let poly1 = PolyRef::new(1);
        let node1 = pool.get_node(poly1, 0).unwrap();
        assert_eq!(node1.id, poly1);
        assert_eq!(node1.state, 0);

        // Test finding
        let found = pool.find_node(poly1, 0);
        assert!(found.is_some());
        assert_eq!(found.unwrap().id, poly1);

        // Test multiple states
        let node2 = pool.get_node(poly1, 1).unwrap();
        assert_eq!(node2.id, poly1);
        assert_eq!(node2.state, 1);

        // Test find_nodes
        let nodes = pool.find_nodes(poly1, 10);
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn test_node_queue() {
        let mut pool = DtNodePool::new(16, 8);
        let mut queue = DtNodeQueue::new(16);

        // Create some nodes
        let poly1 = PolyRef::new(1);
        let poly2 = PolyRef::new(2);
        let poly3 = PolyRef::new(3);

        // Set up node 1
        {
            let node1 = pool.get_node(poly1, 0).unwrap();
            node1.total = 5.0;
            queue.push(node1);
        }

        // Set up node 2
        {
            let node2 = pool.get_node(poly2, 0).unwrap();
            node2.total = 3.0;
            queue.push(node2);
        }

        // Set up node 3
        {
            let node3 = pool.get_node(poly3, 0).unwrap();
            node3.total = 7.0;
            queue.push(node3);
        }

        // Pop in order of total cost
        let top1 = queue.pop().unwrap();
        assert_eq!(top1.id, poly2); // lowest cost

        let top2 = queue.pop().unwrap();
        assert_eq!(top2.id, poly1);

        let top3 = queue.pop().unwrap();
        assert_eq!(top3.id, poly3); // highest cost

        assert!(queue.empty());
    }
}
