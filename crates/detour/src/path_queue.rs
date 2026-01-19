use crate::{NavMesh, NavMeshQuery, PolyRef, QueryFilter, Status};
use recast_common::Error;

pub const DT_PATHQ_INVALID: u32 = 0;
const MAX_QUEUE: usize = 8;

pub type DtPathQueueRef = u32;

#[derive(Clone)]
struct PathQuery {
    reference: DtPathQueueRef,
    start_pos: [f32; 3],
    end_pos: [f32; 3],
    start_ref: PolyRef,
    end_ref: PolyRef,
    path: Vec<PolyRef>,
    status: Status,
    keep_alive: i32,
    filter: Option<QueryFilter>,
}

impl Default for PathQuery {
    fn default() -> Self {
        Self {
            reference: DT_PATHQ_INVALID,
            start_pos: [0.0; 3],
            end_pos: [0.0; 3],
            start_ref: PolyRef::new(0),
            end_ref: PolyRef::new(0),
            path: Vec::new(),
            status: Status::Failure,
            keep_alive: 0,
            filter: None,
        }
    }
}

pub struct DtPathQueue<'a> {
    queue: [PathQuery; MAX_QUEUE],
    next_handle: DtPathQueueRef,
    max_path_size: usize,
    queue_head: usize,
    navquery: Option<NavMeshQuery<'a>>,
}

impl<'a> DtPathQueue<'a> {
    pub fn new() -> Self {
        Self {
            queue: [
                PathQuery::default(),
                PathQuery::default(),
                PathQuery::default(),
                PathQuery::default(),
                PathQuery::default(),
                PathQuery::default(),
                PathQuery::default(),
                PathQuery::default(),
            ],
            next_handle: 1,
            max_path_size: 0,
            queue_head: 0,
            navquery: None,
        }
    }

    pub fn init(
        &mut self,
        max_path_size: usize,
        _max_search_node_count: usize,
        nav: &'a NavMesh,
    ) -> Result<bool, Error> {
        self.max_path_size = max_path_size;

        // Initialize the navigation mesh query
        let navquery = NavMeshQuery::new(nav);
        self.navquery = Some(navquery);

        self.purge();
        Ok(true)
    }

    pub fn update(&mut self, max_iters: usize) -> Result<(), Error> {
        let mut iter_count = 0;

        // Process pending queries
        for _ in 0..MAX_QUEUE {
            if iter_count >= max_iters {
                break;
            }

            let query = &mut self.queue[self.queue_head];
            if query.reference == DT_PATHQ_INVALID {
                self.queue_head = (self.queue_head + 1) % MAX_QUEUE;
                continue;
            }

            // Decrement keep alive
            query.keep_alive -= 1;
            if query.keep_alive <= 0 {
                query.reference = DT_PATHQ_INVALID;
                query.status = Status::Failure;
                self.queue_head = (self.queue_head + 1) % MAX_QUEUE;
                continue;
            }

            // Skip if already completed
            if query.status != Status::InProgress {
                self.queue_head = (self.queue_head + 1) % MAX_QUEUE;
                continue;
            }

            // Process this query
            if let Some(ref mut navquery) = self.navquery {
                if let Some(ref filter) = query.filter {
                    match navquery.find_path(
                        query.start_ref,
                        query.end_ref,
                        &query.start_pos,
                        &query.end_pos,
                        filter,
                    ) {
                        Ok(path) => {
                            // Limit path size
                            let path_size = path.len().min(self.max_path_size);
                            query.path = path.into_iter().take(path_size).collect();
                            query.status = Status::Success;
                        }
                        Err(_) => {
                            query.status = Status::Failure;
                        }
                    }
                } else {
                    query.status = Status::Failure;
                }
            } else {
                query.status = Status::Failure;
            }

            iter_count += 1;
            self.queue_head = (self.queue_head + 1) % MAX_QUEUE;
        }

        Ok(())
    }

    pub fn request(
        &mut self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &QueryFilter,
    ) -> DtPathQueueRef {
        // Find an empty slot
        let mut slot_idx = None;
        for i in 0..MAX_QUEUE {
            if self.queue[i].reference == DT_PATHQ_INVALID {
                slot_idx = Some(i);
                break;
            }
        }

        let slot_idx = match slot_idx {
            Some(idx) => idx,
            None => {
                // Queue is full, return invalid reference
                return DT_PATHQ_INVALID;
            }
        };

        let reference = self.next_handle;
        self.next_handle += 1;
        if self.next_handle == DT_PATHQ_INVALID {
            self.next_handle = 1;
        }

        let query = &mut self.queue[slot_idx];
        query.reference = reference;
        query.start_ref = start_ref;
        query.end_ref = end_ref;
        query.start_pos = *start_pos;
        query.end_pos = *end_pos;
        query.status = Status::InProgress;
        query.keep_alive = 2; // Keep alive for 2 update cycles
        query.filter = Some(filter.clone());
        query.path.clear();

        reference
    }

    pub fn get_request_status(&self, reference: DtPathQueueRef) -> Status {
        for query in &self.queue {
            if query.reference == reference {
                return query.status;
            }
        }
        Status::Failure
    }

    pub fn get_path_result(
        &self,
        reference: DtPathQueueRef,
        path: &mut Vec<PolyRef>,
        max_path: usize,
    ) -> Result<Status, Error> {
        for query in &self.queue {
            if query.reference == reference {
                if query.status == Status::Success {
                    let copy_count = query.path.len().min(max_path);
                    path.clear();
                    path.extend_from_slice(&query.path[..copy_count]);
                }
                return Ok(query.status);
            }
        }
        Ok(Status::Failure)
    }

    pub fn get_nav_query(&self) -> Option<&NavMeshQuery<'_>> {
        self.navquery.as_ref()
    }

    fn purge(&mut self) {
        for query in &mut self.queue {
            query.reference = DT_PATHQ_INVALID;
            query.status = Status::Failure;
            query.keep_alive = 0;
            query.path.clear();
        }
        self.queue_head = 0;
    }
}

impl<'a> Default for DtPathQueue<'a> {
    fn default() -> Self {
        Self::new()
    }
}
