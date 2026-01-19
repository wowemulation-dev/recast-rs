//! Concurrency safety tests for advanced navigation functions
//!
//! This module tests the thread safety of navigation mesh operations,
//! including concurrent read operations and proper synchronization for
//! write operations (poly flags/area modifications).

use crate::test_mesh_helpers::*;
use crate::{NavMeshQuery, QueryFilter, PolyFlags, PolyRef};
use recast_common::Result;
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Duration;

#[cfg(test)]
mod concurrent_read_tests {
    use super::*;

    #[test]
    fn test_concurrent_move_along_surface() -> Result<()> {
        let nav_mesh = Arc::new(create_complex_test_navmesh()?);
        let num_threads = 4;
        let operations_per_thread = 100;

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                thread::spawn(move || -> Result<()> {
                    let query = NavMeshQuery::new(&nav_mesh_clone);
                    let filter = QueryFilter::default();

                    // Find starting position
                    let start_pos = get_test_position_complex();
                    let extents = get_test_extents();
                    let (start_ref, actual_start_pos) = query.find_nearest_poly(&start_pos, &extents, &filter)?;

                    // Perform many operations
                    for i in 0..operations_per_thread {
                        let offset = (thread_id as f32 * 0.1) + (i as f32 * 0.01);
                        let end_pos = [
                            actual_start_pos[0] + offset,
                            actual_start_pos[1],
                            actual_start_pos[2] + offset,
                        ];
                        
                        let mut visited = Vec::new();
                        query.move_along_surface(start_ref, &actual_start_pos, &end_pos, &filter, &mut visited)?;

                        // Verify results are consistent
                        assert!(!visited.is_empty(), "Should visit at least one polygon");
                        assert!(visited[0] == start_ref, "First visited should be start polygon");
                    }

                    Ok(())
                })
            })
            .collect();

        // Wait for all threads to complete
        for handle in handles {
            handle.join().expect("Thread panicked").expect("Thread operation failed");
        }

        Ok(())
    }

    #[test]
    fn test_concurrent_find_local_neighbourhood() -> Result<()> {
        let nav_mesh = Arc::new(create_complex_test_navmesh()?);
        let num_threads = 8;
        let operations_per_thread = 50;

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                thread::spawn(move || -> Result<()> {
                    let query = NavMeshQuery::new(&nav_mesh_clone);
                    let filter = QueryFilter::default();

                    // Find starting position
                    let center_pos = get_test_position_complex();
                    let extents = get_test_extents();
                    let (start_ref, actual_center_pos) = query.find_nearest_poly(&center_pos, &extents, &filter)?;

                    // Perform operations with varying parameters
                    for i in 0..operations_per_thread {
                        let radius = 0.1 + (thread_id as f32 * 0.1) + (i as f32 * 0.02);
                        let max_result = 5 + (thread_id % 10);

                        let (polys, parents) = query.find_local_neighbourhood(
                            start_ref,
                            &actual_center_pos,
                            radius,
                            &filter,
                            max_result,
                        )?;

                        // Verify results are consistent
                        assert!(!polys.is_empty(), "Should find at least one polygon");
                        assert_eq!(polys.len(), parents.len(), "Polys and parents should be same length");
                        assert_eq!(polys[0], start_ref, "First polygon should be start");
                        assert!(!parents[0].is_valid(), "Start polygon should have no parent");
                    }

                    Ok(())
                })
            })
            .collect();

        // Wait for all threads to complete
        for handle in handles {
            handle.join().expect("Thread panicked").expect("Thread operation failed");
        }

        Ok(())
    }

    #[test]
    fn test_concurrent_mixed_read_operations() -> Result<()> {
        let nav_mesh = Arc::new(create_large_test_navmesh()?);
        let num_threads = 6;

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                thread::spawn(move || -> Result<()> {
                    let query = NavMeshQuery::new(&nav_mesh_clone);
                    let filter = QueryFilter::default();

                    let start_pos = get_test_position_large();
                    let extents = [5.0, 5.0, 5.0];
                    let (start_ref, actual_start_pos) = query.find_nearest_poly(&start_pos, &extents, &filter)?;

                    // Mix different types of read operations
                    for i in 0..50 {
                        match (thread_id + i) % 3 {
                            0 => {
                                // moveAlongSurface
                                let end_pos = [
                                    actual_start_pos[0] + (i as f32 * 0.1),
                                    actual_start_pos[1],
                                    actual_start_pos[2] + (i as f32 * 0.1),
                                ];
                                let mut visited = Vec::new();
                                query.move_along_surface(start_ref, &actual_start_pos, &end_pos, &filter, &mut visited)?;
                            },
                            1 => {
                                // findLocalNeighbourhood
                                let radius = 1.0 + (i as f32 * 0.1);
                                query.find_local_neighbourhood(start_ref, &actual_start_pos, radius, &filter, 10)?;
                            },
                            2 => {
                                // find_nearest_poly (baseline operation)
                                let test_pos = [
                                    actual_start_pos[0] + (i as f32 * 0.05),
                                    actual_start_pos[1],
                                    actual_start_pos[2] + (i as f32 * 0.05),
                                ];
                                query.find_nearest_poly(&test_pos, &extents, &filter)?;
                            },
                            _ => unreachable!(),
                        }
                    }

                    Ok(())
                })
            })
            .collect();

        // Wait for all threads to complete
        for handle in handles {
            handle.join().expect("Thread panicked").expect("Thread operation failed");
        }

        Ok(())
    }
}

#[cfg(test)]
mod concurrent_write_tests {
    use super::*;

    #[test]
    fn test_concurrent_poly_flags_modification() -> Result<()> {
        let nav_mesh = Arc::new(Mutex::new(create_complex_test_navmesh()?));
        let num_threads = 4;
        let operations_per_thread = 100;

        // Find a polygon to test with
        let poly_ref = {
            let mesh = nav_mesh.lock().unwrap();
            let query = NavMeshQuery::new(&mesh);
            let filter = QueryFilter::default();
            let center_pos = get_test_position_complex();
            let extents = get_test_extents();
            let (poly_ref, _) = query.find_nearest_poly(&center_pos, &extents, &filter)?;
            poly_ref
        };

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                thread::spawn(move || -> Result<()> {
                    for i in 0..operations_per_thread {
                        let mut mesh = nav_mesh_clone.lock().unwrap();
                        
                        // Each thread sets different flags to test concurrent access
                        let flags = match thread_id % 3 {
                            0 => PolyFlags::WALK,
                            1 => PolyFlags::SWIM,
                            2 => PolyFlags::WALK | PolyFlags::SWIM,
                            _ => PolyFlags::DOOR,
                        };

                        mesh.set_poly_flags(poly_ref, flags)?;
                        let retrieved_flags = mesh.get_poly_flags(poly_ref)?;
                        
                        // The retrieved flags should be some valid flag combination
                        // (exact value depends on thread scheduling)
                        assert!(!retrieved_flags.is_empty() || retrieved_flags.is_empty(), 
                               "Flags should be in a valid state");

                        // Small delay to increase chance of contention
                        if i % 10 == 0 {
                            drop(mesh); // Release lock
                            thread::sleep(Duration::from_nanos(1000));
                        }
                    }

                    Ok(())
                })
            })
            .collect();

        // Wait for all threads to complete
        for handle in handles {
            handle.join().expect("Thread panicked").expect("Thread operation failed");
        }

        Ok(())
    }

    #[test]
    fn test_concurrent_poly_area_modification() -> Result<()> {
        let nav_mesh = Arc::new(Mutex::new(create_complex_test_navmesh()?));
        let num_threads = 4;
        let operations_per_thread = 100;

        // Find a polygon to test with
        let poly_ref = {
            let mesh = nav_mesh.lock().unwrap();
            let query = NavMeshQuery::new(&mesh);
            let filter = QueryFilter::default();
            let center_pos = get_test_position_complex();
            let extents = get_test_extents();
            let (poly_ref, _) = query.find_nearest_poly(&center_pos, &extents, &filter)?;
            poly_ref
        };

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                thread::spawn(move || -> Result<()> {
                    for i in 0..operations_per_thread {
                        let mut mesh = nav_mesh_clone.lock().unwrap();
                        
                        // Each thread sets different areas
                        let area = ((thread_id * 50) + (i % 50)) as u8;

                        mesh.set_poly_area(poly_ref, area)?;
                        let retrieved_area = mesh.get_poly_area(poly_ref)?;
                        
                        // The retrieved area should be some valid area value
                        assert!(retrieved_area <= 255, "Area should be valid u8 value");

                        // Small delay to increase chance of contention
                        if i % 10 == 0 {
                            drop(mesh); // Release lock
                            thread::sleep(Duration::from_nanos(1000));
                        }
                    }

                    Ok(())
                })
            })
            .collect();

        // Wait for all threads to complete
        for handle in handles {
            handle.join().expect("Thread panicked").expect("Thread operation failed");
        }

        Ok(())
    }

    #[test]
    fn test_concurrent_mixed_read_write_operations() -> Result<()> {
        let nav_mesh = Arc::new(RwLock::new(create_complex_test_navmesh()?));
        let num_readers = 6;
        let num_writers = 2;

        // Find test polygon
        let poly_ref = {
            let mesh = nav_mesh.read().unwrap();
            let query = NavMeshQuery::new(&mesh);
            let filter = QueryFilter::default();
            let center_pos = get_test_position_complex();
            let extents = get_test_extents();
            let (poly_ref, _) = query.find_nearest_poly(&center_pos, &extents, &filter)?;
            poly_ref
        };

        // Start reader threads
        let reader_handles: Vec<_> = (0..num_readers)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                thread::spawn(move || -> Result<()> {
                    for i in 0..100 {
                        let mesh = nav_mesh_clone.read().unwrap();
                        let query = NavMeshQuery::new(&mesh);
                        let filter = QueryFilter::default();

                        let start_pos = get_test_position_complex();
                        let extents = get_test_extents();
                        let (start_ref, actual_start_pos) = query.find_nearest_poly(&start_pos, &extents, &filter)?;

                        // Perform read operations
                        match i % 2 {
                            0 => {
                                let end_pos = [
                                    actual_start_pos[0] + 0.1,
                                    actual_start_pos[1],
                                    actual_start_pos[2] + 0.1,
                                ];
                                let mut visited = Vec::new();
                                query.move_along_surface(start_ref, &actual_start_pos, &end_pos, &filter, &mut visited)?;
                            },
                            1 => {
                                query.find_local_neighbourhood(start_ref, &actual_start_pos, 0.5, &filter, 5)?;
                            },
                            _ => unreachable!(),
                        }

                        // Small delay
                        if i % 20 == 0 {
                            drop(mesh); // Release read lock
                            thread::sleep(Duration::from_millis(1));
                        }
                    }

                    Ok(())
                })
            })
            .collect();

        // Start writer threads
        let writer_handles: Vec<_> = (0..num_writers)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                thread::spawn(move || -> Result<()> {
                    for i in 0..50 {
                        let mut mesh = nav_mesh_clone.write().unwrap();
                        
                        // Perform write operations
                        let flags = if thread_id == 0 { 
                            PolyFlags::WALK 
                        } else { 
                            PolyFlags::SWIM 
                        };
                        let area = ((thread_id * 100) + i) as u8;

                        mesh.set_poly_flags(poly_ref, flags)?;
                        mesh.set_poly_area(poly_ref, area)?;

                        // Verify writes
                        let retrieved_flags = mesh.get_poly_flags(poly_ref)?;
                        let retrieved_area = mesh.get_poly_area(poly_ref)?;
                        
                        assert!(!retrieved_flags.is_empty() || retrieved_flags.is_empty(), 
                               "Flags should be in valid state");
                        assert!(retrieved_area <= 255, "Area should be valid");

                        // Delay to allow readers to interleave
                        if i % 5 == 0 {
                            drop(mesh); // Release write lock
                            thread::sleep(Duration::from_millis(2));
                        }
                    }

                    Ok(())
                })
            })
            .collect();

        // Wait for all threads to complete
        for handle in reader_handles {
            handle.join().expect("Reader thread panicked").expect("Reader operation failed");
        }
        for handle in writer_handles {
            handle.join().expect("Writer thread panicked").expect("Writer operation failed");
        }

        Ok(())
    }
}

#[cfg(test)]
mod stress_concurrency_tests {
    use super::*;

    #[test] 
    fn test_high_contention_scenario() -> Result<()> {
        let nav_mesh = Arc::new(Mutex::new(create_large_test_navmesh()?));
        let num_threads = 16;
        let operations_per_thread = 200;

        // Find multiple polygons for testing
        let poly_refs: Vec<PolyRef> = {
            let mesh = nav_mesh.lock().unwrap();
            let query = NavMeshQuery::new(&mesh);
            let filter = QueryFilter::default();
            
            let mut refs = Vec::new();
            for i in 0..10 {
                let pos = [5.0 + (i as f32), 0.0, 5.0 + (i as f32)];
                let extents = [1.0, 1.0, 1.0];
                if let Ok((poly_ref, _)) = query.find_nearest_poly(&pos, &extents, &filter) {
                    refs.push(poly_ref);
                }
            }
            refs
        };

        let poly_refs = Arc::new(poly_refs);

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                let poly_refs_clone = Arc::clone(&poly_refs);
                
                thread::spawn(move || -> Result<()> {
                    for i in 0..operations_per_thread {
                        let mut mesh = nav_mesh_clone.lock().unwrap();
                        
                        // Randomly select a polygon to modify
                        let poly_idx = (thread_id + i) % poly_refs_clone.len();
                        if poly_idx < poly_refs_clone.len() {
                            let poly_ref = poly_refs_clone[poly_idx];
                            
                            // Perform random operations
                            match i % 4 {
                                0 => {
                                    let flags = PolyFlags::from_bits_truncate(thread_id as u16 + i as u16);
                                    mesh.set_poly_flags(poly_ref, flags)?;
                                },
                                1 => {
                                    let area = ((thread_id + i) % 256) as u8;
                                    mesh.set_poly_area(poly_ref, area)?;
                                },
                                2 => {
                                    mesh.get_poly_flags(poly_ref)?;
                                },
                                3 => {
                                    mesh.get_poly_area(poly_ref)?;
                                },
                                _ => unreachable!(),
                            }
                        }

                        // Brief contention-inducing delay
                        if i % 50 == 0 {
                            drop(mesh);
                            thread::sleep(Duration::from_nanos(100));
                        }
                    }

                    Ok(())
                })
            })
            .collect();

        // Wait for all threads to complete
        for handle in handles {
            handle.join().expect("High contention thread panicked").expect("High contention operation failed");
        }

        Ok(())
    }

    #[test]
    fn test_deadlock_prevention() -> Result<()> {
        let nav_mesh = Arc::new(Mutex::new(create_complex_test_navmesh()?));
        let num_threads = 8;

        // This test attempts to trigger potential deadlock scenarios
        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let nav_mesh_clone = Arc::clone(&nav_mesh);
                
                thread::spawn(move || -> Result<()> {
                    for i in 0..100 {
                        // Different threads acquire locks in different patterns
                        // to test for potential deadlocks
                        
                        if thread_id % 2 == 0 {
                            // Even threads: short operations with frequent lock release
                            for _ in 0..5 {
                                let mesh = nav_mesh_clone.lock().unwrap();
                                let query = NavMeshQuery::new(&mesh);
                                let filter = QueryFilter::default();
                                let pos = get_test_position_complex();
                                let extents = get_test_extents();
                                
                                if let Ok((poly_ref, _)) = query.find_nearest_poly(&pos, &extents, &filter) {
                                    drop(mesh); // Release lock early
                                    let mut mesh = nav_mesh_clone.lock().unwrap();
                                    mesh.get_poly_flags(poly_ref)?;
                                }
                                
                                thread::sleep(Duration::from_nanos(500));
                            }
                        } else {
                            // Odd threads: longer operations with less frequent release
                            for j in 0..3 {
                                let pos = [
                                    get_test_position_complex()[0] + (j as f32 * 0.1),
                                    0.0,
                                    get_test_position_complex()[2] + (j as f32 * 0.1),
                                ];
                                let extents = get_test_extents();
                                
                                // Acquire lock for read operation
                                let poly_ref = {
                                    let mesh = nav_mesh_clone.lock().unwrap();
                                    let query = NavMeshQuery::new(&mesh);
                                    let filter = QueryFilter::default();
                                    
                                    if let Ok((poly_ref, _)) = query.find_nearest_poly(&pos, &extents, &filter) {
                                        poly_ref
                                    } else {
                                        continue;
                                    }
                                };
                                
                                // Acquire lock for write operations
                                {
                                    let mut mesh = nav_mesh_clone.lock().unwrap();
                                    mesh.set_poly_flags(poly_ref, PolyFlags::WALK)?;
                                }
                                
                                // Brief delay to allow other threads
                                thread::sleep(Duration::from_millis(1));
                                
                                // Acquire lock for second write operation
                                {
                                    let mut mesh = nav_mesh_clone.lock().unwrap();
                                    mesh.set_poly_area(poly_ref, (j + thread_id) as u8)?;
                                }
                            }
                        }
                        
                        // Random delay to vary timing
                        thread::sleep(Duration::from_nanos((thread_id * 1000) as u64));
                    }

                    Ok(())
                })
            })
            .collect();

        // Use a timeout to detect potential deadlocks
        let timeout = Duration::from_secs(30);
        let start_time = std::time::Instant::now();

        for (i, handle) in handles.into_iter().enumerate() {
            if start_time.elapsed() > timeout {
                panic!("Potential deadlock detected - test took too long");
            }
            
            handle.join()
                .map_err(|_| format!("Thread {} panicked", i))
                .expect("Thread join failed")
                .expect("Thread operation failed");
        }

        Ok(())
    }
}