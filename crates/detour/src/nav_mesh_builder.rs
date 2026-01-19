//! Navigation mesh builder for creating properly formatted tile data
//!
//! This module provides the NavMeshBuilder which creates navigation mesh tiles
//! with all required data structures including BVH trees and polygon links.

use super::nav_mesh::encode_poly_ref;
use super::{
    BVNode, DT_EXT_LINK, MeshTile, NavMeshCreateParams, NavMeshParams, OffMeshConnection, Poly,
    PolyDetail, PolyFlags, PolyType, Status, TileHeader,
};
use recast::{MESH_NULL_IDX, PolyMesh, PolyMeshDetail};
use recast_common::{Error, Result};

/// Request for creating an external link between tiles
#[derive(Debug, Clone)]
pub struct ExternalLinkRequest {
    /// Source tile coordinates (x, y, layer)
    pub source_tile: (i32, i32, i32),
    /// Source polygon index within tile
    pub source_poly: usize,
    /// Source edge index
    pub source_edge: u8,
    /// Target polygon reference
    pub target_poly_ref: super::PolyRef,
    /// Link direction/side
    pub side: u8,
}

/// Builder for creating navigation mesh tiles from polygon mesh data
pub struct NavMeshBuilder;

impl NavMeshBuilder {
    /// Creates navigation mesh data from the provided parameters
    ///
    /// This is the main entry point for building navigation mesh tiles.
    /// It takes polygon mesh data and creates a properly formatted tile
    /// that can be added to a NavMesh.
    pub fn create_nav_mesh_data(params: &NavMeshCreateParams) -> Result<Vec<u8>> {
        // Validate input parameters
        Self::validate_params(params)?;

        // Create tile data structure
        let tile = Self::build_tile(params)?;

        // Serialize the tile to binary format
        super::binary_format::save_tile_to_binary(&tile)
    }

    /// Creates a navigation mesh tile from NavMeshCreateParams
    pub fn build_tile(params: &NavMeshCreateParams) -> Result<MeshTile> {
        let mut tile = MeshTile::new();

        // Set up tile header
        tile.header = Some(TileHeader {
            x: 0, // Will be set when adding to NavMesh
            y: 0,
            layer: 0,
            user_id: 0,
            data_size: 0, // Will be calculated during serialization
            bmin: params.bmin,
            bmax: params.bmax,
            poly_count: params.poly_count,
            vert_count: params.vert_count,
            max_links: params.poly_count * 6, // Max 6 links per polygon
            detail_mesh_count: params.poly_count,
            detail_vert_count: params.detail_vert_count,
            detail_tri_count: params.detail_tri_count,
            bvh_node_count: 0, // Will be set after building BVH
            off_mesh_connection_count: params.off_mesh_con_count,
            bv_quant_factor: 0.0,
        });

        // Copy vertices
        tile.verts = params.verts.clone();

        // Build polygons
        tile.polys = Self::build_polygons(params)?;

        // Copy detail meshes
        tile.detail_meshes = Self::build_detail_meshes(params)?;

        // Copy detail vertices and triangles
        tile.detail_verts = params.detail_verts.clone();
        tile.detail_tris = params.detail_tris.clone();

        // Build off-mesh connections
        if params.off_mesh_con_count > 0 {
            tile.off_mesh_connections = Self::build_off_mesh_connections(params)?;
        }

        // Build BVH tree if requested
        if params.build_bv_tree {
            Self::build_bvh_for_tile(&mut tile)?;
        }

        // Build internal polygon links
        Self::build_internal_links(&mut tile)?;

        Ok(tile)
    }

    /// Validates input parameters
    fn validate_params(params: &NavMeshCreateParams) -> Result<()> {
        if params.vert_count < 3 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.poly_count < 1 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.nvp < 3 || params.nvp > 6 {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.verts.len() != (params.vert_count * 3) as usize {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        if params.polys.len() != (params.poly_count * params.nvp * 2) as usize {
            return Err(Error::Detour(Status::InvalidParam.to_string()));
        }

        Ok(())
    }

    /// Builds polygon structures from raw data
    fn build_polygons(params: &NavMeshCreateParams) -> Result<Vec<Poly>> {
        let mut polys = Vec::with_capacity(params.poly_count as usize);
        let mut poly_idx = 0;

        for i in 0..params.poly_count as usize {
            // Get polygon properties
            let area = params.poly_areas.get(i).copied().unwrap_or(0);
            let flags = params.poly_flags.get(i).copied().unwrap_or(PolyFlags::WALK);
            let mut poly = Poly::new(area, PolyType::Ground, flags);

            // Read polygon vertices
            let mut vert_count = 0;
            for j in 0..params.nvp as usize {
                let idx = poly_idx + j;
                if idx < params.polys.len() {
                    let vert_idx = params.polys[idx];
                    if vert_idx != MESH_NULL_IDX {
                        poly.verts[vert_count] = vert_idx;
                        vert_count += 1;
                    }
                }
            }

            // Read polygon neighbors
            for j in 0..params.nvp as usize {
                let idx = poly_idx + params.nvp as usize + j;
                if idx < params.polys.len() {
                    let neighbor = params.polys[idx];

                    // Check if this is an external link marker (0x8000 | direction)
                    if (neighbor & DT_EXT_LINK) != 0 {
                        // This edge connects to an adjacent tile - store the marker as-is
                        poly.neighbors[j] = neighbor;
                    } else {
                        // Regular internal neighbor or null
                        poly.neighbors[j] = neighbor;
                    }
                } else {
                    poly.neighbors[j] = MESH_NULL_IDX;
                }
            }

            poly_idx += params.nvp as usize * 2;
            poly.vert_count = vert_count as u8;

            polys.push(poly);
        }

        Ok(polys)
    }

    /// Builds detail mesh structures
    fn build_detail_meshes(params: &NavMeshCreateParams) -> Result<Vec<PolyDetail>> {
        let mut detail_meshes = Vec::with_capacity(params.poly_count as usize);

        for i in 0..params.poly_count as usize {
            let base_idx = i * 4;
            if base_idx + 3 < params.detail_meshes.len() {
                detail_meshes.push(PolyDetail {
                    vert_base: params.detail_meshes[base_idx],
                    tri_base: params.detail_meshes[base_idx + 1],
                    vert_count: params.detail_meshes[base_idx + 2] as u8,
                    tri_count: params.detail_meshes[base_idx + 3] as u8,
                });
            } else {
                // Default detail mesh
                detail_meshes.push(PolyDetail {
                    vert_base: 0,
                    tri_base: (i * 3) as u32,
                    vert_count: 0,
                    tri_count: 1,
                });
            }
        }

        Ok(detail_meshes)
    }

    /// Builds off-mesh connections
    fn build_off_mesh_connections(params: &NavMeshCreateParams) -> Result<Vec<OffMeshConnection>> {
        let mut connections = Vec::with_capacity(params.off_mesh_con_count as usize);

        for i in 0..params.off_mesh_con_count as usize {
            let vert_idx = i * 6;
            if vert_idx + 5 >= params.off_mesh_con_verts.len() {
                break;
            }

            let mut conn = OffMeshConnection::new();

            // Copy position data
            conn.pos
                .copy_from_slice(&params.off_mesh_con_verts[vert_idx..vert_idx + 6]);

            // Set connection properties
            conn.radius = params.off_mesh_con_rad.get(i).copied().unwrap_or(1.0);
            conn.flags = params
                .off_mesh_con_flags
                .get(i)
                .copied()
                .unwrap_or(PolyFlags::WALK);
            conn.area = params.off_mesh_con_areas.get(i).copied().unwrap_or(0);
            conn.dir = params.off_mesh_con_dir.get(i).copied().unwrap_or(0);
            conn.user_id = params.off_mesh_con_user_id.get(i).copied().unwrap_or(0);

            connections.push(conn);
        }

        Ok(connections)
    }

    /// Builds BVH tree for the tile
    fn build_bvh_for_tile(tile: &mut MeshTile) -> Result<()> {
        let poly_count = tile.polys.len();
        if poly_count == 0 {
            return Ok(());
        }

        // Calculate bounding boxes for all polygons
        let mut poly_bounds: Vec<([f32; 3], [f32; 3])> = Vec::with_capacity(poly_count);

        for poly in &tile.polys {
            let mut bmin = [f32::MAX; 3];
            let mut bmax = [f32::MIN; 3];

            // Calculate polygon bounds
            for j in 0..poly.vert_count as usize {
                let v_idx = poly.verts[j] as usize * 3;
                if v_idx + 2 < tile.verts.len() {
                    for k in 0..3 {
                        bmin[k] = bmin[k].min(tile.verts[v_idx + k]);
                        bmax[k] = bmax[k].max(tile.verts[v_idx + k]);
                    }
                }
            }

            poly_bounds.push((bmin, bmax));
        }

        // Build the BVH tree
        let mut items: Vec<usize> = (0..poly_count).collect();
        Self::build_bvh_recursive(tile, &poly_bounds, &mut items, 0, poly_count)?;

        // Update header with BVH node count
        if let Some(header) = tile.header.as_mut() {
            header.bvh_node_count = tile.bvh_nodes.len() as i32;
        }

        Ok(())
    }

    /// Recursively builds BVH nodes
    fn build_bvh_recursive(
        tile: &mut MeshTile,
        poly_bounds: &[([f32; 3], [f32; 3])],
        items: &mut [usize],
        item_start: usize,
        item_count: usize,
    ) -> Result<i32> {
        if item_count == 0 {
            return Ok(-1);
        }

        let node_idx = tile.bvh_nodes.len() as i32;

        // Calculate bounds for all items
        let mut node_bmin = [f32::MAX; 3];
        let mut node_bmax = [f32::MIN; 3];

        for i in 0..item_count {
            let poly_idx = items[item_start + i];
            let (bmin, bmax) = poly_bounds[poly_idx];
            for k in 0..3 {
                node_bmin[k] = node_bmin[k].min(bmin[k]);
                node_bmax[k] = node_bmax[k].max(bmax[k]);
            }
        }

        // Get tile header for quantization
        let header = tile
            .header
            .as_ref()
            .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

        // Quantize bounds
        let (quant_bounds, _) =
            BVNode::quantize_bounds(&node_bmin, &node_bmax, &header.bmin, &header.bmax);

        // Create node
        let mut node = BVNode {
            bmin: quant_bounds.bmin,
            bmax: quant_bounds.bmax,
            i: 0,
        };

        // Leaf node for 1-2 items
        if item_count <= 2 {
            if item_count == 1 {
                node.i = -(items[item_start] as i32 + 1);
            } else {
                let idx0 = items[item_start] as i32;
                let idx1 = items[item_start + 1] as i32;
                node.i = (idx0 << 16) | (idx1 & 0xFFFF);
            }
            tile.bvh_nodes.push(node);
        } else {
            // Internal node
            tile.bvh_nodes.push(node); // Reserve space

            // Find best split axis
            let mut axis = 0;
            let mut max_span = node_bmax[0] - node_bmin[0];
            for k in 1..3 {
                let span = node_bmax[k] - node_bmin[k];
                if span > max_span {
                    axis = k;
                    max_span = span;
                }
            }

            // Sort along axis
            let items_slice = &mut items[item_start..item_start + item_count];
            items_slice.sort_by(|&a, &b| {
                let a_center = (poly_bounds[a].0[axis] + poly_bounds[a].1[axis]) * 0.5;
                let b_center = (poly_bounds[b].0[axis] + poly_bounds[b].1[axis]) * 0.5;
                a_center
                    .partial_cmp(&b_center)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            // Split and recurse
            let split = item_count / 2;
            let left = Self::build_bvh_recursive(tile, poly_bounds, items, item_start, split)?;
            let right = Self::build_bvh_recursive(
                tile,
                poly_bounds,
                items,
                item_start + split,
                item_count - split,
            )?;

            // Update node
            tile.bvh_nodes[node_idx as usize].i = ((left & 0xFFFF) << 16) | (right & 0xFFFF);
        }

        Ok(node_idx)
    }

    /// Builds internal links between polygons within a tile
    /// Builds internal links between polygons in a tile (public interface)
    pub fn build_internal_links_for_tile(tile: &mut MeshTile) -> Result<()> {
        Self::build_internal_links(tile)
    }

    fn build_internal_links(tile: &mut MeshTile) -> Result<()> {
        // For each polygon, check if it shares edges with neighbors
        for i in 0..tile.polys.len() {
            // Skip off-mesh connections - they don't use neighbor-based links
            if tile.polys[i].poly_type == super::PolyType::OffMeshConnection {
                continue;
            }

            // Collect neighbor information first
            let mut links_to_add = Vec::new();

            {
                let poly = &tile.polys[i];

                // Check each edge
                for j in 0..poly.vert_count as usize {
                    let neighbor_idx = poly.neighbors[j];

                    // Skip external link markers - they don't create internal links
                    if neighbor_idx != MESH_NULL_IDX && (neighbor_idx & DT_EXT_LINK) == 0 {
                        // Create internal link to neighbor
                        let neighbor_ref = encode_poly_ref(0, neighbor_idx as u32);

                        // Find which edge on the neighbor connects back
                        let mut _neighbor_edge = 0u8;
                        if (neighbor_idx as usize) < tile.polys.len() {
                            let neighbor_poly = &tile.polys[neighbor_idx as usize];
                            for k in 0..neighbor_poly.vert_count as usize {
                                if neighbor_poly.neighbors[k] == i as u16 {
                                    _neighbor_edge = k as u8;
                                    break;
                                }
                            }
                        }

                        // Create link
                        let link = super::Link::new(
                            neighbor_ref,
                            neighbor_idx as u8,
                            j as u8,
                            0, // side
                            0, // boundary_flag (0 = internal)
                        );

                        links_to_add.push(link);
                    }
                }
            }

            // Now add the links
            for link in links_to_add {
                let link_idx = tile.links.len();
                tile.links.push(link);

                // Update polygon's link list
                if tile.polys[i].first_link.is_none() {
                    tile.polys[i].first_link = Some(link_idx);
                } else {
                    // Find last link and append
                    let mut last_idx = tile.polys[i].first_link.unwrap();
                    while let Some(next) = tile.links[last_idx].next {
                        last_idx = next as usize;
                    }
                    tile.links[last_idx].next = Some(link_idx as u32);
                }
            }
        }

        Ok(())
    }

    /// Connects external links between tiles in the navigation mesh  
    /// This function collects external link requests for deferred processing
    pub fn connect_external_links(
        nav_mesh: &super::NavMesh,
        tx: i32,
        ty: i32,
        layer: i32,
    ) -> Result<Vec<ExternalLinkRequest>> {
        // Get the tile we're connecting from
        let tile = nav_mesh.get_tile_at(tx, ty, layer);
        if tile.is_none() {
            return Ok(Vec::new()); // No tile to connect from
        }

        let mut link_requests = Vec::new();

        // For each direction, try to find and connect to adjacent tiles
        let directions = [
            (1, 0, 0),  // East: direction 0
            (0, 1, 2),  // North: direction 2
            (-1, 0, 4), // West: direction 4
            (0, -1, 6), // South: direction 6
        ];

        for (dx, dy, dir) in directions {
            let neighbor_tx = tx + dx;
            let neighbor_ty = ty + dy;

            // Check if neighbor tile exists
            if nav_mesh
                .get_tile_at(neighbor_tx, neighbor_ty, layer)
                .is_some()
            {
                let mut requests = Self::collect_tile_link_requests(
                    nav_mesh,
                    tx,
                    ty,
                    layer,
                    neighbor_tx,
                    neighbor_ty,
                    layer,
                    dir,
                )?;
                link_requests.append(&mut requests);
            }
        }

        Ok(link_requests)
    }

    /// Collects external link requests between two specific tiles in a given direction
    #[allow(clippy::too_many_arguments)]
    fn collect_tile_link_requests(
        nav_mesh: &super::NavMesh,
        tx1: i32,
        ty1: i32,
        layer1: i32,
        tx2: i32,
        ty2: i32,
        layer2: i32,
        direction: u8,
    ) -> Result<Vec<ExternalLinkRequest>> {
        // Get references to both tiles
        let tile1 = nav_mesh.get_tile_at(tx1, ty1, layer1);
        let tile2 = nav_mesh.get_tile_at(tx2, ty2, layer2);

        if tile1.is_none() || tile2.is_none() {
            return Ok(Vec::new());
        }

        let mut requests = Vec::new();

        // Collect external link requests from tile1 to tile2 in the specified direction
        let mut forward_requests = Self::collect_ext_link_requests(
            nav_mesh,
            tx1,
            ty1,
            layer1,
            tx2,
            ty2,
            layer2,
            direction as i32,
        )?;
        requests.append(&mut forward_requests);

        // Collect reverse direction requests (from tile2 to tile1)
        let opposite_dir = Self::opposite_tile_dir(direction as i32);
        let mut reverse_requests = Self::collect_ext_link_requests(
            nav_mesh,
            tx2,
            ty2,
            layer2,
            tx1,
            ty1,
            layer1,
            opposite_dir,
        )?;
        requests.append(&mut reverse_requests);

        Ok(requests)
    }

    /// Collects external link requests (based on C++ connectExtLinks)
    #[allow(clippy::too_many_arguments)]
    fn collect_ext_link_requests(
        nav_mesh: &super::NavMesh,
        source_tx: i32,
        source_ty: i32,
        source_layer: i32,
        target_tx: i32,
        target_ty: i32,
        target_layer: i32,
        side: i32,
    ) -> Result<Vec<ExternalLinkRequest>> {
        let mut requests = Vec::new();

        // Get source tile data first to avoid borrow checker issues
        let source_tile_data = {
            let source_tile = nav_mesh.get_tile_at(source_tx, source_ty, source_layer);
            if source_tile.is_none() {
                return Ok(Vec::new());
            }

            let tile = source_tile.unwrap();
            let poly_count = tile.polys.len();
            let mut portal_edges = Vec::new();

            // Find all portal edges in the source tile
            for i in 0..poly_count {
                let poly = &tile.polys[i];
                let nv = poly.vert_count as usize;

                for j in 0..nv {
                    // Skip non-portal edges
                    if (poly.neighbors[j] & DT_EXT_LINK) == 0 {
                        continue;
                    }

                    let dir = (poly.neighbors[j] & 0xff) as i32;
                    if side != -1 && dir != side {
                        continue;
                    }

                    // Get edge vertices
                    let va_idx = (poly.verts[j] as usize) * 3;
                    let vb_idx = (poly.verts[(j + 1) % nv] as usize) * 3;

                    if va_idx + 2 < tile.verts.len() && vb_idx + 2 < tile.verts.len() {
                        let va = [
                            tile.verts[va_idx],
                            tile.verts[va_idx + 1],
                            tile.verts[va_idx + 2],
                        ];
                        let vb = [
                            tile.verts[vb_idx],
                            tile.verts[vb_idx + 1],
                            tile.verts[vb_idx + 2],
                        ];

                        portal_edges.push((i, j, va, vb, dir));
                    }
                }
            }

            portal_edges
        };

        // For each portal edge, find connecting polygons in target tile and create requests
        for (poly_idx, edge_idx, va, vb, dir) in source_tile_data {
            let connecting_polys = Self::find_connecting_polys(
                nav_mesh,
                &va,
                &vb,
                target_tx,
                target_ty,
                target_layer,
                Self::opposite_tile_dir(dir),
            )?;

            // Create link requests for each connecting polygon
            for conn_poly_ref in connecting_polys {
                requests.push(ExternalLinkRequest {
                    source_tile: (source_tx, source_ty, source_layer),
                    source_poly: poly_idx,
                    source_edge: edge_idx as u8,
                    target_poly_ref: conn_poly_ref,
                    side: dir as u8,
                });
            }
        }

        Ok(requests)
    }

    /// Find polygons in target tile that connect to the given edge (based on C++ findConnectingPolys)
    fn find_connecting_polys(
        nav_mesh: &super::NavMesh,
        va: &[f32; 3],
        vb: &[f32; 3],
        target_tx: i32,
        target_ty: i32,
        target_layer: i32,
        side: i32,
    ) -> Result<Vec<super::PolyRef>> {
        let target_tile = nav_mesh.get_tile_at(target_tx, target_ty, target_layer);
        if target_tile.is_none() {
            return Ok(Vec::new());
        }

        let tile = target_tile.unwrap();
        let mut connections = Vec::new();

        // Calculate slab end points for the source edge
        let (amin, amax) = Self::calc_slab_end_points(va, vb, side);
        let apos = Self::get_slab_coord(va, side);

        let external_marker = DT_EXT_LINK | (side as u16);

        // Check each polygon in the target tile
        for i in 0..tile.polys.len() {
            let poly = &tile.polys[i];
            let nv = poly.vert_count as usize;

            for j in 0..nv {
                // Skip edges that don't point to the right side
                if poly.neighbors[j] != external_marker {
                    continue;
                }

                // Get edge vertices
                let vc_idx = (poly.verts[j] as usize) * 3;
                let vd_idx = (poly.verts[(j + 1) % nv] as usize) * 3;

                if vc_idx + 2 >= tile.verts.len() || vd_idx + 2 >= tile.verts.len() {
                    continue;
                }

                let vc = [
                    tile.verts[vc_idx],
                    tile.verts[vc_idx + 1],
                    tile.verts[vc_idx + 2],
                ];
                let vd = [
                    tile.verts[vd_idx],
                    tile.verts[vd_idx + 1],
                    tile.verts[vd_idx + 2],
                ];

                let bpos = Self::get_slab_coord(&vc, side);

                // Segments must be close enough
                if (apos - bpos).abs() > 0.01 {
                    continue;
                }

                // Check if the segments overlap
                let (bmin, bmax) = Self::calc_slab_end_points(&vc, &vd, side);
                if Self::overlap_slabs(&amin, &amax, &bmin, &bmax, 0.01, 0.9) {
                    // Create polygon reference using tile reference at coordinates
                    if let Some(tile_ref) =
                        nav_mesh.get_tile_ref_at(target_tx, target_ty, target_layer)
                    {
                        // Extract tile ID from the tile reference and combine with polygon index
                        let (tile_id, _) = super::nav_mesh::decode_poly_ref(tile_ref);
                        let poly_ref = super::nav_mesh::encode_poly_ref(tile_id, i as u32);
                        connections.push(poly_ref);
                        break;
                    }
                }
            }
        }

        Ok(connections)
    }

    /// Calculate slab end points for portal edge matching
    fn calc_slab_end_points(va: &[f32; 3], vb: &[f32; 3], side: i32) -> ([f32; 2], [f32; 2]) {
        if side == 0 || side == 4 {
            // X-direction portals use Z coordinates
            let (min_z, max_z) = if va[2] < vb[2] {
                (va[2], vb[2])
            } else {
                (vb[2], va[2])
            };
            let (min_y, max_y) = if va[1] < vb[1] {
                (va[1], vb[1])
            } else {
                (vb[1], va[1])
            };
            ([min_z, min_y], [max_z, max_y])
        } else if side == 2 || side == 6 {
            // Z-direction portals use X coordinates
            let (min_x, max_x) = if va[0] < vb[0] {
                (va[0], vb[0])
            } else {
                (vb[0], va[0])
            };
            let (min_y, max_y) = if va[1] < vb[1] {
                (va[1], vb[1])
            } else {
                (vb[1], va[1])
            };
            ([min_x, min_y], [max_x, max_y])
        } else {
            ([0.0, 0.0], [0.0, 0.0])
        }
    }

    /// Get coordinate perpendicular to the edge direction
    fn get_slab_coord(v: &[f32; 3], side: i32) -> f32 {
        if side == 0 || side == 4 {
            v[0] // X coordinate for Z-direction edges
        } else if side == 2 || side == 6 {
            v[2] // Z coordinate for X-direction edges
        } else {
            0.0
        }
    }

    /// Check if two slab segments overlap
    fn overlap_slabs(
        amin: &[f32; 2],
        amax: &[f32; 2],
        bmin: &[f32; 2],
        bmax: &[f32; 2],
        px: f32,
        py: f32,
    ) -> bool {
        // Check for horizontal overlap
        let minx = (amin[0] + px).max(bmin[0] + px);
        let maxx = (amax[0] - px).min(bmax[0] - px);
        if minx > maxx {
            return false;
        }

        // Check vertical overlap with slope calculation
        let ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
        let ak = amin[1] - ad * amin[0];
        let bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
        let bk = bmin[1] - bd * bmin[0];

        let aminy = ad * minx + ak;
        let amaxy = ad * maxx + ak;
        let bminy = bd * minx + bk;
        let bmaxy = bd * maxx + bk;

        let dmin = bminy - aminy;
        let dmax = bmaxy - amaxy;

        // Crossing segments always overlap
        if dmin * dmax < 0.0 {
            return true;
        }

        // Check for overlap at endpoints
        let thr = py * py * 4.0;
        dmin * dmin <= thr || dmax * dmax <= thr
    }

    /// Get opposite tile direction
    fn opposite_tile_dir(dir: i32) -> i32 {
        match dir {
            0 => 4, // East -> West
            2 => 6, // North -> South
            4 => 0, // West -> East
            6 => 2, // South -> North
            _ => dir,
        }
    }

    /// Creates NavMeshCreateParams from PolyMesh and PolyMeshDetail
    pub fn create_params_from_polymesh(
        nav_params: &NavMeshParams,
        poly_mesh: &PolyMesh,
        detail_mesh: &PolyMeshDetail,
        walkable_height: f32,
        walkable_radius: f32,
        walkable_climb: f32,
    ) -> NavMeshCreateParams {
        // Convert vertices from integer to world coordinates
        let mut verts = Vec::with_capacity(poly_mesh.vert_count * 3);
        for i in 0..poly_mesh.vert_count {
            let x = poly_mesh.bmin.x + (poly_mesh.vertices[i * 3] as f32) * poly_mesh.cs;
            let y = poly_mesh.bmin.y + (poly_mesh.vertices[i * 3 + 1] as f32) * poly_mesh.ch;
            let z = poly_mesh.bmin.z + (poly_mesh.vertices[i * 3 + 2] as f32) * poly_mesh.cs;
            verts.push(x);
            verts.push(y);
            verts.push(z);
        }

        // Convert polygon data
        let polys = poly_mesh.polys.clone();
        let poly_flags = vec![PolyFlags::WALK; poly_mesh.poly_count];
        let poly_areas = poly_mesh.areas.clone();

        // Convert detail mesh data from PolyMeshDetail structure
        let mut detail_meshes = Vec::new();
        let mut detail_verts = Vec::new();
        let mut detail_tris = Vec::new();

        // For each polygon, create detail mesh entry
        for i in 0..poly_mesh.poly_count {
            let vert_base = 0u32; // No extra detail vertices in simple case
            let tri_base = (i * 2) as u32; // Assuming 2 triangles per quad polygon
            let vert_count = 0u32; // No extra detail vertices
            let tri_count = if i < detail_mesh.poly_tri_count.len() {
                detail_mesh.poly_tri_count[i] as u32
            } else {
                2u32 // Default to 2 triangles for a quad
            };

            detail_meshes.push(vert_base);
            detail_meshes.push(tri_base);
            detail_meshes.push(vert_count);
            detail_meshes.push(tri_count);
        }

        // Copy detail vertices (typically empty for simple meshes)
        if detail_mesh.vert_count > poly_mesh.vert_count {
            // Only copy extra detail vertices beyond the base mesh vertices
            let extra_vert_start = poly_mesh.vert_count * 3;
            if extra_vert_start < detail_mesh.vertices.len() {
                detail_verts = detail_mesh.vertices[extra_vert_start..].to_vec();
            }
        }

        // Convert detail triangles from u32 to u8
        for &tri_idx in &detail_mesh.triangles {
            if tri_idx <= 255 {
                detail_tris.push(tri_idx as u8);
            } else {
                // Handle large indices by wrapping or clamping
                detail_tris.push((tri_idx % 256) as u8);
            }
        }

        NavMeshCreateParams {
            nav_mesh_params: nav_params.clone(),
            verts,
            vert_count: poly_mesh.vert_count as i32,
            polys,
            poly_flags,
            poly_areas,
            poly_count: poly_mesh.poly_count as i32,
            nvp: poly_mesh.max_verts_per_poly as i32,
            detail_meshes,
            detail_verts,
            detail_vert_count: ((detail_mesh.vert_count as i32) - (poly_mesh.vert_count as i32))
                .max(0),
            detail_tris,
            detail_tri_count: detail_mesh.tri_count as i32,
            off_mesh_con_verts: Vec::new(),
            off_mesh_con_rad: Vec::new(),
            off_mesh_con_flags: Vec::new(),
            off_mesh_con_areas: Vec::new(),
            off_mesh_con_dir: Vec::new(),
            off_mesh_con_user_id: Vec::new(),
            off_mesh_con_count: 0,
            bmin: [poly_mesh.bmin.x, poly_mesh.bmin.y, poly_mesh.bmin.z],
            bmax: [poly_mesh.bmax.x, poly_mesh.bmax.y, poly_mesh.bmax.z],
            walkable_height,
            walkable_radius,
            walkable_climb,
            cs: poly_mesh.cs,
            ch: poly_mesh.ch,
            build_bv_tree: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nav_mesh_builder_creation() {
        // Create minimal valid params
        let nav_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 32.0,
            tile_height: 32.0,
            max_tiles: 64,
            max_polys_per_tile: 256,
        };

        let params = NavMeshCreateParams {
            nav_mesh_params: nav_params,
            verts: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0],
            vert_count: 4,
            polys: vec![
                0,
                MESH_NULL_IDX,
                1,
                MESH_NULL_IDX,
                2,
                MESH_NULL_IDX,
                3,
                MESH_NULL_IDX,
            ],
            poly_flags: vec![PolyFlags::WALK],
            poly_areas: vec![0],
            poly_count: 1,
            nvp: 4,
            detail_meshes: vec![0, 0, 0, 2],
            detail_verts: vec![],
            detail_vert_count: 0,
            detail_tris: vec![0, 1, 2, 0, 2, 3],
            detail_tri_count: 2,
            off_mesh_con_verts: vec![],
            off_mesh_con_rad: vec![],
            off_mesh_con_flags: vec![],
            off_mesh_con_areas: vec![],
            off_mesh_con_dir: vec![],
            off_mesh_con_user_id: vec![],
            off_mesh_con_count: 0,
            bmin: [0.0, 0.0, 0.0],
            bmax: [1.0, 0.0, 1.0],
            walkable_height: 2.0,
            walkable_radius: 0.6,
            walkable_climb: 0.9,
            cs: 0.3,
            ch: 0.2,
            build_bv_tree: true,
        };

        // Build tile
        let tile = NavMeshBuilder::build_tile(&params).unwrap();

        // Verify tile was built correctly
        assert_eq!(tile.polys.len(), 1);
        assert_eq!(tile.verts.len(), 12);
        assert!(!tile.bvh_nodes.is_empty());

        // Test binary data creation
        let data = NavMeshBuilder::create_nav_mesh_data(&params).unwrap();
        assert!(!data.is_empty());
    }
}
