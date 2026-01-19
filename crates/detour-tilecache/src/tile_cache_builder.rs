//! Tile cache builder for real-time mesh rebuilding
//!
//! This module provides functionality to rebuild navigation mesh tiles
//! from compressed tile cache data, with support for dynamic obstacles.

use super::tile_cache::{Obstacle, ObstacleData, ObstacleState, TileCache};
use super::tile_cache_data::{TileCacheBuilderConfig, TileCacheLayer};
use detour::nav_mesh::{MeshTile, Poly, PolyDetail, TileHeader};
use detour::{PolyFlags, PolyType, MAX_VERTS_PER_POLY};
use glam::Vec3;
use recast::{
    CompactHeightfield, ContourSet, Heightfield, PolyMesh, PolyMeshDetail, RecastConfig,
    MESH_NULL_IDX,
};
use recast_common::{Error, Result};

/// Tile cache builder for converting cached layers to navigation mesh tiles
#[derive(Debug)]
pub struct TileCacheBuilder {
    /// Configuration for building tiles
    config: TileCacheBuilderConfig,
    /// Recast configuration derived from builder config
    recast_config: RecastConfig,
}

impl TileCacheBuilder {
    /// Creates a new tile cache builder
    pub fn new(config: TileCacheBuilderConfig) -> Self {
        // Convert TileCacheBuilderConfig to RecastConfig
        let recast_config = RecastConfig {
            width: 0,  // Will be set per tile
            height: 0, // Will be set per tile
            cs: config.cs,
            ch: config.ch,
            bmin: Vec3::ZERO,     // Will be set per tile
            bmax: Vec3::ZERO,     // Will be set per tile
            walkable_slope_angle: 45.0, // Default, could be added to config
            walkable_height: config.walkable_height,
            walkable_climb: config.walkable_climb,
            walkable_radius: config.walkable_radius,
            max_edge_len: config.max_edge_len as i32,
            max_simplification_error: config.max_simplification_error,
            min_region_area: config.min_region_area,
            merge_region_area: config.merge_region_area,
            max_vertices_per_polygon: config.max_verts_per_poly,
            detail_sample_dist: 6.0,      // Default detail sampling
            detail_sample_max_error: 1.0, // Default detail error
            border_size: 0,               // Default, no border
        };

        Self {
            config,
            recast_config,
        }
    }

    /// Builds a navigation mesh tile from a tile cache layer
    pub fn build_tile_from_layer(
        &self,
        layer: &TileCacheLayer,
        obstacles: &[Obstacle],
        _tile_cache: &TileCache,
    ) -> Result<MeshTile> {
        // Step 1: Create a compact heightfield from the layer data
        let mut chf = self.layer_to_compact_heightfield(layer)?;

        // Step 2: Apply obstacles to the heightfield
        self.rasterize_obstacles(&mut chf, obstacles, &layer.header)?;

        // Step 3: Build contours from the modified heightfield
        let cset = ContourSet::build_from_compact_heightfield(
            &chf,
            self.recast_config.max_simplification_error,
            self.recast_config.max_edge_len,
            self.recast_config.min_region_area,
            self.recast_config.merge_region_area,
        )?;

        // Step 4: Build polygon mesh from contours
        let pmesh = PolyMesh::build_from_contour_set(
            &cset,
            self.recast_config.max_vertices_per_polygon as usize,
        )?;

        // Step 5: Build detail mesh for height accuracy
        let dmesh = PolyMeshDetail::build_from_poly_mesh(
            &pmesh,
            &chf,
            self.recast_config.detail_sample_dist,
            self.recast_config.detail_sample_max_error,
        )?;

        // Step 6: Convert to navigation mesh tile
        let tile = self.create_mesh_tile(&pmesh, &dmesh, &layer.header)?;

        Ok(tile)
    }

    /// Converts a tile cache layer to a compact heightfield
    fn layer_to_compact_heightfield(&self, layer: &TileCacheLayer) -> Result<CompactHeightfield> {
        let header = &layer.header;

        // Calculate dimensions
        let width = header.width as usize;
        let height = header.height as usize;

        // Create heightfield with proper bounds
        let mut hf = Heightfield::new(
            width as i32,
            height as i32,
            Vec3::new(header.bmin[0], header.bmin[1], header.bmin[2]),
            Vec3::new(header.bmax[0], header.bmax[1], header.bmax[2]),
            self.config.cs,
            self.config.ch,
        );

        // Reconstruct spans from layer data
        // The layer stores compressed heightfield data
        // We need to decode regions, areas, and connections
        self.decode_layer_to_heightfield(&mut hf, layer)?;

        // Convert to compact heightfield
        let chf = CompactHeightfield::build_from_heightfield(&hf)?;

        Ok(chf)
    }

    /// Decodes layer data into heightfield spans
    fn decode_layer_to_heightfield(
        &self,
        hf: &mut Heightfield,
        layer: &TileCacheLayer,
    ) -> Result<()> {
        let width = layer.header.width as usize;
        let height = layer.header.height as usize;

        // Validate data sizes
        if layer.regons.len() != width * height {
            return Err(Error::InvalidMesh("Invalid region data size".to_string()));
        }
        if layer.areas.len() != width * height {
            return Err(Error::InvalidMesh("Invalid area data size".to_string()));
        }

        // Reconstruct spans from layer data
        for y in 0..height {
            for x in 0..width {
                let idx = y * width + x;
                let region = layer.regons[idx];
                let area = layer.areas[idx];

                // Skip empty cells
                if region == 0 {
                    continue;
                }

                // Calculate height from layer header
                let cell_height = layer.header.hmin as i32;

                // Add span to heightfield
                hf.add_span(
                    x as i32,
                    y as i32,
                    cell_height as i16,
                    (cell_height + 1) as i16,
                    area,
                )?;
            }
        }

        Ok(())
    }

    /// Rasterizes obstacles into the compact heightfield
    fn rasterize_obstacles(
        &self,
        chf: &mut CompactHeightfield,
        obstacles: &[Obstacle],
        header: &crate::tile_cache_data::TileCacheLayerHeader,
    ) -> Result<()> {
        for obstacle in obstacles {
            // Skip empty or removing obstacles
            if obstacle.state == ObstacleState::Empty || obstacle.state == ObstacleState::Removing {
                continue;
            }

            // Check if obstacle is within tile bounds
            let tile_min = Vec3::new(header.bmin[0], header.bmin[1], header.bmin[2]);
            let tile_max = Vec3::new(header.bmax[0], header.bmax[1], header.bmax[2]);

            let (obs_min, obs_max) = match &obstacle.data {
                ObstacleData::Cylinder {
                    pos,
                    radius,
                    height,
                } => {
                    let obs_min = Vec3::new(pos[0] - radius, pos[1], pos[2] - radius);
                    let obs_max = Vec3::new(pos[0] + radius, pos[1] + height, pos[2] + radius);
                    (obs_min, obs_max)
                }
                ObstacleData::Box { bmin, bmax } => {
                    let obs_min = Vec3::new(bmin[0], bmin[1], bmin[2]);
                    let obs_max = Vec3::new(bmax[0], bmax[1], bmax[2]);
                    (obs_min, obs_max)
                }
                ObstacleData::OrientedBox {
                    center,
                    half_extents,
                    ..
                } => {
                    // Conservative bounds for oriented box
                    let radius = (half_extents[0].powi(2) + half_extents[2].powi(2)).sqrt();
                    let obs_min = Vec3::new(
                        center[0] - radius,
                        center[1] - half_extents[1],
                        center[2] - radius,
                    );
                    let obs_max = Vec3::new(
                        center[0] + radius,
                        center[1] + half_extents[1],
                        center[2] + radius,
                    );
                    (obs_min, obs_max)
                }
            };

            // Skip if obstacle doesn't overlap tile
            if obs_max.x < tile_min.x
                || obs_min.x > tile_max.x
                || obs_max.z < tile_min.z
                || obs_min.z > tile_max.z
            {
                continue;
            }

            // Rasterize obstacle based on type
            match &obstacle.data {
                ObstacleData::Cylinder { .. } => {
                    self.rasterize_cylinder_obstacle(chf, obstacle)?;
                }
                ObstacleData::Box { .. } => {
                    self.rasterize_box_obstacle(chf, obstacle)?;
                }
                ObstacleData::OrientedBox { .. } => {
                    self.rasterize_oriented_box_obstacle(chf, obstacle)?;
                }
            }
        }

        Ok(())
    }

    /// Rasterizes a cylindrical obstacle into the heightfield
    fn rasterize_cylinder_obstacle(
        &self,
        chf: &mut CompactHeightfield,
        obstacle: &Obstacle,
    ) -> Result<()> {
        let (center, radius, height) = match &obstacle.data {
            ObstacleData::Cylinder {
                pos,
                radius,
                height,
            } => (Vec3::new(pos[0], pos[1], pos[2]), *radius, *height),
            _ => return Ok(()), // Skip non-cylinder obstacles
        };

        // Convert world coordinates to cell coordinates
        let min_x = ((center.x - radius - chf.bmin.x) / self.config.cs).floor() as i32;
        let max_x = ((center.x + radius - chf.bmin.x) / self.config.cs).ceil() as i32;
        let min_z = ((center.z - radius - chf.bmin.z) / self.config.cs).floor() as i32;
        let max_z = ((center.z + radius - chf.bmin.z) / self.config.cs).ceil() as i32;

        // Clamp to heightfield bounds
        let min_x = min_x.max(0);
        let max_x = max_x.min(chf.width - 1);
        let min_z = min_z.max(0);
        let max_z = max_z.min(chf.height - 1);

        // Mark cells within the cylinder as unwalkable
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                // Calculate cell center
                let cell_x = chf.bmin.x + (x as f32 + 0.5) * self.config.cs;
                let cell_z = chf.bmin.z + (z as f32 + 0.5) * self.config.cs;

                // Check if cell center is within cylinder radius
                let dx = cell_x - center.x;
                let dz = cell_z - center.z;
                let dist_sq = dx * dx + dz * dz;

                if dist_sq <= radius * radius {
                    // Mark all spans in this column as unwalkable if they overlap the obstacle height
                    let cell_idx = (z * chf.width + x) as usize;
                    if let Some(cell) = chf.cells.get_mut(cell_idx) {
                        if let Some(first_span_idx) = cell.index {
                            let span_idx = first_span_idx;
                            let span_count = cell.count;

                            for s in 0..span_count {
                                let current_span_idx = span_idx + s;
                                if let Some(span) = chf.spans.get_mut(current_span_idx) {
                                    // Check if span overlaps obstacle height
                                    let span_min_y = chf.bmin.y + span.y as f32 * self.config.ch;
                                    let span_max_y = span_min_y + self.config.ch;

                                    if span_min_y < center.y + height && span_max_y > center.y {
                                        // Mark span as non-walkable
                                        span.area = 0; // RC_NULL_AREA
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Creates a navigation mesh tile from polygon and detail meshes
    fn create_mesh_tile(
        &self,
        pmesh: &PolyMesh,
        dmesh: &PolyMeshDetail,
        header: &crate::tile_cache_data::TileCacheLayerHeader,
    ) -> Result<MeshTile> {
        // Create tile header
        let mut tile_header = TileHeader::new(header.tx, header.ty, header.tlayer);
        tile_header.vert_count = pmesh.vert_count as i32;
        tile_header.poly_count = pmesh.poly_count as i32;
        tile_header.detail_mesh_count = dmesh.poly_count as i32;
        tile_header.detail_vert_count = dmesh.vert_count as i32;
        tile_header.detail_tri_count = dmesh.tri_count as i32;

        // Copy vertices (convert from i32 to f32)
        let mut verts = Vec::with_capacity(pmesh.vert_count * 3);
        for i in 0..pmesh.vert_count {
            verts.push(pmesh.vertices[i * 3] as f32);
            verts.push(pmesh.vertices[i * 3 + 1] as f32);
            verts.push(pmesh.vertices[i * 3 + 2] as f32);
        }

        // Convert polygons
        let mut polys = Vec::with_capacity(pmesh.poly_count);
        for i in 0..pmesh.poly_count {
            // Extract vertices from the polygon mesh data
            let mut vert_count = 0;
            let mut verts = [0u16; MAX_VERTS_PER_POLY];
            let mut neighbors = [0u16; MAX_VERTS_PER_POLY];

            // Find actual vertex count by looking at polygon data
            for j in 0..pmesh.max_verts_per_poly {
                let poly_idx = i * pmesh.max_verts_per_poly + j;
                if poly_idx < pmesh.polys.len() {
                    let v = pmesh.polys[poly_idx];
                    if v != MESH_NULL_IDX {
                        verts[vert_count] = v;
                        neighbors[vert_count] = 0; // No neighbor data in PolyMesh
                        vert_count += 1;
                    }
                }
            }

            let poly = Poly {
                first_link: None,
                verts,
                neighbors,
                flags: PolyFlags::WALK,
                vert_count: vert_count as u8,
                area: pmesh.areas[i],
                poly_type: PolyType::Ground,
            };

            polys.push(poly);
        }

        // Convert detail meshes
        let mut detail_meshes = Vec::with_capacity(dmesh.poly_count);
        for i in 0..dmesh.poly_count {
            detail_meshes.push(PolyDetail {
                vert_base: dmesh.poly_start[i] as u32,
                tri_base: dmesh.poly_start[i] as u32,
                vert_count: 0, // Will be calculated from triangles
                tri_count: dmesh.poly_tri_count[i] as u8,
            });
        }

        // Copy detail vertices
        let detail_verts = dmesh.vertices.clone();

        // Copy detail triangles (convert from u32 to u8)
        let mut detail_tris = Vec::with_capacity(dmesh.tri_count * 3);
        for i in 0..dmesh.tri_count {
            detail_tris.push(dmesh.triangles[i * 3] as u8);
            detail_tris.push(dmesh.triangles[i * 3 + 1] as u8);
            detail_tris.push(dmesh.triangles[i * 3 + 2] as u8);
        }

        // Create the mesh tile
        let tile = MeshTile {
            salt: 1, // Will be set by NavMesh when added
            header: Some(tile_header),
            polys,
            verts,
            links: Vec::new(),
            detail_meshes,
            detail_verts,
            detail_tris,
            bvh_root: None, // Will be built when added to NavMesh
            bvh_nodes: Vec::new(),
            off_mesh_connections: Vec::new(),
            flags: 0,
            next: None,
        };

        Ok(tile)
    }

    /// Rasterizes an axis-aligned box obstacle into the heightfield
    fn rasterize_box_obstacle(
        &self,
        chf: &mut CompactHeightfield,
        obstacle: &Obstacle,
    ) -> Result<()> {
        let (bmin, bmax) = match &obstacle.data {
            ObstacleData::Box { bmin, bmax } => (*bmin, *bmax),
            _ => return Ok(()), // Skip non-box obstacles
        };

        // Convert world coordinates to cell coordinates
        let min_x = ((bmin[0] - chf.bmin.x) / self.config.cs).floor() as i32;
        let max_x = ((bmax[0] - chf.bmin.x) / self.config.cs).ceil() as i32;
        let min_z = ((bmin[2] - chf.bmin.z) / self.config.cs).floor() as i32;
        let max_z = ((bmax[2] - chf.bmin.z) / self.config.cs).ceil() as i32;

        // Clamp to heightfield bounds
        let min_x = min_x.max(0);
        let max_x = max_x.min(chf.width - 1);
        let min_z = min_z.max(0);
        let max_z = max_z.min(chf.height - 1);

        // Rasterize the box
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                let cell_idx = (x + z * chf.width) as usize;
                let cell = &mut chf.cells[cell_idx];

                // Mark all spans within the box height range
                if let Some(cell_index) = cell.index {
                    for i in cell_index..(cell_index + cell.count) {
                        let span = &mut chf.spans[i];
                        let span_y_min = span.min as f32 * self.config.ch + chf.bmin.y;
                        let span_y_max = span.max as f32 * self.config.ch + chf.bmin.y;

                        // Check if span overlaps with box height
                        if span_y_max > bmin[1] && span_y_min < bmax[1] {
                            span.area = 0; // Mark as unwalkable
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Rasterizes an oriented box obstacle into the heightfield
    fn rasterize_oriented_box_obstacle(
        &self,
        chf: &mut CompactHeightfield,
        obstacle: &Obstacle,
    ) -> Result<()> {
        let (center, half_extents, rot_aux) = match &obstacle.data {
            ObstacleData::OrientedBox {
                center,
                half_extents,
                rot_aux,
            } => (*center, *half_extents, *rot_aux),
            _ => return Ok(()), // Skip non-oriented-box obstacles
        };

        // Reconstruct rotation from auxiliary values
        // rot_aux[0] = cos(0.5*angle)*sin(-0.5*angle)
        // rot_aux[1] = cos(0.5*angle)*cos(0.5*angle) - 0.5
        let cos_half_cos_half = rot_aux[1] + 0.5;
        let cos_half_sin_half = rot_aux[0];

        // Conservative bounding box for the oriented box
        let radius = (half_extents[0].powi(2) + half_extents[2].powi(2)).sqrt();

        // Convert world coordinates to cell coordinates
        let min_x = ((center[0] - radius - chf.bmin.x) / self.config.cs).floor() as i32;
        let max_x = ((center[0] + radius - chf.bmin.x) / self.config.cs).ceil() as i32;
        let min_z = ((center[2] - radius - chf.bmin.z) / self.config.cs).floor() as i32;
        let max_z = ((center[2] + radius - chf.bmin.z) / self.config.cs).ceil() as i32;

        // Clamp to heightfield bounds
        let min_x = min_x.max(0);
        let max_x = max_x.min(chf.width - 1);
        let min_z = min_z.max(0);
        let max_z = max_z.min(chf.height - 1);

        // Rasterize cells that are inside the oriented box
        for z in min_z..=max_z {
            for x in min_x..=max_x {
                // Get cell center in world space
                let cell_x = x as f32 * self.config.cs + chf.bmin.x + self.config.cs * 0.5;
                let cell_z = z as f32 * self.config.cs + chf.bmin.z + self.config.cs * 0.5;

                // Transform to box-local coordinates
                let dx = cell_x - center[0];
                let dz = cell_z - center[2];

                // Apply inverse rotation (using 2D rotation matrix)
                // For Y-axis rotation: cos(θ) = 2*cos²(θ/2) - 1
                let cos_angle = 2.0 * cos_half_cos_half - 1.0;
                let sin_angle = 2.0 * cos_half_sin_half;

                let local_x = cos_angle * dx + sin_angle * dz;
                let local_z = -sin_angle * dx + cos_angle * dz;

                // Check if point is inside the box
                if local_x.abs() <= half_extents[0] && local_z.abs() <= half_extents[2] {
                    let cell_idx = (x + z * chf.width) as usize;
                    let cell = &mut chf.cells[cell_idx];

                    // Mark all spans within the box height range
                    if let Some(cell_index) = cell.index {
                        for i in cell_index..(cell_index + cell.count) {
                            let span = &mut chf.spans[i];
                            let span_y_min = span.min as f32 * self.config.ch + chf.bmin.y;
                            let span_y_max = span.max as f32 * self.config.ch + chf.bmin.y;

                            // Check if span overlaps with box height
                            if span_y_max > center[1] - half_extents[1]
                                && span_y_min < center[1] + half_extents[1]
                            {
                                span.area = 0; // Mark as unwalkable
                            }
                        }
                    }
                }
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tile_cache_builder_creation() {
        let config = TileCacheBuilderConfig::default();
        let builder = TileCacheBuilder::new(config);

        assert_eq!(builder.recast_config.cs, 0.3);
        assert_eq!(builder.recast_config.ch, 0.2);
        assert_eq!(builder.recast_config.walkable_height, 20);
    }
}
