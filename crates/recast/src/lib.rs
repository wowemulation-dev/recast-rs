//! Recast component for navigation mesh generation
//!
//! Recast is a navigation mesh generator. It takes input triangle meshes and creates navigation
//! meshes suitable for pathfinding in games and simulations.

mod area;
mod compact_heightfield;
mod config;
mod context;
mod contour;
mod convex_volume;
mod detail_mesh;
mod distance_field;
mod heightfield;
mod heightfield_layers;
mod mesh_merging;
mod polymesh;
mod rasterization;
mod triangle_utils;
mod watershed;

pub use area::{
    erode_walkable_area, mark_box_area, mark_convex_poly_area, mark_cylinder_area,
    median_filter_walkable_area, offset_poly,
};
pub use compact_heightfield::{CompactCell, CompactConnection, CompactHeightfield, CompactSpan};
pub use config::RecastConfig;
pub use context::{LogEntry, LogLevel, ProgressInfo, RecastContext, TimerCategory, TimerGuard};
pub use contour::{BuildContoursFlags, Contour, ContourSet, ContourVertex};
pub use convex_volume::{ConvexVolume, ConvexVolumeSet, MAX_CONVEX_VOLUME_VERTS};
pub use detail_mesh::{merge_poly_mesh_details, PolyMeshDetail};
pub use distance_field::{build_layer_regions, build_regions_monotone, build_regions_watershed};
// Region building functions are not exported directly - use CompactHeightfield methods instead
pub use heightfield::{Heightfield, Span};
pub use heightfield_layers::{HeightfieldLayer, LayerConnection, LayeredHeightfield};
pub use mesh_merging::{copy_poly_mesh, MergeResult, MergeStats, MeshMergeConfig, MeshMerger};
pub use polymesh::{PolyMesh, MESH_NULL_IDX};
pub use rasterization::{
    add_span, rasterize_triangle, rasterize_triangle_soup, rasterize_triangles,
    rasterize_triangles_u16,
};
pub use triangle_utils::{
    array_to_vec3, calc_bounds, calc_bounds_vec3, calc_grid_size, clear_unwalkable_triangles,
    get_dir_for_offset, get_dir_offset_x, get_dir_offset_y, get_heightfield_span_count,
    mark_walkable_triangles, vadd, vcopy, vcross, vdist, vdist_sqr, vdot, vec3_to_array, vmad,
    vmax, vmin, vnormalize, vsub, RC_NULL_AREA, RC_WALKABLE_AREA,
};

/// Builder for Recast navigation mesh generation
#[derive(Debug)]
pub struct RecastBuilder {
    /// Configuration for the navigation mesh generation
    config: RecastConfig,
}

impl RecastBuilder {
    /// Creates a new RecastBuilder with the specified configuration
    pub fn new(config: RecastConfig) -> Self {
        Self { config }
    }

    /// Gets a reference to the configuration
    pub fn config(&self) -> &RecastConfig {
        &self.config
    }

    /// Builds a navigation mesh from the input mesh
    pub fn build_mesh(
        &self,
        vertices: &[f32],
        indices: &[i32],
    ) -> recast_common::Result<(PolyMesh, PolyMeshDetail)> {
        // Validate the configuration
        self.config.validate()?;

        // Build the heightfield
        let heightfield = self.build_heightfield(vertices, indices)?;

        // Build the compact heightfield
        let mut compact_heightfield = CompactHeightfield::build_from_heightfield(&heightfield)?;

        // Build regions using watershed partitioning
        compact_heightfield.build_regions(
            self.config.border_size,
            self.config.min_region_area,
            self.config.merge_region_area,
        )?;

        // Build the contour set
        let contour_set = ContourSet::build_from_compact_heightfield(
            &compact_heightfield,
            self.config.max_simplification_error,
            self.config.max_edge_len,
            self.config.min_region_area,
            self.config.merge_region_area,
        )?;

        // Build the polygon mesh
        let poly_mesh = PolyMesh::build_from_contour_set(
            &contour_set,
            self.config.max_vertices_per_polygon as usize,
        )?;

        // Build the detailed polygon mesh
        let poly_mesh_detail = PolyMeshDetail::build_from_poly_mesh(
            &poly_mesh,
            &compact_heightfield,
            self.config.detail_sample_dist,
            self.config.detail_sample_max_error,
        )?;

        Ok((poly_mesh, poly_mesh_detail))
    }

    /// Builds a heightfield from the input mesh
    pub fn build_heightfield(
        &self,
        vertices: &[f32],
        indices: &[i32],
    ) -> recast_common::Result<Heightfield> {
        // Create a new heightfield
        let mut heightfield = Heightfield::new(
            self.config.width,
            self.config.height,
            self.config.bmin,
            self.config.bmax,
            self.config.cs,
            self.config.ch,
        );

        // Rasterize the triangles into the heightfield
        self.rasterize_triangles(&mut heightfield, vertices, indices)?;

        Ok(heightfield)
    }

    /// Builds a layered heightfield from the input mesh for multi-story support
    pub fn build_layered_heightfield(
        &self,
        vertices: &[f32],
        indices: &[i32],
        layer_height_threshold: Option<i16>,
        walkable_height: Option<i16>,
    ) -> recast_common::Result<LayeredHeightfield> {
        // First build a regular heightfield
        let heightfield = self.build_heightfield(vertices, indices)?;

        // Use sensible defaults if not provided
        let layer_threshold =
            layer_height_threshold.unwrap_or(self.config.walkable_height as i16 * 3);
        let walkable_ht = walkable_height.unwrap_or(self.config.walkable_height as i16);

        // Convert to layered heightfield
        LayeredHeightfield::build_from_heightfield(&heightfield, layer_threshold, walkable_ht)
    }

    /// Merges multiple navigation meshes into a single unified mesh
    pub fn merge_navigation_meshes(
        poly_meshes: &[&PolyMesh],
        config: Option<MeshMergeConfig>,
    ) -> recast_common::Result<MergeResult> {
        let merge_config = config.unwrap_or_default();
        let merger = MeshMerger::new(merge_config);
        merger.merge_meshes(poly_meshes)
    }

    /// Builds navigation meshes from multiple input geometries and merges them
    pub fn build_and_merge_from_multiple_inputs(
        &self,
        inputs: &[(&[f32], &[i32])], // (vertices, indices) pairs
        merge_config: Option<MeshMergeConfig>,
    ) -> recast_common::Result<MergeResult> {
        // Build individual meshes
        let mut meshes = Vec::new();
        for (vertices, indices) in inputs {
            let (poly_mesh, _detail_mesh) = self.build_mesh(vertices, indices)?;
            meshes.push(poly_mesh);
        }

        // Merge the meshes
        let mesh_refs: Vec<&PolyMesh> = meshes.iter().collect();
        Self::merge_navigation_meshes(&mesh_refs, merge_config)
    }

    /// Rasterizes triangles into the heightfield
    pub fn rasterize_triangles(
        &self,
        heightfield: &mut Heightfield,
        vertices: &[f32],
        indices: &[i32],
    ) -> recast_common::Result<()> {
        use glam::Vec3;

        // Validate inputs
        if vertices.is_empty() || indices.is_empty() {
            return Ok(());
        }

        if vertices.len() % 3 != 0 {
            return Err(recast_common::Error::NavMeshGeneration(
                "Vertex array length must be a multiple of 3".to_string(),
            ));
        }

        if indices.len() % 3 != 0 {
            return Err(recast_common::Error::NavMeshGeneration(
                "Index array length must be a multiple of 3".to_string(),
            ));
        }

        // Calculate walkable slope threshold
        let walkable_slope_threshold =
            (self.config.walkable_slope_angle * std::f32::consts::PI / 180.0).cos();

        // Process each triangle
        let num_triangles = indices.len() / 3;
        for i in 0..num_triangles {
            // Get triangle indices
            let idx0 = indices[i * 3] as usize;
            let idx1 = indices[i * 3 + 1] as usize;
            let idx2 = indices[i * 3 + 2] as usize;

            // Validate indices
            let max_idx = vertices.len() / 3;
            if idx0 >= max_idx || idx1 >= max_idx || idx2 >= max_idx {
                return Err(recast_common::Error::NavMeshGeneration(format!(
                    "Triangle index out of bounds: {}, {}, {} (max: {})",
                    idx0,
                    idx1,
                    idx2,
                    max_idx - 1
                )));
            }

            // Get triangle vertices
            let v0 = Vec3::new(
                vertices[idx0 * 3],
                vertices[idx0 * 3 + 1],
                vertices[idx0 * 3 + 2],
            );
            let v1 = Vec3::new(
                vertices[idx1 * 3],
                vertices[idx1 * 3 + 1],
                vertices[idx1 * 3 + 2],
            );
            let v2 = Vec3::new(
                vertices[idx2 * 3],
                vertices[idx2 * 3 + 1],
                vertices[idx2 * 3 + 2],
            );

            // Calculate triangle normal to determine walkability
            let e1 = v1 - v0;
            let e2 = v2 - v0;
            let cross = e1.cross(e2);

            // Skip degenerate triangles
            if cross.length() < f32::EPSILON {
                continue;
            }

            let normal = cross.normalize();

            // Check if triangle is walkable based on slope
            // Use absolute value of Y component to handle both winding orders
            let area = if normal.y.abs() >= walkable_slope_threshold {
                1
            } else {
                0
            };

            // Rasterize the triangle with the calculated area
            rasterize_triangle(&v0, &v1, &v2, area, heightfield, 1)?;
        }

        // Apply filtering after all triangles are rasterized
        // First filter low hanging obstacles that agents can step over
        heightfield.filter_low_hanging_walkable_obstacles(self.config.walkable_climb as i16)?;

        // Filter ledge spans that are too steep or dangerous
        heightfield.filter_ledge_spans(
            self.config.walkable_height as i16,
            self.config.walkable_climb as i16,
        )?;

        // Filter spans with insufficient clearance above them
        heightfield.filter_walkable_low_height_spans(self.config.walkable_height as i16)?;

        Ok(())
    }

    /// Builds a navigation mesh with convex volumes applied
    pub fn build_mesh_with_volumes(
        &self,
        vertices: &[f32],
        indices: &[i32],
        convex_volumes: &ConvexVolumeSet,
    ) -> recast_common::Result<(PolyMesh, PolyMeshDetail)> {
        // Validate the configuration
        self.config.validate()?;

        // Build the heightfield
        let mut heightfield = self.build_heightfield(vertices, indices)?;

        // Apply convex volumes to modify area types
        convex_volumes.apply_to_heightfield(&mut heightfield)?;

        // Build the compact heightfield
        let mut compact_heightfield = CompactHeightfield::build_from_heightfield(&heightfield)?;

        // Build regions using watershed partitioning
        compact_heightfield.build_regions(
            self.config.border_size,
            self.config.min_region_area,
            self.config.merge_region_area,
        )?;

        // Build the contour set
        let contour_set = ContourSet::build_from_compact_heightfield(
            &compact_heightfield,
            self.config.max_simplification_error,
            self.config.max_edge_len,
            self.config.min_region_area,
            self.config.merge_region_area,
        )?;

        // Build the polygon mesh
        let poly_mesh = PolyMesh::build_from_contour_set(
            &contour_set,
            self.config.max_vertices_per_polygon as usize,
        )?;

        // Build the detailed polygon mesh
        let poly_mesh_detail = PolyMeshDetail::build_from_poly_mesh(
            &poly_mesh,
            &compact_heightfield,
            self.config.detail_sample_dist,
            self.config.detail_sample_max_error,
        )?;

        Ok((poly_mesh, poly_mesh_detail))
    }

    /// Builds a heightfield with convex volumes applied
    pub fn build_heightfield_with_volumes(
        &self,
        vertices: &[f32],
        indices: &[i32],
        convex_volumes: &ConvexVolumeSet,
    ) -> recast_common::Result<Heightfield> {
        // Build the heightfield
        let mut heightfield = self.build_heightfield(vertices, indices)?;

        // Apply convex volumes to modify area types
        convex_volumes.apply_to_heightfield(&mut heightfield)?;

        Ok(heightfield)
    }
}
