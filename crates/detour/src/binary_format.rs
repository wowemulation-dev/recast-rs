//! Binary format compatibility for C++ 
//!
//! This module provides functions to read and write navigation meshes in the
//! binary format used by the original C++  library.

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::{Cursor, Read, Write};

use super::nav_mesh::{BVNode, MeshTile, OffMeshConnection, Poly, PolyDetail, TileHeader};
use super::Status;
use super::MAX_VERTS_PER_POLY;
use super::{NavMesh, NavMeshParams, PolyFlags, PolyRef, PolyType};
use recast_common::{Error, Result};

/// Magic number for navigation mesh files ('DNAV')
pub const DT_NAVMESH_MAGIC: u32 = 0x5641_4E44; // 'DNAV' in little-endian

/// Current navigation mesh version
pub const DT_NAVMESH_VERSION: u32 = 7;

/// Magic number for navigation mesh state files ('DNMS')
const DT_NAVMESH_STATE_MAGIC: u32 = 0x534D_4E44; // 'DNMS' in little-endian

/// Current navigation mesh state version
const DT_NAVMESH_STATE_VERSION: u32 = 1;

/// Maximum vertices per polygon (must match C++ version)
const DT_VERTS_PER_POLYGON: usize = 6;

/// Null link value
const DT_NULL_LINK: u32 = 0xffffffff;

/// External link flag
/// TODO: Used for marking external polygon connections in tile data
#[allow(dead_code)]
const DT_EXT_LINK: u16 = 0x8000;

/// Align value to 4-byte boundary
fn align4(value: usize) -> usize {
    (value + 3) & !3
}

/// Tile state header structure matching C++ dtTileState
#[repr(C)]
#[derive(Debug, Clone)]
struct TileState {
    magic: u32,
    version: u32,
    tile_ref: PolyRef,
}

/// Polygon state structure matching C++ dtPolyState
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct PolyState {
    flags: u16,
    area: u8,
}

impl TileState {
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer.write_u32::<LittleEndian>(self.magic)?;
        writer.write_u32::<LittleEndian>(self.version)?;
        writer.write_u32::<LittleEndian>(self.tile_ref.id())?;
        Ok(())
    }

    fn read_from<R: Read>(reader: &mut R) -> Result<Self> {
        Ok(Self {
            magic: reader.read_u32::<LittleEndian>()?,
            version: reader.read_u32::<LittleEndian>()?,
            tile_ref: PolyRef::new(reader.read_u32::<LittleEndian>()?),
        })
    }
}

impl PolyState {
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer.write_u16::<LittleEndian>(self.flags)?;
        writer.write_u8(self.area)?;
        Ok(())
    }

    fn read_from<R: Read>(reader: &mut R) -> Result<Self> {
        Ok(Self {
            flags: reader.read_u16::<LittleEndian>()?,
            area: reader.read_u8()?,
        })
    }
}

/// Mesh header structure matching C++ dtMeshHeader
#[repr(C)]
#[derive(Debug, Clone)]
struct MeshHeader {
    magic: u32,
    version: u32,
    x: i32,
    y: i32,
    layer: i32,
    user_id: u32,
    poly_count: i32,
    vert_count: i32,
    max_link_count: i32,
    detail_mesh_count: i32,
    detail_vert_count: i32,
    detail_tri_count: i32,
    bv_node_count: i32,
    off_mesh_con_count: i32,
    off_mesh_base: i32,
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    bmin: [f32; 3],
    bmax: [f32; 3],
    bv_quant_factor: f32,
}

impl MeshHeader {
    /// Reads a mesh header from a reader
    fn read_from<R: Read>(reader: &mut R) -> Result<Self> {
        Ok(Self {
            magic: reader.read_u32::<LittleEndian>()?,
            version: reader.read_u32::<LittleEndian>()?,
            x: reader.read_i32::<LittleEndian>()?,
            y: reader.read_i32::<LittleEndian>()?,
            layer: reader.read_i32::<LittleEndian>()?,
            user_id: reader.read_u32::<LittleEndian>()?,
            poly_count: reader.read_i32::<LittleEndian>()?,
            vert_count: reader.read_i32::<LittleEndian>()?,
            max_link_count: reader.read_i32::<LittleEndian>()?,
            detail_mesh_count: reader.read_i32::<LittleEndian>()?,
            detail_vert_count: reader.read_i32::<LittleEndian>()?,
            detail_tri_count: reader.read_i32::<LittleEndian>()?,
            bv_node_count: reader.read_i32::<LittleEndian>()?,
            off_mesh_con_count: reader.read_i32::<LittleEndian>()?,
            off_mesh_base: reader.read_i32::<LittleEndian>()?,
            walkable_height: reader.read_f32::<LittleEndian>()?,
            walkable_radius: reader.read_f32::<LittleEndian>()?,
            walkable_climb: reader.read_f32::<LittleEndian>()?,
            bmin: [
                reader.read_f32::<LittleEndian>()?,
                reader.read_f32::<LittleEndian>()?,
                reader.read_f32::<LittleEndian>()?,
            ],
            bmax: [
                reader.read_f32::<LittleEndian>()?,
                reader.read_f32::<LittleEndian>()?,
                reader.read_f32::<LittleEndian>()?,
            ],
            bv_quant_factor: reader.read_f32::<LittleEndian>()?,
        })
    }

    /// Writes a mesh header to a writer
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer.write_u32::<LittleEndian>(self.magic)?;
        writer.write_u32::<LittleEndian>(self.version)?;
        writer.write_i32::<LittleEndian>(self.x)?;
        writer.write_i32::<LittleEndian>(self.y)?;
        writer.write_i32::<LittleEndian>(self.layer)?;
        writer.write_u32::<LittleEndian>(self.user_id)?;
        writer.write_i32::<LittleEndian>(self.poly_count)?;
        writer.write_i32::<LittleEndian>(self.vert_count)?;
        writer.write_i32::<LittleEndian>(self.max_link_count)?;
        writer.write_i32::<LittleEndian>(self.detail_mesh_count)?;
        writer.write_i32::<LittleEndian>(self.detail_vert_count)?;
        writer.write_i32::<LittleEndian>(self.detail_tri_count)?;
        writer.write_i32::<LittleEndian>(self.bv_node_count)?;
        writer.write_i32::<LittleEndian>(self.off_mesh_con_count)?;
        writer.write_i32::<LittleEndian>(self.off_mesh_base)?;
        writer.write_f32::<LittleEndian>(self.walkable_height)?;
        writer.write_f32::<LittleEndian>(self.walkable_radius)?;
        writer.write_f32::<LittleEndian>(self.walkable_climb)?;
        writer.write_f32::<LittleEndian>(self.bmin[0])?;
        writer.write_f32::<LittleEndian>(self.bmin[1])?;
        writer.write_f32::<LittleEndian>(self.bmin[2])?;
        writer.write_f32::<LittleEndian>(self.bmax[0])?;
        writer.write_f32::<LittleEndian>(self.bmax[1])?;
        writer.write_f32::<LittleEndian>(self.bmax[2])?;
        writer.write_f32::<LittleEndian>(self.bv_quant_factor)?;
        Ok(())
    }
}

/// Polygon structure matching C++ dtPoly
#[repr(C)]
#[derive(Debug, Clone)]
struct PolyData {
    first_link: u32,
    verts: [u16; DT_VERTS_PER_POLYGON],
    neis: [u16; DT_VERTS_PER_POLYGON],
    flags: u16,
    vert_count: u8,
    area_and_type: u8,
}

impl PolyData {
    /// Reads polygon data from a reader
    fn read_from<R: Read>(reader: &mut R) -> Result<Self> {
        let first_link = reader.read_u32::<LittleEndian>()?;

        let mut verts = [0u16; DT_VERTS_PER_POLYGON];
        for v in &mut verts {
            *v = reader.read_u16::<LittleEndian>()?;
        }

        let mut neis = [0u16; DT_VERTS_PER_POLYGON];
        for n in &mut neis {
            *n = reader.read_u16::<LittleEndian>()?;
        }

        let flags = reader.read_u16::<LittleEndian>()?;
        let vert_count = reader.read_u8()?;
        let area_and_type = reader.read_u8()?;

        Ok(Self {
            first_link,
            verts,
            neis,
            flags,
            vert_count,
            area_and_type,
        })
    }

    /// Writes polygon data to a writer
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer.write_u32::<LittleEndian>(self.first_link)?;

        for &v in &self.verts {
            writer.write_u16::<LittleEndian>(v)?;
        }

        for &n in &self.neis {
            writer.write_u16::<LittleEndian>(n)?;
        }

        writer.write_u16::<LittleEndian>(self.flags)?;
        writer.write_u8(self.vert_count)?;
        writer.write_u8(self.area_and_type)?;

        Ok(())
    }

    /// Converts to internal Poly structure
    fn to_poly(&self) -> Poly {
        // Decode area and type
        let area = self.area_and_type & 0x3F;
        let poly_type = if (self.area_and_type >> 6) == 1 {
            PolyType::OffMeshConnection
        } else {
            PolyType::Ground
        };
        let flags = PolyFlags::from_bits_truncate(self.flags);

        let mut poly = Poly::new(area, poly_type, flags);
        poly.first_link = if self.first_link == DT_NULL_LINK {
            None
        } else {
            Some(self.first_link as usize)
        };

        // Copy vertices
        for i in 0..self.vert_count as usize {
            poly.verts[i] = self.verts[i];
        }
        poly.vert_count = self.vert_count;

        // Copy neighbors
        for i in 0..DT_VERTS_PER_POLYGON {
            poly.neighbors[i] = self.neis[i];
        }

        poly
    }

    /// Creates from internal Poly structure
    fn from_poly(poly: &Poly) -> Self {
        let mut data = Self {
            first_link: poly.first_link.map(|l| l as u32).unwrap_or(DT_NULL_LINK),
            verts: [0; DT_VERTS_PER_POLYGON],
            neis: [0; DT_VERTS_PER_POLYGON],
            flags: poly.flags.bits(),
            vert_count: poly.vert_count,
            area_and_type: poly.area | ((poly.poly_type as u8) << 6),
        };

        // Copy vertices
        for i in 0..DT_VERTS_PER_POLYGON.min(MAX_VERTS_PER_POLY) {
            data.verts[i] = poly.verts[i];
            data.neis[i] = poly.neighbors[i];
        }

        data
    }
}

/// Bounding volume node matching C++ dtBVNode
#[repr(C)]
#[derive(Debug, Clone)]
struct BVNodeData {
    bmin: [u16; 3],
    bmax: [u16; 3],
    i: i32,
}

impl BVNodeData {
    /// Reads BV node data from a reader
    fn read_from<R: Read>(reader: &mut R) -> Result<Self> {
        Ok(Self {
            bmin: [
                reader.read_u16::<LittleEndian>()?,
                reader.read_u16::<LittleEndian>()?,
                reader.read_u16::<LittleEndian>()?,
            ],
            bmax: [
                reader.read_u16::<LittleEndian>()?,
                reader.read_u16::<LittleEndian>()?,
                reader.read_u16::<LittleEndian>()?,
            ],
            i: reader.read_i32::<LittleEndian>()?,
        })
    }

    /// Writes BV node data to a writer
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer.write_u16::<LittleEndian>(self.bmin[0])?;
        writer.write_u16::<LittleEndian>(self.bmin[1])?;
        writer.write_u16::<LittleEndian>(self.bmin[2])?;
        writer.write_u16::<LittleEndian>(self.bmax[0])?;
        writer.write_u16::<LittleEndian>(self.bmax[1])?;
        writer.write_u16::<LittleEndian>(self.bmax[2])?;
        writer.write_i32::<LittleEndian>(self.i)?;
        Ok(())
    }
}

/// Off-mesh connection matching C++ dtOffMeshConnection
#[repr(C)]
#[derive(Debug, Clone)]
struct OffMeshConnectionData {
    pos: [f32; 6],
    rad: f32,
    poly: u16,
    flags: u8,
    side: u8,
    area: u8,
    user_id: u32,
}

impl OffMeshConnectionData {
    /// Reads off-mesh connection data from a reader
    fn read_from<R: Read>(reader: &mut R) -> Result<Self> {
        let mut pos = [0.0; 6];
        for p in &mut pos {
            *p = reader.read_f32::<LittleEndian>()?;
        }

        Ok(Self {
            pos,
            rad: reader.read_f32::<LittleEndian>()?,
            poly: reader.read_u16::<LittleEndian>()?,
            flags: reader.read_u8()?,
            side: reader.read_u8()?,
            area: reader.read_u8()?,
            user_id: reader.read_u32::<LittleEndian>()?,
        })
    }

    /// Writes off-mesh connection data to a writer
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<()> {
        for &p in &self.pos {
            writer.write_f32::<LittleEndian>(p)?;
        }
        writer.write_f32::<LittleEndian>(self.rad)?;
        writer.write_u16::<LittleEndian>(self.poly)?;
        writer.write_u8(self.flags)?;
        writer.write_u8(self.side)?;
        writer.write_u8(self.area)?;
        writer.write_u32::<LittleEndian>(self.user_id)?;
        Ok(())
    }
}

/// Loads a navigation mesh tile from C++ binary format
pub fn load_tile_from_binary(data: &[u8]) -> Result<MeshTile> {
    let mut cursor = Cursor::new(data);

    // Read header
    let header = MeshHeader::read_from(&mut cursor)?;

    // Validate magic and version
    if header.magic != DT_NAVMESH_MAGIC {
        return Err(Error::Detour(Status::WrongMagic.to_string()));
    }
    if header.version != DT_NAVMESH_VERSION {
        return Err(Error::Detour(Status::WrongVersion.to_string()));
    }

    // Create tile
    let mut tile = MeshTile::new();

    // Set tile header
    tile.header = Some(TileHeader {
        x: header.x,
        y: header.y,
        layer: header.layer,
        user_id: header.user_id,
        data_size: data.len(),
        bmin: header.bmin,
        bmax: header.bmax,
        poly_count: header.poly_count,
        vert_count: header.vert_count,
        max_links: header.max_link_count,
        detail_mesh_count: header.detail_mesh_count,
        detail_vert_count: header.detail_vert_count,
        detail_tri_count: header.detail_tri_count,
        bvh_node_count: header.bv_node_count,
        off_mesh_connection_count: header.off_mesh_con_count,
        bv_quant_factor: header.bv_quant_factor,
    });

    // Read vertices
    tile.verts.reserve((header.vert_count * 3) as usize);
    for _ in 0..header.vert_count * 3 {
        tile.verts.push(cursor.read_f32::<LittleEndian>()?);
    }

    // Read polygons
    tile.polys.reserve(header.poly_count as usize);
    for _ in 0..header.poly_count {
        let poly_data = PolyData::read_from(&mut cursor)?;
        tile.polys.push(poly_data.to_poly());
    }

    // Skip link data (will be rebuilt)
    // Each link is: u32(ref) + u32(next) + u8(edge) + u8(side) + u8(bmin) + u8(bmax) = 12 bytes
    let link_size = 4 + 4 + 1 + 1 + 1 + 1; // 12 bytes per link
    cursor.set_position(cursor.position() + (header.max_link_count as u64 * link_size as u64));

    // Read detail meshes
    if header.detail_mesh_count > 0 {
        tile.detail_meshes
            .reserve(header.detail_mesh_count as usize);
        for _ in 0..header.detail_mesh_count {
            let detail = PolyDetail {
                vert_base: cursor.read_u32::<LittleEndian>()?,
                tri_base: cursor.read_u32::<LittleEndian>()?,
                vert_count: cursor.read_u8()?,
                tri_count: cursor.read_u8()?,
            };
            // Read 2 padding bytes for C++ struct alignment
            cursor.read_u8()?;
            cursor.read_u8()?;
            tile.detail_meshes.push(detail);
        }

        // Read detail vertices
        tile.detail_verts
            .reserve((header.detail_vert_count * 3) as usize);
        for _ in 0..header.detail_vert_count * 3 {
            tile.detail_verts.push(cursor.read_f32::<LittleEndian>()?);
        }

        // Read detail triangles
        tile.detail_tris
            .reserve((header.detail_tri_count * 4) as usize);
        for _ in 0..header.detail_tri_count * 4 {
            tile.detail_tris.push(cursor.read_u8()?);
        }
    }

    // Read BV-tree nodes
    if header.bv_node_count > 0 {
        tile.bvh_nodes.reserve(header.bv_node_count as usize);
        for _ in 0..header.bv_node_count {
            let node_data = BVNodeData::read_from(&mut cursor)?;

            // BVNode stores quantized values directly
            tile.bvh_nodes.push(BVNode {
                bmin: node_data.bmin,
                bmax: node_data.bmax,
                i: node_data.i,
            });
        }
    }

    // Read off-mesh connections
    if header.off_mesh_con_count > 0 {
        tile.off_mesh_connections
            .reserve(header.off_mesh_con_count as usize);
        for _ in 0..header.off_mesh_con_count {
            let conn_data = OffMeshConnectionData::read_from(&mut cursor)?;

            let mut conn = OffMeshConnection::new();
            conn.pos = conn_data.pos;
            conn.radius = conn_data.rad;
            conn.poly = PolyRef::new(conn_data.poly as u32);
            conn.flags = PolyFlags::from_bits_truncate(conn_data.flags as u16);
            conn.area = conn_data.area;
            conn.dir = conn_data.side;
            conn.user_id = conn_data.user_id;

            tile.off_mesh_connections.push(conn);
        }
    }

    Ok(tile)
}

/// Saves a navigation mesh tile to C++ binary format
pub fn save_tile_to_binary(tile: &MeshTile) -> Result<Vec<u8>> {
    let mut buffer = Vec::new();

    // Get tile header info
    let header_info = tile
        .header
        .as_ref()
        .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

    // Calculate BV quantization factor
    let bv_quant_factor = if tile.bvh_nodes.is_empty() {
        1.0
    } else {
        let range = [
            header_info.bmax[0] - header_info.bmin[0],
            header_info.bmax[1] - header_info.bmin[1],
            header_info.bmax[2] - header_info.bmin[2],
        ];
        let max_range = range[0].max(range[1]).max(range[2]);
        max_range / 65535.0
    };

    // Create header
    let header = MeshHeader {
        magic: DT_NAVMESH_MAGIC,
        version: DT_NAVMESH_VERSION,
        x: header_info.x,
        y: header_info.y,
        layer: header_info.layer,
        user_id: header_info.user_id,
        poly_count: tile.polys.len() as i32,
        vert_count: (tile.verts.len() / 3) as i32,
        max_link_count: tile.links.len() as i32,
        detail_mesh_count: tile.detail_meshes.len() as i32,
        detail_vert_count: (tile.detail_verts.len() / 3) as i32,
        detail_tri_count: (tile.detail_tris.len() / 4) as i32,
        bv_node_count: tile.bvh_nodes.len() as i32,
        off_mesh_con_count: tile.off_mesh_connections.len() as i32,
        off_mesh_base: tile.polys.len() as i32,
        walkable_height: 2.0, // Default values - should be stored somewhere
        walkable_radius: 0.6,
        walkable_climb: 0.9,
        bmin: header_info.bmin,
        bmax: header_info.bmax,
        bv_quant_factor,
    };

    // Write header
    header.write_to(&mut buffer)?;

    // Write vertices
    for &v in &tile.verts {
        buffer.write_f32::<LittleEndian>(v)?;
    }

    // Write polygons
    for poly in &tile.polys {
        let poly_data = PolyData::from_poly(poly);
        poly_data.write_to(&mut buffer)?;
    }

    // Write empty link data (will be rebuilt on load)
    for _ in 0..header.max_link_count {
        // Write empty dtLink structure
        buffer.write_u32::<LittleEndian>(0)?; // ref
        buffer.write_u32::<LittleEndian>(0)?; // next
        buffer.write_u8(0)?; // edge
        buffer.write_u8(0)?; // side
        buffer.write_u8(0)?; // bmin
        buffer.write_u8(0)?; // bmax
    }

    // Write detail meshes
    for detail in &tile.detail_meshes {
        buffer.write_u32::<LittleEndian>(detail.vert_base)?;
        buffer.write_u32::<LittleEndian>(detail.tri_base)?;
        buffer.write_u8(detail.vert_count)?;
        buffer.write_u8(detail.tri_count)?;
        // Write 2 padding bytes for C++ struct alignment
        buffer.write_u8(0)?;
        buffer.write_u8(0)?;
    }

    // Write detail vertices
    for &v in &tile.detail_verts {
        buffer.write_f32::<LittleEndian>(v)?;
    }

    // Write detail triangles
    for &t in &tile.detail_tris {
        buffer.write_u8(t)?;
    }

    // Write BV-tree nodes
    for node in &tile.bvh_nodes {
        // Nodes are already quantized, use them directly
        let node_data = BVNodeData {
            bmin: node.bmin,
            bmax: node.bmax,
            i: node.i,
        };
        node_data.write_to(&mut buffer)?;
    }

    // Write off-mesh connections
    for conn in &tile.off_mesh_connections {
        let conn_data = OffMeshConnectionData {
            pos: conn.pos,
            rad: conn.radius,
            poly: conn.poly.id() as u16,
            flags: conn.flags.bits() as u8,
            side: conn.dir,
            area: conn.area,
            user_id: conn.user_id,
        };
        conn_data.write_to(&mut buffer)?;
    }

    Ok(buffer)
}

/// Multi-tile navigation mesh header
#[repr(C)]
#[derive(Debug, Clone)]
struct NavMeshSetHeader {
    magic: u32,
    version: u32,
    tile_count: i32,
    params: NavMeshParamsData,
}

/// NavMeshParams in binary format
#[repr(C)]
#[derive(Debug, Clone)]
struct NavMeshParamsData {
    origin: [f32; 3],
    tile_width: f32,
    tile_height: f32,
    max_tiles: i32,
    max_polys_per_tile: i32,
}

impl NavMeshSetHeader {
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<()> {
        writer.write_u32::<LittleEndian>(self.magic)?;
        writer.write_u32::<LittleEndian>(self.version)?;
        writer.write_i32::<LittleEndian>(self.tile_count)?;

        // Write params
        for &v in &self.params.origin {
            writer.write_f32::<LittleEndian>(v)?;
        }
        writer.write_f32::<LittleEndian>(self.params.tile_width)?;
        writer.write_f32::<LittleEndian>(self.params.tile_height)?;
        writer.write_i32::<LittleEndian>(self.params.max_tiles)?;
        writer.write_i32::<LittleEndian>(self.params.max_polys_per_tile)?;

        Ok(())
    }

    fn read_from<R: Read>(reader: &mut R) -> Result<Self> {
        let magic = reader.read_u32::<LittleEndian>()?;
        let version = reader.read_u32::<LittleEndian>()?;
        let tile_count = reader.read_i32::<LittleEndian>()?;

        // Read params
        let mut origin = [0.0; 3];
        for v in &mut origin {
            *v = reader.read_f32::<LittleEndian>()?;
        }
        let tile_width = reader.read_f32::<LittleEndian>()?;
        let tile_height = reader.read_f32::<LittleEndian>()?;
        let max_tiles = reader.read_i32::<LittleEndian>()?;
        let max_polys_per_tile = reader.read_i32::<LittleEndian>()?;

        Ok(NavMeshSetHeader {
            magic,
            version,
            tile_count,
            params: NavMeshParamsData {
                origin,
                tile_width,
                tile_height,
                max_tiles,
                max_polys_per_tile,
            },
        })
    }
}

/// Saves a complete multi-tile navigation mesh to binary format
pub fn save_nav_mesh_to_binary(nav_mesh: &NavMesh) -> Result<Vec<u8>> {
    let mut buffer = Vec::new();

    // Get all tiles
    let tiles = nav_mesh.get_all_tiles();
    let tile_count = tiles.iter().filter(|t| t.header.is_some()).count() as i32;

    // Create multi-tile header
    let params = nav_mesh.get_params();
    let header = NavMeshSetHeader {
        magic: DT_NAVMESH_MAGIC,
        version: DT_NAVMESH_VERSION,
        tile_count,
        params: NavMeshParamsData {
            origin: params.origin,
            tile_width: params.tile_width,
            tile_height: params.tile_height,
            max_tiles: params.max_tiles,
            max_polys_per_tile: params.max_polys_per_tile,
        },
    };

    // Write header
    header.write_to(&mut buffer)?;

    // Write each tile
    for tile in tiles {
        if tile.header.is_some() {
            // Write tile size first for easier loading
            let tile_data = save_tile_to_binary(tile)?;
            buffer.write_u32::<LittleEndian>(tile_data.len() as u32)?;
            buffer.extend_from_slice(&tile_data);
        }
    }

    Ok(buffer)
}

/// Loads a complete navigation mesh from binary format (supports multi-tile)
pub fn load_nav_mesh_from_binary(data: &[u8]) -> Result<NavMesh> {
    let mut cursor = Cursor::new(data);

    // Check if this is a multi-tile format by peeking at magic/version
    let magic = cursor.read_u32::<LittleEndian>()?;
    cursor.set_position(0); // Reset to beginning

    if magic == DT_NAVMESH_MAGIC {
        // Check version to determine format
        cursor.set_position(4);
        let version = cursor.read_u32::<LittleEndian>()?;
        cursor.set_position(0);

        if version == DT_NAVMESH_VERSION {
            // Could be either single-tile or multi-tile
            // Try to read as multi-tile first
            // Minimum size for multi-tile header: magic(4) + version(4) + tile_count(4) + params(7*4) = 40 bytes
            if data.len() >= 40 {
                // Read header
                let header = NavMeshSetHeader::read_from(&mut cursor)?;

                // Check if this looks like a multi-tile header
                if header.tile_count >= 0 && header.tile_count <= 65536 {
                    // Create navigation mesh
                    let params = NavMeshParams {
                        origin: header.params.origin,
                        tile_width: header.params.tile_width,
                        tile_height: header.params.tile_height,
                        max_tiles: header.params.max_tiles,
                        max_polys_per_tile: header.params.max_polys_per_tile,
                    };

                    let mut nav_mesh = NavMesh::new(params)?;

                    // Load each tile (if any)
                    if header.tile_count > 0 {
                        for _ in 0..header.tile_count {
                            let tile_size = cursor.read_u32::<LittleEndian>()? as usize;
                            let pos = cursor.position() as usize;

                            if pos + tile_size > data.len() {
                                return Err(Error::Detour(Status::InvalidParam.to_string()));
                            }

                            let tile_data = &data[pos..pos + tile_size];
                            let tile = load_tile_from_binary(tile_data)?;
                            nav_mesh.add_mesh_tile(tile)?;

                            cursor.set_position((pos + tile_size) as u64);
                        }
                    }

                    return Ok(nav_mesh);
                }
            }
        }
    }

    // Fall back to single-tile format
    let tile = load_tile_from_binary(data)?;

    // Extract parameters from tile
    let header = tile
        .header
        .as_ref()
        .ok_or(Error::Detour(Status::InvalidParam.to_string()))?;

    let params = NavMeshParams {
        origin: header.bmin,
        tile_width: header.bmax[0] - header.bmin[0],
        tile_height: header.bmax[2] - header.bmin[2],
        max_tiles: 1,
        max_polys_per_tile: header.poly_count,
    };

    let mut nav_mesh = NavMesh::new(params)?;

    // Add the tile to the navigation mesh
    let _tile_ref = nav_mesh.add_mesh_tile(tile)?;

    Ok(nav_mesh)
}

/// Gets the size needed to store tile state
pub fn get_tile_state_size(tile: &MeshTile) -> usize {
    if tile.header.is_none() {
        return 0;
    }

    let header = tile.header.as_ref().unwrap();

    // Calculate aligned sizes
    let header_size = align4(std::mem::size_of::<TileState>());
    let state_size = align4(header.poly_count as usize * std::mem::size_of::<PolyState>());

    header_size + state_size
}

/// Stores tile state to a byte buffer
pub fn store_tile_state(tile: &MeshTile, tile_ref: PolyRef) -> Result<Vec<u8>> {
    if tile.header.is_none() {
        return Err(Error::Detour(Status::InvalidParam.to_string()));
    }

    let header = tile.header.as_ref().unwrap();
    let size = get_tile_state_size(tile);
    let mut buffer = Vec::with_capacity(size);

    // Write header
    let state_header = TileState {
        magic: DT_NAVMESH_STATE_MAGIC,
        version: DT_NAVMESH_STATE_VERSION,
        tile_ref,
    };
    state_header.write_to(&mut buffer)?;

    // Align to 4 bytes
    while buffer.len() % 4 != 0 {
        buffer.push(0);
    }

    // Write polygon states
    for i in 0..header.poly_count as usize {
        if i < tile.polys.len() {
            let poly = &tile.polys[i];
            let state = PolyState {
                flags: poly.flags.bits(),
                area: poly.area,
            };
            state.write_to(&mut buffer)?;
        }
    }

    // Final alignment
    while buffer.len() % 4 != 0 {
        buffer.push(0);
    }

    Ok(buffer)
}

/// Restores tile state from a byte buffer
pub fn restore_tile_state(tile: &mut MeshTile, data: &[u8], expected_ref: PolyRef) -> Result<()> {
    if tile.header.is_none() {
        return Err(Error::Detour(Status::InvalidParam.to_string()));
    }

    let header = tile.header.as_ref().unwrap();
    let mut cursor = Cursor::new(data);

    // Read and validate header
    let state_header = TileState::read_from(&mut cursor)?;
    if state_header.magic != DT_NAVMESH_STATE_MAGIC {
        return Err(Error::Detour(Status::WrongMagic.to_string()));
    }
    if state_header.version != DT_NAVMESH_STATE_VERSION {
        return Err(Error::Detour(Status::WrongVersion.to_string()));
    }
    if state_header.tile_ref != expected_ref {
        return Err(Error::Detour(Status::InvalidParam.to_string()));
    }

    // Skip alignment
    let header_size = align4(std::mem::size_of::<TileState>());
    cursor.set_position(header_size as u64);

    // Read polygon states
    for i in 0..header.poly_count as usize {
        if i < tile.polys.len() {
            let state = PolyState::read_from(&mut cursor)?;
            tile.polys[i].flags = PolyFlags::from_bits_truncate(state.flags);
            tile.polys[i].area = state.area;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_align4() {
        assert_eq!(align4(0), 0);
        assert_eq!(align4(1), 4);
        assert_eq!(align4(2), 4);
        assert_eq!(align4(3), 4);
        assert_eq!(align4(4), 4);
        assert_eq!(align4(5), 8);
    }

    #[test]
    fn test_poly_data_conversion() {
        let mut poly = Poly::new(5, PolyType::Ground, PolyFlags::WALK);
        poly.verts[0] = 10;
        poly.verts[1] = 20;
        poly.vert_count = 2;

        let data = PolyData::from_poly(&poly);
        assert_eq!(data.verts[0], 10);
        assert_eq!(data.verts[1], 20);
        assert_eq!(data.vert_count, 2);
        assert_eq!(data.area_and_type, 5); // area=5, type=0
        assert_eq!(data.flags, PolyFlags::WALK.bits());

        let poly2 = data.to_poly();
        assert_eq!(poly2.verts[0], poly.verts[0]);
        assert_eq!(poly2.verts[1], poly.verts[1]);
        assert_eq!(poly2.vert_count, poly.vert_count);
        assert_eq!(poly2.area, poly.area);
        assert_eq!(poly2.poly_type, poly.poly_type);
        assert_eq!(poly2.flags, poly.flags);
    }
}
