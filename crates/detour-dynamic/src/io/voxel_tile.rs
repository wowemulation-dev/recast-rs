use glam::Vec3;
use recast::Heightfield;
use std::io::{Read, Result as IoResult, Write};

const SERIALIZED_SPAN_COUNT_BYTES: usize = 2;
const SERIALIZED_SPAN_BYTES: usize = 12;

#[derive(Debug, Clone)]
pub struct VoxelTile {
    pub tile_x: i32,
    pub tile_z: i32,
    pub border_size: i32,
    pub width: i32,
    pub depth: i32,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
    pub cell_size: f32,
    pub cell_height: f32,
    pub span_data: Vec<u8>,
}

impl VoxelTile {
    pub fn new(
        tile_x: i32,
        tile_z: i32,
        width: i32,
        depth: i32,
        bounds_min: Vec3,
        bounds_max: Vec3,
        cell_size: f32,
        cell_height: f32,
        border_size: i32,
        span_data: Vec<u8>,
    ) -> Self {
        VoxelTile {
            tile_x,
            tile_z,
            width,
            depth,
            bounds_min,
            bounds_max,
            cell_size,
            cell_height,
            border_size,
            span_data,
        }
    }

    pub fn from_heightfield(tile_x: i32, tile_z: i32, heightfield: &Heightfield) -> Self {
        let span_data = Self::serialize_spans(heightfield);

        VoxelTile {
            tile_x,
            tile_z,
            width: heightfield.width,
            depth: heightfield.height,
            bounds_min: heightfield.bmin,
            bounds_max: heightfield.bmax,
            cell_size: heightfield.cs,
            cell_height: heightfield.ch,
            border_size: 0, // Border size not stored in Heightfield
            span_data,
        }
    }

    pub fn heightfield(&self) -> Heightfield {
        self.deserialize_spans()
    }

    fn serialize_spans(heightfield: &Heightfield) -> Vec<u8> {
        // Count spans per cell
        let mut counts = vec![0u16; (heightfield.width * heightfield.height) as usize];
        let mut total_count = 0usize;

        for z in 0..heightfield.height {
            for x in 0..heightfield.width {
                let index = (z * heightfield.width + x) as usize;
                if let Some(span_rc) = heightfield.spans.get(&(x, z)).and_then(|s| s.as_ref()) {
                    let mut current_span = Some(span_rc.clone());
                    while let Some(span) = current_span {
                        counts[index] += 1;
                        total_count += 1;
                        current_span = span.borrow().next.clone();
                    }
                }
            }
        }

        // Serialize spans
        let data_size =
            total_count * SERIALIZED_SPAN_BYTES + counts.len() * SERIALIZED_SPAN_COUNT_BYTES;
        let mut data = Vec::with_capacity(data_size);

        for z in 0..heightfield.height {
            for x in 0..heightfield.width {
                let index = (z * heightfield.width + x) as usize;

                // Write span count
                data.extend_from_slice(&counts[index].to_be_bytes());

                // Write spans
                if let Some(span_rc) = heightfield.spans.get(&(x, z)).and_then(|s| s.as_ref()) {
                    let mut current_span = Some(span_rc.clone());
                    while let Some(span) = current_span {
                        let span_ref = span.borrow();
                        data.extend_from_slice(&(span_ref.min as u32).to_be_bytes());
                        data.extend_from_slice(&(span_ref.max as u32).to_be_bytes());
                        data.extend_from_slice(&(span_ref.area as u32).to_be_bytes());
                        current_span = span_ref.next.clone();
                    }
                }
            }
        }

        data
    }

    fn deserialize_spans(&self) -> Heightfield {
        let mut heightfield = Heightfield::new(
            self.width,
            self.depth,
            self.bounds_min,
            self.bounds_max,
            self.cell_size,
            self.cell_height,
        );

        let mut position = 0;
        for z in 0..self.depth {
            for x in 0..self.width {
                if position + 2 > self.span_data.len() {
                    break;
                }

                let span_count =
                    u16::from_be_bytes([self.span_data[position], self.span_data[position + 1]]);
                position += 2;

                for _ in 0..span_count {
                    if position + 12 > self.span_data.len() {
                        break;
                    }

                    let min = u32::from_be_bytes([
                        self.span_data[position],
                        self.span_data[position + 1],
                        self.span_data[position + 2],
                        self.span_data[position + 3],
                    ]) as i16;
                    position += 4;

                    let max = u32::from_be_bytes([
                        self.span_data[position],
                        self.span_data[position + 1],
                        self.span_data[position + 2],
                        self.span_data[position + 3],
                    ]) as i16;
                    position += 4;

                    let area = u32::from_be_bytes([
                        self.span_data[position],
                        self.span_data[position + 1],
                        self.span_data[position + 2],
                        self.span_data[position + 3],
                    ]) as u8;
                    position += 4;

                    // Add span to heightfield using the add_span method
                    let _ = heightfield.add_span(x, z, min, max, area);
                }
            }
        }

        heightfield
    }

    pub fn write_to<W: Write>(&self, writer: &mut W) -> IoResult<()> {
        writer.write_all(&self.tile_x.to_be_bytes())?;
        writer.write_all(&self.tile_z.to_be_bytes())?;
        writer.write_all(&self.width.to_be_bytes())?;
        writer.write_all(&self.depth.to_be_bytes())?;
        writer.write_all(&self.border_size.to_be_bytes())?;

        writer.write_all(&self.bounds_min.x.to_be_bytes())?;
        writer.write_all(&self.bounds_min.y.to_be_bytes())?;
        writer.write_all(&self.bounds_min.z.to_be_bytes())?;
        writer.write_all(&self.bounds_max.x.to_be_bytes())?;
        writer.write_all(&self.bounds_max.y.to_be_bytes())?;
        writer.write_all(&self.bounds_max.z.to_be_bytes())?;

        writer.write_all(&self.cell_size.to_be_bytes())?;
        writer.write_all(&self.cell_height.to_be_bytes())?;

        writer.write_all(&(self.span_data.len() as u32).to_be_bytes())?;
        writer.write_all(&self.span_data)?;

        Ok(())
    }

    pub fn read_from<R: Read>(reader: &mut R) -> IoResult<Self> {
        let mut buffer = [0u8; 4];

        reader.read_exact(&mut buffer)?;
        let tile_x = i32::from_be_bytes(buffer);

        reader.read_exact(&mut buffer)?;
        let tile_z = i32::from_be_bytes(buffer);

        reader.read_exact(&mut buffer)?;
        let width = i32::from_be_bytes(buffer);

        reader.read_exact(&mut buffer)?;
        let depth = i32::from_be_bytes(buffer);

        reader.read_exact(&mut buffer)?;
        let border_size = i32::from_be_bytes(buffer);

        reader.read_exact(&mut buffer)?;
        let bounds_min_x = f32::from_be_bytes(buffer);
        reader.read_exact(&mut buffer)?;
        let bounds_min_y = f32::from_be_bytes(buffer);
        reader.read_exact(&mut buffer)?;
        let bounds_min_z = f32::from_be_bytes(buffer);
        let bounds_min = Vec3::new(bounds_min_x, bounds_min_y, bounds_min_z);

        reader.read_exact(&mut buffer)?;
        let bounds_max_x = f32::from_be_bytes(buffer);
        reader.read_exact(&mut buffer)?;
        let bounds_max_y = f32::from_be_bytes(buffer);
        reader.read_exact(&mut buffer)?;
        let bounds_max_z = f32::from_be_bytes(buffer);
        let bounds_max = Vec3::new(bounds_max_x, bounds_max_y, bounds_max_z);

        reader.read_exact(&mut buffer)?;
        let cell_size = f32::from_be_bytes(buffer);

        reader.read_exact(&mut buffer)?;
        let cell_height = f32::from_be_bytes(buffer);

        reader.read_exact(&mut buffer)?;
        let span_data_len = u32::from_be_bytes(buffer) as usize;

        let mut span_data = vec![0u8; span_data_len];
        reader.read_exact(&mut span_data)?;

        Ok(VoxelTile {
            tile_x,
            tile_z,
            width,
            depth,
            bounds_min,
            bounds_max,
            cell_size,
            cell_height,
            border_size,
            span_data,
        })
    }
}
