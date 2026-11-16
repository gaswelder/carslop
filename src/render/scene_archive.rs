//! Seekable offline render geometry. Only the index remains in CPU memory;
//! individual layers are decoded, uploaded, and immediately released.

use crate::render::chunks::Chunk;
use crate::render::scene::Vertex;
use crate::storage;
use crate::util;
use crate::util::geometry::XYZ;
use std::{
    fs::File,
    io::{Read, Seek, SeekFrom, Write},
    path::Path,
};
const MAGIC: &[u8; 8] = b"CLOPCH03";
const STRIDE: usize = 24;
const ENTRY_SIZE: usize = 72;
#[derive(Clone, Copy, Debug)]
pub struct Span {
    offset: u64,
    pub count: usize,
}
#[derive(Clone, Copy, Debug)]
pub struct Entry {
    pub bounds: [XYZ; 2],
    pub layers: [Span; 3],
}
pub struct Archive {
    file: std::fs::File,
    path: std::path::PathBuf,
    pub entries: Vec<Entry>,
    pub bytes_read: usize,
    pub peak_decode_bytes: usize,
}
fn error(message: &str) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidData, message)
}
fn encode(vertices: &[Vertex]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(vertices.len() * STRIDE);
    for v in vertices {
        for x in v.position {
            bytes.extend(x.to_le_bytes());
        }
        for x in v.normal {
            bytes.extend(((x.clamp(-1.0, 1.0) * 32767.0).round() as i16).to_le_bytes());
        }
        for x in v.color {
            bytes.push((x.clamp(0.0, 1.0) * 255.0).round() as u8);
        }
        bytes.push(v.material[0] as i8 as u8);
        assert!(
            (0.0..=255.996).contains(&v.material[1]),
            "material parameter exceeds packed unsigned fixed-point range"
        );
        bytes.extend(((v.material[1] * 256.0).round() as u16).to_le_bytes());
    }
    bytes
}

pub fn open(path: &Path) -> std::io::Result<Archive> {
    let mut file = std::fs::File::open(path)?;
    let length = file.metadata()?.len();
    let mut header = [0u8; 16];
    file.read_exact(&mut header)?;
    let n = u32::from_le_bytes(header[12..16].try_into().unwrap()) as usize;
    if &header[..8] != MAGIC || header[8..12] != [0; 4] {
        return Err(error("unsupported city scene archive format"));
    }
    if n == 0 || n > 100_000 || 16 + (n * ENTRY_SIZE) as u64 > length {
        return Err(error("invalid scene index size"));
    }
    let mut entries = Vec::with_capacity(n);
    let data_start = (16 + n * ENTRY_SIZE) as u64;
    let mut previous_end = data_start;
    for _ in 0..n {
        let mut bytes = [0u8; ENTRY_SIZE];
        file.read_exact(&mut bytes)?;
        let bounds: [XYZ; 2] = std::array::from_fn(|side| {
            util::geometry::xyz_from_fn(|axis| {
                let i = (side * 3 + axis) * 4;
                f32::from_le_bytes(bytes[i..i + 4].try_into().unwrap())
            })
        });
        if (0..3).any(|i| {
            !util::geometry::xyz_component(bounds[0], i).is_finite()
                || !util::geometry::xyz_component(bounds[1], i).is_finite()
                || util::geometry::xyz_component(bounds[0], i)
                    > util::geometry::xyz_component(bounds[1], i)
        }) {
            return Err(error("invalid chunk bounds"));
        }
        let mut layers = [Span {
            offset: 0,
            count: 0,
        }; 3];
        for (layer, span) in layers.iter_mut().enumerate() {
            let i = 24 + layer * 16;
            *span = Span {
                offset: u64::from_le_bytes(bytes[i..i + 8].try_into().unwrap()),
                count: u32::from_le_bytes(bytes[i + 8..i + 12].try_into().unwrap()) as usize,
            };
            let end = span.offset.saturating_add((span.count * STRIDE) as u64);
            if span.count % 3 != 0
                || span.count > 2_000_000
                || span.offset != previous_end
                || end > length
            {
                return Err(error("invalid scene layer range"));
            }
            previous_end = end;
        }
        entries.push(Entry { bounds, layers });
    }
    if previous_end != length {
        return Err(error("scene archive has unindexed bytes"));
    }
    Ok(Archive {
        file,
        path: path.to_owned(),
        entries,
        bytes_read: 16 + n * ENTRY_SIZE,
        peak_decode_bytes: 0,
    })
}
pub fn fork(archive: &Archive) -> std::io::Result<Archive> {
    open(&archive.path)
}
pub fn open_for(assets: &storage::city_assets::CityAsset) -> std::io::Result<Archive> {
    open(&storage::city_assets::directory(assets).join("scene.pack"))
}
pub fn read_layer(
    archive: &mut Archive,
    index: usize,
    layer: usize,
) -> std::io::Result<Vec<Vertex>> {
    let entry = archive
        .entries
        .get(index)
        .ok_or_else(|| error("missing scene chunk"))?;
    let span = *entry
        .layers
        .get(layer)
        .ok_or_else(|| error("unknown scene layer"))?;
    let mut bytes = vec![0; span.count * STRIDE];
    archive.file.seek(SeekFrom::Start(span.offset))?;
    archive.file.read_exact(&mut bytes)?;
    archive.bytes_read += bytes.len();
    archive.peak_decode_bytes = archive
        .peak_decode_bytes
        .max(bytes.len() + span.count * std::mem::size_of::<Vertex>());
    let mut vertices = Vec::with_capacity(span.count);
    for b in bytes.as_chunks::<STRIDE>().0 {
        let position =
            std::array::from_fn(|i| f32::from_le_bytes(b[i * 4..i * 4 + 4].try_into().unwrap()));
        let normal = std::array::from_fn(|i| {
            i16::from_le_bytes(b[12 + i * 2..14 + i * 2].try_into().unwrap()) as f32 / 32767.0
        });
        let color = std::array::from_fn(|i| b[18 + i] as f32 / 255.0);
        let material = [
            b[21] as i8 as f32,
            u16::from_le_bytes(b[22..24].try_into().unwrap()) as f32 / 256.0,
        ];
        if position.iter().chain(normal.iter()).any(|v| !v.is_finite()) {
            return Err(error("nonfinite scene vertex"));
        }
        vertices.push(Vertex {
            position,
            normal,
            color,
            material,
        });
    }
    Ok(vertices)
}

// Packed fragments live on the location's disk, never in /tmp (which may be RAM).
// Only fragment offsets and bounds remain resident as the city is generated.
struct SpoolChunk {
    bounds: [XYZ; 2],
    fragments: Vec<(u64, usize)>,
}
pub struct Spool {
    file: tempfile::NamedTempFile,
    chunks: std::collections::BTreeMap<[i32; 2], SpoolChunk>,
}

pub fn new(directory: &Path) -> std::io::Result<Spool> {
    Ok(Spool {
        file: tempfile::NamedTempFile::new_in(directory)?,
        chunks: Default::default(),
    })
}
pub fn append(spool: &mut Spool, chunk: Chunk) -> std::io::Result<()> {
    let bytes = encode(&chunk.vertices);
    let offset = spool.file.stream_position()?;
    spool.file.write_all(&bytes)?;
    let entry = spool
        .chunks
        .entry(chunk.cell)
        .or_insert_with(|| SpoolChunk {
            bounds: chunk.bounds,
            fragments: Vec::new(),
        });
    for axis in 0..3 {
        *util::geometry::xyz_component_mut(&mut entry.bounds[0], axis) =
            util::geometry::xyz_component(entry.bounds[0], axis)
                .min(util::geometry::xyz_component(chunk.bounds[0], axis));
        *util::geometry::xyz_component_mut(&mut entry.bounds[1], axis) =
            util::geometry::xyz_component(entry.bounds[1], axis)
                .max(util::geometry::xyz_component(chunk.bounds[1], axis));
    }
    entry.fragments.push((offset, bytes.len()));
    Ok(())
}

pub fn write_spooled(path: &Path, layers: &mut [Spool]) -> std::io::Result<()> {
    let cells: std::collections::BTreeSet<_> = layers
        .iter()
        .flat_map(|layer| layer.chunks.keys().copied())
        .collect();
    let mut file = tempfile::NamedTempFile::new_in(
        path.parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new(".")),
    )?;
    file.write_all(&vec![0; 16 + cells.len() * ENTRY_SIZE])?;
    let mut entries = Vec::with_capacity(cells.len());
    let mut buffer = vec![0u8; 64 * 1024];
    for cell in cells {
        let mut bounds = [
            util::geometry::xyz_from_fn(|_| f32::INFINITY),
            util::geometry::xyz_from_fn(|_| f32::NEG_INFINITY),
        ];
        let mut spans = [Span {
            offset: 0,
            count: 0,
        }; 3];
        for (layer, spool) in layers.iter_mut().enumerate() {
            let offset = file.stream_position()?;
            let mut count = 0;
            if let Some(chunk) = spool.chunks.get(&cell) {
                for axis in 0..3 {
                    *util::geometry::xyz_component_mut(&mut bounds[0], axis) =
                        util::geometry::xyz_component(bounds[0], axis)
                            .min(util::geometry::xyz_component(chunk.bounds[0], axis));
                    *util::geometry::xyz_component_mut(&mut bounds[1], axis) =
                        util::geometry::xyz_component(bounds[1], axis)
                            .max(util::geometry::xyz_component(chunk.bounds[1], axis));
                }
                for &(start, size) in &chunk.fragments {
                    count += size / STRIDE;
                    spool.file.seek(SeekFrom::Start(start))?;
                    let mut remaining = size;
                    while remaining > 0 {
                        let n = remaining.min(buffer.len());
                        spool.file.read_exact(&mut buffer[..n])?;
                        file.write_all(&buffer[..n])?;
                        remaining -= n;
                    }
                }
            }
            if count > 2_000_000 {
                return Err(error("scene layer exceeds archive limit"));
            }
            spans[layer] = Span { offset, count };
        }
        entries.push(Entry {
            bounds,
            layers: spans,
        });
    }
    write_index(file.as_file_mut(), &entries)?;
    file.as_file().sync_all()?;
    // Validate the index before atomically replacing the previous working scene.
    open(file.path())?;
    file.persist(path).map_err(|e| e.error)?;
    Ok(())
}
fn write_index(file: &mut File, entries: &[Entry]) -> std::io::Result<()> {
    file.seek(SeekFrom::Start(0))?;
    file.write_all(MAGIC)?;
    file.write_all(&0u32.to_le_bytes())?;
    file.write_all(&(entries.len() as u32).to_le_bytes())?;
    for entry in entries {
        for side in entry.bounds {
            for value in side {
                file.write_all(&value.to_le_bytes())?;
            }
        }
        for span in entry.layers {
            file.write_all(&span.offset.to_le_bytes())?;
            file.write_all(&(span.count as u32).to_le_bytes())?;
            // Reserved bytes preserve the existing archive index layout.
            file.write_all(&[0; 4])?;
        }
    }
    Ok(())
}
/// Write already-partitioned geometry through the same disk-spooling path used
/// by city baking. Authored decorations and test fixtures use this entry point.
pub fn write(path: &Path, layers: [Vec<Chunk>; 3]) -> std::io::Result<()> {
    let directory = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let mut spools = Vec::with_capacity(3);
    for chunks in layers {
        let mut spool = new(directory)?;
        for chunk in chunks {
            append(&mut spool, chunk)?;
        }
        spools.push(spool);
    }
    write_spooled(path, &mut spools)
}
