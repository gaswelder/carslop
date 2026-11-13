//! Seekable offline render geometry. Only the index remains in CPU memory;
//! individual layers are decoded, uploaded, and immediately released.
use crate::city_assets;
use crate::geometry::XYZ;
use crate::{chunks::Chunk, scene::Vertex};
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
impl Archive {
    pub fn open(path: &Path) -> std::io::Result<Self> {
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
                XYZ::from_fn(|axis| {
                    let i = (side * 3 + axis) * 4;
                    f32::from_le_bytes(bytes[i..i + 4].try_into().unwrap())
                })
            });
            if (0..3).any(|i| {
                !bounds[0].component(i).is_finite()
                    || !bounds[1].component(i).is_finite()
                    || bounds[0].component(i) > bounds[1].component(i)
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
        Ok(Self {
            file,
            path: path.to_owned(),
            entries,
            bytes_read: 16 + n * ENTRY_SIZE,
            peak_decode_bytes: 0,
        })
    }
    pub fn fork(&self) -> std::io::Result<Self> {
        Self::open(&self.path)
    }
    pub fn open_for(assets: &city_assets::CityAsset) -> std::io::Result<Self> {
        Self::open(&assets.directory().join("scene.pack"))
    }
    pub fn read_layer(&mut self, index: usize, layer: usize) -> std::io::Result<Vec<Vertex>> {
        let entry = self
            .entries
            .get(index)
            .ok_or_else(|| error("missing scene chunk"))?;
        let span = *entry
            .layers
            .get(layer)
            .ok_or_else(|| error("unknown scene layer"))?;
        let mut bytes = vec![0; span.count * STRIDE];
        self.file.seek(SeekFrom::Start(span.offset))?;
        self.file.read_exact(&mut bytes)?;
        self.bytes_read += bytes.len();
        self.peak_decode_bytes = self
            .peak_decode_bytes
            .max(bytes.len() + span.count * std::mem::size_of::<Vertex>());
        let mut vertices = Vec::with_capacity(span.count);
        for b in bytes.as_chunks::<STRIDE>().0 {
            let position = std::array::from_fn(|i| {
                f32::from_le_bytes(b[i * 4..i * 4 + 4].try_into().unwrap())
            });
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
impl Spool {
    pub fn new(directory: &Path) -> std::io::Result<Self> {
        Ok(Self {
            file: tempfile::NamedTempFile::new_in(directory)?,
            chunks: Default::default(),
        })
    }
    pub fn append(&mut self, chunk: Chunk) -> std::io::Result<()> {
        let bytes = encode(&chunk.vertices);
        let offset = self.file.stream_position()?;
        self.file.write_all(&bytes)?;
        let entry = self.chunks.entry(chunk.cell).or_insert_with(|| SpoolChunk {
            bounds: chunk.bounds,
            fragments: Vec::new(),
        });
        for axis in 0..3 {
            *entry.bounds[0].component_mut(axis) = entry.bounds[0]
                .component(axis)
                .min(chunk.bounds[0].component(axis));
            *entry.bounds[1].component_mut(axis) = entry.bounds[1]
                .component(axis)
                .max(chunk.bounds[1].component(axis));
        }
        entry.fragments.push((offset, bytes.len()));
        Ok(())
    }
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
            XYZ::from_fn(|_| f32::INFINITY),
            XYZ::from_fn(|_| f32::NEG_INFINITY),
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
                    *bounds[0].component_mut(axis) = bounds[0]
                        .component(axis)
                        .min(chunk.bounds[0].component(axis));
                    *bounds[1].component_mut(axis) = bounds[1]
                        .component(axis)
                        .max(chunk.bounds[1].component(axis));
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
    Archive::open(file.path())?;
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
        let mut spool = Spool::new(directory)?;
        for chunk in chunks {
            spool.append(chunk)?;
        }
        spools.push(spool);
    }
    write_spooled(path, &mut spools)
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::chunks;
    use crate::test_support;
    use std::path::PathBuf;
    struct TestFile(PathBuf);
    impl TestFile {
        fn new() -> Self {
            static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            Self(std::env::temp_dir().join(format!(
                "cars-scene-{}-{}.pack",
                std::process::id(),
                NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
            )))
        }
    }
    impl Drop for TestFile {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    #[test]
    fn spooled_archive_preserves_geometry_across_flushes_and_sparse_layers() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("scene.pack");
        let mut builder = chunks::ChunkBuilder::spooled(directory.path()).unwrap();
        for i in 0..90_000 {
            let mut tri: [Vertex; 3] = triangle().try_into().unwrap();
            for v in &mut tri {
                v.position[0] += (i % 3) as f32 * 512.0;
                v.color[0] = (i % 251) as f32 / 255.0;
            }
            builder.push(tri);
        }
        let mut layers = vec![
            builder.finish_spooled().unwrap(),
            Spool::new(directory.path()).unwrap(),
            Spool::new(directory.path()).unwrap(),
        ];
        for chunk in test_support::partition(triangle()) {
            layers[2].append(chunk).unwrap();
        }
        write_spooled(&path, &mut layers).unwrap();
        let mut archive = Archive::open(&path).unwrap();
        assert_eq!(archive.entries.len(), 3);
        for cell in 0..3 {
            let offset = cell as f32 * 512.0;
            assert_eq!(
                archive.entries[cell].bounds,
                [
                    XYZ {
                        x: 255.75 + offset,
                        y: 4.125,
                        z: -1234.5
                    },
                    XYZ {
                        x: 257.0 + offset,
                        y: 7.0,
                        z: -1234.5
                    }
                ]
            );
            let loaded = archive.read_layer(cell, 0).unwrap();
            assert_eq!(loaded.len(), 90_000);
            for (j, tri) in loaded.chunks_exact(3).enumerate() {
                let i = j * 3 + cell;
                for (mut expected, actual) in triangle().into_iter().zip(tri) {
                    expected.position[0] += offset;
                    expected.color[0] = (i % 251) as f32 / 255.0;
                    assert_vertex_matches(expected, actual);
                }
            }
            assert!(archive.read_layer(cell, 1).unwrap().is_empty());
            let detail = archive.read_layer(cell, 2).unwrap();
            assert_eq!(detail.len(), if cell == 0 { 3 } else { 0 });
            for (expected, actual) in triangle().into_iter().zip(&detail) {
                assert_vertex_matches(expected, actual);
            }
        }
    }

    fn assert_vertex_matches(expected: Vertex, actual: &Vertex) {
        assert_eq!(expected.position, actual.position);
        assert_eq!(expected.material, actual.material);
        for axis in 0..3 {
            assert!((expected.normal[axis] - actual.normal[axis]).abs() < 0.00004);
            assert!((expected.color[axis] - actual.color[axis]).abs() < 1.0 / 255.0);
        }
    }

    #[test]
    fn failed_spooled_replacement_preserves_the_working_archive() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("scene.pack");
        fixture(&path);
        let before = std::fs::read(&path).unwrap();
        let mut empty: Vec<_> = (0..3)
            .map(|_| Spool::new(directory.path()).unwrap())
            .collect();
        assert!(write_spooled(&path, &mut empty).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), before);
    }

    fn triangle() -> Vec<Vertex> {
        [
            [255.75, 4.125, -1234.5],
            [257.0, 4.125, -1234.5],
            [256.0, 7.0, -1234.5],
        ]
        .map(|position| Vertex {
            position,
            normal: [0.6, 0.8, 0.0],
            color: [0.18, 0.2, 0.22],
            material: [-2.0, 102.25],
        })
        .to_vec()
    }
    fn fixture(path: &Path) {
        write(
            path,
            std::array::from_fn(|_| test_support::partition(triangle())),
        )
        .unwrap();
    }
    #[test]
    fn archive_loads_only_index_then_requested_layer_and_preserves_geometry() {
        let path = TestFile::new();
        fixture(&path.0);
        let mut archive = Archive::open(&path.0).unwrap();
        assert_eq!(archive.peak_decode_bytes, 0);
        assert_eq!(archive.bytes_read, 16 + ENTRY_SIZE);
        assert!((archive.bytes_read as u64) < std::fs::metadata(&path.0).unwrap().len());
        let loaded = archive.read_layer(0, 0).unwrap();
        assert_eq!(archive.bytes_read, 16 + ENTRY_SIZE + 3 * STRIDE);
        for (a, b) in triangle().iter().zip(loaded) {
            assert_eq!(a.position, b.position);
            assert_eq!(a.material, b.material);
            for axis in 0..3 {
                assert!((a.normal[axis] - b.normal[axis]).abs() < 0.00004);
                assert!((a.color[axis] - b.color[axis]).abs() < 1.0 / 255.0);
            }
        }
        assert!(archive.read_layer(1, 0).is_err());
        assert!(archive.read_layer(0, 3).is_err());
    }
}
