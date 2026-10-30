//! Seekable offline render geometry. Only the index remains in CPU memory;
//! individual layers are decoded, uploaded, and immediately released.
use crate::{
    chunks::Chunk,
    scene::Vertex,
    street_surface::{crc32, crc32_bytes},
};
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
    crc: u32,
}
#[derive(Clone, Copy, Debug)]
pub struct Entry {
    pub bounds: [[f32; 3]; 2],
    pub layers: [Span; 3],
}
pub struct Archive {
    file: crate::package::Reader,
    path: std::path::PathBuf,
    key: u32,
    pub entries: Vec<Entry>,
    pub bytes_read: usize,
    pub peak_decode_bytes: usize,
}
fn error(message: &str) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidData, message)
}
pub fn signature() -> u32 {
    static KEY: std::sync::OnceLock<u32> = std::sync::OnceLock::new();
    *KEY.get_or_init(|| {
        let data = [
            "city.txt",
            "streets.txt",
            "furniture.txt",
            "landscape.txt",
            "elevation/heightfield.txt",
        ];
        let data_sums = data
            .iter()
            .map(|name| crc32(&crate::city_assets::read(name)))
            .collect::<Vec<_>>();
        let imagery = match crate::package::read_to_string(
            &crate::city_assets::directory().join("imagery-trees.txt"),
        ) {
            Ok(text) => crc32(&text),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => 0,
            Err(e) => panic!("cannot fingerprint imagery trees: {e}"),
        };
        let sources = [
            include_str!("city_assets.rs"),
            include_str!("scene.rs"),
            include_str!("city_scene.rs"),
            include_str!("engomi_art.rs"),
            include_str!("road_furniture.rs"),
            include_str!("city.rs"),
            include_str!("hud.rs"),
            include_str!("traffic.rs"),
            include_str!("landscape.rs"),
            include_str!("track.rs"),
            include_str!("street_surface.rs"),
            include_str!("elevation.rs"),
            include_str!("chunks.rs"),
            include_str!("scene_archive.rs"),
        ];
        let mut sums = data_sums
            .iter()
            .map(|v| format!("{v:08x}"))
            .collect::<String>();
        sums.push_str(&format!("{imagery:08x}"));
        sums.push_str(
            &sources
                .iter()
                .map(|s| format!("{:08x}", crc32(s)))
                .collect::<String>(),
        );
        crc32(&sums)
    })
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
    pub fn open(path: &Path, expected: u32) -> std::io::Result<Self> {
        let mut file = crate::package::Reader::open(path)?;
        let length = file.len()?;
        let mut header = [0u8; 16];
        file.read_exact(&mut header)?;
        let n = u32::from_le_bytes(header[12..16].try_into().unwrap()) as usize;
        if &header[..8] != MAGIC
            || u32::from_le_bytes(header[8..12].try_into().unwrap()) != expected
        {
            return Err(error(
                "stale city scene archive; run bake-city --location NAME",
            ));
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
            let bounds: [[f32; 3]; 2] = std::array::from_fn(|side| {
                std::array::from_fn(|axis| {
                    let i = (side * 3 + axis) * 4;
                    f32::from_le_bytes(bytes[i..i + 4].try_into().unwrap())
                })
            });
            if (0..3).any(|i| {
                !bounds[0][i].is_finite()
                    || !bounds[1][i].is_finite()
                    || bounds[0][i] > bounds[1][i]
            }) {
                return Err(error("invalid chunk bounds"));
            }
            let mut layers = [Span {
                offset: 0,
                count: 0,
                crc: 0,
            }; 3];
            for (layer, span) in layers.iter_mut().enumerate() {
                let i = 24 + layer * 16;
                *span = Span {
                    offset: u64::from_le_bytes(bytes[i..i + 8].try_into().unwrap()),
                    count: u32::from_le_bytes(bytes[i + 8..i + 12].try_into().unwrap()) as usize,
                    crc: u32::from_le_bytes(bytes[i + 12..i + 16].try_into().unwrap()),
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
            key: expected,
            entries,
            bytes_read: 16 + n * ENTRY_SIZE,
            peak_decode_bytes: 0,
        })
    }
    pub fn fork(&self) -> std::io::Result<Self> {
        Self::open(&self.path, self.key)
    }
    pub fn open_default() -> std::io::Result<Self> {
        if let Some(path) = std::env::var_os("NFS_CITY_SCENE") {
            return Self::open(Path::new(&path), signature());
        }
        Self::open(
            &crate::city_assets::directory().join("scene.pack"),
            signature(),
        )
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
        if crc32_bytes(&bytes) != span.crc {
            return Err(error("damaged scene layer; rebuild the city archive"));
        }
        let mut vertices = Vec::with_capacity(span.count);
        for b in bytes.chunks_exact(STRIDE) {
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
pub fn write(path: &Path, layers: [Vec<Chunk>; 3], key: u32) -> std::io::Result<()> {
    let mut chunks = std::collections::BTreeMap::<[i32; 2], [Option<Chunk>; 3]>::new();
    for (layer, values) in layers.into_iter().enumerate() {
        for chunk in values {
            let cell = chunk.cell;
            chunks.entry(cell).or_default()[layer] = Some(chunk);
        }
    }
    let temporary = path.with_extension("pack.tmp");
    let mut file = File::create(&temporary)?;
    file.write_all(&vec![0; 16 + chunks.len() * ENTRY_SIZE])?;
    let mut entries = Vec::new();
    for (_, layers) in chunks {
        let mut bounds = [[f32::INFINITY; 3], [f32::NEG_INFINITY; 3]];
        let mut spans = [Span {
            offset: 0,
            count: 0,
            crc: 0,
        }; 3];
        for (layer, chunk) in layers.into_iter().enumerate() {
            let vertices = if let Some(chunk) = chunk {
                for axis in 0..3 {
                    bounds[0][axis] = bounds[0][axis].min(chunk.bounds[0][axis]);
                    bounds[1][axis] = bounds[1][axis].max(chunk.bounds[1][axis]);
                }
                chunk.vertices
            } else {
                Vec::new()
            };
            let bytes = encode(&vertices);
            spans[layer] = Span {
                offset: file.stream_position()?,
                count: vertices.len(),
                crc: crc32_bytes(&bytes),
            };
            file.write_all(&bytes)?;
        }
        entries.push(Entry {
            bounds,
            layers: spans,
        });
    }
    file.seek(SeekFrom::Start(0))?;
    file.write_all(MAGIC)?;
    file.write_all(&key.to_le_bytes())?;
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
            file.write_all(&span.crc.to_le_bytes())?;
        }
    }
    file.sync_all()?;
    drop(file);
    std::fs::rename(temporary, path)?;
    Ok(())
}
pub fn bake_default() -> std::io::Result<()> {
    let city = crate::track::TrackId::Engomi.track().city().expect("city");
    let directory = crate::city_assets::directory();
    let packaged = crate::package::is_package(directory);
    let temporary = if packaged {
        Some(
            tempfile::Builder::new()
                .prefix(".location-bake-")
                .tempdir_in(directory.parent().unwrap())?,
        )
    } else {
        None
    };
    let path = temporary
        .as_ref()
        .map(|t| t.path().join("scene.pack"))
        .unwrap_or_else(|| directory.join("scene.pack"));
    let started = std::time::Instant::now();
    println!(
        "Generating city geometry with {} trees...",
        city.landscape.trees.len()
    );
    let key = signature();
    let layers = crate::scene::city_layers(city);
    println!("Writing city render archive...");
    write(&path, layers, key)?;
    let archive = Archive::open(&path, signature())?;
    let counts: [usize; 3] =
        std::array::from_fn(|layer| archive.entries.iter().map(|e| e.layers[layer].count).sum());
    println!("Baked {} chunks: {} ground, {} detailed building, {} distant building vertices; {:.1} MiB; {:.2}s",archive.entries.len(),counts[0],counts[1],counts[2],crate::package::size(&path)? as f32/1048576.0,started.elapsed().as_secs_f32());
    drop(archive);
    if packaged {
        crate::package::replace_asset(directory, "scene.pack", &path)
            .map_err(|e| error(&e.to_string()))?;
        Archive::open(&directory.join("scene.pack"), key)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
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
    fn packed_signal_material_preserves_every_program_phase_and_lens() {
        let path = TestFile::new();
        let mut vertices = Vec::new();
        for phase in 0..50 {
            for lens in 0..3 {
                let mut tri = triangle();
                for v in &mut tri {
                    v.material = [-2.0, phase as f32 * 4.0 + lens as f32 + 0.25];
                }
                vertices.extend(tri);
            }
        }
        write(
            &path.0,
            [
                crate::chunks::partition(vertices.clone()),
                Vec::new(),
                Vec::new(),
            ],
            123,
        )
        .unwrap();
        let mut archive = Archive::open(&path.0, 123).unwrap();
        let decoded = archive.read_layer(0, 0).unwrap();
        assert_eq!(decoded.len(), vertices.len());
        for (before, after) in vertices.iter().zip(decoded) {
            assert_eq!(before.material, after.material);
        }
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
            std::array::from_fn(|_| crate::chunks::partition(triangle())),
            123,
        )
        .unwrap();
    }
    #[test]
    fn archive_loads_only_index_then_requested_layer_and_preserves_geometry() {
        let path = TestFile::new();
        fixture(&path.0);
        let mut archive = Archive::open(&path.0, 123).unwrap();
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
    #[test]
    fn stale_truncated_and_corrupt_archives_fail_explicitly() {
        let path = TestFile::new();
        fixture(&path.0);
        assert!(Archive::open(&path.0, 124).is_err());
        let mut archive = Archive::open(&path.0, 123).unwrap();
        let offset = archive.entries[0].layers[0].offset;
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .open(&path.0)
            .unwrap();
        file.seek(SeekFrom::Start(offset)).unwrap();
        file.write_all(&[255]).unwrap();
        file.flush().unwrap();
        assert!(archive.read_layer(0, 0).is_err());
        file.set_len(20).unwrap();
        assert!(Archive::open(&path.0, 123).is_err());
    }
}
