//! Optional disk-backed aerial tiles, projected onto existing surface geometry.
use crate::geometry::{BoundingBoxXZ, XYZ, XZ};
use std::{
    collections::{HashSet, VecDeque},
    ffi::CString,
    path::{Path, PathBuf},
    sync::{mpsc, Arc, Condvar, Mutex},
};
const CAPACITY: usize = 48;
const GPU_BUDGET: usize = 64 * 1024 * 1024;
fn layer_bytes(mut dimension: usize) -> usize {
    let mut bytes = 0;
    loop {
        bytes += dimension * dimension * 4;
        if dimension == 1 {
            return bytes;
        }
        dimension /= 2;
    }
}
fn capacity(catalog: &Catalog) -> usize {
    CAPACITY
        .min(catalog.tiles.len())
        .min(GPU_BUDGET / layer_bytes(catalog.size + 2 * catalog.gutter))
}
#[derive(Clone, Debug)]
struct Tile {
    bounds: BoundingBoxXZ<f32>,
    name: String,
}
#[derive(Clone, Debug)]
struct Catalog {
    size: usize,
    gutter: usize,
    tiles: Vec<Tile>,
}
pub fn tile_filenames(text: &str) -> Result<Vec<String>, String> {
    Catalog::parse(text).map(|catalog| catalog.tiles.into_iter().map(|tile| tile.name).collect())
}
impl Catalog {
    fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        if lines.next() != Some("AERIAL 1") {
            return Err("unsupported aerial catalog".into());
        }
        let f: Vec<_> = lines
            .next()
            .ok_or("missing aerial dimensions")?
            .split_whitespace()
            .collect();
        if f.len() != 3 || f[0] != "size" {
            return Err("invalid aerial dimensions".into());
        }
        let size = f[1].parse::<usize>().map_err(|_| "invalid aerial size")?;
        let gutter = f[2].parse::<usize>().map_err(|_| "invalid aerial gutter")?;
        if !size.is_power_of_two() || !(64..=2048).contains(&size) || !(1..=32).contains(&gutter) {
            return Err("unsupported aerial dimensions".into());
        }
        let mut tiles = Vec::new();
        let mut names = HashSet::new();
        for line in lines {
            let f: Vec<_> = line.split_whitespace().collect();
            if f.len() < 6 || f[0] != "tile" {
                return Err("invalid aerial tile".into());
            }
            let mut bounds = [0.0; 4];
            for i in 0..4 {
                bounds[i] = f[i + 1]
                    .parse::<f32>()
                    .map_err(|_| "invalid aerial coordinate")?;
            }
            if !bounds.iter().all(|v| v.is_finite())
                || bounds[0] >= bounds[2]
                || bounds[1] >= bounds[3]
            {
                return Err("invalid aerial bounds".into());
            }
            if f[5].contains(['/', '\\'])
                || f[5].starts_with('.')
                || !names.insert(f[5].to_string())
            {
                return Err("invalid aerial filename".into());
            }
            tiles.push(Tile {
                bounds: BoundingBoxXZ {
                    min: XZ {
                        x: bounds[0],
                        z: bounds[1],
                    },
                    max: XZ {
                        x: bounds[2],
                        z: bounds[3],
                    },
                },
                name: f[5].into(),
            });
        }
        if tiles.is_empty() {
            return Err("empty aerial catalog".into());
        }
        Ok(Self {
            size,
            gutter,
            tiles,
        })
    }
}
fn distance(bounds: BoundingBoxXZ<f32>, eye: XYZ) -> f32 {
    (eye.x - eye.x.clamp(bounds.min.x, bounds.max.x))
        .hypot(eye.z - eye.z.clamp(bounds.min.z, bounds.max.z))
}
fn wanted(catalog: &Catalog, eye: XYZ, capacity: usize) -> Vec<usize> {
    let mut ids: Vec<_> = (0..catalog.tiles.len())
        .filter(|i| distance(catalog.tiles[*i].bounds, eye) < 568.0)
        .collect();
    ids.sort_by(|a, b| {
        distance(catalog.tiles[*a].bounds, eye)
            .total_cmp(&distance(catalog.tiles[*b].bounds, eye))
            .then(a.cmp(b))
    });
    ids.truncate(capacity);
    ids
}
fn read_tile(directory: &Path, catalog: &Catalog, id: usize) -> Result<Vec<u8>, String> {
    let tile = &catalog.tiles[id];
    let bytes = std::fs::read(&directory.join(&tile.name)).map_err(|e| e.to_string())?;
    let dimension = (catalog.size + catalog.gutter * 2) as u32;
    if tile.name.ends_with(".rgb") {
        if bytes.len() != dimension as usize * dimension as usize * 3 {
            return Err(format!("invalid aerial dimensions {}", tile.name));
        }
        return Ok(bytes);
    }
    let decoder = image::codecs::jpeg::JpegDecoder::new(std::io::Cursor::new(bytes))
        .map_err(|e| e.to_string())?;
    if image::ImageDecoder::dimensions(&decoder) != (dimension, dimension) {
        return Err(format!("invalid aerial dimensions {}", tile.name));
    }
    image::DynamicImage::from_decoder(decoder)
        .map(|im| im.to_rgb8().into_raw())
        .map_err(|e| e.to_string())
}

/// All aerial builders use the same JPEG quality.
pub fn encode_jpeg(rgb: &[u8], width: u32, height: u32) -> image::ImageResult<Vec<u8>> {
    let mut bytes = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut bytes, 80).encode(
        rgb,
        width,
        height,
        image::ExtendedColorType::Rgb8,
    )?;
    Ok(bytes)
}

type DecodedTile = (usize, Result<Vec<u8>, String>);
#[derive(Default)]
struct DecodeQueue {
    wanted: VecDeque<usize>,
    active: HashSet<usize>,
    stop: bool,
}
struct TileLoader {
    queue: Arc<(Mutex<DecodeQueue>, Condvar)>,
    ready: mpsc::Receiver<DecodedTile>,
}
impl TileLoader {
    fn new(directory: PathBuf, catalog: Catalog) -> Self {
        let queue = Arc::new((Mutex::new(DecodeQueue::default()), Condvar::new()));
        // Keep decoded CPU memory bounded even if the renderer stops polling.
        let (sender, ready) = mpsc::sync_channel(2);
        let worker_queue = queue.clone();
        std::thread::spawn(move || loop {
            let (lock, wake) = &*worker_queue;
            let id = {
                let mut q = lock.lock().unwrap();
                while q.wanted.is_empty() && !q.stop {
                    q = wake.wait(q).unwrap();
                }
                if q.stop {
                    break;
                }
                let id = q.wanted.pop_front().unwrap();
                q.active.insert(id);
                id
            };
            if sender
                .send((id, read_tile(&directory, &catalog, id)))
                .is_err()
            {
                break;
            }
        });
        Self { queue, ready }
    }
    fn request(&self, ids: impl Iterator<Item = usize>) {
        let (lock, wake) = &*self.queue;
        let mut q = lock.lock().unwrap();
        q.wanted = ids.filter(|id| !q.active.contains(id)).collect();
        wake.notify_one();
    }
    fn poll(&self) -> Option<DecodedTile> {
        let result = self.ready.try_recv().ok()?;
        self.queue.0.lock().unwrap().active.remove(&result.0);
        Some(result)
    }
}
impl Drop for TileLoader {
    fn drop(&mut self) {
        let (lock, wake) = &*self.queue;
        lock.lock().unwrap().stop = true;
        wake.notify_one();
    }
}

pub fn attribution_label(directory: &std::path::Path) -> Option<&'static str> {
    if !directory.join("aerial/tiles.txt").is_file() {
        return None;
    }
    let metadata = std::fs::read(directory.join("aerial/source.json"))
        .ok()
        .and_then(|s| serde_json::from_slice::<serde_json::Value>(&s).ok());
    Some(
        if metadata
            .as_ref()
            .and_then(|v| v["source"].as_str())
            .is_some_and(|s| s.contains("arcgisonline.com"))
        {
            "AERIAL: ESRI / VANTOR / EARTHSTAR / GIS COMMUNITY"
        } else if metadata
            .as_ref()
            .and_then(|v| v["provider"].as_str())
            .is_some_and(|s| s.contains("Cyprus"))
        {
            "AERIAL: CYPRUS DLS / 2019"
        } else {
            "AERIAL: LOCAL IMAGERY"
        },
    )
}
pub struct Aerial {
    loader: TileLoader,
    catalog: Catalog,
    texture: u32,
    slots: Vec<Option<usize>>,
    failed: HashSet<usize>,
}
impl Aerial {
    pub fn open(assets: &crate::city_assets::CityAsset) -> Option<Self> {
        let directory = assets.directory().join("aerial");
        let path = directory.join("tiles.txt");
        if !path.is_file() {
            return None;
        }
        let catalog = std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|s| Catalog::parse(&s));
        let catalog = match catalog {
            Ok(c) => c,
            Err(e) => {
                log::warn!("Aerial imagery disabled: {e}");
                return None;
            }
        };
        let capacity = capacity(&catalog);
        let dimension = (catalog.size + 2 * catalog.gutter) as i32;
        let mut texture = 0;
        unsafe {
            gl::GenTextures(1, &mut texture);
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D_ARRAY, texture);
            gl::TexImage3D(
                gl::TEXTURE_2D_ARRAY,
                0,
                gl::RGBA8 as i32,
                dimension,
                dimension,
                capacity as i32,
                0,
                gl::RGB,
                gl::UNSIGNED_BYTE,
                std::ptr::null(),
            );
            gl::TexParameteri(
                gl::TEXTURE_2D_ARRAY,
                gl::TEXTURE_MIN_FILTER,
                gl::LINEAR_MIPMAP_LINEAR as i32,
            );
            gl::TexParameteri(
                gl::TEXTURE_2D_ARRAY,
                gl::TEXTURE_MAG_FILTER,
                gl::NEAREST as i32,
            );
            for axis in [gl::TEXTURE_WRAP_S, gl::TEXTURE_WRAP_T] {
                gl::TexParameteri(gl::TEXTURE_2D_ARRAY, axis, gl::CLAMP_TO_EDGE as i32);
            }
            let mut extensions = 0;
            gl::GetIntegerv(gl::NUM_EXTENSIONS, &mut extensions);
            let supported = (0..extensions).any(|i| {
                let p = gl::GetStringi(gl::EXTENSIONS, i as u32);
                !p.is_null()
                    && matches!(
                        std::ffi::CStr::from_ptr(p.cast()).to_bytes(),
                        b"GL_EXT_texture_filter_anisotropic" | b"GL_ARB_texture_filter_anisotropic"
                    )
            });
            if supported {
                // Anisotropic filtering may override NEAREST magnification.
                // Keep it disabled for the nearest-neighbour comparison.
                gl::TexParameterf(gl::TEXTURE_2D_ARRAY, 0x84FE, 1.0);
            }
        }
        let loader = TileLoader::new(directory, catalog.clone());
        Some(Self {
            loader,
            catalog,
            texture,
            slots: vec![None; capacity],
            failed: HashSet::new(),
        })
    }
    pub fn bind(&mut self, program: u32, eye: XYZ) {
        let wanted = wanted(&self.catalog, eye, self.slots.len());
        for slot in &mut self.slots {
            if slot.is_some_and(|i| !wanted.contains(&i)) {
                *slot = None;
            }
        }
        let mut changed = false;
        unsafe {
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D_ARRAY, self.texture);
            gl::PixelStorei(gl::UNPACK_ALIGNMENT, 1);
        }
        // Upload at most two completed tiles per frame; disk I/O and decoding
        // run entirely on the worker. Obsolete requests are replaced each frame.
        for _ in 0..2 {
            let Some((id, result)) = self.loader.poll() else {
                break;
            };
            if !wanted.contains(&id) || self.slots.contains(&Some(id)) {
                continue;
            }
            let Some(slot) = self.slots.iter().position(Option::is_none) else {
                break;
            };
            match result {
                Ok(bytes) => {
                    let size = (self.catalog.size + 2 * self.catalog.gutter) as i32;
                    unsafe {
                        gl::TexSubImage3D(
                            gl::TEXTURE_2D_ARRAY,
                            0,
                            0,
                            0,
                            slot as i32,
                            size,
                            size,
                            1,
                            gl::RGB,
                            gl::UNSIGNED_BYTE,
                            bytes.as_ptr().cast(),
                        );
                    }
                    self.slots[slot] = Some(id);
                    changed = true;
                }
                Err(e) => {
                    log::warn!("Aerial tile unavailable; using procedural surface: {e}");
                    self.failed.insert(id);
                }
            }
        }
        self.loader.request(
            wanted
                .iter()
                .copied()
                .filter(|id| !self.slots.contains(&Some(*id)) && !self.failed.contains(id)),
        );
        unsafe {
            if changed {
                gl::GenerateMipmap(gl::TEXTURE_2D_ARRAY);
            }
            let location = |name: &str| {
                let name = CString::new(name).unwrap();
                gl::GetUniformLocation(program, name.as_ptr())
            };
            let mut bounds = Vec::<f32>::new();
            let mut layers = Vec::<i32>::new();
            for (slot, id) in self.slots.iter().enumerate() {
                if let Some(id) = id {
                    bounds.extend(self.catalog.tiles[*id].bounds.to_array());
                    layers.push(slot as i32);
                }
            }
            gl::Uniform1i(location("u_aerial_count"), layers.len() as i32);
            gl::Uniform1i(location("u_aerial"), 0);
            gl::Uniform2f(
                location("u_aerial_uv"),
                self.catalog.gutter as f32 / (self.catalog.size + 2 * self.catalog.gutter) as f32,
                self.catalog.size as f32 / (self.catalog.size + 2 * self.catalog.gutter) as f32,
            );
            if !layers.is_empty() {
                gl::Uniform4fv(
                    location("u_aerial_bounds[0]"),
                    layers.len() as i32,
                    bounds.as_ptr(),
                );
                gl::Uniform1iv(
                    location("u_aerial_layers[0]"),
                    layers.len() as i32,
                    layers.as_ptr(),
                );
            }
        }
    }
}
impl Drop for Aerial {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteTextures(1, &self.texture);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn two_imagery_workers_and_attributions_stay_with_their_asset_directories() {
        let root = tempfile::tempdir().unwrap();
        let first =
            crate::test_support::generated_location(&root.path().join("first"), 7.0, "Cyprus DLS");
        let second = crate::test_support::generated_location(
            &root.path().join("second"),
            13.0,
            "arcgisonline.com",
        );
        let mut workers = Vec::new();
        for (assets, expected) in [(&first, 7), (&second, 13)] {
            let directory = assets.directory().join("aerial");
            let catalog =
                Catalog::parse(&std::fs::read_to_string(directory.join("tiles.txt")).unwrap())
                    .unwrap();
            let loader = TileLoader::new(directory, catalog);
            loader.request(std::iter::once(0));
            workers.push((loader, expected));
        }
        for (loader, expected) in workers {
            let (id, pixels) = loader
                .ready
                .recv_timeout(std::time::Duration::from_secs(3))
                .unwrap();
            assert_eq!(id, 0);
            let pixels = pixels.unwrap();
            assert_eq!(pixels.len(), 66 * 66 * 3);
            assert!(pixels.iter().all(|&value| value == expected));
        }
        assert!(first.aerial_attribution().unwrap().contains("CYPRUS DLS"));
        assert!(second.aerial_attribution().unwrap().contains("ESRI"));
    }

    #[test]
    fn catalog_validation_and_nearby_selection_are_bounded() {
        let c=Catalog::parse("AERIAL 1\nsize 512 8\ntile 0 0 100 100 a.rgb 00000000\ntile 100 0 200 100 b.rgb 00000000\n").unwrap();
        assert_eq!(
            wanted(
                &c,
                crate::geometry::XYZ {
                    x: 150.0,
                    y: 0.0,
                    z: 50.0
                },
                1
            ),
            vec![1]
        );
        assert!(capacity(&c) * layer_bytes(c.size + 2 * c.gutter) <= GPU_BUDGET);
        assert!(wanted(
            &c,
            crate::geometry::XYZ {
                x: 2000.0,
                y: 0.0,
                z: 0.0
            },
            32
        )
        .is_empty());
        assert!(
            Catalog::parse("AERIAL 1\nsize 512 8\ntile 0 0 100 100 ../a.rgb 00000000\n").is_err()
        );
        assert!(
            Catalog::parse("AERIAL 1\nsize 512 8\ntile NaN 0 100 100 a.rgb 00000000\n").is_err()
        );
        assert!(read_tile(Path::new("/missing-aerial-test"), &c, 0).is_err());
    }
    #[test]
    fn city_tiles_fit_nearby_coverage_within_the_gpu_budget() {
        let mut text = String::from("AERIAL 1\nsize 512 8\n");
        for z in -6..=6 {
            for x in -6..=6 {
                text.push_str(&format!(
                    "tile {} {} {} {} {x}_{z}.rgb 00000000\n",
                    x as f32 * 249.,
                    z as f32 * 249.,
                    (x + 1) as f32 * 249.,
                    (z + 1) as f32 * 249.
                ));
            }
        }
        let catalog = Catalog::parse(&text).unwrap();
        let slots = capacity(&catalog);
        for x in [0., 62., 124., 248.] {
            for z in [0., 62., 124., 248.] {
                let all = wanted(
                    &catalog,
                    crate::geometry::XYZ { x: x, y: 0., z: z },
                    usize::MAX,
                );
                assert!(
                    all.len() <= slots,
                    "nearby city coverage requires {} slots, only {slots} available",
                    all.len()
                );
            }
        }
        assert!(slots * layer_bytes(catalog.size + 2 * catalog.gutter) <= GPU_BUDGET);
    }
    #[test]
    fn jpeg_worker_decodes_validates_dimensions_and_reports_errors() {
        let dir = tempfile::tempdir().unwrap();
        let bytes = encode_jpeg(&vec![127; 66 * 66 * 3], 66, 66).unwrap();
        std::fs::write(dir.path().join("a.jpg"), &bytes).unwrap();
        let catalog = Catalog::parse(&format!(
            "AERIAL 1\nsize 64 1\ntile 0 0 10 10 a.jpg {:08x}\ntile 10 0 20 10 missing.jpg 00000000\n",
            0
        )).unwrap();
        let loader = TileLoader::new(dir.path().to_path_buf(), catalog.clone());
        loader.request([0, 1].into_iter());
        let (id, result) = loader
            .ready
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        assert_eq!(id, 0);
        assert!(result.unwrap().iter().all(|v| v.abs_diff(127) <= 2));
        let (id, result) = loader
            .ready
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        assert_eq!(id, 1);
        assert!(result.is_err());
        let mut wrong_size = catalog.clone();
        wrong_size.gutter = 2;
        assert!(read_tile(dir.path(), &wrong_size, 0).is_err());
        std::fs::write(dir.path().join("a.jpg"), &bytes[..30]).unwrap();
        assert!(read_tile(dir.path(), &catalog, 0).is_err());
    }
    #[test]
    fn raw_tile_payload_accepts_changes_and_rejects_truncation() {
        let path = std::env::temp_dir().join(format!(
            "aerial-tile-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir(&path).unwrap();
        let bytes = vec![127u8; 66 * 66 * 3];
        let c = Catalog::parse(&format!(
            "AERIAL 1\nsize 64 1\ntile 0 0 10 10 a.rgb {:08x}\n",
            0
        ))
        .unwrap();
        std::fs::write(path.join("a.rgb"), &bytes).unwrap();
        assert_eq!(read_tile(&path, &c, 0).unwrap(), bytes);
        let mut damaged = bytes.clone();
        damaged[15] ^= 1;
        std::fs::write(path.join("a.rgb"), &damaged).unwrap();
        assert_eq!(read_tile(&path, &c, 0).unwrap(), damaged);
        std::fs::write(path.join("a.rgb"), &bytes[..30]).unwrap();
        assert!(read_tile(&path, &c, 0).is_err());
        std::fs::remove_dir_all(path).unwrap();
    }
}
