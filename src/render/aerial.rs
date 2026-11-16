//! Optional disk-backed aerial tiles, projected onto existing surface geometry.

use crate::storage;
use crate::storage::aerial::Catalog;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XYZ;
use std::{
    collections::{HashSet, VecDeque},
    ffi::CString,
    path::PathBuf,
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

fn new(directory: PathBuf, catalog: Catalog) -> TileLoader {
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
            .send((id, storage::aerial::read_tile(&directory, &catalog, id)))
            .is_err()
        {
            break;
        }
    });
    TileLoader { queue, ready }
}
fn request(tile_loader: &TileLoader, ids: impl Iterator<Item = usize>) {
    let (lock, wake) = &*tile_loader.queue;
    let mut q = lock.lock().unwrap();
    q.wanted = ids.filter(|id| !q.active.contains(id)).collect();
    wake.notify_one();
}
fn poll(tile_loader: &TileLoader) -> Option<DecodedTile> {
    let result = tile_loader.ready.try_recv().ok()?;
    tile_loader.queue.0.lock().unwrap().active.remove(&result.0);
    Some(result)
}

impl Drop for TileLoader {
    fn drop(&mut self) {
        let (lock, wake) = &*self.queue;
        lock.lock().unwrap().stop = true;
        wake.notify_one();
    }
}

pub struct Aerial {
    loader: TileLoader,
    catalog: Catalog,
    texture: u32,
    slots: Vec<Option<usize>>,
    failed: HashSet<usize>,
}

pub fn open(assets: &storage::city_assets::CityAsset) -> Option<Aerial> {
    let directory = storage::city_assets::directory(assets).join("aerial");
    let path = directory.join("tiles.txt");
    if !path.is_file() {
        return None;
    }
    let catalog = std::fs::read_to_string(&path)
        .map_err(|e| e.to_string())
        .and_then(|s| storage::aerial::parse(&s));
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
    let loader = new(directory, catalog.clone());
    Some(Aerial {
        loader,
        catalog,
        texture,
        slots: vec![None; capacity],
        failed: HashSet::new(),
    })
}
pub fn bind(aerial: &mut Aerial, program: u32, eye: XYZ) {
    let wanted = wanted(&aerial.catalog, eye, aerial.slots.len());
    for slot in &mut aerial.slots {
        if slot.is_some_and(|i| !wanted.contains(&i)) {
            *slot = None;
        }
    }
    let mut changed = false;
    unsafe {
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D_ARRAY, aerial.texture);
        gl::PixelStorei(gl::UNPACK_ALIGNMENT, 1);
    }
    // Upload at most two completed tiles per frame; disk I/O and decoding
    // run entirely on the worker. Obsolete requests are replaced each frame.
    for _ in 0..2 {
        let Some((id, result)) = poll(&aerial.loader) else {
            break;
        };
        if !wanted.contains(&id) || aerial.slots.contains(&Some(id)) {
            continue;
        }
        let Some(slot) = aerial.slots.iter().position(Option::is_none) else {
            break;
        };
        match result {
            Ok(bytes) => {
                let size = (aerial.catalog.size + 2 * aerial.catalog.gutter) as i32;
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
                aerial.slots[slot] = Some(id);
                changed = true;
            }
            Err(e) => {
                log::warn!("Aerial tile unavailable; using procedural surface: {e}");
                aerial.failed.insert(id);
            }
        }
    }
    request(
        &aerial.loader,
        wanted
            .iter()
            .copied()
            .filter(|id| !aerial.slots.contains(&Some(*id)) && !aerial.failed.contains(id)),
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
        for (slot, id) in aerial.slots.iter().enumerate() {
            if let Some(id) = id {
                bounds.extend(util::geometry::bounding_box_xz_to_array(
                    aerial.catalog.tiles[*id].bounds,
                ));
                layers.push(slot as i32);
            }
        }
        gl::Uniform1i(location("u_aerial_count"), layers.len() as i32);
        gl::Uniform1i(location("u_aerial"), 0);
        gl::Uniform2f(
            location("u_aerial_uv"),
            aerial.catalog.gutter as f32 / (aerial.catalog.size + 2 * aerial.catalog.gutter) as f32,
            aerial.catalog.size as f32 / (aerial.catalog.size + 2 * aerial.catalog.gutter) as f32,
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

impl Drop for Aerial {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteTextures(1, &self.texture);
        }
    }
}

#[cfg(test)]
#[path = "tests/aerial.rs"]
mod tests;
