//! Optional disk-backed aerial tiles, projected onto existing surface geometry.
use std::{
    collections::HashSet,
    ffi::CString,
    path::{Path, PathBuf},
};
const CAPACITY: usize = 32;
const GPU_BUDGET: usize = 32 * 1024 * 1024;
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
#[derive(Debug)]
struct Tile {
    bounds: [f32; 4],
    name: String,
    crc: u32,
}
#[derive(Debug)]
struct Catalog {
    size: usize,
    gutter: usize,
    tiles: Vec<Tile>,
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
            if f.len() != 7 || f[0] != "tile" {
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
                bounds,
                name: f[5].into(),
                crc: u32::from_str_radix(f[6], 16).map_err(|_| "invalid aerial checksum")?,
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
fn distance(bounds: [f32; 4], eye: [f32; 3]) -> f32 {
    (eye[0] - eye[0].clamp(bounds[0], bounds[2])).hypot(eye[2] - eye[2].clamp(bounds[1], bounds[3]))
}
fn wanted(catalog: &Catalog, eye: [f32; 3], capacity: usize) -> Vec<usize> {
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
    let bytes = std::fs::read(directory.join(&tile.name)).map_err(|e| e.to_string())?;
    if bytes.len() != (catalog.size + catalog.gutter * 2).pow(2) * 3
        || crate::street_surface::crc32_bytes(&bytes) != tile.crc
    {
        return Err(format!("corrupt aerial tile {}", tile.name));
    }
    Ok(bytes)
}
pub fn source_available() -> bool {
    std::env::var_os("NFS_NO_AERIAL").is_none()
        && std::env::var_os("NFS_AERIAL_DATA")
            .map(PathBuf::from)
            .unwrap_or_else(|| crate::city_assets::directory().join("aerial"))
            .join("tiles.txt")
            .exists()
}
pub struct Aerial {
    directory: PathBuf,
    catalog: Catalog,
    texture: u32,
    slots: Vec<Option<usize>>,
    failed: HashSet<usize>,
    #[cfg(test)]
    reads: usize,
}
impl Aerial {
    pub fn open() -> Option<Self> {
        if std::env::var_os("NFS_NO_AERIAL").is_some() {
            return None;
        }
        let directory = std::env::var_os("NFS_AERIAL_DATA")
            .map(PathBuf::from)
            .unwrap_or_else(|| crate::city_assets::directory().join("aerial"));
        let path = directory.join("tiles.txt");
        if !path.exists() {
            return None;
        }
        let catalog = std::fs::read_to_string(path)
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
                gl::LINEAR as i32,
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
                let mut maximum = 1.0;
                gl::GetFloatv(0x84FF, &mut maximum);
                gl::TexParameterf(gl::TEXTURE_2D_ARRAY, 0x84FE, maximum.min(16.0));
            }
        }
        Some(Self {
            directory,
            catalog,
            texture,
            slots: vec![None; capacity],
            failed: HashSet::new(),
            #[cfg(test)]
            reads: 0,
        })
    }
    pub fn bind(&mut self, program: u32, eye: [f32; 3]) {
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
        for id in wanted {
            if self.slots.contains(&Some(id)) || self.failed.contains(&id) {
                continue;
            }
            let Some(slot) = self.slots.iter().position(Option::is_none) else {
                break;
            };
            match read_tile(&self.directory, &self.catalog, id) {
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
                    #[cfg(test)]
                    {
                        self.reads += 1;
                    }
                }
                Err(e) => {
                    log::warn!("Aerial tile unavailable; using procedural surface: {e}");
                    self.failed.insert(id);
                }
            }
        }
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
                    bounds.extend(self.catalog.tiles[*id].bounds);
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
    #[cfg(test)]
    pub fn stats(&self) -> (usize, usize) {
        (self.slots.iter().flatten().count(), self.reads)
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
    fn catalog_validation_and_nearby_selection_are_bounded() {
        let c=Catalog::parse("AERIAL 1\nsize 512 8\ntile 0 0 100 100 a.rgb 00000000\ntile 100 0 200 100 b.rgb 00000000\n").unwrap();
        assert_eq!(wanted(&c, [150.0, 0.0, 50.0], 1), vec![1]);
        assert!(capacity(&c) * layer_bytes(c.size + 2 * c.gutter) <= GPU_BUDGET);
        assert!(wanted(&c, [2000.0, 0.0, 0.0], 32).is_empty());
        assert!(
            Catalog::parse("AERIAL 1\nsize 512 8\ntile 0 0 100 100 ../a.rgb 00000000\n").is_err()
        );
        assert!(
            Catalog::parse("AERIAL 1\nsize 512 8\ntile NaN 0 100 100 a.rgb 00000000\n").is_err()
        );
        assert!(read_tile(Path::new("/missing-aerial-test"), &c, 0).is_err());
    }
    #[test]
    fn tile_payload_rejects_corruption_and_truncation() {
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
            crate::street_surface::crc32_bytes(&bytes)
        ))
        .unwrap();
        std::fs::write(path.join("a.rgb"), &bytes).unwrap();
        assert_eq!(read_tile(&path, &c, 0).unwrap(), bytes);
        let mut damaged = bytes.clone();
        damaged[15] ^= 1;
        std::fs::write(path.join("a.rgb"), &damaged).unwrap();
        assert!(read_tile(&path, &c, 0).is_err());
        std::fs::write(path.join("a.rgb"), &bytes[..30]).unwrap();
        assert!(read_tile(&path, &c, 0).is_err());
        std::fs::remove_dir_all(path).unwrap();
    }
}
