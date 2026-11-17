use crate::storage;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XZ;
use std::{
    collections::HashSet,
    path::{Path, PathBuf},
};

const FILE_NAME: &str = "aerial.txt";

#[derive(Clone, Debug)]
pub struct Tile {
    pub bounds: BoundingBoxXZ<f32>,
    name: String,
}

#[derive(Clone, Debug)]
pub struct Catalog {
    pub size: usize,
    pub gutter: usize,
    pub tiles: Vec<Tile>,
}
pub fn parse(text: &str) -> Result<Catalog, String> {
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
        if !bounds.iter().all(|v| v.is_finite()) || bounds[0] >= bounds[2] || bounds[1] >= bounds[3]
        {
            return Err("invalid aerial bounds".into());
        }
        if f[5].contains(['/', '\\']) || f[5].starts_with('.') || !names.insert(f[5].to_string()) {
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
    Ok(Catalog {
        size,
        gutter,
        tiles,
    })
}

pub fn read_tile(directory: &Path, catalog: &Catalog, id: usize) -> Result<Vec<u8>, String> {
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

pub fn attribution_label(directory: &Path) -> Option<String> {
    if !directory.join(FILE_NAME).is_file() {
        return None;
    }
    let attribution = storage::manifest::read(directory)
        .ok()
        .and_then(|manifest| manifest.aerial_attribution);
    Some(attribution.unwrap_or_else(|| "AERIAL: LOCAL IMAGERY".into()))
}

pub struct TileRecord {
    pub bounds: [f64; 4],
    pub name: String,
}

pub fn write(
    directory: &Path,
    size: u32,
    gutter: u32,
    tiles: &[TileRecord],
) -> std::io::Result<()> {
    use std::io::Write;
    let mut output = std::io::BufWriter::new(std::fs::File::create(directory.join(FILE_NAME))?);
    writeln!(output, "AERIAL 1\nsize {size} {gutter}")?;
    for tile in tiles {
        let [x1, y1, x2, y2] = tile.bounds;
        writeln!(output, "tile {x1:.6} {y1:.6} {x2:.6} {y2:.6} {}", tile.name)?;
    }
    output.flush()
}

pub fn read(directory: &Path) -> Result<Option<Catalog>, String> {
    match std::fs::read_to_string(directory.join(FILE_NAME)) {
        Ok(text) => parse(&text).map(Some),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e.to_string()),
    }
}

fn payload_files(catalog: Catalog) -> Vec<PathBuf> {
    let mut files = vec![PathBuf::from(FILE_NAME)];
    files.extend(
        catalog
            .tiles
            .into_iter()
            .map(|tile| PathBuf::from("aerial").join(tile.name)),
    );
    files.sort();
    files.dedup();
    files
}

pub fn read_payload_files(directory: &Path) -> Result<Vec<PathBuf>, String> {
    if !directory.join(FILE_NAME).is_file() {
        return Ok(Vec::new());
    }
    read(directory).map(|catalog| catalog.map(payload_files).unwrap_or_default())
}

pub fn remove_stale_payloads(directory: &Path, retained: &[PathBuf]) -> Result<(), String> {
    let previous = read(directory)
        .ok()
        .flatten()
        .map(payload_files)
        .unwrap_or_else(|| vec![FILE_NAME.into()]);
    for relative in previous {
        if retained.contains(&relative) {
            continue;
        }
        match std::fs::remove_file(directory.join(&relative)) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(format!("{}: {e}", relative.display())),
        }
    }
    Ok(())
}
