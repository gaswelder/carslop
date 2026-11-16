//! Aerial catalog validation, tile decoding, and source attribution.

use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XZ;
use std::{collections::HashSet, path::Path};

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
pub fn tile_filenames(text: &str) -> Result<Vec<String>, String> {
    parse(text).map(|catalog| catalog.tiles.into_iter().map(|tile| tile.name).collect())
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
