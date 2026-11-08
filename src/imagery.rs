//! Resumable city-wide aerial preparation. Downloads are explicit; gameplay is offline.
use crate::location::{invalid, Result};
use crate::tree_preview::{proposals, Geo};
use serde_json::{json, Value};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

pub const SERVICE: &str =
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer";

pub const RES: f64 = 0.5971642835598172; // zoom-18 Web Mercator sampling, ~0.49 m on the ground here
pub const CORE: u32 = 2048;
pub const PAD: u32 = 32;
pub const TILE: u32 = 512;
pub const GUTTER: u32 = 8;

#[derive(Clone, Debug)]
pub struct Block {
    pub x: usize,
    pub y: usize,
    pub extent: [f64; 4],
}
pub fn mercator(lon: f64, lat: f64) -> [f64; 2] {
    [
        lon.to_radians() * 6378137.,
        (std::f64::consts::FRAC_PI_4 + lat.to_radians() / 2.)
            .tan()
            .ln()
            * 6378137.,
    ]
}
pub fn blocks(bbox: [f64; 4]) -> Vec<Block> {
    let sw = mercator(bbox[1], bbox[0]);
    let ne = mercator(bbox[3], bbox[2]);
    let span = CORE as f64 * RES;
    let nx = ((ne[0] - sw[0]) / span).ceil() as usize;
    let ny = ((ne[1] - sw[1]) / span).ceil() as usize;
    (0..ny)
        .flat_map(|y| {
            (0..nx).map(move |x| {
                let west = sw[0] + x as f64 * span;
                let north = ne[1] - y as f64 * span;
                Block {
                    x,
                    y,
                    extent: [
                        west - PAD as f64 * RES,
                        north - span - PAD as f64 * RES,
                        west + span + PAD as f64 * RES,
                        north + PAD as f64 * RES,
                    ],
                }
            })
        })
        .collect()
}
pub fn image_path(work: &Path, b: &Block) -> PathBuf {
    work.join(format!("{}_{}.jpg", b.x, b.y))
}
pub fn city_origin(text: &str) -> Result<[f64; 2]> {
    let line = text
        .lines()
        .find(|l| l.starts_with("origin "))
        .ok_or_else(|| invalid("missing city origin"))?;
    let v: Vec<_> = line.split_whitespace().collect();
    Ok([v[1].parse()?, v[2].parse()?])
}
pub fn local(origin: [f64; 2], ll: [f64; 2]) -> [f64; 2] {
    [
        (ll[0] - origin[1]).to_radians() * 6371000. * origin[0].to_radians().cos(),
        -(ll[1] - origin[0]).to_radians() * 6371000.,
    ]
}
pub fn blank(im: &image::RgbImage, x: u32, y: u32) -> bool {
    // Blank service coverage is white; do not turn that into white terrain.
    let mut nonwhite = 0;
    let mut n = 0;
    for yy in (y..y + TILE).step_by(16) {
        for xx in (x..x + TILE).step_by(16) {
            let p = im.get_pixel(xx, yy).0;
            if p.iter().any(|v| *v < 245) {
                nonwhite += 1;
            }
            n += 1;
        }
    }
    nonwhite * 100 < n
}
pub fn process(
    downloads: &Path,
    work: &Path,
    aerial: &Path,
    b: &Block,
    origin: [f64; 2],
) -> Result<Value> {
    fs::create_dir_all(work)?;
    let meta_path = work.join(format!("block-{}_{}.json", b.x, b.y));
    let image_file = image_path(downloads, b);
    let hash = crate::location::sha(&image_file)?;
    if meta_path.exists() {
        let value: Value = serde_json::from_slice(&fs::read(&meta_path)?)?;
        if value["image_sha256"] == hash
            && value["jpeg_quality"] == 80
            && value["tiles"].as_array().is_some_and(|tiles| {
                tiles.iter().all(|t| {
                    let Some(name) = t["name"].as_str() else {
                        return false;
                    };
                    let Ok(bytes) = fs::read(aerial.join(name)) else {
                        return false;
                    };
                    name.ends_with(".jpg")
                        && t["crc"].as_u64() == Some(crc32fast::hash(&bytes) as u64)
                })
            })
        {
            return Ok(value);
        }
    }
    let im = image::open(&image_file)?.to_rgb8();
    let side = CORE + 2 * PAD;
    let geo = Geo {
        extent: b.extent,
        width: side as usize,
        height: side as usize,
    };
    let scale = geo.metres_per_pixel();
    let mut tiles = vec![];
    let mut missing = 0;
    for z in 0..CORE / TILE {
        for x in 0..CORE / TILE {
            let px = PAD + x * TILE;
            let py = PAD + z * TILE;
            if blank(&im, px, py) {
                missing += 1;
                continue;
            }
            let raw = image::imageops::crop_imm(
                &im,
                px - GUTTER,
                py - GUTTER,
                TILE + 2 * GUTTER,
                TILE + 2 * GUTTER,
            )
            .to_image()
            .into_raw();
            let raw = crate::aerial::encode_jpeg(&raw, TILE + 2 * GUTTER, TILE + 2 * GUTTER)?;
            let name = format!("{}_{}_{}_{}.jpg", b.x, b.y, x, z);
            write_file(&aerial.join(&name), &raw)?;
            let nw = local(origin, geo.lonlat(px as f64, py as f64));
            let se = local(origin, geo.lonlat((px + TILE) as f64, (py + TILE) as f64));
            tiles.push(
                json!({"name":name,"bounds":[nw[0],nw[1],se[0],se[1]],"crc":crc32fast::hash(&raw)}),
            );
        }
    }
    let crowns = proposals(&im, scale);
    let trees: Vec<_> = crowns
        .into_iter()
        .filter(|p| {
            p.x >= PAD as usize
                && p.y >= PAD as usize
                && p.x < (PAD + CORE) as usize
                && p.y < (PAD + CORE) as usize
        })
        .map(|p| {
            let ll = geo.lonlat(p.x as f64, p.y as f64);
            [ll[0], ll[1], p.radius]
        })
        .collect();
    let value = json!({"jpeg_quality":80,"image_sha256":hash,"extent":b.extent,"tiles":tiles,"missing_tiles":missing,"trees":trees});
    write_file(&meta_path, &serde_json::to_vec(&value)?)?;
    Ok(value)
}

pub(crate) fn write_file(path: &Path, data: &[u8]) -> Result<()> {
    let mut file = tempfile::NamedTempFile::new_in(path.parent().unwrap())?;
    file.write_all(data)?;
    file.as_file().sync_all()?;
    file.persist(path)?;
    Ok(())
}
