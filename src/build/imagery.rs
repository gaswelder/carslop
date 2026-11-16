use crate::build;
use crate::util;
use crate::util::error::Result;
use crate::util::geometry::XZ;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub const SERVICE: &str =
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer";

const HALF_WORLD: f64 = std::f64::consts::PI * 6378137.;
const BLOCK_ZOOM: u32 = 15;
const TILE_ZOOM: u32 = 17;
pub const RES: f64 = 2. * HALF_WORLD / (256. * (1_u32 << 18) as f64); // zoom-18 Web Mercator sampling, ~0.49 m on the ground here
pub const CORE: u32 = 2048;
pub const PAD: u32 = 32;
pub const TILE: u32 = 512;
pub const GUTTER: u32 = 8;

/// One aerial imagery processing block, including its padded geographic coverage.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Block {
    /// Global XYZ column index, increasing eastward.
    pub x: usize,
    /// Global XYZ row index, increasing southward.
    pub y: usize,
    /// Padded [west, south, east, north] bounds in Web Mercator metres.
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
    let columns = (1_usize << BLOCK_ZOOM) as f64;
    let west = ((sw[0] + HALF_WORLD) / span).floor().clamp(0., columns) as usize;
    let east = ((ne[0] + HALF_WORLD) / span).ceil().clamp(0., columns) as usize;
    let north = ((HALF_WORLD - ne[1]) / span).floor().clamp(0., columns) as usize;
    let south = ((HALF_WORLD - sw[1]) / span).ceil().clamp(0., columns) as usize;
    (north..south)
        .flat_map(|y| {
            (west..east).map(move |x| {
                let west = -HALF_WORLD + x as f64 * span;
                let north = HALF_WORLD - y as f64 * span;
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
    work.join(format!("{}.jpg", block_name(b)))
}

fn block_name(b: &Block) -> String {
    format!("{BLOCK_ZOOM}_{}_{}", b.x, b.y)
}

fn tile_name(b: &Block, x: u32, y: u32) -> String {
    let across = (CORE / TILE) as usize;
    format!(
        "{TILE_ZOOM}_{}_{}.jpg",
        b.x * across + x as usize,
        b.y * across + y as usize
    )
}

pub fn local(origin: [f64; 2], ll: [f64; 2]) -> XZ {
    XZ {
        x: (ll[0] - origin[1]).to_radians() * 6371000. * origin[0].to_radians().cos(),
        z: -(ll[1] - origin[0]).to_radians() * 6371000.,
    }
}
fn blank(im: &image::RgbImage, x: u32, y: u32) -> bool {
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
/// Persisted block metadata uses the existing JSON keys and tree tuple arrays.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct TileMetadata {
    pub name: String,
    pub bounds: [f64; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "[f64; 3]", into = "[f64; 3]")]
struct CrownProposal {
    longitude: f64,
    latitude: f64,
    radius: f64,
}
impl From<[f64; 3]> for CrownProposal {
    fn from([longitude, latitude, radius]: [f64; 3]) -> Self {
        Self {
            longitude,
            latitude,
            radius,
        }
    }
}
impl From<CrownProposal> for [f64; 3] {
    fn from(p: CrownProposal) -> Self {
        [p.longitude, p.latitude, p.radius]
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct BlockMetadata {
    #[serde(default)]
    jpeg_quality: u8,
    extent: [f64; 4],
    pub tiles: Vec<TileMetadata>,
    missing_tiles: usize,
    trees: Vec<CrownProposal>,
}

/// Compile renderer tiles without running tree inference.
pub fn process_tiles(
    downloads: &Path,
    work: &Path,
    aerial: &Path,
    b: &Block,
    origin: [f64; 2],
) -> Result<BlockMetadata> {
    fs::create_dir_all(work)?;
    let meta_path = work.join(format!("block-{}.json", block_name(b)));
    let image_file = image_path(downloads, b);
    if meta_path.exists() {
        let value: BlockMetadata = serde_json::from_slice(&fs::read(&meta_path)?)?;
        if value.extent == b.extent
            && value.jpeg_quality == 80
            && value
                .tiles
                .iter()
                .all(|t| t.name.ends_with(".jpg") && aerial.join(&t.name).is_file())
        {
            return Ok(value);
        }
    }
    let im = image::open(&image_file)?.to_rgb8();
    let side = CORE + 2 * PAD;
    let geo = build::tree_proposals::Geo {
        extent: b.extent,
        width: side as usize,
        height: side as usize,
    };
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
            let raw = encode_jpeg(&raw, TILE + 2 * GUTTER, TILE + 2 * GUTTER)?;
            let name = tile_name(b, x, z);
            util::file_io::write_file(&aerial.join(&name), &raw)?;
            let nw = local(
                origin,
                build::tree_proposals::lonlat(geo, px as f64, py as f64),
            );
            let se = local(
                origin,
                build::tree_proposals::lonlat(geo, (px + TILE) as f64, (py + TILE) as f64),
            );
            tiles.push(TileMetadata {
                name,
                bounds: [nw.x, nw.z, se.x, se.z],
            });
        }
    }
    let value = BlockMetadata {
        jpeg_quality: 80,
        extent: b.extent,
        tiles,
        missing_tiles: missing,
        trees: vec![],
    };
    util::file_io::write_file(&meta_path, &serde_json::to_vec(&value)?)?;
    Ok(value)
}

fn encode_jpeg(rgb: &[u8], width: u32, height: u32) -> image::ImageResult<Vec<u8>> {
    let mut bytes = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut bytes, 80).encode(
        rgb,
        width,
        height,
        image::ExtendedColorType::Rgb8,
    )?;
    Ok(bytes)
}

#[cfg(test)]
#[path = "tests/imagery.rs"]
mod tests;
