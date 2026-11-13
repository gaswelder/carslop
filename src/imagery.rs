//! Resumable city-wide aerial preparation. Downloads are explicit; gameplay is offline.
use crate::aerial;
use crate::error::Result;
use crate::file_io;
use crate::geometry::XZ;
use crate::tree_preview;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub const SERVICE: &str =
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer";

pub const RES: f64 = 0.5971642835598172; // zoom-18 Web Mercator sampling, ~0.49 m on the ground here
pub const CORE: u32 = 2048;
pub const PAD: u32 = 32;
pub const TILE: u32 = 512;
pub const GUTTER: u32 = 8;

/// One aerial imagery processing block, including its padded geographic coverage.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Block {
    /// Block column index, increasing eastward.
    pub x: usize,
    /// Block row index, increasing southward.
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
    let meta_path = work.join(format!("block-{}_{}.json", b.x, b.y));
    let image_file = image_path(downloads, b);
    if meta_path.exists() {
        let value: BlockMetadata = serde_json::from_slice(&fs::read(&meta_path)?)?;
        if value.jpeg_quality == 80
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
    let geo = tree_preview::Geo {
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
            let raw = aerial::encode_jpeg(&raw, TILE + 2 * GUTTER, TILE + 2 * GUTTER)?;
            let name = format!("{}_{}_{}_{}.jpg", b.x, b.y, x, z);
            file_io::write_file(&aerial.join(&name), &raw)?;
            let nw = local(origin, geo.lonlat(px as f64, py as f64));
            let se = local(origin, geo.lonlat((px + TILE) as f64, (py + TILE) as f64));
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
    file_io::write_file(&meta_path, &serde_json::to_vec(&value)?)?;
    Ok(value)
}

#[cfg(test)]
mod metadata_tests {
    use super::*;

    #[test]
    fn typed_metadata_round_trips_existing_json_and_tree_arrays() {
        let legacy = serde_json::json!({
            "jpeg_quality": 80, "extent": [1., 2., 3., 4.],
            "tiles": [{"name": "0_0_0_0.jpg", "bounds": [-2., -1., 2., 1.]}],
            "missing_tiles": 15, "trees": [[33.123456789, 35.987654321, 2.25]],
        });
        let meta: BlockMetadata = serde_json::from_value(legacy.clone()).unwrap();
        assert_eq!(meta.tiles[0].bounds, [-2., -1., 2., 1.]);
        assert_eq!(meta.trees[0].longitude, 33.123456789);
        assert_eq!(meta.trees[0].radius, 2.25);
        assert_eq!(serde_json::to_value(&meta).unwrap(), legacy);
        let bytes = serde_json::to_vec(&meta).unwrap();
        assert_eq!(
            serde_json::from_slice::<BlockMetadata>(&bytes).unwrap(),
            meta
        );
        let mut old_quality = legacy.clone();
        old_quality.as_object_mut().unwrap().remove("jpeg_quality");
        assert_eq!(
            serde_json::from_value::<BlockMetadata>(old_quality)
                .unwrap()
                .jpeg_quality,
            0
        );
        for bad in [
            serde_json::json!([1., 2.]),
            serde_json::json!([1., "bad", 3.]),
        ] {
            let mut invalid = legacy.clone();
            invalid["trees"] = serde_json::json!([bad]);
            assert!(serde_json::from_value::<BlockMetadata>(invalid).is_err());
        }
    }
}

#[cfg(test)]
mod raster_tests {
    use super::*;
    use std::fs;
    #[test]
    fn block_resume_reuses_existing_tiles_and_preserves_gutters() {
        let temp = tempfile::tempdir().unwrap();
        let work = temp.path();
        fs::create_dir(work.join("downloads")).unwrap();
        let aerial = work.join("aerial");
        fs::create_dir(&aerial).unwrap();
        let b = blocks([35.16, 33.31, 35.161, 33.311]).remove(0);
        let side = CORE + 2 * PAD;
        let im = image::RgbImage::from_fn(side, side, |x, y| {
            image::Rgb([(40 + x % 128) as u8, (40 + y % 128) as u8, 80])
        });
        im.save(image_path(&work.join("downloads"), &b)).unwrap();
        let first =
            process_tiles(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap();
        assert_eq!(first.tiles.len(), 16);
        let a = fs::read(aerial.join("0_0_0_0.jpg")).unwrap();
        let decoded = image::load_from_memory(&a).unwrap().to_rgb8().into_raw();
        let right = image::open(aerial.join("0_0_1_0.jpg"))
            .unwrap()
            .to_rgb8()
            .into_raw();
        let width = (TILE + 2 * GUTTER) as usize;
        for y in 0..width {
            let left = &decoded[(y * width + TILE as usize) * 3..(y + 1) * width * 3];
            let right = &right[y * width * 3..(y * width + 2 * GUTTER as usize) * 3];
            assert!(left.iter().zip(right).all(|(a, b)| a.abs_diff(*b) <= 12));
        }
        assert_eq!(
            first,
            process_tiles(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
        );
        fs::remove_file(aerial.join("0_0_0_0.jpg")).unwrap();
        assert_eq!(
            first,
            process_tiles(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
        );
        assert_eq!(a, fs::read(aerial.join("0_0_0_0.jpg")).unwrap());
    }
}
