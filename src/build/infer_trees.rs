//! Infer tree positions and sizes from the downloaded satellite images.

use crate::build;
use crate::util::error::Result;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

pub const TREES_FILE: &str = "trees.json";

/// Inferred tree data uses geographic positions and sizes in metres.
#[derive(Serialize, Deserialize)]
pub struct InferredTree {
    pub longitude: f64,
    pub latitude: f64,
    pub crown_radius_meters: f64,
    pub height_meters: f64,
}

/// Supplement the downloaded sources with geographic canopy positions and radii.
pub fn add_trees(download_dir: &Path) -> Result<()> {
    println!("extracting trees from satellite images");
    let data = build::download_location::read_manifest(download_dir)?;
    let mut trees = Vec::new();
    for block in &data.imagery_blocks {
        trees.extend(infer_trees(
            &download_dir.join(build::download_location::SATELLITE_IMAGES_DIR),
            block,
        )?);
    }
    fs::write(download_dir.join(TREES_FILE), serde_json::to_vec(&trees)?)?;
    Ok(())
}

/// Infer geographic canopy positions and radii from one downloaded image.
fn infer_trees(downloads: &Path, b: &build::imagery::Block) -> Result<Vec<InferredTree>> {
    let im = image::open(build::imagery::image_path(downloads, b))?.to_rgb8();
    let geo = build::tree_proposals::Geo {
        extent: b.extent,
        width: im.width() as usize,
        height: im.height() as usize,
    };
    let scale = build::tree_proposals::metres_per_pixel(geo);
    let crowns = build::tree_proposals::proposals(&im, scale);
    Ok(crowns
        .into_iter()
        .filter(|p| {
            p.x >= build::imagery::PAD as usize
                && p.y >= build::imagery::PAD as usize
                && p.x < (build::imagery::PAD + build::imagery::CORE) as usize
                && p.y < (build::imagery::PAD + build::imagery::CORE) as usize
        })
        .map(|p| {
            let ll = build::tree_proposals::lonlat(geo, p.x as f64, p.y as f64);
            let radius = p.radius.clamp(0.7, 4.5);
            InferredTree {
                longitude: ll[0],
                latitude: ll[1],
                crown_radius_meters: radius,
                height_meters: (radius * 2.8).clamp(3.5, 14.),
            }
        })
        .collect())
}
