use crate::build;
use crate::build::download::Downloader;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::BoundingBox;
use geo::{Area, Intersects, MapCoords, Relate};
use serde::{Deserialize, Serialize};
use std::{
    fs,
    path::{Path, PathBuf},
    thread,
    time::Duration,
};

const REGIONAL_OSM: &str = "region.osm.pbf";
const REGIONAL_INDEX: &str = "https://download.geofabrik.de/index-v1.json";
const ELEVATION_SERVICE: &str = "https://s3.amazonaws.com/elevation-tiles-prod/skadi";
const MAX_IMAGERY_BLOCKS: usize = 2000;

pub const LOCATION_OSM_FILE: &str = "source.osm";
pub const ELEVATION_DATA_DIR: &str = "elevation_data";
pub const SATELLITE_IMAGES_DIR: &str = "satellite_images";
pub const MANIFEST_FILE: &str = "manifest.json";

#[derive(Serialize, Deserialize)]
/// Acquisition state retained alongside downloaded source files.
pub struct DownloadedSources {
    pub name: String,
    pub area: build::osm::ImportArea,
    pub height_tiles: Vec<String>,
    pub imagery_blocks: Vec<build::imagery::Block>,
}

pub fn read_manifest(directory: &Path) -> Result<DownloadedSources> {
    let bytes = fs::read(directory.join(MANIFEST_FILE))?;
    Ok(serde_json::from_slice(&bytes)?)
}

fn write_manifest(directory: &Path, manifest: &DownloadedSources) -> Result<()> {
    let path = directory.join(MANIFEST_FILE);
    let bytes = serde_json::to_vec_pretty(manifest)?;
    if fs::read(&path).ok().as_ref() != Some(&bytes) {
        fs::write(path, bytes)?;
    }
    Ok(())
}

pub fn download_map_data(area: build::osm::ImportArea) -> Result<PathBuf> {
    let boundary = build::osm::geometry(&area)?;
    let name = build::osm::name(&area);
    let download_dir = PathBuf::from(format!("src-locations/{name}"));
    let cache_dir = Path::new(".cache");
    fs::create_dir_all(&download_dir)?;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(900)))
        .build()
        .into();

    let downloader = build::download::new(agent, cache_dir);
    let imagery_blocks = imagery_blocks(&boundary, MAX_IMAGERY_BLOCKS)?;

    // Persist the index alongside the sources so resuming selects the same extract.
    let index_path = download_dir.join("geofabrik-index.json");
    build::download::copy_download(&downloader, REGIONAL_INDEX, &index_path)?;
    let index: RegionalIndex = serde_json::from_slice(&fs::read(index_path)?)?;
    let regional_osm_url = covering_extract(&index, &boundary)?;

    // Download the region map.
    build::download::copy_download(
        &downloader,
        regional_osm_url,
        &download_dir.join(REGIONAL_OSM),
    )?;

    println!(
        "extracting requested area ({}x{} km)",
        area.width_km, area.height_km
    );
    build::stage::run(
        &download_dir.join("tmp/extract.json"),
        &[download_dir.join(REGIONAL_OSM)],
        &[
            download_dir.join(LOCATION_OSM_FILE),
            download_dir.join("source.json"),
        ],
        &serde_json::to_string(&area)?,
        || {
            build::osm_extract::extract_osm_boundary(
                &download_dir.join(REGIONAL_OSM),
                &download_dir.join(LOCATION_OSM_FILE),
                build::osm::bbox(&boundary),
            )
        },
    )?;

    // Download relevant Mapzen/Skadi HGT tiles.
    let height_tiles = elevation_tiles(build::osm::bbox(&boundary));
    for tile in &height_tiles {
        build::download::copy_download(
            &downloader,
            &format!("{}/{}/{tile}.hgt.gz", ELEVATION_SERVICE, &tile[..3]),
            &download_dir
                .join(ELEVATION_DATA_DIR)
                .join(format!("{tile}.hgt.gz")),
        )?;
    }

    let images_dir = download_dir.join(SATELLITE_IMAGES_DIR);
    fs::create_dir_all(&images_dir)?;
    let imagery_agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .into();
    let imagery_downloader = build::download::new(imagery_agent, cache_dir);
    for block in &imagery_blocks {
        let extent = block.extent;
        let side = build::imagery::CORE + 2 * build::imagery::PAD;
        let url = format!(
            "{}/export?bbox={:.9},{:.9},{:.9},{:.9}&bboxSR=3857&imageSR=3857&size={side},{side}&format=jpg&f=image",
            build::imagery::SERVICE, extent[0], extent[1], extent[2], extent[3],
        );
        download_imagery(
            &imagery_downloader,
            &url,
            &build::imagery::image_path(&images_dir, block),
        )
        .map_err(|error| invalid(format!("block {},{}: {error}", block.x, block.y)))?;
    }
    let data = DownloadedSources {
        name,
        area,
        height_tiles,
        imagery_blocks,
    };
    write_manifest(&download_dir, &data)?;
    Ok(download_dir)
}

fn download_imagery(downloader: &Downloader, url: &str, path: &Path) -> Result<()> {
    const ATTEMPTS: usize = 3;
    const MAX_BYTES: u64 = 32 * 1024 * 1024;
    for attempt in 0..ATTEMPTS {
        let result = (|| -> Result<()> {
            build::download::copy_download(downloader, url, path)?;
            let valid = fs::metadata(path)?.len() <= MAX_BYTES
                && image::load_from_memory(&fs::read(path)?).is_ok_and(|image| {
                    image.width() == build::imagery::CORE + 2 * build::imagery::PAD
                        && image.height() == build::imagery::CORE + 2 * build::imagery::PAD
                });
            if !valid {
                build::download::invalidate(downloader, url)?;
                fs::remove_file(path)?;
                return Err(invalid(format!("invalid downloaded image: {url}")));
            }
            Ok(())
        })();
        match result {
            Ok(()) => return Ok(()),
            Err(error) if attempt + 1 == ATTEMPTS => return Err(error),
            Err(_) => thread::sleep(Duration::from_secs(1 << attempt)),
        }
    }
    unreachable!("imagery download attempts are nonzero")
}

fn imagery_blocks(
    boundary: &build::osm::OSMArea,
    limit: usize,
) -> Result<Vec<build::imagery::Block>> {
    let bbox = build::osm::bbox(boundary);
    if !util::geometry::is_valid_for_imagery(&bbox) {
        return Err(invalid("invalid city geographic bounds"));
    }
    let mercator_boundary = boundary.geometry.map_coords(|c| {
        let p = build::imagery::mercator(c.x, c.y);
        geo::Coord { x: p[0], y: p[1] }
    });
    let all: Vec<_> = build::imagery::blocks(util::geometry::bounding_box_to_array(bbox))
        .into_iter()
        .filter(|b| {
            mercator_boundary.intersects(&geo::Rect::new(
                geo::Coord {
                    x: b.extent[0],
                    y: b.extent[1],
                },
                geo::Coord {
                    x: b.extent[2],
                    y: b.extent[3],
                },
            ))
        })
        .collect();
    if all.is_empty() || all.len() > limit {
        return Err(invalid(
            "imagery job exceeds configured block limit or is empty; crop the map",
        ));
    }
    Ok(all)
}

#[derive(Deserialize)]
struct RegionalIndex {
    features: Vec<RegionalExtract>,
}

#[derive(Deserialize)]
struct RegionalExtract {
    properties: RegionalProperties,
    geometry: RegionalGeometry,
}

#[derive(Deserialize)]
struct RegionalProperties {
    urls: RegionalUrls,
}

#[derive(Deserialize)]
struct RegionalUrls {
    pbf: Option<String>,
}

#[derive(Deserialize)]
#[serde(tag = "type", content = "coordinates")]
enum RegionalGeometry {
    Polygon(Vec<Vec<[f64; 2]>>),
    MultiPolygon(Vec<Vec<Vec<[f64; 2]>>>),
}

fn polygons(regional_geometry: &RegionalGeometry) -> geo::MultiPolygon<f64> {
    let polygons = match regional_geometry {
        RegionalGeometry::Polygon(rings) => std::slice::from_ref(rings),
        RegionalGeometry::MultiPolygon(polygons) => polygons.as_slice(),
    };
    geo::MultiPolygon(
        polygons
            .iter()
            .filter_map(|rings| {
                let (exterior, holes) = rings.split_first()?;
                let ring = |points: &Vec<[f64; 2]>| geo::LineString::from(points.clone());
                Some(geo::Polygon::new(
                    ring(exterior),
                    holes.iter().map(ring).collect(),
                ))
            })
            .collect(),
    )
}

fn covering_extract<'a>(
    regional_index: &'a RegionalIndex,
    boundary: &build::osm::OSMArea,
) -> Result<&'a str> {
    regional_index
        .features
        .iter()
        .filter_map(|extract| {
            let url = extract.properties.urls.pbf.as_deref()?;
            let geometry = polygons(&extract.geometry);
            geometry
                .relate(&boundary.geometry)
                .is_covers()
                .then(|| (geometry.unsigned_area(), url))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(b.1)))
        .map(|(_, url)| url)
        .ok_or_else(|| invalid("no Geofabrik extract covers the complete requested rectangle"))
}

fn elevation_tiles(bounds: BoundingBox) -> Vec<String> {
    // Match Hgt::covering exactly, including latitude-dependent longitude padding.
    let latitude_pad = (160.0_f64 / 6371000.0).to_degrees();
    let longitude_pad = latitude_pad / ((bounds.min_lat + bounds.max_lat) / 2.0).to_radians().cos();
    let mut tiles = vec![];
    for lat in (bounds.min_lat - latitude_pad).floor() as i32
        ..=(bounds.max_lat + latitude_pad).floor() as i32
    {
        for lon in (bounds.min_lon - longitude_pad).floor() as i32
            ..=(bounds.max_lon + longitude_pad).floor() as i32
        {
            tiles.push(format!(
                "{}{:02}{}{:03}",
                if lat >= 0 { 'N' } else { 'S' },
                lat.abs(),
                if lon >= 0 { 'E' } else { 'W' },
                lon.abs()
            ));
        }
    }
    tiles
}

#[cfg(test)]
#[path = "tests/download_location_import.rs"]
mod import_tests;

#[cfg(test)]
#[path = "tests/download_location_tile.rs"]
mod tile_tests;
