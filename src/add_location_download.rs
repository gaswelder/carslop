use crate::{
    download,
    error::{invalid, Result},
    geometry::BoundingBox,
    imagery, osm, sources,
};
use geo::{Area, Intersects, MapCoords, Relate};
use serde::{Deserialize, Serialize};
use std::{fs, path::Path, time::Duration};

const REGIONAL_OSM: &str = "region.osm.pbf";
const REGIONAL_INDEX: &str = "https://download.geofabrik.de/index-v1.json";
const ELEVATION_SERVICE: &str = "https://s3.amazonaws.com/elevation-tiles-prod/skadi";
const MAX_IMAGERY_BLOCKS: usize = 2000;

pub const LOCATION_OSM_FILE: &str = "source.osm";
pub const ELEVATION_DATA_DIR: &str = "elevation_data";
pub const SATELLITE_IMAGES_DIR: &str = "satellite_images";
pub const MANIFEST_FILE: &str = "manifest.json";

#[derive(Serialize, Deserialize)]
pub struct Manifest {
    pub name: String,
    pub area: osm::ImportArea,
    pub height_tiles: Vec<String>,
    pub imagery_blocks: Vec<imagery::Block>,
}

pub fn read_manifest(directory: &Path) -> Result<Manifest> {
    let bytes = fs::read(directory.join(MANIFEST_FILE))?;
    Ok(serde_json::from_slice(&bytes)?)
}

fn write_manifest(directory: &Path, manifest: &Manifest) -> Result<()> {
    fs::write(
        directory.join(MANIFEST_FILE),
        serde_json::to_vec_pretty(manifest)?,
    )?;
    Ok(())
}

pub fn download_map_data(area: osm::ImportArea) -> Result<std::path::PathBuf> {
    let boundary = area.geometry()?;
    let name = area.name();
    let download_dir = std::path::PathBuf::from(format!("src-locations/{name}"));
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(900)))
        .build()
        .into();
    let options = download::DownloadOptions::default();

    let imagery_blocks = imagery_blocks(&boundary, MAX_IMAGERY_BLOCKS)?;

    // Persist the index alongside the sources so resuming selects the same extract.
    let index_path = download_dir.join("geofabrik-index.json");
    download::download(&agent, REGIONAL_INDEX, &index_path, &options)?;
    let index: RegionalIndex = serde_json::from_slice(&fs::read(index_path)?)?;
    let regional_osm_url = index.covering_extract(&boundary)?;

    // Download the region map.
    download::download(
        &agent,
        regional_osm_url,
        &download_dir.join(REGIONAL_OSM),
        &options,
    )?;

    println!(
        "extracting requested area ({}x{} km)",
        area.width_km, area.height_km
    );
    sources::extract_osm_boundary(
        &download_dir.join(REGIONAL_OSM),
        &download_dir.join(LOCATION_OSM_FILE),
        boundary.bbox(),
    )?;

    // Download relevant Mapzen/Skadi HGT tiles.
    let height_tiles = elevation_tiles(boundary.bbox());
    for tile in &height_tiles {
        download::download(
            &agent,
            &format!("{}/{}/{tile}.hgt.gz", ELEVATION_SERVICE, &tile[..3]),
            &download_dir
                .join(ELEVATION_DATA_DIR)
                .join(format!("{tile}.hgt.gz")),
            &options,
        )?;
    }

    let images_dir = download_dir.join(SATELLITE_IMAGES_DIR);
    fs::create_dir_all(&images_dir)?;
    let imagery_agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .into();
    let imagery_options = download::DownloadOptions {
        attempts: 3,
        max_bytes: Some(32 * 1024 * 1024),
        validate: |path| {
            let bytes = fs::read(path)?;
            Ok(image::load_from_memory(&bytes).is_ok_and(|image| {
                image.width() == imagery::CORE + 2 * imagery::PAD
                    && image.height() == imagery::CORE + 2 * imagery::PAD
            }))
        },
        replace_invalid: true,
    };
    for block in &imagery_blocks {
        let extent = block.extent;
        let side = imagery::CORE + 2 * imagery::PAD;
        let url = format!(
            "{}/export?bbox={:.9},{:.9},{:.9},{:.9}&bboxSR=3857&imageSR=3857&size={side},{side}&format=jpg&f=image",
            imagery::SERVICE, extent[0], extent[1], extent[2], extent[3],
        );
        download::download(
            &imagery_agent,
            &url,
            &imagery::image_path(&images_dir, block),
            &imagery_options,
        )
        .map_err(|error| invalid(format!("block {},{}: {error}", block.x, block.y)))?;
    }
    let data = Manifest {
        name,
        area,
        height_tiles,
        imagery_blocks,
    };
    write_manifest(&download_dir, &data)?;
    Ok(download_dir)
}

fn imagery_blocks(boundary: &osm::OSMArea, limit: usize) -> Result<Vec<imagery::Block>> {
    let bbox = boundary.bbox();
    if !bbox.is_valid_for_imagery() {
        return Err(invalid("invalid city geographic bounds"));
    }
    let mercator_boundary = boundary.geometry.map_coords(|c| {
        let p = imagery::mercator(c.x, c.y);
        geo::Coord { x: p[0], y: p[1] }
    });
    let all: Vec<_> = imagery::blocks(bbox.to_array())
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

impl RegionalGeometry {
    fn polygons(&self) -> geo::MultiPolygon<f64> {
        let polygons = match self {
            Self::Polygon(rings) => std::slice::from_ref(rings),
            Self::MultiPolygon(polygons) => polygons.as_slice(),
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
}

impl RegionalIndex {
    fn covering_extract(&self, boundary: &osm::OSMArea) -> Result<&str> {
        self.features
            .iter()
            .filter_map(|extract| {
                let url = extract.properties.urls.pbf.as_deref()?;
                let geometry = extract.geometry.polygons();
                geometry
                    .relate(&boundary.geometry)
                    .is_covers()
                    .then(|| (geometry.unsigned_area(), url))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(b.1)))
            .map(|(_, url)| url)
            .ok_or_else(|| invalid("no Geofabrik extract covers the complete requested rectangle"))
    }
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
mod import_tests {
    use super::*;

    #[test]
    fn extract_selection_checks_full_geometry_and_chooses_smallest_cover() {
        let index: RegionalIndex = serde_json::from_value(serde_json::json!({
            "features": [
                {"properties": {"urls": {"pbf": "large"}}, "geometry": {
                    "type": "MultiPolygon", "coordinates": [[[[0,0],[10,0],[10,10],[0,10],[0,0]]]]}},
                {"properties": {"urls": {"pbf": "partial"}}, "geometry": {
                    "type": "Polygon", "coordinates": [[[1,1],[3,1],[3,3],[1,3],[1,1]]]}},
                {"properties": {"urls": {"pbf": "with-hole"}}, "geometry": {
                    "type": "Polygon", "coordinates": [
                        [[1,1],[5,1],[5,5],[1,5],[1,1]],
                        [[2.5,2.5],[2.5,3.5],[3.5,3.5],[3.5,2.5],[2.5,2.5]]]}},
                {"properties": {"urls": {"pbf": "smallest-cover"}}, "geometry": {
                    "type": "Polygon", "coordinates": [[[1,1],[6,1],[6,6],[1,6],[1,1]]]}}
            ]
        })).unwrap();
        let boundary = osm::OSMArea {
            geometry: geo::MultiPolygon(vec![geo::Rect::new(
                geo::Coord { x: 2., y: 2. },
                geo::Coord { x: 4., y: 4. },
            )
            .to_polygon()]),
        };
        assert_eq!(index.covering_extract(&boundary).unwrap(), "smallest-cover");
        let boundary = osm::OSMArea {
            geometry: geo::MultiPolygon(vec![geo::Rect::new(
                geo::Coord { x: 9., y: 9. },
                geo::Coord { x: 11., y: 11. },
            )
            .to_polygon()]),
        };
        assert!(index.covering_extract(&boundary).is_err());
    }
}

#[cfg(test)]
mod tile_tests {
    use super::*;
    #[test]
    fn elevation_downloads_cover_both_sides_of_degree_edges() {
        assert_eq!(
            elevation_tiles(BoundingBox {
                min_lat: 34.7,
                min_lon: 32.99,
                max_lat: 34.8,
                max_lon: 33.01
            }),
            vec!["N34E032", "N34E033"]
        );
        assert_eq!(
            elevation_tiles(BoundingBox {
                min_lat: 34.998,
                min_lon: 33.2,
                max_lat: 35.001,
                max_lon: 33.3
            }),
            vec!["N34E033", "N35E033"]
        );
    }
}
