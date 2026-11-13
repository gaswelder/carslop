use crate::{
    boundary, download,
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

pub const BOUNDARY_FILE: &str = "boundary.osm";
pub const LOCATION_OSM_FILE: &str = "source.osm";
pub const ELEVATION_DATA_DIR: &str = "elevation_data";
pub const SATELLITE_IMAGES_DIR: &str = "satellite_images";
pub const MANIFEST_FILE: &str = "manifest.json";

#[derive(Serialize, Deserialize)]
pub struct Manifest {
    pub name: String,
    pub boundary_relation: i64,
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

pub fn download_map_data(relation_url: &str) -> Result<std::path::PathBuf> {
    let relation = osm::relation_id(relation_url)?;
    let download_dir = std::path::PathBuf::from(format!("src-locations/relation-{relation}"));
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(900)))
        .build()
        .into();

    // Download the location boundary.
    let options = download::DownloadOptions::default();
    download::download(
        &agent,
        &format!("https://api.openstreetmap.org/api/0.6/relation/{relation}/full"),
        &download_dir.join(BOUNDARY_FILE),
        &options,
    )?;

    // Parse the boundary.
    println!("parsing the boundary");
    let boundary_map = osm::read(&download_dir.join(BOUNDARY_FILE))?;
    let name = osm::import_name(
        boundary_map
            .relations
            .iter()
            .find(|r| r.id == relation)
            .ok_or_else(|| invalid("requested relation missing from boundary response"))?,
    );
    let boundary = boundary::resolve_boundary(&boundary_map, relation)?;
    println!(
        "read the boundary {}: {} polygons",
        relation,
        boundary.geometry.0.len()
    );

    // Check boundary size.
    let (width, height) = boundary.size();
    if width > 80000. || height > 80000. {
        return Err(invalid("the boundary exceeds compiler's 80 km limit"));
    }
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

    println!("extracting the requested area from the region map");
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
        boundary_relation: relation,
        height_tiles,
        imagery_blocks,
    };
    write_manifest(&download_dir, &data)?;
    Ok(download_dir)
}

fn imagery_blocks(boundary: &osm::OSMBoundary, limit: usize) -> Result<Vec<imagery::Block>> {
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
    fn covering_extract(&self, boundary: &osm::OSMBoundary) -> Result<&str> {
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
            .ok_or_else(|| invalid("no Geofabrik extract covers the complete boundary"))
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
        let boundary = osm::OSMBoundary {
            relation: 1,
            geometry: geo::MultiPolygon(vec![geo::Rect::new(
                geo::Coord { x: 2., y: 2. },
                geo::Coord { x: 4., y: 4. },
            )
            .to_polygon()]),
        };
        assert_eq!(index.covering_extract(&boundary).unwrap(), "smallest-cover");
        let boundary = osm::OSMBoundary {
            relation: 2,
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

#[cfg(test)]
mod stage_tests {
    use super::*;
    use crate::{
        add_location_compile::compile_location, add_location_trees, boundary::resolve_boundary,
        imagery, osm,
    };
    use std::io::Write;

    fn downloaded_fixture() -> tempfile::TempDir {
        let directory = tempfile::tempdir().unwrap();
        let download_dir = directory.path();
        fs::create_dir(&download_dir.join(SATELLITE_IMAGES_DIR)).unwrap();
        fs::create_dir(&download_dir.join(ELEVATION_DATA_DIR)).unwrap();
        fs::write(
            &download_dir.join(BOUNDARY_FILE),
            r#"<osm version="0.6">
<node id="1" lat="35.160" lon="33.320"/><node id="2" lat="35.160" lon="33.324"/>
<node id="3" lat="35.164" lon="33.324"/><node id="4" lat="35.164" lon="33.320"/>
<way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/></way>
<relation id="2087230"><member type="way" ref="10" role="outer"/>
<tag k="type" v="boundary"/><tag k="boundary" v="administrative"/></relation></osm>"#,
        )
        .unwrap();
        fs::write(
            &download_dir.join(LOCATION_OSM_FILE),
            r#"<osm version="0.6">
<node id="11" lat="35.161" lon="33.3205"/><node id="12" lat="35.161" lon="33.3235"/>
<way id="20"><nd ref="11"/><nd ref="12"/><tag k="highway" v="residential"/>
<tag k="name" v="Fixture Road"/></way></osm>"#,
        )
        .unwrap();
        let boundary = resolve_boundary(
            &osm::read(&download_dir.join(BOUNDARY_FILE)).unwrap(),
            2087230,
        )
        .unwrap();
        let data = Manifest {
            name: "Fixture".into(),
            boundary_relation: 2087230,
            height_tiles: elevation_tiles(boundary.bbox()),
            imagery_blocks: imagery_blocks(&boundary, 2000).unwrap(),
        };
        assert_eq!(data.imagery_blocks.len(), 1);
        for block in &data.imagery_blocks {
            let side = imagery::CORE + 2 * imagery::PAD;
            let center = (imagery::PAD + 180) as i32;
            let image = image::RgbImage::from_fn(side, side, |x, y| {
                if (x as i32 - center).pow(2) + (y as i32 - center).pow(2) <= 64 {
                    image::Rgb([40, 110, 35])
                } else {
                    image::Rgb([120, 120, 120])
                }
            });
            image
                .save(imagery::image_path(
                    &download_dir.join(SATELLITE_IMAGES_DIR),
                    block,
                ))
                .unwrap();
        }
        for tile in &data.height_tiles {
            let mut gzip = flate2::write::GzEncoder::new(
                fs::File::create(
                    download_dir
                        .join(ELEVATION_DATA_DIR)
                        .join(format!("{tile}.hgt.gz")),
                )
                .unwrap(),
                flate2::Compression::fast(),
            );
            gzip.write_all(&[0; 65 * 65 * 2]).unwrap();
            gzip.finish().unwrap();
        }
        write_manifest(download_dir, &data).unwrap();
        directory
    }

    #[test]
    fn inference_compile_and_write_are_independent_stages() {
        let downloads = downloaded_fixture();
        let download_dir = downloads.path();
        let image_path = imagery::image_path(
            &download_dir.join(SATELLITE_IMAGES_DIR),
            &read_manifest(download_dir).unwrap().imagery_blocks[0],
        );
        let original_image = fs::read(&image_path).unwrap();
        add_location_trees::add_trees(downloads.path()).unwrap();
        let inferred = fs::read(&download_dir.join(add_location_trees::TREES_FILE)).unwrap();
        let trees: Vec<add_location_trees::InferredTree> =
            serde_json::from_slice(&inferred).unwrap();
        assert!(!trees.is_empty());
        assert!(trees
            .iter()
            .all(|t| t.crown_radius_meters > 0. && t.height_meters >= 3.5));
        add_location_trees::add_trees(downloads.path()).unwrap();
        assert_eq!(
            fs::read(&download_dir.join(add_location_trees::TREES_FILE)).unwrap(),
            inferred
        );
        assert_eq!(fs::read(&image_path).unwrap(), original_image);
        assert!(!downloads.path().join("city.txt").exists());

        // The compiler consumes the saved proposals; editing them affects the result.
        let mut trees = trees;
        trees[0].height_meters = 9.25;
        fs::write(
            &download_dir.join(add_location_trees::TREES_FILE),
            serde_json::to_vec(&trees).unwrap(),
        )
        .unwrap();
        let saved_trees = fs::read(&download_dir.join(add_location_trees::TREES_FILE)).unwrap();
        let location = compile_location(downloads.path()).unwrap();
        assert_eq!(location.name, "Fixture");
        assert!(!location.roads.is_empty());
        assert!(location.trees.iter().any(|t| t.height_meters == 9.25));
        assert_eq!(
            fs::read(&download_dir.join(add_location_trees::TREES_FILE)).unwrap(),
            saved_trees
        );
        assert_eq!(fs::read(&image_path).unwrap(), original_image);
        drop(downloads);

        let output = tempfile::tempdir().unwrap();
        location.write(output.path()).unwrap();
        let reread = crate::location::Location::read(output.path()).unwrap();
        assert_eq!(location.trees, reread.trees);
        assert_eq!(location.labels, reread.labels);
        assert_eq!(location.roads, reread.roads);
        crate::world::Location::load(&crate::city_assets::CityAsset::new(output.path().into()))
            .unwrap();
        drop(location);
        assert!(output.path().join("scene.pack").is_file());
    }
}
