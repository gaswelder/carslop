use super::{
    bake_collision, bake_scene, boundary::Boundary, download::download, extract_osmpbf, invalid,
    osm, prepare_imagery, scenery, surfaces, terrain, AerialPaths, BoundingBox, ImageryPaths,
    Result,
};
use std::{fs, path::Path, time::Duration};

const ELEVATION_URL: &str = "https://s3.amazonaws.com/elevation-tiles-prod/skadi";

pub(crate) fn add_limassol() -> Result<()> {
    let boundary_url = "https://api.openstreetmap.org/api/0.6/relation/2087230/full";
    let pbf_url = "https://download.geofabrik.de/europe/cyprus-latest.osm.pbf";

    let relation: i64 = regex::Regex::new(r"/relation/([1-9][0-9]*)/full$")?
        .captures(boundary_url)
        .ok_or_else(|| invalid("expected an OSM relation/full boundary URL"))?[1]
        .parse()?;
    let cache_dir = Path::new(".cache");
    let root = Path::new(crate::city_assets::LOCATIONS_DIRECTORY);
    let destination = root.join("limassol");
    if destination.exists() {
        return Err(invalid(
            "location limassol already exists; refusing to replace it",
        ));
    }
    fs::create_dir_all(cache_dir)?;
    fs::create_dir_all(root)?;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(900)))
        .build()
        .into();
    let dir_sources = cache_dir.join("sources");
    let path_boundary = dir_sources.join("boundary.osm");
    let dir_elevation = dir_sources.join("elevation");
    let path_source = cache_dir.join("limassol.osm");
    let path_pbf = dir_sources.join("cyprus.osm.pbf");

    let staging = tempfile::Builder::new()
        .prefix(".import-limassol-")
        .tempdir_in(root)?;
    let path_city = staging.path().join("city.txt");
    let path_streets = staging.path().join("streets.txt");
    let path_streets_report = staging.path().join("streets.json");
    let path_landscape = staging.path().join("landscape.txt");
    let path_landscape_report = staging.path().join("landscape.json");
    let path_furniture = staging.path().join("furniture.txt");
    let path_furniture_report = staging.path().join("furniture.json");
    let path_heightfield = staging.path().join("elevation/heightfield.txt");
    let path_elevation_tiles_directory = staging.path().join("elevation");
    let path_elevation_attribution_source = dir_elevation.join("README.md");
    let path_elevation_attribution = staging.path().join("elevation/source-attribution.md");
    let path_source_copy = staging.path().join("source.osm");
    let path_manifest = staging.path().join("manifest.json");
    let path_collision = staging.path().join("streets.bin");

    // Fetch the OSM relation from boundary_url and assemble its way
    // members by node identity, including reversed ways. Validate closed outer
    // rings and holes; compilation embeds the geometry in manifest.json.
    // The caller supplies no crop; missing, unclosed, invalid, or nested
    // geometry fails explicitly.
    download(&agent, boundary_url, &path_boundary)?;
    let boundary = Boundary::resolve(&path_boundary, relation)?;
    let bbox = boundary.bbox();

    //
    // Check size limit.
    //
    let width = (bbox.east - bbox.west).to_radians()
        * 6371000.
        * ((bbox.south + bbox.north) / 2.).to_radians().cos();
    let height = (bbox.north - bbox.south).to_radians() * 6371000.;
    if width > 80000. || height > 80000. {
        return Err(invalid(
            "administrative boundary exceeds compiler's 80 km limit",
        ));
    }

    println!(
        "Resolved relation {relation}: {} polygons; bounds {bbox:?}",
        boundary.geometry.0.len()
    );

    //
    // Download the Cyprus extract from Geofabrik.
    //
    download(&agent, pbf_url, &path_pbf)?;

    //
    // Download required Mapzen/Skadi HGT tiles.
    //
    let tiles = elevation_tiles(bbox);
    for tile in &tiles {
        download(
            &agent,
            &format!("{ELEVATION_URL}/{}/{tile}.hgt.gz", &tile[..3]),
            &dir_elevation.join(format!("{tile}.hgt.gz")),
        )?;
    }

    //
    // Extract OSM data.
    //
    extract_osmpbf(&path_pbf, &path_source, bbox)?;

    // Compile roads against the administrative polygon, splitting crossings and
    // excluding holes. Keep buildings whose entire footprint is inside the
    // border. Clip street surfaces and mapped land cover, and filter mapped
    // trees and furniture using the same polygon.

    println!("Reading OSM XML...");
    let source = osm::read(&path_source)?;
    let map = osm::compile_boundary(&source, Some(bbox.to_array()), Some(&boundary))?;
    fs::create_dir_all(&path_elevation_tiles_directory)?;
    let city = osm::write(&map, &path_city)?;
    println!("Building streets ({} segments)...", map.roads.len());
    let surfaces = surfaces::build(&map, &city, &path_streets, &path_streets_report)?;
    println!("Building mapped landscape and roadside controls...");
    scenery::build(
        &source,
        &map,
        &surfaces,
        &city,
        &path_landscape,
        &path_landscape_report,
        &path_furniture,
        &path_furniture_report,
    )?;
    println!("Building elevation...");
    terrain::build(
        &map,
        &city,
        &dir_elevation.join(format!("{}.hgt.gz", tiles[0])),
        &path_heightfield,
        &path_elevation_tiles_directory,
        &path_elevation_attribution_source,
        &path_elevation_attribution,
    )?;
    fs::copy(&path_source, &path_source_copy)?;
    let south_west = [
        map.origin[0] - map.bounds[3].to_degrees() / 6371000.0,
        map.origin[1] + map.bounds[0].to_degrees() / 6371000.0 / map.origin[0].to_radians().cos(),
    ];
    let north_east = [
        map.origin[0] - map.bounds[1].to_degrees() / 6371000.0,
        map.origin[1] + map.bounds[2].to_degrees() / 6371000.0 / map.origin[0].to_radians().cos(),
    ];
    let manifest = serde_json::json!({
        "name": "limassol",
        "boundary": boundary.to_geojson(),
    });
    fs::write(
        &path_manifest,
        serde_json::to_string_pretty(&manifest)? + "\n",
    )?;
    println!("Baking collision data...");
    bake_collision(&path_city, &path_streets, &path_collision)?;

    // Run the existing Esri World Imagery pipeline: textures, gutters, tile
    // checksums, and tree proposals. Skip blocks and texture tiles wholly outside
    // the polygon and exclude outside tree proposals. Boundary tiles retain
    // their rectangular storage and gutters.
    println!("Preparing satellite textures and imagery trees...");
    let imagery_work = cache_dir.join("imagery");
    let aerial_staging = imagery_work.join("aerial");
    let aerial_destination = staging.path().join("aerial");
    prepare_imagery(
        &ImageryPaths {
            city: &path_city,
            streets: &path_streets,
            landscape: &path_landscape,
            manifest: &path_manifest,
            downloads_directory: &imagery_work.join("downloads"),
            blocks_directory: &imagery_work,
            service: &imagery_work.join("service.json"),
            plan: &imagery_work.join("plan.json"),
            previous_trees: &imagery_work.join("previous-imagery-trees.txt"),
            report: &imagery_work.join("report.json"),
            trees: &imagery_work.join("imagery-trees.txt"),
            installed_trees: &staging.path().join("imagery-trees.txt"),
            installed_trees_report: &staging.path().join("imagery-trees.json"),
            aerial_backup: &imagery_work.join("previous-aerial"),
            staged_aerial_directory: &aerial_staging,
            installed_aerial: AerialPaths {
                directory: &aerial_destination,
                service: &aerial_destination.join("service.json"),
                plan: &aerial_destination.join("plan.json"),
                catalog: &aerial_destination.join("tiles.txt"),
                source: &aerial_destination.join("source.json"),
            },
        },
        [south_west[0], south_west[1], north_east[0], north_east[1]],
    )?;

    // Bake collision data and all scene layers, clipping the visible terrain
    // mesh to the border. The elevation raster and geographic extent remain
    // rectangular for the terrain format; the polygon determines which roads,
    // buildings, and terrain triangles are imported.
    println!("Compiling collisions");
    bake_collision(&path_city, &path_streets, &path_collision)?;

    println!("Compiling scene");
    let city = crate::city::City::load(&crate::city_assets::CityAsset::new(
        staging.path().to_owned(),
    ))
    .map_err(invalid)?;
    bake_scene(
        &city,
        &path_manifest,
        &staging.path().join("decorations.pack"),
        &staging.path().join("scene.pack"),
        staging.path(),
    )?;
    // Check required assets before renaming staging into data/locations/limassol.
    // Select the completed location with: cars play --location limassol.
    for file in ["aerial/tiles.txt"] {
        if !staging.path().join(file).is_file() {
            return Err(invalid(format!("missing completed asset {file}")));
        }
    }
    if destination.exists() {
        return Err(invalid(
            "limassol appeared during import; refusing to replace it",
        ));
    }
    fs::rename(staging.path(), &destination)?;
    println!("done",);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn elevation_downloads_cover_both_sides_of_degree_edges() {
        assert_eq!(
            elevation_tiles(BoundingBox {
                south: 34.7,
                west: 32.99,
                north: 34.8,
                east: 33.01
            }),
            vec!["N34E032", "N34E033"]
        );
        assert_eq!(
            elevation_tiles(BoundingBox {
                south: 34.998,
                west: 33.2,
                north: 35.001,
                east: 33.3
            }),
            vec!["N34E033", "N35E033"]
        );
    }
}

fn elevation_tiles(bounds: BoundingBox) -> Vec<String> {
    // Match Hgt::covering exactly, including latitude-dependent longitude padding.
    let latitude_pad = (160.0_f64 / 6371000.0).to_degrees();
    let longitude_pad = latitude_pad / ((bounds.south + bounds.north) / 2.0).to_radians().cos();
    let mut tiles = vec![];
    for lat in
        (bounds.south - latitude_pad).floor() as i32..=(bounds.north + latitude_pad).floor() as i32
    {
        for lon in (bounds.west - longitude_pad).floor() as i32
            ..=(bounds.east + longitude_pad).floor() as i32
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
