//! Explicit, end-to-end Limassol preparation. Nothing runs during game startup.
use super::{
    bake_collision, bake_scene, boundary::Boundary, compile_boundary, download::download,
    extract_osmpbf, invalid, prepare_imagery, BoundingBox, Options, Result,
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

    //
    //
    //
    let staging = tempfile::Builder::new()
        .prefix(".import-limassol-")
        .tempdir_in(root)?;

    // Compile roads against the administrative polygon, splitting crossings and
    // excluding holes. Keep buildings whose entire footprint is inside the
    // border. Clip street surfaces and mapped land cover, and filter mapped
    // trees and furniture using the same polygon.
    compile_boundary(
        Options {
            source: &path_source,
            output: staging.path(),
            name: "limassol",
            bbox: Some(bbox.to_array()),
            elevation: &dir_elevation.join(format!("{}.hgt.gz", tiles[0])),
        },
        &boundary,
    )?;

    // Run the existing Esri World Imagery pipeline: textures, gutters, tile
    // checksums, and tree proposals. Skip blocks and texture tiles wholly outside
    // the polygon and exclude outside tree proposals. Boundary tiles retain
    // their rectangular storage and gutters.
    println!("Preparing satellite textures and imagery trees...");
    prepare_imagery(staging.path(), &cache_dir.join("imagery"), None)?;

    // Bake collision data and all scene layers, clipping the visible terrain
    // mesh to the border. The elevation raster and geographic extent remain
    // rectangular for the terrain format; the polygon determines which roads,
    // buildings, and terrain triangles are imported.
    println!("Baking collision and scene archives...");
    bake_collision(staging.path())?;
    bake_scene(staging.path())?;
    // Check required assets before renaming staging into data/locations/limassol.
    // Select the completed location with: cars play --location limassol.
    for file in [
        "city.txt",
        "streets.txt",
        "landscape.txt",
        "furniture.txt",
        "elevation/heightfield.txt",
        "scene.pack",
        "aerial/tiles.txt",
    ] {
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
