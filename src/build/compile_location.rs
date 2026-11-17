use crate::build;
use crate::build::furniture::build_furniture;
use crate::build::landscape::build_landscape;
use crate::build::scene::build_scene;
use crate::build::surfaces::build_streets;
use crate::model;
use crate::model::source_location::SourceLocation;
use crate::render;
use crate::storage;
use crate::storage::city;
use crate::storage::landscape::read_tree_positions;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::BoundingBox;
use crate::util::geometry::XZ;
use crate::world;
use geo::Intersects;
use std::{
    fs,
    path::{Path, PathBuf},
};

/// Private compiler workspace; none of these intermediates enter the download directory.
struct ImportPaths {
    downloads: PathBuf,
    output: PathBuf,
    source_osm: PathBuf,
    imagery: PathBuf,
    aerial: PathBuf,
    scene: PathBuf,
    decorations: PathBuf,
}

fn new(output: &Path, downloads: &Path) -> ImportPaths {
    ImportPaths {
        downloads: downloads.into(),
        output: output.into(),
        source_osm: output.join("source.osm"),
        imagery: output.join("imagery"),
        aerial: output.join("aerial"),
        scene: output.join("scene.pack"),
        decorations: output.join("decorations.pack"),
    }
}

/// Compile entirely from local sources, retaining large render payloads on disk.
pub fn compile_location(download_dir: &Path) -> Result<model::location::Location> {
    let workspace = tempfile::Builder::new()
        .prefix("carslop-compile-")
        .tempdir()?;
    let paths = new(workspace.path(), download_dir);
    let data = build::download_location::read_manifest(&paths.downloads)?;
    if data.height_tiles.is_empty() || data.imagery_blocks.is_empty() {
        return Err(invalid("downloaded location data is incomplete"));
    }
    if !paths
        .downloads
        .join(build::infer_trees::TREES_FILE)
        .is_file()
    {
        return Err(invalid(
            "missing inferred trees; run infer_trees(download_dir) before compile",
        ));
    }
    let acquired = CompileSources {
        boundary: build::osm::geometry(&data.area)?,
        height_tiles: data.height_tiles.clone(),
    };
    println!("converting OSM map");
    let converted = convert_map(&paths, &acquired.boundary)?;
    let generated = generate_assets(&paths, &acquired, &converted)?;
    drop(converted);
    compile_imagery(&data, &paths, &generated, &acquired.boundary)?;
    println!("preparing scene assets");
    let prepared = prepare(&paths)?;
    println!("baking scene");
    bake(&paths, &prepared, Some(&acquired.boundary))?;
    drop(prepared);
    render::scene_archive::open(&paths.scene)?;
    println!("loading compiled location");
    storage::location::from_compilation(workspace).map_err(invalid)
}

struct ConvertedMap {
    osm: build::osm::OSMMap,
    location: SourceLocation<f64>,
}
fn convert_map(paths: &ImportPaths, boundary: &build::osm::OSMArea) -> Result<ConvertedMap> {
    fs::copy(
        &paths
            .downloads
            .join(build::download_location::LOCATION_OSM_FILE),
        &paths.source_osm,
    )?;

    // Read the map.
    let source_osm_map = build::osm::read(&paths.source_osm)?;

    // Convert the map to locationMap struct, and trim using the boundary.
    let location_map = build::map::create_location_map(&source_osm_map, boundary)?;

    Ok(ConvertedMap {
        osm: source_osm_map,
        location: location_map,
    })
}

struct GeneratedAssets {
    city: SourceLocation<f32>,
    bbox: BoundingBox,
}
struct CompileSources {
    boundary: build::osm::OSMArea,
    height_tiles: Vec<String>,
}

fn generate_assets(
    paths: &ImportPaths,
    acquired: &CompileSources,
    converted: &ConvertedMap,
) -> Result<GeneratedAssets> {
    let height_tiles = &acquired.height_tiles;
    let source_osm_map = &converted.osm;
    let location_map = &converted.location;
    fs::create_dir_all(&paths.output)?;
    let city_result = city::write_imported(location_map, &paths.output)?;

    println!(
        "building streets ({} segments)",
        location_map.road_lines.len()
    );
    let streets_result = build_streets(location_map, &paths.output, Some(source_osm_map))?;

    println!("building landscape");
    let tree_styles = build_landscape(
        source_osm_map,
        location_map,
        &streets_result.surfaces,
        &paths.output,
    )?;

    println!("building furniture");
    build_furniture(
        source_osm_map,
        location_map,
        &streets_result.surfaces,
        &paths.output,
    )?;

    println!("building elevation");
    build::terrain::build_terrain(
        location_map,
        &paths
            .downloads
            .join(build::download_location::ELEVATION_DATA_DIR)
            .join(format!("{}.hgt.gz", height_tiles[0])),
        &paths.output,
    )?;

    println!("building mapped details");
    let mut details = build::details::build(source_osm_map, location_map, &streets_result.surfaces);
    details.trees.extend(tree_styles);
    println!("loading elevation grid");
    let elevation = storage::elevation::read(&paths.output).map_err(invalid)?;
    println!("building bridges, tunnels and approach ramps");
    build::transport::build(
        source_osm_map,
        location_map,
        &streets_result.surfaces,
        &elevation.field,
        elevation.datum,
        &mut details,
        &paths.output.join("road-rises.txt"),
    )?;
    println!("validating and writing mapped details");
    model::map_details::validate(
        &details,
        location_map.buildings.len(),
        location_map.road_lines.len(),
    )
    .map_err(invalid)?;
    model::map_details::write(&details, &paths.output).map_err(invalid)?;

    let origin = location_map
        .origin
        .ok_or_else(|| invalid("imported location requires a geographic origin"))?;
    let south_west = [
        origin.lat - location_map.bounding_box.max.z.to_degrees() / 6371000.0,
        origin.lon
            + location_map.bounding_box.min.x.to_degrees()
                / 6371000.0
                / origin.lat.to_radians().cos(),
    ];
    let north_east = [
        origin.lat - location_map.bounding_box.min.z.to_degrees() / 6371000.0,
        origin.lon
            + location_map.bounding_box.max.x.to_degrees()
                / 6371000.0
                / origin.lat.to_radians().cos(),
    ];
    storage::manifest::write(
        &paths.output,
        &storage::manifest::Manifest {
            origin: Some(origin),
            bounds_metres: city_result.city.bounding_box,
            pavement_height_metres: world::street_surface::DEFAULT_PAVEMENT_HEIGHT,
            aerial_attribution: Some("AERIAL: ESRI / VANTOR / EARTHSTAR / GIS COMMUNITY".into()),
        },
    )?;

    println!("building labels");
    build::osm_labels::write_labels(&paths.output)?;

    println!("building collision data");
    let streets = build::streets::read(&paths.output, city_result.city.road_lines.len())?;
    storage::streets::write_compiled(&paths.output, &streets)?;

    Ok(GeneratedAssets {
        city: city_result.city,
        bbox: BoundingBox {
            min_lat: south_west[0],
            min_lon: south_west[1],
            max_lat: north_east[0],
            max_lon: north_east[1],
        },
    })
}

fn compile_imagery(
    data: &build::download_location::DownloadedSources,
    paths: &ImportPaths,
    generated: &GeneratedAssets,
    boundary: &build::osm::OSMArea,
) -> Result<()> {
    println!("Compiling satellite textures and inferred trees...");

    let bbox = generated.bbox;
    if !util::geometry::is_valid_for_imagery(&bbox) {
        return Err(invalid("invalid city geographic bounds"));
    }

    fs::create_dir_all(&paths.imagery)?;
    fs::create_dir_all(&paths.aerial)?;

    let origin = generated
        .city
        .origin
        .ok_or_else(|| invalid("imported location requires a geographic origin"))?;
    let origin = [origin.lat, origin.lon];
    let local_boundary = build::osm::local(boundary, util::geometry::new(origin[0], origin[1]));
    let all = &data.imagery_blocks;
    let mut catalog = Vec::new();

    let drafts: Vec<build::infer_trees::InferredTree> = serde_json::from_slice(&fs::read(
        &paths.downloads.join(build::infer_trees::TREES_FILE),
    )?)?;

    let mut tile_count = 0;

    for b in all.iter() {
        let meta = build::imagery::process_tiles(
            &paths
                .downloads
                .join(build::download_location::SATELLITE_IMAGES_DIR),
            &paths.imagery,
            &paths.aerial,
            b,
            origin,
        )?;
        // for each tile
        for t in meta.tiles {
            let [x1, y1, x2, y2] = t.bounds;
            if !local_boundary.intersects(&geo::Rect::new(
                geo::Coord { x: x1, y: y1 },
                geo::Coord { x: x2, y: y2 },
            )) {
                continue;
            }

            catalog.push(storage::aerial::TileRecord {
                bounds: t.bounds,
                name: t.name,
            });
            tile_count += 1;
        }
    }
    if tile_count == 0 {
        return Err(invalid("source returned no usable imagery"));
    }

    let obstacles = build::tree_obstacles::read_paths(
        &paths.output,
        [bbox.min_lon, bbox.min_lat, bbox.max_lon, bbox.max_lat],
    )?;

    // Read landscape items into an r-tree.
    let landscape_items = read_tree_positions(&paths.output)?;
    let mut index = rstar::RTree::new();
    for item in landscape_items {
        if item.id & (1_u64 << 62) != 0 {
            continue;
        }
        index.insert(item.coords);
    }

    // Start output
    let mut tree_output = storage::landscape::create_imagery_trees(&paths.output)?;

    let mut count = 0;

    let city = &generated.city;

    let inside = |p: XZ| {
        p.x >= city.bounding_box.min.x as f64
            && p.z >= city.bounding_box.min.z as f64
            && p.x <= city.bounding_box.max.x as f64
            && p.z <= city.bounding_box.max.z as f64
    };

    let mut append = |p: XZ, radius: f64, height: f64| -> Result<()> {
        let p = util::geometry::xz_map(p, |v| (v * 1000.).round() / 1000.);

        if !inside(p) || !local_boundary.intersects(&geo::Point::new(p.x, p.z)) {
            return Ok(());
        }

        if build::tree_obstacles::rejection_at(&obstacles, p).is_some() {
            return Ok(());
        }

        if index
            .nearest_neighbor(&p)
            .is_some_and(|q| (q.x - p.x).hypot(q.z - p.z) < 2.)
        {
            return Ok(());
        }

        let id = (1_u64 << 63) | count;

        storage::landscape::tree(
            &mut tree_output,
            &model::world::Tree {
                id,
                coords: p,
                height_meters: height.clamp(3.5, 14.),
                crown_radius_meters: radius.clamp(0.7, 4.5),
            },
        )?;

        index.insert(p);
        count += 1;
        Ok(())
    };

    for d in drafts {
        append(
            build::imagery::local(origin, [d.longitude, d.latitude]),
            d.crown_radius_meters,
            d.height_meters,
        )?;
    }

    storage::landscape::flush(&mut tree_output)?;

    storage::aerial::write(
        &paths.output,
        build::imagery::TILE,
        build::imagery::GUTTER,
        &catalog,
    )?;

    Ok(())
}

fn prepare(paths: &ImportPaths) -> Result<world::preparation::CompiledLocation> {
    world::preparation::compiled_location_load(&storage::city_assets::new(paths.output.clone()))
        .map_err(|e| invalid(format!("preparing staged import: {e}")))
}

fn bake(
    paths: &ImportPaths,
    city: &world::preparation::CompiledLocation,
    boundary: Option<&build::osm::OSMArea>,
) -> Result<()> {
    build_scene(
        city,
        boundary,
        &paths.decorations,
        &paths.scene,
        &paths.output,
    )?;
    Ok(())
}
