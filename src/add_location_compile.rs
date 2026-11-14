use crate::asset_codecs;
use crate::build_barriers;
use crate::build_traffic_calming;
use crate::city_assets;
use crate::geometry::{add, cross, dist, dot, length, line, mul, polygon, sub};
use crate::geometry::{BoundingBox, BoundingBoxXZ, LatLon, XZ};
use crate::location;
use crate::scene_archive;
use crate::source_location;
use crate::tree_obstacles;
use crate::world;
use crate::{
    add_location_download, add_location_trees,
    boundary::resolve_boundary,
    build_collision::build_collisions,
    build_furniture::build_furniture,
    build_landscape::build_landscape,
    build_scene::build_scene,
    build_surfaces::write_streets_txt,
    build_terrain,
    error::{invalid, Result},
    file_io, imagery, osm,
    source_location::{SourceLocation, SourceRoad},
    src_files::{read_landscape_file, write_city_txt},
    world::Building,
};
use geo::{BooleanOps, Contains, Intersects, MultiLineString};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    fs,
    path::{Path, PathBuf},
};

/// Private compiler workspace; none of these intermediates enter the download directory.
struct ImportPaths {
    downloads: PathBuf,
    output: PathBuf,
    source_osm: PathBuf,
    city: PathBuf,
    streets: PathBuf,
    landscape: PathBuf,
    furniture: PathBuf,
    collisions: PathBuf,
    heightfield: PathBuf,
    elevation: PathBuf,
    manifest: PathBuf,
    imagery: PathBuf,
    aerial: PathBuf,
    trees: PathBuf,
    catalog: PathBuf,
    scene: PathBuf,
    decorations: PathBuf,
}
impl ImportPaths {
    fn new(output: &Path, downloads: &Path) -> Self {
        Self {
            downloads: downloads.into(),
            output: output.into(),
            source_osm: output.join("source.osm"),
            city: output.join("city.txt"),
            streets: output.join("streets.txt"),
            landscape: output.join("landscape.txt"),
            furniture: output.join("furniture.txt"),
            collisions: output.join("streets.bin"),
            heightfield: output.join("elevation/heightfield.txt"),
            elevation: output.join("elevation"),
            manifest: output.join("manifest.json"),
            imagery: output.join("imagery"),
            aerial: output.join("aerial"),
            trees: output.join("imagery-trees.txt"),
            catalog: output.join("aerial/tiles.txt"),
            scene: output.join("scene.pack"),
            decorations: output.join("decorations.pack"),
        }
    }
}

/// Compile entirely from local sources, retaining large render payloads on disk.
pub fn compile_location(download_dir: &Path) -> Result<location::Location> {
    let workspace = tempfile::Builder::new()
        .prefix("carslop-compile-")
        .tempdir()?;
    let paths = ImportPaths::new(workspace.path(), download_dir);
    let data = add_location_download::read_manifest(&paths.downloads)?;
    if data.height_tiles.is_empty() || data.imagery_blocks.is_empty() {
        return Err(invalid("downloaded location data is incomplete"));
    }
    if !paths
        .downloads
        .join(add_location_trees::TREES_FILE)
        .is_file()
    {
        return Err(invalid(
            "missing inferred trees; run infer_trees(download_dir) before compile",
        ));
    }
    let acquired = CompileSources {
        boundary: resolve_boundary(
            &osm::read(&paths.downloads.join(add_location_download::BOUNDARY_FILE))?,
            data.boundary_relation,
        )?,
        height_tiles: data.height_tiles.clone(),
    };
    let converted = convert_map(&paths, &acquired.boundary)?;
    let generated = generate_assets(&data.name, &paths, &acquired, &converted)?;
    drop(converted);
    compile_imagery(&data, &paths, &generated, &acquired.boundary)?;
    let prepared = prepare(&paths)?;
    bake(&paths, &prepared, Some(&acquired.boundary))?;
    drop(prepared);
    scene_archive::Archive::open(&paths.scene)?;
    location::Location::from_compilation(workspace).map_err(invalid)
}

struct ConvertedMap {
    osm: osm::OSMMap,
    location: SourceLocation<f64>,
}
fn convert_map(paths: &ImportPaths, boundary: &osm::OSMBoundary) -> Result<ConvertedMap> {
    fs::copy(
        &paths
            .downloads
            .join(add_location_download::LOCATION_OSM_FILE),
        &paths.source_osm,
    )?;

    // Read the map.
    let source_osm_map = osm::read(&paths.source_osm)?;

    // Convert the map to locationMap struct, and trim using the boundary.
    let location_map = create_location_map(&source_osm_map, boundary)?;

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
    boundary: osm::OSMBoundary,
    height_tiles: Vec<String>,
}

fn generate_assets(
    name: &str,
    paths: &ImportPaths,
    acquired: &CompileSources,
    converted: &ConvertedMap,
) -> Result<GeneratedAssets> {
    let boundary = &acquired.boundary;
    let height_tiles = &acquired.height_tiles;
    let source_osm_map = &converted.osm;
    let location_map = &converted.location;
    fs::create_dir_all(&paths.elevation)?;
    // Write the map to city.txt.
    let city_result = write_city_txt(location_map, &paths.city)?;

    println!(
        "building streets ({} segments)",
        location_map.road_lines.len()
    );
    let streets_result = write_streets_txt(location_map, &paths.streets, Some(source_osm_map))?;

    println!("building traffic calming");
    build_traffic_calming::build_traffic_calming(
        source_osm_map,
        location_map,
        &streets_result.surfaces,
        &paths.output.join("road-rises.txt"),
    )?;

    println!("building barriers");
    build_barriers::build_barriers(
        source_osm_map,
        location_map,
        &streets_result.surfaces,
        &paths.output.join("barriers.txt"),
    )?;

    println!("building landscape");
    build_landscape(
        source_osm_map,
        location_map,
        &streets_result.surfaces,
        &paths.landscape,
    )?;

    println!("building furniture");
    build_furniture(
        source_osm_map,
        location_map,
        &streets_result.surfaces,
        &paths.furniture,
    )?;

    println!("building elevation");
    build_terrain::build_terrain(
        location_map,
        &paths
            .downloads
            .join(add_location_download::ELEVATION_DATA_DIR)
            .join(format!("{}.hgt.gz", height_tiles[0])),
        &paths.heightfield,
        &paths.elevation,
    )?;

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
    let manifest = serde_json::json!({
        "name": name,
        "boundary": boundary.to_geojson(),
    });
    fs::write(
        &paths.manifest,
        serde_json::to_string_pretty(&manifest)? + "\n",
    )?;

    source_location::write_labels(paths.manifest.parent().unwrap())?;

    // Read it back again...
    let streets_txt_source = std::fs::read_to_string(&paths.streets)?;

    println!("building collision data");

    build_collisions(&paths.collisions, &city_result.city, streets_txt_source)?;

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
    data: &add_location_download::Manifest,
    paths: &ImportPaths,
    generated: &GeneratedAssets,
    boundary: &osm::OSMBoundary,
) -> Result<()> {
    println!("Compiling satellite textures and inferred trees...");

    let bbox = generated.bbox;
    if !bbox.is_valid_for_imagery() {
        return Err(invalid("invalid city geographic bounds"));
    }

    fs::create_dir_all(&paths.imagery)?;
    fs::create_dir_all(&paths.aerial)?;

    let origin = generated
        .city
        .origin
        .ok_or_else(|| invalid("imported location requires a geographic origin"))?;
    let origin = [origin.lat, origin.lon];
    let local_boundary = boundary.local(LatLon::new(origin[0], origin[1]));
    let all = &data.imagery_blocks;
    let mut catalog = format!("AERIAL 1\nsize {} {}\n", imagery::TILE, imagery::GUTTER);

    let drafts: Vec<add_location_trees::InferredTree> = serde_json::from_slice(&fs::read(
        &paths.downloads.join(add_location_trees::TREES_FILE),
    )?)?;

    let mut tile_count = 0;

    for b in all.iter() {
        let meta = imagery::process_tiles(
            &paths
                .downloads
                .join(add_location_download::SATELLITE_IMAGES_DIR),
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

            catalog.push_str(&format!(
                "tile {:.6} {:.6} {:.6} {:.6} {}\n",
                x1, y1, x2, y2, t.name
            ));
            tile_count += 1;
        }
    }
    if tile_count == 0 {
        return Err(invalid("source returned no usable imagery"));
    }

    let obstacles = tree_obstacles::Obstacles::read_paths(
        &paths.city,
        &paths.streets,
        [bbox.min_lon, bbox.min_lat, bbox.max_lon, bbox.max_lat],
    )?;

    // Read landscape items into an r-tree.
    let landscape_items = read_landscape_file(paths.landscape.clone())?;
    let mut index = rstar::RTree::new();
    for item in landscape_items {
        index.insert(item.coords);
    }

    // Start output
    let tree_file = fs::File::create(&paths.trees)?;
    let mut tree_output = asset_codecs::landscape::Writer::new(
        std::io::BufWriter::new(tree_file),
        asset_codecs::landscape::Precision::Imported,
    )?;

    let mut count = 0;

    let city = &generated.city;

    let inside = |p: XZ| {
        p.x >= city.bounding_box.min.x as f64
            && p.z >= city.bounding_box.min.z as f64
            && p.x <= city.bounding_box.max.x as f64
            && p.z <= city.bounding_box.max.z as f64
    };

    let mut append = |p: XZ, radius: f64, height: f64| -> Result<()> {
        let p = p.map(|v| (v * 1000.).round() / 1000.);

        if !inside(p) || !local_boundary.intersects(&geo::Point::new(p.x, p.z)) {
            return Ok(());
        }

        if obstacles.rejection_at(p).is_some() {
            return Ok(());
        }

        if index
            .nearest_neighbor(&p)
            .is_some_and(|q| (q.x - p.x).hypot(q.z - p.z) < 2.)
        {
            return Ok(());
        }

        let id = (1_u64 << 63) | count;

        tree_output.tree(&world::Tree {
            id,
            coords: p,
            height_meters: height.clamp(3.5, 14.),
            crown_radius_meters: radius.clamp(0.7, 4.5),
        })?;

        index.insert(p);
        count += 1;
        Ok(())
    };

    for d in drafts {
        append(
            imagery::local(origin, [d.longitude, d.latitude]),
            d.crown_radius_meters,
            d.height_meters,
        )?;
    }

    tree_output.flush()?;

    file_io::write_file(&paths.catalog, catalog.as_bytes())?;

    Ok(())
}

fn prepare(paths: &ImportPaths) -> Result<world::CompiledLocation> {
    world::CompiledLocation::load(&city_assets::CityAsset::new(paths.output.clone()))
        .map_err(|e| invalid(format!("preparing staged import: {e}")))
}

fn bake(
    paths: &ImportPaths,
    city: &world::CompiledLocation,
    boundary: Option<&osm::OSMBoundary>,
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

// More specific side tags override sidewalk:both and the general sidewalk tag.
fn sidewalks(way: &osm::OSMElement) -> crate::source_location::Sidewalks {
    let general = way.tag("sidewalk");
    let side = |key: &str, name: &str| {
        for value in [way.tag(key), way.tag("sidewalk:both")] {
            match value {
                "yes" => return true,
                "no" | "none" | "separate" => return false,
                _ => {}
            }
        }
        match general {
            "both" | "yes" => true,
            "no" | "none" | "separate" => false,
            "left" | "right" => general == name,
            _ => true,
        }
    };
    crate::source_location::Sidewalks {
        left: side("sidewalk:left", "left"),
        right: side("sidewalk:right", "right"),
    }
}

fn create_location_map(
    source: &osm::OSMMap,
    boundary: &osm::OSMBoundary,
) -> Result<SourceLocation<f64>> {
    let bbox = Some(boundary.bbox().to_array());
    let bbox = bbox.or(source.bounds).unwrap_or_else(|| {
        let mut b = [90.0_f64, 180.0_f64, -90.0_f64, -180.0_f64];
        for e in source.nodes.values() {
            let p = e.coord.unwrap();
            b[0] = b[0].min(p.lat);
            b[1] = b[1].min(p.lon);
            b[2] = b[2].max(p.lat);
            b[3] = b[3].max(p.lon);
        }
        b
    });
    if bbox.iter().any(|v| !v.is_finite())
        || bbox[0] >= bbox[2]
        || bbox[1] >= bbox[3]
        || bbox[0] < -90.0
        || bbox[2] > 90.0
        || bbox[1] < -180.0
        || bbox[3] > 180.0
    {
        return Err(invalid("invalid bounding box (south west north east)"));
    }
    let boundary = Some(boundary);
    let origin = LatLon::new((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0);
    if origin.lat.abs() > 85.0 {
        return Err(invalid(
            "locations within 5 degrees of the poles are unsupported",
        ));
    }
    let mut map = SourceLocation {
        origin: Some(origin),
        ..SourceLocation::<f64>::empty()
    };
    let a = map.metres(LatLon::new(bbox[0], bbox[1]));
    let b = map.metres(LatLon::new(bbox[2], bbox[3]));
    map.bounding_box = BoundingBoxXZ {
        min: XZ { x: a.x, z: b.z },
        max: XZ { x: b.x, z: a.z },
    };
    map.boundary = boundary.map(|b| b.local(origin));
    // Bound geometry and height-field allocations before processing an accidental country extract.
    if b.x - a.x > 80000.0 || a.z - b.z > 80000.0 {
        return Err(invalid(
            "crop exceeds 80 km; supply a smaller --bbox SOUTH WEST NORTH EAST",
        ));
    }
    let coords: BTreeMap<_, _> = source
        .nodes
        .iter()
        .map(|(id, e)| (*id, map.metres(e.coord.unwrap())))
        .collect();
    let mut positions = HashMap::new();
    let major = [
        "motorway",
        "trunk",
        "motorway_link",
        "trunk_link",
        "primary",
        "secondary",
        "tertiary",
        "primary_link",
        "secondary_link",
        "tertiary_link",
    ];
    let mut ways: Vec<_> = source.ways.values().collect();
    // Put circuits first so a circuit location starts on its drivable track.
    ways.sort_by_key(|way| way.tag("highway") != "raceway");
    for way in ways {
        let kind = way.tag("highway");
        if !major.contains(&kind)
            && ![
                "unclassified",
                "residential",
                "living_street",
                "service",
                "raceway",
            ]
            .contains(&kind)
        {
            continue;
        }
        // Circuit access restrictions describe public traffic, not whether it
        // can be driven in the game. Keep those roads in offline locations.
        if (kind != "raceway"
            && ["access", "vehicle", "motor_vehicle"]
                .iter()
                .any(|k| matches!(way.tag(k), "no" | "private")))
            || way.tag("area") == "yes"
            || (kind != "raceway"
                && (["bridge", "tunnel"]
                    .iter()
                    .any(|k| !matches!(way.tag(k), "" | "no"))
                    || !matches!(way.tag("layer"), "" | "0")))
        {
            continue;
        }
        let default = match kind {
            "motorway" => 12.0,
            "trunk" => 11.0,
            "primary" => 10.0,
            "secondary" => 9.0,
            "tertiary" => 8.0,
            "raceway" => 10.0,
            "service" | "living_street" => 5.5,
            _ => 6.5,
        };
        let width = osm::number(way.tag("width"))
            .unwrap_or_else(|| {
                let w = osm::number(way.tag("lanes"))
                    .filter(|n| *n > 0.0)
                    .map(|n| n * 3.1)
                    .unwrap_or(default);
                if major.contains(&kind) {
                    (w + 0.9).max(8.0)
                } else {
                    w
                }
            })
            .clamp(5.5, 18.0);
        let direction = if way.tag("oneway") == "-1" {
            -1
        } else if matches!(way.tag("oneway"), "yes" | "1" | "true")
            || way.tag("junction") == "roundabout"
        {
            1
        } else {
            0
        };
        for pair in way.nodes.windows(2) {
            let (Some(a), Some(b)) = (coords.get(&pair[0]), coords.get(&pair[1])) else {
                continue;
            };
            let Some((a, b)) = osm::clip(*a, *b, map.bounding_box) else {
                continue;
            };
            let forward = sub(b, a);
            let pieces = map.boundary.as_ref().map_or_else(
                || vec![line(&[a, b])],
                |boundary| {
                    boundary
                        .clip(&MultiLineString(vec![line(&[a, b])]), false)
                        .0
                },
            );
            for piece in pieces {
                for pair in piece.0.windows(2) {
                    let (a, b) = (
                        XZ {
                            x: pair[0].x,
                            z: pair[0].y,
                        },
                        XZ {
                            x: pair[1].x,
                            z: pair[1].y,
                        },
                    );
                    // Preserve the original OSM direction after boolean line clipping.
                    let (a, b) = if dot(sub(b, a), forward) < 0.0 {
                        (b, a)
                    } else {
                        (a, b)
                    };
                    if dist(a, b) < 0.5 {
                        continue;
                    }
                    let mut ends = [0; 2];
                    for (i, p) in [a, b].iter().enumerate() {
                        let key = ((p.x * 1000.0).round() as i64, (p.z * 1000.0).round() as i64);
                        ends[i] = *positions.entry(key).or_insert_with(|| {
                            let id = map.road_nodes.len();
                            map.road_nodes.push(
                                XZ {
                                    x: key.0 as f64 / 1000.0,
                                    z: key.1 as f64 / 1000.0,
                                }
                                .at_height(0.0),
                            );
                            id
                        });
                    }
                    map.road_lines.push(SourceRoad {
                        direction,
                        sidewalks: sidewalks(way),
                        osm_way_id: Some(way.id),
                        kind: if major.contains(&kind) {
                            0
                        } else if matches!(kind, "service" | "living_street") {
                            2
                        } else {
                            1
                        },
                        lanes: (osm::number(way.tag("lanes")).unwrap_or(if direction == 0 {
                            2.0
                        } else {
                            1.0
                        }) as usize)
                            .clamp(1, 6),
                        ..SourceRoad::new(ends, width)
                    });
                }
            }
        }
    }
    if map.road_lines.is_empty() {
        return Err(invalid(
            "no public, ground-level drivable roads in this crop",
        ));
    }
    map.road_lines.sort_by(|a, b| {
        let key = |r: &SourceRoad<f64>| {
            let a = map.road_nodes[r.ends[0]].xz();
            let b = map.road_nodes[r.ends[1]].xz();
            (
                dist(a, b) < 25.0,
                length(mul(add(a, b), 0.5)),
                r.osm_way_id,
                r.ends[0],
            )
        };
        let a = key(a);
        let b = key(b);
        a.0.cmp(&b.0)
            .then(a.1.total_cmp(&b.1))
            .then(a.2.cmp(&b.2))
            .then(a.3.cmp(&b.3))
    });
    let complex: BTreeSet<_> = source
        .relations
        .iter()
        .filter(|e| !e.tag("building").is_empty() && e.members.iter().any(|m| m.2 == "inner"))
        .flat_map(|e| e.members.iter().filter(|m| m.0 == "way").map(|m| m.1))
        .collect();
    for way in source.ways.values() {
        if matches!(way.tag("building"), "" | "no")
            || complex.contains(&way.id)
            || way.nodes.len() < 4
            || way.nodes.first() != way.nodes.last()
        {
            continue;
        }
        let Some(mut points) = way.nodes[..way.nodes.len() - 1]
            .iter()
            .map(|id| coords.get(id).copied())
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        let original = points.clone();
        points = original
            .iter()
            .enumerate()
            .filter(|(i, p)| dist(**p, original[(*i + original.len() - 1) % original.len()]) > 0.05)
            .map(|(_, p)| *p)
            .collect();
        if points.len() < 3 || points.iter().any(|p| !map.contains(*p)) {
            continue;
        }
        if map
            .boundary
            .as_ref()
            .is_some_and(|b| !b.contains(&polygon(&points)))
        {
            continue;
        }
        let area = points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .take(points.len())
            .map(|(a, b)| cross(*a, *b))
            .sum::<f64>()
            / 2.0;
        if area.abs() < 4.0 {
            continue;
        }
        if area < 0.0 {
            points.reverse();
        }
        let height = osm::number(way.tag("height"));
        map.buildings.push(Building {
            id: way.id as u64,
            height_meters: height
                .unwrap_or_else(|| {
                    osm::number(way.tag("building:levels"))
                        .filter(|n| *n > 0.0)
                        .map(|n| n * 3.1)
                        .unwrap_or(
                            if matches!(way.tag("building"), "apartments" | "commercial" | "office")
                            {
                                9.3
                            } else {
                                6.2
                            },
                        )
                })
                .clamp(2.5, 70.0),
            tagged: height.is_some(),
            base_y: 0.0,
            ground_outline: points,
        });
    }
    Ok(map)
}

#[cfg(test)]
#[path = "location_tests.rs"]
mod tests;
