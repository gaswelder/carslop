use crate::{
    boundary::resolve_boundary,
    build_collision::build_collisions,
    build_furniture::build_furniture,
    build_landscape::build_landscape,
    build_scene::build_scene,
    build_surfaces, build_terrain,
    download::{download, download_tiles, DownloadOptions},
    imagery::{blocks, city_origin, local, mercator, process, write_file, GUTTER, TILE},
    location::{add, cross, dist, dot, invalid, length, line, mul, polygon, sub, Result},
    osm::{self, clip, number, OSMMap},
    sources::extract_osm_boundary,
    src_files::{read_landscape_file, read_trees_file, write_city_txt},
    street_surface::crc32,
    world::{Boundary, BoundingBox, Building, LocationMap, Road},
};
use geo::{BooleanOps, Contains, MultiLineString};
use geo::{Intersects, MapCoords};
use serde_json::Value;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    fs,
    path::Path,
    time::Duration,
};

const ELEVATION_URL: &str = "https://s3.amazonaws.com/elevation-tiles-prod/skadi";

pub(crate) fn add_limassol() -> Result<()> {
    let boundary_url = "https://api.openstreetmap.org/api/0.6/relation/2087230/full";
    let pbf_url = "https://download.geofabrik.de/europe/cyprus-latest.osm.pbf";

    let relation: i64 = regex::Regex::new(r"/relation/([1-9][0-9]*)/full$")?
        .captures(boundary_url)
        .ok_or_else(|| invalid("expected an OSM relation/full boundary URL"))?[1]
        .parse()?;
    let root = Path::new(crate::city_assets::LOCATIONS_DIRECTORY);
    let destination = root.join("limassol");
    fs::create_dir_all(&destination)?;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(900)))
        .build()
        .into();

    //
    // boundary osm, source
    //
    let path_sources_boundary_osm = root.join("limassol/sources/boundary.osm");

    //
    // region osm map, source
    //
    let path_sources_cyprus_osm_pbf = root.join("limassol/sources/cyprus.osm.pbf");

    //
    // osm map subset, source
    //
    let path_source_osm = root.join("limassol/source.osm");

    //
    // elevation tiles dir, source
    //
    let path_sources_elevation_dir = root.join("limassol/sources/elevation");

    //
    // ---
    //

    //
    // city txt, game asset
    //
    let path_city_txt = root.join("limassol/city.txt");

    //
    // streets, game asset
    //
    let path_streets_txt = root.join("limassol/streets.txt");

    //
    // landspace txt, game asset
    //
    let path_landscape_txt = root.join("limassol/landscape.txt");

    //
    // furniture txt, game asset
    //
    let path_furniture_txt = root.join("limassol/furniture.txt");

    //
    // Collision data, game asset
    //
    let path_streets_bin = root.join("limassol/streets.bin");

    //
    // Heightfield.txt, game asset
    //
    let path_elevation_heightfield_txt = root.join("limassol/elevation/heightfield.txt");

    //
    //
    //

    let path_elevation_dir = root.join("limassol/elevation");
    let path_manifest_json = root.join("limassol/manifest.json");

    let path_dir_imagery = root.join("limassol/imagery");
    let path_dir_aerial = root.join("limassol/aerial");
    let path_dir_downloads = root.join("limassol/imagery/downloads");
    let path_imagery_service = root.join("limassol/aerial/service.json");
    let path_imagery_trees = destination.join("imagery-trees.txt");
    let path_aerial_catalog = root.join("limassol/aerial/tiles.txt");

    //
    //
    //
    fs::create_dir_all(&path_elevation_dir)?;

    //
    // ---------------
    //

    // Download the city boundary separately.
    println!("downloading the requested boundary");
    download(
        &agent,
        boundary_url,
        &path_sources_boundary_osm,
        &DownloadOptions::default(),
    )?;

    // Parse the boundary.
    println!("parsing the boundary");
    let boundary_map = crate::osm::read(&path_sources_boundary_osm)?;
    let boundary = resolve_boundary(&boundary_map, relation)?;
    println!(
        "read the boundary {relation}: {} polygons",
        boundary.geometry.0.len()
    );

    // Check boundary size.
    let (width, height) = boundary.size();
    if width > 80000. || height > 80000. {
        return Err(invalid("the boundary exceeds compiler's 80 km limit"));
    }

    // Download the region map.
    println!("downloading the region map");
    download(
        &agent,
        pbf_url,
        &path_sources_cyprus_osm_pbf,
        &DownloadOptions::default(),
    )?;

    // Download relevant Mapzen/Skadi HGT tiles.
    let height_tiles = elevation_tiles(boundary.bbox());
    println!("downloading elevation tiles");
    for tile in &height_tiles {
        download(
            &agent,
            &format!("{ELEVATION_URL}/{}/{tile}.hgt.gz", &tile[..3]),
            &&path_sources_elevation_dir.join(format!("{tile}.hgt.gz")),
            &DownloadOptions::default(),
        )?;
    }

    // Write a subset of the map for our selected boundary.
    println!("extracting the requested area from the region map");
    extract_osm_boundary(
        &path_sources_cyprus_osm_pbf,
        &path_source_osm,
        boundary.bbox(),
    )?;

    //
    // Read the map.
    //
    let source_osm_map = osm::read(&path_source_osm)?;

    //
    // Convert the map to locationMap struct, and trim using the boundary.
    //
    let location_map = create_location_map(&source_osm_map, &boundary)?;

    //
    // Write the map to city.txt.
    //
    let city_checksum = write_city_txt(&location_map, &path_city_txt)?;

    //
    // Write ? to streets.txt
    //
    println!("building streets ({} segments)", location_map.roads.len());
    let surfaces = build_surfaces::build_surfaces(&location_map, city_checksum, &path_streets_txt)?;

    //
    // Write ? to landscape.txt
    //
    println!("Building mapped landscape and roadside controls...");
    build_landscape(
        &source_osm_map,
        &location_map,
        &surfaces,
        city_checksum,
        &path_landscape_txt,
    )?;

    //
    // Write ? to furnitute.txt
    //
    println!("  Placing road controls...");
    build_furniture(
        &source_osm_map,
        &location_map,
        &surfaces,
        city_checksum,
        &path_furniture_txt,
    )?;

    //
    // Write heightfield.txt
    //
    println!("Building elevation...");
    build_terrain::build_terrain(
        &location_map,
        city_checksum,
        &root
            .join("limassol/sources/elevation")
            .join(format!("{}.hgt.gz", height_tiles[0])),
        &path_elevation_heightfield_txt,
        &path_elevation_dir,
    )?;

    //
    //
    //
    let south_west = [
        location_map.origin[0] - location_map.bounds[3].to_degrees() / 6371000.0,
        location_map.origin[1]
            + location_map.bounds[0].to_degrees()
                / 6371000.0
                / location_map.origin[0].to_radians().cos(),
    ];
    let north_east = [
        location_map.origin[0] - location_map.bounds[1].to_degrees() / 6371000.0,
        location_map.origin[1]
            + location_map.bounds[2].to_degrees()
                / 6371000.0
                / location_map.origin[0].to_radians().cos(),
    ];
    let manifest = serde_json::json!({
        "name": "limassol",
        "boundary": boundary.to_geojson(),
    });
    fs::write(
        &path_manifest_json,
        serde_json::to_string_pretty(&manifest)? + "\n",
    )?;

    //
    // Read it back again...
    //
    let city_txt_source = std::fs::read_to_string(&path_city_txt)?;
    let streets_txt_source = std::fs::read_to_string(&path_streets_txt)?;

    //
    //
    //
    println!("Baking collision data...");
    let city = crate::city::City::parse(&city_txt_source).map_err(invalid)?;

    let streets_checksum = crc32(&streets_txt_source);
    build_collisions(
        &path_streets_txt,
        &path_streets_bin,
        &city,
        city_checksum,
        streets_txt_source.clone(),
        streets_checksum,
    )?;

    println!("Preparing satellite textures and imagery trees...");

    let bbox = BoundingBox {
        south: south_west[0],
        west: south_west[1],
        north: north_east[0],
        east: north_east[1],
    };
    if !bbox.is_valid_for_imagery() {
        return Err(invalid("invalid city geographic bounds"));
    }

    fs::create_dir_all(&path_dir_imagery)?;
    fs::create_dir_all(&path_dir_aerial)?;

    //
    //
    //

    let origin = city_origin(&city_txt_source)?;

    let manifest: Value = serde_json::from_slice(&std::fs::read(&path_manifest_json)?)?;

    let boundary = Boundary::from_manifest(&manifest)?;

    //
    //
    //
    let local_boundary = boundary.as_ref().map(|b| b.local(origin));

    //
    //
    //
    let mercator_boundary = boundary.as_ref().map(|b| {
        b.geometry.map_coords(|c| {
            let p = mercator(c.x, c.y);
            geo::Coord { x: p[0], y: p[1] }
        })
    });

    //
    //
    //
    let mut all = Vec::new();
    for b in blocks(bbox.to_array()) {
        //
        // if block does not intersect the mercator boundary,
        // skip it.
        //
        if let Some(g) = mercator_boundary.as_ref() {
            let rect = geo::Rect::new(
                geo::Coord {
                    x: b.extent[0],
                    y: b.extent[1],
                },
                geo::Coord {
                    x: b.extent[2],
                    y: b.extent[3],
                },
            );
            if !g.intersects(&rect) {
                continue;
            }
        }
        all.push(b)
    }

    //
    //
    //
    if all.is_empty() || all.len() > 2000 {
        return Err(invalid(
            "imagery job must contain 1..2000 blocks; crop the map",
        ));
    }

    //
    // download tiles
    //
    download_tiles(&path_dir_downloads, &path_imagery_service, &all)?;

    //
    //
    //
    let mut catalog = format!("AERIAL 1\nsize {TILE} {GUTTER}\n");

    //
    // drafts is a vec...
    //
    let mut drafts = vec![];

    //
    //
    //
    let mut tile_count = 0;

    //
    //
    //
    for b in all.iter() {
        let meta = process(
            &path_dir_downloads,
            &path_dir_imagery,
            &path_dir_aerial,
            b,
            origin,
        )?;
        let tiles = meta["tiles"]
            .as_array()
            .ok_or_else(|| invalid("missing block tiles"))?;
        let trees = meta["trees"]
            .as_array()
            .ok_or_else(|| invalid("missing block crowns"))?;

        //
        // for each tile
        //
        for t in tiles {
            //
            // get bounds from t
            //
            let bounds = t["bounds"]
                .as_array()
                .ok_or_else(|| invalid("missing tile bounds"))?;

            //
            // if bounds are outside local boundary, skip
            //
            let x1 = bounds[0].as_f64().unwrap();
            let y1 = bounds[1].as_f64().unwrap();
            let x2 = bounds[2].as_f64().unwrap();
            let y2 = bounds[3].as_f64().unwrap();
            if let Some(g) = &local_boundary {
                if !g.intersects(&geo::Rect::new(
                    geo::Coord { x: x1, y: y1 },
                    geo::Coord { x: x2, y: y2 },
                )) {
                    continue;
                }
            }

            catalog.push_str(&format!(
                "tile {:.6} {:.6} {:.6} {:.6} {} {:08x}\n",
                x1,
                y1,
                x2,
                y2,
                t["name"].as_str().unwrap(),
                t["crc"].as_u64().unwrap()
            ));
            tile_count += 1;
        }

        //
        // for each tree
        //
        for t in trees {
            drafts.push([
                t[0].as_f64().unwrap(),
                t[1].as_f64().unwrap(),
                t[2].as_f64().unwrap(),
            ]);
        }
    }
    if tile_count == 0 {
        return Err(invalid("source returned no usable imagery"));
    }

    //
    // read obstable paths?
    //
    let obstacles = crate::tree_obstacles::Obstacles::read_paths(
        &path_city_txt,
        &path_streets_txt,
        [bbox.west, bbox.south, bbox.east, bbox.north],
    )?;

    let expected = format!(
        "LANDSCAPE 1 {:08x} {:08x}",
        crate::street_surface::crc32(&city_txt_source),
        crate::imagery_trees::checksum(&path_streets_txt)?
    );

    //
    // Read landscape items into an r-tree.
    //
    let landscape_items = read_landscape_file(path_landscape_txt, expected.clone())?;
    let mut index = rstar::RTree::new();
    for item in landscape_items {
        index.insert([item.a, item.b]);
    }

    //
    // Start output
    //
    let mut output231231 = format!("{expected}\n");

    //
    // ?
    //
    let mut count = 0;

    //
    //
    //
    let inside = |p: [f64; 2]| {
        p[0] >= city.bounds[0][0] as f64
            && p[1] >= city.bounds[0][1] as f64
            && p[0] <= city.bounds[1][0] as f64
            && p[1] <= city.bounds[1][1] as f64
    };

    let mut append = |p: [f64; 2], radius: f64, height: f64| {
        let p = p.map(|v| (v * 1000.).round() / 1000.);
        if !inside(p)
            || local_boundary
                .as_ref()
                .is_some_and(|b| !b.intersects(&geo::Point::new(p[0], p[1])))
        {
            return;
        }
        if obstacles.rejection_at(p).is_some() {
            return;
        }
        if index
            .nearest_neighbor(&p)
            .is_some_and(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < 2.)
        {
            return;
        }
        let id = (1_u64 << 63) | count;

        //
        // append line to output
        //
        output231231.push_str(&format!(
            "t {id} {:.3} {:.3} {:.3} {:.3}\n",
            p[0],
            p[1],
            height.clamp(3.5, 14.),
            radius.clamp(0.7, 4.5)
        ));

        //
        //
        //
        index.insert(p);
        count += 1;
    };

    //
    // if trees file exists
    //
    if path_imagery_trees.exists() {
        let items = read_trees_file(&path_imagery_trees)?;
        for item in items {
            append([item.a, item.b], item.radius, item.height);
        }
    }

    //
    //
    //
    for d in drafts {
        let radius = d[2].clamp(0.7, 4.5);
        append(local(origin, [d[0], d[1]]), radius, radius * 2.8);
    }

    write_file(&path_imagery_trees, output231231.as_bytes())?;

    write_file(&path_aerial_catalog, catalog.as_bytes())?;

    // Bake collision data and all scene layers, clipping the visible terrain
    // mesh to the border. The elevation raster and geographic extent remain
    // rectangular for the terrain format; the polygon determines which roads,
    // buildings, and terrain triangles are imported.
    println!("Compiling collisions");
    build_collisions(
        &path_streets_txt,
        &path_streets_bin,
        &city,
        city_checksum,
        streets_txt_source,
        streets_checksum,
    )?;

    println!("Compiling scene");
    // let city = crate::city::City::load(&crate::city_assets::CityAsset::new(destination.clone()))
    //     .map_err(invalid)?;

    build_scene(
        &city,
        boundary.as_ref(),
        &destination.join("decorations.pack"),
        &destination.join("scene.pack"),
        &destination,
    )?;

    println!("done");
    Ok(())
}

pub fn elevation_tiles(bounds: BoundingBox) -> Vec<String> {
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

pub fn create_location_map(source: &OSMMap, boundary: &Boundary) -> Result<LocationMap> {
    let bbox = Some(boundary.bbox().to_array());
    let bbox = bbox.or(source.bounds).unwrap_or_else(|| {
        let mut b = [90.0_f64, 180.0_f64, -90.0_f64, -180.0_f64];
        for e in source.nodes.values() {
            let p = e.coord.unwrap();
            b[0] = b[0].min(p[0]);
            b[1] = b[1].min(p[1]);
            b[2] = b[2].max(p[0]);
            b[3] = b[3].max(p[1]);
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
    let origin = [(bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0];
    if origin[0].abs() > 85.0 {
        return Err(invalid(
            "locations within 5 degrees of the poles are unsupported",
        ));
    }
    let mut map = LocationMap {
        boundary: None,
        origin,
        bounds: [0.0; 4],
        nodes: vec![],
        roads: vec![],
        buildings: vec![],
    };
    let a = map.metres([bbox[0], bbox[1]]);
    let b = map.metres([bbox[2], bbox[3]]);
    map.bounds = [a[0], b[1], b[0], a[1]];
    map.boundary = boundary.map(|b| b.local(map.origin));
    // Bound geometry and height-field allocations before processing an accidental country extract.
    if b[0] - a[0] > 80000.0 || a[1] - b[1] > 80000.0 {
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
        let width = number(way.tag("width"))
            .unwrap_or_else(|| {
                let w = number(way.tag("lanes"))
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
            let Some((a, b)) = clip(*a, *b, map.bounds) else {
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
                    let (a, b) = ([pair[0].x, pair[0].y], [pair[1].x, pair[1].y]);
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
                        let key = (
                            (p[0] * 1000.0).round() as i64,
                            (p[1] * 1000.0).round() as i64,
                        );
                        ends[i] = *positions.entry(key).or_insert_with(|| {
                            let id = map.nodes.len();
                            map.nodes
                                .push([key.0 as f64 / 1000.0, key.1 as f64 / 1000.0]);
                            id
                        });
                    }
                    map.roads.push(Road {
                        ends,
                        width,
                        direction,
                        way: way.id,
                        kind: if major.contains(&kind) {
                            0
                        } else if matches!(kind, "service" | "living_street") {
                            2
                        } else {
                            1
                        },
                        lanes: (number(way.tag("lanes")).unwrap_or(if direction == 0 {
                            2.0
                        } else {
                            1.0
                        }) as usize)
                            .clamp(1, 6),
                    });
                }
            }
        }
    }
    if map.roads.is_empty() {
        return Err(invalid(
            "no public, ground-level drivable roads in this crop",
        ));
    }
    map.roads.sort_by(|a, b| {
        let key = |r: &Road| {
            let a = map.nodes[r.ends[0]];
            let b = map.nodes[r.ends[1]];
            (
                dist(a, b) < 25.0,
                length(mul(add(a, b), 0.5)),
                r.way,
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
        let height = number(way.tag("height"));
        map.buildings.push(Building {
            id: way.id,
            height: height
                .unwrap_or_else(|| {
                    number(way.tag("building:levels"))
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
            points,
        });
    }
    Ok(map)
}
