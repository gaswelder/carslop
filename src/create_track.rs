//! Compile authored points into the same assets used by imported cities.
use crate::build_surfaces;
use crate::location::{invalid, sha, Result};
use crate::world::{Building, LocationMap, Road};
use std::fs;
use std::path::Path;
use std::{
    fmt::Write as _,
    io::{BufWriter, Write},
    path::PathBuf,
};

pub fn create_track(source: &Path) -> Result<PathBuf> {
    let stem = source
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| invalid("track JSON must have a UTF-8 filename"))?;
    let name = stem.strip_suffix(".track").unwrap_or(stem);
    if name.is_empty()
        || name.len() > 64
        || !name.as_bytes()[0].is_ascii_alphanumeric()
        || !name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
    {
        return Err(invalid("track filename must contain a valid location name"));
    }
    let root = Path::new(crate::city_assets::LOCATIONS_DIRECTORY);
    let destination = root.join(name);
    if destination.exists() {
        return Err(invalid(format!(
            "location {name} already exists; refusing to replace it"
        )));
    }
    // Parse and validate before creating anything in the location store.
    let id = crate::track::register_file(source).map_err(invalid)?.0;
    let track = id.try_track().map_err(invalid)?;
    fs::create_dir_all(root)?;
    let staging = tempfile::Builder::new()
        .prefix(".create-track-")
        .tempdir_in(root)?;
    compile(source, &track, staging.path())?;
    if destination.exists() {
        return Err(invalid(
            "location appeared during compilation; refusing to replace it",
        ));
    }
    fs::rename(staging.path(), &destination)?;
    Ok(destination)
}

pub fn compile(source: &Path, track: &crate::track::Track, output: &Path) -> Result<()> {
    println!("Sampling authored road...");
    let count = (track.length().ceil() as usize).max(4);
    let samples: Vec<_> = (0..count)
        .map(|i| track.at_distance(track.length() * i as f32 / count as f32))
        .collect();
    let scenery = crate::scene::authored_scenery(track);
    let bounds = track.terrain_bounds();
    let map = LocationMap {
        boundary: None,
        // Authored locations have local coordinates, with no geographic origin.
        origin: [0.0; 2],
        bounds: [
            bounds[0][0] as f64,
            bounds[0][1] as f64,
            bounds[1][0] as f64,
            bounds[1][1] as f64,
        ],
        nodes: samples
            .iter()
            .map(|s| [s.point[0] as f64, s.point[2] as f64])
            .collect(),
        roads: (0..count)
            .map(|i| Road {
                ends: [i, (i + 1) % count],
                width: (samples[i].width as f64 + samples[(i + 1) % count].width as f64) * 0.5,
                direction: 0,
                way: 1,
                kind: 1,
                lanes: 2,
            })
            .collect(),
        buildings: scenery
            .buildings
            .iter()
            .map(|b| Building {
                id: b.id as i64,
                height: b.height as f64,
                tagged: true,
                points: b
                    .polygon
                    .iter()
                    .map(|p| [p[0] as f64, p[1] as f64])
                    .collect(),
            })
            .collect(),
    };
    let mut city = format!(
        "CITY 2\nbounds {} {} {} {}\n",
        map.bounds[0], map.bounds[1], map.bounds[2], map.bounds[3]
    );
    for p in &map.nodes {
        writeln!(city, "n {} {}", p[0], p[1])?;
    }
    for road in &map.roads {
        writeln!(
            city,
            "r {} {} {} 0 1 1 2",
            road.ends[0], road.ends[1], road.width
        )?;
    }
    for building in &map.buildings {
        write!(
            city,
            "b {} {} 1 {}",
            building.id,
            building.height,
            building.points.len()
        )?;
        for p in &building.points {
            write!(city, " {} {}", p[0], p[1])?;
        }
        writeln!(city)?;
    }
    fs::write(output.join("city.txt"), &city)?;
    println!("Building streets ({} segments)...", count);
    let city_checksum = crate::street_surface::crc32(&city);
    let streets_checksum = write_streets(&samples, city_checksum, output)?;
    let checksums = crate::street_surface::SourceChecksums {
        city: city_checksum,
        streets: streets_checksum,
    };
    write_elevation(track, checksums.city, output)?;
    write_landscape(track, &scenery.trees, checksums, output)?;
    fs::write(
        output.join("furniture.txt"),
        format!(
            "FURNITURE 3 {:08x} {:08x}\n",
            checksums.city, checksums.streets
        ),
    )?;
    fs::copy(source, output.join("source.track.json"))?;
    // Keep the existing label reader independent of the authoring format.
    fs::write(
        output.join("source.osm"),
        format!(
            "<osm version=\"0.6\"><way id=\"1\"><tag k=\"name\" v=\"{}\"/></way></osm>\n",
            quick_xml::escape::escape(&track.name),
        ),
    )?;
    fs::write(
        &output.join("manifest.json"),
        serde_json::to_string_pretty(&serde_json::json!({
            "name": track.name, "compiler": "native-track-v1", "raceways_only": true,
            "source_file": "source.track.json", "source_sha256": sha(source)?,
            "bounds_metres": map.bounds, "coordinates": "local metres: X east, Y up, Z south",
            "statistics": {"nodes": count, "road_segments": count, "buildings": map.buildings.len(), "trees": scenery.trees.len()}
        }))? + "\n",
    )?;
    println!("Building stored scenery decorations...");
    crate::scene_archive::write(
        &output.join("decorations.pack"),
        crate::scene::authored_decorations(track),
    )?;
    let path_city_txt = &output.join("city.txt");
    let path_streets_txt = &output.join("streets.txt");
    let city_txt_source = std::fs::read_to_string(&path_city_txt)?;
    let streets_txt_source = std::fs::read_to_string(&path_streets_txt)?;
    crate::build_collision::build_collisions(
        &output.join("streets.txt"),
        &output.join("streets.bin"),
        &crate::city::City::parse(&city_txt_source).map_err(invalid)?,
        city_checksum,
        streets_txt_source,
        streets_checksum,
    )?;
    // Validate using the regular runtime loader before publishing the directory.
    let loaded = crate::city::City::load(&crate::city_assets::CityAsset::new(output.to_path_buf()))
        .map_err(invalid)?;
    crate::build_scene::build_scene(
        &loaded,
        None,
        &output.join("decorations.pack"),
        &output.join("scene.pack"),
        output,
    )?;
    if loaded.streets.len() != count {
        return Err(invalid("compiled road count mismatch"));
    }
    Ok(())
}

fn write_streets(
    samples: &[crate::track::RoadSample],
    checksum: u32,
    output: &Path,
) -> Result<u32> {
    let path = output.join("streets.txt");
    let mut file = BufWriter::new(fs::File::create(&path)?);
    writeln!(file, "STREETS 1 {} {checksum:08x}", samples.len())?;
    let point = |sample: crate::track::RoadSample, offset: f32| {
        let normal = sample.normal();
        [
            sample.point[0] + normal[0] * offset,
            sample.point[2] + normal[1] * offset,
        ]
    };
    let mut triangle = |style: usize, mut p: [[f32; 2]; 3]| -> Result<()> {
        let cross =
            (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]);
        if cross.abs() <= 0.00001 {
            return Ok(());
        }
        if cross < 0.0 {
            p.swap(1, 2);
        }
        writeln!(
            file,
            "t {style} {} {} {} {} {} {}",
            p[0][0], p[0][1], p[1][0], p[1][1], p[2][0], p[2][1]
        )?;
        Ok(())
    };
    // Emit each strip with widths at both endpoints, rather than widening a
    // constant-width segment. Shared endpoints leave no steps at width changes.
    for i in 0..samples.len() {
        let a = samples[i];
        let b = samples[(i + 1) % samples.len()];
        let left = [point(a, -a.width * 0.5), point(b, -b.width * 0.5)];
        let right = [point(a, a.width * 0.5), point(b, b.width * 0.5)];
        triangle(1, [left[0], left[1], right[1]])?;
        triangle(1, [left[0], right[1], right[0]])?;
        for side in [-1.0, 1.0] {
            let edge = [
                point(a, side * a.width * 0.5),
                point(b, side * b.width * 0.5),
            ];
            let inside = [
                point(a, side * (a.width * 0.5 - 0.25)),
                point(b, side * (b.width * 0.5 - 0.25)),
            ];
            triangle(4, [edge[0], edge[1], inside[1]])?;
            triangle(4, [edge[0], inside[1], inside[0]])?;
        }
        if (a.distance / 2.8) as usize % 2 == 0 {
            let l = [point(a, -0.04), point(b, -0.04)];
            let r = [point(a, 0.04), point(b, 0.04)];
            triangle(4, [l[0], l[1], r[1]])?;
            triangle(4, [l[0], r[1], r[0]])?;
        }
    }
    for i in 0..samples.len() {
        let a = samples[i];
        let b = samples[(i + 1) % samples.len()];
        writeln!(
            file,
            "p {i} 2 {} {} {} {}",
            a.point[0], a.point[2], b.point[0], b.point[2]
        )?;
        for side in [-1.0, 1.0] {
            let start = point(a, side * a.width * 0.5);
            let end = point(b, side * b.width * 0.5);
            let normal = a.normal().map(|v| -side * v);
            writeln!(
                file,
                "c 2 {} {} {} {} {} {}",
                start[0], start[1], end[0], end[1], normal[0], normal[1]
            )?;
        }
    }
    file.flush()?;
    build_surfaces::crc_file(&path)
}

fn write_elevation(track: &crate::track::Track, checksum: u32, output: &Path) -> Result<()> {
    fs::create_dir_all(output.join("elevation"))?;
    let field = track
        .terrain()
        .ok_or_else(|| invalid("authored track has no terrain"))?;
    let mut file = BufWriter::new(fs::File::create(output.join("elevation/heightfield.txt"))?);
    writeln!(
        file,
        "ELEVATION 1 {checksum:08x}\ngrid {} {} {} {} {} 0",
        field.columns, field.rows, field.origin[0], field.origin[1], field.step
    )?;
    for row in 0..field.rows {
        for col in 0..field.columns {
            let height = field.point(col, row)[1];
            if !height.is_finite() || height.abs() >= 10_000.0 {
                return Err(invalid(
                    "authored elevation must be within 10000 metres of zero",
                ));
            }
            write!(file, "{height} ")?;
        }
        writeln!(file)?;
    }
    file.flush()?;
    Ok(())
}

fn write_landscape(
    track: &crate::track::Track,
    trees: &[crate::landscape::Tree],
    checksums: crate::street_surface::SourceChecksums,
    output: &Path,
) -> Result<()> {
    let field = track.terrain().unwrap();
    let mut file = BufWriter::new(fs::File::create(output.join("landscape.txt"))?);
    writeln!(
        file,
        "LANDSCAPE 1 {:08x} {:08x}",
        checksums.city, checksums.streets
    )?;
    for row in 0..field.rows - 1 {
        for col in 0..field.columns - 1 {
            let points = [
                field.point(col, row),
                field.point(col + 1, row),
                field.point(col + 1, row + 1),
                field.point(col, row + 1),
            ];
            let road = track.project(points[0]);
            let style = if track.is_coastal() && road.offset < 0.0 {
                6
            } else if (col + row * 2) % 5 == 0 {
                1
            } else {
                0
            };
            for tri in [[0, 1, 2], [0, 2, 3]] {
                writeln!(
                    file,
                    "a {style} {} {} {} {} {} {}",
                    points[tri[0]][0],
                    points[tri[0]][2],
                    points[tri[1]][0],
                    points[tri[1]][2],
                    points[tri[2]][0],
                    points[tri[2]][2]
                )?;
            }
        }
    }
    for tree in trees {
        writeln!(
            file,
            "t {} {} {} {} {}",
            tree.id, tree.point[0], tree.point[1], tree.height, tree.radius
        )?;
    }
    file.flush()?;
    Ok(())
}
