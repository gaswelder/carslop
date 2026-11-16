//! Compile authored points into the same assets used by imported cities.

use crate::build;
use crate::model;
use crate::model::source_location::SourceLocation;
use crate::model::source_location::SourceRoad;
use crate::model::world::Building;
use crate::render;
use crate::storage;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XZ;
use crate::world;
use std::fs;
use std::path::Path;
use std::{
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
    let root = Path::new(storage::city_assets::LOCATIONS_DIRECTORY);
    let destination = root.join(name);
    if destination.exists() {
        return Err(invalid(format!(
            "location {name} already exists; refusing to replace it"
        )));
    }
    // Parse and validate before creating anything in the location store.
    let id = world::locations::register_file(source).map_err(invalid)?.0;
    let track = world::locations::try_track(&id).map_err(invalid)?;
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

fn compile(source: &Path, track: &world::locations::WorldLocation, output: &Path) -> Result<()> {
    println!("Sampling authored road...");
    let count = (world::locations::sampling_length(track).ceil() as usize).max(4);
    let samples: Vec<_> = (0..count)
        .map(|i| {
            world::locations::at_distance(
                track,
                world::locations::sampling_length(track) * i as f32 / count as f32,
            )
        })
        .collect();
    let city = track;
    let bounds = world::elevation::terrain_bounds(track);
    let map = SourceLocation {
        boundary: None,
        // Authored locations have local coordinates, with no geographic origin.
        origin: None,
        bounding_box: BoundingBoxXZ {
            min: util::geometry::xz_map(bounds.min, f64::from),
            max: util::geometry::xz_map(bounds.max, f64::from),
        },
        road_nodes: samples
            .iter()
            .map(|s| {
                util::geometry::at_height(
                    util::geometry::xz_map(util::geometry::xz(s.point), f64::from),
                    0.0,
                )
            })
            .collect(),
        road_lines: (0..count)
            .map(|i| SourceRoad {
                direction: 0,
                osm_way_id: Some(1),
                kind: 1,
                lanes: 2,
                ..model::source_location::new(
                    [i, (i + 1) % count],
                    (samples[i].width as f64 + samples[(i + 1) % count].width as f64) * 0.5,
                )
            })
            .collect(),
        buildings: world::buildings::buildings(city)
            .iter()
            .map(|b| Building {
                id: b.id,
                height_meters: b.height_meters as f64,
                base_y: 0.0,
                tagged: true,
                holes: Vec::new(),
                min_height_meters: 0.0,
                ground_outline: b
                    .ground_outline
                    .iter()
                    .map(|p| XZ {
                        x: p.x as f64,
                        z: p.z as f64,
                    })
                    .collect(),
            })
            .collect(),
        ..model::source_location::empty::<f64>()
    };
    let mut city_file = BufWriter::new(fs::File::create(output.join("city.txt"))?);
    storage::codec_city::write(
        &mut city_file,
        &map,
        storage::codec_city::Precision::Authored,
    )?;
    city_file.flush()?;
    println!("Building streets ({} segments)...", count);
    write_streets(&samples, output)?;
    write_elevation(track, output)?;
    write_landscape(track, model::world::trees(track), output)?;
    storage::codec_furniture::write(
        &mut fs::File::create(output.join("furniture.txt"))?,
        &[],
        &[],
    )?;
    fs::copy(source, output.join("source.track.json"))?;
    // Keep the existing label reader independent of the authoring format.
    fs::write(
        output.join("source.osm"),
        format!(
            "<osm version=\"0.6\"><way id=\"1\"><tag k=\"name\" v=\"{}\"/></way></osm>\n",
            quick_xml::escape::escape(&track.display_name),
        ),
    )?;
    fs::write(
        &output.join("manifest.json"),
        serde_json::to_string_pretty(&serde_json::json!({
            "compiler": "native-track-v1", "raceways_only": true,
            "source_file": "source.track.json",
            "bounds_metres": util::geometry::bounding_box_xz_to_array(map.bounding_box), "coordinates": "local metres: X east, Y up, Z south",
            "statistics": {"nodes": count, "road_segments": count, "buildings": map.buildings.len(), "trees": model::world::trees(track).len()}
        }))? + "\n",
    )?;
    println!("Building stored scenery decorations...");
    render::scene_archive::write(
        &output.join("decorations.pack"),
        render::scene::authored_decorations(track, world::locations::coastal_preset(&track.id)),
    )?;
    let path_city_txt = &output.join("city.txt");
    let path_streets_txt = &output.join("streets.txt");
    let city_txt_source = std::fs::read_to_string(&path_city_txt)?;
    let streets_txt_source = std::fs::read_to_string(&path_streets_txt)?;
    build::collision::build_collisions(
        &output.join("streets.bin"),
        &model::source_location::parse(&city_txt_source).map_err(invalid)?,
        streets_txt_source,
    )?;
    build::osm_labels::write_labels(output)?;
    // Validate using the regular runtime loader before publishing the directory.
    let loaded = world::preparation::compiled_location_load(&storage::city_assets::new(
        output.to_path_buf(),
    ))
    .map_err(invalid)?;
    build::scene::build_scene(
        &loaded,
        None,
        &output.join("decorations.pack"),
        &output.join("scene.pack"),
        output,
    )?;
    if world::roads::road_lines(&loaded).len() != count {
        return Err(invalid("compiled road count mismatch"));
    }
    Ok(())
}

fn write_streets(samples: &[world::locations::RoadSample], output: &Path) -> Result<()> {
    let path = output.join("streets.txt");
    let mut file = BufWriter::new(fs::File::create(&path)?);
    writeln!(file, "STREETS 1 {}", samples.len())?;
    let point = |sample: world::locations::RoadSample, offset: f32| {
        let normal = model::world::normal(sample);
        XZ {
            x: sample.point.x + normal.x * offset,
            z: sample.point.z + normal.z * offset,
        }
    };
    let mut triangle = |usage: model::world::TriangleUsage, mut p: [XZ<f32>; 3]| -> Result<()> {
        let cross = (p[1].x - p[0].x) * (p[2].z - p[0].z) - (p[1].z - p[0].z) * (p[2].x - p[0].x);
        if cross.abs() <= 0.00001 {
            return Ok(());
        }
        if cross < 0.0 {
            p.swap(1, 2);
        }
        writeln!(
            file,
            "t {} {} {} {} {} {} {}",
            storage::codec_streets::triangle_usage_street_id(usage).map_err(invalid)?,
            p[0].x,
            p[0].z,
            p[1].x,
            p[1].z,
            p[2].x,
            p[2].z
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
        triangle(
            model::world::TriangleUsage::Road(model::world::CurbStyle::Road),
            [(left[0]), (left[1]), (right[1])],
        )?;
        triangle(
            model::world::TriangleUsage::Road(model::world::CurbStyle::Road),
            [(left[0]), (right[1]), (right[0])],
        )?;
        for side in [-1.0, 1.0] {
            let edge = [
                point(a, side * a.width * 0.5),
                point(b, side * b.width * 0.5),
            ];
            let inside = [
                point(a, side * (a.width * 0.5 - 0.25)),
                point(b, side * (b.width * 0.5 - 0.25)),
            ];
            triangle(
                model::world::TriangleUsage::Marking,
                [(edge[0]), (edge[1]), (inside[1])],
            )?;
            triangle(
                model::world::TriangleUsage::Marking,
                [(edge[0]), (inside[1]), (inside[0])],
            )?;
        }
        if (a.distance / 2.8) as usize % 2 == 0 {
            let l = [point(a, -0.04), point(b, -0.04)];
            let r = [point(a, 0.04), point(b, 0.04)];
            triangle(
                model::world::TriangleUsage::Marking,
                [(l[0]), (l[1]), (r[1])],
            )?;
            triangle(
                model::world::TriangleUsage::Marking,
                [(l[0]), (r[1]), (r[0])],
            )?;
        }
    }
    for i in 0..samples.len() {
        let a = samples[i];
        let b = samples[(i + 1) % samples.len()];
        writeln!(
            file,
            "p {i} 2 {} {} {} {}",
            a.point.x, a.point.z, b.point.x, b.point.z
        )?;
        for side in [-1.0, 1.0] {
            let start = point(a, side * a.width * 0.5);
            let end = point(b, side * b.width * 0.5);
            let normal = util::geometry::xz_map(model::world::normal(a), |v| -side * v);
            writeln!(
                file,
                "c 2 {} {} {} {} {} {}",
                start.x, start.z, end.x, end.z, normal.x, normal.z
            )?;
        }
    }
    file.flush()?;
    Ok(())
}

fn write_elevation(track: &world::locations::WorldLocation, output: &Path) -> Result<()> {
    fs::create_dir_all(output.join("elevation"))?;
    let field =
        model::world::elevation(track).ok_or_else(|| invalid("authored track has no terrain"))?;
    let mut file = BufWriter::new(fs::File::create(output.join("elevation/heightfield.txt"))?);
    writeln!(
        file,
        "ELEVATION 1\ngrid {} {} {} {} {} 0",
        field.columns, field.rows, field.origin.x, field.origin.z, field.step
    )?;
    for row in 0..field.rows {
        for col in 0..field.columns {
            let height = world::elevation::point(field, col, row).y;
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
    track: &world::locations::WorldLocation,
    trees: &[world::landscape::Tree],
    output: &Path,
) -> Result<()> {
    let mut file = storage::codec_landscape::new(
        BufWriter::new(fs::File::create(output.join("landscape.txt"))?),
        storage::codec_landscape::Precision::Authored,
    )?;
    for triangle in model::world::ground_triangles(track) {
        storage::codec_landscape::ground(&mut file, triangle.material, triangle.points)?;
    }
    for tree in trees {
        storage::codec_landscape::tree(&mut file, tree)?;
    }
    storage::codec_landscape::flush(&mut file)?;
    Ok(())
}
