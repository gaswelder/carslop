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
use std::path::PathBuf;

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
    storage::city::write_file(output, &map, storage::city::Precision::Authored)?;
    println!("Building streets ({} segments)...", count);
    write_streets(&samples, output)?;
    write_elevation(track, output)?;
    write_landscape(track, model::world::trees(track), output)?;
    storage::furniture::write_file(output, &[], &[])?;
    fs::copy(source, output.join("source.track.json"))?;
    storage::labels::write(
        output,
        &model::location::Labels {
            streets: vec![(1, track.display_name.clone())],
            ..model::location::Labels::default()
        },
    )?;
    storage::manifest::write(
        output,
        &storage::manifest::Manifest {
            origin: None,
            bounds_metres: BoundingBoxXZ {
                min: util::geometry::xz_map(map.bounding_box.min, |value| value as f32),
                max: util::geometry::xz_map(map.bounding_box.max, |value| value as f32),
            },
            pavement_height_metres: world::street_surface::DEFAULT_PAVEMENT_HEIGHT,
            aerial_attribution: None,
        },
    )?;
    println!("Building stored scenery decorations...");
    render::scene_archive::write(
        &output.join("decorations.pack"),
        render::scene::authored_decorations(track, world::locations::coastal_preset(&track.id)),
    )?;
    let city = storage::city_assets::get_city(&storage::city_assets::new(output.to_path_buf()))
        .map_err(invalid)?;
    let streets = build::streets::read(output, city.road_lines.len())?;
    storage::streets::write_compiled(output, &streets)?;
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
    let mut file =
        build::streets::create(output, samples.len(), build::streets::Precision::Authored)?;
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
        build::streets::triangle(
            &mut file,
            build::streets::triangle_usage_street_id(usage).map_err(invalid)?,
            p,
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
        build::streets::path(
            &mut file,
            i,
            &[util::geometry::xz(a.point), util::geometry::xz(b.point)],
        )?;
        for side in [-1.0, 1.0] {
            let start = point(a, side * a.width * 0.5);
            let end = point(b, side * b.width * 0.5);
            let normal = util::geometry::xz_map(model::world::normal(a), |v| -side * v);
            build::streets::curb(&mut file, 2, [start, end], normal)?;
        }
    }
    build::streets::flush(&mut file)?;
    Ok(())
}

fn write_elevation(track: &world::locations::WorldLocation, output: &Path) -> Result<()> {
    let field =
        model::world::elevation(track).ok_or_else(|| invalid("authored track has no terrain"))?;
    storage::elevation::write(output, field)?;
    Ok(())
}

fn write_landscape(
    track: &world::locations::WorldLocation,
    trees: &[world::landscape::Tree],
    output: &Path,
) -> Result<()> {
    let mut file = storage::landscape::create(output, storage::landscape::Precision::Authored)?;
    for triangle in model::world::ground_triangles(track) {
        storage::landscape::ground(&mut file, triangle.material, triangle.points)?;
    }
    for tree in trees {
        storage::landscape::tree(&mut file, tree)?;
    }
    storage::landscape::flush(&mut file)?;
    Ok(())
}
