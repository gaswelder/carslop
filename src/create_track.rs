//! Compile authored points into the same assets used by imported cities.
use crate::error::{invalid, Result};
use crate::geometry::{BoundingBoxXZ, XZ};
use crate::source_location::{SourceLocation, SourceRoad};
use crate::world::Building;
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

fn compile(source: &Path, track: &crate::track::Location, output: &Path) -> Result<()> {
    println!("Sampling authored road...");
    let count = (track.sampling_length().ceil() as usize).max(4);
    let samples: Vec<_> = (0..count)
        .map(|i| track.at_distance(track.sampling_length() * i as f32 / count as f32))
        .collect();
    let city = track;
    let bounds = track.terrain_bounds();
    let map = SourceLocation {
        boundary: None,
        // Authored locations have local coordinates, with no geographic origin.
        origin: None,
        bounding_box: BoundingBoxXZ {
            min: bounds.min.map(f64::from),
            max: bounds.max.map(f64::from),
        },
        road_nodes: samples
            .iter()
            .map(|s| s.point.xz().map(f64::from).at_height(0.0))
            .collect(),
        road_lines: (0..count)
            .map(|i| SourceRoad {
                direction: 0,
                osm_way_id: Some(1),
                kind: 1,
                lanes: 2,
                ..SourceRoad::new(
                    [i, (i + 1) % count],
                    (samples[i].width as f64 + samples[(i + 1) % count].width as f64) * 0.5,
                )
            })
            .collect(),
        buildings: city
            .buildings()
            .iter()
            .map(|b| Building {
                id: b.id,
                height_meters: b.height_meters as f64,
                base_y: 0.0,
                tagged: true,
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
        ..SourceLocation::<f64>::empty()
    };
    let mut city_file = BufWriter::new(fs::File::create(output.join("city.txt"))?);
    crate::asset_codecs::city::write(
        &mut city_file,
        &map,
        crate::asset_codecs::city::Precision::Authored,
    )?;
    city_file.flush()?;
    println!("Building streets ({} segments)...", count);
    write_streets(&samples, output)?;
    write_elevation(track, output)?;
    write_landscape(track, track.trees(), output)?;
    crate::asset_codecs::furniture::write(
        &mut fs::File::create(output.join("furniture.txt"))?,
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
            "name": track.display_name, "compiler": "native-track-v1", "raceways_only": true,
            "source_file": "source.track.json",
            "bounds_metres": map.bounding_box.to_array(), "coordinates": "local metres: X east, Y up, Z south",
            "statistics": {"nodes": count, "road_segments": count, "buildings": map.buildings.len(), "trees": track.trees().len()}
        }))? + "\n",
    )?;
    println!("Building stored scenery decorations...");
    crate::scene_archive::write(
        &output.join("decorations.pack"),
        crate::scene::authored_decorations(track, crate::track::coastal_preset(&track.id)),
    )?;
    let path_city_txt = &output.join("city.txt");
    let path_streets_txt = &output.join("streets.txt");
    let city_txt_source = std::fs::read_to_string(&path_city_txt)?;
    let streets_txt_source = std::fs::read_to_string(&path_streets_txt)?;
    crate::build_collision::build_collisions(
        &output.join("streets.bin"),
        &crate::source_location::SourceLocation::<f32>::parse(&city_txt_source).map_err(invalid)?,
        streets_txt_source,
    )?;
    crate::source_location::write_labels(output)?;
    // Validate using the regular runtime loader before publishing the directory.
    let loaded = crate::world::CompiledLocation::load(&crate::city_assets::CityAsset::new(
        output.to_path_buf(),
    ))
    .map_err(invalid)?;
    crate::build_scene::build_scene(
        &loaded,
        None,
        &output.join("decorations.pack"),
        &output.join("scene.pack"),
        output,
    )?;
    if loaded.road_lines().len() != count {
        return Err(invalid("compiled road count mismatch"));
    }
    Ok(())
}

fn write_streets(samples: &[crate::track::RoadSample], output: &Path) -> Result<()> {
    let path = output.join("streets.txt");
    let mut file = BufWriter::new(fs::File::create(&path)?);
    writeln!(file, "STREETS 1 {}", samples.len())?;
    let point = |sample: crate::track::RoadSample, offset: f32| {
        let normal = sample.normal();
        crate::geometry::XZ {
            x: sample.point.x + normal.x * offset,
            z: sample.point.z + normal.z * offset,
        }
    };
    let mut triangle = |usage: crate::world::TriangleUsage, mut p: [XZ<f32>; 3]| -> Result<()> {
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
            usage.street_id().map_err(invalid)?,
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
            crate::world::TriangleUsage::Road(crate::world::CurbStyle::Road),
            [(left[0]), (left[1]), (right[1])],
        )?;
        triangle(
            crate::world::TriangleUsage::Road(crate::world::CurbStyle::Road),
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
                crate::world::TriangleUsage::Marking,
                [(edge[0]), (edge[1]), (inside[1])],
            )?;
            triangle(
                crate::world::TriangleUsage::Marking,
                [(edge[0]), (inside[1]), (inside[0])],
            )?;
        }
        if (a.distance / 2.8) as usize % 2 == 0 {
            let l = [point(a, -0.04), point(b, -0.04)];
            let r = [point(a, 0.04), point(b, 0.04)];
            triangle(
                crate::world::TriangleUsage::Marking,
                [(l[0]), (l[1]), (r[1])],
            )?;
            triangle(
                crate::world::TriangleUsage::Marking,
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
            let normal = a.normal().map(|v| -side * v);
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

fn write_elevation(track: &crate::track::Location, output: &Path) -> Result<()> {
    fs::create_dir_all(output.join("elevation"))?;
    let field = track
        .elevation()
        .ok_or_else(|| invalid("authored track has no terrain"))?;
    let mut file = BufWriter::new(fs::File::create(output.join("elevation/heightfield.txt"))?);
    writeln!(
        file,
        "ELEVATION 1\ngrid {} {} {} {} {} 0",
        field.columns, field.rows, field.origin.x, field.origin.z, field.step
    )?;
    for row in 0..field.rows {
        for col in 0..field.columns {
            let height = field.point(col, row).y;
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
    track: &crate::track::Location,
    trees: &[crate::landscape::Tree],
    output: &Path,
) -> Result<()> {
    let mut file = crate::asset_codecs::landscape::Writer::new(
        BufWriter::new(fs::File::create(output.join("landscape.txt"))?),
        crate::asset_codecs::landscape::Precision::Authored,
    )?;
    for triangle in track.ground_triangles() {
        file.ground(triangle.material, triangle.points)?;
    }
    for tree in trees {
        file.tree(tree)?;
    }
    file.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {

    #[test]
    fn invalid_circuit_authoring_input_is_rejected_before_building_runtime_geometry() {
        for points in [
            vec![],
            vec![[0.0, 0.0, 0.0, 1.0]; 4],
            vec![[f32::NAN, 0.0, 0.0, 8.0]; 4],
        ] {
            let circuit = crate::authoring::Circuit {
                control_points: points
                    .into_iter()
                    .map(|[x, y, z, width]| crate::authoring::ControlPoint {
                        position: crate::geometry::XYZ { x, y, z },
                        width,
                    })
                    .collect(),
                coastal: false,
            };
            assert!(circuit.build(crate::world::LocationId::Compact).is_err());
        }
    }
}
