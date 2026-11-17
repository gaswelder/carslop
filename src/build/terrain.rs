use crate::build;
use crate::build::hgt::covering;
use crate::model::source_location::SourceLocation;
use crate::storage;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::XZ;
use std::path::Path;

pub fn build_terrain(map: &SourceLocation<f64>, source: &Path, directory: &Path) -> Result<()> {
    let origin = map
        .origin
        .ok_or_else(|| invalid("terrain requires a geographic origin"))?;
    let (dem, sources) = covering(source, map)?;
    let datum = build::hgt::sample(&dem, origin)?;
    let step = 20.0;
    let min = XZ {
        x: ((map.bounding_box.min.x - 80.0) / step).floor() * step,
        z: ((map.bounding_box.min.z - 80.0) / step).floor() * step,
    };
    let size = [
        ((map.bounding_box.max.x + 80.0 - min.x) / step).ceil() as usize + 1,
        ((map.bounding_box.max.z + 80.0 - min.z) / step).ceil() as usize + 1,
    ];
    if size.iter().any(|n| *n > 4096) {
        return Err(invalid(
            "terrain grid exceeds runtime limit of 4096 posts per axis",
        ));
    }
    println!("  Sampling {} x {} elevation posts...", size[0], size[1]);
    storage::elevation::write_sampled(
        directory,
        storage::elevation::Grid {
            columns: size[0],
            rows: size[1],
            origin: min,
            step,
            datum,
        },
        |col, row| {
            let x = min.x + col as f64 * step;
            let z = min.z + row as f64 * step;
            let p = util::geometry::new(
                origin.lat - z.to_degrees() / 6371000.0,
                origin.lon + x.to_degrees() / 6371000.0 / origin.lat.to_radians().cos(),
            );
            Ok(build::hgt::sample(&dem, p)? - datum)
        },
    )?;
    for path in &sources {
        storage::elevation::copy_source(directory, path)?;
    }
    Ok(())
}
