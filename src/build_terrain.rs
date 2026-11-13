use crate::error::{invalid, Result};
use crate::geometry::{LatLon, XZ};
use crate::hgt::covering;
use crate::source_location::SourceLocation;
use std::fs;
use std::io::{BufWriter, Write};
use std::path::Path;

pub fn build_terrain(
    map: &SourceLocation<f64>,
    source: &Path,
    heightfield_path: &Path,
    tiles_directory: &Path,
) -> Result<()> {
    let origin = map
        .origin
        .ok_or_else(|| invalid("terrain requires a geographic origin"))?;
    let (dem, sources) = covering(source, map)?;
    let datum = dem.sample(origin)?;
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
    let mut file = BufWriter::new(fs::File::create(heightfield_path)?);
    writeln!(
        file,
        "ELEVATION 1\ngrid {} {} {:.3} {:.3} {:.3} {:.3}",
        size[0], size[1], min.x, min.z, step, datum
    )?;
    for row in 0..size[1] {
        for col in 0..size[0] {
            let x = min.x + col as f64 * step;
            let z = min.z + row as f64 * step;
            let p = LatLon::new(
                origin.lat - z.to_degrees() / 6371000.0,
                origin.lon + x.to_degrees() / 6371000.0 / origin.lat.to_radians().cos(),
            );
            let value = dem.sample(p)? - datum;
            if !value.is_finite() || value.abs() >= 10000.0 {
                return Err(invalid(
                    "terrain exceeds the supported relative height range",
                ));
            }
            if col > 0 {
                write!(file, " ")?;
            }
            write!(file, "{value:.3}")?;
        }
        writeln!(file)?;
    }
    file.flush()?;
    for path in &sources {
        let tile_name = path
            .file_name()
            .ok_or_else(|| invalid("missing HGT filename"))?;
        fs::copy(path, tiles_directory.join(tile_name))?;
    }
    Ok(())
}
