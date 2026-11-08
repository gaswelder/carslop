use crate::hgt::covering;
use crate::location::{invalid, Result};
use crate::world::LocationMap;
use std::fs;
use std::io::{BufWriter, Write};
use std::path::Path;

pub fn build_terrain(
    map: &LocationMap,
    checksum: u32,
    source: &Path,
    heightfield_path: &Path,
    tiles_directory: &Path,
) -> Result<()> {
    let (dem, sources) = covering(source, map)?;
    let datum = dem.sample(map.origin)?;
    let step = 20.0;
    let min = [
        ((map.bounds[0] - 80.0) / step).floor() * step,
        ((map.bounds[1] - 80.0) / step).floor() * step,
    ];
    let size = [
        ((map.bounds[2] + 80.0 - min[0]) / step).ceil() as usize + 1,
        ((map.bounds[3] + 80.0 - min[1]) / step).ceil() as usize + 1,
    ];
    if size.iter().any(|n| *n > 4096) {
        return Err(invalid(
            "terrain grid exceeds runtime limit of 4096 posts per axis",
        ));
    }
    let mut file = BufWriter::new(fs::File::create(heightfield_path)?);
    writeln!(
        file,
        "ELEVATION 1 {:08x}\ngrid {} {} {:.3} {:.3} {:.3} {:.3}",
        checksum, size[0], size[1], min[0], min[1], step, datum
    )?;
    for row in 0..size[1] {
        for col in 0..size[0] {
            let x = min[0] + col as f64 * step;
            let z = min[1] + row as f64 * step;
            let p = [
                map.origin[0] - z.to_degrees() / 6371000.0,
                map.origin[1] + x.to_degrees() / 6371000.0 / map.origin[0].to_radians().cos(),
            ];
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
