use crate::error::invalid;
use crate::error::Result;
use crate::geometry::LatLon;
use crate::source_location::SourceLocation;
use std::fs;
use std::io::Read;
use std::path::Path;

/// Decoded HGT elevation tile or mosaic of adjacent tiles.
pub struct Hgt {
    /// Samples per side of one source tile; size - 1 is the number of intervals per degree.
    size: usize,
    /// Number of sample rows in the tile or mosaic, ordered north to south.
    rows: usize,
    /// Number of sample columns in the tile or mosaic, ordered west to east.
    cols: usize,
    /// Latitude of the southern edge in degrees.
    south: f64,
    /// Longitude of the western edge in degrees.
    west: f64,
    /// Row-major elevations in metres above sea level; -32768 denotes a void.
    samples: Vec<i16>,
}

impl Hgt {
    fn read(path: &Path) -> Result<Self> {
        let name = path
            .file_name()
            .and_then(|s| s.to_str())
            .ok_or_else(|| invalid("invalid HGT filename"))?;
        let bytes = name.as_bytes();
        let mut origin = None;
        for i in 0..bytes.len().saturating_sub(6) {
            if matches!(bytes[i], b'N' | b'S')
                && matches!(bytes[i + 3], b'E' | b'W')
                && bytes[i + 1..i + 3].iter().all(u8::is_ascii_digit)
                && bytes[i + 4..i + 7].iter().all(u8::is_ascii_digit)
            {
                let south =
                    name[i + 1..i + 3].parse::<f64>()? * if bytes[i] == b'N' { 1.0 } else { -1.0 };
                let west = name[i + 4..i + 7].parse::<f64>()?
                    * if bytes[i + 3] == b'E' { 1.0 } else { -1.0 };
                origin = Some((south, west));
                break;
            }
        }
        let (south, west) = origin.ok_or_else(|| {
            invalid("HGT filename must contain tile coordinates, e.g. N35E033.hgt.gz")
        })?;
        let file = fs::File::open(path)?;
        let mut raw = vec![];
        if path.extension().is_some_and(|e| e == "gz") {
            flate2::read::GzDecoder::new(file)
                .take(100_000_001)
                .read_to_end(&mut raw)?;
        } else {
            file.take(100_000_001).read_to_end(&mut raw)?;
        }
        let size = ((raw.len() / 2) as f64).sqrt() as usize;
        if size < 4 || size * size * 2 != raw.len() || raw.len() > 100_000_000 {
            return Err(invalid("invalid HGT raster size"));
        }
        Ok(Self {
            size,
            rows: size,
            cols: size,
            south,
            west,
            samples: raw
                .as_chunks::<2>()
                .0
                .iter()
                .map(|p| i16::from_be_bytes([p[0], p[1]]))
                .collect(),
        })
    }
    pub fn sample(&self, p: LatLon) -> Result<f64> {
        let row = (self.south + (self.rows - 1) as f64 / (self.size - 1) as f64 - p.lat)
            * (self.size - 1) as f64;
        let col = (p.lon - self.west) * (self.size - 1) as f64;
        let r = row.floor() as isize;
        let c = col.floor() as isize;
        if r < 1 || c < 1 || r + 2 >= self.rows as isize || c + 2 >= self.cols as isize {
            return Err(invalid("crop plus 80 m padding exceeds the HGT tile/filter coverage; use a smaller crop or a tile covering the requested area"));
        }
        let filtered = |r: isize, c: isize| -> Result<f64> {
            let mut sum = 0_i32;
            for (dy, wy) in [1, 2, 1].iter().enumerate() {
                for (dx, wx) in [1, 2, 1].iter().enumerate() {
                    let v = self.samples[(r - 1 + dy as isize) as usize * self.cols
                        + (c - 1 + dx as isize) as usize];
                    if v == -32768 {
                        return Err(invalid("HGT contains voids in the requested area"));
                    }
                    sum += v as i32 * wy * wx;
                }
            }
            Ok(sum as f64 / 16.0)
        };
        let x = col - c as f64;
        let z = row - r as f64;
        Ok(filtered(r, c)? * (1.0 - x) * (1.0 - z)
            + filtered(r, c + 1)? * x * (1.0 - z)
            + filtered(r + 1, c)? * (1.0 - x) * z
            + filtered(r + 1, c + 1)? * x * z)
    }
}

pub fn covering(
    source: &Path,
    map: &SourceLocation<f64>,
) -> Result<(Hgt, Vec<std::path::PathBuf>)> {
    let origin = map
        .origin
        .ok_or_else(|| invalid("terrain requires a geographic origin"))?;
    let first = Hgt::read(source)?;
    let lat = |z: f64| origin.lat - z.to_degrees() / 6371000.0;
    let lon = |x: f64| origin.lon + x.to_degrees() / 6371000.0 / origin.lat.to_radians().cos();
    // Include terrain padding, grid rounding and the native-post filter.
    let south = lat(map.bounding_box.max.z + 160.0).floor();
    let north = lat(map.bounding_box.min.z - 160.0).floor();
    let west = lon(map.bounding_box.min.x - 160.0).floor();
    let east = lon(map.bounding_box.max.x + 160.0).floor();
    if south == first.south && north == first.south && west == first.west && east == first.west {
        return Ok((first, vec![source.to_owned()]));
    }
    if first.south < south || first.south > north || first.west < west || first.west > east {
        return Err(invalid(
            "supplied HGT tile does not cover the requested area",
        ));
    }
    let size = first.size;
    let rows = (north - south + 1.0) as usize * (size - 1) + 1;
    let cols = (east - west + 1.0) as usize * (size - 1) + 1;
    if rows.checked_mul(cols).is_none_or(|n| n > 100_000_000) {
        return Err(invalid("elevation mosaic exceeds 100 million samples"));
    }
    let mut mosaic = Hgt {
        size,
        rows,
        cols,
        south,
        west,
        samples: vec![-32768; rows * cols],
    };
    let mut paths = Vec::new();
    for y in south as i32..=north as i32 {
        for x in west as i32..=east as i32 {
            let stem = format!(
                "{}{:02}{}{:03}",
                if y >= 0 { "N" } else { "S" },
                y.abs(),
                if x >= 0 { "E" } else { "W" },
                x.abs()
            );
            let directory = source.parent().unwrap_or(Path::new("."));
            let path = if y as f64 == first.south && x as f64 == first.west {
                source.to_owned()
            } else {
                let plain = directory.join(format!("{stem}.hgt"));
                if plain.is_file() {
                    plain
                } else {
                    directory.join(format!("{stem}.hgt.gz"))
                }
            };
            let tile = Hgt::read(&path)
                .map_err(|e| invalid(format!("adjacent elevation tile {}: {e}", path.display())))?;
            if tile.size != size {
                return Err(invalid(
                    "adjacent elevation tiles must have the same resolution",
                ));
            }
            let row = (north as i32 - y) as usize * (size - 1);
            let col = (x - west as i32) as usize * (size - 1);
            for r in 0..size {
                mosaic.samples[(row + r) * cols + col..(row + r) * cols + col + size]
                    .copy_from_slice(&tile.samples[r * size..(r + 1) * size]);
            }
            paths.push(path);
        }
    }
    Ok((mosaic, paths))
}

#[cfg(test)]
mod raster_tests {
    use super::*;
    use crate::test_support;
    #[test]
    fn hgt_endian_orientation_interpolation_and_voids() {
        let temp = test_support::Temp::new();
        let path = temp.directory.join("N35E033.hgt");
        let mut samples: Vec<i16> = (0..9)
            .flat_map(|r| (0..9).map(move |c| 100 + r * 2 + c))
            .collect();
        let save = |values: &[i16]| {
            fs::write(
                &path,
                values
                    .iter()
                    .flat_map(|n| n.to_be_bytes())
                    .collect::<Vec<_>>(),
            )
            .unwrap()
        };
        save(&samples);
        let dem = Hgt::read(&path).unwrap();
        for (p, expected) in [
            ([35.5, 33.5], 112.0),
            ([35.625, 33.5], 110.0),
            ([35.5, 33.625], 113.0),
            ([35.5625, 33.5625], 111.5),
        ] {
            assert!((dem.sample(LatLon::new(p[0], p[1])).unwrap() - expected).abs() < 1e-10);
        }
        assert!(dem.sample(LatLon::new(36.0, 33.0)).is_err());
        samples[40] = -32768;
        save(&samples);
        assert!(Hgt::read(&path)
            .unwrap()
            .sample(LatLon::new(35.5, 33.5))
            .is_err());
    }
}
