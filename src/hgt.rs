use crate::location::{invalid, Result};
use crate::world::{Hgt, LocationMap, P};
use std::fs;
use std::io::Read;
use std::path::Path;

pub fn covering(source: &Path, map: &LocationMap) -> Result<(Hgt, Vec<std::path::PathBuf>)> {
    let first = Hgt::read(source)?;
    let lat = |z: f64| map.origin[0] - z.to_degrees() / 6371000.0;
    let lon =
        |x: f64| map.origin[1] + x.to_degrees() / 6371000.0 / map.origin[0].to_radians().cos();
    // Include terrain padding, grid rounding and the native-post filter.
    let south = lat(map.bounds[3] + 160.0).floor();
    let north = lat(map.bounds[1] - 160.0).floor();
    let west = lon(map.bounds[0] - 160.0).floor();
    let east = lon(map.bounds[2] + 160.0).floor();
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

impl Hgt {
    pub fn read(path: &Path) -> Result<Self> {
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
    pub fn sample(&self, p: P) -> Result<f64> {
        let row = (self.south + (self.rows - 1) as f64 / (self.size - 1) as f64 - p[0])
            * (self.size - 1) as f64;
        let col = (p[1] - self.west) * (self.size - 1) as f64;
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
