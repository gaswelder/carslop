use super::*;
use std::io::{BufWriter, Read, Write};
struct Hgt {
    size: usize,
    rows: usize,
    cols: usize,
    south: f64,
    west: f64,
    samples: Vec<i16>,
}
impl Hgt {
    fn covering(source: &Path, map: &Map) -> Result<(Self, Vec<std::path::PathBuf>)> {
        let first = Self::read(source)?;
        let lat = |z: f64| map.origin[0] - z.to_degrees() / 6371000.0;
        let lon =
            |x: f64| map.origin[1] + x.to_degrees() / 6371000.0 / map.origin[0].to_radians().cos();
        // Include terrain padding, grid rounding and the native-post filter.
        let south = lat(map.bounds[3] + 160.0).floor();
        let north = lat(map.bounds[1] - 160.0).floor();
        let west = lon(map.bounds[0] - 160.0).floor();
        let east = lon(map.bounds[2] + 160.0).floor();
        if south == first.south && north == first.south && west == first.west && east == first.west
        {
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
        let mut mosaic = Self {
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
                let tile = Self::read(&path).map_err(|e| {
                    invalid(format!("adjacent elevation tile {}: {e}", path.display()))
                })?;
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
                .chunks_exact(2)
                .map(|p| i16::from_be_bytes([p[0], p[1]]))
                .collect(),
        })
    }
    fn sample(&self, p: P) -> Result<f64> {
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
pub fn build(map: &Map, city: &str, source: &Path, output: &Path) -> Result<()> {
    let (dem, sources) = Hgt::covering(source, map)?;
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
    let mut file = BufWriter::new(fs::File::create(output.join("elevation/heightfield.txt"))?);
    writeln!(
        file,
        "ELEVATION 1 {:08x}\ngrid {} {} {:.3} {:.3} {:.3} {:.3}",
        crate::street_surface::crc32(city),
        size[0],
        size[1],
        min[0],
        min[1],
        step,
        datum
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
    let name = source
        .file_name()
        .ok_or_else(|| invalid("missing HGT filename"))?;
    let mut source_tiles = Vec::new();
    for path in &sources {
        let tile_name = path
            .file_name()
            .ok_or_else(|| invalid("missing HGT filename"))?;
        fs::copy(path, output.join("elevation").join(tile_name))?;
        source_tiles
            .push(serde_json::json!({"file":tile_name.to_string_lossy(),"sha256":sha(path)?}));
    }
    let attribution = source.parent().unwrap_or(Path::new(".")).join("README.md");
    if attribution.is_file() {
        fs::copy(attribution, output.join("elevation/source-attribution.md"))?;
    }
    let saved_tile = name.to_string_lossy();
    json(
        &output.join("elevation/heightfield.json"),
        serde_json::json!({"compiler":"native-rust-v1","source_tile":saved_tile,"source_sha256":sha(source)?,"source_tiles":source_tiles,"grid_dimensions":size,"grid_step_metres":step,"local_datum_metres_egm96":datum,"processing":"3x3 [1,2,1] native-post Gaussian, bilinear resampling; heights relative to location origin."}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn adjacent_tiles_are_filtered_continuously_across_the_seam() {
        let temp = crate::location::tests::Temp::new();
        for (x, offset) in [(6, 0), (7, 8)] {
            let values: Vec<u8> = (0..9)
                .flat_map(|_| (0..9).flat_map(|c| (100_i16 + offset + c).to_be_bytes()))
                .collect();
            fs::write(temp.0.join(format!("N50E{x:03}.hgt")), values).unwrap();
        }
        let map = Map {
            origin: [50.5, 7.0],
            bounds: [-100.0, -100.0, 100.0, 100.0],
            nodes: vec![],
            roads: vec![],
            buildings: vec![],
        };
        let source = temp.0.join("N50E006.hgt");
        let (dem, sources) = Hgt::covering(&source, &map).unwrap();
        assert_eq!(sources.len(), 2);
        for longitude in [6.95, 7.0, 7.05] {
            let expected = 100.0 + (longitude - 6.0) * 8.0;
            assert!((dem.sample([50.5, longitude]).unwrap() - expected).abs() < 1e-9);
        }
        fs::remove_file(temp.0.join("N50E007.hgt")).unwrap();
        assert!(Hgt::covering(&source, &map).is_err());
    }
    #[test]
    fn hgt_endian_orientation_interpolation_and_voids() {
        let temp = crate::location::tests::Temp::new();
        let path = temp.0.join("N35E033.hgt");
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
            assert!((dem.sample(p).unwrap() - expected).abs() < 1e-10);
        }
        assert!(dem.sample([36.0, 33.0]).is_err());
        samples[40] = -32768;
        save(&samples);
        assert!(Hgt::read(&path).unwrap().sample([35.5, 33.5]).is_err());
    }
}
