use super::*;
use std::io::{BufWriter, Read, Write};
struct Hgt {
    size: usize,
    south: f64,
    west: f64,
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
            south,
            west,
            samples: raw
                .chunks_exact(2)
                .map(|p| i16::from_be_bytes([p[0], p[1]]))
                .collect(),
        })
    }
    fn sample(&self, p: P) -> Result<f64> {
        let row = (self.south + 1.0 - p[0]) * (self.size - 1) as f64;
        let col = (p[1] - self.west) * (self.size - 1) as f64;
        let r = row.floor() as isize;
        let c = col.floor() as isize;
        if r < 1 || c < 1 || r + 2 >= self.size as isize || c + 2 >= self.size as isize {
            return Err(invalid("crop plus 80 m padding exceeds the HGT tile/filter coverage; use a smaller crop or --flat-terrain"));
        }
        let filtered = |r: isize, c: isize| -> Result<f64> {
            let mut sum = 0_i32;
            for (dy, wy) in [1, 2, 1].iter().enumerate() {
                for (dx, wx) in [1, 2, 1].iter().enumerate() {
                    let v = self.samples[(r - 1 + dy as isize) as usize * self.size
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
pub fn build(
    map: &Map,
    city: &str,
    source: Option<&Path>,
    output: &Path,
    flat: bool,
) -> Result<()> {
    if flat && source.is_some() {
        return Err(invalid(
            "--flat-terrain and --elevation are mutually exclusive",
        ));
    }
    let dem = source.map(Hgt::read).transpose()?;
    let datum = dem
        .as_ref()
        .map(|d| d.sample(map.origin))
        .transpose()?
        .unwrap_or(0.0);
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
            let value = dem
                .as_ref()
                .map(|d| d.sample(p))
                .transpose()?
                .unwrap_or(0.0)
                - datum;
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
    let saved_tile = if let Some(source) = source {
        let name = source
            .file_name()
            .ok_or_else(|| invalid("missing HGT filename"))?;
        fs::copy(source, output.join("elevation").join(name))?;
        let attribution = source.parent().unwrap_or(Path::new(".")).join("README.md");
        if attribution.is_file() {
            fs::copy(attribution, output.join("elevation/source-attribution.md"))?;
        }
        Some(name.to_string_lossy().into_owned())
    } else {
        None
    };
    json(
        &output.join("elevation/heightfield.json"),
        serde_json::json!({"compiler":"native-rust-v1","source_tile":saved_tile,"source_sha256":source.map(sha).transpose()?,"flat_terrain":flat,"grid_dimensions":size,"grid_step_metres":step,"local_datum_metres_egm96":datum,"processing":"3x3 [1,2,1] native-post Gaussian, bilinear resampling; heights relative to location origin. Flat terrain only when explicitly requested."}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
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
