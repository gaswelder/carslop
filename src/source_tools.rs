//! Source preparation commands, built into the game executable.
use crate::location_compiler::{extract_osmpbf, invalid, merge, BoundingBox, Result};
use std::{collections::BTreeMap, fs, path::Path};

pub fn aerial(image: &Path, export: &Path, output: &Path, size: u32, gutter: u32) -> Result<()> {
    let im = image::ImageReader::open(image)?.decode()?.to_rgb8();
    let (width, height) = im.dimensions();
    if size == 0 || width == 0 || height == 0 || width % size != 0 || height % size != 0 {
        return Err(invalid(
            "image dimensions must be multiples of positive tile size",
        ));
    }
    let side = size
        .checked_add(
            gutter
                .checked_mul(2)
                .ok_or_else(|| invalid("gutter overflow"))?,
        )
        .ok_or_else(|| invalid("tile size overflow"))?;
    let len = (side as usize)
        .checked_mul(side as usize)
        .and_then(|v| v.checked_mul(3))
        .filter(|v| *v <= 256 * 1024 * 1024)
        .ok_or_else(|| invalid("tile exceeds 256 MiB"))?;
    let metadata = fs::read(export)?;
    let v: serde_json::Value = serde_json::from_slice(&metadata)?;
    let e = &v["extent"];
    let get = |k: &str| -> Result<f64> {
        e[k].as_f64()
            .filter(|v| v.is_finite())
            .ok_or_else(|| invalid(format!("invalid extent {k}")))
    };
    let (xmin, ymin, xmax, ymax) = (get("xmin")?, get("ymin")?, get("xmax")?, get("ymax")?);
    if xmin >= xmax || ymin >= ymax {
        return Err(invalid("invalid export extent"));
    }
    let local = |mx: f64, my: f64| {
        let lat = (2. * (my / 6378137.).exp().atan() - std::f64::consts::FRAC_PI_2).to_degrees();
        let lon = (mx / 6378137.).to_degrees();
        [
            (lon - 33.320).to_radians() * 6371000. * 35.162_f64.to_radians().cos(),
            -(lat - 35.162).to_radians() * 6371000.,
        ]
    };
    let nw = local(xmin, ymax);
    let se = local(xmax, ymin);
    let (nx, nz) = (width / size, height / size);
    fs::create_dir_all(output)?;
    let mut index = format!("AERIAL 1\nsize {size} {gutter}\n");
    for z in 0..nz {
        for x in 0..nx {
            let mut raw = Vec::with_capacity(len);
            for py in 0..side {
                for px in 0..side {
                    let sx = (x as i64 * size as i64 + px as i64 - gutter as i64)
                        .clamp(0, width as i64 - 1) as u32;
                    let sy = (z as i64 * size as i64 + py as i64 - gutter as i64)
                        .clamp(0, height as i64 - 1) as u32;
                    raw.extend_from_slice(&im.get_pixel(sx, sy).0);
                }
            }
            let raw = crate::aerial::encode_jpeg(&raw, side, side)?;
            let name = format!("{x}_{z}.jpg");
            fs::write(output.join(&name), &raw)?;
            let bounds = [
                nw[0] + (se[0] - nw[0]) * x as f64 / nx as f64,
                nw[1] + (se[1] - nw[1]) * z as f64 / nz as f64,
                nw[0] + (se[0] - nw[0]) * (x + 1) as f64 / nx as f64,
                nw[1] + (se[1] - nw[1]) * (z + 1) as f64 / nz as f64,
            ];
            index.push_str(&format!(
                "tile {:.6} {:.6} {:.6} {:.6} {name} {:08x}\n",
                bounds[0],
                bounds[1],
                bounds[2],
                bounds[3],
                crate::street_surface::crc32_bytes(&raw)
            ));
        }
    }
    fs::write(output.join("source-export.json"), metadata)?;
    fs::write(output.join("tiles.txt"), index)?;
    println!("{} aerial tiles", nx * nz);
    Ok(())
}

pub fn run(args: Vec<std::ffi::OsString>) -> Result<()> {
    use std::path::PathBuf;
    let command = args[0].to_str().ok_or_else(|| invalid("invalid command"))?;
    let mut positional = Vec::new();
    let mut options = BTreeMap::new();
    let mut bbox = BoundingBox {
        south: 35.10,
        west: 33.25,
        north: 35.23,
        east: 33.47,
    };
    let mut seen_bbox = false;
    let mut values = args[1..].iter();
    while let Some(arg) = values.next() {
        if arg == "--bbox" && command == "extract-region" {
            if seen_bbox {
                return Err(invalid("duplicate --bbox"));
            }
            seen_bbox = true;
            for v in [
                &mut bbox.south,
                &mut bbox.west,
                &mut bbox.north,
                &mut bbox.east,
            ] {
                *v = values
                    .next()
                    .and_then(|v| v.to_str())
                    .ok_or_else(|| invalid("--bbox requires SOUTH WEST NORTH EAST"))?
                    .parse()?;
            }
        } else if arg.to_string_lossy().starts_with('-') {
            let key = arg.to_str().ok_or_else(|| invalid("invalid option"))?;
            let valid = match command {
                "build-aerial-tiles" => ["--image", "--export", "--output"].contains(&key),
                "prepare-city-imagery" => {
                    ["--city-data", "--work-dir", "--limit-blocks"].contains(&key)
                }
                "import-imagery-trees" => ["--candidates", "--city-data"].contains(&key),
                "preview-trees" => {
                    ["--image", "--export", "--output", "--osm", "--city-data"].contains(&key)
                }
                _ => false,
            };
            if !valid {
                return Err(invalid(format!("unexpected option: {key}")));
            }
            let value = values
                .next()
                .ok_or_else(|| invalid(format!("{key} requires a value")))?;
            if options.insert(key, value).is_some() {
                return Err(invalid(format!("duplicate {key}")));
            }
        } else {
            positional.push(PathBuf::from(arg));
        }
    }
    match command {
        "merge-osm" if positional.len() == 3 => {
            merge(&positional[0], &positional[1], &positional[2])
        }
        "extract-region" if positional.len() == 2 => {
            extract_osmpbf(&positional[0], &positional[1], bbox)
        }
        "prepare-city-imagery" if positional.is_empty() => {
            let path = |key| -> Result<&Path> {
                options
                    .get(key)
                    .map(Path::new)
                    .ok_or_else(|| invalid(format!("missing {key}")))
            };
            let limit = options
                .get("--limit-blocks")
                .map(|v| -> Result<usize> {
                    Ok(v.to_str()
                        .ok_or_else(|| invalid("invalid block count"))?
                        .parse()?)
                })
                .transpose()?;
            crate::location_compiler::prepare_imagery(
                path("--city-data")?,
                path("--work-dir")?,
                limit,
            )
        }
        "import-imagery-trees" if positional.is_empty() => {
            let path = |key| -> Result<&Path> {
                options
                    .get(key)
                    .map(Path::new)
                    .ok_or_else(|| invalid(format!("missing {key}")))
            };
            crate::imagery_trees::import(path("--candidates")?, path("--city-data")?)
        }
        "preview-trees" if positional.is_empty() => {
            let path = |key| -> Result<&Path> {
                options
                    .get(key)
                    .map(Path::new)
                    .ok_or_else(|| invalid(format!("missing {key}")))
            };
            crate::tree_preview::run(
                path("--image")?,
                path("--export")?,
                path("--output")?,
                options.get("--osm").map(Path::new),
                options.get("--city-data").map(Path::new),
            )
        }
        "build-aerial-tiles" if positional.is_empty() => {
            let path = |key| -> Result<&Path> {
                options
                    .get(key)
                    .map(Path::new)
                    .ok_or_else(|| invalid(format!("missing {key}")))
            };
            aerial(
                path("--image")?,
                path("--export")?,
                path("--output")?,
                512,
                8,
            )?;
            Ok(())
        }
        _ => Err(invalid("invalid source tool arguments; see --help")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn aerial_gutters_checksums_and_validation() {
        let dir = tempfile::tempdir().unwrap();
        let image = dir.path().join("image.png");
        let export = dir.path().join("export.json");
        let out = dir.path().join("tiles");
        image::RgbImage::from_fn(128, 128, |x, y| {
            image::Rgb([x as u8, y as u8, (x + y) as u8])
        })
        .save(&image)
        .unwrap();
        fs::write(
            &export,
            r#"{"extent":{"xmin":3709000,"ymin":4185800,"xmax":3709100,"ymax":4185900}}"#,
        )
        .unwrap();
        aerial(&image, &export, &out, 64, 4).unwrap();
        let index = fs::read_to_string(out.join("tiles.txt")).unwrap();
        assert_eq!(index.lines().count(), 6);
        for line in index.lines().skip(2) {
            let f: Vec<_> = line.split_whitespace().collect();
            let raw = fs::read(out.join(f[5])).unwrap();
            assert_eq!(
                image::load_from_memory(&raw)
                    .unwrap()
                    .to_rgb8()
                    .dimensions(),
                (72, 72)
            );
            assert_eq!(
                crate::street_surface::crc32_bytes(&raw),
                u32::from_str_radix(f[6], 16).unwrap()
            );
        }
        let a = image::open(out.join("0_0.jpg"))
            .unwrap()
            .to_rgb8()
            .into_raw();
        let b = image::open(out.join("1_0.jpg"))
            .unwrap()
            .to_rgb8()
            .into_raw();
        for y in 0..72 {
            assert!(a[(y * 72 + 64) * 3..(y * 72 + 72) * 3]
                .iter()
                .zip(&b[y * 72 * 3..(y * 72 + 8) * 3])
                .all(|(a, b)| a.abs_diff(*b) <= 12));
        }
        assert!(a[..3].iter().all(|v| *v <= 5));
        assert!(aerial(&image, &export, &out, 0, 4).is_err());
        assert!(aerial(&image, &export, &out, 63, 4).is_err());
    }
    #[test]
    fn source_command_arguments() {
        let args = |a: &[&str]| run(a.iter().map(Into::into).collect());
        for removed in ["--size", "--gutter", "--preview-output"] {
            assert!(args(&["build-aerial-tiles", removed, "1"]).is_err());
        }
        assert!(args(&["extract-region", "a", "b", "--bbox", "NaN", "0", "1", "1"]).is_err());
        assert!(args(&["merge-osm", "a", "b"]).is_err());
        assert!(args(&["build-aerial-tiles", "--image", "a", "--image", "b"]).is_err());
    }
}
