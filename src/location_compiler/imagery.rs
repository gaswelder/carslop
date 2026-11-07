//! Resumable city-wide aerial preparation. Downloads are explicit; gameplay is offline.
use super::{invalid, Result};
use crate::{
    imagery_trees::atomic,
    tree_preview::{proposals, Geo},
};
use serde_json::{json, Value};
use std::{
    fs,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicUsize, Ordering},
        mpsc,
    },
    time::Duration,
};
pub(super) const SERVICE: &str =
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer";
pub(super) const RES: f64 = 0.5971642835598172; // zoom-18 Web Mercator sampling, ~0.49 m on the ground here
pub(super) const CORE: u32 = 2048;
pub(super) const PAD: u32 = 32;
pub(super) const TILE: u32 = 512;
pub(super) const GUTTER: u32 = 8;
#[derive(Clone, Debug)]
pub(super) struct Block {
    x: usize,
    y: usize,
    pub(super) extent: [f64; 4],
}
pub(super) fn mercator(lon: f64, lat: f64) -> [f64; 2] {
    [
        lon.to_radians() * 6378137.,
        (std::f64::consts::FRAC_PI_4 + lat.to_radians() / 2.)
            .tan()
            .ln()
            * 6378137.,
    ]
}
pub(super) fn blocks(bbox: [f64; 4]) -> Vec<Block> {
    let sw = mercator(bbox[1], bbox[0]);
    let ne = mercator(bbox[3], bbox[2]);
    let span = CORE as f64 * RES;
    let nx = ((ne[0] - sw[0]) / span).ceil() as usize;
    let ny = ((ne[1] - sw[1]) / span).ceil() as usize;
    (0..ny)
        .flat_map(|y| {
            (0..nx).map(move |x| {
                let west = sw[0] + x as f64 * span;
                let north = ne[1] - y as f64 * span;
                Block {
                    x,
                    y,
                    extent: [
                        west - PAD as f64 * RES,
                        north - span - PAD as f64 * RES,
                        west + span + PAD as f64 * RES,
                        north + PAD as f64 * RES,
                    ],
                }
            })
        })
        .collect()
}
fn image_path(work: &Path, b: &Block) -> PathBuf {
    work.join(format!("{}_{}.jpg", b.x, b.y))
}
fn url(b: &Block) -> String {
    let e = b.extent;
    format!("{SERVICE}/export?bbox={:.9},{:.9},{:.9},{:.9}&bboxSR=3857&imageSR=3857&size={},{}&format=jpg&f=image",e[0],e[1],e[2],e[3],CORE+2*PAD,CORE+2*PAD)
}
fn valid_image(bytes: &[u8]) -> bool {
    image::load_from_memory(bytes)
        .is_ok_and(|im| im.width() == CORE + 2 * PAD && im.height() == CORE + 2 * PAD)
}
fn fetch(agent: &ureq::Agent, work: &Path, b: &Block) -> Result<()> {
    let path = image_path(work, b);
    if path.exists() && valid_image(&fs::read(&path)?) {
        return Ok(());
    }
    let mut last = String::new();
    for attempt in 0..3 {
        let result = (|| -> Result<()> {
            let bytes = agent
                .get(url(b))
                .call()?
                .body_mut()
                .with_config()
                .limit(32 * 1024 * 1024)
                .read_to_vec()?;
            if !valid_image(&bytes) {
                return Err(invalid(
                    "imagery service returned an invalid image or error response",
                ));
            }
            atomic(&path, &bytes)?;
            Ok(())
        })();
        match result {
            Ok(()) => return Ok(()),
            Err(e) => last = e.to_string(),
        }
        if attempt < 2 {
            std::thread::sleep(Duration::from_secs(1 << attempt));
        }
    }
    Err(invalid(format!("block {},{}: {last}", b.x, b.y)))
}
pub(super) fn download(work: &Path, service_path: &Path, all: &[Block]) -> Result<()> {
    fs::create_dir_all(work)?;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .into();
    if !service_path.exists() {
        let metadata = agent
            .get(format!("{SERVICE}?f=pjson"))
            .call()?
            .body_mut()
            .read_to_string()?;
        let value: Value = serde_json::from_str(&metadata)?;
        if value.get("error").is_some() {
            return Err(invalid("imagery service metadata request failed"));
        }
        atomic(&service_path, metadata.as_bytes())?;
    }
    let next = AtomicUsize::new(0);
    let (send, recv) = mpsc::channel();
    std::thread::scope(|scope| {
        for _ in 0..4 {
            let send = send.clone();
            let agent = agent.clone();
            let next = &next;
            scope.spawn(move || loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                if i >= all.len() {
                    break;
                }
                let result = fetch(&agent, work, &all[i]).map_err(|e| e.to_string());
                let failed = result.is_err();
                let _ = send.send(result);
                if failed {
                    next.store(all.len(), Ordering::Relaxed);
                    break;
                }
            });
        }
        drop(send);
        let mut count = 0;
        let mut error = None;
        for result in recv {
            match result {
                Ok(()) => {
                    count += 1;
                    if count % 10 == 0 || count == all.len() {
                        println!("Images ready: {count}/{}", all.len());
                    }
                }
                Err(e) => error = Some(e),
            }
        }
        if let Some(e) = error {
            Err(invalid(e))
        } else if count != all.len() {
            Err(invalid("incomplete image downloads"))
        } else {
            Ok(())
        }
    })
}
pub(super) fn city_origin(text: &str) -> Result<[f64; 2]> {
    let line = text
        .lines()
        .find(|l| l.starts_with("origin "))
        .ok_or_else(|| invalid("missing city origin"))?;
    let v: Vec<_> = line.split_whitespace().collect();
    Ok([v[1].parse()?, v[2].parse()?])
}
pub(super) fn local(origin: [f64; 2], ll: [f64; 2]) -> [f64; 2] {
    [
        (ll[0] - origin[1]).to_radians() * 6371000. * origin[0].to_radians().cos(),
        -(ll[1] - origin[0]).to_radians() * 6371000.,
    ]
}
fn blank(im: &image::RgbImage, x: u32, y: u32) -> bool {
    // Blank service coverage is white; do not turn that into white terrain.
    let mut nonwhite = 0;
    let mut n = 0;
    for yy in (y..y + TILE).step_by(16) {
        for xx in (x..x + TILE).step_by(16) {
            let p = im.get_pixel(xx, yy).0;
            if p.iter().any(|v| *v < 245) {
                nonwhite += 1;
            }
            n += 1;
        }
    }
    nonwhite * 100 < n
}
pub(super) fn process(
    downloads: &Path,
    work: &Path,
    aerial: &Path,
    b: &Block,
    origin: [f64; 2],
) -> Result<Value> {
    fs::create_dir_all(work)?;
    let meta_path = work.join(format!("block-{}_{}.json", b.x, b.y));
    let image_file = image_path(downloads, b);
    let hash = super::sha(&image_file)?;
    if meta_path.exists() {
        let value: Value = serde_json::from_slice(&fs::read(&meta_path)?)?;
        if value["image_sha256"] == hash
            && value["jpeg_quality"] == 80
            && value["tiles"].as_array().is_some_and(|tiles| {
                tiles.iter().all(|t| {
                    let Some(name) = t["name"].as_str() else {
                        return false;
                    };
                    let Ok(bytes) = fs::read(aerial.join(name)) else {
                        return false;
                    };
                    name.ends_with(".jpg")
                        && t["crc"].as_u64() == Some(crc32fast::hash(&bytes) as u64)
                })
            })
        {
            return Ok(value);
        }
    }
    let im = image::open(&image_file)?.to_rgb8();
    let side = CORE + 2 * PAD;
    let geo = Geo {
        extent: b.extent,
        width: side as usize,
        height: side as usize,
    };
    let scale = geo.metres_per_pixel();
    let mut tiles = vec![];
    let mut missing = 0;
    for z in 0..CORE / TILE {
        for x in 0..CORE / TILE {
            let px = PAD + x * TILE;
            let py = PAD + z * TILE;
            if blank(&im, px, py) {
                missing += 1;
                continue;
            }
            let raw = image::imageops::crop_imm(
                &im,
                px - GUTTER,
                py - GUTTER,
                TILE + 2 * GUTTER,
                TILE + 2 * GUTTER,
            )
            .to_image()
            .into_raw();
            let raw = crate::aerial::encode_jpeg(&raw, TILE + 2 * GUTTER, TILE + 2 * GUTTER)?;
            let name = format!("{}_{}_{}_{}.jpg", b.x, b.y, x, z);
            atomic(&aerial.join(&name), &raw)?;
            let nw = local(origin, geo.lonlat(px as f64, py as f64));
            let se = local(origin, geo.lonlat((px + TILE) as f64, (py + TILE) as f64));
            tiles.push(
                json!({"name":name,"bounds":[nw[0],nw[1],se[0],se[1]],"crc":crc32fast::hash(&raw)}),
            );
        }
    }
    let crowns = proposals(&im, scale);
    let trees: Vec<_> = crowns
        .into_iter()
        .filter(|p| {
            p.x >= PAD as usize
                && p.y >= PAD as usize
                && p.x < (PAD + CORE) as usize
                && p.y < (PAD + CORE) as usize
        })
        .map(|p| {
            let ll = geo.lonlat(p.x as f64, p.y as f64);
            [ll[0], ll[1], p.radius]
        })
        .collect();
    let value = json!({"jpeg_quality":80,"image_sha256":hash,"extent":b.extent,"tiles":tiles,"missing_tiles":missing,"trees":trees});
    atomic(&meta_path, &serde_json::to_vec(&value)?)?;
    Ok(value)
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::location_compiler::{prepare_imagery, AerialPaths, ImageryPaths};

    #[test]
    fn preparation_uses_explicit_paths_and_resumes_installed_imagery() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();
        let paths = ImageryPaths {
            city: &root.join("map.data"),
            streets: &root.join("roads.data"),
            landscape: &root.join("vegetation.data"),
            manifest: &root.join("metadata.json"),
            downloads_directory: &root.join("raw-images"),
            blocks_directory: &root.join("block-cache"),
            service: &root.join("cache/provider.json"),
            plan: &root.join("cache/job.json"),
            previous_trees: &root.join("cache/original-trees.data"),
            report: &root.join("cache/result.json"),
            trees: &root.join("cache/crowns.data"),
            installed_trees: &root.join("outputs/trees.data"),
            installed_trees_report: &root.join("outputs/trees-report.json"),
            aerial_backup: &root.join("backup/old-imagery"),
            staged_aerial_directory: &root.join("scratch/textures"),
            installed_aerial: AerialPaths {
                directory: &root.join("outputs/textures"),
                service: &root.join("outputs/provenance/provider.json"),
                plan: &root.join("outputs/provenance/job.json"),
                catalog: &root.join("outputs/catalog.data"),
                source: &root.join("outputs/provenance/source.json"),
            },
        };
        let city =
            "CITY 2\norigin 35.162 33.32\nbounds -100 -100 100 100\nn -20 0\nn 20 0\nr 0 1 8 0 1\n";
        let streets = format!("STREETS 1 1 {:08x}\n", crate::street_surface::crc32(city));
        fs::write(paths.city, city).unwrap();
        fs::write(paths.streets, &streets).unwrap();
        fs::write(
            paths.landscape,
            format!(
                "LANDSCAPE 1 {:08x} {:08x}\n",
                crate::street_surface::crc32(city),
                crate::street_surface::crc32(&streets)
            ),
        )
        .unwrap();
        let bbox = [35.162, 33.32, 35.163, 33.321];
        fs::write(paths.manifest, b"{}").unwrap();
        fs::create_dir_all(paths.downloads_directory).unwrap();
        fs::create_dir_all(paths.service.parent().unwrap()).unwrap();
        fs::write(paths.service, "{}").unwrap();
        let side = CORE + 2 * PAD;
        let image = image::RgbImage::from_pixel(side, side, image::Rgb([50, 70, 60]));
        for block in blocks(bbox) {
            image
                .save(image_path(paths.downloads_directory, &block))
                .unwrap();
        }
        fs::create_dir_all(paths.installed_aerial.directory).unwrap();
        fs::write(paths.installed_aerial.directory.join("old-marker"), "old").unwrap();

        prepare_imagery(&paths, bbox).unwrap();
        assert!(paths.aerial_backup.join("old-marker").is_file());
        assert!(!paths.staged_aerial_directory.exists());
        for path in [
            paths.installed_aerial.service,
            paths.installed_aerial.plan,
            paths.installed_aerial.catalog,
            paths.installed_aerial.source,
            paths.report,
            paths.trees,
            paths.installed_trees,
            paths.installed_trees_report,
        ] {
            assert!(path.is_file(), "{}", path.display());
        }
        let catalog = fs::read(paths.installed_aerial.catalog).unwrap();
        prepare_imagery(&paths, bbox).unwrap();
        assert_eq!(fs::read(paths.installed_aerial.catalog).unwrap(), catalog);
        assert_eq!(
            fs::read(paths.trees).unwrap(),
            fs::read(paths.installed_trees).unwrap()
        );
    }

    #[test]
    fn grid_is_contiguous_and_gutters_match() {
        let b = blocks([35.1, 33.25, 35.23, 33.47]);
        assert!(b.len() > 200 && b.len() < 500);
        let a = &b[0];
        let next = &b[1];
        assert!((next.extent[0] - a.extent[0] - CORE as f64 * RES).abs() < 1e-6);
        assert!((a.extent[2] - next.extent[0] - 2. * PAD as f64 * RES).abs() < 1e-6);
        let last = b.last().unwrap();
        let sw = mercator(33.25, 35.1);
        let ne = mercator(33.47, 35.23);
        assert!(a.extent[0] < sw[0] && a.extent[3] > ne[1]);
        assert!(last.extent[2] > ne[0] && last.extent[1] < sw[1]);
    }
    #[test]
    fn block_resume_validates_jpeg_checksums_and_gutters() {
        let temp = tempfile::tempdir().unwrap();
        let work = temp.path();
        fs::create_dir(work.join("downloads")).unwrap();
        let aerial = work.join("aerial");
        fs::create_dir(&aerial).unwrap();
        let b = blocks([35.16, 33.31, 35.161, 33.311]).remove(0);
        let side = CORE + 2 * PAD;
        let im = image::RgbImage::from_fn(side, side, |x, y| {
            image::Rgb([(40 + x % 128) as u8, (40 + y % 128) as u8, 80])
        });
        im.save(image_path(&work.join("downloads"), &b)).unwrap();
        let first = process(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap();
        assert_eq!(first["tiles"].as_array().unwrap().len(), 16);
        let a = fs::read(aerial.join("0_0_0_0.jpg")).unwrap();
        let decoded = image::load_from_memory(&a).unwrap().to_rgb8().into_raw();
        let right = image::open(aerial.join("0_0_1_0.jpg"))
            .unwrap()
            .to_rgb8()
            .into_raw();
        let width = (TILE + 2 * GUTTER) as usize;
        for y in 0..width {
            let left = &decoded[(y * width + TILE as usize) * 3..(y + 1) * width * 3];
            let right = &right[y * width * 3..(y * width + 2 * GUTTER as usize) * 3];
            assert!(left.iter().zip(right).all(|(a, b)| a.abs_diff(*b) <= 12));
        }
        assert_eq!(
            first,
            process(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
        );
        fs::write(aerial.join("0_0_0_0.jpg"), b"damaged").unwrap();
        assert_eq!(
            first,
            process(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
        );
        assert_eq!(a, fs::read(aerial.join("0_0_0_0.jpg")).unwrap());
    }
    #[test]
    fn white_coverage_is_not_published_as_imagery() {
        let mut im = image::RgbImage::from_pixel(TILE, TILE, image::Rgb([255, 255, 255]));
        assert!(blank(&im, 0, 0));
        for y in 0..TILE {
            for x in 0..TILE {
                im.put_pixel(x, y, image::Rgb([50, 70, 60]));
            }
        }
        assert!(!blank(&im, 0, 0));
    }
}
