//! Resumable city-wide aerial preparation. Downloads are explicit; gameplay is offline.
use crate::{
    imagery_trees::atomic,
    location::{invalid, Result},
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
const SERVICE: &str =
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer";
const RES: f64 = 0.5971642835598172; // zoom-18 Web Mercator sampling, ~0.49 m on the ground here
const CORE: u32 = 2048;
const PAD: u32 = 32;
const TILE: u32 = 512;
const GUTTER: u32 = 8;
#[derive(Clone, Debug)]
struct Block {
    x: usize,
    y: usize,
    extent: [f64; 4],
}
fn mercator(lon: f64, lat: f64) -> [f64; 2] {
    [
        lon.to_radians() * 6378137.,
        (std::f64::consts::FRAC_PI_4 + lat.to_radians() / 2.)
            .tan()
            .ln()
            * 6378137.,
    ]
}
fn blocks(bbox: [f64; 4]) -> Vec<Block> {
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
    work.join("downloads").join(format!("{}_{}.jpg", b.x, b.y))
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
fn download(work: &Path, all: &[Block]) -> Result<()> {
    fs::create_dir_all(work.join("downloads"))?;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .into();
    let service_path = work.join("service.json");
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
fn city_origin(text: &str) -> Result<[f64; 2]> {
    let line = text
        .lines()
        .find(|l| l.starts_with("origin "))
        .ok_or_else(|| invalid("missing city origin"))?;
    let v: Vec<_> = line.split_whitespace().collect();
    Ok([v[1].parse()?, v[2].parse()?])
}
fn local(origin: [f64; 2], ll: [f64; 2]) -> [f64; 2] {
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
fn process(work: &Path, aerial: &Path, b: &Block, origin: [f64; 2]) -> Result<Value> {
    let meta_path = work.join(format!("block-{}_{}.json", b.x, b.y));
    let image_file = image_path(work, b);
    let hash = crate::location::sha(&image_file)?;
    if meta_path.exists() {
        let value: Value = serde_json::from_slice(&fs::read(&meta_path)?)?;
        if value["image_sha256"] == hash
            && value["tiles"].as_array().is_some_and(|tiles| {
                tiles.iter().all(|t| {
                    let Some(name) = t["name"].as_str() else {
                        return false;
                    };
                    let Ok(bytes) = fs::read(aerial.join(name)) else {
                        return false;
                    };
                    bytes.len() == ((TILE + 2 * GUTTER).pow(2) * 3) as usize
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
            let name = format!("{}_{}_{}_{}.rgb", b.x, b.y, x, z);
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
    let value = json!({"image_sha256":hash,"extent":b.extent,"tiles":tiles,"missing_tiles":missing,"trees":trees});
    atomic(&meta_path, &serde_json::to_vec(&value)?)?;
    Ok(value)
}
pub fn run(city_dir: &Path, work: &Path, limit: Option<usize>) -> Result<()> {
    if limit == Some(0) {
        return Err(invalid("--limit-blocks must be positive"));
    }
    fs::create_dir_all(work)?;
    let city_text = std::fs::read_to_string(city_dir.join("city.txt"))?;
    let city = crate::city::City::parse(&city_text).map_err(invalid)?;
    let origin = city_origin(&city_text)?;
    let manifest: Value = serde_json::from_slice(&std::fs::read(&city_dir.join("manifest.json"))?)?;
    let mut bbox = [0.; 4];
    for (i, v) in bbox.iter_mut().enumerate() {
        *v = manifest["bbox_south_west_north_east"][i]
            .as_f64()
            .filter(|v| v.is_finite())
            .ok_or_else(|| invalid("missing city geographic bounds"))?;
    }
    if bbox[0] >= bbox[2]
        || bbox[1] >= bbox[3]
        || bbox[0] < -85.
        || bbox[2] > 85.
        || bbox[1] < -180.
        || bbox[3] > 180.
    {
        return Err(invalid("invalid city geographic bounds"));
    }
    let all = blocks(bbox);
    if all.is_empty() || all.len() > 2000 {
        return Err(invalid(
            "imagery job must contain 1..2000 blocks; crop the map",
        ));
    }
    let plan = json!({"version":1,"source":SERVICE,"bbox":bbox,"origin":origin,"resolution_mercator":RES,"core":CORE,"padding":PAD,"tile_size":TILE,"gutter":GUTTER,"blocks":all.len(),"city_sha256":crate::location::sha(&city_dir.join("city.txt"))?});
    let plan_path = work.join("plan.json");
    if plan_path.exists() {
        let previous: Value = serde_json::from_slice(&fs::read(&plan_path)?)?;
        if previous != plan {
            return Err(invalid(
                "work directory contains a different imagery plan; choose a new directory",
            ));
        }
    } else {
        atomic(&plan_path, &serde_json::to_vec_pretty(&plan)?)?;
    }
    println!(
        "City imagery: {} blocks, up to {} tiles; approximately {:.2} m/pixel on the ground.",
        all.len(),
        all.len() * 16,
        RES * origin[0].to_radians().cos()
    );
    if let Some(n) = limit {
        download(work, &all[..n.min(all.len())])?;
        println!("Download sample complete; city assets unchanged. Run without --limit-blocks for full preparation.");
        return Ok(());
    }
    download(work, &all)?;
    let plan_hash = crate::location::sha(&plan_path)?;
    let installed = city_dir.join("aerial/source.json");
    let installed_plan = fs::read(&installed)
        .ok()
        .and_then(|s| serde_json::from_slice::<Value>(&s).ok())
        .is_some_and(|v| v["plan_sha256"] == plan_hash);
    let aerial = if installed_plan {
        city_dir.join("aerial")
    } else {
        work.join("aerial")
    };
    fs::create_dir_all(&aerial)?;
    let pilot = work.join("previous-imagery-trees.txt");
    if !pilot.exists() {
        if let Ok(data) = fs::read(city_dir.join("imagery-trees.txt")) {
            atomic(&pilot, &data)?;
        }
    }
    let mut catalog = format!("AERIAL 1\nsize {TILE} {GUTTER}\n");
    let mut drafts = vec![];
    let (mut tile_count, mut missing) = (0, 0);
    for (i, b) in all.iter().enumerate() {
        let meta = process(work, &aerial, b, origin)?;
        for t in meta["tiles"]
            .as_array()
            .ok_or_else(|| invalid("missing block tiles"))?
        {
            let bounds = t["bounds"]
                .as_array()
                .ok_or_else(|| invalid("missing tile bounds"))?;
            catalog.push_str(&format!(
                "tile {:.6} {:.6} {:.6} {:.6} {} {:08x}\n",
                bounds[0].as_f64().unwrap(),
                bounds[1].as_f64().unwrap(),
                bounds[2].as_f64().unwrap(),
                bounds[3].as_f64().unwrap(),
                t["name"].as_str().unwrap(),
                t["crc"].as_u64().unwrap()
            ));
            tile_count += 1;
        }
        for t in meta["trees"]
            .as_array()
            .ok_or_else(|| invalid("missing block crowns"))?
        {
            drafts.push([
                t[0].as_f64().unwrap(),
                t[1].as_f64().unwrap(),
                t[2].as_f64().unwrap(),
            ]);
        }
        missing += meta["missing_tiles"].as_u64().unwrap_or(0);
        if (i + 1) % 10 == 0 || i + 1 == all.len() {
            println!(
                "Prepared blocks: {}/{}; {tile_count} tiles, {} crown proposals",
                i + 1,
                all.len(),
                drafts.len()
            );
        }
    }
    if tile_count == 0 {
        return Err(invalid(
            "source returned no usable imagery; city assets unchanged",
        ));
    }
    println!(
        "Filtering {} crown proposals against roads, buildings and mapped trees...",
        drafts.len()
    );
    let obstacles =
        crate::tree_obstacles::Obstacles::read(city_dir, [bbox[1], bbox[0], bbox[3], bbox[2]])?;
    let landscape = fs::read_to_string(city_dir.join("landscape.txt"))?;
    let header = landscape
        .lines()
        .next()
        .ok_or_else(|| invalid("missing landscape header"))?;
    let expected = format!(
        "LANDSCAPE 1 {:08x} {:08x}",
        crate::street_surface::crc32(&city_text),
        crate::imagery_trees::checksum(&city_dir.join("streets.txt"))?
    );
    if header != expected {
        return Err(invalid("landscape does not match city and roads"));
    }
    let mut index = rstar::RTree::new();
    for line in landscape.lines() {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.first() == Some(&"t") && f.len() == 6 {
            index.insert([f[2].parse::<f64>()?, f[3].parse::<f64>()?]);
        }
    }
    let mut output = format!("{header}\n");
    let (mut count, mut conflicts, mut duplicates, mut outside) = (0, 0, 0, 0);
    let inside = |p: [f64; 2]| {
        p[0] >= city.bounds[0][0] as f64
            && p[1] >= city.bounds[0][1] as f64
            && p[0] <= city.bounds[1][0] as f64
            && p[1] <= city.bounds[1][1] as f64
    };
    // Preserve the already approved pilot; new imagery proposals are deduplicated around it.
    let mut append = |p: [f64; 2], radius: f64, height: f64| {
        let p = p.map(|v| (v * 1000.).round() / 1000.);
        if !inside(p) {
            outside += 1;
            return;
        }
        if obstacles.rejection_at(p).is_some() {
            conflicts += 1;
            return;
        }
        if index
            .nearest_neighbor(&p)
            .is_some_and(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < 2.)
        {
            duplicates += 1;
            return;
        }
        let id = (1_u64 << 63) | count;
        output.push_str(&format!(
            "t {id} {:.3} {:.3} {:.3} {:.3}\n",
            p[0],
            p[1],
            height.clamp(3.5, 14.),
            radius.clamp(0.7, 4.5)
        ));
        index.insert(p);
        count += 1;
    };
    if pilot.exists() {
        for line in fs::read_to_string(&pilot)?.lines().skip(1) {
            let f: Vec<_> = line.split_whitespace().collect();
            if f.len() == 6 && f[0] == "t" {
                append([f[2].parse()?, f[3].parse()?], f[5].parse()?, f[4].parse()?);
            }
        }
    }
    for d in drafts {
        let radius = d[2].clamp(0.7, 4.5);
        append(local(origin, [d[0], d[1]]), radius, radius * 2.8);
    }
    if plan["city_sha256"] != crate::location::sha(&city_dir.join("city.txt"))? {
        return Err(invalid(
            "city changed during imagery preparation; choose a fresh work directory",
        ));
    }
    let report = json!({"plan_sha256":plan_hash,"source":SERVICE,"attribution":"Source: Esri, Vantor, Earthstar Geographics, and the GIS User Community","local_development_only":true,"bbox":bbox,"blocks":all.len(),"tile_count":tile_count,"missing_tiles":missing,"imagery_trees":count,"excluded_road_or_building":conflicts,"excluded_duplicates":duplicates,"excluded_outside_city":outside,"ground_sampling_metres":RES*origin[0].to_radians().cos(),"sizes":"Tree sizes are artistic estimates; candidates inferred by RGB vegetation/texture heuristics.","rights":"Local development pack; redistribution rights have not been established."});
    // Provenance belongs to the installed pack, not the disposable work cache.
    for name in ["service.json", "plan.json"] {
        atomic(&aerial.join(name), &fs::read(work.join(name))?)?;
    }
    atomic(&aerial.join("tiles.txt"), catalog.as_bytes())?;
    atomic(
        &aerial.join("source.json"),
        &serde_json::to_vec_pretty(&report)?,
    )?;
    atomic(
        &work.join("report.json"),
        &serde_json::to_vec_pretty(&report)?,
    )?;
    atomic(&work.join("imagery-trees.txt"), output.as_bytes())?;
    if !installed_plan {
        let backup = work.join("previous-aerial");
        let destination = city_dir.join("aerial");
        if destination.exists() {
            if backup.exists() {
                return Err(invalid(
                    "previous-aerial backup already exists; inspect it before installing",
                ));
            }
            fs::rename(&destination, &backup)?;
        }
        if let Err(e) = fs::rename(&aerial, &destination) {
            if backup.exists() {
                let _ = fs::rename(&backup, &destination);
            }
            return Err(e.into());
        }
    }
    atomic(
        &city_dir.join("imagery-trees.json"),
        &serde_json::to_vec_pretty(&report)?,
    )?;
    atomic(&city_dir.join("imagery-trees.txt"), output.as_bytes())?;
    println!("Installed {tile_count} imagery tiles and {count} trees; {missing} missing imagery tiles. Run bake-city --location NAME to rebuild the playable scene.");
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
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
    fn block_resume_validates_checksums_and_internal_gutters_share_pixels() {
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
        im.save(image_path(work, &b)).unwrap();
        let first = process(work, &aerial, &b, [35.162, 33.32]).unwrap();
        assert_eq!(first["tiles"].as_array().unwrap().len(), 16);
        let a = fs::read(aerial.join("0_0_0_0.rgb")).unwrap();
        let right = fs::read(aerial.join("0_0_1_0.rgb")).unwrap();
        let width = (TILE + 2 * GUTTER) as usize;
        for y in 0..width {
            assert_eq!(
                &a[(y * width + TILE as usize) * 3..(y + 1) * width * 3],
                &right[y * width * 3..(y * width + 2 * GUTTER as usize) * 3]
            );
        }
        assert_eq!(first, process(work, &aerial, &b, [35.162, 33.32]).unwrap());
        fs::write(aerial.join("0_0_0_0.rgb"), b"damaged").unwrap();
        assert_eq!(first, process(work, &aerial, &b, [35.162, 33.32]).unwrap());
        assert_eq!(a, fs::read(aerial.join("0_0_0_0.rgb")).unwrap());
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
