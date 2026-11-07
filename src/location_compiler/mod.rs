//! Offline location compiler. No subprocesses or Python runtime are used.
#[cfg(test)]
mod acceptance;
mod add_limassol;
mod bake_collision;
mod bake_scene;
pub(crate) mod boundary;
mod download;
mod imagery;
mod osm;
mod scenery;
mod sources;
mod surfaces;
mod terrain;
mod track;
pub(crate) use add_limassol::add_limassol;
pub(crate) use bake_collision::bake_collision;
pub(crate) use bake_scene::bake_scene;
pub(crate) use sources::extract_osmpbf;
pub use track::create_track;
#[cfg(test)]
pub(crate) mod tests;

use crate::imagery_trees::atomic;
use geo::{Coord, Intersects, LineString, MapCoords, MultiPolygon, Polygon};
use imagery::{
    blocks, city_origin, download, local, mercator, process, CORE, GUTTER, PAD, RES, SERVICE, TILE,
};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, fs, io, path::Path};
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
pub type P = [f64; 2];

/// Geographic bounds in degrees, with named latitude and longitude limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct BoundingBox {
    pub south: f64,
    pub west: f64,
    pub north: f64,
    pub east: f64,
}
impl BoundingBox {
    pub fn is_valid(&self) -> bool {
        self.to_array().iter().all(|v| v.is_finite())
            && self.south < self.north
            && self.west < self.east
            && self.south >= -90.0
            && self.north <= 90.0
            && self.west >= -180.0
            && self.east <= 180.0
    }

    /// Ordering used by the existing OSM extraction and compiler APIs.
    pub fn to_array(self) -> [f64; 4] {
        [self.south, self.west, self.north, self.east]
    }
}

pub fn invalid(message: impl Into<String>) -> Box<dyn std::error::Error> {
    io::Error::new(io::ErrorKind::InvalidInput, message.into()).into()
}
pub fn sha(path: &Path) -> io::Result<String> {
    use std::io::Read;
    let mut file = std::fs::File::open(path)?;
    let mut hash = Sha256::new();
    let mut buffer = [0; 65536];
    loop {
        let n = file.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        hash.update(&buffer[..n]);
    }
    Ok(hash
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect())
}
pub fn add(a: P, b: P) -> P {
    [a[0] + b[0], a[1] + b[1]]
}
pub fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
pub fn mul(a: P, s: f64) -> P {
    [a[0] * s, a[1] * s]
}
pub fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
pub fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
pub fn length(a: P) -> f64 {
    a[0].hypot(a[1])
}
pub fn dist(a: P, b: P) -> f64 {
    length(sub(a, b))
}
pub fn unit(a: P) -> P {
    mul(a, 1.0 / length(a).max(1e-12))
}
pub fn line(points: &[P]) -> LineString<f64> {
    LineString::new(points.iter().map(|p| Coord { x: p[0], y: p[1] }).collect())
}
pub fn polygon(points: &[P]) -> Polygon<f64> {
    Polygon::new(line(points), vec![])
}
pub fn empty() -> MultiPolygon<f64> {
    MultiPolygon(vec![])
}
pub fn path_length(path: &[P]) -> f64 {
    path.windows(2).map(|p| dist(p[0], p[1])).sum()
}
pub fn along(path: &[P], mut station: f64) -> P {
    for pair in path.windows(2) {
        let n = dist(pair[0], pair[1]);
        if station <= n {
            return add(pair[0], mul(sub(pair[1], pair[0]), station.max(0.0) / n));
        }
        station -= n;
    }
    *path.last().unwrap()
}
pub fn project(path: &[P], p: P) -> (f64, f64) {
    let mut best = (f64::INFINITY, 0.0);
    let mut station = 0.0;
    for pair in path.windows(2) {
        let d = sub(pair[1], pair[0]);
        let n = length(d);
        let t = (dot(sub(p, pair[0]), d) / (n * n)).clamp(0.0, 1.0);
        let distance = dist(p, add(pair[0], mul(d, t)));
        if distance < best.0 {
            best = (distance, station + t * n);
        }
        station += n;
    }
    best
}
pub fn substring(path: &[P], lo: f64, hi: f64) -> Vec<P> {
    let mut out = vec![along(path, lo)];
    let mut station = 0.0;
    for pair in path.windows(2) {
        station += dist(pair[0], pair[1]);
        if station > lo && station < hi {
            out.push(pair[1]);
        }
    }
    out.push(along(path, hi));
    out
}
#[derive(Clone, Default)]
pub struct Element {
    pub id: i64,
    pub tags: BTreeMap<String, String>,
    pub coord: Option<P>,
    pub nodes: Vec<i64>,
    pub members: Vec<(String, i64, String)>,
}
impl Element {
    pub fn tag(&self, key: &str) -> &str {
        self.tags.get(key).map(String::as_str).unwrap_or("")
    }
}
pub struct Source {
    pub nodes: BTreeMap<i64, Element>,
    pub ways: BTreeMap<i64, Element>,
    pub relations: Vec<Element>,
    pub bounds: Option<[f64; 4]>,
    pub timestamp: String,
}
#[derive(Clone)]
pub struct Road {
    pub ends: [usize; 2],
    pub width: f64,
    pub direction: i8,
    pub way: i64,
    pub kind: usize,
    pub lanes: usize,
}
pub struct Building {
    pub id: i64,
    pub height: f64,
    pub tagged: bool,
    pub points: Vec<P>,
}
pub struct Map {
    pub boundary: Option<MultiPolygon<f64>>,
    pub origin: P,
    pub bounds: [f64; 4],
    pub nodes: Vec<P>,
    pub roads: Vec<Road>,
    pub buildings: Vec<Building>,
}
impl Map {
    pub fn metres(&self, p: P) -> P {
        [
            (p[1] - self.origin[1]).to_radians() * 6371000.0 * self.origin[0].to_radians().cos(),
            -(p[0] - self.origin[0]).to_radians() * 6371000.0,
        ]
    }
    pub fn contains(&self, p: P) -> bool {
        p[0] >= self.bounds[0]
            && p[0] <= self.bounds[2]
            && p[1] >= self.bounds[1]
            && p[1] <= self.bounds[3]
            && self.boundary.as_ref().is_none_or(|b| {
                use geo::Intersects;
                b.intersects(&geo::Point::new(p[0], p[1]))
            })
    }
}
struct AerialPaths<'a> {
    directory: &'a Path,
    service: &'a Path,
    plan: &'a Path,
    catalog: &'a Path,
    source: &'a Path,
}

struct ImageryPaths<'a> {
    city: &'a Path,
    streets: &'a Path,
    landscape: &'a Path,
    manifest: &'a Path,
    downloads_directory: &'a Path,
    blocks_directory: &'a Path,
    service: &'a Path,
    plan: &'a Path,
    previous_trees: &'a Path,
    report: &'a Path,
    trees: &'a Path,
    installed_trees: &'a Path,
    installed_trees_report: &'a Path,
    aerial_backup: &'a Path,
    staged_aerial_directory: &'a Path,
    installed_aerial: AerialPaths<'a>,
}

fn prepare_imagery(paths: &ImageryPaths<'_>, bbox: [f64; 4]) -> Result<()> {
    for path in [
        paths.service,
        paths.plan,
        paths.previous_trees,
        paths.report,
        paths.trees,
        paths.installed_trees,
        paths.installed_trees_report,
        paths.aerial_backup,
        paths.installed_aerial.directory,
    ] {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
    }
    let city_text = std::fs::read_to_string(paths.city)?;
    let city = crate::city::City::parse(&city_text).map_err(invalid)?;
    let origin = city_origin(&city_text)?;
    let manifest: Value = serde_json::from_slice(&std::fs::read(paths.manifest)?)?;
    if bbox.iter().any(|v| !v.is_finite())
        || bbox[0] >= bbox[2]
        || bbox[1] >= bbox[3]
        || bbox[0] < -85.
        || bbox[2] > 85.
        || bbox[1] < -180.
        || bbox[3] > 180.
    {
        return Err(invalid("invalid city geographic bounds"));
    }
    let boundary = boundary::Boundary::from_manifest(&manifest)?;
    let local_boundary = boundary.as_ref().map(|b| b.local(origin));
    let mercator_boundary = boundary.as_ref().map(|b| {
        b.geometry.map_coords(|c| {
            let p = mercator(c.x, c.y);
            geo::Coord { x: p[0], y: p[1] }
        })
    });
    let all: Vec<_> = blocks(bbox)
        .into_iter()
        .filter(|b| {
            mercator_boundary.as_ref().is_none_or(|g| {
                g.intersects(&geo::Rect::new(
                    geo::Coord {
                        x: b.extent[0],
                        y: b.extent[1],
                    },
                    geo::Coord {
                        x: b.extent[2],
                        y: b.extent[3],
                    },
                ))
            })
        })
        .collect();
    if all.is_empty() || all.len() > 2000 {
        return Err(invalid(
            "imagery job must contain 1..2000 blocks; crop the map",
        ));
    }
    let mut plan = json!({"version":1,"source":SERVICE,"bbox":bbox,"origin":origin,"resolution_mercator":RES,"core":CORE,"padding":PAD,"tile_size":TILE,"gutter":GUTTER,"blocks":all.len(),"city_sha256":sha(paths.city)?});
    if boundary.is_some() {
        plan["boundary"] = manifest["boundary"].clone();
    }
    let plan_path = paths.plan;
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
    download(paths.downloads_directory, paths.service, &all)?;
    let plan_hash = sha(&plan_path)?;
    let installed = paths.installed_aerial.source;
    let installed_plan = fs::read(&installed)
        .ok()
        .and_then(|s| serde_json::from_slice::<Value>(&s).ok())
        .is_some_and(|v| v["plan_sha256"] == plan_hash);
    let aerial = if installed_plan {
        paths.installed_aerial.directory
    } else {
        paths.staged_aerial_directory
    };
    fs::create_dir_all(aerial)?;
    let pilot = paths.previous_trees;
    if !pilot.exists() {
        if let Ok(data) = fs::read(paths.installed_trees) {
            atomic(&pilot, &data)?;
        }
    }
    let mut catalog = format!("AERIAL 1\nsize {TILE} {GUTTER}\n");
    let mut drafts = vec![];
    let (mut tile_count, mut missing) = (0, 0);
    for (i, b) in all.iter().enumerate() {
        let meta = process(
            paths.downloads_directory,
            paths.blocks_directory,
            aerial,
            b,
            origin,
        )?;
        for t in meta["tiles"]
            .as_array()
            .ok_or_else(|| invalid("missing block tiles"))?
        {
            let bounds = t["bounds"]
                .as_array()
                .ok_or_else(|| invalid("missing tile bounds"))?;
            if local_boundary.as_ref().is_some_and(|g| {
                !g.intersects(&geo::Rect::new(
                    geo::Coord {
                        x: bounds[0].as_f64().unwrap(),
                        y: bounds[1].as_f64().unwrap(),
                    },
                    geo::Coord {
                        x: bounds[2].as_f64().unwrap(),
                        y: bounds[3].as_f64().unwrap(),
                    },
                ))
            }) {
                continue;
            }
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
    let obstacles = crate::tree_obstacles::Obstacles::read_paths(
        paths.city,
        paths.streets,
        [bbox[1], bbox[0], bbox[3], bbox[2]],
    )?;
    let landscape = fs::read_to_string(paths.landscape)?;
    let header = landscape
        .lines()
        .next()
        .ok_or_else(|| invalid("missing landscape header"))?;
    let expected = format!(
        "LANDSCAPE 1 {:08x} {:08x}",
        crate::street_surface::crc32(&city_text),
        crate::imagery_trees::checksum(paths.streets)?
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
        if !inside(p)
            || local_boundary
                .as_ref()
                .is_some_and(|b| !b.intersects(&geo::Point::new(p[0], p[1])))
        {
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
    if plan["city_sha256"] != sha(paths.city)? {
        return Err(invalid(
            "city changed during imagery preparation; choose a fresh work directory",
        ));
    }
    let report = json!({"plan_sha256":plan_hash,"source":SERVICE,"attribution":"Source: Esri, Vantor, Earthstar Geographics, and the GIS User Community","local_development_only":true,"bbox":bbox,"blocks":all.len(),"tile_count":tile_count,"missing_tiles":missing,"imagery_trees":count,"excluded_road_or_building":conflicts,"excluded_duplicates":duplicates,"excluded_outside_city":outside,"ground_sampling_metres":RES*origin[0].to_radians().cos(),"sizes":"Tree sizes are artistic estimates; candidates inferred by RGB vegetation/texture heuristics.","rights":"Local development pack; redistribution rights have not been established."});
    atomic(paths.report, &serde_json::to_vec_pretty(&report)?)?;
    atomic(paths.trees, output.as_bytes())?;
    if !installed_plan {
        let backup = paths.aerial_backup;
        let destination = paths.installed_aerial.directory;
        if destination.exists() {
            if backup.exists() {
                return Err(invalid(
                    "previous-aerial backup already exists; inspect it before installing",
                ));
            }
            fs::rename(&destination, &backup)?;
        }
        if let Err(e) = fs::rename(aerial, destination) {
            if backup.exists() {
                let _ = fs::rename(&backup, &destination);
            }
            return Err(e.into());
        }
    }
    // Provenance belongs to the installed pack, not the disposable work cache.
    let aerial = &paths.installed_aerial;
    for path in [aerial.service, aerial.plan, aerial.catalog, aerial.source] {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
    }
    atomic(aerial.service, &fs::read(paths.service)?)?;
    atomic(aerial.plan, &fs::read(paths.plan)?)?;
    atomic(aerial.catalog, catalog.as_bytes())?;
    atomic(aerial.source, &serde_json::to_vec_pretty(&report)?)?;
    atomic(
        paths.installed_trees_report,
        &serde_json::to_vec_pretty(&report)?,
    )?;
    atomic(paths.installed_trees, output.as_bytes())?;
    println!("Installed {tile_count} imagery tiles and {count} trees; {missing} missing imagery tiles. Run bake-city --location NAME to rebuild the playable scene.");
    Ok(())
}
