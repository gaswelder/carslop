//! Offline location compiler. No subprocesses or Python runtime are used.
#[cfg(test)]
mod acceptance;
mod osm;
mod scenery;
mod surfaces;
mod terrain;
#[cfg(test)]
pub(crate) mod tests;

use geo::{Coord, LineString, MultiPolygon, Polygon};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, fs, io, path::Path};
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
pub type P = [f64; 2];
pub fn invalid(message: impl Into<String>) -> Box<dyn std::error::Error> {
    io::Error::new(io::ErrorKind::InvalidInput, message.into()).into()
}
pub fn sha(path: &Path) -> Result<String> {
    use std::io::Read;
    let mut file = crate::package::Reader::open(path)?;
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
pub fn json(path: &Path, value: serde_json::Value) -> Result<()> {
    fs::write(path, serde_json::to_string_pretty(&value)? + "\n")?;
    Ok(())
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
    }
}
pub struct Options<'a> {
    pub source: &'a Path,
    pub output: &'a Path,
    pub name: &'a str,
    pub bbox: Option<[f64; 4]>,
    pub elevation: &'a Path,
}
pub fn compile(options: Options<'_>) -> Result<()> {
    println!("Reading OSM XML...");
    let source = osm::read(options.source)?;
    let map = osm::compile(&source, options.bbox)?;
    fs::create_dir_all(options.output.join("elevation"))?;
    let city = osm::write(&map, options.output)?;
    println!("Building streets ({} segments)...", map.roads.len());
    let surfaces = surfaces::build(&map, &city, options.output)?;
    println!("Building mapped landscape and roadside controls...");
    scenery::build(&source, &map, &surfaces, &city, options.output)?;
    println!("Building elevation...");
    terrain::build(&map, &city, options.elevation, options.output)?;
    fs::copy(options.source, options.output.join("source.osm"))?;
    let south_west = [
        map.origin[0] - map.bounds[3].to_degrees() / 6371000.0,
        map.origin[1] + map.bounds[0].to_degrees() / 6371000.0 / map.origin[0].to_radians().cos(),
    ];
    let north_east = [
        map.origin[0] - map.bounds[1].to_degrees() / 6371000.0,
        map.origin[1] + map.bounds[2].to_degrees() / 6371000.0 / map.origin[0].to_radians().cos(),
    ];
    let mut manifest = serde_json::json!({
        "name":options.name,"origin_lat_lon":map.origin,"bounds_metres":map.bounds,
        "bbox_south_west_north_east":[south_west[0],south_west[1],north_east[0],north_east[1]],
        "source_timestamp":source.timestamp,"source_file":"source.osm","source_sha256":sha(options.source)?,
        "package_sha256":sha(&options.output.join("city.txt"))?,
        "attribution":"© OpenStreetMap contributors","license":"ODbL-1.0",
        "copyright_url":"https://www.openstreetmap.org/copyright","compiler":"native-rust-v1",
        "statistics":{"nodes":map.nodes.len(),"road_segments":map.roads.len(),"buildings":map.buildings.len()}
    });
    let provenance = options
        .source
        .parent()
        .unwrap_or(Path::new("."))
        .join("sources/manifest.json");
    if provenance.is_file() {
        manifest["source_snapshots"] = serde_json::from_str(&fs::read_to_string(provenance)?)?;
        manifest["source_timestamp"] = "mixed snapshots; see source_snapshots".into();
    }
    json(&options.output.join("manifest.json"), manifest)?;
    Ok(())
}
