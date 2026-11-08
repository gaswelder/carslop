//! Shared location data, geometry helpers, and compiler errors.
use geo::{Coord, LineString, MultiPolygon, Polygon};
use sha2::{Digest, Sha256};
use std::{io, path::Path};

use crate::world::P;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

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
