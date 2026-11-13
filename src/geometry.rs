use geo::{Coord, LineString, MultiPolygon, Polygon};

/// Local point or vector in metres.
/// x is world X (east), z is world Z (south).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct XZ<T = f64> {
    pub x: T, // Eastward coordinate or displacement in metres.
    pub z: T, // Southward coordinate or displacement in metres.
}

impl<T: Copy> XZ<T> {
    pub fn from_fn(mut f: impl FnMut(usize) -> T) -> Self {
        Self { x: f(0), z: f(1) }
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        [&mut self.x, &mut self.z].into_iter()
    }

    pub fn component(self, axis: usize) -> T {
        match axis {
            0 => self.x,
            1 => self.z,
            _ => panic!("XZ axis out of bounds: {axis}"),
        }
    }
    pub fn component_mut(&mut self, axis: usize) -> &mut T {
        match axis {
            0 => &mut self.x,
            1 => &mut self.z,
            _ => panic!("XZ axis out of bounds: {axis}"),
        }
    }
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> XZ<U> {
        XZ {
            x: f(self.x),
            z: f(self.z),
        }
    }
    pub fn at_height(self, y: T) -> XYZ<T> {
        XYZ {
            x: self.x,
            y,
            z: self.z,
        }
    }
}

impl<T> IntoIterator for XZ<T> {
    type Item = T;
    type IntoIter = std::array::IntoIter<T, 2>;
    fn into_iter(self) -> Self::IntoIter {
        [self.x, self.z].into_iter()
    }
}

/// Local 3D point or vector in metres: X east, Y up, Z south.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct XYZ<T = f32> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Copy> XYZ<T> {
    pub fn from_fn(mut f: impl FnMut(usize) -> T) -> Self {
        Self {
            x: f(0),
            y: f(1),
            z: f(2),
        }
    }
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        [&self.x, &self.y, &self.z].into_iter()
    }
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        [&mut self.x, &mut self.y, &mut self.z].into_iter()
    }

    pub fn component(self, axis: usize) -> T {
        match axis {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => panic!("XYZ axis out of bounds: {axis}"),
        }
    }
    pub fn component_mut(&mut self, axis: usize) -> &mut T {
        match axis {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("XYZ axis out of bounds: {axis}"),
        }
    }
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> XYZ<U> {
        XYZ {
            x: f(self.x),
            y: f(self.y),
            z: f(self.z),
        }
    }
    pub fn xz(self) -> XZ<T> {
        XZ {
            x: self.x,
            z: self.z,
        }
    }
}

impl<T> IntoIterator for XYZ<T> {
    type Item = T;
    type IntoIter = std::array::IntoIter<T, 3>;
    fn into_iter(self) -> Self::IntoIter {
        [self.x, self.y, self.z].into_iter()
    }
}

impl<T> From<[T; 3]> for XYZ<T> {
    fn from([x, y, z]: [T; 3]) -> Self {
        Self { x, y, z }
    }
}

impl<T> From<XYZ<T>> for [T; 3] {
    fn from(point: XYZ<T>) -> Self {
        [point.x, point.y, point.z]
    }
}

/// Axis-aligned local bounds in metres, with inclusive minimum and maximum corners.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoundingBoxXZ<T = f64> {
    /// Minimum eastward X and southward Z coordinates.
    pub min: XZ<T>,
    /// Maximum eastward X and southward Z coordinates.
    pub max: XZ<T>,
}

impl<T> BoundingBoxXZ<T> {
    /// Ordering used by the location manifest and external geometry records.
    pub fn to_array(self) -> [T; 4] {
        [self.min.x, self.min.z, self.max.x, self.max.z]
    }
}

/// Geographic position in latitude and longitude degrees.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LatLon {
    pub lat: f64, // Latitude in degrees, positive northward.
    pub lon: f64, // Longitude in degrees, positive eastward.
}

/// Geographic rectangle with latitude and longitude limits in degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox {
    /// Minimum latitude in degrees.
    pub min_lat: f64,
    /// Minimum longitude in degrees.
    pub min_lon: f64,
    /// Maximum latitude in degrees.
    pub max_lat: f64,
    /// Maximum longitude in degrees.
    pub max_lon: f64,
}

impl BoundingBox {
    pub fn is_valid(&self) -> bool {
        self.to_array().iter().all(|v| v.is_finite())
            && self.min_lat < self.max_lat
            && self.min_lon < self.max_lon
            && self.min_lat >= -90.0
            && self.max_lat <= 90.0
            && self.min_lon >= -180.0
            && self.max_lon <= 180.0
    }

    /// Apply the imagery pipeline's conservative Web Mercator latitude limit.
    pub fn is_valid_for_imagery(&self) -> bool {
        self.is_valid() && self.min_lat >= -85.0 && self.max_lat <= 85.0
    }

    /// Ordering used by the existing OSM extraction and compiler APIs.
    pub fn to_array(self) -> [f64; 4] {
        [self.min_lat, self.min_lon, self.max_lat, self.max_lon]
    }
}

impl LatLon {
    pub const fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }
}

// Spatial indices operate directly on local points, keeping geographic coordinates out.
impl rstar::Point for XZ {
    type Scalar = f64;
    const DIMENSIONS: usize = 2;

    fn generate(mut generator: impl FnMut(usize) -> f64) -> Self {
        Self {
            x: generator(0),
            z: generator(1),
        }
    }

    fn nth(&self, index: usize) -> f64 {
        match index {
            0 => self.x,
            1 => self.z,
            _ => panic!("XZ coordinate index out of bounds: {index}"),
        }
    }

    fn nth_mut(&mut self, index: usize) -> &mut f64 {
        match index {
            0 => &mut self.x,
            1 => &mut self.z,
            _ => panic!("XZ coordinate index out of bounds: {index}"),
        }
    }
}

impl crate::world::Location {
    /// Convert local metres (east +X, south +Z) to geographic coordinates.
    pub fn geographic_position(&self, position: XYZ) -> Option<LatLon> {
        let LatLon { lat, lon } = self.origin?;
        let longitude_scale = lat.to_radians().cos();
        if longitude_scale.abs() < 1e-9 || !position.x.is_finite() || !position.z.is_finite() {
            return None;
        }
        Some(LatLon::new(
            lat - (position.z as f64 / 6371000.0).to_degrees(),
            lon + (position.x as f64 / (6371000.0 * longitude_scale)).to_degrees(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geographic_positions_round_trip_through_local_metres() {
        let origin = LatLon::new(35.1654321, 33.3256789);
        let mut compiled = crate::source_location::SourceLocation::<f64>::empty();
        compiled.origin = Some(origin);
        let mut runtime = crate::test_support::empty_world();
        runtime.origin = Some(origin);
        for target in [origin, LatLon::new(35.17, 33.34), LatLon::new(35.15, 33.31)] {
            let local = compiled.metres(target);
            let result = runtime
                .geographic_position(XYZ {
                    x: local.x as f32,
                    y: 123.0,
                    z: local.z as f32,
                })
                .unwrap();
            assert!((result.lat - target.lat).abs() < 1e-8);
            assert!((result.lon - target.lon).abs() < 1e-8);
        }
    }
}

pub fn add(a: XZ, b: XZ) -> XZ {
    XZ {
        x: a.x + b.x,
        z: a.z + b.z,
    }
}
pub fn sub(a: XZ, b: XZ) -> XZ {
    XZ {
        x: a.x - b.x,
        z: a.z - b.z,
    }
}
pub fn mul(a: XZ, s: f64) -> XZ {
    XZ {
        x: a.x * s,
        z: a.z * s,
    }
}
pub fn dot(a: XZ, b: XZ) -> f64 {
    a.x * b.x + a.z * b.z
}
pub fn cross(a: XZ, b: XZ) -> f64 {
    a.x * b.z - a.z * b.x
}
pub fn length(a: XZ) -> f64 {
    a.x.hypot(a.z)
}
pub fn dist(a: XZ, b: XZ) -> f64 {
    length(sub(a, b))
}
pub fn unit(a: XZ) -> XZ {
    mul(a, 1.0 / length(a).max(1e-12))
}
pub fn line(points: &[XZ]) -> LineString<f64> {
    LineString::new(points.iter().map(|p| Coord { x: p.x, y: p.z }).collect())
}
pub fn polygon(points: &[XZ]) -> Polygon<f64> {
    Polygon::new(line(points), vec![])
}
pub fn empty() -> MultiPolygon<f64> {
    MultiPolygon(vec![])
}
pub fn path_length(path: &[XZ]) -> f64 {
    path.windows(2).map(|p| dist(p[0], p[1])).sum()
}
pub fn along(path: &[XZ], mut station: f64) -> XZ {
    for pair in path.windows(2) {
        let n = dist(pair[0], pair[1]);
        if station <= n {
            return add(pair[0], mul(sub(pair[1], pair[0]), station.max(0.0) / n));
        }
        station -= n;
    }
    *path.last().unwrap()
}
pub fn project(path: &[XZ], p: XZ) -> (f64, f64) {
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
pub fn substring(path: &[XZ], lo: f64, hi: f64) -> Vec<XZ> {
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
