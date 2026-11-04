//! Offline union geometry shared by road rendering and collision.
use std::collections::HashMap;
#[path = "street_surface_binary.rs"]
mod binary;
pub(crate) use binary::bake;
pub const COLORS: [[f32; 3]; 8] = [
    [0.18, 0.20, 0.22],
    [0.25, 0.26, 0.26],
    [0.32, 0.31, 0.28],
    [0.42, 0.43, 0.40],
    [0.88, 0.88, 0.80],
    [0.72, 0.71, 0.65],
    [0.66, 0.64, 0.56],
    [0.56, 0.51, 0.40],
];
pub const DEFAULT_PAVEMENT_HEIGHT: f32 = 0.14;
const CELL: f32 = 32.0;
type RoadPaths = Vec<Vec<[f32; 3]>>;
#[derive(Debug)]
pub struct SurfaceTriangle {
    pub style: usize,
    pub points: [[f32; 2]; 3],
}
#[derive(Debug)]
pub struct Curb {
    pub style: usize,
    pub ends: [[f32; 2]; 2],
    pub inward: [f32; 2],
}
#[derive(Debug)]
pub struct StreetSurface {
    pub pavement_height: f32,
    pub triangles: Vec<SurfaceTriangle>,
    pub curbs: Vec<Curb>,
    #[cfg(test)]
    pub preview_junction: Option<usize>,
    triangles_grid: HashMap<(i32, i32), Vec<usize>>,
    curbs_grid: HashMap<(i32, i32), Vec<usize>>,
}
fn cell(p: [f32; 2]) -> (i32, i32) {
    ((p[0] / CELL).floor() as i32, (p[1] / CELL).floor() as i32)
}
fn index(grid: &mut HashMap<(i32, i32), Vec<usize>>, points: &[[f32; 2]], id: usize, pad: f32) {
    let lo = cell(std::array::from_fn(|i| {
        points.iter().map(|p| p[i]).fold(f32::INFINITY, f32::min) - pad
    }));
    let hi = cell(std::array::from_fn(|i| {
        points
            .iter()
            .map(|p| p[i])
            .fold(f32::NEG_INFINITY, f32::max)
            + pad
    }));
    for x in lo.0..=hi.0 {
        for z in lo.1..=hi.1 {
            grid.entry((x, z)).or_default().push(id);
        }
    }
}
fn cross(a: [f32; 2], b: [f32; 2], p: [f32; 2]) -> f32 {
    (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
}
pub(crate) fn crc32(source: &str) -> u32 {
    crc32_bytes(source.as_bytes())
}
pub(crate) fn crc32_bytes(source: &[u8]) -> u32 {
    crc32fast::hash(source)
}

/// Compute once per load and share across assets tied to the same source data.
#[derive(Clone, Copy)]
pub(crate) struct SourceChecksums {
    pub city: u32,
    pub streets: u32,
}
impl SourceChecksums {
    pub fn new(city: &str, streets: &str) -> Self {
        Self {
            city: crc32(city),
            streets: crc32(streets),
        }
    }
}

impl StreetSurface {
    pub fn load_city(
        directory: &std::path::Path,
        source: &str,
        count: usize,
    ) -> Result<(Self, RoadPaths, SourceChecksums), String> {
        binary::load(directory, source, count).map_err(|error| {
            let name = directory.file_stem().unwrap_or_default().to_string_lossy();
            format!(
                "cannot load required collision data {}: {error}; rebuild with bake-collision --location {name}",
                directory.join("streets.bin").display()
            )
        })
    }

    pub fn load(text: &str, source: &str, count: usize) -> Result<(Self, RoadPaths), String> {
        let mut lines = text.lines();
        let header: Vec<_> = lines
            .next()
            .ok_or("missing surfaces header")?
            .split_whitespace()
            .collect();
        if header.len() != 4
            || header[0] != "STREETS"
            || header[1] != "1"
            || header[2].parse::<usize>().ok() != Some(count)
            || u32::from_str_radix(header[3], 16).ok() != Some(crc32(source))
        {
            return Err(
                "stale or unsupported street surfaces; re-import the source with add-location under a new name".into(),
            );
        }
        let mut result = Self {
            pavement_height: DEFAULT_PAVEMENT_HEIGHT,
            triangles: Vec::new(),
            curbs: Vec::new(),
            #[cfg(test)]
            preview_junction: None,
            triangles_grid: HashMap::new(),
            curbs_grid: HashMap::new(),
        };
        let mut paths = vec![Vec::new(); count];
        for line in lines {
            let f: Vec<_> = line.split_whitespace().collect();
            let n = |i: usize| -> Result<usize, String> {
                f.get(i)
                    .ok_or("missing surface index")?
                    .parse()
                    .map_err(|_| "invalid surface index".into())
            };
            let v = |i: usize| -> Result<f32, String> {
                let value: f32 = f
                    .get(i)
                    .ok_or("missing surface coordinate")?
                    .parse()
                    .map_err(|_| "invalid surface coordinate")?;
                if value.is_finite() {
                    Ok(value)
                } else {
                    Err("nonfinite surface coordinate".into())
                }
            };
            let p = |i: usize| -> Result<[f32; 2], String> { Ok([v(i)?, v(i + 1)?]) };
            match f.first().copied() {
                Some("j") => {
                    let _node = n(1)?;
                    #[cfg(test)]
                    {
                        result.preview_junction = Some(_node);
                    }
                }
                Some("p") => {
                    let id = n(1)?;
                    let length = n(2)?;
                    if id >= count
                        || length < 2
                        || f.len() != 3 + length * 2
                        || !paths[id].is_empty()
                    {
                        return Err("invalid surface path".into());
                    }
                    paths[id] = (0..length)
                        .map(|i| Ok([v(3 + i * 2)?, 0.0, v(4 + i * 2)?]))
                        .collect::<Result<_, String>>()?;
                }
                Some("t") => {
                    let style = n(1)?;
                    if style > 7 || f.len() != 8 {
                        return Err("invalid surface triangle".into());
                    }
                    let points = [p(2)?, p(4)?, p(6)?];
                    if cross(points[0], points[1], points[2]) > 0.00001 {
                        result.triangles.push(SurfaceTriangle { style, points });
                    }
                }
                Some("c") => {
                    let style = n(1)?;
                    if style > 2 || f.len() != 8 {
                        return Err("invalid curb".into());
                    }
                    result.curbs.push(Curb {
                        style,
                        ends: [p(2)?, p(4)?],
                        inward: p(6)?,
                    });
                }
                _ => return Err("unknown surface record".into()),
            }
        }
        if result.curbs.is_empty() || result.triangles.is_empty() || paths.iter().any(Vec::is_empty)
        {
            return Err("incomplete street surfaces".into());
        }
        for (i, t) in result.triangles.iter().enumerate() {
            index(&mut result.triangles_grid, &t.points, i, 0.001);
        }
        for (i, c) in result.curbs.iter().enumerate() {
            index(&mut result.curbs_grid, &c.ends, i, 32.0);
        }
        Ok((result, paths))
    }
    fn triangle_at(&self, p: [f32; 2], road_only: bool) -> Option<&SurfaceTriangle> {
        self.triangles_grid.get(&cell(p)).and_then(|ids| {
            ids.iter().map(|&i| &self.triangles[i]).find(|triangle| {
                if road_only && triangle.style > 3 {
                    return false;
                }
                let t = triangle.points;
                // A distance tolerance can extend a very thin triangle far
                // beyond its acute tip. Bound it before testing its edges.
                if (0..2).any(|axis| {
                    p[axis] < t.iter().map(|v| v[axis]).fold(f32::INFINITY, f32::min) - 0.002
                        || p[axis]
                            > t.iter().map(|v| v[axis]).fold(f32::NEG_INFINITY, f32::max) + 0.002
                }) {
                    return false;
                }
                (0..3).all(|i| {
                    let a = t[i];
                    let b = t[(i + 1) % 3];
                    cross(a, b, p) >= -0.002 * (b[0] - a[0]).hypot(b[1] - a[1])
                })
            })
        })
    }
    pub fn contains(&self, p: [f32; 2]) -> bool {
        self.triangle_at(p, true).is_some()
    }
    pub fn road_color(&self, p: [f32; 2]) -> [f32; 3] {
        self.triangle_at(p, false)
            .map_or([0.23, 0.25, 0.26], |t| COLORS[t.style])
    }
    fn nearest_in(
        &self,
        p: [f32; 2],
        ids: impl Iterator<Item = usize>,
    ) -> ([f32; 2], [f32; 2], f32) {
        ids.map(|i| {
            let c = &self.curbs[i];
            let [a, b] = c.ends;
            let delta = [b[0] - a[0], b[1] - a[1]];
            let t = (((p[0] - a[0]) * delta[0] + (p[1] - a[1]) * delta[1])
                / (delta[0] * delta[0] + delta[1] * delta[1]).max(0.000001))
            .clamp(0.0, 1.0);
            let q = [a[0] + delta[0] * t, a[1] + delta[1] * t];
            (q, c.inward, (q[0] - p[0]).hypot(q[1] - p[1]))
        })
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .unwrap()
    }
    pub fn nearest(&self, p: [f32; 2]) -> ([f32; 2], [f32; 2], f32) {
        if let Some(ids) = self.curbs_grid.get(&cell(p)) {
            let best = self.nearest_in(p, ids.iter().copied());
            if best.2 <= 32.0 {
                return best;
            }
        }
        self.nearest_in(p, 0..self.curbs.len())
    }
    pub fn clearance(&self, p: [f32; 2]) -> f32 {
        self.nearest(p).2 * if self.contains(p) { -1.0 } else { 1.0 }
    }
    pub fn paving_height(&self, p: [f32; 2]) -> Option<f32> {
        self.triangle_at(p, false).map(|t| match t.style {
            5 | 6 => self.pavement_height,
            7 => 0.08,
            _ => 0.05,
        })
    }
    /// A crossing is an event, not a continuous wall force. Parallel travel or
    /// resting alongside a curb must not repeatedly slow the car.
    pub fn crossed_curb(&self, a: [f32; 2], b: [f32; 2]) -> Option<([f32; 2], f32)> {
        if self.contains(a) == self.contains(b) {
            return None;
        }
        let mid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        let (_, normal, distance) = self.nearest(mid);
        if distance > (b[0] - a[0]).hypot(b[1] - a[1]) * 0.5 + 0.01 {
            return None;
        }
        // Physical rise is small for service verges, standard for pavements.
        // The rendered support height determines the exact ride height.
        let rise = (self.paving_height(a).unwrap_or(0.005)
            - self.paving_height(b).unwrap_or(0.005))
        .abs()
        .max(0.03);
        // Cap the bump and speed penalty so taller pavements stay climbable.
        Some((normal, rise.min(0.09)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn accelerated_checksums_match_existing_asset_format() {
        assert_eq!(crc32_bytes(b""), 0);
        assert_eq!(crc32_bytes(b"123456789"), 0xcbf43926);
        // Compare binary data and SIMD/block boundaries with the old polynomial.
        let data: Vec<u8> = (0..=u16::MAX).map(|i| i as u8).collect();
        for length in [1, 15, 16, 17, 63, 64, 65, 255, 256, 257, 65536] {
            let mut expected = !0u32;
            for &byte in &data[..length] {
                expected ^= byte as u32;
                for _ in 0..8 {
                    expected = (expected >> 1) ^ if expected & 1 != 0 { 0xedb88320 } else { 0 };
                }
            }
            assert_eq!(crc32_bytes(&data[..length]), !expected);
        }
    }

    #[test]
    fn stale_surface_package_is_rejected_before_loading_geometry() {
        let source = &crate::city_assets::read("city.txt");
        let stale = format!("{source}\n");
        assert!(
            StreetSurface::load(&crate::city_assets::read("streets.txt"), &stale, 1839).is_err()
        );
    }
    #[test]
    fn curbs_report_crossings_without_becoming_walls() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let surfaces = city.surfaces.as_ref().unwrap();
        let mut checked = 0;
        for curb in surfaces.curbs.iter().step_by(71) {
            let q = [
                (curb.ends[0][0] + curb.ends[1][0]) * 0.5,
                (curb.ends[0][1] + curb.ends[1][1]) * 0.5,
            ];
            let a = [q[0] + curb.inward[0] * 0.15, q[1] + curb.inward[1] * 0.15];
            let b = [q[0] - curb.inward[0] * 0.15, q[1] - curb.inward[1] * 0.15];
            if surfaces.contains(a) == surfaces.contains(b) {
                continue;
            }
            assert!(surfaces.crossed_curb(a, b).is_some());
            assert!(surfaces.crossed_curb(b, a).is_some());
            assert!(surfaces.crossed_curb(a, a).is_none());
            checked += 1;
        }
        assert!(checked > 30);
    }
}
