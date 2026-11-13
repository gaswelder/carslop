//! Offline union geometry shared by road rendering and collision.
use crate::geometry::{XYZ, XZ};
use crate::location;
#[cfg(test)]
use crate::test_support;
pub use crate::world::{Curb, Triangle};
use crate::world::{CurbStyle, Location, Material, TriangleUsage};
use std::collections::HashMap;

/// Runtime centrelines in local metres, indexed like Location::road_lines.
pub type RoadPaths = Vec<Vec<XYZ>>;

pub const DEFAULT_PAVEMENT_HEIGHT: f32 = 0.14;

#[path = "street_surface_binary.rs"]
pub mod binary;

const CELL: f32 = 32.0;

fn cell(p: XZ<f32>) -> (i32, i32) {
    ((p.x / CELL).floor() as i32, (p.z / CELL).floor() as i32)
}

fn index(grid: &mut HashMap<(i32, i32), Vec<usize>>, points: &[XZ<f32>], id: usize, pad: f32) {
    let lo = cell(XZ::from_fn(|i| {
        points
            .iter()
            .map(|p| p.component(i))
            .fold(f32::INFINITY, f32::min)
            - pad
    }));
    let hi = cell(XZ::from_fn(|i| {
        points
            .iter()
            .map(|p| p.component(i))
            .fold(f32::NEG_INFINITY, f32::max)
            + pad
    }));
    for x in lo.0..=hi.0 {
        for z in lo.1..=hi.1 {
            grid.entry((x, z)).or_default().push(id);
        }
    }
}

fn cross(a: XZ<f32>, b: XZ<f32>, p: XZ<f32>) -> f32 {
    (b.x - a.x) * (p.z - a.z) - (b.z - a.z) * (p.x - a.x)
}

/// Temporary compiled surface data transferred into a Location after loading.
#[derive(Debug)]
pub struct StreetSurface {
    pavement_height_meters: f32,
    /// Road, marking and pavement triangles.
    road_triangles: Vec<Triangle>,
    /// Road-edge segments used for clearance and crossing queries.
    curbs: Vec<Curb>,
    /// Optional node index selected for junction previews in test builds.
    #[cfg(test)]
    preview_junction: Option<usize>,
    /// 32 m [X, Z] grid cells mapped to candidate triangle indices.
    triangles_grid: HashMap<(i32, i32), Vec<usize>>,
    /// 32 m [X, Z] grid cells mapped to candidate curb indices.
    curbs_grid: HashMap<(i32, i32), Vec<usize>>,
}

impl StreetSurface {
    pub fn from_geometry(
        data: location::Streets,
        height: f32,
    ) -> Result<(Self, RoadPaths), String> {
        let mut result = Self {
            pavement_height_meters: DEFAULT_PAVEMENT_HEIGHT,
            road_triangles: data.triangles,
            curbs: data.curbs,
            triangles_grid: HashMap::new(),
            curbs_grid: HashMap::new(),
            #[cfg(test)]
            preview_junction: None,
        };
        result.set_pavement_height(height)?;
        for (i, t) in result.road_triangles.iter().enumerate() {
            index(&mut result.triangles_grid, &t.points, i, 0.001);
        }
        for (i, c) in result.curbs.iter().enumerate() {
            index(&mut result.curbs_grid, &c.ends, i, 32.0);
        }
        let paths = data
            .paths
            .into_iter()
            .map(|p| p.into_iter().map(|p| p.at_height(0.0)).collect())
            .collect();
        Ok((result, paths))
    }
    pub fn road_triangles(&self) -> &[Triangle] {
        &self.road_triangles
    }
    pub fn curbs(&self) -> &[Curb] {
        &self.curbs
    }
    pub fn pavement_height_meters(&self) -> f32 {
        self.pavement_height_meters
    }
    fn set_pavement_height(&mut self, height: f32) -> Result<(), String> {
        if !(0.08..=0.25).contains(&height) {
            return Err("invalid pavement height".into());
        }
        self.pavement_height_meters = height;
        Ok(())
    }

    pub fn load(streets_txt_source: &str, count: usize) -> Result<(Self, RoadPaths), String> {
        let mut lines123213 = streets_txt_source.lines();
        let header: Vec<_> = lines123213
            .next()
            .ok_or("missing surfaces header")?
            .split_whitespace()
            .collect();
        if header.len() < 3
            || header[0] != "STREETS"
            || header[1] != "1"
            || header[2].parse::<usize>().ok() != Some(count)
        {
            return Err("unsupported street surfaces header or road count".into());
        }
        let mut result = Self {
            pavement_height_meters: DEFAULT_PAVEMENT_HEIGHT,
            road_triangles: Vec::new(),
            curbs: Vec::new(),
            #[cfg(test)]
            preview_junction: None,
            triangles_grid: HashMap::new(),
            curbs_grid: HashMap::new(),
        };
        let mut paths = vec![Vec::new(); count];
        for line in lines123213 {
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
            let p = |i: usize| -> Result<XZ<f32>, String> {
                Ok(XZ {
                    x: v(i)?,
                    z: v(i + 1)?,
                })
            };
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
                        .map(|i| {
                            Ok(XYZ {
                                x: v(3 + i * 2)?,
                                y: 0.0,
                                z: v(4 + i * 2)?,
                            })
                        })
                        .collect::<Result<_, String>>()?;
                }
                Some("t") => {
                    let id = n(1)?;
                    if f.len() != 8 {
                        return Err("invalid surface triangle".into());
                    }
                    let points = [p(2)?, p(4)?, p(6)?];
                    if cross(points[0], points[1], points[2]) > 0.00001 {
                        result
                            .road_triangles
                            .push(Triangle::from_street_id(id, points)?);
                    }
                }
                Some("c") => {
                    let style = CurbStyle::try_from(n(1)?)?;
                    if f.len() != 8 {
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
        if result.curbs.is_empty()
            || result.road_triangles.is_empty()
            || paths.iter().any(Vec::is_empty)
        {
            return Err("incomplete street surfaces".into());
        }
        for (i, t) in result.road_triangles.iter().enumerate() {
            index(&mut result.triangles_grid, &t.points, i, 0.001);
        }
        for (i, c) in result.curbs.iter().enumerate() {
            index(&mut result.curbs_grid, &c.ends, i, 32.0);
        }
        Ok((result, paths))
    }
}

impl StreetSurface {
    #[cfg(test)]
    fn into_test_location(self) -> Location {
        let mut location = test_support::empty_world();
        test_support::install_surfaces(&mut location, self);
        location
    }
}

impl TryFrom<usize> for CurbStyle {
    type Error = &'static str;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::MajorRoad),
            1 => Ok(Self::Road),
            2 => Ok(Self::ServiceRoad),
            _ => Err("unknown curb style"),
        }
    }
}

impl TriangleUsage {
    /// Decode the categories used by streets.txt and streets.bin.
    fn from_street_id(id: usize) -> Result<Self, &'static str> {
        match id {
            0 => Ok(Self::Road(CurbStyle::MajorRoad)),
            1 => Ok(Self::Road(CurbStyle::Road)),
            2 => Ok(Self::Road(CurbStyle::ServiceRoad)),
            3 => Ok(Self::Gutter),
            4 => Ok(Self::Marking),
            5 => Ok(Self::Pavement(CurbStyle::MajorRoad)),
            6 => Ok(Self::Pavement(CurbStyle::Road)),
            7 => Ok(Self::Pavement(CurbStyle::ServiceRoad)),
            _ => Err("unknown street triangle category"),
        }
    }

    pub fn street_id(self) -> Result<usize, &'static str> {
        match self {
            Self::Road(CurbStyle::MajorRoad) => Ok(0),
            Self::Road(CurbStyle::Road) => Ok(1),
            Self::Road(CurbStyle::ServiceRoad) => Ok(2),
            Self::Gutter => Ok(3),
            Self::Marking => Ok(4),
            Self::Pavement(CurbStyle::MajorRoad) => Ok(5),
            Self::Pavement(CurbStyle::Road) => Ok(6),
            Self::Pavement(CurbStyle::ServiceRoad) => Ok(7),
            Self::Ground => Err("street format does not support ground triangles"),
        }
    }

    /// Existing street formats imply material from the geometry category.
    fn street_material(self) -> Result<Material, &'static str> {
        match self {
            Self::Road(_) => Ok(Material::Asphalt),
            Self::Gutter | Self::Pavement(CurbStyle::MajorRoad) => Ok(Material::Concrete),
            Self::Marking => Ok(Material::Paint),
            Self::Pavement(CurbStyle::Road) => Ok(Material::Paving),
            Self::Pavement(CurbStyle::ServiceRoad) => Ok(Material::BareGround),
            Self::Ground => Err("street format does not support ground triangles"),
        }
    }
}

impl Triangle {
    fn from_street_id(id: usize, points: [XZ<f32>; 3]) -> Result<Self, &'static str> {
        let usage = TriangleUsage::from_street_id(id)?;
        Ok(Self {
            material: usage.street_material()?,
            usage,
            points,
        })
    }

    fn street_id(&self) -> Result<usize, &'static str> {
        if self.material != self.usage.street_material()? {
            return Err("street format cannot encode this material for the triangle category");
        }
        self.usage.street_id()
    }
}

impl Location {
    pub fn paved(&self, p: XZ<f32>) -> bool {
        if let Some(h) = self.paving_height(p) {
            return h != 0.08;
        }
        matches!(
            self.ground_material(p),
            Some(Material::Asphalt | Material::Paving | Material::Concrete)
        )
    }

    /// Whether this location uses compiled surface geometry rather than centreline widths.
    pub fn has_compiled_roads(&self) -> bool {
        self.compiled_surfaces().is_some()
    }

    pub fn triangle_at(&self, p: XZ<f32>, road_only: bool) -> Option<&Triangle> {
        self.compiled_surfaces()?.triangle_at(p, road_only)
    }
    pub fn contains_road(&self, p: XZ<f32>) -> bool {
        self.triangle_at(p, true).is_some()
    }
    pub fn nearest_curb(&self, p: XZ<f32>) -> (XZ<f32>, XZ<f32>, f32) {
        self.compiled_surfaces()
            .expect("curb query requires compiled roads")
            .nearest_curb(p)
    }
    pub fn surface_clearance(&self, p: XZ<f32>) -> f32 {
        self.nearest_curb(p).2 * if self.contains_road(p) { -1.0 } else { 1.0 }
    }
    pub fn paving_height(&self, p: XZ<f32>) -> Option<f32> {
        self.triangle_at(p, false)
            .map(|t| t.usage.support_height(self.pavement_height_meters()))
    }
}

impl StreetSurface {
    fn triangle_at(&self, p: XZ<f32>, road_only: bool) -> Option<&Triangle> {
        self.triangles_grid
            .get(&cell(p))
            .and_then(|ids| {
                ids.iter()
                    .map(|&i| &self.road_triangles[i])
                    .find(|triangle| {
                        if road_only && !triangle.usage.is_road() {
                            return false;
                        }
                        let t = triangle.points;
                        // A distance tolerance can extend a very thin triangle far
                        // beyond its acute tip. Bound it before testing its edges.
                        if (0..2).any(|axis| {
                            p.component(axis)
                                < t.iter()
                                    .map(|v| v.component(axis))
                                    .fold(f32::INFINITY, f32::min)
                                    - 0.002
                                || p.component(axis)
                                    > t.iter()
                                        .map(|v| v.component(axis))
                                        .fold(f32::NEG_INFINITY, f32::max)
                                        + 0.002
                        }) {
                            return false;
                        }
                        (0..3).all(|i| {
                            let a = t[i];
                            let b = t[(i + 1) % 3];
                            cross(a, b, p)
                                >= -0.002 * (b.x - a.x).hypot(b.z - a.z)
                        })
                    })
            })
    }
    fn nearest_curb_in(
        &self,
        p: XZ<f32>,
        ids: impl Iterator<Item = usize>,
    ) -> (XZ<f32>, XZ<f32>, f32) {
        ids.map(|i| {
            let c = &self.curbs()[i];
            let [a, b] = c.ends;
            let delta = XZ {
                x: b.x - a.x,
                z: b.z - a.z,
            };
            let t = (((p.x - a.x) * delta.x + (p.z - a.z) * delta.z)
                / (delta.x * delta.x + delta.z * delta.z).max(0.000001))
            .clamp(0.0, 1.0);
            let q = XZ {
                x: a.x + delta.x * t,
                z: a.z + delta.z * t,
            };
            (q, c.inward, (q.x - p.x).hypot(q.z - p.z))
        })
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .unwrap()
    }
    fn nearest_curb(&self, p: XZ<f32>) -> (XZ<f32>, XZ<f32>, f32) {
        if let Some(ids) = self.curbs_grid.get(&cell(p)) {
            let best = self.nearest_curb_in(p, ids.iter().copied());
            if best.2 <= 32.0 {
                return best;
            }
        }
        self.nearest_curb_in(p, 0..self.curbs().len())
    }
}

#[cfg(test)]
mod index_tests {
    use super::*;
    use crate::geometry;
    use crate::world;
    use geo::{Intersects, Point};

    #[test]
    fn surface_owner_matches_full_triangle_and_curb_scans_at_grid_boundaries() {
        let text = concat!(
            "STREETS 1 1\np 0 2 -64 0 64 0\n",
            "t 1 -64 -4 64 -4 64 4\nt 1 -64 -4 64 4 -64 4\n",
            "t 6 -64 4 64 4 64 8\nt 6 -64 4 64 8 -64 8\n",
            "c 1 -64 4 64 4 0 -1\nc 1 -64 -4 64 -4 0 1\n"
        );
        let (surface, _) = StreetSurface::load(text, 1).unwrap();
        for x in (-100..=100).step_by(7) {
            for z in (-60..=60).step_by(3) {
                let p = XZ {
                    x: x as f32 + 0.13,
                    z: z as f32 + 0.37,
                };
                for road_only in [false, true] {
                    let expected = surface
                        .road_triangles()
                        .iter()
                        .find(|t| {
                            (!road_only || t.usage.is_road())
                                && geometry::polygon(&t.points.map(|p| p.map(f64::from)))
                                    .intersects(&Point::new(p.x as f64, p.z as f64))
                        })
                        .map(|t| t.usage);
                    assert_eq!(
                        surface.triangle_at(p, road_only).map(|t| t.usage),
                        expected,
                        "{p:?}"
                    );
                }
                let expected = surface
                    .curbs()
                    .iter()
                    .map(|c| {
                        let q = world::spatial::nearest_edge(p, c.ends[0], c.ends[1]);
                        (q, c.inward, (q.x - p.x).hypot(q.z - p.z))
                    })
                    .min_by(|a, b| a.2.total_cmp(&b.2))
                    .unwrap();
                assert_eq!(surface.nearest_curb(p), expected, "{p:?}");
            }
        }
    }
}
