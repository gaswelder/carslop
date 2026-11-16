//! Map-space exclusion of proposed tree centres, using the actual road mesh.

use crate::build;
use crate::model;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::XZ;
use geo::{Coord, Intersects, LineString, Point, Polygon};
use rstar::{RTree, RTreeObject, AABB};
use std::{
    io::{BufRead, BufReader},
    path::Path,
};
struct Obstacle {
    polygon: Polygon<f64>,
    bounds: AABB<XZ>,
    reason: &'static str,
}
impl RTreeObject for Obstacle {
    type Envelope = AABB<XZ>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}
pub struct Obstacles {
    index: RTree<Obstacle>,
}
fn local(origin: [f64; 2], lon: f64, lat: f64) -> XZ {
    build::imagery::local(origin, [lon, lat])
}
fn obstacle(points: Vec<XZ>, reason: &'static str, region: &AABB<XZ>) -> Option<Obstacle> {
    let lo = util::geometry::xz_from_fn(|i| {
        points
            .iter()
            .map(|p| util::geometry::xz_component(*p, i))
            .fold(f64::INFINITY, f64::min)
    });
    let hi = util::geometry::xz_from_fn(|i| {
        points
            .iter()
            .map(|p| util::geometry::xz_component(*p, i))
            .fold(f64::NEG_INFINITY, f64::max)
    });
    let bounds = AABB::from_corners(lo, hi);
    use rstar::Envelope;
    if !bounds.intersects(region) {
        return None;
    }
    let ring = LineString::new(
        points
            .into_iter()
            .map(|p| Coord { x: p.x, y: p.z })
            .collect(),
    );
    Some(Obstacle {
        polygon: Polygon::new(ring, vec![]),
        bounds,
        reason,
    })
}

pub fn read_paths(
    city_path: &Path,
    streets_path: &Path,
    lonlat_bounds: [f64; 4],
) -> Result<Obstacles> {
    let text = std::fs::read_to_string(city_path)?;
    let city = model::source_location::parse(&text).map_err(invalid)?;
    let origin_line = text
        .lines()
        .find(|l| l.starts_with("origin "))
        .ok_or_else(|| invalid("city missing origin"))?;
    let mut parts = origin_line.split_whitespace().skip(1);
    let origin = [
        parts
            .next()
            .ok_or_else(|| invalid("missing origin latitude"))?
            .parse::<f64>()?,
        parts
            .next()
            .ok_or_else(|| invalid("missing origin longitude"))?
            .parse::<f64>()?,
    ];
    let nw = local(origin, lonlat_bounds[0], lonlat_bounds[3]);
    let se = local(origin, lonlat_bounds[2], lonlat_bounds[1]);
    let region = AABB::from_corners(nw, se);
    let mut objects = vec![];
    for building in &city.buildings {
        if let Some(o) = obstacle(
            building
                .ground_outline
                .iter()
                .map(|p| util::geometry::xz_map(*p, f64::from))
                .collect(),
            "building",
            &region,
        ) {
            objects.push(o);
        }
    }
    println!("Checking road surfaces for tree exclusions...");
    let mut input = BufReader::new(std::fs::File::open(streets_path)?);
    let mut line = String::new();
    input.read_line(&mut line)?;
    let header: Vec<_> = line.split_whitespace().collect();
    if header.len() < 3
        || header[..2] != ["STREETS", "1"]
        || header[2].parse::<usize>().ok() != Some(city.road_lines.len())
    {
        return Err(invalid("invalid street surfaces header"));
    }
    loop {
        line.clear();
        if input.read_line(&mut line)? == 0 {
            break;
        }
        let mut f = line.split_whitespace();
        if f.next() != Some("t") {
            continue;
        }
        let style = f
            .next()
            .ok_or_else(|| invalid("missing surface style"))?
            .parse::<usize>()?;
        // 0..=3 are asphalt; 4..=7 are pavement/other pedestrian surfaces.
        if style > 3 {
            continue;
        }
        let mut points = vec![];
        for _ in 0..3 {
            let mut p = XZ::<f64>::default();
            for v in util::geometry::xz_iter_mut(&mut p) {
                *v = f
                    .next()
                    .ok_or_else(|| invalid("missing surface coordinate"))?
                    .parse::<f64>()?;
                if !v.is_finite() {
                    return Err(invalid("invalid road coordinate"));
                }
            }
            points.push(p);
        }
        if f.next().is_some() {
            return Err(invalid("invalid road triangle"));
        }
        let [a, b, c] = [points[0], points[1], points[2]];
        if (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x) <= 0.00001 {
            continue;
        }
        if let Some(o) = obstacle(points, "road", &region) {
            objects.push(o);
        }
    }
    Ok(Obstacles {
        index: RTree::bulk_load(objects),
    })
}
pub fn rejection_at(obstacles: &Obstacles, p: XZ) -> Option<&'static str> {
    let point = Point::new(p.x, p.z);
    let mut road = false;
    for o in obstacles
        .index
        .locate_in_envelope_intersecting(&AABB::from_point(p))
    {
        if o.polygon.intersects(&point) {
            if o.reason == "building" {
                return Some("building");
            }
            road = true;
        }
    }
    if road {
        Some("road")
    } else {
        None
    }
}
