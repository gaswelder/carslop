//! Map-space exclusion of proposed tree centres, using the actual road mesh.

use crate::build;
use crate::storage;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::XZ;
use geo::{Coord, Intersects, LineString, Point, Polygon};
use rstar::{RTree, RTreeObject, AABB};
use std::path::Path;
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

pub fn read_paths(directory: &Path, lonlat_bounds: [f64; 4]) -> Result<Obstacles> {
    let city = storage::city_assets::get_city(&storage::city_assets::new(directory.to_path_buf()))
        .map_err(invalid)?;
    let origin = city
        .origin
        .ok_or_else(|| invalid("manifest missing origin"))?;
    let origin = [origin.lat, origin.lon];
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
    for points in build::streets::read_tree_obstacles(directory, city.road_lines.len())? {
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
