//! Raised road triangles and their spatial index share one owner.

use crate::model;
use crate::model::world::HeightField;
use crate::model::world::RoadRise;
use crate::model::world::WorldLocation;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use rstar::{RTree, RTreeObject, AABB};

#[derive(Debug)]
struct IndexedRise {
    index: usize,
    bounds: AABB<XZ<f64>>,
}
impl RTreeObject for IndexedRise {
    type Envelope = AABB<XZ<f64>>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}

#[derive(Debug)]
pub struct RaisedRoads {
    triangles: Vec<RoadRise>,
    index: RTree<IndexedRise>,
}

struct Projection {
    offset: f32,
    inside: bool,
}

fn project(road_rise: &RoadRise, p: XZ<f32>) -> Projection {
    let [a, b, c] = road_rise.points;
    let cross = |a: XZ<f32>, b: XZ<f32>| a.x as f64 * b.z as f64 - a.z as f64 * b.x as f64;
    let delta = |a: XYZ, b: XYZ| XZ {
        x: a.x - b.x,
        z: a.z - b.z,
    };
    let p = util::geometry::at_height(p, 0.0);
    let denominator = cross(delta(b, a), delta(c, a));
    let u = cross(delta(p, a), delta(c, a)) / denominator;
    let v = cross(delta(b, a), delta(p, a)) / denominator;
    Projection {
        offset: (a.y as f64 + u * (b.y - a.y) as f64 + v * (c.y - a.y) as f64) as f32,
        inside: u >= -0.0001 && v >= -0.0001 && u + v <= 1.0001,
    }
}

/// Linear height offset inside this triangle, with tolerance for shared edges.
pub fn road_rise_offset(road_rise: &RoadRise, p: XZ<f32>) -> Option<f32> {
    let projection = project(road_rise, p);
    projection.inside.then_some(projection.offset)
}

/// Evaluate the height plane without restricting the point to its footprint.
pub fn plane_offset(road_rise: &RoadRise, p: XZ<f32>) -> f32 {
    project(road_rise, p).offset
}

fn bounds(points: impl Iterator<Item = XZ<f64>>) -> AABB<XZ<f64>> {
    let mut min = XZ {
        x: f64::INFINITY,
        z: f64::INFINITY,
    };
    let mut max = XZ {
        x: f64::NEG_INFINITY,
        z: f64::NEG_INFINITY,
    };
    for point in points {
        min.x = min.x.min(point.x);
        min.z = min.z.min(point.z);
        max.x = max.x.max(point.x);
        max.z = max.z.max(point.z);
    }
    AABB::from_corners(min, max)
}
fn side(a: XZ<f64>, b: XZ<f64>, p: XZ<f64>) -> f64 {
    (b.x - a.x) * (p.z - a.z) - (b.z - a.z) * (p.x - a.x)
}

struct Split {
    inside: Vec<XZ<f64>>,
    outside: Vec<XZ<f64>>,
}

// Split a convex polygon along a ramp edge. Both halves share the exact same
// intersection coordinates, without quantizing thin fragments onto an overlay grid.
fn split(points: &[XZ<f64>], a: XZ<f64>, b: XZ<f64>, winding: f64) -> Split {
    let mut result = Split {
        inside: Vec::new(),
        outside: Vec::new(),
    };
    let Some(&last) = points.last() else {
        return result;
    };
    let mut previous = last;
    let mut before = side(a, b, previous) * winding;
    for &current in points {
        let after = side(a, b, current) * winding;
        if (before < 0.0 && after > 0.0) || (before > 0.0 && after < 0.0) {
            let t = before / (before - after);
            let point = XZ {
                x: previous.x + (current.x - previous.x) * t,
                z: previous.z + (current.z - previous.z) * t,
            };
            result.inside.push(point);
            result.outside.push(point);
        }
        if after >= 0.0 {
            result.inside.push(current);
        }
        if after <= 0.0 {
            result.outside.push(current);
        }
        previous = current;
        before = after;
    }
    result
}

fn has_area(points: &[XZ<f64>]) -> bool {
    (1..points.len().saturating_sub(1)).any(|i| side(points[0], points[i], points[i + 1]) != 0.0)
}

fn drape_convex(field: &HeightField, points: &[XZ<f64>], mut emit: impl FnMut([XYZ; 3])) {
    for i in 1..points.len().saturating_sub(1) {
        let triangle = [points[0], points[i], points[i + 1]];
        if side(triangle[0], triangle[1], triangle[2]) != 0.0 {
            world::elevation::drape(
                field,
                triangle.map(|p| util::geometry::xz_map(p, |v| v as f32)),
                &mut emit,
            );
        }
    }
}

pub fn lowered_footprints(
    raised_roads: &RaisedRoads,
) -> impl Iterator<Item = geo::Polygon<f64>> + '_ {
    raised_roads
        .triangles
        .iter()
        .filter(|rise| rise.points.iter().any(|p| p.y < -0.03))
        .map(|rise| {
            util::geometry::polygon(
                &rise
                    .points
                    .map(|p| util::geometry::xz_map(util::geometry::xz(p), f64::from)),
            )
        })
}

pub fn new(triangles: Vec<RoadRise>) -> RaisedRoads {
    let index = RTree::bulk_load(
        triangles
            .iter()
            .enumerate()
            .map(|(index, rise)| IndexedRise {
                index,
                bounds: bounds(
                    rise.points
                        .iter()
                        .map(|p| util::geometry::xz_map(util::geometry::xz(*p), f64::from)),
                ),
            })
            .collect(),
    );
    RaisedRoads { triangles, index }
}
fn raised_roads_offset(raised_roads: &RaisedRoads, p: XZ<f32>) -> f32 {
    supporting_rise(raised_roads, p)
        .map(|rise| project(rise, p).offset)
        .unwrap_or(0.0)
}

fn supporting_rise(raised_roads: &RaisedRoads, p: XZ<f32>) -> Option<&RoadRise> {
    raised_roads
        .index
        .locate_in_envelope_intersecting(&AABB::from_point(util::geometry::xz_map(p, f64::from)))
        .map(|item| &raised_roads.triangles[item.index])
        .filter(|rise| project(rise, p).inside)
        .max_by(|a, b| project(a, p).offset.total_cmp(&project(b, p).offset))
}
fn drape(
    raised_roads: &RaisedRoads,
    field: &HeightField,
    points: [XZ<f32>; 3],
    mut emit: impl FnMut([XYZ; 3]),
) {
    let mut candidates = raised_roads
        .index
        .locate_in_envelope_intersecting(&bounds(
            points
                .into_iter()
                .map(|p| util::geometry::xz_map(p, f64::from)),
        ))
        .peekable();
    if candidates.peek().is_none() {
        world::elevation::drape(field, points, emit);
        return;
    }
    // Clipping can leave thin fragments on either side of a ramp edge.
    // Select each fragment's height plane through the contact query, including
    // pieces classified as remaining ground, so rounding cannot lower slivers.
    let mut emit = |mut triangle: [XYZ; 3]| {
        let center = util::geometry::xz_from_fn(|axis| {
            triangle
                .iter()
                .map(|p| util::geometry::xz_component(util::geometry::xz(*p), axis))
                .sum::<f32>()
                / 3.
        });
        // Preserve the ground side of intentional vertical edges on authored
        // humps; only a fragment supported by a ramp inherits its height.
        if let Some(rise) = supporting_rise(raised_roads, center) {
            for point in &mut triangle {
                point.y += project(rise, util::geometry::xz(*point)).offset;
            }
        }
        emit(triangle);
    };
    let original = points
        .map(|p| util::geometry::xz_map(p, f64::from))
        .to_vec();
    let mut remaining = vec![original.clone()];
    for item in candidates {
        let rise = &raised_roads.triangles[item.index];
        let footprint = rise
            .points
            .map(|p| util::geometry::xz_map(util::geometry::xz(p), f64::from));
        let winding = side(footprint[0], footprint[1], footprint[2]).signum();
        let mut next = Vec::new();
        for polygon in remaining {
            let mut inside = polygon.clone();
            let mut outside = Vec::new();
            for edge in 0..3 {
                let parts = split(&inside, footprint[edge], footprint[(edge + 1) % 3], winding);
                if has_area(&parts.outside) {
                    outside.push(parts.outside);
                }
                inside = parts.inside;
            }
            if has_area(&inside) {
                drape_convex(field, &inside, &mut emit);
                next.extend(outside);
            } else {
                // A bounding-box candidate need not intersect the polygon.
                next.push(polygon);
            }
        }
        remaining = next;
    }
    // Preserve the original fast drape when no candidate intersects this triangle.
    if remaining.len() == 1 && remaining[0] == original {
        world::elevation::drape(field, points, emit);
    } else {
        for polygon in remaining {
            drape_convex(field, &polygon, &mut emit);
        }
    }
}
fn segment(raised_roads: &RaisedRoads, field: &HeightField, a: XZ<f32>, b: XZ<f32>) -> Vec<XYZ> {
    let dx = (b.x - a.x) as f64;
    let dz = (b.z - a.z) as f64;
    let length_squared = dx * dx + dz * dz;
    if length_squared < 0.000001 {
        return world::elevation::segment(field, a, b);
    }
    let mut stations: Vec<f64> = world::elevation::segment(field, a, b)
        .iter()
        .map(|p| ((p.x - a.x) as f64 * dx + (p.z - a.z) as f64 * dz) / length_squared)
        .collect();
    for item in raised_roads.index.locate_in_envelope_intersecting(&bounds(
        [a, b]
            .into_iter()
            .map(|p| util::geometry::xz_map(p, f64::from)),
    )) {
        let points = raised_roads.triangles[item.index].points;
        for i in 0..3 {
            let c = points[i];
            let d = points[(i + 1) % 3];
            let ex = (d.x - c.x) as f64;
            let ez = (d.z - c.z) as f64;
            let denominator = dx * ez - dz * ex;
            if denominator.abs() < 0.00000001 {
                continue;
            }
            let cx = (c.x - a.x) as f64;
            let cz = (c.z - a.z) as f64;
            let t = (cx * ez - cz * ex) / denominator;
            let u = (cx * dz - cz * dx) / denominator;
            if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                stations.push(t);
            }
        }
    }
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|a, b| (*a - *b).abs() < 0.000001);
    stations
        .into_iter()
        .map(|t| {
            let p = XZ {
                x: a.x + dx as f32 * t as f32,
                z: a.z + dz as f32 * t as f32,
            };
            util::geometry::at_height(
                p,
                world::elevation::height(field, p) + raised_roads_offset(raised_roads, p),
            )
        })
        .collect()
}

pub fn road_rise(world: &WorldLocation, p: XZ<f32>) -> f32 {
    raised_roads_offset(&world.raised_roads, p)
}
pub fn drape_road_triangle(
    world: &WorldLocation,
    points: [XZ<f32>; 3],
    emit: impl FnMut([XYZ; 3]),
) {
    let field = model::world::elevation(world).expect("raised road rendering requires elevation");
    drape(&world.raised_roads, field, points, emit);
}
pub fn raised_road_segment(world: &WorldLocation, a: XZ<f32>, b: XZ<f32>) -> Vec<XYZ> {
    let field = model::world::elevation(world).expect("raised road rendering requires elevation");
    segment(&world.raised_roads, field, a, b)
}

#[cfg(test)]
#[path = "tests/raised_roads.rs"]
mod tests;
