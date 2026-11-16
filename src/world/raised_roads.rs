//! Raised road triangles and their spatial index share one owner.

use crate::model;
use crate::model::world::HeightField;
use crate::model::world::RoadRise;
use crate::model::world::WorldLocation;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use geo::{BooleanOps, MultiPolygon, TriangulateEarcut};
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
fn polygon(points: [XZ<f32>; 3]) -> MultiPolygon<f64> {
    MultiPolygon(vec![util::geometry::polygon(
        &points.map(|p| util::geometry::xz_map(p, f64::from)),
    )])
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
    // Boolean clipping can leave thin fragments on either side of a ramp edge.
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
    let mut remaining = polygon(points);
    for item in candidates {
        let rise = &raised_roads.triangles[item.index];
        let footprint = polygon(rise.points.map(|p| util::geometry::xz(p)));
        let raised = remaining.intersection(&footprint);
        for triangle in raised
            .0
            .iter()
            .flat_map(|polygon| polygon.earcut_triangles())
        {
            let points = triangle.to_array().map(|p| XZ {
                x: p.x as f32,
                z: p.y as f32,
            });
            world::elevation::drape(field, points, &mut emit);
        }
        remaining = remaining.difference(&footprint);
    }
    // Preserve the original fast drape when this triangle has no nearby rises.
    if remaining == polygon(points) {
        world::elevation::drape(field, points, emit);
    } else {
        for triangle in remaining
            .0
            .iter()
            .flat_map(|polygon| polygon.earcut_triangles())
        {
            world::elevation::drape(
                field,
                triangle.to_array().map(|p| XZ {
                    x: p.x as f32,
                    z: p.y as f32,
                }),
                &mut emit,
            );
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
