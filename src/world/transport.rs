//! Vertical road support and edge contact without joining crossing road levels.

use crate::model;
use crate::model::map_details::Deck;
use crate::model::map_details::RoadSurface;
use crate::model::world::RoadRise;
use crate::model::world::WorldLocation;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

const BRIDGE_JOIN_HEIGHT: f32 = 1.;

pub struct EdgeSegment {
    pub start: XYZ,
    pub end: XYZ,
}

struct Interval {
    start: f64,
    end: f64,
}

fn clip_interval(interval: &mut Interval, before: f64, after: f64) -> bool {
    if before < 0. && after < 0. {
        return false;
    }
    if (before < 0.) != (after < 0.) {
        let t = before / (before - after);
        if before < 0. {
            interval.start = interval.start.max(t);
        } else {
            interval.end = interval.end.min(t);
        }
    }
    interval.start < interval.end
}

fn covered_interval(start: XYZ, end: XYZ, triangle: &RoadRise) -> Option<Interval> {
    let mut interval = Interval { start: 0., end: 1. };
    let side = |a: XYZ, b: XYZ, p: XYZ| {
        (f64::from(b.x) - f64::from(a.x)) * (f64::from(p.z) - f64::from(a.z))
            - (f64::from(b.z) - f64::from(a.z)) * (f64::from(p.x) - f64::from(a.x))
    };
    let [a, b, c] = triangle.points;
    let area = side(a, b, c);
    if area.abs() < 0.00000001 {
        return None;
    }
    let orientation = area.signum();
    let inside = (0..3).all(|i| {
        let a = triangle.points[i];
        let b = triangle.points[(i + 1) % 3];
        clip_interval(
            &mut interval,
            side(a, b, start) * orientation,
            side(a, b, end) * orientation,
        )
    });
    if !inside {
        return None;
    }
    let difference = |p: XYZ| {
        f64::from(p.y - world::raised_roads::plane_offset(triangle, util::geometry::xz(p)))
    };
    let clearance = f64::from(BRIDGE_JOIN_HEIGHT);
    if clip_interval(
        &mut interval,
        clearance - difference(start),
        clearance - difference(end),
    ) && clip_interval(
        &mut interval,
        clearance + difference(start),
        clearance + difference(end),
    ) {
        return Some(interval);
    }
    None
}

/// Rails belong to the outside of joined bridge decks, including pit lanes.
pub fn exposed_bridge_edge(
    world: &WorldLocation,
    deck: &Deck,
    start: XYZ,
    end: XYZ,
) -> Vec<EdgeSegment> {
    let mut covered = Vec::new();
    let point = |t: f64| {
        util::geometry::xyz_from_fn(|axis| {
            util::geometry::xyz_component(start, axis)
                + (util::geometry::xyz_component(end, axis)
                    - util::geometry::xyz_component(start, axis))
                    * t as f32
        })
    };
    for other in world::map_details::decks_near_segment(
        &world.details,
        util::geometry::xz(start),
        util::geometry::xz(end),
    )
    .filter(|other| other.road != deck.road && other.grade > 0)
    {
        for triangle in world::map_details::deck_triangles(other) {
            if let Some(interval) = covered_interval(start, end, &triangle) {
                covered.push(interval);
            }
        }
    }
    // A connected approach can cover an edge just before its bridge deck starts.
    // Compare actual draped road heights to retain rails above roads underneath.
    for triangle in world::roads::road_triangles(world) {
        if !model::world::is_road(triangle.usage) {
            continue;
        }
        let intersects = (0..2).all(|axis| {
            let a = util::geometry::xz_component(util::geometry::xz(start), axis);
            let b = util::geometry::xz_component(util::geometry::xz(end), axis);
            let min = triangle
                .points
                .iter()
                .map(|p| util::geometry::xz_component(*p, axis))
                .fold(f32::INFINITY, f32::min);
            let max = triangle
                .points
                .iter()
                .map(|p| util::geometry::xz_component(*p, axis))
                .fold(f32::NEG_INFINITY, f32::max);
            min <= a.max(b) && max >= a.min(b)
        });
        if !intersects {
            continue;
        }
        world::raised_roads::drape_road_triangle(world, triangle.points, |points| {
            if let Some(interval) = covered_interval(start, end, &RoadRise { points }) {
                covered.push(interval);
            }
        });
    }
    covered.sort_by(|a, b| a.start.total_cmp(&b.start));
    let mut result = Vec::new();
    let mut cursor = 0.;
    for interval in covered {
        if interval.start > cursor + 0.000001 {
            result.push(EdgeSegment {
                start: point(cursor),
                end: point(interval.start),
            });
        }
        cursor = cursor.max(interval.end);
    }
    if cursor < 1. - 0.000001 {
        result.push(EdgeSegment {
            start: point(cursor),
            end,
        });
    }
    result
}

fn bridge_edge_covered(world: &WorldLocation, deck: &Deck, p: XYZ) -> bool {
    deck.grade > 0
        && (world::map_details::nearby_decks(&world.details, util::geometry::xz(p))
            .filter(|other| other.road != deck.road && other.grade > 0)
            .any(|other| {
                world::map_details::height_on(other, util::geometry::xz(p))
                    .is_some_and(|height| (height - p.y).abs() <= BRIDGE_JOIN_HEIGHT)
            })
            || world::street_surface::world_location_triangle_at(
                world,
                util::geometry::xz(p),
                true,
            )
            .is_some_and(|triangle| {
                model::world::is_road(triangle.usage)
                    && (physics::vehicle_location::drive_height(world, util::geometry::xz(p)) - p.y)
                        .abs()
                        <= BRIDGE_JOIN_HEIGHT
            }))
}

pub fn tunnel_openings(world: &WorldLocation) -> geo::MultiPolygon<f64> {
    let pieces: Vec<_> = world::raised_roads::lowered_footprints(&world.raised_roads).collect();
    if pieces.is_empty() {
        util::geometry::empty()
    } else {
        geo::algorithm::unary_union(&pieces)
    }
}

pub fn paved_near(world: &WorldLocation, p: XYZ) -> bool {
    let reference = p.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
    world::map_details::nearby_decks(&world.details, util::geometry::xz(p))
        .find(|deck| {
            world::map_details::height_on(deck, util::geometry::xz(p))
                .is_some_and(|h| (h - reference).abs() < 1.5)
        })
        .map_or_else(
            || world::street_surface::paved(world, util::geometry::xz(p)),
            |deck| deck.surface != RoadSurface::Unpaved,
        )
}
pub fn deck_height(world: &WorldLocation, p: XZ<f32>, reference: f32) -> Option<f32> {
    world::map_details::deck_height(&world.details, p, reference)
}
pub fn drive_height_near(world: &WorldLocation, p: XZ<f32>, reference: f32) -> f32 {
    let ground = physics::vehicle_location::drive_height(world, p);
    deck_height(world, p, reference)
        .filter(|height| (height - reference).abs() < (ground - reference).abs())
        .unwrap_or(ground)
}
pub fn constrain_transport(
    world: &WorldLocation,
    position: &mut XYZ,
    velocity: &mut XZ<f32>,
) -> f32 {
    let mut impact: f32 = 0.;
    for deck in world::map_details::nearby_decks(&world.details, util::geometry::xz(*position)) {
        for pair in deck.path.windows(2) {
            let a = XZ {
                x: pair[0][0],
                z: pair[0][2],
            };
            let b = XZ {
                x: pair[1][0],
                z: pair[1][2],
            };
            let dx = b.x - a.x;
            let dz = b.z - a.z;
            let length = dx.hypot(dz);
            let along = ((position.x - a.x) * dx + (position.z - a.z) * dz) / length;
            if along < 0.05 || along > length - 0.05 {
                continue;
            }
            let height = pair[0][1] + (pair[1][1] - pair[0][1]) * along / length;
            if (position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT - height).abs() > 1.5
            {
                continue;
            }
            let normal = XZ {
                x: -dz / length,
                z: dx / length,
            };
            let lateral = (position.x - a.x) * normal.x + (position.z - a.z) * normal.z;
            let limit = deck.width / 2. - world::locations::CAR_RADIUS - 0.08;
            if lateral.abs() > limit && lateral.abs() < deck.width / 2. + 2. {
                let edge = XYZ {
                    x: a.x + dx * along / length + normal.x * lateral.signum() * deck.width / 2.,
                    y: height,
                    z: a.z + dz * along / length + normal.z * lateral.signum() * deck.width / 2.,
                };
                if bridge_edge_covered(world, deck, edge) {
                    continue;
                }
                let penetration = lateral.abs() - limit;
                let n = XZ {
                    x: normal.x * lateral.signum(),
                    z: normal.z * lateral.signum(),
                };
                position.x -= n.x * penetration;
                position.z -= n.z * penetration;
                impact = impact.max(physics::vehicle_location::deflect(velocity, n));
            }
        }
    }
    impact
}
pub fn tunnel_ceiling(world: &WorldLocation, p: XZ<f32>, reference: f32) -> Option<f32> {
    world::map_details::nearby_decks(&world.details, p)
        .filter(|deck| deck.grade < 0)
        .filter_map(|deck| {
            world::map_details::height_on(deck, p)
                .filter(|h| (h - reference).abs() < 3.)
                .map(|h| h + 4.5)
        })
        .min_by(f32::total_cmp)
}

#[cfg(test)]
#[path = "tests/transport.rs"]
mod tests;
