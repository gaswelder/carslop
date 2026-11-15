//! Vehicle placement and collision response against location geometry.

use crate::geometry;
use crate::geometry::{XYZ, XZ};
use crate::street_surface;
use crate::track;
use crate::world;
use crate::world::{RoadSample, WorldLocation};
use crate::world_barriers;
use crate::world_buildings;
use crate::world_mapped_objects;
use crate::world_raised_roads;
use crate::world_roads;
use crate::world_transport;

/// Vehicle origin height above the road's reference elevation, in metres.
pub const VEHICLE_ORIGIN_HEIGHT: f32 = 0.48;

/// Vehicle origin uses the asphalt's 5 cm reference. Sidewalks and other
/// surfaces retain their rendered heights rather than burying the wheels.
pub fn drive_height(world: &WorldLocation, p: XZ<f32>) -> f32 {
    if !street_surface::has_compiled_roads(world) {
        let height = world::height(world, p);
        if world::elevation(world).is_none() {
            return height;
        }
        let road = world_roads::project(
            world,
            XYZ {
                x: p.x,
                y: 0.0,
                z: p.z,
            },
        );
        return height
            - if road.offset.abs() > road.width * 0.5 {
                0.07
            } else {
                0.0
            };
    }
    let lift = street_surface::paving_height(world, p).unwrap_or_else(|| {
        if world::ground_material(world, p).is_some() {
            0.005
        } else {
            -0.02
        }
    });
    world::height(world, p) + lift - 0.05 + world_raised_roads::road_rise(world, p)
}

fn clear_for_car(world: &WorldLocation, p: XYZ) -> bool {
    if street_surface::has_compiled_roads(world)
        && world::road_clearance(world, p) > -track::CAR_RADIUS - 0.15
    {
        return false;
    }
    world_buildings::building_clearance_at(
        world,
        geometry::at_height(geometry::xz(p), p.y + VEHICLE_ORIGIN_HEIGHT),
    ) > track::CAR_RADIUS + 0.15
        && world_barriers::barrier_clearance(world, geometry::xz(p)) > track::CAR_RADIUS + 0.15
        && world_mapped_objects::mapped_object_clearance(world, geometry::xz(p))
            > track::CAR_RADIUS + 0.15
}
pub fn recovery(world: &WorldLocation, p: XYZ) -> RoadSample {
    // Service roads may lead into garages. Never recover inside a footprint.
    world_roads::candidates(world, p)
        .flat_map(|i| {
            let r = &world_roads::road_lines(world)[i];
            let s = world_roads::projected(world, i, p).1;
            [-12.0, -6.0, -3.0, 0.0, 3.0, 6.0, 12.0]
                .into_iter()
                .map(move |offset| {
                    world_roads::sample(
                        world,
                        i,
                        ((s.distance - r.distance + offset) / r.length).clamp(0.0, 1.0),
                    )
                })
        })
        .filter(|s| clear_for_car(world, s.point))
        .min_by(|a, b| {
            let distance = |s: &RoadSample| {
                let horizontal = (s.point.x - p.x).hypot(s.point.z - p.z);
                if world::map_details(world).decks.is_empty() {
                    horizontal
                } else {
                    horizontal.hypot(s.point.y - (p.y - VEHICLE_ORIGIN_HEIGHT))
                }
            };
            distance(a).total_cmp(&distance(b))
        })
        .unwrap_or_else(|| spawn(world))
}

pub fn spawn(world: &WorldLocation) -> RoadSample {
    for (i, r) in world_roads::road_lines(world).iter().enumerate() {
        for t in [0.5, 0.25, 0.75] {
            let mut s = world_roads::sample(world, i, t);
            if clear_for_car(world, s.point) {
                if r.direction == -1 {
                    s.tangent = geometry::xz_map(s.tangent, |v| -v);
                }
                return s;
            }
        }
    }
    // A narrow authored road may be smaller than the car: its verges are
    // drivable too. Keep the spawn clear of buildings without requiring
    // every wheel to fit inside the asphalt polygon.
    if street_surface::has_compiled_roads(world) {
        for i in 0..world_roads::road_lines(world).len() {
            let mut sample = world_roads::sample(world, i, 0.5);
            if world_buildings::building_clearance(
                world,
                XZ {
                    x: sample.point.x,
                    z: sample.point.z,
                },
            ) > track::CAR_RADIUS + 0.15
                && world_barriers::barrier_clearance(world, geometry::xz(sample.point))
                    > track::CAR_RADIUS + 0.15
            {
                if world_roads::road_lines(world)[i].direction == -1 {
                    sample.tangent = geometry::xz_map(sample.tangent, |v| -v);
                }
                return sample;
            }
        }
    }
    panic!("city contains no safe spawn");
}
pub fn constrain(
    world: &WorldLocation,
    position: &mut XYZ,
    velocity: &mut XZ<f32>,
    vehicle_height: f32,
) -> f32 {
    let mut impact: f32 = 0.0;
    if !street_surface::has_compiled_roads(world) {
        let best = world_roads::candidates(world, *position)
            .map(|i| {
                let (d, s) = world_roads::projected(world, i, *position);
                (d - (s.width * 0.5 - track::CAR_RADIUS), d, s)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap();

        if best.0 > 0.0 && best.1 > 0.0001 {
            let n = XZ {
                x: (position.x - best.2.point.x) / best.1,
                z: (position.z - best.2.point.z) / best.1,
            };
            position.x -= n.x * best.0;
            position.z -= n.z * best.0;
            impact = deflect(velocity, n);
        }
    }
    // Check footprints, not bounding boxes: concave courtyards/streets remain open.
    for b in world_buildings::nearby_buildings(
        world,
        XZ {
            x: position.x,
            z: position.z,
        },
    ) {
        if position.y - VEHICLE_ORIGIN_HEIGHT + vehicle_height
            < world::height(world, geometry::xz(*position)) - 0.2
            || !world::overlaps_vehicle(b, *position, vehicle_height)
        {
            continue;
        }
        let p = XZ {
            x: position.x,
            z: position.z,
        };
        let (q, outward, distance) = world::nearest(b, p);
        let inside = world::contains(b, p);
        if inside || distance < track::CAR_RADIUS {
            let n = if inside || distance < 0.0001 {
                outward
            } else {
                XZ {
                    x: (p.x - q.x) / distance,
                    z: (p.z - q.z) / distance,
                }
            };
            position.x = q.x + n.x * (track::CAR_RADIUS + 0.01);
            position.z = q.z + n.z * (track::CAR_RADIUS + 0.01);
            impact = impact.max(deflect(velocity, XZ { x: -n.x, z: -n.z }));
        }
    }
    impact = impact.max(world_barriers::constrain_barriers(
        world, position, velocity,
    ));
    impact = impact.max(world_mapped_objects::constrain_mapped_objects(
        world, position, velocity,
    ));
    impact = impact.max(world_transport::constrain_transport(
        world, position, velocity,
    ));
    // The saved extract is a finite driving area. Low curbs are crossable;
    // the crop boundary remains solid so the car cannot leave terrain data.
    for (axis, (world_axis, min, max)) in [
        (
            0,
            world::bounding_box(world).min.x,
            world::bounding_box(world).max.x,
        ),
        (
            2,
            world::bounding_box(world).min.z,
            world::bounding_box(world).max.z,
        ),
    ]
    .into_iter()
    .enumerate()
    {
        let value = geometry::xyz_component(*position, world_axis);
        *geometry::xyz_component_mut(position, world_axis) =
            value.clamp(min + track::CAR_RADIUS, max - track::CAR_RADIUS);
        if geometry::xyz_component(*position, world_axis) != value {
            let mut normal = XZ { x: 0.0, z: 0.0 };
            *geometry::xz_component_mut(&mut normal, axis) =
                (value - geometry::xyz_component(*position, world_axis)).signum();
            impact = impact.max(deflect(velocity, normal));
        }
    }
    position.y = world_transport::drive_height_near(
        world,
        geometry::xz(*position),
        position.y - VEHICLE_ORIGIN_HEIGHT,
    ) + VEHICLE_ORIGIN_HEIGHT;
    impact
}

/// A crossing is an event, not a continuous wall force. Parallel travel or
/// resting alongside a curb must not repeatedly slow the car.
pub fn crossed_curb(world: &WorldLocation, a: XZ<f32>, b: XZ<f32>) -> Option<(XZ<f32>, f32)> {
    if street_surface::contains_road(world, a) == street_surface::contains_road(world, b) {
        return None;
    }
    let mid = XZ {
        x: (a.x + b.x) * 0.5,
        z: (a.z + b.z) * 0.5,
    };
    let (_, normal, distance) = street_surface::world_location_nearest_curb(world, mid);
    if distance > (b.x - a.x).hypot(b.z - a.z) * 0.5 + 0.01 {
        return None;
    }
    // Physical rise is small for service verges, standard for pavements.
    // The rendered support height determines the exact ride height.
    let rise = (street_surface::paving_height(world, a).unwrap_or(0.005)
        - street_surface::paving_height(world, b).unwrap_or(0.005))
    .abs()
    .max(0.03);
    // Cap the bump and speed penalty so taller pavements stay climbable.
    Some((normal, rise.min(0.09)))
}

/// Place the vehicle origin at a lateral offset from the road centreline.
pub fn vehicle_position(road_sample: RoadSample, offset: f32) -> XYZ {
    let mut point = world::lane_position(road_sample, offset);
    point.y += VEHICLE_ORIGIN_HEIGHT;
    point
}

pub fn deflect(velocity: &mut XZ<f32>, normal: XZ<f32>) -> f32 {
    let outward = velocity.x * normal.x + velocity.z * normal.z;
    if outward > 0.0 {
        // Friction is limited by the normal impact impulse. A fixed percentage
        // of total speed on every contact makes light, repeated scrapes stop cars.
        let tangent = XZ {
            x: velocity.x - normal.x * outward,
            z: velocity.z - normal.z * outward,
        };
        let speed = tangent.x.hypot(tangent.z);
        let retained = if speed > 0.0 {
            (1.0 - 0.06 * outward / speed).max(0.0)
        } else {
            0.0
        };
        for i in 0..2 {
            *geometry::xz_component_mut(velocity, i) = geometry::xz_component(tangent, i)
                * retained
                - geometry::xz_component(normal, i) * outward * 0.08;
        }
        outward
    } else {
        0.0
    }
}
