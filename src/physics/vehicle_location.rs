//! Vehicle placement and collision response against location geometry.

use crate::model;
use crate::model::world::RoadSample;
use crate::model::world::WorldLocation;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

/// Vehicle origin height above the road's reference elevation, in metres.
pub const VEHICLE_ORIGIN_HEIGHT: f32 = 0.48;

/// Vehicle origin uses the asphalt's 5 cm reference. Sidewalks and other
/// surfaces retain their rendered heights rather than burying the wheels.
pub fn drive_height(world: &WorldLocation, p: XZ<f32>) -> f32 {
    if !world::street_surface::has_compiled_roads(world) {
        let height = world::elevation::world_height(world, p);
        if model::world::elevation(world).is_none() {
            return height;
        }
        let road = world::roads::project(
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
    let lift = world::street_surface::paving_height(world, p).unwrap_or_else(|| {
        if model::world::ground_material(world, p).is_some() {
            0.005
        } else {
            -0.02
        }
    });
    world::elevation::world_height(world, p) + lift - 0.05
        + world::raised_roads::road_rise(world, p)
}

fn clear_for_car(world: &WorldLocation, p: XYZ) -> bool {
    if world::street_surface::has_compiled_roads(world)
        && world::roads::road_clearance(world, p) > -world::locations::CAR_RADIUS - 0.15
    {
        return false;
    }
    world::buildings::building_clearance_at(
        world,
        util::geometry::at_height(util::geometry::xz(p), p.y + VEHICLE_ORIGIN_HEIGHT),
    ) > world::locations::CAR_RADIUS + 0.15
        && world::barriers::barrier_clearance(world, util::geometry::xz(p))
            > world::locations::CAR_RADIUS + 0.15
        && world::mapped_objects::mapped_object_clearance(world, util::geometry::xz(p))
            > world::locations::CAR_RADIUS + 0.15
}
pub fn recovery(world: &WorldLocation, p: XYZ) -> RoadSample {
    // Service roads may lead into garages. Never recover inside a footprint.
    world::roads::candidates(world, p)
        .flat_map(|i| {
            let r = &world::roads::road_lines(world)[i];
            let s = world::roads::projected(world, i, p).1;
            [-12.0, -6.0, -3.0, 0.0, 3.0, 6.0, 12.0]
                .into_iter()
                .map(move |offset| {
                    world::roads::sample(
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
                if model::world::map_details(world).decks.is_empty() {
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
    for (i, r) in world::roads::road_lines(world).iter().enumerate() {
        for t in [0.5, 0.25, 0.75] {
            let mut s = world::roads::sample(world, i, t);
            if clear_for_car(world, s.point) {
                if r.direction == -1 {
                    s.tangent = util::geometry::xz_map(s.tangent, |v| -v);
                }
                return s;
            }
        }
    }
    // A narrow authored road may be smaller than the car: its verges are
    // drivable too. Keep the spawn clear of buildings without requiring
    // every wheel to fit inside the asphalt polygon.
    if world::street_surface::has_compiled_roads(world) {
        for i in 0..world::roads::road_lines(world).len() {
            let mut sample = world::roads::sample(world, i, 0.5);
            if world::buildings::building_clearance(
                world,
                XZ {
                    x: sample.point.x,
                    z: sample.point.z,
                },
            ) > world::locations::CAR_RADIUS + 0.15
                && world::barriers::barrier_clearance(world, util::geometry::xz(sample.point))
                    > world::locations::CAR_RADIUS + 0.15
            {
                if world::roads::road_lines(world)[i].direction == -1 {
                    sample.tangent = util::geometry::xz_map(sample.tangent, |v| -v);
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
    if !world::street_surface::has_compiled_roads(world) {
        let best = world::roads::candidates(world, *position)
            .map(|i| {
                let (d, s) = world::roads::projected(world, i, *position);
                (d - (s.width * 0.5 - world::locations::CAR_RADIUS), d, s)
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
    for b in world::buildings::nearby_buildings(
        world,
        XZ {
            x: position.x,
            z: position.z,
        },
    ) {
        if position.y - VEHICLE_ORIGIN_HEIGHT + vehicle_height
            < world::elevation::world_height(world, util::geometry::xz(*position)) - 0.2
            || !overlaps_vehicle(b, *position, vehicle_height)
        {
            continue;
        }
        let p = XZ {
            x: position.x,
            z: position.z,
        };
        let (q, outward, distance) = world::buildings::nearest(b, p);
        let inside = world::buildings::contains(b, p);
        if inside || distance < world::locations::CAR_RADIUS {
            let n = if inside || distance < 0.0001 {
                outward
            } else {
                XZ {
                    x: (p.x - q.x) / distance,
                    z: (p.z - q.z) / distance,
                }
            };
            position.x = q.x + n.x * (world::locations::CAR_RADIUS + 0.01);
            position.z = q.z + n.z * (world::locations::CAR_RADIUS + 0.01);
            impact = impact.max(deflect(velocity, XZ { x: -n.x, z: -n.z }));
        }
    }
    impact = impact.max(world::barriers::constrain_barriers(
        world, position, velocity,
    ));
    impact = impact.max(world::mapped_objects::constrain_mapped_objects(
        world, position, velocity,
    ));
    impact = impact.max(world::transport::constrain_transport(
        world, position, velocity,
    ));
    // The saved extract is a finite driving area. Low curbs are crossable;
    // the crop boundary remains solid so the car cannot leave terrain data.
    for (axis, (world_axis, min, max)) in [
        (
            0,
            model::world::bounding_box(world).min.x,
            model::world::bounding_box(world).max.x,
        ),
        (
            2,
            model::world::bounding_box(world).min.z,
            model::world::bounding_box(world).max.z,
        ),
    ]
    .into_iter()
    .enumerate()
    {
        let value = util::geometry::xyz_component(*position, world_axis);
        *util::geometry::xyz_component_mut(position, world_axis) = value.clamp(
            min + world::locations::CAR_RADIUS,
            max - world::locations::CAR_RADIUS,
        );
        if util::geometry::xyz_component(*position, world_axis) != value {
            let mut normal = XZ { x: 0.0, z: 0.0 };
            *util::geometry::xz_component_mut(&mut normal, axis) =
                (value - util::geometry::xyz_component(*position, world_axis)).signum();
            impact = impact.max(deflect(velocity, normal));
        }
    }
    position.y = world::transport::drive_height_near(
        world,
        util::geometry::xz(*position),
        position.y - VEHICLE_ORIGIN_HEIGHT,
    ) + VEHICLE_ORIGIN_HEIGHT;
    impact
}

/// A crossing is an event, not a continuous wall force. Parallel travel or
/// resting alongside a curb must not repeatedly slow the car.
#[derive(Debug, PartialEq)]
pub struct CurbCrossing {
    pub normal: XZ<f32>,
    /// Upward change in support height, in metres.
    pub rise: f32,
}

pub fn crossed_curb(world: &WorldLocation, a: XZ<f32>, b: XZ<f32>) -> Option<CurbCrossing> {
    if world::street_surface::contains_road(world, a)
        == world::street_surface::contains_road(world, b)
    {
        return None;
    }
    let mid = XZ {
        x: (a.x + b.x) * 0.5,
        z: (a.z + b.z) * 0.5,
    };
    let (_, normal, distance) = world::street_surface::world_location_nearest_curb(world, mid);
    if distance > (b.x - a.x).hypot(b.z - a.z) * 0.5 + 0.01 {
        return None;
    }
    let rise = world::street_surface::paving_height(world, b).unwrap_or(0.005)
        - world::street_surface::paving_height(world, a).unwrap_or(0.005);
    (rise > 0.001).then_some(CurbCrossing { normal, rise })
}

/// Place the vehicle origin at a lateral offset from the road centreline.
pub fn vehicle_position(road_sample: RoadSample, offset: f32) -> XYZ {
    let mut point = model::world::lane_position(road_sample, offset);
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
            *util::geometry::xz_component_mut(velocity, i) =
                util::geometry::xz_component(tangent, i) * retained
                    - util::geometry::xz_component(normal, i) * outward * 0.08;
        }
        outward
    } else {
        0.0
    }
}

pub fn overlaps_vehicle(building: &model::world::Building, position: XYZ, height: f32) -> bool {
    let bottom = position.y - VEHICLE_ORIGIN_HEIGHT + 0.05;
    bottom <= building.base_y + building.height_meters
        && (building.min_height_meters == 0.0
            || bottom + height >= building.base_y + building.min_height_meters)
}

/// Ground rise per horizontal metre, excluding discrete curb impacts.
pub fn ground_gradient(world: &WorldLocation, position: XYZ) -> XZ<f32> {
    let height = |x, z| {
        world::transport::drive_height_near(world, XZ { x, z }, position.y - VEHICLE_ORIGIN_HEIGHT)
    };
    let XYZ { x, z, .. } = position;
    XZ {
        x: height(x + 0.5, z) - height(x - 0.5, z),
        z: height(x, z + 0.5) - height(x, z - 0.5),
    }
}
