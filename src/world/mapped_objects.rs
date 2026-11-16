//! Contact with the same mapped street objects used by the scene builder.

use crate::model::map_details::MapObject;
use crate::model::map_details::ObjectKind;
use crate::model::world::WorldLocation;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

pub fn mapped_object_clearance(world: &WorldLocation, p: XZ<f32>) -> f32 {
    world::map_details::nearby_objects(&world.details, p)
        .flat_map(capsules)
        .map(|capsule| {
            let q = world::spatial::nearest_edge(p, capsule.ends[0], capsule.ends[1]);
            (p.x - q.x).hypot(p.z - q.z) - capsule.radius
        })
        .fold(f32::INFINITY, f32::min)
}
pub fn constrain_mapped_objects(
    world: &WorldLocation,
    position: &mut XYZ,
    velocity: &mut XZ<f32>,
) -> f32 {
    let p = util::geometry::xz(*position);
    let mut impact: f32 = 0.;
    for object in world::map_details::nearby_objects(&world.details, p) {
        let ground = physics::vehicle_location::drive_height(
            world,
            XZ {
                x: object.position[0],
                z: object.position[1],
            },
        );
        if position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT > ground + object.height {
            continue;
        }
        if position.y + 1.5 < ground {
            continue;
        }
        for Capsule { ends, radius } in capsules(object) {
            let q = world::spatial::nearest_edge(util::geometry::xz(*position), ends[0], ends[1]);
            let delta = XZ {
                x: position.x - q.x,
                z: position.z - q.z,
            };
            let distance = delta.x.hypot(delta.z);
            if distance < radius + world::locations::CAR_RADIUS {
                let n = if distance > 0.0001 {
                    XZ {
                        x: delta.x / distance,
                        z: delta.z / distance,
                    }
                } else {
                    XZ {
                        x: -object.angle.sin(),
                        z: object.angle.cos(),
                    }
                };
                position.x = q.x + n.x * (radius + world::locations::CAR_RADIUS + 0.01);
                position.z = q.z + n.z * (radius + world::locations::CAR_RADIUS + 0.01);
                impact = impact.max(physics::vehicle_location::deflect(
                    velocity,
                    XZ { x: -n.x, z: -n.z },
                ));
            }
        }
    }
    impact
}

struct Capsule {
    ends: [XZ<f32>; 2],
    radius: f32,
}
fn capsules(object: &MapObject) -> Vec<Capsule> {
    let (sin, cos) = object.angle.sin_cos();
    let transform = |x: f32, z: f32| XZ {
        x: object.position[0] + cos * x - sin * z,
        z: object.position[1] + sin * x + cos * z,
    };
    let mut result = Vec::new();
    match object.kind {
        ObjectKind::Gate => {
            for side in [-1., 1.] {
                result.push(Capsule {
                    ends: [transform(side * object.width / 2., 0.); 2],
                    radius: 0.08,
                });
            }
            if object.locked {
                result.push(Capsule {
                    ends: [
                        transform(-object.width / 2., 0.),
                        transform(object.width / 2., 0.),
                    ],
                    radius: 0.06,
                });
            } else {
                result.push(Capsule {
                    ends: [
                        transform(-object.width / 2., 0.),
                        transform(-object.width / 2., object.width * 0.9),
                    ],
                    radius: 0.06,
                });
            }
        }
        ObjectKind::Bench => result.push(Capsule {
            ends: [
                transform(-object.width / 2., 0.),
                transform(object.width / 2., 0.),
            ],
            radius: 0.25,
        }),
        ObjectKind::Shelter => {
            result.push(Capsule {
                ends: [
                    transform(-object.width / 2., 0.7),
                    transform(object.width / 2., 0.7),
                ],
                radius: 0.08,
            });
            for side in [-1., 1.] {
                result.push(Capsule {
                    ends: [
                        transform(side * object.width / 2., -0.7),
                        transform(side * object.width / 2., 0.7),
                    ],
                    radius: 0.06,
                });
            }
        }
        ObjectKind::PowerTower => {
            for x in [-1., 1.] {
                for z in [-1., 1.] {
                    result.push(Capsule {
                        ends: [transform(x * object.width / 2., z * object.width / 2.); 2],
                        radius: 0.12,
                    });
                }
            }
        }
        ObjectKind::TurnSign => result.push(Capsule {
            ends: [transform(0., 0.); 2],
            radius: 0.04,
        }),
        _ => result.push(Capsule {
            ends: [transform(0., 0.); 2],
            radius: object.width / 2.,
        }),
    }
    result
}
