//! Contact with the same mapped street objects used by the scene builder.

use crate::geometry;
use crate::geometry::{XYZ, XZ};
use crate::map_details;
use crate::map_details::{MapObject, ObjectKind};
use crate::track;
use crate::vehicle_location;
use crate::world::WorldLocation;
use crate::world_spatial;

pub fn mapped_object_clearance(world: &WorldLocation, p: XZ<f32>) -> f32 {
    map_details::nearby_objects(&world.details, p)
        .flat_map(capsules)
        .map(|capsule| {
            let q = world_spatial::nearest_edge(p, capsule.ends[0], capsule.ends[1]);
            (p.x - q.x).hypot(p.z - q.z) - capsule.radius
        })
        .fold(f32::INFINITY, f32::min)
}
pub fn constrain_mapped_objects(
    world: &WorldLocation,
    position: &mut XYZ,
    velocity: &mut XZ<f32>,
) -> f32 {
    let p = geometry::xz(*position);
    let mut impact: f32 = 0.;
    for object in map_details::nearby_objects(&world.details, p) {
        let ground = vehicle_location::drive_height(
            world,
            XZ {
                x: object.position[0],
                z: object.position[1],
            },
        );
        if position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT > ground + object.height {
            continue;
        }
        if position.y + 1.5 < ground {
            continue;
        }
        for Capsule { ends, radius } in capsules(object) {
            let q = world_spatial::nearest_edge(geometry::xz(*position), ends[0], ends[1]);
            let delta = XZ {
                x: position.x - q.x,
                z: position.z - q.z,
            };
            let distance = delta.x.hypot(delta.z);
            if distance < radius + track::CAR_RADIUS {
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
                position.x = q.x + n.x * (radius + track::CAR_RADIUS + 0.01);
                position.z = q.z + n.z * (radius + track::CAR_RADIUS + 0.01);
                impact = impact.max(vehicle_location::deflect(velocity, XZ { x: -n.x, z: -n.z }));
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

#[cfg(test)]
mod tests {
    use crate::world_mapped_objects;

    use super::*;
    use crate::map_details;
    use crate::test_support;
    #[test]
    fn open_gates_leave_the_driving_gap_clear_and_locked_gates_stop_a_car() {
        let mut city = test_support::empty_world();
        for locked in [false, true] {
            let mut data = map_details::Data::default();
            data.objects.push(MapObject {
                kind: ObjectKind::Gate,
                position: [0., 0.],
                angle: 0.,
                height: 1.4,
                width: 6.,
                text: String::new(),
                locked,
            });
            city.details = map_details::new(data);
            let mut p = XYZ {
                x: 0.,
                y: 0.48,
                z: 0.4,
            };
            let mut velocity = XZ { x: 0., z: -5. };
            let impact =
                world_mapped_objects::constrain_mapped_objects(&city, &mut p, &mut velocity);
            assert_eq!(impact > 0., locked);
            assert_eq!(
                world_mapped_objects::mapped_object_clearance(&(city), XZ { x: 0., z: 0. })
                    < track::CAR_RADIUS,
                locked
            );
            let mut above = XYZ {
                x: 0.,
                y: 5.48,
                z: 0.4,
            };
            let before = above;
            assert_eq!(
                world_mapped_objects::constrain_mapped_objects(&(city), &mut above, &mut velocity),
                0.
            );
            assert_eq!(above, before);
        }
    }
}
