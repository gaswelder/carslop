//! Fixed-step cosmetic tire trails. Bounded storage; no influence on vehicle physics.

use crate::physics;
use crate::physics::car::Vehicle;
use crate::render;
use crate::render::scene::Vertex;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::{collections::VecDeque, time::Duration};

const MAX_MARKS: usize = 2048;
const LIFETIME: f32 = 18.0;

struct Mark {
    from: XYZ,
    to: XYZ,
    strength: f32,
    age: f32,
    base_color: [f32; 3],
}

#[derive(Default)]
pub struct SkidMarks {
    marks: VecDeque<Mark>,
    previous: [Option<[XYZ; 2]>; 2],
}

pub fn advance(skid_marks: &mut SkidMarks, dt: Duration) {
    for mark in &mut skid_marks.marks {
        mark.age += dt.as_secs_f32();
    }
    while skid_marks.marks.front().is_some_and(|m| m.age >= LIFETIME) {
        skid_marks.marks.pop_front();
    }
}

pub fn break_trail(skid_marks: &mut SkidMarks, car: usize) {
    skid_marks.previous[car] = None;
}

pub fn record(skid_marks: &mut SkidMarks, index: usize, car: &Vehicle) {
    if car.tires.scrub < 0.15 {
        break_trail(skid_marks, index);
        return;
    }
    let centers = car.spec.design.mounts;
    let contacts: [(XYZ, bool); 2] = [centers[0], centers[1]].map(|p| {
        let (s, c) = car.orientation.yaw.sin_cos();
        let x = car.position.x + p.x * c + p.z * s;
        let z = car.position.z - p.x * s + p.z * c;
        let road = world::roads::project(&car.track, XYZ { x, y: 0.0, z });
        (
            XYZ {
                x,
                y: world::transport::drive_height_near(
                    &car.track,
                    XZ { x, z },
                    car.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                ) + 0.078,
                z,
            },
            (if world::street_surface::has_compiled_roads(&car.track) {
                let city = &car.track;
                world::transport::paved_near(
                    city,
                    XYZ {
                        x,
                        y: car.position.y,
                        z,
                    },
                )
            } else {
                road.offset.abs() < road.width * 0.5 - 0.15
            }),
        )
    });
    if !contacts.iter().all(|(_, on_road)| *on_road) {
        break_trail(skid_marks, index);
        return;
    }
    let contacts: [XYZ; 2] = contacts.map(|(p, _)| p);
    if let Some(previous) = skid_marks.previous[index] {
        let distance = |a: XYZ, b: XYZ| (a.x - b.x).hypot(a.z - b.z);
        if previous
            .iter()
            .zip(contacts)
            .any(|(a, b)| distance(*a, b) > 2.5)
        {
            skid_marks.previous[index] = Some(contacts);
            return; // never draw a stripe across recovery/teleport jumps
        }
        if previous
            .iter()
            .zip(contacts)
            .all(|(a, b)| distance(*a, b) < 0.15)
        {
            return;
        }
        for (from, to) in previous.into_iter().zip(contacts) {
            if distance(from, to) < 0.03 {
                continue;
            }
            if skid_marks.marks.len() == MAX_MARKS {
                skid_marks.marks.pop_front();
            }
            skid_marks.marks.push_back(Mark {
                from,
                to: (to),
                strength: car.tires.scrub,
                age: 0.0,
                base_color: render::scene::road_color(
                    &car.track,
                    XZ {
                        x: (from.x + to.x) * 0.5,
                        z: (from.z + to.z) * 0.5,
                    },
                ),
            });
        }
    }
    skid_marks.previous[index] = Some(contacts);
}

pub fn vertices(skid_marks: &SkidMarks) -> Vec<Vertex> {
    let mut vertices = Vec::with_capacity(skid_marks.marks.len() * 6);
    for m in &skid_marks.marks {
        let dx = m.to.x - m.from.x;
        let dz = m.to.z - m.from.z;
        let length = dx.hypot(dz).max(0.001);
        let n = XZ {
            x: -dz / length * 0.09,
            z: dx / length * 0.09,
        };
        let offset = |p: XYZ, side: f32| XYZ {
            x: p.x + n.x * side,
            y: p.y,
            z: p.z + n.z * side,
        };
        let points = [
            offset(m.from, -1.0),
            offset(m.from, 1.0),
            offset(m.to, 1.0),
            offset(m.to, -1.0),
        ];
        let fade = m.strength * (1.0 - m.age / LIFETIME).clamp(0.0, 1.0);
        let asphalt = m.base_color;
        let color = std::array::from_fn(|i| asphalt[i] * (1.0 - fade) + 0.045 * fade);
        // Match the asphalt's sun/ambient/fog response without alpha sorting.
        for i in [0, 1, 2, 0, 2, 3] {
            vertices.push(Vertex {
                position: (points[i]).into(),
                color,
                normal: [0.0, 1.0, 0.0],
                material: [-1.0, 0.0],
            });
        }
    }
    vertices
}
