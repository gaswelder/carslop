//! Fixed-step cosmetic tire trails. Bounded storage; no influence on vehicle physics.

use crate::geometry::{XYZ, XZ};
#[cfg(test)]
use crate::track;
use crate::{scene::Vertex, vehicle::Vehicle};
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

impl SkidMarks {
    pub fn advance(&mut self, dt: Duration) {
        for mark in &mut self.marks {
            mark.age += dt.as_secs_f32();
        }
        while self.marks.front().is_some_and(|m| m.age >= LIFETIME) {
            self.marks.pop_front();
        }
    }

    pub fn break_trail(&mut self, car: usize) {
        self.previous[car] = None;
    }

    pub fn record(&mut self, index: usize, car: &Vehicle) {
        if car.tire_scrub < 0.15 {
            self.break_trail(index);
            return;
        }
        let centers = car.model.wheel_centers();
        let contacts: [(XYZ, bool); 2] = [centers[0], centers[1]].map(|p| {
            let (s, c) = car.heading.sin_cos();
            let x = car.position.x + p.x * c + p.z * s;
            let z = car.position.z - p.x * s + p.z * c;
            let road = car.track.project(XYZ { x, y: 0.0, z });
            (
                XYZ {
                    x,
                    y: car.track.drive_height(XZ { x, z }) + 0.078,
                    z,
                },
                (if car.track.has_compiled_roads() {
                    let city = &car.track;
                    city.paved(XZ { x, z })
                } else {
                    road.offset.abs() < road.width * 0.5 - 0.15
                }),
            )
        });
        if !contacts.iter().all(|(_, on_road)| *on_road) {
            self.break_trail(index);
            return;
        }
        let contacts: [XYZ; 2] = contacts.map(|(p, _)| p);
        if let Some(previous) = self.previous[index] {
            let distance = |a: XYZ, b: XYZ| (a.x - b.x).hypot(a.z - b.z);
            if previous
                .iter()
                .zip(contacts)
                .any(|(a, b)| distance(*a, b) > 2.5)
            {
                self.previous[index] = Some(contacts);
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
                if self.marks.len() == MAX_MARKS {
                    self.marks.pop_front();
                }
                self.marks.push_back(Mark {
                    from,
                    to: (to),
                    strength: car.tire_scrub,
                    age: 0.0,
                    base_color: car.track.road_color(XZ {
                        x: (from.x + to.x) * 0.5,
                        z: (from.z + to.z) * 0.5,
                    }),
                });
            }
        }
        self.previous[index] = Some(contacts);
    }

    pub fn vertices(&self) -> Vec<Vertex> {
        let mut vertices = Vec::with_capacity(self.marks.len() * 6);
        for m in &self.marks {
            let dx = m.to.x - m.from.x;
            let dz = m.to.z - m.from.z;
            let length = dx.hypot(dz).max(0.001);
            let n = XZ {
                x: -dz / length * 0.09,
                z: dx / length * 0.09,
            };
            let offset = |p: XYZ, side: f32| crate::geometry::XYZ {
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cars::CarModel;

    fn sliding(model: CarModel, distance: f32) -> Vehicle {
        let road = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(distance);
        let mut car =
            crate::test_support::vehicle(road.vehicle_position(0.0), road.heading(), model);
        car.tire_scrub = 0.9;
        car
    }

    #[test]
    fn authored_trails_use_road_width_and_height() {
        let model = crate::cars::CarModel::TestFixture(0);
        let file = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(file.path(), r#"{"version":2,"name":"Trail test","points":[[0,3,0,10],[400,6,0,10],[400,5,400,10],[0,3,400,10]]}"#).unwrap();
        let track = track::register_file(file.path())
            .unwrap()
            .0
            .try_track()
            .unwrap();
        let half_track = model.wheel_centers()[..2]
            .iter()
            .map(|p| p.x.abs())
            .fold(0.0, f32::max);
        let lane_offset = track.at_distance(50.0).width * 0.5 - half_track - 0.5;
        let mut marks = SkidMarks::default();
        for distance in [50.0, 50.5] {
            let road = track.at_distance(distance);
            let mut car = Vehicle::on_track(
                road.vehicle_position(lane_offset),
                road.heading(),
                model,
                track.clone(),
            );
            car.tire_scrub = 0.9;
            marks.record(0, &car);
        }
        assert_eq!(
            marks.marks.len(),
            2,
            "both rear tires fit the authored road"
        );
        for vertex in marks.vertices() {
            let road = track.project((vertex.position).into());
            assert!((vertex.position[1] - road.point.y - 0.078).abs() < 0.1);
            assert!(road.offset.abs() < road.width * 0.5);
        }
    }

    #[test]
    fn trails_follow_road_fade_and_expire() {
        let model = crate::cars::CarModel::TestFixture(0);
        let mut marks = SkidMarks::default();
        marks.record(0, &sliding(model, 50.0));
        marks.record(0, &sliding(model, 50.5));
        assert_eq!(marks.marks.len(), 2);
        let before = marks.vertices();
        assert_eq!(before.len(), 12);
        for v in &before {
            assert!(v.position.iter().all(|x| x.is_finite()));
            assert!(
                (v.position[1]
                    - track::LocationId::Compact
                        .try_track()
                        .unwrap()
                        .project((v.position).into())
                        .point
                        .y
                    - 0.078)
                    .abs()
                    < 0.1
            );
        }
        marks.advance(Duration::from_secs(9));
        assert!(marks.vertices()[0].color[0] > before[0].color[0]);
        marks.advance(Duration::from_secs(10));
        assert!(marks.vertices().is_empty());
    }

    #[test]
    fn trails_never_bridge_recovery_or_grip_changes() {
        let model = crate::cars::CarModel::TestFixture(0);
        let mut marks = SkidMarks::default();
        marks.record(0, &sliding(model, 50.0));
        marks.record(0, &sliding(model, 100.0));
        assert!(marks.marks.is_empty());
        marks.break_trail(0);
        marks.record(0, &sliding(model, 100.5));
        assert!(marks.marks.is_empty());
        let mut car = sliding(model, 101.0);
        car.tire_scrub = 0.0;
        marks.record(0, &car);
        marks.record(0, &sliding(model, 101.5));
        assert!(marks.marks.is_empty());
    }

    #[test]
    fn trail_storage_is_bounded_and_stationary_cars_leave_nothing() {
        let model = crate::cars::CarModel::TestFixture(0);
        let mut marks = SkidMarks::default();
        for _ in 0..100 {
            marks.record(0, &sliding(model, 50.0));
        }
        assert!(marks.marks.is_empty());
        for i in 0..3000 {
            marks.record(0, &sliding(model, 50.0 + i as f32 * 0.25));
        }
        assert!(!marks.marks.is_empty());
        assert!(marks.marks.len() <= MAX_MARKS);
        assert!(marks.vertices().len() <= MAX_MARKS * 6);
    }
}
