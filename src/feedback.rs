//! Fixed-step cosmetic tire trails. Bounded storage; no influence on vehicle physics.

#[cfg(test)]
use crate::track;
use crate::{scene::Vertex, vehicle::Vehicle};
use std::{collections::VecDeque, time::Duration};

const MAX_MARKS: usize = 2048;
const LIFETIME: f32 = 18.0;

struct Mark {
    from: [f32; 3],
    to: [f32; 3],
    strength: f32,
    age: f32,
    base_color: [f32; 3],
}

#[derive(Default)]
pub struct SkidMarks {
    marks: VecDeque<Mark>,
    previous: [Option<[[f32; 3]; 2]>; 2],
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
        let contacts = [centers[0], centers[1]].map(|p| {
            let (s, c) = car.heading.sin_cos();
            let x = car.position[0] + p[0] * c + p[2] * s;
            let z = car.position[2] - p[0] * s + p[2] * c;
            let road = car.track.project([x, 0.0, z]);
            (
                [
                    x,
                    car.track
                        .city()
                        .map_or(road.point[1], |c| c.drive_height([x, z]))
                        + 0.078,
                    z,
                ],
                (if let Some(city) = car.track.city() {
                    city.paved([x, z])
                } else {
                    road.offset.abs() < road.width * 0.5 - 0.15
                }),
            )
        });
        if !contacts.iter().all(|(_, on_road)| *on_road) {
            self.break_trail(index);
            return;
        }
        let contacts = contacts.map(|(p, _)| p);
        if let Some(previous) = self.previous[index] {
            let distance = |a: [f32; 3], b: [f32; 3]| (a[0] - b[0]).hypot(a[2] - b[2]);
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
                    to,
                    strength: car.tire_scrub,
                    age: 0.0,
                    base_color: car
                        .track
                        .city()
                        .and_then(|city| city.surfaces.as_ref())
                        .map_or([0.23, 0.25, 0.26], |surface| {
                            surface.road_color([(from[0] + to[0]) * 0.5, (from[2] + to[2]) * 0.5])
                        }),
                });
            }
        }
        self.previous[index] = Some(contacts);
    }

    pub fn vertices(&self) -> Vec<Vertex> {
        let mut vertices = Vec::with_capacity(self.marks.len() * 6);
        for m in &self.marks {
            let dx = m.to[0] - m.from[0];
            let dz = m.to[2] - m.from[2];
            let length = dx.hypot(dz).max(0.001);
            let n = [-dz / length * 0.09, dx / length * 0.09];
            let offset = |p: [f32; 3], side: f32| [p[0] + n[0] * side, p[1], p[2] + n[1] * side];
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
                    position: points[i],
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

    fn sliding(distance: f32) -> Vehicle {
        let road = track::TrackId::Compact.track().at_distance(distance);
        let mut car = Vehicle::new(road.lane_position(0.0), road.heading(), CarModel::Roadster);
        car.tire_scrub = 0.9;
        car
    }

    #[test]
    fn scenic_trails_use_scenic_width_and_road_height() {
        let track = track::TrackId::Scenic.track();
        let mut marks = SkidMarks::default();
        for distance in [50.0, 50.5] {
            let road = track.at_distance(distance);
            let mut car = Vehicle::on_track(
                road.lane_position(2.55),
                road.heading(),
                CarModel::Roadster,
                track.clone(),
            );
            car.tire_scrub = 0.9;
            marks.record(0, &car);
        }
        assert_eq!(
            marks.marks.len(),
            2,
            "both rear tires fit the wider scenic road"
        );
        for vertex in marks.vertices() {
            let road = track.project(vertex.position);
            assert!((vertex.position[1] - road.point[1] - 0.078).abs() < 0.1);
            assert!(road.offset.abs() < track.width * 0.5);
        }
    }

    #[test]
    fn trails_follow_road_fade_and_expire() {
        let mut marks = SkidMarks::default();
        marks.record(0, &sliding(50.0));
        marks.record(0, &sliding(50.5));
        assert_eq!(marks.marks.len(), 2);
        let before = marks.vertices();
        assert_eq!(before.len(), 12);
        for v in &before {
            assert!(v.position.iter().all(|x| x.is_finite()));
            assert!(
                (v.position[1]
                    - track::TrackId::Compact.track().project(v.position).point[1]
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
        let mut marks = SkidMarks::default();
        marks.record(0, &sliding(50.0));
        marks.record(0, &sliding(100.0));
        assert!(marks.marks.is_empty());
        marks.break_trail(0);
        marks.record(0, &sliding(100.5));
        assert!(marks.marks.is_empty());
        let mut car = sliding(101.0);
        car.tire_scrub = 0.0;
        marks.record(0, &car);
        marks.record(0, &sliding(101.5));
        assert!(marks.marks.is_empty());
    }

    #[test]
    fn trail_storage_is_bounded_and_stationary_cars_leave_nothing() {
        let mut marks = SkidMarks::default();
        for _ in 0..100 {
            marks.record(0, &sliding(50.0));
        }
        assert!(marks.marks.is_empty());
        for i in 0..3000 {
            marks.record(0, &sliding(50.0 + i as f32 * 0.25));
        }
        assert!(!marks.marks.is_empty());
        assert!(marks.marks.len() <= MAX_MARKS);
        assert!(marks.vertices().len() <= MAX_MARKS * 6);
    }
}
