//! Render-only chase-camera tuning. Never writes vehicle state.

use crate::geometry::{XYZ, XZ};
use crate::{track, vehicle::Vehicle};
use std::{
    f32::consts::{PI, TAU},
    time::Duration,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FollowView {
    Wide,
    Close,
    Bonnet,
}

impl FollowView {
    pub fn label(self) -> &'static str {
        match self {
            Self::Wide => "WIDE",
            Self::Close => "CLOSE",
            Self::Bonnet => "BONNET",
        }
    }

    fn dimensions(self) -> (f32, f32) {
        // Distances are from the car centre; close sits roughly 3.4 m behind the bumper.
        match self {
            Self::Wide => (9.0, 4.5),
            Self::Close => (5.5, 2.1),
            Self::Bonnet => (0.0, 0.0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub eye: XYZ,
    pub target: XYZ,
}

pub struct ChaseCamera {
    track: std::sync::Arc<track::Location>,
    view: FollowView,
    anchor: XYZ,
    heading: f32,
    look_ahead: XYZ,
    distance: f32,
    height: f32,
    bonnet: Pose,
}

impl ChaseCamera {
    pub fn new(car: &Vehicle) -> Self {
        let (distance, height) = FollowView::Close.dimensions();
        let mut camera = Self {
            track: car.track.clone(),
            view: FollowView::Close,
            anchor: car.position,
            heading: car.heading,
            look_ahead: XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            distance,
            height,
            bonnet: bonnet_pose(car),
        };
        camera.snap(car);
        camera
    }

    pub fn view(&self) -> FollowView {
        self.view
    }

    pub fn toggle_view(&mut self) {
        self.view = match self.view {
            FollowView::Wide => FollowView::Bonnet,
            FollowView::Bonnet => FollowView::Close,
            FollowView::Close => FollowView::Wide,
        };
    }

    pub fn snap(&mut self, car: &Vehicle) {
        self.bonnet = bonnet_pose(car);
        self.track = car.track.clone();
        self.anchor = car.position;
        self.heading = car.heading.rem_euclid(TAU);
        self.look_ahead = desired_look(car);
        (self.distance, self.height) = self.view.dimensions();
        self.distance += speed_pullback(car.speed);
    }

    pub fn update(&mut self, car: &Vehicle, frame_time: Duration) {
        if !std::sync::Arc::ptr_eq(&self.track, &car.track) {
            self.snap(car);
            return;
        }
        let dt = frame_time.as_secs_f32().min(0.25);
        self.bonnet = bonnet_pose(car);
        let position_blend = damp(7.0, dt);
        for axis in 0..3 {
            *self.anchor.component_mut(axis) +=
                (car.position.component(axis) - self.anchor.component(axis)) * position_blend;
        }

        // A little velocity alignment stabilizes slides without swinging away
        // from the nose or forcing the camera to follow the route instead.
        let motion = car.velocity.x.hypot(car.velocity.z);
        let drift = if motion > 1.0 {
            angle(car.velocity.x.atan2(car.velocity.z) - car.heading).clamp(-0.35, 0.35)
                * (car.speed.max(0.0) / 12.0).clamp(0.0, 1.0)
                * 0.25
        } else {
            0.0
        };
        let heading = car.heading + drift;
        self.heading =
            (self.heading + angle(heading - self.heading) * damp(7.0, dt)).rem_euclid(TAU);

        let look = desired_look(car);
        for (current, desired) in self.look_ahead.iter_mut().zip(look) {
            *current += (desired - *current) * damp(4.5, dt);
        }
        let (distance, height) = self.view.dimensions();
        self.distance += (distance + speed_pullback(car.speed) - self.distance) * damp(5.0, dt);
        self.height += (height - self.height) * damp(5.0, dt);
    }

    pub fn pose(&self) -> Pose {
        if self.view == FollowView::Bonnet {
            return self.bonnet;
        }
        let mut eye = XYZ {
            x: self.anchor.x - self.heading.sin() * self.distance,
            y: self.anchor.y + self.height,
            z: self.anchor.z - self.heading.cos() * self.distance,
        };
        // Conservative road-height clearance, not full scenery collision.
        eye.y = eye
            .y
            .max(self.track.height(XZ { x: eye.x, z: eye.z }) + 1.5);
        let target = XYZ::from_fn(|i| {
            self.anchor.component(i) + self.look_ahead.component(i) + if i == 1 { 0.4 } else { 0.0 }
        });
        if self.track.has_compiled_roads() {
            let city = &self.track;
            eye = city.camera_eye(
                XYZ {
                    x: self.anchor.x,
                    y: self.anchor.y + 0.5,
                    z: self.anchor.z,
                },
                eye,
            );
        }
        Pose {
            eye: eye,
            target: target,
        }
    }
}

// A rigid mount above the rear of the bonnet. Both the eye and sightline
// use the rendered body basis, including suspension pitch, roll and curb heave.
fn bonnet_pose(car: &Vehicle) -> Pose {
    let body = crate::gl::vehicle_body_matrix(car);
    let eye = XYZ::from_fn(|i| body[12 + i] + body[4 + i] * 0.65 + body[8 + i] * 0.95);
    Pose {
        eye: (eye),
        target: XYZ::from_fn(|i| eye.component(i) + (body[8 + i] - body[4 + i] * 0.08) * 30.0),
    }
}

fn desired_look(car: &Vehicle) -> XYZ {
    let distance = (car.speed.abs() * 0.22).min(5.5);
    if distance < 0.001 {
        return XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
    }
    // Look a little along actual reverse travel without flipping the chase rig.
    let direction = if car.speed < 0.0 { -1.0 } else { 1.0 };
    let forward = XZ {
        x: car.heading.sin() * direction,
        z: car.heading.cos() * direction,
    };
    XYZ {
        x: forward.x * distance,
        y: 0.0,
        z: forward.z * distance,
    }
}

fn speed_pullback(speed: f32) -> f32 {
    (speed.abs() * 0.03).min(0.8)
}
fn damp(rate: f32, seconds: f32) -> f32 {
    1.0 - (-rate * seconds).exp()
}
fn angle(value: f32) -> f32 {
    (value + PI).rem_euclid(TAU) - PI
}

impl track::Location {
    pub fn camera_eye(&self, anchor: XYZ, eye: XYZ) -> XYZ {
        let length = (eye.x - anchor.x).hypot(eye.z - anchor.z);
        let steps = (length / 0.35).ceil().max(1.0) as usize;
        let mut safe = anchor;
        for step in 1..=steps {
            let t = step as f32 / steps as f32;
            let p = XYZ::from_fn(|i| {
                anchor.component(i) + (eye.component(i) - anchor.component(i)) * t
            });
            let blocked = self.nearby_buildings(XZ { x: p.x, z: p.z }).any(|b| {
                p.y < b.base_y + b.height_meters + 0.5
                    && (b.contains(XZ { x: p.x, z: p.z })
                        || b.nearest(XZ { x: p.x, z: p.z }).2 < 0.5)
            });
            if blocked || p.y < self.height(XZ { x: p.x, z: p.z }) + 0.4 {
                return safe;
            }
            safe = p;
        }
        eye
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn car() -> Vehicle {
        let road = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(50.0);
        crate::test_support::vehicle(
            road.vehicle_position(0.0),
            road.heading(),
            crate::cars::CarModel::TestFixture(0),
        )
    }

    fn assert_close(a: crate::geometry::XYZ, b: crate::geometry::XYZ, tolerance: f32) {
        for i in 0..3 {
            assert!(
                (a.component(i) - b.component(i)).abs() < tolerance,
                "{a:?} != {b:?}"
            );
        }
    }

    #[test]
    fn changing_routes_snaps_camera_and_preserves_selected_view() {
        let mut camera = ChaseCamera::new(&car());
        camera.toggle_view();
        let track = crate::test_support::track();
        let road = track.at_distance(850.0);
        let mut scenic = Vehicle::on_track(
            road.vehicle_position(0.0),
            road.heading(),
            crate::cars::CarModel::TestFixture(0),
            track.clone(),
        );
        scenic.speed = 30.0;
        camera.update(&scenic, Duration::ZERO);
        assert!(std::sync::Arc::ptr_eq(&camera.track, &track));
        assert_eq!(camera.view(), FollowView::Wide);
        assert_eq!(camera.anchor, scenic.position);
        let mut expected = ChaseCamera::new(&scenic);
        expected.toggle_view();
        expected.snap(&scenic);
        assert_eq!(camera.pose(), expected.pose());
        let compact = car();
        camera.update(&compact, Duration::from_millis(16));
        assert!(std::sync::Arc::ptr_eq(&camera.track, &compact.track));
        assert_eq!(camera.anchor, compact.position);
        camera.snap(&scenic);
        assert!(std::sync::Arc::ptr_eq(&camera.track, &scenic.track));
    }

    #[test]
    fn smoothing_is_frame_rate_independent_for_a_constant_target() {
        let v = car();
        let mut moved = car();
        moved.position.x += 4.0;
        moved.heading += 0.3;
        moved.speed = 18.0;
        moved.velocity = crate::geometry::XZ {
            x: moved.heading.sin() * 18.0,
            z: moved.heading.cos() * 18.0,
        };
        let run = |fps: u32| {
            let mut c = ChaseCamera::new(&v);
            c.toggle_view();
            for _ in 0..fps {
                c.update(&moved, Duration::from_secs_f64(1.0 / fps as f64));
            }
            c.pose()
        };
        let a = run(30);
        for fps in [60, 144] {
            let b = run(fps);
            assert_close(a.eye, b.eye, 0.002);
            assert_close(a.target, b.target, 0.002);
        }
    }

    #[test]
    fn heading_wrap_uses_short_path_and_zero_dt_is_frozen() {
        let mut v = car();
        v.heading = TAU - 0.02;
        let mut c = ChaseCamera::new(&v);
        v.heading = 0.02;
        c.update(&v, Duration::from_millis(16));
        assert!(angle(c.heading).abs() < 0.025);
        let before = c.pose();
        v.position.x += 20.0;
        c.update(&v, Duration::ZERO);
        assert_eq!(before, c.pose());
    }

    #[test]
    fn both_views_stay_finite_and_above_road_on_a_complete_route() {
        let mut v = car();
        let mut c = ChaseCamera::new(&v);
        for close in [false, true] {
            if close {
                c.toggle_view();
            }
            for i in 0..600 {
                let road = track::LocationId::Compact.try_track().unwrap().at_distance(
                    i as f32
                        * track::LocationId::Compact
                            .try_track()
                            .unwrap()
                            .sampling_length()
                        / 600.0,
                );
                v.position = road.vehicle_position(1.4);
                v.heading = road.heading();
                v.speed = 20.0;
                v.velocity = road.tangent.map(|x| x * 20.0);
                c.update(&v, Duration::from_secs_f64(1.0 / 60.0));
                let pose = c.pose();
                assert!(pose
                    .eye
                    .iter()
                    .chain(pose.target.iter())
                    .all(|x| x.is_finite()));
                assert!(
                    pose.eye.y
                        >= track::LocationId::Compact.try_track().unwrap().height(
                            crate::geometry::XZ {
                                x: pose.eye.x,
                                z: pose.eye.z
                            }
                        ) + 1.499
                );
                let gap = (0..3)
                    .map(|i| (pose.eye.component(i) - pose.target.component(i)).powi(2))
                    .sum::<f32>();
                assert!(gap > 10.0);
            }
        }
    }
}
