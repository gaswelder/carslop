//! Render-only chase-camera tuning. Never writes vehicle state.

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
    pub eye: [f32; 3],
    pub target: [f32; 3],
}

pub struct ChaseCamera {
    track: std::sync::Arc<track::Track>,
    view: FollowView,
    anchor: [f32; 3],
    heading: f32,
    look_ahead: [f32; 3],
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
            look_ahead: [0.0; 3],
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
            self.anchor[axis] += (car.position[axis] - self.anchor[axis]) * position_blend;
        }

        // A little velocity alignment stabilizes slides without swinging away
        // from the nose or forcing the camera to follow the route instead.
        let motion = car.velocity[0].hypot(car.velocity[1]);
        let drift = if motion > 1.0 {
            angle(car.velocity[0].atan2(car.velocity[1]) - car.heading).clamp(-0.35, 0.35)
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
        let mut eye = [
            self.anchor[0] - self.heading.sin() * self.distance,
            self.anchor[1] + self.height,
            self.anchor[2] - self.heading.cos() * self.distance,
        ];
        // Conservative road-height clearance, not full scenery collision.
        eye[1] = eye[1].max(
            self.track.city().map_or_else(
                || self.track.project(eye).point[1],
                |c| c.height([eye[0], eye[2]]),
            ) + 1.5,
        );
        let target = std::array::from_fn(|i| {
            self.anchor[i] + self.look_ahead[i] + if i == 1 { 0.4 } else { 0.0 }
        });
        if let Some(city) = self.track.city() {
            eye = city.camera_eye([self.anchor[0], self.anchor[1] + 0.5, self.anchor[2]], eye);
        }
        Pose { eye, target }
    }
}

// A rigid mount above the rear of the bonnet. Both the eye and sightline
// use the rendered body basis, including suspension pitch, roll and curb heave.
fn bonnet_pose(car: &Vehicle) -> Pose {
    let body = crate::gl::vehicle_body_matrix(car);
    let eye = std::array::from_fn(|i| body[12 + i] + body[4 + i] * 0.65 + body[8 + i] * 0.95);
    Pose {
        eye,
        target: std::array::from_fn(|i| eye[i] + (body[8 + i] - body[4 + i] * 0.08) * 30.0),
    }
}

fn desired_look(car: &Vehicle) -> [f32; 3] {
    let distance = (car.speed.abs() * 0.22).min(5.5);
    if distance < 0.001 {
        return [0.0; 3];
    }
    // Look a little along actual reverse travel without flipping the chase rig.
    let direction = if car.speed < 0.0 { -1.0 } else { 1.0 };
    let forward = [car.heading.sin() * direction, car.heading.cos() * direction];
    if car.track.city().is_some() {
        return [forward[0] * distance, 0.0, forward[1] * distance];
    }
    let road = car.track.project(car.position);
    let alignment = forward[0] * road.tangent[0] + forward[1] * road.tangent[1];
    let future = car
        .track
        .at_distance(road.distance + distance * alignment.signum());
    let route_weight = alignment.abs() * 0.35;
    let mut look = [
        forward[0] * distance * (1.0 - route_weight)
            + (future.point[0] - road.point[0]) * route_weight,
        ((future.point[1] - road.point[1]) * route_weight).clamp(-0.5, 0.5),
        forward[1] * distance * (1.0 - route_weight)
            + (future.point[2] - road.point[2]) * route_weight,
    ];
    let length = look[0].hypot(look[2]);
    if length > 5.5 {
        look[0] *= 5.5 / length;
        look[2] *= 5.5 / length;
    }
    look
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cars::CarModel;

    fn car() -> Vehicle {
        let road = track::at_distance(50.0);
        Vehicle::new(road.lane_position(0.0), road.heading(), CarModel::Roadster)
    }

    fn assert_close(a: [f32; 3], b: [f32; 3], tolerance: f32) {
        for i in 0..3 {
            assert!((a[i] - b[i]).abs() < tolerance, "{a:?} != {b:?}");
        }
    }

    #[test]
    fn bonnet_cycles_and_stays_attached_during_forward_and_reverse_motion() {
        let mut v = car();
        let mut c = ChaseCamera::new(&v);
        c.toggle_view();
        assert_eq!(c.view(), FollowView::Wide);
        c.toggle_view();
        assert_eq!(c.view(), FollowView::Bonnet);
        assert_eq!(c.view().label(), "BONNET");
        for speed in [35.0, -5.0] {
            v.position[0] += 10.0;
            v.heading += 0.7;
            v.speed = speed;
            v.body_pitch = 0.06;
            v.body_roll = 0.04;
            v.curb_bump = 0.1;
            c.update(&v, Duration::from_millis(16));
            let pose = c.pose();
            let desired = bonnet_pose(&v);
            assert!((pose.eye[0] - desired.eye[0]).abs() < 0.001);
            assert!((pose.eye[2] - desired.eye[2]).abs() < 0.001);
            assert_eq!(pose, desired);
        }
        c.toggle_view();
        assert_eq!(c.view(), FollowView::Close);
    }

    #[test]
    fn bonnet_mount_is_rigid_through_heave_pitch_roll_and_zero_time_updates() {
        let mut v = car();
        let mut camera = ChaseCamera::new(&v);
        camera.toggle_view();
        camera.toggle_view();
        for dt in [
            Duration::ZERO,
            Duration::from_millis(16),
            Duration::from_millis(33),
        ] {
            v.position[1] += 0.3;
            v.heading += 0.3;
            v.body_pitch += 0.025;
            v.body_roll += 0.02;
            v.curb_bump = 0.1;
            camera.update(&v, dt);
            assert_eq!(camera.pose(), bonnet_pose(&v));
        }
    }

    #[test]
    fn changing_routes_snaps_camera_and_preserves_selected_view() {
        let mut camera = ChaseCamera::new(&car());
        camera.toggle_view();
        let track = track::TrackId::Scenic.track();
        let road = track.at_distance(850.0);
        let mut scenic = Vehicle::on_track(
            road.lane_position(0.0),
            road.heading(),
            CarModel::Roadster,
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
    fn scenic_camera_clearance_and_look_ahead_use_scenic_projection() {
        let track = track::TrackId::Scenic.track();
        let road = track.at_distance(0.0);
        let mut car = Vehicle::on_track(
            road.lane_position(0.0),
            road.heading(),
            CarModel::Roadster,
            track.clone(),
        );
        let mut camera = ChaseCamera::new(&car);
        for close in [false, true] {
            if close {
                camera.toggle_view();
            }
            for i in 0..100 {
                let road = track.at_distance(i as f32 * track.length() / 100.0);
                car.position = road.lane_position(1.4);
                car.heading = road.heading();
                car.speed = 30.0;
                car.velocity = road.tangent.map(|v| v * car.speed);
                camera.snap(&car);
                // Exercise clearance independently of the usual high chase rig.
                camera.height = -10.0;
                let pose = camera.pose();
                let clearance = track.project(pose.eye).point[1] + 1.5;
                assert!((pose.eye[1] - clearance).abs() < 0.001);
                assert!(pose
                    .eye
                    .iter()
                    .chain(pose.target.iter())
                    .all(|v| v.is_finite()));
                let look = desired_look(&car);
                assert!(look[0].hypot(look[2]) <= 5.501);
                assert!(look[1].abs() <= 0.5);
            }
        }
    }

    #[test]
    fn close_is_default_and_both_views_are_nearer_with_smooth_switching() {
        let mut v = car();
        let mut c = ChaseCamera::new(&v);
        assert_eq!(c.view(), FollowView::Close);
        assert_eq!(c.distance, 5.5);
        assert_eq!(c.height, 2.1);
        let before = c.pose();
        c.toggle_view();
        assert_eq!(c.pose(), before);
        for _ in 0..120 {
            c.update(&v, Duration::from_secs_f64(1.0 / 60.0));
        }
        assert!((c.distance - 9.0).abs() < 0.01 && (c.height - 4.5).abs() < 0.01);
        v.position = track::at_distance(120.0).lane_position(0.0);
        c.snap(&v);
        assert_eq!(c.view(), FollowView::Wide);
        assert_close(c.anchor, v.position, 0.001);
        c.toggle_view();
        c.toggle_view();
        c.snap(&v);
        assert_eq!(c.distance, 5.5);
        assert_eq!(c.height, 2.1);
    }

    #[test]
    fn smoothing_is_frame_rate_independent_for_a_constant_target() {
        let v = car();
        let mut moved = car();
        moved.position[0] += 4.0;
        moved.heading += 0.3;
        moved.speed = 18.0;
        moved.velocity = [moved.heading.sin() * 18.0, moved.heading.cos() * 18.0];
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
        v.position[0] += 20.0;
        c.update(&v, Duration::ZERO);
        assert_eq!(before, c.pose());
    }

    #[test]
    fn look_ahead_is_speed_based_bounded_and_stable_facing_backwards() {
        let mut v = car();
        assert_eq!(desired_look(&v), [0.0; 3]);
        for speed in [5.0, 15.0, 40.0] {
            v.speed = speed;
            for heading in [v.heading, v.heading + PI, v.heading + PI * 0.5] {
                v.heading = heading;
                let look = desired_look(&v);
                assert!(look.iter().all(|x| x.is_finite()));
                assert!(look[0].hypot(look[2]) <= 5.501);
                assert!(look[1].abs() <= 0.5);
                assert!(look[0] * heading.sin() + look[2] * heading.cos() > 0.0);
            }
        }
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
                let road = track::at_distance(i as f32 * track::length() / 600.0);
                v.position = road.lane_position(1.4);
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
                assert!(pose.eye[1] >= track::project(pose.eye).point[1] + 1.499);
                let gap = (0..3)
                    .map(|i| (pose.eye[i] - pose.target[i]).powi(2))
                    .sum::<f32>();
                assert!(gap > 10.0);
            }
        }
    }

    #[test]
    fn backing_looks_along_reverse_motion_without_turning_the_camera_around() {
        let mut v = car();
        v.speed = -4.0;
        v.velocity = [-v.heading.sin() * 4.0, -v.heading.cos() * 4.0];
        let mut c = ChaseCamera::new(&v);
        let look = desired_look(&v);
        assert!(look[0] * v.heading.sin() + look[2] * v.heading.cos() < 0.0);
        for _ in 0..120 {
            c.update(&v, Duration::from_secs_f64(1.0 / 60.0));
        }
        assert!(angle(c.heading - v.heading).abs() < 0.01);
        assert!(c
            .pose()
            .eye
            .iter()
            .chain(c.pose().target.iter())
            .all(|v| v.is_finite()));
    }
}
