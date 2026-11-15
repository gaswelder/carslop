//! Render-only chase-camera tuning. Never writes vehicle state.

use crate::physics::car::Vehicle;
use crate::render;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
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

fn dimensions(follow_view: FollowView) -> (f32, f32) {
    // Distances are from the car centre; close sits roughly 3.4 m behind the bumper.
    match follow_view {
        FollowView::Wide => (9.0, 4.5),
        FollowView::Close => (5.5, 2.1),
        FollowView::Bonnet => (0.0, 0.0),
    }
}

pub fn view_name(follow_view: FollowView) -> &'static str {
    match follow_view {
        FollowView::Wide => "wide",
        FollowView::Close => "close",
        FollowView::Bonnet => "bonnet",
    }
}

pub fn from_name(name: &str) -> Option<FollowView> {
    [FollowView::Wide, FollowView::Close, FollowView::Bonnet]
        .into_iter()
        .find(|view| view_name(*view).eq_ignore_ascii_case(name))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub eye: XYZ,
    pub target: XYZ,
}

pub struct ChaseCamera {
    track: std::sync::Arc<world::locations::WorldLocation>,
    view: FollowView,
    anchor: XYZ,
    heading: f32,
    look_ahead: XYZ,
    distance: f32,
    height: f32,
    bonnet: Pose,
}

pub fn new(car: &Vehicle) -> ChaseCamera {
    let (distance, height) = dimensions(FollowView::Close);
    let mut camera = ChaseCamera {
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
    snap(&mut camera, car);
    camera
}

pub fn view(chase_camera: &ChaseCamera) -> FollowView {
    chase_camera.view
}

pub fn set_view(chase_camera: &mut ChaseCamera, view: FollowView) {
    chase_camera.view = view;
}

pub fn toggle_view(chase_camera: &mut ChaseCamera) {
    chase_camera.view = match chase_camera.view {
        FollowView::Wide => FollowView::Bonnet,
        FollowView::Bonnet => FollowView::Close,
        FollowView::Close => FollowView::Wide,
    };
}

pub fn snap(chase_camera: &mut ChaseCamera, car: &Vehicle) {
    chase_camera.bonnet = bonnet_pose(car);
    chase_camera.track = car.track.clone();
    chase_camera.anchor = car.position;
    chase_camera.heading = car.heading.rem_euclid(TAU);
    chase_camera.look_ahead = desired_look(car);
    (chase_camera.distance, chase_camera.height) = dimensions(chase_camera.view);
    chase_camera.distance += speed_pullback(car.speed);
}

pub fn update(chase_camera: &mut ChaseCamera, car: &Vehicle, frame_time: Duration) {
    if !std::sync::Arc::ptr_eq(&chase_camera.track, &car.track) {
        snap(chase_camera, car);
        return;
    }
    let dt = frame_time.as_secs_f32().min(0.25);
    chase_camera.bonnet = bonnet_pose(car);
    let position_blend = damp(7.0, dt);
    for axis in 0..3 {
        *util::geometry::xyz_component_mut(&mut chase_camera.anchor, axis) +=
            (util::geometry::xyz_component(car.position, axis)
                - util::geometry::xyz_component(chase_camera.anchor, axis))
                * position_blend;
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
    chase_camera.heading = (chase_camera.heading
        + angle(heading - chase_camera.heading) * damp(7.0, dt))
    .rem_euclid(TAU);

    let look = desired_look(car);
    for (current, desired) in util::geometry::xyz_iter_mut(&mut chase_camera.look_ahead).zip(look) {
        *current += (desired - *current) * damp(4.5, dt);
    }
    let (distance, height) = dimensions(chase_camera.view);
    chase_camera.distance +=
        (distance + speed_pullback(car.speed) - chase_camera.distance) * damp(5.0, dt);
    chase_camera.height += (height - chase_camera.height) * damp(5.0, dt);
}

pub fn pose(chase_camera: &ChaseCamera) -> Pose {
    if chase_camera.view == FollowView::Bonnet {
        return chase_camera.bonnet;
    }
    let mut eye = XYZ {
        x: chase_camera.anchor.x - chase_camera.heading.sin() * chase_camera.distance,
        y: chase_camera.anchor.y + chase_camera.height,
        z: chase_camera.anchor.z - chase_camera.heading.cos() * chase_camera.distance,
    };
    // Conservative road-height clearance, not full scenery collision.
    eye.y = eye.y.max(
        world::transport::drive_height_near(
            &chase_camera.track,
            util::geometry::xz(eye),
            chase_camera.anchor.y - 0.5,
        ) + 1.5,
    );
    let target = util::geometry::xyz_from_fn(|i| {
        util::geometry::xyz_component(chase_camera.anchor, i)
            + util::geometry::xyz_component(chase_camera.look_ahead, i)
            + if i == 1 { 0.4 } else { 0.0 }
    });
    if world::street_surface::has_compiled_roads(&chase_camera.track) {
        let city = &chase_camera.track;
        eye = camera_eye(
            city,
            XYZ {
                x: chase_camera.anchor.x,
                y: chase_camera.anchor.y + 0.5,
                z: chase_camera.anchor.z,
            },
            eye,
        );
    }
    Pose {
        eye: eye,
        target: target,
    }
}

fn bonnet_pose(car: &Vehicle) -> Pose {
    let body = render::gl::vehicle_body_matrix(car);
    let eye =
        util::geometry::xyz_from_fn(|i| body[12 + i] + body[4 + i] * 0.85 + body[8 + i] * 0.95);
    Pose {
        eye: (eye),
        target: util::geometry::xyz_from_fn(|i| {
            util::geometry::xyz_component(eye, i) + (body[8 + i] - body[4 + i] * 0.08) * 30.0
        }),
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

pub fn camera_eye(world: &world::locations::WorldLocation, anchor: XYZ, eye: XYZ) -> XYZ {
    let length = (eye.x - anchor.x).hypot(eye.z - anchor.z);
    let steps = (length / 0.35).ceil().max(1.0) as usize;
    let mut safe = anchor;
    for step in 1..=steps {
        let t = step as f32 / steps as f32;
        let p = util::geometry::xyz_from_fn(|i| {
            util::geometry::xyz_component(anchor, i)
                + (util::geometry::xyz_component(eye, i) - util::geometry::xyz_component(anchor, i))
                    * t
        });
        let blocked = world::buildings::nearby_buildings(world, XZ { x: p.x, z: p.z }).any(|b| {
            p.y >= world::elevation::world_height(world, util::geometry::xz(p)) - 0.5
                && p.y < b.base_y + b.height_meters + 0.5
                && (b.min_height_meters == 0.0 || p.y >= b.base_y + b.min_height_meters - 0.5)
                && (world::buildings::contains(b, XZ { x: p.x, z: p.z })
                    || world::buildings::nearest(b, XZ { x: p.x, z: p.z }).2 < 0.5)
        });
        if blocked
            || p.y
                < world::transport::drive_height_near(world, util::geometry::xz(p), anchor.y - 0.5)
                    + 0.4
            || world::transport::tunnel_ceiling(world, util::geometry::xz(p), anchor.y - 0.5)
                .is_some_and(|ceiling| p.y > ceiling - 0.3)
        {
            return safe;
        }
        safe = p;
    }
    eye
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model;
    use crate::physics;
    use crate::render;
    use crate::test;
    use crate::util;

    fn car() -> Vehicle {
        let road = world::locations::at_distance(&test::support::compact_track(), 50.0);
        test::support::vehicle(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            test::support::car(),
        )
    }

    #[test]
    fn view_names_round_trip() {
        for view in [FollowView::Wide, FollowView::Close, FollowView::Bonnet] {
            assert_eq!(
                render::camera::from_name(render::camera::view_name(view)),
                Some(view)
            );
        }
        assert_eq!(render::camera::from_name("WIDE"), Some(FollowView::Wide));
        assert_eq!(render::camera::from_name("nope"), None);
    }

    fn assert_close(a: XYZ, b: XYZ, tolerance: f32) {
        for i in 0..3 {
            assert!(
                (util::geometry::xyz_component(a, i) - util::geometry::xyz_component(b, i)).abs()
                    < tolerance,
                "{a:?} != {b:?}"
            );
        }
    }

    #[test]
    fn changing_routes_snaps_camera_and_preserves_selected_view() {
        let mut camera = render::camera::new(&car());
        render::camera::toggle_view(&mut camera);
        let track = test::support::track();
        let road = world::locations::at_distance(&track, 850.0);
        let mut scenic = physics::car::on_track(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            test::support::car(),
            track.clone(),
        );
        scenic.speed = 30.0;
        render::camera::update(&mut camera, &scenic, Duration::ZERO);
        assert!(std::sync::Arc::ptr_eq(&camera.track, &track));
        assert_eq!(render::camera::view(&(camera)), FollowView::Wide);
        assert_eq!(camera.anchor, scenic.position);
        let mut expected = render::camera::new(&scenic);
        render::camera::toggle_view(&mut expected);
        render::camera::snap(&mut expected, &scenic);
        assert_eq!(
            render::camera::pose(&(camera)),
            render::camera::pose(&(expected))
        );
        let compact = car();
        render::camera::update(&mut camera, &compact, Duration::from_millis(16));
        assert!(std::sync::Arc::ptr_eq(&camera.track, &compact.track));
        assert_eq!(camera.anchor, compact.position);
        render::camera::snap(&mut camera, &scenic);
        assert!(std::sync::Arc::ptr_eq(&camera.track, &scenic.track));
    }

    #[test]
    fn smoothing_is_frame_rate_independent_for_a_constant_target() {
        let v = car();
        let mut moved = car();
        moved.position.x += 4.0;
        moved.heading += 0.3;
        moved.speed = 18.0;
        moved.velocity = XZ {
            x: moved.heading.sin() * 18.0,
            z: moved.heading.cos() * 18.0,
        };
        let run = |fps: u32| {
            let mut c = render::camera::new(&v);
            render::camera::toggle_view(&mut c);
            for _ in 0..fps {
                render::camera::update(&mut c, &moved, Duration::from_secs_f64(1.0 / fps as f64));
            }
            render::camera::pose(&c)
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
        let mut c = render::camera::new(&v);
        v.heading = 0.02;
        render::camera::update(&mut c, &v, Duration::from_millis(16));
        assert!(angle(c.heading).abs() < 0.025);
        let before = render::camera::pose(&c);
        v.position.x += 20.0;
        render::camera::update(&mut c, &v, Duration::ZERO);
        assert_eq!(before, render::camera::pose(&(c)));
    }

    #[test]
    fn both_views_stay_finite_and_above_road_on_a_complete_route() {
        let mut v = car();
        let mut c = render::camera::new(&v);
        for close in [false, true] {
            if close {
                render::camera::toggle_view(&mut c);
            }
            for i in 0..600 {
                let road = world::locations::at_distance(
                    &test::support::compact_track(),
                    i as f32 * world::locations::sampling_length(&test::support::compact_track())
                        / 600.0,
                );
                v.position = physics::vehicle_location::vehicle_position(road, 1.4);
                v.heading = model::world::heading(road);
                v.speed = 20.0;
                v.velocity = util::geometry::xz_map(road.tangent, |x| x * 20.0);
                render::camera::update(&mut c, &v, Duration::from_secs_f64(1.0 / 60.0));
                let pose = render::camera::pose(&c);
                assert!(util::geometry::iter(&(pose.eye))
                    .chain(util::geometry::iter(&(pose.target)))
                    .all(|x| x.is_finite()));
                assert!(
                    pose.eye.y
                        >= world::elevation::world_height(
                            &test::support::compact_track(),
                            XZ {
                                x: pose.eye.x,
                                z: pose.eye.z
                            }
                        ) + 1.499
                );
                let gap = (0..3)
                    .map(|i| {
                        (util::geometry::xyz_component(pose.eye, i)
                            - util::geometry::xyz_component(pose.target, i))
                        .powi(2)
                    })
                    .sum::<f32>();
                assert!(gap > 10.0);
            }
        }
    }
}
