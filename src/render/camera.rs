//! Render-only chase-camera tuning. Never writes vehicle state.

use crate::physics::car;
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

struct ChaseDimensions {
    distance: f32,
    height: f32,
}

struct BonnetMount {
    forward: f32,
    height: f32,
    downward_slope: f32,
    look_distance: f32,
}

// Chase distances are behind the car centre; heights are above it.
const WIDE: ChaseDimensions = ChaseDimensions {
    distance: 9.0,
    height: 3.5,
};
const CLOSE: ChaseDimensions = ChaseDimensions {
    distance: 5.5,
    height: 2.1,
};
// Body-relative mount near the windscreen, in metres from the car centre.
const BONNET: BonnetMount = BonnetMount {
    forward: 0.2,
    height: 0.95,
    downward_slope: 0.08,
    look_distance: 30.0,
};

fn chase_dimensions(follow_view: FollowView) -> Option<ChaseDimensions> {
    match follow_view {
        FollowView::Wide => Some(WIDE),
        FollowView::Close => Some(CLOSE),
        FollowView::Bonnet => None,
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

pub fn new(car: &car::Vehicle) -> ChaseCamera {
    let mut camera = ChaseCamera {
        track: car.track.clone(),
        view: FollowView::Close,
        anchor: car.position,
        heading: car.orientation.yaw,
        look_ahead: XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        distance: CLOSE.distance,
        height: CLOSE.height,
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

pub fn snap(chase_camera: &mut ChaseCamera, car: &car::Vehicle) {
    chase_camera.bonnet = bonnet_pose(car);
    chase_camera.track = car.track.clone();
    chase_camera.anchor = car.position;
    chase_camera.heading = car.orientation.yaw.rem_euclid(TAU);
    chase_camera.look_ahead = desired_look(car);
    if let Some(dimensions) = chase_dimensions(chase_camera.view) {
        chase_camera.distance = dimensions.distance;
        chase_camera.height = dimensions.height;
    }
}

pub fn update(chase_camera: &mut ChaseCamera, car: &car::Vehicle, frame_time: Duration) {
    if !std::sync::Arc::ptr_eq(&chase_camera.track, &car.track) {
        snap(chase_camera, car);
        return;
    }
    let dt = frame_time.as_secs_f32().min(0.25);
    chase_camera.bonnet = bonnet_pose(car);
    if dt > 0.0 {
        chase_camera.anchor = car.position;
    }

    // A little velocity alignment stabilizes slides without swinging away
    // from the nose or forcing the camera to follow the route instead.
    let motion = car.velocity.x.hypot(car.velocity.z);
    let drift = if motion > 1.0 {
        angle(car.velocity.x.atan2(car.velocity.z) - car.orientation.yaw).clamp(-0.35, 0.35)
            * (car::longitudinal_speed(car).max(0.0) / 12.0).clamp(0.0, 1.0)
            * 0.25
    } else {
        0.0
    };
    let heading = car.orientation.yaw + drift;
    chase_camera.heading = (chase_camera.heading
        + angle(heading - chase_camera.heading) * damp(7.0, dt))
    .rem_euclid(TAU);

    let look = desired_look(car);
    for (current, desired) in util::geometry::xyz_iter_mut(&mut chase_camera.look_ahead).zip(look) {
        *current += (desired - *current) * damp(4.5, dt);
    }
    if let Some(dimensions) = chase_dimensions(chase_camera.view) {
        chase_camera.distance += (dimensions.distance - chase_camera.distance) * damp(5.0, dt);
        chase_camera.height += (dimensions.height - chase_camera.height) * damp(5.0, dt);
    }
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

fn bonnet_pose(car: &car::Vehicle) -> Pose {
    let body = render::gl::vehicle_body_matrix(car);
    let eye = util::geometry::xyz_from_fn(|i| {
        body[12 + i] + body[4 + i] * BONNET.height + body[8 + i] * BONNET.forward
    });
    Pose {
        eye: (eye),
        target: util::geometry::xyz_from_fn(|i| {
            util::geometry::xyz_component(eye, i)
                + (body[8 + i] - body[4 + i] * BONNET.downward_slope) * BONNET.look_distance
        }),
    }
}

fn desired_look(car: &car::Vehicle) -> XYZ {
    let speed = car::longitudinal_speed(car);
    let distance = (speed.abs() * 0.22).min(5.5);
    if distance < 0.001 {
        return XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
    }
    // Look a little along actual reverse travel without flipping the chase rig.
    let direction = if speed < 0.0 { -1.0 } else { 1.0 };
    let forward = XZ {
        x: car.orientation.yaw.sin() * direction,
        z: car.orientation.yaw.cos() * direction,
    };
    XYZ {
        x: forward.x * distance,
        y: 0.0,
        z: forward.z * distance,
    }
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
