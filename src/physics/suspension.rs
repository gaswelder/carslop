//! Chassis support, load transfer, and sprung body motion.

use super::car;
use super::chassis;
use crate::model;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

/// Normal force supported by each axle, in newtons.
#[derive(Default)]
pub struct AxleLoad {
    pub front: f32,
    pub rear: f32,
}

/// Terrain-following suspension motion and supported axle loads.
#[derive(Default)]
pub struct State {
    pub in_motion: Input,
    pub out_force: f32,
    pub out_torque: chassis::Orientation,
    pub out_height: Option<f32>,
    pub pitch: f32,
    pub roll: f32,
    pub axle_load: AxleLoad,
    /// Smoothed local acceleration for cosmetic body lean only, in m/s².
    visual_acceleration: LocalAcceleration,
    rates: [f32; 2],
    initialized: bool,
}

/// Acceleration along the car’s longitudinal and lateral axes, in m/s².
#[derive(Default)]
struct LocalAcceleration {
    longitudinal: f32,
    lateral: f32,
}

#[derive(Default)]
pub struct Input {
    pub height: f32,
    pub vertical_speed: f32,
    pub longitudinal_acceleration: f32,
    pub lateral_acceleration: f32,
    pub ground_height: f32,
    pub ground_angles: GroundAngles,
    pub orientation: chassis::Orientation,
    pub angular_velocity: chassis::Orientation,
}

/// Sample environmental and chassis outputs without modifying either component.
pub fn sample(vehicle: &car::Vehicle) -> Input {
    // CG drag slows the car without the pitching moment of a tire force.
    // Remove only its measured contribution, preserving rolling resistance,
    // tire braking, and collision impulses in load transfer.
    let aerodynamic = vehicle.chassis.out_aerodynamic_acceleration;
    let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
    Input {
        height: vehicle.chassis.position.y,
        vertical_speed: vehicle.chassis.velocity.y,
        longitudinal_acceleration: car::longitudinal_acceleration(vehicle)
            - (aerodynamic.x * sin + aerodynamic.z * cos),
        lateral_acceleration: car::lateral_acceleration(vehicle)
            - (aerodynamic.x * cos - aerodynamic.z * sin),
        ground_height: ride_height(vehicle),
        ground_angles: ground_angles(vehicle),
        orientation: vehicle.chassis.orientation,
        angular_velocity: vehicle.chassis.angular_velocity,
    }
}

/// Publish support force, body torques, and axle loads from the sampled motion.
pub fn update(state: &mut State, spec: &model::car::CarSpec, dt: f32) {
    let ground = state.in_motion.ground_height;
    state.out_height = if state.initialized {
        None
    } else {
        Some(ground)
    };
    let height = if state.initialized {
        state.in_motion.height
    } else {
        ground
    };
    state.initialized = true;
    let omega_squared = spec.chassis.suspension_stiffness_n_m / spec.weight_kg;
    let omega = omega_squared.sqrt();
    let support = (9.81 + omega_squared * (ground - height)
        - 1.3 * omega * state.in_motion.vertical_speed)
        .clamp(0.0, 49.05);
    state.out_force = spec.weight_kg * support;
    let transfer = spec.weight_kg
        * state.in_motion.longitudinal_acceleration
        * model::car::center_of_mass_height_m(spec)
        / spec.chassis.wheelbase;
    let front = (state.out_force * spec.center_of_mass.longitudinal_fraction - transfer)
        .clamp(0.0, state.out_force);
    state.axle_load = AxleLoad {
        front,
        rear: state.out_force - front,
    };
    update_body_motion(state, spec, dt);
}

pub fn ride_height(vehicle: &car::Vehicle) -> f32 {
    surface_height(
        &vehicle.spec,
        &vehicle.track,
        vehicle.chassis.position,
        vehicle.chassis.orientation.yaw,
    )
}

pub fn surface_height(
    spec: &model::car::CarSpec,
    track: &world::locations::WorldLocation,
    position: XYZ,
    yaw: f32,
) -> f32 {
    let height = |p| {
        world::transport::drive_height_near(
            track,
            p,
            position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
        ) + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT
    };
    if !world::street_surface::has_compiled_roads(track) {
        return height(util::geometry::xz(position));
    }
    (0..4)
        .map(|index| height(wheel_position(spec, position, yaw, index)) * wheel_share(spec, index))
        .sum()
}

pub fn wheel_position(
    spec: &model::car::CarSpec,
    position: XYZ,
    yaw: f32,
    index: usize,
) -> XZ<f32> {
    let mount = spec.design.mounts[index];
    let (sin, cos) = yaw.sin_cos();
    XZ {
        x: position.x + mount.x * cos + mount.z * sin,
        z: position.z - mount.x * sin + mount.z * cos,
    }
}

pub fn wheel_share(spec: &model::car::CarSpec, index: usize) -> f32 {
    let front = spec.center_of_mass.longitudinal_fraction;
    if index >= 2 {
        front * 0.5
    } else {
        (1.0 - front) * 0.5
    }
}

#[derive(Default)]
pub struct GroundAngles {
    pub pitch: f32,
    pub roll: f32,
}

pub fn ground_angles(vehicle: &car::Vehicle) -> GroundAngles {
    let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
    let h = |dx: f32, dz: f32| {
        let p = XYZ {
            x: vehicle.chassis.position.x + dx,
            y: vehicle.chassis.position.y,
            z: vehicle.chassis.position.z + dz,
        };
        world::transport::drive_height_near(
            &vehicle.track,
            util::geometry::xz(p),
            vehicle.chassis.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
        )
    };
    let length = vehicle.spec.chassis.wheelbase;
    let width = vehicle.spec.chassis.track_width;
    GroundAngles {
        pitch: ((h(sin * length * 0.5, cos * length * 0.5)
            - h(-sin * length * 0.5, -cos * length * 0.5))
            / length)
            .atan()
            .clamp(-0.35, 0.35),
        roll: ((h(cos * width * 0.5, -sin * width * 0.5)
            - h(-cos * width * 0.5, sin * width * 0.5))
            / width)
            .atan()
            .clamp(-0.25, 0.25),
    }
}

fn update_body_motion(state: &mut State, spec: &model::car::CarSpec, dt: f32) {
    // This filter only steadies rendered pitch and roll; tire loads use raw acceleration.
    let blend = 1.0 - (-dt * 12.0).exp();
    state.visual_acceleration.longitudinal += (state.in_motion.longitudinal_acceleration
        - state.visual_acceleration.longitudinal)
        * blend;
    state.visual_acceleration.lateral +=
        (state.in_motion.lateral_acceleration - state.visual_acceleration.lateral) * blend;
    let target_pitch =
        (state.visual_acceleration.longitudinal * model::car::center_of_mass_height_m(spec) / 80.0)
            .clamp(-0.10, 0.10);
    let target_roll =
        (state.visual_acceleration.lateral * model::car::center_of_mass_height_m(spec) / 65.0)
            .clamp(-0.12, 0.12);
    let omega_squared = spec.chassis.suspension_stiffness_n_m / spec.weight_kg;
    let omega = omega_squared.sqrt();
    state.out_torque = chassis::Orientation {
        pitch: spec.weight_kg
            * (omega_squared * (target_pitch - state.in_motion.orientation.pitch)
                - 1.4 * omega * state.in_motion.angular_velocity.pitch),
        roll: spec.weight_kg
            * (omega_squared * (target_roll - state.in_motion.orientation.roll)
                - 1.4 * omega * state.in_motion.angular_velocity.roll),
        yaw: 0.0,
    };
    // The sprung body follows terrain through damped angular springs.
    // Wheels follow the contact surface independently in the renderer.
    let angles = if state.axle_load.front + state.axle_load.rear > 0.0 {
        GroundAngles {
            pitch: state.in_motion.ground_angles.pitch,
            roll: state.in_motion.ground_angles.roll,
        }
    } else {
        GroundAngles {
            pitch: state.pitch,
            roll: state.roll,
        }
    };
    for (axis, target, angle) in [
        (0, angles.pitch, &mut state.pitch),
        (1, angles.roll, &mut state.roll),
    ] {
        let rate = &mut state.rates[axis];
        *rate += (omega_squared * (target - *angle) - 1.4 * omega * *rate) * dt;
        *angle += *rate * dt;
    }
}

#[cfg(test)]
#[path = "tests/suspension.rs"]
mod tests;
