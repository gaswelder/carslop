//! Chassis support, load transfer, and sprung body motion.

use super::car;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::world;

/// Smoothed local acceleration for load transfer and body lean, in m/s².
#[derive(Default)]
pub struct LocalAcceleration {
    pub longitudinal: f32,
    pub lateral: f32,
}

pub fn update_suspension(vehicle: &mut car::Vehicle, seconds: f32) {
    let spec = &vehicle.spec;
    // Suspension supports the chassis rather than pinning it to terrain.
    // At a crest its contact force can drop to zero; gravity then governs
    // flight until the spring compresses again on landing.
    let ground = ride_height(vehicle);
    if !vehicle.suspension_initialized {
        vehicle.position.y = ground;
        vehicle.suspension_initialized = true;
    }
    let omega = std::f32::consts::TAU * spec.suspension_hz;
    let support = (9.81 + omega * omega * (ground - vehicle.position.y)
        - 1.3 * omega * vehicle.velocity.y)
        .clamp(0.0, 49.05);
    vehicle.acceleration.y = support - 9.81;
    vehicle.velocity.y += vehicle.acceleration.y * seconds;
    vehicle.position.y += vehicle.velocity.y * seconds;
    let transfer = spec.weight_kg * vehicle.filtered_acceleration.longitudinal * spec.cg_height
        / spec.chassis.wheelbase;
    let total_load = spec.weight_kg * support;
    let front_load = (total_load * spec.front_weight - transfer).clamp(0.0, total_load);
    vehicle.axle_load = [front_load, total_load - front_load];
}

pub fn ride_height(vehicle: &car::Vehicle) -> f32 {
    world::transport::drive_height_near(
        &vehicle.track,
        util::geometry::xz(vehicle.position),
        vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
    ) + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT
}

pub struct GroundAngles {
    pub pitch: f32,
    pub roll: f32,
}

pub fn ground_angles(vehicle: &car::Vehicle) -> GroundAngles {
    let (sin, cos) = vehicle.orientation.yaw.sin_cos();
    let h = |dx: f32, dz: f32| {
        let p = XYZ {
            x: vehicle.position.x + dx,
            y: vehicle.position.y,
            z: vehicle.position.z + dz,
        };
        world::transport::drive_height_near(
            &vehicle.track,
            util::geometry::xz(p),
            vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
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

pub fn update_body_motion(vehicle: &mut car::Vehicle, dt: f32) {
    let spec = &vehicle.spec;
    let target_pitch =
        (vehicle.filtered_acceleration.longitudinal * spec.cg_height / 80.0).clamp(-0.10, 0.10);
    let target_roll =
        (vehicle.filtered_acceleration.lateral * spec.cg_height / 65.0).clamp(-0.12, 0.12);
    let omega = std::f32::consts::TAU * spec.suspension_hz;
    vehicle.orientation_rate.pitch += (omega * omega * (target_pitch - vehicle.orientation.pitch)
        - 1.4 * omega * vehicle.orientation_rate.pitch)
        * dt;
    vehicle.orientation_rate.roll += (omega * omega * (target_roll - vehicle.orientation.roll)
        - 1.4 * omega * vehicle.orientation_rate.roll)
        * dt;
    vehicle.orientation.pitch += vehicle.orientation_rate.pitch * dt;
    vehicle.orientation.roll += vehicle.orientation_rate.roll * dt;
    // The sprung body follows terrain through damped angular springs.
    // Wheels follow the contact surface independently in the renderer.
    let angles = if vehicle.axle_load.iter().sum::<f32>() > 0.0 {
        ground_angles(vehicle)
    } else {
        GroundAngles {
            pitch: vehicle.suspension_pitch,
            roll: vehicle.suspension_roll,
        }
    };
    for (axis, target, angle) in [
        (0, angles.pitch, &mut vehicle.suspension_pitch),
        (1, angles.roll, &mut vehicle.suspension_roll),
    ] {
        let rate = &mut vehicle.suspension_rates[axis];
        *rate += (omega * omega * (target - *angle) - 1.4 * omega * *rate) * dt;
        *angle += *rate * dt;
    }
}
