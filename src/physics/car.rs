use super::chassis;
use super::engine;
use super::powertrain;
use super::steering;
use super::suspension;
use super::tires;
use super::wheels;
use crate::driver;
use crate::model;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::time::Duration;

/// Maximum physics substep duration, in seconds.
const PHYSICS_DT: f32 = 1.0 / 240.0;

#[derive(Clone, Copy, Default)]
pub struct Controls {
    pub throttle: f32,
    pub brake: f32,
    pub steering: f32,
    pub handbrake: bool,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Axle {
    Front,
    Rear,
}

pub struct Vehicle {
    pub spec: std::sync::Arc<model::car::CarSpec>,
    pub track: std::sync::Arc<world::locations::WorldLocation>,
    //
    pub chassis: chassis::State,
    //
    pub brake_lights: bool,
    pub brake_pressure: f32,
    pub engine: engine::State,
    pub suspension: suspension::State,
    pub tires: tires::State,
    pub wheels: wheels::State,
    pub steering: steering::State,
    pub gearbox: powertrain::State,
    pub driver: driver::State,

    /// Forces applied during the last completed physics substep.
    pub force_telemetry: ForceTelemetry,
}

#[derive(Clone, Copy, Default)]
pub struct ForceTelemetry {
    /// Net longitudinal tire force, including braking; positive is forward.
    pub drive_force_n: f32,
    /// Applied rolling resistance and aerodynamic drag magnitude.
    pub resistance_force_n: f32,
    /// Net engine torque used by the simulation, including engine drag and fuel cut.
    pub torque_nm: f32,
}

pub fn on_track(
    position: XYZ,
    heading: f32,
    spec: std::sync::Arc<model::car::CarSpec>,
    track: std::sync::Arc<world::locations::WorldLocation>,
) -> Vehicle {
    let engine = engine::new(&spec.engine);
    let mut car = Vehicle {
        spec: spec.clone(),
        track,
        chassis: chassis::new(position, heading),
        engine,
        suspension: suspension::State::default(),
        brake_pressure: 0.0,
        steering: steering::State::default(),
        tires: tires::State::default(),
        wheels: wheels::new(),
        brake_lights: false,
        gearbox: powertrain::new(&spec),
        driver: driver::new(driver::Assistance {
            lift_on_shift: true,
            rev_matching: true,
        }),
        force_telemetry: ForceTelemetry::default(),
    };
    let angles = suspension::ground_angles(&car);
    car.suspension.pitch = angles.pitch;
    car.suspension.roll = angles.roll;
    car
}

pub fn reset(vehicle: &mut Vehicle, position: XYZ, heading: f32) {
    let assistance = vehicle.driver.assistance;
    *vehicle = on_track(
        position,
        heading,
        vehicle.spec.clone(),
        vehicle.track.clone(),
    );
    vehicle.driver.assistance = assistance;
}

pub fn update(vehicle: &mut Vehicle, dt: Duration, controls: Controls) -> f32 {
    // Turn brake lights on/off.
    vehicle.brake_lights = controls.brake > 0.0;
    if dt.is_zero() {
        return 0.0;
    }

    // We don't know what input dt will be, so we reslice into
    // equal increments close to our desired physics_dt.
    let steps = (dt.as_secs_f32() / PHYSICS_DT).ceil().max(1.0) as u32;
    let dt = dt.as_secs_f32() / steps as f32;

    let mut impact: f32 = 0.0;
    for _ in 0..steps {
        impact = impact.max(update_substep(vehicle, controls, dt));
    }

    tires::update_feedback(vehicle, lateral_speed(vehicle), controls.handbrake);
    impact
}

fn update_substep(vehicle: &mut Vehicle, controls: Controls, dt: f32) -> f32 {
    let previous_velocity = vehicle.chassis.velocity;
    let controls = update_inputs(vehicle, controls, dt);
    let frame = motion_frame(vehicle);

    vehicle.suspension.in_motion = suspension::sample(vehicle);
    suspension::update(&mut vehicle.suspension, &vehicle.spec, dt);
    if let Some(height) = vehicle.suspension.out_height {
        vehicle.chassis.position.y = height;
    }
    solve_traction(vehicle, controls, &frame, dt);
    let previous_position = vehicle.chassis.position;
    let previous_yaw = vehicle.chassis.orientation.yaw;
    chassis::update(&mut vehicle.chassis, &vehicle.spec, dt);
    vehicle.force_telemetry.resistance_force_n = vehicle.chassis.out_resistance;
    let impact = resolve_contacts(vehicle, previous_position, previous_yaw);
    // Include contact impulses in the next substep's load transfer and tire grip.
    chassis::measure_acceleration(&mut vehicle.chassis, previous_velocity, dt);
    impact
}

/// Chassis axes and motion at the start of the solve.
struct MotionFrame {
    forward: XZ<f32>,
    right: XZ<f32>,
    gravity: XZ<f32>,
    motion: physics::tire_solver::Motion,
}

fn motion_frame(vehicle: &Vehicle) -> MotionFrame {
    let forward = XZ {
        x: vehicle.chassis.orientation.yaw.sin(),
        z: vehicle.chassis.orientation.yaw.cos(),
    };
    let right = XZ {
        x: vehicle.chassis.orientation.yaw.cos(),
        z: -vehicle.chassis.orientation.yaw.sin(),
    };
    let gradient =
        physics::vehicle_location::ground_gradient(&vehicle.track, vehicle.chassis.position);
    // Horizontal component of gravity projected onto y = h(x, z).
    // The planar gravity projection divides by 1 + |gradient|².
    let gravity = util::geometry::xz_map(gradient, |g| -9.81 * g / (1.0 + dot(gradient, gradient)));
    let longitudinal = dot(util::geometry::xz(vehicle.chassis.velocity), forward);
    let lateral = dot(util::geometry::xz(vehicle.chassis.velocity), right);
    MotionFrame {
        forward,
        right,
        gravity,
        motion: physics::tire_solver::Motion {
            longitudinal,
            lateral,
            yaw_rate: vehicle.chassis.angular_velocity.yaw,
        },
    }
}

/// Solve the coupled engine, clutch, and tire reactions.
fn solve_traction(vehicle: &mut Vehicle, controls: Controls, frame: &MotionFrame, seconds: f32) {
    let spec = vehicle.spec.clone();
    let forward = frame.forward;
    let right = frame.right;
    let gravity = frame.gravity;
    let motion = frame.motion;
    let longitudinal = motion.longitudinal;
    let lateral = motion.lateral;
    let geometry = tires::axle_geometry(vehicle);
    let tires::AxleGeometry {
        front_offset: a,
        sin,
        cos,
        ..
    } = geometry;

    let capacities = [Axle::Front, Axle::Rear].map(|axle| tires::tire_capacity(vehicle, axle));
    let throttle = controls.throttle.clamp(0.0, 1.0);
    let drive_bias = spec.chassis.drive_bias;
    let front_speed =
        longitudinal * cos + (lateral + vehicle.chassis.angular_velocity.yaw * a) * sin;
    let driven_speed = front_speed * drive_bias + longitudinal * (1.0 - drive_bias);
    engine::update(&mut vehicle.engine, throttle, seconds);
    vehicle.gearbox.in_engine_rpm = vehicle.engine.rpm;
    vehicle.gearbox.in_wheel_omega = vehicle.wheels.omega;
    vehicle.gearbox.in_speed = driven_speed;
    vehicle.gearbox.in_shifting = driver::is_shifting(&vehicle.driver);
    let drivetrain_engaged = powertrain::engaged(&vehicle.gearbox);
    let torque_nm = vehicle.engine.out_torque;
    let drive = powertrain::requested_force(&vehicle.gearbox, seconds);
    let contacts = tires::tire_contacts(
        vehicle,
        controls,
        motion,
        capacities,
        drive,
        drivetrain_engaged,
    );
    let driven_brake_force: f32 = contacts
        .iter()
        .enumerate()
        .filter(|(axle, _)| {
            if *axle == 0 {
                drive_bias > 0.0
            } else {
                drive_bias < 1.0
            }
        })
        .map(|(_, contact)| contact.brake_force)
        .sum();
    vehicle.tires.in_motion = motion;
    vehicle.tires.in_free_motion = physics::tire_solver::Motion {
        longitudinal: longitudinal + dot(gravity, forward) * seconds,
        lateral: lateral + dot(gravity, right) * seconds,
        ..motion
    };
    vehicle.tires.in_contacts = contacts;
    tires::update(&mut vehicle.tires, &spec, seconds);
    finish_drivetrain(vehicle, controls, driven_speed, driven_brake_force, seconds);
    let force = vehicle.tires.out_force;
    vehicle.chassis.in_force = XYZ {
        x: forward.x * force.longitudinal + right.x * force.lateral + gravity.x * spec.weight_kg,
        y: vehicle.suspension.out_force - 9.81 * spec.weight_kg,
        z: forward.z * force.longitudinal + right.z * force.lateral + gravity.z * spec.weight_kg,
    };
    vehicle.chassis.in_torque = chassis::Orientation {
        yaw: vehicle.tires.out_torque,
        ..vehicle.suspension.out_torque
    };
    vehicle.force_telemetry = ForceTelemetry {
        drive_force_n: force.longitudinal,
        torque_nm,
        resistance_force_n: 0.0,
    };
}

fn resolve_contacts(vehicle: &mut Vehicle, previous_position: XYZ, previous_yaw: f32) -> f32 {
    let height = vehicle.chassis.position.y;
    let impact = chassis::resolve_road(&mut vehicle.chassis, &vehicle.spec, &vehicle.track);
    vehicle.chassis.position.y = height;
    impact.max(chassis::resolve_curbs(
        &mut vehicle.chassis,
        &vehicle.spec,
        &vehicle.track,
        previous_position,
        previous_yaw,
    ))
}

fn update_inputs(vehicle: &mut Vehicle, controls: Controls, seconds: f32) -> Controls {
    let controls = driver::update(&mut vehicle.driver, &vehicle.gearbox, controls, seconds);
    let speed = longitudinal_speed(vehicle);
    let steering_rate = driver::steering_rate(controls.steering, speed);
    steering::update(&mut vehicle.steering, steering_rate, speed, seconds);
    // Digital pedals build pressure in 0.2 s and release in 0.08 s.
    let requested_brake = controls.brake.clamp(0.0, 1.0);
    let rate = if requested_brake > vehicle.brake_pressure {
        5.0
    } else {
        12.5
    };
    vehicle.brake_pressure +=
        (requested_brake - vehicle.brake_pressure).clamp(-rate * seconds, rate * seconds);
    controls
}

fn finish_drivetrain(
    vehicle: &mut Vehicle,
    controls: Controls,
    driven_speed: f32,
    driven_brake_force: f32,
    seconds: f32,
) {
    let spec = &vehicle.spec;
    let drive_bias = spec.chassis.drive_bias;
    let geometry = tires::axle_geometry(vehicle);
    let motion = vehicle.tires.out_contact_motion;
    let reaction: f32 = vehicle
        .tires
        .out_axle_force
        .iter()
        .enumerate()
        .filter(|(axle, _)| {
            if *axle == 0 {
                drive_bias > 0.0
            } else {
                drive_bias < 1.0
            }
        })
        .map(|(_, force)| force.longitudinal)
        .sum();
    let front_speed = motion.longitudinal * geometry.cos
        + (motion.lateral + geometry.front_offset * motion.yaw_rate) * geometry.sin;
    let rear_speed = if controls.handbrake {
        0.0
    } else {
        motion.longitudinal
    };
    vehicle.gearbox.in_force = reaction;
    vehicle.gearbox.in_brake_force = driven_brake_force;
    vehicle.gearbox.in_rolling_speed = front_speed * drive_bias + rear_speed * (1.0 - drive_bias);
    powertrain::update(&mut vehicle.gearbox, seconds);
    vehicle.engine.in_torque = vehicle.gearbox.out_reaction_torque;
    vehicle.wheels.in_torque = vehicle.gearbox.out_torque;
    vehicle.wheels.in_force = reaction;
    vehicle.wheels.in_brake_torque = vehicle.gearbox.out_brake_torque;
    vehicle.wheels.in_speed = driven_speed;
    vehicle.wheels.in_body_speed = longitudinal_speed(vehicle);
    vehicle.wheels.in_engaged = powertrain::engaged(&vehicle.gearbox);
    vehicle.wheels.in_handbrake = controls.handbrake;
    wheels::update(&mut vehicle.wheels, spec, seconds);
    tires::update_wheel_spin(vehicle, driven_speed);
}

/// Signed velocity along the vehicle's forward axis, in metres per second.
pub fn longitudinal_speed(vehicle: &Vehicle) -> f32 {
    let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
    dot(
        util::geometry::xz(vehicle.chassis.velocity),
        XZ { x: sin, z: cos },
    )
}

/// Acceleration along the vehicle's forward axis, in m/s².
pub fn longitudinal_acceleration(vehicle: &Vehicle) -> f32 {
    let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
    dot(
        util::geometry::xz(vehicle.chassis.acceleration),
        XZ { x: sin, z: cos },
    )
}

/// Acceleration along the vehicle's right-facing axis, in m/s².
pub fn lateral_acceleration(vehicle: &Vehicle) -> f32 {
    let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
    dot(
        util::geometry::xz(vehicle.chassis.acceleration),
        XZ { x: cos, z: -sin },
    )
}

/// Signed velocity along the vehicle's right-facing axis, in metres per second.
fn lateral_speed(vehicle: &Vehicle) -> f32 {
    dot(
        util::geometry::xz(vehicle.chassis.velocity),
        XZ {
            x: vehicle.chassis.orientation.yaw.cos(),
            z: -vehicle.chassis.orientation.yaw.sin(),
        },
    )
}

fn dot(a: XZ<f32>, b: XZ<f32>) -> f32 {
    a.x * b.x + a.z * b.z
}

#[cfg(test)]
#[path = "tests/car.rs"]
mod tests;
