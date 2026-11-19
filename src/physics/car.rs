use super::engine;
use super::powertrain;
use super::steering;
use super::suspension;
use super::tires;
use crate::asset;
use crate::driver;
use crate::model;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::time::Duration;

/// Minimum physics substeps per simulated second.
const PHYSICS_HZ: f32 = 240.0;

const AIR_DENSITY_KG_M3: f32 = 1.225;

#[derive(Clone, Copy, Default)]
pub struct Controls {
    pub throttle: f32,
    pub brake: f32,
    pub steering: f32,
    pub handbrake: bool,
}

/// Pitch, roll, and yaw components of orientation or its rate of change.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Orientation {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
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
    pub position: XYZ,
    /// World-space velocity in metres per second.
    pub velocity: XYZ,
    /// World-space velocity change over the last physics substep, in m/s².
    pub acceleration: XYZ,
    /// Unsmoothed acceleration along the car's forward and sideways axes, in m/s².
    /// Carries the previous substep's acceleration, including contact impulses,
    /// into suspension load transfer and tire grip. Refreshed after the tire/drag
    /// solve for body motion, then after contact resolution for the next substep.
    pub local_acceleration: suspension::LocalAcceleration,
    /// Angles in radians. Pitch and roll are lean relative to the terrain-following
    /// suspension; yaw is the heading in world space.
    pub orientation: Orientation,
    /// Time derivatives of the orientation components, in radians per second.
    pub orientation_rate: Orientation,
    //
    pub engine: engine::EngineState,
    pub suspension: suspension::SuspensionState,
    pub tires: tires::TireState,
    pub brake_lights: bool,
    pub brake_pressure: f32,
    pub steering: steering::SteeringState,
    pub gearbox: powertrain::GearboxState,
    pub driver: driver::DriverState,

    drivetrain_initialized: bool,
    force_telemetry: ForceTelemetry,
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

/// Forces applied during the last completed physics substep.
pub fn force_telemetry(vehicle: &Vehicle) -> ForceTelemetry {
    vehicle.force_telemetry
}

pub fn on_track(
    position: XYZ,
    heading: f32,
    spec: std::sync::Arc<model::car::CarSpec>,
    track: std::sync::Arc<world::locations::WorldLocation>,
) -> Vehicle {
    let engine = engine::new(&spec.engine);
    let mut car = Vehicle {
        spec,
        track,
        position,
        velocity: XYZ::default(),
        acceleration: XYZ::default(),
        orientation: Orientation {
            yaw: heading,
            ..Orientation::default()
        },
        orientation_rate: Orientation::default(),
        engine,
        suspension: suspension::SuspensionState::default(),
        brake_pressure: 0.0,
        steering: steering::SteeringState::default(),
        tires: tires::TireState::default(),
        brake_lights: false,
        gearbox: powertrain::GearboxState { gear: 1 },
        driver: driver::new(driver::Assistance {
            lift_on_shift: true,
            rev_matching: true,
        }),
        local_acceleration: suspension::LocalAcceleration::default(),
        drivetrain_initialized: false,
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

    let steps = (dt.as_secs_f32() * PHYSICS_HZ).ceil().max(1.0) as u32;
    let seconds = dt.as_secs_f32() / steps as f32;
    let mut impact: f32 = 0.0;
    for _ in 0..steps {
        impact = impact.max(update_substep(vehicle, controls, seconds));
    }

    tires::update_feedback(vehicle, lateral_speed(vehicle), controls.handbrake);
    impact
}

fn update_substep(vehicle: &mut Vehicle, controls: Controls, seconds: f32) -> f32 {
    let previous_velocity = vehicle.velocity;
    let controls = update_inputs(vehicle, controls, seconds);
    let frame = motion_frame(vehicle);

    suspension::update_suspension(vehicle, seconds);
    let traction = solve_traction(vehicle, controls, &frame, seconds);
    apply_chassis_motion(vehicle, &frame, traction, seconds);
    let impact = advance_body(vehicle, controls, &frame, seconds);
    update_acceleration(vehicle, previous_velocity, &frame, seconds);
    impact
}

/// Chassis axes and motion at the start of the solve. Keep these axes through
/// contact resolution so acceleration feeds back in the same frame next step.
struct MotionFrame {
    forward: XZ<f32>,
    right: XZ<f32>,
    gravity: XZ<f32>,
    motion: physics::tire_solver::Motion,
}

fn motion_frame(vehicle: &Vehicle) -> MotionFrame {
    let forward = XZ {
        x: vehicle.orientation.yaw.sin(),
        z: vehicle.orientation.yaw.cos(),
    };
    let right = XZ {
        x: vehicle.orientation.yaw.cos(),
        z: -vehicle.orientation.yaw.sin(),
    };
    let gradient = physics::vehicle_location::ground_gradient(&vehicle.track, vehicle.position);
    // Horizontal component of gravity projected onto y = h(x, z).
    // The planar gravity projection divides by 1 + |gradient|².
    let gravity = util::geometry::xz_map(gradient, |g| -9.81 * g / (1.0 + dot(gradient, gradient)));
    let longitudinal = dot(util::geometry::xz(vehicle.velocity), forward);
    let lateral = dot(util::geometry::xz(vehicle.velocity), right);
    MotionFrame {
        forward,
        right,
        gravity,
        motion: physics::tire_solver::Motion {
            longitudinal,
            lateral,
            yaw_rate: vehicle.orientation_rate.yaw,
        },
    }
}

struct Traction {
    motion: physics::tire_solver::Motion,
    tire_force_n: f32,
    torque_nm: f32,
}

/// Solve the coupled engine, clutch, and tire reactions.
fn solve_traction(
    vehicle: &mut Vehicle,
    controls: Controls,
    frame: &MotionFrame,
    seconds: f32,
) -> Traction {
    let spec = vehicle.spec.clone();
    let direction = if vehicle.gearbox.gear == powertrain::REVERSE_GEAR {
        -1.0
    } else {
        1.0
    };
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
    let front_speed = longitudinal * cos + (lateral + vehicle.orientation_rate.yaw * a) * sin;
    let driven_speed = front_speed * drive_bias + longitudinal * (1.0 - drive_bias);
    let drivetrain = predict_powertrain(vehicle, throttle, driven_speed, seconds);
    let torque_nm = vehicle.engine.torque_nm;
    let drive = drivetrain.as_ref().map_or(0.0, |step| {
        physics::powertrain::road_force(
            step,
            driven_speed * direction,
            spec.weight_kg,
            spec.chassis.rolling_radius,
        ) * direction
    });
    let contacts = tires::tire_contacts(
        vehicle,
        controls,
        motion,
        capacities,
        drive,
        drivetrain.is_some(),
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
    let solved = physics::tire_solver::solve(
        motion,
        physics::tire_solver::Motion {
            longitudinal: longitudinal + dot(gravity, forward) * seconds,
            lateral: lateral + dot(gravity, right) * seconds,
            ..motion
        },
        contacts,
        spec.weight_kg,
        spec.yaw_inertia_kg_m2,
        seconds,
    );
    let tire_force = (solved.impulses[0].longitudinal * cos - solved.impulses[0].lateral * sin
        + solved.impulses[1].longitudinal)
        / seconds;
    finish_drivetrain(
        vehicle,
        drivetrain,
        &solved,
        controls,
        driven_speed,
        driven_brake_force,
        seconds,
    );
    Traction {
        motion: solved.motion,
        tire_force_n: tire_force,
        torque_nm,
    }
}

fn apply_chassis_motion(
    vehicle: &mut Vehicle,
    frame: &MotionFrame,
    traction: Traction,
    seconds: f32,
) {
    let spec = &vehicle.spec;
    let forward = frame.forward;
    let right = frame.right;
    let longitudinal = frame.motion.longitudinal;
    let lateral = frame.motion.lateral;
    vehicle.orientation_rate.yaw = traction.motion.yaw_rate;
    let resistance = resistance_force(spec, longitudinal) / spec.weight_kg * seconds;
    let new_longitudinal = traction.motion.longitudinal.signum()
        * (traction.motion.longitudinal.abs() - resistance).max(0.0);
    vehicle.force_telemetry = ForceTelemetry {
        drive_force_n: traction.tire_force_n,
        resistance_force_n: (traction.motion.longitudinal - new_longitudinal).abs()
            * spec.weight_kg
            / seconds,
        torque_nm: traction.torque_nm,
    };
    let new_lateral = traction.motion.lateral;
    vehicle.velocity = XYZ {
        y: vehicle.velocity.y,
        x: forward.x * new_longitudinal + right.x * new_lateral,
        z: forward.z * new_longitudinal + right.z * new_lateral,
    };
    // Load transfer and body motion follow actual chassis acceleration. In particular,
    // static contact balancing gravity must not produce brake dive.
    vehicle.local_acceleration = suspension::LocalAcceleration {
        longitudinal: (new_longitudinal - longitudinal) / seconds,
        lateral: (new_lateral - lateral) / seconds,
    };
}

fn predict_powertrain(
    vehicle: &mut Vehicle,
    throttle: f32,
    driven_speed: f32,
    seconds: f32,
) -> Option<physics::powertrain::Step> {
    let spec = &vehicle.spec;
    if !vehicle.drivetrain_initialized {
        vehicle.tires.driven_wheel_omega = driven_speed / spec.chassis.rolling_radius;
        if vehicle.gearbox.gear != powertrain::NEUTRAL_GEAR {
            let coupled_rpm =
                physics::powertrain::coupled_rpm(spec, driven_speed, vehicle.gearbox.gear);
            vehicle.engine.rpm = vehicle.engine.rpm.max(coupled_rpm);
        }
        vehicle.drivetrain_initialized = true;
    }
    engine::update(&mut vehicle.engine, throttle);
    powertrain::drivetrain_step(vehicle, vehicle.engine.torque_nm, seconds, driven_speed)
}

fn advance_body(
    vehicle: &mut Vehicle,
    controls: Controls,
    frame: &MotionFrame,
    seconds: f32,
) -> f32 {
    suspension::update_body_motion(vehicle, seconds);
    let previous_yaw = vehicle.orientation.yaw;
    vehicle.orientation.yaw = (vehicle.orientation.yaw + vehicle.orientation_rate.yaw * seconds)
        .rem_euclid(std::f32::consts::TAU);
    let previous_position = vehicle.position;
    vehicle.position.x += vehicle.velocity.x * seconds;
    vehicle.position.z += vehicle.velocity.z * seconds;
    tires::update_wheel_rotation(
        vehicle,
        frame.motion.longitudinal,
        controls.handbrake,
        seconds,
    );
    let chassis_height = vehicle.position.y;
    let impact = resolve_road(vehicle);
    vehicle.position.y = chassis_height;
    impact.max(resolve_curbs(vehicle, previous_position, previous_yaw))
}

fn update_acceleration(
    vehicle: &mut Vehicle,
    previous_velocity: XYZ,
    frame: &MotionFrame,
    seconds: f32,
) {
    vehicle.acceleration = XYZ {
        x: (vehicle.velocity.x - previous_velocity.x) / seconds,
        y: (vehicle.velocity.y - previous_velocity.y) / seconds,
        z: (vehicle.velocity.z - previous_velocity.z) / seconds,
    };
    // Include contact impulses in the next substep's suspension load transfer.
    vehicle.local_acceleration = suspension::LocalAcceleration {
        longitudinal: dot(util::geometry::xz(vehicle.acceleration), frame.forward),
        lateral: dot(util::geometry::xz(vehicle.acceleration), frame.right),
    };
}

fn update_inputs(vehicle: &mut Vehicle, controls: Controls, seconds: f32) -> Controls {
    let controls = driver::update(&mut vehicle.driver, &vehicle.gearbox, controls, seconds);
    steering::update(vehicle, controls.steering, seconds);
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
    drivetrain: Option<physics::powertrain::Step>,
    solved: &physics::tire_solver::Solution,
    controls: Controls,
    driven_speed: f32,
    driven_brake_force: f32,
    seconds: f32,
) {
    let spec = vehicle.spec.clone();
    let drive_bias = spec.chassis.drive_bias;
    let direction = if vehicle.gearbox.gear == powertrain::REVERSE_GEAR {
        -1.0
    } else {
        1.0
    };
    let tires::AxleGeometry {
        front_offset: a,
        sin,
        cos,
        ..
    } = tires::axle_geometry(vehicle);
    let crankshaft_impulse = if let Some(step) = drivetrain {
        // Tire force acts on the chassis and reacts on the wheels.
        // Brake torque acts on the wheels separately; inferring drive
        // reaction by subtracting requested braking invents torque
        // whenever the combined contact force reaches its grip limit.
        let mut reaction = 0.0;
        for (axle, share) in [drive_bias, 1.0 - drive_bias].into_iter().enumerate() {
            if share > 0.0 {
                reaction += solved.impulses[axle].longitudinal / seconds;
            }
        }
        let reaction = reaction * direction;
        let front_speed = solved.motion.longitudinal * cos
            + (solved.motion.lateral + a * solved.motion.yaw_rate) * sin;
        let rear_speed = if controls.handbrake {
            0.0
        } else {
            solved.motion.longitudinal
        };
        let rolling_speed = front_speed * drive_bias + rear_speed * (1.0 - drive_bias);
        let rotation = physics::powertrain::finish(
            &step,
            reaction,
            spec.chassis.rolling_radius,
            driven_brake_force,
            rolling_speed * direction,
        );
        vehicle.tires.driven_wheel_omega = rotation.wheel_omega * direction;
        rotation.crankshaft_impulse
    } else {
        vehicle.tires.driven_wheel_omega = driven_speed / spec.chassis.rolling_radius;
        0.0
    };
    engine::advance(&mut vehicle.engine, crankshaft_impulse, seconds);
    tires::update_wheel_spin(vehicle, driven_speed);
}

/// Approximate an inelastic tyre/corner collision for each wheel crossing.
/// Removing velocity into the corner normal dissipates energy; the remaining
/// tangent velocity lifts the chassis. Wheel load shares distribute the impulse
/// without applying a whole-car collision four times.
fn resolve_curbs(vehicle: &mut Vehicle, previous_position: XYZ, previous_yaw: f32) -> f32 {
    if !world::street_surface::has_compiled_roads(&vehicle.track)
        || vehicle.position.y - suspension::ride_height(vehicle) > 0.16
    {
        return 0.0;
    }
    let velocity = vehicle.velocity;
    let mut change: XYZ = XYZ::default();
    for index in 0..4 {
        let a = suspension::wheel_position(vehicle, previous_position, previous_yaw, index);
        let b =
            suspension::wheel_position(vehicle, vehicle.position, vehicle.orientation.yaw, index);
        let reference = vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
        if world::transport::deck_height(&vehicle.track, b, reference)
            .is_some_and(|height| (height - reference).abs() < 1.0)
        {
            continue;
        }
        let Some(crossing) = physics::vehicle_location::crossed_curb(&vehicle.track, a, b) else {
            continue;
        };
        let radius = vehicle.spec.design.wheel_radii[index];
        // At first contact with a step, the tyre centre is r-h above its
        // corner. Steps at or above the axle give a horizontal contact normal.
        let up = (1.0 - crossing.rise / radius).clamp(0.0, 1.0);
        let horizontal = (1.0 - up * up).sqrt();
        let across = dot(util::geometry::xz(velocity), crossing.normal);
        let direction = dot(
            XZ {
                x: b.x - a.x,
                z: b.z - a.z,
            },
            crossing.normal,
        )
        .signum();
        let closing = (across * direction * horizontal - velocity.y * up).max(0.0);
        let impulse = closing * suspension::wheel_share(vehicle, index);
        change.x -= crossing.normal.x * direction * horizontal * impulse;
        change.z -= crossing.normal.z * direction * horizontal * impulse;
        change.y += up * impulse;
    }
    vehicle.velocity.x += change.x;
    vehicle.velocity.y += change.y;
    vehicle.velocity.z += change.z;
    change.x.hypot(change.z)
}

fn resolve_road(vehicle: &mut Vehicle) -> f32 {
    let mut velocity = util::geometry::xz(vehicle.velocity);
    let impact = constrain_road(vehicle, &mut velocity);
    vehicle.velocity.x = velocity.x;
    vehicle.velocity.z = velocity.z;
    impact
}

fn constrain_road(vehicle: &mut Vehicle, velocity: &mut XZ<f32>) -> f32 {
    if world::street_surface::has_compiled_roads(&vehicle.track) {
        let city = &vehicle.track;
        return physics::vehicle_location::constrain(
            city,
            &mut vehicle.position,
            velocity,
            asset::car_glb::height(&vehicle.spec.design),
        );
    }
    let bounds = world::elevation::terrain_bounds(&vehicle.track);
    let mut impact = 0.0_f32;
    for (axis, (coordinate, min, max)) in [
        (0, bounds.min.x, bounds.max.x),
        (2, bounds.min.z, bounds.max.z),
    ]
    .into_iter()
    .enumerate()
    {
        let limit = [
            min + world::locations::CAR_RADIUS,
            max - world::locations::CAR_RADIUS,
        ];
        let clamped =
            util::geometry::xyz_component(vehicle.position, coordinate).clamp(limit[0], limit[1]);
        let excess = util::geometry::xyz_component(vehicle.position, coordinate) - clamped;
        if excess.abs() > 0.0001 {
            *util::geometry::xyz_component_mut(&mut vehicle.position, coordinate) = clamped;
            let mut normal = XZ { x: 0.0, z: 0.0 };
            *util::geometry::xz_component_mut(&mut normal, axis) = excess.signum();
            impact = impact.max(physics::vehicle_location::deflect(velocity, normal));
        }
    }
    vehicle.position.y = suspension::ride_height(vehicle);
    impact
}

/// Signed velocity along the vehicle's forward axis, in metres per second.
pub fn longitudinal_speed(vehicle: &Vehicle) -> f32 {
    let (sin, cos) = vehicle.orientation.yaw.sin_cos();
    dot(util::geometry::xz(vehicle.velocity), XZ { x: sin, z: cos })
}

/// Signed velocity along the vehicle's right-facing axis, in metres per second.
fn lateral_speed(vehicle: &Vehicle) -> f32 {
    dot(
        util::geometry::xz(vehicle.velocity),
        XZ {
            x: vehicle.orientation.yaw.cos(),
            z: -vehicle.orientation.yaw.sin(),
        },
    )
}

fn dot(a: XZ<f32>, b: XZ<f32>) -> f32 {
    a.x * b.x + a.z * b.z
}

fn resistance_force(car_spec: &model::car::CarSpec, speed: f32) -> f32 {
    let drag_force = 0.5
        * AIR_DENSITY_KG_M3
        * car_spec.drag_coefficient
        * car_spec.frontal_area_m2
        * speed
        * speed;
    0.25 * car_spec.weight_kg + drag_force
}

#[cfg(test)]
#[path = "tests/car.rs"]
mod tests;
