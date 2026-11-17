pub use super::engine::calculate_top_speed;

use super::contact;
use super::engine;
use super::suspension;

use crate::asset;
use crate::model;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::time::Duration;

// Forward ratios use 1..=N.
pub const REVERSE_GEAR: u8 = 0;
pub const NEUTRAL_GEAR: u8 = u8::MAX;
pub const IDLE_RPM: f32 = 900.0;
const REV_MATCH_SECONDS: f32 = 0.22;
const SOUND_SHIFT_SECONDS: f32 = 0.36;

// Base rate of normalized steering input change per second.
const STEERING_RATE: f32 = 9.0;
// Forward speed (m/s) at which released steering reaches its full return rate.
const STEERING_CENTERING_SPEED: f32 = 10.0;

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

pub struct Vehicle {
    pub spec: std::sync::Arc<model::car::CarSpec>,
    pub track: std::sync::Arc<world::locations::WorldLocation>,
    //
    pub position: XYZ,
    /// World-space velocity in metres per second.
    pub velocity: XYZ,
    /// World-space velocity change over the last physics substep, in m/s².
    pub acceleration: XYZ,
    /// Angles in radians. Pitch and roll are lean relative to the terrain-following
    /// suspension; yaw is the heading in world space.
    pub orientation: Orientation,
    /// Time derivatives of the orientation components, in radians per second.
    pub orientation_rate: Orientation,
    //
    pub engine: engine::EngineState,
    pub gear: u8,
    slip: f32,
    pub suspension_pitch: f32,
    pub suspension_roll: f32,
    pub curb_bump: f32,
    pub wheel_phase: [f32; 2], // rear/front axles; cosmetic, never affects physics
    pub tire_scrub: f32,
    pub brake_lights: bool,
    pub brake_pressure: f32,
    steering: f32,
    shift_timer: f32,
    pub filtered_acceleration: suspension::LocalAcceleration,
    pub suspension_rates: [f32; 2],
    pub suspension_initialized: bool,
    pub wheel_spin: f32,
    pub driven_wheel_omega: f32, // signed wheel angular speed, radians/second
    drivetrain_initialized: bool,
    pub axle_load: [f32; 2],
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
        engine: engine::EngineState {
            rpm: IDLE_RPM,
            load: 0.0,
            turbo_spool: 0.0,
            turbo_boost: 0.0,
            fuel_cut: false,
            rev_match_timer: 0.0,
            sound_shift_timer: 0.0,
        },
        gear: 1,
        slip: 0.0,
        suspension_pitch: 0.0,
        suspension_roll: 0.0,
        curb_bump: 0.0,
        brake_pressure: 0.0,
        steering: 0.0,
        wheel_phase: [0.0; 2],
        tire_scrub: 0.0,
        brake_lights: false,
        shift_timer: 0.0,
        filtered_acceleration: suspension::LocalAcceleration::default(),
        suspension_rates: [0.0; 2],
        suspension_initialized: false,
        wheel_spin: 0.0,
        driven_wheel_omega: 0.0,
        drivetrain_initialized: false,
        axle_load: [0.0; 2],
        force_telemetry: ForceTelemetry::default(),
    };
    let angles = suspension::ground_angles(&car);
    car.suspension_pitch = angles.pitch;
    car.suspension_roll = angles.roll;
    car
}

pub fn reset(vehicle: &mut Vehicle, position: XYZ, heading: f32) {
    *vehicle = on_track(
        position,
        heading,
        vehicle.spec.clone(),
        vehicle.track.clone(),
    );
}

fn coupled_engine_rpm(vehicle: &Vehicle) -> f32 {
    if vehicle.gear == NEUTRAL_GEAR {
        return IDLE_RPM;
    }
    engine::rpm(&vehicle.spec, longitudinal_speed(vehicle), vehicle.gear).max(IDLE_RPM)
}

pub fn shift(vehicle: &mut Vehicle, up: bool) {
    let next = match (vehicle.gear, up) {
        (REVERSE_GEAR, true) => NEUTRAL_GEAR,
        (NEUTRAL_GEAR, true) => 1,
        (NEUTRAL_GEAR, false) => REVERSE_GEAR,
        (1, false) => NEUTRAL_GEAR,
        (REVERSE_GEAR, false) => REVERSE_GEAR,
        (gear, true) => (gear + 1).min(engine::gears(&vehicle.spec.gearbox)),
        (gear, false) => gear - 1,
    };
    if next != vehicle.gear {
        let previous = vehicle.gear;
        vehicle.gear = next;
        let launching = previous == NEUTRAL_GEAR && matches!(next, REVERSE_GEAR | 1);
        vehicle.shift_timer = if launching { 0.0 } else { 0.16 };
        vehicle.engine.sound_shift_timer = SOUND_SHIFT_SECONDS;
        begin_rev_match(vehicle, previous);
    }
}

fn begin_rev_match(vehicle: &mut Vehicle, previous: u8) {
    // Only a moving forward downshift needs a throttle blip. Neutral,
    // reverse and shifts below idle should not rev the engine.
    vehicle.engine.rev_match_timer = if vehicle.gear != REVERSE_GEAR
        && vehicle.gear < previous
        && previous <= engine::gears(&vehicle.spec.gearbox)
        && coupled_engine_rpm(vehicle) > vehicle.engine.rpm + 100.0
    {
        REV_MATCH_SECONDS
    } else {
        0.0
    };
}

pub fn update(vehicle: &mut Vehicle, dt: Duration, controls: Controls) -> f32 {
    // Turn brake lights on/off.
    vehicle.brake_lights = controls.brake > 0.0;
    if dt.is_zero() {
        return 0.0;
    }

    let steps = (dt.as_secs_f32() * 240.0).ceil().max(1.0) as u32;
    let seconds = dt.as_secs_f32() / steps as f32;
    let mut impact: f32 = 0.0;
    for _ in 0..steps {
        impact = impact.max(update_substep(vehicle, controls, seconds));
    }

    refresh_slip(vehicle);
    update_engine_feedback(vehicle, controls, dt.as_secs_f32());
    let motion = vehicle.velocity.x.hypot(vehicle.velocity.z);
    let sliding = ((vehicle.slip.abs() - 0.8) / 3.0).clamp(0.0, 1.0);
    let braking = if controls.handbrake { 0.8 } else { 0.0 };
    vehicle.tire_scrub =
        (sliding.max(braking) * ((motion - 1.5) / 2.5).clamp(0.0, 1.0)).max(vehicle.wheel_spin);
    impact
}

fn update_substep(vehicle: &mut Vehicle, controls: Controls, seconds: f32) -> f32 {
    let previous_velocity = vehicle.velocity;
    let spec = vehicle.spec.clone();
    let direction = if vehicle.gear == REVERSE_GEAR {
        -1.0
    } else {
        1.0
    };
    update_inputs(vehicle, controls, seconds);
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
    let geometry = contact::axle_geometry(vehicle);
    let contact::AxleGeometry {
        front_offset: a,
        sin,
        cos,
        ..
    } = geometry;

    suspension::update_suspension(vehicle, seconds);
    let capacities = contact::tire_capacities(vehicle);
    let throttle = controls.throttle.clamp(0.0, 1.0);
    update_fuel_cut(vehicle, vehicle.engine.rpm);
    update_turbo(vehicle, seconds, throttle);
    let drive_bias = spec.drive_bias;
    let front_speed = longitudinal * cos + (lateral + vehicle.orientation_rate.yaw * a) * sin;
    let driven_speed = front_speed * drive_bias + longitudinal * (1.0 - drive_bias);
    if !vehicle.drivetrain_initialized {
        vehicle.driven_wheel_omega = driven_speed / spec.rolling_radius;
        if vehicle.gear != NEUTRAL_GEAR {
            let coupled_rpm = engine::rpm(&spec, driven_speed, vehicle.gear);
            vehicle.engine.rpm = vehicle.engine.rpm.max(coupled_rpm);
        }
        vehicle.drivetrain_initialized = true;
    }
    let torque_nm = engine_net_torque(vehicle, throttle);
    let drivetrain = drivetrain_step(vehicle, throttle, seconds, driven_speed);
    let drive = drivetrain.as_ref().map_or(0.0, |step| {
        physics::powertrain::road_force(
            step,
            driven_speed * direction,
            spec.weight_kg,
            spec.rolling_radius,
        ) * direction
    });
    let motion = physics::tire::Motion {
        longitudinal,
        lateral,
        yaw_rate: vehicle.orientation_rate.yaw,
    };
    let contacts = contact::tire_contacts(
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
    let solved = physics::tire::solve(
        motion,
        physics::tire::Motion {
            longitudinal: longitudinal + dot(gravity, forward) * seconds,
            lateral: lateral + dot(gravity, right) * seconds,
            ..motion
        },
        contacts,
        spec.weight_kg,
        spec.yaw_inertia,
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
    vehicle.orientation_rate.yaw = solved.motion.yaw_rate;
    let resistance = engine::resistance_force(&spec, longitudinal) / spec.weight_kg * seconds;
    let new_longitudinal = solved.motion.longitudinal.signum()
        * (solved.motion.longitudinal.abs() - resistance).max(0.0);
    vehicle.force_telemetry = ForceTelemetry {
        drive_force_n: tire_force,
        resistance_force_n: (solved.motion.longitudinal - new_longitudinal).abs() * spec.weight_kg
            / seconds,
        torque_nm,
    };
    let new_lateral = solved.motion.lateral;
    vehicle.velocity = XYZ {
        y: vehicle.velocity.y,
        x: forward.x * new_longitudinal + right.x * new_lateral,
        z: forward.z * new_longitudinal + right.z * new_lateral,
    };
    let blend = 1.0 - (-seconds * 12.0).exp();
    // Body motion follows actual chassis acceleration. In particular,
    // static contact balancing gravity must not produce brake dive.
    let ax = (new_longitudinal - longitudinal) / seconds;
    let ay = (new_lateral - lateral) / seconds;
    vehicle.filtered_acceleration.longitudinal +=
        (ax - vehicle.filtered_acceleration.longitudinal) * blend;
    vehicle.filtered_acceleration.lateral += (ay - vehicle.filtered_acceleration.lateral) * blend;
    suspension::update_body_motion(vehicle, seconds);
    vehicle.orientation.yaw = (vehicle.orientation.yaw + vehicle.orientation_rate.yaw * seconds)
        .rem_euclid(std::f32::consts::TAU);
    let previous_position = vehicle.position;
    vehicle.position.x += vehicle.velocity.x * seconds;
    vehicle.position.z += vehicle.velocity.z * seconds;
    update_wheel_rotation(vehicle, longitudinal, controls.handbrake, seconds);
    let chassis_height = vehicle.position.y;
    let impact = resolve_road(vehicle);
    vehicle.position.y = chassis_height;
    resolve_curbs(vehicle, previous_position, seconds);
    vehicle.acceleration = XYZ {
        x: (vehicle.velocity.x - previous_velocity.x) / seconds,
        y: (vehicle.velocity.y - previous_velocity.y) / seconds,
        z: (vehicle.velocity.z - previous_velocity.z) / seconds,
    };
    impact
}

fn update_inputs(vehicle: &mut Vehicle, controls: Controls, seconds: f32) {
    vehicle.shift_timer = (vehicle.shift_timer - seconds).max(0.0);
    vehicle.engine.rev_match_timer = (vehicle.engine.rev_match_timer - seconds).max(0.0);
    update_steering(vehicle, controls.steering, seconds);
    // Digital pedals build pressure in 0.2 s and release in 0.08 s.
    let requested_brake = controls.brake.clamp(0.0, 1.0);
    let rate = if requested_brake > vehicle.brake_pressure {
        5.0
    } else {
        12.5
    };
    vehicle.brake_pressure +=
        (requested_brake - vehicle.brake_pressure).clamp(-rate * seconds, rate * seconds);
}

fn finish_drivetrain(
    vehicle: &mut Vehicle,
    drivetrain: Option<physics::powertrain::Step>,
    solved: &physics::tire::Solution,
    controls: Controls,
    driven_speed: f32,
    driven_brake_force: f32,
    seconds: f32,
) {
    let spec = vehicle.spec.clone();
    let drive_bias = spec.drive_bias;
    let direction = if vehicle.gear == REVERSE_GEAR {
        -1.0
    } else {
        1.0
    };
    let contact::AxleGeometry {
        front_offset: a,
        sin,
        cos,
        ..
    } = contact::axle_geometry(vehicle);
    let throttle = controls.throttle.clamp(0.0, 1.0);
    if let Some(step) = drivetrain {
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
            spec.rolling_radius,
            driven_brake_force,
            rolling_speed * direction,
        );
        vehicle.engine.rpm = rotation.engine_rpm.max(IDLE_RPM);
        vehicle.driven_wheel_omega = rotation.wheel_omega * direction;
    } else {
        update_free_engine(vehicle, throttle, seconds);
        vehicle.driven_wheel_omega = driven_speed / spec.rolling_radius;
    }
    update_fuel_cut(vehicle, vehicle.engine.rpm);
    let slip_speed = vehicle.driven_wheel_omega * spec.rolling_radius - driven_speed;
    vehicle.wheel_spin = (slip_speed.abs() / 8.0).clamp(0.0, 1.0);
}

fn update_wheel_rotation(vehicle: &mut Vehicle, longitudinal: f32, handbrake: bool, seconds: f32) {
    let spec = &vehicle.spec;
    let spin = longitudinal * seconds / spec.rolling_radius;
    let spin_slip = (vehicle.driven_wheel_omega - longitudinal / spec.rolling_radius) * seconds;
    vehicle.wheel_phase[1] = (vehicle.wheel_phase[1]
        + spin
        + if spec.drive_bias > 0.0 {
            spin_slip
        } else {
            0.0
        })
    .rem_euclid(std::f32::consts::TAU);
    if !handbrake {
        vehicle.wheel_phase[0] = (vehicle.wheel_phase[0]
            + spin
            + if spec.drive_bias < 1.0 {
                spin_slip
            } else {
                0.0
            })
        .rem_euclid(std::f32::consts::TAU);
    }
}

fn resolve_curbs(vehicle: &mut Vehicle, previous_position: XYZ, seconds: f32) {
    vehicle.curb_bump *= (-seconds * 12.0).exp();
    if world::street_surface::has_compiled_roads(&vehicle.track)
        && world::transport::deck_height(
            &vehicle.track,
            util::geometry::xz(vehicle.position),
            vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
        )
        .is_none_or(|h| {
            (h - (vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT)).abs() > 1.
        })
    {
        let city = &vehicle.track;
        if let Some((normal, rise)) = physics::vehicle_location::crossed_curb(
            city,
            XZ {
                x: previous_position.x,
                z: previous_position.z,
            },
            XZ {
                x: vehicle.position.x,
                z: vehicle.position.z,
            },
        ) {
            let across = dot(util::geometry::xz(vehicle.velocity), normal);
            let loss = (across.abs() * 0.08).min(1.2) * (rise / 0.09);
            vehicle.velocity.x -= normal.x * across.signum() * loss;
            vehicle.velocity.z -= normal.z * across.signum() * loss;
            vehicle.curb_bump = vehicle
                .curb_bump
                .max(0.03 * (rise / 0.09) * (across.abs() / 6.0).min(1.0));
        }
    }
}

fn engine_inertia(vehicle: &Vehicle) -> f32 {
    0.12 + vehicle.spec.engine.max_torque_nm * 0.0006
}

fn engine_net_torque(vehicle: &Vehicle, throttle: f32) -> f32 {
    let spec = &vehicle.spec;
    let revs = ((vehicle.engine.rpm - IDLE_RPM) / (spec.engine.redline_rpm - IDLE_RPM)).max(0.0);
    let drag = spec.engine.max_torque_nm * (0.10 + 0.16 * revs + 0.24 * revs * revs);
    let throttle = if vehicle.shift_timer > 0.0 && vehicle.gear != NEUTRAL_GEAR {
        0.85 * vehicle.engine.rev_match_timer / REV_MATCH_SECONDS
    } else {
        throttle
    };
    let combustion = if vehicle.engine.fuel_cut {
        0.0
    } else {
        throttle
            * (engine::engine_torque(spec, vehicle.engine.rpm) * engine::DRIVETRAIN_EFFICIENCY
                + drag)
    };
    combustion - drag
}

fn drivetrain_step(
    vehicle: &Vehicle,
    throttle: f32,
    dt: f32,
    speed: f32,
) -> Option<physics::powertrain::Step> {
    if vehicle.gear == NEUTRAL_GEAR || vehicle.shift_timer > 0.0 {
        return None;
    }
    let spec = &vehicle.spec;
    let gearbox = &spec.gearbox;
    let ratio = if vehicle.gear == REVERSE_GEAR {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[(vehicle.gear - 1) as usize]
    } * gearbox.final_drive;
    // Automatic clutch opens near idle to avoid stalling. Its torque capacity
    // increases with engine speed; there is no launch RPM target.
    let engagement = if engine::rpm(spec, speed, vehicle.gear) > IDLE_RPM + 100.0 {
        1.0
    } else {
        ((vehicle.engine.rpm - IDLE_RPM) / 600.0).clamp(0.0, 1.0)
    };
    let direction = if vehicle.gear == REVERSE_GEAR {
        -1.0
    } else {
        1.0
    };
    Some(physics::powertrain::new(
        vehicle.engine.rpm,
        vehicle.driven_wheel_omega * direction,
        engine_net_torque(vehicle, throttle),
        engine_inertia(vehicle),
        ratio,
        spec.engine.max_torque_nm * 2.0 * engagement,
        dt,
    ))
}

fn update_turbo(vehicle: &mut Vehicle, dt: f32, throttle: f32) {
    let Some(tune) = &vehicle.spec.engine.turbo else {
        vehicle.engine.turbo_spool = 0.0;
        vehicle.engine.turbo_boost = 0.0;
        return;
    };
    let loaded = vehicle.gear != NEUTRAL_GEAR && vehicle.shift_timer <= 0.0;
    let rpm = vehicle.engine.rpm;
    let exhaust = ((rpm - tune.spool_start_rpm) / (tune.full_boost_rpm - tune.spool_start_rpm))
        .clamp(0.0, 1.0);
    let target = if loaded {
        exhaust * throttle * throttle
    } else {
        0.0
    };
    let time = if target > vehicle.engine.turbo_spool {
        tune.spool_seconds
    } else {
        tune.decay_seconds
    };
    vehicle.engine.turbo_spool +=
        (target - vehicle.engine.turbo_spool) * (1.0 - (-dt / time).exp());
    // Closing the throttle vents manifold pressure while the turbine coasts.
    let boost_target = if loaded {
        vehicle.engine.turbo_spool * throttle
    } else {
        0.0
    };
    vehicle.engine.turbo_boost +=
        (boost_target - vehicle.engine.turbo_boost) * (1.0 - (-dt / 0.06).exp());
}

fn update_fuel_cut(vehicle: &mut Vehicle, rpm: f32) {
    // The same hysteresis applies whether the engine is free or coupled to
    // the wheels. Cutting combustion cannot prevent a mechanical overrev.
    if rpm >= vehicle.spec.engine.redline_rpm {
        vehicle.engine.fuel_cut = true;
    } else if rpm <= vehicle.spec.engine.redline_rpm - 250.0 {
        vehicle.engine.fuel_cut = false;
    }
}

fn update_free_engine(vehicle: &mut Vehicle, throttle: f32, dt: f32) {
    update_fuel_cut(vehicle, vehicle.engine.rpm);
    let rpm_rate = engine_net_torque(vehicle, throttle) / engine_inertia(vehicle) * 60.0
        / std::f32::consts::TAU;
    vehicle.engine.rpm = (vehicle.engine.rpm + rpm_rate * dt).max(IDLE_RPM);
    update_fuel_cut(vehicle, vehicle.engine.rpm);
}

fn update_engine_feedback(vehicle: &mut Vehicle, controls: Controls, dt: f32) {
    let throttle = controls.throttle.clamp(0.0, 1.0);

    vehicle.engine.sound_shift_timer = (vehicle.engine.sound_shift_timer - dt).max(0.0);

    let blip = vehicle.engine.rev_match_timer / REV_MATCH_SECONDS;

    // An unloaded engine uses less sustained load; fuel cuts also reach
    // the audio layers so the limiter is audible as well as visible.
    let requested = if vehicle.engine.fuel_cut {
        0.0
    } else if vehicle.gear == NEUTRAL_GEAR {
        throttle * 0.55
    } else {
        throttle
    };
    let shift_progress = 1.0 - vehicle.engine.sound_shift_timer / SOUND_SHIFT_SECONDS;
    let shift_unload = 0.15 + 0.85 * shift_progress * shift_progress;
    // Feed the blip into both recorded load/coast layers and procedural
    // engine voices, without applying extra driving force at the wheels.
    let target_load = (requested * shift_unload + 0.12 * (1.0 - requested))
        .max(if vehicle.engine.fuel_cut {
            0.0
        } else {
            0.85 * blip
        })
        .clamp(0.0, 1.0);
    vehicle.engine.load += (target_load - vehicle.engine.load) * (dt * 10.0).clamp(0.0, 1.0);
}

fn update_steering(vehicle: &mut Vehicle, requested: f32, dt: f32) {
    let requested = requested.clamp(-1.0, 1.0);
    if requested == 0.0 {
        // Stationary wheels hold their angle. Forward motion progressively
        // restores self-centering, independently of deliberate countersteering.
        let motion = (longitudinal_speed(vehicle) / STEERING_CENTERING_SPEED).clamp(0.0, 1.0);
        let step = STEERING_RATE * 0.45 * motion * dt;
        vehicle.steering -= vehicle.steering.clamp(-step, step);
        return;
    }
    // Cross the centre quickly when countersteering, then use the remaining
    // time to build the opposite angle so response is independent of step size.
    let mut remaining = dt;
    if requested * vehicle.steering < 0.0 {
        let rate = STEERING_RATE * 0.85;
        let centering_time = vehicle.steering.abs() / rate;
        let elapsed = remaining.min(centering_time);
        vehicle.steering -= vehicle.steering.clamp(-rate * elapsed, rate * elapsed);
        remaining -= elapsed;
    }
    let rate = STEERING_RATE * 0.45 / (1.0 + longitudinal_speed(vehicle).abs() * 0.008);
    vehicle.steering += (requested - vehicle.steering).clamp(-rate * remaining, rate * remaining);
}

fn steering_angle(vehicle: &Vehicle) -> f32 {
    vehicle.steering.signum() * vehicle.steering.abs().powf(1.10) * 0.54
}

pub fn front_wheel_angle(vehicle: &Vehicle) -> f32 {
    -steering_angle(vehicle)
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

fn refresh_slip(vehicle: &mut Vehicle) {
    vehicle.slip = dot(
        util::geometry::xz(vehicle.velocity),
        XZ {
            x: vehicle.orientation.yaw.cos(),
            z: -vehicle.orientation.yaw.sin(),
        },
    );
}

fn dot(a: XZ<f32>, b: XZ<f32>) -> f32 {
    a.x * b.x + a.z * b.z
}

#[cfg(test)]
#[path = "tests/car.rs"]
mod tests;
