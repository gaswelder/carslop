use crate::asset;
use crate::model;
use crate::model::world::Material;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::time::Duration;

// Keep zero for reverse and 1..=N for forward ratios.
pub const NEUTRAL_GEAR: u8 = u8::MAX;
pub const IDLE_RPM: f32 = 900.0;
const REV_MATCH_SECONDS: f32 = 0.22;
const SOUND_SHIFT_SECONDS: f32 = 0.36;

const AIR_DENSITY_KG_M3: f32 = 1.225;

// 90% drivetrain efficiency to calculate power at the wheels.
const DRIVETRAIN_EFFICIENCY: f32 = 0.90;

fn friction_coefficient(material: Material) -> f32 {
    match material {
        Material::BareGround => 0.65,
        Material::Paving => 0.90,
        Material::Grass => 0.55,
        Material::Park => 0.55,
        Material::Scrub => 0.45,
        Material::Asphalt => 1.0,
        Material::Concrete => 1.0,
        Material::Sports => 0.65,
        Material::Water => 0.10,
        Material::Paint => 0.85,
    }
}

fn engine_torque(car_spec: &model::car::CarSpec, rpm: f32) -> f32 {
    (car_spec.engine_max_torque_nm * model::car::torque_factor(&car_spec.engine_torque_curve, rpm))
        .min(car_spec.engine_power_watts / (rpm.max(IDLE_RPM) * std::f32::consts::TAU / 60.0))
}

fn drive_force(car_spec: &model::car::CarSpec, speed: f32, gear: u8) -> f32 {
    let g = &car_spec.gearbox;
    let ratio = if gear == 0 {
        g.reverse_ratio
    } else {
        g.ratios[(gear - 1) as usize]
    };
    let rpm = rpm(car_spec, speed, gear).max(IDLE_RPM);
    engine_torque(car_spec, rpm) * ratio * g.final_drive * DRIVETRAIN_EFFICIENCY
        / car_spec.rolling_radius
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

/// Full-throttle equilibrium on level ground in the best forward gear.
/// Uses the same spec torque, redline and road load as driving.
pub fn calculate_top_speed(car_spec: &model::car::CarSpec) -> f32 {
    (1..=gears(&car_spec.gearbox))
        .map(|gear| {
            let mut low = 0.0;
            let mut high = speed_at_rpm(car_spec, car_spec.redline_rpm, gear);
            for _ in 0..48 {
                let speed = (low + high) * 0.5;
                if drive_force(car_spec, speed, gear) > resistance_force(car_spec, speed) {
                    low = speed;
                } else {
                    high = speed;
                }
            }
            low
        })
        .fold(0.0, f32::max)
}

fn gears(gearbox: &model::car::Gearbox) -> u8 {
    gearbox.ratios.len() as u8
}
fn rpm(car_spec: &model::car::CarSpec, speed: f32, gear: u8) -> f32 {
    let gearbox = &car_spec.gearbox;
    let ratio = if gear == 0 {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[gear.saturating_sub(1).min(gears(gearbox) - 1) as usize]
    };
    let speed = if gear == 0 {
        speed.abs()
    } else {
        speed.max(0.0)
    };
    speed * 60.0 / (std::f32::consts::TAU * car_spec.rolling_radius) * ratio * gearbox.final_drive
}
fn speed_at_rpm(car_spec: &model::car::CarSpec, rpm: f32, gear: u8) -> f32 {
    let gearbox = &car_spec.gearbox;
    let ratio = if gear == 0 {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[gear.saturating_sub(1).min(gears(gearbox) - 1) as usize]
    };
    rpm * std::f32::consts::TAU * car_spec.rolling_radius / (60.0 * ratio * gearbox.final_drive)
}

// Base rate of normalized steering input change per second.
const STEERING_RATE: f32 = 9.0;
// Forward speed (m/s) at which released steering reaches its full return rate.
const STEERING_CENTERING_SPEED: f32 = 10.0;
// Front axle grip relative to the rear. With equal grip the limit yaw moment
// is exactly neutral (each axle's capacity scales with its static load), so a
// fully developed slide never straightens itself. The front deficit also
// balances the rear's reduced lateral force under engine braking when sliding
// grip reduces the entire friction circle, so lifting can unwind a slide.
const FRONT_LIMIT_GRIP: f32 = 0.86;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction {
    Drive,
    Reverse,
}

#[derive(Clone, Copy, Default)]
pub struct Controls {
    pub throttle: f32,
    pub brake: f32,
    pub steering: f32,
    pub handbrake: bool,
}

pub struct Vehicle {
    pub spec: std::sync::Arc<model::car::CarSpec>,
    pub track: std::sync::Arc<world::locations::WorldLocation>,
    pub position: XYZ,
    pub velocity: XZ<f32>,
    pub speed: f32,
    pub heading: f32,
    pub gear: u8,
    pub manual: bool,
    direction: Direction,
    slip: f32,
    pub body_pitch: f32,
    pub body_roll: f32,
    pub suspension_pitch: f32,
    pub suspension_roll: f32,
    pub curb_bump: f32,
    pub engine_rpm: f32,
    pub engine_load: f32,
    pub turbo_spool: f32,      // normalized turbine speed / available boost
    turbo_boost: f32,          // normalized manifold boost
    pub wheel_phase: [f32; 2], // rear/front axles; cosmetic, never affects physics
    pub tire_scrub: f32,
    pub brake_lights: bool,
    brake_pressure: f32,
    steering: f32,
    yaw_rate: f32,
    shift_timer: f32,
    sound_shift_timer: f32,
    rev_match_timer: f32,
    fuel_cut: bool,
    acceleration: [f32; 2], // longitudinal/lateral, filtered for load transfer
    pitch_rate: f32,
    roll_rate: f32,
    suspension_rates: [f32; 2],
    vertical_speed: f32,
    suspension_initialized: bool,
    pub wheel_spin: f32,
    driven_wheel_omega: f32, // signed wheel angular speed, radians/second
    drivetrain_initialized: bool,
    axle_load: [f32; 2],
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
        velocity: XZ { x: 0.0, z: 0.0 },
        speed: 0.0,
        heading,
        gear: 1,
        manual: false,
        direction: Direction::Drive,
        slip: 0.0,
        body_pitch: 0.0,
        body_roll: 0.0,
        suspension_pitch: 0.0,
        suspension_roll: 0.0,
        curb_bump: 0.0,
        engine_rpm: IDLE_RPM,
        engine_load: 0.0,
        turbo_spool: 0.0,
        turbo_boost: 0.0,
        brake_pressure: 0.0,
        steering: 0.0,
        yaw_rate: 0.0,
        wheel_phase: [0.0; 2],
        tire_scrub: 0.0,
        brake_lights: false,
        shift_timer: 0.0,
        sound_shift_timer: 0.0,
        rev_match_timer: 0.0,
        fuel_cut: false,
        acceleration: [0.0; 2],
        pitch_rate: 0.0,
        roll_rate: 0.0,
        suspension_rates: [0.0; 2],
        vertical_speed: 0.0,
        suspension_initialized: false,
        wheel_spin: 0.0,
        driven_wheel_omega: 0.0,
        drivetrain_initialized: false,
        axle_load: [0.0; 2],
        force_telemetry: ForceTelemetry::default(),
    };
    let (pitch, roll) = ground_angles(&car);
    car.suspension_pitch = pitch;
    car.suspension_roll = roll;
    car
}

pub fn reset(vehicle: &mut Vehicle, position: XYZ, heading: f32) {
    let manual = vehicle.manual;
    *vehicle = on_track(
        position,
        heading,
        vehicle.spec.clone(),
        vehicle.track.clone(),
    );
    vehicle.manual = manual;
}

fn upshift_speed(vehicle: &Vehicle, gear: u8) -> f32 {
    let gearbox = &vehicle.spec.gearbox;
    speed_at_rpm(&vehicle.spec, gearbox.upshift_rpm, gear)
}
fn coupled_engine_rpm(vehicle: &Vehicle) -> f32 {
    if vehicle.gear == NEUTRAL_GEAR {
        return IDLE_RPM;
    }
    rpm(&vehicle.spec, vehicle.speed, vehicle.gear).max(IDLE_RPM)
}
fn brake_accel(vehicle: &Vehicle) -> f32 {
    vehicle.spec.brake_force / vehicle.spec.weight_kg
}

pub fn shift(vehicle: &mut Vehicle, up: bool) {
    let next = match (vehicle.gear, up) {
        (0, true) => NEUTRAL_GEAR,
        (NEUTRAL_GEAR, true) => 1,
        (NEUTRAL_GEAR, false) => 0,
        (1, false) => NEUTRAL_GEAR,
        (0, false) => 0,
        (gear, true) => (gear + 1).min(gears(&vehicle.spec.gearbox)),
        (gear, false) => gear - 1,
    };
    if next != vehicle.gear {
        let previous = vehicle.gear;
        vehicle.gear = next;
        vehicle.direction = if next == 0 {
            Direction::Reverse
        } else {
            Direction::Drive
        };
        let launching = previous == NEUTRAL_GEAR && matches!(next, 0 | 1);
        vehicle.shift_timer = if launching { 0.0 } else { 0.16 };
        vehicle.sound_shift_timer = SOUND_SHIFT_SECONDS;
        begin_rev_match(vehicle, previous);
    }
}

fn begin_rev_match(vehicle: &mut Vehicle, previous: u8) {
    // Only a moving forward downshift needs a throttle blip. Neutral,
    // reverse and shifts below idle should not rev the engine.
    vehicle.rev_match_timer = if vehicle.gear > 0
        && vehicle.gear < previous
        && previous <= gears(&vehicle.spec.gearbox)
        && coupled_engine_rpm(vehicle) > vehicle.engine_rpm + 100.0
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
    let spec = vehicle.spec.clone();
    let reversing = vehicle.direction == Direction::Reverse;
    let direction = if reversing { -1.0 } else { 1.0 };
    let brake_accel = brake_accel(vehicle);
    let mut impact: f32 = 0.0;

    for _ in 0..steps {
        let previous_gear = vehicle.gear;
        refresh_speed(vehicle);
        if previous_gear != vehicle.gear {
            vehicle.shift_timer = 0.16;
            vehicle.sound_shift_timer = SOUND_SHIFT_SECONDS;
        }
        vehicle.shift_timer = (vehicle.shift_timer - seconds).max(0.0);
        vehicle.rev_match_timer = (vehicle.rev_match_timer - seconds).max(0.0);
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
        let forward = XZ {
            x: vehicle.heading.sin(),
            z: vehicle.heading.cos(),
        };
        let right = XZ {
            x: vehicle.heading.cos(),
            z: -vehicle.heading.sin(),
        };
        let gradient = physics::vehicle_location::ground_gradient(&vehicle.track, vehicle.position);
        // Horizontal component of gravity projected onto y = h(x, z).
        // Velocity is stored in XZ, so divide by 1 + |gradient|².
        let gravity =
            util::geometry::xz_map(gradient, |g| -9.81 * g / (1.0 + dot(gradient, gradient)));
        let longitudinal = dot(vehicle.velocity, forward);
        let lateral = dot(vehicle.velocity, right);
        let a = spec.wheelbase * (1.0 - spec.front_weight);
        let b = spec.wheelbase * spec.front_weight;
        let delta = -steering_angle(vehicle);
        let (sin, cos) = delta.sin_cos();

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
            - 1.3 * omega * vehicle.vertical_speed)
            .clamp(0.0, 49.05);
        vehicle.vertical_speed += (support - 9.81) * seconds;
        vehicle.position.y += vehicle.vertical_speed * seconds;
        let transfer = spec.weight_kg * vehicle.acceleration[0] * spec.cg_height / spec.wheelbase;
        let total_load = spec.weight_kg * support;
        let front_load = (total_load * spec.front_weight - transfer).clamp(0.0, total_load);
        vehicle.axle_load = [front_load, total_load - front_load];
        let capacities = [0, 1].map(|axle| {
            // Outside tires gain less grip than inside tires lose. This
            // introduces lateral weight-transfer sensitivity without a full
            // four-wheel rigid-body solver.
            let load = vehicle.axle_load[axle];
            let share = if axle == 0 {
                spec.front_weight
            } else {
                1.0 - spec.front_weight
            };
            let transfer = spec.weight_kg * vehicle.acceleration[1].abs() * spec.cg_height
                / spec.track_width
                * share;
            let imbalance = (2.0 * transfer / load.max(1.0)).clamp(0.0, 1.0);

            let loc = &vehicle.track;
            let position = vehicle.position;

            let reference = position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
            let deck = world::map_details::nearby_decks(&loc.details, util::geometry::xz(position))
                .find(|deck| {
                    world::map_details::height_on(deck, util::geometry::xz(position))
                        .is_some_and(|h| (h - reference).abs() < 1.5)
                });

            let material = if let Some(deck) = deck {
                model::map_details::road_material(deck.surface)
            } else {
                world::street_surface::world_location_triangle_at(
                    loc,
                    util::geometry::xz(position),
                    true,
                )
                .map_or(Material::Asphalt, |triangle| triangle.material)
            };
            let grip = friction_coefficient(material);

            let _hack = 1.5;

            _hack
                * grip
                * load
                * (1.0 - 0.18 * imbalance * imbalance)
                * if axle == 0 { FRONT_LIMIT_GRIP } else { 1.0 }
        });
        let throttle = controls.throttle.clamp(0.0, 1.0);
        update_fuel_cut(vehicle, vehicle.engine_rpm);
        update_turbo(vehicle, seconds, throttle);
        let drive_bias = spec.drive_bias;
        let front_speed = longitudinal * cos + (lateral + vehicle.yaw_rate * a) * sin;
        let driven_speed = front_speed * drive_bias + longitudinal * (1.0 - drive_bias);
        if !vehicle.drivetrain_initialized {
            vehicle.driven_wheel_omega = driven_speed / spec.rolling_radius;
            if vehicle.gear != NEUTRAL_GEAR {
                vehicle.engine_rpm = vehicle
                    .engine_rpm
                    .max(rpm(&spec, driven_speed, vehicle.gear));
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
        let brake = vehicle.brake_pressure * brake_accel * spec.weight_kg;
        let contacts = [0, 1].map(|axle| {
            let capacity = capacities[axle];
            let drive_share = if axle == 0 {
                drive_bias
            } else {
                1.0 - drive_bias
            };
            let wheel_speed = if axle == 0 { front_speed } else { longitudinal };
            let longitudinal_slip = if drive_share > 0.0 && drivetrain.is_some() {
                vehicle.driven_wheel_omega * spec.rolling_radius - wheel_speed
            } else {
                0.0
            };
            physics::tire::Contact {
                offset: if axle == 0 { a } else { -b },
                sin: if axle == 0 { sin } else { 0.0 },
                cos: if axle == 0 { cos } else { 1.0 },
                capacity,
                drive_force: drive * drive_share,
                longitudinal_slip,
                brake_force: brake
                    * if axle == 0 {
                        spec.brake_bias
                    } else {
                        1.0 - spec.brake_bias
                    }
                    + if axle == 1 && controls.handbrake {
                        spec.weight_kg * 5.5
                    } else {
                        0.0
                    },
                corner_stiffness: spec.corner_stiffness[axle],
                locked: axle == 1 && controls.handbrake,
            }
        });
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
        let motion = physics::tire::Motion {
            longitudinal,
            lateral,
            yaw_rate: vehicle.yaw_rate,
        };
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
            vehicle.engine_rpm = rotation.engine_rpm.max(IDLE_RPM);
            vehicle.driven_wheel_omega = rotation.wheel_omega * direction;
        } else {
            update_free_engine(vehicle, throttle, seconds);
            vehicle.driven_wheel_omega = driven_speed / spec.rolling_radius;
        }
        update_fuel_cut(vehicle, vehicle.engine_rpm);
        let slip_speed = vehicle.driven_wheel_omega * spec.rolling_radius - driven_speed;
        vehicle.wheel_spin = (slip_speed.abs() / 8.0).clamp(0.0, 1.0);
        vehicle.yaw_rate = solved.motion.yaw_rate;
        let resistance = resistance_force(&spec, longitudinal) / spec.weight_kg * seconds;
        let new_longitudinal = solved.motion.longitudinal.signum()
            * (solved.motion.longitudinal.abs() - resistance).max(0.0);
        vehicle.force_telemetry = ForceTelemetry {
            drive_force_n: tire_force,
            resistance_force_n: (solved.motion.longitudinal - new_longitudinal).abs()
                * spec.weight_kg
                / seconds,
            torque_nm,
        };
        let new_lateral = solved.motion.lateral;
        vehicle.velocity = XZ {
            x: forward.x * new_longitudinal + right.x * new_lateral,
            z: forward.z * new_longitudinal + right.z * new_lateral,
        };
        let blend = 1.0 - (-seconds * 12.0).exp();
        // Body motion follows actual chassis acceleration. In particular,
        // static contact balancing gravity must not produce brake dive.
        let ax = (new_longitudinal - longitudinal) / seconds;
        let ay = (new_lateral - lateral) / seconds;
        vehicle.acceleration[0] += (ax - vehicle.acceleration[0]) * blend;
        vehicle.acceleration[1] += (ay - vehicle.acceleration[1]) * blend;
        update_body_motion(vehicle, seconds);
        vehicle.heading =
            (vehicle.heading + vehicle.yaw_rate * seconds).rem_euclid(std::f32::consts::TAU);
        let previous_position = vehicle.position;
        vehicle.position.x += vehicle.velocity.x * seconds;
        vehicle.position.z += vehicle.velocity.z * seconds;
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
        if !controls.handbrake {
            vehicle.wheel_phase[0] = (vehicle.wheel_phase[0]
                + spin
                + if spec.drive_bias < 1.0 {
                    spin_slip
                } else {
                    0.0
                })
            .rem_euclid(std::f32::consts::TAU);
        }
        let chassis_height = vehicle.position.y;
        impact = impact.max(resolve_road(vehicle));
        vehicle.position.y = chassis_height;
        vehicle.curb_bump *= (-seconds * 12.0).exp();
        if world::street_surface::has_compiled_roads(&vehicle.track)
            && world::transport::deck_height(
                &vehicle.track,
                util::geometry::xz(vehicle.position),
                vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
            )
            .is_none_or(|h| {
                (h - (vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT)).abs()
                    > 1.
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
                let across = dot(vehicle.velocity, normal);
                let loss = (across.abs() * 0.08).min(1.2) * (rise / 0.09);
                for (velocity, normal) in
                    util::geometry::xz_iter_mut(&mut vehicle.velocity).zip(normal)
                {
                    *velocity -= normal * across.signum() * loss;
                }
                vehicle.curb_bump = vehicle
                    .curb_bump
                    .max(0.03 * (rise / 0.09) * (across.abs() / 6.0).min(1.0));
            }
        }
    }

    refresh_speed(vehicle);
    update_engine_feedback(vehicle, controls, dt.as_secs_f32());
    let motion = vehicle.velocity.x.hypot(vehicle.velocity.z);
    let sliding = ((vehicle.slip.abs() - 0.8) / 3.0).clamp(0.0, 1.0);
    let braking = if controls.handbrake { 0.8 } else { 0.0 };
    vehicle.tire_scrub =
        (sliding.max(braking) * ((motion - 1.5) / 2.5).clamp(0.0, 1.0)).max(vehicle.wheel_spin);
    impact
}

fn ride_height(vehicle: &Vehicle) -> f32 {
    world::transport::drive_height_near(
        &vehicle.track,
        util::geometry::xz(vehicle.position),
        vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
    ) + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT
}

fn engine_inertia(vehicle: &Vehicle) -> f32 {
    0.12 + vehicle.spec.engine_max_torque_nm * 0.0006
}

fn engine_net_torque(vehicle: &Vehicle, throttle: f32) -> f32 {
    let spec = &vehicle.spec;
    let revs = ((vehicle.engine_rpm - IDLE_RPM) / (spec.redline_rpm - IDLE_RPM)).max(0.0);
    let drag = spec.engine_max_torque_nm * (0.10 + 0.16 * revs + 0.24 * revs * revs);
    let throttle = if vehicle.shift_timer > 0.0 && vehicle.gear != NEUTRAL_GEAR {
        0.85 * vehicle.rev_match_timer / REV_MATCH_SECONDS
    } else {
        throttle
    };
    let combustion = if vehicle.fuel_cut {
        0.0
    } else {
        throttle * (engine_torque(spec, vehicle.engine_rpm) * DRIVETRAIN_EFFICIENCY + drag)
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
    let ratio = if vehicle.gear == 0 {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[(vehicle.gear - 1) as usize]
    } * gearbox.final_drive;
    // Automatic clutch opens near idle to avoid stalling. Its torque capacity
    // increases with engine speed; there is no launch RPM target.
    let engagement = if rpm(spec, speed, vehicle.gear) > IDLE_RPM + 100.0 {
        1.0
    } else {
        ((vehicle.engine_rpm - IDLE_RPM) / 600.0).clamp(0.0, 1.0)
    };
    let direction = if vehicle.gear == 0 { -1.0 } else { 1.0 };
    Some(physics::powertrain::new(
        vehicle.engine_rpm,
        vehicle.driven_wheel_omega * direction,
        engine_net_torque(vehicle, throttle),
        engine_inertia(vehicle),
        ratio,
        spec.engine_max_torque_nm * 2.0 * engagement,
        dt,
    ))
}

fn update_turbo(vehicle: &mut Vehicle, dt: f32, throttle: f32) {
    let Some(tune) = &vehicle.spec.turbo else {
        vehicle.turbo_spool = 0.0;
        vehicle.turbo_boost = 0.0;
        return;
    };
    let loaded = vehicle.gear != NEUTRAL_GEAR && vehicle.shift_timer <= 0.0;
    let rpm = vehicle.engine_rpm;
    let exhaust = ((rpm - tune.spool_start_rpm) / (tune.full_boost_rpm - tune.spool_start_rpm))
        .clamp(0.0, 1.0);
    let target = if loaded {
        exhaust * throttle * throttle
    } else {
        0.0
    };
    let time = if target > vehicle.turbo_spool {
        tune.spool_seconds
    } else {
        tune.decay_seconds
    };
    vehicle.turbo_spool += (target - vehicle.turbo_spool) * (1.0 - (-dt / time).exp());
    // Closing the throttle vents manifold pressure while the turbine coasts.
    let boost_target = if loaded {
        vehicle.turbo_spool * throttle
    } else {
        0.0
    };
    vehicle.turbo_boost += (boost_target - vehicle.turbo_boost) * (1.0 - (-dt / 0.06).exp());
}

fn ground_angles(vehicle: &Vehicle) -> (f32, f32) {
    let (sin, cos) = vehicle.heading.sin_cos();
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
    let length = vehicle.spec.wheelbase;
    let width = vehicle.spec.track_width;
    (
        ((h(sin * length * 0.5, cos * length * 0.5) - h(-sin * length * 0.5, -cos * length * 0.5))
            / length)
            .atan()
            .clamp(-0.35, 0.35),
        ((h(cos * width * 0.5, -sin * width * 0.5) - h(-cos * width * 0.5, sin * width * 0.5))
            / width)
            .atan()
            .clamp(-0.25, 0.25),
    )
}

fn update_body_motion(vehicle: &mut Vehicle, dt: f32) {
    let spec = &vehicle.spec;
    let target_pitch = (vehicle.acceleration[0] * spec.cg_height / 80.0).clamp(-0.10, 0.10);
    let target_roll = (vehicle.acceleration[1] * spec.cg_height / 65.0).clamp(-0.12, 0.12);
    let omega = std::f32::consts::TAU * spec.suspension_hz;
    vehicle.pitch_rate += (omega * omega * (target_pitch - vehicle.body_pitch)
        - 1.4 * omega * vehicle.pitch_rate)
        * dt;
    vehicle.roll_rate +=
        (omega * omega * (target_roll - vehicle.body_roll) - 1.4 * omega * vehicle.roll_rate) * dt;
    vehicle.body_pitch += vehicle.pitch_rate * dt;
    vehicle.body_roll += vehicle.roll_rate * dt;
    // The sprung body follows terrain through damped angular springs.
    // Wheels follow the contact surface independently in the renderer.
    let (pitch, roll) = if vehicle.axle_load.iter().sum::<f32>() > 0.0 {
        ground_angles(vehicle)
    } else {
        (vehicle.suspension_pitch, vehicle.suspension_roll)
    };
    for (axis, target, angle) in [
        (0, pitch, &mut vehicle.suspension_pitch),
        (1, roll, &mut vehicle.suspension_roll),
    ] {
        let rate = &mut vehicle.suspension_rates[axis];
        *rate += (omega * omega * (target - *angle) - 1.4 * omega * *rate) * dt;
        *angle += *rate * dt;
    }
}

fn update_fuel_cut(vehicle: &mut Vehicle, rpm: f32) {
    // The same hysteresis applies whether the engine is free or coupled to
    // the wheels. Cutting combustion cannot prevent a mechanical overrev.
    if rpm >= vehicle.spec.redline_rpm {
        vehicle.fuel_cut = true;
    } else if rpm <= vehicle.spec.redline_rpm - 250.0 {
        vehicle.fuel_cut = false;
    }
}

fn update_free_engine(vehicle: &mut Vehicle, throttle: f32, dt: f32) {
    update_fuel_cut(vehicle, vehicle.engine_rpm);
    let rpm_rate = engine_net_torque(vehicle, throttle) / engine_inertia(vehicle) * 60.0
        / std::f32::consts::TAU;
    vehicle.engine_rpm = (vehicle.engine_rpm + rpm_rate * dt).max(IDLE_RPM);
    update_fuel_cut(vehicle, vehicle.engine_rpm);
}

fn update_engine_feedback(vehicle: &mut Vehicle, controls: Controls, dt: f32) {
    let throttle = controls.throttle.clamp(0.0, 1.0);

    vehicle.sound_shift_timer = (vehicle.sound_shift_timer - dt).max(0.0);

    let blip = vehicle.rev_match_timer / REV_MATCH_SECONDS;

    // An unloaded engine uses less sustained load; fuel cuts also reach
    // the audio layers so the limiter is audible as well as visible.
    let requested = if vehicle.fuel_cut {
        0.0
    } else if vehicle.gear == NEUTRAL_GEAR {
        throttle * 0.55
    } else {
        throttle
    };
    let shift_progress = 1.0 - vehicle.sound_shift_timer / SOUND_SHIFT_SECONDS;
    let shift_unload = 0.15 + 0.85 * shift_progress * shift_progress;
    // Feed the blip into both recorded load/coast layers and procedural
    // engine voices, without applying extra driving force at the wheels.
    let target_load = (requested * shift_unload + 0.12 * (1.0 - requested))
        .max(if vehicle.fuel_cut { 0.0 } else { 0.85 * blip })
        .clamp(0.0, 1.0);
    vehicle.engine_load += (target_load - vehicle.engine_load) * (dt * 10.0).clamp(0.0, 1.0);
}

fn update_steering(vehicle: &mut Vehicle, requested: f32, dt: f32) {
    let requested = requested.clamp(-1.0, 1.0);
    if requested == 0.0 {
        // Stationary wheels hold their angle. Forward motion progressively
        // restores self-centering, independently of deliberate countersteering.
        let motion = (vehicle.speed / STEERING_CENTERING_SPEED).clamp(0.0, 1.0);
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
    let rate = STEERING_RATE * 0.45 / (1.0 + vehicle.speed.abs() * 0.008);
    vehicle.steering += (requested - vehicle.steering).clamp(-rate * remaining, rate * remaining);
}

fn steering_angle(vehicle: &Vehicle) -> f32 {
    vehicle.steering.signum() * vehicle.steering.abs().powf(1.10) * 0.54
}

pub fn front_wheel_angle(vehicle: &Vehicle) -> f32 {
    -steering_angle(vehicle)
}

fn resolve_road(vehicle: &mut Vehicle) -> f32 {
    if world::street_surface::has_compiled_roads(&vehicle.track) {
        let city = &vehicle.track;
        return physics::vehicle_location::constrain(
            city,
            &mut vehicle.position,
            &mut vehicle.velocity,
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
            impact = impact.max(physics::vehicle_location::deflect(
                &mut vehicle.velocity,
                normal,
            ));
        }
    }
    vehicle.position.y = world::transport::drive_height_near(
        &vehicle.track,
        util::geometry::xz(vehicle.position),
        vehicle.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
    ) + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
    impact
}

fn refresh_speed(vehicle: &mut Vehicle) {
    vehicle.speed = dot(
        vehicle.velocity,
        XZ {
            x: vehicle.heading.sin(),
            z: vehicle.heading.cos(),
        },
    );
    vehicle.slip = dot(
        vehicle.velocity,
        XZ {
            x: vehicle.heading.cos(),
            z: -vehicle.heading.sin(),
        },
    );
    if vehicle.manual {
        return;
    }
    let previous = vehicle.gear;
    let gearbox = &vehicle.spec.gearbox;
    if vehicle.direction == Direction::Reverse {
        vehicle.gear = 0;
    } else {
        vehicle.gear = vehicle.gear.clamp(1, gears(gearbox));
        // Separate up/down RPM thresholds prevent hunting around a shift.
        while vehicle.gear < gears(gearbox) && vehicle.speed >= upshift_speed(vehicle, vehicle.gear)
        {
            vehicle.gear += 1;
        }
        while vehicle.gear > 1
            && rpm(&vehicle.spec, vehicle.speed, vehicle.gear) < gearbox.downshift_rpm
            && rpm(&vehicle.spec, vehicle.speed, vehicle.gear - 1) < gearbox.upshift_rpm * 0.95
        {
            vehicle.gear -= 1;
        }
    }
    if vehicle.gear != previous {
        begin_rev_match(vehicle, previous);
    }
}

fn dot(a: XZ<f32>, b: XZ<f32>) -> f32 {
    a.x * b.x + a.z * b.z
}

#[cfg(test)]
#[path = "tests/car.rs"]
mod tests;
