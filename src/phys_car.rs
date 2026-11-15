use crate::car_glb;
use crate::geometry;
use crate::geometry::{XYZ, XZ};
use crate::map_details;
use crate::map_details::RoadSurface;
use crate::phys_tire;
use crate::street_surface;
use crate::vehicle_location;
use crate::world;
use crate::world::Material;
use crate::world_transport;
use crate::{asset_car, phys_powertrain, track};
use std::time::Duration;

// Keep zero for reverse and 1..=N for forward ratios.
pub const NEUTRAL_GEAR: u8 = u8::MAX;
pub const IDLE_RPM: f32 = 900.0;
const REV_MATCH_SECONDS: f32 = 0.22;
const SOUND_SHIFT_SECONDS: f32 = 0.36;

const AIR_DENSITY_KG_M3: f32 = 1.225;

fn friction_coefficient_road(s: RoadSurface) -> f32 {
    match s {
        RoadSurface::Unpaved => 0.65,
        RoadSurface::Paving => 0.90,
        _ => 1.0,
    }
}

fn friction_coefficient_other(m: Material) -> f32 {
    match m {
        Material::BareGround => 0.65,
        Material::Paving => 0.90,
        _ => 1.,
    }
}

fn torque_factor(curve: &[asset_car::TorquePoint], rpm: f32) -> f32 {
    let upper = curve.partition_point(|point| point.rpm < rpm);
    if upper == 0 {
        return curve[0].torque;
    }
    if upper == curve.len() {
        return curve[upper - 1].torque;
    }
    let lower = &curve[upper - 1];
    let upper = &curve[upper];
    let fraction = (rpm - lower.rpm) / (upper.rpm - lower.rpm);
    lower.torque + (upper.torque - lower.torque) * fraction
}

// The physics assumes
// 90% drivetrain efficiency to calculate power at the wheels.
const DRIVETRAIN_EFFICIENCY: f32 = 0.90;

fn drive_force(car_spec: &asset_car::CarSpec, speed: f32, gear: u8) -> f32 {
    let g = &car_spec.gearbox;
    let ratio = g.ratios[(gear - 1) as usize];
    let rpm = rpm(g, speed, gear).max(IDLE_RPM);
    engine_torque(car_spec, rpm) * ratio * g.final_drive * DRIVETRAIN_EFFICIENCY / g.rolling_radius
}

/// Full-load crankshaft torque, bounded by the authored engine power.
fn engine_torque(car_spec: &asset_car::CarSpec, rpm: f32) -> f32 {
    (car_spec.engine_max_torque_nm * torque_factor(&car_spec.engine_torque_curve, rpm))
        .min(car_spec.engine_power_watts / (rpm.max(IDLE_RPM) * std::f32::consts::TAU / 60.0))
}

fn resistance_accel(car_spec: &asset_car::CarSpec, speed: f32) -> f32 {
    let drag_force = 0.5
        * AIR_DENSITY_KG_M3
        * car_spec.drag_coefficient
        * car_spec.frontal_area_m2
        * speed
        * speed;
    0.25 + drag_force / car_spec.weight_kg
}

/// Full-throttle equilibrium on level ground in the best forward gear.
/// Uses the same spec torque, redline and road load as driving.
pub fn calculate_top_speed(car_spec: &asset_car::CarSpec) -> f32 {
    (1..=gears(&car_spec.gearbox))
        .map(|gear| {
            let mut low = 0.0;
            let mut high = speed_at_rpm(&car_spec.gearbox, car_spec.redline_rpm, gear);
            for _ in 0..48 {
                let speed = (low + high) * 0.5;
                if drive_force(car_spec, speed, gear)
                    > car_spec.weight_kg * resistance_accel(car_spec, speed)
                {
                    low = speed;
                } else {
                    high = speed;
                }
            }
            low
        })
        .fold(0.0, f32::max)
}

fn gears(gearbox: &asset_car::Gearbox) -> u8 {
    gearbox.ratios.len() as u8
}
fn rpm(gearbox: &asset_car::Gearbox, speed: f32, gear: u8) -> f32 {
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
    speed * 60.0 / (std::f32::consts::TAU * gearbox.rolling_radius) * ratio * gearbox.final_drive
}
fn speed_at_rpm(gearbox: &asset_car::Gearbox, rpm: f32, gear: u8) -> f32 {
    let ratio = if gear == 0 {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[gear.saturating_sub(1).min(gears(gearbox) - 1) as usize]
    };
    rpm * std::f32::consts::TAU * gearbox.rolling_radius / (60.0 * ratio * gearbox.final_drive)
}

// Base rate of normalized steering input change per second.
const STEERING_RATE: f32 = 9.0;
// Forward speed (m/s) at which released steering reaches its full return rate.
const STEERING_CENTERING_SPEED: f32 = 10.0;
// Below this speed the full lock is available for parking and tight turns.
const STEERING_LOCK_SPEED: f32 = 6.0;
// Speed range over which the lock fades from full to its minimum fraction.
const STEERING_LOCK_FADE_SPEED: f32 = 20.0;
// Fraction of full lock available once the fade is complete. It keeps a held
// input from asking for several times the slip angle the tires can use.
const STEERING_HIGH_SPEED_LOCK: f32 = 0.35;
// Front axle grip relative to the rear. With equal grip the limit yaw moment
// is exactly neutral (each axle's capacity scales with its static load), so a
// fully developed slide never straightens itself. A small front deficit makes
// the front saturate first, so the car pushes wide instead of spinning.
const FRONT_LIMIT_GRIP: f32 = 0.94;

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
    pub spec: std::sync::Arc<asset_car::CarSpec>,
    pub track: std::sync::Arc<track::WorldLocation>,
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
}

pub fn on_track(
    position: XYZ,
    heading: f32,
    spec: std::sync::Arc<asset_car::CarSpec>,
    track: std::sync::Arc<track::WorldLocation>,
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
    speed_at_rpm(gearbox, gearbox.upshift_rpm, gear)
}
fn coupled_engine_rpm(vehicle: &Vehicle) -> f32 {
    if vehicle.gear == NEUTRAL_GEAR {
        return IDLE_RPM;
    }
    rpm(&vehicle.spec.gearbox, vehicle.speed, vehicle.gear).max(IDLE_RPM)
}
fn brake_accel(vehicle: &Vehicle) -> f32 {
    vehicle.spec.brake_force / vehicle.spec.weight_kg
}

#[cfg(test)]
fn toggle_direction(vehicle: &mut Vehicle) -> bool {
    // Check total motion, not just longitudinal speed, so slides cannot shift.
    if vehicle.velocity.x.hypot(vehicle.velocity.z) > 0.5 {
        return false;
    }
    vehicle.direction = match vehicle.direction {
        Direction::Drive => Direction::Reverse,
        Direction::Reverse => Direction::Drive,
    };
    refresh_speed(vehicle);
    true
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

#[cfg(test)]
fn gear_label(vehicle: &Vehicle) -> String {
    if vehicle.gear == NEUTRAL_GEAR {
        return "N".into();
    }
    if vehicle.direction == Direction::Reverse {
        "R".into()
    } else {
        vehicle.gear.to_string()
    }
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
        let gradient = world::ground_gradient(&vehicle.track, vehicle.position);
        // Horizontal component of gravity projected onto y = h(x, z).
        // Velocity is stored in XZ, so divide by 1 + |gradient|².
        let gravity = geometry::xz_map(gradient, |g| -9.81 * g / (1.0 + dot(gradient, gradient)));
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

            let reference = position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT;
            let deck =
                map_details::nearby_decks(&loc.details, geometry::xz(position)).find(|deck| {
                    map_details::height_on(deck, geometry::xz(position))
                        .is_some_and(|h| (h - reference).abs() < 1.5)
                });

            let grip = if let Some(deck) = deck {
                friction_coefficient_road(deck.surface)
            } else {
                street_surface::world_location_triangle_at(loc, geometry::xz(position), true)
                    .map_or(1., |triangle| friction_coefficient_other(triangle.material))
            };

            let _hack = 1.5;

            _hack
                * grip
                * load
                * (1.0 - 0.18 * imbalance * imbalance)
                * if axle == 0 { FRONT_LIMIT_GRIP } else { 1.0 }
        });
        let rear_v = lateral - vehicle.yaw_rate * b;
        let throttle = controls.throttle.clamp(0.0, 1.0);
        update_fuel_cut(vehicle, vehicle.engine_rpm);
        update_turbo(vehicle, seconds, throttle);
        let drive_bias = spec.drive_bias;
        let front_speed = longitudinal * cos + (lateral + vehicle.yaw_rate * a) * sin;
        let driven_speed = front_speed * drive_bias + longitudinal * (1.0 - drive_bias);
        if !vehicle.drivetrain_initialized {
            vehicle.driven_wheel_omega = driven_speed / spec.gearbox.rolling_radius;
            if vehicle.gear != NEUTRAL_GEAR {
                vehicle.engine_rpm =
                    vehicle
                        .engine_rpm
                        .max(rpm(&spec.gearbox, driven_speed, vehicle.gear));
            }
            vehicle.drivetrain_initialized = true;
        }
        let drivetrain = drivetrain_step(vehicle, throttle, seconds, driven_speed);
        let drive = drivetrain.as_ref().map_or(0.0, |step| {
            phys_powertrain::road_force(
                step,
                driven_speed * direction,
                spec.weight_kg,
                spec.gearbox.rolling_radius,
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
            let spinning = drive.abs() * drive_share > capacity;
            let drive_limit = if spinning {
                let sideways = (vehicle.slip.abs() / 2.0).clamp(0.0, 1.0);
                if let Some([baseline, reduction, slip_angle]) =
                    spec.rear_combined_slip.filter(|_| axle == 1)
                {
                    let angle = rear_v.abs().atan2(longitudinal.abs().max(2.5));
                    capacity * (baseline - reduction * (angle / slip_angle).clamp(0.0, 1.0))
                } else {
                    capacity * (0.97 - 0.05 * sideways)
                }
            } else {
                capacity
            };
            phys_tire::Contact {
                offset: if axle == 0 { a } else { -b },
                sin: if axle == 0 { sin } else { 0.0 },
                cos: if axle == 0 { cos } else { 1.0 },
                capacity,
                drive_force: drive * drive_share,
                drive_limit,
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
        let motion = phys_tire::Motion {
            longitudinal,
            lateral,
            yaw_rate: vehicle.yaw_rate,
        };
        let solved = phys_tire::solve(
            motion,
            phys_tire::Motion {
                longitudinal: longitudinal + dot(gravity, forward) * seconds,
                lateral: lateral + dot(gravity, right) * seconds,
                ..motion
            },
            contacts,
            spec.weight_kg,
            spec.yaw_inertia,
            seconds,
        );
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
            let rotation = phys_powertrain::finish(
                &step,
                reaction,
                spec.gearbox.rolling_radius,
                driven_brake_force,
                rolling_speed * direction,
            );
            vehicle.engine_rpm = rotation.engine_rpm.max(IDLE_RPM);
            vehicle.driven_wheel_omega = rotation.wheel_omega * direction;
        } else {
            update_free_engine(vehicle, throttle, seconds);
            vehicle.driven_wheel_omega = driven_speed / spec.gearbox.rolling_radius;
        }
        update_fuel_cut(vehicle, vehicle.engine_rpm);
        let slip_speed = vehicle.driven_wheel_omega * spec.gearbox.rolling_radius - driven_speed;
        vehicle.wheel_spin = (slip_speed.abs() / 8.0).clamp(0.0, 1.0);
        vehicle.yaw_rate = solved.motion.yaw_rate;
        let resistance = resistance_accel(&spec, longitudinal) * seconds;
        let new_longitudinal = solved.motion.longitudinal.signum()
            * (solved.motion.longitudinal.abs() - resistance).max(0.0);
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
        let spin = longitudinal * seconds / spec.gearbox.rolling_radius;
        let spin_slip =
            (vehicle.driven_wheel_omega - longitudinal / spec.gearbox.rolling_radius) * seconds;
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
        if street_surface::has_compiled_roads(&vehicle.track)
            && world_transport::deck_height(
                &vehicle.track,
                geometry::xz(vehicle.position),
                vehicle.position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT,
            )
            .is_none_or(|h| {
                (h - (vehicle.position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT)).abs() > 1.
            })
        {
            let city = &vehicle.track;
            if let Some((normal, rise)) = vehicle_location::crossed_curb(
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
                for (velocity, normal) in geometry::xz_iter_mut(&mut vehicle.velocity).zip(normal) {
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
    world_transport::drive_height_near(
        &vehicle.track,
        geometry::xz(vehicle.position),
        vehicle.position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT,
    ) + vehicle_location::VEHICLE_ORIGIN_HEIGHT
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
) -> Option<phys_powertrain::Step> {
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
    let engagement = if rpm(gearbox, speed, vehicle.gear) > IDLE_RPM + 100.0 {
        1.0
    } else {
        ((vehicle.engine_rpm - IDLE_RPM) / 600.0).clamp(0.0, 1.0)
    };
    let direction = if vehicle.gear == 0 { -1.0 } else { 1.0 };
    Some(phys_powertrain::new(
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
        world_transport::drive_height_near(
            &vehicle.track,
            geometry::xz(p),
            vehicle.position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT,
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
    // The lock available at this speed bounds both the held angle and the
    // new request, so speeding up mid-corner also unwinds the wheels.
    let lock = steering_lock_fraction(vehicle);
    vehicle.steering = vehicle.steering.clamp(-lock, lock);
    let requested = requested.clamp(-lock, lock);
    if requested == 0.0 {
        // Stationary wheels hold their angle. Forward motion progressively
        // restores self-centering, independently of deliberate countersteering.
        let motion = (vehicle.speed / STEERING_CENTERING_SPEED).clamp(0.0, 1.0);
        let step = STEERING_RATE * 0.45 * motion * dt;
        vehicle.steering -= vehicle.steering.clamp(-step, step);
        return;
    }
    // Cross the centre quickly when countersteering, then build and release
    // the turn at the same speed-dependent rate. Release no faster than
    // build stops the wheel snapping back when a key is let go.
    let reversing = requested * vehicle.steering < 0.0;
    let target = if reversing { 0.0 } else { requested };
    let rate = if reversing {
        STEERING_RATE * 0.85
    } else {
        STEERING_RATE * 0.45 / (1.0 + vehicle.speed.abs() * 0.008)
    };
    vehicle.steering += (target - vehicle.steering).clamp(-rate * dt, rate * dt);
}

fn steering_lock_fraction(vehicle: &Vehicle) -> f32 {
    let fade =
        ((vehicle.speed.abs() - STEERING_LOCK_SPEED) / STEERING_LOCK_FADE_SPEED).clamp(0.0, 1.0);
    1.0 - (1.0 - STEERING_HIGH_SPEED_LOCK) * fade
}

fn steering_angle(vehicle: &Vehicle) -> f32 {
    vehicle.steering.signum() * vehicle.steering.abs().powf(1.10) * 0.54
}

pub fn front_wheel_angle(vehicle: &Vehicle) -> f32 {
    -steering_angle(vehicle)
}

fn resolve_road(vehicle: &mut Vehicle) -> f32 {
    if street_surface::has_compiled_roads(&vehicle.track) {
        let city = &vehicle.track;
        return vehicle_location::constrain(
            city,
            &mut vehicle.position,
            &mut vehicle.velocity,
            car_glb::height(&vehicle.spec.design),
        );
    }
    let bounds = world::terrain_bounds(&vehicle.track);
    let mut impact = 0.0_f32;
    for (axis, (coordinate, min, max)) in [
        (0, bounds.min.x, bounds.max.x),
        (2, bounds.min.z, bounds.max.z),
    ]
    .into_iter()
    .enumerate()
    {
        let limit = [min + track::CAR_RADIUS, max - track::CAR_RADIUS];
        let clamped =
            geometry::xyz_component(vehicle.position, coordinate).clamp(limit[0], limit[1]);
        let excess = geometry::xyz_component(vehicle.position, coordinate) - clamped;
        if excess.abs() > 0.0001 {
            *geometry::xyz_component_mut(&mut vehicle.position, coordinate) = clamped;
            let mut normal = XZ { x: 0.0, z: 0.0 };
            *geometry::xz_component_mut(&mut normal, axis) = excess.signum();
            impact = impact.max(vehicle_location::deflect(&mut vehicle.velocity, normal));
        }
    }
    vehicle.position.y = world_transport::drive_height_near(
        &vehicle.track,
        geometry::xz(vehicle.position),
        vehicle.position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT,
    ) + vehicle_location::VEHICLE_ORIGIN_HEIGHT;
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
            && rpm(gearbox, vehicle.speed, vehicle.gear) < gearbox.downshift_rpm
            && rpm(gearbox, vehicle.speed, vehicle.gear - 1) < gearbox.upshift_rpm * 0.95
        {
            vehicle.gear -= 1;
        }
    }
    if vehicle.gear != previous {
        begin_rev_match(vehicle, previous);
    }
}

#[cfg(test)]
fn collide(vehicle: &mut Vehicle, other: &mut Vehicle) -> f32 {
    debug_assert!(
        std::sync::Arc::ptr_eq(&vehicle.track, &other.track),
        "vehicle contact requires the same route"
    );
    let dx = vehicle.position.x - other.position.x;
    let dz = vehicle.position.z - other.position.z;
    let distance = dx.hypot(dz);
    let separation = track::CAR_RADIUS * 2.0;
    if distance >= separation {
        return 0.0;
    }
    let normal = if distance < 0.001 {
        XZ { x: 1.0, z: 0.0 }
    } else {
        XZ {
            x: dx / distance,
            z: dz / distance,
        }
    };
    let inverse_mass = 1.0 / vehicle.spec.weight_kg;
    let other_inverse_mass = 1.0 / other.spec.weight_kg;
    let inverse_sum = inverse_mass + other_inverse_mass;
    let penetration = separation - distance;
    let relative = dot(
        XZ {
            x: vehicle.velocity.x - other.velocity.x,
            z: vehicle.velocity.z - other.velocity.z,
        },
        (normal).into(),
    );
    for (axis, component) in [normal.x, normal.z].iter().enumerate() {
        let position_axis = if axis == 0 { 0 } else { 2 };
        *geometry::xyz_component_mut(&mut vehicle.position, position_axis) +=
            component * penetration * inverse_mass / inverse_sum;
        *geometry::xyz_component_mut(&mut other.position, position_axis) -=
            component * penetration * other_inverse_mass / inverse_sum;
        if relative < 0.0 {
            let impulse = -relative * 1.1 / inverse_sum;
            *geometry::xz_component_mut(&mut vehicle.velocity, axis) +=
                component * impulse * inverse_mass;
            *geometry::xz_component_mut(&mut other.velocity, axis) -=
                component * impulse * other_inverse_mass;
        }
    }
    let heights = [vehicle.position.y, other.position.y];
    let wall_impact = resolve_road(vehicle).max(resolve_road(other));
    if vehicle.suspension_initialized {
        vehicle.position.y = heights[0];
    }
    if other.suspension_initialized {
        other.position.y = heights[1];
    }
    refresh_speed(vehicle);
    refresh_speed(other);
    (-relative).max(wall_impact).max(0.0)
}

fn dot(a: XZ<f32>, b: XZ<f32>) -> f32 {
    a.x * b.x + a.z * b.z
}

#[cfg(test)]
mod tests {
    use crate::geometry;
    use crate::phys_car;
    use crate::street_surface;
    use crate::track;
    use crate::vehicle_location;
    use crate::world_preparation;
    use crate::world_roads;

    use super::*;
    use crate::codec_road_rises;
    use crate::test_support;
    use crate::world;

    fn new(position: XYZ, heading: f32, spec: std::sync::Arc<asset_car::CarSpec>) -> Vehicle {
        phys_car::on_track(position, heading, spec, test_support::compact_track())
    }

    #[test]
    fn mapped_speed_table_moves_the_suspension_and_settles() {
        let run = |hz: u32| {
            let folder = tempfile::tempdir().unwrap();
            let assets = test_support::stored_location(folder.path(), false);
            codec_road_rises::write(
                &mut std::fs::File::create(folder.path().join("road-rises.txt")).unwrap(),
                &test_support::speed_table_rises(),
            )
            .unwrap();
            let track =
                std::sync::Arc::new(world_preparation::world_location_load(&assets).unwrap());
            assert!(
                (vehicle_location::drive_height(&track, XZ { x: 14., z: 0. }) - 7.1).abs()
                    < 0.00001
            );
            let mut car = phys_car::on_track(
                geometry::at_height(XZ { x: 11.5, z: 0. }, 7.48),
                std::f32::consts::FRAC_PI_2,
                test_support::car(),
                track,
            );
            car.manual = true;
            car.gear = NEUTRAL_GEAR;
            car.velocity = XZ { x: 5., z: 0. };
            let mut peak_height: f32 = 0.;
            let mut peak_pitch: f32 = 0.;
            for _ in 0..hz * 3 {
                if car.position.x > 18. {
                    car.velocity = XZ::default();
                }
                phys_car::update(
                    &mut car,
                    Duration::from_secs_f64(1. / hz as f64),
                    Controls::default(),
                );
                peak_height = peak_height.max(car.position.y - 7.48);
                peak_pitch = peak_pitch.max(car.suspension_pitch.abs());
                assert!(geometry::iter(&(car.position)).all(|v| v.is_finite()));
            }
            assert!(
                peak_height > 0.04 && peak_height < 0.3,
                "heave {peak_height}"
            );
            assert!(peak_pitch > 0.01 && peak_pitch < 0.2, "pitch {peak_pitch}");
            assert!((car.position.y - phys_car::ride_height(&(car))).abs() < 0.01);
            peak_height
        };
        assert!((run(60) - run(240)).abs() < 0.02);
    }

    #[test]
    fn aerodynamic_drag_uses_coefficient_area_and_mass() {
        let mut spec = test_support::car_spec().clone();
        spec.drag_coefficient = 0.3;
        spec.frontal_area_m2 = 2.0;
        spec.weight_kg = 1000.0;
        // At 20 m/s this gives 147 N of aerodynamic drag, plus rolling resistance.
        let deceleration =
            phys_car::resistance_accel(&spec, 20.0) - phys_car::resistance_accel(&spec, 0.0);
        assert!((deceleration - 0.147).abs() < 0.000001);
        assert_eq!(
            phys_car::resistance_accel(&(spec), -20.0),
            phys_car::resistance_accel(&(spec), 20.0)
        );
        spec.weight_kg *= 2.0;
        let heavier =
            phys_car::resistance_accel(&spec, 20.0) - phys_car::resistance_accel(&spec, 0.0);
        assert!((heavier * 2.0 - deceleration).abs() < 0.000001);
        spec.drag_coefficient *= 2.0;
        let more_drag =
            phys_car::resistance_accel(&spec, 20.0) - phys_car::resistance_accel(&spec, 0.0);
        assert!((more_drag - deceleration).abs() < 0.000001);
        spec.frontal_area_m2 *= 2.0;
        let larger_area =
            phys_car::resistance_accel(&spec, 20.0) - phys_car::resistance_accel(&spec, 0.0);
        assert!((larger_area - 2.0 * deceleration).abs() < 0.000001);
    }

    #[test]
    fn calculated_top_speed_balances_power_and_road_load() {
        let mut spec = test_support::car_spec().clone();
        spec.turbo = None;
        spec.engine_max_torque_nm = 10_000.0;
        spec.gearbox.final_drive = 1.0;
        let speed = phys_car::calculate_top_speed(&spec);
        let road_power = spec.weight_kg * phys_car::resistance_accel(&spec, speed) * speed;
        assert!((road_power - spec.engine_power_watts * DRIVETRAIN_EFFICIENCY).abs() < 1.0);
        spec.engine_power_watts *= 2.0;
        let more_power = phys_car::calculate_top_speed(&spec);
        assert!(more_power > speed);
        spec.drag_coefficient *= 2.0;
        assert!(phys_car::calculate_top_speed(&(spec)) < more_power);
        let before_weight_change = phys_car::calculate_top_speed(&spec);
        spec.weight_kg *= 2.0;
        assert!(phys_car::calculate_top_speed(&(spec)) < before_weight_change);
    }

    fn hill_car(uphill: bool) -> Vehicle {
        let route = track::try_track(&test_support::authored_id()).unwrap();
        let road = (0..200)
            .map(|i| track::at_distance(&route, i as f32 * 10.0))
            .max_by(|a, b| {
                dot(world::ground_gradient(&route, a.point), a.tangent)
                    .total_cmp(&dot(world::ground_gradient(&route, b.point), b.tangent))
            })
            .unwrap();
        assert!(dot(world::ground_gradient(&route, road.point), road.tangent) > 0.04);
        phys_car::on_track(
            vehicle_location::vehicle_position(road, 0.0),
            world::heading(road) + if uphill { 0.0 } else { std::f32::consts::PI },
            test_support::car(),
            route,
        )
    }

    #[test]
    fn manual_shifts_hold_gears_and_pass_through_neutral() {
        let mut car = new(
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            test_support::car(),
        );
        car.manual = true;
        car.velocity = XZ { x: 0.0, z: 40.0 };
        phys_car::refresh_speed(&mut car);
        assert_eq!(car.gear, 1);
        phys_car::shift(&mut car, true);
        phys_car::refresh_speed(&mut car);
        assert_eq!(car.gear, 2);
        car.velocity = XZ { x: 0.0, z: 0.0 };
        phys_car::refresh_speed(&mut car);
        assert_eq!(car.gear, 2);
        phys_car::shift(&mut car, false);
        phys_car::shift(&mut car, false);
        assert_eq!(phys_car::gear_label(&(car)), "N");
        assert!(phys_car::drivetrain_step(&(car), 1.0, 1.0 / 240.0, 0.0).is_none());
        car.velocity = XZ { x: 2.0, z: 0.0 };
        phys_car::shift(&mut car, false);
        assert_eq!(phys_car::gear_label(&(car)), "R");
        assert_eq!(car.velocity, XZ { x: 2.0, z: 0.0 });
        phys_car::shift(&mut car, true);
        assert_eq!(phys_car::gear_label(&(car)), "N");
        phys_car::shift(&mut car, true);
        assert_eq!(phys_car::gear_label(&(car)), "1");
        for _ in 0..20 {
            phys_car::shift(&mut car, true);
        }
        assert_eq!(car.gear, phys_car::gears(&(car.spec.gearbox)));
        phys_car::reset(
            &mut car,
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
        );
        assert!(car.manual);
        assert_eq!(car.gear, 1);
    }

    #[test]
    fn rolling_direction_changes_keep_momentum_then_reverse_under_power() {
        for model in test_support::cars() {
            for initial_speed in [12.0, -4.0] {
                let mut car = level_route_car(test_support::authored_id(), model.clone());
                car.manual = true;
                if initial_speed < 0.0 {
                    phys_car::shift(&mut car, false);
                    phys_car::shift(&mut car, false);
                }
                car.velocity = XZ {
                    x: car.heading.sin() * initial_speed,
                    z: car.heading.cos() * initial_speed,
                };
                phys_car::refresh_speed(&mut car);
                let velocity = car.velocity;
                let heading = car.heading;
                // Both directions pass through neutral while still moving.
                phys_car::shift(&mut car, initial_speed < 0.0);
                assert_eq!(car.gear, NEUTRAL_GEAR);
                assert_eq!(car.velocity, velocity);
                phys_car::shift(&mut car, initial_speed < 0.0);
                assert_eq!(car.gear, if initial_speed > 0.0 { 0 } else { 1 });
                assert_eq!(car.velocity, velocity);
                assert_eq!(car.heading, heading);
                assert!(phys_car::engine_net_torque(&(car), 1.0) > 0.0);
                for _ in 0..240 * 8 {
                    treadmill_tick(
                        &mut car,
                        Controls {
                            throttle: 1.0,
                            ..Controls::default()
                        },
                    );
                }
                assert!(
                    car.speed * initial_speed < 0.0,
                    "{model:?} must pull away in the selected direction: {} -> {}",
                    initial_speed,
                    car.speed
                );
            }
        }
    }

    #[test]
    fn tires_regain_direction_after_lifting_out_of_a_slide() {
        for model in test_support::cars() {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            let h = car.heading;
            car.velocity = XZ {
                x: h.sin() * 15.0 + h.cos() * 6.0,
                z: h.cos() * 15.0 - h.sin() * 6.0,
            };
            car.yaw_rate = 0.5;
            for _ in 0..480 {
                treadmill_tick(&mut car, Controls::default());
            }
            let rear_side = car.slip - car.yaw_rate * model.wheelbase * model.front_weight;
            let angle = rear_side.abs().atan2(car.speed.abs().max(2.5)).to_degrees();
            assert!(angle < 3.0, "{model:?}: slide persists after lift: {angle}");
            assert!(
                car.yaw_rate.abs() < 0.15,
                "{model:?}: rotation persists after lift"
            );
            assert!(
                car.speed > 8.0,
                "recovering grip must not simply stop the car"
            );
        }
    }

    #[test]
    fn revving_in_neutral_then_engaging_a_gear_spins_the_driven_wheels() {
        for (model, reverse) in test_support::cars()
            .iter()
            .cloned()
            .flat_map(|m| [(m.clone(), false), (m, true)])
        {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            car.manual = true;
            phys_car::shift(&mut car, false);
            assert_eq!(car.gear, NEUTRAL_GEAR);
            let throttle = Controls {
                throttle: 1.0,
                ..Controls::default()
            };
            // Start each launch with comparable crankshaft speed. Authored
            // torque curves need different amounts of time to free-rev.
            for _ in 0..720 {
                treadmill_tick(&mut car, throttle);
                if car.engine_rpm > 4500.0 {
                    break;
                }
            }
            assert!(
                car.speed.abs() < 0.02,
                "{model:?}: neutral must not move the car"
            );
            assert!(
                car.engine_rpm > 4500.0,
                "{model:?}: neutral must allow free revving"
            );
            phys_car::shift(&mut car, !reverse);
            assert_eq!(car.gear, if reverse { 0 } else { 1 });
            let mut peak_spin = 0.0_f32;
            for _ in 0..240 {
                treadmill_tick(&mut car, throttle);
                peak_spin = peak_spin.max(car.wheel_spin);
            }
            assert!(
                peak_spin > 0.15,
                "{model:?}: neutral launch must break traction"
            );
            assert!(car.speed.abs() > 1.0);
            for _ in 0..480 {
                treadmill_tick(&mut car, Controls::default());
            }
            assert!(car.wheel_spin < 0.01);
            let position = car.position;
            let heading = car.heading;
            phys_car::reset(&mut car, position, heading);
            assert_eq!(car.driven_wheel_omega, 0.0);
        }
    }

    #[test]
    fn driven_tires_share_cornering_grip_and_yaw_has_inertia() {
        let run = |throttle| {
            let mut car = level_route_car(track::LocationId::Compact, test_support::car());
            let h = car.heading;
            car.velocity = XZ {
                x: h.sin() * 10.0 + h.cos() * 2.0,
                z: h.cos() * 10.0 - h.sin() * 2.0,
            };
            phys_car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / 240.0),
                Controls {
                    throttle,
                    ..Controls::default()
                },
            );
            dot(
                car.velocity,
                XZ {
                    x: h.cos(),
                    z: -h.sin(),
                },
            )
        };
        assert!(
            run(1.0) > run(0.0),
            "drive torque must reduce available cornering force"
        );
        let mut car = level_route_car(track::LocationId::Compact, test_support::car());
        car.velocity = XZ {
            x: car.heading.sin() * 20.0,
            z: car.heading.cos() * 20.0,
        };
        car.yaw_rate = 0.5;
        phys_car::update(
            &mut car,
            Duration::from_secs_f64(1.0 / 240.0),
            Controls::default(),
        );
        assert!(
            car.yaw_rate > 0.0 && car.yaw_rate < 0.5,
            "tire torque should slow existing rotation without deleting it"
        );
    }

    #[test]
    fn bump_moves_sprung_body_then_settles_at_different_frame_rates() {
        let run = |hz: u32| {
            let track = std::sync::Arc::new(test_support::bumpy_track(
                &track::try_track(&test_support::authored_id()).unwrap(),
            ));
            let road = track::at_distance(&track, 50.0);
            let mut car = phys_car::on_track(
                vehicle_location::vehicle_position(road, 0.0),
                world::heading(road),
                test_support::car(),
                track,
            );
            car.velocity = geometry::xz_map(road.tangent, |v| v * 8.0);
            let mut peak_pitch: f32 = 0.0;
            let mut peak_heave: f32 = 0.0;
            for _ in 0..hz * 3 {
                phys_car::update(
                    &mut car,
                    Duration::from_secs_f64(1.0 / hz as f64),
                    Controls::default(),
                );
                peak_pitch = peak_pitch.max(car.suspension_pitch.abs());
                peak_heave = peak_heave.max((car.position.y - 0.48).abs());
            }
            println!(
                "{hz} Hz: bump pitch {peak_pitch}, heave {peak_heave}, final {}",
                car.position.y
            );
            assert!(peak_pitch > 0.01 && peak_pitch < 0.15);
            assert!(peak_heave > 0.025 && peak_heave < 0.25);
            assert!((car.position.y - phys_car::ride_height(&(car))).abs() < 0.003);
            assert!(car.suspension_pitch.abs() < 0.002);
            assert!(car.suspension_roll.abs() < 0.002);
            (peak_pitch, peak_heave)
        };
        let reference = run(240);
        for hz in [30, 60, 144] {
            let sample = run(hz);
            assert!((sample.0 - reference.0).abs() < 0.005);
            assert!((sample.1 - reference.1).abs() < 0.01);
        }
    }

    #[test]
    fn chassis_can_unload_land_and_settle() {
        let mut car = level_route_car(test_support::authored_id(), test_support::car());
        let ground = phys_car::ride_height(&car);
        car.suspension_initialized = true;
        car.position.y = ground + 0.5;
        phys_car::update(&mut car, Duration::from_millis(20), Controls::default());
        assert_eq!(car.axle_load, [0.0; 2]);
        assert!(car.position.y > ground + 0.49);
        assert!(car.vertical_speed < 0.0);
        for _ in 0..240 * 3 {
            phys_car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / 240.0),
                Controls::default(),
            );
        }
        assert!((car.position.y - ground).abs() < 0.002);
        assert!(car.vertical_speed.abs() < 0.002);
        assert!((car.axle_load.iter().sum::<f32>() / car.spec.weight_kg - 9.81).abs() < 0.02);
    }

    #[test]
    fn elevation_changes_coasting_and_power_in_both_directions() {
        // Isolate gravity below tire saturation; a loaded uphill suspension
        // can temporarily provide more launch traction at full throttle.
        for throttle in [0.0, 0.35] {
            for reverse in [false, true] {
                let run = |uphill| {
                    let mut car = hill_car(uphill);
                    if reverse {
                        phys_car::toggle_direction(&mut car);
                        car.heading += std::f32::consts::PI;
                    }
                    let sign = if reverse { -1.0 } else { 1.0 };
                    car.velocity = XZ {
                        x: car.heading.sin() * 8.0 * sign,
                        z: car.heading.cos() * 8.0 * sign,
                    };
                    phys_car::update(
                        &mut car,
                        Duration::from_millis(250),
                        Controls {
                            throttle,
                            ..Controls::default()
                        },
                    );
                    car.speed * sign
                };
                let down = run(false);
                let up = run(true);
                assert!(
                    down > up + 0.15,
                    "throttle {throttle}, reverse {reverse}: downhill {down}, uphill {up}"
                );
            }
        }
    }

    #[test]
    fn hills_allow_rollback_but_brakes_hold_and_substeps_are_consistent() {
        let mut rolling = hill_car(true);
        phys_car::update(&mut rolling, Duration::from_secs(1), Controls::default());
        assert!(rolling.speed < -0.1, "rollback: {}", rolling.speed);
        for handbrake in [false, true] {
            let mut held = hill_car(true);
            phys_car::update(
                &mut held,
                Duration::from_secs(2),
                Controls {
                    brake: if handbrake { 0.0 } else { 1.0 },
                    handbrake,
                    ..Controls::default()
                },
            );
            assert!(held.speed.abs() < 0.001);
        }
        let run = |hz| {
            let mut car = hill_car(false);
            for _ in 0..hz {
                phys_car::update(
                    &mut car,
                    Duration::from_secs_f64(1.0 / hz as f64),
                    Controls::default(),
                );
            }
            car.speed
        };
        assert!((run(30) - run(120)).abs() < 0.005);
    }

    fn car() -> Vehicle {
        let start =
            track::at_distance(&track::try_track(&track::LocationId::Compact).unwrap(), 5.0);
        new(
            vehicle_location::vehicle_position(start, 0.0),
            world::heading(start),
            test_support::car(),
        )
    }

    fn route_car(id: track::LocationId, model: std::sync::Arc<asset_car::CarSpec>) -> Vehicle {
        let track = track::try_track(&id).unwrap();
        let road = track::at_distance(&track, 50.0);
        phys_car::on_track(
            vehicle_location::vehicle_position(road, 0.0),
            world::heading(road),
            model,
            track,
        )
    }

    // Compare car behavior on identical level ground across locations.
    fn level_route_car(
        id: track::LocationId,
        model: std::sync::Arc<asset_car::CarSpec>,
    ) -> Vehicle {
        let track = std::sync::Arc::new(test_support::flat_track(&track::try_track(&id).unwrap()));
        let road = track::at_distance(&track, 50.0);
        phys_car::on_track(
            vehicle_location::vehicle_position(road, 0.0),
            world::heading(road),
            model,
            track,
        )
    }

    // Reset only position each tick: velocity/drivetrain evolve normally, but
    // bends and barriers cannot masquerade as acceleration or braking limits.
    fn treadmill_tick(car: &mut Vehicle, controls: Controls) -> f32 {
        let road = track::at_distance(&car.track, 50.0);
        car.position = vehicle_location::vehicle_position(road, 0.0);
        let previous = car.position;
        phys_car::update(car, Duration::from_secs_f64(1.0 / 240.0), controls);
        dot(
            XZ {
                x: car.position.x - previous.x,
                z: car.position.z - previous.z,
            },
            road.tangent,
        )
        .abs()
    }

    // Keep a fixed speed on a straight test patch so road barriers, acceleration
    // and drag cannot hide a steering regression. Heading/yaw evolve normally.
    fn steering_run(car: &mut Vehicle, speed: f32, input: f32, seconds: f64, hz: f64) {
        // Externally imposed road speed supplies no drivetrain torque.
        car.manual = true;
        car.gear = NEUTRAL_GEAR;
        let road = track::at_distance(&car.track, 50.0);
        let mut remaining = seconds;
        while remaining > 1e-8 {
            let step = remaining.min(1.0 / hz);
            car.position = vehicle_location::vehicle_position(road, 0.0);
            let lateral = dot(
                car.velocity,
                XZ {
                    x: car.heading.cos(),
                    z: -car.heading.sin(),
                },
            );
            car.velocity = XZ {
                x: car.heading.sin() * speed + car.heading.cos() * lateral,
                z: car.heading.cos() * speed - car.heading.sin() * lateral,
            };
            phys_car::refresh_speed(car);
            phys_car::update(
                car,
                Duration::from_secs_f64(step),
                Controls {
                    steering: input,
                    ..Controls::default()
                },
            );
            remaining -= step;
        }
    }

    #[test]
    fn parking_speed_retains_steering_lock_and_follows_the_front_wheels() {
        for model in test_support::cars() {
            for speed in [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0] {
                let mut car = level_route_car(track::LocationId::Compact, model.clone());
                for steering in [-1.0, 1.0] {
                    steering_run(&mut car, speed, steering, 5.0, 240.0);
                    let angle = phys_car::steering_angle(&car).abs();
                    let expected_yaw = speed.abs() * angle.tan() / model.wheelbase;
                    println!(
                        "{model:?} at {speed}: lock {angle}, yaw {} expected {expected_yaw}",
                        car.yaw_rate.abs()
                    );
                    assert!(
                        (angle - 0.54).abs() < 0.001,
                        "{model:?}: full input must retain full steering lock"
                    );
                    assert!(
                        car.yaw_rate.abs() > expected_yaw * 0.75,
                        "{model:?}: low-speed turn pushes wide of the front wheels"
                    );
                }
            }
        }
    }

    #[test]
    fn adding_steering_always_increases_wheel_angle_at_road_speeds() {
        for model in test_support::cars() {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            for speed_kmh in [0.0, 40.0, 80.0, 120.0, 200.0] {
                car.speed = speed_kmh / 3.6;
                for direction in [-1.0, 1.0] {
                    let mut previous = 0.0;
                    for step in 1..=100 {
                        car.steering = direction * step as f32 / 100.0;
                        let angle = phys_car::front_wheel_angle(&car).abs();
                        assert!(
                            angle > previous,
                            "{model:?} at {speed_kmh} km/h: more input reduced wheel angle"
                        );
                        previous = angle;
                    }
                    assert!((previous - 0.54).abs() < 0.001);
                }
            }
        }
    }

    #[test]
    fn steering_lock_tapers_with_speed_but_stays_full_for_parking() {
        let mut car = new(
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            test_support::car(),
        );
        for speed in [-5.0, 0.0, 5.0] {
            car.speed = speed;
            car.steering = 0.0;
            phys_car::update_steering(&mut car, 1.0, 1.0);
            assert!(
                (phys_car::steering_angle(&(car)).abs() - 0.54).abs() < 0.001,
                "parking speed {speed} must keep full lock"
            );
        }
        car.speed = 30.0;
        phys_car::update_steering(&mut car, 1.0, 1.0);
        let fast = phys_car::steering_angle(&car).abs();
        assert!(fast > 0.1, "some lock must remain at speed");
        assert!(
            fast < 0.54 * 0.4,
            "high speed must not offer parking lock: {fast}"
        );
    }

    #[test]
    fn keyboard_taps_turn_less_than_held_input_and_release_steering() {
        let model = test_support::car();
        for speed in [5.0, 15.0, 25.0, 45.0, 55.0] {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            let heading = car.heading;
            // Stay below the high-speed lock so this measures tap versus hold
            // rather than the speed-dependent lock taper.
            steering_run(&mut car, speed, 0.2, 0.1, 120.0);
            let tap_yaw = car.yaw_rate.abs();
            steering_run(&mut car, speed, 0.0, 0.4, 120.0);
            let tap = test_support::angle(car.heading - heading).abs();
            assert!(tap > 0.0, "a steering tap must turn the car");
            assert_eq!(car.steering, 0.0);
            assert!(
                car.yaw_rate.abs() < tap_yaw,
                "releasing steering must reduce yaw"
            );
            let mut held = level_route_car(track::LocationId::Compact, model.clone());
            steering_run(&mut held, speed, 0.2, 1.0, 120.0);
            let turn = test_support::angle(held.heading - heading).abs();
            assert!(
                turn > tap * 3.0,
                "holding must produce a stronger turn than tapping"
            );
        }
    }

    #[test]
    fn tire_torque_turns_and_countersteering_reverses_rotation() {
        for model in test_support::cars() {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            steering_run(&mut car, 15.0, 0.25, 0.6, 240.0);
            assert!(car.yaw_rate < 0.0);
            assert!(car.yaw_rate.is_finite() && car.yaw_rate.abs() < 2.0);
            steering_run(&mut car, 15.0, -0.5, 0.8, 240.0);
            assert!(
                car.yaw_rate > 0.0,
                "{model:?} countersteering remained in original turn"
            );
            assert!(phys_car::front_wheel_angle(&(car)) > 0.0);
            steering_run(&mut car, 0.0, 0.0, 0.02, 240.0);
            assert!(car.yaw_rate.is_finite());
            let position = car.position;
            let heading = car.heading;
            phys_car::reset(&mut car, position, heading);
            assert_eq!(car.yaw_rate, 0.0);
            assert_eq!(phys_car::front_wheel_angle(&(car)), 0.0);
        }
    }

    #[test]
    fn steering_response_is_consistent_across_frame_rates() {
        let model = test_support::car();
        let mut reference = level_route_car(track::LocationId::Compact, model.clone());
        let start = reference.heading;
        for (input, seconds) in [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)] {
            steering_run(&mut reference, 25.0, input, seconds, 240.0);
        }
        for hz in [30.0, 60.0, 144.0] {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            for (input, seconds) in [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)] {
                steering_run(&mut car, 25.0, input, seconds, hz);
            }
            let error = test_support::angle(car.heading - reference.heading)
                .abs()
                .to_degrees();
            assert!(
                error < 0.25,
                "{model:?} {hz} Hz differs by {error} degrees from 240 Hz; start {start}"
            );
        }
    }

    #[test]
    fn released_steering_centres_faster_with_forward_speed() {
        let mut car = new(
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            test_support::car(),
        );
        for direction in [-1.0, 1.0] {
            let mut previous = 1.0;
            for speed in [0.5, 2.0, 5.0, 10.0] {
                car.speed = speed;
                let mut reference: Option<f32> = None;
                for hz in [30, 60, 240] {
                    car.steering = direction;
                    for _ in 0..hz / 10 {
                        phys_car::update_steering(&mut car, 0.0, 1.0 / hz as f32);
                    }
                    let remaining = car.steering.abs();
                    if let Some(expected) = reference {
                        assert!((remaining - expected).abs() < 0.00001);
                    } else {
                        assert!(remaining < previous);
                        previous = remaining;
                        reference = Some(remaining);
                    }
                }
            }
            car.speed = -5.0;
            car.steering = direction;
            phys_car::update_steering(&mut car, 0.0, 1.0);
            assert_eq!(car.steering, direction);
            car.speed = 10.0;
            phys_car::update_steering(&mut car, 0.0, 1.0);
            assert_eq!(car.steering, 0.0);
        }
    }

    #[test]
    fn ratio_shifts_drop_rpm_and_do_not_hunt_around_the_upshift_speed() {
        for model in test_support::cars() {
            let mut car = route_car(test_support::authored_id(), model.clone());
            let road = track::at_distance(&car.track, 50.0);
            let g = &model.gearbox;
            for gear in 1..phys_car::gears(g) {
                car.gear = gear;
                let shift = phys_car::upshift_speed(&car, gear);
                car.velocity = geometry::xz_map(road.tangent, |v| v * (shift - 0.01));
                phys_car::refresh_speed(&mut car);
                assert_eq!(car.gear, gear);
                let before = phys_car::coupled_engine_rpm(&car);
                car.velocity = geometry::xz_map(road.tangent, |v| v * (shift + 0.01));
                phys_car::refresh_speed(&mut car);
                assert_eq!(car.gear, gear + 1);
                assert!(phys_car::coupled_engine_rpm(&(car)) < before);
                for delta in [-0.1, 0.1, -0.05, 0.05] {
                    car.velocity = geometry::xz_map(road.tangent, |v| v * (shift + delta));
                    phys_car::refresh_speed(&mut car);
                    assert_eq!(
                        car.gear,
                        gear + 1,
                        "must retain higher gear near an upshift"
                    );
                }
                let down = phys_car::speed_at_rpm(g, g.downshift_rpm, gear + 1);
                car.velocity = geometry::xz_map(road.tangent, |v| v * (down - 0.01));
                phys_car::refresh_speed(&mut car);
                assert!(car.gear <= gear);
                assert!(phys_car::coupled_engine_rpm(&(car)) < car.spec.redline_rpm);
            }
            car.velocity = XZ { x: 0.0, z: 0.0 };
            phys_car::refresh_speed(&mut car);
            assert_eq!(car.gear, 1);
            assert_eq!(phys_car::coupled_engine_rpm(&(car)), IDLE_RPM);
        }
    }

    #[test]
    fn city_curb_is_crossable_with_a_single_small_speed_loss_and_settling_bump() {
        let track = test_support::city_world();
        let city = &track;
        let (q, normal) = world_roads::curbs(city)
            .iter()
            .find_map(|curb| {
                let q = XZ {
                    x: (curb.ends[0].x + curb.ends[1].x) * 0.5,
                    z: (curb.ends[0].z + curb.ends[1].z) * 0.5,
                };
                let inside = XZ {
                    x: q.x + curb.inward.x * 0.08,
                    z: q.z + curb.inward.z * 0.08,
                };
                let outside = XZ {
                    x: q.x - curb.inward.x * 0.2,
                    z: q.z - curb.inward.z * 0.2,
                };
                if !street_surface::contains_road(city, inside)
                    || street_surface::contains_road(city, outside)
                    || street_surface::paving_height(city, outside)
                        != Some(world_roads::pavement_height_meters(city))
                {
                    return None;
                }
                for distance in [0.08, -0.2, -1.0, -2.0, -3.0] {
                    let mut p = XYZ {
                        x: q.x + curb.inward.x * distance,
                        y: 0.0,
                        z: q.z + curb.inward.z * distance,
                    };
                    let before = p;
                    vehicle_location::constrain(city, &mut p, &mut XZ::default(), 1.7);
                    if (p.x - before.x).hypot(p.z - before.z) > 0.001 {
                        return None;
                    }
                }
                Some((q, curb.inward))
            })
            .expect("missing clear curb crossing fixture");
        let position = XYZ {
            x: q.x + normal.x * 0.08,
            y: 0.0,
            z: q.z + normal.z * 0.08,
        };
        let heading = (-normal.x).atan2(-normal.z);
        let mut car = phys_car::on_track(position, heading, test_support::car(), track.clone());
        car.velocity = geometry::xz_map(normal, |n| -n * 8.0);
        phys_car::update(&mut car, Duration::from_millis(24), Controls::default());
        assert!(!street_surface::contains_road(
            city,
            XZ {
                x: car.position.x,
                z: car.position.z
            }
        ));
        assert!(
            car.speed > 7.0 && car.speed < 7.8,
            "small curb penalty: {}",
            car.speed
        );
        assert!(car.curb_bump > 0.0 && car.curb_bump <= 0.03);
        let speed = car.speed;
        let bump = car.curb_bump;
        phys_car::update(&mut car, Duration::ZERO, Controls::default());
        assert_eq!(car.curb_bump, bump);
        car.gear = NEUTRAL_GEAR; // isolate curb drag from intentional engine braking
        phys_car::update(&mut car, Duration::from_millis(250), Controls::default());
        assert!(car.curb_bump < bump * 0.1);
        assert!(
            car.speed > speed - 0.3,
            "do not repeatedly apply curb drag off the asphalt"
        );
        let position = car.position;
        let heading = car.heading;
        phys_car::reset(&mut car, position, heading);
        assert_eq!(car.curb_bump, 0.0);
    }

    #[test]
    fn reverse_acceleration_terminal_speed_and_braking_are_route_independent() {
        for model in test_support::cars() {
            let mut compact = level_route_car(track::LocationId::Compact, model.clone());
            let mut scenic = level_route_car(test_support::authored_id(), model.clone());
            assert!(
                phys_car::toggle_direction(&mut (compact))
                    && phys_car::toggle_direction(&mut (scenic))
            );
            // Projection rounding can change the phase of repeated limiter
            // cycles. Keep the tight comparison until the limiter first cuts,
            // then allow its 250 RPM band plus one substep of tire acceleration.
            let mut tolerance = 0.01;
            let limiter_tolerance = phys_car::speed_at_rpm(&model.gearbox, 250.0, 0) + 9.81 / 240.0;
            for _ in 0..240 * 12 {
                for car in [&mut compact, &mut scenic] {
                    treadmill_tick(
                        car,
                        Controls {
                            throttle: 1.0,
                            ..Controls::default()
                        },
                    );
                }
                if compact.fuel_cut || scenic.fuel_cut {
                    tolerance = limiter_tolerance;
                }
                assert!(
                    (compact.speed - scenic.speed).abs() < tolerance,
                    "{model:?}: {} vs {}, slip {} vs {}",
                    compact.speed,
                    scenic.speed,
                    compact.slip,
                    scenic.slip
                );
            }
            assert!(scenic.speed < -6.0 && scenic.speed.is_finite());
            let (mut compact_distance, mut scenic_distance) = (0.0, 0.0);
            for _ in 0..720 {
                let brake = Controls {
                    brake: 1.0,
                    ..Controls::default()
                };
                compact_distance += treadmill_tick(&mut compact, brake);
                scenic_distance += treadmill_tick(&mut scenic, brake);
                assert!(
                    (compact.speed - scenic.speed).abs() < tolerance,
                    "{model:?}: {} vs {}, slip {} vs {}",
                    compact.speed,
                    scenic.speed,
                    compact.slip,
                    scenic.slip
                );
            }
            assert!(compact.speed.abs() < 0.001 && scenic.speed.abs() < 0.001);
            // The limiter can leave a small speed difference when braking
            // starts; allow that difference over the three-second stop window.
            assert!((compact_distance - scenic_distance).abs() < tolerance * 3.0);
            assert_eq!(scenic.gear, 0);
        }
    }

    #[test]
    fn offroad_driving_and_reentry_preserve_position_and_follow_terrain() {
        let track = track::try_track(&test_support::authored_id()).unwrap();
        let road = track::at_distance(&track, 50.0);
        for side in [-1.0, 1.0] {
            let position =
                vehicle_location::vehicle_position(road, side * (road.width * 0.5 + 8.0));
            let inward = geometry::xz_map(world::normal(road), |v| -side * v);
            let mut car = phys_car::on_track(
                position,
                inward.x.atan2(inward.z),
                test_support::car(),
                track.clone(),
            );
            car.manual = true;
            car.gear = NEUTRAL_GEAR;
            car.velocity = geometry::xz_map(inward, |v| v * 10.0);
            assert_eq!(phys_car::resolve_road(&mut (car)), 0.0);
            assert_eq!([car.position.x, car.position.z], [position.x, position.z]);
            for _ in 0..288 {
                assert!(
                    phys_car::update(
                        &mut (car),
                        Duration::from_secs_f64(1.0 / 240.0),
                        Controls::default()
                    ) < 0.01
                );
                assert!(geometry::iter(&(car.position)).all(|v| v.is_finite()));
            }
            assert!(world_roads::project(&track, car.position).offset.abs() < road.width * 0.5);
            assert!((car.position.y - phys_car::ride_height(&(car))).abs() < 0.15);
        }
    }

    #[test]
    fn scenic_recovery_preserves_route_and_model_but_clears_motion_and_reverse() {
        let mut car = route_car(test_support::authored_id(), test_support::car());
        let model = car.spec.clone();
        let route = car.track.clone();
        phys_car::toggle_direction(&mut car);
        treadmill_tick(
            &mut car,
            Controls {
                throttle: 1.0,
                steering: 1.0,
                ..Controls::default()
            },
        );
        car.tire_scrub = 1.0;
        let road = track::at_distance(&car.track, 850.0);
        phys_car::reset(
            &mut car,
            vehicle_location::vehicle_position(road, 0.0),
            world::heading(road),
        );
        assert!(std::sync::Arc::ptr_eq(&car.track, &route));
        assert!(std::sync::Arc::ptr_eq(&car.spec, &model));
        assert_eq!(car.position, vehicle_location::vehicle_position(road, 0.0));
        assert_eq!(car.heading, world::heading(road));
        assert_eq!(car.velocity, XZ { x: 0.0, z: 0.0 });
        assert_eq!(car.direction, Direction::Drive);
        assert_eq!(car.gear, 1);
        assert_eq!(car.speed, 0.0);
        assert_eq!(car.slip, 0.0);
        assert_eq!(car.body_pitch, 0.0);
        assert_eq!(car.body_roll, 0.0);
        assert_eq!(car.engine_rpm, IDLE_RPM);
        assert_eq!(car.engine_load, 0.0);
        assert_eq!(car.wheel_phase, [0.0; 2]);
        assert_eq!(phys_car::front_wheel_angle(&(car)), 0.0);
        assert_eq!(car.tire_scrub, 0.0);
    }

    #[test]
    fn outer_terrain_boundary_preserves_tangential_motion_and_ground_height() {
        let track = track::try_track(&track::LocationId::Compact).unwrap();
        let bounds = world::terrain_bounds(&track);
        let mut car = phys_car::on_track(
            XYZ {
                x: bounds.min.x - 1.0,
                y: 0.0,
                z: (bounds.min.z + bounds.max.z) * 0.5,
            },
            0.0,
            test_support::car(),
            track.clone(),
        );
        car.velocity = XZ { x: -4.0, z: 8.0 };
        assert!(phys_car::resolve_road(&mut (car)) > 0.0);
        assert_eq!(car.position.x, bounds.min.x + track::CAR_RADIUS);
        assert!(car.velocity.z > 6.0);
        assert!(
            (car.position.y
                - vehicle_location::drive_height(
                    &track,
                    XZ {
                        x: car.position.x,
                        z: car.position.z
                    }
                )
                - 0.48)
                .abs()
                < 0.001
        );
    }

    #[test]
    fn unequal_mass_contact_conserves_momentum_and_moves_light_car_more() {
        let sample = track::at_distance(
            &track::try_track(&track::LocationId::Compact).unwrap(),
            50.0,
        );
        let mut heavy_spec = test_support::car_spec().clone();
        heavy_spec.weight_kg = 1800.0;
        let mut heavy = new(
            vehicle_location::vehicle_position(sample, 0.8),
            world::heading(sample),
            std::sync::Arc::new(heavy_spec),
        );
        let mut light = phys_car::on_track(
            vehicle_location::vehicle_position(sample, -0.8),
            world::heading(sample),
            test_support::car(),
            heavy.track.clone(),
        );
        let n = world::normal(sample);
        heavy.velocity = geometry::xz_map(n, |x| -x * 3.0);
        let momentum = |a: &Vehicle, b: &Vehicle| {
            std::array::from_fn::<_, 2, _>(|i| {
                geometry::xz_component(a.velocity, i) * a.spec.weight_kg
                    + geometry::xz_component(b.velocity, i) * b.spec.weight_kg
            })
        };
        let before = momentum(&heavy, &light);
        let heavy_before = heavy.position;
        let light_before = light.position;
        assert!(phys_car::collide(&mut (heavy), &mut light) > 0.0);
        assert!(dot(light.velocity, n) < 0.0);
        for car in [&heavy, &light] {
            assert!(geometry::iter(&(car.position))
                .chain([car.velocity.x, car.velocity.z].iter())
                .all(|v| v.is_finite()));
        }
        let after = momentum(&heavy, &light);
        for i in 0..2 {
            assert!((before[i] - after[i]).abs() < 0.01);
        }
        let moved = |a: XYZ, b: XYZ| (a.x - b.x).hypot(a.z - b.z);
        assert!(moved(light_before, light.position) > moved(heavy_before, heavy.position));
    }

    #[test]
    fn holding_brakes_at_rest_does_not_dive_and_settles_after_stopping() {
        for handbrake in [false, true] {
            for initial_speed in [0.0, 8.0, -8.0] {
                let mut v = car();
                let forward = XZ {
                    x: v.heading.sin(),
                    z: v.heading.cos(),
                };
                v.velocity = geometry::xz_map(forward, |component| component * initial_speed);
                phys_car::refresh_speed(&mut v);
                for _ in 0..600 {
                    phys_car::update(
                        &mut v,
                        Duration::from_secs_f64(1.0 / 60.0),
                        Controls {
                            brake: if handbrake { 0.0 } else { 1.0 },
                            handbrake,
                            ..Controls::default()
                        },
                    );
                    if initial_speed == 0.0 {
                        assert!(v.body_pitch.abs() < 0.0001);
                    }
                }
                assert!(v.speed.abs() < 0.001);
                assert!(v.body_pitch.abs() < 0.0001);
            }
        }
    }

    #[test]
    fn body_motion_handles_reverse_and_frame_rate_independent_settling() {
        let mut forward = car();
        let mut reverse = car();
        forward.speed = 6.0;
        reverse.speed = -6.0;
        forward.acceleration[0] = 3.0;
        reverse.acceleration[0] = -3.0;
        for _ in 0..240 {
            phys_car::update_body_motion(&mut forward, 1.0 / 240.0);
            phys_car::update_body_motion(&mut reverse, 1.0 / 240.0);
        }
        assert!(forward.body_pitch > 0.005);
        assert!((forward.body_pitch + reverse.body_pitch).abs() < 0.00001);
        reverse.acceleration[0] = 4.0;
        for _ in 0..240 {
            phys_car::update_body_motion(&mut reverse, 1.0 / 240.0);
        }
        assert!(
            reverse.body_pitch > 0.01,
            "reverse braking should lift the nose"
        );
        forward.acceleration = [0.0; 2];
        reverse.acceleration = [0.0; 2];
        forward.body_pitch = 0.05;
        reverse.body_pitch = 0.05;
        for _ in 0..30 {
            phys_car::update_body_motion(&mut forward, 1.0 / 30.0);
        }
        for _ in 0..144 {
            phys_car::update_body_motion(&mut reverse, 1.0 / 144.0);
        }
        assert!((forward.body_pitch - reverse.body_pitch).abs() < 0.0002);
        assert!(forward.body_pitch.abs() < 0.0005);
    }

    #[test]
    fn rolling_engine_rpm_follows_gearing_without_a_throttle_floor() {
        for model in test_support::cars() {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            car.manual = true;
            let speed = phys_car::speed_at_rpm(&model.gearbox, 1800.0, 1);
            car.velocity = XZ {
                x: car.heading.sin() * speed,
                z: car.heading.cos() * speed,
            };
            for tick in 0..60 {
                treadmill_tick(
                    &mut car,
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
                if tick == 0 {
                    assert!(
                        car.engine_rpm < 2200.0,
                        "throttle must not impose a launch RPM floor"
                    );
                }
                let shaft_rpm = car.driven_wheel_omega
                    * phys_car::rpm(&model.gearbox, 1.0, 1)
                    * model.gearbox.rolling_radius;
                assert!(
                    (car.engine_rpm - shaft_rpm).abs() < 1.0,
                    "{model:?}: clutch lock must determine crankshaft RPM"
                );
            }
        }
    }

    #[test]
    fn shifts_release_the_clutch_and_road_speed_can_mechanically_overrev_the_engine() {
        let model = test_support::car();
        let mut car = level_route_car(track::LocationId::Compact, model.clone());
        car.manual = true;
        car.gear = 2;
        let speed = phys_car::speed_at_rpm(&model.gearbox, 4000.0, 2);
        car.velocity = XZ {
            x: car.heading.sin() * speed,
            z: car.heading.cos() * speed,
        };
        treadmill_tick(&mut car, Controls::default());
        phys_car::shift(&mut car, true);
        let before = car.engine_rpm;
        for _ in 0..12 {
            treadmill_tick(
                &mut car,
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert!(
            car.engine_rpm < before,
            "engine coasts while the clutch is open"
        );
        assert!(phys_car::drivetrain_step(&(car), 1.0, 1.0 / 240.0, car.speed).is_none());
        for _ in 0..240 {
            treadmill_tick(
                &mut car,
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert!((car.engine_rpm - phys_car::coupled_engine_rpm(&(car))).abs() < 10.0);

        let mut overrev = level_route_car(track::LocationId::Compact, model.clone());
        overrev.manual = true;
        let speed = phys_car::speed_at_rpm(&model.gearbox, model.redline_rpm + 500.0, 1);
        overrev.velocity = XZ {
            x: overrev.heading.sin() * speed,
            z: overrev.heading.cos() * speed,
        };
        treadmill_tick(&mut overrev, Controls::default());
        assert!(overrev.engine_rpm > model.redline_rpm);
        assert!(
            overrev.fuel_cut,
            "fuel cut cannot clamp wheel-driven engine speed"
        );
    }

    #[test]
    fn drivetrain_acceleration_and_rpm_are_consistent_across_frame_rates() {
        let run = |hz: u32| {
            let mut car = level_route_car(track::LocationId::Compact, test_support::car());
            for _ in 0..hz * 4 {
                car.position =
                    vehicle_location::vehicle_position(track::at_distance(&car.track, 50.0), 0.0);
                phys_car::update(
                    &mut car,
                    Duration::from_secs_f64(1.0 / hz as f64),
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            (car.speed, car.engine_rpm)
        };
        let reference = run(240);
        for hz in [30, 60, 144] {
            let actual = run(hz);
            assert!(
                (actual.0 - reference.0).abs() < 0.05,
                "{hz} Hz: speed {actual:?}, reference {reference:?}"
            );
            assert!(
                (actual.1 - reference.1).abs() < 20.0,
                "{hz} Hz: RPM {actual:?}, reference {reference:?}"
            );
        }
    }

    #[test]
    fn neutral_revs_build_progressively_cycle_limiter_and_fall_to_idle() {
        let road = track::at_distance(
            &track::try_track(&track::LocationId::Compact).unwrap(),
            50.0,
        );
        for model in test_support::cars() {
            let mut car = new(
                vehicle_location::vehicle_position(road, 0.0),
                world::heading(road),
                model.clone(),
            );
            car.manual = true;
            phys_car::shift(&mut car, false);
            let dt = Duration::from_secs_f64(1.0 / 60.0);
            let pedal = Controls {
                throttle: 1.0,
                ..Controls::default()
            };
            for _ in 0..6 {
                phys_car::update(&mut car, dt, pedal);
            }
            assert!(car.engine_rpm > IDLE_RPM + 100.0);
            assert!(car.engine_rpm < 2500.0, "{model:?}: blip jumps too quickly");
            let mut cuts = 0;
            let mut recoveries = 0;
            for _ in 0..300 {
                let was_cut = car.fuel_cut;
                phys_car::update(&mut car, dt, pedal);
                cuts += usize::from(!was_cut && car.fuel_cut);
                recoveries += usize::from(was_cut && !car.fuel_cut);
                // A discrete combustion step can cross redline before fuel cuts.
                assert!((IDLE_RPM..=car.spec.redline_rpm + 100.0).contains(&car.engine_rpm));
            }
            assert!(cuts > 2 && recoveries > 2, "{model:?}: limiter must cycle");
            phys_car::update(&mut car, dt, Controls::default());
            assert!(
                car.engine_rpm > 5500.0,
                "release must retain engine inertia"
            );
            for _ in 0..300 {
                let before = car.engine_rpm;
                phys_car::update(&mut car, dt, Controls::default());
                assert!(car.engine_rpm <= before);
            }
            assert_eq!(car.engine_rpm, IDLE_RPM);
            assert_eq!(car.gear, NEUTRAL_GEAR);
            assert!(car.speed.abs() < 0.01, "neutral must not propel the car");
            assert!(phys_car::drivetrain_step(&(car), 1.0, 1.0 / 240.0, 0.0).is_none());
            assert_eq!(car.turbo_boost, 0.0);
        }
    }

    #[test]
    fn fuel_limiter_uses_the_same_thresholds_in_neutral_forward_and_reverse() {
        let road = track::at_distance(
            &track::try_track(&track::LocationId::Compact).unwrap(),
            50.0,
        );
        let pedal = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        let custom_models = [5800.0, 7400.0].map(|redline_rpm| {
            let mut spec = test_support::car_spec().clone();
            spec.redline_rpm = redline_rpm;
            std::sync::Arc::new(spec)
        });
        for model in test_support::cars().into_iter().chain(custom_models) {
            for gear in [NEUTRAL_GEAR, 1, 0] {
                let mut car = new(
                    vehicle_location::vehicle_position(road, 0.0),
                    world::heading(road),
                    model.clone(),
                );
                car.manual = true;
                car.gear = gear;
                car.direction = if gear == 0 {
                    Direction::Reverse
                } else {
                    Direction::Drive
                };
                for (rpm, cut) in [
                    (car.spec.redline_rpm + 1.0, true),
                    (car.spec.redline_rpm - 100.0, true),
                    (car.spec.redline_rpm - 251.0, false),
                    (car.spec.redline_rpm - 100.0, false),
                ] {
                    car.engine_rpm = rpm;
                    car.speed = phys_car::speed_at_rpm(&model.gearbox, rpm, gear);
                    let engine_rpm = car.engine_rpm;
                    phys_car::update_fuel_cut(&mut car, engine_rpm);
                    assert_eq!(car.fuel_cut, cut, "{model:?}, gear {gear}, RPM {rpm}");
                    if gear != NEUTRAL_GEAR {
                        assert_eq!(phys_car::engine_net_torque(&(car), 1.0) <= 0.0, cut);
                    }
                }
                car.fuel_cut = true;
                car.engine_load = 1.0;
                car.engine_rpm = car.spec.redline_rpm;
                car.speed =
                    phys_car::speed_at_rpm(&model.gearbox, car.spec.redline_rpm + 1.0, gear);
                phys_car::update_engine_feedback(&mut car, pedal, 0.01);
                assert!(car.engine_load < 1.0, "fuel cut must reach engine audio");
            }
        }
    }

    #[test]
    fn in_gear_limiter_cycles_under_power_and_survives_a_shift_to_neutral() {
        let model = test_support::car();
        for gear in [0, 1] {
            let mut car = level_route_car(track::LocationId::Compact, model.clone());
            car.manual = true;
            car.gear = gear;
            car.direction = if gear == 0 {
                Direction::Reverse
            } else {
                Direction::Drive
            };
            let speed = phys_car::speed_at_rpm(&model.gearbox, car.spec.redline_rpm + 20.0, gear)
                * if gear == 0 { -1.0 } else { 1.0 };
            car.velocity = XZ {
                x: car.heading.sin() * speed,
                z: car.heading.cos() * speed,
            };
            let mut cuts = 0;
            let mut recoveries = 0;
            for _ in 0..240 * 4 {
                let was_cut = car.fuel_cut;
                treadmill_tick(
                    &mut car,
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
                cuts += usize::from(!was_cut && car.fuel_cut);
                recoveries += usize::from(was_cut && !car.fuel_cut);
            }
            assert!(
                cuts > 2 && recoveries > 2,
                "gear {gear}: limiter must cycle"
            );
            car.fuel_cut = true;
            car.engine_rpm = car.spec.redline_rpm - 100.0;
            phys_car::shift(&mut car, gear == 0);
            assert_eq!(car.gear, NEUTRAL_GEAR);
            phys_car::update_engine_feedback(&mut car, Controls::default(), 0.0);
            assert!(car.fuel_cut, "shifting must not reset limiter hysteresis");
        }
    }

    #[test]
    fn neutral_partial_throttle_balances_drag_independently_of_road_speed() {
        let road = track::at_distance(
            &track::try_track(&track::LocationId::Compact).unwrap(),
            50.0,
        );
        for model in test_support::cars() {
            let mut car = new(
                vehicle_location::vehicle_position(road, 0.0),
                world::heading(road),
                model.clone(),
            );
            car.manual = true;
            car.gear = NEUTRAL_GEAR;
            let pedal = Controls {
                throttle: 0.35,
                ..Controls::default()
            };
            for _ in 0..600 {
                phys_car::update_free_engine(&mut car, pedal.throttle, 1.0 / 60.0);
            }
            let stationary = car.engine_rpm;
            assert!(stationary > IDLE_RPM && stationary < model.redline_rpm);
            let mut rolling = new(
                vehicle_location::vehicle_position(road, 0.0),
                world::heading(road),
                model,
            );
            rolling.gear = NEUTRAL_GEAR;
            rolling.engine_rpm = stationary;
            rolling.speed = 30.0;
            for _ in 0..120 {
                phys_car::update_free_engine(&mut car, pedal.throttle, 1.0 / 60.0);
                phys_car::update_free_engine(&mut rolling, pedal.throttle, 1.0 / 60.0);
                assert_eq!(car.engine_rpm, rolling.engine_rpm);
            }
            car.gear = 1;
            car.fuel_cut = true;
            phys_car::update_free_engine(&mut car, pedal.throttle, 1.0 / 60.0);
            assert!(
                !car.fuel_cut,
                "RPM below the recovery threshold clears the limiter"
            );
            phys_car::reset(
                &mut car,
                vehicle_location::vehicle_position(road, 0.0),
                world::heading(road),
            );
            assert!(!car.fuel_cut);
        }
    }

    #[test]
    fn neutral_blips_are_consistent_across_frame_rates() {
        let mut results = Vec::new();
        for hz in [30, 60, 144, 240] {
            let mut car = new(
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                0.0,
                test_support::car(),
            );
            car.gear = NEUTRAL_GEAR;
            for _ in 0..hz / 2 {
                phys_car::update(
                    &mut car,
                    Duration::from_secs_f64(1.0 / hz as f64),
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            results.push(car.engine_rpm);
        }
        let min = results.iter().cloned().fold(f32::INFINITY, f32::min);
        let max = results.iter().cloned().fold(0.0, f32::max);
        assert!(
            max - min < 20.0,
            "frame rates changed blip response: {results:?}"
        );
    }

    #[test]
    fn engine_feedback_tracks_load_and_shift_unload() {
        let road = track::at_distance(
            &track::try_track(&track::LocationId::Compact).unwrap(),
            50.0,
        );
        let mut car = new(
            vehicle_location::vehicle_position(road, 0.0),
            world::heading(road),
            test_support::car(),
        );
        assert_eq!(car.engine_rpm, IDLE_RPM);
        assert_eq!(car.engine_load, 0.0);
        let mut previous_speed = 0.0;
        for _ in 0..180 {
            car.position = vehicle_location::vehicle_position(road, 0.0);
            phys_car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            assert!(car.speed >= previous_speed - 0.02);
            previous_speed = car.speed;
        }
        assert!(car.engine_rpm > IDLE_RPM);
        assert!(car.engine_rpm < car.spec.redline_rpm);
        // Let a longer audible shift finish before checking sustained engine load.
        car.manual = true;
        for _ in 0..30 {
            car.position = vehicle_location::vehicle_position(road, 0.0);
            phys_car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert!(car.engine_load > 0.45);
        car.manual = false;

        // Begin below the first shift in first gear. The previous acceleration
        // may already have shifted; hysteresis correctly retains that gear.
        car.gear = 1;
        let load_before_shift = car.engine_load;
        car.velocity = geometry::xz_map(road.tangent, |v| {
            v * (phys_car::upshift_speed(&car, 1) - 0.02)
        });
        phys_car::refresh_speed(&mut car);
        assert_eq!(car.gear, 1);
        car.velocity = geometry::xz_map(road.tangent, |v| {
            v * (phys_car::upshift_speed(&car, 1) + 0.08)
        });
        phys_car::update(
            &mut car,
            Duration::from_secs_f64(1.0 / 60.0),
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
        assert_eq!(car.gear, 2);
        assert!(
            car.engine_load < load_before_shift,
            "shift should briefly unload the engine: {} -> {}",
            load_before_shift,
            car.engine_load
        );

        phys_car::reset(
            &mut car,
            vehicle_location::vehicle_position(road, 0.0),
            world::heading(road),
        );
        assert!(phys_car::toggle_direction(&mut (car)));
        for _ in 0..60 {
            car.position = vehicle_location::vehicle_position(road, 0.0);
            phys_car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert_eq!(car.gear, 0);
        assert!(car.engine_rpm > IDLE_RPM);
        assert!(car.engine_rpm <= car.spec.redline_rpm);
    }

    #[test]
    fn every_car_can_reverse_brake_and_drive_forward_again() {
        let road = track::at_distance(
            &track::try_track(&track::LocationId::Compact).unwrap(),
            50.0,
        );
        for model in test_support::cars() {
            let mut v = new(
                vehicle_location::vehicle_position(road, 0.0),
                world::heading(road),
                model.clone(),
            );
            assert!(phys_car::toggle_direction(&mut (v)));
            assert_eq!(phys_car::gear_label(&(v)), "R");
            for _ in 0..720 {
                v.position = vehicle_location::vehicle_position(road, 0.0); // straight-line test treadmill
                phys_car::update(
                    &mut v,
                    Duration::from_secs_f64(1.0 / 60.0),
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            assert!(
                v.speed < -6.0 && v.speed.is_finite(),
                "reverse speed for {model:?}: {}",
                v.speed
            );
            assert_eq!(v.gear, 0);
            assert!(
                !phys_car::toggle_direction(&mut (v)),
                "moving car cannot select drive"
            );
            phys_car::update(
                &mut v,
                Duration::from_secs(3),
                Controls {
                    brake: 1.0,
                    ..Controls::default()
                },
            );
            assert!(v.speed.abs() < 0.001);
            assert_eq!(v.direction, Direction::Reverse);
            assert!(phys_car::toggle_direction(&mut (v)));
            assert_eq!(phys_car::gear_label(&(v)), "1");
            phys_car::update(
                &mut v,
                Duration::from_millis(500),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            assert!(
                v.speed > 0.5,
                "{model:?} failed to pull away: {}, engine {}, wheel {}",
                v.speed,
                v.engine_rpm,
                v.driven_wheel_omega
            );
        }
    }
}
