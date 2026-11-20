//! Tire runtime state, wheel rotation, effects, and axle contacts.

use super::car;
use super::car::Axle;
use crate::model;
use crate::model::world::Material;
use crate::physics;
use crate::util;
use crate::world;

// Front axle grip relative to the rear. With equal grip the limit yaw moment
// is exactly neutral (each axle's capacity scales with its static load), so a
// fully developed slide never straightens itself. The front deficit also
// balances the rear's reduced lateral force under engine braking when sliding
// grip reduces the entire friction circle, so lifting can unwind a slide.
const FRONT_LIMIT_GRIP: f32 = 0.86;

/// Runtime wheel rotation and tire effects.
#[derive(Default)]
pub struct State {
    /// Effect intensity from zero to one for squeal, smoke, and skid marks.
    pub scrub: f32,
    /// Normalized driven wheel slip magnitude, from zero to one.
    pub wheel_spin: f32,
    /// Cosmetic rear/front axle rotation angles, in radians.
    pub wheel_phase: [f32; 2],
    /// Signed driven wheel angular speed, in radians per second.
    pub driven_wheel_omega: f32,
}

pub struct AxleGeometry {
    pub front_offset: f32,
    pub rear_offset: f32,
    pub sin: f32,
    pub cos: f32,
}

pub fn axle_geometry(vehicle: &car::Vehicle) -> AxleGeometry {
    let spec = &vehicle.spec;
    let delta = vehicle.steering.angle;
    let (sin, cos) = delta.sin_cos();
    AxleGeometry {
        front_offset: spec.chassis.wheelbase * (1.0 - spec.center_of_mass.longitudinal_fraction),
        rear_offset: spec.chassis.wheelbase * spec.center_of_mass.longitudinal_fraction,
        sin,
        cos,
    }
}

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

pub fn tire_capacity(vehicle: &car::Vehicle, axle: Axle) -> f32 {
    let spec = &vehicle.spec;
    // Outside tires gain less grip than inside tires lose. This
    // introduces lateral weight-transfer sensitivity without a full
    // four-wheel rigid-body solver.

    // At rest, axle load is simply `mass * g`.
    // Acceleration/braking adds/removes some load.
    let load = if axle == Axle::Front {
        vehicle.suspension.axle_load.front
    } else {
        vehicle.suspension.axle_load.rear
    };

    let share = if axle == Axle::Front {
        spec.center_of_mass.longitudinal_fraction
    } else {
        1.0 - spec.center_of_mass.longitudinal_fraction
    };

    let transfer = spec.weight_kg
        * car::lateral_acceleration(vehicle).abs()
        * model::car::center_of_mass_height_m(spec)
        / spec.chassis.track_width
        * share;
    let imbalance = (2.0 * transfer / load.max(1.0)).clamp(0.0, 1.0);

    let loc = &vehicle.track;
    let position = vehicle.position;

    let reference = position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
    let deck =
        world::map_details::nearby_decks(&loc.details, util::geometry::xz(position)).find(|deck| {
            world::map_details::height_on(deck, util::geometry::xz(position))
                .is_some_and(|h| (h - reference).abs() < 1.5)
        });

    let material = if let Some(deck) = deck {
        model::map_details::road_material(deck.surface)
    } else {
        world::street_surface::world_location_triangle_at(loc, util::geometry::xz(position), true)
            .map_or(Material::Asphalt, |triangle| triangle.material)
    };

    let _hack = 1.5;

    _hack
        * friction_coefficient(material)
        * load
        * (1.0 - 0.18 * imbalance * imbalance)
        * if axle == Axle::Front {
            FRONT_LIMIT_GRIP
        } else {
            1.0
        }
}

pub fn tire_contacts(
    vehicle: &car::Vehicle,
    controls: car::Controls,
    motion: physics::tire_solver::Motion,
    capacities: [f32; 2],
    drive: f32,
    drivetrain_engaged: bool,
) -> [physics::tire_solver::Contact; 2] {
    [Axle::Front, Axle::Rear].map(|axle| {
        tire_contact(
            vehicle,
            controls,
            motion,
            match axle {
                Axle::Front => capacities[0],
                Axle::Rear => capacities[1],
            },
            drive,
            drivetrain_engaged,
            axle,
        )
    })
}

fn tire_contact(
    vehicle: &car::Vehicle,
    controls: car::Controls,
    motion: physics::tire_solver::Motion,
    capacity: f32,
    drive: f32,
    drivetrain_engaged: bool,
    axle: Axle,
) -> physics::tire_solver::Contact {
    let spec = &vehicle.spec;
    let drive_bias = spec.chassis.drive_bias;
    let AxleGeometry {
        front_offset: a,
        rear_offset: b,
        sin,
        cos,
    } = axle_geometry(vehicle);
    let longitudinal = motion.longitudinal;
    let lateral = motion.lateral;
    let front_speed = longitudinal * cos + (lateral + vehicle.angular_velocity.yaw * a) * sin;
    let brake = vehicle.brake_pressure * spec.chassis.brake_force_n;
    let drive_share = if axle == Axle::Front {
        drive_bias
    } else {
        1.0 - drive_bias
    };
    let wheel_speed = if axle == Axle::Front {
        front_speed
    } else {
        longitudinal
    };
    let longitudinal_slip = if drive_share > 0.0 && drivetrain_engaged {
        vehicle.tires.driven_wheel_omega * spec.chassis.rolling_radius - wheel_speed
    } else {
        0.0
    };
    physics::tire_solver::Contact {
        offset: if axle == Axle::Front { a } else { -b },
        sin: if axle == Axle::Front { sin } else { 0.0 },
        cos: if axle == Axle::Front { cos } else { 1.0 },
        capacity,
        drive_force: drive * drive_share,
        longitudinal_slip,
        brake_force: brake
            * if axle == Axle::Front {
                spec.chassis.brake_bias
            } else {
                1.0 - spec.chassis.brake_bias
            }
            + if axle == Axle::Rear && controls.handbrake {
                spec.weight_kg * 5.5
            } else {
                0.0
            },
        corner_stiffness: physics::tire_solver::corner_stiffness(if axle == Axle::Front {
            vehicle.suspension.axle_load.front
        } else {
            vehicle.suspension.axle_load.rear
        }),
        locked: axle == Axle::Rear && controls.handbrake,
    }
}

pub fn update_feedback(vehicle: &mut car::Vehicle, lateral_speed: f32, handbrake: bool) {
    if vehicle.suspension.axle_load.front + vehicle.suspension.axle_load.rear <= 0.0 {
        vehicle.tires.scrub = 0.0;
        return;
    }
    let motion = vehicle.velocity.x.hypot(vehicle.velocity.z);
    let sliding = ((lateral_speed.abs() - 0.8) / 3.0).clamp(0.0, 1.0);
    let braking = if handbrake { 0.8 } else { 0.0 };
    vehicle.tires.scrub = (sliding.max(braking) * ((motion - 1.5) / 2.5).clamp(0.0, 1.0))
        .max(vehicle.tires.wheel_spin);
}

pub fn update_wheel_spin(vehicle: &mut car::Vehicle, driven_speed: f32) {
    let spec = &vehicle.spec;
    let slip_speed = vehicle.tires.driven_wheel_omega * spec.chassis.rolling_radius - driven_speed;
    vehicle.tires.wheel_spin = (slip_speed.abs() / 8.0).clamp(0.0, 1.0);
}

pub fn update_wheel_rotation(
    vehicle: &mut car::Vehicle,
    longitudinal: f32,
    handbrake: bool,
    seconds: f32,
) {
    let spec = &vehicle.spec;
    let spin = longitudinal * seconds / spec.chassis.rolling_radius;
    let spin_slip =
        (vehicle.tires.driven_wheel_omega - longitudinal / spec.chassis.rolling_radius) * seconds;
    vehicle.tires.wheel_phase[1] = (vehicle.tires.wheel_phase[1]
        + spin
        + if spec.chassis.drive_bias > 0.0 {
            spin_slip
        } else {
            0.0
        })
    .rem_euclid(std::f32::consts::TAU);
    if !handbrake {
        vehicle.tires.wheel_phase[0] = (vehicle.tires.wheel_phase[0]
            + spin
            + if spec.chassis.drive_bias < 1.0 {
                spin_slip
            } else {
                0.0
            })
        .rem_euclid(std::f32::consts::TAU);
    }
}

#[cfg(test)]
#[path = "tests/tires.rs"]
mod tests;
