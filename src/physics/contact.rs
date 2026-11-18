//! Build axle contacts from vehicle state and the road surface.

use super::car;
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

pub struct AxleGeometry {
    pub front_offset: f32,
    pub rear_offset: f32,
    pub sin: f32,
    pub cos: f32,
}

pub fn axle_geometry(vehicle: &car::Vehicle) -> AxleGeometry {
    let spec = &vehicle.spec;
    let delta = car::front_wheel_angle(vehicle);
    let (sin, cos) = delta.sin_cos();
    AxleGeometry {
        front_offset: spec.chassis.wheelbase * (1.0 - spec.center_of_mass.longitudinal_fraction),
        rear_offset: spec.chassis.wheelbase * spec.center_of_mass.longitudinal_fraction,
        sin,
        cos,
    }
}

fn brake_accel(vehicle: &car::Vehicle) -> f32 {
    vehicle.spec.chassis.brake_force_n / vehicle.spec.weight_kg
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

pub fn tire_capacities(vehicle: &car::Vehicle) -> [f32; 2] {
    let spec = &vehicle.spec;
    [0, 1].map(|axle| {
        // Outside tires gain less grip than inside tires lose. This
        // introduces lateral weight-transfer sensitivity without a full
        // four-wheel rigid-body solver.
        let load = vehicle.axle_load[axle];
        let share = if axle == 0 {
            spec.center_of_mass.longitudinal_fraction
        } else {
            1.0 - spec.center_of_mass.longitudinal_fraction
        };
        let transfer = spec.weight_kg
            * vehicle.filtered_acceleration.lateral.abs()
            * model::car::center_of_mass_height_m(spec)
            / spec.chassis.track_width
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
    })
}

pub fn tire_contacts(
    vehicle: &car::Vehicle,
    controls: car::Controls,
    motion: physics::tire::Motion,
    capacities: [f32; 2],
    drive: f32,
    drivetrain_engaged: bool,
) -> [physics::tire::Contact; 2] {
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
    let front_speed = longitudinal * cos + (lateral + vehicle.orientation_rate.yaw * a) * sin;
    let brake = vehicle.brake_pressure * brake_accel(vehicle) * spec.weight_kg;
    [0, 1].map(|axle| {
        let capacity = capacities[axle];
        let drive_share = if axle == 0 {
            drive_bias
        } else {
            1.0 - drive_bias
        };
        let wheel_speed = if axle == 0 { front_speed } else { longitudinal };
        let longitudinal_slip = if drive_share > 0.0 && drivetrain_engaged {
            vehicle.driven_wheel_omega * spec.chassis.rolling_radius - wheel_speed
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
                    spec.chassis.brake_bias
                } else {
                    1.0 - spec.chassis.brake_bias
                }
                + if axle == 1 && controls.handbrake {
                    spec.weight_kg * 5.5
                } else {
                    0.0
                },
            corner_stiffness: physics::tire::corner_stiffness(vehicle.axle_load[axle]),
            locked: axle == 1 && controls.handbrake,
        }
    })
}

#[cfg(test)]
#[path = "tests/contact.rs"]
mod tests;
