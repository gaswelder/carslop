//! Chassis motion driven by force and torque inputs.

use super::suspension;
use crate::asset;
use crate::model;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

const AIR_DENSITY_KG_M3: f32 = 1.225;

/// Pitch, roll, and yaw components of orientation or its rate of change.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Orientation {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

pub struct State {
    pub position: XYZ,
    pub velocity: XYZ,
    pub acceleration: XYZ,
    pub orientation: Orientation,
    pub angular_velocity: Orientation,
    /// Resultant world-space force in N, including gravity and suspension support.
    pub in_force: XYZ,
    /// Resultant body torque in N m.
    pub in_torque: Orientation,
    /// Applied rolling and aerodynamic resistance, in N.
    pub out_resistance: f32,
    /// World-space acceleration from drag acting through the center of mass.
    pub out_aerodynamic_acceleration: XYZ,
}

pub fn new(position: XYZ, heading: f32) -> State {
    State {
        position,
        velocity: XYZ::default(),
        acceleration: XYZ::default(),
        orientation: Orientation {
            yaw: heading,
            ..Orientation::default()
        },
        angular_velocity: Orientation::default(),
        in_force: XYZ::default(),
        in_torque: Orientation::default(),
        out_resistance: 0.0,
        out_aerodynamic_acceleration: XYZ::default(),
    }
}

/// Integrate the supplied forces and torques once per substep.
/// The caller replaces the inputs before each update.
pub fn update(state: &mut State, spec: &model::car::CarSpec, dt: f32) {
    if dt <= 0.0 {
        return;
    }
    let (sin, cos) = state.orientation.yaw.sin_cos();
    let speed = state.velocity.x * sin + state.velocity.z * cos;
    let longitudinal_force = state.in_force.x * sin + state.in_force.z * cos;
    let lateral_force = state.in_force.x * cos - state.in_force.z * sin;
    let longitudinal = speed + longitudinal_force / spec.weight_kg * dt;
    let lateral =
        state.velocity.x * cos - state.velocity.z * sin + lateral_force / spec.weight_kg * dt;
    let resistance = resistance_force(spec, speed) / spec.weight_kg * dt;
    let new_longitudinal = longitudinal.signum() * (longitudinal.abs() - resistance).max(0.0);
    state.out_resistance = if dt > 0.0 {
        (longitudinal - new_longitudinal).abs() * spec.weight_kg / dt
    } else {
        0.0
    };
    // Partition the resistance actually applied, including the stop-at-zero
    // limit. Rolling resistance acts at the road; aerodynamic drag acts at CG.
    let aerodynamic_share = aerodynamic_drag(spec, speed) / resistance_force(spec, speed);
    let aerodynamic_acceleration = (new_longitudinal - longitudinal) / dt * aerodynamic_share;
    state.out_aerodynamic_acceleration = XYZ {
        x: sin * aerodynamic_acceleration,
        y: 0.0,
        z: cos * aerodynamic_acceleration,
    };
    state.velocity.x = sin * new_longitudinal + cos * lateral;
    state.velocity.z = cos * new_longitudinal - sin * lateral;
    state.velocity.y += state.in_force.y / spec.weight_kg * dt;
    state.angular_velocity.yaw += state.in_torque.yaw / spec.yaw_inertia_kg_m2 * dt;
    // Pitch and roll use the existing sprung-body inertia approximation.
    state.angular_velocity.pitch += state.in_torque.pitch / spec.weight_kg * dt;
    state.angular_velocity.roll += state.in_torque.roll / spec.weight_kg * dt;
    state.orientation.pitch += state.angular_velocity.pitch * dt;
    state.orientation.roll += state.angular_velocity.roll * dt;
    state.orientation.yaw =
        (state.orientation.yaw + state.angular_velocity.yaw * dt).rem_euclid(std::f32::consts::TAU);
    state.position.x += state.velocity.x * dt;
    state.position.y += state.velocity.y * dt;
    state.position.z += state.velocity.z * dt;
}

pub fn measure_acceleration(state: &mut State, previous_velocity: XYZ, dt: f32) {
    state.acceleration = XYZ {
        x: (state.velocity.x - previous_velocity.x) / dt,
        y: (state.velocity.y - previous_velocity.y) / dt,
        z: (state.velocity.z - previous_velocity.z) / dt,
    };
}

/// Approximate an inelastic tyre/corner collision for each wheel crossing.
/// Removing velocity into the corner normal dissipates energy; the remaining
/// tangent velocity lifts the chassis. Wheel load shares distribute the impulse
/// without applying a whole-car collision four times.
pub fn resolve_curbs(
    state: &mut State,
    spec: &model::car::CarSpec,
    track: &world::locations::WorldLocation,
    previous_position: XYZ,
    previous_yaw: f32,
) -> f32 {
    if !world::street_surface::has_compiled_roads(track)
        || state.position.y
            - suspension::surface_height(spec, track, state.position, state.orientation.yaw)
            > 0.16
    {
        return 0.0;
    }
    let velocity = state.velocity;
    let mut change: XYZ = XYZ::default();
    for index in 0..4 {
        let a = suspension::wheel_position(spec, previous_position, previous_yaw, index);
        let b = suspension::wheel_position(spec, state.position, state.orientation.yaw, index);
        let reference = state.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
        if world::transport::deck_height(track, b, reference)
            .is_some_and(|height| (height - reference).abs() < 1.0)
        {
            continue;
        }
        let Some(crossing) = physics::vehicle_location::crossed_curb(track, a, b) else {
            continue;
        };
        let radius = spec.design.wheel_radii[index];
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
        let impulse = closing * suspension::wheel_share(spec, index);
        change.x -= crossing.normal.x * direction * horizontal * impulse;
        change.z -= crossing.normal.z * direction * horizontal * impulse;
        change.y += up * impulse;
    }
    state.velocity.x += change.x;
    state.velocity.y += change.y;
    state.velocity.z += change.z;
    change.x.hypot(change.z)
}

pub fn resolve_road(
    state: &mut State,
    spec: &model::car::CarSpec,
    track: &world::locations::WorldLocation,
) -> f32 {
    let mut velocity = util::geometry::xz(state.velocity);
    let impact = constrain_road(state, spec, track, &mut velocity);
    state.velocity.x = velocity.x;
    state.velocity.z = velocity.z;
    impact
}

fn constrain_road(
    state: &mut State,
    spec: &model::car::CarSpec,
    track: &world::locations::WorldLocation,
    velocity: &mut XZ<f32>,
) -> f32 {
    if world::street_surface::has_compiled_roads(track) {
        let city = track;
        return physics::vehicle_location::constrain(
            city,
            &mut state.position,
            velocity,
            asset::car_glb::height(&spec.design),
        );
    }
    let bounds = world::elevation::terrain_bounds(track);
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
            util::geometry::xyz_component(state.position, coordinate).clamp(limit[0], limit[1]);
        let excess = util::geometry::xyz_component(state.position, coordinate) - clamped;
        if excess.abs() > 0.0001 {
            *util::geometry::xyz_component_mut(&mut state.position, coordinate) = clamped;
            let mut normal = XZ { x: 0.0, z: 0.0 };
            *util::geometry::xz_component_mut(&mut normal, axis) = excess.signum();
            impact = impact.max(physics::vehicle_location::deflect(velocity, normal));
        }
    }
    impact = impact.max(world::trees::constrain(
        track,
        &mut state.position,
        velocity,
        asset::car_glb::height(&spec.design),
    ));
    state.position.y =
        suspension::surface_height(spec, track, state.position, state.orientation.yaw);
    impact
}

fn dot(a: XZ<f32>, b: XZ<f32>) -> f32 {
    a.x * b.x + a.z * b.z
}

fn resistance_force(spec: &model::car::CarSpec, speed: f32) -> f32 {
    0.25 * spec.weight_kg + aerodynamic_drag(spec, speed)
}

fn aerodynamic_drag(spec: &model::car::CarSpec, speed: f32) -> f32 {
    0.5 * AIR_DENSITY_KG_M3 * spec.drag_coefficient * spec.frontal_area_m2 * speed * speed
}

#[cfg(test)]
#[path = "tests/chassis.rs"]
mod tests;
