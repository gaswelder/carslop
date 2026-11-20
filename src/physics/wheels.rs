//! Driven-wheel rotation, advanced only from the wheel input ports.

use crate::model;

pub const INERTIA_KG_M2: f32 = 2.4;

pub struct State {
    /// Signed angular speed in rad/s; positive rolls the car forward.
    pub omega: f32,
    /// Cosmetic rear/front axle rotation angles, in radians.
    pub phase: [f32; 2],
    /// Transmission drive torque, in N m.
    pub in_torque: f32,
    /// Force exerted on the road along the wheel, in N; its reaction loads the wheel.
    pub in_force: f32,
    pub in_brake_torque: f32,
    /// Road speed along the driven wheels, in m/s.
    pub in_speed: f32,
    pub in_body_speed: f32,
    pub in_engaged: bool,
    pub in_handbrake: bool,
}

pub fn new() -> State {
    State {
        omega: 0.0,
        phase: [0.0; 2],
        in_torque: 0.0,
        in_force: 0.0,
        in_brake_torque: 0.0,
        in_speed: 0.0,
        in_body_speed: 0.0,
        in_engaged: false,
        in_handbrake: false,
    }
}

pub fn update(state: &mut State, spec: &model::car::CarSpec, dt: f32) {
    if dt <= 0.0 {
        return;
    }
    if state.in_engaged {
        state.omega -= state.in_force * spec.chassis.rolling_radius * dt / INERTIA_KG_M2;
        state.omega += state.in_brake_torque * dt / INERTIA_KG_M2;
        state.omega += state.in_torque * dt / INERTIA_KG_M2;
    } else {
        state.omega = state.in_speed / spec.chassis.rolling_radius;
    }
    let spin = state.in_body_speed * dt / spec.chassis.rolling_radius;
    let slip = (state.omega - state.in_body_speed / spec.chassis.rolling_radius) * dt;
    state.phase[1] = (state.phase[1]
        + spin
        + if spec.chassis.drive_bias > 0.0 {
            slip
        } else {
            0.0
        })
    .rem_euclid(std::f32::consts::TAU);
    if !state.in_handbrake {
        state.phase[0] = (state.phase[0]
            + spin
            + if spec.chassis.drive_bias < 1.0 {
                slip
            } else {
                0.0
            })
        .rem_euclid(std::f32::consts::TAU);
    }
}

#[cfg(test)]
#[path = "tests/wheels.rs"]
mod tests;
