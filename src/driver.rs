//! Translate requested controls into pedal inputs before the physics solve.

use crate::physics::car;
use crate::physics::engine;
use crate::physics::powertrain;

const REV_MATCH_SECONDS: f32 = 0.22;

#[derive(Clone, Copy)]
pub struct Assistance {
    pub lift_on_shift: bool,
    pub rev_matching: bool,
}

pub struct DriverState {
    pub assistance: Assistance,
    /// Remaining clutch release time after a gear change, in seconds.
    shift_timer: f32,
    rev_match_timer: f32,
}

pub fn new(assistance: Assistance) -> DriverState {
    DriverState {
        assistance,
        shift_timer: 0.0,
        rev_match_timer: 0.0,
    }
}

pub fn is_shifting(state: &DriverState) -> bool {
    state.shift_timer > 0.0
}

/// Sample assisted controls once per physics substep. Advance clutch release
/// before sampling and decay the blip afterward so every solver uses the same
/// throttle and clutch state throughout the step.
pub fn update(
    state: &mut DriverState,
    gearbox: &powertrain::GearboxState,
    requested: car::Controls,
    dt: f32,
) -> car::Controls {
    state.shift_timer = (state.shift_timer - dt).max(0.0);
    let mut throttle = requested.throttle.clamp(0.0, 1.0);
    if is_shifting(state) && gearbox.gear != powertrain::NEUTRAL_GEAR {
        if state.assistance.lift_on_shift {
            throttle = 0.0;
        }
        if state.assistance.rev_matching {
            throttle = throttle.max(0.85 * state.rev_match_timer / REV_MATCH_SECONDS);
        }
    }
    state.rev_match_timer = (state.rev_match_timer - dt).max(0.0);
    car::Controls {
        throttle,
        ..requested
    }
}

/// Request a gear change and prepare the driver's throttle blip if needed.
pub fn shift(vehicle: &mut car::Vehicle, up: bool) {
    let previous = vehicle.gearbox.gear;
    let next = match (previous, up) {
        (powertrain::REVERSE_GEAR, true) => powertrain::NEUTRAL_GEAR,
        (powertrain::NEUTRAL_GEAR, true) => 1,
        (powertrain::NEUTRAL_GEAR, false) => powertrain::REVERSE_GEAR,
        (1, false) => powertrain::NEUTRAL_GEAR,
        (powertrain::REVERSE_GEAR, false) => powertrain::REVERSE_GEAR,
        (gear, true) => (gear + 1).min(vehicle.spec.gearbox.ratios.len() as u8),
        (gear, false) => gear - 1,
    };
    if next == previous {
        return;
    }
    vehicle.gearbox.gear = next;
    let launching = previous == powertrain::NEUTRAL_GEAR
        && matches!(vehicle.gearbox.gear, powertrain::REVERSE_GEAR | 1);
    vehicle.driver.shift_timer = if launching { 0.0 } else { 0.16 };
    // Blip only for a moving forward downshift whose target RPM is higher.
    let rev_match = vehicle.driver.assistance.rev_matching
        && vehicle.gearbox.gear != powertrain::REVERSE_GEAR
        && vehicle.gearbox.gear < previous
        && usize::from(previous) <= vehicle.spec.gearbox.ratios.len()
        && powertrain::coupled_rpm(
            &vehicle.spec,
            car::longitudinal_speed(vehicle),
            vehicle.gearbox.gear,
        )
        .max(engine::IDLE_RPM)
            > vehicle.engine.rpm + 100.0;
    vehicle.driver.rev_match_timer = if rev_match { REV_MATCH_SECONDS } else { 0.0 };
}

#[cfg(test)]
mod tests;
