use super::engine;
use super::wheels;
use crate::model;

// Fraction of transmitted power retained by the gearbox and differential.
const DRIVETRAIN_EFFICIENCY: f32 = 0.90;

// Forward ratios use 1..=N.
pub const REVERSE_GEAR: u8 = 0;
pub const NEUTRAL_GEAR: u8 = u8::MAX;

pub struct State {
    spec: std::sync::Arc<model::car::CarSpec>,
    pub gear: u8,
    pub in_engine_rpm: f32,
    pub in_wheel_omega: f32,
    pub in_speed: f32,
    pub in_shifting: bool,
    pub in_force: f32,
    pub in_brake_force: f32,
    pub in_rolling_speed: f32,
    /// Signed wheel drive torque and crankshaft reaction, in N m.
    pub out_torque: f32,
    pub out_reaction_torque: f32,
    /// Signed wheel brake reaction from the joint clutch/ABS constraint, in N m.
    pub out_brake_torque: f32,
}

pub fn new(spec: &std::sync::Arc<model::car::CarSpec>) -> State {
    State {
        spec: spec.clone(),
        gear: 1,
        in_engine_rpm: engine::IDLE_RPM,
        in_wheel_omega: 0.0,
        in_speed: 0.0,
        in_shifting: false,
        in_force: 0.0,
        in_brake_force: 0.0,
        in_rolling_speed: 0.0,
        out_torque: 0.0,
        out_reaction_torque: 0.0,
        out_brake_torque: 0.0,
    }
}

/// Publish the torques from the current shaft and contact inputs.
/// Every update replaces all outputs, including when the clutch is open.
pub fn update(state: &mut State, dt: f32) {
    state.out_torque = 0.0;
    state.out_reaction_torque = 0.0;
    state.out_brake_torque = 0.0;
    if dt <= 0.0 {
        return;
    }
    if let Some(step) = coupling(state, dt) {
        let direction = direction(state);
        let rotation = finish(
            &step,
            state.in_force * direction,
            state.spec.chassis.rolling_radius,
            state.in_brake_force,
            state.in_rolling_speed * direction,
        );
        state.out_torque = rotation.wheel_torque * direction;
        state.out_reaction_torque = rotation.crankshaft_impulse / dt;
        state.out_brake_torque = rotation.brake_torque * direction;
    }
}

pub fn engaged(state: &State) -> bool {
    state.gear != NEUTRAL_GEAR && !state.in_shifting
}

/// Joint clutch/rolling-contact force demand, before the tire grip limit.
/// This stateless constraint query does not advance any component's motion.
pub fn requested_force(state: &State, dt: f32) -> f32 {
    if dt <= 0.0 {
        return 0.0;
    }
    coupling(state, dt).map_or(0.0, |step| {
        let direction = direction(state);
        road_force(
            &step,
            state.in_speed * direction,
            state.spec.weight_kg,
            state.spec.chassis.rolling_radius,
        ) * direction
    })
}

fn direction(state: &State) -> f32 {
    if state.gear == REVERSE_GEAR {
        -1.0
    } else {
        1.0
    }
}

struct Rotation {
    /// Signed reaction impulse on the crankshaft, in N m s.
    crankshaft_impulse: f32,
    #[cfg(test)]
    wheel_omega: f32,
    wheel_torque: f32,
    brake_torque: f32,
}

struct Step {
    /// Current crankshaft speed after the engine update.
    engine_omega: f32,
    wheel_omega: f32,
    engine_inertia: f32,
    /// Effective inertia of the driven wheels and differential (kg m²).
    wheel_inertia: f32,
    ratio: f32,
    max_impulse: f32,
    dt: f32,
}

/// Engine RPM matching road speed in the selected gear with the clutch fully engaged.
pub fn coupled_rpm(car_spec: &model::car::CarSpec, speed: f32, gear: u8) -> f32 {
    let gearbox = &car_spec.gearbox;
    let ratio = if gear == REVERSE_GEAR {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[gear.saturating_sub(1).min(gearbox.ratios.len() as u8 - 1) as usize]
    };
    let speed = if gear == REVERSE_GEAR {
        speed.abs()
    } else {
        speed.max(0.0)
    };
    speed * 60.0 / (std::f32::consts::TAU * car_spec.chassis.rolling_radius)
        * ratio
        * gearbox.final_drive
}

fn coupling(state: &State, dt: f32) -> Option<Step> {
    if !engaged(state) {
        return None;
    }
    let spec = &state.spec;
    let gearbox = &spec.gearbox;
    let ratio = if state.gear == REVERSE_GEAR {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[(state.gear - 1) as usize]
    } * gearbox.final_drive;
    // Automatic clutch opens near idle to avoid stalling. Its torque capacity
    // increases with engine speed; there is no launch RPM target.
    let engagement = if coupled_rpm(spec, state.in_speed, state.gear) > engine::IDLE_RPM + 100.0 {
        1.0
    } else {
        ((state.in_engine_rpm - engine::IDLE_RPM) / 600.0).clamp(0.0, 1.0)
    };
    let direction = if state.gear == REVERSE_GEAR {
        -1.0
    } else {
        1.0
    };
    Some(Step {
        engine_omega: state.in_engine_rpm * std::f32::consts::TAU / 60.0,
        wheel_omega: state.in_wheel_omega * direction,
        engine_inertia: spec.engine.inertia_kg_m2,
        wheel_inertia: wheels::INERTIA_KG_M2,
        ratio,
        max_impulse: spec.engine.max_torque_nm * 2.0 * engagement * dt,
        dt,
    })
}

fn road_force(step: &Step, speed: f32, mass: f32, radius: f32) -> f32 {
    // Solve clutch lock and rolling contact together. If clutch capacity
    // is exceeded, solve rolling contact with the slipping clutch impulse.
    let engine_slip = step.engine_omega - step.ratio * step.wheel_omega;
    let tire_slip = radius * step.wheel_omega - speed;
    let b = step.ratio * radius / step.wheel_inertia;
    let c = radius * radius / step.wheel_inertia + 1.0 / mass;
    let slip = engine_slip + b * tire_slip / c;
    let torque_ratio = torque_ratio(step, slip, speed / radius);
    let a = 1.0 / step.engine_inertia + step.ratio * torque_ratio / step.wheel_inertia;
    let wheel_coupling = torque_ratio * radius / step.wheel_inertia;
    let clutch = (slip / (a - b * wheel_coupling / c)).clamp(-step.max_impulse, step.max_impulse);
    (tire_slip + wheel_coupling * clutch) / c / step.dt
}

fn finish(
    step: &Step,
    tire_force: f32,
    radius: f32,
    brake_force: f32,
    rolling_speed: f32,
) -> Rotation {
    // Actual grip may be lower than requested. Its equal and opposite
    // reaction changes wheel speed and therefore the clutch load.
    let wheel = step.wheel_omega - tire_force * radius * step.dt / step.wheel_inertia;
    // Ideal ABS releases brake torque before a rolling wheel locks. Brake
    // impulse can oppose rotation or hold rest, but cannot accelerate a
    // wheel toward road speed. Solve its bound together with clutch lock.
    let target = rolling_speed / radius;
    let limit = brake_force * radius * step.dt;
    let minimum = if target > 0.0 { 0.0 } else { -limit };
    let maximum = if target < 0.0 { 0.0 } else { limit };
    let rolling_clutch = ((step.engine_omega - step.ratio * target) * step.engine_inertia)
        .clamp(-step.max_impulse, step.max_impulse);
    let rolling_ratio = torque_ratio(step, rolling_clutch, target);
    let brake = (step.wheel_inertia * (wheel - target) + rolling_ratio * rolling_clutch)
        .clamp(minimum, maximum);
    let wheel = wheel - brake / step.wheel_inertia;
    let slip = step.engine_omega - step.ratio * wheel;
    let torque_ratio = torque_ratio(step, slip, if brake != 0.0 { target } else { wheel });
    let clutch = (slip
        / (1.0 / step.engine_inertia + step.ratio * torque_ratio / step.wheel_inertia))
        .clamp(-step.max_impulse, step.max_impulse);
    Rotation {
        crankshaft_impulse: -clutch,
        #[cfg(test)]
        wheel_omega: wheel + clutch * torque_ratio / step.wheel_inertia,
        wheel_torque: clutch * torque_ratio / step.dt,
        brake_torque: -brake / step.dt,
    }
}

// Speed ratios remain geometric. Torque ratios include losses in the direction
// of power flow: driving the wheels retains 90%; back-driving requires more
// wheel torque to deliver the same crankshaft torque. At rest, the wheel starts
// rotating in the direction of its impulse, so power flows toward the wheel.
fn torque_ratio(step: &Step, clutch: f32, wheel_omega: f32) -> f32 {
    if clutch * wheel_omega >= 0.0 {
        step.ratio * DRIVETRAIN_EFFICIENCY
    } else {
        step.ratio / DRIVETRAIN_EFFICIENCY
    }
}

#[cfg(test)]
#[path = "tests/powertrain.rs"]
mod tests;
