use crate::model;

pub const IDLE_RPM: f32 = 900.0;

/// Runtime engine simulation and feedback state.
pub struct EngineState {
    spec: model::car::Engine,
    pub rpm: f32,
    /// Output torque for the current step, before the gearbox reaction, in N m.
    pub torque_nm: f32,
    /// Smoothed engine load used for sound, from zero to one.
    pub load: f32,
    /// Normalized turbine speed and available boost.
    pub turbo_spool: f32,
    // Normalized manifold boost.
    pub turbo_boost: f32,
    fuel_cut: bool,
    throttle: f32,
}

pub fn new(spec: &model::car::Engine) -> EngineState {
    let mut state = EngineState {
        spec: spec.clone(),
        rpm: IDLE_RPM,
        torque_nm: 0.0,
        load: 0.0,
        turbo_spool: 0.0,
        turbo_boost: 0.0,
        fuel_cut: false,
        throttle: 0.0,
    };
    update(&mut state, 0.0);
    state
}

/// Publish output torque at the current RPM for the upcoming drivetrain solve.
pub fn update(state: &mut EngineState, throttle: f32) {
    state.throttle = throttle.clamp(0.0, 1.0);
    update_fuel_cut(state);
    state.torque_nm = net_torque(state);
}

/// Advance rotation using the published torque and signed gearbox impulse (N m s).
/// Positive impulse drives the crankshaft; negative impulse loads it.
/// An open clutch supplies zero impulse, including in neutral.
/// Call after updating engine output and solving the drivetrain.
pub fn advance(state: &mut EngineState, crankshaft_impulse: f32, dt: f32) {
    let throttle = state.throttle;
    // A resisting crankshaft reaction provides mechanical load. An open
    // clutch produces no reaction, regardless of the selected gear.
    let loaded = crankshaft_impulse < 0.0;
    update_turbo(state, loaded, dt, throttle);
    update_rpm(state, crankshaft_impulse, dt);
    update_feedback(state, loaded, throttle, dt);
}

fn update_fuel_cut(state: &mut EngineState) {
    if state.rpm >= state.spec.redline_rpm {
        state.fuel_cut = true;
    } else if state.rpm <= state.spec.redline_rpm - 250.0 {
        state.fuel_cut = false;
    }
}

fn update_rpm(state: &mut EngineState, crankshaft_impulse: f32, dt: f32) {
    let omega = state.rpm * std::f32::consts::TAU / 60.0;
    let omega = omega + state.torque_nm / state.spec.inertia_kg_m2 * dt;
    let omega = omega + crankshaft_impulse / state.spec.inertia_kg_m2;
    state.rpm = (omega * 60.0 / std::f32::consts::TAU).max(IDLE_RPM);
}

fn engine_torque(spec: &model::car::Engine, rpm: f32) -> f32 {
    (spec.max_torque_nm * model::car::torque_factor(&spec.torque_curve, rpm))
        .min(spec.power_watts / (rpm.max(IDLE_RPM) * std::f32::consts::TAU / 60.0))
}

fn net_torque(state: &EngineState) -> f32 {
    let spec = &state.spec;
    let revs = ((state.rpm - IDLE_RPM) / (spec.redline_rpm - IDLE_RPM)).max(0.0);
    let drag = spec.max_torque_nm * (0.10 + 0.16 * revs + 0.24 * revs * revs);
    let combustion = if state.fuel_cut {
        0.0
    } else {
        state.throttle * (engine_torque(spec, state.rpm) + drag)
    };
    combustion - drag
}

fn update_turbo(state: &mut EngineState, loaded: bool, dt: f32, throttle: f32) {
    let Some(tune) = &state.spec.turbo else {
        state.turbo_spool = 0.0;
        state.turbo_boost = 0.0;
        return;
    };
    let rpm = state.rpm;
    let exhaust = ((rpm - tune.spool_start_rpm) / (tune.full_boost_rpm - tune.spool_start_rpm))
        .clamp(0.0, 1.0);
    let target = if loaded {
        exhaust * throttle * throttle
    } else {
        0.0
    };
    let time = if target > state.turbo_spool {
        tune.spool_seconds
    } else {
        tune.decay_seconds
    };
    state.turbo_spool += (target - state.turbo_spool) * (1.0 - (-dt / time).exp());
    // Closing the throttle vents manifold pressure while the turbine coasts.
    let boost_target = if loaded {
        state.turbo_spool * throttle
    } else {
        0.0
    };
    state.turbo_boost += (boost_target - state.turbo_boost) * (1.0 - (-dt / 0.06).exp());
}

fn update_feedback(state: &mut EngineState, loaded: bool, throttle: f32, dt: f32) {
    // Sound follows actual throttle and crankshaft load, including an
    // unloaded throttle blip and the absence of combustion during fuel cut.
    let requested = if state.fuel_cut {
        0.0
    } else {
        throttle * if loaded { 1.0 } else { 0.55 }
    };
    let target_load = (requested + 0.12 * (1.0 - requested)).clamp(0.0, 1.0);
    state.load += (target_load - state.load) * (1.0 - (-dt * 10.0).exp());
}

#[cfg(test)]
#[path = "tests/engine.rs"]
mod tests;
