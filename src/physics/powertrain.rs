pub struct Rotation {
    pub engine_rpm: f32,
    pub wheel_omega: f32,
}

pub struct Step {
    engine_omega: f32,
    wheel_omega: f32,
    engine_inertia: f32,
    wheel_inertia: f32,
    ratio: f32,
    max_impulse: f32,
    dt: f32,
}

pub fn new(
    rpm: f32,
    wheel_omega: f32,
    engine_torque: f32,
    engine_inertia: f32,
    ratio: f32,
    clutch_capacity: f32,
    dt: f32,
) -> Step {
    Step {
        engine_omega: rpm * std::f32::consts::TAU / 60.0 + engine_torque / engine_inertia * dt,
        wheel_omega,
        engine_inertia,
        // Effective inertia of the driven wheels and differential (kg m²).
        wheel_inertia: 2.4,
        ratio,
        max_impulse: clutch_capacity * dt,
        dt,
    }
}

pub fn road_force(step: &Step, speed: f32, mass: f32, radius: f32) -> f32 {
    // Solve clutch lock and rolling contact together. If clutch capacity
    // is exceeded, solve rolling contact with the slipping clutch impulse.
    let a = 1.0 / step.engine_inertia + step.ratio * step.ratio / step.wheel_inertia;
    let b = step.ratio * radius / step.wheel_inertia;
    let c = radius * radius / step.wheel_inertia + 1.0 / mass;
    let engine_slip = step.engine_omega - step.ratio * step.wheel_omega;
    let tire_slip = radius * step.wheel_omega - speed;
    let clutch = ((engine_slip + b * tire_slip / c) / (a - b * b / c))
        .clamp(-step.max_impulse, step.max_impulse);
    (tire_slip + b * clutch) / c / step.dt
}

pub fn finish(
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
    let brake = (step.wheel_inertia * (wheel - target) + step.ratio * rolling_clutch)
        .clamp(minimum, maximum);
    let clutch = ((step.engine_omega - step.ratio * (wheel - brake / step.wheel_inertia))
        / (1.0 / step.engine_inertia + step.ratio * step.ratio / step.wheel_inertia))
        .clamp(-step.max_impulse, step.max_impulse);
    let engine = step.engine_omega - clutch / step.engine_inertia;
    let wheel = wheel + (clutch * step.ratio - brake) / step.wheel_inertia;
    Rotation {
        engine_rpm: engine * 60.0 / std::f32::consts::TAU,
        wheel_omega: wheel,
    }
}
