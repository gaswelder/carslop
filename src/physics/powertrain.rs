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

#[cfg(test)]
mod tests {
    use crate::physics;

    #[test]
    fn brakes_balance_tire_reaction_and_abs_prevents_wheel_lock() {
        for direction in [-1.0, 1.0] {
            let step = physics::powertrain::new(
                3000.0,
                20.0 * direction,
                0.0,
                0.25,
                12.0,
                0.0,
                1.0 / 240.0,
            );
            let rolling_speed = 19.5 * 0.32 * direction;
            let rotation = physics::powertrain::finish(
                &step,
                -4000.0 * direction,
                0.32,
                8000.0,
                rolling_speed,
            );
            assert!((rotation.wheel_omega * 0.32 - rolling_speed).abs() < 0.00001);
            assert!((rotation.engine_rpm - 3000.0).abs() < 0.001);

            let released =
                physics::powertrain::finish(&step, 0.0, 0.32, 8000.0, 25.0 * 0.32 * direction);
            assert!((released.wheel_omega - 20.0 * direction).abs() < 0.00001);
            let weak = physics::powertrain::finish(
                &step,
                -4000.0 * direction,
                0.32,
                1000.0,
                rolling_speed,
            );
            assert!(weak.wheel_omega * direction > 20.0);
        }
    }

    #[test]
    fn static_brake_and_clutch_exchange_torque_without_rotating_the_wheels() {
        let step = physics::powertrain::new(3000.0, 0.0, 0.0, 0.25, 12.0, 100.0, 1.0 / 240.0);
        let held = physics::powertrain::finish(&step, 0.0, 0.32, 8000.0, 0.0);
        assert!(held.wheel_omega.abs() < 0.00001);
        assert!(held.engine_rpm < 3000.0);
        let released = physics::powertrain::finish(&step, 0.0, 0.32, 0.0, 0.0);
        assert!(released.wheel_omega > 0.0);
    }

    #[test]
    fn locked_clutch_and_tires_preserve_gearing_and_exchange_momentum() {
        for dt in [1.0 / 60.0, 1.0 / 240.0] {
            let step = physics::powertrain::new(
                3000.0,
                3000.0 * std::f32::consts::TAU / 60.0 / 12.0,
                150.0,
                0.25,
                12.0,
                1000.0,
                dt,
            );
            let speed = step.wheel_omega * 0.32;
            let force = physics::powertrain::road_force(&step, speed, 1200.0, 0.32);
            let rotation = physics::powertrain::finish(&step, force, 0.32, 0.0, 0.0);
            let rpm = rotation.engine_rpm;
            let wheel = rotation.wheel_omega;
            assert!((wheel * 0.32 - (speed + force / 1200.0 * dt)).abs() < 0.0001);
            assert!((rpm - wheel * 12.0 * 60.0 / std::f32::consts::TAU).abs() < 0.01);
            let momentum_before =
                0.25 * 3000.0 * std::f32::consts::TAU / 60.0 + 2.4 * step.wheel_omega / 12.0;
            let momentum_after = 0.25 * rpm * std::f32::consts::TAU / 60.0 + 2.4 * wheel / 12.0;
            assert!(
                (momentum_after - momentum_before - (150.0 - force * 0.32 / 12.0) * dt).abs()
                    < 0.0001
            );
        }
    }

    #[test]
    fn slipping_clutch_transfers_momentum_and_dissipates_energy() {
        let step = physics::powertrain::new(6000.0, 0.0, 0.0, 0.25, 12.0, 400.0, 1.0 / 240.0);
        let rotation = physics::powertrain::finish(&step, 0.0, 0.32, 0.0, 0.0);
        let rpm = rotation.engine_rpm;
        let wheel = rotation.wheel_omega;
        let omega = rpm * std::f32::consts::TAU / 60.0;
        assert!(rpm < 6000.0 && wheel > 0.0);
        assert!((0.25 * omega + 2.4 * wheel / 12.0 - 0.25 * step.engine_omega).abs() < 0.0001);
        assert!(
            0.5 * 0.25 * omega * omega + 0.5 * 2.4 * wheel * wheel
                < 0.5 * 0.25 * step.engine_omega * step.engine_omega
        );
    }

    #[test]
    fn lost_grip_accelerates_rotating_parts_and_open_clutch_isolates_engine() {
        let loaded = physics::powertrain::new(3000.0, 20.0, 150.0, 0.25, 12.0, 500.0, 1.0 / 240.0);
        let force = physics::powertrain::road_force(&loaded, 6.4, 1200.0, 0.32);
        let grip = physics::powertrain::finish(&loaded, force, 0.32, 0.0, 0.0);
        let slip = physics::powertrain::finish(&loaded, 0.0, 0.32, 0.0, 0.0);
        assert!(slip.engine_rpm > grip.engine_rpm && slip.wheel_omega > grip.wheel_omega);
        let open = physics::powertrain::new(3000.0, 20.0, 150.0, 0.25, 12.0, 0.0, 1.0 / 240.0);
        assert_eq!(
            physics::powertrain::finish(&(open), 0.0, 0.32, 0.0, 0.0).engine_rpm,
            physics::powertrain::finish(&(open), 5000.0, 0.32, 0.0, 0.0).engine_rpm
        );
    }
}
