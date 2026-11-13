pub struct Step {
    engine_omega: f32,
    wheel_omega: f32,
    engine_inertia: f32,
    wheel_inertia: f32,
    ratio: f32,
    max_impulse: f32,
    dt: f32,
}

impl Step {
    pub fn new(
        rpm: f32,
        wheel_omega: f32,
        engine_torque: f32,
        engine_inertia: f32,
        ratio: f32,
        clutch_capacity: f32,
        dt: f32,
    ) -> Self {
        Self {
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

    pub fn road_force(&self, speed: f32, mass: f32, radius: f32) -> f32 {
        // Solve clutch lock and rolling contact together. If clutch capacity
        // is exceeded, solve rolling contact with the slipping clutch impulse.
        let a = 1.0 / self.engine_inertia + self.ratio * self.ratio / self.wheel_inertia;
        let b = self.ratio * radius / self.wheel_inertia;
        let c = radius * radius / self.wheel_inertia + 1.0 / mass;
        let engine_slip = self.engine_omega - self.ratio * self.wheel_omega;
        let tire_slip = radius * self.wheel_omega - speed;
        let clutch = ((engine_slip + b * tire_slip / c) / (a - b * b / c))
            .clamp(-self.max_impulse, self.max_impulse);
        (tire_slip + b * clutch) / c / self.dt
    }

    pub fn finish(&self, tire_force: f32, radius: f32) -> (f32, f32) {
        // Actual grip may be lower than requested. Its equal and opposite
        // reaction changes wheel speed and therefore the clutch load.
        let wheel = self.wheel_omega - tire_force * radius * self.dt / self.wheel_inertia;
        let clutch = ((self.engine_omega - self.ratio * wheel)
            / (1.0 / self.engine_inertia + self.ratio * self.ratio / self.wheel_inertia))
            .clamp(-self.max_impulse, self.max_impulse);
        let engine = self.engine_omega - clutch / self.engine_inertia;
        let wheel = wheel + clutch * self.ratio / self.wheel_inertia;
        (engine * 60.0 / std::f32::consts::TAU, wheel)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn locked_clutch_and_tires_preserve_gearing_and_exchange_momentum() {
        for dt in [1.0 / 60.0, 1.0 / 240.0] {
            let step = super::Step::new(
                3000.0,
                3000.0 * std::f32::consts::TAU / 60.0 / 12.0,
                150.0,
                0.25,
                12.0,
                1000.0,
                dt,
            );
            let speed = step.wheel_omega * 0.32;
            let force = step.road_force(speed, 1200.0, 0.32);
            let (rpm, wheel) = step.finish(force, 0.32);
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
        let step = super::Step::new(6000.0, 0.0, 0.0, 0.25, 12.0, 400.0, 1.0 / 240.0);
        let (rpm, wheel) = step.finish(0.0, 0.32);
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
        let loaded = super::Step::new(3000.0, 20.0, 150.0, 0.25, 12.0, 500.0, 1.0 / 240.0);
        let force = loaded.road_force(6.4, 1200.0, 0.32);
        let (grip_rpm, grip_wheel) = loaded.finish(force, 0.32);
        let (slip_rpm, slip_wheel) = loaded.finish(0.0, 0.32);
        assert!(slip_rpm > grip_rpm && slip_wheel > grip_wheel);
        let open = super::Step::new(3000.0, 20.0, 150.0, 0.25, 12.0, 0.0, 1.0 / 240.0);
        assert_eq!(open.finish(0.0, 0.32).0, open.finish(5000.0, 0.32).0);
    }
}
