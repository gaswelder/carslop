use crate::{cars::CarModel, track};
use std::time::Duration;

pub const REVERSE_SPEED: f32 = 6.0;
pub const IDLE_RPM: f32 = 900.0;
pub const REDLINE_RPM: f32 = 6500.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Drive,
    Reverse,
}

#[derive(Clone, Copy, Default)]
pub struct Controls {
    pub throttle: f32,
    pub brake: f32,
    pub steering: f32,
    pub handbrake: bool,
}

pub struct Vehicle {
    pub model: CarModel,
    pub track: &'static track::Track,
    pub position: [f32; 3],
    pub velocity: [f32; 2],
    pub speed: f32,
    pub heading: f32,
    pub gear: u8,
    pub direction: Direction,
    pub slip: f32,
    pub body_pitch: f32,
    pub body_roll: f32,
    pub curb_bump: f32,
    pub engine_rpm: f32,
    pub engine_load: f32,
    pub wheel_phase: [f32; 2], // rear/front axles; cosmetic, never affects physics
    pub tire_scrub: f32,
    brake_pressure: f32,
    steering: f32,
    yaw_rate: f32,
    shift_timer: f32,
}

impl Vehicle {
    #[cfg(test)]
    pub fn new(position: [f32; 3], heading: f32, model: CarModel) -> Self {
        Self::on_track(position, heading, model, track::TrackId::Compact.track())
    }

    pub fn on_track(
        position: [f32; 3],
        heading: f32,
        model: CarModel,
        track: &'static track::Track,
    ) -> Self {
        Self {
            model,
            track,
            position,
            velocity: [0.0; 2],
            speed: 0.0,
            heading,
            gear: 1,
            direction: Direction::Drive,
            slip: 0.0,
            body_pitch: 0.0,
            body_roll: 0.0,
            curb_bump: 0.0,
            engine_rpm: IDLE_RPM,
            engine_load: 0.0,
            brake_pressure: 0.0,
            steering: 0.0,
            yaw_rate: 0.0,
            wheel_phase: [0.0; 2],
            tire_scrub: 0.0,
            shift_timer: 0.0,
        }
    }

    pub fn reset(&mut self, position: [f32; 3], heading: f32) {
        *self = Self::on_track(position, heading, self.model, self.track);
    }

    /// Reference speed for AI planning. Legacy cars also taper drive toward
    /// this value; Kestrel instead reaches a power/drag equilibrium.
    pub fn speed_cap(&self) -> f32 {
        let spec = self.model.spec();
        spec.top_speed
            * if spec.road_engine.is_some() {
                1.0
            } else {
                self.track.speed_scale
            }
    }
    pub fn upshift_speed(&self, gear: u8) -> f32 {
        let gearbox = &self.model.spec().gearbox;
        gearbox.speed_at_rpm(gearbox.upshift_rpm, gear)
    }
    pub fn target_engine_rpm(&self) -> f32 {
        self.model
            .spec()
            .gearbox
            .rpm(self.speed, self.gear)
            .clamp(IDLE_RPM, REDLINE_RPM)
    }
    pub fn brake_accel(&self) -> f32 {
        self.model.spec().brake_force / self.model.spec().mass * self.track.brake_scale
    }

    pub fn toggle_direction(&mut self) -> bool {
        // Check total motion, not just longitudinal speed, so slides cannot shift.
        if self.velocity[0].hypot(self.velocity[1]) > 0.5 {
            return false;
        }
        self.direction = match self.direction {
            Direction::Drive => Direction::Reverse,
            Direction::Reverse => Direction::Drive,
        };
        self.refresh_speed();
        true
    }

    pub fn gear_label(&self) -> String {
        if self.direction == Direction::Reverse {
            "R".into()
        } else {
            self.gear.to_string()
        }
    }

    pub fn update(&mut self, dt: Duration, controls: Controls) -> f32 {
        let previous_gear = self.gear;
        let steps = (dt.as_secs_f32() * 240.0).ceil().max(1.0) as u32;
        let seconds = dt.as_secs_f32() / steps as f32;
        let spec = self.model.spec();
        let power = spec.engine_force / spec.mass;
        let grip = spec.lateral_force / spec.mass;
        let reversing = self.direction == Direction::Reverse;
        let direction = if reversing { -1.0 } else { 1.0 };
        let top_speed = if reversing {
            REVERSE_SPEED
        } else {
            self.speed_cap()
        };
        let brake_accel = if reversing {
            spec.brake_force / spec.mass
        } else {
            self.brake_accel()
        };
        let drag = if reversing {
            0.009
        } else {
            spec.road_engine
                .as_ref()
                .map_or(self.track.drag, |engine| engine.drag_accel)
        };
        let mut impact: f32 = 0.0;
        for _ in 0..steps {
            self.update_steering(controls.steering, seconds);
            // Digital pedals build pressure in 0.2 s and release in 0.08 s.
            let requested_brake = controls.brake.clamp(0.0, 1.0);
            let rate = if requested_brake > self.brake_pressure {
                5.0
            } else {
                12.5
            };
            self.brake_pressure +=
                (requested_brake - self.brake_pressure).clamp(-rate * seconds, rate * seconds);
            let forward = [self.heading.sin(), self.heading.cos()];
            let right = [self.heading.cos(), -self.heading.sin()];
            let gradient = self.track.ground_gradient(self.position);
            // Horizontal component of gravity projected onto y = h(x, z).
            // Velocity is stored in XZ, so divide by 1 + |gradient|².
            let gravity = gradient.map(|g| -9.81 * g / (1.0 + dot(gradient, gradient)));
            let mut longitudinal = dot(self.velocity, forward);
            let lateral = dot(self.velocity, right) + dot(gravity, right) * seconds;
            let drive_speed = longitudinal * direction;
            let available_drive =
                if let Some(engine) = spec.road_engine.as_ref().filter(|_| !reversing) {
                    // Force-limited launch, then power-limited acceleration. No
                    // artificial speed taper: road load determines terminal speed.
                    power.min(engine.wheel_power_watts / (spec.mass * drive_speed.max(1.0)))
                } else {
                    power
                        * if reversing { 0.5 } else { 1.0 }
                        * (1.0 - (drive_speed.max(0.0) / top_speed).powi(2)).max(0.0)
                };
            let engine = controls.throttle.clamp(0.0, 1.0) * available_drive;
            // Brake opposes motion in either gear; it never selects a direction.
            // Approximate ABS: cap foot braking below the tire budget,
            // retaining at least 60% lateral authority at full pressure.
            let foot_braking = self.brake_pressure * brake_accel.min(grip * 0.8);
            let lateral_grip = (grip * grip - foot_braking * foot_braking).max(0.0).sqrt();
            let braking = foot_braking + if controls.handbrake { 4.0 } else { 0.0 };
            let driven = drive_speed + (engine + dot(gravity, forward) * direction) * seconds;
            let resistance = (braking + 0.25 + drag * driven * driven) * seconds;
            longitudinal = driven.signum() * (driven.abs() - resistance).max(0.0) * direction;
            let lateral_accel = (-lateral * if controls.handbrake { 1.7 } else { 9.0 })
                .clamp(-lateral_grip, lateral_grip)
                * if controls.handbrake { 0.4 } else { 1.0 };
            let sideways = lateral + lateral_accel * seconds;
            self.velocity = [
                forward[0] * longitudinal + right[0] * sideways,
                forward[1] * longitudinal + right[1] * sideways,
            ];
            let steer_angle = self.steering_angle(longitudinal);
            let requested_yaw = -longitudinal / spec.wheelbase * steer_angle.tan();
            // Available lateral force limits rotation as well as sideways motion.
            // Handbrake retains extra rotation while sacrificing lateral grip.
            let rotation_grip = lateral_grip * if controls.handbrake { 1.25 } else { 1.0 };
            let yaw_limit = rotation_grip / longitudinal.abs().max(1.0);
            let target_yaw = (requested_yaw * if controls.handbrake { 1.35 } else { 1.0 })
                .clamp(-yaw_limit, yaw_limit);
            let unwinding =
                target_yaw * self.yaw_rate <= 0.0 || target_yaw.abs() < self.yaw_rate.abs();
            let response = spec.steering_response * if unwinding { 2.4 } else { 1.4 };
            self.yaw_rate += (target_yaw - self.yaw_rate) * (1.0 - (-response * seconds).exp());
            self.yaw_rate = self.yaw_rate.clamp(-yaw_limit, yaw_limit);
            // A stopped car cannot rotate from residual steering response.
            self.yaw_rate *= (longitudinal.abs() / 0.5).clamp(0.0, 1.0);
            self.heading =
                (self.heading + self.yaw_rate * seconds).rem_euclid(std::f32::consts::TAU);
            let previous_position = self.position;
            self.position[0] += self.velocity[0] * seconds;
            self.position[2] += self.velocity[1] * seconds;
            let spin = longitudinal * seconds / crate::cars::WHEEL_RADIUS;
            self.wheel_phase[1] = (self.wheel_phase[1] + spin).rem_euclid(std::f32::consts::TAU);
            if !controls.handbrake {
                self.wheel_phase[0] =
                    (self.wheel_phase[0] + spin).rem_euclid(std::f32::consts::TAU);
            }
            impact = impact.max(self.resolve_road());
            self.curb_bump *= (-seconds * 12.0).exp();
            if let Some(city) = self.track.city() {
                if let Some((normal, rise)) = city.surfaces.as_ref().and_then(|s| {
                    s.crossed_curb(
                        [previous_position[0], previous_position[2]],
                        [self.position[0], self.position[2]],
                    )
                }) {
                    let across = dot(self.velocity, normal);
                    let loss = (across.abs() * 0.08).min(1.2) * (rise / 0.09);
                    for axis in 0..2 {
                        self.velocity[axis] -= normal[axis] * across.signum() * loss;
                    }
                    self.curb_bump = self
                        .curb_bump
                        .max(0.03 * (rise / 0.09) * (across.abs() / 6.0).min(1.0));
                }
            }
        }
        self.refresh_speed();
        self.update_engine_feedback(previous_gear, controls, dt.as_secs_f32());
        let motion = self.velocity[0].hypot(self.velocity[1]);
        let sliding = ((self.slip.abs() - 0.8) / 3.0).clamp(0.0, 1.0);
        let braking = if controls.handbrake { 0.8 } else { 0.0 };
        self.tire_scrub = sliding.max(braking) * ((motion - 1.5) / 2.5).clamp(0.0, 1.0);
        self.update_body_motion(controls, dt.as_secs_f32());
        impact
    }

    fn update_body_motion(&mut self, controls: Controls, dt: f32) {
        let forward_speed = self.speed.abs();
        let direction = if self.speed < -0.1 { -1.0 } else { 1.0 };
        let braking = self.brake_pressure * (forward_speed / 16.0).clamp(0.0, 1.0);
        let throttle = controls.throttle.clamp(0.0, 1.0) * (forward_speed / 10.0).clamp(0.0, 1.0);
        let target_pitch = ((throttle * 0.018 - braking * 0.055) * direction).clamp(-0.065, 0.065);
        // Lean follows the actual turn, so a key tap cannot visually throw the
        // body over before the steering and tires have responded.
        let grip = self.model.spec().lateral_force / self.model.spec().mass;
        let steering_roll = (self.yaw_rate * self.speed / grip).clamp(-1.0, 1.0) * 0.070;
        let slip_roll = (self.slip / 18.0).clamp(-1.0, 1.0) * 0.020;
        let target_roll = (steering_roll + slip_roll).clamp(-0.07, 0.07);
        let response = 1.0 - (-dt * 7.5).exp();
        self.body_pitch += (target_pitch - self.body_pitch) * response;
        self.body_roll += (target_roll - self.body_roll) * response;
    }

    fn update_engine_feedback(&mut self, previous_gear: u8, controls: Controls, dt: f32) {
        if previous_gear != self.gear && previous_gear != 0 && self.gear != 0 {
            self.shift_timer = 0.16;
        }
        self.shift_timer = (self.shift_timer - dt).max(0.0);
        let target_rpm = self.target_engine_rpm();
        let response = if self.shift_timer > 0.0 { 12.0 } else { 7.5 };
        self.engine_rpm += (target_rpm - self.engine_rpm) * (dt * response).clamp(0.0, 1.0);
        let requested = controls.throttle.clamp(0.0, 1.0);
        let shift_unload = if self.shift_timer > 0.0 { 0.25 } else { 1.0 };
        let target_load = (requested * shift_unload + 0.12 * (1.0 - requested)).clamp(0.0, 1.0);
        self.engine_load += (target_load - self.engine_load) * (dt * 10.0).clamp(0.0, 1.0);
    }

    fn update_steering(&mut self, requested: f32, dt: f32) {
        let requested = requested.clamp(-1.0, 1.0);
        // Cross the centre quickly when countersteering, then build the new
        // turn progressively. Small digital taps never request instant lock.
        let reversing = requested * self.steering < 0.0;
        let centering = reversing || requested.abs() < self.steering.abs();
        let target = if reversing { 0.0 } else { requested };
        let response = self.model.spec().steering_response;
        let rate = if centering {
            response * 0.85
        } else {
            response * 0.45 / (1.0 + self.speed.abs() * 0.008)
        };
        self.steering += (target - self.steering).clamp(-rate * dt, rate * dt);
    }

    fn steering_angle(&self, speed: f32) -> f32 {
        let lock = 0.54 / (1.0 + (speed.abs() / 18.0).powi(2));
        self.steering.signum() * self.steering.abs().powf(1.10) * lock
    }

    pub fn front_wheel_angle(&self) -> f32 {
        -self.steering_angle(self.speed)
    }

    pub fn resolve_road(&mut self) -> f32 {
        if let Some(city) = self.track.city() {
            return city.constrain(&mut self.position, &mut self.velocity);
        }
        let road = self.track.project(self.position);
        let normal = road.normal();
        let limit = road.width * 0.5 - track::CAR_RADIUS;
        let excess = road.offset - road.offset.clamp(-limit, limit);
        let mut impact = 0.0;
        if excess.abs() > 0.0001 {
            self.position[0] -= normal[0] * excess;
            self.position[2] -= normal[1] * excess;
            let outward = dot(self.velocity, normal) * excess.signum();
            if outward > 0.0 {
                impact = outward;
                for (axis, normal_component) in normal.iter().enumerate() {
                    self.velocity[axis] -= normal_component * excess.signum() * outward * 1.08;
                    self.velocity[axis] *= 0.94;
                }
            }
        }
        self.position[1] = road.point[1] + 0.48;
        impact
    }

    pub fn refresh_speed(&mut self) {
        self.speed = dot(self.velocity, [self.heading.sin(), self.heading.cos()]);
        self.slip = dot(self.velocity, [self.heading.cos(), -self.heading.sin()]);
        let gearbox = &self.model.spec().gearbox;
        if self.direction == Direction::Reverse {
            self.gear = 0;
        } else {
            self.gear = self.gear.clamp(1, gearbox.gears());
            // Separate up/down RPM thresholds prevent hunting around a shift.
            while self.gear < gearbox.gears() && self.speed >= self.upshift_speed(self.gear) {
                self.gear += 1;
            }
            while self.gear > 1
                && gearbox.rpm(self.speed, self.gear) < gearbox.downshift_rpm
                && gearbox.rpm(self.speed, self.gear - 1) < gearbox.upshift_rpm * 0.95
            {
                self.gear -= 1;
            }
        }
    }

    pub fn collide(&mut self, other: &mut Vehicle) -> f32 {
        debug_assert!(
            std::ptr::eq(self.track, other.track),
            "vehicle contact requires the same route"
        );
        let dx = self.position[0] - other.position[0];
        let dz = self.position[2] - other.position[2];
        let distance = dx.hypot(dz);
        let separation = track::CAR_RADIUS * 2.0;
        if distance >= separation {
            return 0.0;
        }
        let normal = if distance < 0.001 {
            [1.0, 0.0]
        } else {
            [dx / distance, dz / distance]
        };
        let inverse_mass = 1.0 / self.model.spec().mass;
        let other_inverse_mass = 1.0 / other.model.spec().mass;
        let inverse_sum = inverse_mass + other_inverse_mass;
        let penetration = separation - distance;
        let relative = dot(
            [
                self.velocity[0] - other.velocity[0],
                self.velocity[1] - other.velocity[1],
            ],
            normal,
        );
        for (axis, component) in normal.iter().enumerate() {
            let position_axis = if axis == 0 { 0 } else { 2 };
            self.position[position_axis] += component * penetration * inverse_mass / inverse_sum;
            other.position[position_axis] -=
                component * penetration * other_inverse_mass / inverse_sum;
            if relative < 0.0 {
                let impulse = -relative * 1.1 / inverse_sum;
                self.velocity[axis] += component * impulse * inverse_mass;
                other.velocity[axis] -= component * impulse * other_inverse_mass;
            }
        }
        let wall_impact = self.resolve_road().max(other.resolve_road());
        self.refresh_speed();
        other.refresh_speed();
        (-relative).max(wall_impact).max(0.0)
    }
}

fn dot(a: [f32; 2], b: [f32; 2]) -> f32 {
    a[0] * b[0] + a[1] * b[1]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hill_car(uphill: bool) -> Vehicle {
        let route = track::TrackId::Scenic.track();
        let road = (0..200)
            .map(|i| route.at_distance(i as f32 * 10.0))
            .max_by(|a, b| {
                dot(route.ground_gradient(a.point), a.tangent)
                    .total_cmp(&dot(route.ground_gradient(b.point), b.tangent))
            })
            .unwrap();
        assert!(dot(route.ground_gradient(road.point), road.tangent) > 0.04);
        Vehicle::on_track(
            road.lane_position(0.0),
            road.heading() + if uphill { 0.0 } else { std::f32::consts::PI },
            CarModel::Roadster,
            route,
        )
    }

    #[test]
    fn elevation_changes_coasting_and_power_in_both_directions() {
        for throttle in [0.0, 1.0] {
            for reverse in [false, true] {
                let run = |uphill| {
                    let mut car = hill_car(uphill);
                    if reverse {
                        car.toggle_direction();
                        car.heading += std::f32::consts::PI;
                    }
                    let sign = if reverse { -1.0 } else { 1.0 };
                    car.velocity = [
                        car.heading.sin() * 8.0 * sign,
                        car.heading.cos() * 8.0 * sign,
                    ];
                    car.update(
                        Duration::from_millis(250),
                        Controls {
                            throttle,
                            ..Controls::default()
                        },
                    );
                    car.speed * sign
                };
                assert!(run(false) > run(true) + 0.15);
            }
        }
    }

    #[test]
    fn hills_allow_rollback_but_brakes_hold_and_substeps_are_consistent() {
        let mut rolling = hill_car(true);
        rolling.update(Duration::from_secs(1), Controls::default());
        assert!(rolling.speed < -0.1, "rollback: {}", rolling.speed);
        for handbrake in [false, true] {
            let mut held = hill_car(true);
            held.update(
                Duration::from_secs(2),
                Controls {
                    brake: if handbrake { 0.0 } else { 1.0 },
                    handbrake,
                    ..Controls::default()
                },
            );
            assert!(held.speed.abs() < 0.001);
        }
        let run = |hz| {
            let mut car = hill_car(false);
            for _ in 0..hz {
                car.update(
                    Duration::from_secs_f64(1.0 / hz as f64),
                    Controls::default(),
                );
            }
            car.speed
        };
        assert!((run(30) - run(120)).abs() < 0.005);
    }

    fn car() -> Vehicle {
        let start = track::at_distance(5.0);
        Vehicle::new(
            start.lane_position(0.0),
            start.heading(),
            CarModel::Roadster,
        )
    }

    fn route_car(id: track::TrackId, model: CarModel) -> Vehicle {
        let track = id.track();
        let road = track.at_distance(50.0);
        Vehicle::on_track(road.lane_position(0.0), road.heading(), model, track)
    }

    // Drivetrain calibration compares identical level ground across route tuning.
    fn level_route_car(id: track::TrackId, model: CarModel) -> Vehicle {
        let track = Box::leak(Box::new(id.track().flat_test_track()));
        let road = track.at_distance(50.0);
        Vehicle::on_track(road.lane_position(0.0), road.heading(), model, track)
    }

    // Reset only position each tick: velocity/drivetrain evolve normally, but
    // bends and barriers cannot masquerade as acceleration or braking limits.
    fn treadmill_tick(car: &mut Vehicle, controls: Controls) -> f32 {
        let road = car.track.at_distance(50.0);
        car.position = road.lane_position(0.0);
        let previous = car.position;
        car.update(Duration::from_secs_f64(1.0 / 240.0), controls);
        dot(
            [car.position[0] - previous[0], car.position[2] - previous[2]],
            road.tangent,
        )
        .abs()
    }

    // Keep a fixed speed on a straight test patch so road barriers, acceleration
    // and drag cannot hide a steering regression. Heading/yaw evolve normally.
    fn steering_run(car: &mut Vehicle, speed: f32, input: f32, seconds: f64, hz: f64) {
        let road = car.track.at_distance(50.0);
        let mut remaining = seconds;
        while remaining > 1e-8 {
            let step = remaining.min(1.0 / hz);
            car.position = road.lane_position(0.0);
            car.velocity = [car.heading.sin() * speed, car.heading.cos() * speed];
            car.refresh_speed();
            car.update(
                Duration::from_secs_f64(step),
                Controls {
                    steering: input,
                    ..Controls::default()
                },
            );
            remaining -= step;
        }
    }

    #[test]
    fn keyboard_taps_are_small_corrections_and_held_turns_remain_useful() {
        for model in crate::cars::ALL {
            for speed in [5.0, 15.0, 25.0, 45.0, 55.0] {
                let mut car = route_car(track::TrackId::Scenic, model);
                let heading = car.heading;
                steering_run(&mut car, speed, 1.0, 0.1, 120.0);
                assert!(
                    car.body_roll.abs() < 0.012,
                    "a brief tap must not snap the body into a full lean"
                );
                steering_run(&mut car, speed, 0.0, 0.4, 120.0);
                let tap = crate::ai::angle(car.heading - heading).abs().to_degrees();
                println!("{model:?} {speed} m/s: 100 ms tap {tap:.2} degrees");
                assert!(
                    tap > 0.01 && tap < 3.0,
                    "{model:?} at {speed}: tap rotated {tap} degrees"
                );
                if speed == 15.0 || speed == 25.0 {
                    assert!(tap > 0.5,"a city-speed tap must make a useful correction: {model:?} at {speed}: {tap}");
                }
                assert_eq!(car.steering, 0.0);
                assert!(car.yaw_rate.abs() < 0.002);
                let mut held = route_car(track::TrackId::Scenic, model);
                steering_run(&mut held, speed, 1.0, 1.0, 120.0);
                let turn = crate::ai::angle(held.heading - heading).abs().to_degrees();
                assert!(
                    turn > tap * 3.0,
                    "holding must produce a stronger turn than tapping"
                );
                if speed == 5.0 {
                    assert!(turn > 15.0, "parking-speed turns need useful lock: {turn}");
                }
            }
        }
    }

    #[test]
    fn rotation_is_grip_limited_and_countersteering_crosses_promptly() {
        for model in crate::cars::ALL {
            let mut car = route_car(track::TrackId::Scenic, model);
            let spec = model.spec();
            steering_run(&mut car, 35.0, 1.0, 0.8, 240.0);
            assert!(car.yaw_rate < 0.0);
            assert!(car.yaw_rate.abs() * 35.0 <= spec.lateral_force / spec.mass + 0.01);
            steering_run(&mut car, 35.0, -1.0, 0.4, 240.0);
            assert!(
                car.yaw_rate > 0.0,
                "{model:?} countersteering remained in original turn"
            );
            assert!(car.front_wheel_angle() > 0.0);
            steering_run(&mut car, 0.0, 0.0, 0.02, 240.0);
            assert_eq!(car.yaw_rate, 0.0);
            car.reset(car.position, car.heading);
            assert_eq!(car.yaw_rate, 0.0);
            assert_eq!(car.front_wheel_angle(), 0.0);
        }
    }

    #[test]
    fn steering_response_is_consistent_across_frame_rates() {
        for model in crate::cars::ALL {
            let mut reference = route_car(track::TrackId::Scenic, model);
            let start = reference.heading;
            for (input, seconds) in [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)] {
                steering_run(&mut reference, 25.0, input, seconds, 240.0);
            }
            for hz in [30.0, 60.0, 144.0] {
                let mut car = route_car(track::TrackId::Scenic, model);
                for (input, seconds) in
                    [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)]
                {
                    steering_run(&mut car, 25.0, input, seconds, hz);
                }
                let error = crate::ai::angle(car.heading - reference.heading)
                    .abs()
                    .to_degrees();
                assert!(
                    error < 0.25,
                    "{model:?} {hz} Hz differs by {error} degrees from 240 Hz; start {start}"
                );
            }
        }
    }

    #[test]
    fn steering_builds_progressively_centres_faster_and_preserves_car_identity() {
        let results = crate::cars::ALL.map(|model| {
            let mut car = route_car(track::TrackId::Scenic, model);
            steering_run(&mut car, 15.0, 1.0, 0.1, 240.0);
            let built = car.front_wheel_angle().abs();
            steering_run(&mut car, 15.0, 0.0, 0.05, 240.0);
            assert_eq!(
                car.front_wheel_angle(),
                0.0,
                "a brief correction must centre promptly"
            );
            built
        });
        assert!(
            results[2] > results[0] && results[0] > results[1],
            "Club > Kestrel > Muscle response: {results:?}"
        );
    }

    #[test]
    fn ratio_shifts_drop_rpm_and_do_not_hunt_around_the_upshift_speed() {
        for model in crate::cars::ALL {
            let mut car = route_car(track::TrackId::Scenic, model);
            let road = car.track.at_distance(50.0);
            let g = &model.spec().gearbox;
            for gear in 1..g.gears() {
                car.gear = gear;
                let shift = car.upshift_speed(gear);
                car.velocity = road.tangent.map(|v| v * (shift - 0.01));
                car.refresh_speed();
                assert_eq!(car.gear, gear);
                let before = car.target_engine_rpm();
                car.velocity = road.tangent.map(|v| v * (shift + 0.01));
                car.refresh_speed();
                assert_eq!(car.gear, gear + 1);
                assert!(car.target_engine_rpm() < before);
                for delta in [-0.1, 0.1, -0.05, 0.05] {
                    car.velocity = road.tangent.map(|v| v * (shift + delta));
                    car.refresh_speed();
                    assert_eq!(
                        car.gear,
                        gear + 1,
                        "must retain higher gear near an upshift"
                    );
                }
                let down = g.speed_at_rpm(g.downshift_rpm, gear + 1);
                car.velocity = road.tangent.map(|v| v * (down - 0.01));
                car.refresh_speed();
                assert!(car.gear <= gear);
                assert!(car.target_engine_rpm() < REDLINE_RPM);
            }
            car.velocity = [0.0; 2];
            car.refresh_speed();
            assert_eq!(car.gear, 1);
            assert_eq!(car.target_engine_rpm(), IDLE_RPM);
        }
    }

    #[test]
    fn kestrel_uses_the_earliest_911_ratios_and_longer_first_gear() {
        let g = &CarModel::Roadster.spec().gearbox;
        assert_eq!(g.ratios, &[2.833, 1.778, 1.217, 0.962, 0.821]);
        assert!((g.final_drive - 31.0 / 7.0).abs() < 0.00001);
        let car = route_car(track::TrackId::Engomi, CarModel::Roadster);
        assert!((car.upshift_speed(1) * 3.6 - 58.7).abs() < 0.3);
        let rpm = g.rpm(100.0 / 3.6, 5);
        assert!((rpm - 3062.0).abs() < 5.0, "100 km/h in fifth: {rpm}");
    }

    #[test]
    fn city_curb_is_crossable_with_a_single_small_speed_loss_and_settling_bump() {
        let track = track::TrackId::Engomi.track();
        let city = track.city().unwrap();
        let surfaces = city.surfaces.as_ref().unwrap();
        let (q, normal) = surfaces
            .curbs
            .iter()
            .find_map(|curb| {
                let q = [
                    (curb.ends[0][0] + curb.ends[1][0]) * 0.5,
                    (curb.ends[0][1] + curb.ends[1][1]) * 0.5,
                ];
                let inside = [q[0] + curb.inward[0] * 0.08, q[1] + curb.inward[1] * 0.08];
                let outside = [q[0] - curb.inward[0] * 0.2, q[1] - curb.inward[1] * 0.2];
                if !surfaces.contains(inside)
                    || surfaces.contains(outside)
                    || surfaces.paving_height(outside) != Some(0.14)
                {
                    return None;
                }
                for distance in [0.08, -0.2, -1.0, -2.0, -3.0] {
                    let mut p = [
                        q[0] + curb.inward[0] * distance,
                        0.0,
                        q[1] + curb.inward[1] * distance,
                    ];
                    let before = p;
                    city.constrain(&mut p, &mut [0.0; 2]);
                    if (p[0] - before[0]).hypot(p[2] - before[2]) > 0.001 {
                        return None;
                    }
                }
                Some((q, curb.inward))
            })
            .expect("missing clear curb crossing fixture");
        let position = [q[0] + normal[0] * 0.08, 0.0, q[1] + normal[1] * 0.08];
        let heading = (-normal[0]).atan2(-normal[1]);
        let mut car = Vehicle::on_track(position, heading, CarModel::Roadster, track);
        car.velocity = normal.map(|n| -n * 8.0);
        car.update(Duration::from_millis(24), Controls::default());
        assert!(!surfaces.contains([car.position[0], car.position[2]]));
        assert!(
            car.speed > 7.0 && car.speed < 7.8,
            "small curb penalty: {}",
            car.speed
        );
        assert!(car.curb_bump > 0.0 && car.curb_bump <= 0.03);
        let speed = car.speed;
        let bump = car.curb_bump;
        car.update(Duration::ZERO, Controls::default());
        assert_eq!(car.curb_bump, bump);
        car.update(Duration::from_millis(250), Controls::default());
        assert!(car.curb_bump < bump * 0.1);
        assert!(
            car.speed > speed - 0.3,
            "do not repeatedly apply curb drag off the asphalt"
        );
        car.reset(car.position, car.heading);
        assert_eq!(car.curb_bump, 0.0);
    }

    #[test]
    fn route_calibration_sets_forward_caps_braking_and_shift_points() {
        for model in crate::cars::ALL {
            let mut compact = route_car(track::TrackId::Compact, model);
            let mut scenic = route_car(track::TrackId::Scenic, model);
            let spec = model.spec();
            assert_eq!(compact.speed_cap(), spec.top_speed);
            assert_eq!(
                compact.upshift_speed(1),
                spec.gearbox.speed_at_rpm(spec.gearbox.upshift_rpm, 1)
            );
            assert_eq!(compact.brake_accel(), spec.brake_force / spec.mass);
            assert_eq!(
                scenic.speed_cap(),
                spec.top_speed * if spec.road_engine.is_some() { 1.0 } else { 1.6 }
            );
            assert_eq!(scenic.upshift_speed(1), compact.upshift_speed(1));
            assert_eq!(scenic.brake_accel(), spec.brake_force / spec.mass * 0.65);
            for car in [&mut compact, &mut scenic] {
                let tangent = car.track.at_distance(50.0).tangent;
                car.velocity = tangent.map(|v| v * (car.upshift_speed(1) - 0.01));
                car.refresh_speed();
                assert_eq!(car.gear, 1);
                car.velocity = tangent.map(|v| v * (car.upshift_speed(1) + 0.01));
                car.refresh_speed();
                assert_eq!(car.gear, 2);
            }
        }
        assert_eq!(
            route_car(track::TrackId::Compact, CarModel::Roadster).speed_cap(),
            210.0 / 3.6
        );
        assert_eq!(
            route_car(track::TrackId::Scenic, CarModel::Roadster).speed_cap(),
            210.0 / 3.6
        );
    }

    #[test]
    fn kestrel_passes_old_limit_and_reaches_reference_speed_on_every_map() {
        let throttle = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        let speeds = track::ALL.map(|id| {
            let mut car = level_route_car(id, CarModel::Roadster);
            for _ in 0..240 * 15 {
                treadmill_tick(&mut car, throttle);
            }
            assert!(
                car.speed * 3.6 > 130.0,
                "{id:?} stuck at {} km/h",
                car.speed * 3.6
            );
            for _ in 0..240 * 165 {
                treadmill_tick(&mut car, throttle);
            }
            let terminal = car.speed * 3.6;
            println!(
                "Kestrel {id:?}: terminal {terminal:.1} km/h, gear {}, RPM {:.0}",
                car.gear, car.engine_rpm
            );
            assert!(
                (208.0..213.0).contains(&terminal),
                "{id:?}: {terminal} km/h"
            );
            assert_eq!(car.gear, 5);
            assert!(car.engine_rpm < REDLINE_RPM);
            terminal
        });
        assert!(speeds.iter().all(|s| (*s - speeds[0]).abs() < 0.1));
    }

    #[test]
    fn kestrel_power_limited_acceleration_tapers_and_overspeed_is_not_clamped() {
        let sample = track::TrackId::Scenic.track().at_distance(50.0);
        let acceleration = |speed: f32| {
            let mut car = route_car(track::TrackId::Scenic, CarModel::Roadster);
            car.velocity = sample.tangent.map(|v| v * speed);
            car.refresh_speed();
            treadmill_tick(
                &mut car,
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            (car.speed - speed) * 240.0
        };
        assert!(acceleration(20.0) > acceleration(40.0));
        assert!(acceleration(40.0) > 0.0);
        assert!(acceleration(70.0) < 0.0);
        let mut car = route_car(track::TrackId::Scenic, CarModel::Roadster);
        car.velocity = sample.tangent.map(|v| v * 70.0);
        car.refresh_speed();
        treadmill_tick(
            &mut car,
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
        assert!(
            car.speed > 69.9,
            "overspeed should decelerate continuously, not snap to a cap"
        );
    }

    #[test]
    fn foot_brake_builds_releases_and_does_not_fake_a_skid() {
        let mut car = route_car(track::TrackId::Scenic, CarModel::Roadster);
        let road = car.track.at_distance(50.0);
        car.velocity = road.tangent.map(|v| v * 20.0);
        let pedal = Controls {
            brake: 1.0,
            ..Controls::default()
        };
        car.update(Duration::from_millis(20), pedal);
        assert!((car.brake_pressure - 0.1).abs() < 0.001);
        car.update(Duration::from_millis(180), pedal);
        assert!((car.brake_pressure - 1.0).abs() < 0.001);
        assert!(
            car.tire_scrub < 0.01,
            "straight braking must not signal lockup"
        );
        car.update(Duration::from_millis(80), Controls::default());
        assert!(car.brake_pressure < 0.001);
        car.reset(road.lane_position(0.0), road.heading());
        assert_eq!(car.brake_pressure, 0.0);
    }

    #[test]
    fn scenic_roadster_brakes_from_100_kmh_progressively() {
        let mut car = route_car(track::TrackId::Scenic, CarModel::Roadster);
        car.velocity = car
            .track
            .at_distance(50.0)
            .tangent
            .map(|v| v * (100.0 / 3.6));
        car.refresh_speed();
        let mut distance = 0.0;
        for _ in 0..240 * 5 {
            distance += treadmill_tick(
                &mut car,
                Controls {
                    brake: 1.0,
                    ..Controls::default()
                },
            );
            if car.speed < 0.001 {
                break;
            }
        }
        assert!(
            car.speed.abs() < 0.001,
            "braking did not stop: {}",
            car.speed
        );
        assert!(
            (38.0..44.0).contains(&distance),
            "100 km/h stopping distance: {distance} m"
        );
        assert_eq!(car.direction, Direction::Drive);
    }

    #[test]
    fn reverse_acceleration_terminal_speed_and_braking_are_route_independent() {
        for model in crate::cars::ALL {
            let mut compact = level_route_car(track::TrackId::Compact, model);
            let mut scenic = level_route_car(track::TrackId::Scenic, model);
            assert!(compact.toggle_direction() && scenic.toggle_direction());
            for _ in 0..240 * 12 {
                for car in [&mut compact, &mut scenic] {
                    treadmill_tick(
                        car,
                        Controls {
                            throttle: 1.0,
                            ..Controls::default()
                        },
                    );
                }
                assert!((compact.speed - scenic.speed).abs() < 0.001);
            }
            assert!(scenic.speed < -3.0 && scenic.speed >= -REVERSE_SPEED);
            let (mut compact_distance, mut scenic_distance) = (0.0, 0.0);
            for _ in 0..240 {
                let brake = Controls {
                    brake: 1.0,
                    ..Controls::default()
                };
                compact_distance += treadmill_tick(&mut compact, brake);
                scenic_distance += treadmill_tick(&mut scenic, brake);
                assert!((compact.speed - scenic.speed).abs() < 0.001);
            }
            assert!(compact.speed.abs() < 0.001 && scenic.speed.abs() < 0.001);
            assert!((compact_distance - scenic_distance).abs() < 0.01);
            assert_eq!(scenic.gear, 0);
        }
    }

    #[test]
    fn road_resolution_uses_each_cars_route_width_and_elevation() {
        let mut compact = route_car(track::TrackId::Compact, CarModel::Roadster);
        let compact_position = compact.position;
        let mut scenic = route_car(track::TrackId::Scenic, CarModel::Roadster);
        let road = scenic.track.at_distance(50.0);
        scenic.position = road.lane_position(2.4);
        scenic.resolve_road();
        assert!(
            (scenic.track.project(scenic.position).offset - 2.4).abs() < 0.05,
            "scenic must not inherit the compact barrier at 2.2 m"
        );
        scenic.position = road.lane_position(4.0);
        scenic.velocity = road.normal().map(|v| v * 5.0);
        assert!(scenic.resolve_road() > 4.0);
        let projection = scenic.track.project(scenic.position);
        assert!(projection.offset.abs() <= scenic.track.width * 0.5 - track::CAR_RADIUS + 0.02);
        assert!((scenic.position[1] - projection.point[1] - 0.48).abs() < 0.05);
        assert!(
            (track::TrackId::Compact
                .track()
                .project(scenic.position)
                .point[2]
                - scenic.position[2])
                .abs()
                > 100.0
        );
        compact.resolve_road();
        for (actual, expected) in compact.position.into_iter().zip(compact_position) {
            assert!((actual - expected).abs() < 0.001);
        }
        assert!(std::ptr::eq(compact.track, track::TrackId::Compact.track()));
    }

    #[test]
    fn scenic_recovery_preserves_route_and_model_but_clears_motion_and_reverse() {
        let mut car = route_car(track::TrackId::Scenic, CarModel::Muscle);
        car.toggle_direction();
        treadmill_tick(
            &mut car,
            Controls {
                throttle: 1.0,
                steering: 1.0,
                ..Controls::default()
            },
        );
        car.tire_scrub = 1.0;
        let road = car.track.at_distance(850.0);
        car.reset(road.lane_position(0.0), road.heading());
        assert!(std::ptr::eq(car.track, track::TrackId::Scenic.track()));
        assert_eq!(car.model, CarModel::Muscle);
        assert_eq!(car.position, road.lane_position(0.0));
        assert_eq!(car.heading, road.heading());
        assert_eq!(car.velocity, [0.0; 2]);
        assert_eq!(car.direction, Direction::Drive);
        assert_eq!(car.gear, 1);
        assert_eq!(car.speed, 0.0);
        assert_eq!(car.slip, 0.0);
        assert_eq!(car.body_pitch, 0.0);
        assert_eq!(car.body_roll, 0.0);
        assert_eq!(car.engine_rpm, IDLE_RPM);
        assert_eq!(car.engine_load, 0.0);
        assert_eq!(car.wheel_phase, [0.0; 2]);
        assert_eq!(car.front_wheel_angle(), 0.0);
        assert_eq!(car.tire_scrub, 0.0);
    }

    #[test]
    fn throttle_accelerates_and_brake_never_reverses() {
        let mut v = car();
        v.update(
            Duration::from_secs(1),
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
        assert!(v.speed > 3.0);
        v.update(
            Duration::from_secs(2),
            Controls {
                brake: 1.0,
                ..Controls::default()
            },
        );
        assert!(v.speed.abs() < 0.01);
    }

    #[test]
    fn wall_response_preserves_tangential_motion_and_height() {
        let sample = track::at_distance(50.0);
        let mut v = Vehicle::new(
            sample.lane_position(4.0),
            sample.heading(),
            CarModel::Roadster,
        );
        let normal = sample.normal();
        v.velocity = [
            sample.tangent[0] * 8.0 + normal[0] * 4.0,
            sample.tangent[1] * 8.0 + normal[1] * 4.0,
        ];
        assert!(v.resolve_road() > 0.0);
        assert!(track::project(v.position).offset.abs() < 2.3);
        assert!((v.position[1] - sample.point[1] - 0.48).abs() < 0.2);
        assert!(dot(v.velocity, sample.tangent) > 6.0);
    }

    #[test]
    fn handbrake_reduces_lateral_grip() {
        let mut normal = car();
        let mut sliding = car();
        let h = normal.heading;
        let velocity = [h.sin() * 8.0 + h.cos() * 2.0, h.cos() * 8.0 - h.sin() * 2.0];
        normal.velocity = velocity;
        sliding.velocity = velocity;
        normal.update(Duration::from_millis(80), Controls::default());
        sliding.update(
            Duration::from_millis(80),
            Controls {
                handbrake: true,
                ..Controls::default()
            },
        );
        assert!(sliding.slip.abs() > normal.slip.abs());
    }

    #[test]
    fn car_contact_separates_and_transfers_momentum() {
        let mut a = car();
        let mut b = car();
        b.position[0] += 1.0;
        a.velocity = [6.0, 0.0];
        assert!(a.collide(&mut b) > 0.0);
        assert!(b.velocity[0] > 0.0);
        assert!(a.position.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn models_have_distinct_acceleration_braking_and_grip() {
        let sample = track::at_distance(50.0);
        let results = crate::cars::ALL.map(|model| {
            let make = || Vehicle::new(sample.lane_position(0.0), sample.heading(), model);
            let mut accelerating = make();
            accelerating.update(
                Duration::from_millis(250),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            let mut braking = make();
            braking.velocity = sample.tangent.map(|v| v * 8.0);
            braking.update(
                Duration::from_millis(150),
                Controls {
                    brake: 1.0,
                    ..Controls::default()
                },
            );
            let mut sliding = make();
            let n = sample.normal();
            sliding.velocity = [
                sample.tangent[0] * 5.0 + n[0] * 2.0,
                sample.tangent[1] * 5.0 + n[1] * 2.0,
            ];
            sliding.update(Duration::from_millis(50), Controls::default());
            (accelerating.speed, braking.speed, sliding.slip.abs())
        });
        let [roadster, muscle, club] = results;
        assert!(
            muscle.0 > roadster.0 && roadster.0 > club.0,
            "acceleration: {results:?}"
        );
        assert!(
            club.1 < roadster.1 && roadster.1 < muscle.1,
            "braking: {results:?}"
        );
        assert!(
            club.2 < roadster.2 && roadster.2 < muscle.2,
            "lateral grip: {results:?}"
        );
    }

    #[test]
    fn model_speed_limits_and_gearboxes_are_used() {
        let sample = track::at_distance(50.0);
        let speeds = crate::cars::ALL.map(|model| {
            let mut v = Vehicle::new(sample.lane_position(0.0), sample.heading(), model);
            // Reuse a short straight patch as a test treadmill, not a gameplay feature.
            for _ in 0..1800 {
                v.position = sample.lane_position(0.0);
                v.update(
                    Duration::from_secs_f64(1.0 / 60.0),
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            assert!(v.speed > 15.0 && v.speed <= v.speed_cap());
            let terminal_speed = v.speed;
            v.velocity = sample
                .tangent
                .map(|x| x * v.upshift_speed(v.model.spec().gearbox.gears()));
            v.refresh_speed();
            assert_eq!(v.gear, model.spec().gearbox.gears());
            // Return terminal speed rather than the artificial gearbox test speed.
            terminal_speed
        });
        assert!(
            speeds[0] > speeds[1] && speeds[1] > speeds[2],
            "terminal speeds: {speeds:?}"
        );
    }

    #[test]
    fn unequal_mass_contact_conserves_momentum_and_moves_light_car_more() {
        let sample = track::at_distance(50.0);
        let mut heavy = Vehicle::new(
            sample.lane_position(0.8),
            sample.heading(),
            CarModel::Muscle,
        );
        let mut light = Vehicle::new(sample.lane_position(-0.8), sample.heading(), CarModel::Club);
        let n = sample.normal();
        heavy.velocity = n.map(|x| -x * 3.0);
        let momentum = |a: &Vehicle, b: &Vehicle| {
            std::array::from_fn::<_, 2, _>(|i| {
                a.velocity[i] * a.model.spec().mass + b.velocity[i] * b.model.spec().mass
            })
        };
        let before = momentum(&heavy, &light);
        let heavy_before = heavy.position;
        let light_before = light.position;
        assert!(heavy.collide(&mut light) > 0.0);
        let after = momentum(&heavy, &light);
        for i in 0..2 {
            assert!((before[i] - after[i]).abs() < 0.01);
        }
        let moved = |a: [f32; 3], b: [f32; 3]| (a[0] - b[0]).hypot(a[2] - b[2]);
        assert!(moved(light_before, light.position) > moved(heavy_before, heavy.position));
    }

    #[test]
    fn wheel_feedback_tracks_motion_steering_and_rear_handbrake_lock() {
        let mut v = car();
        v.update(
            Duration::from_millis(16),
            Controls {
                handbrake: true,
                ..Controls::default()
            },
        );
        assert_eq!(v.wheel_phase, [0.0; 2]);
        assert_eq!(v.tire_scrub, 0.0);
        v.velocity = [v.heading.sin() * 8.0, v.heading.cos() * 8.0];
        v.update(
            Duration::from_millis(16),
            Controls {
                steering: 1.0,
                handbrake: true,
                ..Controls::default()
            },
        );
        assert_eq!(v.wheel_phase[0], 0.0);
        assert!(v.wheel_phase[1] > 0.0);
        assert!(v.front_wheel_angle() < 0.0);
        assert!(v.tire_scrub > 0.5);
        v.update(Duration::from_millis(16), Controls::default());
        assert!(v.wheel_phase[0] > 0.0);
        let model = v.model;
        v.reset(v.position, v.heading);
        assert_eq!(v.model, model);
        assert_eq!(v.wheel_phase, [0.0; 2]);
        assert_eq!(v.tire_scrub, 0.0);
    }

    #[test]
    fn cosmetic_body_motion_responds_to_braking_throttle_and_cornering() {
        let road = track::at_distance(50.0);
        let mut braking = Vehicle::new(road.lane_position(0.0), road.heading(), CarModel::Roadster);
        braking.velocity = road.tangent.map(|v| v * 20.0);
        braking.refresh_speed();
        for _ in 0..30 {
            braking.update(
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    brake: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert!(
            braking.body_pitch < -0.025,
            "braking should dip the nose: {}",
            braking.body_pitch
        );

        let mut accelerating =
            Vehicle::new(road.lane_position(0.0), road.heading(), CarModel::Roadster);
        accelerating.velocity = road.tangent.map(|v| v * 8.0);
        accelerating.refresh_speed();
        for _ in 0..30 {
            accelerating.update(
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert!(
            accelerating.body_pitch > 0.005,
            "throttle should settle the rear slightly: {}",
            accelerating.body_pitch
        );

        let mut turning = Vehicle::new(road.lane_position(0.0), road.heading(), CarModel::Roadster);
        turning.velocity = road.tangent.map(|v| v * 18.0);
        turning.refresh_speed();
        for _ in 0..30 {
            turning.update(
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    steering: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert!(turning.body_roll.abs() > 0.01);
        assert!(turning.body_pitch.abs() <= 0.07 && turning.body_roll.abs() <= 0.075);
        turning.reset(turning.position, turning.heading);
        assert_eq!(turning.body_pitch, 0.0);
        assert_eq!(turning.body_roll, 0.0);
    }

    #[test]
    fn body_motion_handles_reverse_and_frame_rate_independent_settling() {
        let mut forward = car();
        let mut reverse = car();
        forward.speed = 6.0;
        reverse.speed = -6.0;
        let throttle = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        for _ in 0..60 {
            forward.update_body_motion(throttle, 1.0 / 60.0);
            reverse.update_body_motion(throttle, 1.0 / 60.0);
        }
        assert!(forward.body_pitch > 0.005);
        assert!((forward.body_pitch + reverse.body_pitch).abs() < 0.00001);
        reverse.brake_pressure = 1.0;
        let brake = Controls {
            brake: 1.0,
            ..Controls::default()
        };
        for _ in 0..60 {
            reverse.update_body_motion(brake, 1.0 / 60.0);
        }
        assert!(
            reverse.body_pitch > 0.01,
            "reverse braking should lift the nose"
        );
        forward.speed = 0.0;
        reverse.speed = 0.0;
        forward.body_pitch = 0.05;
        reverse.body_pitch = 0.05;
        for _ in 0..30 {
            forward.update_body_motion(Controls::default(), 1.0 / 30.0);
        }
        for _ in 0..144 {
            reverse.update_body_motion(Controls::default(), 1.0 / 144.0);
        }
        assert!((forward.body_pitch - reverse.body_pitch).abs() < 0.000001);
        assert!(forward.body_pitch.abs() < 0.0001);
    }

    #[test]
    fn update_reports_road_impacts_without_losing_substep_peaks() {
        let road = track::at_distance(50.0);
        let mut v = Vehicle::new(road.lane_position(2.3), road.heading(), CarModel::Roadster);
        v.velocity = road.normal().map(|x| x * 5.0);
        let impact = v.update(Duration::from_millis(16), Controls::default());
        assert!(impact > 3.0);
        assert_eq!(v.update(Duration::ZERO, Controls::default()), 0.0);
    }

    #[test]
    fn engine_feedback_smooths_rpm_load_and_shift_unload_without_changing_physics() {
        let road = track::at_distance(50.0);
        let mut car = Vehicle::new(road.lane_position(0.0), road.heading(), CarModel::Roadster);
        assert_eq!(car.engine_rpm, IDLE_RPM);
        assert_eq!(car.engine_load, 0.0);
        let mut previous_speed = 0.0;
        for _ in 0..180 {
            car.position = road.lane_position(0.0);
            car.update(
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            assert!(car.speed >= previous_speed - 0.02);
            previous_speed = car.speed;
        }
        assert!(car.engine_rpm > IDLE_RPM);
        assert!(car.engine_rpm < REDLINE_RPM);
        assert!(car.engine_load > 0.45);

        // Begin below the first shift in first gear. The previous acceleration
        // may already have shifted; hysteresis correctly retains that gear.
        car.gear = 1;
        let load_before_shift = car.engine_load;
        car.velocity = road.tangent.map(|v| v * (car.upshift_speed(1) - 0.02));
        car.refresh_speed();
        assert_eq!(car.gear, 1);
        car.velocity = road.tangent.map(|v| v * (car.upshift_speed(1) + 0.08));
        car.update(
            Duration::from_secs_f64(1.0 / 60.0),
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
        assert_eq!(car.gear, 2);
        assert!(
            car.engine_load < load_before_shift,
            "shift should briefly unload the engine: {} -> {}",
            load_before_shift,
            car.engine_load
        );

        car.reset(road.lane_position(0.0), road.heading());
        assert!(car.toggle_direction());
        for _ in 0..60 {
            car.position = road.lane_position(0.0);
            car.update(
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert_eq!(car.gear, 0);
        assert!(car.engine_rpm > IDLE_RPM);
        assert!(car.engine_rpm <= REDLINE_RPM);
    }

    #[test]
    fn every_car_can_reverse_brake_and_drive_forward_again() {
        let road = track::at_distance(50.0);
        for model in crate::cars::ALL {
            let mut v = Vehicle::new(road.lane_position(0.0), road.heading(), model);
            assert!(v.toggle_direction());
            assert_eq!(v.gear_label(), "R");
            for _ in 0..720 {
                v.position = road.lane_position(0.0); // straight-line test treadmill
                v.update(
                    Duration::from_secs_f64(1.0 / 60.0),
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            assert!(
                v.speed < -3.0 && v.speed >= -REVERSE_SPEED,
                "reverse speed for {model:?}: {}",
                v.speed
            );
            assert_eq!(v.gear, 0);
            assert!(!v.toggle_direction(), "moving car cannot select drive");
            v.update(
                Duration::from_secs(1),
                Controls {
                    brake: 1.0,
                    ..Controls::default()
                },
            );
            assert!(v.speed.abs() < 0.001);
            assert_eq!(v.direction, Direction::Reverse);
            assert!(v.toggle_direction());
            assert_eq!(v.gear_label(), "1");
            v.update(
                Duration::from_millis(250),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            assert!(v.speed > 1.0);
        }
    }

    #[test]
    fn reverse_steering_and_wheel_rotation_have_the_correct_sign() {
        let mut forward = car();
        let mut reverse = car();
        reverse.toggle_direction();
        let heading = forward.heading;
        forward.velocity = [heading.sin() * 4.0, heading.cos() * 4.0];
        reverse.velocity = forward.velocity.map(|v| -v);
        forward.wheel_phase = [1.0; 2];
        reverse.wheel_phase = [1.0; 2];
        let controls = Controls {
            steering: 1.0,
            ..Controls::default()
        };
        forward.update(Duration::from_millis(16), controls);
        reverse.update(Duration::from_millis(16), controls);
        assert!(crate::ai::angle(forward.heading - heading) < 0.0);
        assert!(crate::ai::angle(reverse.heading - heading) > 0.0);
        assert!(forward.front_wheel_angle() < 0.0 && reverse.front_wheel_angle() < 0.0);
        assert!(forward.wheel_phase[1] > 1.0 && reverse.wheel_phase[1] < 1.0);
    }

    #[test]
    fn gear_selector_rejects_sideways_motion_and_reset_selects_drive() {
        let mut v = car();
        v.velocity = [0.0, 2.0];
        assert!(!v.toggle_direction());
        v.velocity = [0.1, 0.0];
        assert!(v.toggle_direction());
        assert_eq!(
            v.velocity,
            [0.1, 0.0],
            "selecting gear must not teleport velocity"
        );
        v.reset(v.position, v.heading);
        assert_eq!(v.direction, Direction::Drive);
        assert_eq!(v.gear, 1);
        assert_eq!(v.speed, 0.0);
    }
}
