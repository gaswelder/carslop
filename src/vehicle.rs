use crate::{cars::CarModel, track};
use std::time::Duration;

pub const REVERSE_SPEED: f32 = 6.0;
// Keep zero for reverse and 1..=N for forward ratios.
pub const NEUTRAL_GEAR: u8 = u8::MAX;
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
    pub track: std::sync::Arc<track::Track>,
    pub position: [f32; 3],
    pub velocity: [f32; 2],
    pub speed: f32,
    pub heading: f32,
    pub gear: u8,
    pub manual: bool,
    pub direction: Direction,
    pub slip: f32,
    pub body_pitch: f32,
    pub body_roll: f32,
    pub curb_bump: f32,
    pub engine_rpm: f32,
    pub engine_load: f32,
    pub turbo_spool: f32,      // normalized turbine speed / available boost
    pub turbo_boost: f32,      // normalized manifold boost
    pub wheel_phase: [f32; 2], // rear/front axles; cosmetic, never affects physics
    pub tire_scrub: f32,
    brake_pressure: f32,
    steering: f32,
    yaw_rate: f32,
    shift_timer: f32,
    acceleration: [f32; 2], // longitudinal/lateral, filtered for load transfer
    pitch_rate: f32,
    roll_rate: f32,
    vertical_speed: f32,
    suspension_initialized: bool,
    wheel_spin: f32,
    axle_load: [f32; 2],
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
        track: std::sync::Arc<track::Track>,
    ) -> Self {
        Self {
            model,
            track,
            position,
            velocity: [0.0; 2],
            speed: 0.0,
            heading,
            gear: 1,
            manual: false,
            direction: Direction::Drive,
            slip: 0.0,
            body_pitch: 0.0,
            body_roll: 0.0,
            curb_bump: 0.0,
            engine_rpm: IDLE_RPM,
            engine_load: 0.0,
            turbo_spool: 0.0,
            turbo_boost: 0.0,
            brake_pressure: 0.0,
            steering: 0.0,
            yaw_rate: 0.0,
            wheel_phase: [0.0; 2],
            tire_scrub: 0.0,
            shift_timer: 0.0,
            acceleration: [0.0; 2],
            pitch_rate: 0.0,
            roll_rate: 0.0,
            vertical_speed: 0.0,
            suspension_initialized: false,
            wheel_spin: 0.0,
            axle_load: [0.0; 2],
        }
    }

    pub fn reset(&mut self, position: [f32; 3], heading: f32) {
        let manual = self.manual;
        *self = Self::on_track(position, heading, self.model, self.track.clone());
        self.manual = manual;
    }

    /// Reference speed for physics checks; road load and gearing determine actual speed.
    #[cfg(test)]
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
        if self.gear == NEUTRAL_GEAR {
            return IDLE_RPM;
        }
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

    pub fn shift(&mut self, up: bool) {
        let next = match (self.gear, up) {
            (0, true) => NEUTRAL_GEAR,
            (NEUTRAL_GEAR, true) => 1,
            (NEUTRAL_GEAR, false) => 0,
            (1, false) => NEUTRAL_GEAR,
            (0, false) => 0,
            (gear, true) => (gear + 1).min(self.model.spec().gearbox.gears()),
            (gear, false) => gear - 1,
        };
        // Direction changes require the whole vehicle to be nearly stopped.
        if (next == 0 || self.gear == 0) && self.velocity[0].hypot(self.velocity[1]) > 0.5 {
            return;
        }
        if next != self.gear {
            self.gear = next;
            self.direction = if next == 0 {
                Direction::Reverse
            } else {
                Direction::Drive
            };
            self.shift_timer = 0.16;
        }
    }

    pub fn gear_label(&self) -> String {
        if self.gear == NEUTRAL_GEAR {
            return "N".into();
        }
        if self.direction == Direction::Reverse {
            "R".into()
        } else {
            self.gear.to_string()
        }
    }

    pub fn update(&mut self, dt: Duration, controls: Controls) -> f32 {
        let steps = (dt.as_secs_f32() * 240.0).ceil().max(1.0) as u32;
        let seconds = dt.as_secs_f32() / steps as f32;
        let spec = self.model.spec();
        let dynamics = &spec.dynamics;
        let grip = spec.lateral_force / spec.mass;
        let reversing = self.direction == Direction::Reverse;
        let direction = if reversing { -1.0 } else { 1.0 };
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
            let previous_gear = self.gear;
            self.refresh_speed();
            if previous_gear != self.gear {
                self.shift_timer = 0.16;
            }
            self.shift_timer = (self.shift_timer - seconds).max(0.0);
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
            let longitudinal = dot(self.velocity, forward);
            let lateral = dot(self.velocity, right);
            let a = spec.wheelbase * (1.0 - dynamics.front_weight);
            let b = spec.wheelbase * dynamics.front_weight;
            let delta = -self.steering_angle(longitudinal);
            let (sin, cos) = delta.sin_cos();

            // Suspension supports the chassis rather than pinning it to terrain.
            // At a crest its contact force can drop to zero; gravity then governs
            // flight until the spring compresses again on landing.
            let ground = self.ride_height();
            if !self.suspension_initialized {
                self.position[1] = ground;
                self.suspension_initialized = true;
            }
            let omega = std::f32::consts::TAU * dynamics.suspension_hz;
            let support = (9.81 + omega * omega * (ground - self.position[1])
                - 1.3 * omega * self.vertical_speed)
                .clamp(0.0, 49.05);
            self.vertical_speed += (support - 9.81) * seconds;
            self.position[1] += self.vertical_speed * seconds;
            let transfer = spec.mass * self.acceleration[0] * dynamics.cg_height / spec.wheelbase;
            let total_load = spec.mass * support;
            let front_load = (total_load * dynamics.front_weight - transfer).clamp(0.0, total_load);
            self.axle_load = [front_load, total_load - front_load];
            let mu = grip / 9.81;
            let capacities = [0, 1].map(|axle| {
                // Outside tires gain less grip than inside tires lose. This
                // introduces lateral weight-transfer sensitivity without a full
                // four-wheel rigid-body solver.
                let load = self.axle_load[axle];
                let share = if axle == 0 {
                    dynamics.front_weight
                } else {
                    1.0 - dynamics.front_weight
                };
                let transfer = spec.mass * self.acceleration[1].abs() * dynamics.cg_height
                    / dynamics.track_width
                    * share;
                let imbalance = (2.0 * transfer / load.max(1.0)).clamp(0.0, 1.0);
                mu * load * (1.0 - 0.18 * imbalance * imbalance)
            });
            let front_v = lateral + self.yaw_rate * a;
            let rear_v = lateral - self.yaw_rate * b;
            let wheel_forward = longitudinal * cos + front_v * sin;
            let wheel_side = front_v * cos - longitudinal * sin;
            let throttle = controls.throttle.clamp(0.0, 1.0);
            self.update_turbo(seconds, throttle, longitudinal);
            let drive = self.drive_force(longitudinal) * throttle * direction;
            let engine_braking = (1.0 - throttle) * self.engine_brake_force(longitudinal);
            let brake = self.brake_pressure * brake_accel * spec.mass;
            let front_drive = dynamics.front_drive;
            let requested_x = [
                drive * front_drive
                    - longitudinal.signum()
                        * (brake * dynamics.brake_bias + engine_braking * front_drive),
                drive * (1.0 - front_drive)
                    - longitudinal.signum()
                        * (brake * (1.0 - dynamics.brake_bias)
                            + engine_braking * (1.0 - front_drive)
                            + if controls.handbrake {
                                spec.mass * 5.5
                            } else {
                                0.0
                            }),
            ];
            let mut forces = [[0.0; 2]; 2];
            self.wheel_spin = 0.0;
            for axle in 0..2 {
                let capacity = capacities[axle];
                let locked = axle == 1 && controls.handbrake;
                // ABS regulates foot braking only. Driven-wheel saturation and
                // rear-wheel handbrake lock consume the same tire force budget.
                let limit = capacity
                    * if self.brake_pressure > 0.0 && !locked {
                        0.92
                    } else {
                        1.0
                    };
                let fx = requested_x[axle].clamp(-limit, limit);
                let side = if axle == 0 { wheel_side } else { rear_v };
                let along = if axle == 0 {
                    wheel_forward
                } else {
                    longitudinal
                };
                // atan slip curve peaks, then gently falls toward sliding grip.
                // A speed floor damps the otherwise singular parking regime.
                let alpha = side.atan2(along.abs().max(2.5));
                let stiffness = dynamics.corner_stiffness[axle];
                let normalized = stiffness * alpha / capacity.max(1.0);
                let fy = -capacity
                    * normalized.tanh()
                    * (1.0 - 0.18 * (alpha.abs() / 0.6).clamp(0.0, 1.0))
                    * if locked { 0.18 } else { 1.0 };
                let scale = (capacity / fx.hypot(fy).max(0.001)).min(1.0);
                forces[axle] = [fx * scale, fy * scale];
                if throttle > 0.0 {
                    self.wheel_spin = self.wheel_spin.max(
                        ((requested_x[axle].abs() - capacity) / (spec.mass * 4.0)).clamp(0.0, 1.0),
                    );
                }
            }
            let front_x = forces[0][0] * cos - forces[0][1] * sin;
            let front_y = forces[0][0] * sin + forces[0][1] * cos;
            let ax = (front_x + forces[1][0]) / spec.mass;
            let ay = (front_y + forces[1][1]) / spec.mass;
            // Integrate yaw torque; no target yaw, grip clamp, or automatic
            // faster unwinding. Countersteering must generate opposing torque.
            self.yaw_rate += (a * front_y - b * forces[1][1]) / dynamics.yaw_inertia * seconds;
            let mut new_longitudinal = longitudinal + (ax + dot(gravity, forward)) * seconds;
            let resistance = (0.25 + drag * longitudinal * longitudinal) * seconds;
            // Static friction holds the car without making brakes propel it
            // backwards as speed crosses zero.
            if longitudinal * new_longitudinal < 0.0 && (brake > 0.0 || controls.handbrake) {
                new_longitudinal = 0.0;
            }
            if longitudinal.abs() < 0.1
                && throttle == 0.0
                && (brake
                    + if controls.handbrake {
                        spec.mass * 5.5
                    } else {
                        0.0
                    })
                    > dot(gravity, forward).abs() * spec.mass
            {
                new_longitudinal = 0.0;
            }
            new_longitudinal =
                new_longitudinal.signum() * (new_longitudinal.abs() - resistance).max(0.0);
            let new_lateral = lateral + (ay + dot(gravity, right)) * seconds;
            self.velocity = [
                forward[0] * new_longitudinal + right[0] * new_lateral,
                forward[1] * new_longitudinal + right[1] * new_lateral,
            ];
            let blend = 1.0 - (-seconds * 12.0).exp();
            // Use the acceleration actually applied after stopping friction has
            // limited braking. Requested tire force can remain nonzero at rest.
            let applied_ax = if new_longitudinal == 0.0 {
                (new_longitudinal - longitudinal) / seconds.max(f32::EPSILON)
            } else {
                ax
            };
            self.acceleration[0] += (applied_ax - self.acceleration[0]) * blend;
            self.acceleration[1] += (ay - self.acceleration[1]) * blend;
            self.update_body_motion(seconds);
            self.heading =
                (self.heading + self.yaw_rate * seconds).rem_euclid(std::f32::consts::TAU);
            let previous_position = self.position;
            self.position[0] += self.velocity[0] * seconds;
            self.position[2] += self.velocity[1] * seconds;
            let spin = longitudinal * seconds / spec.gearbox.rolling_radius;
            let spin_slip = self.wheel_spin * seconds * 35.0 * direction;
            self.wheel_phase[1] = (self.wheel_phase[1] + spin + spin_slip * dynamics.front_drive)
                .rem_euclid(std::f32::consts::TAU);
            if !controls.handbrake {
                self.wheel_phase[0] =
                    (self.wheel_phase[0] + spin + spin_slip * (1.0 - dynamics.front_drive))
                        .rem_euclid(std::f32::consts::TAU);
            }
            let chassis_height = self.position[1];
            impact = impact.max(self.resolve_road());
            self.position[1] = chassis_height;
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
        self.update_engine_feedback(controls, dt.as_secs_f32());
        let motion = self.velocity[0].hypot(self.velocity[1]);
        let sliding = ((self.slip.abs() - 0.8) / 3.0).clamp(0.0, 1.0);
        let braking = if controls.handbrake { 0.8 } else { 0.0 };
        self.tire_scrub =
            sliding.max(braking).max(self.wheel_spin) * ((motion - 1.5) / 2.5).clamp(0.0, 1.0);
        impact
    }

    fn ride_height(&self) -> f32 {
        self.track.city().map_or_else(
            || self.track.project(self.position).point[1],
            |city| city.drive_height([self.position[0], self.position[2]]),
        ) + 0.48
    }

    fn drive_force(&self, speed: f32) -> f32 {
        if self.gear == NEUTRAL_GEAR {
            return 0.0;
        }
        let spec = self.model.spec();
        let g = &spec.gearbox;
        let ratio = if self.direction == Direction::Reverse {
            g.reverse_ratio
        } else {
            g.ratios[self.gear.saturating_sub(1).min(g.gears() - 1) as usize]
        };
        let rpm = g.rpm(speed, self.gear).max(2200.0);
        // Broad torque plateau with weaker idle and redline torque. The clutch
        // slips at launch; gearing subsequently sets wheel force and shift loss.
        let torque_factor = if rpm < 3500.0 {
            0.60 + 0.40 * (rpm / 3500.0)
        } else {
            (1.0 - 0.10 * ((rpm - 3500.0) / 3000.0)).max(0.0)
        };
        let force = spec.dynamics.peak_torque * torque_factor * ratio * g.final_drive * 0.90
            / g.rolling_radius;
        let force = force.min(spec.engine_force);
        let force = if self.shift_timer > 0.0 {
            force * 0.2
        } else {
            force
        };
        let force = force * self.boost_power_fraction();
        if self.direction == Direction::Reverse {
            force * (1.0 - (speed.abs() / REVERSE_SPEED).powi(2)).max(0.0)
        } else if rpm >= REDLINE_RPM {
            0.0
        } else if let Some(engine) = &spec.road_engine {
            force.min(engine.wheel_power_watts * self.boost_power_fraction() / speed.abs().max(1.0))
        } else {
            force
        }
    }

    fn boost_power_fraction(&self) -> f32 {
        self.model.spec().turbo.as_ref().map_or(1.0, |t| {
            t.off_boost_fraction + (1.0 - t.off_boost_fraction) * self.turbo_boost
        })
    }

    fn update_turbo(&mut self, dt: f32, throttle: f32, speed: f32) {
        let Some(tune) = &self.model.spec().turbo else {
            self.turbo_spool = 0.0;
            self.turbo_boost = 0.0;
            return;
        };
        let loaded = self.gear != NEUTRAL_GEAR && self.shift_timer <= 0.0;
        let rpm = self.model.spec().gearbox.rpm(speed, self.gear).max(2200.0);
        let exhaust = ((rpm - tune.spool_start_rpm) / (tune.full_boost_rpm - tune.spool_start_rpm))
            .clamp(0.0, 1.0);
        let target = if loaded {
            exhaust * throttle * throttle
        } else {
            0.0
        };
        let time = if target > self.turbo_spool {
            tune.spool_seconds
        } else {
            tune.decay_seconds
        };
        self.turbo_spool += (target - self.turbo_spool) * (1.0 - (-dt / time).exp());
        // Closing the throttle vents manifold pressure while the turbine coasts.
        let boost_target = if loaded {
            self.turbo_spool * throttle
        } else {
            0.0
        };
        self.turbo_boost += (boost_target - self.turbo_boost) * (1.0 - (-dt / 0.06).exp());
    }

    fn engine_brake_force(&self, speed: f32) -> f32 {
        if self.gear == NEUTRAL_GEAR {
            return 0.0;
        }
        let g = &self.model.spec().gearbox;
        let ratio = if self.direction == Direction::Reverse {
            g.reverse_ratio
        } else {
            g.ratios[self.gear.saturating_sub(1).min(g.gears() - 1) as usize]
        };
        let rpm = g.rpm(speed, self.gear);
        self.model.spec().dynamics.peak_torque
            * 0.10
            * (rpm / 3500.0).clamp(0.0, 1.5)
            * ratio
            * g.final_drive
            / g.rolling_radius
            * (speed.abs() / 2.0).clamp(0.0, 1.0)
    }

    fn update_body_motion(&mut self, dt: f32) {
        let d = &self.model.spec().dynamics;
        let target_pitch = (self.acceleration[0] * d.cg_height / 80.0).clamp(-0.10, 0.10);
        let target_roll = (self.acceleration[1] * d.cg_height / 65.0).clamp(-0.12, 0.12);
        let omega = std::f32::consts::TAU * d.suspension_hz;
        self.pitch_rate +=
            (omega * omega * (target_pitch - self.body_pitch) - 1.4 * omega * self.pitch_rate) * dt;
        self.roll_rate +=
            (omega * omega * (target_roll - self.body_roll) - 1.4 * omega * self.roll_rate) * dt;
        self.body_pitch += self.pitch_rate * dt;
        self.body_roll += self.roll_rate * dt;
    }

    fn update_engine_feedback(&mut self, controls: Controls, dt: f32) {
        let throttle = controls.throttle.clamp(0.0, 1.0);
        let target_rpm = (self.target_engine_rpm() + self.wheel_spin * throttle * 1800.0)
            .max(IDLE_RPM + throttle * (2200.0 - IDLE_RPM))
            .min(REDLINE_RPM);
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
        let lock = 0.54
            / (1.0
                + speed.abs() / 22.0
                + self.model.spec().dynamics.steering_speed_squared * speed * speed);
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
            impact = crate::city::deflect(
                &mut self.velocity,
                normal.map(|component| component * excess.signum()),
            );
        }
        self.position[1] = road.point[1] + 0.48;
        impact
    }

    pub fn refresh_speed(&mut self) {
        self.speed = dot(self.velocity, [self.heading.sin(), self.heading.cos()]);
        self.slip = dot(self.velocity, [self.heading.cos(), -self.heading.sin()]);
        if self.manual {
            return;
        }
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

    #[cfg(test)]
    pub fn collide(&mut self, other: &mut Vehicle) -> f32 {
        debug_assert!(
            std::sync::Arc::ptr_eq(&self.track, &other.track),
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
        let heights = [self.position[1], other.position[1]];
        let wall_impact = self.resolve_road().max(other.resolve_road());
        if self.suspension_initialized {
            self.position[1] = heights[0];
        }
        if other.suspension_initialized {
            other.position[1] = heights[1];
        }
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
    fn manual_shifts_hold_gears_and_pass_through_neutral() {
        let mut car = Vehicle::new([0.0; 3], 0.0, CarModel::Roadster);
        car.manual = true;
        car.velocity = [0.0, 40.0];
        car.refresh_speed();
        assert_eq!(car.gear, 1);
        car.shift(true);
        car.refresh_speed();
        assert_eq!(car.gear, 2);
        car.velocity = [0.0; 2];
        car.refresh_speed();
        assert_eq!(car.gear, 2);
        car.shift(false);
        car.shift(false);
        assert_eq!(car.gear_label(), "N");
        assert_eq!(car.drive_force(0.0), 0.0);
        assert_eq!(car.engine_brake_force(10.0), 0.0);
        car.velocity = [2.0, 0.0];
        car.shift(false);
        assert_eq!(car.gear_label(), "N");
        car.velocity = [0.0; 2];
        car.shift(false);
        assert_eq!(car.gear_label(), "R");
        car.shift(true);
        assert_eq!(car.gear_label(), "N");
        car.shift(true);
        assert_eq!(car.gear_label(), "1");
        for _ in 0..20 {
            car.shift(true);
        }
        assert_eq!(car.gear, car.model.spec().gearbox.gears());
        car.reset([0.0; 3], 0.0);
        assert!(car.manual);
        assert_eq!(car.gear, 1);
    }

    #[test]
    fn kestrel_city_speed_cornering_stays_balanced() {
        for input in [-1.0, -0.5, -0.25, 0.25, 0.5, 1.0] {
            for throttle in [0.0, 0.35, 1.0] {
                let mut car = level_route_car(track::TrackId::Scenic, CarModel::Roadster);
                let road = car.track.at_distance(50.0);
                let speed = 40.0 / 3.6;
                let mut peak_slip: f32 = 0.0;
                for _ in 0..480 {
                    let lateral = dot(car.velocity, [car.heading.cos(), -car.heading.sin()]);
                    car.position = road.lane_position(0.0);
                    car.velocity = [
                        car.heading.sin() * speed + car.heading.cos() * lateral,
                        car.heading.cos() * speed - car.heading.sin() * lateral,
                    ];
                    car.update(
                        Duration::from_secs_f64(1.0 / 240.0),
                        Controls {
                            steering: input,
                            throttle,
                            ..Controls::default()
                        },
                    );
                    peak_slip = peak_slip.max(car.slip.abs().atan2(car.speed.abs()).to_degrees());
                }
                println!(
                    "input {input} throttle {throttle}: peak slip {peak_slip:.2}, yaw {:.2}",
                    car.yaw_rate
                );
                assert!(
                    peak_slip < 3.0,
                    "40 km/h turn should remain controllable: {input}, {throttle}, {peak_slip}"
                );
            }
        }
    }

    #[test]
    fn driven_tires_share_cornering_grip_and_yaw_has_inertia() {
        let run = |throttle| {
            let mut car = level_route_car(track::TrackId::Scenic, CarModel::Roadster);
            let h = car.heading;
            car.velocity = [
                h.sin() * 10.0 + h.cos() * 2.0,
                h.cos() * 10.0 - h.sin() * 2.0,
            ];
            car.update(
                Duration::from_secs_f64(1.0 / 240.0),
                Controls {
                    throttle,
                    ..Controls::default()
                },
            );
            dot(car.velocity, [h.cos(), -h.sin()])
        };
        assert!(
            run(1.0) > run(0.0),
            "drive torque must reduce available cornering force"
        );
        let mut car = level_route_car(track::TrackId::Scenic, CarModel::Roadster);
        car.velocity = [car.heading.sin() * 20.0, car.heading.cos() * 20.0];
        car.yaw_rate = 0.5;
        car.update(Duration::from_secs_f64(1.0 / 240.0), Controls::default());
        assert!(
            car.yaw_rate > 0.0 && car.yaw_rate < 0.5,
            "tire torque should slow existing rotation without deleting it"
        );
    }

    #[test]
    fn gearing_changes_wheel_force_and_engine_braking() {
        let mut car = level_route_car(track::TrackId::Scenic, CarModel::Roadster);
        car.gear = 1;
        let first = car.drive_force(10.0);
        let first_drag = car.engine_brake_force(10.0);
        car.gear = 3;
        assert!(first > car.drive_force(10.0) * 1.4);
        assert!(first_drag > car.engine_brake_force(10.0) * 2.0);
        let engaged = car.drive_force(10.0);
        car.shift_timer = 0.1;
        assert!(car.drive_force(10.0) < engaged * 0.3);
    }

    #[test]
    fn acceleration_and_braking_transfer_axle_load() {
        for model in crate::cars::ALL {
            let mut car = level_route_car(track::TrackId::Scenic, model);
            let road = car.track.at_distance(50.0);
            car.velocity = road.tangent.map(|v| v * 10.0);
            car.refresh_speed();
            for _ in 0..60 {
                treadmill_tick(
                    &mut car,
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            let static_front = model.spec().mass * 9.81 * model.spec().dynamics.front_weight;
            assert!(car.axle_load[0] < static_front * 0.98);
            for _ in 0..100 {
                treadmill_tick(
                    &mut car,
                    Controls {
                        brake: 1.0,
                        ..Controls::default()
                    },
                );
            }
            assert!(car.axle_load[0] > static_front * 1.05);
        }
    }

    #[test]
    fn chassis_can_unload_land_and_settle() {
        let mut car = level_route_car(track::TrackId::Scenic, CarModel::Roadster);
        let ground = car.ride_height();
        car.suspension_initialized = true;
        car.position[1] = ground + 0.5;
        car.update(Duration::from_millis(20), Controls::default());
        assert_eq!(car.axle_load, [0.0; 2]);
        assert!(car.position[1] > ground + 0.49);
        assert!(car.vertical_speed < 0.0);
        for _ in 0..240 * 3 {
            car.update(Duration::from_secs_f64(1.0 / 240.0), Controls::default());
        }
        assert!((car.position[1] - ground).abs() < 0.002);
        assert!(car.vertical_speed.abs() < 0.002);
        assert!((car.axle_load.iter().sum::<f32>() / car.model.spec().mass - 9.81).abs() < 0.02);
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
        let track = std::sync::Arc::new(id.track().flat_test_track());
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
            let lateral = dot(car.velocity, [car.heading.cos(), -car.heading.sin()]);
            car.velocity = [
                car.heading.sin() * speed + car.heading.cos() * lateral,
                car.heading.cos() * speed - car.heading.sin() * lateral,
            ];
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
                let tap = crate::track::angle(car.heading - heading)
                    .abs()
                    .to_degrees();
                println!("{model:?} {speed} m/s: 100 ms tap {tap:.2} degrees");
                assert!(
                    tap > 0.01 && tap < 3.0,
                    "{model:?} at {speed}: tap rotated {tap} degrees"
                );
                if speed == 15.0 || speed == 25.0 {
                    assert!(tap > 0.5,"a city-speed tap must make a useful correction: {model:?} at {speed}: {tap}");
                }
                assert_eq!(car.steering, 0.0);
                assert!(car.yaw_rate.abs() < 0.2, "residual yaw {}", car.yaw_rate);
                let mut held = route_car(track::TrackId::Scenic, model);
                steering_run(&mut held, speed, 1.0, 1.0, 120.0);
                let turn = crate::track::angle(held.heading - heading)
                    .abs()
                    .to_degrees();
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
    fn tire_torque_turns_and_countersteering_reverses_rotation() {
        for model in crate::cars::ALL {
            let mut car = route_car(track::TrackId::Scenic, model);
            steering_run(&mut car, 15.0, 0.25, 0.6, 240.0);
            assert!(car.yaw_rate < 0.0);
            assert!(car.yaw_rate.is_finite() && car.yaw_rate.abs() < 2.0);
            steering_run(&mut car, 15.0, -0.5, 0.8, 240.0);
            assert!(
                car.yaw_rate > 0.0,
                "{model:?} countersteering remained in original turn"
            );
            assert!(car.front_wheel_angle() > 0.0);
            steering_run(&mut car, 0.0, 0.0, 0.02, 240.0);
            assert!(car.yaw_rate.is_finite());
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
                let error = crate::track::angle(car.heading - reference.heading)
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
            results[2] > results[1] && results[1] > results[0],
            "city-speed lock: Club > Muscle > stabilized Kestrel: {results:?}"
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
        let mut car = Vehicle::on_track(position, heading, CarModel::Roadster, track.clone());
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
            (33.0..44.0).contains(&distance),
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
        assert!(std::sync::Arc::ptr_eq(
            &compact.track,
            &track::TrackId::Compact.track()
        ));
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
        assert!(std::sync::Arc::ptr_eq(
            &car.track,
            &track::TrackId::Scenic.track()
        ));
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
        let [roadster, muscle, club] = [results[0], results[1], results[2]];
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
            assert!(
                v.speed > 15.0 && v.speed <= v.speed_cap(),
                "{model:?} terminal {} cap {}",
                v.speed,
                v.speed_cap()
            );
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
        assert!(dot(light.velocity, n) < 0.0);
        for car in [&heavy, &light] {
            assert!(car
                .position
                .iter()
                .chain(car.velocity.iter())
                .all(|v| v.is_finite()));
        }
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
        assert!(v.wheel_phase.iter().all(|phase| phase.abs() < 1e-6));
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
    fn holding_brakes_at_rest_does_not_dive_and_settles_after_stopping() {
        for handbrake in [false, true] {
            for initial_speed in [0.0, 8.0, -8.0] {
                let mut v = car();
                let forward = [v.heading.sin(), v.heading.cos()];
                v.velocity = forward.map(|component| component * initial_speed);
                v.refresh_speed();
                for _ in 0..600 {
                    v.update(
                        Duration::from_secs_f64(1.0 / 60.0),
                        Controls {
                            brake: if handbrake { 0.0 } else { 1.0 },
                            handbrake,
                            ..Controls::default()
                        },
                    );
                    if initial_speed == 0.0 {
                        assert!(v.body_pitch.abs() < 0.0001);
                    }
                }
                assert!(v.speed.abs() < 0.001);
                assert!(v.body_pitch.abs() < 0.0001);
            }
        }
    }

    #[test]
    fn sprung_body_motion_responds_to_braking_throttle_and_cornering() {
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
        forward.acceleration[0] = 3.0;
        reverse.acceleration[0] = -3.0;
        for _ in 0..240 {
            forward.update_body_motion(1.0 / 240.0);
            reverse.update_body_motion(1.0 / 240.0);
        }
        assert!(forward.body_pitch > 0.005);
        assert!((forward.body_pitch + reverse.body_pitch).abs() < 0.00001);
        reverse.acceleration[0] = 4.0;
        for _ in 0..240 {
            reverse.update_body_motion(1.0 / 240.0);
        }
        assert!(
            reverse.body_pitch > 0.01,
            "reverse braking should lift the nose"
        );
        forward.acceleration = [0.0; 2];
        reverse.acceleration = [0.0; 2];
        forward.body_pitch = 0.05;
        reverse.body_pitch = 0.05;
        for _ in 0..30 {
            forward.update_body_motion(1.0 / 30.0);
        }
        for _ in 0..144 {
            reverse.update_body_motion(1.0 / 144.0);
        }
        assert!((forward.body_pitch - reverse.body_pitch).abs() < 0.0002);
        assert!(forward.body_pitch.abs() < 0.0005);
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
    fn engine_feedback_smooths_rpm_load_and_shift_unload() {
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
            assert!(v.speed > 0.5, "{model:?} failed to pull away: {}", v.speed);
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
        assert!(crate::track::angle(forward.heading - heading) < 0.0);
        assert!(crate::track::angle(reverse.heading - heading) > 0.0);
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

#[cfg(test)]
mod turbo_tests {
    use super::*;

    #[test]
    fn boost_lags_adds_force_and_dumps_on_lift_shift_and_reset() {
        let model = crate::cars::ALL
            .into_iter()
            .find(|m| m.spec().turbo.is_some())
            .unwrap();
        let mut car = Vehicle::new([0.0; 3], 0.0, model);
        car.gear = 3;
        let speed = model.spec().gearbox.speed_at_rpm(4200.0, 3);
        let cold_force = car.drive_force(speed);
        car.update_turbo(0.02, 1.0, speed);
        assert!(car.turbo_boost < 0.05);
        for _ in 0..1500 {
            car.update_turbo(0.004, 1.0, speed);
        }
        assert!(car.turbo_boost > 0.99);
        assert!(car.drive_force(speed) > cold_force * 1.6);
        for _ in 0..30 {
            car.update_turbo(0.004, 0.0, speed);
        }
        assert!(car.turbo_boost < 0.15);
        assert!(car.turbo_spool > car.turbo_boost);
        car.shift(true);
        car.update_turbo(0.1, 1.0, speed);
        assert!(car.turbo_boost < 0.05);
        car.reset([0.0; 3], 0.0);
        assert_eq!(car.turbo_spool, 0.0);
        assert_eq!(car.turbo_boost, 0.0);
        car.gear = NEUTRAL_GEAR;
        car.update_turbo(5.0, 1.0, speed);
        assert_eq!(car.turbo_spool, 0.0);
    }

    #[test]
    fn low_rpm_and_naturally_aspirated_cars_do_not_build_boost() {
        for model in crate::cars::ALL {
            let mut car = Vehicle::new([0.0; 3], 0.0, model);
            let speed = model.spec().gearbox.speed_at_rpm(1000.0, 1);
            for _ in 0..1000 {
                car.update_turbo(0.004, 1.0, speed);
            }
            if model.spec().turbo.is_none() {
                assert_eq!(car.boost_power_fraction(), 1.0);
                assert_eq!(car.turbo_spool, 0.0);
            } else {
                assert!(car.turbo_boost < 0.12);
            }
        }
    }
}
