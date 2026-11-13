use crate::geometry::{XYZ, XZ};
use crate::{cars::CarModel, track};
use std::time::Duration;

// Keep zero for reverse and 1..=N for forward ratios.
pub const NEUTRAL_GEAR: u8 = u8::MAX;
pub const IDLE_RPM: f32 = 900.0;
pub const REDLINE_RPM: f32 = 6500.0;
const REV_MATCH_SECONDS: f32 = 0.22;
const SOUND_SHIFT_SECONDS: f32 = 0.36;

// Base rate of normalized steering input change per second.
const STEERING_RATE: f32 = 9.0;
// Forward speed (m/s) at which released steering reaches its full return rate.
const STEERING_CENTERING_SPEED: f32 = 10.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction {
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
    pub track: std::sync::Arc<track::Location>,
    pub position: XYZ,
    pub velocity: XZ<f32>,
    pub speed: f32,
    pub heading: f32,
    pub gear: u8,
    pub manual: bool,
    direction: Direction,
    slip: f32,
    pub body_pitch: f32,
    pub body_roll: f32,
    pub suspension_pitch: f32,
    pub suspension_roll: f32,
    pub curb_bump: f32,
    pub engine_rpm: f32,
    pub engine_load: f32,
    pub turbo_spool: f32,      // normalized turbine speed / available boost
    turbo_boost: f32,          // normalized manifold boost
    pub wheel_phase: [f32; 2], // rear/front axles; cosmetic, never affects physics
    pub tire_scrub: f32,
    pub brake_lights: bool,
    brake_pressure: f32,
    steering: f32,
    yaw_rate: f32,
    shift_timer: f32,
    sound_shift_timer: f32,
    rev_match_timer: f32,
    neutral_fuel_cut: bool,
    acceleration: [f32; 2], // longitudinal/lateral, filtered for load transfer
    pitch_rate: f32,
    roll_rate: f32,
    suspension_rates: [f32; 2],
    vertical_speed: f32,
    suspension_initialized: bool,
    pub wheel_spin: f32,
    launch_energy: f32, // stored flywheel energy released through the clutch (joules)
    axle_load: [f32; 2],
}

impl Vehicle {
    #[cfg(test)]
    fn new(position: XYZ, heading: f32, model: CarModel) -> Self {
        Self::on_track(
            position,
            heading,
            model,
            track::LocationId::Compact.try_track().unwrap(),
        )
    }

    pub fn on_track(
        position: XYZ,
        heading: f32,
        model: CarModel,
        track: std::sync::Arc<track::Location>,
    ) -> Self {
        let mut car = Self {
            model,
            track,
            position,
            velocity: XZ { x: 0.0, z: 0.0 },
            speed: 0.0,
            heading,
            gear: 1,
            manual: false,
            direction: Direction::Drive,
            slip: 0.0,
            body_pitch: 0.0,
            body_roll: 0.0,
            suspension_pitch: 0.0,
            suspension_roll: 0.0,
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
            brake_lights: false,
            shift_timer: 0.0,
            sound_shift_timer: 0.0,
            rev_match_timer: 0.0,
            neutral_fuel_cut: false,
            acceleration: [0.0; 2],
            pitch_rate: 0.0,
            roll_rate: 0.0,
            suspension_rates: [0.0; 2],
            vertical_speed: 0.0,
            suspension_initialized: false,
            wheel_spin: 0.0,
            launch_energy: 0.0,
            axle_load: [0.0; 2],
        };
        let (pitch, roll) = car.ground_angles();
        car.suspension_pitch = pitch;
        car.suspension_roll = roll;
        car
    }

    pub fn reset(&mut self, position: XYZ, heading: f32) {
        let manual = self.manual;
        *self = Self::on_track(position, heading, self.model, self.track.clone());
        self.manual = manual;
    }

    fn upshift_speed(&self, gear: u8) -> f32 {
        let gearbox = &self.model.spec().gearbox;
        gearbox.speed_at_rpm(gearbox.upshift_rpm, gear)
    }
    fn target_engine_rpm(&self) -> f32 {
        if self.gear == NEUTRAL_GEAR {
            return IDLE_RPM;
        }
        self.model
            .spec()
            .gearbox
            .rpm(self.speed, self.gear)
            .clamp(IDLE_RPM, REDLINE_RPM)
    }
    fn brake_accel(&self) -> f32 {
        self.model.spec().brake_force / self.model.spec().mass
    }

    #[cfg(test)]
    fn toggle_direction(&mut self) -> bool {
        // Check total motion, not just longitudinal speed, so slides cannot shift.
        if self.velocity.x.hypot(self.velocity.z) > 0.5 {
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
        if next != self.gear {
            let previous = self.gear;
            self.gear = next;
            self.direction = if next == 0 {
                Direction::Reverse
            } else {
                Direction::Drive
            };
            let launching = previous == NEUTRAL_GEAR && matches!(next, 0 | 1);
            self.shift_timer = if launching { 0.0 } else { 0.16 };
            self.sound_shift_timer = SOUND_SHIFT_SECONDS;
            if launching {
                self.launch_energy = self.flywheel_energy();
            }
            self.begin_rev_match(previous);
        }
    }

    fn begin_rev_match(&mut self, previous: u8) {
        // Only a moving forward downshift needs a throttle blip. Neutral,
        // reverse and shifts below idle should not rev the engine.
        self.rev_match_timer = if self.gear > 0
            && self.gear < previous
            && previous <= self.model.spec().gearbox.gears()
            && self.target_engine_rpm() > self.engine_rpm + 100.0
        {
            REV_MATCH_SECONDS
        } else {
            0.0
        };
    }

    #[cfg(test)]
    fn gear_label(&self) -> String {
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
        // The pedal switch works at rest and in reverse, independently of
        // hydraulic pressure, engine braking, and the parking brake.
        self.brake_lights = controls.brake > 0.0;
        let steps = (dt.as_secs_f32() * 240.0).ceil().max(1.0) as u32;
        let seconds = dt.as_secs_f32() / steps as f32;
        let spec = self.model.spec();
        let mu = spec.tire_friction_coefficient;
        let reversing = self.direction == Direction::Reverse;
        let direction = if reversing { -1.0 } else { 1.0 };
        let brake_accel = self.brake_accel();
        let mut impact: f32 = 0.0;
        for _ in 0..steps {
            let previous_gear = self.gear;
            self.refresh_speed();
            if previous_gear != self.gear {
                self.shift_timer = 0.16;
                self.sound_shift_timer = SOUND_SHIFT_SECONDS;
            }
            self.shift_timer = (self.shift_timer - seconds).max(0.0);
            self.rev_match_timer = (self.rev_match_timer - seconds).max(0.0);
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
            let forward = XZ {
                x: self.heading.sin(),
                z: self.heading.cos(),
            };
            let right = XZ {
                x: self.heading.cos(),
                z: -self.heading.sin(),
            };
            let gradient = self.track.ground_gradient(self.position);
            // Horizontal component of gravity projected onto y = h(x, z).
            // Velocity is stored in XZ, so divide by 1 + |gradient|².
            let gravity = gradient.map(|g| -9.81 * g / (1.0 + dot(gradient, gradient)));
            let longitudinal = dot(self.velocity, forward);
            let lateral = dot(self.velocity, right);
            let a = spec.wheelbase * (1.0 - spec.front_weight);
            let b = spec.wheelbase * spec.front_weight;
            let delta = -self.steering_angle();
            let (sin, cos) = delta.sin_cos();

            // Suspension supports the chassis rather than pinning it to terrain.
            // At a crest its contact force can drop to zero; gravity then governs
            // flight until the spring compresses again on landing.
            let ground = self.ride_height();
            if !self.suspension_initialized {
                self.position.y = ground;
                self.suspension_initialized = true;
            }
            let omega = std::f32::consts::TAU * spec.suspension_hz;
            let support = (9.81 + omega * omega * (ground - self.position.y)
                - 1.3 * omega * self.vertical_speed)
                .clamp(0.0, 49.05);
            self.vertical_speed += (support - 9.81) * seconds;
            self.position.y += self.vertical_speed * seconds;
            let transfer = spec.mass * self.acceleration[0] * spec.cg_height / spec.wheelbase;
            let total_load = spec.mass * support;
            let front_load = (total_load * spec.front_weight - transfer).clamp(0.0, total_load);
            self.axle_load = [front_load, total_load - front_load];
            let capacities = [0, 1].map(|axle| {
                // Outside tires gain less grip than inside tires lose. This
                // introduces lateral weight-transfer sensitivity without a full
                // four-wheel rigid-body solver.
                let load = self.axle_load[axle];
                let share = if axle == 0 {
                    spec.front_weight
                } else {
                    1.0 - spec.front_weight
                };
                let transfer = spec.mass * self.acceleration[1].abs() * spec.cg_height
                    / spec.track_width
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
            // Engaging a gear transfers the energy stored while revving in neutral.
            let launch = self.launch_force(seconds, longitudinal, throttle);
            let drive = (self.drive_force(longitudinal) * throttle + launch) * direction;
            let closed_throttle = ((0.15 - throttle) / 0.15).clamp(0.0, 1.0);
            let engine_braking = closed_throttle * self.engine_brake_force(longitudinal);
            let brake = self.brake_pressure * brake_accel * spec.mass;
            let drive_bias = spec.drive_bias;
            let requested_x = [
                drive * drive_bias
                    - longitudinal.signum()
                        * (brake * spec.brake_bias + engine_braking * drive_bias),
                drive * (1.0 - drive_bias)
                    - longitudinal.signum()
                        * (brake * (1.0 - spec.brake_bias)
                            + engine_braking * (1.0 - drive_bias)
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
                // Sliding tires retain some sideways authority. Reserve enough of the
                // friction circle for it during a powered burnout.
                let drive_share = if axle == 0 {
                    drive_bias
                } else {
                    1.0 - drive_bias
                };
                let spinning = drive.abs() * drive_share > capacity;
                let limit = if spinning {
                    let sideways = (self.slip.abs() / 2.0).clamp(0.0, 1.0);
                    if let Some([baseline, reduction, slip_angle]) =
                        spec.rear_combined_slip.filter(|_| axle == 1)
                    {
                        // Combined slip reduces driving traction as the rear slides,
                        // leaving lateral force for recovery and countersteering.
                        let angle = rear_v.abs().atan2(longitudinal.abs().max(2.5));
                        limit * (baseline - reduction * (angle / slip_angle).clamp(0.0, 1.0))
                    } else {
                        limit * (0.97 - 0.05 * sideways)
                    }
                } else {
                    limit
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
                let stiffness = spec.corner_stiffness[axle];
                let normalized = stiffness * alpha / capacity.max(1.0);
                let fy = -capacity
                    * normalized.tanh()
                    * (1.0 - 0.08 * (alpha.abs() / 0.6).clamp(0.0, 1.0))
                    * if locked { 0.18 } else { 1.0 };
                // Longitudinal saturation consumes the contact patch's grip.
                // Scaling an already-clamped drive force together with fy left
                // spinning tires with substantial lateral grip at full torque.
                let lateral_capacity = (capacity * capacity - fx * fx).max(0.0).sqrt();
                forces[axle] = [fx, fy.clamp(-lateral_capacity, lateral_capacity)];
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
            self.yaw_rate += (a * front_y - b * forces[1][1]) / spec.yaw_inertia * seconds;
            let mut new_longitudinal = longitudinal + (ax + dot(gravity, forward)) * seconds;
            let resistance = spec.resistance_accel(longitudinal) * seconds;
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
            self.velocity = XZ {
                x: forward.x * new_longitudinal + right.x * new_lateral,
                z: forward.z * new_longitudinal + right.z * new_lateral,
            };
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
            self.position.x += self.velocity.x * seconds;
            self.position.z += self.velocity.z * seconds;
            let spin = longitudinal * seconds / spec.gearbox.rolling_radius;
            let spin_slip = self.wheel_spin * seconds * 35.0 * direction;
            self.wheel_phase[1] = (self.wheel_phase[1] + spin + spin_slip * spec.drive_bias)
                .rem_euclid(std::f32::consts::TAU);
            if !controls.handbrake {
                self.wheel_phase[0] =
                    (self.wheel_phase[0] + spin + spin_slip * (1.0 - spec.drive_bias))
                        .rem_euclid(std::f32::consts::TAU);
            }
            let chassis_height = self.position.y;
            impact = impact.max(self.resolve_road());
            self.position.y = chassis_height;
            self.curb_bump *= (-seconds * 12.0).exp();
            if self.track.has_compiled_roads() {
                let city = &self.track;
                if let Some((normal, rise)) = city.crossed_curb(
                    XZ {
                        x: previous_position.x,
                        z: previous_position.z,
                    },
                    XZ {
                        x: self.position.x,
                        z: self.position.z,
                    },
                ) {
                    let across = dot(self.velocity, normal);
                    let loss = (across.abs() * 0.08).min(1.2) * (rise / 0.09);
                    for (velocity, normal) in self.velocity.iter_mut().zip(normal) {
                        *velocity -= normal * across.signum() * loss;
                    }
                    self.curb_bump = self
                        .curb_bump
                        .max(0.03 * (rise / 0.09) * (across.abs() / 6.0).min(1.0));
                }
            }
        }
        self.refresh_speed();
        self.update_engine_feedback(controls, dt.as_secs_f32());
        let motion = self.velocity.x.hypot(self.velocity.z);
        let sliding = ((self.slip.abs() - 0.8) / 3.0).clamp(0.0, 1.0);
        let braking = if controls.handbrake { 0.8 } else { 0.0 };
        self.tire_scrub =
            (sliding.max(braking) * ((motion - 1.5) / 2.5).clamp(0.0, 1.0)).max(self.wheel_spin);
        impact
    }

    fn ride_height(&self) -> f32 {
        self.track.drive_height(XZ {
            x: self.position.x,
            z: self.position.z,
        }) + crate::vehicle_location::VEHICLE_ORIGIN_HEIGHT
    }

    fn flywheel_energy(&self) -> f32 {
        let inertia = 0.12 + self.model.spec().engine_max_torque_nm * 0.0006;
        let omega = self.engine_rpm * std::f32::consts::TAU / 60.0;
        let clutch_rpm = 2200.0 * std::f32::consts::TAU / 60.0;
        (0.5 * inertia * (omega * omega - clutch_rpm * clutch_rpm)).max(0.0)
    }

    fn launch_force(&mut self, dt: f32, speed: f32, throttle: f32) -> f32 {
        if self.gear == NEUTRAL_GEAR || self.shift_timer > 0.0 {
            return 0.0;
        }
        if !matches!(self.gear, 0 | 1) || speed.abs() > 15.0 || throttle < 0.1 {
            self.launch_energy = 0.0;
            return 0.0;
        }
        let spec = self.model.spec();
        let clutch_speed = spec
            .gearbox
            .speed_at_rpm(self.engine_rpm, self.gear)
            .abs()
            .max(speed.abs())
            .max(3.0);
        let force = (self.launch_energy / 0.20 / clutch_speed).min(spec.mass * 6.0) * throttle;
        self.launch_energy = (self.launch_energy - force * clutch_speed * dt).max(0.0);
        force
    }

    fn drive_force(&self, speed: f32) -> f32 {
        self.model.spec().drive_force(
            speed,
            self.gear,
            self.direction == Direction::Reverse,
            self.boost_power_fraction(),
            self.shift_timer > 0.0,
        )
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
        // Regulate overrun torque as lateral tire demand rises, just as foot
        // braking is regulated: lifting in a bend must not lock the driven axle.
        let lateral_demand = (self.acceleration[1].abs()
            / (self.model.spec().tire_friction_coefficient * 9.81))
            .clamp(0.0, 1.0);
        self.model.spec().engine_max_torque_nm
            * 0.24
            * (1.0 - 0.70 * lateral_demand)
            * (rpm / 3500.0).clamp(0.0, 1.5)
            * ratio
            * g.final_drive
            / g.rolling_radius
            * (speed.abs() / 2.0).clamp(0.0, 1.0)
            * if self.shift_timer > 0.0 { 0.15 } else { 1.0 }
    }

    fn ground_angles(&self) -> (f32, f32) {
        let (sin, cos) = self.heading.sin_cos();
        let h = |dx: f32, dz: f32| {
            let p = XYZ {
                x: self.position.x + dx,
                y: self.position.y,
                z: self.position.z + dz,
            };
            self.track.drive_height(XZ { x: p.x, z: p.z })
        };
        let length = self.model.spec().wheelbase;
        let width = self.model.spec().track_width;
        (
            ((h(sin * length * 0.5, cos * length * 0.5)
                - h(-sin * length * 0.5, -cos * length * 0.5))
                / length)
                .atan()
                .clamp(-0.35, 0.35),
            ((h(cos * width * 0.5, -sin * width * 0.5) - h(-cos * width * 0.5, sin * width * 0.5))
                / width)
                .atan()
                .clamp(-0.25, 0.25),
        )
    }

    fn update_body_motion(&mut self, dt: f32) {
        let spec = self.model.spec();
        let target_pitch = (self.acceleration[0] * spec.cg_height / 80.0).clamp(-0.10, 0.10);
        let target_roll = (self.acceleration[1] * spec.cg_height / 65.0).clamp(-0.12, 0.12);
        let omega = std::f32::consts::TAU * spec.suspension_hz;
        self.pitch_rate +=
            (omega * omega * (target_pitch - self.body_pitch) - 1.4 * omega * self.pitch_rate) * dt;
        self.roll_rate +=
            (omega * omega * (target_roll - self.body_roll) - 1.4 * omega * self.roll_rate) * dt;
        self.body_pitch += self.pitch_rate * dt;
        self.body_roll += self.roll_rate * dt;
        // The sprung body follows terrain through damped angular springs.
        // Wheels follow the contact surface independently in the renderer.
        let (pitch, roll) = if self.axle_load.iter().sum::<f32>() > 0.0 {
            self.ground_angles()
        } else {
            (self.suspension_pitch, self.suspension_roll)
        };
        for (axis, target, angle) in [
            (0, pitch, &mut self.suspension_pitch),
            (1, roll, &mut self.suspension_roll),
        ] {
            let rate = &mut self.suspension_rates[axis];
            *rate += (omega * omega * (target - *angle) - 1.4 * omega * *rate) * dt;
            *angle += *rate * dt;
        }
    }

    fn update_neutral_engine(&mut self, throttle: f32, dt: f32) {
        let engine_max_torque_nm = self.model.spec().engine_max_torque_nm;
        // Approximate crank/flywheel inertia; larger engines carry more inertia.
        let inertia = 0.12 + engine_max_torque_nm * 0.0006;
        // Feedback runs once per frame. Substep free revs so limiter cycles and
        // throttle blips remain consistent at different rendering frame rates.
        let steps = (dt * 240.0).ceil().max(1.0) as u32;
        let seconds = dt / steps as f32;
        for _ in 0..steps {
            if self.engine_rpm >= REDLINE_RPM {
                self.neutral_fuel_cut = true;
            } else if self.engine_rpm <= REDLINE_RPM - 250.0 {
                self.neutral_fuel_cut = false;
            }
            let revs = ((self.engine_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)).clamp(0.0, 1.0);
            let combustion = if self.neutral_fuel_cut {
                0.0
            } else {
                throttle * engine_max_torque_nm * (1.0 - 0.3 * revs)
            };
            // Pumping/friction losses grow with speed. Unlike an RPM target,
            // a small pedal opening can keep accelerating an unloaded engine.
            let drag = engine_max_torque_nm * (0.10 + 0.16 * revs + 0.24 * revs * revs);
            let rpm_rate = (combustion - drag) / inertia * 60.0 / std::f32::consts::TAU;
            self.engine_rpm = (self.engine_rpm + rpm_rate * seconds).clamp(IDLE_RPM, REDLINE_RPM);
        }
    }

    fn update_engine_feedback(&mut self, controls: Controls, dt: f32) {
        let throttle = controls.throttle.clamp(0.0, 1.0);
        self.sound_shift_timer = (self.sound_shift_timer - dt).max(0.0);
        let blip = self.rev_match_timer / REV_MATCH_SECONDS;
        if self.gear == NEUTRAL_GEAR {
            self.update_neutral_engine(throttle, dt);
            self.launch_energy = self.flywheel_energy();
        } else {
            self.neutral_fuel_cut = false;
            let target_rpm = (self.target_engine_rpm() + self.wheel_spin * throttle * 1800.0)
                .max(IDLE_RPM + throttle * (2200.0 - IDLE_RPM))
                .min(REDLINE_RPM);
            let response = if blip > 0.0 {
                22.0
            } else if self.sound_shift_timer > 0.0 {
                3.2
            } else {
                7.5
            };
            self.engine_rpm += (target_rpm - self.engine_rpm) * (1.0 - (-dt * response).exp());
        }
        // An unloaded engine uses less sustained load; fuel cuts also reach
        // the audio layers so the limiter is audible as well as visible.
        let requested = if self.gear == NEUTRAL_GEAR {
            if self.neutral_fuel_cut {
                0.0
            } else {
                throttle * 0.55
            }
        } else {
            throttle
        };
        let shift_progress = 1.0 - self.sound_shift_timer / SOUND_SHIFT_SECONDS;
        let shift_unload = 0.15 + 0.85 * shift_progress * shift_progress;
        // Feed the blip into both recorded load/coast layers and procedural
        // engine voices, without applying extra driving force at the wheels.
        let target_load = (requested * shift_unload + 0.12 * (1.0 - requested))
            .max(0.85 * blip)
            .clamp(0.0, 1.0);
        self.engine_load += (target_load - self.engine_load) * (dt * 10.0).clamp(0.0, 1.0);
    }

    fn update_steering(&mut self, requested: f32, dt: f32) {
        let requested = requested.clamp(-1.0, 1.0);
        if requested == 0.0 {
            // Stationary wheels hold their angle. Forward motion progressively
            // restores self-centering, independently of deliberate countersteering.
            let motion = (self.speed / STEERING_CENTERING_SPEED).clamp(0.0, 1.0);
            let step = STEERING_RATE * 0.85 * motion * dt;
            self.steering -= self.steering.clamp(-step, step);
            return;
        }
        // Cross the centre quickly when countersteering, then build the new
        // turn progressively. Small digital taps never request instant lock.
        let reversing = requested * self.steering < 0.0;
        let centering = reversing || requested.abs() < self.steering.abs();
        let target = if reversing { 0.0 } else { requested };
        let rate = if centering {
            STEERING_RATE * 0.85
        } else {
            STEERING_RATE * 0.45 / (1.0 + self.speed.abs() * 0.008)
        };
        self.steering += (target - self.steering).clamp(-rate * dt, rate * dt);
    }

    fn steering_angle(&self) -> f32 {
        self.steering.signum() * self.steering.abs().powf(1.10) * 0.54
    }

    pub fn front_wheel_angle(&self) -> f32 {
        -self.steering_angle()
    }

    fn resolve_road(&mut self) -> f32 {
        if self.track.has_compiled_roads() {
            let city = &self.track;
            return city.constrain(&mut self.position, &mut self.velocity);
        }
        let bounds = self.track.terrain_bounds();
        let mut impact = 0.0_f32;
        for (axis, (coordinate, min, max)) in [
            (0, bounds.min.x, bounds.max.x),
            (2, bounds.min.z, bounds.max.z),
        ]
        .into_iter()
        .enumerate()
        {
            let limit = [min + track::CAR_RADIUS, max - track::CAR_RADIUS];
            let clamped = self
                .position
                .component(coordinate)
                .clamp(limit[0], limit[1]);
            let excess = self.position.component(coordinate) - clamped;
            if excess.abs() > 0.0001 {
                *self.position.component_mut(coordinate) = clamped;
                let mut normal = XZ { x: 0.0, z: 0.0 };
                *normal.component_mut(axis) = excess.signum();
                impact = impact.max(crate::vehicle_location::deflect(&mut self.velocity, normal));
            }
        }
        self.position.y = self.track.drive_height(XZ {
            x: self.position.x,
            z: self.position.z,
        }) + crate::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
        impact
    }

    fn refresh_speed(&mut self) {
        self.speed = dot(
            self.velocity,
            XZ {
                x: self.heading.sin(),
                z: self.heading.cos(),
            },
        );
        self.slip = dot(
            self.velocity,
            XZ {
                x: self.heading.cos(),
                z: -self.heading.sin(),
            },
        );
        if self.manual {
            return;
        }
        let previous = self.gear;
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
        if self.gear != previous {
            self.begin_rev_match(previous);
        }
    }

    #[cfg(test)]
    fn collide(&mut self, other: &mut Vehicle) -> f32 {
        debug_assert!(
            std::sync::Arc::ptr_eq(&self.track, &other.track),
            "vehicle contact requires the same route"
        );
        let dx = self.position.x - other.position.x;
        let dz = self.position.z - other.position.z;
        let distance = dx.hypot(dz);
        let separation = track::CAR_RADIUS * 2.0;
        if distance >= separation {
            return 0.0;
        }
        let normal = if distance < 0.001 {
            XZ { x: 1.0, z: 0.0 }
        } else {
            XZ {
                x: dx / distance,
                z: dz / distance,
            }
        };
        let inverse_mass = 1.0 / self.model.spec().mass;
        let other_inverse_mass = 1.0 / other.model.spec().mass;
        let inverse_sum = inverse_mass + other_inverse_mass;
        let penetration = separation - distance;
        let relative = dot(
            XZ {
                x: self.velocity.x - other.velocity.x,
                z: self.velocity.z - other.velocity.z,
            },
            (normal).into(),
        );
        for (axis, component) in [normal.x, normal.z].iter().enumerate() {
            let position_axis = if axis == 0 { 0 } else { 2 };
            *self.position.component_mut(position_axis) +=
                component * penetration * inverse_mass / inverse_sum;
            *other.position.component_mut(position_axis) -=
                component * penetration * other_inverse_mass / inverse_sum;
            if relative < 0.0 {
                let impulse = -relative * 1.1 / inverse_sum;
                *self.velocity.component_mut(axis) += component * impulse * inverse_mass;
                *other.velocity.component_mut(axis) -= component * impulse * other_inverse_mass;
            }
        }
        let heights = [self.position.y, other.position.y];
        let wall_impact = self.resolve_road().max(other.resolve_road());
        if self.suspension_initialized {
            self.position.y = heights[0];
        }
        if other.suspension_initialized {
            other.position.y = heights[1];
        }
        self.refresh_speed();
        other.refresh_speed();
        (-relative).max(wall_impact).max(0.0)
    }
}

fn dot(a: XZ<f32>, b: XZ<f32>) -> f32 {
    a.x * b.x + a.z * b.z
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hill_car(uphill: bool) -> Vehicle {
        let route = crate::test_support::scenic_id().try_track().unwrap();
        let road = (0..200)
            .map(|i| route.at_distance(i as f32 * 10.0))
            .max_by(|a, b| {
                dot(route.ground_gradient(a.point), a.tangent)
                    .total_cmp(&dot(route.ground_gradient(b.point), b.tangent))
            })
            .unwrap();
        assert!(dot(route.ground_gradient(road.point), road.tangent) > 0.04);
        Vehicle::on_track(
            road.vehicle_position(0.0),
            road.heading() + if uphill { 0.0 } else { std::f32::consts::PI },
            crate::cars::CarModel::TestFixture(0),
            route,
        )
    }

    #[test]
    fn manual_shifts_hold_gears_and_pass_through_neutral() {
        let mut car = Vehicle::new(
            crate::geometry::XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            crate::cars::CarModel::TestFixture(0),
        );
        car.manual = true;
        car.velocity = crate::geometry::XZ { x: 0.0, z: 40.0 };
        car.refresh_speed();
        assert_eq!(car.gear, 1);
        car.shift(true);
        car.refresh_speed();
        assert_eq!(car.gear, 2);
        car.velocity = crate::geometry::XZ { x: 0.0, z: 0.0 };
        car.refresh_speed();
        assert_eq!(car.gear, 2);
        car.shift(false);
        car.shift(false);
        assert_eq!(car.gear_label(), "N");
        assert_eq!(car.drive_force(0.0), 0.0);
        assert_eq!(car.engine_brake_force(10.0), 0.0);
        car.velocity = crate::geometry::XZ { x: 2.0, z: 0.0 };
        car.shift(false);
        assert_eq!(car.gear_label(), "R");
        assert_eq!(car.velocity, crate::geometry::XZ { x: 2.0, z: 0.0 });
        car.shift(true);
        assert_eq!(car.gear_label(), "N");
        car.shift(true);
        assert_eq!(car.gear_label(), "1");
        for _ in 0..20 {
            car.shift(true);
        }
        assert_eq!(car.gear, car.model.spec().gearbox.gears());
        car.reset(
            crate::geometry::XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
        );
        assert!(car.manual);
        assert_eq!(car.gear, 1);
    }

    #[test]
    fn rolling_direction_changes_keep_momentum_then_reverse_under_power() {
        for model in crate::cars::ALL {
            for initial_speed in [12.0, -4.0] {
                let mut car = level_route_car(crate::test_support::scenic_id(), model);
                car.manual = true;
                if initial_speed < 0.0 {
                    car.shift(false);
                    car.shift(false);
                }
                car.velocity = XZ {
                    x: car.heading.sin() * initial_speed,
                    z: car.heading.cos() * initial_speed,
                };
                car.refresh_speed();
                let velocity = car.velocity;
                let heading = car.heading;
                // Both directions pass through neutral while still moving.
                car.shift(initial_speed < 0.0);
                assert_eq!(car.gear, NEUTRAL_GEAR);
                assert_eq!(car.velocity, velocity);
                car.shift(initial_speed < 0.0);
                assert_eq!(car.gear, if initial_speed > 0.0 { 0 } else { 1 });
                assert_eq!(car.velocity, velocity);
                assert_eq!(car.heading, heading);
                assert!(car.drive_force(initial_speed) > 0.0);
                for _ in 0..240 * 8 {
                    treadmill_tick(
                        &mut car,
                        Controls {
                            throttle: 1.0,
                            ..Controls::default()
                        },
                    );
                }
                assert!(
                    car.speed * initial_speed < 0.0,
                    "{model:?} must pull away in the selected direction: {} -> {}",
                    initial_speed,
                    car.speed
                );
            }
        }
    }

    #[test]
    fn tires_regain_direction_after_lifting_out_of_a_slide() {
        for model in crate::cars::ALL {
            let mut car = level_route_car(crate::test_support::scenic_id(), model);
            let h = car.heading;
            car.velocity = crate::geometry::XZ {
                x: h.sin() * 15.0 + h.cos() * 6.0,
                z: h.cos() * 15.0 - h.sin() * 6.0,
            };
            car.yaw_rate = 0.5;
            for _ in 0..480 {
                treadmill_tick(&mut car, Controls::default());
            }
            let rear_side =
                car.slip - car.yaw_rate * model.spec().wheelbase * model.spec().front_weight;
            let angle = rear_side.abs().atan2(car.speed.abs().max(2.5)).to_degrees();
            assert!(angle < 3.0, "{model:?}: slide persists after lift: {angle}");
            assert!(
                car.yaw_rate.abs() < 0.15,
                "{model:?}: rotation persists after lift"
            );
            assert!(
                car.speed > 8.0,
                "recovering grip must not simply stop the car"
            );
        }
    }

    #[test]
    fn revving_in_neutral_then_engaging_a_gear_spins_the_driven_wheels() {
        for (model, reverse) in crate::cars::ALL
            .into_iter()
            .flat_map(|m| [(m, false), (m, true)])
        {
            let mut car = level_route_car(crate::test_support::scenic_id(), model);
            car.manual = true;
            car.shift(false);
            assert_eq!(car.gear, NEUTRAL_GEAR);
            let throttle = Controls {
                throttle: 1.0,
                ..Controls::default()
            };
            for _ in 0..180 {
                treadmill_tick(&mut car, throttle);
            }
            assert!(
                car.speed.abs() < 0.02,
                "{model:?}: neutral must not move the car"
            );
            assert!(
                car.engine_rpm > 3500.0,
                "{model:?}: neutral must allow free revving"
            );
            car.shift(!reverse);
            assert_eq!(car.gear, if reverse { 0 } else { 1 });
            let energy = car.launch_energy;
            let mut peak_spin = 0.0_f32;
            for _ in 0..240 {
                treadmill_tick(&mut car, throttle);
                peak_spin = peak_spin.max(car.wheel_spin);
            }
            assert!(
                peak_spin > 0.15,
                "{model:?}: neutral launch must break traction"
            );
            assert!(car.speed.abs() > 1.0);
            assert!(
                car.launch_energy < energy * 0.2,
                "flywheel boost must be spent"
            );
            for _ in 0..480 {
                treadmill_tick(&mut car, Controls::default());
            }
            assert!(car.wheel_spin < 0.01);
            car.reset(car.position, car.heading);
            assert_eq!(car.launch_energy, 0.0);
        }
    }

    #[test]
    fn driven_tires_share_cornering_grip_and_yaw_has_inertia() {
        let run = |throttle| {
            let mut car = level_route_car(
                crate::test_support::scenic_id(),
                crate::cars::CarModel::TestFixture(0),
            );
            let h = car.heading;
            car.velocity = crate::geometry::XZ {
                x: h.sin() * 10.0 + h.cos() * 2.0,
                z: h.cos() * 10.0 - h.sin() * 2.0,
            };
            car.update(
                Duration::from_secs_f64(1.0 / 240.0),
                Controls {
                    throttle,
                    ..Controls::default()
                },
            );
            dot(
                car.velocity,
                crate::geometry::XZ {
                    x: h.cos(),
                    z: -h.sin(),
                },
            )
        };
        assert!(
            run(1.0) > run(0.0),
            "drive torque must reduce available cornering force"
        );
        let mut car = level_route_car(
            crate::test_support::scenic_id(),
            crate::cars::CarModel::TestFixture(0),
        );
        car.velocity = crate::geometry::XZ {
            x: car.heading.sin() * 20.0,
            z: car.heading.cos() * 20.0,
        };
        car.yaw_rate = 0.5;
        car.update(Duration::from_secs_f64(1.0 / 240.0), Controls::default());
        assert!(
            car.yaw_rate > 0.0 && car.yaw_rate < 0.5,
            "tire torque should slow existing rotation without deleting it"
        );
    }

    #[test]
    fn bump_moves_sprung_body_then_settles_at_different_frame_rates() {
        let run = |hz: u32| {
            let track = std::sync::Arc::new(crate::test_support::bumpy_track(
                &crate::test_support::scenic_id().try_track().unwrap(),
            ));
            let road = track.at_distance(50.0);
            let mut car = Vehicle::on_track(
                road.vehicle_position(0.0),
                road.heading(),
                crate::cars::CarModel::TestFixture(0),
                track,
            );
            car.velocity = road.tangent.map(|v| v * 8.0);
            let mut peak_pitch: f32 = 0.0;
            let mut peak_heave: f32 = 0.0;
            for _ in 0..hz * 3 {
                car.update(
                    Duration::from_secs_f64(1.0 / hz as f64),
                    Controls::default(),
                );
                peak_pitch = peak_pitch.max(car.suspension_pitch.abs());
                peak_heave = peak_heave.max((car.position.y - 0.48).abs());
            }
            println!(
                "{hz} Hz: bump pitch {peak_pitch}, heave {peak_heave}, final {}",
                car.position.y
            );
            assert!(peak_pitch > 0.01 && peak_pitch < 0.15);
            assert!(peak_heave > 0.025 && peak_heave < 0.25);
            assert!((car.position.y - car.ride_height()).abs() < 0.003);
            assert!(car.suspension_pitch.abs() < 0.002);
            assert!(car.suspension_roll.abs() < 0.002);
            (peak_pitch, peak_heave)
        };
        let reference = run(240);
        for hz in [30, 60, 144] {
            let sample = run(hz);
            assert!((sample.0 - reference.0).abs() < 0.005);
            assert!((sample.1 - reference.1).abs() < 0.01);
        }
    }

    #[test]
    fn chassis_can_unload_land_and_settle() {
        let mut car = level_route_car(
            crate::test_support::scenic_id(),
            crate::cars::CarModel::TestFixture(0),
        );
        let ground = car.ride_height();
        car.suspension_initialized = true;
        car.position.y = ground + 0.5;
        car.update(Duration::from_millis(20), Controls::default());
        assert_eq!(car.axle_load, [0.0; 2]);
        assert!(car.position.y > ground + 0.49);
        assert!(car.vertical_speed < 0.0);
        for _ in 0..240 * 3 {
            car.update(Duration::from_secs_f64(1.0 / 240.0), Controls::default());
        }
        assert!((car.position.y - ground).abs() < 0.002);
        assert!(car.vertical_speed.abs() < 0.002);
        assert!((car.axle_load.iter().sum::<f32>() / car.model.spec().mass - 9.81).abs() < 0.02);
    }

    #[test]
    fn elevation_changes_coasting_and_power_in_both_directions() {
        // Isolate gravity below tire saturation; a loaded uphill suspension
        // can temporarily provide more launch traction at full throttle.
        for throttle in [0.0, 0.35] {
            for reverse in [false, true] {
                let run = |uphill| {
                    let mut car = hill_car(uphill);
                    if reverse {
                        car.toggle_direction();
                        car.heading += std::f32::consts::PI;
                    }
                    let sign = if reverse { -1.0 } else { 1.0 };
                    car.velocity = crate::geometry::XZ {
                        x: car.heading.sin() * 8.0 * sign,
                        z: car.heading.cos() * 8.0 * sign,
                    };
                    car.update(
                        Duration::from_millis(250),
                        Controls {
                            throttle,
                            ..Controls::default()
                        },
                    );
                    car.speed * sign
                };
                let down = run(false);
                let up = run(true);
                assert!(
                    down > up + 0.15,
                    "throttle {throttle}, reverse {reverse}: downhill {down}, uphill {up}"
                );
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
        let start = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(5.0);
        Vehicle::new(
            start.vehicle_position(0.0),
            start.heading(),
            crate::cars::CarModel::TestFixture(0),
        )
    }

    fn route_car(id: track::LocationId, model: CarModel) -> Vehicle {
        let track = id.try_track().unwrap();
        let road = track.at_distance(50.0);
        Vehicle::on_track(road.vehicle_position(0.0), road.heading(), model, track)
    }

    // Compare car behavior on identical level ground across locations.
    fn level_route_car(id: track::LocationId, model: CarModel) -> Vehicle {
        let track = std::sync::Arc::new(crate::test_support::flat_track(&id.try_track().unwrap()));
        let road = track.at_distance(50.0);
        Vehicle::on_track(road.vehicle_position(0.0), road.heading(), model, track)
    }

    // Reset only position each tick: velocity/drivetrain evolve normally, but
    // bends and barriers cannot masquerade as acceleration or braking limits.
    fn treadmill_tick(car: &mut Vehicle, controls: Controls) -> f32 {
        let road = car.track.at_distance(50.0);
        car.position = road.vehicle_position(0.0);
        let previous = car.position;
        car.update(Duration::from_secs_f64(1.0 / 240.0), controls);
        dot(
            crate::geometry::XZ {
                x: car.position.x - previous.x,
                z: car.position.z - previous.z,
            },
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
            car.position = road.vehicle_position(0.0);
            let lateral = dot(
                car.velocity,
                crate::geometry::XZ {
                    x: car.heading.cos(),
                    z: -car.heading.sin(),
                },
            );
            car.velocity = crate::geometry::XZ {
                x: car.heading.sin() * speed + car.heading.cos() * lateral,
                z: car.heading.cos() * speed - car.heading.sin() * lateral,
            };
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
    fn parking_speed_retains_steering_lock_and_follows_the_front_wheels() {
        for model in crate::cars::ALL {
            for speed in [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0] {
                let mut car = level_route_car(crate::test_support::scenic_id(), model);
                for steering in [-1.0, 1.0] {
                    steering_run(&mut car, speed, steering, 5.0, 240.0);
                    let angle = car.steering_angle().abs();
                    let expected_yaw = speed.abs() * angle.tan() / model.spec().wheelbase;
                    println!(
                        "{model:?} at {speed}: lock {angle}, yaw {} expected {expected_yaw}",
                        car.yaw_rate.abs()
                    );
                    assert!(
                        (angle - 0.54).abs() < 0.001,
                        "{model:?}: full input must retain full steering lock"
                    );
                    assert!(
                        car.yaw_rate.abs() > expected_yaw * 0.75,
                        "{model:?}: low-speed turn pushes wide of the front wheels"
                    );
                }
            }
        }
    }

    #[test]
    fn adding_steering_always_increases_wheel_angle_at_road_speeds() {
        for model in crate::cars::ALL {
            let mut car = level_route_car(track::LocationId::Compact, model);
            for speed_kmh in [0.0, 40.0, 80.0, 120.0, 200.0] {
                car.speed = speed_kmh / 3.6;
                for direction in [-1.0, 1.0] {
                    let mut previous = 0.0;
                    for step in 1..=100 {
                        car.steering = direction * step as f32 / 100.0;
                        let angle = car.front_wheel_angle().abs();
                        assert!(
                            angle > previous,
                            "{model:?} at {speed_kmh} km/h: more input reduced wheel angle"
                        );
                        previous = angle;
                    }
                    assert!((previous - 0.54).abs() < 0.001);
                }
            }
        }
    }

    #[test]
    fn keyboard_taps_turn_less_than_held_input_and_release_steering() {
        let model = crate::cars::CarModel::TestFixture(0);
        for speed in [5.0, 15.0, 25.0, 45.0, 55.0] {
            let mut car = level_route_car(track::LocationId::Compact, model);
            let heading = car.heading;
            steering_run(&mut car, speed, 1.0, 0.1, 120.0);
            let tap_yaw = car.yaw_rate.abs();
            steering_run(&mut car, speed, 0.0, 0.4, 120.0);
            let tap = crate::test_support::angle(car.heading - heading).abs();
            assert!(tap > 0.0, "a steering tap must turn the car");
            assert_eq!(car.steering, 0.0);
            assert!(
                car.yaw_rate.abs() < tap_yaw,
                "releasing steering must reduce yaw"
            );
            let mut held = level_route_car(track::LocationId::Compact, model);
            steering_run(&mut held, speed, 1.0, 1.0, 120.0);
            let turn = crate::test_support::angle(held.heading - heading).abs();
            assert!(
                turn > tap * 3.0,
                "holding must produce a stronger turn than tapping"
            );
        }
    }

    #[test]
    fn tire_torque_turns_and_countersteering_reverses_rotation() {
        for model in crate::cars::ALL {
            let mut car = route_car(crate::test_support::scenic_id(), model);
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
        let model = crate::cars::CarModel::TestFixture(0);
        let mut reference = level_route_car(track::LocationId::Compact, model);
        let start = reference.heading;
        for (input, seconds) in [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)] {
            steering_run(&mut reference, 25.0, input, seconds, 240.0);
        }
        for hz in [30.0, 60.0, 144.0] {
            let mut car = level_route_car(track::LocationId::Compact, model);
            for (input, seconds) in [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)] {
                steering_run(&mut car, 25.0, input, seconds, hz);
            }
            let error = crate::test_support::angle(car.heading - reference.heading)
                .abs()
                .to_degrees();
            assert!(
                error < 0.25,
                "{model:?} {hz} Hz differs by {error} degrees from 240 Hz; start {start}"
            );
        }
    }

    #[test]
    fn released_steering_centres_faster_with_forward_speed() {
        let mut car = Vehicle::new(
            crate::geometry::XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            crate::cars::CarModel::TestFixture(0),
        );
        for direction in [-1.0, 1.0] {
            let mut previous = 1.0;
            for speed in [0.5, 2.0, 5.0, 10.0] {
                car.speed = speed;
                let mut reference: Option<f32> = None;
                for hz in [30, 60, 240] {
                    car.steering = direction;
                    for _ in 0..hz / 10 {
                        car.update_steering(0.0, 1.0 / hz as f32);
                    }
                    let remaining = car.steering.abs();
                    if let Some(expected) = reference {
                        assert!((remaining - expected).abs() < 0.00001);
                    } else {
                        assert!(remaining < previous);
                        previous = remaining;
                        reference = Some(remaining);
                    }
                }
            }
            car.speed = -5.0;
            car.steering = direction;
            car.update_steering(0.0, 1.0);
            assert_eq!(car.steering, direction);
            car.speed = 10.0;
            car.update_steering(0.0, 1.0);
            assert_eq!(car.steering, 0.0);
        }
    }

    #[test]
    fn ratio_shifts_drop_rpm_and_do_not_hunt_around_the_upshift_speed() {
        for model in crate::cars::ALL {
            let mut car = route_car(crate::test_support::scenic_id(), model);
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
            car.velocity = crate::geometry::XZ { x: 0.0, z: 0.0 };
            car.refresh_speed();
            assert_eq!(car.gear, 1);
            assert_eq!(car.target_engine_rpm(), IDLE_RPM);
        }
    }

    #[test]
    fn city_curb_is_crossable_with_a_single_small_speed_loss_and_settling_bump() {
        let track = crate::test_support::nicosia_world();
        let city = &track;
        let (q, normal) = city
            .curbs()
            .iter()
            .find_map(|curb| {
                let q = crate::geometry::XZ {
                    x: (curb.ends[0].x + curb.ends[1].x) * 0.5,
                    z: (curb.ends[0].z + curb.ends[1].z) * 0.5,
                };
                let inside = crate::geometry::XZ {
                    x: q.x + curb.inward.x * 0.08,
                    z: q.z + curb.inward.z * 0.08,
                };
                let outside = crate::geometry::XZ {
                    x: q.x - curb.inward.x * 0.2,
                    z: q.z - curb.inward.z * 0.2,
                };
                if !city.contains_road(inside)
                    || city.contains_road(outside)
                    || city.paving_height(outside) != Some(city.pavement_height_meters())
                {
                    return None;
                }
                for distance in [0.08, -0.2, -1.0, -2.0, -3.0] {
                    let mut p = crate::geometry::XYZ {
                        x: q.x + curb.inward.x * distance,
                        y: 0.0,
                        z: q.z + curb.inward.z * distance,
                    };
                    let before = p;
                    city.constrain(&mut p, &mut crate::geometry::XZ::default());
                    if (p.x - before.x).hypot(p.z - before.z) > 0.001 {
                        return None;
                    }
                }
                Some((q, curb.inward))
            })
            .expect("missing clear curb crossing fixture");
        let position = crate::geometry::XYZ {
            x: q.x + normal.x * 0.08,
            y: 0.0,
            z: q.z + normal.z * 0.08,
        };
        let heading = (-normal.x).atan2(-normal.z);
        let mut car = Vehicle::on_track(
            position,
            heading,
            crate::cars::CarModel::TestFixture(0),
            track.clone(),
        );
        car.velocity = normal.map(|n| -n * 8.0);
        car.update(Duration::from_millis(24), Controls::default());
        assert!(!city.contains_road(crate::geometry::XZ {
            x: car.position.x,
            z: car.position.z
        }));
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
        car.gear = NEUTRAL_GEAR; // isolate curb drag from intentional engine braking
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
    fn reverse_acceleration_terminal_speed_and_braking_are_route_independent() {
        for model in crate::cars::ALL {
            let mut compact = level_route_car(track::LocationId::Compact, model);
            let mut scenic = level_route_car(crate::test_support::scenic_id(), model);
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
                assert!(
                    (compact.speed - scenic.speed).abs() < 0.01,
                    "{model:?}: {} vs {}, slip {} vs {}",
                    compact.speed,
                    scenic.speed,
                    compact.slip,
                    scenic.slip
                );
            }
            assert!(scenic.speed < -6.0 && scenic.speed.is_finite());
            let (mut compact_distance, mut scenic_distance) = (0.0, 0.0);
            for _ in 0..720 {
                let brake = Controls {
                    brake: 1.0,
                    ..Controls::default()
                };
                compact_distance += treadmill_tick(&mut compact, brake);
                scenic_distance += treadmill_tick(&mut scenic, brake);
                assert!(
                    (compact.speed - scenic.speed).abs() < 0.01,
                    "{model:?}: {} vs {}, slip {} vs {}",
                    compact.speed,
                    scenic.speed,
                    compact.slip,
                    scenic.slip
                );
            }
            assert!(compact.speed.abs() < 0.001 && scenic.speed.abs() < 0.001);
            assert!((compact_distance - scenic_distance).abs() < 0.01);
            assert_eq!(scenic.gear, 0);
        }
    }

    #[test]
    fn offroad_driving_and_reentry_preserve_position_and_follow_terrain() {
        let track = crate::test_support::scenic_id().try_track().unwrap();
        let road = track.at_distance(50.0);
        for side in [-1.0, 1.0] {
            let position = road.vehicle_position(side * (road.width * 0.5 + 8.0));
            let inward = road.normal().map(|v| -side * v);
            let mut car = Vehicle::on_track(
                position,
                inward.x.atan2(inward.z),
                crate::cars::CarModel::TestFixture(0),
                track.clone(),
            );
            car.manual = true;
            car.gear = NEUTRAL_GEAR;
            car.velocity = inward.map(|v| v * 10.0);
            assert_eq!(car.resolve_road(), 0.0);
            assert_eq!([car.position.x, car.position.z], [position.x, position.z]);
            for _ in 0..288 {
                assert!(
                    car.update(Duration::from_secs_f64(1.0 / 240.0), Controls::default()) < 0.01
                );
                assert!(car.position.iter().all(|v| v.is_finite()));
            }
            assert!(track.project(car.position).offset.abs() < road.width * 0.5);
            assert!((car.position.y - car.ride_height()).abs() < 0.15);
        }
    }

    #[test]
    fn scenic_recovery_preserves_route_and_model_but_clears_motion_and_reverse() {
        let mut car = route_car(
            crate::test_support::scenic_id(),
            crate::cars::CarModel::TestFixture(0),
        );
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
        car.reset(road.vehicle_position(0.0), road.heading());
        assert!(std::sync::Arc::ptr_eq(
            &car.track,
            &crate::test_support::scenic_id().try_track().unwrap()
        ));
        assert_eq!(car.model, crate::cars::CarModel::TestFixture(0));
        assert_eq!(car.position, road.vehicle_position(0.0));
        assert_eq!(car.heading, road.heading());
        assert_eq!(car.velocity, crate::geometry::XZ { x: 0.0, z: 0.0 });
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
    fn outer_terrain_boundary_preserves_tangential_motion_and_ground_height() {
        let track = track::LocationId::Compact.try_track().unwrap();
        let bounds = track.terrain_bounds();
        let mut car = Vehicle::on_track(
            crate::geometry::XYZ {
                x: bounds.min.x - 1.0,
                y: 0.0,
                z: (bounds.min.z + bounds.max.z) * 0.5,
            },
            0.0,
            crate::cars::CarModel::TestFixture(0),
            track.clone(),
        );
        car.velocity = crate::geometry::XZ { x: -4.0, z: 8.0 };
        assert!(car.resolve_road() > 0.0);
        assert_eq!(car.position.x, bounds.min.x + track::CAR_RADIUS);
        assert!(car.velocity.z > 6.0);
        assert!(
            (car.position.y
                - track.drive_height(crate::geometry::XZ {
                    x: car.position.x,
                    z: car.position.z
                })
                - 0.48)
                .abs()
                < 0.001
        );
    }

    #[test]
    fn unequal_mass_contact_conserves_momentum_and_moves_light_car_more() {
        let sample = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(50.0);
        let mut heavy_spec = crate::test_support::car_spec().clone();
        heavy_spec.mass = 1800.0;
        let mut heavy = Vehicle::new(
            sample.vehicle_position(0.8),
            sample.heading(),
            crate::test_support::car_model_with_spec(heavy_spec),
        );
        let mut light = Vehicle::new(
            sample.vehicle_position(-0.8),
            sample.heading(),
            crate::cars::CarModel::TestFixture(0),
        );
        let n = sample.normal();
        heavy.velocity = n.map(|x| -x * 3.0);
        let momentum = |a: &Vehicle, b: &Vehicle| {
            std::array::from_fn::<_, 2, _>(|i| {
                a.velocity.component(i) * a.model.spec().mass
                    + b.velocity.component(i) * b.model.spec().mass
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
                .chain([car.velocity.x, car.velocity.z].iter())
                .all(|v| v.is_finite()));
        }
        let after = momentum(&heavy, &light);
        for i in 0..2 {
            assert!((before[i] - after[i]).abs() < 0.01);
        }
        let moved = |a: crate::geometry::XYZ, b: crate::geometry::XYZ| (a.x - b.x).hypot(a.z - b.z);
        assert!(moved(light_before, light.position) > moved(heavy_before, heavy.position));
    }

    #[test]
    fn holding_brakes_at_rest_does_not_dive_and_settles_after_stopping() {
        for handbrake in [false, true] {
            for initial_speed in [0.0, 8.0, -8.0] {
                let mut v = car();
                let forward = crate::geometry::XZ {
                    x: v.heading.sin(),
                    z: v.heading.cos(),
                };
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
    fn neutral_revs_build_progressively_cycle_limiter_and_fall_to_idle() {
        let road = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(50.0);
        for model in crate::cars::ALL {
            let mut car = Vehicle::new(road.vehicle_position(0.0), road.heading(), model);
            car.manual = true;
            car.shift(false);
            let dt = Duration::from_secs_f64(1.0 / 60.0);
            let pedal = Controls {
                throttle: 1.0,
                ..Controls::default()
            };
            for _ in 0..6 {
                car.update(dt, pedal);
            }
            assert!(car.engine_rpm > IDLE_RPM + 100.0);
            assert!(car.engine_rpm < 2500.0, "{model:?}: blip jumps too quickly");
            let mut cuts = 0;
            let mut recoveries = 0;
            for _ in 0..300 {
                let was_cut = car.neutral_fuel_cut;
                car.update(dt, pedal);
                cuts += usize::from(!was_cut && car.neutral_fuel_cut);
                recoveries += usize::from(was_cut && !car.neutral_fuel_cut);
                assert!((IDLE_RPM..=REDLINE_RPM).contains(&car.engine_rpm));
            }
            assert!(cuts > 2 && recoveries > 2, "{model:?}: limiter must cycle");
            car.update(dt, Controls::default());
            assert!(
                car.engine_rpm > 5500.0,
                "release must retain engine inertia"
            );
            for _ in 0..300 {
                let before = car.engine_rpm;
                car.update(dt, Controls::default());
                assert!(car.engine_rpm <= before);
            }
            assert_eq!(car.engine_rpm, IDLE_RPM);
            assert_eq!(car.gear, NEUTRAL_GEAR);
            assert!(car.speed.abs() < 0.01, "neutral must not propel the car");
            assert_eq!(car.drive_force(0.0), 0.0);
            assert_eq!(car.turbo_boost, 0.0);
        }
    }

    #[test]
    fn neutral_partial_throttle_balances_drag_independently_of_road_speed() {
        let road = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(50.0);
        for model in crate::cars::ALL {
            let mut car = Vehicle::new(road.vehicle_position(0.0), road.heading(), model);
            car.manual = true;
            car.gear = NEUTRAL_GEAR;
            let pedal = Controls {
                throttle: 0.25,
                ..Controls::default()
            };
            for _ in 0..600 {
                car.update_engine_feedback(pedal, 1.0 / 60.0);
            }
            let stationary = car.engine_rpm;
            assert!(stationary > 3000.0 && stationary < 4500.0);
            let mut rolling = Vehicle::new(road.vehicle_position(0.0), road.heading(), model);
            rolling.gear = NEUTRAL_GEAR;
            rolling.engine_rpm = stationary;
            rolling.speed = 30.0;
            for _ in 0..120 {
                car.update_engine_feedback(pedal, 1.0 / 60.0);
                rolling.update_engine_feedback(pedal, 1.0 / 60.0);
                assert_eq!(car.engine_rpm, rolling.engine_rpm);
            }
            car.gear = 1;
            car.neutral_fuel_cut = true;
            car.update_engine_feedback(pedal, 1.0 / 60.0);
            assert!(!car.neutral_fuel_cut, "engaging a gear clears the limiter");
            car.reset(road.vehicle_position(0.0), road.heading());
            assert!(!car.neutral_fuel_cut);
        }
    }

    #[test]
    fn neutral_blips_are_consistent_across_frame_rates() {
        let mut results = Vec::new();
        for hz in [30, 60, 144, 240] {
            let mut car = Vehicle::new(
                crate::geometry::XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                0.0,
                crate::cars::CarModel::TestFixture(0),
            );
            car.gear = NEUTRAL_GEAR;
            for _ in 0..hz / 2 {
                car.update_engine_feedback(
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                    1.0 / hz as f32,
                );
            }
            results.push(car.engine_rpm);
        }
        let min = results.iter().copied().fold(f32::INFINITY, f32::min);
        let max = results.iter().copied().fold(0.0, f32::max);
        assert!(
            max - min < 20.0,
            "frame rates changed blip response: {results:?}"
        );
    }

    #[test]
    fn engine_feedback_smooths_rpm_load_and_shift_unload() {
        let road = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(50.0);
        let mut car = Vehicle::new(
            road.vehicle_position(0.0),
            road.heading(),
            crate::cars::CarModel::TestFixture(0),
        );
        assert_eq!(car.engine_rpm, IDLE_RPM);
        assert_eq!(car.engine_load, 0.0);
        let mut previous_speed = 0.0;
        for _ in 0..180 {
            car.position = road.vehicle_position(0.0);
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
        // Let a longer audible shift finish before checking sustained engine load.
        car.manual = true;
        for _ in 0..30 {
            car.position = road.vehicle_position(0.0);
            car.update(
                Duration::from_secs_f64(1.0 / 60.0),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        assert!(car.engine_load > 0.45);
        car.manual = false;

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

        car.reset(road.vehicle_position(0.0), road.heading());
        assert!(car.toggle_direction());
        for _ in 0..60 {
            car.position = road.vehicle_position(0.0);
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
        let road = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(50.0);
        for model in crate::cars::ALL {
            let mut v = Vehicle::new(road.vehicle_position(0.0), road.heading(), model);
            assert!(v.toggle_direction());
            assert_eq!(v.gear_label(), "R");
            for _ in 0..720 {
                v.position = road.vehicle_position(0.0); // straight-line test treadmill
                v.update(
                    Duration::from_secs_f64(1.0 / 60.0),
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            assert!(
                v.speed < -6.0 && v.speed.is_finite(),
                "reverse speed for {model:?}: {}",
                v.speed
            );
            assert_eq!(v.gear, 0);
            assert!(!v.toggle_direction(), "moving car cannot select drive");
            v.update(
                Duration::from_secs(3),
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
        let mut car = Vehicle::new(
            crate::geometry::XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            model,
        );
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
        car.reset(
            crate::geometry::XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
        );
        assert_eq!(car.turbo_spool, 0.0);
        assert_eq!(car.turbo_boost, 0.0);
        car.gear = NEUTRAL_GEAR;
        car.update_turbo(5.0, 1.0, speed);
        assert_eq!(car.turbo_spool, 0.0);
    }
}
