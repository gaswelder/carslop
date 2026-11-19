//! Steering input response, self-centering, and front wheel angle.

use super::car;

// Base rate of normalized steering input change per second.
const STEERING_RATE: f32 = 9.0;
// Forward speed (m/s) at which released steering reaches its full return rate.
const STEERING_CENTERING_SPEED: f32 = 10.0;

#[derive(Default)]
pub struct SteeringState {
    input: f32,
}

pub fn update(vehicle: &mut car::Vehicle, requested: f32, dt: f32) {
    let requested = requested.clamp(-1.0, 1.0);
    if requested == 0.0 {
        // Stationary wheels hold their angle. Forward motion progressively
        // restores self-centering, independently of deliberate countersteering.
        let motion = (car::longitudinal_speed(vehicle) / STEERING_CENTERING_SPEED).clamp(0.0, 1.0);
        let step = STEERING_RATE * 0.45 * motion * dt;
        vehicle.steering.input -= vehicle.steering.input.clamp(-step, step);
        return;
    }
    // Cross the centre quickly when countersteering, then use the remaining
    // time to build the opposite angle so response is independent of step size.
    let mut remaining = dt;
    if requested * vehicle.steering.input < 0.0 {
        let rate = STEERING_RATE * 0.85;
        let centering_time = vehicle.steering.input.abs() / rate;
        let elapsed = remaining.min(centering_time);
        vehicle.steering.input -= vehicle
            .steering
            .input
            .clamp(-rate * elapsed, rate * elapsed);
        remaining -= elapsed;
    }
    let rate = STEERING_RATE * 0.45 / (1.0 + car::longitudinal_speed(vehicle).abs() * 0.008);
    vehicle.steering.input +=
        (requested - vehicle.steering.input).clamp(-rate * remaining, rate * remaining);
}

fn steering_angle(vehicle: &car::Vehicle) -> f32 {
    vehicle.steering.input.signum() * vehicle.steering.input.abs().powf(1.10) * 0.54
}

pub fn front_wheel_angle(vehicle: &car::Vehicle) -> f32 {
    -steering_angle(vehicle)
}

#[cfg(test)]
#[path = "tests/steering.rs"]
mod tests;
