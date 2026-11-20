// Maximum centering decay rate, in inverse seconds.
const CENTERING_RATE: f32 = 4.05;
const MAX_ANGLE: f32 = 0.54;

// Forward speed (m/s) at which the centering pull reaches full strength.
const STEERING_CENTERING_SPEED: f32 = 10.0;

#[derive(Default)]
pub struct State {
    /// Front-wheel angle in radians; positive turns toward the vehicle's right.
    pub angle: f32,
}

/// Advance the wheel angle using the driver's angular drive rate, in rad/s.
pub fn update(state: &mut State, driver_rate: f32, longitudinal_speed: f32, dt: f32) {
    let centering_rate =
        CENTERING_RATE * (longitudinal_speed / STEERING_CENTERING_SPEED).clamp(0.0, 1.0);
    // Driver effort and the speed-dependent centering pull act together:
    // angle' = driver_rate - angle * centering_rate.
    // Integrate exactly for constant input/speed, including the zero-rate limit.
    let decay = (-centering_rate * dt).exp();
    let response = if centering_rate > 0.0 {
        -(-centering_rate * dt).exp_m1() / centering_rate
    } else {
        dt
    };
    state.angle = (state.angle * decay + driver_rate * response).clamp(-MAX_ANGLE, MAX_ANGLE);
}

#[cfg(test)]
#[path = "tests/steering.rs"]
mod tests;
