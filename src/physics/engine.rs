//! Engine runtime state, spec-based output, gearing, and road load.

use super::car::IDLE_RPM;
use super::car::REVERSE_GEAR;
use crate::model;

/// Runtime engine simulation and feedback state.
pub struct EngineState {
    pub rpm: f32,
    /// Smoothed engine load used for sound, from zero to one.
    pub load: f32,
    /// Normalized turbine speed and available boost.
    pub turbo_spool: f32,
    // Normalized manifold boost.
    pub turbo_boost: f32,
    pub fuel_cut: bool,
    pub rev_match_timer: f32,
    pub sound_shift_timer: f32,
}

const AIR_DENSITY_KG_M3: f32 = 1.225;

// 90% drivetrain efficiency to calculate power at the wheels.
pub const DRIVETRAIN_EFFICIENCY: f32 = 0.90;

pub fn engine_torque(car_spec: &model::car::CarSpec, rpm: f32) -> f32 {
    (car_spec.engine_max_torque_nm * model::car::torque_factor(&car_spec.engine_torque_curve, rpm))
        .min(car_spec.engine_power_watts / (rpm.max(IDLE_RPM) * std::f32::consts::TAU / 60.0))
}

fn drive_force(car_spec: &model::car::CarSpec, speed: f32, gear: u8) -> f32 {
    let g = &car_spec.gearbox;
    let ratio = if gear == REVERSE_GEAR {
        g.reverse_ratio
    } else {
        g.ratios[(gear - 1) as usize]
    };
    let rpm = rpm(car_spec, speed, gear).max(IDLE_RPM);
    engine_torque(car_spec, rpm) * ratio * g.final_drive * DRIVETRAIN_EFFICIENCY
        / car_spec.rolling_radius
}

pub fn resistance_force(car_spec: &model::car::CarSpec, speed: f32) -> f32 {
    let drag_force = 0.5
        * AIR_DENSITY_KG_M3
        * car_spec.drag_coefficient
        * car_spec.frontal_area_m2
        * speed
        * speed;
    0.25 * car_spec.weight_kg + drag_force
}

/// Full-throttle equilibrium on level ground in the best forward gear.
/// Uses the same spec torque, redline and road load as driving.
pub fn calculate_top_speed(car_spec: &model::car::CarSpec) -> f32 {
    (1..=gears(&car_spec.gearbox))
        .map(|gear| {
            let mut low = 0.0;
            let mut high = speed_at_rpm(car_spec, car_spec.redline_rpm, gear);
            for _ in 0..48 {
                let speed = (low + high) * 0.5;
                if drive_force(car_spec, speed, gear) > resistance_force(car_spec, speed) {
                    low = speed;
                } else {
                    high = speed;
                }
            }
            low
        })
        .fold(0.0, f32::max)
}

pub fn gears(gearbox: &model::car::Gearbox) -> u8 {
    gearbox.ratios.len() as u8
}

pub fn rpm(car_spec: &model::car::CarSpec, speed: f32, gear: u8) -> f32 {
    let gearbox = &car_spec.gearbox;
    let ratio = if gear == REVERSE_GEAR {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[gear.saturating_sub(1).min(gears(gearbox) - 1) as usize]
    };
    let speed = if gear == REVERSE_GEAR {
        speed.abs()
    } else {
        speed.max(0.0)
    };
    speed * 60.0 / (std::f32::consts::TAU * car_spec.rolling_radius) * ratio * gearbox.final_drive
}

fn speed_at_rpm(car_spec: &model::car::CarSpec, rpm: f32, gear: u8) -> f32 {
    let gearbox = &car_spec.gearbox;
    let ratio = if gear == REVERSE_GEAR {
        gearbox.reverse_ratio
    } else {
        gearbox.ratios[gear.saturating_sub(1).min(gears(gearbox) - 1) as usize]
    };
    rpm * std::f32::consts::TAU * car_spec.rolling_radius / (60.0 * ratio * gearbox.final_drive)
}
