use std::fs;
use std::path::Path;

#[derive(serde::Deserialize)]
pub struct CarDefinition {
    #[serde(default = "default_paint_color")]
    pub paint_color: [f32; 3],
    pub weight_kg: f32,
    pub drag_coefficient: f32,
    pub center_of_mass: CenterOfMass,
    pub yaw_inertia_kg_m2: f32,
    pub engine: EngineDefinition,
    pub chassis: ChassisDefinition,
    pub gearbox: Gearbox,
}

#[derive(serde::Deserialize)]
pub struct EngineDefinition {
    pub power_kw: f32,
    pub max_torque_nm: f32,
    pub torque_curve: Vec<TorquePoint>,
    pub redline_rpm: f32,
    pub turbo: Option<Turbo>,
}

#[derive(serde::Deserialize)]
pub struct ChassisDefinition {
    pub brake_force_n: f32,
    pub drive_bias: f32,
    pub brake_bias: f32,
    pub suspension_stiffness_n_m: f32,
}

#[derive(serde::Deserialize)]
pub struct CenterOfMass {
    /// Position between axles: 0 = rear axle, 1 = front axle.
    pub longitudinal_fraction: f32,
    /// Height above ground as a fraction of the full model height, in 0..=1.
    pub height_fraction: f32,
}

#[derive(serde::Deserialize)]
pub struct TorquePoint {
    pub rpm: f32,
    /// Fraction of the engine's maximum torque.
    pub torque: f32,
}

#[derive(serde::Deserialize)]
pub struct Turbo {
    pub spool_start_rpm: f32,
    pub full_boost_rpm: f32,
    pub spool_seconds: f32,
    pub decay_seconds: f32,
}

#[derive(serde::Deserialize)]
pub struct Gearbox {
    pub ratios: Vec<f32>,
    pub final_drive: f32,
    pub reverse_ratio: f32,
}

/// Reads and validates a car's JSON definition.
pub fn load(path: &Path) -> Result<CarDefinition, String> {
    let definition = serde_json::from_slice(&fs::read(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    validate(&definition)?;
    Ok(definition)
}

fn validate(car_definition: &CarDefinition) -> Result<(), String> {
    for channel in car_definition.paint_color {
        fraction(channel)?;
    }
    for value in [
        car_definition.weight_kg,
        car_definition.chassis.brake_force_n,
        car_definition.engine.power_kw,
        car_definition.drag_coefficient,
        car_definition.yaw_inertia_kg_m2,
        car_definition.engine.max_torque_nm,
        car_definition.engine.redline_rpm,
        car_definition.chassis.suspension_stiffness_n_m,
        car_definition.gearbox.final_drive,
        car_definition.gearbox.reverse_ratio,
    ] {
        positive(value)?;
    }
    for value in [
        car_definition.center_of_mass.longitudinal_fraction,
        car_definition.center_of_mass.height_fraction,
        car_definition.chassis.drive_bias,
        car_definition.chassis.brake_bias,
    ] {
        fraction(value)?;
    }
    if car_definition.gearbox.ratios.is_empty() || car_definition.gearbox.ratios.len() > 254 {
        return Err("expected 1..254 ratios".into());
    }
    for &ratio in &car_definition.gearbox.ratios {
        positive(ratio)?;
    }
    if !car_definition
        .gearbox
        .ratios
        .windows(2)
        .all(|ratios| ratios[0] > ratios[1])
    {
        return Err("ratios must descend".into());
    }
    if car_definition.engine.torque_curve.len() < 2 {
        return Err("torque curve requires at least two points".into());
    }
    for point in &car_definition.engine.torque_curve {
        nonnegative(point.rpm)?;
        fraction(point.torque)?;
    }
    if !car_definition
        .engine
        .torque_curve
        .windows(2)
        .all(|points| points[0].rpm < points[1].rpm)
    {
        return Err("torque curve RPM must strictly increase".into());
    }
    if !car_definition
        .engine
        .torque_curve
        .iter()
        .any(|point| point.torque > 0.0)
    {
        return Err("torque curve requires positive torque".into());
    }
    if let Some(turbo) = &car_definition.engine.turbo {
        for value in [
            turbo.spool_start_rpm,
            turbo.full_boost_rpm,
            turbo.spool_seconds,
            turbo.decay_seconds,
        ] {
            positive(value)?;
        }
        if turbo.full_boost_rpm <= turbo.spool_start_rpm {
            return Err("full boost RPM must exceed spool start RPM".into());
        }
    }
    Ok(())
}

fn default_paint_color() -> [f32; 3] {
    [1.0, 0.18, 0.08]
}

fn positive(value: f32) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 {
        return Err("expected finite positive number".into());
    }
    Ok(())
}

fn nonnegative(value: f32) -> Result<(), String> {
    if !value.is_finite() || value < 0.0 {
        return Err("expected finite nonnegative number".into());
    }
    Ok(())
}

fn fraction(value: f32) -> Result<(), String> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err("expected fraction in 0..1".into());
    }
    Ok(())
}
