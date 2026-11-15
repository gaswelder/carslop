use crate::asset;
use crate::physics;
use crate::util::geometry::XYZ;
use std::fs;
use std::path::PathBuf;

pub const WATTS_PER_HP: f32 = 745.6999;

#[derive(Clone, Debug)]
pub struct CarSpec {
    pub name: String,
    pub paint_color: [f32; 3],
    pub weight_kg: f32,
    pub engine_max_torque_nm: f32,
    pub engine_torque_curve: Vec<TorquePoint>,
    pub redline_rpm: f32,
    pub gearbox: Gearbox,
    //
    pub design: std::sync::Arc<asset::car_glb::Design>,
    pub top_speed: f32,
    pub brake_force: f32,
    pub wheelbase: f32,
    pub audio: EngineVoice,
    pub rear_combined_slip: Option<[f32; 3]>, // baseline, reduction, slip angle
    pub front_weight: f32,
    /// Metres above ground, derived from the authored fraction and model height.
    pub cg_height: f32,
    pub track_width: f32,
    pub yaw_inertia: f32,
    /// Fraction of driving force sent to the front axle: 0 = RWD, 1 = FWD.
    pub drive_bias: f32,
    /// Fraction of foot-braking force applied to the front axle.
    pub brake_bias: f32,
    pub corner_stiffness: [f32; 2], // force per radian, front/rear axle

    pub suspension_hz: f32,

    pub engine_power_watts: f32,
    /// Dimensionless aerodynamic drag coefficient (Cd/Cx).
    pub drag_coefficient: f32,
    pub frontal_area_m2: f32,
    pub turbo: Option<Turbo>,
}

#[derive(Clone, Debug, serde::Deserialize)]
pub struct TorquePoint {
    pub rpm: f32,
    /// Fraction of the engine's maximum torque.
    pub torque: f32,
}

#[derive(Clone, Debug, serde::Deserialize)]
pub struct Turbo {
    pub spool_start_rpm: f32,
    pub full_boost_rpm: f32,
    pub spool_seconds: f32,
    pub decay_seconds: f32,
}

#[derive(Clone, Debug, serde::Deserialize)]
pub struct Gearbox {
    pub ratios: Vec<f32>,
    pub final_drive: f32,
    #[serde(skip)]
    pub rolling_radius: f32,
    pub reverse_ratio: f32,
    pub upshift_rpm: f32,
    pub downshift_rpm: f32,
}
/// Per-car engine timbre and mix, independent of model identity.
#[derive(Clone, Debug, serde::Deserialize)]
pub struct EngineVoice {
    pub harmonics: Vec<[f32; 2]>, // frequency multiplier, amplitude
    pub gain: f32,
    pub coast_gain: f32,
    pub soft_limit: Option<[f32; 2]>, // knee, headroom
    pub pulses_per_revolution: f32,
    #[serde(skip)]
    pub sample_directory: Option<PathBuf>,
}

// Loads cars from the given directory.
pub fn load(dirpath: &std::path::Path) -> Result<Vec<std::sync::Arc<CarSpec>>, String> {
    let mut cars = Vec::new();
    let entries = fs::read_dir(dirpath).map_err(|e| format!("{}: {e}", dirpath.display()))?;
    for entry in entries {
        let path = entry.map_err(|e| e.to_string())?.path();
        if !path.is_dir() {
            log::warn!("Skipping car entry {}: not a directory", path.display());
            continue;
        }
        let path = path.join("car.json");
        if !path.is_file() {
            log::warn!("Skipping car definition {}: not a file", path.display());
            continue;
        }
        match load_car(&path) {
            Ok(spec) => cars.push(std::sync::Arc::new(spec)),
            Err(error) => log::warn!("Cannot load car {}: {error}", path.display()),
        }
    }
    cars.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(cars)
}

#[derive(serde::Deserialize)]
struct CarDefinition {
    name: String,
    #[serde(default = "default_paint_color")]
    paint_color: [f32; 3],
    weight_kg: f32,
    brake_force: f32,
    engine_power_hp: f32,
    drag_coefficient: f32,
    front_weight: f32,
    cg_height_fraction: f32,
    yaw_inertia: f32,
    drive_bias: f32,
    brake_bias: f32,
    corner_stiffness: [f32; 2],
    engine_max_torque_nm: f32,
    engine_torque_curve: Vec<TorquePoint>,
    redline_rpm: f32,
    suspension_hz: f32,
    rear_combined_slip: Option<[f32; 3]>,
    gearbox: Gearbox,
    audio: EngineVoice,
    turbo: Option<Turbo>,
}

fn validate(car_definition: &CarDefinition) -> Result<(), String> {
    if car_definition.name.trim().is_empty() {
        return Err("expected nonempty text".into());
    }
    for channel in car_definition.paint_color {
        fraction(channel)?;
    }
    for value in [
        car_definition.weight_kg,
        car_definition.brake_force,
        car_definition.engine_power_hp,
        car_definition.drag_coefficient,
        car_definition.yaw_inertia,
        car_definition.engine_max_torque_nm,
        car_definition.redline_rpm,
        car_definition.suspension_hz,
        car_definition.corner_stiffness[0],
        car_definition.corner_stiffness[1],
        car_definition.gearbox.final_drive,
        car_definition.gearbox.reverse_ratio,
        car_definition.gearbox.upshift_rpm,
        car_definition.gearbox.downshift_rpm,
        car_definition.audio.gain,
        car_definition.audio.pulses_per_revolution,
    ] {
        positive(value)?;
    }
    for value in [
        car_definition.front_weight,
        car_definition.cg_height_fraction,
        car_definition.drive_bias,
        car_definition.brake_bias,
        car_definition.audio.coast_gain,
    ] {
        fraction(value)?;
    }
    if let Some(values) = car_definition.rear_combined_slip {
        fraction(values[0])?;
        fraction(values[1])?;
        positive(values[2])?;
        if values[1] > values[0] {
            return Err("combined slip reduction exceeds baseline".into());
        }
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
    if car_definition.gearbox.downshift_rpm >= car_definition.gearbox.upshift_rpm {
        return Err("invalid shift thresholds".into());
    }
    if car_definition.redline_rpm <= physics::car::IDLE_RPM + 250.0 {
        return Err("redline RPM must leave limiter recovery above idle".into());
    }
    if car_definition.gearbox.upshift_rpm >= car_definition.redline_rpm {
        return Err("upshift RPM must be below redline RPM".into());
    }
    if car_definition.engine_torque_curve.len() < 2 {
        return Err("torque curve requires at least two points".into());
    }
    for point in &car_definition.engine_torque_curve {
        nonnegative(point.rpm)?;
        fraction(point.torque)?;
    }
    if !car_definition
        .engine_torque_curve
        .windows(2)
        .all(|points| points[0].rpm < points[1].rpm)
    {
        return Err("torque curve RPM must strictly increase".into());
    }
    if !car_definition
        .engine_torque_curve
        .iter()
        .any(|point| point.torque > 0.0)
    {
        return Err("torque curve requires positive torque".into());
    }
    if car_definition.audio.harmonics.is_empty() {
        return Err("expected engine harmonics".into());
    }
    for &[frequency, amplitude] in &car_definition.audio.harmonics {
        positive(frequency)?;
        nonnegative(amplitude)?;
    }
    if let Some([knee, headroom]) = car_definition.audio.soft_limit {
        positive(knee)?;
        positive(headroom)?;
    }
    if let Some(turbo) = &car_definition.turbo {
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

fn load_car(path: &std::path::Path) -> Result<CarSpec, String> {
    let mut definition: CarDefinition =
        serde_json::from_slice(&fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    validate(&definition)?;

    let design = load_model(path)?;
    let height = asset::car_glb::height(&design);
    if !height.is_finite() || height <= 0.0 {
        return Err("model must have finite positive height".into());
    }
    let frontal_area_m2 = asset::car_glb::frontal_area_m2(&design);
    if !frontal_area_m2.is_finite() || frontal_area_m2 <= 0.0 {
        return Err("model must have finite positive frontal area".into());
    }
    let cg_height = definition.cg_height_fraction * height;
    let wheelbase = model_wheelbase(&design.mounts)?;
    let track_width = model_track_width(&design.mounts)?;
    definition.gearbox.rolling_radius =
        model_rolling_radius(design.wheel_radii, definition.drive_bias)?;
    let audio_directory = path.parent().unwrap().join("audio");
    definition.audio.sample_directory = audio_directory
        .join("engine.json")
        .is_file()
        .then_some(audio_directory);

    let mut spec = CarSpec {
        name: definition.name,
        paint_color: definition.paint_color,
        weight_kg: definition.weight_kg,
        engine_max_torque_nm: definition.engine_max_torque_nm,
        engine_torque_curve: definition.engine_torque_curve,
        redline_rpm: definition.redline_rpm,
        gearbox: definition.gearbox,
        design,
        top_speed: 0.0,
        brake_force: definition.brake_force,
        wheelbase,
        audio: definition.audio,
        rear_combined_slip: definition.rear_combined_slip,
        front_weight: definition.front_weight,
        cg_height,
        track_width,
        yaw_inertia: definition.yaw_inertia,
        drive_bias: definition.drive_bias,
        brake_bias: definition.brake_bias,
        corner_stiffness: definition.corner_stiffness,
        suspension_hz: definition.suspension_hz,
        engine_power_watts: definition.engine_power_hp * WATTS_PER_HP,
        drag_coefficient: definition.drag_coefficient,
        frontal_area_m2,
        turbo: definition.turbo,
    };
    spec.top_speed = physics::car::calculate_top_speed(&spec);
    Ok(spec)
}

/// Longitudinal separation of axle midpoints; GLB coordinates use +Z forward.
fn model_wheelbase(mounts: &[XYZ; 4]) -> Result<f32, String> {
    let rear = (mounts[0].z + mounts[1].z) * 0.5;
    let front = (mounts[2].z + mounts[3].z) * 0.5;
    let wheelbase = front - rear;
    if !wheelbase.is_finite() || wheelbase <= 0.0 {
        return Err("model front axle must be ahead of rear axle along +Z".into());
    }
    Ok(wheelbase)
}

/// Mean front/rear distance between wheel mounts along X. The handling model
/// uses one track width for lateral load transfer and body roll over terrain.
fn model_track_width(mounts: &[XYZ; 4]) -> Result<f32, String> {
    let rear = mounts[1].x - mounts[0].x;
    let front = mounts[3].x - mounts[2].x;
    if [rear, front]
        .iter()
        .any(|width| !width.is_finite() || *width <= 0.0)
    {
        return Err("model right wheels must be to the right of left wheels along +X".into());
    }
    Ok(rear * 0.5 + front * 0.5)
}

/// Rolling radius is derived from wheel geometry, averaged per axle and weighted
/// by the front/rear drive split. This rigid-tire approximation converts wheel
/// rotation to road speed; one revolution covers `2π × rolling_radius` metres.
fn model_rolling_radius(radii: [f32; 4], drive_bias: f32) -> Result<f32, String> {
    if radii.iter().any(|r| !r.is_finite() || *r <= 0.0) {
        return Err("model wheels must have finite positive radii".into());
    }
    let rear = radii[0] * 0.5 + radii[1] * 0.5;
    let front = radii[2] * 0.5 + radii[3] * 0.5;
    Ok(rear * (1.0 - drive_bias) + front * drive_bias)
}

fn load_model(path: &std::path::Path) -> Result<std::sync::Arc<asset::car_glb::Design>, String> {
    let model = path.with_file_name("model.glb");
    asset::car_glb::load(&model)
        .map(std::sync::Arc::new)
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::asset;
    use crate::test;

    #[test]
    fn engine_bank_is_discovered_without_a_json_path() {
        let directory = tempfile::tempdir().unwrap();
        test::support::write_car(directory.path());
        let path = directory.path().join("car.json");
        assert!(load_car(&path).unwrap().audio.sample_directory.is_none());
        let audio_directory = directory.path().join("audio");
        fs::create_dir(&audio_directory).unwrap();
        // An unrelated WAV does not opt the car into a recording bank.
        fs::write(audio_directory.join("unused.wav"), []).unwrap();
        assert!(load_car(&path).unwrap().audio.sample_directory.is_none());
        fs::write(audio_directory.join("engine.json"), b"{}").unwrap();
        assert_eq!(
            load_car(&path).unwrap().audio.sample_directory,
            Some(audio_directory)
        );
    }

    #[test]
    fn pitsunda_reference_model_loads() {
        let car = load_car(std::path::Path::new("data/cars/2016-pitsunda-2/car.json"))
            .expect("Pitsunda must load with its traced body and separate wheel pivots");
        assert_eq!(car.paint_color, [0.36, 0.355, 0.35]);
        assert!((asset::car_glb::height(&car.design) - 1.495).abs() < 0.002);
        for radius in car.design.wheel_radii {
            assert!((radius - 0.318).abs() < 0.002);
        }
    }

    #[test]
    fn wheelbase_comes_from_axle_midpoints() {
        // Different left/right Z positions exercise axle averaging.
        let mounts = [
            XYZ {
                x: -0.8,
                y: 0.1,
                z: -1.4,
            },
            XYZ {
                x: 0.8,
                y: 0.2,
                z: -1.2,
            },
            XYZ {
                x: -0.9,
                y: 0.3,
                z: 1.1,
            },
            XYZ {
                x: 0.9,
                y: 0.4,
                z: 1.3,
            },
        ];
        assert!((model_wheelbase(&mounts).unwrap() - 2.5).abs() < 0.00001);
        let translated = mounts.map(|p| XYZ { z: p.z + 10.0, ..p });
        assert!((model_wheelbase(&translated).unwrap() - 2.5).abs() < 0.00001);
        for z in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            let invalid = [
                XYZ {
                    x: -1.0,
                    y: 0.0,
                    z: 0.0,
                },
                XYZ {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                XYZ {
                    x: -1.0,
                    y: 0.0,
                    z: z,
                },
                XYZ {
                    x: 1.0,
                    y: 0.0,
                    z: z,
                },
            ];
            assert!(model_wheelbase(&invalid).is_err());
        }
    }

    #[test]
    fn track_width_comes_from_both_axles() {
        let mounts = [
            XYZ {
                x: -0.7,
                y: 0.0,
                z: -1.0,
            },
            XYZ {
                x: 0.7,
                y: 0.0,
                z: -1.0,
            },
            XYZ {
                x: -0.9,
                y: 0.0,
                z: 1.0,
            },
            XYZ {
                x: 0.9,
                y: 0.0,
                z: 1.0,
            },
        ];
        assert!((model_track_width(&mounts).unwrap() - 1.6).abs() < 0.00001);
        let translated = mounts.map(|p| XYZ { x: p.x + 10.0, ..p });
        assert!((model_track_width(&translated).unwrap() - 1.6).abs() < 0.00001);
        for width in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            for wheel in [1, 3] {
                let mut invalid_mounts = mounts;
                invalid_mounts[wheel].x = invalid_mounts[wheel - 1].x + width;
                assert!(model_track_width(&invalid_mounts).is_err());
            }
        }
    }
}
