use crate::asset;
use crate::model::car_loading;
use crate::physics;
use crate::util::geometry::XYZ;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

#[derive(Clone, Debug)]
pub struct CarSpec {
    pub name: String,
    //
    pub design: std::sync::Arc<asset::car_glb::Design>,
    pub paint_color: [f32; 3],
    //
    /// Dimensionless aerodynamic drag coefficient (Cd/Cx).
    pub drag_coefficient: f32,
    pub frontal_area_m2: f32,
    /// Full model height in metres, including wheels.
    pub model_height_m: f32,
    //
    pub weight_kg: f32,
    pub center_of_mass: CenterOfMass,
    /// Moment of inertia about the vertical axis through the center of mass, in kg·m².
    pub yaw_inertia_kg_m2: f32,
    //
    pub engine: Engine,
    pub engine_sample_directory: Option<PathBuf>,
    //
    pub gearbox: Gearbox,
    pub chassis: Chassis,
}

#[derive(Clone, Debug)]
pub struct CenterOfMass {
    /// Position between axles: 0 = rear axle, 1 = front axle.
    pub longitudinal_fraction: f32,
    /// Height above ground as a fraction of the full model height, in 0..=1.
    pub height_fraction: f32,
}

#[derive(Clone, Debug)]
pub struct Chassis {
    /// Maximum total foot-braking force at full pressure, in newtons.
    pub brake_force_n: f32,
    /// Combined effective suspension spring stiffness for the whole vehicle, in N/m.
    pub suspension_stiffness_n_m: f32,
    /// Effective driven-wheel radius in metres, derived from the model.
    pub rolling_radius: f32,
    pub wheelbase: f32,
    pub track_width: f32,
    /// Fraction of driving force sent to the front axle: 0 = RWD, 1 = FWD.
    pub drive_bias: f32,
    /// Fraction of foot-braking force applied to the front axle.
    pub brake_bias: f32,
}

#[derive(Clone, Debug)]
pub struct Engine {
    /// Effective crankshaft and flywheel rotational inertia, in kg m².
    pub inertia_kg_m2: f32,
    pub max_torque_nm: f32,
    pub torque_curve: Vec<TorquePoint>,
    pub redline_rpm: f32,
    pub power_watts: f32,
    pub turbo: Option<Turbo>,
}

#[derive(Clone, Debug)]
pub struct TorquePoint {
    pub rpm: f32,
    /// Fraction of the engine's maximum torque.
    pub torque: f32,
}

#[derive(Clone, Debug)]
pub struct Turbo {
    pub spool_start_rpm: f32,
    pub full_boost_rpm: f32,
    pub spool_seconds: f32,
    pub decay_seconds: f32,
}

#[derive(Clone, Debug)]
pub struct Gearbox {
    pub ratios: Vec<f32>,
    pub final_drive: f32,
    pub reverse_ratio: f32,
}

pub fn torque_factor(curve: &[TorquePoint], rpm: f32) -> f32 {
    let upper = curve.partition_point(|point| point.rpm < rpm);
    if upper == 0 {
        return curve[0].torque;
    }
    if upper == curve.len() {
        return curve[upper - 1].torque;
    }
    let lower = &curve[upper - 1];
    let upper = &curve[upper];
    let fraction = (rpm - lower.rpm) / (upper.rpm - lower.rpm);
    lower.torque + (upper.torque - lower.torque) * fraction
}

/// Height of the center of mass above ground, in metres.
pub fn center_of_mass_height_m(spec: &CarSpec) -> f32 {
    spec.center_of_mass.height_fraction * spec.model_height_m
}

// Loads cars from the given directory.
pub fn load(dirpath: &Path) -> Result<Vec<Arc<CarSpec>>, String> {
    let mut cars = Vec::new();
    let entries = fs::read_dir(dirpath).map_err(|e| format!("{}: {e}", dirpath.display()))?;
    for entry in entries {
        let entry = entry.map_err(|e| e.to_string())?;
        let directory = entry.path();
        if !directory.is_dir() {
            log::warn!(
                "Skipping car entry {}: not a directory",
                directory.display()
            );
            continue;
        }
        let path = directory.join("car.json");
        if !path.is_file() {
            log::warn!("Skipping car definition {}: not a file", path.display());
            continue;
        }
        let Ok(name) = entry.file_name().into_string() else {
            log::warn!(
                "Skipping car entry {}: expected UTF-8 car directory name",
                directory.display()
            );
            continue;
        };
        match load_car(&directory, name) {
            Ok(spec) => cars.push(Arc::new(spec)),
            Err(error) => log::warn!("Cannot load car {}: {error}", path.display()),
        }
    }
    cars.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(cars)
}

fn load_car(directory: &Path, name: String) -> Result<CarSpec, String> {
    let definition = car_loading::load(&directory.join("car.json"))?;
    if definition.engine.redline_rpm <= physics::engine::IDLE_RPM + 250.0 {
        return Err("redline RPM must leave limiter recovery above idle".into());
    }

    let design = load_model(directory)?;
    let height = asset::car_glb::height(&design);
    if !height.is_finite() || height <= 0.0 {
        return Err("model must have finite positive height".into());
    }
    let frontal_area_m2 = asset::car_glb::frontal_area_m2(&design);
    if !frontal_area_m2.is_finite() || frontal_area_m2 <= 0.0 {
        return Err("model must have finite positive frontal area".into());
    }
    let wheelbase = model_wheelbase(&design.mounts)?;
    let track_width = model_track_width(&design.mounts)?;
    let rolling_radius = model_rolling_radius(design.wheel_radii, definition.chassis.drive_bias)?;
    let audio_directory = directory.join("audio");
    let engine_sample_directory = audio_directory.is_dir().then_some(audio_directory);

    let spec = CarSpec {
        name,
        paint_color: definition.paint_color,
        weight_kg: definition.weight_kg,
        engine: Engine {
            inertia_kg_m2: definition.engine.inertia_kg_m2,
            max_torque_nm: definition.engine.max_torque_nm,
            torque_curve: definition
                .engine
                .torque_curve
                .into_iter()
                .map(|point| TorquePoint {
                    rpm: point.rpm,
                    torque: point.torque,
                })
                .collect(),
            redline_rpm: definition.engine.redline_rpm,
            power_watts: definition.engine.power_kw * 1000.0,
            turbo: definition.engine.turbo.map(|turbo| Turbo {
                spool_start_rpm: turbo.spool_start_rpm,
                full_boost_rpm: turbo.full_boost_rpm,
                spool_seconds: turbo.spool_seconds,
                decay_seconds: turbo.decay_seconds,
            }),
        },
        gearbox: Gearbox {
            ratios: definition.gearbox.ratios,
            final_drive: definition.gearbox.final_drive,
            reverse_ratio: definition.gearbox.reverse_ratio,
        },
        design,
        engine_sample_directory,
        center_of_mass: CenterOfMass {
            longitudinal_fraction: definition.center_of_mass.longitudinal_fraction,
            height_fraction: definition.center_of_mass.height_fraction,
        },
        model_height_m: height,
        yaw_inertia_kg_m2: definition.yaw_inertia_kg_m2,
        chassis: Chassis {
            brake_force_n: definition.chassis.brake_force_n,
            rolling_radius,
            wheelbase,
            track_width,
            drive_bias: definition.chassis.drive_bias,
            brake_bias: definition.chassis.brake_bias,
            suspension_stiffness_n_m: definition.chassis.suspension_stiffness_n_m,
        },
        drag_coefficient: definition.drag_coefficient,
        frontal_area_m2,
    };
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

fn load_model(directory: &Path) -> Result<Arc<asset::car_glb::Design>, String> {
    let model = directory.join("model.glb");
    asset::car_glb::load(&model)
        .map(Arc::new)
        .map_err(|e| e.to_string())
}
