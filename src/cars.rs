use crate::geometry::XYZ;
use std::fs;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CarModel {
    Catalog(usize),
    #[cfg(test)]
    TestFixture(usize),
}

pub const ALL: Catalog = Catalog;
#[derive(Clone, Copy)]
pub struct Catalog;
impl Catalog {
    pub fn len(&self) -> usize {
        catalog().len()
    }
    pub fn map<T>(&self, f: impl FnMut(CarModel) -> T) -> Vec<T> {
        self.into_iter().map(f).collect()
    }
}
impl IntoIterator for Catalog {
    type Item = CarModel;
    type IntoIter = std::vec::IntoIter<CarModel>;
    fn into_iter(self) -> Self::IntoIter {
        ordered_models().to_vec().into_iter()
    }
}
impl std::ops::Index<usize> for Catalog {
    type Output = CarModel;
    fn index(&self, index: usize) -> &CarModel {
        &ordered_models()[index]
    }
}
fn ordered_models() -> &'static [CarModel] {
    static MODELS: std::sync::OnceLock<Vec<CarModel>> = std::sync::OnceLock::new();
    MODELS.get_or_init(|| (0..catalog().len()).map(CarModel::Catalog).collect())
}

#[derive(Clone)]
pub struct CarSpec {
    pub design: std::sync::Arc<crate::scene::car_glb::Design>,
    pub name: &'static str,
    pub mass: f32,
    pub top_speed: f32,
    pub brake_force: f32,
    pub tire_friction_coefficient: f32,
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
    pub engine_max_torque_nm: f32,
    pub suspension_hz: f32,
    pub gearbox: Gearbox,
    pub engine_power_hp: f32,
    /// Dimensionless aerodynamic drag coefficient (Cd/Cx).
    drag_coefficient: f32,
    frontal_area_m2: f32,
    pub turbo: Option<Turbo>,
}

const WATTS_PER_HP: f32 = 745.6999;
// Fixed reference air density in kg/m³.
const AIR_DENSITY_KG_M3: f32 = 1.225;

// The physics assumes
// 90% drivetrain efficiency to calculate power at the wheels.
const DRIVETRAIN_EFFICIENCY: f32 = 0.90;

impl CarSpec {
    fn wheel_power_watts(&self) -> f32 {
        self.engine_power_hp * WATTS_PER_HP * DRIVETRAIN_EFFICIENCY
    }
    pub fn drive_force(
        &self,
        speed: f32,
        gear: u8,
        reverse: bool,
        boost_fraction: f32,
        shifting: bool,
    ) -> f32 {
        if gear == crate::vehicle::NEUTRAL_GEAR {
            return 0.0;
        }
        let spec = self;
        let g = &spec.gearbox;
        let ratio = if reverse {
            g.reverse_ratio
        } else {
            g.ratios[gear.saturating_sub(1).min(g.gears() - 1) as usize]
        };
        // While rolling against the selected gear, the clutch slips and drive
        // torque opposes motion.
        let driven_speed = if reverse { -speed } else { speed }.max(0.0);
        let rpm = g.rpm(driven_speed, gear).max(2200.0);
        // Broad torque plateau with weaker idle and redline torque. The clutch
        // slips at launch; gearing subsequently sets wheel force and shift loss.
        let torque_factor = if rpm < 3500.0 {
            0.60 + 0.40 * (rpm / 3500.0)
        } else {
            (1.0 - 0.10 * ((rpm - 3500.0) / 3000.0)).max(0.0)
        };
        let force = spec.engine_max_torque_nm
            * torque_factor
            * ratio
            * g.final_drive
            * crate::cars::DRIVETRAIN_EFFICIENCY
            / g.rolling_radius;
        let force = if shifting { force * 0.2 } else { force };
        let force = force * boost_fraction;
        let force = if reverse {
            force * ((crate::vehicle::REDLINE_RPM - rpm) / 250.0).clamp(0.0, 1.0)
        } else {
            force
        };
        if rpm >= crate::vehicle::REDLINE_RPM {
            0.0
        } else {
            force.min(spec.wheel_power_watts() * boost_fraction / speed.abs().max(1.0))
        }
    }

    pub fn resistance_accel(&self, speed: f32) -> f32 {
        let drag_force =
            0.5 * AIR_DENSITY_KG_M3 * self.drag_coefficient * self.frontal_area_m2 * speed * speed;
        0.25 + drag_force / self.mass
    }

    /// Full-throttle equilibrium on level ground in the best forward gear.
    /// Uses steady turbo boost and the same torque, redline and road load as driving.
    fn calculate_top_speed(&self) -> f32 {
        (1..=self.gearbox.gears())
            .map(|gear| {
                let mut low = 0.0;
                let mut high = self.gearbox.speed_at_rpm(crate::vehicle::REDLINE_RPM, gear);
                for _ in 0..48 {
                    let speed = (low + high) * 0.5;
                    let rpm = self.gearbox.rpm(speed, gear).max(2200.0);
                    let boost = self.turbo.as_ref().map_or(1.0, |t| {
                        let spool = ((rpm - t.spool_start_rpm)
                            / (t.full_boost_rpm - t.spool_start_rpm))
                            .clamp(0.0, 1.0);
                        t.off_boost_fraction + (1.0 - t.off_boost_fraction) * spool
                    });
                    if self.drive_force(speed, gear, false, boost, false)
                        > self.mass * self.resistance_accel(speed)
                    {
                        low = speed;
                    } else {
                        high = speed;
                    }
                }
                low
            })
            .fold(0.0, f32::max)
    }
}

/// Per-car engine timbre and mix, independent of model identity.
#[derive(Clone)]
pub struct EngineVoice {
    pub harmonics: Vec<[f32; 2]>, // frequency multiplier, amplitude
    pub gain: f32,
    pub coast_gain: f32,
    pub soft_limit: Option<[f32; 2]>, // knee, headroom
    pub pulses_per_revolution: f32,
    pub samples: Option<std::path::PathBuf>,
}
fn engine_voice(path: &std::path::Path, value: &serde_json::Value) -> Result<EngineVoice, String> {
    let harmonics = value["harmonics"]
        .as_array()
        .filter(|a| !a.is_empty())
        .ok_or("expected engine harmonics")?
        .iter()
        .map(|v| {
            let pair = v
                .as_array()
                .filter(|a| a.len() == 2)
                .ok_or("expected harmonic pair")?;
            Ok([num(&pair[0])?, nonnegative(&pair[1])?])
        })
        .collect::<Result<Vec<_>, String>>()?;
    let soft_limit = if value["soft_limit"].is_null() {
        None
    } else {
        let pair = value["soft_limit"]
            .as_array()
            .filter(|a| a.len() == 2)
            .ok_or("expected soft limit knee and headroom")?;
        Some([num(&pair[0])?, num(&pair[1])?])
    };
    let samples = if value["samples"].is_null() {
        None
    } else {
        Some(asset_path(path, &value["samples"])?)
    };
    Ok(EngineVoice {
        harmonics,
        gain: num(&value["gain"])?,
        coast_gain: frac(&value["coast_gain"])?,
        soft_limit,
        pulses_per_revolution: num(&value["pulses_per_revolution"])?,
        samples,
    })
}

/// Full-boost torque/power are already included in the engine tune.
#[derive(Clone)]
pub struct Turbo {
    pub off_boost_fraction: f32,
    pub spool_start_rpm: f32,
    pub full_boost_rpm: f32,
    pub spool_seconds: f32,
    pub decay_seconds: f32,
}

fn turbo(value: &serde_json::Value) -> Result<Option<Turbo>, String> {
    if value.is_null() {
        return Ok(None);
    }
    let tune = Turbo {
        off_boost_fraction: frac(&value["off_boost_fraction"])?,
        spool_start_rpm: num(&value["spool_start_rpm"])?,
        full_boost_rpm: num(&value["full_boost_rpm"])?,
        spool_seconds: num(&value["spool_seconds"])?,
        decay_seconds: num(&value["decay_seconds"])?,
    };
    if tune.full_boost_rpm <= tune.spool_start_rpm {
        return Err("full boost RPM must exceed spool start RPM".into());
    }
    Ok(Some(tune))
}

/// Ratios and effective rolling radius determine engine RPM from road speed.
/// Sources and game-specific shift thresholds are recorded in GEARBOXES.md.
#[derive(Clone)]
pub struct Gearbox {
    pub ratios: &'static [f32],
    pub final_drive: f32,
    pub rolling_radius: f32,
    pub reverse_ratio: f32,
    pub upshift_rpm: f32,
    pub downshift_rpm: f32,
}
impl Gearbox {
    pub fn gears(&self) -> u8 {
        self.ratios.len() as u8
    }
    pub fn rpm(&self, speed: f32, gear: u8) -> f32 {
        let ratio = if gear == 0 {
            self.reverse_ratio
        } else {
            self.ratios[gear.saturating_sub(1).min(self.gears() - 1) as usize]
        };
        let speed = if gear == 0 {
            speed.abs()
        } else {
            speed.max(0.0)
        };
        speed * 60.0 / (std::f32::consts::TAU * self.rolling_radius) * ratio * self.final_drive
    }
    pub fn speed_at_rpm(&self, rpm: f32, gear: u8) -> f32 {
        let ratio = if gear == 0 {
            self.reverse_ratio
        } else {
            self.ratios[gear.saturating_sub(1).min(self.gears() - 1) as usize]
        };
        rpm * std::f32::consts::TAU * self.rolling_radius / (60.0 * ratio * self.final_drive)
    }
}

fn num(v: &serde_json::Value) -> Result<f32, String> {
    let n = v.as_f64().ok_or("expected number")? as f32;
    if !n.is_finite() || n <= 0.0 {
        return Err("expected finite positive number".into());
    }
    Ok(n)
}
fn frac(v: &serde_json::Value) -> Result<f32, String> {
    let n = nonnegative(v)?;
    if n > 1.0 {
        return Err("expected fraction in 0..1".into());
    }
    Ok(n)
}
fn nonnegative(v: &serde_json::Value) -> Result<f32, String> {
    let n = signed(v)?;
    if n < 0.0 {
        return Err("expected nonnegative number".into());
    }
    Ok(n)
}
fn signed(v: &serde_json::Value) -> Result<f32, String> {
    let n = v.as_f64().ok_or("expected number")? as f32;
    if !n.is_finite() {
        return Err("expected finite number".into());
    }
    Ok(n)
}
fn text(v: &serde_json::Value) -> Result<&'static str, String> {
    let s = v
        .as_str()
        .filter(|s| !s.trim().is_empty())
        .ok_or("expected nonempty text")?;
    Ok(Box::leak(s.to_owned().into_boxed_str()))
}
fn load_definition(path: &std::path::Path, v: &serde_json::Value) -> Result<CarSpec, String> {
    let stiffness = v["corner_stiffness"]
        .as_array()
        .filter(|a| a.len() == 2)
        .ok_or("expected two stiffness values")?;
    let rear_combined_slip = if v["rear_combined_slip"].is_null() {
        None
    } else {
        let a = v["rear_combined_slip"]
            .as_array()
            .filter(|a| a.len() == 3)
            .ok_or("expected combined slip baseline, reduction and angle")?;
        let values = [frac(&a[0])?, frac(&a[1])?, num(&a[2])?];
        if values[1] > values[0] {
            return Err("combined slip reduction exceeds baseline".into());
        }
        Some(values)
    };

    let g = &v["gearbox"];
    let ratios = g["ratios"]
        .as_array()
        .filter(|a| !a.is_empty() && a.len() <= 254)
        .ok_or("expected 1..254 ratios")?
        .iter()
        .map(num)
        .collect::<Result<Vec<_>, _>>()?;
    if !ratios.windows(2).all(|p| p[0] > p[1]) {
        return Err("ratios must descend".into());
    }

    let design = load_model(path)?;
    let height = design.height();
    if !height.is_finite() || height <= 0.0 {
        return Err("model must have finite positive height".into());
    }
    let frontal_area_m2 = design.frontal_area_m2();
    if !frontal_area_m2.is_finite() || frontal_area_m2 <= 0.0 {
        return Err("model must have finite positive frontal area".into());
    }
    let cg_height = frac(&v["cg_height_fraction"])? * height;
    let wheelbase = model_wheelbase(&design.mounts)?;
    let track_width = model_track_width(&design.mounts)?;
    let drive_bias = frac(&v["drive_bias"])?;
    let rolling_radius = model_rolling_radius(design.wheel_radii, drive_bias)?;
    let gearbox = Gearbox {
        ratios: Box::leak(ratios.into_boxed_slice()),
        final_drive: num(&g["final_drive"])?,
        rolling_radius,
        reverse_ratio: num(&g["reverse_ratio"])?,
        upshift_rpm: num(&g["upshift_rpm"])?,
        downshift_rpm: num(&g["downshift_rpm"])?,
    };

    let mut spec = CarSpec {
        audio: engine_voice(path, &v["audio"])?,
        design,
        turbo: turbo(&v["turbo"])?,
        name: text(&v["name"])?,
        mass: num(&v["weight_kg"])?,
        top_speed: 0.0,
        brake_force: num(&v["brake_force"])?,
        tire_friction_coefficient: num(&v["tire_friction_coefficient"])?,
        wheelbase,
        engine_power_hp: num(&v["engine_power_hp"])?,
        drag_coefficient: num(&v["drag_coefficient"])?,
        frontal_area_m2,
        rear_combined_slip,
        front_weight: frac(&v["front_weight"])?,
        cg_height,
        track_width,
        yaw_inertia: num(&v["yaw_inertia"])?,
        drive_bias,
        brake_bias: frac(&v["brake_bias"])?,
        engine_max_torque_nm: num(&v["engine_max_torque_nm"])?,
        suspension_hz: num(&v["suspension_hz"])?,
        corner_stiffness: [num(&stiffness[0])?, num(&stiffness[1])?],
        gearbox,
    };
    if spec.gearbox.downshift_rpm >= spec.gearbox.upshift_rpm {
        return Err("invalid shift thresholds".into());
    }
    spec.top_speed = spec.calculate_top_speed();
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

fn load_model(
    path: &std::path::Path,
) -> Result<std::sync::Arc<crate::scene::car_glb::Design>, String> {
    let model = path.with_file_name("model.glb");
    crate::scene::car_glb::load(&model)
        .map(std::sync::Arc::new)
        .map_err(|e| e.to_string())
}

impl CarModel {
    /// Rear pair first, then front pair. Shared by artwork and tire feedback.
    pub fn wheel_centers(self) -> [XYZ; 4] {
        self.spec().design.mounts
    }

    pub fn index(self) -> usize {
        match self {
            Self::Catalog(index) => index,
            #[cfg(test)]
            Self::TestFixture(_) => panic!("module test fixture has no catalog index"),
        }
    }
    #[cfg(test)]
    fn folder_name(self) -> &'static str {
        &catalog()[self.index()].folder
    }
    pub fn spec(self) -> &'static CarSpec {
        match self {
            Self::Catalog(index) => &catalog()[index].spec,
            #[cfg(test)]
            Self::TestFixture(index) => crate::test_support::fixture_spec(index),
        }
    }
    pub fn first() -> Self {
        ALL[0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wheelbase_comes_from_axle_midpoints() {
        // Different left/right Z positions exercise axle averaging.
        let mounts = [
            crate::geometry::XYZ {
                x: -0.8,
                y: 0.1,
                z: -1.4,
            },
            crate::geometry::XYZ {
                x: 0.8,
                y: 0.2,
                z: -1.2,
            },
            crate::geometry::XYZ {
                x: -0.9,
                y: 0.3,
                z: 1.1,
            },
            crate::geometry::XYZ {
                x: 0.9,
                y: 0.4,
                z: 1.3,
            },
        ];
        assert!((model_wheelbase(&mounts).unwrap() - 2.5).abs() < 0.00001);
        let translated = mounts.map(|p| crate::geometry::XYZ { z: p.z + 10.0, ..p });
        assert!((model_wheelbase(&translated).unwrap() - 2.5).abs() < 0.00001);
        for z in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            let invalid = [
                crate::geometry::XYZ {
                    x: -1.0,
                    y: 0.0,
                    z: 0.0,
                },
                crate::geometry::XYZ {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                crate::geometry::XYZ {
                    x: -1.0,
                    y: 0.0,
                    z: z,
                },
                crate::geometry::XYZ {
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
            crate::geometry::XYZ {
                x: -0.7,
                y: 0.0,
                z: -1.0,
            },
            crate::geometry::XYZ {
                x: 0.7,
                y: 0.0,
                z: -1.0,
            },
            crate::geometry::XYZ {
                x: -0.9,
                y: 0.0,
                z: 1.0,
            },
            crate::geometry::XYZ {
                x: 0.9,
                y: 0.0,
                z: 1.0,
            },
        ];
        assert!((model_track_width(&mounts).unwrap() - 1.6).abs() < 0.00001);
        let translated = mounts.map(|p| crate::geometry::XYZ { x: p.x + 10.0, ..p });
        assert!((model_track_width(&translated).unwrap() - 1.6).abs() < 0.00001);
        for width in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            for wheel in [1, 3] {
                let mut invalid_mounts = mounts;
                invalid_mounts[wheel].x = invalid_mounts[wheel - 1].x + width;
                assert!(model_track_width(&invalid_mounts).is_err());
            }
        }
    }

    #[test]
    fn rolling_radius_uses_driven_wheel_geometry() {
        let radii = [0.30, 0.34, 0.40, 0.44];
        assert!((model_rolling_radius(radii, 0.0).unwrap() - 0.32).abs() < 0.00001);
        assert!((model_rolling_radius(radii, 1.0).unwrap() - 0.42).abs() < 0.00001);
        assert!((model_rolling_radius(radii, 0.25).unwrap() - 0.345).abs() < 0.00001);
        for invalid in [0.0, -0.1, f32::NAN, f32::INFINITY] {
            assert!(model_rolling_radius([invalid, 0.3, 0.3, 0.3], 1.0).is_err());
        }
    }

    #[test]
    fn aerodynamic_drag_uses_coefficient_area_and_mass() {
        let mut spec = crate::test_support::car_spec().clone();
        spec.drag_coefficient = 0.3;
        spec.frontal_area_m2 = 2.0;
        spec.mass = 1000.0;
        // At 20 m/s this gives 147 N of aerodynamic drag, plus rolling resistance.
        let deceleration = spec.resistance_accel(20.0) - spec.resistance_accel(0.0);
        assert!((deceleration - 0.147).abs() < 0.000001);
        assert_eq!(spec.resistance_accel(-20.0), spec.resistance_accel(20.0));
        spec.mass *= 2.0;
        let heavier = spec.resistance_accel(20.0) - spec.resistance_accel(0.0);
        assert!((heavier * 2.0 - deceleration).abs() < 0.000001);
        spec.drag_coefficient *= 2.0;
        let more_drag = spec.resistance_accel(20.0) - spec.resistance_accel(0.0);
        assert!((more_drag - deceleration).abs() < 0.000001);
        spec.frontal_area_m2 *= 2.0;
        let larger_area = spec.resistance_accel(20.0) - spec.resistance_accel(0.0);
        assert!((larger_area - 2.0 * deceleration).abs() < 0.000001);
    }

    #[test]
    fn calculated_top_speed_balances_power_and_road_load() {
        let mut spec = crate::test_support::car_spec().clone();
        spec.turbo = None;
        spec.engine_max_torque_nm = 10_000.0;
        spec.gearbox.final_drive = 1.0;
        let speed = spec.calculate_top_speed();
        let road_power = spec.mass * spec.resistance_accel(speed) * speed;
        assert!((road_power - spec.wheel_power_watts()).abs() < 1.0);
        spec.engine_power_hp *= 2.0;
        let more_power = spec.calculate_top_speed();
        assert!(more_power > speed);
        spec.drag_coefficient *= 2.0;
        assert!(spec.calculate_top_speed() < more_power);
        let before_weight_change = spec.calculate_top_speed();
        spec.mass *= 2.0;
        assert!(spec.calculate_top_speed() < before_weight_change);
    }

    #[test]
    fn calculated_top_speed_respects_gearing_and_redline() {
        let mut spec = crate::test_support::car_spec().clone();
        spec.turbo = None;
        spec.engine_max_torque_nm = 10_000.0;
        spec.gearbox.final_drive = 100.0;
        let limit = spec
            .gearbox
            .speed_at_rpm(crate::vehicle::REDLINE_RPM, spec.gearbox.gears());
        assert!((spec.calculate_top_speed() - limit).abs() < 0.001);
    }

    #[test]
    fn catalog_is_ordered_and_audio_banks_load() {
        let names: Vec<_> = ALL.into_iter().map(CarModel::folder_name).collect();
        assert!(names.windows(2).all(|p| p[0] < p[1]));
        for (index, model) in ALL.into_iter().enumerate() {
            assert_eq!(model.index(), index);
            let s = model.spec();
            if let Some(samples) = &s.audio.samples {
                let mut engine = crate::engine_sample::RecordedEngine::default();
                engine.select(Some(samples));
                assert!(
                    engine.available(),
                    "{} audio bank {}",
                    s.name,
                    samples.display()
                );
            }
        }
    }
}

struct CarEntry {
    #[cfg(test)]
    folder: String,
    spec: CarSpec,
}
fn asset_path(
    definition: &std::path::Path,
    value: &serde_json::Value,
) -> Result<std::path::PathBuf, String> {
    let name = value
        .as_str()
        .filter(|s| !s.trim().is_empty())
        .ok_or("expected nonempty asset path")?;
    Ok(definition.parent().unwrap().join(name))
}

fn load_car(path: &std::path::Path) -> Result<CarSpec, String> {
    let value: serde_json::Value =
        serde_json::from_slice(&fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    if !value.is_object() {
        return Err("expected car object".into());
    }
    load_definition(path, &value)
}

fn discover(root: &std::path::Path) -> Result<Vec<CarEntry>, String> {
    let entries = fs::read_dir(root).map_err(|e| format!("{}: {e}", root.display()))?;
    let mut paths = Vec::new();
    for entry in entries {
        let path = entry.map_err(|e| e.to_string())?.path();
        if path.is_dir() && path.join("car.json").is_file() {
            paths.push(path.join("car.json"));
        }
    }
    paths.sort();
    let mut cars = Vec::new();
    for path in paths {
        match load_car(&path) {
            Ok(spec) => cars.push(CarEntry {
                #[cfg(test)]
                folder: path
                    .parent()
                    .unwrap()
                    .file_name()
                    .unwrap()
                    .to_string_lossy()
                    .into_owned(),
                spec,
            }),
            Err(error) => log::warn!("Cannot load car {}: {error}", path.display()),
        }
    }
    Ok(cars)
}
fn catalog() -> &'static [CarEntry] {
    static CARS: std::sync::OnceLock<Vec<CarEntry>> = std::sync::OnceLock::new();
    CARS.get_or_init(|| {
        let cars = discover(std::path::Path::new("data/cars"))
            .unwrap_or_else(|e| panic!("Cannot load cars: {e}"));
        assert!(!cars.is_empty(), "No valid cars in data/cars");
        cars
    })
}

#[cfg(test)]
mod runtime_tests {
    use super::*;

    #[test]
    fn runtime_catalog_probe() {
        if std::env::var_os("TEST_RUNTIME_CAR").is_none() {
            return;
        }
        assert_eq!(ALL.len(), 2);
        assert_eq!(
            ALL.into_iter()
                .find(|m| m.folder_name() == "template")
                .unwrap()
                .spec()
                .mass,
            999.0
        );
        let car = ALL[0];
        assert_eq!(car.index(), 0);
        assert_eq!(car.spec().name, "NEW CAR");
        assert_eq!(car.spec().mass, 1234.0);
        assert_eq!(car.spec().gearbox.ratios, &[3.0, 2.0, 1.0]);
        assert_eq!(car.wheel_centers()[0].x, -0.99);
        assert_eq!(
            crate::hud::selection_menu_hit_test(0.5, -0.07, 0, 2, car),
            Some(crate::hud::MenuAction::Car(car))
        );
    }

    #[test]
    fn glb_definitions_load_local_assets() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("car.json");
        crate::test_support::write_car(dir.path());
        assert!(!load_car(&path).unwrap().design.body.is_empty());
        fs::rename(dir.path().join("model.glb"), dir.path().join("other.glb")).unwrap();
        assert!(load_car(&path).err().unwrap().contains("model.glb"));
        fs::rename(dir.path().join("other.glb"), dir.path().join("model.glb")).unwrap();
        fs::write(dir.path().join("model.glb"), b"broken").unwrap();
        let error = load_car(&path).err().unwrap();
        assert!(
            error.contains("model.glb") && error.contains("binary glTF"),
            "{error}"
        );
    }

    #[test]
    fn gearbox_limit_keeps_neutral_distinct() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("car.json");
        let original = crate::test_support::write_car(dir.path());
        for count in [254, 255] {
            let ratios: Vec<_> = (1..=count).rev().collect();
            let mut required = original.clone();
            required["gearbox"]["ratios"] = serde_json::json!(ratios);
            fs::write(&path, serde_json::to_vec(&required).unwrap()).unwrap();
            assert_eq!(load_car(&path).is_ok(), count == 254);
        }
    }

    #[test]
    fn required_parameters_are_validated() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("car.json");
        let original = crate::test_support::write_car(dir.path());
        for (key, value) in [
            ("weight_kg", serde_json::json!(-1)),
            ("drag_coefficient", serde_json::json!(0)),
            ("drag_coefficient", serde_json::json!(-1)),
            ("drag_coefficient", serde_json::Value::Null),
            ("cg_height_fraction", serde_json::json!(-0.1)),
            ("cg_height_fraction", serde_json::json!(1.1)),
            ("cg_height_fraction", serde_json::Value::Null),
            ("tire_friction_coefficient", serde_json::json!(0)),
            ("tire_friction_coefficient", serde_json::json!(-1)),
            ("tire_friction_coefficient", serde_json::Value::Null),
            ("engine_power_hp", serde_json::json!(0)),
            ("engine_power_hp", serde_json::Value::Null),
            ("gearbox", serde_json::json!({"ratios": []})),
        ] {
            let mut invalid = original.clone();
            invalid[key] = value;
            fs::write(&path, serde_json::to_vec(&invalid).unwrap()).unwrap();
            assert!(load_car(&path).is_err(), "accepted invalid {key}");
        }
    }

    #[test]
    fn authoring_units_convert_to_physics_units() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("car.json");
        let mut definition = crate::test_support::write_car(dir.path());
        definition["weight_kg"] = serde_json::json!(1200);
        definition["engine_power_hp"] = serde_json::json!(100);
        definition["tire_friction_coefficient"] = serde_json::json!(1.2);
        definition["cg_height_fraction"] = serde_json::json!(0.5);
        fs::write(&path, serde_json::to_vec(&definition).unwrap()).unwrap();
        let car = load_car(&path).unwrap();
        assert_eq!(car.mass, 1200.0);
        assert!((car.cg_height - 0.65).abs() < 0.00001);
        assert_eq!(car.engine_power_hp, 100.0);
        assert_eq!(car.tire_friction_coefficient, 1.2);
        // 100 mechanical hp = 74.570 kW at the engine, 67.113 kW at the wheels.
        assert!((car.wheel_power_watts() - 67_112.99).abs() < 0.02);
        for (fraction, expected) in [(0.0, 0.0), (1.0, 1.3)] {
            definition["cg_height_fraction"] = serde_json::json!(fraction);
            fs::write(&path, serde_json::to_vec(&definition).unwrap()).unwrap();
            assert!((load_car(&path).unwrap().cg_height - expected).abs() < 0.00001);
        }
    }

    #[test]
    fn bundled_car_definitions_are_independent() {
        for model in ALL {
            let dir = tempfile::tempdir().unwrap();
            let source = std::path::Path::new("data/cars").join(model.folder_name());
            let mut definition: serde_json::Value =
                serde_json::from_slice(&fs::read(source.join("car.json")).unwrap()).unwrap();
            fs::copy(source.join("model.glb"), dir.path().join("model.glb")).unwrap();
            // Audio assets may be shared, but paths must resolve from this car's folder.
            let expected_samples = model
                .spec()
                .audio
                .samples
                .as_ref()
                .map(|p| fs::canonicalize(p).unwrap());
            if let Some(samples) = &expected_samples {
                definition["audio"]["samples"] = serde_json::json!(samples);
            }
            let path = dir.path().join("car.json");
            fs::write(&path, serde_json::to_vec(&definition).unwrap()).unwrap();
            let car = load_car(&path).unwrap();
            assert_eq!(car.name, model.spec().name);
            assert_eq!(car.mass, model.spec().mass);
            assert_eq!(car.wheelbase, model.spec().wheelbase);
            assert_eq!(car.track_width, model.spec().track_width);
            assert_eq!(car.frontal_area_m2, model.spec().frontal_area_m2);
            assert_eq!(car.cg_height, model.spec().cg_height);
            assert_eq!(
                car.gearbox.rolling_radius,
                model.spec().gearbox.rolling_radius
            );
            assert_eq!(car.rear_combined_slip, model.spec().rear_combined_slip);
            assert_eq!(car.audio.samples, expected_samples);
        }
    }

    #[test]
    fn new_car_is_discovered_by_existing_binary() {
        let dir = tempfile::tempdir().unwrap();
        let cars = dir.path().join("data/cars");
        fs::create_dir_all(&cars).unwrap();
        // Neither a known folder name nor any original car is required.
        let mut definition = crate::test_support::write_car(&cars.join("template"));
        definition["weight_kg"] = serde_json::json!(999);
        fs::write(
            cars.join("template/car.json"),
            serde_json::to_vec(&definition).unwrap(),
        )
        .unwrap();

        // Supply custom geometry and pivots through a runtime-only package.
        fs::create_dir_all(cars.join("new")).unwrap();
        let mut design = (*crate::test_support::car_spec().design).clone();
        design.body[0].position[0] = 7.25;
        design.wheels[0][0].position[0] = 0.123;
        design.mounts[0].x = -0.99;
        fs::write(
            cars.join("new/model.glb"),
            crate::test_support::model_glb(&design),
        )
        .unwrap();
        fs::create_dir_all(cars.join("notes")).unwrap();
        fs::write(cars.join("ignored.json"), "{}").unwrap();
        definition["name"] = serde_json::json!("NEW CAR");
        definition["weight_kg"] = serde_json::json!(1234);
        definition["gearbox"]["ratios"] = serde_json::json!([3, 2, 1]);
        fs::write(
            cars.join("new/car.json"),
            serde_json::to_vec(&definition).unwrap(),
        )
        .unwrap();
        let mut invalid = crate::test_support::write_car(&cars.join("bad"));
        invalid["weight_kg"] = serde_json::json!(-1);
        fs::write(
            cars.join("bad/car.json"),
            serde_json::to_vec(&invalid).unwrap(),
        )
        .unwrap();
        for (folder, contents) in [
            ("missing_model", None),
            ("corrupt_model", Some(b"invalid model".as_slice())),
        ] {
            let folder = cars.join(folder);
            fs::create_dir_all(&folder).unwrap();
            if let Some(bytes) = contents {
                fs::write(folder.join("model.glb"), bytes).unwrap();
            }
            fs::write(
                folder.join("car.json"),
                serde_json::to_vec(&definition).unwrap(),
            )
            .unwrap();
        }
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .current_dir(dir.path())
            .env("TEST_RUNTIME_CAR", "1")
            .args([
                "--exact",
                "cars::runtime_tests::runtime_catalog_probe",
                "--nocapture",
            ])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[cfg(test)]
#[path = "test_support.rs"]
pub mod test_support;
