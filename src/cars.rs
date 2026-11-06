//! Runtime car catalog: selection, rendering, physics, and AI use one identity.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CarModel {
    Roadster,
    Muscle,
    Club,
    Custom(usize),
}

pub const ALL: Catalog = Catalog;
#[derive(Clone, Copy)]
pub struct Catalog;
impl Catalog {
    pub fn len(&self) -> usize {
        3 + extras().len()
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
    MODELS.get_or_init(|| {
        let mut models: Vec<_> = [CarModel::Roadster, CarModel::Muscle, CarModel::Club]
            .into_iter()
            .chain((0..extras().len()).map(CarModel::Custom))
            .collect();
        models.sort_by_key(|model| model.folder_name());
        models
    })
}
#[cfg(test)]
pub const WHEEL_RADIUS: f32 = 0.4;

#[derive(Clone)]
pub struct CarSpec {
    pub(crate) design: std::sync::Arc<crate::scene::car_glb::Design>,
    pub name: &'static str,
    pub mass: f32,
    pub engine_force: f32,
    pub top_speed: f32,
    pub brake_force: f32,
    pub lateral_force: f32,
    pub wheelbase: f32,
    pub steering_response: f32,
    pub dynamics: Dynamics,
    pub gearbox: Gearbox,
    pub road_engine: Option<RoadEngine>,
    pub turbo: Option<Turbo>,
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
        off_boost_fraction: fraction(&value["off_boost_fraction"])?,
        spool_start_rpm: number(&value["spool_start_rpm"])?,
        full_boost_rpm: number(&value["full_boost_rpm"])?,
        spool_seconds: number(&value["spool_seconds"])?,
        decay_seconds: number(&value["decay_seconds"])?,
    };
    if tune.full_boost_rpm <= tune.spool_start_rpm {
        return Err("full boost RPM must exceed spool start RPM".into());
    }
    Ok(Some(tune))
}

/// Handling parameters in SI units. Rear drive is 0, front drive is 1.
/// These are game calibrations, not measured vehicle specifications.
#[derive(Clone)]
pub struct Dynamics {
    pub front_weight: f32,
    pub cg_height: f32,
    pub track_width: f32,
    pub yaw_inertia: f32,
    pub front_drive: f32,
    pub brake_bias: f32,
    pub corner_stiffness: [f32; 2], // force per radian, front/rear axle
    pub peak_torque: f32,
    pub suspension_hz: f32,
    pub steering_low_speed_squared: f32,
    pub steering_speed_squared: f32, // additional steering-lock falloff per (m/s)^2
}

/// Car-specific wheel power and aerodynamic resistance, independent of map.
/// Acceleration remains launch-force limited; this is not a full torque curve.
#[derive(Clone)]
pub struct RoadEngine {
    pub wheel_power_watts: f32,
    pub drag_accel: f32,
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
        let ratio = self.ratios[gear.saturating_sub(1).min(self.gears() - 1) as usize];
        rpm * std::f32::consts::TAU * self.rolling_radius / (60.0 * ratio * self.final_drive)
    }
}

fn originals() -> &'static [CarSpec; 3] {
    static CARS: std::sync::OnceLock<[CarSpec; 3]> = std::sync::OnceLock::new();
    CARS.get_or_init(|| {
        ["kestrel", "muscle", "club"].map(|id| {
            let path = std::path::PathBuf::from(format!("data/cars/{id}/car.json"));
            load_original(&path).unwrap_or_else(|error| {
                panic!("Cannot load required car {}: {error}", path.display())
            })
        })
    })
}
fn number(v: &serde_json::Value) -> Result<f32, String> {
    let n = v.as_f64().ok_or("expected number")? as f32;
    if !n.is_finite() || n <= 0.0 {
        return Err("expected finite positive number".into());
    }
    Ok(n)
}
fn fraction(v: &serde_json::Value) -> Result<f32, String> {
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
fn load_original(path: &std::path::Path) -> Result<CarSpec, String> {
    let v: serde_json::Value =
        serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    let d = &v["dynamics"];
    let g = &v["gearbox"];
    let ratios = g["ratios"]
        .as_array()
        .filter(|a| !a.is_empty() && a.len() <= 254)
        .ok_or("expected 1..254 ratios")?
        .iter()
        .map(number)
        .collect::<Result<Vec<_>, _>>()?;
    if !ratios.windows(2).all(|p| p[0] > p[1]) {
        return Err("ratios must descend".into());
    }
    let stiffness = d["corner_stiffness"]
        .as_array()
        .filter(|a| a.len() == 2)
        .ok_or("expected two stiffness values")?;
    let spec = CarSpec {
        design: load_model(path, &v["model"])?,
        turbo: turbo(&v["turbo"])?,
        name: text(&v["name"])?,
        mass: number(&v["mass"])?,
        engine_force: number(&v["engine_force"])?,
        top_speed: number(&v["top_speed"])?,
        brake_force: number(&v["brake_force"])?,
        lateral_force: number(&v["lateral_force"])?,
        wheelbase: number(&v["wheelbase"])?,
        steering_response: number(&v["steering_response"])?,
        dynamics: Dynamics {
            front_weight: fraction(&d["front_weight"])?,
            cg_height: number(&d["cg_height"])?,
            track_width: number(&d["track_width"])?,
            yaw_inertia: number(&d["yaw_inertia"])?,
            front_drive: fraction(&d["front_drive"])?,
            brake_bias: fraction(&d["brake_bias"])?,
            peak_torque: number(&d["peak_torque"])?,
            suspension_hz: number(&d["suspension_hz"])?,
            steering_low_speed_squared: nonnegative(
                d.get("steering_low_speed_squared")
                    .unwrap_or(&d["steering_speed_squared"]),
            )?,
            steering_speed_squared: nonnegative(&d["steering_speed_squared"])?,
            corner_stiffness: [number(&stiffness[0])?, number(&stiffness[1])?],
        },
        gearbox: Gearbox {
            ratios: Box::leak(ratios.into_boxed_slice()),
            final_drive: number(&g["final_drive"])?,
            rolling_radius: number(&g["rolling_radius"])?,
            reverse_ratio: number(&g["reverse_ratio"])?,
            upshift_rpm: number(&g["upshift_rpm"])?,
            downshift_rpm: number(&g["downshift_rpm"])?,
        },
        road_engine: if v
            .get("road_engine")
            .ok_or("road_engine is required")?
            .is_null()
        {
            None
        } else {
            Some(RoadEngine {
                wheel_power_watts: number(&v["road_engine"]["wheel_power_watts"])?,
                drag_accel: number(&v["road_engine"]["drag_accel"])?,
            })
        },
    };
    if spec.gearbox.downshift_rpm >= spec.gearbox.upshift_rpm {
        return Err("invalid shift thresholds".into());
    }
    Ok(spec)
}

fn load_model(
    path: &std::path::Path,
    value: &serde_json::Value,
) -> Result<std::sync::Arc<crate::scene::car_glb::Design>, String> {
    let model = asset_path(path, value)?;
    crate::scene::car_glb::load(&model)
        .map(std::sync::Arc::new)
        .map_err(|e| e.to_string())
}

impl CarModel {
    /// Rear pair first, then front pair. Shared by artwork and tire feedback.
    pub fn wheel_centers(self) -> [[f32; 3]; 4] {
        self.spec().design.mounts
    }

    pub fn index(self) -> usize {
        ordered_models()
            .iter()
            .position(|model| *model == self)
            .unwrap()
    }
    fn folder_name(self) -> &'static str {
        match self {
            Self::Roadster => "kestrel",
            Self::Muscle => "muscle",
            Self::Club => "club",
            Self::Custom(i) => &extras()[i].folder,
        }
    }

    pub fn spec(self) -> &'static CarSpec {
        match self {
            Self::Roadster => &originals()[0],
            Self::Muscle => &originals()[1],
            Self::Club => &originals()[2],
            Self::Custom(i) => &extras()[i].spec,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalog_is_complete_and_physics_values_are_valid() {
        let names: Vec<_> = ALL.into_iter().map(CarModel::folder_name).collect();
        assert!(names.windows(2).all(|p| p[0] < p[1]));
        for (index, model) in ALL.into_iter().enumerate() {
            assert_eq!(model.index(), index);
            let s = model.spec();
            assert!(!s.name.is_empty());
            for value in [
                s.mass,
                s.engine_force,
                s.top_speed,
                s.brake_force,
                s.lateral_force,
                s.wheelbase,
                s.steering_response,
                s.gearbox.final_drive,
                s.gearbox.rolling_radius,
            ] {
                assert!(value.is_finite() && value > 0.0);
            }
            if let Some(engine) = &s.road_engine {
                assert!(engine.wheel_power_watts.is_finite() && engine.wheel_power_watts > 0.0);
                assert!(engine.drag_accel.is_finite() && engine.drag_accel > 0.0);
            }
            assert!((4..=6).contains(&s.gearbox.gears()));
            assert!(s.gearbox.ratios.iter().all(|r| r.is_finite() && *r > 0.0));
            assert!(s.gearbox.ratios.windows(2).all(|p| p[0] > p[1]));
            for gear in 1..=s.gearbox.gears() {
                let speed = s.gearbox.speed_at_rpm(6000.0, gear);
                assert!((s.gearbox.rpm(speed, gear) - 6000.0).abs() < 0.01);
            }
        }
    }
}

struct ExtraCar {
    folder: String,
    spec: CarSpec,
    base: CarModel,
}
impl CarModel {
    pub fn appearance(self) -> Self {
        match self {
            Self::Custom(i) => extras()[i].base,
            _ => self,
        }
    }
}
fn asset_path(
    definition: &std::path::Path,
    value: &serde_json::Value,
) -> Result<std::path::PathBuf, String> {
    let name = value
        .as_str()
        .filter(|s| !s.trim().is_empty())
        .ok_or("expected nonempty GLB model path")?;
    Ok(definition.parent().unwrap().join(name))
}

fn extras() -> &'static Vec<ExtraCar> {
    static CARS: std::sync::OnceLock<Vec<ExtraCar>> = std::sync::OnceLock::new();
    CARS.get_or_init(|| {
        let dir = std::path::Path::new("data/cars");
        let mut paths: Vec<_> = match std::fs::read_dir(dir) {
            Ok(entries) => entries
                .filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .filter(|p| {
                    !["kestrel", "muscle", "club"]
                        .iter()
                        .any(|id| p.file_name().is_some_and(|s| s == *id))
                })
                .map(|p| p.join("car.json"))
                .filter(|p| p.is_file())
                .collect(),
            Err(_) => return Vec::new(),
        };
        paths.sort();
        paths
            .into_iter()
            .filter_map(|path| match load_extra(&path) {
                Ok(car) => Some(car),
                Err(error) => {
                    log::warn!("Cannot load car {}: {error}", path.display());
                    None
                }
            })
            .collect()
    })
}
fn load_extra(path: &std::path::Path) -> Result<ExtraCar, String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let v: serde_json::Value = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    let object = v.as_object().ok_or("expected an object")?;
    let base = match v["base"].as_str().ok_or("base is required")? {
        "kestrel" => CarModel::Roadster,
        "muscle" => CarModel::Muscle,
        "club" => CarModel::Club,
        _ => return Err("unknown base".into()),
    };
    let mut spec = base.spec().clone();
    // Owned runtime strings/ratios live for the startup catalog's lifetime.
    if let Some(value) = object.get("name") {
        let text = value
            .as_str()
            .filter(|s| !s.trim().is_empty())
            .ok_or("expected nonempty text")?;
        spec.name = Box::leak(text.to_owned().into_boxed_str());
    }
    macro_rules! fields { ($obj:expr, $target:expr, $($field:ident),+) => {
        $(if let Some(value) = $obj.get(stringify!($field)) { $target.$field = number(value)?; })+
    }; }
    fields!(
        object,
        spec,
        mass,
        engine_force,
        top_speed,
        brake_force,
        lateral_force,
        wheelbase,
        steering_response
    );
    if let Some(d) = object.get("dynamics") {
        let d = d.as_object().ok_or("expected dynamics object")?;
        fields!(
            d,
            spec.dynamics,
            cg_height,
            track_width,
            yaw_inertia,
            peak_torque,
            suspension_hz
        );
        for (key, target) in [
            ("front_weight", &mut spec.dynamics.front_weight),
            ("front_drive", &mut spec.dynamics.front_drive),
            ("brake_bias", &mut spec.dynamics.brake_bias),
            (
                "steering_low_speed_squared",
                &mut spec.dynamics.steering_low_speed_squared,
            ),
            (
                "steering_speed_squared",
                &mut spec.dynamics.steering_speed_squared,
            ),
        ] {
            if let Some(v) = d.get(key) {
                let n = v.as_f64().ok_or("expected number")? as f32;
                if !n.is_finite() || n < 0.0 || (!key.starts_with("steering_") && n > 1.0) {
                    return Err("invalid dynamics value".into());
                }
                *target = n;
            }
        }
        if let Some(v) = d.get("corner_stiffness") {
            let a = v
                .as_array()
                .filter(|a| a.len() == 2)
                .ok_or("expected two stiffness values")?;
            spec.dynamics.corner_stiffness = [number(&a[0])?, number(&a[1])?];
        }
    }
    if let Some(g) = object.get("gearbox") {
        let g = g.as_object().ok_or("expected gearbox object")?;
        fields!(
            g,
            spec.gearbox,
            final_drive,
            rolling_radius,
            reverse_ratio,
            upshift_rpm,
            downshift_rpm
        );
        if let Some(r) = g.get("ratios") {
            let r = r
                .as_array()
                .filter(|r| !r.is_empty() && r.len() <= 254)
                .ok_or("expected 1..254 ratios")?;
            let ratios = r.iter().map(number).collect::<Result<Vec<_>, _>>()?;
            if !ratios.windows(2).all(|p| p[0] > p[1]) {
                return Err("ratios must descend".into());
            }
            spec.gearbox.ratios = Box::leak(ratios.into_boxed_slice());
        }
        if spec.gearbox.downshift_rpm >= spec.gearbox.upshift_rpm {
            return Err("invalid shift thresholds".into());
        }
    }
    if let Some(e) = object.get("road_engine") {
        spec.road_engine = if e.is_null() {
            None
        } else {
            Some(RoadEngine {
                wheel_power_watts: number(&e["wheel_power_watts"])?,
                drag_accel: number(&e["drag_accel"])?,
            })
        };
    }
    if let Some(value) = object.get("turbo") {
        spec.turbo = turbo(value)?;
    }
    if let Some(model) = object.get("model") {
        spec.design = load_model(path, model)?;
    }
    Ok(ExtraCar {
        spec,
        base,
        folder: path
            .parent()
            .and_then(|p| p.file_name())
            .ok_or("car needs a folder")?
            .to_string_lossy()
            .into_owned(),
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
        assert_eq!(ALL.len(), 4);
        assert_eq!(CarModel::Club.spec().mass, 999.0);
        let car = ALL[3];
        assert_eq!(car.index(), 3);
        assert_eq!(car.spec().name, "NEW CAR");
        assert_eq!(car.spec().mass, 1234.0);
        assert_eq!(car.spec().gearbox.ratios, &[3.0, 2.0, 1.0]);
        assert_eq!(car.wheel_centers()[0][0], -0.99);
        let geometry = crate::scene::cars_only();
        assert_eq!(geometry.car_ranges.len(), 4);
        assert_eq!(
            geometry.vertices[geometry.car_ranges[3].start as usize].position[0],
            7.25
        );
        assert!(!geometry.car_ranges[3].is_empty());
        assert_eq!(
            geometry.vertices[geometry.wheel_range[3][0].start as usize].position[0],
            0.123
        );
        assert_eq!(
            crate::hud::selection_menu_hit_test(0.5, -0.07, 0, 4, car),
            Some(crate::hud::MenuAction::Car(car))
        );
    }

    #[test]
    fn glb_definitions_support_inheritance_replacement_and_model_only_assets() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("car.json");
        std::fs::copy(
            "data/cars/club/model.glb",
            dir.path().join("replacement.glb"),
        )
        .unwrap();
        std::fs::write(&path, r#"{"base":"club", "mass":950}"#).unwrap();
        let inherited = load_extra(&path).unwrap().spec;
        assert!(std::sync::Arc::ptr_eq(
            &inherited.design,
            &CarModel::Club.spec().design
        ));
        // A replacement GLB supplies its own wheel mounts.
        std::fs::write(&path, r#"{"base":"club", "model":"replacement.glb"}"#).unwrap();
        let replacement = load_extra(&path).unwrap().spec;
        assert_eq!(
            replacement.design.mounts[0][0],
            CarModel::Club.wheel_centers()[0][0]
        );
        let mut definition: serde_json::Value =
            serde_json::from_slice(&std::fs::read("data/cars/club/car.json").unwrap()).unwrap();
        definition["model"] = serde_json::json!("replacement.glb");
        std::fs::write(&path, serde_json::to_vec(&definition).unwrap()).unwrap();
        assert!(!load_original(&path).unwrap().design.body.is_empty());
        std::fs::write(dir.path().join("replacement.glb"), b"broken").unwrap();
        let error = load_original(&path).err().unwrap();
        assert!(
            error.contains("replacement.glb") && error.contains("binary glTF"),
            "{error}"
        );
    }

    #[test]
    fn gearbox_limit_keeps_neutral_distinct() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("car.json");
        let mut original: serde_json::Value =
            serde_json::from_slice(&std::fs::read("data/cars/club/car.json").unwrap()).unwrap();
        original["model"] =
            serde_json::json!(std::fs::canonicalize("data/cars/club/model.glb").unwrap());
        for count in [254, 255] {
            let ratios: Vec<_> = (1..=count).rev().collect();
            let mut required = original.clone();
            required["gearbox"]["ratios"] = serde_json::json!(ratios);
            std::fs::write(&path, serde_json::to_vec(&required).unwrap()).unwrap();
            assert_eq!(load_original(&path).is_ok(), count == 254);
            std::fs::write(
                &path,
                serde_json::to_vec(&serde_json::json!({
                    "base":"club", "gearbox":{"ratios":ratios}
                }))
                .unwrap(),
            )
            .unwrap();
            assert_eq!(load_extra(&path).is_ok(), count == 254);
        }
    }

    #[test]
    fn required_parameters_are_validated() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("car.json");
        let mut original: serde_json::Value =
            serde_json::from_slice(&std::fs::read("data/cars/club/car.json").unwrap()).unwrap();
        original["model"] =
            serde_json::json!(std::fs::canonicalize("data/cars/club/model.glb").unwrap());
        for (key, value) in [
            ("mass", serde_json::json!(-1)),
            ("model", serde_json::Value::Null),
            ("gearbox", serde_json::json!({"ratios": []})),
        ] {
            let mut invalid = original.clone();
            invalid[key] = value;
            std::fs::write(&path, serde_json::to_vec(&invalid).unwrap()).unwrap();
            assert!(load_original(&path).is_err(), "accepted invalid {key}");
        }
    }

    #[test]
    fn new_car_is_discovered_by_existing_binary() {
        let dir = tempfile::tempdir().unwrap();
        let cars = dir.path().join("data/cars");
        std::fs::create_dir_all(&cars).unwrap();
        for id in ["kestrel", "muscle", "club"] {
            let folder = cars.join(id);
            std::fs::create_dir_all(&folder).unwrap();
            std::fs::copy(
                format!("data/cars/{id}/model.glb"),
                folder.join("model.glb"),
            )
            .unwrap();
        }

        for id in ["kestrel", "muscle", "club"] {
            let mut v: serde_json::Value =
                serde_json::from_slice(&std::fs::read(format!("data/cars/{id}/car.json")).unwrap())
                    .unwrap();
            if id == "club" {
                v["mass"] = serde_json::json!(999);
            }
            std::fs::write(
                cars.join(id).join("car.json"),
                serde_json::to_vec(&v).unwrap(),
            )
            .unwrap();
        }

        std::fs::create_dir_all(cars.join("new")).unwrap();
        // Edit the GLB in a runtime-only package to exercise geometry and pivots.
        let mut asset = std::fs::read("data/cars/club/model.glb").unwrap();
        let parsed = gltf::Gltf::from_slice(&asset).unwrap();
        let json_len = u32::from_le_bytes(asset[12..16].try_into().unwrap()) as usize;
        let binary_start = 28 + json_len;
        let mut edits = Vec::new();
        for (mesh, value) in parsed.meshes().zip([7.25_f32, 0.123]) {
            let primitive = mesh.primitives().next().unwrap();
            let first = primitive
                .reader(|_| parsed.blob.as_deref())
                .read_indices()
                .unwrap()
                .into_u32()
                .next()
                .unwrap() as usize;
            let positions = primitive.get(&gltf::Semantic::Positions).unwrap();
            let view = positions.view().unwrap();
            edits.push((
                binary_start
                    + view.offset()
                    + positions.offset()
                    + first * view.stride().unwrap_or(positions.size()),
                value,
            ));
        }
        drop(parsed);
        for (offset, value) in edits {
            asset[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        let mut document: serde_json::Value =
            serde_json::from_slice(&asset[20..20 + json_len]).unwrap();
        for node in document["nodes"].as_array_mut().unwrap() {
            if node["name"] == "wheel_rl" {
                node["translation"][0] = serde_json::json!(-0.99);
            }
        }
        let mut json = serde_json::to_vec(&document).unwrap();
        while !json.len().is_multiple_of(4) {
            json.push(b' ');
        }
        let mut replacement = Vec::new();
        for value in [
            0x46546c67_u32,
            2,
            (28 + json.len() + asset.len() - binary_start) as u32,
            json.len() as u32,
            0x4e4f534a,
        ] {
            replacement.extend(value.to_le_bytes());
        }
        replacement.extend(json);
        replacement.extend_from_slice(&asset[binary_start - 8..]);
        std::fs::write(cars.join("new/model.glb"), replacement).unwrap();
        std::fs::create_dir_all(cars.join("bad")).unwrap();
        std::fs::create_dir_all(cars.join("notes")).unwrap();
        std::fs::write(cars.join("ignored.json"), "{}").unwrap();
        std::fs::write(
            cars.join("new/car.json"),
            r#"{
            "base": "club", "name": "NEW CAR", "mass": 1234,
            "model": "model.glb", "gearbox": {"ratios": [3, 2, 1]}
        }"#,
        )
        .unwrap();
        std::fs::write(cars.join("bad/car.json"), r#"{"base":"club","mass":-1}"#).unwrap();
        for (folder, contents) in [
            ("missing_model", None),
            ("corrupt_model", Some(b"invalid model".as_slice())),
        ] {
            let folder = cars.join(folder);
            std::fs::create_dir_all(&folder).unwrap();
            if let Some(bytes) = contents {
                std::fs::write(folder.join("bad.glb"), bytes).unwrap();
            }
            std::fs::write(
                folder.join("car.json"),
                serde_json::to_vec(&serde_json::json!({"base":"club", "model": "bad.glb"}))
                    .unwrap(),
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
