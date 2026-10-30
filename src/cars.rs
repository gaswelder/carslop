//! Shared original car catalog: selection, rendering, physics, and AI use one identity.

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
        [CarModel::Roadster, CarModel::Muscle, CarModel::Club]
            .into_iter()
            .chain((0..extras().len()).map(CarModel::Custom))
            .collect::<Vec<_>>()
            .into_iter()
    }
}
impl std::ops::Index<usize> for Catalog {
    type Output = CarModel;
    fn index(&self, index: usize) -> &CarModel {
        static MODELS: std::sync::OnceLock<Vec<CarModel>> = std::sync::OnceLock::new();
        &MODELS.get_or_init(|| ALL.into_iter().collect())[index]
    }
}
pub const WHEEL_RADIUS: f32 = 0.4;

#[derive(Clone)]
pub struct CarSpec {
    pub name: &'static str,
    pub description: &'static str,
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

const ROADSTER: CarSpec = CarSpec {
    name: "KESTREL",
    description: "REAR-ENGINE HERITAGE COUPE",
    mass: 1000.0,
    engine_force: 9000.0,
    top_speed: 210.0 / 3.6,
    brake_force: 20000.0,
    lateral_force: 12000.0,
    wheelbase: 2.6,
    steering_response: 8.0,
    // 96 kW reference engine, approximate 90% driveline efficiency. Drag is
    // gameplay calibration for a ~210 km/h power/road-load equilibrium.
    road_engine: Some(RoadEngine {
        wheel_power_watts: 86_400.0,
        drag_accel: 0.00036,
    }),
    dynamics: Dynamics {
        front_weight: 0.39,
        cg_height: 0.48,
        track_width: 1.46,
        yaw_inertia: 1450.0,
        front_drive: 0.0,
        brake_bias: 0.61,
        corner_stiffness: [56000.0, 105000.0],
        peak_torque: 155.0,
        suspension_hz: 1.65,
        // Keyboard full lock at city speed should not demand a parking turn.
        steering_speed_squared: 0.01,
    },
    gearbox: Gearbox {
        // Original 1964–65 911, 901/0 (AA / GA / O / U / Y).
        ratios: &[2.833, 1.778, 1.217, 0.962, 0.821],
        final_drive: 31.0 / 7.0,
        rolling_radius: 0.315,
        reverse_ratio: 3.094,
        upshift_rpm: 6200.0,
        downshift_rpm: 2600.0,
    },
};
const MUSCLE: CarSpec = CarSpec {
    name: "MUSCLE",
    description: "HEAVY GT - POWER AND SLIDE",
    mass: 1450.0,
    engine_force: 15950.0,
    top_speed: 29.0,
    brake_force: 26100.0,
    lateral_force: 14500.0,
    wheelbase: 2.9,
    steering_response: 6.8,
    road_engine: None,
    dynamics: Dynamics {
        front_weight: 0.55,
        cg_height: 0.55,
        track_width: 1.62,
        yaw_inertia: 2800.0,
        front_drive: 0.0,
        brake_bias: 0.65,
        corner_stiffness: [78000.0, 82000.0],
        peak_torque: 420.0,
        suspension_hz: 1.45,
        steering_speed_squared: 0.0,
    },
    gearbox: Gearbox {
        // TREMEC Magnum close-ratio six-speed, Ford TUET11010.
        ratios: &[2.66, 1.78, 1.30, 1.0, 0.80, 0.63],
        final_drive: 3.73,
        rolling_radius: 0.335,
        reverse_ratio: 2.90,
        upshift_rpm: 5500.0,
        downshift_rpm: 2200.0,
    },
};
const CLUB: CarSpec = CarSpec {
    name: "CLUB",
    description: "LIGHT HATCH - GRIP AND BRAKES",
    mass: 850.0,
    engine_force: 7225.0,
    top_speed: 22.5,
    brake_force: 18700.0,
    lateral_force: 11900.0,
    wheelbase: 2.3,
    steering_response: 10.0,
    road_engine: None,
    dynamics: Dynamics {
        front_weight: 0.63,
        cg_height: 0.48,
        track_width: 1.40,
        yaw_inertia: 950.0,
        front_drive: 1.0,
        brake_bias: 0.68,
        corner_stiffness: [58000.0, 43000.0],
        peak_torque: 102.0,
        suspension_hz: 1.85,
        steering_speed_squared: 0.0,
    },
    gearbox: Gearbox {
        // Volkswagen Golf Mk1 020 FF, wide-ratio five-speed / economy fifth.
        ratios: &[
            38.0 / 11.0,
            35.0 / 18.0,
            36.0 / 28.0,
            30.0 / 33.0,
            27.0 / 38.0,
        ],
        final_drive: 74.0 / 19.0,
        rolling_radius: 1.75 / std::f32::consts::TAU,
        reverse_ratio: 38.0 / 12.0,
        upshift_rpm: 6000.0,
        downshift_rpm: 2400.0,
    },
};

impl CarModel {
    /// Rear pair first, then front pair. Shared by artwork and tire feedback.
    pub fn wheel_centers(self) -> [[f32; 3]; 4] {
        let half_track = match self {
            Self::Roadster => 0.88,
            Self::Muscle => 1.13,
            Self::Club => 0.95,
            Self::Custom(i) => extras()[i].half_track,
        };
        let axle = self.spec().wheelbase * 0.5;
        [
            [-half_track, -0.02, -axle],
            [half_track, -0.02, -axle],
            [-half_track, -0.02, axle],
            [half_track, -0.02, axle],
        ]
    }

    pub fn index(self) -> usize {
        match self {
            Self::Roadster => 0,
            Self::Muscle => 1,
            Self::Club => 2,
            Self::Custom(i) => i + 3,
        }
    }

    pub fn spec(self) -> &'static CarSpec {
        match self {
            Self::Roadster => &ROADSTER,
            Self::Muscle => &MUSCLE,
            Self::Club => &CLUB,
            Self::Custom(i) => &extras()[i].spec,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalog_is_complete_and_physics_values_are_valid() {
        for (index, model) in ALL.into_iter().enumerate() {
            assert_eq!(model.index(), index);
            let s = model.spec();
            assert!(!s.name.is_empty() && !s.description.is_empty());
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
    spec: CarSpec,
    base: CarModel,
    half_track: f32,
    mesh: Option<std::path::PathBuf>,
}
impl CarModel {
    pub fn appearance(self) -> Self {
        match self {
            Self::Custom(i) => extras()[i].base,
            _ => self,
        }
    }
    pub fn mesh(self) -> Option<&'static std::path::Path> {
        match self {
            Self::Custom(i) => extras()[i].mesh.as_deref(),
            _ => None,
        }
    }
}
fn extras() -> &'static Vec<ExtraCar> {
    static CARS: std::sync::OnceLock<Vec<ExtraCar>> = std::sync::OnceLock::new();
    CARS.get_or_init(|| {
        let dir = std::path::Path::new("data/cars");
        let mut paths: Vec<_> = match std::fs::read_dir(dir) {
            Ok(entries) => entries
                .filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.extension().is_some_and(|e| e == "json"))
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
    for (key, target) in [
        ("name", &mut spec.name),
        ("description", &mut spec.description),
    ] {
        if let Some(value) = object.get(key) {
            let text = value
                .as_str()
                .filter(|s| !s.trim().is_empty())
                .ok_or("expected nonempty text")?;
            *target = Box::leak(text.to_owned().into_boxed_str());
        }
    }
    fn number(v: &serde_json::Value) -> Result<f32, String> {
        let n = v.as_f64().ok_or("expected number")? as f32;
        if !n.is_finite() || n <= 0.0 {
            return Err("expected finite positive number".into());
        }
        Ok(n)
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
                "steering_speed_squared",
                &mut spec.dynamics.steering_speed_squared,
            ),
        ] {
            if let Some(v) = d.get(key) {
                let n = v.as_f64().ok_or("expected number")? as f32;
                if !n.is_finite() || n < 0.0 || (key != "steering_speed_squared" && n > 1.0) {
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
                .filter(|r| !r.is_empty() && r.len() <= 255)
                .ok_or("expected 1..255 ratios")?;
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
    let half_track = object
        .get("half_track")
        .map(number)
        .transpose()?
        .unwrap_or(base.wheel_centers()[0][0].abs());
    let mesh = object
        .get("mesh")
        .map(|v| {
            v.as_str()
                .map(|s| path.parent().unwrap().join(s))
                .ok_or("expected mesh path")
        })
        .transpose()?;
    Ok(ExtraCar {
        spec,
        base,
        half_track,
        mesh,
    })
}

#[cfg(test)]
mod runtime_tests {
    use super::*;

    #[test]
    fn runtime_catalog_probe() {
        if std::env::var_os("NFS_TEST_RUNTIME_CAR").is_none() {
            return;
        }
        assert_eq!(ALL.len(), 4);
        let car = ALL[3];
        assert_eq!(car.index(), 3);
        assert_eq!(car.spec().name, "NEW CAR");
        assert_eq!(car.spec().mass, 1234.0);
        assert_eq!(car.spec().gearbox.ratios, &[3.0, 2.0, 1.0]);
        assert_eq!(car.wheel_centers()[0][0], -0.99);
        let geometry = crate::scene::cars_only();
        assert_eq!(geometry.car_ranges.len(), 4);
        assert!(!geometry.car_ranges[3].is_empty());
        assert_eq!(
            crate::hud::menu_hit_test(-0.08, -0.2),
            Some(crate::hud::MenuAction::Car(car))
        );
    }

    #[test]
    fn new_car_is_discovered_by_existing_binary() {
        let dir = tempfile::tempdir().unwrap();
        let cars = dir.path().join("data/cars");
        std::fs::create_dir_all(&cars).unwrap();
        std::fs::write(
            cars.join("new.json"),
            r#"{
            "base": "club", "name": "NEW CAR", "mass": 1234,
            "half_track": 0.99, "gearbox": {"ratios": [3, 2, 1]}
        }"#,
        )
        .unwrap();
        std::fs::write(cars.join("bad.json"), r#"{"base":"club","mass":-1}"#).unwrap();
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .current_dir(dir.path())
            .env("NFS_TEST_RUNTIME_CAR", "1")
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
