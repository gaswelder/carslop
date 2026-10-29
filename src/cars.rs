//! Shared original car catalog: selection, rendering, physics, and AI use one identity.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CarModel {
    Roadster,
    Muscle,
    Club,
}

pub const ALL: [CarModel; 3] = [CarModel::Roadster, CarModel::Muscle, CarModel::Club];
pub const WHEEL_RADIUS: f32 = 0.4;

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
    pub gearbox: Gearbox,
    pub road_engine: Option<RoadEngine>,
}

/// Car-specific wheel power and aerodynamic resistance, independent of map.
/// Acceleration remains launch-force limited; this is not a full torque curve.
pub struct RoadEngine {
    pub wheel_power_watts: f32,
    pub drag_accel: f32,
}

/// Ratios and effective rolling radius determine engine RPM from road speed.
/// Sources and game-specific shift thresholds are recorded in GEARBOXES.md.
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
        }
    }

    pub fn spec(self) -> &'static CarSpec {
        match self {
            Self::Roadster => &ROADSTER,
            Self::Muscle => &MUSCLE,
            Self::Club => &CLUB,
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
