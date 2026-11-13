//! Lightweight analytic contact footprints, evaluated on actual road fragments.
//! This is ambient grounding, not directional shadow mapping or physics.

use crate::geometry::XYZ;
use crate::vehicle::Vehicle;

pub const MAX_CONTACTS: usize = 2;
pub const STRENGTH: f32 = 0.42;

#[derive(Clone, Copy, Debug)]
pub struct ContactShadow {
    pub origin: XYZ,
    // Heading sin/cos followed by lateral/longitudinal soft footprint radii.
    pub axes: [f32; 4],
}

impl ContactShadow {
    pub fn new(car: &Vehicle) -> Self {
        let wheel = car.model.wheel_centers()[0];
        Self {
            origin: car.position,
            axes: [
                car.heading.sin(),
                car.heading.cos(),
                wheel.x.abs() + 0.45,
                car.model.spec().wheelbase * 0.5 + 1.05,
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track;

    #[test]
    fn footprint_contains_wheel_contacts() {
        let road = track::LocationId::Compact
            .try_track()
            .unwrap()
            .at_distance(50.0);
        let model = crate::cars::CarModel::TestFixture(0);
        let car = crate::test_support::vehicle(road.vehicle_position(0.0), road.heading(), model);
        let shadow = ContactShadow::new(&car);
        assert_eq!(shadow.origin, car.position);
        assert!(shadow.axes.iter().all(|v| v.is_finite()));
        for wheel in model.wheel_centers() {
            assert!(wheel.x.abs() < shadow.axes[2]);
            assert!(wheel.z.abs() < shadow.axes[3] - 0.3);
        }
    }
}
