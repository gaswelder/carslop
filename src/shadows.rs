//! Lightweight analytic contact footprints, evaluated on actual road fragments.
//! This is ambient grounding, not directional shadow mapping or physics.

use crate::vehicle::Vehicle;

pub const MAX_CONTACTS: usize = 2;
pub const STRENGTH: f32 = 0.42;

#[derive(Clone, Copy, Debug)]
pub struct ContactShadow {
    pub origin: [f32; 3],
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
                wheel[0].abs() + 0.45,
                car.model.spec().wheelbase * 0.5 + 1.05,
            ],
        }
    }

    // CPU reference for the small footprint/height envelope in SCENE_FS.
    #[cfg(test)]
    fn opacity_at(&self, point: [f32; 3]) -> f32 {
        let dx = point[0] - self.origin[0];
        let dz = point[2] - self.origin[2];
        let [s, c, width, length] = self.axes;
        let x = (dx * c - dz * s) / width;
        let z = (dx * s + dz * c) / length;
        let smooth = |a: f32, b: f32, value: f32| {
            let t = ((value - a) / (b - a)).clamp(0.0, 1.0);
            t * t * (3.0 - 2.0 * t)
        };
        let soft = 1.0 - smooth(0.16, 1.0, x * x + z * z);
        let height = 1.0 - smooth(0.3, 1.2, (point[1] + 0.43 - self.origin[1]).abs());
        STRENGTH * soft * height
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{cars, track};

    #[test]
    fn footprints_are_finite_bounded_and_match_each_car() {
        let road = track::TrackId::Compact.track().at_distance(50.0);
        let footprints = cars::ALL.map(|model| {
            let car = Vehicle::new(road.lane_position(0.0), road.heading(), model);
            let shadow = ContactShadow::new(&car);
            assert!(shadow
                .origin
                .iter()
                .chain(shadow.axes.iter())
                .all(|x| x.is_finite()));
            assert!(shadow.axes[2] > 1.0 && shadow.axes[2] < 1.7);
            assert!(shadow.axes[3] > 2.0 && shadow.axes[3] < 2.7);
            assert_eq!(shadow.origin, car.position);
            shadow.axes
        });
        // The new cars use different track widths; every contact must still
        // fit inside the projected footprint, regardless of model ordering.
        for model in cars::ALL {
            for wheel in model.wheel_centers() {
                assert!(wheel[0].abs() < footprints[model.index()][2]);
                assert!(wheel[2].abs() < footprints[model.index()][3] - 0.3);
            }
        }
        assert!(
            footprints[cars::CarModel::Muscle.index()][3]
                > footprints[cars::CarModel::Roadster.index()][3]
                && footprints[cars::CarModel::Roadster.index()][3]
                    > footprints[cars::CarModel::Club.index()][3]
        );
    }

    #[test]
    fn soft_contact_rotates_with_car_and_fades_out_at_edges_and_height() {
        let mut car = Vehicle::new([10.0, 3.48, -7.0], 0.0, cars::CarModel::Roadster);
        for heading in [0.0, 0.7, 1.8, 4.5] {
            car.heading = heading;
            let shadow = ContactShadow::new(&car);
            let point = |across: f32| {
                [
                    car.position[0] + heading.cos() * across,
                    3.05,
                    car.position[2] - heading.sin() * across,
                ]
            };
            let center = shadow.opacity_at(point(0.0));
            let middle = shadow.opacity_at(point(shadow.axes[2] * 0.7));
            let edge = shadow.opacity_at(point(shadow.axes[2]));
            assert!((center - STRENGTH).abs() < 0.0001);
            assert!(middle > 0.0 && middle < center);
            assert!(edge < 0.0001);
            assert_eq!(
                shadow.opacity_at([car.position[0], 0.0, car.position[2]]),
                0.0
            );
            assert_eq!(
                shadow.opacity_at([car.position[0], 6.0, car.position[2]]),
                0.0
            );
        }
    }

    #[test]
    fn contacts_follow_elevated_route_and_recovery_without_history() {
        for model in cars::ALL {
            for i in 0..120 {
                let road = track::TrackId::Compact
                    .track()
                    .at_distance(i as f32 * track::TrackId::Compact.track().length() / 120.0);
                let mut car = Vehicle::new(road.lane_position(1.4), road.heading(), model);
                let shadow = ContactShadow::new(&car);
                let p = [car.position[0], road.point[1] + 0.05, car.position[2]];
                assert!((shadow.opacity_at(p) - STRENGTH).abs() < 0.0001);
                let elsewhere = track::TrackId::Compact
                    .track()
                    .at_distance(road.distance + track::TrackId::Compact.track().length() * 0.5);
                car.reset(elsewhere.lane_position(0.0), elsewhere.heading());
                assert_eq!(ContactShadow::new(&car).opacity_at(p), 0.0);
            }
        }
    }
}
