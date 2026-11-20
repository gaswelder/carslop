//! Lightweight analytic contact footprints, evaluated on actual road fragments.
//! This is ambient grounding, not directional shadow mapping or physics.

use crate::physics::car::Vehicle;
use crate::util::geometry::XYZ;

pub const MAX_CONTACTS: usize = 2;
pub const STRENGTH: f32 = 0.42;

#[derive(Clone, Copy, Debug)]
pub struct ContactShadow {
    pub origin: XYZ,
    // Heading sin/cos followed by lateral/longitudinal soft footprint radii.
    pub axes: [f32; 4],
}

pub fn new(car: &Vehicle) -> ContactShadow {
    let wheel = car.spec.design.mounts[0];
    ContactShadow {
        origin: car.chassis.position,
        axes: [
            car.chassis.orientation.yaw.sin(),
            car.chassis.orientation.yaw.cos(),
            wheel.x.abs() + 0.45,
            car.spec.chassis.wheelbase * 0.5 + 1.05,
        ],
    }
}
