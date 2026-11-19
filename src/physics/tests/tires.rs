use super::tire_contacts;
use crate::physics::car;
use crate::physics::suspension;
use crate::physics::tire_solver;
use crate::test;
use crate::util::geometry::XYZ;

#[test]
fn cornering_stiffness_follows_current_axle_load_independently_of_surface_grip() {
    let mut vehicle = car::on_track(
        XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        0.0,
        test::support::car(),
        test::support::compact_track(),
    );
    let contacts = |vehicle: &car::Vehicle, capacities| {
        tire_contacts(
            vehicle,
            car::Controls::default(),
            tire_solver::Motion::default(),
            capacities,
            0.0,
            false,
        )
    };
    vehicle.suspension.axle_load = suspension::AxleLoad {
        front: 4000.0,
        rear: 4000.0,
    };
    let balanced = contacts(&vehicle, [6000.0, 6000.0]);
    assert!(balanced[0].corner_stiffness > 0.0);
    assert_eq!(balanced[0].corner_stiffness, balanced[1].corner_stiffness);

    vehicle.suspension.axle_load = suspension::AxleLoad {
        front: 8000.0,
        rear: 2000.0,
    };
    let transferred = contacts(&vehicle, [6000.0, 6000.0]);
    assert_eq!(
        transferred[0].corner_stiffness,
        balanced[0].corner_stiffness * 2.0
    );
    assert_eq!(
        transferred[1].corner_stiffness,
        balanced[1].corner_stiffness * 0.5
    );
    let slippery = contacts(&vehicle, [600.0, 600.0]);
    for axle in 0..2 {
        assert_eq!(
            slippery[axle].corner_stiffness,
            transferred[axle].corner_stiffness
        );
        assert!(slippery[axle].capacity < transferred[axle].capacity);
    }
}
