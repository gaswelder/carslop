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

#[test]
fn published_forces_and_torque_preserve_contact_impulses_and_the_input_motion() {
    let spec = test::support::car();
    let mut state = super::State::default();
    state.in_motion = tire_solver::Motion {
        longitudinal: 8.0,
        lateral: 1.0,
        yaw_rate: 0.2,
    };
    state.in_free_motion = state.in_motion;
    state.in_contacts = [1.2, -1.3].map(|offset| tire_solver::Contact {
        offset,
        sin: 0.0,
        cos: 1.0,
        capacity: 6000.0,
        drive_force: 800.0,
        brake_force: 0.0,
        corner_stiffness: 60000.0,
        longitudinal_slip: 0.0,
        locked: false,
    });
    super::update(&mut state, &spec, 1.0 / 240.0);
    let forward = state.out_axle_force[0].longitudinal + state.out_axle_force[1].longitudinal;
    let side = state.out_axle_force[0].lateral + state.out_axle_force[1].lateral;
    let yaw = 1.2 * state.out_axle_force[0].lateral - 1.3 * state.out_axle_force[1].lateral;
    assert!((state.out_force.longitudinal - forward).abs() < 0.01);
    assert!((state.out_force.lateral - side).abs() < 0.01);
    assert!((state.out_torque - yaw).abs() < 0.01);
    assert_eq!(state.in_motion.longitudinal, 8.0);
    assert_eq!(state.in_motion.lateral, 1.0);
    assert_eq!(state.in_motion.yaw_rate, 0.2);
}
