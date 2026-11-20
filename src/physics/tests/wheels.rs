use super::*;
use crate::test;

#[test]
fn torque_and_road_reaction_exchange_angular_momentum_in_both_directions() {
    let spec = test::support::car();
    for direction in [-1.0, 1.0] {
        let mut state = new();
        state.omega = direction * 20.0;
        state.in_engaged = true;
        state.in_torque = direction * 120.0;
        state.in_force = direction * 250.0;
        state.in_brake_torque = -direction * 10.0;
        let before = state.omega * INERTIA_KG_M2;
        let dt = 1.0 / 240.0;
        update(&mut state, &spec, dt);
        let expected = direction * (120.0 - 250.0 * spec.chassis.rolling_radius - 10.0) * dt;
        assert!((state.omega * INERTIA_KG_M2 - before - expected).abs() < 0.00001);
    }
}

#[test]
fn open_clutch_wheels_follow_the_road_without_using_stale_drive_torque() {
    let spec = test::support::car();
    let mut state = new();
    state.omega = 100.0;
    state.in_torque = 1000.0;
    state.in_speed = -4.0;
    update(&mut state, &spec, 1.0 / 240.0);
    assert_eq!(state.omega, -4.0 / spec.chassis.rolling_radius);
}

#[test]
fn zero_duration_preserves_rotation_even_with_open_clutch() {
    let mut state = new();
    state.omega = 20.0;
    state.in_speed = -4.0;
    update(&mut state, &test::support::car(), 0.0);
    assert_eq!(state.omega, 20.0);
    assert_eq!(state.phase, [0.0; 2]);
}
