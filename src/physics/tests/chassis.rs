use super::*;
use crate::test;

#[test]
fn aerodynamic_drag_uses_coefficient_area_and_mass() {
    let mut spec = test::support::car_spec().clone();
    spec.drag_coefficient = 0.3;
    spec.frontal_area_m2 = 2.0;
    spec.weight_kg = 1000.0;
    // At 20 m/s this gives 147 N of aerodynamic drag, plus rolling resistance.
    let drag = resistance_force(&spec, 20.0) - resistance_force(&spec, 0.0);
    assert!((drag - 147.0).abs() < 0.0001);
    assert_eq!(resistance_force(&spec, 0.0), 250.0);
    assert_eq!(
        resistance_force(&(spec), -20.0),
        resistance_force(&(spec), 20.0)
    );
    spec.weight_kg *= 2.0;
    let heavier = resistance_force(&spec, 20.0) - resistance_force(&spec, 0.0);
    assert!((heavier - drag).abs() < 0.0001);
    assert_eq!(resistance_force(&spec, 0.0), 500.0);
    spec.drag_coefficient *= 2.0;
    let more_drag = resistance_force(&spec, 20.0) - resistance_force(&spec, 0.0);
    assert!((more_drag - 2.0 * drag).abs() < 0.0001);
    spec.frontal_area_m2 *= 2.0;
    let larger_area = resistance_force(&spec, 20.0) - resistance_force(&spec, 0.0);
    assert!((larger_area - 4.0 * drag).abs() < 0.0001);
}

#[test]
fn force_and_torque_inputs_advance_only_the_chassis() {
    let spec = test::support::car();
    let mut state = new(XYZ::default(), 0.0);
    state.in_force = XYZ {
        x: 1200.0,
        y: -2400.0,
        z: 0.0,
    };
    state.in_torque.yaw = 300.0;
    let dt = 1.0 / 240.0;
    update(&mut state, &spec, dt);
    assert!((state.velocity.x * spec.weight_kg - 1200.0 * dt).abs() < 0.00001);
    assert!((state.velocity.y * spec.weight_kg + 2400.0 * dt).abs() < 0.00001);
    assert!((state.angular_velocity.yaw * spec.yaw_inertia_kg_m2 - 300.0 * dt).abs() < 0.00001);
    assert_eq!(state.velocity.z, 0.0);
}
