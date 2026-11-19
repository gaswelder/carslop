use super::*;
use crate::physics::car;
use crate::physics::powertrain;
use crate::test;
use crate::util::geometry::XYZ;

fn vehicle() -> car::Vehicle {
    car::on_track(
        XYZ::default(),
        0.0,
        test::support::car(),
        test::support::compact_track(),
    )
}

#[test]
fn full_throttle_publishes_crankshaft_torque_without_transmission_losses() {
    let spec = test::support::car().engine.clone();
    let mut engine = new(&spec);
    engine.rpm = 3500.0;
    update(&mut engine, 1.0);
    let expected = spec
        .max_torque_nm
        .min(spec.power_watts / (3500.0 * std::f32::consts::TAU / 60.0));
    assert!((engine.torque_nm - expected).abs() < 0.0001);
}

#[test]
fn output_is_published_before_rotation_and_held_for_the_step() {
    let mut engine = new(&test::support::car().engine);
    engine.rpm = 3000.0;

    update(&mut engine, 1.0);
    let torque = engine.torque_nm;
    assert!(torque > 0.0);
    assert_eq!(engine.rpm, 3000.0);

    advance(&mut engine, 0.0, 1.0 / 240.0);
    assert!(engine.rpm > 3000.0);
    assert_eq!(engine.torque_nm, torque);

    let rpm = engine.rpm;
    update(&mut engine, 0.0);
    assert!(engine.torque_nm < 0.0);
    assert_eq!(engine.rpm, rpm);
    advance(&mut engine, 0.0, 1.0 / 240.0);
    assert!(engine.rpm < rpm);
}

#[test]
fn greater_inertia_reduces_rpm_response_to_torque_and_impulse() {
    let mut spec = test::support::car().engine.clone();
    spec.inertia_kg_m2 = 0.2;
    let mut light = new(&spec);
    spec.inertia_kg_m2 = 0.4;
    let mut heavy = new(&spec);
    light.rpm = 3000.0;
    heavy.rpm = 3000.0;

    update(&mut light, 1.0);
    advance(&mut light, 1.0, 1.0 / 240.0);
    update(&mut heavy, 1.0);
    advance(&mut heavy, 1.0, 1.0 / 240.0);

    assert!(light.rpm > 3000.0);
    assert!(((light.rpm - 3000.0) - 2.0 * (heavy.rpm - 3000.0)).abs() < 0.001);
}

#[test]
fn crankshaft_impulse_changes_angular_momentum_in_both_directions() {
    for dt in [1.0 / 60.0, 1.0 / 240.0] {
        for impulse in [-1.0, 0.0, 1.0] {
            let mut car = vehicle();
            car.engine.rpm = 3000.0;
            let initial = car.engine.rpm * std::f32::consts::TAU / 60.0;
            update(&mut car.engine, 0.5);
            let torque = car.engine.torque_nm;
            let inertia = car.engine.spec.inertia_kg_m2;
            advance(&mut car.engine, impulse, dt);
            let final_omega = car.engine.rpm * std::f32::consts::TAU / 60.0;
            assert!(((final_omega - initial) * inertia - torque * dt - impulse).abs() < 0.0001);
        }
    }
}

#[test]
fn engine_update_preserves_the_solved_clutch_lock() {
    let redline = test::support::car().engine.redline_rpm;
    for rpm in [3000.0, redline, redline - 100.0, redline - 250.0] {
        for was_cut in [false, true] {
            let mut car = vehicle();
            car.engine.rpm = rpm;
            car.engine.fuel_cut = was_cut;
            let dt = 1.0 / 240.0;
            let ratio = 12.0;
            let wheel = car.engine.rpm * std::f32::consts::TAU / 60.0 / ratio;
            update(&mut car.engine, 1.0);
            let step = powertrain::Step {
                engine_omega: car.engine.rpm * std::f32::consts::TAU / 60.0
                    + car.engine.torque_nm / car.engine.spec.inertia_kg_m2 * dt,
                wheel_omega: wheel,
                engine_inertia: car.engine.spec.inertia_kg_m2,
                wheel_inertia: 2.4,
                ratio,
                max_impulse: 10000.0 * dt,
                dt,
            };
            let rotation = powertrain::finish(&step, 0.0, 0.32, 0.0, 0.0);
            advance(&mut car.engine, rotation.crankshaft_impulse, dt);
            let engine_omega = car.engine.rpm * std::f32::consts::TAU / 60.0;
            assert!((engine_omega - rotation.wheel_omega * ratio).abs() < 0.0001);
        }
    }
}

#[test]
fn gearbox_can_drive_the_engine_past_redline_despite_fuel_cut() {
    let mut car = vehicle();
    car.engine.rpm = car.spec.engine.redline_rpm;
    car.engine.fuel_cut = true;
    update(&mut car.engine, 1.0);
    assert!(car.engine.torque_nm < 0.0);
    advance(&mut car.engine, 10.0, 1.0 / 240.0);
    assert!(car.engine.rpm > car.spec.engine.redline_rpm);
    assert!(car.engine.fuel_cut);
    update(&mut car.engine, 1.0);
    assert!(car.engine.torque_nm < 0.0);
}

#[test]
fn turbo_and_sound_follow_crankshaft_load_without_gearbox_state() {
    let mut spec = test::support::car().engine.clone();
    spec.turbo = Some(model::car::Turbo {
        spool_start_rpm: 1000.0,
        full_boost_rpm: 3000.0,
        spool_seconds: 0.2,
        decay_seconds: 0.4,
    });
    let mut loaded = new(&spec);
    let mut unloaded = new(&spec);
    loaded.rpm = 3000.0;
    unloaded.rpm = 3000.0;
    let dt = 1.0 / 240.0;
    for _ in 0..24 {
        update(&mut loaded, 1.0);
        update(&mut unloaded, 1.0);
        let reaction = -loaded.torque_nm * dt;
        advance(&mut loaded, reaction, dt);
        advance(&mut unloaded, 0.0, dt);
    }
    assert!(loaded.turbo_boost > 0.0);
    assert_eq!(unloaded.turbo_boost, 0.0);
    assert!(loaded.load > unloaded.load);
    let boost = loaded.turbo_boost;
    update(&mut loaded, 0.0);
    advance(&mut loaded, 0.0, dt);
    assert!(loaded.turbo_boost < boost);
}
