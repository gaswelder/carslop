use super::*;
use crate::test;

#[test]
fn full_throttle_publishes_crankshaft_torque_without_transmission_losses() {
    let spec = test::support::car().engine.clone();
    let mut engine = new(&spec);
    engine.rpm = 3500.0;
    update(&mut engine, 1.0, 0.0);
    let expected = spec
        .max_torque_nm
        .min(spec.power_watts / (3500.0 * std::f32::consts::TAU / 60.0));
    assert!((engine.out_torque - expected).abs() < 0.0001);
}

#[test]
fn update_advances_rotation_and_publishes_output_together() {
    let mut engine = new(&test::support::car().engine);
    engine.rpm = 3000.0;
    update(&mut engine, 1.0, 1.0 / 240.0);
    assert!(engine.out_torque > 0.0);
    assert!(engine.rpm > 3000.0);

    let rpm = engine.rpm;
    update(&mut engine, 0.0, 1.0 / 240.0);
    assert!(engine.out_torque < 0.0);
    assert!(engine.rpm < rpm);
}

#[test]
fn returned_torque_affects_the_next_update_and_persists_until_replaced() {
    let mut engine = new(&test::support::car().engine);
    engine.rpm = 3000.0;
    update(&mut engine, 1.0, 0.0);
    engine.in_torque = -engine.out_torque;
    assert_eq!(engine.rpm, 3000.0);

    for _ in 0..10 {
        update(&mut engine, 1.0, 1.0 / 240.0);
        assert!((engine.rpm - 3000.0).abs() < 0.001);
        assert_eq!(engine.in_torque, -engine.out_torque);
    }

    engine.in_torque = 0.0;
    update(&mut engine, 1.0, 1.0 / 240.0);
    assert!(engine.rpm > 3000.0);
}

#[test]
fn zero_duration_does_not_apply_either_torque() {
    let mut engine = new(&test::support::car().engine);
    engine.rpm = 3000.0;
    engine.in_torque = -1000.0;
    update(&mut engine, 1.0, 0.0);
    assert!((engine.rpm - 3000.0).abs() < 0.001);
    assert_eq!(engine.load, 0.0);
}

#[test]
fn greater_inertia_reduces_rpm_response_to_output_and_input_torque() {
    let mut spec = test::support::car().engine.clone();
    spec.inertia_kg_m2 = 0.2;
    let mut light = new(&spec);
    spec.inertia_kg_m2 = 0.4;
    let mut heavy = new(&spec);
    light.rpm = 3000.0;
    heavy.rpm = 3000.0;
    light.in_torque = 240.0;
    heavy.in_torque = 240.0;
    update(&mut light, 1.0, 1.0 / 240.0);
    update(&mut heavy, 1.0, 1.0 / 240.0);
    assert!(light.rpm > 3000.0);
    assert!(((light.rpm - 3000.0) - 2.0 * (heavy.rpm - 3000.0)).abs() < 0.001);
}

#[test]
fn input_torque_changes_angular_momentum_in_both_directions() {
    let spec = &test::support::car().engine;
    for dt in [1.0 / 60.0, 1.0 / 240.0] {
        for in_torque in [-240.0, 0.0, 240.0] {
            let mut engine = new(spec);
            engine.rpm = 3000.0;
            let initial = engine.rpm * std::f32::consts::TAU / 60.0;
            engine.in_torque = in_torque;
            update(&mut engine, 0.5, dt);
            let final_omega = engine.rpm * std::f32::consts::TAU / 60.0;
            assert!(
                ((final_omega - initial) * spec.inertia_kg_m2
                    - (engine.out_torque + in_torque) * dt)
                    .abs()
                    < 0.0001
            );
        }
    }
}

#[test]
fn external_torque_can_drive_the_engine_past_redline_despite_fuel_cut() {
    let spec = &test::support::car().engine;
    let mut engine = new(spec);
    engine.rpm = spec.redline_rpm;
    engine.in_torque = 2400.0;
    update(&mut engine, 1.0, 1.0 / 240.0);
    assert!(engine.out_torque < 0.0);
    assert!(engine.rpm > spec.redline_rpm);
    assert!(engine.fuel_cut);
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
    update(&mut loaded, 1.0, 0.0);
    for _ in 0..24 {
        loaded.in_torque = -loaded.out_torque;
        update(&mut loaded, 1.0, dt);
        update(&mut unloaded, 1.0, dt);
    }
    assert!(loaded.turbo_boost > 0.0);
    assert_eq!(unloaded.turbo_boost, 0.0);
    assert!(loaded.load > unloaded.load);
    let boost = loaded.turbo_boost;
    loaded.in_torque = 0.0;
    update(&mut loaded, 0.0, dt);
    assert!(loaded.turbo_boost < boost);
}
