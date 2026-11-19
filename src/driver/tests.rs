use super::shift;
use super::update;
use crate::physics::car;
use crate::physics::engine;
use crate::physics::powertrain;
use crate::test;
use crate::util::geometry::XYZ;

fn moving_car(gear: u8) -> car::Vehicle {
    let mut vehicle = car::on_track(
        XYZ::default(),
        0.0,
        test::support::car(),
        test::support::compact_track(),
    );
    vehicle.gearbox.gear = gear;
    vehicle.engine.rpm = 3000.0;
    vehicle.velocity.z = vehicle.engine.rpm / powertrain::coupled_rpm(&vehicle.spec, 1.0, gear);
    vehicle
}

#[test]
fn upshift_lifts_throttle_then_returns_to_the_requested_pedal() {
    let mut vehicle = moving_car(2);
    let requested = car::Controls {
        throttle: 1.0,
        brake: 0.3,
        steering: -0.5,
        handbrake: true,
    };
    shift(&mut vehicle, true);
    let pedals = update(&mut vehicle.driver, &vehicle.gearbox, requested, 0.01);
    assert_eq!(pedals.throttle, 0.0);
    assert_eq!(pedals.brake, requested.brake);
    assert_eq!(pedals.steering, requested.steering);
    assert_eq!(pedals.handbrake, requested.handbrake);
    let pedals = update(&mut vehicle.driver, &vehicle.gearbox, requested, 0.2);
    assert_eq!(pedals.throttle, requested.throttle);
}

#[test]
fn forward_downshift_blips_the_actual_throttle_with_the_player_pedal_released() {
    let mut vehicle = moving_car(3);
    shift(&mut vehicle, false);
    assert_eq!(vehicle.gearbox.gear, 2);
    let pedals = update(
        &mut vehicle.driver,
        &vehicle.gearbox,
        car::Controls::default(),
        0.01,
    );
    assert!(pedals.throttle > 0.0);
    engine::update(&mut vehicle.engine, pedals.throttle);
    assert!(vehicle.engine.torque_nm > 0.0);
    engine::update(&mut vehicle.engine, 0.0);
    assert!(vehicle.engine.torque_nm < 0.0);
}

#[test]
fn neutral_reverse_and_stationary_downshifts_do_not_request_a_blip() {
    let mut vehicle = moving_car(1);
    for gear in [powertrain::NEUTRAL_GEAR, powertrain::REVERSE_GEAR] {
        shift(&mut vehicle, false);
        assert_eq!(vehicle.gearbox.gear, gear);
        let pedals = update(
            &mut vehicle.driver,
            &vehicle.gearbox,
            car::Controls::default(),
            0.01,
        );
        assert_eq!(pedals.throttle, 0.0);
    }
    vehicle.gearbox.gear = 3;
    vehicle.velocity = XYZ::default();
    shift(&mut vehicle, false);
    let pedals = update(
        &mut vehicle.driver,
        &vehicle.gearbox,
        car::Controls::default(),
        0.01,
    );
    assert_eq!(pedals.throttle, 0.0);
}

#[test]
fn assistance_options_are_independent() {
    let mut vehicle = moving_car(3);
    vehicle.driver.assistance.rev_matching = false;
    shift(&mut vehicle, false);
    let requested = car::Controls {
        throttle: 1.0,
        ..car::Controls::default()
    };
    let pedals = update(&mut vehicle.driver, &vehicle.gearbox, requested, 0.01);
    assert_eq!(pedals.throttle, 0.0);
    vehicle.driver.assistance.lift_on_shift = false;
    let pedals = update(&mut vehicle.driver, &vehicle.gearbox, requested, 0.01);
    assert_eq!(pedals.throttle, 1.0);
}

#[test]
fn reset_clears_an_active_blip_and_preserves_assistance_preferences() {
    let mut vehicle = moving_car(3);
    vehicle.driver.assistance.lift_on_shift = false;
    shift(&mut vehicle, false);
    assert!(vehicle.driver.rev_match_timer > 0.0);
    assert!(super::is_shifting(&vehicle.driver));
    car::reset(&mut vehicle, XYZ::default(), 0.0);
    assert_eq!(vehicle.driver.rev_match_timer, 0.0);
    assert!(!super::is_shifting(&vehicle.driver));
    assert!(!vehicle.driver.assistance.lift_on_shift);
}

#[test]
fn blip_decay_depends_on_elapsed_time() {
    let sample = |hz: u32| {
        let mut vehicle = moving_car(3);
        shift(&mut vehicle, false);
        for _ in 0..hz / 10 {
            let dt = 1.0 / hz as f32;
            update(
                &mut vehicle.driver,
                &vehicle.gearbox,
                car::Controls::default(),
                dt,
            );
        }
        update(
            &mut vehicle.driver,
            &vehicle.gearbox,
            car::Controls::default(),
            0.0,
        )
        .throttle
    };
    assert!((sample(60) - sample(240)).abs() < 0.00001);
}
