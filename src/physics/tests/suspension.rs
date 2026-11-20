use super::*;
use crate::physics::powertrain;
use crate::test;
use std::time::Duration;

fn vehicle() -> car::Vehicle {
    let track = std::sync::Arc::new(test::support::flat_track(&test::support::compact_track()));
    let road = world::locations::at_distance(&track, 50.0);
    car::on_track(
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
        test::support::car(),
        track,
    )
}

#[test]
fn visual_acceleration_does_not_affect_handling_or_tire_loads() {
    let mut baseline = vehicle();
    let mut leaning = vehicle();
    for vehicle in [&mut baseline, &mut leaning] {
        vehicle.gearbox.gear = powertrain::NEUTRAL_GEAR;
        let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
        vehicle.chassis.velocity.x = sin * 15.0;
        vehicle.chassis.velocity.z = cos * 15.0;
        vehicle.chassis.acceleration = XYZ {
            x: sin * -4.0 + cos * 5.0,
            y: 0.0,
            z: cos * -4.0 - sin * 5.0,
        };
    }
    leaning.suspension.visual_acceleration = LocalAcceleration {
        longitudinal: 20.0,
        lateral: -20.0,
    };
    let controls = car::Controls {
        brake: 0.3,
        steering: 0.4,
        ..car::Controls::default()
    };
    for _ in 0..20 {
        car::update(&mut baseline, Duration::from_millis(4), controls);
        car::update(&mut leaning, Duration::from_millis(4), controls);
        assert_eq!(baseline.chassis.velocity, leaning.chassis.velocity);
        assert_eq!(baseline.chassis.position, leaning.chassis.position);
        assert_eq!(
            baseline.chassis.orientation.yaw,
            leaning.chassis.orientation.yaw
        );
        assert_eq!(
            baseline.suspension.axle_load.front,
            leaning.suspension.axle_load.front
        );
        assert_eq!(
            baseline.suspension.axle_load.rear,
            leaning.suspension.axle_load.rear
        );
    }
    assert!((baseline.chassis.orientation.pitch - leaning.chassis.orientation.pitch).abs() > 0.001);
    assert!((baseline.chassis.orientation.roll - leaning.chassis.orientation.roll).abs() > 0.001);
}

#[test]
fn load_transfer_responds_to_raw_acceleration_without_waiting_for_body_lean() {
    let mut vehicle = vehicle();
    vehicle.suspension.initialized = true;
    vehicle.chassis.position.y = ride_height(&vehicle);
    for acceleration in [4.0, -4.0, 0.0] {
        let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
        vehicle.chassis.acceleration = XYZ {
            x: sin * acceleration,
            y: 0.0,
            z: cos * acceleration,
        };
        // Deliberately stale cosmetic acceleration must not delay load transfer.
        vehicle.suspension.visual_acceleration.longitudinal = -20.0;
        vehicle.suspension.in_motion = sample(&vehicle);
        update(&mut vehicle.suspension, &vehicle.spec, 1.0 / 240.0);
        let total = vehicle.suspension.axle_load.front + vehicle.suspension.axle_load.rear;
        let static_front = total * vehicle.spec.center_of_mass.longitudinal_fraction;
        let transfer = vehicle.spec.weight_kg
            * acceleration
            * model::car::center_of_mass_height_m(&vehicle.spec)
            / vehicle.spec.chassis.wheelbase;
        assert!((vehicle.suspension.axle_load.front - (static_front - transfer)).abs() < 0.01);
        assert!((vehicle.suspension.visual_acceleration.longitudinal - acceleration).abs() > 1.0);
    }
}

fn set_longitudinal_speed(vehicle: &mut car::Vehicle, speed: f32) {
    let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
    vehicle.chassis.velocity.x = sin * speed;
    vehicle.chassis.velocity.z = cos * speed;
}

fn set_longitudinal_acceleration(vehicle: &mut car::Vehicle, acceleration: f32) {
    let (sin, cos) = vehicle.chassis.orientation.yaw.sin_cos();
    vehicle.chassis.acceleration = XYZ {
        x: sin * acceleration,
        y: 0.0,
        z: cos * acceleration,
    };
}

fn advance_body(vehicle: &mut car::Vehicle, dt: f32) {
    vehicle.suspension.in_motion = sample(vehicle);
    update_body_motion(&mut vehicle.suspension, &vehicle.spec, dt);
    vehicle.chassis.in_torque = vehicle.suspension.out_torque;
    chassis::update(&mut vehicle.chassis, &vehicle.spec, dt);
}

#[test]
fn body_motion_handles_reverse_and_frame_rate_independent_settling() {
    let mut forward = vehicle();
    let mut reverse = vehicle();
    set_longitudinal_speed(&mut forward, 6.0);
    set_longitudinal_speed(&mut reverse, -6.0);
    set_longitudinal_acceleration(&mut forward, 3.0);
    set_longitudinal_acceleration(&mut reverse, -3.0);
    for _ in 0..240 {
        advance_body(&mut forward, 1.0 / 240.0);
        advance_body(&mut reverse, 1.0 / 240.0);
    }
    assert!(forward.chassis.orientation.pitch > 0.005);
    assert!(
        (forward.chassis.orientation.pitch + reverse.chassis.orientation.pitch).abs() < 0.00001
    );
    set_longitudinal_acceleration(&mut reverse, 4.0);
    for _ in 0..240 {
        advance_body(&mut reverse, 1.0 / 240.0);
    }
    assert!(
        reverse.chassis.orientation.pitch > 0.01,
        "reverse braking should lift the nose"
    );
    forward.chassis.acceleration = XYZ::default();
    reverse.chassis.acceleration = XYZ::default();
    forward.chassis.orientation.pitch = 0.05;
    reverse.chassis.orientation.pitch = 0.05;
    for _ in 0..30 {
        advance_body(&mut forward, 1.0 / 30.0);
    }
    for _ in 0..144 {
        advance_body(&mut reverse, 1.0 / 144.0);
    }
    assert!((forward.chassis.orientation.pitch - reverse.chassis.orientation.pitch).abs() < 0.0002);
    assert!(forward.chassis.orientation.pitch.abs() < 0.0005);
}
