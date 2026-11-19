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
        let (sin, cos) = vehicle.orientation.yaw.sin_cos();
        vehicle.velocity.x = sin * 15.0;
        vehicle.velocity.z = cos * 15.0;
        vehicle.local_acceleration = LocalAcceleration {
            longitudinal: -4.0,
            lateral: 5.0,
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
        assert_eq!(baseline.velocity, leaning.velocity);
        assert_eq!(baseline.position, leaning.position);
        assert_eq!(baseline.orientation.yaw, leaning.orientation.yaw);
        assert_eq!(
            baseline.suspension.axle_load.front,
            leaning.suspension.axle_load.front
        );
        assert_eq!(
            baseline.suspension.axle_load.rear,
            leaning.suspension.axle_load.rear
        );
    }
    assert!((baseline.orientation.pitch - leaning.orientation.pitch).abs() > 0.001);
    assert!((baseline.orientation.roll - leaning.orientation.roll).abs() > 0.001);
}

#[test]
fn load_transfer_responds_to_raw_acceleration_without_waiting_for_body_lean() {
    let mut vehicle = vehicle();
    vehicle.suspension.initialized = true;
    vehicle.position.y = ride_height(&vehicle);
    for acceleration in [4.0, -4.0, 0.0] {
        vehicle.local_acceleration.longitudinal = acceleration;
        update_suspension(&mut vehicle, 1.0 / 240.0);
        let total = vehicle.suspension.axle_load.front + vehicle.suspension.axle_load.rear;
        let static_front = total * vehicle.spec.center_of_mass.longitudinal_fraction;
        let transfer = vehicle.spec.weight_kg
            * acceleration
            * model::car::center_of_mass_height_m(&vehicle.spec)
            / vehicle.spec.chassis.wheelbase;
        assert!((vehicle.suspension.axle_load.front - (static_front - transfer)).abs() < 0.01);
        assert_eq!(vehicle.suspension.visual_acceleration.longitudinal, 0.0);
    }
}
