use super::*;
use crate::test;
use crate::util::geometry::XYZ;

fn set_longitudinal_speed(vehicle: &mut car::Vehicle, speed: f32) {
    let (sin, cos) = vehicle.orientation.yaw.sin_cos();
    vehicle.velocity = XYZ {
        y: vehicle.velocity.y,
        x: sin * speed,
        z: cos * speed,
    };
}

#[test]
fn adding_steering_always_increases_wheel_angle_at_road_speeds() {
    for model in test::support::cars() {
        let mut car = test::support::vehicle(XYZ::default(), 0.0, model.clone());
        for speed_kmh in [0.0, 40.0, 80.0, 120.0, 200.0] {
            set_longitudinal_speed(&mut car, speed_kmh / 3.6);
            for direction in [-1.0, 1.0] {
                car.steering.input = 0.0;
                let mut previous = 0.0;
                for step in 1..=100 {
                    update(&mut car, direction * step as f32 / 100.0, 1.0);
                    let angle = front_wheel_angle(&car).abs();
                    assert!(
                        angle > previous,
                        "{model:?} at {speed_kmh} km/h: more input reduced wheel angle"
                    );
                    previous = angle;
                }
                assert!((previous - 0.54).abs() < 0.001);
            }
        }
    }
}

#[test]
fn accelerating_through_a_corner_does_not_unwind_held_steering() {
    let mut car = test::support::vehicle(
        XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        0.0,
        test::support::car(),
    );
    for direction in [-1.0, 1.0] {
        car.steering.input = 0.0;
        set_longitudinal_speed(&mut car, 5.0);
        update(&mut car, direction, 1.0);
        for speed in [10.0, 20.0, 30.0, 55.0, -55.0] {
            set_longitudinal_speed(&mut car, speed);
            update(&mut car, direction, 1.0 / 240.0);
            assert_eq!(
                car.steering.input, direction,
                "held steering at {speed} m/s"
            );
            assert!((steering_angle(&car).abs() - 0.54).abs() < 0.001);
        }
    }
}

#[test]
fn released_steering_centres_faster_with_forward_speed() {
    let mut car = test::support::vehicle(
        XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        0.0,
        test::support::car(),
    );
    for direction in [-1.0, 1.0] {
        let mut previous = 1.0;
        for speed in [0.5, 2.0, 5.0, 10.0] {
            set_longitudinal_speed(&mut car, speed);
            let mut reference: Option<f32> = None;
            for hz in [30, 60, 240] {
                car.steering.input = direction;
                for _ in 0..hz / 10 {
                    update(&mut car, 0.0, 1.0 / hz as f32);
                }
                let remaining = car.steering.input.abs();
                if let Some(expected) = reference {
                    assert!((remaining - expected).abs() < 0.00001);
                } else {
                    assert!(remaining < previous);
                    previous = remaining;
                    reference = Some(remaining);
                }
            }
        }
        set_longitudinal_speed(&mut car, -5.0);
        car.steering.input = direction;
        update(&mut car, 0.0, 1.0);
        assert_eq!(car.steering.input, direction);
        set_longitudinal_speed(&mut car, 10.0);
        update(&mut car, 0.0, 1.0);
        assert_eq!(car.steering.input, 0.0);
    }
}
