use super::*;

#[test]
fn increasing_driver_rate_increases_wheel_angle_at_road_speeds() {
    for speed_kmh in [0.0, 40.0, 80.0, 120.0, 200.0] {
        for direction in [-1.0, 1.0] {
            let mut previous = 0.0;
            for step in 1..=100 {
                let mut state = State::default();
                update(
                    &mut state,
                    direction * step as f32 / 100.0 * 2.0,
                    speed_kmh / 3.6,
                    0.1,
                );
                let angle = state.angle.abs();
                assert!(
                    angle > previous,
                    "at {speed_kmh} km/h: higher driver rate reduced wheel angle"
                );
                previous = angle;
            }
            assert!(previous > 0.0 && previous <= 0.54);
        }
    }
}

#[test]
fn centering_opposes_driver_input_while_moving_and_vanishes_at_rest() {
    for direction in [-1.0, 1.0] {
        let initial = -direction * 0.4;
        let mut stationary = State { angle: initial };
        let mut moving = State { angle: initial };
        update(&mut stationary, -direction * 0.4, 0.0, 0.1);
        update(&mut moving, -direction * 0.4, 10.0, 0.1);
        assert!(stationary.angle.abs() > initial.abs());
        assert!(moving.angle.abs() < initial.abs());
        assert_eq!(moving.angle.signum(), -direction);

        let mut held = State { angle: initial };
        update(&mut held, 0.0, 0.0, 1.0);
        assert!((held.angle - initial).abs() < 0.000001);
    }
}

#[test]
fn released_steering_centres_faster_with_forward_speed() {
    for direction in [-1.0, 1.0] {
        let mut state = State::default();
        let mut previous = 1.0;
        for speed in [0.5, 2.0, 5.0, 10.0] {
            let mut reference: Option<f32> = None;
            for hz in [30, 60, 240] {
                state.angle = direction * MAX_ANGLE;
                for _ in 0..hz / 10 {
                    update(&mut state, 0.0, speed, 1.0 / hz as f32);
                }
                let remaining = state.angle.abs();
                if let Some(expected) = reference {
                    assert!((remaining - expected).abs() < 0.00001);
                } else {
                    assert!(remaining < previous);
                    previous = remaining;
                    reference = Some(remaining);
                }
            }
        }
        state.angle = direction * MAX_ANGLE;
        update(&mut state, 0.0, -5.0, 1.0);
        assert!((state.angle - direction * MAX_ANGLE).abs() < 0.000001);
        update(&mut state, 0.0, 10.0, 1.0);
        assert!(state.angle.abs() < 0.02);
    }
}
