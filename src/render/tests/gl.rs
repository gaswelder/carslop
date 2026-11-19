use crate::model;
use crate::physics;
use crate::render;
use crate::test;
use crate::util;
use crate::world;

#[test]
fn camera_relative_projection_preserves_fine_car_details_at_city_coordinates() {
    let mut reference = None;
    for origin in [[0.0, 100.0, 0.0], [12000.0, 100.0, 9000.0]] {
        let camera = super::chase_camera(
            16.0 / 9.0,
            render::camera::Pose {
                eye: XYZ {
                    x: origin[0] + 4.0,
                    y: origin[1] + 2.0,
                    z: origin[2] - 6.0,
                },
                target: XYZ {
                    x: origin[0],
                    y: origin[1] + 0.25,
                    z: origin[2],
                },
            },
        );
        let clip =
            render::gl::clip_from_model(&camera, &super::model_matrix((origin).into(), 0.7, 0.0));
        let a = transform(
            clip,
            XYZ {
                x: 0.5,
                y: 0.3,
                z: 1.0,
            },
        );
        let b = transform(
            clip,
            XYZ {
                x: 0.5001,
                y: 0.3,
                z: 1.0,
            },
        );
        assert!(
            (a.x - b.x).abs() > 0.00001,
            "nearby panels must remain distinct"
        );
        if let Some((expected_a, expected_b)) = reference {
            assert_eq!(a, expected_a);
            assert_eq!(b, expected_b);
        }
        reference = Some((a, b));
    }
}

use super::*;

fn transform(m: [f32; 16], p: XYZ) -> XYZ {
    util::geometry::xyz_from_fn(|i| m[i] * p.x + m[4 + i] * p.y + m[8 + i] * p.z + m[12 + i])
}

#[test]
fn wheels_turn_about_their_axles_not_the_vehicle_origin() {
    let car = test::support::car();
    for center in car.design.mounts {
        let body = model_matrix(
            XYZ {
                x: 12.0,
                y: 3.0,
                z: -7.0,
            },
            1.2,
            0.15,
        );
        let axle = transform(body, center);
        for (steer, spin) in [(0.0, 0.0), (-0.4, 1.5), (0.4, 5.0)] {
            let matrix = wheel_matrix(body, center, steer, spin);
            let origin = transform(
                matrix,
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            for i in 0..3 {
                assert!(
                    (util::geometry::xyz_component(origin, i)
                        - util::geometry::xyz_component(axle, i))
                    .abs()
                        < 0.0001
                );
            }
            let rim = transform(
                matrix,
                XYZ {
                    x: 0.0,
                    y: 0.4_f32,
                    z: 0.0,
                },
            );
            let radius = (0..3)
                .map(|i| {
                    (util::geometry::xyz_component(rim, i) - util::geometry::xyz_component(axle, i))
                        .powi(2)
                })
                .sum::<f32>()
                .sqrt();
            assert!((radius - 0.4_f32).abs() < 0.0001);
        }
    }
}

#[test]
fn bonnet_points_stay_at_the_same_screen_position_as_the_body_moves() {
    let track = test::support::track();
    let road = physics::vehicle_location::spawn(&track);
    let mut car = physics::car::on_track(
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
        test::support::car(),
        track,
    );
    let mut chase = render::camera::new(&car);
    render::camera::toggle_view(&mut chase);
    render::camera::toggle_view(&mut chase);
    let point = XYZ {
        x: 0.3,
        y: 0.25,
        z: 1.6,
    };
    let reference = render::gl::project(
        &bonnet_camera(16.0 / 9.0, render::camera::pose(&chase), &car),
        transform(vehicle_body_matrix(&car), point),
    )
    .unwrap();
    for i in 1..5 {
        car.orientation.yaw += 0.7;
        car.position.y += 0.2;
        car.orientation.pitch = i as f32 * 0.03;
        car.orientation.roll = i as f32 * 0.025;
        render::camera::update(&mut chase, &car, std::time::Duration::from_millis(16));
        let actual = render::gl::project(
            &bonnet_camera(16.0 / 9.0, render::camera::pose(&chase), &car),
            transform(vehicle_body_matrix(&car), point),
        )
        .unwrap();
        assert!(
            (actual[0] - reference[0]).abs() < 0.0001 && (actual[1] - reference[1]).abs() < 0.0001
        );
    }
}

#[test]
fn wheels_follow_a_bump_independently_with_bounded_compression_and_droop() {
    let track = std::sync::Arc::new(test::support::bumpy_track(&test::support::track()));
    let road = world::locations::at_distance(&track, 54.0);
    let mut car = physics::car::on_track(
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
        test::support::car(),
        track,
    );
    car.suspension.pitch = 0.0;
    car.suspension.roll = 0.0;
    let body = vehicle_body_matrix(&car);
    let wheel_y: [f32; 4] = std::array::from_fn(|i| {
        let matrix = vehicle_wheel_matrix(&car, body, i);
        let center = transform(
            matrix,
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let ground = physics::vehicle_location::drive_height(
            &car.track,
            XZ {
                x: center.x,
                z: center.z,
            },
        );
        assert!((center.y - car.spec.design.wheel_radii[i] - ground).abs() < 0.005);
        center.y
    });
    assert!(
        wheel_y[2] - wheel_y[0] > 0.05,
        "front and rear wheels must travel independently"
    );
    for height in [-1.0, 1.0] {
        let mut displaced = body;
        displaced[13] += height;
        for (i, mount) in car.spec.design.mounts.into_iter().enumerate() {
            let nominal = transform(displaced, mount);
            let wheel = transform(
                vehicle_wheel_matrix(&car, displaced, i),
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let travel = wheel.y - nominal.y;
            let expected = if height < 0.0 { 0.14 } else { -0.16 };
            assert!(
                (travel - expected).abs() < 0.001,
                "suspension travel must stop at its limit"
            );
        }
    }
}

#[test]
fn body_matrix_adds_roll_without_moving_the_car_or_scaling_axes() {
    let origin = XYZ {
        x: 12.0,
        y: 3.0,
        z: -7.0,
    };
    let flat = model_matrix(origin, 0.4, 0.0);
    let rolled = body_model_matrix(origin, 0.4, 0.0, 0.08);
    assert_eq!(
        transform(
            rolled,
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        ),
        origin
    );
    assert_ne!(flat[1], rolled[1], "roll should tilt the local right axis");
    for axis in 0..3 {
        let length = (0..3)
            .map(|row| rolled[axis * 4 + row].powi(2))
            .sum::<f32>()
            .sqrt();
        assert!((length - 1.0).abs() < 0.0001);
    }
    let right_dot_up: f32 = (0..3).map(|row| rolled[row] * rolled[4 + row]).sum();
    assert!(right_dot_up.abs() < 0.0001);
}
