use super::*;
use crate::model;
use crate::physics;
use crate::storage;
use crate::test;
use crate::util;
use crate::world;

fn new(position: XYZ, heading: f32, spec: std::sync::Arc<model::car::CarSpec>) -> Vehicle {
    physics::car::on_track(position, heading, spec, test::support::compact_track())
}

// Set road speed for a target coupled RPM without exposing engine internals.
fn speed_at_rpm(spec: &model::car::CarSpec, rpm: f32, gear: u8) -> f32 {
    rpm / engine::rpm(spec, 1.0, gear)
}

fn set_longitudinal_speed(vehicle: &mut Vehicle, speed: f32) {
    let (sin, cos) = vehicle.orientation.yaw.sin_cos();
    vehicle.velocity = XYZ {
        y: vehicle.velocity.y,
        x: sin * speed,
        z: cos * speed,
    };
}

#[test]
fn world_acceleration_tracks_airborne_motion_and_collision_impulses() {
    for collision in [false, true] {
        let mut car = level_route_car(world::locations::LocationId::Compact, test::support::car());
        car.gear = NEUTRAL_GEAR;
        car.orientation.yaw = 0.7;
        if collision {
            let bounds = world::elevation::terrain_bounds(&car.track);
            car.position.x = bounds.min.x - 1.0;
        }
        car.position.y += 100.0;
        car.suspension_initialized = true;
        car.velocity = XYZ {
            x: -4.0,
            y: 2.0,
            z: 8.0,
        };
        let previous_velocity = car.velocity;
        let dt = Duration::from_micros(4000);
        let seconds = dt.as_secs_f32();
        let impact = update(&mut car, dt, Controls::default());
        assert_eq!(impact > 0.0, collision);
        assert!((car.acceleration.y + 9.81).abs() < 0.001);
        assert!((car.velocity.y - (2.0 - 9.81 * seconds)).abs() < 0.000001);
        for axis in 0..3 {
            let change = util::geometry::xyz_component(car.velocity, axis)
                - util::geometry::xyz_component(previous_velocity, axis);
            let acceleration = util::geometry::xyz_component(car.acceleration, axis);
            assert!((acceleration * seconds - change).abs() < 0.000001);
        }
        if collision {
            assert!(
                car.acceleration.x > 10.0,
                "collision impulse must reach acceleration"
            );
        }
        let velocity = car.velocity;
        let acceleration = car.acceleration;
        update(&mut car, Duration::ZERO, Controls::default());
        assert_eq!(car.velocity, velocity);
        assert_eq!(car.acceleration, acceleration);
    }
}

#[test]
fn force_telemetry_balances_actual_acceleration_in_both_directions() {
    for gear in [REVERSE_GEAR, 1, NEUTRAL_GEAR] {
        let mut car = level_route_car(world::locations::LocationId::Compact, test::support::car());
        car.gear = gear;
        let direction = if gear == REVERSE_GEAR { -1.0 } else { 1.0 };
        let forward = XZ {
            x: car.orientation.yaw.sin(),
            z: car.orientation.yaw.cos(),
        };
        car.velocity = util::geometry::at_height(
            util::geometry::xz_map(forward, |v| v * 10.0 * direction),
            car.velocity.y,
        );
        car.engine.rpm = 3500.0;
        let before = dot(util::geometry::xz(car.velocity), forward);
        let dt = Duration::from_micros(4000);
        physics::car::update(
            &mut car,
            dt,
            Controls {
                throttle: 0.5,
                ..Controls::default()
            },
        );
        let telemetry = force_telemetry(&car);
        let actual = (dot(util::geometry::xz(car.velocity), forward) - before) * car.spec.weight_kg
            / dt.as_secs_f32();
        let reported = telemetry.drive_force_n - direction * telemetry.resistance_force_n;
        assert!(
            // Velocity differences amplify f32 rounding when converted to newtons.
            (actual - reported).abs() < 1.0,
            "gear {gear}: {actual} vs {reported}"
        );
        assert!(telemetry.torque_nm > 0.0);
        assert!(telemetry.resistance_force_n > 0.0);
        if gear == NEUTRAL_GEAR {
            assert!(telemetry.drive_force_n.abs() < 0.01);
        }
    }
}

#[test]
fn force_telemetry_reports_braking_and_engine_drag() {
    let mut car = level_route_car(world::locations::LocationId::Compact, test::support::car());
    car.gear = NEUTRAL_GEAR;
    car.velocity = XYZ {
        y: car.velocity.y,
        x: car.orientation.yaw.sin() * 10.0,
        z: car.orientation.yaw.cos() * 10.0,
    };
    car.engine.rpm = 3500.0;
    for _ in 0..30 {
        treadmill_tick(
            &mut car,
            Controls {
                brake: 1.0,
                ..Controls::default()
            },
        );
    }
    let telemetry = force_telemetry(&car);
    assert!(telemetry.drive_force_n < 0.0);
    assert!(telemetry.torque_nm < 0.0);
    physics::car::reset(&mut car, XYZ::default(), 0.0);
    let reset = force_telemetry(&car);
    assert_eq!(reset.drive_force_n, 0.0);
    assert_eq!(reset.resistance_force_n, 0.0);
    assert_eq!(reset.torque_nm, 0.0);
}

#[test]
fn force_telemetry_has_no_tire_force_without_ground_contact_or_resistance_at_rest() {
    let mut car = level_route_car(world::locations::LocationId::Compact, test::support::car());
    car.gear = 1;
    car.position.y += 10.0;
    car.suspension_initialized = true;
    car.engine.rpm = 3500.0;
    physics::car::update(
        &mut car,
        Duration::from_micros(4000),
        Controls {
            throttle: 1.0,
            ..Controls::default()
        },
    );
    let telemetry = force_telemetry(&car);
    assert_eq!(telemetry.drive_force_n, 0.0);
    assert_eq!(telemetry.resistance_force_n, 0.0);
    assert!(telemetry.torque_nm > 0.0);
}

#[test]
fn mapped_speed_table_moves_the_suspension_and_settles() {
    let run = |hz: u32| {
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), false);
        storage::road_rises::write(
            &mut std::fs::File::create(folder.path().join("road-rises.txt")).unwrap(),
            &test::support::speed_table_rises(),
        )
        .unwrap();
        let track = std::sync::Arc::new(world::preparation::world_location_load(&assets).unwrap());
        assert!(
            (physics::vehicle_location::drive_height(&track, XZ { x: 14., z: 0. }) - 7.1).abs()
                < 0.00001
        );
        let mut car = physics::car::on_track(
            util::geometry::at_height(XZ { x: 11.5, z: 0. }, 7.48),
            std::f32::consts::FRAC_PI_2,
            test::support::car(),
            track,
        );
        car.gear = NEUTRAL_GEAR;
        car.velocity = XYZ {
            y: car.velocity.y,
            x: 5.,
            z: 0.,
        };
        let mut peak_height: f32 = 0.;
        let mut peak_pitch: f32 = 0.;
        for _ in 0..hz * 3 {
            if car.position.x > 18. {
                car.velocity = XYZ {
                    y: car.velocity.y,
                    ..XYZ::default()
                };
            }
            physics::car::update(
                &mut car,
                Duration::from_secs_f64(1. / hz as f64),
                Controls::default(),
            );
            peak_height = peak_height.max(car.position.y - 7.48);
            peak_pitch = peak_pitch.max(car.suspension_pitch.abs());
            assert!(util::geometry::iter(&(car.position)).all(|v| v.is_finite()));
        }
        assert!(
            peak_height > 0.04 && peak_height < 0.3,
            "heave {peak_height}"
        );
        assert!(peak_pitch > 0.01 && peak_pitch < 0.2, "pitch {peak_pitch}");
        assert!((car.position.y - suspension::ride_height(&(car))).abs() < 0.01);
        peak_height
    };
    assert!((run(60) - run(240)).abs() < 0.02);
}

#[test]
fn aerodynamic_drag_uses_coefficient_area_and_mass() {
    let mut spec = test::support::car_spec().clone();
    spec.drag_coefficient = 0.3;
    spec.frontal_area_m2 = 2.0;
    spec.weight_kg = 1000.0;
    // At 20 m/s this gives 147 N of aerodynamic drag, plus rolling resistance.
    let drag = engine::resistance_force(&spec, 20.0) - engine::resistance_force(&spec, 0.0);
    assert!((drag - 147.0).abs() < 0.0001);
    assert_eq!(engine::resistance_force(&spec, 0.0), 250.0);
    assert_eq!(
        engine::resistance_force(&(spec), -20.0),
        engine::resistance_force(&(spec), 20.0)
    );
    spec.weight_kg *= 2.0;
    let heavier = engine::resistance_force(&spec, 20.0) - engine::resistance_force(&spec, 0.0);
    assert!((heavier - drag).abs() < 0.0001);
    assert_eq!(engine::resistance_force(&spec, 0.0), 500.0);
    spec.drag_coefficient *= 2.0;
    let more_drag = engine::resistance_force(&spec, 20.0) - engine::resistance_force(&spec, 0.0);
    assert!((more_drag - 2.0 * drag).abs() < 0.0001);
    spec.frontal_area_m2 *= 2.0;
    let larger_area = engine::resistance_force(&spec, 20.0) - engine::resistance_force(&spec, 0.0);
    assert!((larger_area - 4.0 * drag).abs() < 0.0001);
}

fn hill_car(uphill: bool) -> Vehicle {
    let route = world::locations::try_track(&test::support::authored_id()).unwrap();
    let road = (0..200)
        .map(|i| world::locations::at_distance(&route, i as f32 * 10.0))
        .max_by(|a, b| {
            dot(
                physics::vehicle_location::ground_gradient(&route, a.point),
                a.tangent,
            )
            .total_cmp(&dot(
                physics::vehicle_location::ground_gradient(&route, b.point),
                b.tangent,
            ))
        })
        .unwrap();
    assert!(
        dot(
            physics::vehicle_location::ground_gradient(&route, road.point),
            road.tangent
        ) > 0.04
    );
    physics::car::on_track(
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road) + if uphill { 0.0 } else { std::f32::consts::PI },
        test::support::car(),
        route,
    )
}

#[test]
fn rolling_direction_changes_keep_momentum_then_reverse_under_power() {
    for model in test::support::cars() {
        for initial_speed in [12.0, -4.0] {
            let mut car = level_route_car(test::support::authored_id(), model.clone());

            if initial_speed < 0.0 {
                physics::car::shift(&mut car, false);
                physics::car::shift(&mut car, false);
            }
            car.velocity = XYZ {
                y: car.velocity.y,
                x: car.orientation.yaw.sin() * initial_speed,
                z: car.orientation.yaw.cos() * initial_speed,
            };
            physics::car::refresh_slip(&mut car);
            let velocity = car.velocity;
            let heading = car.orientation.yaw;
            // Both directions pass through neutral while still moving.
            physics::car::shift(&mut car, initial_speed < 0.0);
            assert_eq!(car.gear, NEUTRAL_GEAR);
            assert_eq!(car.velocity, velocity);
            physics::car::shift(&mut car, initial_speed < 0.0);
            assert_eq!(car.gear, if initial_speed > 0.0 { REVERSE_GEAR } else { 1 });
            assert_eq!(car.velocity, velocity);
            assert_eq!(car.orientation.yaw, heading);
            assert!(physics::car::engine_net_torque(&(car), 1.0) > 0.0);
            for _ in 0..240 * 8 {
                treadmill_tick(
                    &mut car,
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            assert!(
                longitudinal_speed(&car) * initial_speed < 0.0,
                "{model:?} must pull away in the selected direction: {} -> {}",
                initial_speed,
                longitudinal_speed(&car)
            );
        }
    }
}

#[test]
fn tires_regain_direction_after_lifting_out_of_a_slide() {
    for model in test::support::cars() {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
        let h = car.orientation.yaw;
        car.velocity = XYZ {
            y: car.velocity.y,
            x: h.sin() * 15.0 + h.cos() * 6.0,
            z: h.cos() * 15.0 - h.sin() * 6.0,
        };
        car.orientation_rate.yaw = 0.5;
        for _ in 0..480 {
            treadmill_tick(&mut car, Controls::default());
        }
        let rear_side = car.slip
            - car.orientation_rate.yaw
                * model.chassis.wheelbase
                * model.center_of_mass.longitudinal_fraction;
        let angle = rear_side
            .abs()
            .atan2(longitudinal_speed(&car).abs().max(2.5))
            .to_degrees();
        assert!(angle < 3.0, "{model:?}: slide persists after lift: {angle}");
        assert!(
            car.orientation_rate.yaw.abs() < 0.15,
            "{model:?}: rotation persists after lift"
        );
        assert!(
            longitudinal_speed(&car) > 8.0,
            "recovering grip must not simply stop the car"
        );
    }
}

#[test]
fn revving_in_neutral_then_engaging_a_gear_spins_the_driven_wheels() {
    for (model, reverse) in test::support::cars()
        .iter()
        .cloned()
        .flat_map(|m| [(m.clone(), false), (m, true)])
    {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());

        physics::car::shift(&mut car, false);
        assert_eq!(car.gear, NEUTRAL_GEAR);
        let throttle = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        // Start each launch with comparable crankshaft speed. Authored
        // torque curves need different amounts of time to free-rev.
        for _ in 0..720 {
            treadmill_tick(&mut car, throttle);
            if car.engine.rpm > 4500.0 {
                break;
            }
        }
        assert!(
            longitudinal_speed(&car).abs() < 0.02,
            "{model:?}: neutral must not move the car"
        );
        assert!(
            car.engine.rpm > 4500.0,
            "{model:?}: neutral must allow free revving"
        );
        physics::car::shift(&mut car, !reverse);
        assert_eq!(car.gear, if reverse { REVERSE_GEAR } else { 1 });
        let mut peak_spin = 0.0_f32;
        for _ in 0..240 {
            treadmill_tick(&mut car, throttle);
            peak_spin = peak_spin.max(car.wheel_spin);
        }
        assert!(
            peak_spin > 0.15,
            "{model:?}: neutral launch must break traction"
        );
        assert!(longitudinal_speed(&car).abs() > 1.0);
        for _ in 0..480 {
            treadmill_tick(&mut car, Controls::default());
        }
        assert!(car.wheel_spin < 0.01);
        let position = car.position;
        let heading = car.orientation.yaw;
        physics::car::reset(&mut car, position, heading);
        assert_eq!(car.driven_wheel_omega, 0.0);
    }
}

#[test]
fn driven_tires_share_cornering_grip_and_yaw_has_inertia() {
    let run = |throttle| {
        let mut car = level_route_car(world::locations::LocationId::Compact, test::support::car());
        let h = car.orientation.yaw;
        car.velocity = XYZ {
            y: car.velocity.y,
            x: h.sin() * 10.0 + h.cos() * 2.0,
            z: h.cos() * 10.0 - h.sin() * 2.0,
        };
        physics::car::update(
            &mut car,
            Duration::from_secs_f64(1.0 / 240.0),
            Controls {
                throttle,
                ..Controls::default()
            },
        );
        dot(
            util::geometry::xz(car.velocity),
            XZ {
                x: h.cos(),
                z: -h.sin(),
            },
        )
    };
    assert!(
        run(1.0) > run(0.0),
        "drive torque must reduce available cornering force"
    );
    let mut car = level_route_car(world::locations::LocationId::Compact, test::support::car());
    car.velocity = XYZ {
        y: car.velocity.y,
        x: car.orientation.yaw.sin() * 20.0,
        z: car.orientation.yaw.cos() * 20.0,
    };
    car.orientation_rate.yaw = 0.5;
    physics::car::update(
        &mut car,
        Duration::from_secs_f64(1.0 / 240.0),
        Controls::default(),
    );
    assert!(
        car.orientation_rate.yaw > 0.0 && car.orientation_rate.yaw < 0.5,
        "tire torque should slow existing rotation without deleting it"
    );
}

#[test]
fn bump_moves_sprung_body_then_settles_at_different_frame_rates() {
    let run = |hz: u32| {
        let track = std::sync::Arc::new(test::support::bumpy_track(
            &world::locations::try_track(&test::support::authored_id()).unwrap(),
        ));
        let road = world::locations::at_distance(&track, 50.0);
        let mut car = physics::car::on_track(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            test::support::car(),
            track,
        );
        car.velocity = util::geometry::at_height(
            util::geometry::xz_map(road.tangent, |v| v * 8.0),
            car.velocity.y,
        );
        let mut peak_pitch: f32 = 0.0;
        let mut peak_heave: f32 = 0.0;
        for _ in 0..hz * 3 {
            physics::car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / hz as f64),
                Controls::default(),
            );
            peak_pitch = peak_pitch.max(car.suspension_pitch.abs());
            peak_heave = peak_heave.max((car.position.y - 0.48).abs());
        }
        println!(
            "{hz} Hz: bump pitch {peak_pitch}, heave {peak_heave}, final {}",
            car.position.y
        );
        assert!(peak_pitch > 0.01 && peak_pitch < 0.15);
        assert!(peak_heave > 0.025 && peak_heave < 0.25);
        assert!((car.position.y - suspension::ride_height(&(car))).abs() < 0.003);
        assert!(car.suspension_pitch.abs() < 0.002);
        assert!(car.suspension_roll.abs() < 0.002);
        (peak_pitch, peak_heave)
    };
    let reference = run(240);
    for hz in [30, 60, 144] {
        let sample = run(hz);
        assert!((sample.0 - reference.0).abs() < 0.005);
        assert!((sample.1 - reference.1).abs() < 0.01);
    }
}

#[test]
fn fixed_suspension_stiffness_gives_less_acceleration_with_more_mass() {
    let mut spec = test::support::car_spec().clone();
    spec.chassis.suspension_stiffness_n_m = 100_000.0;
    for mass in [1000.0, 2000.0] {
        spec.weight_kg = mass;
        let mut car = level_route_car(
            test::support::authored_id(),
            std::sync::Arc::new(spec.clone()),
        );
        let ground = suspension::ride_height(&car);
        car.suspension_initialized = true;
        car.position.y = ground - 0.05;
        car.velocity.y = 0.0;
        suspension::update_suspension(&mut car, 1.0 / 240.0);
        // The same compression produces 5000 N of restoring force in both cars.
        assert!((car.acceleration.y * mass - 5000.0).abs() < 1.0);
    }
}

#[test]
fn chassis_can_unload_land_and_settle() {
    let mut car = level_route_car(test::support::authored_id(), test::support::car());
    let ground = suspension::ride_height(&car);
    car.suspension_initialized = true;
    car.position.y = ground + 0.5;
    physics::car::update(&mut car, Duration::from_millis(20), Controls::default());
    assert_eq!(car.axle_load.front, 0.0);
    assert_eq!(car.axle_load.rear, 0.0);
    assert!(car.position.y > ground + 0.49);
    assert!(car.velocity.y < 0.0);
    for _ in 0..240 * 3 {
        physics::car::update(
            &mut car,
            Duration::from_secs_f64(1.0 / 240.0),
            Controls::default(),
        );
    }
    assert!((car.position.y - ground).abs() < 0.002);
    assert!(car.velocity.y.abs() < 0.002);
    assert!(((car.axle_load.front + car.axle_load.rear) / car.spec.weight_kg - 9.81).abs() < 0.02);
}

#[test]
fn elevation_changes_coasting_and_power_in_both_directions() {
    // Isolate gravity below tire saturation; a loaded uphill suspension
    // can temporarily provide more launch traction at full throttle.
    for throttle in [0.0, 0.35] {
        for reverse in [false, true] {
            let run = |uphill| {
                let mut car = hill_car(uphill);
                if reverse {
                    select_reverse(&mut car);
                    car.orientation.yaw += std::f32::consts::PI;
                }
                let sign = if reverse { -1.0 } else { 1.0 };
                car.velocity = XYZ {
                    y: car.velocity.y,
                    x: car.orientation.yaw.sin() * 8.0 * sign,
                    z: car.orientation.yaw.cos() * 8.0 * sign,
                };
                physics::car::update(
                    &mut car,
                    Duration::from_millis(250),
                    Controls {
                        throttle,
                        ..Controls::default()
                    },
                );
                longitudinal_speed(&car) * sign
            };
            let down = run(false);
            let up = run(true);
            assert!(
                down > up + 0.15,
                "throttle {throttle}, reverse {reverse}: downhill {down}, uphill {up}"
            );
        }
    }
}

#[test]
fn hills_allow_rollback_but_brakes_hold_and_substeps_are_consistent() {
    let mut rolling = hill_car(true);
    physics::car::update(&mut rolling, Duration::from_secs(1), Controls::default());
    assert!(
        longitudinal_speed(&rolling) < -0.1,
        "rollback: {}",
        longitudinal_speed(&rolling)
    );
    for handbrake in [false, true] {
        let mut held = hill_car(true);
        physics::car::update(
            &mut held,
            Duration::from_secs(2),
            Controls {
                brake: if handbrake { 0.0 } else { 1.0 },
                handbrake,
                ..Controls::default()
            },
        );
        assert!(longitudinal_speed(&held).abs() < 0.001);
    }
    let run = |hz| {
        let mut car = hill_car(false);
        for _ in 0..hz {
            physics::car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / hz as f64),
                Controls::default(),
            );
        }
        longitudinal_speed(&car)
    };
    assert!((run(30) - run(120)).abs() < 0.005);
}

fn car() -> Vehicle {
    let start = world::locations::at_distance(
        &world::locations::try_track(&world::locations::LocationId::Compact).unwrap(),
        5.0,
    );
    new(
        physics::vehicle_location::vehicle_position(start, 0.0),
        model::world::heading(start),
        test::support::car(),
    )
}

fn route_car(
    id: world::locations::LocationId,
    model: std::sync::Arc<model::car::CarSpec>,
) -> Vehicle {
    let track = world::locations::try_track(&id).unwrap();
    let road = world::locations::at_distance(&track, 50.0);
    physics::car::on_track(
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
        model,
        track,
    )
}

// Compare car behavior on identical level ground across locations.
fn level_route_car(
    id: world::locations::LocationId,
    model: std::sync::Arc<model::car::CarSpec>,
) -> Vehicle {
    let track = std::sync::Arc::new(test::support::flat_track(
        &world::locations::try_track(&id).unwrap(),
    ));
    let road = world::locations::at_distance(&track, 50.0);
    physics::car::on_track(
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
        model,
        track,
    )
}

// Reset only position each tick: velocity/drivetrain evolve normally, but
// bends and barriers cannot masquerade as acceleration or braking limits.
fn treadmill_tick(car: &mut Vehicle, controls: Controls) -> f32 {
    let road = world::locations::at_distance(&car.track, 50.0);
    car.position = physics::vehicle_location::vehicle_position(road, 0.0);
    let previous = car.position;
    physics::car::update(car, Duration::from_secs_f64(1.0 / 240.0), controls);
    dot(
        XZ {
            x: car.position.x - previous.x,
            z: car.position.z - previous.z,
        },
        road.tangent,
    )
    .abs()
}

// Keep a fixed speed on a straight test patch so road barriers, acceleration
// and drag cannot hide a steering regression. Heading/yaw evolve normally.
fn steering_run(car: &mut Vehicle, speed: f32, input: f32, seconds: f64, hz: f64) {
    // Externally imposed road speed supplies no drivetrain torque.
    car.gear = NEUTRAL_GEAR;
    let road = world::locations::at_distance(&car.track, 50.0);
    let mut remaining = seconds;
    while remaining > 1e-8 {
        let step = remaining.min(1.0 / hz);
        car.position = physics::vehicle_location::vehicle_position(road, 0.0);
        let lateral = dot(
            util::geometry::xz(car.velocity),
            XZ {
                x: car.orientation.yaw.cos(),
                z: -car.orientation.yaw.sin(),
            },
        );
        car.velocity = XYZ {
            y: car.velocity.y,
            x: car.orientation.yaw.sin() * speed + car.orientation.yaw.cos() * lateral,
            z: car.orientation.yaw.cos() * speed - car.orientation.yaw.sin() * lateral,
        };
        physics::car::refresh_slip(car);
        physics::car::update(
            car,
            Duration::from_secs_f64(step),
            Controls {
                steering: input,
                ..Controls::default()
            },
        );
        remaining -= step;
    }
}

#[test]
fn parking_speed_retains_steering_lock_and_follows_the_front_wheels() {
    for model in test::support::cars() {
        for speed in [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0] {
            let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
            for steering in [-1.0, 1.0] {
                steering_run(&mut car, speed, steering, 5.0, 240.0);
                let angle = physics::car::steering_angle(&car).abs();
                let expected_yaw = speed.abs() * angle.tan() / model.chassis.wheelbase;
                println!(
                    "{model:?} at {speed}: lock {angle}, yaw {} expected {expected_yaw}",
                    car.orientation_rate.yaw.abs()
                );
                assert!(
                    (angle - 0.54).abs() < 0.001,
                    "{model:?}: full input must retain full steering lock"
                );
                assert!(
                    car.orientation_rate.yaw.abs() > expected_yaw * 0.75,
                    "{model:?}: low-speed turn pushes wide of the front wheels"
                );
            }
        }
    }
}

#[test]
fn adding_steering_always_increases_wheel_angle_at_road_speeds() {
    for model in test::support::cars() {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
        for speed_kmh in [0.0, 40.0, 80.0, 120.0, 200.0] {
            set_longitudinal_speed(&mut car, speed_kmh / 3.6);
            for direction in [-1.0, 1.0] {
                car.steering = 0.0;
                let mut previous = 0.0;
                for step in 1..=100 {
                    physics::car::update_steering(&mut car, direction * step as f32 / 100.0, 1.0);
                    let angle = physics::car::front_wheel_angle(&car).abs();
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
    let mut car = new(
        XYZ {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        0.0,
        test::support::car(),
    );
    for direction in [-1.0, 1.0] {
        car.steering = 0.0;
        set_longitudinal_speed(&mut car, 5.0);
        physics::car::update_steering(&mut car, direction, 1.0);
        for speed in [10.0, 20.0, 30.0, 55.0, -55.0] {
            set_longitudinal_speed(&mut car, speed);
            physics::car::update_steering(&mut car, direction, 1.0 / 240.0);
            assert_eq!(car.steering, direction, "held steering at {speed} m/s");
            assert!((physics::car::steering_angle(&car).abs() - 0.54).abs() < 0.001);
        }
    }
}

#[test]
fn keyboard_taps_turn_less_than_held_input_and_release_steering() {
    let model = test::support::car();
    for speed in [5.0, 15.0, 25.0, 45.0, 55.0] {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
        let heading = car.orientation.yaw;
        steering_run(&mut car, speed, 0.2, 0.1, 120.0);
        let tap_yaw = car.orientation_rate.yaw.abs();
        steering_run(&mut car, speed, 0.0, 0.4, 120.0);
        let tap = test::support::angle(car.orientation.yaw - heading).abs();
        assert!(tap > 0.0, "a steering tap must turn the car");
        assert_eq!(car.steering, 0.0);
        assert!(
            car.orientation_rate.yaw.abs() < tap_yaw,
            "releasing steering must reduce yaw"
        );
        let mut held = level_route_car(world::locations::LocationId::Compact, model.clone());
        steering_run(&mut held, speed, 0.2, 1.0, 120.0);
        let turn = test::support::angle(held.orientation.yaw - heading).abs();
        assert!(
            turn > tap * 3.0,
            "holding must produce a stronger turn than tapping"
        );
    }
}

#[test]
fn tire_torque_turns_and_countersteering_reverses_rotation() {
    for model in test::support::cars() {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
        steering_run(&mut car, 15.0, 0.25, 0.6, 240.0);
        assert!(car.orientation_rate.yaw < 0.0);
        assert!(car.orientation_rate.yaw.is_finite() && car.orientation_rate.yaw.abs() < 2.0);
        steering_run(&mut car, 15.0, -0.5, 0.8, 240.0);
        assert!(
            car.orientation_rate.yaw > 0.0,
            "{model:?} countersteering remained in original turn"
        );
        assert!(physics::car::front_wheel_angle(&(car)) > 0.0);
        steering_run(&mut car, 0.0, 0.0, 0.02, 240.0);
        assert!(car.orientation_rate.yaw.is_finite());
        let position = car.position;
        let heading = car.orientation.yaw;
        physics::car::reset(&mut car, position, heading);
        assert_eq!(car.orientation_rate.yaw, 0.0);
        assert_eq!(physics::car::front_wheel_angle(&(car)), 0.0);
    }
}

#[test]
fn steering_response_is_consistent_across_frame_rates() {
    let model = test::support::car();
    let mut reference = level_route_car(world::locations::LocationId::Compact, model.clone());
    let start = reference.orientation.yaw;
    for (input, seconds) in [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)] {
        steering_run(&mut reference, 25.0, input, seconds, 240.0);
    }
    for hz in [30.0, 60.0, 144.0] {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
        for (input, seconds) in [(1.0, 0.1), (0.0, 0.25), (-1.0, 0.7), (1.0, 0.4), (0.0, 0.3)] {
            steering_run(&mut car, 25.0, input, seconds, hz);
        }
        let error = test::support::angle(car.orientation.yaw - reference.orientation.yaw)
            .abs()
            .to_degrees();
        assert!(
            error < 0.25,
            "{model:?} {hz} Hz differs by {error} degrees from 240 Hz; start {start}"
        );
    }
}

#[test]
fn released_steering_centres_faster_with_forward_speed() {
    let mut car = new(
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
                car.steering = direction;
                for _ in 0..hz / 10 {
                    physics::car::update_steering(&mut car, 0.0, 1.0 / hz as f32);
                }
                let remaining = car.steering.abs();
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
        car.steering = direction;
        physics::car::update_steering(&mut car, 0.0, 1.0);
        assert_eq!(car.steering, direction);
        set_longitudinal_speed(&mut car, 10.0);
        physics::car::update_steering(&mut car, 0.0, 1.0);
        assert_eq!(car.steering, 0.0);
    }
}

#[test]
fn selected_gears_hold_across_speeds_and_explicit_upshifts_drop_coupled_rpm() {
    for model in test::support::cars() {
        let mut car = level_route_car(test::support::authored_id(), model.clone());
        let forward = XZ {
            x: car.orientation.yaw.sin(),
            z: car.orientation.yaw.cos(),
        };
        for gear in REVERSE_GEAR..=engine::gears(&model.gearbox) {
            car.gear = gear;
            for speed in [-10.0, 0.0, 10.0, 60.0] {
                car.velocity = util::geometry::at_height(
                    util::geometry::xz_map(forward, |v| v * speed),
                    car.velocity.y,
                );
                physics::car::update(&mut car, Duration::from_micros(4000), Controls::default());
                assert_eq!(car.gear, gear, "speed {speed} must preserve selected gear");
            }
        }
        car.gear = NEUTRAL_GEAR;
        physics::car::update(&mut car, Duration::from_micros(4000), Controls::default());
        assert_eq!(car.gear, NEUTRAL_GEAR);
        for gear in 1..engine::gears(&model.gearbox) {
            car.gear = gear;
            set_longitudinal_speed(
                &mut car,
                speed_at_rpm(&model, model.engine.redline_rpm * 0.8, gear),
            );
            let before = physics::car::coupled_engine_rpm(&car);
            physics::car::shift(&mut car, true);
            assert_eq!(car.gear, gear + 1);
            assert!(physics::car::coupled_engine_rpm(&car) < before);
        }
    }
}

#[test]
fn city_curb_is_crossable_with_a_single_small_speed_loss_and_settling_bump() {
    let track = test::support::city_world();
    let city = &track;
    let (q, normal) = world::roads::curbs(city)
        .iter()
        .find_map(|curb| {
            let q = XZ {
                x: (curb.ends[0].x + curb.ends[1].x) * 0.5,
                z: (curb.ends[0].z + curb.ends[1].z) * 0.5,
            };
            let inside = XZ {
                x: q.x + curb.inward.x * 0.08,
                z: q.z + curb.inward.z * 0.08,
            };
            let outside = XZ {
                x: q.x - curb.inward.x * 0.2,
                z: q.z - curb.inward.z * 0.2,
            };
            if !world::street_surface::contains_road(city, inside)
                || world::street_surface::contains_road(city, outside)
                || world::street_surface::paving_height(city, outside)
                    != Some(world::roads::pavement_height_meters(city))
            {
                return None;
            }
            for distance in [0.08, -0.2, -1.0, -2.0, -3.0] {
                let mut p = XYZ {
                    x: q.x + curb.inward.x * distance,
                    y: 0.0,
                    z: q.z + curb.inward.z * distance,
                };
                let before = p;
                physics::vehicle_location::constrain(city, &mut p, &mut XZ::default(), 1.7);
                if (p.x - before.x).hypot(p.z - before.z) > 0.001 {
                    return None;
                }
            }
            Some((q, curb.inward))
        })
        .expect("missing clear curb crossing fixture");
    let position = XYZ {
        x: q.x + normal.x * 0.08,
        y: 0.0,
        z: q.z + normal.z * 0.08,
    };
    let heading = (-normal.x).atan2(-normal.z);
    let mut car = physics::car::on_track(position, heading, test::support::car(), track.clone());
    car.velocity =
        util::geometry::at_height(util::geometry::xz_map(normal, |n| -n * 8.0), car.velocity.y);
    physics::car::update(&mut car, Duration::from_millis(24), Controls::default());
    assert!(!world::street_surface::contains_road(
        city,
        XZ {
            x: car.position.x,
            z: car.position.z
        }
    ));
    assert!(
        longitudinal_speed(&car) > 7.0 && longitudinal_speed(&car) < 7.8,
        "small curb penalty: {}",
        longitudinal_speed(&car)
    );
    assert!(car.curb_bump > 0.0 && car.curb_bump <= 0.03);
    let speed = longitudinal_speed(&car);
    let bump = car.curb_bump;
    physics::car::update(&mut car, Duration::ZERO, Controls::default());
    assert_eq!(car.curb_bump, bump);
    car.gear = NEUTRAL_GEAR; // isolate curb drag from intentional engine braking
    physics::car::update(&mut car, Duration::from_millis(250), Controls::default());
    assert!(car.curb_bump < bump * 0.1);
    assert!(
        longitudinal_speed(&car) > speed - 0.3,
        "do not repeatedly apply curb drag off the asphalt"
    );
    let position = car.position;
    let heading = car.orientation.yaw;
    physics::car::reset(&mut car, position, heading);
    assert_eq!(car.curb_bump, 0.0);
}

#[test]
fn reverse_acceleration_terminal_speed_and_braking_are_route_independent() {
    for model in test::support::cars() {
        let mut compact = level_route_car(world::locations::LocationId::Compact, model.clone());
        let mut scenic = level_route_car(test::support::authored_id(), model.clone());
        select_reverse(&mut compact);
        select_reverse(&mut scenic);
        // Projection rounding can change the phase of repeated limiter
        // cycles. Keep the tight comparison until the limiter first cuts,
        // then allow its 250 RPM band plus one substep of tire acceleration.
        let mut tolerance = 0.01;
        let limiter_tolerance = speed_at_rpm(&model, 250.0, REVERSE_GEAR) + 9.81 / 240.0;
        for _ in 0..240 * 12 {
            for car in [&mut compact, &mut scenic] {
                treadmill_tick(
                    car,
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            if compact.engine.fuel_cut || scenic.engine.fuel_cut {
                tolerance = limiter_tolerance;
            }
            assert!(
                (longitudinal_speed(&compact) - longitudinal_speed(&scenic)).abs() < tolerance,
                "{model:?}: {} vs {}, slip {} vs {}",
                longitudinal_speed(&compact),
                longitudinal_speed(&scenic),
                compact.slip,
                scenic.slip
            );
        }
        assert!(longitudinal_speed(&scenic) < -6.0 && longitudinal_speed(&scenic).is_finite());
        let (mut compact_distance, mut scenic_distance) = (0.0, 0.0);
        for _ in 0..720 {
            let brake = Controls {
                brake: 1.0,
                ..Controls::default()
            };
            compact_distance += treadmill_tick(&mut compact, brake);
            scenic_distance += treadmill_tick(&mut scenic, brake);
            assert!(
                (longitudinal_speed(&compact) - longitudinal_speed(&scenic)).abs() < tolerance,
                "{model:?}: {} vs {}, slip {} vs {}",
                longitudinal_speed(&compact),
                longitudinal_speed(&scenic),
                compact.slip,
                scenic.slip
            );
        }
        assert!(
            longitudinal_speed(&compact).abs() < 0.001 && longitudinal_speed(&scenic).abs() < 0.001
        );
        // The limiter can leave a small speed difference when braking
        // starts; allow that difference over the three-second stop window.
        assert!((compact_distance - scenic_distance).abs() < tolerance * 3.0);
        assert_eq!(scenic.gear, REVERSE_GEAR);
    }
}

#[test]
fn offroad_driving_and_reentry_preserve_position_and_follow_terrain() {
    let track = world::locations::try_track(&test::support::authored_id()).unwrap();
    let road = world::locations::at_distance(&track, 50.0);
    for side in [-1.0, 1.0] {
        let position =
            physics::vehicle_location::vehicle_position(road, side * (road.width * 0.5 + 8.0));
        let inward = util::geometry::xz_map(model::world::normal(road), |v| -side * v);
        let mut car = physics::car::on_track(
            position,
            inward.x.atan2(inward.z),
            test::support::car(),
            track.clone(),
        );
        car.gear = NEUTRAL_GEAR;
        car.velocity =
            util::geometry::at_height(util::geometry::xz_map(inward, |v| v * 10.0), car.velocity.y);
        assert_eq!(physics::car::resolve_road(&mut (car)), 0.0);
        assert_eq!([car.position.x, car.position.z], [position.x, position.z]);
        for _ in 0..288 {
            assert!(
                physics::car::update(
                    &mut (car),
                    Duration::from_secs_f64(1.0 / 240.0),
                    Controls::default()
                ) < 0.01
            );
            assert!(util::geometry::iter(&(car.position)).all(|v| v.is_finite()));
        }
        assert!(world::roads::project(&track, car.position).offset.abs() < road.width * 0.5);
        assert!((car.position.y - suspension::ride_height(&(car))).abs() < 0.15);
    }
}

#[test]
fn scenic_recovery_preserves_route_and_model_but_clears_motion_and_reverse() {
    let mut car = route_car(test::support::authored_id(), test::support::car());
    let model = car.spec.clone();
    let route = car.track.clone();
    select_reverse(&mut car);
    treadmill_tick(
        &mut car,
        Controls {
            throttle: 1.0,
            steering: 1.0,
            ..Controls::default()
        },
    );
    car.tire_scrub = 1.0;
    let road = world::locations::at_distance(&car.track, 850.0);
    physics::car::reset(
        &mut car,
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
    );
    assert!(std::sync::Arc::ptr_eq(&car.track, &route));
    assert!(std::sync::Arc::ptr_eq(&car.spec, &model));
    assert_eq!(
        car.position,
        physics::vehicle_location::vehicle_position(road, 0.0)
    );
    assert_eq!(
        car.orientation,
        Orientation {
            yaw: model::world::heading(road),
            ..Orientation::default()
        }
    );
    assert_eq!(
        car.velocity,
        XYZ {
            y: 0.0,
            x: 0.0,
            z: 0.0
        }
    );
    assert_eq!(car.gear, 1);
    assert_eq!(car.orientation_rate, Orientation::default());
    assert_eq!(car.acceleration, XYZ::default());
    assert_eq!(longitudinal_speed(&car), 0.0);
    assert_eq!(car.slip, 0.0);
    assert_eq!(car.engine.rpm, IDLE_RPM);
    assert_eq!(car.engine.load, 0.0);
    assert_eq!(car.wheel_phase, [0.0; 2]);
    assert_eq!(physics::car::front_wheel_angle(&(car)), 0.0);
    assert_eq!(car.tire_scrub, 0.0);
}

#[test]
fn outer_terrain_boundary_preserves_tangential_motion_and_ground_height() {
    let track = world::locations::try_track(&world::locations::LocationId::Compact).unwrap();
    let bounds = world::elevation::terrain_bounds(&track);
    let mut car = physics::car::on_track(
        XYZ {
            x: bounds.min.x - 1.0,
            y: 0.0,
            z: (bounds.min.z + bounds.max.z) * 0.5,
        },
        0.0,
        test::support::car(),
        track.clone(),
    );
    car.velocity = XYZ {
        y: car.velocity.y,
        x: -4.0,
        z: 8.0,
    };
    assert!(physics::car::resolve_road(&mut (car)) > 0.0);
    assert_eq!(car.position.x, bounds.min.x + world::locations::CAR_RADIUS);
    assert!(car.velocity.z > 6.0);
    assert!(
        (car.position.y
            - physics::vehicle_location::drive_height(
                &track,
                XZ {
                    x: car.position.x,
                    z: car.position.z
                }
            )
            - 0.48)
            .abs()
            < 0.001
    );
}

#[test]
fn holding_brakes_at_rest_does_not_dive_and_settles_after_stopping() {
    for handbrake in [false, true] {
        for initial_speed in [0.0, 8.0, -8.0] {
            let mut v = car();
            let forward = XZ {
                x: v.orientation.yaw.sin(),
                z: v.orientation.yaw.cos(),
            };
            v.velocity = util::geometry::at_height(
                util::geometry::xz_map(forward, |component| component * initial_speed),
                v.velocity.y,
            );
            physics::car::refresh_slip(&mut v);
            for _ in 0..600 {
                physics::car::update(
                    &mut v,
                    Duration::from_secs_f64(1.0 / 60.0),
                    Controls {
                        brake: if handbrake { 0.0 } else { 1.0 },
                        handbrake,
                        ..Controls::default()
                    },
                );
                if initial_speed == 0.0 {
                    assert!(v.orientation.pitch.abs() < 0.0001);
                }
            }
            assert!(longitudinal_speed(&v).abs() < 0.001);
            assert!(v.orientation.pitch.abs() < 0.0001);
        }
    }
}

#[test]
fn body_motion_handles_reverse_and_frame_rate_independent_settling() {
    let mut forward = car();
    let mut reverse = car();
    set_longitudinal_speed(&mut forward, 6.0);
    set_longitudinal_speed(&mut reverse, -6.0);
    forward.filtered_acceleration.longitudinal = 3.0;
    reverse.filtered_acceleration.longitudinal = -3.0;
    for _ in 0..240 {
        suspension::update_body_motion(&mut forward, 1.0 / 240.0);
        suspension::update_body_motion(&mut reverse, 1.0 / 240.0);
    }
    assert!(forward.orientation.pitch > 0.005);
    assert!((forward.orientation.pitch + reverse.orientation.pitch).abs() < 0.00001);
    reverse.filtered_acceleration.longitudinal = 4.0;
    for _ in 0..240 {
        suspension::update_body_motion(&mut reverse, 1.0 / 240.0);
    }
    assert!(
        reverse.orientation.pitch > 0.01,
        "reverse braking should lift the nose"
    );
    forward.filtered_acceleration = suspension::LocalAcceleration::default();
    reverse.filtered_acceleration = suspension::LocalAcceleration::default();
    forward.orientation.pitch = 0.05;
    reverse.orientation.pitch = 0.05;
    for _ in 0..30 {
        suspension::update_body_motion(&mut forward, 1.0 / 30.0);
    }
    for _ in 0..144 {
        suspension::update_body_motion(&mut reverse, 1.0 / 144.0);
    }
    assert!((forward.orientation.pitch - reverse.orientation.pitch).abs() < 0.0002);
    assert!(forward.orientation.pitch.abs() < 0.0005);
}

#[test]
fn rolling_engine_rpm_follows_gearing_without_a_throttle_floor() {
    for model in test::support::cars() {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());

        let speed = speed_at_rpm(&model, 1800.0, 1);
        car.velocity = XYZ {
            y: car.velocity.y,
            x: car.orientation.yaw.sin() * speed,
            z: car.orientation.yaw.cos() * speed,
        };
        for tick in 0..60 {
            treadmill_tick(
                &mut car,
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            if tick == 0 {
                assert!(
                    car.engine.rpm < 2200.0,
                    "throttle must not impose a launch RPM floor"
                );
            }
            let shaft_rpm =
                car.driven_wheel_omega * engine::rpm(&model, 1.0, 1) * model.chassis.rolling_radius;
            assert!(
                (car.engine.rpm - shaft_rpm).abs() < 1.0,
                "{model:?}: clutch lock must determine crankshaft RPM"
            );
        }
    }
}

#[test]
fn shifts_release_the_clutch_and_road_speed_can_mechanically_overrev_the_engine() {
    let model = test::support::car();
    let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
    car.gear = 2;
    let speed = speed_at_rpm(&model, 4000.0, 2);
    car.velocity = XYZ {
        y: car.velocity.y,
        x: car.orientation.yaw.sin() * speed,
        z: car.orientation.yaw.cos() * speed,
    };
    treadmill_tick(&mut car, Controls::default());
    physics::car::shift(&mut car, true);
    let before = car.engine.rpm;
    for _ in 0..12 {
        treadmill_tick(
            &mut car,
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
    }
    assert!(
        car.engine.rpm < before,
        "engine coasts while the clutch is open"
    );
    assert!(
        physics::car::drivetrain_step(&(car), 1.0, 1.0 / 240.0, longitudinal_speed(&car)).is_none()
    );
    for _ in 0..240 {
        treadmill_tick(
            &mut car,
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
    }
    assert!((car.engine.rpm - physics::car::coupled_engine_rpm(&(car))).abs() < 10.0);

    let mut overrev = level_route_car(world::locations::LocationId::Compact, model.clone());

    let speed = speed_at_rpm(&model, model.engine.redline_rpm + 500.0, 1);
    overrev.velocity = XYZ {
        y: overrev.velocity.y,
        x: overrev.orientation.yaw.sin() * speed,
        z: overrev.orientation.yaw.cos() * speed,
    };
    treadmill_tick(&mut overrev, Controls::default());
    assert!(overrev.engine.rpm > model.engine.redline_rpm);
    assert!(
        overrev.engine.fuel_cut,
        "fuel cut cannot clamp wheel-driven engine speed"
    );
}

#[test]
fn drivetrain_acceleration_and_rpm_are_consistent_across_frame_rates() {
    let run = |hz: u32| {
        let mut car = level_route_car(world::locations::LocationId::Compact, test::support::car());
        for _ in 0..hz * 4 {
            car.position = physics::vehicle_location::vehicle_position(
                world::locations::at_distance(&car.track, 50.0),
                0.0,
            );
            physics::car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / hz as f64),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        (longitudinal_speed(&car), car.engine.rpm)
    };
    let reference = run(240);
    for hz in [30, 60, 144] {
        let actual = run(hz);
        assert!(
            (actual.0 - reference.0).abs() < 0.05,
            "{hz} Hz: speed {actual:?}, reference {reference:?}"
        );
        assert!(
            (actual.1 - reference.1).abs() < 20.0,
            "{hz} Hz: RPM {actual:?}, reference {reference:?}"
        );
    }
}

#[test]
fn neutral_revs_build_progressively_cycle_limiter_and_fall_to_idle() {
    let road = world::locations::at_distance(
        &world::locations::try_track(&world::locations::LocationId::Compact).unwrap(),
        50.0,
    );
    for model in test::support::cars() {
        let mut car = new(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            model.clone(),
        );

        physics::car::shift(&mut car, false);
        let dt = Duration::from_secs_f64(1.0 / 60.0);
        let pedal = Controls {
            throttle: 1.0,
            ..Controls::default()
        };
        for _ in 0..6 {
            physics::car::update(&mut car, dt, pedal);
        }
        assert!(car.engine.rpm > IDLE_RPM + 100.0);
        assert!(car.engine.rpm < 2500.0, "{model:?}: blip jumps too quickly");
        let mut cuts = 0;
        let mut recoveries = 0;
        for _ in 0..300 {
            let was_cut = car.engine.fuel_cut;
            physics::car::update(&mut car, dt, pedal);
            cuts += usize::from(!was_cut && car.engine.fuel_cut);
            recoveries += usize::from(was_cut && !car.engine.fuel_cut);
            // A discrete combustion step can cross redline before fuel cuts.
            assert!((IDLE_RPM..=car.spec.engine.redline_rpm + 100.0).contains(&car.engine.rpm));
        }
        assert!(cuts > 2 && recoveries > 2, "{model:?}: limiter must cycle");
        physics::car::update(&mut car, dt, Controls::default());
        assert!(
            car.engine.rpm > 5500.0,
            "release must retain engine inertia"
        );
        for _ in 0..300 {
            let before = car.engine.rpm;
            physics::car::update(&mut car, dt, Controls::default());
            assert!(car.engine.rpm <= before);
        }
        assert_eq!(car.engine.rpm, IDLE_RPM);
        assert_eq!(car.gear, NEUTRAL_GEAR);
        assert!(
            longitudinal_speed(&car).abs() < 0.01,
            "neutral must not propel the car"
        );
        assert!(physics::car::drivetrain_step(&(car), 1.0, 1.0 / 240.0, 0.0).is_none());
        assert_eq!(car.engine.turbo_boost, 0.0);
    }
}

#[test]
fn fuel_limiter_uses_the_same_thresholds_in_neutral_forward_and_reverse() {
    let road = world::locations::at_distance(
        &world::locations::try_track(&world::locations::LocationId::Compact).unwrap(),
        50.0,
    );
    let pedal = Controls {
        throttle: 1.0,
        ..Controls::default()
    };
    let custom_models = [5800.0, 7400.0].map(|redline_rpm| {
        let mut spec = test::support::car_spec().clone();
        spec.engine.redline_rpm = redline_rpm;
        std::sync::Arc::new(spec)
    });
    for model in test::support::cars().into_iter().chain(custom_models) {
        for gear in [NEUTRAL_GEAR, 1, REVERSE_GEAR] {
            let mut car = new(
                physics::vehicle_location::vehicle_position(road, 0.0),
                model::world::heading(road),
                model.clone(),
            );
            car.gear = gear;
            for (rpm, cut) in [
                (car.spec.engine.redline_rpm + 1.0, true),
                (car.spec.engine.redline_rpm - 100.0, true),
                (car.spec.engine.redline_rpm - 251.0, false),
                (car.spec.engine.redline_rpm - 100.0, false),
            ] {
                car.engine.rpm = rpm;
                set_longitudinal_speed(&mut car, speed_at_rpm(&model, rpm, gear));
                let engine_rpm = car.engine.rpm;
                physics::car::update_fuel_cut(&mut car, engine_rpm);
                assert_eq!(
                    car.engine.fuel_cut, cut,
                    "{model:?}, gear {gear}, RPM {rpm}"
                );
                if gear != NEUTRAL_GEAR {
                    assert_eq!(physics::car::engine_net_torque(&(car), 1.0) <= 0.0, cut);
                }
            }
            car.engine.fuel_cut = true;
            car.engine.load = 1.0;
            car.engine.rpm = car.spec.engine.redline_rpm;
            let speed = speed_at_rpm(&model, car.spec.engine.redline_rpm + 1.0, gear);
            set_longitudinal_speed(&mut car, speed);
            physics::car::update_engine_feedback(&mut car, pedal, 0.01);
            assert!(car.engine.load < 1.0, "fuel cut must reach engine audio");
        }
    }
}

#[test]
fn in_gear_limiter_cycles_under_power_and_survives_a_shift_to_neutral() {
    let model = test::support::car();
    for gear in [REVERSE_GEAR, 1] {
        let mut car = level_route_car(world::locations::LocationId::Compact, model.clone());
        car.gear = gear;
        let speed = speed_at_rpm(&model, car.spec.engine.redline_rpm + 20.0, gear)
            * if gear == REVERSE_GEAR { -1.0 } else { 1.0 };
        car.velocity = XYZ {
            y: car.velocity.y,
            x: car.orientation.yaw.sin() * speed,
            z: car.orientation.yaw.cos() * speed,
        };
        let mut cuts = 0;
        let mut recoveries = 0;
        for _ in 0..240 * 4 {
            let was_cut = car.engine.fuel_cut;
            treadmill_tick(
                &mut car,
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            cuts += usize::from(!was_cut && car.engine.fuel_cut);
            recoveries += usize::from(was_cut && !car.engine.fuel_cut);
        }
        assert!(
            cuts > 2 && recoveries > 2,
            "gear {gear}: limiter must cycle"
        );
        car.engine.fuel_cut = true;
        car.engine.rpm = car.spec.engine.redline_rpm - 100.0;
        physics::car::shift(&mut car, gear == REVERSE_GEAR);
        assert_eq!(car.gear, NEUTRAL_GEAR);
        physics::car::update_engine_feedback(&mut car, Controls::default(), 0.0);
        assert!(
            car.engine.fuel_cut,
            "shifting must not reset limiter hysteresis"
        );
    }
}

#[test]
fn neutral_partial_throttle_balances_drag_independently_of_road_speed() {
    let road = world::locations::at_distance(
        &world::locations::try_track(&world::locations::LocationId::Compact).unwrap(),
        50.0,
    );
    for model in test::support::cars() {
        let mut car = new(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            model.clone(),
        );
        car.gear = NEUTRAL_GEAR;
        let pedal = Controls {
            throttle: 0.35,
            ..Controls::default()
        };
        for _ in 0..600 {
            physics::car::update_free_engine(&mut car, pedal.throttle, 1.0 / 60.0);
        }
        let stationary = car.engine.rpm;
        assert!(stationary > IDLE_RPM && stationary < model.engine.redline_rpm);
        let mut rolling = new(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            model,
        );
        rolling.gear = NEUTRAL_GEAR;
        rolling.engine.rpm = stationary;
        set_longitudinal_speed(&mut rolling, 30.0);
        for _ in 0..120 {
            physics::car::update_free_engine(&mut car, pedal.throttle, 1.0 / 60.0);
            physics::car::update_free_engine(&mut rolling, pedal.throttle, 1.0 / 60.0);
            assert_eq!(car.engine.rpm, rolling.engine.rpm);
        }
        car.gear = 1;
        car.engine.fuel_cut = true;
        physics::car::update_free_engine(&mut car, pedal.throttle, 1.0 / 60.0);
        assert!(
            !car.engine.fuel_cut,
            "RPM below the recovery threshold clears the limiter"
        );
        physics::car::reset(
            &mut car,
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
        );
        assert!(!car.engine.fuel_cut);
    }
}

#[test]
fn neutral_blips_are_consistent_across_frame_rates() {
    let mut results = Vec::new();
    for hz in [30, 60, 144, 240] {
        let mut car = new(
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            test::support::car(),
        );
        car.gear = NEUTRAL_GEAR;
        for _ in 0..hz / 2 {
            physics::car::update(
                &mut car,
                Duration::from_secs_f64(1.0 / hz as f64),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
        }
        results.push(car.engine.rpm);
    }
    let min = results.iter().cloned().fold(f32::INFINITY, f32::min);
    let max = results.iter().cloned().fold(0.0, f32::max);
    assert!(
        max - min < 20.0,
        "frame rates changed blip response: {results:?}"
    );
}

#[test]
fn engine_feedback_tracks_load_and_shift_unload() {
    let road = world::locations::at_distance(
        &world::locations::try_track(&world::locations::LocationId::Compact).unwrap(),
        50.0,
    );
    let mut car = new(
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
        test::support::car(),
    );
    assert_eq!(car.engine.rpm, IDLE_RPM);
    assert_eq!(car.engine.load, 0.0);
    let mut previous_speed = 0.0;
    for _ in 0..60 {
        car.position = physics::vehicle_location::vehicle_position(road, 0.0);
        physics::car::update(
            &mut car,
            Duration::from_secs_f64(1.0 / 60.0),
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
        assert!(longitudinal_speed(&car) >= previous_speed - 0.02);
        previous_speed = longitudinal_speed(&car);
    }
    assert!(car.engine.rpm > IDLE_RPM);
    assert!(car.engine.rpm < car.spec.engine.redline_rpm);
    for _ in 0..30 {
        car.position = physics::vehicle_location::vehicle_position(road, 0.0);
        physics::car::update(
            &mut car,
            Duration::from_secs_f64(1.0 / 60.0),
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
    }
    assert!(car.engine.load > 0.45);

    let load_before_shift = car.engine.load;
    physics::car::shift(&mut car, true);
    physics::car::update(
        &mut car,
        Duration::from_secs_f64(1.0 / 60.0),
        Controls {
            throttle: 1.0,
            ..Controls::default()
        },
    );
    assert_eq!(car.gear, 2);
    assert!(
        car.engine.load < load_before_shift,
        "shift should briefly unload the engine: {} -> {}",
        load_before_shift,
        car.engine.load
    );

    physics::car::reset(
        &mut car,
        physics::vehicle_location::vehicle_position(road, 0.0),
        model::world::heading(road),
    );
    select_reverse(&mut car);
    for _ in 0..60 {
        car.position = physics::vehicle_location::vehicle_position(road, 0.0);
        physics::car::update(
            &mut car,
            Duration::from_secs_f64(1.0 / 60.0),
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
    }
    assert_eq!(car.gear, REVERSE_GEAR);
    assert!(car.engine.rpm > IDLE_RPM);
    assert!(car.engine.rpm <= car.spec.engine.redline_rpm);
}

fn select_reverse(vehicle: &mut Vehicle) {
    physics::car::shift(vehicle, false);
    physics::car::shift(vehicle, false);
    assert_eq!(vehicle.gear, REVERSE_GEAR);
}
