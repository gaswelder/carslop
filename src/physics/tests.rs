//! Tests through the group's production interfaces.

mod powertrain {
    use crate::physics;

    #[test]
    fn brakes_balance_tire_reaction_and_abs_prevents_wheel_lock() {
        for direction in [-1.0, 1.0] {
            let step = physics::powertrain::new(
                3000.0,
                20.0 * direction,
                0.0,
                0.25,
                12.0,
                0.0,
                1.0 / 240.0,
            );
            let rolling_speed = 19.5 * 0.32 * direction;
            let rotation = physics::powertrain::finish(
                &step,
                -4000.0 * direction,
                0.32,
                8000.0,
                rolling_speed,
            );
            assert!((rotation.wheel_omega * 0.32 - rolling_speed).abs() < 0.00001);
            assert!((rotation.engine_rpm - 3000.0).abs() < 0.001);

            let released =
                physics::powertrain::finish(&step, 0.0, 0.32, 8000.0, 25.0 * 0.32 * direction);
            assert!((released.wheel_omega - 20.0 * direction).abs() < 0.00001);
            let weak = physics::powertrain::finish(
                &step,
                -4000.0 * direction,
                0.32,
                1000.0,
                rolling_speed,
            );
            assert!(weak.wheel_omega * direction > 20.0);
        }
    }

    #[test]
    fn static_brake_and_clutch_exchange_torque_without_rotating_the_wheels() {
        let step = physics::powertrain::new(3000.0, 0.0, 0.0, 0.25, 12.0, 100.0, 1.0 / 240.0);
        let held = physics::powertrain::finish(&step, 0.0, 0.32, 8000.0, 0.0);
        assert!(held.wheel_omega.abs() < 0.00001);
        assert!(held.engine_rpm < 3000.0);
        let released = physics::powertrain::finish(&step, 0.0, 0.32, 0.0, 0.0);
        assert!(released.wheel_omega > 0.0);
    }

    #[test]
    fn locked_clutch_and_tires_preserve_gearing_and_exchange_momentum() {
        for dt in [1.0 / 60.0, 1.0 / 240.0] {
            let step = physics::powertrain::new(
                3000.0,
                3000.0 * std::f32::consts::TAU / 60.0 / 12.0,
                150.0,
                0.25,
                12.0,
                1000.0,
                dt,
            );
            let speed = (3000.0 * std::f32::consts::TAU / 60.0 / 12.0) * 0.32;
            let force = physics::powertrain::road_force(&step, speed, 1200.0, 0.32);
            let rotation = physics::powertrain::finish(&step, force, 0.32, 0.0, 0.0);
            let rpm = rotation.engine_rpm;
            let wheel = rotation.wheel_omega;
            assert!((wheel * 0.32 - (speed + force / 1200.0 * dt)).abs() < 0.0001);
            assert!((rpm - wheel * 12.0 * 60.0 / std::f32::consts::TAU).abs() < 0.01);
            let momentum_before = 0.25 * 3000.0 * std::f32::consts::TAU / 60.0
                + 2.4 * (3000.0 * std::f32::consts::TAU / 60.0 / 12.0) / 12.0;
            let momentum_after = 0.25 * rpm * std::f32::consts::TAU / 60.0 + 2.4 * wheel / 12.0;
            assert!(
                (momentum_after - momentum_before - (150.0 - force * 0.32 / 12.0) * dt).abs()
                    < 0.0001
            );
        }
    }

    #[test]
    fn slipping_clutch_transfers_momentum_and_dissipates_energy() {
        let step = physics::powertrain::new(6000.0, 0.0, 0.0, 0.25, 12.0, 400.0, 1.0 / 240.0);
        let rotation = physics::powertrain::finish(&step, 0.0, 0.32, 0.0, 0.0);
        let rpm = rotation.engine_rpm;
        let wheel = rotation.wheel_omega;
        let omega = rpm * std::f32::consts::TAU / 60.0;
        assert!(rpm < 6000.0 && wheel > 0.0);
        assert!(
            (0.25 * omega + 2.4 * wheel / 12.0 - 0.25 * (6000.0 * std::f32::consts::TAU / 60.0))
                .abs()
                < 0.0001
        );
        assert!(
            0.5 * 0.25 * omega * omega + 0.5 * 2.4 * wheel * wheel
                < 0.5
                    * 0.25
                    * (6000.0 * std::f32::consts::TAU / 60.0)
                    * (6000.0 * std::f32::consts::TAU / 60.0)
        );
    }

    #[test]
    fn lost_grip_accelerates_rotating_parts_and_open_clutch_isolates_engine() {
        let loaded = physics::powertrain::new(3000.0, 20.0, 150.0, 0.25, 12.0, 500.0, 1.0 / 240.0);
        let force = physics::powertrain::road_force(&loaded, 6.4, 1200.0, 0.32);
        let grip = physics::powertrain::finish(&loaded, force, 0.32, 0.0, 0.0);
        let slip = physics::powertrain::finish(&loaded, 0.0, 0.32, 0.0, 0.0);
        assert!(slip.engine_rpm > grip.engine_rpm && slip.wheel_omega > grip.wheel_omega);
        let open = physics::powertrain::new(3000.0, 20.0, 150.0, 0.25, 12.0, 0.0, 1.0 / 240.0);
        assert_eq!(
            physics::powertrain::finish(&(open), 0.0, 0.32, 0.0, 0.0).engine_rpm,
            physics::powertrain::finish(&(open), 5000.0, 0.32, 0.0, 0.0).engine_rpm
        );
    }
}

mod tire {
    use crate::physics::tire::*;

    const MASS: f32 = 1200.0;
    const INERTIA: f32 = 1600.0;
    const DT: f32 = 1.0 / 240.0;
    const CONTACTS: [Contact; 2] = [
        Contact {
            offset: 1.2,
            sin: 0.0,
            cos: 1.0,
            capacity: 6000.0,
            drive_force: 0.0,
            drive_limit: 6000.0,
            brake_force: 8000.0,
            corner_stiffness: 60000.0,
            locked: false,
        },
        Contact {
            offset: -1.4,
            sin: 0.0,
            cos: 1.0,
            capacity: 6000.0,
            drive_force: 0.0,
            drive_limit: 6000.0,
            brake_force: 4000.0,
            corner_stiffness: 80000.0,
            locked: false,
        },
    ];

    #[test]
    fn passive_contacts_dissipate_energy_and_respect_friction_and_brake_limits() {
        let energy = |motion: Motion| {
            0.5 * MASS * (motion.longitudinal.powi(2) + motion.lateral.powi(2))
                + 0.5 * INERTIA * motion.yaw_rate.powi(2)
        };
        for along in [-20.0, -0.01, 0.0, 0.01, 20.0] {
            for side in [-5.0, 0.0, 5.0] {
                for yaw in [-2.0, 0.0, 2.0] {
                    for steer in [-0.54_f32, 0.0, 0.54] {
                        for locked in [false, true] {
                            let mut contacts = CONTACTS;
                            contacts[0].sin = steer.sin();
                            contacts[0].cos = steer.cos();
                            contacts[1].locked = locked;
                            let motion = Motion {
                                longitudinal: along,
                                lateral: side,
                                yaw_rate: yaw,
                            };
                            let result = solve(motion, motion, contacts, MASS, INERTIA, DT);
                            assert!(energy(result.motion) <= energy(motion) + 0.05);
                            for (impulse, contact) in result.impulses.iter().zip(CONTACTS) {
                                assert!(
                                    impulse.longitudinal.hypot(impulse.lateral)
                                        <= contact.capacity * DT + 0.0001
                                );
                                assert!(
                                    impulse.longitudinal.abs() <= contact.brake_force * DT + 0.0001
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn static_contact_holds_a_slope_at_any_steering_angle_without_a_speed_clamp() {
        for steer in [-0.54_f32, 0.0, 0.54] {
            for gravity in [-2.0, 2.0] {
                for handbrake in [false, true] {
                    let mut contacts = CONTACTS;
                    contacts[0].sin = steer.sin();
                    contacts[0].cos = steer.cos();
                    if handbrake {
                        contacts[0].brake_force = 0.0;
                        contacts[1].locked = true;
                    }
                    let solution = solve(
                        Motion::default(),
                        Motion {
                            longitudinal: gravity * DT,
                            lateral: 0.3 * DT,
                            yaw_rate: 0.0,
                        },
                        contacts,
                        MASS,
                        INERTIA,
                        DT,
                    );
                    assert!(solution.motion.longitudinal.abs() < 0.000001);
                    assert!(solution.motion.lateral.abs() < 0.000001);
                    assert!(solution.motion.yaw_rate.abs() < 0.000001);
                }
            }
        }
    }

    #[test]
    fn insufficient_brake_force_or_tire_load_cannot_hold_a_slope() {
        for airborne in [false, true] {
            let contacts = CONTACTS.map(|mut contact| {
                if airborne {
                    contact.capacity = 0.0;
                    contact.drive_limit = 0.0;
                } else {
                    contact.brake_force = 100.0;
                }
                contact
            });
            let result = solve(
                Motion::default(),
                Motion {
                    longitudinal: 2.0 * DT,
                    ..Motion::default()
                },
                contacts,
                MASS,
                INERTIA,
                DT,
            );
            assert!(result.motion.longitudinal > DT);
        }
    }

    #[test]
    fn lateral_force_peaks_at_saturation_then_falls_off() {
        let force = |alpha: f32| {
            let contact = Contact {
                offset: 0.0,
                sin: 0.0,
                cos: 1.0,
                capacity: 6000.0,
                corner_stiffness: 60000.0,
                drive_force: 0.0,
                drive_limit: 0.0,
                brake_force: 0.0,
                locked: false,
            };
            let motion = Motion {
                longitudinal: 20.0,
                lateral: 20.0 * alpha,
                yaw_rate: 0.0,
            };
            let dt = 1e-5;
            let solution = solve(
                motion,
                motion,
                [
                    contact,
                    Contact {
                        capacity: 0.0,
                        ..CONTACTS[1]
                    },
                ],
                MASS,
                INERTIA,
                dt,
            );
            solution.impulses[0].lateral.abs() / dt
        };
        let peak = 0.1;
        assert!((force(peak) - 6000.0).abs() < 50.0);
        assert!(force(peak * 0.5) < force(peak));
        assert!(force(peak * 2.0) < force(peak));
        assert!(force(peak * 4.0) < force(peak * 2.0));
        assert!((force(0.001) - 60.0).abs() < 1.0);
    }
}

mod car {
    use crate::physics::car::{Controls, Vehicle, NEUTRAL_GEAR};
    use crate::util::geometry::{XYZ, XZ};
    use crate::{model, physics, test, world};
    use std::time::Duration;

    fn select_reverse(vehicle: &mut Vehicle) {
        physics::car::shift(vehicle, false);
        physics::car::shift(vehicle, false);
        assert_eq!(vehicle.gear, 0);
    }
    fn select_drive(vehicle: &mut Vehicle) {
        physics::car::shift(vehicle, true);
        physics::car::shift(vehicle, true);
        assert_eq!(vehicle.gear, 1);
    }

    #[test]
    fn manual_shifts_hold_gears_and_pass_through_neutral() {
        let mut car = test::support::vehicle(
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            test::support::car(),
        );
        car.manual = true;
        car.velocity = XZ { x: 0.0, z: 40.0 };
        assert_eq!(car.gear, 1);
        physics::car::shift(&mut car, true);
        assert_eq!(car.gear, 2);
        car.velocity = XZ { x: 0.0, z: 0.0 };
        assert_eq!(car.gear, 2);
        physics::car::shift(&mut car, false);
        physics::car::shift(&mut car, false);
        assert_eq!(car.gear, NEUTRAL_GEAR);
        car.velocity = XZ { x: 2.0, z: 0.0 };
        physics::car::shift(&mut car, false);
        assert_eq!(car.gear, 0);
        assert_eq!(car.velocity, XZ { x: 2.0, z: 0.0 });
        physics::car::shift(&mut car, true);
        assert_eq!(car.gear, NEUTRAL_GEAR);
        physics::car::shift(&mut car, true);
        assert_eq!(car.gear, 1);
        for _ in 0..20 {
            physics::car::shift(&mut car, true);
        }
        assert_eq!(car.gear, car.spec.gearbox.ratios.len() as u8);
        physics::car::reset(
            &mut car,
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
        );
        assert!(car.manual);
        assert_eq!(car.gear, 1);
    }
    #[test]
    fn calculated_top_speed_balances_power_and_road_load() {
        let mut spec = test::support::car_spec().clone();
        spec.turbo = None;
        spec.engine_max_torque_nm = 10_000.0;
        spec.gearbox.final_drive = 1.0;
        let speed = physics::car::calculate_top_speed(&spec);
        let road_power = (0.25 * spec.weight_kg
            + 0.5 * 1.225 * spec.drag_coefficient * spec.frontal_area_m2 * speed.powi(2))
            * speed;
        assert!((road_power - spec.engine_power_watts * 0.90).abs() < 1.0);
        spec.engine_power_watts *= 2.0;
        let more_power = physics::car::calculate_top_speed(&spec);
        assert!(more_power > speed);
        spec.drag_coefficient *= 2.0;
        assert!(physics::car::calculate_top_speed(&(spec)) < more_power);
        let before_weight_change = physics::car::calculate_top_speed(&spec);
        spec.weight_kg *= 2.0;
        assert!(physics::car::calculate_top_speed(&(spec)) < before_weight_change);
    }
    #[test]
    fn every_car_can_reverse_brake_and_drive_forward_again() {
        let road = world::locations::at_distance(
            &world::locations::try_track(&world::locations::LocationId::Compact).unwrap(),
            50.0,
        );
        for model in test::support::cars() {
            let mut v = test::support::vehicle(
                physics::vehicle_location::vehicle_position(road, 0.0),
                model::world::heading(road),
                model.clone(),
            );
            select_reverse(&mut v);
            assert_eq!(v.gear, 0);
            for _ in 0..720 {
                v.position = physics::vehicle_location::vehicle_position(road, 0.0); // straight-line test treadmill
                physics::car::update(
                    &mut v,
                    Duration::from_secs_f64(1.0 / 60.0),
                    Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    },
                );
            }
            assert!(
                v.speed < -6.0 && v.speed.is_finite(),
                "reverse speed for {model:?}: {}",
                v.speed
            );
            assert_eq!(v.gear, 0);

            physics::car::update(
                &mut v,
                Duration::from_secs(3),
                Controls {
                    brake: 1.0,
                    ..Controls::default()
                },
            );
            assert!(v.speed.abs() < 0.001);
            assert_eq!(v.gear, 0);
            select_drive(&mut v);
            assert_eq!(v.gear, 1);
            physics::car::update(
                &mut v,
                Duration::from_millis(500),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            assert!(
                v.speed > 0.5,
                "{model:?} failed to pull away: {}, engine {}, wheel {}",
                v.speed,
                v.engine_rpm,
                v.wheel_spin
            );
        }
    }
}
