//! Tests through the group's production interfaces.

mod tire_solver {
    use crate::physics::tire_solver::*;

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
            longitudinal_slip: 0.0,
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
            longitudinal_slip: 0.0,
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
                longitudinal_slip: 0.0,
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
    use crate::driver;
    use crate::physics::car::{Controls, Vehicle};
    use crate::physics::powertrain::{NEUTRAL_GEAR, REVERSE_GEAR};
    use crate::util::geometry::XYZ;
    use crate::{model, physics, test, world};
    use std::time::Duration;

    fn select_reverse(vehicle: &mut Vehicle) {
        driver::shift(vehicle, false);
        driver::shift(vehicle, false);
        assert_eq!(vehicle.gearbox.gear, REVERSE_GEAR);
    }
    fn select_drive(vehicle: &mut Vehicle) {
        driver::shift(vehicle, true);
        driver::shift(vehicle, true);
        assert_eq!(vehicle.gearbox.gear, 1);
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
        car.chassis.velocity = XYZ {
            y: car.chassis.velocity.y,
            x: 0.0,
            z: 40.0,
        };
        assert_eq!(car.gearbox.gear, 1);
        driver::shift(&mut car, true);
        assert_eq!(car.gearbox.gear, 2);
        car.chassis.velocity = XYZ {
            y: car.chassis.velocity.y,
            x: 0.0,
            z: 0.0,
        };
        assert_eq!(car.gearbox.gear, 2);
        driver::shift(&mut car, false);
        driver::shift(&mut car, false);
        assert_eq!(car.gearbox.gear, NEUTRAL_GEAR);
        car.chassis.velocity = XYZ {
            y: car.chassis.velocity.y,
            x: 2.0,
            z: 0.0,
        };
        driver::shift(&mut car, false);
        assert_eq!(car.gearbox.gear, REVERSE_GEAR);
        assert_eq!(
            car.chassis.velocity,
            XYZ {
                y: 0.0,
                x: 2.0,
                z: 0.0
            }
        );
        driver::shift(&mut car, true);
        assert_eq!(car.gearbox.gear, NEUTRAL_GEAR);
        driver::shift(&mut car, true);
        assert_eq!(car.gearbox.gear, 1);
        for _ in 0..20 {
            driver::shift(&mut car, true);
        }
        assert_eq!(car.gearbox.gear, car.spec.gearbox.ratios.len() as u8);
        physics::car::reset(
            &mut car,
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
        );
        assert_eq!(car.gearbox.gear, 1);
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
            assert_eq!(v.gearbox.gear, REVERSE_GEAR);
            for _ in 0..720 {
                v.chassis.position = physics::vehicle_location::vehicle_position(road, 0.0); // straight-line test treadmill
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
                physics::car::longitudinal_speed(&v) < -6.0
                    && physics::car::longitudinal_speed(&v).is_finite(),
                "reverse speed for {model:?}: {}",
                physics::car::longitudinal_speed(&v)
            );
            assert_eq!(v.gearbox.gear, REVERSE_GEAR);

            physics::car::update(
                &mut v,
                Duration::from_secs(3),
                Controls {
                    brake: 1.0,
                    ..Controls::default()
                },
            );
            assert!(physics::car::longitudinal_speed(&v).abs() < 0.001);
            assert_eq!(v.gearbox.gear, REVERSE_GEAR);
            select_drive(&mut v);
            assert_eq!(v.gearbox.gear, 1);
            physics::car::update(
                &mut v,
                Duration::from_millis(500),
                Controls {
                    throttle: 1.0,
                    ..Controls::default()
                },
            );
            assert!(
                physics::car::longitudinal_speed(&v) > 0.5,
                "{model:?} failed to pull away: {}, engine {}, wheel {}",
                physics::car::longitudinal_speed(&v),
                v.engine.rpm,
                v.tires.wheel_spin
            );
        }
    }
}
