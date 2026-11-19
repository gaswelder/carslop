use super::{solve, Contact, Impulse, Motion};

const CONTACT: Contact = Contact {
    offset: 0.0,
    sin: 0.0,
    cos: 1.0,
    capacity: 6000.0,
    drive_force: 0.0,
    longitudinal_slip: 0.0,
    brake_force: 0.0,
    corner_stiffness: 60000.0,
    locked: false,
};

fn force(contact: Contact, motion: Motion) -> Impulse {
    let dt = 1e-5;
    let result = solve(
        motion,
        motion,
        [
            contact,
            Contact {
                capacity: 0.0,
                ..CONTACT
            },
        ],
        1200.0,
        1600.0,
        dt,
    );
    Impulse {
        longitudinal: result.impulses[0].longitudinal / dt,
        lateral: result.impulses[0].lateral / dt,
    }
}

#[test]
fn unloaded_axle_produces_no_force() {
    let result = force(
        Contact {
            capacity: 0.0,
            corner_stiffness: super::corner_stiffness(0.0),
            ..CONTACT
        },
        Motion {
            longitudinal: 20.0,
            lateral: 2.0,
            yaw_rate: 0.0,
        },
    );
    assert_eq!(result.longitudinal, 0.0);
    assert_eq!(result.lateral, 0.0);
}

#[test]
fn developed_slides_share_the_same_grip_in_every_direction() {
    for direction in [-1.0, 1.0] {
        let cornering = force(
            CONTACT,
            Motion {
                longitudinal: 20.0,
                lateral: 20.0 * direction,
                yaw_rate: 0.0,
            },
        );
        let spinning = force(
            Contact {
                drive_force: 10000.0 * direction,
                longitudinal_slip: 20.0 * direction,
                ..CONTACT
            },
            Motion {
                longitudinal: 20.0 * direction,
                ..Motion::default()
            },
        );
        let locked = force(
            Contact {
                locked: true,
                brake_force: 10000.0,
                ..CONTACT
            },
            Motion {
                longitudinal: 12.0 * direction,
                lateral: 16.0,
                yaw_rate: 0.0,
            },
        );
        for impulse in [cornering, spinning, locked] {
            assert!((impulse.longitudinal.hypot(impulse.lateral) - 4800.0).abs() < 1.0);
        }
        assert!(cornering.lateral * direction < 0.0);
        assert!(spinning.longitudinal * direction > 0.0);
        assert!(locked.longitudinal * direction < 0.0 && locked.lateral < 0.0);
    }
}

#[test]
fn wheelspin_reduces_the_shared_cornering_and_braking_budget() {
    let motion = Motion {
        longitudinal: 20.0,
        lateral: 2.0,
        yaw_rate: 0.0,
    };
    let rolling = force(
        Contact {
            brake_force: 3000.0,
            ..CONTACT
        },
        motion,
    );
    let spinning = force(
        Contact {
            brake_force: 3000.0,
            longitudinal_slip: 20.0,
            ..CONTACT
        },
        motion,
    );
    assert!(spinning.longitudinal.hypot(spinning.lateral) <= 4800.1);
    assert!(spinning.lateral.abs() < rolling.lateral.abs());
    assert!(spinning.longitudinal.abs() < rolling.longitudinal.abs());
}

#[test]
fn longitudinal_slip_loses_grip_gradually() {
    let motion = Motion {
        longitudinal: 20.0,
        ..Motion::default()
    };
    let traction = |slip| {
        force(
            Contact {
                drive_force: 10000.0,
                longitudinal_slip: slip,
                ..CONTACT
            },
            motion,
        )
        .longitudinal
    };
    let peak = traction(0.0);
    let onset = traction(2.0);
    let partial = traction(5.0);
    let sliding = traction(20.0);
    assert!((peak - 6000.0).abs() < 1.0);
    assert!((onset - peak).abs() < 1.0);
    assert!(peak > partial && partial > sliding);
}

#[test]
fn combined_slip_can_pass_the_peak_before_either_component_does() {
    let motion = Motion {
        longitudinal: 20.0,
        lateral: 1.6,
        yaw_rate: 0.0,
    };
    let rolling = force(
        Contact {
            drive_force: 10000.0,
            ..CONTACT
        },
        motion,
    );
    let combined = force(
        Contact {
            drive_force: 10000.0,
            longitudinal_slip: 1.6,
            ..CONTACT
        },
        motion,
    );
    assert!((rolling.longitudinal.hypot(rolling.lateral) - 6000.0).abs() < 1.0);
    assert!(combined.longitudinal < rolling.longitudinal - 10.0);
    assert!(combined.longitudinal.hypot(combined.lateral) > 4800.0);
}

#[test]
fn powered_and_engine_braked_tires_retain_cornering_force_at_saturation() {
    for drive in [-10000.0_f32, 10000.0] {
        let result = force(
            Contact {
                drive_force: drive,
                longitudinal_slip: 20.0 * drive.signum(),
                ..CONTACT
            },
            Motion {
                longitudinal: 20.0,
                lateral: 10.0,
                yaw_rate: 0.0,
            },
        );
        assert!(result.longitudinal * drive > 0.0);
        assert!(result.lateral < -1000.0);
        assert!(result.longitudinal.hypot(result.lateral) <= 4800.1);
    }
}

#[test]
fn stopped_locked_contact_holds_above_sliding_capacity() {
    let dt = 1.0 / 240.0;
    let result = solve(
        Motion::default(),
        Motion {
            longitudinal: 5700.0 / 1200.0 * dt,
            ..Motion::default()
        },
        [
            Contact {
                locked: true,
                brake_force: 10000.0,
                ..CONTACT
            },
            Contact {
                capacity: 0.0,
                ..CONTACT
            },
        ],
        1200.0,
        1600.0,
        dt,
    );
    assert!(result.motion.longitudinal.abs() < 1e-6);
    assert!((result.impulses[0].longitudinal.abs() / dt - 5700.0).abs() < 1.0);
}
