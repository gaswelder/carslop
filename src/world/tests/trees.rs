use super::*;
use crate::test;

fn location(compiled: bool) -> WorldLocation {
    let mut world = model::world::empty();
    world.elevation = Some(world::elevation::generate(
        util::geometry::BoundingBoxXZ {
            min: XZ { x: -50., z: -50. },
            max: XZ { x: 50., z: 50. },
        },
        |_| 10.,
    ));
    if compiled {
        let (surfaces, _) = world::street_surface::from_geometry(
            model::location::Streets {
                paths: vec![],
                triangles: vec![],
                curbs: vec![],
            },
            world::street_surface::DEFAULT_PAVEMENT_HEIGHT,
        )
        .unwrap();
        test::support::install_surfaces(&mut world, surfaces);
    }
    world.ground = world::landscape::new(
        vec![],
        vec![Tree {
            id: 1,
            coords: XZ { x: 0., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 4.,
        }],
    );
    world
}

#[test]
fn direct_and_glancing_hits_deflect_but_foliage_does_not_block() {
    for compiled in [false, true] {
        let world = location(compiled);
        for lateral in [0., 15.] {
            let mut position = XYZ {
                x: -1.,
                y: 10.48,
                z: 0.,
            };
            let mut velocity = XZ { x: 20., z: lateral };
            let impact = constrain(&world, &mut position, &mut velocity, 1.7);
            assert_eq!(impact, 20.);
            assert!(velocity.x < 0.);
            if lateral > 0. {
                assert!(velocity.z > 0.9 * lateral);
            }
            assert!(position.x < -world::locations::CAR_RADIUS);
            assert_eq!(constrain(&world, &mut position, &mut velocity, 1.7), 0.);
        }
        let mut position = XYZ {
            x: -3.,
            y: 10.48,
            z: 0.,
        };
        let mut velocity = XZ { x: 20., z: 0. };
        assert_eq!(constrain(&world, &mut position, &mut velocity, 1.7), 0.);
        assert_eq!(position.x, -3.);
        assert_eq!(velocity.x, 20.);
    }
}

#[test]
fn height_separation_and_exact_centre_contacts_are_safe() {
    let world = location(true);
    for y in [5., 20.] {
        let mut position = XYZ { x: 0., y, z: 0. };
        let mut velocity = XZ { x: 20., z: 0. };
        assert_eq!(constrain(&world, &mut position, &mut velocity, 1.7), 0.);
        assert!(clearance(&world, position, 1.7).is_infinite());
        assert_eq!(position.x, 0.);
    }
    for speed in [0., 20.] {
        let mut position = XYZ {
            x: 0.,
            y: 10.48,
            z: 0.,
        };
        let mut velocity = XZ { x: speed, z: 0. };
        assert!(clearance(&world, position, 1.7) < 0.);
        assert_eq!(constrain(&world, &mut position, &mut velocity, 1.7), speed);
        assert!(position.x.is_finite() && position.x.abs() > world::locations::CAR_RADIUS);
    }
}

#[test]
fn vehicle_location_reports_tree_impacts() {
    let mut world = location(true);
    world.bounding_box = util::geometry::BoundingBoxXZ {
        min: XZ { x: -50., z: -50. },
        max: XZ { x: 50., z: 50. },
    };
    let mut position = XYZ {
        x: -1.,
        y: 10.48,
        z: 0.,
    };
    let mut velocity = XZ { x: 20., z: 0. };
    assert_eq!(
        physics::vehicle_location::constrain(&world, &mut position, &mut velocity, 1.7),
        20.
    );
    assert!(velocity.x < 0.);
}

#[test]
fn spawn_and_recovery_avoid_trunks() {
    for compiled in [false, true] {
        let mut world =
            test::support::parse_world("CITY 2\nn -30 0\nn 30 0\nr 0 1 8 0 1\n").unwrap();
        if compiled {
            let prepared = test::support::load_streets(
                concat!(
                    "STREETS 1 1\np 0 2 -30 0 30 0\n",
                    "t 1 -30 -4 30 -4 30 4\nt 1 -30 -4 30 4 -30 4\nc 1 -30 -4 30 -4 0 1\n",
                ),
                1,
            )
            .unwrap();
            test::support::install_surfaces(&mut world, prepared.surfaces);
        }
        world.ground = world::landscape::new(
            vec![],
            vec![Tree {
                id: 1,
                coords: XZ { x: 0., z: 0. },
                height_meters: 8.,
                crown_radius_meters: 4.,
            }],
        );
        let spawn = physics::vehicle_location::spawn(&world);
        let recovery = physics::vehicle_location::recovery(
            &world,
            XYZ {
                x: 0.,
                y: 0.48,
                z: 0.,
            },
        );
        for sample in [spawn, recovery] {
            assert!(sample.point.x.abs() > world::locations::CAR_RADIUS + 0.27);
        }
    }
}
