use super::*;
use crate::model;
use crate::test;

#[test]
fn joined_bridges_keep_outer_rails_and_clip_only_covered_sections() {
    let mut city = model::world::empty();
    let mut data = model::map_details::Data::default();
    data.decks = vec![
        test::support::bridge_deck(0, 5., 0., 0., 30.),
        test::support::bridge_deck(1, 5.3, 6., 10., 20.),
    ];
    let deck = data.decks[0].clone();
    city.details = world::map_details::new(data);
    let edges = exposed_bridge_edge(
        &city,
        &deck,
        XYZ {
            x: 0.,
            y: 5.,
            z: 4.,
        },
        XYZ {
            x: 30.,
            y: 5.,
            z: 4.,
        },
    );
    assert_eq!(edges.len(), 2);
    assert!((edges[0].end.x - 10.).abs() < 0.001);
    assert!((edges[1].start.x - 20.).abs() < 0.001);
    let outer = exposed_bridge_edge(
        &city,
        &deck,
        XYZ {
            x: 0.,
            y: 5.,
            z: -4.,
        },
        XYZ {
            x: 30.,
            y: 5.,
            z: -4.,
        },
    );
    assert_eq!(outer.len(), 1);
    assert_eq!(outer[0].start.x, 0.);
    assert_eq!(outer[0].end.x, 30.);
}

#[test]
fn a_bridge_at_another_level_does_not_remove_rails() {
    let mut city = model::world::empty();
    let mut data = model::map_details::Data::default();
    data.decks = vec![
        test::support::bridge_deck(0, 5., 0., 0., 30.),
        test::support::bridge_deck(1, 12., 6., 10., 20.),
    ];
    let deck = data.decks[0].clone();
    city.details = world::map_details::new(data);
    let edges = exposed_bridge_edge(
        &city,
        &deck,
        XYZ {
            x: 0.,
            y: 5.,
            z: 4.,
        },
        XYZ {
            x: 30.,
            y: 5.,
            z: 4.,
        },
    );
    assert_eq!(edges.len(), 1);
    assert_eq!(edges[0].start.x, 0.);
    assert_eq!(edges[0].end.x, 30.);
}

#[test]
fn a_joined_lane_has_no_invisible_internal_rail_but_keeps_its_outer_barrier() {
    let mut city = model::world::empty();
    let mut data = model::map_details::Data::default();
    data.decks = vec![
        test::support::bridge_deck(0, 5., 0., 0., 30.),
        test::support::bridge_deck(1, 5.3, 6., 0., 30.),
    ];
    city.details = world::map_details::new(data);
    let mut position = XYZ {
        x: 15.,
        y: 5. + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
        z: 3.5,
    };
    let initial = position;
    let mut velocity = XZ { x: 0., z: 4. };
    assert_eq!(constrain_transport(&city, &mut position, &mut velocity), 0.);
    assert_eq!(position, initial);
    assert_eq!(velocity.z, 4.);
    position.z = 9.5;
    assert!(constrain_transport(&city, &mut position, &mut velocity) > 0.);
    assert!(position.z < 9.);
}

#[test]
fn a_level_approach_clears_the_bridge_entry_without_removing_rails_above_a_lower_road() {
    let directory = tempfile::tempdir().unwrap();
    let assets = test::support::stored_location(directory.path(), false);
    let mut city = world::preparation::world_location_load(&assets).unwrap();
    for height in [7., 12.] {
        let mut data = model::map_details::Data::default();
        let deck = test::support::bridge_deck(10, height, 6., 0., 30.);
        data.decks.push(deck.clone());
        city.details = world::map_details::new(data);
        let start = XYZ {
            x: 0.,
            y: height,
            z: 2.,
        };
        let end = XYZ {
            x: 30.,
            y: height,
            z: 2.,
        };
        let edges = exposed_bridge_edge(&city, &deck, start, end);
        assert_eq!(edges.len(), 1);
        assert!((edges[0].start.x - if height == 7. { 20. } else { 0. }).abs() < 0.001);
        assert_eq!(edges[0].end.x, 30.);
        let mut position = XYZ {
            x: 10.,
            y: height + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
            z: 2.5,
        };
        let initial = position;
        let mut velocity = XZ { x: 0., z: -4. };
        let impact = constrain_transport(&city, &mut position, &mut velocity);
        if height == 7. {
            assert_eq!(impact, 0.);
            assert_eq!(position, initial);
        } else {
            assert!(impact > 0.);
        }
    }
}
