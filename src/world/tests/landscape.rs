use super::*;
use crate::model::map_details::Deck;
use crate::model::map_details::RoadSurface;
use crate::util::geometry::BoundingBoxXZ;

#[test]
fn bridge_clearance_checks_crowns_heights_and_all_tree_sources() {
    let field = world::elevation::generate(
        BoundingBoxXZ {
            min: XZ { x: -50., z: -50. },
            max: XZ { x: 50., z: 50. },
        },
        |_| 0.,
    );
    let mut details = Data::default();
    details.decks.push(Deck {
        road: 0,
        grade: 1,
        width: 8.,
        surface: RoadSurface::Asphalt,
        path: vec![[-20., 5., 0.], [20., 5., 0.]],
    });
    details.decks.push(Deck {
        road: 1,
        grade: -1,
        width: 8.,
        surface: RoadSurface::Asphalt,
        path: vec![[-20., -5., 30.], [20., -5., 30.]],
    });
    let mut trees: Vec<_> = [
        Tree {
            id: 1,
            coords: XZ { x: 0., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 1 << 62,
            coords: XZ { x: 0., z: 6. },
            height_meters: 8.,
            crown_radius_meters: 2.5,
        },
        Tree {
            id: 1 << 63,
            coords: XZ { x: 21., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 2,
            coords: XZ { x: 0., z: 0. },
            height_meters: 3.,
            crown_radius_meters: 1.,
        },
        Tree {
            id: 3,
            coords: XZ { x: 0., z: 8. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 4,
            coords: XZ { x: 0., z: 30. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
    ]
    .into();
    clear_transport_trees(&mut trees, &field, &details, &[]);
    assert_eq!(
        trees.iter().map(|tree| tree.id).collect::<Vec<_>>(),
        vec![2, 3, 4]
    );
}

#[test]
fn approach_clearance_uses_offsets_above_local_terrain() {
    let field = world::elevation::generate(
        BoundingBoxXZ {
            min: XZ { x: -20., z: -20. },
            max: XZ { x: 20., z: 20. },
        },
        |_| 12.,
    );
    let mut trees = vec![
        Tree {
            id: 1,
            coords: XZ { x: 0., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 2,
            coords: XZ { x: 6., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 3,
            coords: XZ { x: 10., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
    ];
    let rises = [RoadRise {
        points: [
            util::geometry::at_height(XZ { x: -5., z: -5. }, 2.),
            util::geometry::at_height(XZ { x: 5., z: -5. }, 2.),
            util::geometry::at_height(XZ { x: 5., z: 5. }, 2.),
        ],
    }];
    clear_transport_trees(&mut trees, &field, &Data::default(), &rises);
    assert_eq!(
        trees.iter().map(|tree| tree.id).collect::<Vec<_>>(),
        vec![3]
    );
}
