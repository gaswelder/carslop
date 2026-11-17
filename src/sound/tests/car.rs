use super::*;
use crate::model;
use crate::model::map_details;
use crate::model::map_details::RoadSurface;
use crate::test;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use std::sync::Arc;

#[test]
fn tires_use_ground_material_and_the_supporting_bridge_deck() {
    let mut location = test::support::empty_world();
    location.ground = world::landscape::new(
        vec![model::world::Triangle {
            material: Material::Grass,
            usage: model::world::TriangleUsage::Ground,
            points: [
                XZ { x: -20., z: -20. },
                XZ { x: 20., z: -20. },
                XZ { x: 0., z: 20. },
            ],
        }],
        vec![],
    );
    let mut details = map_details::Data::default();
    let mut deck = test::support::bridge_deck(0, 5., 0., -20., 20.);
    deck.surface = RoadSurface::Unpaved;
    details.decks.push(deck);
    location.details = world::map_details::new(details);
    let mut car = physics::car::on_track(
        XYZ {
            x: 0.,
            y: 0.,
            z: 0.,
        },
        0.,
        test::support::car(),
        Arc::new(location),
    );
    car.position.y = physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
    assert_eq!(tire_material(&car), Material::Grass);
    car.position.y += 5.;
    assert_eq!(tire_material(&car), Material::BareGround);
}
