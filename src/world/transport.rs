//! Vertical road support and edge contact without joining crossing road levels.

use crate::model::map_details::RoadSurface;
use crate::model::world::WorldLocation;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

pub fn tunnel_openings(world: &WorldLocation) -> geo::MultiPolygon<f64> {
    let pieces: Vec<_> = world::raised_roads::lowered_footprints(&world.raised_roads).collect();
    if pieces.is_empty() {
        util::geometry::empty()
    } else {
        geo::algorithm::unary_union(&pieces)
    }
}

pub fn paved_near(world: &WorldLocation, p: XYZ) -> bool {
    let reference = p.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
    world::map_details::nearby_decks(&world.details, util::geometry::xz(p))
        .find(|deck| {
            world::map_details::height_on(deck, util::geometry::xz(p))
                .is_some_and(|h| (h - reference).abs() < 1.5)
        })
        .map_or_else(
            || world::street_surface::paved(world, util::geometry::xz(p)),
            |deck| deck.surface != RoadSurface::Unpaved,
        )
}
pub fn deck_height(world: &WorldLocation, p: XZ<f32>, reference: f32) -> Option<f32> {
    world::map_details::deck_height(&world.details, p, reference)
}
pub fn drive_height_near(world: &WorldLocation, p: XZ<f32>, reference: f32) -> f32 {
    let ground = physics::vehicle_location::drive_height(world, p);
    deck_height(world, p, reference)
        .filter(|height| (height - reference).abs() < (ground - reference).abs())
        .unwrap_or(ground)
}
pub fn constrain_transport(
    world: &WorldLocation,
    position: &mut XYZ,
    velocity: &mut XZ<f32>,
) -> f32 {
    let mut impact: f32 = 0.;
    for deck in world::map_details::nearby_decks(&world.details, util::geometry::xz(*position)) {
        for pair in deck.path.windows(2) {
            let a = XZ {
                x: pair[0][0],
                z: pair[0][2],
            };
            let b = XZ {
                x: pair[1][0],
                z: pair[1][2],
            };
            let dx = b.x - a.x;
            let dz = b.z - a.z;
            let length = dx.hypot(dz);
            let along = ((position.x - a.x) * dx + (position.z - a.z) * dz) / length;
            if along < 0.05 || along > length - 0.05 {
                continue;
            }
            let height = pair[0][1] + (pair[1][1] - pair[0][1]) * along / length;
            if (position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT - height).abs() > 1.5
            {
                continue;
            }
            let normal = XZ {
                x: -dz / length,
                z: dx / length,
            };
            let lateral = (position.x - a.x) * normal.x + (position.z - a.z) * normal.z;
            let limit = deck.width / 2. - world::locations::CAR_RADIUS - 0.08;
            if lateral.abs() > limit && lateral.abs() < deck.width / 2. + 2. {
                let penetration = lateral.abs() - limit;
                let n = XZ {
                    x: normal.x * lateral.signum(),
                    z: normal.z * lateral.signum(),
                };
                position.x -= n.x * penetration;
                position.z -= n.z * penetration;
                impact = impact.max(physics::vehicle_location::deflect(velocity, n));
            }
        }
    }
    impact
}
pub fn tunnel_ceiling(world: &WorldLocation, p: XZ<f32>, reference: f32) -> Option<f32> {
    world::map_details::nearby_decks(&world.details, p)
        .filter(|deck| deck.grade < 0)
        .filter_map(|deck| {
            world::map_details::height_on(deck, p)
                .filter(|h| (h - reference).abs() < 3.)
                .map(|h| h + 4.5)
        })
        .min_by(f32::total_cmp)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model;
    use crate::physics;
    use crate::render;
    use crate::test;
    use crate::world;

    #[test]
    fn tunnels_pass_beneath_ground_buildings_without_invisible_walls() {
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), true);
        let mut city = world::preparation::world_location_load(&assets).unwrap();
        let mut data = model::map_details::Data::default();
        data.decks.push(model::map_details::Deck {
            road: 0,
            grade: -1,
            width: 8.,
            path: vec![[0., 2., 12.], [20., 2., 12.]],
            surface: RoadSurface::Asphalt,
        });
        city.details = world::map_details::new(data);
        let mut position = XYZ {
            x: 12.,
            y: 2.48,
            z: 12.,
        };
        let original = position;
        let mut velocity = XZ { x: 4., z: 0. };
        assert!(world::buildings::building_clearance_at(&(city), position) > 10.);
        assert_eq!(
            physics::vehicle_location::constrain(&(city), &mut position, &mut velocity, 1.7),
            0.
        );
        assert!((position.y - original.y).abs() < 0.001);
        assert_eq!(position.x, original.x);
        assert_eq!(position.z, original.z);
        let anchor = XYZ {
            x: 13.,
            y: 3.2,
            z: 12.,
        };
        let eye = XYZ {
            x: 12.,
            y: 3.8,
            z: 12.,
        };
        assert_eq!(render::camera::camera_eye(&(city), anchor, eye), eye);
    }
    #[test]
    fn recovery_keeps_the_drivers_level_at_an_overlapping_road() {
        use crate::model;
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), false);
        let mut stored = model::location::location_read(folder.path()).unwrap();
        stored.roads.push(model::location::Road {
            ends: stored.roads[0].ends,
            way_id: Some(2),
            width_meters: 8.,
            direction: 0,
        });
        stored.streets.paths.push(stored.streets.paths[0].clone());
        stored.details.decks.push(model::map_details::Deck {
            road: 0,
            grade: 1,
            width: 8.,
            path: vec![[0., 12., 0.], [10., 12., 1.], [20., 12., 0.]],
            surface: RoadSurface::Asphalt,
        });
        model::location::location_write(&stored, folder.path()).unwrap();
        let city = world::preparation::world_location_load(&assets).unwrap();
        for height in [7., 12.] {
            let p = XYZ {
                x: 10.,
                y: height + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 1.,
            };
            assert!((world::roads::project(&(city), p).point.y - height).abs() < 0.001);
            assert!(
                (physics::vehicle_location::recovery(&(city), p).point.y - height).abs() < 0.001
            );
        }
    }
    #[test]
    fn stacked_support_rails_and_camera_follow_the_selected_road_level() {
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), false);
        for grade in [-1, 1] {
            let mut city = world::preparation::world_location_load(&assets).unwrap();
            city.elevation = Some(world::elevation::generate(
                model::world::bounding_box(&city),
                |_| 0.,
            ));
            let mut data = model::map_details::Data::default();
            data.decks.push(model::map_details::Deck {
                road: 0,
                grade,
                width: 8.,
                path: vec![[0., grade as f32 * 5., 0.], [20., grade as f32 * 5., 0.]],
                surface: RoadSurface::Asphalt,
            });
            city.details = world::map_details::new(data);
            let p = XZ { x: 10., z: 0. };
            assert!(world::transport::drive_height_near(&(city), p, 0.).abs() < 0.001);
            assert!(
                (world::transport::drive_height_near(&(city), p, grade as f32 * 5.)
                    - grade as f32 * 5.)
                    .abs()
                    < 0.001
            );
            assert!(
                (world::roads::sample(&(city), 0, 0.5).point.y - grade as f32 * 5.).abs() < 0.001
            );
            let mut position = XYZ {
                x: 10.,
                y: grade as f32 * 5. + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 3.7,
            };
            let mut velocity = XZ { x: 3., z: 5. };
            assert!(
                world::transport::constrain_transport(&(city), &mut position, &mut velocity) > 4.9
            );
            assert!(position.z < 3.);
            let mut ground = XYZ {
                x: 10.,
                y: physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 3.7,
            };
            let original = ground;
            assert_eq!(
                world::transport::constrain_transport(&(city), &mut ground, &mut velocity),
                0.
            );
            assert_eq!(ground, original);
            if grade < 0 {
                let anchor = XYZ {
                    x: 10.,
                    y: -3.8,
                    z: 0.,
                };
                let eye = XYZ {
                    x: 5.,
                    y: -3.0,
                    z: 0.,
                };
                assert_eq!(render::camera::camera_eye(&(city), anchor, eye), eye);
                assert!(render::camera::camera_eye(&(city), anchor, XYZ { y: 0., ..eye }).y < -0.5);
            }
        }
    }
}
