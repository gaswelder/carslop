//! Vertical road support and edge contact without joining crossing road levels.
use crate::geometry;
use crate::geometry::{XYZ, XZ};
use crate::map_details::RoadSurface;
use crate::track;
use crate::vehicle_location;
use crate::world::WorldLocation;

impl WorldLocation {
    pub fn tunnel_openings(&self) -> geo::MultiPolygon<f64> {
        let pieces: Vec<_> = self.raised_roads.lowered_footprints().collect();
        if pieces.is_empty() {
            geometry::empty()
        } else {
            geo::algorithm::unary_union(&pieces)
        }
    }

    pub fn paved_near(&self, p: XYZ) -> bool {
        let reference = p.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT;
        self.details
            .nearby_decks(p.xz())
            .find(|deck| {
                self.details
                    .height_on(deck, p.xz())
                    .is_some_and(|h| (h - reference).abs() < 1.5)
            })
            .map_or_else(
                || self.paved(p.xz()),
                |deck| deck.surface != RoadSurface::Unpaved,
            )
    }
    pub fn deck_height(&self, p: XZ<f32>, reference: f32) -> Option<f32> {
        self.details.deck_height(p, reference)
    }
    pub fn drive_height_near(&self, p: XZ<f32>, reference: f32) -> f32 {
        let ground = self.drive_height(p);
        self.deck_height(p, reference)
            .filter(|height| (height - reference).abs() < (ground - reference).abs())
            .unwrap_or(ground)
    }
    pub fn constrain_transport(&self, position: &mut XYZ, velocity: &mut XZ<f32>) -> f32 {
        let mut impact: f32 = 0.;
        for deck in self.details.nearby_decks(position.xz()) {
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
                if (position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT - height).abs() > 1.5 {
                    continue;
                }
                let normal = XZ {
                    x: -dz / length,
                    z: dx / length,
                };
                let lateral = (position.x - a.x) * normal.x + (position.z - a.z) * normal.z;
                let limit = deck.width / 2. - track::CAR_RADIUS - 0.08;
                if lateral.abs() > limit && lateral.abs() < deck.width / 2. + 2. {
                    let penetration = lateral.abs() - limit;
                    let n = XZ {
                        x: normal.x * lateral.signum(),
                        z: normal.z * lateral.signum(),
                    };
                    position.x -= n.x * penetration;
                    position.z -= n.z * penetration;
                    impact = impact.max(vehicle_location::deflect(velocity, n));
                }
            }
        }
        impact
    }
    pub fn tunnel_ceiling(&self, p: XZ<f32>, reference: f32) -> Option<f32> {
        self.details
            .nearby_decks(p)
            .filter(|deck| deck.grade < 0)
            .filter_map(|deck| {
                self.details
                    .height_on(deck, p)
                    .filter(|h| (h - reference).abs() < 3.)
                    .map(|h| h + 4.5)
            })
            .min_by(f32::total_cmp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map_details;
    use crate::test_support;
    use crate::world::HeightField;
    #[test]
    fn tunnels_pass_beneath_ground_buildings_without_invisible_walls() {
        let folder = tempfile::tempdir().unwrap();
        let assets = test_support::stored_location(folder.path(), true);
        let mut city = WorldLocation::load(&assets).unwrap();
        let mut data = map_details::Data::default();
        data.decks.push(map_details::Deck {
            road: 0,
            grade: -1,
            width: 8.,
            path: vec![[0., 2., 12.], [20., 2., 12.]],
            surface: RoadSurface::Asphalt,
        });
        city.details = map_details::Prepared::new(data);
        let mut position = XYZ {
            x: 12.,
            y: 2.48,
            z: 12.,
        };
        let original = position;
        let mut velocity = XZ { x: 4., z: 0. };
        assert!(city.building_clearance_at(position) > 10.);
        assert_eq!(city.constrain(&mut position, &mut velocity, 1.7), 0.);
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
        assert_eq!(city.camera_eye(anchor, eye), eye);
    }
    #[test]
    fn recovery_keeps_the_drivers_level_at_an_overlapping_road() {
        use crate::location;
        let folder = tempfile::tempdir().unwrap();
        let assets = test_support::stored_location(folder.path(), false);
        let mut stored = location::Location::read(folder.path()).unwrap();
        stored.roads.push(location::Road {
            ends: stored.roads[0].ends,
            way_id: Some(2),
            width_meters: 8.,
            direction: 0,
        });
        stored.streets.paths.push(stored.streets.paths[0].clone());
        stored.details.decks.push(map_details::Deck {
            road: 0,
            grade: 1,
            width: 8.,
            path: vec![[0., 12., 0.], [10., 12., 1.], [20., 12., 0.]],
            surface: RoadSurface::Asphalt,
        });
        stored.write(folder.path()).unwrap();
        let city = WorldLocation::load(&assets).unwrap();
        for height in [7., 12.] {
            let p = XYZ {
                x: 10.,
                y: height + vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 1.,
            };
            assert!((city.project(p).point.y - height).abs() < 0.001);
            assert!((city.recovery(p).point.y - height).abs() < 0.001);
        }
    }
    #[test]
    fn stacked_support_rails_and_camera_follow_the_selected_road_level() {
        let folder = tempfile::tempdir().unwrap();
        let assets = test_support::stored_location(folder.path(), false);
        for grade in [-1, 1] {
            let mut city = WorldLocation::load(&assets).unwrap();
            city.elevation = Some(HeightField::generate(city.bounding_box(), |_| 0.));
            let mut data = map_details::Data::default();
            data.decks.push(map_details::Deck {
                road: 0,
                grade,
                width: 8.,
                path: vec![[0., grade as f32 * 5., 0.], [20., grade as f32 * 5., 0.]],
                surface: RoadSurface::Asphalt,
            });
            city.details = map_details::Prepared::new(data);
            let p = XZ { x: 10., z: 0. };
            assert!(city.drive_height_near(p, 0.).abs() < 0.001);
            assert!(
                (city.drive_height_near(p, grade as f32 * 5.) - grade as f32 * 5.).abs() < 0.001
            );
            assert!((city.sample(0, 0.5).point.y - grade as f32 * 5.).abs() < 0.001);
            let mut position = XYZ {
                x: 10.,
                y: grade as f32 * 5. + vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 3.7,
            };
            let mut velocity = XZ { x: 3., z: 5. };
            assert!(city.constrain_transport(&mut position, &mut velocity) > 4.9);
            assert!(position.z < 3.);
            let mut ground = XYZ {
                x: 10.,
                y: vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 3.7,
            };
            let original = ground;
            assert_eq!(city.constrain_transport(&mut ground, &mut velocity), 0.);
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
                assert_eq!(city.camera_eye(anchor, eye), eye);
                assert!(city.camera_eye(anchor, XYZ { y: 0., ..eye }).y < -0.5);
            }
        }
    }
}
