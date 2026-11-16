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
