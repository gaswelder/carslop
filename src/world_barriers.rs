//! Mapped barriers and their collision grid share one owner.

use crate::model_world;
use crate::model_world::{Barrier, WorldLocation};
use crate::physics_vehicle_location;
use crate::util_geometry;
use crate::util_geometry::{XYZ, XZ};
use crate::world_locations;
use crate::world_spatial;
use std::collections::HashMap;

#[derive(Debug)]
pub struct Barriers {
    items: Vec<Barrier>,
    grid: HashMap<(i32, i32), Vec<usize>>,
}

pub fn new(items: Vec<Barrier>) -> Barriers {
    let mut grid = HashMap::new();
    for (i, barrier) in items.iter().enumerate() {
        world_spatial::insert(
            &mut grid,
            world_spatial::bounds(
                barrier.ends.iter().copied(),
                world_locations::CAR_RADIUS + barrier.width_meters / 2.0 + 0.2,
            ),
            i,
        );
    }
    Barriers { items, grid }
}
fn nearby(barriers: &Barriers, p: XZ<f32>) -> impl Iterator<Item = &Barrier> {
    barriers
        .grid
        .get(&world_spatial::cell(p))
        .into_iter()
        .flatten()
        .map(|i| &barriers.items[*i])
}

pub fn barriers(world: &WorldLocation) -> &[Barrier] {
    &world.barriers.items
}

pub fn barrier_clearance(world: &WorldLocation, p: XZ<f32>) -> f32 {
    nearby(&world.barriers, p)
        .map(|barrier| {
            let q = world_spatial::nearest_edge(p, barrier.ends[0], barrier.ends[1]);
            (p.x - q.x).hypot(p.z - q.z) - barrier.width_meters / 2.0
        })
        .fold(f32::INFINITY, f32::min)
}

pub fn constrain_barriers(
    world: &WorldLocation,
    position: &mut XYZ,
    velocity: &mut XZ<f32>,
) -> f32 {
    let mut impact: f32 = 0.0;
    for barrier in nearby(&world.barriers, util_geometry::xz(*position)) {
        let base = model_world::height(world, util_geometry::xz(*position));
        let bottom = position.y - physics_vehicle_location::VEHICLE_ORIGIN_HEIGHT;
        if bottom > base + barrier.height_meters || position.y + 1. < base {
            continue;
        }
        let p = util_geometry::xz(*position);
        let q = world_spatial::nearest_edge(p, barrier.ends[0], barrier.ends[1]);
        let distance = (p.x - q.x).hypot(p.z - q.z);
        let clearance = world_locations::CAR_RADIUS + barrier.width_meters / 2.0;
        if distance >= clearance {
            continue;
        }
        let normal = if distance > 0.0001 {
            XZ {
                x: (p.x - q.x) / distance,
                z: (p.z - q.z) / distance,
            }
        } else {
            let a = barrier.ends[0];
            let b = barrier.ends[1];
            let length = (b.x - a.x).hypot(b.z - a.z);
            let mut normal = XZ {
                x: (b.z - a.z) / length,
                z: -(b.x - a.x) / length,
            };
            if normal.x * velocity.x + normal.z * velocity.z > 0.0 {
                normal = util_geometry::xz_map(normal, |v| -v);
            }
            normal
        };
        position.x = q.x + normal.x * (clearance + 0.01);
        position.z = q.z + normal.z * (clearance + 0.01);
        impact = impact.max(physics_vehicle_location::deflect(
            velocity,
            util_geometry::xz_map(normal, |v| -v),
        ));
    }
    impact
}

#[cfg(test)]
mod tests {
    use crate::util_geometry;
    use crate::world_barriers;

    use super::*;
    use crate::model_world::BarrierKind;
    use crate::test_support;

    #[test]
    fn barrier_contacts_deflect_from_both_sides_and_leave_openings_clear() {
        let mut city = test_support::empty_world();
        city.barriers = world_barriers::new(vec![
            Barrier {
                source_id: 1,
                kind: BarrierKind::Wall,
                ends: [XZ { x: -20., z: 0. }, XZ { x: -2., z: 0. }],
                height_meters: 2.,
                width_meters: 0.2,
            },
            Barrier {
                source_id: 1,
                kind: BarrierKind::Wall,
                ends: [XZ { x: 2., z: 0. }, XZ { x: 20., z: 0. }],
                height_meters: 2.,
                width_meters: 0.2,
            },
        ]);
        for side in [-1., 1.] {
            let mut position = XYZ {
                x: 10.,
                y: 0.,
                z: side * 0.5,
            };
            let mut velocity = XZ {
                x: 2.,
                z: -side * 10.,
            };
            let impact = world_barriers::constrain_barriers(&city, &mut position, &mut velocity);
            assert!(impact > 9.9);
            assert!(position.z * side >= world_locations::CAR_RADIUS + 0.1);
            assert!(velocity.z * side > 0.);
            assert!(velocity.x > 0.);
        }
        let mut position = XYZ::default();
        let mut velocity = XZ { x: 0., z: 10. };
        assert_eq!(
            world_barriers::constrain_barriers(&(city), &mut position, &mut velocity),
            0.
        );
        assert_eq!(position, XYZ::default());
        assert!(
            world_barriers::barrier_clearance(&(city), util_geometry::xz(position))
                > world_locations::CAR_RADIUS
        );
        position.x = 10.;
        assert!(world_barriers::constrain_barriers(&(city), &mut position, &mut velocity) > 0.);
        assert!(util_geometry::iter(&(position)).all(|v| v.is_finite()));
    }
}
