//! Mapped barriers and their collision grid share one owner.

use crate::model::world::Barrier;
use crate::model::world::WorldLocation;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::collections::HashMap;

#[derive(Debug)]
pub struct Barriers {
    items: Vec<Barrier>,
    grid: HashMap<(i32, i32), Vec<usize>>,
}

pub fn new(items: Vec<Barrier>) -> Barriers {
    let mut grid = HashMap::new();
    for (i, barrier) in items.iter().enumerate() {
        world::spatial::insert(
            &mut grid,
            world::spatial::bounds(
                barrier.ends.iter().copied(),
                world::locations::CAR_RADIUS + barrier.width_meters / 2.0 + 0.2,
            ),
            i,
        );
    }
    Barriers { items, grid }
}
fn nearby(barriers: &Barriers, p: XZ<f32>) -> impl Iterator<Item = &Barrier> {
    barriers
        .grid
        .get(&world::spatial::cell(p))
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
            let q = world::spatial::nearest_edge(p, barrier.ends[0], barrier.ends[1]);
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
    for barrier in nearby(&world.barriers, util::geometry::xz(*position)) {
        let base = world::elevation::world_height(world, util::geometry::xz(*position));
        let bottom = position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
        if bottom > base + barrier.height_meters || position.y + 1. < base {
            continue;
        }
        let p = util::geometry::xz(*position);
        let q = world::spatial::nearest_edge(p, barrier.ends[0], barrier.ends[1]);
        let distance = (p.x - q.x).hypot(p.z - q.z);
        let clearance = world::locations::CAR_RADIUS + barrier.width_meters / 2.0;
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
                normal = util::geometry::xz_map(normal, |v| -v);
            }
            normal
        };
        position.x = q.x + normal.x * (clearance + 0.01);
        position.z = q.z + normal.z * (clearance + 0.01);
        impact = impact.max(physics::vehicle_location::deflect(
            velocity,
            util::geometry::xz_map(normal, |v| -v),
        ));
    }
    impact
}
