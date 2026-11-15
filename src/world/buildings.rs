//! Footprints and their padded collision/camera grid cannot be edited independently.

use crate::model;
use crate::model::world::Building;
use crate::model::world::HeightField;
use crate::model::world::WorldLocation;
use crate::physics;
use crate::util;
#[cfg(test)]
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use geo::Area;
use std::collections::HashMap;
#[derive(Debug)]
pub struct Buildings {
    items: Vec<Building>,
    grid: HashMap<(i32, i32), Vec<usize>>,
    groups: HashMap<u64, Vec<usize>>,
}

pub fn new(items: Vec<Building>) -> Buildings {
    let mut grid = HashMap::new();
    let mut groups: HashMap<u64, Vec<usize>> = HashMap::new();
    for (i, b) in items.iter().enumerate() {
        groups.entry(b.id).or_default().push(i);
        world::spatial::insert(
            &mut grid,
            world::spatial::bounds(
                b.ground_outline.iter().copied(),
                world::locations::CAR_RADIUS + 1.0,
            ),
            i,
        );
    }
    Buildings {
        items,
        grid,
        groups,
    }
}
pub fn with_terrain(mut buildings: Buildings, field: &HeightField) -> Buildings {
    let mut bases: HashMap<u64, f32> = HashMap::new();
    for building in &buildings.items {
        let height = model::world::rings(building)
            .flat_map(|ring| {
                (0..ring.len()).flat_map(|i| {
                    world::elevation::segment(field, ring[i], ring[(i + 1) % ring.len()])
                })
            })
            .map(|p| p.y)
            .fold(f32::NEG_INFINITY, f32::max)
            + 0.02;
        bases
            .entry(building.id)
            .and_modify(|base| *base = base.max(height))
            .or_insert(height);
    }
    for building in &mut buildings.items {
        building.base_y = bases[&building.id];
    }
    buildings
}

pub fn building_style<'a>(
    world: &'a WorldLocation,
    building: &Building,
) -> Option<&'a model::map_details::BuildingStyle> {
    world
        .buildings
        .groups
        .get(&building.id)?
        .iter()
        .find(|i| std::ptr::eq(&world.buildings.items[**i], building))
        .and_then(|i| world::map_details::building(&world.details, *i))
}
pub fn primary_building_volume(world: &WorldLocation, id: u64) -> Option<&Building> {
    world
        .buildings
        .groups
        .get(&id)
        .into_iter()
        .flatten()
        .map(|i| &world.buildings.items[*i])
        .filter(|building| building.min_height_meters == 0.0)
        .max_by(|a, b| {
            model::world::footprint(a)
                .unsigned_area()
                .total_cmp(&model::world::footprint(b).unsigned_area())
        })
}
pub fn entrance_building_volume(
    world: &WorldLocation,
    id: u64,
    point: XZ<f32>,
) -> Option<&Building> {
    world
        .buildings
        .groups
        .get(&id)
        .into_iter()
        .flatten()
        .map(|i| &world.buildings.items[*i])
        .filter(|building| building.min_height_meters == 0.0)
        .min_by(|a, b| {
            nearest(a, point)
                .2
                .total_cmp(&nearest(b, point).2)
                .then_with(|| {
                    model::world::footprint(b)
                        .unsigned_area()
                        .total_cmp(&model::world::footprint(a).unsigned_area())
                })
        })
}
pub fn building_entrances(world: &WorldLocation, building_id: u64) -> &[XZ<f32>] {
    world.entrances.get(&building_id).map_or(&[], Vec::as_slice)
}
pub fn buildings(world: &WorldLocation) -> &[Building] {
    &world.buildings.items
}
pub fn nearby_buildings(world: &WorldLocation, p: XZ<f32>) -> impl Iterator<Item = &Building> {
    world
        .buildings
        .grid
        .get(&world::spatial::cell(p))
        .into_iter()
        .flatten()
        .map(|&i| &world.buildings.items[i])
}
/// Signed distance among nearby footprints; infinity when none is nearby.
pub fn building_clearance(world: &WorldLocation, p: XZ<f32>) -> f32 {
    building_clearance_at(
        world,
        util::geometry::at_height(p, world::elevation::world_height(world, p) + 0.48),
    )
}
pub fn building_clearance_at(world: &WorldLocation, position: XYZ) -> f32 {
    let p = util::geometry::xz(position);
    world
        .buildings
        .grid
        .get(&world::spatial::cell(p))
        .map_or(f32::INFINITY, |ids| {
            ids.iter()
                .filter(|i| {
                    position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT + 1.7
                        >= world::elevation::world_height(world, p) - 0.2
                        && physics::vehicle_location::overlaps_vehicle(
                            &world.buildings.items[**i],
                            position,
                            1.7,
                        )
                })
                .map(|i| {
                    let b = &world.buildings.items[*i];
                    nearest(b, p).2 * if contains(b, p) { -1.0 } else { 1.0 }
                })
                .fold(f32::INFINITY, f32::min)
        })
}

pub fn contains(building: &Building, p: XZ<f32>) -> bool {
    fn inside(ring: &[XZ<f32>], p: XZ<f32>) -> bool {
        let mut inside = false;
        for i in 0..ring.len() {
            let a = ring[i];
            let b = ring[(i + 1) % ring.len()];
            if (a.z > p.z) != (b.z > p.z) && p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x {
                inside = !inside;
            }
        }
        inside
    }
    inside(&building.ground_outline, p) && !building.holes.iter().any(|hole| inside(hole, p))
}

pub fn nearest(building: &Building, p: XZ<f32>) -> (XZ<f32>, XZ<f32>, f32) {
    let mut best = (XZ::<f32>::default(), XZ::<f32>::default(), f32::INFINITY);
    for ring in model::world::rings(building) {
        for i in 0..ring.len() {
            let a = ring[i];
            let b = ring[(i + 1) % ring.len()];
            let q = world::spatial::nearest_edge(p, a, b);
            let distance = (p.x - q.x).hypot(p.z - q.z);
            if distance < best.2 {
                let len = (b.x - a.x).hypot(b.z - a.z).max(0.0001);
                best = (
                    (q),
                    XZ {
                        x: (b.z - a.z) / len,
                        z: -(b.x - a.x) / len,
                    },
                    distance,
                );
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics;
    use crate::render;
    use crate::test;
    use crate::util;
    use crate::world;

    fn building(id: u64, x: f32, z: f32) -> Building {
        Building {
            id,
            ground_outline: vec![
                XZ { x, z },
                XZ { x: x + 8., z },
                XZ {
                    x: x + 8.,
                    z: z + 8.,
                },
                XZ { x, z: z + 8. },
            ],
            height_meters: 6.,
            base_y: 0.,
            tagged: true,
            holes: Vec::new(),
            min_height_meters: 0.0,
        }
    }

    #[test]
    fn raised_building_parts_leave_vehicle_and_camera_clearance() {
        let city = test::support::parse_world(concat!(
            "CITY 2\nbounds -50 -50 50 50\nn 10 -30\nn 10 30\nr 0 1 8 0 1\n",
            "b 100 9 0 4 0 0 5 0 5 10 0 10\n",
            "b 100 9 0 4 5 0 15 0 15 10 5 10\nv 1 3.1 0\n",
            "b 100 9 0 4 15 0 20 0 20 10 15 10\n",
        ))
        .unwrap();
        let start = util::geometry::at_height(XZ { x: 10., z: 5. }, 0.48);
        let mut position = start;
        physics::vehicle_location::constrain(&city, &mut position, &mut XZ { x: 0., z: 4. }, 1.4);
        assert_eq!(position, start);
        assert!(building_clearance(&(city), util::geometry::xz(position)) > 1.4);
        let eye = util::geometry::at_height(XZ { x: 10., z: 15. }, 1.5);
        assert_eq!(
            render::camera::camera_eye(
                &(city),
                util::geometry::at_height(XZ { x: 10., z: -5. }, 1.5),
                eye
            ),
            eye
        );
        physics::vehicle_location::constrain(&city, &mut position, &mut XZ { x: 0., z: 4. }, 3.5);
        assert!((position.x - start.x).hypot(position.z - start.z) > 1.0);
    }

    #[test]
    fn footprint_owner_preserves_clearance_candidates_and_terrain_fitting() {
        let mut world = test::support::empty_world();
        world.buildings = new(vec![
            building(1, -36., -36.),
            building(2, 28., 28.),
            building(3, 40., 28.),
        ]);
        let field = world::elevation::generate(
            BoundingBoxXZ {
                min: XZ { x: -64., z: -64. },
                max: XZ { x: 64., z: 64. },
            },
            |p| p.x * 0.1 + p.z * 0.2,
        );
        let probes: Vec<_> = buildings(&world)
            .iter()
            .flat_map(|b| {
                let p = b.ground_outline[0];
                [
                    XZ {
                        x: p.x - 0.5,
                        z: p.z + 4.,
                    },
                    XZ {
                        x: p.x + 4.,
                        z: p.z + 4.,
                    },
                    XZ {
                        x: p.x + 8.5,
                        z: p.z + 4.,
                    },
                ]
            })
            .collect();
        for p in &probes {
            let expected = buildings(&world)
                .iter()
                .map(|b| nearest(b, *p).2 * if contains(b, *p) { -1. } else { 1. })
                .fold(f32::INFINITY, f32::min);
            assert_eq!(building_clearance(&(world), *p), expected, "{p:?}");
        }
        let before: Vec<_> = probes
            .iter()
            .map(|&p| {
                nearby_buildings(&world, p)
                    .map(|b| b.id)
                    .collect::<Vec<_>>()
            })
            .collect();
        world.buildings = with_terrain(world.buildings, &field);
        let after: Vec<_> = probes
            .iter()
            .map(|&p| {
                nearby_buildings(&world, p)
                    .map(|b| b.id)
                    .collect::<Vec<_>>()
            })
            .collect();
        assert_eq!(before, after);
        for b in buildings(&world) {
            let max_height = b
                .ground_outline
                .iter()
                .map(|&p| world::elevation::height(&field, p))
                .fold(f32::NEG_INFINITY, f32::max);
            assert!((b.base_y - max_height - 0.02).abs() < 0.0001);
        }
        // Replacing the owner discards every candidate from the previous geometry.
        world.buildings = new(vec![building(4, 128., 128.)]);
        assert!(nearby_buildings(&(world), probes[0]).next().is_none());
        assert!(building_clearance(&(world), probes[0]).is_infinite());
    }
}
