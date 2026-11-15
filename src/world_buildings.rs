//! Footprints and their padded collision/camera grid cannot be edited independently.

use crate::model_map_details;
use crate::model_world;
use crate::model_world::{Building, HeightField, WorldLocation};
use crate::physics_vehicle_location;
use crate::util_geometry;
#[cfg(test)]
use crate::util_geometry::BoundingBoxXZ;
use crate::util_geometry::{XYZ, XZ};
use crate::world_elevation;
use crate::world_locations;
use crate::world_spatial;
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
        world_spatial::insert(
            &mut grid,
            world_spatial::bounds(
                b.ground_outline.iter().copied(),
                world_locations::CAR_RADIUS + 1.0,
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
        let height = model_world::rings(building)
            .flat_map(|ring| {
                (0..ring.len()).flat_map(|i| {
                    world_elevation::segment(field, ring[i], ring[(i + 1) % ring.len()])
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
) -> Option<&'a model_map_details::BuildingStyle> {
    world
        .buildings
        .groups
        .get(&building.id)?
        .iter()
        .find(|i| std::ptr::eq(&world.buildings.items[**i], building))
        .and_then(|i| model_map_details::building(&world.details, *i))
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
            model_world::footprint(a)
                .unsigned_area()
                .total_cmp(&model_world::footprint(b).unsigned_area())
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
            model_world::nearest(a, point)
                .2
                .total_cmp(&model_world::nearest(b, point).2)
                .then_with(|| {
                    model_world::footprint(b)
                        .unsigned_area()
                        .total_cmp(&model_world::footprint(a).unsigned_area())
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
        .get(&world_spatial::cell(p))
        .into_iter()
        .flatten()
        .map(|&i| &world.buildings.items[i])
}
/// Signed distance among nearby footprints; infinity when none is nearby.
pub fn building_clearance(world: &WorldLocation, p: XZ<f32>) -> f32 {
    building_clearance_at(
        world,
        util_geometry::at_height(p, model_world::height(world, p) + 0.48),
    )
}
pub fn building_clearance_at(world: &WorldLocation, position: XYZ) -> f32 {
    let p = util_geometry::xz(position);
    world
        .buildings
        .grid
        .get(&world_spatial::cell(p))
        .map_or(f32::INFINITY, |ids| {
            ids.iter()
                .filter(|i| {
                    position.y - physics_vehicle_location::VEHICLE_ORIGIN_HEIGHT + 1.7
                        >= model_world::height(world, p) - 0.2
                        && model_world::overlaps_vehicle(&world.buildings.items[**i], position, 1.7)
                })
                .map(|i| {
                    let b = &world.buildings.items[*i];
                    model_world::nearest(b, p).2
                        * if model_world::contains(b, p) {
                            -1.0
                        } else {
                            1.0
                        }
                })
                .fold(f32::INFINITY, f32::min)
        })
}

#[cfg(test)]
mod tests {
    use crate::model_world;
    use crate::physics_vehicle_location;
    use crate::render_camera;
    use crate::util_geometry;
    use crate::world_buildings;
    use crate::world_elevation;

    use super::*;
    use crate::test_support;

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
        let city = test_support::parse_world(concat!(
            "CITY 2\nbounds -50 -50 50 50\nn 10 -30\nn 10 30\nr 0 1 8 0 1\n",
            "b 100 9 0 4 0 0 5 0 5 10 0 10\n",
            "b 100 9 0 4 5 0 15 0 15 10 5 10\nv 1 3.1 0\n",
            "b 100 9 0 4 15 0 20 0 20 10 15 10\n",
        ))
        .unwrap();
        let start = util_geometry::at_height(XZ { x: 10., z: 5. }, 0.48);
        let mut position = start;
        physics_vehicle_location::constrain(&city, &mut position, &mut XZ { x: 0., z: 4. }, 1.4);
        assert_eq!(position, start);
        assert!(world_buildings::building_clearance(&(city), util_geometry::xz(position)) > 1.4);
        let eye = util_geometry::at_height(XZ { x: 10., z: 15. }, 1.5);
        assert_eq!(
            render_camera::camera_eye(
                &(city),
                util_geometry::at_height(XZ { x: 10., z: -5. }, 1.5),
                eye
            ),
            eye
        );
        physics_vehicle_location::constrain(&city, &mut position, &mut XZ { x: 0., z: 4. }, 3.5);
        assert!((position.x - start.x).hypot(position.z - start.z) > 1.0);
    }

    #[test]
    fn footprint_owner_preserves_clearance_candidates_and_terrain_fitting() {
        let mut world = test_support::empty_world();
        world.buildings = world_buildings::new(vec![
            building(1, -36., -36.),
            building(2, 28., 28.),
            building(3, 40., 28.),
        ]);
        let field = world_elevation::generate(
            BoundingBoxXZ {
                min: XZ { x: -64., z: -64. },
                max: XZ { x: 64., z: 64. },
            },
            |p| p.x * 0.1 + p.z * 0.2,
        );
        let probes: Vec<_> = world_buildings::buildings(&world)
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
            let expected = world_buildings::buildings(&world)
                .iter()
                .map(|b| {
                    model_world::nearest(b, *p).2
                        * if model_world::contains(b, *p) {
                            -1.
                        } else {
                            1.
                        }
                })
                .fold(f32::INFINITY, f32::min);
            assert_eq!(
                world_buildings::building_clearance(&(world), *p),
                expected,
                "{p:?}"
            );
        }
        let before: Vec<_> = probes
            .iter()
            .map(|&p| {
                world_buildings::nearby_buildings(&world, p)
                    .map(|b| b.id)
                    .collect::<Vec<_>>()
            })
            .collect();
        world.buildings = world_buildings::with_terrain(world.buildings, &field);
        let after: Vec<_> = probes
            .iter()
            .map(|&p| {
                world_buildings::nearby_buildings(&world, p)
                    .map(|b| b.id)
                    .collect::<Vec<_>>()
            })
            .collect();
        assert_eq!(before, after);
        for b in world_buildings::buildings(&world) {
            let max_height = b
                .ground_outline
                .iter()
                .map(|&p| world_elevation::height(&field, p))
                .fold(f32::NEG_INFINITY, f32::max);
            assert!((b.base_y - max_height - 0.02).abs() < 0.0001);
        }
        // Replacing the owner discards every candidate from the previous geometry.
        world.buildings = world_buildings::new(vec![building(4, 128., 128.)]);
        assert!(world_buildings::nearby_buildings(&(world), probes[0])
            .next()
            .is_none());
        assert!(world_buildings::building_clearance(&(world), probes[0]).is_infinite());
    }
}
