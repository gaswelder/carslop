//! Footprints and their padded collision/camera grid cannot be edited independently.
use super::*;
use crate::map_details;
use crate::track;
use crate::world;
use geo::Area;
#[derive(Debug)]
pub(super) struct Buildings {
    items: Vec<Building>,
    grid: HashMap<(i32, i32), Vec<usize>>,
    groups: HashMap<u64, Vec<usize>>,
}
impl Buildings {
    pub(super) fn new(items: Vec<Building>) -> Self {
        let mut grid = HashMap::new();
        let mut groups: HashMap<u64, Vec<usize>> = HashMap::new();
        for (i, b) in items.iter().enumerate() {
            groups.entry(b.id).or_default().push(i);
            world::spatial::insert(
                &mut grid,
                world::spatial::bounds(b.ground_outline.iter().copied(), track::CAR_RADIUS + 1.0),
                i,
            );
        }
        Self {
            items,
            grid,
            groups,
        }
    }
    pub(super) fn with_terrain(mut self, field: &HeightField) -> Self {
        let mut bases: HashMap<u64, f32> = HashMap::new();
        for building in &self.items {
            let height = building
                .rings()
                .flat_map(|ring| {
                    (0..ring.len()).flat_map(|i| field.segment(ring[i], ring[(i + 1) % ring.len()]))
                })
                .map(|p| p.y)
                .fold(f32::NEG_INFINITY, f32::max)
                + 0.02;
            bases
                .entry(building.id)
                .and_modify(|base| *base = base.max(height))
                .or_insert(height);
        }
        for building in &mut self.items {
            building.base_y = bases[&building.id];
        }
        self
    }
}
impl WorldLocation {
    pub fn building_style(&self, building: &Building) -> Option<&map_details::BuildingStyle> {
        self.buildings
            .groups
            .get(&building.id)?
            .iter()
            .find(|i| std::ptr::eq(&self.buildings.items[**i], building))
            .and_then(|i| self.details.building(*i))
    }
    pub fn primary_building_volume(&self, id: u64) -> Option<&Building> {
        self.buildings
            .groups
            .get(&id)
            .into_iter()
            .flatten()
            .map(|i| &self.buildings.items[*i])
            .filter(|building| building.min_height_meters == 0.0)
            .max_by(|a, b| {
                a.footprint()
                    .unsigned_area()
                    .total_cmp(&b.footprint().unsigned_area())
            })
    }
    pub fn entrance_building_volume(&self, id: u64, point: XZ<f32>) -> Option<&Building> {
        self.buildings
            .groups
            .get(&id)
            .into_iter()
            .flatten()
            .map(|i| &self.buildings.items[*i])
            .filter(|building| building.min_height_meters == 0.0)
            .min_by(|a, b| {
                a.nearest(point)
                    .2
                    .total_cmp(&b.nearest(point).2)
                    .then_with(|| {
                        b.footprint()
                            .unsigned_area()
                            .total_cmp(&a.footprint().unsigned_area())
                    })
            })
    }
    pub fn building_entrances(&self, building_id: u64) -> &[XZ<f32>] {
        self.entrances.get(&building_id).map_or(&[], Vec::as_slice)
    }
    pub fn buildings(&self) -> &[Building] {
        &self.buildings.items
    }
    pub fn nearby_buildings(&self, p: XZ<f32>) -> impl Iterator<Item = &Building> {
        self.buildings
            .grid
            .get(&world::spatial::cell(p))
            .into_iter()
            .flatten()
            .map(|&i| &self.buildings.items[i])
    }
    /// Signed distance among nearby footprints; infinity when none is nearby.
    pub fn building_clearance(&self, p: XZ<f32>) -> f32 {
        self.building_clearance_at(p.at_height(self.height(p) + 0.48))
    }
    pub fn building_clearance_at(&self, position: XYZ) -> f32 {
        let p = position.xz();
        self.buildings
            .grid
            .get(&world::spatial::cell(p))
            .map_or(f32::INFINITY, |ids| {
                ids.iter()
                    .filter(|i| {
                        position.y - vehicle_location::VEHICLE_ORIGIN_HEIGHT + 1.7
                            >= self.height(p) - 0.2
                            && self.buildings.items[**i].overlaps_vehicle(position, 1.7)
                    })
                    .map(|i| {
                        let b = &self.buildings.items[*i];
                        b.nearest(p).2 * if b.contains(p) { -1.0 } else { 1.0 }
                    })
                    .fold(f32::INFINITY, f32::min)
            })
    }
}

#[cfg(test)]
mod tests {
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
        let start = XZ { x: 10., z: 5. }.at_height(0.48);
        let mut position = start;
        city.constrain(&mut position, &mut XZ { x: 0., z: 4. }, 1.4);
        assert_eq!(position, start);
        assert!(city.building_clearance(position.xz()) > 1.4);
        let eye = XZ { x: 10., z: 15. }.at_height(1.5);
        assert_eq!(
            city.camera_eye(XZ { x: 10., z: -5. }.at_height(1.5), eye),
            eye
        );
        city.constrain(&mut position, &mut XZ { x: 0., z: 4. }, 3.5);
        assert!((position.x - start.x).hypot(position.z - start.z) > 1.0);
    }

    #[test]
    fn footprint_owner_preserves_clearance_candidates_and_terrain_fitting() {
        let mut world = test_support::empty_world();
        world.buildings = Buildings::new(vec![
            building(1, -36., -36.),
            building(2, 28., 28.),
            building(3, 40., 28.),
        ]);
        let field = HeightField::generate(
            BoundingBoxXZ {
                min: XZ { x: -64., z: -64. },
                max: XZ { x: 64., z: 64. },
            },
            |p| p.x * 0.1 + p.z * 0.2,
        );
        let probes: Vec<_> = world
            .buildings()
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
            let expected = world
                .buildings()
                .iter()
                .map(|b| b.nearest(*p).2 * if b.contains(*p) { -1. } else { 1. })
                .fold(f32::INFINITY, f32::min);
            assert_eq!(world.building_clearance(*p), expected, "{p:?}");
        }
        let before: Vec<_> = probes
            .iter()
            .map(|&p| world.nearby_buildings(p).map(|b| b.id).collect::<Vec<_>>())
            .collect();
        world.buildings = world.buildings.with_terrain(&field);
        let after: Vec<_> = probes
            .iter()
            .map(|&p| world.nearby_buildings(p).map(|b| b.id).collect::<Vec<_>>())
            .collect();
        assert_eq!(before, after);
        for b in world.buildings() {
            let max_height = b
                .ground_outline
                .iter()
                .map(|&p| field.height(p))
                .fold(f32::NEG_INFINITY, f32::max);
            assert!((b.base_y - max_height - 0.02).abs() < 0.0001);
        }
        // Replacing the owner discards every candidate from the previous geometry.
        world.buildings = Buildings::new(vec![building(4, 128., 128.)]);
        assert!(world.nearby_buildings(probes[0]).next().is_none());
        assert!(world.building_clearance(probes[0]).is_infinite());
    }
}
