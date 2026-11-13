//! Footprints and their padded collision/camera grid cannot be edited independently.
use super::*;
use crate::track;
use crate::world;
#[derive(Debug)]
pub(super) struct Buildings {
    items: Vec<Building>,
    grid: HashMap<(i32, i32), Vec<usize>>,
}
impl Buildings {
    pub(super) fn new(items: Vec<Building>) -> Self {
        let mut grid = HashMap::new();
        for (i, b) in items.iter().enumerate() {
            world::spatial::insert(
                &mut grid,
                world::spatial::bounds(
                    b.ground_outline.iter().copied(),
                    track::CAR_RADIUS + 1.0,
                ),
                i,
            );
        }
        Self { items, grid }
    }
    pub(super) fn with_terrain(mut self, field: &HeightField) -> Self {
        for building in &mut self.items {
            building.base_y = (0..building.ground_outline.len())
                .flat_map(|i| {
                    field.segment(
                        building.ground_outline[i],
                        building.ground_outline[(i + 1) % building.ground_outline.len()],
                    )
                })
                .map(|p| p.y)
                .fold(f32::NEG_INFINITY, f32::max)
                + 0.02;
        }
        self
    }
}
impl Location {
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
        self.buildings
            .grid
            .get(&world::spatial::cell(p))
            .map_or(f32::INFINITY, |ids| {
                ids.iter()
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
        }
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
