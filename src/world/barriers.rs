//! Mapped barriers and their collision grid share one owner.
use super::{Barrier, Location};
use crate::geometry::{XYZ, XZ};
use crate::track;
use crate::vehicle_location;
use crate::world::spatial;
use std::collections::HashMap;

#[derive(Debug)]
pub struct Barriers {
    items: Vec<Barrier>,
    grid: HashMap<(i32, i32), Vec<usize>>,
}
impl Barriers {
    pub fn new(items: Vec<Barrier>) -> Self {
        let mut grid = HashMap::new();
        for (i, barrier) in items.iter().enumerate() {
            spatial::insert(
                &mut grid,
                spatial::bounds(
                    barrier.ends.iter().copied(),
                    track::CAR_RADIUS + barrier.width_meters / 2.0 + 0.2,
                ),
                i,
            );
        }
        Self { items, grid }
    }
    fn nearby(&self, p: XZ<f32>) -> impl Iterator<Item = &Barrier> {
        self.grid
            .get(&spatial::cell(p))
            .into_iter()
            .flatten()
            .map(|i| &self.items[*i])
    }
}

impl Location {
    pub fn barriers(&self) -> &[Barrier] {
        &self.barriers.items
    }

    pub fn barrier_clearance(&self, p: XZ<f32>) -> f32 {
        self.barriers
            .nearby(p)
            .map(|barrier| {
                let q = spatial::nearest_edge(p, barrier.ends[0], barrier.ends[1]);
                (p.x - q.x).hypot(p.z - q.z) - barrier.width_meters / 2.0
            })
            .fold(f32::INFINITY, f32::min)
    }

    pub fn constrain_barriers(&self, position: &mut XYZ, velocity: &mut XZ<f32>) -> f32 {
        let mut impact: f32 = 0.0;
        for barrier in self.barriers.nearby(position.xz()) {
            let p = position.xz();
            let q = spatial::nearest_edge(p, barrier.ends[0], barrier.ends[1]);
            let distance = (p.x - q.x).hypot(p.z - q.z);
            let clearance = track::CAR_RADIUS + barrier.width_meters / 2.0;
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
                    normal = normal.map(|v| -v);
                }
                normal
            };
            position.x = q.x + normal.x * (clearance + 0.01);
            position.z = q.z + normal.z * (clearance + 0.01);
            impact = impact.max(vehicle_location::deflect(velocity, normal.map(|v| -v)));
        }
        impact
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support;
    use crate::world::BarrierKind;

    #[test]
    fn barrier_contacts_deflect_from_both_sides_and_leave_openings_clear() {
        let mut city = test_support::empty_world();
        city.barriers = Barriers::new(vec![
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
            let impact = city.constrain_barriers(&mut position, &mut velocity);
            assert!(impact > 9.9);
            assert!(position.z * side >= track::CAR_RADIUS + 0.1);
            assert!(velocity.z * side > 0.);
            assert!(velocity.x > 0.);
        }
        let mut position = XYZ::default();
        let mut velocity = XZ { x: 0., z: 10. };
        assert_eq!(city.constrain_barriers(&mut position, &mut velocity), 0.);
        assert_eq!(position, XYZ::default());
        assert!(city.barrier_clearance(position.xz()) > track::CAR_RADIUS);
        position.x = 10.;
        assert!(city.constrain_barriers(&mut position, &mut velocity) > 0.);
        assert!(position.iter().all(|v| v.is_finite()));
    }
}
