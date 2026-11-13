//! Mapped ground cover and vegetation.
use crate::geometry::XZ;
use crate::world::Material;
pub use crate::world::{Tree, Triangle};

pub fn append_imagery_trees(trees: &mut Vec<Tree>, extra: Vec<Tree>) -> Result<(), String> {
    let mut ids: std::collections::HashSet<_> = trees.iter().map(|t| t.id).collect();
    if extra
        .iter()
        .any(|t| t.id < (1_u64 << 63) || !ids.insert(t.id))
    {
        return Err("invalid or duplicate imagery tree id".into());
    }
    trees.extend(extra);
    Ok(())
}

impl Material {
    /// Decode the material IDs used by landscape.txt.
    pub fn from_ground_id(id: usize) -> Result<Self, &'static str> {
        match id {
            0 => Ok(Self::Grass),
            1 => Ok(Self::Park),
            2 => Ok(Self::Scrub),
            3 => Ok(Self::Asphalt),
            4 => Ok(Self::Paving),
            5 => Ok(Self::Concrete),
            6 => Ok(Self::BareGround),
            7 => Ok(Self::Sports),
            8 => Ok(Self::Water),
            _ => Err("unknown ground material"),
        }
    }

    pub fn ground_id(self) -> Result<usize, &'static str> {
        match self {
            Self::Grass => Ok(0),
            Self::Park => Ok(1),
            Self::Scrub => Ok(2),
            Self::Asphalt => Ok(3),
            Self::Paving => Ok(4),
            Self::Concrete => Ok(5),
            Self::BareGround => Ok(6),
            Self::Sports => Ok(7),
            Self::Water => Ok(8),
            Self::Paint => Err("landscape format does not support paint"),
        }
    }
}

/// Prepared cover and its ground-query grid, constructed together.
#[derive(Debug)]
pub struct GroundCover {
    triangles: Vec<Triangle>,
    trees: Vec<Tree>,
    grid: std::collections::HashMap<(i32, i32), Vec<usize>>,
}
impl GroundCover {
    pub fn new(triangles: Vec<Triangle>, trees: Vec<Tree>) -> Self {
        let mut cover = Self {
            triangles,
            trees,
            grid: Default::default(),
        };
        cover.index_ground();
        cover
    }
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }
    pub fn trees(&self) -> &[Tree] {
        &self.trees
    }
}
impl GroundCover {
    fn index_ground(&mut self) {
        self.grid.clear();
        for (i, t) in self.triangles.iter().enumerate() {
            let points = t.points;
            let low: [i32; 2] = std::array::from_fn(|axis| {
                (points
                    .iter()
                    .map(|p| p.component(axis))
                    .fold(f32::INFINITY, f32::min)
                    / 32.0)
                    .floor() as i32
            });
            let high: [i32; 2] = std::array::from_fn(|axis| {
                (points
                    .iter()
                    .map(|p| p.component(axis))
                    .fold(f32::NEG_INFINITY, f32::max)
                    / 32.0)
                    .floor() as i32
            });
            for x in low[0]..=high[0] {
                for z in low[1]..=high[1] {
                    self.grid.entry((x, z)).or_default().push(i);
                }
            }
        }
    }
    pub fn material(&self, p: XZ<f32>) -> Option<Material> {
        self.grid
            .get(&((p.x / 32.0).floor() as i32, (p.z / 32.0).floor() as i32))?
            .iter()
            .map(|i| &self.triangles[*i])
            .find(|t| {
                let points = t.points;
                if (0..2).any(|a| {
                    p.component(a)
                        < points
                            .iter()
                            .map(|q| q.component(a))
                            .fold(f32::INFINITY, f32::min)
                            - 0.001
                        || p.component(a)
                            > points
                                .iter()
                                .map(|q| q.component(a))
                                .fold(f32::NEG_INFINITY, f32::max)
                                + 0.001
                }) {
                    return false;
                }
                let edges: [f32; 3] = std::array::from_fn(|i| {
                    let a = t.points[i];
                    let b = t.points[(i + 1) % 3];
                    (b.x - a.x) * (p.z - a.z) - (b.z - a.z) * (p.x - a.x)
                });
                edges.iter().all(|v| *v >= -0.00001) || edges.iter().all(|v| *v <= 0.00001)
            })
            .map(|t| t.material)
    }
}

// Terrain elevation.

#[cfg(test)]
mod index_tests {
    use super::*;
    use geo::{Intersects, Point};

    #[test]
    fn ground_owner_matches_full_triangle_queries_across_negative_and_positive_cells() {
        let triangles = vec![
            Triangle {
                material: Material::Park,
                usage: crate::world::TriangleUsage::Ground,
                points: [
                    XZ { x: -64., z: -64. },
                    XZ { x: 64., z: -64. },
                    XZ { x: -64., z: 64. },
                ],
            },
            Triangle {
                material: Material::Paving,
                usage: crate::world::TriangleUsage::Ground,
                points: [
                    XZ { x: 64., z: -64. },
                    XZ { x: 64., z: 64. },
                    XZ { x: -64., z: 64. },
                ],
            },
        ];
        let cover = GroundCover::new(triangles, vec![]);
        for x in (-80..=80).step_by(3) {
            for z in (-80..=80).step_by(5) {
                let p = XZ {
                    x: x as f32 + 0.13,
                    z: z as f32 + 0.37,
                };
                let expected = cover
                    .triangles()
                    .iter()
                    .find(|t| {
                        crate::geometry::polygon(&t.points.map(|p| p.map(f64::from)))
                            .intersects(&Point::new(p.x as f64, p.z as f64))
                    })
                    .map(|t| t.material);
                assert_eq!(cover.material(p), expected, "{p:?}");
            }
        }
        let empty = GroundCover::new(vec![], vec![]);
        assert_eq!(empty.material(XZ { x: 0., z: 0. }), None);
        assert_eq!(
            cover.material(XZ { x: 0.1, z: 0.1 }),
            Some(Material::Paving)
        );
    }
}
