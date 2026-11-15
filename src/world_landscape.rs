//! Mapped ground cover and vegetation.

use crate::model_world::Material;
pub use crate::model_world::{Tree, Triangle};
use crate::util_geometry;
use crate::util_geometry::XZ;

pub fn append_imagery_trees(trees: &mut Vec<Tree>, extra: Vec<Tree>) -> Result<(), String> {
    let mut ids: std::collections::HashSet<_> = trees.iter().map(|t| t.id).collect();
    if extra
        .iter()
        .any(|t| t.id < (1_u64 << 63) || !ids.insert(t.id))
    {
        return Err("invalid or duplicate imagery tree id".into());
    }
    // Imagery crowns take precedence over procedural fills from wooded areas.
    let crowns = rstar::RTree::bulk_load(
        extra
            .iter()
            .map(|tree| util_geometry::xz_map(tree.coords, f64::from))
            .collect(),
    );
    trees.retain(|tree| {
        tree.id & (1_u64 << 63) != 0
            || tree.id & (1_u64 << 62) == 0
            || crowns
                .nearest_neighbor(&util_geometry::xz_map(tree.coords, f64::from))
                .is_none_or(|crown| {
                    (crown.x - tree.coords.x as f64).hypot(crown.z - tree.coords.z as f64) >= 3.0
                })
    });
    trees.extend(extra);
    Ok(())
}

/// Prepared cover and its ground-query grid, constructed together.
#[derive(Debug)]
pub struct GroundCover {
    triangles: Vec<Triangle>,
    trees: Vec<Tree>,
    grid: std::collections::HashMap<(i32, i32), Vec<usize>>,
}

pub fn new(triangles: Vec<Triangle>, trees: Vec<Tree>) -> GroundCover {
    let mut cover = GroundCover {
        triangles,
        trees,
        grid: Default::default(),
    };
    index_ground(&mut cover);
    cover
}
pub fn triangles(ground_cover: &GroundCover) -> &[Triangle] {
    &ground_cover.triangles
}
pub fn trees(ground_cover: &GroundCover) -> &[Tree] {
    &ground_cover.trees
}

fn index_ground(ground_cover: &mut GroundCover) {
    ground_cover.grid.clear();
    for (i, t) in ground_cover.triangles.iter().enumerate() {
        let points = t.points;
        let low: [i32; 2] = std::array::from_fn(|axis| {
            (points
                .iter()
                .map(|p| util_geometry::xz_component(*p, axis))
                .fold(f32::INFINITY, f32::min)
                / 32.0)
                .floor() as i32
        });
        let high: [i32; 2] = std::array::from_fn(|axis| {
            (points
                .iter()
                .map(|p| util_geometry::xz_component(*p, axis))
                .fold(f32::NEG_INFINITY, f32::max)
                / 32.0)
                .floor() as i32
        });
        for x in low[0]..=high[0] {
            for z in low[1]..=high[1] {
                ground_cover.grid.entry((x, z)).or_default().push(i);
            }
        }
    }
}
pub fn material(ground_cover: &GroundCover, p: XZ<f32>) -> Option<Material> {
    ground_cover
        .grid
        .get(&((p.x / 32.0).floor() as i32, (p.z / 32.0).floor() as i32))?
        .iter()
        .map(|i| &ground_cover.triangles[*i])
        .find(|t| {
            let points = t.points;
            if (0..2).any(|a| {
                util_geometry::xz_component(p, a)
                    < points
                        .iter()
                        .map(|q| util_geometry::xz_component(*q, a))
                        .fold(f32::INFINITY, f32::min)
                        - 0.001
                    || util_geometry::xz_component(p, a)
                        > points
                            .iter()
                            .map(|q| util_geometry::xz_component(*q, a))
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

// Terrain elevation.

#[cfg(test)]
mod index_tests {
    use crate::world_landscape;

    use super::*;
    use crate::model_world;
    use crate::util_geometry;
    use geo::{Intersects, Point};

    #[test]
    fn imagery_crowns_replace_procedural_fill_but_keep_mapped_tree_nodes() {
        let tree = |id, x| Tree {
            id,
            coords: XZ { x, z: 0. },
            height_meters: 6.,
            crown_radius_meters: 1.,
        };
        let mut trees = vec![
            tree(42, 0.),
            tree((1 << 62) | 1, 0.5),
            tree((1 << 62) | 2, 10.),
        ];
        append_imagery_trees(&mut trees, vec![tree((1 << 63) | 3, 1.)]).unwrap();
        assert_eq!(
            trees.iter().map(|t| t.id).collect::<Vec<_>>(),
            vec![42, (1 << 62) | 2, (1 << 63) | 3]
        );
        assert!(append_imagery_trees(&mut trees, vec![tree((1 << 63) | 3, 1.)]).is_err());
    }
    #[test]
    fn ground_owner_matches_full_triangle_queries_across_negative_and_positive_cells() {
        let triangles = vec![
            Triangle {
                material: Material::Park,
                usage: model_world::TriangleUsage::Ground,
                points: [
                    XZ { x: -64., z: -64. },
                    XZ { x: 64., z: -64. },
                    XZ { x: -64., z: 64. },
                ],
            },
            Triangle {
                material: Material::Paving,
                usage: model_world::TriangleUsage::Ground,
                points: [
                    XZ { x: 64., z: -64. },
                    XZ { x: 64., z: 64. },
                    XZ { x: -64., z: 64. },
                ],
            },
        ];
        let cover = world_landscape::new(triangles, vec![]);
        for x in (-80..=80).step_by(3) {
            for z in (-80..=80).step_by(5) {
                let p = XZ {
                    x: x as f32 + 0.13,
                    z: z as f32 + 0.37,
                };
                let expected = world_landscape::triangles(&cover)
                    .iter()
                    .find(|t| {
                        util_geometry::polygon(
                            &t.points.map(|p| util_geometry::xz_map(p, f64::from)),
                        )
                        .intersects(&Point::new(p.x as f64, p.z as f64))
                    })
                    .map(|t| t.material);
                assert_eq!(world_landscape::material(&(cover), p), expected, "{p:?}");
            }
        }
        let empty = world_landscape::new(vec![], vec![]);
        assert_eq!(
            world_landscape::material(&(empty), XZ { x: 0., z: 0. }),
            None
        );
        assert_eq!(
            world_landscape::material(&(cover), XZ { x: 0.1, z: 0.1 }),
            Some(Material::Paving)
        );
    }
}
