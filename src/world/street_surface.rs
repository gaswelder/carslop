//! Offline union geometry shared by road rendering and collision.

use crate::model;
pub use crate::model::world::Curb;
use crate::model::world::Material;
pub use crate::model::world::Triangle;
use crate::model::world::WorldLocation;
use crate::storage;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::collections::HashMap;

/// Runtime centrelines in local metres, indexed like Location::road_lines.
pub type RoadPaths = Vec<Vec<XYZ>>;

pub const DEFAULT_PAVEMENT_HEIGHT: f32 = 0.14;

const CELL: f32 = 32.0;

fn cell(p: XZ<f32>) -> (i32, i32) {
    ((p.x / CELL).floor() as i32, (p.z / CELL).floor() as i32)
}

pub fn index(grid: &mut HashMap<(i32, i32), Vec<usize>>, points: &[XZ<f32>], id: usize, pad: f32) {
    let lo = cell(util::geometry::xz_from_fn(|i| {
        points
            .iter()
            .map(|p| util::geometry::xz_component(*p, i))
            .fold(f32::INFINITY, f32::min)
            - pad
    }));
    let hi = cell(util::geometry::xz_from_fn(|i| {
        points
            .iter()
            .map(|p| util::geometry::xz_component(*p, i))
            .fold(f32::NEG_INFINITY, f32::max)
            + pad
    }));
    for x in lo.0..=hi.0 {
        for z in lo.1..=hi.1 {
            grid.entry((x, z)).or_default().push(id);
        }
    }
}

pub fn cross(a: XZ<f32>, b: XZ<f32>, p: XZ<f32>) -> f32 {
    (b.x - a.x) * (p.z - a.z) - (b.z - a.z) * (p.x - a.x)
}

/// Temporary compiled surface data transferred into a Location after loading.
#[derive(Debug)]
pub struct StreetSurface {
    pavement_height_meters: f32,
    /// Road, marking and pavement triangles.
    pub road_triangles: Vec<Triangle>,
    /// Road-edge segments used for clearance and crossing queries.
    pub curbs: Vec<Curb>,
    /// 32 m [X, Z] grid cells mapped to candidate triangle indices.
    pub triangles_grid: HashMap<(i32, i32), Vec<usize>>,
    /// 32 m [X, Z] grid cells mapped to candidate curb indices.
    pub curbs_grid: HashMap<(i32, i32), Vec<usize>>,
}

pub fn from_geometry(
    data: model::location::Streets,
    height: f32,
) -> Result<(StreetSurface, RoadPaths), String> {
    let mut result = StreetSurface {
        pavement_height_meters: DEFAULT_PAVEMENT_HEIGHT,
        road_triangles: data.triangles,
        curbs: data.curbs,
        triangles_grid: HashMap::new(),
        curbs_grid: HashMap::new(),
    };
    set_pavement_height(&mut result, height)?;
    for (i, t) in result.road_triangles.iter().enumerate() {
        index(&mut result.triangles_grid, &t.points, i, 0.001);
    }
    for (i, c) in result.curbs.iter().enumerate() {
        index(&mut result.curbs_grid, &c.ends, i, 32.0);
    }
    let paths = data
        .paths
        .into_iter()
        .map(|p| {
            p.into_iter()
                .map(|p| util::geometry::at_height(p, 0.0))
                .collect()
        })
        .collect();
    Ok((result, paths))
}
pub fn road_triangles(street_surface: &StreetSurface) -> &[Triangle] {
    &street_surface.road_triangles
}
pub fn curbs(street_surface: &StreetSurface) -> &[Curb] {
    &street_surface.curbs
}
pub fn pavement_height_meters(street_surface: &StreetSurface) -> f32 {
    street_surface.pavement_height_meters
}
fn set_pavement_height(street_surface: &mut StreetSurface, height: f32) -> Result<(), String> {
    if !(0.08..=0.25).contains(&height) {
        return Err("invalid pavement height".into());
    }
    street_surface.pavement_height_meters = height;
    Ok(())
}

pub fn paved(world: &WorldLocation, p: XZ<f32>) -> bool {
    if world_location_triangle_at(world, p, true)
        .is_some_and(|triangle| triangle.material == Material::BareGround)
    {
        return false;
    }
    if let Some(h) = paving_height(world, p) {
        return h != 0.08;
    }
    matches!(
        model::world::ground_material(world, p),
        Some(Material::Asphalt | Material::Paving | Material::Concrete)
    )
}

/// Whether this location uses compiled surface geometry rather than centreline widths.
pub fn has_compiled_roads(world: &WorldLocation) -> bool {
    world::roads::compiled_surfaces(world).is_some()
}

pub fn world_location_triangle_at(
    world: &WorldLocation,
    p: XZ<f32>,
    road_only: bool,
) -> Option<&Triangle> {
    street_surface_triangle_at(world::roads::compiled_surfaces(world)?, p, road_only)
}
pub fn contains_road(world: &WorldLocation, p: XZ<f32>) -> bool {
    world_location_triangle_at(world, p, true).is_some()
}
pub fn world_location_nearest_curb(world: &WorldLocation, p: XZ<f32>) -> (XZ<f32>, XZ<f32>, f32) {
    street_surface_nearest_curb(
        world::roads::compiled_surfaces(world).expect("curb query requires compiled roads"),
        p,
    )
}
pub fn surface_clearance(world: &WorldLocation, p: XZ<f32>) -> f32 {
    world_location_nearest_curb(world, p).2 * if contains_road(world, p) { -1.0 } else { 1.0 }
}
pub fn paving_height(world: &WorldLocation, p: XZ<f32>) -> Option<f32> {
    world_location_triangle_at(world, p, false)
        .map(|t| model::world::support_height(t.usage, world::roads::pavement_height_meters(world)))
}

fn street_surface_triangle_at(
    street_surface: &StreetSurface,
    p: XZ<f32>,
    road_only: bool,
) -> Option<&Triangle> {
    street_surface.triangles_grid.get(&cell(p)).and_then(|ids| {
        ids.iter()
            .map(|&i| &street_surface.road_triangles[i])
            .find(|triangle| {
                if road_only && !model::world::is_road(triangle.usage) {
                    return false;
                }
                let t = triangle.points;
                // A distance tolerance can extend a very thin triangle far
                // beyond its acute tip. Bound it before testing its edges.
                if (0..2).any(|axis| {
                    util::geometry::xz_component(p, axis)
                        < t.iter()
                            .map(|v| util::geometry::xz_component(*v, axis))
                            .fold(f32::INFINITY, f32::min)
                            - 0.002
                        || util::geometry::xz_component(p, axis)
                            > t.iter()
                                .map(|v| util::geometry::xz_component(*v, axis))
                                .fold(f32::NEG_INFINITY, f32::max)
                                + 0.002
                }) {
                    return false;
                }
                (0..3).all(|i| {
                    let a = t[i];
                    let b = t[(i + 1) % 3];
                    cross(a, b, p) >= -0.002 * (b.x - a.x).hypot(b.z - a.z)
                })
            })
    })
}
fn nearest_curb_in(
    street_surface: &StreetSurface,
    p: XZ<f32>,
    ids: impl Iterator<Item = usize>,
) -> (XZ<f32>, XZ<f32>, f32) {
    ids.map(|i| {
        let c = &curbs(street_surface)[i];
        let [a, b] = c.ends;
        let delta = XZ {
            x: b.x - a.x,
            z: b.z - a.z,
        };
        let t = (((p.x - a.x) * delta.x + (p.z - a.z) * delta.z)
            / (delta.x * delta.x + delta.z * delta.z).max(0.000001))
        .clamp(0.0, 1.0);
        let q = XZ {
            x: a.x + delta.x * t,
            z: a.z + delta.z * t,
        };
        (q, c.inward, (q.x - p.x).hypot(q.z - p.z))
    })
    .min_by(|a, b| a.2.total_cmp(&b.2))
    .unwrap_or((p, XZ { x: 0., z: 0. }, f32::INFINITY))
}
fn street_surface_nearest_curb(
    street_surface: &StreetSurface,
    p: XZ<f32>,
) -> (XZ<f32>, XZ<f32>, f32) {
    if let Some(ids) = street_surface.curbs_grid.get(&cell(p)) {
        let best = nearest_curb_in(street_surface, p, ids.iter().copied());
        if best.2 <= 32.0 {
            return best;
        }
    }
    nearest_curb_in(street_surface, p, 0..curbs(street_surface).len())
}

pub fn load(text: &str, roads: usize) -> Result<(StreetSurface, RoadPaths), String> {
    from_geometry(
        storage::codec_streets::parse(text, roads)?,
        DEFAULT_PAVEMENT_HEIGHT,
    )
}

#[cfg(test)]
mod index_tests {
    use super::*;
    use crate::model;
    use crate::util;
    use crate::world;
    use geo::{Intersects, Point};

    #[test]
    fn mapped_road_materials_round_trip_through_the_binary_surface_cache() {
        let folder = tempfile::tempdir().unwrap();
        for id in 8..=16 {
            let text = format!(
                "STREETS 1 1\np 0 2 0 0 20 0\nt {id} 0 -4 20 -4 20 4\nc 1 0 -4 20 -4 0 1\n"
            );
            let (surface, paths) = world::street_surface::load(&text, 1).unwrap();
            storage::street_surface_binary::write_binary_street_surface(
                &folder.path().join("streets.bin"),
                &surface,
                &paths,
            )
            .unwrap();
            let stored = storage::street_surface_binary::load_geometry(folder.path(), 1).unwrap();
            assert_eq!(
                stored.triangles,
                world::street_surface::road_triangles(&(surface))
            );
            assert_eq!(
                storage::codec_streets::triangle_street_id(&(stored.triangles[0])).unwrap(),
                id
            );
        }
    }
    #[test]
    fn surface_owner_matches_full_triangle_and_curb_scans_at_grid_boundaries() {
        let text = concat!(
            "STREETS 1 1\np 0 2 -64 0 64 0\n",
            "t 1 -64 -4 64 -4 64 4\nt 1 -64 -4 64 4 -64 4\n",
            "t 6 -64 4 64 4 64 8\nt 6 -64 4 64 8 -64 8\n",
            "c 1 -64 4 64 4 0 -1\nc 1 -64 -4 64 -4 0 1\n"
        );
        let (surface, _) = world::street_surface::load(text, 1).unwrap();
        for x in (-100..=100).step_by(7) {
            for z in (-60..=60).step_by(3) {
                let p = XZ {
                    x: x as f32 + 0.13,
                    z: z as f32 + 0.37,
                };
                for road_only in [false, true] {
                    let expected = world::street_surface::road_triangles(&surface)
                        .iter()
                        .find(|t| {
                            (!road_only || model::world::is_road(t.usage))
                                && util::geometry::polygon(
                                    &t.points.map(|p| util::geometry::xz_map(p, f64::from)),
                                )
                                .intersects(&Point::new(p.x as f64, p.z as f64))
                        })
                        .map(|t| t.usage);
                    assert_eq!(
                        world::street_surface::street_surface_triangle_at(&(surface), p, road_only)
                            .map(|t| t.usage),
                        expected,
                        "{p:?}"
                    );
                }
                let expected = world::street_surface::curbs(&surface)
                    .iter()
                    .map(|c| {
                        let q = world::spatial::nearest_edge(p, c.ends[0], c.ends[1]);
                        (q, c.inward, (q.x - p.x).hypot(q.z - p.z))
                    })
                    .min_by(|a, b| a.2.total_cmp(&b.2))
                    .unwrap();
                assert_eq!(
                    world::street_surface::street_surface_nearest_curb(&(surface), p),
                    expected,
                    "{p:?}"
                );
            }
        }
    }
}
