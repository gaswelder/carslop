//! Road geometry and the grid derived from its prepared paths are one owner.

use crate::model_location;
use crate::model_map_details;
#[cfg(test)]
use crate::model_source_location::{SourceLocation, SourceRoad};
use crate::model_world;
use crate::model_world::{Curb, HeightField, Road, RoadSample, Triangle, WorldLocation};
use crate::util_geometry;
use crate::util_geometry::{BoundingBoxXZ, XYZ, XZ};
#[cfg(test)]
use crate::world_buildings;
use crate::world_elevation;
use crate::world_raised_roads;
use crate::world_spatial;
use crate::world_street_surface;
use crate::world_street_surface::{RoadPaths, StreetSurface};
use std::collections::HashMap;

#[derive(Debug)]
pub struct RoadNetwork {
    nodes: Vec<XYZ>,
    lines: Vec<Road>,
    grid: HashMap<(i32, i32), Vec<usize>>,
    geometry: RoadGeometry,
}
#[derive(Debug)]
enum RoadGeometry {
    Centreline,
    Compiled(StreetSurface),
}

pub fn empty() -> RoadNetwork {
    RoadNetwork {
        nodes: vec![],
        lines: vec![],
        grid: HashMap::new(),
        geometry: RoadGeometry::Centreline,
    }
}
pub fn from_stored(nodes: Vec<XZ<f32>>, roads: Vec<model_location::Road>) -> RoadNetwork {
    from_roads(
        nodes
            .into_iter()
            .map(|p| util_geometry::at_height(p, 0.0))
            .collect(),
        roads
            .into_iter()
            .map(|r| Road {
                osm_way_id: r.way_id,
                direction: r.direction,
                ..model_world::new(r.ends, r.width_meters)
            })
            .collect(),
    )
}
#[cfg(test)]
fn road_network_from_source(nodes: Vec<XYZ>, source_roads: Vec<SourceRoad<f32>>) -> RoadNetwork {
    from_roads(
        nodes,
        source_roads
            .into_iter()
            .map(|r| Road {
                osm_way_id: r.osm_way_id,
                direction: r.direction,
                road_width_end_meters: r.road_width_start_meters,
                ..model_world::new(r.ends, r.road_width_start_meters)
            })
            .collect(),
    )
}
fn from_roads(nodes: Vec<XYZ>, source_roads: Vec<Road>) -> RoadNetwork {
    let mut lines = Vec::new();
    let mut grid = HashMap::new();
    let mut distance = 0.0;
    for (i, road) in source_roads.into_iter().enumerate() {
        let a = nodes[road.ends[0]];
        let b = nodes[road.ends[1]];
        let length = (b.x - a.x).hypot(b.z - a.z);
        lines.push(Road {
            ends: road.ends,
            osm_way_id: road.osm_way_id,
            road_width_start_meters: road.road_width_start_meters,
            road_width_end_meters: road.road_width_end_meters,
            direction: road.direction,
            length,
            distance,
            path: vec![a, b],
            path_distances: vec![0.0, length],
        });
        distance += length;
        world_spatial::insert(
            &mut grid,
            world_spatial::bounds(
                lines[i]
                    .ends
                    .iter()
                    .map(|&node| util_geometry::xz(nodes[node])),
                32.0,
            ),
            i,
        );
    }
    RoadNetwork {
        nodes,
        lines,
        grid,
        geometry: RoadGeometry::Centreline,
    }
}
pub fn authored(nodes: Vec<XYZ>, widths: Vec<f32>) -> Result<RoadNetwork, String> {
    let mut lines = Vec::new();
    let mut grid = HashMap::new();
    let mut distance = 0.0;
    for i in 0..nodes.len() {
        let j = (i + 1) % nodes.len();
        let a = nodes[i];
        let b = nodes[j];
        let length = (b.x - a.x).hypot(b.z - a.z);
        if !length.is_finite() || length <= 0.0001 {
            return Err("degenerate route spline segment".into());
        }
        lines.push(Road {
            road_width_end_meters: widths[j],
            length,
            distance,
            path: vec![a, b],
            path_distances: vec![0.0, length],
            ..model_world::new([i, j], widths[i])
        });
        distance += length;
        world_spatial::insert(
            &mut grid,
            world_spatial::bounds([a, b].into_iter().map(|p| util_geometry::xz(p)), 32.0),
            i,
        );
    }
    Ok(RoadNetwork {
        nodes,
        lines,
        grid,
        geometry: RoadGeometry::Centreline,
    })
}
pub fn with_compiled(
    mut road_network: RoadNetwork,
    surfaces: StreetSurface,
    paths: RoadPaths,
    field: &HeightField,
) -> Result<RoadNetwork, String> {
    if paths.len() != road_network.lines.len() || paths.iter().any(|p| p.len() < 2) {
        return Err("streets: incomplete compiled road paths".into());
    }
    let mut grid = std::mem::take(&mut road_network.grid);
    grid.clear();
    let mut distance = 0.0;
    for (i, (street, path)) in road_network.lines.iter_mut().zip(paths).enumerate() {
        street.path = path;
        street.path_distances = vec![0.0];
        street.length = 0.0;
        for pair in street.path.windows(2) {
            let length = (pair[1].x - pair[0].x).hypot(pair[1].z - pair[0].z);
            if !length.is_finite() || length <= 0.0001 {
                return Err(format!("streets: degenerate compiled road path {i}"));
            }
            street.length += length;
            street.path_distances.push(street.length);
        }
        street.distance = distance;
        distance += street.length;
        world_spatial::insert(
            &mut grid,
            world_spatial::bounds(street.path.iter().map(|p| util_geometry::xz(*p)), 32.0),
            i,
        );
    }
    for p in &mut road_network.nodes {
        p.y = world_elevation::height(field, util_geometry::xz(*p));
    }
    for street in &mut road_network.lines {
        for p in &mut street.path {
            p.y = world_elevation::height(field, util_geometry::xz(*p));
        }
    }
    road_network.grid = grid;
    road_network.geometry = RoadGeometry::Compiled(surfaces);
    Ok(road_network)
}
#[cfg(test)]
fn bump(road_network: &mut RoadNetwork) {
    for street in &mut road_network.lines {
        for (point, station) in street
            .path
            .iter_mut()
            .zip([street.distance, street.distance + street.length])
        {
            point.y = 0.14 * (-((station - 55.0) / 1.0).powi(2)).exp();
        }
    }
    for (point, street) in road_network.nodes.iter_mut().zip(&road_network.lines) {
        *point = street.path[0];
    }
}

pub fn road_lines(world: &WorldLocation) -> &[Road] {
    &world.roads.lines
}
pub fn sample(world: &WorldLocation, index: usize, t: f32) -> RoadSample {
    sample_distance(
        world,
        index,
        t.clamp(0.0, 1.0) * world.roads.lines[index].length,
    )
}

pub fn sample_distance(world: &WorldLocation, index: usize, distance: f32) -> RoadSample {
    let deck = model_map_details::road_deck(&world.details, index);
    let r = &world.roads.lines[index];
    let distance = distance.clamp(0.0, r.length);
    let index = r
        .path_distances
        .partition_point(|d| *d < distance)
        .saturating_sub(1)
        .min(r.path.len() - 2);
    let a = r.path[index];
    let b = r.path[index + 1];
    let length = r.path_distances[index + 1] - r.path_distances[index];
    let t = ((distance - r.path_distances[index]) / length).clamp(0.0, 1.0);
    let mut point = util_geometry::xyz_from_fn(|i| {
        util_geometry::xyz_component(a, i)
            + (util_geometry::xyz_component(b, i) - util_geometry::xyz_component(a, i)) * t
    });
    // Compiled road surfaces are draped over the heightfield between path knots.
    if let Some(deck) = deck {
        if let Some(height) = model_map_details::height_on(deck, util_geometry::xz(point)) {
            point.y = height;
        }
    } else if world_street_surface::has_compiled_roads(world) {
        if let Some(field) = &world.elevation {
            point.y = world_elevation::height(field, util_geometry::xz(point))
                + world_raised_roads::road_rise(world, util_geometry::xz(point));
        }
    }
    RoadSample {
        point: (point),
        tangent: XZ {
            x: (b.x - a.x) / length,
            z: (b.z - a.z) / length,
        },
        distance: r.distance + distance,
        offset: 0.0,
        width: r.road_width_start_meters
            + (r.road_width_end_meters - r.road_width_start_meters) * (distance / r.length),
    }
}
fn street_ids_in(world: &WorldLocation, bounds: BoundingBoxXZ<f32>) -> Vec<usize> {
    let a = world_spatial::cell(XZ {
        x: bounds.min.x,
        z: bounds.min.z,
    });
    let b = world_spatial::cell(XZ {
        x: bounds.max.x,
        z: bounds.max.z,
    });
    let mut ids = Vec::new();
    for x in a.0..=b.0 {
        for z in a.1..=b.1 {
            if let Some(values) = world.roads.grid.get(&(x, z)) {
                ids.extend(values);
            }
        }
    }
    ids.sort_unstable();
    ids.dedup();
    ids
}
pub fn road_segments_in(world: &WorldLocation, bounds: BoundingBoxXZ<f32>) -> Vec<(XYZ, XYZ)> {
    street_ids_in(world, bounds)
        .into_iter()
        .flat_map(|i| world.roads.lines[i].path.windows(2).map(|p| (p[0], p[1])))
        .collect()
}
pub fn candidates(world: &WorldLocation, p: XYZ) -> Box<dyn Iterator<Item = usize> + '_> {
    if let Some(ids) = world
        .roads
        .grid
        .get(&world_spatial::cell(XZ { x: p.x, z: p.z }))
    {
        return Box::new(ids.iter().copied());
    }
    for radius in [
        1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0,
    ] {
        let d = radius * world_spatial::CELL;
        let ids = street_ids_in(
            world,
            BoundingBoxXZ {
                min: XZ {
                    x: p.x - d,
                    z: p.z - d,
                },
                max: XZ {
                    x: p.x + d,
                    z: p.z + d,
                },
            },
        );
        if !ids.is_empty() {
            return Box::new(ids.into_iter());
        }
    }
    Box::new(0..world.roads.lines.len())
}
pub fn projected(world: &WorldLocation, index: usize, p: XYZ) -> (f32, RoadSample) {
    let r = &world.roads.lines[index];
    let point = XZ { x: p.x, z: p.z };
    r.path
        .windows(2)
        .enumerate()
        .map(|(i, pair)| {
            let a = XZ {
                x: pair[0].x,
                z: pair[0].z,
            };
            let b = XZ {
                x: pair[1].x,
                z: pair[1].z,
            };
            let q = world_spatial::nearest_edge(point, a, b);
            let along = (q.x - a.x).hypot(q.z - a.z);
            let mut s = sample_distance(world, index, r.path_distances[i] + along);
            let delta = XZ {
                x: p.x - s.point.x,
                z: p.z - s.point.z,
            };
            s.offset = delta.x * model_world::normal(s).x + delta.z * model_world::normal(s).z;
            (delta.x.hypot(delta.z), s)
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .unwrap()
}
pub fn project(world: &WorldLocation, p: XYZ) -> RoadSample {
    nearest_road(world, p).1
}
/// Nearest road index and its local geometric frame.
pub fn nearest_road(world: &WorldLocation, p: XYZ) -> (usize, RoadSample) {
    let origin = world_spatial::cell(XZ { x: p.x, z: p.z });
    let mut seen = std::collections::HashSet::new();
    let mut best: Option<(f32, usize, RoadSample)> = None;
    let limit = ([world.bounding_box.min, world.bounding_box.max]
        .iter()
        .flat_map(|b| [(b.x - p.x).abs(), (b.z - p.z).abs()])
        .fold(0.0_f32, f32::max)
        / world_spatial::CELL)
        .ceil() as i32
        + 2;
    for radius in 0..=limit {
        for dx in -radius..=radius {
            for dz in -radius..=radius {
                if dx.abs() != radius && dz.abs() != radius {
                    continue;
                }
                if let Some(ids) = world.roads.grid.get(&(origin.0 + dx, origin.1 + dz)) {
                    for &i in ids {
                        if seen.insert(i) {
                            let mut candidate = projected(world, i, p);
                            if !model_world::map_details(world).decks.is_empty() {
                                candidate.0 = candidate.0.hypot(candidate.1.point.y - p.y);
                            }
                            if best.is_none_or(|b| {
                                candidate.0.total_cmp(&b.0).then(i.cmp(&b.1)).is_lt()
                            }) {
                                best = Some((candidate.0, i, candidate.1));
                            }
                        }
                    }
                }
            }
        }
        // Indexed path bounds carry a 32 m margin, so unvisited cells
        // cannot hide a closer road once this distance is reached.
        if best.is_some_and(|b| b.0 <= (radius + 1) as f32 * world_spatial::CELL) {
            let (_, index, road) = best.unwrap();
            return (index, road);
        }
    }
    let (_, index, road) = best.expect("city contains roads");
    (index, road)
}
/// Optional mapped name of a road, without presentation fallbacks.
pub fn road_name(world: &WorldLocation, index: usize) -> Option<&str> {
    world.roads.lines[index]
        .osm_way_id
        .and_then(|id| world.street_names.get(&id))
        .map(String::as_str)
}

pub fn compiled_surfaces(world: &WorldLocation) -> Option<&StreetSurface> {
    match &world.roads.geometry {
        RoadGeometry::Compiled(surfaces) => Some(surfaces),
        RoadGeometry::Centreline => None,
    }
}
pub fn road_triangles(world: &WorldLocation) -> &[Triangle] {
    compiled_surfaces(world).map_or(&[], |s| world_street_surface::road_triangles(s))
}
pub fn curbs(world: &WorldLocation) -> &[Curb] {
    compiled_surfaces(world).map_or(&[], |s| world_street_surface::curbs(s))
}
pub fn pavement_height_meters(world: &WorldLocation) -> f32 {
    compiled_surfaces(world).map_or(world_street_surface::DEFAULT_PAVEMENT_HEIGHT, |s| {
        world_street_surface::pavement_height_meters(s)
    })
}

#[cfg(test)]
mod tests {
    use crate::util_geometry;
    use crate::world_elevation;
    use crate::world_roads;
    use crate::world_street_surface;

    use super::*;
    use crate::test_support;

    #[test]
    fn prepared_road_grids_match_full_projection_for_centrelines_and_compiled_paths() {
        for compiled in [false, true] {
            let mut world = test_support::parse_world(concat!(
                "CITY 2\nbounds -100 -100 100 100\n",
                "n -90 0\nn 90 0\nn -90 70\nn 90 70\n",
                "r 0 1 8 0 1\nr 2 3 6.5 0 2\n",
            ))
            .unwrap();
            if compiled {
                let text = concat!(
                    "STREETS 1 2\n",
                    "p 0 3 -90 0 0 20 90 0\np 1 3 -90 70 0 50 90 70\n",
                    "t 1 -90 -4 90 -4 90 4\nc 1 -90 -4 90 -4 0 1\n"
                );
                let (surfaces, paths) = world_street_surface::load(text, 2).unwrap();
                let field =
                    world_elevation::generate(world.bounding_box, |p| p.x * 0.01 + p.z * 0.02);
                world.roads =
                    world_roads::with_compiled(world.roads, surfaces, paths, &field).unwrap();
                world.elevation = Some(field);
                assert_eq!(world_roads::road_lines(&(world))[0].path.len(), 3);
            }
            for x in (-200..=200).step_by(13) {
                for z in (-200..=200).step_by(17) {
                    let point = util_geometry::at_height(
                        XZ {
                            x: x as f32 + 0.13,
                            z: z as f32 + 0.37,
                        },
                        0.,
                    );
                    let (expected_index, (_, expected)) = (0..world_roads::road_lines(&world)
                        .len())
                        .map(|i| (i, world_roads::projected(&world, i, point)))
                        .min_by(|a, b| a.1 .0.total_cmp(&b.1 .0).then(a.0.cmp(&b.0)))
                        .unwrap();
                    let (index, actual) = world_roads::nearest_road(&world, point);
                    assert_eq!(
                        index, expected_index,
                        "compiled={compiled}, point={point:?}"
                    );
                    assert_eq!(actual.point, expected.point);
                    assert_eq!(actual.offset, expected.offset);
                    assert_eq!(actual.distance, expected.distance);
                    assert_eq!(actual.width, expected.width);
                }
            }
            // Minimap candidates retain stored path order and compiled bends.
            let segments = world_roads::road_segments_in(&world, world.bounding_box);
            let all: Vec<_> = world_roads::road_lines(&world)
                .iter()
                .flat_map(|r| r.path.windows(2).map(|p| (p[0], p[1])))
                .collect();
            assert_eq!(segments, all);
            assert!(world_roads::road_segments_in(
                &(world),
                BoundingBoxXZ {
                    min: XZ { x: 1000., z: 1000. },
                    max: XZ { x: 1001., z: 1001. }
                }
            )
            .is_empty());
        }
    }
}

#[cfg(test)]
#[path = "test_world_support.rs"]
pub mod test_support;

#[cfg(test)]
#[cfg(test)]
fn world_location_from_source(source: SourceLocation<f32>) -> Result<WorldLocation, String> {
    let mut city = model_world::empty();
    city.origin = source.origin;
    city.bounding_box = source.bounding_box;
    city.roads = road_network_from_source(source.road_nodes, source.road_lines);
    city.buildings = world_buildings::new(source.buildings);
    for entrance in source.entrances {
        city.entrances
            .entry(entrance.building_id)
            .or_default()
            .push(entrance.position);
    }
    Ok(city)
}
