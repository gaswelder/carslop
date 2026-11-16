//! Road geometry and the grid derived from its prepared paths are one owner.

use crate::model;
#[cfg(test)]
use crate::model::source_location::SourceLocation;
#[cfg(test)]
use crate::model::source_location::SourceRoad;
use crate::model::world::Curb;
use crate::model::world::HeightField;
use crate::model::world::Road;
use crate::model::world::RoadSample;
use crate::model::world::Triangle;
use crate::model::world::WorldLocation;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use crate::world::street_surface::RoadPaths;
use crate::world::street_surface::StreetSurface;
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
pub fn from_stored(nodes: Vec<XZ<f32>>, roads: Vec<model::location::Road>) -> RoadNetwork {
    from_roads(
        nodes
            .into_iter()
            .map(|p| util::geometry::at_height(p, 0.0))
            .collect(),
        roads
            .into_iter()
            .map(|r| Road {
                osm_way_id: r.way_id,
                direction: r.direction,
                ..model::world::new(r.ends, r.width_meters)
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
                ..model::world::new(r.ends, r.road_width_start_meters)
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
        world::spatial::insert(
            &mut grid,
            world::spatial::bounds(
                lines[i]
                    .ends
                    .iter()
                    .map(|&node| util::geometry::xz(nodes[node])),
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
            ..model::world::new([i, j], widths[i])
        });
        distance += length;
        world::spatial::insert(
            &mut grid,
            world::spatial::bounds([a, b].into_iter().map(|p| util::geometry::xz(p)), 32.0),
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
        world::spatial::insert(
            &mut grid,
            world::spatial::bounds(street.path.iter().map(|p| util::geometry::xz(*p)), 32.0),
            i,
        );
    }
    for p in &mut road_network.nodes {
        p.y = world::elevation::height(field, util::geometry::xz(*p));
    }
    for street in &mut road_network.lines {
        for p in &mut street.path {
            p.y = world::elevation::height(field, util::geometry::xz(*p));
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
    let deck = world::map_details::road_deck(&world.details, index);
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
    let mut point = util::geometry::xyz_from_fn(|i| {
        util::geometry::xyz_component(a, i)
            + (util::geometry::xyz_component(b, i) - util::geometry::xyz_component(a, i)) * t
    });
    // Compiled road surfaces are draped over the heightfield between path knots.
    if let Some(deck) = deck {
        if let Some(height) = world::map_details::height_on(deck, util::geometry::xz(point)) {
            point.y = height;
        }
    } else if world::street_surface::has_compiled_roads(world) {
        if let Some(field) = &world.elevation {
            point.y = world::elevation::height(field, util::geometry::xz(point))
                + world::raised_roads::road_rise(world, util::geometry::xz(point));
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
    let a = world::spatial::cell(XZ {
        x: bounds.min.x,
        z: bounds.min.z,
    });
    let b = world::spatial::cell(XZ {
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
        .get(&world::spatial::cell(XZ { x: p.x, z: p.z }))
    {
        return Box::new(ids.iter().copied());
    }
    for radius in [
        1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0,
    ] {
        let d = radius * world::spatial::CELL;
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
            let q = world::spatial::nearest_edge(point, a, b);
            let along = (q.x - a.x).hypot(q.z - a.z);
            let mut s = sample_distance(world, index, r.path_distances[i] + along);
            let delta = XZ {
                x: p.x - s.point.x,
                z: p.z - s.point.z,
            };
            s.offset = delta.x * model::world::normal(s).x + delta.z * model::world::normal(s).z;
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
    let origin = world::spatial::cell(XZ { x: p.x, z: p.z });
    let mut seen = std::collections::HashSet::new();
    let mut best: Option<(f32, usize, RoadSample)> = None;
    let limit = ([world.bounding_box.min, world.bounding_box.max]
        .iter()
        .flat_map(|b| [(b.x - p.x).abs(), (b.z - p.z).abs()])
        .fold(0.0_f32, f32::max)
        / world::spatial::CELL)
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
                            if !model::world::map_details(world).decks.is_empty() {
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
        if best.is_some_and(|b| b.0 <= (radius + 1) as f32 * world::spatial::CELL) {
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
    compiled_surfaces(world).map_or(&[], |s| world::street_surface::road_triangles(s))
}
pub fn curbs(world: &WorldLocation) -> &[Curb] {
    compiled_surfaces(world).map_or(&[], |s| world::street_surface::curbs(s))
}
pub fn pavement_height_meters(world: &WorldLocation) -> f32 {
    compiled_surfaces(world).map_or(world::street_surface::DEFAULT_PAVEMENT_HEIGHT, |s| {
        world::street_surface::pavement_height_meters(s)
    })
}

/// Distance to the union of street capsules, including shared junction nodes.
pub fn road_clearance(world: &WorldLocation, p: XYZ) -> f32 {
    if let Some(height) = world::transport::deck_height(world, util::geometry::xz(p), p.y) {
        if (height - p.y).abs() < 1. {
            let road = project(world, p);
            return road.offset.abs() - road.width / 2.;
        }
    }
    if world::street_surface::has_compiled_roads(world) {
        return world::street_surface::surface_clearance(world, XZ { x: p.x, z: p.z });
    }
    candidates(world, p)
        .map(|i| {
            let (d, s) = projected(world, i, p);
            d - s.width * 0.5
        })
        .fold(f32::INFINITY, f32::min)
}

#[cfg(test)]
#[path = "../test/world_support.rs"]
pub mod test_support;

#[cfg(test)]
#[cfg(test)]
fn world_location_from_source(source: SourceLocation<f32>) -> Result<WorldLocation, String> {
    let mut city = model::world::empty();
    city.origin = source.origin;
    city.bounding_box = source.bounding_box;
    city.roads = road_network_from_source(source.road_nodes, source.road_lines);
    city.buildings = world::buildings::new(source.buildings);
    for entrance in source.entrances {
        city.entrances
            .entry(entrance.building_id)
            .or_default()
            .push(entrance.position);
    }
    Ok(city)
}
