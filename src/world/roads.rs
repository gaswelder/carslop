//! Road geometry and the grid derived from its prepared paths are one owner.
use super::*;
use crate::location;
#[cfg(test)]
use crate::source_location::{SourceLocation, SourceRoad};
use crate::street_surface;
use crate::street_surface::{RoadPaths, StreetSurface};
use crate::world;

#[derive(Debug)]
pub(super) struct RoadNetwork {
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

impl RoadNetwork {
    pub(super) fn empty() -> Self {
        Self {
            nodes: vec![],
            lines: vec![],
            grid: HashMap::new(),
            geometry: RoadGeometry::Centreline,
        }
    }
    pub(super) fn from_stored(nodes: Vec<XZ<f32>>, roads: Vec<location::Road>) -> Self {
        Self::from_roads(
            nodes.into_iter().map(|p| p.at_height(0.0)).collect(),
            roads
                .into_iter()
                .map(|r| Road {
                    osm_way_id: r.way_id,
                    direction: r.direction,
                    ..Road::new(r.ends, r.width_meters)
                })
                .collect(),
        )
    }
    #[cfg(test)]
    fn from_source(nodes: Vec<XYZ>, source_roads: Vec<SourceRoad<f32>>) -> Self {
        Self::from_roads(
            nodes,
            source_roads
                .into_iter()
                .map(|r| Road {
                    osm_way_id: r.osm_way_id,
                    direction: r.direction,
                    road_width_end_meters: r.road_width_start_meters,
                    ..Road::new(r.ends, r.road_width_start_meters)
                })
                .collect(),
        )
    }
    fn from_roads(nodes: Vec<XYZ>, source_roads: Vec<Road>) -> Self {
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
                    lines[i].ends.iter().map(|&node| nodes[node].xz()),
                    32.0,
                ),
                i,
            );
        }
        Self {
            nodes,
            lines,
            grid,
            geometry: RoadGeometry::Centreline,
        }
    }
    pub(super) fn authored(nodes: Vec<XYZ>, widths: Vec<f32>) -> Result<Self, String> {
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
                ..Road::new([i, j], widths[i])
            });
            distance += length;
            world::spatial::insert(
                &mut grid,
                world::spatial::bounds([a, b].into_iter().map(|p| p.xz()), 32.0),
                i,
            );
        }
        Ok(Self {
            nodes,
            lines,
            grid,
            geometry: RoadGeometry::Centreline,
        })
    }
    pub(super) fn with_compiled(
        mut self,
        surfaces: StreetSurface,
        paths: RoadPaths,
        field: &HeightField,
    ) -> Result<Self, String> {
        if paths.len() != self.lines.len() || paths.iter().any(|p| p.len() < 2) {
            return Err("streets: incomplete compiled road paths".into());
        }
        let mut grid = std::mem::take(&mut self.grid);
        grid.clear();
        let mut distance = 0.0;
        for (i, (street, path)) in self.lines.iter_mut().zip(paths).enumerate() {
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
                world::spatial::bounds(street.path.iter().map(|p| p.xz()), 32.0),
                i,
            );
        }
        for p in &mut self.nodes {
            p.y = field.height(p.xz());
        }
        for street in &mut self.lines {
            for p in &mut street.path {
                p.y = field.height(p.xz());
            }
        }
        self.grid = grid;
        self.geometry = RoadGeometry::Compiled(surfaces);
        Ok(self)
    }
    #[cfg(test)]
    fn bump(&mut self) {
        for street in &mut self.lines {
            for (point, station) in street
                .path
                .iter_mut()
                .zip([street.distance, street.distance + street.length])
            {
                point.y = 0.14 * (-((station - 55.0) / 1.0).powi(2)).exp();
            }
        }
        for (point, street) in self.nodes.iter_mut().zip(&self.lines) {
            *point = street.path[0];
        }
    }
}

impl Location {
    pub fn road_lines(&self) -> &[Road] {
        &self.roads.lines
    }
    pub fn sample(&self, index: usize, t: f32) -> RoadSample {
        self.sample_distance(index, t.clamp(0.0, 1.0) * self.roads.lines[index].length)
    }

    pub fn sample_distance(&self, index: usize, distance: f32) -> RoadSample {
        let r = &self.roads.lines[index];
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
        let mut point = XYZ::from_fn(|i| a.component(i) + (b.component(i) - a.component(i)) * t);
        // Compiled road surfaces are draped over the heightfield between path knots.
        if self.has_compiled_roads() {
            if let Some(field) = &self.elevation {
                point.y = field.height(XZ {
                    x: point.x,
                    z: point.z,
                });
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
    fn street_ids_in(&self, bounds: BoundingBoxXZ<f32>) -> Vec<usize> {
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
                if let Some(values) = self.roads.grid.get(&(x, z)) {
                    ids.extend(values);
                }
            }
        }
        ids.sort_unstable();
        ids.dedup();
        ids
    }
    pub fn road_segments_in(&self, bounds: BoundingBoxXZ<f32>) -> Vec<(XYZ, XYZ)> {
        self.street_ids_in(bounds)
            .into_iter()
            .flat_map(|i| self.roads.lines[i].path.windows(2).map(|p| (p[0], p[1])))
            .collect()
    }
    pub fn candidates(&self, p: XYZ) -> Box<dyn Iterator<Item = usize> + '_> {
        if let Some(ids) = self
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
            let ids = self.street_ids_in(BoundingBoxXZ {
                min: XZ {
                    x: p.x - d,
                    z: p.z - d,
                },
                max: XZ {
                    x: p.x + d,
                    z: p.z + d,
                },
            });
            if !ids.is_empty() {
                return Box::new(ids.into_iter());
            }
        }
        Box::new(0..self.roads.lines.len())
    }
    pub fn projected(&self, index: usize, p: XYZ) -> (f32, RoadSample) {
        let r = &self.roads.lines[index];
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
                let mut s = self.sample_distance(index, r.path_distances[i] + along);
                let delta = XZ {
                    x: p.x - s.point.x,
                    z: p.z - s.point.z,
                };
                s.offset = delta.x * s.normal().x + delta.z * s.normal().z;
                (delta.x.hypot(delta.z), s)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap()
    }
    pub fn project(&self, p: XYZ) -> RoadSample {
        self.nearest_road(p).1
    }
    /// Nearest road index and its local geometric frame.
    pub fn nearest_road(&self, p: XYZ) -> (usize, RoadSample) {
        let origin = world::spatial::cell(XZ { x: p.x, z: p.z });
        let mut seen = std::collections::HashSet::new();
        let mut best: Option<(f32, usize, RoadSample)> = None;
        let limit = ([self.bounding_box.min, self.bounding_box.max]
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
                    if let Some(ids) = self.roads.grid.get(&(origin.0 + dx, origin.1 + dz)) {
                        for &i in ids {
                            if seen.insert(i) {
                                let candidate = self.projected(i, p);
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
    pub fn road_name(&self, index: usize) -> Option<&str> {
        self.roads.lines[index]
            .osm_way_id
            .and_then(|id| self.street_names.get(&id))
            .map(String::as_str)
    }
}
impl Location {
    pub fn compiled_surfaces(&self) -> Option<&StreetSurface> {
        match &self.roads.geometry {
            RoadGeometry::Compiled(surfaces) => Some(surfaces),
            RoadGeometry::Centreline => None,
        }
    }
    pub fn road_triangles(&self) -> &[Triangle] {
        self.compiled_surfaces().map_or(&[], |s| s.road_triangles())
    }
    pub fn curbs(&self) -> &[Curb] {
        self.compiled_surfaces().map_or(&[], |s| s.curbs())
    }
    pub fn pavement_height_meters(&self) -> f32 {
        self.compiled_surfaces()
            .map_or(street_surface::DEFAULT_PAVEMENT_HEIGHT, |s| {
                s.pavement_height_meters()
            })
    }
}

#[cfg(test)]
mod tests {
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
                let (surfaces, paths) = StreetSurface::load(text, 2).unwrap();
                let field = HeightField::generate(world.bounding_box, |p| p.x * 0.01 + p.z * 0.02);
                world.roads = world.roads.with_compiled(surfaces, paths, &field).unwrap();
                world.elevation = Some(field);
                assert_eq!(world.road_lines()[0].path.len(), 3);
            }
            for x in (-200..=200).step_by(13) {
                for z in (-200..=200).step_by(17) {
                    let point = XZ {
                        x: x as f32 + 0.13,
                        z: z as f32 + 0.37,
                    }
                    .at_height(0.);
                    let (expected_index, (_, expected)) = (0..world.road_lines().len())
                        .map(|i| (i, world.projected(i, point)))
                        .min_by(|a, b| a.1 .0.total_cmp(&b.1 .0).then(a.0.cmp(&b.0)))
                        .unwrap();
                    let (index, actual) = world.nearest_road(point);
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
            let segments = world.road_segments_in(world.bounding_box);
            let all: Vec<_> = world
                .road_lines()
                .iter()
                .flat_map(|r| r.path.windows(2).map(|p| (p[0], p[1])))
                .collect();
            assert_eq!(segments, all);
            assert!(world
                .road_segments_in(BoundingBoxXZ {
                    min: XZ { x: 1000., z: 1000. },
                    max: XZ { x: 1001., z: 1001. }
                })
                .is_empty());
        }
    }
}

#[cfg(test)]
#[path = "../world_test_support.rs"]
pub mod test_support;

#[cfg(test)]
impl Location {
    #[cfg(test)]
    fn from_source(source: SourceLocation<f32>) -> Result<Self, String> {
        let mut city = Self::empty();
        city.origin = source.origin;
        city.bounding_box = source.bounding_box;
        city.roads = RoadNetwork::from_source(source.road_nodes, source.road_lines);
        city.buildings = Buildings::new(source.buildings);
        Ok(city)
    }
}
