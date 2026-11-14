//! Compile mapped humps, bumps and tables into a shared render/contact surface.
use crate::asset_codecs;
use crate::build_surfaces;
use crate::error::Result;
use crate::geometry;
use crate::geometry::XZ;
use crate::osm;
use crate::source_location::SourceLocation;
use crate::world::RoadRise;
use geo::{BooleanOps, MultiPolygon};
use std::{collections::BTreeMap, fs, path::Path};

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Hump,
    Bump,
    Table,
}
impl Kind {
    fn from_element(element: &osm::OSMElement) -> Option<Self> {
        match element.tag("traffic_calming") {
            "hump" => Some(Self::Hump),
            "bump" => Some(Self::Bump),
            "table" => Some(Self::Table),
            _ => None,
        }
    }
    fn length(self) -> f64 {
        match self {
            Self::Hump => 3.6,
            Self::Bump => 0.8,
            Self::Table => 6.0,
        }
    }
    fn height(self) -> f64 {
        match self {
            Self::Hump => 0.08,
            Self::Bump => 0.06,
            Self::Table => 0.10,
        }
    }
    fn profile(self, station: f64, length: f64) -> f64 {
        if self == Self::Table {
            let ramp = 1.2_f64.min(length * 0.25);
            (station / ramp)
                .min((length - station) / ramp)
                .clamp(0.0, 1.0)
        } else {
            (1.0 - (2.0 * station / length - 1.0).powi(2)).max(0.0)
        }
    }
}
fn ground_level(element: &osm::OSMElement) -> bool {
    matches!(element.tag("layer"), "" | "0")
        && ["bridge", "tunnel"]
            .iter()
            .all(|key| matches!(element.tag(key), "" | "no"))
}
struct Feature {
    path: Vec<XZ>,
    width: f64,
    height: f64,
    kind: Kind,
}

fn append(
    feature: Feature,
    road: &MultiPolygon<f64>,
    covered: &mut MultiPolygon<f64>,
    result: &mut Vec<RoadRise>,
) -> Result<()> {
    let length = geometry::path_length(&feature.path);
    if length < 0.2 {
        return Ok(());
    }
    let mut stations = vec![0.0, length];
    if feature.kind == Kind::Table {
        let ramp = 1.2_f64.min(length * 0.25);
        stations.extend([ramp, length - ramp]);
    } else {
        stations.extend((1..12).map(|i| length * i as f64 / 12.0));
    }
    let mut station = 0.0;
    for pair in feature.path.windows(2) {
        station += geometry::dist(pair[0], pair[1]);
        stations.push(station);
    }
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|a, b| (*a - *b).abs() < 0.0001);
    let mut sections = Vec::new();
    for station in stations {
        let p = geometry::along(&feature.path, station);
        let travel = geometry::unit(geometry::sub(
            geometry::along(&feature.path, (station + 0.01).min(length)),
            geometry::along(&feature.path, (station - 0.01).max(0.0)),
        ));
        let normal = XZ {
            x: -travel.z,
            z: travel.x,
        };
        let height = (feature.height * feature.kind.profile(station, length)) as f32;
        sections.push([
            geometry::add(p, geometry::mul(normal, feature.width / 2.0))
                .map(|v| v as f32)
                .at_height(height),
            geometry::sub(p, geometry::mul(normal, feature.width / 2.0))
                .map(|v| v as f32)
                .at_height(height),
        ]);
    }
    let pieces: Vec<_> = sections
        .windows(2)
        .map(|pair| {
            MultiPolygon(vec![geometry::polygon(
                &[pair[0][0], pair[0][1], pair[1][1], pair[1][0]].map(|p| p.xz().map(f64::from)),
            )])
        })
        .collect();
    let footprint = build_surfaces::union(&pieces).intersection(road);
    let nearby = build_surfaces::PolygonIndex::new(covered).nearby(&footprint);
    let occupied = build_surfaces::union(
        &nearby
            .0
            .into_iter()
            .map(|polygon| MultiPolygon(vec![polygon]))
            .collect::<Vec<_>>(),
    );
    let available = footprint.difference(&occupied);
    for pair in sections.windows(2) {
        for points in [
            [pair[0][0], pair[0][1], pair[1][1]],
            [pair[0][0], pair[1][1], pair[1][0]],
        ] {
            let plane = RoadRise { points };
            let footprint = MultiPolygon(vec![geometry::polygon(
                &points.map(|p| p.xz().map(f64::from)),
            )])
            .intersection(&available);
            build_surfaces::triangulate(&footprint, |points| {
                let points = points.map(|p| p.map(|v| v as f32));
                let rise = RoadRise {
                    points: points.map(|p| p.at_height(plane.offset(p).unwrap_or(0.0))),
                };
                // Validate the f32 footprint too, just as the persisted reader does.
                let [a, b, c] = rise.points;
                let area = ((b.x - a.x) as f64 * (c.z - a.z) as f64
                    - (b.z - a.z) as f64 * (c.x - a.x) as f64)
                    .abs();
                if area >= 0.00001 {
                    result.push(rise);
                }
                Ok(())
            })?;
        }
    }
    covered.0.extend(footprint.0);
    Ok(())
}

fn extract(
    source: &osm::OSMMap,
    map: &SourceLocation<f64>,
    surfaces: &build_surfaces::Surfaces,
) -> Result<Vec<RoadRise>> {
    let mut roads: BTreeMap<i64, Vec<usize>> = BTreeMap::new();
    for (i, road) in map.road_lines.iter().enumerate() {
        if let Some(id) = road.osm_way_id {
            roads.entry(id).or_default().push(i);
        }
    }
    let mut hosts: BTreeMap<i64, Vec<i64>> = BTreeMap::new();
    for id in roads.keys() {
        if let Some(way) = source.ways.get(id) {
            for node in &way.nodes {
                hosts.entry(*node).or_default().push(*id);
            }
        }
    }
    let excluded = build_surfaces::union(
        &map.buildings
            .iter()
            .map(|building| MultiPolygon(vec![geometry::polygon(&building.ground_outline)]))
            .collect::<Vec<_>>(),
    );
    let road = surfaces.asphalt.difference(&excluded);
    let crop = map.boundary.clone().unwrap_or_else(|| {
        MultiPolygon(vec![geometry::polygon(&[
            map.bounding_box.min,
            XZ {
                x: map.bounding_box.max.x,
                z: map.bounding_box.min.z,
            },
            map.bounding_box.max,
            XZ {
                x: map.bounding_box.min.x,
                z: map.bounding_box.max.z,
            },
        ])])
    });
    let road = road.intersection(&crop);
    let mut covered = geometry::empty();
    let mut result = Vec::new();
    let mut inherited = BTreeMap::new();
    // Ways mapped as raised road sections use their actual extent, including bends.
    for way in source
        .ways
        .values()
        .filter(|way| ground_level(way) && Kind::from_element(way).is_some())
    {
        let kind = Kind::from_element(way).unwrap();
        if let Some(indices) = roads.get(&way.id) {
            let Some(mut path) = way
                .nodes
                .iter()
                .map(|id| {
                    source
                        .nodes
                        .get(id)
                        .and_then(|node| node.coord)
                        .map(|p| map.metres(p))
                })
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            path.dedup();
            append(
                Feature {
                    path,
                    width: map.road_lines[indices[0]].road_width_start_meters,
                    height: osm::number(way.tag("height"))
                        .filter(|h| *h > 0.)
                        .unwrap_or(kind.height())
                        .clamp(0.02, 0.25),
                    kind,
                },
                &road,
                &mut covered,
                &mut result,
            )?;
        } else if way.tag("footway") == "crossing" || way.tag("path") == "crossing" {
            for id in &way.nodes {
                inherited.entry(*id).or_insert(way);
            }
        }
    }
    for (id, way_ids) in hosts {
        let Some(node) = source.nodes.get(&id) else {
            continue;
        };
        if !ground_level(node) {
            continue;
        }
        let element = if !node.tag("traffic_calming").is_empty() {
            node
        } else if let Some(way) = inherited.get(&id) {
            way
        } else {
            continue;
        };
        let Some(kind) = Kind::from_element(element) else {
            continue;
        };
        let Some(coord) = node.coord else {
            continue;
        };
        let p = map.metres(coord);
        if !map.contains(p) {
            continue;
        }
        // A table tagged on both its road way and a crossing node is one surface.
        if way_ids.iter().any(|id| {
            source
                .ways
                .get(id)
                .is_some_and(|way| Kind::from_element(way) == Some(kind))
        }) {
            continue;
        }
        let indices: Vec<_> = way_ids
            .iter()
            .flat_map(|id| roads[id].iter().copied())
            .collect();
        let Some(i) = indices.into_iter().min_by(|a, b| {
            geometry::project(&surfaces.paths[*a], p)
                .0
                .total_cmp(&geometry::project(&surfaces.paths[*b], p).0)
        }) else {
            continue;
        };
        let path = &surfaces.paths[i];
        let (distance, station) = geometry::project(path, p);
        if distance > 3.0 {
            continue;
        }
        let length = osm::number(element.tag("length"))
            .filter(|v| *v > 0.)
            .unwrap_or(kind.length())
            .clamp(0.4, 30.0);
        let width = if !node.tag("traffic_calming").is_empty() {
            osm::number(element.tag("width"))
                .filter(|v| *v > 0.)
                .unwrap_or(map.road_lines[i].road_width_start_meters)
        } else {
            map.road_lines[i].road_width_start_meters
        };
        let center = geometry::along(path, station);
        let tangent = geometry::unit(geometry::sub(
            geometry::along(path, (station + 0.5).min(geometry::path_length(path))),
            geometry::along(path, (station - 0.5).max(0.0)),
        ));
        append(
            Feature {
                path: vec![
                    geometry::sub(center, geometry::mul(tangent, length / 2.0)),
                    geometry::add(center, geometry::mul(tangent, length / 2.0)),
                ],
                width: width.clamp(1.0, 20.0),
                height: osm::number(element.tag("height"))
                    .filter(|v| *v > 0.)
                    .unwrap_or(kind.height())
                    .clamp(0.02, 0.25),
                kind,
            },
            &road,
            &mut covered,
            &mut result,
        )?;
    }
    Ok(result)
}

pub fn build_traffic_calming(
    source: &osm::OSMMap,
    map: &SourceLocation<f64>,
    surfaces: &build_surfaces::Surfaces,
    path: &Path,
) -> Result<()> {
    let rises = extract(source, map, surfaces)?;
    println!("  Writing {} raised road triangles...", rises.len());
    asset_codecs::road_rises::write(
        &mut std::io::BufWriter::new(fs::File::create(path)?),
        &rises,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::LatLon;
    use crate::source_location::SourceRoad;
    use crate::test_support;
    use geo::Area;

    #[test]
    fn calming_profiles_preserve_mapped_dimensions_and_skip_nearby_bridges() {
        let mut map = SourceLocation::empty();
        map.origin = Some(LatLon::new(0., 0.));
        map.bounding_box.min = XZ { x: -50., z: -50. };
        map.bounding_box.max = XZ { x: 50., z: 50. };
        let mut source = osm::OSMMap {
            nodes: BTreeMap::new(),
            ways: BTreeMap::new(),
            relations: Vec::new(),
            bounds: None,
        };
        let points: [XZ; 8] = [
            XZ { x: -40., z: 0. },
            XZ { x: -15., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 10., z: 0. },
            XZ { x: 40., z: 0. },
            XZ { x: 20., z: 3. },
            XZ { x: 30., z: 3. },
            XZ { x: 5., z: 0. },
        ];
        for (i, p) in points.iter().enumerate() {
            let mut node = test_support::osm_element(i as i64 + 1, &[]);
            node.coord = Some(LatLon::new(
                -p.z.to_degrees() / 6371000.,
                p.x.to_degrees() / 6371000.,
            ));
            source.nodes.insert(node.id, node);
        }
        test_support::set_osm_tags(
            source.nodes.get_mut(&1).unwrap(),
            &[("traffic_calming", "bump")],
        );
        test_support::set_osm_tags(
            source.nodes.get_mut(&2).unwrap(),
            &[
                ("traffic_calming", "hump"),
                ("height", "0.09"),
                ("length", "4"),
                ("width", "4"),
            ],
        );
        test_support::set_osm_tags(
            source.nodes.get_mut(&6).unwrap(),
            &[("traffic_calming", "bump")],
        );
        test_support::set_osm_tags(
            source.nodes.get_mut(&8).unwrap(),
            &[("traffic_calming", "table")],
        );
        struct Way {
            id: i64,
            nodes: Vec<i64>,
            tags: Vec<(&'static str, &'static str)>,
        }
        for fixture in [
            Way {
                id: 100,
                nodes: vec![1, 2, 3],
                tags: vec![("highway", "residential")],
            },
            Way {
                id: 101,
                nodes: vec![3, 8, 4],
                tags: vec![
                    ("highway", "residential"),
                    ("traffic_calming", "table"),
                    ("height", "0.12"),
                ],
            },
            Way {
                id: 102,
                nodes: vec![4, 5],
                tags: vec![("highway", "residential")],
            },
            Way {
                id: 103,
                nodes: vec![6, 7],
                tags: vec![("highway", "residential"), ("bridge", "yes")],
            },
        ] {
            let mut way = test_support::osm_element(fixture.id, &fixture.tags);
            way.nodes = fixture.nodes;
            source.ways.insert(way.id, way);
        }
        map.road_nodes = points[..5].iter().map(|p| p.at_height(0.)).collect();
        for (i, id) in [100, 100, 101, 102].into_iter().enumerate() {
            let mut road = SourceRoad::new([i, i + 1], 8.);
            road.osm_way_id = Some(id);
            map.road_lines.push(road);
        }
        let folder = tempfile::tempdir().unwrap();
        let surfaces = build_surfaces::write_streets_txt(
            &map,
            &folder.path().join("streets.txt"),
            Some(&source),
        )
        .unwrap()
        .surfaces;
        let path = folder.path().join("road-rises.txt");
        build_traffic_calming(&source, &map, &surfaces, &path).unwrap();
        let rises = asset_codecs::road_rises::parse(&fs::read_to_string(path).unwrap()).unwrap();
        let height = |x, z| {
            rises
                .iter()
                .filter_map(|rise| rise.offset(XZ { x, z }))
                .fold(0., f32::max)
        };
        assert!((height(-15., 0.) - 0.09).abs() < 0.0001);
        assert!((height(-16., 0.) - 0.0675).abs() < 0.0001);
        assert_eq!(height(-15., 3.), 0.);
        assert_eq!(height(-12., 0.), 0.);
        assert!((height(-40., 0.) - 0.06).abs() < 0.0001);
        assert!((height(5., 0.) - 0.12).abs() < 0.0001);
        assert!((height(0.6, 0.) - 0.06).abs() < 0.0001);
        assert_eq!(height(12., 0.), 0.);
        assert_eq!(height(20., 3.), 0.);
        assert_eq!(height(5., 4.5), 0.);
        let polygons: Vec<_> = rises
            .iter()
            .map(|rise| {
                MultiPolygon(vec![geometry::polygon(
                    &rise.points.map(|p| p.xz().map(f64::from)),
                )])
            })
            .collect();
        let sum: f64 = polygons.iter().map(|polygon| polygon.unsigned_area()).sum();
        assert!(
            (sum - build_surfaces::union(&polygons).unsigned_area()).abs() < 0.001,
            "node/way tags must not duplicate surfaces"
        );
    }
}
