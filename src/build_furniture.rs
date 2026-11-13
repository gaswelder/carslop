//! Compile mapped road controls into furniture assets.
use crate::asset_codecs;
use crate::build_surfaces;
use crate::build_surfaces::Surfaces;
use crate::error::Result;
use crate::geometry::XZ;
use crate::geometry::{add, along, dist, dot, mul, path_length, project, sub, unit};
use crate::osm;
use crate::osm::{OSMElement, OSMMap};
use crate::source_location::SourceLocation;
use crate::world;
use geo::MultiPolygon;
use geo::{Buffer, Point};
use std::collections::BTreeMap;
use std::fs;
use std::io::{BufWriter, Write};
use std::path::Path;

struct Feature {
    id: i64,
    kind: usize,
    position: XZ,
    travel: XZ,
    value: u16,
    phase: i64,
    width: f64,
    source: XZ,
}

fn directions(e: &OSMElement, kind: usize) -> (Vec<i8>, bool, Option<XZ>) {
    let key = [
        "traffic_signals:direction",
        "stop:direction",
        "give_way:direction",
        "traffic_sign:direction",
    ][kind];
    let value = if e.tag(key).is_empty() {
        e.tag("direction")
    } else {
        e.tag(key)
    };
    match value {
        "forward" => (vec![1], false, None),
        "backward" => (vec![-1], false, None),
        "both" => (vec![-1, 1], false, None),
        _ => {
            let cardinal = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
                .iter()
                .position(|v| *v == value)
                .map(|i| i as f64 * 45.0);
            if let Some(angle) = cardinal
                .or_else(|| osm::number(value))
                .map(f64::to_radians)
            {
                (
                    vec![],
                    false,
                    Some(XZ {
                        x: -angle.sin(),
                        z: angle.cos(),
                    }),
                )
            } else {
                (vec![-1, 1], true, None)
            }
        }
    }
}

pub fn build_furniture(
    source: &OSMMap,
    map: &SourceLocation<f64>,
    s: &Surfaces,
    output_path: &Path,
) -> Result<()> {
    let placements = place_furniture(source, map, s);
    let mut file = BufWriter::new(fs::File::create(output_path)?);
    asset_codecs::furniture::write(&mut file, &placements)?;
    file.flush()?;
    Ok(())
}

/// Compute placements without creating files or quantizing source geometry.
fn place_furniture(
    source: &OSMMap,
    map: &SourceLocation<f64>,
    s: &Surfaces,
) -> Vec<asset_codecs::furniture::FurnitureRecord> {
    let tree = build_surfaces::index(&s.paths);
    let obstacles = build_surfaces::PolygonIndex::new(&s.obstacles);
    let conflicts = |probe: &MultiPolygon<f64>| obstacles.intersects(probe);
    let lookup: std::collections::HashMap<_, _> = map
        .road_nodes
        .iter()
        .enumerate()
        .map(|(i, p)| {
            (
                ((p.x * 1000.0).round() as i64, (p.z * 1000.0).round() as i64),
                i,
            )
        })
        .collect();
    let mut hosts: BTreeMap<i64, std::collections::BTreeSet<i64>> = BTreeMap::new();
    for road in &map.road_lines {
        let Some(way_id) = road.osm_way_id else {
            continue;
        };
        let way = &source.ways[&way_id];
        for id in &way.nodes {
            hosts.entry(*id).or_default().insert(way.id);
        }
    }
    let mut features: Vec<Feature> = vec![];
    for e in source.nodes.values() {
        let highway = e.tag("highway");
        let sign = e.tag("traffic_sign");
        let kind = if highway == "traffic_signals"
            || (highway == "crossing" && e.tag("crossing") == "traffic_signals")
        {
            0
        } else if highway == "stop" || sign == "stop" {
            1
        } else if highway == "give_way" || sign == "give_way" {
            2
        } else if sign == "maxspeed" {
            3
        } else {
            continue;
        };
        let p = map.metres(e.coord.unwrap());
        if !map.contains(p) {
            continue;
        }
        let value = if kind == 3 {
            let Some(v) =
                osm::number(e.tag("maxspeed")).filter(|v| *v >= 5.0 && *v <= 150.0)
            else {
                continue;
            };
            v as u16
        } else {
            0
        };
        let node = lookup
            .get(&((p.x * 1000.0).round() as i64, (p.z * 1000.0).round() as i64))
            .copied();
        let candidates = if let Some(n) = node {
            s.adjacency[n]
                .iter()
                .filter(|i| {
                    hosts.get(&e.id).is_some_and(|h| {
                        map.road_lines[**i]
                            .osm_way_id
                            .is_some_and(|id| h.contains(&id))
                    })
                })
                .copied()
                .collect::<Vec<_>>()
        } else {
            let closest = tree.nearest_neighbor(&p).unwrap().road_index;
            if project(&s.paths[closest], p).0 <= 15.0 {
                vec![closest]
            } else {
                vec![]
            }
        };
        let (directions, inferred, absolute) = directions(e, kind);
        let mut placements: Vec<Feature> = vec![];
        for i in candidates {
            let r = &map.road_lines[i];
            let path = &s.paths[i];
            let n = path_length(path);
            let direction = if let Some(node) = node {
                if r.ends[1] == node {
                    1
                } else {
                    -1
                }
            } else if r.direction != 0 {
                r.direction
            } else if directions.len() == 1 {
                directions[0]
            } else {
                1
            };
            if node.is_some()
                && ((absolute.is_none() && !directions.contains(&direction))
                    || (r.direction != 0 && direction != r.direction))
            {
                continue;
            }
            if inferred && matches!(kind, 1 | 2) {
                continue;
            }
            let station = if node.is_some() {
                if direction == 1 {
                    n
                } else {
                    0.0
                }
            } else {
                project(path, p).1
            };
            let tangent = unit(sub(
                along(path, (station + 0.5).min(n)),
                along(path, (station - 0.5).max(0.0)),
            ));
            let travel = absolute.unwrap_or(mul(tangent, direction as f64));
            if absolute.is_some()
                && node.is_some()
                && dot(travel, tangent) * (direction as f64) < 0.5
            {
                continue;
            }
            let setback = if node.is_some_and(|n| s.adjacency[n].len() >= 3) {
                (r.road_width_start_meters / 2.0 + 1.5)
                    .max(4.0)
                    .min(n * 0.7)
            } else {
                0.0
            };
            let left = XZ {
                x: travel.z,
                z: -travel.x,
            };
            let mut position = None;
            for extra in [0.0, 1.5, 3.0] {
                let anchor = along(
                    path,
                    (station - direction as f64 * (setback + extra)).clamp(0.0, n),
                );
                if node.is_none() && extra == 0.0 && !conflicts(&Point::new(p.x, p.z).buffer(0.18))
                {
                    position = Some(p);
                    break;
                }
                for step in 0..31 {
                    let q = add(
                        anchor,
                        mul(
                            left,
                            r.road_width_start_meters / 2.0 + 0.35 + step as f64 * 0.25,
                        ),
                    );
                    if map.contains(q) && !conflicts(&Point::new(q.x, q.z).buffer(0.18)) {
                        position = Some(q);
                        break;
                    }
                }
                if position.is_some() {
                    break;
                }
            }
            let Some(position) = position else {
                continue;
            };
            if placements
                .iter()
                .any(|f| dist(position, f.position) < 2.5 && dot(travel, f.travel) > 0.94)
            {
                continue;
            }
            placements.push(Feature {
                id: e.id,
                kind,
                position,
                travel,
                value,
                phase: 0,
                width: r.road_width_start_meters,
                source: p,
            });
        }
        features.extend(placements);
    }
    features.sort_by_key(|f| {
        (
            f.kind,
            source.nodes[&f.id].tag("highway") != "traffic_signals",
            f.id,
        )
    });
    let mut unique: Vec<Feature> = vec![];
    for f in features {
        if !unique.iter().any(|old| {
            old.kind == f.kind
                && old.value == f.value
                && dist(old.position, f.position) < 1.0
                && dot(old.travel, f.travel) > 0.98
        }) {
            unique.push(f);
        }
    }
    let mut pending: std::collections::BTreeSet<_> = unique
        .iter()
        .enumerate()
        .filter(|(_, f)| f.kind == 0)
        .map(|(i, _)| i)
        .collect();
    while let Some(first) = pending.first().copied() {
        let mut todo = vec![first];
        let mut component = vec![];
        while let Some(i) = todo.pop() {
            if !pending.remove(&i) {
                continue;
            }
            component.push(i);
            todo.extend(
                pending
                    .iter()
                    .filter(|j| dist(unique[i].source, unique[**j].source) < 40.0)
                    .copied(),
            );
        }
        let reference = *component
            .iter()
            .max_by(|a, b| {
                unique[**a]
                    .width
                    .total_cmp(&unique[**b].width)
                    .then(unique[**b].id.cmp(&unique[**a].id))
            })
            .unwrap();
        let axis = unique[reference].travel;
        let offset = component
            .iter()
            .map(|i| unique[*i].id)
            .min()
            .unwrap()
            .rem_euclid(50);
        for i in component {
            unique[i].phase = (offset
                + if dot(unique[i].travel, axis).abs() >= 0.5_f64.sqrt() {
                    0
                } else {
                    25
                })
                % 50;
        }
    }
    use world::FurnitureKind;
    unique
        .into_iter()
        .map(|f| asset_codecs::furniture::FurnitureRecord {
            source_id: f.id,
            kind: match f.kind {
                0 => FurnitureKind::Signal,
                1 => FurnitureKind::Stop,
                2 => FurnitureKind::Yield,
                3 => FurnitureKind::Speed,
                _ => unreachable!("unknown furniture kind"),
            },
            position: f.position,
            travel: f.travel,
            value: f.value,
            phase: f.phase as u8,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build_surfaces;
    use crate::geometry;
    use crate::geometry::{BoundingBoxXZ, LatLon};
    use crate::source_location::SourceRoad;
    use crate::test_support;
    use crate::world::FurnitureKind;

    fn element(id: i64, p: XZ, tags: &[(&str, &str)]) -> OSMElement {
        let mut element = test_support::osm_element(id, tags);
        element.coord = Some(LatLon::new(
            -p.z.to_degrees() / 6371000.,
            p.x.to_degrees() / 6371000.,
        ));
        element
    }

    fn junction() -> (OSMMap, SourceLocation<f64>, Surfaces) {
        let points = [
            XZ { x: -40., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 0., z: -40. },
        ];
        let mut source = OSMMap {
            nodes: BTreeMap::new(),
            ways: BTreeMap::new(),
            relations: vec![],
            bounds: None,
        };
        for (i, p) in points.into_iter().enumerate() {
            source
                .nodes
                .insert(i as i64 + 1, element(i as i64 + 1, p, &[]));
        }
        test_support::set_osm_tags(
            source.nodes.get_mut(&2).unwrap(),
            &[
                ("highway", "traffic_signals"),
                ("traffic_signals:direction", "both"),
            ],
        );
        for (id, nodes) in [(101, vec![1, 2]), (102, vec![3, 2])] {
            source.ways.insert(id, {
                let mut way = OSMElement::default();
                way.id = id;
                way.nodes = nodes;
                way
            });
        }
        let mut map = SourceLocation::empty();
        map.origin = Some(LatLon::new(0., 0.));
        map.bounding_box = BoundingBoxXZ {
            min: XZ { x: -50., z: -50. },
            max: XZ { x: 50., z: 50. },
        };
        map.road_nodes = points.into_iter().map(|p| p.at_height(0.)).collect();
        map.road_lines = vec![
            SourceRoad {
                osm_way_id: Some(101),
                ..SourceRoad::new([0, 1], 8.)
            },
            SourceRoad {
                osm_way_id: Some(102),
                ..SourceRoad::new([2, 1], 6.5)
            },
        ];
        let output = tempfile::tempdir().unwrap();
        let mut surfaces =
            build_surfaces::write_streets_txt(&map, &output.path().join("streets.txt"))
                .unwrap()
                .surfaces;
        surfaces.paths = vec![vec![points[0], points[1]], vec![points[2], points[1]]];
        surfaces.adjacency = vec![vec![0], vec![0, 1], vec![1]];
        surfaces.obstacles = MultiPolygon(vec![]);
        (source, map, surfaces)
    }

    #[test]
    fn placement_without_files_preserves_left_curbs_and_signal_phase_groups() {
        let (source, map, surfaces) = junction();
        let placed = place_furniture(&source, &map, &surfaces);
        assert_eq!(placed.len(), 2);
        assert!(placed
            .iter()
            .all(|p| p.kind == FurnitureKind::Signal && p.source_id == 2));
        let horizontal = placed.iter().find(|p| p.travel.x > 0.9).unwrap();
        let vertical = placed.iter().find(|p| p.travel.z > 0.9).unwrap();
        assert!(horizontal.position.z < -4.);
        assert!(vertical.position.x > 3.25);
        assert_eq!(horizontal.phase, 2);
        assert_eq!(vertical.phase, 27);
        // A wrong-way control must not be placed on either incoming one-way road.
        let mut wrong_way = map;
        for road in &mut wrong_way.road_lines {
            road.direction = -1;
        }
        assert!(place_furniture(&source, &wrong_way, &surfaces).is_empty());
    }

    #[test]
    fn placement_filters_ambiguous_stops_invalid_speeds_and_blocked_poles() {
        let (mut source, map, mut surfaces) = junction();
        let node = source.nodes.get_mut(&2).unwrap();
        test_support::set_osm_tags(node, &[("highway", "stop")]);
        assert!(place_furniture(&source, &map, &surfaces).is_empty());
        test_support::set_osm_tags(
            source.nodes.get_mut(&2).unwrap(),
            &[("highway", "stop"), ("direction", "forward")],
        );
        let stops = place_furniture(&source, &map, &surfaces);
        assert_eq!(stops.len(), 2);
        assert!(stops.iter().all(|p| p.kind == FurnitureKind::Stop));
        test_support::set_osm_tags(
            source.nodes.get_mut(&2).unwrap(),
            &[("traffic_sign", "maxspeed"), ("maxspeed", "200")],
        );
        assert!(place_furniture(&source, &map, &surfaces).is_empty());
        test_support::set_osm_tags(
            source.nodes.get_mut(&2).unwrap(),
            &[("traffic_sign", "maxspeed"), ("maxspeed", "30")],
        );
        let speeds = place_furniture(&source, &map, &surfaces);
        assert_eq!(speeds.len(), 2);
        assert!(speeds
            .iter()
            .all(|p| p.kind == FurnitureKind::Speed && p.value == 30));
        surfaces.obstacles = MultiPolygon(vec![geometry::polygon(&[
            XZ { x: -50., z: -50. },
            XZ { x: 50., z: -50. },
            XZ { x: 50., z: 50. },
            XZ { x: -50., z: 50. },
        ])]);
        assert!(place_furniture(&source, &map, &surfaces).is_empty());
    }
}
