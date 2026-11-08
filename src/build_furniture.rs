//! Compile mapped road controls into furniture assets.
use crate::build_surfaces::Surfaces;
use crate::location::{add, along, dist, dot, mul, path_length, project, sub, unit, Result};
use crate::osm::{OSMElement, OSMMap};
use crate::world::{LocationMap, P};
use geo::MultiPolygon;
use geo::{Buffer, Point};
use std::collections::BTreeMap;
use std::fs;
use std::io::{BufWriter, Write};
use std::path::Path;

struct Feature {
    id: i64,
    kind: usize,
    position: P,
    travel: P,
    value: u16,
    phase: i64,
    width: f64,
    source: P,
}

pub fn directions(e: &OSMElement, kind: usize) -> (Vec<i8>, bool, Option<P>) {
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
                .or_else(|| crate::osm::number(value))
                .map(f64::to_radians)
            {
                (vec![], false, Some([-angle.sin(), angle.cos()]))
            } else {
                (vec![-1, 1], true, None)
            }
        }
    }
}

pub fn build_furniture(
    source: &OSMMap,
    map: &LocationMap,
    s: &Surfaces,
    checksum: u32,
    output_path: &Path,
) -> Result<()> {
    let tree = crate::build_surfaces::index(&s.paths);
    let obstacles = crate::build_surfaces::PolygonIndex::new(&s.obstacles);
    let conflicts = |probe: &MultiPolygon<f64>| obstacles.intersects(probe);
    let lookup: std::collections::HashMap<_, _> = map
        .nodes
        .iter()
        .enumerate()
        .map(|(i, p)| {
            (
                (
                    (p[0] * 1000.0).round() as i64,
                    (p[1] * 1000.0).round() as i64,
                ),
                i,
            )
        })
        .collect();
    let mut hosts: BTreeMap<i64, std::collections::BTreeSet<i64>> = BTreeMap::new();
    for road in &map.roads {
        for id in &source.ways[&road.way].nodes {
            hosts.entry(*id).or_default().insert(road.way);
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
                crate::osm::number(e.tag("maxspeed")).filter(|v| *v >= 5.0 && *v <= 150.0)
            else {
                continue;
            };
            v as u16
        } else {
            0
        };
        let node = lookup
            .get(&(
                (p[0] * 1000.0).round() as i64,
                (p[1] * 1000.0).round() as i64,
            ))
            .copied();
        let candidates = if let Some(n) = node {
            s.adjacency[n]
                .iter()
                .filter(|i| {
                    hosts
                        .get(&e.id)
                        .is_some_and(|h| h.contains(&map.roads[**i].way))
                })
                .copied()
                .collect::<Vec<_>>()
        } else {
            let closest = tree.nearest_neighbor(&p).unwrap().road;
            if project(&s.paths[closest], p).0 <= 15.0 {
                vec![closest]
            } else {
                vec![]
            }
        };
        let (directions, inferred, absolute) = directions(e, kind);
        let mut placements: Vec<Feature> = vec![];
        for i in candidates {
            let r = &map.roads[i];
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
                (r.width / 2.0 + 1.5).max(4.0).min(n * 0.7)
            } else {
                0.0
            };
            let left = [travel[1], -travel[0]];
            let mut position = None;
            for extra in [0.0, 1.5, 3.0] {
                let anchor = along(
                    path,
                    (station - direction as f64 * (setback + extra)).clamp(0.0, n),
                );
                if node.is_none()
                    && extra == 0.0
                    && !conflicts(&Point::new(p[0], p[1]).buffer(0.18))
                {
                    position = Some(p);
                    break;
                }
                for step in 0..31 {
                    let q = add(anchor, mul(left, r.width / 2.0 + 0.35 + step as f64 * 0.25));
                    if map.contains(q) && !conflicts(&Point::new(q[0], q[1]).buffer(0.18)) {
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
                width: r.width,
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
    let mut file = BufWriter::new(fs::File::create(output_path)?);
    writeln!(file, "FURNITURE 3 {:08x} {:08x}", checksum, s.text_crc)?;
    for f in &unique {
        let kind = match f.kind {
            0 => "signal",
            1 => "stop",
            2 => "yield",
            3 => "speed_limit",
            _ => unreachable!("unknown furniture kind"),
        };
        writeln!(
            file,
            "{} {} {:.3} {:.3} {:.7} {:.7} {} {}",
            f.id, kind, f.position[0], f.position[1], f.travel[0], f.travel[1], f.value, f.phase
        )?;
    }
    file.flush()?;
    Ok(())
}
