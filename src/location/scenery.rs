use super::surfaces::{union, Surfaces};
use super::*;
use geo::{algorithm::orient::Direction, Area, BooleanOps, Buffer, Orient, Point};
use std::io::{BufWriter, Write};
fn style(e: &Element) -> Option<usize> {
    let surface = e.tag("surface");
    if e.tag("leisure") == "swimming_pool" || e.tag("natural") == "water" {
        Some(8)
    } else if matches!(e.tag("leisure"), "pitch" | "track") {
        Some(7)
    } else if e.tag("amenity") == "parking" {
        Some(match surface {
            "gravel" | "dirt" | "ground" | "unpaved" => 6,
            "paving_stones" => 4,
            "concrete" => 5,
            _ => 3,
        })
    } else if matches!(e.tag("highway"), "pedestrian" | "footway" | "path")
        || !e.tag("area:highway").is_empty()
    {
        Some(match surface {
            "asphalt" => 3,
            "concrete" => 5,
            "gravel" | "dirt" | "ground" | "unpaved" | "compacted" => 6,
            _ => 4,
        })
    } else if e.tag("natural") == "scrub" {
        Some(2)
    } else if matches!(e.tag("landuse"), "grass" | "meadow")
        || matches!(e.tag("natural"), "grassland" | "wood")
    {
        Some(0)
    } else if matches!(
        e.tag("leisure"),
        "park" | "garden" | "common" | "playground"
    ) {
        Some(1)
    } else if matches!(e.tag("landuse"), "greenfield" | "construction") {
        Some(6)
    } else {
        None
    }
}
// Join split relation members by node identity, including reversed OSM ways.
fn rings(mut parts: Vec<Vec<i64>>) -> Option<Vec<Vec<i64>>> {
    let mut rings = vec![];
    while let Some(mut ring) = parts.pop() {
        while ring.first() != ring.last() {
            let end = *ring.last()?;
            let i = parts
                .iter()
                .position(|p| p.first() == Some(&end) || p.last() == Some(&end))?;
            let mut next = parts.swap_remove(i);
            if next.last() == Some(&end) {
                next.reverse();
            }
            ring.extend_from_slice(&next[1..]);
        }
        if ring.len() < 4 {
            return None;
        }
        rings.push(ring);
    }
    Some(rings)
}
pub fn build(source: &Source, map: &Map, s: &Surfaces, city: &str, output: &Path) -> Result<()> {
    let coord = |id: &i64| source.nodes.get(id).map(|e| map.metres(e.coord.unwrap()));
    let mut groups: Vec<Vec<MultiPolygon<f64>>> = vec![vec![]; 9];
    let mut consumed = std::collections::BTreeSet::new();
    let mut incomplete = 0;
    let crop = polygon(&[
        [map.bounds[0], map.bounds[1]],
        [map.bounds[2], map.bounds[1]],
        [map.bounds[2], map.bounds[3]],
        [map.bounds[0], map.bounds[3]],
    ]);
    for relation in &source.relations {
        let Some(kind) = style(relation) else {
            continue;
        };
        if relation.tag("type") != "multipolygon" {
            continue;
        }
        let mut outer = vec![];
        let mut inner = vec![];
        let mut members = vec![];
        let mut valid = true;
        for (typ, id, role) in &relation.members {
            if typ != "way" {
                continue;
            }
            let Some(way) = source.ways.get(id) else {
                valid = false;
                break;
            };
            if way.nodes.len() < 2 || way.nodes.iter().any(|id| !source.nodes.contains_key(id)) {
                valid = false;
                break;
            }
            if role == "inner" {
                inner.push(way.nodes.clone());
            } else {
                outer.push(way.nodes.clone());
            }
            members.push(*id);
        }
        let assembled = if valid {
            rings(outer).zip(rings(inner))
        } else {
            None
        };
        let Some((outer, inner)) = assembled else {
            incomplete += 1;
            continue;
        };
        let make = |rings: Vec<Vec<i64>>| {
            union(
                &rings
                    .into_iter()
                    .map(|r| {
                        MultiPolygon(vec![polygon(
                            &r.iter().filter_map(&coord).collect::<Vec<_>>(),
                        )
                        .orient(Direction::Default)])
                    })
                    .collect::<Vec<_>>(),
            )
        };
        let geometry = make(outer).difference(&make(inner)).intersection(&crop);
        groups[kind].push(geometry);
        consumed.extend(members);
    }
    for way in source.ways.values() {
        let Some(kind) = style(way) else {
            continue;
        };
        if consumed.contains(&way.id)
            || way.nodes.len() < 4
            || way.nodes.first() != way.nodes.last()
        {
            continue;
        }
        let Some(points) = way.nodes.iter().map(&coord).collect::<Option<Vec<_>>>() else {
            continue;
        };
        groups[kind].push(polygon(&points).intersection(&crop));
    }
    let mut file = BufWriter::new(fs::File::create(output.join("landscape.txt"))?);
    let city_crc = crate::street_surface::crc32(city);
    writeln!(file, "LANDSCAPE 1 {city_crc:08x} {:08x}", s.text_crc)?;
    let mut covered = empty();
    let mut count = 0;
    let mut areas = BTreeMap::new();
    for kind in [8, 7, 3, 4, 5, 2, 0, 1, 6] {
        let geometry = union(&groups[kind]).difference(&covered);
        covered = covered.union(&geometry);
        areas.insert(kind, geometry.unsigned_area());
        count += super::surfaces::triangles(&mut file, &format!("a {kind}"), &geometry)?;
    }
    println!("  Filtering mapped trees...");
    let obstacles = super::surfaces::PolygonIndex::new(&s.obstacles);
    let conflicts = |probe: &MultiPolygon<f64>| obstacles.intersects(probe);
    let mut trees = 0;
    let mut suppressed = 0;
    for node in source.nodes.values().filter(|n| n.tag("natural") == "tree") {
        let p = map.metres(node.coord.unwrap());
        if !map.contains(p) {
            continue;
        }
        if conflicts(&Point::new(p[0], p[1]).buffer(0.35)) {
            suppressed += 1;
            continue;
        }
        let height = super::osm::number(node.tag("height"))
            .filter(|n| *n > 0.0)
            .unwrap_or(4.8 + node.id.rem_euclid(7) as f64 * 0.3)
            .clamp(2.5, 18.0);
        let radius = super::osm::number(node.tag("diameter_crown"))
            .filter(|n| *n > 0.0)
            .map(|n| n / 2.0)
            .unwrap_or(height * 0.32)
            .clamp(0.7, 4.5);
        writeln!(
            file,
            "t {} {:.3} {:.3} {:.3} {:.3}",
            node.id, p[0], p[1], height, radius
        )?;
        trees += 1;
    }
    file.flush()?;
    json(
        &output.join("landscape.json"),
        serde_json::json!({"compiler":"native-rust-v1","triangles":count,"trees":trees,"suppressed_conflicting_trees":suppressed,"incomplete_relations_skipped":incomplete,"area_m2_by_style":areas,"decisions":"Mapped areas layered below roads and buildings; trees retain mapped positions or are suppressed on conflicts."}),
    )?;
    println!("  Placing road controls...");
    furniture(source, map, s, city, output)
}
struct Feature {
    id: i64,
    kind: usize,
    position: P,
    travel: P,
    value: u16,
    phase: i64,
    estimated: bool,
    inferred: bool,
    width: f64,
    source: P,
}
fn directions(e: &Element, kind: usize) -> (Vec<i8>, bool, Option<P>) {
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
                .or_else(|| super::osm::number(value))
                .map(f64::to_radians)
            {
                (vec![], false, Some([-angle.sin(), angle.cos()]))
            } else {
                (vec![-1, 1], true, None)
            }
        }
    }
}

#[cfg(test)]
mod direction_tests {
    use super::*;
    #[test]
    fn compass_faces_opposite_travel_and_explicit_directions_take_precedence() {
        let mut e = Element::default();
        e.tags.insert("direction".into(), "N".into());
        let (d, inferred, travel) = directions(&e, 3);
        assert!(d.is_empty());
        assert!(!inferred);
        let p = travel.unwrap();
        assert!(p[0].abs() < 1e-10);
        assert!((p[1] - 1.).abs() < 1e-10);
        e.tags
            .insert("traffic_sign:direction".into(), "backward".into());
        assert_eq!(directions(&e, 3), (vec![-1], false, None));
        e.tags.clear();
        assert_eq!(directions(&e, 1), (vec![-1, 1], true, None));
    }
    #[test]
    fn relation_members_join_in_either_direction_and_reject_open_rings() {
        let joined = rings(vec![vec![1, 2, 3], vec![1, 4, 3]]).unwrap();
        assert_eq!(joined.len(), 1);
        assert_eq!(joined[0].first(), joined[0].last());
        assert_eq!(joined[0].len(), 5);
        assert!(rings(vec![vec![1, 2], vec![3, 4]]).is_none());
    }
}
fn furniture(source: &Source, map: &Map, s: &Surfaces, city: &str, output: &Path) -> Result<()> {
    let tree = super::surfaces::index(&s.paths);
    let obstacles = super::surfaces::PolygonIndex::new(&s.obstacles);
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
    let mut skipped = vec![];
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
                super::osm::number(e.tag("maxspeed")).filter(|v| *v >= 5.0 && *v <= 150.0)
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
        let before = features.len();
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
            let mut estimated = true;
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
                    estimated = false;
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
                estimated,
                inferred,
                width: r.width,
                source: p,
            });
        }
        features.extend(placements);
        if features.len() == before {
            skipped.push(e.id);
        }
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
    let mut file = BufWriter::new(fs::File::create(output.join("furniture.txt"))?);
    writeln!(
        file,
        "FURNITURE 1 {:08x} {:08x}",
        crate::street_surface::crc32(city),
        s.text_crc
    )?;
    for f in &unique {
        writeln!(
            file,
            "f {} {} {:.3} {:.3} {:.7} {:.7} {} {} {} {}",
            f.id,
            f.kind,
            f.position[0],
            f.position[1],
            f.travel[0],
            f.travel[1],
            f.value,
            f.phase,
            u8::from(f.estimated),
            u8::from(f.inferred)
        )?;
    }
    file.flush()?;
    json(
        &output.join("furniture.json"),
        serde_json::json!({"compiler":"native-rust-v1","features":unique.len(),"skipped_source_nodes":skipped,"signal_program":"Fictional 50-second alternating cycle; no real timing or traffic-rule enforcement."}),
    )
}
