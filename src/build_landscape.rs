//! Compile mapped ground cover and trees into landscape assets.
use crate::build_surfaces::{union, Surfaces};
use crate::location::{empty, polygon, Result};
use crate::osm::{OSMElement, OSMMap};
use crate::world::LocationMap;
use geo::MultiPolygon;
use geo::{algorithm::orient::Direction, BooleanOps, Buffer, Orient, Point};
use std::fs;
use std::io::{BufWriter, Write};
use std::path::Path;

fn style(e: &OSMElement) -> Option<usize> {
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
pub fn rings(mut parts: Vec<Vec<i64>>) -> Option<Vec<Vec<i64>>> {
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
pub fn build_landscape(
    source: &OSMMap,
    map: &LocationMap,
    s: &Surfaces,
    city_crc: u32,
    landscape_path: &Path,
) -> Result<()> {
    let coord = |id: &i64| source.nodes.get(id).map(|e| map.metres(e.coord.unwrap()));
    let mut groups: Vec<Vec<MultiPolygon<f64>>> = vec![vec![]; 9];
    let mut consumed = std::collections::BTreeSet::new();
    let crop = map.boundary.clone().unwrap_or_else(|| {
        MultiPolygon(vec![polygon(&[
            [map.bounds[0], map.bounds[1]],
            [map.bounds[2], map.bounds[1]],
            [map.bounds[2], map.bounds[3]],
            [map.bounds[0], map.bounds[3]],
        ])])
    });
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
    let mut file = BufWriter::new(fs::File::create(landscape_path)?);
    writeln!(file, "LANDSCAPE 1 {city_crc:08x} {:08x}", s.text_crc)?;
    let mut covered = empty();
    for kind in [8, 7, 3, 4, 5, 2, 0, 1, 6] {
        let geometry = union(&groups[kind]).difference(&covered);
        covered = covered.union(&geometry);
        crate::build_surfaces::triangles(&mut file, &format!("a {kind}"), &geometry)?;
    }
    println!("  Filtering mapped trees...");
    let obstacles = crate::build_surfaces::PolygonIndex::new(&s.obstacles);
    let conflicts = |probe: &MultiPolygon<f64>| obstacles.intersects(probe);
    for node in source.nodes.values().filter(|n| n.tag("natural") == "tree") {
        let p = map.metres(node.coord.unwrap());
        if !map.contains(p) {
            continue;
        }
        if conflicts(&Point::new(p[0], p[1]).buffer(0.35)) {
            continue;
        }
        let height = crate::osm::number(node.tag("height"))
            .filter(|n| *n > 0.0)
            .unwrap_or(4.8 + node.id.rem_euclid(7) as f64 * 0.3)
            .clamp(2.5, 18.0);
        let radius = crate::osm::number(node.tag("diameter_crown"))
            .filter(|n| *n > 0.0)
            .map(|n| n / 2.0)
            .unwrap_or(height * 0.32)
            .clamp(0.7, 4.5);
        writeln!(
            file,
            "t {} {:.3} {:.3} {:.3} {:.3}",
            node.id, p[0], p[1], height, radius
        )?;
    }
    file.flush()?;
    Ok(())
}
