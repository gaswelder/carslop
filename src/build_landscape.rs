//! Compile mapped ground cover and trees into landscape assets.
use crate::asset_codecs;
use crate::build_surfaces;
use crate::build_surfaces::{union, Surfaces};
use crate::error::{invalid, Result};
use crate::geometry::XZ;
use crate::geometry::{empty, line, polygon};
use crate::osm;
use crate::osm::{OSMElement, OSMMap};
use crate::source_location::SourceLocation;
use crate::world;
use geo::algorithm::buffer;
use geo::MultiPolygon;
use geo::{algorithm::orient::Direction, BooleanOps, Buffer, Orient, Point};
use std::fs;
use std::io::BufWriter;
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
        Some(path_style(surface))
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
fn path_style(surface: &str) -> usize {
    match surface {
        "asphalt" => 3,
        "concrete" => 5,
        "gravel" | "dirt" | "ground" | "unpaved" | "compacted" => 6,
        _ => 4,
    }
}

// Closed highway ways are still paths unless explicitly mapped as areas.
fn path_width(way: &OSMElement) -> Option<f64> {
    let default = match way.tag("highway") {
        "footway" => 1.8,
        "path" => 1.2,
        "pedestrian" => 3.0,
        _ => return None,
    };
    if way.tag("area") == "yes" || !way.tag("area:highway").is_empty() {
        return None;
    }
    Some(
        osm::number(way.tag("width"))
            .filter(|width| *width > 0.0)
            .unwrap_or(default)
            .clamp(0.4, 12.0),
    )
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
    map: &SourceLocation<f64>,
    s: &Surfaces,
    landscape_path: &Path,
) -> Result<()> {
    let coord = |id: &i64| source.nodes.get(id).map(|e| map.metres(e.coord.unwrap()));
    let mut groups: Vec<Vec<MultiPolygon<f64>>> = vec![vec![]; 9];
    let mut consumed = std::collections::BTreeSet::new();
    let mut paths: Vec<Vec<MultiPolygon<f64>>> = vec![vec![]; 9];
    let obstacles = build_surfaces::PolygonIndex::new(&s.obstacles);
    let crop = map.boundary.clone().unwrap_or_else(|| {
        MultiPolygon(vec![polygon(&[
            XZ {
                x: map.bounding_box.min.x,
                z: map.bounding_box.min.z,
            },
            XZ {
                x: map.bounding_box.max.x,
                z: map.bounding_box.min.z,
            },
            XZ {
                x: map.bounding_box.max.x,
                z: map.bounding_box.max.z,
            },
            XZ {
                x: map.bounding_box.min.x,
                z: map.bounding_box.max.z,
            },
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
        if consumed.contains(&way.id) {
            continue;
        }
        if matches!(way.tag("highway"), "footway" | "path" | "pedestrian")
            && (["bridge", "tunnel"]
                .iter()
                .any(|key| !matches!(way.tag(key), "" | "no"))
                || !matches!(way.tag("layer"), "" | "0"))
        {
            continue;
        }
        let Some(mut points) = way.nodes.iter().map(&coord).collect::<Option<Vec<_>>>() else {
            continue;
        };
        if let Some(width) = path_width(way) {
            points.dedup();
            if points.len() < 2 {
                continue;
            }
            let geometry = line(&points)
                .buffer_with_style(
                    buffer::BufferStyle::new(width / 2.0)
                        .line_cap(buffer::LineCap::Butt)
                        .line_join(buffer::LineJoin::Round(std::f64::consts::PI / 16.0)),
                )
                .intersection(&crop);
            let geometry = geometry.difference(&obstacles.nearby(&geometry));
            paths[path_style(way.tag("surface"))].push(geometry);
        } else if points.len() >= 4 && way.nodes.first() == way.nodes.last() {
            groups[kind].push(polygon(&points).intersection(&crop));
        }
    }
    let mut file = asset_codecs::landscape::Writer::new(
        BufWriter::new(fs::File::create(landscape_path)?),
        asset_codecs::landscape::Precision::Imported,
    )?;
    let mut covered = empty();
    let mut emit = |kind: usize, regions: &[MultiPolygon<f64>]| -> Result<()> {
        let geometry = union(regions).difference(&covered);
        covered = covered.union(&geometry);
        let material = world::Material::from_ground_id(kind).map_err(invalid)?;
        build_surfaces::triangulate(&geometry, |points| {
            file.ground(material, points)?;
            Ok(())
        })?;
        Ok(())
    };
    emit(8, &groups[8])?;
    // Paths cut through surrounding park/grass/dirt cover, including unpaved paths.
    for kind in [3, 4, 5, 6] {
        emit(kind, &paths[kind])?;
    }
    for kind in [7, 3, 4, 5, 2, 0, 1, 6] {
        emit(kind, &groups[kind])?;
    }
    println!("  Filtering mapped trees...");
    // Keep small pieces in the index so connected paths do not create a city-wide probe.
    let path_geometry = MultiPolygon(
        paths
            .into_iter()
            .flatten()
            .flat_map(|geometry| geometry.0)
            .collect(),
    );
    let path_obstacles = build_surfaces::PolygonIndex::new(&path_geometry);
    let conflicts =
        |probe: &MultiPolygon<f64>| obstacles.intersects(probe) || path_obstacles.intersects(probe);
    for node in source.nodes.values().filter(|n| n.tag("natural") == "tree") {
        let p = map.metres(node.coord.unwrap());
        if !map.contains(p) {
            continue;
        }
        if conflicts(&Point::new(p.x, p.z).buffer(0.35)) {
            continue;
        }
        let height = osm::number(node.tag("height"))
            .filter(|n| *n > 0.0)
            .unwrap_or(4.8 + node.id.rem_euclid(7) as f64 * 0.3)
            .clamp(2.5, 18.0);
        let radius = osm::number(node.tag("diameter_crown"))
            .filter(|n| *n > 0.0)
            .map(|n| n / 2.0)
            .unwrap_or(height * 0.32)
            .clamp(0.7, 4.5);
        file.tree_fields(node.id, p, height, radius)?;
    }
    file.flush()?;
    Ok(())
}
