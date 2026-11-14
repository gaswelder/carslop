//! Paint mapped pedestrian crossings into the compiled street marking layer.
use crate::build_surfaces;
use crate::geometry;
use crate::geometry::XZ;
use crate::osm;
use crate::source_location::SourceLocation;
use geo::{BooleanOps, Buffer, MultiPolygon};
use std::collections::BTreeMap;

#[derive(Clone, Copy)]
enum Style {
    Zebra,
    Lines,
    Dashes,
}

struct Walking<'a> {
    way: &'a osm::OSMElement,
    direction: XZ,
}

pub struct Crossings {
    pub paint: MultiPolygon<f64>,
    pub clearance: MultiPolygon<f64>,
}

fn markings(value: &str) -> Option<Style> {
    // Unknown patterns are left unpainted instead of being guessed as zebra stripes.
    if value.split(';').any(|part| part == "zebra") || value == "yes" {
        Some(Style::Zebra)
    } else if value.split(';').any(|part| part == "lines") {
        Some(Style::Lines)
    } else if value == "dashes" {
        Some(Style::Dashes)
    } else {
        None
    }
}

fn style(node: &osm::OSMElement, walking: Option<&Walking<'_>>) -> Option<Style> {
    let explicit = node.tag("crossing:markings");
    if !explicit.is_empty() && explicit != "yes" {
        return markings(explicit);
    }
    if explicit.is_empty() && matches!(node.tag("crossing"), "unmarked" | "informal" | "no") {
        return None;
    }
    if let Some(walking) = walking {
        let value = walking.way.tag("crossing:markings");
        if !value.is_empty() {
            return markings(value);
        }
    }
    if explicit == "yes"
        || node.tag("crossing_ref") == "zebra"
        || matches!(node.tag("crossing"), "zebra" | "marked" | "uncontrolled")
        || walking.is_some_and(|walking| {
            matches!(
                walking.way.tag("crossing"),
                "zebra" | "marked" | "uncontrolled"
            )
        })
    {
        Some(Style::Zebra)
    } else {
        None
    }
}

fn rectangle(center: XZ, across: XZ, lo: f64, hi: f64, near: f64, far: f64) -> MultiPolygon<f64> {
    let forward = XZ {
        x: -across.z,
        z: across.x,
    };
    let point = |x, y| {
        geometry::add(
            center,
            geometry::add(geometry::mul(across, x), geometry::mul(forward, y)),
        )
    };
    MultiPolygon(vec![geometry::polygon(&[
        point(lo, near),
        point(hi, near),
        point(hi, far),
        point(lo, far),
    ])])
}

fn ground_level(element: &osm::OSMElement) -> bool {
    matches!(element.tag("layer"), "" | "0")
        && ["bridge", "tunnel"]
            .iter()
            .all(|key| matches!(element.tag(key), "" | "no"))
}

pub fn build_crossings(
    source: &osm::OSMMap,
    map: &SourceLocation<f64>,
    paths: &[Vec<XZ>],
    asphalt: &MultiPolygon<f64>,
) -> Crossings {
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
    let mut walking = BTreeMap::new();
    for way in source.ways.values().filter(|way| {
        ground_level(way) && (way.tag("footway") == "crossing" || way.tag("path") == "crossing")
    }) {
        for (i, id) in way.nodes.iter().enumerate() {
            let before = way.nodes[i.saturating_sub(1)];
            let after = way.nodes[(i + 1).min(way.nodes.len() - 1)];
            let Some(a) = source.nodes.get(&before).and_then(|node| node.coord) else {
                continue;
            };
            let Some(b) = source.nodes.get(&after).and_then(|node| node.coord) else {
                continue;
            };
            let direction = geometry::sub(map.metres(b), map.metres(a));
            if geometry::length(direction) > 0.1 {
                walking.entry(*id).or_insert(Walking {
                    way,
                    direction: geometry::unit(direction),
                });
            }
        }
    }
    let mut paint = Vec::new();
    let mut clearance = Vec::new();
    for (id, way_ids) in hosts {
        let Some(node) = source.nodes.get(&id) else {
            continue;
        };
        let walk = walking.get(&id);
        if (node.tag("highway") != "crossing" && walk.is_none()) || !ground_level(node) {
            continue;
        }
        let Some(style) = style(node, walk) else {
            continue;
        };
        let Some(coord) = node.coord else {
            continue;
        };
        let position = map.metres(coord);
        if !map.contains(position) {
            continue;
        }
        let width = osm::number(node.tag("width"))
            .filter(|width| *width > 0.)
            .or_else(|| {
                walk.and_then(|walk| osm::number(walk.way.tag("width")))
                    .filter(|width| *width > 0.)
            })
            .unwrap_or(3.0)
            .clamp(1.0, 8.0);
        for way_id in way_ids {
            let Some(&road_index) = roads[&way_id].iter().min_by(|a, b| {
                geometry::project(&paths[**a], position)
                    .0
                    .total_cmp(&geometry::project(&paths[**b], position).0)
            }) else {
                continue;
            };
            let path = &paths[road_index];
            let (distance, station) = geometry::project(path, position);
            // Use connected source ways only; nearby bridges/parallel roads are not hosts.
            if distance > 3.0 {
                continue;
            }
            let length = geometry::path_length(path);
            let tangent = geometry::unit(geometry::sub(
                geometry::along(path, (station + 0.5).min(length)),
                geometry::along(path, (station - 0.5).max(0.0)),
            ));
            let across = if let Some(walk) = walk {
                // A crossing path running along a road does not cross that road.
                if geometry::dot(walk.direction, tangent).abs() >= 0.85 {
                    continue;
                }
                walk.direction
            } else {
                XZ {
                    x: -tangent.z,
                    z: tangent.x,
                }
            };
            let half = (map.road_lines[road_index].road_width_start_meters / 2.0 + 0.3)
                / geometry::cross(tangent, across).abs().max(0.5);
            let center = geometry::along(path, station);
            clearance.push(rectangle(
                center,
                across,
                -half,
                half,
                -width / 2.0 - 0.4,
                width / 2.0 + 0.4,
            ));
            match style {
                Style::Zebra => {
                    let count = (half / 0.9).ceil() as i32;
                    for stripe in -count..=count {
                        let lo = (stripe as f64 * 0.9 - 0.25).max(-half);
                        let hi = (stripe as f64 * 0.9 + 0.25).min(half);
                        if hi > lo {
                            paint.push(rectangle(
                                center,
                                across,
                                lo,
                                hi,
                                -width / 2.0,
                                width / 2.0,
                            ));
                        }
                    }
                }
                Style::Lines | Style::Dashes => {
                    for side in [-1.0, 1.0] {
                        let near = side * (width / 2.0 - 0.1);
                        if matches!(style, Style::Lines) {
                            paint.push(rectangle(
                                center,
                                across,
                                -half,
                                half,
                                near - 0.1,
                                near + 0.1,
                            ));
                        } else {
                            let count = (half / 1.0).ceil() as i32;
                            for dash in -count..=count {
                                let lo = (dash as f64 - 0.25).max(-half);
                                let hi = (dash as f64 + 0.25).min(half);
                                if hi > lo {
                                    paint.push(rectangle(
                                        center,
                                        across,
                                        lo,
                                        hi,
                                        near - 0.1,
                                        near + 0.1,
                                    ));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    Crossings {
        paint: build_surfaces::union(&paint).intersection(asphalt),
        clearance: build_surfaces::union(&clearance).buffer(0.05),
    }
}
