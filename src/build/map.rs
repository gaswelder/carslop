//! Translate source OSM topology into local compiler geometry.

use crate::build;
use crate::model;
use crate::model::source_location::Sidewalks;
use crate::model::source_location::SourceLocation;
use crate::model::source_location::SourceRoad;
use crate::model::world::Building;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::add;
use crate::util::geometry::cross;
use crate::util::geometry::dist;
use crate::util::geometry::dot;
use crate::util::geometry::length;
use crate::util::geometry::line;
use crate::util::geometry::mul;
use crate::util::geometry::polygon;
use crate::util::geometry::sub;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XZ;
use geo::{BooleanOps, Contains, MultiLineString};
use std::collections::{BTreeMap, BTreeSet, HashMap};

// More specific side tags override sidewalk:both and the general sidewalk tag.
fn sidewalks(way: &build::osm::OSMElement) -> Sidewalks {
    let general = build::osm::tag(way, "sidewalk");
    let side = |key: &str, name: &str| {
        for value in [
            build::osm::tag(way, key),
            build::osm::tag(way, "sidewalk:both"),
        ] {
            match value {
                "yes" => return true,
                "no" | "none" | "separate" => return false,
                _ => {}
            }
        }
        match general {
            "both" | "yes" => true,
            "no" | "none" | "separate" => false,
            "left" | "right" => general == name,
            _ => true,
        }
    };
    Sidewalks {
        left: side("sidewalk:left", "left"),
        right: side("sidewalk:right", "right"),
    }
}
pub fn create_location_map(
    source: &build::osm::OSMMap,
    boundary: &build::osm::OSMArea,
) -> Result<SourceLocation<f64>> {
    let bbox = Some(util::geometry::bounding_box_to_array(build::osm::bbox(
        boundary,
    )));
    let bbox = bbox.or(source.bounds).unwrap_or_else(|| {
        let mut b = [90.0_f64, 180.0_f64, -90.0_f64, -180.0_f64];
        for e in source.nodes.values() {
            let p = e.coord.unwrap();
            b[0] = b[0].min(p.lat);
            b[1] = b[1].min(p.lon);
            b[2] = b[2].max(p.lat);
            b[3] = b[3].max(p.lon);
        }
        b
    });
    if bbox.iter().any(|v| !v.is_finite())
        || bbox[0] >= bbox[2]
        || bbox[1] >= bbox[3]
        || bbox[0] < -90.0
        || bbox[2] > 90.0
        || bbox[1] < -180.0
        || bbox[3] > 180.0
    {
        return Err(invalid("invalid bounding box (south west north east)"));
    }
    let boundary = Some(boundary);
    let origin = util::geometry::new((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0);
    if origin.lat.abs() > 85.0 {
        return Err(invalid(
            "locations within 5 degrees of the poles are unsupported",
        ));
    }
    let mut map = SourceLocation {
        origin: Some(origin),
        ..model::source_location::empty::<f64>()
    };
    let a = model::source_location::metres(&map, util::geometry::new(bbox[0], bbox[1]));
    let b = model::source_location::metres(&map, util::geometry::new(bbox[2], bbox[3]));
    map.bounding_box = BoundingBoxXZ {
        min: XZ { x: a.x, z: b.z },
        max: XZ { x: b.x, z: a.z },
    };
    map.boundary = boundary.map(|b| build::osm::local(b, origin));
    // Bound geometry and height-field allocations before processing an accidental country extract.
    if b.x - a.x > 80000.0 + 1e-6 || a.z - b.z > 80000.0 + 1e-6 {
        return Err(invalid(
            "crop exceeds 80 km; supply smaller width and height",
        ));
    }
    let coords: BTreeMap<_, _> = source
        .nodes
        .iter()
        .map(|(id, e)| (*id, model::source_location::metres(&map, e.coord.unwrap())))
        .collect();
    #[derive(Hash, PartialEq, Eq)]
    enum NodeKey {
        Mapped(i64),
        Clipped { x: i64, z: i64, grade: i8 },
    }
    let mut positions = HashMap::new();
    let major = [
        "motorway",
        "trunk",
        "motorway_link",
        "trunk_link",
        "primary",
        "secondary",
        "tertiary",
        "primary_link",
        "secondary_link",
        "tertiary_link",
    ];
    let mut ways: Vec<_> = source.ways.values().collect();
    // Put circuits first so a circuit location starts on its drivable track.
    ways.sort_by_key(|way| build::osm::tag(way, "highway") != "raceway");
    for way in ways {
        let kind = build::osm::tag(way, "highway");
        if !major.contains(&kind)
            && ![
                "unclassified",
                "residential",
                "living_street",
                "service",
                "raceway",
            ]
            .contains(&kind)
        {
            continue;
        }
        // Circuit access restrictions describe public traffic, not whether it
        // can be driven in the game. Keep those roads in offline locations.
        if (kind != "raceway"
            && ["access", "vehicle", "motor_vehicle"]
                .iter()
                .any(|k| matches!(build::osm::tag(way, k), "no" | "private")))
            || build::osm::tag(way, "area") == "yes"
        {
            continue;
        }
        let default = match kind {
            "motorway" => 12.0,
            "trunk" => 11.0,
            "primary" => 10.0,
            "secondary" => 9.0,
            "tertiary" => 8.0,
            "raceway" => 10.0,
            "service" | "living_street" => 5.5,
            _ => 6.5,
        };
        let width = build::osm::number(build::osm::tag(way, "width"))
            .unwrap_or_else(|| {
                let w = build::osm::number(build::osm::tag(way, "lanes"))
                    .filter(|n| *n > 0.0)
                    .map(|n| n * 3.1)
                    .unwrap_or(default);
                if major.contains(&kind) {
                    (w + 0.9).max(8.0)
                } else {
                    w
                }
            })
            .clamp(5.5, 18.0);
        let direction = if build::osm::tag(way, "oneway") == "-1" {
            -1
        } else if matches!(build::osm::tag(way, "oneway"), "yes" | "1" | "true")
            || build::osm::tag(way, "junction") == "roundabout"
        {
            1
        } else {
            0
        };
        for node_pair in way.nodes.windows(2) {
            let pair = node_pair;
            let (Some(a), Some(b)) = (coords.get(&pair[0]), coords.get(&pair[1])) else {
                continue;
            };
            let Some((a, b)) = build::osm::clip(*a, *b, map.bounding_box) else {
                continue;
            };
            let forward = sub(b, a);
            let pieces = map.boundary.as_ref().map_or_else(
                || vec![line(&[a, b])],
                |boundary| {
                    boundary
                        .clip(&MultiLineString(vec![line(&[a, b])]), false)
                        .0
                },
            );
            for piece in pieces {
                for pair in piece.0.windows(2) {
                    let (a, b) = (
                        XZ {
                            x: pair[0].x,
                            z: pair[0].y,
                        },
                        XZ {
                            x: pair[1].x,
                            z: pair[1].y,
                        },
                    );
                    // Preserve the original OSM direction after boolean line clipping.
                    let (a, b) = if dot(sub(b, a), forward) < 0.0 {
                        (b, a)
                    } else {
                        (a, b)
                    };
                    if dist(a, b) < 0.5 {
                        continue;
                    }
                    let mut ends = [0; 2];
                    for (i, p) in [a, b].iter().enumerate() {
                        let x = (p.x * 1000.).round() as i64;
                        let z = (p.z * 1000.).round() as i64;
                        let key = node_pair
                            .iter()
                            .find(|id| dist(coords[id], *p) < 0.002)
                            .map_or(
                                NodeKey::Clipped {
                                    x,
                                    z,
                                    grade: build::transport::grade(way),
                                },
                                |id| NodeKey::Mapped(*id),
                            );
                        ends[i] = *positions.entry(key).or_insert_with(|| {
                            let id = map.road_nodes.len();
                            map.road_nodes.push(util::geometry::at_height(
                                XZ {
                                    x: x as f64 / 1000.,
                                    z: z as f64 / 1000.,
                                },
                                0.,
                            ));
                            id
                        });
                    }
                    map.road_lines.push(SourceRoad {
                        direction,
                        sidewalks: sidewalks(way),
                        osm_way_id: Some(way.id),
                        kind: if major.contains(&kind) {
                            0
                        } else if matches!(kind, "service" | "living_street") {
                            2
                        } else {
                            1
                        },
                        lanes: (build::osm::number(build::osm::tag(way, "lanes"))
                            .unwrap_or(if direction == 0 { 2.0 } else { 1.0 })
                            as usize)
                            .clamp(1, 6),
                        ..model::source_location::new(ends, width)
                    });
                }
            }
        }
    }
    if map.road_lines.is_empty() {
        return Err(invalid("no public drivable roads in this crop"));
    }
    map.road_lines.sort_by(|a, b| {
        let key = |r: &SourceRoad<f64>| {
            let a = util::geometry::xz(map.road_nodes[r.ends[0]]);
            let b = util::geometry::xz(map.road_nodes[r.ends[1]]);
            (
                dist(a, b) < 25.0,
                length(mul(add(a, b), 0.5)),
                r.osm_way_id,
                r.ends[0],
            )
        };
        let a = key(a);
        let b = key(b);
        a.0.cmp(&b.0)
            .then(a.1.total_cmp(&b.1))
            .then(a.2.cmp(&b.2))
            .then(a.3.cmp(&b.3))
    });
    let complex: BTreeSet<_> = source
        .relations
        .iter()
        .filter(|e| {
            build::osm::tag(e, "type") == "multipolygon"
                && (!matches!(build::osm::tag(e, "building"), "" | "no")
                    || !matches!(build::osm::tag(e, "building:part"), "" | "no"))
        })
        .flat_map(|e| e.members.iter().filter(|m| m.0 == "way").map(|m| m.1))
        .collect();
    for way in source.ways.values() {
        if (matches!(build::osm::tag(way, "building"), "" | "no")
            && matches!(build::osm::tag(way, "building:part"), "" | "no"))
            || complex.contains(&way.id)
            || way.nodes.len() < 4
            || way.nodes.first() != way.nodes.last()
        {
            continue;
        }
        let Some(mut points) = way.nodes[..way.nodes.len() - 1]
            .iter()
            .map(|id| coords.get(id).copied())
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        let original = points.clone();
        points = original
            .iter()
            .enumerate()
            .filter(|(i, p)| dist(**p, original[(*i + original.len() - 1) % original.len()]) > 0.05)
            .map(|(_, p)| *p)
            .collect();
        if points.len() < 3
            || points
                .iter()
                .any(|p| !model::source_location::contains(&map, *p))
        {
            continue;
        }
        if map
            .boundary
            .as_ref()
            .is_some_and(|b| !b.contains(&polygon(&points)))
        {
            continue;
        }
        let area = points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .take(points.len())
            .map(|(a, b)| cross(*a, *b))
            .sum::<f64>()
            / 2.0;
        if area.abs() < 4.0 {
            continue;
        }
        if area < 0.0 {
            points.reverse();
        }
        let height =
            build::osm::number(build::osm::tag(way, "height")).filter(|height| *height > 0.0);
        let min_height = build::osm::number(build::osm::tag(way, "min_height"))
            .or_else(|| {
                build::osm::number(build::osm::tag(way, "building:min_level"))
                    .map(|levels| levels * 3.1)
            })
            .unwrap_or(0.0)
            .clamp(0.0, 69.0);
        if height.is_some_and(|height| height <= min_height) {
            continue;
        }
        map.buildings.push(Building {
            id: way.id as u64,
            height_meters: height
                .unwrap_or_else(|| {
                    build::osm::number(build::osm::tag(way, "building:levels"))
                        .filter(|n| *n > 0.0)
                        .map(|n| n * 3.1 + build::osm::roof_height(way))
                        .unwrap_or(
                            if matches!(
                                build::osm::tag(way, "building"),
                                "apartments" | "commercial" | "office"
                            ) {
                                9.3
                            } else {
                                6.2
                            },
                        )
                })
                .clamp(2.5, 70.0)
                .max(min_height + 0.1),
            tagged: height.is_some(),
            holes: Vec::new(),
            min_height_meters: min_height,
            base_y: 0.0,
            ground_outline: points,
        });
        for node_id in way.nodes.iter().copied().collect::<BTreeSet<_>>() {
            let Some(node) = source.nodes.get(&node_id) else {
                continue;
            };
            if matches!(build::osm::tag(node, "entrance"), "" | "no") {
                continue;
            }
            if let Some(coord) = node.coord {
                map.entrances.push(model::world::Entrance {
                    building_id: way.id as u64,
                    position: model::source_location::metres(&map, coord),
                });
            }
        }
    }
    build::buildings::import_relations(source, &mut map);
    build::buildings::resolve_parts(source, &mut map);
    build::buildings::open_passages(source, &mut map);
    Ok(map)
}
