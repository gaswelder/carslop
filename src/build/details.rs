//! Reduce mapped appearance and street objects to source-independent playable records.

use crate::build;
use crate::model;
use crate::model::map_details::BuildingStyle;
use crate::model::map_details::Data;
use crate::model::map_details::LineKind;
use crate::model::map_details::MapObject;
use crate::model::map_details::ObjectKind;
use crate::model::map_details::RoadSurface;
use crate::model::map_details::RoofShape;
use crate::model::map_details::SceneryLine;
use crate::model::map_details::TreeKind;
use crate::model::map_details::TreeStyle;
use crate::model::map_details::TurnRestriction;
use crate::model::source_location::SourceLocation;
use crate::model::world::Building;
use crate::util;
use crate::util::geometry::XZ;
use geo::{Area, BoundingRect, Contains, Point, Polygon, Rect};
use std::collections::BTreeMap;

struct ObjectDimensions {
    height: f64,
    width: f64,
}

pub fn road_surface(value: &str) -> RoadSurface {
    match value {
        "concrete" | "concrete:plates" | "concrete:lanes" => RoadSurface::Concrete,
        "paving_stones" | "sett" | "cobblestone" | "unhewn_cobblestone" => RoadSurface::Paving,
        "gravel" | "fine_gravel" | "dirt" | "earth" | "ground" | "unpaved" | "sand" | "mud"
        | "compacted" => RoadSurface::Unpaved,
        _ => RoadSurface::Asphalt,
    }
}
fn color(value: &str) -> Option<[f32; 3]> {
    let value = value.trim().to_ascii_lowercase();
    if let Some(hex) = value.strip_prefix('#') {
        let hex = if hex.len() == 3 {
            hex.chars().flat_map(|c| [c, c]).collect::<String>()
        } else {
            hex.to_owned()
        };
        if hex.len() == 6 {
            let value = u32::from_str_radix(&hex, 16).ok()?;
            return Some([
                ((value >> 16) & 255) as f32 / 255.,
                ((value >> 8) & 255) as f32 / 255.,
                (value & 255) as f32 / 255.,
            ]);
        }
    }
    Some(match value.as_str() {
        "white" => [0.95, 0.95, 0.92],
        "black" => [0.08, 0.09, 0.09],
        "grey" | "gray" | "concrete" | "metal" | "metal_sheet" | "zinc" => [0.57, 0.59, 0.58],
        "red" | "roof_tiles" | "tiles" | "tile" | "terracotta" => [0.61, 0.25, 0.16],
        "brick" | "brown" => [0.49, 0.32, 0.23],
        "wood" | "timber" => [0.46, 0.36, 0.24],
        "stone" | "limestone" | "beige" | "sandstone" => [0.78, 0.73, 0.61],
        "plaster" | "stucco" => [0.89, 0.86, 0.77],
        "yellow" => [0.88, 0.77, 0.42],
        "green" | "copper" => [0.27, 0.45, 0.33],
        "blue" => [0.25, 0.39, 0.55],
        "slate" | "asphalt" => [0.32, 0.35, 0.36],
        "glass" => [0.39, 0.52, 0.56],
        _ => return None,
    })
}
fn direction(value: &str) -> Option<f32> {
    let angle = match value {
        "N" => 0.,
        "NE" => 45.,
        "E" => 90.,
        "SE" => 135.,
        "S" => 180.,
        "SW" => 225.,
        "W" => 270.,
        "NW" => 315.,
        _ => build::osm::number(value)? as f32,
    };
    Some(angle.to_radians())
}
fn style(way: &build::osm::OSMElement, volume: usize, building: &Building<f64>) -> BuildingStyle {
    let shape = match build::osm::tag(way, "roof:shape") {
        "gabled" => RoofShape::Gabled,
        "hipped" => RoofShape::Hipped,
        "pyramidal" | "pyramid" => RoofShape::Pyramidal,
        "skillion" => RoofShape::Skillion,
        "dome" => RoofShape::Dome,
        _ => RoofShape::Flat,
    };
    let edge = building
        .ground_outline
        .iter()
        .zip(building.ground_outline.iter().cycle().skip(1))
        .take(building.ground_outline.len())
        .max_by(|(a, b), (c, d)| {
            util::geometry::dist(**a, **b).total_cmp(&util::geometry::dist(**c, **d))
        });
    let mut angle = edge.map_or(0., |(a, b)| (b.z - a.z).atan2(b.x - a.x) as f32);
    if build::osm::tag(way, "roof:orientation") == "across" {
        angle += std::f32::consts::FRAC_PI_2;
    }
    if let Some(direction) = direction(build::osm::tag(way, "roof:direction")) {
        angle = std::f32::consts::PI - direction;
    }
    let roof_height = if shape == RoofShape::Flat {
        0.
    } else {
        build::osm::number(build::osm::tag(way, "roof:height"))
            .or_else(|| {
                build::osm::number(build::osm::tag(way, "roof:levels")).map(|levels| levels * 3.1)
            })
            .unwrap_or(2.0)
            .clamp(0.2, 30.0)
            .min((building.height_meters - building.min_height_meters - 0.3).max(0.0))
            as f32
    };
    BuildingStyle {
        volume,
        shape,
        roof_height,
        angle,
        wall_color: color(build::osm::tag(way, "building:colour"))
            .or_else(|| color(build::osm::tag(way, "building:material"))),
        roof_color: color(build::osm::tag(way, "roof:colour"))
            .or_else(|| color(build::osm::tag(way, "roof:material"))),
    }
}
struct StyledPart<'a> {
    way: &'a build::osm::OSMElement,
    footprint: Polygon<f64>,
    bounds: Rect<f64>,
}
pub fn tree_style(id: u64, element: &build::osm::OSMElement) -> TreeStyle {
    let species = format!(
        "{} {} {} {}",
        build::osm::tag(element, "species"),
        build::osm::tag(element, "genus"),
        build::osm::tag(element, "leaf_type"),
        build::osm::tag(element, "type")
    )
    .to_ascii_lowercase();
    let kind = if species.contains("palm")
        || species.contains("phoenix")
        || species.contains("washingtonia")
    {
        TreeKind::Palm
    } else if species.contains("cupressus") || species.contains("cypress") {
        TreeKind::Cypress
    } else if species.contains("olea") || species.contains("olive") {
        TreeKind::Olive
    } else if species.contains("needleleaved")
        || species.contains("pinus")
        || species.contains("conifer")
    {
        TreeKind::Conifer
    } else {
        TreeKind::Broadleaf
    };
    TreeStyle {
        id,
        kind,
        evergreen: build::osm::tag(element, "leaf_cycle") == "evergreen",
    }
}

pub fn build(
    source: &build::osm::OSMMap,
    map: &SourceLocation<f64>,
    surfaces: &build::surfaces::Surfaces,
) -> Data {
    let mut data = Data::default();
    let mut parts: Vec<_> = source
        .ways
        .values()
        .filter(|way| !matches!(build::osm::tag(way, "building:part"), "" | "no"))
        .filter_map(|way| {
            if way.nodes.len() < 4 || way.nodes.first() != way.nodes.last() {
                return None;
            }
            let points: Option<Vec<_>> = way
                .nodes
                .iter()
                .map(|id| {
                    source
                        .nodes
                        .get(id)
                        .and_then(|node| node.coord)
                        .map(|p| model::source_location::metres(map, p))
                })
                .collect();
            {
                let footprint = util::geometry::polygon(&points?);
                let bounds = footprint.bounding_rect()?;
                Some(StyledPart {
                    way,
                    footprint,
                    bounds,
                })
            }
        })
        .collect();
    for relation in source
        .relations
        .iter()
        .filter(|relation| !matches!(build::osm::tag(relation, "building:part"), "" | "no"))
    {
        if let Some(geometry) = build::buildings::relation_geometry(source, relation, |p| {
            model::source_location::metres(map, p)
        }) {
            for footprint in geometry.0 {
                if let Some(bounds) = footprint.bounding_rect() {
                    parts.push(StyledPart {
                        way: relation,
                        footprint,
                        bounds,
                    });
                }
            }
        }
    }
    for (volume, building) in map.buildings.iter().enumerate() {
        let parent = build::osm::building_source(source, building.id);
        let footprint = model::world::footprint(building);
        let bounds = footprint.bounding_rect().expect("valid building footprint");
        let child = parts
            .iter()
            .filter(|part| {
                part.bounds.min().x <= bounds.min().x
                    && part.bounds.min().y <= bounds.min().y
                    && part.bounds.max().x >= bounds.max().x
                    && part.bounds.max().y >= bounds.max().y
                    && part.footprint.contains(&footprint)
            })
            .min_by(|a, b| {
                a.footprint
                    .unsigned_area()
                    .total_cmp(&b.footprint.unsigned_area())
            });
        let Some(way) = child.map(|part| part.way).or(parent) else {
            continue;
        };
        let mut appearance = style(way, volume, building);
        if let Some(parent) = parent {
            let inherited = style(parent, volume, building);
            appearance.wall_color = appearance.wall_color.or(inherited.wall_color);
            appearance.roof_color = appearance.roof_color.or(inherited.roof_color);
        }
        if appearance.shape != RoofShape::Flat
            || appearance.wall_color.is_some()
            || appearance.roof_color.is_some()
        {
            data.buildings.push(appearance);
        }
    }
    let obstacles = build::surfaces::new(&surfaces.obstacles);
    let road_index = build::surfaces::index(&surfaces.paths);
    let mut gate_angles = BTreeMap::new();
    for way in source
        .ways
        .values()
        .filter(|way| !build::osm::tag(way, "barrier").is_empty())
    {
        for pair in way.nodes.windows(2) {
            let (Some(a), Some(b)) = (
                source.nodes.get(&pair[0]).and_then(|n| n.coord),
                source.nodes.get(&pair[1]).and_then(|n| n.coord),
            ) else {
                continue;
            };
            let a = model::source_location::metres(map, a);
            let b = model::source_location::metres(map, b);
            let angle = (b.z - a.z).atan2(b.x - a.x) as f32;
            gate_angles.entry(pair[0]).or_insert(angle);
            gate_angles.entry(pair[1]).or_insert(angle);
        }
    }
    for node in source.nodes.values() {
        let Some(coord) = node.coord else {
            continue;
        };
        let p = model::source_location::metres(map, coord);
        if !model::source_location::contains(map, p) {
            continue;
        }
        if build::osm::tag(node, "indoor") == "yes"
            || !matches!(build::osm::tag(node, "level"), "" | "0")
            || matches!(
                build::osm::tag(node, "barrier"),
                "gate" | "lift_gate" | "swing_gate"
            )
        {
            continue;
        }
        let kind = match build::osm::tag(node, "amenity") {
            "bench" => Some(ObjectKind::Bench),
            "waste_basket" | "waste_disposal" => Some(ObjectKind::Bin),
            "shelter" => Some(ObjectKind::Shelter),
            _ => None,
        }
        .or_else(|| match build::osm::tag(node, "highway") {
            "street_lamp" => Some(ObjectKind::Lamp),
            "bus_stop" if build::osm::tag(node, "shelter") == "yes" => Some(ObjectKind::Shelter),
            _ => None,
        })
        .or_else(|| match build::osm::tag(node, "barrier") {
            "bollard" | "block" => Some(ObjectKind::Bollard),
            _ => None,
        })
        .or_else(|| match build::osm::tag(node, "power") {
            "pole" => Some(ObjectKind::PowerPole),
            "tower" => Some(ObjectKind::PowerTower),
            _ => None,
        })
        .or_else(|| match build::osm::tag(node, "historic") {
            "monument" | "memorial" | "wayside_cross" => Some(ObjectKind::Monument),
            _ => None,
        })
        .or_else(|| {
            (build::osm::tag(node, "man_made") == "planter").then_some(ObjectKind::Planter)
        });
        let Some(kind) = kind else {
            continue;
        };
        if kind != ObjectKind::Bollard
            && build::surfaces::intersects(
                &obstacles,
                &geo::Buffer::buffer(&Point::new(p.x, p.z), 0.08),
            )
        {
            continue;
        }
        let defaults = match kind {
            ObjectKind::Bench => ObjectDimensions {
                height: 0.9,
                width: 1.8,
            },
            ObjectKind::Bin => ObjectDimensions {
                height: 0.8,
                width: 0.45,
            },
            ObjectKind::Lamp => ObjectDimensions {
                height: 5.0,
                width: 0.1,
            },
            ObjectKind::Shelter => ObjectDimensions {
                height: 2.5,
                width: 3.0,
            },
            ObjectKind::Bollard if build::osm::tag(node, "barrier") == "block" => {
                ObjectDimensions {
                    height: 0.8,
                    width: 1.0,
                }
            }
            ObjectKind::Bollard => ObjectDimensions {
                height: 0.9,
                width: 0.18,
            },
            ObjectKind::Gate => continue,
            ObjectKind::PowerPole => ObjectDimensions {
                height: 9.0,
                width: 0.2,
            },
            ObjectKind::PowerTower => ObjectDimensions {
                height: 24.0,
                width: 4.0,
            },
            ObjectKind::Monument => ObjectDimensions {
                height: 2.5,
                width: 1.0,
            },
            ObjectKind::Planter => ObjectDimensions {
                height: 0.6,
                width: 1.0,
            },
            ObjectKind::TurnSign => ObjectDimensions {
                height: 2.1,
                width: 0.6,
            },
        };
        let angle = direction(build::osm::tag(node, "direction"))
            .map(|bearing| -bearing)
            .or_else(|| gate_angles.get(&node.id).copied())
            .unwrap_or_else(|| {
                road_index.nearest_neighbor(&p).map_or(0.0, |item| {
                    let path = &surfaces.paths[item.road_index];
                    let (_, station) = util::geometry::project(path, p);
                    let travel = util::geometry::sub(
                        util::geometry::along(path, station + 0.1),
                        util::geometry::along(path, (station - 0.1).max(0.0)),
                    );
                    travel.z.atan2(travel.x) as f32
                })
            });
        data.objects.push(MapObject {
            kind,
            position: [p.x as f32, p.z as f32],
            angle,
            height: build::osm::number(build::osm::tag(node, "height"))
                .filter(|v| *v > 0.)
                .unwrap_or(defaults.height)
                .clamp(0.1, 100.0) as f32,
            width: build::osm::number(build::osm::tag(node, "width"))
                .filter(|v| *v > 0.)
                .unwrap_or(defaults.width)
                .clamp(0.02, 30.0) as f32,
            text: build::osm::tag(node, "name").chars().take(128).collect(),
            locked: build::osm::tag(node, "locked") == "yes",
        });
    }
    for way in source.ways.values() {
        let kind = if matches!(build::osm::tag(way, "power"), "line" | "minor_line") {
            Some(LineKind::Power)
        } else if build::osm::tag(way, "natural") == "cliff" {
            Some(LineKind::Cliff)
        } else {
            None
        };
        let Some(kind) = kind else {
            continue;
        };
        let Some(points) = way
            .nodes
            .iter()
            .map(|id| {
                source
                    .nodes
                    .get(id)
                    .and_then(|node| node.coord)
                    .map(|p| model::source_location::metres(map, p))
            })
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        for pair in points.windows(2) {
            let Some((a, b)) = build::osm::clip(pair[0], pair[1], map.bounding_box) else {
                continue;
            };
            if !model::source_location::contains(map, a)
                || !model::source_location::contains(map, b)
            {
                continue;
            }
            data.lines.push(SceneryLine {
                kind,
                points: vec![[a.x as f32, a.z as f32], [b.x as f32, b.z as f32]],
                height: build::osm::number(build::osm::tag(way, "height"))
                    .unwrap_or(if kind == LineKind::Power { 10.0 } else { 3.0 })
                    .clamp(0.1, 100.0) as f32,
            });
        }
    }
    for relation in source
        .relations
        .iter()
        .filter(|relation| build::osm::tag(relation, "type") == "restriction")
    {
        if build::osm::tag(relation, "except")
            .split(';')
            .any(|value| matches!(value, "motorcar" | "motor_vehicle"))
        {
            continue;
        }
        let from = relation
            .members
            .iter()
            .find(|m| m.0 == "way" && m.2 == "from")
            .map(|m| m.1);
        let to = relation
            .members
            .iter()
            .find(|m| m.0 == "way" && m.2 == "to")
            .map(|m| m.1);
        let via = relation
            .members
            .iter()
            .find(|m| m.0 == "node" && m.2 == "via")
            .map(|m| m.1);
        let (Some(from), Some(to), Some(via)) = (from, to, via) else {
            continue;
        };
        let rule = if build::osm::tag(relation, "restriction:motorcar").is_empty() {
            build::osm::tag(relation, "restriction")
        } else {
            build::osm::tag(relation, "restriction:motorcar")
        };
        if rule.is_empty() {
            continue;
        }
        data.restrictions.push(TurnRestriction {
            from,
            via,
            to,
            rule: rule.to_owned(),
        });
        let Some(node) = source.nodes.get(&via).and_then(|node| node.coord) else {
            continue;
        };
        let p = model::source_location::metres(map, node);
        let Some(road) = map
            .road_lines
            .iter()
            .find(|road| road.osm_way_id == Some(from))
        else {
            continue;
        };
        let Some(way) = source.ways.get(&from) else {
            continue;
        };
        let other = if way.nodes.last() == Some(&via) {
            way.nodes.iter().rev().nth(1)
        } else {
            way.nodes.get(1)
        };
        let Some(a) = other
            .and_then(|id| source.nodes.get(id))
            .and_then(|node| node.coord)
            .map(|p| model::source_location::metres(map, p))
        else {
            continue;
        };
        let travel = util::geometry::unit(util::geometry::sub(p, a));
        let left = XZ {
            x: travel.z,
            z: -travel.x,
        };
        let position = util::geometry::add(
            util::geometry::sub(p, util::geometry::mul(travel, 6.0)),
            util::geometry::mul(left, road.road_width_start_meters / 2.0 + 0.5),
        );
        if model::source_location::contains(map, position)
            && !build::surfaces::intersects(
                &obstacles,
                &geo::Buffer::buffer(&Point::new(position.x, position.z), 0.08),
            )
        {
            data.objects.push(MapObject {
                kind: ObjectKind::TurnSign,
                position: [position.x as f32, position.z as f32],
                angle: travel.z.atan2(travel.x) as f32 + std::f32::consts::FRAC_PI_2,
                height: 2.1,
                width: 0.6,
                text: rule.to_owned(),
                locked: false,
            });
        }
    }
    data
}

#[cfg(test)]
#[path = "tests/details.rs"]
mod tests;
