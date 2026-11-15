//! Resolve mapped building parts against their enclosing outlines.

use crate::build_landscape;
use crate::build_surfaces;
use crate::geometry;
use crate::geometry::{LatLon, XZ};
use crate::osm;
use crate::source_location;
use crate::source_location::SourceLocation;
use crate::world;
use crate::world::{Building, Entrance};
use geo::{
    orient::Direction, Area, BooleanOps, BoundingRect, Contains, MultiPolygon, Orient, Polygon,
};
use rstar::{RTree, RTreeObject, AABB};
use std::collections::BTreeMap;
use std::collections::BTreeSet;

struct Parent {
    index: usize,
    footprint: Polygon<f64>,
    bounds: AABB<XZ>,
}
impl RTreeObject for Parent {
    type Envelope = AABB<XZ>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}
fn bounds(footprint: &Polygon<f64>) -> AABB<XZ> {
    let bounds = footprint
        .bounding_rect()
        .expect("validated building footprint");
    AABB::from_corners(
        XZ {
            x: bounds.min().x,
            z: bounds.min().y,
        },
        XZ {
            x: bounds.max().x,
            z: bounds.max().y,
        },
    )
}

pub fn resolve_parts(source: &osm::OSMMap, map: &mut SourceLocation<f64>) {
    let mut buildings = std::mem::take(&mut map.buildings);
    let is_part: Vec<_> = buildings
        .iter()
        .map(|building| {
            osm::building_source(source, building.id)
                .is_some_and(|way| !matches!(osm::tag(way, "building:part"), "" | "no"))
        })
        .collect();
    let parents = RTree::bulk_load(
        buildings
            .iter()
            .enumerate()
            .filter(|(i, _)| !is_part[*i])
            .map(|(index, building)| {
                let footprint = world::footprint(building);
                Parent {
                    index,
                    bounds: bounds(&footprint),
                    footprint,
                }
            })
            .collect(),
    );
    let indices: BTreeMap<_, _> = buildings
        .iter()
        .enumerate()
        .map(|(i, building)| (building.id, i))
        .collect();
    let mut explicit = BTreeMap::new();
    for relation in source
        .relations
        .iter()
        .filter(|relation| osm::tag(relation, "type") == "building")
    {
        let identity = |member: &(String, i64, String)| {
            if member.0 == "relation" {
                osm::relation_feature_id(member.1)
            } else {
                member.1 as u64
            }
        };
        if let Some(outline) = relation.members.iter().find(|member| member.2 == "outline") {
            for part in relation.members.iter().filter(|member| member.2 == "part") {
                explicit.insert(identity(part), identity(outline));
            }
        }
    }
    let mut children: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut owners = BTreeMap::new();
    for i in 0..buildings.len() {
        if !is_part[i] {
            continue;
        }
        let footprint = world::footprint(&buildings[i]);
        let parent = explicit
            .get(&buildings[i].id)
            .and_then(|id| indices.get(id))
            .copied()
            .or_else(|| {
                parents
                    .locate_in_envelope_intersecting(&bounds(&footprint))
                    .filter(|parent| parent.footprint.contains(&footprint))
                    .min_by(|a, b| {
                        a.footprint
                            .unsigned_area()
                            .total_cmp(&b.footprint.unsigned_area())
                    })
                    .map(|parent| parent.index)
            });
        if let Some(parent) = parent {
            children.entry(parent).or_default().push(i);
            owners.insert(buildings[i].id, buildings[parent].id);
            let way = osm::building_source(source, buildings[i].id).expect("mapped building part");
            if osm::number(osm::tag(way, "height")).is_none()
                && osm::number(osm::tag(way, "building:levels")).is_none()
            {
                buildings[i].height_meters = buildings[parent]
                    .height_meters
                    .max(buildings[i].min_height_meters + 0.1);
            }
            buildings[i].id = buildings[parent].id;
        }
    }
    let part_footprints: BTreeMap<_, _> = children
        .iter()
        .map(|(parent, parts)| {
            (
                *parent,
                build_surfaces::union(
                    &parts
                        .iter()
                        .map(|i| MultiPolygon(vec![world::footprint(&(buildings[*i]))]))
                        .collect::<Vec<_>>(),
                ),
            )
        })
        .collect();
    let mut result = Vec::new();
    for (i, building) in buildings.into_iter().enumerate() {
        if children.contains_key(&i) {
            // Child footprints are collected before consuming the source volumes.
            let covered = &part_footprints[&i];
            let remaining = MultiPolygon(vec![world::footprint(&(building))])
                .difference(covered)
                .orient(Direction::Default);
            for polygon in remaining.0 {
                if polygon.unsigned_area() < 0.01 {
                    continue;
                }
                let mut remainder = building.clone();
                remainder.ground_outline = polygon
                    .exterior()
                    .0
                    .iter()
                    .take(polygon.exterior().0.len() - 1)
                    .map(|p| XZ { x: p.x, z: p.y })
                    .collect();
                remainder.holes = polygon
                    .interiors()
                    .iter()
                    .map(|ring| {
                        ring.0
                            .iter()
                            .take(ring.0.len() - 1)
                            .map(|p| XZ { x: p.x, z: p.y })
                            .collect()
                    })
                    .collect();
                result.push(remainder);
            }
        } else {
            result.push(building);
        }
    }
    for entrance in &mut map.entrances {
        if let Some(owner) = owners.get(&entrance.building_id) {
            entrance.building_id = *owner;
        }
    }
    map.entrances.sort_by(|a, b| {
        a.building_id
            .cmp(&b.building_id)
            .then_with(|| a.position.x.total_cmp(&b.position.x))
            .then_with(|| a.position.z.total_cmp(&b.position.z))
    });
    map.entrances
        .dedup_by(|a, b| a.building_id == b.building_id && a.position == b.position);
    map.buildings = result;
}

/// Assemble a complete building multipolygon using source node identity.
pub fn relation_geometry(
    source: &osm::OSMMap,
    relation: &osm::OSMElement,
    project: impl Fn(LatLon) -> XZ,
) -> Option<MultiPolygon<f64>> {
    let mut outer = Vec::new();
    let mut inner = Vec::new();
    for member in &relation.members {
        if member.0 != "way" {
            continue;
        }
        let way = source.ways.get(&member.1)?;
        if way.nodes.len() < 2 {
            return None;
        }
        if member.2 == "inner" {
            inner.push(way.nodes.clone());
        } else {
            outer.push(way.nodes.clone());
        }
    }
    let make = |parts| -> Option<MultiPolygon<f64>> {
        let rings = build_landscape::rings(parts)?;
        let polygons: Option<Vec<_>> = rings
            .into_iter()
            .map(|ring| {
                let points: Option<Vec<_>> = ring
                    .iter()
                    .map(|id| source.nodes.get(id)?.coord.map(&project))
                    .collect();
                Some(MultiPolygon(vec![geometry::polygon(&points?)]))
            })
            .collect();
        Some(build_surfaces::union(&polygons?))
    };
    Some(
        make(outer)?
            .difference(&make(inner)?)
            .orient(Direction::Default),
    )
}

pub fn import_relations(source: &osm::OSMMap, map: &mut SourceLocation<f64>) {
    for relation in source.relations.iter().filter(|relation| {
        osm::tag(relation, "type") == "multipolygon"
            && (!matches!(osm::tag(relation, "building"), "" | "no")
                || !matches!(osm::tag(relation, "building:part"), "" | "no"))
    }) {
        let Some(geometry) =
            relation_geometry(source, relation, |p| source_location::metres(map, p))
        else {
            continue;
        };
        let height = osm::number(osm::tag(relation, "height")).filter(|v| *v > 0.);
        let min_height = osm::number(osm::tag(relation, "min_height"))
            .or_else(|| osm::number(osm::tag(relation, "building:min_level")).map(|v| v * 3.1))
            .unwrap_or(0.)
            .clamp(0., 69.);
        let top = height
            .or_else(|| {
                osm::number(osm::tag(relation, "building:levels"))
                    .map(|v| v * 3.1 + osm::roof_height(relation))
            })
            .unwrap_or(6.2)
            .clamp(2.5, 70.);
        if top <= min_height {
            continue;
        }
        let id = osm::relation_feature_id(relation.id);
        for polygon in geometry.0 {
            if polygon.unsigned_area() < 4.0
                || polygon
                    .exterior()
                    .0
                    .iter()
                    .any(|p| !source_location::contains(map, XZ { x: p.x, z: p.y }))
                || map
                    .boundary
                    .as_ref()
                    .is_some_and(|boundary| !boundary.contains(&polygon))
            {
                continue;
            }
            map.buildings.push(Building {
                id,
                ground_outline: polygon
                    .exterior()
                    .0
                    .iter()
                    .take(polygon.exterior().0.len() - 1)
                    .map(|p| XZ { x: p.x, z: p.y })
                    .collect(),
                holes: polygon
                    .interiors()
                    .iter()
                    .map(|ring| {
                        ring.0
                            .iter()
                            .take(ring.0.len() - 1)
                            .map(|p| XZ { x: p.x, z: p.y })
                            .collect()
                    })
                    .collect(),
                min_height_meters: min_height,
                height_meters: top,
                base_y: 0.,
                tagged: height.is_some(),
            });
        }
        for member in &relation.members {
            if member.0 != "way" {
                continue;
            }
            if let Some(way) = source.ways.get(&member.1) {
                for node in way.nodes.iter().filter_map(|id| source.nodes.get(id)) {
                    if matches!(osm::tag(node, "entrance"), "" | "no") {
                        continue;
                    }
                    if let Some(p) = node.coord {
                        map.entrances.push(Entrance {
                            building_id: id,
                            position: source_location::metres(map, p),
                        });
                    }
                }
            }
        }
    }
    let ids: BTreeSet<_> = map.buildings.iter().map(|building| building.id).collect();
    map.entrances
        .retain(|entrance| ids.contains(&entrance.building_id));
}

/// A building passage removes the ground volume only; the occupied floors above remain.
pub fn open_passages(source: &osm::OSMMap, map: &mut SourceLocation) {
    use geo::Buffer;
    for road in map.road_lines.iter().filter(|road| {
        road.osm_way_id
            .and_then(|id| source.ways.get(&id))
            .is_some_and(|way| osm::tag(way, "tunnel") == "building_passage")
    }) {
        let way = &source.ways[&road.osm_way_id.unwrap()];
        let corridor = geometry::line(&road.ends.map(|n| geometry::xz(map.road_nodes[n])))
            .buffer(road.road_width_start_meters / 2. + 0.15);
        let clearance = osm::number(osm::tag(way, "maxheight:physical"))
            .or_else(|| osm::number(osm::tag(way, "maxheight")))
            .unwrap_or(3.0)
            .clamp(1.8, 12.);
        let mut result = Vec::new();
        for building in std::mem::take(&mut map.buildings) {
            if building.min_height_meters >= clearance {
                result.push(building);
                continue;
            }
            let footprint = MultiPolygon(vec![world::footprint(&(building))]);
            let overlap = footprint.intersection(&corridor);
            if overlap.unsigned_area() < 0.01 {
                result.push(building);
                continue;
            }
            for polygon in footprint.difference(&corridor).orient(Direction::Default).0 {
                if polygon.unsigned_area() > 0.01 {
                    result.push(volume_polygon(
                        &building,
                        polygon,
                        building.min_height_meters,
                    ));
                }
            }
            if building.height_meters > clearance + 0.2 {
                for polygon in overlap.orient(Direction::Default).0 {
                    if polygon.unsigned_area() > 0.01 {
                        result.push(volume_polygon(&building, polygon, clearance));
                    }
                }
            }
        }
        map.buildings = result;
    }
}
fn volume_polygon(
    building: &Building<f64>,
    polygon: Polygon<f64>,
    min_height: f64,
) -> Building<f64> {
    let mut result = building.clone();
    let points = |ring: &geo::LineString<f64>| {
        ring.0
            .iter()
            .take(ring.0.len().saturating_sub(1))
            .map(|p| XZ { x: p.x, z: p.y })
            .collect()
    };
    result.ground_outline = points(polygon.exterior());
    result.holes = polygon.interiors().iter().map(points).collect();
    result.min_height_meters = min_height;
    result
}
