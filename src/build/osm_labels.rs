//! Extract playable street, building, and storefront labels from source OSM.

use crate::build;
use crate::model;
pub use crate::model::world::Building;
use crate::storage;
use crate::util;
use crate::util::geometry::XZ;
use std::collections::HashMap;

/// Source labels and businesses associated with their containing building.
pub struct OsmLabels {
    pub streets: HashMap<i64, String>,
    pub buildings: HashMap<u64, String>,
    pub storefronts: Vec<(u64, String)>,
}

pub fn read_osm_labels(source: impl std::io::BufRead) -> Result<OsmLabels, String> {
    let map = build::osm::read_xml(source).map_err(|e| e.to_string())?;
    Ok(osm_labels(&map))
}

struct LabelFootprint {
    id: u64,
    polygon: geo::Polygon<f64>,
    bounds: geo::Rect<f64>,
}

fn osm_labels(map: &build::osm::OSMMap) -> OsmLabels {
    use geo::{Area, BoundingRect, Intersects};
    let name = |e: &build::osm::OSMElement| {
        [build::osm::tag(e, "name:en"), build::osm::tag(e, "name")]
            .into_iter()
            .map(str::trim)
            .find(|s| !s.is_empty())
            .unwrap_or("")
            .to_owned()
    };
    let mut labels = OsmLabels {
        streets: HashMap::new(),
        buildings: HashMap::new(),
        storefronts: Vec::new(),
    };
    let mut footprints = Vec::new();
    for way in map.ways.values() {
        let label = name(way);
        let street = if label.is_empty() {
            build::osm::tag(way, "ref").trim()
        } else {
            &label
        };
        if !street.is_empty() {
            labels.streets.insert(way.id, street.to_owned());
        }
        if matches!(build::osm::tag(way, "building"), "" | "no") {
            continue;
        }
        if !label.is_empty() {
            labels.buildings.insert(way.id as u64, label);
        }
        if way.nodes.len() < 4 || way.nodes.first() != way.nodes.last() {
            continue;
        }
        let points: Option<Vec<_>> = way
            .nodes
            .iter()
            .map(|id| map.nodes.get(id)?.coord.map(|p| XZ { x: p.lon, z: p.lat }))
            .collect();
        if let Some(points) = points {
            let polygon = util::geometry::polygon(&points);
            if let Some(bounds) = polygon.bounding_rect() {
                footprints.push(LabelFootprint {
                    id: way.id as u64,
                    polygon,
                    bounds,
                });
            }
        }
    }
    for relation in map.relations.iter().filter(|relation| {
        build::osm::tag(relation, "type") == "multipolygon"
            && !matches!(build::osm::tag(relation, "building"), "" | "no")
    }) {
        let id = build::osm::relation_feature_id(relation.id);
        let label = name(relation);
        if !label.is_empty() {
            labels.buildings.insert(id, label);
        }
        if let Some(geometry) =
            build::buildings::relation_geometry(map, relation, |p| XZ { x: p.lon, z: p.lat })
        {
            for polygon in geometry.0 {
                if let Some(bounds) = polygon.bounding_rect() {
                    footprints.push(LabelFootprint {
                        id,
                        polygon,
                        bounds,
                    });
                }
            }
        }
    }
    let index = rstar::RTree::bulk_load(
        footprints
            .iter()
            .enumerate()
            .map(|(i, footprint)| {
                let bounds = footprint.bounds;
                rstar::primitives::GeomWithData::new(
                    rstar::primitives::Rectangle::from_corners(
                        [bounds.min().x, bounds.min().y],
                        [bounds.max().x, bounds.max().y],
                    ),
                    i,
                )
            })
            .collect(),
    );
    for node in map.nodes.values() {
        // Named outdoor features and transport stops are not building tenants.
        let business = ["shop", "office", "craft"]
            .iter()
            .any(|key| !matches!(build::osm::tag(node, key), "" | "no" | "vacant"))
            || matches!(
                build::osm::tag(node, "amenity"),
                "bank"
                    | "bar"
                    | "cafe"
                    | "restaurant"
                    | "fast_food"
                    | "ice_cream"
                    | "pub"
                    | "pharmacy"
                    | "clinic"
                    | "dentist"
                    | "doctors"
                    | "veterinary"
            )
            || build::osm::tag(node, "tourism") == "gallery"
            || !matches!(build::osm::tag(node, "healthcare"), "" | "no");
        let label = name(node);
        if !business || label.is_empty() {
            continue;
        }
        let Some(p) = node.coord else {
            continue;
        };
        let point = geo::Point::new(p.lon, p.lat);
        let envelope = rstar::AABB::from_point([p.lon, p.lat]);
        let owner = index
            .locate_in_envelope_intersecting(&envelope)
            .map(|entry| &footprints[entry.data])
            .filter(|footprint| footprint.polygon.intersects(&point))
            .min_by(|a, b| {
                a.polygon
                    .unsigned_area()
                    .total_cmp(&b.polygon.unsigned_area())
                    .then(a.id.cmp(&b.id))
            });
        if let Some(owner) = owner {
            labels.storefronts.push((owner.id, label));
        }
    }
    labels.storefronts.sort();
    labels.storefronts.dedup();
    labels
}

/// Build-stage adapter: reduce source tags to playable labels on every rebuild.
pub fn write_labels(directory: &std::path::Path) -> util::error::Result<()> {
    let source = std::fs::File::open(directory.join("source.osm"))?;
    let extracted =
        read_osm_labels(std::io::BufReader::new(source)).map_err(util::error::invalid)?;
    let mut streets: Vec<_> = extracted.streets.into_iter().collect();
    let mut buildings: Vec<_> = extracted.buildings.into_iter().collect();
    streets.sort();
    buildings.sort();
    let labels = model::location::Labels {
        streets,
        buildings,
        storefronts: extracted.storefronts,
    };
    storage::codec_labels::write(directory, &labels)?;
    Ok(())
}
