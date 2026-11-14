//! Compile mapped ground cover and trees into landscape assets.
use crate::asset_codecs;
use crate::build_details;
use crate::build_surfaces;
use crate::build_surfaces::{union, Surfaces};
use crate::error::{invalid, Result};
use crate::geometry;
use crate::geometry::XZ;
use crate::geometry::{empty, line, polygon};
use crate::map_details::TreeStyle;
use crate::osm;
use crate::osm::{OSMElement, OSMMap};
use crate::source_location::SourceLocation;
use crate::world;
use geo::algorithm::buffer;
use geo::MultiPolygon;
use geo::{
    algorithm::orient::Direction, BooleanOps, BoundingRect, Buffer, Contains, Orient, Point,
};
use std::fs;
use std::io::BufWriter;
use std::path::Path;

fn style(e: &OSMElement) -> Option<usize> {
    let surface = e.tag("surface");
    if e.tag("leisure") == "swimming_pool"
        || e.tag("natural") == "water"
        || !e.tag("waterway").is_empty()
    {
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
    } else if matches!(
        e.tag("landuse"),
        "grass" | "meadow" | "forest" | "orchard" | "vineyard" | "farmland"
    ) || matches!(e.tag("natural"), "grassland" | "wood")
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
struct VegetationArea<'a> {
    id: i64,
    element: &'a OSMElement,
    geometry: MultiPolygon<f64>,
}
fn wooded(element: &OSMElement) -> bool {
    element.tag("natural") == "wood" || matches!(element.tag("landuse"), "forest" | "orchard")
}
fn vegetation_id(source: i64, sequence: u64) -> u64 {
    let mut hash = (source as u64)
        .wrapping_mul(0x9e3779b97f4a7c15)
        .wrapping_add(sequence);
    hash = (hash ^ (hash >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    (1_u64 << 62) | ((hash ^ (hash >> 27)) & ((1_u64 << 62) - 1))
}

pub fn build_landscape(
    source: &OSMMap,
    map: &SourceLocation<f64>,
    s: &Surfaces,
    landscape_path: &Path,
) -> Result<Vec<TreeStyle>> {
    let coord = |id: &i64| source.nodes.get(id).map(|e| map.metres(e.coord.unwrap()));
    let mut groups: Vec<Vec<MultiPolygon<f64>>> = vec![vec![]; 9];
    let mut vegetation = Vec::new();
    let mut tree_styles = Vec::new();
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
        if wooded(relation) {
            vegetation.push(VegetationArea {
                id: relation.id,
                element: relation,
                geometry: geometry.clone(),
            });
        }
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
        if !way.tag("waterway").is_empty()
            && way.tag("waterway") != "riverbank"
            && way.tag("area") != "yes"
        {
            points.dedup();
            if points.len() < 2 {
                continue;
            }
            let width = osm::number(way.tag("width")).filter(|v| *v > 0.).unwrap_or(
                match way.tag("waterway") {
                    "river" => 8.,
                    "ditch" | "drain" => 1.,
                    _ => 2.,
                },
            );
            groups[8].push(
                line(&points)
                    .buffer(width.clamp(0.3, 100.) / 2.)
                    .intersection(&crop),
            );
        } else if let Some(width) = path_width(way) {
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
            let geometry = exclude_obstacles(&geometry, &obstacles);
            paths[path_style(way.tag("surface"))].push(geometry);
        } else if points.len() >= 4 && way.nodes.first() == way.nodes.last() {
            let geometry = polygon(&points).intersection(&crop);
            if wooded(way) {
                vegetation.push(VegetationArea {
                    id: way.id,
                    element: way,
                    geometry: geometry.clone(),
                });
            }
            groups[kind].push(geometry);
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
        tree_styles.push(build_details::tree_style(node.id as u64, node));
    }
    let mapped = rstar::RTree::bulk_load(
        source
            .nodes
            .values()
            .filter(|node| node.tag("natural") == "tree")
            .filter_map(|node| node.coord.map(|p| map.metres(p)))
            .collect(),
    );
    let paved_cover = union(
        &[3, 4, 5, 6, 7, 8]
            .into_iter()
            .flat_map(|kind| groups[kind].iter().cloned())
            .collect::<Vec<_>>(),
    );
    let paved_index = build_surfaces::PolygonIndex::new(&paved_cover);
    let mut count = 0;
    let mut accepted = rstar::RTree::new();
    let mut append = |id: u64, p: XZ, element: &OSMElement, default_height: f64| -> Result<()> {
        if accepted
            .nearest_neighbor(&p)
            .is_some_and(|tree| geometry::dist(*tree, p) < 2.5)
            || count >= 50_000
            || !map.contains(p)
            || conflicts(&Point::new(p.x, p.z).buffer(0.35))
            || paved_index.intersects(&Point::new(p.x, p.z).buffer(0.35))
            || mapped
                .nearest_neighbor(&p)
                .is_some_and(|tree| geometry::dist(*tree, p) < 2.5)
        {
            return Ok(());
        }
        let height = osm::number(element.tag("height"))
            .filter(|v| *v > 0.)
            .unwrap_or(default_height)
            .clamp(2.5, 18.);
        let radius = osm::number(element.tag("diameter_crown"))
            .filter(|v| *v > 0.)
            .map(|v| v / 2.)
            .unwrap_or(height * 0.29)
            .clamp(0.7, 4.5);
        file.tree_fields(id as i64, p, height, radius)?;
        tree_styles.push(build_details::tree_style(id, element));
        accepted.insert(p);
        count += 1;
        Ok(())
    };
    for way in source
        .ways
        .values()
        .filter(|way| way.tag("natural") == "tree_row")
    {
        let Some(points) = way.nodes.iter().map(&coord).collect::<Option<Vec<_>>>() else {
            continue;
        };
        if points.len() < 2 {
            continue;
        }
        let length = geometry::path_length(&points);
        let spacing = osm::number(way.tag("tree_row:spacing"))
            .filter(|v| *v > 0.)
            .unwrap_or(5.)
            .clamp(2., 30.);
        let samples = (length / spacing).ceil().clamp(1., 10_000.) as usize;
        for i in 0..=samples {
            append(
                vegetation_id(way.id, i as u64),
                geometry::along(&points, length * i as f64 / samples as f64),
                way,
                5.5,
            )?;
        }
    }
    for area in vegetation {
        let Some(bounds) = area.geometry.bounding_rect() else {
            continue;
        };
        let orchard = area.element.tag("landuse") == "orchard";
        let spacing = if orchard { 6.0 } else { 10.0 };
        let mut row = 0;
        let mut z = bounds.min().y + spacing * 0.5;
        let mut sequence = 0;
        while z < bounds.max().y && sequence < 20_000 {
            let mut x = bounds.min().x + spacing * if row % 2 == 0 || orchard { 0.5 } else { 1.0 };
            while x < bounds.max().x && sequence < 20_000 {
                let id = vegetation_id(area.id, sequence);
                let jitter = if orchard {
                    0.
                } else {
                    ((id & 255) as f64 / 255. - 0.5) * 2.0
                };
                let p = XZ {
                    x: x + jitter,
                    z: z - jitter * 0.6,
                };
                if area.geometry.contains(&Point::new(p.x, p.z)) {
                    append(id, p, area.element, if orchard { 4.5 } else { 8.0 })?;
                }
                sequence += 1;
                x += spacing;
            }
            row += 1;
            z += spacing;
        }
    }
    file.flush()?;
    Ok(tree_styles)
}

fn exclude_obstacles(
    geometry: &MultiPolygon<f64>,
    obstacles: &build_surfaces::PolygonIndex<'_>,
) -> MultiPolygon<f64> {
    // The index stores overlapping road and building pieces. Boolean difference
    // requires a valid MultiPolygon: merge those pieces before subtracting them.
    // Normalize winding first because building footprints may use either direction.
    let nearby = obstacles.nearby(geometry).orient(Direction::Default);
    geometry.difference(&union(&[nearby]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{Area, Coord, Rect};

    #[test]
    fn overlapping_exclusions_do_not_leave_path_patches_inside_obstacles() {
        let path = MultiPolygon(vec![Rect::new(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 2.0 },
        )
        .to_polygon()]);
        let obstacles = MultiPolygon(vec![
            Rect::new(Coord { x: 2.0, y: -1.0 }, Coord { x: 6.0, y: 3.0 })
                .to_polygon()
                .orient(Direction::Reversed),
            Rect::new(Coord { x: 4.0, y: -1.0 }, Coord { x: 8.0, y: 3.0 }).to_polygon(),
        ]);
        let result = exclude_obstacles(&path, &build_surfaces::PolygonIndex::new(&obstacles));
        assert!((result.unsigned_area() - 8.0).abs() < 1e-8);
        for obstacle in &obstacles.0 {
            assert!(result.intersection(obstacle).unsigned_area() < 1e-8);
        }
    }

    #[test]
    fn exclusions_preserve_courtyards_except_where_another_obstacle_covers_them() {
        let path = MultiPolygon(vec![Rect::new(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 2.0 },
        )
        .to_polygon()]);
        let building = geo::Polygon::new(
            Rect::new(Coord { x: 2.0, y: -1.0 }, Coord { x: 8.0, y: 3.0 })
                .to_polygon()
                .exterior()
                .clone(),
            vec![
                Rect::new(Coord { x: 3.0, y: 0.5 }, Coord { x: 7.0, y: 1.5 })
                    .to_polygon()
                    .exterior()
                    .clone(),
            ],
        );
        let road = Rect::new(Coord { x: 5.0, y: -1.0 }, Coord { x: 6.0, y: 3.0 }).to_polygon();
        let obstacles = MultiPolygon(vec![building, road]);
        let result = exclude_obstacles(&path, &build_surfaces::PolygonIndex::new(&obstacles));
        assert!((result.unsigned_area() - 11.0).abs() < 1e-8);
        assert!(result.contains(&Point::new(4.0, 1.0)));
        assert!(!result.contains(&Point::new(5.5, 1.0)));
        assert!(!result.contains(&Point::new(2.5, 1.0)));
    }
}
