//! Grade-separated carriageways. Layer supplies ordering; absent surveys use a 5 m clearance.

use crate::build_details;
use crate::build_osm;
use crate::build_surfaces;
use crate::model_map_details::{Data, Deck};
use crate::model_source_location;
use crate::model_source_location::{SourceLocation, SourceRoad};
use crate::model_world::{HeightField, RoadRise};
use crate::storage_codec_road_rises;
use crate::util_error::Result;
use crate::util_geometry;
use crate::util_geometry::XZ;
use crate::world_elevation;
use crate::world_raised_roads;
use geo::algorithm::orient::Direction;
use geo::{BooleanOps, BoundingRect, MultiPolygon, Orient, Polygon};
use std::{
    collections::BTreeMap,
    fs,
    path::Path,
    time::{Duration, Instant},
};
struct SurveyNode<'a> {
    node: &'a build_osm::OSMElement,
    position: XZ,
}
struct SurveyPath {
    points: Vec<XZ>,
    start_height: f32,
    end_height: f32,
}

pub fn grade(way: &build_osm::OSMElement) -> i8 {
    if build_osm::tag(way, "tunnel") == "building_passage" || build_osm::tag(way, "ford") == "yes" {
        return 0;
    }
    let layer = build_osm::tag(way, "layer")
        .parse::<i8>()
        .unwrap_or(0)
        .clamp(-10, 10);
    if layer != 0 {
        layer
    } else if !matches!(
        build_osm::tag(way, "bridge"),
        "" | "no" | "low_water_crossing"
    ) {
        1
    } else if !matches!(build_osm::tag(way, "tunnel"), "" | "no") {
        -1
    } else {
        0
    }
}
pub fn road_grade(source: Option<&build_osm::OSMMap>, road: &SourceRoad) -> i8 {
    source
        .and_then(|s| road.osm_way_id.and_then(|id| s.ways.get(&id)))
        .map_or(0, grade)
}
pub fn build(
    source: &build_osm::OSMMap,
    map: &SourceLocation,
    surfaces: &build_surfaces::Surfaces,
    field: &HeightField,
    datum: f32,
    details: &mut Data,
    rises_path: &Path,
) -> Result<()> {
    let grades: Vec<_> = map
        .road_lines
        .iter()
        .map(|road| road_grade(Some(source), road))
        .collect();
    println!("  Resolving bridge and tunnel heights...");
    let mut lifts = vec![0f32; map.road_nodes.len()];
    let mut fixed = vec![false; map.road_nodes.len()];
    // Interpolate each surveyed source way between its endpoints rather than following river-bed DEM posts.
    let mut heights = BTreeMap::new();
    for road in map
        .road_lines
        .iter()
        .filter(|r| road_grade(Some(source), r) != 0)
    {
        let Some(id) = road.osm_way_id else { continue };
        if heights.contains_key(&id) {
            continue;
        }
        let way = &source.ways[&id];
        let points: Vec<_> = way
            .nodes
            .iter()
            .filter_map(|id| source.nodes.get(id))
            .filter_map(|node| {
                node.coord.map(|p| SurveyNode {
                    node,
                    position: model_source_location::metres(map, p),
                })
            })
            .collect();
        if points.len() < 2 {
            continue;
        }
        let path: Vec<_> = points.iter().map(|point| point.position).collect();
        let g = grade(way);
        let height = |index: usize| {
            let SurveyNode { node, position: p } = points[index];
            (if build_osm::tag(node, "ele").is_empty() {
                build_osm::tag(way, "ele")
            } else {
                build_osm::tag(node, "ele")
            })
            .trim()
            .trim_end_matches(" m")
            .parse::<f32>()
            .ok()
            .filter(|h| h.is_finite())
            .map(|h| h - datum)
            .unwrap_or(
                world_elevation::height(field, util_geometry::xz_map(p, |v| v as f32))
                    + g as f32 * 5.,
            )
        };
        heights.insert(
            id,
            SurveyPath {
                points: path,
                start_height: height(0),
                end_height: height(points.len() - 1),
            },
        );
    }
    for (road, g) in map.road_lines.iter().zip(&grades) {
        if *g == 0 {
            continue;
        }
        let Some(SurveyPath {
            points: path,
            start_height: a,
            end_height: b,
        }) = road.osm_way_id.and_then(|id| heights.get(&id))
        else {
            continue;
        };
        let length = util_geometry::path_length(path).max(0.01);
        for n in road.ends {
            let p = util_geometry::xz(map.road_nodes[n]);
            let t = (util_geometry::project(path, p).1 / length) as f32;
            let lift = a + (b - a) * t
                - world_elevation::height(field, util_geometry::xz_map(p, |v| v as f32));
            if !fixed[n] || lift.abs() > lifts[n].abs() {
                lifts[n] = lift;
                fixed[n] = true;
            }
        }
    }
    // Shared OSM endpoints connect approaches. Spread a bounded gradient into the ground network.
    let mut adjacency = vec![Vec::new(); map.road_nodes.len()];
    for (i, road) in map
        .road_lines
        .iter()
        .enumerate()
        .filter(|(i, _)| grades[*i] == 0)
    {
        for n in road.ends {
            adjacency[n].push(i);
        }
    }
    let mut queue: std::collections::VecDeque<_> = fixed
        .iter()
        .enumerate()
        .filter(|(_, v)| **v)
        .map(|(i, _)| i)
        .collect();
    while let Some(n) = queue.pop_front() {
        for i in &adjacency[n] {
            let road = &map.road_lines[*i];
            let other = if road.ends[0] == n {
                road.ends[1]
            } else {
                road.ends[0]
            };
            if fixed[other] {
                continue;
            }
            let length = util_geometry::dist(
                util_geometry::xz(map.road_nodes[n]),
                util_geometry::xz(map.road_nodes[other]),
            ) as f32;
            let lift = lifts[n].signum() * (lifts[n].abs() - length * 0.08).max(0.);
            if lift.abs() > lifts[other].abs() + 0.0001 {
                lifts[other] = lift;
                queue.push_back(other);
            }
        }
    }
    println!("  Building bridge decks and approach ramps...");
    let mut ramps = Vec::new();
    for (i, road) in map.road_lines.iter().enumerate() {
        let path = &surfaces.paths[i];
        let length = util_geometry::path_length(path).max(0.01);
        if grades[i] != 0 {
            details.decks.push(Deck {
                road: i,
                grade: grades[i],
                width: road.road_width_start_meters as f32,
                path: path
                    .iter()
                    .map(|p| {
                        let t = (util_geometry::project(path, *p).1 / length) as f32;
                        let a = util_geometry::xz_map(
                            util_geometry::xz(map.road_nodes[road.ends[0]]),
                            |v| v as f32,
                        );
                        let b = util_geometry::xz_map(
                            util_geometry::xz(map.road_nodes[road.ends[1]]),
                            |v| v as f32,
                        );
                        [
                            p.x as f32,
                            (world_elevation::height(field, a) + lifts[road.ends[0]]) * (1. - t)
                                + (world_elevation::height(field, b) + lifts[road.ends[1]]) * t,
                            p.z as f32,
                        ]
                    })
                    .collect(),
                surface: build_details::road_surface(build_osm::tag(
                    &source.ways[&road.osm_way_id.unwrap()],
                    "surface",
                )),
            });
        } else if road.ends.iter().any(|n| lifts[*n].abs() > 0.001) {
            // Include an interior zero knot: long approaches retain an 8% maximum grade.
            let a = lifts[road.ends[0]];
            let b = lifts[road.ends[1]];
            let lift = |s: f32| {
                let left = a.signum() * (a.abs() - s * 0.08).max(0.);
                let right = b.signum() * (b.abs() - (length as f32 - s) * 0.08).max(0.);
                if left.abs() > right.abs() {
                    left
                } else {
                    right
                }
            };
            let steps = (length / 4.).ceil().max(1.) as usize;
            let mut sections = Vec::new();
            for step in 0..=steps {
                let station = length * step as f64 / steps as f64;
                let p = util_geometry::along(path, station);
                let tangent = util_geometry::unit(util_geometry::sub(
                    util_geometry::along(path, (station + 0.01).min(length)),
                    util_geometry::along(path, (station - 0.01).max(0.)),
                ));
                let normal = XZ {
                    x: -tangent.z,
                    z: tangent.x,
                };
                let offset = if step == 0 {
                    a
                } else if step == steps {
                    b
                } else {
                    lift(station as f32)
                };
                sections.push([-1., 1.].map(|side| {
                    util_geometry::at_height(
                        util_geometry::xz_map(
                            util_geometry::add(
                                p,
                                util_geometry::mul(
                                    normal,
                                    side * (road.road_width_start_meters / 2. + 2.5),
                                ),
                            ),
                            |v| v as f32,
                        ),
                        offset,
                    )
                }));
            }
            for pair in sections.windows(2) {
                for points in [
                    [pair[0][0], pair[0][1], pair[1][1]],
                    [pair[0][0], pair[1][1], pair[1][0]],
                ] {
                    ramps.push(RoadRise { points });
                }
            }
        }
    }
    // Partition overlapping approach ramps so joins have a single height plane.
    let result = partition_ramps(&ramps)?;
    println!("  Writing {} approach ramp triangles...", result.len());
    storage_codec_road_rises::write(&mut fs::File::create(rises_path)?, &result)?;
    Ok(())
}

struct IndexedFootprint {
    index: usize,
    bounds: rstar::AABB<XZ>,
}
impl rstar::RTreeObject for IndexedFootprint {
    type Envelope = rstar::AABB<XZ>;

    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}

// Keep source indices stable so overlapping ramps retain their original precedence.
struct Footprints {
    polygons: Vec<Polygon<f64>>,
    tree: rstar::RTree<IndexedFootprint>,
}

fn new() -> Footprints {
    Footprints {
        polygons: vec![],
        tree: rstar::RTree::new(),
    }
}

fn bounds(polygon: &Polygon<f64>) -> Option<rstar::AABB<XZ>> {
    let bounds = polygon.bounding_rect()?;
    Some(rstar::AABB::from_corners(
        XZ {
            x: bounds.min().x,
            z: bounds.min().y,
        },
        XZ {
            x: bounds.max().x,
            z: bounds.max().y,
        },
    ))
}

fn insert(footprints: &mut Footprints, polygon: Polygon<f64>) {
    if let Some(bounds) = bounds(&polygon) {
        footprints.tree.insert(IndexedFootprint {
            index: footprints.polygons.len(),
            bounds,
        });
        footprints.polygons.push(polygon);
    }
}

fn candidates(footprints: &Footprints, polygon: &Polygon<f64>) -> Vec<usize> {
    let Some(bounds) = bounds(polygon) else {
        return vec![];
    };
    let mut indices: Vec<_> = footprints
        .tree
        .locate_in_envelope_intersecting(&bounds)
        .map(|item| item.index)
        .collect();
    indices.sort_unstable();
    indices
}

fn subtract(footprints: &Footprints, polygon: &Polygon<f64>) -> MultiPolygon<f64> {
    let indices = candidates(footprints, polygon);
    if indices.is_empty() {
        return MultiPolygon(vec![polygon.clone()]);
    }
    // Input ramp triangles overlap; union them before using them as a clip.
    let nearby = MultiPolygon(
        indices
            .into_iter()
            .map(|index| footprints.polygons[index].clone())
            .collect(),
    );
    polygon.difference(&build_surfaces::union(&[nearby.orient(Direction::Default)]))
}

fn footprint(rise: &RoadRise) -> Polygon<f64> {
    util_geometry::polygon(
        &rise
            .points
            .map(|p| util_geometry::xz_map(util_geometry::xz(p), f64::from)),
    )
}

fn partition_ramps(ramps: &[RoadRise]) -> Result<Vec<RoadRise>> {
    println!("  Partitioning {} approach ramp triangles...", ramps.len());
    let mut covered = new();
    let mut result = Vec::new();
    let mut last_progress = Instant::now();
    for (index, ramp) in ramps.iter().enumerate() {
        let shape = footprint(ramp);
        let available = subtract(&covered, &shape);
        emit(&available, ramp, &mut result)?;
        // The union of previous ramps and this entire input footprint is the
        // same coverage as adding only its available pieces. Keep the original
        // triangle so clipping round-off and tiny fragments cannot accumulate.
        insert(&mut covered, shape);
        if index + 1 == ramps.len() || last_progress.elapsed() >= Duration::from_secs(5) {
            println!("  Ramp triangles: {}/{}", index + 1, ramps.len());
            last_progress = Instant::now();
        }
    }
    Ok(result)
}

fn emit(geometry: &MultiPolygon<f64>, plane: &RoadRise, result: &mut Vec<RoadRise>) -> Result<()> {
    build_surfaces::triangulate(geometry, |points| {
        let points = points.map(|p| util_geometry::xz_map(p, |v| v as f32));
        let [a, b, c] = points;
        let area = ((b.x - a.x) as f64 * (c.z - a.z) as f64
            - (b.z - a.z) as f64 * (c.x - a.x) as f64)
            .abs();
        if area < 0.00001 {
            return Ok(());
        }
        result.push(RoadRise {
            points: points.map(|p| {
                util_geometry::at_height(
                    p,
                    world_raised_roads::road_rise_offset(plane, p).unwrap_or(0.),
                )
            }),
        });
        Ok(())
    })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::util_geometry;
    use crate::world_raised_roads;

    use super::*;
    use geo::Area;

    fn square(min: XZ<f32>, side: f32, height: f32, slope: f32) -> Vec<RoadRise> {
        let a = util_geometry::at_height(min, height);
        let b = util_geometry::at_height(
            XZ {
                x: min.x + side,
                z: min.z,
            },
            height + slope * side,
        );
        let c = util_geometry::at_height(
            XZ {
                x: min.x + side,
                z: min.z + side,
            },
            height + slope * side,
        );
        let d = util_geometry::at_height(
            XZ {
                x: min.x,
                z: min.z + side,
            },
            height,
        );
        vec![
            RoadRise { points: [a, b, c] },
            RoadRise { points: [a, c, d] },
        ]
    }

    // Check ramp height and precedence independently of spatial queries.
    fn exhaustive(ramps: &[RoadRise]) -> Vec<RoadRise> {
        let mut covered = util_geometry::empty();
        let mut result = vec![];
        for ramp in ramps {
            let shape = MultiPolygon(vec![footprint(ramp)]);
            let available = shape.difference(&covered);
            emit(&available, ramp, &mut result).unwrap();
            covered.0.extend(available.0);
        }
        result
    }

    #[test]
    fn overlapping_ramps_do_not_reuse_clipped_fragments() {
        let mut ramps = vec![];
        for i in 0..30 {
            let center = XZ {
                x: 4500. + (i % 30) as f64 * 1.3,
                z: -4500. + (i / 30) as f64 * 1.3,
            };
            let angle = i as f64 * 0.37;
            let direction = XZ {
                x: angle.cos() * 4.,
                z: angle.sin() * 4.,
            };
            let normal = XZ {
                x: -direction.z * 0.3,
                z: direction.x * 0.3,
            };
            let points = [
                util_geometry::add(util_geometry::sub(center, direction), normal),
                util_geometry::add(util_geometry::add(center, direction), normal),
                util_geometry::sub(util_geometry::add(center, direction), normal),
                util_geometry::sub(util_geometry::sub(center, direction), normal),
            ]
            .map(|p| util_geometry::at_height(util_geometry::xz_map(p, |v| v as f32), 1.));
            ramps.extend([
                RoadRise {
                    points: [points[0], points[1], points[2]],
                },
                RoadRise {
                    points: [points[0], points[2], points[3]],
                },
            ]);
        }
        let result = partition_ramps(&ramps).unwrap();
        let expected =
            build_surfaces::union(&[
                MultiPolygon(ramps.iter().map(footprint).collect()).orient(Direction::Default)
            ])
            .unsigned_area();
        let actual: f64 = result
            .iter()
            .map(|rise| footprint(rise).unsigned_area())
            .sum();
        assert!((actual - expected).abs() < 0.05, "{actual} != {expected}");
        let mut bytes = vec![];
        storage_codec_road_rises::write(&mut bytes, &result).unwrap();
        storage_codec_road_rises::parse(std::str::from_utf8(&bytes).unwrap()).unwrap();
    }

    #[test]
    fn indexed_partition_preserves_overlaps_slopes_and_source_precedence() {
        let mut ramps = square(XZ { x: -8., z: -8. }, 12., -2., 0.125);
        ramps.extend(square(XZ { x: -2., z: -4. }, 12., 3., -0.25));
        for active_ramps in [&ramps[..], &ramps[..0]] {
            let expected = exhaustive(active_ramps);
            let actual = partition_ramps(active_ramps).unwrap();
            let area = |rises: &[RoadRise]| {
                rises
                    .iter()
                    .map(|rise| footprint(rise).unsigned_area())
                    .sum::<f64>()
            };
            assert!((area(&expected) - area(&actual)).abs() < 0.001);
            for row in -40..100 {
                for col in -40..100 {
                    let point = XZ {
                        x: col as f32 * 0.3 + 0.137,
                        z: row as f32 * 0.3 + 0.193,
                    };
                    let height = |rises: &[RoadRise]| {
                        rises
                            .iter()
                            .find_map(|rise| world_raised_roads::road_rise_offset(rise, point))
                    };
                    match (height(&expected), height(&actual)) {
                        (Some(expected), Some(actual)) => {
                            assert!((expected - actual).abs() < 0.0001)
                        }
                        (None, None) => {}
                        other => panic!("coverage differs at {point:?}: {other:?}"),
                    }
                }
            }
            let mut bytes = vec![];
            storage_codec_road_rises::write(&mut bytes, &actual).unwrap();
            storage_codec_road_rises::parse(std::str::from_utf8(&bytes).unwrap()).unwrap();
        }
    }

    #[test]
    fn clipped_ramps_remain_readable_after_large_coordinate_quantization() {
        let plane = RoadRise {
            points: [
                util_geometry::at_height(
                    XZ {
                        x: 40000.,
                        z: 40000.,
                    },
                    -2.,
                ),
                util_geometry::at_height(
                    XZ {
                        x: 40010.,
                        z: 40000.,
                    },
                    -2.,
                ),
                util_geometry::at_height(
                    XZ {
                        x: 40000.,
                        z: 40010.,
                    },
                    -1.,
                ),
            ],
        };
        let sliver = MultiPolygon(vec![util_geometry::polygon(&[
            XZ {
                x: 40000.,
                z: 40000.,
            },
            XZ {
                x: 40000.0001,
                z: 40000.,
            },
            XZ {
                x: 40000.,
                z: 40001.,
            },
        ])]);
        let mut rises = Vec::new();
        emit(&sliver, &plane, &mut rises).unwrap();
        let mut bytes = Vec::new();
        storage_codec_road_rises::write(&mut bytes, &rises).unwrap();
        assert!(storage_codec_road_rises::parse(std::str::from_utf8(&bytes).unwrap()).is_ok());
    }
}
