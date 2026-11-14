//! Grade-separated carriageways. Layer supplies ordering; absent surveys use a 5 m clearance.
use crate::asset_codecs;
use crate::build_details;
use crate::build_surfaces;
use crate::error::{invalid, Result};
use crate::geometry;
use crate::geometry::XZ;
use crate::map_details::{Data, Deck};
use crate::osm;
use crate::source_location::{SourceLocation, SourceRoad};
use crate::world::{HeightField, RoadRise};
use geo::{BooleanOps, MultiPolygon};
use std::{collections::BTreeMap, fs, path::Path};
struct SurveyNode<'a> {
    node: &'a osm::OSMElement,
    position: XZ,
}
struct SurveyPath {
    points: Vec<XZ>,
    start_height: f32,
    end_height: f32,
}

pub fn grade(way: &osm::OSMElement) -> i8 {
    if way.tag("tunnel") == "building_passage" || way.tag("ford") == "yes" {
        return 0;
    }
    let layer = way.tag("layer").parse::<i8>().unwrap_or(0).clamp(-10, 10);
    if layer != 0 {
        layer
    } else if !matches!(way.tag("bridge"), "" | "no" | "low_water_crossing") {
        1
    } else if !matches!(way.tag("tunnel"), "" | "no") {
        -1
    } else {
        0
    }
}
pub fn road_grade(source: Option<&osm::OSMMap>, road: &SourceRoad) -> i8 {
    source
        .and_then(|s| road.osm_way_id.and_then(|id| s.ways.get(&id)))
        .map_or(0, grade)
}
pub fn build(
    source: &osm::OSMMap,
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
                    position: map.metres(p),
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
            (if node.tag("ele").is_empty() {
                way.tag("ele")
            } else {
                node.tag("ele")
            })
            .trim()
            .trim_end_matches(" m")
            .parse::<f32>()
            .ok()
            .filter(|h| h.is_finite())
            .map(|h| h - datum)
            .unwrap_or(field.height(p.map(|v| v as f32)) + g as f32 * 5.)
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
        let length = geometry::path_length(path).max(0.01);
        for n in road.ends {
            let p = map.road_nodes[n].xz();
            let t = (geometry::project(path, p).1 / length) as f32;
            let lift = a + (b - a) * t - field.height(p.map(|v| v as f32));
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
            let length = geometry::dist(map.road_nodes[n].xz(), map.road_nodes[other].xz()) as f32;
            let lift = lifts[n].signum() * (lifts[n].abs() - length * 0.08).max(0.);
            if lift.abs() > lifts[other].abs() + 0.0001 {
                lifts[other] = lift;
                queue.push_back(other);
            }
        }
    }
    let mut ramps = Vec::new();
    for (i, road) in map.road_lines.iter().enumerate() {
        let path = &surfaces.paths[i];
        let length = geometry::path_length(path).max(0.01);
        if grades[i] != 0 {
            details.decks.push(Deck {
                road: i,
                grade: grades[i],
                width: road.road_width_start_meters as f32,
                path: path
                    .iter()
                    .map(|p| {
                        let t = (geometry::project(path, *p).1 / length) as f32;
                        let a = map.road_nodes[road.ends[0]].xz().map(|v| v as f32);
                        let b = map.road_nodes[road.ends[1]].xz().map(|v| v as f32);
                        [
                            p.x as f32,
                            (field.height(a) + lifts[road.ends[0]]) * (1. - t)
                                + (field.height(b) + lifts[road.ends[1]]) * t,
                            p.z as f32,
                        ]
                    })
                    .collect(),
                surface: build_details::road_surface(
                    source.ways[&road.osm_way_id.unwrap()].tag("surface"),
                ),
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
                let p = geometry::along(path, station);
                let tangent = geometry::unit(geometry::sub(
                    geometry::along(path, (station + 0.01).min(length)),
                    geometry::along(path, (station - 0.01).max(0.)),
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
                    geometry::add(
                        p,
                        geometry::mul(normal, side * (road.road_width_start_meters / 2. + 2.5)),
                    )
                    .map(|v| v as f32)
                    .at_height(offset)
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
    // Ramp triangles may overlap at joins. Partition them first, then add existing humps to the resulting planes.
    let existing =
        asset_codecs::road_rises::parse(&fs::read_to_string(rises_path)?).map_err(invalid)?;
    let mut covered = geometry::empty();
    let mut result = Vec::new();
    for ramp in ramps {
        let footprint = MultiPolygon(vec![geometry::polygon(
            &ramp.points.map(|p| p.xz().map(f64::from)),
        )]);
        let available =
            footprint.difference(&build_surfaces::PolygonIndex::new(&covered).nearby(&footprint));
        let mut remaining = available.clone();
        for hump in &existing {
            let shape = MultiPolygon(vec![geometry::polygon(
                &hump.points.map(|p| p.xz().map(f64::from)),
            )]);
            let overlap = remaining.intersection(&shape);
            emit(&overlap, &ramp, Some(hump), &mut result)?;
            remaining = remaining.difference(&shape);
        }
        emit(&remaining, &ramp, None, &mut result)?;
        covered.0.extend(available.0);
    }
    for hump in existing {
        let shape = MultiPolygon(vec![geometry::polygon(
            &hump.points.map(|p| p.xz().map(f64::from)),
        )]);
        emit(
            &shape.difference(&build_surfaces::PolygonIndex::new(&covered).nearby(&shape)),
            &hump,
            None,
            &mut result,
        )?;
    }
    asset_codecs::road_rises::write(&mut fs::File::create(rises_path)?, &result)?;
    Ok(())
}
fn emit(
    geometry: &MultiPolygon<f64>,
    plane: &RoadRise,
    extra: Option<&RoadRise>,
    result: &mut Vec<RoadRise>,
) -> Result<()> {
    build_surfaces::triangulate(geometry, |points| {
        let points = points.map(|p| p.map(|v| v as f32));
        let [a, b, c] = points;
        let area = ((b.x - a.x) as f64 * (c.z - a.z) as f64
            - (b.z - a.z) as f64 * (c.x - a.x) as f64)
            .abs();
        if area < 0.00001 {
            return Ok(());
        }
        result.push(RoadRise {
            points: points.map(|p| {
                p.at_height(
                    plane.offset(p).unwrap_or(0.) + extra.and_then(|e| e.offset(p)).unwrap_or(0.),
                )
            }),
        });
        Ok(())
    })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn clipped_ramps_remain_readable_after_large_coordinate_quantization() {
        let plane = RoadRise {
            points: [
                XZ {
                    x: 40000.,
                    z: 40000.,
                }
                .at_height(-2.),
                XZ {
                    x: 40010.,
                    z: 40000.,
                }
                .at_height(-2.),
                XZ {
                    x: 40000.,
                    z: 40010.,
                }
                .at_height(-1.),
            ],
        };
        let sliver = MultiPolygon(vec![geometry::polygon(&[
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
        emit(&sliver, &plane, None, &mut rises).unwrap();
        let mut bytes = Vec::new();
        asset_codecs::road_rises::write(&mut bytes, &rises).unwrap();
        assert!(asset_codecs::road_rises::parse(std::str::from_utf8(&bytes).unwrap()).is_ok());
    }
}
