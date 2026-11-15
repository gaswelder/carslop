//! Explicit lane directions and roadside lane paint, clipped to the surveyed carriageway.

use crate::build;
use crate::model;
use crate::model::map_details::RoadSurface;
use crate::model::source_location::SourceLocation;
use crate::util;
use crate::util::geometry::XZ;
use geo::{BooleanOps, MultiPolygon};

fn strip(path: &[XZ], offset: f64, half_width: f64) -> MultiPolygon<f64> {
    let points: Vec<_> = path
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let tangent = util::geometry::unit(util::geometry::sub(
                path[(i + 1).min(path.len() - 1)],
                path[i.saturating_sub(1)],
            ));
            util::geometry::add(
                *p,
                util::geometry::mul(
                    XZ {
                        x: -tangent.z,
                        z: tangent.x,
                    },
                    offset,
                ),
            )
        })
        .collect();
    util::geometry::line(&points).buffer(half_width)
}
use geo::Buffer;
pub fn paint(
    source: &build::osm::OSMMap,
    map: &SourceLocation,
    paths: &[Vec<XZ>],
    road: &MultiPolygon<f64>,
) -> MultiPolygon<f64> {
    let mut pieces = Vec::new();
    for (index, line) in map.road_lines.iter().enumerate() {
        let Some(way) = line.osm_way_id.and_then(|id| source.ways.get(&id)) else {
            continue;
        };
        if build::transport::grade(way) != 0
            || build::details::road_surface(build::osm::tag(way, "surface")) == RoadSurface::Unpaved
        {
            continue;
        }
        let path = &paths[index];
        let length = util::geometry::path_length(path);
        for (side, key, old) in [
            (-1., "cycleway:left", "parking:lane:left"),
            (1., "cycleway:right", "parking:lane:right"),
        ] {
            if build::osm::tag(way, key) == "lane"
                || build::osm::tag(way, "cycleway:both") == "lane"
                || build::osm::tag(way, "cycleway") == "lane"
            {
                pieces.push(strip(
                    path,
                    side * (line.road_width_start_meters / 2. - 1.5),
                    0.055,
                ));
            }
            let key = if side < 0. {
                "parking:left"
            } else {
                "parking:right"
            };
            if matches!(
                build::osm::tag(way, old),
                "parallel" | "diagonal" | "perpendicular"
            ) || matches!(build::osm::tag(way, key), "lane" | "street_side")
            {
                let offset = side * (line.road_width_start_meters / 2. - 2.);
                let mut station = 3.;
                while station + 2. < length {
                    pieces.push(strip(
                        &util::geometry::substring(path, station, station + 1.5),
                        offset,
                        0.05,
                    ));
                    station += 6.;
                }
            }
        }
        for backward in [false, true] {
            if (backward && line.direction == 1) || (!backward && line.direction == -1) {
                continue;
            }
            let key = if backward {
                "turn:lanes:backward"
            } else {
                "turn:lanes:forward"
            };
            let tag = if build::osm::tag(way, key).is_empty() && line.direction != 0 {
                build::osm::tag(way, "turn:lanes")
            } else {
                build::osm::tag(way, key)
            };
            if tag.is_empty() {
                continue;
            }
            let terminal = if backward {
                way.nodes.first()
            } else {
                way.nodes.last()
            };
            let Some(end) = terminal
                .and_then(|id| source.nodes.get(id))
                .and_then(|n| n.coord)
                .map(|p| model::source_location::metres(map, p))
            else {
                continue;
            };
            let mut path = path.clone();
            if backward {
                path.reverse();
            }
            if util::geometry::dist(*path.last().unwrap(), end) > 1. || length < 10. {
                continue;
            }
            let lanes: Vec<_> = tag.split('|').collect();
            let total = line.lanes.max(lanes.len());
            let lane_width = (line.road_width_start_meters - 0.9) / total as f64;
            for (lane, turns) in lanes.iter().enumerate() {
                let offset =
                    -(line.road_width_start_meters - 0.9) / 2. + lane_width * (lane as f64 + 0.5);
                let station = length - 6.;
                let center = util::geometry::along(&path, station);
                let travel = util::geometry::unit(util::geometry::sub(
                    util::geometry::along(&path, station + 0.2),
                    util::geometry::along(&path, station - 0.2),
                ));
                let normal = XZ {
                    x: -travel.z,
                    z: travel.x,
                };
                let transform = |x: f64, z: f64| {
                    util::geometry::add(
                        center,
                        util::geometry::add(
                            util::geometry::mul(normal, x + offset),
                            util::geometry::mul(travel, z),
                        ),
                    )
                };
                for turn in turns.split(';') {
                    let lateral = match turn {
                        "through" => 0.,
                        "left" | "slight_left" | "sharp_left" => -1.,
                        "right" | "slight_right" | "sharp_right" => 1.,
                        _ => continue,
                    };
                    let tip = transform(lateral * 0.8, 1.5);
                    let elbow = transform(0., if lateral == 0. { 1.0 } else { 0.4 });
                    pieces.push(
                        util::geometry::line(&[transform(0., -1.5), elbow, tip]).buffer(0.08),
                    );
                    let direction = util::geometry::unit(util::geometry::sub(tip, elbow));
                    let normal = XZ {
                        x: -direction.z,
                        z: direction.x,
                    };
                    pieces.push(MultiPolygon(vec![util::geometry::polygon(&[
                        tip,
                        util::geometry::add(
                            util::geometry::sub(tip, util::geometry::mul(direction, 0.65)),
                            util::geometry::mul(normal, 0.35),
                        ),
                        util::geometry::sub(
                            util::geometry::sub(tip, util::geometry::mul(direction, 0.65)),
                            util::geometry::mul(normal, 0.35),
                        ),
                    ])]));
                }
            }
        }
    }
    build::surfaces::union(&pieces).intersection(road)
}
