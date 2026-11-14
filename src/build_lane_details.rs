//! Explicit lane directions and roadside lane paint, clipped to the surveyed carriageway.
use crate::build_details;
use crate::build_surfaces;
use crate::build_transport;
use crate::geometry;
use crate::geometry::XZ;
use crate::map_details::RoadSurface;
use crate::osm;
use crate::source_location::SourceLocation;
use geo::{BooleanOps, MultiPolygon};

fn strip(path: &[XZ], offset: f64, half_width: f64) -> MultiPolygon<f64> {
    let points: Vec<_> = path
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let tangent = geometry::unit(geometry::sub(
                path[(i + 1).min(path.len() - 1)],
                path[i.saturating_sub(1)],
            ));
            geometry::add(
                *p,
                geometry::mul(
                    XZ {
                        x: -tangent.z,
                        z: tangent.x,
                    },
                    offset,
                ),
            )
        })
        .collect();
    geometry::line(&points).buffer(half_width)
}
use geo::Buffer;
pub fn paint(
    source: &osm::OSMMap,
    map: &SourceLocation,
    paths: &[Vec<XZ>],
    road: &MultiPolygon<f64>,
) -> MultiPolygon<f64> {
    let mut pieces = Vec::new();
    for (index, line) in map.road_lines.iter().enumerate() {
        let Some(way) = line.osm_way_id.and_then(|id| source.ways.get(&id)) else {
            continue;
        };
        if build_transport::grade(way) != 0
            || build_details::road_surface(way.tag("surface")) == RoadSurface::Unpaved
        {
            continue;
        }
        let path = &paths[index];
        let length = geometry::path_length(path);
        for (side, key, old) in [
            (-1., "cycleway:left", "parking:lane:left"),
            (1., "cycleway:right", "parking:lane:right"),
        ] {
            if way.tag(key) == "lane"
                || way.tag("cycleway:both") == "lane"
                || way.tag("cycleway") == "lane"
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
            if matches!(way.tag(old), "parallel" | "diagonal" | "perpendicular")
                || matches!(way.tag(key), "lane" | "street_side")
            {
                let offset = side * (line.road_width_start_meters / 2. - 2.);
                let mut station = 3.;
                while station + 2. < length {
                    pieces.push(strip(
                        &geometry::substring(path, station, station + 1.5),
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
            let tag = if way.tag(key).is_empty() && line.direction != 0 {
                way.tag("turn:lanes")
            } else {
                way.tag(key)
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
                .map(|p| map.metres(p))
            else {
                continue;
            };
            let mut path = path.clone();
            if backward {
                path.reverse();
            }
            if geometry::dist(*path.last().unwrap(), end) > 1. || length < 10. {
                continue;
            }
            let lanes: Vec<_> = tag.split('|').collect();
            let total = line.lanes.max(lanes.len());
            let lane_width = (line.road_width_start_meters - 0.9) / total as f64;
            for (lane, turns) in lanes.iter().enumerate() {
                let offset =
                    -(line.road_width_start_meters - 0.9) / 2. + lane_width * (lane as f64 + 0.5);
                let station = length - 6.;
                let center = geometry::along(&path, station);
                let travel = geometry::unit(geometry::sub(
                    geometry::along(&path, station + 0.2),
                    geometry::along(&path, station - 0.2),
                ));
                let normal = XZ {
                    x: -travel.z,
                    z: travel.x,
                };
                let transform = |x: f64, z: f64| {
                    geometry::add(
                        center,
                        geometry::add(geometry::mul(normal, x + offset), geometry::mul(travel, z)),
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
                    pieces.push(geometry::line(&[transform(0., -1.5), elbow, tip]).buffer(0.08));
                    let direction = geometry::unit(geometry::sub(tip, elbow));
                    let normal = XZ {
                        x: -direction.z,
                        z: direction.x,
                    };
                    pieces.push(MultiPolygon(vec![geometry::polygon(&[
                        tip,
                        geometry::add(
                            geometry::sub(tip, geometry::mul(direction, 0.65)),
                            geometry::mul(normal, 0.35),
                        ),
                        geometry::sub(
                            geometry::sub(tip, geometry::mul(direction, 0.65)),
                            geometry::mul(normal, 0.35),
                        ),
                    ])]));
                }
            }
        }
    }
    build_surfaces::union(&pieces).intersection(road)
}
