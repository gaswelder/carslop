//! Compile mapped wall and fence ways, leaving openings at mapped gates.

use crate::build;
use crate::model;
use crate::model::source_location::SourceLocation;
use crate::model::world::Barrier;
use crate::model::world::BarrierKind;
use crate::util;
use crate::util::geometry::XZ;
use geo::{BooleanOps, Buffer, MultiLineString, MultiPolygon};

fn opening(node: &build::osm::OSMElement) -> f64 {
    if matches!(
        build::osm::tag(node, "barrier"),
        "gate" | "lift_gate" | "swing_gate" | "entrance" | "kissing_gate" | "stile"
    ) {
        build::osm::number(build::osm::tag(node, "width"))
            .filter(|width| *width > 0.0)
            .unwrap_or(
                if matches!(build::osm::tag(node, "barrier"), "kissing_gate" | "stile") {
                    1.0
                } else {
                    3.5
                },
            )
            .clamp(0.5, 12.0)
            / 2.0
    } else {
        0.0
    }
}

pub fn extract(
    source: &build::osm::OSMMap,
    map: &SourceLocation<f64>,
    surfaces: &build::surfaces::Surfaces,
) -> Vec<Barrier> {
    let obstacles = build::surfaces::new(&surfaces.obstacles);
    let crop = map.boundary.clone().unwrap_or_else(|| {
        MultiPolygon(vec![util::geometry::polygon(&[
            map.bounding_box.min,
            XZ {
                x: map.bounding_box.max.x,
                z: map.bounding_box.min.z,
            },
            map.bounding_box.max,
            XZ {
                x: map.bounding_box.min.x,
                z: map.bounding_box.max.z,
            },
        ])])
    });
    let mut result = Vec::new();
    for way in source.ways.values() {
        let kind = match build::osm::tag(way, "barrier") {
            "wall" => BarrierKind::Wall,
            "hedge" => BarrierKind::Hedge,
            "guard_rail" => BarrierKind::GuardRail,
            "retaining_wall" => BarrierKind::RetainingWall,
            "fence" => match build::osm::tag(way, "fence_type") {
                "chain_link" => BarrierKind::ChainLink,
                "wood" | "wooden" => BarrierKind::WoodFence,
                _ if build::osm::tag(way, "material") == "wood" => BarrierKind::WoodFence,
                _ => BarrierKind::Fence,
            },
            _ => continue,
        };
        if !matches!(build::osm::tag(way, "layer"), "" | "0")
            || ["bridge", "tunnel"]
                .iter()
                .any(|key| !matches!(build::osm::tag(way, key), "" | "no"))
        {
            continue;
        }
        let height = build::osm::number(build::osm::tag(way, "height"))
            .filter(|height| *height > 0.0)
            .unwrap_or(match kind {
                BarrierKind::GuardRail => 0.75,
                BarrierKind::Hedge => 1.5,
                BarrierKind::Wall => 1.6,
                _ => 1.8,
            })
            .clamp(0.2, 8.0);
        let width = build::osm::number(build::osm::tag(way, "width"))
            .filter(|width| *width > 0.0)
            .unwrap_or(if kind == BarrierKind::Hedge {
                0.6
            } else if kind == BarrierKind::Wall {
                0.25
            } else {
                0.08
            })
            .clamp(0.04, 2.0);
        for pair in way.nodes.windows(2) {
            let (Some(start), Some(end)) = (source.nodes.get(&pair[0]), source.nodes.get(&pair[1]))
            else {
                continue;
            };
            let (Some(start_coord), Some(end_coord)) = (start.coord, end.coord) else {
                continue;
            };
            let a = model::source_location::metres(map, start_coord);
            let b = model::source_location::metres(map, end_coord);
            let length = util::geometry::dist(a, b);
            let start_gap = opening(start);
            let end_gap = opening(end);
            if length - start_gap - end_gap < 0.1 {
                continue;
            }
            let travel = util::geometry::mul(util::geometry::sub(b, a), 1.0 / length);
            let a = util::geometry::add(a, util::geometry::mul(travel, start_gap));
            let b = util::geometry::sub(b, util::geometry::mul(travel, end_gap));
            let lines = crop.clip(&MultiLineString(vec![util::geometry::line(&[a, b])]), false);
            let probe = util::geometry::line(&[a, b]).buffer(width / 2.0 + 0.1);
            let nearby = build::surfaces::nearby(&obstacles, &probe);
            // Union the small local candidates before clipping; input pieces overlap.
            let exclusion = build::surfaces::union(
                &nearby
                    .0
                    .into_iter()
                    .map(|polygon| MultiPolygon(vec![polygon]))
                    .collect::<Vec<_>>(),
            )
            .buffer(width / 2.0 + 0.05);
            for line in exclusion.clip(&lines, true).0 {
                for points in line.0.windows(2) {
                    let ends = [points[0], points[1]].map(|p| XZ {
                        x: (p.x * 1000.0).round() as f32 / 1000.0,
                        z: (p.y * 1000.0).round() as f32 / 1000.0,
                    });
                    if (ends[1].x - ends[0].x).hypot(ends[1].z - ends[0].z) < 0.1 {
                        continue;
                    }
                    result.push(Barrier {
                        source_id: way.id as u64,
                        kind,
                        ends,
                        height_meters: height as f32,
                        width_meters: width as f32,
                    });
                }
            }
        }
    }
    result
}

#[cfg(test)]
#[path = "tests/barriers.rs"]
mod tests;
