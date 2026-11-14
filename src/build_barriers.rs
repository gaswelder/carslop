//! Compile mapped wall and fence ways, leaving openings at mapped gates.
use crate::asset_codecs;
use crate::build_surfaces;
use crate::error::Result;
use crate::geometry;
use crate::geometry::XZ;
use crate::osm;
use crate::source_location::SourceLocation;
use crate::world::{Barrier, BarrierKind};
use geo::{BooleanOps, Buffer, MultiLineString, MultiPolygon};
use std::{fs, path::Path};

fn opening(node: &osm::OSMElement) -> f64 {
    if matches!(
        node.tag("barrier"),
        "gate" | "lift_gate" | "swing_gate" | "entrance" | "kissing_gate" | "stile"
    ) {
        osm::number(node.tag("width"))
            .filter(|width| *width > 0.0)
            .unwrap_or(if matches!(node.tag("barrier"), "kissing_gate" | "stile") {
                1.0
            } else {
                3.5
            })
            .clamp(0.5, 12.0)
            / 2.0
    } else {
        0.0
    }
}

fn extract(
    source: &osm::OSMMap,
    map: &SourceLocation<f64>,
    surfaces: &build_surfaces::Surfaces,
) -> Vec<Barrier> {
    let obstacles = build_surfaces::PolygonIndex::new(&surfaces.obstacles);
    let crop = map.boundary.clone().unwrap_or_else(|| {
        MultiPolygon(vec![geometry::polygon(&[
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
        let kind = match way.tag("barrier") {
            "wall" => BarrierKind::Wall,
            "fence" => match way.tag("fence_type") {
                "chain_link" => BarrierKind::ChainLink,
                "wood" | "wooden" => BarrierKind::WoodFence,
                _ if way.tag("material") == "wood" => BarrierKind::WoodFence,
                _ => BarrierKind::Fence,
            },
            _ => continue,
        };
        if !matches!(way.tag("layer"), "" | "0")
            || ["bridge", "tunnel"]
                .iter()
                .any(|key| !matches!(way.tag(key), "" | "no"))
        {
            continue;
        }
        let height = osm::number(way.tag("height"))
            .filter(|height| *height > 0.0)
            .unwrap_or(if kind == BarrierKind::Wall { 1.6 } else { 1.8 })
            .clamp(0.2, 8.0);
        let width = osm::number(way.tag("width"))
            .filter(|width| *width > 0.0)
            .unwrap_or(if kind == BarrierKind::Wall {
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
            let a = map.metres(start_coord);
            let b = map.metres(end_coord);
            let length = geometry::dist(a, b);
            let start_gap = opening(start);
            let end_gap = opening(end);
            if length - start_gap - end_gap < 0.1 {
                continue;
            }
            let travel = geometry::mul(geometry::sub(b, a), 1.0 / length);
            let a = geometry::add(a, geometry::mul(travel, start_gap));
            let b = geometry::sub(b, geometry::mul(travel, end_gap));
            let lines = crop.clip(&MultiLineString(vec![geometry::line(&[a, b])]), false);
            let probe = geometry::line(&[a, b]).buffer(width / 2.0 + 0.1);
            let nearby = obstacles.nearby(&probe);
            // Union the small local candidates before clipping; input pieces overlap.
            let exclusion = build_surfaces::union(
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

pub fn build_barriers(
    source: &osm::OSMMap,
    map: &SourceLocation<f64>,
    surfaces: &build_surfaces::Surfaces,
    path: &Path,
) -> Result<()> {
    let records = extract(source, map, surfaces);
    println!("  Writing {} mapped wall/fence segments...", records.len());
    asset_codecs::barriers::write(
        &mut std::io::BufWriter::new(fs::File::create(path)?),
        &records,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{BoundingBoxXZ, LatLon};
    use crate::source_location::SourceRoad;
    use crate::test_support;
    use crate::world;
    use std::collections::BTreeMap;

    #[test]
    fn mapped_barriers_clip_to_crop_avoid_roads_and_leave_gate_openings() {
        let mut map = SourceLocation::empty();
        let origin = LatLon::new(35.162, 33.320);
        map.origin = Some(origin);
        map.bounding_box = BoundingBoxXZ {
            min: XZ { x: -50., z: -50. },
            max: XZ { x: 50., z: 50. },
        };
        let mut source = osm::OSMMap {
            nodes: BTreeMap::new(),
            ways: BTreeMap::new(),
            relations: Vec::new(),
            bounds: None,
        };
        let points: [XZ<f64>; 5] = [
            XZ { x: -60., z: 20. },
            XZ { x: 0., z: 20. },
            XZ { x: 60., z: 20. },
            XZ { x: 20., z: -30. },
            XZ { x: 20., z: 30. },
        ];
        for (i, p) in points.iter().enumerate() {
            let tags = if i == 1 {
                vec![("barrier", "gate"), ("width", "4")]
            } else {
                vec![]
            };
            let mut node = test_support::osm_element(i as i64 + 1, &tags);
            node.coord = Some(LatLon::new(
                origin.lat - p.z.to_degrees() / 6371000.,
                origin.lon + p.x.to_degrees() / 6371000. / origin.lat.to_radians().cos(),
            ));
            source.nodes.insert(node.id, node);
        }
        let mut wall = test_support::osm_element(
            10,
            &[("barrier", "wall"), ("height", "2.3"), ("width", "0.4")],
        );
        wall.nodes = vec![1, 2, 3];
        source.ways.insert(wall.id, wall);
        let mut fence =
            test_support::osm_element(11, &[("barrier", "fence"), ("fence_type", "chain_link")]);
        fence.nodes = vec![4, 5];
        source.ways.insert(fence.id, fence);
        map.road_nodes = vec![
            XZ { x: -40., z: 0. }.at_height(0.),
            XZ { x: 40., z: 0. }.at_height(0.),
        ];
        let mut road = SourceRoad::new([0, 1], 8.0);
        road.osm_way_id = Some(20);
        map.road_lines.push(road);
        map.buildings.push(world::Building {
            id: 40,
            height_meters: 3.0,
            base_y: 0.0,
            tagged: true,
            ground_outline: vec![
                XZ { x: 30., z: 15. },
                XZ { x: 35., z: 15. },
                XZ { x: 35., z: 25. },
                XZ { x: 30., z: 25. },
            ],
        });
        let folder = tempfile::tempdir().unwrap();
        let surfaces = build_surfaces::write_streets_txt(&map, &folder.path().join("streets.txt"), None)
            .unwrap()
            .surfaces;
        let barriers = extract(&source, &map, &surfaces);
        assert_eq!(barriers.len(), 5);
        for barrier in &barriers {
            assert!(barrier
                .ends
                .iter()
                .all(|p| p.x >= -50. && p.x <= 50. && p.z >= -50. && p.z <= 50.));
            if barrier.kind == BarrierKind::Wall {
                assert_eq!(barrier.height_meters, 2.3);
                assert_eq!(barrier.width_meters, 0.4);
                assert!(barrier.ends.iter().all(|p| p.x.abs() >= 2.));
                assert!(barrier.ends[0].x * barrier.ends[1].x > 0.);
                assert!(
                    barrier.ends.iter().all(|p| p.x < 30.)
                        || barrier.ends.iter().all(|p| p.x > 35.)
                );
            } else {
                assert_eq!(barrier.kind, BarrierKind::ChainLink);
                assert!(barrier.ends.iter().all(|p| p.z.abs() >= 4.08));
                assert!(barrier.ends[0].z * barrier.ends[1].z > 0.);
            }
        }
        source.ways.get_mut(&11).unwrap().nodes = vec![4, 999, 5];
        assert!(extract(&source, &map, &surfaces)
            .iter()
            .all(|b| b.kind == BarrierKind::Wall));
    }
}
