use super::*;
use crate::util;
use crate::world;
use geo::Area;

fn square(min: XZ<f32>, side: f32, height: f32, slope: f32) -> Vec<RoadRise> {
    let a = util::geometry::at_height(min, height);
    let b = util::geometry::at_height(
        XZ {
            x: min.x + side,
            z: min.z,
        },
        height + slope * side,
    );
    let c = util::geometry::at_height(
        XZ {
            x: min.x + side,
            z: min.z + side,
        },
        height + slope * side,
    );
    let d = util::geometry::at_height(
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
    let mut covered = util::geometry::empty();
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
            util::geometry::add(util::geometry::sub(center, direction), normal),
            util::geometry::add(util::geometry::add(center, direction), normal),
            util::geometry::sub(util::geometry::add(center, direction), normal),
            util::geometry::sub(util::geometry::sub(center, direction), normal),
        ]
        .map(|p| util::geometry::at_height(util::geometry::xz_map(p, |v| v as f32), 1.));
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
    let expected = build::surfaces::union(&[
        MultiPolygon(ramps.iter().map(footprint).collect()).orient(Direction::Default)
    ])
    .unsigned_area();
    let actual: f64 = result
        .iter()
        .map(|rise| footprint(rise).unsigned_area())
        .sum();
    assert!((actual - expected).abs() < 0.05, "{actual} != {expected}");
    let mut bytes = vec![];
    storage::codec_road_rises::write(&mut bytes, &result).unwrap();
    storage::codec_road_rises::parse(std::str::from_utf8(&bytes).unwrap()).unwrap();
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
                        .find_map(|rise| world::raised_roads::road_rise_offset(rise, point))
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
        storage::codec_road_rises::write(&mut bytes, &actual).unwrap();
        storage::codec_road_rises::parse(std::str::from_utf8(&bytes).unwrap()).unwrap();
    }
}

#[test]
fn clipped_ramps_remain_readable_after_large_coordinate_quantization() {
    let plane = RoadRise {
        points: [
            util::geometry::at_height(
                XZ {
                    x: 40000.,
                    z: 40000.,
                },
                -2.,
            ),
            util::geometry::at_height(
                XZ {
                    x: 40010.,
                    z: 40000.,
                },
                -2.,
            ),
            util::geometry::at_height(
                XZ {
                    x: 40000.,
                    z: 40010.,
                },
                -1.,
            ),
        ],
    };
    let sliver = MultiPolygon(vec![util::geometry::polygon(&[
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
    storage::codec_road_rises::write(&mut bytes, &rises).unwrap();
    assert!(storage::codec_road_rises::parse(std::str::from_utf8(&bytes).unwrap()).is_ok());
}
