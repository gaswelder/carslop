use super::*;
use crate::build;
use crate::model;
use crate::storage;
use crate::test;
use crate::test::support::Temp;
use crate::util;
use crate::util::geometry::project;
use crate::util::geometry::BoundingBoxXZ;
use crate::world;
use geo::{Contains, Point};
use std::collections::BTreeMap;

#[test]
fn gravel_routes_do_not_replace_overlapping_paved_carriageways() {
    struct Case {
        surface: &'static str,
        material: model::world::Material,
    }
    for case in [
        Case {
            surface: "asphalt",
            material: model::world::Material::Asphalt,
        },
        Case {
            surface: "concrete",
            material: model::world::Material::Concrete,
        },
        Case {
            surface: "paving_stones",
            material: model::world::Material::Paving,
        },
    ] {
        for reverse in [false, true] {
            let mut location = model::source_location::empty();
            location.bounding_box = BoundingBoxXZ {
                min: XZ { x: -80., z: -80. },
                max: XZ { x: 80., z: 80. },
            };
            location.road_nodes = [
                XZ { x: -60., z: 0. },
                XZ { x: 60., z: 0. },
                XZ { x: 0., z: -40. },
                XZ { x: 0., z: 40. },
            ]
            .map(|p| util::geometry::at_height(p, 0.))
            .into();
            let mut paved = model::source_location::new([0, 1], 10.);
            paved.osm_way_id = Some(100);
            let mut gravel = model::source_location::new([2, 3], 6.);
            gravel.osm_way_id = Some(200);
            location.road_lines = vec![paved, gravel];
            if reverse {
                location.road_lines.reverse();
            }
            let source = build::osm::OSMMap {
                nodes: BTreeMap::new(),
                ways: BTreeMap::from([
                    (
                        100,
                        test::support::osm_element(
                            100,
                            &[("highway", "raceway"), ("surface", case.surface)],
                        ),
                    ),
                    (
                        200,
                        test::support::osm_element(
                            200,
                            &[("highway", "raceway"), ("surface", "gravel")],
                        ),
                    ),
                ]),
                relations: vec![],
                bounds: None,
            };
            let directory = tempfile::tempdir().unwrap();
            let path = directory.path().join("streets.txt");
            build_streets(&location, &path.parent().unwrap(), Some(&source)).unwrap();
            let loaded = test::support::load_streets(&std::fs::read_to_string(path).unwrap(), 2)
                .unwrap()
                .surfaces;
            let material_at = |p: XZ| {
                world::street_surface::road_triangles(&loaded)
                    .iter()
                    .find(|triangle| {
                        matches!(triangle.usage, model::world::TriangleUsage::Road(_))
                            && util::geometry::polygon(
                                &triangle
                                    .points
                                    .map(|p| util::geometry::xz_map(p, f64::from)),
                            )
                            .contains(&Point::new(p.x, p.z))
                    })
                    .expect("road surface")
                    .material
            };
            assert_eq!(material_at(XZ { x: 0.137, z: 0.193 }), case.material);
            assert_eq!(
                material_at(XZ {
                    x: 0.137,
                    z: 20.193
                }),
                model::world::Material::BareGround
            );
        }
    }
}
fn map(
    nodes: &[XZ],
    roads: &[(usize, usize, f64, usize)],
    buildings: &[Vec<XZ>],
) -> SourceLocation<f64> {
    SourceLocation {
        boundary: None,
        origin: Some(util::geometry::new(35.162, 33.320)),
        bounding_box: BoundingBoxXZ {
            min: XZ {
                x: -100.0,
                z: -100.0,
            },
            max: XZ { x: 100.0, z: 100.0 },
        },
        road_nodes: nodes
            .iter()
            .map(|p| util::geometry::at_height(*p, 0.0))
            .collect(),
        road_lines: roads
            .iter()
            .enumerate()
            .map(|(i, (a, b, width, kind))| SourceRoad {
                kind: *kind,
                direction: 0,
                osm_way_id: Some(i as i64 + 1),
                lanes: 2,
                ..model::source_location::new([*a, *b], *width)
            })
            .collect(),
        buildings: buildings
            .iter()
            .enumerate()
            .map(|(i, p)| model::world::Building {
                id: i as u64 + 1,
                height_meters: 6.0,
                base_y: 0.0,
                tagged: false,
                holes: Vec::new(),
                min_height_meters: 0.0,
                ground_outline: p.clone(),
            })
            .collect(),
        ..model::source_location::empty::<f64>()
    }
}
fn surfaces(
    map: &SourceLocation<f64>,
) -> (
    Temp,
    build::surfaces::Surfaces,
    world::street_surface::StreetSurface,
) {
    let temp = test::support::new();
    storage::city::write_imported(map, &temp.directory).unwrap();
    let s = build::surfaces::build_streets(map, &temp.directory, None).unwrap();
    let loaded = test::support::load_streets(
        &std::fs::read_to_string(temp.directory.join("streets.txt")).unwrap(),
        map.road_lines.len(),
    )
    .unwrap()
    .surfaces;
    (temp, s.surfaces, loaded)
}
fn point(p: XZ) -> Point<f64> {
    Point::new(p.x, p.z)
}
#[test]
fn pavement_follows_left_side_through_a_bend_and_round_end_caps() {
    let mut road = model::source_location::new([0, 1], 8.0);
    road.sidewalks.right = false;
    let path = [
        XZ { x: -30., z: 0. },
        XZ { x: 0., z: 0. },
        XZ { x: 0., z: -30. },
    ];
    let geometry = pavement_ribbon(&path, &road, &[vec![0], vec![0]], 1.4);
    for p in [
        XZ { x: -15., z: -4.7 },
        XZ { x: -4.7, z: -15. },
        XZ { x: -31., z: -4.7 },
    ] {
        assert!(
            geometry.contains(&point(p)),
            "missing left pavement at {p:?}"
        );
    }
    for p in [
        XZ { x: -15., z: 4.7 },
        XZ { x: 4.7, z: -15. },
        XZ { x: -31., z: 4.7 },
    ] {
        assert!(
            !geometry.contains(&point(p)),
            "unexpected right pavement at {p:?}"
        );
    }
}

#[test]
fn junction_without_sidewalks_has_no_generated_pavement() {
    let mut m = map(
        &[
            XZ { x: -40., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 40., z: 0. },
            XZ { x: 0., z: -40. },
        ],
        &[(0, 1, 8., 0), (1, 2, 8., 0), (1, 3, 6.5, 1)],
        &[],
    );
    for road in &mut m.road_lines {
        road.sidewalks.left = false;
        road.sidewalks.right = false;
    }
    let (_, _, compiled) = surfaces(&m);
    assert!(world::street_surface::road_triangles(&(compiled))
        .iter()
        .all(|t| !matches!(t.usage, model::world::TriangleUsage::Pavement(_))));
}

#[test]
fn junction_fillets_partition_asphalt_and_pavement_without_overlap() {
    let m = map(
        &[
            XZ { x: -40., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 40., z: 0. },
            XZ { x: 0., z: -40. },
            XZ { x: 0., z: 40. },
        ],
        &[(0, 1, 8., 0), (1, 2, 8., 0), (3, 1, 6.5, 1), (1, 4, 5.5, 2)],
        &[],
    );
    let (_temp, _, loaded) = surfaces(&m);
    let asphalt = test::support::road_footprint(&loaded);
    assert!(asphalt.contains(&point(XZ { x: 4.4, z: 3.4 })));
    let mut area = 0.;
    let mut styles = std::collections::BTreeSet::new();
    for t in world::street_surface::road_triangles(&loaded) {
        let p: Vec<XZ> = t
            .points
            .iter()
            .map(|p| XZ {
                x: p.x as f64,
                z: p.z as f64,
            })
            .collect();
        let poly = polygon(&p);
        if model::world::is_road(t.usage) {
            area += poly.unsigned_area();
        }
        if matches!(t.usage, model::world::TriangleUsage::Pavement(_)) {
            assert!(poly.intersection(&asphalt).unsigned_area() < 0.0001);
        }
    }
    assert!((area - asphalt.unsigned_area()).abs() < 0.01);
    for line in std::fs::read_to_string(_temp.directory.join("streets.txt"))
        .unwrap()
        .lines()
    {
        let mut fields = line.split_whitespace();
        if fields.next() == Some("t") {
            styles.insert(fields.next().unwrap().parse::<usize>().unwrap());
        }
    }
    assert_eq!(styles, (0..8).collect());
}
#[test]
fn t_junction_preserves_far_curb_and_fillets_avoid_buildings() {
    let m = map(
        &[
            XZ { x: -40., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 40., z: 0. },
            XZ { x: 0., z: 40. },
        ],
        &[(0, 1, 8., 0), (1, 2, 8., 0), (1, 3, 10., 1)],
        &[],
    );
    let (_, _, loaded) = surfaces(&m);
    let asphalt = test::support::road_footprint(&loaded);
    assert!(!asphalt.intersects(&point(XZ { x: 0., z: -4.5 })));
    assert!(asphalt.contains(&point(XZ { x: 0., z: -3.9 })));
    let building = vec![
        XZ { x: 4.2, z: 4.2 },
        XZ { x: 7., z: 4.2 },
        XZ { x: 7., z: 7. },
        XZ { x: 4.2, z: 7. },
    ];
    let m = map(
        &[
            XZ { x: -40., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 40., z: 0. },
            XZ { x: 0., z: -40. },
            XZ { x: 0., z: 40. },
        ],
        &[(0, 1, 8., 1), (1, 2, 8., 1), (3, 1, 8., 1), (1, 4, 8., 1)],
        &[building],
    );
    let (_, _, loaded) = surfaces(&m);
    let asphalt = test::support::road_footprint(&loaded);
    assert!(!asphalt.intersects(&point(XZ { x: 4.5, z: 4.5 })));
}
#[test]
fn parallel_roads_and_enclosed_blocks_keep_their_gaps() {
    let m = map(
        &[
            XZ { x: -30., z: 0. },
            XZ { x: 30., z: 0. },
            XZ { x: -30., z: 10. },
            XZ { x: 30., z: 10. },
        ],
        &[(0, 1, 6., 1), (2, 3, 6., 1)],
        &[],
    );
    let (_, _, loaded) = surfaces(&m);
    let asphalt = test::support::road_footprint(&loaded);
    assert_eq!(asphalt.0.len(), 2);
    assert!(!asphalt.intersects(&point(XZ { x: 0., z: 5. })));
    let m = map(
        &[
            XZ { x: -20., z: -20. },
            XZ { x: 20., z: -20. },
            XZ { x: 20., z: 20. },
            XZ { x: -20., z: 20. },
        ],
        &[(0, 1, 6., 1), (1, 2, 6., 1), (2, 3, 6., 1), (3, 0, 6., 1)],
        &[],
    );
    let (_temp, _, loaded) = surfaces(&m);
    let asphalt = test::support::road_footprint(&loaded);
    assert!(!asphalt.intersects(&point(XZ { x: 0., z: 0. })));
    for t in world::street_surface::road_triangles(&loaded)
        .iter()
        .filter(|t| model::world::is_road(t.usage))
    {
        let p: Vec<XZ> = t
            .points
            .iter()
            .map(|p| XZ {
                x: p.x as f64,
                z: p.z as f64,
            })
            .collect();
        assert!(!polygon(&p).intersects(&point(XZ { x: 0., z: 0. })));
    }
}

#[test]
fn indexed_obstacle_checks_match_merged_coverage_including_block_holes() {
    let building = vec![
        XZ { x: -3., z: -3. },
        XZ { x: 3., z: -3. },
        XZ { x: 3., z: 3. },
        XZ { x: -3., z: 3. },
    ];
    let m = map(
        &[
            XZ { x: -20., z: -20. },
            XZ { x: 20., z: -20. },
            XZ { x: 20., z: 20. },
            XZ { x: -20., z: 20. },
        ],
        &[(0, 1, 6., 1), (1, 2, 6., 1), (2, 3, 6., 1), (3, 0, 6., 1)],
        std::slice::from_ref(&building),
    );
    let (_, s, loaded) = surfaces(&m);
    let asphalt = test::support::road_footprint(&loaded);
    let index = build::surfaces::new(&s.obstacles);
    let footprint = polygon(&building);
    for x in (-40..=40).step_by(4) {
        for z in (-40..=40).step_by(4) {
            let probe = point(XZ {
                x: x as f64,
                z: z as f64,
            })
            .buffer(0.18);
            assert_eq!(
                build::surfaces::intersects(&(index), &probe),
                asphalt.intersects(&probe) || footprint.intersects(&probe),
                "mismatched collision at {x},{z}"
            );
        }
    }
}

#[test]
fn large_polygon_triangulation_preserves_area_and_holes_after_splitting() {
    let ring = |n: usize, radius: f64| {
        line(
            &(0..n)
                .map(|i| {
                    let t = i as f64 / n as f64 * std::f64::consts::TAU;
                    XZ {
                        x: radius * t.cos(),
                        z: radius * t.sin(),
                    }
                })
                .collect::<Vec<_>>(),
        )
    };
    let geometry = MultiPolygon(vec![Polygon::new(ring(5000, 100.), vec![ring(2000, 50.)])]);
    let mut area = 0.;
    build::surfaces::triangulate(&geometry, |points| {
        let p = polygon(&points);
        assert!(!p.contains(&point(XZ { x: 0., z: 0. })));
        area += p.unsigned_area();
        Ok(())
    })
    .unwrap();
    assert!(
        (area - geometry.unsigned_area()).abs() < 0.01,
        "emitted {area}, expected {}",
        geometry.unsigned_area()
    );
}

#[test]
fn disk_precision_does_not_emit_collapsed_triangles() {
    let tiny = polygon(&[
        XZ {
            x: 0.,
            z: 0.00000051,
        },
        XZ {
            x: 100.,
            z: 0.00000251,
        },
        XZ {
            x: 50.,
            z: 0.00000162,
        },
    ]);
    let normal = polygon(&[
        XZ { x: 0., z: 10. },
        XZ { x: 100., z: 10. },
        XZ { x: 50., z: 11. },
    ]);
    let count = build::surfaces::triangulate(&MultiPolygon(vec![tiny, normal]), |points| {
        assert!(
            cross(
                util::geometry::sub(points[1], points[0]),
                util::geometry::sub(points[2], points[0]),
            ) > 0.00001
        );
        Ok(())
    })
    .unwrap();
    assert_eq!(count, 1);
}
#[test]
fn bend_smoothing_shares_endpoints_and_stays_within_sixty_centimetres() {
    let m = map(
        &[
            XZ { x: -30., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 20., z: 20. },
        ],
        &[(0, 1, 6.5, 1), (1, 2, 6.5, 1)],
        &[],
    );
    let (_, s, _) = surfaces(&m);
    assert!(dist(*s.paths[0].last().unwrap(), s.paths[1][0]) < 1e-6);
    assert!(dist(s.paths[1][0], util::geometry::xz(m.road_nodes[1])) > 0.01);
    for (r, path) in m.road_lines.iter().zip(&s.paths) {
        for p in path {
            assert!(
                project(
                    &[
                        util::geometry::xz(m.road_nodes[r.ends[0]]),
                        util::geometry::xz(m.road_nodes[r.ends[1]])
                    ],
                    *p
                )
                .0 <= 0.601
            );
        }
    }
    let m = map(
        &[
            XZ { x: -30., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 30., z: 0. },
        ],
        &[(0, 1, 6.5, 1), (1, 2, 6.5, 1)],
        &[],
    );
    let (_, s, _) = surfaces(&m);
    assert_eq!(s.paths[0], vec![XZ { x: -30., z: 0. }, XZ { x: 0., z: 0. }]);
}
