//! Tests through the group's production interfaces.

mod barriers {
    use crate::model::world::Barrier;
    use crate::model::world::BarrierKind;
    use crate::test;
    use crate::util;
    use crate::util::geometry::XYZ;
    use crate::util::geometry::XZ;
    use crate::world;

    #[test]
    fn barrier_contacts_deflect_from_both_sides_and_leave_openings_clear() {
        let mut city = test::support::empty_world();
        city.barriers = world::barriers::new(vec![
            Barrier {
                source_id: 1,
                kind: BarrierKind::Wall,
                ends: [XZ { x: -20., z: 0. }, XZ { x: -2., z: 0. }],
                height_meters: 2.,
                width_meters: 0.2,
            },
            Barrier {
                source_id: 1,
                kind: BarrierKind::Wall,
                ends: [XZ { x: 2., z: 0. }, XZ { x: 20., z: 0. }],
                height_meters: 2.,
                width_meters: 0.2,
            },
        ]);
        for side in [-1., 1.] {
            let mut position = XYZ {
                x: 10.,
                y: 0.,
                z: side * 0.5,
            };
            let mut velocity = XZ {
                x: 2.,
                z: -side * 10.,
            };
            let impact = world::barriers::constrain_barriers(&city, &mut position, &mut velocity);
            assert!(impact > 9.9);
            assert!(position.z * side >= world::locations::CAR_RADIUS + 0.1);
            assert!(velocity.z * side > 0.);
            assert!(velocity.x > 0.);
        }
        let mut position = XYZ::default();
        let mut velocity = XZ { x: 0., z: 10. };
        assert_eq!(
            world::barriers::constrain_barriers(&(city), &mut position, &mut velocity),
            0.
        );
        assert_eq!(position, XYZ::default());
        assert!(
            world::barriers::barrier_clearance(&(city), util::geometry::xz(position))
                > world::locations::CAR_RADIUS
        );
        position.x = 10.;
        assert!(world::barriers::constrain_barriers(&(city), &mut position, &mut velocity) > 0.);
        assert!(util::geometry::iter(&(position)).all(|v| v.is_finite()));
    }
}

mod buildings {

    use crate::model::world::Building;
    use crate::physics;
    use crate::render;
    use crate::test;
    use crate::util;
    use crate::util::geometry::BoundingBoxXZ;
    use crate::util::geometry::XZ;
    use crate::world;
    use crate::world::buildings::*;

    fn building(id: u64, x: f32, z: f32) -> Building {
        Building {
            id,
            ground_outline: vec![
                XZ { x, z },
                XZ { x: x + 8., z },
                XZ {
                    x: x + 8.,
                    z: z + 8.,
                },
                XZ { x, z: z + 8. },
            ],
            height_meters: 6.,
            base_y: 0.,
            tagged: true,
            holes: Vec::new(),
            min_height_meters: 0.0,
        }
    }

    #[test]
    fn raised_building_parts_leave_vehicle_and_camera_clearance() {
        let city = test::support::parse_world(concat!(
            "CITY 2\nbounds -50 -50 50 50\nn 10 -30\nn 10 30\nr 0 1 8 0 1\n",
            "b 100 9 0 4 0 0 5 0 5 10 0 10\n",
            "b 100 9 0 4 5 0 15 0 15 10 5 10\nv 1 3.1 0\n",
            "b 100 9 0 4 15 0 20 0 20 10 15 10\n",
        ))
        .unwrap();
        let start = util::geometry::at_height(XZ { x: 10., z: 5. }, 0.48);
        let mut position = start;
        physics::vehicle_location::constrain(&city, &mut position, &mut XZ { x: 0., z: 4. }, 1.4);
        assert_eq!(position, start);
        assert!(building_clearance(&(city), util::geometry::xz(position)) > 1.4);
        let eye = util::geometry::at_height(XZ { x: 10., z: 15. }, 1.5);
        assert_eq!(
            render::camera::camera_eye(
                &(city),
                util::geometry::at_height(XZ { x: 10., z: -5. }, 1.5),
                eye
            ),
            eye
        );
        physics::vehicle_location::constrain(&city, &mut position, &mut XZ { x: 0., z: 4. }, 3.5);
        assert!((position.x - start.x).hypot(position.z - start.z) > 1.0);
    }

    #[test]
    fn footprint_owner_preserves_clearance_candidates_and_terrain_fitting() {
        let mut world = test::support::empty_world();
        world.buildings = new(vec![
            building(1, -36., -36.),
            building(2, 28., 28.),
            building(3, 40., 28.),
        ]);
        let field = world::elevation::generate(
            BoundingBoxXZ {
                min: XZ { x: -64., z: -64. },
                max: XZ { x: 64., z: 64. },
            },
            |p| p.x * 0.1 + p.z * 0.2,
        );
        let probes: Vec<_> = buildings(&world)
            .iter()
            .flat_map(|b| {
                let p = b.ground_outline[0];
                [
                    XZ {
                        x: p.x - 0.5,
                        z: p.z + 4.,
                    },
                    XZ {
                        x: p.x + 4.,
                        z: p.z + 4.,
                    },
                    XZ {
                        x: p.x + 8.5,
                        z: p.z + 4.,
                    },
                ]
            })
            .collect();
        for p in &probes {
            let expected = buildings(&world)
                .iter()
                .map(|b| nearest(b, *p).2 * if contains(b, *p) { -1. } else { 1. })
                .fold(f32::INFINITY, f32::min);
            assert_eq!(building_clearance(&(world), *p), expected, "{p:?}");
        }
        let before: Vec<_> = probes
            .iter()
            .map(|&p| {
                nearby_buildings(&world, p)
                    .map(|b| b.id)
                    .collect::<Vec<_>>()
            })
            .collect();
        world.buildings = with_terrain(world.buildings, &field);
        let after: Vec<_> = probes
            .iter()
            .map(|&p| {
                nearby_buildings(&world, p)
                    .map(|b| b.id)
                    .collect::<Vec<_>>()
            })
            .collect();
        assert_eq!(before, after);
        for b in buildings(&world) {
            let max_height = b
                .ground_outline
                .iter()
                .map(|&p| world::elevation::height(&field, p))
                .fold(f32::NEG_INFINITY, f32::max);
            assert!((b.base_y - max_height - 0.02).abs() < 0.0001);
        }
        // Replacing the owner discards every candidate from the previous geometry.
        world.buildings = new(vec![building(4, 128., 128.)]);
        assert!(nearby_buildings(&(world), probes[0]).next().is_none());
        assert!(building_clearance(&(world), probes[0]).is_infinite());
    }
}

mod elevation {

    use crate::model::world::HeightField;
    use crate::util;
    use crate::util::geometry::XYZ;
    use crate::util::geometry::XZ;
    use crate::world::elevation::*;

    fn grid() -> HeightField {
        HeightField {
            columns: 3,
            rows: 3,
            origin: XZ { x: 0.0, z: 0.0 },
            step: 10.0,
            values: vec![0.0, 2.0, 4.0, 1.0, 6.0, 5.0, 2.0, 4.0, 8.0],
        }
    }
    #[test]
    fn sampling_and_draping_use_the_same_planes_across_cells() {
        let grid = grid();
        assert_eq!(height(&(grid), XZ { x: 10.0, z: 10.0 }), 6.0);
        let points = [
            XZ { x: 1.0, z: 1.0 },
            XZ { x: 19.0, z: 3.0 },
            XZ { x: 3.0, z: 19.0 },
        ];
        let mut area = 0.0;
        drape(&grid, points, |tri| {
            let a = tri[0];
            let b = tri[1];
            let c = tri[2];
            area += ((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)).abs() * 0.5;
            let center: XYZ = util::geometry::xyz_from_fn(|i| {
                (util::geometry::xyz_component(a, i)
                    + util::geometry::xyz_component(b, i)
                    + util::geometry::xyz_component(c, i))
                    / 3.0
            });
            assert!(
                (center.y
                    - height(
                        &(grid),
                        XZ {
                            x: center.x,
                            z: center.z
                        }
                    ))
                .abs()
                    < 0.0001
            );
        });
        assert!((area - 160.0).abs() < 0.001);
        let points = segment(&grid, XZ { x: 1.0, z: 2.0 }, XZ { x: 19.0, z: 14.0 });
        assert!(points.len() > 3);
        for pair in points.windows(2) {
            let p: XYZ = util::geometry::xyz_from_fn(|i| {
                (util::geometry::xyz_component(pair[0], i)
                    + util::geometry::xyz_component(pair[1], i))
                    * 0.5
            });
            assert!((p.y - height(&(grid), XZ { x: p.x, z: p.z })).abs() < 0.0001);
        }
    }
}

mod landscape_index {
    use crate::model;
    use crate::model::world::Material;
    use crate::model::world::Tree;
    use crate::model::world::Triangle;
    use crate::util;
    use crate::util::geometry::XZ;
    use crate::world;
    use crate::world::landscape::*;
    use geo::{Intersects, Point};

    #[test]
    fn imagery_crowns_replace_procedural_fill_but_keep_mapped_tree_nodes() {
        let tree = |id, x| Tree {
            id,
            coords: XZ { x, z: 0. },
            height_meters: 6.,
            crown_radius_meters: 1.,
        };
        let mut trees = vec![
            tree(42, 0.),
            tree((1 << 62) | 1, 0.5),
            tree((1 << 62) | 2, 10.),
        ];
        append_imagery_trees(&mut trees, vec![tree((1 << 63) | 3, 1.)]).unwrap();
        assert_eq!(
            trees.iter().map(|t| t.id).collect::<Vec<_>>(),
            vec![42, (1 << 62) | 2, (1 << 63) | 3]
        );
        assert!(append_imagery_trees(&mut trees, vec![tree((1 << 63) | 3, 1.)]).is_err());
    }
    #[test]
    fn ground_owner_matches_full_triangle_queries_across_negative_and_positive_cells() {
        let triangles = vec![
            Triangle {
                material: Material::Park,
                usage: model::world::TriangleUsage::Ground,
                points: [
                    XZ { x: -64., z: -64. },
                    XZ { x: 64., z: -64. },
                    XZ { x: -64., z: 64. },
                ],
            },
            Triangle {
                material: Material::Paving,
                usage: model::world::TriangleUsage::Ground,
                points: [
                    XZ { x: 64., z: -64. },
                    XZ { x: 64., z: 64. },
                    XZ { x: -64., z: 64. },
                ],
            },
        ];
        let cover = world::landscape::new(triangles, vec![]);
        for x in (-80..=80).step_by(3) {
            for z in (-80..=80).step_by(5) {
                let p = XZ {
                    x: x as f32 + 0.13,
                    z: z as f32 + 0.37,
                };
                let expected = world::landscape::triangles(&cover)
                    .iter()
                    .find(|t| {
                        util::geometry::polygon(
                            &t.points.map(|p| util::geometry::xz_map(p, f64::from)),
                        )
                        .intersects(&Point::new(p.x as f64, p.z as f64))
                    })
                    .map(|t| t.material);
                assert_eq!(world::landscape::material(&(cover), p), expected, "{p:?}");
            }
        }
        let empty = world::landscape::new(vec![], vec![]);
        assert_eq!(
            world::landscape::material(&(empty), XZ { x: 0., z: 0. }),
            None
        );
        assert_eq!(
            world::landscape::material(&(cover), XZ { x: 0.1, z: 0.1 }),
            Some(Material::Paving)
        );
    }
}

mod mapped_objects {
    use crate::model;
    use crate::model::map_details::MapObject;
    use crate::model::map_details::ObjectKind;
    use crate::test;
    use crate::util::geometry::XYZ;
    use crate::util::geometry::XZ;
    use crate::world;

    #[test]
    fn open_gates_leave_the_driving_gap_clear_and_locked_gates_stop_a_car() {
        let mut city = test::support::empty_world();
        for locked in [false, true] {
            let mut data = model::map_details::Data::default();
            data.objects.push(MapObject {
                kind: ObjectKind::Gate,
                position: [0., 0.],
                angle: 0.,
                height: 1.4,
                width: 6.,
                text: String::new(),
                locked,
            });
            city.details = world::map_details::new(data);
            let mut p = XYZ {
                x: 0.,
                y: 0.48,
                z: 0.4,
            };
            let mut velocity = XZ { x: 0., z: -5. };
            let impact =
                world::mapped_objects::constrain_mapped_objects(&city, &mut p, &mut velocity);
            assert_eq!(impact > 0., locked);
            assert_eq!(
                world::mapped_objects::mapped_object_clearance(&(city), XZ { x: 0., z: 0. })
                    < world::locations::CAR_RADIUS,
                locked
            );
            let mut above = XYZ {
                x: 0.,
                y: 5.48,
                z: 0.4,
            };
            let before = above;
            assert_eq!(
                world::mapped_objects::constrain_mapped_objects(&(city), &mut above, &mut velocity),
                0.
            );
            assert_eq!(above, before);
        }
    }
}

mod roads {

    use crate::test;
    use crate::util;
    use crate::util::geometry::BoundingBoxXZ;
    use crate::util::geometry::XZ;
    use crate::world;
    use crate::world::roads::*;

    #[test]
    fn prepared_road_grids_match_full_projection_for_centrelines_and_compiled_paths() {
        for compiled in [false, true] {
            let mut world = test::support::parse_world(concat!(
                "CITY 2\nbounds -100 -100 100 100\n",
                "n -90 0\nn 90 0\nn -90 70\nn 90 70\n",
                "r 0 1 8 0 1\nr 2 3 6.5 0 2\n",
            ))
            .unwrap();
            if compiled {
                let text = concat!(
                    "STREETS 1 2\n",
                    "p 0 3 -90 0 0 20 90 0\np 1 3 -90 70 0 50 90 70\n",
                    "t 1 -90 -4 90 -4 90 4\nc 1 -90 -4 90 -4 0 1\n"
                );
                let (surfaces, paths) = world::street_surface::load(text, 2).unwrap();
                let field =
                    world::elevation::generate(world.bounding_box, |p| p.x * 0.01 + p.z * 0.02);
                world.roads = with_compiled(world.roads, surfaces, paths, &field).unwrap();
                world.elevation = Some(field);
                assert_eq!(road_lines(&(world))[0].path.len(), 3);
            }
            for x in (-200..=200).step_by(13) {
                for z in (-200..=200).step_by(17) {
                    let point = util::geometry::at_height(
                        XZ {
                            x: x as f32 + 0.13,
                            z: z as f32 + 0.37,
                        },
                        0.,
                    );
                    let (expected_index, (_, expected)) = (0..road_lines(&world).len())
                        .map(|i| (i, projected(&world, i, point)))
                        .min_by(|a, b| a.1 .0.total_cmp(&b.1 .0).then(a.0.cmp(&b.0)))
                        .unwrap();
                    let (index, actual) = nearest_road(&world, point);
                    assert_eq!(
                        index, expected_index,
                        "compiled={compiled}, point={point:?}"
                    );
                    assert_eq!(actual.point, expected.point);
                    assert_eq!(actual.offset, expected.offset);
                    assert_eq!(actual.distance, expected.distance);
                    assert_eq!(actual.width, expected.width);
                }
            }
            // Minimap candidates retain stored path order and compiled bends.
            let segments = road_segments_in(&world, world.bounding_box);
            let all: Vec<_> = road_lines(&world)
                .iter()
                .flat_map(|r| r.path.windows(2).map(|p| (p[0], p[1])))
                .collect();
            assert_eq!(segments, all);
            assert!(road_segments_in(
                &(world),
                BoundingBoxXZ {
                    min: XZ { x: 1000., z: 1000. },
                    max: XZ { x: 1001., z: 1001. }
                }
            )
            .is_empty());
        }
    }
}

mod transport {
    use crate::model;
    use crate::model::map_details::RoadSurface;
    use crate::physics;
    use crate::render;
    use crate::test;
    use crate::util::geometry::XYZ;
    use crate::util::geometry::XZ;
    use crate::world;

    #[test]
    fn tunnels_pass_beneath_ground_buildings_without_invisible_walls() {
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), true);
        let mut city = world::preparation::world_location_load(&assets).unwrap();
        let mut data = model::map_details::Data::default();
        data.decks.push(model::map_details::Deck {
            road: 0,
            grade: -1,
            width: 8.,
            path: vec![[0., 2., 12.], [20., 2., 12.]],
            surface: RoadSurface::Asphalt,
        });
        city.details = world::map_details::new(data);
        let mut position = XYZ {
            x: 12.,
            y: 2.48,
            z: 12.,
        };
        let original = position;
        let mut velocity = XZ { x: 4., z: 0. };
        assert!(world::buildings::building_clearance_at(&(city), position) > 10.);
        assert_eq!(
            physics::vehicle_location::constrain(&(city), &mut position, &mut velocity, 1.7),
            0.
        );
        assert!((position.y - original.y).abs() < 0.001);
        assert_eq!(position.x, original.x);
        assert_eq!(position.z, original.z);
        let anchor = XYZ {
            x: 13.,
            y: 3.2,
            z: 12.,
        };
        let eye = XYZ {
            x: 12.,
            y: 3.8,
            z: 12.,
        };
        assert_eq!(render::camera::camera_eye(&(city), anchor, eye), eye);
    }
    #[test]
    fn recovery_keeps_the_drivers_level_at_an_overlapping_road() {
        use crate::model;
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), false);
        let mut stored = model::location::location_read(folder.path()).unwrap();
        stored.roads.push(model::location::Road {
            ends: stored.roads[0].ends,
            way_id: Some(2),
            width_meters: 8.,
            direction: 0,
        });
        stored.streets.paths.push(stored.streets.paths[0].clone());
        stored.details.decks.push(model::map_details::Deck {
            road: 0,
            grade: 1,
            width: 8.,
            path: vec![[0., 12., 0.], [10., 12., 1.], [20., 12., 0.]],
            surface: RoadSurface::Asphalt,
        });
        model::location::location_write(&stored, folder.path()).unwrap();
        let city = world::preparation::world_location_load(&assets).unwrap();
        for height in [7., 12.] {
            let p = XYZ {
                x: 10.,
                y: height + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 1.,
            };
            assert!((world::roads::project(&(city), p).point.y - height).abs() < 0.001);
            assert!(
                (physics::vehicle_location::recovery(&(city), p).point.y - height).abs() < 0.001
            );
        }
    }
    #[test]
    fn stacked_support_rails_and_camera_follow_the_selected_road_level() {
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), false);
        for grade in [-1, 1] {
            let mut city = world::preparation::world_location_load(&assets).unwrap();
            city.elevation = Some(world::elevation::generate(
                model::world::bounding_box(&city),
                |_| 0.,
            ));
            let mut data = model::map_details::Data::default();
            data.decks.push(model::map_details::Deck {
                road: 0,
                grade,
                width: 8.,
                path: vec![[0., grade as f32 * 5., 0.], [20., grade as f32 * 5., 0.]],
                surface: RoadSurface::Asphalt,
            });
            city.details = world::map_details::new(data);
            let p = XZ { x: 10., z: 0. };
            assert!(world::transport::drive_height_near(&(city), p, 0.).abs() < 0.001);
            assert!(
                (world::transport::drive_height_near(&(city), p, grade as f32 * 5.)
                    - grade as f32 * 5.)
                    .abs()
                    < 0.001
            );
            assert!(
                (world::roads::sample(&(city), 0, 0.5).point.y - grade as f32 * 5.).abs() < 0.001
            );
            let mut position = XYZ {
                x: 10.,
                y: grade as f32 * 5. + physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 3.7,
            };
            let mut velocity = XZ { x: 3., z: 5. };
            assert!(
                world::transport::constrain_transport(&(city), &mut position, &mut velocity) > 4.9
            );
            assert!(position.z < 3.);
            let mut ground = XYZ {
                x: 10.,
                y: physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                z: 3.7,
            };
            let original = ground;
            assert_eq!(
                world::transport::constrain_transport(&(city), &mut ground, &mut velocity),
                0.
            );
            assert_eq!(ground, original);
            if grade < 0 {
                let anchor = XYZ {
                    x: 10.,
                    y: -3.8,
                    z: 0.,
                };
                let eye = XYZ {
                    x: 5.,
                    y: -3.0,
                    z: 0.,
                };
                assert_eq!(render::camera::camera_eye(&(city), anchor, eye), eye);
                assert!(render::camera::camera_eye(&(city), anchor, XYZ { y: 0., ..eye }).y < -0.5);
            }
        }
    }
}

mod locations {

    use crate::model::world::LocationId;
    use crate::physics;
    use crate::test;
    use crate::world;

    fn variable_width_track() -> std::sync::Arc<world::locations::WorldLocation> {
        let file = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(file.path(), r#"{"version":2,"name":"Variable width","scenery":"coastal","points":[[0,0,0,4],[400,0,0,12],[400,0,400,6],[0,0,400,8]]}"#).unwrap();
        world::locations::try_track(&world::locations::register_file(file.path()).unwrap().0)
            .unwrap()
    }

    #[test]
    fn point_widths_interpolate_and_agree_with_projection_including_the_seam() {
        let track = variable_width_track();
        for (point, width) in [
            ([0.0, 0.0, 0.0], 4.0),
            ([400.0, 0.0, 0.0], 12.0),
            ([400.0, 0.0, 400.0], 6.0),
            ([0.0, 0.0, 400.0], 8.0),
            ([200.0, 0.0, -50.0], 8.0),
        ] {
            assert!((world::roads::project(&track, (point).into()).width - width).abs() < 0.001);
        }
        for i in 0..1000 {
            let sample = world::locations::at_distance(
                &track,
                world::locations::sampling_length(&track) * i as f32 / 1000.0,
            );
            assert!((4.0..=12.0).contains(&sample.width));
            let projected = world::roads::project(&track, sample.point);
            assert!((sample.width - projected.width).abs() < 0.001);
        }
        assert_eq!(world::locations::at_distance(&track, 0.0).width, 4.0);
        assert_eq!(
            world::locations::at_distance(&track, world::locations::sampling_length(&track)).width,
            4.0
        );
        assert!(
            (world::locations::at_distance(
                &track,
                world::locations::sampling_length(&track) - 0.01
            )
            .width
                - 4.0)
                .abs()
                < 0.001
        );
    }

    #[test]
    fn per_point_widths_are_required_and_bounded() {
        for width in [
            serde_json::json!(1.99),
            serde_json::json!(30.01),
            serde_json::Value::Null,
            serde_json::json!("wide"),
        ] {
            let file = tempfile::NamedTempFile::new().unwrap();
            let data = serde_json::json!({"version":2,"name":"Invalid width","points":[[0,0,0,width],[100,0,0,8],[100,0,100,8],[0,0,100,8]]});
            std::fs::write(file.path(), serde_json::to_vec(&data).unwrap()).unwrap();
            assert!(world::locations::register_file(file.path()).is_err());
        }
        for data in [
            r#"{"version":1,"name":"Old format","width":8,"points":[[0,0,0],[100,0,0],[100,0,100],[0,0,100]]}"#,
            r#"{"version":2,"name":"Missing widths","points":[[0,0,0],[100,0,0],[100,0,100],[0,0,100]]}"#,
            r#"{"version":2,"name":"Global width","width":8,"points":[[0,0,0,8],[100,0,0,8],[100,0,100,8],[0,0,100,8]]}"#,
        ] {
            let file = tempfile::NamedTempFile::new().unwrap();
            std::fs::write(file.path(), data).unwrap();
            assert!(world::locations::register_file(file.path()).is_err());
        }
    }

    #[test]
    fn runtime_circuit_rejects_invalid_definitions() {
        for data in [
            r#"{"version":2}"#,
            r#"{"version":2,"name":"bad","scenery":"unknown"}"#,
            r#"{"version":2,"name":"bad","scenery":null}"#,
            r#"{"version":2,"name":"bad","points":[],"width":"wide"}"#,
            r#"{"version":2,"name":"bad","points":[]}"#,
            r#"{"version":2,"name":"bad","points":[[0,0,0,7.2],[1,0,0,7.2],[1,0,1,7.2]]}"#,
            r#"{"version":2,"name":"bad","points":[[0,0,0,7.2],[0,1,0,7.2],[1,0,1,7.2],[2,0,2,7.2]]}"#,
            r#"{"version":2,"name":"bad","points":[[0,0,0,7.2],[20000,0,0,7.2],[20000,0,20000,7.2],[0,0,20000,7.2]]}"#,
        ] {
            let file = tempfile::NamedTempFile::new().unwrap();
            std::fs::write(file.path(), data).unwrap();
            assert!(world::locations::register_file(file.path()).is_err());
        }
    }

    #[test]
    fn spatial_projection_matches_full_search_and_route_caches_are_isolated() {
        let scenic = world::locations::try_track(&test::support::authored_id()).unwrap();
        for i in 0..400 {
            let s = world::locations::at_distance(
                &scenic,
                world::locations::sampling_length(&scenic) * i as f32 / 400.0,
            );
            for offset in [-45.0, -3.0, 0.0, 3.0, 45.0] {
                let p = physics::vehicle_location::vehicle_position(s, offset);
                let indexed = world::roads::project(&scenic, p);
                let exact = (0..world::roads::road_lines(&scenic).len())
                    .map(|i| world::roads::projected(&scenic, i, p))
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .unwrap()
                    .1;
                assert!(
                    (indexed.distance - exact.distance).abs() < 0.001,
                    "point {p:?}: indexed {indexed:?}, brute force {exact:?}"
                );
                assert!((indexed.point.y - exact.point.y).abs() < 0.001);
            }
        }
        let compact_length = world::locations::sampling_length(
            &world::locations::try_track(&LocationId::Compact).unwrap(),
        );
        assert!(world::locations::sampling_length(&scenic) > compact_length * 7.0);
        assert_eq!(
            world::locations::sampling_length(
                &world::locations::try_track(&(LocationId::Compact)).unwrap()
            ),
            compact_length
        );
    }
}

mod street_surface_index {
    use crate::model;
    use crate::storage;
    use crate::test;
    use crate::util;
    use crate::util::geometry::XZ;
    use crate::world;
    use geo::{Intersects, Point};

    #[test]
    fn mapped_road_materials_round_trip_through_the_binary_surface_cache() {
        let folder = tempfile::tempdir().unwrap();
        for id in 8..=16 {
            let text = format!(
                "STREETS 1 1\np 0 2 0 0 20 0\nt {id} 0 -4 20 -4 20 4\nc 1 0 -4 20 -4 0 1\n"
            );
            let (surface, paths) = world::street_surface::load(&text, 1).unwrap();
            storage::street_surface_binary::write_binary_street_surface(
                &folder.path().join("streets.bin"),
                &surface,
                &paths,
            )
            .unwrap();
            let stored = storage::street_surface_binary::load_geometry(folder.path(), 1).unwrap();
            assert_eq!(
                stored.triangles,
                world::street_surface::road_triangles(&(surface))
            );
            assert_eq!(
                storage::codec_streets::triangle_street_id(&(stored.triangles[0])).unwrap(),
                id
            );
        }
    }
    #[test]
    fn surface_owner_matches_full_triangle_and_curb_scans_at_grid_boundaries() {
        let text = concat!(
            "STREETS 1 1\np 0 2 -64 0 64 0\n",
            "t 1 -64 -4 64 -4 64 4\nt 1 -64 -4 64 4 -64 4\n",
            "t 6 -64 4 64 4 64 8\nt 6 -64 4 64 8 -64 8\n",
            "c 1 -64 4 64 4 0 -1\nc 1 -64 -4 64 -4 0 1\n"
        );
        let (surface, _) = world::street_surface::load(text, 1).unwrap();
        let mut world = test::support::empty_world();
        test::support::install_surfaces(&mut world, surface);
        let surface = world::roads::compiled_surfaces(&world).unwrap();
        for x in (-100..=100).step_by(7) {
            for z in (-60..=60).step_by(3) {
                let p = XZ {
                    x: x as f32 + 0.13,
                    z: z as f32 + 0.37,
                };
                for road_only in [false, true] {
                    let expected = world::street_surface::road_triangles(&surface)
                        .iter()
                        .find(|t| {
                            (!road_only || model::world::is_road(t.usage))
                                && util::geometry::polygon(
                                    &t.points.map(|p| util::geometry::xz_map(p, f64::from)),
                                )
                                .intersects(&Point::new(p.x as f64, p.z as f64))
                        })
                        .map(|t| t.usage);
                    assert_eq!(
                        world::street_surface::world_location_triangle_at(&world, p, road_only)
                            .map(|t| t.usage),
                        expected,
                        "{p:?}"
                    );
                }
                let expected = world::street_surface::curbs(&surface)
                    .iter()
                    .map(|c| {
                        let q = world::spatial::nearest_edge(p, c.ends[0], c.ends[1]);
                        (q, c.inward, (q.x - p.x).hypot(q.z - p.z))
                    })
                    .min_by(|a, b| a.2.total_cmp(&b.2))
                    .unwrap();
                assert_eq!(
                    world::street_surface::world_location_nearest_curb(&world, p),
                    expected,
                    "{p:?}"
                );
            }
        }
    }
}
