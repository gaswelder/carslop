//! Tests through the group's production interfaces.

mod building_parts {

    use crate::render;
    use crate::render::building_parts::*;
    use crate::render::scene::Builder;
    use crate::util::geometry::XYZ;
    use crate::util::geometry::XZ;
    use glam::Vec3;

    fn tints() -> Tints {
        Tints {
            wall: [0.95; 3],
            frame: [0.4; 3],
            glass: [0.2, 0.3, 0.4],
            trim: [0.9; 3],
            shutter: [0.3; 3],
        }
    }
    #[test]
    fn model_placement_preserves_normals_winding_tints_and_fixed_scale_axes() {
        let catalog = render::building_parts::load_default().unwrap();
        let mut b = Builder::default();
        // This facade basis reflects the model; nonuniform scale and rotated
        // placement must preserve outward normals and triangle winding.
        render::building_parts::place(
            &catalog.window,
            &mut b,
            [XZ { x: 10., z: 20. }, XZ { x: 10., z: 30. }],
            5.,
            7.,
            XYZ {
                x: 2.,
                y: 3.,
                z: 100.,
            },
            &tints(),
        );
        assert_eq!(b.vertices.len(), 24);
        for tri in b.vertices.chunks_exact(3) {
            let p: [Vec3; 3] = std::array::from_fn(|i| Vec3::from_array(tri[i].position));
            let face = (p[1] - p[0]).cross(p[2] - p[0]).normalize();
            for v in tri {
                let n = Vec3::from_array(v.normal);
                assert!((n.length() - 1.).abs() < 0.00001);
                assert!(n.dot(face) > 0.9999);
                assert!(
                    (10.02..10.06).contains(&v.position[0]),
                    "depth must remain unscaled"
                );
                assert!((7.0..=10.0).contains(&v.position[1]));
                assert!((24.0..=26.0).contains(&v.position[2]));
            }
        }
        assert!(b
            .vertices
            .iter()
            .any(|v| v.color == tints().glass && v.material == [0.0, 0.45]));
        let mut b = Builder::default();
        render::building_parts::place(
            &catalog.entrance,
            &mut b,
            [XZ { x: 0., z: 0. }, XZ { x: 10., z: 0. }],
            5.,
            0.,
            XYZ {
                x: 100.,
                y: 100.,
                z: 100.,
            },
            &tints(),
        );
        let width = b
            .vertices
            .iter()
            .map(|v| v.position[0])
            .fold(f32::NEG_INFINITY, f32::max)
            - b.vertices
                .iter()
                .map(|v| v.position[0])
                .fold(f32::INFINITY, f32::min);
        let height = b.vertices.iter().map(|v| v.position[1]).fold(0., f32::max);
        assert!((width - 1.24).abs() < 0.00001);
        assert_eq!(height, 2.25);
    }
}

mod barriers {
    use crate::model;
    use crate::model::world::BarrierKind;
    use crate::render::barriers::*;
    use crate::render::scene::Builder;
    use crate::storage;
    use crate::test;
    use crate::util::geometry::XZ;
    use crate::world;

    #[test]
    fn barrier_meshes_follow_terrain_and_keep_near_and_far_height() {
        for kind in [
            BarrierKind::Wall,
            BarrierKind::Fence,
            BarrierKind::ChainLink,
            BarrierKind::WoodFence,
        ] {
            let folder = tempfile::tempdir().unwrap();
            let assets = test::support::stored_location(folder.path(), false);
            let barriers = vec![model::world::Barrier {
                source_id: 1,
                kind,
                ends: [XZ { x: -5., z: 10. }, XZ { x: 20., z: 10. }],
                height_meters: 2.,
                width_meters: 0.12,
            }];
            storage::furniture::write(
                &mut std::fs::File::create(folder.path().join("furniture.txt")).unwrap(),
                &[],
                &barriers,
            )
            .unwrap();
            let mut city = world::preparation::world_location_load(&assets).unwrap();
            let field =
                world::elevation::generate(model::world::bounding_box(&city), |p| 7.0 + p.x * 0.1);
            test::support::set_elevation(&mut city, Some(field));
            for detailed in [false, true] {
                let mut builder = Builder::default();
                build(&mut builder, &city, detailed);
                assert!(!builder.vertices.is_empty());
                let mut max_height: f32 = 0.;
                for vertex in &builder.vertices {
                    assert!(vertex
                        .position
                        .iter()
                        .chain(vertex.normal.iter())
                        .all(|v| v.is_finite()));
                    assert!((vertex.normal.iter().map(|v| v * v).sum::<f32>() - 1.0).abs() < 0.001);
                    let point = XZ {
                        x: vertex.position[0],
                        z: vertex.position[2],
                    };
                    let height = vertex.position[1] - world::elevation::world_height(&city, point);
                    assert!(
                        (-0.03..=2.03).contains(&height),
                        "height {height} for {kind:?}"
                    );
                    max_height = max_height.max(height);
                }
                assert!((max_height - 2.0).abs() < 0.03);
            }
        }
    }
}

mod roofs {

    use crate::model::map_details::BuildingStyle;
    use crate::model::map_details::RoofShape;
    use crate::model::world::Building;
    use crate::render;
    use crate::render::roofs::*;
    use crate::render::scene::Builder;
    use crate::util::geometry::XZ;

    #[test]
    fn roof_profiles_keep_total_height_and_courtyard_openings() {
        let building = Building {
            id: 1,
            ground_outline: vec![
                XZ { x: 0., z: 0. },
                XZ { x: 20., z: 0. },
                XZ { x: 20., z: 10. },
                XZ { x: 0., z: 10. },
            ],
            holes: vec![vec![
                XZ { x: 8., z: 3. },
                XZ { x: 8., z: 7. },
                XZ { x: 12., z: 7. },
                XZ { x: 12., z: 3. },
            ]],
            min_height_meters: 0.,
            height_meters: 10.,
            base_y: 2.,
            tagged: true,
        };
        for shape in [
            RoofShape::Gabled,
            RoofShape::Hipped,
            RoofShape::Pyramidal,
            RoofShape::Skillion,
            RoofShape::Dome,
        ] {
            let style = BuildingStyle {
                volume: 0,
                shape,
                roof_height: 3.,
                angle: 0.,
                wall_color: None,
                roof_color: None,
            };
            let mut builder = Builder::default();
            build(
                &mut builder,
                &building,
                &style,
                render::scene::matte([0.8, 0.2, 0.2]),
                render::scene::matte([0.5; 3]),
            );
            assert!(!builder.vertices.is_empty());
            let max = builder
                .vertices
                .iter()
                .map(|v| v.position[1])
                .fold(f32::NEG_INFINITY, f32::max);
            assert!(max <= 12.001 && max > 11., "{shape:?}: {max}");
            for triangle in builder.vertices.chunks_exact(3) {
                let center = XZ {
                    x: triangle.iter().map(|v| v.position[0]).sum::<f32>() / 3.,
                    z: triangle.iter().map(|v| v.position[2]).sum::<f32>() / 3.,
                };
                assert!(
                    !(center.x > 8.001
                        && center.x < 11.999
                        && center.z > 3.001
                        && center.z < 6.999),
                    "roof filled courtyard: {shape:?} {center:?}"
                );
            }
        }
    }
}

mod scene_archive {
    use crate::render;
    use crate::render::scene::Vertex;
    use crate::render::scene_archive::*;
    use crate::test;
    use crate::util::geometry::XYZ;
    use std::path::Path;
    use std::path::PathBuf;
    struct TestFile(PathBuf);

    fn new() -> TestFile {
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        TestFile(std::env::temp_dir().join(format!(
            "cars-scene-{}-{}.pack",
            std::process::id(),
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        )))
    }

    impl Drop for TestFile {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    #[test]
    fn spooled_archive_preserves_geometry_across_flushes_and_sparse_layers() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("scene.pack");
        let mut builder = render::chunks::spooled(directory.path()).unwrap();
        for i in 0..90_000 {
            let mut tri: [Vertex; 3] = triangle().try_into().unwrap();
            for v in &mut tri {
                v.position[0] += (i % 3) as f32 * 512.0;
                v.color[0] = (i % 251) as f32 / 255.0;
            }
            render::chunks::push(&mut builder, tri);
        }
        let mut layers = vec![
            render::chunks::finish_spooled(builder).unwrap(),
            render::scene_archive::new(directory.path()).unwrap(),
            render::scene_archive::new(directory.path()).unwrap(),
        ];
        for chunk in test::support::partition(triangle()) {
            render::scene_archive::append(&mut layers[2], chunk).unwrap();
        }
        write_spooled(&path, &mut layers).unwrap();
        let mut archive = render::scene_archive::open(&path).unwrap();
        assert_eq!(archive.entries.len(), 3);
        for cell in 0..3 {
            let offset = cell as f32 * 512.0;
            assert_eq!(
                archive.entries[cell].bounds,
                [
                    XYZ {
                        x: 255.75 + offset,
                        y: 4.125,
                        z: -1234.5
                    },
                    XYZ {
                        x: 257.0 + offset,
                        y: 7.0,
                        z: -1234.5
                    }
                ]
            );
            let loaded = render::scene_archive::read_layer(&mut archive, cell, 0).unwrap();
            assert_eq!(loaded.len(), 90_000);
            for (j, tri) in loaded.chunks_exact(3).enumerate() {
                let i = j * 3 + cell;
                for (mut expected, actual) in triangle().into_iter().zip(tri) {
                    expected.position[0] += offset;
                    expected.color[0] = (i % 251) as f32 / 255.0;
                    assert_vertex_matches(expected, actual);
                }
            }
            assert!(render::scene_archive::read_layer(&mut (archive), cell, 1)
                .unwrap()
                .is_empty());
            let detail = render::scene_archive::read_layer(&mut archive, cell, 2).unwrap();
            assert_eq!(detail.len(), if cell == 0 { 3 } else { 0 });
            for (expected, actual) in triangle().into_iter().zip(&detail) {
                assert_vertex_matches(expected, actual);
            }
        }
    }

    fn assert_vertex_matches(expected: Vertex, actual: &Vertex) {
        assert_eq!(expected.position, actual.position);
        assert_eq!(expected.material, actual.material);
        for axis in 0..3 {
            assert!((expected.normal[axis] - actual.normal[axis]).abs() < 0.00004);
            assert!((expected.color[axis] - actual.color[axis]).abs() < 1.0 / 255.0);
        }
    }

    #[test]
    fn failed_spooled_replacement_preserves_the_working_archive() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("scene.pack");
        fixture(&path);
        let before = std::fs::read(&path).unwrap();
        let mut empty: Vec<_> = (0..3)
            .map(|_| render::scene_archive::new(directory.path()).unwrap())
            .collect();
        assert!(write_spooled(&path, &mut empty).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), before);
    }

    fn triangle() -> Vec<Vertex> {
        [
            [255.75, 4.125, -1234.5],
            [257.0, 4.125, -1234.5],
            [256.0, 7.0, -1234.5],
        ]
        .map(|position| Vertex {
            position,
            normal: [0.6, 0.8, 0.0],
            color: [0.18, 0.2, 0.22],
            material: [-2.0, 102.25],
        })
        .to_vec()
    }
    fn fixture(path: &Path) {
        write(
            path,
            std::array::from_fn(|_| test::support::partition(triangle())),
        )
        .unwrap();
    }
    #[test]
    fn archive_loads_only_index_then_requested_layer_and_preserves_geometry() {
        let path = new();
        fixture(&path.0);
        let mut archive = render::scene_archive::open(&path.0).unwrap();
        assert_eq!(archive.peak_decode_bytes, 0);
        let index_bytes = archive.bytes_read;
        let total_bytes = std::fs::metadata(&path.0).unwrap().len() as usize;
        assert!(index_bytes > 0);
        assert!((archive.bytes_read as u64) < std::fs::metadata(&path.0).unwrap().len());
        let loaded = render::scene_archive::read_layer(&mut archive, 0, 0).unwrap();
        assert_eq!(
            archive.bytes_read - index_bytes,
            (total_bytes - index_bytes) / 3
        );
        for (a, b) in triangle().iter().zip(loaded) {
            assert_eq!(a.position, b.position);
            assert_eq!(a.material, b.material);
            for axis in 0..3 {
                assert!((a.normal[axis] - b.normal[axis]).abs() < 0.00004);
                assert!((a.color[axis] - b.color[axis]).abs() < 1.0 / 255.0);
            }
        }
        assert!(render::scene_archive::read_layer(&mut (archive), 1, 0).is_err());
        assert!(render::scene_archive::read_layer(&mut (archive), 0, 3).is_err());
    }
}

mod chunks {

    use crate::render::chunks::*;
    use crate::render::scene::Vertex;
    use crate::test;
    use crate::util;
    use crate::util::geometry::XYZ;

    #[test]
    fn partition_keeps_triangles_whole_and_bounds_include_vertices_across_seams() {
        let tri = |x: f32| {
            [[x - 10.0, 0.0, 0.0], [x + 10.0, 0.0, 0.0], [x, 3.0, 5.0]].map(|position| Vertex {
                position,
                color: [1.0; 3],
                normal: [0.0, 1.0, 0.0],
                material: [0.0; 2],
            })
        };
        let vertices = [tri(-256.0), tri(256.0), tri(900.0)].concat();
        let chunks = test::support::partition(vertices.clone());
        assert_eq!(
            chunks.iter().map(|c| c.vertices.len()).sum::<usize>(),
            vertices.len()
        );
        assert_eq!(chunks.len(), 3);
        for chunk in &chunks {
            assert_eq!(chunk.vertices.len() % 3, 0);
            for v in &chunk.vertices {
                for axis in 0..3 {
                    assert!(
                        v.position[axis] >= util::geometry::xyz_component(chunk.bounds[0], axis)
                            && v.position[axis]
                                <= util::geometry::xyz_component(chunk.bounds[1], axis)
                    );
                }
            }
        }
        assert!(chunks
            .iter()
            .any(|c| c.bounds[0].x < 256.0 && c.bounds[1].x > 256.0));
    }
    #[test]
    fn culling_keeps_intersecting_bounds_and_rejects_distant_bounds() {
        let identity = [
            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        ];
        assert!(visible(
            [
                XYZ {
                    x: -2.,
                    y: -2.,
                    z: -2.
                },
                XYZ {
                    x: 2.,
                    y: 2.,
                    z: 2.
                }
            ],
            identity
        ));
        assert!(!visible(
            [
                XYZ {
                    x: 2.,
                    y: 0.,
                    z: 0.
                },
                XYZ {
                    x: 3.,
                    y: 1.,
                    z: 1.
                }
            ],
            identity
        ));
        assert_eq!(
            distance_squared(
                [
                    XYZ {
                        x: 0.,
                        y: 0.,
                        z: 0.
                    },
                    XYZ {
                        x: 10.,
                        y: 10.,
                        z: 10.
                    }
                ],
                XYZ {
                    x: 15.,
                    y: 100.,
                    z: 5.
                }
            ),
            25.
        );
    }
}

mod skid_marks {
    use crate::model;
    use crate::physics;
    use crate::physics::car::Vehicle;
    use crate::render;
    use crate::render::skid_marks::*;
    use crate::test;
    use crate::world;

    fn sliding(model: std::sync::Arc<model::car::CarSpec>, distance: f32) -> Vehicle {
        let road = world::locations::at_distance(&test::support::compact_track(), distance);
        let mut car = test::support::vehicle(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            model,
        );
        car.tires.scrub = 0.9;
        car
    }

    #[test]
    fn trails_never_bridge_recovery_or_grip_changes() {
        let model = test::support::car();
        let mut marks = SkidMarks::default();
        render::skid_marks::record(&mut marks, 0, &sliding(model.clone(), 50.0));
        render::skid_marks::record(&mut marks, 0, &sliding(model.clone(), 100.0));
        assert!(render::skid_marks::vertices(&marks).is_empty());
        render::skid_marks::break_trail(&mut marks, 0);
        render::skid_marks::record(&mut marks, 0, &sliding(model.clone(), 100.5));
        assert!(render::skid_marks::vertices(&marks).is_empty());
        let mut car = sliding(model.clone(), 101.0);
        car.tires.scrub = 0.0;
        render::skid_marks::record(&mut marks, 0, &car);
        render::skid_marks::record(&mut marks, 0, &sliding(model, 101.5));
        assert!(render::skid_marks::vertices(&marks).is_empty());
    }

    #[test]
    fn trail_storage_is_bounded_and_stationary_cars_leave_nothing() {
        let model = test::support::car();
        let mut marks = SkidMarks::default();
        for _ in 0..100 {
            render::skid_marks::record(&mut marks, 0, &sliding(model.clone(), 50.0));
        }
        assert!(render::skid_marks::vertices(&marks).is_empty());
        for i in 0..3000 {
            render::skid_marks::record(
                &mut marks,
                0,
                &sliding(model.clone(), 50.0 + i as f32 * 0.25),
            );
        }
        assert!(!render::skid_marks::vertices(&marks).is_empty());
        assert!(render::skid_marks::vertices(&(marks)).len() <= 2048 * 6);
    }
}

mod tire_particles {
    use crate::model;
    use crate::physics;
    use crate::physics::car::Vehicle;
    use crate::render;
    use crate::render::tire_particles::*;
    use crate::test;
    use crate::util;
    use crate::util::geometry::XYZ;
    use crate::world;
    use std::time::Duration;

    fn car(offset: f32) -> Vehicle {
        let road = world::locations::at_distance(
            &world::locations::try_track(&world::locations::LocationId::Compact).unwrap(),
            50.0,
        );
        let mut car = test::support::vehicle(
            physics::vehicle_location::vehicle_position(road, offset),
            model::world::heading(road),
            test::support::car(),
        );
        car.velocity = util::geometry::at_height(
            util::geometry::xz_map(road.tangent, |v| v * 10.0),
            car.velocity.y,
        );
        car.tires.scrub = 0.9;
        car
    }

    fn emit(p: &mut TireParticles, car: &Vehicle, steps: usize) {
        for _ in 0..steps {
            let dt = Duration::from_secs_f64(1.0 / 60.0);
            render::tire_particles::advance(p, dt);
            render::tire_particles::record(p, 0, car, dt);
        }
    }

    #[test]
    fn particles_are_bounded_expire_and_recovery_clears_only_its_owner() {
        let v = car(0.0);
        let mut p = TireParticles::default();
        for _ in 0..1000 {
            render::tire_particles::record(&mut p, 0, &v, Duration::from_millis(100));
            render::tire_particles::record(&mut p, 1, &v, Duration::from_millis(100));
        }
        assert!(visible(&p).len() <= 160 * 6);
        assert!(!visible(&p).is_empty());
        render::tire_particles::reset_car(&mut p, 0);
        assert!(!visible(&p).is_empty());
        render::tire_particles::reset_car(&mut p, 1);
        assert!(visible(&p).is_empty());
        render::tire_particles::record(&mut p, 1, &v, Duration::from_millis(100));
        render::tire_particles::advance(&mut p, Duration::from_secs(2));
        assert!(visible(&p).is_empty());
        assert!(visible(&TireParticles::default()).is_empty());
    }

    #[test]
    fn fixed_step_emission_is_repeatable_and_teleports_do_not_burst() {
        let mut a = TireParticles::default();
        let mut b = TireParticles::default();
        let v = car(0.0);
        emit(&mut a, &v, 60);
        emit(&mut b, &v, 60);
        assert_eq!(visible(&a), visible(&b));
        let mut moved = car(0.0);
        moved.position.x += 20.0;
        let count = visible(&a).len();
        render::tire_particles::record(&mut a, 0, &moved, Duration::from_millis(100));
        assert_eq!(visible(&a).len(), count);
        render::tire_particles::reset_car(&mut a, 0);
        assert!(visible(&a).is_empty());
    }

    fn visible(particles: &TireParticles) -> Vec<Vertex> {
        render::tire_particles::vertices(
            particles,
            View {
                eye: XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: -10000.0,
                },
                right: XYZ {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                up: XYZ {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                forward: XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
            },
        )
    }
}

mod camera {
    use crate::model;
    use crate::physics;
    use crate::physics::car::Vehicle;
    use crate::render;
    use crate::render::camera::*;
    use crate::test;
    use crate::util;
    use crate::util::geometry::XYZ;
    use crate::util::geometry::XZ;
    use crate::world;
    use std::{f32::consts::TAU, time::Duration};

    fn car() -> Vehicle {
        let road = world::locations::at_distance(&test::support::compact_track(), 50.0);
        test::support::vehicle(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            test::support::car(),
        )
    }

    #[test]
    fn view_names_round_trip() {
        for view in [FollowView::Wide, FollowView::Close, FollowView::Bonnet] {
            assert_eq!(
                render::camera::from_name(render::camera::view_name(view)),
                Some(view)
            );
        }
        assert_eq!(render::camera::from_name("WIDE"), Some(FollowView::Wide));
        assert_eq!(render::camera::from_name("nope"), None);
    }

    fn assert_close(a: XYZ, b: XYZ, tolerance: f32) {
        for i in 0..3 {
            assert!(
                (util::geometry::xyz_component(a, i) - util::geometry::xyz_component(b, i)).abs()
                    < tolerance,
                "{a:?} != {b:?}"
            );
        }
    }

    #[test]
    fn changing_routes_snaps_camera_and_preserves_selected_view() {
        let mut camera = render::camera::new(&car());
        render::camera::toggle_view(&mut camera);
        let track = test::support::track();
        let road = world::locations::at_distance(&track, 850.0);
        let mut scenic = physics::car::on_track(
            physics::vehicle_location::vehicle_position(road, 0.0),
            model::world::heading(road),
            test::support::car(),
            track.clone(),
        );
        scenic.velocity = util::geometry::at_height(
            util::geometry::xz_map(road.tangent, |v| v * 30.0),
            scenic.velocity.y,
        );
        render::camera::update(&mut camera, &scenic, Duration::ZERO);

        assert_eq!(render::camera::view(&(camera)), FollowView::Wide);

        let mut expected = render::camera::new(&scenic);
        render::camera::toggle_view(&mut expected);
        render::camera::snap(&mut expected, &scenic);
        assert_eq!(
            render::camera::pose(&(camera)),
            render::camera::pose(&(expected))
        );
        let compact = car();
        render::camera::update(&mut camera, &compact, Duration::from_millis(16));

        render::camera::snap(&mut camera, &scenic);
    }

    #[test]
    fn smoothing_is_frame_rate_independent_for_a_constant_target() {
        let v = car();
        let mut moved = car();
        moved.position.x += 4.0;
        moved.orientation.yaw += 0.3;
        moved.velocity = XYZ {
            y: moved.velocity.y,
            x: moved.orientation.yaw.sin() * 18.0,
            z: moved.orientation.yaw.cos() * 18.0,
        };
        let run = |fps: u32| {
            let mut c = render::camera::new(&v);
            render::camera::toggle_view(&mut c);
            for _ in 0..fps {
                render::camera::update(&mut c, &moved, Duration::from_secs_f64(1.0 / fps as f64));
            }
            render::camera::pose(&c)
        };
        let a = run(30);
        for fps in [60, 144] {
            let b = run(fps);
            assert_close(a.eye, b.eye, 0.002);
            assert_close(a.target, b.target, 0.002);
        }
    }

    #[test]
    fn heading_wrap_uses_short_path_and_zero_dt_is_frozen() {
        let mut v = car();
        v.orientation.yaw = TAU - 0.02;
        let mut c = render::camera::new(&v);
        v.orientation.yaw = 0.02;
        let previous = render::camera::pose(&c);
        render::camera::update(&mut c, &v, Duration::from_millis(16));
        let current = render::camera::pose(&c);
        let moved = (current.eye.x - previous.eye.x).hypot(current.eye.z - previous.eye.z);
        assert!(
            moved < 0.5,
            "heading wrap must follow the short path: {moved}"
        );
        let before = render::camera::pose(&c);
        v.position.x += 20.0;
        render::camera::update(&mut c, &v, Duration::ZERO);
        assert_eq!(before, render::camera::pose(&(c)));
    }

    #[test]
    fn both_views_stay_finite_and_above_road_on_a_complete_route() {
        let mut v = car();
        let mut c = render::camera::new(&v);
        for close in [false, true] {
            if close {
                render::camera::toggle_view(&mut c);
            }
            for i in 0..600 {
                let road = world::locations::at_distance(
                    &test::support::compact_track(),
                    i as f32 * world::locations::sampling_length(&test::support::compact_track())
                        / 600.0,
                );
                v.position = physics::vehicle_location::vehicle_position(road, 1.4);
                v.orientation.yaw = model::world::heading(road);
                v.velocity = util::geometry::at_height(
                    util::geometry::xz_map(road.tangent, |x| x * 20.0),
                    v.velocity.y,
                );
                render::camera::update(&mut c, &v, Duration::from_secs_f64(1.0 / 60.0));
                let pose = render::camera::pose(&c);
                assert!(util::geometry::iter(&(pose.eye))
                    .chain(util::geometry::iter(&(pose.target)))
                    .all(|x| x.is_finite()));
                assert!(
                    pose.eye.y
                        >= world::elevation::world_height(
                            &test::support::compact_track(),
                            XZ {
                                x: pose.eye.x,
                                z: pose.eye.z
                            }
                        ) + 1.499
                );
                let gap = (0..3)
                    .map(|i| {
                        (util::geometry::xyz_component(pose.eye, i)
                            - util::geometry::xyz_component(pose.target, i))
                        .powi(2)
                    })
                    .sum::<f32>();
                assert!(gap > 10.0);
            }
        }
    }
}
