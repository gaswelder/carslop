//! Tests through the group's production interfaces.

mod car {
    use crate::asset;
    use crate::model;
    use crate::model::car::CarSpec;
    use crate::test;
    use crate::util::geometry::XYZ;
    use std::{fs, sync::Arc};

    #[test]
    fn engine_bank_is_discovered_without_a_json_path() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("car");
        test::support::write_car(&directory);
        assert!(model::car::load(root.path())
            .unwrap()
            .remove(0)
            .audio
            .sample_directory
            .is_none());
        let audio_directory = directory.join("audio");
        fs::create_dir(&audio_directory).unwrap();
        // An unrelated WAV does not opt the car into a recording bank.
        fs::write(audio_directory.join("unused.wav"), []).unwrap();
        assert!(model::car::load(root.path())
            .unwrap()
            .remove(0)
            .audio
            .sample_directory
            .is_none());
        fs::write(audio_directory.join("engine.json"), b"{}").unwrap();
        assert_eq!(
            model::car::load(root.path())
                .unwrap()
                .remove(0)
                .audio
                .sample_directory,
            Some(audio_directory)
        );
    }

    #[test]
    fn pitsunda_reference_model_loads() {
        let cars = model::car::load(std::path::Path::new("data/cars")).unwrap();
        let definition: serde_json::Value =
            serde_json::from_slice(&fs::read("data/cars/2016-pitsunda-2/car.json").unwrap())
                .unwrap();
        let car = cars
            .iter()
            .find(|car| car.name == definition["name"].as_str().unwrap())
            .expect("Pitsunda must load with its traced body and separate wheel pivots");
        assert_eq!(car.paint_color, [0.36, 0.355, 0.35]);
        assert!((asset::car_glb::height(&car.design) - 1.495).abs() < 0.002);
        for radius in car.design.wheel_radii {
            assert!((radius - 0.318).abs() < 0.002);
        }
    }

    #[test]
    fn wheelbase_comes_from_axle_midpoints() {
        // Different left/right Z positions exercise axle averaging.
        let mounts = [
            XYZ {
                x: -0.8,
                y: 0.1,
                z: -1.4,
            },
            XYZ {
                x: 0.8,
                y: 0.2,
                z: -1.2,
            },
            XYZ {
                x: -0.9,
                y: 0.3,
                z: 1.1,
            },
            XYZ {
                x: 0.9,
                y: 0.4,
                z: 1.3,
            },
        ];
        assert!((cars_with_mounts(mounts).remove(0).wheelbase - 2.5).abs() < 0.00001);
        let translated = mounts.map(|p| XYZ { z: p.z + 10.0, ..p });
        assert!((cars_with_mounts(translated).remove(0).wheelbase - 2.5).abs() < 0.00001);
        for z in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            let invalid = [
                XYZ {
                    x: -1.0,
                    y: 0.0,
                    z: 0.0,
                },
                XYZ {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                XYZ {
                    x: -1.0,
                    y: 0.0,
                    z: z,
                },
                XYZ {
                    x: 1.0,
                    y: 0.0,
                    z: z,
                },
            ];
            assert!(cars_with_mounts(invalid).is_empty());
        }
    }

    #[test]
    fn track_width_comes_from_both_axles() {
        let mounts = [
            XYZ {
                x: -0.7,
                y: 0.0,
                z: -1.0,
            },
            XYZ {
                x: 0.7,
                y: 0.0,
                z: -1.0,
            },
            XYZ {
                x: -0.9,
                y: 0.0,
                z: 1.0,
            },
            XYZ {
                x: 0.9,
                y: 0.0,
                z: 1.0,
            },
        ];
        assert!((cars_with_mounts(mounts).remove(0).track_width - 1.6).abs() < 0.00001);
        let translated = mounts.map(|p| XYZ { x: p.x + 10.0, ..p });
        assert!((cars_with_mounts(translated).remove(0).track_width - 1.6).abs() < 0.00001);
        for width in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            for wheel in [1, 3] {
                let mut invalid_mounts = mounts;
                invalid_mounts[wheel].x = invalid_mounts[wheel - 1].x + width;
                assert!(cars_with_mounts(invalid_mounts).is_empty());
            }
        }
    }

    fn cars_with_mounts(mounts: [XYZ; 4]) -> Vec<Arc<CarSpec>> {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("car");
        test::support::write_car(&directory);
        let mut design = (*test::support::car_spec().design).clone();
        design.mounts = mounts;
        fs::write(
            directory.join("model.glb"),
            test::support::model_glb(&design),
        )
        .unwrap();
        model::car::load(root.path()).unwrap()
    }
}

mod traffic_controls {
    use crate::model;
    use crate::storage;
    use crate::test;
    use crate::util;
    use crate::world;

    #[test]
    fn mapped_furniture_is_typed_finite_and_outside_the_driving_surface() {
        let city_track = test::support::city_world();
        let city = &city_track;
        assert!(!city.furniture.is_empty());
        for f in &city.furniture {
            assert!(
                !world::street_surface::contains_road(
                    city,
                    util::geometry::XZ {
                        x: f.coords.x,
                        z: f.coords.z
                    }
                ),
                "furniture {} is on asphalt at {:?}",
                f._source_id,
                f.coords
            );
            assert!(
                world::buildings::building_clearance(
                    city,
                    util::geometry::XZ {
                        x: f.coords.x,
                        z: f.coords.z
                    }
                ) >= 0.0
            );
            assert!(util::geometry::iter(&(f.coords)).all(|v| v.is_finite()));
        }
        let assets = model::world::assets(city).unwrap();
        let furniture =
            std::fs::read_to_string(storage::city_assets::directory(assets).join("furniture.txt"))
                .unwrap();
        let loaded = storage::codec_furniture::parse(&furniture)
            .unwrap()
            .controls;
        assert_eq!(loaded.len(), city.furniture.len());
        for (raw, placed) in loaded.iter().zip(&city.furniture) {
            assert_eq!(raw._source_id, placed._source_id);
            assert_eq!(raw.kind, placed.kind);
            assert_eq!(raw.travel, placed.travel);
            assert_eq!(raw.value, placed.value);
            assert_eq!(raw.phase, placed.phase);
        }
    }
}

mod location {
    use crate::model;
    use crate::model::location::*;
    use crate::storage;
    use crate::test;
    use crate::util;
    use crate::util::geometry::XZ;
    use crate::world;
    use std::fs;

    fn assert_data_equal(a: &Location, b: &Location) {
        assert_eq!(a.origin, b.origin);
        assert_eq!(a.extent, b.extent);
        assert_eq!(a.road_nodes, b.road_nodes);
        assert_eq!(a.roads, b.roads);
        assert_eq!(a.buildings, b.buildings);
        assert_eq!(a.entrances, b.entrances);
        assert_eq!(a.streets, b.streets);
        assert_eq!(a.pavement_height_meters, b.pavement_height_meters);
        assert_eq!(a.elevation, b.elevation);
        assert_eq!(a.ground, b.ground);
        assert_eq!(a.trees, b.trees);
        assert_eq!(a.furniture, b.furniture);
        assert_eq!(a.barriers, b.barriers);
        assert_eq!(a.road_rises, b.road_rises);
        assert_eq!(a.labels, b.labels);
        assert_eq!(a.details, b.details);
    }

    #[test]
    fn labels_never_fall_back_to_build_inputs() {
        let input = tempfile::tempdir().unwrap();
        test::support::stored_location(input.path(), false);
        fs::write(
            input.path().join("source.osm"),
            r#"<osm><way id="1"><tag k="name" v="Unused legacy name"/></way></osm>"#,
        )
        .unwrap();
        assert_eq!(
            model::location::location_read(input.path()).unwrap().labels,
            Labels::default()
        );
        fs::write(
            input.path().join(storage::codec_labels::FILE_NAME),
            "invalid label",
        )
        .unwrap();
        assert!(model::location::location_read(input.path())
            .unwrap_err()
            .contains("labels.txt"));
    }

    #[test]
    fn stored_location_round_trip_is_lossless_without_build_inputs() {
        for populated in [false, true] {
            let input = tempfile::tempdir().unwrap();
            let output = tempfile::tempdir().unwrap();
            test::support::stored_location(input.path(), populated);
            fs::write(
                input.path().join(storage::codec_labels::FILE_NAME),
                "street 1 Road & Square\nbuilding 1 Road & Square\n",
            )
            .unwrap();
            fs::write(input.path().join("source.osm"), "unused build input").unwrap();
            fs::write(input.path().join("source.track.json"), b"build-only").unwrap();
            let mut location = model::location::location_read(input.path()).unwrap();
            location.pavement_height_meters = 0.17345679;
            location.roads[0].way_id = None;
            location.roads[0].width_meters = 8.123457;
            location.elevation.values[0] = 7.1234565;
            location.details.trees.push(model::map_details::TreeStyle {
                id: 9,
                kind: model::map_details::TreeKind::Olive,
                evergreen: true,
            });
            location
                .details
                .objects
                .push(model::map_details::MapObject {
                    kind: model::map_details::ObjectKind::Bench,
                    position: [4.1234565, 6.],
                    angle: 1.2,
                    height: 0.9,
                    width: 1.8,
                    text: "Πάρκο".into(),
                    locked: false,
                });
            location.details.decks.push(model::map_details::Deck {
                road: 0,
                grade: 1,
                width: 8.,
                path: vec![[0., 5.1234565, 0.], [20., 6., 0.]],
                surface: model::map_details::RoadSurface::Concrete,
            });
            if populated {
                location
                    .details
                    .buildings
                    .push(model::map_details::BuildingStyle {
                        volume: 0,
                        shape: model::map_details::RoofShape::Gabled,
                        roof_height: 1.5,
                        angle: 0.3,
                        wall_color: Some([0.2, 0.3, 0.4]),
                        roof_color: None,
                    });
                location.entrances.push(model::world::Entrance {
                    building_id: 1,
                    position: XZ {
                        x: 10.,
                        z: 12.123457,
                    },
                });
                location.road_rises.push(model::world::RoadRise {
                    points: [
                        util::geometry::at_height(XZ { x: 0., z: 0. }, 0.),
                        util::geometry::at_height(XZ { x: 1., z: 0. }, 0.1234567),
                        util::geometry::at_height(XZ { x: 0., z: 1. }, 0.),
                    ],
                });
                location.barriers.push(model::world::Barrier {
                    source_id: 17,
                    kind: model::world::BarrierKind::ChainLink,
                    ends: [
                        XZ {
                            x: -5.1234565,
                            z: 10.,
                        },
                        XZ { x: 5., z: 10. },
                    ],
                    height_meters: 1.8765432,
                    width_meters: 0.1234567,
                });
                location.buildings[0].min_height_meters = 1.234567;
                location.buildings[0].holes = vec![vec![
                    XZ { x: 11., z: 11. },
                    XZ { x: 11., z: 12. },
                    XZ { x: 12., z: 12. },
                    XZ { x: 12., z: 11. },
                ]];
                location.buildings[0].height_meters = 5.1234565;
                location.furniture[0].position.x = 5.1234565;
                location.trees[0].coords.x = -5.1234565;
            }
            model::location::location_write(&location, output.path()).unwrap();
            assert!(!output.path().join("barriers.txt").exists());
            assert!(output
                .path()
                .join(storage::codec_labels::FILE_NAME)
                .is_file());
            assert!(!output.path().join("labels.json").exists());
            let reread = model::location::location_read(output.path()).unwrap();
            assert_data_equal(&location, &reread);
            assert_eq!(
                reread.name,
                output.path().file_name().unwrap().to_string_lossy()
            );
            assert!(!output.path().join("source.osm").exists());
            assert!(!output.path().join("source.track.json").exists());
            assert!(!output.path().join("imagery-trees.txt").exists());
            let manifest: serde_json::Value =
                serde_json::from_slice(&fs::read(output.path().join("manifest.json")).unwrap())
                    .unwrap();
            assert_eq!(manifest.as_object().unwrap().len(), 1);
            let world = world::preparation::world_location_load(&storage::city_assets::new(
                output.path().to_path_buf(),
            ))
            .unwrap();
            assert!(world::street_surface::has_compiled_roads(&(world)));
            assert!(reread.labels.streets.contains(&(1, "Road & Square".into())));
            assert!(reread
                .labels
                .buildings
                .contains(&(1, "Road & Square".into())));
            assert_eq!(world::roads::road_lines(&(world))[0].path.len(), 3);
            assert!(world::roads::road_lines(&(world))[0].length > 20.0);
            assert!(world::roads::road_lines(&(world))
                .iter()
                .all(|r| r.path[0].y != 0.0));
            // Writing in place neither duplicates the optional trees nor rounds data.
            model::location::location_write(&reread, output.path()).unwrap();
            assert_data_equal(
                &location,
                &model::location::location_read(output.path()).unwrap(),
            );
        }
    }

    #[test]
    fn render_payloads_round_trip_and_overwrite_removes_stale_optional_layers() {
        let input = tempfile::tempdir().unwrap();
        let output = tempfile::tempdir().unwrap();
        test::support::stored_location(input.path(), true);
        fs::write(input.path().join("scene.pack"), b"opaque compiled scene").unwrap();
        fs::create_dir(input.path().join("aerial")).unwrap();
        fs::write(
            input.path().join("aerial.txt"),
            "AERIAL 1\nsize 64 1\ntile -10 -10 30 30 tile.rgb\n",
        )
        .unwrap();
        fs::write(input.path().join("aerial/tile.rgb"), b"opaque tile").unwrap();
        fs::write(
            input.path().join("manifest.json"),
            br#"{"aerial_attribution":"AERIAL: CYPRUS DLS / 2019"}"#,
        )
        .unwrap();
        fs::write(input.path().join("aerial/service.json"), b"build-only").unwrap();
        let mut location = model::location::location_read(input.path()).unwrap();
        model::location::location_write(&location, output.path()).unwrap();
        for file in ["scene.pack", "aerial.txt", "aerial/tile.rgb"] {
            assert_eq!(
                fs::read(input.path().join(file)).unwrap(),
                fs::read(output.path().join(file)).unwrap()
            );
        }
        let assets = storage::city_assets::new(output.path().to_path_buf());
        assert_eq!(
            storage::city_assets::aerial_attribution(&assets),
            Some("AERIAL: CYPRUS DLS / 2019")
        );
        let manifest: serde_json::Value =
            serde_json::from_slice(&fs::read(output.path().join("manifest.json")).unwrap())
                .unwrap();
        assert_eq!(manifest["aerial_attribution"], "AERIAL: CYPRUS DLS / 2019");
        assert!(!output.path().join("aerial/service.json").exists());
        model::location::location_write(&location, input.path()).unwrap();
        fs::remove_file(input.path().join("scene.pack")).unwrap();
        fs::remove_dir_all(input.path().join("aerial")).unwrap();
        fs::remove_file(input.path().join("aerial.txt")).unwrap();
        location = model::location::location_read(input.path()).unwrap();
        fs::write(output.path().join("imagery-trees.txt"), "stale").unwrap();
        model::location::location_write(&location, output.path()).unwrap();
        let reread = model::location::location_read(output.path()).unwrap();
        assert!(!output.path().join("scene.pack").exists());
        assert!(!output.path().join("aerial.txt").exists());
        assert!(!output.path().join("aerial/tile.rgb").exists());
        assert_data_equal(&location, &reread);
    }
}
