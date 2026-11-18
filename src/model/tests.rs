//! Tests through the group's production interfaces.

mod car_loading {
    use crate::model::car_loading;
    use crate::test;
    use std::fs;

    #[test]
    fn json_definition_loads_without_runtime_assets() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut definition = test::support::car_definition();
        definition["engine"]["power_kw"] = serde_json::json!(90.0);
        definition.as_object_mut().unwrap().remove("paint_color");
        fs::write(file.path(), serde_json::to_vec(&definition).unwrap()).unwrap();

        let loaded = car_loading::load(file.path()).unwrap();
        assert_eq!(loaded.engine.power_kw, 90.0);
        assert_eq!(loaded.paint_color, [1.0, 0.18, 0.08]);

        for invalid in [-1.0, 0.0] {
            definition["engine"]["power_kw"] = serde_json::json!(invalid);
            fs::write(file.path(), serde_json::to_vec(&definition).unwrap()).unwrap();
            assert!(car_loading::load(file.path()).is_err());
        }
    }
}

mod car {
    use crate::asset;
    use crate::model;
    use crate::model::car::CarSpec;
    use crate::test;
    use crate::util::geometry::XYZ;
    use std::{fs, sync::Arc};

    #[test]
    fn engine_bank_is_discovered_from_the_audio_directory() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("car");
        test::support::write_car(&directory);
        assert!(model::car::load(root.path())
            .unwrap()
            .remove(0)
            .engine_sample_directory
            .is_none());
        let audio_directory = directory.join("audio");
        fs::create_dir(&audio_directory).unwrap();
        assert_eq!(
            model::car::load(root.path())
                .unwrap()
                .remove(0)
                .engine_sample_directory,
            Some(audio_directory)
        );
    }

    #[test]
    fn center_of_mass_ratios_load_and_validate() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("car");
        test::support::write_car(&directory);
        let path = directory.join("car.json");
        let mut definition = test::support::car_definition();
        for longitudinal in [0.0, 0.39, 1.0] {
            for height in [0.0, 0.35, 1.0] {
                definition["center_of_mass"] = serde_json::json!({
                    "longitudinal_fraction": longitudinal,
                    "height_fraction": height
                });
                fs::write(&path, serde_json::to_vec(&definition).unwrap()).unwrap();
                let cars = model::car::load(root.path()).unwrap();
                let car = &cars[0];
                assert!((car.center_of_mass.longitudinal_fraction - longitudinal).abs() < 0.00001);
                assert!((car.center_of_mass.height_fraction - height).abs() < 0.00001);
                assert_eq!(car.model_height_m, asset::car_glb::height(&car.design));
                assert!(
                    (model::car::center_of_mass_height_m(car)
                        - height * asset::car_glb::height(&car.design))
                    .abs()
                        < 0.00001
                );
            }
        }
        for field in ["longitudinal_fraction", "height_fraction"] {
            for invalid in [-0.01, 1.01] {
                definition["center_of_mass"] = serde_json::json!({
                    "longitudinal_fraction": 0.5,
                    "height_fraction": 0.5
                });
                definition["center_of_mass"][field] = serde_json::json!(invalid);
                fs::write(&path, serde_json::to_vec(&definition).unwrap()).unwrap();
                assert!(model::car::load(root.path()).unwrap().is_empty());
            }
        }
    }

    #[test]
    fn pitsunda_reference_model_loads() {
        let cars = model::car::load(std::path::Path::new("data/cars")).unwrap();
        let car = cars
            .iter()
            .find(|car| car.name == "2016-pitsunda-2")
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
        assert!((cars_with_mounts(mounts).remove(0).chassis.wheelbase - 2.5).abs() < 0.00001);
        let translated = mounts.map(|p| XYZ { z: p.z + 10.0, ..p });
        assert!((cars_with_mounts(translated).remove(0).chassis.wheelbase - 2.5).abs() < 0.00001);
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
        assert!((cars_with_mounts(mounts).remove(0).chassis.track_width - 1.6).abs() < 0.00001);
        let translated = mounts.map(|p| XYZ { x: p.x + 10.0, ..p });
        assert!((cars_with_mounts(translated).remove(0).chassis.track_width - 1.6).abs() < 0.00001);
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
        let loaded = storage::furniture::parse(&furniture).unwrap().controls;
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
