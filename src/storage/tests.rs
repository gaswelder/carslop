//! Tests through the group's production interfaces.

mod aerial {
    use crate::storage::aerial::*;

    #[test]
    fn catalog_rejects_unsafe_filenames_and_nonfinite_bounds() {
        assert!(parse("AERIAL 1\nsize 512 8\ntile 0 0 100 100 ../a.rgb 00000000\n").is_err());
        assert!(parse("AERIAL 1\nsize 512 8\ntile NaN 0 100 100 a.rgb 00000000\n").is_err());
    }
}

mod codec_barriers {

    use crate::storage::codec_barriers::*;

    #[test]
    fn rejects_nonfinite_degenerate_and_unknown_barriers() {
        for record in [
            "1 wall 0 0 0 0 2 0.2",
            "1 wall NaN 0 10 0 2 0.2",
            "1 wall 0 0 10 0 inf 0.2",
            "1 wall 0 0 10 0 2 0",
            "1 wall 0 0 10 0 20 0.2",
            "1 unknown 0 0 10 0 2 0.2",
            "1 wall 0 0 10 0 2",
            "1 wall 0 0 100001 0 2 0.2",
        ] {
            assert!(
                parse(&format!("BARRIERS 1\n{record}\n")).is_err(),
                "{record}"
            );
        }
        assert!(parse("BARRIERS 2\n").is_err());
        assert!(parse("BARRIERS 1\n").unwrap().is_empty());
        for kind in ["wall", "fence", "chain_link", "wood_fence"] {
            let records = parse(&format!(
                "BARRIERS 1\n1 {kind} -5.1234565 0 10 0 1.8765432 0.1234567\n"
            ))
            .unwrap();
            let mut bytes = Vec::new();
            write(&mut bytes, &records).unwrap();
            assert_eq!(
                parse(std::str::from_utf8(&bytes).unwrap()).unwrap(),
                records
            );
        }
    }
}

mod codec_city {
    use crate::model;
    use crate::model::source_location::SourceLocation;
    use crate::model::source_location::SourceRoad;
    use crate::model::world::Building;
    use crate::model::world::Entrance;
    use crate::storage::codec_city::*;
    use crate::util;
    use crate::util::geometry::BoundingBoxXZ;
    use crate::util::geometry::XZ;

    fn fixture() -> SourceLocation<f64> {
        let mut map = model::source_location::empty();
        map.origin = Some(util::geometry::new(35.123456789, 33.987654321));
        map.bounding_box = BoundingBoxXZ {
            min: XZ {
                x: -10.0004,
                z: -10.0004,
            },
            max: XZ {
                x: 30.0004,
                z: 30.0004,
            },
        };
        map.road_nodes = vec![
            util::geometry::at_height(
                XZ {
                    x: 0.00049,
                    z: -0.00049,
                },
                0.,
            ),
            util::geometry::at_height(
                XZ {
                    x: 20.123456,
                    z: 0.,
                },
                0.,
            ),
        ];
        map.road_lines.push(SourceRoad {
            osm_way_id: Some(42),
            direction: -1,
            kind: 0,
            lanes: 3,
            ..model::source_location::new([0, 1], 6.123456)
        });
        map.buildings.push(Building {
            id: 7,
            height_meters: 7.987654,
            tagged: false,
            holes: Vec::new(),
            min_height_meters: 0.0,
            base_y: 0.,
            ground_outline: vec![
                XZ { x: 1.123456, z: 2. },
                XZ { x: 3., z: 4. },
                XZ { x: 5., z: 6. },
            ],
        });
        map
    }

    #[test]
    fn invalid_building_volume_metadata_is_rejected() {
        let mut bytes = Vec::new();
        write(&mut bytes, &fixture(), Precision::Imported).unwrap();
        let city = std::str::from_utf8(&bytes).unwrap();
        for record in [
            "v 999 0 0",
            "v 0 -1 0",
            "v 0 100 0",
            "v 0 NaN 0",
            "v 0 0 1 2 0 0 1 1",
            "v 0 0 0 extra",
        ] {
            assert!(parse(&format!("{city}{record}\n")).is_err(), "{record}");
        }
    }

    #[test]
    fn entrance_records_round_trip_and_require_an_existing_building() {
        for precision in [Precision::Imported, Precision::Authored] {
            let mut map = fixture();
            map.entrances.push(Entrance {
                building_id: 7,
                position: XZ {
                    x: 1.123456789,
                    z: 2.0,
                },
            });
            let mut bytes = Vec::new();
            write(&mut bytes, &map, precision).unwrap();
            let text = std::str::from_utf8(&bytes).unwrap();
            let city = parse(text).unwrap();
            assert_eq!(city.entrances.len(), 1);
            assert_eq!(city.entrances[0].building_id, 7);
            assert!((city.entrances[0].position.x - 1.1234568).abs() < 0.001);
            for malformed in ["e 999 0 0", "e 7 NaN 0", "e 7 0", "e bad 0 0"] {
                assert!(parse(&format!("{text}{malformed}\n")).is_err());
            }
        }
    }

    #[test]
    fn city_precision_policies_preserve_legacy_text_and_runtime_geometry() {
        for (precision, expected, first_x, width) in [
            (
                Precision::Imported,
                concat!(
                    "CITY 2\norigin 35.123456789 33.987654321\nbounds -10.000 -10.000 30.000 30.000\n",
                    "n 0.000 -0.000\nn 20.123 0.000\nr 0 1 6.12 -1 42 0 3\n",
                    "b 7 7.99 0 3 1.123 2.000 3.000 4.000 5.000 6.000\n"
                ),
                0.0,
                6.12,
            ),
            (
                Precision::Authored,
                concat!(
                "CITY 2\norigin 35.123456789 33.987654321\nbounds -10.0004 -10.0004 30.0004 30.0004\n",
                "n 0.00049 -0.00049\nn 20.123456 0\nr 0 1 6.123456 -1 42 0 3\n",
                "b 7 7.987654 0 3 1.123456 2 3 4 5 6\n"),
                0.00049,
                6.123456,
            ),
        ] {
            let mut bytes = vec![];
            write(&mut bytes, &fixture(), precision).unwrap();
            assert_eq!(std::str::from_utf8(&bytes).unwrap(), expected);
            let parsed = parse(expected).unwrap();
            assert_eq!(parsed.road_nodes[0].x, first_x);
            assert_eq!(parsed.road_lines[0].road_width_start_meters, width);
            assert_eq!(parsed.road_lines[0].direction, -1);
            assert_eq!(parsed.road_lines[0].osm_way_id, Some(42));
            assert_eq!(parsed.road_lines[0].lanes, 3);
            assert_eq!(parsed.origin, fixture().origin);
            assert!(!parsed.buildings[0].tagged);
        }
    }
}

mod codec_landscape {
    use crate::model::world::Material;
    use crate::model::world::Tree;
    use crate::storage;
    use crate::storage::codec_landscape::*;
    use crate::util::geometry::XZ;
    use std::io::Cursor;

    #[test]
    fn landscape_precisions_round_trip_without_changing_authored_or_imported_text() {
        let tree = Tree {
            id: u64::MAX,
            coords: XZ {
                x: 1.123456,
                z: -2.654321,
            },
            height_meters: 4.123456,
            crown_radius_meters: 0.7,
        };
        let points = [
            XZ {
                x: 0.123456789,
                z: 0.,
            },
            XZ { x: 10., z: 0. },
            XZ { x: 0., z: 10. },
        ];
        for (precision, expected) in [
            (Precision::Imported, "LANDSCAPE 1\na 1 0.123457 0.000000 10.000000 0.000000 0.000000 10.000000\nt 18446744073709551615 1.123 -2.654 4.123 0.700\n"),
            (Precision::Authored, "LANDSCAPE 1\na 1 0.123456789 0 10 0 0 10\nt 18446744073709551615 1.123456 -2.654321 4.123456 0.7\n"),
        ] {
            let mut bytes = vec![];
            let mut writer = storage::codec_landscape::new(&mut bytes, precision).unwrap();
            storage::codec_landscape::ground(&mut writer, Material::Park, points).unwrap();
            storage::codec_landscape::tree(&mut writer, &tree).unwrap();
            storage::codec_landscape::flush(&mut writer).unwrap();
            assert_eq!(std::str::from_utf8(&bytes).unwrap(), expected);
            let mut runtime_tree = None;
            read::<f32>(Cursor::new(&bytes), |r| {
                if let Record::Tree(t) = r { runtime_tree = Some(t); }
                Ok(())
            }).unwrap();
            let runtime_tree = runtime_tree.unwrap();
            assert_eq!(runtime_tree.id, u64::MAX);
            assert_eq!(runtime_tree.crown_radius_meters, 0.7_f32);
        }
    }

    #[test]
    fn landscape_validation_is_shared_and_emission_errors_propagate() {
        for record in [
            "a 9 0 0 1 0 0 1",
            "t 1 NaN 0 4 2",
            "t 1 100000 0 4 2",
            "t 1 0 0 -2 1",
            "t 1 0 0 4 0.69",
            "t 1 0 0 4",
            "unknown",
            "",
        ] {
            let text = format!("LANDSCAPE 1\n{record}\n");
            assert!(
                read::<f32>(Cursor::new(&text), |_| Ok(())).is_err(),
                "{record}"
            );
            assert!(
                read::<f64>(Cursor::new(&text), |_| Ok(())).is_err(),
                "{record}"
            );
        }
        let error = read::<f64>(Cursor::new("LANDSCAPE 1\nt 1 0 0 4 2\n"), |_| {
            Err("stop".into())
        })
        .unwrap_err();
        assert_eq!(error, "stop");
        assert!(storage::codec_landscape::ground(
            &mut (storage::codec_landscape::new(std::io::sink(), Precision::Imported).unwrap()),
            Material::Paint,
            [XZ { x: 0., z: 0. }; 3]
        )
        .is_err());
    }
}

mod codec_road_rises {

    use crate::storage::codec_road_rises::*;

    #[test]
    fn road_rise_codec_preserves_offsets_and_rejects_invalid_triangles() {
        let text = "ROAD_RISES 1\nt 0 0 0 2 0.1234567 0 0 0 2\n";
        let rises = parse(text).unwrap();
        let mut bytes = Vec::new();
        write(&mut bytes, &rises).unwrap();
        assert_eq!(parse(std::str::from_utf8(&bytes).unwrap()).unwrap(), rises);
        for invalid in [
            "t 0 0 0 0 0 0 0 0 0",
            "t 0 0 0 2 NaN 0 0 0 2",
            "t 0 0 0 2 0.5 0 0 0 2",
            "t 0 0 0 2 -0.1 0 0 0 2",
            "t 0 0 0 2 0 0 0 0",
        ] {
            assert!(parse(&format!("ROAD_RISES 1\n{invalid}\n")).is_err());
        }
    }
}

mod street_surface_binary {
    use crate::model;
    use crate::physics;
    use crate::render;
    use crate::storage;
    use crate::test;
    use crate::util::geometry::XYZ;
    use crate::util::geometry::XZ;
    use crate::world;
    use crate::world::street_surface::RoadPaths;
    use crate::world::street_surface::StreetSurface;
    use std::{io, path::Path};

    fn fixture(root: &Path) -> String {
        let source = "CITY 2\nbounds 0 0 20 20\nn 0 0\nn 20 0\nr 0 1 8 0 1\n";
        std::fs::write(root.join("city.txt"), source).unwrap();
        let streets = format!(
            "STREETS 1 1 {:08x}\nj 0\np 0 3 0 0 10 1 20 0\nt 0 0 0 20 0 0 20\nt 4 20 0 20 20 0 20\nc 0 0 0 20 0 0 1\n",
            0
        );
        std::fs::write(root.join("streets.txt"), streets).unwrap();
        source.into()
    }
    fn compare(expected: StreetSurface, actual: StreetSurface) {
        assert_eq!(expected.road_triangles.len(), actual.road_triangles.len());
        assert_eq!(expected.curbs.len(), actual.curbs.len());
        assert_eq!(expected.triangles_grid, actual.triangles_grid);
        assert_eq!(expected.curbs_grid, actual.curbs_grid);
        for (a, b) in expected.road_triangles.iter().zip(&actual.road_triangles) {
            assert_eq!(a.material, b.material);
            assert_eq!(a.usage, b.usage);
            assert_eq!(a.points, b.points);
        }
        for (a, b) in expected.curbs.iter().zip(&actual.curbs) {
            assert_eq!(a.style, b.style);
            assert_eq!(a.ends, b.ends);
            assert_eq!(a.inward, b.inward);
        }
        let mut expected_world = test::support::empty_world();
        test::support::install_surfaces(&mut expected_world, expected);
        let expected = expected_world;
        let mut actual_world = test::support::empty_world();
        test::support::install_surfaces(&mut actual_world, actual);
        let actual = actual_world;
        for x in -2..24 {
            for z in -2..24 {
                let p = XZ {
                    x: x as f32,
                    z: z as f32,
                };
                assert_eq!(
                    world::street_surface::contains_road(&(expected), p),
                    world::street_surface::contains_road(&(actual), p)
                );
                assert_eq!(
                    render::scene::road_color(&(expected), p),
                    render::scene::road_color(&(actual), p)
                );
                assert_eq!(
                    world::street_surface::paving_height(&(expected), p),
                    world::street_surface::paving_height(&(actual), p)
                );
                assert_eq!(
                    physics::vehicle_location::crossed_curb(
                        &(expected),
                        p,
                        XZ {
                            x: p.x,
                            z: p.z + 2.0
                        }
                    ),
                    physics::vehicle_location::crossed_curb(
                        &(actual),
                        p,
                        XZ {
                            x: p.x,
                            z: p.z + 2.0
                        }
                    )
                );
            }
        }
    }
    #[test]
    fn binary_collision_round_trip_preserves_geometry_paths_grids_and_queries() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();
        fixture(root);
        let streets = std::fs::read_to_string(root.join("streets.txt")).unwrap();
        let (expected, paths) = world::street_surface::load(&streets, 1).unwrap();
        test::support::bake_collisions(
            &root.join("city.txt"),
            &root.join("streets.txt"),
            &root.join("streets.bin"),
        )
        .unwrap();
        let (actual, actual_paths) = load(root, 1).unwrap();
        compare(expected, actual);
        assert_eq!(paths, actual_paths);
        let changed = streets.replace("10 1", "10 2");
        std::fs::write(root.join("streets.txt"), &changed).unwrap();
        assert!(load(root, 1).is_ok());
        test::support::bake_collisions(
            &root.join("city.txt"),
            &root.join("streets.txt"),
            &root.join("streets.bin"),
        )
        .unwrap();
        let (_, updated_paths) = load(root, 1).unwrap();
        assert_eq!(
            updated_paths[0][1],
            XYZ {
                x: 10.0,
                y: 0.0,
                z: 2.0
            }
        );

        let city_path = root.join("map.data");
        let streets_path = root.join("roads.data");
        let output_directory = root.join("collision");
        std::fs::create_dir(&output_directory).unwrap();
        let output_path = output_directory.join("cache.bin");
        std::fs::rename(root.join("city.txt"), &city_path).unwrap();
        std::fs::rename(root.join("streets.txt"), &streets_path).unwrap();
        test::support::bake_collisions(&city_path, &streets_path, &output_path).unwrap();
        assert_eq!(
            std::fs::read(output_path).unwrap(),
            std::fs::read(root.join("streets.bin")).unwrap()
        );
        assert_eq!(std::fs::read_dir(output_directory).unwrap().count(), 1);
    }
    #[test]
    fn collision_data_ignores_reserved_bytes_and_requires_valid_geometry() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();
        fixture(root);
        assert!(load(root, 1).is_err());
        test::support::bake_collisions(
            &root.join("city.txt"),
            &root.join("streets.txt"),
            &root.join("streets.bin"),
        )
        .unwrap();
        let path = root.join("streets.bin");
        let original = std::fs::read(&path).unwrap();
        let mut legacy = original.clone();
        legacy[8..80].fill(0xff);
        let footer = legacy.len() - 4;
        legacy[footer..].fill(0xff);
        std::fs::write(&path, legacy).unwrap();
        assert!(load(root, 1).is_ok());
        std::fs::remove_file(root.join("streets.txt")).unwrap();
        assert!(load(root, 1).is_ok());
        assert!(load(root, 2).is_err());
        for length in [0, 16, original.len() - 1] {
            std::fs::write(&path, &original[..length]).unwrap();
            assert!(load(root, 1).is_err());
        }
    }

    #[test]
    fn generated_collision_cache_matches_source() {
        let world = test::support::city_world();
        let assets = model::world::assets(&world).unwrap();
        let root = storage::city_assets::directory(assets);
        let source = std::fs::read_to_string(root.join("city.txt")).unwrap();
        let streets = std::fs::read_to_string(root.join("streets.txt")).unwrap();
        let city = model::source_location::parse(&source).unwrap();
        let (expected, expected_paths) =
            world::street_surface::load(&streets, city.road_lines.len()).unwrap();
        let (actual, actual_paths) = load(root, city.road_lines.len()).unwrap();
        assert_eq!(expected_paths, actual_paths);
        eprintln!(
            "Binary cache exactly matches {} source triangles and {} curbs",
            actual.road_triangles.len(),
            actual.curbs.len()
        );
        compare(expected, actual);
    }

    fn load(directory: &Path, roads: usize) -> io::Result<(StreetSurface, RoadPaths)> {
        world::street_surface::from_geometry(
            storage::street_surface_binary::load_geometry(directory, roads)?,
            world::street_surface::DEFAULT_PAVEMENT_HEIGHT,
        )
        .map_err(|message| io::Error::other(message))
    }
}
