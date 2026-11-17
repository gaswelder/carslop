use crate::model;
use crate::model::location;
use crate::storage;
use crate::test;
use crate::util;
use crate::util::geometry::XZ;
use crate::world;
use std::fs;

fn assert_data_equal(a: &location::Location, b: &location::Location) {
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
fn manifest_coordinates_are_required_and_validated() {
    let input = tempfile::tempdir().unwrap();
    let assets = test::support::stored_location(input.path(), false);
    let city_path = input.path().join("city.txt");
    let city = fs::read_to_string(&city_path).unwrap();
    for record in ["origin 1 2", "bounds -1 -1 1 1"] {
        fs::write(
            &city_path,
            city.replacen("CITY 2\n", &format!("CITY 2\n{record}\n"), 1),
        )
        .unwrap();
        assert!(storage::city_assets::get_city(&assets).is_err(), "{record}");
    }
    fs::write(&city_path, city).unwrap();
    let loaded = storage::city_assets::get_city(&assets).unwrap();
    assert_eq!(loaded.origin, Some(util::geometry::new(34.7, 33.0)));
    assert_eq!(loaded.bounding_box.min, XZ { x: -10., z: -10. });
    for manifest in [
        serde_json::json!({}),
        serde_json::json!({"bounds_metres": [0, 0, 0, 1]}),
        serde_json::json!({"bounds_metres": [0, 2, 1, 1]}),
        serde_json::json!({"bounds_metres": [0, 0, 1]}),
        serde_json::json!({"bounds_metres": [0, 0, 1e100, 1]}),
        serde_json::json!({"bounds_metres": [0, 0, 1, 1], "origin": {"latitude": 91, "longitude": 0}}),
        serde_json::json!({"bounds_metres": [0, 0, 1, 1], "origin": {"latitude": 0, "longitude": -181}}),
        serde_json::json!({"bounds_metres": [0, 0, 1, 1], "origin": {"latitude": 0}}),
        serde_json::json!({"bounds_metres": [0, 0, 1, 1], "pavement_height_metres": 1.0}),
        serde_json::json!({"bounds_metres": [0, 0, 1, 1], "pavement_height_metres": null}),
    ] {
        fs::write(input.path().join("manifest.json"), manifest.to_string()).unwrap();
        assert!(
            storage::city_assets::get_city(&assets)
                .err()
                .unwrap()
                .contains("manifest.json"),
            "{manifest}"
        );
    }
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
        storage::location::location_read(input.path())
            .unwrap()
            .labels,
        location::Labels::default()
    );
    fs::write(input.path().join("labels.txt"), "invalid label").unwrap();
    assert!(storage::location::location_read(input.path())
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
            input.path().join("labels.txt"),
            "street 1 Road & Square\nbuilding 1 Road & Square\n",
        )
        .unwrap();
        fs::write(input.path().join("source.osm"), "unused build input").unwrap();
        fs::write(input.path().join("source.track.json"), b"build-only").unwrap();
        let mut location = storage::location::location_read(input.path()).unwrap();
        if !populated {
            location.origin = None;
        }
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
        storage::location::location_write(&location, output.path()).unwrap();
        assert!(!output.path().join("barriers.txt").exists());
        assert!(output.path().join("labels.txt").is_file());
        assert!(!output.path().join("labels.json").exists());
        let reread = storage::location::location_read(output.path()).unwrap();
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
        assert_eq!(
            manifest["bounds_metres"],
            serde_json::json!(util::geometry::bounding_box_xz_to_array(location.extent))
        );
        if let Some(origin) = location.origin {
            assert_eq!(
                manifest["origin"],
                serde_json::json!({
                    "latitude": origin.lat, "longitude": origin.lon,
                })
            );
        } else {
            assert!(manifest.get("origin").is_none());
        }
        let city = fs::read_to_string(output.path().join("city.txt")).unwrap();
        assert!(!city
            .lines()
            .any(|line| line.starts_with("origin ") || line.starts_with("bounds ")));
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
        storage::location::location_write(&reread, output.path()).unwrap();
        assert_data_equal(
            &location,
            &storage::location::location_read(output.path()).unwrap(),
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
        br#"{"aerial_attribution":"AERIAL: CYPRUS DLS / 2019","origin":{"latitude":34.7,"longitude":33.0},"bounds_metres":[-10,-10,30,30]}"#,
    )
    .unwrap();
    fs::write(input.path().join("aerial/service.json"), b"build-only").unwrap();
    let mut location = storage::location::location_read(input.path()).unwrap();
    storage::location::location_write(&location, output.path()).unwrap();
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
        serde_json::from_slice(&fs::read(output.path().join("manifest.json")).unwrap()).unwrap();
    assert_eq!(manifest["aerial_attribution"], "AERIAL: CYPRUS DLS / 2019");
    assert!(!output.path().join("aerial/service.json").exists());
    storage::location::location_write(&location, input.path()).unwrap();
    fs::remove_file(input.path().join("scene.pack")).unwrap();
    fs::remove_dir_all(input.path().join("aerial")).unwrap();
    fs::remove_file(input.path().join("aerial.txt")).unwrap();
    location = storage::location::location_read(input.path()).unwrap();
    fs::write(output.path().join("imagery-trees.txt"), "stale").unwrap();
    storage::location::location_write(&location, output.path()).unwrap();
    let reread = storage::location::location_read(output.path()).unwrap();
    assert!(!output.path().join("scene.pack").exists());
    assert!(!output.path().join("aerial.txt").exists());
    assert!(!output.path().join("aerial/tile.rgb").exists());
    assert_data_equal(&location, &reread);
}
