//! Tests through the group's production interfaces.

mod osm {
    use crate::build;
    use crate::util;
    use geo::BoundingRect;

    #[test]
    fn import_uses_viewport_center_and_requested_dimensions() {
        for prefix in [
            "https://www.openstreetmap.org/way/535666548",
            "https://www.openstreetmap.org/relation/1?foo=bar",
            "https://www.openstreetmap.org/",
        ] {
            let area = build::osm::from_url(
                &format!("{prefix}#map=19/35.165236/33.318712&layers=N"),
                10.0,
                5.0,
            )
            .unwrap();
            assert_eq!(area.latitude, 35.165236);
            assert_eq!(area.longitude, 33.318712);
            let geometry = build::osm::geometry(&area).unwrap();
            let bounds = build::osm::bbox(&geometry);
            assert!(((bounds.min_lat + bounds.max_lat) / 2.0 - area.latitude).abs() < 1e-12);
            assert!(((bounds.min_lon + bounds.max_lon) / 2.0 - area.longitude).abs() < 1e-12);
            let local = build::osm::local(
                &geometry,
                util::geometry::new(area.latitude, area.longitude),
            );
            let rect = local.bounding_rect().unwrap();
            assert!((rect.width() - 10000.0).abs() < 1e-6);
            assert!((rect.height() - 5000.0).abs() < 1e-6);
            assert!((rect.min().x + 5000.0).abs() < 1e-6);
            assert!((rect.min().y + 2500.0).abs() < 1e-6);
        }
    }

    #[test]
    fn invalid_import_areas_are_rejected() {
        for fragment in [
            "",
            "#map=19",
            "#map=19/35/33/1",
            "#map=19/no/33",
            "#map=19/NaN/33",
            "#map=19/35/inf",
            "#map=19/86/33",
            "#map=19/35/181",
            "#map=19/85/33",
            "#map=19/35/180",
        ] {
            assert!(
                build::osm::from_url(
                    &format!("https://www.openstreetmap.org/{fragment}"),
                    10.0,
                    5.0,
                )
                .is_err(),
                "{fragment}"
            );
        }
        for invalid in [0.0, -1.0, 80.01, f64::NAN, f64::INFINITY] {
            let url = "https://www.openstreetmap.org/#map=19/35/33";
            assert!(build::osm::from_url(url, invalid, 5.0).is_err());
            assert!(build::osm::from_url(url, 10.0, invalid).is_err());
        }
        assert!(
            build::osm::from_url("https://www.openstreetmap.org/#map=1/-35/-33", 80.0, 80.0,)
                .is_ok()
        );
    }
}

mod create_track {
    use crate::build;
    use crate::model;
    use crate::util;

    #[test]
    fn invalid_circuit_authoring_input_is_rejected_before_building_runtime_geometry() {
        for points in [
            vec![],
            vec![[0.0, 0.0, 0.0, 1.0]; 4],
            vec![[f32::NAN, 0.0, 0.0, 8.0]; 4],
        ] {
            let circuit = build::authoring::Circuit {
                control_points: points
                    .into_iter()
                    .map(|[x, y, z, width]| build::authoring::ControlPoint {
                        position: util::geometry::XYZ { x, y, z },
                        width,
                    })
                    .collect(),
                coastal: false,
            };
            assert!(
                build::authoring::circuit_build(&(circuit), model::world::LocationId::Compact)
                    .is_err()
            );
        }
    }
}

mod tree_proposals {
    use crate::build::tree_proposals::*;
    use image::Rgb;
    use image::RgbImage;
    #[test]
    fn muted_textured_crowns_are_detected_but_smooth_muted_surfaces_are_not() {
        let mut im = RgbImage::from_pixel(160, 100, Rgb([110, 110, 110]));
        for y in 10..90 {
            for x in 10..150 {
                if (x as i32 - 45).pow(2) + (y as i32 - 50).pow(2) < 20 * 20 {
                    let noise = ((x * 13 + y * 7) % 25) as i16 - 12;
                    im.put_pixel(
                        x,
                        y,
                        Rgb([(65 + noise) as u8, (64 + noise) as u8, (59 + noise) as u8]),
                    );
                }
                if (x as i32 - 115).pow(2) + (y as i32 - 50).pow(2) < 20 * 20 {
                    im.put_pixel(x, y, Rgb([65, 64, 59]));
                }
            }
        }
        let found = proposals(&im, [0.25, 0.25]);
        assert!(found
            .iter()
            .any(|p| (p.x as i32 - 45).abs() < 5 && (p.y as i32 - 50).abs() < 5));
        assert!(
            found.iter().all(|p| p.x < 85),
            "smooth muted surfaces must not become crowns"
        );
    }
}

mod tree_obstacles {
    use crate::build;
    use crate::util::geometry::XZ;
    use std::fs;
    #[test]
    fn reads_matching_map_surfaces_and_preserves_pavement() {
        let temp = tempfile::tempdir().unwrap();
        let city="CITY 2\norigin 35.162 33.32\nbounds -50 -50 50 50\nn -20 0\nn 20 0\nr 0 1 8 0 1\nb 1 10 1 4 10 10 20 10 20 20 10 20\n";
        fs::write(temp.path().join("city.txt"), city).unwrap();
        let header = format!("STREETS 1 1 {:08x}\n", 0);
        let mesh = "t 0 -20 -2 20 -2 20 2\nt 0 -20 -2 20 2 -20 2\nt 4 -20 3 20 3 20 6\n";
        fs::write(temp.path().join("streets.txt"), format!("{header}{mesh}")).unwrap();
        let o = build::tree_obstacles::read_paths(
            &temp.path().join("city.txt"),
            &temp.path().join("streets.txt"),
            [33.319, 35.161, 33.321, 35.163],
        )
        .unwrap();
        let check = |x: f64, z: f64| build::tree_obstacles::rejection_at(&o, XZ { x, z });
        assert_eq!(check(0., 0.), Some("road"));
        assert_eq!(check(15., 15.), Some("building"));
        assert_eq!(
            check(10., 4.),
            None,
            "pavement must remain available for trees"
        );
        // Empty meshes are valid; the road count still has to match city.txt.
        fs::write(temp.path().join("streets.txt"), "STREETS 1 1\n").unwrap();
        let empty = build::tree_obstacles::read_paths(
            &temp.path().join("city.txt"),
            &temp.path().join("streets.txt"),
            [33.319, 35.161, 33.321, 35.163],
        )
        .unwrap();
        assert_eq!(
            build::tree_obstacles::rejection_at(&(empty), XZ { x: 0., z: 0. }),
            None
        );
        assert_eq!(
            build::tree_obstacles::rejection_at(&(empty), XZ { x: 15., z: 15. }),
            Some("building")
        );
        fs::write(temp.path().join("streets.txt"), "STREETS 1 2\n").unwrap();
        assert!(build::tree_obstacles::read_paths(
            &temp.path().join("city.txt"),
            &temp.path().join("streets.txt"),
            [33.319, 35.161, 33.321, 35.163]
        )
        .is_err());
    }
}

mod imagery_metadata {

    use crate::build::imagery::*;

    #[test]
    fn typed_metadata_round_trips_existing_json_and_tree_arrays() {
        let legacy = serde_json::json!({
            "jpeg_quality": 80, "extent": [1., 2., 3., 4.],
            "tiles": [{"name": "0_0_0_0.jpg", "bounds": [-2., -1., 2., 1.]}],
            "missing_tiles": 15, "trees": [[33.123456789, 35.987654321, 2.25]],
        });
        let meta: BlockMetadata = serde_json::from_value(legacy.clone()).unwrap();
        assert_eq!(meta.tiles[0].bounds, [-2., -1., 2., 1.]);
        assert_eq!(serde_json::to_value(&meta).unwrap(), legacy);
        let bytes = serde_json::to_vec(&meta).unwrap();
        assert_eq!(
            serde_json::from_slice::<BlockMetadata>(&bytes).unwrap(),
            meta
        );
        let mut old_quality = legacy.clone();
        old_quality.as_object_mut().unwrap().remove("jpeg_quality");
        assert_eq!(
            serde_json::to_value(serde_json::from_value::<BlockMetadata>(old_quality).unwrap())
                .unwrap()["jpeg_quality"],
            0
        );
        for bad in [
            serde_json::json!([1., 2.]),
            serde_json::json!([1., "bad", 3.]),
        ] {
            let mut invalid = legacy.clone();
            invalid["trees"] = serde_json::json!([bad]);
            assert!(serde_json::from_value::<BlockMetadata>(invalid).is_err());
        }
    }
}

mod imagery_raster {

    use crate::build::imagery::*;
    use std::fs;

    #[test]
    fn block_resume_reuses_existing_tiles_and_preserves_gutters() {
        let temp = tempfile::tempdir().unwrap();
        let work = temp.path();
        fs::create_dir(work.join("downloads")).unwrap();
        let aerial = work.join("aerial");
        fs::create_dir(&aerial).unwrap();
        let b = blocks([35.16, 33.31, 35.161, 33.311]).remove(0);
        let side = CORE + 2 * PAD;
        let im = image::RgbImage::from_fn(side, side, |x, y| {
            image::Rgb([(40 + x % 128) as u8, (40 + y % 128) as u8, 80])
        });
        im.save(image_path(&work.join("downloads"), &b)).unwrap();
        let first =
            process_tiles(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap();
        assert_eq!(first.tiles.len(), 16);
        let a = fs::read(aerial.join(&first.tiles[0].name)).unwrap();
        let decoded = image::load_from_memory(&a).unwrap().to_rgb8().into_raw();
        let right = image::open(aerial.join(&first.tiles[1].name))
            .unwrap()
            .to_rgb8()
            .into_raw();
        let width = (TILE + 2 * GUTTER) as usize;
        for y in 0..width {
            let left = &decoded[(y * width + TILE as usize) * 3..(y + 1) * width * 3];
            let right = &right[y * width * 3..(y * width + 2 * GUTTER as usize) * 3];
            assert!(left.iter().zip(right).all(|(a, b)| a.abs_diff(*b) <= 12));
        }
        assert_eq!(
            first,
            process_tiles(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
        );
        fs::remove_file(aerial.join(&first.tiles[0].name)).unwrap();
        assert_eq!(
            first,
            process_tiles(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
        );
        assert_eq!(a, fs::read(aerial.join(&first.tiles[0].name)).unwrap());
    }
}

mod location_compilation {
    use crate::build;
    use crate::build::furniture::build_furniture;
    use crate::build::landscape::build_landscape;
    use crate::build::map::create_location_map;
    use crate::build::osm::OSMArea;
    use crate::build::osm::OSMElement;
    use crate::build::osm::OSMMap;
    use crate::build::osm_extract::extract_osm_boundary;
    use crate::model;
    use crate::model::source_location::SourceLocation;
    use crate::model::source_location::SourceRoad;
    use crate::physics;
    use crate::storage;
    use crate::test;
    use crate::util;
    use crate::util::error::Result;
    use crate::util::geometry::cross;
    use crate::util::geometry::polygon;
    use crate::util::geometry::sub;
    use crate::util::geometry::unit;
    use crate::util::geometry::BoundingBox;
    use crate::util::geometry::BoundingBoxXZ;
    use crate::util::geometry::XZ;
    use crate::world;
    use geo::{Area, BooleanOps, Contains, MultiPolygon, Point, Polygon};
    use serde_json::json;
    use std::collections::BTreeMap;
    use std::fs;
    use std::path::Path;

    struct TestOptions<'a> {
        source: &'a Path,
        output: &'a Path,
        name: &'a str,
        bbox: Option<[f64; 4]>,
        elevation: &'a Path,
    }

    // Rectangular crops keep synthetic fixtures on the same import path as real locations.
    fn fixture_boundary(source: &OSMMap, bbox: Option<[f64; 4]>) -> OSMArea {
        let [south, west, north, east] = bbox.or(source.bounds).expect("fixture bounds");
        OSMArea {
            geometry: MultiPolygon(vec![Polygon::new(
                vec![
                    (west, south),
                    (east, south),
                    (east, north),
                    (west, north),
                    (west, south),
                ]
                .into(),
                vec![],
            )]),
        }
    }

    fn fixture_map(source: &OSMMap, bbox: Option<[f64; 4]>) -> Result<SourceLocation<f64>> {
        create_location_map(source, &fixture_boundary(source, bbox))
    }

    fn compile(options: TestOptions<'_>) -> Result<()> {
        let source = build::osm::read(options.source)?;
        let boundary = fixture_boundary(&source, options.bbox);
        let map = create_location_map(&source, &boundary)?;
        let output = options.output;
        fs::create_dir_all(output.join("elevation"))?;
        let city = storage::source_files::write_city_txt(&map, &output.join("city.txt"))?.city;
        let surfaces =
            build::surfaces::write_streets_txt(&map, &output.join("streets.txt"), Some(&source))?
                .surfaces;
        build_landscape(&source, &map, &surfaces, &output.join("landscape.txt"))?;
        build_furniture(&source, &map, &surfaces, &output.join("furniture.txt"))?;
        build::terrain::build_terrain(
            &map,
            options.elevation,
            &output.join("elevation/heightfield.txt"),
            &output.join("elevation"),
        )?;
        fs::copy(options.source, output.join("source.osm"))?;
        fs::write(
            output.join("manifest.json"),
            serde_json::to_string_pretty(
                &json!({"name": options.name, "boundary": build::osm::to_geojson(&(boundary))}),
            )? + "\n",
        )?;
        build::collision::build_collisions(
            &output.join("streets.bin"),
            &city,
            fs::read_to_string(output.join("streets.txt"))?,
        )?;
        Ok(())
    }

    use crate::test::support::Temp;
    const FIXTURE: &str = r#"<osm version="0.6" timestamp="2026-01-01T00:00:00Z">
    <bounds minlat="35.160" minlon="33.318" maxlat="35.164" maxlon="33.322"/>
    <node id="1" lat="35.162" lon="33.319"/>
    <node id="2" lat="35.162" lon="33.320"><tag k="highway" v="traffic_signals"/><tag k="traffic_signals:direction" v="both"/></node>
    <node id="3" lat="35.162" lon="33.321"/>
    <node id="4" lat="35.163" lon="33.320"/>
    <node id="5" lat="35.163" lon="33.321"/>
    <node id="6" lat="35.163" lon="33.3215"/>
    <node id="7" lat="35.1635" lon="33.3215"/>
    <node id="8" lat="35.1635" lon="33.321"/>
    <node id="9" lat="35.161" lon="33.319"><tag k="natural" v="tree"/></node>
    <way id="101"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
    <way id="102"><nd ref="2"/><nd ref="4"/><tag k="highway" v="tertiary"/><tag k="width" v="8"/></way>
    <way id="103"><nd ref="3"/><nd ref="5"/><tag k="highway" v="service"/><tag k="access" v="private"/></way>
    <way id="104"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/><nd ref="5"/><tag k="building" v="yes"/><tag k="height" v="12"/></way>
    <way id="105"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/><nd ref="5"/><tag k="leisure" v="park"/></way>
    </osm>"#;

    #[test]
    fn raceways_allow_restricted_circuit_access_without_opening_private_streets() {
        let temp = test::support::new();
        let source = temp.directory.join("circuit.osm");
        let mut xml = FIXTURE.to_owned();
        for (id, highway, access) in [
            (201, "raceway", "private"),
            (202, "raceway", "no"),
            (203, "service", "private"),
            (204, "cycleway", "yes"),
        ] {
            xml = xml.replace("</osm>", &format!(
                "<way id=\"{id}\"><nd ref=\"1\"/><nd ref=\"3\"/><tag k=\"highway\" v=\"{highway}\"/><tag k=\"access\" v=\"{access}\"/></way></osm>"
            ));
        }
        xml = xml.replace("</osm>", "<way id=\"205\"><nd ref=\"2\"/><nd ref=\"3\"/><tag k=\"highway\" v=\"raceway\"/><tag k=\"bridge\" v=\"yes\"/><tag k=\"layer\" v=\"1\"/><tag k=\"oneway\" v=\"yes\"/></way></osm>");
        fs::write(&source, xml).unwrap();
        let map = fixture_map(&build::osm::read(&source).unwrap(), None).unwrap();
        for id in [201, 202] {
            let road = map
                .road_lines
                .iter()
                .find(|r| r.osm_way_id == Some(id))
                .unwrap();
            assert_eq!(road.road_width_start_meters, 10.0);
        }
        assert!(!map
            .road_lines
            .iter()
            .any(|r| r.osm_way_id.is_some_and(|id| [103, 203, 204].contains(&id))));
        assert_eq!(map.road_lines[0].osm_way_id, Some(201));
        let bridge = map
            .road_lines
            .iter()
            .find(|r| r.osm_way_id == Some(205))
            .unwrap();
        assert_eq!(bridge.direction, 1);
    }

    #[test]
    fn mapped_multipolygon_holes_stay_empty() {
        let temp = test::support::new();
        let tile = temp.directory.join("N35E033.hgt");
        fs::write(
            &tile,
            (0..65)
                .flat_map(|row| {
                    (0..65).flat_map(move |col| (100_i16 + row * 2 + col).to_be_bytes())
                })
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let source = temp.directory.join("holes.osm");
        let extra = r#"
    <node id="10" lat="35.16315" lon="33.32115"/>
    <node id="11" lat="35.16315" lon="33.32135"/>
    <node id="12" lat="35.16335" lon="33.32135"/>
    <node id="13" lat="35.16335" lon="33.32115"/>
    <way id="106"><nd ref="10"/><nd ref="11"/><nd ref="12"/><nd ref="13"/><nd ref="10"/><tag k="landuse" v="grass"/></way>
    <relation id="201"><member type="way" ref="105" role="outer"/><member type="way" ref="106" role="inner"/><tag k="type" v="multipolygon"/><tag k="leisure" v="park"/></relation>
    </osm>"#;
        fs::write(&source, FIXTURE.replace("</osm>", extra)).unwrap();
        let output = temp.directory.join("out");
        compile(TestOptions {
            source: &source,
            output: &output,
            name: "holes",
            bbox: None,
            elevation: &tile,
        })
        .unwrap();
        let landscape =
            test::support::landscape(&fs::read_to_string(output.join("landscape.txt")).unwrap())
                .unwrap();
        let area: f64 = model::world::ground_triangles(&landscape)
            .iter()
            .map(|t| {
                let p = t.points;
                cross(
                    sub(
                        XZ {
                            x: p[1].x as f64,
                            z: p[1].z as f64,
                        },
                        XZ {
                            x: p[0].x as f64,
                            z: p[0].z as f64,
                        },
                    ),
                    sub(
                        XZ {
                            x: p[2].x as f64,
                            z: p[2].z as f64,
                        },
                        XZ {
                            x: p[0].x as f64,
                            z: p[0].z as f64,
                        },
                    ),
                )
                .abs()
                    / 2.0
            })
            .sum();
        let metre_per_degree = std::f64::consts::PI / 180.0 * 6371000.0;
        let expected = (0.0005_f64.powi(2) - 0.0002_f64.powi(2))
            * metre_per_degree.powi(2)
            * 35.162_f64.to_radians().cos();
        assert!(
            (area - expected).abs() < 0.01,
            "expected courtyard area {expected}, got {area}"
        );
        assert!(
            model::world::ground_triangles(&(landscape))
                .iter()
                .all(|t| t.material == model::world::Material::Park),
            "relation members must not create their own ground cover"
        );
    }

    fn landscape_areas(path: &Path) -> [f64; 9] {
        use std::io::{BufRead, BufReader};
        let mut areas = [0.; 9];
        for line in BufReader::new(fs::File::open(path).unwrap()).lines() {
            let line = line.unwrap();
            let fields: Vec<_> = line.split_whitespace().collect();
            if fields.first() != Some(&"a") {
                continue;
            }
            assert_eq!(fields.len(), 8);
            let style: usize = fields[1].parse().unwrap();
            let points: Vec<f64> = fields[2..].iter().map(|v| v.parse().unwrap()).collect();
            let area = cross(
                XZ {
                    x: points[2] - points[0],
                    z: points[3] - points[1],
                },
                XZ {
                    x: points[4] - points[0],
                    z: points[5] - points[1],
                },
            )
            .abs()
                / 2.;
            assert!(area.is_finite());
            areas[style] += area;
        }
        areas
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
        storage::source_files::write_city_txt(map, &temp.directory.join("city.txt")).unwrap();
        let s = build::surfaces::write_streets_txt(map, &temp.directory.join("streets.txt"), None)
            .unwrap();
        let loaded = world::street_surface::load(
            &std::fs::read_to_string(temp.directory.join("streets.txt")).unwrap(),
            map.road_lines.len(),
        )
        .unwrap()
        .0;
        (temp, s.surfaces, loaded)
    }
    fn point(p: XZ) -> Point<f64> {
        Point::new(p.x, p.z)
    }

    fn element(id: i64, tags: &[(&str, &str)]) -> OSMElement {
        test::support::osm_element(id, tags)
    }

    fn source() -> OSMMap {
        OSMMap {
            nodes: BTreeMap::new(),
            ways: BTreeMap::new(),
            relations: vec![],
            bounds: Some([35.160, 33.318, 35.164, 33.322]),
        }
    }
    fn node(s: &mut OSMMap, id: i64, p: XZ, tags: &[(&str, &str)]) {
        let mut e = element(id, tags);
        e.coord = Some(util::geometry::new(
            35.162 - p.z.to_degrees() / 6371000.,
            33.320 + p.x.to_degrees() / 6371000. / 35.162_f64.to_radians().cos(),
        ));
        s.nodes.insert(id, e);
    }
    fn way(s: &mut OSMMap, id: i64, nodes: &[i64], tags: &[(&str, &str)]) {
        let mut e = element(id, tags);
        e.nodes = nodes.to_vec();
        s.ways.insert(id, e);
    }

    #[test]
    fn mapped_controls_keep_direction_speed_and_left_curb_semantics() {
        use crate::model::traffic_controls::FurnitureKind::*;
        let f = controls(false, true);
        assert_eq!(f.iter().filter(|f| f.kind == Signal).count(), 2);
        assert_eq!(f.iter().filter(|f| f.kind == Stop).count(), 1);
        assert_eq!(f.iter().filter(|f| f.kind == Speed).count(), 1);
        assert_eq!(f.iter().find(|f| f.kind == Speed).unwrap().value, 30);
        for f in &f {
            assert!(f.coords.x * f.travel.z > 0.);
            assert!(f.coords.x.abs() > 3.25);
        }
        let f = controls(true, true);
        assert_eq!(f.iter().filter(|f| f.kind == Signal).count(), 1);
        assert!(f.iter().all(|f| f.travel.z > 0.));
        assert!(controls(false, false).is_empty());
    }
    #[test]
    fn landscape_priorities_crop_and_tree_conflicts_are_preserved() {
        let mut s = source();
        let mut next = 10;
        for (id, points, tags) in [
            (
                100,
                vec![[10., 10.], [80., 10.], [80., 80.], [10., 80.]],
                vec![("leisure", "park")],
            ),
            (
                101,
                vec![[30., 30.], [50., 30.], [50., 50.], [30., 50.]],
                vec![("leisure", "pitch")],
            ),
            (
                102,
                vec![[-110., -10.], [15., -10.], [15., 10.], [-110., 10.]],
                vec![("amenity", "parking")],
            ),
        ] {
            let mut refs = vec![];
            for p in points {
                node(&mut s, next, XZ { x: p[0], z: p[1] }, &[]);
                refs.push(next);
                next += 1;
            }
            refs.push(refs[0]);
            way(&mut s, id, &refs, &tags);
        }
        node(
            &mut s,
            1,
            XZ { x: -70., z: -70. },
            &[("natural", "tree"), ("height", "8")],
        );
        node(&mut s, 2, XZ { x: 0., z: 0. }, &[("natural", "tree")]);
        node(&mut s, 3, XZ { x: 60., z: 60. }, &[("natural", "tree")]);
        node(&mut s, 5001, XZ { x: -50., z: 0. }, &[]);
        node(&mut s, 5002, XZ { x: 50., z: 0. }, &[]);
        way(&mut s, 1, &[5001, 5002], &[("highway", "residential")]);
        let m = map(
            &[XZ { x: -50., z: 0. }, XZ { x: 50., z: 0. }],
            &[(0, 1, 8., 1)],
            &[vec![
                XZ { x: 55., z: 55. },
                XZ { x: 65., z: 55. },
                XZ { x: 65., z: 65. },
                XZ { x: 55., z: 65. },
            ]],
        );
        let (temp, surfaces, _) = surfaces(&m);
        build_landscape(&s, &m, &surfaces, &temp.directory.join("landscape.txt")).unwrap();
        println!("  Placing road controls...");
        build_furniture(&s, &m, &surfaces, &temp.directory.join("furniture.txt")).unwrap();
        let l = test::support::landscape(
            &std::fs::read_to_string(temp.directory.join("landscape.txt")).unwrap(),
        )
        .unwrap();
        assert_eq!(model::world::trees(&(l)).len(), 1);
        assert_eq!(model::world::trees(&(l))[0].height_meters, 8.);
        let mut sports = 0.;
        for t in model::world::ground_triangles(&l) {
            let points: Vec<XZ> = t
                .points
                .iter()
                .map(|p| XZ {
                    x: p.x as f64,
                    z: p.z as f64,
                })
                .collect();
            assert!(points.iter().all(|p| p.x >= -100.001 && p.x <= 100.001));
            let poly = polygon(&points);
            if t.material == model::world::Material::Sports {
                sports += poly.unsigned_area();
            } else {
                assert!(!poly.contains(&point(XZ { x: 40., z: 40. })));
            }
        }
        assert!((sports - 400.).abs() < 0.01);
    }

    #[test]
    fn multipolygon_outers_with_opposite_winding_are_unioned_without_cancellation() {
        let mut s = source();
        node(&mut s, 1, XZ { x: -50., z: 0. }, &[]);
        node(&mut s, 2, XZ { x: 50., z: 0. }, &[]);
        way(&mut s, 1, &[1, 2], &[("highway", "residential")]);
        let mut next = 10;
        for (id, points) in [
            (100, vec![[10., 30.], [20., 30.], [20., 40.], [10., 40.]]),
            (101, vec![[15., 30.], [15., 40.], [25., 40.], [25., 30.]]),
        ] {
            let mut refs = vec![];
            for p in points {
                node(&mut s, next, XZ { x: p[0], z: p[1] }, &[]);
                refs.push(next);
                next += 1;
            }
            refs.push(refs[0]);
            way(&mut s, id, &refs, &[]);
        }
        let mut relation = element(500, &[("type", "multipolygon"), ("leisure", "park")]);
        relation.members = vec![
            ("way".into(), 100, "outer".into()),
            ("way".into(), 101, "outer".into()),
        ];
        s.relations.push(relation);
        let m = map(
            &[XZ { x: -50., z: 0. }, XZ { x: 50., z: 0. }],
            &[(0, 1, 8., 1)],
            &[],
        );
        let (temp, surfaces, _) = surfaces(&m);
        build_landscape(&s, &m, &surfaces, &temp.directory.join("landscape.txt")).unwrap();
        println!("  Placing road controls...");
        build_furniture(&s, &m, &surfaces, &temp.directory.join("furniture.txt")).unwrap();
        let areas = landscape_areas(&temp.directory.join("landscape.txt"));
        assert!((areas[1] - 150.).abs() < 0.01);
    }

    #[test]
    fn adjacent_tiles_are_filtered_continuously_across_the_seam() {
        let temp = test::support::new();
        for (x, offset) in [(6, 0), (7, 8)] {
            let values: Vec<u8> = (0..9)
                .flat_map(|_| (0..9).flat_map(|c| (100_i16 + offset + c).to_be_bytes()))
                .collect();
            fs::write(temp.directory.join(format!("N50E{x:03}.hgt")), values).unwrap();
        }
        let map = SourceLocation {
            boundary: None,
            origin: Some(util::geometry::new(50.5, 7.0)),
            bounding_box: BoundingBoxXZ {
                min: XZ {
                    x: -100.0,
                    z: -100.0,
                },
                max: XZ { x: 100.0, z: 100.0 },
            },
            road_nodes: vec![],
            road_lines: vec![],
            buildings: vec![],
            ..model::source_location::empty::<f64>()
        };
        let source = temp.directory.join("N50E006.hgt");
        let (dem, sources) = build::hgt::covering(&source, &map).unwrap();
        assert_eq!(sources.len(), 2);
        for longitude in [6.95, 7.0, 7.05] {
            let expected = 100.0 + (longitude - 6.0) * 8.0;
            assert!(
                (build::hgt::sample(&(dem), util::geometry::new(50.5, longitude)).unwrap()
                    - expected)
                    .abs()
                    < 1e-9
            );
        }
        fs::remove_file(temp.directory.join("N50E007.hgt")).unwrap();
        assert!(build::hgt::covering(&source, &map).is_err());
    }

    fn var(mut n: u64) -> Vec<u8> {
        let mut b = vec![];
        while n >= 128 {
            b.push(n as u8 | 128);
            n >>= 7;
        }
        b.push(n as u8);
        b
    }
    fn uint(field: u64, n: u64) -> Vec<u8> {
        [var(field << 3), var(n)].concat()
    }
    fn bytes(field: u64, b: &[u8]) -> Vec<u8> {
        [var(field << 3 | 2), var(b.len() as u64), b.to_vec()].concat()
    }
    fn zig(n: i64) -> u64 {
        ((n << 1) ^ (n >> 63)) as u64
    }
    fn pbf_fixture(path: &Path, incomplete_relation: bool) {
        let mut table = vec![];
        for s in ["", "type", "multipolygon", "outer", "name", "A & B"] {
            table.extend(bytes(1, s.as_bytes()));
        }
        let mut group = vec![];
        for (id, lat, lon) in [
            (1, 0, 0),
            (2, 20_000_000, 20_000_000),
            (3, 30_000_000, 30_000_000),
            (4, 40_000_000, 40_000_000),
        ] {
            let node = [
                uint(1, zig(id)),
                bytes(2, &var(4)),
                bytes(3, &var(5)),
                bytes(4, &uint(1, 7)),
                uint(8, zig(lat)),
                uint(9, zig(lon)),
            ]
            .concat();
            group.extend(bytes(1, &node));
        }
        for (id, refs) in [(10, vec![1, 2]), (11, vec![3, 4])] {
            let mut last = 0;
            let mut packed = vec![];
            for n in refs {
                packed.extend(var(zig(n - last)));
                last = n;
            }
            group.extend(bytes(
                3,
                &[uint(1, id), bytes(8, &packed), bytes(4, &uint(1, 3))].concat(),
            ));
        }
        let relation = [
            uint(1, 20),
            bytes(2, &var(1)),
            bytes(3, &var(2)),
            bytes(8, &[var(3), var(3)].concat()),
            bytes(9, &[var(zig(10)), var(zig(1))].concat()),
            bytes(10, &[1, 1]),
        ]
        .concat();
        group.extend(bytes(4, &relation));
        if incomplete_relation {
            // A cross-border relation references one local way, an absent way,
            // and an absent node. The complete relation above must still survive.
            let relation = [
                uint(1, 21),
                bytes(2, &var(1)),
                bytes(3, &var(2)),
                bytes(8, &[var(3), var(3), var(0)].concat()),
                bytes(9, &[var(zig(10)), var(zig(2)), var(zig(9987))].concat()),
                bytes(10, &[1, 1, 0]),
            ]
            .concat();
            group.extend(bytes(4, &relation));
        }
        let block = [bytes(1, &table), bytes(2, &group)].concat();
        let blob = bytes(1, &block);
        let header = [bytes(1, b"OSMData"), uint(3, blob.len() as u64)].concat();
        let data = [(header.len() as u32).to_be_bytes().to_vec(), header, blob].concat();
        fs::write(path, data).unwrap();
    }
    #[test]
    fn extraction_completes_outside_ways_and_multipolygons() {
        check_extraction(false);
    }

    #[test]
    fn extraction_skips_incomplete_regional_multipolygons() {
        check_extraction(true);
    }

    fn check_extraction(incomplete_relation: bool) {
        let dir = tempfile::tempdir().unwrap();
        let pbf = dir.path().join("source.pbf");
        let output = dir.path().join("region.osm");
        pbf_fixture(&pbf, incomplete_relation);
        extract_osm_boundary(
            &pbf,
            &output,
            BoundingBox {
                min_lat: -0.1,
                min_lon: -0.1,
                max_lat: 0.1,
                max_lon: 0.1,
            },
        )
        .unwrap();
        let source = build::osm::read(&output).unwrap();
        assert_eq!(
            source.nodes.keys().copied().collect::<Vec<_>>(),
            vec![1, 2, 3, 4]
        );
        assert_eq!(
            source.ways.keys().copied().collect::<Vec<_>>(),
            vec![10, 11]
        );
        assert_eq!(source.relations.len(), 1);
        assert_eq!(source.relations[0].id, 20);
        assert_eq!(build::osm::tag(&(source.nodes[&1]), "name"), "A & B");
        let xml = fs::read_to_string(&output).unwrap();
        assert!(xml.contains("version=\"7\""));
        assert!(xml.contains("A &amp; B"));
        let report: serde_json::Value =
            serde_json::from_slice(&fs::read(output.with_extension("json")).unwrap()).unwrap();
        assert_eq!(report["nodes"], 4);
        assert_eq!(report["multipolygons"], 1);
        assert_eq!(
            report["skipped_multipolygons"],
            usize::from(incomplete_relation)
        );
    }

    fn controls(oneway: bool, enabled: bool) -> Vec<model::traffic_controls::Furniture> {
        let mut s = source();
        for (id, z) in [(1, -40.), (5, -15.), (2, 0.), (4, 15.), (3, 40.)] {
            let tags = if enabled {
                match id {
                    2 => vec![
                        ("highway", "traffic_signals"),
                        ("traffic_signals:direction", "both"),
                    ],
                    3 => vec![("highway", "stop"), ("direction", "forward")],
                    4 => vec![
                        ("traffic_sign", "maxspeed"),
                        ("maxspeed", "30"),
                        ("traffic_sign:direction", "forward"),
                    ],
                    5 => vec![("traffic_sign", "maxspeed")],
                    _ => vec![],
                }
            } else {
                vec![]
            };
            node(&mut s, id, XZ { x: 0., z }, &tags);
        }
        let mut tags = vec![
            ("highway", "residential"),
            ("width", "6.5"),
            ("maxspeed", "50"),
        ];
        if oneway {
            tags.push(("oneway", "yes"));
        }
        way(&mut s, 101, &[1, 5, 2, 4, 3], &tags);
        let m = fixture_map(&s, None).unwrap();
        let (temp, surfaces, _) = surfaces(&m);
        build_landscape(&s, &m, &surfaces, &temp.directory.join("landscape.txt")).unwrap();
        println!("  Placing road controls...");
        build_furniture(&s, &m, &surfaces, &temp.directory.join("furniture.txt")).unwrap();
        storage::codec_furniture::parse(
            &std::fs::read_to_string(temp.directory.join("furniture.txt")).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn sidewalk_tags_reach_compiled_pavement_on_the_original_way_side() {
        struct Case {
            tags: Vec<(&'static str, &'static str)>,
            left: bool,
            right: bool,
        }
        let cases = [
            Case {
                tags: vec![],
                left: true,
                right: true,
            },
            Case {
                tags: vec![("sidewalk", "left")],
                left: true,
                right: false,
            },
            Case {
                tags: vec![("sidewalk", "right"), ("oneway", "-1")],
                left: false,
                right: true,
            },
            Case {
                tags: vec![("sidewalk", "no")],
                left: false,
                right: false,
            },
            Case {
                tags: vec![("sidewalk", "separate")],
                left: false,
                right: false,
            },
            Case {
                tags: vec![("sidewalk", "both"), ("sidewalk:right", "no")],
                left: true,
                right: false,
            },
            Case {
                tags: vec![
                    ("sidewalk", "no"),
                    ("sidewalk:both", "yes"),
                    ("sidewalk:left", "separate"),
                ],
                left: false,
                right: true,
            },
        ];
        for case in cases {
            let mut source = source();
            node(&mut source, 1, XZ { x: -40., z: 0. }, &[]);
            node(&mut source, 2, XZ { x: 40., z: 0. }, &[]);
            let mut tags = case.tags.clone();
            tags.extend([("highway", "residential"), ("width", "8")]);
            way(&mut source, 10, &[1, 2], &tags);
            let map = fixture_map(&source, None).unwrap();
            let (temp, _, compiled) = surfaces(&map);
            let parsed = model::source_location::parse(
                &fs::read_to_string(temp.directory.join("city.txt")).unwrap(),
            )
            .unwrap();
            assert_eq!(parsed.road_lines[0].sidewalks.left, case.left);
            assert_eq!(parsed.road_lines[0].sidewalks.right, case.right);
            let pavement_at = |z| {
                world::street_surface::road_triangles(&compiled)
                    .iter()
                    .any(|t| {
                        matches!(t.usage, model::world::TriangleUsage::Pavement(_))
                            && polygon(&t.points.map(|p| XZ {
                                x: p.x as f64,
                                z: p.z as f64,
                            }))
                            .contains(&point(XZ { x: 1.23, z }))
                    })
            };
            assert_eq!(pavement_at(-4.7), case.left, "{:?}", case.tags);
            assert_eq!(pavement_at(4.7), case.right, "{:?}", case.tags);
        }
    }

    #[test]
    fn mapped_paths_preserve_width_surface_crop_and_clear_roads_and_buildings() {
        let mut source = source();
        let mut next = 100;
        let mut add_way = |id, points: &[XZ], tags: &[(&str, &str)]| {
            let mut refs = Vec::new();
            for (i, p) in points.iter().enumerate() {
                if i > 0 && i == points.len() - 1 && points.first() == points.last() {
                    refs.push(refs[0]);
                    continue;
                }
                node(&mut source, next, *p, &[]);
                refs.push(next);
                next += 1;
            }
            way(&mut source, id, &refs, tags);
        };
        add_way(
            10,
            &[
                XZ { x: -100., z: -100. },
                XZ { x: 100., z: -100. },
                XZ { x: 100., z: 100. },
                XZ { x: -100., z: 100. },
                XZ { x: -100., z: -100. },
            ],
            &[("leisure", "park")],
        );
        add_way(
            11,
            &[XZ { x: -120., z: 20. }, XZ { x: 120., z: 20. }],
            &[("highway", "footway"), ("width", "4 m")],
        );
        add_way(
            12,
            &[XZ { x: -80., z: 30. }, XZ { x: 80., z: 30. }],
            &[("highway", "path"), ("surface", "compacted")],
        );
        add_way(
            13,
            &[XZ { x: -80., z: 40. }, XZ { x: 80., z: 40. }],
            &[("highway", "pedestrian"), ("surface", "concrete")],
        );
        add_way(
            14,
            &[XZ { x: 30., z: -20. }, XZ { x: 30., z: 60. }],
            &[
                ("highway", "footway"),
                ("surface", "asphalt"),
                ("width", "2"),
            ],
        );
        add_way(
            15,
            &[
                XZ { x: 56., z: 16. },
                XZ { x: 64., z: 16. },
                XZ { x: 64., z: 24. },
                XZ { x: 56., z: 24. },
                XZ { x: 56., z: 16. },
            ],
            &[("natural", "water")],
        );
        node(
            &mut source,
            1,
            XZ { x: 20., z: 20. },
            &[("natural", "tree")],
        );
        node(
            &mut source,
            2,
            XZ { x: 20., z: 25. },
            &[("natural", "tree")],
        );
        let map = map(
            &[XZ { x: -80., z: 0. }, XZ { x: 80., z: 0. }],
            &[(0, 1, 8., 1)],
            &[vec![
                XZ { x: -4., z: 16. },
                XZ { x: 4., z: 16. },
                XZ { x: 4., z: 24. },
                XZ { x: -4., z: 24. },
            ]],
        );
        let (temp, surfaces, _) = surfaces(&map);
        let path = temp.directory.join("landscape.txt");
        build_landscape(&source, &map, &surfaces, &path).unwrap();
        let landscape = test::support::landscape(&fs::read_to_string(path).unwrap()).unwrap();
        let material_at = |x, z| model::world::ground_material(&landscape, XZ { x, z });
        assert_eq!(material_at(20., 21.8), Some(model::world::Material::Paving));
        assert_eq!(material_at(20., 22.2), Some(model::world::Material::Park));
        assert_eq!(material_at(99., 20.), Some(model::world::Material::Paving));
        assert_eq!(material_at(101., 20.), None);
        assert_eq!(material_at(0., 20.), Some(model::world::Material::Park));
        assert_eq!(material_at(30., 0.), Some(model::world::Material::Park));
        assert_eq!(
            material_at(20., 30.5),
            Some(model::world::Material::BareGround)
        );
        assert_eq!(material_at(20., 30.8), Some(model::world::Material::Park));
        assert_eq!(
            material_at(20., 41.4),
            Some(model::world::Material::Concrete)
        );
        assert_eq!(material_at(20., 41.7), Some(model::world::Material::Park));
        assert_eq!(material_at(30., 10.), Some(model::world::Material::Asphalt));
        assert_eq!(material_at(60., 20.), Some(model::world::Material::Water));
        assert_eq!(
            model::world::trees(&(landscape))
                .iter()
                .map(|tree| tree.id)
                .collect::<Vec<_>>(),
            vec![2]
        );
        let mut area = 0.;
        for triangle in model::world::ground_triangles(&landscape) {
            let geometry = polygon(&triangle.points.map(|p| XZ {
                x: p.x as f64,
                z: p.z as f64,
            }));
            area += geometry.unsigned_area();
        }
        assert!(
            (area - 40000.).abs() < 0.01,
            "ground layers overlap or leave holes: {area}"
        );
    }

    #[test]
    fn closed_paths_stay_rings_and_elevated_paths_are_not_flattened() {
        for tags in [
            vec![],
            vec![("bridge", "yes")],
            vec![("tunnel", "yes")],
            vec![("layer", "1")],
        ] {
            let mut source = source();
            let points = [
                XZ { x: 10., z: 10. },
                XZ { x: 50., z: 10. },
                XZ { x: 50., z: 50. },
                XZ { x: 10., z: 50. },
            ];
            for (i, p) in points.iter().enumerate() {
                node(&mut source, i as i64 + 1, *p, &[]);
            }
            let mut way_tags = tags.clone();
            way_tags.extend([("highway", "footway"), ("width", "2")]);
            way(&mut source, 10, &[1, 2, 3, 4, 1], &way_tags);
            let map = map(
                &[XZ { x: -80., z: 0. }, XZ { x: 80., z: 0. }],
                &[(0, 1, 8., 1)],
                &[],
            );
            let (temp, surfaces, _) = surfaces(&map);
            let path = temp.directory.join("landscape.txt");
            build_landscape(&source, &map, &surfaces, &path).unwrap();
            let landscape = test::support::landscape(&fs::read_to_string(path).unwrap()).unwrap();
            assert_eq!(
                model::world::ground_material(&(landscape), XZ { x: 30., z: 30. }),
                None
            );
            assert_eq!(
                model::world::ground_material(&(landscape), XZ { x: 30., z: 10. }),
                if tags.is_empty() {
                    Some(model::world::Material::Paving)
                } else {
                    None
                }
            );
        }
    }

    #[test]
    fn mapped_crossing_styles_and_overrides_survive_street_export() {
        struct Case {
            node_tags: Vec<(&'static str, &'static str)>,
            way_tags: Vec<(&'static str, &'static str)>,
            zebra: bool,
            lines: bool,
        }
        let cases = [
            Case {
                node_tags: vec![("crossing:markings", "zebra")],
                way_tags: vec![],
                zebra: true,
                lines: false,
            },
            Case {
                node_tags: vec![("crossing", "uncontrolled")],
                way_tags: vec![],
                zebra: true,
                lines: false,
            },
            Case {
                node_tags: vec![("crossing:markings", "lines")],
                way_tags: vec![],
                zebra: false,
                lines: true,
            },
            Case {
                node_tags: vec![("crossing:markings", "dashes")],
                way_tags: vec![],
                zebra: false,
                lines: true,
            },
            Case {
                node_tags: vec![("crossing:markings", "yes")],
                way_tags: vec![("crossing:markings", "lines")],
                zebra: false,
                lines: true,
            },
            Case {
                node_tags: vec![("crossing:markings", "no"), ("crossing", "zebra")],
                way_tags: vec![("crossing:markings", "zebra")],
                zebra: false,
                lines: false,
            },
            Case {
                node_tags: vec![("crossing", "unmarked")],
                way_tags: vec![("crossing:markings", "zebra")],
                zebra: false,
                lines: false,
            },
            Case {
                node_tags: vec![("crossing", "traffic_signals")],
                way_tags: vec![],
                zebra: false,
                lines: false,
            },
            Case {
                node_tags: vec![("crossing:markings", "unrecognised")],
                way_tags: vec![],
                zebra: false,
                lines: false,
            },
        ];
        for case in cases {
            let mut source = source();
            node(&mut source, 1, XZ { x: -40., z: 0. }, &[]);
            let mut tags = case.node_tags.clone();
            tags.push(("highway", "crossing"));
            node(&mut source, 2, XZ { x: 0., z: 0. }, &tags);
            node(&mut source, 3, XZ { x: 40., z: 0. }, &[]);
            node(&mut source, 4, XZ { x: 0., z: -10. }, &[]);
            node(&mut source, 5, XZ { x: 0., z: 10. }, &[]);
            way(
                &mut source,
                100,
                &[1, 2, 3],
                &[("highway", "residential"), ("width", "8")],
            );
            let mut tags = case.way_tags.clone();
            tags.extend([("highway", "footway"), ("footway", "crossing")]);
            way(&mut source, 101, &[4, 2, 5], &tags);
            let map = fixture_map(&source, None).unwrap();
            let temp = test::support::new();
            let path = temp.directory.join("streets.txt");
            build::surfaces::write_streets_txt(&map, &path, Some(&source)).unwrap();
            let compiled = world::street_surface::load(
                &fs::read_to_string(path).unwrap(),
                map.road_lines.len(),
            )
            .unwrap()
            .0;
            let mut paint_area = 0.;
            let mut paint_polygons = Vec::new();
            for triangle in world::street_surface::road_triangles(&compiled)
                .iter()
                .filter(|triangle| triangle.usage == model::world::TriangleUsage::Marking)
            {
                let geometry = polygon(&triangle.points.map(|point| XZ {
                    x: point.x as f64,
                    z: point.z as f64,
                }));
                paint_area += geometry.unsigned_area();
                paint_polygons.push(MultiPolygon(vec![geometry]));
            }
            let paint = build::surfaces::union(&paint_polygons);
            assert_eq!(
                paint.contains(&point(XZ { x: 0.4, z: 1.9 })),
                case.zebra,
                "{:?}",
                case.node_tags
            );
            assert_eq!(
                paint.contains(&point(XZ { x: 1.4, z: 1.9 })),
                case.zebra || case.lines,
                "{:?}",
                case.node_tags
            );
            if case.lines {
                // The ordinary centre line must not run through the crossing's interior.
                assert!(!paint.contains(&point(XZ { x: -0.4, z: 0. })));
            }
            assert!(
                (paint_area - paint.unsigned_area()).abs() < 0.001,
                "overlapping paint triangles"
            );
        }
    }

    #[test]
    fn skew_crossing_paths_keep_their_angle_and_paint_near_junctions() {
        let mut source = source();
        for (id, p) in [
            (1, XZ { x: -40., z: 0. }),
            (2, XZ { x: -5., z: 0. }),
            (3, XZ { x: 0., z: 0. }),
            (4, XZ { x: 40., z: 0. }),
            (5, XZ { x: 0., z: -40. }),
            (6, XZ { x: -11., z: -10. }),
            (7, XZ { x: 1., z: 10. }),
        ] {
            node(&mut source, id, p, &[]);
        }
        way(
            &mut source,
            100,
            &[1, 2, 3, 4],
            &[("highway", "residential"), ("width", "8")],
        );
        way(
            &mut source,
            101,
            &[3, 5],
            &[("highway", "residential"), ("width", "8")],
        );
        // A crossing way supplies markings even when its shared road node has no tags.
        way(
            &mut source,
            102,
            &[6, 2, 7],
            &[
                ("highway", "footway"),
                ("footway", "crossing"),
                ("crossing:markings", "zebra"),
            ],
        );
        let map = fixture_map(&source, None).unwrap();
        let temp = test::support::new();
        let path = temp.directory.join("streets.txt");
        build::surfaces::write_streets_txt(&map, &path, Some(&source)).unwrap();
        let compiled =
            world::street_surface::load(&fs::read_to_string(path).unwrap(), map.road_lines.len())
                .unwrap()
                .0;
        let mut paint = Vec::new();
        let mut road = Vec::new();
        for triangle in world::street_surface::road_triangles(&compiled) {
            let geometry = MultiPolygon(vec![polygon(&triangle.points.map(|p| XZ {
                x: p.x as f64,
                z: p.z as f64,
            }))]);
            if triangle.usage == model::world::TriangleUsage::Marking {
                paint.push(geometry);
            } else if model::world::is_road(triangle.usage) {
                road.push(geometry);
            }
        }
        let paint = build::surfaces::union(&paint);
        let road = build::surfaces::union(&road);
        let across = unit(XZ { x: 0.6, z: 1.0 });
        assert!(paint.contains(&point(XZ {
            x: -5. + across.x * 1.8,
            z: across.z * 1.8
        })));
        assert!(!paint.contains(&point(XZ {
            x: -5. + across.x * 1.35,
            z: across.z * 1.35
        })));
        assert!(paint.difference(&road).unsigned_area() < 0.001);
    }

    #[test]
    fn excluded_crossing_roads_do_not_paint_nearby_parallel_streets() {
        for restriction in [("bridge", "yes"), ("access", "private")] {
            let mut source = source();
            for (id, p) in [
                (1, XZ { x: -40., z: 0. }),
                (2, XZ { x: 0., z: 0. }),
                (3, XZ { x: 40., z: 0. }),
                (4, XZ { x: -40., z: 3. }),
                (5, XZ { x: 40., z: 3. }),
            ] {
                node(&mut source, id, p, &[]);
            }
            test::support::set_osm_tags(
                source.nodes.get_mut(&2).unwrap(),
                &[("highway", "crossing"), ("crossing:markings", "zebra")],
            );
            way(
                &mut source,
                100,
                &[1, 2, 3],
                &[("highway", "residential"), restriction],
            );
            way(
                &mut source,
                101,
                &[4, 5],
                &[("highway", "residential"), ("width", "7")],
            );
            let map = fixture_map(&source, None).unwrap();
            let temp = test::support::new();
            let path = temp.directory.join("streets.txt");
            build::surfaces::write_streets_txt(&map, &path, Some(&source)).unwrap();
            let compiled = world::street_surface::load(
                &fs::read_to_string(path).unwrap(),
                map.road_lines.len(),
            )
            .unwrap()
            .0;
            assert!(world::street_surface::road_triangles(&(compiled))
                .iter()
                .all(|triangle| triangle.usage != model::world::TriangleUsage::Marking));
        }
    }

    #[test]
    fn entrance_nodes_stay_with_their_buildings_when_footprint_winding_changes() {
        let mut source = source();
        node(&mut source, 1, XZ { x: -40., z: -20. }, &[]);
        node(&mut source, 2, XZ { x: 40., z: -20. }, &[]);
        way(&mut source, 300, &[1, 2], &[("highway", "residential")]);
        struct MappedNode {
            id: i64,
            position: XZ,
        }
        for mapped in [
            MappedNode {
                id: 10,
                position: XZ { x: 0., z: 0. },
            },
            MappedNode {
                id: 11,
                position: XZ { x: 4., z: 0. },
            },
            MappedNode {
                id: 12,
                position: XZ { x: 12., z: 0. },
            },
            MappedNode {
                id: 13,
                position: XZ { x: 12., z: 10. },
            },
            MappedNode {
                id: 14,
                position: XZ { x: 0., z: 10. },
            },
            MappedNode {
                id: 15,
                position: XZ { x: 12., z: 6. },
            },
            MappedNode {
                id: 20,
                position: XZ { x: 20., z: 0. },
            },
            MappedNode {
                id: 21,
                position: XZ { x: 24., z: 0. },
            },
            MappedNode {
                id: 22,
                position: XZ { x: 30., z: 0. },
            },
            MappedNode {
                id: 23,
                position: XZ { x: 30., z: 10. },
            },
            MappedNode {
                id: 24,
                position: XZ { x: 20., z: 10. },
            },
            MappedNode {
                id: 30,
                position: XZ { x: 6., z: 5. },
            },
        ] {
            node(&mut source, mapped.id, mapped.position, &[]);
        }
        test::support::set_osm_tags(source.nodes.get_mut(&11).unwrap(), &[("entrance", "main")]);
        test::support::set_osm_tags(
            source.nodes.get_mut(&15).unwrap(),
            &[("entrance", "service")],
        );
        test::support::set_osm_tags(source.nodes.get_mut(&21).unwrap(), &[("entrance", "no")]);
        test::support::set_osm_tags(source.nodes.get_mut(&30).unwrap(), &[("entrance", "yes")]);
        way(
            &mut source,
            100,
            &[10, 11, 12, 15, 13, 14, 10],
            &[("building", "yes")],
        );
        way(
            &mut source,
            200,
            &[20, 21, 22, 23, 24, 20],
            &[("building", "yes")],
        );
        let map = fixture_map(&source, None).unwrap();
        assert_eq!(map.entrances.len(), 2);
        assert!(map
            .entrances
            .iter()
            .all(|entrance| entrance.building_id == 100));
        assert!(map
            .entrances
            .iter()
            .any(|entrance| (entrance.position.x - 4.).abs() < 0.001
                && entrance.position.z.abs() < 0.001));
        source.ways.get_mut(&100).unwrap().nodes.reverse();
        let reversed = fixture_map(&source, None).unwrap();
        assert_eq!(reversed.entrances, map.entrances);
        let temp = test::support::new();
        let parsed = storage::source_files::write_city_txt(&map, &temp.directory.join("city.txt"))
            .unwrap()
            .city;
        assert_eq!(parsed.entrances.len(), 2);
        assert_eq!(parsed.entrances[0].building_id, 100);
    }

    #[test]
    fn mapped_building_parts_replace_parent_roofs_and_keep_partial_coverage() {
        struct Outline {
            id: i64,
            min_x: f64,
            max_x: f64,
            min_z: f64,
            max_z: f64,
            tags: Vec<(&'static str, &'static str)>,
        }
        for inset in [false, true] {
            let mut source = source();
            node(&mut source, 1, XZ { x: -40., z: -20. }, &[]);
            node(&mut source, 2, XZ { x: 40., z: -20. }, &[]);
            way(&mut source, 300, &[1, 2], &[("highway", "residential")]);
            let mut outlines = vec![Outline {
                id: 100,
                min_x: 0.,
                max_x: 20.,
                min_z: 0.,
                max_z: 10.,
                tags: vec![("building", "yes"), ("height", "18")],
            }];
            if inset {
                outlines.push(Outline {
                    id: 101,
                    min_x: 5.,
                    max_x: 15.,
                    min_z: 2.,
                    max_z: 8.,
                    tags: vec![
                        ("building:part", "yes"),
                        ("height", "24"),
                        ("building:min_level", "2"),
                    ],
                });
            } else {
                outlines.extend([
                    Outline {
                        id: 101,
                        min_x: 0.,
                        max_x: 10.,
                        min_z: 0.,
                        max_z: 10.,
                        tags: vec![("building:part", "yes"), ("building:levels", "1")],
                    },
                    Outline {
                        id: 102,
                        min_x: 10.,
                        max_x: 20.,
                        min_z: 0.,
                        max_z: 10.,
                        tags: vec![("building:part", "yes"), ("building:levels", "3")],
                    },
                ]);
            }
            let mut next = 10;
            for outline in outlines {
                let mut nodes = Vec::new();
                for point in [
                    XZ {
                        x: outline.min_x,
                        z: outline.min_z,
                    },
                    XZ {
                        x: outline.max_x,
                        z: outline.min_z,
                    },
                    XZ {
                        x: outline.max_x,
                        z: outline.max_z,
                    },
                    XZ {
                        x: outline.min_x,
                        z: outline.max_z,
                    },
                ] {
                    node(&mut source, next, point, &[]);
                    nodes.push(next);
                    next += 1;
                }
                nodes.push(nodes[0]);
                way(&mut source, outline.id, &nodes, &outline.tags);
            }
            let map = fixture_map(&source, None).unwrap();
            assert_eq!(map.buildings.len(), 2);
            assert!(map.buildings.iter().all(|building| building.id == 100));
            assert!(
                (map.buildings
                    .iter()
                    .map(|building| model::world::footprint(building).unsigned_area())
                    .sum::<f64>()
                    - 200.)
                    .abs()
                    < 0.001
            );
            if inset {
                assert!(map
                    .buildings
                    .iter()
                    .any(|building| (building.min_height_meters - 6.2).abs() < 0.001));
                assert!(map
                    .buildings
                    .iter()
                    .any(|building| building.holes.len() == 1 && building.height_meters == 18.));
            } else {
                assert!(map
                    .buildings
                    .iter()
                    .all(|building| building.height_meters < 10.));
            }
            let temp = test::support::new();
            let parsed =
                storage::source_files::write_city_txt(&map, &temp.directory.join("city.txt"))
                    .unwrap()
                    .city;
            assert_eq!(
                parsed
                    .buildings
                    .iter()
                    .map(|building| building.holes.len())
                    .sum::<usize>(),
                usize::from(inset)
            );
        }
    }

    #[test]
    fn separated_roads_keep_source_topology_and_join_graded_approaches() {
        use crate::build;
        use crate::model;
        for tags in [vec![("bridge", "yes")], vec![("tunnel", "yes")]] {
            let mut source = source();
            for (id, p) in [
                (1, XZ { x: -100., z: 0. }),
                (2, XZ { x: -20., z: 0. }),
                (3, XZ { x: 20., z: 0. }),
                (4, XZ { x: 100., z: 0. }),
                (5, XZ { x: 0., z: -80. }),
                (6, XZ { x: 0., z: 0. }),
                (7, XZ { x: 0., z: 80. }),
                (8, XZ { x: 0., z: 0. }),
            ] {
                node(&mut source, id, p, &[]);
            }
            way(&mut source, 100, &[1, 2], &[("highway", "residential")]);
            let mut road_tags = tags;
            road_tags.push(("highway", "residential"));
            way(&mut source, 101, &[2, 8, 3], &road_tags);
            way(&mut source, 102, &[3, 4], &[("highway", "residential")]);
            way(&mut source, 103, &[5, 6, 7], &[("highway", "residential")]);
            let map = fixture_map(&source, None).unwrap();
            let folder = tempfile::tempdir().unwrap();
            let surfaces = build::surfaces::write_streets_txt(
                &map,
                &folder.path().join("streets.txt"),
                Some(&source),
            )
            .unwrap()
            .surfaces;
            let path = folder.path().join("road-rises.txt");
            let field = world::elevation::generate(
                BoundingBoxXZ {
                    min: util::geometry::xz_map(map.bounding_box.min, |v| v as f32),
                    max: util::geometry::xz_map(map.bounding_box.max, |v| v as f32),
                },
                |_| 0.,
            );
            let mut data = build::details::build(&source, &map, &surfaces);
            build::transport::build(&source, &map, &surfaces, &field, 0., &mut data, &path)
                .unwrap();
            model::map_details::validate(&data, map.buildings.len(), map.road_lines.len()).unwrap();
            assert_eq!(data.decks.len(), 2);
            let height = data.decks[0].path[0][1];
            assert!((height.abs() - 5.).abs() < 0.001);
            let prepared = world::map_details::new(data);
            assert!(
                (world::map_details::deck_height(&(prepared), XZ { x: 0., z: 0. }, height)
                    .unwrap()
                    - height)
                    .abs()
                    < 0.001
            );
            let rises =
                storage::codec_road_rises::parse(&fs::read_to_string(path).unwrap()).unwrap();
            let lift = |p| {
                rises
                    .iter()
                    .filter_map(|r| world::raised_roads::road_rise_offset(r, p))
                    .max_by(f32::total_cmp)
                    .unwrap_or(0.)
            };
            assert!((lift(XZ { x: -20.01, z: 0. }) - height).abs() < 0.01);
            assert!(lift(XZ { x: -65., z: 0. }).abs() > 1.);
            assert!(
                lift(XZ { x: 0., z: 30. }).abs() < 0.001,
                "crossing road acquired the bridge's elevation"
            );
            assert!(lift(XZ { x: -99., z: 0. }).abs() < 0.001);
            let bridge_nodes: std::collections::BTreeSet<_> = map
                .road_lines
                .iter()
                .filter(|road| road.osm_way_id == Some(101))
                .flat_map(|r| r.ends)
                .collect();
            let ground_nodes: std::collections::BTreeSet<_> = map
                .road_lines
                .iter()
                .filter(|road| road.osm_way_id == Some(103))
                .flat_map(|r| r.ends)
                .collect();
            assert!(bridge_nodes.is_disjoint(&ground_nodes));
        }
    }

    #[test]
    fn building_passages_preserve_upper_floors_and_relation_courtyards() {
        let mut source = source();
        for (id, p) in [
            (1, XZ { x: -40., z: 0. }),
            (2, XZ { x: 40., z: 0. }),
            (10, XZ { x: -10., z: -10. }),
            (11, XZ { x: 10., z: -10. }),
            (12, XZ { x: 10., z: 10. }),
            (13, XZ { x: -10., z: 10. }),
        ] {
            node(&mut source, id, p, &[]);
        }
        way(
            &mut source,
            100,
            &[1, 2],
            &[
                ("highway", "service"),
                ("tunnel", "building_passage"),
                ("maxheight", "3.5"),
            ],
        );
        way(
            &mut source,
            200,
            &[10, 11, 12, 13, 10],
            &[("building", "yes"), ("height", "9")],
        );
        let map = fixture_map(&source, None).unwrap();
        let p = XZ { x: 0., z: 0. };
        let contains = |height: f64| {
            map.buildings.iter().any(|b| {
                model::world::footprint(b).contains(&Point::new(p.x, p.z))
                    && b.min_height_meters <= height
                    && b.height_meters >= height
            })
        };
        assert!(!contains(2.));
        assert!(contains(4.));
        assert!(map.buildings.iter().all(|building| building.id == 200));
        // The same outline imported as a relation retains an actual inner ring.
        for (id, p) in [
            (20, XZ { x: -3., z: -3. }),
            (21, XZ { x: 3., z: -3. }),
            (22, XZ { x: 3., z: 3. }),
            (23, XZ { x: -3., z: 3. }),
        ] {
            node(&mut source, id, p, &[]);
        }
        test::support::set_osm_tags(source.ways.get_mut(&200).unwrap(), &[]);
        way(&mut source, 201, &[20, 21, 22, 23, 20], &[]);
        let mut relation = element(
            300,
            &[
                ("type", "multipolygon"),
                ("building", "yes"),
                ("height", "12"),
                ("name", "Courtyard"),
            ],
        );
        relation.members = vec![
            ("way".into(), 200, "outer".into()),
            ("way".into(), 201, "inner".into()),
        ];
        source.relations.push(relation);
        test::support::set_osm_tags(
            source.ways.get_mut(&100).unwrap(),
            &[("highway", "residential")],
        );
        let map = fixture_map(&source, None).unwrap();
        assert_eq!(map.buildings.len(), 1);
        assert_eq!(map.buildings[0].holes.len(), 1);
        assert!(!model::world::footprint(&(map.buildings[0])).contains(&Point::new(0., 0.)));
        assert_eq!(map.buildings[0].id, build::osm::relation_feature_id(300));
    }

    #[test]
    fn vegetation_uses_rows_species_and_waterway_exclusions_deterministically() {
        use crate::model::map_details::TreeKind;
        let mut source = source();
        for (id, p) in [
            (1, XZ { x: -100., z: 0. }),
            (2, XZ { x: 100., z: 0. }),
            (10, XZ { x: -60., z: -60. }),
            (11, XZ { x: 60., z: -60. }),
            (12, XZ { x: 60., z: 60. }),
            (13, XZ { x: -60., z: 60. }),
            (20, XZ { x: -60., z: 80. }),
            (21, XZ { x: 60., z: 80. }),
            (30, XZ { x: 0., z: -80. }),
            (31, XZ { x: 0., z: 80. }),
        ] {
            node(&mut source, id, p, &[]);
        }
        way(&mut source, 100, &[1, 2], &[("highway", "residential")]);
        way(
            &mut source,
            200,
            &[10, 11, 12, 13, 10],
            &[
                ("landuse", "forest"),
                ("genus", "Pinus"),
                ("leaf_cycle", "evergreen"),
            ],
        );
        way(
            &mut source,
            201,
            &[20, 21],
            &[
                ("natural", "tree_row"),
                ("genus", "Phoenix"),
                ("tree_row:spacing", "6"),
            ],
        );
        way(
            &mut source,
            202,
            &[30, 31],
            &[("waterway", "stream"), ("width", "6")],
        );
        let map = fixture_map(&source, None).unwrap();
        let folder = tempfile::tempdir().unwrap();
        let surfaces = build::surfaces::write_streets_txt(
            &map,
            &folder.path().join("streets.txt"),
            Some(&source),
        )
        .unwrap()
        .surfaces;
        let path = folder.path().join("landscape.txt");
        let styles = build_landscape(&source, &map, &surfaces, &path).unwrap();
        let bytes = fs::read(&path).unwrap();
        let cover = test::support::landscape(std::str::from_utf8(&bytes).unwrap()).unwrap();
        assert!(model::world::trees(&(cover)).len() > 80);
        assert!(styles.iter().any(|style| style.kind == TreeKind::Palm));
        assert!(styles
            .iter()
            .any(|style| style.kind == TreeKind::Conifer && style.evergreen));
        assert!(model::world::trees(&(cover))
            .iter()
            .all(|tree| tree.coords.z.abs() > 4. && tree.coords.x.abs() > 3.));
        assert!(model::world::ground_triangles(&(cover))
            .iter()
            .any(|triangle| triangle.material == model::world::Material::Water));
        build_landscape(&source, &map, &surfaces, &path).unwrap();
        assert_eq!(fs::read(path).unwrap(), bytes);
    }

    #[test]
    fn explicit_lane_turns_and_unpaved_materials_survive_compilation() {
        use crate::build;
        let mut source = source();
        node(&mut source, 1, XZ { x: -80., z: 0. }, &[]);
        node(&mut source, 2, XZ { x: 80., z: 0. }, &[]);
        way(
            &mut source,
            100,
            &[1, 2],
            &[
                ("highway", "primary"),
                ("width", "10"),
                ("lanes", "3"),
                ("lanes:forward", "2"),
                ("lanes:backward", "1"),
                ("turn:lanes:forward", "through|right"),
            ],
        );
        let map = fixture_map(&source, None).unwrap();
        let folder = tempfile::tempdir().unwrap();
        let path = folder.path().join("streets.txt");
        let surfaces = build::surfaces::write_streets_txt(&map, &path, Some(&source))
            .unwrap()
            .surfaces;
        let compiled =
            world::street_surface::load(&fs::read_to_string(&path).unwrap(), map.road_lines.len())
                .unwrap()
                .0;
        let asphalt = test::support::road_footprint(&compiled);
        let paint = build::lane_details::paint(&source, &map, &surfaces.paths, &asphalt);
        assert!(paint.unsigned_area() > 1.);
        assert!(paint
            .0
            .iter()
            .all(|p| p.exterior().0.iter().all(|p| p.x > 65.)));
        test::support::set_osm_tags(
            source.ways.get_mut(&100).unwrap(),
            &[("highway", "residential"), ("surface", "gravel")],
        );
        let map = fixture_map(&source, None).unwrap();
        build::surfaces::write_streets_txt(&map, &path, Some(&source)).unwrap();
        let (surface, _) =
            world::street_surface::load(&fs::read_to_string(path).unwrap(), 1).unwrap();
        assert!(world::street_surface::road_triangles(&(surface))
            .iter()
            .any(
                |triangle| matches!(triangle.usage, model::world::TriangleUsage::Road(_))
                    && triangle.material == model::world::Material::BareGround
            ));
        assert!(world::street_surface::road_triangles(&(surface))
            .iter()
            .all(|triangle| triangle.usage != model::world::TriangleUsage::Marking));
    }

    #[test]
    fn bridge_only_location_can_bake_collision_data_spawn_and_reload() {
        use crate::build;
        use crate::model;
        use crate::storage;
        let folder = tempfile::tempdir().unwrap();
        let assets = test::support::stored_location(folder.path(), false);
        let mut source = source();
        node(&mut source, 1, XZ { x: 0., z: 0. }, &[]);
        node(&mut source, 2, XZ { x: 20., z: 0. }, &[]);
        way(
            &mut source,
            100,
            &[1, 2],
            &[("highway", "residential"), ("bridge", "yes")],
        );
        let map = fixture_map(&source, None).unwrap();
        let city = storage::source_files::write_city_txt(&map, &folder.path().join("city.txt"))
            .unwrap()
            .city;
        let path = folder.path().join("streets.txt");
        let surfaces = build::surfaces::write_streets_txt(&map, &path, Some(&source))
            .unwrap()
            .surfaces;
        let rises = folder.path().join("road-rises.txt");
        let field = storage::city_assets::get_heightfield(&assets).unwrap();
        let mut details = build::details::build(&source, &map, &surfaces);
        build::transport::build(&source, &map, &surfaces, &field, 0., &mut details, &rises)
            .unwrap();
        model::map_details::write(&details, folder.path()).unwrap();
        build::collision::build_collisions(
            &folder.path().join("streets.bin"),
            &city,
            fs::read_to_string(path).unwrap(),
        )
        .unwrap();
        let prepared = world::preparation::world_location_load(&assets).unwrap();
        assert!(world::roads::road_triangles(&(prepared)).is_empty());
        assert!((physics::vehicle_location::spawn(&(prepared)).point.y - 12.).abs() < 0.001);
        let output = tempfile::tempdir().unwrap();
        model::location::location_write(
            &model::location::location_read(folder.path()).unwrap(),
            output.path(),
        )
        .unwrap();
        let reread = world::preparation::world_location_load(&storage::city_assets::new(
            output.path().to_owned(),
        ))
        .unwrap();
        assert!((physics::vehicle_location::spawn(&(reread)).point.y - 12.).abs() < 0.001);
    }

    #[test]
    fn traffic_calming_tags_do_not_generate_or_preserve_raised_roads() {
        use crate::build;
        use crate::model;
        let mut source = source();
        node(&mut source, 1, XZ { x: -40., z: 0. }, &[]);
        node(
            &mut source,
            2,
            XZ { x: 0., z: 0. },
            &[("traffic_calming", "hump"), ("crossing", "raised")],
        );
        node(&mut source, 3, XZ { x: 40., z: 0. }, &[]);
        way(
            &mut source,
            100,
            &[1, 2, 3],
            &[("highway", "residential"), ("traffic_calming", "table")],
        );
        let map = fixture_map(&source, None).unwrap();
        let (temp, surfaces, _) = surfaces(&map);
        let field = world::elevation::generate(
            BoundingBoxXZ {
                min: util::geometry::xz_map(map.bounding_box.min, |v| v as f32),
                max: util::geometry::xz_map(map.bounding_box.max, |v| v as f32),
            },
            |_| 0.,
        );
        let path = temp.directory.join("road-rises.txt");
        // A stale file must not reintroduce traffic calming into a new compilation.
        fs::write(&path, "ROAD_RISES 1\nt 0 0.2 0 1 0.2 0 0 0.2 1\n").unwrap();
        build::transport::build(
            &source,
            &map,
            &surfaces,
            &field,
            0.,
            &mut model::map_details::Data::default(),
            &path,
        )
        .unwrap();
        let rises = storage::codec_road_rises::parse(&fs::read_to_string(path).unwrap()).unwrap();
        assert!(rises.is_empty());
    }
}

mod osm_labels {
    use crate::build::osm_labels::*;
    use crate::model;
    use crate::physics;
    use crate::physics::vehicle_location::deflect;
    use crate::render;
    use crate::storage;
    use crate::test;
    use crate::util;
    use crate::util::geometry::BoundingBoxXZ;
    use crate::util::geometry::XZ;
    use crate::world;
    use crate::world::locations::CAR_RADIUS;
    use crate::world::spatial::*;

    #[test]
    fn business_nodes_share_buildings_without_losing_names_or_label_precedence() {
        let source = r#"<osm>
    <node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="1"/>
    <node id="3" lat="1" lon="1"/><node id="4" lat="1" lon="0"/>
    <node id="5" lat="0.2" lon="0.2"><tag k="shop" v="clothes"/><tag k="name" v="Orchestra"/></node>
    <node id="6" lat="0.8" lon="0.2"><tag k="amenity" v="ice_cream"/><tag k="name" v="Local"/><tag k="name:en" v="Pahit &amp; Ice"/></node>
    <node id="7" lat="0.8" lon="0.8"><tag k="highway" v="bus_stop"/><tag k="name" v="Bus stop"/></node>
    <node id="8" lat="2" lon="2"><tag k="shop" v="electronics"/><tag k="name" v="Outside"/></node>
    <node id="9" lat="0.4" lon="0.2"><tag k="shop" v="clothes"/><tag k="name" v="Orchestra"/></node>
    <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
    <tag k="building" v="yes"/><tag k="name" v="Shopping centre"/></way>
    <way id="11"><nd ref="1"/><nd ref="2"/><tag k="highway" v="primary"/><tag k="name" v="Local road"/><tag k="name:en" v="28th October"/></way>
    </osm>"#;
        let labels = read_osm_labels(source.as_bytes()).unwrap();
        assert_eq!(labels.buildings[&10], "Shopping centre");
        assert_eq!(labels.streets[&11], "28th October");
        assert_eq!(
            labels.storefronts,
            vec![(10, "Orchestra".into()), (10, "Pahit & Ice".into())]
        );
        let stored = model::location::Labels {
            streets: labels.streets.into_iter().collect(),
            buildings: labels.buildings.into_iter().collect(),
            storefronts: labels.storefronts,
        };
        let bytes = serde_json::to_vec(&stored).unwrap();
        assert_eq!(
            serde_json::from_slice::<model::location::Labels>(&bytes).unwrap(),
            stored
        );
        let legacy: model::location::Labels =
            serde_json::from_str(r#"{"streets":[],"buildings":[]}"#).unwrap();
        assert!(legacy.storefronts.is_empty());
    }

    #[test]
    #[ignore = "requires the downloaded Engomi OSM extract; exports labels to /tmp/carslop-engomi-labels.json"]
    fn downloaded_engomi_businesses_have_storefronts() {
        let labels = storage::legacy_labels::read_legacy_labels(std::path::Path::new(
            "src-locations/relation-17716182",
        ))
        .unwrap();
        for (id, expected) in [
            (1019204039, "Stephanis Electronics"),
            (1019204039, "EuroBank"),
            (1019204039, "Cytashop"),
            (1019204034, "Orchestra"),
            (1019204038, "Pahit Ice"),
            (1019204038, "Εν ελλαδι"),
        ] {
            assert!(
                labels.storefronts.contains(&(id, expected.into())),
                "missing {expected} on {id}"
            );
        }
        for expected in ["Art Works", "Dr Xristakis Xristodoulou"] {
            assert!(
                labels.storefronts.iter().any(|(_, name)| name == expected),
                "missing {expected}"
            );
        }
        eprintln!(
            "{} named buildings; {} business storefronts",
            labels.buildings.len(),
            labels.storefronts.len()
        );
        std::fs::write(
            std::env::temp_dir().join("carslop-engomi-labels.json"),
            serde_json::to_vec(&labels).unwrap(),
        )
        .unwrap();
    }

    #[test]
    fn repeated_light_barrier_contacts_preserve_sliding_speed() {
        // Exercise a rotated wall so the response cannot depend on world axes.
        let normal = XZ { x: 0.6, z: 0.8 };
        let tangent = XZ { x: -0.8, z: 0.6 };
        let mut speed = 20.0;
        for _ in 0..120 {
            let mut velocity = util::geometry::xz_from_fn(|i| {
                util::geometry::xz_component(tangent, i) * speed
                    + util::geometry::xz_component(normal, i) * 0.01
            });
            let impact = deflect(&mut velocity, normal);
            assert!((impact - 0.01).abs() < 0.00001);
            assert!(velocity.x * normal.x + velocity.z * normal.z <= 0.0);
            speed = velocity.x * tangent.x + velocity.z * tangent.z;
        }
        assert!(speed > 19.9, "light scrapes drained sliding speed: {speed}");
    }

    #[test]
    fn barrier_response_rebounds_head_on_and_leaves_departing_cars_alone() {
        let mut velocity = XZ { x: 10.0, z: 0.0 };
        assert_eq!(deflect(&mut velocity, XZ { x: 1.0, z: 0.0 }), 10.0);
        assert!((velocity.x + 0.8).abs() < 0.00001);
        assert_eq!(velocity.z, 0.0);
        let departing = velocity;
        assert_eq!(deflect(&mut velocity, XZ { x: 1.0, z: 0.0 }), 0.0);
        assert_eq!(velocity, departing);
    }

    const CROSS:&str="CITY 2\norigin 0 0\nbounds -50 -50 50 50\nn -40 0\nn 0 0\nn 40 0\nn 0 -40\nn 0 40\nr 0 1 8 0 1\nr 1 2 8 0 1\nr 3 1 8 0 2\nr 1 4 8 0 2\nb 1 10 1 4 8 8 18 8 18 18 8 18\n";

    #[test]
    fn branches_are_open_and_dead_ends_do_not_join_other_streets() {
        let city = test::support::parse_world(CROSS).unwrap();
        for i in 0..4 {
            for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let s = world::roads::sample(&city, i, t);
                let mut p = physics::vehicle_location::vehicle_position(s, 0.0);
                let before = p;
                physics::vehicle_location::constrain(&city, &mut p, &mut XZ::default(), 1.7);
                assert_eq!(p, before);
            }
        }
        assert!(
            world::roads::road_clearance(
                &(city),
                util::geometry::XYZ {
                    x: 25.0,
                    y: 0.0,
                    z: 25.0
                }
            ) > 10.0
        );
        let mut p = util::geometry::XYZ {
            x: 0.0,
            y: 0.48,
            z: 45.0,
        };
        physics::vehicle_location::constrain(&city, &mut p, &mut XZ { x: 0.0, z: 10.0 }, 1.7);
        assert!(p.z <= 42.61);
    }
    #[test]
    fn building_contacts_and_camera_respect_actual_footprints() {
        let collision = test::support::parse_world(
            &CROSS.replace("8 8 18 8 18 18 8 18", "8 -4 18 -4 18 4 8 4"),
        )
        .unwrap();
        let mut position = util::geometry::XYZ {
            x: 10.0,
            y: 0.48,
            z: 0.0,
        };
        let mut velocity = XZ { x: 10.0, z: 0.0 };
        assert!(
            physics::vehicle_location::constrain(&(collision), &mut position, &mut velocity, 1.7)
                > 0.0
        );
        assert!(!world::buildings::contains(
            &(world::buildings::buildings(&(collision))[0]),
            XZ {
                x: position.x,
                z: position.z
            }
        ));
        let city = test::support::parse_world(CROSS).unwrap();
        let b = &world::buildings::buildings(&city)[0];
        assert!(world::buildings::contains(b, XZ { x: 10.0, z: 10.0 }));
        assert!(!world::buildings::contains(b, XZ { x: 7.0, z: 10.0 }));
        let eye = render::camera::camera_eye(
            &city,
            util::geometry::XYZ {
                x: 0.0,
                y: 1.0,
                z: 10.0,
            },
            util::geometry::XYZ {
                x: 25.0,
                y: 4.0,
                z: 10.0,
            },
        );
        assert!(eye.x < 7.6);
        assert_eq!(
            render::camera::camera_eye(
                &(city),
                util::geometry::XYZ {
                    x: 0.0,
                    y: 12.0,
                    z: 10.0
                },
                util::geometry::XYZ {
                    x: 25.0,
                    y: 14.0,
                    z: 10.0
                }
            ),
            util::geometry::XYZ {
                x: 25.0,
                y: 14.0,
                z: 10.0
            }
        );
    }
    #[test]
    fn ground_contact_props_and_foundations_share_elevation() {
        let city_track = test::support::city_world();
        let city = &city_track;
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for i in 0..world::roads::road_lines(city).len() {
            for t in [0.0, 0.37, 1.0] {
                let sample = world::roads::sample(city, i, t);
                min = min.min(sample.point.y);
                max = max.max(sample.point.y);
                assert!(
                    (sample.point.y
                        - world::elevation::world_height(
                            city,
                            XZ {
                                x: sample.point.x,
                                z: sample.point.z
                            }
                        ))
                    .abs()
                        < 0.001
                );
                let mut p = physics::vehicle_location::vehicle_position(sample, 0.7);
                physics::vehicle_location::constrain(city, &mut p, &mut XZ::default(), 1.7);
                assert!(
                    (p.y - physics::vehicle_location::drive_height(city, XZ { x: p.x, z: p.z })
                        - 0.48)
                        .abs()
                        < 0.001
                );
            }
        }
        assert!(max - min > 20.0 && max - min < 1000.0);
        for prop in &city.furniture {
            assert!(
                (prop.coords.y
                    - world::elevation::world_height(
                        city,
                        XZ {
                            x: prop.coords.x,
                            z: prop.coords.z
                        }
                    ))
                .abs()
                    < 0.001
            );
        }
        for building in world::buildings::buildings(city) {
            for i in 0..building.ground_outline.len() {
                for p in world::elevation::ground_segment(
                    city,
                    building.ground_outline[i],
                    building.ground_outline[(i + 1) % building.ground_outline.len()],
                ) {
                    assert!(building.base_y >= p.y);
                }
            }
        }
    }
    #[test]
    fn sidewalks_and_parking_are_drivable_at_rendered_height_and_crop_stays_solid() {
        let city_track = test::support::city_world();
        let city = &city_track;
        let mut checked = 0;
        for triangle in world::roads::road_triangles(city) {
            if !matches!(
                triangle.usage,
                model::world::TriangleUsage::Pavement(model::world::CurbStyle::MajorRoad)
                    | model::world::TriangleUsage::Pavement(model::world::CurbStyle::Road)
            ) {
                continue;
            }
            let q = XZ {
                x: triangle.points.iter().map(|p| p.x).sum::<f32>() / 3.0,
                z: triangle.points.iter().map(|p| p.z).sum::<f32>() / 3.0,
            };
            // Sliver centroids can round onto shared edges at city coordinates.
            // Sample pavement interiors, outside the 2 mm edge tolerance.
            if (0..3).any(|i| {
                let a = triangle.points[i];
                let b = triangle.points[(i + 1) % 3];
                let edge = nearest_edge(q, a.into(), b.into());
                (q.x - edge.x).hypot(q.z - edge.z) < 0.02
            }) || world::buildings::building_clearance(city, q) < CAR_RADIUS + 0.1
            {
                continue;
            }
            let mut p = util::geometry::XYZ {
                x: q.x,
                y: 0.0,
                z: q.z,
            };
            let mut velocity = XZ { x: 0.0, z: 0.0 };
            assert_eq!(
                physics::vehicle_location::constrain(city, &mut p, &mut velocity, 1.7),
                0.0
            );
            assert!((p.x - q.x).hypot(p.z - q.z) < 0.001);
            assert!(
                (p.y - world::elevation::world_height(city, q)
                    - (0.48 + world::roads::pavement_height_meters(city) - 0.05))
                    .abs()
                    < 0.001,
                "q={q:?} offset={} style={:?}",
                p.y - world::elevation::world_height(city, q),
                triangle.usage
            );
            assert!(world::street_surface::paved(city, q));
            checked += 1;
            if checked == 20 {
                break;
            }
        }
        assert_eq!(checked, 20);
        let mut parking = 0;
        for triangle in model::world::ground_triangles(city) {
            if triangle.material != model::world::Material::Asphalt {
                continue;
            }
            let q = XZ {
                x: triangle.points.iter().map(|p| p.x).sum::<f32>() / 3.0,
                z: triangle.points.iter().map(|p| p.z).sum::<f32>() / 3.0,
            };
            // Land cover is layered underneath streets; test exposed parking.
            if world::street_surface::paving_height(city, q).is_some()
                || world::buildings::building_clearance(city, q) < CAR_RADIUS + 0.1
            {
                continue;
            }
            let mut p = util::geometry::XYZ {
                x: q.x,
                y: 0.0,
                z: q.z,
            };
            physics::vehicle_location::constrain(city, &mut p, &mut XZ::default(), 1.7);
            assert!((p.x - q.x).hypot(p.z - q.z) < 0.001);
            assert!(world::street_surface::paved(city, q));
            parking += 1;
            if parking == 20 {
                break;
            }
        }
        assert_eq!(parking, 20);
        let mut p = util::geometry::XYZ {
            x: model::world::bounding_box(city).max.x + 10.0,
            y: 0.0,
            z: 0.0,
        };
        let mut velocity = XZ { x: 10.0, z: 0.0 };
        assert!(physics::vehicle_location::constrain(city, &mut p, &mut velocity, 1.7) > 0.0);
        assert!(p.x <= model::world::bounding_box(city).max.x - CAR_RADIUS && velocity.x <= 0.0);
    }

    #[test]
    fn city_map_covers_its_extent_and_queries_only_nearby_streets() {
        let city_track = test::support::city_world();
        let city = &city_track;
        assert_eq!(world::roads::road_lines(city).len(), 21);
        assert_eq!(world::buildings::buildings(city).len(), 21);
        for p in [
            util::geometry::XYZ {
                x: -900.0,
                y: 0.0,
                z: -1000.0,
            },
            util::geometry::XYZ {
                x: 900.0,
                y: 0.0,
                z: -500.0,
            },
            util::geometry::XYZ {
                x: -900.0,
                y: 0.0,
                z: 0.0,
            },
            util::geometry::XYZ {
                x: 900.0,
                y: 0.0,
                z: 500.0,
            },
            util::geometry::XYZ {
                x: -900.0,
                y: 0.0,
                z: 1000.0,
            },
        ] {
            let road = world::roads::project(city, p);
            assert!(
                (road.point.x - p.x).hypot(road.point.z - p.z) < 0.001,
                "missing fixture road near {p:?}"
            );
        }
        let bounds = BoundingBoxXZ {
            min: XZ { x: -40.0, z: -40.0 },
            max: XZ { x: 40.0, z: 40.0 },
        };
        let intersects = |a: util::geometry::XYZ, b: util::geometry::XYZ| {
            a.x.min(b.x) <= bounds.max.x
                && a.x.max(b.x) >= bounds.min.x
                && a.z.min(b.z) <= bounds.max.z
                && a.z.max(b.z) >= bounds.min.z
        };
        let nearby = world::roads::road_segments_in(city, bounds);
        let total_segments: usize = world::roads::road_lines(city)
            .iter()
            .map(|road| road.path.len() - 1)
            .sum();
        assert!(!nearby.is_empty() && nearby.len() < total_segments / 10);
        let actual = nearby.iter().filter(|(a, b)| intersects(*a, *b)).count();
        let all = world::roads::road_lines(city)
            .iter()
            .flat_map(|r| r.path.windows(2))
            .filter(|p| intersects(p[0], p[1]))
            .count();
        assert_eq!(
            actual, all,
            "spatial minimap query must not omit crossing roads"
        );
    }

    #[test]
    fn expanding_grid_projection_matches_brute_force_outside_road_cells() {
        let city = test::support::parse_world(CROSS).unwrap();
        for p in [
            [125.0, 0.0, 175.0],
            [-200.0, 0.0, -300.0],
            [10.0, 0.0, 1000.0],
        ] {
            let indexed = world::roads::project(&city, (p).into());
            let brute = (0..world::roads::road_lines(&city).len())
                .map(|i| world::roads::projected(&city, i, (p).into()))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap()
                .1;
            assert!(
                (indexed.point.x - brute.point.x).hypot(indexed.point.z - brute.point.z) < 0.001
            );
        }
    }
}
