use crate::add_location_compile::create_location_map;
use crate::build_furniture::build_furniture;
use crate::build_landscape::build_landscape;
use crate::error::Result;
use crate::geometry::{cross, polygon, sub};
use crate::geometry::{BoundingBox, BoundingBoxXZ, LatLon, XZ};
use crate::osm::{OSMBoundary, OSMElement, OSMMap};
use crate::source_location::{SourceLocation, SourceRoad};
use crate::sources::extract_osm_boundary;
use crate::{build_surfaces, build_terrain, osm};
use geo::{Area, Contains, MultiPolygon, Point, Polygon};
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

// Small rectangular administrative boundaries keep the synthetic OSM fixtures
// on the same boundary-based import path as real locations.
fn fixture_boundary(source: &OSMMap, bbox: Option<[f64; 4]>) -> OSMBoundary {
    let [south, west, north, east] = bbox.or(source.bounds).expect("fixture bounds");
    OSMBoundary {
        relation: 1,
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
    let source = osm::read(options.source)?;
    let boundary = fixture_boundary(&source, options.bbox);
    let map = create_location_map(&source, &boundary)?;
    let output = options.output;
    fs::create_dir_all(output.join("elevation"))?;
    let city = crate::src_files::write_city_txt(&map, &output.join("city.txt"))?.city;
    let surfaces = build_surfaces::write_streets_txt(&map, &output.join("streets.txt"))?.surfaces;
    build_landscape(&source, &map, &surfaces, &output.join("landscape.txt"))?;
    build_furniture(&source, &map, &surfaces, &output.join("furniture.txt"))?;
    build_terrain::build_terrain(
        &map,
        options.elevation,
        &output.join("elevation/heightfield.txt"),
        &output.join("elevation"),
    )?;
    fs::copy(options.source, output.join("source.osm"))?;
    fs::write(
        output.join("manifest.json"),
        serde_json::to_string_pretty(
            &json!({"name": options.name, "boundary": boundary.to_geojson()}),
        )? + "\n",
    )?;
    crate::build_collision::build_collisions(
        &output.join("streets.bin"),
        &city,
        fs::read_to_string(output.join("streets.txt"))?,
    )?;
    Ok(())
}

use crate::test_support::Temp;
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
    let temp = Temp::new();
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
    let map = fixture_map(&osm::read(&source).unwrap(), None).unwrap();
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
    let temp = Temp::new();
    let tile = temp.directory.join("N35E033.hgt");
    fs::write(
        &tile,
        (0..65)
            .flat_map(|row| (0..65).flat_map(move |col| (100_i16 + row * 2 + col).to_be_bytes()))
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
        crate::test_support::landscape(&fs::read_to_string(output.join("landscape.txt")).unwrap())
            .unwrap();
    let area: f64 = landscape
        .ground_triangles()
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
        landscape
            .ground_triangles()
            .iter()
            .all(|t| t.material == crate::world::Material::Park),
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
        origin: Some(LatLon::new(35.162, 33.320)),
        bounding_box: BoundingBoxXZ {
            min: XZ {
                x: -100.0,
                z: -100.0,
            },
            max: XZ { x: 100.0, z: 100.0 },
        },
        road_nodes: nodes.iter().map(|p| p.at_height(0.0)).collect(),
        road_lines: roads
            .iter()
            .enumerate()
            .map(|(i, (a, b, width, kind))| SourceRoad {
                kind: *kind,
                direction: 0,
                osm_way_id: Some(i as i64 + 1),
                lanes: 2,
                ..SourceRoad::new([*a, *b], *width)
            })
            .collect(),
        buildings: buildings
            .iter()
            .enumerate()
            .map(|(i, p)| crate::world::Building {
                id: i as u64 + 1,
                height_meters: 6.0,
                base_y: 0.0,
                tagged: false,
                ground_outline: p.clone(),
            })
            .collect(),
        ..SourceLocation::<f64>::empty()
    }
}
fn surfaces(
    map: &SourceLocation<f64>,
) -> (
    Temp,
    build_surfaces::Surfaces,
    crate::street_surface::StreetSurface,
) {
    let temp = Temp::new();
    crate::src_files::write_city_txt(map, &temp.directory.join("city.txt")).unwrap();
    let s = build_surfaces::write_streets_txt(map, &temp.directory.join("streets.txt")).unwrap();
    let loaded = crate::street_surface::StreetSurface::load(
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
    crate::test_support::osm_element(id, tags)
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
    e.coord = Some(LatLon::new(
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
    use crate::traffic::FurnitureKind::*;
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
    let l = crate::test_support::landscape(
        &std::fs::read_to_string(temp.directory.join("landscape.txt")).unwrap(),
    )
    .unwrap();
    assert_eq!(l.trees().len(), 1);
    assert_eq!(l.trees()[0].height_meters, 8.);
    let mut sports = 0.;
    for t in l.ground_triangles() {
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
        if t.material == crate::world::Material::Sports {
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
    let temp = crate::test_support::Temp::new();
    for (x, offset) in [(6, 0), (7, 8)] {
        let values: Vec<u8> = (0..9)
            .flat_map(|_| (0..9).flat_map(|c| (100_i16 + offset + c).to_be_bytes()))
            .collect();
        fs::write(temp.directory.join(format!("N50E{x:03}.hgt")), values).unwrap();
    }
    let map = SourceLocation {
        boundary: None,
        origin: Some(LatLon::new(50.5, 7.0)),
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
        ..SourceLocation::<f64>::empty()
    };
    let source = temp.directory.join("N50E006.hgt");
    let (dem, sources) = crate::hgt::covering(&source, &map).unwrap();
    assert_eq!(sources.len(), 2);
    for longitude in [6.95, 7.0, 7.05] {
        let expected = 100.0 + (longitude - 6.0) * 8.0;
        assert!((dem.sample(LatLon::new(50.5, longitude)).unwrap() - expected).abs() < 1e-9);
    }
    fs::remove_file(temp.directory.join("N50E007.hgt")).unwrap();
    assert!(crate::hgt::covering(&source, &map).is_err());
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
    let source = crate::osm::read(&output).unwrap();
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
    assert_eq!(source.nodes[&1].tag("name"), "A & B");
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

fn controls(oneway: bool, enabled: bool) -> Vec<crate::traffic::Furniture> {
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
    crate::asset_codecs::furniture::parse(
        &std::fs::read_to_string(temp.directory.join("furniture.txt")).unwrap(),
    )
    .unwrap()
}
