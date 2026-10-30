//! Acceptance cases transferred from the retired Python map compiler.
use super::*;
use geo::{Area, BooleanOps, Buffer, Contains, Intersects, Point};

fn map(nodes: &[P], roads: &[(usize, usize, f64, usize)], buildings: &[Vec<P>]) -> Map {
    Map {
        origin: [35.162, 33.320],
        bounds: [-100.0, -100.0, 100.0, 100.0],
        nodes: nodes.to_vec(),
        roads: roads
            .iter()
            .enumerate()
            .map(|(i, (a, b, width, kind))| Road {
                ends: [*a, *b],
                width: *width,
                kind: *kind,
                direction: 0,
                way: i as i64 + 1,
                lanes: 2,
            })
            .collect(),
        buildings: buildings
            .iter()
            .enumerate()
            .map(|(i, p)| Building {
                id: i as i64 + 1,
                height: 6.0,
                tagged: false,
                points: p.clone(),
            })
            .collect(),
    }
}
fn surfaces(
    map: &Map,
) -> (
    tests::Temp,
    surfaces::Surfaces,
    crate::street_surface::StreetSurface,
) {
    let temp = tests::Temp::new();
    let city = osm::write(map, &temp.0).unwrap();
    let s = surfaces::build(map, &city, &temp.0).unwrap();
    let loaded = crate::street_surface::StreetSurface::load(
        &crate::package::read_to_string(temp.0.join("streets.txt")).unwrap(),
        &city,
        map.roads.len(),
    )
    .unwrap()
    .0;
    (temp, s, loaded)
}
fn point(p: P) -> Point<f64> {
    Point::new(p[0], p[1])
}
#[test]
fn junction_fillets_partition_asphalt_and_pavement_without_overlap() {
    let m = map(
        &[[-40., 0.], [0., 0.], [40., 0.], [0., -40.], [0., 40.]],
        &[(0, 1, 8., 0), (1, 2, 8., 0), (3, 1, 6.5, 1), (1, 4, 5.5, 2)],
        &[],
    );
    let (_, s, loaded) = surfaces(&m);
    assert!(s.asphalt.contains(&point([4.4, 3.4])));
    let mut area = 0.;
    let mut styles = std::collections::BTreeSet::new();
    for t in &loaded.triangles {
        styles.insert(t.style);
        let p: Vec<P> = t
            .points
            .iter()
            .map(|p| [p[0] as f64, p[1] as f64])
            .collect();
        let poly = polygon(&p);
        if t.style <= 3 {
            area += poly.unsigned_area();
        }
        if t.style >= 5 {
            assert!(poly.intersection(&s.asphalt).unsigned_area() < 0.0001);
        }
    }
    assert!((area - s.asphalt.unsigned_area()).abs() < 0.01);
    assert_eq!(styles, (0..8).collect());
}
#[test]
fn t_junction_preserves_far_curb_and_fillets_avoid_buildings() {
    let m = map(
        &[[-40., 0.], [0., 0.], [40., 0.], [0., 40.]],
        &[(0, 1, 8., 0), (1, 2, 8., 0), (1, 3, 10., 1)],
        &[],
    );
    let (_, s, _) = surfaces(&m);
    assert!(!s.asphalt.intersects(&point([0., -4.5])));
    assert!(s.asphalt.contains(&point([0., -3.9])));
    let building = vec![[4.2, 4.2], [7., 4.2], [7., 7.], [4.2, 7.]];
    let m = map(
        &[[-40., 0.], [0., 0.], [40., 0.], [0., -40.], [0., 40.]],
        &[(0, 1, 8., 1), (1, 2, 8., 1), (3, 1, 8., 1), (1, 4, 8., 1)],
        &[building],
    );
    let (_, s, _) = surfaces(&m);
    assert!(!s.asphalt.intersects(&point([4.5, 4.5])));
}
#[test]
fn parallel_roads_and_enclosed_blocks_keep_their_gaps() {
    let m = map(
        &[[-30., 0.], [30., 0.], [-30., 10.], [30., 10.]],
        &[(0, 1, 6., 1), (2, 3, 6., 1)],
        &[],
    );
    let (_, s, _) = surfaces(&m);
    assert_eq!(s.asphalt.0.len(), 2);
    assert!(!s.asphalt.intersects(&point([0., 5.])));
    let m = map(
        &[[-20., -20.], [20., -20.], [20., 20.], [-20., 20.]],
        &[(0, 1, 6., 1), (1, 2, 6., 1), (2, 3, 6., 1), (3, 0, 6., 1)],
        &[],
    );
    let (_, s, loaded) = surfaces(&m);
    assert!(!s.asphalt.intersects(&point([0., 0.])));
    for t in loaded.triangles.iter().filter(|t| t.style <= 3) {
        let p: Vec<P> = t
            .points
            .iter()
            .map(|p| [p[0] as f64, p[1] as f64])
            .collect();
        assert!(!polygon(&p).intersects(&point([0., 0.])));
    }
}

#[test]
fn indexed_obstacle_checks_match_merged_coverage_including_block_holes() {
    let building = vec![[-3., -3.], [3., -3.], [3., 3.], [-3., 3.]];
    let m = map(
        &[[-20., -20.], [20., -20.], [20., 20.], [-20., 20.]],
        &[(0, 1, 6., 1), (1, 2, 6., 1), (2, 3, 6., 1), (3, 0, 6., 1)],
        std::slice::from_ref(&building),
    );
    let (_, s, _) = surfaces(&m);
    let index = surfaces::PolygonIndex::new(&s.obstacles);
    let footprint = polygon(&building);
    for x in (-40..=40).step_by(4) {
        for z in (-40..=40).step_by(4) {
            let probe = point([x as f64, z as f64]).buffer(0.18);
            assert_eq!(
                index.intersects(&probe),
                s.asphalt.intersects(&probe) || footprint.intersects(&probe),
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
                    [radius * t.cos(), radius * t.sin()]
                })
                .collect::<Vec<_>>(),
        )
    };
    let geometry = MultiPolygon(vec![Polygon::new(ring(5000, 100.), vec![ring(2000, 50.)])]);
    let mut bytes = vec![];
    surfaces::triangles(&mut bytes, "t 0", &geometry).unwrap();
    let mut area = 0.;
    for row in std::str::from_utf8(&bytes).unwrap().lines() {
        let values: Vec<f64> = row
            .split_whitespace()
            .skip(2)
            .map(|s| s.parse().unwrap())
            .collect();
        let p = polygon(&[
            [values[0], values[1]],
            [values[2], values[3]],
            [values[4], values[5]],
        ]);
        assert!(!p.contains(&point([0., 0.])));
        area += p.unsigned_area();
    }
    assert!(
        (area - geometry.unsigned_area()).abs() < 0.01,
        "emitted {area}, expected {}",
        geometry.unsigned_area()
    );
}

#[test]
fn disk_precision_does_not_emit_collapsed_triangles() {
    let tiny = polygon(&[[0., 0.00000051], [100., 0.00000251], [50., 0.00000162]]);
    let normal = polygon(&[[0., 10.], [100., 10.], [50., 11.]]);
    let mut bytes = vec![];
    let count = surfaces::triangles(&mut bytes, "t 0", &MultiPolygon(vec![tiny, normal])).unwrap();
    assert_eq!(count, 1);
    let text = String::from_utf8(bytes).unwrap();
    let f: Vec<f64> = text
        .split_whitespace()
        .skip(2)
        .map(|s| s.parse().unwrap())
        .collect();
    assert!(cross([f[2] - f[0], f[3] - f[1]], [f[4] - f[0], f[5] - f[1]]) > 0.00001);
}
#[test]
fn bend_smoothing_shares_endpoints_and_stays_within_sixty_centimetres() {
    let m = map(
        &[[-30., 0.], [0., 0.], [20., 20.]],
        &[(0, 1, 6.5, 1), (1, 2, 6.5, 1)],
        &[],
    );
    let (_, s, _) = surfaces(&m);
    assert!(dist(*s.paths[0].last().unwrap(), s.paths[1][0]) < 1e-6);
    assert!(dist(s.paths[1][0], m.nodes[1]) > 0.01);
    for (r, path) in m.roads.iter().zip(&s.paths) {
        for p in path {
            assert!(project(&[m.nodes[r.ends[0]], m.nodes[r.ends[1]]], *p).0 <= 0.601);
        }
    }
    let m = map(
        &[[-30., 0.], [0., 0.], [30., 0.]],
        &[(0, 1, 6.5, 1), (1, 2, 6.5, 1)],
        &[],
    );
    let (_, s, _) = surfaces(&m);
    assert_eq!(s.paths[0], vec![[-30., 0.], [0., 0.]]);
}
fn element(id: i64, tags: &[(&str, &str)]) -> Element {
    Element {
        id,
        tags: tags
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
        ..Default::default()
    }
}
fn source() -> Source {
    Source {
        nodes: BTreeMap::new(),
        ways: BTreeMap::new(),
        relations: vec![],
        bounds: Some([35.160, 33.318, 35.164, 33.322]),
        timestamp: "fixture".into(),
    }
}
fn node(s: &mut Source, id: i64, p: P, tags: &[(&str, &str)]) {
    let mut e = element(id, tags);
    e.coord = Some([
        35.162 - p[1].to_degrees() / 6371000.,
        33.320 + p[0].to_degrees() / 6371000. / 35.162_f64.to_radians().cos(),
    ]);
    s.nodes.insert(id, e);
}
fn way(s: &mut Source, id: i64, nodes: &[i64], tags: &[(&str, &str)]) {
    let mut e = element(id, tags);
    e.nodes = nodes.to_vec();
    s.ways.insert(id, e);
}
#[test]
fn importer_retains_separate_networks_and_filters_access_and_crossing_structures() {
    let mut s = source();
    for (id, p) in [
        (1, [-50., 0.]),
        (2, [0., 0.]),
        (3, [50., 0.]),
        (4, [0., 50.]),
        (5, [70., 70.]),
        (6, [90., 70.]),
    ] {
        node(&mut s, id, p, &[]);
    }
    way(&mut s, 1, &[1, 2, 3], &[("highway", "residential")]);
    way(
        &mut s,
        2,
        &[2, 4],
        &[("highway", "tertiary"), ("width", "8"), ("lanes", "3")],
    );
    way(&mut s, 3, &[5, 6], &[("highway", "residential")]);
    for (id, tag, value) in [
        (4, "access", "private"),
        (5, "vehicle", "no"),
        (6, "motor_vehicle", "private"),
        (7, "bridge", "yes"),
        (8, "tunnel", "yes"),
        (9, "layer", "1"),
        (10, "area", "yes"),
    ] {
        way(&mut s, id, &[3, 5], &[("highway", "service"), (tag, value)]);
    }
    let m = osm::compile(&s, None).unwrap();
    assert_eq!(m.roads.len(), 4);
    assert_eq!(m.nodes.len(), 6);
    assert!(m.roads.iter().all(|r| r.way <= 3));
    let avenue = m.roads.iter().find(|r| r.way == 2).unwrap();
    assert_eq!((avenue.width, avenue.kind, avenue.lanes), (8., 0, 3));
    assert!(m.metres([35.162, 33.320]).iter().all(|n| n.abs() < 1e-8));
    assert!((m.metres([35.163, 33.320])[1] + 111.195).abs() < 0.01);
    // New locations intentionally use their crop centre, rather than the legacy global Engomi origin.
    let larger = osm::compile(&s, Some([35.156, 33.312, 35.176, 33.336])).unwrap();
    assert_eq!(larger.origin, [35.166, 33.324]);
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
        node(&mut s, id, [0., z], &tags);
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
    let m = osm::compile(&s, None).unwrap();
    let (temp, surfaces, _) = surfaces(&m);
    let city = crate::package::read_to_string(temp.0.join("city.txt")).unwrap();
    scenery::build(&s, &m, &surfaces, &city, &temp.0).unwrap();
    crate::traffic::load(
        &crate::package::read_to_string(temp.0.join("furniture.txt")).unwrap(),
        &city,
        &crate::package::read_to_string(temp.0.join("streets.txt")).unwrap(),
    )
    .unwrap()
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
        assert!(f.position[0] * f.travel[1] > 0.);
        assert!(f.position[0].abs() > 3.25);
    }
    let f = controls(true, true);
    assert_eq!(f.iter().filter(|f| f.kind == Signal).count(), 1);
    assert!(f.iter().all(|f| f.travel[1] > 0.));
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
            node(&mut s, next, p, &[]);
            refs.push(next);
            next += 1;
        }
        refs.push(refs[0]);
        way(&mut s, id, &refs, &tags);
    }
    node(
        &mut s,
        1,
        [-70., -70.],
        &[("natural", "tree"), ("height", "8")],
    );
    node(&mut s, 2, [0., 0.], &[("natural", "tree")]);
    node(&mut s, 3, [60., 60.], &[("natural", "tree")]);
    node(&mut s, 5001, [-50., 0.], &[]);
    node(&mut s, 5002, [50., 0.], &[]);
    way(&mut s, 1, &[5001, 5002], &[("highway", "residential")]);
    let m = map(
        &[[-50., 0.], [50., 0.]],
        &[(0, 1, 8., 1)],
        &[vec![[55., 55.], [65., 55.], [65., 65.], [55., 65.]]],
    );
    let (temp, surfaces, _) = surfaces(&m);
    let city = crate::package::read_to_string(temp.0.join("city.txt")).unwrap();
    scenery::build(&s, &m, &surfaces, &city, &temp.0).unwrap();
    let streets = crate::package::read_to_string(temp.0.join("streets.txt")).unwrap();
    let l = crate::landscape::Landscape::load(
        &crate::package::read_to_string(temp.0.join("landscape.txt")).unwrap(),
        &city,
        &streets,
    )
    .unwrap();
    assert_eq!(l.trees.len(), 1);
    assert_eq!(l.trees[0].height, 8.);
    let mut sports = 0.;
    for t in &l.ground {
        let points: Vec<P> = t
            .points
            .iter()
            .map(|p| [p[0] as f64, p[1] as f64])
            .collect();
        assert!(points.iter().all(|p| p[0] >= -100.001 && p[0] <= 100.001));
        let poly = polygon(&points);
        if t.style == 7 {
            sports += poly.unsigned_area();
        } else {
            assert!(!poly.contains(&point([40., 40.])));
        }
    }
    assert!((sports - 400.).abs() < 0.01);
}

#[test]
fn multipolygon_outers_with_opposite_winding_are_unioned_without_cancellation() {
    let mut s = source();
    node(&mut s, 1, [-50., 0.], &[]);
    node(&mut s, 2, [50., 0.], &[]);
    way(&mut s, 1, &[1, 2], &[("highway", "residential")]);
    let mut next = 10;
    for (id, points) in [
        (100, vec![[10., 30.], [20., 30.], [20., 40.], [10., 40.]]),
        (101, vec![[15., 30.], [15., 40.], [25., 40.], [25., 30.]]),
    ] {
        let mut refs = vec![];
        for p in points {
            node(&mut s, next, p, &[]);
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
    let m = map(&[[-50., 0.], [50., 0.]], &[(0, 1, 8., 1)], &[]);
    let (temp, surfaces, _) = surfaces(&m);
    let city = crate::package::read_to_string(temp.0.join("city.txt")).unwrap();
    scenery::build(&s, &m, &surfaces, &city, &temp.0).unwrap();
    let meta: serde_json::Value = serde_json::from_str(
        &crate::package::read_to_string(temp.0.join("landscape.json")).unwrap(),
    )
    .unwrap();
    assert!((meta["area_m2_by_style"]["1"].as_f64().unwrap() - 150.).abs() < 0.01);
}

/// Run after importing the full crop; the historical bundle remains the reference.
#[test]
#[ignore = "requires NFS_VALIDATED_CITY pointing to a complete full Nicosia import"]
fn full_nicosia_import_preserves_network_buildings_and_surface_area() {
    use std::io::{BufRead, BufReader};
    let imported = std::path::PathBuf::from(
        std::env::var_os("NFS_VALIDATED_CITY").expect("set NFS_VALIDATED_CITY"),
    );
    let bundled = Path::new(env!("CARGO_MANIFEST_DIR")).join("data/locations/nicosia.sqlite");
    let original = crate::package::read_to_string(bundled.join("city.txt")).unwrap();
    let rebuilt = crate::package::read_to_string(imported.join("city.txt")).unwrap();
    let origin = |text: &str| -> P {
        let f: Vec<_> = text
            .lines()
            .find(|l| l.starts_with("origin "))
            .unwrap()
            .split_whitespace()
            .collect();
        [f[1].parse().unwrap(), f[2].parse().unwrap()]
    };
    let old_origin = origin(&original);
    let new_origin = origin(&rebuilt);
    let latlon = |p: &[f32; 3], o: P| -> [f64; 2] {
        [
            o[0] - (p[2] as f64).to_degrees() / 6371000.,
            o[1] + (p[0] as f64).to_degrees() / 6371000. / o[0].to_radians().cos(),
        ]
    };
    let old = crate::city::City::parse(&original).unwrap();
    let new = crate::city::City::parse(&rebuilt).unwrap();
    assert_eq!(new.nodes.len(), old.nodes.len());
    assert_eq!(new.streets.len(), old.streets.len());
    assert_eq!(new.buildings.len(), old.buildings.len());
    for (a, b) in old.nodes.iter().zip(&new.nodes) {
        let a = latlon(a, old_origin);
        let b = latlon(b, new_origin);
        assert!(
            dist(a, b).to_radians() * 6371000. < 0.004,
            "node moved in geographic space: {a:?} vs {b:?}"
        );
    }
    let roads = |text: &str| {
        let mut result = BTreeMap::new();
        for line in text.lines().filter(|l| l.starts_with("r ")) {
            let f: Vec<_> = line.split_whitespace().collect();
            let key = (
                f[5].parse::<i64>().unwrap(),
                f[1].parse::<usize>().unwrap(),
                f[2].parse::<usize>().unwrap(),
            );
            result.insert(
                key,
                f[3..5]
                    .iter()
                    .chain(&f[6..])
                    .map(|s| s.to_string())
                    .collect::<Vec<_>>(),
            );
        }
        result
    };
    assert_eq!(roads(&original), roads(&rebuilt));
    for (a, b) in old.buildings.iter().zip(&new.buildings) {
        assert_eq!(a.id, b.id);
        assert_eq!(a.height, b.height);
        assert_eq!(a.polygon.len(), b.polygon.len());
    }
    let metadata: serde_json::Value = serde_json::from_str(
        &crate::package::read_to_string(imported.join("streets.json")).unwrap(),
    )
    .unwrap();
    let reference: serde_json::Value = serde_json::from_str(
        &crate::package::read_to_string(bundled.join("streets.json")).unwrap(),
    )
    .unwrap();
    let area = metadata["road_area_m2"].as_f64().unwrap();
    let reference_area = reference["road_area_m2"].as_f64().unwrap();
    assert!(
        (area - reference_area).abs() / reference_area < 0.001,
        "road area changed: {area} vs {reference_area}"
    );
    let mut emitted_area = 0.;
    let mut triangles = 0;
    let mut curbs = 0;
    for line in BufReader::new(fs::File::open(imported.join("streets.txt")).unwrap()).lines() {
        let line = line.unwrap();
        let f: Vec<_> = line.split_whitespace().collect();
        if f.first() == Some(&"t") {
            triangles += 1;
            if f[1].parse::<usize>().unwrap() <= 3 {
                let p: Vec<f64> = f[2..].iter().map(|s| s.parse().unwrap()).collect();
                let signed = cross([p[2] - p[0], p[3] - p[1]], [p[4] - p[0], p[5] - p[1]]) / 2.;
                assert!(signed > 0.);
                emitted_area += signed;
            }
        } else if f.first() == Some(&"c") {
            curbs += 1;
        }
    }
    assert!(
        (area - emitted_area).abs() / area < 0.00001,
        "triangulation lost surface area"
    );
    assert_eq!(curbs, metadata["curb_segments"].as_u64().unwrap());
    let expected_triangles: u64 = metadata["surface_triangles"]
        .as_object()
        .unwrap()
        .values()
        .map(|v| v.as_u64().unwrap())
        .sum();
    assert_eq!(triangles, expected_triangles);
    let key = crate::street_surface::crc32(&rebuilt);
    let streets_crc = super::surfaces::crc_file(&imported.join("streets.txt")).unwrap();
    let elevation =
        crate::package::read_to_string(imported.join("elevation/heightfield.txt")).unwrap();
    crate::elevation::HeightField::load(&elevation, &rebuilt).unwrap();
    for (file, kind) in [
        ("furniture.txt", "FURNITURE"),
        ("landscape.txt", "LANDSCAPE"),
    ] {
        let first = BufReader::new(fs::File::open(imported.join(file)).unwrap())
            .lines()
            .next()
            .unwrap()
            .unwrap();
        assert_eq!(first, format!("{kind} 1 {key:08x} {streets_crc:08x}"));
    }
    let controls = |folder: &Path| {
        let mut counts = BTreeMap::new();
        for line in crate::package::read_to_string(folder.join("furniture.txt"))
            .unwrap()
            .lines()
            .skip(1)
        {
            let f: Vec<_> = line.split_whitespace().collect();
            *counts
                .entry((f[1].to_string(), f[2].to_string()))
                .or_insert(0_u64) += 1;
        }
        counts
    };
    let old_controls = controls(&bundled);
    let new_controls = controls(&imported);
    assert_eq!(
        old_controls, new_controls,
        "mapped control approaches changed"
    );
    let trees = |folder: &Path| {
        crate::package::read_to_string(folder.join("landscape.txt"))
            .unwrap()
            .lines()
            .filter(|l| l.starts_with("t "))
            .map(|l| {
                let f: Vec<_> = l.split_whitespace().collect();
                (f[1].to_string(), (f[4].to_string(), f[5].to_string()))
            })
            .collect::<BTreeMap<_, _>>()
    };
    let old_trees = trees(&bundled);
    let new_trees = trees(&imported);
    assert_eq!(
        old_trees, new_trees,
        "mapped tree identities or dimensions changed"
    );
    let old_land: serde_json::Value = serde_json::from_str(
        &crate::package::read_to_string(bundled.join("landscape.json")).unwrap(),
    )
    .unwrap();
    let new_land: serde_json::Value = serde_json::from_str(
        &crate::package::read_to_string(imported.join("landscape.json")).unwrap(),
    )
    .unwrap();
    let mut landscape_deltas = BTreeMap::new();
    for (i, name) in [
        "grass",
        "park",
        "scrub",
        "asphalt",
        "paving",
        "concrete",
        "bare_ground",
        "sports",
        "water",
    ]
    .iter()
    .enumerate()
    {
        let a = old_land["statistics"][format!("{name}_m2")]
            .as_f64()
            .unwrap();
        let b = new_land["area_m2_by_style"][i.to_string()]
            .as_f64()
            .unwrap();
        let relative = (a - b).abs() / a;
        // These are independently clipped/unioned meshes, not byte-identical
        // rebuilds. Keep class changes below 0.25%, plus a tighter total bound.
        assert!(relative < 0.0025, "{name} area changed: {a} vs {b}");
        landscape_deltas.insert(*name, relative);
    }
    let original_total: f64 = old_land["statistics"]
        .as_object()
        .unwrap()
        .iter()
        .filter(|(k, _)| k.ends_with("_m2"))
        .map(|(_, v)| v.as_f64().unwrap())
        .sum();
    let rebuilt_total: f64 = new_land["area_m2_by_style"]
        .as_object()
        .unwrap()
        .values()
        .map(|v| v.as_f64().unwrap())
        .sum();
    let total_delta = (original_total - rebuilt_total).abs() / original_total;
    assert!(
        total_delta < 0.0001,
        "total land cover area changed: {original_total} vs {rebuilt_total}"
    );
    let report = serde_json::json!({"nodes":new.nodes.len(),"road_segments":new.streets.len(),"buildings":new.buildings.len(),"road_area_m2":area,"reference_road_area_m2":reference_area,"triangles":triangles,"curbs":curbs,"mapped_controls":new_controls.values().sum::<u64>(),"trees":new_trees.len(),"landscape_relative_area_differences":landscape_deltas,"landscape_total_relative_area_difference":total_delta,"scene_pack_bytes":crate::package::size(&imported.join("scene.pack")).unwrap(),"reference_scene_pack_bytes":crate::package::size(&bundled.join("scene.pack")).unwrap(),"checks":"geographic node positions; complete road metadata; building IDs/heights/ring sizes; asphalt area and winding; elevation loader; scenery CRC headers; mapped control/tree identities; land cover area per style within 0.25%, total within 0.01%"});
    json(
        &imported.parent().unwrap().join("validation-report.json"),
        report.clone(),
    )
    .unwrap();
    println!("{report}");
}
