use super::*;
pub struct Temp(pub std::path::PathBuf);
impl Temp {
    pub fn new() -> Self {
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let p = std::env::temp_dir().join(format!(
            "cars-import-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        ));
        fs::create_dir(&p).unwrap();
        Self(p)
    }
}
impl Drop for Temp {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
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
fn native_import_loads_in_game_and_preserves_topology() {
    let temp = Temp::new();
    let tile = temp.0.join("N35E033.hgt");
    fs::write(
        &tile,
        (0..65)
            .flat_map(|row| (0..65).flat_map(move |col| (100_i16 + row * 2 + col).to_be_bytes()))
            .collect::<Vec<_>>(),
    )
    .unwrap();
    let source = temp.0.join("fixture.osm");
    fs::write(&source, FIXTURE).unwrap();
    let output = temp.0.join("compiled");
    compile(Options {
        source: &source,
        output: &output,
        name: "fixture",
        bbox: None,
        elevation: &tile,
    })
    .unwrap();
    let city = fs::read_to_string(output.join("city.txt")).unwrap();
    let parsed = crate::city::City::parse(&city).unwrap();
    assert_eq!(parsed.streets.len(), 3);
    assert_eq!(parsed.buildings.len(), 1);
    assert_eq!(parsed.buildings[0].height, 12.0);
    let streets = fs::read_to_string(output.join("streets.txt")).unwrap();
    let (surface, paths) = crate::street_surface::StreetSurface::load(&streets, &city, 3).unwrap();
    assert!(!surface.triangles.is_empty());
    assert_eq!(paths.len(), 3);
    let furniture = crate::traffic::load(
        &fs::read_to_string(output.join("furniture.txt")).unwrap(),
        &city,
        &streets,
    )
    .unwrap();
    assert_eq!(furniture.len(), 3);
    let landscape = crate::landscape::Landscape::load(
        &fs::read_to_string(output.join("landscape.txt")).unwrap(),
        &city,
        &streets,
    )
    .unwrap();
    assert_eq!(landscape.trees.len(), 1);
    let elevation = crate::elevation::HeightField::load(
        &fs::read_to_string(output.join("elevation/heightfield.txt")).unwrap(),
        &city,
    )
    .unwrap();
    assert_eq!(elevation.height([0.0, 0.0]), 0.0);
    assert!(elevation.height([100.0, 0.0]).abs() > 0.01);
    let second = temp.0.join("second");
    compile(Options {
        source: &source,
        output: &second,
        name: "fixture",
        bbox: None,
        elevation: &tile,
    })
    .unwrap();
    for asset in [
        "city.txt",
        "streets.txt",
        "landscape.txt",
        "furniture.txt",
        "elevation/heightfield.txt",
    ] {
        assert_eq!(
            fs::read(output.join(asset)).unwrap(),
            fs::read(second.join(asset)).unwrap(),
            "nondeterministic {asset}"
        );
    }
}
#[test]
fn raceways_allow_restricted_circuit_access_without_opening_private_streets() {
    let temp = Temp::new();
    let source = temp.0.join("circuit.osm");
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
    let map = osm::compile(&osm::read(&source).unwrap(), None).unwrap();
    for id in [201, 202] {
        let road = map.roads.iter().find(|r| r.way == id).unwrap();
        assert_eq!(road.width, 10.0);
    }
    assert!(!map.roads.iter().any(|r| [103, 203, 204].contains(&r.way)));
    assert_eq!(map.roads[0].way, 201);
    let bridge = map.roads.iter().find(|r| r.way == 205).unwrap();
    assert_eq!(bridge.direction, 1);
}
#[test]
fn clipping_does_not_join_unrelated_roads() {
    assert_eq!(
        osm::clip([-20.0, 0.0], [20.0, 0.0], [-10.0, -10.0, 10.0, 10.0]),
        Some(([-10.0, 0.0], [10.0, 0.0]))
    );
    assert!(osm::clip([-20.0, 20.0], [20.0, 20.0], [-10.0, -10.0, 10.0, 10.0]).is_none());
}
#[test]
fn incomplete_exports_are_rejected() {
    let temp = Temp::new();
    let p = temp.0.join("bad.osm");
    fs::write(&p, "<osm><remark>timeout</remark></osm>").unwrap();
    assert!(osm::read(&p).is_err());
    fs::write(&p, "<osm><bounds minlat=\"35\"/></osm>").unwrap();
    assert!(osm::read(&p).is_err());
    fs::write(&p, "<osm><node id=\"1\" lat=\"35\" lon=\"33\"/>").unwrap();
    assert!(osm::read(&p).is_err());
}

#[test]
fn mapped_multipolygon_holes_stay_empty() {
    let temp = Temp::new();
    let tile = temp.0.join("N35E033.hgt");
    fs::write(
        &tile,
        (0..65)
            .flat_map(|row| (0..65).flat_map(move |col| (100_i16 + row * 2 + col).to_be_bytes()))
            .collect::<Vec<_>>(),
    )
    .unwrap();
    let source = temp.0.join("holes.osm");
    let extra = r#"
<node id="10" lat="35.16315" lon="33.32115"/>
<node id="11" lat="35.16315" lon="33.32135"/>
<node id="12" lat="35.16335" lon="33.32135"/>
<node id="13" lat="35.16335" lon="33.32115"/>
<way id="106"><nd ref="10"/><nd ref="11"/><nd ref="12"/><nd ref="13"/><nd ref="10"/><tag k="landuse" v="grass"/></way>
<relation id="201"><member type="way" ref="105" role="outer"/><member type="way" ref="106" role="inner"/><tag k="type" v="multipolygon"/><tag k="leisure" v="park"/></relation>
</osm>"#;
    fs::write(&source, FIXTURE.replace("</osm>", extra)).unwrap();
    let output = temp.0.join("out");
    compile(Options {
        source: &source,
        output: &output,
        name: "holes",
        bbox: None,
        elevation: &tile,
    })
    .unwrap();
    let city = fs::read_to_string(output.join("city.txt")).unwrap();
    let streets = fs::read_to_string(output.join("streets.txt")).unwrap();
    let landscape = crate::landscape::Landscape::load(
        &fs::read_to_string(output.join("landscape.txt")).unwrap(),
        &city,
        &streets,
    )
    .unwrap();
    let area: f64 = landscape
        .ground
        .iter()
        .map(|t| {
            let p = t.points;
            cross(
                sub(
                    [p[1][0] as f64, p[1][1] as f64],
                    [p[0][0] as f64, p[0][1] as f64],
                ),
                sub(
                    [p[2][0] as f64, p[2][1] as f64],
                    [p[0][0] as f64, p[0][1] as f64],
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
        landscape.ground.iter().all(|t| t.style == 1),
        "relation members must not create their own ground cover"
    );
}
