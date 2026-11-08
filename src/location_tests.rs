use crate::add_limassol::{create_location_map, elevation_tiles};
use crate::build_furniture::{build_furniture, directions};
use crate::build_landscape::{build_landscape, rings};
use crate::create_track::compile as compile_track;
use crate::download::{download, download_tiles, DownloadOptions};
use crate::imagery::{
    blank, blocks, city_origin, image_path, local, mercator, process, write_file, CORE, GUTTER,
    PAD, RES, SERVICE, TILE,
};
use crate::location::{add, cross, dist, invalid, line, mul, polygon, project, sha, sub, Result};
use crate::osm::{OSMElement, OSMMap};
use crate::sources::extract_osm_boundary;
use crate::world::{Boundary, LocationMap, P};
use crate::{boundary, build_surfaces, build_terrain, osm};
use geo::{
    Area, BooleanOps, Buffer, Contains, Intersects, MapCoords, MultiPolygon, Point, Polygon,
};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;
use std::{
    io::{Read, Write},
    net::TcpListener,
    time::Duration,
};

struct FixtureOptions<'a> {
    source: &'a Path,
    paths: CompilePaths<'a>,
    name: &'a str,
    bbox: Option<[f64; 4]>,
    elevation: &'a Path,
}
struct CompilePaths<'a> {
    city: &'a Path,
    streets: &'a Path,
    streets_report: &'a Path,
    landscape: &'a Path,
    furniture: &'a Path,
    heightfield: &'a Path,
    elevation_tiles_directory: &'a Path,
    elevation_attribution_source: &'a Path,
    elevation_attribution: &'a Path,
    source_copy: &'a Path,
    manifest: &'a Path,
    collision: &'a Path,
}
fn compile_fixture(options: FixtureOptions<'_>, boundary: Option<&Boundary>) -> Result<()> {
    println!("Reading OSM XML...");
    let source = osm::read(options.source)?;
    let map = if let Some(boundary) = boundary {
        todo!();
        // create_location_map(&source, options.bbox, Some(boundary))?
    } else {
        todo!();
        // compile(&source, options.bbox)?;
    };
    let paths = &options.paths;
    for path in [
        paths.city,
        paths.streets,
        paths.streets_report,
        paths.landscape,
        paths.furniture,
        paths.heightfield,
        paths.elevation_attribution,
        paths.source_copy,
        paths.manifest,
        paths.collision,
    ] {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
    }
    fs::create_dir_all(paths.elevation_tiles_directory)?;
    let city = osm::write_map_917(&map, paths.city)?;
    println!("Building streets ({} segments)...", map.roads.len());
    let surfaces =
        build_surfaces::build_surfaces(&map, &city, paths.streets, paths.streets_report)?;
    println!("Building mapped landscape and roadside controls...");
    build_landscape(&source, &map, &surfaces, &city, paths.landscape)?;
    println!("  Placing road controls...");
    build_furniture(&source, &map, &surfaces, &city, paths.furniture)?;
    println!("Building elevation...");
    build_terrain::build_terrain(
        &map,
        &city,
        options.elevation,
        paths.heightfield,
        paths.elevation_tiles_directory,
    )?;
    fs::copy(options.source, paths.source_copy)?;
    let mut manifest = serde_json::json!({ "name": options.name });
    if let Some(boundary) = boundary {
        manifest["boundary"] = boundary.to_geojson();
    }
    fs::write(
        paths.manifest,
        serde_json::to_string_pretty(&manifest)? + "\n",
    )?;
    println!("Baking collision data...");
    crate::build_collision::build_collisions(paths.city, paths.streets, paths.collision)?;
    Ok(())
}

struct TestOptions<'a> {
    source: &'a Path,
    output: &'a Path,
    name: &'a str,
    bbox: Option<[f64; 4]>,
    elevation: &'a Path,
}

fn compile(options: TestOptions<'_>) -> Result<()> {
    compile_fixture(
        FixtureOptions {
            source: options.source,
            name: options.name,
            bbox: options.bbox,
            elevation: options.elevation,
            paths: CompilePaths {
                city: &options.output.join("city.txt"),
                streets: &options.output.join("streets.txt"),
                streets_report: &options.output.join("streets.json"),
                landscape: &options.output.join("landscape.txt"),
                furniture: &options.output.join("furniture.txt"),
                heightfield: &options.output.join("elevation/heightfield.txt"),
                elevation_tiles_directory: &options.output.join("elevation"),
                elevation_attribution_source: &options
                    .elevation
                    .parent()
                    .unwrap()
                    .join("README.md"),
                elevation_attribution: &options.output.join("elevation/source-attribution.md"),
                source_copy: &options.output.join("source.osm"),
                manifest: &options.output.join("manifest.json"),
                collision: &options.output.join("streets.bin"),
            },
        },
        None,
    )
}

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
fn compiler_writes_assets_to_explicit_paths() {
    let temp = Temp::new();
    let root = &temp.0;
    let source = root.join("input.osm");
    let elevation = root.join("N35E033.hgt");
    let attribution = root.join("provider.txt");
    fs::write(&source, FIXTURE).unwrap();
    fs::write(&elevation, vec![0; 65 * 65 * 2]).unwrap();
    fs::write(&attribution, "Elevation provider").unwrap();
    let paths = CompilePaths {
        city: &root.join("geometry/map.data"),
        streets: &root.join("roads/surfaces.data"),
        streets_report: &root.join("reports/roads.json"),
        landscape: &root.join("vegetation/ground.data"),
        furniture: &root.join("controls/objects.data"),
        heightfield: &root.join("terrain/grid.data"),
        elevation_tiles_directory: &root.join("preserved-tiles"),
        elevation_attribution_source: &attribution,
        elevation_attribution: &root.join("credits/elevation.txt"),
        source_copy: &root.join("provenance/snapshot.osm"),
        manifest: &root.join("reports/location.json"),
        collision: &root.join("physics/roads.cache"),
    };
    let source_copy = paths.source_copy;
    let attribution_copy = paths.elevation_attribution;
    let outputs = [
        paths.city,
        paths.streets,
        paths.streets_report,
        paths.landscape,
        paths.furniture,
        paths.heightfield,
        paths.elevation_attribution,
        paths.source_copy,
        paths.manifest,
        paths.collision,
    ];
    compile_fixture(
        FixtureOptions {
            source: &source,
            paths,
            name: "explicit-paths",
            bbox: None,
            elevation: &elevation,
        },
        None,
    )
    .unwrap();
    for path in outputs {
        assert!(path.is_file(), "{}", path.display());
    }
    let city = fs::read_to_string(outputs[0]).unwrap();
    let streets = fs::read_to_string(outputs[1]).unwrap();
    let parsed = crate::city::City::parse(&city).unwrap();
    crate::street_surface::StreetSurface::load(&streets, &city, parsed.streets.len()).unwrap();
    assert_eq!(fs::read(source_copy).unwrap(), fs::read(source).unwrap());
    assert_eq!(
        fs::read_to_string(attribution_copy).unwrap(),
        "Elevation provider"
    );
    assert!(root.join("preserved-tiles/N35E033.hgt").is_file());
    assert!(!root.join("city.txt").exists());
    assert!(!root.join("elevation").exists());
}

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
    compile(TestOptions {
        source: &source,
        output: &output,
        name: "fixture",
        bbox: None,
        elevation: &tile,
    })
    .unwrap();
    let second = temp.0.join("second");
    compile(TestOptions {
        source: &source,
        output: &second,
        name: "second",
        bbox: None,
        elevation: &tile,
    })
    .unwrap();
    fs::create_dir(temp.0.join("broken")).unwrap();
    fs::write(temp.0.join("broken/city.txt"), "invalid map").unwrap();
    let result = std::process::Command::new(std::env::current_exe().unwrap())
        .env("TEST_LOCATION_CACHE", &temp.0)
        .args([
            "--exact",
            "location_tests::runtime_location_failure_and_eviction_probe",
            "--nocapture",
        ])
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
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
    compile(TestOptions {
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
    compile(TestOptions {
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

#[test]
fn runtime_location_failure_and_eviction_probe() {
    let Some(root) = std::env::var_os("TEST_LOCATION_CACHE") else {
        return;
    };
    let root = std::path::PathBuf::from(root);
    crate::city_assets::select(root.join("broken")).unwrap();
    assert!(crate::track::TrackId::Engomi.try_track().is_err());
    fs::write(root.join("broken/streets.txt"), "invalid surfaces").unwrap();
    assert!(crate::track::TrackId::Engomi.try_track().is_err());
    crate::city_assets::select(root.join("compiled")).unwrap();
    let workers: Vec<_> = (0..4)
        .map(|_| std::thread::spawn(|| crate::track::TrackId::Engomi.try_track().unwrap()))
        .collect();
    let tracks: Vec<_> = workers
        .into_iter()
        .map(|worker| worker.join().unwrap())
        .collect();
    let first = tracks[0].clone();
    assert!(tracks
        .iter()
        .all(|track| std::sync::Arc::ptr_eq(&first, track)));
    drop(tracks);
    let weak = std::sync::Arc::downgrade(&first);
    crate::city_assets::select(root.join("second")).unwrap();
    let second = crate::track::TrackId::Engomi.try_track().unwrap();
    assert!(first.city().is_some());
    drop(first);
    assert!(weak.upgrade().is_none(), "inactive city remained allocated");
    assert!(second.spawn().point.iter().all(|v| v.is_finite()));
    crate::city_assets::select(root.join("broken")).unwrap();
    assert!(crate::track::TrackId::Engomi.try_track().is_err());
    crate::city_assets::select(root.join("second")).unwrap();
    assert!(std::sync::Arc::ptr_eq(
        &second,
        &crate::track::TrackId::Engomi.try_track().unwrap()
    ));
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
            [points[2] - points[0], points[3] - points[1]],
            [points[4] - points[0], points[5] - points[1]],
        )
        .abs()
            / 2.;
        assert!(area.is_finite());
        areas[style] += area;
    }
    areas
}

fn map(nodes: &[P], roads: &[(usize, usize, f64, usize)], buildings: &[Vec<P>]) -> LocationMap {
    LocationMap {
        boundary: None,
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
    map: &LocationMap,
) -> (
    Temp,
    build_surfaces::Surfaces,
    crate::street_surface::StreetSurface,
) {
    let temp = Temp::new();
    let city = osm::write_map_917(map, &temp.0.join("city.txt")).unwrap();
    let s = build_surfaces::build_surfaces(
        map,
        &city,
        &temp.0.join("streets.txt"),
        &temp.0.join("streets.json"),
    )
    .unwrap();
    let loaded = crate::street_surface::StreetSurface::load(
        &std::fs::read_to_string(temp.0.join("streets.txt")).unwrap(),
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
    let index = build_surfaces::PolygonIndex::new(&s.obstacles);
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
    build_surfaces::triangles(&mut bytes, "t 0", &geometry).unwrap();
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
    let count =
        build_surfaces::triangles(&mut bytes, "t 0", &MultiPolygon(vec![tiny, normal])).unwrap();
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
fn element(id: i64, tags: &[(&str, &str)]) -> OSMElement {
    OSMElement {
        id,
        tags: tags
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
        ..Default::default()
    }
}
fn source() -> OSMMap {
    OSMMap {
        nodes: BTreeMap::new(),
        ways: BTreeMap::new(),
        relations: vec![],
        bounds: Some([35.160, 33.318, 35.164, 33.322]),
    }
}
fn node(s: &mut OSMMap, id: i64, p: P, tags: &[(&str, &str)]) {
    let mut e = element(id, tags);
    e.coord = Some([
        35.162 - p[1].to_degrees() / 6371000.,
        33.320 + p[0].to_degrees() / 6371000. / 35.162_f64.to_radians().cos(),
    ]);
    s.nodes.insert(id, e);
}
fn way(s: &mut OSMMap, id: i64, nodes: &[i64], tags: &[(&str, &str)]) {
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
    // New locations intentionally use their crop centre, rather than a fixed global origin.
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
    let city = std::fs::read_to_string(temp.0.join("city.txt")).unwrap();
    build_landscape(&s, &m, &surfaces, &city, &temp.0.join("landscape.txt")).unwrap();
    println!("  Placing road controls...");
    build_furniture(&s, &m, &surfaces, &city, &temp.0.join("furniture.txt")).unwrap();
    crate::traffic::load(
        &std::fs::read_to_string(temp.0.join("furniture.txt")).unwrap(),
        &city,
        &std::fs::read_to_string(temp.0.join("streets.txt")).unwrap(),
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
    let city = std::fs::read_to_string(temp.0.join("city.txt")).unwrap();
    build_landscape(&s, &m, &surfaces, &city, &temp.0.join("landscape.txt")).unwrap();
    println!("  Placing road controls...");
    build_furniture(&s, &m, &surfaces, &city, &temp.0.join("furniture.txt")).unwrap();
    let streets = std::fs::read_to_string(temp.0.join("streets.txt")).unwrap();
    let l = crate::landscape::Landscape::load(
        &std::fs::read_to_string(temp.0.join("landscape.txt")).unwrap(),
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
    let city = std::fs::read_to_string(temp.0.join("city.txt")).unwrap();
    build_landscape(&s, &m, &surfaces, &city, &temp.0.join("landscape.txt")).unwrap();
    println!("  Placing road controls...");
    build_furniture(&s, &m, &surfaces, &city, &temp.0.join("furniture.txt")).unwrap();
    let areas = landscape_areas(&temp.0.join("landscape.txt"));
    assert!((areas[1] - 150.).abs() < 0.01);
}

/// Run after importing the full crop; the historical bundle remains the reference.
#[test]
#[ignore = "requires VALIDATED_CITY pointing to a complete full Nicosia import"]
fn full_nicosia_import_preserves_network_buildings_and_surface_area() {
    use std::io::{BufRead, BufReader};
    let imported =
        std::path::PathBuf::from(std::env::var_os("VALIDATED_CITY").expect("set VALIDATED_CITY"));
    let bundled = Path::new(env!("CARGO_MANIFEST_DIR")).join("data/locations/nicosia");
    let original = std::fs::read_to_string(bundled.join("city.txt")).unwrap();
    let rebuilt = std::fs::read_to_string(imported.join("city.txt")).unwrap();
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
    let metadata: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(imported.join("streets.json")).unwrap())
            .unwrap();
    let reference: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(bundled.join("streets.json")).unwrap())
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
    let streets_crc = crate::build_surfaces::crc_file(&imported.join("streets.txt")).unwrap();
    let elevation = std::fs::read_to_string(imported.join("elevation/heightfield.txt")).unwrap();
    crate::elevation::HeightField::load(&elevation, &rebuilt).unwrap();
    for (file, kind, version) in [
        ("furniture.txt", "FURNITURE", 3),
        ("landscape.txt", "LANDSCAPE", 1),
    ] {
        let first = BufReader::new(fs::File::open(imported.join(file)).unwrap())
            .lines()
            .next()
            .unwrap()
            .unwrap();
        assert_eq!(
            first,
            format!("{kind} {version} {key:08x} {streets_crc:08x}")
        );
    }
    let controls = |folder: &Path| {
        let mut counts = BTreeMap::new();
        for line in std::fs::read_to_string(folder.join("furniture.txt"))
            .unwrap()
            .lines()
            .skip(1)
        {
            let f: Vec<_> = line.split_whitespace().collect();
            *counts
                .entry((f[0].to_string(), f[1].to_string()))
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
        std::fs::read_to_string(folder.join("landscape.txt"))
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
    let old_land = landscape_areas(&bundled.join("landscape.txt"));
    let new_land = landscape_areas(&imported.join("landscape.txt"));
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
        let a = old_land[i];
        let b = new_land[i];
        let relative = (a - b).abs() / a;
        // These are independently clipped/unioned meshes, not byte-identical
        // rebuilds. Keep class changes below 0.25%, plus a tighter total bound.
        assert!(relative < 0.0025, "{name} area changed: {a} vs {b}");
        landscape_deltas.insert(*name, relative);
    }
    let original_total: f64 = old_land.iter().sum();
    let rebuilt_total: f64 = new_land.iter().sum();
    let total_delta = (original_total - rebuilt_total).abs() / original_total;
    assert!(
        total_delta < 0.0001,
        "total land cover area changed: {original_total} vs {rebuilt_total}"
    );
    let report = serde_json::json!({"nodes":new.nodes.len(),"road_segments":new.streets.len(),"buildings":new.buildings.len(),"road_area_m2":area,"reference_road_area_m2":reference_area,"triangles":triangles,"curbs":curbs,"mapped_controls":new_controls.values().sum::<u64>(),"trees":new_trees.len(),"landscape_relative_area_differences":landscape_deltas,"landscape_total_relative_area_difference":total_delta,"scene_pack_bytes":std::fs::metadata(imported.join("scene.pack")).unwrap().len(),"reference_scene_pack_bytes":std::fs::metadata(bundled.join("scene.pack")).unwrap().len(),"checks":"geographic node positions; complete road metadata; building IDs/heights/ring sizes; asphalt area and winding; elevation loader; scenery CRC headers; mapped control/tree identities; land cover area per style within 0.25%, total within 0.01%"});
    fs::write(
        &imported.parent().unwrap().join("validation-report.json"),
        serde_json::to_string_pretty(&report).unwrap() + "\n",
    )
    .unwrap();
    println!("{report}");
}

#[test]
fn elevation_downloads_cover_both_sides_of_degree_edges() {
    assert_eq!(
        elevation_tiles(BoundingBox {
            south: 34.7,
            west: 32.99,
            north: 34.8,
            east: 33.01
        }),
        vec!["N34E032", "N34E033"]
    );
    assert_eq!(
        elevation_tiles(BoundingBox {
            south: 34.998,
            west: 33.2,
            north: 35.001,
            east: 33.3
        }),
        vec!["N34E033", "N35E033"]
    );
}

fn boundary_fixture() -> (tempfile::TempDir, std::path::PathBuf) {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("boundary.osm");
    fs::write(&path, r#"<osm version="0.6">
<node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="0.01"/>
<node id="3" lat="0.01" lon="0.01"/><node id="4" lat="0.01" lon="0"/>
<node id="5" lat="0.004" lon="0.004"/><node id="6" lat="0.004" lon="0.006"/>
<node id="7" lat="0.006" lon="0.006"/><node id="8" lat="0.006" lon="0.004"/>
<node id="9" lat="0.005" lon="-0.002"/><node id="10" lat="0.005" lon="0.012"/>
<node id="11" lat="0.02" lon="0.02"/><node id="12" lat="0.02" lon="0.021"/>
<node id="13" lat="0.021" lon="0.021"/><node id="14" lat="0.021" lon="0.02"/>
<way id="20"><nd ref="1"/><nd ref="2"/><nd ref="3"/></way>
<way id="21"><nd ref="1"/><nd ref="4"/><nd ref="3"/></way>
<way id="22"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/><nd ref="5"/></way>
<way id="23"><nd ref="9"/><nd ref="10"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>
<way id="24"><nd ref="11"/><nd ref="12"/><nd ref="13"/><nd ref="14"/><nd ref="11"/></way>
<relation id="2087230"><member type="way" ref="21" role="outer"/>
<member type="way" ref="22" role="inner"/><member type="way" ref="20" role="outer"/>
<member type="way" ref="24" role="outer"/>
<tag k="type" v="boundary"/><tag k="boundary" v="administrative"/></relation></osm>"#).unwrap();
    (temp, path)
}
#[test]
fn boundary_assembles_reversed_ways_holes_and_islands() {
    let (temp, path) = boundary_fixture();
    let b = Boundary::resolve(&path, 2087230).unwrap();
    assert_eq!(b.geometry.0.len(), 2);
    assert!(!b.geometry.intersects(&Point::new(0.005, 0.005)));
    assert!(b.geometry.intersects(&Point::new(0.002, 0.005)));
    assert!(b.geometry.intersects(&Point::new(0.0205, 0.0205)));
    let manifest_path = temp.path().join("manifest.json");
    fs::write(
        &manifest_path,
        serde_json::to_string_pretty(&json!({"boundary": b.to_geojson()})).unwrap() + "\n",
    )
    .unwrap();
    let manifest: Value = serde_json::from_slice(&fs::read(manifest_path).unwrap()).unwrap();
    let restored = Boundary::from_manifest(&manifest).unwrap().unwrap();
    assert_eq!(b.relation, restored.relation);
    assert_eq!(b.geometry, restored.geometry);
    assert!(Boundary::from_manifest(&json!({})).unwrap().is_none());
    assert!(Boundary::from_manifest(&json!({"boundary": {}})).is_err());
    let source = crate::osm::read(&path).unwrap();
    let map =
        crate::osm::create_location_map(&source, Some(b.bbox().to_array()), Some(&b)).unwrap();
    // Both original endpoints lie outside; clip through the hole into two roads.
    assert_eq!(map.roads.len(), 2);
    for road in &map.roads {
        assert_eq!(road.direction, 1);
        assert!(map.nodes[road.ends[0]][0] < map.nodes[road.ends[1]][0]);
        let midpoint = mul(add(map.nodes[road.ends[0]], map.nodes[road.ends[1]]), 0.5);
        assert!(map.contains(midpoint));
    }
    let output = temp.path().join("compiled");
    fs::create_dir(&output).unwrap();
    let city = crate::osm::write_map_917(&map, &output.join("city.txt")).unwrap();
    let surfaces = crate::build_surfaces::build_surfaces(
        &map,
        &city,
        &output.join("streets.txt"),
        &output.join("streets.json"),
    )
    .unwrap();
    let outside = surfaces.asphalt.difference(map.boundary.as_ref().unwrap());
    assert!(outside.unsigned_area() < 0.001);
}
#[test]
fn boundary_refuses_incomplete_geometry() {
    let (_temp, path) = boundary_fixture();
    let text = fs::read_to_string(&path)
        .unwrap()
        .replace("<member type=\"way\" ref=\"20\" role=\"outer\"/>", "");
    fs::write(&path, text).unwrap();
    assert!(Boundary::resolve(&path, 2087230).is_err());
}

#[test]
fn compass_faces_opposite_travel_and_explicit_directions_take_precedence() {
    let mut e = OSMElement::default();
    e.tags.insert("direction".into(), "N".into());
    let (d, inferred, travel) = directions(&e, 3);
    assert!(d.is_empty());
    assert!(!inferred);
    let p = travel.unwrap();
    assert!(p[0].abs() < 1e-10);
    assert!((p[1] - 1.).abs() < 1e-10);
    e.tags
        .insert("traffic_sign:direction".into(), "backward".into());
    assert_eq!(directions(&e, 3), (vec![-1], false, None));
    e.tags.clear();
    assert_eq!(directions(&e, 1), (vec![-1, 1], true, None));
}

#[test]
fn relation_members_join_in_either_direction_and_reject_open_rings() {
    let joined = rings(vec![vec![1, 2, 3], vec![1, 4, 3]]).unwrap();
    assert_eq!(joined.len(), 1);
    assert_eq!(joined[0].first(), joined[0].last());
    assert_eq!(joined[0].len(), 5);
    assert!(rings(vec![vec![1, 2], vec![3, 4]]).is_none());
}

#[test]
fn adjacent_tiles_are_filtered_continuously_across_the_seam() {
    let temp = crate::location_tests::Temp::new();
    for (x, offset) in [(6, 0), (7, 8)] {
        let values: Vec<u8> = (0..9)
            .flat_map(|_| (0..9).flat_map(|c| (100_i16 + offset + c).to_be_bytes()))
            .collect();
        fs::write(temp.0.join(format!("N50E{x:03}.hgt")), values).unwrap();
    }
    let map = LocationMap {
        boundary: None,
        origin: [50.5, 7.0],
        bounds: [-100.0, -100.0, 100.0, 100.0],
        nodes: vec![],
        roads: vec![],
        buildings: vec![],
    };
    let source = temp.0.join("N50E006.hgt");
    let (dem, sources) = Hgt::covering(&source, &map).unwrap();
    assert_eq!(sources.len(), 2);
    for longitude in [6.95, 7.0, 7.05] {
        let expected = 100.0 + (longitude - 6.0) * 8.0;
        assert!((dem.sample([50.5, longitude]).unwrap() - expected).abs() < 1e-9);
    }
    fs::remove_file(temp.0.join("N50E007.hgt")).unwrap();
    assert!(Hgt::covering(&source, &map).is_err());
}
#[test]
fn hgt_endian_orientation_interpolation_and_voids() {
    let temp = crate::location_tests::Temp::new();
    let path = temp.0.join("N35E033.hgt");
    let mut samples: Vec<i16> = (0..9)
        .flat_map(|r| (0..9).map(move |c| 100 + r * 2 + c))
        .collect();
    let save = |values: &[i16]| {
        fs::write(
            &path,
            values
                .iter()
                .flat_map(|n| n.to_be_bytes())
                .collect::<Vec<_>>(),
        )
        .unwrap()
    };
    save(&samples);
    let dem = Hgt::read(&path).unwrap();
    for (p, expected) in [
        ([35.5, 33.5], 112.0),
        ([35.625, 33.5], 110.0),
        ([35.5, 33.625], 113.0),
        ([35.5625, 33.5625], 111.5),
    ] {
        assert!((dem.sample(p).unwrap() - expected).abs() < 1e-10);
    }
    assert!(dem.sample([36.0, 33.0]).is_err());
    samples[40] = -32768;
    save(&samples);
    assert!(Hgt::read(&path).unwrap().sample([35.5, 33.5]).is_err());
}

fn serve(responses: Vec<(usize, &'static [u8])>) -> (String, std::thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}/file", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || {
        for (length, body) in responses {
            let (mut stream, _) = listener.accept().unwrap();
            stream
                .set_read_timeout(Some(Duration::from_secs(5)))
                .unwrap();
            let mut request = Vec::new();
            while !request.ends_with(b"\r\n\r\n") {
                let mut byte = [0];
                stream.read_exact(&mut byte).unwrap();
                request.push(byte[0]);
            }
            write!(
                stream,
                "HTTP/1.1 200 OK\r\nContent-Length: {length}\r\nConnection: close\r\n\r\n"
            )
            .unwrap();
            stream.write_all(body).unwrap();
        }
    });
    (url, server)
}

fn valid_payload(path: &Path) -> Result<bool> {
    Ok(fs::read(path)? == b"valid")
}

#[test]
fn retries_invalid_responses_and_replaces_invalid_cache() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("cached.data");
    fs::write(&path, b"invalid").unwrap();
    let (url, server) = serve(vec![(3, b"bad"), (5, b"valid")]);
    let options = DownloadOptions {
        attempts: 2,
        validate: valid_payload,
        replace_invalid: true,
        ..DownloadOptions::default()
    };
    download(&ureq::Agent::new_with_defaults(), &url, &path, &options).unwrap();
    server.join().unwrap();
    assert_eq!(fs::read(&path).unwrap(), b"valid");
    download(
        &ureq::Agent::new_with_defaults(),
        "invalid-url",
        &path,
        &options,
    )
    .unwrap();
    assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
}

#[test]
fn oversized_or_truncated_responses_do_not_replace_existing_files() {
    for (length, body, limit) in [
        (5, b"valid".as_slice(), Some(4)),
        (8, b"short".as_slice(), None),
    ] {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("cached.data");
        fs::write(&path, b"invalid").unwrap();
        let (url, server) = serve(vec![(length, body)]);
        let options = DownloadOptions {
            max_bytes: limit,
            validate: valid_payload,
            replace_invalid: true,
            ..DownloadOptions::default()
        };
        assert!(download(&ureq::Agent::new_with_defaults(), &url, &path, &options).is_err());
        server.join().unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"invalid");
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
    }
}

#[test]
fn default_options_reuse_nonempty_cached_files() {
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("cached.data");
    fs::write(&path, b"cached").unwrap();
    download(
        &ureq::Agent::new_with_defaults(),
        "invalid-url",
        &path,
        &DownloadOptions::default(),
    )
    .unwrap();
    assert_eq!(fs::read(&path).unwrap(), b"cached");
}

#[test]
fn preparation_uses_explicit_paths_and_resumes_installed_imagery() {
    let temp = tempfile::tempdir().unwrap();
    let root = temp.path();
    let paths = ImageryPaths {
        city: &root.join("map.data"),
        streets: &root.join("roads.data"),
        landscape: &root.join("vegetation.data"),
        manifest: &root.join("metadata.json"),
        downloads_directory: &root.join("raw-images"),
        blocks_directory: &root.join("block-cache"),
        service: &root.join("cache/provider.json"),
        plan: &root.join("cache/job.json"),
        previous_trees: &root.join("cache/original-trees.data"),
        report: &root.join("cache/result.json"),
        trees: &root.join("cache/crowns.data"),
        installed_trees: &root.join("outputs/trees.data"),
        installed_trees_report: &root.join("outputs/trees-report.json"),
        aerial_backup: &root.join("backup/old-imagery"),
        staged_aerial_directory: &root.join("scratch/textures"),
        installed_aerial: AerialPaths {
            directory: &root.join("outputs/textures"),
            service: &root.join("outputs/provenance/provider.json"),
            plan: &root.join("outputs/provenance/job.json"),
            catalog: &root.join("outputs/catalog.data"),
            source: &root.join("outputs/provenance/source.json"),
        },
    };
    let city =
        "CITY 2\norigin 35.162 33.32\nbounds -100 -100 100 100\nn -20 0\nn 20 0\nr 0 1 8 0 1\n";
    let streets = format!("STREETS 1 1 {:08x}\n", crate::street_surface::crc32(city));
    fs::write(paths.city, city).unwrap();
    fs::write(paths.streets, &streets).unwrap();
    fs::write(
        paths.landscape,
        format!(
            "LANDSCAPE 1 {:08x} {:08x}\n",
            crate::street_surface::crc32(city),
            crate::street_surface::crc32(&streets)
        ),
    )
    .unwrap();
    let bbox = [35.162, 33.32, 35.163, 33.321];
    fs::write(paths.manifest, b"{}").unwrap();
    fs::create_dir_all(paths.downloads_directory).unwrap();
    fs::create_dir_all(paths.service.parent().unwrap()).unwrap();
    fs::write(paths.service, "{}").unwrap();
    let side = CORE + 2 * PAD;
    let image = image::RgbImage::from_pixel(side, side, image::Rgb([50, 70, 60]));
    for block in blocks(bbox) {
        image
            .save(image_path(paths.downloads_directory, &block))
            .unwrap();
    }
    fs::create_dir_all(paths.installed_aerial.directory).unwrap();
    fs::write(paths.installed_aerial.directory.join("old-marker"), "old").unwrap();

    prepare_imagery_fixture(&paths, bbox).unwrap();
    assert!(paths.aerial_backup.join("old-marker").is_file());
    assert!(!paths.staged_aerial_directory.exists());
    for path in [
        paths.installed_aerial.service,
        paths.installed_aerial.plan,
        paths.installed_aerial.catalog,
        paths.installed_aerial.source,
        paths.report,
        paths.trees,
        paths.installed_trees,
        paths.installed_trees_report,
    ] {
        assert!(path.is_file(), "{}", path.display());
    }
    let catalog = fs::read(paths.installed_aerial.catalog).unwrap();
    prepare_imagery_fixture(&paths, bbox).unwrap();
    assert_eq!(fs::read(paths.installed_aerial.catalog).unwrap(), catalog);
    assert_eq!(
        fs::read(paths.trees).unwrap(),
        fs::read(paths.installed_trees).unwrap()
    );
}

#[test]
fn grid_is_contiguous_and_gutters_match() {
    let b = blocks([35.1, 33.25, 35.23, 33.47]);
    assert!(b.len() > 200 && b.len() < 500);
    let a = &b[0];
    let next = &b[1];
    assert!((next.extent[0] - a.extent[0] - CORE as f64 * RES).abs() < 1e-6);
    assert!((a.extent[2] - next.extent[0] - 2. * PAD as f64 * RES).abs() < 1e-6);
    let last = b.last().unwrap();
    let sw = mercator(33.25, 35.1);
    let ne = mercator(33.47, 35.23);
    assert!(a.extent[0] < sw[0] && a.extent[3] > ne[1]);
    assert!(last.extent[2] > ne[0] && last.extent[1] < sw[1]);
}
#[test]
fn block_resume_validates_jpeg_checksums_and_gutters() {
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
    let first = process(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap();
    assert_eq!(first["tiles"].as_array().unwrap().len(), 16);
    let a = fs::read(aerial.join("0_0_0_0.jpg")).unwrap();
    let decoded = image::load_from_memory(&a).unwrap().to_rgb8().into_raw();
    let right = image::open(aerial.join("0_0_1_0.jpg"))
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
        process(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
    );
    fs::write(aerial.join("0_0_0_0.jpg"), b"damaged").unwrap();
    assert_eq!(
        first,
        process(&work.join("downloads"), work, &aerial, &b, [35.162, 33.32]).unwrap()
    );
    assert_eq!(a, fs::read(aerial.join("0_0_0_0.jpg")).unwrap());
}
#[test]
fn white_coverage_is_not_published_as_imagery() {
    let mut im = image::RgbImage::from_pixel(TILE, TILE, image::Rgb([255, 255, 255]));
    assert!(blank(&im, 0, 0));
    for y in 0..TILE {
        for x in 0..TILE {
            im.put_pixel(x, y, image::Rgb([50, 70, 60]));
        }
    }
    assert!(!blank(&im, 0, 0));
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
fn pbf_fixture(path: &Path) {
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
    let block = [bytes(1, &table), bytes(2, &group)].concat();
    let blob = bytes(1, &block);
    let header = [bytes(1, b"OSMData"), uint(3, blob.len() as u64)].concat();
    let data = [(header.len() as u32).to_be_bytes().to_vec(), header, blob].concat();
    fs::write(path, data).unwrap();
}
#[test]
fn extraction_completes_outside_ways_and_multipolygons() {
    let dir = tempfile::tempdir().unwrap();
    let pbf = dir.path().join("source.pbf");
    let output = dir.path().join("region.osm");
    pbf_fixture(&pbf);
    extract_osm_boundary(
        &pbf,
        &output,
        BoundingBox {
            south: -0.1,
            west: -0.1,
            north: 0.1,
            east: 0.1,
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
    assert_eq!(source.relations[0].id, 20);
    assert_eq!(source.nodes[&1].tag("name"), "A & B");
    let xml = fs::read_to_string(&output).unwrap();
    assert!(xml.contains("version=\"7\""));
    assert!(xml.contains("A &amp; B"));
    let report: serde_json::Value =
        serde_json::from_slice(&fs::read(output.with_extension("json")).unwrap()).unwrap();
    assert_eq!(report["nodes"], 4);
    assert_eq!(report["multipolygons"], 1);
}

#[test]
fn points_compile_to_loadable_city_assets_and_rebake_without_the_json() {
    let temp = tempfile::tempdir().unwrap();
    let source = temp.path().join("course.json");
    fs::write(&source, r#"{"version":2,"name":"Coast & Curves","scenery":"coastal","points":[[0,2,0,4],[100,4,0,12],[100,8,100,6],[0,3,100,8]]}"#).unwrap();
    let track = crate::track::register_file(&source).unwrap().0.track();
    let output = temp.path().join("compiled");
    fs::create_dir(&output).unwrap();
    compile_track(&source, &track, &output).unwrap();
    let assets = crate::city_assets::CityAsset::new(output.clone());
    let city = crate::city::City::load(&assets).unwrap();
    assert!(city.streets.len() > 400);
    assert!(city.buildings.is_empty());
    assert!(!city.landscape.trees.is_empty());
    assert!(city.google_maps_url([0.0; 3]).is_none());
    assert_eq!(city.street_name(city.spawn().point), "COAST & CURVES");
    assert_eq!(city.streets.last().unwrap().ends[1], 0);
    for sample in [
        track.at_distance(0.0),
        track.at_distance(track.length() * 0.4),
    ] {
        let projected = city.project(sample.point);
        assert!((projected.width - sample.width).abs() < 0.1);
        assert!(
            (city.height([sample.point[0], sample.point[2]])
                - track.ground_height([sample.point[0], sample.point[2]]))
            .abs()
                < 0.0001
        );
        let position = sample.lane_position(40.0);
        let mut moved = position;
        city.constrain(&mut moved, &mut [0.0; 2]);
        assert_eq!([moved[0], moved[2]], [position[0], position[2]]);
    }
    let before = sha(&output.join("scene.pack")).unwrap();
    fs::remove_file(output.join("source.track.json")).unwrap();
    crate::build_collision::build_collisions(
        &output.join("city.txt"),
        &output.join("streets.txt"),
        &output.join("streets.bin"),
    )
    .unwrap();
    crate::build_scene::build_scene(
        &city,
        None,
        &output.join("decorations.pack"),
        &output.join("scene.pack"),
        &output,
    )
    .unwrap();
    assert_eq!(sha(&output.join("scene.pack")).unwrap(), before);
    let archive = crate::scene_archive::Archive::open(&output.join("scene.pack")).unwrap();
    assert!(!archive.entries.is_empty());
    assert!(assets.get_manifest().unwrap().get("scenery").is_none());

    // Inputs, output, and spool files can all live outside the location directory.
    let decorations_path = temp.path().join("props.pack");
    let output_path = temp.path().join("render.pack");
    let spool_directory = temp.path().join("spool");
    fs::remove_file(output.join("manifest.json")).unwrap();
    fs::rename(output.join("decorations.pack"), &decorations_path).unwrap();
    fs::create_dir(&spool_directory).unwrap();
    crate::build_scene::build_scene(
        &city,
        None,
        &decorations_path,
        &output_path,
        &spool_directory,
    )
    .unwrap();
    assert_eq!(sha(&output_path).unwrap(), before);
    assert_eq!(fs::read_dir(&spool_directory).unwrap().count(), 0);
}

#[test]
fn standard_scenery_and_two_metre_roads_use_the_normal_city_loader() {
    let temp = tempfile::tempdir().unwrap();
    let source = temp.path().join("narrow.json");
    fs::write(
        &source,
        r#"{"version":2,"name":"Narrow","points":[[0,0,0,2],[30,0,0,2],[30,0,30,2],[0,0,30,2]]}"#,
    )
    .unwrap();
    let track = crate::track::register_file(&source).unwrap().0.track();
    let output = temp.path().join("compiled");
    fs::create_dir(&output).unwrap();
    compile_track(&source, &track, &output).unwrap();
    let city = crate::city::City::load(&crate::city_assets::CityAsset::new(output)).unwrap();
    assert!(city.streets.iter().all(|s| s.width == 2.0));
    assert!(city.spawn().point.iter().all(|x| x.is_finite()));
    assert!(!city.landscape.trees.is_empty());
    assert!(!city.buildings.is_empty());
}

struct AerialPaths<'a> {
    directory: &'a Path,
    service: &'a Path,
    plan: &'a Path,
    catalog: &'a Path,
    source: &'a Path,
}

struct ImageryPaths<'a> {
    city: &'a Path,
    streets: &'a Path,
    landscape: &'a Path,
    manifest: &'a Path,
    downloads_directory: &'a Path,
    blocks_directory: &'a Path,
    service: &'a Path,
    plan: &'a Path,
    previous_trees: &'a Path,
    report: &'a Path,
    trees: &'a Path,
    installed_trees: &'a Path,
    installed_trees_report: &'a Path,
    aerial_backup: &'a Path,
    staged_aerial_directory: &'a Path,
    installed_aerial: AerialPaths<'a>,
}

fn prepare_imagery_fixture(paths: &ImageryPaths<'_>, bbox: [f64; 4]) -> Result<()> {
    for path in [
        paths.service,
        paths.plan,
        paths.previous_trees,
        paths.report,
        paths.trees,
        paths.installed_trees,
        paths.installed_trees_report,
        paths.aerial_backup,
        paths.installed_aerial.directory,
    ] {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
    }
    let city_text = std::fs::read_to_string(paths.city)?;
    let city = crate::city::City::parse(&city_text).map_err(invalid)?;
    let origin = city_origin(&city_text)?;
    let manifest: Value = serde_json::from_slice(&std::fs::read(paths.manifest)?)?;
    if bbox.iter().any(|v| !v.is_finite())
        || bbox[0] >= bbox[2]
        || bbox[1] >= bbox[3]
        || bbox[0] < -85.
        || bbox[2] > 85.
        || bbox[1] < -180.
        || bbox[3] > 180.
    {
        return Err(invalid("invalid city geographic bounds"));
    }
    let boundary = boundary::Boundary::from_manifest(&manifest)?;
    let local_boundary = boundary.as_ref().map(|b| b.local(origin));
    let mercator_boundary = boundary.as_ref().map(|b| {
        b.geometry.map_coords(|c| {
            let p = mercator(c.x, c.y);
            geo::Coord { x: p[0], y: p[1] }
        })
    });
    let all: Vec<_> = blocks(bbox)
        .into_iter()
        .filter(|b| {
            mercator_boundary.as_ref().is_none_or(|g| {
                g.intersects(&geo::Rect::new(
                    geo::Coord {
                        x: b.extent[0],
                        y: b.extent[1],
                    },
                    geo::Coord {
                        x: b.extent[2],
                        y: b.extent[3],
                    },
                ))
            })
        })
        .collect();
    if all.is_empty() || all.len() > 2000 {
        return Err(invalid(
            "imagery job must contain 1..2000 blocks; crop the map",
        ));
    }
    let mut plan = json!({"version":1,"source":SERVICE,"bbox":bbox,"origin":origin,"resolution_mercator":RES,"core":CORE,"padding":PAD,"tile_size":TILE,"gutter":GUTTER,"blocks":all.len(),"city_sha256":sha(paths.city)?});
    if boundary.is_some() {
        plan["boundary"] = manifest["boundary"].clone();
    }
    let plan_path = paths.plan;
    if plan_path.exists() {
        let previous: Value = serde_json::from_slice(&fs::read(&plan_path)?)?;
        if previous != plan {
            return Err(invalid(
                "work directory contains a different imagery plan; choose a new directory",
            ));
        }
    } else {
        write_file(&plan_path, &serde_json::to_vec_pretty(&plan)?)?;
    }
    println!(
        "City imagery: {} blocks, up to {} tiles; approximately {:.2} m/pixel on the ground.",
        all.len(),
        all.len() * 16,
        RES * origin[0].to_radians().cos()
    );
    download_tiles(paths.downloads_directory, paths.service, &all)?;
    let plan_hash = sha(&plan_path)?;
    let installed = paths.installed_aerial.source;
    let installed_plan = fs::read(&installed)
        .ok()
        .and_then(|s| serde_json::from_slice::<Value>(&s).ok())
        .is_some_and(|v| v["plan_sha256"] == plan_hash);
    let aerial = if installed_plan {
        paths.installed_aerial.directory
    } else {
        paths.staged_aerial_directory
    };
    fs::create_dir_all(aerial)?;
    let pilot = paths.previous_trees;
    if !pilot.exists() {
        if let Ok(data) = fs::read(paths.installed_trees) {
            write_file(&pilot, &data)?;
        }
    }
    let mut catalog = format!("AERIAL 1\nsize {TILE} {GUTTER}\n");
    let mut drafts = vec![];
    let (mut tile_count, mut missing) = (0, 0);
    for (i, b) in all.iter().enumerate() {
        let meta = process(
            paths.downloads_directory,
            paths.blocks_directory,
            aerial,
            b,
            origin,
        )?;
        for t in meta["tiles"]
            .as_array()
            .ok_or_else(|| invalid("missing block tiles"))?
        {
            let bounds = t["bounds"]
                .as_array()
                .ok_or_else(|| invalid("missing tile bounds"))?;
            if local_boundary.as_ref().is_some_and(|g| {
                !g.intersects(&geo::Rect::new(
                    geo::Coord {
                        x: bounds[0].as_f64().unwrap(),
                        y: bounds[1].as_f64().unwrap(),
                    },
                    geo::Coord {
                        x: bounds[2].as_f64().unwrap(),
                        y: bounds[3].as_f64().unwrap(),
                    },
                ))
            }) {
                continue;
            }
            catalog.push_str(&format!(
                "tile {:.6} {:.6} {:.6} {:.6} {} {:08x}\n",
                bounds[0].as_f64().unwrap(),
                bounds[1].as_f64().unwrap(),
                bounds[2].as_f64().unwrap(),
                bounds[3].as_f64().unwrap(),
                t["name"].as_str().unwrap(),
                t["crc"].as_u64().unwrap()
            ));
            tile_count += 1;
        }
        for t in meta["trees"]
            .as_array()
            .ok_or_else(|| invalid("missing block crowns"))?
        {
            drafts.push([
                t[0].as_f64().unwrap(),
                t[1].as_f64().unwrap(),
                t[2].as_f64().unwrap(),
            ]);
        }
        missing += meta["missing_tiles"].as_u64().unwrap_or(0);
        if (i + 1) % 10 == 0 || i + 1 == all.len() {
            println!(
                "Prepared blocks: {}/{}; {tile_count} tiles, {} crown proposals",
                i + 1,
                all.len(),
                drafts.len()
            );
        }
    }
    if tile_count == 0 {
        return Err(invalid(
            "source returned no usable imagery; city assets unchanged",
        ));
    }
    println!(
        "Filtering {} crown proposals against roads, buildings and mapped trees...",
        drafts.len()
    );
    let obstacles = crate::tree_obstacles::Obstacles::read_paths(
        paths.city,
        paths.streets,
        [bbox[1], bbox[0], bbox[3], bbox[2]],
    )?;
    let landscape = fs::read_to_string(paths.landscape)?;
    let header = landscape
        .lines()
        .next()
        .ok_or_else(|| invalid("missing landscape header"))?;
    let expected = format!(
        "LANDSCAPE 1 {:08x} {:08x}",
        crate::street_surface::crc32(&city_text),
        crate::imagery_trees::checksum(paths.streets)?
    );
    if header != expected {
        return Err(invalid("landscape does not match city and roads"));
    }
    let mut index = rstar::RTree::new();
    for line in landscape.lines() {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.first() == Some(&"t") && f.len() == 6 {
            index.insert([f[2].parse::<f64>()?, f[3].parse::<f64>()?]);
        }
    }
    let mut output = format!("{header}\n");
    let (mut count, mut conflicts, mut duplicates, mut outside) = (0, 0, 0, 0);
    let inside = |p: [f64; 2]| {
        p[0] >= city.bounds[0][0] as f64
            && p[1] >= city.bounds[0][1] as f64
            && p[0] <= city.bounds[1][0] as f64
            && p[1] <= city.bounds[1][1] as f64
    };
    // Preserve the already approved pilot; new imagery proposals are deduplicated around it.
    let mut append = |p: [f64; 2], radius: f64, height: f64| {
        let p = p.map(|v| (v * 1000.).round() / 1000.);
        if !inside(p)
            || local_boundary
                .as_ref()
                .is_some_and(|b| !b.intersects(&geo::Point::new(p[0], p[1])))
        {
            outside += 1;
            return;
        }
        if obstacles.rejection_at(p).is_some() {
            conflicts += 1;
            return;
        }
        if index
            .nearest_neighbor(&p)
            .is_some_and(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < 2.)
        {
            duplicates += 1;
            return;
        }
        let id = (1_u64 << 63) | count;
        output.push_str(&format!(
            "t {id} {:.3} {:.3} {:.3} {:.3}\n",
            p[0],
            p[1],
            height.clamp(3.5, 14.),
            radius.clamp(0.7, 4.5)
        ));
        index.insert(p);
        count += 1;
    };
    if pilot.exists() {
        for line in fs::read_to_string(&pilot)?.lines().skip(1) {
            let f: Vec<_> = line.split_whitespace().collect();
            if f.len() == 6 && f[0] == "t" {
                append([f[2].parse()?, f[3].parse()?], f[5].parse()?, f[4].parse()?);
            }
        }
    }
    for d in drafts {
        let radius = d[2].clamp(0.7, 4.5);
        append(local(origin, [d[0], d[1]]), radius, radius * 2.8);
    }
    if plan["city_sha256"] != sha(paths.city)? {
        return Err(invalid(
            "city changed during imagery preparation; choose a fresh work directory",
        ));
    }
    let report = json!({"plan_sha256":plan_hash,"source":SERVICE,"attribution":"Source: Esri, Vantor, Earthstar Geographics, and the GIS User Community","local_development_only":true,"bbox":bbox,"blocks":all.len(),"tile_count":tile_count,"missing_tiles":missing,"imagery_trees":count,"excluded_road_or_building":conflicts,"excluded_duplicates":duplicates,"excluded_outside_city":outside,"ground_sampling_metres":RES*origin[0].to_radians().cos(),"sizes":"Tree sizes are artistic estimates; candidates inferred by RGB vegetation/texture heuristics.","rights":"Local development pack; redistribution rights have not been established."});
    write_file(paths.report, &serde_json::to_vec_pretty(&report)?)?;
    write_file(paths.trees, output.as_bytes())?;
    if !installed_plan {
        let backup = paths.aerial_backup;
        let destination = paths.installed_aerial.directory;
        if destination.exists() {
            if backup.exists() {
                return Err(invalid(
                    "previous-aerial backup already exists; inspect it before installing",
                ));
            }
            fs::rename(&destination, &backup)?;
        }
        if let Err(e) = fs::rename(aerial, destination) {
            if backup.exists() {
                let _ = fs::rename(&backup, &destination);
            }
            return Err(e.into());
        }
    }
    // Provenance belongs to the installed pack, not the disposable work cache.
    let aerial = &paths.installed_aerial;
    for path in [aerial.service, aerial.plan, aerial.catalog, aerial.source] {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
    }
    write_file(aerial.service, &fs::read(paths.service)?)?;
    write_file(aerial.plan, &fs::read(paths.plan)?)?;
    write_file(aerial.catalog, catalog.as_bytes())?;
    write_file(aerial.source, &serde_json::to_vec_pretty(&report)?)?;
    write_file(
        paths.installed_trees_report,
        &serde_json::to_vec_pretty(&report)?,
    )?;
    write_file(paths.installed_trees, output.as_bytes())?;
    println!("Installed {tile_count} imagery tiles and {count} trees; {missing} missing imagery tiles. Run bake-city --location NAME to rebuild the playable scene.");
    Ok(())
}
