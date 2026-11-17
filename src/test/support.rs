//! Shared fixture builders and controlled inputs for module tests.

use crate::asset;
use crate::build;
pub use crate::build::streets_test_support::{compile_collisions, load_streets, PreparedStreets};
use crate::model;
use crate::model::world::Building;
use crate::physics;
use crate::render;
use crate::storage;
use crate::storage::city_assets::CityAsset;
use crate::util;
use crate::util::geometry::polygon;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
pub use crate::world::roads::test_support::*;
use geo::MultiPolygon;
use std::path::Path;
use std::sync::{Arc, OnceLock};

pub fn bridge_deck(
    road: usize,
    height: f32,
    z: f32,
    start: f32,
    end: f32,
) -> model::map_details::Deck {
    model::map_details::Deck {
        road,
        grade: 1,
        width: 8.,
        path: vec![[start, height, z], [end, height, z]],
        surface: model::map_details::RoadSurface::Asphalt,
    }
}

pub fn road_footprint(surface: &world::street_surface::StreetSurface) -> MultiPolygon<f64> {
    build::surfaces::union(
        &world::street_surface::road_triangles(surface)
            .iter()
            .filter(|triangle| model::world::is_road(triangle.usage))
            .map(|triangle| {
                MultiPolygon(vec![polygon(&triangle.points.map(|point| XZ {
                    x: point.x as f64,
                    z: point.z as f64,
                }))])
            })
            .collect::<Vec<_>>(),
    )
}

pub fn audio_voice() -> &'static model::car::EngineVoice {
    static VOICE: OnceLock<model::car::EngineVoice> = OnceLock::new();
    VOICE.get_or_init(|| model::car::EngineVoice {
        harmonics: vec![[1.0, 0.6], [2.0, 0.25], [3.0, 0.15]],
        gain: 2.0,
        coast_gain: 0.4,
        soft_limit: None,
        sample_directory: None,
    })
}

/// Mono 48 kHz signed 16-bit PCM in a minimal RIFF/WAVE container.
pub fn engine_wav(pcm: &[u8]) -> Vec<u8> {
    let mut wav = Vec::new();
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&(36 + pcm.len() as u32).to_le_bytes());
    wav.extend_from_slice(b"WAVEfmt ");
    wav.extend_from_slice(&16_u32.to_le_bytes());
    wav.extend_from_slice(&1_u16.to_le_bytes()); // PCM
    wav.extend_from_slice(&1_u16.to_le_bytes()); // Mono
    wav.extend_from_slice(&48_000_u32.to_le_bytes());
    wav.extend_from_slice(&96_000_u32.to_le_bytes()); // Bytes per second
    wav.extend_from_slice(&2_u16.to_le_bytes()); // Bytes per frame
    wav.extend_from_slice(&16_u16.to_le_bytes());
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&(pcm.len() as u32).to_le_bytes());
    wav.extend_from_slice(pcm);
    wav
}

pub fn car_spec() -> &'static model::car::CarSpec {
    static SPEC: OnceLock<model::car::CarSpec> = OnceLock::new();
    SPEC.get_or_init(|| {
        let vertex = |position| render::scene::Vertex {
            position,
            color: [0.5; 3],
            normal: [0.0, 1.0, 0.0],
            material: [0.0; 2],
        };
        let design = Arc::new(asset::car_glb::Design {
            body: vec![
                vertex([-0.9, -0.3, -2.0]),
                vertex([0.9, -0.3, -2.0]),
                vertex([0.0, 1.0, 2.0]),
            ],
            wheels: std::array::from_fn(|_| {
                vec![
                    vertex([0.0; 3]),
                    vertex([0.0, 0.32, 0.0]),
                    vertex([0.0, 0.0, 0.32]),
                ]
            }),
            mounts: [
                XYZ {
                    x: -0.75,
                    y: -0.1,
                    z: -1.25,
                },
                XYZ {
                    x: 0.75,
                    y: -0.1,
                    z: -1.25,
                },
                XYZ {
                    x: -0.75,
                    y: -0.1,
                    z: 1.25,
                },
                XYZ {
                    x: 0.75,
                    y: -0.1,
                    z: 1.25,
                },
            ],
            wheel_radii: [0.32; 4],
        });
        let frontal_area_m2 = asset::car_glb::frontal_area_m2(&design);
        model::car::CarSpec {
            design,
            name: "TEST VEHICLE".into(),
            paint_color: [1.0, 0.18, 0.08],
            weight_kg: 1200.0,
            top_speed: 50.0,
            brake_force: 22000.0,
            wheelbase: 2.5,
            rolling_radius: 0.32,
            audio: audio_voice().clone(),
            front_weight: 0.5,
            cg_height: 0.5,
            track_width: 1.5,
            yaw_inertia: 1500.0,
            drive_bias: 0.0,
            brake_bias: 0.65,
            corner_stiffness: [70000.0; 2],
            engine_max_torque_nm: 180.0,
            engine_torque_curve: vec![
                model::car::TorquePoint {
                    rpm: 0.0,
                    torque: 0.6,
                },
                model::car::TorquePoint {
                    rpm: 3500.0,
                    torque: 1.0,
                },
                model::car::TorquePoint {
                    rpm: 6500.0,
                    torque: 0.9,
                },
                model::car::TorquePoint {
                    rpm: 33500.0,
                    torque: 0.0,
                },
            ],
            redline_rpm: 6500.0,
            suspension_hz: 1.7,
            gearbox: model::car::Gearbox {
                ratios: vec![3.0, 2.0, 1.4, 1.0, 0.8],
                final_drive: 4.0,
                reverse_ratio: 3.0,
            },
            engine_power_watts: 120.0 * model::car::WATTS_PER_HP,
            drag_coefficient: 0.3,
            frontal_area_m2,
            turbo: None,
        }
    })
}

pub fn track() -> Arc<world::locations::WorldLocation> {
    let file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(file.path(), r#"{"version":2,"name":"Module test road","scenery":"coastal","points":[[0,3,0,8],[400,6,0,8],[400,5,400,8],[0,3,400,8]]}"#).unwrap();
    world::locations::try_track(&world::locations::register_file(file.path()).unwrap().0).unwrap()
}

pub fn car_definition() -> serde_json::Value {
    let s = car_spec();
    serde_json::json!({
        "name": s.name, "paint_color": s.paint_color,
        "weight_kg": s.weight_kg, "brake_force": s.brake_force,
        "engine_power_hp": s.engine_power_watts / model::car::WATTS_PER_HP, "redline_rpm": s.redline_rpm,
        "drag_coefficient": s.drag_coefficient, "front_weight": s.front_weight,
        "cg_height_fraction": s.cg_height / asset::car_glb::height(&s.design), "yaw_inertia": s.yaw_inertia,
        "drive_bias": s.drive_bias, "brake_bias": s.brake_bias,
        "corner_stiffness": s.corner_stiffness, "engine_max_torque_nm": s.engine_max_torque_nm,
        "engine_torque_curve": s.engine_torque_curve.iter().map(|point|
            serde_json::json!({"rpm": point.rpm, "torque": point.torque})).collect::<Vec<_>>(),
        "suspension_hz": s.suspension_hz,
        "gearbox": {"ratios": s.gearbox.ratios, "final_drive": s.gearbox.final_drive,
            "reverse_ratio": s.gearbox.reverse_ratio},
        "audio": {"harmonics": s.audio.harmonics, "gain": s.audio.gain,
            "coast_gain": s.audio.coast_gain, "soft_limit": s.audio.soft_limit}
    })
}

/// Minimal GLB made from controlled triangle geometry, with independently editable parts.
pub fn model_glb(design: &asset::car_glb::Design) -> Vec<u8> {
    use serde_json::json;
    let mut binary = Vec::new();
    let mut views = Vec::new();
    let mut accessors = Vec::new();
    let mut meshes = Vec::new();
    let mut nodes = Vec::new();
    for (index, (name, vertices)) in [
        ("body", &design.body),
        ("wheel_rl", &design.wheels[0]),
        ("wheel_rr", &design.wheels[1]),
        ("wheel_fl", &design.wheels[2]),
        ("wheel_fr", &design.wheels[3]),
    ]
    .into_iter()
    .enumerate()
    {
        let offset = binary.len();
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for vertex in vertices {
            for axis in 0..3 {
                let value = vertex.position[axis];
                binary.extend(value.to_le_bytes());
                min[axis] = min[axis].min(value);
                max[axis] = max[axis].max(value);
            }
        }
        views.push(json!({"buffer":0,"byteOffset":offset,"byteLength":binary.len()-offset}));
        accessors.push(json!({"bufferView":index,"componentType":5126,"count":vertices.len(),"type":"VEC3","min":min,"max":max}));
        meshes.push(json!({"primitives":[{"attributes":{"POSITION":index}}]}));
        let translation = if index == 0 {
            [0.0; 3]
        } else {
            (design.mounts[index - 1]).into()
        };
        nodes.push(json!({"name":name,"mesh":index,"translation":translation}));
    }
    let document = json!({
        "asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0,1,2,3,4]}],
        "nodes":nodes,"meshes":meshes,"buffers":[{"byteLength":binary.len()}],
        "bufferViews":views,"accessors":accessors
    });
    let mut json = serde_json::to_vec(&document).unwrap();
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let mut glb = Vec::new();
    for word in [
        0x46546c67_u32,
        2,
        (28 + json.len() + binary.len()) as u32,
        json.len() as u32,
        0x4e4f534a,
    ] {
        glb.extend(word.to_le_bytes());
    }
    glb.extend(json);
    glb.extend((binary.len() as u32).to_le_bytes());
    glb.extend(0x004e4942_u32.to_le_bytes());
    glb.extend(binary);
    glb
}

pub fn write_car(directory: &std::path::Path) -> serde_json::Value {
    std::fs::create_dir_all(directory).unwrap();
    let definition = car_definition();
    std::fs::write(
        directory.join("car.json"),
        serde_json::to_vec(&definition).unwrap(),
    )
    .unwrap();
    std::fs::write(directory.join("model.glb"), model_glb(&car_spec().design)).unwrap();
    definition
}

pub fn cars() -> Vec<std::sync::Arc<model::car::CarSpec>> {
    [0.0, 0.5, 1.0]
        .into_iter()
        .map(|drive_bias| {
            let mut spec = car_spec().clone();
            spec.drive_bias = drive_bias;
            spec.weight_kg *= 1.0 + drive_bias * 0.5;
            // Include a launch-capable AWD fixture as well as both driven axles.
            spec.engine_max_torque_nm *= 3.0;
            spec.engine_power_watts *= 3.0;
            Arc::new(spec)
        })
        .collect()
}

/// Tiny generated world with a real scene archive and one raw aerial tile.
/// All fixtures use the same relative filenames to expose accidental cross-world reads.
pub fn generated_location(directory: &std::path::Path, height: f32, provider: &str) -> CityAsset {
    use std::fs;
    fs::create_dir_all(directory.join("elevation")).unwrap();
    fs::create_dir_all(directory.join("aerial")).unwrap();
    let city = "CITY 2\nn 0 0\nn 20 0\nr 0 1 8 0 1\n";
    fs::write(directory.join("city.txt"), city).unwrap();
    fs::write(
        directory.join("manifest.json"),
        r#"{"bounds_metres":[-10,-10,30,30]}"#,
    )
    .unwrap();
    fs::write(directory.join("landscape.txt"), "LANDSCAPE 1\n").unwrap();
    fs::write(directory.join("furniture.txt"), "").unwrap();
    fs::write(
        directory.join("elevation.txt"),
        format!("ELEVATION 1\ngrid 2 2 -10 -10 40 0\n{height} {height} {height} {height}\n"),
    )
    .unwrap();
    compile_collisions(
        &directory.join("streets.bin"),
        &model::source_location::parse(city).unwrap(),
        "STREETS 1 1\np 0 2 0 0 20 0\nt 1 0 -4 20 -4 20 4\nt 1 0 -4 20 4 0 4\nc 1 0 -4 20 -4 0 1\n"
            .into(),
    )
    .unwrap();
    fs::write(
        directory.join("aerial.txt"),
        "AERIAL 1\nsize 64 1\ntile -10 -10 30 30 tile.rgb 00000000\n",
    )
    .unwrap();
    fs::write(
        directory.join("aerial/tile.rgb"),
        vec![height as u8; 66 * 66 * 3],
    )
    .unwrap();
    let attribution = if provider.contains("Cyprus") {
        "AERIAL: CYPRUS DLS / 2019"
    } else {
        "AERIAL: ESRI / VANTOR / EARTHSTAR / GIS COMMUNITY"
    };
    fs::write(
        directory.join("manifest.json"),
        serde_json::json!({"aerial_attribution": attribution, "bounds_metres": [-10, -10, 30, 30]})
            .to_string(),
    )
    .unwrap();
    let assets = storage::city_assets::new(directory.to_path_buf());
    let world = world::preparation::compiled_location_load(&assets).unwrap();
    build::scene::build_scene(
        &world,
        None,
        &directory.join("decorations.pack"),
        &directory.join("scene.pack"),
        directory,
    )
    .unwrap();
    assets
}

pub fn car() -> Arc<model::car::CarSpec> {
    Arc::new(car_spec().clone())
}

/// A generated elevated loop, deliberately much larger than the compact route.
pub fn authored_id() -> world::locations::LocationId {
    static ID: OnceLock<world::locations::LocationId> = OnceLock::new();
    ID.get_or_init(|| {
        let file = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(file.path(), r#"{"version":2,"name":"Test coastal loop","scenery":"coastal","points":[[0,0,0,8],[600,50,0,8],[600,80,500,8],[0,20,500,8]]}"#).unwrap();
        world::locations::register_file(file.path()).unwrap().0
    }).clone()
}

struct CityFixture {
    id: world::locations::LocationId,
    _directory: tempfile::TempDir,
}

/// Parallel streets, raised pavements, parking, buildings and controls on a slope.
/// Keep the temporary files alive for tests that reload the location by its ID.
fn city_fixture() -> &'static CityFixture {
    static FIXTURE: OnceLock<CityFixture> = OnceLock::new();
    FIXTURE.get_or_init(|| {
        let directory = tempfile::tempdir().unwrap();
        let root = directory.path();
        let mut city = String::from("CITY 2\n");
        let mut streets = String::from("STREETS 1 21\n");
        let mut landscape = String::from("LANDSCAPE 1\n");
        let mut furniture = String::new();
        for i in 0..21 {
            let z = i * 100 - 1000;
            city.push_str(&format!("n -1000 {z}\nn 1000 {z}\nr {} {} 8 0 {}\n", i * 2, i * 2 + 1, i + 1));
            city.push_str(&format!("b {} 8 0 4 200 {} 220 {} 220 {} 200 {}\n", i + 1, z + 30, z + 30, z + 50, z + 50));
            streets.push_str(&format!("p {i} 3 -1000 {z} 0 {z} 1000 {z}\n"));
            struct Strip {
                usage: u8,
                near: i32,
                far: i32,
            }
            for strip in [
                Strip { usage: 1, near: z - 4, far: z + 4 },
                Strip { usage: 6, near: z - 10, far: z - 4 },
                Strip { usage: 6, near: z + 4, far: z + 10 },
            ] {
                let Strip { usage, near, far } = strip;
                streets.push_str(&format!("t {usage} -1000 {near} 1000 {near} 1000 {far}\nt {usage} -1000 {near} 1000 {far} -1000 {far}\n"));
            }
            streets.push_str(&format!("c 1 -1000 {} 1000 {} 0 1\nc 1 -1000 {} 1000 {} 0 -1\n", z - 4, z - 4, z + 4, z + 4));
            landscape.push_str(&format!("a 3 -100 {} 100 {} 100 {}\na 3 -100 {} 100 {} -100 {}\n", z + 30, z + 30, z + 60, z + 30, z + 60, z + 60));
            let kind = ["stop", "yield", "signal", "speed_limit"][i as usize % 4];
            furniture.push_str(&format!("{} {kind} 0 {} 1 0 30 0\n", i + 1, z + 8));
        }
        std::fs::write(root.join("city.txt"), &city).unwrap();
        std::fs::write(root.join("streets.txt"), &streets).unwrap();
        std::fs::write(root.join("landscape.txt"), landscape).unwrap();
        std::fs::write(root.join("furniture.txt"), furniture).unwrap();
        std::fs::write(root.join("manifest.json"), r#"{"bounds_metres":[-1050,-1050,1050,1050]}"#).unwrap();
        std::fs::create_dir(root.join("elevation")).unwrap();
        let mut elevation = String::from("ELEVATION 1\ngrid 22 22 -1050 -1050 100 0\n");
        for _ in 0..22 {
            for column in 0..22 {
                elevation.push_str(&format!("{} ", -21 + column * 2));
            }
            elevation.push('\n');
        }
        std::fs::write(root.join("elevation.txt"), elevation).unwrap();
        compile_collisions(&root.join("streets.bin"), &model::source_location::parse(&city).unwrap(), streets).unwrap();
        CityFixture { id: world::locations::asset(root).unwrap(), _directory: directory }
    })
}

pub fn city_id() -> world::locations::LocationId {
    city_fixture().id.clone()
}

pub fn location_ids() -> [world::locations::LocationId; 3] {
    [
        world::locations::LocationId::Compact,
        authored_id(),
        city_id(),
    ]
}

pub fn variable_width_track() -> Arc<world::locations::WorldLocation> {
    let file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(file.path(), r#"{"version":2,"name":"Variable width","scenery":"coastal","points":[[0,0,0,4],[400,0,0,12],[400,0,400,6],[0,0,400,8]]}"#).unwrap();
    world::locations::try_track(&world::locations::register_file(file.path()).unwrap().0).unwrap()
}

pub fn angle(value: f32) -> f32 {
    (value + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

pub fn compact_track() -> Arc<world::locations::WorldLocation> {
    static TRACK: OnceLock<Arc<world::locations::WorldLocation>> = OnceLock::new();
    TRACK
        .get_or_init(|| {
            world::locations::try_track(&world::locations::LocationId::Compact).unwrap()
        })
        .clone()
}

pub fn vehicle(
    position: XYZ,
    heading: f32,
    model: std::sync::Arc<model::car::CarSpec>,
) -> physics::car::Vehicle {
    physics::car::on_track(position, heading, model, compact_track())
}

pub fn partition(vertices: Vec<render::scene::Vertex>) -> Vec<render::chunks::Chunk> {
    let mut builder = render::chunks::ChunkBuilder::default();
    for triangle in vertices.as_chunks::<3>().0 {
        render::chunks::push(&mut builder, *triangle);
    }
    render::chunks::finish(builder)
}

pub struct Temp {
    pub directory: std::path::PathBuf,
    _lifetime: tempfile::TempDir,
}

pub fn new() -> Temp {
    let dir = tempfile::tempdir().unwrap();
    Temp {
        directory: dir.path().to_path_buf(),
        _lifetime: dir,
    }
}

pub fn set_osm_tags(element: &mut build::osm::OSMElement, tags: &[(&str, &str)]) {
    let mut replacement = osm_element(element.id, tags);
    replacement.coord = element.coord;
    replacement.nodes = std::mem::take(&mut element.nodes);
    replacement.members = std::mem::take(&mut element.members);
    *element = replacement;
}

pub fn osm_element(id: i64, tags: &[(&str, &str)]) -> build::osm::OSMElement {
    let file = tempfile::NamedTempFile::new().unwrap();
    let parsed_id = id.max(1);
    let mut xml = format!("<osm><node id=\"{parsed_id}\" lat=\"0\" lon=\"0\">");
    for (key, value) in tags {
        xml.push_str(&format!(
            "<tag k=\"{}\" v=\"{}\"/>",
            quick_xml::escape::escape(*key),
            quick_xml::escape::escape(*value)
        ));
    }
    xml.push_str("</node></osm>");
    std::fs::write(file.path(), xml).unwrap();
    let mut element = build::osm::read(file.path())
        .unwrap()
        .nodes
        .remove(&parsed_id)
        .unwrap();
    element.id = id;
    element.coord = None;
    element
}

pub fn city_world() -> Arc<world::locations::WorldLocation> {
    static WORLD: OnceLock<Arc<world::locations::WorldLocation>> = OnceLock::new();
    WORLD
        .get_or_init(|| world::locations::try_track(&city_id()).unwrap())
        .clone()
}

pub fn stored_location(directory: &Path, populated: bool) -> CityAsset {
    let mut source = model::source_location::empty::<f64>();
    source.origin = Some(util::geometry::new(34.7, 33.0));
    source.bounding_box = BoundingBoxXZ {
        min: XZ { x: -10.0, z: -10.0 },
        max: XZ { x: 30.0, z: 30.0 },
    };
    source.road_nodes = vec![
        util::geometry::at_height(XZ { x: 0.0, z: 0.0 }, 0.0),
        util::geometry::at_height(XZ { x: 20.0, z: 0.0 }, 0.0),
    ];
    let mut road = model::source_location::new([0, 1], 8.0);
    road.osm_way_id = Some(1);
    source.road_lines.push(road);
    if populated {
        source.buildings.push(Building {
            id: 1,
            height_meters: 5.0,
            base_y: 0.0,
            tagged: true,
            holes: Vec::new(),
            min_height_meters: 0.0,
            ground_outline: vec![
                XZ { x: 10.0, z: 10.0 },
                XZ { x: 15.0, z: 10.0 },
                XZ { x: 15.0, z: 15.0 },
                XZ { x: 10.0, z: 15.0 },
            ],
        });
    }
    let parsed = storage::city::write_imported(&source, &directory)
        .unwrap()
        .city;
    std::fs::write(
        directory.join("manifest.json"),
        serde_json::json!({
            "origin": {"latitude": 34.7, "longitude": 33.0},
            "bounds_metres": util::geometry::bounding_box_xz_to_array(parsed.bounding_box),
        })
        .to_string(),
    )
    .unwrap();
    // The bent compiled path differs from the CITY endpoints.
    compile_collisions(
            &directory.join("streets.bin"), &parsed,
            "STREETS 1 1\np 0 3 0 0 10 1 20 0\nt 1 0 -4 20 -4 20 4\nt 1 0 -4 20 4 0 4\nc 1 0 -4 20 -4 0 1\n".into(),
        ).unwrap();
    std::fs::create_dir(directory.join("elevation")).unwrap();
    std::fs::write(
        directory.join("elevation.txt"),
        "ELEVATION 1\ngrid 2 2 -10 -10 40 0\n7 7\n7 7\n",
    )
    .unwrap();
    std::fs::write(
        directory.join("landscape.txt"),
        if populated {
            "LANDSCAPE 1\na 0 -8 10 -3 10 -3 15\nt 1 -5 20 6 1\n"
        } else {
            "LANDSCAPE 1\n"
        },
    )
    .unwrap();
    std::fs::write(
        directory.join("furniture.txt"),
        if populated {
            "1 stop 5 5 1 0 0 0\n"
        } else {
            ""
        },
    )
    .unwrap();
    if populated {
        std::fs::write(
            directory.join("imagery-trees.txt"),
            "LANDSCAPE 1\nt 9223372036854775808 25 25 5 1\n",
        )
        .unwrap();
    }
    storage::city_assets::new(directory.to_path_buf())
}

/// A four-metre speed table on the straight part of the stored-location fixture.
pub fn speed_table_rises() -> Vec<model::world::RoadRise> {
    struct Section {
        x: f32,
        height: f32,
    }
    let sections = [
        Section { x: 12., height: 0. },
        Section {
            x: 13.,
            height: 0.1,
        },
        Section {
            x: 15.,
            height: 0.1,
        },
        Section { x: 16., height: 0. },
    ];
    let mut rises = Vec::new();
    for pair in sections.windows(2) {
        let a = util::geometry::at_height(
            XZ {
                x: pair[0].x,
                z: -1.,
            },
            pair[0].height,
        );
        let b = util::geometry::at_height(
            XZ {
                x: pair[0].x,
                z: 1.,
            },
            pair[0].height,
        );
        let c = util::geometry::at_height(
            XZ {
                x: pair[1].x,
                z: 1.,
            },
            pair[1].height,
        );
        let d = util::geometry::at_height(
            XZ {
                x: pair[1].x,
                z: -1.,
            },
            pair[1].height,
        );
        rises.extend([
            model::world::RoadRise { points: [a, b, c] },
            model::world::RoadRise { points: [a, c, d] },
        ]);
    }
    rises
}

pub fn bake_collisions(city: &Path, streets: &Path, output: &Path) -> util::error::Result<()> {
    let city = model::source_location::parse(&std::fs::read_to_string(city)?)?;
    compile_collisions(output, &city, std::fs::read_to_string(streets)?)
}
