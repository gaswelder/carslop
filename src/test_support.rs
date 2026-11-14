//! Shared fixture builders and controlled inputs for module tests.
use crate::asset_car;
use crate::build_collision;
use crate::build_scene;
use crate::chunks;
use crate::city_assets::CityAsset;
use crate::geometry::{BoundingBoxXZ, LatLon, XYZ, XZ};
use crate::osm;
use crate::phys_car;
use crate::scene;
use crate::source_location::{SourceLocation, SourceRoad};
use crate::src_files;
use crate::track;
use crate::world;
pub use crate::world::test_support::*;
use crate::world::Building;
use std::path::Path;
use std::sync::{Arc, OnceLock};

pub fn audio_voice() -> &'static asset_car::EngineVoice {
    static VOICE: OnceLock<asset_car::EngineVoice> = OnceLock::new();
    VOICE.get_or_init(|| asset_car::EngineVoice {
        harmonics: vec![[1.0, 0.6], [2.0, 0.25], [3.0, 0.15]],
        gain: 2.0,
        coast_gain: 0.4,
        soft_limit: None,
        pulses_per_revolution: 2.0,
        samples: None,
    })
}

pub fn car_spec() -> &'static asset_car::CarSpec {
    static SPEC: OnceLock<asset_car::CarSpec> = OnceLock::new();
    SPEC.get_or_init(|| {
        let vertex = |position| scene::Vertex {
            position,
            color: [0.5; 3],
            normal: [0.0, 1.0, 0.0],
            material: [0.0; 2],
        };
        let design = Arc::new(scene::car_glb::Design {
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
        let frontal_area_m2 = design.frontal_area_m2();
        asset_car::CarSpec {
            design,
            name: "TEST VEHICLE".into(),
            paint_color: [1.0, 0.18, 0.08],
            weight_kg: 1200.0,
            top_speed: 50.0,
            brake_force: 22000.0,
            tire_friction_coefficient: 1.2,
            wheelbase: 2.5,
            audio: audio_voice().clone(),
            rear_combined_slip: None,
            front_weight: 0.5,
            cg_height: 0.5,
            track_width: 1.5,
            yaw_inertia: 1500.0,
            drive_bias: 0.0,
            brake_bias: 0.65,
            corner_stiffness: [70000.0; 2],
            engine_max_torque_nm: 180.0,
            engine_torque_curve: vec![
                asset_car::TorquePoint {
                    rpm: 0.0,
                    torque: 0.6,
                },
                asset_car::TorquePoint {
                    rpm: 3500.0,
                    torque: 1.0,
                },
                asset_car::TorquePoint {
                    rpm: 6500.0,
                    torque: 0.9,
                },
                asset_car::TorquePoint {
                    rpm: 33500.0,
                    torque: 0.0,
                },
            ],
            redline_rpm: 6500.0,
            suspension_hz: 1.7,
            gearbox: asset_car::Gearbox {
                ratios: vec![3.0, 2.0, 1.4, 1.0, 0.8],
                final_drive: 4.0,
                rolling_radius: 0.32,
                reverse_ratio: 3.0,
                upshift_rpm: 5500.0,
                downshift_rpm: 2200.0,
            },
            engine_power_watts: 120.0 * asset_car::WATTS_PER_HP,
            drag_coefficient: 0.3,
            frontal_area_m2,
            turbo: None,
        }
    })
}

pub fn track() -> Arc<track::Location> {
    let file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(file.path(), r#"{"version":2,"name":"Module test road","scenery":"coastal","points":[[0,3,0,8],[400,6,0,8],[400,5,400,8],[0,3,400,8]]}"#).unwrap();
    track::register_file(file.path())
        .unwrap()
        .0
        .try_track()
        .unwrap()
}

pub fn car_definition() -> serde_json::Value {
    let s = car_spec();
    serde_json::json!({
        "name": s.name, "paint_color": s.paint_color,
        "weight_kg": s.weight_kg, "brake_force": s.brake_force,
        "tire_friction_coefficient": s.tire_friction_coefficient,
        "engine_power_hp": s.engine_power_watts / asset_car::WATTS_PER_HP, "redline_rpm": s.redline_rpm,
        "drag_coefficient": s.drag_coefficient, "front_weight": s.front_weight,
        "cg_height_fraction": s.cg_height / s.design.height(), "yaw_inertia": s.yaw_inertia,
        "drive_bias": s.drive_bias, "brake_bias": s.brake_bias,
        "corner_stiffness": s.corner_stiffness, "engine_max_torque_nm": s.engine_max_torque_nm,
        "engine_torque_curve": s.engine_torque_curve.iter().map(|point|
            serde_json::json!({"rpm": point.rpm, "torque": point.torque})).collect::<Vec<_>>(),
        "suspension_hz": s.suspension_hz,
        "gearbox": {"ratios": s.gearbox.ratios, "final_drive": s.gearbox.final_drive,
            "reverse_ratio": s.gearbox.reverse_ratio, "upshift_rpm": s.gearbox.upshift_rpm,
            "downshift_rpm": s.gearbox.downshift_rpm},
        "audio": {"harmonics": s.audio.harmonics, "gain": s.audio.gain,
            "coast_gain": s.audio.coast_gain, "soft_limit": s.audio.soft_limit,
            "pulses_per_revolution": s.audio.pulses_per_revolution}
    })
}

/// Minimal GLB made from controlled triangle geometry, with independently editable parts.
pub fn model_glb(design: &scene::car_glb::Design) -> Vec<u8> {
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

pub fn cars() -> Vec<std::sync::Arc<asset_car::CarSpec>> {
    asset_car::load(std::path::Path::new("data/cars")).unwrap()
}

/// Tiny generated world with a real scene archive and one raw aerial tile.
/// All fixtures use the same relative filenames to expose accidental cross-world reads.
pub fn generated_location(
    directory: &std::path::Path,
    height: f32,
    provider: &str,
) -> CityAsset {
    use std::fs;
    fs::create_dir_all(directory.join("elevation")).unwrap();
    fs::create_dir_all(directory.join("aerial")).unwrap();
    let city = "CITY 2\nbounds -10 -10 30 30\nn 0 0\nn 20 0\nr 0 1 8 0 1\n";
    fs::write(directory.join("city.txt"), city).unwrap();
    fs::write(
        directory.join("manifest.json"),
        serde_json::json!({"name": directory.file_name().unwrap().to_string_lossy()}).to_string(),
    )
    .unwrap();
    fs::write(directory.join("landscape.txt"), "LANDSCAPE 1\n").unwrap();
    fs::write(directory.join("furniture.txt"), "FURNITURE 3\n").unwrap();
    fs::write(
        directory.join("elevation/heightfield.txt"),
        format!("ELEVATION 1\ngrid 2 2 -10 -10 40 0\n{height} {height} {height} {height}\n"),
    )
    .unwrap();
    build_collision::build_collisions(
        &directory.join("streets.bin"),
        &SourceLocation::<f32>::parse(city).unwrap(),
        "STREETS 1 1\np 0 2 0 0 20 0\nt 1 0 -4 20 -4 20 4\nt 1 0 -4 20 4 0 4\nc 1 0 -4 20 -4 0 1\n"
            .into(),
    )
    .unwrap();
    fs::write(
        directory.join("aerial/tiles.txt"),
        "AERIAL 1\nsize 64 1\ntile -10 -10 30 30 tile.rgb 00000000\n",
    )
    .unwrap();
    fs::write(
        directory.join("aerial/tile.rgb"),
        vec![height as u8; 66 * 66 * 3],
    )
    .unwrap();
    fs::write(
        directory.join("aerial/source.json"),
        serde_json::json!({"source":provider, "provider":provider}).to_string(),
    )
    .unwrap();
    let assets = CityAsset::new(directory.to_path_buf());
    let world = world::CompiledLocation::load(&assets).unwrap();
    build_scene::build_scene(
        &world,
        None,
        &directory.join("decorations.pack"),
        &directory.join("scene.pack"),
        directory,
    )
    .unwrap();
    assets
}

pub fn car() -> Arc<asset_car::CarSpec> {
    Arc::new(car_spec().clone())
}

pub fn scenic_id() -> track::LocationId {
    track::register_file(std::path::Path::new(
        "data/locations/scenic/source.track.json",
    ))
    .expect("cannot load Coastal Highway track definition")
    .0
}

pub fn nicosia_id() -> track::LocationId {
    track::LocationId::asset("data/locations/nicosia").expect("bundled Nicosia assets")
}

pub fn location_ids() -> [track::LocationId; 3] {
    [track::LocationId::Compact, scenic_id(), nicosia_id()]
}

pub fn variable_width_track() -> Arc<track::Location> {
    let file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(file.path(), r#"{"version":2,"name":"Variable width","scenery":"coastal","points":[[0,0,0,4],[400,0,0,12],[400,0,400,6],[0,0,400,8]]}"#).unwrap();
    track::register_file(file.path())
        .unwrap()
        .0
        .try_track()
        .unwrap()
}

pub fn angle(value: f32) -> f32 {
    (value + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

pub fn vehicle(
    position: XYZ,
    heading: f32,
    model: std::sync::Arc<asset_car::CarSpec>,
) -> phys_car::Vehicle {
    phys_car::Vehicle::on_track(
        position,
        heading,
        model,
        track::LocationId::Compact.try_track().unwrap(),
    )
}

pub fn partition(vertices: Vec<scene::Vertex>) -> Vec<chunks::Chunk> {
    let mut builder = chunks::ChunkBuilder::default();
    for triangle in vertices.as_chunks::<3>().0 {
        builder.push(*triangle);
    }
    builder.finish()
}

pub struct Temp {
    pub directory: std::path::PathBuf,
    _lifetime: tempfile::TempDir,
}
impl Temp {
    pub fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        Self {
            directory: dir.path().to_path_buf(),
            _lifetime: dir,
        }
    }
}

pub fn set_osm_tags(element: &mut osm::OSMElement, tags: &[(&str, &str)]) {
    let mut replacement = osm_element(element.id, tags);
    replacement.coord = element.coord;
    replacement.nodes = std::mem::take(&mut element.nodes);
    replacement.members = std::mem::take(&mut element.members);
    *element = replacement;
}

pub fn osm_element(id: i64, tags: &[(&str, &str)]) -> osm::OSMElement {
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
    let mut element = osm::read(file.path())
        .unwrap()
        .nodes
        .remove(&parsed_id)
        .unwrap();
    element.id = id;
    element.coord = None;
    element
}

pub fn nicosia_world() -> Arc<track::Location> {
    static WORLD: OnceLock<Arc<track::Location>> = OnceLock::new();
    WORLD
        .get_or_init(|| nicosia_id().try_track().unwrap())
        .clone()
}

pub fn stored_location(directory: &Path, populated: bool) -> CityAsset {
    let mut source = SourceLocation::<f64>::empty();
    source.origin = Some(LatLon::new(34.7, 33.0));
    source.bounding_box = BoundingBoxXZ {
        min: XZ { x: -10.0, z: -10.0 },
        max: XZ { x: 30.0, z: 30.0 },
    };
    source.road_nodes = vec![
        XZ { x: 0.0, z: 0.0 }.at_height(0.0),
        XZ { x: 20.0, z: 0.0 }.at_height(0.0),
    ];
    let mut road = SourceRoad::new([0, 1], 8.0);
    road.osm_way_id = Some(1);
    source.road_lines.push(road);
    if populated {
        source.buildings.push(Building {
            id: 1,
            height_meters: 5.0,
            base_y: 0.0,
            tagged: true,
            ground_outline: vec![
                XZ { x: 10.0, z: 10.0 },
                XZ { x: 15.0, z: 10.0 },
                XZ { x: 15.0, z: 15.0 },
                XZ { x: 10.0, z: 15.0 },
            ],
        });
    }
    let parsed = src_files::write_city_txt(&source, &directory.join("city.txt"))
        .unwrap()
        .city;
    std::fs::write(directory.join("manifest.json"), r#"{"name":"Fixture"}"#).unwrap();
    // The bent compiled path differs from the CITY endpoints.
    build_collision::build_collisions(
            &directory.join("streets.bin"), &parsed,
            "STREETS 1 1\np 0 3 0 0 10 1 20 0\nt 1 0 -4 20 -4 20 4\nt 1 0 -4 20 4 0 4\nc 1 0 -4 20 -4 0 1\n".into(),
        ).unwrap();
    std::fs::create_dir(directory.join("elevation")).unwrap();
    std::fs::write(
        directory.join("elevation/heightfield.txt"),
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
            "FURNITURE 3\n1 stop 5 5 1 0 0 0\n"
        } else {
            "FURNITURE 3\n"
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
    CityAsset::new(directory.to_path_buf())
}

/// A four-metre speed table on the straight part of the stored-location fixture.
pub fn speed_table_rises() -> Vec<world::RoadRise> {
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
        let a = XZ {
            x: pair[0].x,
            z: -1.,
        }
        .at_height(pair[0].height);
        let b = XZ {
            x: pair[0].x,
            z: 1.,
        }
        .at_height(pair[0].height);
        let c = XZ {
            x: pair[1].x,
            z: 1.,
        }
        .at_height(pair[1].height);
        let d = XZ {
            x: pair[1].x,
            z: -1.,
        }
        .at_height(pair[1].height);
        rises.extend([
            world::RoadRise { points: [a, b, c] },
            world::RoadRise { points: [a, c, d] },
        ]);
    }
    rises
}
