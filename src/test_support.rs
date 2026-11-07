//! Controlled inputs for module tests. No installed car definitions or assets are read.
use crate::cars::{CarSpec, EngineVoice, Gearbox};
use std::sync::{Arc, OnceLock};

pub fn audio_voice() -> &'static EngineVoice {
    static VOICE: OnceLock<EngineVoice> = OnceLock::new();
    VOICE.get_or_init(|| EngineVoice {
        harmonics: vec![[1.0, 0.6], [2.0, 0.25], [3.0, 0.15]],
        gain: 2.0,
        coast_gain: 0.4,
        soft_limit: None,
        pulses_per_revolution: 2.0,
        samples: None,
    })
}

pub fn car_spec() -> &'static CarSpec {
    static SPEC: OnceLock<CarSpec> = OnceLock::new();
    SPEC.get_or_init(|| {
        let vertex = |position| crate::scene::Vertex {
            position,
            color: [0.5; 3],
            normal: [0.0, 1.0, 0.0],
            material: [0.0; 2],
        };
        let design = Arc::new(crate::scene::car_glb::Design {
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
                [-0.75, -0.1, -1.25],
                [0.75, -0.1, -1.25],
                [-0.75, -0.1, 1.25],
                [0.75, -0.1, 1.25],
            ],
            wheel_radii: [0.32; 4],
        });
        let frontal_area_m2 = design.frontal_area_m2();
        CarSpec {
            design,
            name: "TEST VEHICLE",
            mass: 1200.0,
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
            suspension_hz: 1.7,
            gearbox: Gearbox {
                ratios: &[3.0, 2.0, 1.4, 1.0, 0.8],
                final_drive: 4.0,
                rolling_radius: 0.32,
                reverse_ratio: 3.0,
                upshift_rpm: 5500.0,
                downshift_rpm: 2200.0,
            },
            engine_power_hp: 120.0,
            drag_coefficient: 0.3,
            frontal_area_m2,
            turbo: None,
        }
    })
}

pub fn track() -> Arc<crate::track::Track> {
    let file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(file.path(), r#"{"version":2,"name":"Module test road","scenery":"coastal","points":[[0,3,0,8],[400,6,0,8],[400,5,400,8],[0,3,400,8]]}"#).unwrap();
    crate::track::register_file(file.path()).unwrap().0.track()
}

pub fn car_definition() -> serde_json::Value {
    let s = car_spec();
    serde_json::json!({
        "name": s.name, "weight_kg": s.mass, "brake_force": s.brake_force,
        "tire_friction_coefficient": s.tire_friction_coefficient,
        "engine_power_hp": s.engine_power_hp,
        "drag_coefficient": s.drag_coefficient, "front_weight": s.front_weight,
        "cg_height_fraction": s.cg_height / s.design.height(), "yaw_inertia": s.yaw_inertia,
        "drive_bias": s.drive_bias, "brake_bias": s.brake_bias,
        "corner_stiffness": s.corner_stiffness, "engine_max_torque_nm": s.engine_max_torque_nm,
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
pub fn model_glb(design: &crate::scene::car_glb::Design) -> Vec<u8> {
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
            design.mounts[index - 1]
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

// Index zero is the standard fixture; other entries belong to individual tests.
static SPECS: std::sync::Mutex<Vec<&'static CarSpec>> = std::sync::Mutex::new(Vec::new());

pub fn register_spec(spec: CarSpec) -> usize {
    let mut specs = SPECS.lock().unwrap();
    specs.push(Box::leak(Box::new(spec)));
    specs.len()
}

pub fn fixture_spec(index: usize) -> &'static CarSpec {
    if index == 0 {
        car_spec()
    } else {
        SPECS.lock().unwrap()[index - 1]
    }
}
