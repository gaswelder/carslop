//! Tests through the group's production interfaces.

mod car_glb {
    use crate::asset;
    use crate::asset::car_glb::*;
    use crate::render::scene::Vertex;
    use crate::test;
    use crate::util::geometry::XYZ;
    use glam::Vec3;
    use serde_json::{json, Value};

    fn fixture() -> (Value, Vec<u8>) {
        let floats = [0_f32, 0., 0., 1., 0., 0., 0., 1., 0.];
        let mut bin: Vec<u8> = floats.into_iter().flat_map(f32::to_le_bytes).collect();
        bin.extend([0_u16, 1, 2].into_iter().flat_map(u16::to_le_bytes));
        let mut nodes = vec![json!({"name":"body", "mesh":0})];
        for (i, name) in ["wheel_rl", "wheel_rr", "wheel_fl", "wheel_fr"]
            .iter()
            .enumerate()
        {
            nodes.push(json!({"name":name, "mesh":0, "translation":[i as f32, 2, 3]}));
        }
        (
            json!({
                "asset":{"version":"2.0"}, "scene":0,
                "scenes":[{"nodes":[0,1,2,3,4]}], "nodes":nodes,
                "buffers":[{"byteLength":bin.len()}],
                "bufferViews":[{"buffer":0,"byteLength":36}, {"buffer":0,"byteOffset":36,"byteLength":6}],
                "accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]},
                             {"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}],
                "materials":[{"pbrMetallicRoughness":{"baseColorFactor":[0.2,0.3,0.4,1],"metallicFactor":0,"roughnessFactor":0.7}}],
                "meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1,"material":0}]}]
            }),
            bin,
        )
    }
    fn pack(doc: &Value, bin: &[u8]) -> Vec<u8> {
        let mut json = serde_json::to_vec(doc).unwrap();
        while !json.len().is_multiple_of(4) {
            json.push(b' ');
        }
        let mut bin = bin.to_vec();
        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        let mut result = Vec::new();
        for n in [
            0x46546c67_u32,
            2,
            (28 + json.len() + bin.len()) as u32,
            json.len() as u32,
            0x4e4f534a,
        ] {
            result.extend(n.to_le_bytes());
        }
        result.extend(json);
        result.extend((bin.len() as u32).to_le_bytes());
        result.extend(0x004e4942_u32.to_le_bytes());
        result.extend(bin);
        result
    }

    #[test]
    fn frontal_area_counts_projected_silhouette_once_including_wheels() {
        let mut design = (*test::support::car_spec().design).clone();
        // Two triangles make a 2 x 1 rectangle in the front view.
        let vertex = |x, y, z| Vertex {
            position: [x, y, z],
            color: [0.0; 3],
            normal: [0.0, 0.0, 1.0],
            material: [0.0; 2],
        };
        design.body = vec![
            vertex(0.0, 0.0, 0.0),
            vertex(2.0, 0.0, 0.0),
            vertex(0.0, 1.0, 0.0),
            vertex(2.0, 0.0, 0.0),
            vertex(2.0, 1.0, 0.0),
            vertex(0.0, 1.0, 0.0),
        ];
        design.wheels = std::array::from_fn(|_| Vec::new());
        assert!((asset::car_glb::frontal_area_m2(&(design)) - 2.0).abs() < 0.00001);
        let duplicate: Vec<_> = design
            .body
            .iter()
            .rev()
            .copied()
            .map(|mut v| {
                v.position[2] += 3.0;
                v
            })
            .collect();
        design.body.extend(duplicate);
        assert!((asset::car_glb::frontal_area_m2(&(design)) - 2.0).abs() < 0.00001);
        design.mounts[0] = XYZ {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        design.wheels[0] = vec![
            vertex(0.0, 0.0, 0.0),
            vertex(1.0, 0.0, 0.0),
            vertex(0.0, 1.0, 0.0),
        ];
        assert!((asset::car_glb::frontal_area_m2(&(design)) - 2.5).abs() < 0.01);
        for v in &mut design.body {
            v.position[0] += 10.0;
            v.position[1] -= 5.0;
        }
        for mount in &mut design.mounts {
            mount.x += 10.0;
            mount.y -= 5.0;
        }
        assert!((asset::car_glb::frontal_area_m2(&(design)) - 2.5).abs() < 0.01);
    }

    #[test]
    fn indexed_geometry_materials_hierarchy_and_wheel_pivots() {
        let (mut doc, bin) = fixture();
        doc["nodes"]
            .as_array_mut()
            .unwrap()
            .push(json!({"translation":[5,0,0],"scale":[-2,3,1],"children":[0,1,2,3,4]}));
        doc["scenes"][0]["nodes"] = json!([5]);
        let design = load_fixture(&pack(&doc, &bin)).unwrap();
        assert_eq!(
            design.mounts[0],
            XYZ {
                x: 5.,
                y: 6.,
                z: 3.
            }
        );
        assert_eq!(
            design.mounts[3],
            XYZ {
                x: -1.,
                y: 6.,
                z: 3.
            }
        );
        assert_eq!(design.body[0].position, [5., 0., 0.]);
        assert_eq!(design.body[1].position, [5., 3., 0.]); // mirrored winding corrected
        assert_eq!(design.wheels[0][0].position, [0.; 3]);
        assert_eq!(design.wheels[0][2].position, [-2., 0., 0.]);
        assert_eq!(design.body[0].color, [0.2, 0.3, 0.4]);
        assert_eq!(design.body[0].normal, [0., 0., 1.]);
        assert!((design.body[0].material[1] - 0.3).abs() < 1e-6);
    }
    #[test]
    fn authored_normals_use_inverse_transpose_and_textures_fail_explicitly() {
        let (mut doc, mut bin) = fixture();
        // Add an aligned NORMAL accessor to the indexed triangle fixture.
        bin.extend([0, 0]);
        let start = bin.len();
        for _ in 0..3 {
            for value in [
                std::f32::consts::FRAC_1_SQRT_2,
                std::f32::consts::FRAC_1_SQRT_2,
                0.,
            ] {
                bin.extend(value.to_le_bytes());
            }
        }
        doc["buffers"][0]["byteLength"] = json!(bin.len());
        doc["bufferViews"]
            .as_array_mut()
            .unwrap()
            .push(json!({"buffer":0,"byteOffset":start,"byteLength":36}));
        doc["accessors"]
            .as_array_mut()
            .unwrap()
            .push(json!({"bufferView":2,"componentType":5126,"count":3,"type":"VEC3"}));
        doc["meshes"][0]["primitives"][0]["attributes"]["NORMAL"] = json!(2);
        doc["nodes"][0]["scale"] = json!([2, 1, 1]);
        let design = load_fixture(&pack(&doc, &bin)).unwrap();
        let expected = Vec3::new(0.5, 1., 0.).normalize();
        assert!((Vec3::from_array(design.body[0].normal) - expected).length() < 1e-6);
        doc["images"] = json!([{"uri":"texture.png"}]);
        doc["textures"] = json!([{"source":0}]);
        doc["materials"][0]["pbrMetallicRoughness"]["baseColorTexture"] = json!({"index":0});
        assert!(load_fixture(&pack(&doc, &bin))
            .err()
            .unwrap()
            .to_string()
            .contains("textures are not supported"));
    }

    #[test]
    fn rejects_corrupt_and_unsupported_assets_without_panicking() {
        let (doc, bin) = fixture();
        let valid = pack(&doc, &bin);
        assert!(load_fixture(&valid[..valid.len() - 1]).is_err());
        let mut bad_indices = bin.clone();
        bad_indices[36..38].copy_from_slice(&99_u16.to_le_bytes());
        assert!(load_fixture(&pack(&doc, &bad_indices)).is_err());
        let mut nan = bin.clone();
        nan[..4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(load_fixture(&pack(&doc, &nan)).is_err());
        for change in 0..7 {
            let mut doc = doc.clone();
            match change {
                0 => doc["nodes"][4]["name"] = json!("missing"),
                1 => doc["nodes"][4]["name"] = json!("wheel_rl"),
                2 => doc["bufferViews"][0]["byteLength"] = json!(999999),
                3 => doc["accessors"][0]["count"] = json!(999999),
                4 => doc["buffers"][0]["uri"] = json!("external.bin"),
                5 => doc["materials"][0]["alphaMode"] = json!("BLEND"),
                _ => doc["nodes"][0]["scale"] = json!([0, 1, 1]),
            }
            assert!(
                load_fixture(&pack(&doc, &bin)).is_err(),
                "accepted case {change}"
            );
        }
    }

    fn load_fixture(bytes: &[u8]) -> std::io::Result<Design> {
        let file = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(file.path(), bytes).unwrap();
        asset::car_glb::load(file.path())
    }
}
