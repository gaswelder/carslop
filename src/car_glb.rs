//! Static glTF 2.0 car artwork adapted to the existing colored-triangle renderer.
use super::Vertex;
use glam::{Mat3, Mat4, Vec3};
use std::{io, path::Path};

const PARTS: [&str; 5] = ["body", "wheel_rl", "wheel_rr", "wheel_fl", "wheel_fr"];

#[derive(Clone)]
pub(crate) struct Design {
    pub body: Vec<Vertex>,
    pub wheels: [Vec<Vertex>; 4],
    pub mounts: [[f32; 3]; 4],
}
fn invalid(message: impl ToString) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.to_string())
}

pub(crate) fn load(path: &Path) -> io::Result<Design> {
    std::fs::read(path)
        .and_then(|bytes| decode(&bytes))
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", path.display())))
}

fn decode(bytes: &[u8]) -> io::Result<Design> {
    if !bytes.starts_with(b"glTF") {
        return Err(invalid("expected a binary glTF 2.0 (.glb) file"));
    }
    let asset = gltf::Gltf::from_slice(bytes).map_err(invalid)?;
    let blob = asset
        .blob
        .as_deref()
        .ok_or_else(|| invalid("missing GLB binary buffer"))?;
    if asset.buffers().count() != 1
        || asset
            .buffers()
            .any(|b| !matches!(b.source(), gltf::buffer::Source::Bin) || b.length() > blob.len())
    {
        return Err(invalid(
            "use one embedded GLB buffer; external buffers are unsupported",
        ));
    }
    if asset.extensions_required().next().is_some()
        || asset.animations().next().is_some()
        || asset.skins().next().is_some()
    {
        return Err(invalid(
            "required extensions, animations and skinning are unsupported; export static meshes",
        ));
    }
    // Validate storage ranges before using gltf's zero-copy accessor readers.
    for view in asset.views() {
        if view
            .offset()
            .checked_add(view.length())
            .is_none_or(|end| end > blob.len() || end > view.buffer().length())
        {
            return Err(invalid("buffer view exceeds embedded buffer"));
        }
    }
    for accessor in asset.accessors() {
        if accessor.sparse().is_some() {
            return Err(invalid("sparse accessors are unsupported"));
        }
        let view = accessor
            .view()
            .ok_or_else(|| invalid("accessor has no buffer view"))?;
        let stride = view.stride().unwrap_or(accessor.size());
        let end = accessor
            .count()
            .checked_sub(1)
            .and_then(|n| n.checked_mul(stride))
            .and_then(|n| n.checked_add(accessor.size()))
            .and_then(|n| n.checked_add(accessor.offset()));
        if stride < accessor.size() || end.is_none_or(|end| end > view.length()) {
            return Err(invalid("accessor exceeds buffer view"));
        }
    }
    let scene = asset
        .default_scene()
        .or_else(|| asset.scenes().next())
        .ok_or_else(|| invalid("missing scene"))?;
    let mut importer = Importer {
        blob,
        vertices: std::array::from_fn(|_| Vec::new()),
        origins: [None; 5],
        visited: vec![false; asset.nodes().count()],
    };
    for node in scene.nodes() {
        importer.node(node, Mat4::IDENTITY, None, 0)?;
    }
    for (i, part) in PARTS.iter().enumerate() {
        if importer.origins[i].is_none() || importer.vertices[i].is_empty() {
            return Err(invalid(format!("missing or empty named node {part}")));
        }
    }
    let [body, rl, rr, fl, fr] = importer.vertices;
    let mounts = std::array::from_fn(|i| importer.origins[i + 1].unwrap().to_array());
    Ok(Design {
        body,
        wheels: [rl, rr, fl, fr],
        mounts,
    })
}

struct Importer<'a> {
    blob: &'a [u8],
    vertices: [Vec<Vertex>; 5],
    origins: [Option<Vec3>; 5],
    visited: Vec<bool>,
}
impl Importer<'_> {
    fn node(
        &mut self,
        node: gltf::Node<'_>,
        parent: Mat4,
        part: Option<usize>,
        depth: usize,
    ) -> io::Result<()> {
        if depth > 128 || std::mem::replace(&mut self.visited[node.index()], true) {
            return Err(invalid("scene must be a tree with depth at most 128"));
        }
        let transform = parent * Mat4::from_cols_array_2d(&node.transform().matrix());
        if !transform.is_finite() || transform.determinant().abs() < 1e-12 {
            return Err(invalid("nonfinite or singular node transform"));
        }
        let part = if let Some(index) = node
            .name()
            .and_then(|name| PARTS.iter().position(|p| *p == name))
        {
            if part.is_some() || self.origins[index].is_some() {
                return Err(invalid("car part names must be unique and cannot nest"));
            }
            self.origins[index] = Some(transform.transform_point3(Vec3::ZERO));
            Some(index)
        } else {
            part
        };
        if node.skin().is_some() {
            return Err(invalid("skinned car nodes are unsupported"));
        }
        if let Some(mesh) = node.mesh() {
            let part = part.ok_or_else(|| invalid("all meshes must belong to a named car part"))?;
            for primitive in mesh.primitives() {
                // Keep wheel geometry local to its pivot before transforming points.
                // This also preserves identical shared meshes without translation roundoff.
                let geometry_transform = if part == 0 {
                    transform
                } else {
                    Mat4::from_translation(-self.origins[part].unwrap()) * transform
                };
                self.primitive(primitive, geometry_transform, part)?;
            }
        }
        for child in node.children() {
            self.node(child, transform, part, depth + 1)?;
        }
        Ok(())
    }

    fn primitive(
        &mut self,
        primitive: gltf::Primitive<'_>,
        transform: Mat4,
        part: usize,
    ) -> io::Result<()> {
        if primitive.mode() != gltf::mesh::Mode::Triangles
            || primitive.morph_targets().next().is_some()
        {
            return Err(invalid("export static triangle primitives"));
        }
        let material = primitive.material();
        let pbr = material.pbr_metallic_roughness();
        if pbr.base_color_texture().is_some()
            || pbr.metallic_roughness_texture().is_some()
            || material.normal_texture().is_some()
            || material.emissive_texture().is_some()
            || material.occlusion_texture().is_some()
        {
            return Err(invalid(
                "car textures are not supported yet; export vertex colors or solid materials",
            ));
        }
        if material.alpha_mode() != gltf::material::AlphaMode::Opaque {
            return Err(invalid("transparent materials are unsupported; use opaque glass with carslop material extras"));
        }
        let reader = primitive.reader(|_| Some(self.blob));
        let positions: Vec<_> = reader
            .read_positions()
            .ok_or_else(|| invalid("missing POSITION"))?
            .collect();
        let normals = reader.read_normals().map(|v| v.collect::<Vec<_>>());
        let colors = reader
            .read_colors(0)
            .map(|v| v.into_rgba_f32().collect::<Vec<_>>());
        if positions.is_empty()
            || normals.as_ref().is_some_and(|v| v.len() != positions.len())
            || colors.as_ref().is_some_and(|v| v.len() != positions.len())
        {
            return Err(invalid("inconsistent vertex attribute counts"));
        }
        let indices: Vec<usize> = reader
            .read_indices()
            .map(|v| v.into_u32().map(|i| i as usize).collect())
            .unwrap_or_else(|| (0..positions.len()).collect());
        if indices.is_empty()
            || !indices.len().is_multiple_of(3)
            || indices.iter().any(|i| *i >= positions.len())
        {
            return Err(invalid("invalid triangle indices"));
        }
        let extras: serde_json::Value = material
            .extras()
            .as_ref()
            .map(|v| serde_json::from_str(v.get()))
            .transpose()
            .map_err(invalid)?
            .unwrap_or_default();
        let finish = if let Some(v) = extras.get("carslop_material") {
            let values: [f32; 2] = serde_json::from_value(v.clone()).map_err(invalid)?;
            if !values[0].is_finite()
                || !values[1].is_finite()
                || !(0.0..=5.0).contains(&values[0])
                || values[0].fract() != 0.0
                || !(0.0..=1.0).contains(&values[1])
            {
                return Err(invalid("invalid carslop_material extras"));
            }
            values
        } else {
            [
                if pbr.metallic_factor() >= 0.5 {
                    3.0
                } else {
                    0.0
                },
                1.0 - pbr.roughness_factor(),
            ]
        };
        let base = pbr.base_color_factor();
        let normal_matrix = Mat3::from_mat4(transform).inverse().transpose();
        for triangle in indices.chunks_exact(3) {
            let mut triangle = [triangle[0], triangle[1], triangle[2]];
            if transform.determinant() < 0.0 {
                triangle.swap(1, 2);
            }
            let points =
                triangle.map(|i| transform.transform_point3(Vec3::from_array(positions[i])));
            let face = (points[1] - points[0])
                .cross(points[2] - points[0])
                .normalize_or_zero();
            for (corner, i) in triangle.into_iter().enumerate() {
                let normal = normals
                    .as_ref()
                    .map(|v| (normal_matrix * Vec3::from_array(v[i])).normalize_or_zero())
                    .unwrap_or(face);
                let color = colors.as_ref().map(|v| v[i]).unwrap_or([1.0; 4]);
                let vertex = Vertex {
                    position: points[corner].to_array(),
                    normal: normal.to_array(),
                    color: std::array::from_fn(|j| color[j] * base[j]),
                    material: finish,
                };
                if vertex
                    .position
                    .into_iter()
                    .chain(vertex.normal)
                    .chain(vertex.color)
                    .any(|v| !v.is_finite())
                    || normal.length_squared() < 0.5
                {
                    return Err(invalid("nonfinite geometry or zero-length triangle normal"));
                }
                if color.into_iter().chain(base).any(|v| !v.is_finite()) || color[3] * base[3] < 1.0
                {
                    return Err(invalid("vertex and base colors must be opaque"));
                }
                self.vertices[part].push(vertex);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    fn fixture() -> (Value, Vec<u8>) {
        let floats = [0_f32, 0., 0., 1., 0., 0., 0., 1., 0.];
        let mut bin: Vec<u8> = floats.into_iter().flat_map(f32::to_le_bytes).collect();
        bin.extend([0_u16, 1, 2].into_iter().flat_map(u16::to_le_bytes));
        let mut nodes = vec![json!({"name":"body", "mesh":0})];
        for (i, name) in PARTS[1..].iter().enumerate() {
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
    fn indexed_geometry_materials_hierarchy_and_wheel_pivots() {
        let (mut doc, bin) = fixture();
        doc["nodes"]
            .as_array_mut()
            .unwrap()
            .push(json!({"translation":[5,0,0],"scale":[-2,3,1],"children":[0,1,2,3,4]}));
        doc["scenes"][0]["nodes"] = json!([5]);
        let design = decode(&pack(&doc, &bin)).unwrap();
        assert_eq!(design.mounts[0], [5., 6., 3.]);
        assert_eq!(design.mounts[3], [-1., 6., 3.]);
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
        let design = decode(&pack(&doc, &bin)).unwrap();
        let expected = Vec3::new(0.5, 1., 0.).normalize();
        assert!((Vec3::from_array(design.body[0].normal) - expected).length() < 1e-6);
        doc["images"] = json!([{"uri":"texture.png"}]);
        doc["textures"] = json!([{"source":0}]);
        doc["materials"][0]["pbrMetallicRoughness"]["baseColorTexture"] = json!({"index":0});
        assert!(decode(&pack(&doc, &bin))
            .err()
            .unwrap()
            .to_string()
            .contains("textures are not supported"));
    }

    #[test]
    fn rejects_corrupt_and_unsupported_assets_without_panicking() {
        let (doc, bin) = fixture();
        let valid = pack(&doc, &bin);
        assert!(decode(&valid[..valid.len() - 1]).is_err());
        let mut bad_indices = bin.clone();
        bad_indices[36..38].copy_from_slice(&99_u16.to_le_bytes());
        assert!(decode(&pack(&doc, &bad_indices)).is_err());
        let mut nan = bin.clone();
        nan[..4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(decode(&pack(&doc, &nan)).is_err());
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
            assert!(decode(&pack(&doc, &bin)).is_err(), "accepted case {change}");
        }
    }
}
