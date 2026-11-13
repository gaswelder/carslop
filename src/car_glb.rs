//! Static glTF 2.0 car artwork adapted to the existing colored-triangle renderer.
use super::mesh_glb;
use super::Vertex;
use crate::geometry::XYZ;
use glam::{Mat4, Vec3};
use std::{io, path::Path};

const PARTS: [&str; 5] = ["body", "wheel_rl", "wheel_rr", "wheel_fl", "wheel_fr"];

#[derive(Clone, Debug)]
pub struct Design {
    pub body: Vec<Vertex>,
    pub wheels: [Vec<Vertex>; 4],
    pub mounts: [XYZ; 4],
    pub wheel_radii: [f32; 4],
}
impl Design {
    /// Overall vertical extent, including wheels translated to their axle mounts.
    /// CG height fractions use the lowest model point as ground and the highest as 1.
    pub fn height(&self) -> f32 {
        let heights = self.body.iter().map(|v| v.position[1]).chain(
            self.wheels
                .iter()
                .zip(self.mounts)
                .flat_map(|(wheel, mount)| wheel.iter().map(move |v| v.position[1] + mount.y)),
        );
        let (mut min, mut max) = (f32::INFINITY, f32::NEG_INFINITY);
        for y in heights {
            if !y.is_finite() {
                return f32::NAN;
            }
            min = min.min(y);
            max = max.max(y);
        }
        max - min
    }
    /// Estimate the front silhouette area in m² (+Z forward) on a 512×512 grid.
    /// Project both faces of every triangle and mark their union, including mounted
    /// wheels. Overlaps count once; scanline spans keep detailed models inexpensive.
    pub fn frontal_area_m2(&self) -> f32 {
        const RESOLUTION: usize = 512;
        let parts = || {
            std::iter::once((&self.body, XYZ::default())).chain(self.wheels.iter().zip(self.mounts))
        };
        let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
        let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for (vertices, offset) in parts() {
            for vertex in vertices {
                let x = (vertex.position[0] + offset.x) as f64;
                let y = (vertex.position[1] + offset.y) as f64;
                if !x.is_finite() || !y.is_finite() {
                    return f32::NAN;
                }
                min_x = min_x.min(x);
                max_x = max_x.max(x);
                min_y = min_y.min(y);
                max_y = max_y.max(y);
            }
        }
        let (width, height) = (max_x - min_x, max_y - min_y);
        if width <= 0.0 || height <= 0.0 {
            return 0.0;
        }
        let scale = RESOLUTION as f64;
        let mut covered = vec![false; RESOLUTION * RESOLUTION];
        // A cell is covered when its centre lies inside a projected triangle.
        let cell = |edge: f64| (edge - 0.5).ceil().clamp(0.0, scale) as usize;
        for (vertices, offset) in parts() {
            for triangle in vertices.chunks_exact(3) {
                let points: [(f64, f64); 3] = std::array::from_fn(|i| {
                    let p = triangle[i].position;
                    (
                        ((p[0] + offset.x) as f64 - min_x) * scale / width,
                        ((p[1] + offset.y) as f64 - min_y) * scale / height,
                    )
                });
                let bottom = points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
                let top = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
                for row in cell(bottom)..cell(top) {
                    let y = row as f64 + 0.5;
                    let (mut left, mut right) = (f64::INFINITY, f64::NEG_INFINITY);
                    for edge in 0..3 {
                        let (a, b) = (points[edge], points[(edge + 1) % 3]);
                        if (a.1 <= y && y < b.1) || (b.1 <= y && y < a.1) {
                            let x = a.0 + (y - a.1) * (b.0 - a.0) / (b.1 - a.1);
                            left = left.min(x);
                            right = right.max(x);
                        }
                    }
                    if left < right {
                        covered[row * RESOLUTION + cell(left)..row * RESOLUTION + cell(right)]
                            .fill(true);
                    }
                }
            }
        }
        (width * height * covered.iter().filter(|v| **v).count() as f64 / (scale * scale)) as f32
    }
}

fn invalid(message: impl ToString) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.to_string())
}

pub fn load(path: &Path) -> io::Result<Design> {
    std::fs::read(path)
        .and_then(|bytes| decode(&bytes))
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", path.display())))
}

fn decode(bytes: &[u8]) -> io::Result<Design> {
    mesh_glb::with_scene(bytes, |scene, blob| {
        let mut importer = Importer {
            blob,
            vertices: std::array::from_fn(|_| Vec::new()),
            origins: [None; 5],
            visited: std::collections::HashSet::new(),
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
        let mounts =
            std::array::from_fn(|i| XYZ::from(importer.origins[i + 1].unwrap().to_array()));
        let wheels = [rl, rr, fl, fr];
        let wheel_radii = std::array::from_fn(|i| {
            wheels[i]
                .iter()
                .map(|v| v.position[1].hypot(v.position[2]))
                .fold(0.0, f32::max)
        });
        Ok(Design {
            body,
            wheels,
            wheel_radii,
            mounts,
        })
    })
}

struct Importer<'a> {
    blob: &'a [u8],
    vertices: [Vec<Vertex>; 5],
    origins: [Option<Vec3>; 5],
    visited: std::collections::HashSet<usize>,
}
impl Importer<'_> {
    fn node(
        &mut self,
        node: gltf::Node<'_>,
        parent: Mat4,
        part: Option<usize>,
        depth: usize,
    ) -> io::Result<()> {
        if depth > 128 || !self.visited.insert(node.index()) {
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
                mesh_glb::primitive(
                    self.blob,
                    primitive,
                    geometry_transform,
                    &mut self.vertices[part],
                )?;
            }
        }
        for child in node.children() {
            self.node(child, transform, part, depth + 1)?;
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
    fn frontal_area_counts_projected_silhouette_once_including_wheels() {
        let mut design = (*crate::test_support::car_spec().design).clone();
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
        assert!((design.frontal_area_m2() - 2.0).abs() < 0.00001);
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
        assert!((design.frontal_area_m2() - 2.0).abs() < 0.00001);
        design.mounts[0] = crate::geometry::XYZ {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        design.wheels[0] = vec![
            vertex(0.0, 0.0, 0.0),
            vertex(1.0, 0.0, 0.0),
            vertex(0.0, 1.0, 0.0),
        ];
        assert!((design.frontal_area_m2() - 2.5).abs() < 0.01);
        for v in &mut design.body {
            v.position[0] += 10.0;
            v.position[1] -= 5.0;
        }
        for mount in &mut design.mounts {
            mount.x += 10.0;
            mount.y -= 5.0;
        }
        assert!((design.frontal_area_m2() - 2.5).abs() < 0.01);
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
        assert_eq!(
            design.mounts[0],
            crate::geometry::XYZ {
                x: 5.,
                y: 6.,
                z: 3.
            }
        );
        assert_eq!(
            design.mounts[3],
            crate::geometry::XYZ {
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
