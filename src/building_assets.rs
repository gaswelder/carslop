//! File-backed architectural parts, loaded only while baking location geometry.
use super::{mesh_glb, Builder};
use crate::geometry::{XYZ, XZ};
use glam::{Mat3, Mat4, Vec3, Vec4};
use serde_json::Value;
use std::{io, path::Path};

fn invalid(message: impl ToString) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.to_string())
}
fn positive(value: &Value, name: &str) -> io::Result<f32> {
    let number = value[name]
        .as_f64()
        .ok_or_else(|| invalid(format!("missing number {name}")))? as f32;
    if !number.is_finite() || number <= 0.0 {
        return Err(invalid(format!("{name} must be finite and positive")));
    }
    Ok(number)
}
fn range(value: &Value, name: &str) -> io::Result<[f32; 2]> {
    let values: [f32; 2] = serde_json::from_value(value[name].clone()).map_err(invalid)?;
    if values.iter().any(|x| !x.is_finite() || *x <= 0.0) || values[1] < values[0] {
        return Err(invalid(format!("invalid {name} range")));
    }
    Ok(values)
}

pub(super) struct Part {
    meshes: Vec<mesh_glb::Mesh>,
    pub size: XYZ,
    scale_axes: [bool; 3],
}
impl Part {
    fn load(directory: &Path, definition: &Value) -> io::Result<Self> {
        let filename = definition["model"]
            .as_str()
            .ok_or_else(|| invalid("missing model filename"))?;
        let path = Path::new(filename);
        if path.components().count() != 1
            || !matches!(
                path.components().next(),
                Some(std::path::Component::Normal(_))
            )
            || path.extension().is_none_or(|s| s != "glb")
        {
            return Err(invalid(
                "model must name a GLB in the building asset directory",
            ));
        }
        let size: [f32; 3] = serde_json::from_value(definition["size"].clone()).map_err(invalid)?;
        if size.iter().any(|x| !x.is_finite() || *x <= 0.0) {
            return Err(invalid("model size must be finite and positive"));
        }
        let scale_axes =
            serde_json::from_value(definition["scale_axes"].clone()).map_err(invalid)?;
        Ok(Self {
            meshes: mesh_glb::load(&directory.join(path))?,
            size: (size).into(),
            scale_axes,
        })
    }

    /// Place a model centred along a facade edge. X follows the edge, Y is up,
    /// and Z projects out of the wall. Disallowed scale axes retain native size.
    pub fn place(
        &self,
        b: &mut Builder,
        edge: [XZ<f32>; 2],
        center: f32,
        y: f32,
        requested: XYZ,
        tints: &Tints,
    ) {
        let [a, c] = edge;
        let length = (c.x - a.x).hypot(c.z - a.z);
        let tangent = Vec3::new((c.x - a.x) / length, 0.0, (c.z - a.z) / length);
        let outward = Vec3::new(tangent.z, 0.0, -tangent.x);
        let scale = Vec3::from_array(std::array::from_fn(|i| {
            if self.scale_axes[i] {
                requested.component(i) / self.size.component(i)
            } else {
                1.0
            }
        }));
        let origin = Vec3::new(a.x, y, a.z) + tangent * (center - self.size.x * scale.x * 0.5);
        let transform = Mat4::from_cols(
            (tangent * scale.x).extend(0.0),
            (Vec3::Y * scale.y).extend(0.0),
            (outward * scale.z).extend(0.0),
            Vec4::from((origin, 1.0)),
        );
        let normal_matrix = Mat3::from_mat4(transform).inverse().transpose();
        let mirrored = transform.determinant() < 0.0;
        for mesh in &self.meshes {
            let tint = match mesh.name.as_str() {
                "wall" => tints.wall,
                "frame" => tints.frame,
                "glass" => tints.glass,
                "trim" => tints.trim,
                "shutter" => tints.shutter,
                _ => [1.0; 3],
            };
            for tri in mesh.vertices.chunks_exact(3) {
                let mut vertices: [_; 3] = std::array::from_fn(|i| {
                    let mut v = tri[i];
                    v.position = transform
                        .transform_point3(Vec3::from_array(v.position))
                        .to_array();
                    v.normal = (normal_matrix * Vec3::from_array(v.normal))
                        .normalize()
                        .to_array();
                    v.color = std::array::from_fn(|j| v.color[j] * tint[j]);
                    v
                });
                if mirrored {
                    vertices.swap(1, 2);
                }
                if let Some(chunks) = &mut b.chunks {
                    chunks.push(vertices);
                } else {
                    b.vertices.extend(vertices);
                }
            }
        }
    }
}

pub(super) struct Tints {
    pub wall: [f32; 3],
    pub frame: [f32; 3],
    pub glass: [f32; 3],
    pub trim: [f32; 3],
    pub shutter: [f32; 3],
}
pub(super) struct Placement {
    pub window_spacing: [f32; 2],
    pub modern_window_spacing: f32,
    pub entrance_min_wall: f32,
    pub entrance_min_height: f32,
    pub balcony_min_height: f32,
    pub balcony_min_wall: f32,
    pub balcony_max_walls: usize,
    pub balcony_width_ratio: f32,
    pub balcony_max_width: f32,
    pub balcony_depth: [f32; 2],
    pub balcony_floor_offset: f32,
    pub balcony_headroom: f32,
}
pub(super) struct Catalog {
    pub window: Part,
    pub entrance: Part,
    pub balconies: [Part; 2],
    pub placement: Placement,
}
impl Catalog {
    pub fn load_default() -> io::Result<Self> {
        Self::load(Path::new("data/buildings"))
    }
    fn load(directory: &Path) -> io::Result<Self> {
        let path = directory.join("parts.json");
        Self::decode(directory, &std::fs::read(&path)?)
            .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", path.display())))
    }
    fn decode(directory: &Path, bytes: &[u8]) -> io::Result<Self> {
        let definition: Value = serde_json::from_slice(bytes).map_err(invalid)?;
        if definition["version"].as_u64() != Some(1) {
            return Err(invalid("unsupported building parts version"));
        }
        let p = &definition["placement"];
        let max_walls = p["balcony_max_walls"]
            .as_u64()
            .filter(|n| (1..=128).contains(n))
            .ok_or_else(|| invalid("balcony_max_walls must be between 1 and 128"))?
            as usize;
        let width_ratio = positive(p, "balcony_width_ratio")?;
        if width_ratio > 1.0 {
            return Err(invalid("balcony_width_ratio must not exceed 1"));
        }
        Ok(Self {
            window: Part::load(directory, &definition["window"])?,
            entrance: Part::load(directory, &definition["entrance"])?,
            balconies: [
                Part::load(directory, &definition["balconies"][0])?,
                Part::load(directory, &definition["balconies"][1])?,
            ],
            placement: Placement {
                window_spacing: range(p, "window_spacing")?,
                modern_window_spacing: positive(p, "modern_window_spacing")?,
                entrance_min_wall: positive(p, "entrance_min_wall")?,
                entrance_min_height: positive(p, "entrance_min_height")?,
                balcony_min_height: positive(p, "balcony_min_height")?,
                balcony_min_wall: positive(p, "balcony_min_wall")?,
                balcony_max_walls: max_walls,
                balcony_width_ratio: width_ratio,
                balcony_max_width: positive(p, "balcony_max_width")?,
                balcony_depth: range(p, "balcony_depth")?,
                balcony_floor_offset: positive(p, "balcony_floor_offset")?,
                balcony_headroom: positive(p, "balcony_headroom")?,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn tints() -> Tints {
        Tints {
            wall: [0.95; 3],
            frame: [0.4; 3],
            glass: [0.2, 0.3, 0.4],
            trim: [0.9; 3],
            shutter: [0.3; 3],
        }
    }
    #[test]
    fn model_placement_preserves_normals_winding_tints_and_fixed_scale_axes() {
        let catalog = Catalog::load_default().unwrap();
        let mut b = Builder::default();
        // This facade basis reflects the model; nonuniform scale and rotated
        // placement must preserve outward normals and triangle winding.
        catalog.window.place(
            &mut b,
            [
                XZ { x: 10., z: 20. },
                XZ { x: 10., z: 30. },
            ],
            5.,
            7.,
            XYZ {
                x: 2.,
                y: 3.,
                z: 100.,
            },
            &tints(),
        );
        assert_eq!(b.vertices.len(), 24);
        for tri in b.vertices.chunks_exact(3) {
            let p: [Vec3; 3] = std::array::from_fn(|i| Vec3::from_array(tri[i].position));
            let face = (p[1] - p[0]).cross(p[2] - p[0]).normalize();
            for v in tri {
                let n = Vec3::from_array(v.normal);
                assert!((n.length() - 1.).abs() < 0.00001);
                assert!(n.dot(face) > 0.9999);
                assert!(
                    (10.02..10.06).contains(&v.position[0]),
                    "depth must remain unscaled"
                );
                assert!((7.0..=10.0).contains(&v.position[1]));
                assert!((24.0..=26.0).contains(&v.position[2]));
            }
        }
        assert!(b
            .vertices
            .iter()
            .any(|v| v.color == tints().glass && v.material == [0.0, 0.45]));
        let mut b = Builder::default();
        catalog.entrance.place(
            &mut b,
            [
                XZ { x: 0., z: 0. },
                XZ { x: 10., z: 0. },
            ],
            5.,
            0.,
            XYZ {
                x: 100.,
                y: 100.,
                z: 100.,
            },
            &tints(),
        );
        let width = b
            .vertices
            .iter()
            .map(|v| v.position[0])
            .fold(f32::NEG_INFINITY, f32::max)
            - b.vertices
                .iter()
                .map(|v| v.position[0])
                .fold(f32::INFINITY, f32::min);
        let height = b.vertices.iter().map(|v| v.position[1]).fold(0., f32::max);
        assert!((width - 1.24).abs() < 0.00001);
        assert_eq!(height, 2.25);
    }
}
