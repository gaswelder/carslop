//! Shared static GLB validation and triangle decoding for cars and scenery.
use super::Vertex;
use glam::{Mat3, Mat4, Vec3};
use std::{io, path::Path};

fn invalid(message: impl ToString) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.to_string())
}

pub(super) fn with_scene<T>(
    bytes: &[u8],
    visit: impl FnOnce(gltf::Scene<'_>, &[u8]) -> io::Result<T>,
) -> io::Result<T> {
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
    visit(scene, blob)
}

pub(super) struct Mesh {
    pub name: String,
    pub vertices: Vec<Vertex>,
}

/// Load static mesh nodes in scene coordinates, retaining names for tint roles.
pub(super) fn load(path: &Path) -> io::Result<Vec<Mesh>> {
    std::fs::read(path)
        .and_then(|bytes| {
            with_scene(&bytes, |scene, blob| {
                let mut meshes = Vec::new();
                let mut visited = std::collections::HashSet::new();
                for node in scene.nodes() {
                    node_meshes(node, Mat4::IDENTITY, blob, &mut visited, 0, &mut meshes)?;
                }
                if meshes.is_empty() {
                    return Err(invalid("model contains no mesh geometry"));
                }
                Ok(meshes)
            })
        })
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", path.display())))
}

fn node_meshes(
    node: gltf::Node<'_>,
    parent: Mat4,
    blob: &[u8],
    visited: &mut std::collections::HashSet<usize>,
    depth: usize,
    meshes: &mut Vec<Mesh>,
) -> io::Result<()> {
    if depth > 128 || !visited.insert(node.index()) {
        return Err(invalid("scene must be a tree with depth at most 128"));
    }
    let transform = parent * Mat4::from_cols_array_2d(&node.transform().matrix());
    if !transform.is_finite() || transform.determinant().abs() < 1e-12 {
        return Err(invalid("nonfinite or singular node transform"));
    }
    if node.skin().is_some() {
        return Err(invalid("skinned nodes are unsupported"));
    }
    if let Some(mesh) = node.mesh() {
        let mut vertices = Vec::new();
        for p in mesh.primitives() {
            primitive(blob, p, transform, &mut vertices)?;
        }
        if !vertices.is_empty() {
            meshes.push(Mesh {
                name: node.name().unwrap_or("").to_owned(),
                vertices,
            });
        }
    }
    for child in node.children() {
        node_meshes(child, transform, blob, visited, depth + 1, meshes)?;
    }
    Ok(())
}

pub(super) fn primitive(
    blob: &[u8],
    primitive: gltf::Primitive<'_>,
    transform: Mat4,
    vertices: &mut Vec<Vertex>,
) -> io::Result<()> {
    if primitive.mode() != gltf::mesh::Mode::Triangles || primitive.morph_targets().next().is_some()
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
            "textures are not supported yet; export vertex colors or solid materials",
        ));
    }
    if material.alpha_mode() != gltf::material::AlphaMode::Opaque {
        return Err(invalid(
            "transparent materials are unsupported; use opaque glass with carslop material extras",
        ));
    }
    let reader = primitive.reader(|_| Some(blob));
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
            || !(0.0..=6.0).contains(&values[0])
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
        let points = triangle.map(|i| transform.transform_point3(Vec3::from_array(positions[i])));
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
            if color.into_iter().chain(base).any(|v| !v.is_finite()) || color[3] * base[3] < 1.0 {
                return Err(invalid("vertex and base colors must be opaque"));
            }
            vertices.push(vertex);
        }
    }
    Ok(())
}
