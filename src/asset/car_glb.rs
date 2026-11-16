//! Static glTF 2.0 car artwork adapted to the existing colored-triangle renderer.
use crate::asset;
use crate::render::scene::Vertex;
use crate::util::geometry::XYZ;
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

/// Overall vertical extent, including wheels translated to their axle mounts.
/// CG height fractions use the lowest model point as ground and the highest as 1.
pub fn height(design: &Design) -> f32 {
    let heights = design.body.iter().map(|v| v.position[1]).chain(
        design
            .wheels
            .iter()
            .zip(design.mounts)
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
pub fn frontal_area_m2(design: &Design) -> f32 {
    const RESOLUTION: usize = 512;
    let parts = || {
        std::iter::once((&design.body, XYZ::default()))
            .chain(design.wheels.iter().zip(design.mounts))
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

fn invalid(message: impl ToString) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.to_string())
}

pub fn load(path: &Path) -> io::Result<Design> {
    std::fs::read(path)
        .and_then(|bytes| decode(&bytes))
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", path.display())))
}

fn decode(bytes: &[u8]) -> io::Result<Design> {
    asset::mesh_glb::with_scene(bytes, |scene, blob| {
        let mut importer = Importer {
            blob,
            vertices: std::array::from_fn(|_| Vec::new()),
            origins: [None; 5],
            visited: std::collections::HashSet::new(),
        };
        for node in scene.nodes() {
            import_node(&mut importer, node, Mat4::IDENTITY, None, 0)?;
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

fn import_node(
    importer: &mut Importer<'_>,
    node: gltf::Node<'_>,
    parent: Mat4,
    part: Option<usize>,
    depth: usize,
) -> io::Result<()> {
    if depth > 128 || !importer.visited.insert(node.index()) {
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
        if part.is_some() || importer.origins[index].is_some() {
            return Err(invalid("car part names must be unique and cannot nest"));
        }
        importer.origins[index] = Some(transform.transform_point3(Vec3::ZERO));
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
                Mat4::from_translation(-importer.origins[part].unwrap()) * transform
            };
            asset::mesh_glb::primitive(
                importer.blob,
                primitive,
                geometry_transform,
                &mut importer.vertices[part],
            )?;
        }
    }
    for child in node.children() {
        import_node(importer, child, transform, part, depth + 1)?;
    }
    Ok(())
}
