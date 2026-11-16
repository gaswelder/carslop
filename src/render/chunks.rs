use crate::render;
use crate::render::scene::Vertex;
use crate::util;
use crate::util::geometry::XYZ;

pub const SIZE: f32 = 256.0;

pub struct Chunk {
    pub cell: [i32; 2],
    pub vertices: Vec<Vertex>,
    pub bounds: [XYZ; 2],
}

#[derive(Default)]
pub struct ChunkBuilder {
    bins: std::collections::BTreeMap<(i32, i32), Chunk>,
    spool: Option<render::scene_archive::Spool>,
    pending: usize,
    failure: Option<std::io::Error>,
}

pub fn spooled(directory: &std::path::Path) -> std::io::Result<ChunkBuilder> {
    Ok(ChunkBuilder {
        spool: Some(render::scene_archive::new(directory)?),
        ..ChunkBuilder::default()
    })
}
fn flush(chunk_builder: &mut ChunkBuilder) -> std::io::Result<()> {
    if let Some(spool) = &mut chunk_builder.spool {
        for chunk in std::mem::take(&mut chunk_builder.bins).into_values() {
            render::scene_archive::append(spool, chunk)?;
        }
        chunk_builder.pending = 0;
    }
    Ok(())
}
pub fn finish_spooled(
    mut chunk_builder: ChunkBuilder,
) -> std::io::Result<render::scene_archive::Spool> {
    if let Some(error) = chunk_builder.failure.take() {
        return Err(error);
    }
    flush(&mut chunk_builder)?;
    Ok(chunk_builder.spool.take().expect("spooled builder"))
}
pub fn push(chunk_builder: &mut ChunkBuilder, tri: [Vertex; 3]) {
    if chunk_builder.failure.is_some() {
        return;
    }
    let x = tri.iter().map(|v| v.position[0]).sum::<f32>() / 3.;
    let z = tri.iter().map(|v| v.position[2]).sum::<f32>() / 3.;
    let cell = ((x / SIZE).floor() as i32, (z / SIZE).floor() as i32);
    let chunk = chunk_builder.bins.entry(cell).or_insert_with(|| Chunk {
        cell: [cell.0, cell.1],
        vertices: vec![],
        bounds: [
            XYZ {
                x: f32::INFINITY,
                y: f32::INFINITY,
                z: f32::INFINITY,
            },
            XYZ {
                x: f32::NEG_INFINITY,
                y: f32::NEG_INFINITY,
                z: f32::NEG_INFINITY,
            },
        ],
    });
    for v in &tri {
        for axis in 0..3 {
            *util::geometry::xyz_component_mut(&mut chunk.bounds[0], axis) =
                util::geometry::xyz_component(chunk.bounds[0], axis).min(v.position[axis]);
            *util::geometry::xyz_component_mut(&mut chunk.bounds[1], axis) =
                util::geometry::xyz_component(chunk.bounds[1], axis).max(v.position[axis]);
        }
    }
    chunk.vertices.extend_from_slice(&tri);
    chunk_builder.pending += 3;
    // Bound live geometry independently of city size (about 12 MiB of vertices).
    if chunk_builder.spool.is_some() && chunk_builder.pending >= 262_144 {
        if let Err(error) = flush(chunk_builder) {
            chunk_builder.failure = Some(error);
        }
    }
}
pub fn finish(chunk_builder: ChunkBuilder) -> Vec<Chunk> {
    assert!(chunk_builder.spool.is_none());
    chunk_builder.bins.into_values().collect()
}

pub fn distance_squared(bounds: [XYZ; 2], eye: XYZ) -> f32 {
    [0, 2]
        .into_iter()
        .map(|i| {
            (util::geometry::xyz_component(eye, i)
                - util::geometry::xyz_component(eye, i).clamp(
                    util::geometry::xyz_component(bounds[0], i),
                    util::geometry::xyz_component(bounds[1], i),
                ))
            .powi(2)
        })
        .sum()
}
pub fn visible(bounds: [XYZ; 2], matrix: [f32; 16]) -> bool {
    let corners: [[f32; 4]; 8] = std::array::from_fn(|i| {
        let p: XYZ = util::geometry::xyz_from_fn(|axis| {
            util::geometry::xyz_component(bounds[(i >> axis) & 1], axis)
        });
        std::array::from_fn(|row| {
            matrix[row] * p.x + matrix[4 + row] * p.y + matrix[8 + row] * p.z + matrix[12 + row]
        })
    });
    !(0..3).any(|axis| {
        corners.iter().all(|p| p[axis] < -p[3]) || corners.iter().all(|p| p[axis] > p[3])
    })
}
