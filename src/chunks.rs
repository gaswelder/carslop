use crate::geometry::XYZ;
use crate::scene::Vertex;

pub const SIZE: f32 = 256.0;

pub struct Chunk {
    pub cell: [i32; 2],
    pub vertices: Vec<Vertex>,
    pub bounds: [XYZ; 2],
}

#[derive(Default)]
pub struct ChunkBuilder {
    bins: std::collections::BTreeMap<(i32, i32), Chunk>,
    spool: Option<crate::scene_archive::Spool>,
    pending: usize,
    failure: Option<std::io::Error>,
}

impl ChunkBuilder {
    pub fn spooled(directory: &std::path::Path) -> std::io::Result<Self> {
        Ok(Self {
            spool: Some(crate::scene_archive::Spool::new(directory)?),
            ..Self::default()
        })
    }
    fn flush(&mut self) -> std::io::Result<()> {
        if let Some(spool) = &mut self.spool {
            for chunk in std::mem::take(&mut self.bins).into_values() {
                spool.append(chunk)?;
            }
            self.pending = 0;
        }
        Ok(())
    }
    pub fn finish_spooled(mut self) -> std::io::Result<crate::scene_archive::Spool> {
        if let Some(error) = self.failure.take() {
            return Err(error);
        }
        self.flush()?;
        Ok(self.spool.take().expect("spooled builder"))
    }
    pub fn push(&mut self, tri: [Vertex; 3]) {
        if self.failure.is_some() {
            return;
        }
        let x = tri.iter().map(|v| v.position[0]).sum::<f32>() / 3.;
        let z = tri.iter().map(|v| v.position[2]).sum::<f32>() / 3.;
        let cell = ((x / SIZE).floor() as i32, (z / SIZE).floor() as i32);
        let chunk = self.bins.entry(cell).or_insert_with(|| Chunk {
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
                *chunk.bounds[0].component_mut(axis) =
                    chunk.bounds[0].component(axis).min(v.position[axis]);
                *chunk.bounds[1].component_mut(axis) =
                    chunk.bounds[1].component(axis).max(v.position[axis]);
            }
        }
        chunk.vertices.extend_from_slice(&tri);
        self.pending += 3;
        // Bound live geometry independently of city size (about 12 MiB of vertices).
        if self.spool.is_some() && self.pending >= 262_144 {
            if let Err(error) = self.flush() {
                self.failure = Some(error);
            }
        }
    }
    pub fn finish(self) -> Vec<Chunk> {
        assert!(self.spool.is_none());
        self.bins.into_values().collect()
    }
}
#[cfg(test)]
fn partition(vertices: Vec<Vertex>) -> Vec<Chunk> {
    assert_eq!(vertices.len() % 3, 0);
    let mut builder = ChunkBuilder::default();
    for tri in vertices.as_chunks::<3>().0 {
        builder.push(*tri);
    }
    builder.finish()
}
pub fn distance_squared(bounds: [XYZ; 2], eye: XYZ) -> f32 {
    [0, 2]
        .into_iter()
        .map(|i| {
            (eye.component(i)
                - eye
                    .component(i)
                    .clamp(bounds[0].component(i), bounds[1].component(i)))
            .powi(2)
        })
        .sum()
}
pub fn visible(bounds: [XYZ; 2], matrix: [f32; 16]) -> bool {
    let corners: [[f32; 4]; 8] = std::array::from_fn(|i| {
        let p: XYZ = XYZ::from_fn(|axis| bounds[(i >> axis) & 1].component(axis));
        std::array::from_fn(|row| {
            matrix[row] * p.x + matrix[4 + row] * p.y + matrix[8 + row] * p.z + matrix[12 + row]
        })
    });
    !(0..3).any(|axis| {
        corners.iter().all(|p| p[axis] < -p[3]) || corners.iter().all(|p| p[axis] > p[3])
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn partition_keeps_triangles_whole_and_bounds_include_vertices_across_seams() {
        let tri = |x: f32| {
            [[x - 10.0, 0.0, 0.0], [x + 10.0, 0.0, 0.0], [x, 3.0, 5.0]].map(|position| Vertex {
                position,
                color: [1.0; 3],
                normal: [0.0, 1.0, 0.0],
                material: [0.0; 2],
            })
        };
        let vertices = [tri(-256.0), tri(256.0), tri(900.0)].concat();
        let chunks = partition(vertices.clone());
        assert_eq!(
            chunks.iter().map(|c| c.vertices.len()).sum::<usize>(),
            vertices.len()
        );
        assert_eq!(chunks.len(), 3);
        for chunk in &chunks {
            assert_eq!(chunk.vertices.len() % 3, 0);
            for v in &chunk.vertices {
                for axis in 0..3 {
                    assert!(
                        v.position[axis] >= chunk.bounds[0].component(axis)
                            && v.position[axis] <= chunk.bounds[1].component(axis)
                    );
                }
            }
        }
        assert!(chunks
            .iter()
            .any(|c| c.bounds[0].x < 256.0 && c.bounds[1].x > 256.0));
    }
    #[test]
    fn culling_keeps_intersecting_bounds_and_rejects_distant_bounds() {
        let identity = [
            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        ];
        assert!(visible(
            [
                crate::geometry::XYZ {
                    x: -2.,
                    y: -2.,
                    z: -2.
                },
                crate::geometry::XYZ {
                    x: 2.,
                    y: 2.,
                    z: 2.
                }
            ],
            identity
        ));
        assert!(!visible(
            [
                crate::geometry::XYZ {
                    x: 2.,
                    y: 0.,
                    z: 0.
                },
                crate::geometry::XYZ {
                    x: 3.,
                    y: 1.,
                    z: 1.
                }
            ],
            identity
        ));
        assert_eq!(
            distance_squared(
                [
                    crate::geometry::XYZ {
                        x: 0.,
                        y: 0.,
                        z: 0.
                    },
                    crate::geometry::XYZ {
                        x: 10.,
                        y: 10.,
                        z: 10.
                    }
                ],
                crate::geometry::XYZ {
                    x: 15.,
                    y: 100.,
                    z: 5.
                }
            ),
            25.
        );
    }
}
