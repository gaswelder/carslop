//! Static city triangles partitioned without clipping or duplicating shared edges.
use crate::scene::Vertex;
pub const SIZE: f32 = 256.0;
pub struct Chunk {
    pub cell: [i32; 2],
    pub vertices: Vec<Vertex>,
    pub bounds: [[f32; 3]; 2],
}
pub fn partition(vertices: Vec<Vertex>) -> Vec<Chunk> {
    let mut bins = std::collections::BTreeMap::<(i32, i32), Chunk>::new();
    assert_eq!(vertices.len() % 3, 0);
    for tri in vertices.chunks_exact(3) {
        let x = tri.iter().map(|v| v.position[0]).sum::<f32>() / 3.0;
        let z = tri.iter().map(|v| v.position[2]).sum::<f32>() / 3.0;
        let chunk = bins
            .entry(((x / SIZE).floor() as i32, (z / SIZE).floor() as i32))
            .or_insert_with(|| Chunk {
                cell: [(x / SIZE).floor() as i32, (z / SIZE).floor() as i32],
                vertices: Vec::new(),
                bounds: [[f32::INFINITY; 3], [f32::NEG_INFINITY; 3]],
            });
        for v in tri {
            for axis in 0..3 {
                chunk.bounds[0][axis] = chunk.bounds[0][axis].min(v.position[axis]);
                chunk.bounds[1][axis] = chunk.bounds[1][axis].max(v.position[axis]);
            }
        }
        chunk.vertices.extend_from_slice(tri);
    }
    bins.into_values().collect()
}
pub fn distance_squared(bounds: [[f32; 3]; 2], eye: [f32; 3]) -> f32 {
    [0, 2]
        .into_iter()
        .map(|i| (eye[i] - eye[i].clamp(bounds[0][i], bounds[1][i])).powi(2))
        .sum()
}
pub fn visible(bounds: [[f32; 3]; 2], matrix: [f32; 16]) -> bool {
    let corners: [[f32; 4]; 8] = std::array::from_fn(|i| {
        let p: [f32; 3] = std::array::from_fn(|axis| bounds[(i >> axis) & 1][axis]);
        std::array::from_fn(|row| {
            matrix[row] * p[0] + matrix[4 + row] * p[1] + matrix[8 + row] * p[2] + matrix[12 + row]
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
                        v.position[axis] >= chunk.bounds[0][axis]
                            && v.position[axis] <= chunk.bounds[1][axis]
                    );
                }
            }
        }
        assert!(chunks
            .iter()
            .any(|c| c.bounds[0][0] < 256.0 && c.bounds[1][0] > 256.0));
    }
    #[test]
    fn culling_keeps_intersecting_bounds_and_rejects_distant_bounds() {
        let identity = [
            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        ];
        assert!(visible([[-2., -2., -2.], [2., 2., 2.]], identity));
        assert!(!visible([[2., 0., 0.], [3., 1., 1.]], identity));
        assert_eq!(distance_squared([[0.; 3], [10.; 3]], [15., 100., 5.]), 25.);
    }
}
