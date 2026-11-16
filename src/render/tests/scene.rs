use crate::render;
use crate::test;
use crate::world;

#[test]
fn rendered_road_edges_follow_the_local_width() {
    let track = test::support::variable_width_track();
    let mut builder = super::Builder::default();
    super::road(&mut builder, &track);
    let mut widths = [f32::INFINITY, f32::NEG_INFINITY];
    let mut count = 0;
    for vertex in builder
        .vertices
        .iter()
        .filter(|v| v.color == [0.23, 0.25, 0.26])
    {
        let sample = world::roads::project(&track, (vertex.position).into());
        assert!(sample.offset.abs() - sample.width * 0.5 < 0.03);
        assert!(
            (vertex.position[1]
                - world::elevation::world_height(
                    &track,
                    XZ {
                        x: vertex.position[0],
                        z: vertex.position[2]
                    }
                )
                - 0.05)
                .abs()
                < 0.001
        );
        if (sample.offset.abs() - sample.width * 0.5).abs() < 0.03 {
            widths[0] = widths[0].min(sample.width);
            widths[1] = widths[1].max(sample.width);
        }
        count += 1;
    }
    assert!(count > 100);
    assert!(widths[0] < 4.1 && widths[1] > 11.9);
}

#[test]
fn chunked_builder_preserves_triangle_geometry() {
    let mut regular = super::Builder::default();
    let mut streamed = super::Builder {
        chunks: Some(render::chunks::ChunkBuilder::default()),
        ..super::Builder::default()
    };
    for x in [-300., 0., 300.] {
        let points = [
            XYZ { x: x, y: 0., z: 0. },
            XYZ {
                x: x + 2.,
                y: 0.,
                z: 0.,
            },
            XYZ { x: x, y: 0., z: 2. },
        ];
        let surface = render::scene::matte([0.2, 0.4, 0.2]);
        render::scene::emit_triangle(&mut regular, points, surface);
        render::scene::emit_triangle(&mut streamed, points, surface);
    }
    assert!(streamed.vertices.is_empty());
    let got = render::chunks::finish(streamed.chunks.unwrap());
    let expected = test::support::partition(regular.vertices);
    assert_eq!(got.len(), expected.len());
    for (a, b) in got.iter().zip(&expected) {
        assert_eq!(a.cell, b.cell);
        assert_eq!(a.bounds, b.bounds);
        assert_eq!(a.vertices, b.vertices);
    }
}
use super::*;

#[test]
fn scenic_blockout_is_finite_bounded_and_uses_the_selected_road() {
    let route = world::locations::try_track(&test::support::authored_id()).unwrap();
    let geometry = build(&route, &test::support::cars());
    assert!(
        geometry.world_count > 100_000 && geometry.world_count < 1_800_000,
        "scenic geometry budget exceeded: {} world / {} total vertices",
        geometry.world_count,
        geometry.vertices.len()
    );
    for v in &geometry.vertices {
        assert!(v
            .position
            .iter()
            .chain(v.normal.iter())
            .all(|p| p.is_finite()));
        assert!((v.normal.iter().map(|v| v * v).sum::<f32>() - 1.0).abs() < 0.001);
        if v.material[0] == -1.0 && v.color == [0.23, 0.25, 0.26] {
            assert!(v.normal[1] > 0.9);
            assert!(
                (v.position[1]
                    - world::elevation::world_height(
                        &route,
                        XZ {
                            x: v.position[0],
                            z: v.position[2]
                        }
                    )
                    - 0.05)
                    .abs()
                    < 0.001
            );
        }
    }
    assert!(geometry.vertices[..geometry.world_count]
        .iter()
        .any(|v| v.position[1] > 30.0));
}
