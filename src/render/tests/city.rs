use super::*;
use crate::build;
use crate::model;
use crate::render;
use crate::storage;
use crate::test;
use crate::util;
use crate::world;
use geo::Contains;

#[test]
fn building_parts_keep_stepped_and_inset_roofs_in_both_detail_levels() {
    struct Case {
        buildings: &'static str,
        expected_height: f32,
    }
    for case in [
        Case { buildings: "b 1 3 0 4 0 0 10 0 10 10 0 10\nb 1 9 0 4 10 0 20 0 20 10 10 10\n", expected_height: 3. },
        Case { buildings: "b 1 6 0 4 0 0 20 0 20 20 0 20\nv 0 0 1 4 3 3 3 15 15 15 15 3\nb 1 12 0 4 3 3 15 3 15 15 3 15\n", expected_height: 12. },
    ] {
        let city = test::support::parse_world(&format!("CITY 2\nn -40 -20\nn 40 -20\nr 0 1 8 0 1\n{}", case.buildings)).unwrap();
        let models = render::building_parts::load_default().unwrap();
        for detailed in [false, true] {
            let mut builder = Builder::default();
            for building in world::buildings::buildings(&city) { building_mesh(&mut builder, building, &city, detailed, &models); }
            let mut roofs = Vec::new();
            for triangle in builder.vertices.chunks_exact(3) {
                if triangle[0].normal[1] < 0.99 { continue; }
                let points: Vec<_> = triangle.iter().map(|v| XZ { x: v.position[0] as f64, z: v.position[2] as f64 }).collect();
                if util::geometry::polygon(&points).contains(&geo::Point::new(4.3, 5.1)) {
                    roofs.push(triangle.iter().map(|v| v.position[1]).sum::<f32>() / 3.);
                }
            }
            assert!(!roofs.is_empty());
            assert!(roofs.iter().all(|height| (*height - case.expected_height).abs() < 0.001), "wrong roof heights: {roofs:?}");
        }
    }
}

#[test]
fn mapped_doors_use_their_walls_and_local_ground_without_an_extra_default() {
    let mut city = test::support::parse_world(concat!(
        "CITY 2\nn -40 -20\nn 40 -20\nr 0 1 8 0 1\n",
        "b 1 6 1 5 0 5 0 0 12 0 12 10 0 10\ne 1 0 5\ne 1 12 7\n",
    ))
    .unwrap();
    let field = world::elevation::generate(model::world::bounding_box(&city), |point| {
        point.x * 0.1 + point.z * 0.02
    });
    test::support::set_elevation(&mut city, Some(field));
    let building = &world::buildings::buildings(&city)[0];
    let edges = facade_edges(&building.ground_outline);
    assert_eq!(edges.len(), 4, "entrance nodes must not split facade runs");
    let models = render::building_parts::load_default().unwrap();
    let doors = door_placements(building, &city, &edges, &models);
    assert_eq!(doors.len(), 2);
    let mut mesh = Builder::default();
    building_mesh(&mut mesh, building, &city, true, &models);
    let tints = Tints {
        wall: [1.; 3],
        frame: [1.; 3],
        glass: [1.; 3],
        trim: [1.; 3],
        shutter: [1.; 3],
    };
    for door in doors {
        let [a, c] = edges[door.wall];
        let length = (c.x - a.x).hypot(c.z - a.z);
        let p = XZ {
            x: a.x + (c.x - a.x) * door.center / length,
            z: a.z + (c.z - a.z) * door.center / length,
        };
        assert!(world::buildings::building_entrances(&(city), 1)
            .iter()
            .any(|entry| (entry.x - p.x).hypot(entry.z - p.z) < 0.0001));
        assert!((door.bottom - world::elevation::world_height(&(city), p) - 0.03).abs() < 0.0001);
        let mut expected = Builder::default();
        render::building_parts::place(
            &models.entrance,
            &mut expected,
            [a, c],
            door.center,
            door.bottom,
            models.entrance.size,
            &tints,
        );
        assert!(!expected.vertices.is_empty());
        for vertex in expected.vertices {
            assert!(
                mesh.vertices.iter().any(|actual| actual
                    .position
                    .iter()
                    .zip(vertex.position)
                    .all(|(a, b)| (*a - b).abs() < 0.00001)),
                "mapped door missing from the facade mesh"
            );
        }
    }
    let city = test::support::parse_world(
        "CITY 2\nn -40 -20\nn 40 -20\nr 0 1 8 0 1\nb 1 6 1 4 0 0 12 0 12 10 0 10\n",
    )
    .unwrap();
    assert_eq!(
        door_placements(
            &world::buildings::buildings(&(city))[0],
            &city,
            &facade_edges(&world::buildings::buildings(&(city))[0].ground_outline),
            &models
        )
        .len(),
        1
    );
}

#[test]
fn shared_building_storefronts_load_and_render_separate_panels() {
    let directory = tempfile::tempdir().unwrap();
    let asset = test::support::stored_location(directory.path(), true);
    let labels = model::location::Labels {
        streets: Vec::new(),
        buildings: vec![(1, "Centre".into())],
        storefronts: vec![(1, "Centre".into()), (1, "Shop".into()), (1, "Cafe".into())],
    };
    storage::codec_labels::write(directory.path(), &labels).unwrap();
    let city = world::preparation::world_location_load(&asset).unwrap();
    assert_eq!(model::world::storefront_names(&(city), 1).len(), 3);
    let mut builder = Builder::default();
    storefront_signs(&mut builder, &city);
    assert!(builder
        .vertices
        .iter()
        .all(|v| v.position.iter().all(|n| n.is_finite())));
    let panels: Vec<_> = builder
        .vertices
        .iter()
        .filter(|v| {
            STOREFRONT_PALETTES
                .iter()
                .any(|(background, _)| v.color == *background)
        })
        .collect();
    assert_eq!(panels.len(), 18, "each business needs its own panel");
    let bounds: Vec<_> = panels
        .chunks_exact(6)
        .map(|vertices| {
            [0, 1, 2].map(|axis| {
                vertices
                    .iter()
                    .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| {
                        (lo.min(v.position[axis]), hi.max(v.position[axis]))
                    })
            })
        })
        .collect();
    for (i, a) in bounds.iter().enumerate() {
        for b in &bounds[i + 1..] {
            assert!(
                (0..3).any(|axis| a[axis].1 < b[axis].0 || b[axis].1 < a[axis].0),
                "storefront panels overlap"
            );
        }
    }
}

fn small_city() -> WorldLocation {
    test::support::parse_world("CITY 2\nn -40 0\nn 40 0\nr 0 1 8 0 1\n").unwrap()
}

#[test]
fn administrative_terrain_preserves_holes_area_and_height_planes() {
    use geo::{Area, BooleanOps, MultiPolygon};
    let mut city = small_city();
    test::support::set_elevation(
        &mut city,
        Some(world::elevation::generate(
            util::geometry::BoundingBoxXZ {
                min: XZ { x: -8., z: -8. },
                max: XZ { x: 8., z: 8. },
            },
            |p| p.x * p.z * 0.1,
        )),
    );
    let shell = util::geometry::polygon(
        &[[-6., -5.], [6., -5.], [6., 5.], [-6., 5.]].map(|[x, z]| XZ { x, z }),
    );
    let hole = util::geometry::polygon(
        &[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]].map(|[x, z]| XZ { x, z }),
    );
    let boundary = shell.difference(&hole);
    let mut builder = Builder::default();
    terrain_mesh(
        &mut builder,
        model::world::elevation(&city).unwrap(),
        Some(&boundary),
        None,
    );
    let mut area = 0.;
    for vertices in builder.vertices.as_chunks::<3>().0 {
        let triangle = util::geometry::polygon(&vertices.map(|v| XZ {
            x: v.position[0] as f64,
            z: v.position[2] as f64,
        }));
        area += triangle.unsigned_area();
        assert!(
            MultiPolygon(vec![triangle])
                .difference(&boundary)
                .unsigned_area()
                < 0.0001
        );
        for v in vertices {
            assert!(
                (v.position[1]
                    - world::elevation::height(
                        model::world::elevation(&(city)).unwrap(),
                        XZ {
                            x: v.position[0],
                            z: v.position[2]
                        }
                    )
                    + 0.02)
                    .abs()
                    < 0.0001
            );
        }
    }
    assert!((area - boundary.unsigned_area()).abs() < 0.0001);
}

#[test]
fn indexed_tunnel_clipping_preserves_terrain_and_ground_cover() {
    use geo::BooleanOps;
    let field = world::elevation::generate(
        util::geometry::BoundingBoxXZ {
            min: XZ { x: -8., z: -8. },
            max: XZ { x: 8., z: 8. },
        },
        |p| p.x * p.z * 0.1,
    );
    let shell = util::geometry::polygon(
        &[[-7., -6.], [7., -6.], [7., 6.], [-7., 6.]].map(|[x, z]| XZ { x, z }),
    );
    let hole = util::geometry::polygon(
        &[[-6., -1.], [-5., -1.], [-5., 1.], [-6., 1.]].map(|[x, z]| XZ { x, z }),
    );
    let boundary = shell.difference(&hole);
    // A hole inside a cell, an opening crossing cell/area edges, and distant openings.
    let openings = geo::MultiPolygon(vec![
        util::geometry::polygon(
            &[[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]].map(|[x, z]| XZ { x, z }),
        ),
        util::geometry::polygon(
            &[[3., -8.], [5., -8.], [5., 8.], [3., 8.]].map(|[x, z]| XZ { x, z }),
        ),
        util::geometry::polygon(
            &[[50., 50.], [60., 50.], [60., 60.], [50., 60.]].map(|[x, z]| XZ { x, z }),
        ),
    ]);
    let index = build::surfaces::new(&openings);
    for boundary in [None, Some(&boundary)] {
        let mut builder = Builder::default();
        terrain_mesh(&mut builder, &field, boundary, Some(&index));
        let full = geo::MultiPolygon(vec![util::geometry::polygon(
            &[[-8., -8.], [8., -8.], [8., 8.], [-8., 8.]].map(|[x, z]| XZ { x, z }),
        )]);
        let expected = boundary.unwrap_or(&full).difference(&openings);
        assert_ground_coverage(
            &field,
            builder.vertices.as_chunks::<3>().0.iter().map(|triangle| {
                triangle.map(|vertex| XYZ {
                    x: vertex.position[0],
                    y: vertex.position[1] + 0.02,
                    z: vertex.position[2],
                })
            }),
            &expected,
        );
    }
    for points in [
        [[-8., -8.], [8., -8.], [8., 8.]],
        [[-7., 2.], [-3., 2.], [-3., 6.]],
    ] {
        let points = points.map(|[x, z]| XZ { x, z });
        let footprint = geo::MultiPolygon(vec![util::geometry::polygon(
            &points.map(|p| util::geometry::xz_map(p, f64::from)),
        )]);
        let mut triangles = Vec::new();
        drape_ground_cover(&field, points, Some(&index), |triangle| {
            triangles.push(triangle)
        });
        assert_ground_coverage(
            &field,
            triangles.iter().copied(),
            &footprint.difference(&openings),
        );
        if build::surfaces::nearby(&index, &footprint).0.is_empty() {
            let mut unclipped = Vec::new();
            world::elevation::drape(&field, points, |triangle| unclipped.push(triangle));
            assert_eq!(
                triangles, unclipped,
                "distant openings must preserve original triangles"
            );
        }
    }
}

fn assert_ground_coverage(
    field: &world::elevation::HeightField,
    triangles: impl Iterator<Item = [XYZ; 3]>,
    expected: &geo::MultiPolygon<f64>,
) {
    use geo::{Area, BooleanOps};
    let mut area = 0.;
    for points in triangles {
        let triangle = geo::MultiPolygon(vec![util::geometry::polygon(
            &points.map(|p| util::geometry::xz_map(util::geometry::xz(p), f64::from)),
        )]);
        area += triangle.unsigned_area();
        assert!(triangle.difference(expected).unsigned_area() < 0.0001);
        for point in points {
            assert!(
                (point.y - world::elevation::height(field, util::geometry::xz(point))).abs()
                    < 0.0001
            );
        }
    }
    assert!((area - expected.unsigned_area()).abs() < 0.0001);
}

#[test]
fn storefront_styles_vary_and_reproduce_the_same_geometry() {
    let label: Vec<_> = "COFFEE SHOP".chars().collect();
    let render = |id, name| {
        let mut builder = Builder::default();
        storefront_sign(
            &mut builder,
            &label,
            [XZ { x: 0.0, z: 0.0 }, XZ { x: 12.0, z: 0.0 }],
            0.0,
            6.0,
            storefront_seed(id, name),
        );
        builder.vertices
    };
    let first = render(100, "Coffee Shop");
    let variants: Vec<_> = (100..116).map(|id| render(id, "Coffee Shop")).collect();
    assert_eq!(first, render(100, "Coffee Shop"));
    assert_ne!(first, render(100, "Next door"));
    let backgrounds: std::collections::HashSet<_> = variants
        .iter()
        .map(|vertices| vertices[0].color.map(f32::to_bits))
        .collect();
    assert!(
        backgrounds.len() >= 4,
        "neighbours should have distinct palettes"
    );
    for vertices in variants {
        assert!(vertices.iter().all(|v| {
            v.position.iter().all(|n| n.is_finite())
                && (0.0..=12.0).contains(&v.position[0])
                && (0.0..=6.0).contains(&v.position[1])
        }));
    }
}

#[test]
fn storefront_glyphs_read_left_to_right_on_each_outward_wall_face() {
    let label = ['F'];
    for [a, c] in [
        [XZ { x: 0.0, z: 0.0 }, XZ { x: 10.0, z: 0.0 }],
        [XZ { x: 10.0, z: 0.0 }, XZ { x: 10.0, z: 10.0 }],
        [XZ { x: 10.0, z: 10.0 }, XZ { x: 0.0, z: 10.0 }],
        [XZ { x: 0.0, z: 10.0 }, XZ { x: 0.0, z: 0.0 }],
    ] {
        let mut b = Builder::default();
        storefront_sign(&mut b, &label, [a, c], 0.0, 8.0, 0);
        let normal = XZ {
            x: (c.z - a.z) / 10.0,
            z: -(c.x - a.x) / 10.0,
        };
        // Camera right = cross(-outward_normal, world_up).
        let right = XZ {
            x: normal.z,
            z: -normal.x,
        };
        let screen_x = |v: &render::scene::Vertex| {
            (v.position[0] - c.x) * right.x + (v.position[2] - c.z) * right.z
        };
        let text = &b.vertices[6..];
        let bottom = text
            .iter()
            .map(|v| v.position[1])
            .fold(f32::INFINITY, f32::min);
        let top = text
            .iter()
            .map(|v| v.position[1])
            .fold(f32::NEG_INFINITY, f32::max);
        let upper_right = text
            .iter()
            .filter(|v| v.position[1] > top - (top - bottom) * 0.2)
            .map(screen_x)
            .fold(f32::NEG_INFINITY, f32::max);
        let lower_right = text
            .iter()
            .filter(|v| v.position[1] < bottom + (top - bottom) * 0.2)
            .map(screen_x)
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(
            upper_right > lower_right,
            "F must extend toward the viewer's right at the top"
        );
        for v in b.vertices {
            assert_eq!(v.material[0], -3.0);
            assert!(v.normal[0] * normal.x + v.normal[2] * normal.z > 0.999);
        }
    }
}
