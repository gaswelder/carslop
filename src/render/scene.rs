use crate::build;
use crate::model;
use crate::model::world::Material;
use crate::render;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::f32::consts::TAU;
use std::ops::Range;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
    pub normal: [f32; 3],
    // Material X: 0 untinted, 1 paint, 2 glass, 3 metal, 4 optical lens, 5 brake lens, 6 reverse lens;
    // negative values: -1 road receiver, -2 signal lens, -3 one-sided storefront.
    // Y: specular strength, or encoded signal phase/lens for material -2.
    pub material: [f32; 2],
}

#[derive(Clone, Copy)]
pub struct Surface {
    pub color: [f32; 3],
    pub material: [f32; 2],
}

pub fn matte(color: [f32; 3]) -> Surface {
    Surface {
        color,
        material: [0.0, 0.0],
    }
}
pub fn surface_road(color: [f32; 3]) -> Surface {
    Surface {
        color,
        material: [-1.0, 0.0],
    }
}

#[derive(Default)]
pub struct Builder {
    pub vertices: Vec<Vertex>,
    pub chunks: Option<render::chunks::ChunkBuilder>,
}

pub fn emit_triangle(builder: &mut Builder, points: [XYZ; 3], surface: Surface) {
    let a: XYZ = util::geometry::xyz_from_fn(|i| {
        util::geometry::xyz_component(points[1], i) - util::geometry::xyz_component(points[0], i)
    });
    let b: XYZ = util::geometry::xyz_from_fn(|i| {
        util::geometry::xyz_component(points[2], i) - util::geometry::xyz_component(points[0], i)
    });
    let n = XYZ {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    };
    let length = util::geometry::iter(&n).map(|x| x * x).sum::<f32>().sqrt();
    if length < 0.00001 {
        return;
    }
    let normal = util::geometry::xyz_map(n, |x| x / length);
    let vertices = points.map(|position| Vertex {
        position: (position).into(),
        normal: normal.into(),
        color: surface.color,
        material: surface.material,
    });
    if let Some(chunks) = &mut builder.chunks {
        render::chunks::push(chunks, vertices);
    } else {
        builder.vertices.extend(vertices);
    }
}

pub fn quad(builder: &mut Builder, p: [XYZ; 4], surface: Surface) {
    emit_triangle(builder, [p[0], p[1], p[2]], surface);
    emit_triangle(builder, [p[0], p[2], p[3]], surface);
}

pub fn upward_quad(builder: &mut Builder, p: [XYZ; 4], surface: Surface) {
    // Offset edges can fold locally where piecewise track tangents change.
    // Orient each triangle separately so road tops never acquire underside lighting.
    for mut triangle in [[p[0], p[1], p[2]], [p[0], p[2], p[3]]] {
        let a = triangle[0];
        let c = triangle[1];
        let d = triangle[2];
        if (c.z - a.z) * (d.x - a.x) - (c.x - a.x) * (d.z - a.z) < 0.0 {
            triangle.swap(1, 2);
        }
        emit_triangle(builder, triangle, surface);
    }
}

pub fn cuboid(builder: &mut Builder, min: XYZ, max: XYZ, surface: Surface) {
    let p = [
        XYZ {
            x: min.x,
            y: min.y,
            z: min.z,
        },
        XYZ {
            x: max.x,
            y: min.y,
            z: min.z,
        },
        XYZ {
            x: max.x,
            y: max.y,
            z: min.z,
        },
        XYZ {
            x: min.x,
            y: max.y,
            z: min.z,
        },
        XYZ {
            x: min.x,
            y: min.y,
            z: max.z,
        },
        XYZ {
            x: max.x,
            y: min.y,
            z: max.z,
        },
        XYZ {
            x: max.x,
            y: max.y,
            z: max.z,
        },
        XYZ {
            x: min.x,
            y: max.y,
            z: max.z,
        },
    ];
    for [a, b, c, d] in [
        [3, 2, 1, 0],
        [5, 6, 7, 4],
        [1, 5, 4, 0],
        [7, 6, 2, 3],
        [4, 7, 3, 0],
        [2, 6, 5, 1],
    ] {
        quad(builder, [(p[a]), (p[b]), (p[c]), (p[d])], surface);
    }
}

pub struct Geometry {
    pub vertices: Vec<Vertex>,
    pub world_count: usize,
    pub car_ranges: Vec<Range<i32>>,
    pub wheel_range: Vec<[Range<i32>; 4]>,
}

pub fn build(
    track: &world::locations::WorldLocation,
    cars: &[std::sync::Arc<model::car::CarSpec>],
) -> Geometry {
    if world::street_surface::has_compiled_roads(track) {
        let city = track;
        let mut b = Builder::default();
        render::city::storefront_signs(&mut b, city);
        return finish(b, cars);
    }
    let mut b = Builder::default();
    terrain(&mut b, track);
    road(&mut b, track);
    scenery(&mut b, track);
    finish(b, cars)
}

pub fn generation_stage<T>(name: &str, generate: impl FnOnce() -> T) -> T {
    println!("  Generating {name}...");
    let started = std::time::Instant::now();
    let result = generate();
    println!("  Finished {name} in {:.2?}", started.elapsed());
    result
}

pub fn city_layer_spooled(
    city: &model::world::WorldLocation,
    layer: usize,
    temporary_directory: &std::path::Path,
    boundary: Option<&geo::MultiPolygon<f64>>,
) -> std::io::Result<render::scene_archive::Spool> {
    let mut b = Builder {
        chunks: Some(render::chunks::spooled(temporary_directory)?),
        ..Builder::default()
    };
    if layer == 0 {
        render::city::ground_mesh(&mut b, city, boundary);
        generation_stage("bridges and tunnels", || {
            render::transport::build(&mut b, city)
        });
    } else {
        render::city::buildings(&mut b, city, layer == 1)?;
        render::barriers::build(&mut b, city, layer == 1);
        render::details::build(&mut b, city, layer == 1);
    }
    render::chunks::finish_spooled(b.chunks.unwrap())
}

fn finish(mut b: Builder, cars: &[std::sync::Arc<model::car::CarSpec>]) -> Geometry {
    let world_count = b.vertices.len();
    let car_ranges = cars
        .iter()
        .map(|model| {
            let start = b.vertices.len() as i32;
            car(&mut b, model);
            start..b.vertices.len() as i32
        })
        .collect();
    let wheel_range = cars
        .iter()
        .map(|model| {
            let design = &model.design;
            let mut ranges: [Range<i32>; 4] = std::array::from_fn(|_| 0..0);
            for index in 0..4 {
                if let Some(previous) =
                    (0..index).find(|&i| design.wheels[i] == design.wheels[index])
                {
                    ranges[index] = ranges[previous].clone();
                } else {
                    let start = b.vertices.len() as i32;
                    b.vertices.extend_from_slice(&design.wheels[index]);
                    ranges[index] = start..b.vertices.len() as i32;
                }
            }
            ranges
        })
        .collect();
    Geometry {
        vertices: b.vertices,
        world_count,
        car_ranges,
        wheel_range,
    }
}

fn terrain_height(track: &world::locations::WorldLocation, x: f32, z: f32) -> f32 {
    world_location_terrain_height(track, XZ { x, z })
}

fn terrain(b: &mut Builder, track: &world::locations::WorldLocation) {
    for triangle in model::world::ground_triangles(track) {
        let color = color(triangle.material);
        let points = triangle
            .points
            .map(|p| util::geometry::at_height(p, world_location_terrain_height(track, p) - 0.02));
        upward_quad(
            b,
            [(points[0]), (points[1]), (points[2]), (points[0])],
            surface_road(color),
        );
    }
}

fn water(b: &mut Builder, track: &world::locations::WorldLocation) {
    let sea = world::elevation::SEA_LEVEL;
    let BoundingBoxXZ { min, max } = world::elevation::terrain_bounds(track);
    quad(
        b,
        [
            XYZ {
                x: min.x - 700.0,
                y: sea,
                z: min.z - 700.0,
            },
            XYZ {
                x: min.x - 700.0,
                y: sea,
                z: max.z + 700.0,
            },
            XYZ {
                x: max.x + 700.0,
                y: sea,
                z: max.z + 700.0,
            },
            XYZ {
                x: max.x + 700.0,
                y: sea,
                z: min.z - 700.0,
            },
        ],
        Surface {
            color: [0.16, 0.43, 0.52],
            material: [0.0, 0.35],
        },
    );
}

fn perimeter_fence(b: &mut Builder, track: &world::locations::WorldLocation) {
    let BoundingBoxXZ { min, max } = world::elevation::terrain_bounds(track);
    let field = model::world::elevation(track).expect("authored track terrain");
    // Fence only the outer terrain perimeter, leaving both verges open.
    let corners = [
        XZ { x: min.x, z: min.z },
        XZ { x: max.x, z: min.z },
        XZ { x: max.x, z: max.z },
        XZ { x: min.x, z: max.z },
    ];
    let metal = Surface {
        color: [0.52, 0.57, 0.56],
        material: [0.0, 0.2],
    };
    for edge in 0..4 {
        for (i, pair) in world::elevation::segment(field, corners[edge], corners[(edge + 1) % 4])
            .windows(2)
            .enumerate()
        {
            let [a, c] = [pair[0], pair[1]];
            quad(
                b,
                [
                    XYZ {
                        x: a.x,
                        y: a.y,
                        z: a.z,
                    },
                    XYZ {
                        x: c.x,
                        y: c.y,
                        z: c.z,
                    },
                    XYZ {
                        x: c.x,
                        y: c.y + 0.8,
                        z: c.z,
                    },
                    XYZ {
                        x: a.x,
                        y: a.y + 0.8,
                        z: a.z,
                    },
                ],
                metal,
            );
            if i % 3 == 0 {
                cuboid(
                    b,
                    XYZ {
                        x: a.x - 0.06,
                        y: a.y - 0.02,
                        z: a.z - 0.06,
                    },
                    XYZ {
                        x: a.x + 0.06,
                        y: a.y + 0.9,
                        z: a.z + 0.06,
                    },
                    metal,
                );
            }
        }
    }
}

fn road_quad(
    b: &mut Builder,
    track: &world::locations::WorldLocation,
    points: [XYZ; 4],
    lift: f32,
    surface: Surface,
) {
    if let Some(field) = model::world::elevation(track) {
        for indices in [[0, 1, 2], [0, 2, 3]] {
            world::elevation::drape(
                field,
                indices.map(|i| util::geometry::xz(points[i])),
                |mut triangle| {
                    for point in &mut triangle {
                        point.y += lift;
                    }
                    upward_quad(
                        b,
                        [triangle[0], triangle[1], triangle[2], triangle[0]],
                        surface,
                    );
                },
            );
        }
    } else {
        upward_quad(b, points, surface);
    }
}

fn road(b: &mut Builder, track: &world::locations::WorldLocation) {
    bake_road(b, track, world::locations::coastal_preset(&track.id));
}

fn bake_road(b: &mut Builder, track: &world::locations::WorldLocation, coastal: bool) {
    let count = (world::locations::sampling_length(track) / 0.7).ceil() as usize;
    for i in 0..count {
        let a = world::locations::at_distance(
            track,
            world::locations::sampling_length(track) * i as f32 / count as f32,
        );
        let c = world::locations::at_distance(
            track,
            world::locations::sampling_length(track) * (i + 1) as f32 / count as f32,
        );
        let half_a = a.width * 0.5;
        let half_c = c.width * 0.5;
        let point = |s: world::locations::RoadSample, offset: f32, height: f32| {
            let n = model::world::normal(s);
            XYZ {
                x: s.point.x + n.x * offset,
                y: s.point.y + height,
                z: s.point.z + n.z * offset,
            }
        };
        let strip = |b: &mut Builder, edge_a: XZ<f32>, edge_c: XZ<f32>, y: f32, color| {
            road_quad(
                b,
                track,
                [
                    (point(a, edge_a.x, y)),
                    (point(a, edge_a.z, y)),
                    (point(c, edge_c.z, y)),
                    (point(c, edge_c.x, y)),
                ],
                y,
                surface_road(color),
            );
        };
        strip(
            b,
            XZ {
                x: -half_a - 1.0,
                z: half_a + 1.0,
            },
            XZ {
                x: -half_c - 1.0,
                z: half_c + 1.0,
            },
            0.0,
            [0.38, 0.40, 0.26],
        );
        if coastal && a.distance < 320.0 {
            // First representative coast section: mineral shoulders sit below
            // the asphalt lip, with muted gravel against the dry verge.
            strip(
                b,
                XZ {
                    x: -half_a - 0.9,
                    z: -half_a,
                },
                XZ {
                    x: -half_c - 0.9,
                    z: -half_c,
                },
                0.02,
                [0.53, 0.49, 0.38],
            );
            strip(
                b,
                XZ {
                    x: half_a,
                    z: half_a + 0.9,
                },
                XZ {
                    x: half_c,
                    z: half_c + 0.9,
                },
                0.02,
                [0.53, 0.49, 0.38],
            );
        }
        strip(
            b,
            XZ {
                x: -half_a,
                z: half_a,
            },
            XZ {
                x: -half_c,
                z: half_c,
            },
            0.05,
            [0.23, 0.25, 0.26],
        );
        let curb = if coastal {
            [0.80, 0.81, 0.73]
        } else if i / 3 % 2 == 0 {
            [0.78, 0.25, 0.16]
        } else {
            [0.86, 0.83, 0.69]
        };
        strip(
            b,
            XZ {
                x: -half_a,
                z: -half_a + 0.25,
            },
            XZ {
                x: -half_c,
                z: -half_c + 0.25,
            },
            0.065,
            curb,
        );
        strip(
            b,
            XZ {
                x: half_a - 0.25,
                z: half_a,
            },
            XZ {
                x: half_c - 0.25,
                z: half_c,
            },
            0.065,
            curb,
        );
        if i / 4 % 2 == 0 {
            strip(
                b,
                XZ { x: -0.04, z: 0.04 },
                XZ { x: -0.04, z: 0.04 },
                0.07,
                [0.92, 0.82, 0.55],
            );
        }
    }
    if coastal {
        return;
    }
    let start = world::locations::at_distance(track, 0.0);
    let half = start.width * 0.5;
    let n = model::world::normal(start);
    for row in 0..2 {
        for col in 0..12 {
            let point = |x: f32, z: f32| {
                let across = -half + (col as f32 + x) * start.width / 12.0;
                let along = (row as f32 + z) * 0.45 - 0.45;
                XYZ {
                    x: start.point.x + n.x * across + start.tangent.x * along,
                    y: start.point.y + 0.09,
                    z: start.point.z + n.z * across + start.tangent.z * along,
                }
            };
            road_quad(
                b,
                track,
                [
                    (point(0.0, 0.0)),
                    (point(1.0, 0.0)),
                    (point(1.0, 1.0)),
                    (point(0.0, 1.0)),
                ],
                0.09,
                surface_road(if (row + col) % 2 == 0 {
                    [0.95; 3]
                } else {
                    [0.04; 3]
                }),
            );
        }
    }
}

fn tree(b: &mut Builder, track: &world::locations::WorldLocation, x: f32, z: f32, scale: f32) {
    let y = terrain_height(track, x, z) - 0.15;
    cuboid(
        b,
        XYZ {
            x: x - 0.14,
            y,
            z: z - 0.14,
        },
        XYZ {
            x: x + 0.14,
            y: y + 2.2 * scale,
            z: z + 0.14,
        },
        matte([0.28, 0.22, 0.14]),
    );
    for layer in 0..3 {
        let base = y + (1.1 + layer as f32 * 0.8) * scale;
        let radius = (1.35 - layer as f32 * 0.27) * scale;
        for i in 0..7 {
            let a = i as f32 * TAU / 7.0;
            let c = (i + 1) as f32 * TAU / 7.0;
            emit_triangle(
                b,
                [
                    XYZ {
                        x: x + radius * a.cos(),
                        y: base,
                        z: z + radius * a.sin(),
                    },
                    XYZ {
                        x,
                        y: base + 1.6 * scale,
                        z,
                    },
                    XYZ {
                        x: x + radius * c.cos(),
                        y: base,
                        z: z + radius * c.sin(),
                    },
                ],
                matte([
                    0.12 + layer as f32 * 0.025,
                    0.27 + layer as f32 * 0.025,
                    0.16,
                ]),
            );
        }
    }
}

fn house(b: &mut Builder, track: &world::locations::WorldLocation, x: f32, z: f32) {
    let y = [-2.0, 2.0]
        .into_iter()
        .flat_map(|dx| [-2.0, 2.0].map(|dz| terrain_height(track, x + dx, z + dz)))
        .fold(f32::INFINITY, f32::min)
        - 0.15;
    let wall = matte([0.78, 0.72, 0.55]);
    let roof = matte([0.46, 0.23, 0.15]);
    cuboid(
        b,
        XYZ {
            x: x - 2.0,
            y,
            z: z - 2.0,
        },
        XYZ {
            x: x + 2.0,
            y: y + 3.7,
            z: z + 2.0,
        },
        wall,
    );
    let p = [
        XYZ {
            x: x - 2.3,
            y: y + 3.7,
            z: z - 2.3,
        },
        XYZ {
            x: x + 2.3,
            y: y + 3.7,
            z: z - 2.3,
        },
        XYZ {
            x: x,
            y: y + 5.1,
            z: z - 2.3,
        },
        XYZ {
            x: x - 2.3,
            y: y + 3.7,
            z: z + 2.3,
        },
        XYZ {
            x: x + 2.3,
            y: y + 3.7,
            z: z + 2.3,
        },
        XYZ {
            x: x,
            y: y + 5.1,
            z: z + 2.3,
        },
    ];
    quad(b, [(p[3]), (p[5]), (p[2]), (p[0])], roof);
    quad(b, [(p[5]), (p[4]), (p[1]), (p[2])], roof);
    emit_triangle(b, [(p[2]), (p[1]), (p[0])], wall);
    emit_triangle(b, [(p[4]), (p[5]), (p[3])], wall);
    cuboid(
        b,
        XYZ {
            x: x + 0.8,
            y: y + 3.5,
            z: z + 0.4,
        },
        XYZ {
            x: x + 1.25,
            y: y + 5.4,
            z: z + 0.9,
        },
        wall,
    );
    for dx in [-1.0, 1.0] {
        for side in [-1.0, 1.0] {
            let z = z + side * 2.015;
            cuboid(
                b,
                XYZ {
                    x: x + dx - 0.34,
                    y: y + 1.6,
                    z: z - 0.02,
                },
                XYZ {
                    x: x + dx + 0.34,
                    y: y + 2.6,
                    z: z + 0.02,
                },
                Surface {
                    color: [0.12, 0.23, 0.27],
                    material: [0.0, 0.5],
                },
            );
        }
    }
}

pub struct AuthoredScenery {
    pub trees: Vec<world::landscape::Tree>,
    pub buildings: Vec<build::osm_labels::Building>,
}

pub fn authored_scenery(track: &world::locations::WorldLocation, coastal: bool) -> AuthoredScenery {
    let mut trees = Vec::new();
    let mut buildings = Vec::new();
    let mut add_tree = |p: XYZ, scale: f32| {
        trees.push(world::landscape::Tree {
            id: trees.len() as u64 + 1,
            coords: XZ { x: p.x, z: p.z },
            height_meters: 4.3 * scale,
            crown_radius_meters: 1.35 * scale,
        });
    };
    if coastal {
        for i in 0..18 {
            let sample = world::locations::at_distance(track, 24.0 + i as f32 * 16.0);
            let p = model::world::lane_position(
                sample,
                sample.width * 0.5 + 9.0 + (i % 3) as f32 * 1.5,
            );
            let nearest = world::roads::project(track, p);
            if (p.x - nearest.point.x).hypot(p.z - nearest.point.z) > nearest.width * 0.5 + 6.0 {
                add_tree(p, 1.15 + (i % 4) as f32 * 0.13);
            }
        }
    } else {
        for i in 0..42 {
            let sample = world::locations::at_distance(
                track,
                i as f32 * world::locations::sampling_length(track) / 42.0,
            );
            let p = model::world::lane_position(
                sample,
                sample.width * 0.5
                    + (7.5 - world::locations::ROAD_WIDTH * 0.5)
                    + (i % 3) as f32 * 1.5,
            );
            let nearest = world::roads::project(track, p);
            if (p.x - nearest.point.x).hypot(p.z - nearest.point.z) > nearest.width * 0.5 + 2.1 {
                add_tree(p, 0.8 + (i % 4) as f32 * 0.12);
            }
        }
        for [x, z] in [[-9.0, -7.0], [-4.0, 4.0], [8.0, 9.0], [11.0, -7.0]] {
            let sample = world::roads::project(track, XYZ { x, y: 0.0, z });
            if (x - sample.point.x).hypot(z - sample.point.z) > sample.width * 0.5 + 3.5 {
                buildings.push(build::osm_labels::Building {
                    id: buildings.len() as u64 + 1,
                    height_meters: 5.0,
                    base_y: 0.0,
                    tagged: true,
                    holes: Vec::new(),
                    min_height_meters: 0.0,
                    ground_outline: vec![
                        XZ {
                            x: x - 2.0,
                            z: z - 2.0,
                        },
                        XZ {
                            x: x + 2.0,
                            z: z - 2.0,
                        },
                        XZ {
                            x: x + 2.0,
                            z: z + 2.0,
                        },
                        XZ {
                            x: x - 2.0,
                            z: z + 2.0,
                        },
                    ],
                });
            }
        }
    }
    AuthoredScenery { trees, buildings }
}

fn scenery(b: &mut Builder, track: &world::locations::WorldLocation) {
    for t in model::world::trees(track) {
        tree(b, track, t.coords.x, t.coords.z, t.height_meters / 4.3);
    }
    for building in world::buildings::buildings(track) {
        house(
            b,
            track,
            building.ground_outline[0].x + 2.0,
            building.ground_outline[0].z + 2.0,
        );
    }
    decorations(b, track, world::locations::coastal_preset(&track.id));
}

fn rocks(b: &mut Builder) {
    // Offshore rock stacks provide silhouettes beyond the circuit.
    for [x, z, height] in [[-50.0, -12.0, 8.0], [-48.0, 9.0, 5.0], [46.0, 22.0, 7.0]] {
        for i in 0..7 {
            let a = i as f32 * TAU / 7.0;
            let c = (i + 1) as f32 * TAU / 7.0;
            emit_triangle(
                b,
                [
                    XYZ {
                        x: x + a.cos() * 3.5,
                        y: -3.5,
                        z: z + a.sin() * 3.5,
                    },
                    XYZ {
                        x: x - 0.6,
                        y: height - 3.5,
                        z,
                    },
                    XYZ {
                        x: x + c.cos() * 3.5,
                        y: -3.5,
                        z: z + c.sin() * 3.5,
                    },
                ],
                matte([0.52, 0.49, 0.38]),
            );
        }
    }
}

pub fn bake_authored_landscape(
    track: &world::locations::WorldLocation,
    coastal: bool,
) -> Vec<model::world::Triangle> {
    let field = model::world::elevation(track).expect("authored track terrain");
    let mut ground = Vec::new();
    for row in 0..field.rows - 1 {
        for col in 0..field.columns - 1 {
            let points = [
                world::elevation::point(field, col, row),
                world::elevation::point(field, col + 1, row),
                world::elevation::point(field, col + 1, row + 1),
                world::elevation::point(field, col, row + 1),
            ];
            let material = if coastal && world::roads::project(track, points[0]).offset < 0.0 {
                Material::BareGround
            } else if (col + row * 2) % 5 == 0 {
                Material::Park
            } else {
                Material::Grass
            };
            for indices in [[0, 1, 2], [0, 2, 3]] {
                ground.push(model::world::Triangle {
                    material,
                    usage: model::world::TriangleUsage::Ground,
                    points: indices.map(|i| XZ {
                        x: points[i].x,
                        z: points[i].z,
                    }),
                });
            }
        }
    }
    ground
}

fn decorations(b: &mut Builder, track: &world::locations::WorldLocation, coastal: bool) {
    water(b, track);
    perimeter_fence(b, track);
    if !coastal {
        rocks(b);
    }
}

pub fn authored_decorations(
    track: &world::locations::WorldLocation,
    coastal: bool,
) -> [Vec<render::chunks::Chunk>; 3] {
    let mut builder = Builder::default();
    decorations(&mut builder, track, coastal);
    let mut chunks = render::chunks::ChunkBuilder::default();
    for triangle in builder.vertices.as_chunks::<3>().0 {
        render::chunks::push(&mut chunks, *triangle);
    }
    [render::chunks::finish(chunks), Vec::new(), Vec::new()]
}

fn car(b: &mut Builder, model: &std::sync::Arc<model::car::CarSpec>) {
    b.vertices.extend_from_slice(&model.design.body);
}

pub fn color(material: Material) -> [f32; 3] {
    match material {
        Material::Grass => [0.39, 0.48, 0.25],
        Material::Park => [0.43, 0.53, 0.30],
        Material::Scrub => [0.40, 0.46, 0.28],
        Material::Asphalt => [0.25, 0.26, 0.26],
        Material::Paving => [0.68, 0.69, 0.66],
        Material::Concrete => [0.72, 0.73, 0.70],
        Material::BareGround => [0.62, 0.58, 0.46],
        Material::Sports => [0.29, 0.48, 0.35],
        Material::Water => [0.21, 0.57, 0.67],
        Material::Paint => [0.88, 0.88, 0.80],
    }
}

pub fn road_color(world: &world::locations::WorldLocation, p: XZ<f32>) -> [f32; 3] {
    world::street_surface::world_location_triangle_at(world, p, false)
        .map_or([0.23, 0.25, 0.26], |t| color(t.material))
}

fn world_location_terrain_height(
    world: &world::locations::WorldLocation,
    position: XZ<f32>,
) -> f32 {
    world::elevation::world_height(world, position) - 0.02
}

#[cfg(test)]
mod tests {
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
}
