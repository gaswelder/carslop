//! Procedural coastal scenery and file-backed car artwork. No physics lives here.

use crate::{
    cars::{self, CarModel},
    track,
};
use std::f32::consts::TAU;
use std::ops::Range;

#[path = "car_glb.rs"]
pub(crate) mod car_glb;
#[path = "city_scene.rs"]
mod city_scene;
#[path = "road_furniture.rs"]
mod road_furniture;

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
struct Surface {
    color: [f32; 3],
    material: [f32; 2],
}

impl Surface {
    fn matte(color: [f32; 3]) -> Self {
        Self {
            color,
            material: [0.0, 0.0],
        }
    }
    fn road(color: [f32; 3]) -> Self {
        Self {
            color,
            material: [-1.0, 0.0],
        }
    }
}

#[derive(Default)]
struct Builder {
    vertices: Vec<Vertex>,
    chunks: Option<crate::chunks::ChunkBuilder>,
}

impl Builder {
    fn triangle(&mut self, points: [[f32; 3]; 3], surface: Surface) {
        let a: [f32; 3] = std::array::from_fn(|i| points[1][i] - points[0][i]);
        let b: [f32; 3] = std::array::from_fn(|i| points[2][i] - points[0][i]);
        let n = [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ];
        let length = n.iter().map(|x| x * x).sum::<f32>().sqrt();
        if length < 0.00001 {
            return;
        }
        let normal = n.map(|x| x / length);
        let vertices = points.map(|position| Vertex {
            position,
            normal,
            color: surface.color,
            material: surface.material,
        });
        if let Some(chunks) = &mut self.chunks {
            chunks.push(vertices);
        } else {
            self.vertices.extend(vertices);
        }
    }

    fn quad(&mut self, p: [[f32; 3]; 4], surface: Surface) {
        self.triangle([p[0], p[1], p[2]], surface);
        self.triangle([p[0], p[2], p[3]], surface);
    }

    fn upward_quad(&mut self, p: [[f32; 3]; 4], surface: Surface) {
        // Offset edges can fold locally where piecewise track tangents change.
        // Orient each triangle separately so road tops never acquire underside lighting.
        for mut triangle in [[p[0], p[1], p[2]], [p[0], p[2], p[3]]] {
            let a = triangle[0];
            let c = triangle[1];
            let d = triangle[2];
            if (c[2] - a[2]) * (d[0] - a[0]) - (c[0] - a[0]) * (d[2] - a[2]) < 0.0 {
                triangle.swap(1, 2);
            }
            self.triangle(triangle, surface);
        }
    }

    fn cuboid(&mut self, min: [f32; 3], max: [f32; 3], surface: Surface) {
        let p = [
            [min[0], min[1], min[2]],
            [max[0], min[1], min[2]],
            [max[0], max[1], min[2]],
            [min[0], max[1], min[2]],
            [min[0], min[1], max[2]],
            [max[0], min[1], max[2]],
            [max[0], max[1], max[2]],
            [min[0], max[1], max[2]],
        ];
        for [a, b, c, d] in [
            [3, 2, 1, 0],
            [5, 6, 7, 4],
            [1, 5, 4, 0],
            [7, 6, 2, 3],
            [4, 7, 3, 0],
            [2, 6, 5, 1],
        ] {
            self.quad([p[a], p[b], p[c], p[d]], surface);
        }
    }
}

pub struct Geometry {
    pub vertices: Vec<Vertex>,
    pub world_count: usize,
    pub car_ranges: Vec<Range<i32>>,
    pub wheel_range: Vec<[Range<i32>; 4]>,
}

pub fn build(track: &track::Track) -> Geometry {
    if let Some(city) = track.city() {
        let mut b = Builder::default();
        city_scene::storefront_signs(&mut b, city);
        return finish(b);
    }
    let mut b = Builder::default();
    terrain(&mut b, track);
    road(&mut b, track);
    scenery(&mut b, track);
    finish(b)
}

pub fn city_layers(city: &crate::city::City) -> [Vec<crate::chunks::Chunk>; 3] {
    std::array::from_fn(|layer| {
        let mut b = Builder {
            chunks: Some(crate::chunks::ChunkBuilder::default()),
            ..Builder::default()
        };
        if layer == 0 {
            city_scene::ground_mesh(&mut b, city);
        } else {
            city_scene::buildings(&mut b, city, layer == 1);
        }
        b.chunks.unwrap().finish()
    })
}

#[cfg(test)]
pub fn cars_only() -> Geometry {
    finish(Builder::default())
}
fn finish(mut b: Builder) -> Geometry {
    let world_count = b.vertices.len();
    let car_ranges = cars::ALL.map(|model| {
        let start = b.vertices.len() as i32;
        car(&mut b, model);
        start..b.vertices.len() as i32
    });
    let wheel_range = cars::ALL.map(|model| {
        let design = &model.spec().design;
        let mut ranges: [Range<i32>; 4] = std::array::from_fn(|_| 0..0);
        for index in 0..4 {
            if let Some(previous) = (0..index).find(|&i| design.wheels[i] == design.wheels[index]) {
                ranges[index] = ranges[previous].clone();
            } else {
                let start = b.vertices.len() as i32;
                b.vertices.extend_from_slice(&design.wheels[index]);
                ranges[index] = start..b.vertices.len() as i32;
            }
        }
        ranges
    });
    Geometry {
        vertices: b.vertices,
        world_count,
        car_ranges,
        wheel_range,
    }
}

fn terrain_height(track: &track::Track, x: f32, z: f32) -> f32 {
    let s = track.project([x, 0.0, z]);
    if track.id == track::TrackId::Scenic {
        let half = track.width * 0.5;
        let levels = [
            (-half - 22.0, crate::ambience::SEA_LEVEL - 0.4),
            (
                -half - 10.0,
                s.point[1] * 0.3 + crate::ambience::SEA_LEVEL * 0.7,
            ),
            (-half - 1.0, s.point[1] - 0.18),
            (half + 1.0, s.point[1] - 0.18),
            (half + 15.0, s.point[1] - 2.0),
            (half + 50.0, -1.8),
        ];
        for pair in levels.windows(2) {
            if s.offset <= pair[1].0 {
                let t = ((s.offset - pair[0].0) / (pair[1].0 - pair[0].0)).clamp(0.0, 1.0);
                return pair[0].1 + (pair[1].1 - pair[0].1) * t;
            }
        }
        return -1.8;
    }
    let distance = (x - s.point[0]).hypot(z - s.point[2]);
    let blend = ((distance - track.width * 0.5 - 1.0) / 9.0).clamp(0.0, 1.0);
    let floor = if s.offset > 0.0 { -1.8 } else { -3.6 };
    (s.point[1] - 0.25) * (1.0 - blend) + floor * blend
}

fn terrain(b: &mut Builder, track: &track::Track) {
    // Sea extends past the camera's fog distance; land slopes down from the road.
    let sea = crate::ambience::SEA_LEVEL;
    let [min, max] = track.bounds;
    let pad = 700.0;
    b.quad(
        [
            [min[0] - pad, sea, min[1] - pad],
            [min[0] - pad, sea, max[1] + pad],
            [max[0] + pad, sea, max[1] + pad],
            [max[0] + pad, sea, min[1] - pad],
        ],
        Surface {
            color: [0.16, 0.43, 0.52],
            material: [0.0, 0.35],
        },
    );
    if track.id == track::TrackId::Scenic {
        scenic_terrain(b, track);
        return;
    }
    for x in -26..26 {
        for z in -26..26 {
            let point = |x: i32, z: i32| {
                let (x, z) = (x as f32 * 2.0, z as f32 * 2.0);
                [x, terrain_height(track, x, z), z]
            };
            let p = [
                point(x, z),
                point(x, z + 1),
                point(x + 1, z + 1),
                point(x + 1, z),
            ];
            let s = track.project(p[0]);
            let color = if s.offset < -5.0 {
                [0.48, 0.45, 0.34]
            } else if (x + 2 * z).rem_euclid(5) == 0 {
                [0.33, 0.42, 0.21]
            } else {
                [0.28, 0.38, 0.19]
            };
            b.quad(p, Surface::matte(color));
        }
    }
}

fn scenic_terrain(b: &mut Builder, track: &track::Track) {
    // Corridor ribbons scale with road length, not the area of a huge dense grid.
    let count = (track.length() / 4.0).ceil() as usize;
    let half = track.width * 0.5;
    let center = [
        (track.bounds[0][0] + track.bounds[1][0]) * 0.5,
        -1.8,
        (track.bounds[0][1] + track.bounds[1][1]) * 0.5,
    ];
    for i in 0..count {
        let a = track.at_distance(track.length() * i as f32 / count as f32);
        let c = track.at_distance(track.length() * (i + 1) as f32 / count as f32);
        let point = |s: track::RoadSample, offset: f32, height: f32| {
            let n = s.normal();
            [
                s.point[0] + n[0] * offset,
                height,
                s.point[2] + n[1] * offset,
            ]
        };
        let levels = |s: track::RoadSample| {
            [
                point(s, -half - 22.0, crate::ambience::SEA_LEVEL - 0.4),
                point(
                    s,
                    -half - 10.0,
                    s.point[1] * 0.3 + crate::ambience::SEA_LEVEL * 0.7,
                ),
                point(s, -half - 1.0, s.point[1] - 0.18),
                point(s, half + 1.0, s.point[1] - 0.18),
                point(s, half + 15.0, s.point[1] - 2.0),
                point(s, half + 50.0, -1.8),
            ]
        };
        let p = levels(a);
        let q = levels(c);
        for j in [0, 1, 3, 4] {
            let color = if j < 2 {
                [0.43, 0.42, 0.32]
            } else {
                [0.27, 0.38, 0.21]
            };
            b.upward_quad([p[j], p[j + 1], q[j + 1], q[j]], Surface::matte(color));
        }
        // Low interior fill remains below every road section, even on concave bends.
        b.upward_quad(
            [center, p[5], q[5], center],
            Surface::matte([0.25, 0.35, 0.20]),
        );
    }
}

fn road(b: &mut Builder, track: &track::Track) {
    let half = track.width * 0.5;
    let count = (track.length() / 0.7).ceil() as usize;
    for i in 0..count {
        let a = track.at_distance(track.length() * i as f32 / count as f32);
        let c = track.at_distance(track.length() * (i + 1) as f32 / count as f32);
        let point = |s: track::RoadSample, offset: f32, height: f32| {
            let n = s.normal();
            [
                s.point[0] + n[0] * offset,
                s.point[1] + height,
                s.point[2] + n[1] * offset,
            ]
        };
        let strip = |b: &mut Builder, left: f32, right: f32, y: f32, color| {
            b.upward_quad(
                [
                    point(a, left, y),
                    point(a, right, y),
                    point(c, right, y),
                    point(c, left, y),
                ],
                Surface::road(color),
            );
        };
        strip(b, -half - 1.0, half + 1.0, 0.0, [0.38, 0.40, 0.26]);
        if track.id == track::TrackId::Scenic && a.distance < 320.0 {
            // First representative coast section: mineral shoulders sit below
            // the asphalt lip, with muted gravel against the dry verge.
            strip(b, -half - 0.9, -half, 0.02, [0.53, 0.49, 0.38]);
            strip(b, half, half + 0.9, 0.02, [0.53, 0.49, 0.38]);
        }
        strip(b, -half, half, 0.05, [0.23, 0.25, 0.26]);
        let curb = if track.id == track::TrackId::Scenic {
            [0.80, 0.81, 0.73]
        } else if i / 3 % 2 == 0 {
            [0.78, 0.25, 0.16]
        } else {
            [0.86, 0.83, 0.69]
        };
        strip(b, -half, -half + 0.25, 0.065, curb);
        strip(b, half - 0.25, half, 0.065, curb);
        if i / 4 % 2 == 0 {
            strip(b, -0.04, 0.04, 0.07, [0.92, 0.82, 0.55]);
        }
        for side in [-1.0, 1.0] {
            let metal = Surface {
                color: [0.52, 0.57, 0.56],
                material: [0.0, 0.2],
            };
            let p = point(a, side * half, 0.36);
            let q = point(c, side * half, 0.36);
            let top = |p: [f32; 3]| [p[0], p[1] + 0.29, p[2]];
            b.quad([p, q, top(q), top(p)], metal);
            // Rail tops are not part of the road's contact-shadow receiver.
            b.upward_quad(
                [
                    point(a, side * half - 0.09, 0.65),
                    point(a, side * half + 0.09, 0.65),
                    point(c, side * half + 0.09, 0.65),
                    point(c, side * half - 0.09, 0.65),
                ],
                metal,
            );
            if i % 5 == 0 {
                b.cuboid(
                    [p[0] - 0.06, p[1] - 0.36, p[2] - 0.06],
                    [p[0] + 0.06, p[1] + 0.3, p[2] + 0.06],
                    metal,
                );
            }
        }
    }
    if track.id == track::TrackId::Scenic {
        return;
    }
    let start = track.at_distance(0.0);
    let n = start.normal();
    for row in 0..2 {
        for col in 0..12 {
            let point = |x: f32, z: f32| {
                let across = -half + (col as f32 + x) * track.width / 12.0;
                let along = (row as f32 + z) * 0.45 - 0.45;
                [
                    start.point[0] + n[0] * across + start.tangent[0] * along,
                    start.point[1] + 0.09,
                    start.point[2] + n[1] * across + start.tangent[1] * along,
                ]
            };
            b.upward_quad(
                [
                    point(0.0, 0.0),
                    point(1.0, 0.0),
                    point(1.0, 1.0),
                    point(0.0, 1.0),
                ],
                Surface::road(if (row + col) % 2 == 0 {
                    [0.95; 3]
                } else {
                    [0.04; 3]
                }),
            );
        }
    }
}

fn tree(b: &mut Builder, track: &track::Track, x: f32, z: f32, scale: f32) {
    let y = terrain_height(track, x, z) - 0.15;
    b.cuboid(
        [x - 0.14, y, z - 0.14],
        [x + 0.14, y + 2.2 * scale, z + 0.14],
        Surface::matte([0.28, 0.22, 0.14]),
    );
    for layer in 0..3 {
        let base = y + (1.1 + layer as f32 * 0.8) * scale;
        let radius = (1.35 - layer as f32 * 0.27) * scale;
        for i in 0..7 {
            let a = i as f32 * TAU / 7.0;
            let c = (i + 1) as f32 * TAU / 7.0;
            b.triangle(
                [
                    [x + radius * a.cos(), base, z + radius * a.sin()],
                    [x, base + 1.6 * scale, z],
                    [x + radius * c.cos(), base, z + radius * c.sin()],
                ],
                Surface::matte([
                    0.12 + layer as f32 * 0.025,
                    0.27 + layer as f32 * 0.025,
                    0.16,
                ]),
            );
        }
    }
}

fn house(b: &mut Builder, track: &track::Track, x: f32, z: f32) {
    let y = [-2.0, 2.0]
        .into_iter()
        .flat_map(|dx| [-2.0, 2.0].map(|dz| terrain_height(track, x + dx, z + dz)))
        .fold(f32::INFINITY, f32::min)
        - 0.15;
    let wall = Surface::matte([0.78, 0.72, 0.55]);
    let roof = Surface::matte([0.46, 0.23, 0.15]);
    b.cuboid([x - 2.0, y, z - 2.0], [x + 2.0, y + 3.7, z + 2.0], wall);
    let p = [
        [x - 2.3, y + 3.7, z - 2.3],
        [x + 2.3, y + 3.7, z - 2.3],
        [x, y + 5.1, z - 2.3],
        [x - 2.3, y + 3.7, z + 2.3],
        [x + 2.3, y + 3.7, z + 2.3],
        [x, y + 5.1, z + 2.3],
    ];
    b.quad([p[3], p[5], p[2], p[0]], roof);
    b.quad([p[5], p[4], p[1], p[2]], roof);
    b.triangle([p[2], p[1], p[0]], wall);
    b.triangle([p[4], p[5], p[3]], wall);
    b.cuboid(
        [x + 0.8, y + 3.5, z + 0.4],
        [x + 1.25, y + 5.4, z + 0.9],
        wall,
    );
    for dx in [-1.0, 1.0] {
        for side in [-1.0, 1.0] {
            let z = z + side * 2.015;
            b.cuboid(
                [x + dx - 0.34, y + 1.6, z - 0.02],
                [x + dx + 0.34, y + 2.6, z + 0.02],
                Surface {
                    color: [0.12, 0.23, 0.27],
                    material: [0.0, 0.5],
                },
            );
        }
    }
}

fn scenery(b: &mut Builder, track: &track::Track) {
    if track.id == track::TrackId::Scenic {
        // Start with a single 320 m coast section. Keep the sea side open and
        // set inland vegetation beyond both the barrier and chase-camera path.
        for i in 0..18 {
            let sample = track.at_distance(24.0 + i as f32 * 16.0);
            let p = sample.lane_position(track.width * 0.5 + 9.0 + (i % 3) as f32 * 1.5);
            let nearest = track.project(p);
            if (p[0] - nearest.point[0]).hypot(p[2] - nearest.point[2]) > track.width * 0.5 + 6.0 {
                tree(b, track, p[0], p[2], 1.15 + (i % 4) as f32 * 0.13);
            }
        }
        return;
    }
    // Every prop's footprint is checked against the whole route, not one sample.
    for i in 0..42 {
        let sample = track.at_distance(i as f32 * track.length() / 42.0);
        let p = sample.lane_position(7.5 + (i % 3) as f32 * 1.5);
        let nearest = track.project(p);
        let clearance = (p[0] - nearest.point[0]).hypot(p[2] - nearest.point[2]);
        if clearance > track.width * 0.5 + 2.1 {
            tree(b, track, p[0], p[2], 0.8 + (i % 4) as f32 * 0.12);
        }
    }
    for [x, z] in [[-9.0, -7.0], [-4.0, 4.0], [8.0, 9.0], [11.0, -7.0]] {
        let s = track.project([x, 0.0, z]);
        if (x - s.point[0]).hypot(z - s.point[2]) > track.width * 0.5 + 3.5 {
            house(b, track, x, z);
        }
    }
    // Offshore rock stacks provide silhouettes beyond the circuit.
    for [x, z, height] in [[-50.0, -12.0, 8.0], [-48.0, 9.0, 5.0], [46.0, 22.0, 7.0]] {
        for i in 0..7 {
            let a = i as f32 * TAU / 7.0;
            let c = (i + 1) as f32 * TAU / 7.0;
            b.triangle(
                [
                    [x + a.cos() * 3.5, -3.5, z + a.sin() * 3.5],
                    [x - 0.6, height - 3.5, z],
                    [x + c.cos() * 3.5, -3.5, z + c.sin() * 3.5],
                ],
                Surface::matte([0.52, 0.49, 0.38]),
            );
        }
    }
}

#[cfg(test)]
fn wheel(b: &mut Builder, x: f32, z: f32) {
    let mut vertices = CarModel::Roadster.spec().design.wheels[0].clone();
    for vertex in &mut vertices {
        vertex.position[0] += x;
        vertex.position[2] += z;
    }
    b.vertices.extend(vertices);
}
fn car(b: &mut Builder, model: CarModel) {
    b.vertices.extend_from_slice(&model.spec().design.body);
}

#[cfg(test)]
mod tests {
    #[test]
    fn chunked_builder_preserves_triangle_geometry() {
        let mut regular = super::Builder::default();
        let mut streamed = super::Builder {
            chunks: Some(crate::chunks::ChunkBuilder::default()),
            ..super::Builder::default()
        };
        for x in [-300., 0., 300.] {
            let points = [[x, 0., 0.], [x + 2., 0., 0.], [x, 0., 2.]];
            let surface = super::Surface::matte([0.2, 0.4, 0.2]);
            regular.triangle(points, surface);
            streamed.triangle(points, surface);
        }
        assert!(streamed.vertices.is_empty());
        let got = streamed.chunks.unwrap().finish();
        let expected = crate::chunks::partition(regular.vertices);
        assert_eq!(got.len(), expected.len());
        for (a, b) in got.iter().zip(&expected) {
            assert_eq!(a.cell, b.cell);
            assert_eq!(a.bounds, b.bounds);
            assert_eq!(a.vertices, b.vertices);
        }
    }
    use super::*;

    #[test]
    fn scene_has_finite_unit_normals_and_separate_draw_ranges() {
        let Geometry {
            vertices: v,
            world_count: world,
            car_ranges,
            wheel_range,
            ..
        } = build(&track::TrackId::Compact.track());
        assert!(world > 1000 && v.len() > world + 500);
        assert_eq!(world % 3, 0);
        assert_eq!(v.len() % 3, 0);
        assert_eq!(std::mem::size_of::<Vertex>(), 11 * 4);
        for vertex in &v {
            assert!(vertex
                .position
                .iter()
                .chain(vertex.normal.iter())
                .all(|x| x.is_finite()));
            assert!((vertex.normal.iter().map(|x| x * x).sum::<f32>() - 1.0).abs() < 0.0001);
        }
        assert!(v[..world].iter().all(|v| v.material[0] <= 0.0));
        assert!(v[..world].iter().any(|v| v.material[0] == -1.0));
        assert!(v[world..].iter().all(|v| v.material[0] >= 0.0));
        assert!(v[world..].iter().any(|v| v.material[0] == 1.0));
        assert!(v[world..].iter().any(|v| v.material[0] == 0.0));
        let mut end = world as i32;
        for range in car_ranges {
            assert_eq!(range.start, end);
            assert!(range.end - range.start > 200);
            assert_eq!((range.end - range.start) % 3, 0);
            let vertices = &v[range.start as usize..range.end as usize];
            assert!(vertices.iter().any(|v| v.material[0] == 1.0));
            assert!(vertices.iter().any(|v| v.material[0] == 0.0));
            end = range.end;
        }
        let mut uploaded_wheels = Vec::new();
        for range in wheel_range.into_iter().flatten() {
            assert!(range.end - range.start > 200);
            if !uploaded_wheels.contains(&range) {
                assert_eq!(end, range.start);
                end = range.end;
                uploaded_wheels.push(range);
            }
        }
        assert_eq!(end as usize, v.len());
    }

    #[test]
    fn car_body_fits_clearance_and_axle_overhangs() {
        for model in cars::ALL {
            let mut b = Builder::default();
            car(&mut b, model);
            let wheels = model.wheel_centers();
            let rear = wheels[0][2].min(wheels[1][2]);
            let front = wheels[2][2].max(wheels[3][2]);
            for v in b.vertices {
                assert!(v.position[0].abs() < crate::track::CAR_RADIUS);
                assert!(
                    v.position[2] > rear - 1.2 && v.position[2] < front + 1.2,
                    "{model:?}: excessive overhang"
                );
                assert!(v.position[1] >= -0.421 && v.position[1] <= 1.25);
            }
        }
    }

    #[test]
    fn car_silhouettes_are_distinct_not_just_paint_variants() {
        let extents = cars::ALL.map(|model| {
            let mut b = Builder::default();
            car(&mut b, model);
            let min: [f32; 3] = std::array::from_fn(|i| {
                b.vertices
                    .iter()
                    .map(|v| v.position[i])
                    .fold(f32::INFINITY, f32::min)
            });
            let max: [f32; 3] = std::array::from_fn(|i| {
                b.vertices
                    .iter()
                    .map(|v| v.position[i])
                    .fold(f32::NEG_INFINITY, f32::max)
            });
            std::array::from_fn::<_, 3, _>(|i| max[i] - min[i])
        });
        let [roadster, muscle, club] = [
            extents[CarModel::Roadster.index()],
            extents[CarModel::Muscle.index()],
            extents[CarModel::Club.index()],
        ];
        assert!(muscle[0] > club[0] && roadster[0] > club[0]);
        assert!(muscle[2] > roadster[2] && roadster[2] > club[2]);
        assert!(club[1] > muscle[1]);
        let commuter = extents[cars::ALL
            .into_iter()
            .find(|m| m.spec().name == "2016 PITSUNDA 2")
            .unwrap()
            .index()];
        assert!(commuter[1] > club[1] && commuter[2] < muscle[2]);
    }

    #[test]
    fn every_car_has_stop_lenses_on_both_rear_corners() {
        for model in cars::ALL {
            let mut b = Builder::default();
            car(&mut b, model);
            for side in [-1.0, 1.0] {
                assert!(
                    b.vertices.iter().any(|v| {
                        v.material[0] == 5.0 && v.position[0] * side > 0.2 && v.position[2] < 0.0
                    }),
                    "missing rear stop lens: {model:?}, side {side}"
                );
            }
            assert!(b
                .vertices
                .iter()
                .all(|v| v.material[0] != 5.0 || v.position[2] < 0.0));
            assert!(b.vertices.iter().any(|v| v.material[0] == 1.0));
        }
    }

    #[test]
    fn kestrel_uses_its_original_clear_reverse_lenses() {
        let mut b = Builder::default();
        car(&mut b, CarModel::Roadster);
        let lenses: Vec<_> = b.vertices.iter().filter(|v| v.material[0] == 6.0).collect();
        assert!(lenses.len() >= 60);
        for v in lenses {
            assert!(v.position[0].abs() < 0.47);
            assert!((-2.10..-2.07).contains(&v.position[2]));
            assert!(v.color[0] > 0.8 && v.color[1] > 0.8 && v.color[2] > 0.8);
        }
    }

    #[test]
    fn roadster_reference_mesh_has_authored_lights_arches_and_trim() {
        let mut b = Builder::default();
        car(&mut b, CarModel::Roadster);
        let count = |color: [f32; 3]| b.vertices.iter().filter(|v| v.color == color).count();
        assert!(
            b.vertices.len() > 520,
            "reference Roadster needs more than a coarse body shell"
        );
        assert!(
            count([0.84, 0.88, 0.90]) >= 60,
            "missing separate headlamp geometry"
        );
        assert!(
            count([0.78, 0.02, 0.015]) >= 60,
            "missing separate tail lamp geometry"
        );
        assert!(
            count([0.78, 0.80, 0.76]) >= 60,
            "missing beltline/bright trim"
        );
        let dark_side_detail = b
            .vertices
            .iter()
            .filter(|v| {
                v.color == [0.045, 0.055, 0.06]
                    && v.position[0].abs() > 0.8
                    && v.position[1] < 0.35
                    && (v.position[2].abs() - 1.3).abs() < 0.6
            })
            .count();
        assert!(
            dark_side_detail >= 24,
            "shaped wheel arch recesses should be visible on the sides"
        );
    }

    #[test]
    fn reference_panels_have_curved_smooth_shading() {
        let mut b = Builder::default();
        car(&mut b, CarModel::Roadster);
        // The first panel is the sculpted deck. Its triangles must interpolate
        // distinct normals; extra triangles with constant face normals would
        // retain the old faceted appearance.
        let curved_triangles = b.vertices[..18_000]
            .as_chunks::<3>()
            .0
            .iter()
            .filter(|t| {
                t[0].normal
                    .iter()
                    .zip(t[1].normal)
                    .any(|(a, b)| (a - b).abs() > 0.0001)
            })
            .count();
        assert!(curved_triangles > 2000, "body panels lost smooth curvature");
        eprintln!("Kestrel body: {} triangles", b.vertices.len() / 3);
        let mut wheels = Builder::default();
        wheel(&mut wheels, 0.0, 0.0);
        eprintln!("Shared wheel: {} triangles", wheels.vertices.len() / 3);
    }

    #[test]
    fn cuboid_normals_point_outward() {
        let mut b = Builder::default();
        b.cuboid([-1.0; 3], [1.0; 3], Surface::matte([1.0; 3]));
        for v in b.vertices {
            assert!(
                v.position
                    .iter()
                    .zip(v.normal)
                    .map(|(p, n)| p * n)
                    .sum::<f32>()
                    > 0.99
            );
        }
    }

    #[test]
    fn road_markings_face_the_sky_and_wheels_face_outward() {
        let mut b = Builder::default();
        road(&mut b, &track::TrackId::Compact.track());
        for v in &b.vertices {
            // Asphalt, curbs, and lane/finish markings all have upward normals.
            if v.color == [0.23, 0.25, 0.26]
                || v.color == [0.92, 0.82, 0.55]
                || v.color == [0.95; 3]
                || v.color == [0.04; 3]
            {
                assert!(
                    v.normal[1] > 0.95,
                    "road surface at {:?}: normal {:?}, color {:?}",
                    v.position,
                    v.normal,
                    v.color
                );
            }
        }
        b.vertices.clear();
        wheel(&mut b, 0.0, 0.0);
        for v in b.vertices {
            if v.color == [0.035, 0.04, 0.045] {
                let radial = v.position;
                assert!(radial.iter().zip(v.normal).map(|(p, n)| p * n).sum::<f32>() > 0.0);
            }
        }
    }

    #[test]
    fn contact_receivers_include_markings_but_exclude_guardrails() {
        let mut b = Builder::default();
        road(&mut b, &track::TrackId::Compact.track());
        let mut markings = 0;
        let mut rails = 0;
        for v in b.vertices {
            if v.color == [0.95; 3] || v.color == [0.04; 3] || v.color == [0.92, 0.82, 0.55] {
                assert_eq!(v.material[0], -1.0);
                markings += 1;
            }
            if v.color == [0.52, 0.57, 0.56] {
                assert_eq!(v.material[0], 0.0);
                rails += 1;
            }
        }
        assert!(markings > 0 && rails > 0);
    }

    #[test]
    fn scenic_blockout_is_finite_bounded_and_uses_the_selected_road() {
        let route = track::TrackId::Scenic.track();
        let geometry = build(&route);
        assert!(
            geometry.world_count > 100_000 && geometry.world_count < 800_000,
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
            if v.material[0] == -1.0 {
                assert!(v.normal[1] > 0.9);
                assert!((v.position[1] - route.project(v.position).point[1]).abs() < 0.25);
            }
        }
        assert!(geometry.vertices[..geometry.world_count]
            .iter()
            .any(|v| v.position[1] > 30.0));
    }
}
