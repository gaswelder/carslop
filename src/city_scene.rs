//! Original materials on OSM footprints. No downloaded imagery or map tiles.
use super::{Builder, Surface};
use crate::city::{Building, City};

fn cross(a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> f64 {
    (b[0] as f64 - a[0] as f64) * (c[1] as f64 - a[1] as f64)
        - (b[1] as f64 - a[1] as f64) * (c[0] as f64 - a[0] as f64)
}
/// Ear clipping preserves concave footprints rather than filling their courtyards.
fn roof_triangles(polygon: &[[f32; 2]]) -> Vec<[usize; 3]> {
    let mut indices: Vec<_> = (0..polygon.len()).collect();
    let mut triangles = Vec::new();
    while indices.len() > 3 {
        let ear = (0..indices.len()).find(|&i| {
            let a = indices[(i + indices.len() - 1) % indices.len()];
            let b = indices[i];
            let c = indices[(i + 1) % indices.len()];
            cross(polygon[a], polygon[b], polygon[c]) > 0.0001
                && !indices.iter().copied().any(|n| {
                    n != a
                        && n != b
                        && n != c
                        && cross(polygon[a], polygon[b], polygon[n]) >= -0.0001
                        && cross(polygon[b], polygon[c], polygon[n]) >= -0.0001
                        && cross(polygon[c], polygon[a], polygon[n]) >= -0.0001
                })
        });
        if let Some(i) = ear {
            triangles.push([
                indices[(i + indices.len() - 1) % indices.len()],
                indices[i],
                indices[(i + 1) % indices.len()],
            ]);
            indices.remove(i);
        } else {
            // Remove a redundant collinear vertex before considering this invalid.
            if let Some(i) = (0..indices.len()).find(|&i| {
                cross(
                    polygon[indices[(i + indices.len() - 1) % indices.len()]],
                    polygon[indices[i]],
                    polygon[indices[(i + 1) % indices.len()]],
                )
                .abs()
                    < 0.0001
            }) {
                indices.remove(i);
            } else {
                break;
            }
        }
    }
    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }
    triangles
}

#[cfg(test)]
pub(super) fn build(b: &mut Builder, city: &City) {
    ground_mesh(b, city);
    buildings(b, city, true);
}
pub(super) fn buildings(b: &mut Builder, city: &City, detailed: bool) {
    for building in &city.buildings {
        building_mesh(b, building, city, detailed);
    }
}
pub(super) fn ground_mesh(b: &mut Builder, city: &City) {
    let field = city
        .elevation
        .as_ref()
        .expect("city rendering requires elevation");
    for row in 0..field.rows - 1 {
        for col in 0..field.columns - 1 {
            let mut p = [
                field.point(col, row),
                field.point(col + 1, row),
                field.point(col + 1, row + 1),
                field.point(col, row + 1),
            ];
            for v in &mut p {
                v[1] -= 0.02;
            }
            for t in [[0, 1, 2], [0, 2, 3]] {
                b.upward_quad(
                    [p[t[0]], p[t[1]], p[t[2]], p[t[0]]],
                    Surface::road([0.64, 0.63, 0.54]),
                );
            }
        }
    }
    let ground_colors = [
        [0.39, 0.48, 0.25],
        [0.43, 0.53, 0.30],
        [0.40, 0.46, 0.28],
        [0.40, 0.42, 0.43],
        [0.68, 0.69, 0.66],
        [0.72, 0.73, 0.70],
        [0.62, 0.58, 0.46],
        [0.29, 0.48, 0.35],
        [0.21, 0.57, 0.67],
    ];
    for triangle in &city.landscape.ground {
        field.drape(triangle.points, |mut p| {
            for v in &mut p {
                v[1] += 0.005;
            }
            b.upward_quad(
                [p[0], p[1], p[2], p[0]],
                Surface::road(ground_colors[triangle.style]),
            );
        });
    }
    for tree in &city.landscape.trees {
        mapped_tree(b, city, tree);
    }
    let surfaces = city
        .surfaces
        .as_ref()
        .expect("city rendering requires compiled street surfaces");
    // The compiler partitions these surfaces, so gutters and street classes
    // meet along shared edges rather than overlapping coplanar strips.
    let colors = crate::street_surface::COLORS;
    let pavement = surfaces.pavement_height;
    let heights = [0.05, 0.05, 0.05, 0.05, 0.07, pavement, pavement, 0.08];
    for triangle in &surfaces.triangles {
        let style = triangle.style;
        field.drape(triangle.points, |mut p| {
            for v in &mut p {
                v[1] += heights[style];
            }
            b.upward_quad([p[0], p[1], p[2], p[0]], Surface::road(colors[style]));
        });
    }
    for curb in &surfaces.curbs {
        let [a, c] = curb.ends;
        let top = if curb.style == 2 { 0.08 } else { pavement };
        for pair in city.ground_segment(a, c).windows(2) {
            let a = pair[0];
            let c = pair[1];
            b.quad(
                [
                    [a[0], a[1] + 0.05, a[2]],
                    [c[0], c[1] + 0.05, c[2]],
                    [c[0], c[1] + top, c[2]],
                    [a[0], a[1] + top, a[2]],
                ],
                Surface::matte([0.77, 0.75, 0.66]),
            );
        }
    }
    for feature in &city.furniture {
        super::road_furniture::build(b, feature);
    }
}

// Stable independent samples keep neighbouring OSM IDs from repeating a pattern.
fn visual_seed(id: u64, salt: u64) -> u64 {
    let mut value = id.wrapping_add(salt.wrapping_mul(0x9e3779b97f4a7c15));
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
    value ^ (value >> 31)
}
fn variation(id: u64, salt: u64) -> f32 {
    (visual_seed(id, salt) >> 40) as f32 / 16777215.0
}
fn tinted(color: [f32; 3], gain: f32) -> [f32; 3] {
    color.map(|c| (c * gain).clamp(0.0, 1.0))
}

fn building_mesh(b: &mut Builder, building: &Building, city: &City, detailed: bool) {
    // These are regional art variations, rather than surveyed OSM materials.
    let palette = [
        [0.94, 0.93, 0.90], // white concrete
        [0.94, 0.90, 0.80], // cream
        [0.83, 0.77, 0.65], // beige
        [0.95, 0.90, 0.68], // pale yellow
    ];
    let id = building.id;
    let seed = visual_seed(id, 1);
    let wall_color = tinted(
        palette[seed as usize % palette.len()],
        0.95 + variation(id, 2) * 0.08,
    );
    let wall = Surface::matte(wall_color);
    let roof = Surface::matte(
        [
            [0.64, 0.65, 0.62],
            [0.77, 0.75, 0.68],
            [0.58, 0.62, 0.63],
            [0.71, 0.64, 0.56],
            [0.69, 0.57, 0.48],
        ][visual_seed(id, 3) as usize % 5],
    );
    let height = building.base_y + building.height;
    // Generate the complete shared shell first. Near detail never changes the
    // mapped footprint, roof outline or the silhouette when the LOD switches.
    for i in 0..building.polygon.len() {
        let a = building.polygon[i];
        let c = building.polygon[(i + 1) % building.polygon.len()];
        for pair in city.ground_segment(a, c).windows(2) {
            let a = pair[0];
            let c = pair[1];
            b.quad(
                [
                    [c[0], c[1] - 0.03, c[2]],
                    [a[0], a[1] - 0.03, a[2]],
                    [a[0], height, a[2]],
                    [c[0], height, c[2]],
                ],
                wall,
            );
        }
    }
    for triangle in roof_triangles(&building.polygon) {
        let p = triangle.map(|i| [building.polygon[i][0], height, building.polygon[i][1]]);
        b.upward_quad([p[0], p[1], p[2], p[0]], roof);
    }
    if !detailed {
        return;
    }

    // Four coherent facade families: shuttered plaster, banded apartments,
    // wider modern glazing, and modest punched windows.
    let style = visual_seed(id, 4) % 4;
    let trim = Surface::matte(tinted(wall_color, if style == 2 { 0.69 } else { 0.85 }));
    let frame = Surface::matte(
        [
            [0.73, 0.71, 0.63],
            [0.33, 0.37, 0.37],
            [0.49, 0.42, 0.33],
            [0.83, 0.81, 0.73],
        ][visual_seed(id, 5) as usize % 4],
    );
    let shutter = Surface::matte(
        [
            [0.27, 0.35, 0.29],
            [0.33, 0.39, 0.43],
            [0.45, 0.31, 0.23],
            [0.54, 0.52, 0.40],
        ][visual_seed(id, 6) as usize % 4],
    );
    let spacing = if style == 2 {
        2.6
    } else {
        3.0 + variation(id, 7) * 1.4
    };
    let storey = 2.85 + variation(id, 8) * 0.55;
    let window_width = if style == 2 {
        1.7
    } else {
        0.95 + variation(id, 9) * 0.65
    };
    let window_height = 1.15 + variation(id, 10) * 0.6;
    for i in 0..building.polygon.len() {
        let a = building.polygon[i];
        let c = building.polygon[(i + 1) % building.polygon.len()];
        let length = (c[0] - a[0]).hypot(c[1] - a[1]);
        if length < 3.0 {
            continue;
        }
        let normal = [(c[1] - a[1]) / length, -(c[0] - a[0]) / length];
        // Coordinates are metres along the wall and absolute elevation.
        let mut panel =
            |left: f32, right: f32, bottom: f32, top: f32, depth: f32, surface: Surface| {
                let p = |along: f32, y: f32| {
                    [
                        a[0] + (c[0] - a[0]) * along / length + normal[0] * depth,
                        y,
                        a[1] + (c[1] - a[1]) * along / length + normal[1] * depth,
                    ]
                };
                b.quad(
                    [
                        p(right, bottom),
                        p(left, bottom),
                        p(left, top),
                        p(right, top),
                    ],
                    surface,
                );
            };
        panel(0.0, length, height - 0.24, height - 0.04, 0.018, trim);
        panel(
            0.0,
            length,
            building.base_y + 0.06,
            building.base_y + 0.40,
            0.018,
            trim,
        );
        let columns = ((length - 1.0) / spacing).floor().clamp(1.0, 20.0) as usize;
        let half = (window_width * 0.5).min(length / columns as f32 * 0.29);
        for floor in 0..((building.height / storey).floor() as usize).min(16) {
            let bottom = building.base_y + 0.85 + floor as f32 * storey;
            let top = (bottom + window_height).min(height - 0.45);
            if top <= bottom + 0.3 {
                continue;
            }
            if style == 1 && floor > 0 {
                panel(0.0, length, bottom - 0.42, bottom - 0.29, 0.02, trim);
            }
            for column in 0..columns {
                let center = (column as f32 + 0.5) * length / columns as f32;
                if floor == 0
                    && i == 0
                    && length > 4.0
                    && building.height > 3.0
                    && (center - length * 0.5).abs() < half + 0.72
                {
                    continue;
                }
                let key = id ^ ((i as u64) << 32) ^ ((floor as u64) << 16) ^ column as u64;
                let glass = Surface {
                    color: tinted([0.16, 0.24, 0.27], 0.75 + variation(key, 11) * 0.65),
                    material: [0.0, 0.45],
                };
                panel(
                    center - half - 0.07,
                    center + half + 0.07,
                    bottom - 0.07,
                    top + 0.07,
                    0.025,
                    frame,
                );
                panel(center - half, center + half, bottom, top, 0.035, glass);
                // Narrow mullions and a sill make the openings read at street level.
                if style == 2 || half > 0.68 {
                    panel(center - 0.025, center + 0.025, bottom, top, 0.042, frame);
                }
                panel(
                    center - half - 0.10,
                    center + half + 0.10,
                    bottom - 0.12,
                    bottom - 0.045,
                    0.055,
                    trim,
                );
                if style == 0 && length / columns as f32 > window_width + 0.8 {
                    for side in [-1.0, 1.0] {
                        let edge = center + side * (half + 0.24);
                        panel(edge - 0.15, edge + 0.15, bottom, top, 0.045, shutter);
                    }
                } else if variation(key, 12) > 0.70 {
                    let blind = Surface::matte(tinted(wall_color, 0.72));
                    panel(
                        center - half,
                        center + half,
                        top - (top - bottom) * (0.20 + variation(key, 13) * 0.45),
                        top,
                        0.045,
                        blind,
                    );
                }
            }
        }
        // One recessed-looking entrance, reserved for a single substantial wall.
        if i == 0 && length > 4.0 && building.height > 3.0 {
            let center = length * 0.5;
            let bottom = building.base_y + 0.02;
            panel(
                center - 0.62,
                center + 0.62,
                bottom,
                bottom + 2.25,
                0.065,
                frame,
            );
            panel(
                center - 0.50,
                center + 0.50,
                bottom,
                bottom + 2.12,
                0.075,
                shutter,
            );
            panel(
                center + 0.31,
                center + 0.35,
                bottom + 0.85,
                bottom + 1.02,
                0.085,
                trim,
            );
        }
    }
}

// Smooth ellipsoid normals round the canopy without adding triangles to the
// city-wide inferred vegetation. Trunks and building edges keep flat normals.
fn canopy_quad(
    b: &mut Builder,
    points: [[f32; 3]; 4],
    center: [f32; 3],
    vertical_scale: f32,
    color: [f32; 3],
) {
    for indices in [[0, 1, 2], [0, 2, 3]] {
        let p = indices.map(|i| points[i]);
        if p[0] == p[1] || p[1] == p[2] || p[2] == p[0] {
            continue;
        }
        let vertices = p.map(|position| {
            let delta = [
                position[0] - center[0],
                (position[1] - center[1]) * vertical_scale,
                position[2] - center[2],
            ];
            let length = delta.iter().map(|n| n * n).sum::<f32>().sqrt().max(0.00001);
            super::Vertex {
                position,
                normal: delta.map(|n| n / length),
                color,
                material: [0.0, 0.0],
            }
        });
        if let Some(chunks) = &mut b.chunks {
            chunks.push(vertices);
        } else {
            b.vertices.extend(vertices);
        }
    }
}

fn mapped_tree(b: &mut Builder, city: &City, tree: &crate::landscape::Tree) {
    let [x, z] = tree.point;
    let y = city.height([x, z]);
    let h = tree.height;
    let r = tree.radius;
    let id = tree.id;
    let inferred = id >= (1_u64 << 63);
    let family = visual_seed(id, 20) % 4;
    let rotation = variation(id, 21) * std::f32::consts::TAU;
    let lean = [
        (variation(id, 22) - 0.5) * r * 0.16,
        (variation(id, 23) - 0.5) * r * 0.16,
    ];
    let trunk_radius = (r * (0.055 + variation(id, 24) * 0.035)).clamp(0.08, 0.27);
    let bark = Surface::matte(tinted([0.38, 0.31, 0.23], 0.78 + variation(id, 25) * 0.35));
    // A tapered six-sided trunk has the same triangle budget as the old cuboid.
    for i in 0..6 {
        let a = rotation + i as f32 * std::f32::consts::TAU / 6.0;
        let c = rotation + (i + 1) as f32 * std::f32::consts::TAU / 6.0;
        let p = |angle: f32, top: bool| {
            let scale = if top { 0.55 } else { 1.0 };
            [
                x + angle.cos() * trunk_radius * scale + if top { lean[0] } else { 0.0 },
                y + if top { h * 0.65 } else { -0.05 },
                z + angle.sin() * trunk_radius * scale + if top { lean[1] } else { 0.0 },
            ]
        };
        b.quad([p(a, false), p(a, true), p(c, true), p(c, false)], bark);
    }
    // Broad, oval, upright and spreading crowns; all remain inside the mapped
    // height/radius envelope so vegetation does not spread into cleared roads.
    let rings: &[(f32, f32)] = match (inferred, family) {
        (true, 0) => &[(0.30, 0.0), (0.48, 0.92), (0.76, 0.84), (1.0, 0.0)],
        (true, 1) => &[(0.35, 0.0), (0.57, 0.78), (0.82, 0.72), (1.0, 0.0)],
        (true, 2) => &[(0.26, 0.0), (0.46, 0.53), (0.78, 0.48), (1.0, 0.0)],
        (true, _) => &[(0.40, 0.0), (0.57, 0.94), (0.78, 0.88), (0.95, 0.0)],
        (false, 0) => &[
            (0.30, 0.0),
            (0.43, 0.70),
            (0.63, 0.94),
            (0.84, 0.72),
            (1.0, 0.0),
        ],
        (false, 1) => &[
            (0.35, 0.0),
            (0.48, 0.60),
            (0.69, 0.83),
            (0.88, 0.55),
            (1.0, 0.0),
        ],
        (false, 2) => &[
            (0.26, 0.0),
            (0.39, 0.43),
            (0.61, 0.55),
            (0.83, 0.35),
            (1.0, 0.0),
        ],
        (false, _) => &[
            (0.40, 0.0),
            (0.51, 0.72),
            (0.65, 0.94),
            (0.83, 0.81),
            (0.95, 0.0),
        ],
    };
    let palette = [
        [0.22, 0.35, 0.17],
        [0.28, 0.38, 0.22],
        [0.17, 0.29, 0.16],
        [0.31, 0.40, 0.24],
        [0.24, 0.34, 0.20],
        [0.19, 0.33, 0.20],
    ];
    let base = tinted(
        palette[visual_seed(id, 26) as usize % palette.len()],
        0.89 + variation(id, 27) * 0.19,
    );
    let segments = if inferred { 6 } else { 10 };
    let oval = 0.72 + variation(id, 28) * 0.24;
    let (rotation_sin, rotation_cos) = rotation.sin_cos();
    let crown_bottom = rings.first().unwrap().0;
    let crown_top = rings.last().unwrap().0;
    let crown_center = [
        x + lean[0] * 0.2,
        y + h * (crown_bottom + crown_top) * 0.5,
        z + lean[1] * 0.2,
    ];
    let vertical_scale = (r / (h * (crown_top - crown_bottom) * 0.5)).powi(2);
    for pair in rings.windows(2) {
        for i in 0..segments {
            let p = |index: usize, ring: (f32, f32)| {
                let angle = (index % segments) as f32 * std::f32::consts::TAU / segments as f32;
                let ripple = 0.88 + variation(id ^ (index % segments) as u64, 29) * 0.12;
                let radius = r * ring.1 * ripple;
                let drift = (ring.0 - 0.3).max(0.0);
                let local = [angle.cos() * radius, angle.sin() * radius * oval];
                [
                    x + local[0] * rotation_cos - local[1] * rotation_sin + lean[0] * drift,
                    y + h * ring.0,
                    z + local[0] * rotation_sin + local[1] * rotation_cos + lean[1] * drift,
                ]
            };
            let a = p(i, pair[0]);
            let c = p(i, pair[1]);
            let d = p(i + 1, pair[1]);
            let e = p(i + 1, pair[0]);
            canopy_quad(b, [a, c, d, e], crown_center, vertical_scale, base);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn small_city() -> City {
        City::parse("CITY 2\nbounds -50 -50 50 50\nn -40 0\nn 40 0\nr 0 1 8 0 1\n").unwrap()
    }

    #[test]
    fn storefront_glyphs_read_left_to_right_on_each_outward_wall_face() {
        let label = ['F', 'J'];
        let expected: std::collections::HashSet<_> = label
            .iter()
            .enumerate()
            .flat_map(|(column, ch)| {
                crate::hud::glyph(*ch)
                    .into_iter()
                    .enumerate()
                    .flat_map(move |(row, bits)| {
                        (0..5).filter_map(move |bit| {
                            (bits & (1 << (4 - bit)) != 0)
                                .then_some(((column * 6 + bit) as i32, row as i32))
                        })
                    })
            })
            .collect();
        for [a, c] in [
            [[0.0, 0.0], [10.0, 0.0]],
            [[10.0, 0.0], [10.0, 10.0]],
            [[10.0, 10.0], [0.0, 10.0]],
            [[0.0, 10.0], [0.0, 0.0]],
        ] {
            let mut b = Builder::default();
            storefront_sign(&mut b, &label, [a, c], 0.0, 8.0);
            let normal = [(c[1] - a[1]) / 10.0, -(c[0] - a[0]) / 10.0];
            // Camera right = cross(-outward_normal, world_up).
            let right = [normal[1], -normal[0]];
            let screen_x = |v: &super::super::Vertex| {
                (v.position[0] - c[0]) * right[0] + (v.position[2] - c[1]) * right[1]
            };
            let text = &b.vertices[6..];
            let left = text.iter().map(screen_x).fold(f32::INFINITY, f32::min);
            let top_y = text
                .iter()
                .map(|v| v.position[1])
                .fold(f32::NEG_INFINITY, f32::max);
            let pixel = text[..6]
                .iter()
                .map(screen_x)
                .fold(f32::NEG_INFINITY, f32::max)
                - text[..6].iter().map(screen_x).fold(f32::INFINITY, f32::min);
            let mut actual = std::collections::HashSet::new();
            for quad in b.vertices[6..].chunks_exact(6) {
                let min_x = quad
                    .iter()
                    .map(|v| (v.position[0] - c[0]) * right[0] + (v.position[2] - c[1]) * right[1])
                    .fold(f32::INFINITY, f32::min);
                let top = quad
                    .iter()
                    .map(|v| v.position[1])
                    .fold(f32::NEG_INFINITY, f32::max);
                actual.insert((
                    ((min_x - left) / pixel).round() as i32,
                    ((top_y - top) / pixel).round() as i32,
                ));
            }
            assert_eq!(actual, expected);
            for v in b.vertices {
                assert_eq!(v.material[0], -3.0);
                assert!(v.normal[0] * normal[0] + v.normal[2] * normal[1] > 0.999);
            }
        }
    }

    #[test]
    fn varied_trees_are_repeatable_bounded_and_keep_the_dense_canopy_budget() {
        let city = small_city();
        let mut shapes = std::collections::HashSet::new();
        for id in (0..32).chain((0..32).map(|id| id | (1_u64 << 63))) {
            let tree = crate::landscape::Tree {
                id,
                point: [0.0; 2],
                height: 8.0,
                radius: 3.0,
            };
            let mut first = Builder::default();
            let mut again = Builder::default();
            mapped_tree(&mut first, &city, &tree);
            mapped_tree(&mut again, &city, &tree);
            assert_eq!(first.vertices, again.vertices);
            assert!(first.vertices.len() <= if id >= (1_u64 << 63) { 108 } else { 216 });
            for vertex in &first.vertices {
                let [x, y, z] = vertex.position;
                assert!(x.hypot(z) <= tree.radius + 0.001);
                assert!((-0.051..=tree.height).contains(&y));
                assert!((vertex.normal.iter().map(|n| n * n).sum::<f32>() - 1.0).abs() < 0.001);
                assert!(vertex.color.iter().all(|c| (0.0..=1.0).contains(c)));
            }
            shapes.insert(
                first
                    .vertices
                    .iter()
                    .map(|v| v.position.map(f32::to_bits))
                    .collect::<Vec<_>>(),
            );
        }
        assert_eq!(shapes.len(), 64, "each identity should have its own crown");
    }

    #[test]
    fn facade_variants_are_repeatable_and_preserve_the_shared_shell() {
        let city = small_city();
        let mut appearances = std::collections::HashSet::new();
        for id in 0..32 {
            let building = Building {
                id,
                height: 10.0,
                base_y: 0.0,
                polygon: vec![[-6.0, -5.0], [6.0, -5.0], [6.0, 5.0], [-6.0, 5.0]],
            };
            let mut near = Builder::default();
            let mut again = Builder::default();
            let mut far = Builder::default();
            building_mesh(&mut near, &building, &city, true);
            building_mesh(&mut again, &building, &city, true);
            building_mesh(&mut far, &building, &city, false);
            assert_eq!(near.vertices, again.vertices);
            assert_eq!(
                &near.vertices[..far.vertices.len()],
                far.vertices.as_slice()
            );
            assert!(near.vertices.len() < 2500);
            for vertex in &near.vertices {
                assert!(vertex.position.iter().all(|x| x.is_finite()));
                assert!((-0.031..=10.0).contains(&vertex.position[1]));
            }
            appearances.insert(
                near.vertices
                    .iter()
                    .map(|v| v.color.map(f32::to_bits))
                    .collect::<Vec<_>>(),
            );
        }
        assert_eq!(appearances.len(), 32);
    }

    #[test]
    fn concave_and_bundled_roofs_preserve_footprint_area() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let concave = vec![
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 1.0],
            [1.0, 1.0],
            [1.0, 4.0],
            [0.0, 4.0],
        ];
        for polygon in std::iter::once(&concave).chain(city.buildings.iter().map(|b| &b.polygon)) {
            let actual = roof_triangles(polygon)
                .iter()
                .map(|t| cross(polygon[t[0]], polygon[t[1]], polygon[t[2]]) * 0.5)
                .sum::<f64>();
            let expected = (0..polygon.len())
                .map(|i| {
                    let a = polygon[i];
                    let c = polygon[(i + 1) % polygon.len()];
                    a[0] as f64 * c[1] as f64 - c[0] as f64 * a[1] as f64
                })
                .sum::<f64>()
                * 0.5;
            assert!(
                (actual - expected).abs() < 0.25,
                "roof area {actual} != {expected}"
            );
        }
    }
    #[test]
    fn distant_buildings_keep_identical_walls_roofs_and_foundations() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let building = city
            .buildings
            .iter()
            .find(|b| {
                b.height > 6.0
                    && b.polygon
                        .windows(2)
                        .any(|p| (p[0][0] - p[1][0]).hypot(p[0][1] - p[1][1]) > 8.0)
            })
            .unwrap();
        let mut near = Builder::default();
        let mut far = Builder::default();
        building_mesh(&mut near, building, city, true);
        building_mesh(&mut far, building, city, false);
        assert!(near.vertices.len() > far.vertices.len());
        let shell = &near.vertices[..far.vertices.len()];
        assert_eq!(
            shell,
            far.vertices.as_slice(),
            "LOD may omit facade detail, never change the shell or its support height"
        );
    }

    #[test]
    fn city_geometry_is_finite_and_bounded_in_size() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let mut b = Builder::default();
        build(&mut b, city);
        let area =
            (city.bounds[1][0] - city.bounds[0][0]) * (city.bounds[1][1] - city.bounds[0][1]);
        // At most one million vertices per square kilometre at this detail level.
        let budget = area as usize;
        eprintln!(
            "City: {} vertices over {:.2} km2",
            b.vertices.len(),
            area / 1_000_000.0
        );
        assert!(
            b.vertices.len() > 100_000 && b.vertices.len() < budget,
            "{} city vertices",
            b.vertices.len()
        );
        for v in b.vertices {
            assert!(v
                .position
                .iter()
                .chain(v.normal.iter())
                .all(|n| n.is_finite()));
            assert!((v.normal.iter().map(|v| v * v).sum::<f32>() - 1.0).abs() < 0.001);
        }
    }
}

/// Runtime signs also work with existing baked city archives.
pub(super) fn storefront_signs(b: &mut Builder, city: &City) {
    for building in &city.buildings {
        let Some(name) = city.building_name(building.id) else {
            continue;
        };
        let label: Vec<_> = name.to_uppercase().chars().take(40).collect();
        if label.is_empty() {
            continue;
        }
        // Put the shopfront on the wall closest to the street network.
        let edge = (0..building.polygon.len())
            .filter_map(|i| {
                let a = building.polygon[i];
                let c = building.polygon[(i + 1) % building.polygon.len()];
                let length = (c[0] - a[0]).hypot(c[1] - a[1]);
                if length < 2.0 {
                    return None;
                }
                let midpoint = [(a[0] + c[0]) * 0.5, building.base_y, (a[1] + c[1]) * 0.5];
                let road = city.project(midpoint);
                let distance = (midpoint[0] - road.point[0]).hypot(midpoint[2] - road.point[2]);
                Some((distance, a, c))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((_, a, c)) = edge else {
            continue;
        };
        storefront_sign(b, &label, [a, c], building.base_y, building.height);
    }
}

// X runs toward the viewer's right on the outward face of the wall.
fn storefront_sign(
    b: &mut Builder,
    label: &[char],
    [a, c]: [[f32; 2]; 2],
    base_y: f32,
    height: f32,
) {
    let length = (c[0] - a[0]).hypot(c[1] - a[1]);
    let normal = [(c[1] - a[1]) / length, -(c[0] - a[0]) / length];
    let pixel = ((length - 0.6) / (label.len() as f32 * 6.0))
        .min(0.085)
        .min((height * 0.25) / 9.0);
    let width = label.len() as f32 * 6.0 * pixel;
    let left = (length - width) * 0.5;
    let top = base_y + (height - 0.3).min(3.1);
    let mut panel = |x0: f32, x1: f32, y0: f32, y1: f32, depth: f32, color| {
        let point = |x: f32, y: f32| {
            [
                c[0] + (a[0] - c[0]) * x / length + normal[0] * depth,
                y,
                c[1] + (a[1] - c[1]) * x / length + normal[1] * depth,
            ]
        };
        b.quad(
            [point(x0, y0), point(x1, y0), point(x1, y1), point(x0, y1)],
            Surface {
                color,
                material: [-3.0, 0.0],
            },
        );
    };
    panel(
        left - pixel,
        width + left + pixel,
        top - 8.0 * pixel,
        top + pixel,
        0.14,
        [0.035, 0.09, 0.10],
    );
    for (column, ch) in label.iter().copied().enumerate() {
        for (row, bits) in crate::hud::glyph(ch).iter().enumerate() {
            for bit in 0..5 {
                if bits & (1 << (4 - bit)) != 0 {
                    let x = left + (column * 6 + bit) as f32 * pixel;
                    let y = top - row as f32 * pixel;
                    panel(x, x + pixel, y - pixel, y, 0.15, [0.96, 0.93, 0.80]);
                }
            }
        }
    }
}
