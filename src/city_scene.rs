//! Original materials on OSM footprints. No downloaded imagery or map tiles.
use super::{Builder, Surface};
use crate::city::{Building, City};
#[path = "engomi_art.rs"]
mod engomi_art;

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
    let heights = [0.05, 0.05, 0.05, 0.05, 0.07, 0.14, 0.14, 0.08];
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
        let top = if curb.style == 2 { 0.08 } else { 0.14 };
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

fn building_mesh(b: &mut Builder, building: &Building, city: &City, detailed: bool) {
    // Untagged facade colours are regional art choices, not measured OSM colours.
    let palette = [
        [0.91, 0.91, 0.86],
        [0.83, 0.84, 0.81],
        [0.87, 0.82, 0.70],
        [0.77, 0.81, 0.82],
        [0.91, 0.87, 0.80],
        [0.82, 0.75, 0.69],
        [0.87, 0.88, 0.83],
        [0.76, 0.78, 0.74],
    ];
    let seed = building.id.wrapping_mul(2654435761) ^ (building.id >> 7);
    let wall = Surface::matte(palette[seed as usize % palette.len()]);
    let glass = Surface {
        color: [0.13, 0.21, 0.24],
        material: [0.0, 0.45],
    };
    let roof = Surface::matte(
        [
            [0.68, 0.69, 0.67],
            [0.78, 0.77, 0.71],
            [0.62, 0.64, 0.64],
            [0.72, 0.68, 0.61],
        ][(seed >> 4) as usize % 4],
    );
    let height = building.base_y + building.height;
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
        if engomi_art::selected(building) {
            engomi_art::facade(b, building, a, c, detailed);
            continue;
        }
        let length = (c[0] - a[0]).hypot(c[1] - a[1]);
        if !detailed || length < 3.0 {
            continue;
        }
        let normal = [(c[1] - a[1]) / length, -(c[0] - a[0]) / length];
        let windows = ((length - 1.0) / 3.5).floor().clamp(1.0, 20.0) as usize;
        for floor in 0..((building.height / 3.1).floor() as usize).min(12) {
            let bottom = building.base_y + 0.9 + floor as f32 * 3.1;
            let top = (bottom + 1.35).min(height - 0.35);
            if top <= bottom {
                continue;
            }
            for column in 0..windows {
                let t = (column as f32 + 0.5) / windows as f32;
                let half = (0.65 / length).min(0.3 / windows as f32);
                let p = |t: f32, y: f32| {
                    [
                        a[0] + (c[0] - a[0]) * t + normal[0] * 0.025,
                        y,
                        a[1] + (c[1] - a[1]) * t + normal[1] * 0.025,
                    ]
                };
                b.quad(
                    [
                        p(t - half, bottom),
                        p(t + half, bottom),
                        p(t + half, top),
                        p(t - half, top),
                    ],
                    glass,
                );
            }
        }
    }
    for triangle in roof_triangles(&building.polygon) {
        let p = triangle.map(|i| [building.polygon[i][0], height, building.polygon[i][1]]);
        b.upward_quad([p[0], p[1], p[2], p[0]], roof);
    }
    if engomi_art::selected(building) {
        engomi_art::rooftop(b, building);
    }
}

fn mapped_tree(b: &mut Builder, city: &City, tree: &crate::landscape::Tree) {
    let [x, z] = tree.point;
    let y = city.height([x, z]);
    let h = tree.height;
    let r = tree.radius;
    b.cuboid(
        [x - 0.12, y - 0.05, z - 0.12],
        [x + 0.12, y + h * 0.62, z + 0.12],
        Surface::matte([0.37, 0.31, 0.23]),
    );
    // Rounded, layered broadleaf crown; ring normals vary with the geometry.
    let shade = (tree.id % 5) as f32 * 0.018;
    let leaf = Surface::matte([0.24 + shade, 0.39 + shade, 0.20 + shade * 0.5]);
    // Keep the dense inferred canopy practical at city scale. Mapped trees retain
    // the existing profile; inferred crowns use fewer rings and radial segments.
    let inferred = tree.id >= (1_u64 << 63);
    let segments = if inferred { 6 } else { 10 };
    let rings: &[(f32, f32)] = if inferred {
        &[(0.34, 0.0), (0.58, 1.0), (0.82, 0.74), (1.0, 0.0)]
    } else {
        &[
            (0.34, 0.0),
            (0.45, 0.76),
            (0.66, 1.0),
            (0.86, 0.74),
            (1.0, 0.0),
        ]
    };
    for pair in rings.windows(2) {
        for i in 0..segments {
            let a = i as f32 * std::f32::consts::TAU / segments as f32;
            let c = (i + 1) as f32 * std::f32::consts::TAU / segments as f32;
            let p = |angle: f32, ring: (f32, f32)| {
                [
                    x + angle.cos() * r * ring.1,
                    y + h * ring.0,
                    z + angle.sin() * r * ring.1,
                ]
            };
            b.quad(
                [p(a, pair[0]), p(a, pair[1]), p(c, pair[1]), p(c, pair[0])],
                leaf,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn concave_and_bundled_roofs_preserve_footprint_area() {
        let city = crate::track::TrackId::Engomi.track().city().unwrap();
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
    fn reference_block_is_local_and_details_respect_collision_margin() {
        let city = crate::track::TrackId::Engomi.track().city().unwrap();
        let selected: Vec<_> = city
            .buildings
            .iter()
            .filter(|b| engomi_art::selected(b))
            .collect();
        assert!(selected.len() > 40 && selected.len() < 100);
        let mut count = 0;
        for building in selected {
            let mut mesh = Builder::default();
            building_mesh(&mut mesh, building, city, true);
            let mut coarse = Builder::default();
            building_mesh(&mut coarse, building, city, false);
            assert!(mesh.vertices.len() > coarse.vertices.len());
            let roof = building.base_y + building.height;
            for vertex in mesh.vertices {
                let p = vertex.position;
                assert!(p.iter().all(|v| v.is_finite()));
                if p[1] < roof {
                    // At most 40 cm of trim, within the existing car-radius inset.
                    let q = [p[0], p[2]];
                    let distance = (0..building.polygon.len())
                        .map(|i| {
                            let a = building.polygon[i];
                            let c = building.polygon[(i + 1) % building.polygon.len()];
                            let d = [c[0] - a[0], c[1] - a[1]];
                            let t = (((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1])
                                / (d[0] * d[0] + d[1] * d[1]).max(0.0001))
                            .clamp(0.0, 1.0);
                            (q[0] - a[0] - t * d[0]).hypot(q[1] - a[1] - t * d[1])
                        })
                        .fold(f32::INFINITY, f32::min);
                    assert!(building.contains(q) || distance <= 0.401);
                }
            }
            count += coarse
                .vertices
                .iter()
                .filter(|v| v.position[1] > roof + 0.5)
                .count();
        }
        assert!(count > 0, "roof equipment must survive coarse LOD");
    }

    #[test]
    fn distant_buildings_keep_identical_walls_roofs_and_foundations() {
        let city = crate::track::TrackId::Engomi.track().city().unwrap();
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
        building_mesh(&mut near, building, &city, true);
        building_mesh(&mut far, building, &city, false);
        assert!(near.vertices.len() > far.vertices.len());
        let shell: Vec<_> = near
            .vertices
            .into_iter()
            .filter(|v| v.material != [0.0, 0.45])
            .collect();
        assert_eq!(
            shell, far.vertices,
            "LOD may omit windows, never change the shell or its support height"
        );
    }

    #[test]
    fn city_geometry_is_finite_and_bounded_in_size() {
        let city = crate::track::TrackId::Engomi.track().city().unwrap();
        let mut b = Builder::default();
        build(&mut b, &city);
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
