use super::{
    building_assets::{Catalog, Tints},
    Builder, Surface,
};
use crate::city::Building;
use crate::geometry::{XYZ, XZ};
use crate::world::Location;

fn cross(a: XZ<f32>, b: XZ<f32>, c: XZ<f32>) -> f64 {
    (b.x as f64 - a.x as f64) * (c.z as f64 - a.z as f64)
        - (b.z as f64 - a.z as f64) * (c.x as f64 - a.x as f64)
}
/// Ear clipping preserves concave footprints rather than filling their courtyards.
fn roof_triangles(polygon: &[XZ<f32>]) -> Vec<[usize; 3]> {
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

pub(super) fn buildings(b: &mut Builder, city: &Location, detailed: bool) -> std::io::Result<()> {
    let models = Catalog::load_default()?;
    for building in city.buildings() {
        building_mesh(b, building, city, detailed, &models);
    }
    Ok(())
}
fn terrain_mesh(
    b: &mut Builder,
    field: &crate::elevation::HeightField,
    boundary: Option<&geo::MultiPolygon<f64>>,
) {
    use geo::{BooleanOps, Contains, Intersects, TriangulateEarcut};
    for row in 0..field.rows - 1 {
        for col in 0..field.columns - 1 {
            let mut p = [
                field.point(col, row),
                field.point(col + 1, row),
                field.point(col + 1, row + 1),
                field.point(col, row + 1),
            ];
            for v in &mut p {
                v.y -= 0.02;
            }
            if let Some(boundary) = boundary {
                let cell = crate::geometry::polygon(&p.map(|v| XZ {
                    x: v.x as f64,
                    z: v.z as f64,
                }));
                if !boundary.intersects(&cell) {
                    continue;
                }
                if !boundary.contains(&cell) {
                    // Preserve the height field's diagonal and its two planar faces.
                    for indices in [[0, 1, 2], [0, 2, 3]] {
                        let face = crate::geometry::polygon(&indices.map(|i| XZ {
                            x: p[i].x as f64,
                            z: p[i].z as f64,
                        }));
                        for polygon in boundary.intersection(&face) {
                            for triangle in polygon.earcut_triangles() {
                                let points =
                                    [triangle.v1(), triangle.v2(), triangle.v3()].map(|c| {
                                        let x = c.x as f32;
                                        let z = c.y as f32;
                                        XYZ {
                                            x,
                                            y: field.height(XZ { x, z }) - 0.02,
                                            z,
                                        }
                                    });
                                b.upward_quad(
                                    [(points[0]), (points[1]), (points[2]), (points[0])],
                                    Surface::road([0.64, 0.63, 0.54]),
                                );
                            }
                        }
                    }
                    continue;
                }
            }
            for t in [[0, 1, 2], [0, 2, 3]] {
                b.upward_quad(
                    [p[t[0]], p[t[1]], p[t[2]], p[t[0]]],
                    Surface::road([0.64, 0.63, 0.54]),
                );
            }
        }
    }
}
pub(super) fn ground_mesh(
    b: &mut Builder,
    city: &Location,
    boundary: Option<&geo::MultiPolygon<f64>>,
) {
    let field = city.elevation().expect("city rendering requires elevation");
    terrain_mesh(b, field, boundary);
    for triangle in city.ground_triangles() {
        field.drape(triangle.points.map(Into::into), |mut p| {
            for v in &mut p {
                v.y += 0.005;
            }
            b.upward_quad(
                [p[0], p[1], p[2], p[0]],
                Surface::road(triangle.material.color()),
            );
        });
    }
    for tree in city.trees() {
        mapped_tree(b, city, tree);
    }
    assert!(
        city.has_compiled_roads(),
        "city rendering requires compiled street surfaces"
    );
    // The compiler partitions these surfaces, so gutters and street classes
    // meet along shared edges rather than overlapping coplanar strips.
    let pavement = city.pavement_height_meters();
    for triangle in city.road_triangles() {
        let height = triangle.usage.render_height(pavement);
        field.drape(triangle.points.map(Into::into), |mut p| {
            for v in &mut p {
                v.y += height;
            }
            b.upward_quad(
                [p[0], p[1], p[2], p[0]],
                Surface::road(triangle.material.color()),
            );
        });
    }
    for curb in city.curbs() {
        let [a, c] = curb.ends;
        let top = if curb.style == crate::world::CurbStyle::ServiceRoad {
            0.08
        } else {
            pavement
        };
        for pair in city.ground_segment(a, c).windows(2) {
            let a = pair[0];
            let c = pair[1];
            b.quad(
                [
                    XYZ {
                        x: a.x,
                        y: a.y + 0.05,
                        z: a.z,
                    },
                    XYZ {
                        x: c.x,
                        y: c.y + 0.05,
                        z: c.z,
                    },
                    XYZ {
                        x: c.x,
                        y: c.y + top,
                        z: c.z,
                    },
                    XYZ {
                        x: a.x,
                        y: a.y + top,
                        z: a.z,
                    },
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

fn building_mesh(
    b: &mut Builder,
    building: &Building,
    city: &Location,
    detailed: bool,
    models: &Catalog,
) {
    // These are regional art variations, rather than surveyed OSM materials.
    let nicosia = city.origin.is_some_and(|origin| {
        (35.10..35.24).contains(&origin.lat) && (33.24..33.48).contains(&origin.lon)
    });
    let palette = if nicosia {
        [
            [0.97, 0.96, 0.94], // white plaster
            [0.95, 0.94, 0.91], // warm white concrete
            [0.91, 0.88, 0.81], // pale limestone
            [0.96, 0.93, 0.86], // ivory
        ]
    } else {
        [
            [0.94, 0.93, 0.90], // white concrete
            [0.94, 0.90, 0.80], // cream
            [0.83, 0.77, 0.65], // beige
            [0.95, 0.90, 0.68], // pale yellow
        ]
    };
    let id = building.id;
    let seed = visual_seed(id, 1);
    let wall_color = tinted(
        palette[seed as usize % palette.len()],
        0.95 + variation(id, 2) * 0.08,
    );
    let wall = Surface::matte(wall_color);
    let roof = Surface::matte(if nicosia {
        tinted([0.86, 0.85, 0.81], 0.94 + variation(id, 3) * 0.08)
    } else {
        [
            [0.64, 0.65, 0.62],
            [0.77, 0.75, 0.68],
            [0.58, 0.62, 0.63],
            [0.71, 0.64, 0.56],
            [0.69, 0.57, 0.48],
        ][visual_seed(id, 3) as usize % 5]
    });
    let height = building.base_y + building.height_meters;
    // Generate the complete shared shell first. Near detail never changes the
    // mapped footprint and roof outline. Balcony furniture is near detail.
    for i in 0..building.ground_outline.len() {
        let a = building.ground_outline[i];
        let c = building.ground_outline[(i + 1) % building.ground_outline.len()];
        for pair in city.ground_segment(a, c).windows(2) {
            let a = pair[0];
            let c = pair[1];
            b.quad(
                [
                    XYZ {
                        x: c.x,
                        y: c.y - 0.03,
                        z: c.z,
                    },
                    XYZ {
                        x: a.x,
                        y: a.y - 0.03,
                        z: a.z,
                    },
                    XYZ {
                        x: a.x,
                        y: height,
                        z: a.z,
                    },
                    XYZ {
                        x: c.x,
                        y: height,
                        z: c.z,
                    },
                ],
                wall,
            );
        }
    }
    for triangle in roof_triangles(&building.ground_outline) {
        let p = triangle.map(|i| building.ground_outline[i].at_height(height));
        b.upward_quad([(p[0]), (p[1]), (p[2]), (p[0])], roof);
    }
    if !detailed {
        return;
    }

    // Four coherent facade families: shuttered plaster, banded apartments,
    // wider modern glazing, and modest punched windows.
    let style = visual_seed(id, 4) % 4;
    let trim = Surface::matte(tinted(
        wall_color,
        if nicosia {
            0.97
        } else if style == 2 {
            0.69
        } else {
            0.85
        },
    ));
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
        models.placement.modern_window_spacing
    } else {
        let [min, max] = models.placement.window_spacing;
        min + variation(id, 7) * (max - min)
    };
    let storey = if nicosia {
        // OSM heights commonly use 3.1 m per level. Fit complete storeys to the
        // measured height so rounding cannot leave a blank top floor.
        building.height_meters / (building.height_meters / 3.1).round().max(1.0)
    } else {
        2.85 + variation(id, 8) * 0.55
    };
    let window_width = if style == 2 {
        1.7
    } else {
        0.95 + variation(id, 9) * 0.65
    };
    let window_height = if nicosia && (style == 1 || style == 2) {
        1.95
    } else {
        1.15 + variation(id, 10) * 0.6
    };
    let mut tints = Tints {
        wall: wall_color,
        frame: frame.color,
        trim: trim.color,
        shutter: shutter.color,
        glass: [1.0; 3],
    };
    for i in 0..building.ground_outline.len() {
        let a = building.ground_outline[i];
        let c = building.ground_outline[(i + 1) % building.ground_outline.len()];
        let length = (c.x - a.x).hypot(c.z - a.z);
        if length < 3.0 {
            continue;
        }
        let normal = XZ {
            x: (c.z - a.z) / length,
            z: -(c.x - a.x) / length,
        };
        // Coordinates are metres along the wall and absolute elevation.
        let panel = |b: &mut Builder,
                     left: f32,
                     right: f32,
                     bottom: f32,
                     top: f32,
                     depth: f32,
                     surface: Surface| {
            let p = |along: f32, y: f32| crate::geometry::XYZ {
                x: a.x + (c.x - a.x) * along / length + normal.x * depth,
                y: y,
                z: a.z + (c.z - a.z) * along / length + normal.z * depth,
            };
            b.quad(
                [
                    (p(right, bottom)),
                    (p(left, bottom)),
                    (p(left, top)),
                    (p(right, top)),
                ],
                surface,
            );
        };
        panel(b, 0.0, length, height - 0.24, height - 0.04, 0.018, trim);
        panel(
            b,
            0.0,
            length,
            building.base_y + 0.06,
            building.base_y + 0.40,
            0.018,
            trim,
        );
        let columns = ((length - 1.0) / spacing).floor().clamp(1.0, 20.0) as usize;
        let half = (window_width * 0.5).min(length / columns as f32 * 0.29);
        for floor in 0..((building.height_meters / storey + 0.001).floor() as usize).min(16) {
            let bottom = building.base_y + 0.85 + floor as f32 * storey;
            let top = (bottom + window_height).min(height - 0.45);
            if top <= bottom + 0.3 {
                continue;
            }
            if style == 1 && floor > 0 {
                panel(b, 0.0, length, bottom - 0.42, bottom - 0.29, 0.02, trim);
            }
            for column in 0..columns {
                let center = (column as f32 + 0.5) * length / columns as f32;
                if floor == 0
                    && i == 0
                    && length > models.placement.entrance_min_wall
                    && building.height_meters > models.placement.entrance_min_height
                    && (center - length * 0.5).abs() < half + 0.72
                {
                    continue;
                }
                let key = id ^ ((i as u64) << 32) ^ ((floor as u64) << 16) ^ column as u64;
                tints.glass = tinted([0.16, 0.24, 0.27], 0.75 + variation(key, 11) * 0.65);
                models.window.place(
                    b,
                    [a, c],
                    center,
                    bottom - 0.07,
                    XYZ {
                        x: half * 2.0 + 0.14,
                        y: top - bottom + 0.14,
                        z: models.window.size.z,
                    },
                    &tints,
                );
                if style == 0 && length / columns as f32 > window_width + 0.8 {
                    for side in [-1.0, 1.0] {
                        let edge = center + side * (half + 0.24);
                        panel(b, edge - 0.15, edge + 0.15, bottom, top, 0.045, shutter);
                    }
                } else if variation(key, 12) > 0.70 {
                    let blind = Surface::matte(tinted(wall_color, 0.72));
                    panel(
                        b,
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
        if i == 0
            && length > models.placement.entrance_min_wall
            && building.height_meters > models.placement.entrance_min_height
        {
            let center = length * 0.5;
            let bottom = building.base_y + 0.02;
            models
                .entrance
                .place(b, [a, c], center, bottom, models.entrance.size, &tints);
        }
        // Broad stacked terraces on two substantial exterior faces. Keep them
        // off concave footprints, where an outward projection could fill a court.
        let convex = (0..building.ground_outline.len()).all(|j| {
            cross(
                building.ground_outline[j],
                building.ground_outline[(j + 1) % building.ground_outline.len()],
                building.ground_outline[(j + 2) % building.ground_outline.len()],
            ) >= -0.0001
        });
        if nicosia
            && convex
            && (style == 1 || style == 2)
            && building.height_meters >= models.placement.balcony_min_height
            && length >= models.placement.balcony_min_wall
            && i < models.placement.balcony_max_walls
        {
            apartment_balconies(b, a, c, building, storey, style, models, &tints);
        }
    }
}

fn apartment_balconies(
    b: &mut Builder,
    a: XZ<f32>,
    c: XZ<f32>,
    building: &Building,
    storey: f32,
    style: u64,
    models: &Catalog,
    tints: &Tints,
) {
    let length = (c.x - a.x).hypot(c.z - a.z);
    let rules = &models.placement;
    let width = (length * rules.balcony_width_ratio).min(rules.balcony_max_width);
    let depth = rules.balcony_depth[0]
        + variation(building.id, 21) * (rules.balcony_depth[1] - rules.balcony_depth[0]);
    let model = &models.balconies[usize::from(style == 2)];
    for floor in 1..((building.height_meters / storey + 0.001).floor() as usize).min(16) {
        let y = building.base_y + floor as f32 * storey + rules.balcony_floor_offset;
        if y + rules.balcony_headroom >= building.base_y + building.height_meters {
            continue;
        }
        model.place(
            b,
            [a, c],
            length * 0.5,
            y,
            XYZ {
                x: width,
                y: model.size.y,
                z: depth,
            },
            tints,
        );
    }
}

// Smooth ellipsoid normals round the canopy without adding triangles to the
// city-wide inferred vegetation. Trunks and building edges keep flat normals.
fn canopy_quad(
    b: &mut Builder,
    points: [XYZ; 4],
    center: XYZ,
    vertical_scale: f32,
    color: [f32; 3],
) {
    for indices in [[0, 1, 2], [0, 2, 3]] {
        let p = indices.map(|i| points[i]);
        if p[0] == p[1] || p[1] == p[2] || p[2] == p[0] {
            continue;
        }
        let vertices = p.map(|position| {
            let delta = XYZ {
                x: position.x - center.x,
                y: (position.y - center.y) * vertical_scale,
                z: position.z - center.z,
            };
            let length = delta.iter().map(|n| n * n).sum::<f32>().sqrt().max(0.00001);
            super::Vertex {
                position: (position).into(),
                normal: (delta.map(|n| n / length)).into(),
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

fn mapped_tree(b: &mut Builder, city: &Location, tree: &crate::landscape::Tree) {
    let XZ { x, z } = tree.coords;
    let y = city.height(XZ { x, z });
    let h = tree.height_meters;
    let r = tree.crown_radius_meters;
    let id = tree.id;
    let inferred = id >= (1_u64 << 63);
    let family = visual_seed(id, 20) % 4;
    let rotation = variation(id, 21) * std::f32::consts::TAU;
    let lean = XZ {
        x: (variation(id, 22) - 0.5) * r * 0.16,
        z: (variation(id, 23) - 0.5) * r * 0.16,
    };
    let trunk_radius = (r * (0.055 + variation(id, 24) * 0.035)).clamp(0.08, 0.27);
    let bark = Surface::matte(tinted([0.38, 0.31, 0.23], 0.78 + variation(id, 25) * 0.35));
    // A tapered six-sided trunk has the same triangle budget as the old cuboid.
    for i in 0..6 {
        let a = rotation + i as f32 * std::f32::consts::TAU / 6.0;
        let c = rotation + (i + 1) as f32 * std::f32::consts::TAU / 6.0;
        let p = |angle: f32, top: bool| {
            let scale = if top { 0.55 } else { 1.0 };
            crate::geometry::XYZ {
                x: x + angle.cos() * trunk_radius * scale + if top { lean.x } else { 0.0 },
                y: y + if top { h * 0.65 } else { -0.05 },
                z: z + angle.sin() * trunk_radius * scale + if top { lean.z } else { 0.0 },
            }
        };
        b.quad(
            [(p(a, false)), (p(a, true)), (p(c, true)), (p(c, false))],
            bark,
        );
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
    let crown_center = XYZ {
        x: x + lean.x * 0.2,
        y: y + h * (crown_bottom + crown_top) * 0.5,
        z: z + lean.z * 0.2,
    };
    let vertical_scale = (r / (h * (crown_top - crown_bottom) * 0.5)).powi(2);
    for pair in rings.windows(2) {
        for i in 0..segments {
            let p = |index: usize, ring: (f32, f32)| {
                let angle = (index % segments) as f32 * std::f32::consts::TAU / segments as f32;
                let ripple = 0.88 + variation(id ^ (index % segments) as u64, 29) * 0.12;
                let radius = r * ring.1 * ripple;
                let drift = (ring.0 - 0.3).max(0.0);
                let local = XZ {
                    x: angle.cos() * radius,
                    z: angle.sin() * radius * oval,
                };
                crate::geometry::XYZ {
                    x: x + local.x * rotation_cos - local.z * rotation_sin + lean.x * drift,
                    y: y + h * ring.0,
                    z: z + local.x * rotation_sin + local.z * rotation_cos + lean.z * drift,
                }
            };
            let a = p(i, pair[0]);
            let c = p(i, pair[1]);
            let d = p(i + 1, pair[1]);
            let e = p(i + 1, pair[0]);
            canopy_quad(b, [(a), (c), (d), (e)], crown_center, vertical_scale, base);
        }
    }
}

impl crate::world::TriangleUsage {
    fn render_height(self, pavement_height: f32) -> f32 {
        match self {
            Self::Marking => 0.07,
            _ => self.support_height(pavement_height),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_building_storefronts_load_and_render_separate_panels() {
        let directory = tempfile::tempdir().unwrap();
        let asset = crate::test_support::stored_location(directory.path(), true);
        let labels = crate::location::Labels {
            streets: Vec::new(),
            buildings: vec![(1, "Centre".into())],
            storefronts: vec![(1, "Centre".into()), (1, "Shop".into()), (1, "Cafe".into())],
        };
        std::fs::write(
            directory.path().join("labels.json"),
            serde_json::to_vec(&labels).unwrap(),
        )
        .unwrap();
        let city = crate::world::Location::load(&asset).unwrap();
        assert_eq!(city.storefront_names(1).len(), 3);
        let mut builder = Builder::default();
        storefront_signs(&mut builder, &city);
        assert!(builder
            .vertices
            .iter()
            .all(|v| v.position.iter().all(|n| n.is_finite())));
        let panels: Vec<_> = builder
            .vertices
            .iter()
            .filter(|v| v.color == [0.035, 0.09, 0.10])
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

    fn small_city() -> Location {
        crate::test_support::parse_world(
            "CITY 2\nbounds -50 -50 50 50\nn -40 0\nn 40 0\nr 0 1 8 0 1\n",
        )
        .unwrap()
    }

    #[test]
    fn administrative_terrain_preserves_holes_area_and_height_planes() {
        use geo::{Area, BooleanOps, MultiPolygon};
        let mut city = small_city();
        crate::test_support::set_elevation(
            &mut city,
            Some(crate::elevation::HeightField::generate(
                crate::geometry::BoundingBoxXZ {
                    min: crate::geometry::XZ { x: -8., z: -8. },
                    max: crate::geometry::XZ { x: 8., z: 8. },
                },
                |p| p.x * p.z * 0.1,
            )),
        );
        let shell = crate::geometry::polygon(
            &[[-6., -5.], [6., -5.], [6., 5.], [-6., 5.]]
                .map(|[x, z]| crate::geometry::XZ { x, z }),
        );
        let hole = crate::geometry::polygon(
            &[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]
                .map(|[x, z]| crate::geometry::XZ { x, z }),
        );
        let boundary = shell.difference(&hole);
        let mut builder = Builder::default();
        terrain_mesh(&mut builder, city.elevation().unwrap(), Some(&boundary));
        let mut area = 0.;
        for vertices in builder.vertices.as_chunks::<3>().0 {
            let triangle = crate::geometry::polygon(&vertices.map(|v| crate::geometry::XZ {
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
                        - city.elevation().unwrap().height(crate::geometry::XZ {
                            x: v.position[0],
                            z: v.position[2]
                        })
                        + 0.02)
                        .abs()
                        < 0.0001
                );
            }
        }
        assert!((area - boundary.unsigned_area()).abs() < 0.0001);
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
            [XZ { x: 0.0, z: 0.0 }, XZ { x: 10.0, z: 0.0 }],
            [XZ { x: 10.0, z: 0.0 }, XZ { x: 10.0, z: 10.0 }],
            [XZ { x: 10.0, z: 10.0 }, XZ { x: 0.0, z: 10.0 }],
            [XZ { x: 0.0, z: 10.0 }, XZ { x: 0.0, z: 0.0 }],
        ] {
            let mut b = Builder::default();
            storefront_sign(&mut b, &label, [a, c], 0.0, 8.0);
            let normal = crate::geometry::XZ {
                x: (c.z - a.z) / 10.0,
                z: -(c.x - a.x) / 10.0,
            };
            // Camera right = cross(-outward_normal, world_up).
            let right = crate::geometry::XZ {
                x: normal.z,
                z: -normal.x,
            };
            let screen_x = |v: &super::super::Vertex| {
                (v.position[0] - c.x) * right.x + (v.position[2] - c.z) * right.z
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
                    .map(|v| (v.position[0] - c.x) * right.x + (v.position[2] - c.z) * right.z)
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
                assert!(v.normal[0] * normal.x + v.normal[2] * normal.z > 0.999);
            }
        }
    }
}

/// Runtime signs also work with existing baked city archives.
pub(super) fn storefront_signs(b: &mut Builder, city: &Location) {
    for building in city.buildings() {
        let names = city.storefront_names(building.id);
        if names.is_empty() {
            continue;
        }
        // Put the shopfront on the wall closest to the street network.
        let edge = (0..building.ground_outline.len())
            .filter_map(|i| {
                let a = building.ground_outline[i];
                let c = building.ground_outline[(i + 1) % building.ground_outline.len()];
                let length = (c.x - a.x).hypot(c.z - a.z);
                if length < 2.0 {
                    return None;
                }
                let midpoint = crate::geometry::XYZ {
                    x: (a.x + c.x) * 0.5,
                    y: building.base_y,
                    z: (a.z + c.z) * 0.5,
                };
                let road = city.project(midpoint);
                let distance = (midpoint.x - road.point.x).hypot(midpoint.z - road.point.z);
                Some((distance, a, c))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((_, a, c)) = edge else {
            continue;
        };
        // Keep narrow frontages readable by stacking tenants in additional rows.
        let length = (c.x - a.x).hypot(c.z - a.z);
        let columns = names.len().min((length / 2.0).floor().max(1.0) as usize);
        let rows = names.len().div_ceil(columns);
        let height = building.height_meters / rows as f32;
        for (i, name) in names.iter().enumerate() {
            let label: Vec<_> = name.to_uppercase().chars().take(80).collect();
            let point = |t: f32| crate::geometry::XZ {
                x: a.x + (c.x - a.x) * t,
                z: a.z + (c.z - a.z) * t,
            };
            let column = i % columns;
            let start = point(column as f32 / columns as f32);
            let end = point((column + 1) as f32 / columns as f32);
            storefront_sign(
                b,
                &label,
                [start, end],
                building.base_y + (i / columns) as f32 * height,
                height,
            );
        }
    }
}

// X runs toward the viewer's right on the outward face of the wall.
fn storefront_sign(
    b: &mut Builder,
    label: &[char],
    [a, c]: [crate::geometry::XZ<f32>; 2],
    base_y: f32,
    height: f32,
) {
    let length = (c.x - a.x).hypot(c.z - a.z);
    let normal = crate::geometry::XZ {
        x: (c.z - a.z) / length,
        z: -(c.x - a.x) / length,
    };
    let (lines, pixel) = storefront_layout(label, length, height);
    let width = lines.iter().map(Vec::len).max().unwrap_or(0) as f32 * 6.0 * pixel;
    let left = (length - width) * 0.5;
    let panel_height = lines.len() as f32 * 9.0 * pixel;
    let top = base_y + (panel_height + 1.0).min(height - (height * 0.1).min(0.3));
    let mut panel = |x0: f32, x1: f32, y0: f32, y1: f32, depth: f32, color| {
        let point = |x: f32, y: f32| crate::geometry::XYZ {
            x: c.x + (a.x - c.x) * x / length + normal.x * depth,
            y: y,
            z: c.z + (a.z - c.z) * x / length + normal.z * depth,
        };
        b.quad(
            [
                (point(x0, y0)),
                (point(x1, y0)),
                (point(x1, y1)),
                (point(x0, y1)),
            ],
            Surface {
                color,
                material: [-3.0, 0.0],
            },
        );
    };
    panel(
        left - pixel,
        width + left + pixel,
        top - panel_height,
        top,
        0.14,
        [0.035, 0.09, 0.10],
    );
    for (line, chars) in lines.iter().enumerate() {
        let line_left = (length - chars.len() as f32 * 6.0 * pixel) * 0.5;
        for (column, ch) in chars.iter().copied().enumerate() {
            for (row, bits) in crate::hud::glyph(ch).iter().enumerate() {
                for bit in 0..5 {
                    if bits & (1 << (4 - bit)) != 0 {
                        let x = line_left + (column * 6 + bit) as f32 * pixel;
                        let y = top - (1 + line * 9 + row) as f32 * pixel;
                        panel(x, x + pixel, y - pixel, y, 0.15, [0.96, 0.93, 0.80]);
                    }
                }
            }
        }
    }
}

fn storefront_layout(label: &[char], length: f32, height: f32) -> (Vec<Vec<char>>, f32) {
    let wrap = |columns: usize| {
        let mut lines = Vec::new();
        let mut start = 0;
        while start < label.len() {
            while start < label.len() && label[start].is_whitespace() {
                start += 1;
            }
            if start == label.len() {
                break;
            }
            let mut end = (start + columns).min(label.len());
            if end < label.len() {
                if let Some(space) = label[start..end].iter().rposition(|c| c.is_whitespace()) {
                    if space > 0 {
                        end = start + space;
                    }
                }
            }
            lines.push(label[start..end].to_vec());
            start = end;
        }
        lines
    };
    let mut columns = ((length - 0.6) / (6.0 * 0.25)).floor().max(6.0) as usize;
    let mut lines = wrap(columns);
    while lines.len() > 3 {
        columns += 1;
        lines = wrap(columns);
    }
    let widest = lines.iter().map(Vec::len).max().unwrap_or(1) as f32;
    let pixel = 0.25_f32
        .min((length - 0.6) / (widest * 6.0 + 2.0))
        .min((height - 0.6).max(height * 0.5).min(3.8) / (lines.len().max(1) as f32 * 9.0));
    (lines, pixel)
}
