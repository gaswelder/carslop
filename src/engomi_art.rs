//! Engomi reference-block study. Architectural details are inferred, not surveyed.
use super::{Builder, Surface};
use crate::city::Building;

pub fn selected(building: &Building) -> bool {
    let n = building.polygon.len() as f32;
    let x = building.polygon.iter().map(|p| p[0]).sum::<f32>() / n;
    let z = building.polygon.iter().map(|p| p[1]).sum::<f32>() / n;
    x.abs() < 140.0 && z.abs() < 140.0 && building.height < 22.0
}

/// A facade-local solid: u runs along the wall, d points outwards.
struct Facade {
    a: [f32; 2],
    tangent: [f32; 2],
    normal: [f32; 2],
}
impl Facade {
    fn point(&self, u: f32, y: f32, d: f32) -> [f32; 3] {
        [
            self.a[0] + self.tangent[0] * u + self.normal[0] * d,
            y,
            self.a[1] + self.tangent[1] * u + self.normal[1] * d,
        ]
    }
    fn solid(&self, b: &mut Builder, lo: [f32; 3], hi: [f32; 3], s: Surface) {
        let p = |u, y, d| self.point(u, y, d);
        let [u, y, d] = lo;
        let [v, z, e] = hi;
        for face in [
            [p(u, y, e), p(v, y, e), p(v, z, e), p(u, z, e)],
            [p(v, y, d), p(u, y, d), p(u, z, d), p(v, z, d)],
            [p(u, y, d), p(u, y, e), p(u, z, e), p(u, z, d)],
            [p(v, y, e), p(v, y, d), p(v, z, d), p(v, z, e)],
            [p(u, z, e), p(v, z, e), p(v, z, d), p(u, z, d)],
            [p(u, y, d), p(v, y, d), p(v, y, e), p(u, y, e)],
        ] {
            b.quad(face, s);
        }
    }
}

pub fn facade(b: &mut Builder, building: &Building, a: [f32; 2], c: [f32; 2], detailed: bool) {
    let length = (c[0] - a[0]).hypot(c[1] - a[1]);
    if length < 3.5 {
        return;
    }
    let f = Facade {
        a,
        tangent: [(c[0] - a[0]) / length, (c[1] - a[1]) / length],
        normal: [(c[1] - a[1]) / length, -(c[0] - a[0]) / length],
    };
    let base = building.base_y;
    let roof = base + building.height;
    let plaster = Surface::matte([0.88, 0.85, 0.76]);
    let trim = Surface::matte([0.94, 0.93, 0.86]);
    let shadow = Surface::matte([0.24, 0.25, 0.23]);
    let metal = Surface::matte([0.27, 0.31, 0.31]);
    let seed = building.id.wrapping_mul(2654435761);
    // Projecting concrete slab edges and parapets survive building LOD changes.
    for level in 1..=((building.height / 3.1).floor() as usize).min(7) {
        let y = (base + level as f32 * 3.1).min(roof);
        f.solid(
            b,
            [0.02, y - 0.12, 0.015],
            [length - 0.02, y + 0.035, 0.17],
            trim,
        );
    }
    f.solid(
        b,
        [0.02, roof, 0.0],
        [length - 0.02, roof + 0.38, 0.16],
        plaster,
    );
    if !detailed {
        return;
    }
    let bays = ((length - 1.0) / 3.5).floor().clamp(1.0, 20.0) as usize;
    for level in 0..((building.height / 3.1).floor() as usize).min(7) {
        let y = base + level as f32 * 3.1;
        for bay in 0..bays {
            let centre = (bay as f32 + 0.5) * length / bays as f32;
            let balcony =
                level > 0 && !(bay as u64 + seed).is_multiple_of(3) && length / bays as f32 > 3.0;
            let half = if balcony {
                1.25_f32.min(length / bays as f32 * 0.36)
            } else {
                0.78
            };
            let l = centre - half;
            let r = centre + half;
            let bottom = y + if balcony { 0.28 } else { 0.82 };
            let top = (y + 2.46).min(roof - 0.35);
            if top <= bottom {
                continue;
            }
            // Dark recessed reveal, pale jambs, sill and lintel. Glass is a plane
            // behind the projecting frame; no random dark squares on plaster.
            f.solid(
                b,
                [l - 0.09, bottom - 0.09, 0.028],
                [r + 0.09, top + 0.09, 0.05],
                shadow,
            );
            let glass = Surface {
                color: [0.20, 0.30, 0.33],
                material: [0., 0.45],
            };
            b.quad(
                [
                    f.point(l, bottom, 0.057),
                    f.point(r, bottom, 0.057),
                    f.point(r, top, 0.057),
                    f.point(l, top, 0.057),
                ],
                glass,
            );
            for x in [l - 0.075, r] {
                f.solid(
                    b,
                    [x, bottom - 0.06, 0.055],
                    [x + 0.075, top + 0.075, 0.15],
                    trim,
                );
            }
            for h in [bottom - 0.07, top] {
                f.solid(b, [l, h, 0.055], [r, h + 0.075, 0.17], trim);
            }
            f.solid(
                b,
                [centre - 0.025, bottom, 0.06],
                [centre + 0.025, top, 0.10],
                metal,
            );
            // A shallow shaded veranda frame stays inside the car collision margin.
            if balcony {
                f.solid(
                    b,
                    [l - 0.14, y + 0.12, 0.05],
                    [r + 0.14, y + 0.26, 0.39],
                    trim,
                );
                f.solid(
                    b,
                    [l - 0.14, top + 0.08, 0.05],
                    [r + 0.14, top + 0.23, 0.39],
                    trim,
                );
                f.solid(
                    b,
                    [l - 0.1, y + 1.18, 0.30],
                    [r + 0.1, y + 1.23, 0.35],
                    metal,
                );
                for k in 0..=6 {
                    let u = l + (r - l) * k as f32 / 6.0;
                    f.solid(
                        b,
                        [u - 0.012, y + 0.27, 0.31],
                        [u + 0.012, y + 1.2, 0.34],
                        metal,
                    );
                }
                f.solid(
                    b,
                    [l - 0.10, y + 0.27, 0.08],
                    [l - 0.02, y + 1.23, 0.34],
                    plaster,
                );
                f.solid(
                    b,
                    [r + 0.02, y + 0.27, 0.08],
                    [r + 0.10, y + 1.23, 0.34],
                    plaster,
                );
            } else if (seed + bay as u64).is_multiple_of(3) {
                // Roller-shutter housing, with visible horizontal slats.
                f.solid(
                    b,
                    [l - 0.08, top + 0.08, 0.05],
                    [r + 0.08, top + 0.25, 0.23],
                    plaster,
                );
                for k in 0..4 {
                    let h = top - k as f32 * 0.09;
                    f.solid(
                        b,
                        [l, h - 0.065, 0.12],
                        [r, h - 0.015, 0.15],
                        Surface::matte([0.56, 0.58, 0.53]),
                    );
                }
            }
            if bay == 0 && level == 1 && length > 7.0 {
                let u = (r + 0.35).min(length - 0.9);
                f.solid(
                    b,
                    [u, y + 0.45, 0.03],
                    [u + 0.7, y + 0.95, 0.30],
                    Surface::matte([0.77, 0.79, 0.73]),
                );
                for k in 0..5 {
                    let h = y + 0.51 + k as f32 * 0.075;
                    f.solid(b, [u + 0.07, h, 0.301], [u + 0.63, h + 0.022, 0.305], metal);
                }
            }
        }
    }
    // A narrow entrance and stone-clad pier, with actual courses rather than a texture.
    if length > 6.0 {
        let u = length * 0.5;
        f.solid(
            b,
            [u - 0.52, base + 0.04, 0.02],
            [u + 0.52, base + 2.24, 0.08],
            Surface::matte([0.24, 0.29, 0.29]),
        );
        f.solid(
            b,
            [u - 0.65, base + 2.24, 0.04],
            [u + 0.65, base + 2.39, 0.38],
            trim,
        );
        for row in 0..12 {
            for column in 0..2 {
                let x = 0.22 + column as f32 * 0.29 + (row % 2) as f32 * 0.055;
                let shade = ((row * 7 + column * 3 + seed as usize) % 5) as f32 * 0.025;
                f.solid(
                    b,
                    [x, base + row as f32 * 0.24, 0.03],
                    [x + 0.26, base + row as f32 * 0.24 + 0.215, 0.10],
                    Surface::matte([0.59 + shade, 0.56 + shade, 0.47 + shade]),
                );
            }
        }
    }
}

pub fn rooftop(b: &mut Builder, building: &Building) {
    // Select an interior point from the largest roof triangle, never the
    // arithmetic centroid of a concave footprint. Keep equipment fully inside.
    let triangles = super::roof_triangles(&building.polygon);
    let Some(t) = triangles.iter().max_by(|a, c| {
        super::cross(
            building.polygon[a[0]],
            building.polygon[a[1]],
            building.polygon[a[2]],
        )
        .total_cmp(&super::cross(
            building.polygon[c[0]],
            building.polygon[c[1]],
            building.polygon[c[2]],
        ))
    }) else {
        return;
    };
    let p: [f32; 2] =
        std::array::from_fn(|i| t.iter().map(|j| building.polygon[*j][i]).sum::<f32>() / 3.0);
    let clearance = (0..building.polygon.len())
        .map(|i| {
            let a = building.polygon[i];
            let c = building.polygon[(i + 1) % building.polygon.len()];
            let d = [c[0] - a[0], c[1] - a[1]];
            let k = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1])
                / (d[0] * d[0] + d[1] * d[1]).max(0.0001))
            .clamp(0., 1.);
            (p[0] - a[0] - k * d[0]).hypot(p[1] - a[1] - k * d[1])
        })
        .fold(f32::INFINITY, f32::min);
    if clearance < 1.7 {
        return;
    }
    let y = building.base_y + building.height;
    let [x, z] = p;
    let white = Surface::matte([0.89, 0.89, 0.81]);
    let frame = Surface::matte([0.46, 0.48, 0.47]);
    // Vertical water tank, base plinth and lid.
    b.cuboid(
        [x - 0.55, y, z - 0.55],
        [x + 0.55, y + 0.18, z + 0.55],
        frame,
    );
    for i in 0..16 {
        let a = i as f32 * std::f32::consts::TAU / 16.;
        let c = (i + 1) as f32 * std::f32::consts::TAU / 16.;
        let p = |angle: f32, h: f32| [x + angle.cos() * 0.43, h, z + angle.sin() * 0.43];
        b.quad(
            [
                p(a, y + 0.18),
                p(c, y + 0.18),
                p(c, y + 1.28),
                p(a, y + 1.28),
            ],
            white,
        );
        b.triangle([[x, y + 1.28, z], p(a, y + 1.28), p(c, y + 1.28)], white);
    }
    b.cuboid(
        [x - 0.18, y + 1.28, z - 0.18],
        [x + 0.18, y + 1.34, z + 0.18],
        frame,
    );
    // South-facing solar collector: raised north edge, pale metal frame/grid.
    let panel = Surface {
        color: [0.12, 0.23, 0.28],
        material: [0., 0.35],
    };
    for col in 0..2 {
        for row in 0..3 {
            let l = x - 0.82 + col as f32 * 0.82;
            let r = l + 0.78;
            let n = z + 0.67 + row as f32 * 0.23;
            let s = n + 0.21;
            let h = |z: f32| y + 0.60 - (z - (p[1] + 0.67)) * 0.55;
            b.quad(
                [[l, h(n), n], [r, h(n), n], [r, h(s), s], [l, h(s), s]],
                panel,
            );
        }
    }
    b.cuboid(
        [x - 0.88, y + 0.06, z + 0.63],
        [x + 0.88, y + 0.13, z + 1.42],
        frame,
    );
}
