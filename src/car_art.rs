//! Sculpted procedural car surfaces. Shared mounts and physics remain in cars.rs.
use super::{Builder, Surface};
use crate::cars::{CarModel, WHEEL_RADIUS};
use std::f32::consts::{PI, TAU};

// Sample the actual surface derivatives: subdividing flat faces alone cannot
// improve either the silhouette or the reflected highlight across a panel.
fn patch(
    b: &mut Builder,
    nu: usize,
    nv: usize,
    reverse: bool,
    point: impl Fn(f32, f32) -> [f32; 3],
    material: impl Fn(f32, f32) -> Surface,
) {
    let sample = |u: f32, v: f32| {
        let p = point(u, v);
        let (ua, ub) = ((u - 0.0005).max(0.0), (u + 0.0005).min(1.0));
        let (va, vb) = ((v - 0.0005).max(0.0), (v + 0.0005).min(1.0));
        let a = point(ua, v);
        let c = point(ub, v);
        let d = point(u, va);
        let e = point(u, vb);
        let du: [f32; 3] = std::array::from_fn(|i| c[i] - a[i]);
        let dv: [f32; 3] = std::array::from_fn(|i| e[i] - d[i]);
        let n = [
            dv[1] * du[2] - dv[2] * du[1],
            dv[2] * du[0] - dv[0] * du[2],
            dv[0] * du[1] - dv[1] * du[0],
        ];
        let length = n.iter().map(|x| x * x).sum::<f32>().sqrt().max(1e-30);
        (p, n.map(|x| x / length * if reverse { -1.0 } else { 1.0 }))
    };
    let vertices: Vec<_> = (0..=nu)
        .flat_map(|i| (0..=nv).map(move |j| (i, j)))
        .map(|(i, j)| sample(i as f32 / nu as f32, j as f32 / nv as f32))
        .collect();
    for i in 0..nu {
        for j in 0..nv {
            let ids = [
                i * (nv + 1) + j,
                (i + 1) * (nv + 1) + j,
                (i + 1) * (nv + 1) + j + 1,
                i * (nv + 1) + j + 1,
            ];
            let surface = material((i as f32 + 0.5) / nu as f32, (j as f32 + 0.5) / nv as f32);
            for mut tri in [[ids[0], ids[2], ids[1]], [ids[0], ids[3], ids[2]]] {
                if reverse {
                    tri.swap(1, 2);
                }
                let start = b.vertices.len();
                b.triangle(tri.map(|id| vertices[id].0), surface);
                if b.vertices.len() == start + 3 {
                    for (vertex, id) in b.vertices[start..].iter_mut().zip(tri) {
                        let normal = vertices[id].1;
                        // Tiny mirror/lug pole derivatives can round to zero
                        // in f32; retain the valid face normal in that case.
                        if normal.iter().map(|n| n * n).sum::<f32>() > 0.9 {
                            vertex.normal = normal;
                        }
                    }
                }
            }
        }
    }
}

// Cubic Hermite profiles preserve authored stations while rounding the joins.
fn profile(stations: &[[f32; 4]], z: f32) -> [f32; 3] {
    let i = stations
        .windows(2)
        .position(|p| z <= p[1][0])
        .unwrap_or(stations.len() - 2);
    let a = stations[i];
    let c = stations[i + 1];
    let span = c[0] - a[0];
    let t = ((z - a[0]) / span).clamp(0.0, 1.0);
    std::array::from_fn(|axis| {
        let k = axis + 1;
        let prev = stations[i.saturating_sub(1)];
        let next = stations[(i + 2).min(stations.len() - 1)];
        let ma = (c[k] - prev[k]) / (c[0] - prev[0]);
        let mc = (next[k] - a[k]) / (next[0] - a[0]);
        (2.0 * t * t * t - 3.0 * t * t + 1.0) * a[k]
            + (t * t * t - 2.0 * t * t + t) * span * ma
            + (-2.0 * t * t * t + 3.0 * t * t) * c[k]
            + (t * t * t - t * t) * span * mc
    })
}

const BODY: [[f32; 4]; 11] = [
    // Z, half width, centre deck height, shoulder height
    [-2.06, 0.78, 0.13, 0.17],
    [-1.88, 0.98, 0.25, 0.31],
    [-1.30, 1.08, 0.34, 0.51],
    [-0.82, 1.02, 0.37, 0.44],
    [-0.35, 0.98, 0.36, 0.40],
    [0.35, 0.98, 0.31, 0.39],
    [0.80, 1.03, 0.25, 0.43],
    [1.30, 1.08, 0.20, 0.51],
    [1.65, 1.00, 0.12, 0.37],
    [1.93, 0.85, 0.04, 0.17],
    [2.08, 0.66, -0.03, 0.02],
];

fn deck(x: f32, z: f32) -> [f32; 3] {
    let [width, center, shoulder] = profile(&BODY, z);
    [
        x * width,
        center + (shoulder - center) * (x.abs() * PI * 0.5).sin().powi(2) + 0.025 * (1.0 - x * x),
        z,
    ]
}

fn opening(z: f32) -> f32 {
    let radius = WHEEL_RADIUS + 0.045;
    CarModel::Roadster
        .wheel_centers()
        .iter()
        .map(|c| {
            let d = (z - c[2]).abs();
            if d < radius {
                (radius * radius - d * d).sqrt()
            } else {
                -0.20
            }
        })
        .fold(-0.20, f32::max)
}

fn ellipsoid(b: &mut Builder, center: [f32; 3], radius: [f32; 3], surface: Surface) {
    let small = radius.iter().all(|r| *r < 0.06);
    patch(
        b,
        if small { 8 } else { 24 },
        if small { 6 } else { 12 },
        true,
        |u, v| {
            let a = u * TAU;
            let t = 0.001 + v * (PI - 0.002);
            [
                center[0] + radius[0] * t.sin() * a.cos(),
                center[1] + radius[1] * t.cos(),
                center[2] + radius[2] * t.sin() * a.sin(),
            ]
        },
        |_, _| surface,
    );
}

fn flank(side: f32, z: f32, y: f32) -> [f32; 3] {
    let [w, _, top] = profile(&BODY, z);
    let bottom = opening(z).min(top - 0.015);
    let v = ((y - bottom) / (top - bottom)).clamp(0.0, 1.0);
    [
        side * (w - 0.10 * (1.0 - v).powi(2) + 0.025 * (PI * v).sin()),
        y,
        z,
    ]
}

pub(super) fn kestrel(b: &mut Builder, paint: Surface, glass: Surface, trim: Surface) {
    let bright = Surface {
        color: [0.78, 0.80, 0.76],
        material: [0.0, 0.85],
    };
    patch(
        b,
        32,
        96,
        false,
        |u, v| deck(u * 2.0 - 1.0, -2.06 + v * 4.14),
        |_, _| paint,
    );
    for side in [-1.0, 1.0] {
        patch(
            b,
            128,
            12,
            side < 0.0,
            |u, v| {
                let z = -2.06 + u * 4.14;
                let [w, _, top] = profile(&BODY, z);
                let bottom = opening(z).min(top - 0.015);
                [
                    side * (w - 0.10 * (1.0 - v).powi(2) + 0.025 * (PI * v).sin()),
                    bottom + (top - bottom) * v,
                    z,
                ]
            },
            |_, _| paint,
        );
        // Recessed wheel-well walls and a rolled fender lip, sharing the
        // axle coordinates rather than using a decorative polygon over a tire.
        for axle in [-1.3, 1.3] {
            patch(
                b,
                64,
                4,
                side > 0.0,
                |u, v| {
                    let a = -0.46 + u * (PI + 0.92);
                    let r = WHEEL_RADIUS + 0.045 + v * 0.022;
                    let z = axle + r * a.cos();
                    let mut p = flank(side, z, r * a.sin());
                    p[0] += side * 0.003;
                    p
                },
                |_, _| paint,
            );
            patch(
                b,
                64,
                4,
                side < 0.0,
                |u, v| {
                    let a = -0.46 + u * (PI + 0.92);
                    let r = WHEEL_RADIUS + 0.045;
                    let z = axle + r * a.cos();
                    let [w, _, _] = profile(&BODY, z);
                    [side * (w - 0.10 - v * 0.16), r * a.sin(), z]
                },
                |_, _| trim,
            );
        }
    }
    for side in [-1.0, 1.0] {
        for edge in [-0.72, 0.74] {
            patch(
                b,
                2,
                32,
                side < 0.0,
                |u, v| {
                    let z = edge + (u - 0.5) * 0.014 + 0.025 * (PI * v).sin();
                    let top = profile(&BODY, z)[2] - 0.025;
                    let mut p = flank(side, z, -0.105 + v * (top + 0.105));
                    p[0] += side * 0.003;
                    p
                },
                |_, _| trim,
            );
        }
        patch(
            b,
            48,
            2,
            side < 0.0,
            |u, v| {
                let z = -0.72 + u * 1.46;
                let mut p = flank(side, z, -0.105 + (v - 0.5) * 0.014);
                p[0] += side * 0.003;
                p
            },
            |_, _| trim,
        );
    }
    // Rounded front/rear valances: the deck boundary closes into the underbody.
    for (z, reverse) in [(-2.06, false), (2.08, true)] {
        patch(
            b,
            40,
            12,
            reverse,
            |u, v| {
                let t = deck(u * 2.0 - 1.0, z);
                [
                    t[0] * (0.93 + 0.07 * v),
                    -0.23 + (t[1] + 0.23) * v,
                    z + (if z < 0.0 { 0.04 } else { -0.04 }) * (1.0 - v).powi(2),
                ]
            },
            |_, _| paint,
        );
    }

    // One continuous compound-curved greenhouse. Window boundaries follow
    // surface coordinates, so pillars and roof stay attached in every view.
    let cabin = [
        [-1.38, 0.70, 0.02, 0.0],
        [-1.10, 0.72, 0.22, 0.0],
        [-0.64, 0.69, 0.50, 0.0],
        [-0.20, 0.68, 0.54, 0.0],
        [0.20, 0.69, 0.49, 0.0],
        [0.62, 0.72, 0.24, 0.0],
        [0.91, 0.73, 0.02, 0.0],
    ];
    patch(
        b,
        48,
        64,
        false,
        |u, v| {
            let z = -1.38 + v * 2.29;
            let [w, h, _] = profile(&cabin, z);
            let q = u * 2.0 - 1.0;
            let x = w * q;
            let base = deck(x / profile(&BODY, z)[0], z)[1] + 0.012;
            [x, base + h * (1.0 - 0.24 * q * q - 0.76 * q.powi(6)), z]
        },
        |u, v| {
            let z = -1.38 + v * 2.29;
            let across = (u - 0.5).abs() * 2.0;
            let rear = (-1.25..-0.72).contains(&z) && across < 0.78;
            let front = (0.28..0.79).contains(&z) && across < 0.80;
            let side_window = (-0.94..0.49).contains(&z) && across > 0.62 && across < 0.96;
            let pillar = (z + 0.30).abs() < 0.028;
            if rear || front || (side_window && !pillar) {
                glass
            } else if across > 0.96 || (side_window && pillar) {
                trim
            } else {
                paint
            }
        },
    );
    for side in [-1.0, 1.0] {
        // Handles, stemmed mirror housings and shallow bumperettes.
        ellipsoid(b, [side * 1.005, 0.23, -0.30], [0.025, 0.025, 0.12], bright);
        ellipsoid(b, [side * 0.82, 0.49, 0.45], [0.16, 0.025, 0.025], bright);
        ellipsoid(b, [side * 0.98, 0.52, 0.45], [0.12, 0.075, 0.09], paint);
        ellipsoid(b, [side * 0.98, 0.52, 0.385], [0.105, 0.052, 0.012], bright);
        // Headlamps have shaped buckets, a metal bezel and a convex lens.
        ellipsoid(b, [side * 0.78, 0.25, 1.68], [0.20, 0.18, 0.30], paint);
        ellipsoid(b, [side * 0.78, 0.25, 1.989], [0.172, 0.153, 0.018], bright);
        ellipsoid(
            b,
            [side * 0.78, 0.25, 2.007],
            [0.150, 0.130, 0.018],
            Surface {
                color: [1.0, 0.94, 0.72],
                material: [0.0, 0.85],
            },
        );
        ellipsoid(
            b,
            [side * 0.59, 0.08, -2.055],
            [0.28, 0.065, 0.035],
            Surface::matte([0.78, 0.02, 0.015]),
        );
        ellipsoid(b, [side * 0.58, -0.12, 2.02], [0.31, 0.055, 0.075], trim);
        ellipsoid(b, [side * 0.58, -0.12, -2.04], [0.31, 0.055, 0.075], trim);
    }
    // Cooling slots cut a readable rhythm into the rear deck.
    for i in 0..10 {
        let z = -1.87 + i as f32 * 0.042;
        patch(
            b,
            20,
            1,
            false,
            |u, v| {
                let mut p = deck((u * 2.0 - 1.0) * 0.48, z + v * 0.018);
                p[1] += 0.003;
                p
            },
            |_, _| trim,
        );
    }
    b.cuboid(
        [-0.23, -0.17, -2.09],
        [0.23, -0.055, -2.065],
        Surface::matte([0.86, 0.82, 0.64]),
    );
}

pub(super) fn wheel(b: &mut Builder, x: f32, z: f32) {
    let rubber = Surface::matte([0.035, 0.04, 0.045]);
    let alloy = Surface {
        color: [0.65, 0.68, 0.67],
        material: [0.0, 0.85],
    };
    // Rounded tread and sidewalls, rather than a twelve-sided cylinder.
    patch(
        b,
        48,
        16,
        true,
        |u, v| {
            let a = u * TAU;
            let t = -PI * 0.5 + v * PI;
            let r = 0.255 + (WHEEL_RADIUS - 0.255) * t.cos();
            [x + 0.16 * t.sin(), r * a.cos(), z + r * a.sin()]
        },
        |_, _| rubber,
    );
    for side in [-1.0, 1.0] {
        patch(
            b,
            64,
            8,
            side < 0.0,
            |u, v| {
                let a = u * TAU;
                let r = 0.22 + v * 0.045;
                [
                    x + side * (0.158 + 0.012 * (v * PI).sin()),
                    r * a.cos(),
                    z + r * a.sin(),
                ]
            },
            |_, _| alloy,
        );
        let dark = Surface::matte([0.25, 0.26, 0.27]);
        patch(
            b,
            48,
            8,
            side < 0.0,
            |u, v| {
                let a = u * TAU;
                let r = 0.001 + 0.22 * v;
                [x + side * 0.10, r * a.cos(), z + r * a.sin()]
            },
            |_, _| dark,
        );
        for i in 0..5 {
            let a = i as f32 * TAU / 5.0;
            let point = |r: f32, offset: f32, depth: f32| {
                [
                    x + side * depth,
                    r * (a + offset).cos(),
                    z + r * (a + offset).sin(),
                ]
            };
            b.quad(
                [
                    point(0.065, -0.42, 0.175),
                    point(0.235, -0.16, 0.16),
                    point(0.235, 0.16, 0.16),
                    point(0.065, 0.42, 0.175),
                ],
                alloy,
            );
            ellipsoid(
                b,
                [x + side * 0.18, 0.043 * a.cos(), z + 0.043 * a.sin()],
                [0.009, 0.012, 0.012],
                alloy,
            );
        }
        ellipsoid(b, [x + side * 0.17, 0.0, z], [0.018, 0.055, 0.055], alloy);
    }
}
