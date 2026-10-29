//! Original procedural signal heads and sign faces, placed from mapped controls.
use super::{Builder, Surface};
use crate::traffic::{Furniture, FurnitureKind};
use std::f32::consts::{PI, TAU};

fn point(f: &Furniture, x: f32, y: f32, depth: f32) -> [f32; 3] {
    let right = [-f.travel[1], f.travel[0]];
    [
        f.position[0] + right[0] * x - f.travel[0] * depth,
        f.position[1] + y,
        f.position[2] + right[1] * x - f.travel[1] * depth,
    ]
}
fn box_mesh(b: &mut Builder, f: &Furniture, min: [f32; 3], max: [f32; 3], surface: Surface) {
    let p = [
        point(f, min[0], min[1], min[2]),
        point(f, max[0], min[1], min[2]),
        point(f, max[0], max[1], min[2]),
        point(f, min[0], max[1], min[2]),
        point(f, min[0], min[1], max[2]),
        point(f, max[0], min[1], max[2]),
        point(f, max[0], max[1], max[2]),
        point(f, min[0], max[1], max[2]),
    ];
    for [a, c, d, e] in [
        [3, 2, 1, 0],
        [5, 6, 7, 4],
        [1, 5, 4, 0],
        [7, 6, 2, 3],
        [4, 7, 3, 0],
        [2, 6, 5, 1],
    ] {
        b.quad([p[a], p[c], p[d], p[e]], surface);
    }
}
fn disc(
    b: &mut Builder,
    f: &Furniture,
    y: f32,
    radius: f32,
    depth: f32,
    steps: usize,
    surface: Surface,
) {
    for i in 0..steps {
        let a = PI / 8.0 + i as f32 * TAU / steps as f32;
        let c = PI / 8.0 + (i + 1) as f32 * TAU / steps as f32;
        b.triangle(
            [
                point(f, 0.0, y, depth),
                point(f, a.cos() * radius, y + a.sin() * radius, depth),
                point(f, c.cos() * radius, y + c.sin() * radius, depth),
            ],
            surface,
        );
    }
}
fn lettering(b: &mut Builder, f: &Furniture, label: &str, y: f32, size: f32, surface: Surface) {
    let width = (label.len() * 6 - 1) as f32 * size;
    for (i, ch) in label.chars().enumerate() {
        for (row, bits) in crate::hud::glyph(ch).iter().enumerate() {
            for column in 0..5 {
                if bits & (1 << (4 - column)) != 0 {
                    let x = -width * 0.5 + (i * 6 + column) as f32 * size;
                    let top = y + 3.5 * size - row as f32 * size;
                    b.quad(
                        [
                            point(f, x, top - size * 0.9, 0.065),
                            point(f, x + size * 0.9, top - size * 0.9, 0.065),
                            point(f, x + size * 0.9, top, 0.065),
                            point(f, x, top, 0.065),
                        ],
                        surface,
                    );
                }
            }
        }
    }
}
pub(super) fn build(b: &mut Builder, f: &Furniture) {
    let metal = Surface {
        color: [0.29, 0.32, 0.32],
        material: [0.0, 0.35],
    };
    let dark = Surface::matte([0.035, 0.045, 0.045]);
    let red = Surface::matte([0.83, 0.035, 0.025]);
    let white = Surface {
        color: [0.96, 0.96, 0.90],
        material: [0.0, 0.12],
    };
    let pole_top = if f.kind == FurnitureKind::Signal {
        2.45
    } else {
        2.12
    };
    for i in 0..12 {
        let a = i as f32 * TAU / 12.0;
        let c = (i + 1) as f32 * TAU / 12.0;
        b.quad(
            [
                point(f, a.cos() * 0.045, 0.0, a.sin() * 0.045 - 0.065),
                point(f, c.cos() * 0.045, 0.0, c.sin() * 0.045 - 0.065),
                point(f, c.cos() * 0.045, pole_top, c.sin() * 0.045 - 0.065),
                point(f, a.cos() * 0.045, pole_top, a.sin() * 0.045 - 0.065),
            ],
            metal,
        );
    }
    box_mesh(b, f, [-0.09, 0.0, -0.13], [0.09, 0.16, 0.06], metal);
    match f.kind {
        FurnitureKind::Signal => {
            box_mesh(b, f, [-0.23, 2.28, -0.11], [0.23, 3.36, 0.10], metal);
            box_mesh(b, f, [-0.19, 2.32, 0.10], [0.19, 3.32, 0.14], dark);
            for (lamp, (height, color)) in [
                (3.12, [1.0, 0.025, 0.008]),
                (2.82, [1.0, 0.55, 0.015]),
                (2.52, [0.015, 0.95, 0.16]),
            ]
            .into_iter()
            .enumerate()
            {
                disc(b, f, height, 0.13, 0.151, 24, dark);
                disc(
                    b,
                    f,
                    height,
                    0.102,
                    0.166,
                    24,
                    Surface {
                        color,
                        material: [-2.0, f.phase as f32 * 4.0 + lamp as f32 + 0.25],
                    },
                );
                // Shallow visor over each lens gives the head a readable profile.
                box_mesh(
                    b,
                    f,
                    [-0.14, height + 0.11, 0.14],
                    [0.14, height + 0.14, 0.28],
                    dark,
                );
            }
        }
        FurnitureKind::Stop => {
            disc(b, f, 2.12, 0.41, -0.025, 8, metal);
            disc(b, f, 2.12, 0.40, 0.015, 8, white);
            disc(b, f, 2.12, 0.355, 0.035, 8, red);
            lettering(b, f, "STOP", 2.12, 0.023, white);
        }
        FurnitureKind::Speed => {
            disc(b, f, 2.12, 0.37, -0.025, 32, metal);
            disc(b, f, 2.12, 0.365, 0.015, 32, red);
            disc(b, f, 2.12, 0.29, 0.035, 32, white);
            let label = f.value.to_string();
            let size = 0.033_f32.min(0.48 / (label.len() * 6) as f32);
            lettering(b, f, &label, 2.12, size, dark);
        }
        FurnitureKind::Yield => {
            let triangle = |scale: f32, depth: f32| {
                [
                    point(f, -0.42 * scale, 2.40, depth),
                    point(f, 0.42 * scale, 2.40, depth),
                    point(f, 0.0, 2.40 - 0.73 * scale, depth),
                ]
            };
            b.triangle(triangle(1.0, 0.015), red);
            b.triangle(triangle(0.79, 0.035), white);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sign_posts_stay_behind_the_lettered_faces() {
        for kind in [FurnitureKind::Stop, FurnitureKind::Speed] {
            let feature = Furniture {
                _source_id: 1,
                kind,
                position: [0.0; 3],
                travel: [0.0, 1.0],
                value: 30,
                phase: 0,
                _estimated_curb: true,
                _inferred_direction: false,
            };
            let mut b = Builder::default();
            build(&mut b, &feature);
            for v in b
                .vertices
                .iter()
                .filter(|v| v.color == [0.29, 0.32, 0.32] && (1.9..=2.12).contains(&v.position[1]))
            {
                let front = -v.position[2];
                assert!(
                    front <= -0.019,
                    "post pierces the sign face at {:?}",
                    v.position
                );
            }
        }
    }
}
