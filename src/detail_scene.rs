//! Procedural geometry for mapped street objects and linear scenery.

use crate::chunks;
use crate::geometry;
use crate::geometry::{XYZ, XZ};
use crate::map_details::{LineKind, ObjectKind};
use crate::scene;
use crate::scene::{Builder, Surface};
use crate::street_surface;
use crate::world;
use crate::world::WorldLocation;
use crate::world_raised_roads;
use glam::Vec3;

pub fn beam(b: &mut Builder, a: XYZ, c: XYZ, width: f32, surface: Surface) {
    let a = Vec3::new(a.x, a.y, a.z);
    let c = Vec3::new(c.x, c.y, c.z);
    let delta = c - a;
    if delta.length_squared() < 0.0000001 {
        return;
    }
    let along = delta.normalize();
    let right = if along.y.abs() > 0.99 {
        Vec3::X
    } else {
        along.cross(Vec3::Y).normalize()
    } * width
        * 0.5;
    let up = along.cross(right).normalize() * width * 0.5;
    let point = |v: Vec3| XYZ {
        x: v.x,
        y: v.y,
        z: v.z,
    };
    let p = [
        point(a - right - up),
        point(c - right - up),
        point(c + right - up),
        point(a + right - up),
        point(a - right + up),
        point(c - right + up),
        point(c + right + up),
        point(a + right + up),
    ];
    for face in [
        [0, 1, 2, 3],
        [7, 6, 5, 4],
        [4, 5, 1, 0],
        [3, 2, 6, 7],
        [0, 3, 7, 4],
        [5, 6, 2, 1],
    ] {
        scene::quad(b, face.map(|i| p[i]), surface);
    }
}
fn cylinder(b: &mut Builder, x: f32, z: f32, radius: f32, bottom: f32, top: f32, surface: Surface) {
    for i in 0..8 {
        let a = i as f32 * std::f32::consts::TAU / 8.;
        let c = (i + 1) as f32 * std::f32::consts::TAU / 8.;
        let p = |angle: f32, y| XYZ {
            x: x + angle.cos() * radius,
            y,
            z: z + angle.sin() * radius,
        };
        scene::quad(
            b,
            [p(c, bottom), p(a, bottom), p(a, top), p(c, top)],
            surface,
        );
        scene::upward_quad(
            b,
            [
                XYZ { x, y: top, z },
                p(a, top),
                p(c, top),
                XYZ { x, y: top, z },
            ],
            surface,
        );
    }
}
pub fn build(b: &mut Builder, city: &WorldLocation, detailed: bool) {
    let metal = scene::matte([0.28, 0.32, 0.32]);
    let wood = scene::matte([0.43, 0.31, 0.20]);
    let stone = scene::matte([0.66, 0.64, 0.57]);
    for object in &world::map_details(city).objects {
        let mut local = Builder::default();
        let h = object.height;
        let w = object.width;
        match object.kind {
            ObjectKind::Bench => {
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w / 2.,
                        y: 0.40,
                        z: -0.23,
                    },
                    XYZ {
                        x: w / 2.,
                        y: 0.49,
                        z: 0.23,
                    },
                    wood,
                );
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w / 2.,
                        y: 0.53,
                        z: 0.20,
                    },
                    XYZ {
                        x: w / 2.,
                        y: h,
                        z: 0.28,
                    },
                    wood,
                );
                for x in [-w * 0.34, w * 0.34] {
                    scene::cuboid(
                        &mut local,
                        XYZ {
                            x: x - 0.04,
                            y: 0.,
                            z: -0.18,
                        },
                        XYZ {
                            x: x + 0.04,
                            y: 0.43,
                            z: 0.18,
                        },
                        metal,
                    );
                }
            }
            ObjectKind::Bin => {
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w / 2.,
                        y: 0.,
                        z: -w / 2.,
                    },
                    XYZ {
                        x: w / 2.,
                        y: h,
                        z: w / 2.,
                    },
                    scene::matte([0.18, 0.29, 0.22]),
                );
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w * 0.56,
                        y: h,
                        z: -w * 0.56,
                    },
                    XYZ {
                        x: w * 0.56,
                        y: h + 0.04,
                        z: w * 0.56,
                    },
                    metal,
                );
            }
            ObjectKind::Lamp | ObjectKind::PowerPole => {
                cylinder(&mut local, 0., 0., w.max(0.07) / 2., 0., h, metal);
                if object.kind == ObjectKind::Lamp {
                    beam(
                        &mut local,
                        XYZ {
                            x: 0.,
                            y: h - 0.2,
                            z: 0.,
                        },
                        XYZ {
                            x: 0.,
                            y: h - 0.1,
                            z: 0.8,
                        },
                        0.07,
                        metal,
                    );
                    scene::cuboid(
                        &mut local,
                        XYZ {
                            x: -0.16,
                            y: h - 0.18,
                            z: 0.55,
                        },
                        XYZ {
                            x: 0.16,
                            y: h,
                            z: 1.0,
                        },
                        scene::matte([0.88, 0.88, 0.70]),
                    );
                } else {
                    beam(
                        &mut local,
                        XYZ {
                            x: -0.9,
                            y: h - 0.4,
                            z: 0.,
                        },
                        XYZ {
                            x: 0.9,
                            y: h - 0.4,
                            z: 0.,
                        },
                        0.09,
                        metal,
                    );
                }
            }
            ObjectKind::Shelter => {
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w / 2. - 0.08,
                        y: h - 0.12,
                        z: -0.8,
                    },
                    XYZ {
                        x: w / 2. + 0.08,
                        y: h,
                        z: 0.8,
                    },
                    metal,
                );
                for x in [-w / 2., w / 2.] {
                    for z in [-0.7, 0.7] {
                        cylinder(&mut local, x, z, 0.04, 0., h, metal);
                    }
                }
                if detailed {
                    scene::cuboid(
                        &mut local,
                        XYZ {
                            x: -w / 2.,
                            y: 0.25,
                            z: 0.66,
                        },
                        XYZ {
                            x: w / 2.,
                            y: h - 0.15,
                            z: 0.70,
                        },
                        Surface {
                            color: [0.53, 0.66, 0.64],
                            material: [2., 0.15],
                        },
                    );
                }
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w * 0.35,
                        y: 0.42,
                        z: 0.1,
                    },
                    XYZ {
                        x: w * 0.35,
                        y: 0.5,
                        z: 0.5,
                    },
                    wood,
                );
            }
            ObjectKind::Bollard => {
                if w > 0.6 {
                    scene::cuboid(
                        &mut local,
                        XYZ {
                            x: -w / 2.,
                            y: 0.,
                            z: -w / 2.,
                        },
                        XYZ {
                            x: w / 2.,
                            y: h,
                            z: w / 2.,
                        },
                        stone,
                    );
                } else {
                    cylinder(&mut local, 0., 0., w / 2., 0., h, metal);
                    cylinder(
                        &mut local,
                        0.,
                        0.,
                        w * 0.52,
                        h * 0.72,
                        h * 0.87,
                        scene::matte([0.92, 0.91, 0.82]),
                    );
                }
            }
            ObjectKind::Gate => {
                for x in [-w / 2., w / 2.] {
                    cylinder(&mut local, x, 0., 0.065, 0., h, metal);
                }
                let a = XYZ {
                    x: -w / 2.,
                    y: h * 0.2,
                    z: 0.,
                };
                let c = if object.locked {
                    XYZ { x: w / 2., ..a }
                } else {
                    XYZ {
                        x: a.x,
                        z: w * 0.9,
                        ..a
                    }
                };
                for lift in [0., h * 0.65] {
                    beam(
                        &mut local,
                        XYZ { y: a.y + lift, ..a },
                        XYZ { y: c.y + lift, ..c },
                        0.055,
                        metal,
                    );
                }
                if detailed {
                    beam(
                        &mut local,
                        a,
                        XYZ {
                            y: c.y + h * 0.65,
                            ..c
                        },
                        0.035,
                        metal,
                    );
                }
            }
            ObjectKind::PowerTower => {
                for side in [-1., 1.] {
                    for depth in [-1., 1.] {
                        beam(
                            &mut local,
                            XYZ {
                                x: side * w / 2.,
                                y: 0.,
                                z: depth * w / 2.,
                            },
                            XYZ {
                                x: side * w * 0.13,
                                y: h,
                                z: depth * w * 0.13,
                            },
                            0.12,
                            metal,
                        );
                    }
                }
                for step in 1..6 {
                    let y = h * step as f32 / 6.;
                    let radius = w * (0.5 - step as f32 / 6. * 0.37);
                    for z in [-radius, radius] {
                        beam(
                            &mut local,
                            XYZ { x: -radius, y, z },
                            XYZ { x: radius, y, z },
                            0.09,
                            metal,
                        );
                        if detailed {
                            beam(
                                &mut local,
                                XYZ {
                                    x: -radius,
                                    y: y - h / 6.,
                                    z,
                                },
                                XYZ { x: radius, y, z },
                                0.055,
                                metal,
                            );
                        }
                    }
                }
                for y in [h * 0.78, h * 0.94] {
                    beam(
                        &mut local,
                        XYZ { x: -w, y, z: 0. },
                        XYZ { x: w, y, z: 0. },
                        0.12,
                        metal,
                    );
                }
            }
            ObjectKind::Monument => {
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w * 0.6,
                        y: 0.,
                        z: -w * 0.6,
                    },
                    XYZ {
                        x: w * 0.6,
                        y: h * 0.2,
                        z: w * 0.6,
                    },
                    stone,
                );
                beam(
                    &mut local,
                    XYZ {
                        x: 0.,
                        y: h * 0.2,
                        z: 0.,
                    },
                    XYZ { x: 0., y: h, z: 0. },
                    w * 0.45,
                    stone,
                );
            }
            ObjectKind::Planter => {
                scene::cuboid(
                    &mut local,
                    XYZ {
                        x: -w / 2.,
                        y: 0.,
                        z: -w / 2.,
                    },
                    XYZ {
                        x: w / 2.,
                        y: h,
                        z: w / 2.,
                    },
                    stone,
                );
                cylinder(
                    &mut local,
                    0.,
                    0.,
                    w * 0.39,
                    h,
                    h + w * 0.4,
                    scene::matte([0.24, 0.37, 0.17]),
                );
            }
            ObjectKind::TurnSign => {
                cylinder(&mut local, 0., 0., 0.035, 0., h, metal);
                let cy = h - w / 2.;
                let only = object.text.starts_with("only_");
                for i in 0..16 {
                    let a = i as f32 * std::f32::consts::TAU / 16.;
                    let c = (i + 1) as f32 * std::f32::consts::TAU / 16.;
                    scene::emit_triangle(
                        &mut local,
                        [
                            XYZ {
                                x: 0.,
                                y: cy,
                                z: 0.04,
                            },
                            XYZ {
                                x: a.cos() * w / 2.,
                                y: cy + a.sin() * w / 2.,
                                z: 0.04,
                            },
                            XYZ {
                                x: c.cos() * w / 2.,
                                y: cy + c.sin() * w / 2.,
                                z: 0.04,
                            },
                        ],
                        scene::matte(if only {
                            [0.10, 0.27, 0.65]
                        } else {
                            [0.91, 0.90, 0.83]
                        }),
                    );
                }
                let ink = scene::matte(if only {
                    [0.95, 0.95, 0.91]
                } else {
                    [0.05, 0.06, 0.06]
                });
                let turn = if object.text.contains("left") {
                    -1.
                } else if object.text.contains("right") {
                    1.
                } else {
                    0.
                };
                beam(
                    &mut local,
                    XYZ {
                        x: 0.,
                        y: cy - w * 0.3,
                        z: 0.06,
                    },
                    XYZ {
                        x: 0.,
                        y: cy + w * 0.15,
                        z: 0.06,
                    },
                    0.035,
                    ink,
                );
                if turn != 0. {
                    beam(
                        &mut local,
                        XYZ {
                            x: 0.,
                            y: cy + w * 0.15,
                            z: 0.06,
                        },
                        XYZ {
                            x: turn * w * 0.26,
                            y: cy + w * 0.15,
                            z: 0.06,
                        },
                        0.035,
                        ink,
                    );
                }
                if !only {
                    beam(
                        &mut local,
                        XYZ {
                            x: -w * 0.31,
                            y: cy - w * 0.31,
                            z: 0.08,
                        },
                        XYZ {
                            x: w * 0.31,
                            y: cy + w * 0.31,
                            z: 0.08,
                        },
                        0.04,
                        scene::matte([0.79, 0.05, 0.03]),
                    );
                }
            }
        }
        let origin = XZ {
            x: object.position[0],
            z: object.position[1],
        };
        let y = world::height(city, origin)
            + world_raised_roads::road_rise(city, origin)
            + street_surface::paving_height(city, origin).unwrap_or(0.0);
        let (sin, cos) = object.angle.sin_cos();
        for triangle in local.vertices.chunks_exact(3) {
            let vertices = std::array::from_fn(|i| {
                let mut v = triangle[i];
                let [x, height, z] = v.position;
                v.position = [
                    origin.x + x * cos - z * sin,
                    y + height,
                    origin.z + x * sin + z * cos,
                ];
                let [x, height, z] = v.normal;
                v.normal = [x * cos - z * sin, height, x * sin + z * cos];
                v
            });
            if let Some(chunks) = &mut b.chunks {
                chunks::push(chunks, vertices);
            } else {
                b.vertices.extend(vertices);
            }
        }
    }
    for line in &world::map_details(city).lines {
        for pair in line.points.windows(2) {
            let a = XZ {
                x: pair[0][0],
                z: pair[0][1],
            };
            let c = XZ {
                x: pair[1][0],
                z: pair[1][1],
            };
            if line.kind == LineKind::Cliff {
                let a = geometry::at_height(a, world::height(city, a));
                let c = geometry::at_height(c, world::height(city, c));
                scene::quad(
                    b,
                    [
                        XYZ {
                            y: c.y - line.height,
                            ..c
                        },
                        XYZ {
                            y: a.y - line.height,
                            ..a
                        },
                        a,
                        c,
                    ],
                    scene::matte([0.52, 0.50, 0.43]),
                );
            } else {
                let length = (c.x - a.x).hypot(c.z - a.z);
                if length < 0.01 {
                    continue;
                }
                for cable in [-1., 0., 1.] {
                    let point = |t: f32| {
                        let p = XZ {
                            x: a.x + (c.x - a.x) * t - (c.z - a.z) / length * cable * 0.5,
                            z: a.z + (c.z - a.z) * t + (c.x - a.x) / length * cable * 0.5,
                        };
                        geometry::at_height(
                            p,
                            world::height(city, a) * (1. - t)
                                + world::height(city, c) * t
                                + line.height
                                - length.mul_add(0.035, 0.0).min(4.) * 4. * t * (1. - t),
                        )
                    };
                    for step in 0..8 {
                        beam(
                            b,
                            point(step as f32 / 8.),
                            point((step + 1) as f32 / 8.),
                            0.022,
                            metal,
                        );
                    }
                }
            }
        }
    }
}
