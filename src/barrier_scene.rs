//! Terrain-following walls and open metal, chain-link, or wooden fence panels.
use super::{Builder, Surface};
use crate::geometry::{XYZ, XZ};
use crate::world::{BarrierKind, Location};

fn prism(b: &mut Builder, a: XYZ, c: XYZ, width: f32, height: f32, surface: Surface) {
    let length = (c.x - a.x).hypot(c.z - a.z);
    if length < 0.0001 {
        return;
    }
    let side = XZ {
        x: -(c.z - a.z) / length * width / 2.0,
        z: (c.x - a.x) / length * width / 2.0,
    };
    let p = [
        XYZ {
            x: a.x - side.x,
            y: a.y,
            z: a.z - side.z,
        },
        XYZ {
            x: c.x - side.x,
            y: c.y,
            z: c.z - side.z,
        },
        XYZ {
            x: c.x + side.x,
            y: c.y,
            z: c.z + side.z,
        },
        XYZ {
            x: a.x + side.x,
            y: a.y,
            z: a.z + side.z,
        },
    ];
    let top = p.map(|point| XYZ {
        y: point.y + height,
        ..point
    });
    b.quad([p[1], p[0], top[0], top[1]], surface);
    b.quad([p[3], p[2], top[2], top[3]], surface);
    b.quad([p[0], p[3], top[3], top[0]], surface);
    b.quad([p[2], p[1], top[1], top[2]], surface);
    b.upward_quad(top, surface);
}

fn support(city: &Location, point: XZ<f32>) -> XYZ {
    point.at_height(city.height(point) + city.paving_height(point).unwrap_or(0.0))
}

pub(super) fn build(b: &mut Builder, city: &Location, detailed: bool) {
    for barrier in city.barriers() {
        let height = barrier.height_meters;
        let width = barrier.width_meters;
        let surface = Surface::matte(match barrier.kind {
            BarrierKind::Wall => [0.72, 0.70, 0.62],
            BarrierKind::WoodFence => [0.43, 0.34, 0.23],
            _ => [0.35, 0.39, 0.36],
        });
        let [a, c] = barrier.ends;
        if barrier.kind == BarrierKind::Wall {
            for pair in city.ground_segment(a, c).windows(2) {
                let a = support(city, pair[0].xz());
                let c = support(city, pair[1].xz());
                prism(b, a, c, width, height, surface);
                if detailed {
                    prism(
                        b,
                        XYZ {
                            y: a.y + height - 0.07,
                            ..a
                        },
                        XYZ {
                            y: c.y + height - 0.07,
                            ..c
                        },
                        width + 0.04,
                        0.07,
                        Surface::matte([0.79, 0.77, 0.69]),
                    );
                }
            }
            continue;
        }
        let length = (c.x - a.x).hypot(c.z - a.z);
        let panels = (length / 2.4).ceil().max(1.0) as usize;
        let point = |t: f32| {
            support(
                city,
                XZ {
                    x: a.x + (c.x - a.x) * t,
                    z: a.z + (c.z - a.z) * t,
                },
            )
        };
        let direction = XZ {
            x: (c.x - a.x) / length,
            z: (c.z - a.z) / length,
        };
        for panel in 0..=panels {
            let p = point(panel as f32 / panels as f32);
            let half = width.max(0.08) / 2.0;
            prism(
                b,
                XYZ {
                    x: p.x - direction.x * half,
                    z: p.z - direction.z * half,
                    ..p
                },
                XYZ {
                    x: p.x + direction.x * half,
                    z: p.z + direction.z * half,
                    ..p
                },
                width.max(0.08),
                height,
                surface,
            );
            if panel == panels {
                continue;
            }
            let t0 = panel as f32 / panels as f32;
            let t1 = (panel + 1) as f32 / panels as f32;
            let start = point(t0);
            let end = point(t1);
            if barrier.kind == BarrierKind::WoodFence && !detailed {
                prism(b, start, end, width, height, surface);
                continue;
            }
            for fraction in [0.25, 0.95] {
                prism(
                    b,
                    XYZ {
                        y: start.y + height * fraction,
                        ..start
                    },
                    XYZ {
                        y: end.y + height * fraction,
                        ..end
                    },
                    width * 0.65,
                    0.045,
                    surface,
                );
            }
            if !detailed {
                continue;
            }
            let panel_length = length / panels as f32;
            let spacing = if barrier.kind == BarrierKind::ChainLink {
                0.5
            } else {
                0.22
            };
            let slats = (panel_length / spacing).ceil() as usize;
            for slat in 0..slats {
                let left_t = t0 + (t1 - t0) * slat as f32 / slats as f32;
                let right_t = t0 + (t1 - t0) * (slat + 1) as f32 / slats as f32;
                if barrier.kind == BarrierKind::ChainLink {
                    let left = point(left_t);
                    let right = point(right_t);
                    let rows = (height / 0.5).ceil() as usize;
                    for row in 0..rows {
                        let low = row as f32 * height / rows as f32;
                        let high = (row + 1) as f32 * height / rows as f32;
                        for reverse in [false, true] {
                            prism(
                                b,
                                XYZ {
                                    y: left.y + if reverse { high } else { low },
                                    ..left
                                },
                                XYZ {
                                    y: right.y + if reverse { low } else { high },
                                    ..right
                                },
                                0.012,
                                0.012,
                                surface,
                            );
                        }
                    }
                } else {
                    let center = point((left_t + right_t) / 2.0);
                    let slat_width = if barrier.kind == BarrierKind::WoodFence {
                        panel_length / slats as f32 * 0.88
                    } else {
                        0.025
                    };
                    prism(
                        b,
                        XYZ {
                            x: center.x - direction.x * slat_width / 2.0,
                            z: center.z - direction.z * slat_width / 2.0,
                            ..center
                        },
                        XYZ {
                            x: center.x + direction.x * slat_width / 2.0,
                            z: center.z + direction.z * slat_width / 2.0,
                            ..center
                        },
                        width * 0.5,
                        height,
                        surface,
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::asset_codecs;
    use crate::test_support;
    use crate::world;

    #[test]
    fn barrier_meshes_follow_terrain_and_keep_near_and_far_height() {
        for kind in [
            BarrierKind::Wall,
            BarrierKind::Fence,
            BarrierKind::ChainLink,
            BarrierKind::WoodFence,
        ] {
            let folder = tempfile::tempdir().unwrap();
            let assets = test_support::stored_location(folder.path(), false);
            let barriers = vec![world::Barrier {
                source_id: 1,
                kind,
                ends: [XZ { x: -5., z: 10. }, XZ { x: 20., z: 10. }],
                height_meters: 2.,
                width_meters: 0.12,
            }];
            asset_codecs::barriers::write(
                &mut std::fs::File::create(folder.path().join("barriers.txt")).unwrap(),
                &barriers,
            )
            .unwrap();
            let mut city = world::Location::load(&assets).unwrap();
            let field = world::HeightField::generate(city.bounding_box(), |p| 7.0 + p.x * 0.1);
            test_support::set_elevation(&mut city, Some(field));
            for detailed in [false, true] {
                let mut builder = Builder::default();
                build(&mut builder, &city, detailed);
                assert!(!builder.vertices.is_empty());
                let mut max_height: f32 = 0.;
                for vertex in &builder.vertices {
                    assert!(vertex
                        .position
                        .iter()
                        .chain(vertex.normal.iter())
                        .all(|v| v.is_finite()));
                    assert!((vertex.normal.iter().map(|v| v * v).sum::<f32>() - 1.0).abs() < 0.001);
                    let point = XZ {
                        x: vertex.position[0],
                        z: vertex.position[2],
                    };
                    let height = vertex.position[1] - city.height(point);
                    assert!(
                        (-0.03..=2.03).contains(&height),
                        "height {height} for {kind:?}"
                    );
                    max_height = max_height.max(height);
                }
                assert!((max_height - 2.0).abs() < 0.03);
            }
        }
    }
}
