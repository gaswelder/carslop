//! Bridge decks, edge rails and tunnel interiors share their contact footprint.
use super::detail_scene;
use super::{Builder, Surface};
use crate::geometry::{XYZ, XZ};
use crate::map_details;
use crate::map_details::RoadSurface;
use crate::world::WorldLocation;

pub(super) fn build(builder: &mut Builder, city: &WorldLocation) {
    let concrete = Surface::matte([0.57, 0.57, 0.54]);
    let metal = Surface::matte([0.48, 0.52, 0.52]);
    for deck in &city.map_details().decks {
        let color = match deck.surface {
            RoadSurface::Asphalt => [0.24, 0.25, 0.26],
            RoadSurface::Concrete => [0.62, 0.60, 0.55],
            RoadSurface::Paving => [0.55, 0.51, 0.45],
            RoadSurface::Unpaved => [0.52, 0.45, 0.34],
        };
        for triangle in map_details::deck_triangles(deck) {
            let p = triangle.points.map(|mut p| {
                p.y += 0.05;
                p
            });
            builder.upward_quad([p[0], p[1], p[2], p[0]], Surface::road(color));
            if deck.grade < 0 {
                let p = triangle.points.map(|mut p| {
                    p.y += 4.5;
                    p
                });
                builder.quad([p[2], p[1], p[0], p[2]], concrete);
            }
        }
        for pair in deck.path.windows(2) {
            let a = XYZ {
                x: pair[0][0],
                y: pair[0][1],
                z: pair[0][2],
            };
            let b = XYZ {
                x: pair[1][0],
                y: pair[1][1],
                z: pair[1][2],
            };
            let dx = b.x - a.x;
            let dz = b.z - a.z;
            let length = dx.hypot(dz);
            let normal = XZ {
                x: -dz / length,
                z: dx / length,
            };
            for side in [-1., 1.] {
                let point = |p: XYZ, height: f32| XYZ {
                    x: p.x + normal.x * side * deck.width / 2.,
                    y: p.y + height,
                    z: p.z + normal.z * side * deck.width / 2.,
                };
                if deck.grade < 0 {
                    let p = [point(a, 0.), point(b, 0.), point(b, 4.5), point(a, 4.5)];
                    builder.quad(p, concrete);
                    builder.quad([p[3], p[2], p[1], p[0]], concrete);
                } else {
                    detail_scene::beam(builder, point(a, -0.2), point(b, -0.2), 0.4, concrete);
                    for height in [0.5, 0.95] {
                        detail_scene::beam(
                            builder,
                            point(a, height),
                            point(b, height),
                            0.08,
                            metal,
                        );
                    }
                    let steps = (length / 3.).ceil() as usize;
                    for i in 0..=steps {
                        let t = i as f32 / steps.max(1) as f32;
                        let p = XYZ::from_fn(|j| {
                            a.component(j) + (b.component(j) - a.component(j)) * t
                        });
                        detail_scene::beam(builder, point(p, 0.), point(p, 1.), 0.08, metal);
                    }
                }
            }
            if deck.grade > 0 && length > 12. {
                let steps = (length / 25.).ceil() as usize;
                for i in 1..steps {
                    let t = i as f32 / steps as f32;
                    let p =
                        XYZ::from_fn(|j| a.component(j) + (b.component(j) - a.component(j)) * t);
                    if city.contains_road(p.xz()) {
                        continue;
                    }
                    let ground = city.height(p.xz());
                    if p.y - ground > 2. {
                        detail_scene::beam(
                            builder,
                            p.xz().at_height(ground),
                            p.xz().at_height(p.y - 0.2),
                            0.7,
                            concrete,
                        );
                    }
                }
            }
        }
    }
}
