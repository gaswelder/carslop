//! Bridge decks, edge rails and tunnel interiors share their contact footprint.

use crate::model;
use crate::model::map_details::RoadSurface;
use crate::model::world::WorldLocation;
use crate::render;
use crate::render::scene::Builder;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

pub fn build(builder: &mut Builder, city: &WorldLocation) {
    let concrete = render::scene::matte([0.57, 0.57, 0.54]);
    let metal = render::scene::matte([0.48, 0.52, 0.52]);
    for deck in &model::world::map_details(city).decks {
        let color = match deck.surface {
            RoadSurface::Asphalt => [0.24, 0.25, 0.26],
            RoadSurface::Concrete => [0.62, 0.60, 0.55],
            RoadSurface::Paving => [0.55, 0.51, 0.45],
            RoadSurface::Unpaved => [0.52, 0.45, 0.34],
        };
        for triangle in world::map_details::deck_triangles(deck) {
            let p = triangle.points.map(|mut p| {
                p.y += 0.05;
                p
            });
            render::scene::upward_quad(
                builder,
                [p[0], p[1], p[2], p[0]],
                render::scene::surface_road(color),
            );
            if deck.grade < 0 {
                let p = triangle.points.map(|mut p| {
                    p.y += 4.5;
                    p
                });
                render::scene::quad(builder, [p[2], p[1], p[0], p[2]], concrete);
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
                    render::scene::quad(builder, p, concrete);
                    render::scene::quad(builder, [p[3], p[2], p[1], p[0]], concrete);
                } else {
                    render::details::beam(builder, point(a, -0.2), point(b, -0.2), 0.4, concrete);
                    for height in [0.5, 0.95] {
                        render::details::beam(
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
                        let p = util::geometry::xyz_from_fn(|j| {
                            util::geometry::xyz_component(a, j)
                                + (util::geometry::xyz_component(b, j)
                                    - util::geometry::xyz_component(a, j))
                                    * t
                        });
                        render::details::beam(builder, point(p, 0.), point(p, 1.), 0.08, metal);
                    }
                }
            }
            if deck.grade > 0 && length > 12. {
                let steps = (length / 25.).ceil() as usize;
                for i in 1..steps {
                    let t = i as f32 / steps as f32;
                    let p = util::geometry::xyz_from_fn(|j| {
                        util::geometry::xyz_component(a, j)
                            + (util::geometry::xyz_component(b, j)
                                - util::geometry::xyz_component(a, j))
                                * t
                    });
                    if world::street_surface::contains_road(city, util::geometry::xz(p)) {
                        continue;
                    }
                    let ground = world::elevation::world_height(city, util::geometry::xz(p));
                    if p.y - ground > 2. {
                        render::details::beam(
                            builder,
                            util::geometry::at_height(util::geometry::xz(p), ground),
                            util::geometry::at_height(util::geometry::xz(p), p.y - 0.2),
                            0.7,
                            concrete,
                        );
                    }
                }
            }
        }
    }
}
