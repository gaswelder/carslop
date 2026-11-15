//! Mapped roof profiles clipped to the surveyed footprint, including holes.

use crate::model_map_details::{BuildingStyle, RoofShape};
use crate::model_world;
use crate::model_world::Building;
use crate::render_scene;
use crate::render_scene::{Builder, Surface};
use crate::util_geometry;
use crate::util_geometry::{XYZ, XZ};
use geo::{BooleanOps, BoundingRect, MultiPolygon, TriangulateEarcut};

pub fn build(
    b: &mut Builder,
    building: &Building,
    style: &BuildingStyle,
    roof: Surface,
    wall: Surface,
) {
    let (sin, cos) = style.angle.sin_cos();
    let local = |p: XZ<f32>| XZ {
        x: (p.x * cos + p.z * sin) as f64,
        z: (-p.x * sin + p.z * cos) as f64,
    };
    let world = |p: XZ<f64>, height: f32| XYZ {
        x: p.x as f32 * cos - p.z as f32 * sin,
        y: height,
        z: p.x as f32 * sin + p.z as f32 * cos,
    };
    let footprint = geo::Polygon::new(
        util_geometry::line(
            &building
                .ground_outline
                .iter()
                .map(|p| local(*p))
                .collect::<Vec<_>>(),
        ),
        building
            .holes
            .iter()
            .map(|hole| util_geometry::line(&hole.iter().map(|p| local(*p)).collect::<Vec<_>>()))
            .collect(),
    );
    let Some(bounds) = footprint.bounding_rect() else {
        return;
    };
    let min = XZ {
        x: bounds.min().x,
        z: bounds.min().y,
    };
    let size = XZ {
        x: (bounds.max().x - min.x).max(0.001),
        z: (bounds.max().y - min.z).max(0.001),
    };
    let base = building.base_y + building.height_meters - style.roof_height;
    let profile = |p: XZ<f64>| {
        let u = ((p.x - min.x) / size.x).clamp(0., 1.);
        let v = ((p.z - min.z) / size.z).clamp(0., 1.);
        let a = (u * 2. - 1.).abs();
        let c = (v * 2. - 1.).abs();
        let height = match style.shape {
            RoofShape::Flat => 0.,
            RoofShape::Gabled => 1. - c,
            RoofShape::Hipped => (1. - c).min(((1. - a) * size.x / size.z).min(1.)),
            RoofShape::Pyramidal => (1. - a).min(1. - c),
            RoofShape::Skillion => 1. - v,
            RoofShape::Dome => (1. - a * a - c * c).max(0.).sqrt(),
        };
        base + height as f32 * style.roof_height
    };
    let cells = if style.shape == RoofShape::Dome {
        16
    } else if matches!(style.shape, RoofShape::Gabled | RoofShape::Skillion) {
        2
    } else {
        8
    };
    for row in 0..cells {
        for col in 0..cells {
            let start = XZ {
                x: min.x + size.x * col as f64 / cells as f64,
                z: min.z + size.z * row as f64 / cells as f64,
            };
            let end = XZ {
                x: min.x + size.x * (col + 1) as f64 / cells as f64,
                z: min.z + size.z * (row + 1) as f64 / cells as f64,
            };
            let cell = util_geometry::polygon(&[
                start,
                XZ {
                    x: end.x,
                    z: start.z,
                },
                end,
                XZ {
                    x: start.x,
                    z: end.z,
                },
            ]);
            let clipped =
                MultiPolygon(vec![footprint.clone()]).intersection(&MultiPolygon(vec![cell]));
            for triangle in clipped
                .0
                .iter()
                .flat_map(|polygon| polygon.earcut_triangles())
            {
                let p = triangle.to_array().map(|p| {
                    let p = XZ { x: p.x, z: p.y };
                    world(p, profile(p))
                });
                render_scene::upward_quad(b, [p[0], p[1], p[2], p[0]], roof);
            }
        }
    }
    for ring in model_world::rings(building) {
        for i in 0..ring.len() {
            let a = local(ring[i]);
            let c = local(ring[(i + 1) % ring.len()]);
            let mut stations = vec![0.0, 1.0];
            for axis in 0..2 {
                let delta =
                    util_geometry::xz_component(c, axis) - util_geometry::xz_component(a, axis);
                if delta.abs() < 0.00000001 {
                    continue;
                }
                for step in 1..cells {
                    let line = util_geometry::xz_component(min, axis)
                        + util_geometry::xz_component(size, axis) * step as f64 / cells as f64;
                    let station = (line - util_geometry::xz_component(a, axis)) / delta;
                    if station > 0. && station < 1. {
                        stations.push(station);
                    }
                }
            }
            stations.sort_by(f64::total_cmp);
            stations.dedup_by(|a, b| (*a - *b).abs() < 0.000001);
            for pair in stations.windows(2) {
                let start =
                    util_geometry::add(a, util_geometry::mul(util_geometry::sub(c, a), pair[0]));
                let end =
                    util_geometry::add(a, util_geometry::mul(util_geometry::sub(c, a), pair[1]));
                render_scene::quad(
                    b,
                    [
                        world(end, base),
                        world(start, base),
                        world(start, profile(start)),
                        world(end, profile(end)),
                    ],
                    wall,
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::render_scene;

    use super::*;
    #[test]
    fn roof_profiles_keep_total_height_and_courtyard_openings() {
        let building = Building {
            id: 1,
            ground_outline: vec![
                XZ { x: 0., z: 0. },
                XZ { x: 20., z: 0. },
                XZ { x: 20., z: 10. },
                XZ { x: 0., z: 10. },
            ],
            holes: vec![vec![
                XZ { x: 8., z: 3. },
                XZ { x: 8., z: 7. },
                XZ { x: 12., z: 7. },
                XZ { x: 12., z: 3. },
            ]],
            min_height_meters: 0.,
            height_meters: 10.,
            base_y: 2.,
            tagged: true,
        };
        for shape in [
            RoofShape::Gabled,
            RoofShape::Hipped,
            RoofShape::Pyramidal,
            RoofShape::Skillion,
            RoofShape::Dome,
        ] {
            let style = BuildingStyle {
                volume: 0,
                shape,
                roof_height: 3.,
                angle: 0.,
                wall_color: None,
                roof_color: None,
            };
            let mut builder = Builder::default();
            build(
                &mut builder,
                &building,
                &style,
                render_scene::matte([0.8, 0.2, 0.2]),
                render_scene::matte([0.5; 3]),
            );
            assert!(!builder.vertices.is_empty());
            let max = builder
                .vertices
                .iter()
                .map(|v| v.position[1])
                .fold(f32::NEG_INFINITY, f32::max);
            assert!(max <= 12.001 && max > 11., "{shape:?}: {max}");
            for triangle in builder.vertices.chunks_exact(3) {
                let center = XZ {
                    x: triangle.iter().map(|v| v.position[0]).sum::<f32>() / 3.,
                    z: triangle.iter().map(|v| v.position[2]).sum::<f32>() / 3.,
                };
                assert!(
                    !(center.x > 8.001
                        && center.x < 11.999
                        && center.z > 3.001
                        && center.z < 6.999),
                    "roof filled courtyard: {shape:?} {center:?}"
                );
            }
        }
    }
}
