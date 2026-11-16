//! Map projection, visible bounds, and zoom independent of OpenGL.

use crate::model::world::WorldLocation;
use crate::util::geometry::{BoundingBoxXZ, XYZ, XZ};
use crate::{model, util, world};

pub const MAP_CONTENT: [[f32; 2]; 2] = [[0.61, 0.49], [0.95, 0.88]];
pub const FULL_MAP_CONTENT: [[f32; 2]; 2] = [[-0.94, -0.72], [0.94, 0.75]];
const MAP_WIDTH_METRES: f32 = 1000.0;

pub struct MapFit {
    center: XZ<f32>,
    pub scale: [f32; 2],
    pub north_up: bool,
    pub heading: Option<f32>,
    pub content: [[f32; 2]; 2],
}

pub fn map_fit_new(track: &WorldLocation, player: XYZ, viewport: [f32; 2]) -> MapFit {
    let x_scale = (MAP_CONTENT[1][0] - MAP_CONTENT[0][0]) / MAP_WIDTH_METRES;
    MapFit {
        center: util::geometry::xz(player),
        scale: [x_scale, x_scale * viewport[0] / viewport[1]],
        north_up: world::street_surface::has_compiled_roads(track),
        heading: None,
        content: MAP_CONTENT,
    }
}
pub fn world_bounds(map_fit: &MapFit) -> BoundingBoxXZ<f32> {
    let mut x = (map_fit.content[1][0] - map_fit.content[0][0]) * 0.5 / map_fit.scale[0];
    let mut z = (map_fit.content[1][1] - map_fit.content[0][1]) * 0.5 / map_fit.scale[1];
    if let Some(heading) = map_fit.heading {
        let (sin, cos) = heading.sin_cos();
        (x, z) = (x * cos.abs() + z * sin.abs(), x * sin.abs() + z * cos.abs());
    }
    BoundingBoxXZ {
        min: XZ {
            x: map_fit.center.x - x,
            z: map_fit.center.z - z,
        },
        max: XZ {
            x: map_fit.center.x + x,
            z: map_fit.center.z + z,
        },
    }
}
pub fn map_fit_point(map_fit: &MapFit, p: XYZ) -> [f32; 2] {
    let dx = p.x - map_fit.center.x;
    let dz = p.z - map_fit.center.z;
    let [x, y] = if let Some(heading) = map_fit.heading {
        let (sin, cos) = heading.sin_cos();
        [dz * sin - dx * cos, dx * sin + dz * cos]
    } else {
        [dx, dz * if map_fit.north_up { -1.0 } else { 1.0 }]
    };
    [
        (map_fit.content[0][0] + map_fit.content[1][0]) * 0.5 + x * map_fit.scale[0],
        (map_fit.content[0][1] + map_fit.content[1][1]) * 0.5 + y * map_fit.scale[1],
    ]
}

pub struct FullMapView {
    pub copy_status: Option<&'static str>,
    pub fit: MapFit,
    min_scale: f32,
    max_scale: f32,
}

pub fn full_map_view_new(track: &WorldLocation, player: XYZ, viewport: [f32; 2]) -> FullMapView {
    // Keep the minimap's metres per pixel, expanding the visible area.
    let mut fit = map_fit_new(track, player, viewport);
    fit.content = FULL_MAP_CONTENT;
    let mut bounds = model::world::bounding_box(track);
    for p in map_segments(track).flat_map(|(a, b)| [a, b]) {
        bounds.min.x = bounds.min.x.min(p.x);
        bounds.min.z = bounds.min.z.min(p.z);
        bounds.max.x = bounds.max.x.max(p.x);
        bounds.max.z = bounds.max.z.max(p.z);
    }
    // Zooming out can reveal the entire network while keeping the car centered.
    let size: [f32; 2] = std::array::from_fn(|i| {
        2.0 * ([bounds.min.x, bounds.min.z][i] - util::geometry::xz_component(fit.center, i))
            .abs()
            .max(
                ([bounds.max.x, bounds.max.z][i] - util::geometry::xz_component(fit.center, i))
                    .abs(),
            )
            .max(1.0)
    });
    let available: [f32; 2] =
        std::array::from_fn(|i| (FULL_MAP_CONTENT[1][i] - FULL_MAP_CONTENT[0][i]) * 0.9);
    let min_scale = (available[0] / size[0])
        .min(available[1] / (size[1] * viewport[0] / viewport[1]))
        .min(fit.scale[0]);
    FullMapView {
        copy_status: None,
        min_scale,
        max_scale: fit.scale[0] * 16.0,
        fit,
    }
}

pub fn zoom_full_map(full_map_view: &mut FullMapView, steps: f32) -> bool {
    if !steps.is_finite() || steps == 0.0 {
        return false;
    }
    let scale = (full_map_view.fit.scale[0] * 1.25_f32.powf(steps.clamp(-100.0, 100.0)))
        .clamp(full_map_view.min_scale, full_map_view.max_scale);
    if scale == full_map_view.fit.scale[0] {
        return false;
    }
    full_map_view.fit.scale[1] *= scale / full_map_view.fit.scale[0];
    full_map_view.fit.scale[0] = scale;
    true
}

pub fn zoom_factor(full_map_view: &FullMapView) -> f32 {
    full_map_view.fit.scale[0] / (full_map_view.max_scale / 16.0)
}

pub fn set_zoom_factor(full_map_view: &mut FullMapView, factor: f32) {
    let scale = (full_map_view.max_scale / 16.0 * factor)
        .clamp(full_map_view.min_scale, full_map_view.max_scale);
    if scale != full_map_view.fit.scale[0] {
        full_map_view.fit.scale[1] *= scale / full_map_view.fit.scale[0];
        full_map_view.fit.scale[0] = scale;
    }
}

fn map_segments(world: &WorldLocation) -> Box<dyn Iterator<Item = (XYZ, XYZ)> + '_> {
    Box::new(
        world::roads::road_lines(world)
            .iter()
            .flat_map(|street| street.path.windows(2).map(|p| (p[0], p[1]))),
    )
}

pub fn map_segments_near(
    world: &WorldLocation,
    bounds: BoundingBoxXZ<f32>,
) -> Box<dyn Iterator<Item = (XYZ, XYZ)> + '_> {
    Box::new(world::roads::road_segments_in(world, bounds).into_iter())
}
