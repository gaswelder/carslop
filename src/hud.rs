use crate::city_assets;
use crate::font;
use crate::geometry;
use crate::geometry::{BoundingBoxXZ, XYZ, XZ};
use crate::hud_atlas;
use crate::phys_car;
use crate::street_surface;
use crate::track;
#[cfg(test)]
use crate::track::LocationId;
use crate::world;
use crate::world_roads;
use crate::{asset_car, track::WorldLocation};
use std::ffi::CString;

#[repr(C)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 3],
    uv: [f32; 2],
}

fn solid(vertices: &mut Vec<Vertex>, point: [f32; 2], color: [f32; 3]) {
    vertices.push(Vertex {
        position: [point[0], point[1], 0.0],
        color,
        uv: [-1.0; 2],
    });
}

const CHOICES_PER_PAGE: usize = 6;
fn choice_box(index: usize, left: f32) -> [f32; 4] {
    let top = 0.80 - index as f32 * 0.10;
    [left, top - 0.08, left + 0.72, top]
}
fn location_box(index: usize) -> [f32; 4] {
    choice_box(index, -0.96)
}
fn car_box(index: usize) -> [f32; 4] {
    choice_box(index, -0.16)
}
const START_BOX: [f32; 4] = [0.50, -0.98, 0.98, -0.86];
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MenuAction {
    Choice(usize),
    Car(usize),
    Start,
}
pub struct MainMenu<'a> {
    pub locations: &'a [city_assets::MenuLocation],
    pub selected: usize,
    pub car: usize,
    pub cars: &'a [std::sync::Arc<asset_car::CarSpec>],
    pub focus: usize,
}
fn location_menu_hit_test(x: f32, y: f32, selected: usize, count: usize) -> Option<MenuAction> {
    for slot in 0..CHOICES_PER_PAGE {
        let r = location_box(slot);
        if x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3] {
            let index = selected / CHOICES_PER_PAGE * CHOICES_PER_PAGE + slot;
            return (index < count).then_some(MenuAction::Choice(index));
        }
    }
    if x >= START_BOX[0] && x <= START_BOX[2] && y >= START_BOX[1] && y <= START_BOX[3] {
        return Some(MenuAction::Start);
    }
    None
}
pub fn selection_menu_hit_test(
    x: f32,
    y: f32,
    selected: usize,
    count: usize,
    car: usize,
    car_count: usize,
) -> Option<MenuAction> {
    let page_start = car / CHOICES_PER_PAGE * CHOICES_PER_PAGE;
    for slot in 0..CHOICES_PER_PAGE {
        let r = car_box(slot);
        if x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3] {
            let index = page_start + slot;
            return (index < car_count).then_some(MenuAction::Car(index));
        }
    }
    location_menu_hit_test(x, y, selected, count)
}
pub struct Map<'a> {
    pub track: &'a WorldLocation,
    pub player: XYZ,
}

#[derive(Clone, Copy)]
pub struct Gauges {
    pub rpm: f32,
    pub redline_rpm: f32,
    pub speed_kmh: f32,
    pub gear: u8,
}

const PAUSE_BOXES: [[f32; 4]; 5] = [
    [-0.68, 0.06, -0.04, 0.20],
    [0.04, 0.06, 0.68, 0.20],
    [-0.68, -0.13, -0.04, 0.01],
    [0.04, -0.13, 0.68, 0.01],
    [-0.68, -0.32, 0.68, -0.18],
];
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PauseAction {
    Resume,
    Restart,
    Selection,
    Audio,
    Recover,
}
pub fn pause_hit_test(x: f32, y: f32) -> Option<PauseAction> {
    PAUSE_BOXES
        .iter()
        .position(|r| x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3])
        .map(|i| {
            [
                PauseAction::Resume,
                PauseAction::Restart,
                PauseAction::Selection,
                PauseAction::Audio,
                PauseAction::Recover,
            ][i]
        })
}
pub struct PauseMenu<'a> {
    pub city: bool,
    pub aerial_attribution: Option<&'a str>,
    pub audio: &'a str,
}

const MAP_CONTENT: [[f32; 2]; 2] = [[0.61, 0.49], [0.95, 0.88]];
const FULL_MAP_CONTENT: [[f32; 2]; 2] = [[-0.94, -0.72], [0.94, 0.75]];
const MAP_WIDTH_METRES: f32 = 1000.0;

struct MapFit {
    center: XZ<f32>,
    scale: [f32; 2],
    north_up: bool,
    heading: Option<f32>,
    content: [[f32; 2]; 2],
}

fn map_fit_new(track: &WorldLocation, player: XYZ, viewport: [f32; 2]) -> MapFit {
    let x_scale = (MAP_CONTENT[1][0] - MAP_CONTENT[0][0]) / MAP_WIDTH_METRES;
    MapFit {
        center: geometry::xz(player),
        scale: [x_scale, x_scale * viewport[0] / viewport[1]],
        north_up: street_surface::has_compiled_roads(track),
        heading: None,
        content: MAP_CONTENT,
    }
}
fn world_bounds(map_fit: &MapFit) -> BoundingBoxXZ<f32> {
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
fn map_fit_point(map_fit: &MapFit, p: XYZ) -> [f32; 2] {
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

struct FullMapView {
    copy_status: Option<&'static str>,
    fit: MapFit,
    min_scale: f32,
    max_scale: f32,
}

fn full_map_view_new(track: &WorldLocation, player: XYZ, viewport: [f32; 2]) -> FullMapView {
    // Keep the minimap's metres per pixel, expanding the visible area.
    let mut fit = map_fit_new(track, player, viewport);
    fit.content = FULL_MAP_CONTENT;
    let mut bounds = world::bounding_box(track);
    for p in map_segments(track).flat_map(|(a, b)| [a, b]) {
        bounds.min.x = bounds.min.x.min(p.x);
        bounds.min.z = bounds.min.z.min(p.z);
        bounds.max.x = bounds.max.x.max(p.x);
        bounds.max.z = bounds.max.z.max(p.z);
    }
    // Zooming out can reveal the entire network while keeping the car centered.
    let size: [f32; 2] = std::array::from_fn(|i| {
        2.0 * ([bounds.min.x, bounds.min.z][i] - geometry::xz_component(fit.center, i))
            .abs()
            .max(([bounds.max.x, bounds.max.z][i] - geometry::xz_component(fit.center, i)).abs())
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

fn zoom_full_map(full_map_view: &mut FullMapView, steps: f32) -> bool {
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

fn zoom_factor(full_map_view: &FullMapView) -> f32 {
    full_map_view.fit.scale[0] / (full_map_view.max_scale / 16.0)
}

fn set_zoom_factor(full_map_view: &mut FullMapView, factor: f32) {
    let scale = (full_map_view.max_scale / 16.0 * factor)
        .clamp(full_map_view.min_scale, full_map_view.max_scale);
    if scale != full_map_view.fit.scale[0] {
        full_map_view.fit.scale[1] *= scale / full_map_view.fit.scale[0];
        full_map_view.fit.scale[0] = scale;
    }
}

/// Liang–Barsky clipping keeps crossing roads even when both endpoints are
/// outside the vicinity. Padding includes the stroke width at the boundary.
fn clip_map_line(a: [f32; 2], b: [f32; 2], padding: f32) -> Option<[[f32; 2]; 2]> {
    clip_line(a, b, MAP_CONTENT, [padding; 2])
}
fn clip_line(
    a: [f32; 2],
    b: [f32; 2],
    content: [[f32; 2]; 2],
    padding: [f32; 2],
) -> Option<[[f32; 2]; 2]> {
    let delta = [b[0] - a[0], b[1] - a[1]];
    let mut start: f32 = 0.0;
    let mut end: f32 = 1.0;
    for axis in 0..2 {
        let min = content[0][axis] + padding[axis];
        let max = content[1][axis] - padding[axis];
        if delta[axis].abs() < 0.000001 {
            if a[axis] < min || a[axis] > max {
                return None;
            }
        } else {
            let p = (min - a[axis]) / delta[axis];
            let q = (max - a[axis]) / delta[axis];
            start = start.max(p.min(q));
            end = end.min(p.max(q));
            if start > end {
                return None;
            }
        }
    }
    Some([
        std::array::from_fn(|i| a[i] + delta[i] * start),
        std::array::from_fn(|i| a[i] + delta[i] * end),
    ])
}
fn map_marker(v: &mut Vec<Vertex>, point: [f32; 2], size: f32, aspect: f32, color: [f32; 3]) {
    if (0..2).any(|i| point[i] < MAP_CONTENT[0][i] || point[i] > MAP_CONTENT[1][i]) {
        return;
    }
    // A bordered navigation chevron. Pixel aspect keeps its silhouette consistent.
    for (scale, ink) in [
        (1.3, [0.015, 0.025, 0.035]),
        (1.0, [0.95, 0.98, 1.0]),
        (0.72, color),
    ] {
        let shape = [[0.0, 1.35], [-0.8, -1.0], [0.0, -0.55], [0.8, -1.0]];
        let points = shape.map(|p| {
            [
                (point[0] + p[0] * size * scale).clamp(MAP_CONTENT[0][0], MAP_CONTENT[1][0]),
                (point[1] + p[1] * size * scale * aspect)
                    .clamp(MAP_CONTENT[0][1], MAP_CONTENT[1][1]),
            ]
        });
        for i in [1, 2] {
            for p in [points[0], points[i], points[i + 1]] {
                solid(v, p, ink);
            }
        }
    }
}

pub struct Hud {
    font_texture: hud_atlas::Texture,
    vao: u32,
    vbo: u32,
    full_map_vertices: i32,
    full_map_view: Option<FullMapView>,
}

impl Drop for Hud {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteBuffers(1, &self.vbo);
            gl::DeleteVertexArrays(1, &self.vao);
        }
    }
}

pub fn draw_loading(hud: &Hud, program: u32, name: &str, elapsed: f32) {
    let mut vertices = Vec::new();
    let title = format!("Loading {name}");
    let width = font::width(font::labels(), &title);
    let size = 0.009_f32.min(1.8 / width.max(1.0));
    text(
        &mut vertices,
        &title,
        [-width * size * 0.5, 0.15],
        size,
        [0.15, 0.85, 0.95],
    );
    quad(
        &mut vertices,
        [-0.55, -0.22],
        [0.55, -0.19],
        [0.12, 0.20, 0.25],
    );
    let x = -0.55 + (elapsed * 1.5).sin().mul_add(0.5, 0.5) * 0.9;
    quad(
        &mut vertices,
        [x, -0.22],
        [x + 0.2, -0.19],
        [0.15, 0.85, 0.95],
    );
    upload(hud, program, &vertices);
}

pub fn hud_new() -> Hud {
    let font_texture = hud_atlas::texture_new();
    unsafe {
        let (mut vao, mut vbo) = (0, 0);
        gl::GenVertexArrays(1, &mut vao);
        gl::GenBuffers(1, &mut vbo);
        gl::BindVertexArray(vao);
        gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
        gl::EnableVertexAttribArray(0);
        let stride = std::mem::size_of::<Vertex>() as i32;
        gl::VertexAttribPointer(0, 3, gl::FLOAT, gl::FALSE, stride, std::ptr::null());
        gl::EnableVertexAttribArray(1);
        gl::VertexAttribPointer(1, 3, gl::FLOAT, gl::FALSE, stride, 12usize as *const _);
        gl::EnableVertexAttribArray(2);
        gl::VertexAttribPointer(2, 2, gl::FLOAT, gl::FALSE, stride, 24usize as *const _);
        Hud {
            font_texture,
            vao,
            vbo,
            full_map_vertices: 0,
            full_map_view: None,
        }
    }
}

pub fn draw(
    hud: &Hud,
    program: u32,
    map: Map<'_>,
    heading: f32,
    gauges: Gauges,
    pause: Option<PauseMenu<'_>>,
) {
    let mut v = Vec::new();
    let mut viewport = [0_i32; 4];
    unsafe {
        gl::GetIntegerv(gl::VIEWPORT, viewport.as_mut_ptr());
    }
    append_instruments(
        &mut v,
        gauges,
        [viewport[2].max(1) as f32, viewport[3].max(1) as f32],
    );
    quad(&mut v, [0.59, 0.47], [0.97, 0.90], [0.025, 0.05, 0.075]);
    let mut fit = map_fit_new(
        map.track,
        map.player,
        [viewport[2].max(1) as f32, viewport[3].max(1) as f32],
    );
    fit.heading = Some(heading);
    let width = if street_surface::has_compiled_roads(map.track) {
        0.0025
    } else {
        0.006
    };
    for (from, to) in map_segments_near(map.track, world_bounds(&fit)) {
        if let Some([a, b]) =
            clip_map_line(map_fit_point(&fit, from), map_fit_point(&fit, to), width)
        {
            line(&mut v, a, b, width, [0.4, 0.49, 0.52]);
        }
    }
    map_marker(
        &mut v,
        map_fit_point(&fit, map.player),
        0.014,
        fit.scale[1] / fit.scale[0],
        [0.15, 0.85, 0.95],
    );

    append_street_name(&mut v, street_name(map.track, map.player));
    if let Some(pause) = pause {
        append_pause(&mut v, pause);
    }
    upload(hud, program, &v);
}

pub fn prepare_full_map(
    hud: &mut Hud,
    map: Map<'_>,
    heading: f32,
    viewport: [f32; 2],
    zoom: Option<f32>,
) {
    let mut view = full_map_view_new(map.track, map.player, viewport);
    if let Some(zoom) = zoom {
        set_zoom_factor(&mut view, zoom);
    }
    hud.full_map_view = Some(view);
    rebuild_full_map(hud, map, heading, viewport);
}

pub fn hud_zoom_full_map(
    hud: &mut Hud,
    map: Map<'_>,
    heading: f32,
    viewport: [f32; 2],
    steps: f32,
) -> Option<f32> {
    let mut zoom = None;
    if let Some(view) = hud.full_map_view.as_mut() {
        if zoom_full_map(view, steps) {
            zoom = Some(zoom_factor(view));
        }
    }
    if zoom.is_some() {
        rebuild_full_map(hud, map, heading, viewport);
    }
    zoom
}

pub fn map_copy_status(
    hud: &mut Hud,
    map: Map<'_>,
    heading: f32,
    viewport: [f32; 2],
    status: &'static str,
) {
    if let Some(view) = hud.full_map_view.as_mut() {
        view.copy_status = Some(status);
        rebuild_full_map(hud, map, heading, viewport);
    }
}

fn rebuild_full_map(hud: &mut Hud, map: Map<'_>, heading: f32, viewport: [f32; 2]) {
    let view = hud.full_map_view.as_ref().expect("full map is open");
    let mut vertices = Vec::new();
    let hint = view.copy_status.unwrap_or_else(|| {
        if google_maps_url(map.track, map.player).is_some() {
            "C Copy Google Maps URL"
        } else {
            "No Google Maps location"
        }
    });
    append_full_map(&mut vertices, map, heading, viewport, &view.fit);
    text(&mut vertices, hint, [-0.90, -0.89], 0.003, [0.9, 0.95, 1.0]);
    // Reuse the uploaded map until it opens again or the zoom changes.
    set_vertices(hud, &vertices);
    hud.full_map_vertices = vertices.len() as i32;
}

pub fn draw_full_map(hud: &Hud, program: u32) {
    draw_vertices(hud, program, hud.full_map_vertices);
}

pub fn draw_menu(hud: &Hud, program: u32, menu: MainMenu) {
    let mut v = Vec::new();
    let cyan = [0.15, 0.85, 0.95];
    let white = [0.9, 0.95, 1.0];
    let muted = [0.55, 0.66, 0.70];
    for left in [-1.0, -0.20] {
        quad(
            &mut v,
            [left, 0.19],
            [left + 0.80, 0.98],
            [0.025, 0.05, 0.075],
        );
    }
    choice_header(
        &mut v,
        "Location",
        -0.96,
        menu.selected,
        menu.locations.len(),
    );
    for (i, entry) in menu
        .locations
        .iter()
        .enumerate()
        .skip(menu.selected / CHOICES_PER_PAGE * CHOICES_PER_PAGE)
        .take(CHOICES_PER_PAGE)
    {
        choice_button(
            &mut v,
            location_box(i % CHOICES_PER_PAGE),
            &entry.name,
            menu.selected == i,
            menu.focus == 0,
        );
    }
    let selected_car = menu.car;
    choice_header(&mut v, "Car", -0.16, selected_car, menu.cars.len());
    for (i, car) in menu
        .cars
        .iter()
        .enumerate()
        .skip(selected_car / CHOICES_PER_PAGE * CHOICES_PER_PAGE)
        .take(CHOICES_PER_PAGE)
    {
        choice_button(
            &mut v,
            car_box(i % CHOICES_PER_PAGE),
            &car.name,
            menu.car == i,
            menu.focus == 1,
        );
    }
    menu_button(&mut v, START_BOX, true, menu.focus == 2);
    text(
        &mut v,
        "Stard Driving (Enter)",
        [
            (START_BOX[0] + START_BOX[2]
                - font::width(font::labels(), "Stard Driving (Enter)") * 0.0034)
                * 0.5,
            -0.90,
        ],
        0.0034,
        cyan,
    );
    if track::asset_path(&menu.locations[menu.selected].track).is_some() {
        text(
            &mut v,
            "OpenStreetMap contributors / ODbL",
            [-0.92, -0.83],
            0.0025,
            muted,
        );
        text(
            &mut v,
            "Terrain: Mapzen / USGS / EU Copernicus",
            [-0.92, -0.87],
            0.0025,
            muted,
        );
        if let Some(label) = menu.locations[menu.selected]
            .assets
            .as_ref()
            .and_then(|a| city_assets::aerial_attribution(a))
        {
            text(&mut v, label, [-0.92, -0.91], 0.0024, muted);
        }
    }
    text(
        &mut v,
        "Left location / Right car / Up-down browse / Enter start / Q quit",
        [-0.92, -0.96],
        0.0025,
        white,
    );
    upload(hud, program, &v);
}

fn upload(hud: &Hud, program: u32, v: &[Vertex]) {
    set_vertices(hud, v);
    draw_vertices(hud, program, v.len() as i32);
}

fn set_vertices(hud: &Hud, v: &[Vertex]) {
    unsafe {
        gl::BindBuffer(gl::ARRAY_BUFFER, hud.vbo);
        gl::BufferData(
            gl::ARRAY_BUFFER,
            std::mem::size_of_val(v) as isize,
            v.as_ptr() as *const _,
            gl::STREAM_DRAW,
        );
    }
}

fn draw_vertices(hud: &Hud, program: u32, count: i32) {
    let identity = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    unsafe {
        let blending = gl::IsEnabled(gl::BLEND);
        let mut blend = [0; 6];
        for (value, parameter) in blend.iter_mut().zip([
            gl::BLEND_SRC_RGB,
            gl::BLEND_DST_RGB,
            gl::BLEND_SRC_ALPHA,
            gl::BLEND_DST_ALPHA,
            gl::BLEND_EQUATION_RGB,
            gl::BLEND_EQUATION_ALPHA,
        ]) {
            gl::GetIntegerv(parameter, value);
        }
        let mut active_texture = 0;
        gl::GetIntegerv(gl::ACTIVE_TEXTURE, &mut active_texture);
        gl::ActiveTexture(gl::TEXTURE0);
        let mut binding = 0;
        gl::GetIntegerv(gl::TEXTURE_BINDING_2D, &mut binding);
        hud_atlas::bind(&hud.font_texture);
        gl::Enable(gl::BLEND);
        gl::BlendEquation(gl::FUNC_ADD);
        gl::BlendFuncSeparate(
            gl::SRC_ALPHA,
            gl::ONE_MINUS_SRC_ALPHA,
            gl::ONE,
            gl::ONE_MINUS_SRC_ALPHA,
        );
        gl::Disable(gl::DEPTH_TEST);
        gl::BindVertexArray(hud.vao);
        gl::BindBuffer(gl::ARRAY_BUFFER, hud.vbo);
        gl::UseProgram(program);
        let name = CString::new("u_glyphs").unwrap();
        gl::Uniform1i(gl::GetUniformLocation(program, name.as_ptr()), 0);
        let name = CString::new("u_mvp").unwrap();
        gl::UniformMatrix4fv(
            gl::GetUniformLocation(program, name.as_ptr()),
            1,
            gl::FALSE,
            identity.as_ptr(),
        );
        let name = CString::new("u_tint").unwrap();
        gl::Uniform3f(
            gl::GetUniformLocation(program, name.as_ptr()),
            1.0,
            1.0,
            1.0,
        );
        gl::DrawArrays(gl::TRIANGLES, 0, count);
        gl::Enable(gl::DEPTH_TEST);
        gl::BindTexture(gl::TEXTURE_2D, binding as u32);
        gl::ActiveTexture(active_texture as u32);
        gl::BlendFuncSeparate(
            blend[0] as u32,
            blend[1] as u32,
            blend[2] as u32,
            blend[3] as u32,
        );
        gl::BlendEquationSeparate(blend[4] as u32, blend[5] as u32);
        if blending == gl::FALSE {
            gl::Disable(gl::BLEND);
        }
    }
}

fn append_full_map(
    v: &mut Vec<Vertex>,
    map: Map<'_>,
    heading: f32,
    viewport: [f32; 2],
    fit: &MapFit,
) {
    let white = [0.9, 0.95, 1.0];
    let muted = [0.55, 0.66, 0.70];
    let cyan = [0.15, 0.85, 0.95];
    let orange = [1.0, 0.25, 0.12];
    quad(v, [-1.0, -1.0], [1.0, 1.0], [0.015, 0.025, 0.04]);
    quad(
        v,
        FULL_MAP_CONTENT[0],
        FULL_MAP_CONTENT[1],
        [0.025, 0.05, 0.075],
    );
    text(v, "Full map", [-0.94, 0.92], 0.006, cyan);
    let name_size =
        0.004_f32.min(0.94 / font::width(font::labels(), &map.track.display_name).max(1.0));
    text(v, &map.track.display_name, [-0.60, 0.90], name_size, white);
    text(v, "M / Esc back", [0.49, 0.90], 0.004, white);

    let pixel = |p: [f32; 2]| {
        [
            (p[0] + 1.0) * viewport[0] * 0.5,
            (1.0 - p[1]) * viewport[1] * 0.5,
        ]
    };
    let mut canvas = InstrumentCanvas {
        vertices: v,
        viewport,
        scale: 1.0,
        origin: [0.0, 0.0],
    };
    let road_width = if fit.north_up { 1.5 } else { 3.0 };
    let padding = [road_width / viewport[0], road_width / viewport[1]];
    for (a, b) in map_segments_near(map.track, world_bounds(fit)) {
        if let Some([a, b]) = clip_line(
            map_fit_point(fit, a),
            map_fit_point(fit, b),
            fit.content,
            padding,
        ) {
            stroke(
                &mut canvas,
                pixel(a),
                pixel(b),
                road_width,
                [0.40, 0.55, 0.60],
            );
        }
    }
    let player = pixel(map_fit_point(fit, map.player));
    disc(&mut canvas, player, 15.0, white);
    disc(&mut canvas, player, 12.0, [0.025, 0.05, 0.075]);
    let forward = [
        heading.sin(),
        heading.cos() * if fit.north_up { 1.0 } else { -1.0 },
    ];
    let right = [-forward[1], forward[0]];
    let tip = std::array::from_fn(|i| player[i] + forward[i] * 10.0);
    let left = std::array::from_fn(|i| player[i] - forward[i] * 7.0 - right[i] * 7.0);
    let right = std::array::from_fn(|i| player[i] - forward[i] * 7.0 + right[i] * 7.0);
    triangle(&mut canvas, [tip, left, right], orange);
    disc(&mut canvas, pixel([-0.925, -0.855]), 5.0, orange);
    text(v, "Your position", [-0.90, -0.83], 0.0035, white);
    text(v, "Driving paused", [-0.54, -0.83], 0.0035, muted);
    text(v, "Wheel / +/- zoom", [-0.14, -0.83], 0.0035, white);
    if fit.north_up {
        text(v, "N", [0.885, 0.68], 0.0045, cyan);
        line(v, [0.90, 0.54], [0.90, 0.61], 0.002, cyan);
        line(v, [0.90, 0.61], [0.89, 0.58], 0.002, cyan);
        line(v, [0.90, 0.61], [0.91, 0.58], 0.002, cyan);
        text(
            v,
            "Map data: OpenStreetMap contributors / openstreetmap.org/copyright / ODbL",
            [-0.94, -0.94],
            0.0025,
            muted,
        );
    }
    // A round distance whose scale bar fits in the footer.
    let target = 0.25 / fit.scale[0];
    let magnitude = 10.0_f32.powf(target.log10().floor());
    let metres = [5.0, 2.0, 1.0]
        .into_iter()
        .find(|n| n * magnitude <= target)
        .unwrap_or(1.0)
        * magnitude;
    let width = metres * fit.scale[0];
    line(v, [0.94 - width, -0.88], [0.94, -0.88], 0.002, white);
    for x in [0.94 - width, 0.94] {
        line(v, [x, -0.87], [x, -0.89], 0.002, white);
    }
    let distance = if metres >= 1000.0 {
        format!("{:.0} KM", metres / 1000.0)
    } else {
        format!("{metres:.0} M")
    };
    text(v, &distance, [0.94 - width, -0.82], 0.003, white);
}

fn choice_header(v: &mut Vec<Vertex>, label: &str, left: f32, selected: usize, count: usize) {
    let cyan = [0.15, 0.85, 0.95];
    text(v, label, [left, 0.91], 0.0038, cyan);
    if count <= CHOICES_PER_PAGE {
        return;
    }
    text(
        v,
        &format!(
            "{} / {}",
            selected / CHOICES_PER_PAGE + 1,
            count.div_ceil(CHOICES_PER_PAGE)
        ),
        [left + 0.44, 0.91],
        0.0033,
        cyan,
    );
}
fn choice_button(v: &mut Vec<Vertex>, r: [f32; 4], label: &str, selected: bool, focused: bool) {
    menu_button(v, r, selected, focused && selected);
    let size = 0.0038_f32.min((r[2] - r[0] - 0.07) / font::width(font::regular(), label).max(1.0));
    text_with_font(
        v,
        &hud_atlas::get().regular,
        label,
        [r[0] + 0.035, (r[1] + r[3]) * 0.5 + size * 5.0],
        size,
        if selected {
            [0.15, 0.85, 0.95]
        } else {
            [0.9, 0.95, 1.0]
        },
    );
}

fn menu_button(v: &mut Vec<Vertex>, r: [f32; 4], selected: bool, focused: bool) {
    let fill = if focused {
        [0.07, 0.18, 0.23]
    } else if selected {
        [0.045, 0.105, 0.14]
    } else {
        [0.035, 0.065, 0.09]
    };
    quad(v, [r[0], r[1]], [r[2], r[3]], fill);
    if selected {
        quad(
            v,
            [r[0], r[1]],
            [r[0] + 0.008, r[3]],
            if focused {
                [0.15, 0.85, 0.95]
            } else {
                [0.10, 0.45, 0.55]
            },
        );
    }
}

// Instruments use a pixel coordinate canvas so round dials and constant-width
// needles stay round when the window's aspect ratio changes.
struct InstrumentCanvas<'a> {
    vertices: &'a mut Vec<Vertex>,
    viewport: [f32; 2],
    scale: f32,
    origin: [f32; 2],
}

fn instrument_canvas_point(instrument_canvas: &InstrumentCanvas<'_>, p: [f32; 2]) -> [f32; 2] {
    [
        2.0 * (instrument_canvas.origin[0] + p[0] * instrument_canvas.scale)
            / instrument_canvas.viewport[0]
            - 1.0,
        1.0 - 2.0 * (instrument_canvas.origin[1] + p[1] * instrument_canvas.scale)
            / instrument_canvas.viewport[1],
    ]
}
fn triangle(instrument_canvas: &mut InstrumentCanvas<'_>, p: [[f32; 2]; 3], color: [f32; 3]) {
    for p in p {
        let p = instrument_canvas_point(instrument_canvas, p);
        solid(instrument_canvas.vertices, p, color);
    }
}
fn rect(instrument_canvas: &mut InstrumentCanvas<'_>, a: [f32; 2], b: [f32; 2], color: [f32; 3]) {
    let a = instrument_canvas_point(instrument_canvas, a);
    let b = instrument_canvas_point(instrument_canvas, b);
    quad(instrument_canvas.vertices, a, b, color);
}
fn stroke(
    instrument_canvas: &mut InstrumentCanvas<'_>,
    a: [f32; 2],
    b: [f32; 2],
    width: f32,
    color: [f32; 3],
) {
    let delta = [b[0] - a[0], b[1] - a[1]];
    let len = delta[0].hypot(delta[1]).max(0.001);
    let n = [-delta[1] / len * width * 0.5, delta[0] / len * width * 0.5];
    let p = [
        [a[0] + n[0], a[1] + n[1]],
        [a[0] - n[0], a[1] - n[1]],
        [b[0] - n[0], b[1] - n[1]],
        [b[0] + n[0], b[1] + n[1]],
    ];
    triangle(instrument_canvas, [p[0], p[1], p[2]], color);
    triangle(instrument_canvas, [p[0], p[2], p[3]], color);
}
fn disc(
    instrument_canvas: &mut InstrumentCanvas<'_>,
    center: [f32; 2],
    radius: f32,
    color: [f32; 3],
) {
    for i in 0..80 {
        let point = |i: usize| {
            let a = i as f32 * std::f32::consts::TAU / 80.0;
            [center[0] + a.cos() * radius, center[1] + a.sin() * radius]
        };
        triangle(instrument_canvas, [center, point(i), point(i + 1)], color);
    }
}
fn label(
    instrument_canvas: &mut InstrumentCanvas<'_>,
    value: &str,
    center: [f32; 2],
    size: f32,
    color: [f32; 3],
) {
    font_label(
        instrument_canvas,
        &hud_atlas::get().labels,
        value,
        center,
        size,
        color,
    );
}
fn font_label(
    instrument_canvas: &mut InstrumentCanvas<'_>,
    font: &hud_atlas::Font,
    value: &str,
    center: [f32; 2],
    size: f32,
    color: [f32; 3],
) {
    let width = hud_atlas::width(font, value) * size;
    let origin = [center[0] - width * 0.5, center[1] - 3.5 * size];
    hud_atlas::draw(font, value, |sprite| {
        let positions = corners(sprite.bounds).map(|p| {
            instrument_canvas_point(
                instrument_canvas,
                [origin[0] + p[0] * size, origin[1] + p[1] * size],
            )
        });
        glyph_quad(instrument_canvas.vertices, positions, sprite.uv, color);
    });
}

fn dial_point(center: [f32; 2], radius: f32, fraction: f32) -> [f32; 2] {
    let angle = (135.0 + 270.0 * fraction.clamp(0.0, 1.0)).to_radians();
    [
        center[0] + radius * angle.cos(),
        center[1] + radius * angle.sin(),
    ]
}
fn dial_value(value: f32, maximum: f32) -> f32 {
    if value.is_finite() && maximum.is_finite() && maximum > 0.0 {
        (value / maximum).clamp(0.0, 1.0)
    } else {
        0.0
    }
}
fn dial_face(
    c: &mut InstrumentCanvas<'_>,
    center: [f32; 2],
    radius: f32,
    maximum: u32,
    step: u32,
    major: u32,
    redline: Option<f32>,
) {
    let ink = [0.89, 0.90, 0.85];
    let red = [1.0, 0.28, 0.18];
    disc(
        c,
        [center[0] + 1.0, center[1] + 2.0],
        radius + 2.0,
        [0.015, 0.02, 0.025],
    );
    disc(c, center, radius, [0.29, 0.34, 0.36]);
    disc(c, center, radius - 1.5, [0.07, 0.09, 0.10]);
    disc(c, center, radius - 4.0, [0.025, 0.035, 0.045]);
    if let Some(redline) = redline {
        let start = dial_value(redline, maximum as f32);
        for i in 0..32 {
            let a = start + (1.0 - start) * i as f32 / 32.0;
            let b = start + (1.0 - start) * (i + 1) as f32 / 32.0;
            stroke(
                c,
                dial_point(center, radius - 8.0, a),
                dial_point(center, radius - 8.0, b),
                3.0,
                red,
            );
        }
    }
    for value in (0..=maximum).step_by(step as usize) {
        let fraction = value as f32 / maximum as f32;
        let big = value % major == 0;
        let color = if redline.is_some_and(|r| value as f32 >= r) {
            red
        } else {
            ink
        };
        stroke(
            c,
            dial_point(center, radius - 8.0, fraction),
            dial_point(center, radius - if big { 17.0 } else { 12.0 }, fraction),
            if big { 1.8 } else { 1.0 },
            color,
        );
        // Leave breathing room for three-digit speed labels; retain a major
        // tick every 20 km/h but label every 40.
        if big && (maximum > 1000 || value % (major * 2) == 0) {
            let label = if maximum > 1000 {
                (value / 1000).to_string()
            } else {
                value.to_string()
            };
            font_label(
                c,
                &hud_atlas::get().digits,
                &label,
                dial_point(center, radius - 26.0, fraction),
                1.4,
                color,
            );
        }
    }
}
fn dial_needle(
    c: &mut InstrumentCanvas<'_>,
    center: [f32; 2],
    radius: f32,
    fraction: f32,
    color: [f32; 3],
) {
    let tip = dial_point(center, radius * 0.73, fraction);
    let dx = (tip[0] - center[0]) / (radius * 0.73);
    let dy = (tip[1] - center[1]) / (radius * 0.73);
    let rear = [center[0] - dx * 11.0, center[1] - dy * 11.0];
    triangle(
        c,
        [
            [rear[0] - dy * 2.0, rear[1] + dx * 2.0],
            [rear[0] + dy * 2.0, rear[1] - dx * 2.0],
            tip,
        ],
        color,
    );
    disc(c, center, 6.0, [0.12, 0.15, 0.17]);
    disc(c, center, 3.0, [0.65, 0.69, 0.69]);
}
fn append_instruments(v: &mut Vec<Vertex>, gauges: Gauges, viewport: [f32; 2]) {
    let scale = (viewport[0] / 1280.0).min(viewport[1] / 720.0);
    let mut c = InstrumentCanvas {
        vertices: v,
        viewport,
        scale,
        origin: [viewport[0] - 330.0 * scale, viewport[1] - 168.0 * scale],
    };
    let speed = [82.0, 82.0];
    let tach = [246.0, 88.0];
    dial_face(&mut c, speed, 76.0, 240, 10, 20, None);
    dial_face(
        &mut c,
        tach,
        68.0,
        8000,
        250,
        1000,
        Some(gauges.redline_rpm),
    );
    let muted = [0.56, 0.65, 0.68];
    label(&mut c, "KM/H", [82.0, 116.0], 1.1, muted);
    label(&mut c, "RPM X1000", [246.0, 118.0], 0.85, muted);
    let needle = [1.0, 0.28, 0.18];
    dial_needle(
        &mut c,
        speed,
        76.0,
        dial_value(gauges.speed_kmh.abs(), 240.0),
        needle,
    );
    dial_needle(
        &mut c,
        tach,
        68.0,
        dial_value(gauges.rpm, 8000.0),
        if gauges.rpm >= gauges.redline_rpm * 0.9 {
            [1.0, 0.55, 0.18]
        } else {
            needle
        },
    );
    rect(&mut c, [148.0, 122.0], [184.0, 166.0], [0.025, 0.04, 0.05]);
    label(&mut c, "GEAR", [166.0, 129.0], 0.95, muted);
    font_label(
        &mut c,
        &hud_atlas::get().digits,
        &if gauges.gear == phys_car::NEUTRAL_GEAR {
            "N".into()
        } else if gauges.gear == 0 {
            "R".into()
        } else {
            gauges.gear.to_string()
        },
        [166.0, 149.0],
        3.3,
        [0.15, 0.85, 0.95],
    );
}

fn append_pause(v: &mut Vec<Vertex>, pause: PauseMenu<'_>) {
    let white = [0.9, 0.95, 1.0];
    let cyan = [0.15, 0.85, 0.95];
    quad(v, [-0.76, -0.55], [0.76, 0.55], [0.025, 0.05, 0.075]);
    text(v, "Paused", [-0.68, 0.43], 0.009, cyan);
    let labels = [
        "Resume  [Esc]".to_string(),
        "Restart  [R]".into(),
        "Location / car  [L]".into(),
        format!("Audio: {}  [V]", pause.audio),
        "Recover  [H]".into(),
    ];
    for (r, label) in PAUSE_BOXES.into_iter().zip(labels) {
        menu_button(v, r, true, false);
        text(v, &label, [r[0] + 0.025, r[3] - 0.045], 0.0032, white);
    }
    if pause.city {
        text(
            v,
            "OpenStreetMap contributors / ODbL",
            [-0.68, -0.41],
            0.0025,
            [0.55, 0.66, 0.70],
        );
        if let Some(label) = pause.aerial_attribution {
            text(v, label, [-0.68, -0.47], 0.0022, [0.55, 0.66, 0.70]);
        }
    }
}

fn quad(v: &mut Vec<Vertex>, a: [f32; 2], b: [f32; 2], color: [f32; 3]) {
    for [x, y] in [
        [a[0], a[1]],
        [b[0], a[1]],
        [b[0], b[1]],
        [a[0], a[1]],
        [b[0], b[1]],
        [a[0], b[1]],
    ] {
        solid(v, [x, y], color);
    }
}

fn line(v: &mut Vec<Vertex>, a: [f32; 2], b: [f32; 2], width: f32, color: [f32; 3]) {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let len = dx.hypot(dy).max(0.0001);
    let n = [-dy / len * width, dx / len * width];
    let points = [
        [a[0] + n[0], a[1] + n[1]],
        [a[0] - n[0], a[1] - n[1]],
        [b[0] - n[0], b[1] - n[1]],
        [b[0] + n[0], b[1] + n[1]],
    ];
    for i in [0, 1, 2, 0, 2, 3] {
        solid(v, points[i], color);
    }
}

fn append_street_name(v: &mut Vec<Vertex>, name: &str) {
    let size = 0.0025;
    let available_width = MAP_CONTENT[1][0] - MAP_CONTENT[0][0];
    // Bound the label to two lines at a readable size, including long OSM names.
    let chars: Vec<_> = name.chars().collect();
    let mut lines = Vec::new();
    let mut start = 0;
    while start < chars.len() && lines.len() < 2 {
        let mut end = start;
        let mut width = 0.0;
        while end < chars.len() {
            width += font::width(font::labels(), &chars[end].to_string()) * size;
            if width > available_width {
                break;
            }
            end += 1;
        }
        if end < chars.len() {
            if let Some(space) = chars[start..end].iter().rposition(|ch| *ch == ' ') {
                if space > 0 {
                    end = start + space;
                }
            }
        }
        let mut line: String = chars[start..end].iter().collect();
        start = end;
        while start < chars.len() && chars[start] == ' ' {
            start += 1;
        }
        if lines.len() == 1 && start < chars.len() {
            while font::width(font::labels(), &(line.clone() + "...")) * size > available_width {
                line.pop();
            }
            line.push_str("...");
        }
        lines.push(line);
    }
    if lines.is_empty() {
        return;
    }
    let top = MAP_CONTENT[0][1] - 0.04;
    let center = (MAP_CONTENT[0][0] + MAP_CONTENT[1][0]) * 0.5;
    quad(
        v,
        [
            MAP_CONTENT[0][0] - 0.02,
            top - (lines.len() - 1) as f32 * 0.045 - 0.045,
        ],
        [MAP_CONTENT[1][0] + 0.02, MAP_CONTENT[0][1] - 0.02],
        [0.025, 0.05, 0.075],
    );
    for (i, line) in lines.iter().enumerate() {
        let x = center - font::width(font::labels(), line) * 0.5 * size;
        text(v, line, [x, top - i as f32 * 0.045], size, [0.9, 0.95, 1.0]);
    }
}

fn corners(bounds: [[f32; 2]; 2]) -> [[f32; 2]; 4] {
    let [a, b] = bounds;
    [a, [b[0], a[1]], b, [a[0], b[1]]]
}

fn glyph_quad(v: &mut Vec<Vertex>, positions: [[f32; 2]; 4], uv: [[f32; 2]; 2], color: [f32; 3]) {
    let uv = corners(uv);
    for i in [0, 1, 2, 0, 2, 3] {
        v.push(Vertex {
            position: [positions[i][0], positions[i][1], 0.0],
            color,
            uv: uv[i],
        });
    }
}

fn text(v: &mut Vec<Vertex>, value: &str, origin: [f32; 2], size: f32, color: [f32; 3]) {
    text_with_font(v, &hud_atlas::get().labels, value, origin, size, color);
}

fn text_with_font(
    v: &mut Vec<Vertex>,
    font: &hud_atlas::Font,
    value: &str,
    origin: [f32; 2],
    size: f32,
    color: [f32; 3],
) {
    hud_atlas::draw(font, value, |sprite| {
        let positions = corners(sprite.bounds)
            .map(|p| [origin[0] + p[0] * size, origin[1] - p[1] * size * 1.65]);
        glyph_quad(v, positions, sprite.uv, color);
    });
}

fn map_segments(world: &WorldLocation) -> Box<dyn Iterator<Item = (XYZ, XYZ)> + '_> {
    Box::new(
        world_roads::road_lines(world)
            .iter()
            .flat_map(|street| street.path.windows(2).map(|p| (p[0], p[1]))),
    )
}
fn map_segments_near(
    world: &WorldLocation,
    bounds: BoundingBoxXZ<f32>,
) -> Box<dyn Iterator<Item = (XYZ, XYZ)> + '_> {
    Box::new(world_roads::road_segments_in(world, bounds).into_iter())
}

pub fn google_maps_url(world: &WorldLocation, position: XYZ) -> Option<String> {
    let p = geometry::geographic_position(world, position)?;
    Some(format!(
        "https://www.google.com/maps/search/?api=1&query={:.7},{:.7}",
        p.lat, p.lon
    ))
}

/// Label the closest road only while the car is on or beside its surface.
fn street_name(world: &WorldLocation, p: XYZ) -> &str {
    let (index, road) = world_roads::nearest_road(world, p);
    if (p.x - road.point.x).hypot(p.z - road.point.z) > road.width * 0.5 + 5.0 {
        return "";
    }
    world_roads::road_name(world, index).unwrap_or_else(|| {
        if world_roads::road_lines(world)[index].osm_way_id.is_none() {
            world.display_name.as_str()
        } else {
            "UNNAMED ROAD"
        }
    })
}

#[cfg(test)]
mod tests {
    use crate::hud;
    use crate::track;
    use crate::vehicle_location;
    use crate::world;

    use super::*;
    use crate::test_support;

    #[test]
    #[ignore = "reports HUD generation time and geometry size"]
    fn hud_geometry_profile() {
        let mut vertices = Vec::new();
        let mut triangles = 0;
        let cold = std::time::Instant::now();
        let atlas = hud_atlas::get();
        eprintln!(
            "Atlas: {}x{}, {:.3} s to initialize",
            atlas.width,
            atlas.height,
            cold.elapsed().as_secs_f64()
        );
        let started = std::time::Instant::now();
        for frame in 0..1000 {
            vertices.clear();
            append_instruments(
                &mut vertices,
                Gauges {
                    rpm: 3000.0 + frame as f32,
                    redline_rpm: 6500.0,
                    speed_kmh: 90.0,
                    gear: 3,
                },
                [1280.0, 720.0],
            );
            append_street_name(&mut vertices, "ΛΕΩΦΟΡΟΣ ΑΡΧΙΕΠΙΣΚΟΠΟΥ ΜΑΚΑΡΙΟΥ III");
            std::hint::black_box(&vertices);
            triangles = vertices.len() / 3;
        }
        eprintln!(
            "HUD: {triangles} triangles, {} bytes, {:.3} ms/frame",
            std::mem::size_of_val(vertices.as_slice()),
            started.elapsed().as_secs_f64()
        );
    }

    #[test]
    fn map_zoom_factor_applies_to_every_location() {
        let track = track::try_track(&LocationId::Compact).unwrap();
        let other = track::try_track(&test_support::authored_id()).unwrap();
        let viewport = [1280.0, 720.0];

        let mut view =
            hud::full_map_view_new(&track, vehicle_location::spawn(&track).point, viewport);
        assert!(hud::zoom_full_map(&mut (view), 3.0));
        let factor = hud::zoom_factor(&view);
        assert!(factor > 1.0);

        for track in [&track, &other] {
            let mut view =
                hud::full_map_view_new(track, vehicle_location::spawn(track).point, viewport);
            assert!((hud::zoom_factor(&(view)) - 1.0).abs() < 0.000001);
            hud::set_zoom_factor(&mut view, factor);
            assert!((hud::zoom_factor(&(view)) - factor).abs() < 0.00001);
        }
    }

    #[test]
    fn location_pages_hit_only_visible_entries() {
        assert_eq!(
            location_menu_hit_test(-0.5, 0.76, CHOICES_PER_PAGE, CHOICES_PER_PAGE + 1),
            Some(MenuAction::Choice(CHOICES_PER_PAGE))
        );
        assert_eq!(
            location_menu_hit_test(-0.5, 0.66, CHOICES_PER_PAGE, CHOICES_PER_PAGE + 1),
            None
        );
        assert_eq!(location_menu_hit_test(-0.5, 0.71, 0, 4), None);
        assert_eq!(location_menu_hit_test(-0.5, 0.91, 0, 4), None);
        assert_eq!(
            location_menu_hit_test(-0.5, 0.56, 0, 4),
            Some(MenuAction::Choice(2))
        );
        assert_eq!(location_menu_hit_test(-0.5, -0.07, 0, 4), None);
    }

    #[test]
    fn car_pages_select_visible_rows_and_ignore_empty_rows() {
        let car_count = CHOICES_PER_PAGE + 2;
        for selected in 0..car_count {
            for slot in 0..CHOICES_PER_PAGE {
                let r = car_box(slot);
                let index = selected / CHOICES_PER_PAGE * CHOICES_PER_PAGE + slot;
                let expected = (index < car_count).then_some(MenuAction::Car(index));
                assert_eq!(
                    selection_menu_hit_test(0.5, (r[1] + r[3]) * 0.5, 0, 4, selected, car_count),
                    expected
                );
            }
            assert_eq!(
                selection_menu_hit_test(0.5, 0.91, 0, 4, selected, car_count),
                None
            );
        }
    }

    #[test]
    fn minimap_heading_keeps_forward_up_and_queries_rotated_corners() {
        let track = track::try_track(&LocationId::Compact).unwrap();
        for heading in [0.0, 0.7, 1.57, 3.14, -2.0] {
            let mut fit = hud::map_fit_new(
                &track,
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                [1280.0, 720.0],
            );
            fit.heading = Some(heading);
            let center = hud::map_fit_point(
                &fit,
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let forward = hud::map_fit_point(
                &fit,
                XYZ {
                    x: heading.sin() * 30.0,
                    y: 0.0,
                    z: heading.cos() * 30.0,
                },
            );
            assert!((forward[0] - center[0]).abs() < 0.00001);
            assert!(forward[1] > center[1]);
            let camera_right = hud::map_fit_point(
                &fit,
                XYZ {
                    x: -heading.cos() * 30.0,
                    y: 0.0,
                    z: heading.sin() * 30.0,
                },
            );
            assert!(camera_right[0] > center[0]);
            assert!((camera_right[1] - center[1]).abs() < 0.00001);
            let bounds = hud::world_bounds(&fit);
            for x in [MAP_CONTENT[0][0], MAP_CONTENT[1][0]] {
                for y in [MAP_CONTENT[0][1], MAP_CONTENT[1][1]] {
                    let right = (x - center[0]) / fit.scale[0];
                    let ahead = (y - center[1]) / fit.scale[1];
                    let world = [
                        -right * heading.cos() + ahead * heading.sin(),
                        right * heading.sin() + ahead * heading.cos(),
                    ];
                    for i in 0..2 {
                        assert!(
                            world[i] >= [bounds.min.x, bounds.min.z][i] - 0.001
                                && world[i] <= [bounds.max.x, bounds.max.z][i] + 0.001
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn full_map_zoom_matches_minimap_and_can_fit_every_road() {
        for id in test_support::location_ids() {
            let track = track::try_track(&id).unwrap();
            for player in [
                vehicle_location::spawn(&track).point,
                XYZ {
                    x: world::bounding_box(&track).max.x + 100.0,
                    y: 0.0,
                    z: world::bounding_box(&track).max.z + 100.0,
                },
            ] {
                for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
                    let mini = hud::map_fit_new(&track, player, viewport);
                    let mut view = hud::full_map_view_new(&track, player, viewport);
                    assert_eq!(view.fit.scale, mini.scale);
                    assert_eq!(view.fit.center, mini.center);
                    let center = hud::map_fit_point(&view.fit, player);
                    assert!(center[0].abs() < 0.00001);
                    assert!((center[1] - 0.015).abs() < 0.00001);
                    assert!(hud::zoom_full_map(&mut (view), 1.0));
                    assert!((view.fit.scale[0] / mini.scale[0] - 1.25).abs() < 0.00001);
                    assert_eq!(hud::map_fit_point(&(view.fit), player), center);
                    assert!(hud::zoom_full_map(&mut (view), -1.0));
                    assert!((view.fit.scale[0] - mini.scale[0]).abs() < 0.00001);
                    assert!(hud::zoom_full_map(&mut (view), 100.0));
                    assert!(!hud::zoom_full_map(&mut (view), 1.0));
                    assert_eq!(view.fit.scale[0], view.max_scale);
                    assert!(hud::zoom_full_map(&mut (view), -100.0));
                    assert!(!hud::zoom_full_map(&mut (view), -1.0));
                    assert_eq!(view.fit.scale[0], view.min_scale);
                    for invalid in [0.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                        assert!(!hud::zoom_full_map(&mut (view), invalid));
                    }
                    assert_eq!(hud::map_fit_point(&(view.fit), player), center);
                    let fit = &view.fit;
                    for p in hud::map_segments(&track)
                        .flat_map(|(a, b)| [a, b])
                        .chain([player])
                    {
                        let point = hud::map_fit_point(fit, p);
                        for axis in 0..2 {
                            assert!(point[axis] > FULL_MAP_CONTENT[0][axis]);
                            assert!(point[axis] < FULL_MAP_CONTENT[1][axis]);
                        }
                    }
                    let center = hud::map_fit_point(fit, player);
                    let east = hud::map_fit_point(
                        fit,
                        XYZ {
                            x: player.x + 100.0,
                            y: 0.0,
                            z: player.z,
                        },
                    );
                    let south = hud::map_fit_point(
                        fit,
                        XYZ {
                            x: player.x,
                            y: 0.0,
                            z: player.z + 100.0,
                        },
                    );
                    let width = (east[0] - center[0]) * viewport[0];
                    let height = (south[1] - center[1]).abs() * viewport[1];
                    assert!((width - height).abs() < 0.001);
                    assert_eq!(south[1] < center[1], id == test_support::city_id());
                }
            }
        }
    }

    #[test]
    fn full_map_zoom_clips_road_strokes_inside_the_panel() {
        let track = track::try_track(&test_support::authored_id()).unwrap();
        let player = vehicle_location::spawn(&track).point;
        for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
            for steps in [-100.0, 0.0, 100.0] {
                let mut view = hud::full_map_view_new(&track, player, viewport);
                hud::zoom_full_map(&mut view, steps);
                let mut vertices = Vec::new();
                append_full_map(
                    &mut vertices,
                    Map {
                        track: &track,
                        player,
                    },
                    0.0,
                    viewport,
                    &view.fit,
                );
                let roads: Vec<_> = vertices
                    .iter()
                    .filter(|p| p.color == [0.40, 0.55, 0.60])
                    .map(|p| p.position)
                    .collect();
                assert!(!roads.is_empty());
                for p in roads {
                    for axis in 0..2 {
                        assert!(p[axis] >= FULL_MAP_CONTENT[0][axis] - 0.00001);
                        assert!(p[axis] <= FULL_MAP_CONTENT[1][axis] + 0.00001);
                    }
                }
            }
        }
    }
}
