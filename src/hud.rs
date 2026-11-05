use crate::{
    cars::CarModel,
    track::{Track, TrackId},
};
use std::ffi::CString;

#[cfg(test)]
pub const LOCATIONS: [TrackId; 2] = [TrackId::Scenic, TrackId::Engomi];
#[cfg(test)]
pub fn available_locations(city_enabled: bool) -> &'static [TrackId] {
    if city_enabled {
        &LOCATIONS
    } else {
        &LOCATIONS[..1]
    }
}
const CHOICES_PER_PAGE: usize = 3;
fn choice_box(index: usize, left: f32) -> [f32; 4] {
    let top = -0.02 - index as f32 * 0.125;
    [left, top - 0.10, left + 0.88, top]
}
fn location_box(index: usize) -> [f32; 4] {
    choice_box(index, -0.92)
}
fn car_box(index: usize) -> [f32; 4] {
    choice_box(index, 0.06)
}
const START_BOX: [f32; 4] = [-0.92, -0.65, 0.94, -0.50];
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MenuAction {
    #[cfg(test)]
    Location(TrackId),
    Choice(usize),
    Page(i32),
    CarPage(i32),
    Car(CarModel),
    Start,
}
pub struct MainMenu<'a> {
    pub locations: &'a [crate::city_assets::MenuLocation],
    pub selected: usize,
    pub car: CarModel,
    pub focus: usize,
}
fn page_hit_test(x: f32, y: f32, left: f32) -> Option<i32> {
    if (0.09..=0.19).contains(&y) {
        if (left + 0.43..=left + 0.52).contains(&x) {
            return Some(-1);
        }
        if (left + 0.76..=left + 0.88).contains(&x) {
            return Some(1);
        }
    }
    None
}
pub fn location_menu_hit_test(x: f32, y: f32, selected: usize, count: usize) -> Option<MenuAction> {
    if let Some(direction) = page_hit_test(x, y, -0.92) {
        return Some(MenuAction::Page(direction));
    }
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
    car: CarModel,
) -> Option<MenuAction> {
    if let Some(direction) = page_hit_test(x, y, 0.06) {
        return Some(MenuAction::CarPage(direction));
    }
    let page_start = crate::cars::ALL
        .into_iter()
        .position(|model| model == car)
        .unwrap_or(0)
        / CHOICES_PER_PAGE
        * CHOICES_PER_PAGE;
    for slot in 0..CHOICES_PER_PAGE {
        let r = car_box(slot);
        if x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3] {
            return crate::cars::ALL
                .into_iter()
                .nth(page_start + slot)
                .map(MenuAction::Car);
        }
    }
    location_menu_hit_test(x, y, selected, count)
}
#[cfg(test)]
pub fn menu_hit_test(x: f32, y: f32) -> Option<MenuAction> {
    let inside = |r: [f32; 4]| x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3];
    for (i, id) in LOCATIONS.into_iter().enumerate() {
        let r = location_box(i);
        if inside(r) {
            return Some(MenuAction::Location(id));
        }
    }
    for i in 0..crate::cars::ALL.len().min(CHOICES_PER_PAGE) {
        let r = car_box(i);
        if inside(r) {
            return Some(MenuAction::Car(crate::cars::ALL[i]));
        }
    }
    if inside(START_BOX) {
        return Some(MenuAction::Start);
    }
    None
}

pub struct Map<'a> {
    pub track: &'a Track,
    pub player: [f32; 3],
}

#[derive(Clone, Copy)]
pub struct Gauges {
    pub rpm: f32,
    pub redline_rpm: f32,
    pub speed_kmh: f32,
    pub gear: u8,
}

const PAUSE_BOXES: [[f32; 4]; 6] = [
    [-0.68, 0.06, -0.04, 0.20],
    [0.04, 0.06, 0.68, 0.20],
    [-0.68, -0.13, -0.04, 0.01],
    [0.04, -0.13, 0.68, 0.01],
    [-0.68, -0.32, -0.04, -0.18],
    [0.04, -0.32, 0.68, -0.18],
];
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PauseAction {
    Resume,
    Restart,
    Selection,
    Audio,
    Camera,
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
                PauseAction::Camera,
                PauseAction::Recover,
            ][i]
        })
}
pub struct PauseMenu<'a> {
    pub city: bool,
    pub view: &'a str,
    pub audio: &'a str,
}

const MAP_CONTENT: [[f32; 2]; 2] = [[0.61, 0.49], [0.95, 0.88]];
const FULL_MAP_CONTENT: [[f32; 2]; 2] = [[-0.94, -0.72], [0.94, 0.75]];
const MAP_WIDTH_METRES: f32 = 400.0;

struct MapFit {
    center: [f32; 2],
    scale: [f32; 2],
    north_up: bool,
    heading: Option<f32>,
    content: [[f32; 2]; 2],
}
impl MapFit {
    fn new(track: &Track, player: [f32; 3], viewport: [f32; 2]) -> Self {
        let x_scale = (MAP_CONTENT[1][0] - MAP_CONTENT[0][0]) / MAP_WIDTH_METRES;
        Self {
            center: [player[0], player[2]],
            scale: [x_scale, x_scale * viewport[0] / viewport[1]],
            north_up: track.city().is_some(),
            heading: None,
            content: MAP_CONTENT,
        }
    }
    fn world_bounds(&self) -> [[f32; 2]; 2] {
        let mut x = (self.content[1][0] - self.content[0][0]) * 0.5 / self.scale[0];
        let mut z = (self.content[1][1] - self.content[0][1]) * 0.5 / self.scale[1];
        if let Some(heading) = self.heading {
            let (sin, cos) = heading.sin_cos();
            (x, z) = (x * cos.abs() + z * sin.abs(), x * sin.abs() + z * cos.abs());
        }
        [
            [self.center[0] - x, self.center[1] - z],
            [self.center[0] + x, self.center[1] + z],
        ]
    }
    fn point(&self, p: [f32; 3]) -> [f32; 2] {
        let dx = p[0] - self.center[0];
        let dz = p[2] - self.center[1];
        let [x, y] = if let Some(heading) = self.heading {
            let (sin, cos) = heading.sin_cos();
            [dx * cos - dz * sin, dx * sin + dz * cos]
        } else {
            [dx, dz * if self.north_up { -1.0 } else { 1.0 }]
        };
        [
            (self.content[0][0] + self.content[1][0]) * 0.5 + x * self.scale[0],
            (self.content[0][1] + self.content[1][1]) * 0.5 + y * self.scale[1],
        ]
    }
}

struct FullMapView {
    copy_status: Option<&'static str>,
    fit: MapFit,
    min_scale: f32,
    max_scale: f32,
}
impl FullMapView {
    fn new(track: &Track, player: [f32; 3], viewport: [f32; 2]) -> Self {
        // Keep the minimap's metres per pixel, expanding the visible area.
        let mut fit = MapFit::new(track, player, viewport);
        fit.content = FULL_MAP_CONTENT;
        let mut bounds = track.bounds;
        for p in track.map_segments().flat_map(|(a, b)| [a, b]) {
            for (axis, coordinate) in [p[0], p[2]].into_iter().enumerate() {
                bounds[0][axis] = bounds[0][axis].min(coordinate);
                bounds[1][axis] = bounds[1][axis].max(coordinate);
            }
        }
        // Zooming out can reveal the entire network while keeping the car centered.
        let size: [f32; 2] = std::array::from_fn(|i| {
            2.0 * (bounds[0][i] - fit.center[i])
                .abs()
                .max((bounds[1][i] - fit.center[i]).abs())
                .max(1.0)
        });
        let available: [f32; 2] =
            std::array::from_fn(|i| (FULL_MAP_CONTENT[1][i] - FULL_MAP_CONTENT[0][i]) * 0.9);
        let min_scale = (available[0] / size[0])
            .min(available[1] / (size[1] * viewport[0] / viewport[1]))
            .min(fit.scale[0]);
        Self {
            copy_status: None,
            min_scale,
            max_scale: fit.scale[0] * 16.0,
            fit,
        }
    }

    fn zoom(&mut self, steps: f32) -> bool {
        if !steps.is_finite() || steps == 0.0 {
            return false;
        }
        let scale = (self.fit.scale[0] * 1.25_f32.powf(steps.clamp(-100.0, 100.0)))
            .clamp(self.min_scale, self.max_scale);
        if scale == self.fit.scale[0] {
            return false;
        }
        self.fit.scale[1] *= scale / self.fit.scale[0];
        self.fit.scale[0] = scale;
        true
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
fn map_marker(v: &mut Vec<f32>, point: [f32; 2], size: f32, color: [f32; 3]) {
    let min = std::array::from_fn(|i| (point[i] - size).max(MAP_CONTENT[0][i]));
    let max = std::array::from_fn(|i| (point[i] + size).min(MAP_CONTENT[1][i]));
    if min[0] < max[0] && min[1] < max[1] {
        quad(v, min, max, color);
    }
}

pub struct Hud {
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

impl Hud {
    pub fn new() -> Self {
        unsafe {
            let (mut vao, mut vbo) = (0, 0);
            gl::GenVertexArrays(1, &mut vao);
            gl::GenBuffers(1, &mut vbo);
            gl::BindVertexArray(vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
            gl::EnableVertexAttribArray(0);
            gl::VertexAttribPointer(0, 3, gl::FLOAT, gl::FALSE, 24, std::ptr::null());
            gl::EnableVertexAttribArray(1);
            gl::VertexAttribPointer(1, 3, gl::FLOAT, gl::FALSE, 24, 12usize as *const _);
            Self {
                vao,
                vbo,
                full_map_vertices: 0,
                full_map_view: None,
            }
        }
    }

    pub fn draw(
        &self,
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
        let mut fit = MapFit::new(
            map.track,
            map.player,
            [viewport[2].max(1) as f32, viewport[3].max(1) as f32],
        );
        fit.heading = Some(heading);
        let width = if map.track.city().is_some() {
            0.0025
        } else {
            0.006
        };
        for (from, to) in map.track.map_segments_near(fit.world_bounds()) {
            if let Some([a, b]) = clip_map_line(fit.point(from), fit.point(to), width) {
                line(&mut v, a, b, width, [0.4, 0.49, 0.52]);
            }
        }
        map_marker(&mut v, fit.point(map.player), 0.013, [1.0, 0.25, 0.12]);

        let street = map
            .track
            .city()
            .map_or(map.track.name, |city| city.street_name(map.player));
        append_street_name(&mut v, street);
        if let Some(pause) = pause {
            append_pause(&mut v, pause);
        }
        self.upload(program, &v);
    }

    pub fn prepare_full_map(&mut self, map: Map<'_>, heading: f32, viewport: [f32; 2]) {
        self.full_map_view = Some(FullMapView::new(map.track, map.player, viewport));
        self.rebuild_full_map(map, heading, viewport);
    }

    pub fn zoom_full_map(&mut self, map: Map<'_>, heading: f32, viewport: [f32; 2], steps: f32) {
        if self
            .full_map_view
            .as_mut()
            .is_some_and(|view| view.zoom(steps))
        {
            self.rebuild_full_map(map, heading, viewport);
        }
    }

    pub fn map_copy_status(
        &mut self,
        map: Map<'_>,
        heading: f32,
        viewport: [f32; 2],
        status: &'static str,
    ) {
        if let Some(view) = self.full_map_view.as_mut() {
            view.copy_status = Some(status);
            self.rebuild_full_map(map, heading, viewport);
        }
    }

    fn rebuild_full_map(&mut self, map: Map<'_>, heading: f32, viewport: [f32; 2]) {
        let view = self.full_map_view.as_ref().expect("full map is open");
        let mut vertices = Vec::new();
        let hint = view.copy_status.unwrap_or_else(|| {
            if map
                .track
                .city()
                .and_then(|city| city.google_maps_url(map.player))
                .is_some()
            {
                "C COPY GOOGLE MAPS URL"
            } else {
                "NO GOOGLE MAPS LOCATION"
            }
        });
        append_full_map(&mut vertices, map, heading, viewport, &view.fit);
        text(&mut vertices, hint, [-0.90, -0.89], 0.003, [0.9, 0.95, 1.0]);
        // Reuse the uploaded map until it opens again or the zoom changes.
        self.set_vertices(&vertices);
        self.full_map_vertices = (vertices.len() / 6) as i32;
    }

    pub fn draw_full_map(&self, program: u32) {
        self.draw_vertices(program, self.full_map_vertices);
    }

    pub fn draw_menu(&self, program: u32, menu: MainMenu) {
        let mut v = Vec::new();
        let cyan = [0.15, 0.85, 0.95];
        let white = [0.9, 0.95, 1.0];
        let muted = [0.55, 0.66, 0.70];
        text(&mut v, "CHOOSE YOUR DRIVE", [-0.92, 0.89], 0.006, white);
        let name = menu.car.spec().name;
        let size = 0.005_f32.min(1.5 / (name.chars().count().max(1) as f32 * 6.0));
        text(
            &mut v,
            name,
            [-(name.chars().count() as f32) * 3.0 * size, 0.82],
            size,
            white,
        );
        for left in [-0.96, 0.02] {
            quad(
                &mut v,
                [left, -0.42],
                [left + 0.96, 0.21],
                [0.025, 0.05, 0.075],
            );
        }
        choice_header(
            &mut v,
            "LOCATION",
            -0.92,
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
        let selected_car = menu.car.index();
        choice_header(&mut v, "CAR", 0.06, selected_car, crate::cars::ALL.len());
        for (i, car) in crate::cars::ALL
            .into_iter()
            .enumerate()
            .skip(selected_car / CHOICES_PER_PAGE * CHOICES_PER_PAGE)
            .take(CHOICES_PER_PAGE)
        {
            choice_button(
                &mut v,
                car_box(i % CHOICES_PER_PAGE),
                car.spec().name,
                menu.car == car,
                menu.focus == 1,
            );
        }
        menu_button(&mut v, START_BOX, true, menu.focus == 2);
        text(&mut v, "START DRIVING", [-0.19, -0.54], 0.005, cyan);
        if menu.locations[menu.selected].track == TrackId::Engomi {
            text(
                &mut v,
                "OPENSTREETMAP CONTRIBUTORS / ODBL",
                [-0.92, -0.73],
                0.0025,
                muted,
            );
            text(
                &mut v,
                "TERRAIN: MAPZEN / USGS / EU COPERNICUS",
                [-0.92, -0.77],
                0.0025,
                muted,
            );
            if crate::aerial::source_available() {
                text(
                    &mut v,
                    crate::aerial::attribution_label(),
                    [-0.92, -0.81],
                    0.0024,
                    muted,
                );
            }
        }
        text(
            &mut v,
            "LEFT LOCATION / RIGHT CAR / UP-DOWN BROWSE / ENTER START / Q QUIT",
            [-0.92, -0.91],
            0.0032,
            white,
        );
        self.upload(program, &v);
    }

    fn upload(&self, program: u32, v: &[f32]) {
        self.set_vertices(v);
        self.draw_vertices(program, (v.len() / 6) as i32);
    }

    fn set_vertices(&self, v: &[f32]) {
        unsafe {
            gl::BindBuffer(gl::ARRAY_BUFFER, self.vbo);
            gl::BufferData(
                gl::ARRAY_BUFFER,
                std::mem::size_of_val(v) as isize,
                v.as_ptr() as *const _,
                gl::STREAM_DRAW,
            );
        }
    }

    fn draw_vertices(&self, program: u32, count: i32) {
        let identity = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        unsafe {
            gl::Disable(gl::DEPTH_TEST);
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.vbo);
            gl::UseProgram(program);
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
        }
    }
}

fn append_full_map(v: &mut Vec<f32>, map: Map<'_>, heading: f32, viewport: [f32; 2], fit: &MapFit) {
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
    text(v, "FULL MAP", [-0.94, 0.92], 0.006, cyan);
    let name_size = 0.004_f32.min(0.94 / (map.track.name.chars().count().max(1) as f32 * 6.0));
    text(v, map.track.name, [-0.60, 0.90], name_size, white);
    text(v, "M / ESC BACK", [0.49, 0.90], 0.004, white);

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
    for (a, b) in map.track.map_segments_near(fit.world_bounds()) {
        if let Some([a, b]) = clip_line(fit.point(a), fit.point(b), fit.content, padding) {
            canvas.stroke(pixel(a), pixel(b), road_width, [0.40, 0.55, 0.60]);
        }
    }
    let player = pixel(fit.point(map.player));
    canvas.disc(player, 15.0, white);
    canvas.disc(player, 12.0, [0.025, 0.05, 0.075]);
    let forward = [
        heading.sin(),
        heading.cos() * if fit.north_up { 1.0 } else { -1.0 },
    ];
    let right = [-forward[1], forward[0]];
    let tip = std::array::from_fn(|i| player[i] + forward[i] * 10.0);
    let left = std::array::from_fn(|i| player[i] - forward[i] * 7.0 - right[i] * 7.0);
    let right = std::array::from_fn(|i| player[i] - forward[i] * 7.0 + right[i] * 7.0);
    canvas.triangle([tip, left, right], orange);
    canvas.disc(pixel([-0.925, -0.855]), 5.0, orange);
    text(v, "YOUR POSITION", [-0.90, -0.83], 0.0035, white);
    text(v, "DRIVING PAUSED", [-0.54, -0.83], 0.0035, muted);
    text(v, "WHEEL / +/- ZOOM", [-0.14, -0.83], 0.0035, white);
    if fit.north_up {
        text(v, "N", [0.885, 0.68], 0.0045, cyan);
        line(v, [0.90, 0.54], [0.90, 0.61], 0.002, cyan);
        line(v, [0.90, 0.61], [0.89, 0.58], 0.002, cyan);
        line(v, [0.90, 0.61], [0.91, 0.58], 0.002, cyan);
        text(
            v,
            "MAP DATA: OPENSTREETMAP CONTRIBUTORS / OPENSTREETMAP.ORG/COPYRIGHT / ODBL",
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

fn choice_header(v: &mut Vec<f32>, label: &str, left: f32, selected: usize, count: usize) {
    let cyan = [0.15, 0.85, 0.95];
    text(v, label, [left, 0.14], 0.0038, cyan);
    text(
        v,
        &format!(
            "{} / {}",
            selected / CHOICES_PER_PAGE + 1,
            count.div_ceil(CHOICES_PER_PAGE)
        ),
        [left + 0.55, 0.14],
        0.0033,
        cyan,
    );
    text(v, "<", [left + 0.46, 0.14], 0.0035, cyan);
    text(v, ">", [left + 0.80, 0.14], 0.0035, cyan);
}
fn choice_button(v: &mut Vec<f32>, r: [f32; 4], label: &str, selected: bool, focused: bool) {
    menu_button(v, r, selected, focused && selected);
    let size = 0.0038_f32.min((r[2] - r[0] - 0.07) / (label.chars().count().max(1) as f32 * 6.0));
    text(
        v,
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

fn menu_button(v: &mut Vec<f32>, r: [f32; 4], selected: bool, focused: bool) {
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
    vertices: &'a mut Vec<f32>,
    viewport: [f32; 2],
    scale: f32,
    origin: [f32; 2],
}
impl InstrumentCanvas<'_> {
    fn point(&self, p: [f32; 2]) -> [f32; 2] {
        [
            2.0 * (self.origin[0] + p[0] * self.scale) / self.viewport[0] - 1.0,
            1.0 - 2.0 * (self.origin[1] + p[1] * self.scale) / self.viewport[1],
        ]
    }
    fn triangle(&mut self, p: [[f32; 2]; 3], color: [f32; 3]) {
        for p in p {
            let p = self.point(p);
            self.vertices
                .extend_from_slice(&[p[0], p[1], 0.0, color[0], color[1], color[2]]);
        }
    }
    fn rect(&mut self, a: [f32; 2], b: [f32; 2], color: [f32; 3]) {
        let a = self.point(a);
        let b = self.point(b);
        quad(self.vertices, a, b, color);
    }
    fn stroke(&mut self, a: [f32; 2], b: [f32; 2], width: f32, color: [f32; 3]) {
        let delta = [b[0] - a[0], b[1] - a[1]];
        let len = delta[0].hypot(delta[1]).max(0.001);
        let n = [-delta[1] / len * width * 0.5, delta[0] / len * width * 0.5];
        let p = [
            [a[0] + n[0], a[1] + n[1]],
            [a[0] - n[0], a[1] - n[1]],
            [b[0] - n[0], b[1] - n[1]],
            [b[0] + n[0], b[1] + n[1]],
        ];
        self.triangle([p[0], p[1], p[2]], color);
        self.triangle([p[0], p[2], p[3]], color);
    }
    fn disc(&mut self, center: [f32; 2], radius: f32, color: [f32; 3]) {
        for i in 0..80 {
            let point = |i: usize| {
                let a = i as f32 * std::f32::consts::TAU / 80.0;
                [center[0] + a.cos() * radius, center[1] + a.sin() * radius]
            };
            self.triangle([center, point(i), point(i + 1)], color);
        }
    }
    fn label(&mut self, value: &str, center: [f32; 2], size: f32, color: [f32; 3]) {
        let width = (value.chars().count() as f32 * 6.0 - 1.0) * size;
        let origin = [center[0] - width * 0.5, center[1] - 3.5 * size];
        for (i, ch) in value.chars().enumerate() {
            for (row, bits) in glyph(ch).iter().enumerate() {
                for col in 0..5 {
                    if bits & (1 << (4 - col)) != 0 {
                        let x = origin[0] + (i * 6 + col) as f32 * size;
                        let y = origin[1] + row as f32 * size;
                        self.rect([x, y], [x + size * 0.88, y + size * 0.88], color);
                    }
                }
            }
        }
    }
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
    c.disc(
        [center[0] + 1.0, center[1] + 2.0],
        radius + 2.0,
        [0.015, 0.02, 0.025],
    );
    c.disc(center, radius, [0.29, 0.34, 0.36]);
    c.disc(center, radius - 1.5, [0.07, 0.09, 0.10]);
    c.disc(center, radius - 4.0, [0.025, 0.035, 0.045]);
    if let Some(redline) = redline {
        let start = dial_value(redline, maximum as f32);
        for i in 0..32 {
            let a = start + (1.0 - start) * i as f32 / 32.0;
            let b = start + (1.0 - start) * (i + 1) as f32 / 32.0;
            c.stroke(
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
        c.stroke(
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
            c.label(
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
    c.triangle(
        [
            [rear[0] - dy * 2.0, rear[1] + dx * 2.0],
            [rear[0] + dy * 2.0, rear[1] - dx * 2.0],
            tip,
        ],
        color,
    );
    c.disc(center, 6.0, [0.12, 0.15, 0.17]);
    c.disc(center, 3.0, [0.65, 0.69, 0.69]);
}
fn append_instruments(v: &mut Vec<f32>, gauges: Gauges, viewport: [f32; 2]) {
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
    c.label("KM/H", [82.0, 116.0], 1.1, muted);
    c.label("RPM X1000", [246.0, 118.0], 0.85, muted);
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
    c.rect([148.0, 122.0], [184.0, 166.0], [0.025, 0.04, 0.05]);
    c.label("GEAR", [166.0, 129.0], 0.95, muted);
    c.label(
        &if gauges.gear == crate::vehicle::NEUTRAL_GEAR {
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

fn append_pause(v: &mut Vec<f32>, pause: PauseMenu<'_>) {
    let white = [0.9, 0.95, 1.0];
    let cyan = [0.15, 0.85, 0.95];
    quad(v, [-0.76, -0.55], [0.76, 0.55], [0.025, 0.05, 0.075]);
    text(v, "PAUSED", [-0.68, 0.43], 0.009, cyan);
    let labels = [
        "RESUME  [ESC]".to_string(),
        "RESTART  [R]".into(),
        "LOCATION / CAR  [L]".into(),
        format!("AUDIO: {}  [V]", pause.audio),
        format!("CAMERA: {}  [C]", pause.view),
        "RECOVER  [H]".into(),
    ];
    for (r, label) in PAUSE_BOXES.into_iter().zip(labels) {
        menu_button(v, r, true, false);
        text(v, &label, [r[0] + 0.025, r[3] - 0.045], 0.0032, white);
    }
    if pause.city {
        text(
            v,
            "OPENSTREETMAP CONTRIBUTORS / ODBL",
            [-0.68, -0.41],
            0.0025,
            [0.55, 0.66, 0.70],
        );
        if crate::aerial::source_available() {
            text(
                v,
                crate::aerial::attribution_label(),
                [-0.68, -0.47],
                0.0022,
                [0.55, 0.66, 0.70],
            );
        }
    }
}

fn quad(v: &mut Vec<f32>, a: [f32; 2], b: [f32; 2], color: [f32; 3]) {
    for [x, y] in [
        [a[0], a[1]],
        [b[0], a[1]],
        [b[0], b[1]],
        [a[0], a[1]],
        [b[0], b[1]],
        [a[0], b[1]],
    ] {
        v.extend_from_slice(&[x, y, 0.0, color[0], color[1], color[2]]);
    }
}

fn line(v: &mut Vec<f32>, a: [f32; 2], b: [f32; 2], width: f32, color: [f32; 3]) {
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
        v.extend_from_slice(&[
            points[i][0],
            points[i][1],
            0.0,
            color[0],
            color[1],
            color[2],
        ]);
    }
}

fn append_street_name(v: &mut Vec<f32>, name: &str) {
    let name = name.to_uppercase();
    // Bound the label to two lines at a readable size, including long OSM names.
    let chars: Vec<_> = name.chars().collect();
    let mut lines = Vec::new();
    let mut start = 0;
    while start < chars.len() && lines.len() < 2 {
        let mut end = (start + 32).min(chars.len());
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
            line = line.chars().take(29).collect::<String>() + "...";
        }
        lines.push(line);
    }
    if lines.is_empty() {
        return;
    }
    let size = 0.0043;
    let top = -0.89 + (lines.len() - 1) as f32 * 0.075;
    let width = lines.iter().map(|s| s.chars().count()).max().unwrap_or(0) as f32 * 6.0 * size;
    quad(
        v,
        [-width * 0.5 - 0.018, -0.96],
        [width * 0.5 + 0.018, top + 0.025],
        [0.025, 0.05, 0.075],
    );
    for (i, line) in lines.iter().enumerate() {
        let x = -(line.chars().count() as f32) * 3.0 * size;
        text(v, line, [x, top - i as f32 * 0.075], size, [0.9, 0.95, 1.0]);
    }
}

fn text(v: &mut Vec<f32>, value: &str, origin: [f32; 2], size: f32, color: [f32; 3]) {
    for (index, ch) in value.chars().enumerate() {
        for (row, bits) in glyph(ch).iter().enumerate() {
            for column in 0..5 {
                if bits & (1 << (4 - column)) != 0 {
                    let x = origin[0] + (index * 6 + column) as f32 * size;
                    let y = origin[1] - row as f32 * size * 1.65;
                    quad(v, [x, y - size * 1.5], [x + size * 0.9, y], color);
                }
            }
        }
    }
}

pub(crate) fn glyph(ch: char) -> [u8; 7] {
    let ch = match ch {
        'Α' | 'Ά' | 'Á' | 'À' | 'Â' | 'Ä' => 'A',
        'Β' => 'B',
        'Ε' | 'Έ' | 'É' | 'È' | 'Ê' => 'E',
        'Ζ' => 'Z',
        'Η' | 'Ή' => 'H',
        'Ι' | 'Ί' | 'Ϊ' | 'İ' | 'Í' => 'I',
        'Κ' => 'K',
        'Μ' => 'M',
        'Ν' => 'N',
        'Ο' | 'Ό' | 'Ó' | 'Ö' => 'O',
        'Ρ' => 'P',
        'Τ' => 'T',
        'Υ' | 'Ύ' | 'Ϋ' => 'Y',
        'Χ' => 'X',
        'Ü' | 'Ú' => 'U',
        'Ç' => 'C',
        'Ş' => 'S',
        'Ğ' => 'G',
        _ => ch,
    };
    match ch {
        'Γ' => [31, 16, 16, 16, 16, 16, 16],
        'Δ' => [4, 10, 10, 17, 17, 17, 31],
        'Θ' => [14, 17, 17, 31, 17, 17, 14],
        'Λ' => [4, 10, 10, 17, 17, 17, 17],
        'Ξ' => [31, 0, 0, 14, 0, 0, 31],
        'Π' => [31, 17, 17, 17, 17, 17, 17],
        'Σ' => [31, 16, 8, 4, 8, 16, 31],
        'Φ' => [4, 14, 21, 21, 21, 14, 4],
        'Ψ' => [21, 21, 21, 14, 4, 4, 4],
        'Ω' | 'Ώ' => [14, 17, 17, 17, 10, 10, 27],
        '\'' => [4, 4, 0, 0, 0, 0, 0],
        'A' => [14, 17, 17, 31, 17, 17, 17],
        'B' => [30, 17, 17, 30, 17, 17, 30],
        'C' => [14, 17, 16, 16, 16, 17, 14],
        'D' => [30, 17, 17, 17, 17, 17, 30],
        'E' => [31, 16, 16, 30, 16, 16, 31],
        'F' => [31, 16, 16, 30, 16, 16, 16],
        'G' => [14, 17, 16, 23, 17, 17, 15],
        'H' => [17, 17, 17, 31, 17, 17, 17],
        'I' => [14, 4, 4, 4, 4, 4, 14],
        'J' => [7, 2, 2, 2, 2, 18, 12],
        'K' => [17, 18, 20, 24, 20, 18, 17],
        'L' => [16, 16, 16, 16, 16, 16, 31],
        'M' => [17, 27, 21, 21, 17, 17, 17],
        'N' => [17, 25, 21, 19, 17, 17, 17],
        'O' => [14, 17, 17, 17, 17, 17, 14],
        'P' => [30, 17, 17, 30, 16, 16, 16],
        'Q' => [14, 17, 17, 17, 21, 18, 13],
        'R' => [30, 17, 17, 30, 20, 18, 17],
        'S' => [15, 16, 16, 14, 1, 1, 30],
        'T' => [31, 4, 4, 4, 4, 4, 4],
        'U' => [17, 17, 17, 17, 17, 17, 14],
        'V' => [17, 17, 17, 17, 17, 10, 4],
        'W' => [17, 17, 17, 21, 21, 21, 10],
        'X' => [17, 17, 10, 4, 10, 17, 17],
        'Y' => [17, 17, 10, 4, 4, 4, 4],
        'Z' => [31, 1, 2, 4, 8, 16, 31],
        '0' => [14, 17, 19, 21, 25, 17, 14],
        '1' => [4, 12, 4, 4, 4, 4, 14],
        '2' => [14, 17, 1, 2, 4, 8, 31],
        '3' => [30, 1, 1, 14, 1, 1, 30],
        '4' => [2, 6, 10, 18, 31, 2, 2],
        '5' => [31, 16, 16, 30, 1, 1, 30],
        '6' => [14, 16, 16, 30, 17, 17, 14],
        '7' => [31, 1, 2, 4, 8, 8, 8],
        '8' => [14, 17, 17, 14, 17, 17, 14],
        '9' => [14, 17, 17, 15, 1, 1, 14],
        ':' => [0, 4, 4, 0, 4, 4, 0],
        '.' => [0, 0, 0, 0, 0, 6, 6],
        '/' => [1, 2, 2, 4, 8, 8, 16],
        '-' => [0, 0, 0, 31, 0, 0, 0],
        '+' => [0, 4, 4, 31, 4, 4, 0],
        '<' => [1, 2, 4, 8, 4, 2, 1],
        '>' => [16, 8, 4, 2, 4, 8, 16],
        _ => [0; 7],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn street_label_stays_inside_its_panel_for_long_and_local_names() {
        for name in [
            "Main Street",
            "Οδός Αρχιεπισκόπου Μακαρίου Γ",
            &"LONG STREET NAME ".repeat(30),
            &"X".repeat(200),
        ] {
            let mut vertices = Vec::new();
            append_street_name(&mut vertices, name);
            assert!(!vertices.is_empty());
            for vertex in vertices.chunks_exact(6) {
                assert!((-0.46..=0.46).contains(&vertex[0]));
                assert!((-0.97..=-0.76).contains(&vertex[1]));
            }
        }
        for ch in "ΑΒΓΔΕΖΗΘΙΚΛΜΝΞΟΠΡΣΤΥΦΧΨΩΆΈΉΊΌΎΏ".chars() {
            assert_ne!(glyph(ch), [0; 7], "missing glyph {ch}");
        }
    }

    #[test]
    fn location_pages_hit_only_visible_entries() {
        assert_eq!(
            location_menu_hit_test(-0.5, -0.07, 3, 4),
            Some(MenuAction::Choice(3))
        );
        assert_eq!(location_menu_hit_test(-0.5, -0.195, 3, 4), None);
        assert_eq!(
            location_menu_hit_test(-0.45, 0.14, 0, 4),
            Some(MenuAction::Page(-1))
        );
        assert_eq!(
            location_menu_hit_test(-0.07, 0.14, 0, 4),
            Some(MenuAction::Page(1))
        );
        assert_eq!(location_menu_hit_test(-0.30, 0.14, 0, 4), None);
        assert_eq!(
            location_menu_hit_test(-0.5, -0.32, 0, 4),
            Some(MenuAction::Choice(2))
        );
        assert_eq!(
            location_menu_hit_test(-0.41, 0.14, 0, 4),
            Some(MenuAction::Page(-1))
        );
        assert_eq!(
            location_menu_hit_test(-0.1, 0.14, 0, 4),
            Some(MenuAction::Page(1))
        );
    }

    #[test]
    fn car_pages_select_visible_rows_and_ignore_empty_rows() {
        for (selected, car) in crate::cars::ALL.into_iter().enumerate() {
            for slot in 0..CHOICES_PER_PAGE {
                let r = car_box(slot);
                let expected = crate::cars::ALL
                    .into_iter()
                    .nth(selected / CHOICES_PER_PAGE * CHOICES_PER_PAGE + slot)
                    .map(MenuAction::Car);
                assert_eq!(
                    selection_menu_hit_test(0.5, (r[1] + r[3]) * 0.5, 0, 4, car),
                    expected
                );
            }
            assert_eq!(
                selection_menu_hit_test(0.52, 0.14, 0, 4, car),
                Some(MenuAction::CarPage(-1))
            );
            assert_eq!(
                selection_menu_hit_test(0.87, 0.14, 0, 4, car),
                Some(MenuAction::CarPage(1))
            );
        }
    }

    #[test]
    fn main_menu_click_targets_cover_all_locations_cars_and_start() {
        for (y, id) in [(-0.07, TrackId::Scenic), (-0.195, TrackId::Engomi)] {
            assert_eq!(menu_hit_test(-0.5, y), Some(MenuAction::Location(id)));
        }
        for (index, car) in crate::cars::ALL
            .into_iter()
            .take(CHOICES_PER_PAGE)
            .enumerate()
        {
            let r = car_box(index);
            assert_eq!(
                menu_hit_test((r[0] + r[2]) * 0.5, (r[1] + r[3]) * 0.5),
                Some(MenuAction::Car(car))
            );
        }
        assert_eq!(menu_hit_test(-0.5, -0.35), None);
        assert_eq!(menu_hit_test(-0.5, -0.575), Some(MenuAction::Start));
        assert_eq!(menu_hit_test(0.5, 0.2), None);
        assert_eq!(menu_hit_test(-0.5, -0.8), None);
    }

    #[test]
    fn pause_buttons_match_their_visible_actions() {
        for (x, y, action) in [
            (-0.36, 0.13, PauseAction::Resume),
            (0.36, 0.13, PauseAction::Restart),
            (-0.36, -0.06, PauseAction::Selection),
            (0.36, -0.06, PauseAction::Audio),
            (-0.36, -0.25, PauseAction::Camera),
            (0.36, -0.25, PauseAction::Recover),
        ] {
            assert_eq!(pause_hit_test(x, y), Some(action));
        }
        assert_eq!(pause_hit_test(0.0, 0.5), None);
    }

    #[test]
    fn minimap_heading_keeps_forward_up_and_queries_rotated_corners() {
        let track = TrackId::Compact.track();
        for heading in [0.0, 0.7, 1.57, 3.14, -2.0] {
            let mut fit = MapFit::new(&track, [0.0; 3], [1280.0, 720.0]);
            fit.heading = Some(heading);
            let center = fit.point([0.0; 3]);
            let forward = fit.point([heading.sin() * 30.0, 0.0, heading.cos() * 30.0]);
            assert!((forward[0] - center[0]).abs() < 0.00001);
            assert!(forward[1] > center[1]);
            let bounds = fit.world_bounds();
            for x in [MAP_CONTENT[0][0], MAP_CONTENT[1][0]] {
                for y in [MAP_CONTENT[0][1], MAP_CONTENT[1][1]] {
                    let right = (x - center[0]) / fit.scale[0];
                    let ahead = (y - center[1]) / fit.scale[1];
                    let world = [
                        right * heading.cos() + ahead * heading.sin(),
                        -right * heading.sin() + ahead * heading.cos(),
                    ];
                    for i in 0..2 {
                        assert!(
                            world[i] >= bounds[0][i] - 0.001 && world[i] <= bounds[1][i] + 0.001
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn vicinity_map_follows_player_and_preserves_pixel_aspect() {
        for id in crate::track::ALL {
            let track = id.track();
            for player in [track.spawn().point, track.at_distance(600.0).point] {
                for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
                    let fit = MapFit::new(&track, player, viewport);
                    let center = fit.point(player);
                    assert!(
                        (center[0] - 0.78).abs() < 0.00001 && (center[1] - 0.685).abs() < 0.00001
                    );
                    let east = fit.point([player[0] + 100.0, 0.0, player[2]]);
                    let south = fit.point([player[0], 0.0, player[2] + 100.0]);
                    let width = (east[0] - center[0]) * viewport[0] * 0.5;
                    let height = (south[1] - center[1]).abs() * viewport[1] * 0.5;
                    assert!((width - height).abs() < 0.001);
                    assert_eq!(south[1] < center[1], id == TrackId::Engomi);
                    assert!(fit.point([player[0] + 1000.0, 0.0, player[2]])[0] > MAP_CONTENT[1][0]);
                }
            }
        }
    }

    #[test]
    fn vicinity_clips_crossing_roads_and_hides_distant_markers() {
        let [a, b] = clip_map_line([0.0, 0.68], [2.0, 0.68], 0.006).unwrap();
        assert!((a[0] - 0.616).abs() < 0.00001 && (b[0] - 0.944).abs() < 0.00001);
        assert!(clip_map_line([0.0, 0.0], [2.0, 0.0], 0.006).is_none());
        assert!(clip_map_line([0.8, 0.0], [0.8, 2.0], 0.006).is_some());
        let mut v = Vec::new();
        map_marker(&mut v, [2.0, 2.0], 0.013, [1.0, 0.0, 0.0]);
        assert!(v.is_empty());
        map_marker(&mut v, [0.95, 0.88], 0.013, [1.0, 0.0, 0.0]);
        assert!(!v.is_empty());
        for p in v.as_chunks::<6>().0 {
            assert!(p[0] >= MAP_CONTENT[0][0] && p[0] <= MAP_CONTENT[1][0]);
            assert!(p[1] >= MAP_CONTENT[0][1] && p[1] <= MAP_CONTENT[1][1]);
        }
    }

    #[test]
    fn full_map_zoom_matches_minimap_and_can_fit_every_road() {
        for id in crate::track::ALL {
            let track = id.track();
            for player in [
                track.spawn().point,
                [track.bounds[1][0] + 100.0, 0.0, track.bounds[1][1] + 100.0],
            ] {
                for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
                    let mini = MapFit::new(&track, player, viewport);
                    let mut view = FullMapView::new(&track, player, viewport);
                    assert_eq!(view.fit.scale, mini.scale);
                    assert_eq!(view.fit.center, mini.center);
                    let center = view.fit.point(player);
                    assert!(center[0].abs() < 0.00001);
                    assert!((center[1] - 0.015).abs() < 0.00001);
                    assert!(view.zoom(1.0));
                    assert!((view.fit.scale[0] / mini.scale[0] - 1.25).abs() < 0.00001);
                    assert_eq!(view.fit.point(player), center);
                    assert!(view.zoom(-1.0));
                    assert!((view.fit.scale[0] - mini.scale[0]).abs() < 0.00001);
                    assert!(view.zoom(100.0));
                    assert!(!view.zoom(1.0));
                    assert_eq!(view.fit.scale[0], view.max_scale);
                    assert!(view.zoom(-100.0));
                    assert!(!view.zoom(-1.0));
                    assert_eq!(view.fit.scale[0], view.min_scale);
                    for invalid in [0.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                        assert!(!view.zoom(invalid));
                    }
                    assert_eq!(view.fit.point(player), center);
                    let fit = &view.fit;
                    for p in track
                        .map_segments()
                        .flat_map(|(a, b)| [a, b])
                        .chain([player])
                    {
                        let point = fit.point(p);
                        for axis in 0..2 {
                            assert!(point[axis] > FULL_MAP_CONTENT[0][axis]);
                            assert!(point[axis] < FULL_MAP_CONTENT[1][axis]);
                        }
                    }
                    let center = fit.point(player);
                    let east = fit.point([player[0] + 100.0, 0.0, player[2]]);
                    let south = fit.point([player[0], 0.0, player[2] + 100.0]);
                    let width = (east[0] - center[0]) * viewport[0];
                    let height = (south[1] - center[1]).abs() * viewport[1];
                    assert!((width - height).abs() < 0.001);
                    assert_eq!(south[1] < center[1], id == TrackId::Engomi);
                }
            }
        }
    }

    #[test]
    fn full_map_zoom_clips_road_strokes_inside_the_panel() {
        let track = TrackId::Scenic.track();
        let player = track.spawn().point;
        for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
            for steps in [-100.0, 0.0, 100.0] {
                let mut view = FullMapView::new(&track, player, viewport);
                view.zoom(steps);
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
                    .chunks_exact(6)
                    .filter(|p| p[3..] == [0.40, 0.55, 0.60])
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

    #[test]
    fn dial_sweep_places_zero_at_lower_left_and_half_scale_at_top() {
        let center = [100.0, 100.0];
        let zero = dial_point(center, 50.0, 0.0);
        let half = dial_point(center, 50.0, 0.5);
        let full = dial_point(center, 50.0, 1.0);
        assert!(zero[0] < center[0] && zero[1] > center[1]);
        assert!((half[0] - 100.0).abs() < 0.001 && (half[1] - 50.0).abs() < 0.001);
        assert!(full[0] > center[0] && full[1] > center[1]);
        assert_eq!(dial_value(9000.0, 8000.0), 1.0);
        assert_eq!(dial_value(-20.0, 240.0), 0.0);
        assert_eq!(dial_value(f32::NAN, 240.0), 0.0);
        for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
            let mut v = Vec::new();
            let canvas = InstrumentCanvas {
                vertices: &mut v,
                viewport,
                scale: 1.0,
                origin: [0.0, 0.0],
            };
            let a = canvas.point([100.0, 100.0]);
            let b = canvas.point([150.0, 100.0]);
            let c = canvas.point([100.0, 150.0]);
            let width = (b[0] - a[0]) * viewport[0] * 0.5;
            let height = (c[1] - a[1]).abs() * viewport[1] * 0.5;
            assert!(
                (width - height).abs() < 0.001,
                "dials must stay circular at {viewport:?}"
            );
        }
    }

    #[test]
    fn gauges_are_bounded_and_clamped_to_the_hud_panel() {
        let mut v = Vec::new();
        append_instruments(
            &mut v,
            Gauges {
                rpm: 9000.0,
                redline_rpm: 6500.0,
                speed_kmh: 120.0,
                gear: 3,
            },
            [1280.0, 720.0],
        );
        assert!(!v.is_empty());
        for vertex in v.as_chunks::<6>().0 {
            assert!(vertex[0] >= 0.48 && vertex[0] <= 0.99, "x {}", vertex[0]);
            assert!(vertex[1] >= -0.995 && vertex[1] <= -0.50, "y {}", vertex[1]);
            assert!(vertex[2] == 0.0);
            assert!(vertex[3..6].iter().all(|c| *c >= 0.0 && *c <= 1.0));
        }
        let amber_vertices = v
            .as_chunks::<6>()
            .0
            .iter()
            .filter(|vertex| vertex[3..6] == [1.0, 0.55, 0.18])
            .count();
        assert!(amber_vertices > 0, "high RPM should use warning color");
    }
}
