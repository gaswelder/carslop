use crate::{
    cars::CarModel,
    track::{Track, TrackId},
};
use std::ffi::CString;

pub const LOCATIONS: [TrackId; 3] = [TrackId::Scenic, TrackId::Compact, TrackId::Engomi];
pub fn available_locations(city_enabled: bool) -> &'static [TrackId] {
    if city_enabled {
        &LOCATIONS
    } else {
        &LOCATIONS[..2]
    }
}
const LOCATION_BOXES: [[f32; 4]; 3] = [
    [-0.92, 0.42, 0.0, 0.62],
    [-0.92, 0.19, 0.0, 0.39],
    [-0.92, -0.04, 0.0, 0.16],
];
const CAR_BOXES: [[f32; 4]; 3] = [
    [-0.92, -0.27, -0.64, -0.16],
    [-0.60, -0.27, -0.32, -0.16],
    [-0.28, -0.27, 0.0, -0.16],
];
const ACTIVITY_BOX: [f32; 4] = [-0.92, -0.46, 0.0, -0.35];
const START_BOX: [f32; 4] = [-0.92, -0.68, 0.0, -0.54];
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MenuAction {
    Location(TrackId),
    Car(CarModel),
    Activity,
    Start,
}
pub struct MainMenu {
    pub location: TrackId,
    pub car: CarModel,
    pub race: bool,
    pub focus: usize,
    pub city_enabled: bool,
}
pub fn menu_hit_test(x: f32, y: f32) -> Option<MenuAction> {
    let inside = |r: [f32; 4]| x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3];
    for (i, r) in LOCATION_BOXES.into_iter().enumerate() {
        if inside(r) {
            return Some(MenuAction::Location(LOCATIONS[i]));
        }
    }
    for (i, r) in CAR_BOXES.into_iter().enumerate() {
        if inside(r) {
            return Some(MenuAction::Car(crate::cars::ALL[i]));
        }
    }
    if inside(ACTIVITY_BOX) {
        return Some(MenuAction::Activity);
    }
    if inside(START_BOX) {
        return Some(MenuAction::Start);
    }
    None
}

pub struct Map<'a> {
    pub track: &'a Track,
    pub player: [f32; 3],
    pub rival: Option<[f32; 3]>,
    pub gate: Option<[f32; 3]>,
}

#[derive(Clone, Copy)]
pub struct Gauges {
    pub rpm: f32,
    pub redline_rpm: f32,
    pub speed_kmh: f32,
    pub gear: u8,
}

const PAUSE_BOXES: [[f32; 4]; 6] = [
    [-0.86, -0.67, -0.32, -0.53],
    [-0.27, -0.67, 0.27, -0.53],
    [0.32, -0.67, 0.86, -0.53],
    [-0.86, -0.45, -0.32, -0.33],
    [-0.27, -0.45, 0.27, -0.33],
    [0.32, -0.45, 0.86, -0.33],
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
    pub lines: &'a [String],
    pub city: bool,
    pub view: &'a str,
    pub audio: &'a str,
    pub load: f32,
}

const MAP_CONTENT: [[f32; 2]; 2] = [[0.61, 0.49], [0.95, 0.88]];
const MAP_WIDTH_METRES: f32 = 400.0;

struct MapFit {
    center: [f32; 2],
    scale: [f32; 2],
    north_up: bool,
}
impl MapFit {
    fn new(track: &Track, player: [f32; 3], viewport: [f32; 2]) -> Self {
        let x_scale = (MAP_CONTENT[1][0] - MAP_CONTENT[0][0]) / MAP_WIDTH_METRES;
        Self {
            center: [player[0], player[2]],
            scale: [x_scale, x_scale * viewport[0] / viewport[1]],
            north_up: track.city().is_some(),
        }
    }
    fn world_bounds(&self) -> [[f32; 2]; 2] {
        let x = MAP_WIDTH_METRES * 0.5;
        let z = (MAP_CONTENT[1][1] - MAP_CONTENT[0][1]) * 0.5 / self.scale[1];
        [
            [self.center[0] - x, self.center[1] - z],
            [self.center[0] + x, self.center[1] + z],
        ]
    }
    fn point(&self, p: [f32; 3]) -> [f32; 2] {
        [
            0.78 + (p[0] - self.center[0]) * self.scale[0],
            0.685
                + (p[2] - self.center[1]) * self.scale[1] * if self.north_up { -1.0 } else { 1.0 },
        ]
    }
}
/// Liang–Barsky clipping keeps crossing roads even when both endpoints are
/// outside the vicinity. Padding includes the stroke width at the boundary.
fn clip_map_line(a: [f32; 2], b: [f32; 2], padding: f32) -> Option<[[f32; 2]; 2]> {
    let delta = [b[0] - a[0], b[1] - a[1]];
    let mut start: f32 = 0.0;
    let mut end: f32 = 1.0;
    for axis in 0..2 {
        let min = MAP_CONTENT[0][axis] + padding;
        let max = MAP_CONTENT[1][axis] - padding;
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
            Self { vao, vbo }
        }
    }

    pub fn draw(
        &self,
        program: u32,
        map: Map<'_>,
        gauges: Gauges,
        pause: Option<PauseMenu<'_>>,
        overlay: &[String],
    ) {
        let mut v = Vec::new();
        let white = [0.9, 0.95, 1.0];
        let cyan = [0.15, 0.85, 0.95];
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
        let fit = MapFit::new(
            map.track,
            map.player,
            [viewport[2].max(1) as f32, viewport[3].max(1) as f32],
        );
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
        for (position, color, size) in [
            (map.gate, [1.0, 0.8, 0.2], 0.009),
            (map.rival, [0.15, 0.65, 1.0], 0.013),
            (Some(map.player), [1.0, 0.25, 0.12], 0.013),
        ] {
            let Some(p) = position else {
                continue;
            };
            map_marker(&mut v, fit.point(p), size, color);
        }
        if map.track.city().is_some() {
            text(&mut v, "N", [0.94, 0.86], 0.0035, white);
        }
        if !overlay.is_empty() {
            quad(&mut v, [-0.65, -0.62], [0.65, 0.57], [0.025, 0.05, 0.075]);
            quad(&mut v, [-0.65, 0.54], [0.65, 0.57], cyan);
            for (i, entry) in overlay.iter().enumerate() {
                let size = if i == 0 { 0.007 } else { 0.005 };
                let width = entry.len() as f32 * 6.0 * size;
                text(
                    &mut v,
                    entry,
                    [-width * 0.5, 0.43 - i as f32 * 0.13],
                    size,
                    if i == 0 { cyan } else { white },
                );
            }
        }
        if let Some(pause) = pause {
            append_pause(&mut v, pause);
        }
        self.upload(program, &v);
    }

    pub fn draw_menu(&self, program: u32, menu: MainMenu) {
        let mut v = Vec::new();
        let cyan = [0.15, 0.85, 0.95];
        let white = [0.9, 0.95, 1.0];
        let muted = [0.55, 0.66, 0.70];
        quad(&mut v, [-0.96, -0.75], [0.04, 0.94], [0.025, 0.05, 0.075]);
        text(&mut v, "CHOOSE YOUR DRIVE", [-0.92, 0.86], 0.006, white);
        text(&mut v, "LOCATION", [-0.92, 0.71], 0.0038, cyan);
        let details = [
            "SCENIC COASTAL ROAD / 2.3 KM",
            "SHORT CIRCUIT / FREE DRIVE + RACE",
            "NORTH + SOUTH / 289 KM2 / OFFLINE",
        ];
        for (i, &id) in available_locations(menu.city_enabled).iter().enumerate() {
            let r = LOCATION_BOXES[i];
            let selected = menu.location == id;
            menu_button(&mut v, r, selected, menu.focus == 0 && selected);
            text(
                &mut v,
                id.track().name,
                [r[0] + 0.035, r[3] - 0.035],
                0.0042,
                if selected { cyan } else { white },
            );
            text(
                &mut v,
                details[i],
                [r[0] + 0.035, r[3] - 0.115],
                0.003,
                muted,
            );
        }
        text(&mut v, "CAR", [-0.92, -0.10], 0.0038, cyan);
        for (i, car) in crate::cars::ALL.into_iter().enumerate() {
            let r = CAR_BOXES[i];
            let selected = menu.car == car;
            menu_button(&mut v, r, selected, menu.focus == 1 && selected);
            text(
                &mut v,
                car.spec().name,
                [r[0] + 0.025, r[3] - 0.035],
                0.0037,
                if selected { cyan } else { white },
            );
        }
        menu_button(&mut v, ACTIVITY_BOX, true, menu.focus == 2);
        text(
            &mut v,
            if menu.race {
                "ACTIVITY: RACE"
            } else {
                "ACTIVITY: FREE DRIVE"
            },
            [-0.89, -0.385],
            0.0038,
            white,
        );
        if menu.location != TrackId::Compact {
            text(
                &mut v,
                "RACING AVAILABLE ON COASTAL LOOP",
                [-0.90, -0.495],
                0.0027,
                muted,
            );
        }
        menu_button(&mut v, START_BOX, true, menu.focus == 3);
        text(
            &mut v,
            if menu.race {
                "START RACE"
            } else {
                "START DRIVING"
            },
            [-0.87, -0.585],
            0.005,
            cyan,
        );
        quad(&mut v, [0.13, 0.58], [0.98, 0.94], [0.025, 0.05, 0.075]);
        quad(&mut v, [0.13, -0.835], [0.98, -0.56], [0.025, 0.05, 0.075]);
        text(&mut v, "CAR PREVIEW", [0.17, 0.86], 0.0035, muted);
        text(&mut v, menu.car.spec().name, [0.17, 0.77], 0.0065, white);
        text(
            &mut v,
            menu.car.spec().description,
            [0.17, 0.66],
            0.0032,
            white,
        );
        text(
            &mut v,
            menu.location.track().name,
            [0.17, -0.62],
            0.0038,
            white,
        );
        if menu.location == TrackId::Engomi {
            text(
                &mut v,
                "MAP DATA: OPENSTREETMAP CONTRIBUTORS",
                [0.17, -0.69],
                0.0027,
                white,
            );
            text(
                &mut v,
                "OPENSTREETMAP.ORG/COPYRIGHT / ODBL",
                [0.17, -0.725],
                0.0027,
                white,
            );
            text(
                &mut v,
                "TERRAIN: MAPZEN / USGS / EU COPERNICUS",
                [0.17, -0.76],
                0.0024,
                white,
            );
            if crate::aerial::source_available() {
                text(
                    &mut v,
                    crate::aerial::attribution_label(),
                    [0.17, -0.793],
                    0.0024,
                    white,
                );
            }
        }
        quad(&mut v, [-0.98, -0.96], [0.98, -0.85], [0.025, 0.05, 0.075]);
        text(
            &mut v,
            "CLICK OPTIONS / UP-DOWN SECTION / LEFT-RIGHT CHOOSE / ENTER START / Q QUIT",
            [-0.94, -0.885],
            0.0036,
            white,
        );
        self.upload(program, &v);
    }

    fn upload(&self, program: u32, v: &[f32]) {
        let identity = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        unsafe {
            gl::Disable(gl::DEPTH_TEST);
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.vbo);
            gl::BufferData(
                gl::ARRAY_BUFFER,
                (v.len() * 4) as isize,
                v.as_ptr() as *const _,
                gl::STREAM_DRAW,
            );
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
            gl::DrawArrays(gl::TRIANGLES, 0, (v.len() / 6) as i32);
            gl::Enable(gl::DEPTH_TEST);
        }
    }
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
        &if gauges.gear == 0 {
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
    let muted = [0.55, 0.66, 0.70];
    quad(v, [-0.94, -0.83], [0.94, 0.91], [0.025, 0.05, 0.075]);
    quad(v, [-0.94, 0.88], [0.94, 0.91], cyan);
    text(v, "PAUSED", [-0.86, 0.79], 0.008, cyan);
    text(v, "SESSION", [-0.86, 0.65], 0.004, muted);
    text(v, "CONTROLS", [0.12, 0.65], 0.004, muted);
    for (i, entry) in pause.lines.iter().enumerate() {
        let size = 0.0042_f32.min(0.76 / (entry.chars().count().max(1) as f32 * 6.0));
        text(v, entry, [-0.86, 0.55 - i as f32 * 0.09], size, white);
    }
    for (i, entry) in [
        "W / UP: ACCELERATE",
        "S / DOWN: BRAKE",
        "A / D: STEER",
        "SPACE: HANDBRAKE",
        "X: DRIVE / REVERSE",
        "C: RECOVER",
        "TAB: CAMERA",
        "V: AUDIO",
        "Q: QUIT",
    ]
    .iter()
    .enumerate()
    {
        text(v, entry, [0.12, 0.55 - i as f32 * 0.085], 0.0037, white);
    }
    text(
        v,
        &format!("ENGINE LOAD {:.0} PCT", pause.load.clamp(0.0, 1.0) * 100.0),
        [0.12, -0.23],
        0.0032,
        muted,
    );
    if pause.city {
        text(
            v,
            "MAP DATA: OPENSTREETMAP CONTRIBUTORS",
            [-0.86, -0.08],
            0.0029,
            muted,
        );
        text(
            v,
            "OPENSTREETMAP.ORG/COPYRIGHT / ODBL",
            [-0.86, -0.14],
            0.0029,
            muted,
        );
        text(
            v,
            "TERRAIN: MAPZEN / USGS / EU COPERNICUS",
            [-0.86, -0.20],
            0.0027,
            muted,
        );
        if crate::aerial::source_available() {
            text(
                v,
                crate::aerial::attribution_label(),
                [-0.86, -0.26],
                0.0027,
                muted,
            );
        }
    }
    let labels = [
        "RESUME".to_string(),
        "RESTART".into(),
        "LOCATION / CAR".into(),
        format!("AUDIO: {}", pause.audio),
        format!("CAMERA: {}", pause.view),
        "RECOVER".into(),
    ];
    for (r, label) in PAUSE_BOXES.into_iter().zip(labels) {
        menu_button(v, r, true, false);
        text(v, &label, [r[0] + 0.025, r[3] - 0.04], 0.0032, white);
    }
    text(
        v,
        "ESC RESUME / R RESTART / M LOCATION AND CAR / Q QUIT",
        [-0.86, -0.755],
        0.0032,
        muted,
    );
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
    match ch {
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
        _ => [0; 7],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn main_menu_click_targets_cover_all_locations_cars_and_start() {
        for (y, id) in [
            (0.52, TrackId::Scenic),
            (0.29, TrackId::Compact),
            (0.06, TrackId::Engomi),
        ] {
            assert_eq!(menu_hit_test(-0.5, y), Some(MenuAction::Location(id)));
        }
        for (x, car) in [
            (-0.78, CarModel::Roadster),
            (-0.46, CarModel::Muscle),
            (-0.14, CarModel::Club),
        ] {
            assert_eq!(menu_hit_test(x, -0.215), Some(MenuAction::Car(car)));
        }
        assert_eq!(menu_hit_test(-0.5, -0.40), Some(MenuAction::Activity));
        assert_eq!(menu_hit_test(-0.5, -0.61), Some(MenuAction::Start));
        assert_eq!(menu_hit_test(0.5, 0.2), None);
        assert_eq!(menu_hit_test(-0.5, -0.8), None);
    }

    #[test]
    fn pause_buttons_match_their_visible_actions() {
        for (x, y, action) in [
            (-0.59, -0.60, PauseAction::Resume),
            (0.0, -0.60, PauseAction::Restart),
            (0.59, -0.60, PauseAction::Selection),
            (-0.59, -0.39, PauseAction::Audio),
            (0.0, -0.39, PauseAction::Camera),
            (0.59, -0.39, PauseAction::Recover),
        ] {
            assert_eq!(pause_hit_test(x, y), Some(action));
        }
        assert_eq!(pause_hit_test(0.0, 0.5), None);
    }

    #[test]
    fn vicinity_map_follows_player_and_preserves_pixel_aspect() {
        for id in crate::track::ALL {
            let track = id.track();
            for player in [track.spawn().point, track.at_distance(600.0).point] {
                for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
                    let fit = MapFit::new(track, player, viewport);
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
        for p in v.chunks_exact(6) {
            assert!(p[0] >= MAP_CONTENT[0][0] && p[0] <= MAP_CONTENT[1][0]);
            assert!(p[1] >= MAP_CONTENT[0][1] && p[1] <= MAP_CONTENT[1][1]);
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
        for vertex in v.chunks_exact(6) {
            assert!(vertex[0] >= 0.48 && vertex[0] <= 0.99, "x {}", vertex[0]);
            assert!(vertex[1] >= -0.995 && vertex[1] <= -0.50, "y {}", vertex[1]);
            assert!(vertex[2] == 0.0);
            assert!(vertex[3..6].iter().all(|c| *c >= 0.0 && *c <= 1.0));
        }
        let amber_vertices = v
            .chunks_exact(6)
            .filter(|vertex| vertex[3..6] == [1.0, 0.55, 0.18])
            .count();
        assert!(amber_vertices > 0, "high RPM should use warning color");
    }
}
