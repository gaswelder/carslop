mod aerial;
mod ambience;
mod audio;
mod camera;
mod cars;
mod chunks;
mod city;
mod city_assets;
mod city_imagery;
mod cli;
mod elevation;
mod engine_sample;
mod feedback;
mod gl;
mod hud;
mod imagery_trees;
mod landscape;
mod loading;
mod location;
mod package;
mod particles;
mod prefetch;
mod scene;
mod scene_archive;
mod shadows;
mod source_tools;
mod street_surface;
mod track;
mod traffic;
mod tree_obstacles;
mod tree_preview;
mod vehicle;

use cars::CarModel;
use log::{info, warn};
use sdl3::{
    event::{Event, WindowEvent},
    keyboard::Keycode,
    video::Window,
};
use std::{
    collections::HashSet,
    time::{Duration, Instant},
};
use track::{Track, TrackId};
use vehicle::{Controls, Vehicle};

const TICK: Duration = Duration::from_nanos(16_666_667);
const WINDOW_WIDTH: i32 = 1280;
const WINDOW_HEIGHT: i32 = 720;

#[derive(Clone, Copy, PartialEq, Eq)]
enum GameState {
    Menu,
    Driving,
    Paused,
    FullMap,
}

struct Session {
    vehicle: Vehicle,
    state: GameState,
    recoveries: u32,
    skids: feedback::SkidMarks,
    particles: particles::TireParticles,
    impact: f32,
    signal_time: f32,
}

impl Session {
    fn new(model: CarModel) -> Self {
        Self::on_track(model, TrackId::Scenic)
    }

    fn on_track(model: CarModel, id: TrackId) -> Self {
        let track = id.track();
        let start = track.spawn();
        Self {
            vehicle: {
                let mut vehicle =
                    Vehicle::on_track(start.lane_position(0.0), start.heading(), model, track);
                vehicle.manual = true;
                vehicle
            },
            state: GameState::Driving,
            recoveries: 0,
            skids: feedback::SkidMarks::default(),
            particles: particles::TireParticles::default(),
            impact: 0.0,
            signal_time: 0.0,
        }
    }

    fn track(&self) -> &Track {
        &self.vehicle.track
    }

    fn restart(&mut self) {
        *self = Self::on_track(self.vehicle.model, self.track().id);
    }

    fn select_car(&mut self, model: CarModel) {
        if self.state != GameState::Menu {
            return;
        }
        *self = Self::on_track(model, self.track().id);
        self.state = GameState::Menu;
    }

    fn select_location(&mut self, id: TrackId) {
        if self.state != GameState::Menu {
            return;
        }
        *self = Self::on_track(self.vehicle.model, id);
        self.state = GameState::Menu;
    }
    #[cfg(test)]
    fn cycle_location(&mut self, direction: i32, city_enabled: bool) {
        let locations = hud::available_locations(city_enabled);
        let current = locations
            .iter()
            .position(|id| *id == self.track().id)
            .unwrap();
        let next = (current as i32 + direction).rem_euclid(locations.len() as i32) as usize;
        self.select_location(locations[next]);
    }
    fn cycle_car(&mut self, direction: i32) {
        let next = (self.vehicle.model.index() as i32 + direction)
            .rem_euclid(cars::ALL.len() as i32) as usize;
        self.select_car(cars::ALL[next]);
    }
    fn update_feedback(&mut self) {
        self.skids.advance(TICK);
        self.skids.record(0, &self.vehicle);
        self.particles.advance(TICK);
        self.particles.record(0, &self.vehicle, TICK);
    }

    fn tick(&mut self, controls: Controls) {
        if self.state != GameState::Driving {
            return;
        }
        self.signal_time =
            (self.signal_time + TICK.as_secs_f32()).rem_euclid(traffic::CYCLE_SECONDS);
        self.impact = self.impact.max(self.vehicle.update(TICK, controls));
        self.update_feedback();
    }

    fn recover(&mut self) -> bool {
        if self.state != GameState::Driving {
            return false;
        }
        let road = self.track().recovery(self.vehicle.position);
        let backwards = (self.vehicle.heading - road.heading()).cos() < 0.0;
        let heading = road.heading() + if backwards { std::f32::consts::PI } else { 0.0 };
        self.vehicle.reset(road.lane_position(0.0), heading);
        self.skids.break_trail(0);
        self.particles.reset_car(0);
        self.impact = 0.0;
        self.recoveries += 1;
        true
    }

    fn pause_lines(&self, _view: &str, _audio: &str) -> Vec<String> {
        let car = &self.vehicle;
        let speed = format!(
            "{:03} KM/H GEAR {}",
            (car.speed.abs() * 3.6) as u32,
            car.gear_label()
        );
        vec![
            format!("{} FREE DRIVE", car.model.spec().name),
            format!(
                "{} {:.2} KM{}",
                self.track().name,
                self.track().length() / 1000.0,
                if self.track().city().is_some() {
                    " ROADS"
                } else {
                    ""
                }
            ),
            speed,
        ]
    }
}

struct App {
    // Audio and GL resources are dropped before their owning SDL context.
    audio: Option<audio::EngineAudio>,
    meshes: [Option<gl::Mesh>; 3],
    skid_renderer: gl::SkidRenderer,
    particle_renderer: gl::ParticleRenderer,
    hud: hud::Hud,
    program: u32,
    scene_program: u32,
    _gl_context: sdl3::video::GLContext,
    window: Window,
    sdl_context: sdl3::Sdl,
    events: sdl3::EventPump,
    session: Session,
    chase: camera::ChaseCamera,
    preview_angle: f32,
    menu_focus: usize,
    locations: Vec<city_assets::MenuLocation>,
    selected_location: usize,
    #[cfg(test)]
    capture_path: Option<std::path::PathBuf>,
    keys: HashSet<Keycode>,
}

impl Drop for App {
    fn drop(&mut self) {
        // Delete programs while the owning GL context is still alive.
        unsafe {
            ::gl::DeleteProgram(self.program);
            ::gl::DeleteProgram(self.scene_program);
        }
    }
}

impl App {
    #[cfg(test)]
    fn new() -> Self {
        Self::with_location(false)
    }

    fn with_location(city: bool) -> Self {
        let sdl_context = sdl3::init().expect("SDL init failed");
        let video = sdl_context.video().expect("failed to get video subsystem");
        let gl_attr = video.gl_attr();
        gl_attr.set_context_profile(sdl3::video::GLProfile::Core);
        gl_attr.set_context_version(3, 3);
        gl_attr.set_depth_size(24);
        let events = sdl_context.event_pump().expect("failed to get event pump");
        let mut window = video
            .window("Coastal Loop", WINDOW_WIDTH as u32, WINDOW_HEIGHT as u32)
            .opengl()
            .build()
            .expect("failed to create window");
        let gl_context = gl::create_gl_context(&window).expect("failed to create GL context");
        let _ = video.gl_set_swap_interval(sdl3::video::SwapInterval::VSync);
        gl::load_gl();
        gl::enable_depth_test();
        info!("{}", gl::debug_context_info());
        let mut locations = city_assets::menu_locations();
        let selected_city = city.then(|| {
            city_assets::directory()
                .canonicalize()
                .unwrap_or_else(|_| city_assets::directory().to_path_buf())
        });
        let selected_location = selected_city.map_or(0, |path| {
            if let Some(index) = locations.iter().position(|entry| {
                entry
                    .path
                    .as_deref()
                    .and_then(|p| p.canonicalize().ok())
                    .as_deref()
                    == Some(path.as_path())
            }) {
                index
            } else {
                locations.push(city_assets::MenuLocation {
                    name: city_assets::name().into(),
                    track: TrackId::Engomi,
                    path: Some(path),
                });
                locations.len() - 1
            }
        });
        let mut session = Session::new(CarModel::Roadster);
        session.state = GameState::Menu;
        let _ = window.set_title(session.track().name);

        let program =
            gl::create_program(gl::VS_SRC, gl::FS_SRC).expect("failed to create GL program");
        let scene_program =
            gl::create_program(gl::SCENE_VS, gl::SCENE_FS).expect("failed to create scene program");
        let skid_renderer = gl::SkidRenderer::new();
        let particle_renderer =
            gl::ParticleRenderer::new().expect("failed to create particle renderer");
        let hud = hud::Hud::new();
        let audio = match audio::EngineAudio::new(&sdl_context) {
            Ok(audio) => Some(audio),
            Err(error) => {
                warn!("Audio unavailable; continuing silently: {error}");
                None
            }
        };
        let chase = camera::ChaseCamera::new(&session.vehicle);
        let mut app = Self {
            sdl_context,
            events,
            window,
            _gl_context: gl_context,
            program,
            scene_program,
            meshes: track::ALL.map(|_| None),
            skid_renderer,
            particle_renderer,
            hud,
            audio,
            session,
            chase,
            preview_angle: 0.0,
            menu_focus: 0,
            locations,
            selected_location,
            #[cfg(test)]
            capture_path: None,
            keys: HashSet::new(),
        };
        app.ensure_mesh();
        app
    }

    fn start_selected(&mut self) {
        let entry = self.locations[self.selected_location].clone();
        let previous = city_assets::directory().to_path_buf();
        let mut changed = false;
        if let Some(path) = entry.path {
            changed = path.canonicalize().ok().as_deref() != Some(city_assets::directory());
            if let Err(error) = city_assets::select(path) {
                warn!("Cannot select location: {error}");
                return;
            }
        }
        if !self.prepare_location(entry.track) {
            let _ = city_assets::select(previous);
            return;
        }
        if changed || self.meshes[entry.track.index()].is_none() {
            match gl::Mesh::new(&entry.track.track()) {
                Ok(mesh) => self.meshes[entry.track.index()] = Some(mesh),
                Err(error) => {
                    warn!("Cannot prepare location geometry: {error}");
                    let _ = city_assets::select(previous);
                    return;
                }
            }
        }
        self.session.select_location(entry.track);
        self.ensure_mesh();
        self.restart();
    }

    fn prepare_location(&mut self, id: TrackId) -> bool {
        let loaded = loading::prepare(id, &mut self.window, &mut self.events, &self.sdl_context);
        self.keys.clear();
        loaded
    }

    fn ensure_mesh(&mut self) {
        let track = self.session.track();
        let slot = &mut self.meshes[track.id.index()];
        if slot.is_none() {
            if track.id == TrackId::Engomi {
                loading::draw(
                    &self.window,
                    track.name,
                    "PREPARING VEHICLES AND SCENERY",
                    0.0,
                );
                gl::swap_window(&self.window);
            }
            *slot = Some(gl::Mesh::new(track).expect("failed to create route mesh"));
        }
    }

    fn mesh(&self) -> &gl::Mesh {
        self.meshes[self.session.track().id.index()]
            .as_ref()
            .expect("selected route mesh must be cached")
    }

    fn restart(&mut self) {
        self.session.restart();
        if let Some(audio) = &mut self.audio {
            audio.reset_feedback();
        }
        self.keys.clear();
        self.snap_camera();
    }

    fn snap_camera(&mut self) {
        self.chase.snap(&self.session.vehicle);
    }

    fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let mut last_frame = Instant::now();
        let mut accumulator = Duration::ZERO;
        'running: loop {
            while let Some(event) = self.events.poll_event() {
                match event {
                    Event::Quit { .. }
                    | Event::Window {
                        win_event: WindowEvent::CloseRequested,
                        ..
                    } => break 'running,
                    Event::KeyDown {
                        keycode: Some(Keycode::Q),
                        repeat: false,
                        ..
                    } => break 'running,
                    Event::KeyDown {
                        keycode: Some(key),
                        repeat: false,
                        ..
                    } => self.key_down(key),
                    Event::MouseButtonDown {
                        mouse_btn: sdl3::mouse::MouseButton::Left,
                        x,
                        y,
                        ..
                    } => self.menu_click(x, y),
                    Event::MouseWheel { y, direction, .. } => self.mouse_wheel(y, direction),
                    Event::KeyUp {
                        keycode: Some(key), ..
                    } => {
                        self.keys.remove(&key);
                    }
                    Event::Window {
                        win_event: WindowEvent::FocusLost,
                        ..
                    } => {
                        self.keys.clear();
                    }
                    _ => {}
                }
            }
            let now = Instant::now();
            let frame_time = (now - last_frame).min(Duration::from_millis(250));
            last_frame = now;
            accumulator += frame_time;
            while accumulator >= TICK {
                let held = |keys: &[Keycode]| keys.iter().any(|k| self.keys.contains(k));
                let controls = Controls {
                    throttle: held(&[Keycode::Up]) as u8 as f32,
                    brake: held(&[Keycode::Down]) as u8 as f32,
                    steering: held(&[Keycode::Right]) as u8 as f32
                        - held(&[Keycode::Left]) as u8 as f32,
                    handbrake: held(&[Keycode::Space]),
                };
                self.session.tick(controls);
                accumulator -= TICK;
            }
            let impact = std::mem::take(&mut self.session.impact);
            if let Some(audio) = &mut self.audio {
                let car = &self.session.vehicle;
                if let Err(error) = audio.update(
                    car,
                    self.chase.pose(),
                    self.keys.contains(&Keycode::Up),
                    matches!(self.session.state, GameState::Driving),
                    impact,
                ) {
                    warn!("Audio disabled: {error}");
                    self.audio = None;
                }
            }
            self.render(frame_time);
        }
        Ok(())
    }

    fn menu_action(&mut self, action: hud::MenuAction) {
        if self.session.state != GameState::Menu {
            return;
        }
        match action {
            hud::MenuAction::Location(id) => {
                if let Some(index) = self.locations.iter().position(|entry| entry.track == id) {
                    self.selected_location = index;
                }
                self.menu_focus = 0;
            }
            hud::MenuAction::Choice(index) => {
                if index < self.locations.len() {
                    self.selected_location = index;
                }
                self.menu_focus = 0;
            }
            hud::MenuAction::Page(direction) => {
                let pages = self.locations.len().div_ceil(3);
                let page = (self.selected_location as i32 / 3 + direction).rem_euclid(pages as i32)
                    as usize;
                self.selected_location = page * 3;
                self.menu_focus = 0;
            }
            hud::MenuAction::CarPage(direction) => {
                let current = cars::ALL
                    .into_iter()
                    .position(|car| car == self.session.vehicle.model)
                    .unwrap_or(0);
                let pages = cars::ALL.len().div_ceil(3);
                let page = (current as i32 / 3 + direction).rem_euclid(pages as i32) as usize;
                self.session.select_car(cars::ALL[page * 3]);
                self.menu_focus = 1;
            }
            hud::MenuAction::Car(car) => {
                self.menu_focus = 1;
                self.session.select_car(car);
            }
            hud::MenuAction::Start => {
                self.start_selected();
                return;
            }
        }
        self.ensure_mesh();
        self.snap_camera();
        self.keys.clear();
    }
    fn menu_click(&mut self, x: f32, y: f32) {
        let (width, height) = self.window.size();
        if width == 0 || height == 0 {
            return;
        }
        let point = [x / width as f32 * 2.0 - 1.0, 1.0 - y / height as f32 * 2.0];
        if self.session.state == GameState::Paused {
            if let Some(action) = hud::pause_hit_test(point[0], point[1]) {
                self.key_down(match action {
                    hud::PauseAction::Resume => Keycode::Escape,
                    hud::PauseAction::Restart => Keycode::R,
                    hud::PauseAction::Selection => Keycode::L,
                    hud::PauseAction::Audio => Keycode::V,
                    hud::PauseAction::Camera => Keycode::C,
                    hud::PauseAction::Recover => Keycode::H,
                });
            }
        } else if self.session.state == GameState::Menu {
            if let Some(action) = hud::selection_menu_hit_test(
                point[0],
                point[1],
                self.selected_location,
                self.locations.len(),
                self.session.vehicle.model,
            ) {
                self.menu_action(action);
            }
        }
    }
    fn key_down(&mut self, key: Keycode) {
        if self.session.state == GameState::FullMap {
            match key {
                Keycode::Escape => self.session.state = GameState::Driving,
                Keycode::C => self.copy_map_location(),
                Keycode::Plus | Keycode::Equals | Keycode::KpPlus => self.zoom_map(1.0),
                Keycode::Minus | Keycode::KpMinus => self.zoom_map(-1.0),
                _ => {}
            }
            self.keys.clear();
            return;
        }
        self.keys.insert(key);
        if self.session.state == GameState::Menu {
            match key {
                Keycode::Left => self.menu_focus = 0,
                Keycode::Right => self.menu_focus = 1,
                Keycode::Tab => self.menu_focus = (self.menu_focus + 1) % 3,
                Keycode::Up | Keycode::Down | Keycode::Space => {
                    let direction = if key == Keycode::Up { -1 } else { 1 };
                    match self.menu_focus {
                        0 => {
                            self.selected_location = (self.selected_location as i32 + direction)
                                .rem_euclid(self.locations.len() as i32)
                                as usize;
                        }
                        1 => self.session.cycle_car(direction),
                        _ if key == Keycode::Space => {
                            self.start_selected();
                            return;
                        }
                        _ => {}
                    }
                    self.ensure_mesh();
                    self.snap_camera();
                }
                _ => {
                    self.handle_key(key);
                    return;
                }
            }
            self.keys.clear();
            return;
        }
        self.handle_key(key);
    }

    fn mouse_wheel(&mut self, y: f32, direction: sdl3::mouse::MouseWheelDirection) {
        self.zoom_map(if direction == sdl3::mouse::MouseWheelDirection::Flipped {
            -y
        } else {
            y
        });
    }

    fn copy_map_location(&mut self) {
        let url = self
            .session
            .track()
            .city()
            .and_then(|city| city.google_maps_url(self.session.vehicle.position));
        let status = if let Some(url) = url {
            match self
                .sdl_context
                .video()
                .and_then(|video| video.clipboard().set_clipboard_text(&url))
            {
                Ok(()) => "COPIED GOOGLE MAPS URL",
                Err(error) => {
                    log::warn!("Cannot copy map location: {error}");
                    "COPY FAILED - PRESS C TO RETRY"
                }
            }
        } else {
            "NO GOOGLE MAPS LOCATION"
        };
        self.hud.map_copy_status(
            hud::Map {
                track: self.session.track(),
                player: self.session.vehicle.position,
            },
            self.session.vehicle.heading,
            [WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32],
        );
    }

    fn zoom_map(&mut self, steps: f32) {
        if self.session.state != GameState::FullMap {
            return;
        }
        self.hud.zoom_full_map(
            hud::Map {
                track: self.session.track(),
                player: self.session.vehicle.position,
            },
            self.session.vehicle.heading,
            [WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32],
            steps,
        );
    }

    fn handle_key(&mut self, key: Keycode) {
        match key {
            Keycode::Escape => {
                self.session.state = match self.session.state {
                    GameState::Driving => GameState::Paused,
                    GameState::Paused => GameState::Driving,
                    other => other,
                };
                self.keys.clear();
            }
            Keycode::Return if self.session.state == GameState::Menu => self.start_selected(),
            Keycode::M if matches!(self.session.state, GameState::Driving | GameState::Paused) => {
                self.hud.prepare_full_map(
                    hud::Map {
                        track: self.session.track(),
                        player: self.session.vehicle.position,
                    },
                    self.session.vehicle.heading,
                    [WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32],
                );
                self.session.state = GameState::FullMap;
                self.keys.clear();
            }
            Keycode::L => {
                self.menu_focus = 0;
                self.session.state = GameState::Menu;
                self.keys.clear();
            }
            Keycode::R if self.session.state == GameState::Menu => self.start_selected(),
            Keycode::R => self.restart(),
            Keycode::H => {
                let paused = self.session.state == GameState::Paused;
                if paused {
                    self.session.state = GameState::Driving;
                }
                let recovered = self.session.recover();
                if paused {
                    self.session.state = GameState::Paused;
                }
                if recovered {
                    if let Some(audio) = &mut self.audio {
                        audio.reset_feedback();
                    }
                    self.snap_camera();
                }
            }
            Keycode::F1 | Keycode::F2 | Keycode::F3 if self.session.state == GameState::Menu => {
                let model = match key {
                    Keycode::F1 => CarModel::Roadster,
                    Keycode::F2 => CarModel::Muscle,
                    _ => CarModel::Club,
                };
                self.session.select_car(model);
                self.snap_camera();
            }
            Keycode::F6 if self.session.state == GameState::Menu => {
                self.selected_location = if self.selected_location == 0 { 1 } else { 0 };
            }
            Keycode::F7 if self.session.state == GameState::Menu => {
                if let Some(index) = self
                    .locations
                    .iter()
                    .position(|entry| entry.track == TrackId::Engomi)
                {
                    self.selected_location = index;
                }
            }
            Keycode::A | Keycode::Z if self.session.state == GameState::Driving => {
                self.session.vehicle.shift(key == Keycode::A);
            }
            Keycode::V => {
                if let Some(audio) = &mut self.audio {
                    audio.muted = !audio.muted;
                }
            }
            Keycode::C | Keycode::Tab
                if matches!(self.session.state, GameState::Driving | GameState::Paused) =>
            {
                self.chase.toggle_view();
                if self.session.state == GameState::Paused {
                    self.snap_camera();
                }
            }
            _ => {}
        }
    }

    fn render(&mut self, frame_time: Duration) {
        gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
        gl::clear_screen();
        if self.session.state == GameState::FullMap {
            self.hud.draw_full_map(self.program);
        } else {
            self.render_scene(frame_time);
        }
        #[cfg(test)]
        if let Some(path) = &self.capture_path {
            tests::capture_frame(path);
        }
        gl::swap_window(&self.window);
        let _ = self.window.set_title(&format!(
            "{} | {} | FREE DRIVE",
            self.session.track().name,
            self.session.vehicle.model.spec().name,
        ));
    }

    fn render_scene(&mut self, frame_time: Duration) {
        gl::set_signal_time(self.scene_program, self.session.signal_time);
        let car = &self.session.vehicle;
        if matches!(self.session.state, GameState::Driving) {
            self.chase.update(car, frame_time);
        }
        let aspect = WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32;
        let in_menu = self.session.state == GameState::Menu;
        let camera = if in_menu {
            self.preview_angle = (self.preview_angle + frame_time.as_secs_f32() * 0.22)
                .rem_euclid(std::f32::consts::TAU);
            gl::garage_camera(
                aspect,
                car.position,
                car.heading + self.preview_angle,
                self.session.track().city(),
            )
        } else {
            gl::chase_camera(aspect, self.chase.pose())
        };
        let vehicles = [(&self.session.vehicle, [1.0, 0.18, 0.08])];
        let contacts = vehicles.map(|(car, _)| shadows::ContactShadow::new(car));
        self.mesh()
            .draw_ground(self.scene_program, &camera, &contacts);
        if !in_menu {
            self.skid_renderer
                .draw(self.scene_program, &camera, &self.session.skids);
        }
        for &(vehicle, tint) in &vehicles {
            let model = gl::vehicle_body_matrix(vehicle);
            self.mesh()
                .draw(self.scene_program, &camera, &model, tint, vehicle);
        }
        // Transparent puffs follow all opaque world/car draws and precede the HUD.
        if !in_menu {
            self.particle_renderer
                .draw(&camera, &self.session.particles);
        }
        let s = &self.session;
        let audio_status = match &self.audio {
            Some(a) if a.muted => "MUTED",
            Some(_) => "ON",
            None => "UNAVAILABLE",
        };
        let lines = if s.state == GameState::Paused {
            s.pause_lines(self.chase.view().label(), audio_status)
        } else {
            Vec::new()
        };
        if in_menu {
            self.hud.draw_menu(
                self.program,
                hud::MainMenu {
                    locations: &self.locations,
                    selected: self.selected_location,
                    car: s.vehicle.model,
                    focus: self.menu_focus,
                },
            );
        } else {
            self.hud.draw(
                self.program,
                hud::Map {
                    track: s.track(),
                    player: car.position,
                },
                hud::Gauges {
                    rpm: car.engine_rpm,
                    redline_rpm: vehicle::REDLINE_RPM,
                    speed_kmh: car.speed.abs() * 3.6,
                    gear: car.gear,
                },
                (s.state == GameState::Paused).then_some(hud::PauseMenu {
                    lines: &lines,
                    city: s.track().city().is_some(),
                    view: self.chase.view().label(),
                    audio: audio_status,
                }),
            );
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let Some(city) = cli::run()? else {
        return Ok(());
    };
    info!("Coastal Loop: starting");
    App::with_location(city).run()
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(super) fn capture_frame(path: &std::path::Path) {
        use std::io::Write;
        let mut pixels = vec![0_u8; (WINDOW_WIDTH * WINDOW_HEIGHT * 3) as usize];
        unsafe {
            ::gl::ReadBuffer(::gl::BACK);
            ::gl::PixelStorei(::gl::PACK_ALIGNMENT, 1);
            ::gl::ReadPixels(
                0,
                0,
                WINDOW_WIDTH,
                WINDOW_HEIGHT,
                ::gl::RGB,
                ::gl::UNSIGNED_BYTE,
                pixels.as_mut_ptr().cast(),
            );
        }
        let mut file = std::fs::File::create(path).unwrap();
        write!(file, "P6\n{} {}\n255\n", WINDOW_WIDTH, WINDOW_HEIGHT).unwrap();
        for row in pixels
            .as_chunks::<{ (WINDOW_WIDTH * 3) as usize }>()
            .0
            .iter()
            .rev()
        {
            file.write_all(row).unwrap();
        }
    }

    #[test]
    #[ignore = "requires SDL3/OpenGL and NFS_CITY_DATA pointing to an imported, baked location"]
    fn imported_location_renderer_smoke() {
        assert!(
            std::env::var_os("NFS_CITY_DATA").is_some(),
            "select an imported location"
        );
        let mut app = App::with_location(true);
        assert_ne!(app.session.track().id, TrackId::Engomi);
        app.start_selected();
        assert_eq!(app.session.track().id, TrackId::Engomi);
        app.session.state = GameState::Menu;
        if let Some(path) = std::env::var_os("NFS_IMPORTED_PREVIEW") {
            app.capture_path = Some(path.into());
        }
        app.render(Duration::ZERO);
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        app.key_down(Keycode::Return);
        app.render(Duration::ZERO);
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    }

    #[test]
    #[ignore = "requires SDL3/OpenGL and the bundled Nicosia map"]
    fn readme_screenshot() {
        let mut app = App::new();
        app.key_down(Keycode::F7);
        app.start_selected();
        let city = app.session.track().city().unwrap();
        let url = city.google_maps_url([0.0; 3]).unwrap();
        let coords: Vec<f64> = url
            .split("query=")
            .nth(1)
            .unwrap()
            .split(',')
            .map(|v| v.parse().unwrap())
            .collect();
        let (lat, lon) = (coords[0], coords[1]);
        let x = ((33.3169556_f64 - lon).to_radians() * 6371000.0 * lat.to_radians().cos()) as f32;
        let z = (-(35.1670537_f64 - lat).to_radians() * 6371000.0) as f32;
        let road = city.project([x, 0.0, z]);
        app.session
            .vehicle
            .reset([x, road.point[1] + 0.48, z], road.heading());
        app.snap_camera();
        app.capture_path = Some("/tmp/carslop-readme.ppm".into());
        app.render(Duration::ZERO);
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    }

    #[test]
    #[ignore = "requires SDL3/OpenGL; exports Engomi views to NFS_CITY_PREVIEW_DIR"]
    fn engomi_renderer_smoke() {
        let directory = std::path::PathBuf::from(
            std::env::var("NFS_CITY_PREVIEW_DIR").unwrap_or_else(|_| "/tmp/engomi-preview".into()),
        );
        std::fs::create_dir_all(&directory).unwrap();
        let mut app = App::new();
        let loading = std::time::Instant::now();
        app.key_down(Keycode::F7);
        app.start_selected();
        app.session.state = GameState::Menu;
        eprintln!(
            "City selection: {:.2}s including map/collision loading and archive validation",
            loading.elapsed().as_secs_f32()
        );
        assert_eq!(app.session.track().id, TrackId::Engomi);
        let mesh = app.mesh() as *const gl::Mesh;
        let (index_bytes, decode_peak, _) = app.mesh().paging_stats();
        assert!(
            index_bytes < 2 * 1024 * 1024 && decode_peak == 0,
            "city selection should read only the archive index"
        );
        app.capture_path = Some(directory.join("garage.ppm"));
        let first_frame = std::time::Instant::now();
        app.render(Duration::ZERO);
        eprintln!(
            "First city render + screenshot: {:.3}s",
            first_frame.elapsed().as_secs_f32()
        );
        let first_read = app.mesh().paging_stats().0;
        assert!(first_read >= index_bytes && first_read < 64 * 1024 * 1024);
        app.capture_path = None;
        app.render(Duration::ZERO);
        assert_eq!(
            first_read,
            app.mesh().paging_stats().0,
            "stationary frames should not reread geometry"
        );
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        app.key_down(Keycode::Return);
        for (label, view) in [
            ("wide", camera::FollowView::Wide),
            ("close", camera::FollowView::Close),
            ("bonnet", camera::FollowView::Bonnet),
        ] {
            while app.chase.view() != view {
                app.key_down(Keycode::Tab);
            }
            app.snap_camera();
            app.capture_path = Some(directory.join(format!("{label}.ppm")));
            app.render(Duration::ZERO);
            assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        }
        for (label, speed, rpm, gear) in [
            ("instruments", 120.0, 4800.0, 3),
            ("instruments-redline", 210.0, 6500.0, 5),
            ("instruments-reverse", 18.0, 2500.0, 0),
        ] {
            app.session.vehicle.speed = speed / 3.6;
            app.session.vehicle.engine_rpm = rpm;
            app.session.vehicle.gear = gear;
            app.capture_path = Some(directory.join(format!("{label}.ppm")));
            app.render(Duration::ZERO);
            assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        }
        app.session.vehicle.speed = 0.0;
        app.session.vehicle.engine_rpm = vehicle::IDLE_RPM;
        app.session.vehicle.gear = 1;
        app.capture_path = Some(directory.join("pause.ppm"));
        app.key_down(Keycode::Escape);
        assert!(app.session.state == GameState::Paused);
        let position = app.session.vehicle.position;
        for _ in 0..30 {
            app.session.tick(Controls {
                throttle: 1.0,
                ..Controls::default()
            });
        }
        assert_eq!(app.session.vehicle.position, position);
        app.render(Duration::ZERO);
        let view = app.chase.view();
        app.menu_click(640.0, 500.0);
        assert_ne!(app.chase.view(), view);
        assert!(app.session.state == GameState::Paused);
        let recovered = app.session.recoveries;
        app.menu_click(1018.0, 500.0);
        assert_eq!(app.session.recoveries, recovered + 1);
        assert!(app.session.state == GameState::Paused);
        if let Some(audio) = &app.audio {
            let muted = audio.muted;
            app.menu_click(262.0, 500.0);
            assert_ne!(app.audio.as_ref().unwrap().muted, muted);
        }
        app.menu_click(262.0, 576.0);
        assert!(app.session.state == GameState::Driving);
        // Repeatable reference-block street and facade reviews.
        let city = app.session.track().city().unwrap();
        let p = city.recovery([0.0, 0.0, 0.0]).point;
        app.session.vehicle.reset(p, city.project(p).heading());
        app.snap_camera();
        app.capture_path = Some(directory.join("reference-block.ppm"));
        app.render(Duration::ZERO);
        let city = app.session.track().city().unwrap();
        let building = city.buildings.iter().find(|b| b.id == 1020326527).unwrap();
        let [a, c] = [building.polygon[0], building.polygon[1]];
        let length = (c[0] - a[0]).hypot(c[1] - a[1]);
        let mid = [
            (a[0] + c[0]) * 0.5,
            building.base_y + 3.3,
            (a[1] + c[1]) * 0.5,
        ];
        let camera = gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            camera::Pose {
                eye: [
                    mid[0] + (c[1] - a[1]) / length * 17.0,
                    mid[1] + 4.5,
                    mid[2] - (c[0] - a[0]) / length * 17.0,
                ],
                target: mid,
            },
        );
        gl::clear_screen();
        app.mesh().draw_ground(app.scene_program, &camera, &[]);
        capture_frame(&directory.join("reference-facade.ppm"));
        // An oblique junction view makes curb returns and road hierarchy
        // reviewable without the chase camera or HUD covering the intersection.
        let city = app.session.track().city().unwrap();
        let node = city
            .surfaces
            .as_ref()
            .unwrap()
            .preview_junction
            .or_else(|| {
                // Binary collision caches omit the text-only review marker.
                let mut degree = vec![0_u32; city.nodes.len()];
                for street in &city.streets {
                    for end in street.ends {
                        degree[end] += 1;
                    }
                }
                city.nodes
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| degree[*i] >= 3)
                    .min_by(|(_, a), (_, b)| a[0].hypot(a[2]).total_cmp(&b[0].hypot(b[2])))
                    .map(|(i, _)| i)
            })
            .expect("missing junction review location");
        let p = city.nodes[node];
        let camera = gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            camera::Pose {
                eye: [p[0] + 25.0, p[1] + 32.0, p[2] + 26.0],
                target: p,
            },
        );
        gl::clear_screen();
        app.mesh().draw_ground(app.scene_program, &camera, &[]);
        capture_frame(&directory.join("junction.ppm"));
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        // Choose the mapped planting review from OSM trees only. Inferred trees
        // are rendered too, but must not turn this small preview query into an
        // all-pairs scan of the city-wide inferred canopy.
        let city = app.session.track().city().unwrap();
        let mapped: Vec<_> = city
            .landscape
            .trees
            .iter()
            .filter(|tree| tree.id < (1_u64 << 63))
            .collect();
        let tree = mapped
            .iter()
            .copied()
            .max_by_key(|tree| {
                mapped
                    .iter()
                    .filter(|other| {
                        (tree.point[0] - other.point[0]).hypot(tree.point[1] - other.point[1]) < 45.
                    })
                    .count()
            })
            .expect("missing mapped trees");
        let p = [tree.point[0], city.height(tree.point), tree.point[1]];
        let camera = gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            camera::Pose {
                eye: [p[0] + 48.0, p[1] + 60.0, p[2] + 52.0],
                target: [p[0], p[1] + 2.0, p[2]],
            },
        );
        gl::clear_screen();
        app.mesh().draw_ground(app.scene_program, &camera, &[]);
        capture_frame(&directory.join("landscape.ppm"));
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        // Place the car on real mapped paving to review support height and
        // confirm the renderer/camera tolerate driving beyond the asphalt.
        let city = app.session.track().city().unwrap();
        let safe = |q: [f32; 2]| {
            let mut p = [q[0], 0.0, q[1]];
            city.constrain(&mut p, &mut [0.0; 2]);
            ((p[0] - q[0]).hypot(p[2] - q[1]) < 0.001).then_some(p)
        };
        let pavement = city
            .surfaces
            .as_ref()
            .unwrap()
            .triangles
            .iter()
            .filter(|t| t.style == 5)
            .map(|t| {
                [
                    t.points.iter().map(|p| p[0]).sum::<f32>() / 3.0,
                    t.points.iter().map(|p| p[1]).sum::<f32>() / 3.0,
                ]
            })
            .filter(|q| city.road_clearance([q[0], 0.0, q[1]]) > 0.7)
            .find_map(safe)
            .expect("missing safe pavement preview");
        let parking = city
            .landscape
            .ground
            .iter()
            .filter(|t| t.style == 3)
            .map(|t| {
                [
                    t.points.iter().map(|p| p[0]).sum::<f32>() / 3.0,
                    t.points.iter().map(|p| p[1]).sum::<f32>() / 3.0,
                ]
            })
            .find_map(safe)
            .expect("missing safe parking preview");
        for (label, p) in [("pavement", pavement), ("parking", parking)] {
            let heading = app.session.track().project(p).heading();
            app.session.vehicle.reset(p, heading);
            app.snap_camera();
            app.capture_path = Some(directory.join(format!("{label}.ppm")));
            app.render(Duration::ZERO);
            assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        }
        for (label, lat, lon) in [
            ("nicosia-centre", 35.1746503_f64, 33.3638783_f64),
            ("strovolos", 35.1328993, 33.3454972),
            ("gonyeli", 35.2164170, 33.3058171),
        ] {
            let city = app.session.track().city().unwrap();
            let x =
                ((lon - 33.320).to_radians() * 6_371_000.0 * 35.162_f64.to_radians().cos()) as f32;
            let z = (-(lat - 35.162).to_radians() * 6_371_000.0) as f32;
            let p = city.recovery([x, 0.0, z]).point;
            app.session.vehicle.reset(p, city.project(p).heading());
            app.snap_camera();
            app.capture_path = Some(directory.join(format!("{label}.ppm")));
            app.render(Duration::ZERO);
            assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        }
        let city = app.session.track().city().unwrap();
        let old_north = -0.006 * std::f32::consts::PI / 180.0 * 6_371_000.0;
        let extension = city.recovery([300.0, 0.0, old_north - 500.0]).point;
        let heading = city.project(extension).heading();
        app.session.vehicle.reset(extension, heading);
        app.snap_camera();
        app.capture_path = Some(directory.join("extension.ppm"));
        app.render(Duration::ZERO);
        let overview = gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            camera::Pose {
                eye: [
                    extension[0] + 65.0,
                    extension[1] + 55.0,
                    extension[2] + 60.0,
                ],
                target: extension,
            },
        );
        gl::clear_screen();
        app.mesh().draw_ground(app.scene_program, &overview, &[]);
        capture_frame(&directory.join("extension-overview.ppm"));
        let (total, resident, visible, bytes) = app.mesh().chunk_stats();
        eprintln!(
            "City chunks: {total} total, {resident} resident, {visible} visible, {:.1} MiB GPU",
            bytes as f32 / 1048576.0
        );
        assert!(total > 20 && visible > 0 && visible <= resident && resident < total);
        let (read, peak, coarse) = app.mesh().paging_stats();
        eprintln!("City paging: {:.1} MiB read across review views; {:.1} MiB peak temporary decode; {coarse} coarse building chunks",read as f32/1048576.0,peak as f32/1048576.0);
        assert!(coarse > 0 && peak < 32 * 1024 * 1024);
        // Travel to opposite corners and back. Visible data must reload, while
        // buffers beyond the retention radius are actually released.
        let bounds = app.session.track().city().unwrap().bounds;
        for q in [
            [bounds[0][0] + 100.0, bounds[0][1] + 100.0],
            [bounds[1][0] - 100.0, bounds[1][1] - 100.0],
            [extension[0], extension[2]],
        ] {
            let city = app.session.track().city().unwrap();
            let y = city.height(q);
            let camera = gl::chase_camera(
                WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
                camera::Pose {
                    eye: [q[0] + 10.0, y + 15.0, q[1] + 12.0],
                    target: [q[0], y, q[1]],
                },
            );
            gl::clear_screen();
            app.mesh().draw_ground(app.scene_program, &camera, &[]);
            app.mesh().check_chunk_residency(&camera);
            let (_, resident, visible, _) = app.mesh().chunk_stats();
            assert!(visible > 0 && resident < total);
            assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        }
        // Visit a ground tile already prepared by the worker: this must be a
        // cache hit, not another synchronous disk read for that layer.
        if let Some(q) = app.mesh().prefetched_ground_location() {
            let before = app.mesh().prefetch_stats().0;
            let y = app.session.track().city().unwrap().height(q);
            let camera = gl::chase_camera(
                WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
                camera::Pose {
                    eye: [q[0] + 10.0, y + 20.0, q[1] + 12.0],
                    target: [q[0], y, q[1]],
                },
            );
            app.mesh().draw_ground(app.scene_program, &camera, &[]);
            assert!(
                app.mesh().prefetch_stats().0 > before,
                "visiting a prefetched tile should consume the ready cache"
            );
        } else {
            panic!("worker did not prefetch an approaching ground chunk");
        }
        let (hits, misses, cache, background_read, decode) = app.mesh().prefetch_stats();
        eprintln!("Prefetch: {hits} hits, {misses} fallbacks, {:.1} MiB peak cache, {:.1} MiB background read, {:.1} MiB worker decode",cache as f32/1048576.0,background_read as f32/1048576.0,decode as f32/1048576.0);
        assert!(cache <= 32 * 1024 * 1024);
        app.capture_path = None;
        app.key_down(Keycode::F6);
        assert_eq!(app.session.track().id, TrackId::Engomi);
        app.key_down(Keycode::F7);
        assert_eq!(app.session.track().id, TrackId::Engomi);
        app.key_down(Keycode::L);
        app.key_down(Keycode::F6);
        app.key_down(Keycode::F7);
        assert!(std::ptr::eq(app.mesh(), mesh));
        app.key_down(Keycode::F5);
        assert_eq!(app.session.track().id, TrackId::Engomi);
    }

    #[test]
    #[ignore = "requires SDL3/OpenGL; previews mapped signs and verifies animated signal pixels"]
    fn road_furniture_renderer_smoke() {
        let directory = std::path::PathBuf::from(
            std::env::var("NFS_FURNITURE_PREVIEW_DIR")
                .unwrap_or_else(|_| "/tmp/furniture-preview".into()),
        );
        std::fs::create_dir_all(&directory).unwrap();
        let mut app = App::new();
        app.key_down(Keycode::F7);
        app.key_down(Keycode::Return);
        for (label, kind) in [
            ("signals", traffic::FurnitureKind::Signal),
            ("stop", traffic::FurnitureKind::Stop),
            ("speed", traffic::FurnitureKind::Speed),
        ] {
            let feature = app
                .session
                .track()
                .city()
                .unwrap()
                .furniture
                .iter()
                .find(|f| f.kind == kind)
                .unwrap();
            let p = feature.position;
            let travel = feature.travel;
            let right = [-travel[1], travel[0]];
            let height = p[1]
                + if kind == traffic::FurnitureKind::Signal {
                    2.82
                } else {
                    2.12
                };
            let camera = gl::chase_camera(
                WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
                camera::Pose {
                    eye: [
                        p[0] - travel[0] * 7.0 + right[0] * 1.5,
                        height,
                        p[2] - travel[1] * 7.0 + right[1] * 1.5,
                    ],
                    target: [p[0], height, p[2]],
                },
            );
            let start =
                (traffic::CYCLE_SECONDS - feature.phase as f32).rem_euclid(traffic::CYCLE_SECONDS);
            let times: &[f32] = if kind == traffic::FurnitureKind::Signal {
                &[0.0, 21.0, 24.0, 49.0]
            } else {
                &[0.0]
            };
            for &time in times {
                gl::clear_screen();
                gl::set_signal_time(
                    app.scene_program,
                    (start + time).rem_euclid(traffic::CYCLE_SECONDS),
                );
                app.mesh().draw_ground(app.scene_program, &camera, &[]);
                if kind == traffic::FurnitureKind::Signal {
                    for (lamp, y) in [3.12, 2.82, 2.52].into_iter().enumerate() {
                        let center = [p[0] - travel[0] * 0.167, p[1] + y, p[2] - travel[1] * 0.167];
                        let pixel = gl::test_pixel(&camera, center, WINDOW_WIDTH, WINDOW_HEIGHT);
                        let mut color = [0_u8; 3];
                        unsafe {
                            ::gl::ReadBuffer(::gl::BACK);
                            ::gl::PixelStorei(::gl::PACK_ALIGNMENT, 1);
                            ::gl::ReadPixels(
                                pixel[0],
                                pixel[1],
                                1,
                                1,
                                ::gl::RGB,
                                ::gl::UNSIGNED_BYTE,
                                color.as_mut_ptr().cast(),
                            );
                        }
                        let channel = if lamp == 2 { 1 } else { 0 };
                        let active = traffic::lamp_lit(time, 0, lamp);
                        assert!(
                            if active {
                                color[channel] > 150
                            } else {
                                color[channel] < 60
                            },
                            "lens {lamp} at {time} returned {color:?}"
                        );
                    }
                }
                if time == 0.0 {
                    capture_frame(&directory.join(format!("{label}.ppm")));
                }
                assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
            }
        }
        app.session.tick(Controls::default());
        let clock = app.session.signal_time;
        app.key_down(Keycode::Escape);
        for _ in 0..60 {
            app.session.tick(Controls::default());
        }
        assert_eq!(app.session.signal_time, clock);
    }

    #[test]
    #[ignore = "requires SDL3/OpenGL; captures the location loading screen"]
    fn location_loading_renderer_smoke() {
        let mut app = App::with_location(false);
        let path = std::env::var("NFS_LOADING_PREVIEW")
            .unwrap_or_else(|_| "/tmp/location-loading.ppm".into());
        loading::draw(
            &app.window,
            "NICOSIA",
            "READING MAP AND BUILDING COLLISIONS",
            12.0,
        );
        capture_frame(std::path::Path::new(&path));
        gl::swap_window(&app.window);
        unsafe {
            assert_eq!(::gl::GetError(), ::gl::NO_ERROR);
        }
        // Completed loads must not create another loader or consume input.
        assert!(loading::prepare(
            TrackId::Scenic,
            &mut app.window,
            &mut app.events,
            &app.sdl_context
        ));
    }

    #[test]
    #[ignore = "requires SDL3/OpenGL; compares rear lamp pixels with the pedal up/down"]
    fn stop_lights_renderer() {
        let app = App::new();
        for model in cars::ALL {
            let mut car = vehicle::Vehicle::new(app.session.vehicle.position, 0.0, model);
            let p = car.position;
            let camera = gl::chase_camera(
                WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
                camera::Pose {
                    eye: [p[0], p[1] + 1.2, p[2] - 5.0],
                    target: [p[0], p[1] + 0.25, p[2]],
                },
            );
            let matrix = gl::model_matrix(p, 0.0, 0.0);
            let mut frames = Vec::new();
            for pressed in [false, true, false] {
                car.brake_lights = pressed;
                gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
                gl::clear_screen();
                app.mesh()
                    .draw(app.scene_program, &camera, &matrix, [0.1, 0.3, 0.8], &car);
                let mut pixels = vec![0u8; (WINDOW_WIDTH * WINDOW_HEIGHT * 3) as usize];
                unsafe {
                    ::gl::ReadBuffer(::gl::BACK);
                    ::gl::PixelStorei(::gl::PACK_ALIGNMENT, 1);
                    ::gl::ReadPixels(
                        0,
                        0,
                        WINDOW_WIDTH,
                        WINDOW_HEIGHT,
                        ::gl::RGB,
                        ::gl::UNSIGNED_BYTE,
                        pixels.as_mut_ptr().cast(),
                    );
                }
                assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
                frames.push(pixels);
            }
            let brightened = frames[0]
                .chunks_exact(3)
                .zip(frames[1].chunks_exact(3))
                .filter(|(off, on)| on[0] > off[0].saturating_add(20) && on[0] > 220 && on[1] < 80)
                .count();
            assert!(
                brightened > 100,
                "stop lamps did not brighten: {model:?}, {brightened} pixels"
            );
            assert_eq!(
                frames[0], frames[2],
                "lamps must return to their unlit finish"
            );
        }
    }

    /// Export repeatable front/rear/side views for reviewing silhouette and shading.
    #[test]
    #[ignore = "requires SDL3/OpenGL; exports PPM previews to NFS_CAR_PREVIEW_DIR"]
    fn kestrel_art_preview() {
        use std::io::Write;
        let directory = std::env::var("NFS_CAR_PREVIEW_DIR")
            .expect("set NFS_CAR_PREVIEW_DIR to an output directory");
        std::fs::create_dir_all(&directory).unwrap();
        let app = App::new();
        for car_model in cars::ALL {
            let preview_car = vehicle::Vehicle::new(app.session.vehicle.position, 0.0, car_model);
            let car = &preview_car;
            let p = car.position;
            for (name, offset) in [
                ("front", [4.2, 2.1, 5.0]),
                ("rear", [-4.2, 1.8, -5.0]),
                ("side", [6.0, 1.0, 0.0]),
                ("detail-front", [2.8, 1.45, 3.65]),
                ("detail-rear", [-2.9, 1.55, -3.8]),
                ("detail-side", [4.6, 0.65, 0.0]),
                ("road", [-3.2, 1.5, -4.6]),
            ] {
                gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
                gl::clear_screen();
                let camera = gl::chase_camera(
                    WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
                    camera::Pose {
                        eye: std::array::from_fn(|i| p[i] + offset[i]),
                        target: [p[0], p[1] + 0.25, p[2]],
                    },
                );
                let model = gl::model_matrix(p, 0.0, 0.0);
                let tint = if name == "road" {
                    app.mesh().draw_ground(
                        app.scene_program,
                        &camera,
                        &[shadows::ContactShadow::new(car)],
                    );
                    [0.10, 0.30, 0.24]
                } else {
                    [0.60, 0.12, 0.065]
                };
                app.mesh()
                    .draw(app.scene_program, &camera, &model, tint, car);
                let mut pixels = vec![0u8; (WINDOW_WIDTH * WINDOW_HEIGHT * 3) as usize];
                unsafe {
                    ::gl::ReadBuffer(::gl::BACK);
                    ::gl::PixelStorei(::gl::PACK_ALIGNMENT, 1);
                    ::gl::ReadPixels(
                        0,
                        0,
                        WINDOW_WIDTH,
                        WINDOW_HEIGHT,
                        ::gl::RGB,
                        ::gl::UNSIGNED_BYTE,
                        pixels.as_mut_ptr().cast(),
                    );
                }
                assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
                let mut file = std::fs::File::create(format!(
                    "{directory}/car-{}-{name}.ppm",
                    car_model.index()
                ))
                .unwrap();
                write!(file, "P6\n{} {}\n255\n", WINDOW_WIDTH, WINDOW_HEIGHT).unwrap();
                for row in pixels
                    .as_chunks::<{ (WINDOW_WIDTH * 3) as usize }>()
                    .0
                    .iter()
                    .rev()
                {
                    file.write_all(row).unwrap();
                }
            }
        }
    }

    fn contact_count(program: u32) -> i32 {
        let name = std::ffi::CString::new("u_contact_count").unwrap();
        let mut count = -1;
        unsafe {
            ::gl::GetUniformiv(
                program,
                ::gl::GetUniformLocation(program, name.as_ptr()),
                &mut count,
            );
        }
        count
    }

    fn assert_contact_pixel_readback(app: &App) {
        let car = &app.session.vehicle;
        let camera = gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            camera::Pose {
                eye: [
                    car.position[0] + 4.0,
                    car.position[1] + 9.0,
                    car.position[2] + 5.0,
                ],
                target: car.position,
            },
        );
        let read = || {
            let mut pixels = vec![0u8; (WINDOW_WIDTH * WINDOW_HEIGHT * 3) as usize];
            unsafe {
                ::gl::ReadPixels(
                    0,
                    0,
                    WINDOW_WIDTH,
                    WINDOW_HEIGHT,
                    ::gl::RGB,
                    ::gl::UNSIGNED_BYTE,
                    pixels.as_mut_ptr() as *mut _,
                );
            }
            pixels
        };
        gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
        gl::clear_screen();
        app.mesh().draw_ground(app.scene_program, &camera, &[]);
        app.skid_renderer
            .draw(app.scene_program, &camera, &app.session.skids);
        assert_eq!(contact_count(app.scene_program), 0);
        let unshadowed = read();
        gl::clear_screen();
        app.mesh().draw_ground(
            app.scene_program,
            &camera,
            &[shadows::ContactShadow::new(car)],
        );
        app.skid_renderer
            .draw(app.scene_program, &camera, &app.session.skids);
        let shadowed = read();
        let mut changed = 0;
        for (before, after) in unshadowed
            .as_chunks::<3>()
            .0
            .iter()
            .zip(shadowed.as_chunks::<3>().0)
        {
            assert!(
                before
                    .iter()
                    .zip(after)
                    .all(|(a, b)| *b <= a.saturating_add(1)),
                "contact shading must never brighten the scene"
            );
            if before
                .iter()
                .zip(after)
                .any(|(a, b)| a.saturating_sub(*b) > 2)
            {
                changed += 1;
            }
        }
        assert!(
            changed > 100,
            "contact shadow changed only {changed} pixels for {:?}",
            car.model
        );
        assert!(
            changed < (WINDOW_WIDTH * WINDOW_HEIGHT / 8) as usize,
            "contact shading must remain local"
        );
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    }

    fn assert_particle_pixel_readback(app: &App) {
        let car = &app.session.vehicle;
        let camera = gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            camera::Pose {
                eye: [
                    car.position[0] + 4.0,
                    car.position[1] + 6.0,
                    car.position[2] + 5.0,
                ],
                target: car.position,
            },
        );
        let read_color = || {
            let mut pixels = vec![0u8; (WINDOW_WIDTH * WINDOW_HEIGHT * 3) as usize];
            unsafe {
                ::gl::ReadPixels(
                    0,
                    0,
                    WINDOW_WIDTH,
                    WINDOW_HEIGHT,
                    ::gl::RGB,
                    ::gl::UNSIGNED_BYTE,
                    pixels.as_mut_ptr() as *mut _,
                );
            }
            pixels
        };
        let read_depth = || {
            let mut pixels = vec![0.0f32; (WINDOW_WIDTH * WINDOW_HEIGHT) as usize];
            unsafe {
                ::gl::ReadPixels(
                    0,
                    0,
                    WINDOW_WIDTH,
                    WINDOW_HEIGHT,
                    ::gl::DEPTH_COMPONENT,
                    ::gl::FLOAT,
                    pixels.as_mut_ptr() as *mut _,
                );
            }
            pixels
        };
        let state = || unsafe {
            let mut blend = [0; 6];
            for (i, parameter) in [
                ::gl::BLEND_SRC_RGB,
                ::gl::BLEND_DST_RGB,
                ::gl::BLEND_SRC_ALPHA,
                ::gl::BLEND_DST_ALPHA,
                ::gl::BLEND_EQUATION_RGB,
                ::gl::BLEND_EQUATION_ALPHA,
            ]
            .into_iter()
            .enumerate()
            {
                ::gl::GetIntegerv(parameter, &mut blend[i]);
            }
            let mut depth_write = 0;
            ::gl::GetBooleanv(::gl::DEPTH_WRITEMASK, &mut depth_write);
            (
                blend,
                depth_write,
                ::gl::IsEnabled(::gl::BLEND),
                ::gl::IsEnabled(::gl::DEPTH_TEST),
            )
        };
        gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
        gl::clear_screen();
        app.mesh().draw_ground(
            app.scene_program,
            &camera,
            &[shadows::ContactShadow::new(car)],
        );
        let before = read_color();
        let depth = read_depth();
        let previous = state();
        let count = app.particle_renderer.draw(&camera, &app.session.particles);
        assert!(count > 0 && count <= particles::MAX_PARTICLES * 6);
        assert_eq!(
            state(),
            previous,
            "particle pass must restore blend/depth state"
        );
        assert_eq!(
            read_depth(),
            depth,
            "transparent puffs must not write depth"
        );
        let after = read_color();
        let changed = before
            .as_chunks::<3>()
            .0
            .iter()
            .zip(after.as_chunks::<3>().0)
            .filter(|(a, b)| a.iter().zip(*b).any(|(a, b)| a.abs_diff(*b) > 2))
            .count();
        assert!(
            changed > 25,
            "particle pass did not visibly blend for {:?}",
            car.model
        );
        assert!(
            changed < (WINDOW_WIDTH * WINDOW_HEIGHT / 6) as usize,
            "puffs must remain local"
        );

        // Put an opaque depth plane in front of every puff: no color may leak through.
        gl::clear_screen();
        app.mesh().draw_ground(app.scene_program, &camera, &[]);
        let occluded = read_color();
        unsafe {
            ::gl::ClearDepth(0.0);
            ::gl::Clear(::gl::DEPTH_BUFFER_BIT);
            ::gl::ClearDepth(1.0);
        }
        app.particle_renderer.draw(&camera, &app.session.particles);
        assert_eq!(
            read_color(),
            occluded,
            "particles must respect opaque depth"
        );
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        gl::clear_screen();
    }

    #[test]
    #[ignore = "requires SDL/OpenGL"]
    fn feedback_renderer_smoke() {
        let mut app = App::with_location(false);
        // Scenic fixtures are sampled from the route, never compact world coordinates.
        for model in cars::ALL {
            app.session = Session::on_track(model, TrackId::Scenic);
            app.ensure_mesh();
            for distance in [50.0, 50.6] {
                let road = app.session.track().at_distance(distance);
                app.session
                    .vehicle
                    .reset(road.lane_position(0.0), road.heading());
                app.session.vehicle.tire_scrub = 0.9;
                app.session.skids.record(0, &app.session.vehicle);
            }
            assert!(!app.session.skids.vertices().is_empty());
            let v = &mut app.session.vehicle;
            v.velocity = [v.heading.sin() * 8.0, v.heading.cos() * 8.0];
            v.update(
                TICK,
                Controls {
                    steering: 0.8,
                    handbrake: true,
                    ..Controls::default()
                },
            );
            for _ in 0..20 {
                app.session.particles.advance(TICK);
                app.session.particles.record(0, &app.session.vehicle, TICK);
            }
            assert_particle_pixel_readback(&app);
            for fraction in [0.0, 0.25, 0.5, 0.75] {
                let road = app
                    .session
                    .track()
                    .at_distance(app.session.track().length() * fraction);
                app.session
                    .vehicle
                    .reset(road.lane_position(0.0), road.heading());
                app.snap_camera();
                if model == CarModel::Roadster {
                    assert_contact_pixel_readback(&app);
                }
                for view in [camera::FollowView::Wide, camera::FollowView::Close] {
                    while app.chase.view() != view {
                        app.key_down(Keycode::Tab);
                    }
                    for _ in 0..120 {
                        app.chase.update(&app.session.vehicle, TICK);
                    }
                    app.render(TICK);
                    assert_eq!(
                        contact_count(app.scene_program),
                        1,
                        "Scenic Free Drive has one shadow"
                    );
                    assert_eq!(
                        unsafe { ::gl::GetError() },
                        ::gl::NO_ERROR,
                        "GL errors rendering Scenic {model:?} at {fraction} in {view:?}"
                    );
                }
                app.session.state = GameState::Menu;
                app.render(TICK);
                assert_eq!(
                    contact_count(app.scene_program),
                    1,
                    "Scenic garage has one shadow"
                );
                assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
                app.session.state = GameState::Driving;
            }
        }
    }

    #[test]
    fn car_selection_and_restart_preserve_each_free_route() {
        for id in track::ALL {
            let mut s = Session::on_track(CarModel::Roadster, id);
            s.state = GameState::Menu;
            for model in cars::ALL {
                s.select_car(model);
                assert_eq!(s.vehicle.model, model);
                assert_eq!(s.track().id, id);
                assert!(s.state == GameState::Menu);
                let road = s.track().at_distance(s.track().length() * 0.5);
                s.vehicle.reset(road.lane_position(0.0), road.heading());
                s.recoveries = 2;
                s.restart();
                assert_eq!(s.track().id, id);
                assert_eq!(s.vehicle.model, model);
                assert_eq!(s.vehicle.position, s.track().spawn().lane_position(0.0));
                assert_eq!(s.recoveries, 0);
                assert!(s.state == GameState::Driving);
                s.state = GameState::Menu;
            }
        }
    }

    #[test]
    fn free_recovery_and_pause_info_use_selected_route() {
        for id in track::ALL {
            let mut s = Session::on_track(CarModel::Muscle, id);
            let road = s.track().at_distance(s.track().length() * 0.5);
            s.vehicle.reset(road.lane_position(1.0), road.heading());
            assert!(s.recover());
            let recovered = s.track().project(s.vehicle.position);
            assert!((recovered.distance - road.distance).abs() < 0.1);
            assert!(recovered.offset.abs() < 0.01);
            assert_eq!(s.track().id, id);
            let lines = s.pause_lines("WIDE", "ON");
            assert!(lines.iter().any(|line| line.contains(s.track().name)
                && line.contains(&format!("{:.2} KM", s.track().length() / 1000.0))));
            assert!(!lines.iter().any(|line| line.starts_with("TIME ")
                || line.starts_with("LAP ")
                || line.starts_with("BEST ")));
        }
    }

    #[test]
    fn engomi_drive_recovery_pause_and_return_preserve_the_map() {
        let mut s = Session::on_track(CarModel::Roadster, TrackId::Engomi);
        let start = s.vehicle.position;
        for _ in 0..40 {
            s.tick(Controls {
                throttle: 0.6,
                ..Controls::default()
            });
        }
        assert!((s.vehicle.position[0] - start[0]).hypot(s.vehicle.position[2] - start[2]) > 0.5);
        s.state = GameState::Paused;
        let frozen = s.vehicle.position;
        for _ in 0..10 {
            s.tick(Controls {
                throttle: 1.0,
                ..Controls::default()
            });
        }
        assert_eq!(s.vehicle.position, frozen);
        s.state = GameState::Driving;
        assert!(s.recover());
        assert_eq!(s.track().id, TrackId::Engomi);
        assert!(s.track().city().unwrap().road_clearance(s.vehicle.position) < 0.0);
    }

    #[test]
    fn free_recovery_is_nearby_unpenalized_and_preserves_facing_direction() {
        let mut s = Session::on_track(CarModel::Muscle, TrackId::Compact);
        let road = track::at_distance(120.0);
        s.vehicle.reset(
            road.lane_position(1.4),
            road.heading() + std::f32::consts::PI,
        );
        s.vehicle.toggle_direction();
        assert!(s.recover());
        let recovered = track::project(s.vehicle.position);
        assert!((recovered.distance - 120.0).abs() < 0.1);
        assert!(recovered.offset.abs() < 0.01);
        assert!((s.vehicle.heading - road.heading()).cos() < -0.9);
        assert_eq!(s.vehicle.direction, vehicle::Direction::Drive);
        assert_eq!(s.vehicle.model, CarModel::Muscle);
        assert_eq!(s.recoveries, 1);
    }

    #[test]
    fn opening_menu_location_and_car_choices_survive_start() {
        for location in hud::LOCATIONS {
            for car in cars::ALL {
                let mut s = Session::new(CarModel::Roadster);
                s.state = GameState::Menu;
                s.select_location(location);
                s.select_car(car);
                assert!(s.state == GameState::Menu);
                s.restart();
                assert!(s.state == GameState::Driving);
                assert_eq!(s.track().id, location);
                assert_eq!(s.vehicle.model, car);
                s.select_location(TrackId::Compact);
                s.cycle_car(1);
                assert_eq!(s.track().id, location);
                assert_eq!(s.vehicle.model, car);
            }
        }
    }
    #[test]
    fn location_cycle_excludes_unselected_cities() {
        let mut session = Session::new(CarModel::Club);
        session.state = GameState::Menu;
        for direction in [1, 1, -1, -1, 1, 1] {
            session.cycle_location(direction, false);
            assert_ne!(session.track().id, TrackId::Engomi);
        }
    }
    #[test]
    #[ignore = "requires SDL/OpenGL; exports the bonnet camera for visual review"]
    fn bonnet_camera_preview() {
        let mut app = App::with_location(false);
        app.key_down(Keycode::Return);
        while app.chase.view() != camera::FollowView::Bonnet {
            app.key_down(Keycode::Tab);
        }
        app.snap_camera();
        app.capture_path = Some(std::path::PathBuf::from("/tmp/bonnet-camera.ppm"));
        app.render(Duration::ZERO);
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    }

    #[test]
    #[ignore = "requires SDL/OpenGL; verifies full map rendering and modal controls"]
    fn full_map_renderer_smoke() {
        let mut app = App::new();
        app.key_down(Keycode::M);
        assert!(app.session.state == GameState::Menu);
        app.key_down(Keycode::Return);
        let directory = std::path::PathBuf::from(
            std::env::var("NFS_MAP_PREVIEW_DIR").unwrap_or_else(|_| "/tmp/full-map-preview".into()),
        );
        std::fs::create_dir_all(&directory).unwrap();
        for id in track::ALL {
            app.session = Session::on_track(CarModel::Roadster, id);
            app.session.vehicle.speed = 12.0;
            app.session.vehicle.velocity = [1.0, 12.0];
            let position = app.session.vehicle.position;
            let velocity = app.session.vehicle.velocity;
            let gear = app.session.vehicle.gear;
            let signal_time = app.session.signal_time;
            let view = app.chase.view();
            let mut initial_frame = None;
            for from in [GameState::Driving, GameState::Paused] {
                app.session.state = from;
                app.key_down(Keycode::Up);
                app.key_down(Keycode::M);
                assert!(app.session.state == GameState::FullMap);
                assert!(app.keys.is_empty());
                for key in [
                    Keycode::R,
                    Keycode::C,
                    Keycode::A,
                    Keycode::Z,
                    Keycode::Tab,
                    Keycode::L,
                    Keycode::Return,
                    Keycode::M,
                    Keycode::Up,
                ] {
                    app.key_down(key);
                    assert!(app.session.state == GameState::FullMap);
                }
                app.menu_click(640.0, 576.0);
                for _ in 0..60 {
                    app.session.tick(Controls {
                        throttle: 1.0,
                        ..Controls::default()
                    });
                }
                assert_eq!(app.session.vehicle.position, position);
                assert_eq!(app.session.vehicle.velocity, velocity);
                assert_eq!(app.session.vehicle.speed, 12.0);
                assert_eq!(app.session.vehicle.gear, gear);
                assert_eq!(app.session.signal_time, signal_time);
                assert_eq!(app.session.recoveries, 0);
                assert_eq!(app.chase.view(), view);
                let path = directory.join(format!("map-{}.ppm", id.index()));
                app.capture_path = Some(path.clone());
                app.render(Duration::ZERO);
                let initial = std::fs::read(&path).unwrap();
                if let Some(previous) = &initial_frame {
                    assert!(
                        *previous == initial,
                        "reopening must restore the minimap scale"
                    );
                }
                initial_frame = Some(initial.clone());
                for key in [Keycode::Equals, Keycode::Plus, Keycode::KpPlus] {
                    app.key_down(key);
                }
                let zoomed = directory.join(format!("map-{}-zoom-in.ppm", id.index()));
                app.capture_path = Some(zoomed.clone());
                app.render(Duration::ZERO);
                assert!(
                    initial != std::fs::read(zoomed).unwrap(),
                    "zoom keys must change the map"
                );
                for key in [Keycode::Minus, Keycode::KpMinus] {
                    app.key_down(key);
                }
                app.mouse_wheel(1.0, sdl3::mouse::MouseWheelDirection::Flipped);
                app.mouse_wheel(0.5, sdl3::mouse::MouseWheelDirection::Normal);
                let wheel_path = directory.join(format!("map-{}-wheel.ppm", id.index()));
                app.capture_path = Some(wheel_path.clone());
                app.render(Duration::ZERO);
                assert!(
                    initial != std::fs::read(wheel_path).unwrap(),
                    "fractional wheel input must zoom"
                );
                app.mouse_wheel(-100.0, sdl3::mouse::MouseWheelDirection::Normal);
                app.capture_path = Some(directory.join(format!("map-{}-zoom-out.ppm", id.index())));
                app.render(Duration::ZERO);
                assert!(app.session.state == GameState::FullMap);
                assert!(app.keys.is_empty());
                assert_eq!(app.session.vehicle.position, position);
                app.capture_path = None;
                app.render(Duration::ZERO);
                assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
                app.key_down(Keycode::Escape);
                assert!(app.session.state == GameState::Driving);
                assert!(app.keys.is_empty());
                assert_eq!(app.session.vehicle.position, position);
            }
        }
        // Exercise the regular HUD after the map reused its vertex buffer.
        app.session = Session::new(CarModel::Roadster);
        app.snap_camera();
        app.render(Duration::ZERO);
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        app.key_down(Keycode::Escape);
        app.menu_click(1018.0, 576.0);
        assert!(app.session.state == GameState::Menu);
    }

    #[test]
    #[ignore = "requires SDL/OpenGL; verifies startup without a selected city"]
    fn no_city_startup_renderer_smoke() {
        let mut app = App::with_location(false);
        assert!(app.locations.len() >= 2);
        app.capture_path = std::env::var_os("NFS_MENU_PREVIEW").map(Into::into);
        app.render(Duration::ZERO);
        app.capture_path = None;
        if cars::ALL.len() > 3 {
            app.menu_action(hud::MenuAction::CarPage(1));
            assert_eq!(app.session.vehicle.model, cars::ALL[3]);
            app.render(Duration::ZERO);
            app.menu_action(hud::MenuAction::CarPage(-1));
            assert_eq!(app.session.vehicle.model, cars::ALL[0]);
        }
        app.menu_action(hud::MenuAction::Location(TrackId::Engomi));
        app.key_down(Keycode::F7);
        for _ in 0..6 {
            app.menu_focus = 0;
            app.key_down(Keycode::Down);
            app.render(Duration::ZERO);
            assert_ne!(app.session.track().id, TrackId::Engomi);
        }
        assert!(app.meshes[TrackId::Engomi.index()].is_none());
        app.menu_action(hud::MenuAction::Choice(1));
        assert_eq!(app.session.track().id, TrackId::Scenic);
        app.key_down(Keycode::Return);
        assert_eq!(app.session.track().id, TrackId::Compact);
        assert!(app.session.state == GameState::Driving);
        assert!(app.meshes[TrackId::Engomi.index()].is_none());
        app.render(Duration::ZERO);
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        drop(app);
        if let Ok(path) = city_assets::directory().canonicalize() {
            let mut app = App::with_location(true);
            let selected = &app.locations[app.selected_location];
            assert_eq!(
                selected.path.as_ref().unwrap().canonicalize().unwrap(),
                path
            );
            assert_ne!(app.session.track().id, TrackId::Engomi);
            assert!(app.meshes[TrackId::Engomi.index()].is_none());
            app.render(Duration::ZERO);
            assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
        }
    }

    #[test]
    fn free_drive_pause_info_and_reverse_have_no_race_statistics() {
        let mut s = Session::new(CarModel::Roadster);
        s.vehicle.toggle_direction();
        for _ in 0..20 {
            s.tick(Controls {
                throttle: 1.0,
                ..Controls::default()
            });
        }
        let position = s.vehicle.position;
        let phase = s.vehicle.wheel_phase;
        s.state = GameState::Paused;
        for _ in 0..120 {
            s.tick(Controls {
                throttle: 1.0,
                ..Controls::default()
            });
        }
        assert_eq!(s.vehicle.position, position);
        assert_eq!(s.vehicle.wheel_phase, phase);
        let lines = s.pause_lines("CLOSE", "ON");
        assert!(lines[0].contains("FREE DRIVE"));
        assert!(lines.iter().any(|s| s.contains("GEAR R")));
        assert!(!lines.iter().any(|s| s.starts_with("TIME ")
            || s.starts_with("LAP ")
            || s.starts_with("BEST ")
            || s.contains("POS ")));
    }
}
