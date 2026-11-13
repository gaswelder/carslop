mod add_location;
mod add_location_compile;
mod add_location_download;
mod add_location_trees;
mod aerial;
mod asset_car;
mod asset_codecs;
mod authoring;
mod boundary;
mod build_collision;
mod build_furniture;
mod build_landscape;
mod build_scene;
mod build_surfaces;
mod build_terrain;
mod camera;
mod chunks;
mod city;
mod city_assets;
mod create_track;
mod download;
mod elevation;
mod engine_sample;
mod error;
mod feedback;
mod file_io;
mod geometry;
mod gl;
mod hgt;
mod hud;
mod imagery;
mod landscape;
mod loading;
mod location;
mod osm;
mod particles;
mod phys_powertrain;
mod prefetch;
mod scene;
mod scene_archive;
mod shadows;
mod sound_car;
mod source_location;
mod sources;
mod src_files;
mod street_surface;
#[cfg(test)]
use world::test_support;
mod phys_car;
mod track;
mod traffic;
mod tree_obstacles;
mod tree_preview;
mod vehicle_location;
mod world;

use error::invalid;
use log::{info, warn};
use phys_car::{Controls, Vehicle};
use sdl3::{
    event::{Event, WindowEvent},
    keyboard::Keycode,
    video::Window,
};
use std::{
    collections::{HashMap, HashSet},
    path::PathBuf,
    time::{Duration, Instant},
};
use track::{Location, LocationId};

const HELP: &str = "Arguments: <command>
Commands:

    play
    add-location <OpenStreetMap relation URL>
    create-track <track.json>
";

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
    #[cfg(test)]
    fn on_track(model: std::sync::Arc<asset_car::CarSpec>, id: LocationId) -> Self {
        Self::on_world(model, id.try_track().unwrap())
    }

    fn on_world(
        model: std::sync::Arc<asset_car::CarSpec>,
        track: std::sync::Arc<Location>,
    ) -> Self {
        let start = track.spawn();
        Self {
            vehicle: {
                let mut vehicle =
                    Vehicle::on_track(start.vehicle_position(0.0), start.heading(), model, track);
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

    fn track(&self) -> &Location {
        &self.vehicle.track
    }

    fn restart(&mut self) {
        *self = Self::on_world(self.vehicle.spec.clone(), self.vehicle.track.clone());
    }

    fn select_car(&mut self, model: std::sync::Arc<asset_car::CarSpec>) {
        if self.state != GameState::Menu {
            return;
        }
        *self = Self::on_world(model, self.vehicle.track.clone());
        self.state = GameState::Menu;
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
        self.vehicle.reset(road.vehicle_position(0.0), heading);
        self.skids.break_trail(0);
        self.particles.reset_car(0);
        self.impact = 0.0;
        self.recoveries += 1;
        true
    }
}

struct App {
    // Audio and GL resources are dropped before their owning SDL context.
    audio: Option<sound_car::EngineAudio>,
    meshes: HashMap<LocationId, gl::Mesh>,
    locations_cache: track::LocationCache,
    skid_renderer: gl::SkidRenderer,
    particle_renderer: gl::ParticleRenderer,
    hud: hud::Hud,
    program: u32,
    scene_program: u32,
    _gl_context: sdl3::video::GLContext,
    window: Window,
    sdl_context: sdl3::Sdl,
    events: sdl3::EventPump,
    cars: Vec<std::sync::Arc<asset_car::CarSpec>>,
    selected_car: usize,
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
        let cars = asset_car::load(std::path::Path::new("data/cars")).expect("failed to load cars");
        assert!(!cars.is_empty(), "No valid cars in data/cars");
        let sdl_context = sdl3::init().expect("SDL init failed");
        let video = sdl_context.video().expect("failed to get video subsystem");
        let gl_attr = video.gl_attr();
        gl_attr.set_context_profile(sdl3::video::GLProfile::Core);
        gl_attr.set_context_version(3, 3);
        gl_attr.set_depth_size(24);
        gl_attr.set_multisample_buffers(1);
        gl_attr.set_multisample_samples(4);
        let events = sdl_context.event_pump().expect("failed to get event pump");
        let build_window = || {
            video
                .window("Cars LOP", WINDOW_WIDTH as u32, WINDOW_HEIGHT as u32)
                .opengl()
                .build()
        };
        let mut window = build_window()
            .or_else(|error| {
                log::warn!(
                    "Multisampled window unavailable: {error}; using the default framebuffer"
                );
                gl_attr.set_multisample_buffers(0);
                gl_attr.set_multisample_samples(0);
                build_window()
            })
            .expect("failed to create window");
        let gl_context = gl::create_gl_context(&window).expect("failed to create GL context");
        let _ = video.gl_set_swap_interval(sdl3::video::SwapInterval::VSync);
        gl::load_gl();
        gl::enable_depth_test();
        info!("{}", gl::debug_context_info());
        let mut locations = city_assets::menu_locations();
        let selected_city = city
            .then(|| LocationId::asset("data/locations/nicosia").ok())
            .flatten();
        let default_location = locations
            .iter()
            .position(|entry| {
                entry
                    .track
                    .asset_path()
                    .is_some_and(|p| p.file_name().is_some_and(|name| name == "scenic"))
                    || entry.name == "COASTAL HIGHWAY"
            })
            .unwrap_or(0);
        let selected_location = selected_city.map_or(default_location, |id| {
            if let Some(index) = locations.iter().position(|entry| entry.track == id) {
                index
            } else {
                locations.push(
                    city_assets::MenuLocation::from_assets(city_assets::CityAsset::new(
                        id.asset_path().unwrap().to_path_buf(),
                    ))
                    .expect("resolved location directory"),
                );
                locations.len() - 1
            }
        });
        // City data loads only after Start. The menu needs a car preview, not
        // the original point file used to author a compiled location.
        let preview = locations
            .get(selected_location)
            .filter(|entry| entry.track.asset_path().is_none())
            .map_or(LocationId::Compact, |entry| entry.track.clone());
        let mut session = Session::on_world(
            cars[0].clone(),
            preview.try_track().expect("menu preview world"),
        );
        session.state = GameState::Menu;
        let _ = window.set_title(&session.track().display_name);

        let program =
            gl::create_program(gl::VS_SRC, gl::FS_SRC).expect("failed to create GL program");
        let scene_program =
            gl::create_program(gl::SCENE_VS, gl::SCENE_FS).expect("failed to create scene program");
        let skid_renderer = gl::SkidRenderer::new();
        let particle_renderer =
            gl::ParticleRenderer::new().expect("failed to create particle renderer");
        let hud = hud::Hud::new();
        let audio = match sound_car::EngineAudio::new(&sdl_context) {
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
            meshes: HashMap::new(),
            locations_cache: track::LocationCache::default(),
            skid_renderer,
            particle_renderer,
            hud,
            audio,
            cars,
            selected_car: 0,
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
        let id = self.locations[self.selected_location].track.clone();
        let Some(world) = loading::prepare(
            id,
            self.locations_cache.clone(),
            &mut self.window,
            &mut self.events,
            &self.sdl_context,
        ) else {
            self.keys.clear();
            return;
        };
        self.keys.clear();
        if let Err(error) = self.install_location(world) {
            warn!("Cannot prepare location geometry: {error}");
        }
    }

    fn install_location(&mut self, world: std::sync::Arc<Location>) -> Result<(), String> {
        // Keep the active session and all its GL resources until candidate creation succeeds.
        let candidate_mesh = if self.meshes.contains_key(&world.id) {
            None
        } else {
            Some(gl::Mesh::new(&world, &self.cars)?)
        };
        if let Some(mesh) = candidate_mesh {
            self.meshes.insert(world.id.clone(), mesh);
        }
        // Bound retained city GPU data while keeping authored preview meshes reusable.
        self.meshes
            .retain(|key, _| key.asset_path().is_none() || key == &world.id);
        self.locations_cache.install(world.clone());
        self.session = Session::on_world(self.session.vehicle.spec.clone(), world);
        self.restart();
        Ok(())
    }

    fn ensure_mesh(&mut self) {
        let track = self.session.track();
        if !self.meshes.contains_key(&track.id) {
            if track.has_compiled_roads() {
                loading::draw(
                    &self.window,
                    &track.display_name,
                    "PREPARING VEHICLES AND SCENERY",
                    0.0,
                );
                gl::swap_window(&self.window);
            }
            self.meshes.insert(
                track.id.clone(),
                gl::Mesh::new(track, &self.cars).expect("failed to create route mesh"),
            );
        }
    }

    fn mesh(&self) -> &gl::Mesh {
        self.meshes
            .get(&self.session.track().id)
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

    fn select_car(&mut self, index: usize) {
        if self.session.state == GameState::Menu {
            if let Some(spec) = self.cars.get(index) {
                self.selected_car = index;
                self.session.select_car(spec.clone());
            }
        }
    }

    fn cycle_car(&mut self, direction: i32) {
        let next =
            (self.selected_car as i32 + direction).rem_euclid(self.cars.len() as i32) as usize;
        self.select_car(next);
    }

    fn menu_action(&mut self, action: hud::MenuAction) {
        if self.session.state != GameState::Menu {
            return;
        }
        match action {
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
                let pages = self.cars.len().div_ceil(3);
                let page =
                    (self.selected_car as i32 / 3 + direction).rem_euclid(pages as i32) as usize;
                self.select_car(page * 3);
                self.menu_focus = 1;
            }
            hud::MenuAction::Car(index) => {
                self.menu_focus = 1;
                self.select_car(index);
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
                self.selected_car,
                self.cars.len(),
            ) {
                self.menu_action(action);
            }
        }
    }
    fn key_down(&mut self, key: Keycode) {
        if self.session.state == GameState::FullMap {
            match key {
                Keycode::Escape | Keycode::M => self.session.state = GameState::Driving,
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
                        1 => self.cycle_car(direction),
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
            .google_maps_url(self.session.vehicle.position);
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
            status,
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
                    Keycode::F1 => 0,
                    Keycode::F2 => 1,
                    _ => 2,
                };
                self.select_car(model);
                self.snap_camera();
            }
            Keycode::F6 if self.session.state == GameState::Menu => {
                self.selected_location = if self.selected_location == 0 { 1 } else { 0 };
            }
            Keycode::F7 if self.session.state == GameState::Menu => {
                if let Some(index) = self.locations.iter().position(|entry| {
                    entry
                        .track
                        .asset_path()
                        .is_some_and(|path| path.ends_with("nicosia"))
                }) {
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
            capture_frame(path);
        }
        gl::swap_window(&self.window);
        let _ = self.window.set_title(&format!(
            "{} | {} | FREE DRIVE",
            self.session.track().display_name,
            self.session.vehicle.spec.name,
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
                self.session
                    .track()
                    .has_compiled_roads()
                    .then_some(self.session.track()),
            )
        } else if self.chase.view() == camera::FollowView::Bonnet {
            gl::bonnet_camera(aspect, self.chase.pose(), car)
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
            self.mesh().draw(
                self.scene_program,
                &camera,
                &model,
                tint,
                vehicle,
                self.selected_car,
            );
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
        if in_menu {
            self.hud.draw_menu(
                self.program,
                hud::MainMenu {
                    locations: &self.locations,
                    selected: self.selected_location,
                    car: self.selected_car,
                    cars: &self.cars,
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
                car.heading,
                hud::Gauges {
                    rpm: car.engine_rpm,
                    redline_rpm: car.spec.redline_rpm,
                    speed_kmh: car.speed.abs() * 3.6,
                    gear: car.gear,
                },
                (s.state == GameState::Paused).then_some(hud::PauseMenu {
                    city: s.track().has_compiled_roads(),
                    aerial_attribution: s.track().assets().and_then(|a| a.aerial_attribution()),
                    view: self.chase.view().label(),
                    audio: audio_status,
                }),
            );
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut args = std::env::args_os().skip(1);
    if let Some(command) = args.next() {
        let command = command
            .into_string()
            .map_err(|_| invalid("expected UTF-8 command"))?;
        match command.as_str() {
            "add-location" => {
                let url = args
                    .next()
                    .ok_or_else(|| {
                        invalid("add-location requires exactly one OpenStreetMap relation URL")
                    })?
                    .into_string()
                    .map_err(|_| invalid("expected a UTF-8 relation URL"))?;
                if args.next().is_some() {
                    return Err(invalid(
                        "add-location requires exactly one OpenStreetMap relation URL",
                    ));
                }
                add_location::add_location(&url)?;
                return Ok(());
            }
            "create-track" => {
                let source = args
                    .next()
                    .ok_or_else(|| invalid("create-track requires exactly one JSON path"))?;
                if args.next().is_some() {
                    return Err(invalid("create-track requires exactly one JSON path"));
                }
                let destination = create_track::create_track(&PathBuf::from(source))?;
                println!("Created {}", destination.display());
                return Ok(());
            }
            "play" => {}
            _ => return Err(invalid(format!("unknown command: {command}\n\n{HELP}"))),
        }
    }
    info!("Cars LOP: starting");
    App::with_location(false).run()
}

#[cfg(test)]
fn capture_frame(path: &std::path::Path) {
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
