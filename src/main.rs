mod aerial;
mod ai;
mod ambience;
mod audio;
mod camera;
mod cars;
mod chunks;
mod city;
mod city_assets;
mod cli;
mod elevation;
mod feedback;
mod gl;
mod hud;
mod landscape;
mod location;
mod particles;
mod prefetch;
mod race;
mod scene;
mod scene_archive;
mod shadows;
mod street_surface;
mod track;
mod traffic;
mod vehicle;

use cars::CarModel;
use log::{info, warn};
use race::{format_time, RaceProgress};
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
    Finishing,
    Paused,
    Finished,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DriveMode {
    FreeDrive,
    Race,
}

impl DriveMode {
    fn label(self) -> &'static str {
        match self {
            Self::FreeDrive => "FREE DRIVE",
            Self::Race => "RACE",
        }
    }
}

struct Session {
    mode: DriveMode,
    vehicle: Vehicle,
    rival: Vehicle,
    race: RaceProgress,
    rival_race: RaceProgress,
    countdown: f32,
    state: GameState,
    recoveries: u32,
    skids: feedback::SkidMarks,
    particles: particles::TireParticles,
    impact: f32,
    signal_time: f32,
}

impl Session {
    fn new(model: CarModel, mode: DriveMode) -> Self {
        Self::on_track(
            model,
            mode,
            if mode == DriveMode::Race {
                TrackId::Compact
            } else {
                TrackId::Scenic
            },
        )
    }

    fn on_track(model: CarModel, mode: DriveMode, id: TrackId) -> Self {
        // RaceProgress deliberately retains its original compact-route rules.
        let track = if mode == DriveMode::Race {
            TrackId::Compact
        } else {
            id
        }
        .track();
        let start = track.spawn();
        Self {
            mode,
            vehicle: Vehicle::on_track(
                start.lane_position(if mode == DriveMode::Race { -1.45 } else { 0.0 }),
                start.heading(),
                model,
                track,
            ),
            rival: Vehicle::on_track(
                start.lane_position(1.45),
                start.heading(),
                CarModel::Roadster,
                track,
            ),
            race: RaceProgress::new(),
            rival_race: RaceProgress::new(),
            countdown: if mode == DriveMode::Race { 3.0 } else { 0.0 },
            state: GameState::Driving,
            recoveries: 0,
            skids: feedback::SkidMarks::default(),
            particles: particles::TireParticles::default(),
            impact: 0.0,
            signal_time: 0.0,
        }
    }

    fn track(&self) -> &'static Track {
        self.vehicle.track
    }

    fn restart(&mut self) {
        *self = Self::on_track(self.vehicle.model, self.mode, self.track().id);
    }

    fn select_car(&mut self, model: CarModel) {
        if self.state != GameState::Menu {
            return;
        }
        *self = Self::on_track(model, self.mode, self.track().id);
        self.state = GameState::Menu;
    }

    fn select_mode(&mut self, mode: DriveMode) {
        if self.state != GameState::Menu {
            return;
        }
        *self = Self::new(self.vehicle.model, mode);
        self.state = GameState::Menu;
    }

    fn toggle_track(&mut self) {
        if self.state != GameState::Menu || self.mode != DriveMode::FreeDrive {
            return;
        }
        let id = match self.track().id {
            TrackId::Compact => TrackId::Scenic,
            TrackId::Scenic => TrackId::Compact,
            TrackId::Engomi => TrackId::Scenic,
        };
        *self = Self::on_track(self.vehicle.model, self.mode, id);
        self.state = GameState::Menu;
    }

    fn select_location(&mut self, id: TrackId) {
        if self.state != GameState::Menu {
            return;
        }
        let mode = if id == TrackId::Compact {
            self.mode
        } else {
            DriveMode::FreeDrive
        };
        *self = Self::on_track(self.vehicle.model, mode, id);
        self.state = GameState::Menu;
    }
    fn cycle_location(&mut self, direction: i32) {
        let current = hud::LOCATIONS
            .iter()
            .position(|id| *id == self.track().id)
            .unwrap();
        let next = (current as i32 + direction).rem_euclid(hud::LOCATIONS.len() as i32) as usize;
        self.select_location(hud::LOCATIONS[next]);
    }
    fn cycle_car(&mut self, direction: i32) {
        let next = (self.vehicle.model.index() as i32 + direction)
            .rem_euclid(cars::ALL.len() as i32) as usize;
        self.select_car(cars::ALL[next]);
    }
    fn toggle_activity(&mut self) {
        if self.state != GameState::Menu || self.track().id != TrackId::Compact {
            return;
        }
        let mode = if self.mode == DriveMode::FreeDrive {
            DriveMode::Race
        } else {
            DriveMode::FreeDrive
        };
        *self = Self::on_track(self.vehicle.model, mode, TrackId::Compact);
        self.state = GameState::Menu;
    }

    fn has_rival(&self) -> bool {
        self.mode == DriveMode::Race
    }

    fn update_feedback(&mut self) {
        self.skids.advance(TICK);
        self.skids.record(0, &self.vehicle);
        self.particles.advance(TICK);
        self.particles.record(0, &self.vehicle, TICK);
        if self.has_rival() {
            self.skids.record(1, &self.rival);
            self.particles.record(1, &self.rival, TICK);
        }
    }

    fn tick(&mut self, controls: Controls) {
        if !matches!(self.state, GameState::Driving | GameState::Finishing) {
            return;
        }
        if self.countdown > 0.0 {
            self.countdown = (self.countdown - TICK.as_secs_f32()).max(0.0);
            return;
        }
        self.signal_time =
            (self.signal_time + TICK.as_secs_f32()).rem_euclid(traffic::CYCLE_SECONDS);
        if self.mode == DriveMode::FreeDrive {
            self.impact = self.impact.max(self.vehicle.update(TICK, controls));
            self.update_feedback();
            return;
        }
        let previous = self.vehicle.position;
        let rival_previous = self.rival.position;
        let player_controls = if self.race.finish_time.is_some() {
            Controls {
                brake: 1.0,
                ..Controls::default()
            }
        } else {
            controls
        };
        self.impact = self.impact.max(self.vehicle.update(TICK, player_controls));
        let rival_controls = if self.rival_race.finish_time.is_some() {
            Controls {
                brake: 1.0,
                ..Controls::default()
            }
        } else {
            ai::controls(&self.rival, Some(&self.vehicle))
        };
        self.rival.update(TICK, rival_controls);
        self.impact = self.impact.max(self.vehicle.collide(&mut self.rival));
        self.update_feedback();
        self.race.update(previous, self.vehicle.position, TICK);
        self.rival_race
            .update(rival_previous, self.rival.position, TICK);
        if let Some(player_time) = self.race.finish_time {
            // A player who crossed first can still lose on recovery penalties.
            // Let the rival finish, or exhaust that adjusted time, before
            // freezing the simulation and awarding a definitive result.
            self.state = if self.rival_race.finish_time.is_some()
                || self.rival_race.elapsed >= player_time
            {
                GameState::Finished
            } else {
                GameState::Finishing
            };
        }
    }

    fn player_leads(&self) -> bool {
        match (self.race.finish_time, self.rival_race.finish_time) {
            (Some(player), Some(rival)) => player <= rival,
            (Some(_), None) => true,
            (None, Some(_)) => false,
            _ => {
                self.race.score(self.vehicle.position) >= self.rival_race.score(self.rival.position)
            }
        }
    }

    fn recover(&mut self) -> bool {
        if self.state != GameState::Driving || self.countdown > 0.0 {
            return false;
        }
        if self.mode == DriveMode::FreeDrive {
            let road = self.track().recovery(self.vehicle.position);
            let backwards = (self.vehicle.heading - road.heading()).cos() < 0.0;
            let heading = road.heading() + if backwards { std::f32::consts::PI } else { 0.0 };
            self.vehicle.reset(road.lane_position(0.0), heading);
        } else {
            // Race recovery remains behind the next gate and costs five seconds.
            let distance =
                self.race.next_gate as f32 * self.track().length() / race::GATES as f32 + 1.0;
            let point = self.track().at_distance(distance);
            self.vehicle
                .reset(point.lane_position(-1.45), point.heading());
            self.race.elapsed += 5.0;
        }
        self.skids.break_trail(0);
        self.particles.reset_car(0);
        self.impact = 0.0;
        self.recoveries += 1;
        true
    }

    fn pause_lines(&self, view: &str, audio: &str) -> Vec<String> {
        let car = &self.vehicle;
        let speed = format!(
            "{:03} KM/H GEAR {} {}",
            (car.speed.abs() * 3.6) as u32,
            car.gear_label(),
            view
        );
        let status = format!("AUDIO {audio}");
        let mut lines = if self.mode == DriveMode::FreeDrive {
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
                "NO TIMER / NO RIVAL".into(),
                "X SHIFT DRIVE/REVERSE".into(),
                "C NEARBY RECOVERY".into(),
                speed,
                status,
            ]
        } else {
            vec![
                format!(
                    "{}   POS {}/2",
                    car.model.spec().name,
                    if self.player_leads() { 1 } else { 2 }
                ),
                format!(
                    "LAP {}/{}   GATE {}/{}",
                    (self.race.completed_laps + 1).min(race::LAPS),
                    race::LAPS,
                    self.race.next_gate + 1,
                    race::GATES
                ),
                format!("TIME {}", format_time(self.race.elapsed)),
                format!(
                    "LAP  {}",
                    format_time(self.race.elapsed - self.race.lap_started)
                ),
                format!(
                    "BEST {}",
                    self.race
                        .best_lap()
                        .map(format_time)
                        .unwrap_or_else(|| "--:--.--".into())
                ),
                speed,
                status,
            ]
        };
        if self.mode == DriveMode::Race && self.race.finish_time.is_some() {
            lines.push("RESULT: WAITING FOR RIVAL".into());
        }
        lines
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
    session: Session,
    chase: camera::ChaseCamera,
    preview_angle: f32,
    menu_focus: usize,
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
        let mut session = if city {
            Session::on_track(CarModel::Roadster, DriveMode::FreeDrive, TrackId::Engomi)
        } else {
            Session::new(CarModel::Roadster, DriveMode::FreeDrive)
        };
        session.state = GameState::Menu;
        let window = video
            .window(
                session.track().name,
                WINDOW_WIDTH as u32,
                WINDOW_HEIGHT as u32,
            )
            .opengl()
            .build()
            .expect("failed to create window");
        let gl_context = gl::create_gl_context(&window).expect("failed to create GL context");
        let _ = video.gl_set_swap_interval(sdl3::video::SwapInterval::VSync);
        gl::load_gl();
        gl::enable_depth_test();
        info!("{}", gl::debug_context_info());
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
            #[cfg(test)]
            capture_path: None,
            keys: HashSet::new(),
        };
        app.ensure_mesh();
        app
    }

    fn ensure_mesh(&mut self) {
        let track = self.session.track();
        let slot = &mut self.meshes[track.id.index()];
        if slot.is_none() {
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
        let mut events = self.sdl_context.event_pump()?;
        let mut last_frame = Instant::now();
        let mut accumulator = Duration::ZERO;
        'running: loop {
            for event in events.poll_iter() {
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
                        if matches!(
                            self.session.state,
                            GameState::Driving | GameState::Finishing
                        ) {
                            self.session.state = GameState::Paused;
                        }
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
                    throttle: held(&[Keycode::W, Keycode::Up, Keycode::F]) as u8 as f32,
                    brake: held(&[Keycode::S, Keycode::Down, Keycode::B]) as u8 as f32,
                    steering: held(&[Keycode::D, Keycode::Right]) as u8 as f32
                        - held(&[Keycode::A, Keycode::Left]) as u8 as f32,
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
                    self.keys.contains(&Keycode::W)
                        || self.keys.contains(&Keycode::Up)
                        || self.keys.contains(&Keycode::F),
                    matches!(
                        self.session.state,
                        GameState::Driving | GameState::Finishing
                    ),
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
                self.menu_focus = 0;
                self.session.select_location(id);
            }
            hud::MenuAction::Car(car) => {
                self.menu_focus = 1;
                self.session.select_car(car);
            }
            hud::MenuAction::Activity => {
                self.menu_focus = 2;
                self.session.toggle_activity();
            }
            hud::MenuAction::Start => {
                self.restart();
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
                    hud::PauseAction::Selection => Keycode::M,
                    hud::PauseAction::Audio => Keycode::V,
                    hud::PauseAction::Camera => Keycode::Tab,
                    hud::PauseAction::Recover => Keycode::C,
                });
            }
        } else if self.session.state == GameState::Menu {
            if let Some(action) = hud::menu_hit_test(point[0], point[1]) {
                self.menu_action(action);
            }
        }
    }
    fn key_down(&mut self, key: Keycode) {
        self.keys.insert(key);
        if self.session.state == GameState::Menu {
            match key {
                Keycode::Up => self.menu_focus = (self.menu_focus + 3) % 4,
                Keycode::Down | Keycode::Tab => self.menu_focus = (self.menu_focus + 1) % 4,
                Keycode::Left | Keycode::Right | Keycode::Space => {
                    let direction = if key == Keycode::Left { -1 } else { 1 };
                    match self.menu_focus {
                        0 => self.session.cycle_location(direction),
                        1 => self.session.cycle_car(direction),
                        2 => self.session.toggle_activity(),
                        _ if key == Keycode::Space => {
                            self.restart();
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
    fn handle_key(&mut self, key: Keycode) {
        match key {
            Keycode::Escape => {
                self.session.state = match self.session.state {
                    GameState::Driving | GameState::Finishing => GameState::Paused,
                    GameState::Paused => {
                        if self.session.race.finish_time.is_some() {
                            GameState::Finishing
                        } else {
                            GameState::Driving
                        }
                    }
                    other => other,
                };
                self.keys.clear();
            }
            Keycode::Return
                if matches!(self.session.state, GameState::Menu | GameState::Finished) =>
            {
                self.restart()
            }
            Keycode::M => {
                self.menu_focus = 0;
                self.session.state = GameState::Menu;
                self.keys.clear();
            }
            Keycode::R => self.restart(),
            Keycode::C => {
                let paused = self.session.state == GameState::Paused;
                if paused && self.session.race.finish_time.is_none() {
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
            Keycode::F4 | Keycode::F5 if self.session.state == GameState::Menu => {
                self.session.select_mode(if key == Keycode::F4 {
                    DriveMode::FreeDrive
                } else {
                    DriveMode::Race
                });
                self.ensure_mesh();
                self.snap_camera();
            }
            Keycode::F6
                if self.session.state == GameState::Menu
                    && self.session.mode == DriveMode::FreeDrive =>
            {
                self.session.toggle_track();
                self.ensure_mesh();
                self.snap_camera();
            }
            Keycode::F7
                if self.session.state == GameState::Menu
                    && self.session.mode == DriveMode::FreeDrive =>
            {
                self.session = Session::on_track(
                    self.session.vehicle.model,
                    DriveMode::FreeDrive,
                    TrackId::Engomi,
                );
                self.session.state = GameState::Menu;
                self.ensure_mesh();
                self.snap_camera();
            }
            Keycode::X
                if self.session.state == GameState::Driving && self.session.countdown <= 0.0 =>
            {
                self.session.vehicle.toggle_direction();
            }
            Keycode::V => {
                if let Some(audio) = &mut self.audio {
                    audio.muted = !audio.muted;
                }
            }
            Keycode::Tab
                if matches!(
                    self.session.state,
                    GameState::Driving | GameState::Finishing | GameState::Paused
                ) =>
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
        gl::set_signal_time(self.scene_program, self.session.signal_time);
        let car = &self.session.vehicle;
        if matches!(
            self.session.state,
            GameState::Driving | GameState::Finishing
        ) {
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
        let vehicles = [
            (&self.session.vehicle, [1.0, 0.18, 0.08]),
            (&self.session.rival, [0.12, 0.65, 1.0]),
        ];
        let visible_count = if in_menu || !self.session.has_rival() {
            1
        } else {
            2
        };
        let contacts = vehicles.map(|(car, _)| shadows::ContactShadow::new(car));
        self.mesh()
            .draw_ground(self.scene_program, &camera, &contacts[..visible_count]);
        if !in_menu {
            self.skid_renderer
                .draw(self.scene_program, &camera, &self.session.skids);
        }
        for &(vehicle, tint) in &vehicles[..visible_count] {
            let track = self.session.track();
            let (road_pitch, road_roll) = if let Some(city) = track.city() {
                let h = |dx: f32, dz: f32| {
                    city.drive_height([vehicle.position[0] + dx, vehicle.position[2] + dz])
                };
                let (sin, cos) = vehicle.heading.sin_cos();
                // Sample along this car, never across unrelated city graph edges.
                (
                    ((h(sin * 1.3, cos * 1.3) - h(-sin * 1.3, -cos * 1.3)) / 2.6).atan(),
                    ((h(cos * 0.8, -sin * 0.8) - h(-cos * 0.8, sin * 0.8)) / 1.6).atan(),
                )
            } else {
                let road = track.project(vehicle.position);
                let grade = track.at_distance(road.distance + 0.5).point[1]
                    - track.at_distance(road.distance - 0.5).point[1];
                (
                    (grade * (vehicle.heading - road.heading()).cos()).atan(),
                    0.0,
                )
            };
            let mut body_position = vehicle.position;
            body_position[1] += vehicle.curb_bump;
            let model = gl::body_model_matrix(
                body_position,
                vehicle.heading,
                road_pitch + vehicle.body_pitch,
                road_roll + vehicle.body_roll,
            );
            self.mesh()
                .draw(self.scene_program, &camera, &model, tint, vehicle);
        }
        // Transparent puffs follow all opaque world/car draws and precede the HUD.
        if !in_menu {
            self.particle_renderer
                .draw(&camera, &self.session.particles);
        }
        let s = &self.session;
        let spec = s.vehicle.model.spec();
        let profile = spec.name;
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
        let overlay = match s.state {
            GameState::Menu => Vec::new(),
            GameState::Paused | GameState::Finishing => Vec::new(),
            GameState::Finished => {
                let mut entries = vec![
                    if s.player_leads() {
                        "YOU WIN".into()
                    } else {
                        "SECOND PLACE".into()
                    },
                    format!("TOTAL {}", format_time(s.race.elapsed)),
                    format!(
                        "BEST LAP {}",
                        s.race.best_lap().map(format_time).unwrap_or_default()
                    ),
                ];
                for (i, time) in s.race.lap_times.iter().enumerate() {
                    entries.push(format!("LAP {}  {}", i + 1, format_time(*time)));
                }
                entries.push("ENTER TO RACE AGAIN".into());
                entries
            }
            GameState::Driving if s.countdown > 0.0 => {
                vec!["GET READY".into(), format!("{}", s.countdown.ceil() as u32)]
            }
            _ => Vec::new(),
        };
        if in_menu {
            self.hud.draw_menu(
                self.program,
                hud::MainMenu {
                    location: s.track().id,
                    car: s.vehicle.model,
                    race: s.mode == DriveMode::Race,
                    focus: self.menu_focus,
                },
            );
        } else {
            self.hud.draw(
                self.program,
                hud::Map {
                    track: s.track(),
                    player: car.position,
                    rival: s.has_rival().then_some(s.rival.position),
                    gate: s.has_rival().then(|| s.race.gate().point),
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
                    load: car.engine_load,
                }),
                &overlay,
            );
        }
        #[cfg(test)]
        if let Some(path) = &self.capture_path {
            tests::capture_frame(path);
        }
        gl::swap_window(&self.window);
        let _ = self.window.set_title(&format!(
            "{} | {} | {}",
            s.track().name,
            profile,
            s.mode.label()
        ));
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
        for row in pixels.chunks_exact((WINDOW_WIDTH * 3) as usize).rev() {
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
        assert_eq!(app.session.track().id, TrackId::Engomi);
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
    #[ignore = "requires SDL3/OpenGL; exports Engomi views to NFS_CITY_PREVIEW_DIR"]
    fn engomi_renderer_smoke() {
        let directory = std::path::PathBuf::from(
            std::env::var("NFS_CITY_PREVIEW_DIR").unwrap_or_else(|_| "/tmp/engomi-preview".into()),
        );
        std::fs::create_dir_all(&directory).unwrap();
        let mut app = App::new();
        let loading = std::time::Instant::now();
        app.key_down(Keycode::F7);
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
        ] {
            if app.chase.view() != view {
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
        // Review mapped planting and adjacent paving from an unobstructed oblique view.
        let city = app.session.track().city().unwrap();
        let tree = city
            .landscape
            .trees
            .iter()
            .max_by_key(|tree| {
                city.landscape
                    .trees
                    .iter()
                    .filter(|other| {
                        (tree.point[0] - other.point[0]).hypot(tree.point[1] - other.point[1])
                            < 45.0
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
        app.key_down(Keycode::M);
        app.key_down(Keycode::F6);
        app.key_down(Keycode::F7);
        assert!(std::ptr::eq(app.mesh(), mesh));
        app.key_down(Keycode::F5);
        app.key_down(Keycode::F7);
        assert_eq!(app.session.track().id, TrackId::Compact);
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
    #[ignore = "requires SDL3/OpenGL; verifies opening menu mouse and keyboard selection"]
    fn main_menu_renderer_smoke() {
        let directory = std::path::PathBuf::from(
            std::env::var("NFS_MENU_PREVIEW_DIR")
                .unwrap_or_else(|_| "/tmp/main-menu-preview".into()),
        );
        std::fs::create_dir_all(&directory).unwrap();
        let mut app = App::new();
        assert!(app.session.state == GameState::Menu);
        app.capture_path = Some(directory.join("startup.ppm"));
        app.render(Duration::ZERO);
        app.menu_click(320.0, 338.0);
        app.menu_click(550.0, 437.0);
        assert_eq!(app.session.track().id, TrackId::Engomi);
        assert_eq!(app.session.vehicle.model, CarModel::Club);
        assert!(app.session.state == GameState::Menu);
        app.capture_path = Some(directory.join("engomi-club.ppm"));
        app.render(Duration::ZERO);
        app.menu_click(350.0, 580.0);
        assert!(app.session.state == GameState::Driving);
        assert_eq!(app.session.track().id, TrackId::Engomi);
        app.capture_path = None;
        app.key_down(Keycode::M);
        app.key_down(Keycode::Left);
        assert_eq!(app.session.track().id, TrackId::Compact);
        app.key_down(Keycode::Down);
        app.key_down(Keycode::Right);
        assert_eq!(app.session.vehicle.model, CarModel::Roadster);
        app.key_down(Keycode::Down);
        app.key_down(Keycode::Right);
        assert_eq!(app.session.mode, DriveMode::Race);
        app.capture_path = Some(directory.join("loop-race.ppm"));
        app.render(Duration::ZERO);
        app.key_down(Keycode::Return);
        assert!(app.session.state == GameState::Driving);
        assert_eq!(app.session.track().id, TrackId::Compact);
        assert_eq!(app.session.countdown, 3.0);
        assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
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
        let car = &app.session.vehicle;
        let p = car.position;
        for (name, offset) in [
            ("front", [4.2, 2.1, 5.0]),
            ("rear", [-4.2, 1.8, -5.0]),
            ("side", [6.0, 1.0, 0.0]),
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
            app.mesh()
                .draw(app.scene_program, &camera, &model, [0.60, 0.12, 0.065], car);
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
            let mut file =
                std::fs::File::create(format!("{directory}/kestrel-{name}.ppm")).unwrap();
            write!(file, "P6\n{} {}\n255\n", WINDOW_WIDTH, WINDOW_HEIGHT).unwrap();
            for row in pixels.chunks_exact((WINDOW_WIDTH * 3) as usize).rev() {
                file.write_all(row).unwrap();
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
    #[ignore = "requires SDL3/OpenGL; run through Check race build and renderer"]
    fn feedback_renderer_smoke() {
        let mut app = App::new();
        assert_eq!(app.session.mode, DriveMode::FreeDrive);
        assert_eq!(app.session.track().id, TrackId::Scenic);
        assert!(app.meshes[TrackId::Scenic.index()].is_some());
        assert!(app.meshes[TrackId::Compact.index()].is_none());
        for model in cars::ALL {
            app.session = Session::new(model, DriveMode::Race);
            app.ensure_mesh();
            app.session.countdown = 0.0;
            for distance in [50.0, 50.6] {
                let road = track::at_distance(distance);
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
            let road = track::at_distance(50.6);
            let mut edge = Vehicle::new(road.lane_position(2.15), road.heading(), model);
            edge.velocity = road.tangent.map(|v| v * 8.0);
            for _ in 0..20 {
                app.session.particles.advance(TICK);
                app.session.particles.record(0, &app.session.vehicle, TICK);
                app.session.particles.record(1, &edge, TICK);
            }
            app.snap_camera();
            assert_contact_pixel_readback(&app);
            assert_particle_pixel_readback(&app);
            for view in [camera::FollowView::Wide, camera::FollowView::Close] {
                if app.chase.view() != view {
                    app.key_down(Keycode::Tab);
                }
                assert_eq!(app.chase.view(), view);
                for _ in 0..120 {
                    app.chase.update(&app.session.vehicle, TICK);
                }
                app.render(TICK);
                assert_eq!(contact_count(app.scene_program), 2);
                assert_eq!(
                    unsafe { ::gl::GetError() },
                    ::gl::NO_ERROR,
                    "GL errors rendering {model:?} in {view:?}"
                );
            }
            app.session.state = GameState::Paused;
            let frozen = app.chase.pose();
            app.render(TICK);
            assert_eq!(app.chase.view(), camera::FollowView::Close);
            assert_eq!(app.chase.pose(), frozen);
            app.restart();
            assert_eq!(app.chase.view(), camera::FollowView::Close);
            app.session.state = GameState::Menu;
            app.key_down(Keycode::Tab);
            assert_eq!(app.chase.view(), camera::FollowView::Close);
            app.render(TICK);
            assert_eq!(
                contact_count(app.scene_program),
                1,
                "hidden rival must not cast a garage shadow"
            );
            assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
            if let Some(audio) = &mut app.audio {
                audio.reset_feedback();
                audio.update(&app.session.vehicle, true, true, 8.0).unwrap();
                audio.muted = true;
                audio
                    .update(&app.session.vehicle, false, false, 0.0)
                    .unwrap();
                audio.muted = false;
            }
            app.key_down(Keycode::F4);
            assert_eq!(app.session.mode, DriveMode::FreeDrive);
            assert_eq!(app.session.track().id, TrackId::Scenic);
            let scenic_mesh = app.mesh() as *const gl::Mesh;
            app.key_down(Keycode::F6);
            assert_eq!(app.session.track().id, TrackId::Compact);
            let compact_mesh = app.mesh() as *const gl::Mesh;
            app.key_down(Keycode::F6);
            assert!(
                std::ptr::eq(app.mesh(), scenic_mesh),
                "route switch must reuse Scenic cache"
            );
            app.key_down(Keycode::F6);
            assert!(
                std::ptr::eq(app.mesh(), compact_mesh),
                "route switch must reuse Compact cache"
            );
            app.key_down(Keycode::Return);
            app.key_down(Keycode::F6);
            assert_eq!(
                app.session.track().id,
                TrackId::Compact,
                "F6 is garage-only"
            );
            assert!(
                std::ptr::eq(app.mesh(), compact_mesh),
                "restart must reuse mesh"
            );
            assert_eq!(app.session.countdown, 0.0);
            app.key_down(Keycode::X);
            assert_eq!(app.session.vehicle.gear_label(), "R");
            for _ in 0..30 {
                app.session.tick(Controls {
                    throttle: 1.0,
                    ..Controls::default()
                });
            }
            assert!(app.session.vehicle.speed < 0.0);
            for view in [camera::FollowView::Wide, camera::FollowView::Close] {
                if app.chase.view() != view {
                    app.key_down(Keycode::Tab);
                }
                app.render(TICK);
                assert_eq!(
                    contact_count(app.scene_program),
                    1,
                    "Free Drive has no rival shadow"
                );
                assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
            }
            app.key_down(Keycode::R);
            assert_eq!(app.session.mode, DriveMode::FreeDrive);
            assert_eq!(app.session.vehicle.gear_label(), "1");
            assert_eq!(app.session.countdown, 0.0);
            app.key_down(Keycode::M);
            app.key_down(Keycode::F5);
            app.key_down(Keycode::F6);
            assert_eq!(
                app.session.track().id,
                TrackId::Compact,
                "Race ignores F6 even in garage"
            );
            app.key_down(Keycode::Return);
            assert_eq!(app.session.mode, DriveMode::Race);
            assert_eq!(app.session.countdown, 3.0);
            app.key_down(Keycode::X);
            assert_eq!(
                app.session.vehicle.direction,
                vehicle::Direction::Drive,
                "no shifting during countdown"
            );
        }

        // Scenic fixtures are sampled from the route, never compact world coordinates.
        for model in cars::ALL {
            app.session = Session::on_track(model, DriveMode::FreeDrive, TrackId::Scenic);
            app.ensure_mesh();
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
                    if app.chase.view() != view {
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
                    "Scenic garage hides rival"
                );
                assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
                app.session.state = GameState::Driving;
            }
        }
    }

    #[test]
    fn route_defaults_and_race_constructor_guard_share_one_track_for_both_cars() {
        for model in cars::ALL {
            let free = Session::new(model, DriveMode::FreeDrive);
            assert_eq!(free.track().id, TrackId::Scenic);
            let race = Session::new(model, DriveMode::Race);
            assert_eq!(race.track().id, TrackId::Compact);
            for id in track::ALL {
                for mode in [DriveMode::FreeDrive, DriveMode::Race] {
                    let s = Session::on_track(model, mode, id);
                    let expected = if mode == DriveMode::Race {
                        TrackId::Compact
                    } else {
                        id
                    }
                    .track();
                    assert!(std::ptr::eq(s.track(), expected));
                    assert!(std::ptr::eq(s.vehicle.track, s.rival.track));
                    let spawn = expected.spawn();
                    assert_eq!(
                        s.vehicle.position,
                        spawn.lane_position(if mode == DriveMode::Race { -1.45 } else { 0.0 })
                    );
                    assert_eq!(s.rival.position, spawn.lane_position(1.45));
                }
            }
        }
    }

    #[test]
    fn scenic_reference_drive_completes_a_full_loop_without_recovery() {
        for model in cars::ALL {
            let mut s = Session::new(model, DriveMode::FreeDrive);
            let route = s.track();
            let mut previous = route.project(s.vehicle.position).distance;
            let mut travelled = 0.0;
            let mut seconds = 0.0;
            let mut hard_contacts = 0;
            let mut peak_speed: f32 = 0.0;
            for _ in 0..60 * 300 {
                s.tick(ai::controls(&s.vehicle, None));
                let road = route.project(s.vehicle.position);
                let mut delta = road.distance - previous;
                if delta < -route.length() * 0.5 {
                    delta += route.length();
                }
                if delta > route.length() * 0.5 {
                    delta -= route.length();
                }
                travelled += delta;
                previous = road.distance;
                seconds += TICK.as_secs_f32();
                peak_speed = peak_speed.max(s.vehicle.speed.max(0.0));
                if s.impact > 3.0 {
                    hard_contacts += 1;
                }
                s.impact = 0.0;
                assert!(s.vehicle.position.iter().all(|p| p.is_finite()));
                assert!(road.offset.abs() <= route.width * 0.5 - track::CAR_RADIUS + 0.05);
                if travelled >= route.length() {
                    break;
                }
            }
            assert!(
                travelled >= route.length(),
                "{model:?} reference drive stalled after {travelled:.0} m / {seconds:.1} s"
            );
            assert!(
                peak_speed > 20.0,
                "{model:?} reference drive did not exercise Highway pace: {:.1} km/h",
                peak_speed * 3.6
            );
            assert!(
                hard_contacts < 3,
                "{model:?} reference drive hit barriers {hard_contacts} times"
            );
            assert_eq!(s.recoveries, 0);
            assert_eq!(s.race.elapsed, 0.0);
            assert!(s.state == GameState::Driving);
            // This is conservative automated pursuit, not a human pace/feel claim.
            eprintln!("Scenic {model:?} reference: {:.0} m in {seconds:.1} s; peak {:.0} km/h; {hard_contacts} hard contacts",
                route.length(), peak_speed * 3.6);
        }
    }

    #[test]
    fn free_route_toggle_is_garage_only_and_race_always_stays_compact() {
        let mut s = Session::new(CarModel::Club, DriveMode::FreeDrive);
        for state in [
            GameState::Driving,
            GameState::Paused,
            GameState::Finishing,
            GameState::Finished,
        ] {
            s.state = state;
            s.toggle_track();
            assert_eq!(s.track().id, TrackId::Scenic);
        }
        s.state = GameState::Menu;
        for expected in [TrackId::Compact, TrackId::Scenic, TrackId::Compact] {
            s.toggle_track();
            assert_eq!(s.track().id, expected);
            assert_eq!(s.vehicle.model, CarModel::Club);
            assert!(std::ptr::eq(s.vehicle.track, s.rival.track));
            assert!(s.state == GameState::Menu);
        }
        s.select_mode(DriveMode::Race);
        s.toggle_track();
        assert_eq!(s.mode, DriveMode::Race);
        assert_eq!(s.track().id, TrackId::Compact);
        s.select_mode(DriveMode::FreeDrive);
        assert_eq!(
            s.track().id,
            TrackId::Scenic,
            "mode selection restores its default route"
        );
        s.toggle_track();
        s.select_mode(DriveMode::FreeDrive);
        assert_eq!(
            s.track().id,
            TrackId::Scenic,
            "explicit mode selection resets route choice"
        );
    }

    #[test]
    fn car_selection_and_restart_preserve_each_free_route() {
        for id in track::ALL {
            let mut s = Session::on_track(CarModel::Roadster, DriveMode::FreeDrive, id);
            s.state = GameState::Menu;
            for model in cars::ALL {
                s.select_car(model);
                assert_eq!(s.vehicle.model, model);
                assert_eq!(s.track().id, id);
                assert!(s.state == GameState::Menu);
                let road = s.track().at_distance(s.track().length() * 0.5);
                s.vehicle.reset(road.lane_position(0.0), road.heading());
                s.race.elapsed = 42.0;
                s.recoveries = 2;
                s.restart();
                assert_eq!(s.track().id, id);
                assert!(std::ptr::eq(s.vehicle.track, s.rival.track));
                assert_eq!(s.vehicle.model, model);
                assert_eq!(s.vehicle.position, s.track().spawn().lane_position(0.0));
                assert_eq!(s.countdown, 0.0);
                assert_eq!(s.race.elapsed, 0.0);
                assert_eq!(s.recoveries, 0);
                assert!(s.state == GameState::Driving);
                s.state = GameState::Menu;
            }
        }
    }

    #[test]
    fn free_recovery_and_pause_info_use_selected_route() {
        for id in track::ALL {
            let mut s = Session::on_track(CarModel::Muscle, DriveMode::FreeDrive, id);
            let road = s.track().at_distance(s.track().length() * 0.5);
            s.vehicle.reset(road.lane_position(1.0), road.heading());
            assert!(s.recover());
            let recovered = s.track().project(s.vehicle.position);
            assert!((recovered.distance - road.distance).abs() < 0.1);
            assert!(recovered.offset.abs() < 0.01);
            assert_eq!(s.track().id, id);
            assert_eq!(s.race.elapsed, 0.0);
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
        let mut s = Session::on_track(CarModel::Roadster, DriveMode::FreeDrive, TrackId::Engomi);
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
        assert_eq!(s.race.elapsed, 0.0);
        assert!(!s.has_rival());
    }

    #[test]
    fn free_drive_never_runs_a_rival_timer_or_finish_logic() {
        for model in cars::ALL {
            let mut s = Session::on_track(model, DriveMode::FreeDrive, TrackId::Compact);
            assert_eq!(s.countdown, 0.0);
            assert!(!s.has_rival());
            // Even an overlapping inert rival must not collide with the player.
            s.rival.position = s.vehicle.position;
            s.rival.velocity = [10.0, 0.0];
            let rival_position = s.rival.position;
            let mut probe = RaceProgress::new();
            for _ in 0..60 * 180 {
                let previous = s.vehicle.position;
                s.tick(ai::controls(&s.vehicle, None));
                probe.update(previous, s.vehicle.position, TICK);
                if probe.finish_time.is_some() {
                    break;
                }
            }
            assert!(
                probe.finish_time.is_some(),
                "free driving failed to complete route with {model:?}"
            );
            assert!(
                s.state == GameState::Driving,
                "three laps must not end Free Drive"
            );
            assert_eq!(s.rival.position, rival_position);
            assert_eq!(s.rival.velocity, [10.0, 0.0]);
            assert_eq!(s.race.elapsed, 0.0);
            assert_eq!(s.rival_race.elapsed, 0.0);
            assert_eq!(s.race.completed_laps, 0);
            assert!(s.race.lap_times.is_empty() && s.race.finish_time.is_none());
        }
    }

    #[test]
    fn free_recovery_is_nearby_unpenalized_and_preserves_facing_direction() {
        let mut s = Session::on_track(CarModel::Muscle, DriveMode::FreeDrive, TrackId::Compact);
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
        assert_eq!(s.race.elapsed, 0.0);
        assert_eq!(s.recoveries, 1);
    }

    #[test]
    fn opening_menu_location_and_car_choices_survive_start() {
        for location in hud::LOCATIONS {
            for car in cars::ALL {
                let mut s = Session::new(CarModel::Roadster, DriveMode::FreeDrive);
                s.state = GameState::Menu;
                s.select_location(location);
                s.select_car(car);
                assert!(s.state == GameState::Menu);
                s.restart();
                assert!(s.state == GameState::Driving);
                assert_eq!(s.track().id, location);
                assert_eq!(s.vehicle.model, car);
                assert_eq!(s.mode, DriveMode::FreeDrive);
                assert_eq!(s.countdown, 0.0);
                s.select_location(TrackId::Compact);
                s.cycle_car(1);
                assert_eq!(s.track().id, location);
                assert_eq!(s.vehicle.model, car);
            }
        }
    }
    #[test]
    fn menu_activity_keeps_the_location_and_city_selection_uses_free_drive() {
        let mut s = Session::new(CarModel::Club, DriveMode::FreeDrive);
        s.state = GameState::Menu;
        s.toggle_activity();
        assert_eq!(s.mode, DriveMode::FreeDrive);
        s.cycle_location(1);
        assert_eq!(s.track().id, TrackId::Compact);
        s.toggle_activity();
        assert_eq!(s.mode, DriveMode::Race);
        s.toggle_activity();
        assert_eq!(s.mode, DriveMode::FreeDrive);
        assert_eq!(s.track().id, TrackId::Compact);
        s.toggle_activity();
        s.cycle_location(1);
        assert_eq!(s.track().id, TrackId::Engomi);
        assert_eq!(s.mode, DriveMode::FreeDrive);
        assert_eq!(s.vehicle.model, CarModel::Club);
        s.cycle_location(1);
        assert_eq!(s.track().id, TrackId::Scenic);
        s.cycle_location(-1);
        assert_eq!(s.track().id, TrackId::Engomi);
        s.cycle_car(1);
        assert_eq!(s.vehicle.model, CarModel::Roadster);
    }

    #[test]
    fn garage_mode_selection_is_guarded_and_preserves_the_car() {
        let mut s = Session::new(CarModel::Club, DriveMode::FreeDrive);
        s.select_mode(DriveMode::Race);
        assert_eq!(s.mode, DriveMode::FreeDrive);
        s.state = GameState::Menu;
        s.select_mode(DriveMode::Race);
        assert_eq!(s.mode, DriveMode::Race);
        assert_eq!(s.vehicle.model, CarModel::Club);
        assert_eq!(s.countdown, 3.0);
        s.select_car(CarModel::Muscle);
        assert_eq!(s.mode, DriveMode::Race);
        s.select_mode(DriveMode::FreeDrive);
        assert_eq!(s.vehicle.model, CarModel::Muscle);
        assert_eq!(s.countdown, 0.0);
        for state in [
            GameState::Driving,
            GameState::Paused,
            GameState::Finishing,
            GameState::Finished,
        ] {
            s.state = state;
            s.select_mode(DriveMode::Race);
            assert_eq!(s.mode, DriveMode::FreeDrive);
        }
    }

    #[test]
    fn free_drive_pause_info_and_reverse_have_no_race_statistics() {
        let mut s = Session::new(CarModel::Roadster, DriveMode::FreeDrive);
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

    #[test]
    fn pause_freezes_countdown_vehicles_and_timer() {
        let mut s = Session::new(CarModel::Roadster, DriveMode::Race);
        s.state = GameState::Paused;
        let position = s.vehicle.position;
        for _ in 0..120 {
            s.tick(Controls {
                throttle: 1.0,
                ..Controls::default()
            });
        }
        assert_eq!(s.countdown, 3.0);
        assert_eq!(s.race.elapsed, 0.0);
        assert_eq!(s.vehicle.position, position);
    }

    #[test]
    fn feedback_freezes_on_pause_and_recovery_clears_transients() {
        let mut s = Session::new(CarModel::Club, DriveMode::Race);
        s.countdown = 0.0;
        let road = track::at_distance(50.0);
        s.vehicle.reset(road.lane_position(0.0), road.heading());
        s.vehicle.velocity = road.tangent.map(|x| x * 8.0);
        for _ in 0..20 {
            s.tick(Controls {
                handbrake: true,
                ..Controls::default()
            });
        }
        let phase = s.vehicle.wheel_phase;
        let marks = s.skids.vertices();
        let view = particles::View {
            eye: [0.0, 5.0, -100.0],
            right: [1.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            forward: [0.0, 0.0, 1.0],
        };
        let puffs = s.particles.vertices(view);
        assert!(!puffs.is_empty());
        assert!(!marks.is_empty());
        s.state = GameState::Paused;
        for _ in 0..120 {
            s.tick(Controls::default());
        }
        assert_eq!(s.vehicle.wheel_phase, phase);
        assert_eq!(s.particles.vertices(view), puffs);
        let after = s.skids.vertices();
        assert_eq!(marks.len(), after.len());
        assert_eq!(marks[0].color, after[0].color);
        s.state = GameState::Driving;
        s.impact = 9.0;
        s.recover();
        assert_eq!(s.impact, 0.0);
        assert_eq!(s.vehicle.wheel_phase, [0.0; 2]);
        assert_eq!(s.vehicle.tire_scrub, 0.0);
        assert!(s.particles.vertices(view).is_empty());
        s.restart();
        assert!(s.skids.vertices().is_empty());
        assert!(s.particles.vertices(view).is_empty());
        assert_eq!(s.impact, 0.0);
    }

    #[test]
    fn restart_resets_both_cars_and_all_timing() {
        let mut s = Session::new(CarModel::Muscle, DriveMode::Race);
        s.countdown = 0.0;
        for _ in 0..60 {
            s.tick(Controls::default());
        }
        s.restart();
        assert_eq!(s.vehicle.model, CarModel::Muscle);
        assert_eq!(s.countdown, 3.0);
        assert_eq!(s.race.elapsed, 0.0);
        assert_eq!(s.rival_race.elapsed, 0.0);
        assert_eq!(s.rival.speed, 0.0);
    }

    #[test]
    fn recovery_penalizes_without_advancing_checkpoints() {
        let mut s = Session::new(CarModel::Roadster, DriveMode::Race);
        s.countdown = 0.0;
        s.race.next_gate = 2;
        s.recover();
        assert_eq!(s.race.next_gate, 2);
        assert_eq!(s.race.elapsed, 5.0);
        assert_eq!(s.recoveries, 1);
    }

    #[test]
    fn recovery_reports_when_camera_and_audio_should_snap() {
        let mut s = Session::new(CarModel::Roadster, DriveMode::Race);
        assert!(!s.recover()); // countdown
        s.countdown = 0.0;
        for state in [
            GameState::Menu,
            GameState::Paused,
            GameState::Finishing,
            GameState::Finished,
        ] {
            s.state = state;
            assert!(!s.recover());
        }
        s.state = GameState::Driving;
        assert!(s.recover());
        assert_eq!(s.recoveries, 1);
    }

    #[test]
    fn car_selection_is_menu_only_and_identity_survives_recovery_and_restart() {
        let mut s = Session::new(CarModel::Roadster, DriveMode::Race);
        s.select_car(CarModel::Club);
        assert_eq!(s.vehicle.model, CarModel::Roadster);
        s.state = GameState::Menu;
        for model in cars::ALL {
            s.select_car(model);
            assert_eq!(s.vehicle.model, model);
            assert_eq!(s.rival.model, CarModel::Roadster);
            assert!(s.state == GameState::Menu);
        }
        s.state = GameState::Driving;
        s.countdown = 0.0;
        s.recover();
        assert_eq!(s.vehicle.model, CarModel::Club);
        s.restart();
        assert_eq!(s.vehicle.model, CarModel::Club);
        assert_eq!(s.countdown, 3.0);
        for state in [
            GameState::Driving,
            GameState::Paused,
            GameState::Finishing,
            GameState::Finished,
        ] {
            s.state = state;
            s.select_car(CarModel::Muscle);
            assert_eq!(s.vehicle.model, CarModel::Club);
        }
    }

    #[test]
    fn complete_two_car_races_and_results_are_repeatable() {
        for profile in cars::ALL {
            let run = || {
                let mut s = Session::new(profile, DriveMode::Race);
                for _ in 0..60 * 180 {
                    let controls = ai::controls(&s.vehicle, Some(&s.rival));
                    s.tick(controls);
                    if s.state == GameState::Finished {
                        break;
                    }
                }
                assert!(
                    s.state == GameState::Finished,
                    "two-car race did not finish for profile {profile:?}"
                );
                assert_eq!(s.race.lap_times.len(), race::LAPS as usize);
                let time = s.race.elapsed;
                let position = s.vehicle.position;
                for _ in 0..120 {
                    s.tick(Controls::default());
                }
                assert_eq!(s.race.elapsed, time);
                assert_eq!(s.vehicle.position, position);
                (time, s.player_leads())
            };
            assert_eq!(run(), run(), "fixed-step race must be deterministic");
        }
    }

    #[test]
    fn penalty_adjusted_results_wait_for_the_rival() {
        let mut s = Session::new(CarModel::Roadster, DriveMode::Race);
        s.countdown = 0.0;
        s.state = GameState::Finishing;
        s.race.completed_laps = race::LAPS;
        s.race.elapsed = 105.0;
        s.race.finish_time = Some(105.0);
        s.rival_race.elapsed = 100.0;
        s.rival_race.completed_laps = race::LAPS - 1;
        s.rival_race.next_gate = race::GATES - 1;
        let road = track::at_distance(track::length() - 12.0);
        s.rival.reset(road.lane_position(0.0), road.heading());
        s.rival.velocity = [road.tangent[0] * 10.0, road.tangent[1] * 10.0];
        s.rival.refresh_speed();
        for _ in 0..600 {
            s.tick(Controls::default());
            if s.state == GameState::Finished {
                break;
            }
        }
        assert!(s.state == GameState::Finished);
        assert!(s.rival_race.finish_time.unwrap() < 105.0);
        assert!(
            !s.player_leads(),
            "crossing first must not bypass a time penalty"
        );
        assert_eq!(s.race.elapsed, 105.0);
    }
}
