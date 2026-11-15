mod add_location;
mod add_location_compile;
mod add_location_download;
mod add_location_trees;
mod aerial;
mod asset_car;
mod authoring;
mod barrier_scene;
mod build_barriers;
mod build_buildings;
mod build_collision;
mod build_crossings;
mod build_details;
mod build_furniture;
mod build_landscape;
mod build_lane_details;
mod build_scene;
mod build_surfaces;
mod build_terrain;
mod build_transport;
mod building_assets;
mod camera;
mod car_glb;
mod chunks;
mod city;
mod city_assets;
mod city_scene;
mod codec_barriers;
mod codec_city;
mod codec_furniture;
mod codec_landscape;
mod codec_number;
mod codec_road_rises;
mod create_track;
mod detail_scene;
mod download;
mod elevation;
mod engine_sample;
mod error;
mod feedback;
mod file_io;
mod font;
mod geometry;
mod gl;
mod hgt;
mod hud;
mod hud_atlas;
mod imagery;
mod landscape;
mod loading;
mod location;
mod map_details;
mod mesh_glb;
mod osm;
mod particles;
mod phys_car;
mod phys_powertrain;
mod phys_tire;
mod prefetch;
mod road_furniture;
mod roof_scene;
mod scene;
mod scene_archive;
mod settings;
mod shadows;
mod sound_car;
mod source_location;
mod sources;
mod src_files;
mod street_surface;
mod street_surface_binary;
#[cfg(test)]
mod test_support;
mod track;
mod traffic;
mod transport_scene;
mod tree_obstacles;
mod tree_preview;
mod vehicle_location;
mod world;
mod world_barriers;
mod world_buildings;
mod world_mapped_objects;
mod world_preparation;
mod world_raised_roads;
mod world_roads;
mod world_spatial;
mod world_transport;

use error::invalid;
use log::{info, warn};
use phys_car::{Controls, Vehicle};
use sdl3::{
    event::{Event, WindowEvent},
    keyboard::Keycode,
    video::Window,
};
use settings::Settings;
use std::{
    collections::{HashMap, HashSet},
    path::PathBuf,
    time::{Duration, Instant},
};
use track::{LocationId, WorldLocation};

const HELP: &str = "Arguments: <command>
Commands:

    play
    add-location <OpenStreetMap URL> <width-km> <height-km>
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
    car: Vehicle,
    state: GameState,
    recoveries: u32,
    skids: feedback::SkidMarks,
    particles: particles::TireParticles,
    impact: f32,
    signal_time: f32,
}

#[cfg(test)]
fn on_track(model: std::sync::Arc<asset_car::CarSpec>, id: LocationId) -> Session {
    on_world(model, track::try_track(&id).unwrap())
}

fn on_world(
    model: std::sync::Arc<asset_car::CarSpec>,
    track: std::sync::Arc<WorldLocation>,
) -> Session {
    let start = vehicle_location::spawn(&track);
    Session {
        car: {
            let mut vehicle = phys_car::on_track(
                vehicle_location::vehicle_position(start, 0.0),
                world::heading(start),
                model,
                track,
            );
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

fn session_track(session: &Session) -> &WorldLocation {
    &session.car.track
}

fn session_restart(session: &mut Session) {
    *session = on_world(session.car.spec.clone(), session.car.track.clone());
}

fn session_select_car(session: &mut Session, model: std::sync::Arc<asset_car::CarSpec>) {
    if session.state != GameState::Menu {
        return;
    }
    *session = on_world(model, session.car.track.clone());
    session.state = GameState::Menu;
}

fn update_feedback(session: &mut Session) {
    feedback::advance(&mut session.skids, TICK);
    feedback::record(&mut session.skids, 0, &session.car);
    particles::advance(&mut session.particles, TICK);
    particles::record(&mut session.particles, 0, &session.car, TICK);
}

fn tick(session: &mut Session, controls: Controls) {
    if session.state != GameState::Driving {
        return;
    }
    session.signal_time =
        (session.signal_time + TICK.as_secs_f32()).rem_euclid(traffic::CYCLE_SECONDS);
    session.impact = session
        .impact
        .max(phys_car::update(&mut session.car, TICK, controls));
    update_feedback(session);
}

fn recover(session: &mut Session) -> bool {
    if session.state != GameState::Driving {
        return false;
    }
    let road = vehicle_location::recovery(session_track(session), session.car.position);
    let backwards = (session.car.heading - world::heading(road)).cos() < 0.0;
    let heading = world::heading(road) + if backwards { std::f32::consts::PI } else { 0.0 };
    phys_car::reset(
        &mut session.car,
        vehicle_location::vehicle_position(road, 0.0),
        heading,
    );
    feedback::break_trail(&mut session.skids, 0);
    particles::reset_car(&mut session.particles, 0);
    session.impact = 0.0;
    session.recoveries += 1;
    true
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
    settings: Settings,
    preview_angle: f32,
    menu_focus: usize,
    locations: Vec<city_assets::MenuLocation>,
    selected_location: usize,
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

#[cfg(test)]
fn new() -> App {
    with_location(false)
}

fn with_location(city: bool) -> App {
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
            log::warn!("Multisampled window unavailable: {error}; using the default framebuffer");
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
        .then(|| track::asset("data/locations/nicosia").ok())
        .flatten();
    let default_location = locations
        .iter()
        .position(|entry| {
            track::asset_path(&entry.track)
                .is_some_and(|p| p.file_name().is_some_and(|name| name == "scenic"))
        })
        .unwrap_or(0);
    let selected_location = selected_city.map_or(default_location, |id| {
        if let Some(index) = locations.iter().position(|entry| entry.track == id) {
            index
        } else {
            locations.push(
                city_assets::from_assets(city_assets::new(
                    track::asset_path(&id).unwrap().to_path_buf(),
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
        .filter(|entry| track::asset_path(&entry.track).is_none())
        .map_or(LocationId::Compact, |entry| entry.track.clone());
    let mut session = on_world(
        cars[0].clone(),
        track::try_track(&preview).expect("menu preview world"),
    );
    session.state = GameState::Menu;
    let _ = window.set_title(&session_track(&session).display_name);

    let program = gl::create_program(gl::VS_SRC, gl::FS_SRC).expect("failed to create GL program");
    let scene_program =
        gl::create_program(gl::SCENE_VS, gl::SCENE_FS).expect("failed to create scene program");
    let skid_renderer = gl::skid_renderer_new();
    let particle_renderer =
        gl::particle_renderer_new().expect("failed to create particle renderer");
    let hud = hud::hud_new();
    let audio = match sound_car::new(&sdl_context) {
        Ok(audio) => Some(audio),
        Err(error) => {
            warn!("Audio unavailable; continuing silently: {error}");
            None
        }
    };
    let settings = if cfg!(test) {
        Settings::default()
    } else {
        settings::load()
    };
    let mut chase = camera::new(&session.car);
    if let Some(view) = settings::camera(&settings) {
        camera::set_view(&mut chase, view);
        camera::snap(&mut chase, &session.car);
    }
    let mut app = App {
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
        settings,
        preview_angle: 0.0,
        menu_focus: 0,
        locations,
        selected_location,
        keys: HashSet::new(),
    };
    ensure_mesh(&mut app);
    app
}

fn start_selected(app: &mut App) {
    let id = app.locations[app.selected_location].track.clone();
    let Some(world) = loading::prepare(
        id,
        app.locations_cache.clone(),
        &mut app.window,
        &mut app.events,
        &app.sdl_context,
        &app.hud,
        app.program,
    ) else {
        app.keys.clear();
        return;
    };
    app.keys.clear();
    if let Err(error) = install_location(app, world) {
        warn!("Cannot prepare location geometry: {error}");
    }
}

fn install_location(app: &mut App, world: std::sync::Arc<WorldLocation>) -> Result<(), String> {
    // Keep the active session and all its GL resources until candidate creation succeeds.
    let candidate_mesh = if app.meshes.contains_key(&world.id) {
        None
    } else {
        Some(gl::mesh_new(&world, &app.cars)?)
    };
    if let Some(mesh) = candidate_mesh {
        app.meshes.insert(world.id.clone(), mesh);
    }
    // Bound retained city GPU data while keeping authored preview meshes reusable.
    app.meshes
        .retain(|key, _| track::asset_path(key).is_none() || key == &world.id);
    track::install(&mut app.locations_cache, world.clone());
    app.session = on_world(app.session.car.spec.clone(), world);
    app_restart(app);
    Ok(())
}

fn ensure_mesh(app: &mut App) {
    let track = session_track(&app.session);
    if !app.meshes.contains_key(&track.id) {
        if street_surface::has_compiled_roads(track) {
            loading::draw(&app.window, &app.hud, app.program, &track.display_name, 0.0);
            gl::swap_window(&app.window);
        }
        app.meshes.insert(
            track.id.clone(),
            gl::mesh_new(track, &app.cars).expect("failed to create route mesh"),
        );
    }
}

fn mesh(app: &App) -> &gl::Mesh {
    app.meshes
        .get(&session_track(&app.session).id)
        .expect("selected route mesh must be cached")
}

fn app_restart(app: &mut App) {
    session_restart(&mut app.session);
    if let Some(audio) = &mut app.audio {
        sound_car::reset_feedback(audio);
    }
    app.keys.clear();
    snap_camera(app);
}

fn snap_camera(app: &mut App) {
    camera::snap(&mut app.chase, &app.session.car);
}

fn run(app: &mut App) -> Result<(), Box<dyn std::error::Error>> {
    let mut last_frame = Instant::now();
    let mut accumulator = Duration::ZERO;
    'running: loop {
        while let Some(event) = app.events.poll_event() {
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
                } => key_down(app, key),
                Event::MouseButtonDown {
                    mouse_btn: sdl3::mouse::MouseButton::Left,
                    x,
                    y,
                    ..
                } => menu_click(app, x, y),
                Event::MouseWheel { y, direction, .. } => mouse_wheel(app, y, direction),
                Event::KeyUp {
                    keycode: Some(key), ..
                } => {
                    app.keys.remove(&key);
                }
                Event::Window {
                    win_event: WindowEvent::FocusLost,
                    ..
                } => {
                    app.keys.clear();
                }
                _ => {}
            }
        }
        let now = Instant::now();
        let frame_time = (now - last_frame).min(Duration::from_millis(250));
        last_frame = now;
        accumulator += frame_time;
        while accumulator >= TICK {
            let held = |keys: &[Keycode]| keys.iter().any(|k| app.keys.contains(k));
            let controls = Controls {
                throttle: held(&[Keycode::Up]) as u8 as f32,
                brake: held(&[Keycode::Down]) as u8 as f32,
                steering: held(&[Keycode::Right]) as u8 as f32
                    - held(&[Keycode::Left]) as u8 as f32,
                handbrake: held(&[Keycode::Space]),
            };
            tick(&mut app.session, controls);
            accumulator -= TICK;
        }
        let impact = std::mem::take(&mut app.session.impact);
        if let Some(audio) = &mut app.audio {
            let car = &app.session.car;
            if let Err(error) = sound_car::update(
                audio,
                car,
                camera::pose(&app.chase),
                app.keys.contains(&Keycode::Up),
                matches!(app.session.state, GameState::Driving),
                impact,
            ) {
                warn!("Audio disabled: {error}");
                app.audio = None;
            }
        }
        render(app, frame_time);
    }
    Ok(())
}

fn app_select_car(app: &mut App, index: usize) {
    if app.session.state == GameState::Menu {
        if let Some(spec) = app.cars.get(index) {
            app.selected_car = index;
            session_select_car(&mut app.session, spec.clone());
        }
    }
}

fn cycle_car(app: &mut App, direction: i32) {
    let next = (app.selected_car as i32 + direction).rem_euclid(app.cars.len() as i32) as usize;
    app_select_car(app, next);
}

fn menu_action(app: &mut App, action: hud::MenuAction) {
    if app.session.state != GameState::Menu {
        return;
    }
    match action {
        hud::MenuAction::Choice(index) => {
            if index < app.locations.len() {
                app.selected_location = index;
            }
            app.menu_focus = 0;
        }
        hud::MenuAction::Car(index) => {
            app.menu_focus = 1;
            app_select_car(app, index);
        }
        hud::MenuAction::Start => {
            start_selected(app);
            return;
        }
    }
    ensure_mesh(app);
    snap_camera(app);
    app.keys.clear();
}
fn menu_click(app: &mut App, x: f32, y: f32) {
    let (width, height) = app.window.size();
    if width == 0 || height == 0 {
        return;
    }
    let point = [x / width as f32 * 2.0 - 1.0, 1.0 - y / height as f32 * 2.0];
    if app.session.state == GameState::Paused {
        if let Some(action) = hud::pause_hit_test(point[0], point[1]) {
            key_down(
                app,
                match action {
                    hud::PauseAction::Resume => Keycode::Escape,
                    hud::PauseAction::Restart => Keycode::R,
                    hud::PauseAction::Selection => Keycode::L,
                    hud::PauseAction::Audio => Keycode::V,
                    hud::PauseAction::Recover => Keycode::H,
                },
            );
        }
    } else if app.session.state == GameState::Menu {
        if let Some(action) = hud::selection_menu_hit_test(
            point[0],
            point[1],
            app.selected_location,
            app.locations.len(),
            app.selected_car,
            app.cars.len(),
        ) {
            menu_action(app, action);
        }
    }
}
fn key_down(app: &mut App, key: Keycode) {
    if app.session.state == GameState::FullMap {
        match key {
            Keycode::Escape | Keycode::M => app.session.state = GameState::Driving,
            Keycode::C => copy_map_location(app),
            Keycode::Plus | Keycode::Equals | Keycode::KpPlus => zoom_map(app, 1.0),
            Keycode::Minus | Keycode::KpMinus => zoom_map(app, -1.0),
            _ => {}
        }
        app.keys.clear();
        return;
    }
    app.keys.insert(key);
    if app.session.state == GameState::Menu {
        match key {
            Keycode::Left => app.menu_focus = 0,
            Keycode::Right => app.menu_focus = 1,
            Keycode::Tab => app.menu_focus = (app.menu_focus + 1) % 3,
            Keycode::Up | Keycode::Down | Keycode::Space => {
                let direction = if key == Keycode::Up { -1 } else { 1 };
                match app.menu_focus {
                    0 => {
                        app.selected_location = (app.selected_location as i32 + direction)
                            .rem_euclid(app.locations.len() as i32)
                            as usize;
                    }
                    1 => cycle_car(app, direction),
                    _ if key == Keycode::Space => {
                        start_selected(app);
                        return;
                    }
                    _ => {}
                }
                ensure_mesh(app);
                snap_camera(app);
            }
            _ => {
                handle_key(app, key);
                return;
            }
        }
        app.keys.clear();
        return;
    }
    handle_key(app, key);
}

fn mouse_wheel(app: &mut App, y: f32, direction: sdl3::mouse::MouseWheelDirection) {
    zoom_map(
        app,
        if direction == sdl3::mouse::MouseWheelDirection::Flipped {
            -y
        } else {
            y
        },
    );
}

fn copy_map_location(app: &mut App) {
    let url = hud::google_maps_url(session_track(&app.session), app.session.car.position);
    let status = if let Some(url) = url {
        match app
            .sdl_context
            .video()
            .and_then(|video| video.clipboard().set_clipboard_text(&url))
        {
            Ok(()) => "Copied Google Maps URL",
            Err(error) => {
                log::warn!("Cannot copy map location: {error}");
                "Copy failed - press C to retry"
            }
        }
    } else {
        "No Google Maps location"
    };
    hud::map_copy_status(
        &mut app.hud,
        hud::Map {
            track: session_track(&app.session),
            player: app.session.car.position,
        },
        app.session.car.heading,
        [WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32],
        status,
    );
}

fn zoom_map(app: &mut App, steps: f32) {
    if app.session.state != GameState::FullMap {
        return;
    }
    if let Some(zoom) = hud::hud_zoom_full_map(
        &mut app.hud,
        hud::Map {
            track: session_track(&app.session),
            player: app.session.car.position,
        },
        app.session.car.heading,
        [WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32],
        steps,
    ) {
        settings::set_map_zoom(&mut app.settings, zoom);
    }
}

fn handle_key(app: &mut App, key: Keycode) {
    match key {
        Keycode::Escape => {
            app.session.state = match app.session.state {
                GameState::Driving => GameState::Paused,
                GameState::Paused => GameState::Driving,
                other => other,
            };
            app.keys.clear();
        }
        Keycode::Return if app.session.state == GameState::Menu => start_selected(app),
        Keycode::M if matches!(app.session.state, GameState::Driving | GameState::Paused) => {
            hud::prepare_full_map(
                &mut app.hud,
                hud::Map {
                    track: session_track(&app.session),
                    player: app.session.car.position,
                },
                app.session.car.heading,
                [WINDOW_WIDTH as f32, WINDOW_HEIGHT as f32],
                settings::map_zoom(&app.settings),
            );
            app.session.state = GameState::FullMap;
            app.keys.clear();
        }
        Keycode::L => {
            app.menu_focus = 0;
            app.session.state = GameState::Menu;
            app.keys.clear();
        }
        Keycode::R if app.session.state == GameState::Menu => start_selected(app),
        Keycode::R => app_restart(app),
        Keycode::H => {
            let paused = app.session.state == GameState::Paused;
            if paused {
                app.session.state = GameState::Driving;
            }
            let recovered = recover(&mut app.session);
            if paused {
                app.session.state = GameState::Paused;
            }
            if recovered {
                if let Some(audio) = &mut app.audio {
                    sound_car::reset_feedback(audio);
                }
                snap_camera(app);
            }
        }
        Keycode::F1 | Keycode::F2 | Keycode::F3 if app.session.state == GameState::Menu => {
            let model = match key {
                Keycode::F1 => 0,
                Keycode::F2 => 1,
                _ => 2,
            };
            app_select_car(app, model);
            snap_camera(app);
        }
        Keycode::F6 if app.session.state == GameState::Menu => {
            app.selected_location = if app.selected_location == 0 { 1 } else { 0 };
        }
        Keycode::F7 if app.session.state == GameState::Menu => {
            if let Some(index) = app.locations.iter().position(|entry| {
                track::asset_path(&entry.track).is_some_and(|path| path.ends_with("nicosia"))
            }) {
                app.selected_location = index;
            }
        }
        Keycode::A | Keycode::Z if app.session.state == GameState::Driving => {
            phys_car::shift(&mut app.session.car, key == Keycode::A);
        }
        Keycode::V => {
            if let Some(audio) = &mut app.audio {
                audio.muted = !audio.muted;
            }
        }
        Keycode::C | Keycode::Tab
            if matches!(app.session.state, GameState::Driving | GameState::Paused) =>
        {
            camera::toggle_view(&mut app.chase);
            settings::set_camera(&mut app.settings, camera::view(&app.chase));
            if app.session.state == GameState::Paused {
                snap_camera(app);
            }
        }
        _ => {}
    }
}

fn render(app: &mut App, frame_time: Duration) {
    gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
    gl::clear_screen();
    if app.session.state == GameState::FullMap {
        hud::draw_full_map(&app.hud, app.program);
    } else {
        render_scene(app, frame_time);
    }
    gl::swap_window(&app.window);
    let _ = app.window.set_title(&format!(
        "{} | {} | FREE DRIVE",
        session_track(&(app.session)).display_name,
        app.session.car.spec.name,
    ));
}

fn render_scene(app: &mut App, frame_time: Duration) {
    gl::set_signal_time(app.scene_program, app.session.signal_time);
    let car = &app.session.car;
    if matches!(app.session.state, GameState::Driving) {
        camera::update(&mut app.chase, car, frame_time);
    }
    let aspect = WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32;
    let in_menu = app.session.state == GameState::Menu;
    let camera = if in_menu {
        app.preview_angle =
            (app.preview_angle + frame_time.as_secs_f32() * 0.22).rem_euclid(std::f32::consts::TAU);
        gl::garage_camera(
            aspect,
            car.position,
            car.heading + app.preview_angle,
            street_surface::has_compiled_roads(session_track(&app.session))
                .then_some(session_track(&app.session)),
        )
    } else if camera::view(&app.chase) == camera::FollowView::Bonnet {
        gl::bonnet_camera(aspect, camera::pose(&app.chase), car)
    } else {
        gl::chase_camera(aspect, camera::pose(&app.chase))
    };
    let vehicles = [(&app.session.car, car.spec.paint_color)];
    let contacts = vehicles.map(|(car, _)| shadows::new(car));
    gl::draw_ground(mesh(app), app.scene_program, &camera, &contacts);
    if !in_menu {
        gl::skid_renderer_draw(
            &app.skid_renderer,
            app.scene_program,
            &camera,
            &app.session.skids,
        );
    }
    for &(vehicle, tint) in &vehicles {
        let model = gl::vehicle_body_matrix(vehicle);
        gl::mesh_draw(
            mesh(app),
            app.scene_program,
            &camera,
            &model,
            tint,
            vehicle,
            app.selected_car,
        );
    }
    // Transparent puffs follow all opaque world/car draws and precede the HUD.
    if !in_menu {
        gl::particle_renderer_draw(&app.particle_renderer, &camera, &app.session.particles);
    }
    let s = &app.session;
    let audio_status = match &app.audio {
        Some(a) if a.muted => "Muted",
        Some(_) => "On",
        None => "Unavailable",
    };
    if in_menu {
        hud::draw_menu(
            &app.hud,
            app.program,
            hud::MainMenu {
                locations: &app.locations,
                selected: app.selected_location,
                car: app.selected_car,
                cars: &app.cars,
                focus: app.menu_focus,
            },
        );
    } else {
        hud::draw(
            &app.hud,
            app.program,
            hud::Map {
                track: session_track(s),
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
                city: street_surface::has_compiled_roads(session_track(s)),
                aerial_attribution: world::assets(session_track(s))
                    .and_then(|a| city_assets::aerial_attribution(a)),
                audio: audio_status,
            }),
        );
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
                let usage =
                    "add-location requires an OpenStreetMap URL, width in km, and height in km";
                let url = args
                    .next()
                    .ok_or_else(|| invalid(usage))?
                    .into_string()
                    .map_err(|_| invalid("expected a UTF-8 URL"))?;
                let width = args
                    .next()
                    .ok_or_else(|| invalid(usage))?
                    .into_string()
                    .map_err(|_| invalid("expected a numeric width"))?
                    .parse::<f64>()?;
                let height = args
                    .next()
                    .ok_or_else(|| invalid(usage))?
                    .into_string()
                    .map_err(|_| invalid("expected a numeric height"))?
                    .parse::<f64>()?;
                if args.next().is_some() {
                    return Err(invalid(usage));
                }
                let area = osm::from_url(&url, width, height)?;
                add_location::add_location(area)?;
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
    run(&mut with_location(false))
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
