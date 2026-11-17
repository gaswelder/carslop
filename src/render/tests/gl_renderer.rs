use crate::ensure_mesh;
use crate::install_location;
use crate::key_down;
use crate::menu_action;
use crate::mesh;
use crate::model;
use crate::new;
use crate::on_track;
use crate::on_world;
use crate::physics;
use crate::recover;
use crate::render;
use crate::session_restart;
use crate::session_select_car;
use crate::session_track;
use crate::snap_camera;
use crate::storage;
use crate::test;
use crate::tick;
use crate::ui;
use crate::util;
use crate::with_location;
use crate::world;

fn assert_particle_pixel_readback(app: &App) {
    let car = &app.session.car;
    let camera = render::gl::chase_camera(
        WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
        render::camera::Pose {
            eye: util::geometry::XYZ {
                x: car.position.x + 4.0,
                y: car.position.y + 6.0,
                z: car.position.z + 5.0,
            },
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
    render::gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
    render::gl::clear_screen();
    render::gl::draw_ground(
        mesh(app),
        app.scene_program,
        &camera,
        &[render::shadows::new(car)],
    );
    let before = read_color();
    let depth = read_depth();
    let previous = state();
    let count =
        render::gl::particle_renderer_draw(&app.particle_renderer, &camera, &app.session.particles);
    assert!(count > 0);
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
        car.spec
    );
    assert!(
        changed < (WINDOW_WIDTH * WINDOW_HEIGHT / 6) as usize,
        "puffs must remain local"
    );

    // Put an opaque depth plane in front of every puff: no color may leak through.
    render::gl::clear_screen();
    render::gl::draw_ground(mesh(app), app.scene_program, &camera, &[]);
    let occluded = read_color();
    unsafe {
        ::gl::ClearDepth(0.0);
        ::gl::Clear(::gl::DEPTH_BUFFER_BIT);
        ::gl::ClearDepth(1.0);
    }
    render::gl::particle_renderer_draw(&app.particle_renderer, &camera, &app.session.particles);
    assert_eq!(
        read_color(),
        occluded,
        "particles must respect opaque depth"
    );
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    render::gl::clear_screen();
}

use crate::*;

#[test]
#[ignore = "requires SDL/OpenGL and CARSLOP_PREVIEW_LOCATION; exports a fixed-camera map preview"]
fn location_renderer_preview() {
    let path = std::env::var("CARSLOP_PREVIEW_LOCATION").expect("location directory");
    let pose: [[f32; 3]; 2] = serde_json::from_str(
        &std::env::var("CARSLOP_PREVIEW_POSE").expect("JSON [eye, target] coordinates"),
    )
    .unwrap();
    let mut app = new();
    let id = world::locations::asset(&path).unwrap();
    install_location(&mut app, world::locations::try_track(&id).unwrap()).unwrap();
    let camera = render::gl::chase_camera(
        WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
        render::camera::Pose {
            eye: pose[0].into(),
            target: pose[1].into(),
        },
    );
    render::gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
    for _ in 0..12 {
        render::gl::clear_screen();
        render::gl::draw_ground(mesh(&app), app.scene_program, &camera, &[]);
    }
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    capture_frame(&std::env::temp_dir().join("carslop-location-preview.ppm"));
}

#[test]
#[ignore = "requires SDL/OpenGL; profiles a warm HUD including GPU completion"]
fn hud_renderer_profile() {
    let app = with_location(false);
    render::gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
    let draw = |frame: u32| {
        render::gl::clear_screen();
        ui::hud::draw(
            &app.hud,
            app.program,
            ui::hud::Map {
                track: session_track(&app.session),
                player: app.session.car.position,
            },
            0.0,
            ui::hud::Gauges {
                rpm: 3000.0 + frame as f32,
                redline_rpm: 6500.0,
                speed_kmh: 90.0,
                gear: 3,
                forces: physics::car::force_telemetry(&app.session.car),
            },
            None,
        );
        unsafe {
            ::gl::Finish();
        }
    };
    for frame in 0..10 {
        draw(frame);
    }
    let mut samples = Vec::new();
    for frame in 0..120 {
        let started = std::time::Instant::now();
        draw(frame);
        samples.push(started.elapsed().as_secs_f64() * 1000.0);
    }
    samples.sort_by(f64::total_cmp);
    eprintln!(
        "HUD CPU + GPU: median {:.3} ms, p95 {:.3} ms, max {:.3} ms",
        samples[60], samples[114], samples[119]
    );
    capture_frame(&std::env::temp_dir().join("carslop-hud-atlas.ppm"));
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
}

#[test]
#[ignore = "requires SDL3/OpenGL; compares rear lamp pixels with the pedal up/down"]
fn stop_lights_renderer() {
    let app = new();
    for (car_index, model) in app.cars.clone().into_iter().enumerate() {
        let mut car = test::support::vehicle(app.session.car.position, 0.0, model.clone());
        let p = car.position;
        let camera = render::gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            render::camera::Pose {
                eye: util::geometry::XYZ {
                    x: p.x,
                    y: p.y + 1.2,
                    z: p.z - 5.0,
                },
                target: util::geometry::XYZ {
                    x: p.x,
                    y: p.y + 0.25,
                    z: p.z,
                },
            },
        );
        let matrix = render::gl::model_matrix(p, 0.0, 0.0);
        let mut frames = Vec::new();
        for pressed in [false, true, false] {
            car.brake_lights = pressed;
            render::gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
            render::gl::clear_screen();
            render::gl::mesh_draw(
                mesh(&app),
                app.scene_program,
                &camera,
                &matrix,
                [0.1, 0.3, 0.8],
                &car,
                car_index,
            );
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

#[test]
#[ignore = "requires SDL/OpenGL; compares car detail near and far from the map origin"]
fn car_details_stay_stable_far_from_the_map_origin() {
    let app = with_location(false);
    let track = std::sync::Arc::new(test::support::flat_track(
        &world::locations::try_track(&LocationId::Compact).unwrap(),
    ));
    let mut frames = Vec::new();
    for (label, position) in [
        ("near", [0.0, 100.0, 0.0]),
        ("far", [12000.0, 100.0, 9000.0]),
    ] {
        let car =
            physics::car::on_track((position).into(), 0.0, app.cars[0].clone(), track.clone());
        let camera = render::gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            render::camera::Pose {
                eye: util::geometry::XYZ {
                    x: position[0] + 4.0,
                    y: position[1] + 2.0,
                    z: position[2] - 6.0,
                },
                target: util::geometry::XYZ {
                    x: position[0],
                    y: position[1] + 0.25,
                    z: position[2],
                },
            },
        );
        render::gl::clear_screen();
        render::gl::mesh_draw(
            mesh(&app),
            app.scene_program,
            &camera,
            &render::gl::vehicle_body_matrix(&car),
            [1.0, 0.18, 0.08],
            &car,
            0,
        );
        let path = std::env::temp_dir().join(format!("carslop-car-precision-{label}.ppm"));
        capture_frame(&path);
        let bytes = std::fs::read(path).unwrap();
        let header = format!("P6\n{} {}\n255\n", WINDOW_WIDTH, WINDOW_HEIGHT).len();
        frames.push(bytes[header..].to_vec());
    }
    let changed = frames[0]
        .chunks_exact(3)
        .zip(frames[1].chunks_exact(3))
        .filter(|(a, b)| (0..3).any(|i| a[i].abs_diff(b[i]) > 20))
        .count();
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    assert!(
        changed < 150,
        "car details flicker after a world translation: {changed} pixels"
    );
}

#[test]
#[ignore = "requires SDL3/OpenGL; verifies reverse lamps in reverse, drive and neutral"]
fn reverse_lights_renderer() {
    let app = new();
    for (car_index, model) in app.cars.clone().into_iter().enumerate() {
        let mut car = test::support::vehicle(app.session.car.position, 0.0, model.clone());
        let p = car.position;
        let camera = render::gl::chase_camera(
            WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
            render::camera::Pose {
                eye: util::geometry::XYZ {
                    x: p.x,
                    y: p.y + 1.2,
                    z: p.z - 5.0,
                },
                target: util::geometry::XYZ {
                    x: p.x,
                    y: p.y + 0.25,
                    z: p.z,
                },
            },
        );
        let matrix = render::gl::model_matrix(p, 0.0, 0.0);
        let mut frames = Vec::new();
        for gear in [1, physics::car::REVERSE_GEAR, physics::car::NEUTRAL_GEAR] {
            car.gear = gear;
            render::gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
            render::gl::clear_screen();
            render::gl::mesh_draw(
                mesh(&app),
                app.scene_program,
                &camera,
                &matrix,
                [0.1, 0.3, 0.8],
                &car,
                car_index,
            );
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
            .filter(|(off, on)| {
                on[0] > off[0].saturating_add(20) && on[0] > 220 && on[1] > 220 && on[2] > 200
            })
            .count();
        assert!(
            brightened > 30,
            "reverse lamps did not brighten: {model:?}, {brightened} pixels"
        );
        assert_eq!(
            frames[0], frames[2],
            "neutral must extinguish reverse lamps"
        );
    }
}

/// Export repeatable front/rear/side views for reviewing silhouette and shading.

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
    let car = &app.session.car;
    let camera = render::gl::chase_camera(
        WINDOW_WIDTH as f32 / WINDOW_HEIGHT as f32,
        render::camera::Pose {
            eye: util::geometry::XYZ {
                x: car.position.x + 4.0,
                y: car.position.y + 9.0,
                z: car.position.z + 5.0,
            },
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
    render::gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
    render::gl::clear_screen();
    render::gl::draw_ground(mesh(app), app.scene_program, &camera, &[]);
    render::gl::skid_renderer_draw(
        &app.skid_renderer,
        app.scene_program,
        &camera,
        &app.session.skids,
    );
    assert_eq!(contact_count(app.scene_program), 0);
    let unshadowed = read();
    render::gl::clear_screen();
    render::gl::draw_ground(
        mesh(app),
        app.scene_program,
        &camera,
        &[render::shadows::new(car)],
    );
    render::gl::skid_renderer_draw(
        &app.skid_renderer,
        app.scene_program,
        &camera,
        &app.session.skids,
    );
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
        car.spec
    );
    assert!(
        changed < (WINDOW_WIDTH * WINDOW_HEIGHT / 8) as usize,
        "contact shading must remain local"
    );
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
}

#[test]
#[ignore = "requires SDL/OpenGL"]
fn feedback_renderer_smoke() {
    let mut app = with_location(false);
    // Scenic fixtures are sampled from the route, never compact world coordinates.
    for (car_index, model) in app.cars.clone().into_iter().enumerate() {
        app.selected_car = car_index;
        app.session = on_track(model.clone(), test::support::authored_id());
        ensure_mesh(&mut app);
        for distance in [50.0, 50.6] {
            let road = world::locations::at_distance(session_track(&app.session), distance);
            physics::car::reset(
                &mut app.session.car,
                physics::vehicle_location::vehicle_position(road, 0.0),
                model::world::heading(road),
            );
            app.session.car.tire_scrub = 0.9;
            render::skid_marks::record(&mut app.session.skids, 0, &app.session.car);
        }
        assert!(!render::skid_marks::vertices(&(app.session.skids)).is_empty());
        let v = &mut app.session.car;
        v.velocity = util::geometry::XYZ {
            y: v.velocity.y,
            x: v.orientation.yaw.sin() * 8.0,
            z: v.orientation.yaw.cos() * 8.0,
        };
        physics::car::update(
            v,
            TICK,
            Controls {
                steering: 0.8,
                handbrake: true,
                ..Controls::default()
            },
        );
        for _ in 0..20 {
            render::tire_particles::advance(&mut app.session.particles, TICK);
            render::tire_particles::record(&mut app.session.particles, 0, &app.session.car, TICK);
        }
        assert_particle_pixel_readback(&app);
        for fraction in [0.0, 0.25, 0.5, 0.75] {
            let road = world::locations::at_distance(
                session_track(&app.session),
                world::locations::sampling_length(session_track(&app.session)) * fraction,
            );
            physics::car::reset(
                &mut app.session.car,
                physics::vehicle_location::vehicle_position(road, 0.0),
                model::world::heading(road),
            );
            snap_camera(&mut app);
            assert_contact_pixel_readback(&app);
            for view in [
                render::camera::FollowView::Wide,
                render::camera::FollowView::Close,
            ] {
                while render::camera::view(&app.chase) != view {
                    key_down(&mut app, Keycode::Tab);
                }
                for _ in 0..120 {
                    render::camera::update(&mut app.chase, &app.session.car, TICK);
                }
                render(&mut app, TICK);
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
            render(&mut app, TICK);
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
    for id in test::support::location_ids() {
        let mut s = on_track(test::support::car(), id.clone());
        s.state = GameState::Menu;
        for model in test::support::cars() {
            session_select_car(&mut s, model.clone());
            assert!(std::sync::Arc::ptr_eq(&s.car.spec, &model));
            assert_eq!(session_track(&(s)).id, id);
            assert!(s.state == GameState::Menu);
            let road = world::locations::at_distance(
                session_track(&s),
                world::locations::sampling_length(session_track(&s)) * 0.5,
            );
            physics::car::reset(
                &mut s.car,
                physics::vehicle_location::vehicle_position(road, 0.0),
                model::world::heading(road),
            );
            s.recoveries = 2;
            session_restart(&mut s);
            assert_eq!(session_track(&(s)).id, id);
            assert!(std::sync::Arc::ptr_eq(&s.car.spec, &model));
            assert_eq!(
                s.car.position,
                physics::vehicle_location::vehicle_position(
                    physics::vehicle_location::spawn(session_track(&(s))),
                    0.0
                )
            );
            assert_eq!(s.recoveries, 0);
            assert!(s.state == GameState::Driving);
            s.state = GameState::Menu;
        }
    }
}

#[test]
fn free_recovery_uses_selected_route() {
    for id in test::support::location_ids() {
        let mut s = on_track(test::support::car(), id.clone());
        let road = world::locations::at_distance(
            session_track(&s),
            world::locations::sampling_length(session_track(&s)) * 0.5,
        );
        physics::car::reset(
            &mut s.car,
            physics::vehicle_location::vehicle_position(road, 1.0),
            model::world::heading(road),
        );
        assert!(recover(&mut (s)));
        let recovered = world::roads::project(session_track(&s), s.car.position);
        assert!((recovered.distance - road.distance).abs() < 0.1);
        assert!(recovered.offset.abs() < 0.01);
        assert_eq!(session_track(&(s)).id, id);
    }
}

#[test]
fn city_drive_recovery_pause_and_return_preserve_the_map() {
    let mut s = on_track(test::support::car(), test::support::city_id());
    let start = s.car.position;
    for _ in 0..240 {
        tick(
            &mut s,
            Controls {
                throttle: 0.6,
                ..Controls::default()
            },
        );
    }
    assert!((s.car.position.x - start.x).hypot(s.car.position.z - start.z) > 0.5);
    s.state = GameState::Paused;
    let frozen = s.car.position;
    for _ in 0..10 {
        tick(
            &mut s,
            Controls {
                throttle: 1.0,
                ..Controls::default()
            },
        );
    }
    assert_eq!(s.car.position, frozen);
    s.state = GameState::Driving;
    assert!(recover(&mut (s)));
    assert_eq!(session_track(&(s)).id, test::support::city_id());
    assert!(world::roads::road_clearance(session_track(&(s)), s.car.position) < 0.0);
}

#[test]
#[ignore = "requires SDL/OpenGL; verifies startup without a selected city"]
fn no_city_startup_renderer_smoke() {
    let fixtures = tempfile::tempdir().unwrap();
    let mut app = with_location(false);
    app.locations = ["first", "second"]
        .map(|name| {
            let assets =
                test::support::generated_location(&fixtures.path().join(name), 7.0, "test imagery");
            storage::city_assets::from_assets(assets).unwrap()
        })
        .to_vec();
    app.selected_location = 0;
    let city_id = app.locations[0].track.clone();
    ui::loading::draw(
        &app.window,
        &app.hud,
        app.program,
        "ΛΕΥΚΩΣΙΑ / NICOSIA",
        0.5,
    );
    capture_frame(&std::env::temp_dir().join("carslop-loading.ppm"));
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    render::gl::set_viewport(WINDOW_WIDTH, WINDOW_HEIGHT);
    render::gl::clear_screen();
    render_scene(&mut app, Duration::ZERO);
    capture_frame(&std::env::temp_dir().join("carslop-menu.ppm"));
    render(&mut app, Duration::ZERO);
    if app.cars.len() > 3 {
        menu_action(&mut app, ui::hud::MenuAction::Car(3));
        assert_eq!(app.selected_car, 3);
        assert!(std::sync::Arc::ptr_eq(&app.session.car.spec, &app.cars[3]));
        render(&mut app, Duration::ZERO);
        menu_action(&mut app, ui::hud::MenuAction::Car(0));
        assert_eq!(app.selected_car, 0);
        assert!(std::sync::Arc::ptr_eq(&app.session.car.spec, &app.cars[0]));
    }
    let city_choice = app
        .locations
        .iter()
        .position(|entry| entry.track == city_id)
        .unwrap();
    menu_action(&mut app, ui::hud::MenuAction::Choice(city_choice));
    key_down(&mut app, Keycode::F7);
    for _ in 0..6 {
        app.menu_focus = 0;
        key_down(&mut app, Keycode::Down);
        render(&mut app, Duration::ZERO);
        assert_ne!(session_track(&(app.session)).id, city_id);
    }
    assert!(!app.meshes.contains_key(&city_id));
    let scenic_choice = 1;
    menu_action(&mut app, ui::hud::MenuAction::Choice(scenic_choice));
    assert_eq!(session_track(&(app.session)).id, LocationId::Compact);
    key_down(&mut app, Keycode::Return);
    assert_eq!(
        session_track(&(app.session)).id,
        app.locations[scenic_choice].track
    );
    assert!(app.session.state == GameState::Driving);
    assert!(app.meshes.contains_key(&session_track(&(app.session)).id));
    render(&mut app, Duration::ZERO);
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    drop(app);
}

#[test]
fn session_changes_keep_the_loaded_world_after_another_selection_or_failure() {
    let root = tempfile::tempdir().unwrap();
    let first_assets =
        test::support::generated_location(&root.path().join("first"), 7.0, "Cyprus DLS");
    let second_assets =
        test::support::generated_location(&root.path().join("second"), 13.0, "arcgisonline.com");
    let first_id = world::locations::asset(storage::city_assets::directory(&first_assets)).unwrap();
    let second_id =
        world::locations::asset(storage::city_assets::directory(&second_assets)).unwrap();
    let mut cache = world::locations::LocationCache::default();
    let first = world::locations::prepare_for_render(&cache, &first_id).unwrap();
    world::locations::install(&mut cache, first.clone());
    let mut session = on_world(test::support::car(), first.clone());
    let second = world::locations::prepare_for_render(&cache, &second_id).unwrap();
    world::locations::install(&mut cache, second);
    session_restart(&mut session);
    assert!(std::sync::Arc::ptr_eq(&first, &session.car.track));
    assert_eq!(
        session.car.position,
        physics::vehicle_location::vehicle_position(physics::vehicle_location::spawn(&first), 0.0)
    );
    session.state = GameState::Menu;
    session_select_car(&mut session, test::support::car());
    assert!(std::sync::Arc::ptr_eq(&first, &session.car.track));
    let position = session.car.position;
    std::fs::remove_file(storage::city_assets::directory(&second_assets).join("scene.pack"))
        .unwrap();
    assert!(world::locations::prepare_for_render(&(cache), &second_id).is_err());
    assert_eq!(session.car.position, position);
    assert!(std::sync::Arc::ptr_eq(&first, &session.car.track));
}

#[test]
#[ignore = "requires SDL/OpenGL; checks two asset identities and failed render preparation"]
fn explicit_location_switch_renderer() {
    let root = tempfile::tempdir().unwrap();
    let first_assets =
        test::support::generated_location(&root.path().join("first"), 7.0, "Cyprus DLS");
    let second_assets =
        test::support::generated_location(&root.path().join("second"), 13.0, "arcgisonline.com");
    let first_id = world::locations::asset(storage::city_assets::directory(&first_assets)).unwrap();
    let second_id =
        world::locations::asset(storage::city_assets::directory(&second_assets)).unwrap();
    let mut app = with_location(false);
    let first = world::locations::try_track(&first_id).unwrap();
    install_location(&mut app, first.clone()).unwrap();
    render(&mut app, Duration::ZERO);
    let second = world::locations::try_track(&second_id).unwrap();
    std::fs::rename(
        storage::city_assets::directory(&second_assets).join("scene.pack"),
        storage::city_assets::directory(&second_assets).join("saved.pack"),
    )
    .unwrap();
    let position = app.session.car.position;
    assert!(install_location(&mut (app), second.clone()).is_err());
    assert!(std::sync::Arc::ptr_eq(&first, &app.session.car.track));
    assert_eq!(app.session.car.position, position);
    assert!(app.meshes.contains_key(&first_id));
    assert!(!app.meshes.contains_key(&second_id));
    render(&mut app, Duration::ZERO);
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
    std::fs::rename(
        storage::city_assets::directory(&second_assets).join("saved.pack"),
        storage::city_assets::directory(&second_assets).join("scene.pack"),
    )
    .unwrap();
    install_location(&mut app, second.clone()).unwrap();
    assert_eq!(session_track(&(app.session)).id, second_id);
    assert!(app.meshes.contains_key(&second_id));
    assert!(!app.meshes.contains_key(&first_id));
    assert!(std::sync::Arc::ptr_eq(
        &second,
        &world::locations::prepare_for_render(&(app.locations_cache), &second_id).unwrap()
    ));
    assert_eq!(
        storage::city_assets::directory(model::world::assets(&first).unwrap()),
        storage::city_assets::directory(&(first_assets))
    );
    render(&mut app, Duration::ZERO);
    assert_eq!(unsafe { ::gl::GetError() }, ::gl::NO_ERROR);
}
