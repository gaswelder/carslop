//! A responsive startup screen; map work stays off the OpenGL thread.

use crate::city_assets;
use crate::gl::swap_window;
use crate::track;
use crate::world;
use crate::{hud, track::LocationId};
use sdl3::{
    event::{Event, WindowEvent},
    keyboard::Keycode,
    video::Window,
};
use std::time::{Duration, Instant};

/// Prepare an explicitly identified candidate and validate its archive.
/// A close request is requeued for the application's normal shutdown path.
pub fn prepare(
    id: LocationId,
    cache: track::LocationCache,
    window: &mut Window,
    events: &mut sdl3::EventPump,
    sdl: &sdl3::Sdl,
    hud: &hud::Hud,
    program: u32,
) -> Option<std::sync::Arc<world::WorldLocation>> {
    if track::asset_path(&id).is_none() {
        return track::prepare_for_render(&cache, &id)
            .map_err(|error| log::warn!("Cannot load track: {error}"))
            .ok();
    }
    let started = Instant::now();
    let title = window.title().to_owned();
    let name = city_assets::name(&city_assets::new(
        track::asset_path(&id).unwrap().to_path_buf(),
    ))
    .to_owned();
    draw(window, hud, program, &name, 0.0);
    swap_window(window);
    let worker = std::thread::Builder::new()
        .name("location-loader".into())
        .spawn(move || track::prepare_for_render(&cache, &id))
        .expect("failed to start location loader");
    while !worker.is_finished() {
        while let Some(event) = events.poll_event() {
            if matches!(
                event,
                Event::Quit { .. }
                    | Event::Window {
                        win_event: WindowEvent::CloseRequested,
                        ..
                    }
                    | Event::KeyDown {
                        keycode: Some(Keycode::Q | Keycode::Escape),
                        ..
                    }
            ) {
                let _ = sdl
                    .event()
                    .and_then(|events| events.push_event(Event::Quit { timestamp: 0 }));
                let _ = worker.join();
                let _ = window.set_title(&title);
                return None;
            }
        }
        let elapsed = started.elapsed().as_secs_f32();
        let _ = window.set_title(&format!("Loading {name}"));
        draw(window, hud, program, &name, elapsed);
        swap_window(window);
        std::thread::sleep(Duration::from_millis(16));
    }
    let result = worker
        .join()
        .unwrap_or_else(|_| Err("location loader panicked".into()));
    if let Err(error) = &result {
        log::warn!("Cannot load location: {error}");
        let _ = sdl3::messagebox::show_simple_message_box(
            sdl3::messagebox::MessageBoxFlag::ERROR,
            "Cannot load location",
            &error,
            Some(&*window),
        );
        let _ = window.set_title(&title);
        return None;
    }
    let _ = window.set_title(&title);
    result.ok()
}

/// Reuse the normal HUD geometry so startup text uses the same outline fonts.
pub fn draw(window: &Window, hud: &hud::Hud, program: u32, name: &str, elapsed: f32) {
    let (width, height) = window.size_in_pixels();
    unsafe {
        gl::Viewport(0, 0, width as i32, height as i32);
        gl::Disable(gl::SCISSOR_TEST);
        gl::ClearColor(0.025, 0.05, 0.075, 1.0);
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
    }
    hud::draw_loading(hud, program, name, elapsed);
}
