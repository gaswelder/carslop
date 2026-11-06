//! A responsive startup screen; map work stays off the OpenGL thread.
use crate::{hud, track::TrackId};
use sdl3::{
    event::{Event, WindowEvent},
    keyboard::Keycode,
    video::Window,
};
use std::time::{Duration, Instant};

/// Warm the shared city map and validate its archive before creating a session.
/// A close request is requeued for the application's normal shutdown path.
pub fn prepare(
    id: TrackId,
    window: &mut Window,
    events: &mut sdl3::EventPump,
    sdl: &sdl3::Sdl,
) -> bool {
    if id != TrackId::Engomi {
        return match id.try_track() {
            Ok(_) => true,
            Err(error) => {
                log::warn!("Cannot load track: {error}");
                false
            }
        };
    }
    let started = Instant::now();
    let title = window.title().to_owned();
    let name = crate::city_assets::CITY_ASSET
        .read()
        .unwrap()
        .name()
        .to_owned();
    draw(window, &name, "READING MAP AND BUILDING COLLISIONS", 0.0);
    crate::gl::swap_window(window);
    let worker = std::thread::Builder::new()
        .name("location-loader".into())
        .spawn(|| {
            TrackId::Engomi.try_track()?;
            crate::scene_archive::Archive::open_default().map_err(|e| e.to_string())?;
            Ok::<_, String>(())
        })
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
                return false;
            }
        }
        let elapsed = started.elapsed().as_secs_f32();
        let _ = window.set_title(&format!("Loading {name}"));
        draw(
            window,
            &name,
            "READING MAP AND BUILDING COLLISIONS",
            elapsed,
        );
        crate::gl::swap_window(window);
        std::thread::sleep(Duration::from_millis(16));
    }
    let result = worker
        .join()
        .unwrap_or_else(|_| Err("location loader panicked".into()));
    if let Err(error) = result {
        log::warn!("Cannot load location: {error}");
        let _ = sdl3::messagebox::show_simple_message_box(
            sdl3::messagebox::MessageBoxFlag::ERROR,
            "Cannot load location",
            &error,
            Some(&*window),
        );
        let _ = window.set_title(&title);
        return false;
    }
    let _ = window.set_title(&title);
    true
}

/// Scissored clears need no shaders or geometry and reuse the HUD's bitmap font.
pub fn draw(window: &Window, name: &str, stage: &str, elapsed: f32) {
    let (width, height) = window.size_in_pixels();
    unsafe {
        gl::Disable(gl::SCISSOR_TEST);
        gl::ClearColor(0.025, 0.05, 0.075, 1.0);
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        gl::Enable(gl::SCISSOR_TEST);
    }
    let rect = |a: [f32; 2], b: [f32; 2], color: [f32; 3]| unsafe {
        let x = ((a[0] + 1.0) * 0.5 * width as f32) as i32;
        let y = ((a[1] + 1.0) * 0.5 * height as f32) as i32;
        let right = ((b[0] + 1.0) * 0.5 * width as f32) as i32;
        let top = ((b[1] + 1.0) * 0.5 * height as f32) as i32;
        gl::Scissor(x, y, (right - x).max(1), (top - y).max(1));
        gl::ClearColor(color[0], color[1], color[2], 1.0);
        gl::Clear(gl::COLOR_BUFFER_BIT);
    };
    let text = |value: &str, y: f32, size: f32, color| {
        let origin = -(value.chars().count() as f32 * 6.0 * size) * 0.5;
        for (index, ch) in value.chars().enumerate() {
            for (row, bits) in hud::glyph(ch).iter().enumerate() {
                for column in 0..5 {
                    if bits & (1 << (4 - column)) != 0 {
                        let x = origin + (index * 6 + column) as f32 * size;
                        let y = y - row as f32 * size * 1.65;
                        rect([x, y - size * 1.5], [x + size * 0.9, y], color);
                    }
                }
            }
        }
    };
    text("LOADING LOCATION", 0.35, 0.009, [0.15, 0.85, 0.95]);
    let name: String = name.chars().take(48).collect();
    text(&name, 0.15, 0.005, [0.9, 0.95, 1.0]);
    text(stage, -0.02, 0.004, [0.55, 0.66, 0.70]);
    rect([-0.55, -0.22], [0.55, -0.19], [0.12, 0.20, 0.25]);
    let x = -0.55 + (elapsed * 1.5).sin().mul_add(0.5, 0.5) * 0.9;
    rect([x, -0.22], [x + 0.2, -0.19], [0.15, 0.85, 0.95]);
    text("ESC TO CLOSE", -0.52, 0.003, [0.55, 0.66, 0.70]);
    unsafe {
        gl::Disable(gl::SCISSOR_TEST);
    }
}
