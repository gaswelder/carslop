//! Persistent user settings stored in `settings.ini`.

use crate::camera;
use crate::camera::FollowView;
use crate::file_io;
use std::path::PathBuf;

const PATH: &str = "./settings.ini";
const MAP_ZOOM: &str = "map_zoom";
const CAMERA: &str = "camera";

/// Settings loaded from `settings.ini`. Missing or malformed entries use defaults.
#[derive(Default)]
pub struct Settings {
    path: Option<PathBuf>,
    map_zoom: Option<f32>,
    camera: Option<FollowView>,
}

pub fn load() -> Settings {
    load_from(PathBuf::from(PATH))
}

fn load_from(path: PathBuf) -> Settings {
    let text = std::fs::read_to_string(&path).unwrap_or_default();
    let map_zoom = value(&text, MAP_ZOOM).and_then(|raw| {
        raw.trim()
            .parse()
            .ok()
            .filter(|zoom: &f32| zoom.is_finite() && *zoom > 0.0)
    });
    let camera = value(&text, CAMERA).and_then(|name| camera::from_name(name.trim()));
    Settings {
        path: Some(path),
        map_zoom,
        camera,
    }
}

/// The remembered zoom factor, relative to each location's default fitted view.
pub fn map_zoom(settings: &Settings) -> Option<f32> {
    settings.map_zoom
}

pub fn camera(settings: &Settings) -> Option<FollowView> {
    settings.camera
}

pub fn set_map_zoom(settings: &mut Settings, zoom: f32) {
    settings.map_zoom = Some(zoom);
    save(settings, MAP_ZOOM, &zoom.to_string());
}

pub fn set_camera(settings: &mut Settings, camera: FollowView) {
    settings.camera = Some(camera);
    save(settings, CAMERA, camera::view_name(camera));
}

fn save(settings: &Settings, key: &str, value: &str) {
    let Some(path) = settings.path.as_deref() else {
        return;
    };
    let text = std::fs::read_to_string(path).unwrap_or_default();
    let text = set_value(&text, key, value);
    if let Err(error) = file_io::write_file(path, text.as_bytes()) {
        log::warn!("Cannot save {}: {error}", path.display());
    }
}

fn value<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with(';') {
            continue;
        }
        if let Some((name, value)) = line.split_once('=') {
            if name.trim().eq_ignore_ascii_case(key) {
                return Some(value.trim());
            }
        }
    }
    None
}

fn set_value(text: &str, key: &str, value: &str) -> String {
    let newline = if text.contains("\r\n") { "\r\n" } else { "\n" };
    let mut output = String::with_capacity(text.len() + 32);
    let mut written = false;
    for line in text.split_inclusive('\n') {
        let content = line.trim_end_matches(|c| c == '\r' || c == '\n');
        let ending = &line[content.len()..];
        if !written {
            if let Some((name, _)) = content.split_once('=') {
                if name.trim().eq_ignore_ascii_case(key) {
                    output.push_str(key);
                    output.push_str(" = ");
                    output.push_str(value);
                    output.push_str(ending);
                    written = true;
                    continue;
                }
            }
        }
        output.push_str(line);
    }
    if !written {
        if !output.is_empty() && !output.ends_with('\n') {
            output.push_str(newline);
        }
        push_setting(&mut output, key, value, newline);
    }
    output
}

fn push_setting(output: &mut String, key: &str, value: &str, newline: &str) {
    output.push_str(key);
    output.push_str(" = ");
    output.push_str(value);
    output.push_str(newline);
}

#[cfg(test)]
mod tests {
    use crate::settings;

    use super::FollowView;

    #[test]
    fn map_zoom_round_trips() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("settings.ini");
        let mut settings = settings::load_from(path.clone());
        assert_eq!(settings::map_zoom(&(settings)), None);
        settings::set_map_zoom(&mut settings, 0.75);
        assert_eq!(settings::map_zoom(&(settings::load_from(path))), Some(0.75));
    }

    #[test]
    fn camera_round_trips() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("settings.ini");
        let mut settings = settings::load_from(path.clone());
        assert_eq!(settings::camera(&(settings)), None);
        settings::set_camera(&mut settings, FollowView::Bonnet);
        assert_eq!(
            settings::camera(&(settings::load_from(path))),
            Some(FollowView::Bonnet)
        );
    }

    #[test]
    fn unrelated_settings_are_preserved() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("settings.ini");
        std::fs::write(&path, "volume = 0.8\r\nmap_zoom = 1\r\n").unwrap();
        let mut settings = settings::load_from(path.clone());
        assert_eq!(settings::map_zoom(&(settings)), Some(1.0));
        settings::set_camera(&mut settings, FollowView::Wide);
        settings::set_map_zoom(&mut settings, 2.5);
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("volume = 0.8\r\n"));
        assert!(text.contains("map_zoom = 2.5\r\n"));
        assert!(text.contains("camera = wide\r\n"));
        let settings = settings::load_from(path);
        assert_eq!(settings::map_zoom(&(settings)), Some(2.5));
        assert_eq!(settings::camera(&(settings)), Some(FollowView::Wide));
    }

    #[test]
    fn missing_or_invalid_values_are_ignored() {
        let root = tempfile::tempdir().unwrap();
        for text in [
            "",
            "map_zoom = nope\n",
            "map_zoom = 0\n",
            "map_zoom = -1\n",
            "map_zoom = NaN\n",
            "camera = nope\n",
            "volume = 0.8\n",
        ] {
            let path = root.path().join("settings.ini");
            std::fs::write(&path, text).unwrap();
            let settings = settings::load_from(path);
            assert_eq!(settings::map_zoom(&(settings)), None);
            assert_eq!(settings::camera(&(settings)), None);
        }
    }

    #[test]
    fn keys_are_case_insensitive() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("settings.ini");
        std::fs::write(&path, "  MAP_ZOOM = 0.5  \n  CAMERA = Bonnet  \n").unwrap();
        let settings = settings::load_from(path);
        assert_eq!(settings::map_zoom(&(settings)), Some(0.5));
        assert_eq!(settings::camera(&(settings)), Some(FollowView::Bonnet));
    }
}
