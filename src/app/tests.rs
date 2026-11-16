use super::FollowView;
use crate::app;

#[test]
fn map_zoom_round_trips() {
    let root = tempfile::tempdir().unwrap();
    let path = root.path().join("settings.ini");
    let mut settings = app::settings::load_from(path.clone());
    assert_eq!(app::settings::map_zoom(&(settings)), None);
    app::settings::set_map_zoom(&mut settings, 0.75);
    assert_eq!(
        app::settings::map_zoom(&(app::settings::load_from(path))),
        Some(0.75)
    );
}

#[test]
fn camera_round_trips() {
    let root = tempfile::tempdir().unwrap();
    let path = root.path().join("settings.ini");
    let mut settings = app::settings::load_from(path.clone());
    assert_eq!(app::settings::camera(&(settings)), None);
    app::settings::set_camera(&mut settings, FollowView::Bonnet);
    assert_eq!(
        app::settings::camera(&(app::settings::load_from(path))),
        Some(FollowView::Bonnet)
    );
}

#[test]
fn unrelated_settings_are_preserved() {
    let root = tempfile::tempdir().unwrap();
    let path = root.path().join("settings.ini");
    std::fs::write(&path, "volume = 0.8\r\nmap_zoom = 1\r\n").unwrap();
    let mut settings = app::settings::load_from(path.clone());
    assert_eq!(app::settings::map_zoom(&(settings)), Some(1.0));
    app::settings::set_camera(&mut settings, FollowView::Wide);
    app::settings::set_map_zoom(&mut settings, 2.5);
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains("volume = 0.8\r\n"));
    assert!(text.contains("map_zoom = 2.5\r\n"));
    assert!(text.contains("camera = wide\r\n"));
    let settings = app::settings::load_from(path);
    assert_eq!(app::settings::map_zoom(&(settings)), Some(2.5));
    assert_eq!(app::settings::camera(&(settings)), Some(FollowView::Wide));
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
        let settings = app::settings::load_from(path);
        assert_eq!(app::settings::map_zoom(&(settings)), None);
        assert_eq!(app::settings::camera(&(settings)), None);
    }
}

#[test]
fn keys_are_case_insensitive() {
    let root = tempfile::tempdir().unwrap();
    let path = root.path().join("settings.ini");
    std::fs::write(&path, "  MAP_ZOOM = 0.5  \n  CAMERA = Bonnet  \n").unwrap();
    let settings = app::settings::load_from(path);
    assert_eq!(app::settings::map_zoom(&(settings)), Some(0.5));
    assert_eq!(app::settings::camera(&(settings)), Some(FollowView::Bonnet));
}
