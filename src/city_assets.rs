//! Runtime city data lives beside the executable, rather than in giant Rust
//! string literals. This keeps compiling and loading larger regions practical.
use std::path::{Path, PathBuf};
static SELECTED: std::sync::RwLock<Option<&'static Path>> = std::sync::RwLock::new(None);
pub fn select(path: PathBuf) -> std::io::Result<()> {
    let path = path.canonicalize()?;
    if directory() != path {
        *SELECTED.write().unwrap() = Some(Box::leak(path.into_boxed_path()));
    }
    Ok(())
}

/// Cache immutable runtime data separately for each selected location.
pub fn cached<T: Send + Sync + 'static>(
    cache: &'static std::sync::OnceLock<
        std::sync::Mutex<std::collections::HashMap<PathBuf, &'static T>>,
    >,
    build: impl FnOnce() -> T,
) -> &'static T {
    let mut entries = cache.get_or_init(Default::default).lock().unwrap();
    entries
        .entry(directory().to_path_buf())
        .or_insert_with(|| Box::leak(Box::new(build())))
}
pub fn name() -> &'static str {
    static NAME: std::sync::OnceLock<
        std::sync::Mutex<std::collections::HashMap<PathBuf, &'static String>>,
    > = std::sync::OnceLock::new();
    cached::<String>(&NAME, || {
        crate::package::read_to_string(directory().join("manifest.json"))
            .ok()
            .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
            .and_then(|v| v.get("name").and_then(|n| n.as_str()).map(str::to_owned))
            .unwrap_or_else(|| {
                directory()
                    .file_stem()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into_owned()
            })
            .replace(['_', '-'], " ")
            .to_ascii_uppercase()
    })
}

#[derive(Clone, Debug)]
pub struct MenuLocation {
    pub name: String,
    pub track: crate::track::TrackId,
    pub path: Option<PathBuf>,
}
fn built_in_locations() -> Vec<MenuLocation> {
    vec![
        MenuLocation {
            name: "COASTAL HIGHWAY".into(),
            track: crate::track::TrackId::Scenic,
            path: None,
        },
        MenuLocation {
            name: "COASTAL LOOP".into(),
            track: crate::track::TrackId::Compact,
            path: None,
        },
    ]
}
/// Discover names and paths without reading map assets or building tracks.
pub fn discover(root: &Path) -> std::io::Result<Vec<MenuLocation>> {
    let mut locations = built_in_locations();
    if !root.exists() {
        return Ok(locations);
    }
    let mut paths = std::fs::read_dir(root)?
        .map(|e| e.map(|e| e.path()))
        .collect::<std::io::Result<Vec<_>>>()?;
    paths.sort();
    for path in paths {
        let package = crate::package::is_package(&path) && path.is_file();
        if !package
            && !(path.is_dir()
                && path.join("city.txt").is_file()
                && !path.with_extension("sqlite").is_file())
        {
            continue;
        }
        let name = if package {
            path.file_stem()
        } else {
            path.file_name()
        };
        let Some(name) = name.and_then(|s| s.to_str()) else {
            continue;
        };
        if name.starts_with('.') {
            continue;
        }
        locations.push(MenuLocation {
            name: name.replace(['_', '-'], " ").to_ascii_uppercase(),
            track: crate::track::TrackId::Engomi,
            path: Some(path),
        });
    }
    Ok(locations)
}
pub fn menu_locations() -> Vec<MenuLocation> {
    let root = LOCATIONS
        .get()
        .cloned()
        .or_else(|| std::env::var_os("NFS_LOCATIONS_DIR").map(PathBuf::from))
        .unwrap_or_else(default_locations_directory);
    match discover(&root) {
        Ok(locations) => locations,
        Err(error) => {
            log::warn!("Cannot list locations: {error}");
            built_in_locations()
        }
    }
}
static LOCATIONS: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();
/// Prefer a portable asset folder beside the executable, then the working tree.
pub fn default_locations_directory() -> PathBuf {
    let mut paths = vec![];
    if let Ok(exe) = std::env::current_exe() {
        if let Some(parent) = exe.parent() {
            paths.push(parent.join("data/locations"));
        }
    }
    paths.push(PathBuf::from("data/locations"));
    paths.push(Path::new(env!("CARGO_MANIFEST_DIR")).join("data/locations"));
    paths
        .into_iter()
        .find(|p| p.is_dir())
        .unwrap_or_else(|| PathBuf::from("data/locations"))
}
pub fn configure_locations(root: &Path) -> std::io::Result<()> {
    LOCATIONS
        .set(root.to_path_buf())
        .map_err(|_| std::io::Error::other("location directory already configured"))
}
pub fn directory() -> &'static Path {
    if let Some(path) = *SELECTED.read().unwrap() {
        return path;
    }
    static DIRECTORY: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();
    DIRECTORY.get_or_init(|| {
        if let Some(path) = std::env::var_os("NFS_CITY_DATA") {
            return path.into();
        }
        let root = LOCATIONS
            .get()
            .cloned()
            .or_else(|| std::env::var_os("NFS_LOCATIONS_DIR").map(PathBuf::from))
            .unwrap_or_else(default_locations_directory);
        let package = root.join("nicosia.sqlite");
        if package.is_file() {
            package
        } else {
            root.join("nicosia")
        }
    })
}
pub fn try_read(name: &str) -> Result<String, String> {
    crate::package::read_to_string(directory().join(name))
        .map_err(|e| format!("cannot read city asset {name}: {e}"))
}

#[cfg(test)]
pub fn read(name: &str) -> String {
    try_read(name).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discovery_lists_packages_and_folders_without_loading_maps() {
        let root = tempfile::tempdir().unwrap();
        // Invalid map contents intentionally prove discovery never decodes them.
        std::fs::write(root.path().join("zebra.sqlite"), b"not a database").unwrap();
        let folder = root.path().join("alpha_city");
        std::fs::create_dir(&folder).unwrap();
        std::fs::write(folder.join("city.txt"), b"not a map").unwrap();
        std::fs::create_dir(root.path().join("empty")).unwrap();
        std::fs::write(root.path().join("notes.txt"), b"ignore").unwrap();
        let locations = discover(root.path()).unwrap();
        assert_eq!(locations.len(), 4);
        assert_eq!(locations[2].name, "ALPHA CITY");
        assert_eq!(locations[3].name, "ZEBRA");
        assert_eq!(locations[2].path.as_deref(), Some(folder.as_path()));
        std::fs::write(root.path().join("alpha_city.sqlite"), b"package preferred").unwrap();
        let locations = discover(root.path()).unwrap();
        assert_eq!(locations.len(), 4);
        assert!(locations[2].path.as_ref().unwrap().is_file());
        assert_eq!(discover(&root.path().join("missing")).unwrap().len(), 2);
    }
}
