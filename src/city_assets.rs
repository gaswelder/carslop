//! Runtime city data lives beside the executable, rather than in giant Rust
//! string literals. This keeps compiling and loading larger regions practical.
use std::path::{Path, PathBuf};
static SELECTED: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();
pub fn select(path: PathBuf) -> std::io::Result<()> {
    SELECTED
        .set(path.canonicalize()?)
        .map_err(|_| std::io::Error::other("city location already selected"))
}
pub fn name() -> &'static str {
    static NAME: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    NAME.get_or_init(|| {
        std::fs::read_to_string(directory().join("manifest.json"))
            .ok()
            .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
            .and_then(|v| v.get("name").and_then(|n| n.as_str()).map(str::to_owned))
            .unwrap_or_else(|| "NICOSIA URBAN AREA".into())
            .replace(['_', '-'], " ")
            .to_ascii_uppercase()
    })
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
    if let Some(path) = SELECTED.get() {
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
        root.join("nicosia")
    })
}
pub fn read(name: &str) -> String {
    std::fs::read_to_string(directory().join(name))
        .unwrap_or_else(|e| panic!("cannot read city asset {name}: {e}"))
}
