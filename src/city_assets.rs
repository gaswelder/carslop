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
pub fn directory() -> &'static Path {
    if let Some(path) = SELECTED.get() {
        return path;
    }
    static DIRECTORY: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();
    DIRECTORY.get_or_init(|| {
        if let Some(path) = std::env::var_os("NFS_CITY_DATA") {
            return PathBuf::from(path);
        }
        let mut paths = Vec::new();
        if let Ok(exe) = std::env::current_exe() {
            if let Some(parent) = exe.parent() {
                paths.push(parent.join("data/engomi"));
            }
        }
        paths.push(PathBuf::from("data/engomi"));
        paths.push(Path::new(env!("CARGO_MANIFEST_DIR")).join("data/engomi"));
        paths
            .into_iter()
            .find(|p| p.join("city.txt").is_file())
            .expect("missing city data directory")
    })
}
pub fn read(name: &str) -> String {
    std::fs::read_to_string(directory().join(name))
        .unwrap_or_else(|e| panic!("cannot read city asset {name}: {e}"))
}
