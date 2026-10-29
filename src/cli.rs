//! Commands run before SDL or a graphics context is initialized.
use crate::location::{invalid, Result};
use std::{
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
};
const HELP:&str="Usage: nfs5like [COMMAND]\n\nCommands:\n  play [--location NAME]                  Open the game (default command)\n  add-location NAME --source FILE.osm     Import a local OSM XML extract\n      [--bbox SOUTH WEST NORTH EAST]      Crop; defaults to OSM bounds\n      (--elevation TILE.hgt[.gz] | --flat-terrain)\n  list-locations                         List imported locations\n  bake-city [--location NAME]             Rebuild the render archive\n\n  extract-region SOURCE.pbf OUTPUT.osm [--bbox SOUTH WEST NORTH EAST]\n  merge-osm ORIGINAL.osm[.gz] EXTENSION.osm[.gz] OUTPUT.osm\n  build-aerial-tiles --image FILE --export FILE --output DIR\n      [--size 512] [--gutter 8] [--preview-output DIR]\n\nGlobal options:\n  --locations-dir PATH                    Location store (also NFS_LOCATIONS_DIR)\n  -h, --help                              Show help\n  -V, --version                           Show version\n\nImported locations are stored under the user's data directory.\nBundled city: play --location nicosia. No command opens the existing menu.\n";
#[derive(Debug, PartialEq)]
enum Command {
    Play(Option<String>),
    Add {
        name: String,
        source: PathBuf,
        bbox: Option<[f64; 4]>,
        elevation: Option<PathBuf>,
        flat: bool,
    },
    List,
    Bake(Option<String>),
    Source(Vec<OsString>),
    Help,
    Version,
}
struct Args {
    command: Command,
    root: PathBuf,
}
fn root() -> Result<PathBuf> {
    if let Some(p) = std::env::var_os("NFS_LOCATIONS_DIR") {
        return Ok(p.into());
    }
    #[cfg(target_os = "windows")]
    if let Some(p) = std::env::var_os("LOCALAPPDATA") {
        return Ok(PathBuf::from(p).join("nfs5like/locations"));
    }
    #[cfg(target_os = "macos")]
    if let Some(p) = std::env::var_os("HOME") {
        return Ok(PathBuf::from(p).join("Library/Application Support/nfs5like/locations"));
    }
    if let Some(p) = std::env::var_os("XDG_DATA_HOME").filter(|p| Path::new(p).is_absolute()) {
        return Ok(PathBuf::from(p).join("nfs5like/locations"));
    }
    Ok(PathBuf::from(
        std::env::var_os("HOME")
            .ok_or_else(|| invalid("no user data directory; supply --locations-dir"))?,
    )
    .join(".local/share/nfs5like/locations"))
}
fn valid_name(s: &str) -> bool {
    !s.is_empty()
        && s.len() <= 64
        && s.as_bytes()[0].is_ascii_alphanumeric()
        && s.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}
fn text(s: &OsString) -> Result<String> {
    s.clone()
        .into_string()
        .map_err(|_| invalid("expected UTF-8 command or location name"))
}
fn parse(mut args: Vec<OsString>, default_root: PathBuf) -> Result<Args> {
    let mut root = default_root;
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--locations-dir" {
            if i + 1 >= args.len() {
                return Err(invalid("--locations-dir needs a path"));
            }
            root = PathBuf::from(args.remove(i + 1));
            args.remove(i);
        } else {
            i += 1;
        }
    }
    if args.iter().any(|a| a == "--help" || a == "-h") {
        return Ok(Args {
            command: Command::Help,
            root,
        });
    }
    if args == [OsString::from("--version")] || args == [OsString::from("-V")] {
        return Ok(Args {
            command: Command::Version,
            root,
        });
    }
    if args.is_empty() {
        return Ok(Args {
            command: Command::Play(None),
            root,
        });
    }
    let cmd = text(&args.remove(0))?;
    if matches!(
        cmd.as_str(),
        "extract-region" | "merge-osm" | "build-aerial-tiles"
    ) {
        args.insert(0, cmd.into());
        return Ok(Args {
            command: Command::Source(args),
            root,
        });
    }
    let mut location = None;
    let mut source = None;
    let mut bbox = None;
    let mut elevation = None;
    let mut flat = false;
    let mut name = None;
    let mut values = args.into_iter();
    while let Some(arg) = values.next() {
        let key = text(&arg)?;
        match key.as_str() {
            "--location" if matches!(cmd.as_str(), "play" | "bake-city" | "--bake-city") => {
                if location.is_some() {
                    return Err(invalid("duplicate --location"));
                }
                location = Some(text(
                    &values
                        .next()
                        .ok_or_else(|| invalid("--location needs a name"))?,
                )?);
            }
            "--source" if cmd == "add-location" => {
                if source.is_some() {
                    return Err(invalid("duplicate --source"));
                }
                source = Some(PathBuf::from(
                    values
                        .next()
                        .ok_or_else(|| invalid("--source needs a file"))?,
                ));
            }
            "--elevation" if cmd == "add-location" => {
                if elevation.is_some() {
                    return Err(invalid("duplicate --elevation"));
                }
                elevation = Some(PathBuf::from(
                    values
                        .next()
                        .ok_or_else(|| invalid("--elevation needs a file"))?,
                ));
            }
            "--flat-terrain" if cmd == "add-location" => flat = true,
            "--bbox" if cmd == "add-location" => {
                if bbox.is_some() {
                    return Err(invalid("duplicate --bbox"));
                }
                let mut b = [0.0; 4];
                for v in &mut b {
                    *v = text(
                        &values
                            .next()
                            .ok_or_else(|| invalid("--bbox requires SOUTH WEST NORTH EAST"))?,
                    )?
                    .parse::<f64>()?;
                }
                bbox = Some(b);
            }
            _ if cmd == "add-location" && name.is_none() && !key.starts_with('-') => {
                name = Some(key)
            }
            _ => return Err(invalid(format!("unexpected argument: {key}\n\n{HELP}"))),
        }
    }
    if let Some(n) = location.as_ref().or(name.as_ref()) {
        if !valid_name(n) {
            return Err(invalid("location names must start with a letter or digit and contain only letters, digits, - or _ (max 64 characters)"));
        }
    }
    let command = match cmd.as_str() {
        "play" => Command::Play(location),
        "list-locations" => Command::List,
        "bake-city" | "--bake-city" => Command::Bake(location),
        "add-location" => {
            let name = name.ok_or_else(|| invalid("add-location requires NAME"))?;
            if name.eq_ignore_ascii_case("nicosia") {
                return Err(invalid("nicosia is reserved for the bundled city"));
            }
            if flat == elevation.is_some() {
                return Err(invalid(
                    "choose exactly one of --elevation TILE.hgt[.gz] or --flat-terrain",
                ));
            }
            Command::Add {
                name,
                source: source.ok_or_else(|| invalid("add-location requires --source FILE.osm"))?,
                bbox,
                elevation,
                flat,
            }
        }
        _ => return Err(invalid(format!("unknown command: {cmd}\n\n{HELP}"))),
    };
    Ok(Args { command, root })
}
fn select(root: &Path, name: &str) -> Result<()> {
    let path = if name.eq_ignore_ascii_case("nicosia") {
        crate::city_assets::directory().to_path_buf()
    } else {
        root.join(name)
    };
    validate(&path)?;
    crate::city_assets::select(path)?;
    Ok(())
}
fn validate(path: &Path) -> Result<()> {
    for name in [
        "city.txt",
        "streets.txt",
        "furniture.txt",
        "landscape.txt",
        "elevation/heightfield.txt",
    ] {
        if !path.join(name).is_file() {
            return Err(invalid(format!(
                "missing location asset: {}",
                path.join(name).display()
            )));
        }
    }
    Ok(())
}
struct Pending(PathBuf);
impl Drop for Pending {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
/// Returns true when the game should be opened, after resolving its location.
pub fn run() -> Result<Option<bool>> {
    let args = parse(std::env::args_os().skip(1).collect(), root()?)?;
    match args.command {
        Command::Source(values) => crate::source_tools::run(values)?,
        Command::Help => print!("{HELP}"),
        Command::Version => println!("nfs5like {}", env!("CARGO_PKG_VERSION")),
        Command::Play(location) => {
            if let Some(name) = location {
                select(&args.root, &name)?;
                return Ok(Some(true));
            }
            return Ok(Some(false));
        }
        Command::List => {
            println!("nicosia\tBundled city");
            if args.root.exists() {
                let mut paths = fs::read_dir(&args.root)?.collect::<std::io::Result<Vec<_>>>()?;
                paths.sort_by_key(|e| e.file_name());
                for e in paths {
                    let name = e.file_name();
                    let Some(name) = name.to_str() else {
                        continue;
                    };
                    if valid_name(name) && e.file_type()?.is_dir() && validate(&e.path()).is_ok() {
                        println!("{name}\t{}", e.path().display());
                    }
                }
            }
        }
        Command::Bake(location) => {
            if let Some(name) = location {
                select(&args.root, &name)?;
            }
            crate::scene_archive::bake_default()?;
        }
        Command::Add {
            name,
            source,
            bbox,
            elevation,
            flat,
        } => {
            let destination = args.root.join(&name);
            if destination.exists() {
                return Err(invalid(format!(
                    "location {name} already exists; choose another name"
                )));
            }
            fs::create_dir_all(&args.root)?;
            let staging = args
                .root
                .join(format!(".import-{name}-{}", std::process::id()));
            fs::create_dir(&staging)?;
            let pending = Pending(staging);
            crate::location::compile(crate::location::Options {
                source: &source,
                output: &pending.0,
                name: &name,
                bbox,
                elevation: elevation.as_deref(),
                flat,
            })?;
            validate(&pending.0)?;
            crate::city_assets::select(pending.0.clone())?;
            println!("Baking render archive...");
            crate::scene_archive::bake_default()?;
            if destination.exists() {
                return Err(invalid(
                    "location appeared during import; refusing to replace it",
                ));
            }
            fs::rename(&pending.0, &destination)?;
            println!(
                "Added {name}: {}\nSelect with: nfs5like play --location {name}\nUse the same --locations-dir if you supplied one.",
                destination.display()
            );
        }
    }
    Ok(None)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn args(a: &[&str]) -> Result<Args> {
        parse(
            a.iter().map(OsString::from).collect(),
            PathBuf::from("/tmp/test-locations"),
        )
    }
    #[test]
    fn command_routing() {
        assert_eq!(args(&[]).unwrap().command, Command::Play(None));
        assert_eq!(
            args(&["play", "--location", "limassol"]).unwrap().command,
            Command::Play(Some("limassol".into()))
        );
        assert_eq!(args(&["--bake-city"]).unwrap().command, Command::Bake(None));
        assert!(args(&["unknown"]).is_err());
        assert!(args(&["play", "typo"]).is_err());
    }
    #[test]
    fn import_requires_explicit_terrain_and_safe_name() {
        for a in [
            vec!["add-location", "test", "--source", "a.osm"],
            vec![
                "add-location",
                "../escape",
                "--source",
                "a.osm",
                "--flat-terrain",
            ],
            vec![
                "add-location",
                "nicosia",
                "--source",
                "a.osm",
                "--flat-terrain",
            ],
            vec![
                "add-location",
                "test",
                "--source",
                "a.osm",
                "--flat-terrain",
                "--elevation",
                "a.hgt",
            ],
        ] {
            assert!(args(&a).is_err());
        }
        assert!(args(&[
            "add-location",
            "test",
            "--source",
            "a.osm",
            "--flat-terrain"
        ])
        .is_ok());
    }
    #[test]
    fn global_location_directory() {
        let a = args(&["--locations-dir", "/tmp/elsewhere", "list-locations"]).unwrap();
        assert_eq!(a.root, PathBuf::from("/tmp/elsewhere"));
        assert_eq!(a.command, Command::List);
    }
}
