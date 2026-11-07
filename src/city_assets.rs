use std::{
    path::{Path, PathBuf},
    sync::{LazyLock, RwLock},
};

pub const LOCATIONS_DIRECTORY: &str = "data/locations";

#[derive(Clone, Debug)]
pub struct CityAsset {
    directory: PathBuf,
    name: String,
}

pub static CITY_ASSET: LazyLock<RwLock<CityAsset>> =
    LazyLock::new(|| RwLock::new(CityAsset::new(PathBuf::from("data/locations/nicosia"))));

pub struct StreetAssets {
    pub surfaces: crate::street_surface::StreetSurface,
    pub paths: Vec<Vec<[f32; 3]>>,
    pub checksums: crate::street_surface::SourceChecksums,
}

impl CityAsset {
    pub fn new(directory: PathBuf) -> Self {
        let name = Self::load_name(&directory);
        Self { directory, name }
    }

    pub fn directory(&self) -> &Path {
        &self.directory
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    fn load_name(directory: &Path) -> String {
        std::fs::read_to_string(directory.join("manifest.json"))
            .ok()
            .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
            .and_then(|v| v.get("name").and_then(|n| n.as_str()).map(str::to_owned))
            .unwrap_or_else(|| {
                directory
                    .file_stem()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into_owned()
            })
            .replace(['_', '-'], " ")
            .to_ascii_uppercase()
    }

    pub fn get_manifest(&self) -> Result<serde_json::Value, String> {
        serde_json::from_str(&self.read_text("manifest.json")?)
            .map_err(|e| format!("invalid location manifest: {e}"))
    }

    /// Load the road graph and building footprints; other getters supply its layers.
    pub fn get_city(&self) -> Result<crate::city::City, String> {
        crate::city::City::parse(&self.read_text("city.txt")?)
    }

    pub fn get_streets(&self, city: &crate::city::City) -> Result<StreetAssets, String> {
        let (surfaces, paths, checksums) =
            crate::street_surface::StreetSurface::load_city_with_checksum(
                self.directory(),
                city.source_checksum,
                city.streets.len(),
            )?;
        Ok(StreetAssets {
            surfaces,
            paths,
            checksums,
        })
    }

    // Returns the city's furniture - signs and traffic lights.
    pub fn get_furniture(
        &self,
        checksums: crate::street_surface::SourceChecksums,
    ) -> Result<Vec<crate::traffic::Furniture>, String> {
        // The format is:
        // header:
        // FURNITURE 3 CITY_CRC32 STREETS_CRC32
        // lines:
        // SOURCE_ID KIND X Z DX DZ VALUE PHASE
        // KIND is signal, stop, yield, or speed_limit.
        use crate::traffic::{Furniture, FurnitureKind};
        let text = self.read_text("furniture.txt")?;
        let mut lines = text.lines();
        let header: Vec<_> = lines
            .next()
            .ok_or("missing furniture header")?
            .split_whitespace()
            .collect();
        if header.len() != 4
            || header[0] != "FURNITURE"
            || header[1] != "3"
            || u32::from_str_radix(header[2], 16).ok() != Some(checksums.city)
            || u32::from_str_radix(header[3], 16).ok() != Some(checksums.streets)
        {
            return Err(
                "stale furniture package; re-import the source with add-location under a new name"
                    .into(),
            );
        }
        let mut result = Vec::new();
        for line in lines {
            let f: Vec<_> = line.split_whitespace().collect();
            if f.len() != 8 {
                return Err("invalid road furniture record".into());
            }
            let n = |i: usize| -> Result<u64, String> {
                f[i].parse().map_err(|_| "invalid furniture index".into())
            };
            let v = |i: usize| -> Result<f32, String> {
                let value: f32 = f[i].parse().map_err(|_| "invalid furniture coordinate")?;
                if value.is_finite() {
                    Ok(value)
                } else {
                    Err("nonfinite furniture coordinate".into())
                }
            };
            let kind = match f[1] {
                "signal" => FurnitureKind::Signal,
                "stop" => FurnitureKind::Stop,
                "yield" => FurnitureKind::Yield,
                "speed_limit" => FurnitureKind::Speed,
                _ => return Err("unknown furniture kind".into()),
            };
            let travel = [v(4)?, v(5)?];
            let length = travel[0].hypot(travel[1]);
            let value = n(6)?;
            let phase = n(7)?;
            if (length - 1.0).abs() > 0.001
                || phase >= 50
                || value > 150
                || (kind == FurnitureKind::Speed && value < 5)
            {
                return Err("invalid furniture direction, phase or value".into());
            }
            result.push(Furniture {
                _source_id: n(0)?,
                kind,
                position: [v(2)?, 0.0, v(3)?],
                travel,
                value: value as u16,
                phase: phase as u8,
            });
        }
        Ok(result)
    }

    pub fn get_heightfield(
        &self,
        city: &crate::city::City,
    ) -> Result<crate::elevation::HeightField, String> {
        crate::elevation::HeightField::load_with_checksum(
            &self.read_text("elevation/heightfield.txt")?,
            city.source_checksum,
        )
    }

    pub fn get_landscape(
        &self,
        checksums: crate::street_surface::SourceChecksums,
    ) -> Result<crate::landscape::Landscape, String> {
        use crate::landscape::{GroundTriangle, Landscape, Tree};
        let mut landscape = Landscape::default();
        for (file, imagery) in [("landscape.txt", false), ("imagery-trees.txt", true)] {
            let text = if imagery {
                match std::fs::read_to_string(self.directory.join(file)) {
                    Ok(text) => text,
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => continue,
                    Err(e) => return Err(format!("cannot read imagery trees: {e}")),
                }
            } else {
                self.read_text(file)?
            };
            let mut lines = text.lines();
            let expected = format!(
                "LANDSCAPE 1 {:08x} {:08x}",
                checksums.city, checksums.streets
            );
            if lines.next() != Some(expected.as_str()) {
                return Err(
                    "stale landscape package; re-import the source with add-location under a new name"
                        .into(),
                );
            }
            let mut result = Landscape::default();
            for line in lines {
                let f: Vec<_> = line.split_whitespace().collect();
                let value = |i: usize| -> Result<f32, String> {
                    let v: f32 = f
                        .get(i)
                        .ok_or("missing landscape coordinate")?
                        .parse()
                        .map_err(|_| "invalid landscape number")?;
                    if v.is_finite() && v.abs() < 100_000.0 {
                        Ok(v)
                    } else {
                        Err("invalid landscape coordinate".into())
                    }
                };
                match f.first().copied() {
                    Some("a") if f.len() == 8 => {
                        let style: usize = f[1].parse().map_err(|_| "invalid landscape style")?;
                        if style > 8 {
                            return Err("unknown ground style".into());
                        }
                        result.ground.push(GroundTriangle {
                            style,
                            points: [
                                [value(2)?, value(3)?],
                                [value(4)?, value(5)?],
                                [value(6)?, value(7)?],
                            ],
                        });
                    }
                    Some("t") if f.len() == 6 => {
                        let height = value(4)?;
                        let radius = value(5)?;
                        if !(2.5..=18.0).contains(&height) || !(0.7..=4.5).contains(&radius) {
                            return Err("invalid tree size".into());
                        }
                        result.trees.push(Tree {
                            id: f[1].parse().map_err(|_| "invalid tree id")?,
                            point: [value(2)?, value(3)?],
                            height,
                            radius,
                        });
                    }
                    _ => return Err("invalid landscape record".into()),
                }
            }
            if imagery {
                if !result.ground.is_empty() {
                    return Err("imagery supplement must contain only trees".into());
                }
                landscape.append_imagery_trees(result.trees)?;
            } else {
                landscape = result;
            }
        }
        landscape.index_ground();
        Ok(landscape)
    }

    pub fn get_source_osm(&self) -> std::io::Result<std::fs::File> {
        std::fs::File::open(self.directory.join("source.osm"))
    }

    fn read_text(&self, name: &str) -> Result<String, String> {
        std::fs::read_to_string(self.directory.join(name))
            .map_err(|e| format!("cannot read city asset {name}: {e}"))
    }
}

/// Construct a location before replacing the current global value.
pub fn select(path: PathBuf) -> std::io::Result<()> {
    let assets = CityAsset::new(path.canonicalize()?);
    *CITY_ASSET.write().unwrap() = assets;
    Ok(())
}

#[derive(Clone, Debug)]
pub struct MenuLocation {
    pub name: String,
    pub track: crate::track::TrackId,
    pub path: Option<PathBuf>,
}
/// Discover names and paths without reading map assets or building tracks.
pub fn discover(root: &Path) -> std::io::Result<Vec<MenuLocation>> {
    let mut locations = Vec::new();
    if !root.exists() {
        return Ok(locations);
    }
    let mut paths = std::fs::read_dir(root)?
        .map(|e| e.map(|e| e.path()))
        .collect::<std::io::Result<Vec<_>>>()?;
    paths.sort();
    for path in paths {
        if !path.is_dir() || !path.join("city.txt").is_file() {
            continue;
        }
        let name = path.file_name();
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
    match discover(Path::new(LOCATIONS_DIRECTORY)) {
        Ok(locations) => locations,
        Err(error) => {
            log::warn!("Cannot list locations: {error}");
            Vec::new()
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn locations_are_independent_values() {
        let root = tempfile::tempdir().unwrap();
        let first = root.path().join("first");
        let second = root.path().join("second");
        for (path, name) in [(&first, "First City"), (&second, "Second City")] {
            std::fs::create_dir(path).unwrap();
            let width = if name == "First City" { 6.0 } else { 8.0 };
            std::fs::write(
                path.join("city.txt"),
                format!("CITY 2\nbounds -10 -10 110 10\nn 0 0\nn 100 0\nr 0 1 {width} 0\n"),
            )
            .unwrap();
            std::fs::write(
                path.join("manifest.json"),
                serde_json::json!({"name": name}).to_string(),
            )
            .unwrap();
        }
        let assets = CityAsset::new(first.clone());
        let other = CityAsset::new(second.clone());
        assert_eq!(assets.get_manifest().unwrap()["name"], "First City");
        assert_eq!(assets.name(), "FIRST CITY");
        assert_eq!(other.name(), "SECOND CITY");
        assert_eq!(assets.directory(), first);
        assert_eq!(other.directory(), second);
        assert_eq!(assets.get_city().unwrap().streets[0].width, 6.0);
        assert_eq!(other.get_city().unwrap().streets[0].width, 8.0);
        let replacement = CityAsset::new(second);
        assert_eq!(replacement.name(), "SECOND CITY");
        assert_eq!(assets.name(), "FIRST CITY");
        assert_eq!(assets.get_city().unwrap().streets[0].width, 6.0);
    }

    #[test]
    fn discovery_ignores_authoring_json() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("new.track.json");
        std::fs::write(&path, r#"{"version":2,"name":"New Circuit","points":[[0,0,0,7.2],[100,0,0,7.2],[100,0,100,7.2],[0,0,100,7.2]]}"#).unwrap();
        std::fs::write(root.path().join("broken.track.json"), b"{}").unwrap();
        let entries = discover(root.path()).unwrap();
        assert!(entries.is_empty());
    }

    #[test]
    fn discovery_lists_directories_without_loading_maps() {
        let root = tempfile::tempdir().unwrap();
        // Invalid map contents intentionally prove discovery never decodes them.
        std::fs::write(root.path().join("zebra.sqlite"), b"not a database").unwrap();
        let folder = root.path().join("alpha_city");
        std::fs::create_dir(&folder).unwrap();
        std::fs::write(folder.join("city.txt"), b"not a map").unwrap();
        std::fs::create_dir(root.path().join("empty")).unwrap();
        std::fs::write(root.path().join("notes.txt"), b"ignore").unwrap();
        let locations = discover(root.path()).unwrap();
        assert_eq!(locations.len(), 1);
        assert_eq!(locations[0].name, "ALPHA CITY");
        assert_eq!(locations[0].path.as_deref(), Some(folder.as_path()));
        // Discovery never loads authoring sources at runtime.
        std::fs::write(root.path().join("alpha_city.track.json"), b"invalid source").unwrap();
        assert_eq!(discover(root.path()).unwrap().len(), 1);
        std::fs::write(
            root.path().join("alpha_city.sqlite"),
            b"legacy file is ignored",
        )
        .unwrap();
        let locations = discover(root.path()).unwrap();
        assert_eq!(locations.len(), 1);
        assert_eq!(locations[0].path.as_deref(), Some(folder.as_path()));
        assert!(discover(&root.path().join("missing")).unwrap().is_empty());
    }
}
