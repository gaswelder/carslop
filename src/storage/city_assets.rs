use crate::model;
use crate::model::source_location::SourceLocation;
use crate::storage;
use crate::world;
use std::path::{Path, PathBuf};

pub const LOCATIONS_DIRECTORY: &str = "data/locations";

#[derive(Clone, Debug)]
pub struct CityAsset {
    directory: PathBuf,
    name: String,
    aerial_attribution: Option<String>,
}

pub fn new(directory: PathBuf) -> CityAsset {
    let name = load_name(&directory);
    let aerial_attribution = storage::aerial::attribution_label(&directory);
    CityAsset {
        directory,
        name,
        aerial_attribution,
    }
}

pub fn resolved(assets: &CityAsset) -> std::io::Result<CityAsset> {
    Ok(new(assets.directory.canonicalize()?))
}
pub fn aerial_attribution(assets: &CityAsset) -> Option<&str> {
    assets.aerial_attribution.as_deref()
}

pub fn directory(assets: &CityAsset) -> &Path {
    &assets.directory
}

pub fn name(assets: &CityAsset) -> &str {
    &assets.name
}

fn load_name(directory: &Path) -> String {
    directory
        .file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .into_owned()
}

pub fn get_manifest(assets: &CityAsset) -> Result<storage::manifest::Manifest, String> {
    storage::manifest::read(&assets.directory)
}

/// Load the road graph and building footprints; other getters supply its layers.
pub fn get_city(assets: &CityAsset) -> Result<SourceLocation<f32>, String> {
    let mut city = storage::city::read(&assets.directory)?;
    let manifest = get_manifest(assets)?;
    city.origin = manifest.origin;
    city.bounding_box = manifest.bounds_metres;
    Ok(city)
}

// Returns the city's road controls and barriers.
pub fn get_furniture(assets: &CityAsset) -> Result<storage::furniture::FurnitureData, String> {
    storage::furniture::read(&assets.directory)
}

pub fn get_heightfield(assets: &CityAsset) -> Result<world::elevation::HeightField, String> {
    storage::elevation::read(&assets.directory).map(|elevation| elevation.field)
}

pub fn get_landscape_data(assets: &CityAsset) -> Result<storage::landscape::Data, String> {
    storage::landscape::read_file(&assets.directory)
}

#[derive(Clone, Debug)]
pub struct MenuLocation {
    pub name: String,
    pub track: world::locations::LocationId,
    pub assets: Option<CityAsset>,
}
/// Discover canonical identities and small metadata without decoding world geometry.
fn discover(root: &Path) -> std::io::Result<Vec<MenuLocation>> {
    let mut locations = Vec::new();
    if !root.exists() {
        return Ok(locations);
    }
    let mut paths = std::fs::read_dir(root)?
        .map(|e| e.map(|e| e.path()))
        .collect::<std::io::Result<Vec<_>>>()?;
    paths.sort();
    for path in paths {
        if !path.is_dir() || !storage::city::exists(&path) {
            continue;
        }
        let name = path.file_name();
        let Some(name) = name.and_then(|s| s.to_str()) else {
            continue;
        };
        if name.starts_with('.') {
            continue;
        }
        locations.push(from_assets(new(path))?);
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

pub fn from_assets(assets: CityAsset) -> std::io::Result<MenuLocation> {
    let assets = resolved(&assets)?;
    Ok(MenuLocation {
        name: name(&assets).into(),
        track: model::world::LocationId::Asset(directory(&assets).to_path_buf()),
        assets: Some(assets),
    })
}
