//! The persisted, playable location. No spatial indexes, terrain fitting, OSM
//! objects, compiler provenance, or derived query bounds live in these records.

use crate::model;
use crate::model::world::Curb;
use crate::model::world::FurnitureKind;
use crate::model::world::HeightField;
use crate::model::world::Tree;
use crate::model::world::Triangle;
use crate::storage;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::LatLon;
use crate::util::geometry::XZ;
use crate::world;
use std::{
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

#[derive(Debug)]
pub struct Location {
    pub name: String,
    pub origin: Option<LatLon>,
    /// Explicit authored extent from CITY; query bounds are derived by the game.
    pub extent: BoundingBoxXZ<f32>,
    pub road_nodes: Vec<XZ<f32>>,
    pub roads: Vec<Road>,
    pub buildings: Vec<Building>,
    pub entrances: Vec<model::world::Entrance>,
    pub streets: Streets,
    pub pavement_height_meters: f32,
    pub elevation: HeightField,
    pub ground: Vec<Triangle>,
    pub trees: Vec<Tree>,
    pub furniture: Vec<Furniture>,
    pub barriers: Vec<model::world::Barrier>,
    pub road_rises: Vec<model::world::RoadRise>,
    pub labels: Labels,
    pub details: model::map_details::Data,
    /// Large render payloads stay on disk and are streamed when writing.
    media: Media,
}

#[derive(Debug, PartialEq)]
pub struct Road {
    pub ends: [usize; 2],
    pub way_id: Option<i64>,
    pub width_meters: f32,
    pub direction: i8,
}

#[derive(Debug, PartialEq)]
pub struct Building {
    pub id: u64,
    pub outline: Vec<XZ<f32>>,
    pub holes: Vec<Vec<XZ<f32>>>,
    pub min_height_meters: f32,
    pub height_meters: f32,
}

#[derive(Debug, PartialEq)]
pub struct Furniture {
    pub id: u64,
    pub kind: FurnitureKind,
    pub position: XZ<f32>,
    pub travel: XZ<f32>,
    pub value: u16,
    pub phase: u8,
}

#[derive(Debug, PartialEq)]
pub struct Streets {
    pub paths: Vec<Vec<XZ<f32>>>,
    pub triangles: Vec<Triangle>,
    pub curbs: Vec<Curb>,
}

#[derive(Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Labels {
    pub streets: Vec<(i64, String)>,
    pub buildings: Vec<(u64, String)>,
    #[serde(default)]
    pub storefronts: Vec<(u64, String)>,
}

/// References to immutable, already compiled payloads, not a source/build folder.
/// These remain disk-backed to avoid retaining a whole city scene or tile set.
#[derive(Debug, Default)]
struct Media {
    scene: Option<Payload>,
    aerial: Vec<Payload>,
    // Own temporary compiled payloads until the location is dropped.
    workspace: Option<tempfile::TempDir>,
}
#[derive(Debug)]
struct Payload {
    source: PathBuf,
    relative: PathBuf,
}

fn payload_read(root: &Path, relative: PathBuf) -> Result<Payload, String> {
    let source = root
        .join(&relative)
        .canonicalize()
        .map_err(|e| format!("{}: {e}", relative.display()))?;
    Ok(Payload { source, relative })
}
fn payload_write(payload: &Payload, root: &Path) -> Result<(), String> {
    let target = root.join(&payload.relative);
    if target.canonicalize().ok().as_ref() == Some(&payload.source) {
        return Ok(());
    }
    if let Some(parent) = target.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    fs::copy(&payload.source, target).map_err(|e| e.to_string())?;
    Ok(())
}

pub fn from_compilation(workspace: tempfile::TempDir) -> Result<Location, String> {
    let mut location = location_read(workspace.path())?;
    location.media.workspace = Some(workspace);
    Ok(location)
}

pub fn location_read(dirpath: impl AsRef<Path>) -> Result<Location, String> {
    let directory = dirpath.as_ref();
    read_inner(directory).map_err(|e| format!("location {}: {e}", directory.display()))
}

fn read_inner(directory: &Path) -> Result<Location, String> {
    let assets = storage::city_assets::new(directory.to_path_buf());
    let source = storage::city_assets::get_city(&assets).map_err(|e| format!("city.txt: {e}"))?;
    let streets = storage::street_surface_binary::load_geometry(directory, source.road_lines.len())
        .map_err(|e| format!("streets.bin: {e}"))?;
    let manifest =
        storage::city_assets::get_manifest(&assets).map_err(|e| format!("manifest.json: {e}"))?;
    let pavement_height_meters = match manifest.get("pavement_height_metres") {
        Some(v) => v
            .as_f64()
            .filter(|h| (0.08..=0.25).contains(h))
            .ok_or("manifest.json: pavement_height_metres must be between 0.08 and 0.25")?
            as f32,
        None => world::street_surface::DEFAULT_PAVEMENT_HEIGHT,
    };
    let (ground, trees) = storage::city_assets::get_landscape_data(&assets)?;
    let details =
        model::map_details::read(directory, source.buildings.len(), source.road_lines.len())?;
    let mut media = Media::default();
    if directory.join("scene.pack").exists() {
        media.scene = Some(payload_read(directory, "scene.pack".into())?);
    }
    // Keep only the catalog, tiles, and attribution used by the renderer.
    // Service requests and download/build metadata are not location data.
    if directory.join("aerial/tiles.txt").is_file() {
        let catalog = storage::aerial::tile_filenames(
            &fs::read_to_string(directory.join("aerial/tiles.txt")).map_err(|e| e.to_string())?,
        )?;
        let mut files = vec![PathBuf::from("aerial/tiles.txt")];
        for name in catalog {
            files.push(PathBuf::from("aerial").join(name));
        }
        if directory.join("aerial/source.json").exists() {
            files.push("aerial/source.json".into());
        }
        files.sort();
        files.dedup();
        media.aerial = files
            .into_iter()
            .map(|file| payload_read(directory, file))
            .collect::<Result<_, _>>()?;
    }
    Ok(Location {
        name: storage::city_assets::name(&assets).to_owned(),
        origin: source.origin,
        extent: source.bounding_box,
        road_nodes: source
            .road_nodes
            .into_iter()
            .map(|p| util::geometry::xz(p))
            .collect(),
        roads: source
            .road_lines
            .into_iter()
            .map(|r| Road {
                ends: r.ends,
                way_id: r.osm_way_id,
                width_meters: r.road_width_start_meters,
                direction: r.direction,
            })
            .collect(),
        buildings: source
            .buildings
            .into_iter()
            .map(|b| Building {
                id: b.id,
                outline: b.ground_outline,
                holes: b.holes,
                min_height_meters: b.min_height_meters,
                height_meters: b.height_meters,
            })
            .collect(),
        entrances: source.entrances,
        streets,
        pavement_height_meters,
        elevation: storage::city_assets::get_heightfield(&assets)
            .map_err(|e| format!("elevation/heightfield.txt: {e}"))?,
        ground,
        trees,
        furniture: storage::city_assets::get_furniture(&assets)
            .map_err(|e| format!("furniture.txt: {e}"))?
            .into_iter()
            .map(|f| Furniture {
                id: f._source_id,
                kind: f.kind,
                position: util::geometry::xz(f.coords),
                travel: f.travel,
                value: f.value,
                phase: f.phase,
            })
            .collect(),
        barriers: match fs::read_to_string(directory.join("barriers.txt")) {
            Ok(text) => {
                storage::codec_barriers::parse(&text).map_err(|e| format!("barriers.txt: {e}"))?
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
            Err(e) => return Err(format!("barriers.txt: {e}")),
        },
        road_rises: match fs::read_to_string(directory.join("road-rises.txt")) {
            Ok(text) => storage::codec_road_rises::parse(&text)
                .map_err(|e| format!("road-rises.txt: {e}"))?,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
            Err(e) => return Err(format!("road-rises.txt: {e}")),
        },
        labels: read_labels(directory)?,
        details,
        media,
    })
}

pub fn location_write(location: &Location, dirpath: impl AsRef<Path>) -> Result<(), String> {
    let directory = dirpath.as_ref();
    model::map_details::validate(
        &location.details,
        location.buildings.len(),
        location.roads.len(),
    )?;
    write_inner(location, directory).map_err(|e| format!("location {}: {e}", directory.display()))
}

fn write_inner(location: &Location, directory: &Path) -> Result<(), String> {
    fs::create_dir_all(directory.join("elevation")).map_err(|e| e.to_string())?;
    let write = || -> util::error::Result<()> {
        let mut city = BufWriter::new(fs::File::create(directory.join("city.txt"))?);
        writeln!(city, "CITY 2")?;
        if let Some(p) = location.origin {
            writeln!(city, "origin {} {}", p.lat, p.lon)?;
        }
        writeln!(
            city,
            "bounds {} {} {} {}",
            location.extent.min.x,
            location.extent.min.z,
            location.extent.max.x,
            location.extent.max.z
        )?;
        for p in &location.road_nodes {
            writeln!(city, "n {} {}", p.x, p.z)?;
        }
        for r in &location.roads {
            writeln!(
                city,
                "r {} {} {} {} {}",
                r.ends[0],
                r.ends[1],
                r.width_meters,
                r.direction,
                r.way_id
                    .map(|id| id.to_string())
                    .unwrap_or_else(|| "-".into())
            )?;
        }
        for (index, b) in location.buildings.iter().enumerate() {
            write!(city, "b {} {} 0 {}", b.id, b.height_meters, b.outline.len())?;
            for p in &b.outline {
                write!(city, " {} {}", p.x, p.z)?;
            }
            writeln!(city)?;
            storage::codec_city::write_volume(
                &mut city,
                index,
                b.min_height_meters,
                &b.holes,
                storage::codec_city::Precision::Authored,
            )?;
        }
        for entrance in &location.entrances {
            writeln!(
                city,
                "e {} {} {}",
                entrance.building_id, entrance.position.x, entrance.position.z,
            )?;
        }
        city.flush()?;
        fs::write(
            directory.join("manifest.json"),
            serde_json::to_vec_pretty(&serde_json::json!({
                "pavement_height_metres": location.pavement_height_meters,
            }))?,
        )?;
        fs::write(
            directory.join("labels.json"),
            serde_json::to_vec(&location.labels)?,
        )?;
        // Serialize at full f32 precision; compiler writers deliberately quantize.
        let mut elevation = BufWriter::new(fs::File::create(
            directory.join("elevation/heightfield.txt"),
        )?);
        let f = &location.elevation;
        writeln!(
            elevation,
            "ELEVATION 1\ngrid {} {} {} {} {} 0",
            f.columns, f.rows, f.origin.x, f.origin.z, f.step
        )?;
        for row in f.values.chunks(f.columns) {
            for v in row {
                write!(elevation, "{v} ")?;
            }
            writeln!(elevation)?;
        }
        elevation.flush()?;
        let mut landscape = storage::codec_landscape::new(
            BufWriter::new(fs::File::create(directory.join("landscape.txt"))?),
            storage::codec_landscape::Precision::Authored,
        )?;
        for t in &location.ground {
            if t.usage != model::world::TriangleUsage::Ground {
                return Err(util::error::invalid(
                    "ground layer contains street geometry",
                ));
            }
            storage::codec_landscape::ground(&mut landscape, t.material, t.points)?;
        }
        for t in &location.trees {
            storage::codec_landscape::tree(&mut landscape, t)?;
        }
        storage::codec_landscape::flush(&mut landscape)?;
        let mut furniture = BufWriter::new(fs::File::create(directory.join("furniture.txt"))?);
        writeln!(furniture, "FURNITURE 3")?;
        for f in &location.furniture {
            let kind = match f.kind {
                FurnitureKind::Signal => "signal",
                FurnitureKind::Stop => "stop",
                FurnitureKind::Yield => "yield",
                FurnitureKind::Speed => "speed_limit",
            };
            writeln!(
                furniture,
                "{} {kind} {} {} {} {} {} {}",
                f.id, f.position.x, f.position.z, f.travel.x, f.travel.z, f.value, f.phase
            )?;
        }
        furniture.flush()?;
        storage::codec_barriers::write(
            &mut BufWriter::new(fs::File::create(directory.join("barriers.txt"))?),
            &location.barriers,
        )?;
        storage::codec_road_rises::write(
            &mut BufWriter::new(fs::File::create(directory.join("road-rises.txt"))?),
            &location.road_rises,
        )?;
        storage::street_surface_binary::write_geometry(
            &directory.join("streets.bin"),
            &location.streets,
        )?;
        Ok(())
    };
    write().map_err(|e| e.to_string())?;
    // Imagery trees are merged into the ordinary vegetation layer on read.
    remove_optional(&directory.join("imagery-trees.txt"))?;
    if let Some(scene) = &location.media.scene {
        payload_write(scene, directory)?;
    } else {
        remove_optional(&directory.join("scene.pack"))?;
    }
    // Remove only files referenced by the previous renderer catalog.
    let previous_tiles = fs::read_to_string(directory.join("aerial/tiles.txt"))
        .ok()
        .and_then(|text| storage::aerial::tile_filenames(&text).ok())
        .unwrap_or_default();
    let previous_files = previous_tiles
        .into_iter()
        .map(|name| PathBuf::from("aerial").join(name))
        .chain([
            PathBuf::from("aerial/tiles.txt"),
            PathBuf::from("aerial/source.json"),
        ]);
    for relative in previous_files {
        if !location.media.aerial.iter().any(|p| p.relative == relative) {
            remove_optional(&directory.join(relative))?;
        }
    }
    model::map_details::write(&location.details, directory)?;
    for payload in &location.media.aerial {
        payload_write(payload, directory)?;
    }
    Ok(())
}

fn remove_optional(path: &Path) -> Result<(), String> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

fn read_labels(directory: &Path) -> Result<Labels, String> {
    match fs::read(directory.join("labels.json")) {
        Ok(bytes) => serde_json::from_slice(&bytes).map_err(|e| format!("labels.json: {e}")),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            storage::legacy_labels::read_legacy_labels(directory)
        }
        Err(e) => Err(format!("labels.json: {e}")),
    }
}
