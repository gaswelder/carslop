use crate::model;
use crate::model::location;
use crate::storage;
use crate::util;
use std::{
    fs,
    io::BufWriter,
    path::{Path, PathBuf},
};

/// References to immutable, already compiled payloads, not a source/build folder.
/// These remain disk-backed to avoid retaining a whole city scene or tile set.
#[derive(Debug, Default)]
pub struct Media {
    scene: Option<Payload>,
    aerial: Vec<Payload>,
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

pub fn location_read(dirpath: impl AsRef<Path>) -> Result<location::Location, String> {
    let directory = dirpath.as_ref();
    read_inner(directory).map_err(|e| format!("location {}: {e}", directory.display()))
}

fn read_inner(directory: &Path) -> Result<location::Location, String> {
    let assets = storage::city_assets::new(directory.to_path_buf());
    let furniture = storage::city_assets::get_furniture(&assets)?;
    let source = storage::city_assets::get_city(&assets)?;
    let streets = storage::streets::load_geometry(directory, source.road_lines.len())
        .map_err(|e| format!("streets.bin: {e}"))?;
    let manifest = storage::city_assets::get_manifest(&assets)?;
    let pavement_height_meters = manifest.pavement_height_metres;
    let landscape = storage::city_assets::get_landscape_data(&assets)?;
    let details =
        model::map_details::read(directory, source.buildings.len(), source.road_lines.len())?;
    let mut media = Media::default();
    if directory.join("scene.pack").exists() {
        media.scene = Some(payload_read(directory, "scene.pack".into())?);
    }
    // Keep only the catalog and tiles used by the renderer.
    // Service requests and download/build metadata are not location data.
    media.aerial = storage::aerial::read_payload_files(directory)?
        .into_iter()
        .map(|file| payload_read(directory, file))
        .collect::<Result<_, _>>()?;
    Ok(location::Location {
        aerial_attribution: manifest.aerial_attribution,
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
            .map(|r| location::Road {
                ends: r.ends,
                way_id: r.osm_way_id,
                width_meters: r.road_width_start_meters,
                direction: r.direction,
            })
            .collect(),
        buildings: source
            .buildings
            .into_iter()
            .map(|b| location::Building {
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
            .map_err(|e| format!("elevation.txt: {e}"))?,
        ground: landscape.ground,
        trees: landscape.trees,
        furniture: furniture
            .controls
            .into_iter()
            .map(|f| location::Furniture {
                id: f._source_id,
                kind: f.kind,
                position: util::geometry::xz(f.coords),
                travel: f.travel,
                value: f.value,
                phase: f.phase,
            })
            .collect(),
        barriers: furniture.barriers,
        road_rises: match fs::read_to_string(directory.join("road-rises.txt")) {
            Ok(text) => {
                storage::road_rises::parse(&text).map_err(|e| format!("road-rises.txt: {e}"))?
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
            Err(e) => return Err(format!("road-rises.txt: {e}")),
        },
        labels: storage::labels::read_optional(directory)?,
        details,
        media,
    })
}

pub fn location_write(
    location: &location::Location,
    dirpath: impl AsRef<Path>,
) -> Result<(), String> {
    let directory = dirpath.as_ref();
    model::map_details::validate(
        &location.details,
        location.buildings.len(),
        location.roads.len(),
    )?;
    write_inner(location, directory).map_err(|e| format!("location {}: {e}", directory.display()))
}

fn write_inner(location: &location::Location, directory: &Path) -> Result<(), String> {
    fs::create_dir_all(directory).map_err(|e| e.to_string())?;
    let write = || -> util::error::Result<()> {
        storage::city::write_location(directory, location)?;
        storage::manifest::write(
            directory,
            &storage::manifest::Manifest {
                origin: location.origin,
                bounds_metres: location.extent,
                pavement_height_metres: location.pavement_height_meters,
                aerial_attribution: location.aerial_attribution.clone(),
            },
        )?;
        storage::labels::write(directory, &location.labels)?;
        storage::elevation::write(directory, &location.elevation)?;
        let mut landscape =
            storage::landscape::create(directory, storage::landscape::Precision::Authored)?;
        for t in &location.ground {
            if t.usage != model::world::TriangleUsage::Ground {
                return Err(util::error::invalid(
                    "ground layer contains street geometry",
                ));
            }
            storage::landscape::ground(&mut landscape, t.material, t.points)?;
        }
        for t in &location.trees {
            storage::landscape::tree(&mut landscape, t)?;
        }
        storage::landscape::flush(&mut landscape)?;
        storage::furniture::write_location(directory, location)?;
        storage::road_rises::write(
            &mut BufWriter::new(fs::File::create(directory.join("road-rises.txt"))?),
            &location.road_rises,
        )?;
        storage::streets::write_compiled(directory, &location.streets)?;
        Ok(())
    };
    write().map_err(|e| e.to_string())?;
    // Imagery trees are merged into the ordinary vegetation layer on read.
    storage::landscape::remove_imagery_trees(directory)?;
    if let Some(scene) = &location.media.scene {
        payload_write(scene, directory)?;
    } else {
        remove_optional(&directory.join("scene.pack"))?;
    }
    // Remove only files referenced by the previous renderer catalog.
    let retained = location
        .media
        .aerial
        .iter()
        .map(|payload| payload.relative.clone())
        .collect::<Vec<_>>();
    storage::aerial::remove_stale_payloads(directory, &retained)?;
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

#[cfg(test)]
#[path = "tests/location.rs"]
mod tests;
