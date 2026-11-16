//! Immutable authored routes. No global mutable "active track".

use crate::build;
use crate::model;
pub use crate::model::world::LocationId;
pub use crate::model::world::RoadSample;
pub use crate::model::world::WorldLocation;
use crate::render;
use crate::storage;
use crate::util::geometry::XYZ;
use crate::world;

// Authoring and test sampling enumerate streets in storage order. This is
// deliberately separate from gameplay navigation, which uses nearby roads.

/// Total horizontal distance of sampled streets, derived for generation and tests.
pub fn sampling_length(world: &WorldLocation) -> f32 {
    world::roads::road_lines(world)
        .last()
        .map_or(0.0, |street| street.distance + street.length)
}

/// Sample streets in storage order, wrapping at the end for authoring loops.
pub fn at_distance(world: &WorldLocation, distance: f32) -> RoadSample {
    let d = distance.rem_euclid(sampling_length(world));
    let i = world::roads::road_lines(world)
        .partition_point(|street| street.distance <= d)
        .saturating_sub(1);
    world::roads::sample_distance(world, i, d - world::roads::road_lines(world)[i].distance)
}

use std::sync::OnceLock;

pub const ROAD_WIDTH: f32 = 7.2;
pub const CAR_RADIUS: f32 = 1.4;
pub const OFFROAD_MARGIN: f32 = 75.0;
pub const TERRAIN_PADDING: f32 = 100.0;

struct RuntimeTrack {
    path: std::path::PathBuf,
    name: &'static str,
    circuit: build::authoring::Circuit,
}
fn runtime_tracks() -> &'static std::sync::RwLock<Vec<std::sync::Arc<RuntimeTrack>>> {
    static TRACKS: OnceLock<std::sync::RwLock<Vec<std::sync::Arc<RuntimeTrack>>>> = OnceLock::new();
    TRACKS.get_or_init(Default::default)
}
/// Register a closed circuit with [x, y, z, width] control points in metres.
/// Version 2 uses per-point full road widths, each between 2 and 30 metres.
/// The compiler samples the spline and generates location assets.
/// Optional `scenery` selects `standard` (default) or `coastal` terrain and roadside scenery.
pub fn register_file(path: &std::path::Path) -> Result<(LocationId, &'static str), String> {
    let path = path.canonicalize().map_err(|e| e.to_string())?;
    let mut tracks = runtime_tracks().write().unwrap();
    if let Some(index) = tracks.iter().position(|t| t.path == path) {
        return Ok((LocationId::File(index), tracks[index].name));
    }
    let data = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
    let v: serde_json::Value = serde_json::from_str(&data).map_err(|e| e.to_string())?;
    if v["version"].as_u64() != Some(2) {
        return Err("track version must be 2".into());
    }
    let name = v["name"]
        .as_str()
        .filter(|s| !s.trim().is_empty() && s.chars().count() <= 80)
        .ok_or("track needs a name of 1..80 characters")?;
    if v.get("width").is_some() {
        return Err("put width in each [x, y, z, width] point".into());
    }
    let coastal = match v.get("scenery").and_then(serde_json::Value::as_str) {
        None if v.get("scenery").is_none() => false,
        Some("standard") => false,
        Some("coastal") => true,
        _ => return Err("track scenery must be standard or coastal".into()),
    };
    let points: Vec<[f32; 4]> =
        serde_json::from_value(v["points"].clone()).map_err(|e| e.to_string())?;
    let circuit = build::authoring::Circuit {
        control_points: points
            .into_iter()
            .map(|[x, y, z, width]| build::authoring::ControlPoint {
                position: XYZ { x, y, z },
                width,
            })
            .collect(),
        coastal,
    };
    build::authoring::validate(&circuit)?;
    let name = Box::leak(name.trim().to_owned().into_boxed_str());
    let index = tracks.len();
    tracks.push(std::sync::Arc::new(RuntimeTrack {
        path,
        name,
        circuit,
    }));
    Ok((LocationId::File(index), name))
}
// Preserve the original compact route and its sampling exactly.
const ROAD_POINTS: [XYZ; 12] = [
    XYZ {
        x: 0.0,
        y: 0.0,
        z: -35.0,
    },
    XYZ {
        x: 14.0,
        y: 1.0,
        z: -30.0,
    },
    XYZ {
        x: 25.0,
        y: 2.0,
        z: -18.0,
    },
    XYZ {
        x: 30.0,
        y: 3.0,
        z: -2.0,
    },
    XYZ {
        x: 24.0,
        y: 2.0,
        z: 14.0,
    },
    XYZ {
        x: 10.0,
        y: 1.0,
        z: 27.0,
    },
    XYZ {
        x: -8.0,
        y: 0.0,
        z: 34.0,
    },
    XYZ {
        x: -24.0,
        y: 2.0,
        z: 25.0,
    },
    XYZ {
        x: -31.0,
        y: 1.0,
        z: 10.0,
    },
    XYZ {
        x: -27.0,
        y: 0.0,
        z: -8.0,
    },
    XYZ {
        x: -16.0,
        y: -1.0,
        z: -24.0,
    },
    XYZ {
        x: -4.0,
        y: 0.0,
        z: -34.0,
    },
];

pub fn asset(path: impl AsRef<std::path::Path>) -> Result<LocationId, String> {
    path.as_ref()
        .canonicalize()
        .map(LocationId::Asset)
        .map_err(|e| format!("cannot resolve location {}: {e}", path.as_ref().display()))
}
pub fn asset_path(location_id: &LocationId) -> Option<&std::path::Path> {
    match location_id {
        LocationId::Asset(path) => Some(path),
        _ => None,
    }
}

pub fn try_track(location_id: &LocationId) -> Result<std::sync::Arc<WorldLocation>, String> {
    use std::sync::Arc;
    Ok(match location_id {
        LocationId::File(index) => {
            let definition = runtime_tracks()
                .read()
                .unwrap()
                .get(*index)
                .cloned()
                .ok_or("unknown runtime track")?;
            let mut track =
                build::authoring::circuit_build(&definition.circuit, location_id.clone())?;
            track.display_name = definition.name.to_owned();
            Arc::new(track)
        }

        LocationId::Compact => Arc::new(build::authoring::world_location_build(
            location_id.clone(),
            &ROAD_POINTS,
        )),
        LocationId::Asset(path) => Arc::new(world::preparation::world_location_load(
            &storage::city_assets::new(path.clone()),
        )?),
    })
}

// Compact-route fixtures retained for existing regression tests, never selected globally.

/// Rendering preset belongs to the authoring definition, not the runtime location.
pub fn coastal_preset(id: &LocationId) -> bool {
    match id {
        LocationId::File(index) => runtime_tracks()
            .read()
            .unwrap()
            .get(*index)
            .is_some_and(|definition| definition.circuit.coastal),
        _ => false,
    }
}

/// Application-owned cache. Only the latest selected world is retained here;
/// sessions keep their own Arcs. Files at a path are immutable until invalidate.
#[derive(Clone, Default)]
pub struct LocationCache {
    current: Option<std::sync::Arc<WorldLocation>>,
}

/// Prepare a candidate without changing the installed cache or active session.
fn prepare(
    location_cache: &LocationCache,
    id: &LocationId,
) -> Result<std::sync::Arc<WorldLocation>, String> {
    let world = match location_cache
        .current
        .as_ref()
        .filter(|world| &world.id == id)
    {
        Some(world) => world.clone(),
        None => try_track(id)?,
    };
    Ok(world)
}
pub fn prepare_for_render(
    location_cache: &LocationCache,
    id: &LocationId,
) -> Result<std::sync::Arc<WorldLocation>, String> {
    let world = prepare(location_cache, id)?;
    if let Some(assets) = model::world::assets(&world) {
        render::scene_archive::open_for(assets).map_err(|e| {
            format!(
                "cannot prepare scene {}: {e}",
                storage::city_assets::directory(assets).display()
            )
        })?;
    }
    Ok(world)
}
pub fn install(location_cache: &mut LocationCache, world: std::sync::Arc<WorldLocation>) {
    invalidate(location_cache);
    location_cache.current = Some(world);
}
/// Call together with eviction of this world's GPU mesh after rebuilding files.
fn invalidate(location_cache: &mut LocationCache) {
    location_cache.current = None;
}
