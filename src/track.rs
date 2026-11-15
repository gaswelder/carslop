//! Immutable authored routes. No global mutable "active track".

use crate::authoring;
use crate::city_assets;
use crate::geometry::XYZ;
use crate::scene_archive;
use crate::world;
pub use crate::world::{LocationId, RoadSample, WorldLocation};
use crate::world_preparation;
use crate::world_roads;

// Authoring and test sampling enumerate streets in storage order. This is
// deliberately separate from gameplay navigation, which uses nearby roads.

/// Total horizontal distance of sampled streets, derived for generation and tests.
pub fn sampling_length(world: &WorldLocation) -> f32 {
    world_roads::road_lines(world)
        .last()
        .map_or(0.0, |street| street.distance + street.length)
}

/// Sample streets in storage order, wrapping at the end for authoring loops.
pub fn at_distance(world: &WorldLocation, distance: f32) -> RoadSample {
    let d = distance.rem_euclid(sampling_length(world));
    let i = world_roads::road_lines(world)
        .partition_point(|street| street.distance <= d)
        .saturating_sub(1);
    world_roads::sample_distance(world, i, d - world_roads::road_lines(world)[i].distance)
}

use std::sync::OnceLock;

pub const ROAD_WIDTH: f32 = 7.2;
pub const CAR_RADIUS: f32 = 1.4;
pub const OFFROAD_MARGIN: f32 = 75.0;
pub const TERRAIN_PADDING: f32 = 100.0;

struct RuntimeTrack {
    path: std::path::PathBuf,
    name: &'static str,
    circuit: authoring::Circuit,
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
    let circuit = authoring::Circuit {
        control_points: points
            .into_iter()
            .map(|[x, y, z, width]| authoring::ControlPoint {
                position: XYZ { x, y, z },
                width,
            })
            .collect(),
        coastal,
    };
    authoring::validate(&circuit)?;
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
            let mut track = authoring::circuit_build(&definition.circuit, location_id.clone())?;
            track.display_name = definition.name.to_owned();
            Arc::new(track)
        }

        LocationId::Compact => Arc::new(authoring::world_location_build(
            location_id.clone(),
            &ROAD_POINTS,
        )),
        LocationId::Asset(path) => Arc::new(world_preparation::world_location_load(
            &city_assets::new(path.clone()),
        )?),
    })
}

// Compact-route fixtures retained for existing regression tests, never selected globally.

#[cfg(test)]
mod tests {
    use crate::track;
    use crate::vehicle_location;
    use crate::world_roads;

    use crate::test_support;

    fn variable_width_track() -> std::sync::Arc<super::WorldLocation> {
        let file = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(file.path(), r#"{"version":2,"name":"Variable width","scenery":"coastal","points":[[0,0,0,4],[400,0,0,12],[400,0,400,6],[0,0,400,8]]}"#).unwrap();
        track::try_track(&super::register_file(file.path()).unwrap().0).unwrap()
    }

    #[test]
    fn point_widths_interpolate_and_agree_with_projection_including_the_seam() {
        let track = variable_width_track();
        for (point, width) in [
            ([0.0, 0.0, 0.0], 4.0),
            ([400.0, 0.0, 0.0], 12.0),
            ([400.0, 0.0, 400.0], 6.0),
            ([0.0, 0.0, 400.0], 8.0),
            ([200.0, 0.0, -50.0], 8.0),
        ] {
            assert!((world_roads::project(&track, (point).into()).width - width).abs() < 0.001);
        }
        for i in 0..1000 {
            let sample =
                track::at_distance(&track, track::sampling_length(&track) * i as f32 / 1000.0);
            assert!((4.0..=12.0).contains(&sample.width));
            let projected = world_roads::project(&track, sample.point);
            assert!((sample.width - projected.width).abs() < 0.001);
        }
        assert_eq!(track::at_distance(&track, 0.0).width, 4.0);
        assert_eq!(
            track::at_distance(&track, track::sampling_length(&track)).width,
            4.0
        );
        assert!(
            (track::at_distance(&track, track::sampling_length(&track) - 0.01).width - 4.0).abs()
                < 0.001
        );
    }

    #[test]
    fn per_point_widths_are_required_and_bounded() {
        for width in [
            serde_json::json!(1.99),
            serde_json::json!(30.01),
            serde_json::Value::Null,
            serde_json::json!("wide"),
        ] {
            let file = tempfile::NamedTempFile::new().unwrap();
            let data = serde_json::json!({"version":2,"name":"Invalid width","points":[[0,0,0,width],[100,0,0,8],[100,0,100,8],[0,0,100,8]]});
            std::fs::write(file.path(), serde_json::to_vec(&data).unwrap()).unwrap();
            assert!(super::register_file(file.path()).is_err());
        }
        for data in [
            r#"{"version":1,"name":"Old format","width":8,"points":[[0,0,0],[100,0,0],[100,0,100],[0,0,100]]}"#,
            r#"{"version":2,"name":"Missing widths","points":[[0,0,0],[100,0,0],[100,0,100],[0,0,100]]}"#,
            r#"{"version":2,"name":"Global width","width":8,"points":[[0,0,0,8],[100,0,0,8],[100,0,100,8],[0,0,100,8]]}"#,
        ] {
            let file = tempfile::NamedTempFile::new().unwrap();
            std::fs::write(file.path(), data).unwrap();
            assert!(super::register_file(file.path()).is_err());
        }
    }

    #[test]
    fn runtime_circuit_rejects_invalid_definitions() {
        for data in [
            r#"{"version":2}"#,
            r#"{"version":2,"name":"bad","scenery":"unknown"}"#,
            r#"{"version":2,"name":"bad","scenery":null}"#,
            r#"{"version":2,"name":"bad","points":[],"width":"wide"}"#,
            r#"{"version":2,"name":"bad","points":[]}"#,
            r#"{"version":2,"name":"bad","points":[[0,0,0,7.2],[1,0,0,7.2],[1,0,1,7.2]]}"#,
            r#"{"version":2,"name":"bad","points":[[0,0,0,7.2],[0,1,0,7.2],[1,0,1,7.2],[2,0,2,7.2]]}"#,
            r#"{"version":2,"name":"bad","points":[[0,0,0,7.2],[20000,0,0,7.2],[20000,0,20000,7.2],[0,0,20000,7.2]]}"#,
        ] {
            let file = tempfile::NamedTempFile::new().unwrap();
            std::fs::write(file.path(), data).unwrap();
            assert!(super::register_file(file.path()).is_err());
        }
    }

    use super::*;

    #[test]
    fn spatial_projection_matches_full_search_and_route_caches_are_isolated() {
        let scenic = track::try_track(&test_support::authored_id()).unwrap();
        for i in 0..400 {
            let s = track::at_distance(&scenic, track::sampling_length(&scenic) * i as f32 / 400.0);
            for offset in [-45.0, -3.0, 0.0, 3.0, 45.0] {
                let p = vehicle_location::vehicle_position(s, offset);
                let indexed = world_roads::project(&scenic, p);
                let exact = (0..world_roads::road_lines(&scenic).len())
                    .map(|i| world_roads::projected(&scenic, i, p))
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .unwrap()
                    .1;
                assert!(
                    (indexed.distance - exact.distance).abs() < 0.001,
                    "point {p:?}: indexed {indexed:?}, brute force {exact:?}"
                );
                assert!((indexed.point.y - exact.point.y).abs() < 0.001);
            }
        }
        let compact_length =
            track::sampling_length(&track::try_track(&LocationId::Compact).unwrap());
        assert!(track::sampling_length(&scenic) > compact_length * 7.0);
        assert_eq!(
            track::sampling_length(&track::try_track(&(LocationId::Compact)).unwrap()),
            compact_length
        );
    }
}

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
    if let Some(assets) = world::assets(&world) {
        scene_archive::open_for(assets).map_err(|e| {
            format!(
                "cannot prepare scene {}: {e}",
                city_assets::directory(assets).display()
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
