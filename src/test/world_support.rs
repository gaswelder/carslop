//! Shared world fixtures. Private world and road state stays inside its owner.

use super::*;
use crate::build;
use crate::model;
use crate::storage;
use crate::world;

pub fn empty_world() -> WorldLocation {
    model::world::empty()
}

pub fn parse_world(text: &str) -> Result<WorldLocation, String> {
    world::roads::world_location_from_source(model::source_location::parse(text)?)
}

pub fn landscape(text: &str) -> Result<WorldLocation, String> {
    let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
    std::fs::write(folder.path().join("landscape.txt"), text).map_err(|e| e.to_string())?;
    let (ground, trees) = storage::city_assets::get_landscape_data(&storage::city_assets::new(
        folder.path().to_path_buf(),
    ))?;
    let mut world = empty_world();
    world.ground = world::landscape::new(ground, trees);
    Ok(world)
}

pub fn set_elevation(world: &mut WorldLocation, field: Option<HeightField>) {
    world.elevation = field;
}

pub fn install_surfaces(world: &mut WorldLocation, surfaces: world::street_surface::StreetSurface) {
    world.roads.geometry = RoadGeometry::Compiled(surfaces);
}

pub fn flat_track(world: &WorldLocation) -> WorldLocation {
    let mut track = build::authoring::circuit_build(
        &build::authoring::Circuit {
            control_points: FLAT_POINTS
                .map(|p| build::authoring::ControlPoint {
                    position: XYZ { y: 0.0, ..p },
                    width: world::locations::at_distance(world, 50.0).width,
                })
                .to_vec(),
            coastal: false,
        },
        world.id.clone(),
    )
    .unwrap();
    track.elevation = None;
    track
}

pub fn bumpy_track(world: &WorldLocation) -> WorldLocation {
    let mut track = flat_track(world);
    world::roads::bump(&mut track.roads);
    track
}
const FLAT_POINTS: [XYZ; 12] = [
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
