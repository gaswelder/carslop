//! Shared world fixtures. Private world and road state stays inside its owner.
use super::*;
use crate::authoring;
use crate::city_assets::CityAsset;
use crate::landscape;
use crate::source_location::SourceLocation;
use crate::street_surface;

pub fn empty_world() -> Location {
    Location::empty()
}

pub fn parse_world(text: &str) -> Result<Location, String> {
    Location::from_source(SourceLocation::<f32>::parse(text)?)
}

pub fn landscape(text: &str) -> Result<Location, String> {
    let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
    std::fs::write(folder.path().join("landscape.txt"), text).map_err(|e| e.to_string())?;
    let (ground, trees) = CityAsset::new(folder.path().to_path_buf()).get_landscape_data()?;
    let mut world = empty_world();
    world.ground = landscape::GroundCover::new(ground, trees);
    Ok(world)
}

pub fn set_elevation(world: &mut Location, field: Option<HeightField>) {
    world.elevation = field;
}

pub fn install_surfaces(world: &mut Location, surfaces: street_surface::StreetSurface) {
    world.roads.geometry = RoadGeometry::Compiled(surfaces);
}

pub fn flat_track(world: &Location) -> Location {
    let mut track = authoring::Circuit {
        control_points: FLAT_POINTS
            .map(|p| authoring::ControlPoint {
                position: XYZ { y: 0.0, ..p },
                width: world.at_distance(50.0).width,
            })
            .to_vec(),
        coastal: false,
    }
    .build(world.id.clone())
    .unwrap();
    track.elevation = None;
    track
}

pub fn bumpy_track(world: &Location) -> Location {
    let mut track = flat_track(world);
    track.roads.bump();
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
