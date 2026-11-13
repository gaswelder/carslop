//! Shared world fixtures. Private world and road state stays inside its owner.
use super::*;
pub use crate::cars::test_support::*;
use crate::city_assets::CityAsset;
use crate::source_location::{SourceLocation, SourceRoad};
use std::path::Path;

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
    world.ground = crate::landscape::GroundCover::new(ground, trees);
    Ok(world)
}

pub fn set_elevation(world: &mut Location, field: Option<HeightField>) {
    world.elevation = field;
}

pub fn install_surfaces(world: &mut Location, surfaces: crate::street_surface::StreetSurface) {
    world.roads.geometry = RoadGeometry::Compiled(surfaces);
}

pub fn flat_track(world: &Location) -> Location {
    let mut track = crate::authoring::Circuit {
        control_points: FLAT_POINTS
            .map(|p| crate::authoring::ControlPoint {
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
pub fn stored_location(directory: &Path, populated: bool) -> CityAsset {
    let mut source = SourceLocation::<f64>::empty();
    source.origin = Some(LatLon::new(34.7, 33.0));
    source.bounding_box = BoundingBoxXZ {
        min: XZ { x: -10.0, z: -10.0 },
        max: XZ { x: 30.0, z: 30.0 },
    };
    source.road_nodes = vec![
        XZ { x: 0.0, z: 0.0 }.at_height(0.0),
        XZ { x: 20.0, z: 0.0 }.at_height(0.0),
    ];
    let mut road = SourceRoad::new([0, 1], 8.0);
    road.osm_way_id = Some(1);
    source.road_lines.push(road);
    if populated {
        source.buildings.push(Building {
            id: 1,
            height_meters: 5.0,
            base_y: 0.0,
            tagged: true,
            ground_outline: vec![
                XZ { x: 10.0, z: 10.0 },
                XZ { x: 15.0, z: 10.0 },
                XZ { x: 15.0, z: 15.0 },
                XZ { x: 10.0, z: 15.0 },
            ],
        });
    }
    let parsed = crate::src_files::write_city_txt(&source, &directory.join("city.txt"))
        .unwrap()
        .city;
    std::fs::write(directory.join("manifest.json"), r#"{"name":"Fixture"}"#).unwrap();
    // The bent compiled path differs from the CITY endpoints.
    crate::build_collision::build_collisions(
            &directory.join("streets.bin"), &parsed,
            "STREETS 1 1\np 0 3 0 0 10 1 20 0\nt 1 0 -4 20 -4 20 4\nt 1 0 -4 20 4 0 4\nc 1 0 -4 20 -4 0 1\n".into(),
        ).unwrap();
    std::fs::create_dir(directory.join("elevation")).unwrap();
    std::fs::write(
        directory.join("elevation/heightfield.txt"),
        "ELEVATION 1\ngrid 2 2 -10 -10 40 0\n7 7\n7 7\n",
    )
    .unwrap();
    std::fs::write(
        directory.join("landscape.txt"),
        if populated {
            "LANDSCAPE 1\na 0 -8 10 -3 10 -3 15\nt 1 -5 20 6 1\n"
        } else {
            "LANDSCAPE 1\n"
        },
    )
    .unwrap();
    std::fs::write(
        directory.join("furniture.txt"),
        if populated {
            "FURNITURE 3\n1 stop 5 5 1 0 0 0\n"
        } else {
            "FURNITURE 3\n"
        },
    )
    .unwrap();
    if populated {
        std::fs::write(
            directory.join("imagery-trees.txt"),
            "LANDSCAPE 1\nt 9223372036854775808 25 25 5 1\n",
        )
        .unwrap();
    }
    CityAsset::new(directory.to_path_buf())
}
