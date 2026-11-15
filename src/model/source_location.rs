//! Compiler geometry and parsed CITY records, without runtime layers or indexes.
//! Importers and generators keep f64 geometry. CITY parsing deliberately retains
//! the existing f32 quantization; writers retain their existing decimal precision.
use crate::model::world::Building;
use crate::model::world::Entrance;
use crate::storage;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::LatLon;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use geo::{Intersects, MultiPolygon, Point};

#[derive(Debug)]
pub struct SourceLocation<T = f64> {
    pub bounding_box: BoundingBoxXZ<T>,
    pub origin: Option<LatLon>,
    pub boundary: Option<MultiPolygon<f64>>,
    pub road_nodes: Vec<XYZ<T>>,
    pub road_lines: Vec<SourceRoad<T>>,
    pub buildings: Vec<Building<T>>,
    pub entrances: Vec<Entrance<T>>,
}

/// Road metadata at source precision; path lengths and indexes belong to preparation.
#[derive(Clone, Debug)]
pub struct SourceRoad<T = f64> {
    pub ends: [usize; 2],
    pub osm_way_id: Option<i64>,
    pub road_width_start_meters: T,
    pub direction: i8,
    pub kind: usize,
    pub lanes: usize,
    /// Generated pavement sides relative to the original way direction.
    pub sidewalks: Sidewalks,
}

#[derive(Clone, Copy, Debug)]
pub struct Sidewalks {
    pub left: bool,
    pub right: bool,
}

impl Default for Sidewalks {
    fn default() -> Self {
        Self {
            left: true,
            right: true,
        }
    }
}

pub fn new<T: Copy>(ends: [usize; 2], width: T) -> SourceRoad<T> {
    SourceRoad::<T> {
        ends,
        osm_way_id: None,
        road_width_start_meters: width,
        direction: 0,
        kind: 1,
        lanes: 2,
        sidewalks: Sidewalks::default(),
    }
}

pub fn empty<T: Default>() -> SourceLocation<T> {
    SourceLocation::<T> {
        bounding_box: BoundingBoxXZ::default(),
        origin: None,
        boundary: None,
        road_nodes: Vec::new(),
        road_lines: Vec::new(),
        buildings: Vec::new(),
        entrances: Vec::new(),
    }
}

/// Project a geographic position into local metres; requires a geographic origin.
pub fn metres(source_location: &SourceLocation<f64>, p: LatLon) -> XZ {
    let origin = source_location
        .origin
        .expect("geographic projection requires a location origin");
    XZ {
        x: (p.lon - origin.lon).to_radians() * 6371000.0 * origin.lat.to_radians().cos(),
        z: -(p.lat - origin.lat).to_radians() * 6371000.0,
    }
}
pub fn contains(source_location: &SourceLocation<f64>, p: XZ) -> bool {
    p.x >= source_location.bounding_box.min.x
        && p.x <= source_location.bounding_box.max.x
        && p.z >= source_location.bounding_box.min.z
        && p.z <= source_location.bounding_box.max.z
        && source_location
            .boundary
            .as_ref()
            .is_none_or(|b| b.intersects(&Point::new(p.x, p.z)))
}

pub fn parse(text: &str) -> Result<SourceLocation<f32>, String> {
    storage::codec_city::parse(text)
}
