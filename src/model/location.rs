//! Playable location records. Persistence belongs to storage.

use crate::model;
use crate::model::world::{Curb, FurnitureKind, HeightField, Tree, Triangle};
use crate::storage;
use crate::util::geometry::{BoundingBoxXZ, LatLon, XZ};

#[derive(Debug)]
pub struct Location {
    pub name: String,
    pub origin: Option<LatLon>,
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
    pub media: storage::location::Media,
    pub aerial_attribution: Option<String>,
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

#[derive(Debug, Default, PartialEq)]
pub struct Labels {
    pub streets: Vec<(i64, String)>,
    pub buildings: Vec<(u64, String)>,
    pub storefronts: Vec<(u64, String)>,
}
