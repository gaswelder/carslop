//! Optional mapped appearance, scenery and transport geometry, retained by saved locations.

use crate::model_world::RoadRise;
use crate::util_geometry::{XYZ, XZ};
use crate::world_raised_roads;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, fs, path::Path};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoofShape {
    #[default]
    Flat,
    Gabled,
    Hipped,
    Pyramidal,
    Skillion,
    Dome,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BuildingStyle {
    pub volume: usize,
    pub shape: RoofShape,
    pub roof_height: f32,
    pub angle: f32,
    pub wall_color: Option<[f32; 3]>,
    pub roof_color: Option<[f32; 3]>,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TreeKind {
    #[default]
    Broadleaf,
    Conifer,
    Palm,
    Olive,
    Cypress,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TreeStyle {
    pub id: u64,
    pub kind: TreeKind,
    #[serde(default)]
    pub evergreen: bool,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObjectKind {
    #[default]
    Bench,
    Bin,
    Lamp,
    Shelter,
    Bollard,
    Gate,
    PowerPole,
    PowerTower,
    Monument,
    Planter,
    TurnSign,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapObject {
    pub kind: ObjectKind,
    pub position: [f32; 2],
    pub angle: f32,
    pub height: f32,
    pub width: f32,
    pub text: String,
    pub locked: bool,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LineKind {
    Power,
    Cliff,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneryLine {
    pub kind: LineKind,
    pub points: Vec<[f32; 2]>,
    pub height: f32,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadSurface {
    #[default]
    Asphalt,
    Concrete,
    Paving,
    Unpaved,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Deck {
    pub road: usize,
    pub grade: i8,
    pub width: f32,
    pub path: Vec<[f32; 3]>,
    pub surface: RoadSurface,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TurnRestriction {
    pub from: i64,
    pub via: i64,
    pub to: i64,
    pub rule: String,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Data {
    pub version: u32,
    #[serde(default)]
    pub buildings: Vec<BuildingStyle>,
    #[serde(default)]
    pub trees: Vec<TreeStyle>,
    #[serde(default)]
    pub objects: Vec<MapObject>,
    #[serde(default)]
    pub lines: Vec<SceneryLine>,
    #[serde(default)]
    pub decks: Vec<Deck>,
    #[serde(default)]
    pub restrictions: Vec<TurnRestriction>,
}
impl Default for Data {
    fn default() -> Self {
        Self {
            version: 1,
            buildings: Vec::new(),
            trees: Vec::new(),
            objects: Vec::new(),
            lines: Vec::new(),
            decks: Vec::new(),
            restrictions: Vec::new(),
        }
    }
}

pub fn validate(data: &Data, buildings: usize, roads: usize) -> Result<(), String> {
    let color = |value: Option<[f32; 3]>| {
        value.is_none_or(|color| {
            color
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        })
    };
    let position = |point: &[f32; 2]| point.iter().all(|v| v.is_finite() && v.abs() <= 100_000.0);
    if data.version != 1 {
        return Err("unsupported mapped details version".into());
    }
    for style in &data.buildings {
        if style.volume >= buildings
            || !style.roof_height.is_finite()
            || !(0.0..=30.0).contains(&style.roof_height)
            || !style.angle.is_finite()
            || !color(style.wall_color)
            || !color(style.roof_color)
        {
            return Err("invalid mapped building style".into());
        }
    }
    for object in &data.objects {
        if !position(&object.position)
            || !object.angle.is_finite()
            || !object.height.is_finite()
            || !(0.1..=100.0).contains(&object.height)
            || !object.width.is_finite()
            || !(0.02..=30.0).contains(&object.width)
            || object.text.len() > 512
        {
            return Err("invalid mapped street object".into());
        }
    }
    for line in &data.lines {
        if line.points.len() < 2
            || line.points.iter().any(|p| !position(p))
            || !line.height.is_finite()
            || !(0.1..=100.0).contains(&line.height)
        {
            return Err("invalid mapped scenery line".into());
        }
    }
    for deck in &data.decks {
        if deck.road >= roads
            || deck.grade == 0
            || deck.grade.unsigned_abs() > 10
            || !deck.width.is_finite()
            || !(2.0..=30.0).contains(&deck.width)
            || deck.path.len() < 2
            || deck.path.iter().any(|p| {
                p.iter().any(|v| !v.is_finite())
                    || p[0].abs() > 100_000.0
                    || p[2].abs() > 100_000.0
                    || p[1].abs() > 10_000.0
            })
            || deck
                .path
                .windows(2)
                .any(|p| (p[1][0] - p[0][0]).hypot(p[1][2] - p[0][2]) < 0.0001)
        {
            return Err("invalid mapped bridge/tunnel".into());
        }
    }
    Ok(())
}
pub fn read(directory: &Path, buildings: usize, roads: usize) -> Result<Data, String> {
    let data: Data = match fs::read(directory.join("details.json")) {
        Ok(bytes) => {
            serde_json::from_slice(&bytes).map_err(|error| format!("details.json: {error}"))?
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(Data::default()),
        Err(error) => return Err(error.to_string()),
    };
    validate(&data, buildings, roads)?;
    Ok(data)
}
pub fn write(data: &Data, directory: &Path) -> Result<(), String> {
    fs::write(
        directory.join("details.json"),
        serde_json::to_vec(data).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())
}

#[derive(Debug)]
struct IndexedObject {
    index: usize,
    bounds: rstar::AABB<[f32; 2]>,
}
impl rstar::RTreeObject for IndexedObject {
    type Envelope = rstar::AABB<[f32; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}
#[derive(Debug)]
pub struct Prepared {
    data: Data,
    buildings: HashMap<usize, usize>,
    trees: HashMap<u64, TreeStyle>,
    objects: rstar::RTree<IndexedObject>,
    decks: rstar::RTree<IndexedObject>,
    deck_roads: HashMap<usize, usize>,
}

pub fn new(data: Data) -> Prepared {
    let buildings = data
        .buildings
        .iter()
        .enumerate()
        .map(|(i, style)| (style.volume, i))
        .collect();
    let trees = data
        .trees
        .iter()
        .map(|style| (style.id, style.clone()))
        .collect();
    let objects = rstar::RTree::bulk_load(
        data.objects
            .iter()
            .enumerate()
            .map(|(index, object)| {
                let reach = object.width / 2. + 3.;
                IndexedObject {
                    index,
                    bounds: rstar::AABB::from_corners(
                        object.position.map(|v| v - reach),
                        object.position.map(|v| v + reach),
                    ),
                }
            })
            .collect(),
    );
    let decks = rstar::RTree::bulk_load(
        data.decks
            .iter()
            .enumerate()
            .map(|(index, deck)| {
                let reach = deck.width / 2. + 3.;
                let min = [
                    deck.path.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min) - reach,
                    deck.path.iter().map(|p| p[2]).fold(f32::INFINITY, f32::min) - reach,
                ];
                let max = [
                    deck.path
                        .iter()
                        .map(|p| p[0])
                        .fold(f32::NEG_INFINITY, f32::max)
                        + reach,
                    deck.path
                        .iter()
                        .map(|p| p[2])
                        .fold(f32::NEG_INFINITY, f32::max)
                        + reach,
                ];
                IndexedObject {
                    index,
                    bounds: rstar::AABB::from_corners(min, max),
                }
            })
            .collect(),
    );
    let deck_roads = data
        .decks
        .iter()
        .enumerate()
        .map(|(i, deck)| (deck.road, i))
        .collect();
    Prepared {
        data,
        buildings,
        decks,
        deck_roads,
        trees,
        objects,
    }
}
pub fn road_deck(prepared: &Prepared, road: usize) -> Option<&Deck> {
    prepared
        .deck_roads
        .get(&road)
        .map(|i| &prepared.data.decks[*i])
}
pub fn nearby_decks(prepared: &Prepared, p: XZ<f32>) -> impl Iterator<Item = &Deck> {
    prepared
        .decks
        .locate_in_envelope_intersecting(&rstar::AABB::from_point([p.x, p.z]))
        .map(|item| &prepared.data.decks[item.index])
}
pub fn height_on(deck: &Deck, p: XZ<f32>) -> Option<f32> {
    deck_triangles(deck)
        .into_iter()
        .filter_map(|triangle| world_raised_roads::road_rise_offset(&triangle, p))
        .max_by(f32::total_cmp)
}
pub fn deck_height(prepared: &Prepared, p: XZ<f32>, reference: f32) -> Option<f32> {
    nearby_decks(prepared, p)
        .filter_map(|deck| height_on(deck, p))
        .min_by(|a, b| (a - reference).abs().total_cmp(&(b - reference).abs()))
}
pub fn nearby_objects(prepared: &Prepared, p: XZ<f32>) -> impl Iterator<Item = &MapObject> {
    prepared
        .objects
        .locate_in_envelope_intersecting(&rstar::AABB::from_point([p.x, p.z]))
        .map(|item| &prepared.data.objects[item.index])
}
pub fn data(prepared: &Prepared) -> &Data {
    &prepared.data
}
pub fn building(prepared: &Prepared, index: usize) -> Option<&BuildingStyle> {
    prepared
        .buildings
        .get(&index)
        .map(|i| &prepared.data.buildings[*i])
}
pub fn tree(prepared: &Prepared, id: u64) -> Option<&TreeStyle> {
    prepared.trees.get(&id)
}

/// Shared deck footprint and height planes for contact and scene construction.
pub fn deck_triangles(deck: &Deck) -> Vec<RoadRise> {
    let points: Vec<_> = deck
        .path
        .iter()
        .map(|p| XYZ {
            x: p[0],
            y: p[1],
            z: p[2],
        })
        .collect();
    let sections: Vec<_> = points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let a = points[i.saturating_sub(1)];
            let b = points[(i + 1).min(points.len() - 1)];
            let length = (b.x - a.x).hypot(b.z - a.z).max(0.0001);
            let normal = XZ {
                x: -(b.z - a.z) / length,
                z: (b.x - a.x) / length,
            };
            [-1., 1.].map(|side| XYZ {
                x: p.x + normal.x * side * deck.width / 2.,
                y: p.y,
                z: p.z + normal.z * side * deck.width / 2.,
            })
        })
        .collect();
    sections
        .windows(2)
        .flat_map(|p| {
            [
                RoadRise {
                    points: [p[0][0], p[0][1], p[1][1]],
                },
                RoadRise {
                    points: [p[0][0], p[1][1], p[1][0]],
                },
            ]
        })
        .collect()
}
