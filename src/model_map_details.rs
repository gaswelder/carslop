//! Optional mapped appearance, scenery and transport geometry, retained by saved locations.

use serde::{Deserialize, Serialize};
use std::{fs, path::Path};

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
