//! Optional mapped appearance, scenery and transport geometry, retained by saved locations.

use crate::model::world::Material;
use std::fmt;
use std::fs;
use std::io;
use std::io::Write;
use std::path::Path;
use std::str::FromStr;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum RoofShape {
    #[default]
    Flat,
    Gabled,
    Hipped,
    Pyramidal,
    Skillion,
    Dome,
}
#[derive(Clone, Debug, PartialEq)]
pub struct BuildingStyle {
    pub volume: usize,
    pub shape: RoofShape,
    pub roof_height: f32,
    pub angle: f32,
    pub wall_color: Option<[f32; 3]>,
    pub roof_color: Option<[f32; 3]>,
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum TreeKind {
    #[default]
    Broadleaf,
    Conifer,
    Palm,
    Olive,
    Cypress,
}
#[derive(Clone, Debug, PartialEq)]
pub struct TreeStyle {
    pub id: u64,
    pub kind: TreeKind,
    pub evergreen: bool,
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
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
#[derive(Clone, Debug, PartialEq)]
pub struct MapObject {
    pub kind: ObjectKind,
    pub position: [f32; 2],
    pub angle: f32,
    pub height: f32,
    pub width: f32,
    pub text: String,
    pub locked: bool,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LineKind {
    Power,
    Cliff,
}
#[derive(Clone, Debug, PartialEq)]
pub struct SceneryLine {
    pub kind: LineKind,
    pub points: Vec<[f32; 2]>,
    pub height: f32,
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum RoadSurface {
    #[default]
    Asphalt,
    Concrete,
    Paving,
    Unpaved,
}

pub fn road_material(surface: RoadSurface) -> Material {
    match surface {
        RoadSurface::Asphalt => Material::Asphalt,
        RoadSurface::Concrete => Material::Concrete,
        RoadSurface::Paving => Material::Paving,
        RoadSurface::Unpaved => Material::BareGround,
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Deck {
    pub road: usize,
    pub grade: i8,
    pub width: f32,
    pub path: Vec<[f32; 3]>,
    pub surface: RoadSurface,
}
#[derive(Clone, Debug, PartialEq)]
pub struct TurnRestriction {
    pub from: i64,
    pub via: i64,
    pub to: i64,
    pub rule: String,
}
#[derive(Clone, Debug, PartialEq)]
pub struct Data {
    pub version: u32,
    pub buildings: Vec<BuildingStyle>,
    pub trees: Vec<TreeStyle>,
    pub objects: Vec<MapObject>,
    pub lines: Vec<SceneryLine>,
    pub decks: Vec<Deck>,
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
// Keep the on-disk enum names explicit and independent of Rust's Debug output.
macro_rules! text_enum {
    ($type:ty, $($variant:ident => $name:literal),+ $(,)?) => {
        impl fmt::Display for $type {
            fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str(match self { $(Self::$variant => $name),+ })
            }
        }
        impl FromStr for $type {
            type Err = String;
            fn from_str(text: &str) -> Result<Self, Self::Err> {
                match text {
                    $($name => Ok(Self::$variant),)+
                    _ => Err(format!("unknown {}: {text}", stringify!($type))),
                }
            }
        }
    };
}
text_enum!(RoofShape, Flat => "flat", Gabled => "gabled", Hipped => "hipped",
    Pyramidal => "pyramidal", Skillion => "skillion", Dome => "dome");
text_enum!(TreeKind, Broadleaf => "broadleaf", Conifer => "conifer", Palm => "palm",
    Olive => "olive", Cypress => "cypress");
text_enum!(ObjectKind, Bench => "bench", Bin => "bin", Lamp => "lamp", Shelter => "shelter",
    Bollard => "bollard", Gate => "gate", PowerPole => "power_pole", PowerTower => "power_tower",
    Monument => "monument", Planter => "planter", TurnSign => "turn_sign");
text_enum!(LineKind, Power => "power", Cliff => "cliff");
text_enum!(RoadSurface, Asphalt => "asphalt", Concrete => "concrete", Paving => "paving",
    Unpaved => "unpaved");

pub fn read(directory: &Path, buildings: usize, roads: usize) -> Result<Data, String> {
    let text = match fs::read_to_string(directory.join("details.txt")) {
        Ok(text) => text,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Data::default()),
        Err(error) => return Err(format!("details.txt: {error}")),
    };
    let data = parse(&text)?;
    validate(&data, buildings, roads).map_err(|error| format!("details.txt: {error}"))?;
    Ok(data)
}

pub fn write(data: &Data, directory: &Path) -> Result<(), String> {
    let mut writer = io::BufWriter::new(
        fs::File::create(directory.join("details.txt")).map_err(|error| error.to_string())?,
    );
    write_records(data, &mut writer).map_err(|error| error.to_string())
}

fn write_records(data: &Data, writer: &mut impl Write) -> io::Result<()> {
    writeln!(writer, "DETAILS {}", data.version)?;
    for building in &data.buildings {
        write!(
            writer,
            "building {} {} {} {}",
            building.volume, building.shape, building.roof_height, building.angle
        )?;
        for color in [building.wall_color, building.roof_color] {
            match color {
                Some([r, g, b]) => write!(writer, " {r} {g} {b}")?,
                None => write!(writer, " -")?,
            }
        }
        writeln!(writer)?;
    }
    for tree in &data.trees {
        writeln!(writer, "tree {} {} {}", tree.id, tree.kind, tree.evergreen)?;
    }
    for object in &data.objects {
        writeln!(
            writer,
            "object {} {} {} {} {} {} {} {}",
            object.kind,
            object.position[0],
            object.position[1],
            object.angle,
            object.height,
            object.width,
            object.locked,
            quote(&object.text)
        )?;
    }
    for line in &data.lines {
        write!(
            writer,
            "line {} {} {}",
            line.kind,
            line.height,
            line.points.len()
        )?;
        for [x, z] in &line.points {
            write!(writer, " {x} {z}")?;
        }
        writeln!(writer)?;
    }
    for deck in &data.decks {
        write!(
            writer,
            "deck {} {} {} {} {}",
            deck.road,
            deck.grade,
            deck.width,
            deck.surface,
            deck.path.len()
        )?;
        for [x, y, z] in &deck.path {
            write!(writer, " {x} {y} {z}")?;
        }
        writeln!(writer)?;
    }
    for restriction in &data.restrictions {
        writeln!(
            writer,
            "restriction {} {} {} {}",
            restriction.from,
            restriction.via,
            restriction.to,
            quote(&restriction.rule)
        )?;
    }
    writer.flush()
}

fn quote(text: &str) -> String {
    let mut result = String::from("\"");
    for character in text.chars() {
        match character {
            '"' => result.push_str("\\\""),
            '\\' => result.push_str("\\\\"),
            '\n' => result.push_str("\\n"),
            '\r' => result.push_str("\\r"),
            '\t' => result.push_str("\\t"),
            _ => result.push(character),
        }
    }
    result.push('"');
    result
}

fn tokens(line: &str) -> Result<Vec<String>, String> {
    let mut characters = line.chars().peekable();
    let mut result = Vec::new();
    while let Some(character) = characters.next() {
        if character.is_whitespace() {
            continue;
        }
        let mut token = String::new();
        if character == '"' {
            loop {
                match characters.next().ok_or("unterminated quoted text")? {
                    '"' => break,
                    '\\' => token.push(match characters.next().ok_or("incomplete escape")? {
                        'n' => '\n',
                        'r' => '\r',
                        't' => '\t',
                        '"' => '"',
                        '\\' => '\\',
                        _ => return Err("unknown text escape".into()),
                    }),
                    value => token.push(value),
                }
            }
            if characters
                .peek()
                .is_some_and(|value| !value.is_whitespace())
            {
                return Err("expected whitespace after quoted text".into());
            }
        } else {
            token.push(character);
            while characters
                .peek()
                .is_some_and(|value| !value.is_whitespace())
            {
                let value = characters.next().unwrap();
                if value == '"' {
                    return Err("quote inside unquoted field".into());
                }
                token.push(value);
            }
        }
        result.push(token);
    }
    Ok(result)
}

fn value<T: FromStr>(fields: &mut std::slice::Iter<'_, String>) -> Result<T, String> {
    let field = fields.next().ok_or("missing field")?;
    field.parse().map_err(|_| format!("invalid field: {field}"))
}

fn color(fields: &mut std::slice::Iter<'_, String>) -> Result<Option<[f32; 3]>, String> {
    if fields.as_slice().first().is_some_and(|field| field == "-") {
        fields.next();
        Ok(None)
    } else {
        Ok(Some([value(fields)?, value(fields)?, value(fields)?]))
    }
}

fn points<const N: usize>(
    fields: &mut std::slice::Iter<'_, String>,
) -> Result<Vec<[f32; N]>, String> {
    let count: usize = value(fields)?;
    if count.checked_mul(N) != Some(fields.len()) {
        return Err("point count does not match coordinates".into());
    }
    let mut points = Vec::new();
    for _ in 0..count {
        let mut point = [0.; N];
        for coordinate in &mut point {
            *coordinate = value(fields)?;
        }
        points.push(point);
    }
    Ok(points)
}

fn parse(text: &str) -> Result<Data, String> {
    let mut lines = text.lines();
    if lines.next() != Some("DETAILS 1") {
        return Err("details.txt: unsupported or missing DETAILS 1 header".into());
    }
    let mut data = Data::default();
    for (index, line) in lines.enumerate() {
        let parsed = (|| -> Result<(), String> {
            let tokens = tokens(line)?;
            let mut fields = tokens.iter();
            let Some(kind) = fields.next() else {
                return Ok(());
            };
            match kind.as_str() {
                "building" => data.buildings.push(BuildingStyle {
                    volume: value(&mut fields)?,
                    shape: value(&mut fields)?,
                    roof_height: value(&mut fields)?,
                    angle: value(&mut fields)?,
                    wall_color: color(&mut fields)?,
                    roof_color: color(&mut fields)?,
                }),
                "tree" => data.trees.push(TreeStyle {
                    id: value(&mut fields)?,
                    kind: value(&mut fields)?,
                    evergreen: value(&mut fields)?,
                }),
                "object" => data.objects.push(MapObject {
                    kind: value(&mut fields)?,
                    position: [value(&mut fields)?, value(&mut fields)?],
                    angle: value(&mut fields)?,
                    height: value(&mut fields)?,
                    width: value(&mut fields)?,
                    locked: value(&mut fields)?,
                    text: value(&mut fields)?,
                }),
                "line" => data.lines.push(SceneryLine {
                    kind: value(&mut fields)?,
                    height: value(&mut fields)?,
                    points: points(&mut fields)?,
                }),
                "deck" => data.decks.push(Deck {
                    road: value(&mut fields)?,
                    grade: value(&mut fields)?,
                    width: value(&mut fields)?,
                    surface: value(&mut fields)?,
                    path: points(&mut fields)?,
                }),
                "restriction" => data.restrictions.push(TurnRestriction {
                    from: value(&mut fields)?,
                    via: value(&mut fields)?,
                    to: value(&mut fields)?,
                    rule: value(&mut fields)?,
                }),
                _ => return Err(format!("unknown record kind: {kind}")),
            }
            if fields.next().is_some() {
                return Err("unexpected extra fields".into());
            }
            Ok(())
        })();
        parsed.map_err(|error| format!("details.txt:{}: {error}", index + 2))?;
    }
    Ok(data)
}

#[cfg(test)]
#[path = "tests/map_details.rs"]
mod tests;
