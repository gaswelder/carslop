use crate::model;
use crate::model::source_location::SourceLocation;
use crate::model::source_location::SourceRoad;
use crate::model::world::Building;
use crate::model::world::Entrance;
use crate::storage;
use crate::util;
use crate::util::error::invalid;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use std::collections::BTreeSet;
use std::fmt::Display;
use std::io::Write;

#[derive(Clone, Copy)]
pub enum Precision {
    /// Imported coordinates: 3 decimals; widths/heights: 2 decimals.
    Imported,
    /// Authored tracks retain their original shortest f64 text, without rounding.
    Authored,
}

/// Optional volume metadata follows its owning b record; old records remain valid.
pub fn write_volume<T: Copy + Into<f64> + Display>(
    writer: &mut impl Write,
    index: usize,
    min_height: T,
    holes: &[Vec<XZ<T>>],
    precision: Precision,
) -> util::error::Result<()> {
    if min_height.into() == 0.0 && holes.is_empty() {
        return Ok(());
    }
    let coords = match precision {
        Precision::Imported => Some(3),
        Precision::Authored => None,
    };
    let sizes = match precision {
        Precision::Imported => Some(2),
        Precision::Authored => None,
    };
    write!(
        writer,
        "v {} {} {}",
        index,
        storage::codec_number::new(min_height, sizes),
        holes.len()
    )?;
    for hole in holes {
        write!(writer, " {}", hole.len())?;
        for p in hole {
            write!(
                writer,
                " {} {}",
                storage::codec_number::new(p.x, coords),
                storage::codec_number::new(p.z, coords)
            )?;
        }
    }
    writeln!(writer)?;
    Ok(())
}

pub fn write(
    writer: &mut impl Write,
    map: &SourceLocation<f64>,
    precision: Precision,
) -> util::error::Result<()> {
    let (coords, sizes) = match precision {
        Precision::Imported => (Some(3), Some(2)),
        Precision::Authored => (None, None),
    };
    writeln!(writer, "CITY 2")?;
    for p in &map.road_nodes {
        writeln!(
            writer,
            "n {} {}",
            storage::codec_number::new(p.x, coords),
            storage::codec_number::new(p.z, coords)
        )?;
    }
    for r in &map.road_lines {
        write!(
            writer,
            "r {} {} {} {} {} {} {}",
            r.ends[0],
            r.ends[1],
            storage::codec_number::new(r.road_width_start_meters, sizes),
            r.direction,
            r.osm_way_id
                .ok_or_else(|| invalid("compiler road is missing a way ID"))?,
            r.kind,
            r.lanes
        )?;
        if !r.sidewalks.left || !r.sidewalks.right {
            write!(
                writer,
                " {} {}",
                u8::from(r.sidewalks.left),
                u8::from(r.sidewalks.right)
            )?;
        }
        writeln!(writer)?;
    }
    for (index, b) in map.buildings.iter().enumerate() {
        write!(
            writer,
            "b {} {} {} {}",
            b.id,
            storage::codec_number::new(b.height_meters, sizes),
            u8::from(b.tagged),
            b.ground_outline.len()
        )?;
        for p in &b.ground_outline {
            write!(
                writer,
                " {} {}",
                storage::codec_number::new(p.x, coords),
                storage::codec_number::new(p.z, coords)
            )?;
        }
        writeln!(writer)?;
        write_volume(writer, index, b.min_height_meters, &b.holes, precision)?;
    }
    for entrance in &map.entrances {
        writeln!(
            writer,
            "e {} {} {}",
            entrance.building_id,
            storage::codec_number::new(entrance.position.x, coords),
            storage::codec_number::new(entrance.position.z, coords)
        )?;
    }
    Ok(())
}

// Keep existing optional road metadata, strict CITY header, record validation,
// and f32 parsing at the format boundary.
pub fn parse(text: &str) -> std::result::Result<SourceLocation<f32>, String> {
    let mut city = model::source_location::empty();
    let mut lines = text.lines();
    if !matches!(lines.next(), Some("CITY 2")) {
        return Err("unsupported city format".into());
    }
    for line in lines {
        let fields: Vec<_> = line.split_whitespace().collect();
        let f = |i: usize| -> Result<f32, String> {
            let v: f32 = fields
                .get(i)
                .ok_or("missing coordinate")?
                .parse()
                .map_err(|_| "invalid coordinate")?;
            if v.is_finite() {
                Ok(v)
            } else {
                Err("nonfinite coordinate".into())
            }
        };
        let n = |i: usize| -> Result<usize, String> {
            fields
                .get(i)
                .ok_or("missing index")?
                .parse()
                .map_err(|_| "invalid index".into())
        };
        match fields.first().copied() {
            Some("n") => city.road_nodes.push(XYZ {
                x: f(1)?,
                y: 0.0,
                z: f(2)?,
            }),
            Some("r") => {
                let ends = [n(1)?, n(2)?];
                if ends.iter().any(|i| *i >= city.road_nodes.len()) {
                    return Err("road index out of range".into());
                }
                let a = city.road_nodes[ends[0]];
                let b = city.road_nodes[ends[1]];
                let length = (b.x - a.x).hypot(b.z - a.z);
                let width = f(3)?;
                if length < 0.1 || !(2.0..=30.0).contains(&width) {
                    return Err("invalid road dimensions".into());
                }
                let direction = fields
                    .get(4)
                    .ok_or("missing direction")?
                    .parse::<i8>()
                    .map_err(|_| "invalid direction")?;
                if !(-1..=1).contains(&direction) {
                    return Err("invalid direction".into());
                }
                city.road_lines.push(SourceRoad {
                    ends,
                    osm_way_id: fields.get(5).and_then(|id| id.parse().ok()),
                    road_width_start_meters: width,
                    direction,
                    kind: fields.get(6).map(|_| n(6)).transpose()?.unwrap_or(1),
                    lanes: fields.get(7).map(|_| n(7)).transpose()?.unwrap_or(2),
                    sidewalks: crate::model::source_location::Sidewalks {
                        left: match fields.get(8).copied() {
                            None | Some("1") => true,
                            Some("0") => false,
                            _ => return Err("invalid left sidewalk".into()),
                        },
                        right: match fields.get(9).copied() {
                            None | Some("1") => true,
                            Some("0") => false,
                            _ => return Err("invalid right sidewalk".into()),
                        },
                    },
                });
            }
            Some("v") => {
                if fields.len() < 4 {
                    return Err("invalid building volume".into());
                }
                let index = n(1)?;
                let min_height = f(2)?;
                let count = n(3)?;
                let building = city
                    .buildings
                    .get_mut(index)
                    .ok_or("unknown building volume index")?;
                if min_height < 0.0 || min_height >= building.height_meters {
                    return Err("invalid building minimum height".into());
                }
                let mut cursor = 4;
                let mut holes = Vec::new();
                for _ in 0..count {
                    let count = n(cursor)?;
                    cursor += 1;
                    if count < 3 || count > fields.len().saturating_sub(cursor) / 2 {
                        return Err("invalid building interior ring".into());
                    }
                    let mut hole = Vec::new();
                    for _ in 0..count {
                        hole.push(XZ {
                            x: f(cursor)?,
                            z: f(cursor + 1)?,
                        });
                        cursor += 2;
                    }
                    holes.push(hole);
                }
                if cursor != fields.len() {
                    return Err("invalid building volume fields".into());
                }
                building.min_height_meters = min_height;
                building.holes = holes;
            }
            Some("e") => {
                if fields.len() != 4 {
                    return Err("invalid entrance record".into());
                }
                city.entrances.push(Entrance {
                    building_id: fields[1]
                        .parse()
                        .map_err(|_| "invalid entrance building ID")?,
                    position: XZ { x: f(2)?, z: f(3)? },
                });
            }
            Some("b") => {
                let id = fields
                    .get(1)
                    .ok_or("missing building id")?
                    .parse()
                    .map_err(|_| "invalid building id")?;
                let height = f(2)?;
                let count = n(4)?;
                if count < 3 || fields.len() != 5 + count * 2 || !(2.0..=100.0).contains(&height) {
                    return Err("invalid building dimensions".into());
                }
                let polygon = (0..count)
                    .map(|i| {
                        Ok(XZ {
                            x: f(5 + i * 2)?,
                            z: f(6 + i * 2)?,
                        })
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                city.buildings.push(Building {
                    id,
                    height_meters: height,
                    base_y: 0.0,
                    tagged: fields[3] == "1",
                    holes: Vec::new(),
                    min_height_meters: 0.0,
                    ground_outline: polygon,
                });
            }
            _ => return Err("unknown city record".into()),
        }
    }
    if city.road_lines.is_empty() {
        return Err("empty city".into());
    }
    let buildings: BTreeSet<_> = city.buildings.iter().map(|building| building.id).collect();
    if city
        .entrances
        .iter()
        .any(|entrance| !buildings.contains(&entrance.building_id))
    {
        return Err("entrance references a missing building".into());
    }
    Ok(city)
}
