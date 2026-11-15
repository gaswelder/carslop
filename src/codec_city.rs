use crate::codec_number;
use crate::error;
use crate::error::invalid;
use crate::geometry;
use crate::geometry::{BoundingBoxXZ, XYZ, XZ};
use crate::source_location;
use crate::source_location::{SourceLocation, SourceRoad};
use crate::world::{Building, Entrance};
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
) -> error::Result<()> {
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
        codec_number::new(min_height, sizes),
        holes.len()
    )?;
    for hole in holes {
        write!(writer, " {}", hole.len())?;
        for p in hole {
            write!(
                writer,
                " {} {}",
                codec_number::new(p.x, coords),
                codec_number::new(p.z, coords)
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
) -> error::Result<()> {
    let (coords, sizes) = match precision {
        Precision::Imported => (Some(3), Some(2)),
        Precision::Authored => (None, None),
    };
    writeln!(writer, "CITY 2")?;
    if let Some(origin) = map.origin {
        writeln!(writer, "origin {} {}", origin.lat, origin.lon)?;
    }
    writeln!(
        writer,
        "bounds {} {} {} {}",
        codec_number::new(map.bounding_box.min.x, coords),
        codec_number::new(map.bounding_box.min.z, coords),
        codec_number::new(map.bounding_box.max.x, coords),
        codec_number::new(map.bounding_box.max.z, coords)
    )?;
    for p in &map.road_nodes {
        writeln!(
            writer,
            "n {} {}",
            codec_number::new(p.x, coords),
            codec_number::new(p.z, coords)
        )?;
    }
    for r in &map.road_lines {
        write!(
            writer,
            "r {} {} {} {} {} {} {}",
            r.ends[0],
            r.ends[1],
            codec_number::new(r.road_width_start_meters, sizes),
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
            codec_number::new(b.height_meters, sizes),
            u8::from(b.tagged),
            b.ground_outline.len()
        )?;
        for p in &b.ground_outline {
            write!(
                writer,
                " {} {}",
                codec_number::new(p.x, coords),
                codec_number::new(p.z, coords)
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
            codec_number::new(entrance.position.x, coords),
            codec_number::new(entrance.position.z, coords)
        )?;
    }
    Ok(())
}

// Keep existing optional road metadata, strict CITY header, record validation,
// and f32 parsing at the format boundary.
pub fn parse(text: &str) -> std::result::Result<SourceLocation<f32>, String> {
    let mut city = source_location::empty();
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
            Some("origin") => {
                let latitude: f64 = fields
                    .get(1)
                    .ok_or("missing latitude")?
                    .parse()
                    .map_err(|_| "invalid latitude")?;
                let longitude: f64 = fields
                    .get(2)
                    .ok_or("missing longitude")?
                    .parse()
                    .map_err(|_| "invalid longitude")?;
                if !(-90.0..=90.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude) {
                    return Err("invalid geographic origin".into());
                }
                city.origin = Some(geometry::new(latitude, longitude));
            }
            Some("bounds") => {
                city.bounding_box = BoundingBoxXZ {
                    min: XZ { x: f(1)?, z: f(2)? },
                    max: XZ { x: f(3)?, z: f(4)? },
                }
            }
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
                    sidewalks: crate::source_location::Sidewalks {
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
    if city.road_lines.is_empty()
        || [city.bounding_box.min.x, city.bounding_box.min.z]
            .into_iter()
            .zip([city.bounding_box.max.x, city.bounding_box.max.z])
            .any(|(a, b)| a >= b)
    {
        return Err("empty city or invalid bounds".into());
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

#[cfg(test)]
mod tests {
    use crate::geometry;
    use crate::source_location;

    use super::*;

    fn fixture() -> SourceLocation<f64> {
        let mut map = source_location::empty();
        map.origin = Some(geometry::new(35.123456789, 33.987654321));
        map.bounding_box = BoundingBoxXZ {
            min: XZ {
                x: -10.0004,
                z: -10.0004,
            },
            max: XZ {
                x: 30.0004,
                z: 30.0004,
            },
        };
        map.road_nodes = vec![
            geometry::at_height(
                XZ {
                    x: 0.00049,
                    z: -0.00049,
                },
                0.,
            ),
            geometry::at_height(
                XZ {
                    x: 20.123456,
                    z: 0.,
                },
                0.,
            ),
        ];
        map.road_lines.push(SourceRoad {
            osm_way_id: Some(42),
            direction: -1,
            kind: 0,
            lanes: 3,
            ..source_location::new([0, 1], 6.123456)
        });
        map.buildings.push(Building {
            id: 7,
            height_meters: 7.987654,
            tagged: false,
            holes: Vec::new(),
            min_height_meters: 0.0,
            base_y: 0.,
            ground_outline: vec![
                XZ { x: 1.123456, z: 2. },
                XZ { x: 3., z: 4. },
                XZ { x: 5., z: 6. },
            ],
        });
        map
    }

    #[test]
    fn invalid_building_volume_metadata_is_rejected() {
        let mut bytes = Vec::new();
        write(&mut bytes, &fixture(), Precision::Imported).unwrap();
        let city = std::str::from_utf8(&bytes).unwrap();
        for record in [
            "v 999 0 0",
            "v 0 -1 0",
            "v 0 100 0",
            "v 0 NaN 0",
            "v 0 0 1 2 0 0 1 1",
            "v 0 0 0 extra",
        ] {
            assert!(parse(&format!("{city}{record}\n")).is_err(), "{record}");
        }
    }

    #[test]
    fn entrance_records_round_trip_and_require_an_existing_building() {
        for precision in [Precision::Imported, Precision::Authored] {
            let mut map = fixture();
            map.entrances.push(Entrance {
                building_id: 7,
                position: XZ {
                    x: 1.123456789,
                    z: 2.0,
                },
            });
            let mut bytes = Vec::new();
            write(&mut bytes, &map, precision).unwrap();
            let text = std::str::from_utf8(&bytes).unwrap();
            let city = parse(text).unwrap();
            assert_eq!(city.entrances.len(), 1);
            assert_eq!(city.entrances[0].building_id, 7);
            assert!((city.entrances[0].position.x - 1.1234568).abs() < 0.001);
            for malformed in ["e 999 0 0", "e 7 NaN 0", "e 7 0", "e bad 0 0"] {
                assert!(parse(&format!("{text}{malformed}\n")).is_err());
            }
        }
    }

    #[test]
    fn city_precision_policies_preserve_legacy_text_and_runtime_geometry() {
        for (precision, expected, first_x, width) in [
            (Precision::Imported, concat!(
                "CITY 2\norigin 35.123456789 33.987654321\nbounds -10.000 -10.000 30.000 30.000\n",
                "n 0.000 -0.000\nn 20.123 0.000\nr 0 1 6.12 -1 42 0 3\n",
                "b 7 7.99 0 3 1.123 2.000 3.000 4.000 5.000 6.000\n"), 0.0, 6.12),
            (Precision::Authored, concat!(
                "CITY 2\norigin 35.123456789 33.987654321\nbounds -10.0004 -10.0004 30.0004 30.0004\n",
                "n 0.00049 -0.00049\nn 20.123456 0\nr 0 1 6.123456 -1 42 0 3\n",
                "b 7 7.987654 0 3 1.123456 2 3 4 5 6\n"), 0.00049, 6.123456),
        ] {
            let mut bytes = vec![];
            write(&mut bytes, &fixture(), precision).unwrap();
            assert_eq!(std::str::from_utf8(&bytes).unwrap(), expected);
            let parsed = parse(expected).unwrap();
            assert_eq!(parsed.road_nodes[0].x, first_x);
            assert_eq!(parsed.road_lines[0].road_width_start_meters, width);
            assert_eq!(parsed.road_lines[0].direction, -1);
            assert_eq!(parsed.road_lines[0].osm_way_id, Some(42));
            assert_eq!(parsed.road_lines[0].lanes, 3);
            assert_eq!(parsed.origin, fixture().origin);
            assert!(!parsed.buildings[0].tagged);
        }
    }
}
