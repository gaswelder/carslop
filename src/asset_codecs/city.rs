//! CITY 2: f64 compiler input, deliberately f32 parsed geometry.
use super::Number;
use crate::error::invalid;
use crate::geometry::{BoundingBoxXZ, LatLon, XYZ, XZ};
use crate::source_location::{SourceLocation, SourceRoad};
use crate::world::Building;
use std::io::Write;

#[derive(Clone, Copy)]
pub enum Precision {
    /// Imported coordinates: 3 decimals; widths/heights: 2 decimals.
    Imported,
    /// Authored tracks retain their original shortest f64 text, without rounding.
    Authored,
}

pub fn write(
    writer: &mut impl Write,
    map: &SourceLocation<f64>,
    precision: Precision,
) -> crate::error::Result<()> {
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
        Number(map.bounding_box.min.x, coords),
        Number(map.bounding_box.min.z, coords),
        Number(map.bounding_box.max.x, coords),
        Number(map.bounding_box.max.z, coords)
    )?;
    for p in &map.road_nodes {
        writeln!(writer, "n {} {}", Number(p.x, coords), Number(p.z, coords))?;
    }
    for r in &map.road_lines {
        writeln!(
            writer,
            "r {} {} {} {} {} {} {}",
            r.ends[0],
            r.ends[1],
            Number(r.road_width_start_meters, sizes),
            r.direction,
            r.osm_way_id
                .ok_or_else(|| invalid("compiler road is missing a way ID"))?,
            r.kind,
            r.lanes
        )?;
    }
    for b in &map.buildings {
        write!(
            writer,
            "b {} {} {} {}",
            b.id,
            Number(b.height_meters, sizes),
            u8::from(b.tagged),
            b.ground_outline.len()
        )?;
        for p in &b.ground_outline {
            write!(writer, " {} {}", Number(p.x, coords), Number(p.z, coords))?;
        }
        writeln!(writer)?;
    }
    Ok(())
}

// Keep existing optional road metadata, strict CITY header, record validation,
// and f32 parsing at the format boundary.
pub fn parse(text: &str) -> std::result::Result<SourceLocation<f32>, String> {
    let mut city = SourceLocation::empty();
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
                city.origin = Some(LatLon::new(latitude, longitude));
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
    Ok(city)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> SourceLocation<f64> {
        let mut map = SourceLocation::empty();
        map.origin = Some(LatLon::new(35.123456789, 33.987654321));
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
            XZ {
                x: 0.00049,
                z: -0.00049,
            }
            .at_height(0.),
            XZ {
                x: 20.123456,
                z: 0.,
            }
            .at_height(0.),
        ];
        map.road_lines.push(SourceRoad {
            osm_way_id: Some(42),
            direction: -1,
            kind: 0,
            lanes: 3,
            ..SourceRoad::new([0, 1], 6.123456)
        });
        map.buildings.push(Building {
            id: 7,
            height_meters: 7.987654,
            tagged: false,
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
