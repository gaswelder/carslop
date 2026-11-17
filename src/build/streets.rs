//! Builder-owned streets.txt intermediates, including their text format.

use crate::model;
use crate::model::world::Curb;
use crate::model::world::CurbStyle;
use crate::model::world::Material;
use crate::model::world::Triangle;
use crate::model::world::TriangleUsage;
use crate::util;
use crate::util::error;
use crate::util::geometry::XZ;
use crate::world::street_surface::cross;

const FILE_NAME: &str = "streets.txt";

fn parse(streets_txt_source: &str, count: usize) -> Result<model::location::Streets, String> {
    let mut lines123213 = streets_txt_source.lines();
    let header: Vec<_> = lines123213
        .next()
        .ok_or("missing surfaces header")?
        .split_whitespace()
        .collect();
    if header.len() < 3
        || header[0] != "STREETS"
        || header[1] != "1"
        || header[2].parse::<usize>().ok() != Some(count)
    {
        return Err("unsupported street surfaces header or road count".into());
    }
    let mut result = model::location::Streets {
        triangles: Vec::new(),
        curbs: Vec::new(),
        paths: Vec::new(),
    };
    let mut paths = vec![Vec::new(); count];
    for line in lines123213 {
        let f: Vec<_> = line.split_whitespace().collect();
        let n = |i: usize| -> Result<usize, String> {
            f.get(i)
                .ok_or("missing surface index")?
                .parse()
                .map_err(|_| "invalid surface index".into())
        };
        let v = |i: usize| -> Result<f32, String> {
            let value: f32 = f
                .get(i)
                .ok_or("missing surface coordinate")?
                .parse()
                .map_err(|_| "invalid surface coordinate")?;
            if value.is_finite() {
                Ok(value)
            } else {
                Err("nonfinite surface coordinate".into())
            }
        };
        let p = |i: usize| -> Result<XZ<f32>, String> {
            Ok(XZ {
                x: v(i)?,
                z: v(i + 1)?,
            })
        };
        match f.first().copied() {
            Some("j") => {
                n(1)?;
            }
            Some("p") => {
                let id = n(1)?;
                let length = n(2)?;
                if id >= count || length < 2 || f.len() != 3 + length * 2 || !paths[id].is_empty() {
                    return Err("invalid surface path".into());
                }
                paths[id] = (0..length)
                    .map(|i| {
                        Ok(XZ {
                            x: v(3 + i * 2)?,
                            z: v(4 + i * 2)?,
                        })
                    })
                    .collect::<Result<_, String>>()?;
            }
            Some("t") => {
                let id = n(1)?;
                if f.len() != 8 {
                    return Err("invalid surface triangle".into());
                }
                let points = [p(2)?, p(4)?, p(6)?];
                if cross(points[0], points[1], points[2]) > 0.00001 {
                    result.triangles.push(triangle_from_street_id(id, points)?);
                }
            }
            Some("c") => {
                let style = curb_style(n(1)?)?;
                if f.len() != 8 {
                    return Err("invalid curb".into());
                }
                result.curbs.push(Curb {
                    style,
                    ends: [p(2)?, p(4)?],
                    inward: p(6)?,
                });
            }
            _ => return Err("unknown surface record".into()),
        }
    }
    // An entirely grade-separated location has paths and no ground mesh.
    if result.curbs.is_empty() != result.triangles.is_empty() || paths.iter().any(Vec::is_empty) {
        return Err("incomplete street surfaces".into());
    }
    result.paths = paths;
    Ok(result)
}
fn curb_style(value: usize) -> Result<CurbStyle, &'static str> {
    match value {
        0 => Ok(CurbStyle::MajorRoad),
        1 => Ok(CurbStyle::Road),
        2 => Ok(CurbStyle::ServiceRoad),
        _ => Err("unknown curb style"),
    }
}

/// Decode intermediate triangle categories.
fn triangle_usage_from_street_id(id: usize) -> Result<TriangleUsage, &'static str> {
    match id {
        0 => Ok(TriangleUsage::Road(CurbStyle::MajorRoad)),
        1 => Ok(TriangleUsage::Road(CurbStyle::Road)),
        2 => Ok(TriangleUsage::Road(CurbStyle::ServiceRoad)),
        3 => Ok(TriangleUsage::Gutter),
        4 => Ok(TriangleUsage::Marking),
        5 => Ok(TriangleUsage::Pavement(CurbStyle::MajorRoad)),
        6 => Ok(TriangleUsage::Pavement(CurbStyle::Road)),
        7 => Ok(TriangleUsage::Pavement(CurbStyle::ServiceRoad)),
        8..=16 => Ok(TriangleUsage::Road(curb_style((id - 8) % 3)?)),
        _ => Err("unknown street triangle category"),
    }
}

pub fn triangle_usage_street_id(triangle_usage: TriangleUsage) -> Result<usize, &'static str> {
    match triangle_usage {
        TriangleUsage::Road(CurbStyle::MajorRoad) => Ok(0),
        TriangleUsage::Road(CurbStyle::Road) => Ok(1),
        TriangleUsage::Road(CurbStyle::ServiceRoad) => Ok(2),
        TriangleUsage::Gutter => Ok(3),
        TriangleUsage::Marking => Ok(4),
        TriangleUsage::Pavement(CurbStyle::MajorRoad) => Ok(5),
        TriangleUsage::Pavement(CurbStyle::Road) => Ok(6),
        TriangleUsage::Pavement(CurbStyle::ServiceRoad) => Ok(7),
        TriangleUsage::Ground => Err("street format does not support ground triangles"),
    }
}

/// Existing street formats imply material from the geometry category.
fn street_material(triangle_usage: TriangleUsage) -> Result<Material, &'static str> {
    match triangle_usage {
        TriangleUsage::Road(_) => Ok(Material::Asphalt),
        TriangleUsage::Gutter | TriangleUsage::Pavement(CurbStyle::MajorRoad) => {
            Ok(Material::Concrete)
        }
        TriangleUsage::Marking => Ok(Material::Paint),
        TriangleUsage::Pavement(CurbStyle::Road) => Ok(Material::Paving),
        TriangleUsage::Pavement(CurbStyle::ServiceRoad) => Ok(Material::BareGround),
        TriangleUsage::Ground => Err("street format does not support ground triangles"),
    }
}

fn triangle_from_street_id(id: usize, points: [XZ<f32>; 3]) -> Result<Triangle, &'static str> {
    let usage = triangle_usage_from_street_id(id)?;
    Ok(Triangle {
        material: match id {
            8..=10 => Material::Concrete,
            11..=13 => Material::Paving,
            14..=16 => Material::BareGround,
            _ => street_material(usage)?,
        },
        usage,
        points,
    })
}

#[derive(Clone, Copy)]
pub enum Precision {
    Imported,
    Authored,
}

pub struct Writer {
    output: std::io::BufWriter<std::fs::File>,
    precision: Precision,
}

pub fn create(
    directory: &std::path::Path,
    road_count: usize,
    precision: Precision,
) -> std::io::Result<Writer> {
    use std::io::Write;
    let mut output = std::io::BufWriter::new(std::fs::File::create(directory.join(FILE_NAME))?);
    writeln!(output, "STREETS 1 {road_count}")?;
    Ok(Writer { output, precision })
}

pub fn junction(writer: &mut Writer, node: usize) -> std::io::Result<()> {
    use std::io::Write;
    writeln!(writer.output, "j {node}")
}

fn coordinates<T: std::fmt::Display + Copy>(
    writer: &mut Writer,
    points: &[XZ<T>],
) -> std::io::Result<()> {
    use std::io::Write;
    for point in points {
        match writer.precision {
            Precision::Imported => write!(writer.output, " {:.6} {:.6}", point.x, point.z)?,
            Precision::Authored => write!(writer.output, " {} {}", point.x, point.z)?,
        }
    }
    writeln!(writer.output)
}

pub fn path<T: std::fmt::Display + Copy>(
    writer: &mut Writer,
    road: usize,
    points: &[XZ<T>],
) -> std::io::Result<()> {
    use std::io::Write;
    write!(writer.output, "p {road} {}", points.len())?;
    coordinates(writer, points)
}

pub fn triangle<T: std::fmt::Display + Copy>(
    writer: &mut Writer,
    category: usize,
    points: [XZ<T>; 3],
) -> std::io::Result<()> {
    use std::io::Write;
    write!(writer.output, "t {category}")?;
    coordinates(writer, &points)
}

pub fn curb<T: std::fmt::Display + Copy>(
    writer: &mut Writer,
    kind: usize,
    ends: [XZ<T>; 2],
    inward: XZ<T>,
) -> std::io::Result<()> {
    use std::io::Write;
    write!(writer.output, "c {kind}")?;
    coordinates(writer, &[ends[0], ends[1], inward])
}

pub fn flush(writer: &mut Writer) -> std::io::Result<()> {
    use std::io::Write;
    writer.output.flush()
}

pub fn read_tree_obstacles(
    directory: &std::path::Path,
    road_count: usize,
) -> error::Result<Vec<Vec<XZ<f64>>>> {
    use crate::util::error::invalid;
    use std::io::{BufRead, BufReader};
    let mut triangles = Vec::new();
    let mut input = BufReader::new(std::fs::File::open(directory.join(FILE_NAME))?);
    let mut line = String::new();
    input.read_line(&mut line)?;
    let header: Vec<_> = line.split_whitespace().collect();
    if header.len() < 3
        || header[..2] != ["STREETS", "1"]
        || header[2].parse::<usize>().ok() != Some(road_count)
    {
        return Err(invalid("invalid street surfaces header"));
    }
    loop {
        line.clear();
        if input.read_line(&mut line)? == 0 {
            break;
        }
        let mut f = line.split_whitespace();
        if f.next() != Some("t") {
            continue;
        }
        let style = f
            .next()
            .ok_or_else(|| invalid("missing surface style"))?
            .parse::<usize>()?;
        // 0..=3 are asphalt; 4..=7 are pavement/other pedestrian surfaces.
        if style > 3 {
            continue;
        }
        let mut points = vec![];
        for _ in 0..3 {
            let mut p = XZ::<f64>::default();
            for v in util::geometry::xz_iter_mut(&mut p) {
                *v = f
                    .next()
                    .ok_or_else(|| invalid("missing surface coordinate"))?
                    .parse::<f64>()?;
                if !v.is_finite() {
                    return Err(invalid("invalid road coordinate"));
                }
            }
            points.push(p);
        }
        if f.next().is_some() {
            return Err(invalid("invalid road triangle"));
        }
        let [a, b, c] = [points[0], points[1], points[2]];
        if (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x) <= 0.00001 {
            continue;
        }
        triangles.push(points);
    }
    Ok(triangles)
}

pub fn read(directory: &std::path::Path, roads: usize) -> error::Result<model::location::Streets> {
    let source = std::fs::read_to_string(directory.join(FILE_NAME))?;
    parse(&source, roads).map_err(error::invalid)
}

#[cfg(test)]
#[path = "tests/streets_support.rs"]
pub mod streets_test_support;
