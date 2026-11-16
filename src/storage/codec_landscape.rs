//! LANDSCAPE 1 records, shared by runtime loading and compiler exclusion checks.
//! Legacy trailing header fields remain accepted. Unknown/malformed records fail.

use crate::model::world::Material;
use crate::model::world::Tree;
use crate::storage;
use crate::util::geometry::XZ;
use std::{fmt::Display, io, str::FromStr};

pub enum Record<T> {
    Ground {
        material: Material,
        points: [XZ<T>; 3],
    },
    Tree(Tree<T>),
}

/// Decode one line at a time. T chooses the established f32 runtime or f64 compiler
/// precision *before* validation; compiler checks do not quantize tree positions.
pub fn read<T>(
    reader: impl io::BufRead,
    mut emit: impl FnMut(Record<T>) -> Result<(), String>,
) -> Result<(), String>
where
    T: Copy + FromStr + Into<f64> + PartialOrd,
{
    let mut lines = reader.lines();
    let header = lines
        .next()
        .transpose()
        .map_err(|e| e.to_string())?
        .unwrap_or_default();
    if header.split_whitespace().take(2).collect::<Vec<_>>() != ["LANDSCAPE", "1"] {
        return Err("unsupported landscape header".into());
    }
    for line in lines {
        let line = line.map_err(|e| e.to_string())?;
        let f: Vec<_> = line.split_whitespace().collect();
        let value = |i: usize| -> Result<T, String> {
            let v: T = f
                .get(i)
                .ok_or("missing landscape coordinate")?
                .parse()
                .map_err(|_| "invalid landscape number")?;
            let n: f64 = v.into();
            if n.is_finite() && n.abs() < 100_000.0 {
                Ok(v)
            } else {
                Err("invalid landscape coordinate".into())
            }
        };
        let record = match f.first().copied() {
            Some("a") if f.len() == 8 => Record::Ground {
                material: from_ground_id(f[1].parse().map_err(|_| "invalid landscape style")?)?,
                points: [
                    XZ {
                        x: value(2)?,
                        z: value(3)?,
                    },
                    XZ {
                        x: value(4)?,
                        z: value(5)?,
                    },
                    XZ {
                        x: value(6)?,
                        z: value(7)?,
                    },
                ],
            },
            Some("t") if f.len() == 6 => {
                let height = value(4)?;
                let radius = value(5)?;
                // Parse limits at the selected precision too: f32(0.7) is slightly
                // below f64(0.7), but remains a valid runtime radius.
                let limit = |s: &str| s.parse::<T>().map_err(|_| "invalid landscape limit");
                if !(limit("2.5")?..=limit("18")?).contains(&height)
                    || !(limit("0.7")?..=limit("4.5")?).contains(&radius)
                {
                    return Err("invalid tree size".into());
                }
                Record::Tree(Tree {
                    id: f[1].parse().map_err(|_| "invalid tree id")?,
                    coords: XZ {
                        x: value(2)?,
                        z: value(3)?,
                    },
                    height_meters: height,
                    crown_radius_meters: radius,
                })
            }
            _ => return Err("invalid landscape record".into()),
        };
        emit(record)?;
    }
    Ok(())
}

#[derive(Clone, Copy)]
pub enum Precision {
    /// Mapped ground uses six decimals after quantized winding checks; trees use three.
    Imported,
    /// Authored f32 geometry retains shortest text without extra rounding.
    Authored,
}

/// Incremental serialization; generators can emit records without collecting a mesh.
pub struct Writer<W> {
    output: W,
    precision: Precision,
}

pub fn new<W: io::Write>(mut output: W, precision: Precision) -> io::Result<Writer<W>> {
    writeln!(output, "LANDSCAPE 1")?;
    Ok(Writer::<W> { output, precision })
}
pub fn ground<W: io::Write, T: Copy + Display>(
    writer: &mut Writer<W>,
    material: Material,
    points: [XZ<T>; 3],
) -> io::Result<()> {
    let digits = match writer.precision {
        Precision::Imported => Some(6),
        Precision::Authored => None,
    };
    write!(
        writer.output,
        "a {}",
        ground_id(material).map_err(io::Error::other)?
    )?;
    for p in points {
        write!(
            writer.output,
            " {} {}",
            storage::codec_number::new(p.x, digits),
            storage::codec_number::new(p.z, digits)
        )?;
    }
    writeln!(writer.output)
}
pub fn tree<W: io::Write, T: Copy + Display>(
    writer: &mut Writer<W>,
    tree: &Tree<T>,
) -> io::Result<()> {
    tree_fields(
        writer,
        tree.id,
        tree.coords,
        tree.height_meters,
        tree.crown_radius_meters,
    )
}

/// Signed source IDs are retained verbatim by the mapped compiler, as before.
pub fn tree_fields<W: io::Write, T: Copy + Display>(
    writer: &mut Writer<W>,
    id: impl Display,
    coords: XZ<T>,
    height: T,
    radius: T,
) -> io::Result<()> {
    let digits = match writer.precision {
        Precision::Imported => Some(3),
        Precision::Authored => None,
    };
    writeln!(
        writer.output,
        "t {} {} {} {} {}",
        id,
        storage::codec_number::new(coords.x, digits),
        storage::codec_number::new(coords.z, digits),
        storage::codec_number::new(height, digits),
        storage::codec_number::new(radius, digits)
    )
}
pub fn flush<W: io::Write>(writer: &mut Writer<W>) -> io::Result<()> {
    writer.output.flush()
}

/// Decode the material IDs used by landscape.txt.
pub fn from_ground_id(id: usize) -> Result<Material, &'static str> {
    match id {
        0 => Ok(Material::Grass),
        1 => Ok(Material::Park),
        2 => Ok(Material::Scrub),
        3 => Ok(Material::Asphalt),
        4 => Ok(Material::Paving),
        5 => Ok(Material::Concrete),
        6 => Ok(Material::BareGround),
        7 => Ok(Material::Sports),
        8 => Ok(Material::Water),
        _ => Err("unknown ground material"),
    }
}

fn ground_id(material: Material) -> Result<usize, &'static str> {
    match material {
        Material::Grass => Ok(0),
        Material::Park => Ok(1),
        Material::Scrub => Ok(2),
        Material::Asphalt => Ok(3),
        Material::Paving => Ok(4),
        Material::Concrete => Ok(5),
        Material::BareGround => Ok(6),
        Material::Sports => Ok(7),
        Material::Water => Ok(8),
        Material::Paint => Err("landscape format does not support paint"),
    }
}
