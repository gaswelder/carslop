//! LANDSCAPE 1 records, shared by runtime loading and compiler exclusion checks.
//! Legacy trailing header fields remain accepted. Unknown/malformed records fail.
use super::Number;
use crate::geometry::XZ;
use crate::world::{Material, Tree};
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
                material: Material::from_ground_id(
                    f[1].parse().map_err(|_| "invalid landscape style")?,
                )?,
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
impl<W: io::Write> Writer<W> {
    pub fn new(mut output: W, precision: Precision) -> io::Result<Self> {
        writeln!(output, "LANDSCAPE 1")?;
        Ok(Self { output, precision })
    }
    pub fn ground<T: Copy + Display>(
        &mut self,
        material: Material,
        points: [XZ<T>; 3],
    ) -> io::Result<()> {
        let digits = match self.precision {
            Precision::Imported => Some(6),
            Precision::Authored => None,
        };
        write!(
            self.output,
            "a {}",
            material.ground_id().map_err(io::Error::other)?
        )?;
        for p in points {
            write!(
                self.output,
                " {} {}",
                Number(p.x, digits),
                Number(p.z, digits)
            )?;
        }
        writeln!(self.output)
    }
    pub fn tree<T: Copy + Display>(&mut self, tree: &Tree<T>) -> io::Result<()> {
        self.tree_fields(
            tree.id,
            tree.coords,
            tree.height_meters,
            tree.crown_radius_meters,
        )
    }

    /// Signed source IDs are retained verbatim by the mapped compiler, as before.
    pub fn tree_fields<T: Copy + Display>(
        &mut self,
        id: impl Display,
        coords: XZ<T>,
        height: T,
        radius: T,
    ) -> io::Result<()> {
        let digits = match self.precision {
            Precision::Imported => Some(3),
            Precision::Authored => None,
        };
        writeln!(
            self.output,
            "t {} {} {} {} {}",
            id,
            Number(coords.x, digits),
            Number(coords.z, digits),
            Number(height, digits),
            Number(radius, digits)
        )
    }
    pub fn flush(&mut self) -> io::Result<()> {
        self.output.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn landscape_precisions_round_trip_without_changing_authored_or_imported_text() {
        let tree = Tree {
            id: u64::MAX,
            coords: XZ {
                x: 1.123456,
                z: -2.654321,
            },
            height_meters: 4.123456,
            crown_radius_meters: 0.7,
        };
        let points = [
            XZ {
                x: 0.123456789,
                z: 0.,
            },
            XZ { x: 10., z: 0. },
            XZ { x: 0., z: 10. },
        ];
        for (precision, expected) in [
            (Precision::Imported, "LANDSCAPE 1\na 1 0.123457 0.000000 10.000000 0.000000 0.000000 10.000000\nt 18446744073709551615 1.123 -2.654 4.123 0.700\n"),
            (Precision::Authored, "LANDSCAPE 1\na 1 0.123456789 0 10 0 0 10\nt 18446744073709551615 1.123456 -2.654321 4.123456 0.7\n"),
        ] {
            let mut bytes = vec![];
            let mut writer = Writer::new(&mut bytes, precision).unwrap();
            writer.ground(Material::Park, points).unwrap();
            writer.tree(&tree).unwrap();
            writer.flush().unwrap();
            assert_eq!(std::str::from_utf8(&bytes).unwrap(), expected);
            let mut runtime_tree = None;
            read::<f32>(Cursor::new(&bytes), |r| {
                if let Record::Tree(t) = r { runtime_tree = Some(t); }
                Ok(())
            }).unwrap();
            let runtime_tree = runtime_tree.unwrap();
            assert_eq!(runtime_tree.id, u64::MAX);
            assert_eq!(runtime_tree.crown_radius_meters, 0.7_f32);
        }
    }

    #[test]
    fn landscape_validation_is_shared_and_emission_errors_propagate() {
        for record in [
            "a 9 0 0 1 0 0 1",
            "t 1 NaN 0 4 2",
            "t 1 100000 0 4 2",
            "t 1 0 0 -2 1",
            "t 1 0 0 4 0.69",
            "t 1 0 0 4",
            "unknown",
            "",
        ] {
            let text = format!("LANDSCAPE 1\n{record}\n");
            assert!(
                read::<f32>(Cursor::new(&text), |_| Ok(())).is_err(),
                "{record}"
            );
            assert!(
                read::<f64>(Cursor::new(&text), |_| Ok(())).is_err(),
                "{record}"
            );
        }
        let error = read::<f64>(Cursor::new("LANDSCAPE 1\nt 1 0 0 4 2\n"), |_| {
            Err("stop".into())
        })
        .unwrap_err();
        assert_eq!(error, "stop");
        assert!(Writer::new(std::io::sink(), Precision::Imported)
            .unwrap()
            .ground(Material::Paint, [XZ { x: 0., z: 0. }; 3])
            .is_err());
    }
}
