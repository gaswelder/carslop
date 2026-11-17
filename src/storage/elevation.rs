use crate::model::world::HeightField;
use crate::util::error;
use crate::util::geometry::XZ;
use std::{
    fs,
    io::{BufWriter, Write},
    path::Path,
};

const FILE_NAME: &str = "elevation.txt";

pub struct Elevation {
    pub field: HeightField,
    pub datum: f32,
}

pub fn read(directory: &Path) -> Result<Elevation, String> {
    let text =
        fs::read_to_string(directory.join(FILE_NAME)).map_err(|e| format!("elevation: {e}"))?;
    parse(&text).map_err(|e| format!("elevation: {e}"))
}

pub fn write(directory: &Path, field: &HeightField) -> std::io::Result<()> {
    let mut writer = BufWriter::new(fs::File::create(directory.join(FILE_NAME))?);
    writeln!(
        writer,
        "ELEVATION 1\ngrid {} {} {} {} {} 0",
        field.columns, field.rows, field.origin.x, field.origin.z, field.step
    )?;
    for row in field.values.chunks(field.columns) {
        for value in row {
            if !value.is_finite() || value.abs() >= 10000.0 {
                return Err(std::io::Error::other(
                    "elevation exceeds the supported relative height range",
                ));
            }
            write!(writer, "{value} ")?;
        }
        writeln!(writer)?;
    }
    writer.flush()
}

pub struct Grid {
    pub columns: usize,
    pub rows: usize,
    pub origin: XZ,
    pub step: f64,
    pub datum: f64,
}

pub fn write_sampled(
    directory: &Path,
    grid: Grid,
    mut sample: impl FnMut(usize, usize) -> error::Result<f64>,
) -> error::Result<()> {
    let mut writer = BufWriter::new(fs::File::create(directory.join(FILE_NAME))?);
    writeln!(
        writer,
        "ELEVATION 1\ngrid {} {} {:.3} {:.3} {:.3} {:.3}",
        grid.columns, grid.rows, grid.origin.x, grid.origin.z, grid.step, grid.datum
    )?;
    for row in 0..grid.rows {
        for column in 0..grid.columns {
            let value = sample(column, row)?;
            if !value.is_finite() || value.abs() >= 10000.0 {
                return Err(std::io::Error::other(
                    "terrain exceeds the supported relative height range",
                )
                .into());
            }
            if column > 0 {
                write!(writer, " ")?;
            }
            write!(writer, "{value:.3}")?;
        }
        writeln!(writer)?;
    }
    writer.flush()?;
    Ok(())
}

pub fn copy_source(directory: &Path, source: &Path) -> std::io::Result<()> {
    let target = directory.join("elevation");
    fs::create_dir_all(&target)?;
    let name = source
        .file_name()
        .ok_or_else(|| std::io::Error::other("missing HGT filename"))?;
    fs::copy(source, target.join(name))?;
    Ok(())
}

fn parse(text: &str) -> Result<Elevation, String> {
    let mut lines = text.lines();
    let header: Vec<_> = lines
        .next()
        .ok_or("missing elevation header")?
        .split_whitespace()
        .collect();
    if header.len() < 2 || header[0] != "ELEVATION" || header[1] != "1" {
        return Err("unsupported elevation header".into());
    }
    let fields: Vec<_> = lines
        .next()
        .ok_or("missing grid")?
        .split_whitespace()
        .collect();
    if fields.len() != 7 || fields[0] != "grid" {
        return Err("invalid elevation grid".into());
    }
    let number = |i: usize| -> Result<f32, String> {
        let v: f32 = fields[i].parse().map_err(|_| "invalid grid number")?;
        if v.is_finite() {
            Ok(v)
        } else {
            Err("nonfinite grid number".into())
        }
    };
    let columns = fields[1].parse::<usize>().map_err(|_| "invalid columns")?;
    let rows = fields[2].parse::<usize>().map_err(|_| "invalid rows")?;
    let origin = XZ {
        x: number(3)?,
        z: number(4)?,
    };
    let step = number(5)?;
    let datum = number(6)?;
    if !(2..=4096).contains(&columns)
        || !(2..=4096).contains(&rows)
        || !(1.0..=100.0).contains(&step)
    {
        return Err("invalid grid dimensions".into());
    }
    let values = lines
        .flat_map(str::split_whitespace)
        .map(|s| {
            let v: f32 = s.parse().map_err(|_| "invalid elevation value")?;
            if v.is_finite() && v.abs() < 10000.0 {
                Ok(v)
            } else {
                Err("invalid elevation range")
            }
        })
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != columns * rows {
        return Err("incomplete elevation grid".into());
    }
    Ok(Elevation {
        datum,
        field: HeightField {
            columns,
            rows,
            origin,
            step,
            values,
        },
    })
}

#[cfg(test)]
#[path = "tests/elevation.rs"]
mod tests;
