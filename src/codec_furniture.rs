//! FURNITURE 3: placement precision and runtime validation are separate concerns.
use crate::geometry::{XYZ, XZ};
use crate::world::{Furniture, FurnitureKind};
use std::io;

/// Compiler placement before the file's deliberate position/direction quantization.
#[derive(Debug)]
pub struct FurnitureRecord {
    pub source_id: i64,
    pub kind: FurnitureKind,
    pub position: XZ<f64>,
    pub travel: XZ<f64>,
    pub value: u16,
    pub phase: u8,
}

pub fn write(writer: &mut impl io::Write, records: &[FurnitureRecord]) -> io::Result<()> {
    writeln!(writer, "FURNITURE 3")?;
    for f in records {
        let kind = match f.kind {
            FurnitureKind::Signal => "signal",
            FurnitureKind::Stop => "stop",
            FurnitureKind::Yield => "yield",
            FurnitureKind::Speed => "speed_limit",
        };
        writeln!(
            writer,
            "{} {} {:.3} {:.3} {:.7} {:.7} {} {}",
            f.source_id, kind, f.position.x, f.position.z, f.travel.x, f.travel.z, f.value, f.phase
        )?;
    }
    Ok(())
}

pub fn parse(text: &str) -> Result<Vec<Furniture>, String> {
    let mut lines = text.lines();
    let header: Vec<_> = lines
        .next()
        .ok_or("missing furniture header")?
        .split_whitespace()
        .collect();
    if header.len() < 2 || header[0] != "FURNITURE" || header[1] != "3" {
        return Err("unsupported furniture header".into());
    }
    let mut result = Vec::new();
    for line in lines {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.len() != 8 {
            return Err("invalid road furniture record".into());
        }
        let n = |i: usize| -> Result<u64, String> {
            f[i].parse().map_err(|_| "invalid furniture index".into())
        };
        let v = |i: usize| -> Result<f32, String> {
            let value: f32 = f[i].parse().map_err(|_| "invalid furniture coordinate")?;
            if value.is_finite() {
                Ok(value)
            } else {
                Err("nonfinite furniture coordinate".into())
            }
        };
        let kind = match f[1] {
            "signal" => FurnitureKind::Signal,
            "stop" => FurnitureKind::Stop,
            "yield" => FurnitureKind::Yield,
            "speed_limit" => FurnitureKind::Speed,
            _ => return Err("unknown furniture kind".into()),
        };
        let travel = XZ { x: v(4)?, z: v(5)? };
        let length = travel.x.hypot(travel.z);
        let value = n(6)?;
        let phase = n(7)?;
        if (length - 1.0).abs() > 0.001
            || phase >= 50
            || value > 150
            || (kind == FurnitureKind::Speed && value < 5)
        {
            return Err("invalid furniture direction, phase or value".into());
        }
        result.push(Furniture {
            _source_id: n(0)?,
            kind,
            coords: XYZ {
                x: v(2)?,
                y: 0.0,
                z: v(3)?,
            },
            travel,
            value: value as u16,
            phase: phase as u8,
        });
    }
    Ok(result)
}
