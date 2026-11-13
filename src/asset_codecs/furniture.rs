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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn furniture_serialization_preserves_precision_kinds_and_runtime_fields() {
        let records: Vec<_> = [
            FurnitureKind::Signal,
            FurnitureKind::Stop,
            FurnitureKind::Yield,
            FurnitureKind::Speed,
        ]
        .into_iter()
        .enumerate()
        .map(|(i, kind)| FurnitureRecord {
            source_id: 42 + i as i64,
            kind,
            position: XZ {
                x: 1.123456,
                z: -2.654321,
            },
            travel: XZ { x: 0.6, z: 0.8 },
            value: if kind == FurnitureKind::Speed { 30 } else { 0 },
            phase: 49,
        })
        .collect();
        let mut bytes = vec![];
        write(&mut bytes, &records).unwrap();
        let text = std::str::from_utf8(&bytes).unwrap();
        assert_eq!(
            text,
            concat!(
                "FURNITURE 3\n",
                "42 signal 1.123 -2.654 0.6000000 0.8000000 0 49\n",
                "43 stop 1.123 -2.654 0.6000000 0.8000000 0 49\n",
                "44 yield 1.123 -2.654 0.6000000 0.8000000 0 49\n",
                "45 speed_limit 1.123 -2.654 0.6000000 0.8000000 30 49\n"
            )
        );
        let loaded = parse(text).unwrap();
        for (source, actual) in records.iter().zip(loaded) {
            assert_eq!(actual._source_id, source.source_id as u64);
            assert_eq!(actual.kind, source.kind);
            assert_eq!(actual.coords.x, 1.123);
            assert_eq!(actual.coords.z, -2.654);
            assert_eq!(actual.travel, XZ { x: 0.6, z: 0.8 });
            assert_eq!(actual.value, source.value);
            assert_eq!(actual.phase, source.phase);
        }
    }
}
