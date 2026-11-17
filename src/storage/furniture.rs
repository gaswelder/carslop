//! Road controls and terrain-following barriers, identified by record kind.
use crate::model::location;
use crate::model::world::Barrier;
use crate::model::world::BarrierKind;
use crate::model::world::Furniture;
use crate::model::world::FurnitureKind;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use std::fs;
use std::io;
use std::io::{BufWriter, Write};
use std::path::Path;

const FILE_NAME: &str = "furniture.txt";

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

pub fn write(
    writer: &mut impl io::Write,
    records: &[FurnitureRecord],
    barriers: &[Barrier],
) -> io::Result<()> {
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
    write_barriers(writer, barriers)
}

pub struct FurnitureData {
    pub controls: Vec<Furniture>,
    pub barriers: Vec<Barrier>,
}

pub fn parse(text: &str) -> Result<FurnitureData, String> {
    let mut result = Vec::new();
    let mut barriers = Vec::new();
    for line in text.lines() {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.len() != 8 {
            return Err("invalid road furniture record".into());
        }
        if !matches!(f[1], "signal" | "stop" | "yield" | "speed_limit") {
            barriers.push(parse_barrier(&f)?);
            continue;
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
    Ok(FurnitureData {
        controls: result,
        barriers,
    })
}

pub fn write_barriers(writer: &mut impl io::Write, records: &[Barrier]) -> io::Result<()> {
    for barrier in records {
        let kind = match barrier.kind {
            BarrierKind::Wall => "wall",
            BarrierKind::Hedge => "hedge",
            BarrierKind::GuardRail => "guard_rail",
            BarrierKind::RetainingWall => "retaining_wall",
            BarrierKind::Fence => "fence",
            BarrierKind::ChainLink => "chain_link",
            BarrierKind::WoodFence => "wood_fence",
        };
        writeln!(
            writer,
            "{} {} {} {} {} {} {} {}",
            barrier.source_id,
            kind,
            barrier.ends[0].x,
            barrier.ends[0].z,
            barrier.ends[1].x,
            barrier.ends[1].z,
            barrier.height_meters,
            barrier.width_meters
        )?;
    }
    writer.flush()
}

fn parse_barrier(fields: &[&str]) -> Result<Barrier, String> {
    let number = |i: usize| -> Result<f32, String> {
        fields[i]
            .parse::<f32>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| "invalid barrier number".into())
    };
    let kind = match fields[1] {
        "wall" => BarrierKind::Wall,
        "hedge" => BarrierKind::Hedge,
        "guard_rail" => BarrierKind::GuardRail,
        "retaining_wall" => BarrierKind::RetainingWall,
        "fence" => BarrierKind::Fence,
        "chain_link" => BarrierKind::ChainLink,
        "wood_fence" => BarrierKind::WoodFence,
        _ => return Err("invalid barrier kind".into()),
    };
    let ends = [
        XZ {
            x: number(2)?,
            z: number(3)?,
        },
        XZ {
            x: number(4)?,
            z: number(5)?,
        },
    ];
    let height_meters = number(6)?;
    let width_meters = number(7)?;
    if ends
        .iter()
        .any(|p| p.x.abs() > 100_000.0 || p.z.abs() > 100_000.0)
        || (ends[1].x - ends[0].x).hypot(ends[1].z - ends[0].z) < 0.1
        || !(0.2..=8.0).contains(&height_meters)
        || !(0.04..=2.0).contains(&width_meters)
    {
        return Err("invalid barrier dimensions".into());
    }
    Ok(Barrier {
        source_id: fields[0].parse().map_err(|_| "invalid barrier ID")?,
        kind,
        ends,
        height_meters,
        width_meters,
    })
}

#[cfg(test)]
#[path = "tests/furniture.rs"]
mod tests;

pub fn write_file(
    directory: &std::path::Path,
    records: &[FurnitureRecord],
    barriers: &[Barrier],
) -> io::Result<()> {
    use std::io::Write;
    let mut writer = io::BufWriter::new(std::fs::File::create(directory.join(FILE_NAME))?);
    write(&mut writer, records, barriers)?;
    writer.flush()
}

pub fn read(directory: &Path) -> Result<FurnitureData, String> {
    let text =
        fs::read_to_string(directory.join(FILE_NAME)).map_err(|e| format!("{FILE_NAME}: {e}"))?;
    parse(&text).map_err(|e| format!("{FILE_NAME}: {e}"))
}

pub fn write_location(directory: &Path, location: &location::Location) -> io::Result<()> {
    let mut furniture = BufWriter::new(fs::File::create(directory.join(FILE_NAME))?);
    for f in &location.furniture {
        let kind = match f.kind {
            FurnitureKind::Signal => "signal",
            FurnitureKind::Stop => "stop",
            FurnitureKind::Yield => "yield",
            FurnitureKind::Speed => "speed_limit",
        };
        writeln!(
            furniture,
            "{} {kind} {} {} {} {} {} {}",
            f.id, f.position.x, f.position.z, f.travel.x, f.travel.z, f.value, f.phase
        )?;
    }
    write_barriers(&mut furniture, &location.barriers)?;
    furniture.flush()
}
