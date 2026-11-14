//! BARRIERS 1: terrain-following wall/fence segments at runtime precision.
use crate::geometry::XZ;
use crate::world::{Barrier, BarrierKind};
use std::io;

pub fn write(writer: &mut impl io::Write, records: &[Barrier]) -> io::Result<()> {
    writeln!(writer, "BARRIERS 1")?;
    for barrier in records {
        let kind = match barrier.kind {
            BarrierKind::Wall => "wall",
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

pub fn parse(text: &str) -> Result<Vec<Barrier>, String> {
    let mut lines = text.lines();
    if lines.next() != Some("BARRIERS 1") {
        return Err("unsupported barriers header".into());
    }
    let mut result = Vec::new();
    for line in lines {
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() != 8 {
            return Err("invalid barrier record".into());
        }
        let number = |i: usize| -> Result<f32, String> {
            fields[i]
                .parse::<f32>()
                .ok()
                .filter(|v| v.is_finite())
                .ok_or_else(|| "invalid barrier number".into())
        };
        let kind = match fields[1] {
            "wall" => BarrierKind::Wall,
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
        result.push(Barrier {
            source_id: fields[0].parse().map_err(|_| "invalid barrier ID")?,
            kind,
            ends,
            height_meters,
            width_meters,
        });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_nonfinite_degenerate_and_unknown_barriers() {
        for record in [
            "1 wall 0 0 0 0 2 0.2",
            "1 wall NaN 0 10 0 2 0.2",
            "1 wall 0 0 10 0 inf 0.2",
            "1 wall 0 0 10 0 2 0",
            "1 wall 0 0 10 0 20 0.2",
            "1 unknown 0 0 10 0 2 0.2",
            "1 wall 0 0 10 0 2",
            "1 wall 0 0 100001 0 2 0.2",
        ] {
            assert!(
                parse(&format!("BARRIERS 1\n{record}\n")).is_err(),
                "{record}"
            );
        }
        assert!(parse("BARRIERS 2\n").is_err());
        assert!(parse("BARRIERS 1\n").unwrap().is_empty());
        for kind in ["wall", "fence", "chain_link", "wood_fence"] {
            let records = parse(&format!(
                "BARRIERS 1\n1 {kind} -5.1234565 0 10 0 1.8765432 0.1234567\n"
            ))
            .unwrap();
            let mut bytes = Vec::new();
            write(&mut bytes, &records).unwrap();
            assert_eq!(
                parse(std::str::from_utf8(&bytes).unwrap()).unwrap(),
                records
            );
        }
    }
}
