//! ROAD_RISES 1: triangles whose Y values are offsets above the normal road.
use crate::geometry::XYZ;
use crate::world::RoadRise;
use std::io;

pub fn write(writer: &mut impl io::Write, rises: &[RoadRise]) -> io::Result<()> {
    writeln!(writer, "ROAD_RISES 1")?;
    for rise in rises {
        write!(writer, "t")?;
        for point in rise.points {
            write!(writer, " {} {} {}", point.x, point.y, point.z)?;
        }
        writeln!(writer)?;
    }
    writer.flush()
}
pub fn parse(text: &str) -> Result<Vec<RoadRise>, String> {
    let mut lines = text.lines();
    if lines.next() != Some("ROAD_RISES 1") {
        return Err("unsupported road rises header".into());
    }
    let mut result = Vec::new();
    for line in lines {
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() != 10 || fields[0] != "t" {
            return Err("invalid road rise record".into());
        }
        let number = |i: usize| -> Result<f32, String> {
            fields[i]
                .parse::<f32>()
                .ok()
                .filter(|v| v.is_finite())
                .ok_or_else(|| "invalid road rise coordinate".into())
        };
        let mut points = [XYZ::default(); 3];
        for (i, point) in points.iter_mut().enumerate() {
            *point = XYZ {
                x: number(1 + i * 3)?,
                y: number(2 + i * 3)?,
                z: number(3 + i * 3)?,
            };
            if point.x.abs() > 100_000.
                || point.z.abs() > 100_000.
                || !(0.0..=0.3).contains(&point.y)
            {
                return Err("invalid road rise dimensions".into());
            }
        }
        let [a, b, c] = points;
        let area = ((b.x - a.x) as f64 * (c.z - a.z) as f64
            - (b.z - a.z) as f64 * (c.x - a.x) as f64)
            .abs();
        if area < 0.00001 {
            return Err("degenerate road rise".into());
        }
        result.push(RoadRise { points });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn road_rise_codec_preserves_offsets_and_rejects_invalid_triangles() {
        let text = "ROAD_RISES 1\nt 0 0 0 2 0.1234567 0 0 0 2\n";
        let rises = parse(text).unwrap();
        let mut bytes = Vec::new();
        write(&mut bytes, &rises).unwrap();
        assert_eq!(parse(std::str::from_utf8(&bytes).unwrap()).unwrap(), rises);
        for invalid in [
            "t 0 0 0 0 0 0 0 0 0",
            "t 0 0 0 2 NaN 0 0 0 2",
            "t 0 0 0 2 0.5 0 0 0 2",
            "t 0 0 0 2 -0.1 0 0 0 2",
            "t 0 0 0 2 0 0 0 0",
        ] {
            assert!(parse(&format!("ROAD_RISES 1\n{invalid}\n")).is_err());
        }
    }
}
