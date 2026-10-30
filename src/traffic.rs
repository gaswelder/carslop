//! Mapped roadside controls. Pole locations and timings are renderer estimates.
pub const CYCLE_SECONDS: f32 = 50.0;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FurnitureKind {
    Signal,
    Stop,
    Yield,
    Speed,
}
#[derive(Debug)]
pub struct Furniture {
    pub _source_id: u64,
    pub kind: FurnitureKind,
    pub position: [f32; 3],
    pub travel: [f32; 2],
    pub value: u16,
    pub phase: u8,
    pub _estimated_curb: bool,
    pub _inferred_direction: bool,
}
#[cfg(test)]
pub fn load(text: &str, city: &str, streets: &str) -> Result<Vec<Furniture>, String> {
    load_with_checksums(
        text,
        crate::street_surface::SourceChecksums::new(city, streets),
    )
}
pub(crate) fn load_with_checksums(
    text: &str,
    checksums: crate::street_surface::SourceChecksums,
) -> Result<Vec<Furniture>, String> {
    let mut lines = text.lines();
    let header: Vec<_> = lines
        .next()
        .ok_or("missing furniture header")?
        .split_whitespace()
        .collect();
    if header.len() != 4
        || header[0] != "FURNITURE"
        || header[1] != "1"
        || u32::from_str_radix(header[2], 16).ok() != Some(checksums.city)
        || u32::from_str_radix(header[3], 16).ok() != Some(checksums.streets)
    {
        return Err(
            "stale furniture package; re-import the source with add-location under a new name"
                .into(),
        );
    }
    let mut result = Vec::new();
    for line in lines {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.len() != 11 || f[0] != "f" {
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
        let kind = match n(2)? {
            0 => FurnitureKind::Signal,
            1 => FurnitureKind::Stop,
            2 => FurnitureKind::Yield,
            3 => FurnitureKind::Speed,
            _ => return Err("unknown furniture kind".into()),
        };
        let travel = [v(5)?, v(6)?];
        let length = travel[0].hypot(travel[1]);
        let value = n(7)?;
        let phase = n(8)?;
        if (length - 1.0).abs() > 0.001
            || phase >= 50
            || value > 150
            || (kind == FurnitureKind::Speed && value < 5)
            || n(9)? > 1
            || n(10)? > 1
        {
            return Err("invalid furniture direction, phase or value".into());
        }
        result.push(Furniture {
            _source_id: n(1)?,
            kind,
            position: [v(3)?, 0.0, v(4)?],
            travel,
            value: value as u16,
            phase: phase as u8,
            _estimated_curb: n(9)? != 0,
            _inferred_direction: n(10)? != 0,
        });
    }
    Ok(result)
}

#[cfg(test)]
pub fn lamp_lit(time: f32, offset: u8, lamp: usize) -> bool {
    let phase = (time + offset as f32).rem_euclid(CYCLE_SECONDS);
    match lamp {
        0 => phase >= 23.0,
        1 => (20.0..23.0).contains(&phase) || phase >= 48.0,
        2 => phase < 20.0,
        _ => false,
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shared_checksums_reject_stale_furniture_sources() {
        use crate::street_surface::SourceChecksums;
        let checksums = SourceChecksums::new("city", "streets");
        let text = format!(
            "FURNITURE 1 {:08x} {:08x}\n",
            checksums.city, checksums.streets
        );
        assert!(load_with_checksums(&text, checksums).unwrap().is_empty());
        for stale in [
            SourceChecksums::new("changed city", "streets"),
            SourceChecksums::new("city", "changed streets"),
        ] {
            assert!(load_with_checksums(&text, stale).is_err());
        }
    }

    #[test]
    fn alternating_signal_phases_include_clearance_and_never_conflicting_green() {
        for t in 0..5000 {
            let time = t as f32 / 100.0;
            assert!(!(lamp_lit(time, 0, 2) && lamp_lit(time, 25, 2)));
        }
        assert!(lamp_lit(0.0, 0, 2));
        assert!(lamp_lit(21.0, 0, 1));
        assert!(lamp_lit(24.0, 0, 0) && lamp_lit(24.0, 25, 0));
        assert!(lamp_lit(49.0, 0, 0) && lamp_lit(49.0, 0, 1));
    }
    #[test]
    fn mapped_furniture_is_typed_finite_and_outside_the_driving_surface() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let surface = city.surfaces.as_ref().unwrap();
        assert_eq!(
            city.furniture
                .iter()
                .filter(|f| f.kind == FurnitureKind::Signal)
                .count(),
            1193
        );
        assert_eq!(
            city.furniture
                .iter()
                .filter(|f| f.kind == FurnitureKind::Stop)
                .count(),
            1281
        );
        assert_eq!(
            city.furniture
                .iter()
                .filter(|f| f.kind == FurnitureKind::Speed)
                .count(),
            111
        );
        for f in &city.furniture {
            assert!(
                !surface.contains([f.position[0], f.position[2]]),
                "furniture {} is on asphalt at {:?}",
                f._source_id,
                f.position
            );
            assert!(city.building_clearance([f.position[0], f.position[2]]) >= 0.0);
            assert!(f.position.iter().all(|v| v.is_finite()));
        }
        assert!(load(
            &crate::city_assets::read("furniture.txt"),
            "stale",
            &crate::city_assets::read("streets.txt")
        )
        .is_err());
    }
}
