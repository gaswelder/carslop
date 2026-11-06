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
}
#[cfg(test)]
pub fn load(text: &str, city: &str, streets: &str) -> Result<Vec<Furniture>, String> {
    load_with_checksums(
        text,
        crate::street_surface::SourceChecksums::new(city, streets),
    )
}
#[cfg(test)]
fn load_with_checksums(
    text: &str,
    checksums: crate::street_surface::SourceChecksums,
) -> Result<Vec<Furniture>, String> {
    let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
    std::fs::write(folder.path().join("furniture.txt"), text).map_err(|e| e.to_string())?;
    crate::city_assets::CityAsset::new(folder.path().to_path_buf()).get_furniture(checksums)
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
    fn eight_column_records_preserve_signal_and_sign_fields() {
        let text = format!(
            "FURNITURE 3 {:08x} {:08x}\n123 signal 12.5 -40 1 0 0 33\n456 speed_limit -10 20 0 -1 50 0\n789 stop 0 0 1 0 0 0\n987 yield 0 0 1 0 0 0\n",
            crate::street_surface::crc32("city"),
            crate::street_surface::crc32("streets"),
        );
        let controls = load(&text, "city", "streets").unwrap();
        assert_eq!(controls.len(), 4);
        assert_eq!(controls[0]._source_id, 123);
        assert_eq!(controls[0].kind, FurnitureKind::Signal);
        assert_eq!(controls[0].position, [12.5, 0.0, -40.0]);
        assert_eq!(controls[0].travel, [1.0, 0.0]);
        assert_eq!(controls[0].phase, 33);
        assert_eq!(controls[1]._source_id, 456);
        assert_eq!(controls[1].kind, FurnitureKind::Speed);
        assert_eq!(controls[1].position, [-10.0, 0.0, 20.0]);
        assert_eq!(controls[1].travel, [0.0, -1.0]);
        assert_eq!(controls[1].value, 50);
        assert_eq!(controls[2].kind, FurnitureKind::Stop);
        assert_eq!(controls[3].kind, FurnitureKind::Yield);
    }

    #[test]
    fn obsolete_furniture_headers_and_columns_are_rejected() {
        let header = format!(
            "FURNITURE 3 {:08x} {:08x}\n",
            crate::street_surface::crc32("city"),
            crate::street_surface::crc32("streets"),
        );
        for record in [
            "f 123 signal 0 0 1 0 0 0",
            "123 signal 0 0 1 0 0 0 1 1",
            "123 0 0 0 1 0 0 0",
            "123 unknown 0 0 1 0 0 0",
        ] {
            assert!(load(&(header.clone() + record), "city", "streets").is_err());
        }
        for old_version in ["FURNITURE 1", "FURNITURE 2"] {
            assert!(load(
                &header.replace("FURNITURE 3", old_version),
                "city",
                "streets"
            )
            .is_err());
        }
    }

    #[test]
    fn shared_checksums_reject_stale_furniture_sources() {
        use crate::street_surface::SourceChecksums;
        let checksums = SourceChecksums::new("city", "streets");
        let text = format!(
            "FURNITURE 3 {:08x} {:08x}\n",
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
        let assets = crate::city_assets::CITY_ASSET.read().unwrap().clone();
        let furniture = std::fs::read_to_string(assets.directory().join("furniture.txt")).unwrap();
        let streets = std::fs::read_to_string(assets.directory().join("streets.txt")).unwrap();
        assert!(load(&furniture, "stale", &streets).is_err());
    }
}
