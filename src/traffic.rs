//! Mapped roadside controls. Pole locations and timings are renderer estimates.
pub use crate::world::{Furniture, FurnitureKind};
pub const CYCLE_SECONDS: f32 = 50.0;
#[cfg(test)]
fn load(text: &str) -> Result<Vec<Furniture>, String> {
    let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
    std::fs::write(folder.path().join("furniture.txt"), text).map_err(|e| e.to_string())?;
    crate::city_assets::CityAsset::new(folder.path().to_path_buf()).get_furniture()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn furniture_accepts_plain_and_legacy_headers() {
        for header in ["FURNITURE 3", "FURNITURE 3 deadbeef cafebabe"] {
            assert!(load(&format!("{header}\n")).unwrap().is_empty());
        }
    }

    #[test]
    fn eight_column_records_preserve_signal_and_sign_fields() {
        let text = format!(
            "FURNITURE 3 {:08x} {:08x}\n123 signal 12.5 -40 1 0 0 33\n456 speed_limit -10 20 0 -1 50 0\n789 stop 0 0 1 0 0 0\n987 yield 0 0 1 0 0 0\n",
            0,
            0,
        );
        let controls = load(&text).unwrap();
        assert_eq!(controls.len(), 4);
        assert_eq!(controls[0]._source_id, 123);
        assert_eq!(controls[0].kind, FurnitureKind::Signal);
        assert_eq!(
            controls[0].coords,
            crate::geometry::XYZ {
                x: 12.5,
                y: 0.0,
                z: -40.0
            }
        );
        assert_eq!(controls[0].travel, crate::geometry::XZ { x: 1.0, z: 0.0 });
        assert_eq!(controls[0].phase, 33);
        assert_eq!(controls[1]._source_id, 456);
        assert_eq!(controls[1].kind, FurnitureKind::Speed);
        assert_eq!(
            controls[1].coords,
            crate::geometry::XYZ {
                x: -10.0,
                y: 0.0,
                z: 20.0
            }
        );
        assert_eq!(controls[1].travel, crate::geometry::XZ { x: 0.0, z: -1.0 });
        assert_eq!(controls[1].value, 50);
        assert_eq!(controls[2].kind, FurnitureKind::Stop);
        assert_eq!(controls[3].kind, FurnitureKind::Yield);
    }

    #[test]
    fn obsolete_furniture_headers_and_columns_are_rejected() {
        let header = format!("FURNITURE 3 {:08x} {:08x}\n", 0, 0,);
        for record in [
            "f 123 signal 0 0 1 0 0 0",
            "123 signal 0 0 1 0 0 0 1 1",
            "123 0 0 0 1 0 0 0",
            "123 unknown 0 0 1 0 0 0",
        ] {
            assert!(load(&(header.clone() + record)).is_err());
        }
        for old_version in ["FURNITURE 1", "FURNITURE 2"] {
            assert!(load(&header.replace("FURNITURE 3", old_version)).is_err());
        }
    }

    #[test]
    fn mapped_furniture_is_typed_finite_and_outside_the_driving_surface() {
        let city_track = crate::test_support::nicosia_world();
        let city = &city_track;
        assert!(!city.furniture.is_empty());
        for f in &city.furniture {
            assert!(
                !city.contains_road(crate::geometry::XZ {
                    x: f.coords.x,
                    z: f.coords.z
                }),
                "furniture {} is on asphalt at {:?}",
                f._source_id,
                f.coords
            );
            assert!(
                city.building_clearance(crate::geometry::XZ {
                    x: f.coords.x,
                    z: f.coords.z
                }) >= 0.0
            );
            assert!(f.coords.iter().all(|v| v.is_finite()));
        }
        let assets = city.assets().unwrap();
        let furniture = std::fs::read_to_string(assets.directory().join("furniture.txt")).unwrap();
        let loaded = load(&furniture).unwrap();
        assert_eq!(loaded.len(), city.furniture.len());
        for (raw, placed) in loaded.iter().zip(&city.furniture) {
            assert_eq!(raw._source_id, placed._source_id);
            assert_eq!(raw.kind, placed.kind);
            assert_eq!(raw.travel, placed.travel);
            assert_eq!(raw.value, placed.value);
            assert_eq!(raw.phase, placed.phase);
        }
    }
}
