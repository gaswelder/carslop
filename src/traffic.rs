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
