//! Mapped roadside controls. Pole locations and timings are renderer estimates.

pub use crate::model::world::Furniture;
pub use crate::model::world::FurnitureKind;
#[cfg(test)]
use crate::storage;
pub const CYCLE_SECONDS: f32 = 50.0;
#[cfg(test)]
fn load(text: &str) -> Result<Vec<Furniture>, String> {
    let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
    std::fs::write(folder.path().join("furniture.txt"), text).map_err(|e| e.to_string())?;
    storage::city_assets::get_furniture(&storage::city_assets::new(folder.path().to_path_buf()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model;
    use crate::storage;
    use crate::test;
    use crate::util;
    use crate::world;

    #[test]
    fn mapped_furniture_is_typed_finite_and_outside_the_driving_surface() {
        let city_track = test::support::city_world();
        let city = &city_track;
        assert!(!city.furniture.is_empty());
        for f in &city.furniture {
            assert!(
                !world::street_surface::contains_road(
                    city,
                    util::geometry::XZ {
                        x: f.coords.x,
                        z: f.coords.z
                    }
                ),
                "furniture {} is on asphalt at {:?}",
                f._source_id,
                f.coords
            );
            assert!(
                world::buildings::building_clearance(
                    city,
                    util::geometry::XZ {
                        x: f.coords.x,
                        z: f.coords.z
                    }
                ) >= 0.0
            );
            assert!(util::geometry::iter(&(f.coords)).all(|v| v.is_finite()));
        }
        let assets = model::world::assets(city).unwrap();
        let furniture =
            std::fs::read_to_string(storage::city_assets::directory(assets).join("furniture.txt"))
                .unwrap();
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
