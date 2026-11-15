//! Mapped roadside controls. Pole locations and timings are renderer estimates.

pub use crate::model_world::{Furniture, FurnitureKind};
#[cfg(test)]
use crate::storage_city_assets;
pub const CYCLE_SECONDS: f32 = 50.0;
#[cfg(test)]
fn load(text: &str) -> Result<Vec<Furniture>, String> {
    let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
    std::fs::write(folder.path().join("furniture.txt"), text).map_err(|e| e.to_string())?;
    storage_city_assets::get_furniture(&storage_city_assets::new(folder.path().to_path_buf()))
}

#[cfg(test)]
mod tests {
    use crate::model_world;
    use crate::storage_city_assets;
    use crate::world_buildings;
    use crate::world_street_surface;

    use super::*;
    use crate::test_support;
    use crate::util_geometry;

    #[test]
    fn mapped_furniture_is_typed_finite_and_outside_the_driving_surface() {
        let city_track = test_support::city_world();
        let city = &city_track;
        assert!(!city.furniture.is_empty());
        for f in &city.furniture {
            assert!(
                !world_street_surface::contains_road(
                    city,
                    util_geometry::XZ {
                        x: f.coords.x,
                        z: f.coords.z
                    }
                ),
                "furniture {} is on asphalt at {:?}",
                f._source_id,
                f.coords
            );
            assert!(
                world_buildings::building_clearance(
                    city,
                    util_geometry::XZ {
                        x: f.coords.x,
                        z: f.coords.z
                    }
                ) >= 0.0
            );
            assert!(util_geometry::iter(&(f.coords)).all(|v| v.is_finite()));
        }
        let assets = model_world::assets(city).unwrap();
        let furniture =
            std::fs::read_to_string(storage_city_assets::directory(assets).join("furniture.txt"))
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
