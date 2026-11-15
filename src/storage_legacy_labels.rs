//! Compatibility reader for labels in older location asset folders.

use crate::build_osm_labels;
use crate::model_location;

/// Compatibility for old asset folders; build inputs never enter stored Location.
pub fn read_legacy_labels(directory: &std::path::Path) -> Result<model_location::Labels, String> {
    let labels = match std::fs::File::open(directory.join("source.osm")) {
        Ok(file) => match build_osm_labels::read_osm_labels(std::io::BufReader::new(file)) {
            Ok(labels) => labels,
            Err(e) => {
                log::warn!("Cannot load legacy labels: {e}");
                return Ok(model_location::Labels::default());
            }
        },
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok(model_location::Labels::default())
        }
        Err(e) => return Err(format!("source.osm labels: {e}")),
    };
    let mut streets: Vec<_> = labels.streets.into_iter().collect();
    let mut buildings: Vec<_> = labels.buildings.into_iter().collect();
    streets.sort();
    buildings.sort();
    Ok(model_location::Labels {
        streets,
        buildings,
        storefronts: labels.storefronts,
    })
}
