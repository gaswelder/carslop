//! Compatibility reader for labels in older location asset folders.

use crate::build;
use crate::model;

/// Compatibility for old asset folders; build inputs never enter stored Location.
pub fn read_legacy_labels(directory: &std::path::Path) -> Result<model::location::Labels, String> {
    let labels = match std::fs::File::open(directory.join("source.osm")) {
        Ok(file) => match build::osm_labels::read_osm_labels(std::io::BufReader::new(file)) {
            Ok(labels) => labels,
            Err(e) => {
                log::warn!("Cannot load legacy labels: {e}");
                return Ok(model::location::Labels::default());
            }
        },
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok(model::location::Labels::default())
        }
        Err(e) => return Err(format!("source.osm labels: {e}")),
    };
    let mut streets: Vec<_> = labels.streets.into_iter().collect();
    let mut buildings: Vec<_> = labels.buildings.into_iter().collect();
    streets.sort();
    buildings.sort();
    Ok(model::location::Labels {
        streets,
        buildings,
        storefronts: labels.storefronts,
    })
}
