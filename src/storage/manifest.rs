//! Metadata published with a playable location.

use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::LatLon;
use crate::world;
use serde::{Deserialize, Serialize};
use std::{fs, path::Path};

#[derive(Debug, Serialize, Deserialize)]
pub struct Manifest {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<LatLon>,
    pub bounds_metres: BoundingBoxXZ<f32>,
    #[serde(default = "default_pavement_height")]
    pub pavement_height_metres: f32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub aerial_attribution: Option<String>,
}

fn default_pavement_height() -> f32 {
    world::street_surface::DEFAULT_PAVEMENT_HEIGHT
}

fn validate(manifest: &Manifest) -> std::result::Result<(), String> {
    if let Some(origin) = &manifest.origin {
        if !(-90.0..=90.0).contains(&origin.lat) {
            return Err("invalid origin latitude".into());
        }
        if !(-180.0..=180.0).contains(&origin.lon) {
            return Err("invalid origin longitude".into());
        }
    }
    let bounds = manifest.bounds_metres;
    if ![bounds.min.x, bounds.min.z, bounds.max.x, bounds.max.z]
        .into_iter()
        .all(f32::is_finite)
        || bounds.min.x >= bounds.max.x
        || bounds.min.z >= bounds.max.z
    {
        return Err("invalid bounds_metres".into());
    }
    if !(0.08..=0.25).contains(&manifest.pavement_height_metres) {
        return Err("pavement_height_metres must be between 0.08 and 0.25".into());
    }
    Ok(())
}

pub fn read(directory: &Path) -> std::result::Result<Manifest, String> {
    let read = || -> std::result::Result<Manifest, String> {
        let bytes = fs::read(directory.join("manifest.json")).map_err(|e| e.to_string())?;
        let manifest = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        validate(&manifest)?;
        Ok(manifest)
    };
    read().map_err(|e| format!("manifest.json: {e}"))
}

pub fn write(directory: &Path, manifest: &Manifest) -> Result<()> {
    validate(manifest).map_err(invalid)?;
    fs::write(
        directory.join("manifest.json"),
        serde_json::to_vec_pretty(manifest)?,
    )?;
    Ok(())
}
