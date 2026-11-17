//! Shared street fixtures exercise the builder's file reader and storage writer.

use crate::model;
use crate::storage;
use crate::util;
use crate::world;
use std::fs;
use std::path::Path;

/// Compile fixture text to an arbitrary cache path for codec and world tests.
pub fn compile_collisions(
    output: &Path,
    city: &model::source_location::SourceLocation<f32>,
    streets: String,
) -> util::error::Result<()> {
    let staging = tempfile::tempdir()?;
    fs::write(staging.path().join("streets.txt"), streets)?;
    let geometry = super::read(staging.path(), city.road_lines.len())?;
    storage::streets::write_compiled(staging.path(), &geometry)?;
    std::fs::copy(staging.path().join("streets.bin"), output)?;
    Ok(())
}

pub struct PreparedStreets {
    pub surfaces: world::street_surface::StreetSurface,
    pub paths: world::street_surface::RoadPaths,
}

pub fn load_streets(text: &str, roads: usize) -> Result<PreparedStreets, String> {
    let staging = tempfile::tempdir().map_err(|e| e.to_string())?;
    fs::write(staging.path().join("streets.txt"), text).map_err(|e| e.to_string())?;
    let geometry = super::read(staging.path(), roads).map_err(|e| e.to_string())?;
    let (surfaces, paths) = world::street_surface::from_geometry(
        geometry,
        world::street_surface::DEFAULT_PAVEMENT_HEIGHT,
    )?;
    Ok(PreparedStreets { surfaces, paths })
}
