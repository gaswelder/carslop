use crate::{
    add_location_compile, add_location_download, add_location_trees,
    error::{invalid, Result},
    osm,
};
use std::{fs, path::PathBuf};

/// Download, infer, compile, then write to a timestamped directory.
pub fn add_location(area: osm::ImportArea) -> Result<()> {
    let download_dir = add_location_download::download_map_data(area)?;
    add_location_trees::add_trees(&download_dir)?;
    let location = add_location_compile::compile_location(&download_dir)?;

    // Write the location to disk.
    let timestamp = jiff::Timestamp::now().strftime("%Y%m%d-%H%M%S");
    let location_dir = PathBuf::from(format!("location-{timestamp}"));
    fs::create_dir(&location_dir)?;
    println!("Writing location to {}...", location_dir.display());
    location.write(&location_dir).map_err(invalid)?;
    println!("Created {}", location_dir.display());
    Ok(())
}
