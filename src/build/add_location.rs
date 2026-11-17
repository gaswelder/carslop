use crate::build;
use crate::storage;
use crate::util::error::invalid;
use crate::util::error::Result;
use std::{fs, path::PathBuf};

/// Download, infer, compile, then write to a timestamped directory.
pub fn add_location(area: build::osm::ImportArea) -> Result<()> {
    let download_dir = build::download_location::download_map_data(area)?;
    build::infer_trees::add_trees(&download_dir)?;
    let location = build::compile_location::compile_location(&download_dir)?;

    // Write the location to disk.
    let timestamp = jiff::Timestamp::now().strftime("%Y%m%d-%H%M%S");
    let location_dir = PathBuf::from(format!("location-{timestamp}"));
    fs::create_dir(&location_dir)?;
    println!("Writing location to {}...", location_dir.display());
    storage::location::location_write(&location, &location_dir).map_err(invalid)?;
    println!("Created {}", location_dir.display());
    Ok(())
}
