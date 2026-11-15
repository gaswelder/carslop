use crate::model_location;
use crate::{
    build_compile_location, build_download_location, build_infer_trees, build_osm,
    util_error::{invalid, Result},
};
use std::{fs, path::PathBuf};

/// Download, infer, compile, then write to a timestamped directory.
pub fn add_location(area: build_osm::ImportArea) -> Result<()> {
    let download_dir = build_download_location::download_map_data(area)?;
    build_infer_trees::add_trees(&download_dir)?;
    let location = build_compile_location::compile_location(&download_dir)?;

    // Write the location to disk.
    let timestamp = jiff::Timestamp::now().strftime("%Y%m%d-%H%M%S");
    let location_dir = PathBuf::from(format!("location-{timestamp}"));
    fs::create_dir(&location_dir)?;
    println!("Writing location to {}...", location_dir.display());
    model_location::location_write(&location, &location_dir).map_err(invalid)?;
    println!("Created {}", location_dir.display());
    Ok(())
}
