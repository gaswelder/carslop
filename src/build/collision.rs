use crate::model::source_location::SourceLocation;
use crate::storage::street_surface_binary::write_binary_street_surface;
use crate::util::error::Result;
use crate::world;
use std::path::Path;

pub fn build_collisions(
    path_streets_bin: &Path,
    city: &SourceLocation<f32>,
    streets_txt_source: String,
) -> Result<()> {
    let (surfaces, paths) =
        world::street_surface::load(&streets_txt_source, city.road_lines.len())?;
    let directory = path_streets_bin
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let temporary = tempfile::NamedTempFile::new_in(directory)?;

    write_binary_street_surface(temporary.path(), &surfaces, &paths)?;
    temporary.persist(path_streets_bin)?;
    Ok(())
}
