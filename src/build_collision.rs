use crate::error::Result;
use crate::source_location::SourceLocation;
use crate::street_surface::binary::write_binary_street_surface;
use crate::street_surface::StreetSurface;
use std::path::Path;

pub fn build_collisions(
    path_streets_bin: &Path,
    city: &SourceLocation<f32>,
    streets_txt_source: String,
) -> Result<()> {
    let (surfaces, paths) = StreetSurface::load(&streets_txt_source, city.road_lines.len())?;
    let directory = path_streets_bin
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let temporary = tempfile::NamedTempFile::new_in(directory)?;

    write_binary_street_surface(temporary.path(), &surfaces, &paths)?;
    temporary.persist(path_streets_bin)?;
    Ok(())
}
