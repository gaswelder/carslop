//! Compile street-surface collision data from prepared location assets.
use crate::city::City;
use crate::location::Result;
use crate::street_surface::{SourceChecksums, StreetSurface};
use sha2::{Digest, Sha256};
use std::path::Path;

pub fn build_collisions(
    path_streets_txt: &Path,
    path_streets_bin: &Path,
    city: &City,
    city_checksum: u32,
    streets_txt_source: String,
    streets_checksum: u32,
) -> Result<()> {
    let (surfaces, paths) = StreetSurface::load(&streets_txt_source, city.streets.len())?;
    let fingerprint: String = Sha256::digest(streets_txt_source.as_bytes())
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    if fingerprint != crate::location::sha(path_streets_txt)? {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "street source changed during collision bake",
        )
        .into());
    }
    let directory = path_streets_bin
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let temporary = tempfile::NamedTempFile::new_in(directory)?;

    crate::street_surface::write_binary(
        temporary.path(),
        &surfaces,
        &paths,
        SourceChecksums {
            city: city_checksum,
            streets: streets_checksum,
        },
        &fingerprint,
    )?;
    temporary.persist(path_streets_bin)?;
    Ok(())
}
