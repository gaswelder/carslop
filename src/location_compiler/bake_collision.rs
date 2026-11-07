//! Compile street-surface collision data from prepared location assets.
use super::Result;
use crate::street_surface::{SourceChecksums, StreetSurface};
use sha2::{Digest, Sha256};
use std::path::Path;

pub(crate) fn bake_collision(
    city_path: &Path,
    streets_path: &Path,
    output_path: &Path,
) -> Result<()> {
    let started = std::time::Instant::now();
    let source = std::fs::read_to_string(city_path)?;
    let streets = std::fs::read_to_string(streets_path)?;
    let city = crate::city::City::parse(&source)?;
    let checksums = SourceChecksums::new(&source, &streets);
    let (surfaces, paths) = StreetSurface::load(&streets, &source, city.streets.len())?;
    let fingerprint: String = Sha256::digest(streets.as_bytes())
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    if fingerprint != super::sha(streets_path)? {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "street source changed during collision bake",
        )
        .into());
    }
    let directory = output_path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let temporary = tempfile::NamedTempFile::new_in(directory)?;
    crate::street_surface::write_binary(
        temporary.path(),
        &surfaces,
        &paths,
        checksums,
        &fingerprint,
    )?;
    let bytes = temporary.as_file().metadata()?.len();
    temporary.persist(output_path)?;
    println!(
        "Baked collision cache: {} triangles, {} curbs, {:.1} MiB; {:.2}s",
        surfaces.triangles.len(),
        surfaces.curbs.len(),
        bytes as f64 / 1048576.0,
        started.elapsed().as_secs_f64()
    );
    Ok(())
}
