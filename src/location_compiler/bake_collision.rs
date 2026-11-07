//! Compile street-surface collision data from prepared location assets.
use super::Result;
use crate::street_surface::{SourceChecksums, StreetSurface};
use sha2::{Digest, Sha256};
use std::path::Path;

pub(crate) fn bake_collision(directory: &Path) -> Result<()> {
    let started = std::time::Instant::now();
    let source = std::fs::read_to_string(directory.join("city.txt"))?;
    let streets = std::fs::read_to_string(directory.join("streets.txt"))?;
    let city = crate::city::City::parse(&source)?;
    let checksums = SourceChecksums::new(&source, &streets);
    let (surfaces, paths) = StreetSurface::load(&streets, &source, city.streets.len())?;
    let fingerprint: String = Sha256::digest(streets.as_bytes())
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    if fingerprint != super::sha(&directory.join("streets.txt"))? {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "street source changed during collision bake",
        )
        .into());
    }
    let temporary = tempfile::NamedTempFile::new_in(directory)?;
    crate::street_surface::write_binary(
        temporary.path(),
        &surfaces,
        &paths,
        checksums,
        &fingerprint,
    )?;
    let bytes = temporary.as_file().metadata()?.len();
    temporary.persist(directory.join("streets.bin"))?;
    println!(
        "Baked collision cache: {} triangles, {} curbs, {:.1} MiB; {:.2}s",
        surfaces.triangles.len(),
        surfaces.curbs.len(),
        bytes as f64 / 1048576.0,
        started.elapsed().as_secs_f64()
    );
    Ok(())
}
