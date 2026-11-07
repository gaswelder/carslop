//! Download source files into the compiler cache.
use super::{invalid, Result};
use std::{fs, io::Write, path::Path};

// Cache entries are published only after a complete response has been flushed.
pub(super) fn download(agent: &ureq::Agent, url: &str, path: &Path) -> Result<()> {
    if path.is_file() && fs::metadata(path)?.len() > 0 {
        return Ok(());
    }
    println!("Downloading {url}");
    let parent = path
        .parent()
        .ok_or_else(|| invalid("download has no parent"))?;
    fs::create_dir_all(parent)?;
    let mut file = tempfile::NamedTempFile::new_in(parent)?;
    let mut response = agent
        .get(url)
        .header("User-Agent", "carslop-location-import/0.1")
        .call()?;
    let expected = response
        .headers()
        .get("content-length")
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.parse::<u64>().ok());
    let mut reader = response.body_mut().as_reader();
    let count = std::io::copy(&mut reader, &mut file)?;
    if count == 0 || expected.is_some_and(|n| n != count) {
        return Err(invalid(format!("incomplete download: {url}")));
    }
    file.flush()?;
    file.as_file().sync_all()?;
    file.persist_noclobber(path).map_err(|e| e.error)?;
    Ok(())
}
