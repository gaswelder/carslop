//! Download source files into the compiler cache.
use crate::error::{invalid, Result};
use std::{
    fs,
    io::{Read, Write},
    path::Path,
    time::Duration,
};

pub struct DownloadOptions {
    pub attempts: usize,
    pub(super) max_bytes: Option<u64>,
    pub(super) validate: fn(&Path) -> Result<bool>,
    pub(super) replace_invalid: bool,
}

impl Default for DownloadOptions {
    fn default() -> Self {
        Self {
            attempts: 1,
            max_bytes: None,
            validate: |path| Ok(fs::metadata(path)?.len() > 0),
            replace_invalid: false,
        }
    }
}

// Cache entries are published only after a complete, validated response has been flushed.
pub fn download(
    agent: &ureq::Agent,
    url: &str,
    path: &Path,
    options: &DownloadOptions,
) -> Result<()> {
    if options.attempts == 0 {
        return Err(invalid("download requires at least one attempt"));
    }
    if path.is_file()
        && options
            .max_bytes
            .is_none_or(|limit| fs::metadata(path).is_ok_and(|m| m.len() <= limit))
        && (options.validate)(path)?
    {
        return Ok(());
    }
    println!("Downloading {url}");
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    for attempt in 0..options.attempts {
        let result = (|| -> Result<()> {
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
            let count = if let Some(limit) = options.max_bytes {
                std::io::copy(
                    &mut reader.by_ref().take(limit.saturating_add(1)),
                    &mut file,
                )?
            } else {
                std::io::copy(&mut reader, &mut file)?
            };
            if options.max_bytes.is_some_and(|limit| count > limit) {
                return Err(invalid(format!("download exceeds size limit: {url}")));
            }
            if count == 0 || expected.is_some_and(|n| n != count) {
                return Err(invalid(format!("incomplete download: {url}")));
            }
            file.flush()?;
            if !(options.validate)(file.path())? {
                return Err(invalid(format!("invalid downloaded file: {url}")));
            }
            file.as_file().sync_all()?;
            if options.replace_invalid {
                file.persist(path).map_err(|e| e.error)?;
            } else {
                file.persist_noclobber(path).map_err(|e| e.error)?;
            }
            Ok(())
        })();
        match result {
            Ok(()) => return Ok(()),
            Err(error) if attempt + 1 == options.attempts => return Err(error),
            Err(_) => std::thread::sleep(Duration::from_secs(1 << attempt.min(6))),
        }
    }
    unreachable!("download attempts are nonzero")
}
