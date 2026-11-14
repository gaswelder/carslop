use crate::error::{invalid, Result};
use std::{
    collections::hash_map::DefaultHasher,
    fs,
    hash::{Hash, Hasher},
    io,
    path::{Path, PathBuf},
};

pub struct Downloader {
    agent: ureq::Agent,
    cache_dir: PathBuf,
}

impl Downloader {
    pub fn new(agent: ureq::Agent, cache_dir: &Path) -> Self {
        Self {
            agent,
            cache_dir: cache_dir.to_path_buf(),
        }
    }

    pub fn copy_download(&self, url: &str, path: &Path) -> Result<()> {
        let cached = self.download(url)?;
        if fs::metadata(&cached)?.len() == 0 {
            fs::remove_file(cached)?;
            return Err(invalid(format!("empty downloaded file: {url}")));
        }
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::copy(cached, path)?;
        Ok(())
    }

    pub fn invalidate(&self, url: &str) -> Result<()> {
        match fs::remove_file(self.cache_path(url)) {
            Ok(()) => Ok(()),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(error.into()),
        }
    }

    fn cache_path(&self, url: &str) -> PathBuf {
        let mut hash = DefaultHasher::new();
        url.hash(&mut hash);
        self.cache_dir.join(format!("{:016x}", hash.finish()))
    }

    fn download(&self, url: &str) -> Result<PathBuf> {
        let path = self.cache_path(url);
        if path.is_file() {
            println!("from cache {url}");
            return Ok(path);
        }
        println!("downloading {url}");
        fs::create_dir_all(&self.cache_dir)?;
        let mut file = tempfile::NamedTempFile::new_in(&self.cache_dir)?;
        let mut response = self
            .agent
            .get(url)
            .header("User-Agent", "carslop-location-import/0.1")
            .call()?;
        let expected = response
            .headers()
            .get("content-length")
            .and_then(|value| value.to_str().ok())
            .and_then(|value| value.parse::<u64>().ok());
        let count = io::copy(&mut response.body_mut().as_reader(), &mut file)?;
        if expected.is_some_and(|length| length != count) {
            return Err(invalid(format!("incomplete download: {url}")));
        }
        io::Write::flush(&mut file)?;
        file.as_file().sync_all()?;
        match file.persist_noclobber(&path) {
            Ok(_) => {}
            Err(error) if error.error.kind() == io::ErrorKind::AlreadyExists => {}
            Err(error) => return Err(error.error.into()),
        }
        Ok(path)
    }
}
